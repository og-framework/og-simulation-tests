// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

#include "glm/geometric.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulationJolt/JoltBodyDefaults.h"
#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsFactory.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using namespace joltTestRig;

namespace
{
    constexpr uint32_t kRoundTripSamples = 1000;
    constexpr uint32_t kRoundTripSeed = 25;
    constexpr float kPositionRangeMetres = 50.f;
    constexpr float kLinearVelocityRangeMetresPerSecond = 20.f;
    constexpr float kAngularVelocityRangeRadiansPerSecond = 10.f;
    constexpr float kCentimetreMetres = 0.01f;

    static_assert(3.f * kLinearVelocityRangeMetresPerSecond * kLinearVelocityRangeMetresPerSecond
            < joltBodyDefaults::kMaxLinearVelocityMetresPerSecond * joltBodyDefaults::kMaxLinearVelocityMetresPerSecond,
        "A seeded linear velocity must stay below the shipped clamp, or Jolt's SetLinearVelocityClamped rewrites it");
    static_assert(3.f * kAngularVelocityRangeRadiansPerSecond * kAngularVelocityRangeRadiansPerSecond
            < joltBodyDefaults::kMaxAngularVelocityRadiansPerSecond * joltBodyDefaults::kMaxAngularVelocityRadiansPerSecond,
        "A seeded angular velocity must stay below the shipped clamp, or Jolt's SetAngularVelocityClamped rewrites it");

    class RoundTripRandom
    {
    public:
        explicit RoundTripRandom(uint32_t seed) : m_engine(seed) {}

        float uniform(float low, float high) { return low + (high - low) * float(double(m_engine()) / 4294967296.0); }

        glm::vec3 vector(float magnitude) { return glm::vec3(uniform(-magnitude, magnitude), uniform(-magnitude, magnitude), uniform(-magnitude, magnitude)); }

        glm::quat unitQuaternion()
        {
            for (;;)
            {
                const glm::vec4 candidate(uniform(-1.f, 1.f), uniform(-1.f, 1.f), uniform(-1.f, 1.f), uniform(-1.f, 1.f));
                const float lengthSquared = glm::dot(candidate, candidate);
                if (lengthSquared > 0.01f && lengthSquared <= 1.f)
                    return glm::normalize(glm::quat(candidate.w, candidate.x, candidate.y, candidate.z));
            }
        }

    private:
        std::mt19937 m_engine;
    };

    enum class Outcome : uint8_t
    {
        Exact,
        Zeroed,
        Changed
    };

    template <typename T>
    Outcome outcomeOf(const T& written, const T& captured)
    {
        if (std::memcmp(&written, &captured, sizeof(T)) == 0)
            return Outcome::Exact;
        const std::array<std::byte, sizeof(T)> zero{};
        return std::memcmp(&captured, zero.data(), sizeof(T)) == 0 ? Outcome::Zeroed : Outcome::Changed;
    }

    struct FieldTally
    {
        uint32_t exact = 0;
        uint32_t zeroed = 0;
        uint32_t changed = 0;

        void add(Outcome outcome)
        {
            switch (outcome)
            {
            case Outcome::Exact: ++exact; break;
            case Outcome::Zeroed: ++zeroed; break;
            case Outcome::Changed: ++changed; break;
            }
        }
    };

    struct TemplateTally
    {
        FieldTally position;
        FieldTally rotation;
        FieldTally linearVelocity;
        FieldTally angularVelocity;
    };

    FieldTally allExact()
    {
        return FieldTally{ kRoundTripSamples, 0u, 0u };
    }

    TemplateTally expectedTallyOf(const SlotBodyTemplate& slotTemplate)
    {
        TemplateTally expected{ allExact(), allExact(), allExact(), allExact() };
        if (slotTemplate.descriptor.body.lockRotation)
            expected.angularVelocity = FieldTally{ 0u, kRoundTripSamples, 0u };
        return expected;
    }

    struct ShippedSlot
    {
        explicit ShippedSlot(JoltRuntime& runtime)
            : world(runtime, configOf(), &logJoltProblems)
            , bodies(world)
        {
            JoltPhysicsFactory factory(bodies, brawlerSlotTemplate(), 0u, 1u, JoltPhysicsFactoryOptions{});
            for (const SlotBodyTemplate& slotTemplate : brawlerSlotTemplate())
                declarationBodies.push_back(factory.createPhysicalObject(slotTemplate.descriptor, "roundTrip").bodyId);
            world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 0u }));
        }

        static JoltWorldConfig configOf()
        {
            JoltWorldConfig config;
            config.simulatableSlots = 1;
            config.slotTemplate = brawlerSlotTemplate();
            config.staticLayers = { kWorldStaticKey };
            config.tempAllocatorBytes = 4u << 20;
            return config;
        }

        void offsetCentreOfMass(uint32_t templateIndex, float offsetMetres)
        {
            JPH::BodyInterface& bodyInterface = world.bodies();
            const JPH::BodyID body = world.slotBodyId(0u, templateIndex);
            bodyInterface.SetShape(body, new JPH::OffsetCenterOfMassShape(bodyInterface.GetShape(body), JPH::Vec3(offsetMetres, 0.f, 0.f)), false,
                JPH::EActivation::DontActivate);
        }

        JoltWorld world;
        JoltPhysicsBodyAdapter bodies;
        std::vector<BodyId> declarationBodies;
    };

    TemplateTally roundTrip(JoltPhysicsBodyAdapter& bodies, BodyId body)
    {
        RoundTripRandom random(kRoundTripSeed);
        TemplateTally tally;
        for (uint32_t sample = 0; sample < kRoundTripSamples; ++sample)
        {
            PhysicsBodyState written;
            written.position = random.vector(kPositionRangeMetres);
            written.rotation = random.unitQuaternion();
            written.linearVelocity = random.vector(kLinearVelocityRangeMetresPerSecond);
            written.angularVelocity = random.vector(kAngularVelocityRangeRadiansPerSecond);

            bodies.setBodyPose(body, written.position, written.rotation);
            bodies.setBodyLinearVelocity(body, written.linearVelocity);
            bodies.setBodyAngularVelocity(body, written.angularVelocity);
            const PhysicsBodyState captured = bodies.captureBodyState(body);

            tally.position.add(outcomeOf(written.position, captured.position));
            tally.rotation.add(outcomeOf(written.rotation, captured.rotation));
            tally.linearVelocity.add(outcomeOf(written.linearVelocity, captured.linearVelocity));
            tally.angularVelocity.add(outcomeOf(written.angularVelocity, captured.angularVelocity));
        }
        return tally;
    }

    void printTally(uint32_t templateIndex, bool lockRotation, const TemplateTally& tally)
    {
        std::printf("[RoundTrip] template=%u lockRotation=%d exact/zeroed/changed position=%u/%u/%u rotation=%u/%u/%u linearVelocity=%u/%u/%u "
                    "angularVelocity=%u/%u/%u\n",
            templateIndex, lockRotation ? 1 : 0, tally.position.exact, tally.position.zeroed, tally.position.changed, tally.rotation.exact,
            tally.rotation.zeroed, tally.rotation.changed, tally.linearVelocity.exact, tally.linearVelocity.zeroed, tally.linearVelocity.changed,
            tally.angularVelocity.exact, tally.angularVelocity.zeroed, tally.angularVelocity.changed);
    }

    void requireField(const char* field, const FieldTally& actual, const FieldTally& expected)
    {
        INFO(field);
        CHECK(actual.exact == expected.exact);
        CHECK(actual.zeroed == expected.zeroed);
        CHECK(actual.changed == expected.changed);
    }

    void requireTally(uint32_t templateIndex, const TemplateTally& actual, const TemplateTally& expected)
    {
        INFO("template " << templateIndex);
        requireField("position", actual.position, expected.position);
        requireField("rotation", actual.rotation, expected.rotation);
        requireField("linearVelocity", actual.linearVelocity, expected.linearVelocity);
        requireField("angularVelocity", actual.angularVelocity, expected.angularVelocity);
    }
} // namespace

TEST_CASE("JoltShippedTemplateRoundTrip.EveryShippedTemplateRoundTripsAThousandSeededStatesThroughThePoseWriter", "[Jolt]")
{
    RuntimeLease lease;
    ShippedSlot slot(lease.runtime);
    const std::vector<SlotBodyTemplate> shipped = brawlerSlotTemplate();
    REQUIRE(shipped.size() == kBodiesPerBrawler);
    REQUIRE(slot.declarationBodies.size() == shipped.size());

    uint32_t rotationLockedTemplates = 0;
    for (uint32_t templateIndex = 0; templateIndex < shipped.size(); ++templateIndex)
    {
        REQUIRE(slot.world.bodies().GetShape(slot.world.slotBodyId(0u, templateIndex))->GetCenterOfMass() == JPH::Vec3::sZero());
        const TemplateTally tally = roundTrip(slot.bodies, slot.declarationBodies[templateIndex]);
        printTally(templateIndex, shipped[templateIndex].descriptor.body.lockRotation, tally);
        requireTally(templateIndex, tally, expectedTallyOf(shipped[templateIndex]));
        if (shipped[templateIndex].descriptor.body.lockRotation)
            ++rotationLockedTemplates;
    }
    REQUIRE(rotationLockedTemplates == 1u);
}

TEST_CASE("JoltShippedTemplateRoundTrip.Poison.ACentimetreCentreOfMassOffsetBreaksTheRoundTrip", "[.][JoltRoundTripPoison]")
{
    RuntimeLease lease;
    ShippedSlot slot(lease.runtime);
    slot.offsetCentreOfMass(kTemplateRadial, kCentimetreMetres);
    const TemplateTally tally = roundTrip(slot.bodies, slot.declarationBodies[kTemplateRadial]);
    printTally(kTemplateRadial, false, tally);
    requireTally(kTemplateRadial, tally, expectedTallyOf(brawlerSlotTemplate()[kTemplateRadial]));
}

#endif // WITH_LOW_LEVEL_TESTS
