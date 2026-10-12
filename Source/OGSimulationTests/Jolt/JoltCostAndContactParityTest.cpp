// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltRollbackSuiteRig.h"

#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/PhysicsSettings.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <map>
#include <optional>
#include <utility>
#include <vector>

using namespace joltSuite;

namespace
{
    using Clock = std::chrono::steady_clock;

    double millisecondsSince(Clock::time_point start)
    {
        return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    }

    struct Distribution
    {
        double median = 0.0;
        double p95 = 0.0;
        double max = 0.0;
    };

    Distribution distributionOf(std::vector<double> samples)
    {
        std::sort(samples.begin(), samples.end());
        Distribution distribution;
        distribution.median = samples[samples.size() / 2u];
        distribution.p95 = samples[std::min(samples.size() - 1u, (samples.size() * 95u) / 100u)];
        distribution.max = samples.back();
        return distribution;
    }

    struct CostResult
    {
        size_t staticBodies = 0;
        uint32_t staticElements = 0;
        Distribution step;
        Distribution save;
        Distribution stepPlusSave;
        Distribution restore;
        Distribution replay20;
    };

    CostResult measureCost(JoltRuntime& runtime, uint32_t extraStatics, uint32_t maxShapesPerChunk, bool linearCast = false)
    {
        SceneConfig sceneConfig;
        sceneConfig.factoryOptions.linearCastCharacters = linearCast;
        sceneConfig.ringDepthTicks = 22;
        sceneConfig.extraStatics = extraStatics;
        sceneConfig.maxShapesPerChunk = maxShapesPerChunk;
        JoltScene scene(runtime, sceneConfig);
        for (uint32_t slot = 0; slot < kSlots; ++slot)
            (void)scene.bindProbe(slot, 101u + slot);
        scene.world.applyOccupancy(allOccupied());

        CostResult result;
        result.staticBodies = scene.staticBodies.size();
        result.staticElements = uint32_t(scene.description.shapes.size());

        SimTick tick = 0;
        auto advance = [&]() {
            ++tick;
            applyScriptedForces(scene.world, tick);
            scene.world.step(kDt);
            scene.world.saveTick(tick);
        };
        for (int index = 0; index < 120; ++index)
            advance();

        std::vector<double> step;
        std::vector<double> save;
        std::vector<double> stepPlusSave;
        for (int index = 0; index < 240; ++index)
        {
            ++tick;
            applyScriptedForces(scene.world, tick);
            const Clock::time_point start = Clock::now();
            scene.world.step(kDt);
            const double stepMs = millisecondsSince(start);
            const Clock::time_point saveStart = Clock::now();
            scene.world.saveTick(tick);
            save.push_back(millisecondsSince(saveStart));
            step.push_back(stepMs);
            stepPlusSave.push_back(millisecondsSince(start));
        }

        std::vector<double> restore;
        std::vector<double> replay;
        const std::vector<uint8_t> presentBytes = fullState(scene.world);
        for (int index = 0; index < 60; ++index)
        {
            const SimTick present = tick;
            const SimTick anchor = present - 20u;
            const Clock::time_point start = Clock::now();
            REQUIRE(scene.world.restoreTick(anchor));
            restore.push_back(millisecondsSince(start));
            for (SimTick replayTick = anchor + 1u; replayTick <= present; ++replayTick)
            {
                applyScriptedForces(scene.world, replayTick);
                scene.world.step(kDt);
                scene.world.saveTick(replayTick);
            }
            replay.push_back(millisecondsSince(start));
            REQUIRE(sameBytes(fullState(scene.world), presentBytes));
        }

        result.step = distributionOf(step);
        result.save = distributionOf(save);
        result.stepPlusSave = distributionOf(stepPlusSave);
        result.restore = distributionOf(restore);
        result.replay20 = distributionOf(replay);
        return result;
    }

    void print(const char* label, const Distribution& distribution)
    {
        std::printf(" %s(median=%.4f p95=%.4f max=%.4f)", label, distribution.median, distribution.p95, distribution.max);
    }

    struct SingleSlotScene
    {
        SingleSlotScene(JoltRuntime& runtime, const StaticWorldDescription& statics, glm::vec3 gravityCmPerS2,
            std::optional<float> minVelocityForRestitutionCmPerS, JoltPhysicsFactoryOptions options)
            : world(runtime, configOf(statics, gravityCmPerS2, minVelocityForRestitutionCmPerS), &logJoltProblems)
            , bodies(world)
        {
            JoltStaticWorldBuilder builder(world, [](const char*) {});
            builder.build(statics);
            JoltPhysicsFactory factory(bodies, brawlerSlotTemplate(), 0u, 1u, options);
            for (const SlotBodyTemplate& slotTemplate : brawlerSlotTemplate())
                declarationBodies.push_back(factory.createPhysicalObject(slotTemplate.descriptor, "probe").bodyId);
            capsule = factory.parentBodyId();
            world.applyOccupancy(occupancyOf({ 0u }));
            for (uint32_t index = 0; index + 1u < declarationBodies.size(); ++index)
                bodies.setBodyTransform(declarationBodies[index], glm::translate(glm::mat4(1.f), glm::vec3(-3000.f, 300.f * float(index), 2000.f)));
        }

        static JoltWorldConfig configOf(const StaticWorldDescription& statics, glm::vec3 gravityCmPerS2, std::optional<float> minVelocityForRestitution)
        {
            JoltWorldConfig config;
            config.simulatableSlots = 1;
            config.slotTemplate = brawlerSlotTemplate();
            config.staticLayers = staticLayerKeysOf(statics);
            config.gravityCmPerS2 = gravityCmPerS2;
            config.tempAllocatorBytes = 4u << 20;
            if (minVelocityForRestitution.has_value())
                config.minVelocityForRestitutionCmPerS = *minVelocityForRestitution;
            return config;
        }

        glm::vec3 capsulePosition() const { return bodies.captureBodyState(capsule).position; }
        glm::vec3 capsuleVelocity() const { return bodies.captureBodyState(capsule).linearVelocity; }

        JoltWorld world;
        JoltPhysicsBodyAdapter bodies;
        std::vector<BodyId> declarationBodies;
        BodyId capsule;
    };

    constexpr float kCapsuleRadiusCm = 42.f;
    constexpr float kCapsuleHalfHeightCm = 96.f;
    constexpr float kWallNearFaceCm = 300.f;

    enum class WallKind
    {
        Box1Cm,
        Box10Cm,
        TriangleMesh
    };

    const char* nameOf(WallKind kind)
    {
        switch (kind)
        {
        case WallKind::Box1Cm: return "box 1 cm";
        case WallKind::Box10Cm: return "box 10 cm";
        default: return "triangle mesh";
        }
    }

    float thicknessOf(WallKind kind)
    {
        return kind == WallKind::Box1Cm ? 1.f : kind == WallKind::Box10Cm ? 10.f : 0.f;
    }

    StaticWorldDescription wallOf(WallKind kind)
    {
        StaticWorldDescription description;
        if (kind == WallKind::TriangleMesh)
        {
            StaticShapeDescriptor wall = staticBox(glm::vec3(0.f), glm::vec3(1.f), 7u);
            StaticTriangleMesh mesh;
            mesh.vertices = { { kWallNearFaceCm, -400.f, -400.f }, { kWallNearFaceCm, 400.f, -400.f }, { kWallNearFaceCm, 400.f, 400.f },
                { kWallNearFaceCm, -400.f, 400.f } };
            mesh.indices = { 0, 2, 1, 0, 3, 2 };
            wall.shape = mesh;
            wall.localToWorld = glm::mat4(1.f);
            description.shapes.push_back(wall);
        }
        else
        {
            const float halfThickness = 0.5f * thicknessOf(kind);
            description.shapes.push_back(staticBox(glm::vec3(kWallNearFaceCm + halfThickness, 0.f, 0.f), glm::vec3(halfThickness, 400.f, 400.f), 7u));
        }
        return description;
    }

    struct TunnelResult
    {
        uint32_t trials = 0;
        uint32_t tunnelled = 0;
        uint32_t blocked = 0;
        float maxPenetrationCm = 0.f;
    };

    TunnelResult runTunnelling(JoltRuntime& runtime, WallKind kind, bool linearCast, float speedCmPerS)
    {
        TunnelResult result;
        const float farFace = kWallNearFaceCm + thicknessOf(kind);
        const float travelPerTick = speedCmPerS * kDt;
        for (uint32_t phase = 0; phase < 16u; ++phase)
        {
            SingleSlotScene scene(runtime, wallOf(kind), glm::vec3(0.f), std::nullopt, JoltPhysicsFactoryOptions{ linearCast });
            const float startX = kWallNearFaceCm - kCapsuleRadiusCm - 2.f * travelPerTick - travelPerTick * float(phase) / 16.f;
            scene.bodies.setBodyTransform(scene.capsule, glm::translate(glm::mat4(1.f), glm::vec3(startX, 0.f, 0.f)));
            float maxCentreX = startX;
            for (int tick = 0; tick < 20; ++tick)
            {
                scene.bodies.setBodyLinearVelocity(scene.capsule, glm::vec3(speedCmPerS, 0.f, 0.f));
                scene.world.step(kDt);
                maxCentreX = std::max(maxCentreX, scene.capsulePosition().x);
            }
            ++result.trials;
            const float finalX = scene.capsulePosition().x;
            if (finalX > farFace || maxCentreX > farFace)
                ++result.tunnelled;
            else
                ++result.blocked;
            result.maxPenetrationCm = std::max(result.maxPenetrationCm, maxCentreX + kCapsuleRadiusCm - kWallNearFaceCm);
        }
        return result;
    }

    float bounceSpeed(JoltRuntime& runtime, std::optional<float> minVelocityForRestitutionCmPerS, float approachSpeedCmPerS)
    {
        StaticWorldDescription floor;
        floor.shapes.push_back(staticBox(glm::vec3(0.f, 0.f, -50.f), glm::vec3(1000.f, 1000.f, 50.f), 1u));
        SingleSlotScene scene(runtime, floor, glm::vec3(0.f), minVelocityForRestitutionCmPerS, JoltPhysicsFactoryOptions{});
        scene.bodies.setBodyTransform(scene.capsule, glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, kCapsuleHalfHeightCm + 2.f)));
        scene.bodies.setBodyLinearVelocity(scene.capsule, glm::vec3(0.f, 0.f, -approachSpeedCmPerS));
        float maxUpward = 0.f;
        for (int tick = 0; tick < 40; ++tick)
        {
            scene.world.step(kDt);
            maxUpward = std::max(maxUpward, scene.capsuleVelocity().z);
        }
        REQUIRE(scene.capsulePosition().z > kCapsuleHalfHeightCm - 2.f);
        return maxUpward;
    }

    constexpr uint8_t kContactSphereNearDeclaration = 0;
    constexpr uint8_t kContactSphereFarDeclaration = 1;
    constexpr uint8_t kContactCapsuleDeclaration = 2;
    constexpr float kContactSphereRadiusCm = 30.f;
    constexpr float kGravityCmPerS2 = 980.f;
    constexpr float kPushOutEpsilonCm = 0.05f;
    constexpr float kCosMaxWalkableSlope = 0.70710678f;

    std::vector<SlotBodyTemplate> contactSlotTemplate()
    {
        SlotBodyTemplate nearSphere =
            sphereTemplate(kContactSphereRadiusCm, bit(kProjectile), bit(kWorld) | bit(kCharacter), kContactSphereNearDeclaration);
        nearSphere.descriptor.body.enableGravity = true;
        SlotBodyTemplate farSphere = nearSphere;
        farSphere.declarationIndex = kContactSphereFarDeclaration;
        SlotBodyTemplate capsule = capsuleTemplate(bit(kCharacter), bit(kWorld) | bit(kCharacter) | bit(kProjectile), kContactCapsuleDeclaration);
        capsule.descriptor.body.enableGravity = false;
        return { nearSphere, farSphere, capsule };
    }

    StaticShapeDescriptor staticSurface(StaticShapeDescriptor descriptor, float friction)
    {
        descriptor.friction = friction;
        descriptor.restitution = 0.3f;
        return descriptor;
    }

    struct ContactScene
    {
        ContactScene(JoltRuntime& runtime, StaticWorldDescription description)
            : statics(std::move(description))
            , world(runtime, configOf(statics), &logJoltProblems)
            , bodies(world)
        {
            JoltStaticWorldBuilder builder(world, [](const char*) {});
            (void)builder.build(statics);
            staticBodies = builder.stats().bodies;
            JoltPhysicsFactory factory(bodies, contactSlotTemplate(), 0u, 1u);
            for (const SlotBodyTemplate& slotTemplate : contactSlotTemplate())
                declarationBodies.push_back(factory.createPhysicalObject(slotTemplate.descriptor, "contact").bodyId);
            world.applyOccupancy(occupancyOf({ 0u }));
        }

        static JoltWorldConfig configOf(const StaticWorldDescription& statics)
        {
            JoltWorldConfig config;
            config.simulatableSlots = 1;
            config.slotTemplate = contactSlotTemplate();
            config.staticLayers = staticLayerKeysOf(statics);
            config.tempAllocatorBytes = 4u << 20;
            return config;
        }

        BodyId body(uint8_t declaration) const { return declarationBodies[declaration]; }
        void place(uint8_t declaration, const glm::vec3& positionCm)
        {
            bodies.setBodyTransform(body(declaration), glm::translate(glm::mat4(1.f), positionCm));
            bodies.setBodyLinearVelocity(body(declaration), glm::vec3(0.f));
        }
        glm::vec3 positionOf(uint8_t declaration) const { return bodies.captureBodyState(body(declaration)).position; }

        StaticWorldDescription statics;
        JoltWorld world;
        JoltPhysicsBodyAdapter bodies;
        std::vector<JPH::BodyID> staticBodies;
        std::vector<BodyId> declarationBodies;
    };

    class CombinedFrictionRecorder final : public JPH::ContactListener
    {
    public:
        void OnContactAdded(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold&, JPH::ContactSettings& settings) override
        {
            record(first, second, settings);
        }

        void OnContactPersisted(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold&, JPH::ContactSettings& settings) override
        {
            record(first, second, settings);
        }

        std::optional<float> between(JPH::BodyID first, JPH::BodyID second) const
        {
            const auto found = m_friction.find(keyOf(first, second));
            return found == m_friction.end() ? std::nullopt : std::optional<float>(found->second);
        }

    private:
        static std::pair<uint32_t, uint32_t> keyOf(JPH::BodyID first, JPH::BodyID second)
        {
            const uint32_t a = first.GetIndexAndSequenceNumber();
            const uint32_t b = second.GetIndexAndSequenceNumber();
            return { std::min(a, b), std::max(a, b) };
        }

        void record(const JPH::Body& first, const JPH::Body& second, const JPH::ContactSettings& settings)
        {
            m_friction[keyOf(first.GetID(), second.GetID())] = settings.mCombinedFriction;
        }

        std::map<std::pair<uint32_t, uint32_t>, float> m_friction;
    };

    struct SlopeRun
    {
        std::vector<float> pushOutUp;
        float downSlopeTravelCm = 0.f;
        float finalDownSlopeSpeedCmPerS = 0.f;
    };

    SlopeRun runSixtyDegreeSlope(JoltRuntime& runtime)
    {
        const glm::mat4 slopeRotation = glm::rotate(glm::mat4(1.f), glm::radians(60.f), glm::vec3(0.f, 1.f, 0.f));
        const glm::vec3 normal = glm::vec3(slopeRotation * glm::vec4(0.f, 0.f, 1.f, 0.f));
        const glm::vec3 downSlope = glm::vec3(slopeRotation * glm::vec4(1.f, 0.f, 0.f, 0.f));
        REQUIRE(normal.z == Catch::Approx(0.5f).margin(1e-6f));
        REQUIRE(downSlope.z < 0.f);

        StaticWorldDescription description;
        StaticShapeDescriptor slope = staticSurface(staticBox(glm::vec3(0.f), glm::vec3(1.f), 11u), 0.7f);
        StaticTriangleMesh mesh;
        mesh.vertices = { { -2000.f, -2000.f, 0.f }, { 2000.f, -2000.f, 0.f }, { 2000.f, 2000.f, 0.f }, { -2000.f, 2000.f, 0.f } };
        mesh.indices = { 0, 1, 2, 0, 2, 3 };
        slope.shape = mesh;
        slope.localToWorld = slopeRotation;
        description.shapes.push_back(slope);
        ContactScene scene(runtime, description);
        scene.place(kContactSphereNearDeclaration, glm::vec3(-3000.f, 0.f, 3000.f));
        scene.place(kContactSphereFarDeclaration, glm::vec3(-3000.f, 300.f, 3000.f));

        const glm::vec3 lowerSphereCentre = normal * (kCapsuleRadiusCm + 0.5f);
        const glm::vec3 start = lowerSphereCentre + glm::vec3(0.f, 0.f, kCapsuleHalfHeightCm - kCapsuleRadiusCm);
        scene.place(kContactCapsuleDeclaration, start);

        SlopeRun run;
        const BodyId capsule = scene.body(kContactCapsuleDeclaration);
        glm::vec3 velocity(0.f);
        glm::vec3 command(0.f);
        bool hasCommand = false;
        for (int tick = 0; tick < 30; ++tick)
        {
            const glm::vec3 position = scene.positionOf(kContactCapsuleDeclaration);
            if (hasCommand)
            {
                const glm::vec3 pushOut = position - (command + velocity * kDt);
                if (glm::dot(pushOut, pushOut) > kPushOutEpsilonCm * kPushOutEpsilonCm)
                {
                    const glm::vec3 contactNormal = glm::normalize(pushOut);
                    run.pushOutUp.push_back(contactNormal.z);
                    if (contactNormal.z >= kCosMaxWalkableSlope)
                        velocity.z = 0.f;
                    else
                        velocity -= glm::dot(velocity, contactNormal) * contactNormal;
                }
            }
            velocity.z -= kGravityCmPerS2 * kDt;
            command = position;
            hasCommand = true;
            scene.bodies.setBodyTransform(capsule, glm::translate(glm::mat4(1.f), position));
            scene.bodies.setBodyLinearVelocity(capsule, velocity);
            scene.world.step(kDt);
        }
        run.downSlopeTravelCm = glm::dot(scene.positionOf(kContactCapsuleDeclaration) - start, downSlope);
        run.finalDownSlopeSpeedCmPerS = glm::dot(velocity, downSlope);
        return run;
    }
} // namespace

TEST_CASE("JoltSuite.SaveRestoreAndDepthTwentyReplayCostWithOneTwentyAndThreeHundredStatics", "[Jolt][Stress]")
{
    RuntimeLease lease;
    struct Scene
    {
        const char* name;
        uint32_t extraStatics;
        uint32_t maxShapesPerChunk;
        bool linearCast;
    };
    const Scene scenes[] = { { "1 static element, 1 body", 0u, 256u, false }, { "20 static elements, 20 bodies", 19u, 1u, false },
        { "300 static elements, 300 bodies", 299u, 1u, false }, { "300 static elements, builder chunks", 299u, 256u, false },
        { "300 static elements, 300 bodies, LinearCast characters", 299u, 1u, true } };
    for (const Scene& sceneSpec : scenes)
    {
        const CostResult result = measureCost(lease.runtime, sceneSpec.extraStatics, sceneSpec.maxShapesPerChunk, sceneSpec.linearCast);
        std::printf("[JoltSuite] C cost, 8 occupied slots, %s (elements=%u staticBodies=%zu), ms:", sceneSpec.name, result.staticElements,
            result.staticBodies);
        print("step", result.step);
        print("save", result.save);
        print("step+save", result.stepPlusSave);
        print("restore", result.restore);
        print("restore+replay20", result.replay20);
        std::printf("\n");
        CHECK(result.stepPlusSave.median < 1.0);
        CHECK(result.replay20.median < 5.0);
    }
}

TEST_CASE("JoltSuite.TunnellingAtTheMaximumKnockbackSpeedWithAndWithoutLinearCast", "[Jolt]")
{
    RuntimeLease lease;
    constexpr float kMaxKnockbackCmPerS = 2000.f;
    for (const WallKind kind : { WallKind::Box1Cm, WallKind::Box10Cm, WallKind::TriangleMesh })
    {
        for (const float speed : { kMaxKnockbackCmPerS, 2500.f, 2828.f, 2.f * kMaxKnockbackCmPerS, 4.f * kMaxKnockbackCmPerS, 8.f * kMaxKnockbackCmPerS })
        {
            const TunnelResult discrete = runTunnelling(lease.runtime, kind, false, speed);
            const TunnelResult linearCast = runTunnelling(lease.runtime, kind, true, speed);
            std::printf("[JoltSuite] D tunnelling, %s wall, %.0f cm/s (%.1f cm/tick): discrete tunnelled %u/%u (max penetration %.2f cm); "
                        "LinearCast tunnelled %u/%u (max penetration %.2f cm)\n",
                nameOf(kind), double(speed), double(speed * kDt), discrete.tunnelled, discrete.trials, double(discrete.maxPenetrationCm),
                linearCast.tunnelled, linearCast.trials, double(linearCast.maxPenetrationCm));
            REQUIRE(linearCast.tunnelled == 0u);
            if (speed == kMaxKnockbackCmPerS)
                REQUIRE(discrete.tunnelled == 0u);
        }
    }
    REQUIRE_FALSE(JoltPhysicsFactoryOptions{}.linearCastCharacters);
}

TEST_CASE("JoltSuite.TheRestitutionVelocityThresholdIsChaosParity", "[Jolt]")
{
    REQUIRE(kChaosParityMinVelocityForRestitutionCmPerS == Catch::Approx(1000.0 / 60.0).epsilon(1e-6));
    REQUIRE(JoltWorldConfig{}.minVelocityForRestitutionCmPerS == kChaosParityMinVelocityForRestitutionCmPerS);

    RuntimeLease lease;
    {
        StaticWorldDescription floor;
        floor.shapes.push_back(staticBox(glm::vec3(0.f, 0.f, -50.f), glm::vec3(1000.f, 1000.f, 50.f), 1u));
        SingleSlotScene scene(lease.runtime, floor, glm::vec3(0.f), std::nullopt, JoltPhysicsFactoryOptions{});
        REQUIRE(scene.world.physics().GetPhysicsSettings().mMinVelocityForRestitution
            == joltUnits::centimetresToMetres(kChaosParityMinVelocityForRestitutionCmPerS));
        REQUIRE(JPH::PhysicsSettings{}.mMinVelocityForRestitution == 1.f);
    }

    const float joltDefaultCmPerS = 100.f;
    const float slowParity = bounceSpeed(lease.runtime, std::nullopt, 10.f);
    const float betweenParity = bounceSpeed(lease.runtime, std::nullopt, 50.f);
    const float fastParity = bounceSpeed(lease.runtime, std::nullopt, 150.f);
    const float slowJolt = bounceSpeed(lease.runtime, joltDefaultCmPerS, 10.f);
    const float betweenJolt = bounceSpeed(lease.runtime, joltDefaultCmPerS, 50.f);
    const float fastJolt = bounceSpeed(lease.runtime, joltDefaultCmPerS, 150.f);
    std::printf("[JoltSuite] restitution threshold: capsule (0/0) on a 0.7/0.3 floor, combined restitution 0.15; rebound cm/s at approach 10 / 50 / 150: "
                "Chaos-parity threshold %.2f / %.2f / %.2f, Jolt default threshold %.2f / %.2f / %.2f\n",
        double(slowParity), double(betweenParity), double(fastParity), double(slowJolt), double(betweenJolt), double(fastJolt));

    CHECK(slowParity < 0.5f);
    CHECK(slowJolt < 0.5f);
    CHECK(betweenParity == Catch::Approx(0.15f * 50.f).margin(2.f));
    CHECK(betweenJolt < 0.5f);
    CHECK(fastParity == Catch::Approx(0.15f * 150.f).margin(4.f));
    CHECK(fastJolt == Catch::Approx(0.15f * 150.f).margin(4.f));
}

TEST_CASE("JoltSuite.CapsulePushOutOnASixtyDegreeSlopeIsTheFaceNormal", "[Jolt]")
{
    RuntimeLease lease;
    const SlopeRun run = runSixtyDegreeSlope(lease.runtime);
    std::printf("[JoltSuite] 60 deg slope, root capsule under the movement sim's command/step/push-out loop: %zu push-out ticks, "
                "normalize(pushOut).z per tick:",
        run.pushOutUp.size());
    for (const float up : run.pushOutUp)
        std::printf(" %.4f", double(up));
    std::printf("; down-slope travel after 30 ticks %.2f cm, final down-slope speed %.1f cm/s\n", double(run.downSlopeTravelCm),
        double(run.finalDownSlopeSpeedCmPerS));

    REQUIRE(run.pushOutUp.size() >= 20u);
    for (const float up : run.pushOutUp)
        CHECK(up == Catch::Approx(0.5f).margin(0.03f));
    CHECK(run.downSlopeTravelCm > 50.f);
}

TEST_CASE("JoltSuite.OnlyContactsWithARootCapsuleAreFrictionless", "[Jolt]")
{
    RuntimeLease lease;
    StaticWorldDescription description;
    description.shapes.push_back(staticSurface(staticBox(glm::vec3(-230.f, 0.f, -50.f), glm::vec3(270.f, 500.f, 50.f), 21u), 0.7f));
    description.shapes.push_back(staticSurface(staticBox(glm::vec3(270.f, 0.f, -50.f), glm::vec3(230.f, 500.f, 50.f), 22u), 0.f));
    CombinedFrictionRecorder recorder;
    ContactScene scene(lease.runtime, description);
    REQUIRE(scene.staticBodies.size() == 2u);
    const bool firstIsHigh = scene.world.bodies().GetFriction(scene.staticBodies[0]) == 0.7f;
    const JPH::BodyID highFrictionFloor = scene.staticBodies[firstIsHigh ? 0u : 1u];
    const JPH::BodyID zeroFrictionFloor = scene.staticBodies[firstIsHigh ? 1u : 0u];
    REQUIRE(scene.world.bodies().GetFriction(highFrictionFloor) == 0.7f);
    REQUIRE(scene.world.bodies().GetFriction(zeroFrictionFloor) == 0.f);

    scene.place(kContactCapsuleDeclaration, glm::vec3(0.f, 0.f, kCapsuleHalfHeightCm - 0.5f));
    scene.place(kContactSphereFarDeclaration, glm::vec3(-300.f, 0.f, kContactSphereRadiusCm - 0.5f));
    scene.place(kContactSphereNearDeclaration, glm::vec3(kCapsuleRadiusCm + kContactSphereRadiusCm - 1.5f, 0.f, kContactSphereRadiusCm - 0.5f));
    scene.world.physics().SetContactListener(&recorder);
    for (int tick = 0; tick < 2; ++tick)
        scene.world.step(kDt);
    scene.world.physics().SetContactListener(nullptr);

    const JPH::BodyID capsule = JoltPhysicsBodyAdapter::joltBodyIdOf(scene.body(kContactCapsuleDeclaration));
    const JPH::BodyID nearSphere = JoltPhysicsBodyAdapter::joltBodyIdOf(scene.body(kContactSphereNearDeclaration));
    const JPH::BodyID farSphere = JoltPhysicsBodyAdapter::joltBodyIdOf(scene.body(kContactSphereFarDeclaration));
    const std::optional<float> capsuleOnFloor = recorder.between(capsule, highFrictionFloor);
    const std::optional<float> capsuleOnSphere = recorder.between(capsule, nearSphere);
    const std::optional<float> sphereOnZeroFloor = recorder.between(nearSphere, zeroFrictionFloor);
    const std::optional<float> sphereOnHighFloor = recorder.between(farSphere, highFrictionFloor);
    REQUIRE(capsuleOnFloor.has_value());
    REQUIRE(capsuleOnSphere.has_value());
    REQUIRE(sphereOnZeroFloor.has_value());
    REQUIRE(sphereOnHighFloor.has_value());
    std::printf("[JoltSuite] combined friction: root capsule vs 0.7 static %.3f, root capsule vs created sphere %.3f, "
                "created sphere vs 0 static %.3f, created sphere vs 0.7 static %.3f\n",
        double(*capsuleOnFloor), double(*capsuleOnSphere), double(*sphereOnZeroFloor), double(*sphereOnHighFloor));

    CHECK(*capsuleOnFloor == 0.f);
    CHECK(*capsuleOnSphere == 0.f);
    CHECK(*sphereOnZeroFloor == 0.35f);
    CHECK(*sphereOnHighFloor == 0.7f);
}

#endif // WITH_LOW_LEVEL_TESTS
