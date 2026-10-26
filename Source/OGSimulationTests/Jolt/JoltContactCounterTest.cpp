// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltBodyUserData.h"
#include "OGSimulationJolt/JoltContactCounter.h"

#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/Body.h>


#include <array>
#include <cstring>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

using namespace joltTestRig;

namespace joltContactCounterTest
{
    constexpr uint32_t kSphere = 0;
    constexpr uint32_t kCapsule = 1;
    constexpr uint32_t kContactCategories = bit(kWorld) | bit(kCharacter) | bit(kBody) | bit(kProjectile);

    JoltWorldConfig sceneConfig()
    {
        SlotBodyTemplate sphere = sphereTemplate(0.3f, bit(kBody), kContactCategories, uint8_t(kSphere));
        sphere.descriptor.body.enableGravity = true;
        JoltWorldConfig config;
        config.simulatableSlots = 4;
        config.slotTemplate = { sphere, capsuleTemplate(bit(kCharacter), kContactCategories, uint8_t(kCapsule)) };
        config.staticLayers = { kWorldStaticKey };
        config.tempAllocatorBytes = 4u << 20;
        return config;
    }

    void bindSlot(JoltWorld& world, uint32_t slot, uint32_t simulatableId)
    {
        for (uint32_t templateIndex = 0; templateIndex < world.bodiesPerSlot(); ++templateIndex)
        {
            world.bodies().SetUserData(world.slotBodyId(slot, templateIndex),
                joltBodyUserData::encode(JoltBodyBinding{ simulatableId, uint8_t(templateIndex) }, templateIndex == kCapsule));
        }
    }

    void place(JoltWorld& world, uint32_t slot, uint32_t templateIndex, float x, float y, float z)
    {
        world.bodies().SetPosition(world.slotBodyId(slot, templateIndex), JPH::RVec3(x, y, z), JPH::EActivation::Activate);
    }

    void park(JoltWorld& world, uint32_t slot, uint32_t templateIndex)
    {
        const JPH::BodyID id = world.slotBodyId(slot, templateIndex);
        world.bodies().SetGravityFactor(id, 0.f);
        place(world, slot, templateIndex, 3.f * float(templateIndex), 6.f, -(12.f + 3.f * float(slot)));
    }

    constexpr float kCapsuleRestHeight = 0.96f - 0.005f;
    constexpr float kSphereRestHeight = 0.30f - 0.005f;
    constexpr float kCapsuleSphereTouchDistance = 0.42f + 0.30f - 0.005f;

    struct Expected
    {
        std::array<uint32_t, kJoltContactPairClassCount> pairs{};
        uint32_t added = 0;
        std::map<uint32_t, std::pair<std::array<uint32_t, kJoltContactPairClassCount>, uint32_t>> ids;
    };

    std::array<uint32_t, kJoltContactPairClassCount> pairsOf(uint32_t rootRoot, uint32_t rootChild, uint32_t rootStatic,
        uint32_t childChild, uint32_t childStatic, uint32_t staticStatic)
    {
        return { rootRoot, rootChild, rootStatic, childChild, childStatic, staticStatic };
    }

    void checkCounts(const JoltContactStepCounts& counts, const Expected& expected)
    {
        for (size_t index = 0; index < kJoltContactPairClassCount; ++index)
        {
            INFO("pair class " << index);
            CHECK(counts.pairs[index] == expected.pairs[index]);
        }
        CHECK(counts.added == expected.added);
        CHECK(counts.unrecordedIds == 0u);
        CHECK(counts.unrecordedDeclarations == 0u);
        REQUIRE(counts.idCount == expected.ids.size());
        uint32_t index = 0;
        for (const auto& [simulatableId, perId] : expected.ids)
        {
            INFO("id " << simulatableId);
            const JoltContactIdCounts& id = counts.ids[index++];
            CHECK(id.simulatableId == simulatableId);
            for (size_t pairIndex = 0; pairIndex < kJoltContactPairClassCount; ++pairIndex)
            {
                INFO("pair class " << pairIndex);
                CHECK(id.pairs[pairIndex] == perId.first[pairIndex]);
            }
            CHECK(id.childDeclarations == perId.second);
        }
    }

    void checkEmpty(const JoltContactStepCounts& counts)
    {
        CHECK(counts.manifolds() == 0u);
        CHECK(counts.added == 0u);
        CHECK(counts.idCount == 0u);
        CHECK(counts.unrecordedIds == 0u);
        CHECK(counts.unrecordedDeclarations == 0u);
    }

    struct Scene
    {
        RuntimeLease lease;
        JoltWorld world{ lease.runtime, sceneConfig(), &logJoltProblems };
        JoltContactCounter counter;

        explicit Scene(bool floor)
        {
            if (floor)
            {
                addFloor(world);
            }
            world.applyBodySlotOccupancy(allOccupied());
            for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
            {
                bindSlot(world, slot, 7u + 2u * slot);
                park(world, slot, kSphere);
                park(world, slot, kCapsule);
            }
            world.physics().SetContactListener(&counter);
        }

        JoltContactStepCounts stepAndDrain()
        {
            world.step(kDt);
            return counter.drain();
        }
    };

    std::vector<uint8_t> bodyStateBytes(const JoltWorld& world)
    {
        std::vector<uint8_t> bytes;
        const JPH::BodyInterface& bodies = world.bodies();
        const auto append = [&bytes](const void* data, size_t size)
        {
            const auto* begin = static_cast<const uint8_t*>(data);
            bytes.insert(bytes.end(), begin, begin + size);
        };
        for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
        {
            for (uint32_t templateIndex = 0; templateIndex < world.bodiesPerSlot(); ++templateIndex)
            {
                const JPH::BodyID id = world.slotBodyId(slot, templateIndex);
                const JPH::RVec3 position = bodies.GetPosition(id);
                const JPH::Quat rotation = bodies.GetRotation(id);
                const JPH::Vec3 linear = bodies.GetLinearVelocity(id);
                const JPH::Vec3 angular = bodies.GetAngularVelocity(id);
                const float values[13] = { float(position.GetX()), float(position.GetY()), float(position.GetZ()),
                    rotation.GetX(), rotation.GetY(), rotation.GetZ(), rotation.GetW(),
                    linear.GetX(), linear.GetY(), linear.GetZ(), angular.GetX(), angular.GetY(), angular.GetZ() };
                append(values, sizeof(values));
            }
        }
        return bytes;
    }

    JoltWorldConfig busyConfig()
    {
        JoltWorldConfig config = brawlerWorldConfig(0);
        for (uint32_t templateIndex = kTemplateProjectileFirst; templateIndex < kTemplateCapsule; ++templateIndex)
        {
            config.slotTemplate[templateIndex].descriptor.body.enableGravity = true;
            config.slotTemplate[templateIndex].descriptor.shapes[0].blockingCategories =
                CollisionCategories{ bit(kWorld) | bit(kCharacter) | bit(kProjectile) };
        }
        config.slotTemplate[kTemplateCapsule].descriptor.shapes[0].blockingCategories =
            CollisionCategories{ bit(kWorld) | bit(kCharacter) | bit(kProjectile) };
        return config;
    }

    void prepareBusyWorld(JoltWorld& world)
    {
        addFloor(world);
        world.applyBodySlotOccupancy(allOccupied());
        placeAllBrawlers(world);
        for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
        {
            for (uint32_t templateIndex = 0; templateIndex < world.bodiesPerSlot(); ++templateIndex)
            {
                world.bodies().SetUserData(world.slotBodyId(slot, templateIndex),
                    joltBodyUserData::encode(JoltBodyBinding{ 100u + slot, uint8_t(templateIndex) }, templateIndex == kTemplateCapsule));
            }
        }
    }

    struct Tally
    {
        std::array<uint32_t, kJoltContactPairClassCount> pairs{};
        uint32_t added = 0;
        std::map<uint32_t, std::pair<std::array<uint32_t, kJoltContactPairClassCount>, uint32_t>> ids;
    };

    size_t referenceClassOf(uint64_t userData)
    {
        if ((userData & joltBodyUserData::kRootBit) != 0u)
        {
            return 0;
        }
        return (userData & joltBodyUserData::kBoundBit) != 0u ? 1 : 2;
    }

    size_t referencePairOf(size_t first, size_t second)
    {
        static constexpr size_t table[3][3] = { { 0, 1, 2 }, { 1, 3, 4 }, { 2, 4, 5 } };
        return table[first][second];
    }

    class ReferenceListener final : public JPH::ContactListener
    {
    public:
        explicit ReferenceListener(JoltContactCounter& counter) : m_counter(counter) {}

        void OnContactAdded(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold& manifold, JPH::ContactSettings& settings) override
        {
            m_counter.OnContactAdded(first, second, manifold, settings);
            record(first, second, true);
        }

        void OnContactPersisted(const JPH::Body& first, const JPH::Body& second, const JPH::ContactManifold& manifold, JPH::ContactSettings& settings) override
        {
            m_counter.OnContactPersisted(first, second, manifold, settings);
            record(first, second, false);
        }

        Tally take()
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            Tally taken = m_tally;
            m_tally = Tally{};
            return taken;
        }

        size_t threadsSeen()
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_threads.size();
        }

    private:
        void record(const JPH::Body& first, const JPH::Body& second, bool added)
        {
            const uint64_t firstData = first.GetUserData();
            const uint64_t secondData = second.GetUserData();
            const size_t pair = referencePairOf(referenceClassOf(firstData), referenceClassOf(secondData));
            std::lock_guard<std::mutex> lock(m_mutex);
            m_threads.insert(std::this_thread::get_id());
            ++m_tally.pairs[pair];
            m_tally.added += added ? 1u : 0u;
            std::set<uint32_t> counted;
            for (const uint64_t data : { firstData, secondData })
            {
                if ((data & joltBodyUserData::kBoundBit) == 0u)
                {
                    continue;
                }
                const uint32_t id = uint32_t((data >> 8) & 0xFFFFFFFFu);
                auto& perId = m_tally.ids[id];
                if (counted.insert(id).second)
                {
                    ++perId.first[pair];
                }
                if ((data & joltBodyUserData::kRootBit) == 0u)
                {
                    perId.second |= uint32_t(1) << (data & 0xFFu);
                }
            }
        }

        JoltContactCounter& m_counter;
        std::mutex m_mutex;
        Tally m_tally;
        std::set<std::thread::id> m_threads;
    };
}

using namespace joltContactCounterTest;

TEST_CASE("JoltContactCounter.ACapsuleRestingOnAStaticIsOneRootStaticManifold", "[Jolt]")
{
    Scene scene(true);
    place(scene.world, 0, kCapsule, 0.f, kCapsuleRestHeight, -0.f);
    scene.world.bodies().SetGravityFactor(scene.world.slotBodyId(0, kCapsule), 1.f);

    Expected expected;
    expected.pairs = pairsOf(0, 0, 1, 0, 0, 0);
    expected.added = 1;
    expected.ids[7] = { pairsOf(0, 0, 1, 0, 0, 0), 0u };
    checkCounts(scene.stepAndDrain(), expected);
    checkEmpty(scene.counter.drain());

    expected.added = 0;
    checkCounts(scene.stepAndDrain(), expected);
}

TEST_CASE("JoltContactCounter.TwoTouchingCapsulesAreOneRootRootManifoldForEachId", "[Jolt]")
{
    Scene scene(false);
    place(scene.world, 0, kCapsule, 0.f, 3.f, -0.f);
    place(scene.world, 1, kCapsule, 0.f, 3.f, -(0.84f - 0.005f));

    Expected expected;
    expected.pairs = pairsOf(1, 0, 0, 0, 0, 0);
    expected.added = 1;
    expected.ids[7] = { pairsOf(1, 0, 0, 0, 0, 0), 0u };
    expected.ids[9] = { pairsOf(1, 0, 0, 0, 0, 0), 0u };
    checkCounts(scene.stepAndDrain(), expected);
    checkEmpty(scene.counter.drain());
}

TEST_CASE("JoltContactCounter.AChildTouchingACapsuleAndAStaticCarriesItsDeclaration", "[Jolt]")
{
    Scene scene(true);
    place(scene.world, 0, kCapsule, 0.f, kCapsuleRestHeight, -0.f);
    place(scene.world, 1, kSphere, 0.f, kSphereRestHeight, -kCapsuleSphereTouchDistance);
    scene.world.bodies().SetGravityFactor(scene.world.slotBodyId(0, kCapsule), 1.f);
    scene.world.bodies().SetGravityFactor(scene.world.slotBodyId(1, kSphere), 1.f);

    Expected expected;
    expected.pairs = pairsOf(0, 1, 1, 0, 1, 0);
    expected.added = 3;
    expected.ids[7] = { pairsOf(0, 1, 1, 0, 0, 0), 0u };
    expected.ids[9] = { pairsOf(0, 1, 0, 0, 1, 0), 1u << kSphere };
    checkCounts(scene.stepAndDrain(), expected);
    checkEmpty(scene.counter.drain());
}

TEST_CASE("JoltContactCounter.ChildChildSameIdAndUnboundBodies", "[Jolt]")
{
    Scene scene(true);
    place(scene.world, 0, kSphere, 0.f, 3.f, 3.f);
    place(scene.world, 1, kSphere, 0.f, 3.f, -(-3.f + 0.6f - 0.005f));
    place(scene.world, 2, kCapsule, 0.f, 3.f, -3.f);
    place(scene.world, 2, kSphere, 0.f, 3.f, -(3.f + kCapsuleSphereTouchDistance));
    place(scene.world, 3, kCapsule, 3.f, kCapsuleRestHeight, -0.f);
    place(scene.world, 3, kSphere, -3.f, kSphereRestHeight, -0.f);
    scene.world.bodies().SetUserData(scene.world.slotBodyId(3, kCapsule), joltBodyUserData::encodeUnbound(true));
    scene.world.bodies().SetUserData(scene.world.slotBodyId(3, kSphere), joltBodyUserData::encodeUnbound(false));

    Expected expected;
    expected.pairs = pairsOf(0, 1, 1, 1, 0, 1);
    expected.added = 4;
    expected.ids[7] = { pairsOf(0, 0, 0, 1, 0, 0), 1u << kSphere };
    expected.ids[9] = { pairsOf(0, 0, 0, 1, 0, 0), 1u << kSphere };
    expected.ids[11] = { pairsOf(0, 1, 0, 0, 0, 0), 1u << kSphere };
    checkCounts(scene.stepAndDrain(), expected);
    checkEmpty(scene.counter.drain());
}

TEST_CASE("JoltContactCounter.AWorldWithTheCounterStepsBitIdenticallyToOneWithout", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld without(lease.runtime, busyConfig(), &logJoltProblems);
    JoltWorld with(lease.runtime, busyConfig(), &logJoltProblems);
    prepareBusyWorld(without);
    prepareBusyWorld(with);

    JoltContactCounter counter;
    struct Drained
    {
        JoltContactCounter* counter = nullptr;
        std::array<uint64_t, kJoltContactPairClassCount> pairs{};
        uint64_t steps = 0;
        uint64_t idSteps = 0;
    } drained{ &counter };
    with.physics().SetContactListener(&counter);
    with.setAfterStepObserver([](void* context)
    {
        Drained& state = *static_cast<Drained*>(context);
        const JoltContactStepCounts counts = state.counter->drain();
        for (size_t index = 0; index < kJoltContactPairClassCount; ++index)
        {
            state.pairs[index] += counts.pairs[index];
        }
        ++state.steps;
        state.idSteps += counts.idCount != 0u ? 1u : 0u;
    }, &drained);
    REQUIRE(without.physics().GetContactListener() == nullptr);

    const std::vector<uint8_t> initial = bodyStateBytes(without);
    constexpr uint32_t kSteps = 240;
    for (uint32_t tick = 0; tick < kSteps; ++tick)
    {
        applyScriptedForces(without, tick);
        applyScriptedForces(with, tick);
        without.step(kDt);
        with.step(kDt);
        const std::vector<uint8_t> first = bodyStateBytes(without);
        const std::vector<uint8_t> second = bodyStateBytes(with);
        INFO("tick " << tick);
        REQUIRE(first.size() == second.size());
        REQUIRE(std::memcmp(first.data(), second.data(), first.size()) == 0);
    }

    CHECK(bodyStateBytes(without) != initial);
    CHECK(sameBytes(fullState(without), fullState(with)));
    CHECK(without.liveStateHash() == with.liveStateHash());
    CHECK(drained.steps == kSteps);
    CHECK(drained.idSteps == kSteps);
    CHECK(drained.pairs[size_t(JoltContactPairClass::RootStatic)] > 0u);
    CHECK(drained.pairs[size_t(JoltContactPairClass::ChildStatic)] > 0u);
    CHECK(drained.pairs[size_t(JoltContactPairClass::ChildChild)] > 0u);
    CHECK(drained.pairs[size_t(JoltContactPairClass::RootChild)] > 0u);
}

TEST_CASE("JoltContactCounter.CountsAreExactUnderAThreadPoolJobSystem", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, busyConfig(), &logJoltProblems);
    prepareBusyWorld(world);

    JoltContactCounter counter;
    ReferenceListener reference(counter);
    world.physics().SetContactListener(&reference);
    JPH::TempAllocatorImpl tempAllocator(16u << 20);
    JPH::JobSystemThreadPool jobs(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 4);

    uint32_t stepsWithContacts = 0;
    uint64_t manifolds = 0;
    for (uint32_t tick = 0; tick < 240; ++tick)
    {
        applyScriptedForces(world, tick);
        world.physics().Update(kDt, 1, &tempAllocator, &jobs);
        const JoltContactStepCounts counts = counter.drain();
        const Tally tally = reference.take();
        INFO("tick " << tick);
        Expected expected;
        expected.pairs = tally.pairs;
        expected.added = tally.added;
        expected.ids = tally.ids;
        checkCounts(counts, expected);
        stepsWithContacts += counts.manifolds() != 0u ? 1u : 0u;
        manifolds += counts.manifolds();
    }
    CHECK(stepsWithContacts == 240u);
    CHECK(manifolds > 240u * 8u);
    CHECK(reference.threadsSeen() > 1u);
}

#endif
