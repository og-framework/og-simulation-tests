// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulation/PhysicsWorldAdapter.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <vector>

using namespace joltTestRig;

namespace
{
    struct RecordedRun
    {
        std::vector<std::vector<uint8_t>> stateAtTick;
        std::vector<uint64_t> hashAtTick;
    };

    void simulateTick(JoltWorld& world, uint32_t tick)
    {
        applyScriptedForces(world, tick);
        world.step(kDt);
        world.saveTick(tick);
    }

    RecordedRun runEightBrawlers(JoltWorld& world, uint32_t ticks)
    {
        addFloor(world);
        world.applyOccupancy(allOccupied());
        placeAllBrawlers(world);

        RecordedRun run;
        run.stateAtTick.push_back(fullState(world));
        run.hashAtTick.push_back(world.liveStateHash());
        for (uint32_t tick = 1; tick <= ticks; ++tick)
        {
            simulateTick(world, tick);
            run.stateAtTick.push_back(fullState(world));
            run.hashAtTick.push_back(world.liveStateHash());
            if (world.ring().depthTicks() > 0u)
            {
                const std::optional<uint64_t> held = world.stateHash(tick);
                REQUIRE(held.has_value());
                REQUIRE(*held == run.hashAtTick.back());
            }
        }
        return run;
    }
}

TEST_CASE("JoltWorld.SatisfiesThePhysicsWorldAdapterConcept", "[Jolt]")
{
    STATIC_REQUIRE(PhysicsWorldAdapter<JoltWorld>);
    STATIC_REQUIRE(!JoltWorld::coverage.bodySetAndIds);
    STATIC_REQUIRE(!JoltWorld::coverage.broadphase);
    STATIC_REQUIRE(!JoltWorld::coverage.staticShapes);
}

TEST_CASE("JoltWorld.S1UnfilteredRestoreAndReplayIsByteIdentical", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(0), nullptr);
    const RecordedRun original = runEightBrawlers(world, 300);
    REQUIRE(original.stateAtTick[300].size() > 0u);
    REQUIRE_FALSE(sameBytes(original.stateAtTick[300], original.stateAtTick[200]));

    REQUIRE(restoreFullState(world, original.stateAtTick[200]));
    for (uint32_t tick = 201; tick <= 300; ++tick)
    {
        simulateTick(world, tick);
    }
    CHECK(sameBytes(fullState(world), original.stateAtTick[300]));
}

TEST_CASE("JoltWorld.S1FilteredRingRestoreAndReplayIsByteIdentical", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(128), nullptr);
    const RecordedRun original = runEightBrawlers(world, 300);

    REQUIRE(world.hasTick(200));
    REQUIRE(world.restoreTick(200));
    CHECK(sameBytes(fullState(world), original.stateAtTick[200]));
    for (uint32_t tick = 201; tick <= 300; ++tick)
    {
        simulateTick(world, tick);
        INFO("tick " << tick);
        CHECK(world.stateHash(tick) == std::optional<uint64_t>(original.hashAtTick[tick]));
    }
    CHECK(sameBytes(fullState(world), original.stateAtTick[300]));
}

TEST_CASE("JoltWorld.S3FilteredRingNextStepAfterRestoreIsByteIdentical", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(320), nullptr);
    const RecordedRun original = runEightBrawlers(world, 300);

    uint32_t compared = 0;
    for (uint32_t tick = 1; tick < 300; tick += 7)
    {
        REQUIRE(world.restoreTick(tick));
        applyScriptedForces(world, tick + 1u);
        world.step(kDt);
        INFO("restored tick " << tick);
        CHECK(sameBytes(fullState(world), original.stateAtTick[tick + 1u]));
        CHECK(world.liveStateHash() == original.hashAtTick[tick + 1u]);
        ++compared;
    }
    CHECK(compared == 43u);
}

TEST_CASE("JoltWorld.BytesPerSavedTickWithEightOccupiedSlots", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorldConfig config = brawlerWorldConfig(32);
    JoltWorld world(lease.runtime, config, nullptr);
    (void)runEightBrawlers(world, 120);

    uint32_t largest = 0;
    for (uint32_t tick = 89; tick <= 120; ++tick)
    {
        REQUIRE(world.ring().find(tick) != nullptr);
        largest = std::max(largest, world.ring().find(tick)->byteCount);
    }
    const uint32_t savedBodies = static_cast<uint32_t>(world.ring().find(120)->bodyIds.size());
    CHECK(savedBodies == 48u);
    CHECK(largest > 48u * 100u);
    CHECK(largest < config.ringSlotBytes);
    CHECK(world.ring().reservedBytes() < 34u * (config.ringSlotBytes + 1024u));
}

TEST_CASE("JoltWorld.RingHoldsEvictsOverwritesAndInvalidates", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(4), nullptr);
    addFloor(world);
    world.applyOccupancy(allOccupied());
    placeAllBrawlers(world);

    for (uint32_t tick = 1; tick <= 6; ++tick)
    {
        simulateTick(world, tick);
    }
    CHECK_FALSE(world.hasTick(1));
    CHECK_FALSE(world.hasTick(2));
    CHECK_FALSE(world.restoreTick(2));
    CHECK_FALSE(world.stateHash(2).has_value());
    CHECK(world.hasTick(3));
    CHECK(world.hasTick(6));
    REQUIRE(world.oldestHeldTick().has_value());
    CHECK(world.oldestHeldTick() == std::optional<SimTick>(3u));

    world.saveTick(1);
    CHECK_FALSE(world.hasTick(1));
    CHECK(world.oldestHeldTick() == std::optional<SimTick>(3u));
    CHECK(world.ring().heldTickCount() == 4u);

    const std::vector<uint8_t> beforeOverwrite = fullState(world);
    applyScriptedForces(world, 7);
    world.step(kDt);
    const std::vector<uint8_t> overwritten = fullState(world);
    REQUIRE_FALSE(sameBytes(overwritten, beforeOverwrite));
    world.saveTick(5);
    CHECK(world.ring().heldTickCount() == 4u);
    CHECK(world.hasTick(3));
    REQUIRE(world.restoreTick(5));
    CHECK(sameBytes(fullState(world), overwritten));

    world.invalidateAllTicks();
    for (uint32_t tick = 1; tick <= 7; ++tick)
    {
        CHECK_FALSE(world.hasTick(tick));
    }
    CHECK_FALSE(world.oldestHeldTick().has_value());
    CHECK_FALSE(world.restoreTick(6));
}

TEST_CASE("JoltWorld.AuthorityDepthZeroHoldsNothing", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(0), nullptr);
    world.saveTick(1);
    world.saveScratch();
    world.commitScratch(2);
    CHECK_FALSE(world.hasTick(1));
    CHECK_FALSE(world.hasTick(2));
    CHECK_FALSE(world.restoreTick(1));
    CHECK_FALSE(world.oldestHeldTick().has_value());
}

TEST_CASE("JoltWorld.CommitScratchRestoresThePreStepWorld", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(8), nullptr);
    (void)runEightBrawlers(world, 20);

    world.saveScratch();
    const std::vector<uint8_t> preStep = fullState(world);
    const uint64_t preStepHash = world.liveStateHash();
    simulateTick(world, 21);
    simulateTick(world, 22);
    REQUIRE_FALSE(sameBytes(fullState(world), preStep));

    world.commitScratch(42);
    REQUIRE(world.hasTick(42));
    CHECK(world.stateHash(42) == std::optional<uint64_t>(preStepHash));
    REQUIRE(world.restoreTick(42));
    CHECK(sameBytes(fullState(world), preStep));
}

TEST_CASE("JoltWorld.BodyIdsAreNeverZero", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(2), nullptr);
    addFloor(world);
    addFloor(world, kWorldStaticKey, -3.f);

    CHECK(world.slotBodyId(0, 0).GetIndexAndSequenceNumber() == 1u);
    JPH::BodyIDVector ids;
    world.physics().GetBodies(ids);
    CHECK(ids.size() == kMaxSimulatableSlots * kBodiesPerBrawler + 2u);
    for (const JPH::BodyID& id : ids)
    {
        CHECK(id.GetIndexAndSequenceNumber() != 0u);
        CHECK(id.GetIndex() != 0u);
    }
}

TEST_CASE("JoltWorld.ParkedBodiesMakeNoContactsAndKeepZeroVelocity", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorldConfig config = brawlerWorldConfig(0);
    config.slotTemplate = brawlerSlotTemplate(bit(kProjectile));
    config.parkingPositionCm = glm::vec3(0.f, 0.f, 1000.f);

    JoltWorld parked(lease.runtime, config, nullptr);
    addFloor(parked);
    CountingContactListener parkedContacts;
    parked.physics().SetContactListener(&parkedContacts);
    for (uint32_t index = 0; index < 100; ++index)
    {
        parked.step(kDt);
    }
    CHECK(parkedContacts.added == 0u);
    CHECK(parkedContacts.persisted == 0u);
    uint32_t parkedProjectiles = 0;
    for (uint32_t slot = 0; slot < parked.simulatableSlots(); ++slot)
    {
        for (uint32_t index = 0; index < kBodiesPerBrawler; ++index)
        {
            const JPH::BodyID id = parked.slotBodyId(slot, index);
            CHECK(parked.bodies().GetLinearVelocity(id) == JPH::Vec3::sZero());
            CHECK(parked.bodies().GetAngularVelocity(id) == JPH::Vec3::sZero());
            CHECK(parked.bodies().GetCenterOfMassPosition(id) == joltUnits::centimetresToMetres(config.parkingPositionCm));
            CHECK(parked.bodies().GetObjectLayer(id) == parked.layers().parkedLayer());
            parkedProjectiles += (index >= kTemplateProjectileFirst && index < kTemplateCapsule) ? 1u : 0u;
        }
    }
    CHECK(parkedProjectiles == 24u);
    parked.physics().SetContactListener(nullptr);

    JoltWorld live(lease.runtime, config, nullptr);
    addFloor(live);
    live.applyOccupancy(allOccupied());
    CountingContactListener liveContacts;
    live.physics().SetContactListener(&liveContacts);
    for (uint32_t index = 0; index < 100; ++index)
    {
        live.step(kDt);
    }
    CHECK(liveContacts.added > 0u);
    CHECK(live.bodies().GetLinearVelocity(live.slotBodyId(0, kTemplateCapsule)) != JPH::Vec3::sZero());
    live.physics().SetContactListener(nullptr);
}

TEST_CASE("JoltWorld.VacantSlotIsInvisibleToAWorldCollideShape", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(0), nullptr);
    addFloor(world, kWorldStaticKey, -50.f);

    const auto countSlotHits = [&world]()
    {
        JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
        const JoltQueryLayerFilter anyCategory(world.layers(), ~0u);
        const JPH::RefConst<JPH::Shape> probe = new JPH::SphereShape(2.f);
        world.query().CollideShape(probe, JPH::Vec3::sOne(), JPH::RMat44::sTranslation(JPH::RVec3::sZero()),
            JPH::CollideShapeSettings(), JPH::RVec3::sZero(), collector, JPH::BroadPhaseLayerFilter(), anyCategory);
        uint32_t slotHits = 0;
        for (const JPH::CollideShapeResult& hit : collector.mHits)
        {
            slotHits += hit.mBodyID2.GetIndex() <= kMaxSimulatableSlots * kBodiesPerBrawler ? 1u : 0u;
        }
        return slotHits;
    };

    CHECK(countSlotHits() == 0u);

    world.applyOccupancy(occupancyOf({ 3 }));
    CHECK(countSlotHits() == kBodiesPerBrawler);

    world.applyOccupancy(BodySlotOccupancy{});
    CHECK(countSlotHits() == 0u);
}

TEST_CASE("JoltWorld.JoinAcrossARollbackReplaysByteIdentically", "[Jolt]")
{
    constexpr uint32_t kJoinTick = 100;
    constexpr uint32_t kJoiningSlot = 1;
    const auto occupancyAt = [](uint32_t tick)
    {
        return tick >= kJoinTick ? occupancyOf({ 0, 2, kJoiningSlot }) : occupancyOf({ 0, 2 });
    };
    const auto runTick = [&occupancyAt](JoltWorld& world, uint32_t tick)
    {
        world.applyOccupancy(occupancyAt(tick));
        if (tick == kJoinTick)
        {
            placeBrawler(world, kJoiningSlot, 0.3f, 0.2f, 1.4f);
        }
        applyScriptedForces(world, tick);
        world.step(kDt);
        world.saveTick(tick);
    };

    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(64), nullptr);
    addFloor(world);
    world.applyOccupancy(occupancyAt(0));
    placeBrawler(world, 0, -0.5f, 0.f, 1.0f);
    placeBrawler(world, 2, 0.9f, 0.4f, 1.2f);

    std::map<uint32_t, std::vector<uint8_t>> original;
    std::map<uint32_t, uint64_t> originalHash;
    for (uint32_t tick = 1; tick <= kJoinTick + 5u; ++tick)
    {
        runTick(world, tick);
        original[tick] = fullState(world);
        originalHash[tick] = world.liveStateHash();
        REQUIRE(world.stateHash(tick) == std::optional<uint64_t>(originalHash[tick]));
    }
    const JPH::BodyID joiningCapsule = world.slotBodyId(kJoiningSlot, kTemplateCapsule);
    REQUIRE(world.bodies().GetObjectLayer(joiningCapsule) == world.templateLayer(kTemplateCapsule));

    REQUIRE(world.restoreTick(kJoinTick - 5u));
    world.applyOccupancy(occupancyAt(kJoinTick - 5u));
    CHECK(sameBytes(fullState(world), original[kJoinTick - 5u]));
    CHECK(world.bodies().GetObjectLayer(joiningCapsule) == world.layers().parkedLayer());
    CHECK_FALSE(world.appliedOccupancy().occupied.test(kJoiningSlot));

    for (uint32_t tick = kJoinTick - 4u; tick <= kJoinTick + 5u; ++tick)
    {
        runTick(world, tick);
        INFO("replayed tick " << tick);
        CHECK(sameBytes(fullState(world), original[tick]));
        CHECK(world.stateHash(tick) == std::optional<uint64_t>(originalHash[tick]));
        const JPH::ObjectLayer expected = tick < kJoinTick ? world.layers().parkedLayer() : world.templateLayer(kTemplateCapsule);
        CHECK(world.bodies().GetObjectLayer(joiningCapsule) == expected);
    }
}

TEST_CASE("JoltWorld.ShapeEnableBitsRestoreWithTheTick", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(8), nullptr);
    CHECK(world.shapeEnables().isEnabled(ShapeId{ 3 }));

    world.shapeEnables().disable(ShapeId{ 3 });
    world.saveTick(10);
    world.shapeEnables().enable(ShapeId{ 3 });
    world.shapeEnables().disable(ShapeId{ 5 });
    world.saveTick(11);

    REQUIRE(world.restoreTick(10));
    CHECK_FALSE(world.shapeEnables().isEnabled(ShapeId{ 3 }));
    CHECK(world.shapeEnables().isEnabled(ShapeId{ 5 }));

    REQUIRE(world.restoreTick(11));
    CHECK(world.shapeEnables().isEnabled(ShapeId{ 3 }));
    CHECK_FALSE(world.shapeEnables().isEnabled(ShapeId{ 5 }));
}

TEST_CASE("JoltWorld.OccupancyRestoresWithTheTick", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(8), nullptr);
    world.applyOccupancy(occupancyOf({ 1, 4 }));
    world.saveTick(3);
    world.applyOccupancy(occupancyOf({ 4, 6 }));
    world.saveTick(4);

    REQUIRE(world.restoreTick(3));
    CHECK(world.appliedOccupancy() == occupancyOf({ 1, 4 }));
    CHECK(world.bodies().GetObjectLayer(world.slotBodyId(6, kTemplateCapsule)) == world.layers().parkedLayer());
    CHECK(world.bodies().GetObjectLayer(world.slotBodyId(1, kTemplateCapsule)) == world.templateLayer(kTemplateCapsule));
    CHECK(world.bodies().GetGravityFactor(world.slotBodyId(1, kTemplateCapsule)) == 1.f);
    CHECK(world.bodies().GetGravityFactor(world.slotBodyId(6, kTemplateCapsule)) == 0.f);
}

TEST_CASE("JoltWorld.RestoreOfAMismatchedBodySetIsRefusedAndLeavesTheWorldUntouched", "[Jolt]")
{
    RuntimeLease lease;
    std::vector<std::string> lines;
    JoltWorld world(lease.runtime, brawlerWorldConfig(16), [&lines](const char* line) { lines.emplace_back(line); });
    (void)runEightBrawlers(world, 10);
    simulateTick(world, 11);
    simulateTick(world, 12);

    const JPH::BodyID removed = world.slotBodyId(2, kTemplateCapsule);
    world.bodies().RemoveBody(removed);
    const std::vector<uint8_t> before = fullState(world);
    const JPH::RVec3 removedPosition = world.bodies().GetCenterOfMassPosition(removed);
    const JPH::Vec3 removedVelocity = world.bodies().GetLinearVelocity(removed);
    const BodySlotOccupancy occupancyBefore = world.appliedOccupancy();

    CHECK_FALSE(world.restoreTick(10));
    CHECK(sameBytes(fullState(world), before));
    CHECK(world.bodies().GetCenterOfMassPosition(removed) == removedPosition);
    CHECK(world.bodies().GetLinearVelocity(removed) == removedVelocity);
    CHECK(world.appliedOccupancy() == occupancyBefore);
    REQUIRE(lines.size() == 1u);
    CHECK(lines.front().find("restoreTick(10) refused") != std::string::npos);

    world.bodies().AddBody(removed, JPH::EActivation::Activate);
    CHECK(world.restoreTick(10));
}

TEST_CASE("JoltWorld.StateHashMatchesOnReplayAndSeesOneUlp", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(64), nullptr);
    const RecordedRun original = runEightBrawlers(world, 60);
    for (uint32_t tick = 1; tick <= 60; ++tick)
    {
        CHECK(original.hashAtTick[tick] != original.hashAtTick[tick - 1u]);
    }

    REQUIRE(world.restoreTick(30));
    CHECK(world.liveStateHash() == original.hashAtTick[30]);
    for (uint32_t tick = 31; tick <= 60; ++tick)
    {
        simulateTick(world, tick);
        INFO("tick " << tick);
        CHECK(world.stateHash(tick) == std::optional<uint64_t>(original.hashAtTick[tick]));
    }

    const uint64_t unchanged = world.liveStateHash();
    const JPH::BodyID capsule = world.slotBodyId(5, kTemplateCapsule);
    const JPH::RVec3 position = world.bodies().GetCenterOfMassPosition(capsule);
    const float nudgedX = std::nextafter(position.GetX(), 1.0e9f);
    world.bodies().SetPosition(capsule, JPH::RVec3(nudgedX, position.GetY(), position.GetZ()), JPH::EActivation::Activate);
    REQUIRE(world.bodies().GetCenterOfMassPosition(capsule).GetX() == nudgedX);
    REQUIRE(nudgedX != position.GetX());
    world.saveTick(61);
    CHECK(world.stateHash(61) != std::optional<uint64_t>(unchanged));
}

TEST_CASE("JoltWorld.StepLeavesTheCallersFpModeUnchanged", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(0), nullptr);
    const JoltFpMode before = readJoltFpMode();
    world.step(kDt);
    const JoltFpMode after = readJoltFpMode();
    CHECK(after.flushToZero == before.flushToZero);
    CHECK(after.denormalsAreZero == before.denormalsAreZero);
    CHECK(after.roundToNearest == before.roundToNearest);
#if defined(JPH_CPU_X86)
    constexpr uint64_t kMxcsrExceptionStatusFlags = 0x3Fu;
    CHECK((after.rawControlWord & ~kMxcsrExceptionStatusFlags) == (before.rawControlWord & ~kMxcsrExceptionStatusFlags));
#endif

    JoltFpMode inside;
    {
        JoltStepFpScope scope;
        inside = readJoltFpMode();
    }
    CHECK(isExpectedJoltStepFpMode(inside));
#if defined(JPH_CPU_X86)
    CHECK(inside.readable);
    CHECK(inside.flushToZero);
    CHECK(inside.denormalsAreZero);
    CHECK(inside.roundToNearest);
#endif
}

#endif // WITH_LOW_LEVEL_TESTS
