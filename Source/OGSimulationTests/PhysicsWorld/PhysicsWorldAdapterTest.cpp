// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <initializer_list>
#include <optional>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/PhysicsWorldAdapter.h"
#include "../Mocks/MockPhysicsWorld.h"

namespace
{
BodySlotOccupancy physicsWorldOccupancyOf(std::initializer_list<uint32_t> slots)
{
    BodySlotOccupancy occupancy;
    for (const uint32_t slot : slots)
    {
        occupancy.occupied.set(slot);
    }
    return occupancy;
}

void physicsWorldSaveTicks(MockPhysicsWorld& world, SimTick first, SimTick last)
{
    for (SimTick tick = first; tick <= last; ++tick)
    {
        world.setLiveValue(static_cast<int64_t>(tick) * 10);
        world.saveTick(tick);
    }
}
}

TEST_CASE("PhysicsWorld concept is modelled by the mock and declares no snapshot coverage", "[PhysicsWorld]")
{
    STATIC_REQUIRE(PhysicsWorldAdapter<MockPhysicsWorld>);
    STATIC_REQUIRE(StaticWorldBuilder<MockStaticWorldBuilder>);
    STATIC_REQUIRE(!PhysicsWorldAdapter<physicsWorldAdapterSelfCheck::MissingInvalidateAllTicks>);
    STATIC_REQUIRE(!MockPhysicsWorld::coverage.bodySetAndIds);
    STATIC_REQUIRE(!MockPhysicsWorld::coverage.broadphase);
    STATIC_REQUIRE(!MockPhysicsWorld::coverage.staticShapes);
    STATIC_REQUIRE(kMaxSimulatableSlots == 8);
    STATIC_REQUIRE(BodySlotOccupancy{}.occupied.size() == kMaxSimulatableSlots);
}

TEST_CASE("PhysicsWorld ring evicts the oldest held tick once it is full", "[PhysicsWorld]")
{
    MockPhysicsWorld world(3);
    physicsWorldSaveTicks(world, 1, 3);
    REQUIRE(world.heldTickCount() == 3);
    REQUIRE(world.oldestHeldTick() == std::optional<SimTick>(1));

    world.setLiveValue(40);
    world.saveTick(4);

    CHECK(world.heldTickCount() == 3);
    CHECK_FALSE(world.hasTick(1));
    CHECK(world.hasTick(2));
    CHECK(world.hasTick(3));
    CHECK(world.hasTick(4));
    CHECK(world.oldestHeldTick() == std::optional<SimTick>(2));
}

TEST_CASE("PhysicsWorld restoreTick is false after eviction and leaves the world untouched", "[PhysicsWorld]")
{
    MockPhysicsWorld world(2);
    physicsWorldSaveTicks(world, 1, 3);
    world.applyOccupancy(physicsWorldOccupancyOf({ 5 }));
    world.setLiveValue(999);
    const MockWorldState before = world.live();

    CHECK_FALSE(world.restoreTick(1));
    CHECK(world.live() == before);

    REQUIRE(world.restoreTick(2));
    CHECK(world.live().value == 20);
    CHECK(world.live().occupancy == BodySlotOccupancy{});
}

TEST_CASE("PhysicsWorld saveTick overwrites a held tick without evicting another", "[PhysicsWorld]")
{
    MockPhysicsWorld world(3);
    physicsWorldSaveTicks(world, 1, 3);

    world.setLiveValue(777);
    world.applyOccupancy(physicsWorldOccupancyOf({ 0, 7 }));
    world.saveTick(2);

    CHECK(world.heldTickCount() == 3);
    CHECK(world.hasTick(1));
    CHECK(world.hasTick(3));
    REQUIRE(world.heldState(2).has_value());
    CHECK(world.heldState(2)->value == 777);
    CHECK(world.heldState(2)->occupancy == physicsWorldOccupancyOf({ 0, 7 }));
    CHECK(world.heldState(1)->value == 10);
}

TEST_CASE("PhysicsWorld a tick older than a full ring is not held", "[PhysicsWorld]")
{
    MockPhysicsWorld world(3);
    physicsWorldSaveTicks(world, 10, 12);

    world.saveTick(5);

    CHECK_FALSE(world.hasTick(5));
    CHECK(world.heldTickCount() == 3);
    CHECK(world.oldestHeldTick() == std::optional<SimTick>(10));
}

TEST_CASE("PhysicsWorld invalidateAllTicks empties the ring", "[PhysicsWorld]")
{
    MockPhysicsWorld world(4);
    physicsWorldSaveTicks(world, 1, 4);

    world.invalidateAllTicks();

    CHECK(world.heldTickCount() == 0);
    CHECK(world.oldestHeldTick() == std::nullopt);
    for (SimTick tick = 1; tick <= 4; ++tick)
    {
        CHECK_FALSE(world.hasTick(tick));
        CHECK_FALSE(world.stateHash(tick).has_value());
        CHECK_FALSE(world.restoreTick(tick));
    }

    world.saveTick(9);
    CHECK(world.oldestHeldTick() == std::optional<SimTick>(9));
}

TEST_CASE("PhysicsWorld commitScratch makes the tick held with the scratch value", "[PhysicsWorld]")
{
    MockPhysicsWorld world(4);
    world.setLiveValue(50);
    world.applyOccupancy(physicsWorldOccupancyOf({ 2 }));

    world.saveScratch();
    world.step(1.f / 60.f);
    world.applyOccupancy(physicsWorldOccupancyOf({ 2, 3 }));
    REQUIRE(world.live().value == 51);
    REQUIRE_FALSE(world.hasTick(6));

    world.commitScratch(6);

    REQUIRE(world.hasTick(6));
    CHECK(world.heldState(6) == std::optional<MockWorldState>(MockWorldState{ 50, physicsWorldOccupancyOf({ 2 }) }));
    CHECK(world.live().value == 51);

    REQUIRE(world.restoreTick(6));
    CHECK(world.live().value == 50);
    CHECK(world.live().occupancy == physicsWorldOccupancyOf({ 2 }));
}

TEST_CASE("PhysicsWorld commitScratch obeys overwrite and eviction like saveTick", "[PhysicsWorld]")
{
    MockPhysicsWorld world(2);
    physicsWorldSaveTicks(world, 1, 2);

    world.setLiveValue(-3);
    world.saveScratch();
    world.commitScratch(2);
    CHECK(world.heldTickCount() == 2);
    CHECK(world.heldState(2)->value == -3);

    world.commitScratch(3);
    CHECK_FALSE(world.hasTick(1));
    CHECK(world.heldState(3)->value == -3);
}

TEST_CASE("PhysicsWorld applyOccupancy is recorded in call order", "[PhysicsWorld]")
{
    MockPhysicsWorld world(4);
    const BodySlotOccupancy joined = physicsWorldOccupancyOf({ 1 });
    const BodySlotOccupancy both = physicsWorldOccupancyOf({ 1, 4 });

    world.saveScratch();
    world.applyOccupancy(joined);
    world.step(0.5f);
    world.saveTick(7);
    REQUIRE(world.restoreTick(7));
    world.applyOccupancy(both);
    world.step(0.25f);
    world.commitScratch(8);
    world.invalidateAllTicks();

    const std::vector<MockWorldCallKind> expected = {
        MockWorldCallKind::SaveScratch,
        MockWorldCallKind::ApplyOccupancy,
        MockWorldCallKind::Step,
        MockWorldCallKind::SaveTick,
        MockWorldCallKind::RestoreTick,
        MockWorldCallKind::ApplyOccupancy,
        MockWorldCallKind::Step,
        MockWorldCallKind::CommitScratch,
        MockWorldCallKind::InvalidateAllTicks,
    };
    const std::vector<MockWorldCall>& calls = world.calls();
    REQUIRE(calls.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i)
    {
        INFO("call " << i);
        CHECK(calls[i].kind == expected[i]);
    }
    CHECK(calls[1].occupancy == joined);
    CHECK(calls[5].occupancy == both);
    CHECK(calls[2].dt == 0.5f);
    CHECK(calls[6].dt == 0.25f);
    CHECK(calls[3].tick == 7);
    CHECK(calls[4].tick == 7);
    CHECK(calls[4].result);
    CHECK(calls[7].tick == 8);

    const std::vector<MockWorldCall> occupancyCalls = world.callsOf(MockWorldCallKind::ApplyOccupancy);
    REQUIRE(occupancyCalls.size() == 2);
    CHECK(occupancyCalls[0].occupancy == joined);
    CHECK(occupancyCalls[1].occupancy == both);
}

TEST_CASE("PhysicsWorld const queries are recorded too", "[PhysicsWorld]")
{
    MockPhysicsWorld world(2);
    world.saveTick(3);
    world.clearCalls();

    const MockPhysicsWorld& view = world;
    CHECK(view.hasTick(3));
    CHECK(view.oldestHeldTick() == std::optional<SimTick>(3));
    CHECK(view.stateHash(4) == std::nullopt);

    REQUIRE(world.calls().size() == 3);
    CHECK(world.calls()[0].kind == MockWorldCallKind::HasTick);
    CHECK(world.calls()[0].result);
    CHECK(world.calls()[1].kind == MockWorldCallKind::OldestHeldTick);
    CHECK(world.calls()[2].kind == MockWorldCallKind::StateHash);
    CHECK_FALSE(world.calls()[2].result);
}

TEST_CASE("PhysicsWorld stateHash is empty for an unheld tick and the held value after saveTick", "[PhysicsWorld]")
{
    MockPhysicsWorld world(3);
    CHECK(world.stateHash(1) == std::nullopt);

    world.setLiveValue(123);
    world.applyOccupancy(physicsWorldOccupancyOf({ 6 }));
    world.saveTick(1);

    const MockWorldState held{ 123, physicsWorldOccupancyOf({ 6 }) };
    REQUIRE(world.stateHash(1).has_value());
    CHECK(*world.stateHash(1) == MockPhysicsWorld::hashOf(held));
    CHECK(world.stateHash(2) == std::nullopt);

    world.setLiveValue(456);
    CHECK(*world.stateHash(1) == MockPhysicsWorld::hashOf(held));

    world.saveTick(1);
    CHECK(*world.stateHash(1) == MockPhysicsWorld::hashOf(MockWorldState{ 456, physicsWorldOccupancyOf({ 6 }) }));
    CHECK(*world.stateHash(1) != MockPhysicsWorld::hashOf(held));

    world.applyOccupancy(physicsWorldOccupancyOf({}));
    world.saveTick(2);
    CHECK(*world.stateHash(2) != *world.stateHash(1));
}

TEST_CASE("PhysicsWorld a depth-0 authority world holds no tick", "[PhysicsWorld]")
{
    MockPhysicsWorld world(0);
    world.saveTick(1);
    world.saveScratch();
    world.commitScratch(2);

    CHECK(world.heldTickCount() == 0);
    CHECK_FALSE(world.hasTick(1));
    CHECK_FALSE(world.hasTick(2));
    CHECK(world.oldestHeldTick() == std::nullopt);
    CHECK_FALSE(world.restoreTick(1));
    CHECK(world.stateHash(1) == std::nullopt);
}

#endif // WITH_LOW_LEVEL_TESTS
