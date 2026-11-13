// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltBodyUserData.h"
#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsBodyReaderAdapter.h"
#include "OGSimulationJolt/JoltWorld.h"

#include "glm/ext/matrix_transform.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"
#include "OGSimulation/BodyId.h"
#include "OGSimulation/PhysicsBodyState.h"

#include <cstdint>
#include <thread>

using namespace joltTestRig;

namespace
{
    thread_local bool t_worldAccessAllowed = false;

    bool worldAccessAllowedOnThisThread()
    {
        return t_worldAccessAllowed;
    }

    struct WorldAccessAllowedScope
    {
        WorldAccessAllowedScope() { t_worldAccessAllowed = true; }
        ~WorldAccessAllowedScope() { t_worldAccessAllowed = false; }
        WorldAccessAllowedScope(const WorldAccessAllowedScope&) = delete;
        WorldAccessAllowedScope& operator=(const WorldAccessAllowedScope&) = delete;
    };

    JoltWorldConfig checkedWorldConfig()
    {
        JoltWorldConfig config = brawlerWorldConfig(2);
        config.accessCheck = &worldAccessAllowedOnThisThread;
        return config;
    }

    bool callerMayAccessOnAnotherThread(const JoltWorld& world)
    {
        bool mayAccess = false;
        std::thread probe([&world, &mayAccess] { mayAccess = world.callerMayAccess(); });
        probe.join();
        return mayAccess;
    }

    constexpr uint32_t kAccessCheckSimulatableId = 7;
}

TEST_CASE("JoltWorld.AccessCheck.WithoutACheckEveryCallerMayAccess", "[Jolt]")
{
    STATIC_REQUIRE(JoltWorldConfig{}.accessCheck == nullptr);
    CHECK(brawlerWorldConfig(2).accessCheck == nullptr);

    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerWorldConfig(2), nullptr);
    CHECK(world.callerMayAccess());
    CHECK(callerMayAccessOnAnotherThread(world));

    JoltPhysicsBodyAdapter adapter(world);
    CHECK(adapter.world().callerMayAccess());
}

TEST_CASE("JoltWorld.AccessCheck.TheInstalledPredicateDecidesForTheWorldAndBothAdapters", "[Jolt]")
{
    REQUIRE_FALSE(t_worldAccessAllowed);
    RuntimeLease lease;
    JoltWorld world(lease.runtime, checkedWorldConfig(), nullptr);
    JoltPhysicsBodyAdapter adapter(world);
    const JoltPhysicsBodyReaderAdapter reader(world, adapter.bindTable());
    const BodyId body = JoltPhysicsBodyAdapter::bodyIdOf(world.slotBodyId(0, kTemplateRadial));
    adapter.bindTable().bind(body, JoltBodyBinding{ kAccessCheckSimulatableId, 0 }, glm::vec3(0.f));

    CHECK_FALSE(world.callerMayAccess());
    CHECK_FALSE(adapter.world().callerMayAccess());
    CHECK(world.simulatableSlots() == kMaxSimulatableSlots);
    CHECK(world.bodiesPerSlot() == kBodiesPerBrawler);
    CHECK(adapter.isBodyResolvable(body));
    CHECK(reader.isBodyResolvable(body));

    const glm::vec3 position(0.5f, 1.25f, -0.75f);
    {
        const WorldAccessAllowedScope allowed;
        CHECK(world.callerMayAccess());
        CHECK(adapter.world().callerMayAccess());
        CHECK_FALSE(callerMayAccessOnAnotherThread(world));

        world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 0 }));
        adapter.setBodyTransform(body, glm::translate(glm::mat4(1.f), position));
        const glm::mat4 written = adapter.getBodyTransform(body);
        CHECK(glm::vec3(written[3]) == position);
        CHECK(glm::vec3(reader.getBodyTransform(body)[3]) == position);
        CHECK(reader.captureBodyState(body).position == position);
        CHECK(reader.getBodyMass(body) > 0.f);

        world.step(kDt);
        world.saveTick(1);
        CHECK(world.physicsStepCount() == 1u);
        CHECK(world.hasTick(1));
    }

    CHECK_FALSE(world.callerMayAccess());
    CHECK_FALSE(t_worldAccessAllowed);
}

TEST_CASE("JoltWorld.AccessCheck.SetBodyTransformOffTheOwnerFiresTheCheck", "[.][JoltWorldAccessCheckFires]")
{
    REQUIRE_FALSE(t_worldAccessAllowed);
    RuntimeLease lease;
    JoltWorld world(lease.runtime, checkedWorldConfig(), nullptr);
    JoltPhysicsBodyAdapter adapter(world);
    REQUIRE_FALSE(world.callerMayAccess());

    adapter.setBodyTransform(JoltPhysicsBodyAdapter::bodyIdOf(world.slotBodyId(0, kTemplateRadial)), glm::mat4(1.f));
}

#endif
