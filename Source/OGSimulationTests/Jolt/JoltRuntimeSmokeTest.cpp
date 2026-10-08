// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulationJolt/JoltRuntime.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <algorithm>
#include <string>
#include <vector>

namespace
{
    std::vector<std::string> g_joltSmokeLog;

    void captureLog(const char* message)
    {
        g_joltSmokeLog.emplace_back(message);
    }

    struct RuntimeLease
    {
        RuntimeLease() { (void)JoltRuntime::acquire(&captureLog); }
        ~RuntimeLease() { JoltRuntime::release(); }
        RuntimeLease(const RuntimeLease&) = delete;
        RuntimeLease& operator=(const RuntimeLease&) = delete;
    };

    constexpr JPH::ObjectLayer kMovingLayer = 0;
    constexpr JPH::BroadPhaseLayer kMovingBroadPhaseLayer(0);

    struct OneLayerWorld
    {
        JPH::BroadPhaseLayerInterfaceTable broadPhaseLayers{ 1, 1 };
        JPH::ObjectLayerPairFilterTable objectLayerPairs{ 1 };
        JPH::ObjectVsBroadPhaseLayerFilterTable objectVsBroadPhase;
        JPH::TempAllocatorImpl tempAllocator{ 1024 * 1024 };
        JPH::JobSystemSingleThreaded jobSystem{ JPH::cMaxPhysicsJobs };
        JPH::PhysicsSystem physics;

        OneLayerWorld()
            : objectVsBroadPhase(initLayers(broadPhaseLayers, objectLayerPairs), 1, objectLayerPairs, 1)
        {
            physics.Init(16, 0, 16, 16, broadPhaseLayers, objectVsBroadPhase, objectLayerPairs);
        }

        static const JPH::BroadPhaseLayerInterfaceTable& initLayers(
            JPH::BroadPhaseLayerInterfaceTable& layers, JPH::ObjectLayerPairFilterTable& pairs)
        {
            layers.MapObjectToBroadPhaseLayer(kMovingLayer, kMovingBroadPhaseLayer);
            pairs.EnableCollision(kMovingLayer, kMovingLayer);
            return layers;
        }

        JPH::BodyID addFallingSphere(float height)
        {
            JPH::BodyCreationSettings settings(new JPH::SphereShape(0.5f), JPH::RVec3(0.0f, height, 0.0f),
                JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, kMovingLayer);
            return physics.GetBodyInterfaceNoLock().CreateAndAddBody(settings, JPH::EActivation::Activate);
        }

        JPH::EPhysicsUpdateError stepOnce()
        {
            return physics.Update(1.0f / 60.0f, 1, &tempAllocator, &jobSystem);
        }
    };
}

TEST_CASE("JoltRuntime.SmokeAcquireCreateStepRelease", "[Jolt]")
{
    const uint32_t before = JoltRuntime::referenceCount();
    g_joltSmokeLog.clear();

    {
        RuntimeLease lease;
        REQUIRE(JoltRuntime::referenceCount() == before + 1);

        OneLayerWorld world;
        const JPH::BodyID sphere = world.addFallingSphere(10.0f);
        REQUIRE_FALSE(sphere.IsInvalid());

        REQUIRE(world.stepOnce() == JPH::EPhysicsUpdateError::None);

        JPH::BodyInterface& bodies = world.physics.GetBodyInterfaceNoLock();
        CHECK(bodies.GetLinearVelocity(sphere).GetY() < 0.0f);
        CHECK(bodies.GetPosition(sphere).GetY() < 10.0f);

        bodies.RemoveBody(sphere);
        bodies.DestroyBody(sphere);
    }

    CHECK(JoltRuntime::referenceCount() == before);

    if (before == 0)
    {
        REQUIRE(g_joltSmokeLog.size() >= 2);
        CHECK(g_joltSmokeLog.front().find("JoltRuntime: Jolt 5.6.0 initialised") == 0);
        CHECK(g_joltSmokeLog.back() == "JoltRuntime: Jolt shut down");
    }
}

TEST_CASE("JoltRuntime.TwoAcquiresNeedTwoReleases", "[Jolt]")
{
    const uint32_t before = JoltRuntime::referenceCount();

    {
        RuntimeLease firstWorld;
        {
            RuntimeLease secondWorld;
            REQUIRE(JoltRuntime::referenceCount() == before + 2);
        }
        REQUIRE(JoltRuntime::referenceCount() == before + 1);

        OneLayerWorld world;
        const JPH::BodyID sphere = world.addFallingSphere(5.0f);
        CHECK(world.stepOnce() == JPH::EPhysicsUpdateError::None);
        world.physics.GetBodyInterfaceNoLock().RemoveBody(sphere);
        world.physics.GetBodyInterfaceNoLock().DestroyBody(sphere);
    }
    CHECK(JoltRuntime::referenceCount() == before);

    {
        RuntimeLease reacquired;
        OneLayerWorld world;
        const JPH::BodyID sphere = world.addFallingSphere(5.0f);
        CHECK(world.stepOnce() == JPH::EPhysicsUpdateError::None);
        world.physics.GetBodyInterfaceNoLock().RemoveBody(sphere);
        world.physics.GetBodyInterfaceNoLock().DestroyBody(sphere);
    }
    CHECK(JoltRuntime::referenceCount() == before);
}

TEST_CASE("JoltRuntime.AcquireReturnsOneSharedRuntime", "[Jolt]")
{
    RuntimeLease firstWorld;
    JoltRuntime& first = JoltRuntime::acquire(&captureLog);
    JoltRuntime& second = JoltRuntime::acquire(&captureLog);
    CHECK(&first == &second);
    JoltRuntime::release();
    JoltRuntime::release();
}

TEST_CASE("JoltRuntime.TraceReachesTheInjectedLoggerUntilRelease", "[Jolt]")
{
    if (JoltRuntime::referenceCount() != 0)
    {
        SKIP("another holder installed the logger first");
    }

    const JPH::TraceFunction traceBefore = JPH::Trace;
    g_joltSmokeLog.clear();
    {
        RuntimeLease lease;
        CHECK(JPH::Trace != traceBefore);
        JPH::Trace("trace %d from %s", 42, "jolt");
    }

    CHECK(std::find(g_joltSmokeLog.begin(), g_joltSmokeLog.end(), std::string("trace 42 from jolt")) != g_joltSmokeLog.end());
    CHECK(JPH::Trace == traceBefore);
}

#endif // WITH_LOW_LEVEL_TESTS
