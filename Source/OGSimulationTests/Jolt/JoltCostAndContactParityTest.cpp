// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltRollbackSuiteRig.h"

#include <Jolt/Physics/PhysicsSettings.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
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

#endif // WITH_LOW_LEVEL_TESTS
