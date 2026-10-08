// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsFactory.h"
#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"
#include "OGSimulationJolt/JoltStaticWorldBuilder.h"

#include "glm/gtc/matrix_transform.hpp"
#include "OGSimulation/StaticGeometry.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <optional>
#include <random>
#include <string>
#include <vector>

using namespace joltTestRig;

namespace
{
    constexpr uint32_t kSlots = 8;
    constexpr uint32_t kTicks = 60;
    constexpr uint32_t kQueries = 200;
    constexpr uint32_t kRestoreSamples = 200;

    enum class StaticOrder
    {
        FloorFirst,
        ObstaclesFirst
    };

    glm::mat4 at(const glm::vec3& positionCm)
    {
        return glm::translate(glm::mat4(1.f), positionCm);
    }

    StaticShapeDescriptor staticElement(StaticShape shape, const glm::vec3& positionCm, uint64_t stableKey)
    {
        StaticShapeDescriptor descriptor;
        descriptor.shape = std::move(shape);
        descriptor.localToWorld = at(positionCm);
        descriptor.categories = CollisionCategories{ bit(kWorld) };
        descriptor.blockingCategories = CollisionCategories{ kAllMappedCategories };
        descriptor.friction = 0.7f;
        descriptor.restitution = 0.3f;
        descriptor.stableKey = stableKey;
        return descriptor;
    }

    StaticWorldDescription floorPart()
    {
        StaticWorldDescription description;
        description.shapes.push_back(staticElement(StaticBox{ glm::vec3(2000.f, 2000.f, 50.f) }, glm::vec3(0.f, 0.f, -50.f), 101));
        return description;
    }

    StaticWorldDescription obstaclePart()
    {
        StaticWorldDescription description;
        description.shapes.push_back(staticElement(StaticBox{ glm::vec3(50.f, 50.f, 50.f) }, glm::vec3(300.f, 0.f, 50.f), 102));
        description.shapes.push_back(staticElement(StaticSphere{ 40.f }, glm::vec3(-300.f, 0.f, 40.f), 103));
        description.shapes.push_back(staticElement(StaticCapsuleZ{ 30.f, 90.f }, glm::vec3(0.f, 300.f, 90.f), 104));
        StaticTriangleMesh ramp;
        ramp.vertices = { glm::vec3(-100.f, -100.f, 0.f), glm::vec3(100.f, -100.f, 0.f), glm::vec3(100.f, 100.f, 0.f), glm::vec3(-100.f, 100.f, 0.f) };
        ramp.indices = { 0, 1, 2, 0, 2, 3 };
        description.shapes.push_back(staticElement(ramp, glm::vec3(0.f, -400.f, 20.f), 105));
        return description;
    }

    std::vector<JoltLayerKey> arenaStaticLayers()
    {
        StaticWorldDescription whole = floorPart();
        for (const StaticShapeDescriptor& shape : obstaclePart().shapes)
        {
            whole.shapes.push_back(shape);
        }
        return staticLayerKeysOf(whole);
    }

    JoltWorldConfig stepWorldConfig(std::vector<SlotBodyTemplate> slotTemplate = brawlerSlotTemplate(), uint32_t slots = kSlots)
    {
        JoltWorldConfig config;
        config.simulatableSlots = slots;
        config.slotTemplate = std::move(slotTemplate);
        config.staticLayers = arenaStaticLayers();
        config.ringDepthTicks = 8;
        config.tempAllocatorBytes = 4u << 20;
        return config;
    }

    JoltWorldConfig shadowConfigOf(JoltWorldConfig config)
    {
        config.ringDepthTicks = 0;
        config.tempAllocatorBytes = 64u << 10;
        return config;
    }

    void logToNowhere(const char*)
    {
    }

    struct TwoWorldScene
    {
        std::vector<std::string> lines;
        JoltWorld world;
        JoltPhysicsBodyAdapter bodies;
        JoltSpatialQueryAdapter query;
        std::vector<JPH::BodyID> floorBodies;
        std::vector<JPH::BodyID> obstacleBodies;

        TwoWorldScene(JoltRuntime& runtime, const JoltWorldConfig& config, StaticOrder order)
            : world(runtime, config, [this](const char* line) { lines.emplace_back(line); })
            , bodies(world)
            , query(world, nullptr, JoltSpatialQueryConfig{ kAllMappedCategories })
        {
            JoltStaticWorldBuilder floorBuilder(world, &logToNowhere);
            JoltStaticWorldBuilder obstacleBuilder(world, &logToNowhere);
            if (order == StaticOrder::FloorFirst)
            {
                (void)floorBuilder.build(floorPart());
                (void)obstacleBuilder.build(obstaclePart());
            }
            else
            {
                (void)obstacleBuilder.build(obstaclePart());
                (void)floorBuilder.build(floorPart());
            }
            floorBodies = floorBuilder.stats().bodies;
            obstacleBodies = obstacleBuilder.stats().bodies;

            for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
            {
                JoltPhysicsFactory factory(bodies, config.slotTemplate, slot, 100u + slot, {},
                    [this](BodyId body, uint32_t shapeIndex, std::optional<BodyId> root) { return query.registerShape(body, shapeIndex, root); });
                for (const SlotBodyTemplate& slotTemplate : config.slotTemplate)
                {
                    (void)factory.createPhysicalObject(slotTemplate.descriptor, "declaration");
                }
            }
        }
    };

    uint32_t runStepWorld(TwoWorldScene& scene, uint32_t ticks)
    {
        CountingContactListener listener;
        scene.world.applyOccupancy(allOccupied());
        placeAllBrawlers(scene.world);
        scene.world.saveTick(0);
        for (uint32_t tick = 1; tick <= ticks; ++tick)
        {
            applyScriptedForces(scene.world, tick);
            if (tick == ticks)
            {
                scene.world.physics().SetContactListener(&listener);
            }
            scene.world.step(kDt);
            scene.world.saveTick(tick);
        }
        scene.world.physics().SetContactListener(nullptr);
        return listener.added + listener.persisted;
    }

    void giveTheShadowAState(JoltWorld& world)
    {
        world.applyOccupancy(occupancyOf({ 0, 1, 2 }));
        for (uint32_t slot = 0; slot < 3; ++slot)
        {
            for (uint32_t index = 0; index < world.bodiesPerSlot(); ++index)
            {
                const JPH::BodyID id = world.slotBodyId(slot, index);
                world.bodies().SetPosition(id, JPH::RVec3(0.5f * float(slot), 0.3f * float(index), 2.f), JPH::EActivation::Activate);
                world.bodies().SetLinearVelocity(id, JPH::Vec3(0.1f * float(index), -0.2f, 0.05f * float(slot)));
            }
        }
    }

    std::vector<QueryVolumeId> randomSphereVolumes(JoltSpatialQueryAdapter& query, uint32_t count)
    {
        std::mt19937 generator(5050u);
        std::uniform_real_distribution<float> horizontal(-500.f, 500.f);
        std::uniform_real_distribution<float> vertical(-20.f, 250.f);
        std::uniform_real_distribution<float> radius(20.f, 150.f);
        std::vector<QueryVolumeId> volumes;
        for (uint32_t index = 0; index < count; ++index)
        {
            const glm::vec3 centre(horizontal(generator), horizontal(generator), vertical(generator));
            const QueryVolumeDescriptor descriptor{ SphereGeometry{ radius(generator) }, CollisionCategories{ kAllMappedCategories }, glm::mat4(1.f), kQueryRouting };
            const QueryVolumeId volume = query.registerVolume(descriptor, BodyId{});
            query.setVolumeParentTransform(volume, at(centre));
            volumes.push_back(volume);
        }
        return volumes;
    }

    bool sameReport(const SpatialQueryReport& first, const SpatialQueryReport& second)
    {
        if (first.size() != second.size())
        {
            return false;
        }
        for (size_t index = 0; index < first.size(); ++index)
        {
            const SpatialQueryHit& a = first[index];
            const SpatialQueryHit& b = second[index];
            if (a.bodyId != b.bodyId || a.rootBodyId != b.rootBodyId || a.objectCategories.bits != b.objectCategories.bits
                || a.objectPosition != b.objectPosition)
            {
                return false;
            }
        }
        return true;
    }

    size_t linesContaining(const std::vector<std::string>& lines, const std::string& text)
    {
        return size_t(std::count_if(lines.begin(), lines.end(), [&text](const std::string& line) { return line.find(text) != std::string::npos; }));
    }

    struct Refusal
    {
        bool restored = true;
        bool hashUnchanged = false;
        bool bytesUnchanged = false;
        bool occupancyUnchanged = false;
        size_t refusalLines = 0;
        size_t allLines = 0;
    };

    Refusal restoreIntoAMismatchedShadow(JoltRuntime& runtime, const JoltStateSlot& slot, const JoltWorldConfig& shadowConfig)
    {
        TwoWorldScene shadow(runtime, shadowConfig, StaticOrder::FloorFirst);
        giveTheShadowAState(shadow.world);
        const uint64_t hashBefore = shadow.world.liveStateHash();
        const std::vector<uint8_t> bytesBefore = fullState(shadow.world);
        const BodySlotOccupancy occupancyBefore = shadow.world.appliedOccupancy();

        Refusal refusal;
        refusal.restored = shadow.world.restoreFromSnapshot(slot);
        refusal.hashUnchanged = shadow.world.liveStateHash() == hashBefore;
        refusal.bytesUnchanged = sameBytes(fullState(shadow.world), bytesBefore);
        refusal.occupancyUnchanged = shadow.world.appliedOccupancy() == occupancyBefore;
        refusal.refusalLines = linesContaining(shadow.lines, "[Warning] JoltWorld: restoreFromSnapshot(" + std::to_string(slot.tick) + ") refused");
        refusal.allLines = shadow.lines.size();
        return refusal;
    }
}

TEST_CASE("JoltShadowWorld.RestoreFromTheStepWorldsNewestSlotMatchesItsStateAndItsQueries", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorldAccessScope access;
    TwoWorldScene step(lease.runtime, stepWorldConfig(), StaticOrder::FloorFirst);
    TwoWorldScene shadow(lease.runtime, shadowConfigOf(stepWorldConfig()), StaticOrder::FloorFirst);
    REQUIRE(step.floorBodies == shadow.floorBodies);
    REQUIRE(step.obstacleBodies == shadow.obstacleBodies);

    const uint32_t contacts = runStepWorld(step, kTicks);
    INFO("the step world must hold contacts in its newest slot");
    REQUIRE(contacts > 0u);
    const JoltStateSlot* newest = step.world.ring().find(kTicks);
    REQUIRE(newest != nullptr);

    const std::vector<QueryVolumeId> stepVolumes = randomSphereVolumes(step.query, kQueries);
    const std::vector<QueryVolumeId> shadowVolumes = randomSphereVolumes(shadow.query, kQueries);
    REQUIRE(stepVolumes == shadowVolumes);

    uint32_t differBeforeRestore = 0;
    for (uint32_t index = 0; index < kQueries; ++index)
    {
        differBeforeRestore += sameReport(step.query.overlap({ stepVolumes[index] }), shadow.query.overlap({ shadowVolumes[index] })) ? 0u : 1u;
    }
    REQUIRE(differBeforeRestore > 0u);
    REQUIRE(shadow.world.liveStateHash() != step.world.liveStateHash());

    REQUIRE(shadow.world.restoreFromSnapshot(*newest));
    CHECK(shadow.lines.empty());
    CHECK(shadow.world.liveStateHash() == step.world.liveStateHash());
    CHECK(shadow.world.liveStateHash() == newest->stateHash);
    CHECK(shadow.world.appliedOccupancy() == step.world.appliedOccupancy());
    CHECK(shadow.world.shapeEnables().bits() == step.world.shapeEnables().bits());
    CHECK(shadow.world.physicsStepCount() == 0u);
    CHECK(shadow.world.ring().heldTickCount() == 0u);

    uint32_t identical = 0;
    uint32_t withSlotBodyHits = 0;
    for (uint32_t index = 0; index < kQueries; ++index)
    {
        const SpatialQueryReport stepHits = step.query.overlap({ stepVolumes[index] });
        const SpatialQueryReport shadowHits = shadow.query.overlap({ shadowVolumes[index] });
        INFO("query " << index);
        CHECK(sameReport(stepHits, shadowHits));
        identical += sameReport(stepHits, shadowHits) ? 1u : 0u;
        const bool hitsASlotBody = std::any_of(stepHits.begin(), stepHits.end(),
            [&step](const SpatialQueryHit& hit) {
                const std::optional<JoltQueryHitKey> key = step.query.hitKeyOf(hit.bodyId);
                return key.has_value() && key->isStatic == 0u;
            });
        withSlotBodyHits += hitsASlotBody ? 1u : 0u;
    }
    CHECK(identical == kQueries);
    CHECK(withSlotBodyHits >= 20u);

    std::vector<double> samplesMs;
    samplesMs.reserve(kRestoreSamples);
    for (uint32_t sample = 0; sample < kRestoreSamples; ++sample)
    {
        const JoltStateSlot* slot = step.world.ring().find(kTicks - (sample % 2u));
        REQUIRE(slot != nullptr);
        const auto start = std::chrono::steady_clock::now();
        const bool restored = shadow.world.restoreFromSnapshot(*slot);
        const auto stop = std::chrono::steady_clock::now();
        REQUIRE(restored);
        REQUIRE(shadow.world.liveStateHash() == slot->stateHash);
        samplesMs.push_back(std::chrono::duration<double, std::milli>(stop - start).count());
    }
    std::sort(samplesMs.begin(), samplesMs.end());
    std::printf("[JoltShadowWorld] restoreFromSnapshot cost, %u occupied slots, %u bytes, %u samples, ms: median=%.4f p95=%.4f max=%.4f\n",
        kSlots, newest->byteCount, kRestoreSamples, samplesMs[samplesMs.size() / 2u], samplesMs[samplesMs.size() * 95u / 100u], samplesMs.back());
    CHECK(samplesMs[samplesMs.size() / 2u] < 1.0);
    CHECK(shadow.lines.empty());
}

TEST_CASE("JoltShadowWorld.RestoreFromSnapshotTakesNoUndoSaveAndWritesNoRingSlot", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorldAccessScope access;
    TwoWorldScene step(lease.runtime, stepWorldConfig(), StaticOrder::FloorFirst);
    (void)runStepWorld(step, 20);
    const JoltStateSlot* newest = step.world.ring().find(20);
    REQUIRE(newest != nullptr);

    JoltWorldConfig tinySlots = shadowConfigOf(stepWorldConfig());
    tinySlots.ringDepthTicks = 4;
    tinySlots.ringSlotBytes = 64;
    INFO("a 64-byte slot holds no snapshot, so an undo save (or any ring write) would log 'does not fit' and fail its OG_CHECK");
    REQUIRE(newest->byteCount > tinySlots.ringSlotBytes);
    TwoWorldScene shadow(lease.runtime, tinySlots, StaticOrder::FloorFirst);

    REQUIRE(shadow.world.restoreFromSnapshot(*newest));
    CHECK(shadow.world.liveStateHash() == newest->stateHash);
    CHECK(shadow.lines.empty());
    CHECK(shadow.world.ring().heldTickCount() == 0u);
    CHECK_FALSE(shadow.world.oldestHeldTick().has_value());
    CHECK_FALSE(shadow.world.hasTick(20));
}

TEST_CASE("JoltShadowWorld.ADifferentSlotBodySetIsRefusedWithAWarningAndLeavesTheShadowUnchanged", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorldAccessScope access;
    TwoWorldScene step(lease.runtime, stepWorldConfig(), StaticOrder::FloorFirst);
    (void)runStepWorld(step, kTicks);
    const JoltStateSlot* newest = step.world.ring().find(kTicks);
    REQUIRE(newest != nullptr);

    SECTION("one slot fewer")
    {
        const Refusal refusal = restoreIntoAMismatchedShadow(lease.runtime, *newest, shadowConfigOf(stepWorldConfig(brawlerSlotTemplate(), kSlots - 1u)));
        CHECK_FALSE(refusal.restored);
        CHECK(refusal.refusalLines == 1u);
        CHECK(refusal.allLines == 1u);
        CHECK(refusal.hashUnchanged);
        CHECK(refusal.bytesUnchanged);
        CHECK(refusal.occupancyUnchanged);
    }

    SECTION("one template body fewer")
    {
        std::vector<SlotBodyTemplate> fewer = brawlerSlotTemplate();
        fewer.erase(fewer.begin() + kTemplateProjectileFirst + 2);
        REQUIRE(fewer.size() == kBodiesPerBrawler - 1u);
        const Refusal refusal = restoreIntoAMismatchedShadow(lease.runtime, *newest, shadowConfigOf(stepWorldConfig(fewer)));
        CHECK_FALSE(refusal.restored);
        CHECK(refusal.refusalLines == 1u);
        CHECK(refusal.allLines == 1u);
        CHECK(refusal.hashUnchanged);
        CHECK(refusal.bytesUnchanged);
        CHECK(refusal.occupancyUnchanged);
    }

    SECTION("the same body set, as a control")
    {
        const Refusal control = restoreIntoAMismatchedShadow(lease.runtime, *newest, shadowConfigOf(stepWorldConfig()));
        CHECK(control.restored);
        CHECK(control.refusalLines == 0u);
        CHECK(control.allLines == 0u);
        CHECK_FALSE(control.hashUnchanged);
    }
}

TEST_CASE("JoltShadowWorld.StaticsInADifferentOrderAreNotRefusedTheStaticCoverageLimit", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorldAccessScope access;
    TwoWorldScene step(lease.runtime, stepWorldConfig(), StaticOrder::FloorFirst);
    TwoWorldScene reordered(lease.runtime, shadowConfigOf(stepWorldConfig()), StaticOrder::ObstaclesFirst);
    INFO("the two worlds' statics must really differ, or this case pins nothing");
    REQUIRE(step.floorBodies.size() == reordered.floorBodies.size());
    REQUIRE(step.floorBodies != reordered.floorBodies);
    REQUIRE(step.obstacleBodies != reordered.obstacleBodies);

    const uint32_t contacts = runStepWorld(step, kTicks);
    REQUIRE(contacts > 0u);
    const JoltStateSlot* newest = step.world.ring().find(kTicks);
    REQUIRE(newest != nullptr);

    CHECK(reordered.world.restoreFromSnapshot(*newest));
    CHECK(reordered.lines.empty());
    CHECK(reordered.world.liveStateHash() == newest->stateHash);
}

TEST_CASE("JoltShadowWorld.AnEmptySlotIsRefused", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorldAccessScope access;
    TwoWorldScene shadow(lease.runtime, shadowConfigOf(stepWorldConfig()), StaticOrder::FloorFirst);
    giveTheShadowAState(shadow.world);
    const uint64_t hashBefore = shadow.world.liveStateHash();

    const JoltStateSlot empty;
    CHECK_FALSE(shadow.world.restoreFromSnapshot(empty));
    CHECK(shadow.world.liveStateHash() == hashBefore);
    REQUIRE(shadow.lines.size() == 1u);
    CHECK(shadow.lines.front().find("restoreFromSnapshot refused: the slot holds no snapshot") != std::string::npos);
}

#endif // WITH_LOW_LEVEL_TESTS
