// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsFactory.h"
#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"
#include "OGSimulationJolt/JoltStaticWorldBuilder.h"

#include "glm/gtc/matrix_transform.hpp"
#include "OGSimulation/SimulationFrame.h"
#include "OGSimulation/SpatialQueryAdapter.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <random>
#include <string>
#include <vector>

using namespace joltTestRig;

namespace
{
    constexpr uint32_t kRadialDeclaration = 0;
    constexpr uint32_t kGuardDeclaration = 1;
    constexpr uint32_t kProjectileDeclaration = 2;
    constexpr uint32_t kCapsuleDeclaration = 5;

    constexpr uint32_t kUnmappedCategory = 9;

    JoltWorldConfig queryWorldConfig(uint32_t ringDepthTicks = 0, std::vector<JoltLayerKey> staticLayers = { kWorldStaticKey })
    {
        JoltWorldConfig config;
        config.slotTemplate = brawlerSlotTemplate();
        config.staticLayers = std::move(staticLayers);
        config.ringDepthTicks = ringDepthTicks;
        config.tempAllocatorBytes = 4u << 20;
        return config;
    }

    struct QueryWorld
    {
        std::vector<std::string> logLines;
        JoltWorld world;
        JoltPhysicsBodyAdapter bodies;
        JoltSpatialQueryAdapter query;

        explicit QueryWorld(JoltRuntime& runtime, const JoltWorldConfig& config = queryWorldConfig())
            : world(runtime, config, nullptr)
            , bodies(world)
            , query(world, [this](const char* line) { logLines.emplace_back(line); }, JoltSpatialQueryConfig{ kAllMappedCategories })
        {
        }

        size_t linesContaining(const char* tag) const
        {
            return size_t(std::count_if(logLines.begin(), logLines.end(),
                [tag](const std::string& line) { return line.find(tag) != std::string::npos; }));
        }
    };

    struct Brawler
    {
        uint32_t slot = 0;
        uint32_t simulatableId = 0;
        std::vector<BodyId> bodies;
        std::vector<ShapeId> shapes;
        BodyId root;

        BodyId body(uint32_t declaration) const { return bodies[declaration]; }
        ShapeId shape(uint32_t declaration) const { return shapes[declaration]; }
    };

    Brawler bind(QueryWorld& scene, uint32_t slot, uint32_t simulatableId)
    {
        JoltPhysicsFactory factory(scene.bodies, brawlerSlotTemplate(), slot, simulatableId, {},
            [&scene](BodyId body, uint32_t shapeIndex, std::optional<BodyId> root) { return scene.query.registerShape(body, shapeIndex, root); });
        Brawler brawler{ slot, simulatableId, {}, {}, factory.parentBodyId() };
        for (const SlotBodyTemplate& slotTemplate : brawlerSlotTemplate())
        {
            const JoltPhysicsFactory::PhysicalObjectResult result = factory.createPhysicalObject(slotTemplate.descriptor, "declaration");
            brawler.bodies.push_back(result.bodyId);
            brawler.shapes.push_back(result.shapeIds.front());
        }
        return brawler;
    }

    void place(QueryWorld& scene, BodyId body, const glm::vec3& position)
    {
        scene.world.bodies().SetPosition(JoltPhysicsBodyAdapter::joltBodyIdOf(body), JPH::RVec3(position.x, position.y, position.z),
            JPH::EActivation::DontActivate);
    }

    glm::vec3 farAway(const Brawler& brawler, uint32_t declaration)
    {
        return glm::vec3(7.f * float(declaration), 30.f, -(100.f + 7.f * float(brawler.slot)));
    }

    void spreadOut(QueryWorld& scene, const Brawler& brawler)
    {
        for (uint32_t declaration = 0; declaration < brawler.bodies.size(); ++declaration)
        {
            place(scene, brawler.body(declaration), farAway(brawler, declaration));
        }
    }

    glm::mat4 at(const glm::vec3& position)
    {
        return glm::translate(glm::mat4(1.f), position);
    }

    QueryVolumeDescriptor sphereVolume(float radius, uint32_t searchCategories, uint32_t traceCategory = kQueryRouting)
    {
        return QueryVolumeDescriptor{ SphereGeometry{ radius }, CollisionCategories{ searchCategories }, glm::mat4(1.f), traceCategory };
    }

    QueryVolumeId volumeAt(QueryWorld& scene, const QueryVolumeDescriptor& descriptor, const glm::vec3& position, BodyId ignoredRoot = {})
    {
        const QueryVolumeId volume = scene.query.registerVolume(descriptor, ignoredRoot);
        scene.query.setVolumeParentTransform(volume, at(position));
        return volume;
    }

    std::vector<BodyId> bodiesOf(const SpatialQueryReport& report)
    {
        std::vector<BodyId> bodies;
        for (const SpatialQueryHit& hit : report)
        {
            bodies.push_back(hit.bodyId);
        }
        return bodies;
    }

    bool contains(const SpatialQueryReport& report, BodyId body)
    {
        return std::any_of(report.begin(), report.end(), [body](const SpatialQueryHit& hit) { return hit.bodyId == body; });
    }

    bool sameHits(const SpatialQueryReport& first, const SpatialQueryReport& second)
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

    struct PeerStableHit
    {
        JoltQueryHitKey key;
        uint32_t categories = 0;
        glm::vec3 objectPosition{ 0.f };
        uint32_t rootSimulatableId = 0;

        bool operator==(const PeerStableHit&) const = default;
    };

    std::vector<PeerStableHit> peerStable(const QueryWorld& scene, const SpatialQueryReport& report)
    {
        std::vector<PeerStableHit> hits;
        for (const SpatialQueryHit& hit : report)
        {
            const std::optional<JoltQueryHitKey> key = scene.query.hitKeyOf(hit.bodyId);
            const std::optional<JoltQueryHitKey> rootKey = scene.query.hitKeyOf(hit.rootBodyId);
            REQUIRE(key.has_value());
            REQUIRE(rootKey.has_value());
            hits.push_back(PeerStableHit{ *key, hit.objectCategories.bits, hit.objectPosition, rootKey->simulatableId });
        }
        return hits;
    }

    float distance(const glm::vec3& first, const glm::vec3& second)
    {
        const glm::vec3 d = first - second;
        return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    }

    StaticShapeDescriptor staticElement(StaticShape shape, const glm::vec3& position, uint64_t stableKey)
    {
        StaticShapeDescriptor descriptor;
        descriptor.shape = std::move(shape);
        descriptor.localToWorld = at(position);
        descriptor.categories = CollisionCategories{ bit(kWorld) };
        descriptor.blockingCategories = CollisionCategories{ kAllMappedCategories };
        descriptor.friction = 0.7f;
        descriptor.restitution = 0.3f;
        descriptor.stableKey = stableKey;
        return descriptor;
    }

    StaticWorldDescription arenaDescription()
    {
        StaticWorldDescription description;
        description.shapes.push_back(staticElement(StaticBox{ glm::vec3(20.f, 0.5f, 20.f) }, glm::vec3(0.f, -0.5f, -0.f), 101));
        description.shapes.push_back(staticElement(StaticBox{ glm::vec3(0.5f, 0.5f, 0.5f) }, glm::vec3(0.f, 0.5f, -3.f), 102));
        description.shapes.push_back(staticElement(StaticSphere{ 0.4f }, glm::vec3(0.f, 0.4f, 3.f), 103));
        description.shapes.push_back(staticElement(StaticCapsule{ 0.3f, 0.9f }, glm::vec3(3.f, 0.9f, -0.f), 104));
        StaticTriangleMesh ramp;
        ramp.vertices = { glm::vec3(-1.f, 0.f, 1.f), glm::vec3(-1.f, 0.f, -1.f),
            glm::vec3(1.f, 0.f, -1.f), glm::vec3(1.f, 0.f, 1.f) };
        ramp.indices = { 0, 2, 1, 0, 3, 2 };
        description.shapes.push_back(staticElement(ramp, glm::vec3(-4.f, 0.2f, -0.f), 105));
        return description;
    }

    void logToNowhere(const char*)
    {
    }
} // namespace

TEST_CASE("JoltSpatialQuery.SatisfiesTheConceptAndTheAccessCheckFollowsTheScope", "[Jolt]")
{
    STATIC_REQUIRE(SpatialQueryAdapter<JoltSpatialQueryAdapter>);

    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    CHECK_FALSE(scene.query.callerMayAccessWorld());
    {
        JoltWorldAccessScope outer;
        CHECK(scene.query.callerMayAccessWorld());
        {
            JoltWorldAccessScope inner;
            CHECK(scene.query.callerMayAccessWorld());
        }
        CHECK(scene.query.callerMayAccessWorld());
    }
    CHECK_FALSE(scene.query.callerMayAccessWorld());

    JoltSpatialQueryAdapter alwaysAllowed(scene.world, nullptr, JoltSpatialQueryConfig{ kAllMappedCategories, [] { return true; } });
    JoltSpatialQueryAdapter neverAllowed(scene.world, nullptr, JoltSpatialQueryConfig{ kAllMappedCategories, [] { return false; } });
    JoltWorldAccessScope scope;
    CHECK(alwaysAllowed.callerMayAccessWorld());
    CHECK_FALSE(neverAllowed.callerMayAccessWorld());
}

TEST_CASE("JoltSpatialQuery.OverlapHitAndMiss", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const Brawler target = bind(scene, 2, 42);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 2 }));
    spreadOut(scene, target);

    const QueryVolumeId volume = volumeAt(scene, sphereVolume(0.5f, bit(kBody) | bit(kGuard)), glm::vec3(0.f, 0.f, -0.f));
    const glm::vec3 capsulePosition = glm::vec3(-2.f, 1.f, -4.f);
    place(scene, target.root, capsulePosition);

    place(scene, target.body(kRadialDeclaration), glm::vec3(0.f, 0.f, -0.7f));
    const SpatialQueryReport hit = scene.query.overlap({ volume });
    REQUIRE(hit.size() == 1);
    CHECK(hit[0].bodyId == target.body(kRadialDeclaration));
    CHECK(hit[0].rootBodyId == target.root);
    CHECK(hit[0].objectCategories.bits == bit(kBody));
    CHECK(distance(hit[0].objectPosition, capsulePosition) < 1.0000001e-5f);

    place(scene, target.body(kRadialDeclaration), glm::vec3(0.f, 0.f, -0.9f));
    CHECK(scene.query.overlap({ volume }).empty());

    scene.query.setVolumeParentTransform(volume, at(glm::vec3(0.f, 0.f, -0.3f)));
    CHECK(scene.query.overlap({ volume }).size() == 1);
    CHECK(scene.logLines.empty());
}

TEST_CASE("JoltSpatialQuery.SelfIgnoreDropsEveryBodyOfTheIgnoredRoot", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const Brawler self = bind(scene, 0, 10);
    const Brawler other = bind(scene, 1, 20);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 0, 1 }));
    spreadOut(scene, self);
    spreadOut(scene, other);
    for (uint32_t declaration = 0; declaration < self.bodies.size(); ++declaration)
    {
        place(scene, self.body(declaration), glm::vec3(0.f, 0.f, -(0.1f * float(declaration))));
    }
    place(scene, other.body(kRadialDeclaration), glm::vec3(0.6f, 0.f, -0.f));

    const uint32_t everything = kAllMappedCategories;
    const QueryVolumeId ignoringSelf = volumeAt(scene, sphereVolume(1.f, everything), glm::vec3(0.f, 0.f, -0.f), self.root);
    const QueryVolumeId ignoringNothing = volumeAt(scene, sphereVolume(1.f, everything), glm::vec3(0.f, 0.f, -0.f));

    const SpatialQueryReport withSelf = scene.query.overlap({ ignoringNothing });
    CHECK(withSelf.size() == self.bodies.size() + 1u);
    const SpatialQueryReport withoutSelf = scene.query.overlap({ ignoringSelf });
    REQUIRE(withoutSelf.size() == 1);
    CHECK(withoutSelf[0].bodyId == other.body(kRadialDeclaration));
    CHECK(withoutSelf[0].rootBodyId == other.root);

    const QueryVolumeDescriptor probe{ SphereGeometry{ 0.05f }, CollisionCategories{ everything }, glm::mat4(1.f), kQueryRouting };
    const QueryVolumeId sweepIgnoringSelf = scene.query.registerVolume(probe, self.root);
    const SweepHit swept = scene.query.sweep(sweepIgnoringSelf, at(glm::vec3(-0.5f, 0.f, -0.f)), glm::vec3(2.f, 0.f, -0.f));
    REQUIRE(swept.blocked);
    CHECK(swept.bodyId == other.body(kRadialDeclaration));
}

TEST_CASE("JoltSpatialQuery.CategoryFilteringIsAnyOfOverMembership", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const Brawler target = bind(scene, 3, 33);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 3 }));
    spreadOut(scene, target);
    for (uint32_t declaration = 0; declaration < target.bodies.size(); ++declaration)
    {
        place(scene, target.body(declaration), glm::vec3(0.f, 0.15f * float(declaration), -0.f));
    }

    const auto found = [&scene](uint32_t search) {
        const QueryVolumeId volume = volumeAt(scene, sphereVolume(1.5f, search), glm::vec3(0.f, 0.f, -0.f));
        std::vector<uint32_t> categories;
        for (const SpatialQueryHit& hit : scene.query.overlap({ volume }))
        {
            categories.push_back(hit.objectCategories.bits);
        }
        return categories;
    };

    CHECK(found(bit(kBody) | bit(kGuard)) == std::vector<uint32_t>{ bit(kBody), bit(kGuard) });
    CHECK(found(bit(kGuard)) == std::vector<uint32_t>{ bit(kGuard) });
    CHECK(found(bit(kProjectile)) == std::vector<uint32_t>{ bit(kProjectile), bit(kProjectile), bit(kProjectile) });
    CHECK(found(bit(kCharacter) | bit(kBody)) == std::vector<uint32_t>{ bit(kBody), bit(kCharacter) });
    CHECK(found(bit(kWorld)).empty());
    CHECK(scene.logLines.empty());

    CHECK(found(bit(kBody) | bit(kUnmappedCategory)) == std::vector<uint32_t>{ bit(kBody) });
    CHECK(scene.linesContaining("[SpatialQuery.PartialObjectQuery]") == 1);
}

TEST_CASE("JoltSpatialQuery.AnEmptySearchIsATraceChannelQuery", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const JPH::BodyID floor = addFloor(scene.world);
    const Brawler target = bind(scene, 0, 5);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 0 }));
    spreadOut(scene, target);
    place(scene, target.body(kRadialDeclaration), glm::vec3(0.f, 0.6f, -0.f));
    place(scene, target.root, glm::vec3(0.f, 1.2f, -0.8f));

    const QueryVolumeId traceOnWorld = volumeAt(scene, sphereVolume(1.5f, 0u, kWorld), glm::vec3(0.f, 0.5f, -0.f));
    const SpatialQueryReport overlaps = scene.query.overlap({ traceOnWorld });
    CHECK(contains(overlaps, target.body(kRadialDeclaration)));
    CHECK(contains(overlaps, target.root));
    CHECK(contains(overlaps, JoltPhysicsBodyAdapter::bodyIdOf(floor)));
    CHECK(overlaps.size() == 3);
    CHECK(scene.logLines.empty());

    const QueryVolumeId traceOnGuard = volumeAt(scene, sphereVolume(1.5f, 0u, kGuard), glm::vec3(0.f, 0.5f, -0.f));
    const SpatialQueryReport guardTrace = scene.query.overlap({ traceOnGuard });
    CHECK(contains(guardTrace, target.body(kRadialDeclaration)));
    CHECK(contains(guardTrace, target.root));
    CHECK(contains(guardTrace, JoltPhysicsBodyAdapter::bodyIdOf(floor)));

    const QueryVolumeDescriptor probe{ SphereGeometry{ 0.05f }, CollisionCategories{}, glm::mat4(1.f), kWorld };
    const QueryVolumeId sweepOnWorld = scene.query.registerVolume(probe, BodyId{});
    const SweepHit throughSphere = scene.query.sweep(sweepOnWorld, at(glm::vec3(0.f, 2.f, -0.f)), glm::vec3(0.f, -3.f, -0.f));
    REQUIRE(throughSphere.blocked);
    CHECK(throughSphere.bodyId == JoltPhysicsBodyAdapter::bodyIdOf(floor));
    const SweepHit throughCapsule = scene.query.sweep(sweepOnWorld, at(glm::vec3(0.f, 4.f, -0.8f)), glm::vec3(0.f, -5.f, -0.f));
    REQUIRE(throughCapsule.blocked);
    CHECK(throughCapsule.bodyId == target.root);

    const QueryVolumeId allUnmapped = volumeAt(scene, sphereVolume(1.5f, bit(kUnmappedCategory), kWorld), glm::vec3(0.f, 0.5f, -0.f));
    CHECK(scene.linesContaining("[SpatialQuery.EmptyObjectQuery]") == 1);
    CHECK(bodiesOf(scene.query.overlap({ allUnmapped })) == bodiesOf(overlaps));

    (void)scene.query.registerVolume(sphereVolume(0.1f, bit(kBody), kUnmappedCategory), BodyId{});
    CHECK(scene.linesContaining("[SpatialQuery.UnmappedTraceCategory]") == 1);
}

TEST_CASE("JoltSpatialQuery.ASweepReportsTheFirstHitAndItsFraction", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const JPH::BodyID floor = addFloor(scene.world);
    const Brawler target = bind(scene, 1, 7);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 1 }));
    spreadOut(scene, target);

    const QueryVolumeDescriptor probe{ SphereGeometry{ 0.1f }, CollisionCategories{ bit(kWorld) | bit(kBody) }, glm::mat4(1.f), kQueryRouting };
    const QueryVolumeId volume = scene.query.registerVolume(probe, BodyId{});

    const SweepHit onFloor = scene.query.sweep(volume, at(glm::vec3(0.f, 1.f, -0.f)), glm::vec3(0.f, -2.f, -0.f));
    REQUIRE(onFloor.blocked);
    CHECK_FALSE(onFloor.startPenetrating);
    CHECK(onFloor.fraction == Catch::Approx(0.45f).margin(1.e-3f));
    CHECK(simFrame::height(onFloor.normal) == Catch::Approx(1.f).margin(1.e-4f));
    CHECK(simFrame::height(onFloor.impactPoint) == Catch::Approx(0.f).margin(0.0005f));
    CHECK(onFloor.penetrationDepth == 0.f);
    CHECK(onFloor.bodyId == JoltPhysicsBodyAdapter::bodyIdOf(floor));
    CHECK(onFloor.rootBodyId == onFloor.bodyId);
    CHECK(onFloor.objectCategories.bits == bit(kWorld));

    place(scene, target.body(kRadialDeclaration), glm::vec3(0.f, 0.5f, -0.f));
    const SweepHit onBody = scene.query.sweep(volume, at(glm::vec3(0.f, 1.5f, -0.f)), glm::vec3(0.f, -2.f, -0.f));
    REQUIRE(onBody.blocked);
    CHECK(onBody.bodyId == target.body(kRadialDeclaration));
    CHECK(onBody.rootBodyId == target.root);
    CHECK(onBody.fraction == Catch::Approx((150.f - 10.f - 80.f) / 200.f).margin(1.e-3f));
    CHECK(simFrame::height(onBody.impactPoint) == Catch::Approx(0.8f).margin(0.0005f));

    const SweepHit miss = scene.query.sweep(volume, at(glm::vec3(0.f, 1.5f, -5.f)), glm::vec3(0.f, 1.f, -0.f));
    CHECK_FALSE(miss.blocked);
    CHECK(miss.fraction == 1.f);
}

TEST_CASE("JoltSpatialQuery.ASweepThatStartsInsideATargetReportsItAtFractionZero", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const JPH::BodyID floor = addFloor(scene.world);
    const Brawler target = bind(scene, 0, 3);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 0 }));
    spreadOut(scene, target);

    const QueryVolumeDescriptor probe{ SphereGeometry{ 0.1f }, CollisionCategories{ bit(kWorld) | bit(kBody) }, glm::mat4(1.f), kQueryRouting };
    const QueryVolumeId volume = scene.query.registerVolume(probe, BodyId{});
    const glm::vec3 insideFloor = glm::vec3(0.f, 0.05f, -0.f);

    for (const glm::vec3& delta : { glm::vec3(0.f, 0.5f, -0.f), glm::vec3(0.f, -0.5f, -0.f),
             glm::vec3(0.f, 0.f, -0.4f), glm::vec3(0.f) })
    {
        const SweepHit hit = scene.query.sweep(volume, at(insideFloor), delta);
        REQUIRE(hit.blocked);
        CHECK(hit.startPenetrating);
        CHECK(hit.fraction == 0.f);
        CHECK(hit.penetrationDepth == Catch::Approx(0.05f).margin(0.0005f));
        CHECK(simFrame::height(hit.normal) == Catch::Approx(1.f).margin(1.e-3f));
        CHECK(hit.bodyId == JoltPhysicsBodyAdapter::bodyIdOf(floor));
    }

    place(scene, target.body(kRadialDeclaration), glm::vec3(0.f, 1.05f, -0.f));
    const SweepHit insideWins = scene.query.sweep(volume, at(insideFloor), glm::vec3(0.f, 2.f, -0.f));
    REQUIRE(insideWins.blocked);
    CHECK(insideWins.startPenetrating);
    CHECK(insideWins.bodyId == JoltPhysicsBodyAdapter::bodyIdOf(floor));

    const SweepHit fromInsideTheBody = scene.query.sweep(volume, at(glm::vec3(0.f, 1.15f, -0.f)), glm::vec3(0.f, 1.f, -0.f));
    REQUIRE(fromInsideTheBody.blocked);
    CHECK(fromInsideTheBody.startPenetrating);
    CHECK(fromInsideTheBody.bodyId == target.body(kRadialDeclaration));
    CHECK(fromInsideTheBody.penetrationDepth == Catch::Approx(0.3f).margin(0.0005f));
    CHECK(simFrame::height(fromInsideTheBody.normal) == Catch::Approx(1.f).margin(1.e-3f));
}

TEST_CASE("JoltSpatialQuery.DisabledShapesDoNotHit", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const Brawler target = bind(scene, 4, 44);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 4 }));
    spreadOut(scene, target);
    place(scene, target.body(kGuardDeclaration), glm::vec3(0.f, 0.f, -0.f));

    const QueryVolumeId volume = volumeAt(scene, sphereVolume(0.2f, bit(kGuard)), glm::vec3(0.f, 0.f, -0.f));
    const QueryVolumeDescriptor probe{ SphereGeometry{ 0.05f }, CollisionCategories{ bit(kGuard) }, glm::mat4(1.f), kQueryRouting };
    const QueryVolumeId sweeper = scene.query.registerVolume(probe, BodyId{});
    const auto sweepHitsGuard = [&] { return scene.query.sweep(sweeper, at(glm::vec3(0.f, 0.f, 2.f)), glm::vec3(0.f, 0.f, -4.f)).blocked; };

    CHECK(scene.query.overlap({ volume }).size() == 1);
    CHECK(sweepHitsGuard());
    scene.query.disableShape(target.shape(kGuardDeclaration));
    CHECK(scene.query.overlap({ volume }).empty());
    CHECK_FALSE(sweepHitsGuard());
    scene.query.enableShape(target.shape(kGuardDeclaration));
    CHECK(scene.query.overlap({ volume }).size() == 1);
    CHECK(sweepHitsGuard());

    scene.query.disableShape(target.shape(kGuardDeclaration));
    const Brawler rebound = bind(scene, 4, 45);
    CHECK(rebound.shape(kGuardDeclaration) == target.shape(kGuardDeclaration));
    CHECK(scene.query.overlap({ volume }).size() == 1);
}

TEST_CASE("JoltSpatialQuery.AfterRestoreTickTheFirstOverlapSeesTheRestoredEnableFlags", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime, queryWorldConfig(8));
    JoltWorldAccessScope access;
    const Brawler target = bind(scene, 0, 1);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 0 }));
    spreadOut(scene, target);
    place(scene, target.body(kGuardDeclaration), glm::vec3(0.f, 0.f, -0.f));
    const QueryVolumeId volume = volumeAt(scene, sphereVolume(0.2f, bit(kGuard)), glm::vec3(0.f, 0.f, -0.f));

    scene.world.saveTick(5);
    scene.query.disableShape(target.shape(kGuardDeclaration));
    scene.world.saveTick(6);
    CHECK(scene.query.overlap({ volume }).empty());

    REQUIRE(scene.world.restoreTick(5));
    CHECK(scene.query.overlap({ volume }).size() == 1);

    REQUIRE(scene.world.restoreTick(6));
    CHECK(scene.query.overlap({ volume }).empty());
}

TEST_CASE("JoltSpatialQuery.ParkedAndUnboundSlotBodiesAreNeverReturned", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const QueryVolumeId everythingAtTheParkingPoint = volumeAt(scene, sphereVolume(3.f, kAllMappedCategories), glm::vec3(0.f, 0.f, -0.f));
    const QueryVolumeId traceAtTheParkingPoint = volumeAt(scene, sphereVolume(3.f, 0u, kQueryRouting), glm::vec3(0.f, 0.f, -0.f));
    const QueryVolumeDescriptor probe{ SphereGeometry{ 0.05f }, CollisionCategories{ kAllMappedCategories }, glm::mat4(1.f), kQueryRouting };
    const QueryVolumeId sweeper = scene.query.registerVolume(probe, BodyId{});
    const auto nothingFound = [&] {
        return scene.query.overlap({ everythingAtTheParkingPoint }).empty() && scene.query.overlap({ traceAtTheParkingPoint }).empty()
            && !scene.query.sweep(sweeper, at(glm::vec3(0.f, 0.f, 5.f)), glm::vec3(0.f, 0.f, -10.f)).blocked;
    };

    CHECK(nothingFound());

    const Brawler boundButVacant = bind(scene, 1, 11);
    CHECK(nothingFound());

    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 2 }));
    CHECK(nothingFound());

    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 1, 2 }));
    CHECK(scene.query.overlap({ everythingAtTheParkingPoint }).size() == boundButVacant.bodies.size());
    for (const SpatialQueryHit& hit : scene.query.overlap({ traceAtTheParkingPoint }))
    {
        CHECK(scene.query.hitKeyOf(hit.bodyId)->simulatableId == 11u);
    }
}

TEST_CASE("JoltSpatialQuery.HitsAreSortedByAPeerStableKeyUnderADifferentSlotAssignment", "[Jolt]")
{
    RuntimeLease lease;
    const std::vector<uint32_t> simulatables{ 10, 20, 30 };
    const std::vector<glm::vec3> bodyPositions{ glm::vec3(0.f, 0.f, -1.f), glm::vec3(1.f, 0.f, -0.f),
        glm::vec3(0.f, 0.f, 1.f) };

    const auto run = [&](const std::vector<uint32_t>& slots) {
        QueryWorld scene(lease.runtime);
        JoltWorldAccessScope access;
        BodySlotOccupancy occupancy;
        for (size_t index = 0; index < simulatables.size(); ++index)
        {
            const Brawler brawler = bind(scene, slots[index], simulatables[index]);
            occupancy.occupied.set(slots[index]);
            for (uint32_t declaration = 0; declaration < brawler.bodies.size(); ++declaration)
            {
                place(scene, brawler.body(declaration), glm::vec3(7.f * float(declaration), 0.f, -(50.f + 10.f * float(index))));
            }
            place(scene, brawler.body(kRadialDeclaration), bodyPositions[index]);
            place(scene, brawler.body(kGuardDeclaration), bodyPositions[index] + glm::vec3(0.f, 0.5f, -0.f));
        }
        scene.world.applyBodySlotOccupancy(occupancy);
        const QueryVolumeId volume = volumeAt(scene, sphereVolume(3.f, bit(kBody) | bit(kGuard)), glm::vec3(0.f, 0.f, -0.f));
        const SpatialQueryReport report = scene.query.overlap({ volume });
        std::vector<BodyId> bodies = bodiesOf(report);
        return std::make_pair(peerStable(scene, report), bodies);
    };

    const auto [first, firstBodies] = run({ 0, 1, 2 });
    const auto [second, secondBodies] = run({ 5, 3, 0 });
    REQUIRE(first.size() == 6);
    CHECK(first == second);
    CHECK(firstBodies != secondBodies);
    CHECK(!std::is_sorted(secondBodies.begin(), secondBodies.end()));
    for (size_t index = 0; index < first.size(); ++index)
    {
        CHECK(first[index].key.simulatableId == simulatables[index / 2]);
        CHECK(first[index].key.declarationIndex == (index % 2 == 0 ? kRadialDeclaration : kGuardDeclaration));
    }
}

TEST_CASE("JoltSpatialQuery.IdenticalHitListsAcrossASaveRestoreCycleAndAShuffledCreationOrder", "[Jolt]")
{
    RuntimeLease lease;
    const StaticWorldDescription description = arenaDescription();

    const auto build = [&](QueryWorld& scene, StaticWorldDescription shapes, const std::vector<uint32_t>& bindOrder) {
        JoltStaticWorldBuilder builder(scene.world, &logToNowhere);
        (void)builder.build(shapes);
        BodySlotOccupancy occupancy;
        for (const uint32_t slot : bindOrder)
        {
            const Brawler brawler = bind(scene, slot, 100u + slot);
            occupancy.occupied.set(slot);
            for (uint32_t declaration = 0; declaration < brawler.bodies.size(); ++declaration)
            {
                place(scene, brawler.body(declaration),
                    glm::vec3(0.4f * float(declaration), 1.5f + 0.2f * float(declaration), -(1.5f * float(slot) - 1.5f)));
            }
        }
        scene.world.applyBodySlotOccupancy(occupancy);
    };

    const std::vector<JoltLayerKey> staticLayers = staticLayerKeysOf(description);
    QueryWorld original(lease.runtime, queryWorldConfig(16, staticLayers));
    JoltWorldAccessScope access;
    build(original, description, { 0, 1, 2 });
    const QueryVolumeId arena = volumeAt(original, sphereVolume(6.f, kAllMappedCategories), glm::vec3(0.f, 1.f, -0.f));
    const SpatialQueryReport before = original.query.overlap({ arena });
    CHECK(before.size() > 18u);

    original.world.saveTick(1);
    for (uint32_t tick = 2; tick < 12; ++tick)
    {
        applyScriptedForces(original.world, tick);
        original.world.step(kDt);
        original.world.saveTick(tick);
    }
    CHECK_FALSE(sameHits(original.query.overlap({ arena }), before));
    REQUIRE(original.world.restoreTick(1));
    CHECK(sameHits(original.query.overlap({ arena }), before));

    StaticWorldDescription shuffled = description;
    std::mt19937 generator(1234u);
    std::shuffle(shuffled.shapes.begin(), shuffled.shapes.end(), generator);
    QueryWorld reordered(lease.runtime, queryWorldConfig(16, staticLayers));
    build(reordered, shuffled, { 2, 0, 1 });
    const QueryVolumeId reorderedArena = volumeAt(reordered, sphereVolume(6.f, kAllMappedCategories), glm::vec3(0.f, 1.f, -0.f));
    CHECK(sameHits(reordered.query.overlap({ reorderedArena }), before));
}

TEST_CASE("JoltSpatialQuery.OneHitPerStaticElementNotPerTriangle", "[Jolt]")
{
    RuntimeLease lease;
    const StaticWorldDescription description = arenaDescription();
    QueryWorld scene(lease.runtime, queryWorldConfig(0, staticLayerKeysOf(description)));
    JoltWorldAccessScope access;
    JoltStaticWorldBuilder builder(scene.world, &logToNowhere);
    (void)builder.build(description);

    const QueryVolumeId onTheRamp = volumeAt(scene, sphereVolume(0.2f, bit(kWorld)), glm::vec3(-4.f, 0.25f, -0.f));
    const SpatialQueryReport ramp = scene.query.overlap({ onTheRamp });
    REQUIRE(ramp.size() == 1);
    CHECK(distance(ramp[0].objectPosition, glm::vec3(-4.f, 0.2f, -0.f)) < 0.01f);
    CHECK(ramp[0].objectCategories.bits == bit(kWorld));

    const QueryVolumeId onTheBoxAndFloor = volumeAt(scene, sphereVolume(0.2f, bit(kWorld)), glm::vec3(0.f, 1.05f, -3.f));
    const SpatialQueryReport box = scene.query.overlap({ onTheBoxAndFloor });
    REQUIRE(box.size() == 1);
    CHECK(distance(box[0].objectPosition, glm::vec3(0.f, 0.5f, -3.f)) < 0.01f);

    const QueryVolumeId acrossTwoElements = volumeAt(scene, sphereVolume(0.8f, bit(kWorld)), glm::vec3(0.f, 0.2f, -3.f));
    CHECK(scene.query.overlap({ acrossTwoElements }).size() == 2);
}

TEST_CASE("JoltSpatialQuery.PhysicsOnlyStaticsAreNotFoundButStillCollide", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime, queryWorldConfig(0, { kWorldStaticKey }));
    JoltWorldAccessScope access;
    StaticWorldDescription queryable;
    queryable.shapes.push_back(staticElement(StaticBox{ glm::vec3(2.f, 0.5f, 2.f) }, glm::vec3(0.f, -0.5f, -0.f), 1));
    StaticWorldDescription physicsOnly;
    physicsOnly.shapes.push_back(staticElement(StaticBox{ glm::vec3(2.f, 0.5f, 2.f) }, glm::vec3(0.f, -0.5f, -10.f), 2));
    JoltStaticWorldBuilder queryableBuilder(scene.world, &logToNowhere);
    (void)queryableBuilder.build(queryable);
    JoltStaticWorldBuilder physicsOnlyBuilder(scene.world, &logToNowhere);
    (void)physicsOnlyBuilder.build(physicsOnly);
    REQUIRE(physicsOnlyBuilder.stats().bodies.size() == 1);

    const QueryVolumeId overPhysicsOnly = volumeAt(scene, sphereVolume(0.2f, bit(kWorld)), glm::vec3(0.f, 0.1f, -10.f));
    const QueryVolumeId overQueryable = volumeAt(scene, sphereVolume(0.2f, bit(kWorld)), glm::vec3(0.f, 0.1f, -0.f));
    const QueryVolumeDescriptor probe{ SphereGeometry{ 0.05f }, CollisionCategories{ bit(kWorld) }, glm::mat4(1.f), kQueryRouting };
    const QueryVolumeId sweeper = scene.query.registerVolume(probe, BodyId{});
    CHECK(scene.query.overlap({ overPhysicsOnly }).size() == 1);

    scene.query.excludeStaticBodiesFromQueries(physicsOnlyBuilder.stats().bodies);
    CHECK(scene.query.overlap({ overPhysicsOnly }).empty());
    CHECK_FALSE(scene.query.sweep(sweeper, at(glm::vec3(0.f, 1.f, -10.f)), glm::vec3(0.f, -2.f, -0.f)).blocked);
    CHECK(scene.query.overlap({ overQueryable }).size() == 1);
    CHECK(scene.query.sweep(sweeper, at(glm::vec3(0.f, 1.f, -0.f)), glm::vec3(0.f, -2.f, -0.f)).blocked);

    const Brawler faller = bind(scene, 0, 1);
    spreadOut(scene, faller);
    place(scene, faller.root, glm::vec3(0.f, 1.5f, -10.f));
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 0 }));
    for (int tick = 0; tick < 120; ++tick)
    {
        scene.world.step(kDt);
    }
    const float restingHeight = simFrame::height(glm::vec3(scene.bodies.getBodyTransform(faller.root)[3]));
    CHECK(restingHeight == Catch::Approx(0.96f).margin(0.02f));
}

TEST_CASE("JoltSpatialQuery.OverlapReportsOnlyTheLastVolumeAsTheChaosAdapterDoes", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const Brawler target = bind(scene, 0, 1);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 0 }));
    spreadOut(scene, target);
    place(scene, target.body(kRadialDeclaration), glm::vec3(0.f, 0.f, -0.f));
    place(scene, target.body(kGuardDeclaration), glm::vec3(0.f, 0.f, -5.f));

    const QueryVolumeId aroundRadial = volumeAt(scene, sphereVolume(0.2f, bit(kBody) | bit(kGuard)), glm::vec3(0.f, 0.f, -0.f));
    const QueryVolumeId aroundGuard = volumeAt(scene, sphereVolume(0.2f, bit(kBody) | bit(kGuard)), glm::vec3(0.f, 0.f, -5.f));
    CHECK(bodiesOf(scene.query.overlap({ aroundRadial, aroundGuard })) == std::vector<BodyId>{ target.body(kGuardDeclaration) });
    CHECK(bodiesOf(scene.query.overlap({ aroundGuard, aroundRadial })) == std::vector<BodyId>{ target.body(kRadialDeclaration) });

    const QueryVolumeId innerRadial = volumeAt(scene, sphereVolume(0.2f, bit(kBody) | bit(kGuard)), glm::vec3(0.f, 0.f, -0.f));
    const QueryVolumeId outerRadial = volumeAt(scene, sphereVolume(0.4f, bit(kBody) | bit(kGuard)), glm::vec3(0.f, 0.f, -0.f));
    CHECK(scene.query.overlap({ innerRadial, outerRadial }).size() == 1);
}

TEST_CASE("JoltSpatialQuery.TheQueryPoseIgnoresTheVolumeRotationAsTheChaosAdapterDoes", "[Jolt]")
{
    RuntimeLease lease;
    QueryWorld scene(lease.runtime);
    JoltWorldAccessScope access;
    const Brawler target = bind(scene, 0, 1);
    scene.world.applyBodySlotOccupancy(bodySlotOccupancyOf({ 0 }));
    spreadOut(scene, target);
    place(scene, target.body(kRadialDeclaration), glm::vec3(0.f, 0.f, -1.5f));

    const QueryVolumeDescriptor slab{ BoxGeometry{ glm::vec3(0.1f, 0.1f, 2.f) }, CollisionCategories{ bit(kBody) }, glm::mat4(1.f), kQueryRouting };
    const QueryVolumeId volume = scene.query.registerVolume(slab, BodyId{});
    const glm::mat4 yawed = glm::rotate(glm::mat4(1.f), glm::radians(90.f), glm::vec3(-0.f, -1.f, 0.f));
    scene.query.setVolumeParentTransform(volume, yawed);
    CHECK(scene.query.overlap({ volume }).size() == 1);
    CHECK(scene.query.sweep(volume, yawed, glm::vec3(0.f, 0.01f, -0.f)).blocked);

    const QueryVolumeDescriptor yawedOffset{ BoxGeometry{ glm::vec3(0.1f, 0.1f, 2.f) }, CollisionCategories{ bit(kBody) }, yawed, kQueryRouting };
    const QueryVolumeId offsetVolume = scene.query.registerVolume(yawedOffset, BodyId{});
    CHECK(scene.query.overlap({ offsetVolume }).size() == 1);
}

#endif
