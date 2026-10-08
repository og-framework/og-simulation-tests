// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltStaticWorldBuilder.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include "glm/gtc/matrix_transform.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

using namespace joltTestRig;

namespace
{
    constexpr uint32_t kWorldBlockingCharacterOnly = bit(kCharacter);

    glm::mat4 placed(const glm::vec3& positionCm, float yawDegrees = 0.f, float rollDegrees = 0.f)
    {
        glm::mat4 transform = glm::translate(glm::mat4(1.f), positionCm);
        transform = glm::rotate(transform, glm::radians(yawDegrees), glm::vec3(0.f, 0.f, 1.f));
        transform = glm::rotate(transform, glm::radians(rollDegrees), glm::vec3(1.f, 0.f, 0.f));
        return transform;
    }

    StaticShapeDescriptor element(StaticShape shape, const glm::mat4& localToWorld, uint64_t stableKey,
        uint32_t categories = bit(kWorld), uint32_t blocking = kAllMappedCategories, float friction = 0.7f, float restitution = 0.3f)
    {
        StaticShapeDescriptor descriptor;
        descriptor.shape = std::move(shape);
        descriptor.localToWorld = localToWorld;
        descriptor.categories = CollisionCategories{ categories };
        descriptor.blockingCategories = CollisionCategories{ blocking };
        descriptor.friction = friction;
        descriptor.restitution = restitution;
        descriptor.stableKey = stableKey;
        return descriptor;
    }

    uint64_t keyOf(uint64_t index)
    {
        return (index + 1u) * 0x9E3779B97F4A7C15ull;
    }

    StaticConvexHull boxHull(const glm::vec3& halfExtents)
    {
        StaticConvexHull hull;
        for (int corner = 0; corner < 8; ++corner)
        {
            hull.points.push_back(glm::vec3((corner & 1) ? halfExtents.x : -halfExtents.x, (corner & 2) ? halfExtents.y : -halfExtents.y,
                (corner & 4) ? halfExtents.z : -halfExtents.z));
        }
        return hull;
    }

    StaticTriangleMesh quadFacingUp(float halfSizeCm)
    {
        StaticTriangleMesh mesh;
        mesh.vertices = { { -halfSizeCm, -halfSizeCm, 0.f }, { halfSizeCm, -halfSizeCm, 0.f }, { halfSizeCm, halfSizeCm, 0.f }, { -halfSizeCm, halfSizeCm, 0.f } };
        mesh.indices = { 0, 1, 2, 0, 2, 3 };
        return mesh;
    }

    struct CapturedLog
    {
        std::shared_ptr<std::vector<std::string>> lines = std::make_shared<std::vector<std::string>>();

        JoltWorldLoggerFn logger() const
        {
            return [lines = lines](const char* line) { lines->emplace_back(line); };
        }

        size_t countContaining(const std::string& text) const
        {
            return size_t(std::count_if(lines->begin(), lines->end(), [&text](const std::string& line) { return line.find(text) != std::string::npos; }));
        }
    };

    struct StaticRig
    {
        StaticRig(const StaticWorldDescription& description, JoltStaticWorldBuilderOptions options = {})
        {
            JoltWorldConfig config = brawlerWorldConfig(4);
            config.staticLayers = staticLayerKeysOf(description);
            world = std::make_unique<JoltWorld>(lease.runtime, config, &logJoltProblems);
            builder = std::make_unique<JoltStaticWorldBuilder>(*world, log.logger(), options);
            report = builder->build(description);
        }

        RuntimeLease lease;
        CapturedLog log;
        std::unique_ptr<JoltWorld> world;
        std::unique_ptr<JoltStaticWorldBuilder> builder;
        StaticWorldBuildReport report;
    };

    class StaticClassOnly final : public JPH::ObjectLayerFilter
    {
    public:
        explicit StaticClassOnly(const JoltLayerTable& table) : m_table(table) {}

        bool ShouldCollide(JPH::ObjectLayer layer) const override { return m_table.keyOf(layer).broadPhaseClass == JoltBroadPhaseClass::Static; }

    private:
        const JoltLayerTable& m_table;
    };

    std::optional<glm::vec3> castRayCm(const JoltWorld& world, const glm::vec3& fromCm, const glm::vec3& toCm)
    {
        const JPH::RRayCast ray{ joltUnits::centimetresToMetres(fromCm), joltUnits::centimetresToMetres(toCm - fromCm) };
        JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
        world.query().CastRay(ray, JPH::RayCastSettings(), collector, {}, StaticClassOnly(world.layers()));
        if (!collector.HadHit())
        {
            return std::nullopt;
        }
        return fromCm + (toCm - fromCm) * collector.mHit.mFraction;
    }

    std::optional<float> topZCm(const JoltWorld& world, float xCm, float yCm)
    {
        const std::optional<glm::vec3> hit = castRayCm(world, { xCm, yCm, 5000.f }, { xCm, yCm, -5000.f });
        return hit.has_value() ? std::optional<float>(hit->z) : std::nullopt;
    }

    uint32_t subShapeCountOf(const JPH::Shape& shape)
    {
        if (shape.GetSubType() == JPH::EShapeSubType::StaticCompound)
        {
            return static_cast<const JPH::StaticCompoundShape&>(shape).GetNumSubShapes();
        }
        return 1;
    }

    StaticWorldDescription restingArena()
    {
        StaticWorldDescription description;
        uint64_t index = 0;
        description.shapes.push_back(element(boxHull({ 2000.f, 2000.f, 50.f }), placed({ 0.f, 0.f, -50.f }), keyOf(index++)));
        description.shapes.push_back(element(StaticSphere{ 40.f }, placed({ -225.f, -75.f, 20.f }), keyOf(index++)));
        description.shapes.push_back(element(StaticBox{ { 60.f, 40.f, 25.f } }, placed({ -75.f, -75.f, 25.f }, 20.f), keyOf(index++)));
        description.shapes.push_back(element(StaticCapsuleZ{ 30.f, 60.f }, placed({ 75.f, -75.f, 60.f }), keyOf(index++)));
        StaticTriangleMesh ramp = quadFacingUp(80.f);
        description.shapes.push_back(element(ramp, placed({ 225.f, -75.f, 40.f }, 0.f, 25.f), keyOf(index++)));
        description.shapes.push_back(element(boxHull({ 50.f, 50.f, 15.f }), placed({ -225.f, 75.f, 15.f }, 45.f), keyOf(index++), bit(kWorld),
            kWorldBlockingCharacterOnly, 0.5f, 0.1f));
        description.shapes.push_back(element(StaticBox{ { 30.f, 30.f, 10.f } }, placed({ -75.f, 75.f, 10.f }, -15.f), keyOf(index++), bit(kWorld),
            kWorldBlockingCharacterOnly));
        description.shapes.push_back(element(StaticSphere{ 25.f }, placed({ 75.f, 75.f, 0.f }), keyOf(index++), bit(kWorld), kAllMappedCategories, 0.f, 0.f));
        for (int peg = 0; peg < 12; ++peg)
        {
            const float angle = 0.5236f * float(peg);
            description.shapes.push_back(element(StaticBox{ { 10.f, 10.f, 40.f } }, placed({ 450.f * std::cos(angle), 450.f * std::sin(angle), 40.f }, 7.f * float(peg)),
                keyOf(index++), bit(kWorld), peg % 2 == 0 ? kAllMappedCategories : kWorldBlockingCharacterOnly, peg % 3 == 0 ? 0.5f : 0.7f, 0.3f));
        }
        return description;
    }

    struct RestingRun
    {
        std::vector<uint8_t> bytes;
        uint64_t hash = 0;
        std::vector<float> capsuleZMetres;
        JoltStaticBuildStats stats;
    };

    RestingRun runRestingCapsules(const StaticWorldDescription& description)
    {
        JoltStaticWorldBuilderOptions options;
        options.maxShapesPerChunk = 3;
        StaticRig rig(description, options);
        JoltWorld& world = *rig.world;
        world.applyOccupancy(allOccupied());
        placeAllBrawlers(world);
        for (uint32_t tick = 1; tick <= 300; ++tick)
        {
            world.applyOccupancy(allOccupied());
            world.step(kDt);
        }
        RestingRun run;
        run.bytes = fullState(world);
        run.hash = world.liveStateHash();
        for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
        {
            run.capsuleZMetres.push_back(float(world.bodies().GetCenterOfMassPosition(world.slotBodyId(slot, kTemplateCapsule)).GetZ()));
        }
        run.stats = rig.builder->stats();
        return run;
    }

    StaticWorldDescription manyElements(uint32_t count)
    {
        StaticWorldDescription description;
        std::mt19937 random(1234u);
        std::uniform_real_distribution<float> unit(0.f, 1.f);
        for (uint32_t index = 0; index < count; ++index)
        {
            const glm::mat4 transform = placed({ 4000.f * unit(random) - 2000.f, 4000.f * unit(random) - 2000.f, 300.f * unit(random) }, 360.f * unit(random),
                30.f * unit(random));
            StaticShape shape;
            switch (index % 5)
            {
            case 0: shape = StaticBox{ { 20.f + 80.f * unit(random), 20.f + 80.f * unit(random), 10.f + 40.f * unit(random) } }; break;
            case 1: shape = StaticSphere{ 10.f + 50.f * unit(random) }; break;
            case 2: shape = StaticCapsuleZ{ 15.f, 30.f + 60.f * unit(random) }; break;
            case 3:
            {
                StaticConvexHull hull;
                for (int point = 0; point < 16; ++point)
                {
                    hull.points.push_back({ 100.f * unit(random) - 50.f, 100.f * unit(random) - 50.f, 60.f * unit(random) - 30.f });
                }
                shape = hull;
                break;
            }
            default:
            {
                StaticTriangleMesh mesh;
                for (int y = 0; y < 5; ++y)
                {
                    for (int x = 0; x < 5; ++x)
                    {
                        mesh.vertices.push_back({ 40.f * float(x), 40.f * float(y), 10.f * unit(random) });
                    }
                }
                for (uint32_t y = 0; y < 4; ++y)
                {
                    for (uint32_t x = 0; x < 4; ++x)
                    {
                        const uint32_t corner = y * 5u + x;
                        mesh.indices.insert(mesh.indices.end(), { corner, corner + 1u, corner + 6u, corner, corner + 6u, corner + 5u });
                    }
                }
                shape = mesh;
                break;
            }
            }
            description.shapes.push_back(element(std::move(shape), transform, keyOf(index), bit(kWorld),
                index % 4 == 0 ? kWorldBlockingCharacterOnly : kAllMappedCategories, index % 7 == 0 ? 0.5f : 0.7f, 0.3f));
        }
        return description;
    }
} // namespace

TEST_CASE("JoltStaticWorldBuilder.EachShapeTypeIsQueryableAtTheExpectedPlace", "[Jolt][StaticGeometry]")
{
    StaticWorldDescription description;
    description.shapes.push_back(element(StaticBox{ { 200.f, 50.f, 30.f } }, placed({ 1000.f, 0.f, 100.f }, 90.f), keyOf(0)));
    description.shapes.push_back(element(StaticSphere{ 50.f }, placed({ 0.f, 1000.f, 200.f }), keyOf(1)));
    description.shapes.push_back(element(StaticCapsuleZ{ 40.f, 120.f }, placed({ -1000.f, 0.f, 200.f }), keyOf(2)));
    StaticConvexHull pyramid;
    pyramid.points = { { -100.f, -100.f, 0.f }, { 100.f, -100.f, 0.f }, { 100.f, 100.f, 0.f }, { -100.f, 100.f, 0.f }, { 0.f, 0.f, 150.f } };
    description.shapes.push_back(element(pyramid, placed({ 0.f, -1000.f, 50.f }, 0.f, 180.f), keyOf(3)));
    description.shapes.push_back(element(quadFacingUp(100.f), placed({ 1000.f, 1000.f, 75.f }, 45.f), keyOf(4)));

    StaticRig rig(description);
    const JoltWorld& world = *rig.world;
    REQUIRE(rig.builder->stats().shapesBuilt == 5);
    for (const size_t typeIndex : { kStaticShapeIndex<StaticBox>, kStaticShapeIndex<StaticSphere>, kStaticShapeIndex<StaticCapsuleZ>,
             kStaticShapeIndex<StaticConvexHull>, kStaticShapeIndex<StaticTriangleMesh> })
    {
        CHECK(rig.report.shapeCountByType[typeIndex] == 1);
    }

    constexpr float kToleranceCm = 0.05f;

    SECTION("box: yawed 90 degrees, so its long axis runs along world Y")
    {
        CHECK(topZCm(world, 1000.f, 0.f).value_or(-1.f) == Catch::Approx(130.f).margin(kToleranceCm));
        CHECK(topZCm(world, 1000.f, 150.f).value_or(-1.f) == Catch::Approx(130.f).margin(kToleranceCm));
        CHECK_FALSE(topZCm(world, 1150.f, 0.f).has_value());
    }
    SECTION("sphere: top at centre + radius, and the curve off-centre")
    {
        CHECK(topZCm(world, 0.f, 1000.f).value_or(-1.f) == Catch::Approx(250.f).margin(kToleranceCm));
        CHECK(topZCm(world, 0.f, 1030.f).value_or(-1.f) == Catch::Approx(240.f).margin(kToleranceCm));
        CHECK_FALSE(topZCm(world, 0.f, 1060.f).has_value());
    }
    SECTION("capsule: Z-up, totalHalfHeight to the tip, radius on the side")
    {
        CHECK(topZCm(world, -1000.f, 0.f).value_or(-1.f) == Catch::Approx(320.f).margin(kToleranceCm));
        const std::optional<glm::vec3> side = castRayCm(world, { -500.f, 0.f, 250.f }, { -1500.f, 0.f, 250.f });
        REQUIRE(side.has_value());
        CHECK(side->x == Catch::Approx(-960.f).margin(kToleranceCm));
        CHECK_FALSE(topZCm(world, -950.f, 0.f).has_value());
    }
    SECTION("convex: an upside-down pyramid, so its flat base is the top face")
    {
        CHECK(topZCm(world, 0.f, -1000.f).value_or(-1.f) == Catch::Approx(50.f).margin(kToleranceCm));
        CHECK(topZCm(world, 80.f, -1080.f).value_or(-1.f) == Catch::Approx(50.f).margin(kToleranceCm));
        CHECK_FALSE(topZCm(world, 120.f, -1000.f).has_value());
    }
    SECTION("triangle mesh: yawed 45 degrees, front face up, back face ignored by a default ray")
    {
        CHECK(topZCm(world, 1000.f, 1000.f).value_or(-1.f) == Catch::Approx(75.f).margin(kToleranceCm));
        CHECK(topZCm(world, 1130.f, 1000.f).value_or(-1.f) == Catch::Approx(75.f).margin(kToleranceCm));
        CHECK_FALSE(topZCm(world, 1150.f, 1000.f).has_value());
        CHECK_FALSE(castRayCm(world, { 1000.f, 1000.f, -500.f }, { 1000.f, 1000.f, 500.f }).has_value());
    }
}

TEST_CASE("JoltStaticWorldBuilder.GroupsByCollisionPairAndSurfaceIntoStaticBodiesOnTheirLayer", "[Jolt][StaticGeometry]")
{
    StaticWorldDescription description;
    for (uint64_t index = 0; index < 3; ++index)
    {
        description.shapes.push_back(element(StaticBox{ { 20.f, 20.f, 20.f } }, placed({ 100.f * float(index), 0.f, 20.f }), keyOf(index)));
    }
    for (uint64_t index = 3; index < 5; ++index)
    {
        description.shapes.push_back(element(StaticBox{ { 20.f, 20.f, 20.f } }, placed({ 100.f * float(index), 0.f, 20.f }), keyOf(index), bit(kWorld),
            kAllMappedCategories, 0.5f, 0.1f));
    }
    for (uint64_t index = 5; index < 7; ++index)
    {
        description.shapes.push_back(element(StaticSphere{ 20.f }, placed({ 100.f * float(index), 0.f, 20.f }), keyOf(index), bit(kWorld),
            kWorldBlockingCharacterOnly));
    }

    JoltStaticWorldBuilderOptions options;
    options.maxShapesPerChunk = 2;
    StaticRig rig(description, options);
    const JoltWorld& world = *rig.world;
    const JoltStaticBuildStats& stats = rig.builder->stats();

    CHECK(stats.groups == 3);
    CHECK(stats.chunks == 4);
    CHECK(stats.chunksSplitIntoSingles == 0);
    REQUIRE(stats.bodies.size() == 4);

    const JPH::BodyLockInterface& locks = world.physics().GetBodyLockInterfaceNoLock();
    uint32_t subShapes = 0;
    uint32_t previousIndex = world.simulatableSlots() * world.bodiesPerSlot();
    for (const JPH::BodyID& id : stats.bodies)
    {
        const JPH::Body* body = locks.TryGetBody(id);
        REQUIRE(body != nullptr);
        CHECK(body->IsStatic());
        CHECK(body->IsInBroadPhase());
        CHECK_FALSE(body->IsActive());
        CHECK(id.GetIndex() == previousIndex + 1u);
        previousIndex = id.GetIndex();
        const JoltLayerKey& key = world.layers().keyOf(body->GetObjectLayer());
        CHECK(key.broadPhaseClass == JoltBroadPhaseClass::Static);
        CHECK(key.categories == bit(kWorld));
        const bool secondSurface = body->GetFriction() == 0.5f;
        if (key.blockingCategories == kWorldBlockingCharacterOnly)
        {
            CHECK(body->GetFriction() == 0.7f);
            CHECK(body->GetRestitution() == 0.3f);
        }
        else
        {
            CHECK(key.blockingCategories == kAllMappedCategories);
            CHECK(body->GetRestitution() == (secondSurface ? 0.1f : 0.3f));
        }
        subShapes += subShapeCountOf(*body->GetShape());
    }
    CHECK(subShapes == 7);
    CHECK(rig.log.countContaining("built 7 of 7 shapes (box=5 sphere=2 capsule=0 convex=0 trimesh=0) into 4 static bodies") == 1);
}

TEST_CASE("JoltStaticWorldBuilder.ZeroFrictionAndRestitutionGetTheUnrealDefaultSurfaceAndAreLogged", "[Jolt][StaticGeometry]")
{
    StaticWorldDescription description;
    description.shapes.push_back(element(StaticBox{ { 20.f, 20.f, 20.f } }, placed({ 0.f, 0.f, 20.f }), keyOf(0), bit(kWorld), kAllMappedCategories, 0.f, 0.f));
    description.shapes.push_back(element(StaticBox{ { 20.f, 20.f, 20.f } }, placed({ 100.f, 0.f, 20.f }), keyOf(1), bit(kWorld), kAllMappedCategories, 0.f, 0.4f));

    SECTION("default options: the UE engine default material (friction 0.7, restitution 0.3)")
    {
        StaticRig rig(description);
        const JoltStaticBuildStats& stats = rig.builder->stats();
        CHECK(stats.zeroSurfaceElements == 1);
        REQUIRE(stats.bodies.size() == 2);
        CHECK(rig.world->bodies().GetFriction(stats.bodies[0]) == 0.f);
        CHECK(rig.world->bodies().GetRestitution(stats.bodies[0]) == 0.4f);
        CHECK(rig.world->bodies().GetFriction(stats.bodies[1]) == kUnrealDefaultPhysicalMaterialSurface.friction);
        CHECK(rig.world->bodies().GetRestitution(stats.bodies[1]) == kUnrealDefaultPhysicalMaterialSurface.restitution);
        CHECK(rig.log.countContaining("1 static element(s) arrived with friction = restitution = 0") == 1);
        CHECK(rig.log.countContaining("configured fallback surface") == 1);
    }
    SECTION("no fallback: the element keeps 0/0 and is still logged")
    {
        JoltStaticWorldBuilderOptions options;
        options.surfaceForZeroFrictionAndRestitution.reset();
        StaticRig rig(description, options);
        const JoltStaticBuildStats& stats = rig.builder->stats();
        REQUIRE(stats.bodies.size() == 2);
        CHECK(rig.world->bodies().GetFriction(stats.bodies[0]) == 0.f);
        CHECK(rig.world->bodies().GetRestitution(stats.bodies[0]) == 0.f);
        CHECK(rig.log.countContaining("they keep 0/0 (no fallback configured)") == 1);
    }
    CHECK(kUnrealDefaultPhysicalMaterialSurface.friction == 0.7f);
    CHECK(kUnrealDefaultPhysicalMaterialSurface.restitution == 0.3f);
}

TEST_CASE("JoltStaticWorldBuilder.ScaleIsAppliedOrApproximatedAndInvalidElementsAreSkipped", "[Jolt][StaticGeometry]")
{
    StaticWorldDescription description;
    description.shapes.push_back(element(StaticBox{ { 20.f, 20.f, 20.f } }, glm::scale(placed({ 0.f, 0.f, 40.f }), glm::vec3(2.f)), keyOf(0)));
    description.shapes.push_back(element(StaticSphere{ 20.f }, glm::scale(placed({ 300.f, 0.f, 0.f }), glm::vec3(1.f, 1.f, 2.f)), keyOf(1)));
    description.shapes.push_back(element(StaticConvexHull{}, placed({ 600.f, 0.f, 0.f }), keyOf(2)));
    StaticTriangleMesh broken = quadFacingUp(50.f);
    broken.indices.back() = 17;
    description.shapes.push_back(element(broken, placed({ 900.f, 0.f, 0.f }), keyOf(3)));
    description.shapes.push_back(element(StaticCapsuleZ{ 40.f, 30.f }, placed({ 1200.f, 0.f, 0.f }), keyOf(4)));
    description.shapes.push_back(element(StaticBox{ { 20.f, 20.f, 20.f } }, glm::scale(placed({ 1500.f, 0.f, 0.f }), glm::vec3(0.f)), keyOf(5)));

    StaticRig rig(description);
    const JoltStaticBuildStats& stats = rig.builder->stats();
    CHECK(stats.shapesBuilt == 2);
    CHECK(stats.skippedInvalidShape == 4);
    CHECK(stats.approximatedScales == 1);
    CHECK(rig.log.countContaining("skipped an element") == 4);
    CHECK(rig.log.countContaining("approximated a scale (stableKey 0x") == 1);
    CHECK(topZCm(*rig.world, 0.f, 0.f).value_or(-1.f) == Catch::Approx(80.f).margin(0.05f));
    CHECK(topZCm(*rig.world, 300.f, 0.f).has_value());
}

TEST_CASE("JoltStaticWorldBuilder.ShuffledDescriptionGivesIdenticalStateAfter300TicksOfRestingCapsules", "[Jolt][StaticGeometry]")
{
    const StaticWorldDescription original = restingArena();
    const RestingRun reference = runRestingCapsules(original);

    REQUIRE_FALSE(reference.bytes.empty());
    CHECK(reference.stats.shapesBuilt == uint32_t(original.shapes.size()));
    CHECK(reference.stats.groups >= 3);
    CHECK(reference.stats.chunks > reference.stats.groups);
    CHECK(reference.stats.zeroSurfaceElements == 1);
    for (const float zMetres : reference.capsuleZMetres)
    {
        CHECK(zMetres > 0.9f);
        CHECK(zMetres < 2.5f);
    }

    std::vector<StaticWorldDescription> shuffled(3, original);
    std::reverse(shuffled[0].shapes.begin(), shuffled[0].shapes.end());
    std::mt19937 random(7u);
    std::shuffle(shuffled[1].shapes.begin(), shuffled[1].shapes.end(), random);
    std::shuffle(shuffled[2].shapes.begin(), shuffled[2].shapes.end(), random);
    REQUIRE(shuffled[1].shapes.front().stableKey != original.shapes.front().stableKey);

    for (size_t variant = 0; variant < shuffled.size(); ++variant)
    {
        INFO("shuffle variant " << variant);
        const RestingRun run = runRestingCapsules(shuffled[variant]);
        CHECK(run.stats.bodies == reference.stats.bodies);
        CHECK(run.hash == reference.hash);
        CHECK(sameBytes(run.bytes, reference.bytes));
    }
}

TEST_CASE("JoltStaticWorldBuilder.ThreeHundredElementsBuildWithinBound", "[Jolt][StaticGeometry]")
{
    const StaticWorldDescription description = manyElements(300);
    StaticRig rig(description);
    const JoltStaticBuildStats& stats = rig.builder->stats();

    CHECK(stats.shapesBuilt == 300);
    CHECK(stats.skippedInvalidShape == 0);
    for (size_t typeIndex = 0; typeIndex < kStaticShapeTypeCount; ++typeIndex)
    {
        CHECK(rig.report.shapeCountByType[typeIndex] == 60);
    }
    std::printf("[JoltStaticWorldBuilder] 300 elements -> %u bodies in %.4f s\n", unsigned(stats.bodies.size()), rig.report.buildSeconds);
    CHECK(rig.report.buildSeconds > 0.0);
    CHECK(rig.report.buildSeconds < 1.0);
}

#endif // WITH_LOW_LEVEL_TESTS
