// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltStaticWorldBuilder.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include "glm/gtc/matrix_transform.hpp"
#include "OGSimulation/SimulationFrame.h"

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

    glm::mat4 placed(const glm::vec3& position, float yawDegrees = 0.f, float rollDegrees = 0.f)
    {
        glm::mat4 transform = glm::translate(glm::mat4(1.f), position);
        transform = glm::rotate(transform, glm::radians(yawDegrees), glm::vec3(-0.f, -1.f, 0.f));
        transform = glm::rotate(transform, glm::radians(rollDegrees), glm::vec3(-0.f, -0.f, 1.f));
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

    StaticConvexHull boxHull(const glm::vec3& extents)
    {
        StaticConvexHull hull;
        for (int corner = 0; corner < 8; ++corner)
        {
            hull.points.push_back(glm::vec3((corner & 1) ? extents.x : -extents.x, (corner & 2) ? extents.y : -extents.y,
                (corner & 4) ? extents.z : -extents.z));
        }
        return hull;
    }

    StaticTriangleMesh quadFacingUp(float halfSize)
    {
        StaticTriangleMesh mesh;
        mesh.vertices = { glm::vec3(-halfSize, 0.f, halfSize), glm::vec3(-halfSize, 0.f, -halfSize),
            glm::vec3(halfSize, 0.f, -halfSize), glm::vec3(halfSize, 0.f, halfSize) };
        mesh.indices = { 0, 2, 1, 0, 3, 2 };
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

    std::optional<glm::vec3> castRay(const JoltWorld& world, const glm::vec3& from, const glm::vec3& to)
    {
        const glm::vec3 direction = to - from;
        const JPH::RRayCast ray{ JPH::RVec3(from.x, from.y, from.z), JPH::Vec3(direction.x, direction.y, direction.z) };
        JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> collector;
        world.query().CastRay(ray, JPH::RayCastSettings(), collector, {}, StaticClassOnly(world.layers()));
        if (!collector.HadHit())
        {
            return std::nullopt;
        }
        return from + (to - from) * collector.mHit.mFraction;
    }

    std::optional<float> topHeight(const JoltWorld& world, const glm::vec3& column)
    {
        const std::optional<glm::vec3> hit = castRay(world, glm::vec3(column.x, 50.f, column.z), glm::vec3(column.x, -50.f, column.z));
        return hit.has_value() ? std::optional<float>(simFrame::height(*hit)) : std::nullopt;
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
        description.shapes.push_back(element(boxHull(glm::vec3(20.f, 0.5f, 20.f)), placed(glm::vec3(0.f, -0.5f, -0.f)), keyOf(index++)));
        description.shapes.push_back(element(StaticSphere{ 0.4f }, placed(glm::vec3(-0.75f, 0.2f, 2.25f)), keyOf(index++)));
        description.shapes.push_back(element(StaticBox{ glm::vec3(0.4f, 0.25f, 0.6f) }, placed(glm::vec3(-0.75f, 0.25f, 0.75f), 20.f), keyOf(index++)));
        description.shapes.push_back(element(StaticCapsule{ 0.3f, 0.6f }, placed(glm::vec3(-0.75f, 0.6f, -0.75f)), keyOf(index++)));
        StaticTriangleMesh ramp = quadFacingUp(0.8f);
        description.shapes.push_back(element(ramp, placed(glm::vec3(-0.75f, 0.4f, -2.25f), 0.f, 25.f), keyOf(index++)));
        description.shapes.push_back(element(boxHull(glm::vec3(0.5f, 0.15f, 0.5f)), placed(glm::vec3(0.75f, 0.15f, 2.25f), 45.f), keyOf(index++), bit(kWorld),
            kWorldBlockingCharacterOnly, 0.5f, 0.1f));
        description.shapes.push_back(element(StaticBox{ glm::vec3(0.3f, 0.1f, 0.3f) }, placed(glm::vec3(0.75f, 0.1f, 0.75f), -15.f), keyOf(index++), bit(kWorld),
            kWorldBlockingCharacterOnly));
        description.shapes.push_back(element(StaticSphere{ 0.25f }, placed(glm::vec3(0.75f, 0.f, -0.75f)), keyOf(index++), bit(kWorld), kAllMappedCategories, 0.f, 0.f));
        for (int peg = 0; peg < 12; ++peg)
        {
            const float angle = 0.5236f * float(peg);
            description.shapes.push_back(element(StaticBox{ glm::vec3(0.1f, 0.4f, 0.1f) }, placed(glm::vec3(4.5f * std::sin(angle), 0.4f, -(4.5f * std::cos(angle))), 7.f * float(peg)),
                keyOf(index++), bit(kWorld), peg % 2 == 0 ? kAllMappedCategories : kWorldBlockingCharacterOnly, peg % 3 == 0 ? 0.5f : 0.7f, 0.3f));
        }
        return description;
    }

    struct RestingRun
    {
        std::vector<uint8_t> bytes;
        uint64_t hash = 0;
        std::vector<float> capsuleHeights;
        JoltStaticBuildStats stats;
    };

    RestingRun runRestingCapsules(const StaticWorldDescription& description)
    {
        JoltStaticWorldBuilderOptions options;
        options.maxShapesPerChunk = 3;
        StaticRig rig(description, options);
        JoltWorld& world = *rig.world;
        world.applyBodySlotOccupancy(allOccupied());
        placeAllBrawlers(world);
        for (uint32_t tick = 1; tick <= 300; ++tick)
        {
            world.applyBodySlotOccupancy(allOccupied());
            world.step(kDt);
        }
        RestingRun run;
        run.bytes = fullState(world);
        run.hash = world.liveStateHash();
        for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
        {
            run.capsuleHeights.push_back(joltHeight(world.bodies().GetCenterOfMassPosition(world.slotBodyId(slot, kTemplateCapsule))));
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
            const float forward = 40.f * unit(random) - 20.f;
            const float right = 40.f * unit(random) - 20.f;
            const float up = 3.f * unit(random);
            const float yawDegrees = 360.f * unit(random);
            const float rollDegrees = 30.f * unit(random);
            const glm::mat4 transform = placed(glm::vec3(right, up, -forward), yawDegrees, rollDegrees);
            StaticShape shape;
            switch (index % 5)
            {
            case 0:
            {
                const glm::vec3 forwardRightUp{ 0.2f + 0.8f * unit(random), 0.2f + 0.8f * unit(random), 0.1f + 0.4f * unit(random) };
                shape = StaticBox{ glm::vec3(forwardRightUp.y, forwardRightUp.z, forwardRightUp.x) };
                break;
            }
            case 1: shape = StaticSphere{ 0.1f + 0.5f * unit(random) }; break;
            case 2: shape = StaticCapsule{ 0.15f, 0.3f + 0.6f * unit(random) }; break;
            case 3:
            {
                StaticConvexHull hull;
                for (int point = 0; point < 16; ++point)
                {
                    const glm::vec3 forwardRightUp{ unit(random) - 0.5f, unit(random) - 0.5f, 0.6f * unit(random) - 0.3f };
                    hull.points.push_back(glm::vec3(forwardRightUp.y, forwardRightUp.z, -forwardRightUp.x));
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
                        mesh.vertices.push_back(glm::vec3(0.4f * float(y), 0.1f * unit(random), -(0.4f * float(x))));
                    }
                }
                for (uint32_t y = 0; y < 4; ++y)
                {
                    for (uint32_t x = 0; x < 4; ++x)
                    {
                        const uint32_t corner = y * 5u + x;
                        mesh.indices.insert(mesh.indices.end(), { corner, corner + 6u, corner + 1u, corner, corner + 5u, corner + 6u });
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
    description.shapes.push_back(element(StaticBox{ glm::vec3(0.5f, 0.3f, 2.f) }, placed(glm::vec3(0.f, 1.f, -10.f), 90.f), keyOf(0)));
    description.shapes.push_back(element(StaticSphere{ 0.5f }, placed(glm::vec3(10.f, 2.f, -0.f)), keyOf(1)));
    description.shapes.push_back(element(StaticCapsule{ 0.4f, 1.2f }, placed(glm::vec3(0.f, 2.f, 10.f)), keyOf(2)));
    StaticConvexHull pyramid;
    pyramid.points = { glm::vec3(-1.f, 0.f, 1.f), glm::vec3(-1.f, 0.f, -1.f),
        glm::vec3(1.f, 0.f, -1.f), glm::vec3(1.f, 0.f, 1.f), glm::vec3(0.f, 1.5f, -0.f) };
    description.shapes.push_back(element(pyramid, placed(glm::vec3(-10.f, 0.5f, -0.f), 0.f, 180.f), keyOf(3)));
    description.shapes.push_back(element(quadFacingUp(1.f), placed(glm::vec3(10.f, 0.75f, -10.f), 45.f), keyOf(4)));

    StaticRig rig(description);
    const JoltWorld& world = *rig.world;
    REQUIRE(rig.builder->stats().shapesBuilt == 5);
    for (const size_t typeIndex : { kStaticShapeIndex<StaticBox>, kStaticShapeIndex<StaticSphere>, kStaticShapeIndex<StaticCapsule>,
             kStaticShapeIndex<StaticConvexHull>, kStaticShapeIndex<StaticTriangleMesh> })
    {
        CHECK(rig.report.shapeCountByType[typeIndex] == 1);
    }

    constexpr float kTolerance = 0.0005f;

    SECTION("box: yawed 90 degrees, so its long axis runs along the frame's right")
    {
        CHECK(topHeight(world, glm::vec3(0.f, 0.f, -10.f)).value_or(-0.01f) == Catch::Approx(1.3f).margin(kTolerance));
        CHECK(topHeight(world, glm::vec3(1.5f, 0.f, -10.f)).value_or(-0.01f) == Catch::Approx(1.3f).margin(kTolerance));
        CHECK_FALSE(topHeight(world, glm::vec3(0.f, 0.f, -11.5f)).has_value());
    }
    SECTION("sphere: top at centre + radius, and the curve off-centre")
    {
        CHECK(topHeight(world, glm::vec3(10.f, 0.f, -0.f)).value_or(-0.01f) == Catch::Approx(2.5f).margin(kTolerance));
        CHECK(topHeight(world, glm::vec3(10.3f, 0.f, -0.f)).value_or(-0.01f) == Catch::Approx(2.4f).margin(kTolerance));
        CHECK_FALSE(topHeight(world, glm::vec3(10.6f, 0.f, -0.f)).has_value());
    }
    SECTION("capsule: along up, totalHalfHeight to the tip, radius on the side")
    {
        CHECK(topHeight(world, glm::vec3(0.f, 0.f, 10.f)).value_or(-0.01f) == Catch::Approx(3.2f).margin(kTolerance));
        const std::optional<glm::vec3> side =
            castRay(world, glm::vec3(0.f, 2.5f, 5.f), glm::vec3(0.f, 2.5f, 15.f));
        REQUIRE(side.has_value());
        CHECK(simFrame::forwardRight(*side).x == Catch::Approx(-9.6f).margin(kTolerance));
        CHECK_FALSE(topHeight(world, glm::vec3(0.f, 0.f, 9.5f)).has_value());
    }
    SECTION("convex: an upside-down pyramid, so its flat base is the top face")
    {
        CHECK(topHeight(world, glm::vec3(-10.f, 0.f, -0.f)).value_or(-0.01f) == Catch::Approx(0.5f).margin(kTolerance));
        CHECK(topHeight(world, glm::vec3(-10.8f, 0.f, -0.8f)).value_or(-0.01f) == Catch::Approx(0.5f).margin(kTolerance));
        CHECK_FALSE(topHeight(world, glm::vec3(-10.f, 0.f, -1.2f)).has_value());
    }
    SECTION("triangle mesh: yawed 45 degrees, front face up, back face ignored by a default ray")
    {
        CHECK(topHeight(world, glm::vec3(10.f, 0.f, -10.f)).value_or(-0.01f) == Catch::Approx(0.75f).margin(kTolerance));
        CHECK(topHeight(world, glm::vec3(10.f, 0.f, -11.3f)).value_or(-0.01f) == Catch::Approx(0.75f).margin(kTolerance));
        CHECK_FALSE(topHeight(world, glm::vec3(10.f, 0.f, -11.5f)).has_value());
        CHECK_FALSE(castRay(world, glm::vec3(10.f, -5.f, -10.f), glm::vec3(10.f, 5.f, -10.f)).has_value());
    }
}

TEST_CASE("JoltStaticWorldBuilder.GroupsByCollisionPairAndSurfaceIntoStaticBodiesOnTheirLayer", "[Jolt][StaticGeometry]")
{
    StaticWorldDescription description;
    for (uint64_t index = 0; index < 3; ++index)
    {
        description.shapes.push_back(element(StaticBox{ glm::vec3(0.2f, 0.2f, 0.2f) }, placed(glm::vec3(0.f, 0.2f, -float(index))), keyOf(index)));
    }
    for (uint64_t index = 3; index < 5; ++index)
    {
        description.shapes.push_back(element(StaticBox{ glm::vec3(0.2f, 0.2f, 0.2f) }, placed(glm::vec3(0.f, 0.2f, -float(index))), keyOf(index), bit(kWorld),
            kAllMappedCategories, 0.5f, 0.1f));
    }
    for (uint64_t index = 5; index < 7; ++index)
    {
        description.shapes.push_back(element(StaticSphere{ 0.2f }, placed(glm::vec3(0.f, 0.2f, -float(index))), keyOf(index), bit(kWorld),
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
    description.shapes.push_back(element(StaticBox{ glm::vec3(0.2f, 0.2f, 0.2f) }, placed(glm::vec3(0.f, 0.2f, -0.f)), keyOf(0), bit(kWorld), kAllMappedCategories, 0.f, 0.f));
    description.shapes.push_back(element(StaticBox{ glm::vec3(0.2f, 0.2f, 0.2f) }, placed(glm::vec3(0.f, 0.2f, -1.f)), keyOf(1), bit(kWorld), kAllMappedCategories, 0.f, 0.4f));

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
    description.shapes.push_back(element(StaticBox{ glm::vec3(0.2f, 0.2f, 0.2f) }, glm::scale(placed(glm::vec3(0.f, 0.4f, -0.f)), glm::vec3(2.f)), keyOf(0)));
    description.shapes.push_back(element(StaticSphere{ 0.2f }, glm::scale(placed(glm::vec3(0.f, 0.f, -3.f)), glm::vec3(1.f, 2.f, 1.f)), keyOf(1)));
    description.shapes.push_back(element(StaticConvexHull{}, placed(glm::vec3(0.f, 0.f, -6.f)), keyOf(2)));
    StaticTriangleMesh broken = quadFacingUp(0.5f);
    broken.indices.back() = 17;
    description.shapes.push_back(element(broken, placed(glm::vec3(0.f, 0.f, -9.f)), keyOf(3)));
    description.shapes.push_back(element(StaticCapsule{ 0.4f, 0.3f }, placed(glm::vec3(0.f, 0.f, -12.f)), keyOf(4)));
    description.shapes.push_back(element(StaticBox{ glm::vec3(0.2f, 0.2f, 0.2f) }, glm::scale(placed(glm::vec3(0.f, 0.f, -15.f)), glm::vec3(0.f)), keyOf(5)));

    StaticRig rig(description);
    const JoltStaticBuildStats& stats = rig.builder->stats();
    CHECK(stats.shapesBuilt == 2);
    CHECK(stats.skippedInvalidShape == 4);
    CHECK(stats.approximatedScales == 1);
    CHECK(rig.log.countContaining("skipped an element") == 4);
    CHECK(rig.log.countContaining("approximated a scale (stableKey 0x") == 1);
    CHECK(topHeight(*rig.world, glm::vec3(0.f, 0.f, -0.f)).value_or(-0.01f) == Catch::Approx(0.8f).margin(0.0005f));
    CHECK(topHeight(*rig.world, glm::vec3(0.f, 0.f, -3.f)).has_value());
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
    for (const float height : reference.capsuleHeights)
    {
        CHECK(height > 0.9f);
        CHECK(height < 2.5f);
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
