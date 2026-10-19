// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "catch_amalgamated.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "OGSimulation/StaticGeometry.h"
#include "../Mocks/MockPhysicsWorld.h"

namespace
{
StaticShapeDescriptor describeStaticShape(StaticShape shape, uint32_t index)
{
    StaticShapeDescriptor descriptor;
    descriptor.shape = std::move(shape);
    descriptor.localToWorld = glm::translate(glm::mat4{ 1.f }, glm::vec3{ 100.f * index, -20.f, 5.f + index });
    descriptor.categories = CollisionCategories::single(index);
    descriptor.blockingCategories = CollisionCategories::single(index + 1) | CollisionCategories::single(31);
    descriptor.friction = 0.1f * static_cast<float>(index + 1);
    descriptor.restitution = 0.05f * static_cast<float>(index + 1);
    descriptor.stableKey = 0xA5A5000000000000ull + index;
    return descriptor;
}

StaticWorldDescription staticArenaWithEveryShape()
{
    StaticWorldDescription description;
    description.shapes.push_back(describeStaticShape(StaticBox{ glm::vec3{ 500.f, 400.f, 25.f } }, 0));
    description.shapes.push_back(describeStaticShape(StaticSphere{ 75.f }, 1));
    description.shapes.push_back(describeStaticShape(StaticCapsuleZ{ 30.f, 90.f }, 2));
    description.shapes.push_back(describeStaticShape(StaticConvexHull{ { { 0.f, 0.f, 0.f }, { 10.f, 0.f, 0.f }, { 0.f, 10.f, 0.f }, { 0.f, 0.f, 10.f } } }, 3));
    description.shapes.push_back(describeStaticShape(StaticTriangleMesh{ { { 0.f, 0.f, 0.f }, { 10.f, 0.f, 0.f }, { 0.f, 10.f, 0.f }, { 10.f, 10.f, 0.f } }, { 0, 1, 2, 2, 1, 3 } }, 4));
    description.shapes.push_back(describeStaticShape(StaticBox{ glm::vec3{ 1.f, 2.f, 3.f } }, 5));
    return description;
}

void requireSameStaticDescriptor(const StaticShapeDescriptor& received, const StaticShapeDescriptor& sent)
{
    REQUIRE(received.shape.index() == sent.shape.index());
    CHECK(received.localToWorld == sent.localToWorld);
    CHECK(received.categories.bits == sent.categories.bits);
    CHECK(received.blockingCategories.bits == sent.blockingCategories.bits);
    CHECK(received.friction == sent.friction);
    CHECK(received.restitution == sent.restitution);
    CHECK(received.stableKey == sent.stableKey);

    std::visit([&received](const auto& sentShape)
    {
        using Shape = std::decay_t<decltype(sentShape)>;
        const Shape& receivedShape = std::get<Shape>(received.shape);
        if constexpr (std::is_same_v<Shape, StaticBox>)
        {
            CHECK(receivedShape.halfExtents == sentShape.halfExtents);
        }
        else if constexpr (std::is_same_v<Shape, StaticSphere>)
        {
            CHECK(receivedShape.radius == sentShape.radius);
        }
        else if constexpr (std::is_same_v<Shape, StaticCapsuleZ>)
        {
            CHECK(receivedShape.radius == sentShape.radius);
            CHECK(receivedShape.totalHalfHeight == sentShape.totalHalfHeight);
        }
        else if constexpr (std::is_same_v<Shape, StaticConvexHull>)
        {
            CHECK(receivedShape.points == sentShape.points);
        }
        else
        {
            static_assert(std::is_same_v<Shape, StaticTriangleMesh>, "a new StaticShape alternative needs a round-trip arm");
            CHECK(receivedShape.vertices == sentShape.vertices);
            CHECK(receivedShape.indices == sentShape.indices);
        }
    }, sent.shape);
}
}

TEST_CASE("StaticGeometry descriptors round-trip through a StaticWorldBuilder intact", "[StaticGeometry]")
{
    const StaticWorldDescription sent = staticArenaWithEveryShape();
    MockStaticWorldBuilder builder;

    const StaticWorldBuildReport report = builder.build(sent);

    REQUIRE(builder.received().size() == 1);
    const StaticWorldDescription& received = builder.received().front();
    REQUIRE(received.shapes.size() == sent.shapes.size());
    for (std::size_t i = 0; i < sent.shapes.size(); ++i)
    {
        INFO("shape " << i);
        requireSameStaticDescriptor(received.shapes[i], sent.shapes[i]);
    }

    CHECK(report.countOf<StaticBox>() == 2);
    CHECK(report.countOf<StaticSphere>() == 1);
    CHECK(report.countOf<StaticCapsuleZ>() == 1);
    CHECK(report.countOf<StaticConvexHull>() == 1);
    CHECK(report.countOf<StaticTriangleMesh>() == 1);
}

TEST_CASE("StaticGeometry holds exactly the five M1 shapes in a fixed order", "[StaticGeometry]")
{
    STATIC_REQUIRE(kStaticShapeTypeCount == 5);
    STATIC_REQUIRE(kStaticShapeIndex<StaticBox> == 0);
    STATIC_REQUIRE(kStaticShapeIndex<StaticSphere> == 1);
    STATIC_REQUIRE(kStaticShapeIndex<StaticCapsuleZ> == 2);
    STATIC_REQUIRE(kStaticShapeIndex<StaticConvexHull> == 3);
    STATIC_REQUIRE(kStaticShapeIndex<StaticTriangleMesh> == 4);
    STATIC_REQUIRE(std::tuple_size_v<decltype(StaticWorldBuildReport{}.shapeCountByType)> == kStaticShapeTypeCount);
}

TEST_CASE("StaticGeometry a default descriptor is an identity-placed box in no category that blocks nothing", "[StaticGeometry]")
{
    const StaticShapeDescriptor descriptor;
    CHECK(std::holds_alternative<StaticBox>(descriptor.shape));
    CHECK(descriptor.localToWorld == glm::mat4{ 1.f });
    CHECK(descriptor.categories.bits == 0u);
    CHECK(descriptor.blockingCategories.bits == 0u);
    CHECK(descriptor.stableKey == 0u);

    const StaticWorldBuildReport report;
    for (const uint32_t count : report.shapeCountByType)
    {
        CHECK(count == 0u);
    }
    CHECK(report.buildSeconds == 0.0);
}

#endif // WITH_LOW_LEVEL_TESTS
