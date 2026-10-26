// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulation/SimulationFrame.h"
#include "OGSimulationJolt/JoltFrame.h"

#include <Jolt/Geometry/AABox.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

using namespace joltTestRig;

namespace
{
    constexpr float kCylinderHalfHeightMetres = 0.54f;
    constexpr float kRadiusMetres = 0.42f;

    void checkLongestAlongUp(const JPH::Shape& shape)
    {
        const JPH::Vec3 extent = shape.GetLocalBounds().GetExtent();
        INFO("extent " << extent.GetX() << ", " << extent.GetY() << ", " << extent.GetZ() << "; up index " << simFrame::kUpIndex);
        CHECK(extent[simFrame::kUpIndex] > extent[(simFrame::kUpIndex + 1) % 3]);
        CHECK(extent[simFrame::kUpIndex] > extent[(simFrame::kUpIndex + 2) % 3]);
    }
}

TEST_CASE("JoltFrame.CapsuleIsLongestAlongTheFrameUp", "[Jolt]")
{
    RuntimeLease lease;
    const JPH::RefConst<JPH::Shape> shape = joltFrame::capsuleShape(kCylinderHalfHeightMetres, kRadiusMetres);
    REQUIRE(shape != nullptr);
    checkLongestAlongUp(*shape);

    const JPH::Shape::ShapeResult result = joltFrame::capsuleShapeFromSettings(kCylinderHalfHeightMetres, kRadiusMetres);
    REQUIRE(result.IsValid());
    checkLongestAlongUp(*result.Get());
    CHECK(result.Get()->GetSubType() == shape->GetSubType());
}

TEST_CASE("JoltFrame.TheCapsuleIsAPlainCapsuleShape", "[Jolt]")
{
    RuntimeLease lease;
    CHECK(joltFrame::capsuleShape(kCylinderHalfHeightMetres, kRadiusMetres)->GetSubType() == JPH::EShapeSubType::Capsule);
    const JPH::Shape::ShapeResult result = joltFrame::capsuleShapeFromSettings(kCylinderHalfHeightMetres, kRadiusMetres);
    REQUIRE(result.IsValid());
    CHECK(result.Get()->GetSubType() == JPH::EShapeSubType::Capsule);
}

#endif
