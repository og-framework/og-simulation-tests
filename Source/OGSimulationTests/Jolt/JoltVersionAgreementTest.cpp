// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulationJolt/JoltRuntime.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

TEST_CASE("JoltRuntime.VersionIdSeenByThisModuleEqualsTheLibrarys", "[Jolt]")
{
    using JPH::uint64;
    CHECK(JoltRuntime::libraryVersionId() == static_cast<uint64_t>(JPH_VERSION_ID));
}

TEST_CASE("JoltRuntime.ShapeVirtualsAgreeAcrossModules", "[Jolt]")
{
    struct Lease
    {
        Lease() { (void)JoltRuntime::acquire(nullptr); }
        ~Lease() { JoltRuntime::release(); }
    } lease;

    const float radius = 2.0f;
    const JPH::ShapeSettings::ShapeResult created = JPH::SphereShapeSettings(radius).Create();
    REQUIRE(created.IsValid());
    const JPH::Shape& shape = *created.Get();

    CHECK(shape.GetInnerRadius() == radius);
    CHECK(shape.GetVolume() == Catch::Approx(4.0f / 3.0f * JPH::JPH_PI * radius * radius * radius));
    CHECK(shape.GetLocalBounds().mMax.GetX() == radius);
}

#endif // WITH_LOW_LEVEL_TESTS
