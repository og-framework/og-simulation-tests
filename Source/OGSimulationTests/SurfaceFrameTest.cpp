// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "OGSimulation/SurfaceFrame.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace
{
    bool bitsEqual(float a, float b)
    {
        return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b);
    }

    bool bitsEqual(const glm::vec3& a, const glm::vec3& b)
    {
        return bitsEqual(a.x, b.x) && bitsEqual(a.y, b.y) && bitsEqual(a.z, b.z);
    }

    bool bitsEqual(const glm::mat3& a, const glm::mat3& b)
    {
        return bitsEqual(a[0], b[0]) && bitsEqual(a[1], b[1]) && bitsEqual(a[2], b[2]);
    }

    float maxAbsDifference(const glm::vec3& a, const glm::vec3& b)
    {
        const glm::vec3 d = glm::abs(a - b);
        return std::max({d.x, d.y, d.z});
    }

    float maxAbsDifference(const glm::mat3& a, const glm::mat3& b)
    {
        return std::max({maxAbsDifference(a[0], b[0]), maxAbsDifference(a[1], b[1]), maxAbsDifference(a[2], b[2])});
    }

    glm::vec3 tiltedNormal(float tiltRadians, float azimuthRadians)
    {
        const glm::vec3 across = std::cos(azimuthRadians) * simFrame::kRight + std::sin(azimuthRadians) * simFrame::kForward;
        return glm::normalize(std::cos(tiltRadians) * simFrame::kUp + std::sin(tiltRadians) * across);
    }

    std::vector<glm::vec3> seededVectorsWithSignedZeros(std::uint32_t seed, int count)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> spread(-100.f, 100.f);
        std::vector<glm::vec3> out;
        out.reserve(static_cast<std::size_t>(count));
        for (int index = 0; index < count; ++index)
        {
            const float x = spread(rng);
            const float y = spread(rng);
            const float z = spread(rng);
            glm::vec3 v(x, y, z);
            switch (index % 6)
            {
            case 1: v.x = -0.f; break;
            case 2: v.y = 0.f; v.z = -0.f; break;
            case 3: v = glm::vec3(-0.f, -0.f, -0.f); break;
            case 4: v = glm::vec3(0.f, -0.f, 0.f); break;
            default: break;
            }
            out.push_back(v);
        }
        return out;
    }

    glm::mat3 rotation(float angle, const glm::vec3& axis)
    {
        return glm::mat3(glm::rotate(glm::mat4(1.f), angle, axis));
    }

    std::vector<glm::vec3> seededWalkableNormals(std::uint32_t seed, int count)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> tilt(1.e-3f, glm::quarter_pi<float>());
        std::uniform_real_distribution<float> azimuth(0.f, glm::two_pi<float>());
        std::vector<glm::vec3> out;
        out.reserve(static_cast<std::size_t>(count));
        for (int index = 0; index < count; ++index)
        {
            const float t = tilt(rng);
            const float a = azimuth(rng);
            out.push_back(tiltedNormal(t, a));
        }
        return out;
    }
} // namespace

TEST_CASE("SurfaceFrame.FlatNormalsReturnTheirInputBits", "[SimulationFrame]")
{
    const std::vector<glm::vec3> flatNormals = {simFrame::kUp, glm::vec3(-0.f, 1.f, 0.f), glm::vec3(0.f, 1.f, -0.f),
        glm::vec3(-0.f, 1.f, -0.f), glm::vec3(5.e-7f, 1.f, -5.e-7f)};
    const std::vector<glm::vec3> vectors = seededVectorsWithSignedZeros(20261028u, 1000);
    for (const glm::vec3& n : flatNormals)
    {
        REQUIRE(surfaceFrame::isFlat(n));
        CHECK(bitsEqual(surfaceFrame::tiltMatrix(n), glm::mat3(1.f)));
        int mismatches = 0;
        for (const glm::vec3& v : vectors)
        {
            if (!bitsEqual(surfaceFrame::tiltToSurface(n, v), v) || !bitsEqual(surfaceFrame::tiltFromSurface(n, v), v))
            {
                ++mismatches;
            }
        }
        CHECK(mismatches == 0);
    }
}

TEST_CASE("SurfaceFrame.IsFlatIsTheFlattenedLengthAgainstTheTolerance", "[SimulationFrame]")
{
    CHECK(surfaceFrame::isFlat(simFrame::kUp));
    CHECK(surfaceFrame::isFlat(-simFrame::kUp));
    CHECK(surfaceFrame::isFlat(glm::vec3(1.e-6f, 1.f, 0.f)));
    CHECK_FALSE(surfaceFrame::isFlat(glm::vec3(2.e-6f, 1.f, 0.f)));
    CHECK_FALSE(surfaceFrame::isFlat(glm::vec3(0.f, 1.f, 2.e-6f)));
    CHECK_FALSE(surfaceFrame::isFlat(tiltedNormal(glm::radians(10.f), 0.f)));
}

TEST_CASE("SurfaceFrame.WalkableTiltIsARotationTakingUpToTheNormal", "[SimulationFrame]")
{
    const std::vector<glm::vec3> normals = seededWalkableNormals(20261029u, 1000);
    std::mt19937 rng(20261030u);
    std::uniform_real_distribution<float> spread(-1.f, 1.f);
    float worstUp = 0.f;
    float worstOrthonormality = 0.f;
    float worstRoundTrip = 0.f;
    for (const glm::vec3& n : normals)
    {
        REQUIRE_FALSE(surfaceFrame::isFlat(n));
        worstUp = std::max(worstUp, maxAbsDifference(surfaceFrame::tiltToSurface(n, simFrame::kUp), n));
        const glm::mat3 g = surfaceFrame::tiltMatrix(n);
        worstOrthonormality = std::max(worstOrthonormality, maxAbsDifference(glm::transpose(g) * g, glm::mat3(1.f)));
        const float x = spread(rng);
        const float y = spread(rng);
        const float z = spread(rng);
        const glm::vec3 v(x, y, z);
        worstRoundTrip = std::max(worstRoundTrip,
            maxAbsDifference(surfaceFrame::tiltFromSurface(n, surfaceFrame::tiltToSurface(n, v)), v));
    }
    INFO("worst |G kUp - n| = " << worstUp << ", worst |GtG - I| = " << worstOrthonormality
        << ", worst round trip = " << worstRoundTrip);
    CHECK(worstUp <= 2.e-7f);
    CHECK(worstOrthonormality <= 4.e-7f);
    CHECK(worstRoundTrip <= 4.e-7f);
}

TEST_CASE("SurfaceFrame.ThirtyDegreeSlopeTiltsForwardUpTheSlope", "[SimulationFrame]")
{
    const float tilt = glm::radians(30.f);
    const glm::vec3 n(0.f, std::cos(tilt), std::sin(tilt));
    const glm::vec3 forward = surfaceFrame::tiltToSurface(n, simFrame::kForward);
    INFO("tiltToSurface(n, kForward) = (" << forward.x << ", " << forward.y << ", " << forward.z << ")");
    CHECK(maxAbsDifference(forward, glm::vec3(0.f, 0.5f, -0.8660254f)) <= 1.e-7f);
    CHECK(maxAbsDifference(surfaceFrame::tiltMatrix(n)[simFrame::kUpIndex], n) <= 1.e-7f);
}

TEST_CASE("SurfaceFrame.AngleAxisReproducesTheTiltedSwingFrame", "[SimulationFrame]")
{
    const float tiltsDegrees[] = {0.f, 10.f, 30.f, 45.f};
    for (const float tiltDegrees : tiltsDegrees)
    {
        const glm::vec3 n = tiltDegrees == 0.f ? simFrame::kUp : tiltedNormal(glm::radians(tiltDegrees), 0.6f);
        const glm::mat3 g = surfaceFrame::tiltMatrix(n);
        float worst = 0.f;
        float largestAngle = 0.f;
        for (int aimIndex = 0; aimIndex < 3600; ++aimIndex)
        {
            const float aim = glm::radians(0.1f * static_cast<float>(aimIndex));
            const glm::mat3 r = g * rotation(aim, simFrame::kUp);
            const surfaceFrame::AngleAxis angleAxis = surfaceFrame::toAngleAxis(r);
            worst = std::max(worst, maxAbsDifference(rotation(angleAxis.angle, angleAxis.axis), r));
            largestAngle = std::max(largestAngle, angleAxis.angle);
        }
        INFO("tilt " << tiltDegrees << " deg: worst |rotate(toAngleAxis(r)) - r| = " << worst
            << ", largest angle = " << largestAngle);
        CHECK(worst <= 1.e-6f);
        CHECK(largestAngle <= glm::pi<float>());
    }
}

TEST_CASE("SurfaceFrame.AngleAxisIsCanonicalNearAHalfTurn", "[SimulationFrame]")
{
    const std::vector<glm::vec3> axes = {simFrame::kUp, simFrame::kRight, simFrame::kForward,
        tiltedNormal(glm::radians(30.f), 0.f), tiltedNormal(glm::radians(45.f), 2.f), glm::normalize(glm::vec3(1.f, 2.f, -3.f))};
    const float offsets[] = {-1.e-2f, -1.e-3f, -1.e-4f, 0.f, 1.e-4f, 1.e-3f, 1.e-2f};
    for (const glm::vec3& axis : axes)
    {
        for (const float offset : offsets)
        {
            const float angle = glm::pi<float>() + offset;
            const glm::mat3 r = rotation(angle, axis);
            const surfaceFrame::AngleAxis angleAxis = surfaceFrame::toAngleAxis(r);
            INFO("axis (" << axis.x << ", " << axis.y << ", " << axis.z << "), angle " << angle << " -> "
                << angleAxis.angle << " about (" << angleAxis.axis.x << ", " << angleAxis.axis.y << ", "
                << angleAxis.axis.z << ")");
            CHECK(angleAxis.angle >= 0.f);
            CHECK(angleAxis.angle <= glm::pi<float>());
            CHECK(std::abs(glm::length(angleAxis.axis) - 1.f) <= 1.e-6f);
            CHECK(maxAbsDifference(rotation(angleAxis.angle, angleAxis.axis), r) <= 1.e-6f);
            if (offset > 0.f)
            {
                CHECK(glm::dot(angleAxis.axis, axis) < 0.f);
            }
            else if (offset < 0.f)
            {
                CHECK(glm::dot(angleAxis.axis, axis) > 0.f);
            }
        }
    }
}

TEST_CASE("SurfaceFrame.AngleAxisOfTheIdentityIsZeroAboutUp", "[SimulationFrame]")
{
    const surfaceFrame::AngleAxis angleAxis = surfaceFrame::toAngleAxis(glm::mat3(1.f));
    CHECK(angleAxis.angle == 0.f);
    CHECK(bitsEqual(angleAxis.axis, simFrame::kUp));
}

TEST_CASE("SurfaceFrame.SurfaceUpOfAYawIsFlatAndOfATiltIsNot", "[SimulationFrame]")
{
    int notFlat = 0;
    for (int degrees = 0; degrees < 360; ++degrees)
    {
        const float angle = glm::radians(static_cast<float>(degrees));
        if (!surfaceFrame::isFlat(surfaceFrame::surfaceUpOf(glm::rotate(glm::mat4(1.f), angle, simFrame::kUp)))
            || !surfaceFrame::isFlat(surfaceFrame::surfaceUpOf(glm::rotate(glm::mat4(1.f), angle, -simFrame::kUp))))
        {
            ++notFlat;
        }
    }
    CHECK(notFlat == 0);

    const float tilt = glm::radians(30.f);
    const glm::vec3 n(0.f, std::cos(tilt), std::sin(tilt));
    const glm::vec3 up = surfaceFrame::surfaceUpOf(glm::mat4(surfaceFrame::tiltMatrix(n)));
    CHECK_FALSE(surfaceFrame::isFlat(up));
    CHECK(maxAbsDifference(up, n) <= 1.e-7f);
}

#endif
