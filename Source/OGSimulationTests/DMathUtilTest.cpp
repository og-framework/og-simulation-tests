// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/DMathUtil.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <utility>
#include <vector>

namespace
{
    FORCEINLINE void legacyGetRotationMatrix(glm::vec3 fromDirection, glm::vec3 toDirection, glm::mat4& outTransform)
    {
        const float dot = glm::dot(fromDirection, toDirection);
        const bool fromEqualsTo = abs(abs(dot) - 1.f) < 0.0001f;
        if (fromEqualsTo)
            outTransform = glm::mat4(1.0f);

        const float angle = glm::acos(dot);
        const glm::vec3 rotationAxis = glm::normalize(glm::cross(fromDirection, toDirection));
        outTransform = glm::rotate(glm::mat4(1.f), angle, rotationAxis);
    }

    glm::vec3 fromBits(std::uint32_t x, std::uint32_t y, std::uint32_t z)
    {
        return glm::vec3(std::bit_cast<float>(x), std::bit_cast<float>(y), std::bit_cast<float>(z));
    }

    bool allFinite(const glm::mat4& m)
    {
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
                if (!std::isfinite(m[column][row]))
                    return false;
        return true;
    }

    bool sameBits(const glm::mat4& a, const glm::mat4& b)
    {
        return std::memcmp(&a, &b, sizeof(glm::mat4)) == 0;
    }

    glm::vec3 transformDirection(const glm::mat4& m, const glm::vec3& v)
    {
        return glm::vec3(m * glm::vec4(v, 0.f));
    }

    void checkClose(const glm::vec3& actual, const glm::vec3& expected)
    {
        CAPTURE(actual.x, actual.y, actual.z, expected.x, expected.y, expected.z);
        CHECK(actual.x == Catch::Approx(expected.x).margin(1e-6));
        CHECK(actual.y == Catch::Approx(expected.y).margin(1e-6));
        CHECK(actual.z == Catch::Approx(expected.z).margin(1e-6));
    }

    struct DegenerateCase
    {
        glm::vec3 from;
        glm::vec3 to;
        glm::vec3 antiparallelAxis;
    };

    const glm::vec3 kX(1.f, 0.f, 0.f);
    const glm::vec3 kY(0.f, 1.f, 0.f);
    const glm::vec3 kZ(0.f, 0.f, 1.f);
}

TEST_CASE("DMathUtil.GetRotationMatrixExactParallelIsTheIdentity", "[SimulationFrame][DMathUtil]")
{
    const std::vector<DegenerateCase> cases{
        { kY, kY, kZ },
        { -kX, -kX, kY },
        { kZ, kZ, kX },
        { kY, glm::vec3(1e-30f, 1.f, 0.f), kZ },
        { -kX, glm::vec3(-1.f, 0.f, -1e-30f), kY },
    };
    for (const DegenerateCase& c : cases)
    {
        CAPTURE(c.from.x, c.from.y, c.from.z, c.to.x, c.to.y, c.to.z);
        glm::mat4 rotation(0.f);
        dMathUtil::getRotationMatrix(c.from, c.to, c.antiparallelAxis, rotation);
        CHECK(allFinite(rotation));
        CHECK(sameBits(rotation, glm::mat4(1.f)));
    }
}

TEST_CASE("DMathUtil.GetRotationMatrixExactAntiparallelTurnsHalfAboutTheGivenAxis", "[SimulationFrame][DMathUtil]")
{
    const std::vector<DegenerateCase> cases{
        { kY, -kY, kZ },
        { -kX, kX, kY },
        { kZ, -kZ, kY },
        { kZ, -kZ, kX },
        { kY, glm::vec3(1e-30f, -1.f, 0.f), kZ },
        { -kX, glm::vec3(1.f, 0.f, -1e-30f), kY },
    };
    for (const DegenerateCase& c : cases)
    {
        CAPTURE(c.from.x, c.from.y, c.from.z, c.to.x, c.to.y, c.to.z, c.antiparallelAxis.x, c.antiparallelAxis.y,
            c.antiparallelAxis.z);
        glm::mat4 rotation(0.f);
        dMathUtil::getRotationMatrix(c.from, c.to, c.antiparallelAxis, rotation);
        REQUIRE(allFinite(rotation));
        checkClose(transformDirection(rotation, c.from), c.to);
        checkClose(transformDirection(rotation, c.antiparallelAxis), c.antiparallelAxis);
        CHECK(glm::determinant(glm::mat3(rotation)) == Catch::Approx(1.f).margin(1e-6));
    }
}

TEST_CASE("DMathUtil.GetRotationMatrixMatchesTheLegacyExpressionBitForBitWhenTheCrossIsNotZero",
    "[SimulationFrame][DMathUtil]")
{
    std::vector<std::pair<glm::vec3, glm::vec3>> inputs{
        { kY, fromBits(0xb4a17279u, 0xbf800000u, 0x3452fffbu) },
        { kY, fromBits(0xb3a29e7eu, 0xbf800000u, 0x338d6182u) },
        { kY, fromBits(0xb1857297u, 0xbf800000u, 0xb37c2ad6u) },
        { kY, fromBits(0xbb050059u, 0xbf7fffdeu, 0x3351c9b2u) },
        { kY, fromBits(0x3c5674a0u, 0xbf7ffa63u, 0x00000000u) },
        { kY, glm::normalize(glm::vec3(1e-5f, 1.f, 0.f)) },
        { kY, glm::normalize(glm::vec3(0.f, 1.f, 3e-3f)) },
        { kY, glm::normalize(glm::vec3(-2e-3f, -1.f, 0.f)) },
        { -kX, glm::normalize(glm::vec3(-1.f, 0.f, 1e-4f)) },
        { -kX, glm::normalize(glm::vec3(1.f, 0.f, -1e-6f)) },
        { -kX, glm::normalize(glm::vec3(1.f, 0.f, 2e-2f)) },
        { -kX, kZ },
        { -kX, -kZ },
        { kY, kX },
    };
    std::mt19937 random(260272u);
    std::uniform_real_distribution<float> component(-1.f, 1.f);
    while (inputs.size() < 200u)
    {
        const glm::vec3 a(component(random), component(random), component(random));
        const glm::vec3 b(component(random), component(random), component(random));
        if (glm::dot(a, a) < 1e-2f || glm::dot(b, b) < 1e-2f)
            continue;
        inputs.emplace_back(glm::normalize(a), glm::normalize(b));
    }

    int compared = 0;
    for (const auto& [from, to] : inputs)
    {
        CAPTURE(from.x, from.y, from.z, to.x, to.y, to.z);
        glm::mat4 legacy(0.f);
        legacyGetRotationMatrix(from, to, legacy);
        REQUIRE(allFinite(legacy));
        glm::mat4 rotation(0.f);
        dMathUtil::getRotationMatrix(from, to, kZ, rotation);
        CHECK(sameBits(rotation, legacy));
        ++compared;
    }
    CHECK(compared == 200);
}

TEST_CASE("DMathUtil.GetRotationMatrixKeepsTheTiltedTurnForTheProbeAim", "[SimulationFrame][DMathUtil]")
{
    const glm::vec3 probeAim = fromBits(0xb4a17279u, 0xbf800000u, 0x3452fffbu);
    glm::mat4 rotation(0.f);
    dMathUtil::getRotationMatrix(kY, probeAim, kZ, rotation);
    REQUIRE(allFinite(rotation));
    const glm::quat turn = glm::quat_cast(rotation);
    CAPTURE(turn.w, turn.x, turn.y, turn.z);
    CHECK(std::abs(turn.x) > 0.5f);
    checkClose(transformDirection(rotation, kY), probeAim);
}

#endif // WITH_LOW_LEVEL_TESTS
