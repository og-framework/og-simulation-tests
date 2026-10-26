// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/SimulationFrame.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace
{
    bool bitsEqual(float a, float b)
    {
        return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b);
    }

    bool bitsEqual(const glm::vec2& a, const glm::vec2& b)
    {
        return bitsEqual(a.x, b.x) && bitsEqual(a.y, b.y);
    }

    bool bitsEqual(const glm::vec3& a, const glm::vec3& b)
    {
        return bitsEqual(a.x, b.x) && bitsEqual(a.y, b.y) && bitsEqual(a.z, b.z);
    }

    float maxAbsDifference(const glm::vec3& a, const glm::vec3& b)
    {
        const glm::vec3 d = glm::abs(a - b);
        return std::max({d.x, d.y, d.z});
    }

    std::vector<float> sampleFloats()
    {
        std::vector<float> values = {-0.f, 0.f, 1.f, -1.f, 30.f, 42.f, -980.f, 1.0e-30f, -1.0e-40f, 3.0e38f,
            std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()};
        std::mt19937 rng(20261025u);
        std::uniform_real_distribution<float> spread(-5000.f, 5000.f);
        for (int index = 0; index < 64; ++index)
        {
            values.push_back(spread(rng));
        }
        return values;
    }

    glm::vec3 uniformVector(std::mt19937& rng, float limit)
    {
        std::uniform_real_distribution<float> spread(-limit, limit);
        const float x = spread(rng);
        const float y = spread(rng);
        const float z = spread(rng);
        return glm::vec3(x, y, z);
    }
} // namespace

TEST_CASE("SimulationFrame.PicksReturnTheirInputBitsOverEdgeFloats", "[SimulationFrame]")
{
    const std::vector<float> values = sampleFloats();
    for (std::size_t index = 0; index + 3 < values.size(); ++index)
    {
        const float x = values[index];
        const float y = values[index + 1];
        const float z = values[index + 2];
        const float w = values[index + 3];
        const glm::vec3 v(x, y, z);
        CHECK(bitsEqual(simFrame::height(v), y));
        CHECK(bitsEqual(simFrame::withHeight(v, w), glm::vec3(x, w, z)));
        CHECK(bitsEqual(simFrame::planar(v), glm::vec2(-z, x)));
        CHECK(bitsEqual(simFrame::compose(glm::vec2(x, y), z), glm::vec3(y, z, -x)));
        CHECK(bitsEqual(simFrame::flatten(v), glm::vec3(x, 0.f, z)));
        CHECK(bitsEqual(simFrame::turnComponent(v), -y));
        CHECK(bitsEqual(simFrame::forwardRight(v), glm::vec2(-z, x)));
        CHECK(bitsEqual(simFrame::fromForwardRight(glm::vec2(x, y), z), glm::vec3(y, z, -x)));
        CHECK(bitsEqual(simFrame::alongUp(v), y));
        const glm::mat4 m(glm::vec4(w, x, y, z), glm::vec4(z, w, x, y), glm::vec4(x, y, z, w), glm::vec4(y, z, w, x));
        CHECK(bitsEqual(simFrame::forwardColumn(m), -v));
    }

    CHECK(std::signbit(simFrame::height(glm::vec3(1.f, -0.f, 1.f))));
    CHECK(std::signbit(simFrame::planar(glm::vec3(1.f, 1.f, 0.f)).x));
    CHECK(std::signbit(simFrame::planar(glm::vec3(-0.f, 1.f, 1.f)).y));
    CHECK(std::signbit(simFrame::turnComponent(glm::vec3(1.f, 0.f, 1.f))));
    CHECK(std::signbit(simFrame::forwardRight(glm::vec3(-0.f, 1.f, 1.f)).y));
    CHECK(std::signbit(simFrame::compose(glm::vec2(0.f, 1.f), 1.f).z));
    CHECK(std::signbit(simFrame::withHeight(glm::vec3(1.f), -0.f).y));
    const glm::mat4 signedZeroColumns(glm::vec4(1.f), glm::vec4(1.f), glm::vec4(0.f, 1.f, 0.f, 0.f), glm::vec4(1.f));
    CHECK(std::signbit(simFrame::forwardColumn(signedZeroColumns).x));
    CHECK(std::signbit(simFrame::forwardColumn(signedZeroColumns).z));
    CHECK(bitsEqual(simFrame::forwardColumn(glm::mat4(1.f)), glm::vec3(-0.f, -0.f, -1.f)));
}

TEST_CASE("SimulationFrame.AxisTableAndIdentities", "[SimulationFrame]")
{
    CHECK(simFrame::kForward == glm::vec3(0.f, 0.f, -1.f));
    CHECK(simFrame::kRight == glm::vec3(1.f, 0.f, 0.f));
    CHECK(simFrame::kUp == glm::vec3(0.f, 1.f, 0.f));
    CHECK(simFrame::kTurnAxis == glm::vec3(0.f, -1.f, 0.f));
    CHECK(simFrame::kVerticalTurnAxis == glm::vec3(-1.f, 0.f, 0.f));
    CHECK(simFrame::kChartOrientation == -1.f);
    CHECK(simFrame::kUpIndex == 1);
    CHECK(simFrame::kUeCentimetresPerUnit == 100.f);

    CHECK(glm::cross(simFrame::kForward, simFrame::kRight) == simFrame::kTurnAxis);
    CHECK(glm::cross(simFrame::kUp, simFrame::kForward) == simFrame::kVerticalTurnAxis);
    CHECK(glm::dot(glm::cross(simFrame::kForward, simFrame::kRight), simFrame::kUp) == simFrame::kChartOrientation);
}

TEST_CASE("SimulationFrame.PicksMatchTheTables", "[SimulationFrame]")
{
    const glm::vec3 v(2.f, 3.f, 5.f);
    CHECK(bitsEqual(simFrame::height(v), 3.f));
    CHECK(bitsEqual(simFrame::planar(v), glm::vec2(-5.f, 2.f)));
    CHECK(bitsEqual(simFrame::forwardRight(v), glm::vec2(-5.f, 2.f)));
    CHECK(bitsEqual(simFrame::compose(glm::vec2(7.f, 11.f), 13.f), glm::vec3(11.f, 13.f, -7.f)));
    CHECK(bitsEqual(simFrame::fromForwardRight(glm::vec2(7.f, 11.f), 13.f), glm::vec3(11.f, 13.f, -7.f)));
    CHECK(bitsEqual(simFrame::flatten(v), glm::vec3(2.f, 0.f, 5.f)));
    CHECK(bitsEqual(simFrame::withHeight(v, 17.f), glm::vec3(2.f, 17.f, 5.f)));
    CHECK(bitsEqual(simFrame::turnComponent(v), -3.f));
    CHECK(bitsEqual(simFrame::alongUp(v), 3.f));
    const glm::mat4 columns(glm::vec4(2.f, 3.f, 5.f, 0.f), glm::vec4(7.f, 11.f, 13.f, 0.f), glm::vec4(17.f, -0.f, 19.f, 0.f),
        glm::vec4(23.f, 29.f, 31.f, 1.f));
    CHECK(bitsEqual(simFrame::forwardColumn(columns), glm::vec3(-17.f, 0.f, -19.f)));
}

TEST_CASE("SimulationFrame.PicksRoundTripExactly", "[SimulationFrame]")
{
    const std::vector<float> values = sampleFloats();
    for (std::size_t index = 0; index + 2 < values.size(); ++index)
    {
        const glm::vec3 v(values[index], values[index + 1], values[index + 2]);
        const glm::vec2 p(values[index], values[index + 1]);
        const float h = values[index + 2];
        CHECK(bitsEqual(simFrame::compose(simFrame::planar(v), simFrame::height(v)), v));
        CHECK(bitsEqual(simFrame::planar(simFrame::compose(p, h)), p));
        CHECK(bitsEqual(simFrame::height(simFrame::compose(p, h)), h));
        CHECK(bitsEqual(simFrame::fromForwardRight(simFrame::forwardRight(v), simFrame::height(v)), v));
        CHECK(bitsEqual(simFrame::forwardRight(simFrame::fromForwardRight(p, h)), p));
        CHECK(bitsEqual(simFrame::height(simFrame::fromForwardRight(p, h)), h));
        CHECK(bitsEqual(simFrame::flatten(v), simFrame::compose(simFrame::planar(v), 0.f)));
        CHECK(bitsEqual(simFrame::withHeight(v, h), simFrame::compose(simFrame::planar(v), h)));
        CHECK(bitsEqual(simFrame::alongUp(v), simFrame::height(v)));
    }
}

TEST_CASE("SimulationFrame.ForwardColumnFollowsTheRotatedForward", "[SimulationFrame]")
{
    std::mt19937 rng(11u);
    std::uniform_real_distribution<float> angle(-3.f, 3.f);
    for (int index = 0; index < 500; ++index)
    {
        const float turn = angle(rng);
        const glm::mat4 turned = glm::rotate(glm::mat4(1.f), turn, simFrame::kTurnAxis);
        const glm::vec3 expected = glm::angleAxis(turn, simFrame::kTurnAxis) * simFrame::kForward;
        CHECK(maxAbsDifference(simFrame::forwardColumn(turned), expected) <= 1.0e-6f);

        const glm::vec3 axis = glm::normalize(uniformVector(rng, 1.f));
        const glm::mat4 rotated = glm::rotate(glm::mat4(1.f), turn, axis);
        CHECK(maxAbsDifference(simFrame::forwardColumn(rotated), glm::angleAxis(turn, axis) * simFrame::kForward) <= 1.0e-6f);
    }
}

#endif
