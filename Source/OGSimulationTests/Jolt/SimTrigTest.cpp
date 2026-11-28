// SPDX-License-Identifier: MPL-2.0
// docs/SimTrigTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulationJolt/JoltDefineChecks.h"
#include <Jolt/Math/Trigonometry.h>

#include "OGSimulation/SimTrig.h"

#include "glm/gtc/matrix_transform.hpp"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/vector_angle.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

namespace
{
    std::uint32_t bitsOf(float value)
    {
        return std::bit_cast<std::uint32_t>(value);
    }

    struct BitParity
    {
        std::uint64_t compared = 0;
        std::uint64_t differing = 0;
        float firstInput = 0.f;
        std::uint32_t firstOurs = 0;
        std::uint32_t firstJolt = 0;

        void add(float input, float ours, float jolt)
        {
            ++compared;
            if (bitsOf(ours) == bitsOf(jolt))
                return;
            if (differing == 0)
            {
                firstInput = input;
                firstOurs = bitsOf(ours);
                firstJolt = bitsOf(jolt);
            }
            ++differing;
        }
    };

    template <typename Visit>
    void sweepBothSigns(float high, std::uint32_t stride, Visit visit)
    {
        const std::uint32_t last = bitsOf(high);
        for (std::uint64_t bits = 0; bits <= last; bits += stride)
        {
            const float value = std::bit_cast<float>(static_cast<std::uint32_t>(bits));
            visit(value);
            visit(-value);
        }
    }

    struct MaxError
    {
        double worst = 0.0;

        void add(float ours, double reference)
        {
            worst = std::fmax(worst, std::fabs(static_cast<double>(ours) - reference));
        }
    };
}

TEST_CASE("SimTrig.MatchesJoltBitForBit", "[SimTrig][Jolt]")
{
    BitParity sinParity, cosParity, tanParity, asinParity, acosParity, atanParity, atan2Parity;

    sweepBothSigns(1000.f, 997u, [&](float angle) {
        const simTrig::SinCos sc = simTrig::sinCos(angle);
        sinParity.add(angle, sc.sin, JPH::Sin(angle));
        cosParity.add(angle, sc.cos, JPH::Cos(angle));
        sinParity.add(angle, simTrig::sin(angle), JPH::Sin(angle));
        cosParity.add(angle, simTrig::cos(angle), JPH::Cos(angle));
        tanParity.add(angle, simTrig::tan(angle), JPH::Tan(angle));
    });
    sweepBothSigns(1.5f, 401u, [&](float value) {
        asinParity.add(value, simTrig::asin(value), JPH::ASin(value));
        acosParity.add(value, simTrig::acos(value), JPH::ACos(value));
    });
    sweepBothSigns(std::numeric_limits<float>::max(), 10007u, [&](float value) {
        atanParity.add(value, simTrig::atan(value), JPH::ATan(value));
    });

    std::mt19937 rng(22u);
    std::uniform_real_distribution<float> coordinate(-4.f, 4.f);
    for (int i = 0; i < 200000; ++i)
    {
        float y = coordinate(rng);
        float x = coordinate(rng);
        if (i % 5 == 1)
            y = 0.f;
        if (i % 7 == 2)
            x = -0.f;
        if (i % 11 == 3)
            y = x;
        atan2Parity.add(y, simTrig::atan2(y, x), JPH::ATan2(y, x));
    }

    const BitParity* all[] = {&sinParity, &cosParity, &tanParity, &asinParity, &acosParity, &atanParity, &atan2Parity};
    const char* names[] = {"sin", "cos", "tan", "asin", "acos", "atan", "atan2"};
    for (int k = 0; k < 7; ++k)
    {
        INFO(names[k] << ": " << all[k]->differing << " of " << all[k]->compared << " differ; first at " << all[k]->firstInput
            << " ours 0x" << std::hex << all[k]->firstOurs << " Jolt 0x" << all[k]->firstJolt);
        CHECK(all[k]->compared > 1000u);
        CHECK(all[k]->differing == 0u);
    }
}

TEST_CASE("SimTrig.RotateAndOrientedAngleEqualGlmWhereTheTrigAgrees", "[SimTrig]")
{
    std::mt19937 rng(2211u);
    std::uniform_real_distribution<float> angleOf(-7.f, 7.f);
    std::uniform_real_distribution<float> component(-1.f, 1.f);
    int rotateAgreeing = 0;
    int rotateDiffering = 0;
    int angleAgreeing = 0;
    int angleDiffering = 0;
    for (int i = 0; i < 100000; ++i)
    {
        const float angle = angleOf(rng);
        const glm::vec3 axis(component(rng), component(rng), component(rng));
        glm::mat4 base(1.f);
        base[3] = glm::vec4(component(rng), component(rng), component(rng), 1.f);
        if (glm::dot(axis, axis) > 1e-6f && bitsOf(std::cos(angle)) == bitsOf(simTrig::cos(angle))
            && bitsOf(std::sin(angle)) == bitsOf(simTrig::sin(angle)))
        {
            ++rotateAgreeing;
            const glm::mat4 ours = simTrig::rotate(base, angle, axis);
            const glm::mat4 theirs = glm::rotate(base, angle, axis);
            bool same = true;
            for (int c = 0; c < 4; ++c)
                for (int r = 0; r < 4; ++r)
                    same = same && bitsOf(ours[c][r]) == bitsOf(theirs[c][r]);
            rotateDiffering += same ? 0 : 1;
        }

        const glm::vec2 from = glm::normalize(glm::vec2(component(rng), component(rng)) + glm::vec2(1e-3f, 0.f));
        const glm::vec2 to = glm::normalize(glm::vec2(component(rng), component(rng)) + glm::vec2(0.f, 1e-3f));
        const float clampedDot = glm::clamp(glm::dot(from, to), -1.f, 1.f);
        if (bitsOf(std::acos(clampedDot)) == bitsOf(simTrig::acos(clampedDot)))
        {
            ++angleAgreeing;
            angleDiffering += bitsOf(simTrig::orientedAngle(from, to)) == bitsOf(glm::orientedAngle(from, to)) ? 0 : 1;
        }
    }
    INFO("rotate: " << rotateAgreeing << " samples where libm and simTrig agree; orientedAngle: " << angleAgreeing);
    CHECK(rotateAgreeing > 10000);
    CHECK(rotateDiffering == 0);
    CHECK(angleAgreeing > 10000);
    CHECK(angleDiffering == 0);
}

TEST_CASE("SimTrig.AccuracyAgainstDoublePrecision", "[SimTrig]")
{
    MaxError sinError, cosError, asinError, acosError, atanError, atan2Error;
    sweepBothSigns(100.f, 4099u, [&](float angle) {
        const simTrig::SinCos sc = simTrig::sinCos(angle);
        sinError.add(sc.sin, std::sin(static_cast<double>(angle)));
        cosError.add(sc.cos, std::cos(static_cast<double>(angle)));
    });
    sweepBothSigns(1.f, 1021u, [&](float value) {
        asinError.add(simTrig::asin(value), std::asin(static_cast<double>(value)));
        acosError.add(simTrig::acos(value), std::acos(static_cast<double>(value)));
    });
    sweepBothSigns(1.0e6f, 4099u, [&](float value) {
        atanError.add(simTrig::atan(value), std::atan(static_cast<double>(value)));
    });
    std::mt19937 rng(23u);
    std::uniform_real_distribution<float> coordinate(-4.f, 4.f);
    for (int i = 0; i < 100000; ++i)
    {
        const float y = coordinate(rng);
        const float x = coordinate(rng);
        atan2Error.add(simTrig::atan2(y, x), std::atan2(static_cast<double>(y), static_cast<double>(x)));
    }
    INFO("max abs error: sin " << sinError.worst << " cos " << cosError.worst << " asin " << asinError.worst << " acos "
        << acosError.worst << " atan " << atanError.worst << " atan2 " << atan2Error.worst);
    CHECK(sinError.worst < 1.0e-7);
    CHECK(cosError.worst < 1.0e-7);
    CHECK(asinError.worst < 2.0e-7);
    CHECK(acosError.worst < 5.0e-7);
    CHECK(atanError.worst < 2.0e-7);
    CHECK(atan2Error.worst < 4.0e-7);
}

TEST_CASE("SimTrig.InverseTrigClampsInputsOutsideTheUnitInterval", "[SimTrig]")
{
    CHECK(bitsOf(simTrig::asin(1.0000001f)) == bitsOf(simTrig::kHalfPi));
    CHECK(bitsOf(simTrig::asin(-3.f)) == bitsOf(-simTrig::kHalfPi));
    CHECK(bitsOf(simTrig::acos(1.0000001f)) == bitsOf(0.f));
    CHECK(bitsOf(simTrig::acos(-1.5f)) == bitsOf(simTrig::kPi));
    CHECK(bitsOf(simTrig::acos(1.f)) == bitsOf(0.f));
}

TEST_CASE("SimTrig.RecordedOutputsAreFixed", "[SimTrig]")
{
    CHECK(bitsOf(simTrig::sin(1.f)) == 0x3f576aa4u);
    CHECK(bitsOf(simTrig::cos(1.f)) == 0x3f0a5140u);
    CHECK(bitsOf(simTrig::sin(2.f)) == 0x3f68c7b7u);
    CHECK(bitsOf(simTrig::cos(2.f)) == 0xbed51132u);
    CHECK(bitsOf(simTrig::sin(-0.f)) == 0x80000000u);
    CHECK(bitsOf(simTrig::acos(-1.f)) == 0x40490fdbu);
    CHECK(bitsOf(simTrig::acos(0.f)) == 0x3fc90fdbu);
    CHECK(bitsOf(simTrig::atan(1.f)) == 0x3f490fdbu);
    CHECK(bitsOf(simTrig::atan(2.f)) == 0x3f8db70du);
    CHECK(bitsOf(simTrig::atan2(0.f, -1.f)) == bitsOf(simTrig::kPi));
    CHECK(bitsOf(simTrig::atan2(-0.f, -1.f)) == bitsOf(-simTrig::kPi));
}

#endif
