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
#include <string>
#include <vector>

namespace
{
    using UeCm = BasicSimFrame<AxisFrame::UeZUpLeftHanded, 100.f>;
    using UeM = BasicSimFrame<AxisFrame::UeZUpLeftHanded, 1.f>;
    using YCm = BasicSimFrame<AxisFrame::YUpRightHanded, 100.f>;
    using YM = BasicSimFrame<AxisFrame::YUpRightHanded, 1.f>;

    static_assert(simFrameDetail::picksKeepTheSignOfZero<UeCm>(-0.f) && simFrameDetail::picksKeepTheSignOfZero<UeCm>(0.f));
    static_assert(simFrameDetail::picksKeepTheSignOfZero<UeM>(-0.f) && simFrameDetail::picksKeepTheSignOfZero<UeM>(0.f));
    static_assert(simFrameDetail::picksKeepTheSignOfZero<YCm>(-0.f) && simFrameDetail::picksKeepTheSignOfZero<YCm>(0.f));
    static_assert(simFrameDetail::picksKeepTheSignOfZero<YM>(-0.f) && simFrameDetail::picksKeepTheSignOfZero<YM>(0.f));

    static_assert(UeM::legacyCm(30.f) == 0.3f && UeM::legacyCm(42.f) == 0.42f && YM::legacyCm2(2500.f) == 0.25f);


    bool bitsEqual(float a, float b)
    {
        return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b);
    }

    bool bitsEqual(double a, double b)
    {
        return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
    }

    bool bitsEqual(const glm::vec2& a, const glm::vec2& b)
    {
        return bitsEqual(a.x, b.x) && bitsEqual(a.y, b.y);
    }

    bool bitsEqual(const glm::vec3& a, const glm::vec3& b)
    {
        return bitsEqual(a.x, b.x) && bitsEqual(a.y, b.y) && bitsEqual(a.z, b.z);
    }

    bool bitsEqual(const glm::dvec3& a, const glm::dvec3& b)
    {
        return bitsEqual(a.x, b.x) && bitsEqual(a.y, b.y) && bitsEqual(a.z, b.z);
    }

    bool bitsEqual(const glm::quat& a, const glm::quat& b)
    {
        return bitsEqual(a.w, b.w) && bitsEqual(a.x, b.x) && bitsEqual(a.y, b.y) && bitsEqual(a.z, b.z);
    }

    float maxAbsDifference(const glm::vec3& a, const glm::vec3& b)
    {
        const glm::vec3 d = glm::abs(a - b);
        return std::max({d.x, d.y, d.z});
    }

    double maxAbsDifference(const glm::dvec3& a, const glm::dvec3& b)
    {
        const glm::dvec3 d = glm::abs(a - b);
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

    template <class Frame>
    glm::vec3 mapAxes(const glm::vec3& ue)
    {
        return Frame::legacyAxes(ue.x, ue.y, ue.z);
    }

    template <class Frame>
    glm::quat mapRotation(const glm::quat& ue)
    {
        return Frame::legacyRotation(ue.w, ue.x, ue.y, ue.z);
    }

    template <class Frame>
    void checkAxisIdentities()
    {
        CHECK(glm::cross(Frame::kForward, Frame::kRight) == Frame::kTurnAxis);
        CHECK(glm::cross(Frame::kUp, Frame::kForward) == Frame::kVerticalTurnAxis);
        CHECK(glm::dot(glm::cross(Frame::kForward, Frame::kRight), Frame::kUp) == Frame::kChartOrientation);
    }

    template <class Frame>
    void checkPicksRoundTrip()
    {
        const std::vector<float> values = sampleFloats();
        for (std::size_t index = 0; index + 2 < values.size(); ++index)
        {
            const glm::vec3 v(values[index], values[index + 1], values[index + 2]);
            const glm::vec2 p(values[index], values[index + 1]);
            const float h = values[index + 2];
            CHECK(bitsEqual(Frame::compose(Frame::planar(v), Frame::height(v)), v));
            CHECK(bitsEqual(Frame::planar(Frame::compose(p, h)), p));
            CHECK(bitsEqual(Frame::height(Frame::compose(p, h)), h));
            CHECK(bitsEqual(Frame::fromForwardRight(Frame::forwardRight(v), Frame::height(v)), v));
            CHECK(bitsEqual(Frame::forwardRight(Frame::fromForwardRight(p, h)), p));
            CHECK(bitsEqual(Frame::height(Frame::fromForwardRight(p, h)), h));
            CHECK(bitsEqual(Frame::flatten(v), Frame::compose(Frame::planar(v), 0.f)));
            CHECK(bitsEqual(Frame::withHeight(v, h), Frame::compose(Frame::planar(v), h)));
            CHECK(bitsEqual(Frame::alongUp(v), Frame::height(v)));
        }
    }

    template <class Frame>
    void checkQuaternionRule()
    {
        std::mt19937 rng(7u);
        std::uniform_real_distribution<float> angle(-3.f, 3.f);
        for (int index = 0; index < 500; ++index)
        {
            const glm::vec3 axis = glm::normalize(uniformVector(rng, 1.f));
            const glm::quat rotation = glm::angleAxis(angle(rng), axis);
            const glm::vec3 v = uniformVector(rng, 1.f);
            const glm::vec3 rotatedThenMapped = mapAxes<Frame>(rotation * v);
            const glm::vec3 mappedThenRotated = mapRotation<Frame>(rotation) * mapAxes<Frame>(v);
            CHECK(maxAbsDifference(rotatedThenMapped, mappedThenRotated) <= 1.0e-6f);
        }
    }

    template <class Frame>
    void checkForwardColumnFollowsTheRotatedForward()
    {
        std::mt19937 rng(11u);
        std::uniform_real_distribution<float> angle(-3.f, 3.f);
        for (int index = 0; index < 500; ++index)
        {
            const float turn = angle(rng);
            const glm::mat4 turned = glm::rotate(glm::mat4(1.f), turn, Frame::kTurnAxis);
            const glm::vec3 expected = glm::angleAxis(turn, Frame::kTurnAxis) * Frame::kForward;
            CHECK(maxAbsDifference(Frame::forwardColumn(turned), expected) <= 1.0e-6f);

            const glm::vec3 axis = glm::normalize(uniformVector(rng, 1.f));
            const glm::mat4 rotated = glm::rotate(glm::mat4(1.f), turn, axis);
            CHECK(maxAbsDifference(Frame::forwardColumn(rotated), glm::angleAxis(turn, axis) * Frame::kForward) <= 1.0e-6f);
        }
    }
} // namespace

TEST_CASE("SimulationFrame.UeCentimetresReturnsEveryInputBitForBit", "[SimulationFrame]")
{
    const std::vector<float> values = sampleFloats();
    for (std::size_t index = 0; index + 3 < values.size(); ++index)
    {
        const float x = values[index];
        const float y = values[index + 1];
        const float z = values[index + 2];
        const float w = values[index + 3];
        const glm::vec3 v(x, y, z);
        CHECK(bitsEqual(UeCm::legacyCm(x), x));
        CHECK(bitsEqual(UeCm::legacyCm(double(x)), double(x)));
        CHECK(bitsEqual(UeCm::legacyCm2(x), x));
        CHECK(bitsEqual(UeCm::legacyPerCm3(x), x));
        CHECK(bitsEqual(UeCm::legacyPlanarCm(x, y), glm::vec2(x, y)));
        CHECK(bitsEqual(UeCm::legacyAxes(x, y, z), v));
        CHECK(bitsEqual(UeCm::legacyAxesUnsigned(x, y, z), v));
        CHECK(bitsEqual(UeCm::legacyAxial(x, y, z), v));
        CHECK(bitsEqual(UeCm::legacyPosition(x, y, z), v));
        CHECK(bitsEqual(UeCm::legacyExtents(x, y, z), v));
        CHECK(bitsEqual(UeCm::legacyTorque(x, y, z), v));
        CHECK(bitsEqual(UeCm::legacyInertia(x, y, z), v));
        CHECK(bitsEqual(UeCm::legacyRotation(w, x, y, z), glm::quat(w, x, y, z)));
        CHECK(bitsEqual(UeCm::height(v), z));
        CHECK(bitsEqual(UeCm::withHeight(v, w), glm::vec3(x, y, w)));
        CHECK(bitsEqual(UeCm::planar(v), glm::vec2(x, y)));
        CHECK(bitsEqual(UeCm::compose(glm::vec2(x, y), z), v));
        CHECK(bitsEqual(UeCm::flatten(v), glm::vec3(x, y, 0.f)));
        CHECK(bitsEqual(UeCm::turnComponent(v), z));
        CHECK(bitsEqual(UeCm::forwardRight(v), glm::vec2(x, y)));
        CHECK(bitsEqual(UeCm::fromForwardRight(glm::vec2(x, y), z), v));
        CHECK(bitsEqual(UeCm::alongUp(v), z));
        CHECK(bitsEqual(UeCm::toLegacyCm(x), double(x)));
        CHECK(bitsEqual(UeCm::toLegacyAxes(v), glm::dvec3(v)));
        CHECK(bitsEqual(UeCm::toLegacyAxial(v), glm::dvec3(v)));
        CHECK(bitsEqual(UeCm::toLegacyPosition(v), glm::dvec3(v)));
        CHECK(bitsEqual(UeCm::toLegacyPlanar(glm::vec2(x, y)).x, double(x)));
        CHECK(bitsEqual(UeCm::toLegacyPlanar(glm::vec2(x, y)).y, double(y)));
        const glm::mat4 m(glm::vec4(x, y, z, w), glm::vec4(w, x, y, z), glm::vec4(z, w, x, y), glm::vec4(y, z, w, x));
        CHECK(bitsEqual(UeCm::forwardColumn(m), v));
    }

    CHECK(std::signbit(UeCm::legacyCm(-0.f)));
    CHECK(std::signbit(UeCm::legacyCm(-0.0)));
    CHECK(std::signbit(UeCm::legacyPosition(-0.f, -0.f, -0.f).x));
    CHECK(std::signbit(UeCm::legacyAxial(1.f, -0.f, 1.f).y));
    CHECK(std::signbit(UeCm::legacyRotation(-0.f, 1.f, 1.f, 1.f).w));
    CHECK(std::signbit(UeCm::height(glm::vec3(1.f, 1.f, -0.f))));
    CHECK(std::signbit(UeCm::planar(glm::vec3(-0.f, 1.f, 1.f)).x));
    CHECK(std::signbit(UeCm::turnComponent(glm::vec3(1.f, 1.f, -0.f))));
    CHECK(std::signbit(UeCm::forwardRight(glm::vec3(1.f, -0.f, 1.f)).y));
    CHECK(std::signbit(UeCm::compose(glm::vec2(-0.f, 1.f), 1.f).x));
    CHECK(std::signbit(UeCm::toLegacyPosition(glm::vec3(1.f, 1.f, -0.f)).z));
    const glm::mat4 signedZeroColumns(glm::vec4(-0.f, 1.f, -0.f, 0.f), glm::vec4(1.f), glm::vec4(1.f), glm::vec4(1.f));
    CHECK(std::signbit(UeCm::forwardColumn(signedZeroColumns).x));
    CHECK(std::signbit(UeCm::forwardColumn(signedZeroColumns).z));
    CHECK(!std::signbit(UeCm::forwardColumn(glm::mat4(1.f)).y));
}

TEST_CASE("SimulationFrame.AxisTablesMatchBothFrames", "[SimulationFrame]")
{
    CHECK(UeCm::kForward == glm::vec3(1.f, 0.f, 0.f));
    CHECK(UeCm::kRight == glm::vec3(0.f, 1.f, 0.f));
    CHECK(UeCm::kUp == glm::vec3(0.f, 0.f, 1.f));
    CHECK(UeCm::kTurnAxis == glm::vec3(0.f, 0.f, 1.f));
    CHECK(UeCm::kVerticalTurnAxis == glm::vec3(0.f, 1.f, 0.f));
    CHECK(UeCm::kChartOrientation == 1.f);
    CHECK(UeCm::kUpIndex == 2);

    CHECK(YCm::kForward == glm::vec3(0.f, 0.f, -1.f));
    CHECK(YCm::kRight == glm::vec3(1.f, 0.f, 0.f));
    CHECK(YCm::kUp == glm::vec3(0.f, 1.f, 0.f));
    CHECK(YCm::kTurnAxis == glm::vec3(0.f, -1.f, 0.f));
    CHECK(YCm::kVerticalTurnAxis == glm::vec3(-1.f, 0.f, 0.f));
    CHECK(YCm::kChartOrientation == -1.f);
    CHECK(YCm::kUpIndex == 1);

    CHECK(UeM::kForward == UeCm::kForward);
    CHECK(UeM::kUpIndex == UeCm::kUpIndex);
    CHECK(YM::kForward == YCm::kForward);
    CHECK(YM::kTurnAxis == YCm::kTurnAxis);
    CHECK(YM::kUpIndex == YCm::kUpIndex);

    checkAxisIdentities<UeCm>();
    checkAxisIdentities<UeM>();
    checkAxisIdentities<YCm>();
    checkAxisIdentities<YM>();
}

TEST_CASE("SimulationFrame.YFramePicksAndLegacyMapsMatchTheTables", "[SimulationFrame]")
{
    const glm::vec3 v(2.f, 3.f, 5.f);
    CHECK(bitsEqual(YCm::height(v), 3.f));
    CHECK(bitsEqual(YCm::planar(v), glm::vec2(-5.f, 2.f)));
    CHECK(bitsEqual(YCm::forwardRight(v), glm::vec2(-5.f, 2.f)));
    CHECK(bitsEqual(YCm::compose(glm::vec2(7.f, 11.f), 13.f), glm::vec3(11.f, 13.f, -7.f)));
    CHECK(bitsEqual(YCm::fromForwardRight(glm::vec2(7.f, 11.f), 13.f), glm::vec3(11.f, 13.f, -7.f)));
    CHECK(bitsEqual(YCm::flatten(v), glm::vec3(2.f, 0.f, 5.f)));
    CHECK(bitsEqual(YCm::withHeight(v, 17.f), glm::vec3(2.f, 17.f, 5.f)));
    CHECK(bitsEqual(YCm::turnComponent(v), -3.f));
    CHECK(bitsEqual(YCm::alongUp(v), 3.f));
    const glm::mat4 columns(glm::vec4(2.f, 3.f, 5.f, 0.f), glm::vec4(7.f, 11.f, 13.f, 0.f), glm::vec4(17.f, -0.f, 19.f, 0.f),
        glm::vec4(23.f, 29.f, 31.f, 1.f));
    CHECK(bitsEqual(YCm::forwardColumn(columns), glm::vec3(-17.f, 0.f, -19.f)));
    CHECK(bitsEqual(YM::forwardColumn(columns), glm::vec3(-17.f, 0.f, -19.f)));
    CHECK(bitsEqual(UeM::forwardColumn(columns), glm::vec3(2.f, 3.f, 5.f)));

    CHECK(bitsEqual(YCm::legacyAxes(2.f, 3.f, 5.f), glm::vec3(3.f, 5.f, -2.f)));
    CHECK(bitsEqual(YCm::legacyPosition(2.f, 3.f, 5.f), glm::vec3(3.f, 5.f, -2.f)));
    CHECK(bitsEqual(YCm::legacyAxesUnsigned(2.f, 3.f, 5.f), glm::vec3(3.f, 5.f, 2.f)));
    CHECK(bitsEqual(YCm::legacyExtents(2.f, 3.f, 5.f), glm::vec3(3.f, 5.f, 2.f)));
    CHECK(bitsEqual(YCm::legacyAxial(2.f, 3.f, 5.f), glm::vec3(-3.f, -5.f, 2.f)));
    CHECK(bitsEqual(YCm::legacyTorque(2.f, 3.f, 5.f), glm::vec3(-3.f, -5.f, 2.f)));
    CHECK(bitsEqual(YCm::legacyInertia(2.f, 3.f, 5.f), glm::vec3(3.f, 5.f, 2.f)));
    CHECK(bitsEqual(YCm::legacyRotation(0.5f, 2.f, 3.f, 5.f), glm::quat(0.5f, -3.f, -5.f, 2.f)));
    CHECK(bitsEqual(YCm::legacyPlanarCm(2.f, 3.f), glm::vec2(2.f, 3.f)));

    CHECK(bitsEqual(YM::legacyPosition(30.f, 42.f, -10.f), glm::vec3(0.42f, -0.1f, -0.3f)));
    CHECK(bitsEqual(YM::legacyExtents(30.f, 42.f, 10.f), glm::vec3(0.42f, 0.1f, 0.3f)));
    CHECK(bitsEqual(YM::legacyTorque(10000.f, 20000.f, 30000.f), glm::vec3(-2.f, -3.f, 1.f)));
    CHECK(bitsEqual(YM::legacyInertia(10000.f, 20000.f, 30000.f), glm::vec3(2.f, 3.f, 1.f)));

    CHECK(bitsEqual(YCm::toLegacyAxes(glm::vec3(3.f, 5.f, -2.f)), glm::dvec3(2.0, 3.0, 5.0)));
    CHECK(bitsEqual(YCm::toLegacyAxial(glm::vec3(-3.f, -5.f, 2.f)), glm::dvec3(2.0, 3.0, 5.0)));
    CHECK(bitsEqual(YM::toLegacyPosition(glm::vec3(0.5f, 0.25f, -2.f)), glm::dvec3(200.0, 50.0, 25.0)));
    const glm::dquat legacy = YCm::toLegacyRotation(glm::quat(0.5f, -3.f, -5.f, 2.f));
    CHECK(legacy == glm::dquat(0.5, 2.0, 3.0, 5.0));
    CHECK(YCm::toLegacyPlanar(glm::vec2(-5.f, 2.f)) == glm::dvec2(-5.0, 2.0));

    const std::vector<float> values = sampleFloats();
    for (std::size_t index = 0; index + 3 < values.size(); ++index)
    {
        const float x = values[index];
        const float y = values[index + 1];
        const float z = values[index + 2];
        const float w = values[index + 3];
        const glm::dvec3 ue(x, y, z);
        CHECK(bitsEqual(YCm::toLegacyAxes(YCm::legacyAxes(x, y, z)), ue));
        CHECK(bitsEqual(YCm::toLegacyAxial(YCm::legacyAxial(x, y, z)), ue));
        CHECK(bitsEqual(YCm::toLegacyPosition(YCm::legacyPosition(x, y, z)), ue));
        const glm::dquat rotation = YCm::toLegacyRotation(YCm::legacyRotation(w, x, y, z));
        CHECK(bitsEqual(rotation.w, double(w)));
        CHECK(bitsEqual(glm::dvec3(rotation.x, rotation.y, rotation.z), ue));
    }
}

TEST_CASE("SimulationFrame.PicksRoundTripExactlyAtEveryCombo", "[SimulationFrame]")
{
    checkPicksRoundTrip<UeCm>();
    checkPicksRoundTrip<UeM>();
    checkPicksRoundTrip<YCm>();
    checkPicksRoundTrip<YM>();
}

TEST_CASE("SimulationFrame.LengthWrappersGiveCorrectlyRoundedMetres", "[SimulationFrame]")
{
    volatile float thirty = 30.f;
    volatile float fortyTwo = 42.f;
    CHECK(bitsEqual(UeM::legacyCm(thirty), 0.3f));
    CHECK(bitsEqual(UeM::legacyCm(fortyTwo), 0.42f));
    CHECK(bitsEqual(YM::legacyCm(thirty), 0.3f));
    CHECK(bitsEqual(YM::legacyCm(fortyTwo), 0.42f));
    CHECK(bitsEqual(UeM::legacyCm(30.f), 0.3f));
    CHECK(bitsEqual(UeM::legacyCm(42.f), 0.42f));
    CHECK(bitsEqual(UeM::legacyCm(54.f), 0.54f));
    CHECK(bitsEqual(UeM::legacyCm(10.f), 0.1f));
    CHECK(bitsEqual(UeM::legacyCm(-980.f), -9.8f));

    volatile double micro = 1.0e-6;
    CHECK(bitsEqual(UeM::legacyCm(micro), 1.0e-6 / 100.0));
    CHECK(bitsEqual(UeCm::legacyCm(micro), 1.0e-6));
    CHECK(bitsEqual(UeM::legacyCm2(10000.f), 1.f));
    CHECK(bitsEqual(UeM::legacyCm2(2500.f), 0.25f));
    CHECK(bitsEqual(UeM::legacyPerCm3(0.5f), 500000.f));
    CHECK(bitsEqual(UeM::legacyPlanarCm(30.f, -42.f), glm::vec2(0.3f, -0.42f)));
    CHECK(bitsEqual(UeM::legacyPosition(30.f, 42.f, -10.f), glm::vec3(0.3f, 0.42f, -0.1f)));

    CHECK(bitsEqual(UeCm::legacyCm(thirty), 30.f));
    CHECK(bitsEqual(YCm::legacyCm(fortyTwo), 42.f));

    CHECK(UeCm::kUnitsPerCentimetre == 1.f);
    CHECK(UeCm::kUeCentimetresPerUnit == 1.f);
    CHECK(UeM::kUnitsPerCentimetre == 0.01f);
    CHECK(UeM::kUeCentimetresPerUnit == 100.f);
    CHECK(YM::kUnitsPerCentimetre == 0.01f);
    CHECK(YM::kUeCentimetresPerUnit == 100.f);

    CHECK(bitsEqual(UeM::toLegacyCm(0.5f), 50.0));
    CHECK(bitsEqual(UeCm::toLegacyCm(0.5f), 0.5));
    CHECK(bitsEqual(UeM::toLegacyCm(0.25), 25.0));
}

TEST_CASE("SimulationFrame.LogHelpersFollowTheUnits", "[SimulationFrame]")
{
    CHECK(std::string(UeCm::kLengthFieldSuffix) == "Cm");
    CHECK(std::string(YCm::kLengthFieldSuffix) == "Cm");
    CHECK(std::string(UeM::kLengthFieldSuffix) == "M");
    CHECK(std::string(YM::kLengthFieldSuffix) == "M");
    for (int decimals = 0; decimals <= 6; ++decimals)
    {
        CHECK(UeCm::lengthDecimals(decimals) == decimals);
        CHECK(YCm::lengthDecimals(decimals) == decimals);
        CHECK(UeM::lengthDecimals(decimals) == decimals + 2);
        CHECK(YM::lengthDecimals(decimals) == decimals + 2);
    }
}

TEST_CASE("SimulationFrame.QuaternionRuleHoldsOverSeededCases", "[SimulationFrame]")
{
    checkQuaternionRule<UeCm>();
    checkQuaternionRule<UeM>();
    checkQuaternionRule<YCm>();
    checkQuaternionRule<YM>();
}

TEST_CASE("SimulationFrame.ForwardColumnFollowsTheRotatedForwardAtEveryCombo", "[SimulationFrame]")
{
    checkForwardColumnFollowsTheRotatedForward<UeCm>();
    checkForwardColumnFollowsTheRotatedForward<UeM>();
    checkForwardColumnFollowsTheRotatedForward<YCm>();
    checkForwardColumnFollowsTheRotatedForward<YM>();
}

TEST_CASE("SimulationFrame.ChiralityFactorKeepsTheTangentFrame", "[SimulationFrame]")
{
    std::mt19937 rng(5u);
    for (int index = 0; index < 2000; ++index)
    {
        const glm::vec3 a = uniformVector(rng, 1.f);
        const glm::vec3 b = uniformVector(rng, 1.f);

        const glm::dvec3 n = glm::normalize(glm::dvec3(a.x * 0.3, a.y * 0.3, 1.0));
        const glm::dvec3 nY = glm::dvec3(mapAxes<YCm>(glm::vec3(n)));
        const glm::dvec3 forward(UeCm::kForward);
        const glm::dvec3 right(UeCm::kRight);
        const glm::dvec3 forwardY(YCm::kForward);
        const glm::dvec3 rightY(YCm::kRight);
        const glm::dvec3 reference = std::abs(glm::dot(n, forward)) < 0.9 ? forward : right;
        const glm::dvec3 referenceY = std::abs(glm::dot(nY, forwardY)) < 0.9 ? forwardY : rightY;
        const glm::dvec3 u = glm::normalize(reference - n * glm::dot(reference, n));
        const glm::dvec3 uY = glm::normalize(referenceY - nY * glm::dot(referenceY, nY));
        const glm::dvec3 v = double(UeCm::kChartOrientation) * glm::cross(n, u);
        const glm::dvec3 vY = double(YCm::kChartOrientation) * glm::cross(nY, uY);

        CHECK(maxAbsDifference(glm::dvec3(mapAxes<YCm>(glm::vec3(u))), uY) <= 1.0e-6);
        CHECK(maxAbsDifference(glm::dvec3(mapAxes<YCm>(glm::vec3(v))), vY) <= 1.0e-6);
        CHECK(maxAbsDifference(glm::dvec3(mapAxes<YCm>(glm::vec3(glm::cross(n, u)))), -glm::cross(nY, uY)) <= 1.0e-6);

        const float turnUe = UeCm::turnComponent(glm::cross(a, b));
        const float turnY = YCm::turnComponent(glm::cross(mapAxes<YCm>(a), mapAxes<YCm>(b)));
        CHECK(std::abs(turnUe - turnY) <= 1.0e-6f);

        CHECK(bitsEqual(YCm::planar(mapAxes<YCm>(a)), UeCm::planar(a)));
    }
}

#endif
