// SPDX-License-Identifier: MPL-2.0
// docs/VerdictMarginTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/CorrectionCache.h"
#include "OGSimulation/NetSyncTelemetry.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/SimulationComparison.h"
#include "OGSimulation/SimulationComparisonGlm.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationSerialization.h"

namespace verdictMarginMock
{
    enum class MockPhase : std::uint8_t { Idle = 0, Moving = 1 };

    struct MotionState
    {
        MockPhase          phase   = MockPhase::Idle;
        std::int32_t       counter = 0;
        float              timer   = 0.f;
        LinearBodyState    bodyState;
        glm::vec2          aim{0.6f, 0.8f};
        glm::quat          facing{1.f, 0.f, 0.f, 0.f};
        std::vector<float> trail = std::vector<float>(3u, 0.f);
    };

    struct PoolState
    {
        float spawnX = 0.f;

        bool isSimilarTo(const PoolState& other) const { return spawnX == other.spawnX; }
    };

    struct MockInput
    {
        std::int32_t value = 0;
    };
}

template <>
struct SerializableFields<verdictMarginMock::MotionState>
{
    static constexpr auto get()
    {
        using S = verdictMarginMock::MotionState;
        return std::make_tuple(SIM_MEMBER(S, phase), SIM_MEMBER(S, counter), SIM_MEMBER(S, timer),
                               SIM_MEMBER(S, bodyState), SIM_MEMBER(S, aim), SIM_MEMBER(S, facing),
                               SIM_VECTOR(S, trail, 4));
    }
};

template <>
struct SerializableFields<verdictMarginMock::PoolState>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(verdictMarginMock::PoolState, spawnX));
    }
};

namespace verdictMarginAdlOnly
{
    struct Hider {};
    void similarityRatioField(Hider);

    template <typename T>
    concept FoundByAdlAlone = requires(const T& v) {
        { similarityRatioField(v, v) } -> std::same_as<double>;
    };
}

static_assert(verdictMarginAdlOnly::FoundByAdlAlone<glm::vec3>
    && verdictMarginAdlOnly::FoundByAdlAlone<glm::vec2>
    && verdictMarginAdlOnly::FoundByAdlAlone<glm::quat>,
    "the glm overloads of similarityRatioField must be found by ADL alone (namespace glm), or the margin walk's "
    "dependent call reaches the scalar arm. og-input-latency task 2h");
static_assert(!verdictMarginAdlOnly::FoundByAdlAlone<float>,
    "FoundByAdlAlone must be false for a type with no associated namespace, or the assert above proves nothing. "
    "og-input-latency task 2h");
static_assert(verdictMarginDetail::DecidedByMemberIsSimilarTo<verdictMarginMock::PoolState>
    && !verdictMarginDetail::DecidedByMemberIsSimilarTo<verdictMarginMock::MotionState>,
    "the margin walk's member-isSimilarTo test must pick out exactly the elements compareElement hands to their own "
    "isSimilarTo. og-input-latency task 2h");

namespace
{
    using verdictMarginMock::MockPhase;
    using verdictMarginMock::MotionState;
    using verdictMarginMock::PoolState;
    using MockComposite = SimulationStateComposite<MotionState, PoolState>;
    using MockCache     = StateCorrectionCache<MockComposite, verdictMarginMock::MockInput>;

    const char* const kVelocityPath = "verdictMarginMock::MotionState.bodyState.linearVelocity";

    class MarginGate
    {
    public:
        explicit MarginGate(std::function<bool()> predicate)
        {
            correctionVerdictMargin::setEnabledPredicate(std::move(predicate));
            correctionVerdictMargin::resetWalkCount();
        }
        ~MarginGate()
        {
            correctionVerdictMargin::clearEnabledPredicate();
            correctionVerdictMargin::resetWalkCount();
        }
        MarginGate(const MarginGate&) = delete;
        MarginGate& operator=(const MarginGate&) = delete;
    };

    MotionState base()
    {
        MotionState s;
        s.phase                    = MockPhase::Moving;
        s.counter                  = 7;
        s.timer                    = 0.25f;
        s.bodyState.position       = glm::vec3(13.0513f, 5.2657f, -16.7586f);
        s.bodyState.linearVelocity = glm::vec3(0.f, 0.f, 0.f);
        s.facing                   = glm::angleAxis(0.7f, glm::vec3(0.f, 1.f, 0.f));
        return s;
    }

    MockComposite composite(const MotionState& motion, const PoolState& pool = PoolState{})
    {
        MockComposite c;
        c.edit<MotionState>() = motion;
        c.edit<PoolState>()   = pool;
        return c;
    }

    MotionState velocityOffBy(float offset)
    {
        MotionState s = base();
        s.bodyState.linearVelocity.x += offset;
        return s;
    }

    CorrectionInsertVerdict land(const MockComposite& predicted, const MockComposite& authority)
    {
        MockCache cache(nullptr);
        cache.pushPredictionTick(500u);
        cache.pushPredictionState(MockComposite(predicted));
        CorrectionInsertVerdict report;
        cache.tryInsertingCorrectState(MockComposite(authority), 500u, kNoInputCaptureTick, &report);
        REQUIRE(report.landed);
        return report;
    }

    CorrectionInsertVerdict land(const MotionState& predicted, const MotionState& authority)
    {
        return land(composite(predicted), composite(authority));
    }

    std::size_t bucketOf(const CorrectionInsertVerdict& v)
    {
        REQUIRE(v.verdictMargin.evaluated);
        return verdictMarginBucketOf(v.verdictMargin.maxRatio);
    }

    CorrectionArrivalDecision decisionFrom(const CorrectionInsertVerdict& v, PredictedCharacterClass characterClass)
    {
        CorrectionArrivalDecision d;
        d.characterClass       = characterClass;
        d.landingSite          = CorrectionLandingSite::AtFrontier;
        d.landed               = v.landed;
        d.tick                 = v.tick;
        d.predictionWasCorrect = v.predictionWasCorrect;
        d.fieldDivergence      = v.fieldDivergence;
        d.verdictMargin        = v.verdictMargin;
        return d;
    }

    std::vector<std::string> linesWith(const std::vector<std::string>& lines, const char* tag)
    {
        std::vector<std::string> found;
        for (const std::string& line : lines)
            if (line.find(tag) != std::string::npos)
                found.push_back(line);
        return found;
    }

    std::uint32_t valueOf(const std::string& line, const char* key)
    {
        const std::string needle = std::string(" ") + key + "=";
        const std::size_t at = line.find(needle);
        REQUIRE(at != std::string::npos);
        return static_cast<std::uint32_t>(std::stoul(line.substr(at + needle.size())));
    }

    std::string formatG(double v)
    {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%g", v);
        return buf;
    }
}

TEST_CASE("Verdict margin: injected offsets fill the right buckets, through the real cache",
          "[Network][VerdictMargin]")
{
    MarginGate gate([] { return true; });

    SECTION("no offset: r = 0, b0, agreeing, no field named")
    {
        const CorrectionInsertVerdict v = land(base(), base());
        CHECK(v.predictionWasCorrect);
        CHECK(bucketOf(v) == 0u);
        CHECK(v.verdictMargin.maxRatio == 0.0);
        CHECK_FALSE(v.verdictMargin.discrete);
        CHECK(std::string(v.verdictMargin.maxField).empty());
    }
    SECTION("5e-10: b1, agreeing")
    {
        const CorrectionInsertVerdict v = land(base(), velocityOffBy(5e-10f));
        CHECK(v.predictionWasCorrect);
        CHECK(bucketOf(v) == 1u);
        CHECK(std::string(v.verdictMargin.maxField) == kVelocityPath);
    }
    SECTION("5e-8: b2, agreeing")
    {
        const CorrectionInsertVerdict v = land(base(), velocityOffBy(5e-8f));
        CHECK(v.predictionWasCorrect);
        CHECK(bucketOf(v) == 2u);
    }
    SECTION("5e-6: b3, agreeing")
    {
        const CorrectionInsertVerdict v = land(base(), velocityOffBy(5e-6f));
        CHECK(v.predictionWasCorrect);
        CHECK(bucketOf(v) == 3u);
    }
    SECTION("5e-5, ratio 0.5: b4, agreeing")
    {
        const CorrectionInsertVerdict v = land(base(), velocityOffBy(5e-5f));
        CHECK(v.predictionWasCorrect);
        CHECK(bucketOf(v) == 4u);
        CHECK(v.verdictMargin.maxRatio == Catch::Approx(0.5).epsilon(1e-6));
        CHECK(std::string(v.verdictMargin.maxField) == kVelocityPath);
    }
    SECTION("2e-4: b5, disagreeing")
    {
        const CorrectionInsertVerdict v = land(base(), velocityOffBy(2e-4f));
        CHECK_FALSE(v.predictionWasCorrect);
        CHECK(bucketOf(v) == 5u);
        CHECK(v.verdictMargin.maxRatio == Catch::Approx(2.0).epsilon(1e-6));
        CHECK_FALSE(v.verdictMargin.discrete);
    }
    SECTION("the largest field wins, and names itself")
    {
        MotionState authority = velocityOffBy(5e-8f);
        authority.timer += 5e-5f;
        const CorrectionInsertVerdict v = land(base(), authority);
        CHECK(v.predictionWasCorrect);
        CHECK(bucketOf(v) == 4u);
        CHECK(std::string(v.verdictMargin.maxField) == "verdictMarginMock::MotionState.timer");
    }
}

TEST_CASE("Verdict margin: a discrete mismatch counts as discrete, outside the ratio",
          "[Network][VerdictMargin]")
{
    MarginGate gate([] { return true; });

    SECTION("an enum")
    {
        MotionState authority = base();
        authority.phase = MockPhase::Idle;
        const CorrectionInsertVerdict v = land(base(), authority);
        CHECK_FALSE(v.predictionWasCorrect);
        CHECK(v.verdictMargin.discrete);
        CHECK(v.verdictMargin.maxRatio == 0.0);
    }
    SECTION("an integer, with a float offset beside it that sets the ratio")
    {
        MotionState authority = velocityOffBy(5e-5f);
        authority.counter = 8;
        const CorrectionInsertVerdict v = land(base(), authority);
        CHECK_FALSE(v.predictionWasCorrect);
        CHECK(v.verdictMargin.discrete);
        CHECK(bucketOf(v) == 4u);
    }
    SECTION("a vector's size")
    {
        MotionState authority = base();
        authority.trail.push_back(0.f);
        const CorrectionInsertVerdict v = land(base(), authority);
        CHECK_FALSE(v.predictionWasCorrect);
        CHECK(v.verdictMargin.discrete);
    }
    SECTION("a vector's float element: the fold compares a std::vector field exactly today, so it is discrete")
    {
        MotionState authority = base();
        authority.trail[1] += 5e-8f;
        const CorrectionInsertVerdict v = land(base(), authority);
        CHECK_FALSE(v.predictionWasCorrect);
        CHECK(v.verdictMargin.discrete);
        CHECK(v.verdictMargin.maxRatio == 0.0);
    }
    SECTION("an element decided by its own member isSimilarTo")
    {
        PoolState pool;
        pool.spawnX = 5e-5f;
        const CorrectionInsertVerdict v = land(composite(base()), composite(base(), pool));
        CHECK_FALSE(v.predictionWasCorrect);
        CHECK(v.verdictMargin.discrete);
        CHECK(v.verdictMargin.maxRatio == 0.0);
    }
}

TEST_CASE("Verdict margin: glm vec2 and quat leaves, the quat after sign alignment",
          "[Network][VerdictMargin]")
{
    MarginGate gate([] { return true; });

    SECTION("q against -q: r = 0")
    {
        MotionState authority = base();
        authority.facing = -authority.facing;
        const CorrectionInsertVerdict v = land(base(), authority);
        CHECK(v.predictionWasCorrect);
        CHECK(v.verdictMargin.maxRatio == 0.0);
    }
    SECTION("rotated by 1e-4 rad: agreeing, below 1")
    {
        MotionState authority = base();
        authority.facing = authority.facing * glm::angleAxis(1e-4f, glm::vec3(0.f, 1.f, 0.f));
        const CorrectionInsertVerdict v = land(base(), authority);
        CHECK(v.predictionWasCorrect);
        CHECK(v.verdictMargin.maxRatio > 0.0);
        CHECK(v.verdictMargin.maxRatio < 1.0);
        CHECK(std::string(v.verdictMargin.maxField) == "verdictMarginMock::MotionState.facing");
    }
    SECTION("rotated by 5e-4 rad: disagreeing, b5")
    {
        MotionState authority = base();
        authority.facing = authority.facing * glm::angleAxis(5e-4f, glm::vec3(0.f, 1.f, 0.f));
        const CorrectionInsertVerdict v = land(base(), authority);
        CHECK_FALSE(v.predictionWasCorrect);
        CHECK(bucketOf(v) == 5u);
    }
    SECTION("aim off by 5e-5: b4")
    {
        MotionState authority = base();
        authority.aim.y += 5e-5f;
        const CorrectionInsertVerdict v = land(base(), authority);
        CHECK(v.predictionWasCorrect);
        CHECK(bucketOf(v) == 4u);
        CHECK(std::string(v.verdictMargin.maxField) == "verdictMarginMock::MotionState.aim");
    }
}

TEST_CASE("Verdict margin: the walk agrees with the fold on every perturbation",
          "[Network][VerdictMargin]")
{
    MarginGate gate([] { return true; });

    const std::array<float, 9> offsets = { 0.f, 1e-12f, 5e-10f, 5e-8f, 5e-6f, 5e-5f, 9.9e-5f, 1.01e-4f, 2e-4f };
    int disagreements = 0;
    for (const float offset : offsets)
    {
        for (int field = 0; field < 6; ++field)
        {
            MotionState authority = base();
            switch (field)
            {
            case 0: authority.timer += offset; break;
            case 1: authority.bodyState.linearVelocity.y += offset; break;
            case 2: authority.bodyState.position.z += offset; break;
            case 3: authority.aim.x += offset; break;
            case 4: authority.facing.w += offset; break;
            default: authority.trail[2] += offset; break;
            }
            const CorrectionInsertVerdict v = land(base(), authority);
            INFO("offset " << offset << " field " << field << " ratio " << v.verdictMargin.maxRatio);
            CHECK(v.predictionWasCorrect == (v.verdictMargin.maxRatio < 1.0 && !v.verdictMargin.discrete));
            if (!v.predictionWasCorrect)
                ++disagreements;
        }
    }
    CHECK(disagreements > 0);
    CHECK(disagreements < static_cast<int>(offsets.size() * 6u));
}

TEST_CASE("Verdict margin: the gate. Unset or false walks nothing; true walks every landed verdict",
          "[Network][VerdictMargin]")
{
    const auto drive = [](bool withObserver) {
        MockCache cache(nullptr);
        std::uint32_t landed = 0u;
        for (std::uint32_t tick = 100u; tick < 110u; ++tick)
        {
            cache.pushPredictionTick(tick);
            cache.pushPredictionState(composite(base()));
        }
        for (std::uint32_t tick = 100u; tick < 112u; ++tick)
        {
            const float offset = (tick % 2u == 0u) ? 0.f : 2e-4f;
            CorrectionInsertVerdict report;
            cache.tryInsertingCorrectState(composite(velocityOffBy(offset)), tick, kNoInputCaptureTick,
                                           withObserver ? &report : nullptr);
            if (withObserver && report.landed)
            {
                ++landed;
                CHECK(report.predictionWasCorrect == (offset == 0.f));
            }
        }
        return landed;
    };

    SECTION("unset: no walk")
    {
        correctionVerdictMargin::clearEnabledPredicate();
        correctionVerdictMargin::resetWalkCount();
        CHECK(drive(true) == 10u);
        CHECK(correctionVerdictMargin::walkCount() == 0u);
    }
    SECTION("false: consulted once per landed verdict, no walk")
    {
        std::uint32_t asked = 0u;
        MarginGate gate([&asked] { ++asked; return false; });
        CHECK(drive(true) == 10u);
        CHECK(asked == 10u);
        CHECK(correctionVerdictMargin::walkCount() == 0u);
    }
    SECTION("true: one walk per landed verdict, agreeing and disagreeing; none for a discard")
    {
        MarginGate gate([] { return true; });
        CHECK(drive(true) == 10u);
        CHECK(correctionVerdictMargin::walkCount() == 10u);
    }
    SECTION("true, but no observer: no walk")
    {
        MarginGate gate([] { return true; });
        CHECK(drive(false) == 0u);
        CHECK(correctionVerdictMargin::walkCount() == 0u);
    }
    SECTION("the field-diff gate stays its own: the margin gate does not open it")
    {
        MarginGate gate([] { return true; });
        correctionFieldDiff::clearEnabledPredicate();
        correctionFieldDiff::resetWalkCount();
        CHECK(drive(true) == 10u);
        CHECK(correctionFieldDiff::walkCount() == 0u);
    }
}

TEST_CASE("Verdict margin: verdicts, slots and the resim anchor are identical with the gate on and off",
          "[Network][VerdictMargin]")
{
    struct Outcome
    {
        std::vector<bool>          correct;
        std::vector<std::uint32_t> anchors;
        std::vector<MotionState>   slots;
        std::uint32_t              landingSeqNr = 0u;
    };

    const std::array<float, 10> offsets = { 0.f, 5e-10f, 2e-4f, 5e-5f, 0.f, 3e-4f, 5e-6f, 0.f, 1e-3f, 9e-5f };
    const auto run = [&offsets](bool gateOn) {
        MarginGate gate([gateOn] { return gateOn; });
        MockCache cache(nullptr);
        for (std::uint32_t tick = 200u; tick < 210u; ++tick)
        {
            cache.pushPredictionTick(tick);
            cache.pushPredictionState(composite(base()));
        }
        Outcome out;
        for (std::uint32_t i = 0u; i < offsets.size(); ++i)
        {
            CorrectionInsertVerdict report;
            cache.tryInsertingCorrectState(composite(velocityOffBy(offsets[i])), 200u + i, kNoInputCaptureTick, &report);
            REQUIRE(report.landed);
            CHECK(report.verdictMargin.evaluated == gateOn);
            out.correct.push_back(report.predictionWasCorrect);
            out.anchors.push_back(cache.getPendingResimAnchorTick());
        }
        for (std::uint32_t tick = 200u; tick < 210u; ++tick)
            out.slots.push_back(cache.getState(cache.getCacheIndex(tick)).get<MotionState>());
        out.landingSeqNr = cache.getDiagnostics().landingSeqNr();
        CHECK(correctionVerdictMargin::walkCount() == (gateOn ? offsets.size() : 0u));
        return out;
    };

    const Outcome off = run(false);
    const Outcome on  = run(true);
    CHECK(off.correct == on.correct);
    CHECK(off.anchors == on.anchors);
    CHECK(off.landingSeqNr == on.landingSeqNr);
    REQUIRE(off.slots.size() == on.slots.size());
    for (std::size_t i = 0; i < off.slots.size(); ++i)
    {
        INFO("slot " << i);
        CHECK(std::memcmp(&off.slots[i].bodyState, &on.slots[i].bodyState, sizeof(LinearBodyState)) == 0);
        CHECK(off.slots[i].timer == on.slots[i].timer);
    }
    CHECK(std::count(off.correct.begin(), off.correct.end(), false) == 3);
}

TEST_CASE("Verdict margin: one [DivergenceProbe.Margin] line per closed window and class, verdicts = corrections",
          "[Network][VerdictMargin]")
{
    std::vector<std::string> lines;
    NetSyncTelemetry telemetry;
    telemetry.setLogger([&lines](const char* msg) { lines.emplace_back(msg); });

    SECTION("gate on: both classes, the buckets, the max and its field")
    {
        MarginGate gate([] { return true; });
        const std::array<float, 6> offsets = { 0.f, 5e-10f, 5e-8f, 5e-6f, 5e-5f, 2e-4f };
        std::vector<CorrectionInsertVerdict> local;
        for (const float offset : offsets)
            local.push_back(land(base(), velocityOffBy(offset)));
        MotionState discreteOnly = base();
        discreteOnly.counter = 9;
        const CorrectionInsertVerdict discreteVerdict = land(base(), discreteOnly);

        std::uint32_t sent = 0u;
        for (const CorrectionInsertVerdict& v : local)
        {
            telemetry.emitCorrectionArrival(1u, decisionFrom(v, PredictedCharacterClass::LocallyPredicted));
            ++sent;
        }
        for (int i = 0; i < 10; ++i, ++sent)
            telemetry.emitCorrectionArrival(2u, decisionFrom(i == 0 ? discreteVerdict : local[1],
                                                             PredictedCharacterClass::RemoteProxy));
        while (sent < kCorrectionVerdictProbeWindowSamples)
        {
            telemetry.emitCorrectionArrival(1u, decisionFrom(local[0], PredictedCharacterClass::LocallyPredicted));
            ++sent;
        }

        const std::vector<std::string> margins = linesWith(lines, "[DivergenceProbe.Margin]");
        const std::vector<std::string> windows = linesWith(lines, "[DivergenceProbe.Window]");
        REQUIRE(margins.size() == 2u);
        REQUIRE(windows.size() == 2u);
        CHECK(margins[0] ==
            "[DivergenceProbe.Margin] class=LocallyPredicted verdicts=110 exactFail=5 b0=105 b1=1 b2=1 b3=1 b4=1 "
            "b5=1 discrete=0 maxRatio=" + formatG(local[5].verdictMargin.maxRatio) + " maxField=" + kVelocityPath
            + " maxAgreeRatio=" + formatG(local[4].verdictMargin.maxRatio) + " maxAgreeField=" + kVelocityPath);
        CHECK(margins[1] ==
            "[DivergenceProbe.Margin] class=RemoteProxy verdicts=10 exactFail=9 b0=1 b1=9 b2=0 b3=0 b4=0 b5=0 "
            "discrete=1 maxRatio=" + formatG(local[1].verdictMargin.maxRatio) + " maxField=" + kVelocityPath
            + " maxAgreeRatio=" + formatG(local[1].verdictMargin.maxRatio) + " maxAgreeField=" + kVelocityPath);
        CHECK(valueOf(margins[0], "verdicts") == valueOf(windows[0], "corrections"));
        CHECK(valueOf(margins[1], "verdicts") == valueOf(windows[1], "corrections"));
    }

    SECTION("the next window starts empty")
    {
        MarginGate gate([] { return true; });
        const CorrectionInsertVerdict big = land(base(), velocityOffBy(2e-4f));
        const CorrectionInsertVerdict zero = land(base(), base());
        for (std::uint32_t i = 0u; i < kCorrectionVerdictProbeWindowSamples; ++i)
            telemetry.emitCorrectionArrival(1u, decisionFrom(i == 0u ? big : zero, PredictedCharacterClass::LocallyPredicted));
        for (std::uint32_t i = 0u; i < kCorrectionVerdictProbeWindowSamples; ++i)
            telemetry.emitCorrectionArrival(1u, decisionFrom(zero, PredictedCharacterClass::LocallyPredicted));

        const std::vector<std::string> margins = linesWith(lines, "[DivergenceProbe.Margin]");
        REQUIRE(margins.size() == 2u);
        CHECK(valueOf(margins[0], "b5") == 1u);
        CHECK(margins[1] ==
            "[DivergenceProbe.Margin] class=LocallyPredicted verdicts=120 exactFail=0 b0=120 b1=0 b2=0 b3=0 b4=0 "
            "b5=0 discrete=0 maxRatio=0 maxField=- maxAgreeRatio=0 maxAgreeField=-");
    }

    SECTION("gate off: the Window lines, no Margin line")
    {
        correctionVerdictMargin::clearEnabledPredicate();
        const CorrectionInsertVerdict v = land(base(), velocityOffBy(2e-4f));
        REQUIRE_FALSE(v.verdictMargin.evaluated);
        for (std::uint32_t i = 0u; i < kCorrectionVerdictProbeWindowSamples; ++i)
            telemetry.emitCorrectionArrival(1u, decisionFrom(v, PredictedCharacterClass::LocallyPredicted));
        CHECK(linesWith(lines, "[DivergenceProbe.Window]").size() == 1u);
        CHECK(linesWith(lines, "[DivergenceProbe.Margin]").empty());
    }
}

TEST_CASE("Verdict margin: a maximal line fits its buffer, path last and whole",
          "[Network][VerdictMargin]")
{
    std::vector<std::string> lines;
    NetSyncTelemetry telemetry;
    telemetry.setLogger([&lines](const char* msg) { lines.emplace_back(msg); });

    CorrectionArrivalDecision d;
    d.characterClass       = PredictedCharacterClass::LocallyPredicted;
    d.landingSite          = CorrectionLandingSite::AtFrontier;
    d.landed               = true;
    d.predictionWasCorrect = false;
    d.verdictMargin.evaluated = true;
    d.verdictMargin.discrete  = true;
    d.verdictMargin.maxRatio  = 1.23456789e+300;
    std::string path(kFieldPathCapacity - 1u, 'p');
    std::memcpy(d.verdictMargin.maxField, path.c_str(), path.size() + 1u);

    CorrectionArrivalDecision agreeing = d;
    agreeing.predictionWasCorrect   = true;
    agreeing.verdictMargin.discrete = false;
    agreeing.verdictMargin.maxRatio = 1.23456789e-300;
    std::string agreePath(kFieldPathCapacity - 1u, 'q');
    std::memcpy(agreeing.verdictMargin.maxField, agreePath.c_str(), agreePath.size() + 1u);

    for (std::uint32_t i = 0u; i < kCorrectionVerdictProbeWindowSamples; ++i)
        telemetry.emitCorrectionArrival(4294967295u, i % 2u == 0u ? d : agreeing);

    const std::vector<std::string> margins = linesWith(lines, "[DivergenceProbe.Margin]");
    REQUIRE(margins.size() == 1u);
    CHECK(margins[0].size() < kVerdictMarginLineCapacity);
    CHECK(margins[0].find(" maxField=" + path + " maxAgreeRatio=1.23457e-300 maxAgreeField=") != std::string::npos);
    CHECK(margins[0].size() >= agreePath.size());
    CHECK(margins[0].compare(margins[0].size() - agreePath.size(), agreePath.size(), agreePath) == 0);

    char widest[kVerdictMarginLineCapacity];
    const int n = std::snprintf(widest, sizeof(widest),
        "[DivergenceProbe.Margin] class=LocallyPredicted verdicts=%u exactFail=%u "
        "b0=%u b1=%u b2=%u b3=%u b4=%u b5=%u discrete=%u maxRatio=%g maxField=%s "
        "maxAgreeRatio=%g maxAgreeField=%s",
        4294967295u, 4294967295u, 4294967295u, 4294967295u, 4294967295u, 4294967295u, 4294967295u, 4294967295u,
        4294967295u, 1.23456789e+300, path.c_str(), 1.23456789e-300, agreePath.c_str());
    CHECK(n == 456);
    CHECK(n > 256);
    CHECK(n < static_cast<int>(kVerdictMarginLineCapacity));
}

TEST_CASE("Verdict margin: maxAgreeField names the largest AGREEING verdict's field, under a larger disagreeing one",
          "[Network][VerdictMargin]")
{
    std::vector<std::string> lines;
    NetSyncTelemetry telemetry;
    telemetry.setLogger([&lines](const char* msg) { lines.emplace_back(msg); });
    MarginGate gate([] { return true; });

    MotionState timerOff = base();
    timerOff.timer += 5e-5f;
    const CorrectionInsertVerdict agreeTimer = land(base(), timerOff);
    const CorrectionInsertVerdict agreeSmall = land(base(), velocityOffBy(5e-6f));
    const CorrectionInsertVerdict disagreeBig = land(base(), velocityOffBy(2e-4f));
    MotionState discreteNear = velocityOffBy(9e-5f);
    discreteNear.counter = 9;
    const CorrectionInsertVerdict disagreeDiscreteNear = land(base(), discreteNear);
    const CorrectionInsertVerdict zero = land(base(), base());

    REQUIRE(agreeTimer.predictionWasCorrect);
    REQUIRE(agreeSmall.predictionWasCorrect);
    REQUIRE_FALSE(disagreeBig.predictionWasCorrect);
    REQUIRE_FALSE(disagreeDiscreteNear.predictionWasCorrect);
    REQUIRE(disagreeDiscreteNear.verdictMargin.discrete);
    REQUIRE(agreeSmall.verdictMargin.maxRatio < agreeTimer.verdictMargin.maxRatio);
    REQUIRE(agreeTimer.verdictMargin.maxRatio < disagreeDiscreteNear.verdictMargin.maxRatio);
    REQUIRE(disagreeDiscreteNear.verdictMargin.maxRatio < 1.0);
    REQUIRE(disagreeBig.verdictMargin.maxRatio >= 1.0);

    SECTION("the agreeing max is the timer's, though two disagreeing verdicts have larger ratios")
    {
        const std::array<const CorrectionInsertVerdict*, 4> sequence = {
            &agreeTimer, &disagreeBig, &disagreeDiscreteNear, &agreeSmall };
        std::uint32_t sent = 0u;
        for (const CorrectionInsertVerdict* v : sequence)
        {
            telemetry.emitCorrectionArrival(1u, decisionFrom(*v, PredictedCharacterClass::LocallyPredicted));
            ++sent;
        }
        for (; sent < kCorrectionVerdictProbeWindowSamples; ++sent)
            telemetry.emitCorrectionArrival(1u, decisionFrom(zero, PredictedCharacterClass::LocallyPredicted));

        const std::vector<std::string> margins = linesWith(lines, "[DivergenceProbe.Margin]");
        REQUIRE(margins.size() == 1u);
        CHECK(margins[0] ==
            "[DivergenceProbe.Margin] class=LocallyPredicted verdicts=120 exactFail=4 b0=116 b1=0 b2=0 b3=1 b4=2 "
            "b5=1 discrete=1 maxRatio=" + formatG(disagreeBig.verdictMargin.maxRatio) + " maxField=" + kVelocityPath
            + " maxAgreeRatio=" + formatG(agreeTimer.verdictMargin.maxRatio)
            + " maxAgreeField=verdictMarginMock::MotionState.timer");
    }

    SECTION("no agreeing verdict above 0: maxAgreeRatio=0 maxAgreeField=-, while maxRatio is not")
    {
        telemetry.emitCorrectionArrival(1u, decisionFrom(disagreeDiscreteNear, PredictedCharacterClass::RemoteProxy));
        for (std::uint32_t sent = 1u; sent < kCorrectionVerdictProbeWindowSamples; ++sent)
            telemetry.emitCorrectionArrival(1u, decisionFrom(zero, PredictedCharacterClass::RemoteProxy));

        const std::vector<std::string> margins = linesWith(lines, "[DivergenceProbe.Margin]");
        REQUIRE(margins.size() == 1u);
        CHECK(margins[0] ==
            "[DivergenceProbe.Margin] class=RemoteProxy verdicts=120 exactFail=1 b0=119 b1=0 b2=0 b3=0 b4=1 "
            "b5=0 discrete=1 maxRatio=" + formatG(disagreeDiscreteNear.verdictMargin.maxRatio) + " maxField="
            + kVelocityPath + " maxAgreeRatio=0 maxAgreeField=-");
    }
}

TEST_CASE("Verdict margin: the buckets' edges and a non-finite leaf", "[Network][VerdictMargin]")
{
    CHECK(verdictMarginBucketOf(0.0) == 0u);
    CHECK(verdictMarginBucketOf(std::numeric_limits<double>::denorm_min()) == 1u);
    CHECK(verdictMarginBucketOf(1e-5) == 1u);
    CHECK(verdictMarginBucketOf(1.0000001e-5) == 2u);
    CHECK(verdictMarginBucketOf(1e-3) == 2u);
    CHECK(verdictMarginBucketOf(1e-1) == 3u);
    CHECK(verdictMarginBucketOf(0.99999) == 4u);
    CHECK(verdictMarginBucketOf(1.0) == 5u);
    CHECK(verdictMarginBucketOf(std::numeric_limits<double>::infinity()) == 5u);

    MarginGate gate([] { return true; });
    MotionState authority = base();
    authority.timer = std::numeric_limits<float>::infinity();
    const CorrectionInsertVerdict v = land(base(), authority);
    CHECK_FALSE(v.predictionWasCorrect);
    CHECK(bucketOf(v) == 5u);
    CHECK(std::string(v.verdictMargin.maxField) == "verdictMarginMock::MotionState.timer");
}

#endif // WITH_LOW_LEVEL_TESTS
