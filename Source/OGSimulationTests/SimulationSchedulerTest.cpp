// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationScheduler.h"

#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

// ---------------------------------------------------------------------------
// SimulationScheduler: the host-pumped fixed step (og-simulationscheduler-withjolt
// task 5). Every case pumps a 60 Hz scheduler with synthetic wall times. Pumps
// land half a step after a deadline wherever a count is pinned, so no assertion
// depends on how a time exactly ON a deadline rounds.
// ---------------------------------------------------------------------------

namespace
{
    constexpr double kDt = 1.0 / 60.0;
    constexpr double kStart = 1000.0;

    template <typename C>
    concept BracedFromDtOnly = requires { C{kDt}; };

    template <typename C>
    concept BracedFromDtAndCap = requires { C{kDt, 8u}; };

    SimulationScheduler makeScheduler(uint32_t cap, double start = kStart)
    {
        return SimulationScheduler(SchedulerConfig{kDt, cap}, start);
    }

    // Pumps once half a step after each of the next `count` deadlines and
    // returns the time of the last pump.
    double pumpSteady(SimulationScheduler& scheduler, int count)
    {
        double now = scheduler.nextDeadline();
        for (int i = 0; i < count; ++i)
        {
            now = scheduler.nextDeadline() + kDt * 0.5;
            REQUIRE(scheduler.pump(now) == 1u);
        }
        return now;
    }
}

// The cap has no default: omitting it must not compile.
static_assert(!BracedFromDtOnly<SchedulerConfig>);
static_assert(BracedFromDtAndCap<SchedulerConfig>);
static_assert(!std::is_default_constructible_v<SchedulerConfig>);

TEST_CASE("SimulationScheduler.ConfigCarriesThePerRoleCap", "[Scheduler]")
{
    const SchedulerConfig server{.dtSeconds = kDt, .maxCatchUpSteps = 60u};
    const SchedulerConfig client{kDt, 8u};
    CHECK(server.maxCatchUpSteps.value == 60u);
    CHECK(client.maxCatchUpSteps.value == 8u);
    CHECK(client.minRateScale == 0.9);
    CHECK(client.maxRateScale == 1.1);

    const SimulationScheduler scheduler(client, kStart);
    CHECK(scheduler.config().maxCatchUpSteps.value == 8u);
    CHECK(scheduler.physicsStepCount() == 0u);
    CHECK(scheduler.rateScale() == 1.0);
    CHECK(scheduler.nextDeadline() == kStart);
}

TEST_CASE("SimulationScheduler.SteadyPumpingYieldsOneStepPerDt", "[Scheduler]")
{
    SECTION("the first step is due at the start time")
    {
        SimulationScheduler scheduler = makeScheduler(8u);
        CHECK(scheduler.pump(kStart) == 1u);
        CHECK(scheduler.pump(kStart + kDt * 0.5) == 0u);
        CHECK(scheduler.physicsStepCount() == 1u);
    }

    SECTION("one pump per dt, 600 pumps")
    {
        SimulationScheduler scheduler = makeScheduler(8u);
        for (int k = 0; k < 600; ++k)
        {
            INFO("pump " << k);
            REQUIRE(scheduler.pump(kStart + k * kDt + kDt * 0.5) == 1u);
        }
        CHECK(scheduler.physicsStepCount() == 600u);
        CHECK(scheduler.lostTime().steps == 0u);
        CHECK(scheduler.lostTime().seconds == 0.0);
    }

    SECTION("pumping at 4x the step rate still yields one step per dt")
    {
        SimulationScheduler scheduler = makeScheduler(8u);
        uint64_t total = 0;
        for (int k = 0; k < 2400; ++k)
        {
            const uint32_t steps = scheduler.pump(kStart + k * (kDt / 4.0) + kDt / 8.0);
            REQUIRE(steps <= 1u);
            total += steps;
        }
        CHECK(total == 600u);
        CHECK(scheduler.lostTime().steps == 0u);
    }
}

TEST_CASE("SimulationScheduler.HitchRunsTheCapAndCountsTheRest", "[Scheduler]")
{
    SimulationScheduler scheduler = makeScheduler(4u);
    const double last = pumpSteady(scheduler, 10);
    REQUIRE(scheduler.lostTime().steps == 0u);

    const double hitch = last + 0.250;
    CHECK(scheduler.pump(hitch) == 4u);
    CHECK(scheduler.physicsStepCount() == 14u);
    CHECK(scheduler.lostTime().steps == 11u);
    CHECK(std::abs(scheduler.lostTime().seconds - 11.0 * kDt) <= 1e-9);

    CHECK(scheduler.stepDeadline(scheduler.physicsStepCount() - 1u) <= hitch);
    CHECK(scheduler.nextDeadline() > hitch);
    CHECK(scheduler.nextDeadline() - hitch <= kDt);

    CHECK(scheduler.pump(hitch + kDt) == 1u);
    CHECK(scheduler.lostTime().steps == 11u);
}

TEST_CASE("SimulationScheduler.NoDriftOverAMillionSteps", "[Scheduler]")
{
    constexpr uint64_t kSteps = 1000000u;
    SimulationScheduler scheduler = makeScheduler(8u);
    for (uint64_t k = 0; k < kSteps; ++k)
    {
        if (scheduler.pump(kStart + static_cast<double>(k) * kDt + kDt * 0.5) != 1u)
        {
            FAIL("pump " << k << " did not yield exactly one step");
        }
    }
    REQUIRE(scheduler.physicsStepCount() == kSteps);

    const double expected = kStart + static_cast<double>(kSteps) * kDt;
    INFO("nextDeadline " << scheduler.nextDeadline() << " expected " << expected);
    CHECK(std::abs(scheduler.nextDeadline() - expected) <= 1e-6);
    CHECK(scheduler.lostTime().steps == 0u);

    // Control: a float accumulator misses the same bound by orders of magnitude,
    // so the 1e-6 tolerance above can tell the two apart.
    float floatDeadline = static_cast<float>(kStart);
    for (uint64_t k = 0; k < kSteps; ++k)
    {
        floatDeadline += static_cast<float>(kDt);
    }
    CHECK(std::abs(static_cast<double>(floatDeadline) - expected) > 1e-3);
}

TEST_CASE("SimulationScheduler.RateScaleScalesTheStepRate", "[Scheduler]")
{
    auto stepsOverTenSeconds = [](double scale)
    {
        SimulationScheduler scheduler = makeScheduler(8u);
        scheduler.setRateScale(scale);
        uint64_t total = 0;
        for (int k = 0; k < 10000; ++k)
        {
            total += scheduler.pump(kStart + k * 0.001 + 0.0005);
        }
        return total;
    };

    const uint64_t nominal = stepsOverTenSeconds(1.0);
    const uint64_t slow = stepsOverTenSeconds(0.95);
    const uint64_t fast = stepsOverTenSeconds(1.05);
    INFO("nominal " << nominal << " slow " << slow << " fast " << fast);

    CHECK(nominal == 600u);
    CHECK(slow == 570u);
    CHECK(fast == 630u);

    SECTION("the scale is clamped to the configured bounds")
    {
        SimulationScheduler scheduler = makeScheduler(8u);
        scheduler.setRateScale(2.0);
        CHECK(scheduler.rateScale() == 1.1);
        scheduler.setRateScale(0.1);
        CHECK(scheduler.rateScale() == 0.9);
        scheduler.setRateScale(std::numeric_limits<double>::quiet_NaN());
        CHECK(scheduler.rateScale() == 0.9);
        scheduler.setRateScale(std::numeric_limits<double>::infinity());
        CHECK(scheduler.rateScale() == 0.9);
    }
}

TEST_CASE("SimulationScheduler.CurrentStepIntervalIsTheNewestSegmentsInterval", "[Scheduler]")
{
    SimulationScheduler scheduler = makeScheduler(2u);
    CHECK(scheduler.currentStepInterval() == kDt);

    scheduler.setRateScale(1.05);
    CHECK(scheduler.currentStepInterval() == kDt / 1.05);
    const double deadline = scheduler.nextDeadline();
    CHECK(scheduler.pump(deadline + 10.0 * kDt) == 2u);
    REQUIRE(scheduler.lostTime().steps > 0u);
    CHECK(scheduler.currentStepInterval() == kDt / 1.05);
    const uint64_t next = scheduler.physicsStepCount();
    CHECK(scheduler.stepDeadline(next + 1u) - scheduler.stepDeadline(next) == Catch::Approx(kDt / 1.05).epsilon(1e-9));

    scheduler.reanchor(scheduler.nextDeadline() + 1.0);
    CHECK(scheduler.currentStepInterval() == kDt / 1.05);

    scheduler.setRateScale(2.0);
    CHECK(scheduler.currentStepInterval() == kDt / 1.1);
    scheduler.setRateScale(std::numeric_limits<double>::quiet_NaN());
    CHECK(scheduler.currentStepInterval() == kDt / 1.1);
}

TEST_CASE("SimulationScheduler.BackwardTimeIsIgnoredAndCounted", "[Scheduler]")
{
    SimulationScheduler scheduler = makeScheduler(8u);
    const double last = pumpSteady(scheduler, 5);
    const uint64_t steps = scheduler.physicsStepCount();
    const double next = scheduler.nextDeadline();
    REQUIRE(scheduler.ignoredTimeSamples() == 0u);

    CHECK(scheduler.pump(last - 0.5) == 0u);
    CHECK(scheduler.ignoredTimeSamples() == 1u);
    CHECK(scheduler.pump(last - 0.001) == 0u);
    CHECK(scheduler.ignoredTimeSamples() == 2u);

    CHECK(scheduler.pump(std::numeric_limits<double>::quiet_NaN()) == 0u);
    CHECK(scheduler.pump(std::numeric_limits<double>::infinity()) == 0u);
    CHECK(scheduler.ignoredTimeSamples() == 4u);

    scheduler.reanchor(last - 1.0);
    CHECK(scheduler.ignoredTimeSamples() == 5u);

    CHECK(scheduler.physicsStepCount() == steps);
    CHECK(scheduler.nextDeadline() == next);
    CHECK(scheduler.lostTime().steps == 0u);
    CHECK(scheduler.lostTime().seconds == 0.0);

    CHECK(scheduler.pump(next + kDt * 0.5) == 1u);
    CHECK(scheduler.ignoredTimeSamples() == 5u);
}

TEST_CASE("SimulationScheduler.ReanchorCountsTheGapAsLost", "[Scheduler]")
{
    SECTION("after a 10 s gap")
    {
        SimulationScheduler scheduler = makeScheduler(8u);
        const double last = pumpSteady(scheduler, 60);
        const double nextBefore = scheduler.nextDeadline();
        const uint64_t stepsBefore = scheduler.physicsStepCount();

        const double resume = last + 10.0;
        scheduler.reanchor(resume);
        CHECK(scheduler.lostTime().seconds == resume - nextBefore);
        CHECK(std::abs(scheduler.lostTime().seconds - 10.0) <= kDt);
        CHECK(scheduler.lostTime().steps == 599u);
        CHECK(scheduler.physicsStepCount() == stepsBefore);
        CHECK(scheduler.nextDeadline() == resume);

        CHECK(scheduler.pump(resume) == 1u);
        CHECK(scheduler.pump(resume + kDt * 0.5) == 0u);
        CHECK(scheduler.pump(resume + kDt * 1.5) == 1u);
        CHECK(scheduler.lostTime().steps == 599u);
    }

    SECTION("with no gap it changes nothing")
    {
        SimulationScheduler scheduler = makeScheduler(8u);
        const double last = pumpSteady(scheduler, 3);
        const double next = scheduler.nextDeadline();
        scheduler.reanchor(last);
        CHECK(scheduler.nextDeadline() == next);
        CHECK(scheduler.lostTime().seconds == 0.0);
        CHECK(scheduler.lostTime().steps == 0u);
    }
}

TEST_CASE("SimulationScheduler.StepDeadlineIsStrictlyIncreasingUnderRateChanges", "[Scheduler]")
{
    SimulationScheduler scheduler = makeScheduler(4u);
    std::vector<double> issued;
    const double scales[] = {1.0, 0.93, 1.07, 0.9, 1.1, 1.5, 0.2, 0.97, 1.02};

    uint32_t lcg = 12345u;
    double now = kStart;
    for (int frame = 0; frame < 3000; ++frame)
    {
        lcg = lcg * 1664525u + 1013904223u;
        now += 0.005 + 0.025 * static_cast<double>(lcg >> 8) / static_cast<double>(1u << 24);
        if (frame == 1000)
        {
            now += 0.3;
        }
        if (frame == 2000)
        {
            now += 5.0;
            scheduler.reanchor(now);
        }
        if (frame % 100 == 0)
        {
            scheduler.setRateScale(scales[(frame / 100) % 9]);
        }

        const uint64_t first = scheduler.physicsStepCount();
        const uint32_t steps = scheduler.pump(now);
        for (uint64_t step = first; step < first + steps; ++step)
        {
            const double deadline = scheduler.stepDeadline(step);
            INFO("frame " << frame << " step " << step);
            REQUIRE(deadline <= now);
            issued.push_back(deadline);
        }
        REQUIRE(scheduler.nextDeadline() > now);
    }

    REQUIRE(scheduler.lostTime().steps > 0u);
    REQUIRE(issued.size() == scheduler.physicsStepCount());
    for (size_t step = 1; step < issued.size(); ++step)
    {
        INFO("step " << step);
        REQUIRE(issued[step] > issued[step - 1]);
        REQUIRE(scheduler.stepDeadline(step) == issued[step]);
    }
    for (uint64_t step = 1; step < scheduler.physicsStepCount() + 16u; ++step)
    {
        REQUIRE(scheduler.stepDeadline(step) > scheduler.stepDeadline(step - 1u));
    }

    SECTION("past the retained history the deadlines stay strictly increasing")
    {
        SimulationScheduler churn = makeScheduler(8u);
        double t = kStart;
        for (int change = 0; change < 3 * static_cast<int>(SimulationScheduler::kDeadlineHistorySegments); ++change)
        {
            churn.setRateScale(change % 2 == 0 ? 0.95 : 1.05);
            t = churn.nextDeadline() + kDt * 0.5;
            REQUIRE(churn.pump(t) == 1u);
        }
        for (uint64_t step = 1; step < churn.physicsStepCount() + 16u; ++step)
        {
            REQUIRE(churn.stepDeadline(step) > churn.stepDeadline(step - 1u));
        }
    }
}

namespace
{
    class DequeSchedulerTranscription
    {
    public:
        static constexpr size_t kDeadlineHistorySegments = 64;

        DequeSchedulerTranscription(const SchedulerConfig& config, double startSeconds)
            : config_(config)
            , latestSeconds_(startSeconds)
        {
            OG_CHECK(std::isfinite(config.dtSeconds) && config.dtSeconds > 0.0,
                "DequeSchedulerTranscription: dtSeconds must be finite and positive");
            OG_CHECK(config.maxCatchUpSteps.value >= 1u,
                "DequeSchedulerTranscription: maxCatchUpSteps must be at least 1");
            OG_CHECK(std::isfinite(config.minRateScale) && std::isfinite(config.maxRateScale)
                && config.minRateScale > 0.0 && config.minRateScale <= 1.0 && config.maxRateScale >= 1.0,
                "DequeSchedulerTranscription: rate-scale bounds must satisfy 0 < min <= 1 <= max");
            OG_CHECK(std::isfinite(startSeconds), "DequeSchedulerTranscription: startSeconds must be finite");
            segments_.push_back(DeadlineSegment{0u, startSeconds, config.dtSeconds});
        }

        uint32_t pump(double nowSeconds)
        {
            if (!acceptTimeSample(nowSeconds))
            {
                return 0u;
            }

            const uint64_t due = stepsDueAt(nowSeconds);
            const uint64_t run = std::min<uint64_t>(due, config_.maxCatchUpSteps.value);
            const uint64_t dropped = due - run;
            if (dropped > 0u)
            {
                const double droppedFrom = deadlineAt(physicsSteps_);
                const double droppedUntil = deadlineAt(physicsSteps_ + dropped);
                recordLostTime(dropped, droppedUntil - droppedFrom);
                beginSegment(droppedUntil, currentInterval());
            }
            physicsSteps_ += run;
            return static_cast<uint32_t>(run);
        }

        void reanchor(double nowSeconds)
        {
            if (!acceptTimeSample(nowSeconds))
            {
                return;
            }

            const double next = nextDeadline();
            if (nowSeconds <= next)
            {
                return;
            }
            recordLostTime(stepsDueAt(nowSeconds) - 1u, nowSeconds - next);
            beginSegment(nowSeconds, currentInterval());
        }

        void setRateScale(double scale)
        {
            if (!std::isfinite(scale))
            {
                return;
            }
            const double clamped = std::clamp(scale, config_.minRateScale, config_.maxRateScale);
            if (clamped == rateScale_)
            {
                return;
            }
            rateScale_ = clamped;
            beginSegment(nextDeadline(), config_.dtSeconds / rateScale_);
        }

        double rateScale() const { return rateScale_; }

        double nextDeadline() const { return deadlineAt(physicsSteps_); }

        double stepDeadline(uint64_t step) const { return deadlineAt(step); }

        uint64_t physicsStepCount() const { return physicsSteps_; }

        LostTime lostTime() const { return lost_; }

        uint64_t ignoredTimeSamples() const { return ignoredTimeSamples_; }

        const SchedulerConfig& config() const { return config_; }

        uint64_t oldestKeptStep() const { return segments_.front().firstStep; }

        uint64_t segmentsBegun() const { return segmentsBegun_; }

    private:
        struct DeadlineSegment
        {
            uint64_t firstStep;
            double firstDeadline;
            double interval;
        };

        bool acceptTimeSample(double nowSeconds)
        {
            if (!std::isfinite(nowSeconds) || nowSeconds < latestSeconds_)
            {
                ++ignoredTimeSamples_;
                return false;
            }
            latestSeconds_ = nowSeconds;
            return true;
        }

        void recordLostTime(uint64_t steps, double seconds)
        {
            lost_.steps += steps;
            lost_.seconds += seconds;
        }

        double currentInterval() const { return segments_.back().interval; }

        const DeadlineSegment& segmentFor(uint64_t step) const
        {
            for (auto it = segments_.rbegin(); it != segments_.rend(); ++it)
            {
                if (it->firstStep <= step)
                {
                    return *it;
                }
            }
            return segments_.front();
        }

        double deadlineAt(uint64_t step) const
        {
            const DeadlineSegment& segment = segmentFor(step);
            const double offset = static_cast<double>(step) - static_cast<double>(segment.firstStep);
            return segment.firstDeadline + offset * segment.interval;
        }

        uint64_t stepsDueAt(double nowSeconds) const
        {
            const double first = nextDeadline();
            if (nowSeconds < first)
            {
                return 0u;
            }
            constexpr double kMaxEstimate = 4503599627370496.0;
            const double estimate = std::min(std::floor((nowSeconds - first) / currentInterval()) + 1.0, kMaxEstimate);
            uint64_t due = static_cast<uint64_t>(estimate);
            for (int fix = 0; fix < 4 && due > 1u && deadlineAt(physicsSteps_ + due - 1u) > nowSeconds; ++fix)
            {
                --due;
            }
            for (int fix = 0; fix < 4 && deadlineAt(physicsSteps_ + due) <= nowSeconds; ++fix)
            {
                ++due;
            }
            return due;
        }

        void beginSegment(double firstDeadline, double interval)
        {
            if (segments_.back().firstStep == physicsSteps_)
            {
                segments_.back() = DeadlineSegment{physicsSteps_, firstDeadline, interval};
                return;
            }
            segments_.push_back(DeadlineSegment{physicsSteps_, firstDeadline, interval});
            ++segmentsBegun_;
            if (segments_.size() > kDeadlineHistorySegments)
            {
                segments_.pop_front();
            }
        }

        SchedulerConfig config_;
        std::deque<DeadlineSegment> segments_;
        uint64_t physicsSteps_ = 0u;
        double rateScale_ = 1.0;
        double latestSeconds_;
        LostTime lost_;
        uint64_t ignoredTimeSamples_ = 0u;
        uint64_t segmentsBegun_ = 0u;
    };
}

TEST_CASE("SimulationScheduler.LostTime.ByCauseSplitsCatchUpDropsFromReanchorGaps", "[Scheduler]")
{
    SimulationScheduler scheduler = makeScheduler(3u);
    const double last = pumpSteady(scheduler, 10);
    REQUIRE(scheduler.lostTime().steps == 0u);

    const double droppedFrom = scheduler.nextDeadline();
    const double hitch = last + 0.250;
    REQUIRE(scheduler.pump(hitch) == 3u);
    const double droppedUntil = scheduler.stepDeadline(10u);
    const LostTimeByCause afterHitch = scheduler.lostTimeByCause();
    CHECK(afterHitch.catchUp.steps == 12u);
    CHECK(afterHitch.catchUp.seconds == droppedUntil - droppedFrom);
    CHECK(std::abs(afterHitch.catchUp.seconds - 12.0 * kDt) <= 1e-9);
    CHECK(afterHitch.reanchor.steps == 0u);
    CHECK(afterHitch.reanchor.seconds == 0.0);
    CHECK(scheduler.lostTime().steps == afterHitch.catchUp.steps);
    CHECK(scheduler.lostTime().seconds == afterHitch.catchUp.seconds);

    const double resumeFrom = pumpSteady(scheduler, 5);
    const double gapFrom = scheduler.nextDeadline();
    const double resume = resumeFrom + 2.0;
    scheduler.reanchor(resume);
    const LostTimeByCause afterGap = scheduler.lostTimeByCause();
    CHECK(afterGap.reanchor.steps == 119u);
    CHECK(afterGap.reanchor.seconds == resume - gapFrom);
    CHECK(afterGap.catchUp.steps == afterHitch.catchUp.steps);
    CHECK(afterGap.catchUp.seconds == afterHitch.catchUp.seconds);
    CHECK(scheduler.lostTime().steps == afterGap.catchUp.steps + afterGap.reanchor.steps);
    CHECK(scheduler.lostTime().seconds == afterGap.catchUp.seconds + afterGap.reanchor.seconds);
    CHECK(scheduler.lostTime().steps == 131u);

    CHECK(scheduler.pump(resume) == 1u);
    CHECK(scheduler.lostTimeByCause().catchUp.steps == 12u);
    CHECK(scheduler.lostTimeByCause().reanchor.steps == 119u);
}

TEST_CASE("SimulationScheduler.Segments.TheRingKeepsTheDequesDeadlinesOverManySegments", "[Scheduler]")
{
    SimulationScheduler scheduler = makeScheduler(4u);
    DequeSchedulerTranscription reference(SchedulerConfig{kDt, 4u}, kStart);
    const double scales[] = {0.93, 1.07, 0.98, 1.02};

    uint64_t mismatchedDeadlines = 0u;
    int dropPumps = 0;
    for (int pump = 0; pump < 1000; ++pump)
    {
        if (pump % 3 == 0)
        {
            scheduler.setRateScale(scales[(pump / 3) % 4]);
            reference.setRateScale(scales[(pump / 3) % 4]);
        }
        const double now = reference.nextDeadline() + ((pump == 400 || pump == 800) ? 0.5 : kDt * 0.5);
        const uint64_t lostBefore = reference.lostTime().steps;
        INFO("pump " << pump);
        REQUIRE(scheduler.pump(now) == reference.pump(now));
        dropPumps += reference.lostTime().steps > lostBefore ? 1 : 0;
        REQUIRE(scheduler.nextDeadline() == reference.nextDeadline());
        for (uint64_t step = reference.oldestKeptStep(); step <= reference.physicsStepCount() + 10u; ++step)
        {
            mismatchedDeadlines += scheduler.stepDeadline(step) == reference.stepDeadline(step) ? 0u : 1u;
        }
    }
    CHECK(mismatchedDeadlines == 0u);
    REQUIRE(dropPumps == 2);
    REQUIRE(reference.segmentsBegun() >= 300u);
    CHECK(scheduler.physicsStepCount() == reference.physicsStepCount());
    CHECK(scheduler.lostTime().steps == reference.lostTime().steps);
    CHECK(scheduler.lostTime().seconds == reference.lostTime().seconds);

    for (uint64_t step = reference.oldestKeptStep(); step <= reference.physicsStepCount() + 10u; ++step)
    {
        INFO("step " << step << " oldest kept " << reference.oldestKeptStep());
        REQUIRE(scheduler.stepDeadline(step) == reference.stepDeadline(step));
    }
}

#endif // WITH_LOW_LEVEL_TESTS
