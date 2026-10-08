// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationScheduler.h"

#include <cmath>
#include <cstdint>
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

#endif // WITH_LOW_LEVEL_TESTS
