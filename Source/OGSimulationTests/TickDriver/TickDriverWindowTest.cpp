// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cfloat>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/LatencyBudgetProbe.h"
#include "OGSimulation/SimulationScheduler.h"
#include "OGTickDriver/TickDriverWindow.h"

namespace tickDriverWindowTest
{
    struct WindowAllocationCounter
    {
        static inline uint64_t allocations = 0u;
    };

    template <typename T>
    struct WindowCountingAllocator
    {
        using value_type = T;

        WindowCountingAllocator() = default;

        template <typename U>
        WindowCountingAllocator(const WindowCountingAllocator<U>&) noexcept
        {
        }

        T* allocate(std::size_t count)
        {
            ++WindowAllocationCounter::allocations;
            return std::allocator<T>{}.allocate(count);
        }

        void deallocate(T* pointer, std::size_t count) noexcept { std::allocator<T>{}.deallocate(pointer, count); }

        template <typename U>
        bool operator==(const WindowCountingAllocator<U>&) const noexcept
        {
            return true;
        }
    };

    using CountingWindow        = BasicTickDriverWindow<WindowCountingAllocator<char>>;
    using CountingFirstStepLine = BasicTickDriverFirstStepLine<WindowCountingAllocator<char>>;
    using Series                = TickDriverWindowSeries;

    constexpr Series kAllSeries[] = {
        Series::FrameMs, Series::StartPhysicsUs, Series::EndWaitUs, Series::PostPhysicsUs,
        Series::J2WaitUs, Series::StepUs, Series::PreUs, Series::ShadowRestoreUs,
    };

    TickDriverWindowTotals totals(uint64_t lostSteps, double lostSeconds, uint64_t ignored, uint64_t j1Drops, uint64_t j6Drops)
    {
        TickDriverWindowTotals result;
        result.lost.steps         = lostSteps;
        result.lost.seconds       = lostSeconds;
        result.ignoredTimeSamples = ignored;
        result.j1Drops            = j1Drops;
        result.j6Drops            = j6Drops;
        return result;
    }

    template <typename WindowT>
    void fillAuthorityInlineWindow(WindowT& window)
    {
        window.restart(100.0, totals(5u, 0.25, 7u, 3u, 4u));
        window.noteFramePeriod(0.020);
        window.noteFramePeriod(0.016);
        window.noteFramePeriod(0.0175);
        window.noteStartPhysics(2u, 0.0001);
        window.noteStartPhysics(0u, 0.00005);
        window.noteStartPhysics(3u, 0.0002);
        window.notePostPhysics(0.0012);
        window.noteWorldLockWait(0.000003);
        window.noteStepCost(0.00001, 0.0005, 0u, false);
        window.noteStepCost(0.00001, 0.0004, 0u, false);
    }

    constexpr const char* kAuthorityInlineGolden =
        "[SimHost.Window] role=Authority stepping=inline seconds=10.50 frames=3 frameMs=17.50/20.00/20.00 "
        "startPhysicsUs=100.0/200.0/200.0 endWaits=0 endWaitUs=- postPhysicsUs=1200.0/1200.0/1200.0 steps=5 "
        "maxStepsPerFrame=3 lostSteps=3 lostSeconds=0.0625 ignoredTimeSamples=2 j1Drops=0 j2Waits=1 j2WaitUs=3.0/3.0/3.0 "
        "costSamples=2 stepUs=400.0/500.0/500.0 preTicks=2 preUs=10.0/10.0/10.0 replayDepth=- resimRefused=0 j6Drops=2 "
        "shadowRestores=0 shadowRestoreUs=- shadowRefused=0 sampleOverflow=0";

    constexpr const char* kClientWorkerGolden =
        "[SimHost.Window] role=Client stepping=worker seconds=10.00 frames=1 frameMs=- startPhysicsUs=20.0/20.0/20.0 "
        "endWaits=1 endWaitUs=250.0/250.0/250.0 postPhysicsUs=- steps=1 maxStepsPerFrame=1 lostSteps=0 "
        "lostSeconds=0.0000 ignoredTimeSamples=0 j1Drops=0 j2Waits=0 j2WaitUs=- costSamples=3 stepUs=600.0/900.0/900.0 "
        "preTicks=2 preUs=40.0/80.0/80.0 replayDepth=3.50/4 resimRefused=1 j6Drops=0 shadowRestores=2 "
        "shadowRestoreUs=150.0/350.0/350.0 shadowRefused=1 sampleOverflow=0";

    constexpr const char* kEmptyClientInlineGolden =
        "[SimHost.Window] role=Client stepping=inline seconds=0.00 frames=0 frameMs=- startPhysicsUs=- endWaits=0 "
        "endWaitUs=- postPhysicsUs=- steps=0 maxStepsPerFrame=0 lostSteps=0 lostSeconds=0.0000 ignoredTimeSamples=0 "
        "j1Drops=0 j2Waits=0 j2WaitUs=- costSamples=0 stepUs=- preTicks=0 preUs=- replayDepth=- resimRefused=0 "
        "j6Drops=0 shadowRestores=0 shadowRestoreUs=- shadowRefused=0 sampleOverflow=0";
} // namespace tickDriverWindowTest

using namespace tickDriverWindowTest;

TEST_CASE("TickDriverWindow.WindowLine.Goldens", "[TickDriverWindow]")
{
    SECTION("authority, inline: every series that has samples, frameMs with two decimals")
    {
        TickDriverWindow window(true, true);
        fillAuthorityInlineWindow(window);
        CHECK(std::string(window.formatWindowLine(110.5, totals(8u, 0.3125, 9u, 3u, 6u))) == kAuthorityInlineGolden);
    }

    SECTION("client, worker: end waits, replays, refusals and shadow restores")
    {
        TickDriverWindow window(false, false);
        window.restart(0.0, TickDriverWindowTotals{});
        window.noteStartPhysics(1u, 0.00002);
        window.noteEndWait(0.00025);
        window.noteStepCost(0.00002, 0.0003, 0u, false);
        window.noteStepCost(0.00004, 0.0006, 3u, false);
        window.noteStepCost(0.00008, 0.0009, 4u, true);
        window.noteShadowRestore(0.00015, false);
        window.noteShadowRestore(0.00035, true);
        CHECK(std::string(window.formatWindowLine(10.0, TickDriverWindowTotals{})) == kClientWorkerGolden);
    }

    SECTION("an empty window prints '-' for every series")
    {
        TickDriverWindow window(false, true);
        window.restart(3.0, TickDriverWindowTotals{});
        CHECK(std::string(window.formatWindowLine(3.0, TickDriverWindowTotals{})) == kEmptyClientInlineGolden);
    }
}

TEST_CASE("TickDriverWindow.FrameAndFirstStepLines.Goldens", "[TickDriverWindow]")
{
    TickDriverWindow authority(true, true);
    CHECK(std::string(authority.formatFrameLine(1.0 / 60.0, 60u, 0.75, 1.25)) ==
        "[SimHost.Frame] role=Authority stepping=inline dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] "
        "timeBase=worldDelta");

    TickDriverWindow client(false, false);
    CHECK(std::string(client.formatFrameLine(0.03125, 240u, 0.75, 1.25)) ==
        "[SimHost.Frame] role=Client stepping=worker dt=0.031250 maxCatchUpSteps=240 rateScale=[0.75,1.25] "
        "timeBase=worldDelta");
    CHECK(std::string(client.formatFrameLine(0.03125, 240u, 0.75, 1.25, 0.0, false, true)) ==
        "[SimHost.Frame] role=Client stepping=worker dt=0.031250 maxCatchUpSteps=240 rateScale=[0.75,1.25] "
        "timeBase=worldDelta frameWait=thisFramesSingleStep");
    CHECK(std::string(client.formatFrameLine(0.03125, 240u, 0.9, 1.1)) ==
        "[SimHost.Frame] role=Client stepping=worker dt=0.031250 maxCatchUpSteps=240 rateScale=[0.90,1.10] "
        "timeBase=worldDelta");
    CHECK(std::string(authority.formatFrameLine(1.0 / 60.0, 60u, 0.99, 1.005, 0.0, true)) ==
        "[SimHost.Frame] role=Authority stepping=inline dt=0.016667 maxCatchUpSteps=60 rateScale=[0.99,1.00] "
        "timeBase=monotonic");

    CHECK(std::string(client.formatCatchUpCapWarningLine(0, 240u, 1u)) ==
        "[SimHost.Frame] og.Sim.MaxCatchUpSteps=0 is outside [1, 240]; using 1");
    CHECK(std::string(client.formatCatchUpCapWarningLine(999, 240u, 240u)) ==
        "[SimHost.Frame] og.Sim.MaxCatchUpSteps=999 is outside [1, 240]; using 240");
    CHECK(std::string(client.formatCatchUpCapWarningLine(-5, 240u, 1u)) ==
        "[SimHost.Frame] og.Sim.MaxCatchUpSteps=-5 is outside [1, 240]; using 1");

    TickDriverFirstStepLine authorityFirst(true);
    CHECK(std::string(authorityFirst.format(true, 0u, 1u)) ==
        "[SimHost.FirstStep] role=Authority thread=game physicsStep=0 batch=1");

    TickDriverFirstStepLine clientFirst(false);
    CHECK(std::string(clientFirst.format(false, 12345u, 3u)) ==
        "[SimHost.FirstStep] role=Client thread=worker physicsStep=12345 batch=3");
    CHECK(std::string(clientFirst.format(true, std::numeric_limits<uint64_t>::max(), 240u)) ==
        "[SimHost.FirstStep] role=Client thread=game physicsStep=18446744073709551615 batch=240");
}

TEST_CASE("TickDriverWindow.Restart.ClearsCountsAndSamplesAndTakesTheNewTotals", "[TickDriverWindow]")
{
    TickDriverWindow window(true, true);
    fillAuthorityInlineWindow(window);
    for (int index = 0; index < 16385; ++index)
        window.noteFramePeriod(0.001);
    REQUIRE(window.sampleOverflow() > 0u);

    window.restart(200.0, totals(8u, 0.3125, 9u, 3u, 6u));
    CHECK(window.counts().startSeconds == 200.0);
    CHECK(window.counts().frames == 0u);
    CHECK(window.counts().steps == 0u);
    CHECK(window.counts().atStart.lost.steps == 8u);
    CHECK(window.counts().atStart.j6Drops == 6u);
    CHECK(window.sampleOverflow() == 0u);
    for (const Series series : kAllSeries)
        CHECK(window.samples(series).count() == 0u);

    CHECK_FALSE(window.isDue(209.999));
    CHECK(window.isDue(200.0 + tickDriverWindow::kWindowSeconds));

    const std::string line = window.formatWindowLine(200.0, totals(8u, 0.3125, 9u, 3u, 6u));
    CHECK(line.find("lostSteps=0 lostSeconds=0.0000 ignoredTimeSamples=0 j1Drops=0") != std::string::npos);
    CHECK(line.find("j6Drops=0") != std::string::npos);
    CHECK(line.find("frameMs=- ") != std::string::npos);
}

TEST_CASE("TickDriverWindow.Percentiles.NearestRankLikeTheLatencyProbe", "[TickDriverWindow]")
{
    for (const uint32_t n : { 1u, 2u, 3u, 7u, 100u, 101u, 1000u, 16384u })
    {
        CAPTURE(n);
        std::vector<float> values;
        for (uint32_t index = 0u; index < n; ++index)
            values.push_back(static_cast<float>((index * 7919u) % n + 1u));

        const TickDriverPercentiles p = tickDriverPercentilesOf(values.data(), n);
        CHECK(p.n == n);
        CHECK(p.p50 == static_cast<double>(latencyBudget::nearestRankIndex(n, 500u) + 1u));
        CHECK(p.p99 == static_cast<double>(latencyBudget::nearestRankIndex(n, 990u) + 1u));
        CHECK(p.max == static_cast<double>(n));
    }

    const TickDriverPercentiles none = tickDriverPercentilesOf(nullptr, 0u);
    CHECK(none.n == 0u);

    TickDriverWindow window(false, true);
    window.restart(0.0, TickDriverWindowTotals{});
    for (int value = 100; value >= 1; --value)
        window.samples(Series::StepUs).add(static_cast<double>(value));
    CHECK(std::string(window.formatWindowLine(0.0, TickDriverWindowTotals{})).find("stepUs=50.0/99.0/100.0 ") !=
        std::string::npos);
}

TEST_CASE("TickDriverWindow.Samples.OverflowIsCountedWithoutAllocationAfterConstruction", "[TickDriverWindow]")
{
    CountingWindow window(false, false);
    CountingFirstStepLine firstStep(false);
    REQUIRE(WindowAllocationCounter::allocations > 0u);
    const uint64_t afterConstruction = WindowAllocationCounter::allocations;

    SECTION("16,385 samples in one series report sampleOverflow=1")
    {
        window.restart(0.0, TickDriverWindowTotals{});
        for (int index = 0; index < 16385; ++index)
            window.noteFramePeriod(0.001 * static_cast<double>(index % 50));

        CHECK(window.samples(Series::FrameMs).count() == tickDriverWindow::kMaxSamplesPerSeries);
        CHECK(window.samples(Series::FrameMs).overflow() == 1u);
        const std::string line = window.formatWindowLine(10.0, TickDriverWindowTotals{});
        CHECK(line.find(" sampleOverflow=1") == line.size() - std::string(" sampleOverflow=1").size());
        CHECK(WindowAllocationCounter::allocations == afterConstruction);
    }

    SECTION("every series full, at the extremes, through every formatter, twice")
    {
        for (int pass = 0; pass < 2; ++pass)
        {
            window.restart(-DBL_MAX, totals(0u, 0.0, 0u, 0u, 0u));
            for (const Series series : kAllSeries)
            {
                for (uint32_t index = 0u; index <= tickDriverWindow::kMaxSamplesPerSeries; ++index)
                    window.samples(series).add(index % 2u == 0u ? -static_cast<double>(FLT_MAX) : static_cast<double>(FLT_MAX));
            }
            TickDriverWindowCounts& counts = window.counts();
            counts.frames           = std::numeric_limits<uint32_t>::max();
            counts.steps            = std::numeric_limits<uint64_t>::max();
            counts.maxStepsPerFrame = std::numeric_limits<uint32_t>::max();
            counts.endWaits         = std::numeric_limits<uint32_t>::max();
            counts.j2Waits          = std::numeric_limits<uint32_t>::max();
            counts.replays          = 1u;
            counts.replayedTicks    = std::numeric_limits<uint64_t>::max();
            counts.maxReplayDepth   = std::numeric_limits<uint32_t>::max();
            counts.resimRefused     = std::numeric_limits<uint32_t>::max();
            counts.shadowRefused    = std::numeric_limits<uint32_t>::max();

            const TickDriverWindowTotals maxima = totals(std::numeric_limits<uint64_t>::max(), DBL_MAX,
                std::numeric_limits<uint64_t>::max(), std::numeric_limits<uint64_t>::max(), std::numeric_limits<uint64_t>::max());
            const std::string line = window.formatWindowLine(0.0, maxima);
            CHECK(line.size() <= tickDriverWindow::kWindowLineReserve);
            CHECK(line.find(" sampleOverflow=8") == line.size() - std::string(" sampleOverflow=8").size());
            CHECK(line.find(" replayDepth=18446744073709551616.00/4294967295 ") != std::string::npos);

            CHECK(std::string(window.formatFrameLine(-DBL_MAX, std::numeric_limits<uint32_t>::max(), -DBL_MAX, -DBL_MAX)).size() <=
                tickDriverWindow::kFrameLineReserve);
            CHECK(std::string(window.formatFrameLine(-DBL_MAX, std::numeric_limits<uint32_t>::max(), -DBL_MAX, -DBL_MAX, 0.0, true,
                true)).size() <= tickDriverWindow::kFrameLineReserve);
            CHECK(std::string(window.formatCatchUpCapWarningLine(std::numeric_limits<int32_t>::min(),
                std::numeric_limits<uint32_t>::max(), std::numeric_limits<uint32_t>::max())).size() <=
                tickDriverWindow::kFrameLineReserve);
            CHECK(std::string(firstStep.format(false, std::numeric_limits<uint64_t>::max(),
                std::numeric_limits<uint32_t>::max())).size() <= tickDriverWindow::kFirstStepLineReserve);
        }
    }

    SECTION("the note doors and restart")
    {
        window.restart(1.0, TickDriverWindowTotals{});
        for (uint32_t index = 0u; index < 20000u; ++index)
        {
            window.noteFramePeriod(0.016);
            window.noteStartPhysics(index % 3u, 0.0001);
            window.noteEndWait(0.0002);
            window.notePostPhysics(0.0003);
            window.noteWorldLockWait(0.0004);
            window.noteShadowRestore(0.0005, index % 2u == 0u);
            window.noteStepCost(0.00001, 0.0006, index % 4u, index % 5u == 0u);
        }
        CHECK(window.samples(Series::PreUs).count() == 15000u);
        CHECK(window.sampleOverflow() == 7u * (20000u - tickDriverWindow::kMaxSamplesPerSeries));
        window.formatWindowLine(12.0, TickDriverWindowTotals{});
        window.restart(12.0, TickDriverWindowTotals{});
        window.formatWindowLine(13.0, TickDriverWindowTotals{});
        CHECK(WindowAllocationCounter::allocations == afterConstruction);
    }
}

namespace tickDriverWindowFreeRunningTest
{
    constexpr Series kSuffixSeries[] = { Series::LateUs, Series::ApproachUs };

    template <typename WindowT>
    void fillEverySeries(WindowT& window)
    {
        fillAuthorityInlineWindow(window);
        window.noteEndWait(0.00025);
        window.noteShadowRestore(0.00015, false);
        window.noteLateness(0.000012);
        window.noteLateness(0.000004);
        window.noteLateness(0.00003);
        window.noteApproach(0.0005);
        window.noteApproach(0.0003);
        window.noteActivityWake();
        window.noteActivityWake();
        window.noteEarlyWake();
        window.noteIdleFallback();
        window.noteIdleFallback();
        window.noteIdleFallback();
    }

    constexpr const char* kFreeRunningGolden =
        "[SimHost.Window] role=Authority stepping=runner seconds=10.50 frames=3 frameMs=17.50/20.00/20.00 "
        "startPhysicsUs=100.0/200.0/200.0 endWaits=1 endWaitUs=250.0/250.0/250.0 postPhysicsUs=1200.0/1200.0/1200.0 steps=5 "
        "maxStepsPerFrame=3 lostSteps=3 lostSeconds=0.0625 ignoredTimeSamples=2 j1Drops=0 j2Waits=1 j2WaitUs=3.0/3.0/3.0 "
        "costSamples=2 stepUs=400.0/500.0/500.0 preTicks=2 preUs=10.0/10.0/10.0 replayDepth=- resimRefused=0 j6Drops=2 "
        "shadowRestores=1 shadowRestoreUs=150.0/150.0/150.0 shadowRefused=0 sampleOverflow=0 "
        "lateUs=12.0/30.0/30.0 activityWakes=2 earlyWakes=1 idleFallbacks=3 approachUs=300.0/500.0/500.0";

    constexpr const char* kEmptyFreeRunningGolden =
        "[SimHost.Window] role=Authority stepping=runner seconds=0.00 frames=0 frameMs=- startPhysicsUs=- endWaits=0 "
        "endWaitUs=- postPhysicsUs=- steps=0 maxStepsPerFrame=0 lostSteps=0 lostSeconds=0.0000 ignoredTimeSamples=0 "
        "j1Drops=0 j2Waits=0 j2WaitUs=- costSamples=0 stepUs=- preTicks=0 preUs=- replayDepth=- resimRefused=0 "
        "j6Drops=0 shadowRestores=0 shadowRestoreUs=- shadowRefused=0 sampleOverflow=0 "
        "lateUs=- activityWakes=0 earlyWakes=0 idleFallbacks=0 approachUs=-";

    std::string withoutSuffix(const std::string& line, const char* steppingWord)
    {
        std::string result = line.substr(0u, line.find(" lateUs="));
        const std::string runner = "stepping=runner ";
        result.replace(result.find(runner), runner.size(), std::string("stepping=") + steppingWord + " ");
        return result;
    }
} // namespace tickDriverWindowFreeRunningTest

using namespace tickDriverWindowFreeRunningTest;

TEST_CASE("TickDriverWindow.FreeRunning.LinesMatchTheGoldens", "[TickDriverWindow]")
{
    SECTION("the window line: every series has samples, and the suffix follows sampleOverflow")
    {
        TickDriverWindow window(true, StepThreading::FreeRunning);
        CHECK(window.threading() == StepThreading::FreeRunning);
        CHECK_FALSE(window.stepsInline());
        fillEverySeries(window);
        for (const Series series : kAllSeries)
            CHECK(window.samples(series).count() > 0u);
        for (const Series series : kSuffixSeries)
            CHECK(window.samples(series).count() > 0u);
        CHECK(std::string(window.formatWindowLine(110.5, totals(8u, 0.3125, 9u, 3u, 6u))) == kFreeRunningGolden);
    }

    SECTION("an empty window prints '-' for both suffix series and zero wakes")
    {
        TickDriverWindow window(true, StepThreading::FreeRunning);
        window.restart(3.0, TickDriverWindowTotals{});
        CHECK(std::string(window.formatWindowLine(3.0, TickDriverWindowTotals{})) == kEmptyFreeRunningGolden);
    }

    SECTION("restart clears the suffix series and the wake counts")
    {
        TickDriverWindow window(true, StepThreading::FreeRunning);
        fillEverySeries(window);
        window.restart(3.0, TickDriverWindowTotals{});
        CHECK(window.counts().activityWakes == 0u);
        CHECK(window.counts().earlyWakes == 0u);
        CHECK(window.counts().idleFallbacks == 0u);
        CHECK(std::string(window.formatWindowLine(3.0, TickDriverWindowTotals{})) == kEmptyFreeRunningGolden);
    }

    SECTION("the frame line: stepping=runner, timeBase=monotonic and the slack in whole microseconds")
    {
        TickDriverWindow window(true, StepThreading::FreeRunning);
        CHECK(std::string(window.formatFrameLine(1.0 / 60.0, 60u, 0.75, 1.25, 0.0005)) ==
            "[SimHost.Frame] role=Authority stepping=runner dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] "
            "timeBase=monotonic idleSlackUs=500");
        CHECK(std::string(window.formatFrameLine(1.0 / 60.0, 60u, 0.75, 1.25)) ==
            "[SimHost.Frame] role=Authority stepping=runner dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] "
            "timeBase=monotonic idleSlackUs=0");
        CHECK(std::string(window.formatFrameLine(0.03125, 240u, 0.75, 1.25, 0.0003)) ==
            "[SimHost.Frame] role=Authority stepping=runner dt=0.031250 maxCatchUpSteps=240 rateScale=[0.75,1.25] "
            "timeBase=monotonic idleSlackUs=300");
        CHECK(std::string(window.formatFrameLine(0.03125, 240u, 0.75, 1.25, 0.0002996)) ==
            "[SimHost.Frame] role=Authority stepping=runner dt=0.031250 maxCatchUpSteps=240 rateScale=[0.75,1.25] "
            "timeBase=monotonic idleSlackUs=300");
        CHECK(std::string(window.formatFrameLine(0.03125, 240u, 0.75, 1.25, 0.0000006)) ==
            "[SimHost.Frame] role=Authority stepping=runner dt=0.031250 maxCatchUpSteps=240 rateScale=[0.75,1.25] "
            "timeBase=monotonic idleSlackUs=1");
        CHECK(std::string(window.formatFrameLine(0.03125, 240u, 0.75, 1.25, 0.0000004)) ==
            "[SimHost.Frame] role=Authority stepping=runner dt=0.031250 maxCatchUpSteps=240 rateScale=[0.75,1.25] "
            "timeBase=monotonic idleSlackUs=0");
        CHECK(std::string(window.formatFrameLine(0.03125, 240u, 0.75, 1.25, -0.001)) ==
            "[SimHost.Frame] role=Authority stepping=runner dt=0.031250 maxCatchUpSteps=240 rateScale=[0.75,1.25] "
            "timeBase=monotonic idleSlackUs=0");
        CHECK(std::string(window.formatFrameLine(0.03125, 240u, 0.75, 1.25, 1.0e4)) ==
            "[SimHost.Frame] role=Authority stepping=runner dt=0.031250 maxCatchUpSteps=240 rateScale=[0.75,1.25] "
            "timeBase=monotonic idleSlackUs=4294967295");
        CHECK(std::string(window.formatFrameLine(1.0 / 60.0, 60u, 0.9, 1.1, 0.0005)) ==
            "[SimHost.Frame] role=Authority stepping=runner dt=0.016667 maxCatchUpSteps=60 rateScale=[0.90,1.10] "
            "timeBase=monotonic idleSlackUs=500");
        CHECK(std::string(window.formatCatchUpCapWarningLine(0, 240u, 1u)) ==
            "[SimHost.Frame] og.Sim.MaxCatchUpSteps=0 is outside [1, 240]; using 1");
    }

    SECTION("the first-step line: thread=runner in FreeRunning, game or worker otherwise")
    {
        TickDriverFirstStepLine first(true);
        CHECK(std::string(first.format(StepThreading::FreeRunning, true, 0u, 2u)) ==
            "[SimHost.FirstStep] role=Authority thread=runner physicsStep=0 batch=2");
        CHECK(std::string(first.format(StepThreading::FreeRunning, false, 12345u, 3u)) ==
            "[SimHost.FirstStep] role=Authority thread=runner physicsStep=12345 batch=3");
        CHECK(std::string(first.format(StepThreading::Inline, true, 0u, 2u)) ==
            "[SimHost.FirstStep] role=Authority thread=game physicsStep=0 batch=2");
        CHECK(std::string(first.format(StepThreading::FrameBounded, false, 7u, 1u)) ==
            "[SimHost.FirstStep] role=Authority thread=worker physicsStep=7 batch=1");
        CHECK(std::string(first.format(StepThreading::Inline, false, 7u, 1u)) ==
            "[SimHost.FirstStep] role=Authority thread=worker physicsStep=7 batch=1");
        CHECK(std::string(first.format(StepThreading::FrameBounded, true, 7u, 1u)) ==
            "[SimHost.FirstStep] role=Authority thread=game physicsStep=7 batch=1");
    }
}

TEST_CASE("TickDriverWindow.FreeRunning.TheSuffixIsAbsentInInlineAndFrameBounded", "[TickDriverWindow]")
{
    TickDriverWindow freeRunning(true, StepThreading::FreeRunning);
    fillEverySeries(freeRunning);
    const std::string freeRunningLine = freeRunning.formatWindowLine(110.5, totals(8u, 0.3125, 9u, 3u, 6u));
    REQUIRE(freeRunningLine.find(" lateUs=") != std::string::npos);

    SECTION("Inline, through the StepThreading constructor and the bool one")
    {
        TickDriverWindow byThreading(true, StepThreading::Inline);
        TickDriverWindow byBool(true, true);
        CHECK(byThreading.threading() == StepThreading::Inline);
        CHECK(byBool.threading() == StepThreading::Inline);
        CHECK(byThreading.stepsInline());
        CHECK(byBool.stepsInline());
        fillEverySeries(byThreading);
        fillEverySeries(byBool);
        const std::string line = byThreading.formatWindowLine(110.5, totals(8u, 0.3125, 9u, 3u, 6u));
        CHECK(line.find(" lateUs=") == std::string::npos);
        CHECK(line.find("approachUs") == std::string::npos);
        CHECK(line.find("Wakes") == std::string::npos);
        CHECK(line == withoutSuffix(freeRunningLine, "inline"));
        CHECK(std::string(byBool.formatWindowLine(110.5, totals(8u, 0.3125, 9u, 3u, 6u))) == line);
        CHECK(std::string(byThreading.formatFrameLine(1.0 / 60.0, 60u, 0.75, 1.25, 0.0005)) ==
            "[SimHost.Frame] role=Authority stepping=inline dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] "
            "timeBase=worldDelta");
    }

    SECTION("FrameBounded, through the StepThreading constructor and the bool one")
    {
        TickDriverWindow byThreading(true, StepThreading::FrameBounded);
        TickDriverWindow byBool(true, false);
        CHECK(byThreading.threading() == StepThreading::FrameBounded);
        CHECK(byBool.threading() == StepThreading::FrameBounded);
        CHECK_FALSE(byThreading.stepsInline());
        CHECK_FALSE(byBool.stepsInline());
        fillEverySeries(byThreading);
        fillEverySeries(byBool);
        const std::string line = byThreading.formatWindowLine(110.5, totals(8u, 0.3125, 9u, 3u, 6u));
        CHECK(line.find(" lateUs=") == std::string::npos);
        CHECK(line.find("approachUs") == std::string::npos);
        CHECK(line.find("Wakes") == std::string::npos);
        CHECK(line == withoutSuffix(freeRunningLine, "worker"));
        CHECK(std::string(byBool.formatWindowLine(110.5, totals(8u, 0.3125, 9u, 3u, 6u))) == line);
        CHECK(std::string(byThreading.formatFrameLine(1.0 / 60.0, 60u, 0.75, 1.25, 0.0005)) ==
            "[SimHost.Frame] role=Authority stepping=worker dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] "
            "timeBase=worldDelta");
    }
}

TEST_CASE("TickDriverWindow.FreeRunning.NoAllocationAfterConstruction", "[TickDriverWindow]")
{
    CountingWindow window(true, StepThreading::FreeRunning);
    CountingFirstStepLine firstStep(true);
    REQUIRE(WindowAllocationCounter::allocations > 0u);
    const uint64_t afterConstruction = WindowAllocationCounter::allocations;

    SECTION("16,385 samples through every door into every series, and a FreeRunning window line")
    {
        window.restart(0.0, TickDriverWindowTotals{});
        for (uint32_t index = 0u; index <= tickDriverWindow::kMaxSamplesPerSeries; ++index)
        {
            const double seconds = 0.000001 * static_cast<double>(index % 50u);
            window.noteFramePeriod(seconds);
            window.noteStartPhysics(1u, seconds);
            window.noteEndWait(seconds);
            window.notePostPhysics(seconds);
            window.noteWorldLockWait(seconds);
            window.noteStepCost(seconds, seconds, 0u, false);
            window.noteShadowRestore(seconds, false);
            window.noteLateness(seconds);
            window.noteApproach(seconds);
            window.noteActivityWake();
            window.noteEarlyWake();
            window.noteIdleFallback();
        }
        for (const Series series : kAllSeries)
            CHECK(window.samples(series).overflow() == 1u);
        for (const Series series : kSuffixSeries)
        {
            CHECK(window.samples(series).count() == tickDriverWindow::kMaxSamplesPerSeries);
            CHECK(window.samples(series).overflow() == 1u);
        }
        CHECK(window.sampleOverflow() == 10u);

        const std::string line = window.formatWindowLine(10.0, TickDriverWindowTotals{});
        CHECK(line.find(" sampleOverflow=10 lateUs=") != std::string::npos);
        CHECK(line.find(" activityWakes=16385 earlyWakes=16385 idleFallbacks=16385 approachUs=") != std::string::npos);
        CHECK(std::string(window.formatFrameLine(1.0 / 60.0, 60u, 0.75, 1.25, 0.0005)).find("idleSlackUs=500") != std::string::npos);
        CHECK(std::string(firstStep.format(StepThreading::FreeRunning, true, 0u, 1u)).find("thread=runner") != std::string::npos);
        CHECK(WindowAllocationCounter::allocations == afterConstruction);
    }

    SECTION("every series full, at the extremes, through every formatter, twice")
    {
        for (int pass = 0; pass < 2; ++pass)
        {
            window.restart(-DBL_MAX, totals(0u, 0.0, 0u, 0u, 0u));
            for (uint32_t index = 0u; index < tickDriverWindow::kSeriesCount; ++index)
            {
                const Series series = static_cast<Series>(index);
                for (uint32_t sample = 0u; sample <= tickDriverWindow::kMaxSamplesPerSeries; ++sample)
                    window.samples(series).add(sample % 2u == 0u ? -static_cast<double>(FLT_MAX) : static_cast<double>(FLT_MAX));
            }
            TickDriverWindowCounts& counts = window.counts();
            counts.frames           = std::numeric_limits<uint32_t>::max();
            counts.steps            = std::numeric_limits<uint64_t>::max();
            counts.maxStepsPerFrame = std::numeric_limits<uint32_t>::max();
            counts.endWaits         = std::numeric_limits<uint32_t>::max();
            counts.j2Waits          = std::numeric_limits<uint32_t>::max();
            counts.replays          = 1u;
            counts.replayedTicks    = std::numeric_limits<uint64_t>::max();
            counts.maxReplayDepth   = std::numeric_limits<uint32_t>::max();
            counts.resimRefused     = std::numeric_limits<uint32_t>::max();
            counts.shadowRefused    = std::numeric_limits<uint32_t>::max();
            counts.activityWakes    = std::numeric_limits<uint32_t>::max();
            counts.earlyWakes       = std::numeric_limits<uint32_t>::max();
            counts.idleFallbacks    = std::numeric_limits<uint32_t>::max();

            const TickDriverWindowTotals maxima = totals(std::numeric_limits<uint64_t>::max(), DBL_MAX,
                std::numeric_limits<uint64_t>::max(), std::numeric_limits<uint64_t>::max(), std::numeric_limits<uint64_t>::max());
            const std::string line = window.formatWindowLine(0.0, maxima);
            CHECK(line.size() <= tickDriverWindow::kWindowLineReserve);
            CHECK(line.find(" sampleOverflow=10 lateUs=") != std::string::npos);
            CHECK(line.find(" activityWakes=4294967295 earlyWakes=4294967295 idleFallbacks=4294967295 approachUs=") !=
                std::string::npos);

            const std::string frameLine = window.formatFrameLine(-DBL_MAX, std::numeric_limits<uint32_t>::max(), -DBL_MAX,
                -DBL_MAX, DBL_MAX);
            CHECK(frameLine.size() <= tickDriverWindow::kFrameLineReserve);
            CHECK(frameLine.find(" idleSlackUs=4294967295") != std::string::npos);
            CHECK(std::string(firstStep.format(StepThreading::FreeRunning, true, std::numeric_limits<uint64_t>::max(),
                std::numeric_limits<uint32_t>::max())).size() <= tickDriverWindow::kFirstStepLineReserve);
        }
        CHECK(WindowAllocationCounter::allocations == afterConstruction);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
