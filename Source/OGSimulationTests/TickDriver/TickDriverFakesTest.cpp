// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationScheduler.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGTickDriver/TickDriverConcepts.h"
#include "OGTickDriver/TickDriverLog.h"
#include "OGTickDriver/TickDriverThreadContext.h"
#include "TickDriverFakes.h"

namespace tickDriverFakesTest
{
    using namespace tickDriverFakes;

    struct VirtualIdleRig
    {
        Events events;
        FakeManager manager;
        FakeStorage storage;
        FakeEngine engine;
    };

    struct RealIdleRig
    {
        Events events;
        FakeManager manager;
        FakeStorage storage;
        RealClockFakeEngine engine;
    };

    bool hasFakesTestEvent(const Events& events, const std::string& event)
    {
        return std::find(events.begin(), events.end(), event) != events.end();
    }

    void buildFakeHostWith(FakeWorldHost& host, const FakeWorldHostConfig& config)
    {
        StaticWorldDescription queryable;
        StaticWorldDescription physicsOnly;
        std::optional<FakeAdapter> bodyAdapter;
        std::optional<FakeAdapter> queryAdapter;
        host.build(config, queryable, physicsOnly, bodyAdapter, queryAdapter);
    }
} // namespace tickDriverFakesTest

using namespace tickDriverFakesTest;

TEST_CASE("TickDriverFakes.HonourDeadline.LandsExactlyOnTheDeadline", "[TickDriver]")
{
    VirtualIdleRig rig;
    RecordingIdleGame game(rig.events, rig.manager, rig.storage, rig.engine, idleScript::HonourDeadline{});
    rig.engine.now = 0.25;

    CHECK(game.idleUntil(0.3) == IdleWake::Deadline);
    CHECK(rig.engine.now == 0.3);

    CHECK(game.idleUntil(0.1) == IdleWake::Deadline);
    CHECK(rig.engine.now == 0.3 + idleScript::kOnePassSeconds);

    REQUIRE(game.idleCalls.size() == 2u);
    CHECK(game.idleCalls[0].deadline == 0.3);
    CHECK(game.idleCalls[0].now == 0.25);
    CHECK(game.idleCalls[0].returnedAt == 0.3);
    CHECK(game.idleCalls[0].wake == IdleWake::Deadline);
    CHECK(game.idleCalls[1].deadline == 0.1);
    CHECK(game.idleCalls[1].now == 0.3);
    CHECK(rig.events == Events{ "idleUntil:Deadline", "idleUntil:Deadline" });
}

TEST_CASE("TickDriverFakes.Wakes.GiveActivityForAnEarlyScriptedTimeAndDeadlineOtherwise", "[TickDriver]")
{
    VirtualIdleRig rig;
    RecordingIdleGame game(rig.events, rig.manager, rig.storage, rig.engine, idleScript::Wakes{ { 0.010, 0.050, 0.060 } });

    CHECK(game.idleUntil(1.0 / 60.0) == IdleWake::Activity);
    CHECK(rig.engine.now == 0.010);
    CHECK(game.idleUntil(2.0 / 60.0) == IdleWake::Deadline);
    CHECK(rig.engine.now == 0.050);
    CHECK(game.idleUntil(0.060) == IdleWake::Deadline);
    CHECK(rig.engine.now == 0.060);
    CHECK(game.scriptExhaustedCalls == 0u);

    CHECK(game.idleUntil(0.070) == IdleWake::Deadline);
    CHECK(rig.engine.now == 0.070);
    CHECK(game.scriptExhaustedCalls == 1u);
    CHECK(rig.events == Events{ "idleUntil:Activity", "idleUntil:Deadline", "idleUntil:Deadline", "idleUntil:Deadline" });
}

TEST_CASE("TickDriverFakes.ActivityAt.StopsAtTheEarlierOfTheActivityAndTheDeadline", "[TickDriver]")
{
    VirtualIdleRig rig;
    RecordingIdleGame game(rig.events, rig.manager, rig.storage, rig.engine, idleScript::ActivityAt{ { 0.004, 0.030 } });

    CHECK(game.idleUntil(0.010) == IdleWake::Activity);
    CHECK(rig.engine.now == 0.004);
    CHECK(game.idleUntil(0.010) == IdleWake::Deadline);
    CHECK(rig.engine.now == 0.010);
    CHECK(game.idleUntil(0.020) == IdleWake::Deadline);
    CHECK(rig.engine.now == 0.020);
    CHECK(game.idleUntil(0.040) == IdleWake::Activity);
    CHECK(rig.engine.now == 0.030);
    CHECK(game.idleUntil(0.040) == IdleWake::Deadline);
    CHECK(rig.engine.now == 0.040);
}

TEST_CASE("TickDriverFakes.Unavailable.ReturnsAtOnceWithoutAdvancing", "[TickDriver]")
{
    VirtualIdleRig rig;
    RecordingIdleGame game(rig.events, rig.manager, rig.storage, rig.engine, idleScript::Unavailable{});
    rig.engine.now = 0.5;

    CHECK(game.idleUntil(1.0) == IdleWake::Unavailable);
    CHECK(rig.engine.now == 0.5);
    CHECK(game.idleUntil(0.25) == IdleWake::Unavailable);
    CHECK(rig.engine.now == 0.5);
    CHECK(rig.events == Events{ "idleUntil:Unavailable", "idleUntil:Unavailable" });
}

TEST_CASE("TickDriverFakes.EarlyDeadline.AdvancesOneMicrosecondAndReturnsDeadline", "[TickDriver]")
{
    VirtualIdleRig rig;
    RecordingIdleGame game(rig.events, rig.manager, rig.storage, rig.engine, idleScript::EarlyDeadline{});

    CHECK(game.idleUntil(1.0) == IdleWake::Deadline);
    CHECK(rig.engine.now == idleScript::kEarlyDeadlineSeconds);
    CHECK(rig.engine.now < 1.0);
}

TEST_CASE("TickDriverFakes.FakeRunnerHost.SleepUntilAdvancesTheEngineClock", "[TickDriver]")
{
    FakeEngine engine;
    Events events;
    FakeRunnerHost runner(engine, &events);
    engine.now = 0.1;

    runner.sleepUntil(0.2);
    CHECK(engine.now == 0.2);
    runner.sleepUntil(0.15);
    CHECK(engine.now == 0.2);
    CHECK(runner.sleeps == std::vector<double>{ 0.2, 0.15 });
    CHECK(events == Events{ "sleepUntil", "sleepUntil" });
}

TEST_CASE("TickDriverFakes.RealDeadline.NeverReturnsDeadlineBeforeTheDeadline", "[TickDriver]")
{
    RealIdleRig rig;
    RecordingIdleGame game(rig.events, rig.manager, rig.storage, idleScript::RealDeadline{});

    const std::vector<double> horizons{ 0.0001, 0.0005, 0.001, 0.0015, 0.0019, 0.003, 0.010 };
    uint32_t deadlines = 0u;
    uint32_t early = 0u;
    for (size_t i = 0u; i < 3u * horizons.size(); ++i)
    {
        const double deadline = rig.engine.nowSeconds() + horizons[i % horizons.size()];
        const IdleWake wake = game.idleUntil(deadline);
        const double after = rig.engine.nowSeconds();
        if (wake == IdleWake::Deadline)
            ++deadlines;
        if (after < deadline)
            ++early;
    }
    CHECK(deadlines == 3u * horizons.size());
    CHECK(early == 0u);

    CHECK(game.idleUntil(rig.engine.nowSeconds() - 1.0) == IdleWake::Deadline);
    REQUIRE(game.idleCalls.size() == 3u * horizons.size() + 1u);
    CHECK(std::all_of(game.idleCalls.begin(), game.idleCalls.end(),
        [](const IdleCall& call) { return call.returnedAt >= call.deadline; }));
}

TEST_CASE("TickDriverFakes.RealDeadline.AWakeFromAnotherThreadReturnsActivity", "[TickDriver]")
{
    RealIdleRig rig;
    RecordingIdleGame game(rig.events, rig.manager, rig.storage, idleScript::RealDeadline{});

    std::thread waker([&game] { game.wake(); });
    const IdleWake woken = game.idleUntil(rig.engine.nowSeconds() + 5.0);
    waker.join();
    CHECK(woken == IdleWake::Activity);

    const double deadline = rig.engine.nowSeconds() + 0.002;
    CHECK(game.idleUntil(deadline) == IdleWake::Deadline);
    CHECK(rig.engine.nowSeconds() >= deadline);
}

TEST_CASE("TickDriverFakes.RealSpin.ReturnsDeadlineAtOrAfterTheDeadline", "[TickDriver]")
{
    RealIdleRig rig;
    RecordingIdleGame game(rig.events, rig.manager, rig.storage, idleScript::RealSpin{});

    const double deadline = rig.engine.nowSeconds() + 0.001;
    CHECK(game.idleUntil(deadline) == IdleWake::Deadline);
    CHECK(rig.engine.nowSeconds() >= deadline);
}

TEST_CASE("TickDriverFakes.RealRunnerHost.SleepUntilReturnsAtOrAfterTheTime", "[TickDriver]")
{
    RealClockFakeEngine engine;
    RealRunnerHost runner;

    const double until = engine.nowSeconds() + 0.002;
    runner.sleepUntil(until);
    CHECK(engine.nowSeconds() >= until);
    CHECK(runner.sleepCalls.load() == 1u);
}

TEST_CASE("TickDriverFakes.RealClockFakeEngine.RunsDispatchedWorkInlineAndDefersNothing", "[TickDriver]")
{
    RealClockFakeEngine engine;
    int runs = 0;

    const RealClockFakeEngine::Handle handle = engine.dispatchAfter(RealClockFakeEngine::Handle{}, [&runs] { ++runs; });
    CHECK(runs == 1);
    REQUIRE(handle != nullptr);
    CHECK(handle->done);
    CHECK_FALSE(engine.isPending(handle));
    engine.waitFor(handle);
    CHECK(runs == 1);

    FakeCompletion completion;
    CHECK_FALSE(engine.deferUntil(completion, handle, [&runs] { ++runs; }));
    CHECK(runs == 1);
    CHECK(completion.continuations.empty());
    CHECK(engine.inlineDispatches.load() == 1u);

    CHECK(engine.frameNumber() == 0u);
    engine.advanceFrame();
    CHECK(engine.frameNumber() == 1u);

    const double first = engine.nowSeconds();
    CHECK(engine.nowSeconds() >= first);

    engine.minimumLevel = HostLogLevel::Log;
    CHECK_FALSE(engine.logEnabled(HostLogLevel::Verbose));
    CHECK(engine.logEnabled(HostLogLevel::Warning));
    std::thread runner([&engine] { engine.log(HostLogLevel::Log, "runner line"); });
    runner.join();
    const std::vector<LoggedLine> lines = engine.snapshotLines();
    REQUIRE(lines.size() == 1u);
    CHECK(lines[0].level == HostLogLevel::Log);
    CHECK(lines[0].text == "runner line");
}

TEST_CASE("TickDriverFakes.RecordingIdleGame.RunsTheBeforePumpHookAndSetsTheStopAfterNSteps", "[TickDriver]")
{
    VirtualIdleRig rig;
    RecordingIdleGame game(rig.events, rig.manager, rig.storage, rig.engine);
    SimulationScheduler scheduler(SchedulerConfig{ 1.0 / 60.0, 4u }, 0.0);
    uint64_t refused = 0u;
    TickRateDoor door(scheduler, 0, false, refused);

    int hookRuns = 0;
    game.beforePumpHook = [&rig, &hookRuns] {
        ++hookRuns;
        rig.events.push_back("hook");
    };
    std::atomic<bool> stop{ false };
    game.setStopAfterSteps(stop, 2u);

    game.beforePump(0.5, door);
    game.onStepEnded();
    CHECK_FALSE(stop.load());
    game.onStepEnded();
    CHECK(stop.load());

    CHECK(hookRuns == 1);
    CHECK(game.stepsEnded.load() == 2u);
    CHECK(game.beforePumpHostSeconds == std::vector<double>{ 0.5 });
    CHECK(rig.events == Events{ "beforePump", "hook", "onStepEnded", "onStepEnded" });
}

TEST_CASE("TickDriverFakes.FakeWorldHost.BuildConfiguresTheConfigsThreading", "[TickDriver]")
{
    Events events;

    SECTION("an authority without a threading derives Inline, as before")
    {
        FakeWorldHost host(events, kConfigureAtBuild);
        FakeWorldHostConfig config;
        config.isAuthority = true;
        buildFakeHostWith(host, config);
        CHECK(hasFakesTestEvent(events, "configure:authority:inline"));
        CHECK(host.threadContext().threading() == StepThreading::Inline);
        CHECK(host.threadContext().entryCheck() == nullptr);
    }

    SECTION("a client without a threading derives FrameBounded, as before")
    {
        FakeWorldHost host(events, kConfigureAtBuild);
        FakeWorldHostConfig config;
        buildFakeHostWith(host, config);
        CHECK(hasFakesTestEvent(events, "configure:client:worker"));
        CHECK(host.threadContext().threading() == StepThreading::FrameBounded);
    }

    SECTION("a client that runs inline derives Inline, as before")
    {
        FakeWorldHost host(events, kConfigureAtBuild);
        FakeWorldHostConfig config;
        config.runInline = true;
        buildFakeHostWith(host, config);
        CHECK(hasFakesTestEvent(events, "configure:client:inline"));
        CHECK(host.threadContext().threading() == StepThreading::Inline);
    }

    SECTION("a FreeRunning authority is configured on the building thread")
    {
        FakeWorldHost host(events, kConfigureAtBuild);
        FakeWorldHostConfig config;
        config.isAuthority = true;
        config.threading = StepThreading::FreeRunning;
        buildFakeHostWith(host, config);
        CHECK(hasFakesTestEvent(events, "configure:authority:runner"));
        CHECK(host.threadContext().threading() == StepThreading::FreeRunning);
        REQUIRE(host.threadContext().entryCheck() != nullptr);
        CHECK(host.threadContext().entryCheck()());
        CHECK(host.threadContext().isOwnerThreadForRole());
    }

    SECTION("the constructor takes a threading too")
    {
        FakeWorldHost host(events, tickDriverThread::kAuthorityRoleBit, StepThreading::FreeRunning);
        CHECK(host.threadContext().threading() == StepThreading::FreeRunning);
        CHECK(host.threadContext().entryCheck() != nullptr);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
