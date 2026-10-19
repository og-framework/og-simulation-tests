// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/OGAssert.h"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/ServerTickClock.h"
#include "OGSimulation/ResimGateProbe.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationStepDriver.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/TickDriverConcepts.h"
#include "OGSimulation/TickDriverStepHooks.h"
#include "OGSimulation/TickDriverThreadContext.h"
#include "TickDriverFakes.h"

namespace
{
    using namespace tickDriverFakes;
    using stepDriverTestSupport::tickEvent;
    using tickDriverThread::kAuthorityRoleBit;
    using tickDriverThread::kClientRoleBit;
    using AdvanceResult = ClientPredictionClock::AdvanceResult;

    constexpr float kDt = 1.f / 60.f;
    constexpr uint32_t kRingDepthTicks = 64u;

    class ScriptedPredictionClock
    {
    public:
        using ResyncCallback = ClientPredictionClock::ResyncCallback;

        void script(AdvanceResult result, unsigned int hardResyncTarget = 0u)
        {
            m_script.push_back({ result, hardResyncTarget });
        }

        AdvanceResult advancePrediction()
        {
            Scripted next{ AdvanceResult::Normal, 0u };
            if (!m_script.empty())
            {
                next = m_script.front();
                m_script.pop_front();
            }
            switch (next.result)
            {
            case AdvanceResult::Normal:
                normalAdvance();
                break;
            case AdvanceResult::Skip:
                if (m_predictionTick == m_resimulationTick)
                {
                    ++m_resimulationTick;
                }
                ++m_predictionTick;
                normalAdvance();
                break;
            case AdvanceResult::Stall:
                break;
            case AdvanceResult::HardResync:
                m_predictionTick = next.hardResyncTarget;
                m_resimulationTick = next.hardResyncTarget;
                for (auto& registered : m_callbacks)
                {
                    registered.second(next.hardResyncTarget);
                }
                break;
            }
            return next.result;
        }

        unsigned int getPredictionTick() const { return m_predictionTick; }
        bool isResimulating() const { return m_resimulationTick < m_predictionTick; }
        void startResimulation(unsigned int tick) { m_resimulationTick = tick; }
        unsigned int advanceResimulation() { return ++m_resimulationTick; }

        unsigned int registerResyncCallback(ResyncCallback callback)
        {
            const unsigned int id = m_nextId++;
            m_callbacks.emplace_back(id, std::move(callback));
            return id;
        }

        void unregisterResyncCallback(unsigned int id)
        {
            m_callbacks.erase(std::remove_if(m_callbacks.begin(), m_callbacks.end(),
                [id](const auto& registered) { return registered.first == id; }), m_callbacks.end());
        }

    private:
        struct Scripted
        {
            AdvanceResult result;
            unsigned int hardResyncTarget;
        };

        void normalAdvance()
        {
            if (m_predictionTick == m_resimulationTick)
            {
                ++m_resimulationTick;
            }
            ++m_predictionTick;
        }

        std::deque<Scripted> m_script;
        unsigned int m_predictionTick = 0u;
        unsigned int m_resimulationTick = 0u;
        std::vector<std::pair<unsigned int, ResyncCallback>> m_callbacks;
        unsigned int m_nextId = 0u;
    };

    class ScriptedManager
    {
    public:
        ScriptedManager(bool runsPrediction, Events& events)
            : m_runsPrediction(runsPrediction)
            , m_events(&events)
            , m_serverClock(kDt)
        {
        }

        bool runsPrediction() const { return m_runsPrediction; }
        ScriptedPredictionClock& editClientClock() { return m_clock; }
        const ScriptedPredictionClock& getClientClock() const { return m_clock; }
        const ServerTickClock& getServerClock() const { return m_serverClock; }
        ResimGateProbe& editResimGateProbe() { return m_probe; }

        void onGameSimulation(const SimulationUpdateInfo& info)
        {
            if (!m_runsPrediction)
            {
                m_serverClock.advanceTick();
                m_lastStep = SimulationTimeStep(m_serverClock.getTick(), false, StepKind::Normal, kDt);
                m_events->push_back("sim");
                return;
            }
            if (info.isResimulation())
            {
                m_lastStep = SimulationTimeStep(m_clock.advanceResimulation(), true, StepKind::Normal, kDt);
                m_events->push_back("simResim");
                return;
            }
            const AdvanceResult result = m_clock.advancePrediction();
            const StepKind kind = result == AdvanceResult::Stall ? StepKind::Stall
                                : result == AdvanceResult::Skip  ? StepKind::Skip
                                                                 : StepKind::Normal;
            m_lastStep = SimulationTimeStep(m_clock.getPredictionTick(), false, kind, kDt);
            m_events->push_back("sim");
        }

        void onPostGameSimulation(const SimulationUpdateInfo& info)
        {
            if (!info.isResimulation())
            {
                m_events->push_back("post");
                return;
            }
            m_events->push_back("postResim");
            if (!m_clock.isResimulating())
            {
                pendingAnchor.reset();
            }
        }

        unsigned int onCheckIsSimilar()
        {
            return pendingAnchor.has_value() ? *pendingAnchor : std::numeric_limits<unsigned int>::max();
        }

        void prepareResimulation(int32_t /*physicsStep*/, uint32_t simTick)
        {
            m_events->push_back(tickEvent("prepare", simTick));
            m_clock.startResimulation(simTick);
        }

        uint32_t currentIntegratedTick() const { return m_lastStep.has_value() ? m_lastStep->getTick() : 0u; }
        const std::optional<SimulationTimeStep>& lastIntegratedStep() const { return m_lastStep; }

        std::optional<unsigned int> pendingAnchor;

    private:
        bool m_runsPrediction;
        Events* m_events;
        ScriptedPredictionClock m_clock;
        ServerTickClock m_serverClock;
        ResimGateProbe m_probe;
        std::optional<SimulationTimeStep> m_lastStep;
    };

    class FakeDriverDoors
    {
    public:
        FakeDriverDoors(Events& events, const TickDriverThreadContext& context)
            : m_events(&events)
            , m_context(&context)
        {
        }

        void applyBodySlotOccupancyCommands_Step() { m_events->push_back("drainBodySlotOccupancy"); }

        void publishTickOffset_Step(int64_t offset)
        {
            m_events->push_back("publishTickOffset:" + std::to_string(offset));
            offsets.push_back(offset);
        }

        void pushStepCost_Step(const StepCostSample& sample)
        {
            m_events->push_back("pushStepCost");
            samples.push_back(sample);
        }

        bool isStepRunningOnThisThread() const
        {
            ++stepContextTests;
            return m_context->isStepRunningOnThisThread();
        }

        bool isOwnerThread() const
        {
            ++ownerThreadTests;
            return m_context->isOwnerThreadForRole();
        }

        std::vector<int64_t> offsets;
        std::vector<StepCostSample> samples;
        mutable uint32_t stepContextTests = 0u;
        mutable uint32_t ownerThreadTests = 0u;

    private:
        Events* m_events;
        const TickDriverThreadContext* m_context;
    };

    class EventClockEngine : public FakeEngine
    {
    public:
        explicit EventClockEngine(Events& events)
            : m_events(&events)
        {
        }

        double nowSeconds() const
        {
            m_events->push_back("clock");
            return static_cast<double>(m_events->size());
        }

    private:
        Events* m_events;
    };

    static_assert(DriverStepDoors<FakeDriverDoors>, "FakeDriverDoors must model DriverStepDoors");
    static_assert(TickDriverEngine<EventClockEngine>, "EventClockEngine must model TickDriverEngine");

    using Doors = StepDoors<FakeDriverDoors, RecordingGame, FakePresenter, FakeWorldHost, EventClockEngine>;
    using Hooks = TickDriverStepHooks<Doors>;
    using Driver = SimulationStepDriver<ScriptedManager, LoggedWorld, Hooks>;

    static_assert(StepHooks<Hooks>, "TickDriverStepHooks over the fake doors must model StepHooks");

    double deadlineFor(uint64_t physicsStep)
    {
        return 1000.0 + static_cast<double>(physicsStep) * 0.25;
    }

    struct Rig
    {
        explicit Rig(bool authority)
            : host(events, authority ? kAuthorityRoleBit : kClientRoleBit, /*stepsInline=*/authority, kRingDepthTicks)
            , manager(/*runsPrediction=*/!authority, events)
            , game(events, gameManager, gameStorage)
            , presenter(events)
            , driverDoors(events, host.threadContext())
            , engine(events)
            , hooks(Doors{ driverDoors, game, presenter, host, engine })
            , driver(manager, host.stepWorld(), hooks, StepDriverConfig{ kDt })
        {
        }

        TickOutcome tick()
        {
            ++physicsStep;
            hooks.currentStepDeadline = deadlineFor(physicsStep);
            const auto worldLock = host.lockForStep();
            const StepScope stepScope(host.threadContext());
            return driver.runTick(physicsStep);
        }

        void ticks(uint32_t count)
        {
            for (uint32_t index = 0u; index < count; ++index)
            {
                tick();
            }
        }

        SimTick present() const { return manager.getClientClock().getPredictionTick(); }

        Events events;
        FakeWorldHost host;
        ScriptedManager manager;
        FakeManager gameManager;
        FakeStorage gameStorage;
        RecordingGame game;
        FakePresenter presenter;
        FakeDriverDoors driverDoors;
        EventClockEngine engine;
        Hooks hooks;
        Driver driver;
        uint64_t physicsStep = 0u;
    };

    std::string offsetEvent(SimTick tick, uint64_t physicsStep)
    {
        return "publishTickOffset:" + std::to_string(static_cast<int64_t>(tick) - static_cast<int64_t>(physicsStep));
    }

    Events afterTickEvents(SimTick tick, uint64_t physicsStep)
    {
        return { "clock", "onStepEnded", tickEvent("publish", tick), tickEvent("hostAfterTick", tick),
                 offsetEvent(tick, physicsStep), "pushStepCost" };
    }

    Events clientResimTick(SimTick anchor, SimTick present, uint64_t physicsStep)
    {
        Events expected{ "clock", "drainBodySlotOccupancy", tickEvent("restore", anchor), "occ", tickEvent("prepare", anchor) };
        for (SimTick tick = anchor + 1u; tick <= present; ++tick)
        {
            expected.insert(expected.end(), { "occ", "simResim", "step", "postResim", tickEvent("save", tick) });
        }
        expected.insert(expected.end(),
            { "captureCorrection:" + std::to_string(physicsStep), "clock", "scratch", "sim", "onStepStarted", "step", "post",
              tickEvent("save", present + 1u) });
        const Events after = afterTickEvents(present + 1u, physicsStep);
        expected.insert(expected.end(), after.begin(), after.end());
        return expected;
    }

    Events authorityTick(SimTick upcomingTick, uint64_t physicsStep)
    {
        Events expected{ "clock", "drainBodySlotOccupancy", tickEvent("releaseInputs", upcomingTick),
                         "captureCorrection:" + std::to_string(physicsStep), "clock", "sim", "onStepStarted", "step", "post" };
        const Events after = afterTickEvents(upcomingTick, physicsStep);
        expected.insert(expected.end(), after.begin(), after.end());
        return expected;
    }

    bool isHookEvent(const std::string& event)
    {
        static const char* const kHookPrefixes[] = { "clock", "drainBodySlotOccupancy", "releaseInputs", "captureCorrection",
            "onStepStarted", "onStepEnded", "publish:", "hostAfterTick", "publishTickOffset", "pushStepCost" };
        return std::any_of(std::begin(kHookPrefixes), std::end(kHookPrefixes),
            [&event](const char* prefix) { return event.rfind(prefix, 0) == 0; });
    }

    std::vector<std::size_t> clockPositions(const Events& events)
    {
        std::vector<std::size_t> positions;
        for (std::size_t index = 0u; index < events.size(); ++index)
        {
            if (events[index] == "clock")
            {
                positions.push_back(index);
            }
        }
        return positions;
    }

    bool checksAreCompiled()
    {
        bool evaluated = false;
        OG_CHECK([&evaluated] { evaluated = true; return true; }(), "TickDriverStepHooksTest: the check probe");
        return evaluated;
    }
} // namespace

TEST_CASE("TickDriverStepHooks.CallOrder.AClientTickWithAResimRunsEachHookOnceAroundTheReplay", "[TickDriverHooks]")
{
    Rig rig(/*authority=*/false);
    rig.ticks(10u);
    REQUIRE(rig.present() == 10u);
    rig.manager.pendingAnchor = 6u;
    rig.events.clear();
    const uint32_t stepTestsBefore = rig.driverDoors.stepContextTests;

    const TickOutcome outcome = rig.tick();

    REQUIRE(outcome.replayedTicks == 4u);
    REQUIRE_FALSE(outcome.resimRefused);
    REQUIRE(rig.events == clientResimTick(6u, 10u, 11u));
    CHECK(rig.game.releasedTicks.empty());
    CHECK(rig.presenter.capturedSteps.back() == 11u);
    CHECK(rig.presenter.published.back().tick == 11u);
    CHECK(rig.presenter.publishedDeadlines.back() == deadlineFor(11u));
    CHECK(rig.host.afterTickOutcomes.back().replayedTicks == 4u);
    CHECK(rig.driverDoors.ownerThreadTests == 0u);
    if (checksAreCompiled())
    {
        CHECK(rig.driverDoors.stepContextTests - stepTestsBefore == 4u);
    }
}

TEST_CASE("TickDriverStepHooks.CallOrder.AnAuthorityTickReleasesBeforeTheCaptureAndPublishesInOrder", "[TickDriverHooks]")
{
    Rig rig(/*authority=*/true);
    rig.ticks(3u);
    rig.events.clear();
    const uint32_t stepTestsBefore = rig.driverDoors.stepContextTests;
    const uint32_t ownerTestsBefore = rig.driverDoors.ownerThreadTests;

    const TickOutcome outcome = rig.tick();

    REQUIRE(outcome.tick == 4u);
    REQUIRE(rig.events == authorityTick(4u, 4u));
    CHECK(rig.presenter.publishedDeadlines.back() == deadlineFor(4u));
    if (checksAreCompiled())
    {
        CHECK(rig.driverDoors.stepContextTests - stepTestsBefore == 4u);
        CHECK(rig.driverDoors.ownerThreadTests - ownerTestsBefore == 1u);
    }
}

TEST_CASE("TickDriverStepHooks.Authority.ReleasesExactlyServerTickPlusOneOncePerStep", "[TickDriverHooks]")
{
    Rig rig(/*authority=*/true);
    for (uint32_t step = 0u; step < 20u; ++step)
    {
        const SimTick serverTickBefore = rig.manager.getServerClock().getTick();
        const std::size_t releasedBefore = rig.game.releasedTicks.size();
        rig.events.clear();

        const TickOutcome outcome = rig.tick();

        REQUIRE(rig.game.releasedTicks.size() == releasedBefore + 1u);
        CHECK(rig.game.releasedTicks.back() == serverTickBefore + 1u);
        CHECK(rig.game.releasedTicks.back() == outcome.tick);
        CHECK(stepDriverTestSupport::countOf(rig.events, tickEvent("releaseInputs", serverTickBefore + 1u)) == 1u);
    }
    std::vector<SimTick> expected(20u);
    for (uint32_t index = 0u; index < 20u; ++index)
    {
        expected[index] = index + 1u;
    }
    CHECK(rig.game.releasedTicks == expected);
}

TEST_CASE("TickDriverStepHooks.TickOffset.IsTheTickMinusThePhysicsStepAcrossStallSkipAndHardResync", "[TickDriverHooks]")
{
    Rig rig(/*authority=*/false);
    std::vector<TickOutcome> outcomes;
    const auto run = [&rig, &outcomes]
    {
        outcomes.push_back(rig.tick());
        REQUIRE(rig.driverDoors.offsets.size() == outcomes.size());
        CHECK(rig.driverDoors.offsets.back()
            == static_cast<int64_t>(outcomes.back().tick) - static_cast<int64_t>(outcomes.back().physicsStep));
    };

    for (uint32_t index = 0u; index < 4u; ++index)
    {
        run();
    }
    rig.manager.editClientClock().script(AdvanceResult::Stall);
    run();
    CHECK(outcomes.back().kind == StepKind::Stall);
    rig.manager.editClientClock().script(AdvanceResult::Skip);
    run();
    CHECK(outcomes.back().kind == StepKind::Skip);
    rig.manager.editClientClock().script(AdvanceResult::HardResync, /*hardResyncTarget=*/40u);
    run();
    CHECK(outcomes.back().hardResync);
    run();

    CHECK(rig.driverDoors.offsets == std::vector<int64_t>{ 0, 0, 0, 0, -1, 0, 33, 33 });
    CHECK(rig.presenter.published.size() == outcomes.size());
    CHECK(rig.host.afterTickOutcomes.size() == outcomes.size());
    CHECK(rig.driverDoors.samples.size() == outcomes.size());
}

TEST_CASE("TickDriverStepHooks.Replay.NoHookFiresWhileTheStepDriverReplays", "[TickDriverHooks]")
{
    Rig rig(/*authority=*/false);
    rig.ticks(20u);
    rig.manager.pendingAnchor = 8u;
    rig.events.clear();

    const TickOutcome outcome = rig.tick();

    REQUIRE(outcome.replayedTicks == 12u);
    const auto restore = std::find(rig.events.begin(), rig.events.end(), tickEvent("restore", 8u));
    const auto capture = std::find(rig.events.begin(), rig.events.end(), "captureCorrection:21");
    REQUIRE(restore != rig.events.end());
    REQUIRE(capture != rig.events.end());
    REQUIRE(restore < capture);
    CHECK(std::none_of(restore, capture, isHookEvent));
    CHECK(stepDriverTestSupport::countOf(rig.events, "simResim") == 12u);
    CHECK(stepDriverTestSupport::countOf(rig.events, "onStepStarted") == 1u);
    CHECK(stepDriverTestSupport::countOf(rig.events, "onStepEnded") == 1u);
    CHECK(stepDriverTestSupport::countOf(rig.events, "pushStepCost") == 1u);
    CHECK(clockPositions(rig.events).size() == 3u);
}

TEST_CASE("TickDriverStepHooks.StepCost.SamplesTheThreeInstantsAndTheOutcome", "[TickDriverHooks]")
{
    Rig rig(/*authority=*/false);
    rig.ticks(70u);

    SECTION("a granted resim")
    {
        rig.manager.pendingAnchor = 65u;
        rig.events.clear();
        const TickOutcome outcome = rig.tick();
        const std::vector<std::size_t> clocks = clockPositions(rig.events);
        REQUIRE(clocks.size() == 3u);
        const StepCostSample& sample = rig.driverDoors.samples.back();
        CHECK(sample.preSeconds == static_cast<double>(clocks[1] - clocks[0]));
        CHECK(sample.stepSeconds == static_cast<double>(clocks[2] - clocks[1]));
        CHECK(sample.replayedTicks == outcome.replayedTicks);
        CHECK(sample.replayedTicks == 5u);
        CHECK_FALSE(sample.resimRefused);
        CHECK(rig.game.stepStartedSeconds.back() == static_cast<double>(clocks[1] + 1u));
    }

    SECTION("a refused resim")
    {
        rig.manager.pendingAnchor = 1u;
        rig.events.clear();
        const TickOutcome outcome = rig.tick();
        REQUIRE(outcome.resimRefused);
        const StepCostSample& sample = rig.driverDoors.samples.back();
        CHECK(sample.resimRefused);
        CHECK(sample.replayedTicks == 0u);
        const std::vector<std::size_t> clocks = clockPositions(rig.events);
        REQUIRE(clocks.size() == 3u);
        CHECK(sample.preSeconds == static_cast<double>(clocks[1] - clocks[0]));
        CHECK(sample.stepSeconds == static_cast<double>(clocks[2] - clocks[1]));
    }
}

#endif // WITH_LOW_LEVEL_TESTS
