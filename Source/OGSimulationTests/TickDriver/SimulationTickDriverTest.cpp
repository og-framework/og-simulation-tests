// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/ServerTickClock.h"
#include "OGSimulation/ResimGateProbe.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationTickDriver.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/TickDriverConcepts.h"
#include "OGSimulation/TickDriverThreadContext.h"
#include "LegacyFrameHostModel.h"
#include "TickDriverFakes.h"

namespace
{
    using namespace tickDriverFakes;
    using stepDriverTestSupport::tickEvent;
    using tickDriverThread::kAuthorityRoleBit;
    using tickDriverThread::kClientRoleBit;

    constexpr float kDt = 1.f / 60.f;
    constexpr uint32_t kRingDepthTicks = 64u;

    class PredictionClock
    {
    public:
        using ResyncCallback = ClientPredictionClock::ResyncCallback;

        void advancePrediction()
        {
            if (m_predictionTick == m_resimulationTick)
                ++m_resimulationTick;
            ++m_predictionTick;
        }

        unsigned int getPredictionTick() const { return m_predictionTick; }
        bool isResimulating() const { return m_resimulationTick < m_predictionTick; }
        void startResimulation(unsigned int tick) { m_resimulationTick = tick; }
        unsigned int advanceResimulation() { return ++m_resimulationTick; }

        unsigned int registerResyncCallback(ResyncCallback callback)
        {
            m_callbacks.emplace_back(m_nextId, std::move(callback));
            return m_nextId++;
        }

        void unregisterResyncCallback(unsigned int id)
        {
            m_callbacks.erase(std::remove_if(m_callbacks.begin(), m_callbacks.end(),
                [id](const auto& registered) { return registered.first == id; }), m_callbacks.end());
        }

    private:
        unsigned int m_predictionTick = 0u;
        unsigned int m_resimulationTick = 0u;
        std::vector<std::pair<unsigned int, ResyncCallback>> m_callbacks;
        unsigned int m_nextId = 0u;
    };

    class TestManager
    {
    public:
        TestManager(bool runsPrediction, Events& events)
            : m_runsPrediction(runsPrediction)
            , m_events(&events)
            , m_serverClock(kDt)
        {
        }

        bool runsPrediction() const { return m_runsPrediction; }
        PredictionClock& editClientClock() { return m_clock; }
        const PredictionClock& getClientClock() const { return m_clock; }
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
            m_clock.advancePrediction();
            m_lastStep = SimulationTimeStep(m_clock.getPredictionTick(), false, StepKind::Normal, kDt);
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
                pendingAnchor.reset();
        }

        unsigned int onCheckIsSimilar()
        {
            return pendingAnchor.has_value() ? *pendingAnchor : std::numeric_limits<unsigned int>::max();
        }

        void prepareResimulation(int32_t, uint32_t simTick)
        {
            m_events->push_back(tickEvent("prepare", simTick));
            m_clock.startResimulation(simTick);
        }

        uint32_t currentIntegratedTick() const { return m_lastStep.has_value() ? m_lastStep->getTick() : 0u; }
        const std::optional<SimulationTimeStep>& lastIntegratedStep() const { return m_lastStep; }
        unsigned int predictionTick() const { return m_clock.getPredictionTick(); }

        std::optional<unsigned int> pendingAnchor;

    private:
        bool m_runsPrediction;
        Events* m_events;
        PredictionClock m_clock;
        ServerTickClock m_serverClock;
        ResimGateProbe m_probe;
        std::optional<SimulationTimeStep> m_lastStep;
    };

    const char* levelTag(HostLogLevel level)
    {
        switch (level)
        {
        case HostLogLevel::Verbose:
            return "log:V:";
        case HostLogLevel::Log:
            return "log:L:";
        case HostLogLevel::Warning:
            return "log:W:";
        }
        return "log:?:";
    }

    class TraceEngine : public FakeEngine
    {
    public:
        explicit TraceEngine(Events& events)
            : m_events(&events)
        {
        }

        double nowSeconds() const
        {
            m_events->push_back("clock");
            m_readOffsetSeconds += 3.0e-6;
            return now + m_readOffsetSeconds;
        }

        void log(HostLogLevel level, const char* line)
        {
            m_events->push_back(std::string(levelTag(level)) + line);
            FakeEngine::log(level, line);
        }

        template <typename F>
        Handle dispatchAfter(const Handle& previous, F&& work)
        {
            const std::shared_ptr<uint64_t> id = std::make_shared<uint64_t>(0u);
            Events* const events = m_events;
            const Handle task = FakeEngine::dispatchAfter(previous,
                [events, id, body = std::function<void()>(std::forward<F>(work))]()
                {
                    events->push_back("task:" + std::to_string(*id));
                    body();
                });
            *id = task->id;
            m_events->push_back("dispatch:" + std::to_string(task->id) + "<-" + std::to_string(dispatchedAfter.back()));
            return task;
        }

        void waitFor(const Handle& handle)
        {
            m_events->push_back("wait:" + std::to_string(handle != nullptr ? handle->id : 0u));
            FakeEngine::waitFor(handle);
            if (runDeferredInWaits)
                runDeferred();
        }

        template <typename F>
        bool deferUntil(Completion& completion, const Handle& handle, F&& continuation)
        {
            Events* const events = m_events;
            const uint64_t after = handle != nullptr ? handle->id : 0u;
            const bool held = FakeEngine::deferUntil(completion, handle,
                [events, after, body = std::function<void()>(std::forward<F>(continuation))]()
                {
                    events->push_back("deferred:" + std::to_string(after));
                    body();
                });
            m_events->push_back("defer:" + std::to_string(after) + (held ? ":held" : ":refused"));
            return held;
        }

        void finishOpenFrame()
        {
            while (pendingDeferrals() > 0u)
            {
                runDeferred();
                if (pendingDeferrals() > 0u && !runNextTask())
                    break;
            }
        }

        bool runDeferredInWaits = false;

    private:
        Events* m_events;
        mutable double m_readOffsetSeconds = 0.0;
    };

    static_assert(TickDriverEngine<TraceEngine>, "TraceEngine must model TickDriverEngine");

    class TraceWorldHost : public FakeWorldHost
    {
    public:
        TraceWorldHost(Events& events, uint8_t roleBit, bool stepsInline, uint32_t ringDepthTicks)
            : FakeWorldHost(events, roleBit, stepsInline, ringDepthTicks)
        {
        }

        WorldLock lockForStep() { return lockWorld(); }
    };

    static_assert(TickDriverWorldHost<TraceWorldHost>, "TraceWorldHost must model TickDriverWorldHost");

    using Game = BasicRecordingGame<TestManager>;
    using Driver = SimulationTickDriver<Game, TraceWorldHost, FakePresenter, TraceEngine>;
    using Legacy = legacyFrameHost::LegacyFrameHostModel<Game, TraceWorldHost, FakePresenter, TraceEngine>;

    struct Setup
    {
        bool authority = true;
        bool stepsInline = true;
        int32_t maxCatchUpSteps = 60;
    };

    struct Side
    {
        Side(Events& events, const Setup& setup)
            : host(events, setup.authority ? kAuthorityRoleBit : kClientRoleBit, setup.stepsInline, kRingDepthTicks)
            , manager(!setup.authority, events)
            , game(events, manager, storage)
            , presenter(events)
        {
        }

        TraceWorldHost host;
        TestManager manager;
        FakeStorage storage;
        Game game;
        FakePresenter presenter;
    };

    class DriverPipeline
    {
    public:
        DriverPipeline(Side& side, TraceEngine& engine, const Setup& setup)
            : m_driver(side.game, side.host, side.presenter, engine,
                Driver::Config{ static_cast<double>(kDt), setup.authority,
                    setup.stepsInline ? StepThreading::Inline : StepThreading::FrameBounded, setup.maxCatchUpSteps })
        {
        }

        void startFrame(float deltaSeconds) { m_driver.startFrame(static_cast<double>(deltaSeconds)); }
        void endFrame(FakeCompletion* completion, int32_t blockMode) { m_driver.endFrame(completion, blockMode); }
        void stop() { m_driver.stop(); }
        void pushOccupancy(uint32_t slot, bool occupied) { m_driver.pushOccupancy(slot, occupied); }
        void noteWorldLockWait(double seconds) { m_driver.noteWorldLockWait(seconds); }
        void noteShadowRestore(double seconds, bool refused) { m_driver.noteShadowRestore(seconds, refused); }
        double hostNowSeconds() const { return m_driver.hostNowSeconds(); }
        double stepIntervalSeconds() { return m_driver.stepIntervalSeconds(); }
        LostTime lostTime() const { return m_driver.getDiagnostics().lostTime(); }
        uint64_t physicsStepCount() const { return m_driver.getDiagnostics().physicsStepCount(); }
        double rateScale() const { return m_driver.getDiagnostics().rateScale(); }

        Driver& driver() { return m_driver; }

    private:
        Driver m_driver;
    };

    class LegacyPipeline
    {
    public:
        LegacyPipeline(Side& side, TraceEngine& engine, const Setup& setup)
            : m_model(side.game, side.host, side.presenter, engine)
        {
            m_model.cvarSimMaxCatchUpSteps = setup.maxCatchUpSteps;
            m_model.startJoltStepping(kDt, setup.authority);
        }

        void startFrame(float deltaSeconds) { m_model.startSteps_GameThread(deltaSeconds); }

        void endFrame(FakeCompletion* completion, int32_t blockMode)
        {
            m_model.cvarSimBlockMode = blockMode;
            m_model.endSteps_GameThread(completion);
        }

        void stop()
        {
            m_model.unregisterTickFunctions();
            m_model.waitForOutstandingSteps();
        }

        void pushOccupancy(uint32_t slot, bool occupied) { m_model.pushOccupancy_GameThread(slot, occupied); }
        void noteWorldLockWait(double seconds) { m_model.noteWorldMutexWait_GameThread(seconds); }
        void noteShadowRestore(double seconds, bool refused) { m_model.noteShadowRestore_GameThread(seconds, refused); }
        double hostNowSeconds() const { return m_model.hostNowSeconds_GameThread(); }
        double stepIntervalSeconds() { return m_model.stepIntervalSeconds_GameThread(); }
        LostTime lostTime() const { return m_model.lostTime(); }
        uint64_t physicsStepCount() const { return m_model.physicsStepCount(); }
        double rateScale() const { return m_model.rateScale(); }

    private:
        Legacy m_model;
    };

    template <typename PipelineT>
    struct Run
    {
        explicit Run(const Setup& setup)
            : engine(events)
            , side(events, setup)
            , pipeline(side, engine, setup)
        {
        }

        Events events;
        TraceEngine engine;
        Side side;
        PipelineT pipeline;
    };

    template <typename PipelineT>
    struct TwoRoleRun
    {
        TwoRoleRun(const Setup& authoritySetup, const Setup& clientSetup)
            : engine(events)
            , authoritySide(events, authoritySetup)
            , clientSide(events, clientSetup)
            , authority(authoritySide, engine, authoritySetup)
            , client(clientSide, engine, clientSetup)
        {
        }

        Events events;
        TraceEngine engine;
        Side authoritySide;
        Side clientSide;
        PipelineT authority;
        PipelineT client;
    };

    class Lcg
    {
    public:
        explicit Lcg(uint32_t seed) : m_state(seed) {}

        uint32_t next()
        {
            m_state = m_state * 1664525u + 1013904223u;
            return m_state >> 8;
        }

        float unit() { return static_cast<float>(next() % 10000u) / 10000.f; }

    private:
        uint32_t m_state;
    };

    float offTheDeadlines(uint32_t frame, float deltaSeconds)
    {
        return frame == 0u ? deltaSeconds + 0.5f * kDt : deltaSeconds;
    }

    std::string firstDifference(const Events& driver, const Events& legacy)
    {
        const size_t common = std::min(driver.size(), legacy.size());
        for (size_t index = 0u; index < common; ++index)
        {
            if (driver[index] == legacy[index])
                continue;
            std::string report = "event " + std::to_string(index) + ": driver '" + driver[index] + "' vs transcription '"
                + legacy[index] + "'; before it:";
            for (size_t back = index >= 4u ? index - 4u : 0u; back < index; ++back)
                report += " '" + driver[back] + "'";
            return report;
        }
        if (driver.size() != legacy.size())
        {
            return "driver has " + std::to_string(driver.size()) + " events, transcription " + std::to_string(legacy.size())
                + "; first extra: '" + (driver.size() > legacy.size() ? driver[common] : legacy[common]) + "'";
        }
        return {};
    }

    size_t countPrefix(const Events& events, const std::string& prefix)
    {
        return static_cast<size_t>(std::count_if(events.begin(), events.end(),
            [&prefix](const std::string& event) { return event.rfind(prefix, 0) == 0; }));
    }

    std::vector<int32_t> stepsPerFrame(const Events& events)
    {
        std::vector<int32_t> steps;
        for (const std::string& event : events)
        {
            if (event == "beforePump")
            {
                steps.push_back(0);
            }
            else if (event.rfind("onFrameStepsDue:", 0) == 0 && !steps.empty())
            {
                steps.back() = std::stoi(event.substr(event.find(',') + 1u));
            }
        }
        return steps;
    }

    uint64_t expectedLostSteps(const std::vector<int32_t>& uncappedSteps, int32_t cap)
    {
        uint64_t lost = 0u;
        for (int32_t due : uncappedSteps)
            lost += due > cap ? static_cast<uint64_t>(due - cap) : 0u;
        return lost;
    }

    std::vector<int32_t> schedulerSteps(const std::vector<double>& hostSeconds, std::optional<uint32_t> rateFrame)
    {
        SimulationScheduler scheduler(SchedulerConfig{ static_cast<double>(kDt), MaxCatchUpSteps{ 60u }, 0.75, 1.25 }, 0.0);
        std::vector<int32_t> steps;
        for (size_t frame = 0u; frame < hostSeconds.size(); ++frame)
        {
            if (rateFrame.has_value() && frame == *rateFrame)
                scheduler.setRateScale(1.1);
            steps.push_back(static_cast<int32_t>(scheduler.pump(hostSeconds[frame])));
        }
        return steps;
    }

    size_t indexOf(const Events& events, const std::string& prefix)
    {
        const auto found = std::find_if(events.begin(), events.end(),
            [&prefix](const std::string& event) { return event.rfind(prefix, 0) == 0; });
        return static_cast<size_t>(found - events.begin());
    }

    void probeDoors(Events& events, double hostNowSeconds, double stepIntervalSeconds)
    {
        events.push_back("hostNow:" + std::to_string(hostNowSeconds) + " stepInterval:" + std::to_string(stepIntervalSeconds));
    }

    template <typename RunT>
    std::vector<float> authorityInlineSession(RunT& run, uint32_t frames, uint32_t hitchFrame, float hitchSeconds)
    {
        Lcg random(7u);
        std::vector<float> deltas;
        FakeCompletion completion;
        for (uint32_t frame = 0u; frame < frames; ++frame)
        {
            const float delta = frame == hitchFrame ? hitchSeconds : kDt * (0.3f + 1.4f * random.unit());
            deltas.push_back(delta);
            run.engine.advance(static_cast<double>(delta));
            if (frame % 50u == 7u)
                run.pipeline.pushOccupancy(frame % 4u, frame % 100u < 50u);
            if (frame % 97u == 3u)
                run.pipeline.noteWorldLockWait(0.0002 + 0.00001 * static_cast<double>(frame));
            run.pipeline.startFrame(delta);
            probeDoors(run.events, run.pipeline.hostNowSeconds(), run.pipeline.stepIntervalSeconds());
            run.pipeline.endFrame(&completion, 0);
        }
        run.events.push_back("stop");
        run.pipeline.stop();
        return deltas;
    }

    template <typename RunT>
    void clientFrameBoundedSession(RunT& run, uint32_t frames)
    {
        for (uint32_t frame = 0u; frame < frames; ++frame)
        {
            const float delta = offTheDeadlines(frame, kDt * (0.6f + 0.8f * static_cast<float>((frame * 7u) % 5u) / 4.f));
            run.engine.advance(static_cast<double>(delta));
            if (frame % 23u == 11u && run.side.manager.predictionTick() > 6u)
                run.side.manager.pendingAnchor = run.side.manager.predictionTick() - 3u;
            if (frame % 31u == 5u)
                run.pipeline.pushOccupancy(1u + frame % 3u, frame % 62u < 31u);
            if (frame % 13u == 2u)
                run.pipeline.noteShadowRestore(0.00005 * static_cast<double>(1u + frame % 4u), frame % 26u == 2u);

            run.pipeline.startFrame(delta);

            const int32_t blockMode = static_cast<int32_t>((frame / 8u) % 2u);
            const uint32_t kind = frame % 6u;
            if (kind == 0u)
                run.engine.runAllTasks();

            FakeCompletion completion;
            run.engine.deferralSupported = kind != 3u;
            run.pipeline.endFrame(kind == 2u ? nullptr : &completion, blockMode);
            if (kind == 4u)
                run.engine.runAllTasks();
            run.engine.finishOpenFrame();
            run.engine.deferralSupported = true;
        }
        run.events.push_back("stop");
        run.pipeline.stop();
        run.engine.runDeferred();
    }

    template <typename RunT>
    void hitchSession(RunT& run, float hitchSeconds)
    {
        FakeCompletion completion;
        for (uint32_t frame = 0u; frame < 60u; ++frame)
        {
            const float delta = offTheDeadlines(frame, frame == 30u ? hitchSeconds : kDt);
            run.engine.advance(frame == 59u ? 10.0 : static_cast<double>(delta));
            run.pipeline.startFrame(delta);
            run.pipeline.endFrame(&completion, 0);
        }
        run.events.push_back("stop");
        run.pipeline.stop();
    }

    template <typename RunT>
    void twoOutstandingTasksThenStop(RunT& run)
    {
        FakeCompletion first;
        FakeCompletion second;
        run.engine.advance(static_cast<double>(offTheDeadlines(0u, kDt)));
        run.pipeline.startFrame(offTheDeadlines(0u, kDt));
        run.pipeline.endFrame(&first, 0);
        run.engine.advance(static_cast<double>(kDt));
        run.pipeline.startFrame(kDt);
        run.pipeline.endFrame(&second, 0);
        run.events.push_back("stop");
        run.pipeline.stop();
        run.events.push_back("afterStop");
        run.engine.runDeferred();
        run.engine.runAllTasks();
    }

    template <typename RunT>
    void twoRoleSession(RunT& run, uint32_t frames)
    {
        Lcg random(11u);
        for (uint32_t frame = 0u; frame < frames; ++frame)
        {
            const float delta = kDt * (0.5f + random.unit());
            run.engine.advance(static_cast<double>(delta));
            FakeCompletion authorityCompletion;
            FakeCompletion clientCompletion;
            run.authority.startFrame(delta);
            run.client.startFrame(delta);
            if (frame % 3u == 0u)
                run.engine.runAllTasks();
            run.authority.endFrame(&authorityCompletion, 0);
            run.client.endFrame(&clientCompletion, static_cast<int32_t>(frame % 2u));
            run.engine.finishOpenFrame();
        }
        run.events.push_back("stop");
        run.authority.stop();
        run.client.stop();
    }

    template <typename RunT>
    void rateScaleSession(RunT& run, uint32_t frames, std::optional<uint32_t> rateFrame)
    {
        uint32_t frame = 0u;
        run.side.game.onBeforePump = [&frame, rateFrame](double, TickRateDoor& rate)
        {
            if (rateFrame.has_value() && frame == *rateFrame)
                rate.setRateScale(1.1);
        };
        FakeCompletion completion;
        for (; frame < frames; ++frame)
        {
            const float delta = offTheDeadlines(frame, kDt * (0.9f + 0.1f * static_cast<float>(frame % 3u)));
            run.engine.advance(static_cast<double>(delta));
            run.pipeline.startFrame(delta);
            probeDoors(run.events, run.pipeline.hostNowSeconds(), run.pipeline.stepIntervalSeconds());
            run.pipeline.endFrame(&completion, 0);
        }
        run.events.push_back("stop");
        run.pipeline.stop();
        run.side.game.onBeforePump = nullptr;
    }
} // namespace

TEST_CASE("SimulationTickDriver.Inline.AnAuthoritySessionMatchesTheFrameHostTranscription", "[TickDriver]")
{
    const Setup setup{ true, true, 60 };
    Run<DriverPipeline> driver(setup);
    Run<LegacyPipeline> legacy(setup);
    const std::vector<float> deltas = authorityInlineSession(driver, 600u, 300u, 0.25f);
    authorityInlineSession(legacy, 600u, 300u, 0.25f);

    CHECK(firstDifference(driver.events, legacy.events) == "");

    const std::vector<int32_t> steps = stepsPerFrame(driver.events);
    REQUIRE(steps.size() == 600u);
    CHECK(steps[300] == 15);
    CHECK(std::count(steps.begin(), steps.end(), 0) > 0);
    CHECK(std::count(steps.begin(), steps.end(), 1) > 0);
    CHECK(std::count(steps.begin(), steps.end(), 2) > 0);
    CHECK(driver.pipeline.lostTime().steps == 0u);
    CHECK(legacy.pipeline.lostTime().steps == 0u);

    const size_t framesWithSteps = static_cast<size_t>(std::count_if(steps.begin(), steps.end(), [](int32_t n) { return n > 0; }));
    CHECK(countPrefix(driver.events, "onFrameStepsDue:") == framesWithSteps);
    CHECK(countPrefix(driver.events, "afterInlineBatch:") == framesWithSteps);
    CHECK(countPrefix(driver.events, "releaseInputs:") == driver.pipeline.physicsStepCount());
    CHECK(countPrefix(driver.events, "onPostPhysics") == 600u);
    CHECK(countPrefix(driver.events, "beforePump") == 600u);
    CHECK(countPrefix(driver.events, "wait:") == 0u);
    CHECK(countPrefix(driver.events, "defer:") == 0u);
    CHECK(countPrefix(driver.events, "dispatch:") == 0u);
    CHECK(countPrefix(driver.events, "log:W:[SimHost.Frame] role=Authority stepping=inline") == 1u);
    CHECK(countPrefix(driver.events, "log:W:[SimHost.FirstStep] role=Authority thread=game") == 1u);
    CHECK(countPrefix(driver.events, "log:W:[SimHost.Window] role=Authority stepping=inline") == 1u);

    double hostSeconds = 0.0;
    for (float delta : deltas)
        hostSeconds += static_cast<double>(delta);
    CHECK(driver.side.game.beforePumpHostSeconds.back() == hostSeconds);

    const TickOutcome& last = driver.side.presenter.published.back();
    CHECK(driver.pipeline.driver().tickOffset() == static_cast<int64_t>(last.tick) - static_cast<int64_t>(last.physicsStep));
}

TEST_CASE("SimulationTickDriver.Inline.AClientSteppingInlineMatchesTheFrameHostTranscription", "[TickDriver]")
{
    const Setup setup{ false, true, 60 };
    Run<DriverPipeline> driver(setup);
    Run<LegacyPipeline> legacy(setup);
    authorityInlineSession(driver, 240u, 100u, 0.1f);
    authorityInlineSession(legacy, 240u, 100u, 0.1f);

    CHECK(firstDifference(driver.events, legacy.events) == "");
    CHECK(countPrefix(driver.events, "releaseInputs:") == 0u);
    CHECK(countPrefix(driver.events, "afterInlineBatch:") == countPrefix(driver.events, "onFrameStepsDue:"));
    CHECK(countPrefix(driver.events, "log:W:[SimHost.Frame] role=Client stepping=inline") == 1u);
}

TEST_CASE("SimulationTickDriver.FrameBounded.AClientSessionMatchesTheFrameHostTranscription", "[TickDriver]")
{
    const Setup setup{ false, false, 60 };
    Run<DriverPipeline> driver(setup);
    Run<LegacyPipeline> legacy(setup);
    clientFrameBoundedSession(driver, 240u);
    clientFrameBoundedSession(legacy, 240u);

    CHECK(firstDifference(driver.events, legacy.events) == "");

    CHECK(countPrefix(driver.events, "afterInlineBatch:") == 0u);
    CHECK(countPrefix(driver.events, "dispatch:") == countPrefix(driver.events, "onFrameStepsDue:"));
    CHECK(countPrefix(driver.events, "dispatch:") > 100u);
    CHECK(std::count_if(driver.events.begin(), driver.events.end(),
        [](const std::string& event) { return event.rfind("dispatch:", 0) == 0 && event.find("<-0") == std::string::npos; }) > 0);
    CHECK(countPrefix(driver.events, "defer:") > 0u);
    CHECK(std::count_if(driver.events.begin(), driver.events.end(),
        [](const std::string& event) { return event.rfind("defer:", 0) == 0 && event.find(":held") != std::string::npos; }) > 0);
    CHECK(std::count_if(driver.events.begin(), driver.events.end(),
        [](const std::string& event) { return event.rfind("defer:", 0) == 0 && event.find(":refused") != std::string::npos; }) > 0);
    CHECK(countPrefix(driver.events, "wait:") > 0u);
    CHECK(countPrefix(driver.events, "deferred:") > 0u);
    CHECK(countPrefix(driver.events, "onPostPhysics") == 240u);
    CHECK(countPrefix(driver.events, "simResim") > 0u);
    CHECK(countPrefix(driver.events, "log:W:[SimHost.Frame] role=Client stepping=worker") == 1u);
    CHECK(countPrefix(driver.events, "log:W:[SimHost.FirstStep] role=Client") == 1u);
    CHECK(driver.engine.queuedTasks() == 0u);
    CHECK(driver.engine.pendingDeferrals() == 0u);
}

TEST_CASE("SimulationTickDriver.CatchUpCap.AHitchBeyondACapOfThreeLosesStepsAsTheTranscriptionDoes", "[TickDriver]")
{
    const Setup setup{ true, true, 3 };
    Run<DriverPipeline> uncapped(Setup{ true, true, 240 });
    Run<DriverPipeline> driver(setup);
    Run<LegacyPipeline> legacy(setup);
    hitchSession(uncapped, 0.25f);
    hitchSession(driver, 0.25f);
    hitchSession(legacy, 0.25f);

    CHECK(firstDifference(driver.events, legacy.events) == "");

    const std::vector<int32_t> due = stepsPerFrame(uncapped.events);
    REQUIRE(due.size() == 60u);
    REQUIRE(due[30] > 3);
    CHECK(stepsPerFrame(driver.events)[30] == 3);
    const uint64_t lost = expectedLostSteps(due, 3);
    CHECK(lost >= static_cast<uint64_t>(due[30] - 3));
    CHECK(driver.pipeline.lostTime().steps == lost);
    CHECK(legacy.pipeline.lostTime().steps == lost);
    CHECK(uncapped.pipeline.lostTime().steps == 0u);
    CHECK(countPrefix(driver.events, "log:W:[SimHost.Window] role=Authority stepping=inline") == 1u);
    const std::string lostField = " lostSteps=" + std::to_string(lost) + " ";
    CHECK(std::count_if(driver.events.begin(), driver.events.end(),
        [&lostField](const std::string& event)
        { return event.find("[SimHost.Window]") != std::string::npos && event.find(lostField) != std::string::npos; }) == 1);
    CHECK(countPrefix(driver.events, "log:W:[SimHost.Frame] og.Sim.MaxCatchUpSteps") == 0u);
}

TEST_CASE("SimulationTickDriver.CatchUpCap.ZeroAndNineHundredNinetyNineAreClampedWithTheWarning", "[TickDriver]")
{
    SECTION("a cap of 0 is clamped to 1")
    {
        const Setup setup{ true, true, 0 };
        Run<DriverPipeline> uncapped(Setup{ true, true, 240 });
        Run<DriverPipeline> driver(setup);
        Run<LegacyPipeline> legacy(setup);
        hitchSession(uncapped, 0.25f);
        hitchSession(driver, 0.25f);
        hitchSession(legacy, 0.25f);

        CHECK(firstDifference(driver.events, legacy.events) == "");
        const size_t warning = indexOf(driver.events, "log:");
        REQUIRE(warning < driver.events.size());
        CHECK(driver.events[warning] == "log:W:[SimHost.Frame] og.Sim.MaxCatchUpSteps=0 is outside [1, 240]; using 1");
        const size_t frameLine = indexOf(driver.events,
            "log:W:[SimHost.Frame] role=Authority stepping=inline dt=0.016667 maxCatchUpSteps=1 rateScale=[0.75,1.25] timeBase=worldDelta");
        CHECK(frameLine > warning);
        CHECK(frameLine < driver.events.size());
        CHECK(stepsPerFrame(driver.events)[30] == 1);
        CHECK(driver.pipeline.lostTime().steps == expectedLostSteps(stepsPerFrame(uncapped.events), 1));
    }
    SECTION("a cap of 999 is clamped to 240")
    {
        const Setup setup{ true, true, 999 };
        Run<DriverPipeline> reference(Setup{ true, true, 240 });
        Run<DriverPipeline> driver(setup);
        Run<LegacyPipeline> legacy(setup);
        hitchSession(reference, 5.0f);
        hitchSession(driver, 5.0f);
        hitchSession(legacy, 5.0f);

        CHECK(firstDifference(driver.events, legacy.events) == "");
        const size_t warning = indexOf(driver.events, "log:");
        REQUIRE(warning < driver.events.size());
        CHECK(driver.events[warning] == "log:W:[SimHost.Frame] og.Sim.MaxCatchUpSteps=999 is outside [1, 240]; using 240");
        const size_t frameLine = indexOf(driver.events,
            "log:W:[SimHost.Frame] role=Authority stepping=inline dt=0.016667 maxCatchUpSteps=240 rateScale=[0.75,1.25] timeBase=worldDelta");
        CHECK(frameLine > warning);
        CHECK(frameLine < driver.events.size());
        CHECK(stepsPerFrame(driver.events)[30] == 240);
        CHECK(driver.pipeline.lostTime().steps > 0u);
        CHECK(driver.pipeline.lostTime().steps == reference.pipeline.lostTime().steps);
        CHECK(countPrefix(reference.events, "log:W:[SimHost.Frame] og.Sim.MaxCatchUpSteps") == 0u);
    }
}

TEST_CASE("SimulationTickDriver.Stop.WaitsForBothOutstandingTasksAndNothingRunsAfterwards", "[TickDriver]")
{
    const Setup setup{ false, false, 60 };
    Run<DriverPipeline> driver(setup);
    Run<LegacyPipeline> legacy(setup);
    bool continuationInsideTheWait = false;
    SECTION("the executor runs the held continuation after stop returns")
    {
    }
    SECTION("the executor runs the held continuation inside stop's wait")
    {
        continuationInsideTheWait = true;
    }
    driver.engine.runDeferredInWaits = continuationInsideTheWait;
    legacy.engine.runDeferredInWaits = continuationInsideTheWait;
    twoOutstandingTasksThenStop(driver);
    twoOutstandingTasksThenStop(legacy);

    CHECK(firstDifference(driver.events, legacy.events) == "");

    const auto stopAt = std::find(driver.events.begin(), driver.events.end(), "stop");
    const auto afterStop = std::find(driver.events.begin(), driver.events.end(), "afterStop");
    REQUIRE(stopAt != driver.events.end());
    REQUIRE(afterStop != driver.events.end());
    CHECK(std::count(driver.events.begin(), stopAt, std::string("dispatch:1<-0")) == 1);
    CHECK(std::count(driver.events.begin(), stopAt, std::string("dispatch:2<-1")) == 1);
    CHECK(std::count(driver.events.begin(), stopAt, std::string("defer:1:held")) == 1);
    CHECK(std::find(stopAt, afterStop, std::string("wait:2")) != afterStop);
    CHECK(std::find(stopAt, afterStop, std::string("wait:1")) != afterStop);
    CHECK(std::find(stopAt, afterStop, std::string("task:1")) != afterStop);
    CHECK(std::find(stopAt, afterStop, std::string("task:2")) != afterStop);
    const Events tail(afterStop + 1, driver.events.end());
    CHECK(tail == (continuationInsideTheWait ? Events{} : Events{ "deferred:1" }));
    CHECK(std::count(stopAt, driver.events.end(), std::string("deferred:1")) == 1);
    CHECK(std::find(stopAt, driver.events.end(), std::string("onPostPhysics")) == driver.events.end());
    CHECK(driver.side.game.postPhysicsCalls == 1u);
    CHECK(driver.engine.queuedTasks() == 0u);
    CHECK_FALSE(driver.pipeline.driver().getDiagnostics().isRunning());
    CHECK_FALSE(driver.pipeline.driver().getDiagnostics().hasPendingSteps());
}

TEST_CASE("SimulationTickDriver.TwoDrivers.AnAuthorityAndAClientInOneProcessMatchTheTranscription", "[TickDriver]")
{
    const Setup authoritySetup{ true, true, 60 };
    const Setup clientSetup{ false, false, 60 };
    TwoRoleRun<DriverPipeline> driver(authoritySetup, clientSetup);
    TwoRoleRun<LegacyPipeline> legacy(authoritySetup, clientSetup);
    twoRoleSession(driver, 180u);
    twoRoleSession(legacy, 180u);

    CHECK(firstDifference(driver.events, legacy.events) == "");
    CHECK(countPrefix(driver.events, "log:W:[SimHost.Frame] role=Authority stepping=inline") == 1u);
    CHECK(countPrefix(driver.events, "log:W:[SimHost.Frame] role=Client stepping=worker") == 1u);
    CHECK(countPrefix(driver.events, "releaseInputs:") == driver.authority.physicsStepCount());
    CHECK(countPrefix(driver.events, "dispatch:") > 0u);
    CHECK(countPrefix(driver.events, "afterInlineBatch:") > 0u);
    CHECK(driver.authoritySide.game.postPhysicsCalls == 180u);
    CHECK(driver.clientSide.game.postPhysicsCalls == 180u);
    CHECK(driver.authoritySide.host.threadContext().roleBit() == kAuthorityRoleBit);
    CHECK(driver.clientSide.host.threadContext().roleBit() == kClientRoleBit);
    CHECK_FALSE(driver.authoritySide.host.threadContext().isStepRunningOnThisThread());
    CHECK_FALSE(driver.clientSide.host.threadContext().isStepRunningOnThisThread());
}

TEST_CASE("SimulationTickDriver.BeforePump.RunsOncePerFrameBeforeThePumpWithTheHostTime", "[TickDriver]")
{
    const Setup setup{ true, true, 60 };
    Run<DriverPipeline> driver(setup);
    Run<LegacyPipeline> legacy(setup);
    const std::vector<float> deltas = authorityInlineSession(driver, 120u, 60u, 0.05f);
    authorityInlineSession(legacy, 120u, 60u, 0.05f);

    CHECK(firstDifference(driver.events, legacy.events) == "");

    REQUIRE(driver.side.game.beforePumpHostSeconds.size() == 120u);
    double hostSeconds = 0.0;
    for (size_t frame = 0u; frame < deltas.size(); ++frame)
    {
        hostSeconds += static_cast<double>(deltas[frame]);
        CHECK(driver.side.game.beforePumpHostSeconds[frame] == hostSeconds);
    }

    size_t pumps = 0u;
    bool stepsSincePump = false;
    for (const std::string& event : driver.events)
    {
        if (event == "beforePump")
        {
            ++pumps;
            stepsSincePump = false;
        }
        else if (event.rfind("onFrameStepsDue:", 0) == 0)
        {
            CHECK_FALSE(stepsSincePump);
            stepsSincePump = true;
        }
        else if (event == "onPostPhysics")
        {
            CHECK(pumps > 0u);
        }
    }
    CHECK(pumps == 120u);
}

TEST_CASE("SimulationTickDriver.BeforePump.ARateScaleThroughTheDoorChangesTheStepCountsAsTheTranscriptionDoes", "[TickDriver]")
{
    const Setup setup{ true, true, 60 };
    Run<DriverPipeline> baseline(setup);
    Run<DriverPipeline> driver(setup);
    Run<LegacyPipeline> legacy(setup);
    rateScaleSession(baseline, 120u, std::nullopt);
    rateScaleSession(driver, 120u, 40u);
    rateScaleSession(legacy, 120u, 40u);

    CHECK(firstDifference(driver.events, legacy.events) == "");
    CHECK(driver.pipeline.rateScale() == 1.1);
    CHECK(legacy.pipeline.rateScale() == 1.1);
    CHECK(baseline.pipeline.rateScale() == 1.0);

    const std::vector<int32_t> rated = stepsPerFrame(driver.events);
    const std::vector<int32_t> unrated = stepsPerFrame(baseline.events);
    REQUIRE(rated.size() == 120u);
    REQUIRE(unrated.size() == 120u);
    const std::vector<double>& hostSeconds = driver.side.game.beforePumpHostSeconds;
    REQUIRE(hostSeconds == baseline.side.game.beforePumpHostSeconds);
    CHECK(rated == schedulerSteps(hostSeconds, 40u));
    CHECK(unrated == schedulerSteps(hostSeconds, std::nullopt));
    CHECK(rated != schedulerSteps(hostSeconds, 41u));
    const auto firstChange = std::mismatch(rated.begin(), rated.end(), unrated.begin());
    REQUIRE(firstChange.first != rated.end());
    CHECK(firstChange.first - rated.begin() >= 40);
    CHECK(driver.pipeline.physicsStepCount() > baseline.pipeline.physicsStepCount());
}

namespace
{
    class DoorReadingPresenter : public FakePresenter
    {
    public:
        explicit DoorReadingPresenter(Events& events)
            : FakePresenter(events)
            , m_events(&events)
        {
        }

        void captureCorrection_Step(uint64_t physicsStep)
        {
            FakePresenter::captureCorrection_Step(physicsStep);
            grantedAtCapture.push_back(readGrantedResims());
            preparesAtCapture.push_back(countPrefix(*m_events, "prepare:"));
        }

        void publish_Step(const TickOutcome& outcome, double stepDeadlineSeconds)
        {
            FakePresenter::publish_Step(outcome, stepDeadlineSeconds);
            savesTicksAtPublish.push_back(readSavesTicks());
        }

        std::function<bool()>     readSavesTicks;
        std::function<uint64_t()> readGrantedResims;
        std::vector<uint64_t>     grantedAtCapture;
        std::vector<size_t>       preparesAtCapture;
        std::vector<bool>         savesTicksAtPublish;

    private:
        Events* m_events;
    };

    static_assert(TickDriverPresenterSteps<DoorReadingPresenter>, "DoorReadingPresenter must model TickDriverPresenterSteps");

    using DoorDriver = SimulationTickDriver<Game, TraceWorldHost, DoorReadingPresenter, TraceEngine>;

    struct DoorRun
    {
        explicit DoorRun(bool authority)
            : engine(events)
            , side(events, Setup{ authority, true, 60 })
            , presenter(events)
            , driver(side.game, side.host, presenter, engine,
                DoorDriver::Config{ static_cast<double>(kDt), authority, StepThreading::Inline, 60 })
        {
            presenter.readSavesTicks    = [this]() { return driver.savesTicks(); };
            presenter.readGrantedResims = [this]() { return driver.grantedResims(); };
        }

        Events               events;
        TraceEngine          engine;
        Side                 side;
        DoorReadingPresenter presenter;
        DoorDriver           driver;
    };
} // namespace

TEST_CASE("SimulationTickDriver.Doors.SavesTicksAndGrantedResimsEqualTheStepDriversAfterAGrantedResim", "[TickDriver]")
{
    const bool authority = GENERATE(false, true);
    DoorRun run(authority);

    FakeCompletion completion;
    for (uint32_t frame = 0u; frame < 120u; ++frame)
    {
        const float delta = offTheDeadlines(frame, kDt);
        run.engine.advance(static_cast<double>(delta));
        if (!authority && frame % 23u == 11u && run.side.manager.predictionTick() > 6u)
            run.side.manager.pendingAnchor = run.side.manager.predictionTick() - 3u;
        run.driver.startFrame(static_cast<double>(delta));
        run.driver.endFrame(&completion, 0);

        CHECK(run.driver.savesTicks() == run.side.manager.runsPrediction());
        CHECK(run.driver.grantedResims() == countPrefix(run.events, "prepare:"));
    }
    run.events.push_back("stop");
    run.driver.stop();

    CHECK(run.driver.savesTicks() == !authority);
    REQUIRE(run.presenter.grantedAtCapture.size() == run.presenter.capturedSteps.size());
    REQUIRE(run.presenter.grantedAtCapture.size() == run.presenter.preparesAtCapture.size());
    REQUIRE(run.presenter.capturedSteps.size() > 100u);
    for (size_t step = 0u; step < run.presenter.grantedAtCapture.size(); ++step)
        CHECK(run.presenter.grantedAtCapture[step] == run.presenter.preparesAtCapture[step]);
    CHECK(std::all_of(run.presenter.savesTicksAtPublish.begin(), run.presenter.savesTicksAtPublish.end(),
        [authority](bool savesTicks) { return savesTicks == !authority; }));
    if (authority)
    {
        CHECK(run.driver.grantedResims() == 0u);
        CHECK(countPrefix(run.events, "simResim") == 0u);
    }
    else
    {
        CHECK(run.driver.grantedResims() >= 3u);
        CHECK(countPrefix(run.events, "simResim") > 0u);
        CHECK(std::adjacent_find(run.presenter.grantedAtCapture.begin(), run.presenter.grantedAtCapture.end(),
            [](uint64_t before, uint64_t after) { return after > before; }) != run.presenter.grantedAtCapture.end());
    }
}

#endif // WITH_LOW_LEVEL_TESTS
