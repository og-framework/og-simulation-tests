// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
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
        void pushBodySlotOccupancy(uint32_t slot, bool occupied) { m_driver.pushBodySlotOccupancy(slot, occupied); }
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

        void pushBodySlotOccupancy(uint32_t slot, bool occupied) { m_model.pushBodySlotOccupancy_GameThread(slot, occupied); }
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
                run.pipeline.pushBodySlotOccupancy(frame % 4u, frame % 100u < 50u);
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
                run.pipeline.pushBodySlotOccupancy(1u + frame % 3u, frame % 62u < 31u);
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
    void doorSession(RunT& run, uint32_t frames, std::function<void(uint32_t, TickRateDoor&)> onDoor)
    {
        uint32_t frame = 0u;
        run.side.game.onBeforePump = [&frame, &onDoor](double, TickRateDoor& rate) { onDoor(frame, rate); };
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

    template <typename RunT>
    void rateScaleSession(RunT& run, uint32_t frames, std::optional<uint32_t> rateFrame, double scale = 1.1)
    {
        doorSession(run, frames, [rateFrame, scale](uint32_t frame, TickRateDoor& rate)
        {
            if (rateFrame.has_value() && frame == *rateFrame)
                rate.setRateScale(scale);
        });
    }

    struct DoorReading
    {
        double nextDeadline = 0.0;
        double currentInterval = 0.0;
        int64_t tickOffset = 0;
        uint64_t refusedRateChanges = 0u;
    };

    template <typename RunT>
    std::vector<DoorReading> readingDoorSession(RunT& run, uint32_t frames, uint32_t rateFrame, double scale,
        std::optional<uint64_t>& refusedAfterTheCall)
    {
        std::vector<DoorReading> readings;
        doorSession(run, frames, [&readings, &refusedAfterTheCall, rateFrame, scale](uint32_t frame, TickRateDoor& rate)
        {
            readings.push_back(DoorReading{ rate.nextDeadline(), rate.currentInterval(), rate.tickOffset(),
                rate.refusedRateChanges() });
            if (frame != rateFrame)
                return;
            rate.setRateScale(scale);
            refusedAfterTheCall = rate.refusedRateChanges();
        });
        return readings;
    }

    std::vector<DoorReading> referenceDoorReadings(const std::vector<double>& hostSeconds, uint32_t rateFrame, double scale)
    {
        SimulationScheduler scheduler(SchedulerConfig{ static_cast<double>(kDt), MaxCatchUpSteps{ 60u }, 0.75, 1.25 }, 0.0);
        std::vector<DoorReading> readings;
        for (size_t frame = 0u; frame < hostSeconds.size(); ++frame)
        {
            readings.push_back(DoorReading{ scheduler.nextDeadline(), scheduler.currentStepInterval(), 0, 0u });
            if (frame == rateFrame)
                scheduler.setRateScale(scale);
            scheduler.pump(hostSeconds[frame]);
        }
        return readings;
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
    const Setup setup{ false, true, 60 };
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
    CHECK(driver.pipeline.driver().getDiagnostics().refusedRateChanges() == 0u);
}

TEST_CASE("SimulationTickDriver.BeforePump.TheDoorReadsTheSchedulerAndTheOffsetTheStepsPublished", "[TickDriver]")
{
    constexpr uint32_t kRateFrame = 40u;
    const Setup setup{ false, true, 60 };
    Run<DriverPipeline> driver(setup);
    driver.side.manager.editClientClock().advancePrediction();
    std::optional<uint64_t> refusedAfterTheCall;
    const std::vector<DoorReading> readings = readingDoorSession(driver, 120u, kRateFrame, 1.01, refusedAfterTheCall);

    const std::vector<double>& hostSeconds = driver.side.game.beforePumpHostSeconds;
    REQUIRE(readings.size() == 120u);
    REQUIRE(hostSeconds.size() == 120u);
    const std::vector<DoorReading> reference = referenceDoorReadings(hostSeconds, kRateFrame, 1.01);
    for (size_t frame = 0u; frame < readings.size(); ++frame)
    {
        INFO("frame " << frame);
        CHECK(readings[frame].nextDeadline == reference[frame].nextDeadline);
        CHECK(readings[frame].currentInterval == reference[frame].currentInterval);
        CHECK(readings[frame].refusedRateChanges == 0u);
    }
    CHECK(readings[kRateFrame].currentInterval == static_cast<double>(kDt));
    CHECK(readings[kRateFrame + 1u].currentInterval == static_cast<double>(kDt) / 1.01);

    const std::vector<int32_t> steps = stepsPerFrame(driver.events);
    REQUIRE(steps.size() == 120u);
    const size_t firstFrameWithSteps = static_cast<size_t>(
        std::find_if(steps.begin(), steps.end(), [](int32_t n) { return n > 0; }) - steps.begin());
    REQUIRE(firstFrameWithSteps < 100u);
    for (size_t frame = 0u; frame < readings.size(); ++frame)
    {
        INFO("frame " << frame);
        CHECK(readings[frame].tickOffset == (frame <= firstFrameWithSteps ? 0 : 2));
    }
    const TickOutcome& last = driver.side.presenter.published.back();
    CHECK(static_cast<int64_t>(last.tick) - static_cast<int64_t>(last.physicsStep) == 2);
    CHECK(driver.pipeline.driver().tickOffset() == 2);

    CHECK(refusedAfterTheCall == std::optional<uint64_t>{ 0u });
    CHECK(driver.pipeline.rateScale() == 1.01);
    CHECK(driver.pipeline.driver().getDiagnostics().refusedRateChanges() == 0u);
}

TEST_CASE("SimulationTickDriver.BeforePump.AnAuthoritysRateChangeIsRefusedCountedAndChangesNothing", "[TickDriver]")
{
    const Setup setup{ true, true, 60 };
    Run<DriverPipeline> baseline(setup);
    Run<DriverPipeline> driver(setup);
    Run<LegacyPipeline> legacy(setup);
    std::optional<uint64_t> refusedAfterTheCall;
    rateScaleSession(baseline, 120u, std::nullopt, 1.01);
    const std::vector<DoorReading> readings = readingDoorSession(driver, 120u, 40u, 1.01, refusedAfterTheCall);
    rateScaleSession(legacy, 120u, 40u, 1.01);

    CHECK(refusedAfterTheCall == std::optional<uint64_t>{ 1u });
    CHECK(driver.pipeline.rateScale() == 1.0);
    CHECK(driver.pipeline.driver().getDiagnostics().refusedRateChanges() == 1u);
    CHECK(driver.pipeline.stepIntervalSeconds() == static_cast<double>(kDt));
    REQUIRE(readings.size() == 120u);
    for (size_t frame = 0u; frame < readings.size(); ++frame)
    {
        INFO("frame " << frame);
        CHECK(readings[frame].currentInterval == static_cast<double>(kDt));
        CHECK(readings[frame].refusedRateChanges == (frame <= 40u ? 0u : 1u));
    }

    CHECK(firstDifference(driver.events, baseline.events) == "");
    CHECK(driver.pipeline.physicsStepCount() == baseline.pipeline.physicsStepCount());
    CHECK(firstDifference(driver.events, legacy.events) == "");
    CHECK(legacy.pipeline.rateScale() == 1.0);
}

TEST_CASE("SimulationTickDriver.Begin.TheFrameLinePrintsTheConfiguredRateBounds", "[TickDriver]")
{
    const auto frameLine = [](const Events& events)
    {
        const auto line = std::find_if(events.begin(), events.end(),
            [](const std::string& event) { return event.rfind("log:W:[SimHost.Frame] ", 0) == 0; });
        return line == events.end() ? std::string() : *line;
    };

    const Setup setup{ true, true, 60 };
    Run<DriverPipeline> defaults(setup);
    CHECK(frameLine(defaults.events) ==
        "log:W:[SimHost.Frame] role=Authority stepping=inline dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] "
        "timeBase=worldDelta");

    Events events;
    TraceEngine engine(events);
    Side side(events, Setup{ false, false, 60 });
    Driver narrow(side.game, side.host, side.presenter, engine,
        Driver::Config{ static_cast<double>(kDt), false, StepThreading::FrameBounded, 60, 0.9, 1.1 });
    CHECK(frameLine(events) ==
        "log:W:[SimHost.Frame] role=Client stepping=worker dt=0.016667 maxCatchUpSteps=60 rateScale=[0.90,1.10] "
        "timeBase=worldDelta");
    narrow.stop();
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

namespace
{
    using IdleGame = BasicRecordingIdleGame<true, TestManager>;
    using RunnerDriver = SimulationTickDriver<IdleGame, FakeWorldHost, FakePresenter, FakeEngine>;
    using RealRunnerDriver = SimulationTickDriver<IdleGame, FakeWorldHost, FakePresenter, RealClockFakeEngine>;

    constexpr double kDtSeconds = static_cast<double>(std::bit_cast<float>(std::bit_cast<uint32_t>(kDt)));
    static_assert(kDtSeconds != 1.0 / 60.0, "kDtSeconds is the float kDt widened, as the Inline pipelines pass it; "
        "a constexpr static_cast<double>(kDt) keeps 1/60 in double under MSVC constant evaluation");
    constexpr double kLoopOriginSeconds = 0.0;

    struct RunnerSetup
    {
        IdleScript script = idleScript::HonourDeadline{};
        double idleSlackSeconds = 0.0;
        int32_t maxCatchUpSteps = 60;
        double buildClockSeconds = 0.0;
    };

    RunnerDriver::Config runnerConfig(const RunnerSetup& setup)
    {
        return RunnerDriver::Config{ kDtSeconds, true, StepThreading::FreeRunning, setup.maxCatchUpSteps, 0.75, 1.25,
            setup.idleSlackSeconds };
    }

    struct RunnerRun
    {
        explicit RunnerRun(const RunnerSetup& setup)
            : host(events, kAuthorityRoleBit, StepThreading::FreeRunning, kRingDepthTicks)
            , manager(false, events)
            , game(events, manager, storage, engine, setup.script)
            , presenter(events)
            , runner(engine, &events)
        {
            engine.now = setup.buildClockSeconds;
            driver.emplace(game, host, presenter, engine, runnerConfig(setup));
        }

        void runLoopThenStop()
        {
            driver->runAuthorityLoop(runner, stop);
            driver->stop();
        }

        size_t wakes() const { return game.beforePumpHostSeconds.size(); }

        Events events;
        FakeEngine engine;
        FakeWorldHost host;
        TestManager manager;
        FakeStorage storage;
        IdleGame game;
        FakePresenter presenter;
        FakeRunnerHost runner;
        std::atomic<bool> stop{ false };
        std::optional<RunnerDriver> driver;
    };

    std::vector<float> jitteredAuthorityDeltas(uint32_t frames, uint32_t hitchFrame, float hitchSeconds)
    {
        Lcg random(7u);
        std::vector<float> deltas;
        for (uint32_t frame = 0u; frame < frames; ++frame)
            deltas.push_back(frame == hitchFrame ? hitchSeconds : kDt * (0.3f + 1.4f * random.unit()));
        return deltas;
    }

    std::vector<double> wakeTimes(const std::vector<float>& deltas)
    {
        std::vector<double> times;
        double hostSeconds = 0.0;
        for (float delta : deltas)
        {
            hostSeconds += static_cast<double>(delta);
            times.push_back(hostSeconds);
        }
        return times;
    }

    Events stepTrace(const Events& events)
    {
        Events trace;
        for (const std::string& event : events)
        {
            if (event == "clock" || event == "onPostPhysics" || event == "sleepUntil" || event.rfind("log:", 0) == 0
                || event.rfind("idleUntil:", 0) == 0)
                continue;
            trace.push_back(event);
        }
        return trace;
    }

    template <typename RunT>
    void inlineSessionAfterALeadingZeroFrame(RunT& run, const std::vector<float>& deltas)
    {
        FakeCompletion completion;
        run.pipeline.startFrame(0.f);
        run.pipeline.endFrame(&completion, 0);
        for (uint32_t frame = 0u; frame < deltas.size(); ++frame)
        {
            run.engine.advance(static_cast<double>(deltas[frame]));
            if (frame % 50u == 7u)
                run.pipeline.pushBodySlotOccupancy(frame % 4u, frame % 100u < 50u);
            if (frame % 97u == 3u)
                run.pipeline.noteWorldLockWait(0.0002 + 0.00001 * static_cast<double>(frame));
            run.pipeline.startFrame(deltas[frame]);
            run.pipeline.endFrame(&completion, 0);
        }
        run.pipeline.stop();
    }

    void runnerWokenAtTheFrameTimes(RunnerRun& run, const std::vector<float>& deltas)
    {
        const size_t frames = deltas.size();
        run.game.beforePumpHook = [&run, frames]()
        {
            const size_t wake = run.wakes() - 1u;
            if (wake == 0u)
                return;
            const uint32_t frame = static_cast<uint32_t>(wake - 1u);
            if (frame % 50u == 7u)
                run.driver->pushBodySlotOccupancy(frame % 4u, frame % 100u < 50u);
            if (frame % 97u == 3u)
                run.driver->noteWorldLockWait(0.0002 + 0.00001 * static_cast<double>(frame));
            if (wake == frames)
                run.stop.store(true);
        };
        run.runLoopThenStop();
    }

    std::vector<std::string> linesWith(const std::vector<LoggedLine>& lines, const std::string& prefix)
    {
        std::vector<std::string> found;
        for (const LoggedLine& line : lines)
        {
            if (line.text.rfind(prefix, 0) == 0)
                found.push_back(line.text);
        }
        return found;
    }

    std::string fieldOf(const std::string& line, const std::string& name)
    {
        const std::string key = " " + name + "=";
        const size_t at = line.find(key);
        if (at == std::string::npos)
            return "<missing>";
        const size_t from = at + key.size();
        return line.substr(from, line.find(' ', from) - from);
    }

    double p99Of(const std::string& percentiles)
    {
        const size_t first = percentiles.find('/');
        const size_t second = percentiles.find('/', first + 1u);
        return std::stod(percentiles.substr(first + 1u, second - first - 1u));
    }

    std::vector<std::string> idlePatternsBetweenSteps(const Events& events)
    {
        std::vector<std::string> patterns;
        for (const std::string& event : events)
        {
            if (event == "onStepEnded")
                patterns.emplace_back();
            else if (!patterns.empty() && event.rfind("idleUntil:", 0) == 0)
                patterns.back() += 'i';
            else if (!patterns.empty() && event == "sleepUntil")
                patterns.back() += 's';
        }
        return patterns;
    }

    struct OffRunnerReading
    {
        double hostNowSeconds = -1.0;
        double stepIntervalSeconds = -1.0;
    };

    OffRunnerReading readOffTheRunner(RunnerDriver& driver)
    {
        OffRunnerReading reading;
        std::thread reader([&driver, &reading]() { reading = OffRunnerReading{ driver.hostNowSeconds(), driver.stepIntervalSeconds() }; });
        reader.join();
        return reading;
    }
} // namespace

TEST_CASE("SimulationTickDriver.FreeRunning.WokenAtTheInlineFrameTimesItGivesTheInlineAuthoritysStepTrace", "[TickDriver]")
{
    const std::vector<float> deltas = jitteredAuthorityDeltas(600u, 300u, 0.25f);
    {
        Run<DriverPipeline> reference(Setup{ true, true, 60 });
        REQUIRE(authorityInlineSession(reference, 600u, 300u, 0.25f) == deltas);
    }

    Run<DriverPipeline> inlineAuthority(Setup{ true, true, 60 });
    inlineSessionAfterALeadingZeroFrame(inlineAuthority, deltas);
    RunnerRun runner(RunnerSetup{ idleScript::Wakes{ wakeTimes(deltas) } });
    runnerWokenAtTheFrameTimes(runner, deltas);

    const Events runnerTrace = stepTrace(runner.events);
    const Events inlineTrace = stepTrace(inlineAuthority.events);
    CHECK(firstDifference(runnerTrace, inlineTrace) == "");

    CHECK(countPrefix(runnerTrace, "beforePump") == 601u);
    CHECK(countPrefix(runnerTrace, "onFrameStepsDue:") > 300u);
    CHECK(countPrefix(runnerTrace, "afterInlineBatch:") == countPrefix(runnerTrace, "onFrameStepsDue:"));
    CHECK(countPrefix(runnerTrace, "releaseInputs:") == runner.driver->getDiagnostics().physicsStepCount());
    CHECK(countPrefix(runnerTrace, "onStepStarted") == runner.driver->getDiagnostics().physicsStepCount());
    CHECK(countPrefix(runnerTrace, "onStepEnded") == runner.driver->getDiagnostics().physicsStepCount());
    CHECK(countPrefix(runnerTrace, "publish:") == runner.driver->getDiagnostics().physicsStepCount());
    CHECK(countPrefix(runnerTrace, "afterRunTick:") == runner.driver->getDiagnostics().physicsStepCount());
    CHECK(countPrefix(runnerTrace, "occ") > 0u);
    REQUIRE_FALSE(runner.game.frameStepsDue.empty());
    CHECK(runner.game.frameStepsDue.front() == std::make_pair(0, 1));
    CHECK(std::count_if(runner.game.frameStepsDue.begin(), runner.game.frameStepsDue.end(),
        [](const std::pair<int32_t, int32_t>& due) { return due.second == 15; }) == 1);

    CHECK(runner.driver->getDiagnostics().physicsStepCount() == inlineAuthority.pipeline.physicsStepCount());
    CHECK(runner.driver->getDiagnostics().lostTime().steps == inlineAuthority.pipeline.lostTime().steps);
    CHECK(runner.driver->getDiagnostics().lostTime().seconds == inlineAuthority.pipeline.lostTime().seconds);
    CHECK(runner.game.beforePumpHostSeconds == inlineAuthority.side.game.beforePumpHostSeconds);
    CHECK(runner.game.scriptExhaustedCalls == 1u);
    CHECK(countPrefix(runner.events, "onPostPhysics") == 0u);
    CHECK(countPrefix(inlineAuthority.events, "onPostPhysics") == 601u);
    CHECK(linesWith(runner.engine.lines, "[SimHost.Frame] ")
        == std::vector<std::string>{ "[SimHost.Frame] role=Authority stepping=runner dt=0.016667 maxCatchUpSteps=60 "
                                     "rateScale=[0.75,1.25] timeBase=monotonic idleSlackUs=0" });
    CHECK(linesWith(runner.engine.lines, "[SimHost.FirstStep] ")
        == std::vector<std::string>{ "[SimHost.FirstStep] role=Authority thread=runner physicsStep=0 batch=1" });
    CHECK_FALSE(runner.driver->getDiagnostics().isRunning());
}

TEST_CASE("SimulationTickDriver.FreeRunning.ADeadlineHonouringIdleRunsOneStepPerWakeAtItsDeadline", "[TickDriver]")
{
    RunnerRun run(RunnerSetup{ .buildClockSeconds = -1.0 });
    run.engine.now = kLoopOriginSeconds;
    run.game.setStopAfterSteps(run.stop, 600u);
    run.runLoopThenStop();

    REQUIRE(run.presenter.published.size() == 600u);
    REQUIRE(run.game.frameStepsDue.size() == 600u);
    CHECK(std::all_of(run.game.frameStepsDue.begin(), run.game.frameStepsDue.end(),
        [](const std::pair<int32_t, int32_t>& due) { return due.second == 1; }));
    REQUIRE(run.game.idleCalls.size() == 600u);
    REQUIRE(run.game.stepStartedSeconds.size() == 600u);
    for (size_t step = 0u; step < 600u; ++step)
    {
        CHECK(run.game.stepStartedSeconds[step] == kLoopOriginSeconds + run.presenter.publishedDeadlines[step]);
        if (step > 0u)
            CHECK(run.game.idleCalls[step - 1u].deadline == kLoopOriginSeconds + run.presenter.publishedDeadlines[step]);
    }
    CHECK(std::all_of(run.game.idleCalls.begin(), run.game.idleCalls.end(),
        [](const IdleCall& call) { return call.wake == IdleWake::Deadline && call.deadline > call.now; }));

    const std::vector<std::string> windows = linesWith(run.engine.lines, "[SimHost.Window] role=Authority stepping=runner ");
    REQUIRE(windows.size() == 1u);
    for (const std::string& line : windows)
    {
        CHECK(fieldOf(line, "maxStepsPerFrame") == "1");
        CHECK(fieldOf(line, "lateUs") == "0.0/0.0/0.0");
        CHECK(fieldOf(line, "frames") == fieldOf(line, "steps"));
        CHECK(fieldOf(line, "frames") == "541");
        CHECK(fieldOf(line, "lostSteps") == "0");
    }
    const TickDriverWindowCounts& counts = run.driver->getDiagnostics().windowCounts();
    CHECK(counts.frames == 59u);
    CHECK(counts.steps == 59u);
    CHECK(counts.maxStepsPerFrame == 1u);
    CHECK(counts.activityWakes == 0u);
    CHECK(counts.earlyWakes == 0u);
    CHECK(counts.idleFallbacks == 0u);
    CHECK(run.driver->getDiagnostics().windowSampleCount(TickDriverWindowSeries::LateUs) == 59u);
    CHECK(run.driver->getDiagnostics().windowSampleCount(TickDriverWindowSeries::ApproachUs) == 0u);
    CHECK(run.runner.sleeps.empty());
}

TEST_CASE("SimulationTickDriver.FreeRunning.ActivityWakesRePumpWithoutStepping", "[TickDriver]")
{
    std::vector<double> activity;
    for (uint32_t index = 1u; index <= 3400u; ++index)
        activity.push_back(0.003 * static_cast<double>(index));
    RunnerRun run(RunnerSetup{ idleScript::ActivityAt{ activity } });
    run.game.setStopAfterSteps(run.stop, 600u);
    run.runLoopThenStop();

    REQUIRE(run.presenter.published.size() == 600u);
    REQUIRE(run.game.stepStartedSeconds.size() == 600u);
    const uint32_t activityReturns = static_cast<uint32_t>(std::count_if(run.game.idleCalls.begin(), run.game.idleCalls.end(),
        [](const IdleCall& call) { return call.wake == IdleWake::Activity; }));
    const TickDriverWindowCounts& counts = run.driver->getDiagnostics().windowCounts();
    CHECK(activityReturns > 3000u);
    CHECK(counts.activityWakes == activityReturns);
    CHECK(counts.earlyWakes == 0u);
    CHECK(counts.idleFallbacks == 0u);
    CHECK(counts.frames == 600u);
    CHECK(counts.steps == 600u);
    REQUIRE_FALSE(run.game.idleCalls.empty());
    const uint32_t finalActivity = run.game.idleCalls.back().wake == IdleWake::Activity ? 1u : 0u;
    CHECK(countPrefix(run.events, "beforePump") == 600u + activityReturns - finalActivity);
    CHECK(countPrefix(run.events, "beforePump") == run.game.idleCalls.size());
    CHECK(run.game.frameStepsDue.size() == 600u);
    CHECK(std::all_of(run.game.frameStepsDue.begin(), run.game.frameStepsDue.end(),
        [](const std::pair<int32_t, int32_t>& due) { return due.second == 1; }));
    for (size_t step = 0u; step < 600u; ++step)
    {
        CHECK(run.game.stepStartedSeconds[step] >= kLoopOriginSeconds + run.presenter.publishedDeadlines[step]);
        CHECK(run.game.stepStartedSeconds[step] < 10.0);
    }
    CHECK(run.presenter.publishedDeadlines.back() + kDtSeconds >= 10.0);
    CHECK(linesWith(run.engine.lines, "[SimHost.Window] ").empty());
}

TEST_CASE("SimulationTickDriver.FreeRunning.TheSlackIdlesToTheApproachThenPollsToTheDeadline", "[TickDriver]")
{
    constexpr double kSlackSeconds = 0.0005;
    RunnerRun run(RunnerSetup{ idleScript::HonourDeadline{}, kSlackSeconds });
    run.game.setStopAfterSteps(run.stop, 600u);
    run.runLoopThenStop();

    REQUIRE(run.presenter.published.size() == 600u);
    REQUIRE(run.game.stepStartedSeconds.size() == 600u);
    std::vector<std::vector<IdleCall>> idles;
    for (const IdleCall& call : run.game.idleCalls)
    {
        if (call.deadline > call.now)
            idles.emplace_back();
        REQUIRE_FALSE(idles.empty());
        idles.back().push_back(call);
    }
    REQUIRE(idles.size() == 600u);
    for (size_t idle = 0u; idle + 1u < idles.size(); ++idle)
    {
        const double deadline = kLoopOriginSeconds + run.presenter.publishedDeadlines[idle + 1u];
        const std::vector<IdleCall>& calls = idles[idle];
        CHECK(calls.front().deadline == kLoopOriginSeconds + (run.presenter.publishedDeadlines[idle + 1u] - kSlackSeconds));
        REQUIRE(calls.size() >= 2u);
        for (size_t poll = 1u; poll < calls.size(); ++poll)
        {
            CHECK(calls[poll].deadline <= calls[poll].now);
            CHECK(calls[poll].now < deadline);
        }
        CHECK(calls.back().returnedAt >= deadline);
    }
    CHECK(idles.back().size() == 1u);
    double maxLateSeconds = 0.0;
    for (size_t step = 0u; step < 600u; ++step)
    {
        const double lateSeconds = run.game.stepStartedSeconds[step] - (kLoopOriginSeconds + run.presenter.publishedDeadlines[step]);
        CHECK(lateSeconds >= 0.0);
        maxLateSeconds = std::max(maxLateSeconds, lateSeconds);
    }
    CHECK(maxLateSeconds <= 50e-6);
    CHECK(maxLateSeconds > 0.0);
    CHECK(run.driver->getDiagnostics().windowSampleCount(TickDriverWindowSeries::LateUs) == 600u);
    CHECK(run.driver->getDiagnostics().windowSampleCount(TickDriverWindowSeries::ApproachUs) == 600u);
    CHECK(run.driver->getDiagnostics().windowCounts().earlyWakes == 0u);
    CHECK(linesWith(run.engine.lines, "[SimHost.Frame] ")
        == std::vector<std::string>{ "[SimHost.Frame] role=Authority stepping=runner dt=0.016667 maxCatchUpSteps=60 "
                                     "rateScale=[0.75,1.25] timeBase=monotonic idleSlackUs=500" });
}

TEST_CASE("SimulationTickDriver.FreeRunning.AnUnavailableIdleSleepsOnTheRunnerHost", "[TickDriver]")
{
    RunnerRun run(RunnerSetup{ idleScript::Unavailable{} });
    run.game.setStopAfterSteps(run.stop, 60u);
    run.runLoopThenStop();

    REQUIRE(run.presenter.published.size() == 60u);
    REQUIRE(run.game.idleCalls.size() == 60u);
    REQUIRE(run.runner.sleeps.size() == 60u);
    for (size_t idle = 0u; idle < 60u; ++idle)
    {
        CHECK(run.game.idleCalls[idle].wake == IdleWake::Unavailable);
        CHECK(run.runner.sleeps[idle] == run.game.idleCalls[idle].deadline);
        if (idle + 1u < 60u)
            CHECK(run.game.idleCalls[idle].deadline == kLoopOriginSeconds + run.presenter.publishedDeadlines[idle + 1u]);
    }
    CHECK(idlePatternsBetweenSteps(run.events) == std::vector<std::string>(60u, "is"));
    CHECK(run.driver->getDiagnostics().windowCounts().idleFallbacks == 60u);
    CHECK(run.driver->getDiagnostics().windowCounts().earlyWakes == 0u);
    for (size_t step = 0u; step < 60u; ++step)
        CHECK(run.game.stepStartedSeconds[step] >= kLoopOriginSeconds + run.presenter.publishedDeadlines[step]);
}

TEST_CASE("SimulationTickDriver.FreeRunning.ThreeEarlyDeadlineReturnsFallBackToTheRunnerSleep", "[TickDriver]")
{
    RunnerRun run(RunnerSetup{ idleScript::EarlyDeadline{} });
    run.game.setStopAfterSteps(run.stop, 30u);
    run.runLoopThenStop();

    REQUIRE(run.presenter.published.size() == 30u);
    const std::vector<std::string> patterns = idlePatternsBetweenSteps(run.events);
    REQUIRE(patterns.size() == 30u);
    for (const std::string& pattern : patterns)
    {
        CHECK(std::count(pattern.begin(), pattern.end(), 'i') <= 4);
        CHECK(std::count(pattern.begin(), pattern.end(), 's') == 1);
        CHECK(pattern.back() == 's');
    }
    CHECK(run.driver->getDiagnostics().windowCounts().earlyWakes == 30u * simulationTickDriver::kEarlyWakesBeforeFallback);
    CHECK(run.driver->getDiagnostics().windowCounts().idleFallbacks == 30u);
    for (size_t step = 0u; step < 30u; ++step)
        CHECK(run.game.stepStartedSeconds[step] >= kLoopOriginSeconds + run.presenter.publishedDeadlines[step]);
}

TEST_CASE("SimulationTickDriver.FreeRunning.AStalledRunnerCatchesUpToTheCapAndCountsTheRest", "[TickDriver]")
{
    std::vector<float> deltas(30u, kDt);
    deltas.push_back(2.f);
    deltas.insert(deltas.end(), 30u, kDt);

    Run<LegacyPipeline> transcription(Setup{ true, true, 60 });
    inlineSessionAfterALeadingZeroFrame(transcription, deltas);
    RunnerRun run(RunnerSetup{ idleScript::Wakes{ wakeTimes(deltas) } });
    runnerWokenAtTheFrameTimes(run, deltas);

    CHECK(firstDifference(stepTrace(run.events), stepTrace(transcription.events)) == "");
    CHECK(std::count_if(run.game.frameStepsDue.begin(), run.game.frameStepsDue.end(),
        [](const std::pair<int32_t, int32_t>& due) { return due.second == 60; }) == 1);
    CHECK(std::all_of(run.game.frameStepsDue.begin(), run.game.frameStepsDue.end(),
        [](const std::pair<int32_t, int32_t>& due) { return due.second <= 60; }));
    CHECK(run.driver->getDiagnostics().windowCounts().maxStepsPerFrame == 60u);

    const LostTime runnerLost = run.driver->getDiagnostics().lostTime();
    const LostTime transcriptionLost = transcription.pipeline.lostTime();
    CHECK(runnerLost.steps > 0u);
    CHECK(runnerLost.steps == transcriptionLost.steps);
    CHECK(runnerLost.seconds == transcriptionLost.seconds);
}

TEST_CASE("SimulationTickDriver.FreeRunning.AFrameThreadStallLeavesTheRunnersStepsOnePerWake", "[TickDriver]")
{
    RunnerRun run(RunnerSetup{});
    uint32_t endFrames = 0u;
    uint32_t stalledWakes = 0u;
    run.game.beforePumpHook = [&run, &endFrames, &stalledWakes]()
    {
        if (run.engine.now >= 1.0 && run.engine.now < 1.2)
        {
            ++stalledWakes;
            return;
        }
        std::thread frameThread([&run]() { run.driver->endFrame(nullptr, 0); });
        frameThread.join();
        ++endFrames;
    };
    run.game.setStopAfterSteps(run.stop, 120u);
    run.runLoopThenStop();

    REQUIRE(run.presenter.published.size() == 120u);
    CHECK(stalledWakes >= 11u);
    CHECK(run.game.postPhysicsCalls == endFrames);
    CHECK(std::all_of(run.game.frameStepsDue.begin(), run.game.frameStepsDue.end(),
        [](const std::pair<int32_t, int32_t>& due) { return due.second == 1; }));
    const uint32_t runnerMax = run.driver->getDiagnostics().windowCounts().maxStepsPerFrame;
    CHECK(runnerMax <= 2u);
    CHECK(runnerMax == 1u);

    Run<DriverPipeline> inlineAuthority(Setup{ true, true, 60 });
    FakeCompletion completion;
    double hostSeconds = 0.0;
    bool stalled = false;
    for (uint32_t frame = 0u; hostSeconds < 2.0; ++frame)
    {
        float delta = offTheDeadlines(frame, kDt);
        if (!stalled && hostSeconds >= 1.0)
        {
            delta = 0.2f;
            stalled = true;
        }
        inlineAuthority.engine.advance(static_cast<double>(delta));
        inlineAuthority.pipeline.startFrame(delta);
        inlineAuthority.pipeline.endFrame(&completion, 0);
        hostSeconds += static_cast<double>(delta);
    }
    inlineAuthority.pipeline.stop();
    const uint32_t inlineMax = inlineAuthority.pipeline.driver().getDiagnostics().windowCounts().maxStepsPerFrame;
    CHECK(stalled);
    CHECK(inlineMax >= 12u);
    CHECK_FALSE(inlineMax <= 2u);
}

TEST_CASE("SimulationTickDriver.FreeRunning.TheFrameThreadPassRunsOnlyOnPostPhysics", "[TickDriver]")
{
    Events events;
    FakeEngine engine;
    TestManager manager(false, events);
    FakeStorage storage;
    IdleGame game(events, manager, storage, engine);
    FakePresenter presenter(events);
    std::optional<FakeWorldHost> host;
    std::optional<RunnerDriver> driver;
    std::thread runner([&]()
    {
        host.emplace(events, kAuthorityRoleBit, StepThreading::FreeRunning, kRingDepthTicks);
        driver.emplace(game, *host, presenter, engine, runnerConfig(RunnerSetup{}));
    });
    runner.join();
    REQUIRE(driver.has_value());

    const size_t eventsAtBuild = events.size();
    const size_t linesAtBuild = engine.lines.size();
    FakeCompletion completion;
    for (uint32_t frame = 0u; frame < 30u; ++frame)
    {
        engine.advance(kDtSeconds);
        driver->startFrame(kDtSeconds);
        driver->endFrame(frame % 2u == 0u ? &completion : nullptr, static_cast<int32_t>(frame % 2u));
    }

    CHECK(Events(events.begin() + static_cast<std::ptrdiff_t>(eventsAtBuild), events.end()) == Events(30u, "onPostPhysics"));
    CHECK(game.postPhysicsCalls == 30u);
    CHECK(game.beforePumpHostSeconds.empty());
    CHECK(driver->getDiagnostics().physicsStepCount() == 0u);
    const TickDriverWindowCounts& counts = driver->getDiagnostics().windowCounts();
    CHECK(counts.frames == 0u);
    CHECK(counts.steps == 0u);
    CHECK(counts.endWaits == 0u);
    for (size_t series = 0u; series < tickDriverWindow::kSeriesCount; ++series)
        CHECK(driver->getDiagnostics().windowSampleCount(static_cast<TickDriverWindowSeries>(series)) == 0u);
    CHECK(engine.lines.size() == linesAtBuild);
    CHECK(engine.deferrals.empty());
    CHECK(engine.waits.empty());
    CHECK(completion.continuations.empty());
    CHECK(driver->hostNowSeconds() == 0.0);

    driver->stop();
    CHECK(engine.waits.empty());
    CHECK_FALSE(driver->getDiagnostics().isRunning());
}

TEST_CASE("SimulationTickDriver.FreeRunning.ThePresentClockIsReadableOffTheRunner", "[TickDriver]")
{
    constexpr double kClockAtStartSeconds = 5.0;
    RunnerRun run(RunnerSetup{ .buildClockSeconds = kClockAtStartSeconds });
    const OffRunnerReading beforeLoop = readOffTheRunner(*run.driver);
    CHECK(beforeLoop.hostNowSeconds == 0.0);
    CHECK(beforeLoop.stepIntervalSeconds == kDtSeconds);

    std::vector<OffRunnerReading> readings;
    std::vector<double> clockAtWake;
    run.game.onBeforePump = [&run](double, TickRateDoor& rate)
    {
        if (run.wakes() == 11u)
            rate.setRateScale(1.25);
    };
    run.game.beforePumpHook = [&run, &readings, &clockAtWake]()
    {
        clockAtWake.push_back(run.engine.now);
        readings.push_back(readOffTheRunner(*run.driver));
        if (run.wakes() == 21u)
            run.stop.store(true);
    };
    run.runLoopThenStop();

    REQUIRE(readings.size() == 21u);
    REQUIRE(run.game.beforePumpHostSeconds.size() == 21u);
    for (size_t wake = 0u; wake < readings.size(); ++wake)
    {
        CHECK(readings[wake].hostNowSeconds == clockAtWake[wake] - kClockAtStartSeconds);
        CHECK(readings[wake].hostNowSeconds == run.game.beforePumpHostSeconds[wake]);
        CHECK(readings[wake].stepIntervalSeconds == kDtSeconds);
    }
    CHECK(readings.back().hostNowSeconds > 0.0);
    CHECK(run.driver->getDiagnostics().rateScale() == 1.0);
    CHECK(run.driver->getDiagnostics().refusedRateChanges() == 1u);
    const OffRunnerReading afterLoop = readOffTheRunner(*run.driver);
    CHECK(afterLoop.hostNowSeconds == run.engine.now - kClockAtStartSeconds);
    CHECK(afterLoop.stepIntervalSeconds == kDtSeconds);
}

TEST_CASE("SimulationTickDriver.FreeRunning.StopFromTheFrameThreadEndsTheLoopAndStopChecksIt", "[TickDriver]")
{
    Events events;
    RealClockFakeEngine engine;
    TestManager manager(false, events);
    FakeStorage storage;
    IdleGame game(events, manager, storage, IdleScript{ idleScript::RealDeadline{} });
    FakePresenter presenter(events);
    RealRunnerHost runnerHost;
    std::atomic<bool> stop{ false };
    std::optional<FakeWorldHost> host;
    std::optional<RealRunnerDriver> driver;
    std::thread runner([&]()
    {
        host.emplace(events, kAuthorityRoleBit, StepThreading::FreeRunning, kRingDepthTicks);
        driver.emplace(game, *host, presenter, engine,
            RealRunnerDriver::Config{ kDtSeconds, true, StepThreading::FreeRunning, 60 });
        driver->runAuthorityLoop(runnerHost, stop);
    });

    const auto giveUp = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (game.stepsEnded.load() < 5u && std::chrono::steady_clock::now() < giveUp)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const uint32_t stepsAtStop = game.stepsEnded.load();
    stop.store(true);
    runner.join();

    REQUIRE(stepsAtStop >= 5u);
    REQUIRE(driver.has_value());
    const uint32_t stepsAtJoin = game.stepsEnded.load();
    driver->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK(game.stepsEnded.load() == stepsAtJoin);
    CHECK(stepsAtJoin >= stepsAtStop);
    CHECK_FALSE(driver->getDiagnostics().isRunning());
    const std::vector<LoggedLine> lines = engine.snapshotLines();
    CHECK(linesWith(lines, "[SimHost.FirstStep] role=Authority thread=runner ").size() == 1u);
    CHECK(linesWith(lines, "[SimHost.Frame] role=Authority stepping=runner ").size() == 1u);
}

TEST_CASE("SimulationTickDriver.FreeRunning.StopWhileTheLoopIsActiveIsRefused", "[.][TickDriverStopBeforeJoin]")
{
    RunnerRun& run = *new RunnerRun(RunnerSetup{});
    run.game.beforePumpHook = [&run]()
    {
        if (run.wakes() == 3u)
        {
            run.stop.store(true);
            run.driver->stop();
        }
    };
    run.driver->runAuthorityLoop(run.runner, run.stop);
    FAIL("SimulationTickDriver::stop() was accepted while the authority loop was active");
}

TEST_CASE("SimulationTickDriver.FreeRunning.Timing.LoopOverheadWithASpinPreciseIdle", "[.][OGSimTiming]")
{
    Events events;
    RealClockFakeEngine engine;
    FakeWorldHost host(events, kAuthorityRoleBit, StepThreading::FreeRunning, kRingDepthTicks);
    TestManager manager(false, events);
    FakeStorage storage;
    IdleGame game(events, manager, storage, IdleScript{ idleScript::RealSpin{} });
    FakePresenter presenter(events);
    RealRunnerHost runnerHost;
    std::atomic<bool> stop{ false };
    RealRunnerDriver driver(game, host, presenter, engine, RealRunnerDriver::Config{ kDtSeconds, true, StepThreading::FreeRunning, 60 });
    game.setStopAfterSteps(stop, 601u);
    driver.runAuthorityLoop(runnerHost, stop);
    driver.stop();

    const std::vector<std::string> windows = linesWith(engine.snapshotLines(), "[SimHost.Window] role=Authority stepping=runner ");
    REQUIRE(windows.size() == 1u);
    WARN(windows.front());
    WARN("lateUs=" + fieldOf(windows.front(), "lateUs"));
    CHECK(fieldOf(windows.front(), "steps") == "601");
    CHECK(p99Of(fieldOf(windows.front(), "lateUs")) <= 100.0);
}

#endif // WITH_LOW_LEVEL_TESTS
