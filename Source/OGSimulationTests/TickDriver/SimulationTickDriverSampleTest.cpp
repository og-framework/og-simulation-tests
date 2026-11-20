// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
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
#include "OGSimulation/TickDriverConcepts.h"
#include "OGSimulation/TickDriverThreadContext.h"
#include "TickDriverFakes.h"

namespace simulationTickDriverSampleTest
{
    using namespace tickDriverFakes;
    using tickDriverThread::kAuthorityRoleBit;
    using tickDriverThread::kClientRoleBit;

    constexpr float kDt = 1.f / 60.f;
    constexpr uint32_t kRingDepthTicks = 64u;
    constexpr uint32_t kFrames = 90u;

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
            m_events->push_back(info.isResimulation() ? "postResim" : "post");
        }

        unsigned int onCheckIsSimilar() { return std::numeric_limits<unsigned int>::max(); }

        void prepareResimulation(int32_t, uint32_t simTick) { m_clock.startResimulation(simTick); }

        uint32_t currentIntegratedTick() const { return m_lastStep.has_value() ? m_lastStep->getTick() : 0u; }
        const std::optional<SimulationTimeStep>& lastIntegratedStep() const { return m_lastStep; }

    private:
        bool m_runsPrediction;
        Events* m_events;
        PredictionClock m_clock;
        ServerTickClock m_serverClock;
        ResimGateProbe m_probe;
        std::optional<SimulationTimeStep> m_lastStep;
    };

    class RecordingEngine : public FakeEngine
    {
    public:
        explicit RecordingEngine(Events& events)
            : m_events(&events)
        {
        }

        double nowSeconds() const
        {
            m_events->push_back("clock");
            return now;
        }

    private:
        Events* m_events;
    };

    static_assert(TickDriverEngine<RecordingEngine>, "RecordingEngine must model TickDriverEngine");

    using PlainGame = BasicRecordingGame<TestManager>;
    using SampleGame = BasicSamplingGame<PlainGame>;
    using SampleIdleGame = BasicSamplingGame<BasicRecordingIdleGame<true, TestManager>>;

    static_assert(TickDriverSampleGame<SampleGame> && !TickDriverSampleGame<PlainGame>,
        "the sampling rig samples and the plain rig does not");
    static_assert(TickDriverSampleGame<SampleIdleGame> && TickDriverIdleGame<SampleIdleGame>,
        "the FreeRunning rig's game both samples and idles");

    enum class Mode
    {
        InlineAuthority,
        FrameBoundedClient,
    };

    bool isAuthority(Mode mode) { return mode == Mode::InlineAuthority; }
    StepThreading threadingOf(Mode mode) { return mode == Mode::InlineAuthority ? StepThreading::Inline : StepThreading::FrameBounded; }
    const char* nameOf(Mode mode) { return mode == Mode::InlineAuthority ? "Inline authority" : "FrameBounded client"; }

    template <typename GameT>
    struct FrameRig
    {
        using Driver = SimulationTickDriver<GameT, FakeWorldHost, FakePresenter, RecordingEngine>;

        explicit FrameRig(Mode mode)
            : engine(events)
            , host(events, isAuthority(mode) ? kAuthorityRoleBit : kClientRoleBit, threadingOf(mode), kRingDepthTicks)
            , manager(!isAuthority(mode), events)
            , game(events, manager, storage)
            , presenter(events)
            , driver(game, host, presenter, engine,
                typename Driver::Config{ static_cast<double>(kDt), isAuthority(mode), threadingOf(mode), 60 })
        {
        }

        Events events;
        RecordingEngine engine;
        FakeWorldHost host;
        TestManager manager;
        FakeStorage storage;
        GameT game;
        FakePresenter presenter;
        Driver driver;
    };

    struct FrameSlice
    {
        size_t start = 0u;
        size_t startFrameEnd = 0u;
        size_t end = 0u;
    };

    float deltaOf(uint32_t frame)
    {
        constexpr float kFactors[] = { 0.5f, 1.0f, 2.0f, 0.25f, 1.5f };
        return kDt * kFactors[frame % 5u];
    }

    template <typename RigT>
    std::vector<FrameSlice> runFrames(RigT& rig)
    {
        std::vector<FrameSlice> slices;
        for (uint32_t frame = 0u; frame < kFrames; ++frame)
        {
            const float delta = deltaOf(frame);
            FrameSlice slice;
            slice.start = rig.events.size();
            rig.driver.startFrame(static_cast<double>(delta));
            slice.startFrameEnd = rig.events.size();
            rig.driver.endFrame(nullptr, 0);
            slice.end = rig.events.size();
            slices.push_back(slice);
            rig.engine.advance(static_cast<double>(delta));
        }
        rig.driver.stop();
        return slices;
    }

    Events eventsIn(const Events& events, size_t begin, size_t end)
    {
        return Events(events.begin() + static_cast<std::ptrdiff_t>(begin), events.begin() + static_cast<std::ptrdiff_t>(end));
    }

    size_t countOf(const Events& events, const std::string& event)
    {
        return static_cast<size_t>(std::count(events.begin(), events.end(), event));
    }

    size_t indexOf(const Events& events, const std::string& event)
    {
        return static_cast<size_t>(std::find(events.begin(), events.end(), event) - events.begin());
    }

    Events withoutSamples(const Events& events)
    {
        Events kept;
        for (const std::string& event : events)
        {
            if (event != "sampleInput")
                kept.push_back(event);
        }
        return kept;
    }
} // namespace simulationTickDriverSampleTest

using namespace simulationTickDriverSampleTest;

TEST_CASE("SimulationTickDriver.SampleInput.RunsOncePerStartFrameFirstAndBeforeBeforePump", "[TickDriver]")
{
    const Mode mode = GENERATE(Mode::InlineAuthority, Mode::FrameBoundedClient);
    CAPTURE(nameOf(mode));
    FrameRig<SampleGame> rig(mode);
    const std::vector<FrameSlice> slices = runFrames(rig);

    REQUIRE(slices.size() == kFrames);
    CHECK(rig.game.sampleInputCalls == kFrames);
    CHECK(countOf(rig.events, "sampleInput") == kFrames);
    CHECK(countOf(rig.events, "onStepStarted") > kFrames / 2u);
    for (size_t frame = 0u; frame < slices.size(); ++frame)
    {
        CAPTURE(frame);
        const Events started = eventsIn(rig.events, slices[frame].start, slices[frame].startFrameEnd);
        REQUIRE_FALSE(started.empty());
        CHECK(started.front() == "sampleInput");
        CHECK(countOf(started, "sampleInput") == 1u);
        CHECK(countOf(started, "beforePump") == 1u);
        CHECK(indexOf(started, "sampleInput") < indexOf(started, "beforePump"));
        CHECK(countOf(eventsIn(rig.events, slices[frame].startFrameEnd, slices[frame].end), "sampleInput") == 0u);
    }
}

TEST_CASE("SimulationTickDriver.SampleInput.AGameThatSamplesHasTheSameTraceWithTheSamplesRemoved", "[TickDriver]")
{
    const Mode mode = GENERATE(Mode::InlineAuthority, Mode::FrameBoundedClient);
    CAPTURE(nameOf(mode));
    FrameRig<PlainGame> plain(mode);
    FrameRig<SampleGame> sampling(mode);
    runFrames(plain);
    runFrames(sampling);

    CHECK(countOf(plain.events, "sampleInput") == 0u);
    CHECK(countOf(sampling.events, "sampleInput") == kFrames);
    CHECK(withoutSamples(sampling.events) == plain.events);
    CHECK(sampling.presenter.published.size() == plain.presenter.published.size());
    CHECK(sampling.engine.lines.size() == plain.engine.lines.size());
}

TEST_CASE("SimulationTickDriver.SampleInput.AFreeRunningAuthorityNeverSamples", "[TickDriver]")
{
    using Driver = SimulationTickDriver<SampleIdleGame, FakeWorldHost, FakePresenter, FakeEngine>;

    Events events;
    FakeEngine engine;
    FakeWorldHost host(events, kAuthorityRoleBit, StepThreading::FreeRunning, kRingDepthTicks);
    TestManager manager(false, events);
    FakeStorage storage;
    SampleIdleGame game(events, manager, storage, engine);
    FakePresenter presenter(events);
    FakeRunnerHost runnerHost(engine, &events);
    std::atomic<bool> stop{ false };
    Driver driver(game, host, presenter, engine, Driver::Config{ static_cast<double>(kDt), true, StepThreading::FreeRunning, 60 });
    game.setStopAfterSteps(stop, 60u);

    driver.runAuthorityLoop(runnerHost, stop);

    CHECK(presenter.published.size() == 60u);
    CHECK(game.beforePumpHostSeconds.size() >= 60u);

    const size_t eventsAfterLoop = events.size();
    std::thread frameThread([&]()
    {
        for (uint32_t frame = 0u; frame < 30u; ++frame)
        {
            driver.startFrame(static_cast<double>(kDt));
            driver.endFrame(nullptr, 0);
        }
    });
    frameThread.join();
    driver.stop();

    CHECK(game.sampleInputCalls == 0u);
    CHECK(countOf(events, "sampleInput") == 0u);
    CHECK(eventsIn(events, eventsAfterLoop, events.size()) == Events(30u, "onPostPhysics"));
}

#endif
