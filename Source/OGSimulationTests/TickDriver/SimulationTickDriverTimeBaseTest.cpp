// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/ServerTickClock.h"
#include "OGSimulation/ResimGateProbe.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationScheduler.h"
#include "OGTickDriver/SimulationTickDriver.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGTickDriver/TickDriverConcepts.h"
#include "OGTickDriver/TickDriverThreadContext.h"
#include "TickDriverFakes.h"

namespace simulationTickDriverTimeBaseTest
{
    using namespace tickDriverFakes;
    using tickDriverThread::kAuthorityRoleBit;
    using tickDriverThread::kClientRoleBit;

    constexpr float kDt = 1.f / 60.f;
    constexpr uint32_t kRingDepthTicks = 64u;
    constexpr int32_t kCap = 240;
    constexpr double kClampSeconds = 0.4;
    constexpr double kIgnoredFrameDelta = 99.0;
    constexpr double kHalfStep = 0.5 * static_cast<double>(kDt);

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

    using Game = BasicRecordingGame<TestManager>;
    using Driver = SimulationTickDriver<Game, FakeWorldHost, FakePresenter, RecordingEngine>;

    static_assert(Driver::Config{ 1.0 / 60.0, true, StepThreading::Inline, 1 }.frameTimeBase == FrameTimeBase::HostDelta,
        "HostDelta is the default time base");
    static_assert(Driver::Config{ 1.0 / 60.0, true, StepThreading::Inline, 1 }.monotonicFrameClampSeconds == 0.4,
        "the Monotonic frame clamp defaults to 0.4 s");

    enum class Mode
    {
        InlineAuthority,
        FrameBoundedClient,
    };

    bool isAuthority(Mode mode) { return mode == Mode::InlineAuthority; }
    StepThreading threadingOf(Mode mode) { return mode == Mode::InlineAuthority ? StepThreading::Inline : StepThreading::FrameBounded; }
    const char* nameOf(Mode mode) { return mode == Mode::InlineAuthority ? "Inline authority" : "FrameBounded client"; }

    std::string frameLineOf(Mode mode, const char* timeBase)
    {
        return std::string("[SimHost.Frame] role=") + (isAuthority(mode) ? "Authority stepping=inline" : "Client stepping=worker")
            + " dt=0.016667 maxCatchUpSteps=240 rateScale=[0.75,1.25] timeBase=" + timeBase;
    }

    Driver::Config configOf(Mode mode, FrameTimeBase timeBase)
    {
        Driver::Config config{ static_cast<double>(kDt), isAuthority(mode), threadingOf(mode), kCap };
        config.frameTimeBase = timeBase;
        return config;
    }

    struct Rig
    {
        Rig(Mode mode, FrameTimeBase timeBase)
            : engine(events)
            , host(events, isAuthority(mode) ? kAuthorityRoleBit : kClientRoleBit, threadingOf(mode), kRingDepthTicks)
            , manager(!isAuthority(mode), events)
            , game(events, manager, storage)
            , presenter(events)
            , driver(game, host, presenter, engine, configOf(mode, timeBase))
        {
        }

        uint32_t frame(double clockAdvanceSeconds, double frameDeltaSeconds)
        {
            engine.advance(clockAdvanceSeconds);
            const uint64_t before = driver.getDiagnostics().physicsStepCount();
            driver.startFrame(frameDeltaSeconds);
            const uint64_t after = driver.getDiagnostics().physicsStepCount();
            driver.endFrame(nullptr, 0);
            return static_cast<uint32_t>(after - before);
        }

        Events events;
        RecordingEngine engine;
        FakeWorldHost host;
        TestManager manager;
        FakeStorage storage;
        Game game;
        FakePresenter presenter;
        Driver driver;
    };

    double deltaOf(uint32_t frame)
    {
        constexpr float kFactors[] = { 0.5f, 1.0f, 2.0f, 0.25f, 1.5f };
        return static_cast<double>(kDt * kFactors[frame % 5u]);
    }

    std::vector<std::string> textsOf(const std::vector<LoggedLine>& lines)
    {
        std::vector<std::string> texts;
        for (const LoggedLine& line : lines)
            texts.push_back(line.text);
        return texts;
    }
} // namespace simulationTickDriverTimeBaseTest

using namespace simulationTickDriverTimeBaseTest;

TEST_CASE("SimulationTickDriver.TimeBase.MonotonicGivesTheHostDeltaStepsTraceAndLinesForTheSameDeltas", "[TickDriver]")
{
    const Mode mode = GENERATE(Mode::InlineAuthority, Mode::FrameBoundedClient);
    CAPTURE(nameOf(mode));
    Rig hostDelta(mode, FrameTimeBase::HostDelta);
    Rig monotonic(mode, FrameTimeBase::Monotonic);

    std::vector<uint32_t> hostDeltaSteps;
    std::vector<uint32_t> monotonicSteps;
    for (uint32_t frame = 0u; frame < 900u; ++frame)
    {
        const double delta = frame == 450u ? 0.25 : deltaOf(frame);
        hostDeltaSteps.push_back(hostDelta.frame(delta, delta));
        monotonicSteps.push_back(monotonic.frame(delta, kIgnoredFrameDelta));
    }
    hostDelta.driver.stop();
    monotonic.driver.stop();

    CHECK(hostDeltaSteps[450] == 15u);
    CHECK(*std::max_element(hostDeltaSteps.begin(), hostDeltaSteps.end()) == 15u);
    CHECK(std::count(hostDeltaSteps.begin(), hostDeltaSteps.end(), 0u) > 0);
    CHECK(monotonicSteps == hostDeltaSteps);
    CHECK(monotonic.events == hostDelta.events);
    CHECK(monotonic.game.beforePumpHostSeconds == hostDelta.game.beforePumpHostSeconds);
    CHECK(monotonic.game.stepStartedSeconds == hostDelta.game.stepStartedSeconds);
    CHECK(monotonic.presenter.publishedDeadlines == hostDelta.presenter.publishedDeadlines);
    CHECK(monotonic.driver.getDiagnostics().lostTime().steps == 0u);
    CHECK(hostDelta.driver.getDiagnostics().lostTime().steps == 0u);

    const std::vector<std::string> hostDeltaLines = textsOf(hostDelta.engine.lines);
    const std::vector<std::string> monotonicLines = textsOf(monotonic.engine.lines);
    REQUIRE(hostDeltaLines.size() >= 2u);
    REQUIRE(monotonicLines.size() == hostDeltaLines.size());
    CHECK(hostDeltaLines.front() == frameLineOf(mode, "worldDelta"));
    CHECK(monotonicLines.front() == frameLineOf(mode, "monotonic"));
    CHECK(std::vector<std::string>(monotonicLines.begin() + 1, monotonicLines.end())
        == std::vector<std::string>(hostDeltaLines.begin() + 1, hostDeltaLines.end()));
}

TEST_CASE("SimulationTickDriver.TimeBase.AOneSecondGapAdvancesTheClampAndCountsTheRestAsReanchorLoss", "[TickDriver]")
{
    const Mode mode = GENERATE(Mode::InlineAuthority, Mode::FrameBoundedClient);
    CAPTURE(nameOf(mode));
    Rig monotonic(mode, FrameTimeBase::Monotonic);
    Rig clampedHostDelta(mode, FrameTimeBase::HostDelta);
    Rig unclampedHostDelta(mode, FrameTimeBase::HostDelta);

    std::vector<uint32_t> monotonicSteps;
    std::vector<uint32_t> clampedSteps;
    for (uint32_t frame = 0u; frame < 120u; ++frame)
    {
        monotonicSteps.push_back(monotonic.frame(deltaOf(frame), kIgnoredFrameDelta));
        clampedSteps.push_back(clampedHostDelta.frame(deltaOf(frame), deltaOf(frame)));
        unclampedHostDelta.frame(deltaOf(frame), deltaOf(frame));
    }
    monotonicSteps.push_back(monotonic.frame(kHalfStep, kIgnoredFrameDelta));
    clampedSteps.push_back(clampedHostDelta.frame(kHalfStep, kHalfStep));
    unclampedHostDelta.frame(kHalfStep, kHalfStep);
    const double hostSecondsBeforeGap = monotonic.driver.hostNowSeconds();

    const uint32_t monotonicGapSteps = monotonic.frame(1.0, kIgnoredFrameDelta);
    const double hostSecondsAfterGap = monotonic.driver.hostNowSeconds();
    const uint32_t clampedGapSteps = clampedHostDelta.frame(1.0, kClampSeconds);
    const uint32_t unclampedGapSteps = unclampedHostDelta.frame(1.0, 1.0);
    const LostTimeByCause gapLoss = monotonic.driver.getDiagnostics().lostTimeByCause();

    for (uint32_t frame = 120u; frame < 240u; ++frame)
    {
        monotonicSteps.push_back(monotonic.frame(deltaOf(frame), kIgnoredFrameDelta));
        clampedSteps.push_back(clampedHostDelta.frame(deltaOf(frame), deltaOf(frame)));
    }
    monotonic.driver.stop();
    clampedHostDelta.driver.stop();
    unclampedHostDelta.driver.stop();

    CHECK(clampedGapSteps == 24u);
    CHECK(monotonicGapSteps == clampedGapSteps);
    CHECK(unclampedGapSteps == 60u);
    CHECK(gapLoss.reanchor.steps == 35u);
    CHECK(gapLoss.reanchor.seconds == Catch::Approx(0.6).margin(1e-9));
    CHECK(gapLoss.catchUp.steps == 0u);
    CHECK(monotonic.driver.getDiagnostics().lostTimeByCause().reanchor.steps == gapLoss.reanchor.steps);
    CHECK(hostSecondsAfterGap - hostSecondsBeforeGap == Catch::Approx(1.0).margin(1e-9));
    CHECK(monotonicSteps == clampedSteps);
    CHECK(clampedHostDelta.driver.getDiagnostics().lostTime().steps == 0u);
    CHECK(unclampedHostDelta.driver.getDiagnostics().lostTime().steps == 0u);
}

TEST_CASE("SimulationTickDriver.TimeBase.AReanchorOnResumeSkipsThePauseAndCountsItAsReanchorLoss", "[TickDriver]")
{
    const Mode mode = GENERATE(Mode::InlineAuthority, Mode::FrameBoundedClient);
    CAPTURE(nameOf(mode));
    Rig resumed(mode, FrameTimeBase::Monotonic);
    Rig clamped(mode, FrameTimeBase::Monotonic);

    for (uint32_t frame = 0u; frame < 60u; ++frame)
    {
        resumed.frame(deltaOf(frame), kIgnoredFrameDelta);
        clamped.frame(deltaOf(frame), kIgnoredFrameDelta);
    }
    resumed.frame(kHalfStep, kIgnoredFrameDelta);
    clamped.frame(kHalfStep, kIgnoredFrameDelta);
    const double pausedAtSeconds = resumed.driver.hostNowSeconds();

    resumed.engine.advance(5.0);
    clamped.engine.advance(5.0);
    resumed.driver.reanchor();
    const LostTimeByCause pauseLoss = resumed.driver.getDiagnostics().lostTimeByCause();

    CHECK(resumed.driver.hostNowSeconds() == Catch::Approx(pausedAtSeconds + 5.0).margin(1e-9));
    CHECK(pauseLoss.reanchor.steps == 299u);
    CHECK(pauseLoss.reanchor.seconds == Catch::Approx(5.0 - kHalfStep).margin(1e-9));
    CHECK(pauseLoss.catchUp.steps == 0u);

    const double shortFrame = 0.4 * static_cast<double>(kDt);
    CHECK(resumed.frame(shortFrame, kIgnoredFrameDelta) == 1u);
    CHECK(resumed.frame(shortFrame, kIgnoredFrameDelta) == 0u);
    CHECK(resumed.frame(shortFrame, kIgnoredFrameDelta) == 1u);
    CHECK(clamped.frame(shortFrame, kIgnoredFrameDelta) == 24u);
    CHECK(clamped.driver.getDiagnostics().lostTimeByCause().reanchor.steps == 276u);
    CHECK(clamped.driver.getDiagnostics().lostTimeByCause().reanchor.seconds
        == Catch::Approx(5.0 + shortFrame - kClampSeconds).margin(1e-9));

    resumed.driver.stop();
    clamped.driver.stop();

    CHECK(resumed.driver.getDiagnostics().lostTimeByCause().reanchor.steps == pauseLoss.reanchor.steps);
    CHECK(resumed.driver.getDiagnostics().lostTimeByCause().reanchor.seconds == pauseLoss.reanchor.seconds);
}

#endif
