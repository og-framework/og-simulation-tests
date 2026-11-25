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
#include "OGTickDriver/SimulationTickDriver.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGTickDriver/TickDriverConcepts.h"
#include "OGTickDriver/TickDriverThreadContext.h"
#include "TickDriverFakes.h"

namespace simulationTickDriverReceiveTest
{
    using namespace tickDriverFakes;
    using stepDriverTestSupport::tickEvent;
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

    struct Collected
    {
        SimTick tick = 0u;
        uint32_t token = 0u;
    };

    class InboxManager
    {
    public:
        InboxManager(bool runsPrediction, Events& events)
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
                m_events->push_back("collect:" + std::to_string(inbox));
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
            collected.push_back(Collected{ m_clock.getPredictionTick(), inbox });
            m_events->push_back("collect:" + std::to_string(inbox));
        }

        void onPostGameSimulation(const SimulationUpdateInfo& info)
        {
            m_events->push_back(info.isResimulation() ? "postResim" : "post");
            if (info.isResimulation() && !m_clock.isResimulating())
                pendingAnchor.reset();
        }

        unsigned int onCheckIsSimilar()
        {
            m_events->push_back("checkSimilar");
            return pendingAnchor.has_value() ? *pendingAnchor : std::numeric_limits<unsigned int>::max();
        }

        void prepareResimulation(int32_t, uint32_t simTick)
        {
            m_events->push_back(tickEvent("prepare", simTick));
            m_clock.startResimulation(simTick);
        }

        uint32_t currentIntegratedTick() const { return m_lastStep.has_value() ? m_lastStep->getTick() : 0u; }
        const std::optional<SimulationTimeStep>& lastIntegratedStep() const { return m_lastStep; }

        uint32_t inbox = 0u;
        std::optional<unsigned int> pendingAnchor;
        std::vector<Collected> collected;

    private:
        bool m_runsPrediction;
        Events* m_events;
        PredictionClock m_clock;
        ServerTickClock m_serverClock;
        ResimGateProbe m_probe;
        std::optional<SimulationTimeStep> m_lastStep;
    };

    using PlainGame = BasicRecordingGame<InboxManager>;
    using ReceiveGame = BasicReceivingGame<PlainGame>;
    using ReceiveIdleGame = BasicReceivingGame<BasicRecordingIdleGame<true, InboxManager>>;

    static_assert(TickDriverReceiveGame<ReceiveGame> && !TickDriverReceiveGame<PlainGame>,
        "the receiving rig receives and the plain rig does not");
    static_assert(TickDriverReceiveGame<ReceiveIdleGame> && TickDriverIdleGame<ReceiveIdleGame>,
        "the FreeRunning rig's game both receives and idles");

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
        using Driver = SimulationTickDriver<GameT, FakeWorldHost, FakePresenter, FakeEngine>;

        explicit FrameRig(Mode mode)
            : host(events, isAuthority(mode) ? kAuthorityRoleBit : kClientRoleBit, threadingOf(mode), kRingDepthTicks)
            , manager(!isAuthority(mode), events)
            , game(events, manager, storage)
            , presenter(events)
            , driver(game, host, presenter, engine,
                typename Driver::Config{ static_cast<double>(kDt), isAuthority(mode), threadingOf(mode), 60 })
        {
        }

        Events events;
        FakeEngine engine;
        FakeWorldHost host;
        InboxManager manager;
        FakeStorage storage;
        GameT game;
        FakePresenter presenter;
        Driver driver;
    };

    float deltaOf(uint32_t frame)
    {
        constexpr float kFactors[] = { 0.5f, 1.0f, 2.0f, 0.25f, 1.5f };
        return kDt * kFactors[frame % 5u];
    }

    template <typename RigT>
    void runFrames(RigT& rig, uint32_t frames = kFrames)
    {
        for (uint32_t frame = 0u; frame < frames; ++frame)
        {
            const float delta = deltaOf(frame);
            rig.driver.startFrame(static_cast<double>(delta));
            rig.driver.endFrame(nullptr, 0);
            rig.engine.advance(static_cast<double>(delta));
        }
    }

    size_t countOf(const Events& events, const std::string& event)
    {
        return static_cast<size_t>(std::count(events.begin(), events.end(), event));
    }

    bool startsWith(const std::string& event, const char* prefix) { return event.rfind(prefix, 0) == 0; }

    std::vector<Events> stepsOf(const Events& events)
    {
        std::vector<Events> steps;
        for (const std::string& event : events)
        {
            if (event == "receiveForStep")
                steps.emplace_back();
            if (!steps.empty())
                steps.back().push_back(event);
        }
        return steps;
    }

    size_t firstIndexWithPrefix(const Events& events, const char* prefix)
    {
        for (size_t index = 0u; index < events.size(); ++index)
        {
            if (startsWith(events[index], prefix))
                return index;
        }
        return events.size();
    }

    Events withoutReceives(const Events& events)
    {
        Events kept;
        for (const std::string& event : events)
        {
            if (event != "receiveForStep")
                kept.push_back(event);
        }
        return kept;
    }
} // namespace simulationTickDriverReceiveTest

using namespace simulationTickDriverReceiveTest;

TEST_CASE("SimulationTickDriver.ReceiveForStep.WhatAClientStepReceivesIsCollectedByThatSameStep", "[TickDriver]")
{
    FrameRig<ReceiveGame> rig(Mode::FrameBoundedClient);
    rig.game.onReceive = [&rig]() { rig.manager.inbox = rig.game.receiveForStepCalls; };
    runFrames(rig);
    rig.driver.stop();

    REQUIRE(rig.manager.collected.size() > kFrames / 2u);
    CHECK(rig.game.receiveForStepCalls == rig.manager.collected.size());
    for (size_t step = 0u; step < rig.manager.collected.size(); ++step)
    {
        CAPTURE(step, rig.manager.collected[step].tick);
        CHECK(rig.manager.collected[step].tick == static_cast<SimTick>(step + 1u));
        CHECK(rig.manager.collected[step].token == static_cast<uint32_t>(step + 1u));
    }
}

TEST_CASE("SimulationTickDriver.ReceiveForStep.ACorrectionReceivedForAStepIsReplayedByThatSameStep", "[TickDriver]")
{
    constexpr uint32_t kCorrectedStep = 40u;
    constexpr uint32_t kReplayTicks = 3u;
    FrameRig<ReceiveGame> rig(Mode::FrameBoundedClient);
    SimTick anchor = 0u;
    rig.game.onReceive = [&rig, &anchor]()
    {
        if (rig.game.receiveForStepCalls != kCorrectedStep)
            return;
        anchor = rig.manager.getClientClock().getPredictionTick() - kReplayTicks;
        rig.manager.pendingAnchor = anchor;
    };
    runFrames(rig);
    rig.driver.stop();

    REQUIRE(anchor == kCorrectedStep - 1u - kReplayTicks);
    const std::vector<Events> steps = stepsOf(rig.events);
    REQUIRE(steps.size() > kCorrectedStep);
    for (size_t index = 0u; index < steps.size(); ++index)
    {
        CAPTURE(index);
        const bool corrected = index + 1u == kCorrectedStep;
        CHECK(countOf(steps[index], tickEvent("prepare", anchor)) == (corrected ? 1u : 0u));
        CHECK(countOf(steps[index], "simResim") == (corrected ? kReplayTicks : 0u));
    }
    const Events& corrected = steps[kCorrectedStep - 1u];
    CHECK(firstIndexWithPrefix(corrected, "checkSimilar") < firstIndexWithPrefix(corrected, "prepare:"));
    CHECK(firstIndexWithPrefix(corrected, "simResim") < firstIndexWithPrefix(corrected, "collect:"));
}

TEST_CASE("SimulationTickDriver.ReceiveForStep.RunsOncePerClientStepFirstAndNeverOnTheAuthority", "[TickDriver]")
{
    const Mode mode = GENERATE(Mode::InlineAuthority, Mode::FrameBoundedClient);
    CAPTURE(nameOf(mode));
    FrameRig<ReceiveGame> rig(mode);
    runFrames(rig);
    rig.driver.stop();

    const size_t steps = countOf(rig.events, "onStepStarted");
    REQUIRE(steps > kFrames / 2u);
    if (isAuthority(mode))
    {
        CHECK(rig.game.receiveForStepCalls == 0u);
        CHECK(countOf(rig.events, "receiveForStep") == 0u);
        CHECK(rig.game.releasedTicks.size() == steps);
        return;
    }
    CHECK(rig.game.receiveForStepCalls == steps);
    CHECK(countOf(rig.events, "receiveForStep") == steps);
    for (const Events& step : stepsOf(rig.events))
    {
        CHECK(countOf(step, "receiveForStep") == 1u);
        CHECK(countOf(step, "checkSimilar") == 1u);
        CHECK(firstIndexWithPrefix(step, "checkSimilar") == 1u);
        CHECK(firstIndexWithPrefix(step, "captureCorrection:") > 1u);
        CHECK(firstIndexWithPrefix(step, "collect:") < firstIndexWithPrefix(step, "onStepStarted"));
    }
}

TEST_CASE("SimulationTickDriver.ReceiveForStep.AFreeRunningAuthorityNeverReceives", "[TickDriver]")
{
    using Driver = SimulationTickDriver<ReceiveIdleGame, FakeWorldHost, FakePresenter, FakeEngine>;

    Events events;
    FakeEngine engine;
    FakeWorldHost host(events, kAuthorityRoleBit, StepThreading::FreeRunning, kRingDepthTicks);
    InboxManager manager(false, events);
    FakeStorage storage;
    ReceiveIdleGame game(events, manager, storage, engine);
    FakePresenter presenter(events);
    FakeRunnerHost runnerHost(engine, &events);
    std::atomic<bool> stop{ false };
    Driver driver(game, host, presenter, engine, Driver::Config{ static_cast<double>(kDt), true, StepThreading::FreeRunning, 60 });
    game.setStopAfterSteps(stop, 60u);

    driver.runAuthorityLoop(runnerHost, stop);
    driver.stop();

    CHECK(presenter.published.size() == 60u);
    CHECK(game.receiveForStepCalls == 0u);
    CHECK(countOf(events, "receiveForStep") == 0u);
}

TEST_CASE("SimulationTickDriver.ReceiveForStep.AGameThatReceivesHasTheSameTraceWithTheReceivesRemoved", "[TickDriver]")
{
    const Mode mode = GENERATE(Mode::InlineAuthority, Mode::FrameBoundedClient);
    CAPTURE(nameOf(mode));
    FrameRig<PlainGame> plain(mode);
    FrameRig<ReceiveGame> receiving(mode);
    runFrames(plain);
    runFrames(receiving);
    plain.driver.stop();
    receiving.driver.stop();

    CHECK(countOf(plain.events, "receiveForStep") == 0u);
    CHECK(countOf(receiving.events, "receiveForStep") == (isAuthority(mode) ? 0u : receiving.manager.collected.size()));
    CHECK(withoutReceives(receiving.events) == plain.events);
    CHECK(receiving.presenter.published.size() == plain.presenter.published.size());
    CHECK(receiving.engine.lines.size() == plain.engine.lines.size());
}

#endif
