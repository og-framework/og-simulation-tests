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
#include "OGSimulation/SimulationTickDriver.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/TickDriverConcepts.h"
#include "OGSimulation/TickDriverThreadContext.h"
#include "TickDriverFakes.h"

namespace simulationTickDriverFrameWaitTest
{
    using namespace tickDriverFakes;
    using tickDriverThread::kClientRoleBit;

    constexpr float kDt = 1.f / 60.f;
    constexpr uint32_t kRingDepthTicks = 64u;
    constexpr uint32_t kFrames = 120u;

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
        explicit TestManager(Events& events)
            : m_events(&events)
            , m_serverClock(kDt)
        {
        }

        bool runsPrediction() const { return true; }
        PredictionClock& editClientClock() { return m_clock; }
        const PredictionClock& getClientClock() const { return m_clock; }
        const ServerTickClock& getServerClock() const { return m_serverClock; }
        ResimGateProbe& editResimGateProbe() { return m_probe; }

        void onGameSimulation(const SimulationUpdateInfo& info)
        {
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
        Events* m_events;
        PredictionClock m_clock;
        ServerTickClock m_serverClock;
        ResimGateProbe m_probe;
        std::optional<SimulationTimeStep> m_lastStep;
    };

    class ShowingGame : public BasicRecordingGame<TestManager>
    {
    public:
        using Base = BasicRecordingGame<TestManager>;

        ShowingGame(Events& events, TestManager& gameManager, FakeStorage& gameStorage, const FakePresenter& presenter)
            : Base(events, gameManager, gameStorage)
            , m_presenter(&presenter)
        {
        }

        void onPostPhysics()
        {
            Base::onPostPhysics();
            shownAtPostPhysics.push_back(m_presenter->published.size());
        }

        std::vector<size_t> shownAtPostPhysics;

    private:
        const FakePresenter* m_presenter;
    };

    enum class Ending
    {
        DeferredToTheCompletionEvent,
        BlockingWait,
    };

    const char* nameOf(Ending ending)
    {
        return ending == Ending::DeferredToTheCompletionEvent ? "deferred to the completion event" : "blocking wait";
    }

    struct FrameRig
    {
        using Driver = SimulationTickDriver<ShowingGame, FakeWorldHost, FakePresenter, FakeEngine>;

        explicit FrameRig(FrameWait frameWait)
            : host(events, kClientRoleBit, StepThreading::FrameBounded, kRingDepthTicks)
            , manager(events)
            , presenter(events)
            , game(events, manager, storage, presenter)
            , driver(game, host, presenter, engine, configFor(frameWait))
        {
        }

        static Driver::Config configFor(FrameWait frameWait)
        {
            Driver::Config config{ static_cast<double>(kDt), false, StepThreading::FrameBounded, 60 };
            config.frameWait = frameWait;
            return config;
        }

        Events events;
        FakeEngine engine;
        FakeWorldHost host;
        TestManager manager;
        FakeStorage storage;
        FakePresenter presenter;
        ShowingGame game;
        Driver driver;
    };

    struct FrameRecord
    {
        uint32_t steps = 0u;
        size_t dispatched = 0u;
        size_t shown = 0u;
    };

    float deltaOf(uint32_t frame)
    {
        constexpr float kFactors[] = { 0.6f, 1.0f, 2.0f, 0.3f, 1.0f, 0.6f };
        return kDt * kFactors[frame % 6u];
    }

    std::vector<FrameRecord> runFrames(FrameRig& rig, Ending ending)
    {
        std::vector<FrameRecord> records;
        size_t dispatched = 0u;
        for (uint32_t frame = 0u; frame < kFrames; ++frame)
        {
            const size_t dueBefore = rig.game.frameStepsDue.size();
            const size_t shownBefore = rig.game.shownAtPostPhysics.size();
            rig.driver.startFrame(static_cast<double>(deltaOf(frame)));
            const uint32_t steps = rig.game.frameStepsDue.size() > dueBefore
                ? static_cast<uint32_t>(rig.game.frameStepsDue.back().second) : 0u;
            dispatched += steps;

            FakeCompletion completion;
            rig.driver.endFrame(ending == Ending::DeferredToTheCompletionEvent ? &completion : nullptr, 0);
            rig.engine.runAllTasks();
            rig.engine.runDeferred();

            REQUIRE(rig.game.shownAtPostPhysics.size() == shownBefore + 1u);
            records.push_back(FrameRecord{ steps, dispatched, rig.game.shownAtPostPhysics.back() });
            rig.engine.advance(static_cast<double>(deltaOf(frame)));
        }
        rig.driver.stop();
        return records;
    }

    size_t framesWithSteps(const std::vector<FrameRecord>& records, uint32_t steps)
    {
        return static_cast<size_t>(std::count_if(records.begin(), records.end(),
            [steps](const FrameRecord& record) { return record.steps == steps; }));
    }

    bool loggedFrameWait(const FakeEngine& engine)
    {
        return std::any_of(engine.lines.begin(), engine.lines.end(), [](const LoggedLine& line)
        {
            return line.text.find("[SimHost.Frame] ") == 0u && line.text.find(" frameWait=thisFramesSingleStep") != std::string::npos;
        });
    }
} // namespace simulationTickDriverFrameWaitTest

using namespace simulationTickDriverFrameWaitTest;

TEST_CASE("SimulationTickDriver.FrameWait.AStepPublishedMidFrameIsShownByThatFramesPostPhysics", "[TickDriver]")
{
    const Ending ending = GENERATE(Ending::DeferredToTheCompletionEvent, Ending::BlockingWait);
    CAPTURE(nameOf(ending));
    FrameRig rig(FrameWait::ThisFramesSingleStep);
    const std::vector<FrameRecord> records = runFrames(rig, ending);

    REQUIRE(framesWithSteps(records, 0u) > 0u);
    REQUIRE(framesWithSteps(records, 1u) > kFrames / 3u);
    REQUIRE(framesWithSteps(records, 2u) > 0u);
    for (size_t frame = 0u; frame < records.size(); ++frame)
    {
        CAPTURE(frame, records[frame].steps, records[frame].dispatched);
        if (records[frame].steps <= 1u)
            CHECK(records[frame].shown == records[frame].dispatched);
        else
            CHECK(records[frame].shown == records[frame].dispatched - records[frame].steps);
    }
    CHECK(loggedFrameWait(rig.engine));
}

TEST_CASE("SimulationTickDriver.FrameWait.TheDefaultShowsAFramesStepsInTheNextFrameAsBlockModeZeroDid", "[TickDriver]")
{
    const Ending ending = GENERATE(Ending::DeferredToTheCompletionEvent, Ending::BlockingWait);
    CAPTURE(nameOf(ending));
    FrameRig rig(FrameWait::EarlierFrames);
    const std::vector<FrameRecord> records = runFrames(rig, ending);

    REQUIRE(framesWithSteps(records, 1u) > kFrames / 3u);
    for (size_t frame = 0u; frame < records.size(); ++frame)
    {
        CAPTURE(frame, records[frame].steps, records[frame].dispatched);
        CHECK(records[frame].shown == records[frame].dispatched - records[frame].steps);
    }
    CHECK_FALSE(loggedFrameWait(rig.engine));
}

TEST_CASE("SimulationTickDriver.FrameWait.TheSettingAddsNoEventToTheTraceAndOnlyMovesThePostPhysicsPass", "[TickDriver]")
{
    FrameRig earlier(FrameWait::EarlierFrames);
    FrameRig thisFrame(FrameWait::ThisFramesSingleStep);
    runFrames(earlier, Ending::DeferredToTheCompletionEvent);
    runFrames(thisFrame, Ending::DeferredToTheCompletionEvent);

    Events earlierSorted = earlier.events;
    Events thisFrameSorted = thisFrame.events;
    std::sort(earlierSorted.begin(), earlierSorted.end());
    std::sort(thisFrameSorted.begin(), thisFrameSorted.end());
    CHECK(thisFrameSorted == earlierSorted);
    CHECK(thisFrame.presenter.published.size() == earlier.presenter.published.size());
    CHECK(thisFrame.game.frameStepsDue == earlier.game.frameStepsDue);
    CHECK(thisFrame.events != earlier.events);
}

#endif
