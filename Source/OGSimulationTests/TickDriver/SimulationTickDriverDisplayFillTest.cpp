// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
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

namespace simulationTickDriverDisplayFillTest
{
    using namespace tickDriverFakes;
    using tickDriverThread::kClientRoleBit;

    constexpr float kDt = 1.f / 60.f;
    constexpr uint32_t kRingDepthTicks = 64u;
    constexpr uint32_t kFrames = 60u;
    constexpr auto kGateTimeout = std::chrono::seconds(5);

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
        TestManager() : m_serverClock(kDt) {}

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
                return;
            }
            m_clock.advancePrediction();
            m_lastStep = SimulationTimeStep(m_clock.getPredictionTick(), false, StepKind::Normal, kDt);
        }

        void onPostGameSimulation(const SimulationUpdateInfo&) {}

        unsigned int onCheckIsSimilar() { return std::numeric_limits<unsigned int>::max(); }

        void prepareResimulation(int32_t, uint32_t simTick) { m_clock.startResimulation(simTick); }

        uint32_t currentIntegratedTick() const { return m_lastStep.has_value() ? m_lastStep->getTick() : 0u; }
        const std::optional<SimulationTimeStep>& lastIntegratedStep() const { return m_lastStep; }

    private:
        PredictionClock m_clock;
        ServerTickClock m_serverClock;
        ResimGateProbe m_probe;
        std::optional<SimulationTimeStep> m_lastStep;
    };

    enum class FillSite
    {
        PostPhysics,
        AfterBatch,
    };

    const char* nameOf(FillSite site) { return site == FillSite::PostPhysics ? "post-physics" : "after the batch"; }

    struct FillRecord
    {
        bool stepInFlight = false;
        bool onWorker = false;
        uint64_t stepsEnded = 0u;
    };

    class FillProbe
    {
    public:
        void noteBatch(uint32_t frame, uint32_t steps, bool gated)
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            if (gated)
                m_gateFrameOfStep.emplace_back(m_stepsDispatched + 1u, frame);
            m_stepsDispatched += steps;
            m_batchEnds.push_back(m_stepsDispatched);
        }

        void stepBegan()
        {
            const uint64_t step = m_stepsBegun.fetch_add(1u) + 1u;
            m_currentStep.store(step);
            m_inFlight.fetch_add(1u);
        }

        void stepEnding()
        {
            const uint64_t step = m_currentStep.load();
            std::optional<uint32_t> gateFrame;
            {
                const std::lock_guard<std::mutex> guard(m_mutex);
                for (const auto& [gatedStep, frame] : m_gateFrameOfStep)
                {
                    if (gatedStep == step)
                        gateFrame = frame;
                }
            }
            if (gateFrame.has_value())
            {
                std::unique_lock<std::mutex> lock(m_mutex);
                m_parkedStep = step;
                m_changed.notify_all();
                if (!m_changed.wait_for(lock, kGateTimeout, [&] { return m_releasedFrame >= static_cast<int64_t>(*gateFrame); }))
                    ++m_timeouts;
                m_parkedStep = 0u;
            }
            m_inFlight.fetch_sub(1u);
            m_stepsEnded.fetch_add(1u);
        }

        void waitForParkedStep(uint32_t frame)
        {
            std::optional<uint64_t> firstGatedStep;
            std::unique_lock<std::mutex> lock(m_mutex);
            for (const auto& [gatedStep, gatedFrame] : m_gateFrameOfStep)
            {
                if (gatedFrame == frame)
                    firstGatedStep = gatedStep;
            }
            if (!firstGatedStep.has_value())
                return;
            if (!m_changed.wait_for(lock, kGateTimeout, [&] { return m_parkedStep == *firstGatedStep; }))
                ++m_timeouts;
        }

        void releaseFrame(uint32_t frame)
        {
            {
                const std::lock_guard<std::mutex> guard(m_mutex);
                m_releasedFrame = frame;
            }
            m_changed.notify_all();
        }

        void noteFill(std::thread::id worker)
        {
            const FillRecord record{ m_inFlight.load() != 0u, std::this_thread::get_id() == worker, m_stepsEnded.load() };
            const std::lock_guard<std::mutex> guard(m_mutex);
            m_fills.push_back(record);
        }

        size_t fillCount() const
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            return m_fills.size();
        }

        std::vector<FillRecord> fills() const
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            return m_fills;
        }

        std::vector<uint64_t> batchEnds() const
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            return m_batchEnds;
        }

        size_t gatedBatches() const
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            return m_gateFrameOfStep.size();
        }

        uint32_t timeouts() const
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            return m_timeouts;
        }

    private:
        mutable std::mutex m_mutex;
        std::condition_variable m_changed;
        std::vector<std::pair<uint64_t, uint32_t>> m_gateFrameOfStep;
        std::vector<uint64_t> m_batchEnds;
        std::vector<FillRecord> m_fills;
        uint64_t m_stepsDispatched = 0u;
        uint64_t m_parkedStep = 0u;
        int64_t m_releasedFrame = -1;
        uint32_t m_timeouts = 0u;
        std::atomic<uint64_t> m_stepsBegun{ 0u };
        std::atomic<uint64_t> m_currentStep{ 0u };
        std::atomic<uint64_t> m_stepsEnded{ 0u };
        std::atomic<uint32_t> m_inFlight{ 0u };
    };

    template <FillSite Site>
    class DisplayFillGame
    {
    public:
        using Manager = TestManager;
        using Storage = FakeStorage;
        using RenderSnapshot = FakeRenderSnapshot;

        DisplayFillGame(TestManager& gameManager, FrameWait frameWait, const FakeEngine& engine)
            : m_manager(&gameManager)
            , m_frameWait(frameWait)
            , m_engine(&engine)
        {
        }

        Manager& manager() { return *m_manager; }
        Storage& storage() { return m_storage; }

        void beforePump(double, TickRateDoor&) {}

        void onFrameStepsDue(int32_t, int32_t steps)
        {
            const uint32_t count = static_cast<uint32_t>(steps);
            const bool driverWaitsForIt = m_frameWait == FrameWait::ThisFramesSingleStep && count == 1u;
            probe.noteBatch(frame, count, !driverWaitsForIt);
            stepsThisFrame = count;
        }

        void afterInlineBatch(int32_t) {}
        void releaseInputsForAuthorityTick(SimTick) {}

        void receiveForStep() { probe.stepBegan(); }

        void onStepStarted(double) {}

        void onStepEnded() { probe.stepEnding(); }

        void publishForDisplay()
            requires (Site == FillSite::AfterBatch)
        {
            probe.noteFill(m_engine->workerThreadId());
        }

        void onPostPhysics()
        {
            probe.waitForParkedStep(frame);
            if constexpr (Site == FillSite::PostPhysics)
                probe.noteFill(m_engine->workerThreadId());
            shownAtPostPhysics.push_back(probe.fillCount());
            probe.releaseFrame(frame);
        }

        FillProbe probe;
        uint32_t frame = 0u;
        uint32_t stepsThisFrame = 0u;
        std::vector<size_t> shownAtPostPhysics;

    private:
        TestManager* m_manager;
        FakeStorage m_storage;
        FrameWait m_frameWait;
        const FakeEngine* m_engine;
    };

    template <FillSite Site>
    struct FillRig
    {
        using Game = DisplayFillGame<Site>;
        using Driver = SimulationTickDriver<Game, FakeWorldHost, FakePresenter, FakeEngine>;

        explicit FillRig(FrameWait frameWait)
            : host(events, kClientRoleBit, StepThreading::FrameBounded, kRingDepthTicks)
            , presenter(events)
            , game(manager, frameWait, engine)
            , driver(game, host, presenter, engine, configFor(frameWait))
        {
            engine.startWorkerThread();
        }

        static typename Driver::Config configFor(FrameWait frameWait)
        {
            typename Driver::Config config{ static_cast<double>(kDt), false, StepThreading::FrameBounded, 60 };
            config.frameWait = frameWait;
            return config;
        }

        Events events;
        FakeEngine engine;
        FakeWorldHost host;
        TestManager manager;
        FakePresenter presenter;
        Game game;
        Driver driver;
    };

    float deltaOf(uint32_t frame)
    {
        constexpr float kFactors[] = { 1.0f, 2.0f, 1.0f, 1.0f, 2.0f, 0.5f, 1.5f };
        return kDt * kFactors[frame % 7u];
    }

    struct FrameRecord
    {
        uint32_t steps = 0u;
        size_t batchesThrough = 0u;
        size_t shown = 0u;
    };

    template <typename RigT>
    std::vector<FrameRecord> runFrames(RigT& rig)
    {
        std::vector<FrameRecord> records;
        size_t batches = 0u;
        for (uint32_t frame = 0u; frame < kFrames; ++frame)
        {
            rig.game.frame = frame;
            rig.game.stepsThisFrame = 0u;
            rig.driver.startFrame(static_cast<double>(deltaOf(frame)));
            batches += rig.game.stepsThisFrame > 0u ? 1u : 0u;
            rig.driver.endFrame(nullptr, 0);
            records.push_back(FrameRecord{ rig.game.stepsThisFrame, batches, rig.game.shownAtPostPhysics.back() });
        }
        rig.driver.stop();
        rig.engine.stopWorkerThread();
        return records;
    }

    size_t framesWithSteps(const std::vector<FrameRecord>& records, uint32_t steps)
    {
        return static_cast<size_t>(std::count_if(records.begin(), records.end(),
            [steps](const FrameRecord& record) { return record.steps == steps; }));
    }

    size_t fillsBesideAStep(const std::vector<FillRecord>& fills)
    {
        return static_cast<size_t>(std::count_if(fills.begin(), fills.end(), [](const FillRecord& fill) { return fill.stepInFlight; }));
    }

    const char* nameOf(FrameWait frameWait)
    {
        return frameWait == FrameWait::EarlierFrames ? "EarlierFrames" : "ThisFramesSingleStep";
    }
} // namespace simulationTickDriverDisplayFillTest

using namespace simulationTickDriverDisplayFillTest;

TEST_CASE("SimulationTickDriver.DisplayFill.TheAfterBatchFillNeverRunsBesideAStep", "[TickDriver]")
{
    const FrameWait frameWait = GENERATE(FrameWait::EarlierFrames, FrameWait::ThisFramesSingleStep);
    CAPTURE(nameOf(frameWait));
    FillRig<FillSite::AfterBatch> rig(frameWait);
    const std::vector<FrameRecord> records = runFrames(rig);
    const std::vector<FillRecord> fills = rig.game.probe.fills();
    const std::vector<uint64_t> batchEnds = rig.game.probe.batchEnds();

    REQUIRE(framesWithSteps(records, 1u) > kFrames / 4u);
    REQUIRE(framesWithSteps(records, 2u) > kFrames / 8u);
    REQUIRE(rig.game.probe.gatedBatches() > kFrames / 8u);
    CHECK(rig.game.probe.timeouts() == 0u);
    CHECK(fillsBesideAStep(fills) == 0u);
    CHECK(fills.size() == batchEnds.size());
    for (size_t index = 0u; index < std::min(fills.size(), batchEnds.size()); ++index)
    {
        CAPTURE(index);
        CHECK(fills[index].onWorker);
        CHECK(fills[index].stepsEnded == batchEnds[index]);
    }
    if (frameWait == FrameWait::ThisFramesSingleStep)
    {
        for (size_t frame = 0u; frame < records.size(); ++frame)
        {
            CAPTURE(frame, records[frame].steps);
            if (records[frame].steps == 1u)
                CHECK(records[frame].shown == records[frame].batchesThrough);
        }
    }
}

TEST_CASE("SimulationTickDriver.DisplayFill.AFillAtPostPhysicsRunsBesideAStepTheDriverDoesNotWaitFor", "[TickDriver]")
{
    const FrameWait frameWait = GENERATE(FrameWait::EarlierFrames, FrameWait::ThisFramesSingleStep);
    CAPTURE(nameOf(frameWait));
    FillRig<FillSite::PostPhysics> rig(frameWait);
    const std::vector<FrameRecord> records = runFrames(rig);
    const std::vector<FillRecord> fills = rig.game.probe.fills();

    REQUIRE(framesWithSteps(records, 2u) > kFrames / 8u);
    CHECK(rig.game.probe.timeouts() == 0u);
    REQUIRE(fills.size() == kFrames);
    CHECK(fillsBesideAStep(fills) == rig.game.probe.gatedBatches());
    CHECK(std::none_of(fills.begin(), fills.end(), [](const FillRecord& fill) { return fill.onWorker; }));
}

#endif
