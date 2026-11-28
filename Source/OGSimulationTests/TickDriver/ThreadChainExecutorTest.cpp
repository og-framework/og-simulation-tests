// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
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
#include "OGSimulation/StepHooks.h"
#include "OGTickDriver/ThreadChainExecutor.h"
#include "OGTickDriver/TickDriverConcepts.h"
#include "OGTickDriver/TickDriverThreadContext.h"
#include "TickDriverFakes.h"

namespace threadChainExecutorTest
{
    using namespace tickDriverFakes;
    using stepDriverTestSupport::tickEvent;
    using tickDriverThread::kClientRoleBit;

    constexpr float kDt = 1.f / 60.f;
    constexpr uint32_t kRingDepthTicks = 64u;
    constexpr uint32_t kConformanceFrames = 900u;
    constexpr uint32_t kFifoDispatches = 10000u;
    constexpr int kHighPriority = 1;

    class StdThreadHost
    {
    public:
        StdThreadHost() = default;
        StdThreadHost(const StdThreadHost&) = delete;
        StdThreadHost& operator=(const StdThreadHost&) = delete;

        ~StdThreadHost() { joinThreads(); }

        void startThread(const char* name, int priority, void (*fn)(void*), void* arg)
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            names.emplace_back(name != nullptr ? name : "");
            priorities.push_back(priority);
            m_threads.emplace_back([this, fn, arg]
                {
                    {
                        const std::lock_guard<std::mutex> started(m_mutex);
                        threadIds.push_back(std::this_thread::get_id());
                    }
                    fn(arg);
                    returned.fetch_add(1u);
                });
        }

        void joinThreads()
        {
            std::vector<std::thread> threads;
            {
                const std::lock_guard<std::mutex> guard(m_mutex);
                threads.swap(m_threads);
            }
            for (std::thread& thread : threads)
                thread.join();
        }

        std::vector<std::thread::id> startedThreadIds()
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            return threadIds;
        }

        std::vector<std::string> names;
        std::vector<int> priorities;
        std::atomic<uint32_t> returned{ 0u };

    private:
        std::mutex m_mutex;
        std::vector<std::thread> m_threads;
        std::vector<std::thread::id> threadIds;
    };

    static_assert(ExecutorThreadHost<StdThreadHost>, "StdThreadHost must model ExecutorThreadHost");

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

    template <typename ExecutorT>
    class ConformanceEngine
    {
    public:
        static constexpr bool kHostDiagnostics = true;
        using Handle = typename ExecutorT::Handle;
        using Completion = typename ExecutorT::Completion;

        ConformanceEngine(Events& events, ExecutorT& executor)
            : m_events(&events)
            , m_executor(&executor)
            , m_frameThread(std::this_thread::get_id())
        {
        }

        double nowSeconds() const { return now; }
        uint64_t frameNumber() const { return frame; }
        bool logEnabled(HostLogLevel) const { return true; }
        void log(HostLogLevel level, const char* line) { m_events->push_back(std::string(levelTag(level)) + line); }

        bool isPending(const Handle& handle) const { return m_executor->isPending(handle); }

        void waitFor(const Handle& handle)
        {
            ++waits;
            {
                const std::lock_guard<std::mutex> gate(m_gateMutex);
                m_released = dispatches;
            }
            m_gateChanged.notify_all();
            m_executor->waitFor(handle);
        }

        template <typename F>
        Handle dispatchAfter(const Handle& previous, F&& work)
        {
            const uint64_t dispatch = ++dispatches;
            if (m_executor->isPending(previous))
                ++chainedDispatches;
            m_events->push_back("dispatch:" + std::to_string(dispatch));
            return m_executor->dispatchAfter(previous,
                [this, dispatch, body = std::function<void()>(std::forward<F>(work))]()
                {
                    {
                        std::unique_lock<std::mutex> gate(m_gateMutex);
                        m_gateChanged.wait(gate, [this, dispatch] { return m_released >= dispatch; });
                    }
                    m_events->push_back("task:" + std::to_string(dispatch));
                    if (std::this_thread::get_id() == m_frameThread)
                        tasksOnFrameThread.fetch_add(1u);
                    body();
                });
        }

        template <typename F>
        bool deferUntil(Completion& completion, const Handle& handle, F&& continuation)
        {
            ++deferralCalls;
            const bool held = m_executor->deferUntil(completion, handle, std::forward<F>(continuation));
            if (held)
                ++deferralsHeld;
            return held;
        }

        double now = 0.0;
        uint64_t frame = 0u;
        uint64_t dispatches = 0u;
        uint64_t chainedDispatches = 0u;
        uint64_t waits = 0u;
        uint64_t deferralCalls = 0u;
        uint64_t deferralsHeld = 0u;
        std::atomic<uint64_t> tasksOnFrameThread{ 0u };

    private:
        Events* m_events;
        ExecutorT* m_executor;
        std::thread::id m_frameThread;
        std::mutex m_gateMutex;
        std::condition_variable m_gateChanged;
        uint64_t m_released = 0u;
    };

    static_assert(TickDriverEngine<ConformanceEngine<ThreadChainExecutor>>,
        "ConformanceEngine over a ThreadChainExecutor must model TickDriverEngine");
    static_assert(TickDriverEngine<ConformanceEngine<FakeEngine>>,
        "ConformanceEngine over the fakes' FakeEngine must model TickDriverEngine");

    using Game = BasicRecordingGame<TestManager>;

    template <typename ExecutorT>
    using Driver = SimulationTickDriver<Game, FakeWorldHost, FakePresenter, ConformanceEngine<ExecutorT>>;

    struct Trace
    {
        Events events;
        std::vector<double> beforePumpHostSeconds;
        std::vector<double> stepStartedSeconds;
        std::vector<double> publishedDeadlines;
        std::vector<SimTick> publishedTicks;
        std::vector<uint64_t> capturedSteps;
        std::vector<uint64_t> afterRunTickSteps;
        uint64_t physicsSteps = 0u;
        uint64_t dispatches = 0u;
        uint64_t chainedDispatches = 0u;
        uint64_t waits = 0u;
        uint64_t deferralCalls = 0u;
        uint64_t deferralsHeld = 0u;
        uint64_t tasksOnFrameThread = 0u;
    };

    float frameDelta(uint32_t frame)
    {
        if (frame == 0u)
            return 1.5f * kDt;
        if (frame == 450u)
            return 0.25f;
        return kDt * (0.4f + 0.4f * static_cast<float>((frame * 7u) % 6u));
    }

    template <typename ExecutorT>
    Trace runFrameBoundedClient(ExecutorT& executor)
    {
        Trace trace;
        ConformanceEngine<ExecutorT> engine(trace.events, executor);
        FakeWorldHost host(trace.events, kClientRoleBit, false, kRingDepthTicks);
        TestManager manager(trace.events);
        FakeStorage storage;
        Game game(trace.events, manager, storage);
        FakePresenter presenter(trace.events);
        {
            Driver<ExecutorT> driver(game, host, presenter, engine,
                typename Driver<ExecutorT>::Config{ static_cast<double>(kDt), false, StepThreading::FrameBounded, 60 });
            for (uint32_t frame = 0u; frame < kConformanceFrames; ++frame)
            {
                const float delta = frameDelta(frame);
                engine.now += static_cast<double>(delta);
                ++engine.frame;
                if (frame % 23u == 11u && manager.predictionTick() > 6u)
                    manager.pendingAnchor = manager.predictionTick() - 3u;
                if (frame % 31u == 5u)
                    driver.pushBodySlotOccupancy(1u + frame % 3u, frame % 62u < 31u);
                if (frame % 13u == 2u)
                    driver.noteShadowRestore(0.00005 * static_cast<double>(1u + frame % 4u), frame % 26u == 2u);

                driver.startFrame(static_cast<double>(delta));

                typename ExecutorT::Completion completion;
                driver.endFrame(frame % 3u == 0u ? nullptr : &completion, 1);
            }
            trace.events.push_back("stop");
            driver.stop();
            trace.physicsSteps = driver.getDiagnostics().physicsStepCount();
        }
        trace.beforePumpHostSeconds = game.beforePumpHostSeconds;
        trace.stepStartedSeconds = game.stepStartedSeconds;
        trace.publishedDeadlines = presenter.publishedDeadlines;
        for (const TickOutcome& outcome : presenter.published)
            trace.publishedTicks.push_back(outcome.tick);
        trace.capturedSteps = presenter.capturedSteps;
        for (const auto& step : host.afterRunTickSteps)
            trace.afterRunTickSteps.push_back(step.first);
        trace.dispatches = engine.dispatches;
        trace.chainedDispatches = engine.chainedDispatches;
        trace.waits = engine.waits;
        trace.deferralCalls = engine.deferralCalls;
        trace.deferralsHeld = engine.deferralsHeld;
        trace.tasksOnFrameThread = engine.tasksOnFrameThread.load();
        return trace;
    }

    std::string firstDifference(const Events& candidate, const Events& reference)
    {
        const size_t common = std::min(candidate.size(), reference.size());
        for (size_t index = 0u; index < common; ++index)
        {
            if (candidate[index] == reference[index])
                continue;
            std::string report = "event " + std::to_string(index) + ": executor '" + candidate[index] + "' vs fake '"
                + reference[index] + "'; before it:";
            for (size_t back = index >= 4u ? index - 4u : 0u; back < index; ++back)
                report += " '" + candidate[back] + "'";
            return report;
        }
        if (candidate.size() != reference.size())
            return "lengths differ: executor " + std::to_string(candidate.size()) + ", fake " + std::to_string(reference.size());
        return "";
    }

    size_t countPrefix(const Events& events, const std::string& prefix)
    {
        return static_cast<size_t>(std::count_if(events.begin(), events.end(),
            [&prefix](const std::string& event) { return event.rfind(prefix, 0) == 0; }));
    }

    bool bitEqual(const std::vector<double>& a, const std::vector<double>& b)
    {
        if (a.size() != b.size())
            return false;
        for (size_t index = 0u; index < a.size(); ++index)
        {
            if (std::bit_cast<uint64_t>(a[index]) != std::bit_cast<uint64_t>(b[index]))
                return false;
        }
        return true;
    }
TEST_CASE("ThreadChainExecutor.Host.StartsOneWorkerThroughTheHostWithItsNameAndPriority", "[TickDriver]")
{
    StdThreadHost host;
    {
        ThreadChainExecutor executor(host, "OGSimChain", kHighPriority);
        std::thread::id ranOn;
        const ThreadChainExecutor::Handle task =
            executor.dispatchAfter(ThreadChainExecutor::Handle{}, [&ranOn] { ranOn = std::this_thread::get_id(); });
        executor.waitFor(task);

        CHECK_FALSE(executor.isPending(task));
        REQUIRE(host.names.size() == 1u);
        CHECK(host.names[0] == "OGSimChain");
        CHECK(host.priorities == std::vector<int>{ kHighPriority });
        const std::vector<std::thread::id> started = host.startedThreadIds();
        REQUIRE(started.size() == 1u);
        CHECK(ranOn == started[0]);
        CHECK(ranOn != std::this_thread::get_id());
    }
    host.joinThreads();
    CHECK(host.returned.load() == 1u);
}

TEST_CASE("ThreadChainExecutor.Fifo.TenThousandDispatchesRunInOrderOneAtATime", "[TickDriver]")
{
    StdThreadHost host;
    std::vector<uint32_t> order(kFifoDispatches, std::numeric_limits<uint32_t>::max());
    std::atomic<uint32_t> nextSlot{ 0u };
    std::atomic<uint32_t> running{ 0u };
    std::atomic<uint32_t> overlaps{ 0u };
    std::atomic<uint32_t> offWorker{ 0u };
    std::vector<ThreadChainExecutor::Handle> handles;
    handles.reserve(kFifoDispatches);
    {
        ThreadChainExecutor executor(host, "OGSimChain", kHighPriority);
        const std::vector<std::thread::id> started = [&host]
        {
            for (;;)
            {
                const std::vector<std::thread::id> ids = host.startedThreadIds();
                if (!ids.empty())
                    return ids;
                std::this_thread::yield();
            }
        }();
        const std::thread::id worker = started.front();

        ThreadChainExecutor::Handle previous;
        for (uint32_t index = 0u; index < kFifoDispatches; ++index)
        {
            const ThreadChainExecutor::Handle after = index % 2u == 0u ? previous : ThreadChainExecutor::Handle{};
            previous = executor.dispatchAfter(after, [&, index]
                {
                    if (running.fetch_add(1u) != 0u)
                        overlaps.fetch_add(1u);
                    if (std::this_thread::get_id() != worker)
                        offWorker.fetch_add(1u);
                    if (index % 97u == 0u)
                        std::this_thread::yield();
                    order[nextSlot.fetch_add(1u)] = index;
                    running.fetch_sub(1u);
                });
            handles.push_back(previous);
        }
        executor.waitFor(handles.back());

        for (const ThreadChainExecutor::Handle& handle : handles)
            CHECK_FALSE(executor.isPending(handle));
    }

    CHECK(nextSlot.load() == kFifoDispatches);
    CHECK(overlaps.load() == 0u);
    CHECK(offWorker.load() == 0u);
    uint32_t outOfOrder = 0u;
    for (uint32_t index = 0u; index < kFifoDispatches; ++index)
    {
        if (order[index] != index)
            ++outOfOrder;
    }
    CHECK(outOfOrder == 0u);
    for (uint32_t index = 1u; index < kFifoDispatches; ++index)
        REQUIRE(handles[index].ticket == handles[index - 1u].ticket + 1u);
}

TEST_CASE("ThreadChainExecutor.Stop.JoinsAfterTheWorkerRanEveryQueuedTask", "[TickDriver]")
{
    StdThreadHost host;
    std::atomic<uint32_t> ran{ 0u };
    std::atomic<bool> firstFinished{ false };
    std::vector<ThreadChainExecutor::Handle> handles;
    {
        ThreadChainExecutor executor(host, "OGSimChain", kHighPriority);
        handles.push_back(executor.dispatchAfter(ThreadChainExecutor::Handle{}, [&]
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                firstFinished.store(true);
                ran.fetch_add(1u);
            }));
        for (uint32_t index = 1u; index < 100u; ++index)
            handles.push_back(executor.dispatchAfter(handles.back(), [&ran] { ran.fetch_add(1u); }));

        executor.stop();

        CHECK(firstFinished.load());
        CHECK(ran.load() == 100u);
        for (const ThreadChainExecutor::Handle& handle : handles)
            CHECK_FALSE(executor.isPending(handle));
        executor.waitFor(handles.back());

        executor.stop();
        CHECK(ran.load() == 100u);
    }
    host.joinThreads();
    CHECK(host.returned.load() == 1u);
}

TEST_CASE("ThreadChainExecutor.Wait.ReturnsOnlyAfterTheTaskRanAndItsCapturesAreDestroyed", "[TickDriver]")
{
    StdThreadHost host;
    ThreadChainExecutor executor(host, "OGSimChain", kHighPriority);
    std::atomic<bool> finished{ false };
    auto token = std::make_shared<int>(7);
    const std::weak_ptr<int> watch = token;
    const ThreadChainExecutor::Handle task = executor.dispatchAfter(ThreadChainExecutor::Handle{},
        [&finished, token = std::move(token)]
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            finished.store(*token == 7);
        });

    executor.waitFor(task);

    CHECK(finished.load());
    CHECK(watch.expired());
    CHECK_FALSE(executor.isPending(task));
    executor.stop();
}

TEST_CASE("ThreadChainExecutor.Deferral.DeferUntilNeverHoldsTheContinuation", "[TickDriver]")
{
    StdThreadHost host;
    ThreadChainExecutor executor(host, "OGSimChain", kHighPriority);
    std::atomic<bool> release{ false };
    const ThreadChainExecutor::Handle blocked = executor.dispatchAfter(ThreadChainExecutor::Handle{}, [&release]
        {
            while (!release.load())
                std::this_thread::yield();
        });

    ThreadChainExecutor::Completion completion;
    bool continued = false;
    CHECK(executor.isPending(blocked));
    CHECK_FALSE(executor.deferUntil(completion, blocked, [&continued] { continued = true; }));
    CHECK_FALSE(executor.deferUntil(completion, ThreadChainExecutor::Handle{}, [&continued] { continued = true; }));

    release.store(true);
    executor.waitFor(blocked);
    executor.stop();
    CHECK_FALSE(continued);
}

TEST_CASE("ThreadChainExecutor.Conformance.AFrameBoundedClientGivesTheFakeChainedExecutorsStepTrace", "[TickDriver]")
{
    FakeEngine fake;
    fake.deferralSupported = false;
    fake.startWorkerThread();
    const Trace reference = runFrameBoundedClient(fake);
    fake.stopWorkerThread();

    StdThreadHost host;
    Trace candidate;
    {
        ThreadChainExecutor executor(host, "OGSimChain", kHighPriority);
        candidate = runFrameBoundedClient(executor);
        executor.stop();
    }

    CHECK(firstDifference(candidate.events, reference.events) == "");
    CHECK(bitEqual(candidate.beforePumpHostSeconds, reference.beforePumpHostSeconds));
    CHECK(bitEqual(candidate.stepStartedSeconds, reference.stepStartedSeconds));
    CHECK(bitEqual(candidate.publishedDeadlines, reference.publishedDeadlines));
    CHECK(candidate.publishedTicks == reference.publishedTicks);
    CHECK(candidate.capturedSteps == reference.capturedSteps);
    CHECK(candidate.afterRunTickSteps == reference.afterRunTickSteps);
    CHECK(candidate.physicsSteps == reference.physicsSteps);
    CHECK(candidate.dispatches == reference.dispatches);
    CHECK(candidate.chainedDispatches == reference.chainedDispatches);
    CHECK(candidate.waits == reference.waits);
    CHECK(candidate.deferralCalls == reference.deferralCalls);
    CHECK(candidate.deferralsHeld == 0u);
    CHECK(reference.deferralsHeld == 0u);

    CHECK(candidate.tasksOnFrameThread == 0u);
    CHECK(reference.tasksOnFrameThread == 0u);
    CHECK(candidate.dispatches > 400u);
    CHECK(candidate.waits == candidate.dispatches + 2u);
    CHECK(candidate.deferralCalls > 0u);
    CHECK(countPrefix(candidate.events, "task:") == candidate.dispatches);
    CHECK(countPrefix(candidate.events, "simResim") > 0u);
    CHECK(countPrefix(candidate.events, "captureCorrection:") > 0u);
    CHECK(countPrefix(candidate.events, "log:W:[SimHost.FirstStep]") == 1u);
    CHECK(countPrefix(candidate.events, "log:W:[SimHost.Window]") >= 2u);
    CHECK(host.names.size() == 1u);
}

TEST_CASE("ThreadChainExecutor.Death.AHandleFromAnotherExecutorFiresTheCheck", "[.][OGSimDeath]")
{
    StdThreadHost host;
    ThreadChainExecutor first(host, "OGSimChainA", kHighPriority);
    ThreadChainExecutor second(host, "OGSimChainB", kHighPriority);
    const ThreadChainExecutor::Handle task = first.dispatchAfter(ThreadChainExecutor::Handle{}, [] {});
    first.waitFor(task);
    [[maybe_unused]] const bool pending = second.isPending(task);
    FAIL("a handle issued by another executor did NOT fire ThreadChainExecutor's issued-here OG_CHECK");
}

TEST_CASE("ThreadChainExecutor.Death.ADispatchAfterStopFiresTheCheck", "[.][OGSimDeath]")
{
    StdThreadHost host;
    ThreadChainExecutor executor(host, "OGSimChain", kHighPriority);
    executor.stop();
    executor.dispatchAfter(ThreadChainExecutor::Handle{}, [] {});
    FAIL("a dispatch after stop did NOT fire ThreadChainExecutor's dispatch-after-stop OG_CHECK");
}

} // namespace threadChainExecutorTest

#endif
