// SPDX-License-Identifier: MPL-2.0
// docs/TickDriverFakes-rationale.md

#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "glm/gtc/quaternion.hpp"
#include "glm/vec3.hpp"
#include "OGSimulation/QueryGeometry.h"
#include "OGTickDriver/SimulationTickBackend.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGSimulation/StepHooks.h"
#include "OGTickDriver/TickDriverConcepts.h"
#include "OGTickDriver/TickDriverLog.h"
#include "OGTickDriver/TickDriverThreadContext.h"
#include "../StepDriver/StepDriverTestSupport.h"

namespace tickDriverFakes
{
    using stepDriverTestSupport::Events;
    using stepDriverTestSupport::LoggedWorld;

    struct LoggedLine
    {
        HostLogLevel level = HostLogLevel::Log;
        std::string text;
    };

    struct FakeTask
    {
        uint64_t id = 0;
        std::function<void()> work;
        bool done = false;
    };

    struct FakeCompletion
    {
        std::vector<std::function<void()>> continuations;
    };

    template <bool HostDiagnostics>
    class BasicFakeEngine
    {
    public:
        static constexpr bool kHostDiagnostics = HostDiagnostics;
        using Handle = std::shared_ptr<FakeTask>;
        using Completion = FakeCompletion;

        BasicFakeEngine() = default;
        BasicFakeEngine(const BasicFakeEngine&) = delete;
        BasicFakeEngine& operator=(const BasicFakeEngine&) = delete;

        ~BasicFakeEngine() { stopWorkerThread(); }

        double nowSeconds() const { return now; }
        uint64_t frameNumber() const { return frame; }

        void advance(double seconds) { now += seconds; }
        void advanceFrame() { ++frame; }

        bool logEnabled(HostLogLevel level) const { return static_cast<int>(level) >= static_cast<int>(minimumLevel); }

        void log(HostLogLevel level, const char* line)
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            lines.push_back(LoggedLine{ level, line });
        }

        bool isPending(const Handle& handle) const
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            return handle != nullptr && !handle->done;
        }

        template <typename F>
        Handle dispatchAfter(const Handle& previous, F&& work)
        {
            Handle task = std::make_shared<FakeTask>();
            {
                const std::lock_guard<std::mutex> guard(m_mutex);
                task->id = ++m_nextTaskId;
                task->work = std::function<void()>(std::forward<F>(work));
                dispatchedAfter.push_back(previous != nullptr && !previous->done ? previous->id : 0u);
                m_queue.push_back(task);
            }
            m_changed.notify_all();
            return task;
        }

        void waitFor(const Handle& handle)
        {
            {
                const std::lock_guard<std::mutex> guard(m_mutex);
                waits.push_back(handle != nullptr ? handle->id : 0u);
            }
            if (handle == nullptr)
                return;
            if (!m_worker.joinable())
            {
                while (!handle->done && runNextTask())
                {
                }
                return;
            }
            std::unique_lock<std::mutex> lock(m_mutex);
            m_changed.wait(lock, [&handle] { return handle->done; });
        }

        template <typename F>
        bool deferUntil(Completion& completion, const Handle& handle, F&& continuation)
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            deferrals.push_back(handle != nullptr ? handle->id : 0u);
            if (!deferralSupported)
                return false;
            m_deferred.push_back(Deferred{ &completion, handle, std::function<void()>(std::forward<F>(continuation)) });
            return true;
        }

        bool runNextTask()
        {
            Handle task;
            {
                const std::lock_guard<std::mutex> guard(m_mutex);
                if (m_queue.empty())
                    return false;
                task = m_queue.front();
                m_queue.pop_front();
            }
            task->work();
            {
                const std::lock_guard<std::mutex> guard(m_mutex);
                task->done = true;
                ++tasksRun;
            }
            m_changed.notify_all();
            return true;
        }

        uint32_t runAllTasks()
        {
            uint32_t run = 0u;
            while (runNextTask())
                ++run;
            return run;
        }

        size_t queuedTasks() const
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            return m_queue.size();
        }

        uint32_t runDeferred()
        {
            std::vector<Deferred> ready;
            {
                const std::lock_guard<std::mutex> guard(m_mutex);
                for (auto it = m_deferred.begin(); it != m_deferred.end();)
                {
                    if (it->after == nullptr || it->after->done)
                    {
                        ready.push_back(std::move(*it));
                        it = m_deferred.erase(it);
                    }
                    else
                    {
                        ++it;
                    }
                }
            }
            for (Deferred& deferred : ready)
            {
                deferred.completion->continuations.push_back(deferred.continuation);
                deferred.continuation();
            }
            return static_cast<uint32_t>(ready.size());
        }

        size_t pendingDeferrals() const
        {
            const std::lock_guard<std::mutex> guard(m_mutex);
            return m_deferred.size();
        }

        void startWorkerThread()
        {
            if (m_worker.joinable())
                return;
            m_stopWorker = false;
            m_worker = std::thread([this] { workerLoop(); });
        }

        void stopWorkerThread()
        {
            if (!m_worker.joinable())
                return;
            {
                const std::lock_guard<std::mutex> guard(m_mutex);
                m_stopWorker = true;
            }
            m_changed.notify_all();
            m_worker.join();
        }

        std::thread::id workerThreadId() const { return m_worker.get_id(); }

        double now = 0.0;
        uint64_t frame = 0u;
        HostLogLevel minimumLevel = HostLogLevel::Verbose;
        bool deferralSupported = true;
        std::vector<LoggedLine> lines;
        std::vector<uint64_t> dispatchedAfter;
        std::vector<uint64_t> waits;
        std::vector<uint64_t> deferrals;
        uint32_t tasksRun = 0u;

    private:
        struct Deferred
        {
            Completion* completion = nullptr;
            Handle after;
            std::function<void()> continuation;
        };

        void workerLoop()
        {
            for (;;)
            {
                {
                    std::unique_lock<std::mutex> lock(m_mutex);
                    m_changed.wait(lock, [this] { return m_stopWorker || !m_queue.empty(); });
                    if (m_queue.empty())
                        return;
                }
                runNextTask();
            }
        }

        mutable std::mutex m_mutex;
        std::condition_variable m_changed;
        std::deque<Handle> m_queue;
        std::vector<Deferred> m_deferred;
        uint64_t m_nextTaskId = 0u;
        std::thread m_worker;
        bool m_stopWorker = false;
    };

    using FakeEngine = BasicFakeEngine<true>;
    using FakeEngineNoDiagnostics = BasicFakeEngine<false>;

    namespace realClock
    {
        using Clock = std::chrono::steady_clock;

        inline Clock::time_point origin()
        {
            static const Clock::time_point start = Clock::now();
            return start;
        }

        inline double seconds() { return std::chrono::duration<double>(Clock::now() - origin()).count(); }

        inline Clock::time_point at(double whenSeconds)
        {
            return origin() + std::chrono::ceil<Clock::duration>(std::chrono::duration<double>(whenSeconds)) + Clock::duration(1);
        }
    } // namespace realClock

    class RealClockFakeEngine
    {
    public:
        static constexpr bool kHostDiagnostics = true;
        using Handle = std::shared_ptr<FakeTask>;
        using Completion = FakeCompletion;

        RealClockFakeEngine() = default;
        RealClockFakeEngine(const RealClockFakeEngine&) = delete;
        RealClockFakeEngine& operator=(const RealClockFakeEngine&) = delete;

        double nowSeconds() const { return realClock::seconds(); }
        uint64_t frameNumber() const { return m_frame.load(); }

        void advanceFrame() { m_frame.fetch_add(1u); }

        bool logEnabled(HostLogLevel level) const
        {
            return static_cast<int>(level) >= static_cast<int>(minimumLevel.load());
        }

        void log(HostLogLevel level, const char* line)
        {
            const std::lock_guard<std::mutex> guard(m_linesMutex);
            m_lines.push_back(LoggedLine{ level, line });
        }

        std::vector<LoggedLine> snapshotLines() const
        {
            const std::lock_guard<std::mutex> guard(m_linesMutex);
            return m_lines;
        }

        bool isPending(const Handle&) const { return false; }

        template <typename F>
        Handle dispatchAfter(const Handle&, F&& work)
        {
            std::forward<F>(work)();
            Handle task = std::make_shared<FakeTask>();
            task->id = inlineDispatches.fetch_add(1u) + 1u;
            task->done = true;
            return task;
        }

        void waitFor(const Handle&) {}

        template <typename F>
        bool deferUntil(Completion&, const Handle&, F&&)
        {
            return false;
        }

        std::atomic<HostLogLevel> minimumLevel{ HostLogLevel::Verbose };
        std::atomic<uint64_t> inlineDispatches{ 0u };

    private:
        std::atomic<uint64_t> m_frame{ 0u };
        mutable std::mutex m_linesMutex;
        std::vector<LoggedLine> m_lines;
    };

    template <bool HostDiagnostics = true>
    class BasicFakeRunnerHost
    {
    public:
        explicit BasicFakeRunnerHost(BasicFakeEngine<HostDiagnostics>& engine, Events* events = nullptr)
            : m_engine(&engine)
            , m_events(events)
        {
        }

        void sleepUntil(double untilSeconds)
        {
            if (m_events != nullptr)
                m_events->push_back("sleepUntil");
            sleeps.push_back(untilSeconds);
            m_engine->now = std::max(m_engine->now, untilSeconds);
        }

        std::vector<double> sleeps;

    private:
        BasicFakeEngine<HostDiagnostics>* m_engine;
        Events* m_events;
    };

    using FakeRunnerHost = BasicFakeRunnerHost<true>;
    using FakeRunnerHostNoDiagnostics = BasicFakeRunnerHost<false>;

    class RealRunnerHost
    {
    public:
        void sleepUntil(double untilSeconds)
        {
            sleepCalls.fetch_add(1u);
            std::this_thread::sleep_until(realClock::at(untilSeconds));
        }

        std::atomic<uint32_t> sleepCalls{ 0u };
    };

    struct FakeManager
    {
    };

    struct FakeStorage
    {
    };

    struct FakeRenderSnapshot
    {
    };

    template <typename ManagerT = FakeManager, typename StorageT = FakeStorage, typename RenderSnapshotT = FakeRenderSnapshot>
    class BasicRecordingGame
    {
    public:
        using Manager = ManagerT;
        using Storage = StorageT;
        using RenderSnapshot = RenderSnapshotT;

        BasicRecordingGame(Events& events, Manager& gameManager, Storage& gameStorage)
            : m_events(&events)
            , m_manager(&gameManager)
            , m_storage(&gameStorage)
        {
        }

        Manager& manager() { return *m_manager; }
        Storage& storage() { return *m_storage; }

        void beforePump(double hostNowSeconds, TickRateDoor& rate)
        {
            record("beforePump");
            beforePumpHostSeconds.push_back(hostNowSeconds);
            if (onBeforePump)
                onBeforePump(hostNowSeconds, rate);
        }

        void onFrameStepsDue(int32_t firstUpcomingTick, int32_t steps)
        {
            record("onFrameStepsDue:" + std::to_string(firstUpcomingTick) + "," + std::to_string(steps));
            frameStepsDue.emplace_back(firstUpcomingTick, steps);
        }

        void afterInlineBatch(int32_t firstUpcomingTick)
        {
            record("afterInlineBatch:" + std::to_string(firstUpcomingTick));
            afterInlineBatchTicks.push_back(firstUpcomingTick);
        }

        void releaseInputsForAuthorityTick(SimTick tick)
        {
            record(stepDriverTestSupport::tickEvent("releaseInputs", tick));
            releasedTicks.push_back(tick);
        }

        void onStepStarted(double stepStartSeconds)
        {
            record("onStepStarted");
            stepStartedSeconds.push_back(stepStartSeconds);
        }

        void onStepEnded() { record("onStepEnded"); }

        void onPostPhysics()
        {
            record("onPostPhysics");
            ++postPhysicsCalls;
        }

        std::function<void(double, TickRateDoor&)> onBeforePump;
        std::vector<double> beforePumpHostSeconds;
        std::vector<std::pair<int32_t, int32_t>> frameStepsDue;
        std::vector<int32_t> afterInlineBatchTicks;
        std::vector<SimTick> releasedTicks;
        std::vector<double> stepStartedSeconds;
        uint32_t postPhysicsCalls = 0u;

    private:
        void record(std::string event) { m_events->push_back(std::move(event)); }

        Events* m_events;
        Manager* m_manager;
        Storage* m_storage;
    };

    using RecordingGame = BasicRecordingGame<>;

    namespace idleScript
    {
        struct HonourDeadline
        {
        };

        struct Wakes
        {
            std::vector<double> times;
        };

        struct ActivityAt
        {
            std::vector<double> times;
        };

        struct Unavailable
        {
        };

        struct EarlyDeadline
        {
        };

        struct RealDeadline
        {
        };

        struct RealSpin
        {
        };

        inline constexpr double kOnePassSeconds = 50e-6;
        inline constexpr double kEarlyDeadlineSeconds = 1e-6;
    } // namespace idleScript

    using IdleScript = std::variant<idleScript::HonourDeadline, idleScript::Wakes, idleScript::ActivityAt, idleScript::Unavailable,
        idleScript::EarlyDeadline, idleScript::RealDeadline, idleScript::RealSpin>;

    struct IdleCall
    {
        double deadline = 0.0;
        double now = 0.0;
        double returnedAt = 0.0;
        IdleWake wake = IdleWake::Deadline;
    };

    inline const char* idleWakeName(IdleWake wake)
    {
        switch (wake)
        {
        case IdleWake::Deadline:
            return "Deadline";
        case IdleWake::Activity:
            return "Activity";
        case IdleWake::Unavailable:
            return "Unavailable";
        }
        return "?";
    }

    template <bool HostDiagnostics = true, typename ManagerT = FakeManager, typename StorageT = FakeStorage,
        typename RenderSnapshotT = FakeRenderSnapshot>
    class BasicRecordingIdleGame : public BasicRecordingGame<ManagerT, StorageT, RenderSnapshotT>
    {
        using Base = BasicRecordingGame<ManagerT, StorageT, RenderSnapshotT>;

    public:
        using Manager = ManagerT;
        using Storage = StorageT;
        using RenderSnapshot = RenderSnapshotT;
        using Engine = BasicFakeEngine<HostDiagnostics>;

        BasicRecordingIdleGame(Events& events, Manager& gameManager, Storage& gameStorage, Engine& clock,
            IdleScript script = idleScript::HonourDeadline{})
            : Base(events, gameManager, gameStorage)
            , m_events(&events)
            , m_engine(&clock)
            , m_script(std::move(script))
        {
        }

        BasicRecordingIdleGame(Events& events, Manager& gameManager, Storage& gameStorage, IdleScript script)
            : Base(events, gameManager, gameStorage)
            , m_events(&events)
            , m_engine(nullptr)
            , m_script(std::move(script))
        {
        }

        void setScript(IdleScript script)
        {
            m_script = std::move(script);
            m_cursor = 0u;
        }

        void setStopAfterSteps(std::atomic<bool>& stop, uint32_t steps)
        {
            OG_CHECK(steps > 0u, "BasicRecordingIdleGame::setStopAfterSteps: the stop is set after at least one onStepEnded");
            m_stop = &stop;
            m_stopAfterSteps = steps;
        }

        void beforePump(double hostNowSeconds, TickRateDoor& rate)
        {
            Base::beforePump(hostNowSeconds, rate);
            if (beforePumpHook)
                beforePumpHook();
        }

        void onStepEnded()
        {
            Base::onStepEnded();
            const uint32_t ended = stepsEnded.fetch_add(1u) + 1u;
            if (m_stop != nullptr && ended >= m_stopAfterSteps)
                m_stop->store(true);
        }

        IdleWake idleUntil(double deadline)
        {
            IdleCall call;
            call.deadline = deadline;
            call.now = clockNow();
            call.wake = follow(deadline);
            call.returnedAt = clockNow();
            idleCalls.push_back(call);
            m_events->push_back(std::string("idleUntil:") + idleWakeName(call.wake));
            return call.wake;
        }

        void wake()
        {
            {
                const std::lock_guard<std::mutex> guard(m_wakeMutex);
                m_wakeRequested = true;
            }
            m_wakeChanged.notify_all();
        }

        std::function<void()> beforePumpHook;
        std::vector<IdleCall> idleCalls;
        std::atomic<uint32_t> stepsEnded{ 0u };
        uint32_t scriptExhaustedCalls = 0u;

    private:
        bool isRealScript() const
        {
            return std::holds_alternative<idleScript::RealDeadline>(m_script) || std::holds_alternative<idleScript::RealSpin>(m_script);
        }

        double clockNow() const
        {
            if (isRealScript())
                return realClock::seconds();
            OG_CHECK(m_engine != nullptr,
                "BasicRecordingIdleGame: a virtual-time idle script advances the BasicFakeEngine it was constructed over");
            return m_engine->now;
        }

        IdleWake honourOrActivity(double deadline, const std::vector<double>* activities)
        {
            const double now = m_engine->now;
            const double end = deadline > now ? deadline : now + idleScript::kOnePassSeconds;
            if (activities != nullptr && m_cursor < activities->size() && (*activities)[m_cursor] < end)
            {
                m_engine->now = std::max(now, (*activities)[m_cursor]);
                ++m_cursor;
                return IdleWake::Activity;
            }
            m_engine->now = end;
            return IdleWake::Deadline;
        }

        IdleWake follow(double deadline)
        {
            if (std::holds_alternative<idleScript::HonourDeadline>(m_script))
                return honourOrActivity(deadline, nullptr);
            if (const idleScript::Wakes* wakes = std::get_if<idleScript::Wakes>(&m_script))
            {
                if (m_cursor >= wakes->times.size())
                {
                    ++scriptExhaustedCalls;
                    return honourOrActivity(deadline, nullptr);
                }
                m_engine->now = std::max(m_engine->now, wakes->times[m_cursor]);
                ++m_cursor;
                return m_engine->now < deadline ? IdleWake::Activity : IdleWake::Deadline;
            }
            if (const idleScript::ActivityAt* activity = std::get_if<idleScript::ActivityAt>(&m_script))
                return honourOrActivity(deadline, &activity->times);
            if (std::holds_alternative<idleScript::Unavailable>(m_script))
                return IdleWake::Unavailable;
            if (std::holds_alternative<idleScript::EarlyDeadline>(m_script))
            {
                m_engine->now += idleScript::kEarlyDeadlineSeconds;
                return IdleWake::Deadline;
            }
            if (std::holds_alternative<idleScript::RealDeadline>(m_script))
            {
                std::unique_lock<std::mutex> lock(m_wakeMutex);
                const bool woken = m_wakeChanged.wait_until(lock, realClock::at(deadline), [this] { return m_wakeRequested; });
                m_wakeRequested = false;
                return woken ? IdleWake::Activity : IdleWake::Deadline;
            }
            while (realClock::seconds() < deadline)
            {
            }
            return IdleWake::Deadline;
        }

        Events* m_events;
        Engine* m_engine;
        IdleScript m_script;
        size_t m_cursor = 0u;
        std::atomic<bool>* m_stop = nullptr;
        uint32_t m_stopAfterSteps = 0u;
        std::mutex m_wakeMutex;
        std::condition_variable m_wakeChanged;
        bool m_wakeRequested = false;
    };

    using RecordingIdleGame = BasicRecordingIdleGame<true>;
    using RecordingIdleGameNoDiagnostics = BasicRecordingIdleGame<false>;

    template <typename GameT>
    class BasicSamplingGame : public GameT
    {
    public:
        template <typename... ArgsT>
        explicit BasicSamplingGame(Events& events, ArgsT&&... args)
            : GameT(events, std::forward<ArgsT>(args)...)
            , m_sampleEvents(&events)
        {
        }

        void sampleInput()
        {
            m_sampleEvents->push_back("sampleInput");
            ++sampleInputCalls;
        }

        uint32_t sampleInputCalls = 0u;

    private:
        Events* m_sampleEvents;
    };

    using SamplingGame = BasicSamplingGame<RecordingGame>;
    using SamplingIdleGame = BasicSamplingGame<RecordingIdleGame>;

    template <typename GameT>
    class BasicReceivingGame : public GameT
    {
    public:
        template <typename... ArgsT>
        explicit BasicReceivingGame(Events& events, ArgsT&&... args)
            : GameT(events, std::forward<ArgsT>(args)...)
            , m_receiveEvents(&events)
        {
        }

        void receiveForStep()
        {
            m_receiveEvents->push_back("receiveForStep");
            ++receiveForStepCalls;
            if (onReceive)
                onReceive();
        }

        std::function<void()> onReceive;
        uint32_t receiveForStepCalls = 0u;

    private:
        Events* m_receiveEvents;
    };

    using ReceivingGame = BasicReceivingGame<RecordingGame>;

    class FakePresenter
    {
    public:
        explicit FakePresenter(Events& events)
            : m_events(&events)
        {
        }

        void captureCorrection_Step(uint64_t physicsStep)
        {
            m_events->push_back("captureCorrection:" + std::to_string(physicsStep));
            capturedSteps.push_back(physicsStep);
        }

        void publish_Step(const TickOutcome& outcome, double stepDeadlineSeconds)
        {
            m_events->push_back(stepDriverTestSupport::tickEvent("publish", outcome.tick));
            published.push_back(outcome);
            publishedDeadlines.push_back(stepDeadlineSeconds);
        }

        std::vector<uint64_t> capturedSteps;
        std::vector<TickOutcome> published;
        std::vector<double> publishedDeadlines;

    private:
        Events* m_events;
    };

    class LockUnlockOnly
    {
    public:
        void Lock()
        {
            m_mutex.lock();
            ++locks;
        }

        void Unlock() { m_mutex.unlock(); }

        uint32_t locks = 0u;

    private:
        std::mutex m_mutex;
    };

    class WrapperOverLockUnlockOnly
    {
    public:
        void lock() { m_inner.Lock(); }
        void unlock() { m_inner.Unlock(); }

        uint32_t locks() const { return m_inner.locks; }

    private:
        LockUnlockOnly m_inner;
    };

    static_assert(BasicLockable<WrapperOverLockUnlockOnly>,
        "WrapperOverLockUnlockOnly: lock/unlock over a Lock/Unlock-only mutex models BasicLockable");
    static_assert(!BasicLockable<LockUnlockOnly>,
        "LockUnlockOnly: a mutex with only Lock/Unlock does not model BasicLockable without a wrapper");

    struct FakeAccessScope
    {
        FakeAccessScope() { ++t_openScopes; }
        ~FakeAccessScope() { --t_openScopes; }
        FakeAccessScope(const FakeAccessScope&) = delete;
        FakeAccessScope& operator=(const FakeAccessScope&) = delete;

        static bool isOpenOnThisThread() { return t_openScopes > 0; }

        static inline thread_local int t_openScopes = 0;
    };

    struct FakeWorldHostConfig
    {
        bool isAuthority = false;
        bool runInline = false;
        std::optional<StepThreading> threading;

        StepThreading resolvedThreading() const
        {
            return threading.value_or(isAuthority || runInline ? StepThreading::Inline : StepThreading::FrameBounded);
        }
    };

    inline const char* steppingWord(StepThreading threading)
    {
        switch (threading)
        {
        case StepThreading::Inline:
            return "inline";
        case StepThreading::FrameBounded:
            return "worker";
        case StepThreading::FreeRunning:
            return "runner";
        }
        return "?";
    }

    struct ConfigureAtBuild
    {
    };

    inline constexpr ConfigureAtBuild kConfigureAtBuild{};

    class FakeAdapter
    {
    public:
        FakeAdapter(Events& events, const char* kind, const TickDriverThreadContext& context)
            : stepWorldPredicate(context.stepWorldPredicate())
        {
            events.push_back(std::string("emplace:") + kind + (stepWorldPredicate != nullptr ? "" : ":unconfigured"));
        }

        FakeAdapter(const FakeAdapter&) = delete;
        FakeAdapter& operator=(const FakeAdapter&) = delete;

        TickDriverThreadContext::AccessPredicate stepWorldPredicate = nullptr;
    };

    struct FakeVizAdapter
    {
        uint32_t id = 0u;
    };

    template <bool HostDiagnostics = true>
    class BasicFakeWorldHost
    {
    public:
        using World = LoggedWorld;
        using Lock = WrapperOverLockUnlockOnly;
        using WorldLock = WorldLockScope<Lock, FakeAccessScope>;
        using Config = FakeWorldHostConfig;

        BasicFakeWorldHost(Events& events, uint8_t roleBit, bool stepsInline, uint32_t ringDepthTicks = 8u)
            : m_events(&events)
            , m_world(ringDepthTicks, events)
        {
            m_context.template configure<HostDiagnostics>(roleBit, stepsInline);
        }

        BasicFakeWorldHost(Events& events, uint8_t roleBit, StepThreading threading, uint32_t ringDepthTicks = 8u)
            : m_events(&events)
            , m_world(ringDepthTicks, events)
        {
            m_context.template configure<HostDiagnostics>(roleBit, threading);
        }

        BasicFakeWorldHost(Events& events, ConfigureAtBuild, uint32_t ringDepthTicks = 8u)
            : m_events(&events)
            , m_world(ringDepthTicks, events)
        {
        }

        void build(const Config& config, const StaticWorldDescription& queryableStatics,
            const StaticWorldDescription& physicsOnlyStatics, std::optional<FakeAdapter>& bodyAdapter,
            std::optional<FakeAdapter>& queryAdapter)
        {
            const StepThreading threading = config.resolvedThreading();
            m_context.template configure<HostDiagnostics>(
                config.isAuthority ? tickDriverThread::kAuthorityRoleBit : tickDriverThread::kClientRoleBit, threading);
            m_events->push_back(std::string("configure:") + (config.isAuthority ? "authority" : "client") + ":"
                + steppingWord(threading));
            builtStatics.emplace_back(queryableStatics.shapes.size(), physicsOnlyStatics.shapes.size());
            bodyAdapter.emplace(*m_events, "body", m_context);
            queryAdapter.emplace(*m_events, "query", m_context);
        }

        template <typename ManagerT>
        void bindIntegratedTickSource(const ManagerT& manager)
        {
            m_events->push_back("bindIntegratedTickSource");
            integratedTick = [&manager] { return static_cast<uint32_t>(manager.currentIntegratedTick()); };
        }

        uint8_t rootDeclarationIndex() const
        {
            m_events->push_back("rootDeclarationIndex");
            return rootIndex;
        }

        template <typename PublishF>
        std::optional<uint32_t> acquire(uint64_t key, const char*, bool isAuthority, std::optional<uint32_t> replicatedSlot,
            PublishF&& publish)
        {
            const auto held = m_slots.find(key);
            if (held != m_slots.end())
            {
                m_events->push_back("acquire:" + std::to_string(key) + ":" + std::to_string(held->second));
                return held->second;
            }
            if (!isAuthority && !replicatedSlot.has_value())
            {
                m_events->push_back("acquire:" + std::to_string(key) + ":wait");
                return std::nullopt;
            }
            const uint32_t slot = isAuthority ? m_nextSlot++ : *replicatedSlot;
            m_slots[key] = slot;
            m_events->push_back("acquire:" + std::to_string(key) + ":" + std::to_string(slot));
            if (isAuthority)
                publish(slot);
            return slot;
        }

        template <typename SimulatableT, typename StaticDataT, typename OnWorldLockWaitF>
        void bind(uint32_t slot, unsigned int id, SimulatableT&, const StaticDataT&, OnWorldLockWaitF&& onWorldLockWait)
        {
            const WorldLock worldLock = lockWorld();
            m_events->push_back("bind:" + std::to_string(slot) + ":" + std::to_string(id));
            onWorldLockWait(bindLockWaitSeconds);
        }

        template <typename PushBodySlotOccupancyF>
        std::optional<uint32_t> release(uint64_t key, unsigned int id, PushBodySlotOccupancyF&& pushBodySlotOccupancy)
        {
            const auto held = m_slots.find(key);
            if (held == m_slots.end())
                return std::nullopt;
            const uint32_t slot = held->second;
            m_slots.erase(held);
            m_events->push_back("release:" + std::to_string(key) + ":" + std::to_string(slot) + ":" + std::to_string(id));
            pushBodySlotOccupancy(slot, false);
            return slot;
        }

        std::optional<uint32_t> slotOf(uint64_t key) const
        {
            const auto held = m_slots.find(key);
            if (held == m_slots.end())
                return std::nullopt;
            return held->second;
        }

        FakeVizAdapter& editVizQuery()
        {
            m_events->push_back("editVizQuery");
            return vizQuery;
        }

        const FakeVizAdapter& getVizReader() const
        {
            m_events->push_back("getVizReader");
            return vizReader;
        }

        template <typename PublishF>
        std::optional<QueryVolumeId> registerVizVolume(const QueryVolumeDescriptor&, uint64_t key, const char* name,
            bool isAuthority, std::optional<uint32_t> replicatedSlot, PublishF&& publish)
        {
            const std::optional<uint32_t> slot = acquire(key, name, isAuthority, replicatedSlot, std::forward<PublishF>(publish));
            if (!slot.has_value())
                return std::nullopt;
            m_events->push_back("registerVizVolume:" + std::to_string(*slot));
            return QueryVolumeId{ *slot };
        }

        bool hasShadowWorld() const { return shadowWorld; }

        template <typename OnRestoreF>
        void restore_GameThread(uint64_t frameNumber, OnRestoreF&& onRestore)
        {
            m_events->push_back("restore:" + std::to_string(frameNumber));
            onRestore(restoreSeconds, restoreRefused);
        }

        template <typename PosesT, typename TargetsT, typename VisitorT>
        void overwriteShadowPoses_GameThread(const PosesT&, TargetsT&&, VisitorT&&)
        {
            m_events->push_back("overwriteShadowPoses");
        }

        World& stepWorld() { return m_world; }

        WorldLock lockForStep() { return WorldLock(m_lock, m_context); }

        WorldLock lockWorld() { return WorldLock(m_lock, m_context); }

        void afterTick_Step(const TickOutcome& outcome)
        {
            m_events->push_back(stepDriverTestSupport::tickEvent("hostAfterTick", outcome.tick));
            afterTickOutcomes.push_back(outcome);
        }

        void afterRunTick_Step(uint64_t physicsStep, const TickOutcome& outcome)
        {
            m_events->push_back("afterRunTick:" + std::to_string(physicsStep));
            afterRunTickSteps.emplace_back(physicsStep, outcome);
        }

        const TickDriverThreadContext& threadContext() const { return m_context; }

        Lock& lock() { return m_lock; }

        std::vector<TickOutcome> afterTickOutcomes;
        std::vector<std::pair<uint64_t, TickOutcome>> afterRunTickSteps;
        std::vector<std::pair<size_t, size_t>> builtStatics;
        std::function<uint32_t()> integratedTick;
        uint8_t rootIndex = 0u;
        double bindLockWaitSeconds = 0.0;
        bool shadowWorld = false;
        double restoreSeconds = 0.0;
        bool restoreRefused = false;
        FakeVizAdapter vizQuery{ 1u };
        FakeVizAdapter vizReader{ 2u };

    private:
        TickDriverThreadContext m_context;
        Events* m_events;
        World m_world;
        Lock m_lock;
        std::map<uint64_t, uint32_t> m_slots;
        uint32_t m_nextSlot = 0u;
    };

    using FakeWorldHost = BasicFakeWorldHost<true>;
    using FakeWorldHostNoDiagnostics = BasicFakeWorldHost<false>;

    class FakeRenderSink
    {
    public:
        struct Target
        {
            glm::vec3 position{ 0.f };
            std::string name;
        };

        struct Apply
        {
            uint32_t id = 0u;
            glm::vec3 position{ 0.f };
            std::optional<glm::quat> rotation;
        };

        void addTarget(uint32_t id, const glm::vec3& position, std::string displayName)
        {
            targets[id] = Target{ position, std::move(displayName) };
        }

        void removeTarget(uint32_t id) { targets.erase(id); }

        std::optional<glm::vec3> applyRootPose(uint32_t id, const glm::vec3& position, const std::optional<glm::quat>& rotation)
        {
            applies.push_back(Apply{ id, position, rotation });
            const auto target = targets.find(id);
            if (target == targets.end())
                return std::nullopt;
            target->second.position = holdRule ? holdRule(target->second.position, position) : position;
            return target->second.position;
        }

        std::optional<glm::vec3> readAnchor(uint32_t id) const
        {
            const auto target = targets.find(id);
            if (target == targets.end())
                return std::nullopt;
            return target->second.position;
        }

        const char* name(uint32_t id) const
        {
            const auto target = targets.find(id);
            return target == targets.end() ? "?" : target->second.name.c_str();
        }

        std::function<glm::vec3(const glm::vec3& held, const glm::vec3& requested)> holdRule;
        std::map<uint32_t, Target> targets;
        std::vector<Apply> applies;
    };

    static_assert(TickDriverEngine<FakeEngine>, "FakeEngine must model TickDriverEngine");
    static_assert(TickDriverEngine<FakeEngineNoDiagnostics>, "FakeEngineNoDiagnostics must model TickDriverEngine");
    static_assert(TickDriverGame<RecordingGame>, "RecordingGame must model TickDriverGame");
    static_assert(TickDriverPresenterSteps<FakePresenter>, "FakePresenter must model TickDriverPresenterSteps");
    static_assert(TickDriverWorldHost<FakeWorldHost>, "FakeWorldHost must model TickDriverWorldHost");
    static_assert(TickDriverWorldHost<FakeWorldHostNoDiagnostics>, "FakeWorldHostNoDiagnostics must model TickDriverWorldHost");
    static_assert(TickDriverRenderSink<FakeRenderSink>, "FakeRenderSink must model TickDriverRenderSink");
    static_assert(TickBackendWorldHost<FakeWorldHost, tickBackendWorldHost::SimulatableArchetype>,
        "FakeWorldHost must model TickBackendWorldHost");
    static_assert(TickBackendWorldHost<FakeWorldHostNoDiagnostics, tickBackendWorldHost::SimulatableArchetype>,
        "FakeWorldHostNoDiagnostics must model TickBackendWorldHost");
    static_assert(TickBackendWorldBuild<FakeWorldHost, std::optional<FakeAdapter>, std::optional<FakeAdapter>>,
        "FakeWorldHost's build must model TickBackendWorldBuild over its two fake adapters");
    static_assert(TickDriverEngine<RealClockFakeEngine>, "RealClockFakeEngine must model TickDriverEngine");
    static_assert(TickDriverRunnerHost<FakeRunnerHost>, "FakeRunnerHost must model TickDriverRunnerHost");
    static_assert(TickDriverRunnerHost<FakeRunnerHostNoDiagnostics>, "FakeRunnerHostNoDiagnostics must model TickDriverRunnerHost");
    static_assert(TickDriverRunnerHost<RealRunnerHost>, "RealRunnerHost must model TickDriverRunnerHost");
    static_assert(TickDriverIdleGame<RecordingIdleGame>, "RecordingIdleGame must model TickDriverIdleGame");
    static_assert(TickDriverIdleGame<RecordingIdleGameNoDiagnostics>, "RecordingIdleGameNoDiagnostics must model TickDriverIdleGame");
    static_assert(TickDriverSampleGame<SamplingGame>, "SamplingGame must model TickDriverSampleGame");
    static_assert(TickDriverSampleGame<SamplingIdleGame> && TickDriverIdleGame<SamplingIdleGame>,
        "SamplingIdleGame must model both TickDriverSampleGame and TickDriverIdleGame");
    static_assert(!TickDriverSampleGame<RecordingGame> && !TickDriverSampleGame<RecordingIdleGame>,
        "RecordingGame and RecordingIdleGame must not sample, so every existing trace keeps its events");
    static_assert(TickDriverReceiveGame<ReceivingGame> && !TickDriverSampleGame<ReceivingGame>,
        "ReceivingGame must model TickDriverReceiveGame and nothing else the plain game does not");
    static_assert(!TickDriverReceiveGame<RecordingGame> && !TickDriverReceiveGame<RecordingIdleGame>
            && !TickDriverReceiveGame<SamplingGame>,
        "RecordingGame, RecordingIdleGame and SamplingGame must not receive, so every existing trace keeps its events");
} // namespace tickDriverFakes
