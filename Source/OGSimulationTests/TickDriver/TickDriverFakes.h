// SPDX-License-Identifier: MPL-2.0
// docs/TickDriverFakes-rationale.md

#pragma once

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
#include <vector>

#include "glm/gtc/quaternion.hpp"
#include "glm/vec3.hpp"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationTickBackend.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/TickDriverConcepts.h"
#include "OGSimulation/TickDriverLog.h"
#include "OGSimulation/TickDriverThreadContext.h"
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
    };

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

        BasicFakeWorldHost(Events& events, ConfigureAtBuild, uint32_t ringDepthTicks = 8u)
            : m_events(&events)
            , m_world(ringDepthTicks, events)
        {
        }

        void build(const Config& config, const StaticWorldDescription& queryableStatics,
            const StaticWorldDescription& physicsOnlyStatics, std::optional<FakeAdapter>& bodyAdapter,
            std::optional<FakeAdapter>& queryAdapter)
        {
            const bool stepsInline = config.isAuthority || config.runInline;
            m_context.template configure<HostDiagnostics>(
                config.isAuthority ? tickDriverThread::kAuthorityRoleBit : tickDriverThread::kClientRoleBit, stepsInline);
            m_events->push_back(std::string("configure:") + (config.isAuthority ? "authority" : "client")
                + (stepsInline ? ":inline" : ":worker"));
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

        template <typename PushOccupancyF>
        std::optional<uint32_t> release(uint64_t key, unsigned int id, PushOccupancyF&& pushOccupancy)
        {
            const auto held = m_slots.find(key);
            if (held == m_slots.end())
                return std::nullopt;
            const uint32_t slot = held->second;
            m_slots.erase(held);
            m_events->push_back("release:" + std::to_string(key) + ":" + std::to_string(slot) + ":" + std::to_string(id));
            pushOccupancy(slot, false);
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
} // namespace tickDriverFakes
