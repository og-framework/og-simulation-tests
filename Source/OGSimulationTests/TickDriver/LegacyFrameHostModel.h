// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "OGSimulation/LatencyBudgetProbe.h"
#include "OGSimulation/Mailbox.h"
#include "OGSimulation/OGAssert.h"
#include "OGSimulation/SimulationScheduler.h"
#include "OGSimulation/SimulationStepDriver.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/TickDriverConcepts.h"
#include "OGSimulation/TickDriverLog.h"
#include "OGSimulation/TickDriverStepHooks.h"
#include "OGSimulation/TickDriverThreadContext.h"

namespace legacyFrameHost
{
    inline std::string printfString(const char* fmt, ...)
    {
        va_list args;
        va_start(args, fmt);
        va_list retry;
        va_copy(retry, args);
        const int length = std::vsnprintf(nullptr, 0, fmt, args);
        va_end(args);
        std::string out(length > 0 ? static_cast<size_t>(length) : 0u, '\0');
        if (length > 0)
            std::vsnprintf(out.data(), out.size() + 1u, fmt, retry);
        va_end(retry);
        return out;
    }

    struct OccupancyCommandUImpl
    {
        uint32_t slot     = 0;
        bool     occupied = false;
    };

    struct PercentilesUImpl
    {
        uint32_t n   = 0u;
        double   p50 = 0.0;
        double   p99 = 0.0;
        double   max = 0.0;
    };

    inline PercentilesUImpl percentilesOf(std::vector<float>& values)
    {
        PercentilesUImpl result;
        result.n = static_cast<uint32_t>(values.size());
        if (result.n == 0u)
            return result;

        std::sort(values.begin(), values.end());
        result.p50 = values[latencyBudget::nearestRankIndex(result.n, 500u)];
        result.p99 = values[latencyBudget::nearestRankIndex(result.n, 990u)];
        result.max = values.back();
        return result;
    }

    inline std::string formatPercentiles(std::vector<float>& values, bool twoDecimals)
    {
        const PercentilesUImpl p = percentilesOf(values);
        if (p.n == 0u)
            return "-";
        return twoDecimals ? printfString("%.2f/%.2f/%.2f", p.p50, p.p99, p.max)
                           : printfString("%.1f/%.1f/%.1f", p.p50, p.p99, p.max);
    }

    template <typename GameT, typename WorldHostT, typename PresenterT, typename EngineT>
    class LegacyFrameHostModel
    {
    public:
        static constexpr uint32_t kMaxStepsPerFrame = 240u;
        static constexpr size_t   kOccupancyCommandCapacity = 64u;
        static constexpr size_t   kStepCostSampleCapacity   = 256u;
        static constexpr double   kWindowSeconds            = 10.0;
        static constexpr uint32_t kMaxWindowSamples         = 16384u;

        using Handle = typename EngineT::Handle;
        using Completion = typename EngineT::Completion;

        struct Config
        {
            double dtSeconds   = 0.0;
            bool   isAuthority = false;
            bool   stepsInline = true;
        };

        LegacyFrameHostModel(GameT& owner, WorldHostT& worlds, PresenterT& presenter, EngineT& engine)
            : m_owner(&owner)
            , m_worlds(&worlds)
            , m_presenter(&presenter)
            , m_engine(&engine)
            , m_doors(*this)
        {
        }

        LegacyFrameHostModel(const LegacyFrameHostModel&) = delete;
        LegacyFrameHostModel& operator=(const LegacyFrameHostModel&) = delete;

        int32_t cvarSimMaxCatchUpSteps = 60;
        int32_t cvarSimBlockMode       = 0;

        void startJoltStepping(float dt, bool worldIsAuthority)
        {
            {
                const auto worldLock = m_worlds->lockForStep();
                m_stepHooks.emplace(Doors{ m_doors, *m_owner, *m_presenter, *m_worlds, *m_engine });
                m_stepDriver.emplace(m_owner->manager(), m_worlds->stepWorld(), *m_stepHooks,
                    StepDriverConfig{ dt, ResimPolicy::OnRequest, 0u });
            }

            begin(Config{ static_cast<double>(dt), worldIsAuthority, m_worlds->threadContext().stepsInline() });
        }

        void begin(const Config& config)
        {
            OG_CHECK(!m_begun, "SimulationFrameHostUImpl::begin: the frame host is already running.");
            OG_CHECK(!config.isAuthority || config.stepsInline,
                "SimulationFrameHostUImpl::begin: the authority must step inline on the game thread. Its input release "
                "delivers into UObjects. Was design D2 (task 18).");

            m_begun       = true;
            m_isAuthority = config.isAuthority;
            m_stepsInline = config.stepsInline;

            const int32_t requestedCap = cvarSimMaxCatchUpSteps;
            const uint32_t cap = static_cast<uint32_t>(std::clamp<int32_t>(requestedCap, 1, static_cast<int32_t>(kMaxStepsPerFrame)));
            if (static_cast<int32_t>(cap) != requestedCap)
            {
                logWarning(printfString("[SimHost.Frame] og.Sim.MaxCatchUpSteps=%d is outside [1, %u]; using %u",
                    requestedCap, kMaxStepsPerFrame, cap));
            }

            m_hostTimeSeconds = 0.0;
            m_lastFrameStartSeconds = 0.0;
            m_scheduler.emplace(SchedulerConfig{ config.dtSeconds, MaxCatchUpSteps{ cap }, 0.75, 1.25 }, m_hostTimeSeconds);
            for (WindowSamplesUImpl* samples : m_windowSamples.all())
                samples->values.reserve(kMaxWindowSamples);
            startWindow_GameThread(m_engine->nowSeconds());

            m_tickFunctionsRegistered = true;

            logWarning(printfString(
                "[SimHost.Frame] role=%s stepping=%s dt=%.6f maxCatchUpSteps=%u rateScale=[0.75,1.25] timeBase=worldDelta",
                m_isAuthority ? "Authority" : "Client",
                m_stepsInline ? "inline" : "worker",
                config.dtSeconds, cap));
        }

        void unregisterTickFunctions()
        {
            if (!m_begun)
                return;

            m_tickFunctionsRegistered = false;
        }

        void waitForOutstandingSteps()
        {
            if (!m_begun)
                return;

            OG_CHECK(!m_worlds->threadContext().isWorldLockHeldOnThisThread(),
                "SimulationFrameHostUImpl::waitForOutstandingSteps: this thread holds the Jolt world mutex (J2) and is "
                "about to wait for a step task that takes it: a deadlock. Was frame-host G-05.");

            std::array<Handle, 2> outstanding{};
            size_t count = 0u;
            if (isValid(m_pendingSteps))
                outstanding[count++] = m_pendingSteps;
            if (isValid(m_blockingSteps))
                outstanding[count++] = m_blockingSteps;
            if (count != 0u)
            {
                for (size_t index = 0u; index < count; ++index)
                    m_engine->waitFor(outstanding[index]);
            }

            m_pendingSteps  = Handle{};
            m_blockingSteps = Handle{};
        }

        void pushOccupancy_GameThread(uint32_t slot, bool occupied)
        {
            [[maybe_unused]] const bool pushed = m_occupancyCommands.tryPush(OccupancyCommandUImpl{ slot, occupied });
            OG_CHECK(pushed,
                "SimulationFrameHostUImpl::pushOccupancy_GameThread: the occupancy ring (J1) is full.");
        }

        void noteWorldMutexWait_GameThread(double waitSeconds)
        {
            ++m_windowCounts.j2Waits;
            m_windowSamples.j2WaitUs.add(waitSeconds * 1.0e6);
        }

        void noteShadowRestore_GameThread(double restoreSeconds, bool refused)
        {
            m_windowSamples.shadowRestoreUs.add(restoreSeconds * 1.0e6);
            m_windowCounts.shadowRefused += refused ? 1u : 0u;
        }

        bool isRunningBatchOnThisThread() const
        {
            return t_batchHost == this;
        }

        bool isStepRunningOnThisThread() const
        {
            return m_begun && m_worlds->threadContext().isStepRunningOnThisThread();
        }

        void applyOccupancyCommands_Step()
        {
            OG_CHECK(isStepRunningOnThisThread(),
                "SimulationFrameHostUImpl::applyOccupancyCommands_Step: called outside a step of this manager.");
            while (std::optional<OccupancyCommandUImpl> command = m_occupancyCommands.tryPop())
                m_stepDriver->noteOccupancy(command->slot, command->occupied);
        }

        void publishTickOffset_Step(int64_t offset)
        {
            m_tickOffset.store(offset, std::memory_order_relaxed);
        }

        void pushStepCost_Step(const StepCostSample& sample)
        {
            m_stepCosts.tryPush(sample);
        }

        double hostNowSeconds_GameThread() const
        {
            OG_CHECK(m_worlds->threadContext().isOwnerThreadForRole(),
                "SimulationFrameHostUImpl::hostNowSeconds_GameThread: read off the game thread.");
            return m_hostTimeSeconds;
        }

        double stepIntervalSeconds_GameThread()
        {
            const SimulationScheduler& scheduler = schedulerOnOwningThread();
            return scheduler.config().dtSeconds / scheduler.rateScale();
        }

        void startSteps_GameThread(float deltaSeconds)
        {
            const double startSeconds = m_engine->nowSeconds();
            if (m_lastFrameStartSeconds > 0.0)
                m_windowSamples.frameMs.add((startSeconds - m_lastFrameStartSeconds) * 1000.0);
            m_lastFrameStartSeconds = startSeconds;

            const uint32_t numSteps = pumpAndRunSteps_GameThread(deltaSeconds);

            ++m_windowCounts.frames;
            m_windowCounts.steps += numSteps;
            m_windowCounts.maxStepsPerFrame = std::max(m_windowCounts.maxStepsPerFrame, numSteps);
            m_windowSamples.startPhysicsUs.add((m_engine->nowSeconds() - startSeconds) * 1.0e6);
        }

        void endSteps_GameThread(Completion* completion)
        {
            OG_CHECK(!m_worlds->threadContext().isWorldLockHeldOnThisThread(),
                "SimulationFrameHostUImpl::endSteps_GameThread: the game thread holds the Jolt world mutex (J2) at end of "
                "physics, where it waits for step tasks that take it: a deadlock. Was frame-host G-05.");

            const double endStartSeconds = m_engine->nowSeconds();
            const Handle waitFor = cvarSimBlockMode == 1 ? m_pendingSteps : m_blockingSteps;
            if (!isValid(waitFor) || !m_engine->isPending(waitFor))
            {
                runPostPhysicsStep_GameThread(std::nullopt);
                return;
            }

            if (completion == nullptr)
            {
                m_engine->waitFor(waitFor);
                runPostPhysicsStep_GameThread(endStartSeconds);
                return;
            }

            const bool held = m_engine->deferUntil(*completion, waitFor,
                [this, endStartSeconds]()
                {
                    if (!m_tickFunctionsRegistered)
                        return;
                    runPostPhysicsStep_GameThread(endStartSeconds);
                });
            if (!held)
            {
                m_engine->waitFor(waitFor);
                runPostPhysicsStep_GameThread(endStartSeconds);
            }
        }

        void runJoltStep_Step(uint64_t physicsStep, double stepDeadlineSeconds)
        {
            OG_CHECK(isRunningBatchOnThisThread(),
                "runJoltStep_Step: called outside the frame host's step batch.");
            OG_CHECK(!m_stepsInline || m_worlds->threadContext().isOwnerThreadForRole(),
                "runJoltStep_Step: an inline step ran off the game thread.");
            const auto worldLock = m_worlds->lockForStep();
            const StepScope stepScope(m_worlds->threadContext());
            m_stepHooks->currentStepDeadline = stepDeadlineSeconds;
            const TickOutcome outcome = m_stepDriver->runTick(physicsStep);
            m_worlds->afterRunTick_Step(physicsStep, outcome);
        }

        LostTime lostTime() const { return m_scheduler->lostTime(); }
        uint64_t physicsStepCount() const { return m_scheduler->physicsStepCount(); }
        double rateScale() const { return m_scheduler->rateScale(); }

    private:
        class FrameHostDoors
        {
        public:
            explicit FrameHostDoors(LegacyFrameHostModel& host) : m_host(&host) {}

            void applyOccupancyCommands_Step() { m_host->applyOccupancyCommands_Step(); }
            void publishTickOffset_Step(int64_t offset) { m_host->publishTickOffset_Step(offset); }
            void pushStepCost_Step(const StepCostSample& sample) { m_host->pushStepCost_Step(sample); }
            bool isStepRunningOnThisThread() const { return m_host->isStepRunningOnThisThread(); }
            bool isOwnerThread() const { return m_host->m_worlds->threadContext().isOwnerThreadForRole(); }

        private:
            LegacyFrameHostModel* m_host;
        };

        using Doors      = StepDoors<FrameHostDoors, GameT, PresenterT, WorldHostT, EngineT>;
        using StepHooksT = TickDriverStepHooks<Doors>;
        using StepDriverT = SimulationStepDriver<typename GameT::Manager, typename WorldHostT::World, StepHooksT>;

        struct StepBatch
        {
            uint64_t first = 0u;
            uint32_t count = 0u;
            std::array<double, kMaxStepsPerFrame> deadlines{};
        };

        struct WindowSamplesUImpl
        {
            std::vector<float> values;
            uint32_t           overflow = 0u;

            void add(double value)
            {
                if (values.size() < kMaxWindowSamples)
                    values.push_back(static_cast<float>(value));
                else
                    ++overflow;
            }
        };

        struct HostWindowCountsUImpl
        {
            double   startSeconds     = 0.0;
            uint32_t frames           = 0u;
            uint64_t steps            = 0u;
            uint32_t maxStepsPerFrame = 0u;
            uint32_t endWaits         = 0u;
            uint32_t j2Waits          = 0u;
            uint32_t replays          = 0u;
            uint64_t replayedTicks    = 0u;
            uint32_t maxReplayDepth   = 0u;
            uint32_t resimRefused     = 0u;
            uint32_t shadowRefused    = 0u;
            LostTime lostAtStart;
            uint64_t ignoredAtStart   = 0u;
            uint64_t j1DropsAtStart   = 0u;
            uint64_t j6DropsAtStart   = 0u;
        };

        struct HostWindowSamplesUImpl
        {
            WindowSamplesUImpl frameMs;
            WindowSamplesUImpl startPhysicsUs;
            WindowSamplesUImpl endWaitUs;
            WindowSamplesUImpl postPhysicsUs;
            WindowSamplesUImpl j2WaitUs;
            WindowSamplesUImpl stepUs;
            WindowSamplesUImpl preUs;
            WindowSamplesUImpl shadowRestoreUs;

            std::array<WindowSamplesUImpl*, 8> all()
            {
                return { &frameMs, &startPhysicsUs, &endWaitUs, &postPhysicsUs, &j2WaitUs, &stepUs, &preUs, &shadowRestoreUs };
            }
        };

        class BatchScopeUImpl
        {
        public:
            explicit BatchScopeUImpl(const LegacyFrameHostModel& host)
                : m_previous(t_batchHost)
            {
                t_batchHost = &host;
            }

            ~BatchScopeUImpl()
            {
                t_batchHost = m_previous;
            }

            BatchScopeUImpl(const BatchScopeUImpl&) = delete;
            BatchScopeUImpl& operator=(const BatchScopeUImpl&) = delete;

        private:
            const LegacyFrameHostModel* m_previous;
        };

        static bool isValid(const Handle& handle) { return handle != Handle{}; }

        void logWarning(const std::string& line) { m_engine->log(HostLogLevel::Warning, line.c_str()); }

        SimulationScheduler& schedulerOnOwningThread()
        {
            OG_CHECK(m_worlds->threadContext().isOwnerThreadForRole(),
                "SimulationFrameHostUImpl: the scheduler was reached off the game thread.");
            OG_CHECK(m_scheduler.has_value(), "SimulationFrameHostUImpl: the scheduler was reached before begin().");
            return *m_scheduler;
        }

        uint32_t pumpAndRunSteps_GameThread(float deltaSeconds)
        {
            m_blockingSteps = m_pendingSteps;

            SimulationScheduler& scheduler = schedulerOnOwningThread();
            m_hostTimeSeconds += static_cast<double>(deltaSeconds);
            TickRateDoor rate(scheduler);
            m_owner->beforePump(m_hostTimeSeconds, rate);
            const uint32_t numSteps = scheduler.pump(m_hostTimeSeconds);
            if (numSteps == 0u)
                return 0u;

            StepBatch batch;
            batch.first = scheduler.physicsStepCount() - numSteps;
            batch.count = numSteps;
            for (uint32_t index = 0u; index < numSteps; ++index)
                batch.deadlines[index] = scheduler.stepDeadline(batch.first + index);

            const int32_t firstUpcomingSimTick =
                static_cast<int32_t>(static_cast<int64_t>(batch.first) + m_tickOffset.load(std::memory_order_relaxed));

            m_owner->onFrameStepsDue(firstUpcomingSimTick, static_cast<int32_t>(numSteps));

            if (m_stepsInline)
            {
                runBatch(batch);
                m_owner->afterInlineBatch(firstUpcomingSimTick);
                return numSteps;
            }

            Handle prerequisites{};
            if (isValid(m_pendingSteps) && m_engine->isPending(m_pendingSteps))
                prerequisites = m_pendingSteps;

            m_pendingSteps = m_engine->dispatchAfter(prerequisites, [this, batch]() { runBatch(batch); });
            return numSteps;
        }

        void runPostPhysicsStep_GameThread(std::optional<double> waitStartSeconds)
        {
            const double postStartSeconds = m_engine->nowSeconds();
            if (waitStartSeconds.has_value())
            {
                ++m_windowCounts.endWaits;
                m_windowSamples.endWaitUs.add((postStartSeconds - *waitStartSeconds) * 1.0e6);
            }

            m_owner->onPostPhysics();

            const double nowSeconds = m_engine->nowSeconds();
            m_windowSamples.postPhysicsUs.add((nowSeconds - postStartSeconds) * 1.0e6);
            drainStepCosts_GameThread();
            if (nowSeconds - m_windowCounts.startSeconds >= kWindowSeconds)
                logWindow_GameThread(nowSeconds);
        }

        void drainStepCosts_GameThread()
        {
            while (std::optional<StepCostSample> sample = m_stepCosts.tryPop())
            {
                m_windowSamples.stepUs.add(sample->stepSeconds * 1.0e6);
                if (m_isAuthority || sample->replayedTicks > 0u)
                    m_windowSamples.preUs.add(sample->preSeconds * 1.0e6);
                if (sample->replayedTicks > 0u)
                {
                    ++m_windowCounts.replays;
                    m_windowCounts.replayedTicks += sample->replayedTicks;
                    m_windowCounts.maxReplayDepth = std::max(m_windowCounts.maxReplayDepth, sample->replayedTicks);
                }
                m_windowCounts.resimRefused += sample->resimRefused ? 1u : 0u;
            }
        }

        void startWindow_GameThread(double nowSeconds)
        {
            const SimulationScheduler& scheduler = schedulerOnOwningThread();

            m_windowCounts                = HostWindowCountsUImpl{};
            m_windowCounts.startSeconds   = nowSeconds;
            m_windowCounts.lostAtStart    = scheduler.lostTime();
            m_windowCounts.ignoredAtStart = scheduler.ignoredTimeSamples();
            m_windowCounts.j1DropsAtStart = m_occupancyCommands.drops();
            m_windowCounts.j6DropsAtStart = m_stepCosts.drops();
            for (WindowSamplesUImpl* samples : m_windowSamples.all())
            {
                samples->values.clear();
                samples->overflow = 0u;
            }
        }

        void logWindow_GameThread(double nowSeconds)
        {
            const SimulationScheduler& scheduler = schedulerOnOwningThread();
            const HostWindowCountsUImpl& counts = m_windowCounts;
            HostWindowSamplesUImpl& samples = m_windowSamples;

            const LostTime lost = scheduler.lostTime();
            uint32_t overflow = 0u;
            for (const WindowSamplesUImpl* windowSamples : samples.all())
                overflow += windowSamples->overflow;

            const std::string replayDepth = counts.replays == 0u
                ? std::string("-")
                : printfString("%.2f/%u",
                    static_cast<double>(counts.replayedTicks) / static_cast<double>(counts.replays), counts.maxReplayDepth);
            const uint32_t costSamples = static_cast<uint32_t>(samples.stepUs.values.size());
            const uint32_t preTicks    = static_cast<uint32_t>(samples.preUs.values.size());
            const uint32_t restores    = static_cast<uint32_t>(samples.shadowRestoreUs.values.size());

            logWarning(printfString(
                "[SimHost.Window] role=%s stepping=%s seconds=%.2f frames=%u frameMs=%s startPhysicsUs=%s endWaits=%u "
                "endWaitUs=%s postPhysicsUs=%s steps=%llu maxStepsPerFrame=%u lostSteps=%llu lostSeconds=%.4f "
                "ignoredTimeSamples=%llu j1Drops=%llu j2Waits=%u j2WaitUs=%s costSamples=%u stepUs=%s preTicks=%u preUs=%s "
                "replayDepth=%s resimRefused=%u j6Drops=%llu shadowRestores=%u shadowRestoreUs=%s shadowRefused=%u "
                "sampleOverflow=%u",
                m_isAuthority ? "Authority" : "Client",
                m_stepsInline ? "inline" : "worker",
                nowSeconds - counts.startSeconds, counts.frames,
                formatPercentiles(samples.frameMs.values, true).c_str(),
                formatPercentiles(samples.startPhysicsUs.values, false).c_str(),
                counts.endWaits,
                formatPercentiles(samples.endWaitUs.values, false).c_str(),
                formatPercentiles(samples.postPhysicsUs.values, false).c_str(),
                static_cast<unsigned long long>(counts.steps), counts.maxStepsPerFrame,
                static_cast<unsigned long long>(lost.steps - counts.lostAtStart.steps),
                lost.seconds - counts.lostAtStart.seconds,
                static_cast<unsigned long long>(scheduler.ignoredTimeSamples() - counts.ignoredAtStart),
                static_cast<unsigned long long>(m_occupancyCommands.drops() - counts.j1DropsAtStart),
                counts.j2Waits,
                formatPercentiles(samples.j2WaitUs.values, false).c_str(),
                costSamples,
                formatPercentiles(samples.stepUs.values, false).c_str(),
                preTicks,
                formatPercentiles(samples.preUs.values, false).c_str(),
                replayDepth.c_str(), counts.resimRefused,
                static_cast<unsigned long long>(m_stepCosts.drops() - counts.j6DropsAtStart),
                restores,
                formatPercentiles(samples.shadowRestoreUs.values, false).c_str(),
                counts.shadowRefused, overflow));

            startWindow_GameThread(nowSeconds);
        }

        void runBatch(const StepBatch& batch)
        {
            if (!m_loggedFirstStep)
            {
                m_loggedFirstStep = true;
                logWarning(printfString("[SimHost.FirstStep] role=%s thread=%s physicsStep=%llu batch=%u",
                    m_isAuthority ? "Authority" : "Client",
                    m_worlds->threadContext().isOwnerThreadForRole() ? "game" : "worker",
                    static_cast<unsigned long long>(batch.first), batch.count));
            }

            const BatchScopeUImpl batchScope(*this);
            for (uint32_t index = 0u; index < batch.count; ++index)
                runJoltStep_Step(batch.first + index, batch.deadlines[index]);
        }

        static inline thread_local const LegacyFrameHostModel* t_batchHost = nullptr;

        GameT*                             m_owner;
        WorldHostT*                        m_worlds;
        PresenterT*                        m_presenter;
        EngineT*                           m_engine;
        bool                               m_begun       = false;
        bool                               m_isAuthority = false;
        bool                               m_stepsInline = true;
        bool                               m_tickFunctionsRegistered = false;
        bool                               m_loggedFirstStep = false;
        double                             m_hostTimeSeconds = 0.0;
        double                             m_lastFrameStartSeconds = 0.0;
        std::optional<SimulationScheduler> m_scheduler;
        SpscRing<OccupancyCommandUImpl, kOccupancyCommandCapacity> m_occupancyCommands;
        SpscRing<StepCostSample, kStepCostSampleCapacity>          m_stepCosts;
        std::atomic<int64_t>               m_tickOffset{0};
        HostWindowCountsUImpl              m_windowCounts;
        HostWindowSamplesUImpl             m_windowSamples;
        Handle                             m_pendingSteps;
        Handle                             m_blockingSteps;
        FrameHostDoors                     m_doors;
        std::optional<StepHooksT>          m_stepHooks;
        std::optional<StepDriverT>         m_stepDriver;
    };
} // namespace legacyFrameHost
