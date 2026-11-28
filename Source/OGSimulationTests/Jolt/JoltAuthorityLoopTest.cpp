// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"
#include "../TickDriver/TickDriverFakes.h"

#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsBodyReaderAdapter.h"
#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"
#include "OGSimulationJolt/JoltTickWorldHost.h"

#include "glm/gtc/matrix_transform.hpp"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/ServerTickClock.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/PhysicsDeclaration.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/ResimGateProbe.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGTickDriver/SimulationTickBackend.h"
#include "OGSimulation/StepHooks.h"
#include "OGTickDriver/TickDriverPresenter.h"
#include "OGTickDriver/TickDriverThreadContext.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{
    using tickDriverFakes::Events;
    using tickDriverFakes::FakeEngine;
    using tickDriverFakes::IdleScript;
    using tickDriverFakes::LoggedLine;
    using tickDriverFakes::RealClockFakeEngine;

    constexpr auto kStartGiveUp = std::chrono::seconds(5);
    constexpr auto kStepsGiveUp = std::chrono::seconds(5);
    constexpr uint32_t kFunctionalSteps = 30u;
    constexpr uint32_t kEquivalenceTicks = 300u;
    constexpr uint32_t kCoarseIdleSteps = 600u;
    constexpr double kNonRepresentableOriginSeconds = 0.1;

    double widenedDtSeconds()
    {
        const uint32_t bits = std::bit_cast<uint32_t>(joltTestRig::kDt);
        return static_cast<double>(std::bit_cast<float>(bits));
    }

    struct HostStaticData
    {
        glm::vec3 radialOffset = glm::vec3(0.7f, 0.3f, -0.f);
        float queryRadius = 0.5f;
    };

    struct HostRadialPart
    {
        PhysicsBodyState bodyState;
    };

    struct HostCapsulePart
    {
        PhysicsBodyState bodyState;
    };

    struct HostRadialDeclaration
    {
        using StateType = HostRadialPart;
        static constexpr const char* name = "hostRadial";

        static const PhysicalObjectDescriptor& descriptor()
        {
            static const PhysicalObjectDescriptor radial =
                joltTestRig::sphereTemplate(0.3f, joltTestRig::bit(joltTestRig::kBody), 0u, 0).descriptor;
            return radial;
        }

        static const HostStaticData& staticDataOf(const HostStaticData& staticData) { return staticData; }

        static std::vector<QueryVolumeDescriptor> queryVolumes(const HostStaticData& staticData)
        {
            return { QueryVolumeDescriptor{ SphereGeometry{ staticData.queryRadius },
                CollisionCategories{ joltTestRig::bit(joltTestRig::kCharacter) }, glm::mat4(1.f), joltTestRig::kQueryRouting } };
        }

        static glm::vec3 attachmentOffset(const HostStaticData& staticData) { return staticData.radialOffset; }

        static PhysicsBodyState& bodyStateOf(StateType& state) { return state.bodyState; }
        static const PhysicsBodyState& bodyStateOf(const StateType& state) { return state.bodyState; }

        PhysicsRuntimeBindings bindings;
    };

    struct HostCapsuleDeclaration
    {
        using StateType = HostCapsulePart;
        static constexpr const char* name = "hostCapsule";

        static const PhysicalObjectDescriptor& descriptor()
        {
            static const PhysicalObjectDescriptor capsule = joltTestRig::capsuleTemplate(joltTestRig::bit(joltTestRig::kCharacter),
                joltTestRig::bit(joltTestRig::kWorld) | joltTestRig::bit(joltTestRig::kCharacter), 1).descriptor;
            return capsule;
        }

        static const HostStaticData& staticDataOf(const HostStaticData& staticData) { return staticData; }
        static std::vector<QueryVolumeDescriptor> queryVolumes(const HostStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const HostStaticData&) { return glm::vec3(0.f); }

        static PhysicsBodyState& bodyStateOf(StateType& state) { return state.bodyState; }
        static const PhysicsBodyState& bodyStateOf(const StateType& state) { return state.bodyState; }

        PhysicsRuntimeBindings bindings;
    };

    static_assert(PhysicsDeclaration<HostRadialDeclaration, HostStaticData>);
    static_assert(PhysicsDeclaration<HostCapsuleDeclaration, HostStaticData>);

    using HostComposite = SimulationComposite<HostRadialDeclaration, HostCapsuleDeclaration>;

    class HostSim
    {
    public:
        const HostComposite& getPhysicsComposite() const { return m_composite; }
        HostComposite& editPhysicsComposite() { return m_composite; }

    private:
        HostComposite m_composite;
    };

    StaticShapeDescriptor staticBoxOf(const glm::vec3& centre, const glm::vec3& halfExtents, uint64_t stableKey)
    {
        StaticShapeDescriptor descriptor;
        descriptor.shape = StaticBox{ halfExtents };
        descriptor.localToWorld = glm::translate(glm::mat4(1.f), centre);
        descriptor.categories = CollisionCategories{ joltTestRig::bit(joltTestRig::kWorld) };
        descriptor.blockingCategories = CollisionCategories{ joltTestRig::kAllMappedCategories };
        descriptor.friction = 0.7f;
        descriptor.restitution = 0.3f;
        descriptor.stableKey = stableKey;
        return descriptor;
    }

    StaticWorldDescription importedStatics()
    {
        StaticWorldDescription description;
        description.shapes.push_back(staticBoxOf(glm::vec3(0.f, -0.5f, -0.f), glm::vec3(15.f, 0.5f, 15.f), 1u));
        description.shapes.push_back(staticBoxOf(glm::vec3(0.f, 2.f, -9.f), glm::vec3(0.4f, 2.f, 0.4f), 2u));
        description.shapes.push_back(staticBoxOf(glm::vec3(0.f, 1.f, 9.f), glm::vec3(0.6f, 1.f, 0.2f), 3u));
        return description;
    }

    const std::vector<uint8_t> kImportedPhysicsOnly{ 0u, 1u, 0u };

    JoltTickWorldHostConfig authorityConfig(bool freeRunning)
    {
        JoltTickWorldHostConfig config;
        config.isAuthority = true;
        config.runInline = false;
        config.freeRunning = freeRunning;
        config.gravity = glm::vec3(0.f, -9.8f, -0.f);
        config.mappedCategories = joltTestRig::kAllMappedCategories;
        config.log = &joltTestRig::logJoltProblems;
        return config;
    }

    class LoopManager
    {
    public:
        explicit LoopManager(float dtSeconds)
            : m_serverClock(dtSeconds)
            , m_dtSeconds(dtSeconds)
        {
        }

        struct Clock
        {
            unsigned int getPredictionTick() const { return predictionTick; }

            unsigned int registerResyncCallback(ClientPredictionClock::ResyncCallback callback)
            {
                callbacks.push_back(std::move(callback));
                return static_cast<unsigned int>(callbacks.size() - 1u);
            }

            void unregisterResyncCallback(unsigned int) {}

            unsigned int predictionTick = 0u;
            std::vector<ClientPredictionClock::ResyncCallback> callbacks;
        };

        bool runsPrediction() const { return false; }
        Clock& editClientClock() { return m_clock; }
        const Clock& getClientClock() const { return m_clock; }
        const ServerTickClock& getServerClock() const { return m_serverClock; }
        ResimGateProbe& editResimGateProbe() { return m_probe; }

        void onGameSimulation(const SimulationUpdateInfo&)
        {
            m_serverClock.advanceTick();
            m_lastStep = SimulationTimeStep(m_serverClock.getTick(), false, StepKind::Normal, m_dtSeconds);
        }

        void onPostGameSimulation(const SimulationUpdateInfo&) {}
        unsigned int onCheckIsSimilar() { return std::numeric_limits<unsigned int>::max(); }
        void prepareResimulation(int32_t, uint32_t) {}

        uint32_t currentIntegratedTick() const { return m_lastStep.has_value() ? m_lastStep->getTick() : 0u; }
        const std::optional<SimulationTimeStep>& lastIntegratedStep() const { return m_lastStep; }

    private:
        Clock m_clock;
        ServerTickClock m_serverClock;
        ResimGateProbe m_probe;
        float m_dtSeconds;
        std::optional<SimulationTimeStep> m_lastStep;
    };

    using Storage = SimulationObjectStorage<>;
    using Snapshot = RenderSnapshotT<static_cast<size_t>(kMaxSimulatableSlots) * 2u>;
    using IdleGameBase = tickDriverFakes::BasicRecordingIdleGame<true, LoopManager, Storage, Snapshot>;

    struct RootStates
    {
        SimTick tick = 0u;
        std::array<PhysicsBodyState, kMaxSimulatableSlots> roots{};
    };

    class LoopGame : public IdleGameBase
    {
    public:
        LoopGame(Events& events, LoopManager& gameManager, Storage& gameStorage, FakeEngine& clock, IdleScript script)
            : IdleGameBase(events, gameManager, gameStorage, clock, std::move(script))
        {
        }

        void releaseInputsForAuthorityTick(SimTick tick)
        {
            IdleGameBase::releaseInputsForAuthorityTick(tick);
            m_tick = tick;
            if (scriptedInputs != nullptr)
                joltTestRig::applyScriptedForces(*scriptedInputs, tick);
        }

        void onStepEnded()
        {
            IdleGameBase::onStepEnded();
            if (!recordRoots || bodies == nullptr)
                return;
            RootStates states;
            states.tick = m_tick;
            for (uint32_t slot = 0u; slot < kMaxSimulatableSlots; ++slot)
                states.roots[slot] = bodies->captureBodyState(rootBodies[slot]);
            recorded.push_back(states);
        }

        JoltWorld* scriptedInputs = nullptr;
        const JoltPhysicsBodyAdapter* bodies = nullptr;
        std::array<BodyId, kMaxSimulatableSlots> rootBodies{};
        bool recordRoots = false;
        std::vector<RootStates> recorded;

    private:
        SimTick m_tick = 0u;
    };

    static_assert(TickDriverIdleGame<LoopGame>, "the loop rig's game models TickDriverIdleGame");

    template <typename EngineT>
    using LoopHost = JoltTickWorldHost<HostSim, std::mutex, EngineT>;

    template <typename EngineT>
    using LoopBackend = SimulationTickBackend<LoopGame, LoopHost<EngineT>, HostSim, EngineT>;

    template <typename EngineT>
    using RunnerOf = std::conditional_t<std::is_same_v<EngineT, FakeEngine>, tickDriverFakes::FakeRunnerHost,
        tickDriverFakes::RealRunnerHost>;

    struct Registration
    {
        uint64_t key = 0u;
        const char* name = "";
        unsigned int id = 0u;
        size_t character = 0u;
    };

    const std::array<Registration, 2> kCharacters{ Registration{ 21u, "BP_Brawler_C_0", 4u, 0u },
        Registration{ 22u, "BP_Brawler_C_1", 5u, 1u } };

    class AnchorReadingSink : public tickDriverFakes::FakeRenderSink
    {
    public:
        std::optional<glm::vec3> readAnchor(uint32_t id) const
        {
            ++reads[id];
            return tickDriverFakes::FakeRenderSink::readAnchor(id);
        }

        mutable std::map<uint32_t, uint32_t> reads;
    };

    template <typename EngineT>
    struct LoopRig
    {
        explicit LoopRig(IdleScript script)
            : game(events, manager, storage, virtualClock(), std::move(script))
            , host(engine)
            , runner(makeRunner(engine))
            , backend(game, host, engine)
        {
        }

        FakeEngine& virtualClock()
        {
            if constexpr (std::is_same_v<EngineT, FakeEngine>)
                return engine;
            else
                return spareClock;
        }

        static RunnerOf<EngineT> makeRunner(EngineT& clock)
        {
            if constexpr (std::is_same_v<EngineT, FakeEngine>)
                return RunnerOf<EngineT>(clock);
            else
                return RunnerOf<EngineT>{};
        }

        void attachRootBodies(bool scriptedInputs)
        {
            for (uint32_t slot = 0u; slot < kMaxSimulatableSlots; ++slot)
                game.rootBodies[slot] = joltTickWorldHost::slotRootBodyId(host.stepWorld(), host.getDiagnostics().slotTemplate(), slot);
            game.bodies = &*bodyAdapter;
            game.scriptedInputs = scriptedInputs ? &host.stepWorld() : nullptr;
        }

        void registerCharacter(const Registration& registration)
        {
            const bool bound = backend.acquireAndBind(registration.key, registration.name, true, std::nullopt, [](uint32_t) {},
                registration.id, characters[registration.character], HostStaticData{});
            if (!bound)
                return;
            backend.onRegistered(registration.id, registration.key);
            registered.fetch_add(1u);
        }

        void queueRegistration(const Registration& registration)
        {
            const std::lock_guard<std::mutex> guard(queueMutex);
            queue.push_back(registration);
        }

        void drainRegistrations()
        {
            std::vector<Registration> taken;
            {
                const std::lock_guard<std::mutex> guard(queueMutex);
                taken.swap(queue);
            }
            for (const Registration& registration : taken)
                registerCharacter(registration);
        }

        template <typename AfterBuildF, typename AfterStartF>
        void runAuthorityLoop(JoltTickStatics statics, AfterBuildF&& afterBuild, AfterStartF&& afterStart)
        {
            backend.runAuthorityLoop(authorityConfig(true), std::move(statics.queryable), std::move(statics.physicsOnly),
                AuthorityLoopConfig{ widenedDtSeconds(), 60, 0.0 }, runner, stop, std::forward<AfterBuildF>(afterBuild),
                std::forward<AfterStartF>(afterStart), bodyAdapter, readerAdapter, queryAdapter);
        }

        Events events;
        FakeEngine spareClock;
        EngineT engine;
        LoopManager manager{ joltTestRig::kDt };
        Storage storage;
        LoopGame game;
        LoopHost<EngineT> host;
        std::optional<JoltPhysicsBodyAdapter> bodyAdapter;
        std::optional<JoltPhysicsBodyReaderAdapter> readerAdapter;
        std::optional<JoltSpatialQueryAdapter> queryAdapter;
        std::array<HostSim, 2> characters;
        std::mutex queueMutex;
        std::vector<Registration> queue;
        std::atomic<uint32_t> registered{ 0u };
        double originLowerBoundSeconds = -std::numeric_limits<double>::infinity();
        RunnerOf<EngineT> runner;
        std::atomic<bool> stop{ false };
        LoopBackend<EngineT> backend;
    };

    JoltTickStatics splitImportedStatics()
    {
        return joltTickWorldHost::splitStatics(importedStatics(), kImportedPhysicsOnly);
    }

    uint32_t authorityOwnerCountOnThisThread()
    {
        return tickDriverThread::OwnerThreadCounts::onThisThread(tickDriverThread::kAuthorityRoleBit);
    }

    template <typename PredicateT>
    bool waitUntil(std::chrono::steady_clock::duration giveUpAfter, PredicateT predicate)
    {
        const auto giveUp = std::chrono::steady_clock::now() + giveUpAfter;
        while (!predicate())
        {
            if (std::chrono::steady_clock::now() >= giveUp)
                return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return true;
    }

    std::vector<std::string> linesWith(const std::vector<std::string>& lines, const std::string& prefix)
    {
        std::vector<std::string> result;
        for (const std::string& line : lines)
        {
            if (line.rfind(prefix, 0) == 0u)
                result.push_back(line);
        }
        return result;
    }

    bool contains(const std::string& text, const std::string& part)
    {
        return text.find(part) != std::string::npos;
    }

    std::array<uint32_t, 13> bitsOf(const PhysicsBodyState& state)
    {
        return { std::bit_cast<uint32_t>(state.position.x), std::bit_cast<uint32_t>(state.position.y),
            std::bit_cast<uint32_t>(state.position.z), std::bit_cast<uint32_t>(state.rotation.x),
            std::bit_cast<uint32_t>(state.rotation.y), std::bit_cast<uint32_t>(state.rotation.z),
            std::bit_cast<uint32_t>(state.rotation.w), std::bit_cast<uint32_t>(state.linearVelocity.x),
            std::bit_cast<uint32_t>(state.linearVelocity.y), std::bit_cast<uint32_t>(state.linearVelocity.z),
            std::bit_cast<uint32_t>(state.angularVelocity.x), std::bit_cast<uint32_t>(state.angularVelocity.y),
            std::bit_cast<uint32_t>(state.angularVelocity.z) };
    }

    int64_t firstDifferingTick(const std::vector<RootStates>& first, const std::vector<RootStates>& second, size_t ticks)
    {
        for (size_t index = 0u; index < ticks; ++index)
        {
            if (first[index].tick != second[index].tick)
                return static_cast<int64_t>(std::min(first[index].tick, second[index].tick));
            for (uint32_t slot = 0u; slot < kMaxSimulatableSlots; ++slot)
            {
                if (bitsOf(first[index].roots[slot]) != bitsOf(second[index].roots[slot]))
                    return static_cast<int64_t>(first[index].tick);
            }
        }
        return -1;
    }

    struct Distribution
    {
        double p50 = 0.0;
        double p99 = 0.0;
        double max = 0.0;
    };

    Distribution distributionOf(std::vector<double> values)
    {
        Distribution result;
        if (values.empty())
            return result;
        std::sort(values.begin(), values.end());
        const auto rank = [&values](double quantile)
        {
            const size_t index = static_cast<size_t>(std::ceil(quantile * static_cast<double>(values.size())));
            return values[std::clamp<size_t>(index, 1u, values.size()) - 1u];
        };
        result.p50 = rank(0.50);
        result.p99 = rank(0.99);
        result.max = values.back();
        return result;
    }

    std::string formatLateness(const char* label, const Distribution& lateUs, size_t samples)
    {
        char text[256];
        std::snprintf(text, sizeof(text), "%s samples=%zu lateUs p50=%.1f p99=%.1f max=%.1f", label, samples, lateUs.p50,
            lateUs.p99, lateUs.max);
        return std::string(text);
    }

    struct RealRunnerScript
    {
        bool registerCharacters = true;
        std::chrono::milliseconds sleepAfterRegistering{ 0 };
        uint32_t stepsToWaitFor = kFunctionalSteps;
        std::chrono::steady_clock::duration stepsGiveUp = kStepsGiveUp;
    };

    struct RealRunnerRun
    {
        bool started = false;
        bool stepsReached = false;
        uint32_t registered = 0u;
        double sleepStartSeconds = 0.0;
        double sleepEndSeconds = 0.0;
        std::map<uint32_t, uint32_t> anchorReads;
        bool testThreadMayAccess = true;
        uint32_t stepsAtJoin = 0u;
        uint32_t stepsAfterTheJoin = 0u;
        uint64_t physicsStepsAtJoin = 0u;
        bool runningAfterStop = true;
        LostTime lost;
        uint32_t maxStepsPerFrame = 0u;
        std::vector<double> stepStartedSeconds;
        double originLowerBoundSeconds = 0.0;
        std::vector<std::string> lines;
        std::array<PhysicsBodyState, 3> rootsAfterTheJoin{};
    };

    RealRunnerRun runOnARealRunner(const RealRunnerScript& script)
    {
        LoopRig<RealClockFakeEngine> rig(IdleScript{ tickDriverFakes::idleScript::RealDeadline{} });
        rig.game.beforePumpHook = [&rig]()
        {
            const double clockSeconds = rig.engine.nowSeconds();
            const double hostSeconds = rig.backend.driver().hostNowSeconds();
            rig.originLowerBoundSeconds = std::max(rig.originLowerBoundSeconds, clockSeconds - hostSeconds);
            rig.drainRegistrations();
        };

        RealRunnerRun run;
        std::promise<void> started;
        std::future<void> startedSeen = started.get_future();
        std::thread runner([&rig, &started, statics = splitImportedStatics()]() mutable
        {
            rig.runAuthorityLoop(std::move(statics), [&rig]() { rig.attachRootBodies(false); },
                [&started](LoopBackend<RealClockFakeEngine>::Driver&) { started.set_value(); });
        });

        run.started = startedSeen.wait_for(kStartGiveUp) == std::future_status::ready;
        if (run.started)
        {
            if (script.registerCharacters)
            {
                for (const Registration& registration : kCharacters)
                    rig.queueRegistration(registration);
            }
            if (script.sleepAfterRegistering.count() > 0)
            {
                run.sleepStartSeconds = tickDriverFakes::realClock::seconds();
                std::this_thread::sleep_for(script.sleepAfterRegistering);
                run.sleepEndSeconds = tickDriverFakes::realClock::seconds();
            }
            const uint32_t expectedRegistrations = script.registerCharacters ? static_cast<uint32_t>(kCharacters.size()) : 0u;
            run.stepsReached = waitUntil(script.stepsGiveUp, [&rig, &script, expectedRegistrations]()
            {
                return rig.game.stepsEnded.load() >= script.stepsToWaitFor && rig.registered.load() == expectedRegistrations;
            });
            if (run.stepsReached)
            {
                AnchorReadingSink sink;
                PresentControls controls;
                controls.newestOnly = true;
                rig.backend.present(controls, sink);
                run.anchorReads = sink.reads;
            }
        }
        rig.stop.store(true);
        runner.join();

        if (!run.started)
            return run;
        run.registered = rig.registered.load();
        run.testThreadMayAccess = rig.host.stepWorld().callerMayAccess();
        run.stepsAtJoin = rig.game.stepsEnded.load();
        run.physicsStepsAtJoin = rig.backend.driver().getDiagnostics().physicsStepCount();
        rig.backend.stop();
        run.runningAfterStop = rig.backend.driver().getDiagnostics().isRunning();
        run.lost = rig.backend.driver().getDiagnostics().lostTime();
        run.maxStepsPerFrame = rig.backend.driver().getDiagnostics().windowCounts().maxStepsPerFrame;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        run.stepsAfterTheJoin = rig.game.stepsEnded.load();
        run.stepStartedSeconds = rig.game.stepStartedSeconds;
        run.originLowerBoundSeconds = rig.originLowerBoundSeconds;
        for (const LoggedLine& line : rig.engine.snapshotLines())
            run.lines.push_back(line.text);
        {
            auto worldLock = rig.host.lockForStep();
            for (uint32_t slot = 0u; slot < run.rootsAfterTheJoin.size(); ++slot)
                run.rootsAfterTheJoin[slot] = rig.bodyAdapter->captureBodyState(rig.game.rootBodies[slot]);
        }
        return run;
    }

    std::vector<double> latenessFromTheFirstStepUs(const std::vector<double>& stepStartedSeconds, double dtSeconds)
    {
        std::vector<double> lateUs;
        for (size_t index = 0u; index < stepStartedSeconds.size(); ++index)
        {
            const double scheduled = stepStartedSeconds.front() + static_cast<double>(index) * dtSeconds;
            lateUs.push_back((stepStartedSeconds[index] - scheduled) * 1e6);
        }
        return lateUs;
    }

    std::vector<RootStates> runInlineAuthority()
    {
        LoopRig<FakeEngine> rig(IdleScript{ tickDriverFakes::idleScript::HonourDeadline{} });
        const JoltTickStatics statics = splitImportedStatics();
        rig.backend.build(authorityConfig(false), statics.queryable, statics.physicsOnly, rig.bodyAdapter, rig.readerAdapter,
            rig.queryAdapter);
        REQUIRE(rig.host.threadContext().threading() == StepThreading::Inline);
        rig.attachRootBodies(true);
        rig.game.recordRoots = true;
        const double dtSeconds = widenedDtSeconds();
        LoopBackend<FakeEngine>::Driver& driver = rig.backend.start(dtSeconds, true, 60);
        for (const Registration& registration : kCharacters)
            rig.registerCharacter(registration);
        REQUIRE(rig.registered.load() == 2u);

        driver.startFrame(0.0);
        driver.endFrame(nullptr, 0);
        while (rig.game.stepsEnded.load() < kEquivalenceTicks)
        {
            driver.startFrame(dtSeconds);
            driver.endFrame(nullptr, 0);
        }
        rig.backend.stop();
        return rig.game.recorded;
    }

    struct FreeRunningVirtualRun
    {
        std::vector<RootStates> recorded;
        bool threading = false;
        uint32_t earlyWakes = 0u;
        double originSeconds = 0.0;
    };

    FreeRunningVirtualRun runFreeRunningAuthorityInVirtualTime()
    {
        LoopRig<FakeEngine> rig(IdleScript{ tickDriverFakes::idleScript::HonourDeadline{} });
        rig.engine.now = kNonRepresentableOriginSeconds;
        rig.game.recordRoots = true;
        rig.game.setStopAfterSteps(rig.stop, kEquivalenceTicks);
        FreeRunningVirtualRun run;
        run.originSeconds = rig.engine.now;
        rig.runAuthorityLoop(splitImportedStatics(), [&rig]() { rig.attachRootBodies(true); },
            [&rig](LoopBackend<FakeEngine>::Driver&)
            {
                for (const Registration& registration : kCharacters)
                    rig.registerCharacter(registration);
            });
        run.threading = rig.host.threadContext().threading() == StepThreading::FreeRunning;
        run.earlyWakes = rig.backend.driver().getDiagnostics().windowCounts().earlyWakes;
        rig.backend.stop();
        run.recorded = rig.game.recorded;
        return run;
    }
} // namespace

TEST_CASE("JoltAuthorityLoop.ARunnerBuildsStepsBindsAndStopsInTheDesignedOrder", "[JoltTickWorldHost]")
{
    REQUIRE(authorityOwnerCountOnThisThread() == 0u);

    const RealRunnerRun run = runOnARealRunner(RealRunnerScript{});

    REQUIRE(run.started);
    REQUIRE(run.stepsReached);
    CHECK(run.registered == 2u);

    CHECK(linesWith(run.lines, "[SimHost.Mode] role=Authority stepping=runner viz=none").size() == 1u);
    const std::vector<std::string> frame = linesWith(run.lines, "[SimHost.Frame] role=Authority ");
    REQUIRE(frame.size() == 1u);
    CHECK(contains(frame.front(), " stepping=runner "));
    CHECK(contains(frame.front(), " timeBase=monotonic "));
    const std::vector<std::string> firstStep = linesWith(run.lines, "[SimHost.FirstStep] role=Authority ");
    REQUIRE(firstStep.size() == 1u);
    CHECK(contains(firstStep.front(), " thread=runner "));

    const glm::vec3 parking = JoltWorldConfig{}.parkingPosition;
    CHECK(glm::distance(run.rootsAfterTheJoin[0].position, parking) > 1e-3f);
    CHECK(glm::distance(run.rootsAfterTheJoin[1].position, parking) > 1e-3f);
    CHECK(run.rootsAfterTheJoin[2].position == parking);

    CHECK(run.anchorReads.size() == 2u);
    CHECK(run.anchorReads.count(4u) == 1u);
    CHECK(run.anchorReads.count(5u) == 1u);

    CHECK(run.stepsAtJoin >= kFunctionalSteps);
    CHECK(run.physicsStepsAtJoin == run.stepsAtJoin);
    CHECK(run.stepsAfterTheJoin == run.stepsAtJoin);
    CHECK_FALSE(run.runningAfterStop);

    CHECK_FALSE(run.testThreadMayAccess);
    CHECK(authorityOwnerCountOnThisThread() == 0u);
}

TEST_CASE("JoltAuthorityLoop.VirtualTimeStepsAreBitIdenticalToTheInlineAuthority", "[JoltTickWorldHost]")
{
    const std::vector<RootStates> inlineStates = runInlineAuthority();
    const FreeRunningVirtualRun freeRunning = runFreeRunningAuthorityInVirtualTime();

    REQUIRE(freeRunning.threading);
    CHECK(std::bit_cast<uint64_t>(freeRunning.originSeconds) == std::bit_cast<uint64_t>(kNonRepresentableOriginSeconds));
    CHECK(freeRunning.earlyWakes > 0u);
    REQUIRE(inlineStates.size() >= kEquivalenceTicks);
    REQUIRE(freeRunning.recorded.size() == kEquivalenceTicks);
    CHECK(inlineStates.front().tick == 1u);
    CHECK(inlineStates[kEquivalenceTicks - 1u].tick == kEquivalenceTicks);
    CHECK(bitsOf(inlineStates.front().roots[0]) != bitsOf(inlineStates[kEquivalenceTicks - 1u].roots[0]));
    CHECK(bitsOf(inlineStates.front().roots[1]) != bitsOf(inlineStates[kEquivalenceTicks - 1u].roots[1]));
    CHECK(firstDifferingTick(inlineStates, freeRunning.recorded, kEquivalenceTicks) == -1);
}

TEST_CASE("JoltAuthorityLoop.Timing.AGameThreadBlockOf200MsLeavesTheRunnerOnSchedule", "[.][OGSimTiming]")
{
    REQUIRE(authorityOwnerCountOnThisThread() == 0u);

    RealRunnerScript script;
    script.sleepAfterRegistering = std::chrono::milliseconds(200);
    const RealRunnerRun run = runOnARealRunner(script);

    REQUIRE(run.started);
    REQUIRE(run.stepsReached);
    const auto duringTheSleep = std::count_if(run.stepStartedSeconds.begin(), run.stepStartedSeconds.end(),
        [&run](double started) { return started >= run.sleepStartSeconds && started <= run.sleepEndSeconds; });
    const std::vector<double> lateUs = latenessFromTheFirstStepUs(run.stepStartedSeconds, widenedDtSeconds());
    WARN(formatLateness("AGameThreadBlockOf200Ms", distributionOf(lateUs), lateUs.size()) + " stepsDuringTheSleep="
        + std::to_string(duringTheSleep) + " sleepMs=" + std::to_string((run.sleepEndSeconds - run.sleepStartSeconds) * 1e3)
        + " maxStepsPerFrame=" + std::to_string(run.maxStepsPerFrame) + " lostSteps=" + std::to_string(run.lost.steps));
    CHECK(duringTheSleep >= 11);
    CHECK(run.lost.steps == 0u);
    CHECK(run.maxStepsPerFrame <= 2u);
}

TEST_CASE("JoltAuthorityLoop.Timing.ACoarseIdleOnThisHost", "[.][OGSimTiming]")
{
    REQUIRE(authorityOwnerCountOnThisThread() == 0u);

    RealRunnerScript script;
    script.registerCharacters = false;
    script.stepsToWaitFor = kCoarseIdleSteps;
    script.stepsGiveUp = std::chrono::seconds(30);
    const RealRunnerRun run = runOnARealRunner(script);

    REQUIRE(run.started);
    REQUIRE(run.stepsReached);
    REQUIRE(run.stepStartedSeconds.size() >= kCoarseIdleSteps);
    const double dtSeconds = widenedDtSeconds();
    std::vector<double> lateUs;
    uint32_t early = 0u;
    for (size_t index = 0u; index < kCoarseIdleSteps; ++index)
    {
        const double deadline = run.originLowerBoundSeconds + static_cast<double>(index) * dtSeconds;
        const double late = run.stepStartedSeconds[index] - deadline;
        lateUs.push_back(late * 1e6);
        early += late < -1e-9 ? 1u : 0u;
    }
    WARN(formatLateness("ACoarseIdleOnThisHost", distributionOf(lateUs), lateUs.size()) + " early=" + std::to_string(early));
    CHECK(early == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS
