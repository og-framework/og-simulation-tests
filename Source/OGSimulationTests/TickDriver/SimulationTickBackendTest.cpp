// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/ServerTickClock.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/ResimGateProbe.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTickBackend.h"
#include "OGSimulation/SimulationTickDriver.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGSimulation/TickDriverConcepts.h"
#include "OGSimulation/TickDriverPresenter.h"
#include "OGSimulation/TickDriverThreadContext.h"
#include "OGSimulation/TickDriverWindow.h"
#include "TickDriverFakes.h"

namespace simulationTickBackendTest
{
    using namespace tickDriverFakes;
    using tickDriverThread::kAuthorityRoleBit;

    constexpr float kDt = 1.f / 60.f;
    constexpr uint32_t kRingDepthTicks = 64u;

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

    struct NoInput
    {
        int unused = 0;
    };

    struct RootPart { LinearBodyState bodyState; };
    struct WeaponPart { PhysicsBodyState bodyState; };

    using TestState = SimulationComposite<RootPart, WeaponPart>;

    struct TestAllState
    {
        TestState state;
        const TestState& getState() const { return state; }
        TestState& editState() { return state; }
    };

    struct TestBodyDescriptor
    {
        struct Body
        {
            bool isRoot = false;
        };
        Body body;
    };

    struct RootDeclaration
    {
        using StateType = RootPart;
        static const LinearBodyState& bodyStateOf(const RootPart& part) { return part.bodyState; }
        static LinearBodyState& bodyStateOf(RootPart& part) { return part.bodyState; }
        static TestBodyDescriptor descriptor() { return TestBodyDescriptor{ { true } }; }
    };

    struct WeaponDeclaration
    {
        using StateType = WeaponPart;
        static const PhysicsBodyState& bodyStateOf(const WeaponPart& part) { return part.bodyState; }
        static PhysicsBodyState& bodyStateOf(WeaponPart& part) { return part.bodyState; }
        static TestBodyDescriptor descriptor() { return TestBodyDescriptor{ { false } }; }
    };

    using TestComposite = SimulationComposite<RootDeclaration, WeaponDeclaration>;

    class TestSimulatable
    {
    public:
        using StateType = TestState;
        using InputType = NoInput;

        const TestAllState& getAllState() const { return m_all; }
        TestAllState& editAllState() { return m_all; }
        void updateVizState() { m_viz = m_all; }
        const TestAllState& getVizState() const { return m_viz; }
        TestAllState& editVizState() { return m_viz; }
        const TestComposite& getPhysicsComposite() const { return m_composite; }

    private:
        TestAllState m_all;
        TestAllState m_viz;
        TestComposite m_composite;
    };

    struct TestStaticData
    {
    };

    using Storage = SimulationObjectStorage<TestSimulatable>;
    using Snapshot = RenderSnapshotT<static_cast<size_t>(kMaxSimulatableSlots) * 2u>;
    using Game = BasicRecordingGame<TestManager, Storage, Snapshot>;
    using Backend = SimulationTickBackend<Game, FakeWorldHost, TestSimulatable, RecordingEngine>;

    static_assert(TickBackendWorldHost<FakeWorldHost, TestSimulatable>,
        "the kit's world host models TickBackendWorldHost for the test's simulatable");
    static_assert(std::is_same_v<Backend::Presenter, TickDriverPresenter<Snapshot, RecordingEngine, Backend::RenderSource>>,
        "the backend's presenter is the presenter over the game's render snapshot and the backend's own render source");
    static_assert(TickDriverRenderSource<Backend::RenderSource>, "the backend's render source models TickDriverRenderSource");

    struct Rig
    {
        explicit Rig(bool authority, bool runInline = false)
            : isAuthority(authority)
            , host(events, kConfigureAtBuild, kRingDepthTicks)
            , manager(!authority, events)
            , game(events, manager, storage)
            , engine(events)
            , backend(game, host, engine)
        {
            config.isAuthority = authority;
            config.runInline = runInline;
        }

        void build()
        {
            StaticWorldDescription queryable;
            queryable.shapes.resize(2u);
            StaticWorldDescription physicsOnly;
            physicsOnly.shapes.resize(1u);
            backend.build(config, queryable, physicsOnly, bodyAdapter, queryAdapter);
        }

        Backend::Driver& buildAndStart()
        {
            build();
            return backend.start(static_cast<double>(kDt), isAuthority, 60);
        }

        void frame(float deltaSeconds)
        {
            backend.driver().startFrame(static_cast<double>(deltaSeconds));
            backend.driver().endFrame(nullptr, 0);
        }

        bool slotOccupiedInTheStepWorld(uint32_t slot)
        {
            return host.stepWorld().inner().live().occupancy.occupied.test(slot);
        }

        bool isAuthority;
        Events events;
        FakeWorldHost::Config config;
        FakeWorldHost host;
        TestManager manager;
        Storage storage;
        Game game;
        RecordingEngine engine;
        std::optional<FakeAdapter> bodyAdapter;
        std::optional<FakeAdapter> queryAdapter;
        Backend backend;
    };

    size_t indexOf(const Events& events, const std::string& event)
    {
        return static_cast<size_t>(std::distance(events.begin(), std::find(events.begin(), events.end(), event)));
    }
} // namespace simulationTickBackendTest

using namespace simulationTickBackendTest;

TEST_CASE("SimulationTickBackend.KitFix.TheDriverStepsOverTheKitsWorldHostWithoutNestingStepScopes", "[TickDriver]")
{
    Events events;
    FakeWorldHost host(events, kAuthorityRoleBit, true, kRingDepthTicks);
    TestManager manager(false, events);
    FakeStorage storage;
    BasicRecordingGame<TestManager> game(events, manager, storage);
    FakePresenter presenter(events);
    FakeEngine engine;
    using Driver = SimulationTickDriver<BasicRecordingGame<TestManager>, FakeWorldHost, FakePresenter, FakeEngine>;
    Driver driver(game, host, presenter, engine,
        Driver::Config{ static_cast<double>(kDt), true, StepThreading::Inline, 60 });

    {
        const auto worldLock = host.lockForStep();
        CHECK(host.threadContext().isWorldLockHeldOnThisThread());
        CHECK_FALSE(host.threadContext().isStepRunningOnThisThread());
    }

    driver.startFrame(static_cast<double>(2.5f * kDt));
    driver.endFrame(nullptr, 0);

    CHECK(host.stepWorld().steps() == 3u);
    CHECK(std::count(events.begin(), events.end(), std::string("onStepStarted")) == 3);
    CHECK(std::count(events.begin(), events.end(), std::string("onStepEnded")) == 3);
    CHECK(presenter.published.size() == 3u);
    CHECK_FALSE(host.threadContext().isStepRunningOnThisThread());
    CHECK_FALSE(host.threadContext().isWorldLockHeldOnThisThread());
    driver.stop();
}

TEST_CASE("SimulationTickBackend.Build.ConfiguresTheThreadContextBeforeAnyAdapterIsEmplaced", "[TickDriver]")
{
    const bool authority = GENERATE(true, false);
    Rig rig(authority);

    rig.build();

    CHECK(rig.events == Events{ authority ? "configure:authority:inline" : "configure:client:worker", "emplace:body",
        "emplace:query" });
    REQUIRE(rig.bodyAdapter.has_value());
    REQUIRE(rig.queryAdapter.has_value());
    CHECK(rig.bodyAdapter->stepWorldPredicate == rig.host.threadContext().stepWorldPredicate());
    CHECK(rig.queryAdapter->stepWorldPredicate == rig.host.threadContext().stepWorldPredicate());
    CHECK(rig.queryAdapter->stepWorldPredicate != nullptr);
    CHECK(rig.host.threadContext().isOwnerThreadForRole());
    CHECK(rig.host.builtStatics == std::vector<std::pair<size_t, size_t>>{ { 2u, 1u } });
}

TEST_CASE("SimulationTickBackend.Start.EmplacesThePresenterBeforeTheDriver", "[TickDriver]")
{
    const bool authority = GENERATE(true, false);
    Rig rig(authority);
    rig.build();
    rig.events.clear();
    rig.host.rootIndex = 1u;

    Backend::Driver& driver = rig.backend.start(static_cast<double>(kDt), authority, 60);

    REQUIRE(rig.events.size() >= 3u);
    CHECK(Events(rig.events.begin(), rig.events.begin() + 3) == Events{ "bindIntegratedTickSource", "rootDeclarationIndex",
        "occ" });
    CHECK(indexOf(rig.events, "clock") > 2u);
    CHECK(indexOf(rig.events, "clock") < rig.events.size());
    CHECK(std::count(rig.events.begin(), rig.events.end(), std::string("rootDeclarationIndex")) == 1);
    CHECK(&driver == &rig.backend.driver());
    CHECK(rig.host.lock().locks() == 1u);
    REQUIRE(rig.host.integratedTick);
    CHECK(rig.host.integratedTick() == 0u);
    CHECK(driver.getDiagnostics().isRunning());
    rig.backend.stop();
}

TEST_CASE("SimulationTickBackend.Stop.WaitsForTheOutstandingStepTaskBeforeTheWorldHostGoes", "[TickDriver]")
{
    Events events;
    {
        Rig rig(false);
        rig.buildAndStart();
        rig.backend.driver().startFrame(static_cast<double>(2.5f * kDt));
        REQUIRE(rig.engine.queuedTasks() == 1u);
        REQUIRE(rig.backend.driver().getDiagnostics().hasPendingSteps());

        rig.backend.stop();
        rig.events.push_back("worldHostTeardown");

        CHECK(rig.engine.queuedTasks() == 0u);
        CHECK(rig.engine.tasksRun == 1u);
        CHECK_FALSE(rig.backend.driver().getDiagnostics().hasPendingSteps());
        CHECK_FALSE(rig.backend.driver().getDiagnostics().isRunning());
        CHECK(rig.host.stepWorld().steps() == 3u);
        const size_t teardown = indexOf(rig.events, "worldHostTeardown");
        CHECK(teardown == rig.events.size() - 1u);
        CHECK(indexOf(rig.events, "onStepEnded") < teardown);

        rig.backend.stop();
        CHECK(rig.engine.tasksRun == 1u);
        rig.engine.runAllTasks();
        events = rig.events;
    }
    CHECK(events.back() == "worldHostTeardown");
}

TEST_CASE("SimulationTickBackend.Stop.BeforeStartIsANoOp", "[TickDriver]")
{
    Rig rig(true);
    rig.build();
    rig.backend.stop();
    CHECK(rig.events == Events{ "configure:authority:inline", "emplace:body", "emplace:query" });
}

TEST_CASE("SimulationTickBackend.Unregister.TheReleaseFreesTheSlotAndPushesItsVacancyToTheDriver", "[TickDriver]")
{
    Rig rig(true);
    rig.buildAndStart();
    TestSimulatable simulatable;
    const TestStaticData staticData;
    std::vector<uint32_t> published;

    REQUIRE(rig.backend.acquireAndBind(7u, "seven", true, std::nullopt,
        [&published](uint32_t slot) { published.push_back(slot); }, 3u, simulatable, staticData));
    CHECK(published == std::vector<uint32_t>{ 0u });
    rig.backend.onRegistered(3u, 7u);
    rig.frame(1.5f * kDt);
    CHECK(rig.slotOccupiedInTheStepWorld(0u));

    rig.events.clear();
    rig.backend.forgetPresenterTarget(3u, 7u);
    rig.backend.releaseSlot(7u, 3u);

    CHECK(rig.events == Events{ "release:7:0:3" });
    CHECK_FALSE(rig.host.slotOf(7u).has_value());
    CHECK(rig.slotOccupiedInTheStepWorld(0u));
    rig.frame(kDt);
    CHECK_FALSE(rig.slotOccupiedInTheStepWorld(0u));

    rig.events.clear();
    rig.backend.releaseSlot(7u, 3u);
    CHECK(rig.events.empty());
    rig.backend.stop();
}

TEST_CASE("SimulationTickBackend.Present.TheShadowRestoreNoteReachesTheDriversWindow", "[TickDriver]")
{
    Rig rig(false);
    rig.host.shadowWorld = true;
    rig.host.restoreSeconds = 0.0025;
    rig.host.restoreRefused = true;
    Backend::Driver& driver = rig.buildAndStart();
    FakeRenderSink sink;
    rig.engine.frame = 41u;
    rig.events.clear();

    rig.backend.present(PresentControls{}, sink);

    CHECK(indexOf(rig.events, "restore:41") < rig.events.size());
    CHECK(driver.getDiagnostics().windowSampleCount(TickDriverWindowSeries::ShadowRestoreUs) == 1u);
    CHECK(driver.getDiagnostics().windowCounts().shadowRefused == 1u);

    rig.host.restoreRefused = false;
    rig.backend.present(PresentControls{}, sink);
    CHECK(driver.getDiagnostics().windowSampleCount(TickDriverWindowSeries::ShadowRestoreUs) == 2u);
    CHECK(driver.getDiagnostics().windowCounts().shadowRefused == 1u);

    rig.host.shadowWorld = false;
    rig.backend.present(PresentControls{}, sink);
    CHECK(driver.getDiagnostics().windowSampleCount(TickDriverWindowSeries::ShadowRestoreUs) == 2u);
    rig.backend.stop();
}

TEST_CASE("SimulationTickBackend.Doors.ForwardToTheWorldHostThePresenterAndTheDriver", "[TickDriver]")
{
    Rig rig(false);
    Backend::Driver& driver = rig.buildAndStart();
    TestSimulatable simulatable;
    const TestStaticData staticData;
    uint32_t publishes = 0u;
    const auto publish = [&publishes](uint32_t) { ++publishes; };

    rig.events.clear();
    CHECK_FALSE(rig.backend.acquireAndBind(5u, "five", false, std::nullopt, publish, 9u, simulatable, staticData));
    CHECK(rig.events == Events{ "acquire:5:wait" });

    rig.host.bindLockWaitSeconds = 0.004;
    rig.events.clear();
    CHECK(rig.backend.acquireAndBind(5u, "five", false, std::optional<uint32_t>(2u), publish, 9u, simulatable, staticData));
    CHECK(rig.events == Events{ "acquire:5:2", "bind:2:9" });
    CHECK(publishes == 0u);
    CHECK(driver.getDiagnostics().windowSampleCount(TickDriverWindowSeries::J2WaitUs) == 1u);

    CHECK(rig.backend.editVizQuery().id == 1u);
    CHECK(std::as_const(rig.backend).getVizReader().id == 2u);
    const std::optional<QueryVolumeId> volume =
        rig.backend.registerVizVolume(QueryVolumeDescriptor{}, 5u, "five", false, std::nullopt, publish);
    REQUIRE(volume.has_value());
    CHECK(volume->value == 2u);

    Backend::RenderSource source(rig.backend);
    CHECK(&source.storage() == &rig.storage);
    CHECK(source.savesTicks() == driver.savesTicks());
    CHECK(source.grantedResims() == driver.grantedResims());
    CHECK(source.integratedTick() == rig.manager.currentIntegratedTick());

    rig.backend.onRegistered(9u, 5u);
    rig.backend.updateVisualization();
    rig.backend.forgetPresenterTarget(9u, 5u);
    rig.backend.forgetPresenterTarget(9u, 99u);
    rig.backend.stop();
}

namespace simulationTickBackendLoopTest
{
    using namespace simulationTickBackendTest;

    class LoopEngine : public FakeEngine
    {
    public:
        explicit LoopEngine(Events& events)
            : m_events(&events)
        {
        }

        void log(HostLogLevel level, const char* line)
        {
            m_events->push_back(std::string("log:") + line);
            FakeEngine::log(level, line);
        }

    private:
        Events* m_events;
    };

    static_assert(TickDriverEngine<LoopEngine>, "LoopEngine must model TickDriverEngine");

    using IdleGame = BasicRecordingIdleGame<true, TestManager, Storage, Snapshot>;
    using LoopBackend = SimulationTickBackend<IdleGame, FakeWorldHost, TestSimulatable, LoopEngine>;

    static_assert(TickDriverIdleGame<IdleGame>, "the loop rig's game models TickDriverIdleGame");

    struct LoopRig
    {
        LoopRig()
            : engine(events)
            , host(events, kConfigureAtBuild, kRingDepthTicks)
            , manager(false, events)
            , game(events, manager, storage, engine)
            , runner(engine, &events)
            , backend(game, host, engine)
        {
            config.isAuthority = true;
            config.threading = StepThreading::FreeRunning;
        }

        template <typename AfterBuildF, typename AfterStartF>
        void run(double idleSlackSeconds, AfterBuildF&& afterBuild, AfterStartF&& afterStart)
        {
            StaticWorldDescription queryable;
            queryable.shapes.resize(2u);
            StaticWorldDescription physicsOnly;
            physicsOnly.shapes.resize(1u);
            backend.runAuthorityLoop(config, std::move(queryable), std::move(physicsOnly),
                AuthorityLoopConfig{ static_cast<double>(kDt), 60, idleSlackSeconds }, runner, stop,
                std::forward<AfterBuildF>(afterBuild), std::forward<AfterStartF>(afterStart), bodyAdapter, queryAdapter);
        }

        Events events;
        FakeWorldHost::Config config;
        LoopEngine engine;
        FakeWorldHost host;
        TestManager manager;
        Storage storage;
        IdleGame game;
        FakeRunnerHost runner;
        std::atomic<bool> stop{ false };
        std::optional<FakeAdapter> bodyAdapter;
        std::optional<FakeAdapter> queryAdapter;
        LoopBackend backend;
    };

    class AnchorReadingSink : public FakeRenderSink
    {
    public:
        std::optional<glm::vec3> readAnchor(uint32_t id) const
        {
            ++reads[id];
            return FakeRenderSink::readAnchor(id);
        }

        mutable std::map<uint32_t, uint32_t> reads;
    };

    uint32_t anchorReadsOfAPresentWithoutTheDrain(LoopRig& rig, const PresentControls& controls, AnchorReadingSink& sink,
        uint32_t id)
    {
        sink.reads.clear();
        rig.backend.presenter().present_GameThread(rig.backend.driver(), controls, sink, rig.host);
        return sink.reads[id];
    }

    bool startsWith(const std::string& text, const std::string& prefix)
    {
        return text.rfind(prefix, 0) == 0u;
    }

    std::string frameLineOf(const FakeEngine& engine)
    {
        for (const LoggedLine& line : engine.lines)
        {
            if (startsWith(line.text, "[SimHost.Frame]"))
                return line.text;
        }
        return std::string();
    }
} // namespace simulationTickBackendLoopTest

using namespace simulationTickBackendLoopTest;

TEST_CASE("SimulationTickBackend.AuthorityLoop.RunsBuildAfterBuildStartAfterStartAndTheLoopInOrder", "[TickDriver]")
{
    {
        LoopRig rig;
        rig.game.setStopAfterSteps(rig.stop, 3u);
        LoopBackend::Driver* started = nullptr;
        rig.run(0.0005,
            [&rig] { rig.events.push_back("afterBuild"); },
            [&rig, &started](LoopBackend::Driver& driver)
            {
                rig.events.push_back("afterStart");
                started = &driver;
            });

        Events ordered;
        for (const std::string& event : rig.events)
        {
            if (startsWith(event, "configure:") || startsWith(event, "emplace:") || event == "afterBuild"
                || startsWith(event, "log:[SimHost.Frame]") || event == "afterStart" || event == "onStepStarted")
                ordered.push_back(event);
        }
        REQUIRE(ordered.size() >= 7u);
        CHECK(Events(ordered.begin(), ordered.begin() + 7) == Events{
            "configure:authority:runner",
            "emplace:body",
            "emplace:query",
            "afterBuild",
            "log:[SimHost.Frame] role=Authority stepping=runner dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] "
            "timeBase=monotonic idleSlackUs=500",
            "afterStart",
            "onStepStarted",
        });
        CHECK(started == &rig.backend.driver());
        CHECK(rig.game.stepsEnded.load() == 3u);
        CHECK(rig.host.builtStatics == std::vector<std::pair<size_t, size_t>>{ { 2u, 1u } });
        CHECK(rig.host.threadContext().threading() == StepThreading::FreeRunning);
        rig.backend.stop();
        CHECK_FALSE(rig.backend.driver().getDiagnostics().isRunning());
    }
    {
        LoopRig rig;
        rig.run(0.0, [] {}, [&rig](LoopBackend::Driver&) { rig.stop.store(true); });
        CHECK(std::count(rig.events.begin(), rig.events.end(), std::string("onStepStarted")) == 0);
        CHECK(std::count(rig.events.begin(), rig.events.end(), std::string("beforePump")) == 0);
        CHECK(std::count_if(rig.events.begin(), rig.events.end(),
            [](const std::string& event) { return startsWith(event, "log:[SimHost.Frame]"); }) == 1);
        CHECK(rig.game.stepsEnded.load() == 0u);
        CHECK(rig.game.idleCalls.empty());
        rig.backend.stop();
    }
}

TEST_CASE("SimulationTickBackend.AuthorityLoop.PresenterRegistrationsCrossThroughTheRingToPresent", "[TickDriver]")
{
    LoopRig rig;
    rig.game.setStopAfterSteps(rig.stop, 8u);
    TestSimulatable simulatable;
    const TestStaticData staticData;
    AnchorReadingSink sink;
    PresentControls controls;
    controls.newestOnly = true;

    bool bound = false;
    uint32_t readsBeforeTheDrain = 99u;
    uint32_t readsAfterPresent = 0u;
    uint32_t readsAfterForgetBeforeTheDrain = 0u;
    uint32_t readsAfterForgetAndPresent = 99u;
    std::vector<std::pair<uint32_t, bool>> occupancyBeforeRegistering;
    uint32_t wakes = 0u;
    rig.game.beforePumpHook = [&]
    {
        if (++wakes != 4u)
            return;
        bound = rig.backend.acquireAndBind(7u, "seven", true, std::nullopt, [](uint32_t) {}, 3u, simulatable, staticData);
        rig.backend.onRegistered(3u, 7u);
        readsBeforeTheDrain = anchorReadsOfAPresentWithoutTheDrain(rig, controls, sink, 3u);
        rig.backend.present(controls, sink);
        readsAfterPresent = anchorReadsOfAPresentWithoutTheDrain(rig, controls, sink, 3u);
        rig.backend.forgetPresenterTarget(3u, 7u);
        readsAfterForgetBeforeTheDrain = anchorReadsOfAPresentWithoutTheDrain(rig, controls, sink, 3u);
        rig.backend.present(controls, sink);
        readsAfterForgetAndPresent = anchorReadsOfAPresentWithoutTheDrain(rig, controls, sink, 3u);
        rig.backend.releaseSlot(7u, 3u);
    };

    rig.run(0.0, [] {}, [](LoopBackend::Driver&) {});

    CHECK(wakes >= 4u);
    CHECK(bound);
    CHECK(readsBeforeTheDrain == 0u);
    CHECK(readsAfterPresent >= 1u);
    CHECK(readsAfterForgetBeforeTheDrain >= 1u);
    CHECK(readsAfterForgetAndPresent == 0u);
    CHECK_FALSE(rig.host.slotOf(7u).has_value());
    CHECK(std::count(rig.events.begin(), rig.events.end(), std::string("bind:0:3")) == 1);
    CHECK(std::count(rig.events.begin(), rig.events.end(), std::string("release:7:0:3")) == 1);
    rig.backend.stop();
}

TEST_CASE("SimulationTickBackend.Start.ThreadingComesFromTheWorldHostsContext", "[TickDriver]")
{
    const auto startedFrameLine = [](Rig& rig, double idleSlackSeconds)
    {
        rig.build();
        rig.backend.start(static_cast<double>(kDt), rig.isAuthority, 60, idleSlackSeconds);
        const std::string line = frameLineOf(rig.engine);
        rig.backend.stop();
        return line;
    };

    {
        Rig rig(true);
        CHECK(startedFrameLine(rig, 0.0)
            == "[SimHost.Frame] role=Authority stepping=inline dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] timeBase=worldDelta");
        CHECK(rig.host.threadContext().threading() == StepThreading::Inline);
    }
    {
        Rig rig(false, true);
        CHECK(startedFrameLine(rig, 0.0)
            == "[SimHost.Frame] role=Client stepping=inline dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] timeBase=worldDelta");
        CHECK(rig.host.threadContext().threading() == StepThreading::Inline);
    }
    {
        Rig rig(false);
        CHECK(startedFrameLine(rig, 0.0)
            == "[SimHost.Frame] role=Client stepping=worker dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] timeBase=worldDelta");
        CHECK(rig.host.threadContext().threading() == StepThreading::FrameBounded);
    }
    {
        Rig rig(true);
        rig.config.threading = StepThreading::FreeRunning;
        CHECK(startedFrameLine(rig, 0.00025)
            == "[SimHost.Frame] role=Authority stepping=runner dt=0.016667 maxCatchUpSteps=60 rateScale=[0.75,1.25] "
               "timeBase=monotonic idleSlackUs=250");
        CHECK(rig.host.threadContext().threading() == StepThreading::FreeRunning);
    }
}

TEST_CASE("SimulationTickBackend.FreeRunning.AFrameThreadBindIsRefused", "[.][TickBackendFreeRunningFrameThreadBindRefused]")
{
    Rig rig(true);
    rig.config.threading = StepThreading::FreeRunning;
    std::thread runner([&rig] { rig.build(); });
    runner.join();
    REQUIRE(rig.bodyAdapter.has_value());

    TestSimulatable simulatable;
    const TestStaticData staticData;
    rig.backend.acquireAndBind(7u, "seven", true, std::nullopt, [](uint32_t) {}, 3u, simulatable, staticData);
    FAIL("SimulationTickBackend::acquireAndBind accepted a frame-thread bind on a free-running authority");
}

#endif
