// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
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

#endif
