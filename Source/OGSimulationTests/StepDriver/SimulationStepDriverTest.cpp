// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/ServerTickClock.h"
#include "OGSimulation/ResimGateProbe.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationStepDriver.h"
#include "OGSimulation/StepHooks.h"
#include "StepDriverTestSupport.h"

// ---------------------------------------------------------------------------
// SimulationStepDriver against MockPhysicsWorld and a RECORDING MOCK MANAGER.
//
// The mock manager models exactly what the driver relies on, using the core's own predicates:
//   * a scripted prediction clock that returns the AdvanceResults a test asks for (Normal, Stall,
//     Skip, HardResync), with ClientPredictionClock's tick arithmetic and resync callbacks;
//   * a correction-cache FRONTIER per registered character, filled by the same rule as
//     SimulationReconciliation::allocateFrontierSlotsAll (Skip backfills tick - 1;
//     stepAllocatesFrontierSlot gates the tick) and wiped by the resync callback;
//   * the resim NoSlot rule: a replayed tick integrates a character only if its cache holds a
//     slot for that tick;
//   * the apply edge: the replay step after which the resim cursor has caught up.
// Its integrate writes to the world (live value + 1000), so a snapshot taken after the integrate
// differs from one taken before it.
//
// The real SimulationManager runs the same driver in SimulationStepDriverRealManagerTest.cpp.
// ---------------------------------------------------------------------------

namespace
{
    using namespace stepDriverTestSupport;

    constexpr float kDt = 1.f / 60.f;
    constexpr unsigned int kCharacter = 1u;
    constexpr uint32_t kCharacterSlot = 0u;
    constexpr int64_t kIntegrateWrite = 1000;

    using AdvanceResult = ClientPredictionClock::AdvanceResult;

    // ClientPredictionClock's tick arithmetic with a scripted AdvanceResult per call.
    class ScriptedClock
    {
    public:
        using ResyncCallback = ClientPredictionClock::ResyncCallback;

        void script(AdvanceResult result, uint32_t times = 1u, unsigned int hardResyncTarget = 0u)
        {
            for (uint32_t i = 0; i < times; ++i)
            {
                m_script.push_back({ result, hardResyncTarget });
            }
        }

        AdvanceResult advancePrediction()
        {
            Scripted next{ AdvanceResult::Normal, 0u };
            if (!m_script.empty())
            {
                next = m_script.front();
                m_script.pop_front();
            }
            switch (next.result)
            {
            case AdvanceResult::Normal:
                normalAdvance();
                break;
            case AdvanceResult::Skip:
                if (m_predictionTick == m_resimulationTick)
                {
                    ++m_resimulationTick;
                }
                ++m_predictionTick;
                normalAdvance();
                break;
            case AdvanceResult::Stall:
                break;
            case AdvanceResult::HardResync:
                m_predictionTick = next.hardResyncTarget;
                m_resimulationTick = next.hardResyncTarget;
                for (auto& registered : m_callbacks)
                {
                    registered.second(next.hardResyncTarget);
                }
                break;
            }
            return next.result;
        }

        unsigned int getPredictionTick() const { return m_predictionTick; }
        unsigned int getResimulationTick() const { return m_resimulationTick; }
        bool isResimulating() const { return m_resimulationTick < m_predictionTick; }
        void startResimulation(unsigned int tick) { m_resimulationTick = tick; }
        void advanceResimulation() { ++m_resimulationTick; }

        unsigned int registerResyncCallback(ResyncCallback callback)
        {
            const unsigned int id = m_nextId++;
            m_callbacks.emplace_back(id, std::move(callback));
            return id;
        }

        void unregisterResyncCallback(unsigned int id)
        {
            m_callbacks.erase(std::remove_if(m_callbacks.begin(), m_callbacks.end(),
                [id](const auto& registered) { return registered.first == id; }), m_callbacks.end());
        }

        std::size_t callbackCount() const { return m_callbacks.size(); }

    private:
        struct Scripted
        {
            AdvanceResult result;
            unsigned int hardResyncTarget;
        };

        void normalAdvance()
        {
            if (m_predictionTick == m_resimulationTick)
            {
                ++m_resimulationTick;
            }
            ++m_predictionTick;
        }

        std::deque<Scripted> m_script;
        unsigned int m_predictionTick = 0u;
        unsigned int m_resimulationTick = 0u;
        std::vector<std::pair<unsigned int, ResyncCallback>> m_callbacks;
        unsigned int m_nextId = 0u;
    };

    struct ReplayRecord
    {
        SimTick tick = 0u;
        bool first = false;
        bool characterLive = false;
        bool characterIntegrated = false;
    };

    class RecordingManager
    {
    public:
        RecordingManager(bool runsPrediction, Events& events, LoggedWorld& world, uint32_t cacheDepthTicks)
            : m_runsPrediction(runsPrediction)
            , m_events(&events)
            , m_world(&world)
            , m_cacheDepthTicks(cacheDepthTicks)
            , m_serverClock(kDt)
        {
            if (runsPrediction)
            {
                // Registered first, like SimulationManager's own wipe callback.
                m_clock.registerResyncCallback([this](unsigned int) { m_cacheSlots.clear(); ++wipes; });
            }
        }

        bool runsPrediction() const { return m_runsPrediction; }
        ScriptedClock& editClientClock() { return m_clock; }
        const ScriptedClock& getClientClock() const { return m_clock; }
        const ServerTickClock& getServerClock() const { return m_serverClock; }
        ResimGateProbe& editResimGateProbe() { return m_probe; }
        const ResimGateProbe& probe() const { return m_probe; }

        void registerCharacter(unsigned int id) { m_registered.insert(id); }
        void unregisterCharacter(unsigned int id)
        {
            m_registered.erase(id);
            m_cacheSlots.erase(id);
        }

        void onGameSimulation(const SimulationUpdateInfo& info)
        {
            if (!m_runsPrediction)
            {
                m_serverClock.advanceTick();
                const SimTick tick = m_serverClock.getTick();
                const auto released = releasedInputs.find(tick);
                consumedInputs[tick] = released != releasedInputs.end() ? std::optional<int>(released->second) : std::nullopt;
                if (released != releasedInputs.end())
                {
                    releasedInputs.erase(released);
                }
                m_lastStep = SimulationTimeStep(tick, false, StepKind::Normal, kDt);
                m_events->push_back("sim");
                integrate();
                return;
            }

            if (info.isResimulation())
            {
                m_clock.advanceResimulation();
                const SimTick tick = m_clock.getResimulationTick();
                ReplayRecord record;
                record.tick = tick;
                record.first = info.isFirstResimulationStep();
                record.characterLive = m_world->inner().live().occupancy.occupied.test(kCharacterSlot);
                record.characterIntegrated = hasSlot(kCharacter, tick);
                replays.push_back(record);
                m_lastStep = SimulationTimeStep(tick, true, StepKind::Normal, kDt);
                m_events->push_back("simResim");
                integrate();
                return;
            }

            const AdvanceResult result = m_clock.advancePrediction();
            const SimTick tick = m_clock.getPredictionTick();
            const StepKind kind = result == AdvanceResult::Stall ? StepKind::Stall
                                : result == AdvanceResult::Skip  ? StepKind::Skip
                                                                 : StepKind::Normal;
            for (const unsigned int id : m_registered)
            {
                if (kind == StepKind::Skip)
                {
                    pushSlot(id, tick - 1u);
                }
                if (stepAllocatesFrontierSlot(kind))
                {
                    pushSlot(id, tick);
                }
            }
            m_lastStep = SimulationTimeStep(tick, false, kind, kDt);
            m_events->push_back("sim");
            integrate();
        }

        void onPostGameSimulation(const SimulationUpdateInfo& info)
        {
            if (info.isResimulation())
            {
                m_events->push_back("postResim");
                if (!m_clock.isResimulating())
                {
                    ++applyEdges;
                    m_probe.noteFinish();
                    pendingAnchor.reset();
                }
                return;
            }
            m_events->push_back("post");
        }

        unsigned int onCheckIsSimilar()
        {
            ++checks;
            return pendingAnchor.has_value() ? *pendingAnchor : std::numeric_limits<unsigned int>::max();
        }

        void prepareResimulation(int32_t physicsStep, uint32_t simTick)
        {
            m_events->push_back(tickEvent("prepare", simTick));
            preparedPhysicsSteps.push_back(physicsStep);
            m_probe.notePrepare(simTick);
            m_clock.startResimulation(simTick);
        }

        uint32_t currentIntegratedTick() const { return m_lastStep.has_value() ? m_lastStep->getTick() : 0u; }
        const std::optional<SimulationTimeStep>& lastIntegratedStep() const { return m_lastStep; }

        std::set<SimTick> frontier(unsigned int id) const
        {
            const auto it = m_cacheSlots.find(id);
            return it != m_cacheSlots.end() ? it->second : std::set<SimTick>{};
        }

        bool hasSlot(unsigned int id, SimTick tick) const
        {
            const auto it = m_cacheSlots.find(id);
            return it != m_cacheSlots.end() && it->second.count(tick) != 0u;
        }

        std::optional<unsigned int> pendingAnchor;
        std::map<SimTick, int> releasedInputs;
        std::map<SimTick, std::optional<int>> consumedInputs;
        std::vector<ReplayRecord> replays;
        std::vector<int32_t> preparedPhysicsSteps;
        uint32_t applyEdges = 0u;
        uint32_t checks = 0u;
        uint32_t wipes = 0u;

    private:
        void integrate()
        {
            m_world->inner().setLiveValue(m_world->inner().live().value + kIntegrateWrite);
        }

        void pushSlot(unsigned int id, SimTick tick)
        {
            std::set<SimTick>& slots = m_cacheSlots[id];
            slots.insert(tick);
            while (slots.size() > m_cacheDepthTicks)
            {
                slots.erase(slots.begin());
            }
        }

        bool m_runsPrediction;
        Events* m_events;
        LoggedWorld* m_world;
        uint32_t m_cacheDepthTicks;
        ScriptedClock m_clock;
        ServerTickClock m_serverClock;
        ResimGateProbe m_probe;
        std::set<unsigned int> m_registered;
        std::map<unsigned int, std::set<SimTick>> m_cacheSlots;
        std::optional<SimulationTimeStep> m_lastStep;
    };

    using Driver = SimulationStepDriver<RecordingManager, LoggedWorld, RecordingIntegration, RecordingHooks>;

    struct Rig
    {
        explicit Rig(bool client, uint32_t ringDepthTicks = 64u, StepDriverConfig config = StepDriverConfig{ kDt })
            : world(ringDepthTicks, events)
            , manager(client, events, world, ringDepthTicks)
            , integration(events)
            , hooks(events)
            , driver(manager, world, integration, hooks, config)
        {
        }

        TickOutcome tick() { return driver.runTick(++physicsStep); }

        void ticks(uint32_t count)
        {
            for (uint32_t i = 0; i < count; ++i)
            {
                tick();
            }
        }

        SimTick present() const { return manager.getClientClock().getPredictionTick(); }

        // Registers the character the frontier is read from, before the first tick.
        void registerTrackedCharacter()
        {
            manager.registerCharacter(kCharacter);
            driver.noteOccupancy(kCharacterSlot, true);
        }

        std::set<SimTick> ring() const { return world.heldTicks(present() + 2u); }

        ResimGateWindowSummary probeWindow() const
        {
            ResimGateWindowSummary summary;
            manager.probe().fillSummary(summary);
            return summary;
        }

        Events events;
        LoggedWorld world;
        RecordingManager manager;
        RecordingIntegration integration;
        RecordingHooks hooks;
        Driver driver;
        uint64_t physicsStep = 0u;
    };

    // Runs a resim of `anchor` and asserts everything that must hold of ANY granted resim.
    void requireGrantedResim(Rig& rig, SimTick anchor)
    {
        const SimTick present = rig.present();
        const uint32_t edgesBefore = rig.manager.applyEdges;
        const uint64_t stepsBefore = rig.world.steps();
        rig.manager.pendingAnchor = anchor;
        rig.events.clear();
        rig.manager.replays.clear();

        const TickOutcome outcome = rig.tick();

        REQUIRE_FALSE(outcome.resimRefused);
        REQUIRE(outcome.replayedTicks == present - anchor);
        REQUIRE(rig.manager.applyEdges == edgesBefore + 1u);
        REQUIRE(rig.world.steps() - stepsBefore == (present - anchor) + 1u);
        REQUIRE(rig.manager.replays.size() == present - anchor);
        REQUIRE(rig.world.failedRestores() == 0u);
    }
} // namespace

// ---------------------------------------------------------------------------
// Call order
// ---------------------------------------------------------------------------

TEST_CASE("StepDriver.CallOrder.ANormalTickRunsTheHooksAroundTheIntegrateAndTheStep", "[StepDriver]")
{
    Rig rig(/*client=*/true);
    rig.ticks(4u);
    rig.events.clear();

    const TickOutcome outcome = rig.tick();

    REQUIRE(rig.events == normalTick(5u));
    CHECK(outcome.tick == 5u);
    CHECK(outcome.kind == StepKind::Normal);
    CHECK(outcome.replayedTicks == 0u);
    CHECK_FALSE(outcome.hardResync);
    CHECK(outcome.physicsStep == 5u);
    REQUIRE(rig.hooks.upcomingTicks.back().authority == false);
    REQUIRE_FALSE(rig.hooks.upcomingTicks.back().authorityTick.has_value());
    REQUIRE(sameUpcoming(rig.hooks.simulateUpcomingTicks.back(), rig.hooks.upcomingTicks.back()));
    REQUIRE(rig.hooks.beforePhysicsTicks.back() == 5u);
}

TEST_CASE("StepDriver.CallOrder.AResimOfDepthOneFiveAndTwelve", "[StepDriver]")
{
    for (const uint32_t depth : { 1u, 5u, 12u })
    {
        INFO("depth " << depth);
        Rig rig(/*client=*/true, /*ringDepthTicks=*/16u);
        rig.ticks(20u);
        const SimTick present = rig.present();
        const SimTick anchor = present - depth;
        const uint64_t physicsStep = rig.physicsStep + 1u;

        requireGrantedResim(rig, anchor);

        // The whole interleaving: beforeTick -> restore -> applyOccupancy -> prepare -> push ->
        // N x (occupancy, sim, step, post, save) -> beforeSimulate -> scratch -> sim -> beforePhysics -> step ->
        // post -> save -> afterTick.
        REQUIRE(rig.events == resimThenNormalTick(anchor, present));

        // restoreTick received the anchor, prepareResimulation the physics step of this tick.
        REQUIRE(rig.world.inner().callsOf(MockWorldCallKind::RestoreTick).back().tick == anchor);
        REQUIRE(rig.manager.preparedPhysicsSteps.back() == static_cast<int32_t>(physicsStep));

        // Every replayed tick is saved under the tick the manager integrated, in order.
        std::vector<SimTick> expectedSaves;
        for (SimTick tick = anchor + 1u; tick <= present + 1u; ++tick)
        {
            expectedSaves.push_back(tick);
        }
        REQUIRE(savedTicksIn(rig.events) == expectedSaves);

        // Only the first replay step says first.
        REQUIRE(rig.manager.replays.front().first);
        for (std::size_t i = 1; i < rig.manager.replays.size(); ++i)
        {
            REQUIRE_FALSE(rig.manager.replays[i].first);
        }

        // beforePhysics lies after the normal tick's onGameSimulation and before its step.
        const auto sim = std::find(rig.events.rbegin(), rig.events.rend(), std::string("sim"));
        const auto beforePhysics = std::find(rig.events.rbegin(), rig.events.rend(), std::string("beforePhysics"));
        const auto lastStep = std::find(rig.events.rbegin(), rig.events.rend(), std::string("step"));
        REQUIRE(sim > beforePhysics);
        REQUIRE(beforePhysics > lastStep);
        REQUIRE(rig.hooks.beforePhysicsTicks.back() == present + 1u);
    }
}

TEST_CASE("StepDriver.CallOrder.BeforeSimulateRunsOncePerTickAfterAnyReplayWithTheBeforeTickValue", "[StepDriver]")
{
    SECTION("client: a replay on every tick (Always), plus a Stall, a Skip and a HardResync")
    {
        constexpr uint32_t kDepth = 5u;
        constexpr uint32_t kTicks = 60u;
        Rig rig(/*client=*/true, /*ringDepthTicks=*/kDepth + 2u,
            StepDriverConfig{ kDt, ResimPolicy::Always, kDepth });
        rig.registerTrackedCharacter();

        uint64_t replayedTicksSeen = 0u;
        for (uint32_t index = 0u; index < kTicks; ++index)
        {
            INFO("runTick " << index + 1u);
            if (index == 20u)
            {
                rig.manager.editClientClock().script(AdvanceResult::Stall);
            }
            else if (index == 30u)
            {
                rig.manager.editClientClock().script(AdvanceResult::Skip);
            }
            else if (index == 40u)
            {
                rig.manager.editClientClock().script(AdvanceResult::HardResync, 1u, /*hardResyncTarget=*/rig.present() + 30u);
            }

            rig.events.clear();
            const TickOutcome outcome = rig.tick();
            replayedTicksSeen += outcome.replayedTicks;

            // Exactly once per runTick, immediately before the scratch save, and after every replay
            // event of the tick: nothing of a replay follows it.
            REQUIRE(countOf(rig.events, "beforeSimulate") == 1u);
            const auto simulate = std::find(rig.events.begin(), rig.events.end(), std::string("beforeSimulate"));
            REQUIRE(simulate + 1 != rig.events.end());
            REQUIRE(*(simulate + 1) == "scratch");
            for (const char* replayEvent : { "restore", "prepare", "push", "simResim", "postResim" })
            {
                INFO("replay event " << replayEvent);
                REQUIRE(std::none_of(simulate, rig.events.end(), [&](const std::string& event)
                    { return event.rfind(replayEvent, 0) == 0; }));
            }
            if (outcome.replayedTicks > 0u)
            {
                REQUIRE(std::count(rig.events.begin(), simulate, std::string("simResim"))
                    == static_cast<std::ptrdiff_t>(outcome.replayedTicks));
            }
        }

        // Vacuity: replays really ran around the hook.
        CHECK(replayedTicksSeen > uint64_t{ kTicks });
        CHECK(rig.driver.getDiagnostics().skipBackfills() == 1u);
        CHECK(rig.driver.getDiagnostics().hardResyncInvalidations() == 1u);

        // The value beforeTick received, every tick.
        REQUIRE(rig.hooks.simulateUpcomingTicks.size() == kTicks);
        REQUIRE(rig.hooks.upcomingTicks.size() == kTicks);
        for (uint32_t index = 0u; index < kTicks; ++index)
        {
            INFO("runTick " << index + 1u);
            CHECK(sameUpcoming(rig.hooks.simulateUpcomingTicks[index], rig.hooks.upcomingTicks[index]));
            CHECK(rig.hooks.simulateUpcomingTicks[index].physicsStep == index + 1u);
            CHECK_FALSE(rig.hooks.simulateUpcomingTicks[index].authority);
        }
    }

    SECTION("authority: between beforeTick and onGameSimulation, with the same authorityTick")
    {
        constexpr uint32_t kTicks = 30u;
        Rig rig(/*client=*/false);
        std::vector<SimTick> serverTickAtSimulate;
        rig.hooks.onBeforeSimulate = [&](const UpcomingTick&)
        {
            serverTickAtSimulate.push_back(rig.manager.getServerClock().getTick());
        };
        rig.ticks(kTicks);

        CHECK(countOf(rig.events, "beforeSimulate") == kTicks);
        REQUIRE(rig.hooks.simulateUpcomingTicks.size() == kTicks);
        REQUIRE(rig.hooks.upcomingTicks.size() == kTicks);
        for (uint32_t index = 0u; index < kTicks; ++index)
        {
            INFO("runTick " << index + 1u);
            const UpcomingTick& simulate = rig.hooks.simulateUpcomingTicks[index];
            CHECK(sameUpcoming(simulate, rig.hooks.upcomingTicks[index]));
            CHECK(simulate.authority);
            REQUIRE(simulate.authorityTick.has_value());
            CHECK(*simulate.authorityTick == index + 1u);
            // The server clock has not advanced yet: the tick is still to be simulated.
            CHECK(serverTickAtSimulate[index] + 1u == *simulate.authorityTick);
        }
    }
}

// ---------------------------------------------------------------------------
// Saves and anchors
// ---------------------------------------------------------------------------

TEST_CASE("StepDriver.Anchors.ARequestOlderThanTheOldestHeldTickIsRefusedCountedAndSurvives", "[StepDriver]")
{
    Rig rig(/*client=*/true, /*ringDepthTicks=*/8u);
    rig.ticks(20u);
    REQUIRE(rig.world.oldestHeldTick() == std::optional<SimTick>(13u));

    rig.manager.pendingAnchor = 12u;
    rig.events.clear();
    const TickOutcome first = rig.tick();
    const TickOutcome second = rig.tick();

    CHECK(first.resimRefused);
    CHECK(second.resimRefused);
    CHECK(first.replayedTicks == 0u);
    CHECK(rig.driver.getDiagnostics().refusedResims() == 2u);
    CHECK(rig.driver.getDiagnostics().grantedResims() == 0u);
    CHECK(countOf(rig.events, "restore:12") == 0u);
    CHECK(rig.manager.preparedPhysicsSteps.empty());

    // The anchor survives: nothing consumed it, so it was asked again.
    REQUIRE(rig.manager.pendingAnchor == std::optional<unsigned int>(12u));
    REQUIRE(rig.manager.checks == 22u);

    // The probe reads the refusal the same way it read an engine refusal: requests - grants.
    const ResimGateWindowSummary window = rig.probeWindow();
    CHECK(window.requests == 2u);
    CHECK(window.grants == 0u);
    CHECK(window.refusedFrames == 2u);
    CHECK(window.prepares == 0u);

    // A held anchor is then granted, and the probe's grant/prepare/finish line up.
    requireGrantedResim(rig, 20u);
    const ResimGateWindowSummary after = rig.probeWindow();
    CHECK(after.requests == 3u);
    CHECK(after.grants == 1u);
    CHECK(after.refusedFrames == 2u);
    CHECK(after.clampedGrants == 0u);
    CHECK(after.prepares == 1u);
    CHECK(after.finishes == 1u);
}

TEST_CASE("StepDriver.Anchors.AlwaysModeRunsAThousandTicksWithNoFailedRestore", "[StepDriver]")
{
    constexpr uint32_t kDepth = 6u;
    Rig rig(/*client=*/true, /*ringDepthTicks=*/kDepth + 2u,
        StepDriverConfig{ kDt, ResimPolicy::Always, kDepth });
    rig.registerTrackedCharacter();

    rig.ticks(1000u);

    // Every tick whose present is past kDepth replays exactly kDepth ticks; none is refused.
    CHECK(rig.world.failedRestores() == 0u);
    CHECK(rig.driver.getDiagnostics().refusedResims() == 0u);
    CHECK(rig.driver.getDiagnostics().grantedResims() == 1000u - (kDepth + 1u));
    CHECK(rig.driver.getDiagnostics().replayedTicks() == uint64_t{ 1000u - (kDepth + 1u) } * kDepth);
    CHECK(rig.manager.applyEdges == 1000u - (kDepth + 1u));
    CHECK(rig.driver.getDiagnostics().occupancyTimelineMisses() == 0u);
    // Always never asks the manager.
    CHECK(rig.manager.checks == 0u);
    REQUIRE(rig.ring() == rig.manager.frontier(kCharacter));
}

// ---------------------------------------------------------------------------
// Clock sequences — each in prediction, then with the event inside a later replay window.
// After every sequence the physics ring and the correction-cache frontier hold the same ticks.
// ---------------------------------------------------------------------------

TEST_CASE("StepDriver.Clock.NormalStallNormal", "[StepDriver]")
{
    Rig rig(/*client=*/true);
    rig.registerTrackedCharacter();
    rig.ticks(5u);
    const SimTick before = rig.present();
    const uint64_t stepsBefore = rig.world.steps();

    rig.events.clear();
    rig.tick();
    rig.manager.editClientClock().script(AdvanceResult::Stall);
    const TickOutcome stall = rig.tick();
    rig.tick();

    // No save on the Stall, and one more physics step than the tick delta.
    CHECK(stall.kind == StepKind::Stall);
    CHECK(stall.tick == before + 1u);
    CHECK(savedTicksIn(rig.events) == std::vector<SimTick>{ before + 1u, before + 2u });
    CHECK(rig.world.steps() - stepsBefore == (rig.present() - before) + 1u);
    REQUIRE(rig.ring() == rig.manager.frontier(kCharacter));

    // Later: a resim whose window contains the Stall replays ONE step per tick (Q1).
    rig.ticks(2u);
    requireGrantedResim(rig, before);
    REQUIRE(rig.ring() == rig.manager.frontier(kCharacter));
}

TEST_CASE("StepDriver.Clock.SkipBackfillsThePreIntegrateWorld", "[StepDriver]")
{
    Rig rig(/*client=*/true);
    rig.registerTrackedCharacter();
    rig.ticks(5u);

    // The world as it is before the Skip step's integrate (no resim runs in that tick).
    const MockWorldState beforeIntegrate = rig.world.inner().live();
    rig.events.clear();
    rig.manager.editClientClock().script(AdvanceResult::Skip);
    const TickOutcome skip = rig.tick();

    CHECK(skip.kind == StepKind::Skip);
    REQUIRE(skip.tick == 7u);
    CHECK(countOf(rig.events, "commit:6") == 1u);
    CHECK(rig.driver.getDiagnostics().skipBackfills() == 1u);

    // The backfilled tick is held, and it is the PRE-integrate world: not the integrate's write,
    // not the post-step world.
    REQUIRE(rig.world.hasTick(6u));
    REQUIRE(rig.world.inner().heldState(6u) == std::optional<MockWorldState>(beforeIntegrate));
    MockWorldState afterIntegrate = beforeIntegrate;
    afterIntegrate.value += kIntegrateWrite;
    CHECK(rig.world.inner().heldState(6u) != std::optional<MockWorldState>(afterIntegrate));
    CHECK(rig.world.inner().heldState(7u) != rig.world.inner().heldState(6u));

    // Restoring it brings that world back.
    rig.world.restoreTick(6u);
    REQUIRE(rig.world.inner().live() == beforeIntegrate);
    rig.world.restoreTick(7u);
    REQUIRE(rig.ring() == rig.manager.frontier(kCharacter));

    // Later: a resim from before the Skip replays the skipped tick as an ordinary tick.
    rig.ticks(2u);
    requireGrantedResim(rig, 5u);
    CHECK(rig.manager.replays.size() == rig.present() - 1u - 5u);
    REQUIRE(rig.ring() == rig.manager.frontier(kCharacter));
}

TEST_CASE("StepDriver.Clock.AWorldChangeInBeforeSimulateIsInsideTheSkipBackfill", "[StepDriver]")
{
    constexpr uint32_t kJoinSlot = 3u;
    Rig rig(/*client=*/true);
    rig.registerTrackedCharacter();
    rig.ticks(5u);

    // The host joins a character in beforeSimulate of the Skip tick: a world change at that moment.
    std::optional<MockWorldState> afterJoin;
    rig.hooks.onBeforeSimulate = [&](const UpcomingTick&)
    {
        rig.driver.noteOccupancy(kJoinSlot, true);
        afterJoin = rig.world.inner().live();
    };
    REQUIRE_FALSE(rig.world.inner().live().occupancy.occupied.test(kJoinSlot));
    rig.manager.editClientClock().script(AdvanceResult::Skip);
    const TickOutcome skip = rig.tick();

    REQUIRE(skip.kind == StepKind::Skip);
    REQUIRE(skip.tick == 7u);
    REQUIRE(afterJoin.has_value());
    REQUIRE(afterJoin->occupancy.occupied.test(kJoinSlot));

    // The backfilled tick is the world as the hook left it: the scratch was taken after the hook.
    REQUIRE(rig.world.hasTick(6u));
    REQUIRE(rig.world.inner().heldState(6u) == afterJoin);
    REQUIRE(rig.driver.getDiagnostics().occupancyAt(6u).has_value());
    CHECK(rig.driver.getDiagnostics().occupancyAt(6u)->occupied.test(kJoinSlot));
}

TEST_CASE("StepDriver.Clock.HardResyncInvalidatesAndAnOlderAnchorIsRefused", "[StepDriver]")
{
    Rig rig(/*client=*/true);
    rig.registerTrackedCharacter();
    rig.ticks(10u);
    REQUIRE(rig.ring().size() == 10u);

    rig.events.clear();
    rig.manager.editClientClock().script(AdvanceResult::HardResync, 1u, /*hardResyncTarget=*/40u);
    const TickOutcome resync = rig.tick();

    // The callback invalidated, before the step; the new tick is the only one held.
    CHECK(resync.hardResync);
    CHECK(resync.tick == 40u);
    CHECK(resync.kind == StepKind::Normal);
    CHECK(countOf(rig.events, "invalidate") == 1u);
    const auto invalidate = std::find(rig.events.begin(), rig.events.end(), std::string("invalidate"));
    const auto step = std::find(rig.events.begin(), rig.events.end(), std::string("step"));
    CHECK(invalidate < step);
    CHECK(rig.driver.getDiagnostics().hardResyncInvalidations() == 1u);
    CHECK(rig.manager.wipes == 1u);
    REQUIRE(rig.ring() == std::set<SimTick>{ 40u });
    REQUIRE(rig.ring() == rig.manager.frontier(kCharacter));

    // Later: an anchor from before the resync is refused and counted; one after it is granted.
    rig.ticks(3u);
    CHECK_FALSE(rig.tick().hardResync);
    rig.manager.pendingAnchor = 8u;
    const TickOutcome refused = rig.tick();
    CHECK(refused.resimRefused);
    CHECK(rig.driver.getDiagnostics().refusedResims() == 1u);
    requireGrantedResim(rig, 41u);
    REQUIRE(rig.ring() == rig.manager.frontier(kCharacter));
}

TEST_CASE("StepDriver.Clock.ATierStallTimesThreeSavesNothing", "[StepDriver]")
{
    Rig rig(/*client=*/true);
    rig.registerTrackedCharacter();
    rig.ticks(6u);
    const SimTick before = rig.present();
    const std::set<SimTick> ringBefore = rig.ring();

    rig.events.clear();
    rig.manager.editClientClock().script(AdvanceResult::Stall, 3u);
    std::vector<TickOutcome> stalls;
    for (int i = 0; i < 3; ++i)
    {
        stalls.push_back(rig.tick());
    }

    for (const TickOutcome& stall : stalls)
    {
        CHECK(stall.kind == StepKind::Stall);
        CHECK(stall.tick == before);
    }
    CHECK(countOf(rig.events, "step") == 3u);
    CHECK(savedTicksIn(rig.events).empty());
    CHECK(rig.ring() == ringBefore);
    REQUIRE(rig.ring() == rig.manager.frontier(kCharacter));

    // Later: a replay across the three stalls is one step per tick.
    rig.ticks(2u);
    requireGrantedResim(rig, before - 1u);
    CHECK(rig.manager.replays.size() == 3u);
    REQUIRE(rig.ring() == rig.manager.frontier(kCharacter));
}

// ---------------------------------------------------------------------------
// Occupancy
// ---------------------------------------------------------------------------

TEST_CASE("StepDriver.Occupancy.AJoinAtJIsOccupiedFromJAndALeaveAtLClearsEveryHeldTick", "[StepDriver]")
{
    constexpr uint32_t kSlot = 3u;
    Rig rig(/*client=*/true);
    rig.ticks(5u);

    // The join is applied in beforeTick of the tick that simulates J.
    const SimTick join = rig.present() + 1u;
    rig.hooks.onBeforeTick = [&](const UpcomingTick&) { rig.driver.noteOccupancy(kSlot, true); };
    rig.tick();
    rig.hooks.onBeforeTick = nullptr;
    rig.ticks(4u);

    for (SimTick tick = 1u; tick <= rig.present(); ++tick)
    {
        INFO("tick " << tick);
        const std::optional<BodySlotOccupancy> held = rig.driver.getDiagnostics().occupancyAt(tick);
        REQUIRE(held.has_value());
        CHECK(held->occupied.test(kSlot) == (tick >= join));
    }

    // A leave at L clears the slot in every held tick, not just from L on.
    const SimTick leave = rig.present() + 1u;
    rig.hooks.onBeforeTick = [&](const UpcomingTick&) { rig.driver.noteOccupancy(kSlot, false); };
    rig.tick();
    rig.hooks.onBeforeTick = nullptr;
    REQUIRE(rig.present() == leave);
    REQUIRE(rig.driver.getDiagnostics().occupancyTimelineSize() == rig.ring().size());
    for (const SimTick tick : rig.ring())
    {
        INFO("tick " << tick);
        CHECK_FALSE(rig.driver.getDiagnostics().occupancyAt(tick)->occupied.test(kSlot));
    }
    CHECK_FALSE(rig.world.inner().live().occupancy.occupied.test(kSlot));

    // A later replay across the old join keeps the slot parked throughout.
    rig.ticks(2u);
    rig.world.inner().clearCalls();
    requireGrantedResim(rig, join - 2u);
    for (const MockWorldCall& call : rig.world.inner().callsOf(MockWorldCallKind::ApplyOccupancy))
    {
        CHECK_FALSE(call.occupancy.occupied.test(kSlot));
    }
}

TEST_CASE("StepDriver.Occupancy.AJoinInTheSameTickAsAResimIsParkedInTheReplayAndLiveForTheNormalStep", "[StepDriver]")
{
    constexpr uint32_t kSlot = 2u;
    Rig rig(/*client=*/true);
    rig.ticks(8u);
    const SimTick present = rig.present();

    // beforeTick applies the join, then the same tick resims across older ticks.
    rig.hooks.onBeforeTick = [&](const UpcomingTick&) { rig.driver.noteOccupancy(kSlot, true); };
    rig.world.inner().clearCalls();
    requireGrantedResim(rig, present - 3u);
    rig.hooks.onBeforeTick = nullptr;

    // Every replay step ran with the slot parked (the restore and each replayed tick)...
    const std::vector<MockWorldCall> applied = rig.world.inner().callsOf(MockWorldCallKind::ApplyOccupancy);
    REQUIRE(applied.size() == 1u + 1u + 3u + 1u);
    for (std::size_t i = 1; i + 1 < applied.size(); ++i)
    {
        INFO("apply " << i);
        CHECK_FALSE(applied[i].occupancy.occupied.test(kSlot));
    }
    // ...and the normal step after the replay ran with it live, and saved it live.
    CHECK(applied.back().occupancy.occupied.test(kSlot));
    CHECK(rig.driver.getDiagnostics().occupancyAt(present + 1u)->occupied.test(kSlot));
    CHECK_FALSE(rig.driver.getDiagnostics().occupancyAt(present)->occupied.test(kSlot));
}

TEST_CASE("StepDriver.Occupancy.AReplayedCharacterIsParkedAndUnintegratedExactlyUntilItsFirstCacheSlot",
    "[StepDriver]")
{
    // The character registers in beforeTick of one tick. Its first cache slot is the first tick
    // the frontier allocates for it after that: the same tick, unless that tick is a Stall (no
    // slot: the next one) or a Skip (the backfilled tick before it).
    struct Variant
    {
        const char* name;
        std::optional<AdvanceResult> registrationTickResult;
        int32_t firstSlotOffset;
    };
    const Variant variants[] = {
        { "a Normal registration tick", std::nullopt, 0 },
        { "a Stall registration tick", AdvanceResult::Stall, 1 },
        { "a Skip registration tick", AdvanceResult::Skip, 1 },
    };

    for (const Variant& variant : variants)
    {
        INFO(variant.name);
        Rig rig(/*client=*/true);
        rig.ticks(6u);
        const SimTick anchor = rig.present() - 2u;

        if (variant.registrationTickResult.has_value())
        {
            rig.manager.editClientClock().script(*variant.registrationTickResult);
        }
        rig.hooks.onBeforeTick = [&](const UpcomingTick&)
        {
            rig.manager.registerCharacter(kCharacter);
            rig.driver.noteOccupancy(kCharacterSlot, true);
        };
        rig.tick();
        rig.hooks.onBeforeTick = nullptr;
        const SimTick registrationTick = rig.present();
        const SimTick firstSlot = variant.registrationTickResult == AdvanceResult::Skip
            ? registrationTick - 1u
            : registrationTick + static_cast<SimTick>(variant.firstSlotOffset);
        if (variant.registrationTickResult != AdvanceResult::Skip)
        {
            CHECK(rig.manager.frontier(kCharacter).empty() == (variant.firstSlotOffset != 0));
        }
        rig.ticks(3u);
        REQUIRE(*rig.manager.frontier(kCharacter).begin() == firstSlot);

        // A resim from before the registration replays across the boundary.
        requireGrantedResim(rig, anchor);
        uint32_t parkedTicks = 0u;
        uint32_t liveTicks = 0u;
        for (const ReplayRecord& replay : rig.manager.replays)
        {
            INFO("replayed tick " << replay.tick << ", first cache slot " << firstSlot);
            const bool expectLive = replay.tick >= firstSlot;
            CHECK((replay.characterLive == expectLive && replay.characterIntegrated == expectLive));
            ++(expectLive ? liveTicks : parkedTicks);
        }
        // Vacuity guard: the window straddles the boundary.
        REQUIRE(parkedTicks >= 1u);
        REQUIRE(liveTicks >= 1u);
    }
}

// ---------------------------------------------------------------------------
// Authority
// ---------------------------------------------------------------------------

TEST_CASE("StepDriver.Authority.AnInputReleasedInBeforeTickIsConsumedByTheStepThatSimulatesIt", "[StepDriver]")
{
    SECTION("released for the tick beforeTick names: every input consumed by its own tick")
    {
        Rig rig(/*client=*/false);
        REQUIRE_FALSE(rig.driver.savesTicks());
        rig.hooks.onBeforeTick = [&](const UpcomingTick& upcoming)
        {
            REQUIRE(upcoming.authority);
            REQUIRE(upcoming.authorityTick.has_value());
            REQUIRE(*upcoming.authorityTick == rig.manager.getServerClock().getTick() + 1u);
            rig.manager.releasedInputs[*upcoming.authorityTick] = static_cast<int>(*upcoming.authorityTick) * 10;
        };
        rig.events.clear();
        rig.ticks(30u);

        for (SimTick tick = 1u; tick <= 30u; ++tick)
        {
            INFO("tick " << tick);
            REQUIRE(rig.manager.consumedInputs.at(tick) == std::optional<int>(static_cast<int>(tick) * 10));
        }
        CHECK(rig.manager.releasedInputs.empty());

        // The authority order, and no ring saves.
        const Events authorityTick{ "beforeTick", "beforeSimulate", "sim", "beforePhysics", "step", "post", "afterTick" };
        REQUIRE(rig.events.size() == 30u * authorityTick.size());
        for (std::size_t tick = 0; tick < 30u; ++tick)
        {
            INFO("authority tick " << tick + 1u);
            const auto first = rig.events.begin() + static_cast<std::ptrdiff_t>(tick * authorityTick.size());
            CHECK(Events(first, first + static_cast<std::ptrdiff_t>(authorityTick.size())) == authorityTick);
        }
        CHECK(savedTicksIn(rig.events).empty());
        CHECK(countOf(rig.events, "scratch") == 0u);
        CHECK(rig.hooks.outcomes.back().tick == 30u);
        CHECK(rig.hooks.beforePhysicsTicks.back() == 30u);
    }

    SECTION("POISON: released one tick early (no + 1), nothing is ever consumed")
    {
        Rig rig(/*client=*/false);
        rig.hooks.onBeforeTick = [&](const UpcomingTick& upcoming)
        {
            const SimTick withoutPlusOne = *upcoming.authorityTick - 1u;
            rig.manager.releasedInputs[withoutPlusOne] = static_cast<int>(withoutPlusOne) * 10;
        };
        rig.ticks(30u);

        for (SimTick tick = 1u; tick <= 30u; ++tick)
        {
            INFO("tick " << tick);
            REQUIRE_FALSE(rig.manager.consumedInputs.at(tick).has_value());
        }
        CHECK(rig.manager.releasedInputs.size() == 30u);
    }
}

// ---------------------------------------------------------------------------
// Resync callbacks: stable ids (the real ClientPredictionClock)
// ---------------------------------------------------------------------------

namespace
{
    struct RealClockRig
    {
        RealClockRig()
            : estimator(config, nullptr)
            , clock(config, estimator, nullptr)
        {
            config.minTicksBeforeDriftCheck = 0;
        }

        void hardResync()
        {
            const unsigned int target = clock.getPredictionTick() + 30u;
            estimator.recordAuthorityTick(target - estimator.getPredictionOffsetTicks());
            REQUIRE(clock.advancePrediction() == AdvanceResult::HardResync);
        }

        TimeConfig config;
        NetworkTimeEstimator estimator;
        ClientPredictionClock clock;
    };
}

TEST_CASE("StepDriver.ResyncCallbacks.UnregisteringTheFirstLeavesTheSecondFiring", "[StepDriver][ClientPredictionClock]")
{
    RealClockRig rig;
    std::vector<std::string> fired;
    const unsigned int first = rig.clock.registerResyncCallback([&](unsigned int) { fired.push_back("first"); });
    const unsigned int second = rig.clock.registerResyncCallback([&](unsigned int) { fired.push_back("second"); });
    REQUIRE(first != second);

    rig.clock.unregisterResyncCallback(first);
    rig.hardResync();
    REQUIRE(fired == std::vector<std::string>{ "second" });

    // The second's id still names the second: unregistering it silences it.
    fired.clear();
    rig.clock.unregisterResyncCallback(second);
    rig.hardResync();
    REQUIRE(fired.empty());
}

TEST_CASE("StepDriver.ResyncCallbacks.IdsAreStableAcrossAnyUnregisterOrder", "[StepDriver][ClientPredictionClock]")
{
    // Three registrations; remove the middle, then the last. Under the old swap-with-back the
    // last one's id pointed past the end of the vector after the first removal.
    RealClockRig rig;
    std::vector<std::string> fired;
    const unsigned int a = rig.clock.registerResyncCallback([&](unsigned int) { fired.push_back("a"); });
    const unsigned int b = rig.clock.registerResyncCallback([&](unsigned int) { fired.push_back("b"); });
    const unsigned int c = rig.clock.registerResyncCallback([&](unsigned int) { fired.push_back("c"); });

    rig.clock.unregisterResyncCallback(b);
    rig.hardResync();
    REQUIRE(fired == std::vector<std::string>{ "a", "c" });

    fired.clear();
    rig.clock.unregisterResyncCallback(c);
    rig.hardResync();
    REQUIRE(fired == std::vector<std::string>{ "a" });

    // An unknown or already-removed id is a no-op.
    fired.clear();
    rig.clock.unregisterResyncCallback(c);
    rig.clock.unregisterResyncCallback(ClientPredictionClock::InvalidCallbackId);
    rig.hardResync();
    REQUIRE(fired == std::vector<std::string>{ "a" });
    (void)a;
}

TEST_CASE("StepDriver.ResyncCallbacks.TheDriverUnregistersItsOwnCallbackOnDestruction", "[StepDriver]")
{
    Events events;
    LoggedWorld world(8u, events);
    RecordingManager manager(/*runsPrediction=*/true, events, world, 8u);
    RecordingIntegration integration(events);
    RecordingHooks hooks(events);
    REQUIRE(manager.getClientClock().callbackCount() == 1u);
    {
        Driver driver(manager, world, integration, hooks, StepDriverConfig{ kDt });
        REQUIRE(manager.getClientClock().callbackCount() == 2u);
    }
    REQUIRE(manager.getClientClock().callbackCount() == 1u);

    // The manager's own callback still fires, and nothing dangles.
    manager.editClientClock().script(AdvanceResult::HardResync, 1u, 50u);
    manager.onGameSimulation(SimulationUpdateInfo(false, false));
    REQUIRE(manager.wipes == 1u);
    REQUIRE(countOf(events, "invalidate") == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS
