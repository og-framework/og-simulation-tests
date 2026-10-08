// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <set>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/ResimGateProbe.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SimulationStepDriver.h"
#include "StepDriverTestSupport.h"

// ---------------------------------------------------------------------------
// The step driver over the REAL SimulationManager: its real ClientPredictionClock (driven into
// Stall, Skip and HardResync through its own inputs), its real resim cursor and apply edge, and
// its real ResimGateProbe. The peers are mocks; the reconciliation mock keeps a frontier by the
// same rule as SimulationReconciliation::allocateFrontierSlotsAll, so the physics ring can be
// compared with what the real clock's step kinds made the cache allocate.
//
// This is the instantiation that proves the driver's duck-typed manager surface is the real
// manager's: lastIntegratedStep, currentIntegratedTick, editClientClock().registerResyncCallback,
// getServerClock, editResimGateProbe, prepareResimulation(physicsStep, simTick).
// ---------------------------------------------------------------------------

namespace
{
    using namespace stepDriverTestSupport;

    struct RmIntegration
    {
        void firstResimStepAll(int32_t) {}
        void captureBodyStatesAll() {}
        void pushCorrectedBodyStatesAll() { ++pushes; }
        void integrateAll(const SimulationTimeStep&, int) {}
        uint32_t pushes = 0u;
    };

    struct RmNetSync
    {
        void setAuthorityGuardContext(unsigned int, int32_t) {}
    };

    struct RmInputResolution
    {
        void wipeAllForResync(unsigned int) {}
        int collectInputAll(const SimulationTimeStep&) { return 0; }
        int collectResimInputAll(unsigned int) { return 0; }
    };

    struct RmReconciliation
    {
        void wipeAllForResync(unsigned int)
        {
            frontier.clear();
            ++wipes;
        }

        unsigned int checkDivergenceAll(std::uint32_t, unsigned int* outDeepSkips = nullptr)
        {
            if (outDeepSkips != nullptr)
            {
                *outDeepSkips = 0u;
            }
            return anchor;
        }

        unsigned int consumeResimAnchorsAll()
        {
            ++consumes;
            anchor = 0u;
            return 0u;
        }

        void setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy) {}

        void allocateFrontierSlotsAll(const SimulationTimeStep& step)
        {
            if (step.getStepKind() == StepKind::Skip)
            {
                frontier.insert(step.getTick() - 1u);
            }
            if (stepAllocatesFrontierSlot(step.getStepKind()))
            {
                frontier.insert(step.getTick());
            }
        }

        void prepareResimAll(std::uint32_t) {}
        void applyResimAll() { ++applies; }
        void postPredictionAll(const SimulationTimeStep&) {}
        ResimSweepDiagnostics postResimulationAll(const SimulationTimeStep&) { return ResimSweepDiagnostics{}; }

        struct Diagnostics
        {
            void logSlotProvenanceAll() const {}
        };
        Diagnostics getDiagnostics() const { return Diagnostics{}; }

        std::set<SimTick> frontier;
        unsigned int anchor = 0u;
        uint32_t wipes = 0u;
        uint32_t consumes = 0u;
        uint32_t applies = 0u;
    };

    struct RmSystemsExec
    {
        template <typename StorageT, typename StaticDataT>
        void firePreIntegrate(const SimulationTimeStep&, StorageT&, const StaticDataT&, bool) {}
        template <typename StorageT, typename StaticDataT>
        void firePostIntegrate(const SimulationTimeStep&, StorageT&, const StaticDataT&, bool) {}
        template <typename StorageT, typename StaticDataT>
        void notifyCharacterRegistered(unsigned int, StorageT&, const StaticDataT&, bool) {}
        template <typename StorageT, typename StaticDataT>
        void notifyCharacterUnregistered(unsigned int, StorageT&, const StaticDataT&, bool) {}
    };

    struct RmStorage {};
    struct RmStaticData {};

    using RealManager = SimulationManager<RmIntegration, RmNetSync, RmInputResolution, RmReconciliation,
        RmSystemsExec, RmStorage, RmStaticData>;

    struct RealManagerRig
    {
        RealManagerRig(bool client, uint32_t ringDepthTicks)
            : world(ringDepthTicks, events)
            , manager(client, /*tickFrequency (dt, seconds)=*/1.0 / 60.0,
                  RealManager::Params{ integration, netSync, inputResolution, reconciliation, systemsExec,
                                       storage, staticData, nullptr })
            , hooks(events)
            , driver(manager, world, integration, hooks, StepDriverConfig{ 1.f / 60.f })
        {
        }

        TickOutcome tick() { return driver.runTick(++physicsStep); }

        SimTick present() const { return manager.getClientClock().getPredictionTick(); }

        ResimGateWindowSummary probeWindow() const
        {
            ResimGateWindowSummary summary;
            manager.getDiagnostics().resimGateProbe().fillSummary(summary);
            return summary;
        }

        // Feeds the real estimator so the real clock's target is `present + drift` before the next
        // advance (past the warm-up guard once the target is at least minTicksBeforeDriftCheck).
        void holdDrift(int32_t drift)
        {
            NetworkTimeEstimator& estimator = manager.editNetworkEstimator();
            const int64_t target = static_cast<int64_t>(present()) + drift;
            estimator.recordAuthorityTick(static_cast<unsigned int>(target - estimator.getPredictionOffsetTicks()));
        }

        Events events;
        RmIntegration integration;
        RmNetSync netSync;
        RmInputResolution inputResolution;
        RmReconciliation reconciliation;
        RmSystemsExec systemsExec;
        RmStorage storage;
        RmStaticData staticData;
        LoggedWorld world;
        RealManager manager;
        RecordingHooks hooks;
        SimulationStepDriver<RealManager, LoggedWorld, RmIntegration, RecordingHooks> driver;
        uint64_t physicsStep = 0u;
    };
}

TEST_CASE("StepDriver.RealManager.NormalStallResimSkipAndHardResyncKeepTheRingOnTheFrontier", "[StepDriver]")
{
    RealManagerRig rig(/*client=*/true, /*ringDepthTicks=*/32u);
    REQUIRE(rig.driver.savesTicks());

    // HardResync through the real clock: the target jumps far past the frontier.
    rig.holdDrift(500);
    const TickOutcome resync = rig.tick();
    REQUIRE(resync.hardResync);
    REQUIRE(rig.reconciliation.wipes == 1u);
    REQUIRE(rig.driver.getDiagnostics().hardResyncInvalidations() == 1u);
    REQUIRE(rig.world.heldTicks(rig.present() + 1u) == std::set<SimTick>{ rig.present() });
    REQUIRE(rig.manager.lastIntegratedStep()->getTick() == rig.present());

    // Normal ticks with the drift held at zero.
    for (int i = 0; i < 10; ++i)
    {
        rig.holdDrift(1);
        REQUIRE(rig.tick().kind == StepKind::Normal);
    }
    REQUIRE(rig.world.heldTicks(rig.present() + 1u) == rig.reconciliation.frontier);

    // A tier stall through the real clock's own door: no save, the frontier does not move.
    const SimTick beforeStall = rig.present();
    rig.manager.editClientClock().requestInputDelayIncreaseStall(1);
    rig.holdDrift(1);
    const TickOutcome stall = rig.tick();
    REQUIRE(stall.kind == StepKind::Stall);
    REQUIRE(stall.tick == beforeStall);
    REQUIRE(rig.world.heldTicks(rig.present() + 1u) == rig.reconciliation.frontier);

    // A resim through the real cursor: the replay count is predictionTick - anchor, the real apply
    // edge fires once (applyResimAll, consumeResimAnchorsAll, the probe's finish), the clock ends
    // caught up, and every replayed tick is saved.
    const SimTick present = rig.present();
    const SimTick anchor = present - 6u;
    rig.reconciliation.anchor = anchor;
    rig.holdDrift(1);
    rig.events.clear();
    const TickOutcome resim = rig.tick();
    REQUIRE(resim.replayedTicks == present - anchor);
    REQUIRE(rig.reconciliation.applies == 1u);
    REQUIRE(rig.reconciliation.consumes == 1u);
    REQUIRE(rig.integration.pushes == 1u);
    REQUIRE_FALSE(rig.manager.getClientClock().isResimulating());
    REQUIRE(countOf(rig.events, "step") == (present - anchor) + 1u);
    {
        const ResimGateWindowSummary window = rig.probeWindow();
        CHECK(window.requests == 1u);
        CHECK(window.grants == 1u);
        CHECK(window.prepares == 1u);
        CHECK(window.finishes == 1u);
        CHECK(window.replayTicks == present - anchor);
        CHECK(window.clampedGrants == 0u);
    }
    REQUIRE(rig.world.heldTicks(rig.present() + 1u) == rig.reconciliation.frontier);

    // A refused anchor (older than the ring) survives: the real gate asks again next tick.
    rig.reconciliation.anchor = anchor - 30u;
    rig.holdDrift(1);
    REQUIRE(rig.tick().resimRefused);
    rig.holdDrift(1);
    REQUIRE(rig.tick().resimRefused);
    REQUIRE(rig.reconciliation.consumes == 1u);
    {
        const ResimGateWindowSummary window = rig.probeWindow();
        CHECK(window.requests == 3u);
        CHECK(window.grants == 1u);
        CHECK(window.refusedFrames == 2u);
        CHECK(window.prepares == 1u);
    }
    rig.reconciliation.anchor = 0u;

    // A graduated Skip through the real clock: hold a drift inside the soft band's far side until
    // the clock skips; the backfilled tick is committed and held.
    TickOutcome skip;
    for (int i = 0; i < 16 && skip.kind != StepKind::Skip; ++i)
    {
        rig.holdDrift(8);
        skip = rig.tick();
    }
    REQUIRE(skip.kind == StepKind::Skip);
    REQUIRE(rig.world.hasTick(skip.tick - 1u));
    REQUIRE(rig.driver.getDiagnostics().skipBackfills() == 1u);
    REQUIRE(rig.world.heldTicks(rig.present() + 1u) == rig.reconciliation.frontier);
}

TEST_CASE("StepDriver.RealManager.TheAuthorityStepsServerTickPlusOneAndSavesNothing", "[StepDriver]")
{
    RealManagerRig rig(/*client=*/false, /*ringDepthTicks=*/8u);
    REQUIRE_FALSE(rig.driver.savesTicks());
    std::vector<SimTick> named;
    rig.hooks.onBeforeTick = [&](const UpcomingTick& upcoming) { named.push_back(*upcoming.authorityTick); };
    rig.events.clear();

    for (SimTick expected = 1u; expected <= 20u; ++expected)
    {
        const TickOutcome outcome = rig.tick();
        REQUIRE(outcome.tick == expected);
        REQUIRE(rig.manager.getServerClock().getTick() == expected);
        REQUIRE(named.back() == expected);
    }
    CHECK(savedTicksIn(rig.events).empty());
    CHECK(countOf(rig.events, "scratch") == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS
