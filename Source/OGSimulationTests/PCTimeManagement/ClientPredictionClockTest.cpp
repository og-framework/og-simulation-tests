// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/Network/ReplicatedTierConsumer.h"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"

// ---------------------------------------------------------------------------
// Test helpers
//
// Strategy: NetworkTimeEstimator has no RTT sample on construction →
//   getPredictionOffsetTicks() == 0  →  getTargetPredictionTick() == authorityTick.
// We control the "target" simply by calling recordAuthorityTick() before each
// advancePrediction().  TimeConfig is stored by const-ref, so mutating the
// local cfg object is immediately visible to both estimator and clock.
// ---------------------------------------------------------------------------

// Advance the clock N times with a huge soft-drift threshold (dead-band) so no
// correction fires.  Used to wind the prediction tick up to a known starting value.
static void advanceNoDrift(ClientPredictionClock& clock, NetworkTimeEstimator& est,
                           unsigned int targetOffset, unsigned int n)
{
    for (unsigned int i = 0; i < n; ++i)
    {
        est.recordAuthorityTick(clock.getPredictionTick() + targetOffset);
        clock.advancePrediction();
    }
}

// ---------------------------------------------------------------------------
// AC1 — Zero drift: every call increments by exactly 1
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.ZeroDrift", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck  = 0;    // no startup guard
    cfg.softDriftThresholdTicks   = 5;
    cfg.hardResyncThresholdTicks  = 15;
    cfg.gradualCorrectionRate     = 4;
    cfg.tickFrequency             = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // Keep authorityTick == predictionTick → drift == 0 → always dead-band
    for (unsigned int i = 0; i < 20; ++i)
    {
        const unsigned int before = clock.getPredictionTick();
        est.recordAuthorityTick(clock.getPredictionTick());   // drift = 0
        clock.advancePrediction();
        REQUIRE(clock.getPredictionTick() == before + 1);
    }
}

// ---------------------------------------------------------------------------
// AC2 — +5 drift (client behind), rate=4: every 4th call advances by 2
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.SkipCorrection", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck  = 0;
    cfg.softDriftThresholdTicks   = 3;
    cfg.hardResyncThresholdTicks  = 15;
    cfg.gradualCorrectionRate     = 4;
    cfg.tickFrequency             = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // Advance 4 calls with drift = +5 (client behind target by 5)
    // Expected: calls 1-3 advance by 1; call 4 advances by 2 (skip)
    unsigned int totalAdvance = 0;
    for (unsigned int call = 1; call <= 4; ++call)
    {
        const unsigned int before = clock.getPredictionTick();
        est.recordAuthorityTick(clock.getPredictionTick() + 5);
        clock.advancePrediction();
        const unsigned int delta = clock.getPredictionTick() - before;

        if (call < 4)
            REQUIRE(delta == 1u);
        else
            REQUIRE(delta == 2u);

        totalAdvance += delta;
    }
    REQUIRE(totalAdvance == 5u);
}

// ---------------------------------------------------------------------------
// AC3 — -5 drift (client ahead), rate=4: every 4th call does not advance
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.StallCorrection", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck  = 0;
    cfg.softDriftThresholdTicks   = 3;
    cfg.hardResyncThresholdTicks  = 15;
    cfg.gradualCorrectionRate     = 4;
    cfg.tickFrequency             = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // Wind prediction tick up to 20 in dead-band so target=predTick-5 doesn't underflow
    cfg.softDriftThresholdTicks = 100;
    advanceNoDrift(clock, est, 0, 20);
    cfg.softDriftThresholdTicks = 3;    // restore real threshold

    // Now predictionTick == 20. Keep authorityTick = predictionTick - 5 → drift = -5 (client ahead)
    unsigned int totalAdvance = 0;
    for (unsigned int call = 1; call <= 4; ++call)
    {
        const unsigned int before = clock.getPredictionTick();
        // authorityTick = predTick - 5 → target = predTick - 5 → drift = -5
        const int authTick = static_cast<int>(clock.getPredictionTick()) - 5;
        est.recordAuthorityTick(static_cast<unsigned int>(authTick));
        clock.advancePrediction();
        const unsigned int delta = clock.getPredictionTick() - before;

        if (call < 4)
            REQUIRE(delta == 1u);
        else
            REQUIRE(delta == 0u);

        totalAdvance += delta;
    }
    REQUIRE(totalAdvance == 3u);
}

// ---------------------------------------------------------------------------
// AC4 — +20 drift: hard resync fires, predictionTick jumps, callbacks called
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.HardResync", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck  = 0;
    cfg.softDriftThresholdTicks   = 3;
    cfg.hardResyncThresholdTicks  = 15;
    cfg.gradualCorrectionRate     = 4;
    cfg.tickFrequency             = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    bool callbackFired = false;
    unsigned int callbackTick = 0;

    clock.registerResyncCallback([&](unsigned int newTick)
    {
        callbackFired = true;
        callbackTick  = newTick;
    });

    // Set drift to +20 (well above hardResyncThreshold=15)
    const unsigned int targetTick = clock.getPredictionTick() + 20;
    est.recordAuthorityTick(targetTick);
    clock.advancePrediction();

    REQUIRE(callbackFired);
    REQUIRE(callbackTick == targetTick);
    REQUIRE(clock.getPredictionTick() == targetTick);
    REQUIRE(!clock.isResimulating());
}

// ---------------------------------------------------------------------------
// AC5 — Resimulation cursor: startResimulation → advance → isResimulating
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.Resimulation", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck = 0;
    cfg.softDriftThresholdTicks  = 100;   // dead-band for all setup advances
    cfg.tickFrequency            = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // Advance to tick 10 in dead-band
    advanceNoDrift(clock, est, 0, 10);
    REQUIRE(!clock.isResimulating());

    // Start resimulation from tick 5 (5 ticks behind prediction frontier)
    clock.startResimulation(5);
    REQUIRE(clock.isResimulating());
    REQUIRE(clock.getResimulationTick() == 5u);

    // Advance resimulation up to the frontier
    for (unsigned int i = 5; i < 10; ++i)
    {
        REQUIRE(clock.isResimulating());
        clock.advanceResimulation();
    }

    // finishResimulation explicitly
    REQUIRE(!clock.isResimulating());

    // Verify getResimulationStep reports isResimulating == true during resim
    clock.startResimulation(7);
    REQUIRE(clock.getResimulationStep().getIsResimulating());

    clock.finishResimulation();
    REQUIRE(!clock.isResimulating());
    REQUIRE(clock.getResimulationTick() == clock.getPredictionTick());
}

// ===========================================================================
// Fix B (Task 24) — one-sided (authority-only) warm-up pastGuard.
//
// The pastGuard in advancePrediction() and evaluateDrift() used to require BOTH
// m_predictionTick >= minTicksBeforeDriftCheck AND targetTick >= that threshold.
// In the cooked late-connect scenario the client walks its own ticks 0..59 by
// normal-advance while the server is already at 600+, so drift correction was
// blocked for ~1 s and every incoming correction was discarded. Fix B trusts
// authority as soon as the *authority* tick crosses the warm-up threshold.
// See ../og-brawler-hit-resolution/netcode_finding_pred_offset_floor.md §3.2.
//
// These cases use the DEFAULT TimeConfig (minTicksBeforeDriftCheck = 60,
// softDriftThresholdTicks = 3, hardResyncThresholdTicks = 21,
// gradualCorrectionRate = 4, predOffsetFloorTicks = 4, tickFrequency = 60.0)
// so the guard threshold and the T23 predOffset floor are both in effect.
// ===========================================================================

// ---------------------------------------------------------------------------
// WarmupGuard.PIESymmetric — regression: Fix B must NOT destabilize the
// symmetric-startup (PIE listen-server) case where both clocks tick from 0.
// Asserts SEMANTIC properties, NOT exact tick numbers for when Skip fires
// (under Fix B the first graduated Skip fires ~4 ticks earlier than under the
// old two-sided guard — expected + beneficial, not a regression).
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.WarmupGuard.PIESymmetric", "[PCTM][ClientPredictionClock][WarmupGuard]")
{
    TimeConfig cfg;   // all defaults

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // One LAN RTT sample so getPredictionOffsetTicks() settles at the T23 floor (4).
    est.updateRTT(0.0005);   // 0.5 ms loopback

    const unsigned int softLimit = static_cast<unsigned int>(cfg.softDriftThresholdTicks) + 1u;

    // Lockstep startup: advance authorityTick by 1 before each advancePrediction()
    // (server and client tick in the same physics loop).
    for (unsigned int authorityTick = 1; authorityTick <= 120; ++authorityTick)
    {
        est.recordAuthorityTick(authorityTick);
        const ClientPredictionClock::AdvanceResult result = clock.advancePrediction();

        // (a) HardResync MUST NEVER fire in the symmetric case.
        REQUIRE(result != ClientPredictionClock::AdvanceResult::HardResync);

        // (b) Once authority is past the warm-up threshold, the client stays
        //     predictively at or ahead of authority (Fix A's invariant).
        if (authorityTick >= cfg.minTicksBeforeDriftCheck)
            REQUIRE(clock.getPredictionTick() >= authorityTick);

        // (c) Drift stabilizes into the dead band (|drift| <= softDrift + 1)
        //     well before the loop ends.
        if (authorityTick >= 90)
        {
            const int drift = static_cast<int>(est.getTargetPredictionTick())
                            - static_cast<int>(clock.getPredictionTick());
            const unsigned int absDrift =
                drift >= 0 ? static_cast<unsigned int>(drift) : static_cast<unsigned int>(-drift);
            REQUIRE(absDrift <= softLimit);
        }
    }
}

// ---------------------------------------------------------------------------
// WarmupGuard.LateConnect — the load-bearing test. Client at predictionTick 0
// meets an authority already at tick 1000. Under Fix B the authority-only guard
// opens immediately (targetTick = 1004 >= 60) and HardResync jumps the client
// forward. Under the OLD two-sided guard predictionTick 0 < 60 would block the
// correction and the call would return Normal — so this test proves Fix B is
// applied in advancePrediction().
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.WarmupGuard.LateConnect", "[PCTM][ClientPredictionClock][WarmupGuard]")
{
    TimeConfig cfg;   // all defaults

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    est.updateRTT(0.0005);          // one sample → predOffset floored to 4 (T23)
    est.recordAuthorityTick(1000);  // authority already far ahead

    // Sanity: target = authorityTick + predOffsetFloorTicks = 1004.
    REQUIRE(est.getTargetPredictionTick() == 1004u);

    const ClientPredictionClock::AdvanceResult result = clock.advancePrediction();

    REQUIRE(result == ClientPredictionClock::AdvanceResult::HardResync);
    REQUIRE(clock.getPredictionTick() == 1004u);
}

// ---------------------------------------------------------------------------
// WarmupGuard.EvaluateDriftConsistency — the finding-gap-plug test, and the
// first-ever caller of evaluateDrift() in the codebase. With the SAME
// late-connect state, evaluateDrift() must report the SAME drift zone that
// advancePrediction() acts on (HardResync). If evaluateDrift() still carried
// the old two-sided guard it would return None here — so this pins the two
// pastGuard occurrences together against future one-sided edits.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.WarmupGuard.EvaluateDriftConsistency", "[PCTM][ClientPredictionClock][WarmupGuard]")
{
    TimeConfig cfg;   // all defaults

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    est.updateRTT(0.0005);
    est.recordAuthorityTick(1000);

    // BEFORE advancing: the pure-query drift zone must already be HardResync.
    REQUIRE(clock.evaluateDrift() == ClientPredictionClock::DriftAction::HardResync);

    // And advancePrediction() acts on that SAME zone — the two call-sites agree.
    REQUIRE(clock.advancePrediction() == ClientPredictionClock::AdvanceResult::HardResync);
}

// ===========================================================================
// T11 — tier-transition rollback (D5.3)
//
// HOW "ROLLBACK" IS MEASURED. The prediction frontier is monotonically
// non-decreasing outside a hard resync (see the rationale comment on the debt
// paydown in ClientPredictionClock.cpp), so a rollback is a RELATIVE regression:
// the clock ends up N ticks behind where the identical drive sequence would
// otherwise have left it. Every case below therefore runs a CONTROL clock with
// no rollback requested, drives both identically, and asserts on the gap. A
// test that asserted an absolute decrement of getPredictionTick() would be
// asserting the invariant-breaking behaviour we deliberately did not implement.
//
// The tier -> delay deltas are taken from the SHARED tierDelayDeltaTicks lookup
// rather than written as literals, so these cases follow a TimeConfig retune
// instead of silently testing stale numbers.
// ===========================================================================

namespace
{
    // A dead-band config: drift correction can never fire, so the only thing
    // that can move the two clocks apart is the rollback under test.
    TimeConfig makeRollbackTestConfig()
    {
        TimeConfig cfg;
        cfg.minTicksBeforeDriftCheck = 0;      // no startup guard
        cfg.softDriftThresholdTicks  = 1000;   // everything below is dead-band
        cfg.hardResyncThresholdTicks = 5000;   // never reached in these cases
        cfg.tickFrequency            = 60.0;
        return cfg;
    }

    // Drive a clock n times against a fixed authority tick of 0. With the config
    // above this is pure dead-band, so each call advances by exactly 1 unless a
    // rollback Stall consumes it.
    void driveTicks(ClientPredictionClock& clock, NetworkTimeEstimator& est, unsigned int n)
    {
        for (unsigned int i = 0; i < n; ++i)
        {
            est.recordAuthorityTick(0);
            clock.advancePrediction();
        }
    }
}

// ---------------------------------------------------------------------------
// AC-T11-1 — an UPWARD tier transition (0 -> 2) makes the client give back
// exactly `tierDelayDeltaTicks` predicted ticks.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.UpwardTierTransitionTriggersProactiveRollback",
          "[PCTM][ClientPredictionClock][TierRollback]")
{
    TimeConfig cfg = makeRollbackTestConfig();

    // Model the client's tier state exactly as production does: a
    // ReplicatedTierConsumer fed from OnRep, with the delta taken from the
    // shared lookup. No client-side tier derivation (Option A).
    ReplicatedTierConsumer consumer(cfg);
    consumer.onReplicatedTierReceived(0);
    const int32_t fromTier = consumer.currentTierIndex();
    consumer.onReplicatedTierReceived(2);
    const int32_t toTier = consumer.currentTierIndex();

    const int32_t delta = tierDelayDeltaTicks(fromTier, toTier, cfg);
    REQUIRE(delta > 0);        // premise: 0 -> 2 really is an upward transition

    NetworkTimeEstimator estA(cfg, nullptr);
    NetworkTimeEstimator estB(cfg, nullptr);
    ClientPredictionClock control(cfg, estA, nullptr);
    ClientPredictionClock rolled(cfg, estB, nullptr);

    rolled.requestTierTransitionRollback(delta);
    REQUIRE(rolled.getPendingTierRollbackTicks() == static_cast<unsigned int>(delta));

    // Enough ticks to pay the debt off in full and then run on normally.
    const unsigned int kDriveTicks = 40;
    driveTicks(control, estA, kDriveTicks);
    driveTicks(rolled,  estB, kDriveTicks);

    // Debt fully paid, and the frontier sits exactly `delta` ticks behind the
    // control clock that was driven identically.
    REQUIRE(rolled.getPendingTierRollbackTicks() == 0u);
    REQUIRE(control.getPredictionTick() - rolled.getPredictionTick()
            == static_cast<unsigned int>(delta));

    // The debt is paid as Stalls, and a Stall does not advance either cursor —
    // so the resim cursor must still sit at the frontier (not resimulating).
    REQUIRE_FALSE(rolled.isResimulating());
}

// ---------------------------------------------------------------------------
// AC-T11-2 — a DOWNWARD tier transition (2 -> 0) causes NO rollback. The client
// may now predict further ahead; that is a natural extension reached by the
// ordinary drift path, not something to undo here.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.DownwardTierTransitionDoesNotRollback",
          "[PCTM][ClientPredictionClock][TierRollback]")
{
    TimeConfig cfg = makeRollbackTestConfig();

    ReplicatedTierConsumer consumer(cfg);
    consumer.onReplicatedTierReceived(2);
    const int32_t fromTier = consumer.currentTierIndex();
    consumer.onReplicatedTierReceived(0);
    const int32_t toTier = consumer.currentTierIndex();

    const int32_t delta = tierDelayDeltaTicks(fromTier, toTier, cfg);
    REQUIRE(delta < 0);        // premise: 2 -> 0 really is a downward transition

    NetworkTimeEstimator estA(cfg, nullptr);
    NetworkTimeEstimator estB(cfg, nullptr);
    ClientPredictionClock control(cfg, estA, nullptr);
    ClientPredictionClock rolled(cfg, estB, nullptr);

    rolled.requestTierTransitionRollback(delta);

    // The negative delta was dropped outright — no debt was ever registered.
    REQUIRE(rolled.getPendingTierRollbackTicks() == 0u);

    const unsigned int kDriveTicks = 40;
    driveTicks(control, estA, kDriveTicks);
    driveTicks(rolled,  estB, kDriveTicks);

    // No regression whatsoever: the two clocks are indistinguishable.
    REQUIRE(rolled.getPredictionTick() == control.getPredictionTick());
}

// ---------------------------------------------------------------------------
// AC-T11-3 — consecutive UPWARD transitions (0 -> 1 -> 3) accumulate: the total
// rollback is the SUM of the deltas, not just the most recent one. Both
// transitions are registered before any tick is driven, so the second lands
// while the first is still unpaid — the case that would expose an
// overwrite-instead-of-accumulate bug.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.MultipleUpwardTransitionsAccumulate",
          "[PCTM][ClientPredictionClock][TierRollback]")
{
    TimeConfig cfg = makeRollbackTestConfig();

    ReplicatedTierConsumer consumer(cfg);
    consumer.onReplicatedTierReceived(0);
    const int32_t tier0 = consumer.currentTierIndex();
    consumer.onReplicatedTierReceived(1);
    const int32_t tier1 = consumer.currentTierIndex();
    consumer.onReplicatedTierReceived(3);
    const int32_t tier3 = consumer.currentTierIndex();

    const int32_t deltaA = tierDelayDeltaTicks(tier0, tier1, cfg);
    const int32_t deltaB = tierDelayDeltaTicks(tier1, tier3, cfg);
    REQUIRE(deltaA > 0);
    REQUIRE(deltaB > 0);

    NetworkTimeEstimator estA(cfg, nullptr);
    NetworkTimeEstimator estB(cfg, nullptr);
    ClientPredictionClock control(cfg, estA, nullptr);
    ClientPredictionClock rolled(cfg, estB, nullptr);

    rolled.requestTierTransitionRollback(deltaA);
    rolled.requestTierTransitionRollback(deltaB);

    // Accumulated, not overwritten.
    REQUIRE(rolled.getPendingTierRollbackTicks()
            == static_cast<unsigned int>(deltaA + deltaB));

    const unsigned int kDriveTicks = 40;
    driveTicks(control, estA, kDriveTicks);
    driveTicks(rolled,  estB, kDriveTicks);

    REQUIRE(rolled.getPendingTierRollbackTicks() == 0u);
    REQUIRE(control.getPredictionTick() - rolled.getPredictionTick()
            == static_cast<unsigned int>(deltaA + deltaB));

    // And the cumulative rollback genuinely exceeds either single transition —
    // guards against a "last delta wins" implementation passing by accident.
    REQUIRE(control.getPredictionTick() - rolled.getPredictionTick()
            > static_cast<unsigned int>(deltaB));
}

#endif // WITH_LOW_LEVEL_TESTS
