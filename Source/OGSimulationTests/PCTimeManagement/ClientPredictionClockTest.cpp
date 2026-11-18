// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/Network/ReplicatedTierConsumer.h"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Test helpers
//
// Strategy: we control the "target" by calling recordAuthorityTick() before each
// advancePrediction().  TimeConfig is stored by const-ref, so mutating the
// local cfg object is immediately visible to both estimator and clock.
//
// ⚠ CORRECTED BY T26a — READ THIS BEFORE ADDING A CASE. This comment used to
// say "no RTT sample on construction → getPredictionOffsetTicks() == 0 →
// getTargetPredictionTick() == authorityTick". That premise is DEAD. The
// architect's T26a ruling makes the no-sample path return
// `predOffsetFloorTicks`, because the floor is a STRUCTURAL INVARIANT ("the
// client predicts forward") rather than an estimate, and returning 0 there
// re-created the dead-band lock the floor exists to prevent. So for an estimator
// that has been fed no RTT sample:
//
//     getTargetPredictionTick() == authorityTick + predOffsetFloorTicks
//
// A case that needs a SPECIFIC drift must therefore subtract the offset when it
// sets the authority tick — see StallCorrection and HardResync below, which are
// the two that do. Cases whose intended drift merely has to land in a
// particular zone (ZeroDrift, SkipCorrection) are unaffected and are left
// untouched.
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
        // INTENDED DRIFT IS -5 (client ahead). target = authorityTick + offset,
        // and with no RTT sample that offset is predOffsetFloorTicks (T26a — it
        // used to be 0, which is why this line used to omit the subtraction).
        // Subtracting it keeps the drift this case is named for.
        const int authTick = static_cast<int>(clock.getPredictionTick()) - 5
                           - static_cast<int>(est.getPredictionOffsetTicks());
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

    // Set drift to +20 (well above hardResyncThreshold=15).
    // target = authorityTick + offset, and with no RTT sample that offset is
    // predOffsetFloorTicks (T26a — it used to be 0, which is why this line used
    // to record `targetTick` directly). The authority tick is set that much
    // lower so the TARGET is the +20 this case asserts against.
    const unsigned int targetTick = clock.getPredictionTick() + 20;
    est.recordAuthorityTick(targetTick - est.getPredictionOffsetTicks());
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
// T11 — tier-transition stall (D5.3)
//
// HOW THE STALL IS MEASURED. The prediction frontier is monotonically
// non-decreasing outside a hard resync (see the rationale comment on the debt
// paydown in ClientPredictionClock.cpp), so the effect is a RELATIVE regression:
// the clock ends up N ticks behind where the identical drive sequence would
// otherwise have left it. Every case below therefore runs a CONTROL clock with
// no stall requested, drives both identically, and asserts on the gap. A
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
    // that can move the two clocks apart is the stall under test.
    TimeConfig makeStallTestConfig()
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
    // debt-paydown Stall consumes it.
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
TEST_CASE("PCTM.ClientPredictionClock.UpwardTierTransitionTriggersProactiveStall",
          "[PCTM][ClientPredictionClock][TierStall]")
{
    TimeConfig cfg = makeStallTestConfig();

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
    ClientPredictionClock stalled(cfg, estB, nullptr);

    stalled.requestInputDelayIncreaseStall(delta);
    REQUIRE(stalled.getRequiredInputDelayIncreaseStallTicks() == static_cast<unsigned int>(delta));

    // Enough ticks to pay the debt off in full and then run on normally.
    const unsigned int kDriveTicks = 40;
    driveTicks(control, estA, kDriveTicks);
    driveTicks(stalled, estB, kDriveTicks);

    // Debt fully paid, and the frontier sits exactly `delta` ticks behind the
    // control clock that was driven identically.
    REQUIRE(stalled.getRequiredInputDelayIncreaseStallTicks() == 0u);
    REQUIRE(control.getPredictionTick() - stalled.getPredictionTick()
            == static_cast<unsigned int>(delta));

    // The debt is paid as Stalls, and a Stall does not advance either cursor —
    // so the resim cursor must still sit at the frontier (not resimulating).
    REQUIRE_FALSE(stalled.isResimulating());
}

// ---------------------------------------------------------------------------
// AC-T11-2 — a DOWNWARD tier transition (2 -> 0) causes NO stall. The client
// may now predict further ahead; that is a natural extension reached by the
// ordinary drift path, not something to undo here.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.DownwardTierTransitionDoesNotStall",
          "[PCTM][ClientPredictionClock][TierStall]")
{
    TimeConfig cfg = makeStallTestConfig();

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
    ClientPredictionClock stalled(cfg, estB, nullptr);

    stalled.requestInputDelayIncreaseStall(delta);

    // The negative delta was dropped outright — no debt was ever registered.
    REQUIRE(stalled.getRequiredInputDelayIncreaseStallTicks() == 0u);

    const unsigned int kDriveTicks = 40;
    driveTicks(control, estA, kDriveTicks);
    driveTicks(stalled, estB, kDriveTicks);

    // No regression whatsoever: the two clocks are indistinguishable.
    REQUIRE(stalled.getPredictionTick() == control.getPredictionTick());
}

// ---------------------------------------------------------------------------
// AC-T11-3 — consecutive UPWARD transitions (0 -> 1 -> 3) accumulate: the total
// stall is the SUM of the deltas, not just the most recent one. Both
// transitions are registered before any tick is driven, so the second lands
// while the first is still unpaid — the case that would expose an
// overwrite-instead-of-accumulate bug.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.MultipleUpwardTransitionsAccumulate",
          "[PCTM][ClientPredictionClock][TierStall]")
{
    TimeConfig cfg = makeStallTestConfig();

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
    ClientPredictionClock stalled(cfg, estB, nullptr);

    stalled.requestInputDelayIncreaseStall(deltaA);
    stalled.requestInputDelayIncreaseStall(deltaB);

    // Accumulated, not overwritten.
    REQUIRE(stalled.getRequiredInputDelayIncreaseStallTicks()
            == static_cast<unsigned int>(deltaA + deltaB));

    const unsigned int kDriveTicks = 40;
    driveTicks(control, estA, kDriveTicks);
    driveTicks(stalled, estB, kDriveTicks);

    REQUIRE(stalled.getRequiredInputDelayIncreaseStallTicks() == 0u);
    REQUIRE(control.getPredictionTick() - stalled.getPredictionTick()
            == static_cast<unsigned int>(deltaA + deltaB));

    // And the cumulative stall genuinely exceeds either single transition —
    // guards against a "last delta wins" implementation passing by accident.
    REQUIRE(control.getPredictionTick() - stalled.getPredictionTick()
            > static_cast<unsigned int>(deltaB));
}

// ===========================================================================
// THE CLOCK'S EVENT SEAM - skip / stall / hard-resync counts and last ticks
//
// The seven words live on `getDiagnostics()`; they are written PER EVENT inside
// `advancePrediction`'s branches, never once per call. Every case below therefore
// asserts the count on EVERY call, not only on the ones that fire, so a per-tick
// implementation fails on the very first non-event call.
//
// ⛔ No reader in the clock, the manager or any peer may consult these words. The
// case that makes that a red test rather than a comment is the last one in this file.
// ===========================================================================

namespace
{
    // A graduated zone wide enough for a +/-20 drift to stay inside it.
    TimeConfig makeEventSeamConfig()
    {
        TimeConfig cfg;
        cfg.minTicksBeforeDriftCheck = 0;
        cfg.softDriftThresholdTicks  = 3;
        cfg.hardResyncThresholdTicks = 50;
        cfg.gradualCorrectionRate    = 4;
        cfg.tickFrequency            = 60.0;
        return cfg;
    }

    // Drive one call at a chosen drift. `target == authority + predOffsetFloorTicks` with no
    // RTT sample, so the authority tick is set that much lower and the named drift is the real one.
    ClientPredictionClock::AdvanceResult driveOneAtDrift(ClientPredictionClock& clock,
                                                         NetworkTimeEstimator& est, int drift)
    {
        const int authority = static_cast<int>(clock.getPredictionTick()) + drift
                            - static_cast<int>(est.getPredictionOffsetTicks());
        est.recordAuthorityTick(authority > 0 ? static_cast<unsigned int>(authority) : 0u);
        return clock.advancePrediction();
    }
}

// ---------------------------------------------------------------------------
// Graduated SKIPS: one count per event, and the tick is the one jumped TO.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.Events.GraduatedSkipsCountPerEventAtTheTickJumpedTo",
          "[PCTM][ClientPredictionClock][ClockEvents]")
{
    TimeConfig cfg = makeEventSeamConfig();
    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    REQUIRE(clock.getDiagnostics().skipCount()    == 0u);
    REQUIRE(clock.getDiagnostics().lastSkipTick() == 0u);

    unsigned int expectedSkips = 0u;
    for (unsigned int call = 1u; call <= 12u; ++call)
    {
        const ClientPredictionClock::AdvanceResult result = driveOneAtDrift(clock, est, +20);
        const bool isSkipCall = (call % 4u) == 0u;
        REQUIRE((result == ClientPredictionClock::AdvanceResult::Skip) == isSkipCall);

        if (isSkipCall)
        {
            ++expectedSkips;
            REQUIRE(clock.getDiagnostics().lastSkipTick() == clock.getPredictionTick());
        }
        // Asserted on EVERY call: a per-tick counter would already be ahead here.
        REQUIRE(clock.getDiagnostics().skipCount() == expectedSkips);
    }

    REQUIRE(expectedSkips == 3u);
    REQUIRE(clock.getDiagnostics().stallCount()      == 0u);
    REQUIRE(clock.getDiagnostics().hardResyncCount() == 0u);
}

// ---------------------------------------------------------------------------
// Graduated STALLS: one count per event, and the tick is the frontier held at.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.Events.GraduatedStallsCountPerEventAtTheHeldFrontier",
          "[PCTM][ClientPredictionClock][ClockEvents]")
{
    TimeConfig cfg = makeEventSeamConfig();
    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // Wind the frontier up in a pure dead band so a -20 drift is expressible at all.
    cfg.softDriftThresholdTicks = 1000;
    for (unsigned int i = 0u; i < 30u; ++i)
        driveOneAtDrift(clock, est, 0);
    cfg.softDriftThresholdTicks = 3;

    REQUIRE(clock.getDiagnostics().stallCount()    == 0u);
    REQUIRE(clock.getDiagnostics().lastStallTick() == 0u);

    unsigned int expectedStalls = 0u;
    for (unsigned int call = 1u; call <= 12u; ++call)
    {
        const unsigned int before = clock.getPredictionTick();
        const ClientPredictionClock::AdvanceResult result = driveOneAtDrift(clock, est, -20);
        const bool isStallCall = (call % 4u) == 0u;
        REQUIRE((result == ClientPredictionClock::AdvanceResult::Stall) == isStallCall);

        if (isStallCall)
        {
            ++expectedStalls;
            REQUIRE(clock.getPredictionTick() == before);
            REQUIRE(clock.getDiagnostics().lastStallTick() == clock.getPredictionTick());
        }
        REQUIRE(clock.getDiagnostics().stallCount() == expectedStalls);
    }

    REQUIRE(expectedStalls == 3u);
    REQUIRE(clock.getDiagnostics().skipCount()       == 0u);
    REQUIRE(clock.getDiagnostics().hardResyncCount() == 0u);
}

// ---------------------------------------------------------------------------
// The TIER-DEBT stall is the second stall branch, and it counts too.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.Events.TheTierDebtStallCountsAtTheHeldFrontier",
          "[PCTM][ClientPredictionClock][ClockEvents]")
{
    TimeConfig cfg = makeStallTestConfig();
    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    driveTicks(clock, est, 10u);
    REQUIRE(clock.getDiagnostics().stallCount() == 0u);

    clock.requestInputDelayIncreaseStall(3);
    const unsigned int heldFrontier = clock.getPredictionTick();

    for (unsigned int paid = 1u; paid <= 3u; ++paid)
    {
        est.recordAuthorityTick(0);
        REQUIRE(clock.advancePrediction() == ClientPredictionClock::AdvanceResult::Stall);
        REQUIRE(clock.getDiagnostics().stallCount()    == paid);
        REQUIRE(clock.getDiagnostics().lastStallTick() == heldFrontier);
    }

    // Debt paid: the next call is an ordinary advance and moves no counter.
    est.recordAuthorityTick(0);
    REQUIRE(clock.advancePrediction() == ClientPredictionClock::AdvanceResult::Normal);
    REQUIRE(clock.getDiagnostics().stallCount() == 3u);
    REQUIRE(clock.getDiagnostics().skipCount()  == 0u);
}

// ---------------------------------------------------------------------------
// The HARD RESYNC records the exact from/to pair - the user's storm, by its numbers.
// The display had only "at least as old as the last poll" before this seam existed.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.Events.AHardResyncRecordsTheExactFromAndToTicks",
          "[PCTM][ClientPredictionClock][ClockEvents]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck = 0;
    cfg.softDriftThresholdTicks  = 1000;
    cfg.hardResyncThresholdTicks = 5000;
    cfg.gradualCorrectionRate    = 4;
    cfg.tickFrequency            = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // The pair the user's session logged, 22 ticks backward.
    const unsigned int stormFromTick = 6219u;
    const unsigned int stormToTick   = 6197u;

    while (clock.getPredictionTick() < stormFromTick)
        driveOneAtDrift(clock, est, 0);
    REQUIRE(clock.getPredictionTick() == stormFromTick);
    REQUIRE(clock.getDiagnostics().hardResyncCount() == 0u);

    cfg.softDriftThresholdTicks  = 3;
    cfg.hardResyncThresholdTicks = 21;

    unsigned int callbackTick = 0u;
    clock.registerResyncCallback([&](unsigned int newTick) { callbackTick = newTick; });

    est.recordAuthorityTick(stormToTick - est.getPredictionOffsetTicks());
    REQUIRE(clock.advancePrediction() == ClientPredictionClock::AdvanceResult::HardResync);

    REQUIRE(clock.getDiagnostics().hardResyncCount()        == 1u);
    REQUIRE(clock.getDiagnostics().lastHardResyncFromTick() == stormFromTick);
    REQUIRE(clock.getDiagnostics().lastHardResyncToTick()   == stormToTick);
    REQUIRE(callbackTick                                    == stormToTick);
    REQUIRE(clock.getPredictionTick()                       == stormToTick);
    REQUIRE(clock.getDiagnostics().skipCount()              == 0u);
    REQUIRE(clock.getDiagnostics().stallCount()             == 0u);
}

// ---------------------------------------------------------------------------
// ⛔⛔ THE FENCE - the seven words cannot reach any clock decision.
//
// One fixed drift lifecycle (dead band, graduated skips, graduated stalls, a tier-debt
// stall, a hard resync and a tail) is driven CLEAN and then again for every combination
// of scribbler seed at the three points, recording every clock output after every call.
// Every scribbled trace must be byte-identical to the clean one.
//
// ⛔ `finalCounters` is deliberately outside `operator==`: the seam is the ONE thing the
// runs are allowed to differ in, and the sweep requires that they actually do.
//
// A trace that captured nothing would compare equal forever, so the last run moves ONE
// REAL INPUT - a fourth tick of tier debt.
//
// ⭐ It REQUIRES the trace to move, so the instrument is proven sensitive in the same case
// that proves the seam invisible.
// ---------------------------------------------------------------------------

namespace
{
    struct ClockTrace
    {
        std::vector<unsigned int> values;
        std::vector<unsigned int> finalCounters;
        bool operator==(const ClockTrace& other) const { return values == other.values; }
    };

    void recordClockOutputs(const ClientPredictionClock& clock,
                            ClientPredictionClock::AdvanceResult result, ClockTrace& trace)
    {
        trace.values.push_back(static_cast<unsigned int>(result));
        trace.values.push_back(clock.getPredictionTick());
        trace.values.push_back(clock.getResimulationTick());
        trace.values.push_back(static_cast<unsigned int>(clock.evaluateDrift()));
        trace.values.push_back(clock.getRequiredInputDelayIncreaseStallTicks());
        trace.values.push_back(clock.isResimulating() ? 1u : 0u);
    }

    // A nonzero seed scribbles the seven words at that point; 0 means do not scribble.
    // `extraDebtTicks` is the sensitivity knob and is the only REAL input that ever moves.
    void driveTracedClockLifecycle(ClockTrace& trace, unsigned int afterDeadBand,
                                   unsigned int afterGraduated, unsigned int afterDebt,
                                   unsigned int extraDebtTicks)
    {
        TimeConfig cfg;
        cfg.minTicksBeforeDriftCheck = 0;
        cfg.softDriftThresholdTicks  = 1000;
        cfg.hardResyncThresholdTicks = 5000;
        cfg.gradualCorrectionRate    = 4;
        cfg.tickFrequency            = 60.0;

        NetworkTimeEstimator est(cfg, nullptr);
        ClientPredictionClock clock(cfg, est, nullptr);
        clock.registerResyncCallback([&](unsigned int newTick) { trace.values.push_back(newTick); });

        for (unsigned int i = 0u; i < 30u; ++i)
            recordClockOutputs(clock, driveOneAtDrift(clock, est, 0), trace);
        if (afterDeadBand != 0u)
            clock.editDiagnostics().scribbleEventCountersForFenceTest(afterDeadBand);

        cfg.softDriftThresholdTicks = 3;
        for (unsigned int i = 0u; i < 12u; ++i)
            recordClockOutputs(clock, driveOneAtDrift(clock, est, +20), trace);
        for (unsigned int i = 0u; i < 12u; ++i)
            recordClockOutputs(clock, driveOneAtDrift(clock, est, -20), trace);
        if (afterGraduated != 0u)
            clock.editDiagnostics().scribbleEventCountersForFenceTest(afterGraduated);

        cfg.softDriftThresholdTicks = 1000;
        clock.requestInputDelayIncreaseStall(static_cast<int32_t>(3u + extraDebtTicks));
        for (unsigned int i = 0u; i < 6u; ++i)
            recordClockOutputs(clock, driveOneAtDrift(clock, est, 0), trace);
        if (afterDebt != 0u)
            clock.editDiagnostics().scribbleEventCountersForFenceTest(afterDebt);

        cfg.softDriftThresholdTicks  = 3;
        cfg.hardResyncThresholdTicks = 21;
        recordClockOutputs(clock, driveOneAtDrift(clock, est, -22), trace);
        for (unsigned int i = 0u; i < 4u; ++i)
            recordClockOutputs(clock, driveOneAtDrift(clock, est, 0), trace);

        const ClientPredictionClock::Diagnostics seam = clock.getDiagnostics();
        trace.finalCounters = { seam.skipCount(), seam.lastSkipTick(),
                               seam.stallCount(), seam.lastStallTick(),
                               seam.hardResyncCount(), seam.lastHardResyncFromTick(),
                               seam.lastHardResyncToTick() };
    }
}

TEST_CASE("ClientPredictionClock.Events.TheEventCountersCannotReachAnyClockDecision",
          "[PCTM][ClientPredictionClock][ClockEvents]")
{
    ClockTrace clean;
    driveTracedClockLifecycle(clean, 0u, 0u, 0u, 0u);
    REQUIRE(clean.values.size() > 300u);   // the trace is substantial, not a stub

    unsigned int poisonedRuns = 0u;
    for (unsigned int afterDeadBand = 1u; afterDeadBand <= 4u; ++afterDeadBand)
    for (unsigned int afterGraduated = 1u; afterGraduated <= 4u; ++afterGraduated)
    for (unsigned int afterDebt = 1u; afterDebt <= 4u; ++afterDebt)
    {
        ClockTrace scribbled;
        driveTracedClockLifecycle(scribbled, afterDeadBand, afterGraduated, afterDebt, 0u);
        REQUIRE(scribbled == clean);
        if (scribbled.finalCounters != clean.finalCounters)
            ++poisonedRuns;
    }
    REQUIRE(poisonedRuns == 64u);   // every seed really did poison the seam

    ClockTrace sensitivity;
    driveTracedClockLifecycle(sensitivity, 0u, 0u, 0u, 1u);
    REQUIRE(sensitivity.values.size() == clean.values.size());
    REQUIRE_FALSE(sensitivity == clean);
}

// ---------------------------------------------------------------------------
// og-input-latency task 27 — the one-shot re-centre (TimeConfig::recentreSettleSamples)
//
// The rig replays a join the way the host drives it: one frame is at most one
// clock sample (the og-sim-net client coalesces, SimNetClient-rationale.md §8)
// and then that frame's steps. The join frame runs a catch-up BATCH of steps
// after the hard resync, with no new sample in between, which is the Ph3 gate's
// landing mechanism. After the join the server advances one tick per frame and
// the client steps once per frame.
// ---------------------------------------------------------------------------
namespace recentreTest
{
    struct JoinRig
    {
        explicit JoinRig(uint32_t settleSamples)
        {
            cfg.recentreSettleSamples = settleSamples;
        }

        TimeConfig cfg;
        std::vector<std::string> lines;
        NetworkTimeEstimator est{cfg, nullptr};
        ClientPredictionClock clock{cfg, est, [this](const char* line) { lines.emplace_back(line); }};
        std::vector<ClientPredictionClock::AdvanceResult> results;
        std::vector<unsigned int> predictionTicks;
        unsigned int authority = 1502u;
        int inputDrift = 0;
        std::vector<int> leadErrors;   // leadMinusTarget() at every frame
        std::vector<unsigned int> offsets;   // the prediction offset at every frame
        bool consistent = true;

        // One frame: a sample (an RTT reading and an authority tick), then `steps` advances.
        void frame(double rttSeconds, unsigned int authorityTick, unsigned int steps)
        {
            est.updateRTT(rttSeconds);
            est.recordAuthorityTick(authorityTick);
            inputDrift = static_cast<int>(est.getTargetPredictionTick()) - static_cast<int>(clock.getPredictionTick());
            leadErrors.push_back(leadMinusTarget());
            offsets.push_back(est.getPredictionOffsetTicks());
            for (unsigned int step = 0; step < steps; ++step)
                advance();
        }

        void advance()
        {
            const ClientPredictionClock::DriftAction action = clock.evaluateDrift();
            const bool owedStall = clock.getRequiredInputDelayIncreaseStallTicks() > 0u;
            const ClientPredictionClock::AdvanceResult result = clock.advancePrediction();
            using A = ClientPredictionClock::AdvanceResult;
            using D = ClientPredictionClock::DriftAction;
            if (action == D::None && result != A::Normal) consistent = false;
            if (result == A::Skip && action != D::Skip) consistent = false;
            if (result == A::Stall && !owedStall && action != D::Stall) consistent = false;
            if ((result == A::HardResync) != (action == D::HardResync)) consistent = false;
            results.push_back(result);
            predictionTicks.push_back(clock.getPredictionTick());
        }

        // Steady frames after the join: the server moves one tick, the client steps once.
        void steady(double rttSeconds, unsigned int frames)
        {
            for (unsigned int i = 0; i < frames; ++i)
                frame(rttSeconds, ++authority, 1u);
        }

        // The last frame's lead minus its target, in ticks: the tick a Normal advance
        // steps (the last stepped tick + 1) minus the target, read where the clock
        // reads its drift. This is estimator_trace.py's lead - (target - server).
        int leadMinusTarget() const
        {
            return 1 - inputDrift;
        }

        // The median lead minus target over the last `frames` frames (odd), as P5-25's AC reads it.
        int p50LeadMinusTarget(size_t frames) const
        {
            std::vector<int> last(leadErrors.end() - static_cast<std::ptrdiff_t>(frames), leadErrors.end());
            std::sort(last.begin(), last.end());
            return last[frames / 2u];
        }

        // The median and the mean lead minus target over frames [from, to).
        int p50LeadMinusTarget(size_t from, size_t to) const
        {
            std::vector<int> span(leadErrors.begin() + static_cast<std::ptrdiff_t>(from),
                                  leadErrors.begin() + static_cast<std::ptrdiff_t>(to));
            std::sort(span.begin(), span.end());
            return span[span.size() / 2u];
        }

        double meanLeadMinusTarget(size_t from, size_t to) const
        {
            double sum = 0.0;
            for (size_t i = from; i < to; ++i)
                sum += leadErrors[i];
            return sum / static_cast<double>(to - from);
        }

        size_t count(ClientPredictionClock::AdvanceResult kind, size_t from = 0u) const
        {
            size_t n = 0u;
            for (size_t i = from; i < results.size(); ++i)
                n += results[i] == kind ? 1u : 0u;
            return n;
        }

        std::vector<std::string> recentreLines() const
        {
            std::vector<std::string> out;
            for (const std::string& line : lines)
            {
                if (line.find("[Clock.Recentre]") != std::string::npos)
                    out.push_back(line);
            }
            return out;
        }
    };

    // The Ph3 join (gate_Ph3 §3.1): sixteen queued clock messages coalesce to the
    // newest one, read with a 0 ms smoothed RTT, and the join frame runs a batch of
    // 16 steps; then a 10 ms link.
    void ph3Join(JoinRig& rig)
    {
        rig.frame(0.0, rig.authority, 16u);
        rig.steady(0.010, 300u);
    }

    // P4-12's inflated first RTT: the first reading is the 104 ms first-bucket
    // value (under the cold-start ceiling, so it seeds the estimate), then the
    // link's 10 ms. The hard resync lands on the inflated target, which then falls.
    void inflatedFirstRttJoin(JoinRig& rig)
    {
        rig.frame(0.104, rig.authority, 1u);
        rig.steady(0.010, 300u);
    }

    // A join whose lead error flickers across a tick boundary (review_27 F3): the
    // join frame reads the fresh authority tick and runs `batchSteps`, so the lead
    // lands batchSteps - 1 above target; afterwards the authority tick arrives one
    // tick stale on `staleFrames` of every `period` frames, which reads one tick more.
    void flickeringJoin(JoinRig& rig, unsigned int batchSteps, unsigned int staleFrames, unsigned int period)
    {
        rig.frame(0.010, rig.authority, batchSteps);
        for (unsigned int i = 0; i < 300u; ++i)
        {
            ++rig.authority;
            const bool stale = (i % period) < staleFrames;
            rig.frame(0.010, stale ? rig.authority - 1u : rig.authority, 1u);
        }
    }

    // A raw RTT reading held for a run of samples (an averaged ping's one-second bucket).
    struct RttRun
    {
        double       ms;
        unsigned int samples;
    };

    // The Ph3 gate's shaped rendered (b) client1s (P5-25 rework 1, Ph3_b_C1_27 and Ph3_b_C2_27): the
    // raw RTT their first 1300 NTE lines imply (the smoothed RTT's EWMA inverted, runs merged within
    // 0.6 ms). The first real reading is an outlier against the 0 seed: rejected 29 times, the 30th
    // re-seeds onto it and fills the window, so it decides on the start-up spike. Then the RTT falls
    // about 50 ms over 4 s and settles at 78-87 ms (offset 5-6).
    const RttRun kPh3bC1Rtt[] = {
        {0.000, 28u}, {131.571, 51u}, {106.000, 50u}, {96.523, 42u}, {90.000, 45u}, {77.214, 44u},
        {78.285, 46u}, {81.500, 44u}, {85.071, 91u}, {85.785, 45u}, {83.643, 192u}, {84.714, 47u},
        {87.211, 93u}, {86.143, 48u}, {85.429, 45u}, {84.000, 47u}, {85.429, 48u}, {82.214, 41u},
        {84.358, 45u}, {82.214, 48u}, {80.429, 50u}, {83.285, 50u}, {79.000, 48u}, {82.215, 12u},
    };
    const RttRun kPh3bC2Rtt[] = {
        {0.000, 28u}, {139.857, 47u}, {112.286, 49u}, {100.667, 48u}, {94.892, 45u}, {78.964, 47u},
        {80.750, 48u}, {83.286, 45u}, {85.071, 48u}, {86.142, 49u}, {84.357, 45u}, {81.857, 48u},
        {79.357, 46u}, {80.786, 95u}, {82.214, 45u}, {84.358, 45u}, {82.214, 88u}, {80.786, 45u},
        {80.072, 45u}, {80.785, 46u}, {77.929, 48u}, {80.423, 94u}, {82.571, 49u}, {85.072, 46u},
        {81.856, 61u},
    };

    // Their 10-30 s window, in samples (frames here).
    constexpr size_t kPh3bSettledFrom = 440u;
    constexpr size_t kPh3bSettledTo   = 1250u;

    // One sample per frame: the join frame runs `batchSteps` (its landing above the floor target),
    // then the server moves one tick and the client steps once per frame.
    template <size_t N>
    void rttReplay(JoinRig& rig, unsigned int batchSteps, const RttRun (&runs)[N])
    {
        bool joinFrame = true;
        for (const RttRun& run : runs)
        {
            for (unsigned int i = 0; i < run.samples; ++i)
            {
                if (joinFrame)
                    rig.frame(run.ms / 1000.0, rig.authority, batchSteps);
                else
                    rig.frame(run.ms / 1000.0, ++rig.authority, 1u);
                joinFrame = false;
            }
        }
    }

    // The Ph3 gate's loopback (c) main client Ph3_c_C1_27b client1 (P5-25 rework 2, R13), its first 900 NTE lines (15 s)
    // as its log has them, one sample each: the raw RTT the smoothed RTT's EWMA implies (41 zero readings, the 89 ms
    // reading rejected 29 times then re-seeded, then the once-a-second ping buckets, runs merged within 0.05 ms), the
    // authority tick it carried and the advances the client ran before the next sample. Replayed through rework 3's
    // clock and estimator, the unmerged lines reproduce all 900 target ticks, every advance (16 Stalls, 1 Skip) and the
    // three [Clock.Recentre] lines byte for byte.
    const RttRun kPh3cC1Rtt[] = {
        {0.000, 41u}, {89.429, 59u}, {52.069, 59u}, {40.094, 60u}, {33.390, 60u}, {15.073, 179u}, {15.431, 120u}, {15.782, 60u},
        {15.426, 60u}, {15.782, 202u},
    };
    constexpr unsigned int kPh3cC1FirstAuthority = 1508u;
    constexpr unsigned int kPh3cC1JoinSteps      = 16u;   // the hard resync and its catch-up batch
    // One character per sample after the first: 'a' + 3 * (the advances after it) + (the authority ticks it moved - 1).
    const char kPh3cC1Frames[] =
        "lgagadddgagdagdagdagdagdagdagdagdddddddddddddddddddddddddddddddddddddddddddddddeddgdeddddddddddddddddgadgadgad"
        "gadgadgadgadgdagdagdagdagddddddddddddddddddddddddddddddddddddddddddedddgadgadgadgadgadgadgadgadgdagdagdagdagda"
        "gdagdagdagdagdagdagdagddddddddddddddddddddddddddddddddddddddddgaddddddddddddddddddddeddddddddddddddddddddddddd"
        "gadgadgadgadgdagdagdagdddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddedddgadgadgadgadgad"
        "gadgdagdagdagdagdagdagddddddddddddddddddddddddddddddedddddddddddddddddddddddddddddddddgadgadgadgadgadgadgadgad"
        "gadgdagdagdagdagdagddddddddddddddddddddddddddddddddddddddddddddddddddddedddddddddddgadgadgadgadgadgdagdagdagda"
        "gdagdagdddddddddddddddddddddeddddddddddddgadgadgadgadgadgadgadgdagdagdagdagddddddddddddddddddddddddddhdddddded"
        "dddddddgadgadgadgadgadgdagdagdagddddddddddddddddddddddddddddddddddddddddddedddddddddgadgadddddddgadgadgadgadga"
        "dgadgadgdagdagdagda";
    static_assert(sizeof(kPh3cC1Frames) == 900u, "one character per sample after the first (899) plus the terminator");

    void ph3cC1Replay(JoinRig& rig)
    {
        rig.authority = kPh3cC1FirstAuthority;
        size_t sample = 0u;
        for (const RttRun& run : kPh3cC1Rtt)
        {
            for (unsigned int i = 0; i < run.samples; ++i, ++sample)
            {
                if (sample == 0u)
                {
                    rig.frame(run.ms / 1000.0, rig.authority, kPh3cC1JoinSteps);
                    continue;
                }
                const int code = kPh3cC1Frames[sample - 1u] - 'a';
                rig.authority += static_cast<unsigned int>(code % 3 + 1);
                rig.frame(run.ms / 1000.0, rig.authority, static_cast<unsigned int>(code / 3));
            }
        }
    }

    // review_27 R1's jittery steady links (task 27 rework 3), one RTT reading per frame from a seeded
    // mt19937_64 (its output is fixed by the standard; the uniform and the exponential are written
    // out here because the <random> distributions are not).
    struct JitteryLink
    {
        explicit JitteryLink(uint64_t seed) : rng(seed) {}

        std::mt19937_64 rng;

        double uniform() { return static_cast<double>(rng() >> 11) * 0x1.0p-53; }
        double exponential() { return -std::log(1.0 - uniform()); }

        // 70 ms + Exp(30 ms): a heavy-tailed link with no RTT step.
        double exp30Ms() { return 70.0 + 30.0 * exponential(); }

        // Bursty Wi-Fi: 75 ms + Exp(5 ms), with an Exp(80 ms) burst on 5 % of the readings.
        double wifiMs()
        {
            const bool burst = uniform() < 0.05;
            return 75.0 + (burst ? 80.0 : 5.0) * exponential();
        }
    };
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.ThePh3JoinLandsOnTargetAfterTheSettleWindow",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    recentreTest::JoinRig today(0u);
    recentreTest::ph3Join(today);
    INFO("recentreSettleSamples=0 lead - target " << today.leadMinusTarget());
    CHECK(today.leadMinusTarget() == 4);               // the dead band holds the batch's overshoot
    CHECK(today.count(A::HardResync) == 1u);
    CHECK(today.count(A::Skip) == 0u);
    CHECK(today.recentreLines().empty());

    recentreTest::JoinRig fixed(TimeConfig{}.recentreSettleSamples);
    recentreTest::ph3Join(fixed);
    INFO("default lead - target " << fixed.leadMinusTarget());
    CHECK(std::abs(fixed.leadMinusTarget()) * 2 <= 1); // the settled lead within +/-0.5 tick of the target
    CHECK(fixed.count(A::HardResync) == 1u);
    CHECK(fixed.count(A::Skip) == 0u);
    CHECK(fixed.count(A::Stall) == today.count(A::Stall) + 4u);   // exactly the four ticks the dead band held
    CHECK(fixed.consistent);
    CHECK_FALSE(fixed.clock.isRecentreArmed());
    CHECK_FALSE(fixed.clock.isRecentring());

    const std::vector<std::string> lines = fixed.recentreLines();
    REQUIRE(lines.size() == 1u);
    CHECK(lines[0] == "[Warning][Clock.Recentre] fromLead=9 toLead=4 owed=5 target=1536 rttMs=9.924 samples=30 cause=join");
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AnInflatedFirstRttJoinRecentresToTarget",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    recentreTest::JoinRig today(0u);
    recentreTest::inflatedFirstRttJoin(today);
    INFO("recentreSettleSamples=0 lead - target " << today.leadMinusTarget());
    CHECK(today.leadMinusTarget() == 3);
    CHECK(today.recentreLines().empty());

    recentreTest::JoinRig fixed(TimeConfig{}.recentreSettleSamples);
    recentreTest::inflatedFirstRttJoin(fixed);
    INFO("default lead - target " << fixed.leadMinusTarget());
    CHECK(std::abs(fixed.leadMinusTarget()) * 2 <= 1);
    CHECK(fixed.count(A::Skip) == 0u);
    CHECK(fixed.count(A::Stall) == today.count(A::Stall) + 3u);
    CHECK(fixed.consistent);
    CHECK(fixed.recentreLines().size() == 1u);
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.ALandingBelowTargetSkipsUpToIt",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // The IpNetDriver pattern: the first sample is two ticks stale, so the resync
    // lands two ticks low, and the dead band keeps it there.
    const auto join = [](recentreTest::JoinRig& rig) {
        rig.frame(0.010, rig.authority - 2u, 1u);
        rig.steady(0.010, 300u);
    };

    recentreTest::JoinRig today(0u);
    join(today);
    CHECK(today.leadMinusTarget() == -2);
    CHECK(today.count(A::Skip) == 0u);

    recentreTest::JoinRig fixed(TimeConfig{}.recentreSettleSamples);
    join(fixed);
    CHECK(fixed.leadMinusTarget() == 0);
    CHECK(fixed.count(A::Skip) == 2u);
    CHECK(fixed.count(A::Stall) == 0u);
    CHECK(fixed.consistent);
    REQUIRE(fixed.recentreLines().size() == 1u);
    CHECK(fixed.recentreLines()[0].find("fromLead=2 toLead=4 ") != std::string::npos);
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.ZeroReadingsDoNotFillTheWindow",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // An averaged ping reads 0 until its first bucket fills (60 frames here), then
    // the link's 100 ms. The window starts at the first real reading, so the
    // decision sees the 100 ms target, not the floor. The outlier gate re-seeds
    // onto 100 ms on the window's last sample, so this case also pins that the
    // decision takes its own frame's offset, not the median run's.
    const auto join = [](recentreTest::JoinRig& rig) {
        rig.frame(0.0, rig.authority, 1u);
        rig.steady(0.0, 59u);
        rig.steady(0.100, 300u);
    };

    recentreTest::JoinRig today(0u);
    join(today);
    CHECK(today.leadMinusTarget() == -2);

    recentreTest::JoinRig fixed(TimeConfig{}.recentreSettleSamples);
    join(fixed);
    CHECK(fixed.leadMinusTarget() == 0);
    CHECK(fixed.count(A::Skip) == 2u);
    CHECK(fixed.consistent);
    const std::vector<std::string> lines = fixed.recentreLines();
    REQUIRE(lines.size() == 1u);
    CHECK(lines[0] == "[Warning][Clock.Recentre] fromLead=4 toLead=6 owed=-2 target=1597 rttMs=100.000 samples=30 cause=join");
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AJoinOnTargetDoesNothing",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    const auto join = [](recentreTest::JoinRig& rig) {
        rig.frame(0.010, rig.authority, 1u);
        rig.steady(0.010, 300u);
    };

    recentreTest::JoinRig today(0u);
    join(today);
    recentreTest::JoinRig fixed(TimeConfig{}.recentreSettleSamples);
    join(fixed);

    CHECK(today.leadMinusTarget() == 0);
    CHECK(fixed.leadMinusTarget() == 0);
    CHECK(fixed.results == today.results);
    CHECK(fixed.predictionTicks == today.predictionTicks);
    CHECK(fixed.consistent);
    const std::vector<std::string> lines = fixed.recentreLines();
    REQUIRE(lines.size() == 1u);
    CHECK(lines[0] == "[Warning][Clock.Recentre] fromLead=4 toLead=4 owed=0 target=1536 rttMs=10.000 samples=30 cause=join skipped=within1");
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AHardResyncReArmsItAndSteadyStateNeverFiresIt",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
    recentreTest::ph3Join(rig);
    REQUIRE(rig.leadMinusTarget() == 0);
    REQUIRE(rig.recentreLines().size() == 1u);

    // Steady state: the server holds its tick for two frames, the client keeps
    // stepping, so the lead rises by two inside the dead band. No second re-centre.
    rig.frame(0.010, rig.authority, 1u);
    rig.frame(0.010, rig.authority, 1u);
    rig.steady(0.010, 300u);
    CHECK(rig.leadMinusTarget() == 2);
    CHECK(rig.recentreLines().size() == 1u);
    CHECK_FALSE(rig.clock.isRecentreArmed());

    // A hard resync re-arms it: the server jumps 40 ticks and the client runs a
    // six-step batch in that frame.
    const size_t before = rig.results.size();
    rig.authority += 40u;
    rig.frame(0.010, rig.authority, 6u);
    CHECK(rig.count(A::HardResync, before) == 1u);
    CHECK(rig.clock.isRecentreArmed());
    rig.steady(0.010, 300u);
    CHECK(rig.leadMinusTarget() == 0);
    REQUIRE(rig.recentreLines().size() == 2u);
    CHECK(rig.recentreLines()[1].find(" cause=resync") != std::string::npos);
    CHECK(rig.consistent);
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.ZeroDisablesItExactly",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    // 0 never arms it, so the clock is the pre-re-centre clock: the same results,
    // ticks and lines as a window that can never fill.
    recentreTest::JoinRig off(0u);
    recentreTest::JoinRig never(std::numeric_limits<uint32_t>::max());
    for (recentreTest::JoinRig* rig : {&off, &never})
    {
        recentreTest::ph3Join(*rig);
        rig->authority += 40u;
        rig->frame(0.010, rig->authority, 6u);
        rig->steady(0.010, 300u);
    }
    CHECK(off.results == never.results);
    CHECK(off.predictionTicks == never.predictionTicks);
    CHECK(off.lines == never.lines);
    CHECK(off.recentreLines().empty());
    CHECK_FALSE(off.clock.isRecentreArmed());
    CHECK(off.consistent);
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AFlickeringLeadIsDecidedOnItsMedianAndPaidInFull",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // review_27 F3: a lead error that flickers across a tick boundary frame to frame.
    // The decision reads the median of the last frames, not one frame, and the
    // correction it owes is paid in full: no single on-target frame ends it.
    struct Scenario
    {
        unsigned int batchSteps;
        unsigned int staleFrames;
        unsigned int period;
        int          landing;   // the p50 lead minus target the dead band keeps
        const char*  line;      // decided on steady frame 29, a fresh one: server 1532, offset 4
    };
    const Scenario scenarios[] = {
        // +1 on 2 frames of 3, on target on the third
        {1u, 2u, 3u, 1, "[Warning][Clock.Recentre] fromLead=5 toLead=4 owed=1 target=1536 rttMs=10.000 samples=30 cause=join"},
        // +1 on 4 frames of 5
        {1u, 4u, 5u, 1, "[Warning][Clock.Recentre] fromLead=5 toLead=4 owed=1 target=1536 rttMs=10.000 samples=30 cause=join"},
        // a Ph3-style +4 landing that reads +3 on 1 frame of 3
        {4u, 2u, 3u, 4, "[Warning][Clock.Recentre] fromLead=8 toLead=4 owed=4 target=1536 rttMs=10.000 samples=30 cause=join"},
    };
    for (const Scenario& s : scenarios)
    {
        INFO("batch " << s.batchSteps << ", stale on " << s.staleFrames << " of " << s.period << " frames");

        recentreTest::JoinRig today(0u);
        recentreTest::flickeringJoin(today, s.batchSteps, s.staleFrames, s.period);
        CHECK(today.p50LeadMinusTarget(121u) == s.landing);
        CHECK(today.count(A::Stall) == 0u);

        recentreTest::JoinRig fixed(TimeConfig{}.recentreSettleSamples);
        recentreTest::flickeringJoin(fixed, s.batchSteps, s.staleFrames, s.period);
        CHECK(fixed.p50LeadMinusTarget(121u) == 0);
        CHECK(fixed.count(A::Stall) == static_cast<size_t>(s.landing));   // exactly the median's ticks
        CHECK(fixed.count(A::Skip) == 0u);
        CHECK(fixed.consistent);
        CHECK_FALSE(fixed.clock.isRecentring());
        const std::vector<std::string> lines = fixed.recentreLines();
        REQUIRE(lines.size() == 1u);
        CHECK(lines[0] == s.line);
    }
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.ThePh3bJoinsReArmWhenTheirStartUpRttSpikeSettles",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // Ph3 R10 (task 27 rework 2): both (b) client1s decided on the start-up RTT spike, offset 8 and 9,
    // and the target then fell 2-4 ticks, which the dead band held at +2 for the session. The replay
    // reproduces both logged join decisions exactly; the re-arm re-decides once the RTT-only offset
    // has stood 2 ticks away for 15 advances (C2 twice: 9 -> 7, then 7 -> 5). Re-pinned in rework 3
    // (the RTT-only trigger): each re-arm lands 12-14 frames sooner, because the full offset's
    // jitter term, which the RTT's fall itself raises, no longer holds the run back.
    struct Client
    {
        const char*              name;
        unsigned int             batchSteps;   // the join's landing above the floor target, plus one
        size_t                   skips;
        size_t                   stalls;
        std::vector<std::string> lines;
    };
    const Client clients[] = {
        {"Ph3_b_C1_27 client1", 5u, 0u, 2u,
         {"[Warning][Clock.Recentre] fromLead=8 toLead=8 owed=0 target=1567 rttMs=131.571 samples=30 cause=join skipped=within1",
          "[Warning][Clock.Recentre] fromLead=8 toLead=6 owed=2 target=1687 rttMs=91.513 samples=30 cause=rearm fromTarget=8 toTarget=6"}},
        {"Ph3_b_C2_27 client1", 4u, 2u, 4u,
         {"[Warning][Clock.Recentre] fromLead=7 toLead=9 owed=-2 target=1568 rttMs=139.857 samples=30 cause=join",
          "[Warning][Clock.Recentre] fromLead=9 toLead=7 owed=2 target=1639 rttMs=104.395 samples=30 cause=rearm fromTarget=9 toTarget=7",
          "[Warning][Clock.Recentre] fromLead=7 toLead=5 owed=2 target=1775 rttMs=79.961 samples=30 cause=rearm fromTarget=7 toTarget=5"}},
    };
    for (const Client& c : clients)
    {
        INFO(c.name);
        recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
        if (c.batchSteps == 5u)
            recentreTest::rttReplay(rig, c.batchSteps, recentreTest::kPh3bC1Rtt);
        else
            recentreTest::rttReplay(rig, c.batchSteps, recentreTest::kPh3bC2Rtt);

        const int    p50  = rig.p50LeadMinusTarget(recentreTest::kPh3bSettledFrom, recentreTest::kPh3bSettledTo);
        const double mean = rig.meanLeadMinusTarget(recentreTest::kPh3bSettledFrom, recentreTest::kPh3bSettledTo);
        INFO("settled lead - target p50 " << p50 << " mean " << mean);
        CHECK(std::abs(p50) <= 1);            // P5-25's reading of task 27's AC (lead 13:12): every client within +/-1
        CHECK(std::abs(mean) <= 0.5);         // and the mean within +/-0.5
        CHECK(rig.count(A::HardResync) == 1u);
        CHECK(rig.count(A::Skip) == c.skips);
        CHECK(rig.count(A::Stall) == c.stalls);
        CHECK(rig.consistent);
        CHECK(rig.recentreLines() == c.lines);
    }
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AOneTickOffsetWobbleNeverReArmsIt",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // A steady link whose RTT swings in one-second buckets: the offset's ceil() wobbles one tick
    // around the decision's offset (or one tick off it) and never stands two away, so it never re-arms.
    struct Wobble
    {
        double       joinMs;
        double       highMs;
        double       lowMs;
        unsigned int decisionOffset;
        unsigned int lowestOffset;
        unsigned int highestOffset;
    };
    const Wobble wobbles[] = {
        {80.0, 88.0, 78.0, 5u, 5u, 6u},    // decided at 5, wobbles 5..6
        {88.0, 100.0, 78.0, 6u, 5u, 7u},   // decided at 6, wobbles 5..7
    };
    for (const Wobble& w : wobbles)
    {
        INFO("join " << w.joinMs << " ms, wobble " << w.lowMs << ".." << w.highMs << " ms");
        recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
        rig.frame(w.joinMs / 1000.0, rig.authority, 1u);
        rig.steady(w.joinMs / 1000.0, 59u);
        REQUIRE(rig.recentreLines().size() == 1u);
        CHECK(rig.offsets.back() == w.decisionOffset);
        for (int i = 0; i < 10; ++i)
        {
            rig.steady(w.highMs / 1000.0, 50u);
            rig.steady(w.lowMs / 1000.0, 50u);
        }
        const auto [lo, hi] = std::minmax_element(rig.offsets.begin() + 100, rig.offsets.end());
        CHECK(*lo == w.lowestOffset);
        CHECK(*hi == w.highestOffset);
        CHECK(rig.recentreLines().size() == 1u);
        CHECK(rig.recentreLines()[0].find(" cause=join") != std::string::npos);
        CHECK_FALSE(rig.clock.isRecentreArmed());
        CHECK(rig.count(A::Skip) == 0u);
        CHECK(rig.count(A::Stall) == 0u);
        CHECK(rig.consistent);
    }
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AMidSessionRttStepReArmsItOnce",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // task 27's F2: an RTT step long after the join's decision, 80 -> 120 ms (offset 5 -> 8). The
    // graduated path takes the two ticks outside the dead band, as before; the re-arm takes the rest.
    // Re-pinned in rework 3 (the RTT-only trigger): it re-arms 3 frames later and logs the RTT-only
    // offsets, toTarget=8 where the full offset read 9.
    const auto session = [](recentreTest::JoinRig& rig) {
        rig.frame(0.080, rig.authority, 1u);
        rig.steady(0.080, 300u);
        rig.steady(0.120, 600u);
    };

    recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
    session(rig);
    INFO("lead - target " << rig.leadMinusTarget());
    CHECK(rig.p50LeadMinusTarget(700u, 901u) == 0);
    CHECK(rig.meanLeadMinusTarget(700u, 901u) == 0.0);
    CHECK(rig.count(A::Skip) == 3u);
    CHECK(rig.count(A::Stall) == 0u);
    CHECK(rig.consistent);
    CHECK_FALSE(rig.clock.isRecentreArmed());
    const std::vector<std::string> lines = rig.recentreLines();
    REQUIRE(lines.size() == 2u);
    CHECK(lines[0] == "[Warning][Clock.Recentre] fromLead=5 toLead=5 owed=0 target=1537 rttMs=80.000 samples=30 cause=join skipped=within1");
    CHECK(lines[1] == "[Warning][Clock.Recentre] fromLead=7 toLead=8 owed=-1 target=1859 rttMs=119.986 samples=30 cause=rearm fromTarget=5 toTarget=8");
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AJitterySteadyLinkDoesNotReArmIt",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // review_27 R1 (task 27 rework 3): ten minutes of a jittery link with no RTT step. The full
    // prediction offset carries twice the jitter EWMA and swings over 10 ticks here (5..21), so a
    // re-arm on it fired 62-108 times a session (rework 2). The re-arm reads the RTT-only offset:
    // at most two re-arms, and no more than 5 Skips + Stalls beyond the clock before the re-arm
    // (task 27 rework 1: the same seeds through the same clock and estimator in a standalone harness).
    enum class Link { Exp30, Wifi };
    struct Session
    {
        const char* name;
        Link        link;
        uint64_t    seed;
        size_t      noRearmSkipsAndStalls;
    };
    const Session sessions[] = {
        {"70 ms + Exp(30 ms), seed 1",  Link::Exp30, 1u,  76u},
        {"70 ms + Exp(30 ms), seed 2",  Link::Exp30, 2u,  80u},
        {"70 ms + Exp(30 ms), seed 27", Link::Exp30, 27u, 82u},
        {"bursty Wi-Fi, seed 1",        Link::Wifi,  1u,  107u},
        {"bursty Wi-Fi, seed 2",        Link::Wifi,  2u,  69u},
        {"bursty Wi-Fi, seed 27",       Link::Wifi,  27u, 94u},
    };
    for (const Session& s : sessions)
    {
        recentreTest::JitteryLink link(s.seed);
        const auto readingMs = [&]() { return s.link == Link::Exp30 ? link.exp30Ms() : link.wifiMs(); };

        recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
        rig.frame(readingMs() / 1000.0, rig.authority, 1u);
        for (unsigned int i = 1; i < 60u * 600u; ++i)
            rig.frame(readingMs() / 1000.0, ++rig.authority, 1u);

        const std::vector<std::string> lines = rig.recentreLines();
        size_t rearms = 0u;
        for (const std::string& line : lines)
            rearms += line.find(" cause=rearm") != std::string::npos ? 1u : 0u;
        const size_t skipsAndStalls = rig.count(A::Skip) + rig.count(A::Stall);
        const auto [lo, hi] = std::minmax_element(rig.offsets.begin() + 600, rig.offsets.end());
        INFO(s.name << ": re-arms " << rearms << ", Skips + Stalls " << skipsAndStalls << " (before the re-arm "
                    << s.noRearmSkipsAndStalls << "), full offset " << *lo << ".." << *hi);
        CHECK(*hi - *lo >= 10u);                   // the full offset does swing
        CHECK(lines.size() == rearms + 1u);       // one join decision, then only re-arms
        CHECK(rearms <= 2u);
        CHECK(skipsAndStalls <= s.noRearmSkipsAndStalls + 5u);
        CHECK(rig.count(A::HardResync) == 1u);
        CHECK(rig.consistent);
    }
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.ThePh3cMainClientDoesNotReArmUnderTheOffsetFloor",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // Ph3 R13 (task 27 rework 4): the loopback RTT fell 52 -> 15 ms after the client's second decision, which moved
    // ceil(smoothedRTT * tickFrequency) 4 -> 2 while the target offset stayed on predOffsetFloorTicks (4). Rework 3's
    // re-arm fired on it and decided on a 0/-1 lead flicker: `fromLead=3 toLead=4 owed=-1 target=1841 rttMs=15.085 ...
    // cause=rearm fromTarget=4 toTarget=2`, one Skip, and the client sat at +1 (p50 +1, mean +0.59 over samples
    // 400-900). The re-arm's offset is floored as the target's is, so that move cannot re-arm: two decisions.
    recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
    recentreTest::ph3cC1Replay(rig);
    REQUIRE(rig.leadErrors.size() == 900u);

    const int    p50  = rig.p50LeadMinusTarget(400u, 900u);
    const double mean = rig.meanLeadMinusTarget(400u, 900u);
    INFO("lead - target over samples 400-900: p50 " << p50 << " mean " << mean);
    CHECK(p50 == 0);
    CHECK(std::abs(mean) <= 0.5);
    CHECK(rig.count(A::HardResync) == 1u);
    CHECK(rig.count(A::Skip) == 0u);
    CHECK(rig.count(A::Stall) == 16u);      // the hard resync's catch-up and the two decisions' 2 + 2
    CHECK(rig.consistent);
    CHECK_FALSE(rig.clock.isRecentreArmed());
    const std::vector<std::string> lines = rig.recentreLines();
    REQUIRE(lines.size() == 2u);
    CHECK(lines[0] == "[Warning][Clock.Recentre] fromLead=8 toLead=6 owed=2 target=1586 rttMs=89.429 samples=30 cause=join");
    CHECK(lines[1] == "[Warning][Clock.Recentre] fromLead=6 toLead=4 owed=2 target=1665 rttMs=52.080 samples=30 cause=rearm fromTarget=6 toTarget=4");
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AnRttFallUnderTheOffsetFloorReArmsOnlyOnTheFlooredOffset",
          "[PCTM][ClientPredictionClock][Recentre]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // Task 27 rework 4: an RTT that falls from above the floor to under it, 15 ms (ceil 1, floored to 4). It re-arms only
    // when the floored offset moved 2 or more from the decision's. From 6 it does, and logs the floored offset. From 5
    // the target falls one tick, inside the dead band, as a one-tick fall above the floor does: no re-arm, the lead
    // stays +1 (src-29's). From 4 the target does not move at all. Rework 3 re-armed on all three (toTarget=2).
    struct Fall
    {
        double      joinMs;
        size_t      stalls;
        int         settled;      // the p50 lead minus target after the fall
        const char* rearm;        // the re-arm's line, or nullptr
    };
    const Fall falls[] = {
        {90.0, 2u, 0, "[Warning][Clock.Recentre] fromLead=6 toLead=4 owed=2 target=1852 rttMs=15.036 samples=30 cause=rearm fromTarget=6 toTarget=4"},
        {75.0, 0u, 1, nullptr},
        {55.0, 0u, 0, nullptr},
    };
    for (const Fall& f : falls)
    {
        INFO("join " << f.joinMs << " ms, then 15 ms");
        recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
        rig.frame(f.joinMs / 1000.0, rig.authority, 1u);
        rig.steady(f.joinMs / 1000.0, 299u);
        rig.steady(0.015, 600u);

        CHECK(rig.p50LeadMinusTarget(600u, 900u) == f.settled);
        CHECK(rig.count(A::Skip) == 0u);
        CHECK(rig.count(A::Stall) == f.stalls);
        CHECK(rig.consistent);
        CHECK_FALSE(rig.clock.isRecentreArmed());
        const std::vector<std::string> lines = rig.recentreLines();
        REQUIRE(lines.size() == (f.rearm != nullptr ? 2u : 1u));
        CHECK(lines[0].find(" owed=0 ") != std::string::npos);
        CHECK(lines[0].find(" cause=join skipped=within1") != std::string::npos);
        if (f.rearm != nullptr)
            CHECK(lines[1] == f.rearm);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
