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
#include <functional>
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
    // ReplicatedTierConsumer fed from the tier's arrival (og-sim-net's, an OnRep
    // until og-input-latency P5-7), with the delta taken from the
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
            afterSample(authorityTick, steps);
        }

        // [P3-10] The same frame, its RTT reading routed by the estimator's source: the
        // engine's, or a symmetric SimClock ping/pong whose RTT field is rttSeconds.
        void sourceFrame(bool simClock, double rttSeconds, unsigned int authorityTick, unsigned int steps)
        {
            if (simClock)
            {
                SimClockRttSample s;
                s.rttSeconds = rttSeconds;
                s.t1Seconds  = rttSeconds / 2.0;
                s.t2Seconds  = rttSeconds / 2.0;
                s.t3Seconds  = rttSeconds;
                est.noteSimClockSample(s);
            }
            else
            {
                est.noteEngineRttSample(rttSeconds);
            }
            afterSample(authorityTick, steps);
        }

        void afterSample(unsigned int authorityTick, unsigned int steps)
        {
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
            return linesWith("[Clock.Recentre]");
        }

        std::vector<std::string> linesWith(const char* needle) const
        {
            std::vector<std::string> out;
            for (const std::string& line : lines)
            {
                if (line.find(needle) != std::string::npos)
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
    // about 50 ms over 4 s and settles at 78-87 ms (offset 5-6). Since og-input-latency P5-28 the replay
    // seeds on that first real reading instead (the 0s do not seed); every pinned value is unchanged.
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
    // three [Clock.Recentre] lines byte for byte. Since og-input-latency P5-28 the 89 ms reading seeds at once (the
    // 0s do not seed); the pinned values below are unchanged.
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
    // Re-recorded by og-input-latency P5-28 (was rttMs=9.924): the join frame's 0 reading no longer seeds the
    // estimator, so the 10 ms link seeds it verbatim instead of an EWMA climbing from 0. The offset is the floor either way.
    CHECK(lines[0] == "[Warning][Clock.Recentre] fromLead=9 toLead=4 owed=5 target=1536 rttMs=10.000 samples=30 cause=join");
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
    // decision sees the 100 ms target, not the floor. Before og-input-latency
    // P5-28 the 0 seeded the estimator and the outlier gate re-seeded onto 100 ms
    // on the window's last sample; since P5-28 the first 100 ms reading seeds and
    // the target is 6 for the whole window. Through P5-28 the decision owed -2
    // (`fromLead=4 toLead=6 owed=-2 target=1597`, two Skips): the join's hard resync
    // landed on the floor target (4) before any reading, and a target 2 above it is
    // inside the dead band. Re-pinned by og-input-latency P3-10: the first 100 ms
    // reading now resyncs the clock once more, onto that target, the window counts
    // from that resync (so the decision is one sample later), and it owes nothing. P3-10 rework 1: the second resync
    // waits for a settled reading (one advance later), so the decision moves one more sample (target 1598 -> 1599).
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
    CHECK(fixed.count(A::Skip) == 0u);
    CHECK(fixed.count(A::HardResync) == 2u);
    CHECK(fixed.consistent);
    const std::vector<std::string> lines = fixed.recentreLines();
    REQUIRE(lines.size() == 1u);
    CHECK(lines[0] == "[Warning][Clock.Recentre] fromLead=6 toLead=6 owed=0 target=1599 rttMs=100.000 samples=30 cause=join skipped=within1");
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
    // Re-pinned by og-input-latency P3-11 (the settle window counts advances with a reading, not samples): the +4
    // landing's join frame runs 3 catch-up advances after its resync, which now count, so it decides 3 frames sooner
    // (was `target=1536 ... samples=30`). Same median, same owed 4, same Stalls; the two batch-1 joins are unchanged.
    const Scenario scenarios[] = {
        // +1 on 2 frames of 3, on target on the third
        {1u, 2u, 3u, 1, "[Warning][Clock.Recentre] fromLead=5 toLead=4 owed=1 target=1536 rttMs=10.000 samples=30 cause=join"},
        // +1 on 4 frames of 5
        {1u, 4u, 5u, 1, "[Warning][Clock.Recentre] fromLead=5 toLead=4 owed=1 target=1536 rttMs=10.000 samples=30 cause=join"},
        // a Ph3-style +4 landing that reads +3 on 1 frame of 3
        {4u, 2u, 3u, 4, "[Warning][Clock.Recentre] fromLead=8 toLead=4 owed=4 target=1533 rttMs=10.000 samples=27 cause=join"},
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
    // Re-pinned by og-input-latency P3-10 (C2 only): its join resync landed 2 ticks under the first reading's target, so
    // the clock now resyncs once more onto it and the join decision owes 0 (was `fromLead=7 toLead=9 owed=-2
    // target=1568`, two Skips). C1's landing was already on that target (drift 1): no second resync, nothing changed.
    // P3-10 rework 1: the second resync waits for a settled reading, one advance later: C2's join decision 1569 -> 1570.
    struct Client
    {
        const char*              name;
        unsigned int             batchSteps;   // the join's landing above the floor target, plus one
        size_t                   hardResyncs;
        size_t                   skips;
        size_t                   stalls;
        std::vector<std::string> lines;
    };
    const Client clients[] = {
        {"Ph3_b_C1_27 client1", 5u, 1u, 0u, 2u,
         {"[Warning][Clock.Recentre] fromLead=8 toLead=8 owed=0 target=1567 rttMs=131.571 samples=30 cause=join skipped=within1",
          "[Warning][Clock.Recentre] fromLead=8 toLead=6 owed=2 target=1687 rttMs=91.513 samples=30 cause=rearm fromTarget=8 toTarget=6"}},
        {"Ph3_b_C2_27 client1", 4u, 2u, 0u, 4u,
         {"[Warning][Clock.Recentre] fromLead=9 toLead=9 owed=0 target=1570 rttMs=139.857 samples=30 cause=join skipped=within1",
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
        CHECK(rig.count(A::HardResync) == c.hardResyncs);
        CHECK(rig.linesWith("cause=firstRtt").size() == c.hardResyncs - 1u);
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
    // Re-pinned by og-input-latency P3-10: the first 89 ms reading (offset 6, the join resync's 4) resyncs the clock once
    // more, onto that target, ending the catch-up walk two ticks early; the join decision then owes 0 (was `fromLead=8
    // toLead=6 owed=2 target=1586`, 16 Stalls in all). The re-arm's decision is unchanged. P3-10 rework 1: the second
    // resync waits for a settled reading, one advance later: the join decision 1587 -> 1588. Re-pinned by og-input-latency
    // P3-11 (the settle window counts advances with a reading, not samples): the replay's samples carry 0 to 2 advances
    // each, and the re-arm's window reaches 30 advances on its 29th sample, one sample sooner (was `target=1665
    // rttMs=52.080 samples=30`); same owed 2, Stalls and settled reading. The join's window still ends on its 30th.
    recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
    recentreTest::ph3cC1Replay(rig);
    REQUIRE(rig.leadErrors.size() == 900u);

    const int    p50  = rig.p50LeadMinusTarget(400u, 900u);
    const double mean = rig.meanLeadMinusTarget(400u, 900u);
    INFO("lead - target over samples 400-900: p50 " << p50 << " mean " << mean);
    CHECK(p50 == 0);
    CHECK(std::abs(mean) <= 0.5);
    CHECK(rig.count(A::HardResync) == 2u);
    CHECK(rig.linesWith("cause=firstRtt offset=4->6").size() == 1u);
    CHECK(rig.count(A::Skip) == 0u);
    CHECK(rig.count(A::Stall) == 14u);      // the catch-up walk before the first reading, and the re-arm's 2
    CHECK(rig.consistent);
    CHECK_FALSE(rig.clock.isRecentreArmed());
    const std::vector<std::string> lines = rig.recentreLines();
    REQUIRE(lines.size() == 2u);
    CHECK(lines[0] == "[Warning][Clock.Recentre] fromLead=6 toLead=6 owed=0 target=1588 rttMs=89.429 samples=30 cause=join skipped=within1");
    CHECK(lines[1] == "[Warning][Clock.Recentre] fromLead=6 toLead=4 owed=2 target=1664 rttMs=52.082 samples=29 cause=rearm fromTarget=6 toTarget=4");
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

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AJoinBeforeTheFirstRttReadingResyncsOnceMoreWhenItLands",
          "[PCTM][ClientPredictionClock][Recentre][ClockSource]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // og-input-latency P3-10 (lead 07:20, from P5-28's smoke): the join's hard resync lands on the first authority
    // tick, before the averaged engine ping has any reading, so on the floor target (4). When the first reading
    // lands and moves the offset, the clock resyncs once more, onto the measured target, and the join decision 30
    // samples later owes nothing. Before P3-10 the dead band held the gap and the decision owed it (P5-28's smoke:
    // owed -1 on 3 of 6 shaped joins). Seen failing with firstRttResyncDue() returning false. Re-pinned by rework 1: the
    // second resync waits for a settled reading (read on two advances in a row), so it and the decision land one later.
    struct Join
    {
        const char*  name;
        unsigned int zeroFrames;     // the join frame and the frames after it that read 0
        double       firstMs;        // the first real reading, held from then on
        const char*  resync;         // the second resync's line, or nullptr
        const char*  decision;
    };
    const Join joins[] = {
        {"100 ms link, a second of zeros", 60u, 100.0,
         "[Warning] PCTM CPC: hard resync oldTick=1566 -> newTick=1569 drift=3 cause=firstRtt offset=4->6",
         "[Warning][Clock.Recentre] fromLead=6 toLead=6 owed=0 target=1599 rttMs=100.000 samples=30 cause=join skipped=within1"},
        {"P5-28 smoke's P528_a2 client2b, 81.571 ms", 41u, 81.571,
         "[Warning] PCTM CPC: hard resync oldTick=1547 -> newTick=1549 drift=2 cause=firstRtt offset=4->5",
         "[Warning][Clock.Recentre] fromLead=5 toLead=5 owed=0 target=1579 rttMs=81.571 samples=30 cause=join skipped=within1"},
        {"a link under the floor: the offset does not move", 30u, 30.0, nullptr,
         "[Warning][Clock.Recentre] fromLead=4 toLead=4 owed=0 target=1565 rttMs=30.000 samples=30 cause=join skipped=within1"},
    };
    for (const Join& j : joins)
    {
        INFO(j.name);
        const auto join = [&j](recentreTest::JoinRig& rig) {
            rig.frame(0.0, rig.authority, 1u);
            rig.steady(0.0, j.zeroFrames - 1u);
            rig.steady(j.firstMs / 1000.0, 300u);
        };

        recentreTest::JoinRig off(0u);
        join(off);
        CHECK(off.count(A::HardResync) == 1u);   // gated with the re-centre: 0 is the clock before both
        CHECK(off.linesWith("cause=firstRtt").empty());

        recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
        join(rig);
        CHECK(rig.leadMinusTarget() == 0);
        CHECK(rig.count(A::Skip) == 0u);
        CHECK(rig.count(A::Stall) == 0u);
        CHECK(rig.consistent);
        const std::vector<std::string> resyncs = rig.linesWith("cause=firstRtt");
        CHECK(rig.count(A::HardResync) == (j.resync != nullptr ? 2u : 1u));
        REQUIRE(resyncs.size() == (j.resync != nullptr ? 1u : 0u));
        if (j.resync != nullptr)
            CHECK(resyncs[0] == j.resync);
        const std::vector<std::string> lines = rig.recentreLines();
        REQUIRE(lines.size() == 1u);
        CHECK(lines[0] == j.decision);
    }

    // A reading that lands before the first authority tick (a SimClock pong usually does): the join's resync
    // already has it, so there is nothing to wait for.
    recentreTest::JoinRig early(TimeConfig{}.recentreSettleSamples);
    early.frame(0.100, early.authority, 1u);
    early.steady(0.100, 300u);
    CHECK(early.count(A::HardResync) == 1u);
    CHECK(early.linesWith("cause=firstRtt").empty());
    CHECK(early.leadMinusTarget() == 0);
}

// og-input-latency P3-10 rework 1: the estimator's read probe, declared a friend in NetworkTimeEstimator.h and
// defined only here. It models what the clock's thread can read while the game thread's updateRTT is landing.
struct NetworkTimeEstimatorReadProbe
{
    // The first seed's stores with only the flag visible: the smoothed RTT and jitter still read their values from
    // before it (0), so the offset reads the floor. show() puts the seed back.
    struct HiddenSeed
    {
        double rtt    = 0.0;
        double jitter = 0.0;
    };
    static HiddenSeed hideSeed(NetworkTimeEstimator& est)
    {
        HiddenSeed seed{est.m_smoothedRTT.load(), est.m_smoothedJitter.load()};
        est.m_smoothedRTT.store(0.0);
        est.m_smoothedJitter.store(0.0);
        return seed;
    }
    static void showSeed(NetworkTimeEstimator& est, const HiddenSeed& seed)
    {
        est.m_smoothedRTT.store(seed.rtt);
        est.m_smoothedJitter.store(seed.jitter);
    }

    // A write that lands right after the next hasFirstRTTSample() load, once; clear() removes the hook.
    static void afterNextFlagLoad(NetworkTimeEstimator& est, std::function<void()> write, unsigned int& fired)
    {
        est.m_afterFirstSampleLoad = [write, &fired]() {
            if (fired++ == 0u)
                write();
        };
    }
    static void clear(NetworkTimeEstimator& est)
    {
        est.m_afterFirstSampleLoad = nullptr;
    }
};

TEST_CASE("PCTM.ClientPredictionClock.Recentre.TheFirstRttResyncSurvivesAMixedReadOfTheSeed",
          "[PCTM][ClientPredictionClock][Recentre][ClockSource]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // og-input-latency P3-10 rework 1 (review R1). Frame-locked, the estimator's updateRTT runs on the game thread and
    // the clock reads it on the physics thread: relaxed atomics, not a snapshot (ThreadingCrossings.md row 10). The
    // second resync is a one-shot, so a mixed read on the advance that first sees the reading must not end its wait.
    // The 100 ms join of AJoinBeforeTheFirstRttReadingResyncsOnceMoreWhenItLands: a second of zero readings, then
    // 100 ms (offset 4 -> 6). Seen failing on the clock before rework 1 (HardResync 1, no cause=firstRtt line).
    const auto zeros = [](recentreTest::JoinRig& rig) {
        rig.frame(0.0, rig.authority, 1u);
        rig.steady(0.0, 59u);
    };
    const auto settled = [](const recentreTest::JoinRig& rig, const char* resync, const char* decision) {
        CHECK(rig.count(A::HardResync) == 2u);
        CHECK(rig.count(A::Skip) == 0u);
        CHECK(rig.count(A::Stall) == 0u);
        CHECK(rig.leadMinusTarget() == 0);
        const std::vector<std::string> resyncs = rig.linesWith("cause=firstRtt");
        REQUIRE(resyncs.size() == 1u);
        CHECK(resyncs[0] == resync);
        const std::vector<std::string> lines = rig.recentreLines();
        REQUIRE(lines.size() == 1u);
        CHECK(lines[0] == decision);
    };

    SECTION("the flag is visible before its seed")
    {
        // The reading's frame: the clock reads the flag true and the floor offset. The next frame reads the seed.
        recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
        zeros(rig);
        rig.est.updateRTT(0.100);
        const NetworkTimeEstimatorReadProbe::HiddenSeed seed = NetworkTimeEstimatorReadProbe::hideSeed(rig.est);
        rig.afterSample(++rig.authority, 1u);
        CHECK(rig.est.hasFirstRTTSample());
        CHECK(rig.offsets.back() == TimeConfig{}.predOffsetFloorTicks);   // the mixed read is modelled
        NetworkTimeEstimatorReadProbe::showSeed(rig.est, seed);
        rig.steady(0.100, 299u);
        CHECK(rig.consistent);
        settled(rig,
                "[Warning] PCTM CPC: hard resync oldTick=1566 -> newTick=1569 drift=3 cause=firstRtt offset=4->6",
                "[Warning][Clock.Recentre] fromLead=6 toLead=6 owed=0 target=1599 rttMs=100.000 samples=30 cause=join skipped=within1");
    }

    SECTION("the flag flips between two loads of one advance")
    {
        // The reading's frame: the seed lands right after the advance's first flag load (advancePrediction alone, as
        // the physics thread runs it: evaluateDrift is a separate call with its own loads).
        recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
        zeros(rig);
        rig.est.recordAuthorityTick(++rig.authority);
        unsigned int fired = 0u;
        NetworkTimeEstimatorReadProbe::afterNextFlagLoad(rig.est, [&rig]() { rig.est.updateRTT(0.100); }, fired);
        rig.results.push_back(rig.clock.advancePrediction());
        NetworkTimeEstimatorReadProbe::clear(rig.est);
        CHECK(fired >= 1u);   // the write landed inside the advance
        CHECK(rig.est.hasFirstRTTSample());
        rig.steady(0.100, 299u);
        CHECK(rig.consistent);
        settled(rig,
                "[Warning] PCTM CPC: hard resync oldTick=1567 -> newTick=1570 drift=3 cause=firstRtt offset=4->6",
                "[Warning][Clock.Recentre] fromLead=6 toLead=6 owed=0 target=1600 rttMs=100.000 samples=30 cause=join skipped=within1");
    }
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.AFourMillisecondSourceStepNeverReArmsIt",
          "[PCTM][ClientPredictionClock][Recentre][ClockSource]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // og-input-latency P3-10 and task 27: the re-arm reads the RTT through the same estimator, so the source switch
    // must not re-arm it. P5-27 measured the engine ping ~4 ms above SimClock (the engine lane's ack turnaround). A
    // 4 ms step is 0.24 tick at 60 Hz: the floored RTT-only offset moves by at most one tick, under the re-arm's two,
    // at every RTT. Swept over 61-120 ms in 1 ms steps, down (the expected sign) and up (the re-based gate). The
    // control, a 40 ms step (2+ ticks), re-arms in the same rig.
    struct Run
    {
        double engineMs;
        double simClockMs;
    };
    const auto session = [](recentreTest::JoinRig& rig, const Run& run) {
        rig.cfg.clockRttSource = TimeConfig::ClockRttSource::SimClock;
        rig.sourceFrame(false, run.engineMs / 1000.0, rig.authority, 1u);
        for (unsigned int i = 1; i < 300u; ++i)
            rig.sourceFrame(false, run.engineMs / 1000.0, ++rig.authority, 1u);
        REQUIRE(rig.recentreLines().size() == 1u);
        REQUIRE(rig.est.getActiveRttSource() == TimeConfig::ClockRttSource::Engine);
        for (unsigned int i = 0; i < 900u; ++i)
            rig.sourceFrame(true, run.simClockMs / 1000.0, ++rig.authority, 1u);
        REQUIRE(rig.est.getActiveRttSource() == TimeConfig::ClockRttSource::SimClock);
    };
    const auto rttOnlyTicks = [](double ms) {
        return std::max(static_cast<unsigned int>(std::ceil(ms / 1000.0 * 60.0)), TimeConfig{}.predOffsetFloorTicks);
    };

    for (int engineMs = 61; engineMs <= 120; ++engineMs)
    {
        for (const double stepMs : {-4.0, 4.0})
        {
            const Run run{static_cast<double>(engineMs), engineMs + stepMs};
            INFO(run.engineMs << " ms -> " << run.simClockMs << " ms");
            const unsigned int from = rttOnlyTicks(run.engineMs);
            const unsigned int to   = rttOnlyTicks(run.simClockMs);
            CHECK((from > to ? from - to : to - from) <= 1u);

            recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
            session(rig, run);
            CHECK(rig.recentreLines().size() == 1u);
            CHECK_FALSE(rig.clock.isRecentreArmed());
            CHECK(rig.count(A::HardResync) == 1u);
            CHECK(rig.count(A::Skip, 300u) == 0u);
            CHECK(rig.count(A::Stall, 300u) == 0u);
            CHECK(rig.est.getOutlierEscapeCount() == 0u);
            CHECK(rig.consistent);
        }
    }

    recentreTest::JoinRig control(TimeConfig{}.recentreSettleSamples);
    session(control, Run{110.0, 70.0});
    CHECK(control.recentreLines().size() == 2u);
    CHECK(control.recentreLines().back().find(" cause=rearm fromTarget=7 toTarget=5") != std::string::npos);
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.TheSettleWindowIsTimeNotSamples",
          "[PCTM][ClientPredictionClock][Recentre][ClockSource]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // og-input-latency P3-11 (P3-10 part B F3): the settle window counts the clock's advances with a reading since the
    // arm, not the estimator's samples. SimClock's pongs come at 20 Hz, one every third frame at 60 Hz; counted in
    // samples its join decided 90 frames (1.5 s) after the resync where the engine's one reading per frame took 30. Both
    // arms here: the seed, then the first authority tick in one frame (the latched sample fed ahead of the tick), the
    // resync onto an 80 ms target (offset 5), steady 80 ms. Each decides on the 30th frame after it, on target.
    struct Arm
    {
        bool         simClock;
        unsigned int framesPerSample;
        const char*  line;
    };
    const Arm arms[] = {
        {true, 3u, "[Warning][Clock.Recentre] fromLead=5 toLead=5 owed=0 target=1537 rttMs=80.000 samples=10 cause=join skipped=within1"},
        {false, 1u, "[Warning][Clock.Recentre] fromLead=5 toLead=5 owed=0 target=1537 rttMs=80.000 samples=30 cause=join skipped=within1"},
    };
    for (const Arm& arm : arms)
    {
        INFO((arm.simClock ? "SimClock, one sample per 3 frames" : "engine, one sample per frame"));
        recentreTest::JoinRig rig(30u);
        rig.sourceFrame(arm.simClock, 0.080, rig.authority, 1u);
        REQUIRE(rig.count(A::HardResync) == 1u);
        REQUIRE(rig.est.getPredictionOffsetTicks() == 5u);
        unsigned int decidedAfter = 0u;
        for (unsigned int frame = 1u; frame <= 120u && decidedAfter == 0u; ++frame)
        {
            if (frame % arm.framesPerSample == 0u)
                rig.sourceFrame(arm.simClock, 0.080, ++rig.authority, 1u);
            else
                rig.afterSample(++rig.authority, 1u);
            if (!rig.recentreLines().empty())
                decidedAfter = frame;
        }
        CHECK(decidedAfter == 30u);
        REQUIRE(rig.recentreLines().size() == 1u);
        CHECK(rig.recentreLines().front() == arm.line);
        CHECK(rig.linesWith("cause=firstRtt").empty());
        CHECK(rig.count(A::HardResync) == 1u);
        CHECK(rig.count(A::Skip) == 0u);
        CHECK(rig.count(A::Stall) == 0u);
        CHECK(rig.consistent);
    }
}

namespace recentreTest
{
    // The Ph4 gate's shaped rendered (b) client Ph4_b_F1 client1 (P5-34, gate R3), its first 600 authority lines (10 s) as
    // its log has them: SimClock's samples (the raw RTT each change of the logged smoothed RTT implies, 20 Hz, fed
    // before the line's authority tick; two samples before 5 of the lines), the authority tick of each line and
    // the advances the client ran before the next. Replayed through P3-11's clock and estimator, these tables reproduce
    // all 600 logged targets, all 883 advances (1 HardResync, 21 Stalls) and the logged decision byte for byte.
    struct LoggedLine
    {
        unsigned int samples;
        unsigned int authorityDelta;   // the first line's is its authority tick
        unsigned int advances;
    };
    const LoggedLine kPh4bF1C1Join[] = {
       {1u, 1877u, 1u}, {0u, 1u, 26u}, {1u, 8u, 7u}, {0u, 2u, 1u}, {1u, 9u, 10u},
    };
    const double kPh4bF1C1RttMs[] = {
        68.052, 59.079, 44.232, 68.475, 71.453, 77.701, 71.522, 79.282, 96.104, 68.026, 95.323, 53.587, 80.485, 47.615, 91.659,
        67.760, 87.967, 81.200, 87.208, 67.617, 76.827, 85.068, 76.536, 80.470, 55.600, 80.482, 82.435, 73.231, 90.407, 63.399,
        76.832, 79.179, 77.377, 81.526, 90.267, 83.560, 82.318, 83.941, 61.867, 68.447, 86.607, 75.640, 72.601, 81.455, 69.693,
        75.376, 82.139, 91.339, 83.253, 77.409, 73.266, 56.870, 71.302, 95.620, 82.449, 66.685, 80.455, 50.121, 73.963, 72.205,
        70.994, 75.367, 66.832, 53.808, 71.382, 80.860, 82.142, 65.267, 83.007, 77.082, 85.972, 68.617, 80.096, 69.407, 69.829,
        83.133, 51.847, 78.781, 56.770, 50.241, 56.239, 92.911, 74.083, 52.032, 76.036, 62.090, 56.622, 74.763, 84.178, 82.080,
        63.898, 74.832, 72.683, 50.397, 77.749, 88.345, 65.391, 75.936, 86.342, 59.035, 73.185, 69.214, 63.460, 89.188, 81.395,
        61.106, 75.030, 57.224, 79.191, 55.644, 87.357, 57.438, 77.913, 57.182, 81.209, 64.206, 84.731, 76.577, 85.145, 87.984,
        86.051, 76.505, 58.230, 96.587, 62.718, 63.774, 91.541, 79.465, 84.369, 77.071, 81.315, 88.280, 75.401, 66.257, 83.437,
        75.574, 84.181, 73.919, 71.201, 60.005, 86.005, 75.420, 72.589, 85.091, 55.166, 84.939, 79.135, 86.962, 69.492, 46.374,
        61.251, 55.302, 67.513, 54.939, 81.505, 86.490, 64.805, 69.395, 75.312, 68.347, 78.984, 74.954, 73.151, 68.941, 85.302,
        52.923, 88.063, 74.418, 57.463, 50.106, 72.821, 78.527, 87.553, 68.583, 75.118, 65.112, 75.053, 75.945, 57.136, 85.759,
        65.768, 78.698, 83.620, 55.880, 80.146, 60.292, 54.647, 79.909, 84.215, 91.529, 65.419, 90.412, 78.916, 71.445, 86.181,
        74.583, 59.513, 81.814, 74.638, 77.818, 84.349, 82.049, 90.598, 84.295, 78.412, 77.329, 69.148, 76.729, 80.571, 80.447,
        88.842, 77.650, 82.429, 57.613, 80.089, 89.324, 72.110, 83.509, 75.490, 54.224, 89.123, 75.002, 68.964, 60.324, 80.373,
        73.097, 82.527, 75.726, 76.641, 84.471, 55.129, 66.106, 59.192, 71.100, 76.638, 80.630, 72.389, 77.689, 83.361, 96.628,
        76.489, 91.798, 83.042, 80.255, 76.991, 72.103, 65.598, 64.123, 84.528, 85.594, 74.434, 74.221, 82.575, 68.478, 71.922,
        83.128, 75.402, 78.652, 77.556, 92.886, 66.860, 74.400, 67.458, 87.171, 67.342, 54.238, 72.541, 76.326, 53.316, 82.634,
        85.952, 65.798, 63.369, 89.644, 58.932,
    };
    const char kPh4bF1C1Lines[] =
        "fanemabnnamaaajjabdAbemkaedjabmaejbmbmbjembnamaajkaembjijaannanajdbjamabnnanajennekdjejejambdjbjeanmabejemkdbm"
        "abjemkaamjbfqmbjenniEdbnanjaenambjdkdbjjdfnmkaemkanandbanjmabdkjdkdbjaakdjajaembmfIbmdjabmbjndbmakejejaannaamj"
        "bemkdaajkaaajnaejejaajejejenajaajmbeanjembajajaajembjhkajajeajembjendnkembnembrDaabmaakmbjejaqabmoejhjakdjabae"
        "jajaembmbnjaemkdaakajmfjadbjajdakjadbjmbajaeEdfnmbjejmbaajamakaejdbjaamkeanmfjajdabajndaajbmbjembmbdabnmaajabm"
        "jbarjadjabnnajkgbjaajejaajernajemakajaembnajejaambnjdaknaaambnajaajenamabjdkenrnejajdjfmbmbnajdajbmajbajadkdab"
        "najaajeajjaenmbdjajaoaajaamkaaamaajajbjdbjnaa"
        ;
    // One character per line after the join prefix: 'a' + 9 * samples + 3 * (authority ticks moved - 1) + (advances - 1)
    // for 0 or 1 samples, 'A' + 3 * (ticks moved - 1) + (advances - 1) for 2.
    static_assert(sizeof(kPh4bF1C1Lines) == 596u, "one character per authority line after the five-line join prefix, plus the terminator");
    static_assert(sizeof(kPh4bF1C1RttMs) / sizeof(double) == 275u, "275 SimClock samples in the first 600 authority lines");

    // Returns the samples fed.
    size_t ph4bF1C1Replay(JoinRig& rig)
    {
        size_t sample = 0u;
        rig.authority = 0u;
        const auto line = [&](unsigned int samples, unsigned int authorityDelta, unsigned int advances)
        {
            for (unsigned int i = 1u; i < samples; ++i)
                rig.est.updateRTT(kPh4bF1C1RttMs[sample++] / 1000.0);
            rig.authority += authorityDelta;
            if (samples > 0u)
                rig.frame(kPh4bF1C1RttMs[sample++] / 1000.0, rig.authority, advances);
            else
                rig.afterSample(rig.authority, advances);
        };
        for (const LoggedLine& join : kPh4bF1C1Join)
            line(join.samples, join.authorityDelta, join.advances);
        for (size_t i = 0u; i + 1u < sizeof(kPh4bF1C1Lines); ++i)
        {
            const char c = kPh4bF1C1Lines[i];
            const bool two = c >= 'A' && c <= 'Z';
            int code = two ? c - 'A' : c - 'a';
            const unsigned int samples = two ? 2u : static_cast<unsigned int>(code / 9);
            code %= 9;
            line(samples, static_cast<unsigned int>(code / 3 + 1), static_cast<unsigned int>(code % 3 + 1));
        }
        return sample;
    }
}

TEST_CASE("PCTM.ClientPredictionClock.Recentre.TheWindowFloorsItsSamplesSoACatchUpBatchCannotFillIt",
          "[PCTM][ClientPredictionClock][Recentre][ClockSource]")
{
    using A = ClientPredictionClock::AdvanceResult;

    // og-input-latency P3-12 (Ph4 gate R3): the join's hard resync lands in a rendered start-up frame whose catch-up
    // batch runs 26 more advances in 16 ms, all with a reading, so P3-11's 30-advance window filled 0.14 s after the
    // resync, on 1 sample, with the client's backlog and the authority ticks still arriving in bursts. The decision
    // `fromLead=19 toLead=5 owed=14 target=1891 rttMs=66.706 samples=1 cause=join` paid its 14 Stalls, but its 56
    // advances outran the authority's 52 ticks by 4 while it did, so the lead fell 10, and with the offset up 2 the client
    // sat at +3, which the dead band (drift within 3, a lead error of -2 to +4) holds: p50 +2, mean +2.307 over lines
    // 300-600 (the log's session: +2, +2.323). With the window also waiting for kRecentreMinSamples (8) samples, it
    // decides 0.68 s after the resync, on target.
    recentreTest::JoinRig rig(TimeConfig{}.recentreSettleSamples);
    REQUIRE(recentreTest::ph4bF1C1Replay(rig) == 275u);
    REQUIRE(rig.leadErrors.size() == 600u);

    const int    p50  = rig.p50LeadMinusTarget(300u, 600u);
    const double mean = rig.meanLeadMinusTarget(300u, 600u);
    INFO("lead - target over lines 300-600: p50 " << p50 << " mean " << mean);
    CHECK(p50 == 0);
    CHECK(std::abs(mean) <= 0.5);
    CHECK(rig.results.size() == 883u);
    CHECK(rig.count(A::HardResync) == 1u);
    CHECK(rig.count(A::Skip) == 0u);
    CHECK(rig.count(A::Stall) == 23u);
    CHECK(rig.consistent);
    const std::vector<std::string> lines = rig.recentreLines();
    REQUIRE(lines.size() == 1u);
    CHECK(lines[0] == "[Warning][Clock.Recentre] fromLead=14 toLead=6 owed=8 target=1925 rttMs=73.472 samples=8 cause=join");
}

#endif // WITH_LOW_LEVEL_TESTS
