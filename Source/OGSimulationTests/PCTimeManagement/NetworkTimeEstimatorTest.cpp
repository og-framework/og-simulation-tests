// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Feed N identical RTT samples so the EMA converges close to that value.
static void feedStableRTT(NetworkTimeEstimator& est, double rttSeconds, int samples)
{
    for (int i = 0; i < samples; ++i)
        est.updateRTT(rttSeconds);
}

// Capturing logger. The rejection warning is only observable through the
// injected LoggerFn (the core has no UE_LOG), so the one-shot property is only
// testable with a sink that records what it was handed.
struct CapturingLogger
{
    std::vector<std::string> lines;

    NetworkTimeEstimator::LoggerFn fn()
    {
        return [this](const char* msg) { lines.emplace_back(msg); };
    }

    // Count lines carrying a substring — used to assert the one-shot property
    // without pinning the exact wording of the message.
    int countContaining(const char* needle) const
    {
        int n = 0;
        for (const std::string& line : lines)
        {
            if (line.find(needle) != std::string::npos)
                ++n;
        }
        return n;
    }
};

// The config the T21 cases share. Values mirror the shipped defaults that make
// the defect reachable: a floor > 0 is what masked the silent zero.
static TimeConfig t21Config()
{
    TimeConfig cfg;
    cfg.tickFrequency        = 60.0;
    cfg.rttSmoothingAlpha    = 0.15;
    cfg.jitterSmoothingAlpha = 0.15;
    cfg.jitterMultiplier     = 2.0;
    return cfg;
}

// ---------------------------------------------------------------------------
// AC2: Stable RTT 0.1s at 60Hz → target offset == authorityTick + 6
//
// With stable RTT, jitter converges to 0.
// offset = ceil((0.1 + 2.0 * 0.0) * 60) = ceil(6.0) = 6
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.StableRTT_Offset6", "[PCTM][NetworkTimeEstimator]")
{
    TimeConfig cfg;
    cfg.tickFrequency        = 60.0;
    cfg.rttSmoothingAlpha    = 0.15;
    cfg.jitterSmoothingAlpha = 0.15;
    cfg.jitterMultiplier     = 2.0;

    NetworkTimeEstimator est(cfg, nullptr);
    est.recordAuthorityTick(100);

    // 200 stable samples is more than enough for EMA to converge
    feedStableRTT(est, 0.1, 200);

    const unsigned int offset = est.getPredictionOffsetTicks();
    // Jitter converges to 0 → offset = ceil(0.1 * 60) = 6
    REQUIRE(offset == 6u);

    const unsigned int target = est.getTargetPredictionTick();
    REQUIRE(target == 106u);
}

// ---------------------------------------------------------------------------
// AC3: Wildly varying RTT → getSmoothedRTT() changes gradually (not jump)
//
// We alternate between 0.01s and 0.5s. After each extreme sample the
// smoothed value must stay within (0, 0.5] and must NOT equal the raw sample.
//
// ⚠ T26b INTERACTION, RECORDED RATHER THAN PAPERED OVER. This case is UNCHANGED
// (pre-existing; the T26b work extended the suite, it did not rewrite it) but the
// path it exercises moved. A 0.5 s sample against a ~0.1 s estimate is a 5x
// excursion, which is outside the T26b plausibility bound
// (4 * smoothedRTT + 0.030 s), so those samples are now REJECTED rather than
// smoothed in. The alternation resets the consecutive-outlier run every other
// sample, so the escape hatch never fires and the estimate tracks the 0.01 s
// baseline instead of oscillating between the extremes.
//
// All three assertions still hold, and still mean what they say — "the smoothed
// value is damped, never the raw sample" is if anything demonstrated more
// strongly by a rejection than by a blend. The DELIBERATE choice not to widen
// rttOutlierMultiplier to 5.0 so this case would keep its old trajectory is
// recorded in the T26b impl notes: a config default must be sized against the
// network, not against a test's inputs. The direct pin for the new behaviour is
// `Outlier.SpuriousHitchSampleIsRejected` below.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.GradualSmoothing", "[PCTM][NetworkTimeEstimator]")
{
    TimeConfig cfg;
    cfg.rttSmoothingAlpha    = 0.15;
    cfg.jitterSmoothingAlpha = 0.15;
    cfg.tickFrequency        = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);

    // Seed with a mid-range value
    est.updateRTT(0.1);

    // Now hammer with extreme alternating values
    for (int i = 0; i < 20; ++i)
    {
        const double raw = (i % 2 == 0) ? 0.5 : 0.01;
        est.updateRTT(raw);

        const double smoothed = est.getSmoothedRTT();

        // Smoothed value must be strictly between the two extremes
        REQUIRE(smoothed > 0.01);
        REQUIRE(smoothed < 0.5);

        // Smoothed value must not equal the raw sample (it is damped)
        REQUIRE(smoothed != raw);
    }
}

// ---------------------------------------------------------------------------
// AC4: Monotonically increasing authority tick + constant RTT →
//      getTargetPredictionTick() is monotonically non-decreasing
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.MonotonicTargetTick", "[PCTM][NetworkTimeEstimator]")
{
    TimeConfig cfg;
    cfg.tickFrequency        = 60.0;
    cfg.rttSmoothingAlpha    = 0.15;
    cfg.jitterSmoothingAlpha = 0.15;
    cfg.jitterMultiplier     = 2.0;

    NetworkTimeEstimator est(cfg, nullptr);

    // Converge RTT to stable 0.1s first
    feedStableRTT(est, 0.1, 200);

    unsigned int prevTarget = 0;
    for (unsigned int tick = 0; tick <= 100; ++tick)
    {
        est.recordAuthorityTick(tick);
        const unsigned int target = est.getTargetPredictionTick();

        REQUIRE(target >= prevTarget);
        prevTarget = target;
    }
}

// ===========================================================================
// T21 — the RTT validity gate (og-netcode-v2-input-relay).
//
// These EXTEND the offset-math cases above rather than replacing them: the
// arithmetic they pin is unchanged, and StableRTT_Offset6 doubles as the
// positive control proving the gate does not reject ordinary traffic.
//
// WHAT THEY GUARD. UE's FPingValues::Current is -1.0 when the ping type is
// disabled or its accumulator is empty. That sentinel used to reach updateRTT
// unguarded and be latched as the first EMA sample.
// ===========================================================================

// ---------------------------------------------------------------------------
// The engine sentinel must not latch. Pre-fix this set m_smoothedRTT = -1.0 and
// m_hasFirstSample = true, and getPredictionOffsetTicks then reported
// predOffsetFloorTicks — a real-looking number produced by nothing.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.RttGate.EngineSentinelIsRejected",
          "[PCTM][NetworkTimeEstimator]")
{
    const TimeConfig cfg = t21Config();
    CapturingLogger  log;
    NetworkTimeEstimator est(cfg, log.fn());

    est.updateRTT(-1.0);        // exactly what GetPingValues(...).Current yields

    // Nothing latched: the estimator is honestly "no reading yet".
    REQUIRE(est.hasFirstRTTSample() == false);
    REQUIRE(est.getSmoothedRTT() == 0.0);
    REQUIRE(est.getSmoothedJitter() == 0.0);
    REQUIRE(est.getRejectedRTTSampleCount() == 1u);

    // And it is reported, not swallowed.
    REQUIRE(log.countContaining("REJECTED an invalid RTT sample") == 1);

    // T26a RULING (was `== 0u` as T21 shipped it): the no-sample path returns
    // the FLOOR. The floor is a structural invariant — "the client predicts
    // forward" — not an estimate, so it holds whether or not a reading exists.
    // The honesty requirement is carried by the warning asserted just above, not
    // by making the offset wrong. Full ruling at getPredictionOffsetTicks.
    REQUIRE(est.getPredictionOffsetTicks() == cfg.predOffsetFloorTicks);

    // A real sample afterwards is accepted normally and seeds the EMA clean —
    // it is NOT averaged against the rejected value.
    est.updateRTT(0.1);
    REQUIRE(est.hasFirstRTTSample() == true);
    REQUIRE(est.getSmoothedRTT() == Catch::Approx(0.1));
    REQUIRE(est.getSmoothedJitter() == 0.0);
}

// ---------------------------------------------------------------------------
// POSITIVE CONTROL, and the discrimination that matters most: a genuine 0 ms
// reading is a VALID sample and must be accepted. Rejecting it would be the
// obvious over-correction here, and it would be indistinguishable from the
// silent-zero bug this task exists to remove — which is exactly why the
// sentinel is negative rather than zero.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.RttGate.GenuineZeroIsAccepted",
          "[PCTM][NetworkTimeEstimator]")
{
    const TimeConfig cfg = t21Config();
    CapturingLogger  log;
    NetworkTimeEstimator est(cfg, log.fn());

    est.updateRTT(0.0);         // a real sub-millisecond loopback reading

    REQUIRE(est.hasFirstRTTSample() == true);
    REQUIRE(est.getSmoothedRTT() == 0.0);
    REQUIRE(est.getRejectedRTTSampleCount() == 0u);
    REQUIRE(log.countContaining("REJECTED an invalid RTT sample") == 0);

    // Latched a real (if tiny) reading, so the floor legitimately applies.
    REQUIRE(est.getPredictionOffsetTicks() == cfg.predOffsetFloorTicks);
}

// ---------------------------------------------------------------------------
// NaN and infinity are rejected too. `!(x >= 0.0)` catches NaN, which compares
// false against every ordering test including `x < 0.0`; isfinite catches +inf.
// Without both, a NaN would propagate into the EMA and permanently NaN the
// offset for the rest of the session.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.RttGate.NonFiniteIsRejected",
          "[PCTM][NetworkTimeEstimator]")
{
    const TimeConfig cfg = t21Config();
    CapturingLogger  log;
    NetworkTimeEstimator est(cfg, log.fn());

    est.updateRTT(std::numeric_limits<double>::quiet_NaN());
    est.updateRTT(std::numeric_limits<double>::infinity());
    est.updateRTT(-std::numeric_limits<double>::infinity());

    REQUIRE(est.hasFirstRTTSample() == false);
    REQUIRE(est.getRejectedRTTSampleCount() == 3u);
    REQUIRE(std::isfinite(est.getSmoothedRTT()));
    REQUIRE(std::isfinite(est.getSmoothedJitter()));

    // Still usable afterwards — rejection is not a latching error state.
    feedStableRTT(est, 0.1, 200);
    REQUIRE(est.hasFirstRTTSample() == true);
    REQUIRE(est.getPredictionOffsetTicks() == 6u);
}

// ---------------------------------------------------------------------------
// THE REGRESSION THAT MATTERS MOST FOR THIS INITIATIVE. A sentinel arriving
// mid-session must not disturb a converged estimator.
//
// Pre-fix it did, badly: -1.0 dragged m_smoothedRTT down AND — because jitter
// is the absolute delta against the previous smoothed value — injected a
// ~1.1 s phantom jitter delta, which jitterMultiplier = 2 doubled into the
// offset. That is an inflated-offset transient lasting tens of samples,
// produced by a bug rather than by the network, and indistinguishable in a PIE
// trace from the hitch sensitivity T21 was dispatched to investigate.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.RttGate.SentinelDoesNotDisturbConvergedState",
          "[PCTM][NetworkTimeEstimator]")
{
    const TimeConfig cfg = t21Config();
    CapturingLogger  log;
    NetworkTimeEstimator est(cfg, log.fn());

    feedStableRTT(est, 0.1, 200);

    const double       rttBefore    = est.getSmoothedRTT();
    const double       jitterBefore = est.getSmoothedJitter();
    const unsigned int offsetBefore = est.getPredictionOffsetTicks();
    REQUIRE(offsetBefore == 6u);

    // The ping source drops out for a while mid-session.
    for (int i = 0; i < 50; ++i)
        est.updateRTT(-1.0);

    // Bit-for-bit untouched — not "close to", untouched.
    REQUIRE(est.getSmoothedRTT() == rttBefore);
    REQUIRE(est.getSmoothedJitter() == jitterBefore);
    REQUIRE(est.getPredictionOffsetTicks() == offsetBefore);
    REQUIRE(est.getRejectedRTTSampleCount() == 50u);

    // No phantom jitter spike when real readings resume.
    est.updateRTT(0.1);
    REQUIRE(est.getSmoothedJitter() == Catch::Approx(jitterBefore).margin(1e-12));
    REQUIRE(est.getPredictionOffsetTicks() == 6u);
}

// ---------------------------------------------------------------------------
// The warning is ONE-SHOT. OnRep_Buffer runs at the timing relay's 100 Hz
// replication rate, so an unthrottled warning would be the same per-tick
// firehose class that produced the 28k-line log T17 had to clean up. The
// running count stays available so throttling does not cost the magnitude.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.RttGate.WarningIsOneShotButCountIsNot",
          "[PCTM][NetworkTimeEstimator]")
{
    const TimeConfig cfg = t21Config();
    CapturingLogger  log;
    NetworkTimeEstimator est(cfg, log.fn());

    for (int i = 0; i < 500; ++i)
        est.updateRTT(-1.0);

    REQUIRE(log.countContaining("REJECTED an invalid RTT sample") == 1);
    REQUIRE(est.getRejectedRTTSampleCount() == 500u);

    // Still one-shot after a valid sample has intervened: the flag is not
    // re-armed, so a flapping ping source cannot reopen the firehose.
    est.updateRTT(0.1);
    for (int i = 0; i < 500; ++i)
        est.updateRTT(-1.0);

    REQUIRE(log.countContaining("REJECTED an invalid RTT sample") == 1);
    REQUIRE(est.getRejectedRTTSampleCount() == 1000u);
}

// ---------------------------------------------------------------------------
// A null logger must stay safe — production constructs the estimator with one,
// but the pre-existing cases above pass nullptr and the gate runs for them too.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.RttGate.NullLoggerIsSafe",
          "[PCTM][NetworkTimeEstimator]")
{
    const TimeConfig cfg = t21Config();
    NetworkTimeEstimator est(cfg, nullptr);

    est.updateRTT(-1.0);
    est.updateRTT(std::numeric_limits<double>::quiet_NaN());

    REQUIRE(est.getRejectedRTTSampleCount() == 2u);
    REQUIRE(est.hasFirstRTTSample() == false);
}

// ===========================================================================
// T26a — the no-sample path returns the FLOOR, not 0.
//
// The architect's ruling on the behaviour change T21 escalated. The floor is a
// STRUCTURAL INVARIANT ("the client predicts forward"), not an estimate, so it
// holds whether or not an RTT reading exists. Returning 0 would put
// target = authorityTick + 0, which is exactly the dead-band-lock pathology the
// floor was introduced to prevent — see the ruling block at
// getPredictionOffsetTicks.
// ===========================================================================

TEST_CASE("PCTM.NetworkTimeEstimator.NoSamplePathReturnsFloor",
          "[PCTM][NetworkTimeEstimator][PredOffsetFloor]")
{
    const TimeConfig cfg = t21Config();

    SECTION("a fresh estimator that has never been fed anything")
    {
        NetworkTimeEstimator est(cfg, nullptr);

        REQUIRE(est.hasFirstRTTSample() == false);
        REQUIRE(est.getPredictionOffsetTicks() == cfg.predOffsetFloorTicks);
    }

    SECTION("THE HARMFUL CASE the ruling is about — authority is live, the ping "
            "source is not")
    {
        // The timing relay's OnRep delivers authorityTick AND reads the ping in
        // the same handler, so this combination is reachable in production: a
        // real, advancing authority tick with no usable RTT reading behind it.
        CapturingLogger  log;
        NetworkTimeEstimator est(cfg, log.fn());

        est.updateRTT(-1.0);                 // the engine's "no reading" sentinel
        est.recordAuthorityTick(1260);       // late-connect: server is well past warm-up

        REQUIRE(est.hasFirstRTTSample() == false);

        // The target must sit strictly AHEAD of authority. With a 0 offset it
        // would equal authorityTick, pastGuard would go true, and the clock would
        // steer the client to run AT the server tick — perpetually behind,
        // discarding every correction.
        REQUIRE(est.getPredictionOffsetTicks() == cfg.predOffsetFloorTicks);
        REQUIRE(est.getTargetPredictionTick() > est.getLastAuthorityTick());
        REQUIRE(est.getTargetPredictionTick() == 1260u + cfg.predOffsetFloorTicks);

        // HONESTY IS CARRIED BY THE LOG, NOT BY A WRONG OFFSET. This assertion is
        // the other half of the ruling: it is only defensible to report the floor
        // here BECAUSE the failure announces itself.
        REQUIRE(log.countContaining("REJECTED an invalid RTT sample") == 1);
    }

    SECTION("the offset alone cannot distinguish the two floored states — "
            "hasFirstRTTSample can, and that is the point")
    {
        NetworkTimeEstimator noReading(cfg, nullptr);
        NetworkTimeEstimator realReading(cfg, nullptr);

        realReading.updateRTT(0.0005);       // 0.5 ms loopback: genuine, below the floor

        // Same reported offset...
        REQUIRE(noReading.getPredictionOffsetTicks()
                == realReading.getPredictionOffsetTicks());

        // ...deliberately. The distinction that matters is exposed separately,
        // so a consumer that needs it is not forced to infer it from a magic
        // offset value. (T21's positive control, GenuineZeroIsAccepted above,
        // pins the same discrimination from the other side: a genuine 0 ms
        // reading is ACCEPTED, not treated as absent.)
        REQUIRE(noReading.hasFirstRTTSample() == false);
        REQUIRE(realReading.hasFirstRTTSample() == true);
    }
}

// ===========================================================================
// T26b — outlier RTT rejection.
//
// UE measures RTT from FApp::GetCurrentTime() (FRAME-START time), so a frame
// hitch delays ack processing and lands that delay directly in the sample: a
// ~1 s reading on a loopback LAN carrying 5-10 ms of emulated lag. The gate
// screens such samples against the current estimate; the escape hatch is what
// keeps it a filter rather than a lock.
// ===========================================================================

// The pre-T26b estimator, reproduced exactly: bounds set so wide that the gate
// can never fire. Used as the REFERENCE for both the "what this would have cost"
// contrast and the degenerate byte-identical proof.
static TimeConfig unfilteredConfig()
{
    TimeConfig cfg = t21Config();
    cfg.rttOutlierMultiplier              = 1.0e9;
    cfg.rttOutlierColdStartCeilingSeconds = 1.0e9;
    return cfg;
}

// A converged 10 ms LAN estimator: smoothedRTT is exactly 0.010 and jitter is
// exactly 0.0 (every delta after the seed is zero), so post-condition assertions
// can be exact rather than approximate.
static void converge10ms(NetworkTimeEstimator& est)
{
    feedStableRTT(est, 0.010, 200);
}

// ---------------------------------------------------------------------------
// THE MOTIVATING CASE. One spurious ~1 s sample against a stable ~10 ms estimate
// leaves the estimator bit-for-bit untouched — and the contrast estimator shows
// what believing it used to cost.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.Outlier.SpuriousHitchSampleIsRejected",
          "[PCTM][NetworkTimeEstimator][RttOutlier]")
{
    const TimeConfig cfg = t21Config();
    NetworkTimeEstimator est(cfg, nullptr);
    converge10ms(est);

    REQUIRE(est.getSmoothedRTT() == 0.010);
    REQUIRE(est.getSmoothedJitter() == 0.0);
    const unsigned int offsetBefore = est.getPredictionOffsetTicks();
    REQUIRE(offsetBefore == cfg.predOffsetFloorTicks); // ceil(0.6) = 1, floored to 4

    est.updateRTT(1.0);                  // the frame-hitch artifact

    // Bit-for-bit untouched — not "close to", untouched.
    REQUIRE(est.getSmoothedRTT() == 0.010);
    REQUIRE(est.getSmoothedJitter() == 0.0);
    REQUIRE(est.getPredictionOffsetTicks() == offsetBefore);

    // Counted as an OUTLIER, and kept strictly separate from the T21 validity
    // counter: "the engine has no reading" and "the game thread hitched" are
    // different failures and a PIE trace has to be able to tell them apart.
    REQUIRE(est.getOutlierRejectedSampleCount() == 1u);
    REQUIRE(est.getOutlierEscapeCount() == 0u);
    REQUIRE(est.getRejectedRTTSampleCount() == 0u);

    // No phantom jitter when normal readings resume — the artifact left nothing
    // behind for the next delta to be measured against.
    est.updateRTT(0.010);
    REQUIRE(est.getSmoothedJitter() == 0.0);
    REQUIRE(est.getPredictionOffsetTicks() == offsetBefore);

    // WHAT IT WOULD HAVE COST. Same trace through the pre-T26b estimator: the
    // 1 s sample drags the RTT EMA up AND injects ~0.99 s of jitter, which
    // jitterMultiplier = 2 doubles into the offset. Four ticks becomes tens, and
    // both EMAs need tens of samples to shed it — the ~8 s transient, from one
    // sample.
    // NOTE: the config MUST be a named local — NetworkTimeEstimator stores a
    // `const TimeConfig&`, so binding it to a temporary would dangle.
    const TimeConfig unfilteredCfg = unfilteredConfig();
    NetworkTimeEstimator unfiltered(unfilteredCfg, nullptr);
    converge10ms(unfiltered);
    unfiltered.updateRTT(1.0);
    REQUIRE(unfiltered.getOutlierRejectedSampleCount() == 0u);
    REQUIRE(unfiltered.getPredictionOffsetTicks() > 4u * offsetBefore);
}

// ---------------------------------------------------------------------------
// THE MANDATORY CASE — the one that distinguishes a filter from a bug.
//
// A GENUINE sustained step of the SAME magnitude as the artifact above must be
// followed. The discriminator is persistence, not size: a hitch is isolated, a
// route change is sustained. After rttOutlierConsecutiveLimit consecutive
// implausible samples the estimator re-seeds from the new level.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.Outlier.GenuineSustainedStepIsEventuallyAccepted",
          "[PCTM][NetworkTimeEstimator][RttOutlier]")
{
    const TimeConfig cfg = t21Config();
    NetworkTimeEstimator est(cfg, nullptr);
    converge10ms(est);

    const unsigned int limit = cfg.rttOutlierConsecutiveLimit;
    REQUIRE(limit > 1u);   // guards the case against a config that disables the filter

    // The link genuinely moves to ~1 s and STAYS there.
    for (unsigned int i = 0; i < limit - 1u; ++i)
        est.updateRTT(1.0);

    // Still holding the old estimate — the filter has not yet been convinced.
    REQUIRE(est.getSmoothedRTT() == 0.010);
    REQUIRE(est.getOutlierRejectedSampleCount() == limit - 1u);
    REQUIRE(est.getOutlierEscapeCount() == 0u);

    // The sample that trips the escape hatch.
    est.updateRTT(1.0);

    // RE-SEEDED, not blended: a step change makes the old estimate wrong rather
    // than stale, and re-seeding also zeroes the jitter EMA instead of recording
    // the step itself as ~1 s of phantom jitter.
    REQUIRE(est.getSmoothedRTT() == 1.0);
    REQUIRE(est.getSmoothedJitter() == 0.0);
    REQUIRE(est.getOutlierEscapeCount() == 1u);
    REQUIRE(est.getOutlierRejectedSampleCount() == limit - 1u); // the escape is not a reject
    REQUIRE(est.getPredictionOffsetTicks() == 60u);             // ceil(1.0 * 60)

    // And it STAYS followed: at the new level the bound has moved with it, so
    // subsequent samples are ordinary traffic and no further escape is needed.
    for (int i = 0; i < 50; ++i)
        est.updateRTT(1.0);

    REQUIRE(est.getSmoothedRTT() == 1.0);
    REQUIRE(est.getOutlierEscapeCount() == 1u);
    REQUIRE(est.getOutlierRejectedSampleCount() == limit - 1u);
    REQUIRE(est.getPredictionOffsetTicks() == 60u);
}

// ---------------------------------------------------------------------------
// COLD START — the subtle part, and the rule is pinned rather than left implicit.
//
// The first sample is latched VERBATIM, so the relative bound has nothing to
// compare against. RULE CHOSEN: an absolute ceiling on the seed. RULE REJECTED:
// accept-then-correct. The escape hatch is what keeps the ceiling from locking
// out a genuinely slow link.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.Outlier.ColdStartIsGatedByTheAbsoluteCeiling",
          "[PCTM][NetworkTimeEstimator][RttOutlier]")
{
    const TimeConfig cfg = t21Config();

    SECTION("an ordinary seed is accepted")
    {
        NetworkTimeEstimator est(cfg, nullptr);
        est.updateRTT(0.2);                  // slow but believable, under the ceiling

        REQUIRE(est.hasFirstRTTSample() == true);
        REQUIRE(est.getSmoothedRTT() == 0.2);
        REQUIRE(est.getOutlierRejectedSampleCount() == 0u);
    }

    SECTION("a hitch-inflated FIRST reading does not become the baseline")
    {
        NetworkTimeEstimator est(cfg, nullptr);
        est.updateRTT(0.8);                  // above rttOutlierColdStartCeilingSeconds

        REQUIRE(est.hasFirstRTTSample() == false);
        REQUIRE(est.getSmoothedRTT() == 0.0);
        REQUIRE(est.getOutlierRejectedSampleCount() == 1u);

        // T26a cross-check: while there is no reading, the offset reports the
        // structural floor. The ceiling and the ruling meet exactly here.
        REQUIRE(est.getPredictionOffsetTicks() == cfg.predOffsetFloorTicks);
    }

    SECTION("but a genuinely slow link is NOT locked out — it seeds via the escape hatch")
    {
        NetworkTimeEstimator est(cfg, nullptr);
        for (unsigned int i = 0; i < cfg.rttOutlierConsecutiveLimit; ++i)
            est.updateRTT(0.8);

        REQUIRE(est.hasFirstRTTSample() == true);
        REQUIRE(est.getSmoothedRTT() == 0.8);
        REQUIRE(est.getOutlierEscapeCount() == 1u);
    }
}

// ---------------------------------------------------------------------------
// ONE-SIDED BY DESIGN. Only samples that are too HIGH are screened. A hitch can
// only ever inflate a measurement — it delays the ack, it cannot deliver it
// early — and screening the low side would block RECOVERY after a genuine
// congestion episode.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.Outlier.LowSideIsNeverFiltered",
          "[PCTM][NetworkTimeEstimator][RttOutlier]")
{
    const TimeConfig cfg = t21Config();
    NetworkTimeEstimator est(cfg, nullptr);

    est.updateRTT(0.4);                      // congested link, seeded legitimately
    REQUIRE(est.getSmoothedRTT() == 0.4);

    // Congestion clears. Every one of these is 80x below the estimate and every
    // one must be believed immediately.
    for (int i = 0; i < 60; ++i)
        est.updateRTT(0.005);

    REQUIRE(est.getOutlierRejectedSampleCount() == 0u);
    REQUIRE(est.getOutlierEscapeCount() == 0u);
    REQUIRE(est.getSmoothedRTT() < 0.05);
}

// ---------------------------------------------------------------------------
// DEGENERATE PROOF. With no anomalous sample the gate is not merely harmless —
// it is inert. A shipping-default estimator and one whose bounds are set so wide
// the gate cannot fire are compared BIT-FOR-BIT at every step of a realistic
// trace.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.Outlier.DegenerateTraceIsByteIdentical",
          "[PCTM][NetworkTimeEstimator][RttOutlier]")
{
    NetworkTimeEstimator shipping(t21Config(), nullptr);

    const TimeConfig unfilteredCfg = unfilteredConfig();
    NetworkTimeEstimator unfiltered(unfilteredCfg, nullptr);

    // A deterministic 8-14 ms LAN wander — ordinary traffic, nothing the gate
    // has any business touching.
    for (int i = 0; i < 200; ++i)
    {
        const double raw = 0.008 + 0.001 * static_cast<double>(i % 7);

        shipping.updateRTT(raw);
        unfiltered.updateRTT(raw);

        REQUIRE(shipping.getSmoothedRTT() == unfiltered.getSmoothedRTT());
        REQUIRE(shipping.getSmoothedJitter() == unfiltered.getSmoothedJitter());
        REQUIRE(shipping.getPredictionOffsetTicks() == unfiltered.getPredictionOffsetTicks());
    }

    REQUIRE(shipping.getOutlierRejectedSampleCount() == 0u);
    REQUIRE(shipping.getOutlierEscapeCount() == 0u);
    REQUIRE(shipping.hasFirstRTTSample() == true);
}

// ---------------------------------------------------------------------------
// REJECTION COUNTER PER WINDOW, AND ITS LOG.
//
// A silent reject hides a genuine RTT step change exactly as well as it hides a
// hitch artifact; that ambiguity is this gate's main risk and the per-window
// line is what makes it recoverable. It must fire when there is something to
// report, stay quiet when there is not, and stay BOUNDED under a continuously
// misbehaving source — the T17 lesson about per-event logging still applies.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.Outlier.WindowSummaryIsReportedAndBounded",
          "[PCTM][NetworkTimeEstimator][RttOutlier]")
{
    TimeConfig cfg = t21Config();
    cfg.rttOutlierLogWindowSamples = 10;   // member access: the shipped default is 600

    CapturingLogger  log;
    NetworkTimeEstimator est(cfg, log.fn());

    // A clean window says nothing.
    for (int i = 0; i < 10; ++i)
        est.updateRTT(0.010);
    REQUIRE(log.countContaining("[RttSample.Outlier]") == 0);

    // A window containing a rejection reports it — once.
    est.updateRTT(1.0);
    for (int i = 0; i < 9; ++i)
        est.updateRTT(0.010);
    REQUIRE(log.countContaining("[RttSample.Outlier]") == 1);
    REQUIRE(est.getOutlierRejectedSampleCount() == 1u);

    // Now a continuously hitching source: 100 artifacts interleaved with normal
    // traffic. The alternation resets the consecutive run each time, so nothing
    // escapes — these really are 100 separate rejections.
    for (int i = 0; i < 100; ++i)
    {
        est.updateRTT(1.0);
        est.updateRTT(0.010);
    }

    REQUIRE(est.getOutlierRejectedSampleCount() == 101u);
    REQUIRE(est.getOutlierEscapeCount() == 0u);

    // 200 further samples = 20 further windows = 20 further lines. ONE PER
    // WINDOW, not one per rejection: the magnitude lives in the counter, the log
    // carries only the per-window summary.
    REQUIRE(log.countContaining("[RttSample.Outlier]") == 21);

    // And the estimate survived all of it untouched.
    REQUIRE(est.getSmoothedRTT() == 0.010);
    REQUIRE(est.getPredictionOffsetTicks() == cfg.predOffsetFloorTicks);
}

#endif // WITH_LOW_LEVEL_TESTS
