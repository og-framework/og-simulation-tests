// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
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
// POSITIVE CONTROL, and the discrimination that matters most: a 0 ms reading is
// a VALID sample, not the engine's "no reading" sentinel. It is never counted or
// warned about as rejected, which is exactly why the sentinel is negative rather
// than zero. Since og-input-latency P5-28 a 0 does not SEED (an averaged ping
// reads 0 before it has measured anything), so before the first reading above 0
// it latches nothing; after a seed it is an ordinary low reading (T21 shipped
// this case as GenuineZeroIsAccepted, with a 0 seed).
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.RttGate.ZeroIsValidButDoesNotSeed",
          "[PCTM][NetworkTimeEstimator]")
{
    const TimeConfig cfg = t21Config();
    CapturingLogger  log;
    NetworkTimeEstimator est(cfg, log.fn());

    est.updateRTT(0.0);

    REQUIRE(est.hasFirstRTTSample() == false);
    REQUIRE(est.getSmoothedRTT() == 0.0);
    REQUIRE(est.getRejectedRTTSampleCount() == 0u);
    REQUIRE(est.getMeasuredRTTSampleCount() == 0u);
    REQUIRE(log.countContaining("REJECTED an invalid RTT sample") == 0);
    REQUIRE(est.getPredictionOffsetTicks() == cfg.predOffsetFloorTicks);

    // After a seed, a 0 moves both EMAs like any low reading.
    est.updateRTT(0.010);
    REQUIRE(est.hasFirstRTTSample() == true);
    REQUIRE(est.getSmoothedRTT() == 0.010);
    est.updateRTT(0.0);
    REQUIRE(est.getSmoothedRTT() == Catch::Approx(0.0085));
    REQUIRE(est.getSmoothedJitter() == Catch::Approx(0.0015));
    REQUIRE(est.getRejectedRTTSampleCount() == 0u);
    REQUIRE(est.getOutlierRejectedSampleCount() == 0u);
}

// ---------------------------------------------------------------------------
// og-input-latency P5-28 (review_P5-25 N5): the start-up sequence of one
// adapter's averaged ping (UE `FNetPing`, `RoundTrip=PlayerStateAvg`): the
// "no reading" sentinel until the first ack, then a 0 (its first average,
// taken over empty buckets) held until the next bucket closes, then the link's
// readings, here 75-89 ms as on Ph3's shaped (a) links. Before P5-28 the 0
// seeded, every real reading was then implausible against 4 * 0 + 30 ms, and
// the 30th forced a re-seed: 29 rejects and 1 escape.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.StartUp.TheFirstRealReadingAfterAZeroSeeds",
          "[PCTM][NetworkTimeEstimator]")
{
    const TimeConfig cfg = t21Config();
    const auto readingSeconds = [](int i) { return 0.001 * static_cast<double>(75 + i % 15); };

    for (const int zeros : {1, 46})
    {
        INFO(zeros << " zero reading(s)");
        NetworkTimeEstimator est(cfg, nullptr);
        feedStableRTT(est, -1.0, 3);
        feedStableRTT(est, 0.0, zeros);
        CHECK(est.hasFirstRTTSample() == false);
        CHECK(est.getPredictionOffsetTicks() == cfg.predOffsetFloorTicks);

        est.updateRTT(readingSeconds(0));
        CHECK(est.hasFirstRTTSample() == true);
        CHECK(est.getSmoothedRTT() == readingSeconds(0));   // seeded verbatim
        CHECK(est.getSmoothedJitter() == 0.0);
        CHECK(est.getPredictionOffsetTicks() == 5u);   // ceil(0.075 * 60) = ceil(4.5)

        for (int i = 1; i < 60; ++i)
            est.updateRTT(readingSeconds(i));
        CHECK(est.getOutlierRejectedSampleCount() == 0u);
        CHECK(est.getOutlierEscapeCount() == 0u);
        CHECK(est.getRejectedRTTSampleCount() == 3u);
        CHECK(est.getMeasuredRTTSampleCount() == 60u);
        CHECK(est.getSmoothedRTT() > 0.075);
        CHECK(est.getSmoothedRTT() < 0.089);
    }
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
// The warning is ONE-SHOT. The engine reading arrives with every clock tick the
// server sends (the timing relay's OnRep_Buffer at 100 Hz until og-input-latency
// P5-7), so an unthrottled warning would be the same per-tick
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
        // The clock arrival (the timing relay's OnRep until og-input-latency P5-7)
        // delivers authorityTick AND the ping reading in the same call, so this combination is reachable in production: a
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
        // offset value. (T21's positive control, ZeroIsValidButDoesNotSeed
        // above, pins the sentinel/0 discrimination: a 0 ms reading is never
        // REJECTED, and since P5-28 it does not seed either.)
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

TEST_CASE("PCTM.NetworkTimeEstimator.TwoThreads.TheReaderSeesOnlyValuesTheWriterWrote", "[Network]")
{
    constexpr int kSamples = 100000;
    NetworkTimeEstimator est(t21Config(), nullptr);

    const auto readRTT    = [&est] { return est.getSmoothedRTT(); };
    const auto readJitter = [&est] { return est.getSmoothedJitter(); };
    const auto readFlag   = [&est] { return est.hasFirstRTTSample(); };

    std::vector<double> writtenRTT{ 0.0 };
    std::vector<double> writtenJitter{ 0.0 };
    writtenRTT.reserve(kSamples + 1);
    writtenJitter.reserve(kSamples + 1);
    std::atomic<bool> writerDone{ false };

    std::thread writer([&] {
        for (int i = 0; i < kSamples; ++i)
        {
            est.updateRTT(0.010 + 0.000001 * static_cast<double>(i % 977) + 0.0000003 * static_cast<double>(i % 13));
            writtenRTT.push_back(est.getSmoothedRTT());
            writtenJitter.push_back(est.getSmoothedJitter());
        }
        writerDone.store(true, std::memory_order_release);
    });

    std::vector<double> seenRTT;
    std::vector<double> seenJitter;
    std::size_t reads = 0;
    bool flagFellBack = false;
    bool lastFlag = false;
    double lastRTT = -1.0;
    double lastJitter = -1.0;
    for (bool done = false; !done;)
    {
        done = writerDone.load(std::memory_order_acquire);
        const double rtt = readRTT();
        const double jitter = readJitter();
        const bool flag = readFlag();
        if (rtt != lastRTT)
        {
            seenRTT.push_back(rtt);
            lastRTT = rtt;
        }
        if (jitter != lastJitter)
        {
            seenJitter.push_back(jitter);
            lastJitter = jitter;
        }
        flagFellBack = flagFellBack || (lastFlag && !flag);
        lastFlag = flag;
        ++reads;
    }
    writer.join();

    const double finalRTT = writtenRTT.back();
    const double finalJitter = writtenJitter.back();
    std::sort(writtenRTT.begin(), writtenRTT.end());
    std::sort(writtenJitter.begin(), writtenJitter.end());
    std::size_t tornRTT = 0;
    for (const double value : seenRTT)
    {
        tornRTT += std::binary_search(writtenRTT.begin(), writtenRTT.end(), value) ? 0u : 1u;
    }
    std::size_t tornJitter = 0;
    for (const double value : seenJitter)
    {
        tornJitter += std::binary_search(writtenJitter.begin(), writtenJitter.end(), value) ? 0u : 1u;
    }

    INFO("reads=" << reads << " distinctRTT=" << seenRTT.size() << " distinctJitter=" << seenJitter.size());
    CHECK(est.getOutlierRejectedSampleCount() == 0u);
    CHECK(est.getRejectedRTTSampleCount() == 0u);
    CHECK(tornRTT == 0u);
    CHECK(tornJitter == 0u);
    CHECK_FALSE(flagFellBack);
    CHECK(lastFlag);
    CHECK(lastRTT == finalRTT);
    CHECK(lastJitter == finalJitter);
}

// ---------------------------------------------------------------------------
// og-input-latency P3-10: the clock RTT source (TimeConfig::clockRttSource),
// noteRttSourceChanged() and the SimClock shadow (P3 design §7.1).
// ---------------------------------------------------------------------------
namespace clockSourceTest
{
    using Source = TimeConfig::ClockRttSource;

    // One ping/pong as og-sim-net's simNetClock::sampleOf forms it: the client sends at
    // clientSendSeconds, the server's clock reads serverOffsetSeconds ahead of the client's,
    // the server turns it round in turnaroundSeconds.
    SimClockRttSample pingPong(double clientSendSeconds, double upSeconds, double downSeconds,
                               double serverOffsetSeconds = 0.75, double turnaroundSeconds = 0.001)
    {
        SimClockRttSample s;
        s.t0Seconds    = clientSendSeconds;
        s.t1Seconds    = clientSendSeconds + upSeconds + serverOffsetSeconds;
        s.t2Seconds    = s.t1Seconds + turnaroundSeconds;
        s.t3Seconds    = clientSendSeconds + upSeconds + turnaroundSeconds + downSeconds;
        s.rttSeconds   = (s.t3Seconds - s.t0Seconds) - (s.t2Seconds - s.t1Seconds);
        s.thetaSeconds = ((s.t1Seconds - s.t0Seconds) + (s.t2Seconds - s.t3Seconds)) / 2.0;
        return s;
    }

    // A symmetric ping/pong whose RTT field is exactly rttSeconds.
    SimClockRttSample ofRtt(double rttSeconds)
    {
        SimClockRttSample s = pingPong(10.0, rttSeconds / 2.0, rttSeconds / 2.0);
        s.rttSeconds = rttSeconds;
        return s;
    }

    TimeConfig simClockConfig()
    {
        TimeConfig cfg = t21Config();
        cfg.clockRttSource = Source::SimClock;
        return cfg;
    }
}

// The switch keeps the estimator: no re-seed, the first-sample flag and the counts stay,
// the first SimClock sample moves the RTT EMA by alpha of the step and leaves the jitter
// EMA alone (the step is the two sources' bias, not jitter). Seen failing with
// noteRttSourceChanged() emptied: the jitter EMA takes the 4 ms step.
TEST_CASE("PCTM.NetworkTimeEstimator.ClockSource.ASwitchNeverResetsTheEstimator",
          "[PCTM][NetworkTimeEstimator][ClockSource]")
{
    using clockSourceTest::Source;
    const TimeConfig cfg = clockSourceTest::simClockConfig();
    CapturingLogger log;
    NetworkTimeEstimator est(cfg, log.fn());

    for (int i = 0; i < 200; ++i)
        est.noteEngineRttSample(i % 2 == 0 ? 0.082 : 0.086);
    REQUIRE(est.getActiveRttSource() == Source::Engine);
    const double       rtt0      = est.getSmoothedRTT();
    const double       jitter0   = est.getSmoothedJitter();
    const unsigned int measured0 = est.getMeasuredRTTSampleCount();
    REQUIRE(jitter0 > 0.001);

    const SimClockRttSample first = clockSourceTest::ofRtt(0.080);
    est.noteSimClockSample(first);
    CHECK(est.getActiveRttSource() == Source::SimClock);
    CHECK(est.hasFirstRTTSample());
    CHECK(est.getSmoothedRTT() == Catch::Approx(cfg.rttSmoothingAlpha * first.rttSeconds
                                                + (1.0 - cfg.rttSmoothingAlpha) * rtt0).margin(1e-12));
    CHECK(est.getSmoothedJitter() == jitter0);
    CHECK(est.getOutlierEscapeCount() == 0u);
    CHECK(est.getOutlierRejectedSampleCount() == 0u);
    CHECK(est.getRejectedRTTSampleCount() == 0u);
    CHECK(est.getMeasuredRTTSampleCount() == measured0 + 1u);
    CHECK(log.countContaining("[ClockSource.Switch]") == 1);
    CHECK(log.countContaining("hasFirstRTTSample=1") == 1);

    // The next SimClock sample is an ordinary one: its distance moves the jitter EMA.
    const double rtt1 = est.getSmoothedRTT();
    est.noteSimClockSample(first);
    CHECK(est.getSmoothedJitter() == Catch::Approx(cfg.jitterSmoothingAlpha * std::abs(first.rttSeconds - rtt1)
                                                   + (1.0 - cfg.jitterSmoothingAlpha) * jitter0).margin(1e-12));

    // Engine readings no longer reach the estimate, sentinel included.
    const double rtt2 = est.getSmoothedRTT();
    est.noteEngineRttSample(0.5);
    est.noteEngineRttSample(-1.0);
    CHECK(est.getSmoothedRTT() == rtt2);
    CHECK(est.getRejectedRTTSampleCount() == 0u);

    // One switch per session.
    for (int i = 0; i < 50; ++i)
        est.noteSimClockSample(first);
    CHECK(log.countContaining("[ClockSource.Switch]") == 1);
    CHECK(est.getOutlierEscapeCount() == 0u);
}

// P3 §7.1: "A step down is accepted and slews in ~7 samples" - the EMA's time constant,
// 1 / rttSmoothingAlpha = 6.7: the 7th sample is the first to have covered 1 - 1/e of the step.
// The ~4 ms step is P5-27's measured engine-lane turnaround (an engine ping above SimClock).
TEST_CASE("PCTM.NetworkTimeEstimator.ClockSource.AStepDownSlewsInAboutSevenSamples",
          "[PCTM][NetworkTimeEstimator][ClockSource]")
{
    const TimeConfig cfg = clockSourceTest::simClockConfig();
    NetworkTimeEstimator est(cfg, nullptr);
    for (int i = 0; i < 300; ++i)
        est.noteEngineRttSample(0.084);
    const double from = est.getSmoothedRTT();
    REQUIRE(from == Catch::Approx(0.084).margin(1e-12));
    CHECK(est.getPredictionOffsetTicks() == 6u);   // ceil(5.04)

    const double to = 0.080;
    int    coveredAt = 0;
    double previous  = from;
    for (int k = 1; k <= 30; ++k)
    {
        est.noteSimClockSample(clockSourceTest::ofRtt(to));
        const double remaining = (est.getSmoothedRTT() - to) / (from - to);
        INFO("sample " << k << " remaining " << remaining);
        CHECK(remaining == Catch::Approx(std::pow(1.0 - cfg.rttSmoothingAlpha, k)).margin(1e-9));
        CHECK(est.getSmoothedRTT() < previous);
        previous = est.getSmoothedRTT();
        if (coveredAt == 0 && remaining <= std::exp(-1.0))
            coveredAt = k;
    }
    CHECK(coveredAt == 7);
    CHECK(est.getOutlierRejectedSampleCount() == 0u);
    CHECK(est.getOutlierEscapeCount() == 0u);
    CHECK(est.getPredictionOffsetTicks() == 5u);   // ceil(4.8): one tick, inside the dead band
}

// P3 §7.1: "A step up beyond the plausibility bound for rttOutlierConsecutiveLimit samples
// re-seeds, which breaks 'no reset'". The control shows each step is beyond the bound (the
// gate re-seeds on it); after the switch the re-based gate takes it as an ordinary slew.
// Seen failing with noteRttSourceChanged() emptied: 29 rejects and the re-seed.
TEST_CASE("PCTM.NetworkTimeEstimator.ClockSource.AStepUpBeyondTheBoundDoesNotReSeedAfterTheReBase",
          "[PCTM][NetworkTimeEstimator][ClockSource]")
{
    struct Step
    {
        double from;
        double to;
    };
    for (const Step step : {Step{0.020, 0.200}, Step{0.010, 0.400}})
    {
        INFO(step.from * 1000.0 << " ms -> " << step.to * 1000.0 << " ms");
        const TimeConfig cfg = clockSourceTest::simClockConfig();

        NetworkTimeEstimator control(t21Config(), nullptr);
        feedStableRTT(control, step.from, 300);
        REQUIRE(step.to > cfg.rttOutlierMultiplier * control.getSmoothedRTT() + cfg.rttOutlierMarginSeconds);
        feedStableRTT(control, step.to, 40);
        CHECK(control.getOutlierEscapeCount() == 1u);
        CHECK(control.getOutlierRejectedSampleCount() == 29u);

        NetworkTimeEstimator est(cfg, nullptr);
        for (int i = 0; i < 300; ++i)
            est.noteEngineRttSample(step.from);
        const double from         = est.getSmoothedRTT();
        const double jitterBefore = est.getSmoothedJitter();
        est.noteSimClockSample(clockSourceTest::ofRtt(step.to));
        CHECK(est.getSmoothedRTT() == Catch::Approx(cfg.rttSmoothingAlpha * step.to
                                                    + (1.0 - cfg.rttSmoothingAlpha) * from).margin(1e-12));
        CHECK(est.getSmoothedJitter() == jitterBefore);
        for (int i = 1; i < 200; ++i)
            est.noteSimClockSample(clockSourceTest::ofRtt(step.to));
        CHECK(est.getOutlierEscapeCount() == 0u);
        CHECK(est.getOutlierRejectedSampleCount() == 0u);
        CHECK(est.hasFirstRTTSample());
        CHECK(est.getSmoothedRTT() == Catch::Approx(step.to).margin(1e-6));

        // The re-base has lapsed: a frame-hitch reading is filtered again.
        const double settled = est.getSmoothedRTT();
        est.noteSimClockSample(clockSourceTest::ofRtt(2.0));
        CHECK(est.getOutlierRejectedSampleCount() == 1u);
        CHECK(est.getSmoothedRTT() == settled);
    }
}

// A first new-source reading above the cold-start ceiling earns no re-base (it would earn no
// seed): the ordinary gate rejects it, and the next reading is the one that skips the jitter update.
TEST_CASE("PCTM.NetworkTimeEstimator.ClockSource.AHitchOnTheFirstNewSourceReadingIsNotReBasedOn",
          "[PCTM][NetworkTimeEstimator][ClockSource]")
{
    const TimeConfig cfg = clockSourceTest::simClockConfig();
    NetworkTimeEstimator est(cfg, nullptr);
    for (int i = 0; i < 300; ++i)
        est.noteEngineRttSample(i % 2 == 0 ? 0.018 : 0.022);
    const double rtt0    = est.getSmoothedRTT();
    const double jitter0 = est.getSmoothedJitter();
    REQUIRE(jitter0 > 0.001);

    est.noteSimClockSample(clockSourceTest::ofRtt(0.6));
    CHECK(est.getOutlierRejectedSampleCount() == 1u);
    CHECK(est.getSmoothedRTT() == rtt0);

    est.noteSimClockSample(clockSourceTest::ofRtt(0.016));
    CHECK(est.getSmoothedJitter() == jitter0);
    est.noteSimClockSample(clockSourceTest::ofRtt(2.0));
    CHECK(est.getOutlierRejectedSampleCount() == 2u);
    CHECK(est.getOutlierEscapeCount() == 0u);
}

// Before the first seed there is nothing to keep: the first SimClock sample seeds verbatim.
// The engine's averaged ping reads 0 for about a second (P5-28), so this is the usual join.
TEST_CASE("PCTM.NetworkTimeEstimator.ClockSource.ASwitchBeforeTheFirstSeedSeedsNormally",
          "[PCTM][NetworkTimeEstimator][ClockSource]")
{
    const TimeConfig cfg = clockSourceTest::simClockConfig();
    CapturingLogger log;
    NetworkTimeEstimator est(cfg, log.fn());
    for (int i = 0; i < 30; ++i)
        est.noteEngineRttSample(0.0);
    REQUIRE_FALSE(est.hasFirstRTTSample());

    est.noteSimClockSample(clockSourceTest::ofRtt(0.080));
    CHECK(est.getActiveRttSource() == clockSourceTest::Source::SimClock);
    CHECK(est.getSmoothedRTT() == 0.080);
    CHECK(est.getSmoothedJitter() == 0.0);
    CHECK(log.countContaining("[ClockSource.Switch]") == 1);
    CHECK(log.countContaining("hasFirstRTTSample=0") == 1);

    est.noteSimClockSample(clockSourceTest::ofRtt(0.084));
    CHECK(est.getSmoothedJitter() == Catch::Approx(cfg.jitterSmoothingAlpha * (0.084 - 0.080)).margin(1e-12));
}

// At Engine (the shipped default until P3-10's flip, still the ini's `Engine`): a SimClock
// sample feeds the shadow and nothing else. Every estimator output equals a twin fed only
// updateRTT with the engine readings.
TEST_CASE("PCTM.NetworkTimeEstimator.ClockSource.AtEngineTheShadowNeverMovesTheEstimate",
          "[PCTM][NetworkTimeEstimator][ClockSource]")
{
    TimeConfig cfg = t21Config();
    cfg.clockRttSource = clockSourceTest::Source::Engine;
    NetworkTimeEstimator est(cfg, nullptr);
    NetworkTimeEstimator twin(cfg, nullptr);

    const double engine[] = {-1.0, 0.0, 0.0, 0.081, 0.083, 0.9, 0.082, 0.0, 0.086, 0.079};
    int index = 0;
    for (int i = 0; i < 400; ++i, index = (index + 1) % 10)
    {
        est.noteSimClockSample(clockSourceTest::ofRtt(0.040 + 0.0001 * (i % 7)));
        est.noteEngineRttSample(engine[index]);
        twin.updateRTT(engine[index]);
        est.recordAuthorityTick(1000u + static_cast<unsigned int>(i));
        twin.recordAuthorityTick(1000u + static_cast<unsigned int>(i));
        CHECK(est.getSmoothedRTT() == twin.getSmoothedRTT());
        CHECK(est.getSmoothedJitter() == twin.getSmoothedJitter());
        CHECK(est.getTargetPredictionTick() == twin.getTargetPredictionTick());
    }
    CHECK(est.getActiveRttSource() == clockSourceTest::Source::Engine);
    CHECK(est.hasFirstRTTSample() == twin.hasFirstRTTSample());
    CHECK(est.getRejectedRTTSampleCount() == twin.getRejectedRTTSampleCount());
    CHECK(est.getOutlierRejectedSampleCount() == twin.getOutlierRejectedSampleCount());
    CHECK(est.getOutlierEscapeCount() == twin.getOutlierEscapeCount());
    CHECK(est.getMeasuredRTTSampleCount() == twin.getMeasuredRTTSampleCount());
    CHECK(est.getClockSourceShadow().samples == 400u);
}

// The shadow: the SimClock RTT's EMA, the NTP offset of the minimum-RTT sample of the last 8,
// and RFC 3550's interarrival jitter per direction (J += (|D| - J) / 16, D the change of a
// direction's transit time). The values below are worked by hand from the lags.
TEST_CASE("PCTM.NetworkTimeEstimator.ClockSource.TheShadowReadsTheRttTheOffsetAndTheOwdVariation",
          "[PCTM][NetworkTimeEstimator][ClockSource]")
{
    TimeConfig cfg = t21Config();
    cfg.clockRttSource = clockSourceTest::Source::Engine; // the shadow alone; was the default until P3-10's flip
    NetworkTimeEstimator est(cfg, nullptr);
    CHECK(est.getClockSourceShadow().samples == 0u);

    // Up / down one-way lags (ms); the server's clock is 750 ms ahead, a 1 ms turnaround.
    const double up[]   = {20.0, 22.0, 20.0, 25.0};
    const double down[] = {30.0, 30.0, 31.0, 30.0};
    double send = 10.0;
    for (int i = 0; i < 4; ++i, send += 0.05)
        est.noteSimClockSample(clockSourceTest::pingPong(send, up[i] / 1000.0, down[i] / 1000.0));

    ClockSourceShadow shadow = est.getClockSourceShadow();
    CHECK(shadow.samples == 4u);
    CHECK(shadow.invalidSamples == 0u);
    // up: D = 2, 2, 5 ms: J = 0.125, 0.2421875, 0.53955078125 ms
    CHECK(shadow.owdVarUpSeconds == Catch::Approx(0.53955078125e-3).margin(1e-12));
    // down: D = 0, 1, 1 ms: J = 0, 0.0625, 0.12109375 ms
    CHECK(shadow.owdVarDownSeconds == Catch::Approx(0.12109375e-3).margin(1e-12));
    // theta = 750 + (up - down) / 2 ms; the minimum RTT is the first sample's 50 ms: 745 ms
    CHECK(shadow.offsetSeconds == Catch::Approx(0.745).margin(1e-9));
    // EMA of 50, 52, 51, 55 ms at alpha 0.15
    const double ema = 0.15 * 0.055 + 0.85 * (0.15 * 0.051 + 0.85 * (0.15 * 0.052 + 0.85 * 0.050));
    CHECK(shadow.simClockRttSeconds == Catch::Approx(ema).margin(1e-9));

    // 60 ms samples whose RTT field is exactly 60 ms, so that ties are exact.
    const auto sixty = [&send](double serverOffsetSeconds = 0.75) {
        SimClockRttSample s = clockSourceTest::pingPong(send, 0.030, 0.030, serverOffsetSeconds);
        s.rttSeconds = 0.060;
        send += 0.05;
        return s;
    };

    // Four fill the 8; the 50 ms one is still in them.
    for (int i = 0; i < 4; ++i)
        est.noteSimClockSample(sixty());
    CHECK(est.getClockSourceShadow().offsetSeconds == Catch::Approx(0.745).margin(1e-9));

    // The 9th pushes it out: the 51 ms sample (750 + (20 - 31) / 2 = 744.5 ms).
    est.noteSimClockSample(sixty());
    CHECK(est.getClockSourceShadow().offsetSeconds == Catch::Approx(0.7445).margin(1e-9));
    est.noteSimClockSample(sixty());
    CHECK(est.getClockSourceShadow().offsetSeconds == Catch::Approx(0.7445).margin(1e-9));
    // The 11th: the 55 ms sample (747.5 ms); the 12th leaves only 60 ms ones, a tie read on the newest.
    est.noteSimClockSample(sixty());
    CHECK(est.getClockSourceShadow().offsetSeconds == Catch::Approx(0.7475).margin(1e-9));
    est.noteSimClockSample(sixty(0.752));
    CHECK(est.getClockSourceShadow().offsetSeconds == Catch::Approx(0.752).margin(1e-9));

    // Invalid samples are counted and ignored, and never switch a SimClock config.
    SimClockRttSample nan = clockSourceTest::ofRtt(0.050);
    nan.rttSeconds = std::numeric_limits<double>::quiet_NaN();
    const SimClockRttSample zero = clockSourceTest::ofRtt(0.0);
    est.noteSimClockSample(nan);
    est.noteSimClockSample(zero);
    shadow = est.getClockSourceShadow();
    CHECK(shadow.samples == 12u);
    CHECK(shadow.invalidSamples == 2u);
    CHECK_FALSE(est.hasFirstRTTSample());

    const TimeConfig simCfg = clockSourceTest::simClockConfig();
    NetworkTimeEstimator sim(simCfg, nullptr);
    sim.noteSimClockSample(nan);
    sim.noteSimClockSample(zero);
    CHECK(sim.getActiveRttSource() == clockSourceTest::Source::Engine);
    CHECK_FALSE(sim.hasFirstRTTSample());
}

// og-input-latency P3-13: each RTT source's own jitter multiplier. With both EMAs at 1 the
// estimate is the last reading (1/16 s) and the jitter its distance from the one before
// (1/32 s), so every offset ceil((1/16 + m/32) * 60) = ceil(3.75 + 1.875 m) is exact.
TEST_CASE("PCTM.NetworkTimeEstimator.ClockSource.EachSourceHasItsOwnJitterMultiplier",
          "[PCTM][NetworkTimeEstimator][ClockSource]")
{
    struct Row { double multiplier; unsigned int offset; };
    const Row rows[] = { { 2.0, 8u }, { 1.5, 7u }, { 1.0, 6u }, { 0.5, 5u }, { 0.0, 4u } };
    for (const Row& row : rows)
    {
        INFO("multiplier " << row.multiplier);
        TimeConfig cfg = clockSourceTest::simClockConfig();
        cfg.rttSmoothingAlpha    = 1.0;
        cfg.jitterSmoothingAlpha = 1.0;
        cfg.predOffsetFloorTicks = 1u;

        cfg.jitterMultiplier         = 2.0;
        cfg.simClockJitterMultiplier = row.multiplier;
        NetworkTimeEstimator sim(cfg, nullptr);
        sim.noteSimClockSample(clockSourceTest::ofRtt(1.0 / 32.0));
        sim.noteSimClockSample(clockSourceTest::ofRtt(1.0 / 16.0));
        REQUIRE(sim.getActiveRttSource() == clockSourceTest::Source::SimClock);
        REQUIRE(sim.getSmoothedRTT() == 1.0 / 16.0);
        REQUIRE(sim.getSmoothedJitter() == 1.0 / 32.0);
        CHECK(sim.getPredictionOffsetTicks() == row.offset);
        CHECK(sim.getTargetPredictionTick() == row.offset);

        cfg.jitterMultiplier         = row.multiplier;
        cfg.simClockJitterMultiplier = 2.0;
        NetworkTimeEstimator engine(cfg, nullptr);
        engine.noteEngineRttSample(1.0 / 32.0);
        engine.noteEngineRttSample(1.0 / 16.0);
        REQUIRE(engine.getActiveRttSource() == clockSourceTest::Source::Engine);
        REQUIRE(engine.getSmoothedJitter() == 1.0 / 32.0);
        CHECK(engine.getPredictionOffsetTicks() == row.offset);
    }

    CHECK(clockJitter::clampJitterMultiplier(1.5) == 1.5);
    CHECK(clockJitter::clampJitterMultiplier(-0.5) == 0.0);
    CHECK(clockJitter::clampJitterMultiplier(std::numeric_limits<double>::quiet_NaN()) == 0.0);
    CHECK(clockJitter::clampJitterMultiplier(9.0) == 4.0);
}

#endif // WITH_LOW_LEVEL_TESTS
