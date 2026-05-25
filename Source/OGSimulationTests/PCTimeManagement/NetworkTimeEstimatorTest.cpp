// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Feed N identical RTT samples so the EMA converges close to that value.
static void feedStableRTT(NetworkTimeEstimator& est, double rttSeconds, int samples)
{
    for (int i = 0; i < samples; ++i)
        est.updateRTT(rttSeconds);
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

#endif // WITH_LOW_LEVEL_TESTS
