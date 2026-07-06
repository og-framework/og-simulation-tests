// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"

// ---------------------------------------------------------------------------
// predOffsetFloorTicks regression tests (netcode Fix A, Task 23).
//
// Guards the minimum floor applied in NetworkTimeEstimator::getPredictionOffsetTicks().
// The floor keeps the client predictively AHEAD of authority even on LAN /
// near-zero RTT, closing the cooked-dedicated + late-connect dead-band lock
// documented in
// ../og-brawler-hit-resolution/netcode_finding_pred_offset_floor.md.
//
// Two scenarios:
//   * LANScenario — sub-millisecond RTT rounds the natural offset to ~1 tick;
//     the floor (default 4) engages and the reported offset is 4.
//   * WANScenario — at real broadband / cellular RTT the natural ceiled offset
//     already meets or exceeds the floor, so the floor is a no-op and the
//     natural value dominates.
//
// [PCTM] is in the [@og] whitelist so both cases run under the default filter.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// LAN loopback: 0.5 ms RTT. Natural rawOffset = (0.0005 + 0) * 60 = 0.03 →
// ceil = 1. The predOffsetFloorTicks floor (4) engages and dominates, so the
// reported offset is exactly 4 rather than the natural 1. This is the bug the
// finding describes: without the floor the client would target authorityTick+1
// and settle inside the softDrift dead band at/behind authority.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.PredOffsetFloor.LANScenario",
          "[PCTM][NetworkTimeEstimator][PredOffsetFloor]")
{
    TimeConfig cfg; // defaults: tickFrequency=60, jitterMultiplier=2, predOffsetFloorTicks=4
    NetworkTimeEstimator est(cfg, nullptr);

    est.updateRTT(0.0005); // 0.5 ms LAN loopback RTT (first sample → jitter 0)

    // Natural ceil would be 1; the floor lifts it to 4.
    REQUIRE(est.getPredictionOffsetTicks() == 4u);
}

// ---------------------------------------------------------------------------
// WAN / cellular: the floor is a no-op because the natural offset dominates.
//
// Part 1 — 50 ms broadband, single sample (jitter still 0): natural rawOffset =
// 0.050 * 60 = 3.0 → ceil = 3. The floor (4) supplies the reported value, so
// the offset is 4. (At steady state with realistic broadband jitter the natural
// value reaches 4 on its own; here the assertion of 4 holds either way.)
//
// Part 2 — smooth toward 150 ms cellular RTT with a handful of samples. The
// step up from 50 ms → 150 ms drives the jitter EMA up, so during the smoothing
// transient rawOffset = (smoothedRTT + 2*smoothedJitter) * 60 climbs well past
// the floor: the natural ceil dominates at >= 12 ticks. This proves the floor
// does not clamp real WAN latencies.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.NetworkTimeEstimator.PredOffsetFloor.WANScenario",
          "[PCTM][NetworkTimeEstimator][PredOffsetFloor]")
{
    TimeConfig cfg; // defaults
    NetworkTimeEstimator est(cfg, nullptr);

    // Part 1 — 50 ms broadband, first sample.
    est.updateRTT(0.050);
    REQUIRE(est.getPredictionOffsetTicks() == 4u);

    // Part 2 — smooth toward 150 ms cellular. A handful of samples keeps the
    // jitter EMA elevated from the 50→150 ms step, so the natural offset sits
    // well above the floor.
    for (int i = 0; i < 8; ++i)
        est.updateRTT(0.150);

    // Floor (4) is irrelevant here — the natural ceil dominates at >= 12.
    REQUIRE(est.getPredictionOffsetTicks() >= 12u);
}

#endif // WITH_LOW_LEVEL_TESTS
