// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

#include <type_traits>

// ---------------------------------------------------------------------------
// C.2 RTT-tier array ordering invariants (Stage 5 / D3.1 companion).
//
// The four tier arrays in TimeConfig are indexed by the SAME tier index 0..3,
// and the tier machinery (ConnectionTierTable, T4) assumes each table is ordered.
// These assertions lock in the assumptions that the lookup code is entitled to
// make, so a future retune that reorders a row fails here rather than producing
// a silently wrong effective input delay in a shipped build.
//
// Invariants asserted:
//   1. `rttTierBoundariesMs`      strictly increasing.
//   2. `rttTierInputDelays`       monotonically non-decreasing.
//   3. `rttTierRollbackCeilings`  monotonically non-decreasing.
//   4. last `rttTierRollbackCeilings` entry <= `rollbackWindowHardCap`.
//
// The tier arrays are fixed-size [4] by declaration; the loops below derive the
// bound from the array itself so that widening the tier count does not silently
// leave the tail entries unchecked.
// ---------------------------------------------------------------------------

TEST_CASE("PCTM.TimeConfig.TierBoundariesStrictlyIncreasing", "[PCTM][TimeConfig]")
{
    TimeConfig tc;

    // Strictly increasing: a flat or inverted boundary would make the bucket
    // lookup order-dependent, and a zero-width bucket could strand a connection
    // in a tier it can never leave.
    constexpr int boundaryCount =
        static_cast<int>(std::extent_v<decltype(TimeConfig::rttTierBoundariesMs)>);
    for (int i = 1; i < boundaryCount; ++i)
    {
        INFO("rttTierBoundariesMs index " << i);
        REQUIRE(tc.rttTierBoundariesMs[i] > tc.rttTierBoundariesMs[i - 1]);
    }
}

TEST_CASE("PCTM.TimeConfig.TierInputDelaysNonDecreasing", "[PCTM][TimeConfig]")
{
    TimeConfig tc;

    // Non-decreasing (not strict): two adjacent tiers are allowed to share an
    // input delay. What must never happen is a WORSE tier getting a SHORTER
    // delay, which would defeat the whole point of escalating.
    constexpr int delayCount =
        static_cast<int>(std::extent_v<decltype(TimeConfig::rttTierInputDelays)>);
    for (int i = 1; i < delayCount; ++i)
    {
        INFO("rttTierInputDelays index " << i);
        REQUIRE(tc.rttTierInputDelays[i] >= tc.rttTierInputDelays[i - 1]);
    }
}

TEST_CASE("PCTM.TimeConfig.TierRollbackCeilingsNonDecreasing", "[PCTM][TimeConfig]")
{
    TimeConfig tc;

    constexpr int ceilingCount =
        static_cast<int>(std::extent_v<decltype(TimeConfig::rttTierRollbackCeilings)>);
    for (int i = 1; i < ceilingCount; ++i)
    {
        INFO("rttTierRollbackCeilings index " << i);
        REQUIRE(tc.rttTierRollbackCeilings[i] >= tc.rttTierRollbackCeilings[i - 1]);
    }
}

TEST_CASE("PCTM.TimeConfig.TopTierCeilingWithinHardCap", "[PCTM][TimeConfig][Determinism]")
{
    TimeConfig tc;

    // The worst tier's SOFT-cap ceiling must not exceed the absolute failsafe.
    // If it did, tier-3 escalation would push the soft cap past the hard cap and
    // invert the "clamp before snap" ordering that the ADR block in TimeConfig.h
    // depends on. Both operands are int32_t, so no sign-compare cast is needed
    // here (unlike the uint32/int32 pairing in TimeConfigOrderingTest.cpp).
    constexpr int ceilingCount =
        static_cast<int>(std::extent_v<decltype(TimeConfig::rttTierRollbackCeilings)>);
    REQUIRE(tc.rttTierRollbackCeilings[ceilingCount - 1] <= tc.rollbackWindowHardCap);
}

#endif // WITH_LOW_LEVEL_TESTS
