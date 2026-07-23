// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

// ---------------------------------------------------------------------------
// TimeConfig default-drift gate (R-P1) + R-D3 ordering sanity test.
//
// These two tests guard the WHOLE TimeConfig struct against silent default
// drift and lock in the bounded-depth ordering invariant.
//
//   * Default-drift gate (risks_and_plan §6.2): asserts every TimeConfig field
//     default matches the synthesis-recommended initial value. If anyone edits
//     a default without updating the configurability surface (the R-P1 lint
//     BLACKLIST + this test), this test fails loudly. One REQUIRE per field.
//
//   * R-D3 ordering sanity (proposal §4.2): the HardResync failsafe backstop
//     must fire strictly LATER than the RollbackWindow soft cap, i.e.
//     `hardResyncThresholdTicks > rollbackWindowHardCap`. With T1's values
//     (21 > 20) this passes by a margin of 1.
//
// Stage hand-off note: `redundancyDepthTicks == 3` is the ratified 60 Hz target
// value (active since Stage 2 / T15). The historical Stage 1 interim value was 5
// (matched the 100 Hz runtime). Stage 2 task T15 flipped both the TimeConfig.h
// default AND this assertion 5 -> 3 in the same atomic change, when the runtime
// moved to the ratified 60 Hz target (proposal §11).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Default-drift gate — closes the default-drift surface for the whole struct.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.TimeConfig.DefaultsMatchSynthesisRecommendation", "[PCTM][TimeConfig]")
{
    TimeConfig tc;

    // --- Network estimation (NetworkTimeEstimator) -------------------------
    REQUIRE(tc.rttSmoothingAlpha == 0.15);
    REQUIRE(tc.jitterSmoothingAlpha == 0.15);
    REQUIRE(tc.jitterMultiplier == 2.0);
    REQUIRE(tc.predOffsetFloorTicks == 4);

    // --- Drift correction (ClientPredictionClock) --------------------------
    REQUIRE(tc.softDriftThresholdTicks == 3);
    REQUIRE(tc.gradualCorrectionRate == 4);
    REQUIRE(tc.minTicksBeforeDriftCheck == 60);

    // --- Tick frequency ----------------------------------------------------
    // Struct default is the long-term 60 Hz target. The runtime overrides this
    // at session construction (config.tickFrequency = 1.0 / asyncDeltaTime);
    // T15 changes that async-DeltaTime setup, not this struct default — so this
    // assertion stays == 60.0 across Stage 1 and Stage 2.
    REQUIRE(tc.tickFrequency == 60.0);

    // --- Bounded-depth prediction (RollbackWindow) -------------------------
    REQUIRE(tc.rollbackWindowTicks == 12);
    REQUIRE(tc.rollbackWindowHardCap == 20);

    // --- HardResync failsafe backstop (bumped 15 -> 21 in T1) --------------
    REQUIRE(tc.hardResyncThresholdTicks == 21);

    // --- Input redundancy (FInputRedundancyBundle) -------------------------
    // 60 Hz ratified target (active default since Stage 2 / T15). The historical
    // 100 Hz interim value was 5; T15 flipped both this assertion and the
    // TimeConfig.h default to 3 in the same atomic change.
    REQUIRE(tc.redundancyDepthTicks == 3);

    // --- Test harness mode selector ----------------------------------------
    REQUIRE(tc.harnessMode == TimeConfig::HarnessMode::Production);

    // --- C.2 tiered input delay (Stage 5) ----------------------------------
    REQUIRE(tc.forcedInputLatencyTicks == 2);

    REQUIRE(tc.rttTierBoundariesMs[0] == 30);
    REQUIRE(tc.rttTierBoundariesMs[1] == 80);
    REQUIRE(tc.rttTierBoundariesMs[2] == 150);
    REQUIRE(tc.rttTierBoundariesMs[3] == 999);

    REQUIRE(tc.rttTierInputDelays[0] == 1);
    REQUIRE(tc.rttTierInputDelays[1] == 2);
    REQUIRE(tc.rttTierInputDelays[2] == 3);
    REQUIRE(tc.rttTierInputDelays[3] == 4);

    REQUIRE(tc.rttTierRollbackCeilings[0] == 6);
    REQUIRE(tc.rttTierRollbackCeilings[1] == 9);
    REQUIRE(tc.rttTierRollbackCeilings[2] == 12);
    REQUIRE(tc.rttTierRollbackCeilings[3] == 20);

    REQUIRE(tc.tierHysteresisMs == 10);
    REQUIRE(tc.tierMinDwellTicks == 60);

    // `muteEchoOnDegradedTier` has NO consumer this initiative (optional task
    // T15 owns the behaviour). The default is asserted anyway so that if T15
    // flips it, the flip is a deliberate, visible change rather than a silent
    // drift.
    REQUIRE(tc.muteEchoOnDegradedTier == true);
    REQUIRE(tc.lanZeroDelayOverride == false);

    // --- Stage 4 observability (fields only, no consumer yet) --------------
    REQUIRE(tc.sn1BroadcastPolicy == TimeConfig::Sn1BroadcastPolicy::RttTiered);
    REQUIRE(tc.sn1IdleBroadcastIntervalTicks == 6);
    REQUIRE(tc.hashBroadcastPolicy == TimeConfig::HashBroadcastPolicy::EveryTick);
    REQUIRE(tc.hashBroadcastIntervalTicks == 1);
    REQUIRE(tc.hashMismatchTickThreshold == 5);
    REQUIRE(tc.hashMismatchReaction == TimeConfig::HashMismatchReaction::LogOnly);
    REQUIRE(tc.sparseSaveMode == false);
    REQUIRE(tc.recordSkipEvents == true);
    REQUIRE(tc.recordStallEvents == true);
    REQUIRE(tc.recordSubstitutionEvents == true);
    REQUIRE(tc.recordRedundancyHits == false);
    REQUIRE(tc.aggregateSiblingInputBundles == false);
    REQUIRE(tc.hashLogRingCapacity == 600);
}

// NOTE: the R-D3 HardResync ordering invariant that previously lived here has
// MOVED to the sibling TimeConfigOrderingTest.cpp (D3.10 completion), which is
// the dedicated home for TimeConfig ordering/relational invariants. The test
// name, tags and assertion are preserved verbatim there — this is a relocation,
// not a removal, and the file was split so the ordering invariants sit together
// rather than being buried in the default-drift gate.

#endif // WITH_LOW_LEVEL_TESTS
