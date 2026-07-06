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
}

// ---------------------------------------------------------------------------
// R-D3 ordering sanity — failsafe backstop must fire strictly later than the
// soft cap. Carries the extra [Determinism] tag so it can be filtered apart
// from the default-drift gate (it still carries [PCTM] so the [@og] alias and
// `oglltest simulation` default filter pick it up).
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.TimeConfig.HardResyncOrderingInvariant", "[PCTM][TimeConfig][Determinism]")
{
    TimeConfig tc;

    // `hardResyncThresholdTicks` is uint32_t; `rollbackWindowHardCap` is int32_t.
    // Cast the int32 side explicitly to uint32_t so the strict-inequality
    // comparison does not emit -Wsign-compare on the Clang/GCC standalone CMake
    // build path (T1 reviewer forward-looking requirement). Both defaults are
    // small positive values, so the cast is value-preserving.
    REQUIRE(tc.hardResyncThresholdTicks > static_cast<uint32_t>(tc.rollbackWindowHardCap));
}

#endif // WITH_LOW_LEVEL_TESTS
