// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

// ---------------------------------------------------------------------------
// D3.10 — TimeConfig ordering invariants.
//
// This file is the dedicated home for RELATIONAL invariants between TimeConfig
// fields, as distinct from TimeConfigDefaultsTest.cpp, which guards each field's
// default value in isolation. The distinction matters: a change that keeps every
// individual default "reasonable" can still invert an ordering and break the
// bounded-depth design.
//
// This is the R-D3 mitigation "config-validity gate at session start",
// implemented as a compile-anchored Catch2 assertion so the gate is enforced in
// CI rather than only at runtime on a machine someone happens to be running.
//
// RELOCATION NOTE: PCTM.TimeConfig.HardResyncOrderingInvariant previously lived
// in TimeConfigDefaultsTest.cpp. It was moved here verbatim (same test name,
// same tags, same assertion) when this file was created — no coverage was lost.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// R-D3 ordering sanity — failsafe backstop must fire strictly later than the
// soft cap. Carries the extra [Determinism] tag so it can be filtered apart
// from the default-drift gate (it still carries [PCTM] so the [@og] alias and
// `oglltest simulation` default filter pick it up).
//
// See the ADR block at the top of TimeConfig.h: the RollbackWindow soft cap is
// the primary circuit-breaker and HardResync is the absolute failsafe. If this
// inequality ever inverted, HardResync would fire BEFORE the soft cap had a
// chance to clamp — turning the rare, expensive "snap the clock and wipe the
// cache" path into the common one.
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
