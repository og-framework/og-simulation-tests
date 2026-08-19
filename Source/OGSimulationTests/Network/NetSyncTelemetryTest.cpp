// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/NetSyncTelemetry.h"

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / task 79 (extract NetSyncTelemetry, architecture
// review B3): SimulationNetSync's four probes, its sixteen `emit*` helpers and
// the per-window flush bookkeeping moved onto this sibling object. Most of that
// move is covered by proof-of-no-regression elsewhere — the 12 pre-existing
// `getDiagnostics()` call sites in og-simulation-tests and the four
// og-brawler-tests wiring-proof tests are UNCHANGED and still green, which is
// the behaviour-preservation evidence for the relocation itself (see this
// task's impl notes).
//
// WHAT THIS FILE PINS INSTEAD: the ONE piece of behaviour that is new rather
// than relocated — the wire-version-mismatch log-once gate, which used to live
// on `RemoteInputCache` (per-store, so implicitly per-character) and now lives
// on `NetSyncTelemetry`, id-keyed. `RemoteInputCacheTest.cpp` carried four
// assertions against the old `store.shouldLogVersionMismatchOnce()`; they move
// here one-for-one against the new id-keyed method, plus the property the old
// per-store shape made unaskable: that two DIFFERENT ids never share a gate.
//////////////////////////////////////////////////////////////////////////////

TEST_CASE("NetSyncTelemetry: the version-mismatch log gate opens exactly once per id, "
          "and independently per id",
          "[Network][NetSyncTelemetry]")
{
    NetSyncTelemetry telemetry;

    // Mirrors the retired "RemoteInputCache: the mismatch log gate opens
    // exactly once per store" case one-for-one for id=1: open once, then
    // closed on every subsequent query for the SAME id.
    REQUIRE(telemetry.shouldLogVersionMismatchOnce(1u));
    REQUIRE_FALSE(telemetry.shouldLogVersionMismatchOnce(1u));
    REQUIRE_FALSE(telemetry.shouldLogVersionMismatchOnce(1u));

    // A DIFFERENT id (2) was never queried before — a property the old
    // per-store gate could never have been asked at all (a store IS a
    // per-character object, so "does id 2's gate depend on id 1's" was
    // structurally unanswerable). The id-keyed shape makes it a real question,
    // and it must open independently rather than inheriting id 1's closed state.
    REQUIRE(telemetry.shouldLogVersionMismatchOnce(2u));
}

#endif // WITH_LOW_LEVEL_TESTS
