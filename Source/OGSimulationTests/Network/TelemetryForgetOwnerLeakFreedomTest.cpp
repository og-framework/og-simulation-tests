// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/NetSyncTelemetry.h"
#include "OGSimulation/InputResolutionTelemetry.h"

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / item 91 part C (85 review finding 2, Backlog
// item 91 part C): a LEAK-FREEDOM test exercising BOTH telemetry siblings'
// `forgetOwner` TOGETHER, so a future id-keyed probe added to either sibling
// and never wired into that sibling's own `forgetOwner` fails a TEST instead
// of relying on a comment being read.
//
// WHY THIS EXISTS. Item 85 split the pre-item-85 telemetry object along its
// two-thread banner into `NetSyncTelemetry` (GT) and `InputResolutionTelemetry`
// (PT). Both are correctly wired today — `SimulationNetSync::unregisterSimulatable`
// step 4 calls both siblings' `forgetOwner` for the same id (see that method,
// and each sibling's own `forgetOwner` banner naming the other half). But
// nothing STRUCTURAL enforces that pairing: the split doubled the number of
// independent places a future author adding a THIRD id-keyed probe (to
// either sibling) must remember to wire into that sibling's own `forgetOwner`.
// This file is the "compiler/test enforced" half review 85 recommended in
// place of "must remember" — it does NOT guard against a probe added but
// never called from `unregisterSimulatable`'s own site (that remains
// convention), only against a probe added to either sibling class whose OWN
// `forgetOwner` body forgets to erase it.
//
// SCOPE, HONESTLY STATED: this pins the id-keyed state that exists TODAY —
// `RelayArrivalProbe::m_lastNewestCaptureTick` (GT, via `NetSyncTelemetry`),
// `NetSyncTelemetry::m_versionMismatchLogged` (GT), and `RelayReadProbe::
// m_fallbackRuns` (PT, via `InputResolutionTelemetry`) — the three id-keyed
// maps either sibling's `forgetOwner` is documented to erase.
// `CorrectionVerdictProbe`/`CorrectionLandingProbe` hold no id-keyed state at
// all, by design (see `NetSyncTelemetry::forgetOwner`'s own comment), so
// there is nothing of theirs to pin here. This does NOT retroactively catch a
// probe that already shipped unwired; it catches the NEXT one, which is the
// gap review 85 named.
//
// `RelayArrivalProbe::trackedOwnerCount()` (`Network/RelayReadProbe.h`) is a
// NEW introspection accessor added for this test, mirroring its PT sibling
// `RelayReadProbe::trackedOwnerCount()` (already used directly by
// `RelayReadProbeTest.cpp`) — before this task the GT probe had no
// map-size seam, only per-id or window-shaped reads. The version-mismatch
// latch has no such seam either; it is pinned INDIRECTLY below via
// `shouldLogVersionMismatchOnce`'s own re-opens-after-forget behaviour,
// exactly as `NetSyncTelemetryTest.cpp`'s existing case already does for the
// "opens once" half.
//////////////////////////////////////////////////////////////////////////////

TEST_CASE("Telemetry: forgetOwner drains every id-keyed map on BOTH siblings, "
          "for owners registered on both at once",
          "[Network][TelemetryForgetOwner]")
{
    NetSyncTelemetry netSyncTelemetry;
    InputResolutionTelemetry inputResolutionTelemetry;

    constexpr unsigned int kIdA = 101u;
    constexpr unsigned int kIdB = 102u;
    const unsigned int allIds[] = { kIdA, kIdB };

    // --- Populate every id-keyed map both siblings own, for BOTH ids. ---
    for (unsigned int id : allIds)
    {
        // GT: RelayArrivalProbe::m_lastNewestCaptureTick, via NetSyncTelemetry.
        RelayedInputIngestReport arrivalReport;
        arrivalReport.newestCaptureTickValid = true;
        arrivalReport.newestCaptureTick = 10u;
        arrivalReport.newCaptureTicksIngested = 1u;
        netSyncTelemetry.emitRelayArrival(id, arrivalReport);

        // GT: NetSyncTelemetry::m_versionMismatchLogged — open the gate.
        REQUIRE(netSyncTelemetry.shouldLogVersionMismatchOnce(id));

        // PT: RelayReadProbe::m_fallbackRuns, via InputResolutionTelemetry.
        // Outcome must NOT be NoProbe — NoProbe is excluded from the run map
        // entirely (RelayReadProbe.h noteRun's own early return), which would
        // leave nothing for forgetOwner to erase and make this test vacuous.
        ScheduledRelayedReadReport predictionReport;
        predictionReport.outcome = ScheduledRelayedReadOutcome::Hit;
        inputResolutionTelemetry.emitPredictionInputRead(id, 10u, /*hasStore=*/true, predictionReport);
    }

    REQUIRE(netSyncTelemetry.relayArrivalProbe().trackedOwnerCount() == 2u);
    REQUIRE(inputResolutionTelemetry.relayReadProbe().trackedOwnerCount() == 2u);
    // The version-mismatch gate is now closed for both ids (opened above).
    REQUIRE_FALSE(netSyncTelemetry.shouldLogVersionMismatchOnce(kIdA));
    REQUIRE_FALSE(netSyncTelemetry.shouldLogVersionMismatchOnce(kIdB));

    // --- Unregister kIdA only, on BOTH siblings — mirrors
    // SimulationNetSync::unregisterSimulatable's step 4 (both halves called
    // for the same id, from the same site). ---
    netSyncTelemetry.forgetOwner(kIdA);
    inputResolutionTelemetry.forgetOwner(kIdA);

    // kIdA's entries are gone from every map on both siblings...
    REQUIRE(netSyncTelemetry.relayArrivalProbe().trackedOwnerCount() == 1u);
    REQUIRE(inputResolutionTelemetry.relayReadProbe().trackedOwnerCount() == 1u);
    // ...and its version-mismatch gate re-opens, proving the id-keyed entry
    // was actually erased rather than merely left at a value that happens to
    // read the same (the indirect proof this latch's lack of a size seam
    // requires).
    REQUIRE(netSyncTelemetry.shouldLogVersionMismatchOnce(kIdA));

    // ...while kIdB, never forgotten, is UNTOUCHED on every map on both
    // siblings — the property that makes this a per-id erase proof rather
    // than an accidental full-clear.
    REQUIRE_FALSE(netSyncTelemetry.shouldLogVersionMismatchOnce(kIdB));

    // --- Now forget kIdB too — both siblings drain to empty. ---
    netSyncTelemetry.forgetOwner(kIdB);
    inputResolutionTelemetry.forgetOwner(kIdB);

    REQUIRE(netSyncTelemetry.relayArrivalProbe().trackedOwnerCount() == 0u);
    REQUIRE(inputResolutionTelemetry.relayReadProbe().trackedOwnerCount() == 0u);
    REQUIRE(netSyncTelemetry.shouldLogVersionMismatchOnce(kIdB));
}

#endif // WITH_LOW_LEVEL_TESTS
