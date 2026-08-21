// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/InputResolutionTelemetry.h"

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / item 85 (split NetSyncTelemetry along its own
// two-thread banner, design C.6): the PHYSICS-THREAD-ONLY half of task 79's
// sibling — the `RelayReadProbe` member, its `forgetOwner` half, and the ten
// PT `emit*` helpers (`emitLocalInputRead`, `emitRemoteQueueRead`,
// `emitPredictionInputRead`, the six `emitResim*`, `emitRelayReadWindowIfDue`
// plus its now-private `emitMissClassLine` / `emitDeltaLine`) — moves here
// verbatim from `NetSyncTelemetry.h`. `NetSyncTelemetryTest.cpp` keeps its one
// pre-existing case unmodified (it exercises `shouldLogVersionMismatchOnce`,
// which is GAME-THREAD-ONLY and did not move); this file is the PT sibling's
// half of that same split.
//
// CASE-COUNT CONSERVATION (this item's own acceptance criterion): the
// pre-split `NetSyncTelemetryTest.cpp` carried exactly ONE case / FOUR
// assertions, and every one of them exercised `shouldLogVersionMismatchOnce`
// — a GAME-THREAD-ONLY method that stayed on `NetSyncTelemetry` and was never
// touched by this split. There was no PT-specific case to carry across: the
// PT `emit*` helpers were never driven by a direct-instantiation unit test in
// this file (unlike the GT latch, which needed an id-keyed-independence test
// only a live object could show). Their behaviour-preservation evidence is,
// and always was, the og-brawler-tests wiring block (task 59's standard —
// `SimulationNetSyncTest.cpp`'s `relayReadProbe()` cases, which drive the
// REAL `collectInputAll` against the shipped branches) plus
// `RelayReadProbeTest.cpp`'s own unit coverage of the probe's arithmetic,
// including its `forgetOwner` erasure — neither of which this split touches.
// So the count this item conserves is 1 case / 4 assertions total, held
// entirely by `NetSyncTelemetryTest.cpp`, 0 here — reported before/after in
// this item's impl notes rather than assumed.
//////////////////////////////////////////////////////////////////////////////

#endif // WITH_LOW_LEVEL_TESTS
