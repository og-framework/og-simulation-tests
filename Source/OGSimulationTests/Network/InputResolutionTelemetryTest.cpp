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
// So the count this item conserved was 1 case / 4 assertions total, held
// entirely by `NetSyncTelemetryTest.cpp`, 0 here — reported before/after in
// that item's impl notes rather than assumed. og-input-latency task 6 later
// added this file's first case, which drives the PT `emit*` helpers
// directly: the per-sender `[RelayProbe.Delta]` line.
//////////////////////////////////////////////////////////////////////////////

#include <string>
#include <vector>

namespace
{
    ScheduledRelayedReadReport formedRead(ScheduledRelayedReadOutcome outcome, std::int32_t deltaToNewest)
    {
        ScheduledRelayedReadReport report;
        report.outcome            = outcome;
        report.deltaToNewestValid = true;
        report.deltaToNewest      = deltaToNewest;
        if (outcome == ScheduledRelayedReadOutcome::Miss)
            report.missClass = ScheduledRelayedReadMissClass::AboveNewest;
        return report;
    }

    std::vector<std::string> linesTagged(const std::vector<std::string>& lines, const char* tag)
    {
        std::vector<std::string> tagged;
        for (const std::string& line : lines)
        {
            if (line.find(tag) != std::string::npos)
                tagged.push_back(line);
        }
        return tagged;
    }
}

TEST_CASE("InputResolutionTelemetry: the prediction delta line is one line per sender with sender= and lateShareX1000= appended",
          "[Network][RelayMargin]")
{
    InputResolutionTelemetry telemetry;
    std::vector<std::string> lines;
    telemetry.setLogger([&lines](const char* line) { lines.emplace_back(line); });

    telemetry.emitRelayReadWindowIfDue(500u);
    telemetry.emitPredictionInputRead(9u, 501u, true, formedRead(ScheduledRelayedReadOutcome::Hit, -3));
    telemetry.emitPredictionInputRead(9u, 502u, true, formedRead(ScheduledRelayedReadOutcome::Miss, 1));
    telemetry.emitPredictionInputRead(9u, 503u, true, formedRead(ScheduledRelayedReadOutcome::Hit, -3));
    telemetry.emitPredictionInputRead(9u, 504u, true, formedRead(ScheduledRelayedReadOutcome::Hit, -3));
    telemetry.emitPredictionInputRead(4u, 501u, true, formedRead(ScheduledRelayedReadOutcome::Hit, -2));
    telemetry.emitResimScheduledRead(9u, 499u, formedRead(ScheduledRelayedReadOutcome::Miss, 2));
    lines.clear();

    telemetry.emitRelayReadWindowIfDue(500u + kRelayReadProbeWindowTicks);

    const std::vector<std::string> delta = linesTagged(lines, "[RelayProbe.Delta]");
    REQUIRE(delta.size() == 3u);
    CHECK(delta[0] == "[Warning][RelayProbe.Delta] window=[500,620] src=Prediction n=1 p10=-2 p50=-2 p90=-2 "
                      "min=-2 max=-2 satLow=0 satHigh=0 sender=4 lateShareX1000=0");
    CHECK(delta[1] == "[Warning][RelayProbe.Delta] window=[500,620] src=Prediction n=4 p10=-3 p50=-3 p90=1 "
                      "min=-3 max=1 satLow=0 satHigh=0 sender=9 lateShareX1000=250");
    CHECK(delta[2] == "[Warning][RelayProbe.Delta] window=[500,620] src=Resim n=1 p10=2 p50=2 p90=2 "
                      "min=2 max=2 satLow=0 satHigh=0");
}

#endif // WITH_LOW_LEVEL_TESTS
