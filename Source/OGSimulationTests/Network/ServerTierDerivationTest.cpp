// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <type_traits>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
// Sibling-relative — the test tree is not on an include root under either UBT
// or the standalone CMake build (see ConnectionTierTableTest.cpp).
#include "StandaloneTestHandle.h"

// ---------------------------------------------------------------------------
// SERVER-SIDE AUTHORITATIVE TIER DERIVATION (task 10 of
// og-netcode-v2-arch-latency — D5.1 server half, Option A).
//
// WHAT THIS SUITE COVERS, AND WHAT IT DELIBERATELY DOES NOT.
//
// Under Option A the server is the sole producer of the connection tier: the UE
// adapter reads the RAW FNetPing RoundTrip value and forwards it to
// ServerReceptionCoordinator::noteRttSample (T20/T21), which feeds a
// ConnectionTierTable and, on a tier change, drives the send through the
// ConnectionTierSink (T23) — USimmableUpdateComponent, whose implementation writes
// a COND_OwnerOnly replicated uint8.
//
// The FNetPing read and the replication itself are ENGINE surface: this suite
// links no UE and cannot exercise either. They are covered by the manual PIE
// smoke test. What IS engine-free — and is what this suite pins — is the
// contract sitting between them:
//
//   1. SINGLE SMOOTHING. The server feeds raw per-sample readings straight in;
//      the table's own rttSmoothingAlpha EMA is the only filter. Both EMAs
//      converge to the same steady state, so the cost of double-smoothing is
//      LAG: this suite pins that an externally pre-smoothed stream reaches a
//      given tier strictly LATER than the raw one. Re-applying the
//      NetworkTimeEstimator EMA upstream would therefore be a behavior change,
//      not a no-op — it is the exact asymmetry Option A exists to remove, so it
//      gets a failing test and not just a comment.
//   2. PUBLISH NARROWING. The published value is a uint8. This pins that every
//      value lookupTierIndex can return survives the narrowing intact.
//   3. PUBLISH-ON-CHANGE EQUIVALENCE. The core publish path (T23:
//      ServerReceptionCoordinator::noteRttSample) skips the send when the tier is
//      unchanged for that owner (so an unchanged tier does not dirty the
//      replicated property every input RPC). This pins that the resulting
//      sequence of DISTINCT published values is identical to publishing on
//      every sample — i.e. the bandwidth optimization drops no transition.
//
// Assertions read TimeConfig values rather than literals wherever a default is
// the thing under test, matching the sibling suites and keeping the
// configurability lint clean.
// ---------------------------------------------------------------------------

using TestTierTable = ConnectionTierTable<FStandaloneTestHandle>;

namespace
{
    // Mirrors the core publish rule now in
    // ServerReceptionCoordinator::noteRttSample (T23) — skip negatives, skip
    // unchanged, narrow to uint8 — so the rule can be exercised on a bare
    // ConnectionTierTable without linking UE. Kept deliberately tiny and adjacent
    // to the assertions that depend on it — if that core rule grows a fourth
    // condition, this stops being a faithful mirror and the reviewer should be
    // told rather than this quietly drifting. (The coordinator's own suite,
    // ServerReceptionCoordinatorTest.cpp, exercises the real rule end-to-end via a
    // mock sink; this mirror stays focused on the tier-derivation sequence.)
    struct PublishRecorder
    {
        std::uint8_t current = 0;
        bool everPublished = false;
        std::vector<std::uint8_t> published;

        void publish(std::int32_t tierIndex)
        {
            if (tierIndex < 0)
                return;                       // "no usable reading yet"

            const auto next = static_cast<std::uint8_t>(tierIndex);
            if (everPublished && next == current)
                return;                       // unchanged — do not dirty

            current = next;
            everPublished = true;
            published.push_back(next);
        }
    };

    // Drives `sampleCount` samples of a constant RTT, one per tick, exactly as
    // the server does on the input-RPC receive path.
    void driveConstantRtt(TestTierTable& table,
                          const FStandaloneTestHandle& handle,
                          double rttMs,
                          std::int32_t sampleCount,
                          std::int32_t startTick = 0)
    {
        for (std::int32_t i = 0; i < sampleCount; ++i)
        {
            table.onRttSample(handle, startTick + i, rttMs);
        }
    }
}

TEST_CASE("Server tier derivation escalates on a sustained raw RTT rise", "[Network][ServerTierDerivation]")
{
    const TimeConfig cfg;
    TestTierTable table(cfg);
    const FStandaloneTestHandle conn(1, /*alive=*/true);

    // A connection well inside the lowest band sits at tier 0.
    driveConstantRtt(table, conn, cfg.rttTierBoundariesMs[0] / 2.0, cfg.tierMinDwellTicks * 2);
    REQUIRE(table.lookupTierIndex(conn) == 0);
    REQUIRE(table.lookupInputDelayTicks(conn) == cfg.rttTierInputDelays[0]);

    // Push it clearly past the first boundary + hysteresis and hold it there
    // long enough to clear the dwell gate. The table promotes at most one tier
    // per sample, so a long hold is what a multi-tier climb needs.
    driveConstantRtt(table, conn,
                     cfg.rttTierBoundariesMs[0] + cfg.tierHysteresisMs * 4.0,
                     cfg.tierMinDwellTicks * 6,
                     cfg.tierMinDwellTicks * 2);

    REQUIRE(table.lookupTierIndex(conn) >= 1);
    REQUIRE(table.lookupInputDelayTicks(conn) == cfg.rttTierInputDelays[table.lookupTierIndex(conn)]);
}

TEST_CASE("Pre-smoothing the RTT stream delays tier escalation", "[Network][ServerTierDerivation]")
{
    // THE SINGLE-SMOOTHING GUARANTEE, stated as a behavioral difference.
    //
    // Two tables see the same underlying RTT step. `rawTable` gets the samples
    // as the engine reports them, which is what the server actually does.
    // `preSmoothedTable` gets the same signal first passed through a second
    // EMA, standing in for the NetworkTimeEstimator smoothing that must NOT be
    // applied on this path.
    //
    // Both EMAs converge to the same value eventually, so a snapshot taken late
    // enough would show no difference — the cost of double-smoothing is LAG,
    // not a different steady state. This therefore measures the sample index at
    // which each path first reaches the top tier. If double-smoothing were
    // harmless those indices would be equal and this case would be vacuous; it
    // asserts the pre-smoothed path is strictly slower.
    //
    // That lag is precisely the C1 failure mode: while the two ends disagree
    // about the tier, the client predicts its input being consumed at a
    // different tick than the server consumes it.
    const TimeConfig cfg;
    TestTierTable rawTable(cfg);
    TestTierTable preSmoothedTable(cfg);

    const FStandaloneTestHandle conn(7, /*alive=*/true);

    const double lowRtt  = cfg.rttTierBoundariesMs[0] / 2.0;
    const double highRtt = cfg.rttTierBoundariesMs[2] + cfg.tierHysteresisMs * 4.0;

    // Settle both at the low value identically.
    driveConstantRtt(rawTable, conn, lowRtt, cfg.tierMinDwellTicks * 2);
    driveConstantRtt(preSmoothedTable, conn, lowRtt, cfg.tierMinDwellTicks * 2);
    REQUIRE(rawTable.lookupTierIndex(conn) == preSmoothedTable.lookupTierIndex(conn));

    // Step to the high value and record when each first reaches the top tier.
    const std::int32_t startTick   = cfg.tierMinDwellTicks * 2;
    const std::int32_t sampleCount = cfg.tierMinDwellTicks * 8;   // ample headroom for both

    std::int32_t rawReachedAt = -1;
    std::int32_t preSmoothedReachedAt = -1;
    double external = lowRtt;   // the second, redundant EMA

    for (std::int32_t i = 0; i < sampleCount; ++i)
    {
        rawTable.onRttSample(conn, startTick + i, highRtt);
        if (rawReachedAt < 0 && rawTable.lookupTierIndex(conn) == TestTierTable::kMaxTierIndex)
            rawReachedAt = i;

        external += cfg.rttSmoothingAlpha * (highRtt - external);
        preSmoothedTable.onRttSample(conn, startTick + i, external);
        if (preSmoothedReachedAt < 0 && preSmoothedTable.lookupTierIndex(conn) == TestTierTable::kMaxTierIndex)
            preSmoothedReachedAt = i;
    }

    // Both must actually get there, or the comparison below is meaningless.
    REQUIRE(rawReachedAt >= 0);
    REQUIRE(preSmoothedReachedAt >= 0);

    // The raw path tracks the real signal; the double-smoothed path lags. An
    // implementation that re-smoothed upstream would fail here.
    REQUIRE(rawReachedAt < preSmoothedReachedAt);
}

TEST_CASE("Every derivable tier index survives the uint8 publish narrowing", "[Network][ServerTierDerivation]")
{
    // The replicated property is a uint8. lookupTierIndex returns int32_t. This
    // pins that the narrowing at the publish site is lossless across the whole
    // range the table can produce, so the client never reads a tier the server
    // did not derive.
    const TimeConfig cfg;

    for (std::int32_t tier = 0; tier <= TestTierTable::kMaxTierIndex; ++tier)
    {
        const auto narrowed = static_cast<std::uint8_t>(tier);
        REQUIRE(static_cast<std::int32_t>(narrowed) == tier);
    }

    // And the range itself is what the config declares, so widening the tier
    // arrays without revisiting the wire width fails here rather than silently
    // truncating on the wire.
    REQUIRE(TestTierTable::kTierCount ==
            std::extent_v<decltype(TimeConfig::rttTierBoundariesMs)>);
    REQUIRE(TestTierTable::kMaxTierIndex <= 255);
}

TEST_CASE("Publish-on-change drops no tier transition", "[Network][ServerTierDerivation]")
{
    // The production setter skips writes when the tier is unchanged. That is a
    // bandwidth optimization on a property replicated per input RPC, so it must
    // be transition-preserving: the sequence of distinct values a client
    // observes has to match publishing unconditionally.
    const TimeConfig cfg;
    TestTierTable table(cfg);
    const FStandaloneTestHandle conn(3, /*alive=*/true);

    PublishRecorder onChange;
    std::vector<std::uint8_t> everySample;

    const auto feed = [&](double rttMs, std::int32_t count, std::int32_t startTick)
    {
        for (std::int32_t i = 0; i < count; ++i)
        {
            table.onRttSample(conn, startTick + i, rttMs);
            const std::int32_t tier = table.lookupTierIndex(conn);
            everySample.push_back(static_cast<std::uint8_t>(tier));
            onChange.publish(tier);
        }
    };

    std::int32_t tick = 0;
    feed(cfg.rttTierBoundariesMs[0] / 2.0, cfg.tierMinDwellTicks * 2, tick);
    tick += cfg.tierMinDwellTicks * 2;
    feed(cfg.rttTierBoundariesMs[1] + cfg.tierHysteresisMs * 4.0, cfg.tierMinDwellTicks * 6, tick);
    tick += cfg.tierMinDwellTicks * 6;
    feed(cfg.rttTierBoundariesMs[0] / 2.0, cfg.tierMinDwellTicks * 6, tick);

    // Collapse the every-sample sequence to its distinct runs; that is exactly
    // what publish-on-change should have produced.
    std::vector<std::uint8_t> expected;
    for (const std::uint8_t value : everySample)
    {
        if (expected.empty() || expected.back() != value)
            expected.push_back(value);
    }

    REQUIRE(onChange.published == expected);

    // Guard against a vacuous pass: the run must actually contain a transition,
    // otherwise both sequences would trivially be one element long.
    REQUIRE(expected.size() >= 3);
}

#endif // WITH_LOW_LEVEL_TESTS
