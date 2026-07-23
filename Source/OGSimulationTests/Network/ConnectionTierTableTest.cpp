// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
// Sibling-relative, matching the Determinism/ and NetConfigConceptTest
// convention — the test tree is not on an include root under either UBT or the
// standalone CMake build.
#include "StandaloneTestHandle.h"

// ---------------------------------------------------------------------------
// Coverage for ConnectionTierTable<Address> (task 4 of og-netcode-v2-arch-latency
// — D3.4 / proposal §1.2 + §8.1 / risks_and_plan R-A2), instantiated on the
// engine-free FStandaloneTestHandle from task 2.
//
// The table is delivered with NO production consumer this stage, so this suite
// is the only thing exercising the escalation policy. It is written against
// TimeConfig VALUES rather than literals wherever a default is the thing under
// test (`cfg.rttTierInputDelays[0]`, not `1`), so a future default change moves
// the test with the config instead of silently invalidating it — and so the
// configurability lint has nothing to flag here.
// ---------------------------------------------------------------------------

using TestTierTable = ConnectionTierTable<FStandaloneTestHandle>;

// Pins the relationship the ConnectionTierTable doc comment claims: the
// standalone concept in ConnectionTierTable.h and the Address requirements
// inside NetConfig are the same requirement set. If someone tightens one
// without the other, this fails to compile rather than drifting quietly.
static_assert(ConnectionAddress<FStandaloneTestHandle>,
    "FStandaloneTestHandle must satisfy ConnectionAddress");
static_assert(ConnectionAddress<StandaloneTestNetConfig::Address>,
    "Any NetConfig::Address must also satisfy ConnectionAddress");
static_assert(NetConfig<StandaloneTestNetConfig>,
    "guard: the config used to derive Address above is still a NetConfig");

namespace
{
    constexpr std::uint32_t kConnA = 1;
    constexpr std::uint32_t kConnB = 2;
    constexpr std::uint32_t kConnC = 3;

    FStandaloneTestHandle liveHandle(std::uint32_t id)
    {
        return FStandaloneTestHandle(id, true);
    }

    // The stand-in for the AC's "test-only setter". FStandaloneTestHandle is a
    // POD whose aliveBit participates in BOTH operator== and std::hash (a
    // deliberate task-2 decision: a live handle and the dead entry it replaced
    // must not conflate). A stored map KEY is const, so a test cannot flip
    // liveness on an entry already in the table — instead it samples a handle
    // that is already dead, which drives the exact same `!isAlive()` branch of
    // reapDeadHandles. See impl notes for why task 2's header was left untouched.
    FStandaloneTestHandle deadHandle(std::uint32_t id)
    {
        return FStandaloneTestHandle(id, false);
    }

    // Drives `table` up to `targetTier` using a config with the dwell gate
    // opened to one sample. Returns nothing; asserts the tier was reached, so a
    // policy regression surfaces in the arranging step rather than as a
    // confusing failure in the case's real assertion.
    void driveToTier(TestTierTable& table,
                     const TimeConfig& cfg,
                     const FStandaloneTestHandle& addr,
                     int32_t targetTier)
    {
        // Comfortably above the worst tier's promotion threshold, so each call
        // promotes exactly one step until the target is reached.
        const double farAboveTopBoundary =
            static_cast<double>(cfg.rttTierBoundariesMs[TestTierTable::kMaxTierIndex]) * 2.0;

        for (int32_t step = 0; step < targetTier; ++step)
        {
            // T11: every promotion here is a real transition, so the returned
            // TierSampleResult must describe it. Asserting it inside the shared
            // arrange helper is what makes the whole pre-existing suite exercise
            // the new return type — including under both lanZeroDelayOverride
            // settings, since callers of driveToTier use each.
            const TierSampleResult result = table.onRttSample(addr, step, farAboveTopBoundary);

            REQUIRE(result.newTierIndex == step + 1);
            REQUIRE(result.newTierIndex == table.lookupTierIndex(addr));
            REQUIRE(result.deltaDelayTicks == tierDelayDeltaTicks(step, step + 1, cfg));
        }
        REQUIRE(table.lookupTierIndex(addr) == targetTier);
    }
}

TEST_CASE("HysteresisRejectsBounceOnNoise", "[Network][ConnectionTierTable]")
{
    // Noise straddling the tier-0/1 boundary by +/-5ms with the default 10ms
    // hysteresis band. Neither direction may ever fire, on any of the 60
    // samples — including sample 60, where the dwell gate has opened and only
    // hysteresis is left holding the tier.
    const TimeConfig cfg;
    TestTierTable table(cfg);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    const double boundary = static_cast<double>(cfg.rttTierBoundariesMs[0]);

    for (int32_t tick = 0; tick < 60; ++tick)
    {
        const double sample = (tick % 2 == 0) ? boundary - 5.0 : boundary + 5.0;
        table.onRttSample(addr, tick, sample);
        REQUIRE(table.lookupTierIndex(addr) == 0);
    }

    // The smoothed value really did sit inside the band the whole time — this
    // rules out the case passing for the wrong reason (e.g. an EMA that never
    // moved off its seed and so was never near the boundary at all).
    const TestTierTable::TierState* state = table.findState(addr);
    REQUIRE(state != nullptr);
    REQUIRE(state->smoothedRttMs > boundary - 5.0 - 1e-9);
    REQUIRE(state->smoothedRttMs < boundary + 5.0 + 1e-9);
    REQUIRE(state->ticksInCurrentTier == 60);
}

TEST_CASE("UpwardTransitionOnSustainedSpike", "[Network][ConnectionTierTable]")
{
    // Sustained RTT 15ms above the tier-0 boundary — past the 10ms hysteresis
    // band, so only the dwell gate is holding the transition back.
    const TimeConfig cfg;
    TestTierTable table(cfg);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    const double spike = static_cast<double>(cfg.rttTierBoundariesMs[0]) + 15.0;

    // Samples 1..(dwell-1): the boundary is crossed on every one of them and the
    // transition is refused on every one of them.
    for (int32_t tick = 0; tick < cfg.tierMinDwellTicks - 1; ++tick)
    {
        table.onRttSample(addr, tick, spike);
        REQUIRE(table.lookupTierIndex(addr) == 0);
    }

    // The sample that opens the gate promotes.
    table.onRttSample(addr, cfg.tierMinDwellTicks - 1, spike);
    REQUIRE(table.lookupTierIndex(addr) == 1);
    REQUIRE(table.findState(addr)->ticksInCurrentTier == 0);

    // EXACTLY once: 15ms past the tier-0 boundary is nowhere near the tier-1
    // boundary, so continuing to hammer the same value must not walk the tier
    // up any further, even long after the dwell gate reopens.
    for (int32_t tick = 0; tick < cfg.tierMinDwellTicks * 2; ++tick)
    {
        table.onRttSample(addr, cfg.tierMinDwellTicks + tick, spike);
        REQUIRE(table.lookupTierIndex(addr) == 1);
    }
}

TEST_CASE("DwellGateBlocksTransitionEvenIfBoundaryCrossed", "[Network][ConnectionTierTable]")
{
    // One spike past the hysteresis threshold, immediately followed by a drop.
    // The dwell gate discards the candidate transition outright — there is no
    // pending-transition memory — so the tier never moves.
    const TimeConfig cfg;
    TestTierTable table(cfg);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    const double spike = static_cast<double>(cfg.rttTierBoundariesMs[0]) + 15.0;

    table.onRttSample(addr, 0, spike);
    REQUIRE(table.lookupTierIndex(addr) == 0);
    // The gate, not the hysteresis band, is what refused it: the smoothed value
    // is genuinely above the promotion threshold at this moment.
    REQUIRE(table.findState(addr)->smoothedRttMs
            > static_cast<double>(cfg.rttTierBoundariesMs[0] + cfg.tierHysteresisMs));
    REQUIRE(table.findState(addr)->ticksInCurrentTier < cfg.tierMinDwellTicks);

    // Immediate drop to a healthy RTT; run well past the dwell period to prove
    // the discarded candidate was not merely deferred until the gate opened.
    for (int32_t tick = 1; tick < cfg.tierMinDwellTicks * 2; ++tick)
    {
        table.onRttSample(addr, tick, 1.0);
        REQUIRE(table.lookupTierIndex(addr) == 0);
    }
}

TEST_CASE("LanOverrideZeroesTierZeroDelay", "[Network][ConnectionTierTable]")
{
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    SECTION("override off returns the configured tier-0 delay")
    {
        TimeConfig cfg;
        cfg.lanZeroDelayOverride = false;
        TestTierTable table(cfg);

        REQUIRE(table.lookupTierIndex(addr) == 0);
        REQUIRE(table.lookupInputDelayTicks(addr) == cfg.rttTierInputDelays[0]);
    }

    SECTION("override on collapses tier 0 to zero delay")
    {
        TimeConfig cfg;
        cfg.lanZeroDelayOverride = true;
        TestTierTable table(cfg);

        REQUIRE(table.lookupTierIndex(addr) == 0);
        REQUIRE(table.lookupInputDelayTicks(addr) == 0);
    }

    SECTION("override affects tier 0 ONLY — a bad link on a LAN keeps its delay")
    {
        TimeConfig cfg;
        cfg.lanZeroDelayOverride = true;
        cfg.tierMinDwellTicks = 1;
        TestTierTable table(cfg);

        driveToTier(table, cfg, addr, 1);
        REQUIRE(table.lookupInputDelayTicks(addr) == cfg.rttTierInputDelays[1]);
    }
}

TEST_CASE("MuteEchoOnlyOnTierThree", "[Network][ConnectionTierTable]")
{
    SECTION("with the policy enabled, only the worst tier mutes")
    {
        TimeConfig cfg;
        cfg.muteEchoOnDegradedTier = true;
        cfg.tierMinDwellTicks = 1;

        for (int32_t tier = 0; tier <= TestTierTable::kMaxTierIndex; ++tier)
        {
            TestTierTable table(cfg);
            const FStandaloneTestHandle addr = liveHandle(kConnA);
            driveToTier(table, cfg, addr, tier);

            REQUIRE(table.shouldMuteEcho(addr) == (tier == TestTierTable::kMaxTierIndex));
        }
    }

    SECTION("with the policy disabled, even the worst tier does not mute")
    {
        TimeConfig cfg;
        cfg.muteEchoOnDegradedTier = false;
        cfg.tierMinDwellTicks = 1;

        TestTierTable table(cfg);
        const FStandaloneTestHandle addr = liveHandle(kConnA);
        driveToTier(table, cfg, addr, TestTierTable::kMaxTierIndex);

        REQUIRE(table.shouldMuteEcho(addr) == false);
    }
}

TEST_CASE("DeadHandleReap", "[Network][ConnectionTierTable]")
{
    const TimeConfig cfg;
    TestTierTable table(cfg);

    const FStandaloneTestHandle live = liveHandle(kConnA);
    const FStandaloneTestHandle dead = deadHandle(kConnB);
    const FStandaloneTestHandle stale = liveHandle(kConnC);

    constexpr int32_t kDeadlineTicks = 60;
    constexpr int32_t kNowTick = 100;

    table.onRttSample(live, kNowTick, 10.0);
    table.onRttSample(dead, kNowTick, 10.0);
    table.onRttSample(stale, kNowTick - kDeadlineTicks - 1, 10.0);   // last heard from too long ago

    REQUIRE(table.entryCount() == 3);
    REQUIRE(table.hasEntry(live));
    REQUIRE(table.hasEntry(dead));
    REQUIRE(table.hasEntry(stale));

    table.reapDeadHandles(kNowTick, kDeadlineTicks);

    // Dead handle evicted by the liveness branch, silent handle by the deadline
    // branch, freshly-sampled live handle by neither.
    REQUIRE(table.hasEntry(dead) == false);
    REQUIRE(table.hasEntry(stale) == false);
    REQUIRE(table.hasEntry(live));
    REQUIRE(table.entryCount() == 1);

    // Reaping is not destructive to the survivor's state.
    REQUIRE(table.findState(live) != nullptr);
    REQUIRE(table.findState(live)->lastSampleTick == kNowTick);

    // Idempotent: a second reap at the same tick changes nothing.
    table.reapDeadHandles(kNowTick, kDeadlineTicks);
    REQUIRE(table.entryCount() == 1);
}

TEST_CASE("UnknownAddressReturnsTierZero", "[Network][ConnectionTierTable]")
{
    const TimeConfig cfg;
    TestTierTable table(cfg);
    const FStandaloneTestHandle neverSampled = liveHandle(kConnA);

    REQUIRE(table.hasEntry(neverSampled) == false);
    REQUIRE(table.lookupTierIndex(neverSampled) == 0);

    // The unknown-Address answer is consistent across the whole query surface —
    // an unknown handle behaves exactly like a known tier-0 one, so a caller
    // cannot accidentally read an out-of-range tier off a missing entry.
    REQUIRE(table.lookupInputDelayTicks(neverSampled) == cfg.rttTierInputDelays[0]);
    REQUIRE(table.lookupRollbackCeiling(neverSampled) == cfg.rttTierRollbackCeilings[0]);
    REQUIRE(table.shouldMuteEcho(neverSampled) == false);

    // Querying must not create an entry — otherwise a lookup-heavy caller would
    // grow the map with connections that were never sampled, and reaping would
    // be the only thing keeping it bounded.
    REQUIRE(table.entryCount() == 0);
}

// ---------------------------------------------------------------------------
// T11 — TierSampleResult, the per-sample transition report.
//
// The delta, NOT the index change, is the signal a transition consumer acts on:
// `deltaDelayTicks > 0` is exactly "this transition increased the player's input
// delay". These cases pin the contract stated in the struct's doc comment.
// ---------------------------------------------------------------------------

TEST_CASE("SampleWithoutTransitionReportsZeroDelta", "[Network][ConnectionTierTable]")
{
    const TimeConfig cfg;
    TestTierTable table(cfg);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    // A steady healthy RTT never crosses a boundary, so no sample transitions —
    // including the entry-creating first one, which must report the tier it
    // created (0) rather than a spurious 0 -> 0 "transition".
    for (int32_t tick = 0; tick < cfg.tierMinDwellTicks * 2; ++tick)
    {
        const TierSampleResult result = table.onRttSample(addr, tick, 1.0);

        REQUIRE(result.newTierIndex == 0);
        REQUIRE(result.deltaDelayTicks == 0);
        REQUIRE(result.newTierIndex == table.lookupTierIndex(addr));
    }
}

TEST_CASE("UpwardTransitionReportsPositiveDelta", "[Network][ConnectionTierTable]")
{
    TimeConfig cfg;
    cfg.tierMinDwellTicks = 1;
    TestTierTable table(cfg);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    const double farAboveTopBoundary =
        static_cast<double>(cfg.rttTierBoundariesMs[TestTierTable::kMaxTierIndex]) * 2.0;

    const TierSampleResult result = table.onRttSample(addr, 0, farAboveTopBoundary);

    REQUIRE(result.newTierIndex == 1);
    REQUIRE(result.deltaDelayTicks > 0);
    // Read from the config, not written as a literal: a degrading transition
    // costs exactly the difference between the two tiers' effective delays.
    REQUIRE(result.deltaDelayTicks
            == tierInputDelayTicks(1, cfg) - tierInputDelayTicks(0, cfg));
}

TEST_CASE("DownwardTransitionReportsNegativeDelta", "[Network][ConnectionTierTable]")
{
    TimeConfig cfg;
    cfg.tierMinDwellTicks = 1;
    TestTierTable table(cfg);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    // Reach tier 2 with an RTT that genuinely BELONGS to tier 2 — deliberately
    // NOT via driveToTier, which promotes using an RTT far above the WORST
    // tier's boundary. That helper leaves the EMA pinned near that spike, and
    // since the tier reads the SMOOTHED value the connection would simply
    // promote again on the next sample; there would be no downward transition
    // to observe. A tier-consistent EMA is the whole precondition of this case.
    const double tier2Rtt = 0.5 * (static_cast<double>(cfg.rttTierBoundariesMs[1])
                                 + static_cast<double>(cfg.rttTierBoundariesMs[2]));

    for (int32_t tick = 0; tick < 8; ++tick)
        table.onRttSample(addr, tick, tier2Rtt);

    REQUIRE(table.lookupTierIndex(addr) == 2);      // arrangement precondition

    // Now drop to a healthy RTT. One sample is NOT enough — the EMA has to
    // decay past the demotion band first — so feed healthy samples and capture
    // the FIRST transition they produce. That transition must be a DEMOTION:
    // the sign is the whole point, since it is what lets a consumer react to
    // degradation only.
    TierSampleResult firstTransition{};
    for (int32_t tick = 8; tick < 500; ++tick)
    {
        const TierSampleResult result = table.onRttSample(addr, tick, 1.0);
        if (result.deltaDelayTicks != 0)
        {
            firstTransition = result;
            break;
        }
    }

    REQUIRE(firstTransition.newTierIndex == 1);
    REQUIRE(firstTransition.deltaDelayTicks < 0);
    REQUIRE(firstTransition.deltaDelayTicks
            == tierInputDelayTicks(1, cfg) - tierInputDelayTicks(2, cfg));
}

TEST_CASE("DwellBlockedSampleReportsNoTransition", "[Network][ConnectionTierTable]")
{
    // A candidate transition the dwell gate discards must be invisible in the
    // result, exactly as it is invisible in lookupTierIndex. Reporting it would
    // make a consumer react to a transition that never happened.
    const TimeConfig cfg;
    TestTierTable table(cfg);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    const double spike = static_cast<double>(cfg.rttTierBoundariesMs[0]) + 15.0;

    for (int32_t tick = 0; tick < cfg.tierMinDwellTicks - 1; ++tick)
    {
        const TierSampleResult result = table.onRttSample(addr, tick, spike);

        REQUIRE(result.newTierIndex == 0);
        REQUIRE(result.deltaDelayTicks == 0);
    }

    // The sample that opens the gate is the one — and the only one — that
    // reports the transition.
    const TierSampleResult opened =
        table.onRttSample(addr, cfg.tierMinDwellTicks - 1, spike);

    REQUIRE(opened.newTierIndex == 1);
    REQUIRE(opened.deltaDelayTicks
            == tierInputDelayTicks(1, cfg) - tierInputDelayTicks(0, cfg));
}

TEST_CASE("DeltaHonoursLanZeroDelayOverride", "[Network][ConnectionTierTable]")
{
    // THE case that discriminates the correct delta from the plausible-but-wrong
    // one. With the LAN override on, tier 0's EFFECTIVE delay is 0, not
    // rttTierInputDelays[0] — so a delta computed as a bare array difference
    // (`rttTierInputDelays[1] - rttTierInputDelays[0]`) under-reports every
    // transition that leaves tier 0, and a client rolling back by that number
    // would roll back too little. Every other case in this suite runs with the
    // override OFF, where the two formulations agree and the bug is invisible.
    TimeConfig cfg;
    cfg.lanZeroDelayOverride = true;
    cfg.tierMinDwellTicks = 1;
    TestTierTable table(cfg);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    const double farAboveTopBoundary =
        static_cast<double>(cfg.rttTierBoundariesMs[TestTierTable::kMaxTierIndex]) * 2.0;

    const TierSampleResult result = table.onRttSample(addr, 0, farAboveTopBoundary);

    REQUIRE(result.newTierIndex == 1);
    REQUIRE(result.deltaDelayTicks == cfg.rttTierInputDelays[1] - 0);

    // Stated as an inequality too, so the case cannot silently stop
    // discriminating if the tier-0/tier-1 defaults ever become equal.
    REQUIRE(result.deltaDelayTicks
            != cfg.rttTierInputDelays[1] - cfg.rttTierInputDelays[0]);
}

TEST_CASE("TierDelayDeltaTicksIsAntisymmetric", "[Network][ConnectionTierTable]")
{
    // The shared helper the client half will use for the OnRep delta must agree
    // with the server's per-sample report, in both directions and under both
    // override settings.
    for (const bool lanOverride : { false, true })
    {
        TimeConfig cfg;
        cfg.lanZeroDelayOverride = lanOverride;

        for (int32_t from = 0; from <= TestTierTable::kMaxTierIndex; ++from)
        {
            for (int32_t to = 0; to <= TestTierTable::kMaxTierIndex; ++to)
            {
                REQUIRE(tierDelayDeltaTicks(from, to, cfg)
                        == -tierDelayDeltaTicks(to, from, cfg));
                REQUIRE(tierDelayDeltaTicks(from, from, cfg) == 0);
            }
        }
    }
}

#endif // WITH_LOW_LEVEL_TESTS
