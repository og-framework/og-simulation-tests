// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ClientInputDelayLine.h"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/Network/RelayedInputStore.h"   // [T5/AM-3] kRelayedInputStoreCapacityTicks
#include "OGSimulation/Network/ReplicatedTierConsumer.h"
#include "OGSimulation/Network/ServerInputDelayQueue.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
// Sibling-relative includes, matching the convention in this directory — the
// test tree is not on an include root under either UBT or the standalone CMake
// build (see the same note in ServerInputDelayQueueTest.cpp).
#include "MockSimulatablesForNetworkTests.h"
#include "StandaloneTestHandle.h"

// ---------------------------------------------------------------------------
// THE RELAY DELAY FLOOR (og-netcode-v2-input-relay T11;
// RelayDelaySpectrumDesign.md §6, §10, §11 Q1/Q5).
//
// `TimeConfig::relayDelayFloorTicks` is a session-scoped MINIMUM on the effective
// Layer-1 input delay, applied as `max(floor, tier-or-fallback)` at every site
// that derives one. This suite pins the engine-free half of that — which is all
// of the arithmetic and every one of the derivation sites:
//
//   1. DEGENERATE PROOF. At the shipped default (0) every derivation answers
//      exactly what it answered before the floor existed. This is the case that
//      makes "ships off, byte-identical" a checked claim rather than a promise.
//   2. DOMINANCE. A floor above a tier's delay wins — including over
//      `lanZeroDelayOverride`, whose whole purpose is to answer 0.
//   3. THE NO-TIER ARM (review amendment A2b). The two ends must agree during the
//      window before a tier has replicated, and the ONLY way they can is if the
//      client's no-tier fallback and the server's no-entry fallback apply the
//      floor to the SAME base (`forcedInputLatencyTicks`) — not to tier 0.
//   4. THE A5 CLAMP, at both intake points and once more on read.
//   5. THE BENIGN DELTA COLLAPSE. A dominating floor makes tier transitions
//      report a zero delay delta, which is correct: felt delay did not change, so
//      there is no prediction-stall debt.
//
// WHAT IT DELIBERATELY DOES NOT COVER. The UE transport — the replicated uint8
// on ASimulationTimingRelay, its OnRep, the latch/replay, and the composition
// root's GConfig read — is engine surface this suite links none of (there is no
// engine-coupled LLT target; see docs/low-level-tests.md). Those are PIE-gated.
// What is testable here is the contract sitting UNDER that transport, plus the
// clamp function both intake points call.
//
// ASSERTIONS READ `cfg` RATHER THAN LITERALS wherever a default is the subject,
// so a future TimeConfig change moves the expectation with it and the R-P1
// configurability lint stays clean.
// ---------------------------------------------------------------------------

using TestQueue = ServerInputDelayQueue<FStandaloneTestHandle, MockSimA, MockSimB>;
using TestTierTable = ConnectionTierTable<FStandaloneTestHandle>;

namespace
{
    constexpr std::uint32_t kConnA = 1;

    FStandaloneTestHandle liveHandle(std::uint32_t id)
    {
        return FStandaloneTestHandle(id, true);
    }

    TestQueue::SlotKey slot0(const FStandaloneTestHandle& addr)
    {
        return TestQueue::SlotKey(addr, 0);
    }

    // The PRE-T11 expression for a tier's effective delay, written out by hand.
    // The degenerate proof compares the shipped code against THIS rather than
    // against itself, so a refactor that quietly changed the tier rule would fail
    // here instead of passing vacuously.
    int32_t preFloorTierInputDelayTicks(int32_t tierIndex, const TimeConfig& cfg)
    {
        const int32_t tier = clampConnectionTierIndex(tierIndex);
        if (tier == 0 && cfg.lanZeroDelayOverride)
        {
            return 0;
        }
        return cfg.rttTierInputDelays[tier];
    }

    // Drive `table` until `addr` sits in tier 0 with both R-A2 gates open.
    void seedAtTierZero(TestTierTable& table, const FStandaloneTestHandle& addr,
                        const TimeConfig& cfg)
    {
        for (int32_t i = 0; i <= cfg.tierMinDwellTicks; ++i)
        {
            table.onRttSample(addr, i, 1.0);    // sub-boundary RTT: stays in tier 0
        }
    }
}

// ---------------------------------------------------------------------------
// 1. DEGENERATE PROOF — floor 0 changes nothing, anywhere.
// ---------------------------------------------------------------------------

TEST_CASE("RelayDelayFloor: the shipped default is 0 and every derivation is unchanged",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;
    REQUIRE(cfg.relayDelayFloorTicks == 0);     // the whole feature ships OFF

    // Every tier, both override settings, against the hand-written pre-T11 rule.
    for (int pass = 0; pass < 2; ++pass)
    {
        cfg.lanZeroDelayOverride = (pass == 1);
        for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
        {
            REQUIRE(tierInputDelayTicks(tier, cfg) == preFloorTierInputDelayTicks(tier, cfg));
        }

        // Deltas too — they are derived from the same lookup, and the stall path
        // reads them.
        REQUIRE(tierDelayDeltaTicks(0, 1, cfg)
                == preFloorTierInputDelayTicks(1, cfg) - preFloorTierInputDelayTicks(0, cfg));
        REQUIRE(tierDelayDeltaTicks(3, 2, cfg)
                == preFloorTierInputDelayTicks(2, cfg) - preFloorTierInputDelayTicks(3, cfg));
    }

    cfg.lanZeroDelayOverride = false;

    // Both no-tier fallbacks answer the bare baseline, exactly as before.
    const ReplicatedTierConsumer consumer(cfg);
    REQUIRE_FALSE(consumer.hasReceivedTier());
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.forcedInputLatencyTicks);

    const TestQueue tierlessQueue(cfg);
    REQUIRE(tierlessQueue.effectiveDelay(liveHandle(kConnA)) == cfg.forcedInputLatencyTicks);

    // applyRelayDelayFloor itself is the identity on every plausible base.
    for (int32_t base = 0; base <= 10; ++base)
    {
        REQUIRE(applyRelayDelayFloor(base, cfg) == base);
    }
}

// ---------------------------------------------------------------------------
// 2. DOMINANCE — max semantics, including over the LAN override.
// ---------------------------------------------------------------------------

TEST_CASE("RelayDelayFloor: max semantics — the floor raises, never lowers, a tier delay",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;
    cfg.relayDelayFloorTicks = 3;

    // Tiers whose own delay is BELOW the floor are raised to it; tiers above it
    // keep their own, larger value. Both directions in one loop so a `min`-shaped
    // mistake cannot pass.
    for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
    {
        const int32_t own = cfg.rttTierInputDelays[tier];
        const int32_t expected = own > cfg.relayDelayFloorTicks ? own : cfg.relayDelayFloorTicks;
        REQUIRE(tierInputDelayTicks(tier, cfg) == expected);
        REQUIRE(tierInputDelayTicks(tier, cfg) >= cfg.relayDelayFloorTicks);
    }

    // The guard that keeps this case non-vacuous: the defaults must straddle the
    // floor, or "raises" and "leaves alone" are not both being exercised.
    REQUIRE(cfg.rttTierInputDelays[0] < cfg.relayDelayFloorTicks);
    REQUIRE(cfg.rttTierInputDelays[kMaxConnectionTierIndex] > cfg.relayDelayFloorTicks);
}

TEST_CASE("RelayDelayFloor: a nonzero floor dominates lanZeroDelayOverride",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;
    cfg.lanZeroDelayOverride = true;

    // Baseline: with no floor the override does its job — tier 0 gets ZERO delay.
    REQUIRE(tierInputDelayTicks(0, cfg) == 0);

    // With a floor, the override is dominated. This is the ordering the floor is
    // applied AFTER the override branch to produce: on a mixed session a LAN
    // sender must still be schedulable by WAN receivers, so it cannot apply its
    // own input at capture+0 while its peers schedule it at capture+floor.
    cfg.relayDelayFloorTicks = 5;
    REQUIRE(tierInputDelayTicks(0, cfg) == cfg.relayDelayFloorTicks);

    // And the override still suppresses the tier's own configured delay — the
    // answer is the FLOOR, not max(floor, rttTierInputDelays[0]) — which is only
    // observable because the floor is bigger than both.
    REQUIRE(cfg.relayDelayFloorTicks > cfg.rttTierInputDelays[0]);

    // Non-tier-0 is untouched by the OVERRIDE (it never applied there) and is
    // floored like everything else — here the floor of 5 still dominates tier 3's
    // own 4, so lower the floor to see the tier's own value survive.
    REQUIRE(tierInputDelayTicks(kMaxConnectionTierIndex, cfg) == cfg.relayDelayFloorTicks);
    cfg.relayDelayFloorTicks = 1;
    REQUIRE(tierInputDelayTicks(kMaxConnectionTierIndex, cfg)
            == cfg.rttTierInputDelays[kMaxConnectionTierIndex]);
    REQUIRE(tierInputDelayTicks(0, cfg) == cfg.relayDelayFloorTicks);   // override still dominated
}

// ---------------------------------------------------------------------------
// 3. THE NO-TIER ARM (review amendment A2b) — the two ends must agree BEFORE a
//    tier has replicated, which is the window the C2 divergence bug lives in.
// ---------------------------------------------------------------------------

TEST_CASE("RelayDelayFloor: the no-tier arm floors forcedInputLatencyTicks, not tier 0",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;
    cfg.relayDelayFloorTicks = 1;   // deliberately BELOW forcedInputLatencyTicks (2)

    const ReplicatedTierConsumer consumer(cfg);
    REQUIRE_FALSE(consumer.hasReceivedTier());

    // The client's pre-arrival answer is max(floor, forced) = 2.
    REQUIRE(consumer.effectiveInputDelayTicks()
            == applyRelayDelayFloor(cfg.forcedInputLatencyTicks, cfg));
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.forcedInputLatencyTicks);

    // THE BUG THIS PINS: a recompute that dropped the no-tier arm would answer
    // tier 0's floored delay instead — max(1, 1) = 1 — while the server's
    // no-entry fallback still answers 2. One tick of standing disagreement, on
    // every client, for the whole pre-tier window.
    REQUIRE(tierInputDelayTicks(0, cfg) != consumer.effectiveInputDelayTicks());

    // The server's two no-tier paths (no table wired / table has no entry) answer
    // the client's value exactly.
    const TestQueue tierlessQueue(cfg);
    TestTierTable tierTable(cfg);
    const TestQueue tieredQueue(cfg, tierTable);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    REQUIRE(tierTable.hasEntry(addr) == false);
    REQUIRE(tierlessQueue.effectiveDelay(addr) == consumer.effectiveInputDelayTicks());
    REQUIRE(tieredQueue.effectiveDelay(addr) == consumer.effectiveInputDelayTicks());
}

TEST_CASE("RelayDelayFloor: a floor above the baseline raises BOTH ends' no-tier answer",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;
    cfg.relayDelayFloorTicks = 8;   // above forcedInputLatencyTicks and every tier delay

    const ReplicatedTierConsumer consumer(cfg);
    const TestQueue tierlessQueue(cfg);
    const FStandaloneTestHandle addr = liveHandle(kConnA);

    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.relayDelayFloorTicks);
    REQUIRE(tierlessQueue.effectiveDelay(addr) == cfg.relayDelayFloorTicks);
}

TEST_CASE("RelayDelayFloor: server and client agree once a tier HAS arrived",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;
    cfg.relayDelayFloorTicks = 3;

    TestTierTable tierTable(cfg);
    const TestQueue queue(cfg, tierTable);
    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr, cfg);
    REQUIRE(tierTable.hasEntry(addr));

    // The client is handed the same index the server derived, as the relay would.
    ReplicatedTierConsumer consumer(cfg);
    consumer.onReplicatedTierReceived(tierTable.lookupTierIndex(addr));

    REQUIRE(consumer.effectiveInputDelayTicks() == queue.effectiveDelay(addr));
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.relayDelayFloorTicks);   // floor dominates tier 0
}

TEST_CASE("RelayDelayFloor: the floor moves the SERVER's park schedule, not just its arithmetic",
          "[Network][RelayDelayFloor]")
{
    // effectiveDelay is not an accessor — it is the number of ticks the queue
    // holds an input for. This drives the real park/release path so the floor is
    // shown to change WHEN an input is applied, which is the whole point of it.
    TimeConfig cfg;
    cfg.relayDelayFloorTicks = 6;

    TestQueue queue(cfg);   // no tier table: the fallback arm, floored
    const FStandaloneTestHandle addr = liveHandle(kConnA);
    constexpr int32_t kCaptureTick = 100;

    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 42);

    // Not due at the un-floored baseline...
    REQUIRE_FALSE(queue.hasReadyForTick<MockSimA>(
        slot0(addr), kCaptureTick + cfg.forcedInputLatencyTicks));
    // ...nor one tick early...
    REQUIRE_FALSE(queue.hasReadyForTick<MockSimA>(
        slot0(addr), kCaptureTick + cfg.relayDelayFloorTicks - 1));
    // ...and due exactly at capture + floor.
    REQUIRE(queue.hasReadyForTick<MockSimA>(
        slot0(addr), kCaptureTick + cfg.relayDelayFloorTicks));

    int out = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(
        slot0(addr), kCaptureTick + cfg.relayDelayFloorTicks, out));
    REQUIRE(out == 42);
}

// ---------------------------------------------------------------------------
// 4. UNIFORM-D FAIRNESS MODE (§11 Q5) — a config VALUE, not a feature.
// ---------------------------------------------------------------------------

TEST_CASE("RelayDelayFloor: floor >= max tier delay collapses every path to one D",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;
    cfg.lanZeroDelayOverride = true;    // the most divergent configuration available

    int32_t maxTierDelay = 0;
    for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
    {
        maxTierDelay = cfg.rttTierInputDelays[tier] > maxTierDelay
            ? cfg.rttTierInputDelays[tier] : maxTierDelay;
    }
    cfg.relayDelayFloorTicks = maxTierDelay > cfg.forcedInputLatencyTicks
        ? maxTierDelay : cfg.forcedInputLatencyTicks;

    // Path 1: every tier. Path 2: the LAN override. Path 3: the no-tier fallback,
    // on both ends. All three answer the identical D — no sender is advantaged by
    // their connection, with zero extra mechanism.
    for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
    {
        REQUIRE(tierInputDelayTicks(tier, cfg) == cfg.relayDelayFloorTicks);
    }

    const ReplicatedTierConsumer unarrived(cfg);
    REQUIRE(unarrived.effectiveInputDelayTicks() == cfg.relayDelayFloorTicks);

    const TestQueue tierlessQueue(cfg);
    REQUIRE(tierlessQueue.effectiveDelay(liveHandle(kConnA)) == cfg.relayDelayFloorTicks);
}

// ---------------------------------------------------------------------------
// 5. THE A5 CLAMP.
// ---------------------------------------------------------------------------

TEST_CASE("RelayDelayFloor: the hard cap is DERIVED from the delay line and the hard cap",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;

    // [T5 / AM-3] The derivation is now over the SMALLER of BOTH ring capacities:
    // the sender's ClientInputDelayLine and the receiver's RelayedInputStore. The
    // receiver-side bound is the SAME inequality with the wire term set to zero
    // (`dA + rollbackWindowHardCap <= capacity + wire`) — see the full derivation
    // at relayDelayFloorHardCapTicks. Written symbolically, NOT against the literal
    // 44, so lowering either capacity moves the cap instead of invalidating this.
    const int32_t smallerCapacity =
        static_cast<int32_t>(kClientInputDelayLineCapacityTicks < kRelayedInputStoreCapacityTicks
                                 ? kClientInputDelayLineCapacityTicks
                                 : kRelayedInputStoreCapacityTicks);

    REQUIRE(relayDelayFloorHardCapTicks(cfg) == smallerCapacity - cfg.rollbackWindowHardCap);
    REQUIRE(relayDelayFloorHardCapTicks(cfg) == 44);

    // Both capacities are 64 today, so the min is a coincidence at the CURRENT
    // values — which is exactly why the arms below feed the derivation different
    // capacities rather than trusting the shipped ones.
    REQUIRE(relayDelayFloorHardCapTicks(cfg)
            == relayDelayFloorHardCapForCapacities(kClientInputDelayLineCapacityTicks,
                                                   kRelayedInputStoreCapacityTicks,
                                                   cfg.rollbackWindowHardCap));

    // The invariant the cap exists to protect: an entry scheduled `floor` ticks out
    // must still be resident after a worst-case resim reaches back
    // `rollbackWindowHardCap` further — on BOTH ends of the relay.
    REQUIRE(relayDelayFloorHardCapTicks(cfg) + cfg.rollbackWindowHardCap == smallerCapacity);

    // Lowering the hard cap frees room and RAISES the ceiling. (The original T11
    // arm, kept: this is the behaviour AM-3 must not have disturbed.)
    cfg.rollbackWindowHardCap = 10;
    REQUIRE(relayDelayFloorHardCapTicks(cfg) == 54);

    // A pathological hard cap can never produce a negative ceiling.
    cfg.rollbackWindowHardCap = smallerCapacity + 5;
    REQUIRE(relayDelayFloorHardCapTicks(cfg) == 0);
}

// THE ARM AM-3 EXISTS FOR. Under the previous delay-line-only derivation this case
// fails: it would answer 44 for a store that only holds 32 entries, admitting a
// floor whose scheduled entries are evicted before their resim window closes — the
// scheduled regime degenerating SILENTLY into permanent fallback, which is the one
// failure relayDelayFloorHardCapTicks exists to make impossible.
//
// Both capacities are compile-time constants, so proving "the cap follows the STORE
// capacity down" is only possible by feeding a different capacity into the
// parameterized derivation. That is why that overload exists.
TEST_CASE("RelayDelayFloor: lowering the RELAY STORE capacity lowers the cap (AM-3)",
          "[Network][RelayDelayFloor]")
{
    const TimeConfig cfg;
    REQUIRE(cfg.rollbackWindowHardCap == 20);

    // Store smaller than the delay line -> the STORE binds.
    REQUIRE(relayDelayFloorHardCapForCapacities(64u, 32u, cfg.rollbackWindowHardCap) == 12);
    REQUIRE(relayDelayFloorHardCapForCapacities(64u, 32u, cfg.rollbackWindowHardCap)
            != relayDelayFloorHardCapForCapacities(64u, 64u, cfg.rollbackWindowHardCap));

    // Delay line smaller than the store -> the DELAY LINE binds. The rule is a MIN,
    // not "whichever one we happened to name first".
    REQUIRE(relayDelayFloorHardCapForCapacities(32u, 64u, cfg.rollbackWindowHardCap) == 12);

    // Equal capacities -> today's shipped answer, from the same expression.
    REQUIRE(relayDelayFloorHardCapForCapacities(64u, 64u, cfg.rollbackWindowHardCap) == 44);

    // A store smaller than the rollback window cannot produce a negative cap; it
    // produces "no floor is safe", which is the honest answer.
    REQUIRE(relayDelayFloorHardCapForCapacities(64u, 8u, cfg.rollbackWindowHardCap) == 0);
}

TEST_CASE("RelayDelayFloor: clampRelayDelayFloorTicks is the shared intake guard",
          "[Network][RelayDelayFloor]")
{
    const TimeConfig cfg;
    const int32_t cap = relayDelayFloorHardCapTicks(cfg);

    // In range: the identity, including both boundaries.
    REQUIRE(clampRelayDelayFloorTicks(0, cfg) == 0);
    REQUIRE(clampRelayDelayFloorTicks(1, cfg) == 1);
    REQUIRE(clampRelayDelayFloorTicks(cap - 1, cfg) == cap - 1);
    REQUIRE(clampRelayDelayFloorTicks(cap, cfg) == cap);

    // Above: capped, not wrapped, not rejected.
    REQUIRE(clampRelayDelayFloorTicks(cap + 1, cfg) == cap);
    REQUIRE(clampRelayDelayFloorTicks(255, cfg) == cap);          // the widest wire value
    REQUIRE(clampRelayDelayFloorTicks(1000000, cfg) == cap);

    // Below: a negative floor is meaningless; it means "no floor".
    REQUIRE(clampRelayDelayFloorTicks(-1, cfg) == 0);
    REQUIRE(clampRelayDelayFloorTicks(-1000, cfg) == 0);

    // BOTH INTAKE POINTS CALL EXACTLY THIS. The ini override at the composition
    // root and the client's floor OnRep are UE-side (no engine-coupled LLT target
    // exists to drive them), but neither owns a private rule — they hand their
    // value to this function and then to
    // SimulationManager::setRelayDelayFloorTicks, which calls it again.
    // Idempotence is what makes that safe:
    REQUIRE(clampRelayDelayFloorTicks(clampRelayDelayFloorTicks(9999, cfg), cfg) == cap);
}

TEST_CASE("RelayDelayFloor: an unclamped field still cannot escape the cap on READ",
          "[Network][RelayDelayFloor]")
{
    // The third guard, and the reason a missed intake point could never silently
    // degenerate the regime: applyRelayDelayFloor re-clamps what it reads. This
    // writes the field DIRECTLY, bypassing both intake points and the setter.
    TimeConfig cfg;
    cfg.relayDelayFloorTicks = 10000;

    const int32_t cap = relayDelayFloorHardCapTicks(cfg);

    REQUIRE(applyRelayDelayFloor(0, cfg) == cap);
    for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
    {
        REQUIRE(tierInputDelayTicks(tier, cfg) == cap);
    }

    const ReplicatedTierConsumer consumer(cfg);
    REQUIRE(consumer.effectiveInputDelayTicks() == cap);

    const TestQueue tierlessQueue(cfg);
    REQUIRE(tierlessQueue.effectiveDelay(liveHandle(kConnA)) == cap);

    // A negative field is equally contained.
    cfg.relayDelayFloorTicks = -7;
    REQUIRE(applyRelayDelayFloor(0, cfg) == 0);
    REQUIRE(tierInputDelayTicks(1, cfg) == cfg.rttTierInputDelays[1]);
}

TEST_CASE("RelayDelayFloor: a floor exactly at the cap is fully usable",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;
    cfg.relayDelayFloorTicks = relayDelayFloorHardCapTicks(cfg);

    const ReplicatedTierConsumer consumer(cfg);
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.relayDelayFloorTicks);
    REQUIRE(tierInputDelayTicks(kMaxConnectionTierIndex, cfg) == cfg.relayDelayFloorTicks);

    // ...and the capture it schedules is still resident in a default delay line.
    REQUIRE(static_cast<std::size_t>(cfg.relayDelayFloorTicks + cfg.rollbackWindowHardCap)
            <= ClientInputDelayLine<int>::kDefaultCapacityTicks);
}

// ---------------------------------------------------------------------------
// 6. THE BENIGN DELTA COLLAPSE + the floor-change delta the client stalls by.
// ---------------------------------------------------------------------------

TEST_CASE("RelayDelayFloor: a dominating floor collapses tier-transition deltas to zero",
          "[Network][RelayDelayFloor]")
{
    TimeConfig cfg;
    cfg.relayDelayFloorTicks = 8;   // above every configured tier delay

    // Every transition, in both directions, reports NO delay change — because
    // there IS none. The player's felt delay is the floor throughout, so there is
    // no prediction-stall debt to pay, and the client's stall path correctly does
    // nothing. (The tier INDEX still changes and still replicates: the server's
    // publish predicate compares indices, not deltas.)
    for (int32_t from = 0; from <= kMaxConnectionTierIndex; ++from)
    {
        for (int32_t to = 0; to <= kMaxConnectionTierIndex; ++to)
        {
            REQUIRE(tierDelayDeltaTicks(from, to, cfg) == 0);
        }
    }

    // A PARTIALLY dominating floor collapses only the transitions it covers —
    // this is not an all-or-nothing switch.
    cfg.relayDelayFloorTicks = 3;
    REQUIRE(tierDelayDeltaTicks(0, 1, cfg) == 0);   // 1 -> 2, both floored to 3
    REQUIRE(tierDelayDeltaTicks(0, 2, cfg) == 0);   // 1 -> 3, floor == 3
    REQUIRE(tierDelayDeltaTicks(0, 3, cfg) == 1);   // 1 -> 4, floor covers only the 3
    REQUIRE(tierDelayDeltaTicks(3, 0, cfg) == -1);  // and the descent mirrors it

    // The un-floored deltas differ, or the case above would be vacuous.
    cfg.relayDelayFloorTicks = 0;
    REQUIRE(tierDelayDeltaTicks(0, 1, cfg) == 1);
    REQUIRE(tierDelayDeltaTicks(0, 3, cfg) == 3);
}

TEST_CASE("RelayDelayFloor: a floor CHANGE produces the delta the client stalls by",
          "[Network][RelayDelayFloor]")
{
    // This is the quantity ASimulationManagerUImpl's shared recompute returns and
    // hands to requestInputDelayIncreaseStall: the change in EFFECTIVE delay, not
    // the change in the floor. The two differ exactly when the other input
    // dominates — which is why the manager cannot compute it from the floor alone.
    TimeConfig cfg;
    ReplicatedTierConsumer consumer(cfg);

    // --- no tier yet: the floor competes with forcedInputLatencyTicks ---------
    const int32_t before = consumer.effectiveInputDelayTicks();
    REQUIRE(before == cfg.forcedInputLatencyTicks);

    cfg.relayDelayFloorTicks = 1;                       // still below the baseline
    REQUIRE(consumer.effectiveInputDelayTicks() - before == 0);     // no stall

    cfg.relayDelayFloorTicks = 6;                       // now dominating
    REQUIRE(consumer.effectiveInputDelayTicks() - before == 6 - cfg.forcedInputLatencyTicks);
    REQUIRE(consumer.effectiveInputDelayTicks() == 6);

    // A DECREASE reports a negative delta, which the stall path drops (the client
    // may simply predict further ahead; ordinary drift gets it there).
    const int32_t atSix = consumer.effectiveInputDelayTicks();
    cfg.relayDelayFloorTicks = 2;
    REQUIRE(consumer.effectiveInputDelayTicks() - atSix < 0);

    // --- tier arrived: the tier can absorb a floor rise entirely -------------
    consumer.onReplicatedTierReceived(kMaxConnectionTierIndex);
    const int32_t tierDelay = cfg.rttTierInputDelays[kMaxConnectionTierIndex];
    cfg.relayDelayFloorTicks = 0;
    const int32_t beforeRise = consumer.effectiveInputDelayTicks();
    REQUIRE(beforeRise == tierDelay);

    cfg.relayDelayFloorTicks = tierDelay - 1;           // below the tier's own delay
    REQUIRE(consumer.effectiveInputDelayTicks() - beforeRise == 0);  // absorbed, no stall

    cfg.relayDelayFloorTicks = tierDelay + 2;
    REQUIRE(consumer.effectiveInputDelayTicks() - beforeRise == 2);  // only the excess is paid
}

#endif // WITH_LOW_LEVEL_TESTS
