// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <limits>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/Network/ServerInputDelayQueue.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
// Sibling-relative, matching the Determinism/, NetConfigConceptTest and
// ConnectionTierTableTest convention — the test tree is not on an include root
// under either UBT or the standalone CMake build.
#include "MockSimulatablesForNetworkTests.h"
#include "StandaloneTestHandle.h"

// ---------------------------------------------------------------------------
// Coverage for ServerInputDelayQueue<Address, SimulatableTs...> (task 5 of
// og-netcode-v2-arch-latency — D3.5 / proposal §1.2 + §3.3 + Correction 4),
// instantiated on the engine-free FStandaloneTestHandle from task 2 and the
// two-type mock pack, with no dependency on OGBrawler's SimulatableBrawler.
//
// The queue ships with NO production consumer this stage, so this suite is the
// only thing exercising the delay policy. It is written against TimeConfig
// VALUES rather than literals wherever a default is the thing under test
// (`cfg.rttTierInputDelays[0]`, not `1`), so a future default change moves the
// test with the config instead of silently invalidating it — and so the
// configurability lint has nothing to flag here.
// ---------------------------------------------------------------------------

using TestQueue = ServerInputDelayQueue<FStandaloneTestHandle, MockSimA, MockSimB>;
using TestTierTable = ConnectionTierTable<FStandaloneTestHandle>;

// Correction 4, pinned at compile time rather than left to inspection alone:
// ONE container carrying the whole pack. If someone ever "simplified" this into
// N per-sim instances, the pack size would stop being a property of the type.
static_assert(TestQueue::kSimTypeCount == 2,
    "ServerInputDelayQueue must be ONE container variadic over the sim pack");

// Both input-type spellings resolve — the mocks' `Input` and production's
// `InputType`. Pins the bridge that lets Phase B bind the real simulatables.
static_assert(std::is_same_v<TestQueue::InputFor<MockSimA>, int>);
static_assert(std::is_same_v<TestQueue::InputFor<MockSimB>, float>);
namespace
{
    struct ProductionSpellingProbe { using InputType = double; };
}
static_assert(std::is_same_v<SimulatableInputOf_t<ProductionSpellingProbe>, double>,
    "the production `InputType` spelling must resolve too, or Phase B cannot bind");

namespace
{
    constexpr std::uint32_t kConnA = 1;
    constexpr std::uint32_t kConnB = 2;
    constexpr std::uint32_t kConnC = 3;

    FStandaloneTestHandle liveHandle(std::uint32_t id)
    {
        return FStandaloneTestHandle(id, true);
    }

    // [T17] The queue is keyed on (Address, playerSlot) now, not on Address
    // alone. These pre-existing single-character cases all describe one player
    // per connection, so they route through slot 0 — the primary/parent
    // connection's player. The MULTI-slot behaviour those cases cannot express
    // is covered separately by the CouchCoopSlots suite below.
    TestQueue::SlotKey slot0(const FStandaloneTestHandle& addr)
    {
        return TestQueue::SlotKey(addr, 0);
    }

    // Same rationale as ConnectionTierTableTest's helper: FStandaloneTestHandle's
    // aliveBit participates in BOTH operator== and std::hash (a deliberate task-2
    // decision), so a stored map key's liveness cannot be flipped in place and a
    // flipped local copy is a DIFFERENT key that would not find the entry. A
    // handle that is already dead drives the exact same `!isAlive()` branch of
    // reapDeadHandles. Task 2's header is deliberately left untouched.
    FStandaloneTestHandle deadHandle(std::uint32_t id)
    {
        return FStandaloneTestHandle(id, false);
    }

    // Puts `addr` in the tier table at tier 0 — a real entry, not an unknown
    // Address. The distinction matters: an unknown Address falls back to
    // forcedInputLatencyTicks, a known tier-0 one uses rttTierInputDelays[0].
    void seedAtTierZero(TestTierTable& table, const FStandaloneTestHandle& addr)
    {
        table.onRttSample(addr, 0, 1.0);
        REQUIRE(table.lookupTierIndex(addr) == 0);
        REQUIRE(table.hasEntry(addr));
    }

    // Drives `table` up to `targetTier`, one promotion per sample. Requires a cfg
    // whose dwell gate has been opened to a single sample. Asserts the tier was
    // reached so a tier-policy regression surfaces here rather than as a
    // confusing failure in the delay assertion the case actually cares about.
    void driveToTier(TestTierTable& table,
                     const TimeConfig& cfg,
                     const FStandaloneTestHandle& addr,
                     int32_t targetTier)
    {
        const double farAboveTopBoundary =
            static_cast<double>(cfg.rttTierBoundariesMs[TestTierTable::kMaxTierIndex]) * 2.0;

        for (int32_t step = 0; step < targetTier; ++step)
        {
            table.onRttSample(addr, step, farAboveTopBoundary);
        }
        REQUIRE(table.lookupTierIndex(addr) == targetTier);
    }
}

TEST_CASE("EnqueueDequeueRoundTrip", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);

    constexpr int32_t kCaptureTick = 100;
    constexpr int kInputValue = 42;

    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, kInputValue);

    const int32_t delay = queue.effectiveDelay(addr);
    REQUIRE(delay == cfg.rttTierInputDelays[0]);

    // Not servable before its due tick — the delay is the whole point of the
    // type, so "arrives early" must be a failure, not merely unasserted.
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), kCaptureTick) == false);
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), kCaptureTick + delay));

    int out = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), kCaptureTick + delay, out));
    REQUIRE(out == kInputValue);

    // Consumed, not merely read: a second dequeue at the same tick finds nothing.
    int again = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), kCaptureTick + delay, again) == false);
    REQUIRE(queue.pendingCount<MockSimA>(slot0(addr)) == 0u);
}

TEST_CASE("DedupOnDuplicateCaptureTick", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);

    constexpr int32_t kCaptureTick = 100;

    // The redundancy bundle re-sends recent inputs by design, so the same
    // capture tick legitimately arrives twice. FIRST value wins, matching
    // tryAppendSlot and RemoteMoveQueue::queueMove (task 3's invariant).
    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 42);
    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 99);

    REQUIRE(queue.pendingCount<MockSimA>(slot0(addr)) == 1u);

    const int32_t dueTick = kCaptureTick + queue.effectiveDelay(addr);

    int out = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), dueTick, out));
    REQUIRE(out == 42);     // first-wins, NOT last-wins

    // Exactly one survived — the duplicate was dropped, not parked behind it.
    int again = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), dueTick, again) == false);
}

TEST_CASE("DequeueRespectsTierDelay", "[Network][InputDelayQueue]")
{
    TimeConfig cfg;
    cfg.tierMinDwellTicks = 1;      // open the R-A2 dwell gate; not the thing under test here

    constexpr int32_t kCaptureTick = 100;

    SECTION("tier 0 uses rttTierInputDelays[0]")
    {
        TestTierTable tierTable(cfg);
        TestQueue queue(cfg, tierTable);
        const FStandaloneTestHandle addr = liveHandle(kConnA);

        seedAtTierZero(tierTable, addr);
        queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 7);

        REQUIRE(queue.effectiveDelay(addr) == cfg.rttTierInputDelays[0]);
        REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), kCaptureTick + cfg.rttTierInputDelays[0]));
    }

    SECTION("tier 2 uses rttTierInputDelays[2]")
    {
        TestTierTable tierTable(cfg);
        TestQueue queue(cfg, tierTable);
        const FStandaloneTestHandle addr = liveHandle(kConnB);

        driveToTier(tierTable, cfg, addr, 2);
        queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 7);

        REQUIRE(queue.effectiveDelay(addr) == cfg.rttTierInputDelays[2]);
        REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), kCaptureTick + cfg.rttTierInputDelays[2]));

        // The tier delay REPLACES the baseline (C2, locked) — it is NOT added to
        // forcedInputLatencyTicks. This is the assertion that would catch a
        // regression to additive semantics.
        REQUIRE(queue.effectiveDelay(addr)
                != cfg.forcedInputLatencyTicks + cfg.rttTierInputDelays[2]);
        // [T26] Under due-or-overdue release readiness holds at the due tick AND
        // after, so the additive tick (100 + 2 + 3) is now also "ready" — the
        // additive regression is instead caught by readiness arriving at the
        // REPLACE due tick (asserted just above) but NOT one tick earlier, which an
        // additive delay (due at 100 + 5) could not satisfy.
        REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), kCaptureTick + cfg.rttTierInputDelays[2] - 1)
                == false);
    }
}

TEST_CASE("DifferentSimTypesIndependent", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);

    constexpr int32_t kCaptureTick = 100;
    const int32_t dueTick = kCaptureTick + queue.effectiveDelay(addr);

    // Same Address, same capture tick, different simulatable — the per-sim
    // storage must keep these completely apart.
    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 42);

    REQUIRE(queue.pendingCount<MockSimA>(slot0(addr)) == 1u);
    REQUIRE(queue.pendingCount<MockSimB>(slot0(addr)) == 0u);
    REQUIRE(queue.hasReadyForTick<MockSimB>(slot0(addr), dueTick) == false);

    float outB = 0.0f;
    REQUIRE(queue.tryDequeueForTick<MockSimB>(slot0(addr), dueTick, outB) == false);

    // ...and the MockSimA entry is untouched by MockSimB's failed dequeue.
    int outA = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), dueTick, outA));
    REQUIRE(outA == 42);

    // The converse direction too: a MockSimB enqueue at the SAME capture tick is
    // not deduped against MockSimA's, because dedup is per (Address, SimT).
    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 42);
    queue.enqueue<MockSimB>(slot0(addr), kCaptureTick, 1.5f);
    REQUIRE(queue.pendingCount<MockSimA>(slot0(addr)) == 1u);
    REQUIRE(queue.pendingCount<MockSimB>(slot0(addr)) == 1u);

    REQUIRE(queue.tryDequeueForTick<MockSimB>(slot0(addr), dueTick, outB));
    REQUIRE(outB == Catch::Approx(1.5f));
    REQUIRE(queue.pendingCount<MockSimA>(slot0(addr)) == 1u);      // survivor untouched
}

TEST_CASE("PurgeOlderThan", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);

    constexpr int32_t kStaleTick = 95;

    queue.enqueue<MockSimA>(slot0(addr), kStaleTick - 5, 1);    // 90 — dropped
    queue.enqueue<MockSimA>(slot0(addr), kStaleTick,     2);    // 95 — boundary, KEPT
    queue.enqueue<MockSimA>(slot0(addr), kStaleTick + 5, 3);    // 100 — kept
    queue.enqueue<MockSimB>(slot0(addr), kStaleTick - 5, 9.0f); // other sim, untouched

    REQUIRE(queue.pendingCount<MockSimA>(slot0(addr)) == 3u);

    queue.purgeOlderThan<MockSimA>(kStaleTick);

    // Threshold is STRICTLY older: the entry captured exactly at kStaleTick
    // survives.
    REQUIRE(queue.pendingCount<MockSimA>(slot0(addr)) == 2u);

    const int32_t delay = queue.effectiveDelay(addr);
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), (kStaleTick - 5) + delay) == false);
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), kStaleTick + delay));
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), (kStaleTick + 5) + delay));

    // Purge is scoped to the simulatable it names — MockSimB's equally-old entry
    // is deliberately still there.
    REQUIRE(queue.pendingCount<MockSimB>(slot0(addr)) == 1u);
}

TEST_CASE("UnknownAddressYieldsNoDequeue", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle known = liveHandle(kConnA);
    const FStandaloneTestHandle unknown = liveHandle(kConnB);

    seedAtTierZero(tierTable, known);
    queue.enqueue<MockSimA>(slot0(known), 100, 42);

    int out = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(unknown), 101, out) == false);
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(unknown), 101) == false);
    REQUIRE(out == 0);                                  // out left untouched
    REQUIRE(queue.hasConnection(unknown) == false);

    // Querying must not create a bucket — otherwise a lookup-heavy tick loop
    // would grow the map with connections that never sent anything.
    REQUIRE(queue.connectionCount() == 1u);

    // An Address unknown to the TIER TABLE (as opposed to the queue) is the
    // other fallback path: it gets the flat baseline, not tier 0's delay.
    REQUIRE(tierTable.hasEntry(unknown) == false);
    REQUIRE(queue.effectiveDelay(unknown) == cfg.forcedInputLatencyTicks);

    // ...and with no tier table wired at all, every Address does.
    TestQueue tierlessQueue(cfg);
    REQUIRE(tierlessQueue.hasTierTable() == false);
    REQUIRE(tierlessQueue.effectiveDelay(known) == cfg.forcedInputLatencyTicks);
}

TEST_CASE("TierChangeUpdatesEffectiveDelay", "[Network][InputDelayQueue]")
{
    TimeConfig cfg;
    cfg.tierMinDwellTicks = 1;      // open the dwell gate; tier policy is task 4's subject

    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    constexpr int32_t kCaptureTick = 100;

    seedAtTierZero(tierTable, addr);
    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 42);

    const int32_t tier0Delay = cfg.rttTierInputDelays[0];
    REQUIRE(queue.effectiveDelay(addr) == tier0Delay);
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), kCaptureTick + tier0Delay));

    // Promote the SAME Address to tier 2 while the input is already parked.
    driveToTier(tierTable, cfg, addr, 2);

    const int32_t tier2Delay = cfg.rttTierInputDelays[2];
    REQUIRE(tier2Delay > tier0Delay);       // guard: the shift under test is real

    // The already-parked entry moves with the tier — readiness is computed at
    // query time from the CURRENT tier, not frozen at enqueue time. This is why
    // tryDequeueForTick scans rather than trusting the front of the deque.
    REQUIRE(queue.effectiveDelay(addr) == tier2Delay);
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), kCaptureTick + tier0Delay) == false);
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), kCaptureTick + tier2Delay));

    int out = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), kCaptureTick + tier0Delay, out) == false);
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), kCaptureTick + tier2Delay, out));
    REQUIRE(out == 42);
}

TEST_CASE("DeadHandleReap", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle live = liveHandle(kConnA);
    const FStandaloneTestHandle dead = deadHandle(kConnB);   // already dead — see helper
    const FStandaloneTestHandle stale = liveHandle(kConnC);  // alive but long silent

    constexpr int32_t kDeadlineTicks = 60;
    constexpr int32_t kNowTick = 100;

    // Every connection parks input for BOTH simulatables, so the reap has to
    // clear the whole tuple rather than just the first queue.
    for (const FStandaloneTestHandle& addr : { live, dead })
    {
        queue.enqueue<MockSimA>(slot0(addr), kNowTick, 1);
        queue.enqueue<MockSimB>(slot0(addr), kNowTick, 1.0f);
    }
    queue.enqueue<MockSimA>(slot0(stale), kNowTick - kDeadlineTicks - 1, 1);
    queue.enqueue<MockSimB>(slot0(stale), kNowTick - kDeadlineTicks - 1, 1.0f);

    REQUIRE(queue.connectionCount() == 3u);

    queue.reapDeadHandles(kNowTick, kDeadlineTicks);

    // Dead handle evicted by the liveness branch, silent handle by the deadline
    // branch, freshly-active handle by neither.
    REQUIRE(queue.hasConnection(dead) == false);
    REQUIRE(queue.hasConnection(stale) == false);
    REQUIRE(queue.hasConnection(live));
    REQUIRE(queue.connectionCount() == 1u);

    // Cleared from EVERY per-sim queue, not just the first in the tuple — the
    // single-container invariant this type exists to guarantee.
    REQUIRE(queue.pendingCount<MockSimA>(slot0(dead)) == 0u);
    REQUIRE(queue.pendingCount<MockSimB>(slot0(dead)) == 0u);
    REQUIRE(queue.pendingCount<MockSimA>(slot0(stale)) == 0u);
    REQUIRE(queue.pendingCount<MockSimB>(slot0(stale)) == 0u);

    // The survivor keeps its input in both queues — reaping is not destructive
    // to connections that passed.
    REQUIRE(queue.pendingCount<MockSimA>(slot0(live)) == 1u);
    REQUIRE(queue.pendingCount<MockSimB>(slot0(live)) == 1u);

    // Idempotent: a second reap at the same tick changes nothing.
    queue.reapDeadHandles(kNowTick, kDeadlineTicks);
    REQUIRE(queue.connectionCount() == 1u);
    REQUIRE(queue.pendingCount<MockSimA>(slot0(live)) == 1u);
}

// ---------------------------------------------------------------------------
// T26 — DUE-OR-OVERDUE RELEASE + the four fable amendments (F1-F3).
//
// tryDequeueForTick releases an entry due-or-overdue (captureTick + delay <=
// currentServerTick) instead of exact-match, so a skipped tick or a delay shift
// delivers late rather than stranding. These cases pin the amendments the fix
// turns on: F1 (the TRUE stored captureTick is surfaced), F2 (min-captureTick
// scan, not deque-front), F3 (the staleBefore window gate leaves beyond-window
// entries for the purge), and the in-time case staying identical to exact-match.
// ---------------------------------------------------------------------------

TEST_CASE("OverdueReleaseSurfacesTrueCaptureTick", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);
    const int32_t delay = queue.effectiveDelay(addr);

    constexpr int32_t kCaptureTick = 100;
    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 42);

    // Drain well PAST the exact due tick (kCaptureTick + delay): the entry is
    // overdue but still released, and the surfaced captureTick is the TRUE stored
    // value (F1), NOT a reconstructed simTick - delay.
    const int32_t overdueTick = kCaptureTick + delay + 5;
    int out = 0;
    int32_t captured = -1;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(
        slot0(addr), overdueTick, out,
        std::numeric_limits<int32_t>::min(), &captured));
    REQUIRE(out == 42);
    REQUIRE(captured == kCaptureTick);                 // TRUE stored tick (F1)
    REQUIRE(captured != overdueTick - delay);          // NOT simTick - delay
}

TEST_CASE("MinCaptureTickReleasedFirstUnderReorder", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);
    const int32_t delay = queue.effectiveDelay(addr);

    // UDP reorder of redundancy bundles: the OLDER capture tick is enqueued AFTER
    // the newer one, so it sits BEHIND it in the deque. enqueue is a bare push_back
    // (no sort), so deque-front order is newer-then-older here.
    constexpr int32_t kNewer = 201;
    constexpr int32_t kOlder = 200;
    queue.enqueue<MockSimA>(slot0(addr), kNewer, /*value=*/2);   // pushed first
    queue.enqueue<MockSimA>(slot0(addr), kOlder, /*value=*/1);   // pushed second, older

    // Both are due at a tick past kNewer + delay. The min-captureTick scan (F2)
    // must release the OLDER (200) first, not the deque front (201) — otherwise
    // the FIFO RemoteMoveQueue would receive the two inputs out of capture order.
    const int32_t drainTick = kNewer + delay;
    int out = 0;
    int32_t captured = -1;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), drainTick, out,
        std::numeric_limits<int32_t>::min(), &captured));
    REQUIRE(captured == kOlder);        // 200 released first despite being enqueued last
    REQUIRE(out == 1);

    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), drainTick, out,
        std::numeric_limits<int32_t>::min(), &captured));
    REQUIRE(captured == kNewer);        // 201 second
    REQUIRE(out == 2);
}

TEST_CASE("StaleBeforeGateLeavesBeyondWindowEntryForPurge", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);
    const int32_t delay = queue.effectiveDelay(addr);

    constexpr int32_t kCaptureTick = 100;
    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 42);

    // Drain far in the future with a window gate that puts kCaptureTick BEYOND the
    // rollback window (staleBefore > kCaptureTick). The entry is due-and-overdue
    // but the F3 gate refuses to release it — it must fall through to the purge,
    // keeping the purge the single drop point. It is NOT consumed and NOT erased.
    const int32_t drainTick   = kCaptureTick + 500;
    const int32_t staleBefore = kCaptureTick + 1;      // captureTick < staleBefore
    int out = 0;
    REQUIRE_FALSE(queue.tryDequeueForTick<MockSimA>(slot0(addr), drainTick, out, staleBefore));
    REQUIRE_FALSE(queue.hasReadyForTick<MockSimA>(slot0(addr), drainTick, staleBefore));
    REQUIRE(queue.pendingCount<MockSimA>(slot0(addr)) == 1u);   // still parked, for the purge

    // With the gate open (an in-window staleBefore), the same overdue entry IS
    // released — proving the gate, not the due predicate, held it back above.
    const int32_t inWindow = kCaptureTick - 5;
    REQUIRE(queue.hasReadyForTick<MockSimA>(slot0(addr), drainTick, inWindow));
    int32_t captured = -1;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), drainTick, out, inWindow, &captured));
    REQUIRE(captured == kCaptureTick);
    (void)delay;
}

TEST_CASE("InTimeReleaseIdenticalToExactMatch", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);
    const int32_t delay = queue.effectiveDelay(addr);

    // Steady state: one entry per tick, drained on its exact due tick. Under
    // due-or-overdue the ONLY due entry at tick T is captureTick == T - delay, so
    // `<=` selects exactly what `==` did and the delivered tick is byte-identical.
    constexpr int32_t kFirst = 100;
    constexpr int32_t kCount = 6;
    for (int32_t i = 0; i < kCount; ++i)
        queue.enqueue<MockSimA>(slot0(addr), kFirst + i, 1000 + int(i));

    for (int32_t i = 0; i < kCount; ++i)
    {
        const int32_t simTick = kFirst + i + delay;    // the exact due tick each time
        int out = 0;
        int32_t captured = -1;
        REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), simTick, out,
            std::numeric_limits<int32_t>::min(), &captured));
        REQUIRE(captured == simTick - delay);           // == exact-match's delivered tick
        REQUIRE(captured == kFirst + i);
        REQUIRE(out == 1000 + int(i));
    }
    REQUIRE(queue.pendingCount<MockSimA>(slot0(addr)) == 0u);
}

TEST_CASE("DelayIncreaseStillDeliversWhenExactTickSkipped", "[Network][InputDelayQueue]")
{
    TimeConfig cfg;
    cfg.tierMinDwellTicks = 1;      // open the dwell gate; tier policy is task 4's subject

    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);

    constexpr int32_t kCaptureTick = 100;
    queue.enqueue<MockSimA>(slot0(addr), kCaptureTick, 42);
    const int32_t tier0Delay = cfg.rttTierInputDelays[0];

    // Delay INCREASES under the parked input (tier 0 -> 2 shifts its due tick out).
    driveToTier(tierTable, cfg, addr, 2);
    const int32_t tier2Delay = cfg.rttTierInputDelays[2];
    REQUIRE(tier2Delay > tier0Delay);

    // Draining at the OLD due tick finds nothing (correctly not-yet-due), and
    // draining PAST the new due tick still delivers — never dropped.
    int out = 0;
    REQUIRE_FALSE(queue.tryDequeueForTick<MockSimA>(slot0(addr), kCaptureTick + tier0Delay, out));
    int32_t captured = -1;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slot0(addr), kCaptureTick + tier2Delay + 3, out,
        std::numeric_limits<int32_t>::min(), &captured));
    REQUIRE(out == 42);
    REQUIRE(captured == kCaptureTick);      // still its true tick after the shift
}

// ---------------------------------------------------------------------------
// COUCH CO-OP — N>1 CHARACTERS ON ONE ADDRESS. (T17.)
//
// THE REGRESSION SUITE. These are the cases that would have caught the original
// defect, and they are the reason the queue key was widened from `Address` to
// `(Address, playerSlot)`.
//
// This game ships multiple player-controlled characters on one client (shared
// isometric camera, couch brawl / co-op) — a designed topology, implemented by
// UE as UChildConnections under one parent UNetConnection. The queue was
// originally keyed on the wire alone, so every character on a machine shared one
// deque and the capture-tick dedup silently ate all but the first character's
// input for any tick they both captured on. Two players on one couch, on two
// different input timelines against the same server tick.
//
// Every case below is written so it FAILS under the old Address-only key. The
// first one fails on the dedup (one entry parked where two were enqueued); the
// value assertions fail on the clobber (player 2 reading player 1's input).
// ---------------------------------------------------------------------------

namespace
{
    // Same wire, different local players — the exact shape the UE binding
    // produces from one parent connection and its UChildConnections.
    TestQueue::SlotKey slotOn(const FStandaloneTestHandle& addr, std::uint8_t slot)
    {
        return TestQueue::SlotKey(addr, slot);
    }
}

TEST_CASE("CouchCoopSlotsDoNotClobber", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    // ONE address — one machine, one wire, one latency.
    const FStandaloneTestHandle sharedWire = liveHandle(kConnA);
    const TestQueue::SlotKey player1 = slotOn(sharedWire, 0);
    const TestQueue::SlotKey player2 = slotOn(sharedWire, 1);

    constexpr int32_t kCaptureTick = 100;
    constexpr int      kP1Input = 11;
    constexpr int      kP2Input = 22;

    // THE COLLIDING CASE: both characters capture on the SAME tick, which is the
    // normal case for two players on one machine sharing a frame clock — not a
    // rare coincidence. Under the old key the second enqueue hit the capture-tick
    // dedup and was dropped on the floor.
    queue.enqueue<MockSimA>(player1, kCaptureTick, kP1Input);
    queue.enqueue<MockSimA>(player2, kCaptureTick, kP2Input);

    REQUIRE(queue.pendingCount<MockSimA>(player1) == 1u);
    REQUIRE(queue.pendingCount<MockSimA>(player2) == 1u);
    REQUIRE(queue.slotCountFor<MockSimA>(sharedWire) == 2u);

    // ONE connection, though — liveness and tier are per-wire, and widening the
    // input key must not have split the per-connection bookkeeping too.
    REQUIRE(queue.connectionCount() == 1u);
    REQUIRE(queue.hasConnection(sharedWire));

    // Both delayed by the SAME amount: they share a wire, so they share a tier.
    // This is the half of the design that deliberately did NOT change.
    const int32_t delay = queue.effectiveDelay(sharedWire);
    REQUIRE(queue.effectiveDelay(player1) == delay);
    REQUIRE(queue.effectiveDelay(player2) == delay);

    const int32_t dueTick = kCaptureTick + delay;
    REQUIRE(queue.hasReadyForTick<MockSimA>(player1, dueTick));
    REQUIRE(queue.hasReadyForTick<MockSimA>(player2, dueTick));

    // And each receives ITS OWN input, not its couch partner's.
    int outP1 = 0;
    int outP2 = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(player1, dueTick, outP1));
    REQUIRE(queue.tryDequeueForTick<MockSimA>(player2, dueTick, outP2));
    REQUIRE(outP1 == kP1Input);
    REQUIRE(outP2 == kP2Input);

    // Consuming one slot leaves the other untouched — no shared deque anywhere.
    REQUIRE(queue.pendingCount<MockSimA>(player1) == 0u);
    REQUIRE(queue.pendingCount<MockSimA>(player2) == 0u);
}

TEST_CASE("CouchCoopSlotsIndependentAcrossTicks", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle sharedWire = liveHandle(kConnA);
    const TestQueue::SlotKey player1 = slotOn(sharedWire, 0);
    const TestQueue::SlotKey player2 = slotOn(sharedWire, 1);
    const TestQueue::SlotKey player3 = slotOn(sharedWire, 2);

    const int32_t delay = queue.effectiveDelay(sharedWire);
    constexpr int32_t kFirstCapture = 200;
    constexpr int32_t kTickCount    = 5;

    // A run of ticks, all three players active on every one. Values are chosen so
    // each (player, tick) pair is UNIQUE — a value collision would let a clobber
    // pass unnoticed, which is the failure mode a naive "both got something"
    // assertion has.
    for (int32_t t = 0; t < kTickCount; ++t)
    {
        const int32_t captureTick = kFirstCapture + t;
        queue.enqueue<MockSimA>(player1, captureTick, 1000 + int(t));
        queue.enqueue<MockSimA>(player2, captureTick, 2000 + int(t));
        queue.enqueue<MockSimA>(player3, captureTick, 3000 + int(t));
    }

    REQUIRE(queue.slotCountFor<MockSimA>(sharedWire) == 3u);
    REQUIRE(queue.pendingCount<MockSimA>(player1) == std::size_t(kTickCount));
    REQUIRE(queue.pendingCount<MockSimA>(player2) == std::size_t(kTickCount));
    REQUIRE(queue.pendingCount<MockSimA>(player3) == std::size_t(kTickCount));

    // Drain in tick order, exactly as the production drain does, and check every
    // release lands on the right player with the right value.
    for (int32_t t = 0; t < kTickCount; ++t)
    {
        const int32_t simTick = kFirstCapture + t + delay;

        int out1 = 0;
        int out2 = 0;
        int out3 = 0;
        REQUIRE(queue.tryDequeueForTick<MockSimA>(player1, simTick, out1));
        REQUIRE(queue.tryDequeueForTick<MockSimA>(player2, simTick, out2));
        REQUIRE(queue.tryDequeueForTick<MockSimA>(player3, simTick, out3));

        REQUIRE(out1 == 1000 + int(t));
        REQUIRE(out2 == 2000 + int(t));
        REQUIRE(out3 == 3000 + int(t));
    }

    REQUIRE(queue.pendingCount<MockSimA>(player1) == 0u);
    REQUIRE(queue.pendingCount<MockSimA>(player2) == 0u);
    REQUIRE(queue.pendingCount<MockSimA>(player3) == 0u);
}

TEST_CASE("SameSlotOnDifferentAddressesIsIndependent", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    // The mirror-image confusion: two SEPARATE clients, each with a primary
    // player on slot 0. The slot must not be the whole key any more than the
    // address was.
    const FStandaloneTestHandle wireA = liveHandle(kConnA);
    const FStandaloneTestHandle wireB = liveHandle(kConnB);

    constexpr int32_t kCaptureTick = 100;
    queue.enqueue<MockSimA>(slotOn(wireA, 0), kCaptureTick, 1);
    queue.enqueue<MockSimA>(slotOn(wireB, 0), kCaptureTick, 2);

    REQUIRE(queue.pendingCount<MockSimA>(slotOn(wireA, 0)) == 1u);
    REQUIRE(queue.pendingCount<MockSimA>(slotOn(wireB, 0)) == 1u);
    REQUIRE(queue.connectionCount() == 2u);      // two wires, unlike the couch case

    const int32_t dueTick = kCaptureTick + queue.effectiveDelay(wireA);
    int outA = 0;
    int outB = 0;
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slotOn(wireA, 0), dueTick, outA));
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slotOn(wireB, 0), dueTick, outB));
    REQUIRE(outA == 1);
    REQUIRE(outB == 2);

    // An unused slot on a KNOWN address is as absent as an unknown address.
    int unused = 0;
    REQUIRE(queue.hasReadyForTick<MockSimA>(slotOn(wireA, 1), dueTick) == false);
    REQUIRE(queue.tryDequeueForTick<MockSimA>(slotOn(wireA, 1), dueTick, unused) == false);
}

TEST_CASE("ReapClearsEverySlotForADeadAddress", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    // A whole couch disconnects at once — a dead wire takes every local player
    // with it. Under the widened key the reap only knows the ADDRESS half, so
    // this pins that it still clears ALL of that address's slots. A reap that
    // erased only slot 0 would leak the other players' parked input permanently:
    // nothing would ever stamp that address active again, so no later reap could
    // reach it.
    const FStandaloneTestHandle dead = deadHandle(kConnA);
    const FStandaloneTestHandle live = liveHandle(kConnB);

    constexpr int32_t kNowTick       = 100;
    constexpr int32_t kDeadlineTicks = 60;

    for (std::uint8_t slot = 0; slot < 3; ++slot)
    {
        queue.enqueue<MockSimA>(slotOn(dead, slot), kNowTick, int(slot));
        queue.enqueue<MockSimB>(slotOn(dead, slot), kNowTick, float(slot));
    }
    queue.enqueue<MockSimA>(slotOn(live, 0), kNowTick, 99);
    queue.enqueue<MockSimA>(slotOn(live, 1), kNowTick, 98);

    REQUIRE(queue.slotCountFor<MockSimA>(dead) == 3u);
    REQUIRE(queue.slotCountFor<MockSimB>(dead) == 3u);
    REQUIRE(queue.slotCountFor<MockSimA>(live) == 2u);

    queue.reapDeadHandles(kNowTick, kDeadlineTicks);

    // Every slot, in every simulatable's queue, gone.
    REQUIRE(queue.slotCountFor<MockSimA>(dead) == 0u);
    REQUIRE(queue.slotCountFor<MockSimB>(dead) == 0u);
    for (std::uint8_t slot = 0; slot < 3; ++slot)
    {
        REQUIRE(queue.pendingCount<MockSimA>(slotOn(dead, slot)) == 0u);
        REQUIRE(queue.pendingCount<MockSimB>(slotOn(dead, slot)) == 0u);
    }
    REQUIRE(queue.hasConnection(dead) == false);

    // The surviving couch keeps BOTH of its players' input — the reap is keyed
    // per address, so it must not spill across wires at any slot.
    REQUIRE(queue.slotCountFor<MockSimA>(live) == 2u);
    REQUIRE(queue.pendingCount<MockSimA>(slotOn(live, 0)) == 1u);
    REQUIRE(queue.pendingCount<MockSimA>(slotOn(live, 1)) == 1u);
    REQUIRE(queue.hasConnection(live));
}

TEST_CASE("PurgeOlderThanSpansAllSlots", "[Network][InputDelayQueue]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    TestQueue queue(cfg, tierTable);

    const FStandaloneTestHandle sharedWire = liveHandle(kConnA);
    constexpr int32_t kStaleTick = 95;

    // Both couch players hold one stale and one fresh entry. The purge walks
    // buckets, and the widened key multiplied the bucket count per wire — this
    // pins that it still reaches every one of them.
    for (std::uint8_t slot = 0; slot < 2; ++slot)
    {
        queue.enqueue<MockSimA>(slotOn(sharedWire, slot), kStaleTick - 5, 1);
        queue.enqueue<MockSimA>(slotOn(sharedWire, slot), kStaleTick + 5, 2);
    }

    queue.purgeOlderThan<MockSimA>(kStaleTick);

    for (std::uint8_t slot = 0; slot < 2; ++slot)
    {
        REQUIRE(queue.pendingCount<MockSimA>(slotOn(sharedWire, slot)) == 1u);
    }
    REQUIRE(queue.slotCountFor<MockSimA>(sharedWire) == 2u);
}

TEST_CASE("SlotRangeCheckMatchesSubstitutionMaskBound", "[Network][InputDelayQueue]")
{
    // The bound is proposal §8.2's uint8 substitution mask, shared with Stage 4's
    // ServerSubstitutionTracker so the two cannot drift. Pinned here because the
    // UE binding's fallback branch is written against this exact predicate.
    STATIC_REQUIRE(TestQueue::SlotKey::kMaxPlayerSlot == 7);

    const FStandaloneTestHandle addr = liveHandle(kConnA);

    REQUIRE(TestQueue::SlotKey(addr, 0).hasValidSlot());
    REQUIRE(TestQueue::SlotKey(addr, 7).hasValidSlot());
    REQUIRE(TestQueue::SlotKey(addr, 8).hasValidSlot() == false);
    REQUIRE(TestQueue::SlotKey(addr, 255).hasValidSlot() == false);

    // Slots are part of the key's identity, not decoration — a differing slot
    // must produce a differing key even when the address matches exactly.
    REQUIRE(TestQueue::SlotKey(addr, 0) == TestQueue::SlotKey(addr, 0));
    REQUIRE_FALSE(TestQueue::SlotKey(addr, 0) == TestQueue::SlotKey(addr, 1));
}

#endif // WITH_LOW_LEVEL_TESTS
