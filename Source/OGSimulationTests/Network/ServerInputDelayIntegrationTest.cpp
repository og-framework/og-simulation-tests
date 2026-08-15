// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <limits>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/Network/ServerInputDelayQueue.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/SimulationQueues.h"
// Sibling-relative, matching the Network/ and Determinism/ convention — the test
// tree is not on an include root under either UBT or the standalone CMake build.
#include "MockSimulatablesForNetworkTests.h"
#include "StandaloneTestHandle.h"

// ---------------------------------------------------------------------------
// SERVER INPUT DELAY — INTEGRATION (task 10 parts 3+4 of
// og-netcode-v2-arch-latency; D5.1 + D5.2 server half).
//
// T5's ServerInputDelayQueueTest covers the queue in ISOLATION. This suite
// covers the thing that isolation cannot reach: the queue wired into the shape
// the server actually runs it in, end to end —
//
//     RPC arrival  -> ServerInputDelayQueue::enqueue          (game thread)
//     pre-step     -> tryDequeueForTick -> RemoteMoveQueue    (game thread)
//     authority    -> collectInputAll pops RemoteMoveQueue    (physics thread)
//
// WHAT THIS PINS THAT THE UNIT SUITE DOES NOT. The delay must be applied EXACTLY
// ONCE across that whole chain. The queue alone cannot prove that, because the
// second half of the chain is where a second offset would be introduced — and
// the production consumer (SimulationNetSync::collectInputAll, authority branch)
// pops RemoteMoveQueue in ARRIVAL ORDER without ever comparing the stored
// captureTick to the tick being simulated. `ServerHarness` below reproduces that
// exact contract, so a change that made the server start matching on captureTick
// (a plausible "improvement") would fail here.
//
// WHAT THIS DELIBERATELY DOES NOT COVER. The GAME-THREAD/PHYSICS-THREAD split
// itself, and the ChaosTickMapper step->tick conversion, are UE-side and cannot
// be linked from this engine-free suite. The mapper's off-by-one is verified by
// derivation and documented on
// ASimulationManagerUImpl::releaseDelayedInputsForStep; it is provable only in
// the PIE smoke test. Do NOT read a green run here as evidence of tick
// alignment in the real engine — this suite fixes the DELAY, not the EPOCH.
// ---------------------------------------------------------------------------

using TestQueue     = ServerInputDelayQueue<FStandaloneTestHandle, MockSimA, MockSimB>;
using TestTierTable = ConnectionTierTable<FStandaloneTestHandle>;

namespace
{
    constexpr std::uint32_t kConnA = 1;
    constexpr std::uint32_t kConnB = 2;

    FStandaloneTestHandle liveHandle(std::uint32_t id)
    {
        return FStandaloneTestHandle(id, true);
    }

    // [T17] The queue is keyed on (Address, playerSlot) now, not on Address
    // alone. These pre-existing single-character cases all describe one player
    // per connection, so they route through slot 0 — the primary/parent
    // connection's player. The MULTI-slot behaviour those cases cannot express
    // is covered by CouchCoopEndToEndIndependentDelivery at the bottom of this
    // file, and exhaustively by the CouchCoop suite in ServerInputDelayQueueTest.
    TestQueue::SlotKey slot0(const FStandaloneTestHandle& addr)
    {
        return TestQueue::SlotKey(addr, 0);
    }

    // What the simulation actually consumed on one tick.
    struct ConsumedInput
    {
        bool      consumed    = false;
        int       value       = 0;
        std::uint32_t captureTick = 0;
    };

    // The server, reduced to the two hops that matter and nothing else.
    //
    // Mirrors ASimulationManagerUImpl::releaseDelayedInputsForStep +
    // SimulationNetSync::collectInputAll. Under the T26 due-or-overdue release the
    // delivered captureTick is the entry's STORED tick, surfaced by
    // tryDequeueForTick's out-param (F1) — NOT reconstructed as `simTick - delay`,
    // which would name a future input's tick on an overdue release. This harness
    // carries that true tick through the RemoteMoveQueue hop exactly as production.
    class ServerHarness
    {
    public:
        ServerHarness(const TimeConfig& cfg, TestTierTable& tierTable)
            : m_queue(cfg, tierTable)
        {
        }

        TestQueue& queue() { return m_queue; }

        // Game thread, immediately before the authority step for `simTick`.
        // Single-player convenience form — every pre-T17 case in this file
        // describes one character per connection.
        void releaseForTick(const FStandaloneTestHandle& addr, std::int32_t simTick)
        {
            releaseForTick(TestQueue::SlotKey(addr, 0), simTick);
        }

        // [T17] Slot-aware form. Production's drain iterates (connection, slot)
        // keys and calls this once per key per tick; the couch co-op case below
        // drives it that way.
        //
        // [T26] Releases due-or-overdue and carries the entry's TRUE stored capture
        // tick (F1 out-param) into the RemoteMoveQueue, not `simTick - delay`. The
        // window gate is left at the "no gate" sentinel here: this suite fixes the
        // DELAY and the capture-tick identity end to end; the rollback-window drop
        // boundary is a coordinator concern, covered by ServerReceptionCoordinatorTest.
        void releaseForTick(const TestQueue::SlotKey& key, std::int32_t simTick)
        {
            int released = 0;
            std::int32_t capturedTick = 0;
            if (!m_queue.tryDequeueForTick<MockSimA>(
                    key, simTick, released,
                    std::numeric_limits<std::int32_t>::min(), &capturedTick))
                return;

            m_remoteQueue.queueMove(
                int(released),
                static_cast<std::uint32_t>(capturedTick),      // TRUE stored captureTick (F1)
                static_cast<std::uint32_t>(simTick),
                kGuardDisabled);
        }

        // Physics thread, the authority tick. ARRIVAL ORDER — deliberately
        // ignores the tick argument, exactly as the production consumer does.
        ConsumedInput consume()
        {
            if (m_remoteQueue.empty())
                return ConsumedInput{};

            const auto move = m_remoteQueue.dequeueMove();
            return ConsumedInput{ true, move.input, move.tick };
        }

    private:
        static constexpr std::int32_t kGuardDisabled = -1;

        TestQueue                 m_queue;
        RemoteMoveQueue<int>      m_remoteQueue;
    };

    void seedAtTierZero(TestTierTable& table, const FStandaloneTestHandle& addr)
    {
        table.onRttSample(addr, 0, 1.0);
        REQUIRE(table.lookupTierIndex(addr) == 0);
    }

    void driveToTier(TestTierTable& table,
                     const TimeConfig& cfg,
                     const FStandaloneTestHandle& addr,
                     std::int32_t targetTier)
    {
        const double farAboveTopBoundary =
            static_cast<double>(cfg.rttTierBoundariesMs[TestTierTable::kMaxTierIndex]) * 2.0;

        for (std::int32_t step = 0; step < targetTier; ++step)
        {
            table.onRttSample(addr, step, farAboveTopBoundary);
        }
        REQUIRE(table.lookupTierIndex(addr) == targetTier);
    }
}

// The task's headline acceptance criterion: drive the server tick loop and
// assert the input consumed at each tick is the one captured exactly
// `tierDelay` ticks earlier.
TEST_CASE("ConsumedInputMatchesTierDelayedCaptureTick", "[Network][InputDelayIntegration]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    ServerHarness server(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);

    const std::int32_t delay = server.queue().effectiveDelay(addr);
    REQUIRE(delay == cfg.rttTierInputDelays[0]);

    constexpr std::int32_t kFirstCapture = 100;
    constexpr std::int32_t kNumTicks     = 20;

    // A client streaming one input per tick, arriving as the server ticks.
    for (std::int32_t i = 0; i < kNumTicks; ++i)
    {
        const std::int32_t captureTick = kFirstCapture + i;
        server.queue().enqueue<MockSimA>(slot0(addr), captureTick, /*value=*/int(captureTick));
    }

    // Before the first due tick nothing may reach the sim. If the delay were
    // dropped entirely this loop is where it shows.
    for (std::int32_t simTick = kFirstCapture; simTick < kFirstCapture + delay; ++simTick)
    {
        server.releaseForTick(addr, simTick);
        REQUIRE(server.consume().consumed == false);
    }

    // Steady state: exactly one input per tick, each carrying the capture tick
    // `delay` ticks in the past.
    for (std::int32_t i = 0; i < kNumTicks; ++i)
    {
        const std::int32_t simTick = kFirstCapture + delay + i;

        server.releaseForTick(addr, simTick);
        const ConsumedInput got = server.consume();

        REQUIRE(got.consumed);
        REQUIRE(got.captureTick == static_cast<std::uint32_t>(simTick - delay));
        REQUIRE(got.value       == simTick - delay);
    }
}

// NO DOUBLE-APPLIED OFFSET (task AC part 4). The delay must appear exactly once
// between capture and consumption — never once in the queue and again in the
// consumer.
TEST_CASE("DelayIsAppliedExactlyOnce", "[Network][InputDelayIntegration]")
{
    TimeConfig cfg;
    cfg.tierMinDwellTicks = 1;      // open the R-A2 dwell gate; tier policy is task 4's subject

    TestTierTable tierTable(cfg);
    ServerHarness server(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    driveToTier(tierTable, cfg, addr, /*targetTier=*/2);

    const std::int32_t delay = server.queue().effectiveDelay(addr);
    REQUIRE(delay == cfg.rttTierInputDelays[2]);

    constexpr std::int32_t kCaptureTick = 500;
    server.queue().enqueue<MockSimA>(slot0(addr), kCaptureTick, /*value=*/7);

    // The single-application tick.
    server.releaseForTick(addr, kCaptureTick + delay);
    const ConsumedInput got = server.consume();
    REQUIRE(got.consumed);
    REQUIRE(got.captureTick == static_cast<std::uint32_t>(kCaptureTick));
    REQUIRE(got.value == 7);

    // The consumed input's capture tick is the ORIGINAL, not the release tick.
    // A consumer that re-stamped the input with the tick it was released on
    // would pass the "arrives at the right time" assertion above and fail here.
    REQUIRE(got.captureTick != static_cast<std::uint32_t>(kCaptureTick + delay));

    // And the delay is NOT additive over the baseline. Stated as an explicit
    // inequality because the additive reading is the plausible misimplementation
    // (C2 locks REPLACES, not ADDS).
    REQUIRE(delay != cfg.forcedInputLatencyTicks + cfg.rttTierInputDelays[2]);
}

// Cadence must not drift. Over a long run the sim consumes exactly as many
// inputs as the client produced — no accumulation in the delay queue, no burst.
TEST_CASE("SteadyStateConsumesOnePerTickWithoutDrift", "[Network][InputDelayIntegration]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    ServerHarness server(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);

    const std::int32_t delay = server.queue().effectiveDelay(addr);

    constexpr std::int32_t kFirstCapture = 0;
    constexpr std::int32_t kNumTicks     = 120;

    std::int32_t consumedCount = 0;
    std::int32_t lastCaptureTick = -1;

    for (std::int32_t i = 0; i < kNumTicks; ++i)
    {
        const std::int32_t simTick = kFirstCapture + i;

        // Arrival, then release for this tick — the production ordering.
        server.queue().enqueue<MockSimA>(slot0(addr), simTick, /*value=*/int(simTick));
        server.releaseForTick(addr, simTick);

        const ConsumedInput got = server.consume();
        if (!got.consumed)
            continue;

        ++consumedCount;

        // Strictly increasing: the delay must never reorder the input stream.
        REQUIRE(static_cast<std::int32_t>(got.captureTick) > lastCaptureTick);
        lastCaptureTick = static_cast<std::int32_t>(got.captureTick);
    }

    // Everything captured early enough to be due within the run was consumed —
    // exactly once each, hence a count rather than a lower bound.
    REQUIRE(consumedCount == kNumTicks - delay);
    REQUIRE(lastCaptureTick == kNumTicks - delay - 1);
}

// [T26] A SKIPPED due tick no longer strands the input — it is released late.
//
// This is the headline behavioural delta of the fix. Under the old exact-match
// release, skipping an entry's due tick stranded it until purge (the case this
// test used to pin). Under due-or-overdue release the skipped input is delivered
// on the next drain, one or more ticks late, and the min-captureTick scan (F2)
// keeps two overdue entries in capture order into the FIFO RemoteMoveQueue.
//
// It is also the F1 witness: each delivered captureTick is the entry's TRUE
// stored tick, NOT `simTick - delay` (which would name a future input's tick on
// an overdue release and collide with RemoteMoveQueue's capture-tick dedup).
TEST_CASE("OverdueEntriesReleaseInCaptureOrderNotStranded", "[Network][InputDelayIntegration]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    ServerHarness server(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);

    const std::int32_t delay = server.queue().effectiveDelay(addr);   // tier 0 -> 1

    constexpr std::int32_t kFirstCapture = 400;

    server.queue().enqueue<MockSimA>(slot0(addr), kFirstCapture,     /*value=*/1);  // due 401
    server.queue().enqueue<MockSimA>(slot0(addr), kFirstCapture + 1, /*value=*/2);  // due 402
    REQUIRE(server.queue().pendingCount<MockSimA>(slot0(addr)) == 2);

    // Skip BOTH exact due ticks (401, 402). Drain at 403: both entries are now
    // overdue. The min-captureTick scan releases the OLDER one first — capture
    // order preserved — carrying its TRUE tick, not simTick-delay (== 402).
    server.releaseForTick(addr, kFirstCapture + 3);
    const ConsumedInput first = server.consume();
    REQUIRE(first.consumed);
    REQUIRE(first.value == 1);
    REQUIRE(first.captureTick == static_cast<std::uint32_t>(kFirstCapture));       // 400 (F1)
    REQUIRE(first.captureTick != static_cast<std::uint32_t>(kFirstCapture + 3 - delay));

    // The second entry releases on the following drain, still in order and still
    // carrying its own true capture tick.
    server.releaseForTick(addr, kFirstCapture + 4);
    const ConsumedInput second = server.consume();
    REQUIRE(second.consumed);
    REQUIRE(second.value == 2);
    REQUIRE(second.captureTick == static_cast<std::uint32_t>(kFirstCapture + 1));  // 401 (F1)

    // Neither stranded — the fix delivered both late instead of dropping them.
    REQUIRE(server.queue().pendingCount<MockSimA>(slot0(addr)) == 0);
}

// An unknown connection must still be served. This is the fallback the task
// requires so no player input is silently dropped during rollout — here at the
// policy level (baseline delay), with the UE-side "deliver on the legacy path"
// half living in ASimulationManagerUImpl::tryEnqueueDelayedRemoteInput.
TEST_CASE("UnknownConnectionUsesBaselineAndStillDelivers", "[Network][InputDelayIntegration]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    ServerHarness server(cfg, tierTable);

    // Never sampled — deliberately absent from the tier table.
    const FStandaloneTestHandle unknown = liveHandle(kConnB);
    REQUIRE(tierTable.hasEntry(unknown) == false);

    const std::int32_t delay = server.queue().effectiveDelay(unknown);
    REQUIRE(delay == cfg.forcedInputLatencyTicks);

    constexpr std::int32_t kCaptureTick = 250;
    server.queue().enqueue<MockSimA>(slot0(unknown), kCaptureTick, /*value=*/99);

    server.releaseForTick(unknown, kCaptureTick + delay);
    const ConsumedInput got = server.consume();

    REQUIRE(got.consumed);
    REQUIRE(got.value == 99);
    REQUIRE(got.captureTick == static_cast<std::uint32_t>(kCaptureTick));
}

// A tier change mid-flight shifts the due tick of everything already parked.
// Pinned because it is the one case where the queue's "scan the deque rather
// than test the front" implementation is load-bearing.
TEST_CASE("TierEscalationShiftsParkedInputDueTick", "[Network][InputDelayIntegration]")
{
    TimeConfig cfg;
    cfg.tierMinDwellTicks = 1;      // open the dwell gate; tier policy is task 4's subject

    TestTierTable tierTable(cfg);
    ServerHarness server(cfg, tierTable);

    const FStandaloneTestHandle addr = liveHandle(kConnA);
    seedAtTierZero(tierTable, addr);

    const std::int32_t tier0Delay = server.queue().effectiveDelay(addr);
    REQUIRE(tier0Delay == cfg.rttTierInputDelays[0]);

    constexpr std::int32_t kCaptureTick = 300;
    server.queue().enqueue<MockSimA>(slot0(addr), kCaptureTick, /*value=*/11);

    // RTT degrades before the parked input came due.
    driveToTier(tierTable, cfg, addr, /*targetTier=*/2);
    const std::int32_t tier2Delay = server.queue().effectiveDelay(addr);
    REQUIRE(tier2Delay > tier0Delay);

    // The OLD due tick no longer serves it.
    server.releaseForTick(addr, kCaptureTick + tier0Delay);
    REQUIRE(server.consume().consumed == false);

    // The NEW one does, and the captureTick recovery stays exact across the
    // shift — which is the property that would break if the release site cached
    // the delay from enqueue time instead of reading it at release time.
    server.releaseForTick(addr, kCaptureTick + tier2Delay);
    const ConsumedInput got = server.consume();
    REQUIRE(got.consumed);
    REQUIRE(got.value == 11);
    REQUIRE(got.captureTick == static_cast<std::uint32_t>(kCaptureTick));
}

// ---------------------------------------------------------------------------
// [T17] COUCH CO-OP, END TO END. Two characters on ONE address, driven through
// the full enqueue -> release -> consume path over a run of ticks.
//
// The queue-level suite proves the storage no longer conflates slots. This
// proves the DELIVERY doesn't either: that each character's input survives the
// RemoteMoveQueue hop with its own value and its own recovered captureTick,
// while both stay on the single shared per-wire delay.
//
// Note the RemoteMoveQueue here is deliberately ONE queue, matching production —
// each character has its own USimmableUpdateComponent and therefore its own
// RemoteMoveQueue, but modelling them as one and draining in order is the
// STRICTER test: if the two streams were conflated anywhere upstream, the
// interleaving below would show it as a wrong value or a wrong captureTick.
// ---------------------------------------------------------------------------
TEST_CASE("CouchCoopEndToEndIndependentDelivery", "[Network][InputDelayIntegration]")
{
    const TimeConfig cfg;
    TestTierTable tierTable(cfg);
    ServerHarness server(cfg, tierTable);

    // ONE wire. Two local players on it.
    const FStandaloneTestHandle sharedWire = liveHandle(kConnA);
    seedAtTierZero(tierTable, sharedWire);

    const TestQueue::SlotKey player1(sharedWire, 0);
    const TestQueue::SlotKey player2(sharedWire, 1);

    const std::int32_t delay = server.queue().effectiveDelay(sharedWire);

    constexpr std::int32_t kFirstCapture = 500;
    constexpr std::int32_t kTickCount    = 4;

    // Both players input on every tick, on the SAME capture ticks — the normal
    // case for two controllers on one machine, and the case the old key ate.
    for (std::int32_t t = 0; t < kTickCount; ++t)
    {
        const std::int32_t captureTick = kFirstCapture + t;
        server.queue().enqueue<MockSimA>(player1, captureTick, 100 + int(t));
        server.queue().enqueue<MockSimA>(player2, captureTick, 200 + int(t));
    }

    for (std::int32_t t = 0; t < kTickCount; ++t)
    {
        const std::int32_t simTick = kFirstCapture + t + delay;

        // Production drains per key per tick; do the same, player 1 first.
        server.releaseForTick(player1, simTick);
        const ConsumedInput got1 = server.consume();
        REQUIRE(got1.consumed);
        REQUIRE(got1.value == 100 + int(t));
        REQUIRE(got1.captureTick == static_cast<std::uint32_t>(kFirstCapture + t));

        server.releaseForTick(player2, simTick);
        const ConsumedInput got2 = server.consume();
        REQUIRE(got2.consumed);
        REQUIRE(got2.value == 200 + int(t));
        REQUIRE(got2.captureTick == static_cast<std::uint32_t>(kFirstCapture + t));
    }

    // Nothing stranded in either slot — both streams drained completely.
    REQUIRE(server.queue().pendingCount<MockSimA>(player1) == 0u);
    REQUIRE(server.queue().pendingCount<MockSimA>(player2) == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS
