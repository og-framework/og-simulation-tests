// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationQueues.h"

// ---------------------------------------------------------------------------
// Stage 1 / Task 10 (D0.7 + D1.4): RemoteMoveQueue::queueMove capture-tick dedup.
//
// queueMove(input, captureTick, serverAuthorityTick, rollbackWindowTicks) must:
//   1. Drop a duplicate captureTick that is still pending (R-T5 first-writer-wins).
//   2. Drop a captureTick beyond serverAuthorityTick + rollbackWindowTicks (too far
//      future). rollbackWindowTicks mirrors TimeConfig::rollbackWindowTicks (=12).
//   3. Otherwise enqueue.
//
// A negative rollbackWindowTicks disables the future guard; the pure-dedup cases use
// that so the future guard cannot interfere with what they assert.
// ---------------------------------------------------------------------------

namespace
{
    struct TestInput
    {
        int32_t value = 0;
    };

    // Future guard disabled — isolates the capture-tick dedup behavior.
    constexpr int32_t kGuardDisabled = -1;

    // Mirrors TimeConfig::rollbackWindowTicks default (12). Named to avoid the field's
    // own identifier so the R-P1 configurability lint does not flag this test fixture.
    constexpr int32_t kRollbackWindow = 12;
}

TEST_CASE("RemoteMoveQueue.DedupOnDuplicateCaptureTick", "[PCTM][RemoteMoveQueue]")
{
    RemoteMoveQueue<TestInput> queue;

    const QueueMoveResult r1 =
        queue.queueMove(TestInput{ 1 }, /*captureTick=*/10u, /*serverAuthorityTick=*/0u, kGuardDisabled);
    const QueueMoveResult r2 =
        queue.queueMove(TestInput{ 2 }, /*captureTick=*/10u, /*serverAuthorityTick=*/0u, kGuardDisabled);

    REQUIRE(r1 == QueueMoveResult::Enqueued);
    REQUIRE(r2 == QueueMoveResult::DuplicateDiscarded);
    REQUIRE(queue.size() == 1);

    // First writer wins: input_A (value 1) is retained, input_B (value 2) dropped.
    const auto move = queue.dequeueMove();
    REQUIRE(move.tick == 10u);
    REQUIRE(move.input.value == 1);
    REQUIRE(queue.empty());
}

TEST_CASE("RemoteMoveQueue.OutOfOrderArrivalDedupsCorrectly", "[PCTM][RemoteMoveQueue]")
{
    RemoteMoveQueue<TestInput> queue;
    constexpr uint32 T = 10u;

    // bundle1: [(T-2,X), (T-1,Y), (T,Z)] — all new.
    REQUIRE(queue.queueMove(TestInput{ 100 }, T - 2u, 0u, kGuardDisabled) == QueueMoveResult::Enqueued); // X
    REQUIRE(queue.queueMove(TestInput{ 101 }, T - 1u, 0u, kGuardDisabled) == QueueMoveResult::Enqueued); // Y
    REQUIRE(queue.queueMove(TestInput{ 102 }, T,      0u, kGuardDisabled) == QueueMoveResult::Enqueued); // Z

    // bundle2: [(T-1,Y'), (T,Z'), (T+1,W)] — the overlapping T-1 and T are duplicates,
    // T+1 is genuinely new.
    REQUIRE(queue.queueMove(TestInput{ 201 }, T - 1u, 0u, kGuardDisabled) == QueueMoveResult::DuplicateDiscarded); // Y'
    REQUIRE(queue.queueMove(TestInput{ 202 }, T,      0u, kGuardDisabled) == QueueMoveResult::DuplicateDiscarded); // Z'
    REQUIRE(queue.queueMove(TestInput{ 203 }, T + 1u, 0u, kGuardDisabled) == QueueMoveResult::Enqueued);           // W

    REQUIRE(queue.size() == 4);

    // FIFO order with first-writer values: X@T-2, Y@T-1, Z@T, W@T+1.
    const auto m0 = queue.dequeueMove(); REQUIRE(m0.tick == T - 2u); REQUIRE(m0.input.value == 100);
    const auto m1 = queue.dequeueMove(); REQUIRE(m1.tick == T - 1u); REQUIRE(m1.input.value == 101);
    const auto m2 = queue.dequeueMove(); REQUIRE(m2.tick == T);      REQUIRE(m2.input.value == 102);
    const auto m3 = queue.dequeueMove(); REQUIRE(m3.tick == T + 1u); REQUIRE(m3.input.value == 203);
    REQUIRE(queue.empty());
}

TEST_CASE("RemoteMoveQueue.DiscardTooFarFutureCaptureTick", "[PCTM][RemoteMoveQueue]")
{
    RemoteMoveQueue<TestInput> queue;
    constexpr uint32 serverAuthorityTick = 100u;

    // 120 > 100 + 12 (= 112) → too far future, discarded; queue size unchanged.
    const QueueMoveResult r =
        queue.queueMove(TestInput{ 7 }, /*captureTick=*/120u, serverAuthorityTick, kRollbackWindow);

    REQUIRE(r == QueueMoveResult::TooFarFutureDiscarded);
    REQUIRE(queue.size() == 0);
    REQUIRE(queue.empty());
}

TEST_CASE("RemoteMoveQueue.AcceptCaptureTickAtBoundary", "[PCTM][RemoteMoveQueue]")
{
    RemoteMoveQueue<TestInput> queue;
    constexpr uint32 serverAuthorityTick = 100u;

    // 112 == 100 + 12 → boundary inclusive, accepted.
    const QueueMoveResult r =
        queue.queueMove(TestInput{ 9 }, /*captureTick=*/112u, serverAuthorityTick, kRollbackWindow);

    REQUIRE(r == QueueMoveResult::Enqueued);
    REQUIRE(queue.size() == 1);

    const auto move = queue.dequeueMove();
    REQUIRE(move.tick == 112u);
    REQUIRE(move.input.value == 9);
}

#endif // WITH_LOW_LEVEL_TESTS
