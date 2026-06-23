// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationQueues.h"

#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// Stage 1 / Task 12: FInputRedundancyBundle wire-type regression suite.
//
// These exercise the ENGINE-AGNOSTIC bundle codec
// (OGSimulation/InputRedundancyBundleCodec.h) — the exact append / dedup /
// iterate / version logic that the UE-side FInputRedundancyBundle USTRUCT
// delegates to. Running against the codec (not the USTRUCT) lets these tests
// live in the pure-C++ Low-Level-Tests target, which deliberately does NOT link
// the engine-coupled OGSimulationUnreal module (bCompileAgainstEngine = false).
//
// DEFERRED (UE-coupled, not in this file): the watermark-trim NetSerialize +
// version-byte-offset regressions for FSimulationStateSyncBuffer /
// FSimulationInputSyncBuffer (Backlog T12 "WireFormat_Buffers.cpp"). Those need
// FArchive + the USTRUCT NetSerialize, which require an engine-coupled LLT
// target that does not exist yet (docs/low-level-tests.md "Future: testing
// UE-coupled code"). Deferred per user direction 2026-06-22; see impl notes.
//
// NOTE on R-T5 / kMaxSlots enforcement: appendSlot() hard-fails via OG_CHECK
// (→ assert / checkf, which ABORT — they do not throw). The suite has no
// death-test convention (no REQUIRE_THROWS anywhere), so the two
// invariant tests verify the GUARD PREDICATES the OG_CHECKs evaluate
// (containsCaptureTick / isFull / slotCount) rather than tripping the abort.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // Trivial Serializable test input: a single int32 field. The
    // SerializableFields specialization makes it satisfy the Serializable
    // concept so the codec serializes it via syncSize / writeToSyncedBuffer
    // (syncSize<TestInput>() == 4), exactly like a brawler PlayerInput field.
    struct TestInput
    {
        std::int32_t value = 0;
    };
} // namespace

template <>
struct SerializableFields<TestInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(TestInput, value));
    }
};

namespace
{
    // UE-free byte buffer satisfying the codec's "BUFFER CONCEPT". Mirrors what
    // the FInputRedundancyBundle USTRUCT supplies (TArray-backed) without any
    // engine dependency.
    struct TestBundleBuffer
    {
        std::vector<std::uint8_t> bytes;

        std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

        void bundleAddZeroedBytes(std::int32_t count)
        {
            bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u);
        }

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        {
            std::memcpy(bytes.data() + off, &value, sizeof(T));
        }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        {
            T value;
            std::memcpy(&value, bytes.data() + off, sizeof(T));
            return value;
        }
    };

    // Future guard disabled — isolates the bundle/dedup behavior from the
    // too-far-future guard (which has its own dedicated tests in
    // RemoteMoveQueueDedupTest.cpp, Task 10).
    constexpr std::int32_t kGuardDisabled = -1;

    // Drains a RemoteMoveQueue into a flat list of (tick, value) for assertions.
    std::vector<std::pair<std::uint32_t, std::int32_t>> drain(RemoteMoveQueue<TestInput>& queue)
    {
        std::vector<std::pair<std::uint32_t, std::int32_t>> out;
        while (!queue.empty())
        {
            const auto move = queue.dequeueMove();
            out.emplace_back(move.tick, move.input.value);
        }
        return out;
    }

    // Feeds every slot of a bundle into the queue, mirroring the server's
    // ServerReceiveRemoteMove → forEachSlot → queueMove path (Task 9 + Task 10).
    void deliverBundle(const TestBundleBuffer& bundle, RemoteMoveQueue<TestInput>& queue)
    {
        inputRedundancyBundle::forEachSlot<TestInput>(
            bundle, [&](std::uint32_t tick, const TestInput& in)
            {
                queue.queueMove(TestInput{ in.value }, tick, /*serverAuthorityTick=*/0u, kGuardDisabled);
            });
    }
} // namespace

TEST_CASE("Bundle.RoundTripSerialization", "[WireFormat][Bundle]")
{
    // Build a 3-slot bundle, copy its raw wire bytes (as a replicated bundle
    // would arrive), and confirm all three slots deserialize identically.
    TestBundleBuffer sender;
    inputRedundancyBundle::appendSlot<TestInput>(sender, 100u, TestInput{ 11 });
    inputRedundancyBundle::appendSlot<TestInput>(sender, 101u, TestInput{ 22 });
    inputRedundancyBundle::appendSlot<TestInput>(sender, 102u, TestInput{ 33 });

    REQUIRE(inputRedundancyBundle::getWireFormatVersion(sender)
        == inputRedundancyBundle::kWireFormatVersion);
    REQUIRE(inputRedundancyBundle::slotCount(sender) == 3u);

    // Simulate the wire hop: the receiver gets only the serialized bytes.
    TestBundleBuffer receiver;
    receiver.bytes = sender.bytes;

    std::vector<std::pair<std::uint32_t, std::int32_t>> recovered;
    inputRedundancyBundle::forEachSlot<TestInput>(
        receiver, [&](std::uint32_t tick, const TestInput& in)
        {
            recovered.emplace_back(tick, in.value);
        });

    REQUIRE(recovered.size() == 3u);
    REQUIRE(recovered[0] == std::make_pair<std::uint32_t, std::int32_t>(100u, 11));
    REQUIRE(recovered[1] == std::make_pair<std::uint32_t, std::int32_t>(101u, 22));
    REQUIRE(recovered[2] == std::make_pair<std::uint32_t, std::int32_t>(102u, 33));
}

TEST_CASE("Bundle.DedupByCaptureTickOnReceive", "[WireFormat][Bundle]")
{
    // Two overlapping bundles delivered in order. The server-receive path
    // (forEachSlot → RemoteMoveQueue::queueMove) must keep first-writer-wins per
    // capture tick (R-T5), so the second bundle's overlapping ticks are dropped.
    TestBundleBuffer bundle1;
    inputRedundancyBundle::appendSlot<TestInput>(bundle1, 100u, TestInput{ 1 }); // X
    inputRedundancyBundle::appendSlot<TestInput>(bundle1, 101u, TestInput{ 2 }); // Y
    inputRedundancyBundle::appendSlot<TestInput>(bundle1, 102u, TestInput{ 3 }); // Z

    TestBundleBuffer bundle2;
    inputRedundancyBundle::appendSlot<TestInput>(bundle2, 101u, TestInput{ 22 }); // Y' (dup)
    inputRedundancyBundle::appendSlot<TestInput>(bundle2, 102u, TestInput{ 33 }); // Z' (dup)
    inputRedundancyBundle::appendSlot<TestInput>(bundle2, 103u, TestInput{ 4 });  // W  (new)

    RemoteMoveQueue<TestInput> queue;
    deliverBundle(bundle1, queue);
    deliverBundle(bundle2, queue);

    const auto moves = drain(queue);
    REQUIRE(moves.size() == 4u);
    REQUIRE(moves[0] == std::make_pair<std::uint32_t, std::int32_t>(100u, 1)); // X
    REQUIRE(moves[1] == std::make_pair<std::uint32_t, std::int32_t>(101u, 2)); // Y (first writer)
    REQUIRE(moves[2] == std::make_pair<std::uint32_t, std::int32_t>(102u, 3)); // Z (first writer)
    REQUIRE(moves[3] == std::make_pair<std::uint32_t, std::int32_t>(103u, 4)); // W
}

TEST_CASE("Bundle.PacketLossRecovery", "[WireFormat][Bundle]")
{
    // bundle1 is dropped entirely (lost UDP datagram). bundle2 carries the
    // redundancy overlap [101, 102] plus the new 103, so the queue self-heals
    // 101 and 102 from bundle2's redundancy slots. Tick 100 is a genuine loss
    // (outside bundle2's window) and is NOT recoverable here — correction-state
    // broadcast handles that case (out of scope for the input channel).
    TestBundleBuffer bundle2;
    inputRedundancyBundle::appendSlot<TestInput>(bundle2, 101u, TestInput{ 2 });
    inputRedundancyBundle::appendSlot<TestInput>(bundle2, 102u, TestInput{ 3 });
    inputRedundancyBundle::appendSlot<TestInput>(bundle2, 103u, TestInput{ 4 });

    RemoteMoveQueue<TestInput> queue;
    // bundle1 never delivered.
    deliverBundle(bundle2, queue);

    const auto moves = drain(queue);
    REQUIRE(moves.size() == 3u);
    REQUIRE(moves[0] == std::make_pair<std::uint32_t, std::int32_t>(101u, 2));
    REQUIRE(moves[1] == std::make_pair<std::uint32_t, std::int32_t>(102u, 3));
    REQUIRE(moves[2] == std::make_pair<std::uint32_t, std::int32_t>(103u, 4));

    // The genuinely-lost tick 100 is absent.
    for (const auto& m : moves)
        REQUIRE(m.first != 100u);
}

TEST_CASE("Bundle.AppendSlotInvariantViolation", "[WireFormat][Bundle]")
{
    // R-T5: a capture_tick may occupy at most one slot. appendSlot() OG_CHECK-
    // fails (aborts) on a duplicate; the suite has no death-test facility, so we
    // verify the predicate the OG_CHECK guards on — containsCaptureTick — which
    // is exactly what flips a second appendSlot(100, ...) from legal to a hard
    // failure. (Re-appending 100 would abort the process; intentionally NOT
    // executed here.)
    TestBundleBuffer bundle;
    inputRedundancyBundle::appendSlot<TestInput>(bundle, 100u, TestInput{ 7 });

    REQUIRE(inputRedundancyBundle::slotCount(bundle) == 1u);
    REQUIRE(inputRedundancyBundle::containsCaptureTick<TestInput>(bundle, 100u));      // dup → would OG_CHECK
    REQUIRE_FALSE(inputRedundancyBundle::containsCaptureTick<TestInput>(bundle, 101u)); // fresh → legal
}

TEST_CASE("Bundle.RespectsMaxSlots", "[WireFormat][Bundle]")
{
    // Wire-safety bound: kMaxSlots (8) slots fill the bundle; a 9th appendSlot
    // would OG_CHECK-fail (abort). Verify the boundary predicate (isFull /
    // slotCount) the overflow OG_CHECK guards on — the 9th append is
    // intentionally NOT executed (no death-test facility).
    TestBundleBuffer bundle;
    for (std::uint32_t i = 0; i < inputRedundancyBundle::kMaxSlots; ++i)
    {
        REQUIRE_FALSE(inputRedundancyBundle::isFull(bundle)); // room before each of the 8 appends
        inputRedundancyBundle::appendSlot<TestInput>(bundle, 200u + i, TestInput{ static_cast<std::int32_t>(i) });
    }

    REQUIRE(inputRedundancyBundle::slotCount(bundle) == inputRedundancyBundle::kMaxSlots);
    REQUIRE(inputRedundancyBundle::isFull(bundle)); // a 9th appendSlot would OG_CHECK-abort
}

#endif // WITH_LOW_LEVEL_TESTS
