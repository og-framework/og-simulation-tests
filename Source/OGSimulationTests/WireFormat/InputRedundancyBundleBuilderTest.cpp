// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/SimulationFieldDescriptors.h"

#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// Stage 3 / Task 3 (D3.11): FInputRedundancyBundle append-only invariant.
//
// R-T5 states that inputs written into a bundle are append-only and immutable
// per capture-tick: a producer must never revise the input value for a
// capture-tick it has already emitted. The server's receive path
// (forEachSlot -> RemoteMoveQueue::queueMove) is first-writer-wins and cannot
// distinguish a legitimate redundancy re-send from a revision, so a violating
// producer loses the corrected value with no diagnostic.
//
// Enforcement has two build-config-dependent layers, both exercised here:
//
//   1. CHECKED builds — appendSlot() OG_CHECK-fails on a duplicate capture_tick.
//   2. SHIPPING builds — the OG_CHECK compiles out and the duplicate is SILENTLY
//      DROPPED, preserving the first-arrival input. Never overwritten.
//
// TESTING THE CHECKED LAYER: OG_CHECK maps to assert / checkf, which ABORT — they
// do not throw. This suite has no death-test convention (no REQUIRE_THROWS
// anywhere), matching the note in WireFormat_Bundle.cpp. So the checked layer is
// verified through the PREDICATE the OG_CHECK evaluates (containsCaptureTick),
// which is exactly what flips a second append from legal to a hard failure. The
// aborting call is intentionally NOT executed.
//
// TESTING THE SHIPPING LAYER: the drop decision lives in tryAppendSlot(), an
// always-compiled kernel that appendSlot() wraps with the OG_CHECK. Calling the
// kernel directly reproduces the exact shipping-build code path from within this
// checked-build test binary — no shipping-configured test target required.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // Trivial Serializable test input, mirroring WireFormat_Bundle.cpp so both
    // files exercise the codec against the same fixed-stride input shape.
    struct BuilderTestInput
    {
        std::int32_t value = 0;
    };
} // namespace

template <>
struct SerializableFields<BuilderTestInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(BuilderTestInput, value));
    }
};

namespace
{
    // UE-free byte buffer satisfying the codec's "BUFFER CONCEPT" — the same
    // adapter surface FInputRedundancyBundle exposes over its TArray<uint8>.
    struct BuilderTestBuffer
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

    // Reads back every (tick, value) pair currently in the bundle.
    std::vector<std::pair<std::uint32_t, std::int32_t>> readSlots(const BuilderTestBuffer& buf)
    {
        std::vector<std::pair<std::uint32_t, std::int32_t>> out;
        inputRedundancyBundle::forEachSlot<BuilderTestInput>(
            buf, [&](std::uint32_t tick, const BuilderTestInput& in)
            {
                out.emplace_back(tick, in.value);
            });
        return out;
    }
} // namespace

TEST_CASE("Bundle.RevisionAttemptTrapsInCheckedBuild", "[WireFormat][Bundle]")
{
    // CHECKED-BUILD LAYER. After tick 100 is emitted, containsCaptureTick(100) is
    // true — which is precisely the condition appendSlot()'s OG_CHECK negates, so
    // a second appendSlot(100, ...) would abort the process and surface the
    // producer bug at its origin. A fresh tick leaves the predicate false and the
    // append legal. (The aborting call is deliberately not executed.)
    BuilderTestBuffer bundle;
    REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 100u, BuilderTestInput{ 7 }));

    REQUIRE(inputRedundancyBundle::slotCount(bundle) == 1u);
    REQUIRE(inputRedundancyBundle::containsCaptureTick<BuilderTestInput>(bundle, 100u));
    REQUIRE_FALSE(inputRedundancyBundle::containsCaptureTick<BuilderTestInput>(bundle, 101u));
}

TEST_CASE("Bundle.RevisionSilentlyDroppedFirstArrivalWins", "[WireFormat][Bundle]")
{
    // SHIPPING-BUILD LAYER — the locked D3.11 semantics. Write tick 100 = 7, then
    // attempt to REVISE tick 100 to 999. The revision must be rejected and the
    // first-arrival value preserved; the bundle must not grow a second slot for
    // the same capture tick.
    BuilderTestBuffer bundle;
    REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 100u, BuilderTestInput{ 7 }));

    const std::vector<std::uint8_t> bytesBeforeRevision = bundle.bytes;

    REQUIRE_FALSE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 100u, BuilderTestInput{ 999 }));

    // The drop is total: not merely "value unchanged" but the buffer byte-for-byte
    // untouched, so no partial write or stray zeroed stride leaks onto the wire.
    REQUIRE(bundle.bytes == bytesBeforeRevision);

    REQUIRE(inputRedundancyBundle::slotCount(bundle) == 1u);
    const auto slots = readSlots(bundle);
    REQUIRE(slots.size() == 1u);
    REQUIRE(slots[0] == std::make_pair<std::uint32_t, std::int32_t>(100u, 7)); // first arrival, NOT 999
}

TEST_CASE("Bundle.DropDoesNotBlockSubsequentFreshTicks", "[WireFormat][Bundle]")
{
    // A dropped revision must not wedge the bundle: the producer keeps appending
    // new capture ticks normally afterwards, and slot ordering stays arrival-order.
    BuilderTestBuffer bundle;
    REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 100u, BuilderTestInput{ 1 }));
    REQUIRE_FALSE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 100u, BuilderTestInput{ 42 }));
    REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 101u, BuilderTestInput{ 2 }));
    REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 102u, BuilderTestInput{ 3 }));

    REQUIRE(inputRedundancyBundle::slotCount(bundle) == 3u);

    const auto slots = readSlots(bundle);
    REQUIRE(slots.size() == 3u);
    REQUIRE(slots[0] == std::make_pair<std::uint32_t, std::int32_t>(100u, 1));
    REQUIRE(slots[1] == std::make_pair<std::uint32_t, std::int32_t>(101u, 2));
    REQUIRE(slots[2] == std::make_pair<std::uint32_t, std::int32_t>(102u, 3));
}

TEST_CASE("Bundle.HeaderIntactAfterDroppedRevision", "[WireFormat][Bundle]")
{
    // The drop path returns before touching the header. Version byte and slot
    // count must both survive, so a dropped revision cannot corrupt the compat
    // fence (Task 11) or desynchronise the receiver's slot-scan bound.
    BuilderTestBuffer bundle;
    REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 200u, BuilderTestInput{ 5 }));
    REQUIRE_FALSE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 200u, BuilderTestInput{ 6 }));

    REQUIRE(inputRedundancyBundle::getWireFormatVersion(bundle)
        == inputRedundancyBundle::kWireFormatVersion);
    REQUIRE(inputRedundancyBundle::slotCount(bundle) == 1u);
}

TEST_CASE("Bundle.DropAppliesToEveryOccupiedTickNotJustTheLast", "[WireFormat][Bundle]")
{
    // Guards against an implementation that only compares against the most
    // recently appended tick: revising an OLDER occupied slot must drop too.
    BuilderTestBuffer bundle;
    REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 300u, BuilderTestInput{ 1 }));
    REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 301u, BuilderTestInput{ 2 }));
    REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 302u, BuilderTestInput{ 3 }));

    // Revise the OLDEST slot, not the newest.
    REQUIRE_FALSE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(bundle, 300u, BuilderTestInput{ 77 }));

    REQUIRE(inputRedundancyBundle::slotCount(bundle) == 3u);
    const auto slots = readSlots(bundle);
    REQUIRE(slots[0] == std::make_pair<std::uint32_t, std::int32_t>(300u, 1)); // preserved
}

//////////////////////////////////////////////////////////////////////////////
// Task 16: kMaxSlots wire-budget bound, enforced in EVERY build config.
//
// Same mechanism and same rationale as the R-T5 cases above: the bound check
// lives in the always-compiled tryAppendSlot() kernel, so calling the kernel
// directly from this checked-build binary reproduces the exact shipping code
// path. Before T16 the bound was OG_CHECK-only, so a shipping build appended a
// 9th slot and produced a bounded-but-malformed over-budget payload (the
// TArray-backed storage grows, so this was never an out-of-bounds write).
//
// This is defence in depth, not a live production fix: buildRedundancyBundle
// already clamps depth to min(redundancyDepthTicks, kMaxSlots), so the overflow
// is unreachable through the current call graph. These cases pin the kernel's
// behavior for a future caller that skips that clamp.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // Appends ticks 1..count, requiring every append to succeed.
    void fillSlots(BuilderTestBuffer& buf, std::uint8_t count)
    {
        for (std::uint8_t i = 0; i < count; ++i)
        {
            REQUIRE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(
                buf, static_cast<std::uint32_t>(i + 1), BuilderTestInput{ static_cast<std::int32_t>(i + 1) }));
        }
    }
} // namespace

TEST_CASE("Bundle.FillsToExactlyMaxSlots", "[WireFormat][Bundle]")
{
    // Boundary guard against an off-by-one that would reject the LAST legal slot:
    // the kMaxSlots'th append must still succeed.
    BuilderTestBuffer bundle;
    fillSlots(bundle, inputRedundancyBundle::kMaxSlots);

    REQUIRE(inputRedundancyBundle::slotCount(bundle) == inputRedundancyBundle::kMaxSlots);
    REQUIRE(inputRedundancyBundle::isFull(bundle));
    REQUIRE(readSlots(bundle).size() == static_cast<std::size_t>(inputRedundancyBundle::kMaxSlots));
}

TEST_CASE("Bundle.OverflowAppendDroppedBufferUntouched", "[WireFormat][Bundle]")
{
    // THE T16 CASE. With the bundle at kMaxSlots (8), a 9th append of a FRESH
    // capture tick — so the R-T5 duplicate guard cannot be what rejects it — must
    // be dropped by the always-compiled kernel, leaving the payload byte-for-byte
    // untouched rather than emitting a 9th slot.
    BuilderTestBuffer bundle;
    fillSlots(bundle, inputRedundancyBundle::kMaxSlots);

    const std::vector<std::uint8_t> bytesBeforeOverflow = bundle.bytes;
    const std::uint32_t freshTick = static_cast<std::uint32_t>(inputRedundancyBundle::kMaxSlots) + 1u;
    REQUIRE_FALSE(inputRedundancyBundle::containsCaptureTick<BuilderTestInput>(bundle, freshTick));

    REQUIRE_FALSE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(
        bundle, freshTick, BuilderTestInput{ 999 }));

    REQUIRE(bundle.bytes == bytesBeforeOverflow);
    REQUIRE(inputRedundancyBundle::slotCount(bundle) == inputRedundancyBundle::kMaxSlots);
    REQUIRE_FALSE(inputRedundancyBundle::containsCaptureTick<BuilderTestInput>(bundle, freshTick));

    // Header survives, so the compat fence and the receiver's slot-scan bound are
    // both unaffected by a dropped overflow.
    REQUIRE(inputRedundancyBundle::getWireFormatVersion(bundle)
        == inputRedundancyBundle::kWireFormatVersion);

    // Every originally-appended slot is still readable and correct.
    const auto slots = readSlots(bundle);
    REQUIRE(slots.size() == static_cast<std::size_t>(inputRedundancyBundle::kMaxSlots));
    REQUIRE(slots.front() == std::make_pair<std::uint32_t, std::int32_t>(1u, 1));
    REQUIRE(slots.back() == std::make_pair<std::uint32_t, std::int32_t>(
        static_cast<std::uint32_t>(inputRedundancyBundle::kMaxSlots),
        static_cast<std::int32_t>(inputRedundancyBundle::kMaxSlots)));
}

TEST_CASE("Bundle.OverflowStaysDroppedOnRepeatedAttempts", "[WireFormat][Bundle]")
{
    // A full bundle must stay full: repeated overflow attempts must not creep the
    // slot count upward one byte at a time, and must not corrupt the payload.
    BuilderTestBuffer bundle;
    fillSlots(bundle, inputRedundancyBundle::kMaxSlots);

    const std::vector<std::uint8_t> bytesBeforeOverflow = bundle.bytes;
    for (std::uint32_t i = 0; i < 4u; ++i)
    {
        REQUIRE_FALSE(inputRedundancyBundle::tryAppendSlot<BuilderTestInput>(
            bundle, 1000u + i, BuilderTestInput{ static_cast<std::int32_t>(i) }));
    }

    REQUIRE(bundle.bytes == bytesBeforeOverflow);
    REQUIRE(inputRedundancyBundle::slotCount(bundle) == inputRedundancyBundle::kMaxSlots);
}

#endif // WITH_LOW_LEVEL_TESTS
