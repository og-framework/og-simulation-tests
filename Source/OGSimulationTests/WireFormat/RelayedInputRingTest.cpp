// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"

#include <cstdint>
#include <cstring>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T1: the OUTBOUND relay payload — a replace-latest,
// depth-configurable ring of (captureTick, dA, input) entries.
//
// WHAT THESE TESTS EXIST TO PIN (fable finding B2). The inbound
// FInputRedundancyBundle is append-only and immutable per capture tick: it
// OG_CHECK-fails on a duplicate and hard-caps at kMaxSlots = 8. The outbound
// relay is a PERSISTENT replicated property written again on every newer capture
// tick, so those two properties would make it overflow and assert within 8 ticks.
// This ring is therefore the OPPOSITE contract, and each half of that opposition
// is asserted below:
//
//   * REPLACE, never append — a newer capture tick supersedes the OLDEST entry
//     once the ring is at depth, so the entry count never exceeds depth no matter
//     how long the session runs (RelayRing.NeverGrowsBeyondDepth).
//   * REVISION IS LEGAL — rewriting a resident capture tick updates it in place
//     instead of asserting (RelayRing.ReplaceRoundTripCarriesTheStamp).
//
// The dA SCHEDULE STAMP (RelayDelaySpectrumDesign.md §5) rides every entry, and
// the APPLICATION TICK is derived from it (`captureTick + dA`), never stored —
// so the round-trip tests assert the stamp explicitly and the derivation is
// checked against its two operands rather than against a stored field.
//
// These run against the ENGINE-AGNOSTIC codec, which is the same code the UE
// USTRUCT (FRelayedInputRing) delegates to — a std::vector-backed buffer here
// versus its TArray-backed one, per the codec's BUFFER CONCEPT. The USTRUCT's
// UE-only half (NetSerialize) is not reachable from this target and follows the
// existing deferral for engine-coupled wire tests (see WireFormat_Bundle.cpp /
// docs/low-level-tests.md "Future: testing UE-coupled code").
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // Trivial Serializable test input, mirroring the bundle test files so every
    // WireFormat test exercises the codecs against the same fixed-stride shape.
    struct RingTestInput
    {
        std::int32_t value = 0;
    };

    // Second Serializable part, used only to build a SimulationComposite input —
    // production's simulatableBrawler::PlayerInput is a composite, and the codec
    // dispatches composites down a different arm than plain Serializables.
    struct RingTestPart
    {
        float scale = 0.f;
    };
} // namespace

template <>
struct SerializableFields<RingTestInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(RingTestInput, value));
    }
};

template <>
struct SerializableFields<RingTestPart>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(RingTestPart, scale));
    }
};

namespace
{
    // UE-free byte buffer satisfying the codec's BUFFER CONCEPT — the same four
    // methods FRelayedInputRing exposes over its TArray<uint8>.
    struct RingTestBuffer
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

        // [T34] The FIFTH concept method, required only by the flush path
        // (`resetEntries` / `flushStagedInto`). Mirrors FRelayedInputRing's
        // TArray-backed one, including its shrink-only semantics: a request above
        // the current size is ignored, because every caller is shrinking and a
        // silent grow would hand the codec uninitialized bytes. `std::vector::
        // resize` down keeps capacity, which is the std equivalent of
        // `EAllowShrinking::No`.
        void bundleTruncateTo(std::int32_t byteCount)
        {
            if (byteCount < 0)
                byteCount = 0;
            if (static_cast<std::size_t>(byteCount) < bytes.size())
                bytes.resize(static_cast<std::size_t>(byteCount));
        }
    };

    struct ReadEntry
    {
        std::uint32_t captureTick = 0;
        std::uint8_t  dA          = 0;
        std::int32_t  value       = 0;
    };

    // Reads back every resident entry, in ring-position order.
    std::vector<ReadEntry> readEntries(const RingTestBuffer& ring)
    {
        std::vector<ReadEntry> out;
        relayedInputRing::forEachEntry<RingTestInput>(
            ring, [&](std::uint32_t captureTick, std::uint8_t dA, const RingTestInput& in)
            {
                out.push_back(ReadEntry{ captureTick, dA, in.value });
            });
        return out;
    }

    // True when SOME resident entry carries this capture tick with this stamp and
    // value. Ring position is deliberately not asserted anywhere — entries live at
    // ring slots, not in age order, so a position-sensitive assertion would pin
    // an implementation detail the consumers must never rely on.
    bool hasEntry(const RingTestBuffer& ring,
                  std::uint32_t captureTick,
                  std::uint8_t dA,
                  std::int32_t value)
    {
        for (const ReadEntry& e : readEntries(ring))
        {
            if (e.captureTick == captureTick && e.dA == dA && e.value == value)
                return true;
        }
        return false;
    }

    bool hasCaptureTick(const RingTestBuffer& ring, std::uint32_t captureTick)
    {
        return relayedInputRing::containsCaptureTick<RingTestInput>(ring, captureTick);
    }

    constexpr std::int32_t kDepth1 = 1;
} // namespace

// ---------------------------------------------------------------------------
// Replace round-trip, INCLUDING the dA schedule stamp — the headline case.
// ---------------------------------------------------------------------------
TEST_CASE("RelayRing.ReplaceRoundTripCarriesTheStamp", "[WireFormat][RelayRing]")
{
    RingTestBuffer ring;

    // A never-written ring costs nothing on the wire and reads as version 0, so it
    // can never be mistaken for a version mismatch.
    REQUIRE(ring.bundleByteNum() == 0);
    REQUIRE(relayedInputRing::entryCount(ring) == 0u);
    REQUIRE(relayedInputRing::getWireFormatVersion(ring) == 0u);

    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 100u, 4u, RingTestInput{ 7 }, kDepth1));

    REQUIRE(relayedInputRing::getWireFormatVersion(ring) == relayedInputRing::kWireFormatVersion);
    REQUIRE(relayedInputRing::entryCount(ring) == 1u);

    // Round-trip by capture-tick lookup: the value AND the stamp survive.
    std::uint8_t  dA = 0;
    RingTestInput value{};
    REQUIRE(relayedInputRing::findEntry<RingTestInput>(ring, 100u, dA, value));
    REQUIRE(dA == 4u);
    REQUIRE(value.value == 7);

    // The APPLICATION tick is derived from the pair, never stored.
    REQUIRE(relayedInputRing::applicationTick(100u, dA) == 104u);

    // A newer capture tick REPLACES at depth 1 — the ring does not grow, and the
    // superseded entry is gone (a lookup for it now misses).
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 101u, 4u, RingTestInput{ 9 }, kDepth1));
    REQUIRE(relayedInputRing::entryCount(ring) == 1u);
    REQUIRE_FALSE(relayedInputRing::findEntry<RingTestInput>(ring, 100u, dA, value));
    REQUIRE(relayedInputRing::findEntry<RingTestInput>(ring, 101u, dA, value));
    REQUIRE(dA == 4u);
    REQUIRE(value.value == 9);

    // A REVISION of the resident capture tick is legal and updates in place — the
    // exact operation FInputRedundancyBundle::appendSlot OG_CHECK-fails on. This
    // is what a re-stamp after a tier change does: same capture tick, new dA.
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 101u, 2u, RingTestInput{ 11 }, kDepth1));
    REQUIRE(relayedInputRing::entryCount(ring) == 1u);
    REQUIRE(relayedInputRing::findEntry<RingTestInput>(ring, 101u, dA, value));
    REQUIRE(dA == 2u);
    REQUIRE(value.value == 11);
    REQUIRE(relayedInputRing::applicationTick(101u, dA) == 103u);
}

// ---------------------------------------------------------------------------
// The stamp is PER ENTRY, not per ring: a dA change mid-session leaves older
// entries stamped with the schedule they were actually relayed under.
// ---------------------------------------------------------------------------
TEST_CASE("RelayRing.StampIsPerEntryNotPerRing", "[WireFormat][RelayRing]")
{
    RingTestBuffer ring;
    constexpr std::int32_t depth = 3;

    // A tier change between the second and third relay moves dA from 2 to 5.
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 50u, 2u, RingTestInput{ 1 }, depth));
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 51u, 2u, RingTestInput{ 2 }, depth));
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 52u, 5u, RingTestInput{ 3 }, depth));

    std::uint8_t  dA = 0;
    RingTestInput value{};

    REQUIRE(relayedInputRing::findEntry<RingTestInput>(ring, 50u, dA, value));
    REQUIRE(dA == 2u);
    REQUIRE(relayedInputRing::applicationTick(50u, dA) == 52u);

    REQUIRE(relayedInputRing::findEntry<RingTestInput>(ring, 52u, dA, value));
    REQUIRE(dA == 5u);
    REQUIRE(relayedInputRing::applicationTick(52u, dA) == 57u);
}

// ---------------------------------------------------------------------------
// Depth-N shape: the ring FILLS to depth, then stops growing.
// ---------------------------------------------------------------------------
TEST_CASE("RelayRing.DepthNShapeFillsThenHolds", "[WireFormat][RelayRing]")
{
    RingTestBuffer ring;
    constexpr std::int32_t depth = 4;

    // Loop bodies accumulate rather than REQUIRE per iteration: a per-iteration
    // assertion would add hundreds of assertions to the suite's headline count for
    // no extra coverage (the counts are diffed release-to-release).
    bool allAccepted    = true;
    bool countTrackedIt = true;
    for (std::uint32_t i = 0; i < 4u; ++i)
    {
        allAccepted &= relayedInputRing::writeLatest<RingTestInput>(
            ring, 200u + i, static_cast<std::uint8_t>(i), RingTestInput{ static_cast<std::int32_t>(i) }, depth);
        countTrackedIt &= (relayedInputRing::entryCount(ring) == static_cast<std::uint8_t>(i + 1u));
    }
    REQUIRE(allAccepted);
    REQUIRE(countTrackedIt);

    // All four capture ticks are simultaneously resident, each with its own stamp.
    REQUIRE(hasEntry(ring, 200u, 0u, 0));
    REQUIRE(hasEntry(ring, 201u, 1u, 1));
    REQUIRE(hasEntry(ring, 202u, 2u, 2));
    REQUIRE(hasEntry(ring, 203u, 3u, 3));

    const std::size_t bytesAtDepth = ring.bytes.size();

    // Past depth, the payload stops growing entirely — this is the property that
    // makes a persistent replicated ring safe where the append-only bundle is not.
    for (std::uint32_t i = 4; i < 40u; ++i)
    {
        allAccepted &= relayedInputRing::writeLatest<RingTestInput>(
            ring, 200u + i, 6u, RingTestInput{ static_cast<std::int32_t>(i) }, depth);
    }
    REQUIRE(allAccepted);
    REQUIRE(relayedInputRing::entryCount(ring) == 4u);
    REQUIRE(ring.bytes.size() == bytesAtDepth);
}

// ---------------------------------------------------------------------------
// Newer supersedes OLDER — and only the oldest.
// ---------------------------------------------------------------------------
TEST_CASE("RelayRing.NewerSupersedesOldest", "[WireFormat][RelayRing]")
{
    RingTestBuffer ring;
    constexpr std::int32_t depth = 3;

    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 10u, 1u, RingTestInput{ 100 }, depth));
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 11u, 1u, RingTestInput{ 101 }, depth));
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 12u, 1u, RingTestInput{ 102 }, depth));

    // Tick 13 evicts tick 10 (the oldest) and NOTHING else.
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 13u, 1u, RingTestInput{ 103 }, depth));
    REQUIRE(relayedInputRing::entryCount(ring) == 3u);
    REQUIRE_FALSE(hasCaptureTick(ring, 10u));
    REQUIRE(hasEntry(ring, 11u, 1u, 101));
    REQUIRE(hasEntry(ring, 12u, 1u, 102));
    REQUIRE(hasEntry(ring, 13u, 1u, 103));

    // And again — the window slides one tick at a time, always dropping the oldest.
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 14u, 1u, RingTestInput{ 104 }, depth));
    REQUIRE_FALSE(hasCaptureTick(ring, 11u));
    REQUIRE(hasEntry(ring, 12u, 1u, 102));
    REQUIRE(hasEntry(ring, 13u, 1u, 103));
    REQUIRE(hasEntry(ring, 14u, 1u, 104));
}

// ---------------------------------------------------------------------------
// A stale write never evicts fresher data.
//
// The server's per-id `acceptedNew` watermark means production only ever relays
// strictly newer capture ticks, so this arm is a defensive guard for an
// out-of-order caller — but if it were missing, one late input could silently
// knock a fresh entry out of a peer's ring.
// ---------------------------------------------------------------------------
TEST_CASE("RelayRing.StaleWriteDroppedAtDepthLeavesRingUntouched", "[WireFormat][RelayRing]")
{
    RingTestBuffer ring;
    constexpr std::int32_t depth = 2;

    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 30u, 3u, RingTestInput{ 1 }, depth));
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 31u, 3u, RingTestInput{ 2 }, depth));

    const std::vector<std::uint8_t> bytesBefore = ring.bytes;

    // Tick 29 is older than everything resident: there is nothing staler to evict.
    REQUIRE_FALSE(relayedInputRing::writeLatest<RingTestInput>(ring, 29u, 3u, RingTestInput{ 999 }, depth));

    // The drop is total — byte-for-byte untouched, so no partial write leaks onto
    // the wire and no entry count is bumped for a slot that was never filled.
    REQUIRE(ring.bytes == bytesBefore);
    REQUIRE_FALSE(hasCaptureTick(ring, 29u));

    // An out-of-order tick that is still newer than the OLDEST resident entry is
    // accepted (it supersedes that oldest entry) — "stale" means older than the
    // whole ring, not merely out of order.
    REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 30u, 4u, RingTestInput{ 5 }, depth));
    REQUIRE(hasEntry(ring, 30u, 4u, 5));   // rewritten in place, new stamp
    REQUIRE(relayedInputRing::entryCount(ring) == 2u);
}

// ---------------------------------------------------------------------------
// Depth clamping: a misconfigured depth can never disable the relay or blow the
// wire budget.
// ---------------------------------------------------------------------------
TEST_CASE("RelayRing.DepthClampedToUsableRange", "[WireFormat][RelayRing]")
{
    SECTION("depth 0 behaves as depth 1 rather than dropping every write")
    {
        RingTestBuffer ring;
        REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 1u, 0u, RingTestInput{ 1 }, 0));
        REQUIRE(relayedInputRing::entryCount(ring) == 1u);

        REQUIRE(relayedInputRing::writeLatest<RingTestInput>(ring, 2u, 0u, RingTestInput{ 2 }, 0));
        REQUIRE(relayedInputRing::entryCount(ring) == 1u);
        REQUIRE(hasEntry(ring, 2u, 0u, 2));
    }

    SECTION("depth above kMaxDepth is capped at kMaxDepth")
    {
        RingTestBuffer ring;
        constexpr std::int32_t absurdDepth = 64;

        bool allAccepted = true;
        for (std::uint32_t i = 0; i < 32u; ++i)
        {
            allAccepted &= relayedInputRing::writeLatest<RingTestInput>(
                ring, 500u + i, 1u, RingTestInput{ static_cast<std::int32_t>(i) }, absurdDepth);
        }
        REQUIRE(allAccepted);
        REQUIRE(relayedInputRing::entryCount(ring) == relayedInputRing::kMaxDepth);

        // The survivors are the kMaxDepth most recent capture ticks.
        bool survivorsResident = true;
        for (std::uint32_t i = 32u - relayedInputRing::kMaxDepth; i < 32u; ++i)
            survivorsResident &= hasCaptureTick(ring, 500u + i);
        REQUIRE(survivorsResident);
        REQUIRE_FALSE(hasCaptureTick(ring, 500u + (32u - relayedInputRing::kMaxDepth - 1u)));
    }
}

// ---------------------------------------------------------------------------
// COMPOSITE InputType — the branch production actually takes.
//
// The relay carries simulatableBrawler::PlayerInput, which is a
// SimulationComposite, NOT a plain Serializable. That selects a different arm of
// the codec's entryInputSize / writeInput / readInput dispatch (compositeSyncSize
// + writeCompositeToSyncedBuffer). Every other case in this file exercises the
// plain-Serializable arm, so without this one the shipped configuration would be
// the untested one.
// ---------------------------------------------------------------------------
TEST_CASE("RelayRing.CompositeInputRoundTrip", "[WireFormat][RelayRing]")
{
    using CompositeInput = SimulationComposite<RingTestInput, RingTestPart>;
    constexpr std::int32_t depth = 2;

    RingTestBuffer ring;

    REQUIRE(relayedInputRing::writeLatest<CompositeInput>(
        ring, 900u, 3u, CompositeInput{ RingTestInput{ 42 }, RingTestPart{ 1.5f } }, depth));
    REQUIRE(relayedInputRing::writeLatest<CompositeInput>(
        ring, 901u, 3u, CompositeInput{ RingTestInput{ 43 }, RingTestPart{ 2.5f } }, depth));

    REQUIRE(relayedInputRing::entryCount(ring) == 2u);

    std::uint8_t   dA = 0;
    CompositeInput out{};
    REQUIRE(relayedInputRing::findEntry<CompositeInput>(ring, 900u, dA, out));
    REQUIRE(dA == 3u);
    REQUIRE(out.get<RingTestInput>().value == 42);
    REQUIRE(out.get<RingTestPart>().scale == 1.5f);

    // Replace semantics hold identically for a composite payload: 902 supersedes
    // the oldest (900), and both surviving entries keep their own fields.
    REQUIRE(relayedInputRing::writeLatest<CompositeInput>(
        ring, 902u, 7u, CompositeInput{ RingTestInput{ 44 }, RingTestPart{ 3.5f } }, depth));
    REQUIRE(relayedInputRing::entryCount(ring) == 2u);
    REQUIRE_FALSE(relayedInputRing::findEntry<CompositeInput>(ring, 900u, dA, out));

    REQUIRE(relayedInputRing::findEntry<CompositeInput>(ring, 901u, dA, out));
    REQUIRE(dA == 3u);
    REQUIRE(out.get<RingTestInput>().value == 43);

    REQUIRE(relayedInputRing::findEntry<CompositeInput>(ring, 902u, dA, out));
    REQUIRE(dA == 7u);
    REQUIRE(out.get<RingTestPart>().scale == 3.5f);
    REQUIRE(relayedInputRing::applicationTick(902u, dA) == 909u);
}

// ---------------------------------------------------------------------------
// The depth-1 default shipped by this increment: a long session leaves exactly
// the latest input on the wire, at constant cost.
// ---------------------------------------------------------------------------
TEST_CASE("RelayRing.NeverGrowsBeyondDepth", "[WireFormat][RelayRing]")
{
    RingTestBuffer ring;

    // Well past FInputRedundancyBundle::kMaxSlots (8) — the count where the
    // append-only inbound payload would have overflowed and asserted (fable B2).
    bool allAccepted = true;
    for (std::uint32_t tick = 0; tick < 1000u; ++tick)
    {
        allAccepted &= relayedInputRing::writeLatest<RingTestInput>(
            ring, tick, 1u, RingTestInput{ static_cast<std::int32_t>(tick) }, kDepth1);
    }

    REQUIRE(allAccepted);
    REQUIRE(relayedInputRing::entryCount(ring) == 1u);
    REQUIRE(hasEntry(ring, 999u, 1u, 999));

    // Constant payload size: header + exactly one entry.
    const std::size_t entryStride =
        sizeof(std::uint32_t) + sizeof(std::uint8_t) + syncSize<RingTestInput>();
    REQUIRE(ring.bytes.size() == relayedInputRing::kHeaderBytes + entryStride);
}

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T29: THE MALFORMED-LENGTH BOUND.
//
// This rule used to live inside FRelayedInputRing::NetSerialize as an inline
// `used > kMaxWireBytes` comparison, where this target could not see it — and
// where, from the day Iris replication went live, it did not even RUN, because
// Iris resolves custom serializers from a name registry and never called that
// function at all (T20 §4.2; T29 registers one so it is called again).
//
// The lesson worth pinning in a test rather than a comment: a guard is two
// separate claims — that the RULE is right, and that the rule RUNS. These cases
// cover the first. The second is a property of the transport and is proved at
// runtime by the one-shot `[IrisSerializerProof]` report inside NetSerialize.
//////////////////////////////////////////////////////////////////////////////

TEST_CASE("RelayRing.WireLengthBoundAcceptsWellFormedPayloads", "[WireFormat][RelayRing]")
{
    // The bound must clear anything the ring itself can produce, or a legitimate
    // sender would be rejected. Build the worst case the codec allows — kMaxDepth
    // resident entries of the largest input the bound is anchored to — and check
    // it fits, rather than trusting the arithmetic in the constant's definition.
    const std::uint32_t worstCaseBytes =
        relayedInputRing::kHeaderBytes
        + static_cast<std::uint32_t>(relayedInputRing::kMaxDepth)
            * (static_cast<std::uint32_t>(sizeof(std::uint32_t) + sizeof(std::uint8_t))
               + relayedInputRing::kMaxInputBytes);

    REQUIRE(relayedInputRing::isAcceptableWireLength(worstCaseBytes));
    REQUIRE(relayedInputRing::kMaxWireBytes == worstCaseBytes);

    // An empty ring (nothing written yet) and a real depth-1 payload both pass.
    REQUIRE(relayedInputRing::isAcceptableWireLength(0u));

    RingTestBuffer ring;
    relayedInputRing::writeLatest<RingTestInput>(ring, 42u, 3u, RingTestInput{ 7 }, kDepth1);
    REQUIRE(relayedInputRing::isAcceptableWireLength(static_cast<std::uint32_t>(ring.bytes.size())));

    // And a ring filled to the hard depth cap with a COMPOSITE input — the shape
    // production actually relays (simulatableBrawler::PlayerInput is a composite),
    // and the largest thing the codec can emit.
    using CompositeInput = SimulationComposite<RingTestInput, RingTestPart>;
    constexpr std::int32_t maxDepth = static_cast<std::int32_t>(relayedInputRing::kMaxDepth);

    RingTestBuffer deepRing;
    for (std::uint32_t tick = 0; tick < relayedInputRing::kMaxDepth; ++tick)
    {
        relayedInputRing::writeLatest<CompositeInput>(
            deepRing, tick, 2u,
            CompositeInput{ RingTestInput{ static_cast<std::int32_t>(tick) }, RingTestPart{ 1.f } },
            maxDepth);
    }
    REQUIRE(relayedInputRing::entryCount(deepRing) == relayedInputRing::kMaxDepth);
    REQUIRE(relayedInputRing::isAcceptableWireLength(static_cast<std::uint32_t>(deepRing.bytes.size())));
}

TEST_CASE("RelayRing.WireLengthBoundRejectsOversizePayloads", "[WireFormat][RelayRing]")
{
    // One byte past the bound is the whole point: the transport must refuse it
    // rather than allocate and read that many bytes out of a corrupt or hostile
    // bunch.
    REQUIRE_FALSE(relayedInputRing::isAcceptableWireLength(relayedInputRing::kMaxWireBytes + 1u));

    // The rejection is not a near-miss special case — it holds for the whole
    // range up to the engine's own outer bound (Iris caps a LastResort blob at
    // 65535 bits; FArrayPropertyNetSerializer at 65535 elements), which is the
    // range a uint16 length prefix can actually express.
    REQUIRE_FALSE(relayedInputRing::isAcceptableWireLength(4096u));
    REQUIRE_FALSE(relayedInputRing::isAcceptableWireLength(65535u));

    // The boundary itself is INCLUSIVE — a payload exactly at the bound is
    // well-formed, and an off-by-one here would silently reject a full-depth ring.
    REQUIRE(relayedInputRing::isAcceptableWireLength(relayedInputRing::kMaxWireBytes));
}

//////////////////////////////////////////////////////////////////////////////
// ⭐ og-netcode-v2-input-relay / T34 — FLUSH-ON-POLL (BARE C1, R = 0).
//
// The relay tap runs on the RPC receipt path (paced by packet arrival) while Iris
// polls the replicated ring once per server game-thread frame. Under the retired
// replace-latest write path a second arrival in one frame overwrote the first in
// server memory before replication ever compared the property — measured at
// ~11.6 % of relayed inputs, and indistinguishable from a wire drop at the client.
// Flush-on-poll STAGES every arrival and publishes the whole burst at the poll.
//
// WHAT THESE CASES PIN, in order of how expensive getting it wrong would be:
//
//   1. THE CAPACITY RULE. The stage's capacity is `kMaxDepth`, taken as a constant
//      inside `stageArrival`. If the flush path ever read
//      `TimeConfig::relayRedundancyDepthTicks` (session value 1) instead, the
//      second and every later staged entry would supersede the first, the ring
//      would carry exactly one entry per round, and bare C1 would silently
//      degenerate into the behaviour it replaces — no compile error, no warning,
//      and every payload-level test still green. That is T43 finding 1, and
//      `RelayRing.StagedBurstSurvivesAtTheSessionDefaultDepth` is the case that
//      makes it impossible to reintroduce quietly: it drives the burst with the
//      session default sitting right there in the same test.
//   2. SUPPRESS-CLEAR-ON-EMPTY. Not a bandwidth optimization — the skip-recovery
//      mechanism (T43 finding 3). An empty flush must leave the destination BYTE-
//      IDENTICAL so `FProperty::Identical` finds no change, the property is not
//      dirty, and a round Iris skipped under packet pressure survives the quiet
//      frames that follow to be retried. Under R = 0 that retry is the only
//      recovery there is.
//   3. THE SHRINK, AND THE VERSION BYTE. The transport ships `bundleByteNum()`
//      bytes, so a ring whose entry count fell without its buffer shrinking would
//      send stale trailing entries forever; and truncating to 0 rather than to the
//      header would re-arm `initHeaderIfEmpty`'s laziness and emit a version-0 ring
//      that the consumer classifies as NeverWritten and silently drops.
//////////////////////////////////////////////////////////////////////////////

TEST_CASE("RelayRing.ResetEntriesShrinksToTheHeaderAndKeepsTheVersion", "[WireFormat][RelayRing]")
{
    RingTestBuffer ring;
    constexpr std::int32_t depth = 3;

    relayedInputRing::writeLatest<RingTestInput>(ring, 10u, 1u, RingTestInput{ 100 }, depth);
    relayedInputRing::writeLatest<RingTestInput>(ring, 11u, 1u, RingTestInput{ 101 }, depth);
    relayedInputRing::writeLatest<RingTestInput>(ring, 12u, 1u, RingTestInput{ 102 }, depth);
    REQUIRE(relayedInputRing::entryCount(ring) == 3u);
    const std::size_t highWaterMark = ring.bytes.size();
    REQUIRE(highWaterMark > relayedInputRing::kHeaderBytes);

    relayedInputRing::resetEntries(ring);

    // The buffer really came back to the header — not just the count byte. This is
    // the trap: `used = wireBytes.Num()` is what rides the wire.
    REQUIRE(relayedInputRing::entryCount(ring) == 0u);
    REQUIRE(ring.bytes.size() == relayedInputRing::kHeaderBytes);

    // ...and the version byte survived, so the ring never reads as never-written.
    REQUIRE(relayedInputRing::getWireFormatVersion(ring) == relayedInputRing::kWireFormatVersion);
    REQUIRE(relayedInputRing::getWireFormatVersion(ring) != 0u);

    // Reset is idempotent, and a never-written ring is left alone entirely (it has
    // no header to preserve, and writing one would cost wire bytes an unwritten
    // ring is documented not to pay).
    relayedInputRing::resetEntries(ring);
    REQUIRE(ring.bytes.size() == relayedInputRing::kHeaderBytes);

    RingTestBuffer virgin;
    relayedInputRing::resetEntries(virgin);
    REQUIRE(virgin.bytes.empty());
    REQUIRE(relayedInputRing::getWireFormatVersion(virgin) == 0u);
}

TEST_CASE("RelayRing.StagedBurstSurvivesAtTheSessionDefaultDepth", "[WireFormat][RelayRing]")
{
    // ⭐ THE CASE T43 FINDING 1 EXISTS FOR. `kSessionDepthKnob` is the value
    // `TimeConfig::relayRedundancyDepthTicks` ships at and the value
    // `Config/DefaultEngine.ini` carries; it is declared here, unused by the flush,
    // precisely so that a future implementer who threads it into the flush path
    // watches this case go red instead of shipping a silent regression.
    constexpr std::int32_t kSessionDepthKnob = 1;
    REQUIRE(kSessionDepthKnob < static_cast<std::int32_t>(relayedInputRing::kMaxDepth));

    RingTestBuffer stage;
    RingTestBuffer ring;

    // A five-arrival burst inside ONE server frame — above the session knob, below
    // the stage's own capacity.
    for (std::uint32_t i = 0; i < 5u; ++i)
    {
        const relayedInputRing::StageArrivalOutcome outcome =
            relayedInputRing::stageArrival<RingTestInput>(
                stage, 200u + i, 4u, RingTestInput{ static_cast<std::int32_t>(i) });
        REQUIRE(outcome.accepted);
        REQUIRE_FALSE(outcome.droppedOldest);
    }
    REQUIRE(relayedInputRing::entryCount(stage) == 5u);

    REQUIRE(relayedInputRing::flushStagedInto(ring, stage) == 5u);

    // FIVE resident, not one. One is what a depth-parameterised flush would have
    // produced, and it is what ~88 % observability looks like.
    REQUIRE(relayedInputRing::entryCount(ring) == 5u);
    for (std::uint32_t i = 0; i < 5u; ++i)
        REQUIRE(hasEntry(ring, 200u + i, 4u, static_cast<std::int32_t>(i)));

    // The stage came back empty, header intact, ready for the next frame.
    REQUIRE(relayedInputRing::entryCount(stage) == 0u);
    REQUIRE(stage.bytes.size() == relayedInputRing::kHeaderBytes);
    REQUIRE(relayedInputRing::getWireFormatVersion(stage) == relayedInputRing::kWireFormatVersion);

    // The published ring is byte-identical to a ring of that residency built the
    // ordinary way — the wire format did not change, which is why no version bump
    // is warranted (design_task30_c1_flush_on_poll.md §5.3).
    RingTestBuffer reference;
    for (std::uint32_t i = 0; i < 5u; ++i)
    {
        relayedInputRing::writeLatest<RingTestInput>(
            reference, 200u + i, 4u, RingTestInput{ static_cast<std::int32_t>(i) },
            static_cast<std::int32_t>(relayedInputRing::kMaxDepth));
    }
    REQUIRE(ring.bytes == reference.bytes);
}

TEST_CASE("RelayRing.FlushWithAnEmptyStageLeavesTheRingByteIdentical", "[WireFormat][RelayRing]")
{
    // ⭐ SUPPRESS-CLEAR-ON-EMPTY (T43 finding 3). Byte-identical is the whole
    // assertion: it is what makes `FProperty::Identical` report no change, which is
    // what keeps the object out of `ObjectsWithDirtyChanges`, which is what lets a
    // scheduler-skipped round survive the quiet frames and be retried. Under R = 0
    // there is no other recovery.
    RingTestBuffer stage;
    RingTestBuffer ring;

    relayedInputRing::stageArrival<RingTestInput>(stage, 300u, 2u, RingTestInput{ 7 });
    relayedInputRing::stageArrival<RingTestInput>(stage, 301u, 2u, RingTestInput{ 8 });
    REQUIRE(relayedInputRing::flushStagedInto(ring, stage) == 2u);

    const std::vector<std::uint8_t> publishedBytes = ring.bytes;

    // Three empty frames in a row. The round stays resident and untouched.
    for (int frame = 0; frame < 3; ++frame)
    {
        REQUIRE(relayedInputRing::flushStagedInto(ring, stage) == 0u);
        REQUIRE(ring.bytes == publishedBytes);
    }
    REQUIRE(relayedInputRing::entryCount(ring) == 2u);
    REQUIRE(hasEntry(ring, 300u, 2u, 7));
    REQUIRE(hasEntry(ring, 301u, 2u, 8));

    // An empty flush against a NEVER-WRITTEN ring must also leave it never-written
    // — not a header-only, version-carrying, 2-byte payload that would replicate
    // once for nothing.
    RingTestBuffer virginRing;
    RingTestBuffer emptyStage;
    REQUIRE(relayedInputRing::flushStagedInto(virginRing, emptyStage) == 0u);
    REQUIRE(virginRing.bytes.empty());
}

TEST_CASE("RelayRing.FlushShrinksTheRingWhenAQuietRoundFollowsABurst", "[WireFormat][RelayRing]")
{
    // The other half of the empty-frame story: a NON-empty small round after a big
    // one must actually shrink the payload. `NetSerialize` ships
    // `wireBytes.Num()`, so a ring left at its high-water mark would send stale
    // trailing entries every round and the whole saving would be zero.
    RingTestBuffer stage;
    RingTestBuffer ring;

    for (std::uint32_t i = 0; i < relayedInputRing::kMaxDepth; ++i)
        relayedInputRing::stageArrival<RingTestInput>(stage, 400u + i, 1u, RingTestInput{ 1 });
    REQUIRE(relayedInputRing::flushStagedInto(ring, stage)
            == static_cast<std::uint8_t>(relayedInputRing::kMaxDepth));

    const std::size_t burstBytes = ring.bytes.size();

    relayedInputRing::stageArrival<RingTestInput>(stage, 500u, 1u, RingTestInput{ 2 });
    REQUIRE(relayedInputRing::flushStagedInto(ring, stage) == 1u);

    REQUIRE(relayedInputRing::entryCount(ring) == 1u);
    REQUIRE(ring.bytes.size() < burstBytes);
    REQUIRE(hasEntry(ring, 500u, 1u, 2));

    // No trace of the burst survives — not as a resident entry, and not as trailing
    // bytes past the count.
    for (std::uint32_t i = 0; i < relayedInputRing::kMaxDepth; ++i)
        REQUIRE_FALSE(hasCaptureTick(ring, 400u + i));

    const std::size_t stride =
        sizeof(std::uint32_t) + sizeof(std::uint8_t) + syncSize<RingTestInput>();
    REQUIRE(ring.bytes.size() == relayedInputRing::kHeaderBytes + stride);
    REQUIRE(relayedInputRing::isAcceptableWireLength(
        static_cast<std::uint32_t>(ring.bytes.size())));
}

TEST_CASE("RelayRing.StageOverflowDropsTheOldestAndReportsIt", "[WireFormat][RelayRing]")
{
    // The stage does NOT raise the ring's ceiling — `kMaxDepth` is a hard wire
    // bound, and a burst longer than it loses its oldest entries exactly as the
    // ring itself would have. What flush-on-poll adds is that the loss is REPORTED
    // rather than silent: this is the only input loss the server side of R = 0 can
    // observe at all, since Iris exposes no send-success signal to game code.
    RingTestBuffer stage;

    for (std::uint32_t i = 0; i < relayedInputRing::kMaxDepth; ++i)
    {
        const relayedInputRing::StageArrivalOutcome outcome =
            relayedInputRing::stageArrival<RingTestInput>(
                stage, 600u + i, 1u, RingTestInput{ static_cast<std::int32_t>(i) });
        REQUIRE(outcome.accepted);
        REQUIRE_FALSE(outcome.droppedOldest);
    }
    REQUIRE(relayedInputRing::entryCount(stage) == relayedInputRing::kMaxDepth);

    // One past capacity: accepted, and the OLDEST is gone.
    const relayedInputRing::StageArrivalOutcome overflow =
        relayedInputRing::stageArrival<RingTestInput>(stage, 700u, 1u, RingTestInput{ 99 });
    REQUIRE(overflow.accepted);
    REQUIRE(overflow.droppedOldest);
    REQUIRE(relayedInputRing::entryCount(stage) == relayedInputRing::kMaxDepth);
    REQUIRE(hasCaptureTick(stage, 700u));
    REQUIRE_FALSE(hasCaptureTick(stage, 600u));

    // A RE-STAMP of a resident capture tick evicts nothing — it rewrites in place,
    // so reporting a drop there would inflate the only loss signal this side has.
    const relayedInputRing::StageArrivalOutcome restamp =
        relayedInputRing::stageArrival<RingTestInput>(stage, 700u, 9u, RingTestInput{ 99 });
    REQUIRE(restamp.accepted);
    REQUIRE_FALSE(restamp.droppedOldest);
    REQUIRE(hasEntry(stage, 700u, 9u, 99));

    // A REFUSED stale write evicts nothing either, and must not be reported as a
    // drop: nothing was lost, the write was.
    const relayedInputRing::StageArrivalOutcome stale =
        relayedInputRing::stageArrival<RingTestInput>(stage, 1u, 1u, RingTestInput{ -1 });
    REQUIRE_FALSE(stale.accepted);
    REQUIRE_FALSE(stale.droppedOldest);
}

TEST_CASE("RelayRing.FlushIsEquivalentToSequentialWritesAcrossManyRounds", "[WireFormat][RelayRing]")
{
    // A session-shaped drive: a steady 1-per-frame stream with occasional 2- and
    // 3-bursts, over enough rounds to catch a leak in either buffer. Every capture
    // tick must be published exactly once, the ring must never exceed kMaxDepth,
    // and neither buffer may grow without bound.
    RingTestBuffer stage;
    RingTestBuffer ring;

    std::uint32_t nextTick   = 1000u;
    std::uint32_t published  = 0u;
    std::size_t   maxRingBytes = 0u;

    for (std::uint32_t frame = 0; frame < 200u; ++frame)
    {
        const std::uint32_t arrivals = (frame % 17u == 0u) ? 3u
                                     : (frame % 5u == 0u) ? 2u
                                     : 1u;
        for (std::uint32_t a = 0; a < arrivals; ++a)
        {
            relayedInputRing::stageArrival<RingTestInput>(
                stage, nextTick, 3u, RingTestInput{ static_cast<std::int32_t>(nextTick) });
            ++nextTick;
        }

        const std::uint8_t count = relayedInputRing::flushStagedInto(ring, stage);
        REQUIRE(count == static_cast<std::uint8_t>(arrivals));
        published += count;

        REQUIRE(relayedInputRing::entryCount(ring) <= relayedInputRing::kMaxDepth);
        REQUIRE(stage.bytes.size() == relayedInputRing::kHeaderBytes);
        maxRingBytes = (ring.bytes.size() > maxRingBytes) ? ring.bytes.size() : maxRingBytes;
    }

    // Every produced capture tick reached the wire — that is the 88 % -> ~100 %
    // server-side observability claim, asserted rather than modelled.
    REQUIRE(published == nextTick - 1000u);

    const std::size_t stride =
        sizeof(std::uint32_t) + sizeof(std::uint8_t) + syncSize<RingTestInput>();
    REQUIRE(maxRingBytes == relayedInputRing::kHeaderBytes + 3u * stride);
    REQUIRE(relayedInputRing::isAcceptableWireLength(static_cast<std::uint32_t>(maxRingBytes)));
}

TEST_CASE("RelayRing.FlushCarriesCompositeInputsAndTheirStamps", "[WireFormat][RelayRing]")
{
    // Production relays a SimulationComposite (simulatableBrawler::PlayerInput), and
    // the flush is byte-wise and type-erased — it never names an InputType. That is
    // what lets it run on the ring's UE host actor, which must not know the game's
    // types. This case is the proof that type-erasure does not corrupt a composite
    // payload or its dA schedule stamp.
    using CompositeInput = SimulationComposite<RingTestInput, RingTestPart>;

    RingTestBuffer stage;
    RingTestBuffer ring;

    relayedInputRing::stageArrival<CompositeInput>(
        stage, 800u, 6u, CompositeInput{ RingTestInput{ 11 }, RingTestPart{ 2.5f } });
    relayedInputRing::stageArrival<CompositeInput>(
        stage, 801u, 7u, CompositeInput{ RingTestInput{ 12 }, RingTestPart{ 3.5f } });

    REQUIRE(relayedInputRing::flushStagedInto(ring, stage) == 2u);

    std::uint8_t   dA = 0;
    CompositeInput out{};
    REQUIRE(relayedInputRing::findEntry<CompositeInput>(ring, 801u, dA, out));
    REQUIRE(dA == 7u);
    REQUIRE(out.get<RingTestInput>().value == 12);
    REQUIRE(out.get<RingTestPart>().scale == 3.5f);

    // And the derived application tick still comes from its two operands.
    REQUIRE(relayedInputRing::applicationTick(801u, dA) == 808u);
}

#endif // WITH_LOW_LEVEL_TESTS
