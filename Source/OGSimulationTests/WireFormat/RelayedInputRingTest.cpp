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

#endif // WITH_LOW_LEVEL_TESTS
