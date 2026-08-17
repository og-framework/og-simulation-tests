// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/Network/LocalInputCache.h"   // the has-clear() contrast
#include "OGSimulation/Network/RemoteInputCache.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationFieldDescriptors.h"

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T5: RemoteInputCache — the CLIENT-side read cache
// of relayed inputs, and populateRemoteInputCache, the one ingest that carries
// the wire-version fence.
//
// WHAT THESE TESTS EXIST TO PIN, in order of how easy each is to break silently:
//
//   1. "LATEST" IS A DERIVATION, NOT A FIELD (the architect ruling). The
//      randomized case below drives re-stamps of the resident-latest tick with a
//      FRESHER dA, ring wraparound, and out-of-order capture ticks against an
//      INDEPENDENTLY MODELLED oracle (a map keyed by slot index — a different
//      algorithm, not a copy of the scan). A stored "latest" scalar with a `>`
//      update rule instead of `>=` passes every simple case and fails exactly this
//      one; the same case is the gate any future memo must pass.
//
//   2. THE MISS IS VISIBLE. `find` is pure hit/miss and never invents a neutral —
//      that is the whole behavioural difference from LocalInputCache::at(),
//      and T7's ladder and T6's resolution table both depend on seeing the miss.
//
//   3. THE INJECTED ZERO IS NOT `InputT{}`. The test input below deliberately has
//      a field whose value-initialised value (0) differs from its game-zero value
//      (1), mirroring simulatableBrawler::getZeroPlayerInput's (0,0,1) forward
//      vector against a value-initialised (0,0,0). Without that asymmetry the
//      fallback assertions would pass vacuously — the exact blind spot the T9
//      part-2 notes recorded.
//
//   4. THE WIRE-VERSION FENCE. T5 is the FIRST reader of the ring's version byte;
//      nothing checked it before. Entry stride is a compile-time constant per
//      InputType, so on a layout change an old peer reads arbitrary bytes as
//      capture ticks and inserts them as STORE KEYS. These cases hand-patch the
//      version byte on a well-formed ring and require that NOTHING enters.
//
// These run against the REAL production types — the store, the real codec, and the
// real populate — over a std::vector-backed buffer standing in for the USTRUCT's
// TArray-backed one, per the codec's BUFFER CONCEPT. Only UE replication itself
// (NetSerialize / OnRep) is out of reach from this target.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // Test input with the load-bearing asymmetry: `forward` is 0 when
    // value-initialised and 1 in the "game zero", so an assertion can tell the
    // injected neutral from `InputT{}` — which no tag-style single-field probe can.
    struct StoreTestInput
    {
        std::int32_t forward = 0;
        std::int32_t tag     = 0;

        bool operator==(const StoreTestInput& other) const
        {
            return forward == other.forward && tag == other.tag;
        }
    };
} // namespace

template <>
struct SerializableFields<StoreTestInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(StoreTestInput, forward),
                               SIM_MEMBER(StoreTestInput, tag));
    }
};

namespace
{
    // Stands in for simulatableBrawler::getZeroPlayerInput(): NOT StoreTestInput{}.
    StoreTestInput gameZeroInput()
    {
        StoreTestInput zero;
        zero.forward = 1;
        zero.tag     = 0;
        return zero;
    }

    StoreTestInput taggedInput(std::int32_t tag)
    {
        StoreTestInput input = gameZeroInput();
        input.tag = tag;
        return input;
    }

    // UE-free byte buffer satisfying the codec's BUFFER CONCEPT — the same four
    // methods FRelayedInputRing exposes over its TArray<uint8>.
    struct StoreTestRing
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
} // namespace

// ---------------------------------------------------------------------------
// 1. THE LOOKUP CONTRACT — hit/miss, last-wins, eviction.
// ---------------------------------------------------------------------------

TEST_CASE("RemoteInputCache: find is pure hit/miss and never invents a neutral",
          "[Network][RemoteInputCache]")
{
    RemoteInputCache<StoreTestInput> store(gameZeroInput());

    std::uint8_t   outDA = 0xEEu;
    StoreTestInput outInput = taggedInput(-999);

    // MISS on an empty store: false, and BOTH out params left exactly as they were.
    // Leaving them untouched is what lets a caller distinguish "no entry" from
    // "an entry whose value happens to look neutral".
    REQUIRE_FALSE(store.find(10u, outDA, outInput));
    REQUIRE(outDA == 0xEEu);
    REQUIRE(outInput.tag == -999);
    REQUIRE_FALSE(store.has(10u));

    REQUIRE(store.push(10u, 4u, taggedInput(10)));
    REQUIRE(store.has(10u));
    REQUIRE(store.find(10u, outDA, outInput));
    REQUIRE(outDA == 4u);
    REQUIRE(outInput.tag == 10);

    // A neighbouring tick that was never pushed is still a MISS, not the resident
    // neighbour: find validates the stored tick, so the ring's modulus is never
    // something the caller has to reason about.
    REQUIRE_FALSE(store.find(11u, outDA, outInput));
    REQUIRE(outDA == 4u);           // still untouched by the miss
}

TEST_CASE("RemoteInputCache: push is LAST-WINS, including a re-stamp with a fresher dA",
          "[Network][RemoteInputCache]")
{
    RemoteInputCache<StoreTestInput> store(gameZeroInput());

    REQUIRE(store.push(20u, 2u, taggedInput(100)));
    REQUIRE(store.push(20u, 7u, taggedInput(101)));   // same tick, fresher stamp

    std::uint8_t   dA = 0u;
    StoreTestInput input;
    REQUIRE(store.find(20u, dA, input));
    REQUIRE(dA == 7u);
    REQUIRE(input.tag == 101);
    REQUIRE(store.residentCount() == 1u);

    // The wire codec explicitly permits rewriting a resident capture tick (a
    // re-stamp after a delay change), so a first-wins store would silently keep a
    // stale schedule stamp for that tick.
    REQUIRE(store.findLatest().dA == 7u);
}

TEST_CASE("RemoteInputCache: an entry is evicted by the tick that shares its slot",
          "[Network][RemoteInputCache]")
{
    RemoteInputCache<StoreTestInput> store(gameZeroInput());
    const std::uint32_t cap = static_cast<std::uint32_t>(store.capacity());
    REQUIRE(cap == static_cast<std::uint32_t>(kRemoteInputCacheCapacityTicks));

    REQUIRE(store.push(5u, 1u, taggedInput(5)));
    REQUIRE(store.has(5u));

    REQUIRE(store.push(5u + cap, 1u, taggedInput(5 + static_cast<std::int32_t>(cap))));

    // The older tick is gone — and reads as ABSENT rather than as its stale
    // neighbour, which is what makes the residency bound in
    // relayDelayFloorHardCapTicks a real bound rather than a silent corruption.
    REQUIRE_FALSE(store.has(5u));
    REQUIRE(store.has(5u + cap));
    REQUIRE(store.residentCount() == 1u);
}

// ---------------------------------------------------------------------------
// 2. "LATEST" IS DERIVED — the randomized-equivalence case.
// ---------------------------------------------------------------------------

TEST_CASE("RemoteInputCache: findLatest is a DERIVATION over the slots, not a scalar",
          "[Network][RemoteInputCache]")
{
    RemoteInputCache<StoreTestInput> store(gameZeroInput());
    const std::uint32_t cap = static_cast<std::uint32_t>(store.capacity());

    // Nothing has arrived: the T7 rung-0 condition, and the contract that forbids
    // a scheduled probe with a default dA of 0.
    REQUIRE_FALSE(store.findLatest().valid);

    // INDEPENDENT ORACLE: last-writer-per-slot-index, keyed by slot index rather
    // than scanned over a vector. Deliberately a different algorithm from the one
    // under test — a copy of the scan would prove nothing.
    struct OracleEntry { std::uint32_t captureTick; std::uint8_t dA; std::int32_t tag; };
    std::unordered_map<std::uint32_t, OracleEntry> lastPerSlot;

    // A deterministic, deliberately nasty sequence: forward progress, backward
    // jumps (out-of-order pushes), re-stamps of the resident-latest tick with a
    // FRESHER dA, and enough range to wrap the ring several times.
    std::uint32_t seed = 12345u;
    auto nextRandom = [&seed]() {
        seed = seed * 1103515245u + 12345u;
        return (seed >> 16) & 0x7FFFu;
    };

    std::uint32_t cursor = 0u;
    for (int step = 0; step < 400; ++step)
    {
        const std::uint32_t roll = nextRandom() % 10u;

        std::uint32_t captureTick;
        if (roll < 5u)                    // advance
            captureTick = (cursor += 1u + (nextRandom() % 3u));
        else if (roll < 8u)               // jump BACK — an out-of-order capture tick
            captureTick = cursor > 20u ? cursor - (nextRandom() % 20u) : cursor;
        else                              // RE-STAMP the current newest
            captureTick = cursor;

        const std::uint8_t dA  = static_cast<std::uint8_t>(nextRandom() % 40u);
        const std::int32_t tag = static_cast<std::int32_t>(step);

        REQUIRE(store.push(captureTick, dA, taggedInput(tag)));
        lastPerSlot[captureTick % cap] = OracleEntry{ captureTick, dA, tag };

        // Recompute the expected latest from the oracle, every single step.
        bool        expectValid = false;
        OracleEntry expected{ 0u, 0u, 0 };
        for (const auto& [slotIndex, entry] : lastPerSlot)
        {
            if (!expectValid || entry.captureTick > expected.captureTick)
            {
                expectValid = true;
                expected    = entry;
            }
        }

        const auto latest = store.findLatest();
        REQUIRE(latest.valid == expectValid);
        REQUIRE(latest.captureTick == expected.captureTick);
        // The dA is the assertion a `>`-updated stored scalar fails: a re-stamp of
        // the already-latest tick must move the stamp, not leave it.
        REQUIRE(latest.dA == expected.dA);
        REQUIRE(latest.input.tag == expected.tag);

        // fallback() is the SAME derivation, viewed as `lastKnown`. Asserting it
        // in the same loop is what pins "one derivation, two views".
        REQUIRE(store.fallback().tag == expected.tag);
    }

    // Sanity: the sequence actually exercised wraparound and re-stamping rather
    // than quietly degenerating into a monotonic walk.
    REQUIRE(cursor > cap * 2u);
    REQUIRE(lastPerSlot.size() == cap);
}

// ---------------------------------------------------------------------------
// 3. FALLBACK — argument-less, and the injected zero is NOT InputT{}.
// ---------------------------------------------------------------------------

TEST_CASE("RemoteInputCache: fallback answers the INJECTED game zero before anything arrives",
          "[Network][RemoteInputCache]")
{
    // ANTI-VACUITY GUARD. If these two ever became equal the whole case below
    // would pass without testing anything — which is precisely how the
    // (0,0,1)-vs-(0,0,0) forward-vector trap hides.
    REQUIRE_FALSE(gameZeroInput() == StoreTestInput{});

    RemoteInputCache<StoreTestInput> store(gameZeroInput());

    REQUIRE_FALSE(store.findLatest().valid);
    REQUIRE(store.fallback() == gameZeroInput());
    REQUIRE_FALSE(store.fallback() == StoreTestInput{});

    // Once ANYTHING has arrived, fallback is the newest arrived input — held
    // indefinitely. The bounded-hold rule (drop to zero after K quiet ticks) is
    // DEFERRED by design (RelayDelaySpectrumDesign.md §8.7): unbounded hold is
    // today's behaviour, so shipping it keeps the no-observable-regression gate
    // clean. This assertion is therefore pinning the DEFERRAL, and a future task
    // adding the hold rule is expected to change it.
    REQUIRE(store.push(3u, 2u, taggedInput(77)));
    REQUIRE(store.fallback().tag == 77);

    // ...and it does not decay with distance: no tick is passed in, because
    // fallback() deliberately takes NO frontierTick.
    REQUIRE(store.fallback().tag == 77);
}

TEST_CASE("RemoteInputCache: a re-injected neutral reaches an already-constructed store",
          "[Network][RemoteInputCache]")
{
    // The composition root injects the game zero without ordering itself before
    // every registration, so a store built with the default must be correctable.
    RemoteInputCache<StoreTestInput> store;
    REQUIRE(store.fallback() == StoreTestInput{});

    store.setNeutralInput(gameZeroInput());
    REQUIRE(store.fallback() == gameZeroInput());
    REQUIRE(store.getNeutralInput() == gameZeroInput());
}

// ---------------------------------------------------------------------------
// 4. THE SENTINEL IS NEVER A KEY.
// ---------------------------------------------------------------------------

TEST_CASE("RemoteInputCache: kNoInputCaptureTick is rejected as a key",
          "[Network][RemoteInputCache]")
{
    RemoteInputCache<StoreTestInput> store(gameZeroInput());

    // push REFUSES it and changes nothing. Defensive: the relay path only carries
    // real capture ticks, but T6 resolves a sentinel REF to the game zero and must
    // never see it become a successful lookup.
    REQUIRE_FALSE(store.push(kNoInputCaptureTick, 3u, taggedInput(1)));
    REQUIRE(store.residentCount() == 0u);
    REQUIRE_FALSE(store.findLatest().valid);

    std::uint8_t   dA = 0u;
    StoreTestInput input;
    REQUIRE_FALSE(store.has(kNoInputCaptureTick));
    REQUIRE_FALSE(store.find(kNoInputCaptureTick, dA, input));

    // A real entry alongside it is unaffected — the rejection is targeted, not a
    // poisoned store.
    REQUIRE(store.push(9u, 1u, taggedInput(9)));
    REQUIRE(store.residentCount() == 1u);
    REQUIRE_FALSE(store.find(kNoInputCaptureTick, dA, input));
}

// ---------------------------------------------------------------------------
// 5. THE INGEST + THE WIRE-VERSION FENCE.
// ---------------------------------------------------------------------------

namespace
{
    // A well-formed depth-N ring, written through the REAL codec.
    StoreTestRing makeRing(const std::vector<std::uint32_t>& captureTicks, std::int32_t depth)
    {
        StoreTestRing ring;
        for (std::uint32_t tick : captureTicks)
        {
            relayedInputRing::writeLatest<StoreTestInput>(
                ring, tick, static_cast<std::uint8_t>(tick % 7u),
                taggedInput(static_cast<std::int32_t>(tick)), depth);
        }
        return ring;
    }
}

TEST_CASE("RemoteInputCache: populate consumes a matching-version ring",
          "[Network][RemoteInputCache]")
{
    RemoteInputCache<StoreTestInput> store(gameZeroInput());
    const StoreTestRing ring = makeRing({ 100u, 101u, 102u }, 3);

    const RelayedInputIngestReport report =
        populateRemoteInputCache<StoreTestInput>(store, ring);

    REQUIRE(report.outcome == RelayedInputIngestOutcome::Consumed);
    REQUIRE(report.versionOnWire == relayedInputRing::kWireFormatVersion);
    REQUIRE(report.entriesIngested == 3u);
    REQUIRE(store.residentCount() == 3u);

    std::uint8_t   dA = 0u;
    StoreTestInput input;
    REQUIRE(store.find(101u, dA, input));
    REQUIRE(dA == static_cast<std::uint8_t>(101u % 7u));
    REQUIRE(input.tag == 101);
    REQUIRE(store.findLatest().captureTick == 102u);
}

TEST_CASE("RemoteInputCache: populate re-consumes the whole ring idempotently",
          "[Network][RemoteInputCache]")
{
    // NO DIFFING: every arrival re-pushes every resident entry. That is what keeps
    // the same code correct unchanged when relay depth rises above 1 (§8.2).
    RemoteInputCache<StoreTestInput> store(gameZeroInput());
    StoreTestRing ring = makeRing({ 200u, 201u }, 3);

    REQUIRE(populateRemoteInputCache<StoreTestInput>(store, ring).entriesIngested == 2u);
    REQUIRE(populateRemoteInputCache<StoreTestInput>(store, ring).entriesIngested == 2u);
    REQUIRE(store.residentCount() == 2u);

    // A re-stamp on the wire lands as a re-stamp in the store.
    relayedInputRing::writeLatest<StoreTestInput>(ring, 201u, 30u, taggedInput(-201), 3);
    REQUIRE(populateRemoteInputCache<StoreTestInput>(store, ring).outcome
            == RelayedInputIngestOutcome::Consumed);

    std::uint8_t   dA = 0u;
    StoreTestInput input;
    REQUIRE(store.find(201u, dA, input));
    REQUIRE(dA == 30u);
    REQUIRE(input.tag == -201);
    REQUIRE(store.residentCount() == 2u);
}

TEST_CASE("RemoteInputCache: a never-written ring is a SILENT no-op",
          "[Network][RemoteInputCache]")
{
    // Version 0 means "the codec never wrote its header", i.e. this property has
    // never been replicated. It must not be reported as a mismatch and must not be
    // logged — every client sees this state for every character before the first
    // relay write.
    RemoteInputCache<StoreTestInput> store(gameZeroInput());
    const StoreTestRing emptyRing;

    const RelayedInputIngestReport report =
        populateRemoteInputCache<StoreTestInput>(store, emptyRing);

    REQUIRE(report.outcome == RelayedInputIngestOutcome::NeverWritten);
    REQUIRE(report.versionOnWire == 0u);
    REQUIRE(report.entriesIngested == 0u);
    REQUIRE(store.residentCount() == 0u);

    // The log gate is untouched by a version-0 ring, so the FIRST genuine mismatch
    // still gets its one line.
    REQUIRE(store.shouldLogVersionMismatchOnce());
}

TEST_CASE("RemoteInputCache: a version mismatch drops the WHOLE ring",
          "[Network][RemoteInputCache]")
{
    RemoteInputCache<StoreTestInput> store(gameZeroInput());

    // Start from a ring that is valid in every respect...
    StoreTestRing ring = makeRing({ 300u, 301u, 302u }, 3);
    REQUIRE(relayedInputRing::getWireFormatVersion(ring) == relayedInputRing::kWireFormatVersion);
    REQUIRE(relayedInputRing::entryCount(ring) == 3u);

    // ...then hand-patch ONLY the version byte, exactly as a future layout change
    // would present itself to an old peer: correct length, correct framing,
    // untrustworthy contents.
    ring.bytes[relayedInputRing::kVersionOffset] =
        static_cast<std::uint8_t>(relayedInputRing::kWireFormatVersion + 1u);

    const RelayedInputIngestReport report =
        populateRemoteInputCache<StoreTestInput>(store, ring);

    REQUIRE(report.outcome == RelayedInputIngestOutcome::VersionMismatch);
    REQUIRE(report.versionOnWire == relayedInputRing::kWireFormatVersion + 1u);

    // NOTHING enters the store — not a partial read, not a "best effort" entry.
    // Garbage capture ticks here would become STORE KEYS and then be applied to a
    // proxy: silent corruption with no crash to notice.
    REQUIRE(report.entriesIngested == 0u);
    REQUIRE(store.residentCount() == 0u);
    REQUIRE_FALSE(store.findLatest().valid);
}

TEST_CASE("RemoteInputCache: the mismatch log gate opens exactly once per store",
          "[Network][RemoteInputCache]")
{
    // An incompatible peer re-replicates its ring forever; an ungated warning would
    // fire on every single replication.
    RemoteInputCache<StoreTestInput> store(gameZeroInput());

    REQUIRE(store.shouldLogVersionMismatchOnce());
    REQUIRE_FALSE(store.shouldLogVersionMismatchOnce());
    REQUIRE_FALSE(store.shouldLogVersionMismatchOnce());
}

// ---------------------------------------------------------------------------
// 6. RESYNC SURVIVAL, at the container level.
//
// The netsync-level proof (that wipeAllForResync really does leave the stores
// alone while wiping the delay lines) lives in og-brawler-tests, which can drive
// the real function. What is pinned HERE is the property that makes that safe: the
// store exposes no clear()/wipe surface at all, so there is nothing for a future
// mirror of those per-id map loops to call.
// ---------------------------------------------------------------------------

// Declared at file scope (a concept cannot be declared inside a function body, and
// the detection must be DEPENDENT — MSVC 14.38 treats a non-dependent
// `requires(RemoteInputCache<X>& s) { s.clear(); }` as a hard error rather than
// as an unsatisfied requirement).
template <typename T>
concept HasClearMethod = requires(T& t) { t.clear(); };

// LocalInputCache deliberately HAS clear(), because wipeAllForResync must call
// it; this type deliberately does NOT, because relayed entries are sender-domain
// capture ticks that a LOCAL resync does not invalidate.
static_assert(HasClearMethod<LocalInputCache<StoreTestInput>>,
    "sanity: the detection must actually detect a clear() where one exists");
static_assert(!HasClearMethod<RemoteInputCache<StoreTestInput>>,
    "RemoteInputCache must NOT grow a clear() - a resync must not be able to "
    "sweep sender-domain capture ticks (T5 naming ruling, reason 3)");

TEST_CASE("RemoteInputCache: the store has no wipe surface to be swept by a resync",
          "[Network][RemoteInputCache]")
{
    RemoteInputCache<StoreTestInput> store(gameZeroInput());
    REQUIRE(store.push(400u, 5u, taggedInput(400)));
    REQUIRE(store.has(400u));
}

#endif // WITH_LOW_LEVEL_TESTS
