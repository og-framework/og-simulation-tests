// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ServerReceptionCoordinator.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
// Sibling-relative, matching the Determinism/ + other Network/ suites — the test
// tree is not on an include root under either UBT or the standalone CMake build.
#include "MockSimulatablesForNetworkTests.h"
#include "StandaloneTestHandle.h"

// ---------------------------------------------------------------------------
// Coverage for ServerReceptionCoordinator<Address, SimulatableTs...> (T20 of
// og-netcode-v2-arch-latency — the engine-boundary refactor). Exercises the four
// relocated surface methods + the forgetOwner lifecycle hook + the dedup signal,
// instantiated on the engine-free FStandaloneTestHandle and the two-type mock
// pack, with NO dependency on OGBrawler or any UE type.
//
// What these cases CAN and CANNOT prove: they drive every branch reachable
// without a live engine — the couch-coop two-slot split, the malformed-slot
// fallback + its once-ever warning, the per-BUNDLE tier feed staying separate
// from the per-SLOT input path, the drain routing through a mock deliver
// callback with the ORIGINAL capture tick, and the reap evicting an
// already-dead handle from all three owned containers. What they CANNOT model is
// the in-place GC liveness transition (a handle going dead under an unchanged
// map key) — FStandaloneTestHandle's aliveBit is part of its identity, so that
// path is PIE-only, exactly as the T4/T5 suites already document.
// ---------------------------------------------------------------------------

using TestCoordinator = ServerReceptionCoordinator<FStandaloneTestHandle, MockSimA, MockSimB>;

namespace
{
    FStandaloneTestHandle live(std::uint32_t id)  { return FStandaloneTestHandle(id, true); }
    FStandaloneTestHandle dead(std::uint32_t id)  { return FStandaloneTestHandle(id, false); }

    // A deliver callback that records every delivery and answers "owner alive".
    struct RecordingDeliver
    {
        struct Delivery { unsigned int id; std::uint32_t captureTick; int value; };
        std::vector<Delivery> calls;
        bool ownerAlive = true;

        bool operator()(unsigned int id, std::uint32_t captureTick, const int& input)
        {
            calls.push_back(Delivery{ id, captureTick, input });
            return ownerAlive;
        }
    };

    // A ConnectionTierSink (T23) that records every (id, tier) publish the core
    // drives through it. Its mere existence proves the concept is satisfiable; the
    // recorded calls prove the core's fire-on-change-only policy.
    struct RecordingTierSink
    {
        struct Publish { unsigned int id; std::uint8_t tier; };
        std::vector<Publish> calls;

        void sendConnectionTierToOwningClient(unsigned int id, std::uint8_t tier)
        {
            calls.push_back(Publish{ id, tier });
        }

        std::size_t countFor(unsigned int id) const
        {
            std::size_t n = 0;
            for (const Publish& p : calls) if (p.id == id) ++n;
            return n;
        }
        // Tier of this id's most recent publish; 0xFF if it was never published.
        std::uint8_t lastTierFor(unsigned int id) const
        {
            std::uint8_t t = 0xFF;
            for (const Publish& p : calls) if (p.id == id) t = p.tier;
            return t;
        }
    };
    static_assert(ConnectionTierSink<RecordingTierSink>,
        "the mock sink must satisfy the concept it is standing in for");
}

// ---------------------------------------------------------------------------
// T24 — receiveInputBundle + the RemoteInputDeliverySink. Exercising the folded
// per-slot loop needs a Serializable input type (the redundancy codec serializes
// each slot) and a codec-Buffer, so this section uses its own single-sim pack
// `BundleSim` (Input = BundleInput) instead of MockSimA (Input = int, which the
// codec cannot serialize). The buffer + serializable-input pattern mirrors the
// existing WireFormat_Bundle.cpp suite.
// ---------------------------------------------------------------------------
namespace
{
    // Trivial Serializable slot input: one int32 field (syncSize == 4), exactly
    // like a brawler PlayerInput field. In the anonymous namespace (internal
    // linkage); its SerializableFields specialization is at global scope below,
    // matching WireFormat_Bundle.cpp.
    struct BundleInput
    {
        std::int32_t value = 0;
    };
}

template <>
struct SerializableFields<BundleInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(BundleInput, value));
    }
};

namespace
{
    struct BundleSim
    {
        using Input = BundleInput;
    };

    using BundleCoordinator = ServerReceptionCoordinator<FStandaloneTestHandle, BundleSim>;

    // UE-free byte buffer satisfying the codec's Buffer concept — what the
    // FInputRedundancyBundle USTRUCT supplies in production, without the engine.
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

    // Builds a bundle buffer from a list of (captureTick, value) slots.
    TestBundleBuffer makeBundle(std::initializer_list<std::pair<std::uint32_t, std::int32_t>> slots)
    {
        TestBundleBuffer buf;
        for (const auto& s : slots)
            inputRedundancyBundle::appendSlot<BundleInput>(buf, s.first, BundleInput{ s.second });
        return buf;
    }

    // A RemoteInputDeliverySink (T24) recording every deliver-now the core drives
    // through it. Its existence proves the concept is satisfiable; the recorded
    // calls prove routing (id) and the ORIGINAL captureTick.
    struct RecordingDeliverySink
    {
        struct Delivery { unsigned int id; std::uint32_t captureTick; std::int32_t value; };
        std::vector<Delivery> calls;

        void deliverRemoteInput(unsigned int id, std::uint32_t captureTick, const BundleInput& in)
        {
            calls.push_back(Delivery{ id, captureTick, in.value });
        }

        std::size_t countFor(unsigned int id) const
        {
            std::size_t n = 0;
            for (const Delivery& d : calls) if (d.id == id) ++n;
            return n;
        }
    };
    static_assert(RemoteInputDeliverySink<RecordingDeliverySink, BundleInput>,
        "the mock delivery sink must satisfy the concept it is standing in for");
}

TEST_CASE("ReceptionCoordinator parks two couch-coop slots independently", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);

    // Same wire, two local players (slot 0 + slot 1), each capturing on the same
    // tick — the exact conflation the (Address, slot) key exists to prevent.
    const ReceiveRemoteInputResult a =
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42);
    const ReceiveRemoteInputResult b =
        coord.receiveRemoteInput<MockSimA>(/*id=*/11, wire, /*slot=*/1, /*captureTick=*/100, 43);

    REQUIRE(a.parked);
    REQUIRE(b.parked);
    REQUIRE(coord.claimCount() == 2);

    // Two distinct slots parked for the one wire — not collapsed onto one deque.
    REQUIRE(coord.delayQueue().slotCountFor<MockSimA>(wire) == 2);
}

TEST_CASE("ReceptionCoordinator falls back and warns once on a malformed slot", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    std::vector<std::string> logged;
    coord.setLogger([&logged](const char* msg) { logged.emplace_back(msg); });

    const FStandaloneTestHandle wire = live(1);
    const std::uint8_t badSlot = ConnectionSlotKey<FStandaloneTestHandle>::kMaxPlayerSlot + 1;

    const ReceiveRemoteInputResult first =
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, badSlot, /*captureTick=*/100, 42);
    const ReceiveRemoteInputResult second =
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, badSlot, /*captureTick=*/101, 43);

    // Not parked => the adapter must take the legacy undelayed path.
    REQUIRE_FALSE(first.parked);
    REQUIRE_FALSE(second.parked);
    REQUIRE(coord.claimCount() == 0);

    // Warned exactly once for that (id, slot) despite two attempts.
    REQUIRE(logged.size() == 1);
    REQUIRE(logged[0].find("[Warning]") != std::string::npos);
}

TEST_CASE("ReceptionCoordinator feeds the tier EMA per bundle, not per slot", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingTierSink sink;

    // ONE bundle-level RTT sample seeds the tier EMA.
    coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/0, /*rttMs=*/50.0, sink);
    const auto* seeded = coord.tierTable().findState(wire);
    REQUIRE(seeded != nullptr);
    REQUIRE(seeded->smoothedRttMs == Catch::Approx(50.0));

    // Several PER-SLOT input receipts must NOT touch the tier EMA — that is the
    // whole point of splitting noteRttSample (per bundle) from receiveRemoteInput
    // (per slot). If receiveRemoteInput ever fed onRttSample, this would drift.
    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 1);
    coord.receiveRemoteInput<MockSimA>(/*id=*/11, wire, /*slot=*/1, /*captureTick=*/100, 2);
    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/101, 3);

    const auto* afterInputs = coord.tierTable().findState(wire);
    REQUIRE(afterInputs != nullptr);
    REQUIRE(afterInputs->smoothedRttMs == Catch::Approx(50.0));

    // And a negative sample is skipped: no EMA update, and — the T23 policy that
    // moved into the core — no publish.
    const std::size_t before = sink.calls.size();
    coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/1, /*rttMs=*/-1.0, sink);
    REQUIRE(sink.calls.size() == before);
}

TEST_CASE("ReceptionCoordinator drain delivers the original capture tick", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);

    // No tier sample => effectiveDelay falls back to forcedInputLatencyTicks.
    const std::int32_t delay = cfg.forcedInputLatencyTicks;
    const std::int32_t captureTick = 100;

    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, captureTick, 42).parked);

    RecordingDeliver deliver;

    // Nothing due one tick early.
    coord.releaseDelayedInputs<MockSimA>(captureTick + delay - 1, /*numSteps=*/1, std::ref(deliver));
    REQUIRE(deliver.calls.empty());

    // Due exactly at captureTick + delay.
    coord.releaseDelayedInputs<MockSimA>(captureTick + delay, /*numSteps=*/1, std::ref(deliver));
    REQUIRE(deliver.calls.size() == 1);
    REQUIRE(deliver.calls[0].id == 10);
    REQUIRE(deliver.calls[0].captureTick == static_cast<std::uint32_t>(captureTick)); // ORIGINAL tick
    REQUIRE(deliver.calls[0].value == 42);
}

TEST_CASE("ReceptionCoordinator drain drops the claim when the owner reports dead", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t delay = cfg.forcedInputLatencyTicks;

    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42).parked);
    REQUIRE(coord.claimCount() == 1);

    RecordingDeliver deliver;
    deliver.ownerAlive = false;     // owner GC'd without an unregister

    coord.releaseDelayedInputs<MockSimA>(100 + delay, /*numSteps=*/1, std::ref(deliver));

    // Deliver was attempted, reported dead, and the stale claim was pruned.
    REQUIRE(deliver.calls.size() == 1);
    REQUIRE(coord.claimCount() == 0);
}

TEST_CASE("ReceptionCoordinator reaps a dead wire from all three containers", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle liveWire = live(1);
    const FStandaloneTestHandle deadWire = dead(2);   // already-dead: drives the !isAlive branch

    // Populate all three containers for BOTH wires. A throwaway sink absorbs the
    // (irrelevant-here) tier publishes.
    RecordingTierSink sink;
    coord.noteRttSample(liveWire, /*ownerId=*/10, /*serverTick=*/0, /*rttMs=*/40.0, sink);
    coord.noteRttSample(deadWire, /*ownerId=*/11, /*serverTick=*/0, /*rttMs=*/40.0, sink);
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, liveWire, /*slot=*/0, /*captureTick=*/1, 1).parked);
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/11, deadWire, /*slot=*/0, /*captureTick=*/1, 2).parked);

    REQUIRE(coord.tierTable().hasEntry(deadWire));
    REQUIRE(coord.delayQueue().hasConnection(deadWire));
    REQUIRE(coord.hasClaim(ConnectionSlotKey<FStandaloneTestHandle>(deadWire, 0)));

    // Reap on a dwell-boundary tick (dwell divides it). The dead wire is evicted
    // regardless of deadline via the !isAlive branch; the live wire survives.
    const std::int32_t boundaryTick = cfg.tierMinDwellTicks; // serverTick % dwell == 0
    coord.reapConnections(boundaryTick);

    REQUIRE_FALSE(coord.tierTable().hasEntry(deadWire));
    REQUIRE_FALSE(coord.delayQueue().hasConnection(deadWire));
    REQUIRE_FALSE(coord.hasClaim(ConnectionSlotKey<FStandaloneTestHandle>(deadWire, 0)));

    // The live wire is untouched across all three.
    REQUIRE(coord.tierTable().hasEntry(liveWire));
    REQUIRE(coord.delayQueue().hasConnection(liveWire));
    REQUIRE(coord.hasClaim(ConnectionSlotKey<FStandaloneTestHandle>(liveWire, 0)));
}

TEST_CASE("ReceptionCoordinator reap is gated on the dwell boundary", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle deadWire = dead(2);
    RecordingTierSink sink;
    coord.noteRttSample(deadWire, /*ownerId=*/11, /*serverTick=*/0, /*rttMs=*/40.0, sink);
    REQUIRE(coord.tierTable().hasEntry(deadWire));

    // A NON-boundary tick must not reap (this is what makes the per-tick call in
    // T21 cheap — the internal modulo gate keeps the real frequency unchanged).
    coord.reapConnections(cfg.tierMinDwellTicks + 1);
    REQUIRE(coord.tierTable().hasEntry(deadWire));

    // The next boundary reaps it.
    coord.reapConnections(cfg.tierMinDwellTicks * 2);
    REQUIRE_FALSE(coord.tierTable().hasEntry(deadWire));
}

TEST_CASE("ReceptionCoordinator forgetOwner drops the claim on unregister", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42).parked);
    REQUIRE(coord.claimCount() == 1);

    coord.forgetOwner(10);
    REQUIRE(coord.claimCount() == 0);

    // A different owner's claim is unaffected.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/101, 43).parked);
    coord.forgetOwner(99);
    REQUIRE(coord.claimCount() == 1);
}

TEST_CASE("ReceptionCoordinator surfaces the accept-vs-duplicate signal", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);

    // First sight of a capture tick for this id => accepted-new.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42).acceptedNew);
    // Redundancy re-send of the same tick => duplicate (still parked-attempted).
    REQUIRE_FALSE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42).acceptedNew);
    // A newer tick => accepted-new again.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/101, 43).acceptedNew);
    // An older tick (late/reordered) => not new.
    REQUIRE_FALSE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 44).acceptedNew);

    // The watermark is per id — a different id starts fresh.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/11, wire, /*slot=*/1, /*captureTick=*/50, 7).acceptedNew);
}

// ---------------------------------------------------------------------------
// T23 — the tier SEND is now core-driven through a ConnectionTierSink. These
// cases exercise noteRttSample's fire-on-change-only publish policy against a
// mock sink, with NO UE dependency. A tier transition needs `tierMinDwellTicks`
// samples above `boundary + hysteresis` to clear both anti-flap gates (see
// ConnectionTierTable); 60 ms sits in the tier-1 band ( >40, <90 ) for the
// default config, so a full dwell of 60 ms samples produces exactly one 0->1
// transition and therefore exactly one publish.
// ---------------------------------------------------------------------------

TEST_CASE("ReceptionCoordinator publishes the tier through the sink on a change", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    const FStandaloneTestHandle wire = live(1);
    RecordingTierSink sink;

    // One dwell of tier-1-band samples: the first dwell-1 produce no transition
    // (and so no publish); the dwell-crossing sample transitions 0 -> 1 and fires
    // the sink exactly once, with this owner's id and the new tier.
    for (int32_t t = 0; t < cfg.tierMinDwellTicks; ++t)
    {
        coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/t, /*rttMs=*/60.0, sink);
    }

    REQUIRE(sink.calls.size() == 1);
    REQUIRE(sink.calls[0].id == 10);
    REQUIRE(sink.calls[0].tier == 1);
}

TEST_CASE("ReceptionCoordinator does not re-publish an unchanged tier", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    const FStandaloneTestHandle wire = live(1);
    RecordingTierSink sink;

    // Drive to tier 1 (one publish)...
    for (int32_t t = 0; t < cfg.tierMinDwellTicks; ++t)
        coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/t, /*rttMs=*/60.0, sink);
    REQUIRE(sink.calls.size() == 1);

    // ...then keep sampling the SAME band. The tier stays 1, so the wire is never
    // dirtied again — the publish-only-on-change dedup (now core-owned) holds.
    for (int32_t t = 0; t < cfg.tierMinDwellTicks; ++t)
        coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/cfg.tierMinDwellTicks + t, /*rttMs=*/60.0, sink);

    REQUIRE(sink.calls.size() == 1);   // still just the single 0 -> 1 transition
}

TEST_CASE("ReceptionCoordinator publishes nothing on a negative reading", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    const FStandaloneTestHandle wire = live(1);
    RecordingTierSink sink;

    // rttMs < 0 is the engine "no reading yet" sentinel — the core skips it before
    // touching the tier table, so nothing is sampled and nothing is published.
    coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/0, /*rttMs=*/-1.0, sink);

    REQUIRE(sink.calls.empty());
    REQUIRE(coord.tierTable().findState(wire) == nullptr);   // not even sampled
}

TEST_CASE("ReceptionCoordinator publishes once per bundle, not per slot", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    const FStandaloneTestHandle wire = live(1);
    RecordingTierSink sink;

    // Park the wire one sample short of the dwell-crossing: still tier 0, no publish.
    for (int32_t t = 0; t < cfg.tierMinDwellTicks - 1; ++t)
        coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/t, /*rttMs=*/60.0, sink);
    REQUIRE(sink.calls.empty());

    // A bundle's worth of PER-SLOT input receipts must not sample or publish — the
    // tier feed is per BUNDLE (noteRttSample), the input path is per SLOT.
    for (int32_t s = 0; s < 8; ++s)
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/200 + s, s);
    REQUIRE(sink.calls.empty());

    // The ONE bundle-level sample that crosses the dwell fires the sink exactly
    // once — not once per slot of the bundle that carried the RTT.
    coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/cfg.tierMinDwellTicks, /*rttMs=*/60.0, sink);
    REQUIRE(sink.calls.size() == 1);
    REQUIRE(sink.calls[0].tier == 1);
}

TEST_CASE("ReceptionCoordinator publishes to both couch-coop owners sharing one wire", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    // ONE root connection (Address)...
    const FStandaloneTestHandle wire = live(1);
    RecordingTierSink sink;

    // ...two local players (two owner ids) on it. Each carries its owning
    // component's id into the per-bundle sample. The tier table is keyed on the
    // shared Address, so ONE of the two samples triggers the transition — but the
    // publish dedup is keyed on ownerId, so BOTH owners must still be told once
    // (a per-Address dedup would tell the transition-triggering sibling and starve
    // the other; that is the couch-coop regression this pins against).
    for (int32_t t = 0; t < cfg.tierMinDwellTicks; ++t)
    {
        coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/t, /*rttMs=*/60.0, sink);
        coord.noteRttSample(wire, /*ownerId=*/11, /*serverTick=*/t, /*rttMs=*/60.0, sink);
    }

    REQUIRE(sink.countFor(10) == 1);
    REQUIRE(sink.countFor(11) == 1);
    REQUIRE(sink.lastTierFor(10) == 1);
    REQUIRE(sink.lastTierFor(11) == 1);
}

// ---------------------------------------------------------------------------
// T24 — the per-slot receive loop is now core (receiveInputBundle), decoding the
// wire bundle and either parking each slot or, on a malformed slot, delivering it
// immediately through the RemoteInputDeliverySink. These pin: parked input is NOT
// delivered now, a malformed slot IS, and couch-coop two-slot delivery carries the
// right id to the sink.
// ---------------------------------------------------------------------------

TEST_CASE("ReceptionCoordinator receiveInputBundle parks valid slots without delivering now", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    BundleCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingDeliverySink deliver;

    // A two-slot bundle on a valid player slot => both slots park; NOTHING is
    // delivered now (the drain releases them later).
    const TestBundleBuffer bundle = makeBundle({ { 100u, 42 }, { 101u, 43 } });
    coord.receiveInputBundle<BundleSim>(/*id=*/10, wire, /*slot=*/0, bundle, deliver);

    REQUIRE(deliver.calls.empty());
    REQUIRE(coord.claimCount() == 1);
    // Both ticks parked into the ONE slot-0 bucket for this wire.
    REQUIRE(coord.delayQueue().slotCountFor<BundleSim>(wire) == 1);

    // And BOTH parked inputs ARE delivered by the drain, each at captureTick +
    // delay, with the ORIGINAL capture tick — through the drain's own callback (the
    // UE side routes that callback through the SAME deliverRemoteInput sink method).
    const std::int32_t delay = cfg.forcedInputLatencyTicks;
    std::vector<std::uint32_t> drainTicks;
    auto drainDeliver = [&](unsigned int, std::uint32_t tick, const BundleInput&) -> bool
    {
        drainTicks.push_back(tick);
        return true;
    };
    coord.releaseDelayedInputs<BundleSim>(100 + delay, /*numSteps=*/2, drainDeliver);
    REQUIRE(drainTicks.size() == 2);
    REQUIRE(drainTicks[0] == 100u);
    REQUIRE(drainTicks[1] == 101u);
}

TEST_CASE("ReceptionCoordinator receiveInputBundle delivers a malformed slot immediately", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    BundleCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    const std::uint8_t badSlot = ConnectionSlotKey<FStandaloneTestHandle>::kMaxPlayerSlot + 1;
    RecordingDeliverySink deliver;

    // The delay queue refuses a malformed slot => the core delivers it NOW through
    // the sink so no player input is dropped; nothing is parked.
    const TestBundleBuffer bundle = makeBundle({ { 200u, 7 }, { 201u, 8 } });
    coord.receiveInputBundle<BundleSim>(/*id=*/10, wire, badSlot, bundle, deliver);

    REQUIRE(coord.claimCount() == 0);
    REQUIRE(deliver.calls.size() == 2);
    REQUIRE(deliver.calls[0].id == 10);
    REQUIRE(deliver.calls[0].captureTick == 200u);   // ORIGINAL tick
    REQUIRE(deliver.calls[0].value == 7);
    REQUIRE(deliver.calls[1].captureTick == 201u);
    REQUIRE(deliver.calls[1].value == 8);
}

TEST_CASE("ReceptionCoordinator receiveInputBundle routes each couch-coop owner to the right id", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    BundleCoordinator coord(cfg);

    // Two couch-coop owners share ONE wire. Each is delivered-now here (malformed
    // slot) so the delivery is observable through the sink; the point is that the
    // sink is handed the RIGHT id + captureTick for each — a shared sink resolving
    // by id, exactly the UE manager's role.
    const FStandaloneTestHandle wire = live(1);
    const std::uint8_t badSlot = ConnectionSlotKey<FStandaloneTestHandle>::kMaxPlayerSlot + 1;
    RecordingDeliverySink deliver;

    coord.receiveInputBundle<BundleSim>(/*id=*/10, wire, badSlot, makeBundle({ { 300u, 1 } }), deliver);
    coord.receiveInputBundle<BundleSim>(/*id=*/11, wire, badSlot, makeBundle({ { 400u, 2 } }), deliver);

    REQUIRE(deliver.calls.size() == 2);
    REQUIRE(deliver.countFor(10) == 1);
    REQUIRE(deliver.countFor(11) == 1);

    // Each owner's delivery carries its own captureTick + value — not conflated.
    for (const auto& d : deliver.calls)
    {
        if (d.id == 10) { REQUIRE(d.captureTick == 300u); REQUIRE(d.value == 1); }
        if (d.id == 11) { REQUIRE(d.captureTick == 400u); REQUIRE(d.value == 2); }
    }
}

// ---------------------------------------------------------------------------
// T25 — input-path diagnostics. These pin the two Warning-severity drop signals
// against a mock logger: [InputDrop] (a parked input stranded by a delay change
// then reclaimed by the purge), [InputGap] (a hole in an id's delivered capture
// ticks), and [DelayShift] (a wire's effective delay changing between drains) —
// and prove NONE of the drop signals fire on a clean contiguous delivery. The
// logger captures every SIMLOG line; a leading [Warning] token is present on the
// escalated ones (the same token RouteOGMessage strips to pick Warning severity).
// ---------------------------------------------------------------------------
namespace
{
    bool hasLineContaining(const std::vector<std::string>& lines, const char* needle)
    {
        for (const std::string& s : lines)
        {
            if (s.find(needle) != std::string::npos) return true;
        }
        return false;
    }
}

TEST_CASE("ReceptionCoordinator logs [InputDrop] when a parked input is stranded", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    std::vector<std::string> logged;
    coord.setLogger([&logged](const char* m) { logged.emplace_back(m); });

    const FStandaloneTestHandle wire = live(1);

    // Park at the fallback delay: no tier entry yet => forcedInputLatencyTicks (2),
    // so its release tick is captureTick + 2 == 102.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42).parked);

    // Bump the tier: a single low-RTT sample creates a tier-0 entry, so
    // effectiveDelay drops 2 -> 1 and the parked input's release tick shifts
    // 102 -> 101. The exact-match dequeue (T25 KEEPS this; T26 is the fix) now only
    // fires at tick 101 — the classic strand.
    RecordingTierSink sink;
    coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/0, /*rttMs=*/10.0, sink);

    // Drain PAST tick 101 (a skipped tick) and past the rollback window: the input
    // is never released and the purge reclaims it, producing [InputDrop].
    RecordingDeliver deliver;
    coord.releaseDelayedInputs<MockSimA>(/*firstUpcomingSimTick=*/121, /*numSteps=*/1, std::ref(deliver));

    REQUIRE(deliver.calls.empty());                     // stranded — never delivered
    REQUIRE(hasLineContaining(logged, "[InputDrop]"));
    REQUIRE(hasLineContaining(logged, "[Warning]"));    // escalated to Warning
    REQUIRE(hasLineContaining(logged, "captureTick=100"));
}

TEST_CASE("ReceptionCoordinator logs [InputGap] on a hole in delivered ticks", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    std::vector<std::string> logged;
    coord.setLogger([&logged](const char* m) { logged.emplace_back(m); });

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t delay = cfg.forcedInputLatencyTicks;   // 2, no tier entry

    // Park tick 100 and tick 103 — ticks 101 and 102 are missing (dropped on the
    // wire before they ever reached the server).
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 1).parked);
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/103, 2).parked);

    // Drain across the release ticks of both (102 and 105 at delay 2).
    RecordingDeliver deliver;
    coord.releaseDelayedInputs<MockSimA>(100 + delay, /*numSteps=*/4, std::ref(deliver));

    // Both delivered, with the original capture ticks and a 2-tick hole between.
    REQUIRE(deliver.calls.size() == 2);
    REQUIRE(deliver.calls[0].captureTick == 100u);
    REQUIRE(deliver.calls[1].captureTick == 103u);

    REQUIRE(hasLineContaining(logged, "[InputGap]"));
    REQUIRE(hasLineContaining(logged, "dropped=2"));
}

TEST_CASE("ReceptionCoordinator logs no gap or drop on clean contiguous delivery", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    std::vector<std::string> logged;
    coord.setLogger([&logged](const char* m) { logged.emplace_back(m); });

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t delay = cfg.forcedInputLatencyTicks;   // 2

    // Three contiguous capture ticks, all parked, all releasable in order.
    for (std::int32_t t = 100; t <= 102; ++t)
    {
        REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, t, static_cast<int>(t)).parked);
    }

    RecordingDeliver deliver;
    coord.releaseDelayedInputs<MockSimA>(100 + delay, /*numSteps=*/3, std::ref(deliver));  // ticks 102,103,104

    REQUIRE(deliver.calls.size() == 3);
    REQUIRE_FALSE(hasLineContaining(logged, "[InputGap]"));    // contiguous — no hole
    REQUIRE_FALSE(hasLineContaining(logged, "[InputDrop]"));   // nothing stranded/purged
}

TEST_CASE("ReceptionCoordinator logs [DelayShift] when a wire's effective delay changes", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    std::vector<std::string> logged;
    coord.setLogger([&logged](const char* m) { logged.emplace_back(m); });

    const FStandaloneTestHandle wire = live(1);

    // First drain establishes the wire's baseline delay (forced = 2). A first
    // observation logs no shift.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 1).parked);
    RecordingDeliver deliver;
    coord.releaseDelayedInputs<MockSimA>(102, /*numSteps=*/1, std::ref(deliver));
    REQUIRE(deliver.calls.size() == 1);
    REQUIRE_FALSE(hasLineContaining(logged, "[DelayShift]"));

    // A low-RTT sample creates a tier-0 entry => effectiveDelay 2 -> 1. Park + drain
    // again; the wire's delay differs from the memo, so [DelayShift] fires.
    RecordingTierSink sink;
    coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/0, /*rttMs=*/10.0, sink);
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/200, 2).parked);
    coord.releaseDelayedInputs<MockSimA>(201, /*numSteps=*/1, std::ref(deliver));

    REQUIRE(hasLineContaining(logged, "[DelayShift]"));
}

// ---------------------------------------------------------------------------
// T26 — due-or-overdue release + the fable amendments, at the coordinator drain.
// The queue-level suite pins the predicate mechanics; these pin the DRAIN: that
// an overdue input is delivered late (not stranded) with its TRUE stored capture
// tick (F1), that a delay decrease no longer strands the parked input, and that a
// beyond-rollback-window entry is PURGED not released (F3 — the single drop
// point). The in-time steady path is unchanged (the existing "drain delivers the
// original capture tick" case above already pins that at exact-due).
// ---------------------------------------------------------------------------

TEST_CASE("ReceptionCoordinator drain releases an overdue input with its true capture tick", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t delay = cfg.forcedInputLatencyTicks;     // 2, no tier entry
    const std::int32_t captureTick = 100;
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, captureTick, 42).parked);

    RecordingDeliver deliver;

    // Skip the exact due tick (102). Drain several ticks late but WITHIN the
    // rollback window (staleBefore = 107 - 20 = 87 <= 100). The input is delivered
    // late instead of stranded, and the delivered captureTick is the TRUE stored
    // tick (F1) — NOT the reconstructed simTick - delay (== 105).
    coord.releaseDelayedInputs<MockSimA>(/*firstUpcomingSimTick=*/107, /*numSteps=*/1, std::ref(deliver));

    REQUIRE(deliver.calls.size() == 1);
    REQUIRE(deliver.calls[0].captureTick == static_cast<std::uint32_t>(captureTick));  // 100 (F1)
    REQUIRE(deliver.calls[0].captureTick != static_cast<std::uint32_t>(107 - delay));  // not 105
    REQUIRE(deliver.calls[0].value == 42);
}

TEST_CASE("ReceptionCoordinator drain still delivers after a delay decrease", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);

    // Park at the fallback delay (forced = 2): exact due tick would be 102.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42).parked);

    // A low-RTT sample creates a tier-0 entry => effectiveDelay drops 2 -> 1, so
    // the exact due tick shifts 102 -> 101. Under the old exact-match a drain at
    // 102 would MISS the shifted tick and strand the input; due-or-overdue delivers
    // it late (101 <= 102, in window).
    RecordingTierSink sink;
    coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/0, /*rttMs=*/10.0, sink);

    RecordingDeliver deliver;
    coord.releaseDelayedInputs<MockSimA>(/*firstUpcomingSimTick=*/102, /*numSteps=*/1, std::ref(deliver));

    REQUIRE(deliver.calls.size() == 1);
    REQUIRE(deliver.calls[0].captureTick == 100u);      // true tick, not stranded
    REQUIRE(deliver.calls[0].value == 42);
}

#endif // WITH_LOW_LEVEL_TESTS
