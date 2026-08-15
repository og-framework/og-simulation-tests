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

// ---------------------------------------------------------------------------
// T12 — the OUT-OF-DOMAIN RECEIPT GATE (og-netcode-v2-input-relay;
// RelayDelaySpectrumDesign.md §8.1). At receipt, before the dedup watermark and
// before park/claim/relay, a capture tick outside
//     [serverTick - rollbackWindowHardCap, serverTick + hardResyncThresholdTicks]
// is refused: not parked, not claimed, not delivered — counted and summarised in
// one rate-limited [Warning][InputDomain] line per window.
//
// The gate's tick reference is armed ONLY by reapConnections (the game-thread
// drain hook the adapter already drives every physics frame), which is why every
// case below calls it first and why the pre-T12 cases above — which never drive a
// tick — are completely unaffected (the last case here pins that fail-open
// property explicitly).
//
// Bounds are read from the SAME TimeConfig the coordinator borrows, never
// hardcoded, so these cases keep pinning the real window if a default moves.
// ---------------------------------------------------------------------------

namespace
{
    // The gate's accepted window for a given server tick, derived exactly as the
    // production code derives it (both bounds inclusive).
    struct DomainWindow
    {
        std::int32_t lower;
        std::int32_t upper;
    };

    DomainWindow domainWindowFor(const TimeConfig& cfg, std::int32_t serverTick)
    {
        return DomainWindow{
            serverTick - cfg.rollbackWindowHardCap,
            serverTick + static_cast<std::int32_t>(cfg.hardResyncThresholdTicks) };
    }
}

TEST_CASE("ReceptionCoordinator rejects a capture tick below the domain window", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t serverTick = 600;
    const DomainWindow w = domainWindowFor(cfg, serverTick);

    coord.reapConnections(serverTick);       // arm the gate

    const ReceiveRemoteInputResult r =
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, w.lower - 1, 42);

    REQUIRE(r.rejectedOutOfDomain);
    REQUIRE_FALSE(r.parked);
    REQUIRE_FALSE(r.acceptedNew);            // the relay signal must not fire either

    // Nothing reached the queue, the claim map, or the relay tap.
    REQUIRE(coord.claimCount() == 0);
    REQUIRE_FALSE(coord.delayQueue().hasConnection(wire));
    REQUIRE(coord.outOfDomainRejectCount() == 1);
}

TEST_CASE("ReceptionCoordinator rejects a capture tick above the domain window", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t serverTick = 600;
    const DomainWindow w = domainWindowFor(cfg, serverTick);

    coord.reapConnections(serverTick);

    const ReceiveRemoteInputResult r =
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, w.upper + 1, 42);

    REQUIRE(r.rejectedOutOfDomain);
    REQUIRE_FALSE(r.parked);
    REQUIRE(coord.claimCount() == 0);
    REQUIRE(coord.outOfDomainRejectCount() == 1);
}

TEST_CASE("ReceptionCoordinator pins both domain-window edges as inclusive", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t serverTick = 600;
    const DomainWindow w = domainWindowFor(cfg, serverTick);

    // The window is [serverTick - 20, serverTick + 21] at the defaults, and the
    // ordering invariant hardResyncThresholdTicks > rollbackWindowHardCap keeps it
    // non-empty.
    REQUIRE(w.lower == serverTick - cfg.rollbackWindowHardCap);
    REQUIRE(w.upper == serverTick + static_cast<std::int32_t>(cfg.hardResyncThresholdTicks));
    REQUIRE(w.upper > w.lower);

    coord.reapConnections(serverTick);

    // BOTH EDGES ARE INSIDE the accepted window (inclusive), and the first tick
    // outside on each side is refused. Distinct slots so each park is independent.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, w.lower, 1).parked);
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/11, wire, /*slot=*/1, w.upper, 2).parked);
    REQUIRE(coord.outOfDomainRejectCount() == 0);        // neither edge was refused

    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/12, wire, /*slot=*/2, w.lower - 1, 3).rejectedOutOfDomain);
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/13, wire, /*slot=*/3, w.upper + 1, 4).rejectedOutOfDomain);
    REQUIRE(coord.outOfDomainRejectCount() == 2);

    // Only the two in-window edges parked.
    REQUIRE(coord.claimCount() == 2);
    REQUIRE(coord.delayQueue().slotCountFor<MockSimA>(wire) == 2);
}

TEST_CASE("ReceptionCoordinator refuses a warm-up client's free-running capture ticks", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    std::vector<std::string> logged;
    coord.setLogger([&logged](const char* m) { logged.emplace_back(m); });

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t serverTick = 600;

    coord.reapConnections(serverTick);

    // THE MOTIVATING SCENARIO: a client that has not yet been anchored to the
    // server's numbering emits its own counter, 0..59, while the server is at 600.
    // Every one of those is outside [580, 621] and must die here — once T3 makes
    // receipt-time input peer-visible, relaying them would push a foreign timeline
    // onto every other client.
    for (std::int32_t t = 0; t < 60; ++t)
    {
        const ReceiveRemoteInputResult r =
            coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, t, static_cast<int>(t));
        REQUIRE(r.rejectedOutOfDomain);
        REQUIRE_FALSE(r.parked);        // NEVER parked
        REQUIRE_FALSE(r.acceptedNew);   // NEVER relayed (the T3 relay gate)
    }

    REQUIRE(coord.outOfDomainRejectCount() == 60);
    REQUIRE(coord.claimCount() == 0);
    REQUIRE_FALSE(coord.delayQueue().hasConnection(wire));

    // 60 rejects produced ZERO log lines so far — the summary is rate-limited onto
    // the window boundary, never emitted per rejection.
    REQUIRE_FALSE(hasLineContaining(logged, "[InputDomain]"));

    // And the drain has nothing to release — the garbage never entered the queue.
    RecordingDeliver deliver;
    coord.releaseDelayedInputs<MockSimA>(serverTick, /*numSteps=*/4, std::ref(deliver));
    REQUIRE(deliver.calls.empty());

    // Once the same client IS anchored, its input flows normally through the very
    // same coordinator — the gate is not sticky per id/wire.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, serverTick + 2, 7).parked);
    REQUIRE(coord.outOfDomainRejectCount() == 60);
}

TEST_CASE("ReceptionCoordinator does not deliver an out-of-domain bundle slot", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    BundleCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingDeliverySink deliver;
    const std::int32_t serverTick = 600;

    coord.reapConnections(serverTick);

    // A malformed slot normally falls back to UNDELAYED DELIVERY (the case above
    // pins that). An out-of-domain tick must NOT take that fallback — it is the one
    // false-parked path that discards, otherwise the gate would be a no-op for any
    // client that also has a broken slot.
    const std::uint8_t badSlot = ConnectionSlotKey<FStandaloneTestHandle>::kMaxPlayerSlot + 1;
    coord.receiveInputBundle<BundleSim>(/*id=*/10, wire, badSlot, makeBundle({ { 5u, 1 } }), deliver);
    REQUIRE(deliver.calls.empty());
    REQUIRE(coord.outOfDomainRejectCount() == 1);

    // A VALID slot carrying an out-of-domain tick: not parked, not delivered.
    coord.receiveInputBundle<BundleSim>(/*id=*/10, wire, /*slot=*/0, makeBundle({ { 5u, 1 } }), deliver);
    REQUIRE(deliver.calls.empty());
    REQUIRE(coord.claimCount() == 0);
    REQUIRE(coord.outOfDomainRejectCount() == 2);

    // In-domain slots on the same wire are parked exactly as before the gate.
    coord.receiveInputBundle<BundleSim>(
        /*id=*/10, wire, /*slot=*/0,
        makeBundle({ { static_cast<std::uint32_t>(serverTick), 2 } }), deliver);
    REQUIRE(deliver.calls.empty());
    REQUIRE(coord.claimCount() == 1);
    REQUIRE(coord.outOfDomainRejectCount() == 2);
}

TEST_CASE("ReceptionCoordinator does not let a rejected tick poison the dedup watermark", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t serverTick = 600;
    const DomainWindow w = domainWindowFor(cfg, serverTick);

    coord.reapConnections(serverTick);

    // WHY THE GATE RUNS BEFORE noteCaptureTick. The watermark is a monotonic max;
    // a single wild future tick would raise it forever and every subsequent
    // LEGITIMATE input would then report acceptedNew == false — silently killing
    // the [Park] trace and, post-T3, the relay write itself.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, w.upper + 5000, 1)
                .rejectedOutOfDomain);

    // The very next in-window input is still ACCEPTED-NEW: the watermark never saw
    // the garbage.
    const ReceiveRemoteInputResult good =
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, serverTick, 2);
    REQUIRE(good.parked);
    REQUIRE(good.acceptedNew);
    REQUIRE_FALSE(good.rejectedOutOfDomain);
}

TEST_CASE("ReceptionCoordinator emits one rate-limited [InputDomain] line per burst", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    std::vector<std::string> logged;
    coord.setLogger([&logged](const char* m) { logged.emplace_back(m); });

    const FStandaloneTestHandle wire = live(1);
    const std::int32_t serverTick = 600;
    // The [InputStats] window (T25) the [InputDomain] line deliberately rides.
    const std::int32_t window = static_cast<std::int32_t>(2.0 * cfg.tickFrequency);

    coord.reapConnections(serverTick);       // arms the gate AND opens the window

    // A burst: 30 rejects inside one window must NOT produce 30 log lines.
    for (std::int32_t i = 0; i < 30; ++i)
    {
        REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, i, 1).rejectedOutOfDomain);
    }
    REQUIRE_FALSE(hasLineContaining(logged, "[InputDomain]"));   // nothing yet — rate-limited

    // Closing the window emits exactly ONE summary line, at Warning severity, with
    // the burst count and the most recent offender.
    coord.reapConnections(serverTick + window);
    REQUIRE(hasLineContaining(logged, "[InputDomain]"));
    REQUIRE(hasLineContaining(logged, "[Warning]"));
    REQUIRE(hasLineContaining(logged, "rejected 30 out-of-domain"));
    REQUIRE(hasLineContaining(logged, "captureTick=29"));

    std::size_t lines = 0;
    for (const std::string& s : logged)
    {
        if (s.find("[InputDomain]") != std::string::npos) ++lines;
    }
    REQUIRE(lines == 1);

    // A quiet window emits nothing more (the counter reset with the window), while
    // the LIFETIME total is preserved.
    coord.reapConnections(serverTick + 2 * window);
    REQUIRE(coord.outOfDomainRejectCount() == 30);
    lines = 0;
    for (const std::string& s : logged)
    {
        if (s.find("[InputDomain]") != std::string::npos) ++lines;
    }
    REQUIRE(lines == 1);
}

TEST_CASE("ReceptionCoordinator leaves in-domain receipt untouched and fails open unarmed", "[Network][ReceptionCoordinator]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    std::vector<std::string> logged;
    coord.setLogger([&logged](const char* m) { logged.emplace_back(m); });

    const FStandaloneTestHandle wire = live(1);

    // (a) FAIL OPEN BEFORE THE FIRST TICK REFERENCE. Nothing has armed the gate, so
    // it cannot judge and must accept — a gate that failed CLOSED here would drop
    // every player input on any path that never fed it. This is also exactly why
    // every pre-T12 case in this file is unaffected by the gate.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42).parked);
    REQUIRE(coord.outOfDomainRejectCount() == 0);
    REQUIRE_FALSE(hasLineContaining(logged, "[InputDomain]"));

    // (b) ARMED, IN-WINDOW: the accepted path is unchanged — parked, claimed,
    // accepted-new, no reject flag, no counter movement, and it drains normally at
    // captureTick + delay with its original tick.
    const std::int32_t serverTick = 600;
    coord.reapConnections(serverTick);

    const ReceiveRemoteInputResult r =
        coord.receiveRemoteInput<MockSimA>(/*id=*/11, wire, /*slot=*/1, serverTick + 3, 7);
    REQUIRE(r.parked);
    REQUIRE(r.acceptedNew);
    REQUIRE_FALSE(r.rejectedOutOfDomain);
    REQUIRE(coord.outOfDomainRejectCount() == 0);

    RecordingDeliver deliver;
    coord.releaseDelayedInputs<MockSimA>(serverTick + 3 + cfg.forcedInputLatencyTicks,
                                         /*numSteps=*/1, std::ref(deliver));
    REQUIRE(deliver.calls.size() == 1);
    REQUIRE(deliver.calls[0].captureTick == static_cast<std::uint32_t>(serverTick + 3));
    REQUIRE(deliver.calls[0].value == 7);
}

// ---------------------------------------------------------------------------
// T3 — THE RELAY TAP (og-netcode-v2-input-relay; InputRelayDesign.md §3a,
// RelayDelaySpectrumDesign.md §5). At receipt, each GENUINELY NEW capture tick is
// handed to a RemoteInputRelaySink stamped with the wire's effective delay at that
// moment (`dA`), so a peer can derive the application tick as `captureTick + dA`.
//
// WHAT THESE CASES PIN, in the order the risk lives:
//   * the stamp is the effective delay AT RECEIPT, and it follows a tier change on
//     the NEXT accepted input while already-relayed entries keep their own stamp;
//   * exactly-once per newer capture tick — a redundancy re-send relays nothing;
//   * an out-of-order-OLDER tick is APPLIED by the server but NOT relayed (the
//     stream is monotonic at depth 1), counted as relayOooSkipCount();
//   * NO relay on either false-parked path — the malformed-slot fallback (review
//     A6: `acceptedNew` is TRUE there, so a bare-acceptedNew gate would relay a
//     stamp promising a delay the authority is not applying) and the out-of-domain
//     rejection.
//
// The DUAL-WRITE / additivity half of the task is proven by every case ABOVE this
// section still passing unchanged: they call the same entry points without a relay
// sink, which binds the defaulted no-op sink, and none of their expectations moved.
// ---------------------------------------------------------------------------
namespace
{
    // A RemoteInputRelaySink (T3) recording every relay the core taps through it.
    struct RecordingRelaySink
    {
        struct Relayed
        {
            unsigned int  id;
            std::uint32_t captureTick;
            std::uint8_t  dA;
            int           value;
        };
        std::vector<Relayed> calls;

        void relayRemoteInput(unsigned int id, std::uint32_t captureTick,
                              std::uint8_t dA, const int& in)
        {
            calls.push_back(Relayed{ id, captureTick, dA, in });
        }

        std::size_t countFor(unsigned int id) const
        {
            std::size_t n = 0;
            for (const Relayed& r : calls) if (r.id == id) ++n;
            return n;
        }
    };
    static_assert(RemoteInputRelaySink<RecordingRelaySink, int>,
        "the mock relay sink must satisfy the concept it is standing in for");

    // The same, typed for the BundleSim pack used by the receiveInputBundle cases.
    struct RecordingBundleRelaySink
    {
        struct Relayed
        {
            unsigned int  id;
            std::uint32_t captureTick;
            std::uint8_t  dA;
            std::int32_t  value;
        };
        std::vector<Relayed> calls;

        void relayRemoteInput(unsigned int id, std::uint32_t captureTick,
                              std::uint8_t dA, const BundleInput& in)
        {
            calls.push_back(Relayed{ id, captureTick, dA, in.value });
        }
    };
    static_assert(RemoteInputRelaySink<RecordingBundleRelaySink, BundleInput>,
        "the mock bundle relay sink must satisfy the concept it is standing in for");

    // THE "A SINK MISSING THE METHOD IS A COMPILE ERROR" AC, expressed as the
    // compile-time NEGATIVE (a genuinely non-compiling call cannot be a test case).
    // Both of these FAIL the concept, so passing either to receiveRemoteInput /
    // receiveInputBundle is a hard constraint failure at the call site.
    struct SinkWithoutRelay
    {
        void deliverRemoteInput(unsigned int, std::uint32_t, const int&) {}
    };
    static_assert(!RemoteInputRelaySink<SinkWithoutRelay, int>,
        "a sink with no relayRemoteInput must NOT satisfy RemoteInputRelaySink");

    // Missing the STAMP specifically — the pre-spectrum (InputRelayDesign.md §3a
    // draft) signature. It must not silently pass: the whole scheduled read depends
    // on dA travelling with the entry.
    struct SinkWithUnstampedRelay
    {
        void relayRemoteInput(unsigned int, std::uint32_t, const int&) {}
    };
    static_assert(!RemoteInputRelaySink<SinkWithUnstampedRelay, int>,
        "the unstamped (id, captureTick, input) signature must NOT satisfy the concept");

    // ...and the defaulted no-op sink DOES satisfy it, which is what makes every
    // pre-T3 call site compile untouched.
    static_assert(RemoteInputRelaySink<NullRemoteInputRelaySink, int>,
        "the default no-op sink must satisfy RemoteInputRelaySink");
}

TEST_CASE("ReceptionCoordinator relays each newer capture tick once, stamped at receipt", "[Network][ReceptionCoordinator][RelayTap]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingRelaySink relay;

    // No tier sampled for this wire yet, so the effective delay is the queue's
    // no-tier fallback. Read it from the queue rather than hardcoding it — the
    // stamp must be whatever the release schedule is actually using.
    const std::int32_t expectedDA = coord.delayQueue().effectiveDelay(wire);
    REQUIRE(expectedDA == cfg.forcedInputLatencyTicks);

    const ReceiveRemoteInputResult a =
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42, relay);

    REQUIRE(a.parked);
    REQUIRE(a.acceptedNew);

    REQUIRE(relay.calls.size() == 1);
    REQUIRE(relay.calls[0].id == 10);
    REQUIRE(relay.calls[0].captureTick == 100u);
    REQUIRE(relay.calls[0].dA == static_cast<std::uint8_t>(expectedDA));
    REQUIRE(relay.calls[0].value == 42);

    // A newer tick relays again — once, with its own stamp.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/101, 43, relay).acceptedNew);
    REQUIRE(relay.calls.size() == 2);
    REQUIRE(relay.calls[1].captureTick == 101u);
    REQUIRE(relay.calls[1].value == 43);

    // Two couch-coop owners on ONE wire relay independently, each under its own id.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/11, wire, /*slot=*/1, /*captureTick=*/100, 7, relay).acceptedNew);
    REQUIRE(relay.countFor(10) == 2);
    REQUIRE(relay.countFor(11) == 1);

    // Nothing here is out-of-order, so the skip counter never moved.
    REQUIRE(coord.relayOooSkipCount() == 0);
}

TEST_CASE("ReceptionCoordinator does not relay a duplicate capture tick", "[Network][ReceptionCoordinator][RelayTap]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingRelaySink relay;

    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42, relay);

    // The redundancy bundle re-sends recent ticks BY DESIGN (depth 3 in production),
    // so this is the common case, not an error. It must not re-write the peers' ring
    // and must not be counted as a relay hole — the value the authority parked is
    // first-wins anyway.
    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 42, relay);
    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 99, relay);

    REQUIRE(relay.calls.size() == 1);
    REQUIRE(relay.calls[0].value == 42);
    REQUIRE(coord.relayOooSkipCount() == 0);
}

TEST_CASE("ReceptionCoordinator applies but does not relay an out-of-order-older capture tick", "[Network][ReceptionCoordinator][RelayTap]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);
    std::vector<std::string> logged;
    coord.setLogger([&logged](const char* m) { logged.emplace_back(m); });

    const FStandaloneTestHandle wire = live(1);
    RecordingRelaySink relay;
    const std::int32_t delay = cfg.forcedInputLatencyTicks;

    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 1, relay);
    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/103, 2, relay);
    REQUIRE(relay.calls.size() == 2);

    // 101 now arrives LATE — after 103 already moved the watermark. It is genuinely
    // new (the delay queue has never seen it), so the SERVER takes it...
    const ReceiveRemoteInputResult late =
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/101, 3, relay);
    REQUIRE(late.parked);
    REQUIRE_FALSE(late.acceptedNew);

    // ...but it is NOT relayed: at depth 1 the payload is replace-latest, so writing
    // an older input would drag every peer's "latest" backwards. The peer sees a
    // hole and falls back to last-known; the state channel heals it.
    REQUIRE(relay.calls.size() == 2);
    REQUIRE(relay.calls[1].captureTick == 103u);      // still the newest relayed

    // Counted + traced, because the depth>1 future reopens this gate decision on
    // measured evidence (T9's probe reads exactly this counter).
    REQUIRE(coord.relayOooSkipCount() == 1);
    REQUIRE(hasLineContaining(logged, "[RelaySkip]"));
    REQUIRE(hasLineContaining(logged, "[Verbose]"));
    REQUIRE(hasLineContaining(logged, "captureTick=101"));

    // AND THE SERVER REALLY DOES APPLY IT — the half that makes the skip a relay
    // hole rather than an input drop. The drain releases in CAPTURE order (T26's
    // min-scan), so 101 comes out between 100 and 103 despite arriving last.
    RecordingDeliver deliver;
    coord.releaseDelayedInputs<MockSimA>(/*firstUpcomingSimTick=*/100 + delay,
                                         /*numSteps=*/4, std::ref(deliver));
    REQUIRE(deliver.calls.size() == 3);
    REQUIRE(deliver.calls[0].captureTick == 100u);
    REQUIRE(deliver.calls[1].captureTick == 101u);    // the never-relayed tick, applied
    REQUIRE(deliver.calls[2].captureTick == 103u);
}

TEST_CASE("ReceptionCoordinator stamps the relay with the effective delay in force at that receipt", "[Network][ReceptionCoordinator][RelayTap]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingRelaySink relay;
    RecordingTierSink tierSink;

    // (a) One low-RTT sample puts the wire in tier 0, so the queue now answers the
    // TIER delay rather than the no-tier fallback.
    coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/0, /*rttMs=*/10.0, tierSink);
    const std::int32_t d0 = coord.delayQueue().effectiveDelay(wire);
    REQUIRE(d0 == cfg.rttTierInputDelays[0]);

    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/100, 1, relay);
    REQUIRE(relay.calls.size() == 1);
    REQUIRE(relay.calls[0].dA == static_cast<std::uint8_t>(d0));

    // (b) A full dwell of tier-1-band samples (60 ms sits in ( >30+hysteresis,
    // <80 )) moves the wire 0 -> 1, which changes the effective delay.
    for (std::int32_t t = 1; t <= cfg.tierMinDwellTicks; ++t)
        coord.noteRttSample(wire, /*ownerId=*/10, /*serverTick=*/t, /*rttMs=*/60.0, tierSink);

    REQUIRE(coord.lookupTierIndex(wire) == 1);
    const std::int32_t d1 = coord.delayQueue().effectiveDelay(wire);
    REQUIRE(d1 == cfg.rttTierInputDelays[1]);
    // Guard against a config edit making this case vacuous: the two delays MUST
    // differ or nothing below discriminates.
    REQUIRE(d1 != d0);

    // (c) The NEXT accepted input carries the NEW stamp...
    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, /*captureTick=*/101, 2, relay);
    REQUIRE(relay.calls.size() == 2);
    REQUIRE(relay.calls[1].dA == static_cast<std::uint8_t>(d1));

    // ...while the ALREADY-RELAYED entry keeps the stamp it was actually relayed
    // with. The stamp is PER ENTRY (spectrum doc §5.3, intended-vs-actual): a
    // mid-session delay change must not retroactively rewrite a peer's schedule for
    // an input the authority already committed to.
    REQUIRE(relay.calls[0].dA == static_cast<std::uint8_t>(d0));
}

TEST_CASE("ReceptionCoordinator never relays on the malformed-slot fallback path", "[Network][ReceptionCoordinator][RelayTap]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingRelaySink relay;
    const std::uint8_t badSlot = ConnectionSlotKey<FStandaloneTestHandle>::kMaxPlayerSlot + 1;

    const ReceiveRemoteInputResult r =
        coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, badSlot, /*captureTick=*/100, 42, relay);

    // THE TRAP THIS CASE EXISTS FOR (review A6). `acceptedNew` is computed BEFORE
    // the parked/fallback split, so it is TRUE here even though the input was NOT
    // parked and the adapter will deliver it UNDELAYED. A relay gated on bare
    // `acceptedNew` would forward it stamped `captureTick + dA` while the authority
    // applies it at arrival — the peer would schedule it wrongly.
    REQUIRE_FALSE(r.parked);
    REQUIRE(r.acceptedNew);
    REQUIRE(relay.calls.empty());

    // Nor is the undelayed input mistaken for an out-of-order relay hole.
    REQUIRE(coord.relayOooSkipCount() == 0);
}

TEST_CASE("ReceptionCoordinator never relays an out-of-domain capture tick", "[Network][ReceptionCoordinator][RelayTap]")
{
    TimeConfig cfg;
    TestCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingRelaySink relay;
    const std::int32_t serverTick = 600;
    const DomainWindow w = domainWindowFor(cfg, serverTick);

    coord.reapConnections(serverTick);       // arm T12's gate

    // The composition T12's notes flagged forward: the gate runs first, so a
    // warm-up/free-running client's garbage tick can never reach the tap. This is
    // the whole reason T12 was mandatory BEFORE the relay made receipt peer-visible.
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, w.lower - 1, 1, relay)
                .rejectedOutOfDomain);
    REQUIRE(coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, w.upper + 1, 2, relay)
                .rejectedOutOfDomain);
    REQUIRE(relay.calls.empty());
    REQUIRE(coord.relayOooSkipCount() == 0);

    // An in-window tick from the same client on the same wire still relays.
    coord.receiveRemoteInput<MockSimA>(/*id=*/10, wire, /*slot=*/0, serverTick, 3, relay);
    REQUIRE(relay.calls.size() == 1);
    REQUIRE(relay.calls[0].captureTick == static_cast<std::uint32_t>(serverTick));
}

TEST_CASE("ReceptionCoordinator receiveInputBundle relays every newer slot exactly once", "[Network][ReceptionCoordinator][RelayTap]")
{
    TimeConfig cfg;
    BundleCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingDeliverySink deliver;
    RecordingBundleRelaySink relay;

    const std::int32_t expectedDA = coord.delayQueue().effectiveDelay(wire);

    // Both slots park; both relay. Delivery is untouched — the relay is an outbound
    // tap, not a second delivery path.
    coord.receiveInputBundle<BundleSim>(/*id=*/10, wire, /*slot=*/0,
                                        makeBundle({ { 100u, 42 }, { 101u, 43 } }),
                                        deliver, relay);
    REQUIRE(deliver.calls.empty());
    REQUIRE(relay.calls.size() == 2);
    REQUIRE(relay.calls[0].captureTick == 100u);
    REQUIRE(relay.calls[0].dA == static_cast<std::uint8_t>(expectedDA));
    REQUIRE(relay.calls[1].captureTick == 101u);
    REQUIRE(relay.calls[1].value == 43);

    // The NEXT bundle overlaps by design (redundancy depth): only the genuinely
    // newer slot relays, so the peer's ring is written once per capture tick.
    coord.receiveInputBundle<BundleSim>(/*id=*/10, wire, /*slot=*/0,
                                        makeBundle({ { 101u, 43 }, { 102u, 44 } }),
                                        deliver, relay);
    REQUIRE(relay.calls.size() == 3);
    REQUIRE(relay.calls[2].captureTick == 102u);
    REQUIRE(coord.relayOooSkipCount() == 0);
}

TEST_CASE("ReceptionCoordinator receiveInputBundle delivers a malformed slot without relaying it", "[Network][ReceptionCoordinator][RelayTap]")
{
    TimeConfig cfg;
    BundleCoordinator coord(cfg);

    const FStandaloneTestHandle wire = live(1);
    RecordingDeliverySink deliver;
    RecordingBundleRelaySink relay;
    const std::uint8_t badSlot = ConnectionSlotKey<FStandaloneTestHandle>::kMaxPlayerSlot + 1;

    coord.receiveInputBundle<BundleSim>(/*id=*/10, wire, badSlot,
                                        makeBundle({ { 100u, 42 } }), deliver, relay);

    // The A6 gate at the bundle level: the undelayed fallback still runs (no player
    // input is ever silently dropped), and NOTHING is relayed alongside it.
    REQUIRE(deliver.calls.size() == 1);
    REQUIRE(deliver.calls[0].captureTick == 100u);
    REQUIRE(relay.calls.empty());
}

#endif // WITH_LOW_LEVEL_TESTS
