// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/SimulationInputResolution.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationTimeContext.h"

#include <cstdint>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// The RELAYED-READ OBSERVATION RING -- what the scheduled read actually served
// for a remote character, per sim tick.
//
// WHAT THIS FILE IS REALLY GUARDING is that the ring records the capture tick
// the simulation RAN ON, not the one it asked for. The ladder probes
// `tick - dLatest` and then falls back on a miss, and a ring that recorded the
// probe tick on every rung would report a schedule that never happened -- which
// on a display is worse than reporting nothing, because it exonerates exactly
// the starvation it was built to expose.
//
// The second thing it guards is the PAIRING. The ring is created with the relay
// store and erased with it, on the game thread, so its physics-thread writer
// only ever looks an id up. A ring that inserted on first write would rehash
// the map underneath a game-thread reader; the lifecycle case below is what
// keeps the creation where it is.
//
// The ladder's ARMS are not re-tested here: SimulationInputResolutionTest.cpp
// and Network/RelayReadProbeTest.cpp own the outcome, the miss class and the
// served input. What is tested is the one field the split added and where it
// is recorded from.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    struct MockState
    {
        std::int32_t position = 0;
        bool isSimilarTo(const MockState& other) const { return position == other.position; }
    };

    struct MockInput
    {
        std::int32_t value = 0;
    };
} // namespace

template <>
struct SerializableFields<MockInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(MockInput, value));
    }
};

namespace
{
    class MockAllState
    {
    public:
        const MockState& getState() const { return m_state; }
        MockState&       editState()       { return m_state; }
    private:
        MockState m_state;
    };

    struct MockSimulatable
    {
        using StateType = MockState;
        using InputType = MockInput;

        MockAllState m_allState;
        MockAllState m_vizState;

        const MockAllState& getAllState() const { return m_allState; }
        MockAllState&       editAllState()       { return m_allState; }
        void updateVizState() { m_vizState = m_allState; }
        const MockAllState& getVizState() const { return m_vizState; }
    };

    using MockStorage        = SimulationObjectStorage<MockSimulatable>;
    using MockReconciliation = SimulationReconciliation<MockSimulatable>;
    using MockResolution     = SimulationInputResolution<MockSimulatable>;

    constexpr unsigned int kLocalId  = 1u;
    constexpr unsigned int kRemoteId = 2u;
    constexpr float        kDeltaSeconds = 1.0f / 60.0f;

    SimulationTimeStep normalStep(std::uint32_t tick)
    {
        return SimulationTimeStep(tick, /*isResimulating=*/false, StepKind::Normal, kDeltaSeconds);
    }

    struct ResolutionRig
    {
        MockStorage        storage;
        MockReconciliation reconciliation{ storage };
        MockResolution     resolution{ storage, reconciliation };
    };

    // Same shape as SimulationInputResolutionTest.cpp's: the two operations
    // injectCorrectionState actually calls.
    struct MockCorrectionBuffer
    {
        MockState     state{};
        std::uint32_t tick = 0;
        std::uint32_t appliedCaptureTick = kNoInputCaptureTick;

        std::uint32_t readInto(MockState& out) const { out = state; return tick; }
        std::uint32_t getAppliedCaptureTick() const { return appliedCaptureTick; }
    };

    // The observation filed for `simTick`, or nullptr when the ring holds none.
    // The ring is addressed by sim tick and each slot carries its own, so this
    // is a JOIN on the tick rather than an index lookup -- a wrapped slot must
    // never answer for a tick it no longer holds.
    const RelayedReadObservation* observationAt(const RelayedReadObservationRing& ring,
                                                std::uint32_t                     simTick)
    {
        for (std::size_t index = 0u; index < ring.size(); ++index)
        {
            const RelayedReadObservation* const observation = ring.at(index);
            if (observation != nullptr && observation->simTick == simTick)
                return observation;
        }
        return nullptr;
    }
} // namespace

// ---------------------------------------------------------------------------
// 1. THE LADDER NAMES THE CAPTURE IT SERVED, ON EVERY RUNG.
// ---------------------------------------------------------------------------

TEST_CASE("RelayedReadObservation.EveryLadderRungNamesTheCaptureTickItActuallyServed",
    "[InputResolution]")
{
    // Rung 0 -- nothing ever arrived. The fallback is the injected zero, which
    // stands behind no capture at all.
    // ⛔ ABSENT, NOT ZERO: capture tick 0 is ordinary, so a zero here would be a claim.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        const auto decision = decideScheduledRelayedRead(store, 10u);
        REQUIRE(decision.outcome == ScheduledRelayedReadOutcome::NoProbe);
        CHECK_FALSE(decision.appliedCaptureTickValid);

        const RelayedReadObservation observation = relayedReadObservationOf(decision, 10u);
        CHECK(observation.simTick == 10u);
        CHECK_FALSE(observation.hasAppliedCaptureTick);
    }

    // Hit -- the probe tick, which is what the integrator ran on.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(7u, /*dA=*/3u, MockInput{ 42 }));

        const auto decision = decideScheduledRelayedRead(store, 10u);
        REQUIRE(decision.outcome == ScheduledRelayedReadOutcome::Hit);
        CHECK(decision.probeTick == 7u);
        CHECK(decision.appliedCaptureTickValid);
        CHECK(decision.appliedCaptureTick == 7u);
        CHECK(relayedReadObservationOf(decision, 10u).appliedCaptureTick == 7u);
        CHECK(relayedReadObservationOf(decision, 10u).dLatest == 3u);
    }

    // Miss -- the probe tick is absent, so `fallback()` serves the newest
    // arrival. ⛔ THE FALLBACK'S OWN CAPTURE TICK, NEVER THE PROBE TICK: the
    //   probe tick names an input this client never had.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(9u, /*dA=*/3u, MockInput{ 42 }));

        const auto decision = decideScheduledRelayedRead(store, 20u);
        REQUIRE(decision.outcome == ScheduledRelayedReadOutcome::Miss);
        CHECK(decision.probeTick == 17u);
        CHECK(decision.appliedCaptureTickValid);
        CHECK(decision.appliedCaptureTick == 9u);
    }

    // VerifyFail -- resident, but stamped against a delay that is no longer
    // current. Same fallback, so the same served capture tick.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(7u, /*dA=*/1u, MockInput{ 11 }));
        REQUIRE(store.push(9u, /*dA=*/3u, MockInput{ 22 }));

        const auto decision = decideScheduledRelayedRead(store, 10u);
        REQUIRE(decision.outcome == ScheduledRelayedReadOutcome::VerifyFail);
        CHECK(decision.appliedCaptureTickValid);
        CHECK(decision.appliedCaptureTick == 9u);
    }

    // The tick < dA underflow guard -- a session younger than the delay. It
    // forms no probe tick at all, but the fallback it serves still has one.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(4u, /*dA=*/8u, MockInput{ 33 }));

        const auto decision = decideScheduledRelayedRead(store, 2u);
        REQUIRE(decision.outcome == ScheduledRelayedReadOutcome::Miss);
        REQUIRE(decision.isUnderflowMiss);
        CHECK_FALSE(decision.probeTickFormed);
        CHECK(decision.appliedCaptureTickValid);
        CHECK(decision.appliedCaptureTick == 4u);
    }
}

TEST_CASE("RelayedReadObservation.TheRefRungNamesTheRefWhenResidentAndTheFallbackWhenNot",
    "[InputResolution]")
{
    RemoteInputCache<MockInput> store{ MockInput{ -1 } };
    REQUIRE(store.push(6u, /*dA=*/0u, MockInput{ 55 }));

    // The replay found the authority's own ref: that is what this tick ran on.
    const RelayedReadObservation hit =
        relayedReadObservationOfRefRead(store, /*simTick=*/6u, /*refCaptureTick=*/6u, true);
    CHECK(hit.simTick == 6u);
    CHECK(hit.hasAppliedCaptureTick);
    CHECK(hit.appliedCaptureTick == 6u);
    CHECK(hit.outcome == ScheduledRelayedReadOutcome::Hit);

    // ⭐ THE SELF-HEAL, OBSERVED: the replay degraded to last-known, and the ring says so.
    const RelayedReadObservation missed =
        relayedReadObservationOfRefRead(store, /*simTick=*/9u, /*refCaptureTick=*/3u, false);
    CHECK(missed.hasAppliedCaptureTick);
    CHECK(missed.appliedCaptureTick == 6u);
    CHECK(missed.outcome == ScheduledRelayedReadOutcome::Miss);

    // An empty store degrades to the injected zero, which names no capture.
    RemoteInputCache<MockInput> cold{ MockInput{ -1 } };
    const RelayedReadObservation nothing =
        relayedReadObservationOfRefRead(cold, /*simTick=*/9u, /*refCaptureTick=*/3u, false);
    CHECK_FALSE(nothing.hasAppliedCaptureTick);
    CHECK(nothing.outcome == ScheduledRelayedReadOutcome::NoProbe);
}

// ---------------------------------------------------------------------------
// 2. THE RING ITSELF.
// ---------------------------------------------------------------------------

TEST_CASE("RelayedReadObservation.TheRingIsLastWriteWinsPerSimTickAndWrapsAtItsOwnWidth",
    "[InputResolution]")
{
    RelayedReadObservationRing ring;

    // Nothing written is nothing to read: an unfilled slot answers nullptr rather
    // than a default-constructed observation claiming sim tick 0.
    std::size_t emptySlots = 0u;
    for (std::size_t index = 0u; index < ring.size(); ++index)
    {
        if (ring.at(index) == nullptr)
            ++emptySlots;
    }
    CHECK(emptySlots == kRelayedReadObservationCapacityTicks);

    RelayedReadObservation first;
    first.simTick               = 100u;
    first.appliedCaptureTick    = 90u;
    first.hasAppliedCaptureTick = true;
    ring.note(first);

    REQUIRE(observationAt(ring, 100u) != nullptr);
    CHECK(observationAt(ring, 100u)->appliedCaptureTick == 90u);

    // ⭐ A RESIM RE-ANSWERS A TICK THE PREDICTION PASS ALREADY ANSWERED, and the
    // replay is what this client actually ran.
    RelayedReadObservation replayed = first;
    replayed.appliedCaptureTick = 97u;
    ring.note(replayed);
    CHECK(observationAt(ring, 100u)->appliedCaptureTick == 97u);

    // One window later the same slot is reused, and the slot's own tick is what
    // stops it answering for the tick it used to hold.
    RelayedReadObservation wrapped = first;
    wrapped.simTick            = 100u + static_cast<std::uint32_t>(ring.size());
    wrapped.appliedCaptureTick = 300u;
    ring.note(wrapped);

    CHECK(observationAt(ring, 100u) == nullptr);
    REQUIRE(observationAt(ring, wrapped.simTick) != nullptr);
    CHECK(observationAt(ring, wrapped.simTick)->appliedCaptureTick == 300u);
}

// ---------------------------------------------------------------------------
// 3. THE PAIRING -- created with the relay store, erased with it.
// ---------------------------------------------------------------------------

TEST_CASE("RelayedReadObservation.TheRingIsCreatedWithTheRelayStoreAndErasedWithIt",
    "[InputResolution]")
{
    ResolutionRig rig;

    // ⛔ ABSENT BEFORE REGISTRATION: a physics-thread write cannot rehash the map.
    CHECK(rig.resolution.getDiagnostics()
        .relayedReadObservations<MockSimulatable>(kRemoteId) == nullptr);

    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);
    CHECK(rig.resolution.getDiagnostics()
        .relayedReadObservations<MockSimulatable>(kRemoteId) != nullptr);

    // A LOCALLY controlled character gets no relay store, and so no ring: its
    // client-side delay is answered by its own delay line instead.
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalId,
        [](const SimulationTimeStep&, const LocalInputCache<MockInput>&) {
            return MockInput{ 1 };
        });
    CHECK(rig.resolution.getDiagnostics()
        .relayedReadObservations<MockSimulatable>(kLocalId) == nullptr);

    rig.resolution.unregisterCharacter<MockSimulatable>(kRemoteId);
    CHECK(rig.resolution.getDiagnostics()
        .relayedReadObservations<MockSimulatable>(kRemoteId) == nullptr);
    CHECK(rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId) == nullptr);
}

// ---------------------------------------------------------------------------
// 4. BOTH REMOTE READ SITES RECORD.
// ---------------------------------------------------------------------------

TEST_CASE("RelayedReadObservation.ThePredictionReadRecordsWhatItServedForTheProxy",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -1 });

    auto* store = rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId);
    REQUIRE(store != nullptr);
    REQUIRE(store->push(5u, /*dA=*/0u, MockInput{ 42 }));

    const auto step = normalStep(5u);
    rig.resolution.collectInputAll(step);
    rig.reconciliation.allocateFrontierSlotsAll(step);
    rig.reconciliation.postPredictionAll(step);

    const RelayedReadObservationRing* const ring =
        rig.resolution.getDiagnostics().relayedReadObservations<MockSimulatable>(kRemoteId);
    REQUIRE(ring != nullptr);

    const RelayedReadObservation* const observation = observationAt(*ring, 5u);
    REQUIRE(observation != nullptr);
    CHECK(observation->hasAppliedCaptureTick);
    CHECK(observation->appliedCaptureTick == 5u);
    CHECK(observation->outcome == ScheduledRelayedReadOutcome::Hit);

    // A tick the relay could not answer records the FALLBACK's capture tick --
    // which is how a stalled relay shows as a delay that grows tick by tick.
    const auto laterStep = normalStep(9u);
    rig.resolution.collectInputAll(laterStep);
    rig.reconciliation.allocateFrontierSlotsAll(laterStep);
    rig.reconciliation.postPredictionAll(laterStep);

    const RelayedReadObservation* const stalled = observationAt(*ring, 9u);
    REQUIRE(stalled != nullptr);
    CHECK(stalled->outcome == ScheduledRelayedReadOutcome::Miss);
    CHECK(stalled->hasAppliedCaptureTick);
    CHECK(stalled->appliedCaptureTick == 5u);
}

TEST_CASE("RelayedReadObservation.TheResimRefReadRecordsTheRefTheReplayRanOn",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);

    auto* store = rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId);
    REQUIRE(store != nullptr);
    REQUIRE(store->push(6u, /*dA=*/0u, MockInput{ 55 }));

    rig.reconciliation.pushPredictionTick<MockSimulatable>(kRemoteId, 6u);
    const auto step = normalStep(6u);
    rig.reconciliation.postPredictionAll(step);

    MockCorrectionBuffer buffer;
    buffer.state              = MockState{ 0 };
    buffer.tick               = 6u;
    buffer.appliedCaptureTick = 6u;
    rig.reconciliation.injectCorrectionState<MockSimulatable>(kRemoteId, buffer);

    rig.resolution.collectResimInputAll(6u);

    const RelayedReadObservationRing* const ring =
        rig.resolution.getDiagnostics().relayedReadObservations<MockSimulatable>(kRemoteId);
    REQUIRE(ring != nullptr);

    const RelayedReadObservation* const observation = observationAt(*ring, 6u);
    REQUIRE(observation != nullptr);
    CHECK(observation->hasAppliedCaptureTick);
    CHECK(observation->appliedCaptureTick == 6u);
    CHECK(observation->outcome == ScheduledRelayedReadOutcome::Hit);
}

TEST_CASE("RelayedReadObservation.TheResimNoRefRungRecordsTheSameScheduledReadThePredictionRan",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);

    auto* store = rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId);
    REQUIRE(store != nullptr);
    REQUIRE(store->push(4u, /*dA=*/0u, MockInput{ 77 }));

    rig.reconciliation.pushPredictionTick<MockSimulatable>(kRemoteId, 4u);
    const auto step = normalStep(4u);
    rig.reconciliation.postPredictionAll(step);

    // No correction has landed, so the ref rung is NoRef and the replay runs the
    // same ladder the prediction pass ran.
    REQUIRE(rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kRemoteId, 4u).kind
        == AppliedCaptureRefKind::NoRef);

    rig.resolution.collectResimInputAll(4u);

    const RelayedReadObservationRing* const ring =
        rig.resolution.getDiagnostics().relayedReadObservations<MockSimulatable>(kRemoteId);
    REQUIRE(ring != nullptr);

    const RelayedReadObservation* const observation = observationAt(*ring, 4u);
    REQUIRE(observation != nullptr);
    CHECK(observation->hasAppliedCaptureTick);
    CHECK(observation->appliedCaptureTick == 4u);
}

TEST_CASE("RelayedReadObservation.AReplayOverwritesWhatThePredictionPassRecordedForThatTick",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -1 });

    auto* store = rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId);
    REQUIRE(store != nullptr);

    // Only capture 2 has arrived, so the prediction of tick 9 falls back onto it.
    REQUIRE(store->push(2u, /*dA=*/0u, MockInput{ 11 }));

    const auto step = normalStep(9u);
    rig.resolution.collectInputAll(step);
    rig.reconciliation.allocateFrontierSlotsAll(step);
    rig.reconciliation.postPredictionAll(step);

    const RelayedReadObservationRing* const ring =
        rig.resolution.getDiagnostics().relayedReadObservations<MockSimulatable>(kRemoteId);
    REQUIRE(ring != nullptr);
    REQUIRE(observationAt(*ring, 9u) != nullptr);
    CHECK(observationAt(*ring, 9u)->appliedCaptureTick == 2u);
    CHECK(observationAt(*ring, 9u)->outcome == ScheduledRelayedReadOutcome::Miss);

    // The late capture then arrives and the correction names it. The replay runs
    // tick 9 on capture 9, and the ring's answer for that tick becomes the
    // replay's -- which is exactly why a cell that still disagrees AFTER a replay
    // is a relay-store miss rather than a scheduling difference.
    REQUIRE(store->push(9u, /*dA=*/0u, MockInput{ 99 }));

    MockCorrectionBuffer buffer;
    buffer.state              = MockState{ 0 };
    buffer.tick               = 9u;
    buffer.appliedCaptureTick = 9u;
    rig.reconciliation.injectCorrectionState<MockSimulatable>(kRemoteId, buffer);

    rig.resolution.collectResimInputAll(9u);

    REQUIRE(observationAt(*ring, 9u) != nullptr);
    CHECK(observationAt(*ring, 9u)->appliedCaptureTick == 9u);
    CHECK(observationAt(*ring, 9u)->outcome == ScheduledRelayedReadOutcome::Hit);
}

#endif // WITH_LOW_LEVEL_TESTS
