// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationInputResolution.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SimulationTimeContext.h"

#include <cstdint>
#include <cstring>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// WHEN A RELAYED CAPTURE FINALLY GOT HERE -- and the two fields a scheduled
// read now keeps about what it asked for.
//
// A read that missed says nothing at all about whether the capture it wanted
// ever turned up. The store cannot answer that later either: it is 64 capture
// ticks wide and reclaims slots by overwrite, so by the time a display asks, a
// capture that DID arrive may already be gone and one that never arrived looks
// identical to it.
//
// WHAT THIS FILE IS REALLY GUARDING is that there is exactly ONE place that
// learns "it arrived" -- the by-id ingest door -- and that the answer it files
// is joined to a read through the tick the read ASKED FOR, never the one it
// served. Those are different ticks on every rung but Hit, and a join on the
// served tick would say a fallback arrived on time, every time.
//
// The ladder's own arms are not re-tested here: SimulationInputResolutionTest.cpp
// and Network/RelayReadProbeTest.cpp own the outcome and the served input, and
// RelayedReadObservationTest.cpp owns the observation's applied-capture field.
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

    constexpr unsigned int kRemoteId = 2u;
    constexpr unsigned int kLocalId  = 1u;
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

    // The std::vector-backed satisfier of RelayedInputRingCodec.h's BUFFER CONCEPT,
    // the same shape SimulationInputResolutionTest.cpp uses.
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

        void bundleTruncateTo(std::int32_t byteCount)
        {
            if (byteCount < 0) byteCount = 0;
            if (static_cast<std::size_t>(byteCount) < bytes.size())
                bytes.resize(static_cast<std::size_t>(byteCount));
        }
    };

    // One capture on the wire, as the relay would carry it.
    RingTestBuffer ringCarrying(std::uint32_t captureTick, std::uint8_t dA, std::int32_t value)
    {
        RingTestBuffer ring;
        REQUIRE(relayedInputRing::writeLatest<MockInput>(ring, captureTick, dA,
            MockInput{ value }, /*depth=*/1));
        return ring;
    }

    // The observation filed for `simTick`, or nullptr. The ring is addressed by sim tick
    // and each slot carries its own, so this is a JOIN rather than an index lookup.
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
// 1. THE OBSERVATION NOW CARRIES WHAT THE READ ASKED FOR, AND WHY IT MISSED.
// ---------------------------------------------------------------------------

TEST_CASE("RelayedInputArrival.TheObservationNamesTheProbeTickAndWhyTheMissMissed",
    "[InputResolution]")
{
    // Hit -- the probe tick was formed and found. A hit is not a miss, so it names
    // no cause.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(7u, /*dA=*/3u, MockInput{ 42 }));

        RelayedReadObservation observation;
        resolveScheduledRelayedInput(store, 10u, nullptr, &observation);

        CHECK(observation.outcome == ScheduledRelayedReadOutcome::Hit);
        CHECK(observation.probeTickFormed);
        CHECK(observation.probeTick == 7u);
        CHECK(observation.missClass == ScheduledRelayedReadMissClass::NotAMiss);
    }

    // Miss, InSpan -- the probe tick lies between two residents and is absent. The
    // coverage hole the whole classification exists to separate.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(5u, /*dA=*/3u, MockInput{ 1 }));
        REQUIRE(store.push(9u, /*dA=*/3u, MockInput{ 2 }));

        RelayedReadObservation observation;
        resolveScheduledRelayedInput(store, 10u, nullptr, &observation);

        CHECK(observation.outcome == ScheduledRelayedReadOutcome::Miss);
        CHECK(observation.probeTick == 7u);
        CHECK(observation.missClass == ScheduledRelayedReadMissClass::InSpan);
    }

    // Miss, AboveNewest -- nothing that new has been relayed yet.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(4u, /*dA=*/3u, MockInput{ 1 }));

        RelayedReadObservation observation;
        resolveScheduledRelayedInput(store, 10u, nullptr, &observation);

        CHECK(observation.probeTick == 7u);
        CHECK(observation.missClass == ScheduledRelayedReadMissClass::AboveNewest);
    }

    // Miss, BelowOldest -- the read reached below what the store still holds.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(100u, /*dA=*/3u, MockInput{ 1 }));

        RelayedReadObservation observation;
        resolveScheduledRelayedInput(store, 20u, nullptr, &observation);

        CHECK(observation.probeTick == 17u);
        CHECK(observation.missClass == ScheduledRelayedReadMissClass::BelowOldest);
    }

    // VerifyFail -- resident, but stamped against a delay that is no longer current. It
    // DID form a probe tick, so an arrival can still be joined to it; the cause is the
    // regime shift rather than any miss class.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(7u, /*dA=*/1u, MockInput{ 11 }));
        REQUIRE(store.push(9u, /*dA=*/3u, MockInput{ 22 }));

        RelayedReadObservation observation;
        resolveScheduledRelayedInput(store, 10u, nullptr, &observation);

        CHECK(observation.outcome == ScheduledRelayedReadOutcome::VerifyFail);
        CHECK(observation.probeTickFormed);
        CHECK(observation.probeTick == 7u);
        CHECK(observation.missClass == ScheduledRelayedReadMissClass::NotAMiss);
    }

    // The tick < dA underflow guard -- a session younger than the delay. It formed no
    // probe tick, so it asked for nothing and NOTHING CAN HAVE BEEN LATE FOR IT.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(4u, /*dA=*/8u, MockInput{ 33 }));

        RelayedReadObservation observation;
        resolveScheduledRelayedInput(store, 2u, nullptr, &observation);

        CHECK(observation.outcome == ScheduledRelayedReadOutcome::Miss);
        CHECK_FALSE(observation.probeTickFormed);
        CHECK(observation.missClass == ScheduledRelayedReadMissClass::NoProbeTick);
    }

    // Rung 0 -- nothing ever arrived, so the ladder never probed at all.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };

        RelayedReadObservation observation;
        resolveScheduledRelayedInput(store, 10u, nullptr, &observation);

        CHECK(observation.outcome == ScheduledRelayedReadOutcome::NoProbe);
        CHECK_FALSE(observation.probeTickFormed);
        CHECK(observation.missClass == ScheduledRelayedReadMissClass::NotAMiss);
    }

    // The replay's ref rung reads by the authority's ref, so it asks the relay for no
    // capture tick and names no cause -- whether or not the ref was resident.
    {
        RemoteInputCache<MockInput> store{ MockInput{ -1 } };
        REQUIRE(store.push(9u, /*dA=*/3u, MockInput{ 5 }));

        const RelayedReadObservation resident =
            relayedReadObservationOfRefRead(store, 12u, 9u, /*refWasResident=*/true);
        CHECK_FALSE(resident.probeTickFormed);
        CHECK(resident.missClass == ScheduledRelayedReadMissClass::NotAMiss);

        const RelayedReadObservation absent =
            relayedReadObservationOfRefRead(store, 12u, 4u, /*refWasResident=*/false);
        CHECK(absent.outcome == ScheduledRelayedReadOutcome::Miss);
        CHECK_FALSE(absent.probeTickFormed);
        CHECK(absent.missClass == ScheduledRelayedReadMissClass::NotAMiss);
    }
}

TEST_CASE("RelayedInputArrival.TheReportAndTheObservationNameOneClassificationNotTwo",
    "[InputResolution]")
{
    // Two classifications of one read could disagree, and the display would then
    // contradict the counters about the same tick.
    // ⭐ ONE SPAN SCAN FEEDS BOTH.
    RemoteInputCache<MockInput> store{ MockInput{ -1 } };
    REQUIRE(store.push(5u, /*dA=*/3u, MockInput{ 1 }));
    REQUIRE(store.push(9u, /*dA=*/3u, MockInput{ 2 }));

    ScheduledRelayedReadReport report;
    RelayedReadObservation     observation;
    resolveScheduledRelayedInput(store, 10u, &report, &observation);

    CHECK(report.missClass == observation.missClass);
    CHECK(report.probeTick == observation.probeTick);

    // The report's span fields are still filled on exactly the rung they always were.
    CHECK(report.spanValid);
    CHECK(report.oldestResident == 5u);
    CHECK(report.newestResident == 9u);
    // `residentCount` is the number of OCCUPIED SLOTS, not the width of the span:
    // two captures with a hole between them are two residents spanning five ticks.
    CHECK(report.residentCount == 2u);

    // Asking for the OBSERVATION ALONE classifies the same miss: the answer does not
    // depend on a report having been asked for beside it.
    RelayedReadObservation observationOnly;
    resolveScheduledRelayedInput(store, 10u, nullptr, &observationOnly);
    CHECK(observationOnly.missClass == report.missClass);

    // And asking for the REPORT ALONE is unchanged in every field the counters read.
    ScheduledRelayedReadReport reportOnly;
    resolveScheduledRelayedInput(store, 10u, &reportOnly, nullptr);
    CHECK(reportOnly.missClass == report.missClass);
    CHECK(reportOnly.spanValid == report.spanValid);
    CHECK(reportOnly.residentCount == report.residentCount);
    CHECK(reportOnly.deltaToNewest == report.deltaToNewest);
}

// ---------------------------------------------------------------------------
// 2. THE ARRIVAL RING ITSELF.
// ---------------------------------------------------------------------------

TEST_CASE("RelayedInputArrival.TheRingIsFirstWriteWinsAndAnswersOnItsOwnCaptureTick",
    "[InputResolution]")
{
    RelayedInputArrivalRing ring;

    CHECK(ring.findArrival(5u) == nullptr);

    ring.note(5u, /*atSimTick=*/10u);
    REQUIRE(ring.findArrival(5u) != nullptr);
    CHECK(ring.findArrival(5u)->arrivedAtSimTick == 10u);

    // The relay re-sends entries it has already sent, and every one of those would
    // otherwise reset the arrival tick.
    // ⭐ A RE-DELIVERY IS NOT WHEN THE CAPTURE GOT HERE.
    ring.note(5u, /*atSimTick=*/40u);
    CHECK(ring.findArrival(5u)->arrivedAtSimTick == 10u);

    // ⛔ A WRAPPED SLOT MUST NOT ANSWER FOR THE TICK IT NO LONGER HOLDS: one whole
    //   window later shares the slot index, and the join is on the slot's own tick.
    const std::uint32_t wrapped =
        5u + static_cast<std::uint32_t>(kRelayedReadObservationCapacityTicks);
    ring.note(wrapped, /*atSimTick=*/300u);
    CHECK(ring.findArrival(wrapped) != nullptr);
    CHECK(ring.findArrival(wrapped)->arrivedAtSimTick == 300u);
    CHECK(ring.findArrival(5u) == nullptr);

    // One display window wide, like the observation ring it is joined to.
    CHECK(ring.size() == kRelayedReadObservationCapacityTicks);
}

// ---------------------------------------------------------------------------
// 3. THE ONE ARRIVAL DOOR.
// ---------------------------------------------------------------------------

TEST_CASE("RelayedInputArrival.TheIngestDoorStampsEveryCaptureTheWireCarried",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -1 });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);

    // A first read has to have run, or there is no frontier to stamp an arrival against.
    // The store is empty here, so this is rung 0 at tick 30.
    rig.resolution.collectInputAll(normalStep(30u));

    RingTestBuffer first = ringCarrying(/*captureTick=*/25u, /*dA=*/3u, /*value=*/7);
    REQUIRE(rig.resolution.ingestRelayRing<MockSimulatable>(kRemoteId, first).outcome
        == RelayedInputIngestOutcome::Consumed);

    const RelayedInputArrivalRing* const arrivals =
        rig.resolution.getDiagnostics().relayedInputArrivals<MockSimulatable>(kRemoteId);
    REQUIRE(arrivals != nullptr);

    REQUIRE(arrivals->findArrival(25u) != nullptr);
    CHECK(arrivals->findArrival(25u)->arrivedAtSimTick == 30u);

    // The frontier moves with the reads, so a capture arriving later is stamped later.
    rig.resolution.collectInputAll(normalStep(33u));
    RingTestBuffer second = ringCarrying(/*captureTick=*/26u, /*dA=*/3u, /*value=*/8);
    REQUIRE(rig.resolution.ingestRelayRing<MockSimulatable>(kRemoteId, second).outcome
        == RelayedInputIngestOutcome::Consumed);

    REQUIRE(arrivals->findArrival(26u) != nullptr);
    CHECK(arrivals->findArrival(26u)->arrivedAtSimTick == 33u);

    // The door's own ingest pushes every entry the wire carried and the store refuses
    // only the sentinel capture tick, so a capture 40 ticks behind the frontier is
    // recorded exactly like a fresh one.
    // ⭐ A LATE ARRIVAL IS NEVER TURNED AWAY.
    rig.resolution.collectInputAll(normalStep(70u));
    RingTestBuffer late = ringCarrying(/*captureTick=*/27u, /*dA=*/3u, /*value=*/9);
    REQUIRE(rig.resolution.ingestRelayRing<MockSimulatable>(kRemoteId, late).outcome
        == RelayedInputIngestOutcome::Consumed);

    REQUIRE(arrivals->findArrival(27u) != nullptr);
    CHECK(arrivals->findArrival(27u)->arrivedAtSimTick == 70u);

    // ⛔ THE STAMP IS THE FIRST ARRIVAL'S, not the newest read's: re-ingesting the same
    //   ring at a later frontier must not make an old arrival look recent.
    rig.resolution.collectInputAll(normalStep(90u));
    REQUIRE(rig.resolution.ingestRelayRing<MockSimulatable>(kRemoteId, first).outcome
        == RelayedInputIngestOutcome::Consumed);
    CHECK(arrivals->findArrival(25u)->arrivedAtSimTick == 30u);
}

TEST_CASE("RelayedInputArrival.AnArrivalBeforeThisClientHasReadAnythingIsNotStamped",
    "[InputResolution]")
{
    // Lateness is measured against the ticks this client has served relayed reads for,
    // and before the first one there is no such tick. Nothing is lost by it: no
    // observation exists yet for such an arrival to have been late for, and the read that
    // later asks for that capture finds it resident and HITS.
    // ⚠ A DISCLOSED BOUNDARY, PINNED RATHER THAN LEFT TO CHANCE.
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -1 });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);

    RingTestBuffer ring = ringCarrying(/*captureTick=*/25u, /*dA=*/3u, /*value=*/7);
    REQUIRE(rig.resolution.ingestRelayRing<MockSimulatable>(kRemoteId, ring).outcome
        == RelayedInputIngestOutcome::Consumed);

    const RelayedInputArrivalRing* const arrivals =
        rig.resolution.getDiagnostics().relayedInputArrivals<MockSimulatable>(kRemoteId);
    REQUIRE(arrivals != nullptr);
    CHECK(arrivals->findArrival(25u) == nullptr);

    // The capture is in the store all the same, so the read at tick 28 hits it.
    rig.resolution.collectInputAll(normalStep(28u));

    const RelayedReadObservationRing* const observations =
        rig.resolution.getDiagnostics().relayedReadObservations<MockSimulatable>(kRemoteId);
    REQUIRE(observations != nullptr);
    REQUIRE(observationAt(*observations, 28u) != nullptr);
    CHECK(observationAt(*observations, 28u)->outcome == ScheduledRelayedReadOutcome::Hit);
}

TEST_CASE("RelayedInputArrival.TheArrivalRingIsCreatedAndErasedWithTheCharacter",
    "[InputResolution]")
{
    ResolutionRig rig;
    const auto diagnostics = rig.resolution.getDiagnostics();

    // Before registration there is nothing to answer with.
    CHECK(diagnostics.relayedInputArrivals<MockSimulatable>(kRemoteId) == nullptr);

    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);
    CHECK(diagnostics.relayedInputArrivals<MockSimulatable>(kRemoteId) != nullptr);

    // ⛔ THE SAME PAIRING THE OBSERVATION RING KEEPS -- both created here, both erased
    //   below, so neither diagnostic can outlive the id it describes.
    CHECK(diagnostics.relayedReadObservations<MockSimulatable>(kRemoteId) != nullptr);

    // A LOCALLY CONTROLLED character gets neither: no relay serves one.
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalId,
        [](const SimulationTimeStep&, const LocalInputCache<MockInput>&) { return MockInput{ 1 }; });
    CHECK(diagnostics.relayedInputArrivals<MockSimulatable>(kLocalId) == nullptr);
    CHECK(diagnostics.relayedReadObservations<MockSimulatable>(kLocalId) == nullptr);

    rig.resolution.unregisterCharacter<MockSimulatable>(kRemoteId);
    CHECK(diagnostics.relayedInputArrivals<MockSimulatable>(kRemoteId) == nullptr);
    CHECK(diagnostics.relayedReadObservations<MockSimulatable>(kRemoteId) == nullptr);
}

TEST_CASE("RelayedInputArrival.TheRelayReadCountersAreUnchangedByTheArrivalRecord",
    "[InputResolution]")
{
    // The counters are the probe's own answer about the same reads, and a diagnostic that
    // moved them would be measuring itself.
    // ⭐ IT OBSERVES, IT DOES NOT CONSUME.
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -1 });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);

    rig.resolution.collectInputAll(normalStep(30u));

    const RelayReadCounters before =
        rig.resolution.getDiagnostics().relayReadProbe().predictionCounters();

    RingTestBuffer ring = ringCarrying(/*captureTick=*/25u, /*dA=*/3u, /*value=*/7);
    REQUIRE(rig.resolution.ingestRelayRing<MockSimulatable>(kRemoteId, ring).outcome
        == RelayedInputIngestOutcome::Consumed);

    const RelayReadCounters after =
        rig.resolution.getDiagnostics().relayReadProbe().predictionCounters();

    CHECK(after.total() == before.total());
    CHECK(after.hit == before.hit);
    CHECK(after.miss == before.miss);
    CHECK(after.verifyFail == before.verifyFail);
    CHECK(after.noProbe == before.noProbe);
    CHECK(after.missInSpan == before.missInSpan);
    CHECK(after.missAboveNewest == before.missAboveNewest);
    CHECK(after.missBelowOldest == before.missBelowOldest);
    CHECK(after.missNoProbeTick == before.missNoProbeTick);

    // And the arrival WAS recorded, so the equality above is not vacuous.
    const RelayedInputArrivalRing* const arrivals =
        rig.resolution.getDiagnostics().relayedInputArrivals<MockSimulatable>(kRemoteId);
    REQUIRE(arrivals != nullptr);
    CHECK(arrivals->findArrival(25u) != nullptr);
}

#endif // WITH_LOW_LEVEL_TESTS
