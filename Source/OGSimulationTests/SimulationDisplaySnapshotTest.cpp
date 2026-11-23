// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/Network/ReplicatedTierConsumer.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationDisplaySnapshot.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationInputResolution.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SimulationTimeContext.h"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

namespace
{
    struct DisplayMockState
    {
        std::int32_t position = 0;
        bool isSimilarTo(const DisplayMockState& other) const { return position == other.position; }
    };

    struct DisplayMockInput
    {
        std::int32_t value = 0;
    };
} // namespace

template <>
struct SerializableFields<DisplayMockInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(DisplayMockInput, value));
    }
};

namespace
{
    class DisplayMockAllState
    {
    public:
        const DisplayMockState& getState() const { return m_state; }
        DisplayMockState&       editState()       { return m_state; }
    private:
        DisplayMockState m_state;
    };

    struct DisplayMockSimulatable
    {
        using StateType = DisplayMockState;
        using InputType = DisplayMockInput;

        DisplayMockAllState m_allState;
        DisplayMockAllState m_vizState;

        const DisplayMockAllState& getAllState() const { return m_allState; }
        DisplayMockAllState&       editAllState()       { return m_allState; }
        void updateVizState() { m_vizState = m_allState; }
        const DisplayMockAllState& getVizState() const { return m_vizState; }
        DisplayMockAllState&       editVizState()       { return m_vizState; }
    };

    using DisplayStorage        = SimulationObjectStorage<DisplayMockSimulatable>;
    using DisplayReconciliation = SimulationReconciliation<DisplayMockSimulatable>;
    using DisplayResolution     = SimulationInputResolution<DisplayMockSimulatable>;

    constexpr std::size_t kCaptureWindow = 16u;
    constexpr std::size_t kAppliedWindow = 8u;
    using DisplaySnapshot  = SimulationDisplaySnapshotT<DisplayMockSimulatable, 4u, kCaptureWindow, kAppliedWindow>;
    using DisplayPublisher = SimulationDisplayPublisher<DisplaySnapshot>;

    constexpr unsigned int  kLocalId      = 1u;
    constexpr unsigned int  kRemoteId     = 2u;
    constexpr unsigned int  kUnknownId    = 9u;
    constexpr std::uint32_t kNewestTick   = 20u;
    constexpr std::uint32_t kFirstTick    = 1u;
    constexpr float         kDeltaSeconds = 1.0f / 60.0f;

    SimulationTimeStep displayStep(std::uint32_t tick)
    {
        return SimulationTimeStep(tick, false, StepKind::Normal, kDeltaSeconds);
    }

    struct DisplayRingBuffer
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

    DisplayRingBuffer displayRingCarrying(std::uint32_t captureTick, std::int32_t value)
    {
        DisplayRingBuffer ring;
        REQUIRE(relayedInputRing::writeLatest<DisplayMockInput>(ring, captureTick, 2u, DisplayMockInput{ value }, 1));
        return ring;
    }

    struct DisplayRig
    {
        DisplayStorage        storage;
        DisplayReconciliation reconciliation{ storage };
        DisplayResolution     resolution{ storage, reconciliation };

        DisplayRig()
        {
            storage.add<DisplayMockSimulatable>(kLocalId, DisplayMockSimulatable{});
            storage.add<DisplayMockSimulatable>(kRemoteId, DisplayMockSimulatable{});
            reconciliation.createCacheFor<DisplayMockSimulatable>(kLocalId);
            reconciliation.createCacheFor<DisplayMockSimulatable>(kRemoteId);
            resolution.setNeutralInput<DisplayMockSimulatable>(DisplayMockInput{ -1 });
            resolution.registerLocalCharacter<DisplayMockSimulatable>(kLocalId,
                [](const SimulationTimeStep& step, const LocalInputCache<DisplayMockInput>&)
                {
                    return DisplayMockInput{ static_cast<std::int32_t>(step.getTick()) * 10 };
                });
            resolution.registerRemoteCharacter<DisplayMockSimulatable>(kRemoteId);

            for (std::uint32_t tick = kFirstTick; tick <= kNewestTick; ++tick)
            {
                if (tick % 3u == 0u)
                {
                    DisplayRingBuffer ring = displayRingCarrying(tick - 1u, static_cast<std::int32_t>(tick) * 100);
                    REQUIRE(resolution.ingestRelayRing<DisplayMockSimulatable>(kRemoteId, ring).outcome
                        == RelayedInputIngestOutcome::Consumed);
                }
                const SimulationTimeStep step = displayStep(tick);
                resolution.collectInputAll(step);
                reconciliation.allocateFrontierSlotsAll(step);
                reconciliation.postPredictionAll(step);
            }

            storage.get<DisplayMockSimulatable>(kLocalId).editAllState().editState().position  = 11;
            storage.get<DisplayMockSimulatable>(kRemoteId).editAllState().editState().position = 22;
            storage.get<DisplayMockSimulatable>(kLocalId).editVizState().editState().position  = -11;
            storage.get<DisplayMockSimulatable>(kRemoteId).editVizState().editState().position = -22;
        }

        std::unique_ptr<DisplaySnapshot> fill() const
        {
            auto snapshot = std::make_unique<DisplaySnapshot>();
            DisplayClockReading clock;
            clock.predictionTick = kNewestTick;
            fillSimulationDisplaySnapshot(*snapshot, storage, resolution, reconciliation, clock, DisplayTierReading{});
            return snapshot;
        }
    };

    bool sameObservation(const RelayedReadObservation& first, const RelayedReadObservation& second)
    {
        return first.simTick == second.simTick
            && first.appliedCaptureTick == second.appliedCaptureTick
            && first.hasAppliedCaptureTick == second.hasAppliedCaptureTick
            && first.outcome == second.outcome
            && first.dLatest == second.dLatest
            && first.probeTick == second.probeTick
            && first.probeTickFormed == second.probeTickFormed
            && first.missClass == second.missClass;
    }
} // namespace

TEST_CASE("SimulationDisplaySnapshot.EntriesAreOnePerStoredSimulatableAndFoundBySimId", "[Mailbox]")
{
    const DisplayRig rig;
    const auto snapshot = rig.fill();

    CHECK(snapshot->entryCount == 2u);
    REQUIRE(snapshot->find(kLocalId) != nullptr);
    REQUIRE(snapshot->find(kRemoteId) != nullptr);
    CHECK(snapshot->find(kLocalId)->simId == kLocalId);
    CHECK(snapshot->find(kRemoteId)->simId == kRemoteId);
    CHECK(snapshot->find(kUnknownId) == nullptr);
}

TEST_CASE("SimulationDisplaySnapshot.VizStateIsTheSimulatablesAllStateNotItsVizCopy", "[Mailbox]")
{
    const DisplayRig rig;
    const auto snapshot = rig.fill();

    CHECK(snapshot->find(kLocalId)->vizState.getState().position == 11);
    CHECK(snapshot->find(kRemoteId)->vizState.getState().position == 22);
    CHECK(rig.storage.get<DisplayMockSimulatable>(kLocalId).getVizState().getState().position == -11);
}

TEST_CASE("SimulationDisplaySnapshot.LastRelayedInputIsTheResolutionsAnswer", "[Mailbox]")
{
    const DisplayRig rig;
    const auto snapshot = rig.fill();

    const std::optional<DisplayMockInput> remote =
        rig.resolution.getLastRelayedInput<DisplayMockSimulatable>(kRemoteId);
    REQUIRE(remote.has_value());
    REQUIRE(snapshot->find(kRemoteId)->lastRelayedInput.has_value());
    CHECK(snapshot->find(kRemoteId)->lastRelayedInput->value == remote->value);
    CHECK(remote->value == 1800);

    CHECK_FALSE(rig.resolution.getLastRelayedInput<DisplayMockSimulatable>(kLocalId).has_value());
    CHECK_FALSE(snapshot->find(kLocalId)->lastRelayedInput.has_value());
}

TEST_CASE("SimulationDisplaySnapshot.LocalCapturesMatchTheCacheOverTheWindow", "[Mailbox]")
{
    const DisplayRig rig;
    const auto snapshot = rig.fill();

    const LocalInputCache<DisplayMockInput>* const cache =
        rig.resolution.getDiagnostics().localInputCache<DisplayMockSimulatable>(kLocalId);
    REQUIRE(cache != nullptr);

    const DisplaySnapshot::Entry& local = *snapshot->find(kLocalId);
    CHECK(local.hasLocalInputCache);

    std::size_t present = 0u;
    for (std::uint32_t back = 0u; back < kCaptureWindow; ++back)
    {
        const std::uint32_t tick = kNewestTick - back;
        const DisplayMockInput* const copied = local.localCaptures.at(tick);
        const bool resident = cache->has(static_cast<std::int32_t>(tick));
        CHECK((copied != nullptr) == resident);
        if (copied != nullptr && resident)
        {
            CHECK(copied->value == cache->at(static_cast<std::int32_t>(tick)).value);
            ++present;
        }
    }
    CHECK(present == kCaptureWindow);
    CHECK(local.localCaptures.at(kNewestTick - static_cast<std::uint32_t>(kCaptureWindow)) == nullptr);
    CHECK(local.localCaptures.at(kNewestTick + 1u) == nullptr);

    const DisplaySnapshot::Entry& remote = *snapshot->find(kRemoteId);
    CHECK_FALSE(remote.hasLocalInputCache);
    CHECK(rig.resolution.getDiagnostics().localInputCache<DisplayMockSimulatable>(kRemoteId) == nullptr);
    CHECK(remote.localCaptures.at(kNewestTick) == nullptr);
}

TEST_CASE("SimulationDisplaySnapshot.RelayedReadObservationsAreTheRingsCopy", "[Mailbox]")
{
    const DisplayRig rig;
    const auto snapshot = rig.fill();

    const RelayedReadObservationRing* const ring =
        rig.resolution.getDiagnostics().relayedReadObservations<DisplayMockSimulatable>(kRemoteId);
    REQUIRE(ring != nullptr);

    const DisplaySnapshot::Entry& remote = *snapshot->find(kRemoteId);
    CHECK(remote.hasRelayedReadObservations);
    CHECK(remote.relayedReadObservations.hasNewestSimTick() == ring->hasNewestSimTick());
    CHECK(remote.relayedReadObservations.newestSimTick() == ring->newestSimTick());

    std::size_t filled = 0u;
    std::size_t mismatched = 0u;
    for (std::size_t index = 0u; index < ring->size(); ++index)
    {
        const RelayedReadObservation* const source = ring->at(index);
        const RelayedReadObservation* const copied = remote.relayedReadObservations.at(index);
        if ((source == nullptr) != (copied == nullptr))
        {
            ++mismatched;
            continue;
        }
        if (source == nullptr)
            continue;
        ++filled;
        mismatched += sameObservation(*source, *copied) ? 0u : 1u;
    }
    CHECK(mismatched == 0u);
    CHECK(filled == kNewestTick);

    const DisplaySnapshot::Entry& local = *snapshot->find(kLocalId);
    CHECK_FALSE(local.hasRelayedReadObservations);
    CHECK_FALSE(local.relayedReadObservations.hasNewestSimTick());
}

TEST_CASE("SimulationDisplaySnapshot.RelayedInputArrivalsAreTheRingsCopy", "[Mailbox]")
{
    const DisplayRig rig;
    const auto snapshot = rig.fill();

    const RelayedInputArrivalRing* const ring =
        rig.resolution.getDiagnostics().relayedInputArrivals<DisplayMockSimulatable>(kRemoteId);
    REQUIRE(ring != nullptr);

    const DisplaySnapshot::Entry& remote = *snapshot->find(kRemoteId);
    CHECK(remote.hasRelayedInputArrivals);

    std::size_t filled = 0u;
    std::size_t mismatched = 0u;
    for (std::size_t index = 0u; index < ring->size(); ++index)
    {
        const RelayedInputArrival* const source = ring->at(index);
        const RelayedInputArrival* const copied = remote.relayedInputArrivals.at(index);
        if ((source == nullptr) != (copied == nullptr))
        {
            ++mismatched;
            continue;
        }
        if (source == nullptr)
            continue;
        ++filled;
        mismatched += (source->captureTick == copied->captureTick && source->arrivedAtSimTick == copied->arrivedAtSimTick)
            ? 0u : 1u;
    }
    CHECK(mismatched == 0u);
    CHECK(filled == kNewestTick / 3u);
    CHECK(remote.relayedInputArrivals.findArrival(2u) != nullptr);

    const DisplaySnapshot::Entry& local = *snapshot->find(kLocalId);
    CHECK_FALSE(local.hasRelayedInputArrivals);
    CHECK(local.relayedInputArrivals.findArrival(2u) == nullptr);
}

TEST_CASE("SimulationDisplaySnapshot.AStagedRelayFillsWhatTheStageHeldNotTheLiveStores", "[Mailbox]")
{
    DisplayRig rig;
    DisplayRelayStage<DisplayMockInput, DisplaySnapshot::kMaxSimulatables> stage;
    stageDisplayRelay<DisplayMockSimulatable>(stage, rig.storage, rig.resolution);
    REQUIRE(stage.count == 2u);
    CHECK(stage.relayedInputArrivals(kRemoteId) != nullptr);
    CHECK(stage.relayedInputArrivals(kLocalId) == nullptr);
    CHECK_FALSE(stage.lastRelayedInput(kLocalId).has_value());
    REQUIRE(stage.lastRelayedInput(kRemoteId).has_value());

    const auto before = rig.fill();
    const std::uint32_t laterCapture = kNewestTick + 5u;
    DisplayRingBuffer ring = displayRingCarrying(laterCapture, 7);
    REQUIRE(rig.resolution.ingestRelayRing<DisplayMockSimulatable>(kRemoteId, ring).outcome == RelayedInputIngestOutcome::Consumed);

    const auto live = rig.fill();
    auto staged = std::make_unique<DisplaySnapshot>();
    DisplayClockReading clock;
    clock.predictionTick = kNewestTick;
    fillSimulationDisplaySnapshot(*staged, rig.storage, rig.resolution, rig.reconciliation, clock, DisplayTierReading{}, stage);

    const DisplaySnapshot::Entry& beforeRemote = *before->find(kRemoteId);
    const DisplaySnapshot::Entry& liveRemote = *live->find(kRemoteId);
    const DisplaySnapshot::Entry& stagedRemote = *staged->find(kRemoteId);
    CHECK(liveRemote.relayedInputArrivals.findArrival(laterCapture) != nullptr);
    REQUIRE(liveRemote.lastRelayedInput.has_value());
    CHECK(liveRemote.lastRelayedInput->value == 7);

    CHECK(stagedRemote.hasRelayedInputArrivals);
    CHECK(stagedRemote.relayedInputArrivals.findArrival(laterCapture) == nullptr);
    CHECK(stagedRemote.relayedInputArrivals.findArrival(2u) != nullptr);
    REQUIRE(stagedRemote.lastRelayedInput.has_value());
    REQUIRE(beforeRemote.lastRelayedInput.has_value());
    CHECK(stagedRemote.lastRelayedInput->value == beforeRemote.lastRelayedInput->value);
    CHECK(stagedRemote.lastRelayedInput->value != 7);
    CHECK(stagedRemote.vizState.getState().position == liveRemote.vizState.getState().position);

    const DisplaySnapshot::Entry& stagedLocal = *staged->find(kLocalId);
    CHECK_FALSE(stagedLocal.hasRelayedInputArrivals);
    CHECK_FALSE(stagedLocal.lastRelayedInput.has_value());
}

TEST_CASE("SimulationDisplaySnapshot.AppliedWindowMatchesTheReconciliation", "[Mailbox]")
{
    const DisplayRig rig;
    const auto snapshot = rig.fill();

    for (const unsigned int id : { kLocalId, kRemoteId })
    {
        const DisplaySnapshot::Entry& entry = *snapshot->find(id);
        CHECK(entry.applied.hasCorrectionCache());
        for (std::uint32_t back = 0u; back < kAppliedWindow; ++back)
        {
            const std::uint32_t tick = kNewestTick - back;
            const AppliedCaptureRef source = rig.reconciliation.getAppliedCaptureTickRef<DisplayMockSimulatable>(id, tick);
            const AppliedCaptureRef copied = entry.applied.appliedCaptureRef(tick);
            CHECK(copied.kind == source.kind);
            CHECK(copied.captureTick == source.captureTick);
            CHECK(source.kind == AppliedCaptureRefKind::NoRef);

            const std::optional<SlotStateProvenance> sourceProvenance =
                rig.reconciliation.getDiagnostics().slotStateProvenance<DisplayMockSimulatable>(id, tick);
            REQUIRE(sourceProvenance.has_value());
            CHECK(entry.applied.slotProvenance(tick) == sourceProvenance);
        }
        const std::uint32_t outside = kNewestTick - static_cast<std::uint32_t>(kAppliedWindow);
        CHECK(entry.applied.appliedCaptureRef(outside).kind == AppliedCaptureRefKind::NoSlot);
        CHECK_FALSE(entry.applied.slotProvenance(outside).has_value());
        CHECK(entry.applied.appliedCaptureRef(kNewestTick + 1u).kind == AppliedCaptureRefKind::NoSlot);
    }

    DisplayRig withoutCache;
    withoutCache.reconciliation.removeCacheFor<DisplayMockSimulatable>(kRemoteId);
    const auto uncached = withoutCache.fill();
    CHECK_FALSE(uncached->find(kRemoteId)->applied.hasCorrectionCache());
    CHECK(uncached->find(kRemoteId)->applied.appliedCaptureRef(kNewestTick).kind == AppliedCaptureRefKind::NoSlot);
    CHECK_FALSE(uncached->find(kRemoteId)->applied.slotProvenance(kNewestTick).has_value());
}

TEST_CASE("SimulationDisplaySnapshot.ClockReadingIsTheClocksAndTheEstimatorsAnswer", "[Mailbox]")
{
    const TimeConfig      config;
    NetworkTimeEstimator  estimator(config, nullptr);
    ClientPredictionClock clock(config, estimator, nullptr);
    for (int advance = 0; advance < 7; ++advance)
        clock.advancePrediction();

    const DisplayClockReading reading = readDisplayClock(clock, estimator);

    CHECK(reading.predictionTick == clock.getPredictionTick());
    CHECK(reading.predictionTick > 0u);
    CHECK(reading.stepDeltaSeconds == clock.getPredictionStep().getDeltaSeconds());
    CHECK(reading.stepDeltaSeconds > 0.f);
    CHECK(reading.targetTick == estimator.getTargetPredictionTick());
    CHECK(reading.authorityTick == estimator.getLastAuthorityTick());
    CHECK(reading.predictionOffsetTicks == estimator.getPredictionOffsetTicks());
    CHECK(reading.pendingAction == clock.evaluateDrift());
    CHECK(reading.stallDebtTicks == clock.getRequiredInputDelayIncreaseStallTicks());
    CHECK(reading.events.skipCount == clock.getDiagnostics().skipCount());
    CHECK(reading.events.lastSkipTick == clock.getDiagnostics().lastSkipTick());
    CHECK(reading.events.stallCount == clock.getDiagnostics().stallCount());
    CHECK(reading.events.lastStallTick == clock.getDiagnostics().lastStallTick());
    CHECK(reading.events.hardResyncCount == clock.getDiagnostics().hardResyncCount());
    CHECK(reading.events.lastHardResyncFromTick == clock.getDiagnostics().lastHardResyncFromTick());
    CHECK(reading.events.lastHardResyncToTick == clock.getDiagnostics().lastHardResyncToTick());

    const DisplayRig rig;
    auto snapshot = std::make_unique<DisplaySnapshot>();
    fillSimulationDisplaySnapshot(*snapshot, rig.storage, rig.resolution, rig.reconciliation, reading, DisplayTierReading{});
    CHECK(snapshot->clock.predictionTick == reading.predictionTick);
    CHECK(snapshot->clock.stepDeltaSeconds == reading.stepDeltaSeconds);
    CHECK(snapshot->clock.pendingAction == reading.pendingAction);
    CHECK(snapshot->clock.events.hardResyncCount == reading.events.hardResyncCount);
}

TEST_CASE("SimulationDisplaySnapshot.TierReadingIsTheConsumersAndTheFloorsAnswer", "[Mailbox]")
{
    TimeConfig             config;
    ReplicatedTierConsumer consumer(config);
    consumer.onReplicatedTierReceived(2);
    config.relayDelayFloorTicks = 3;

    const DisplayTierReading reading = readDisplayTier(consumer, config, 5);

    CHECK(reading.hasReceivedTier == consumer.hasReceivedTier());
    CHECK(reading.hasReceivedTier);
    CHECK(reading.tierIndex == consumer.currentTierIndex());
    CHECK(reading.tierIndex == 2);
    CHECK(reading.tierInputDelayTicks == consumer.effectiveInputDelayTicks());
    CHECK(reading.relayDelayFloorTicks == 3);
    CHECK(reading.clientEffectiveInputDelayTicks == 5);

    const DisplayRig rig;
    auto snapshot = std::make_unique<DisplaySnapshot>();
    DisplayClockReading clock;
    clock.predictionTick = kNewestTick;
    fillSimulationDisplaySnapshot(*snapshot, rig.storage, rig.resolution, rig.reconciliation, clock, reading);
    CHECK(snapshot->tier.hasReceivedTier == reading.hasReceivedTier);
    CHECK(snapshot->tier.tierIndex == reading.tierIndex);
    CHECK(snapshot->tier.tierInputDelayTicks == reading.tierInputDelayTicks);
    CHECK(snapshot->tier.relayDelayFloorTicks == reading.relayDelayFloorTicks);
    CHECK(snapshot->tier.clientEffectiveInputDelayTicks == reading.clientEffectiveInputDelayTicks);
}

TEST_CASE("SimulationDisplaySnapshot.TheReaderSeesAPublishOnlyOnceItIsCommitted", "[Mailbox]")
{
    const DisplayRig rig;
    auto publisher = std::make_unique<DisplayPublisher>();

    CHECK(publisher->acquire_GameThread() == nullptr);

    const auto fillFromRig = [&rig](DisplaySnapshot& snapshot)
    {
        DisplayClockReading clock;
        clock.predictionTick = kNewestTick;
        fillSimulationDisplaySnapshot(snapshot, rig.storage, rig.resolution, rig.reconciliation, clock, DisplayTierReading{});
    };

    REQUIRE(publisher->publish_Step(fillFromRig));
    const DisplaySnapshot* const first = publisher->acquire_GameThread();
    REQUIRE(first != nullptr);
    CHECK(first->publishSequence == 1u);
    CHECK(first->entryCount == 2u);
    CHECK(first->find(kRemoteId)->vizState.getState().position == 22);
    publisher->release_GameThread(first);

    REQUIRE(publisher->publish_Step(fillFromRig));
    const DisplaySnapshot* const newest   = publisher->acquire_GameThread(0u);
    const DisplaySnapshot* const previous = publisher->acquire_GameThread(1u);
    REQUIRE(newest != nullptr);
    REQUIRE(previous != nullptr);
    CHECK(newest->publishSequence == 2u);
    CHECK(previous->publishSequence == 1u);
    publisher->release_GameThread(previous);
    publisher->release_GameThread(newest);
    CHECK(publisher->drops() == 0u);
}

TEST_CASE("SimulationDisplaySnapshot.ARefreshedViewFollowsEveryPublish", "[Mailbox]")
{
    auto publisher = std::make_unique<DisplayPublisher>();
    const auto fillWithEntries = [](std::size_t count)
    {
        return [count](DisplaySnapshot& snapshot)
        {
            snapshot.entryCount = count;
            for (std::size_t index = 0u; index < count; ++index)
                snapshot.entries[index].simId = static_cast<unsigned int>(index + 1u);
        };
    };

    const DisplaySnapshot* view = publisher->refreshView_GameThread(nullptr);
    CHECK(view == nullptr);

    REQUIRE(publisher->publish_Step(fillWithEntries(0u)));
    view = publisher->refreshView_GameThread(view);
    REQUIRE(view != nullptr);
    CHECK(view->publishSequence == 1u);
    CHECK(view->find(1u) == nullptr);

    REQUIRE(publisher->publish_Step(fillWithEntries(2u)));
    view = publisher->refreshView_GameThread(view);
    REQUIRE(view != nullptr);
    CHECK(view->publishSequence == 2u);
    CHECK(view->find(1u) != nullptr);
    CHECK(view->find(2u) != nullptr);

    view = publisher->refreshView_GameThread(view);
    REQUIRE(view != nullptr);
    CHECK(view->publishSequence == 2u);

    std::uint64_t followed = 0u;
    for (std::uint64_t sequence = 3u; sequence <= 3u + 4u * DisplayPublisher::Channel::kCapacity; ++sequence)
    {
        REQUIRE(publisher->publish_Step(fillWithEntries(2u)));
        view = publisher->refreshView_GameThread(view);
        REQUIRE(view != nullptr);
        CHECK(view->publishSequence == sequence);
        ++followed;
    }
    CHECK(followed == 1u + 4u * DisplayPublisher::Channel::kCapacity);
    CHECK(publisher->drops() == 0u);
    publisher->release_GameThread(view);
}

namespace
{
    void stampSnapshot(DisplaySnapshot& snapshot, std::uint64_t stamp)
    {
        const auto value = static_cast<std::uint32_t>(stamp);
        snapshot.clock.predictionTick          = value;
        snapshot.clock.targetTick              = value;
        snapshot.clock.events.hardResyncCount  = value;
        snapshot.tier.tierIndex                = static_cast<std::int32_t>(value);
        snapshot.tier.relayDelayFloorTicks     = static_cast<std::int32_t>(value);
        snapshot.entryCount                    = DisplaySnapshot::kMaxSimulatables;
        for (DisplaySnapshot::Entry& entry : snapshot.entries)
        {
            entry.simId = value;
            entry.vizState.editState().position = static_cast<std::int32_t>(value);
            entry.lastRelayedInput = DisplayMockInput{ static_cast<std::int32_t>(value) };
            entry.relayedInputArrivals.note(value, value);
        }
    }

    std::uint64_t tornFields(const DisplaySnapshot& snapshot)
    {
        const auto expected = static_cast<std::uint32_t>(snapshot.publishSequence);
        std::uint64_t torn = 0u;
        torn += snapshot.clock.predictionTick != expected ? 1u : 0u;
        torn += snapshot.clock.targetTick != expected ? 1u : 0u;
        torn += snapshot.clock.events.hardResyncCount != expected ? 1u : 0u;
        torn += snapshot.tier.tierIndex != static_cast<std::int32_t>(expected) ? 1u : 0u;
        torn += snapshot.tier.relayDelayFloorTicks != static_cast<std::int32_t>(expected) ? 1u : 0u;
        torn += snapshot.entryCount != DisplaySnapshot::kMaxSimulatables ? 1u : 0u;
        for (const DisplaySnapshot::Entry& entry : snapshot.entries)
        {
            torn += entry.simId != expected ? 1u : 0u;
            torn += entry.vizState.getState().position != static_cast<std::int32_t>(expected) ? 1u : 0u;
            torn += (!entry.lastRelayedInput.has_value() || entry.lastRelayedInput->value != static_cast<std::int32_t>(expected))
                ? 1u : 0u;
            torn += entry.relayedInputArrivals.findArrival(expected) == nullptr ? 1u : 0u;
        }
        return torn;
    }
} // namespace

TEST_CASE("SimulationDisplaySnapshot.StressAReaderHoldingTwoNeverSeesATornSnapshot", "[Mailbox][og][Stress]")
{
    constexpr std::uint64_t kPublishes = 100'000u;
    auto publisher = std::make_unique<DisplayPublisher>();
    std::atomic<bool> start{ false };
    std::atomic<bool> writerDone{ false };
    std::uint64_t writerRefused = 0u;

    std::thread writer([&]
    {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (std::uint64_t publish = 1u; publish <= kPublishes; ++publish)
        {
            if (!publisher->publish_Step([publish](DisplaySnapshot& snapshot) { stampSnapshot(snapshot, publish); }))
                ++writerRefused;
        }
        writerDone.store(true, std::memory_order_release);
    });

    std::uint64_t pairs = 0u;
    std::uint64_t torn = 0u;
    std::uint64_t changedWhileHeld = 0u;
    std::uint64_t pairNotOlder = 0u;
    std::uint64_t newestWentBackwards = 0u;
    std::uint64_t distinct = 0u;
    std::uint64_t lastNewest = 0u;
    start.store(true, std::memory_order_release);
    for (;;)
    {
        const bool done = writerDone.load(std::memory_order_acquire);
        const DisplaySnapshot* const newest   = publisher->acquire_GameThread(0u);
        const DisplaySnapshot* const previous = publisher->acquire_GameThread(1u);
        if (newest != nullptr)
        {
            const std::uint64_t newestSequence = newest->publishSequence;
            torn += tornFields(*newest);
            newestWentBackwards += newestSequence < lastNewest ? 1u : 0u;
            distinct += newestSequence != lastNewest ? 1u : 0u;
            lastNewest = newestSequence;
            if (previous != nullptr)
            {
                ++pairs;
                const std::uint64_t previousSequence = previous->publishSequence;
                torn += tornFields(*previous);
                pairNotOlder += previousSequence >= newestSequence ? 1u : 0u;
                std::this_thread::yield();
                torn += tornFields(*previous);
                changedWhileHeld += previous->publishSequence != previousSequence ? 1u : 0u;
            }
            torn += tornFields(*newest);
            changedWhileHeld += newest->publishSequence != newestSequence ? 1u : 0u;
        }
        publisher->release_GameThread(previous);
        publisher->release_GameThread(newest);
        if (done)
            break;
    }
    writer.join();

    const DisplaySnapshot* const last = publisher->acquire_GameThread(0u);
    INFO("pairs: " << pairs << ", distinct newest: " << distinct);
    REQUIRE(torn == 0u);
    REQUIRE(changedWhileHeld == 0u);
    REQUIRE(pairNotOlder == 0u);
    REQUIRE(newestWentBackwards == 0u);
    REQUIRE(pairs > 0u);
    REQUIRE(distinct >= 2u);
    REQUIRE(writerRefused == 0u);
    REQUIRE(publisher->drops() == 0u);
    REQUIRE(last != nullptr);
    REQUIRE(last->publishSequence == kPublishes);
    publisher->release_GameThread(last);
}

#endif // WITH_LOW_LEVEL_TESTS
