// SPDX-License-Identifier: MPL-2.0
// docs/RedundancyWindowTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/Network/ServerReceptionCoordinator.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationInputResolution.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationQueues.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/SimulationNetSync.h"
#include "LinkModel.h"
#include "StandaloneTestHandle.h"

namespace redundancyWindow
{
    struct WinState
    {
        std::int32_t position = 0;
        bool isSimilarTo(const WinState& other) const { return position == other.position; }
    };

    struct WinInput
    {
        std::int32_t captureTick = 0;
    };
}

template <>
struct SerializableFields<redundancyWindow::WinInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(redundancyWindow::WinInput, captureTick));
    }
};

namespace redundancyWindow
{
    using linkModel::LinkModel;

    class WinAllState
    {
    public:
        const WinState& getState() const { return m_state; }
        WinState&       editState()       { return m_state; }

    private:
        WinState m_state;
    };

    struct WinSim
    {
        using StateType = WinState;
        using InputType = WinInput;

        WinAllState m_allState;
        WinAllState m_vizState;

        const WinAllState& getAllState() const { return m_allState; }
        WinAllState&       editAllState()       { return m_allState; }
        void               updateVizState() { m_vizState = m_allState; }
        const WinAllState& getVizState() const { return m_vizState; }
    };

    static_assert(SimulatableState<WinSim>, "WinSim must satisfy the storage and reconciliation concept");

    struct WireBuffer
    {
        std::vector<std::uint8_t> bytes;

        WireBuffer() = default;
        explicit WireBuffer(const LinkModel::Payload& payload)
            : bytes(payload)
        {
        }

        std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

        void bundleAddZeroedBytes(std::int32_t count)
        {
            bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u);
        }

        void bundleTruncateTo(std::int32_t byteCount)
        {
            if (static_cast<std::size_t>(byteCount) < bytes.size())
            {
                bytes.resize(static_cast<std::size_t>(byteCount));
            }
        }

        template <typename T>
        void writeToBuffer(std::uint32_t offset, const T& value)
        {
            std::memcpy(bytes.data() + offset, &value, sizeof(T));
        }

        template <typename T>
        T readFromBuffer(std::uint32_t offset) const
        {
            T value;
            std::memcpy(&value, bytes.data() + offset, sizeof(T));
            return value;
        }
    };

    struct WinStateSyncBuffer
    {
        std::uint32_t lastTick               = 0;
        std::uint32_t lastAppliedCaptureTick = kNoInputCaptureTick;

        void write(const WinState&, std::uint32_t tick, std::uint32_t appliedCaptureTick)
        {
            lastTick               = tick;
            lastAppliedCaptureTick = appliedCaptureTick;
        }
        void write(const WinState& state, std::uint32_t tick) { write(state, tick, kNoInputCaptureTick); }

        std::uint32_t readInto(WinState&) const { return lastTick; }
        std::uint32_t getAppliedCaptureTick() const { return lastAppliedCaptureTick; }
    };

    struct SentBundle
    {
        std::uint32_t              tick  = 0;
        std::uint32_t              slots = 0;
        LinkModel::Payload         bytes;
        std::vector<std::uint32_t> slotTicks;
    };

    struct WinOwner
    {
        using SyncedCorrectionBufferType = WinStateSyncBuffer;
        using RelayedInputRingType       = WireBuffer;

        std::function<void(const WinStateSyncBuffer&)>          onCorrectionStateReceived;
        std::function<void(const WireBuffer&)>                  onRelayedInputReceived;
        std::function<void(std::uint32_t, const WinInput&)>     onRemoteMoveReceived;
        WireBuffer                                              relayedInputRing;
        WinStateSyncBuffer                                      correctionStateBuffer;
        std::vector<SentBundle>                                 sent;

        void setOnCorrectionStateReceivedCallback(std::function<void(const WinStateSyncBuffer&)> fn)
        {
            onCorrectionStateReceived = std::move(fn);
        }
        void clearOnCorrectionStateReceivedCallback() { onCorrectionStateReceived = nullptr; }
        void setOnRelayedInputReceivedCallback(std::function<void(const WireBuffer&)> fn)
        {
            onRelayedInputReceived = std::move(fn);
        }
        void clearOnRelayedInputReceivedCallback() { onRelayedInputReceived = nullptr; }
        const WireBuffer& getRelayedInputRing() const { return relayedInputRing; }
        WinStateSyncBuffer& getSyncedCorrectionStateBuffer() { return correctionStateBuffer; }
        void setOnRemoteMoveReceivedCallback(std::function<void(std::uint32_t, const WinInput&)> fn)
        {
            onRemoteMoveReceived = std::move(fn);
        }
        void clearOnRemoteMoveReceivedCallback() { onRemoteMoveReceived = nullptr; }

        void sendLocalInputToAuthority(const PendingInputQueue<WinInput>& queue, std::uint32_t currentTick,
                                       std::uint32_t redundancyDepth)
        {
            WireBuffer   bundle;
            SentBundle   record;
            record.tick  = currentTick;
            record.slots = redundancyDepth;
            const std::uint8_t depth = redundancyDepth < inputRedundancyBundle::kMaxSlots
                ? static_cast<std::uint8_t>(redundancyDepth)
                : inputRedundancyBundle::kMaxSlots;
            queue.forEachRecent(currentTick, static_cast<std::size_t>(depth),
                [&bundle, &record](std::uint32_t captureTick, const WinInput& input)
                {
                    inputRedundancyBundle::appendSlot<WinInput>(bundle, captureTick, input);
                    record.slotTicks.push_back(captureTick);
                });
            record.bytes = bundle.bytes;
            sent.push_back(std::move(record));
        }
    };

    static_assert(PredictionSyncedBufferOwnerConcept<WinOwner, WinState, WinInput>,
        "WinOwner must satisfy PredictionSyncedBufferOwnerConcept");
}

template <>
struct SimulatableOwnerTraits<redundancyWindow::WinSim>
{
    using PredictionOwnerType = redundancyWindow::WinOwner;
    using AuthorityOwnerType  = redundancyWindow::WinOwner;
};

namespace redundancyWindow
{
    using WinStorage        = SimulationObjectStorage<WinSim>;
    using WinReconciliation = SimulationReconciliation<WinSim>;
    using WinResolution     = SimulationInputResolution<WinSim>;
    using WinNetSync        = SimulationNetSync<WinSim>;
    using WinCoordinator    = ServerReceptionCoordinator<FStandaloneTestHandle, WinSim>;

    constexpr unsigned int kSenderId        = 21u;
    constexpr std::uint8_t kPlayerSlot      = 0u;
    constexpr std::int32_t kServerDelayTicks = 8;

    struct ClientRig
    {
        WinStorage               storage;
        WinReconciliation        reconciliation{ storage };
        WinResolution            resolution{ storage, reconciliation };
        WinNetSync               netSync{ storage, reconciliation, resolution };
        WinOwner                 owner;
        std::vector<std::string> log;

        ClientRig()
        {
            netSync.setLogger([this](const char* line) { log.emplace_back(line); });
            resolution.setNeutralInput<WinSim>(WinInput{});
            join();
        }

        void join()
        {
            registerSimulatable<WinSim>(storage, reconciliation, resolution, netSync, kSenderId, WinSim{}, owner,
                [](const SimulationTimeStep&, const LocalInputCache<WinInput>&) { return WinInput{}; });
        }

        void leave()
        {
            unregisterSimulatable<WinSim>(storage, reconciliation, resolution, netSync, kSenderId, &owner);
        }

        PendingInputQueue<WinInput>& queue()
        {
            PendingInputQueue<WinInput>* found = resolution.findPendingInputQueue<WinSim>(kSenderId);
            REQUIRE(found != nullptr);
            return *found;
        }

        void capture(std::uint32_t tick) { queue().enqueue(tick, WinInput{ static_cast<std::int32_t>(tick) }); }

        std::size_t retained()
        {
            std::size_t count = 0u;
            queue().forEachRecent(~0u, PendingInputQueue<WinInput>::Capacity,
                [&count](std::uint32_t, const WinInput&) { ++count; });
            return count;
        }

        void send(std::uint32_t tick, std::uint32_t depth, std::int32_t redundancyFrames)
        {
            netSync.sendLocalInputToAuthorityAll(tick, depth, redundancyFrames);
        }
    };

    struct UndelayedDeliveries
    {
        std::vector<std::uint32_t> captureTicks;

        void deliverRemoteInput(unsigned int, std::uint32_t captureTick, const WinInput&)
        {
            captureTicks.push_back(captureTick);
        }
    };

    struct Server
    {
        TimeConfig              config;
        WinCoordinator          coordinator{ config };
        FStandaloneTestHandle   wire{ 1u, true };
        UndelayedDeliveries     undelayed;
        std::set<std::uint32_t> released;
        std::set<std::uint32_t> arrived;

        Server()
        {
            for (std::int32_t& delay : config.rttTierInputDelays)
            {
                delay = kServerDelayTicks;
            }
        }

        void receive(const LinkModel::Payload& payload)
        {
            const WireBuffer received(payload);
            inputRedundancyBundle::forEachSlot<WinInput>(received,
                [this](std::uint32_t captureTick, const WinInput&) { arrived.insert(captureTick); });
            coordinator.receiveInputBundle<WinSim>(kSenderId, wire, kPlayerSlot, received, undelayed);
        }

        std::int32_t effectiveDelay() const
        {
            return coordinator.delayQueue().effectiveDelay(WinCoordinator::SlotKey(wire, kPlayerSlot));
        }

        void drain(std::int32_t serverTick)
        {
            coordinator.releaseDelayedInputs<WinSim>(serverTick, 1,
                [this](unsigned int, std::uint32_t captureTick, const WinInput&)
                {
                    released.insert(captureTick);
                    return true;
                });
        }

        std::size_t pendingCount() const
        {
            return coordinator.delayQueue().pendingCount<WinSim>(WinCoordinator::SlotKey(wire, kPlayerSlot));
        }
    };

    struct RunResult
    {
        std::vector<SentBundle>  sent;
        std::vector<std::size_t> retainedAfterSend;
        std::vector<std::size_t> capturedAtSend;
        std::set<std::uint32_t>  neverReleased;
        std::set<std::uint32_t>  neverArrived;
        std::size_t              pendingAtEnd   = 0u;
        std::size_t              lateDropped    = 0u;
        std::int32_t             serverDelay    = 0;
        std::vector<std::string> log;
    };

    std::vector<std::uint32_t> steadyFrames(std::uint32_t stepsPerFrame, std::size_t frames)
    {
        std::vector<std::uint32_t> schedule{ 1u };
        schedule.insert(schedule.end(), frames, stepsPerFrame);
        return schedule;
    }

    RunResult runSession(const std::vector<std::uint32_t>& stepsPerFrame, std::uint32_t depth,
                         std::int32_t redundancyFrames, std::optional<std::size_t> droppedSend)
    {
        ClientRig client;
        Server    server;
        linkModel::Config linkConfig;
        linkConfig.seed           = 7u;
        linkConfig.baseDelayTicks = 1.0;
        LinkModel uplink(linkConfig);

        std::vector<std::uint32_t> frameEnds;
        std::uint32_t              lastCaptureTick = 0u;
        for (const std::uint32_t steps : stepsPerFrame)
        {
            lastCaptureTick += steps;
            frameEnds.push_back(lastCaptureTick);
        }

        RunResult   result;
        std::size_t nextFrame = 0u;
        std::size_t captured  = 0u;
        const std::int32_t lastServerTick = static_cast<std::int32_t>(lastCaptureTick) + 40;
        for (std::int32_t tick = 1; tick <= lastServerTick; ++tick)
        {
            const std::uint32_t clientTick = static_cast<std::uint32_t>(tick);
            if (clientTick <= lastCaptureTick)
            {
                client.capture(clientTick);
                ++captured;
            }
            while (nextFrame < frameEnds.size() && frameEnds[nextFrame] == clientTick)
            {
                const std::size_t sendIndex = client.owner.sent.size();
                client.send(clientTick, depth, redundancyFrames);
                result.retainedAfterSend.push_back(client.retained());
                result.capturedAtSend.push_back(captured);
                if (!droppedSend.has_value() || *droppedSend != sendIndex)
                {
                    uplink.send(client.owner.sent.back().bytes, static_cast<double>(tick));
                }
                ++nextFrame;
            }
            uplink.deliverDue(static_cast<double>(tick),
                [&server](const LinkModel::Payload& payload) { server.receive(payload); });
            server.drain(tick);
        }

        for (std::uint32_t tick = 1u; tick <= lastCaptureTick; ++tick)
        {
            if (server.released.find(tick) == server.released.end())
            {
                result.neverReleased.insert(tick);
            }
            if (server.arrived.find(tick) == server.arrived.end())
            {
                result.neverArrived.insert(tick);
            }
        }
        result.serverDelay  = server.effectiveDelay();
        result.sent         = client.owner.sent;
        result.pendingAtEnd = server.pendingCount();
        result.lateDropped  = server.coordinator.lateDroppedCount();
        result.log          = client.log;
        return result;
    }

    std::string slotSignature(const std::vector<SentBundle>& sent)
    {
        std::ostringstream out;
        for (const SentBundle& bundle : sent)
        {
            out << bundle.tick << ':';
            for (std::size_t i = 0; i < bundle.slotTicks.size(); ++i)
            {
                out << (i == 0u ? "" : ",") << bundle.slotTicks[i];
            }
            out << '|';
        }
        return out.str();
    }

    std::uint64_t bytesDigest(const std::vector<SentBundle>& sent)
    {
        std::uint64_t hash = 14695981039346656037ull;
        for (const SentBundle& bundle : sent)
        {
            for (const std::uint8_t byte : bundle.bytes)
            {
                hash = (hash ^ byte) * 1099511628211ull;
            }
            hash = (hash ^ 0xFFu) * 1099511628211ull;
        }
        return hash;
    }

    std::size_t totalBytes(const std::vector<SentBundle>& sent)
    {
        std::size_t total = 0u;
        for (const SentBundle& bundle : sent)
        {
            total += bundle.bytes.size();
        }
        return total;
    }

    const std::vector<std::uint32_t> kIrregularFrames{ 1u, 2u, 0u, 1u, 3u, 1u, 0u, 2u, 4u, 1u, 2u, 1u, 3u, 0u, 1u, 2u };
}

using namespace redundancyWindow;

TEST_CASE("RedundancyWindow: at redundancyFrames = 0 the bundle bytes equal the pre-task send path's",
          "[Network][RedundancyWindow]")
{
    SECTION("depth 3, the compiled default")
    {
        REQUIRE(TimeConfig{}.redundancyDepthTicks == 3);
        const RunResult run = runSession(kIrregularFrames, 3u, 0, std::nullopt);
        CHECK(slotSignature(run.sent) == "1:1|3:1,2,3|3:1,2,3|4:2,3,4|7:5,6,7|8:6,7,8|8:6,7,8|10:8,9,10|14:12,13,14|15:13,14,15|17:15,16,17|18:16,17,18|21:19,20,21|21:19,20,21|22:20,21,22|24:22,23,24|");
        CHECK(totalBytes(run.sent) == 400u);
        CHECK(bytesDigest(run.sent) == 0xae830467fb821bf3ull);
    }

    SECTION("depth 5, the 100 Hz interim value")
    {
        const RunResult run = runSession(kIrregularFrames, 5u, 0, std::nullopt);
        CHECK(slotSignature(run.sent) == "1:1|3:1,2,3|3:1,2,3|4:1,2,3,4|7:3,4,5,6,7|8:4,5,6,7,8|8:4,5,6,7,8|10:6,7,8,9,10|14:10,11,12,13,14|15:11,12,13,14,15|17:13,14,15,16,17|18:14,15,16,17,18|21:17,18,19,20,21|21:17,18,19,20,21|22:18,19,20,21,22|24:20,21,22,23,24|");
        CHECK(totalBytes(run.sent) == 600u);
        CHECK(bytesDigest(run.sent) == 0xbd3fb49e4a6fb440ull);
    }

    SECTION("depth 10, above the wire cap")
    {
        const RunResult run = runSession(kIrregularFrames, 10u, 0, std::nullopt);
        CHECK(slotSignature(run.sent) == "1:1|3:1,2,3|3:1,2,3|4:1,2,3,4|7:1,2,3,4,5,6,7|8:1,2,3,4,5,6,7,8|8:1,2,3,4,5,6,7,8|10:3,4,5,6,7,8,9,10|14:7,8,9,10,11,12,13,14|15:8,9,10,11,12,13,14,15|17:10,11,12,13,14,15,16,17|18:11,12,13,14,15,16,17,18|21:14,15,16,17,18,19,20,21|21:14,15,16,17,18,19,20,21|22:15,16,17,18,19,20,21,22|24:17,18,19,20,21,22,23,24|");
        CHECK(totalBytes(run.sent) == 880u);
        CHECK(bytesDigest(run.sent) == 0x49913f1f490c75b5ull);
        CHECK(run.retainedAfterSend.back() == 10u);
    }
}

TEST_CASE("RedundancyWindow: redundancySlots returns the depth at 0 frames and never exceeds kMaxSlots otherwise",
          "[Network][RedundancyWindow]")
{
    const std::vector<float>         emas{ 0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 4.0f, 8.0f, 100.0f, -3.0f };
    const std::vector<std::uint32_t> depths{ 0u, 1u, 3u, 5u, 8u, 10u };
    for (const float ema : emas)
    {
        for (const std::uint32_t depth : depths)
        {
            CAPTURE(ema, depth);
            CHECK(WinNetSync::redundancySlots(ema, depth, 0) == depth);
            CHECK(WinNetSync::redundancySlots(ema, depth, -4) == depth);
            for (std::int32_t frames = 1; frames <= 12; ++frames)
            {
                CAPTURE(frames);
                const std::uint32_t slots = WinNetSync::redundancySlots(ema, depth, frames);
                CHECK(slots <= inputRedundancyBundle::kMaxSlots);
                CHECK(slots >= std::min<std::uint32_t>(depth, inputRedundancyBundle::kMaxSlots));
            }
        }
    }

    CHECK(WinNetSync::redundancySlots(1.0f, 3u, 3) == 3u);
    CHECK(WinNetSync::redundancySlots(0.6f, 3u, 3) == 3u);
    CHECK(WinNetSync::redundancySlots(2.0f, 3u, 3) == 6u);
    CHECK(WinNetSync::redundancySlots(3.0f, 3u, 3) == 8u);
    CHECK(WinNetSync::redundancySlots(4.0f, 3u, 3) == 8u);
    CHECK(WinNetSync::redundancySlots(2.0f, 3u, 1) == 3u);
    CHECK(WinNetSync::redundancySlots(2.0f, 5u, 3) == 6u);
    CHECK(WinNetSync::redundancySlots(2.0f, 10u, 3) == 8u);
    CHECK(WinNetSync::redundancySlots(1.9f, 3u, 3) == 6u);
    CHECK(WinNetSync::redundancySlots(2.1f, 3u, 3) == 6u);
}

TEST_CASE("RedundancyWindow: 1 to 4 steps per frame, any single lost bundle leaves 0 never-parked ticks at redundancyFrames = 3",
          "[Network][RedundancyWindow]")
{
    constexpr std::int32_t kRedundancyFrames = 3;
    const std::uint32_t    depth             = static_cast<std::uint32_t>(TimeConfig{}.redundancyDepthTicks);

    for (std::uint32_t stepsPerFrame = 1u; stepsPerFrame <= 4u; ++stepsPerFrame)
    {
        DYNAMIC_SECTION("steps per frame " << stepsPerFrame)
        {
            const std::vector<std::uint32_t> schedule = steadyFrames(stepsPerFrame, 14u);
            const RunResult clean = runSession(schedule, depth, kRedundancyFrames, std::nullopt);
            REQUIRE(clean.sent.size() == schedule.size());
            REQUIRE(clean.serverDelay == kServerDelayTicks);
            CHECK(clean.neverReleased.empty());

            const std::uint32_t steadySlots = std::min<std::uint32_t>(
                static_cast<std::uint32_t>(kRedundancyFrames) * stepsPerFrame, inputRedundancyBundle::kMaxSlots);
            for (std::size_t i = 0; i < clean.sent.size(); ++i)
            {
                CAPTURE(i);
                CHECK(clean.sent[i].slots <= inputRedundancyBundle::kMaxSlots);
                CHECK(clean.sent[i].slotTicks.size() <= inputRedundancyBundle::kMaxSlots);
                CHECK(clean.sent[i].slots == (i == 0u ? depth : std::max(depth, steadySlots)));
                CHECK(clean.retainedAfterSend[i]
                      == std::min<std::size_t>(clean.sent[i].slots, clean.capturedAtSend[i]));
            }

            for (std::size_t dropped = 0; dropped + 1u < clean.sent.size(); ++dropped)
            {
                CAPTURE(stepsPerFrame, dropped);
                const RunResult lossy = runSession(schedule, depth, kRedundancyFrames, dropped);
                CHECK(lossy.neverArrived.empty());
                CHECK(lossy.neverReleased.empty());
                CHECK(lossy.pendingAtEnd == 0u);
            }
        }
    }
}

TEST_CASE("RedundancyWindow: at redundancyFrames = 0, two steps per frame loses the lost bundle's middle tick (task 3 (i))",
          "[Network][RedundancyWindow]")
{
    const std::vector<std::uint32_t> schedule = steadyFrames(2u, 14u);
    const RunResult clean = runSession(schedule, 3u, 0, std::nullopt);
    REQUIRE(clean.neverReleased.empty());

    for (std::size_t dropped = 1; dropped + 1u < clean.sent.size(); ++dropped)
    {
        CAPTURE(dropped);
        const RunResult lossy = runSession(schedule, 3u, 0, dropped);
        CHECK(lossy.neverArrived == std::set<std::uint32_t>{ clean.sent[dropped].tick - 1u });
        CHECK(lossy.neverReleased == std::set<std::uint32_t>{ clean.sent[dropped].tick - 1u });
    }
}

TEST_CASE("RedundancyWindow: the send line appends slots=, and the cadence is forgotten at unregistration",
          "[Network][RedundancyWindow]")
{
    ClientRig client;
    for (std::uint32_t tick = 1u; tick <= 6u; ++tick)
    {
        client.capture(tick);
        if (tick % 2u == 0u)
        {
            client.send(tick, 3u, 3);
        }
    }
    REQUIRE(client.owner.sent.size() == 3u);
    CHECK(client.owner.sent[0].slots == 3u);
    CHECK(client.owner.sent[1].slots == 6u);
    CHECK(client.owner.sent[2].slots == 6u);
    CHECK(std::find(client.log.begin(), client.log.end(),
                    std::string("[SendLocalInputToServer] id=21 tick=6 depth=3 slots=6")) != client.log.end());

    client.send(6u, 3u, 0);
    CHECK(client.owner.sent.back().slots == 3u);
    CHECK(client.log.back() == "[SendLocalInputToServer] id=21 tick=6 depth=3 slots=3");

    client.leave();
    client.join();
    for (std::uint32_t tick = 7u; tick <= 8u; ++tick)
    {
        client.capture(tick);
    }
    client.send(8u, 3u, 3);
    CHECK(client.owner.sent.back().slots == 3u);
    client.send(8u, 3u, 3);
    CHECK(client.owner.sent.back().slots == 3u);
}

#endif
