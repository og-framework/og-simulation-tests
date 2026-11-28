// SPDX-License-Identifier: MPL-2.0
// docs/LinkModelPinsTodayTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <set>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/Network/RemoteInputCache.h"
#include "OGSimulation/Network/ServerReceptionCoordinator.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationQueues.h"
#include "LinkModel.h"
#include "StandaloneTestHandle.h"

namespace linkModelPins
{
    struct PinInput
    {
        std::int32_t captureTick = 0;
        std::int32_t arrival     = 0;
    };
}

template <>
struct SerializableFields<linkModelPins::PinInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(linkModelPins::PinInput, captureTick),
                               SIM_MEMBER(linkModelPins::PinInput, arrival));
    }
};

namespace linkModelPins
{
    using linkModel::LinkModel;

    struct PinSim
    {
        using Input = PinInput;
    };

    using PinCoordinator = ServerReceptionCoordinator<FStandaloneTestHandle, PinSim>;

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

    struct UplinkClient
    {
        PendingInputQueue<PinInput> queue;

        void capture(std::uint32_t tick)
        {
            queue.enqueue(tick, PinInput{ static_cast<std::int32_t>(tick), 0 });
        }

        LinkModel::Payload buildAndRelease(std::uint32_t currentTick, std::uint32_t redundancyDepth)
        {
            WireBuffer bundle;
            const std::uint8_t depth = redundancyDepth < inputRedundancyBundle::kMaxSlots
                ? static_cast<std::uint8_t>(redundancyDepth)
                : inputRedundancyBundle::kMaxSlots;
            queue.forEachRecent(currentTick, static_cast<std::size_t>(depth),
                [&bundle](std::uint32_t captureTick, const PinInput& input)
                {
                    inputRedundancyBundle::appendSlot<PinInput>(bundle, captureTick, input);
                });
            queue.releaseAllButRecent(static_cast<std::size_t>(redundancyDepth));
            return bundle.bytes;
        }
    };

    struct UndelayedDeliveries
    {
        std::vector<std::uint32_t> captureTicks;

        void deliverRemoteInput(unsigned int, std::uint32_t captureTick, const PinInput&)
        {
            captureTicks.push_back(captureTick);
        }
    };

    struct RelayStage
    {
        WireBuffer                 stage;
        std::vector<std::uint32_t> relayedTicks;

        void relayRemoteInput(unsigned int, std::uint32_t captureTick, std::uint8_t dA, const PinInput& input)
        {
            relayedTicks.push_back(captureTick);
            (void)relayedInputRing::stageArrival<PinInput>(stage, captureTick, dA, input);
        }
    };

    struct UplinkServer
    {
        static constexpr unsigned int kSenderId   = 7u;
        static constexpr std::uint8_t kPlayerSlot = 0u;

        TimeConfig                        config;
        PinCoordinator                    coordinator{ config };
        FStandaloneTestHandle             wire{ 1u, true };
        UndelayedDeliveries               undelayed;
        RelayStage                        relay;
        std::map<std::uint32_t, PinInput> released;
        std::vector<std::uint32_t>        releaseOrder;
        std::size_t                       arrivals     = 0u;
        bool                              stampArrival = false;
        std::map<std::uint32_t, std::int32_t> firstArrivalCarrying;
        std::map<std::uint32_t, std::int32_t> arrivalByNewestTick;

        void receive(const LinkModel::Payload& payload)
        {
            const std::int32_t ordinal = static_cast<std::int32_t>(++arrivals);
            const WireBuffer   received(payload);

            WireBuffer   fed;
            std::uint32_t newest = 0u;
            inputRedundancyBundle::forEachSlot<PinInput>(received,
                [&](std::uint32_t captureTick, const PinInput& input)
                {
                    firstArrivalCarrying.emplace(captureTick, ordinal);
                    newest = std::max(newest, captureTick);
                    PinInput stamped = input;
                    stamped.arrival  = ordinal;
                    inputRedundancyBundle::appendSlot<PinInput>(fed, captureTick, stamped);
                });
            arrivalByNewestTick[newest] = ordinal;

            coordinator.receiveInputBundle<PinSim>(kSenderId, wire, kPlayerSlot, stampArrival ? fed : received,
                                                   undelayed, relay);
        }

        void drain(std::int32_t serverTick)
        {
            coordinator.releaseDelayedInputs<PinSim>(serverTick, 1,
                [this](unsigned int, std::uint32_t captureTick, const PinInput& input)
                {
                    released.emplace(captureTick, input);
                    releaseOrder.push_back(captureTick);
                    return true;
                });
        }

        std::size_t pendingCount() const
        {
            return coordinator.delayQueue().pendingCount<PinSim>(PinCoordinator::SlotKey(wire, kPlayerSlot));
        }
    };

    struct DownlinkClient
    {
        RemoteInputCache<PinInput> store;
        std::size_t                arrivals = 0u;

        void ingest(const LinkModel::Payload& payload)
        {
            ++arrivals;
            const WireBuffer replicated(payload);
            (void)populateRemoteInputCache<PinInput>(store, replicated);
        }
    };

    struct Session
    {
        std::uint32_t capturesPerSend = 1u;
        std::uint32_t lastCaptureTick = 20u;
        std::int32_t  lastServerTick  = 40;

        UplinkClient   client;
        UplinkServer   server;
        DownlinkClient peer;
        WireBuffer     replicatedRing;
        std::size_t    flushes = 0u;

        void run(LinkModel& uplink, LinkModel* downlink)
        {
            const std::uint32_t depth = static_cast<std::uint32_t>(server.config.redundancyDepthTicks);
            for (std::int32_t tick = 1; tick <= lastServerTick; ++tick)
            {
                const std::uint32_t clientTick = static_cast<std::uint32_t>(tick);
                if (clientTick <= lastCaptureTick)
                {
                    client.capture(clientTick);
                    if (clientTick % capturesPerSend == 0u)
                    {
                        uplink.send(client.buildAndRelease(clientTick, depth), static_cast<double>(tick));
                    }
                }

                uplink.deliverDue(static_cast<double>(tick),
                    [this](const LinkModel::Payload& payload) { server.receive(payload); });
                server.drain(tick);

                if (relayedInputRing::flushStagedInto(replicatedRing, server.relay.stage) > 0u)
                {
                    ++flushes;
                    if (downlink != nullptr)
                    {
                        downlink->send(replicatedRing.bytes, static_cast<double>(tick));
                    }
                }

                if (downlink != nullptr)
                {
                    downlink->deliverDue(static_cast<double>(tick),
                        [this](const LinkModel::Payload& payload) { peer.ingest(payload); });
                }
            }
        }

        std::set<std::uint32_t> neverReleased() const
        {
            std::set<std::uint32_t> missing;
            for (std::uint32_t tick = 1u; tick <= lastCaptureTick; ++tick)
            {
                if (server.released.find(tick) == server.released.end())
                {
                    missing.insert(tick);
                }
            }
            return missing;
        }
    };

    linkModel::Config oneTickLink(std::uint64_t seed)
    {
        linkModel::Config config;
        config.seed           = seed;
        config.baseDelayTicks = 1.0;
        return config;
    }
}

using namespace linkModelPins;

TEST_CASE("LinkModel: seeded loss, fractional delay, both jitter shapes and the reorder window",
          "[Network][LinkModel]")
{
    SECTION("the same seed draws the same losses, and a different seed draws different ones")
    {
        linkModel::Config config = oneTickLink(8u);
        config.lossProbability   = 0.1;
        LinkModel first(config);
        LinkModel again(config);
        config.seed = 9u;
        LinkModel other(config);
        for (int i = 0; i < 200; ++i)
        {
            first.send({ 1u }, 0.0);
            again.send({ 1u }, 0.0);
            other.send({ 1u }, 0.0);
        }
        CHECK(first.lostSends() == again.lostSends());
        CHECK(first.lostSends() != other.lostSends());
        CHECK(first.lostSends().size() + first.inFlightCount() == 200u);
    }

    SECTION("a fractional delay delivers on the first whole tick at or after send + delay")
    {
        linkModel::Config config;
        config.baseDelayTicks = 1.5;
        LinkModel link(config);
        link.send({ 42u }, 10.0);
        std::vector<LinkModel::Payload> got;
        const auto collect = [&got](const LinkModel::Payload& p) { got.push_back(p); };
        CHECK(link.deliverDue(11.0, collect) == 0u);
        CHECK(link.deliverDue(11.5, collect) == 1u);
        REQUIRE(got.size() == 1u);
        CHECK(got[0] == LinkModel::Payload{ 42u });
    }

    SECTION("two-point jitter adds either nothing or the whole spread")
    {
        linkModel::Config config;
        config.seed              = 3u;
        config.baseDelayTicks    = 2.0;
        config.jitterShape       = linkModel::JitterShape::TwoPoint;
        config.jitterSpreadTicks = 3.0;
        LinkModel link(config);
        for (int i = 0; i < 100; ++i)
        {
            link.send({ static_cast<std::uint8_t>(i) }, 0.0);
        }
        std::size_t early = 0u;
        link.deliverDue(2.0, [&early](const LinkModel::Payload&) { ++early; });
        std::size_t late = 0u;
        link.deliverDue(4.999, [&late](const LinkModel::Payload&) { ++late; });
        link.deliverDue(5.0, [&late](const LinkModel::Payload&) { ++late; });
        CHECK(early > 0u);
        CHECK(late > 0u);
        CHECK(early + late == 100u);
        CHECK(link.inFlightCount() == 0u);
    }

    SECTION("uniform jitter lands inside [base, base + spread)")
    {
        linkModel::Config config;
        config.seed              = 5u;
        config.baseDelayTicks    = 1.0;
        config.jitterShape       = linkModel::JitterShape::Uniform;
        config.jitterSpreadTicks = 2.0;
        LinkModel link(config);
        for (int i = 0; i < 100; ++i)
        {
            link.send({ 0u }, 0.0);
        }
        CHECK(link.deliverDue(0.999, [](const LinkModel::Payload&) {}) == 0u);
        const std::size_t byTwo = link.deliverDue(2.0, [](const LinkModel::Payload&) {});
        CHECK(byTwo > 0u);
        CHECK(byTwo < 100u);
        CHECK(link.deliverDue(3.0, [](const LinkModel::Payload&) {}) == 100u - byTwo);
    }

    SECTION("a held packet is delivered after the next reorderWindowPackets packets")
    {
        linkModel::Config config = oneTickLink(29u);
        config.reorderProbability   = 0.1;
        config.reorderWindowPackets = 1u;
        LinkModel link(config);
        std::vector<std::uint8_t> order;
        for (std::uint8_t i = 0; i < 10u; ++i)
        {
            link.send({ i }, static_cast<double>(i));
            link.deliverDue(static_cast<double>(i), [&order](const LinkModel::Payload& p) { order.push_back(p[0]); });
        }
        link.deliverDue(100.0, [&order](const LinkModel::Payload& p) { order.push_back(p[0]); });
        CHECK(link.reorderedSends() == std::vector<std::uint64_t>{ 4u });
        CHECK(order == std::vector<std::uint8_t>{ 0u, 1u, 2u, 3u, 5u, 4u, 6u, 7u, 8u, 9u });
    }
}

TEST_CASE("LinkModel pins today (i): a 30 fps client at depth 3 loses exactly one capture tick to one lost bundle",
          "[Network][LinkModel]")
{
    linkModel::Config upConfig = oneTickLink(8u);
    upConfig.lossProbability   = 0.1;
    LinkModel uplink(upConfig);

    Session session;
    session.capturesPerSend = 2u;
    REQUIRE(session.server.config.redundancyDepthTicks == 3);
    session.run(uplink, nullptr);

    CHECK(uplink.sentCount() == 10u);
    CHECK(uplink.lostSends() == std::vector<std::uint64_t>{ 4u });
    CHECK(session.neverReleased() == std::set<std::uint32_t>{ 9u });
    CHECK(session.server.released.size() == 19u);
    CHECK(session.server.pendingCount() == 0u);
    CHECK(session.server.undelayed.captureTicks.empty());
    CHECK(session.server.coordinator.relayOooSkipCount() == 0u);
    CHECK(session.server.coordinator.lateDroppedCount() == 0u);
}

TEST_CASE("LinkModel pins today (ii): a 100 fps client at depth 3 loses no capture tick to one lost bundle",
          "[Network][LinkModel]")
{
    linkModel::Config upConfig = oneTickLink(96u);
    upConfig.lossProbability   = 0.1;
    LinkModel uplink(upConfig);

    Session session;
    session.capturesPerSend = 1u;
    session.run(uplink, nullptr);

    CHECK(uplink.sentCount() == 20u);
    CHECK(uplink.lostSends() == std::vector<std::uint64_t>{ 9u });
    CHECK(session.neverReleased().empty());
    CHECK(session.server.released.size() == 20u);
    CHECK(session.server.pendingCount() == 0u);
    CHECK(session.server.undelayed.captureTicks.empty());
    CHECK(session.server.coordinator.relayOooSkipCount() == 0u);
    CHECK(session.server.coordinator.lateDroppedCount() == 0u);
}

TEST_CASE("LinkModel pins today (iii): a lost relay flush leaves exactly its entries missing from the store",
          "[Network][LinkModel]")
{
    LinkModel uplink(oneTickLink(1u));

    linkModel::Config downConfig = oneTickLink(8u);
    downConfig.lossProbability   = 0.1;
    LinkModel downlink(downConfig);

    Session session;
    session.capturesPerSend = 2u;
    session.run(uplink, &downlink);

    CHECK(uplink.lostSends().empty());
    CHECK(session.flushes == 10u);
    CHECK(downlink.sentCount() == 10u);
    CHECK(downlink.lostSends() == std::vector<std::uint64_t>{ 4u });
    CHECK(session.peer.arrivals == 9u);

    std::set<std::uint32_t> missing;
    for (std::uint32_t tick = 1u; tick <= session.lastCaptureTick; ++tick)
    {
        if (!session.peer.store.has(tick))
        {
            missing.insert(tick);
        }
    }
    CHECK(missing == std::set<std::uint32_t>{ 9u, 10u });
    CHECK(session.peer.store.residentCount() == 18u);

    std::uint8_t dA = 0u;
    PinInput     input;
    REQUIRE(session.peer.store.find(11u, dA, input));
    CHECK(input.captureTick == 11);
    CHECK(static_cast<int>(dA) == 4);
}

TEST_CASE("LinkModel pins today (iv): a reordered bundle is first-writer-wins and its older new tick is a relay OOO skip",
          "[Network][LinkModel]")
{
    linkModel::Config upConfig = oneTickLink(29u);
    upConfig.reorderProbability   = 0.1;
    upConfig.reorderWindowPackets = 1u;
    LinkModel uplink(upConfig);

    Session session;
    session.capturesPerSend     = 2u;
    session.server.stampArrival = true;
    session.run(uplink, nullptr);

    CHECK(uplink.lostSends().empty());
    CHECK(uplink.reorderedSends() == std::vector<std::uint64_t>{ 4u });
    REQUIRE(session.server.arrivalByNewestTick.count(10u) == 1u);
    REQUIRE(session.server.arrivalByNewestTick.count(12u) == 1u);
    CHECK(session.server.arrivalByNewestTick[12u] < session.server.arrivalByNewestTick[10u]);

    CHECK(session.neverReleased().empty());
    CHECK(session.server.released.size() == 20u);
    for (const auto& [captureTick, input] : session.server.released)
    {
        CHECK(input.captureTick == static_cast<std::int32_t>(captureTick));
        CHECK(input.arrival == session.server.firstArrivalCarrying.at(captureTick));
    }
    CHECK(session.server.released.at(10u).arrival == session.server.arrivalByNewestTick[12u]);
    CHECK(session.server.released.at(9u).arrival == session.server.arrivalByNewestTick[10u]);

    CHECK(session.server.coordinator.relayOooSkipCount() == 1u);
    CHECK(session.server.coordinator.lateDroppedCount() == 1u);
    CHECK(std::find(session.server.relay.relayedTicks.begin(), session.server.relay.relayedTicks.end(), 9u)
          == session.server.relay.relayedTicks.end());
    CHECK(session.server.relay.relayedTicks.size() == 19u);
    CHECK(std::is_sorted(session.server.releaseOrder.begin(), session.server.releaseOrder.end()));
    CHECK(session.server.undelayed.captureTicks.empty());
}

#endif // WITH_LOW_LEVEL_TESTS
