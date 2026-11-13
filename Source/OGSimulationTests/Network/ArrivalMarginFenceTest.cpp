// SPDX-License-Identifier: MPL-2.0
// docs/ArrivalMarginFenceTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <tuple>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/Network/ServerReceptionCoordinator.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "LinkModel.h"
#include "StandaloneTestHandle.h"

namespace arrivalMarginFence
{
    struct FenceInput
    {
        std::int32_t captureTick = 0;
        std::int32_t owner       = 0;
    };
}

template <>
struct SerializableFields<arrivalMarginFence::FenceInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(arrivalMarginFence::FenceInput, captureTick),
                               SIM_MEMBER(arrivalMarginFence::FenceInput, owner));
    }
};

namespace arrivalMarginFence
{
    using linkModel::LinkModel;

    struct FenceSim
    {
        using Input = FenceInput;
    };

    using FenceCoordinator = ServerReceptionCoordinator<FStandaloneTestHandle, FenceSim>;

    constexpr std::int32_t kScribbleTick   = 600;
    constexpr std::int32_t kLastTick       = 1200;
    constexpr std::uint32_t kScribbleSeed  = 0xA11CEu;
    constexpr std::int32_t kDelayTicks     = 6;

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

    using Release = std::tuple<unsigned int, std::uint32_t, std::int32_t, std::int32_t>;
    using Relay   = std::tuple<unsigned int, std::uint32_t, std::uint8_t, std::int32_t>;

    struct Outputs
    {
        std::vector<Release>     releases;
        std::vector<Relay>       relays;
        std::vector<Release>     undelayed;
        std::vector<std::string> parkAndReleaseLines;
        std::vector<std::string> arrivalLines;
        std::vector<std::size_t> lateDroppedPerFrame;
        std::vector<std::string> backDateLines;
        std::uint32_t            backDateEpisodes = 0u;
    };

    struct RecordingRelay
    {
        Outputs* outputs = nullptr;

        void relayRemoteInput(unsigned int id, std::uint32_t captureTick, std::uint8_t dA, const FenceInput& input)
        {
            outputs->relays.emplace_back(id, captureTick, dA, input.captureTick);
        }
    };

    struct RecordingUndelayed
    {
        Outputs* outputs = nullptr;

        void deliverRemoteInput(unsigned int id, std::uint32_t captureTick, const FenceInput& input)
        {
            outputs->undelayed.emplace_back(id, captureTick, input.captureTick, input.owner);
        }
    };

    struct Sender
    {
        unsigned int  id   = 0u;
        std::uint8_t  slot = 0u;
        std::int32_t  lead = 0;
        linkModel::Config link;
    };

    std::vector<Sender> senders()
    {
        Sender first;
        first.id                     = 7u;
        first.slot                   = 0u;
        first.lead                   = -2;
        first.link.seed              = 61u;
        first.link.baseDelayTicks    = 3.0;
        first.link.jitterShape       = linkModel::JitterShape::Uniform;
        first.link.jitterSpreadTicks = 4.0;
        first.link.lossProbability   = 0.1;

        Sender second = first;
        second.id                = 8u;
        second.slot              = 1u;
        second.lead              = -1;
        second.link.seed         = 62u;
        second.link.reorderProbability   = 0.1;
        second.link.reorderWindowPackets = 2u;
        return { first, second };
    }

    Outputs runSession(bool scribble)
    {
        constexpr std::int32_t kRedundancyDepth = 3;

        TimeConfig config;
        config.relayDelayFloorTicks = kDelayTicks;
        FenceCoordinator coordinator(config);
        const FStandaloneTestHandle wire(1u, true);

        Outputs outputs;
        coordinator.setLogger([&outputs](const char* line)
        {
            const std::string text(line);
            if (text.rfind("[Park]", 0) == 0 || text.rfind("[Release]", 0) == 0)
            {
                outputs.parkAndReleaseLines.push_back(text);
            }
            else if (text.find("[InputArrival.Window]") != std::string::npos)
            {
                outputs.arrivalLines.push_back(text);
            }
            else if (text.find("[InputArrival.BackDate]") != std::string::npos)
            {
                outputs.backDateLines.push_back(text);
            }
        });

        RecordingRelay     relay{ &outputs };
        RecordingUndelayed undelayed{ &outputs };

        const std::vector<Sender> plan = senders();
        std::vector<LinkModel> uplinks;
        for (const Sender& sender : plan)
        {
            uplinks.emplace_back(sender.link);
        }

        for (std::int32_t t = 0; t <= kLastTick; ++t)
        {
            if (scribble && t == kScribbleTick)
            {
                coordinator.editArrivalMarginForFenceTest().scribbleForFenceTest(kScribbleSeed);
            }
            for (std::size_t i = 0; i < plan.size(); ++i)
            {
                const std::int32_t newest = t + plan[i].lead;
                WireBuffer bundle;
                for (std::int32_t c = newest - kRedundancyDepth + 1; c <= newest; ++c)
                {
                    if (c >= 1)
                    {
                        inputRedundancyBundle::appendSlot<FenceInput>(bundle, static_cast<std::uint32_t>(c),
                            FenceInput{ c, static_cast<std::int32_t>(plan[i].id) });
                    }
                }
                if (bundle.bundleByteNum() > 0)
                {
                    uplinks[i].send(bundle.bytes, static_cast<double>(t));
                }
            }
            for (std::size_t i = 0; i < plan.size(); ++i)
            {
                const Sender& sender = plan[i];
                uplinks[i].deliverDue(static_cast<double>(t),
                    [&](const LinkModel::Payload& payload)
                    {
                        const WireBuffer received(payload);
                        coordinator.receiveInputBundle<FenceSim>(sender.id, wire, sender.slot, received, undelayed,
                                                                 relay);
                    });
            }
            coordinator.releaseDelayedInputs<FenceSim>(t, 1,
                [&outputs](unsigned int id, std::uint32_t captureTick, const FenceInput& input)
                {
                    outputs.releases.emplace_back(id, captureTick, input.captureTick, input.owner);
                    return true;
                });
            coordinator.reapConnections(t);
            outputs.lateDroppedPerFrame.push_back(coordinator.lateDroppedCount());
        }
        if (const auto signal = coordinator.arrivalMargin().signal(wire); signal.has_value())
        {
            outputs.backDateEpisodes = signal->backDateEpisodes;
        }
        return outputs;
    }

TEST_CASE("ArrivalMarginFence: scribbling the arrival margin and its back-dating episode state mid-session changes "
          "no coordinator output", "[Network][InputArrivalMargin][ArrivalMarginFence]")
{
    const Outputs clean     = runSession(false);
    const Outputs scribbled = runSession(true);

    CAPTURE(clean.releases.size(), clean.relays.size(), clean.parkAndReleaseLines.size(),
            clean.lateDroppedPerFrame.back(), clean.arrivalLines.size(), clean.backDateLines.size(),
            scribbled.backDateLines.size(), clean.backDateEpisodes, scribbled.backDateEpisodes);
    REQUIRE(clean.releases.size() > 500u);
    REQUIRE(clean.relays.size() > 500u);
    REQUIRE(clean.lateDroppedPerFrame.back() > 100u);
    REQUIRE(clean.lateDroppedPerFrame[static_cast<std::size_t>(kScribbleTick)] < clean.lateDroppedPerFrame.back());
    REQUIRE(clean.arrivalLines.size() == static_cast<std::size_t>(kLastTick / (2 * 60)));
    REQUIRE(clean.arrivalLines != scribbled.arrivalLines);
    REQUIRE(clean.backDateEpisodes != scribbled.backDateEpisodes);
    REQUIRE(clean.backDateLines != scribbled.backDateLines);

    CHECK(scribbled.releases == clean.releases);
    CHECK(scribbled.relays == clean.relays);
    CHECK(scribbled.undelayed == clean.undelayed);
    CHECK(scribbled.parkAndReleaseLines == clean.parkAndReleaseLines);
    CHECK(scribbled.lateDroppedPerFrame == clean.lateDroppedPerFrame);
}
} // namespace arrivalMarginFence

#endif // WITH_LOW_LEVEL_TESTS
