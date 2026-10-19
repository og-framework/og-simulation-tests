// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/InputResolutionTelemetry.h"
#include "OGSimulation/NetSyncTelemetry.h"
#include "OGSimulation/Network/RelayReadProbe.h"
#include "OGSimulation/Network/RemoteInputCache.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationNetSync.h"

#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace
{
    struct TickLineInput
    {
        std::int32_t forward = 0;
        std::int32_t tag     = 0;

        bool operator==(const TickLineInput& other) const
        {
            return forward == other.forward && tag == other.tag;
        }
    };
} // namespace

template <>
struct SerializableFields<TickLineInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(TickLineInput, forward),
                               SIM_MEMBER(TickLineInput, tag));
    }
};

namespace
{
    TickLineInput tickLineZero()
    {
        TickLineInput zero;
        zero.forward = 1;
        return zero;
    }

    TickLineInput tickLineTagged(std::int32_t tag)
    {
        TickLineInput input = tickLineZero();
        input.tag = tag;
        return input;
    }

    struct TickLineRing
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

    TickLineRing makeTickLineRing(const std::vector<std::uint32_t>& captureTicks,
                                  std::uint8_t dA, std::int32_t depth)
    {
        TickLineRing ring;
        for (std::uint32_t tick : captureTicks)
        {
            relayedInputRing::writeLatest<TickLineInput>(
                ring, tick, dA, tickLineTagged(static_cast<std::int32_t>(tick)), depth);
        }
        return ring;
    }

    struct VerboseGatedSink
    {
        bool verboseGateOpen = true;
        std::vector<std::string> lines;

        std::function<void(const char*)> logger()
        {
            return [this](const char* message)
            {
                if (!verboseGateOpen && std::strncmp(message, "[Verbose]", 9) == 0)
                {
                    return;
                }
                lines.emplace_back(message);
            };
        }

        std::size_t countContaining(const char* needle) const
        {
            std::size_t count = 0u;
            for (const std::string& line : lines)
            {
                count += (line.find(needle) != std::string::npos) ? 1u : 0u;
            }
            return count;
        }

        std::vector<std::string> linesContaining(const char* needle) const
        {
            std::vector<std::string> found;
            for (const std::string& line : lines)
            {
                if (line.find(needle) != std::string::npos)
                {
                    found.push_back(line);
                }
            }
            return found;
        }
    };

    RemoteInputCache<TickLineInput> makeStoreHolding100And104()
    {
        RemoteInputCache<TickLineInput> store(tickLineZero());
        store.push(100u, 4u, tickLineTagged(100));
        store.push(104u, 4u, tickLineTagged(104));
        return store;
    }

    ScheduledRelayedReadReport readReportAt(const RemoteInputCache<TickLineInput>& store,
                                            std::uint32_t tick, std::int32_t& outServedTag)
    {
        ScheduledRelayedReadReport report;
        outServedTag = resolveScheduledRelayedInput(store, tick, &report).tag;
        return report;
    }
} // namespace

TEST_CASE("RelayProbeTickLines: a hit names the probe tick as the served capture",
          "[Network][RelayProbeTickLines]")
{
    const RemoteInputCache<TickLineInput> store = makeStoreHolding100And104();
    std::int32_t servedTag = 0;
    const ScheduledRelayedReadReport report = readReportAt(store, 104u, servedTag);
    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Hit);
    REQUIRE(servedTag == 100);

    VerboseGatedSink sink;
    InputResolutionTelemetry telemetry;
    telemetry.setLogger(sink.logger());
    telemetry.emitPredictionInputRead(7u, 104u, true, report);

    const std::vector<std::string> readTicks = sink.linesContaining("[RelayProbe.ReadTick]");
    REQUIRE(readTicks.size() == 1u);
    CHECK(readTicks[0] ==
        "[Verbose][RelayProbe.ReadTick] id=7 tick=104 src=Prediction outcome=Hit miss=- "
        "probe=100 served=100 newest=104");
    CHECK(sink.countContaining("[CollectInput] id=7 tick=104 source=RemoteInputCache hasStore=1") == 1u);
}

TEST_CASE("RelayProbeTickLines: an aboveNewest miss names the newest arrival as the served capture",
          "[Network][RelayProbeTickLines]")
{
    const RemoteInputCache<TickLineInput> store = makeStoreHolding100And104();
    std::int32_t servedTag = 0;
    const ScheduledRelayedReadReport report = readReportAt(store, 110u, servedTag);
    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(report.missClass == ScheduledRelayedReadMissClass::AboveNewest);
    REQUIRE(servedTag == 104);

    VerboseGatedSink sink;
    InputResolutionTelemetry telemetry;
    telemetry.setLogger(sink.logger());
    telemetry.emitPredictionInputRead(7u, 110u, true, report);
    telemetry.emitResimScheduledRead(7u, 110u, report);

    const std::vector<std::string> readTicks = sink.linesContaining("[RelayProbe.ReadTick]");
    REQUIRE(readTicks.size() == 2u);
    CHECK(readTicks[0] ==
        "[Verbose][RelayProbe.ReadTick] id=7 tick=110 src=Prediction outcome=Miss "
        "miss=AboveNewest probe=106 served=104 newest=104");
    CHECK(readTicks[1] ==
        "[Verbose][RelayProbe.ReadTick] id=7 tick=110 src=Resim outcome=Miss "
        "miss=AboveNewest probe=106 served=104 newest=104");
}

TEST_CASE("RelayProbeTickLines: an inSpan miss names the hole as the probe and the newest as served",
          "[Network][RelayProbeTickLines]")
{
    const RemoteInputCache<TickLineInput> store = makeStoreHolding100And104();
    std::int32_t servedTag = 0;
    const ScheduledRelayedReadReport report = readReportAt(store, 106u, servedTag);
    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(report.missClass == ScheduledRelayedReadMissClass::InSpan);
    REQUIRE(servedTag == 104);

    VerboseGatedSink sink;
    InputResolutionTelemetry telemetry;
    telemetry.setLogger(sink.logger());
    telemetry.emitPredictionInputRead(9u, 106u, true, report);

    const std::vector<std::string> readTicks = sink.linesContaining("[RelayProbe.ReadTick]");
    REQUIRE(readTicks.size() == 1u);
    CHECK(readTicks[0] ==
        "[Verbose][RelayProbe.ReadTick] id=9 tick=106 src=Prediction outcome=Miss "
        "miss=InSpan probe=102 served=104 newest=104");
}

TEST_CASE("RelayProbeTickLines: a read that formed no probe tick prints dashes, not zeros",
          "[Network][RelayProbeTickLines]")
{
    const RemoteInputCache<TickLineInput> empty(tickLineZero());
    std::int32_t servedTag = 0;
    const ScheduledRelayedReadReport rungZero = readReportAt(empty, 500u, servedTag);
    REQUIRE(rungZero.outcome == ScheduledRelayedReadOutcome::NoProbe);

    RemoteInputCache<TickLineInput> young(tickLineZero());
    young.push(1u, 8u, tickLineTagged(1));
    const ScheduledRelayedReadReport underflow = readReportAt(young, 3u, servedTag);
    REQUIRE(underflow.outcome == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(underflow.missClass == ScheduledRelayedReadMissClass::NoProbeTick);

    VerboseGatedSink sink;
    InputResolutionTelemetry telemetry;
    telemetry.setLogger(sink.logger());
    telemetry.emitPredictionInputRead(4u, 500u, true, rungZero);
    telemetry.emitResimScheduledRead(4u, 3u, underflow);

    const std::vector<std::string> readTicks = sink.linesContaining("[RelayProbe.ReadTick]");
    REQUIRE(readTicks.size() == 2u);
    CHECK(readTicks[0] ==
        "[Verbose][RelayProbe.ReadTick] id=4 tick=500 src=Prediction outcome=NoProbe miss=- "
        "probe=- served=- newest=-");
    CHECK(readTicks[1] ==
        "[Verbose][RelayProbe.ReadTick] id=4 tick=3 src=Resim outcome=Miss miss=NoProbeTick "
        "probe=- served=- newest=-");
}

TEST_CASE("RelayProbeTickLines: a prediction read without a store prints no ReadTick line",
          "[Network][RelayProbeTickLines]")
{
    VerboseGatedSink sink;
    InputResolutionTelemetry telemetry;
    telemetry.setLogger(sink.logger());
    telemetry.emitPredictionInputRead(7u, 104u, false, ScheduledRelayedReadReport{});

    CHECK(sink.countContaining("[RelayProbe.ReadTick]") == 0u);
    CHECK(sink.countContaining("[CollectInput] id=7 tick=104 source=RemoteInputCache hasStore=0") == 1u);
}

TEST_CASE("RelayProbeTickLines: a 3-capture arrival prints one Ingest line, a re-delivery none",
          "[Network][RelayProbeTickLines]")
{
    RemoteInputCache<TickLineInput> store(tickLineZero());
    const TickLineRing ring = makeTickLineRing({ 100u, 101u, 102u }, 4u, 3);

    const RelayedInputIngestReport first = populateRemoteInputCache(store, ring);
    REQUIRE(first.newCaptureTicksIngested == 3u);
    REQUIRE(first.newestCaptureTickValid);
    REQUIRE(first.newestCaptureTick == 102u);

    VerboseGatedSink sink;
    NetSyncTelemetry telemetry;
    telemetry.setLogger(sink.logger());
    telemetry.emitRelayArrival(5u, first);

    const std::vector<std::string> ingests = sink.linesContaining("[RelayProbe.Ingest]");
    REQUIRE(ingests.size() == 1u);
    CHECK(ingests[0] == "[Verbose][RelayProbe.Ingest] id=5 newest=102 new=3");

    const RelayedInputIngestReport again = populateRemoteInputCache(store, ring);
    REQUIRE(again.newCaptureTicksIngested == 0u);
    REQUIRE(again.newestCaptureTickValid);
    telemetry.emitRelayArrival(5u, again);

    CHECK(sink.countContaining("[RelayProbe.Ingest]") == 1u);
}

TEST_CASE("RelayProbeTickLines: neither line reaches a logger whose Verbose gate is closed",
          "[Network][RelayProbeTickLines]")
{
    const RemoteInputCache<TickLineInput> readStore = makeStoreHolding100And104();
    std::int32_t servedTag = 0;
    const ScheduledRelayedReadReport hit = readReportAt(readStore, 104u, servedTag);
    const ScheduledRelayedReadReport miss = readReportAt(readStore, 110u, servedTag);

    for (const bool gateOpen : { true, false })
    {
        INFO("verboseGateOpen=" << gateOpen);

        VerboseGatedSink sink;
        sink.verboseGateOpen = gateOpen;

        InputResolutionTelemetry readTelemetry;
        readTelemetry.setLogger(sink.logger());
        readTelemetry.emitPredictionInputRead(7u, 104u, true, hit);
        readTelemetry.emitPredictionInputRead(7u, 110u, true, miss);
        readTelemetry.emitResimScheduledRead(7u, 110u, miss);

        RemoteInputCache<TickLineInput> arrivalStore(tickLineZero());
        NetSyncTelemetry arrivalTelemetry;
        arrivalTelemetry.setLogger(sink.logger());
        arrivalTelemetry.emitRelayArrival(
            5u, populateRemoteInputCache(arrivalStore, makeTickLineRing({ 100u, 101u, 102u }, 4u, 3)));
        arrivalTelemetry.emitRelayArrival(
            5u, populateRemoteInputCache(arrivalStore, makeTickLineRing({ 103u }, 4u, 1)));

        CHECK(sink.countContaining("[CollectInput]") == 2u);
        CHECK(sink.countContaining("[RelayProbe.ReadTick]") == (gateOpen ? 3u : 0u));
        CHECK(sink.countContaining("[RelayProbe.Ingest]") == (gateOpen ? 2u : 0u));
    }
}

#endif // WITH_LOW_LEVEL_TESTS
