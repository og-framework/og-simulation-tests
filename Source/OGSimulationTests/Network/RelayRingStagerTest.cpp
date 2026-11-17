// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/Network/RelayRingStager.h"
#include "OGSimulation/Network/SimPayloadBuffers.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationLogRoutes.h"
#include "OGSimulation/TickDriverLog.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <tuple>
#include <vector>

namespace relayRingStagerTest
{
    struct RrsInput
    {
        std::int32_t move  = 0;
        std::uint8_t flags = 0u;
    };
} // namespace relayRingStagerTest

template <>
struct SerializableFields<relayRingStagerTest::RrsInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(relayRingStagerTest::RrsInput, move),
                               SIM_MEMBER(relayRingStagerTest::RrsInput, flags));
    }
};

namespace relayRingStagerTest
{
    struct RrsGrowRefBuffer
    {
        std::vector<std::uint8_t> bytes;

        std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

        void bundleAddZeroedBytes(std::int32_t count)
        {
            bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u);
        }

        void bundleTruncateTo(std::int32_t byteCount)
        {
            if (byteCount < 0)
                byteCount = 0;
            if (static_cast<std::size_t>(byteCount) < bytes.size())
                bytes.resize(static_cast<std::size_t>(byteCount));
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

    std::vector<std::uint8_t> rrsToU8(std::span<const std::byte> bytes)
    {
        std::vector<std::uint8_t> out;
        out.reserve(bytes.size());
        for (const std::byte b : bytes)
            out.push_back(std::to_integer<std::uint8_t>(b));
        return out;
    }

    std::string rrsExpectedLine(const char* reason, const char* owner, const RelayRingStagerCounters& c)
    {
        const std::uint32_t perRoundX100 = (c.flushedRounds == 0u)
            ? 0u
            : static_cast<std::uint32_t>((static_cast<std::uint64_t>(c.entries) * 100u) / c.flushedRounds);
        char line[512];
        std::snprintf(line, sizeof(line),
            "[Warning][RelayFlush] %s owner=%s flushedRounds=%u entries=%u entriesPerRoundX100=%u "
            "suppressed=%u stageOverflowDrops=%u resent=%u",
            reason, owner,
            static_cast<unsigned>(c.flushedRounds), static_cast<unsigned>(c.entries), static_cast<unsigned>(perRoundX100),
            static_cast<unsigned>(c.suppressed), static_cast<unsigned>(c.stageOverflowDrops), static_cast<unsigned>(c.resent));
        return std::string(line);
    }

    struct RrsActorReference
    {
        std::string               owner;
        RrsGrowRefBuffer          ring;
        RrsGrowRefBuffer          stage;
        RelayRingStagerCounters   counters;
        std::uint32_t             pollsSinceReport = 0u;
        std::vector<std::string>  lines;

        relayedInputRing::StageArrivalOutcome stageArrival(std::uint32_t captureTick, std::uint8_t dA, const RrsInput& input)
        {
            const relayedInputRing::StageArrivalOutcome outcome =
                relayedInputRing::stageArrival<RrsInput>(stage, captureTick, dA, input);
            if (outcome.droppedOldest)
                ++counters.stageOverflowDrops;
            return outcome;
        }

        std::uint8_t flush(std::uint8_t resendEntries)
        {
            const std::uint8_t published = relayedInputRing::flushStagedInto(ring, stage, resendEntries);
            if (published > 0)
            {
                ++counters.flushedRounds;
                counters.entries += published;
                counters.resent += relayedInputRing::entryCount(ring) - published;
            }
            else
            {
                ++counters.suppressed;
            }
            if (++pollsSinceReport >= 120u)
            {
                pollsSinceReport = 0u;
                lines.push_back(rrsExpectedLine("window", owner.c_str(), counters));
            }
            return published;
        }
    };

    RrsInput rrsInputFor(std::uint32_t captureTick)
    {
        return RrsInput{static_cast<std::int32_t>(captureTick * 7u), static_cast<std::uint8_t>(captureTick & 0xFFu)};
    }

    std::uint8_t rrsDAFor(std::uint32_t captureTick)
    {
        return static_cast<std::uint8_t>(3u + (captureTick % 4u));
    }
} // namespace relayRingStagerTest

using namespace relayRingStagerTest;

TEST_CASE("RelayRingStager.ScriptedStagingMatchesTheCodecReferenceAtEveryFlush", "[Network]")
{
    const std::vector<std::uint32_t> pattern = {3u, 0u, 1u, 12u, 0u, 0u, 2u, 8u, 1u, 0u, 5u, 0u, 0u, 0u, 9u, 17u};

    for (const std::uint8_t resend : {std::uint8_t{0u}, std::uint8_t{1u}, std::uint8_t{2u}})
    {
        CAPTURE(resend);
        std::vector<std::string> captured;
        RelayRingStager stager("Character_7", [&captured](const char* line) { captured.emplace_back(line); });
        RrsActorReference reference;
        reference.owner = "Character_7";

        std::uint32_t captureTick = 1000u;
        for (std::uint32_t flushIndex = 0; flushIndex < 250u; ++flushIndex)
        {
            const std::uint32_t size = pattern[flushIndex % pattern.size()];
            CAPTURE(flushIndex, size, captureTick);
            for (std::uint32_t i = 0; i < size; ++i)
            {
                const relayedInputRing::StageArrivalOutcome expected =
                    reference.stageArrival(captureTick, rrsDAFor(captureTick), rrsInputFor(captureTick));
                const relayedInputRing::StageArrivalOutcome actual =
                    stager.stageArrival(captureTick, rrsDAFor(captureTick), rrsInputFor(captureTick));
                CHECK(actual.accepted == expected.accepted);
                CHECK(actual.droppedOldest == expected.droppedOldest);
                ++captureTick;
            }
            if (size > relayedInputRing::kMaxDepth)
            {
                const std::uint32_t oldTick = captureTick - size - 5u;
                const relayedInputRing::StageArrivalOutcome expected =
                    reference.stageArrival(oldTick, rrsDAFor(oldTick), rrsInputFor(oldTick));
                const relayedInputRing::StageArrivalOutcome actual =
                    stager.stageArrival(oldTick, rrsDAFor(oldTick), rrsInputFor(oldTick));
                CHECK_FALSE(expected.accepted);
                CHECK(actual.accepted == expected.accepted);
                CHECK(actual.droppedOldest == expected.droppedOldest);

                const std::uint32_t sameTick = captureTick - 1u;
                const RrsInput rewritten{-5, 0xA5u};
                CHECK(reference.stageArrival(sameTick, 2u, rewritten).accepted);
                CHECK(stager.stageArrival(sameTick, 2u, rewritten).accepted);
            }

            const std::vector<std::uint8_t> before = rrsToU8(stager.published().payload());
            const std::uint8_t expectedCount = reference.flush(resend);
            const std::uint8_t actualCount   = stager.flush(resend);
            CHECK(actualCount == expectedCount);
            CHECK(rrsToU8(stager.published().payload()) == reference.ring.bytes);
            CHECK(stager.counters() == reference.counters);
            CHECK(captured == reference.lines);
            if (actualCount == 0u)
            {
                CHECK(rrsToU8(stager.published().payload()) == before);
            }
        }

        CHECK(reference.counters.stageOverflowDrops > 0u);
        CHECK(reference.counters.suppressed > 0u);
        CHECK(reference.lines.size() == 2u);
        if (resend > 0u)
        {
            CHECK(reference.counters.resent > 0u);
        }
        else
        {
            CHECK(reference.counters.resent == 0u);
        }

        RelayRingBuffer received;
        REQUIRE(received.assignFromWire(stager.published().payload()));
        CHECK(received.num() == relayedInputRing::entryCount(reference.ring));
        CHECK(received.getWireFormatVersionOnWire() == relayedInputRing::kWireFormatVersion);
    }
}

TEST_CASE("RelayRingStager.WindowLineOnFlush120And240AndTeardownFromFinalReport", "[Network]")
{
    std::vector<std::string> captured;
    RelayRingStager stager("OGBrawlerUECharacter_2147482462", [&captured](const char* line) { captured.emplace_back(line); });
    RrsActorReference reference;
    reference.owner = "OGBrawlerUECharacter_2147482462";

    std::uint32_t captureTick = 50u;
    for (std::uint32_t flushNumber = 1u; flushNumber <= 241u; ++flushNumber)
    {
        CAPTURE(flushNumber);
        const std::uint32_t size = (flushNumber % 3u == 0u) ? 0u : (flushNumber % 7u == 0u ? 2u : 1u);
        for (std::uint32_t i = 0; i < size; ++i)
        {
            reference.stageArrival(captureTick, 4u, rrsInputFor(captureTick));
            stager.stageArrival(captureTick, 4u, rrsInputFor(captureTick));
            ++captureTick;
        }
        reference.flush(1u);
        stager.flush(1u);

        if (flushNumber < 120u)
        {
            CHECK(captured.empty());
        }
        else if (flushNumber < 240u)
        {
            CHECK(captured.size() == 1u);
        }
        else
        {
            CHECK(captured.size() == 2u);
        }

        if (flushNumber == 120u || flushNumber == 240u)
        {
            REQUIRE_FALSE(captured.empty());
            CHECK(captured.back() == rrsExpectedLine("window", "OGBrawlerUECharacter_2147482462", stager.counters()));
            CHECK(captured.back().rfind("[Warning][RelayFlush] window owner=OGBrawlerUECharacter_2147482462 flushedRounds=", 0) == 0u);
        }
    }
    CHECK(captured == reference.lines);
    CHECK(stager.counters().flushedRounds + stager.counters().suppressed == 241u);
    CHECK(stager.counters().suppressed == 80u);

    stager.finalReport();
    REQUIRE(captured.size() == 3u);
    CHECK(captured.back() == rrsExpectedLine("teardown", "OGBrawlerUECharacter_2147482462", reference.counters));
    CHECK(captured.back().rfind("[Warning][RelayFlush] teardown owner=OGBrawlerUECharacter_2147482462 ", 0) == 0u);
}

TEST_CASE("RelayRingStager.EmptyFramesCountTowardTheWindow", "[Network]")
{
    SECTION("one published round then 119 empty flushes")
    {
        std::vector<std::string> captured;
        RelayRingStager stager("Character_3", [&captured](const char* line) { captured.emplace_back(line); });
        stager.stageArrival(7u, 4u, rrsInputFor(7u));
        CHECK(stager.flush(0u) == 1u);
        for (std::uint32_t i = 0; i < 119u; ++i)
            CHECK(stager.flush(0u) == 0u);

        REQUIRE(captured.size() == 1u);
        CHECK(captured[0] == "[Warning][RelayFlush] window owner=Character_3 flushedRounds=1 entries=1 entriesPerRoundX100=100 "
                             "suppressed=119 stageOverflowDrops=0 resent=0");
    }
    SECTION("120 empty flushes")
    {
        std::vector<std::string> captured;
        RelayRingStager stager("Character_4", [&captured](const char* line) { captured.emplace_back(line); });
        for (std::uint32_t i = 0; i < 120u; ++i)
            CHECK(stager.flush(2u) == 0u);

        REQUIRE(captured.size() == 1u);
        CHECK(captured[0] == "[Warning][RelayFlush] window owner=Character_4 flushedRounds=0 entries=0 entriesPerRoundX100=0 "
                             "suppressed=120 stageOverflowDrops=0 resent=0");
        CHECK(stager.published().payload().empty());
    }
}

TEST_CASE("RelayRingStager.StageOverflowIsCountedAndTheWindowKeepsSessionTotals", "[Network]")
{
    std::vector<std::string> captured;
    RelayRingStager stager("Character_9", [&captured](const char* line) { captured.emplace_back(line); });

    std::uint32_t dropped = 0u;
    for (std::uint32_t tick = 1u; tick <= 12u; ++tick)
    {
        if (stager.stageArrival(tick, 4u, rrsInputFor(tick)).droppedOldest)
            ++dropped;
    }
    CHECK(dropped == 4u);
    CHECK(stager.counters().stageOverflowDrops == 4u);
    CHECK(stager.flush(0u) == relayedInputRing::kMaxDepth);
    for (std::uint32_t i = 1u; i < 240u; ++i)
        stager.flush(0u);

    REQUIRE(captured.size() == 2u);
    CHECK(captured[0] == "[Warning][RelayFlush] window owner=Character_9 flushedRounds=1 entries=8 entriesPerRoundX100=800 "
                         "suppressed=119 stageOverflowDrops=4 resent=0");
    CHECK(captured[1] == "[Warning][RelayFlush] window owner=Character_9 flushedRounds=1 entries=8 entriesPerRoundX100=800 "
                         "suppressed=239 stageOverflowDrops=4 resent=0");
}

TEST_CASE("RelayRingStager.RouteTableResolvesRelayFlushToNetAtWarning", "[Network]")
{
    std::vector<std::string> captured;
    RelayRingStager stager("Character_1", [&captured](const char* line) { captured.emplace_back(line); });
    stager.finalReport();
    REQUIRE(captured.size() == 1u);

    const std::string line = captured[0];
    const std::string token = "[Warning]";
    REQUIRE(line.rfind(token, 0) == 0u);
    const std::string body = line.substr(token.size());
    CHECK(body == "[RelayFlush] teardown owner=Character_1 flushedRounds=0 entries=0 entriesPerRoundX100=0 "
                  "suppressed=0 stageOverflowDrops=0 resent=0");

    const SimulationLogRouteEntry* match = nullptr;
    for (const SimulationLogRouteEntry& entry : kSimulationLogRoutes)
    {
        if (simulationLogRoutes::startsWithIgnoringCase(body.c_str(), entry.prefix))
        {
            match = &entry;
            break;
        }
    }
    REQUIRE(match != nullptr);
    CHECK(std::string(match->prefix) == "[RelayFlush]");
    CHECK(match->route == SimulationLogRoute::Net);
    CHECK(match->plainLevel == HostLogLevel::Log);
}

TEST_CASE("RelayRingStager.WithoutALoggerFlushesAndReportsNothing", "[Network]")
{
    RelayRingStager stager("Character_2");
    CHECK(stager.debugName() == "Character_2");
    stager.stageArrival(10u, 4u, rrsInputFor(10u));
    for (std::uint32_t i = 0; i < 240u; ++i)
        stager.flush(0u);
    stager.finalReport();
    CHECK(stager.counters().flushedRounds == 1u);
    CHECK(stager.counters().suppressed == 239u);
    CHECK(stager.published().num() == 1u);
}

#endif // WITH_LOW_LEVEL_TESTS
