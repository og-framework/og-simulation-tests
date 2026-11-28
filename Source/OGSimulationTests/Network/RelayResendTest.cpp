// SPDX-License-Identifier: MPL-2.0
// docs/RelayResendTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <set>
#include <tuple>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/RemoteInputCache.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "LinkModel.h"

namespace relayResend
{
    struct ResendInput
    {
        std::int32_t captureTick = 0;
        std::int32_t value       = 0;
    };
}

template <>
struct SerializableFields<relayResend::ResendInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(relayResend::ResendInput, captureTick),
                               SIM_MEMBER(relayResend::ResendInput, value));
    }
};

static_assert(relayedInputRing::kMaxResendEntries == latencyKnobs::kMaxRelayResendEntries,
    "the codec's re-send bound and the TimeConfig knob's clamp must agree, or a session value the knob accepts would "
    "be silently cut by the flush. og-input-latency task 8");

namespace relayResend
{
    using linkModel::LinkModel;

    constexpr std::uint8_t kStampDA = 4u;

    struct ResendBuffer
    {
        std::vector<std::uint8_t> bytes;

        ResendBuffer() = default;
        explicit ResendBuffer(const std::vector<std::uint8_t>& payload)
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

    ResendInput inputFor(std::uint32_t tick)
    {
        return ResendInput{ static_cast<std::int32_t>(tick), static_cast<std::int32_t>(tick * 7u) };
    }

    void stage(ResendBuffer& stageBuffer, std::uint32_t tick, std::uint8_t dA = kStampDA)
    {
        (void)relayedInputRing::stageArrival<ResendInput>(stageBuffer, tick, dA, inputFor(tick));
    }

    std::vector<std::uint32_t> ticksInRingOrder(const ResendBuffer& ring)
    {
        std::vector<std::uint32_t> ticks;
        relayedInputRing::forEachEntry<ResendInput>(ring,
            [&ticks](std::uint32_t captureTick, std::uint8_t, const ResendInput&) { ticks.push_back(captureTick); });
        return ticks;
    }

    struct PinRound
    {
        std::uint32_t staged = 0u;
        std::uint8_t  dA     = 0u;
    };

    constexpr PinRound kPinRounds[] = {
        { 1u, 4u }, { 2u, 4u }, { 0u, 0u }, { 3u, 5u }, { 0u, 0u }, { 0u, 0u },
        { 9u, 5u }, { 1u, 6u }, { 2u, 6u }, { 0u, 0u }, { 1u, 4u },
    };

    struct PinTrace
    {
        std::vector<std::uint8_t> ringBytes;
        std::vector<std::uint8_t> published;
    };

    enum class PinCall
    {
        TwoArguments,
        ExplicitZero,
        NegativeClampsToZero,
    };

    PinTrace runPinRounds(PinCall call)
    {
        PinTrace      trace;
        ResendBuffer  stageBuffer;
        ResendBuffer  ring;
        std::uint32_t nextTick = 100u;
        for (const PinRound& round : kPinRounds)
        {
            for (std::uint32_t i = 0u; i < round.staged; ++i)
            {
                stage(stageBuffer, nextTick++, round.dA);
            }
            std::uint8_t published = 0u;
            switch (call)
            {
            case PinCall::TwoArguments:
                published = relayedInputRing::flushStagedInto(ring, stageBuffer);
                break;
            case PinCall::ExplicitZero:
                published = relayedInputRing::flushStagedInto(ring, stageBuffer, 0);
                break;
            case PinCall::NegativeClampsToZero:
                published = relayedInputRing::flushStagedInto(ring, stageBuffer, -5);
                break;
            }
            trace.published.push_back(published);
            trace.ringBytes.insert(trace.ringBytes.end(), ring.bytes.begin(), ring.bytes.end());
            trace.ringBytes.push_back(0xEEu);
        }
        return trace;
    }

    const std::vector<std::uint8_t> kPinPublishedBeforeTask8 = { 1, 2, 0, 3, 0, 0, 8, 1, 2, 0, 1 };
    const std::vector<std::uint8_t> kPinBytesBeforeTask8 = {
        0x03, 0x01, 0x64, 0x00, 0x00, 0x00, 0x04, 0x64, 0x00, 0x00, 0x00, 0xBC, 0x02, 0x00, 0x00, 0xEE,
        0x03, 0x02, 0x65, 0x00, 0x00, 0x00, 0x04, 0x65, 0x00, 0x00, 0x00, 0xC3, 0x02, 0x00, 0x00, 0x66,
        0x00, 0x00, 0x00, 0x04, 0x66, 0x00, 0x00, 0x00, 0xCA, 0x02, 0x00, 0x00, 0xEE, 0x03, 0x02, 0x65,
        0x00, 0x00, 0x00, 0x04, 0x65, 0x00, 0x00, 0x00, 0xC3, 0x02, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00,
        0x04, 0x66, 0x00, 0x00, 0x00, 0xCA, 0x02, 0x00, 0x00, 0xEE, 0x03, 0x03, 0x67, 0x00, 0x00, 0x00,
        0x05, 0x67, 0x00, 0x00, 0x00, 0xD1, 0x02, 0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x05, 0x68, 0x00,
        0x00, 0x00, 0xD8, 0x02, 0x00, 0x00, 0x69, 0x00, 0x00, 0x00, 0x05, 0x69, 0x00, 0x00, 0x00, 0xDF,
        0x02, 0x00, 0x00, 0xEE, 0x03, 0x03, 0x67, 0x00, 0x00, 0x00, 0x05, 0x67, 0x00, 0x00, 0x00, 0xD1,
        0x02, 0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x05, 0x68, 0x00, 0x00, 0x00, 0xD8, 0x02, 0x00, 0x00,
        0x69, 0x00, 0x00, 0x00, 0x05, 0x69, 0x00, 0x00, 0x00, 0xDF, 0x02, 0x00, 0x00, 0xEE, 0x03, 0x03,
        0x67, 0x00, 0x00, 0x00, 0x05, 0x67, 0x00, 0x00, 0x00, 0xD1, 0x02, 0x00, 0x00, 0x68, 0x00, 0x00,
        0x00, 0x05, 0x68, 0x00, 0x00, 0x00, 0xD8, 0x02, 0x00, 0x00, 0x69, 0x00, 0x00, 0x00, 0x05, 0x69,
        0x00, 0x00, 0x00, 0xDF, 0x02, 0x00, 0x00, 0xEE, 0x03, 0x08, 0x72, 0x00, 0x00, 0x00, 0x05, 0x72,
        0x00, 0x00, 0x00, 0x1E, 0x03, 0x00, 0x00, 0x6B, 0x00, 0x00, 0x00, 0x05, 0x6B, 0x00, 0x00, 0x00,
        0xED, 0x02, 0x00, 0x00, 0x6C, 0x00, 0x00, 0x00, 0x05, 0x6C, 0x00, 0x00, 0x00, 0xF4, 0x02, 0x00,
        0x00, 0x6D, 0x00, 0x00, 0x00, 0x05, 0x6D, 0x00, 0x00, 0x00, 0xFB, 0x02, 0x00, 0x00, 0x6E, 0x00,
        0x00, 0x00, 0x05, 0x6E, 0x00, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x05,
        0x6F, 0x00, 0x00, 0x00, 0x09, 0x03, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x05, 0x70, 0x00, 0x00,
        0x00, 0x10, 0x03, 0x00, 0x00, 0x71, 0x00, 0x00, 0x00, 0x05, 0x71, 0x00, 0x00, 0x00, 0x17, 0x03,
        0x00, 0x00, 0xEE, 0x03, 0x01, 0x73, 0x00, 0x00, 0x00, 0x06, 0x73, 0x00, 0x00, 0x00, 0x25, 0x03,
        0x00, 0x00, 0xEE, 0x03, 0x02, 0x74, 0x00, 0x00, 0x00, 0x06, 0x74, 0x00, 0x00, 0x00, 0x2C, 0x03,
        0x00, 0x00, 0x75, 0x00, 0x00, 0x00, 0x06, 0x75, 0x00, 0x00, 0x00, 0x33, 0x03, 0x00, 0x00, 0xEE,
        0x03, 0x02, 0x74, 0x00, 0x00, 0x00, 0x06, 0x74, 0x00, 0x00, 0x00, 0x2C, 0x03, 0x00, 0x00, 0x75,
        0x00, 0x00, 0x00, 0x06, 0x75, 0x00, 0x00, 0x00, 0x33, 0x03, 0x00, 0x00, 0xEE, 0x03, 0x01, 0x76,
        0x00, 0x00, 0x00, 0x04, 0x76, 0x00, 0x00, 0x00, 0x3A, 0x03, 0x00, 0x00, 0xEE,
    };

    struct DownlinkRun
    {
        std::int32_t  resendEntries    = 0;
        std::uint32_t capturesPerFlush = 1u;
        std::uint32_t lastCaptureTick  = 20u;
        std::int64_t  droppedFlush     = -1;
        bool          deliverTwice     = false;
    };

    struct DownlinkResult
    {
        RemoteInputCache<ResendInput>      store;
        std::size_t                        flushes = 0u;
        std::vector<std::vector<std::uint32_t>> flushTicks;
        std::set<std::uint32_t>            missing;
        std::uint8_t                       maxEntries = 0u;
    };

    DownlinkResult runDownlink(LinkModel& link, const DownlinkRun& run)
    {
        DownlinkResult result;
        ResendBuffer   stageBuffer;
        ResendBuffer   ring;
        std::uint32_t  nextTick = 1u;

        const auto ingest = [&result, &run](const LinkModel::Payload& payload)
        {
            const ResendBuffer replicated(payload);
            (void)populateRemoteInputCache<ResendInput>(result.store, replicated);
            if (run.deliverTwice)
            {
                (void)populateRemoteInputCache<ResendInput>(result.store, replicated);
            }
        };

        const std::uint32_t frames = run.lastCaptureTick / run.capturesPerFlush + 4u;
        for (std::uint32_t frame = 1u; frame <= frames; ++frame)
        {
            std::vector<std::uint32_t> staged;
            for (std::uint32_t i = 0u; i < run.capturesPerFlush && nextTick <= run.lastCaptureTick; ++i)
            {
                staged.push_back(nextTick);
                stage(stageBuffer, nextTick++);
            }

            if (relayedInputRing::flushStagedInto(ring, stageBuffer, run.resendEntries) > 0u)
            {
                const std::int64_t flushIndex = static_cast<std::int64_t>(result.flushes++);
                result.flushTicks.push_back(staged);
                if (relayedInputRing::entryCount(ring) > result.maxEntries)
                {
                    result.maxEntries = relayedInputRing::entryCount(ring);
                }
                if (flushIndex != run.droppedFlush)
                {
                    link.send(ring.bytes, static_cast<double>(frame));
                }
            }
            link.deliverDue(static_cast<double>(frame), ingest);
        }
        link.deliverDue(static_cast<double>(frames + 100u), ingest);

        for (std::uint32_t tick = 1u; tick <= run.lastCaptureTick; ++tick)
        {
            if (!result.store.has(tick))
            {
                result.missing.insert(tick);
            }
        }
        return result;
    }

    linkModel::Config oneTickLink(std::uint64_t seed)
    {
        linkModel::Config config;
        config.seed           = seed;
        config.baseDelayTicks = 1.0;
        return config;
    }

    constexpr std::int32_t kRecoveryResendEntries = 1;
}

using namespace relayResend;

TEST_CASE("RelayResend: at resendEntries = 0 the flush publishes exactly the bytes it published before task 8",
          "[Network][RelayResend]")
{
    REQUIRE_FALSE(kPinBytesBeforeTask8.empty());

    const PinTrace twoArguments = runPinRounds(PinCall::TwoArguments);
    CHECK(twoArguments.published == kPinPublishedBeforeTask8);
    CHECK(twoArguments.ringBytes == kPinBytesBeforeTask8);

    const PinTrace explicitZero = runPinRounds(PinCall::ExplicitZero);
    CHECK(explicitZero.published == kPinPublishedBeforeTask8);
    CHECK(explicitZero.ringBytes == kPinBytesBeforeTask8);

    const PinTrace negative = runPinRounds(PinCall::NegativeClampsToZero);
    CHECK(negative.published == kPinPublishedBeforeTask8);
    CHECK(negative.ringBytes == kPinBytesBeforeTask8);
}

TEST_CASE("RelayResend: r = 1 recovers any single lost flush of one entry",
          "[Network][RelayResend]")
{
    DownlinkRun run;
    run.resendEntries    = kRecoveryResendEntries;
    run.capturesPerFlush = 1u;

    LinkModel     probeLink(oneTickLink(1u));
    const std::size_t flushes = runDownlink(probeLink, run).flushes;
    REQUIRE(flushes == 20u);

    for (std::int64_t lost = 0; lost + 1 < static_cast<std::int64_t>(flushes); ++lost)
    {
        CAPTURE(lost);
        run.droppedFlush = lost;
        LinkModel            link(oneTickLink(1u));
        const DownlinkResult result = runDownlink(link, run);
        CHECK(result.missing.empty());
        CHECK(result.store.residentCount() == 20u);
    }
}

TEST_CASE("RelayResend: at r = 0 a lost flush leaves exactly its entries missing, as task 3 (iii) pins",
          "[Network][RelayResend]")
{
    for (const std::uint32_t perFlush : { 1u, 2u })
    {
        DownlinkRun run;
        run.resendEntries    = 0;
        run.capturesPerFlush = perFlush;
        for (std::int64_t lost = 0; lost < 6; ++lost)
        {
            CAPTURE(perFlush, lost);
            run.droppedFlush = lost;
            LinkModel            link(oneTickLink(1u));
            const DownlinkResult result = runDownlink(link, run);
            const std::vector<std::uint32_t>& ticks = result.flushTicks[static_cast<std::size_t>(lost)];
            CHECK(result.missing == std::set<std::uint32_t>(ticks.begin(), ticks.end()));
        }
    }
}

TEST_CASE("RelayResend: a flush of two entries is recovered newer-only at r = 1 and whole at r = 2",
          "[Network][RelayResend]")
{
    DownlinkRun run;
    run.capturesPerFlush = 2u;

    LinkModel probeLink(oneTickLink(1u));
    const std::size_t flushes = runDownlink(probeLink, run).flushes;
    REQUIRE(flushes == 10u);

    for (std::int64_t lost = 0; lost + 1 < static_cast<std::int64_t>(flushes); ++lost)
    {
        CAPTURE(lost);
        run.droppedFlush = lost;

        run.resendEntries = 1;
        LinkModel            linkOne(oneTickLink(1u));
        const DownlinkResult one = runDownlink(linkOne, run);
        const std::vector<std::uint32_t>& ticks = one.flushTicks[static_cast<std::size_t>(lost)];
        REQUIRE(ticks.size() == 2u);
        CHECK(one.missing == std::set<std::uint32_t>{ ticks[0] });

        run.resendEntries = 2;
        LinkModel            linkTwo(oneTickLink(1u));
        const DownlinkResult two = runDownlink(linkTwo, run);
        CHECK(two.missing.empty());
    }
}

TEST_CASE("RelayResend: under random loss at r = 1 a tick is missing only when its flush and the next are both lost",
          "[Network][RelayResend]")
{
    linkModel::Config config = oneTickLink(8u);
    config.lossProbability   = 0.3;

    DownlinkRun run;
    run.resendEntries    = 1;
    run.capturesPerFlush = 1u;
    run.lastCaptureTick  = 60u;

    LinkModel            link(config);
    const DownlinkResult result = runDownlink(link, run);

    const std::set<std::uint64_t> lost(link.lostSends().begin(), link.lostSends().end());
    std::set<std::uint32_t> expected;
    for (std::uint32_t tick = 1u; tick <= run.lastCaptureTick; ++tick)
    {
        const std::uint64_t flushIndex = tick - 1u;
        const bool nextLostOrAbsent = (flushIndex + 1u >= result.flushes) || (lost.count(flushIndex + 1u) != 0u);
        if (lost.count(flushIndex) != 0u && nextLostOrAbsent)
        {
            expected.insert(tick);
        }
    }
    REQUIRE(run.lastCaptureTick < kRemoteInputCacheCapacityTicks);
    CHECK(lost.size() > 8u);
    CHECK_FALSE(expected.empty());
    CHECK(result.missing == expected);

    LinkModel            linkAtZero(config);
    DownlinkRun          atZero = run;
    atZero.resendEntries = 0;
    const DownlinkResult zero = runDownlink(linkAtZero, atZero);
    CHECK(zero.missing.size() == lost.size());
    CHECK(result.missing.size() < zero.missing.size());
}

TEST_CASE("RelayResend: a duplicated and reordered delivery leaves the store identical (ingest is last-wins)",
          "[Network][RelayResend]")
{
    for (const std::int32_t resend : { 1, 2 })
    {
        for (const std::uint32_t perFlush : { 1u, 2u })
        {
            CAPTURE(resend, perFlush);
            DownlinkRun run;
            run.resendEntries    = resend;
            run.capturesPerFlush = perFlush;
            run.lastCaptureTick  = 40u;

            LinkModel            cleanLink(oneTickLink(1u));
            const DownlinkResult clean = runDownlink(cleanLink, run);
            REQUIRE(clean.missing.empty());

            linkModel::Config messy   = oneTickLink(29u);
            messy.jitterShape         = linkModel::JitterShape::Uniform;
            messy.jitterSpreadTicks   = 2.5;
            messy.reorderProbability  = 0.3;
            messy.reorderWindowPackets = 2u;
            LinkModel   messyLink(messy);
            DownlinkRun twice   = run;
            twice.deliverTwice  = true;
            const DownlinkResult shuffled = runDownlink(messyLink, twice);
            CHECK_FALSE(messyLink.reorderedSends().empty());

            CHECK(shuffled.store.residentCount() == clean.store.residentCount());
            for (std::uint32_t tick = 1u; tick <= run.lastCaptureTick; ++tick)
            {
                CAPTURE(tick);
                std::uint8_t dAClean = 0u;
                std::uint8_t dAShuffled = 0u;
                ResendInput  inputClean;
                ResendInput  inputShuffled;
                REQUIRE(clean.store.find(tick, dAClean, inputClean));
                REQUIRE(shuffled.store.find(tick, dAShuffled, inputShuffled));
                CHECK(dAShuffled == dAClean);
                CHECK(inputShuffled.captureTick == inputClean.captureTick);
                CHECK(inputShuffled.value == inputClean.value);
            }
        }
    }
}

TEST_CASE("RelayResend: an entry count above kMaxDepth is unreachable",
          "[Network][RelayResend]")
{
    const std::uint32_t stride = static_cast<std::uint32_t>(sizeof(std::uint32_t) + sizeof(std::uint8_t)
                                                           + 2u * sizeof(std::int32_t));
    const std::int32_t requests[] = { -3, 0, 1, 2, 3, 99 };
    for (std::uint32_t previous = 1u; previous <= relayedInputRing::kMaxDepth; ++previous)
    {
        for (std::uint32_t staged = 1u; staged <= relayedInputRing::kMaxDepth + 1u; ++staged)
        {
            for (const std::int32_t request : requests)
            {
                CAPTURE(previous, staged, request);
                ResendBuffer  stageBuffer;
                ResendBuffer  ring;
                std::uint32_t tick = 500u;
                for (std::uint32_t i = 0u; i < previous; ++i)
                {
                    stage(stageBuffer, tick++);
                }
                REQUIRE(relayedInputRing::flushStagedInto(ring, stageBuffer, 2) == previous);

                for (std::uint32_t i = 0u; i < staged; ++i)
                {
                    stage(stageBuffer, tick++);
                }
                const std::uint32_t resident =
                    staged < relayedInputRing::kMaxDepth ? staged : relayedInputRing::kMaxDepth;
                REQUIRE(relayedInputRing::flushStagedInto(ring, stageBuffer, request) == resident);

                const std::uint32_t room    = relayedInputRing::kMaxDepth - resident;
                std::uint32_t       tailMax = relayedInputRing::clampResendEntries(request);
                tailMax = tailMax < room ? tailMax : room;
                tailMax = tailMax < previous ? tailMax : previous;

                const std::uint8_t count = relayedInputRing::entryCount(ring);
                CHECK(count == resident + tailMax);
                CHECK(count <= relayedInputRing::kMaxDepth);
                CHECK(ring.bytes.size() == relayedInputRing::kHeaderBytes + count * stride);
                CHECK(relayedInputRing::isAcceptableWireLength(static_cast<std::uint32_t>(ring.bytes.size())));
            }
        }
    }
}

TEST_CASE("RelayResend: the tail is the newest entries the ring last published",
          "[Network][RelayResend]")
{
    ResendBuffer stageBuffer;
    ResendBuffer ring;

    stage(stageBuffer, 100u);
    stage(stageBuffer, 101u);
    stage(stageBuffer, 102u);
    REQUIRE(relayedInputRing::flushStagedInto(ring, stageBuffer, 2) == 3u);
    CHECK(ticksInRingOrder(ring) == std::vector<std::uint32_t>{ 100u, 101u, 102u });

    stage(stageBuffer, 103u);
    REQUIRE(relayedInputRing::flushStagedInto(ring, stageBuffer, 2) == 1u);
    CHECK(ticksInRingOrder(ring) == std::vector<std::uint32_t>{ 101u, 102u, 103u });

    stage(stageBuffer, 104u);
    stage(stageBuffer, 105u);
    REQUIRE(relayedInputRing::flushStagedInto(ring, stageBuffer, 1) == 2u);
    CHECK(ticksInRingOrder(ring) == std::vector<std::uint32_t>{ 103u, 104u, 105u });

    SECTION("a staged capture tick is never re-sent from the tail as well")
    {
        stage(stageBuffer, 105u, 9u);
        stage(stageBuffer, 106u);
        REQUIRE(relayedInputRing::flushStagedInto(ring, stageBuffer, 2) == 2u);
        CHECK(ticksInRingOrder(ring) == std::vector<std::uint32_t>{ 103u, 104u, 105u, 106u });
        std::uint8_t dA = 0u;
        ResendInput  input;
        REQUIRE(relayedInputRing::findEntry<ResendInput>(ring, 105u, dA, input));
        CHECK(dA == 9u);
    }

    SECTION("a ring of another wire version yields no tail")
    {
        ring.writeToBuffer(relayedInputRing::kVersionOffset,
                           static_cast<std::uint8_t>(relayedInputRing::kWireFormatVersion + 1u));
        stage(stageBuffer, 106u);
        REQUIRE(relayedInputRing::flushStagedInto(ring, stageBuffer, 2) == 1u);
        CHECK(ticksInRingOrder(ring) == std::vector<std::uint32_t>{ 106u });
        CHECK(relayedInputRing::getWireFormatVersion(ring) == relayedInputRing::kWireFormatVersion);
    }

    SECTION("a never-written ring yields no tail")
    {
        ResendBuffer fresh;
        stage(stageBuffer, 106u);
        REQUIRE(relayedInputRing::flushStagedInto(fresh, stageBuffer, 2) == 1u);
        CHECK(ticksInRingOrder(fresh) == std::vector<std::uint32_t>{ 106u });
    }
}

TEST_CASE("RelayResend: an empty stage re-sends nothing and leaves the ring byte-identical",
          "[Network][RelayResend]")
{
    ResendBuffer stageBuffer;
    ResendBuffer ring;
    stage(stageBuffer, 10u);
    stage(stageBuffer, 11u);
    REQUIRE(relayedInputRing::flushStagedInto(ring, stageBuffer, 2) == 2u);
    stage(stageBuffer, 12u);
    REQUIRE(relayedInputRing::flushStagedInto(ring, stageBuffer, 2) == 1u);

    const std::vector<std::uint8_t> published = ring.bytes;
    for (int frame = 0; frame < 5; ++frame)
    {
        CHECK(relayedInputRing::flushStagedInto(ring, stageBuffer, 2) == 0u);
        CHECK(ring.bytes == published);
    }
    CHECK(relayedInputRing::entryCount(stageBuffer) == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS
