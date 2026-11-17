// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/Network/SimPayloadBuffers.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationNetSync.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <tuple>
#include <vector>

namespace simPayloadBuffersTest
{
    struct SpbPosition
    {
        std::int32_t x = 0;
    };

    struct SpbHealth
    {
        float value = 0.f;
    };

    struct SpbInput
    {
        std::int32_t move  = 0;
        std::uint8_t flags = 0u;
    };
} // namespace simPayloadBuffersTest

template <>
struct SerializableFields<simPayloadBuffersTest::SpbPosition>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(simPayloadBuffersTest::SpbPosition, x));
    }
};

template <>
struct SerializableFields<simPayloadBuffersTest::SpbHealth>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(simPayloadBuffersTest::SpbHealth, value));
    }
};

template <>
struct SerializableFields<simPayloadBuffersTest::SpbInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(simPayloadBuffersTest::SpbInput, move),
                               SIM_MEMBER(simPayloadBuffersTest::SpbInput, flags));
    }
};

namespace simPayloadBuffersTest
{
    using SpbState = SimulationComposite<SpbPosition, SpbHealth>;

    static_assert(CorrectionStateSyncedBufferConcept<CorrectionStateBuffer, SpbState>,
        "CorrectionStateBuffer must satisfy the correction buffer concept NetSync binds, as FSimulationStateSyncBuffer does");
    static_assert(CompositeSyncedBufferConcept<CorrectionStateBuffer, SpbState>,
        "CorrectionStateBuffer must satisfy the composite buffer concept the correction concept refines");

    struct SpbCorrRefBuffer
    {
        std::vector<std::uint8_t> bytes = std::vector<std::uint8_t>(384, 0u);

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        {
            std::memcpy(bytes.data() + off, &value, sizeof(T));
        }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        {
            T value{};
            std::memcpy(&value, bytes.data() + off, sizeof(T));
            return value;
        }
    };

    struct SpbGrowRefBuffer
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
            if (byteCount < 0)
                byteCount = 0;
            if (static_cast<std::size_t>(byteCount) < bytes.size())
                bytes.resize(static_cast<std::size_t>(byteCount));
        }
    };

    SpbState makeState(std::int32_t x, float health)
    {
        SpbState state;
        state.edit<SpbPosition>().x   = x;
        state.edit<SpbHealth>().value = health;
        return state;
    }

    std::vector<std::uint8_t> toU8(std::span<const std::byte> bytes)
    {
        std::vector<std::uint8_t> out;
        out.reserve(bytes.size());
        for (const std::byte b : bytes)
            out.push_back(std::to_integer<std::uint8_t>(b));
        return out;
    }

    constexpr std::uint32_t kSpbCompositeBytes = 8u;
    constexpr std::uint32_t kSpbUsedBytes      = correctionStateBuffer::kHeaderBytes + kSpbCompositeBytes;

    static_assert(compositeSyncSize<SpbPosition, SpbHealth>() == kSpbCompositeBytes,
        "the test composite is two 4 B fields");

TEST_CASE("SimPayloadBuffers.CorrectionWriteEqualsTheCodecIntoItsTestBuffer", "[WireFormat]")
{
    const SpbState state = makeState(-123456, 42.5f);

    SpbCorrRefBuffer reference;
    correctionStateBuffer::write(reference, state, 7001u, 6994u);

    CorrectionStateBuffer buffer;
    buffer.write(state, 7001u, 6994u);

    REQUIRE(buffer.usedBytes() == kSpbUsedBytes);
    const std::vector<std::uint8_t> expected(reference.bytes.begin(), reference.bytes.begin() + kSpbUsedBytes);
    CHECK(toU8(buffer.payload()) == expected);
    CHECK(buffer.getAppliedCaptureTick() == 6994u);
}

TEST_CASE("SimPayloadBuffers.CorrectionPayloadRoundTripsThroughAssignFromWire", "[WireFormat]")
{
    CorrectionStateBuffer sent;
    sent.write(makeState(77, -3.25f), 123u, 0u);

    CorrectionStateBuffer received;
    REQUIRE(received.assignFromWire(sent.payload()));

    CHECK(received.usedBytes() == sent.usedBytes());
    CHECK(toU8(received.payload()) == toU8(sent.payload()));

    SpbState out;
    CHECK(received.readInto(out) == 123u);
    CHECK(out.get<SpbPosition>().x == 77);
    CHECK(out.get<SpbHealth>().value == -3.25f);
    CHECK(received.getAppliedCaptureTick() == 0u);
}

TEST_CASE("SimPayloadBuffers.CorrectionRefLessWriteCarriesTheSentinel", "[WireFormat]")
{
    CorrectionStateBuffer buffer;
    buffer.write(makeState(5, 1.f), 9u, 9u);
    buffer.write(makeState(6, 2.f), 10u);

    SpbState out;
    CHECK(buffer.readInto(out) == 10u);
    CHECK(out.get<SpbPosition>().x == 6);
    CHECK(buffer.getAppliedCaptureTick() == kNoInputCaptureTick);
}

TEST_CASE("SimPayloadBuffers.CorrectionAssignFromWireRefusesAboveCapacity", "[WireFormat]")
{
    CorrectionStateBuffer buffer;
    buffer.write(makeState(1, 1.f), 1u, 1u);
    const std::vector<std::uint8_t> before = toU8(buffer.payload());

    const std::vector<std::byte> oversize(385u, std::byte{0xAB});
    CHECK_FALSE(buffer.assignFromWire(oversize));
    CHECK(buffer.usedBytes() == kSpbUsedBytes);
    CHECK(toU8(buffer.payload()) == before);

    const std::vector<std::byte> full(384u, std::byte{0x11});
    CHECK(buffer.assignFromWire(full));
    CHECK(buffer.usedBytes() == 384u);
}

TEST_CASE("SimPayloadBuffers.CorrectionWithoutAHeaderAnswersTheSentinel", "[WireFormat]")
{
    CorrectionStateBuffer fresh;
    CHECK(fresh.usedBytes() == 0u);
    CHECK(fresh.payload().empty());
    CHECK(fresh.getAppliedCaptureTick() == kNoInputCaptureTick);

    const std::vector<std::byte> shortWire(correctionStateBuffer::kHeaderBytes - 1u, std::byte{0});
    CorrectionStateBuffer received;
    REQUIRE(received.assignFromWire(shortWire));
    CHECK(received.getAppliedCaptureTick() == kNoInputCaptureTick);

    CorrectionStateBuffer zeroRef;
    zeroRef.write(makeState(0, 0.f), 0u, 0u);
    CHECK(zeroRef.getAppliedCaptureTick() == 0u);
}

TEST_CASE("SimPayloadBuffers.CorrectionWriteResetsTheWatermarkAfterALongerWirePayload", "[WireFormat]")
{
    CorrectionStateBuffer buffer;
    const std::vector<std::byte> longWire(300u, std::byte{0x5A});
    REQUIRE(buffer.assignFromWire(longWire));
    REQUIRE(buffer.usedBytes() == 300u);

    buffer.write(makeState(3, 4.f), 11u, 10u);
    CHECK(buffer.usedBytes() == kSpbUsedBytes);
    CHECK(buffer.payload().size() == kSpbUsedBytes);
}

TEST_CASE("SimPayloadBuffers.RelayRingStageAndFlushEqualTheCodecIntoItsTestBuffer", "[WireFormat]")
{
    for (const std::int32_t resend : {0, 1, 2})
    {
        CAPTURE(resend);
        SpbGrowRefBuffer refStage;
        SpbGrowRefBuffer refRing;
        RelayRingBuffer  stage;
        RelayRingBuffer  ring;

        const std::vector<std::uint32_t> roundSizes = {3u, 0u, 1u, 12u, 0u, 0u, 2u, 8u, 1u, 0u, 5u};
        std::uint32_t captureTick = 100u;
        for (const std::uint32_t size : roundSizes)
        {
            CAPTURE(size, captureTick);
            for (std::uint32_t i = 0; i < size; ++i)
            {
                const SpbInput input{static_cast<std::int32_t>(captureTick * 3u), static_cast<std::uint8_t>(captureTick & 0xFFu)};
                const std::uint8_t dA = static_cast<std::uint8_t>(4u + (captureTick % 3u));
                const relayedInputRing::StageArrivalOutcome expected =
                    relayedInputRing::stageArrival<SpbInput>(refStage, captureTick, dA, input);
                const relayedInputRing::StageArrivalOutcome actual = stage.stageArrival<SpbInput>(captureTick, dA, input);
                CHECK(actual.accepted == expected.accepted);
                CHECK(actual.droppedOldest == expected.droppedOldest);
                ++captureTick;
            }

            const std::uint8_t expectedCount = relayedInputRing::flushStagedInto(refRing, refStage, resend);
            const std::uint8_t actualCount   = relayedInputRing::flushStagedInto(ring, stage, resend);
            CHECK(actualCount == expectedCount);
            CHECK(toU8(ring.payload()) == refRing.bytes);
            CHECK(toU8(stage.payload()) == refStage.bytes);
        }

        RelayRingBuffer received;
        REQUIRE(received.assignFromWire(ring.payload()));
        CHECK(toU8(received.payload()) == refRing.bytes);
        CHECK(received.num() == relayedInputRing::entryCount(refRing));
        CHECK(received.getWireFormatVersionOnWire() == relayedInputRing::kWireFormatVersion);

        std::vector<std::uint32_t> expectedTicks;
        relayedInputRing::forEachEntry<SpbInput>(refRing,
            [&expectedTicks](std::uint32_t tick, std::uint8_t, const SpbInput&) { expectedTicks.push_back(tick); });
        std::vector<std::uint32_t> actualTicks;
        received.forEachEntry<SpbInput>(
            [&actualTicks](std::uint32_t tick, std::uint8_t, const SpbInput&) { actualTicks.push_back(tick); });
        CHECK(actualTicks == expectedTicks);

        REQUIRE_FALSE(expectedTicks.empty());
        std::uint8_t dA = 0u;
        SpbInput input{};
        CHECK(received.findEntry<SpbInput>(expectedTicks.back(), dA, input));
        CHECK(input.move == static_cast<std::int32_t>(expectedTicks.back() * 3u));
    }
}

TEST_CASE("SimPayloadBuffers.RelayRingWriteLatestEqualsTheCodecIntoItsTestBuffer", "[WireFormat]")
{
    SpbGrowRefBuffer reference;
    RelayRingBuffer  ring;
    for (std::uint32_t tick = 50u; tick < 70u; ++tick)
    {
        const SpbInput input{static_cast<std::int32_t>(tick), 1u};
        CHECK(ring.writeLatest<SpbInput>(tick, 6u, input, 3) ==
              relayedInputRing::writeLatest<SpbInput>(reference, tick, 6u, input, 3));
        CHECK(toU8(ring.payload()) == reference.bytes);
    }
    CHECK(ring.num() == 3u);
}

TEST_CASE("SimPayloadBuffers.RelayRingAssignFromWireRefusesAboveTheWireBound", "[WireFormat]")
{
    RelayRingBuffer ring;
    const std::vector<std::byte> oversize(relayedInputRing::kMaxWireBytes + 1u, std::byte{0});
    CHECK_FALSE(ring.assignFromWire(oversize));
    CHECK(ring.payload().empty());

    const std::vector<std::byte> full(relayedInputRing::kMaxWireBytes, std::byte{0});
    CHECK(ring.assignFromWire(full));
    CHECK(ring.payload().size() == relayedInputRing::kMaxWireBytes);
}

TEST_CASE("SimPayloadBuffers.BundleEqualsTheCodecIntoItsTestBuffer", "[WireFormat]")
{
    SpbGrowRefBuffer reference;
    InputBundleBuffer bundle;
    for (std::uint32_t tick = 200u; tick < 210u; ++tick)
    {
        const SpbInput input{static_cast<std::int32_t>(tick) * -7, static_cast<std::uint8_t>(tick)};
        CHECK(bundle.tryAppendSlot<SpbInput>(tick, input) ==
              inputRedundancyBundle::tryAppendSlot<SpbInput>(reference, tick, input));
        CHECK(bundle.tryAppendSlot<SpbInput>(tick, input) ==
              inputRedundancyBundle::tryAppendSlot<SpbInput>(reference, tick, input));
        CHECK(toU8(bundle.payload()) == reference.bytes);
    }
    CHECK(inputRedundancyBundle::slotCount(bundle) == InputBundleBuffer::kMaxSlots);
    CHECK(bundle.getWireFormatVersion() == inputRedundancyBundle::kWireFormatVersion);

    InputBundleBuffer received;
    REQUIRE(received.assignFromWire(bundle.payload()));
    std::vector<std::uint32_t> ticks;
    received.forEachSlot<SpbInput>([&ticks](std::uint32_t tick, const SpbInput& input)
    {
        CHECK(input.move == static_cast<std::int32_t>(tick) * -7);
        ticks.push_back(tick);
    });
    CHECK(ticks == std::vector<std::uint32_t>{200u, 201u, 202u, 203u, 204u, 205u, 206u, 207u});

    bundle.reset();
    SpbGrowRefBuffer fresh;
    bundle.appendSlot<SpbInput>(300u, SpbInput{1, 2u});
    inputRedundancyBundle::appendSlot<SpbInput>(fresh, 300u, SpbInput{1, 2u});
    CHECK(toU8(bundle.payload()) == fresh.bytes);
}

TEST_CASE("SimPayloadBuffers.BundleAssignFromWireRefusesAboveCapacity", "[WireFormat]")
{
    InputBundleBuffer bundle;
    const std::vector<std::byte> oversize(InputBundleBuffer::kMaxWireBytes + 1u, std::byte{0});
    CHECK_FALSE(bundle.assignFromWire(oversize));
    CHECK(bundle.payload().empty());
    CHECK(bundle.getWireFormatVersion() == 0u);
}
} // namespace simPayloadBuffersTest

#endif
