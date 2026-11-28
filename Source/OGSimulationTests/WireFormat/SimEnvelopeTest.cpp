// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/Network/SimEnvelope.h"
#include "OGSimulation/Network/SimPayloadBuffers.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace simEnvelopeTest
{
struct EnvInput
{
	uint8_t a = 0u;
	uint8_t b = 0u;
};

struct EnvState
{
	int32_t x = 0;
};
} // namespace simEnvelopeTest

template <>
struct SerializableFields<simEnvelopeTest::EnvInput>
{
	static constexpr auto get()
	{
		return std::make_tuple(SIM_MEMBER(simEnvelopeTest::EnvInput, a), SIM_MEMBER(simEnvelopeTest::EnvInput, b));
	}
};

template <>
struct SerializableFields<simEnvelopeTest::EnvState>
{
	static constexpr auto get()
	{
		return std::make_tuple(SIM_MEMBER(simEnvelopeTest::EnvState, x));
	}
};

namespace simEnvelopeTest
{
using EnvComposite = SimulationComposite<EnvState>;
using TestShape = simEnvelope::ShapeFor<EnvInput, EnvComposite>;
using BrawlerStrideShape = simEnvelope::Shape<39u, 325u>;

static_assert(std::is_same_v<TestShape, simEnvelope::Shape<2u, 4u>>,
	"the test input is two 1 B fields and the test state one 4 B field");
static_assert(BrawlerStrideShape::kMaxBundleBytes == 346u, "a brawler bundle is 2 + 8 x (4 + 39) B");
static_assert(BrawlerStrideShape::kCorrectionPayloadBytes == 333u, "a brawler correction is 8 + 325 B");
static_assert(BrawlerStrideShape::inputBundleSetMaxBytes(4u) == 1391u, "a four-player brawler set is 3 + 4 x (1 + 346) B");

std::vector<uint8_t> hex(const std::string& text)
{
	std::vector<uint8_t> out;
	int high = -1;
	for (const char c : text)
	{
		int nibble = -1;
		if (c >= '0' && c <= '9')
			nibble = c - '0';
		else if (c >= 'A' && c <= 'F')
			nibble = c - 'A' + 10;
		else if (c >= 'a' && c <= 'f')
			nibble = c - 'a' + 10;
		if (nibble < 0)
			continue;
		if (high < 0)
		{
			high = nibble;
		}
		else
		{
			out.push_back(static_cast<uint8_t>(high * 16 + nibble));
			high = -1;
		}
	}
	REQUIRE(high < 0);
	return out;
}

std::vector<std::byte> toBytes(const std::vector<uint8_t>& values)
{
	std::vector<std::byte> out;
	out.reserve(values.size());
	for (const uint8_t v : values)
		out.push_back(static_cast<std::byte>(v));
	return out;
}

std::vector<uint8_t> toU8(std::span<const std::byte> bytes)
{
	std::vector<uint8_t> out;
	out.reserve(bytes.size());
	for (const std::byte b : bytes)
		out.push_back(std::to_integer<uint8_t>(b));
	return out;
}

struct Decoded
{
	DecodeRefusal refusal = DecodeRefusal::None;
	SimMessageKind kind = SimMessageKind{};
	uint8_t simId = 0u;
	uint8_t count = 0u;
	uint8_t corrVersion = 0u;
	uint8_t value = 0u;
	uint32_t tick = 0u;
	bool bodyInSpan = true;
	std::vector<uint8_t> body;
};

bool operator==(const Decoded& l, const Decoded& r)
{
	return l.refusal == r.refusal && l.kind == r.kind && l.simId == r.simId && l.count == r.count
		&& l.corrVersion == r.corrVersion && l.value == r.value && l.tick == r.tick && l.bodyInSpan == r.bodyInSpan
		&& l.body == r.body;
}

template <typename Shape = TestShape>
Decoded decodeGuarded(SimLane lane, SimDirection direction, const std::vector<uint8_t>& message, size_t length, uint8_t guard)
{
	std::vector<std::byte> storage(length + 64u, static_cast<std::byte>(guard));
	for (size_t i = 0u; i < length; ++i)
		storage[i] = static_cast<std::byte>(message[i]);
	const std::span<const std::byte> bytes(storage.data(), length);
	const DecodedSimMessage message0 = simEnvelope::decode<Shape>(lane, direction, bytes);

	Decoded out;
	out.refusal = message0.refusal;
	out.kind = message0.kind;
	out.simId = message0.simId;
	out.count = message0.count;
	out.corrVersion = message0.corrVersion;
	out.value = message0.value;
	out.tick = message0.tick;
	const std::byte* const begin = storage.data();
	const std::byte* const end = storage.data() + length;
	out.bodyInSpan = message0.body.empty()
		|| (message0.body.data() >= begin && message0.body.size() <= static_cast<size_t>(end - message0.body.data()));
	if (out.bodyInSpan)
		out.body = toU8(message0.body);
	return out;
}

template <typename Shape = TestShape>
Decoded decodeAll(SimLane lane, SimDirection direction, const std::vector<uint8_t>& message)
{
	return decodeGuarded<Shape>(lane, direction, message, message.size(), 0xA5u);
}

struct Route
{
	SimLane lane;
	SimDirection direction;
};

constexpr SimDirection kUp = SimDirection::ClientToServer;
constexpr SimDirection kDown = SimDirection::ServerToClient;

struct GoldenVector
{
	std::string name;
	SimMessageKind kind;
	std::vector<uint8_t> bytes;
};

const std::array<Route, 10> kExpectedRoutes = {{
	{SimLane::SimInput, kUp},
	{SimLane::SimRelay, kDown},
	{SimLane::SimState, kDown},
	{SimLane::SimClock, kDown},
	{SimLane::SimClock, kUp},
	{SimLane::SimClock, kDown},
	{SimLane::Control, kDown},
	{SimLane::Control, kDown},
	{SimLane::Control, kDown},
	{SimLane::Control, kDown},
}};

Route routeOf(SimMessageKind kind)
{
	return kExpectedRoutes[static_cast<size_t>(kind) - 1u];
}

const char* const kSetHex = "01 01 02  03 03 01 10 00 00 00 AA BB  05 03 02 11 00 00 00 01 02 12 00 00 00 03 04";
const char* const kRingHex = "01 02 04  03 01 20 00 00 00 02 CC DD";
const char* const kCorrectionHex = "01 03 02 0F  30 00 00 00 2F 00 00 00 01 02 03 04";
const char* const kClockHex = "01 04 78 56 34 12";
const char* const kSnapshotHex = "01 07 01 01 05 00 02 01 00 FF 02 01 00";
const char* const kAddHex = "01 07 01 02 06 00 03 02 01";
const char* const kRemoveHex = "01 07 01 03 07 00 02 01";
const char* const kFloorHex = "01 08 06";
const char* const kTierHex = "01 09 02";

std::vector<GoldenVector> goldenVectors()
{
	return {
		{"InputBundleSet", SimMessageKind::InputBundleSet, hex(kSetHex)},
		{"RelayRing", SimMessageKind::RelayRing, hex(kRingHex)},
		{"CorrectionState", SimMessageKind::CorrectionState, hex(kCorrectionHex)},
		{"ClockPublish", SimMessageKind::ClockPublish, hex(kClockHex)},
		{"Roster.Snapshot", SimMessageKind::Roster, hex(kSnapshotHex)},
		{"Roster.Add", SimMessageKind::Roster, hex(kAddHex)},
		{"Roster.Remove", SimMessageKind::Roster, hex(kRemoveHex)},
		{"RelayFloor", SimMessageKind::RelayFloor, hex(kFloorHex)},
		{"ConnectionTier", SimMessageKind::ConnectionTier, hex(kTierHex)},
	};
}

std::vector<uint8_t> kindVector(SimMessageKind kind)
{
	for (const GoldenVector& golden : goldenVectors())
	{
		if (golden.kind == kind)
			return golden.bytes;
	}
	return {0x01u, static_cast<uint8_t>(kind)};
}

EnvComposite makeState(int32_t x)
{
	EnvComposite state;
	state.edit<EnvState>().x = x;
	return state;
}

std::vector<uint8_t> brawlerBundle(uint8_t seed)
{
	std::vector<uint8_t> bundle = {inputRedundancyBundle::kWireFormatVersion, inputRedundancyBundle::kMaxSlots};
	for (uint8_t slot = 0u; slot < inputRedundancyBundle::kMaxSlots; ++slot)
	{
		const uint32_t tick = 1000u * seed + slot;
		bundle.push_back(static_cast<uint8_t>(tick & 0xFFu));
		bundle.push_back(static_cast<uint8_t>((tick >> 8u) & 0xFFu));
		bundle.push_back(static_cast<uint8_t>((tick >> 16u) & 0xFFu));
		bundle.push_back(static_cast<uint8_t>((tick >> 24u) & 0xFFu));
		for (uint32_t i = 0u; i < BrawlerStrideShape::kInputBytes; ++i)
			bundle.push_back(static_cast<uint8_t>(seed * 31u + slot * 7u + i));
	}
	return bundle;
}

struct SeenBundle
{
	uint8_t simId;
	std::vector<uint8_t> bytes;
};

template <typename Shape = TestShape>
std::vector<SeenBundle> collectBundles(std::span<const std::byte> setBody, DecodeRefusal& outRefusal)
{
	std::vector<SeenBundle> seen;
	outRefusal = simEnvelope::forEachInputBundle<Shape>(setBody, [&](uint8_t simId, std::span<const std::byte> bundle) {
		seen.push_back({simId, toU8(bundle)});
	});
	return seen;
}
} // namespace simEnvelopeTest

using namespace simEnvelopeTest;

TEST_CASE("SimEnvelope.TheCodecsBytesTravelUntouchedInsideTheGoldenVectors", "[WireFormat]")
{
	InputBundleBuffer bundle3;
	bundle3.appendSlot(0x10u, EnvInput{0xAAu, 0xBBu});
	InputBundleBuffer bundle5;
	bundle5.appendSlot(0x11u, EnvInput{0x01u, 0x02u});
	bundle5.appendSlot(0x12u, EnvInput{0x03u, 0x04u});
	CHECK(toU8(bundle3.payload()) == hex("03 01 10 00 00 00 AA BB"));
	CHECK(toU8(bundle5.payload()) == hex("03 02 11 00 00 00 01 02 12 00 00 00 03 04"));

	RelayRingBuffer ring;
	REQUIRE(ring.writeLatest(0x20u, 2u, EnvInput{0xCCu, 0xDDu}, relayedInputRing::kMaxDepth));
	CHECK(toU8(ring.payload()) == hex("03 01 20 00 00 00 02 CC DD"));

	CorrectionStateBuffer correction;
	correction.write(makeState(0x04030201), 0x30u, 0x2Fu);
	CHECK(toU8(correction.payload()) == hex("30 00 00 00 2F 00 00 00 01 02 03 04"));
	CHECK(CorrectionStateBuffer::kWireFormatVersion == 0x0Fu);

	std::array<std::byte, 64> out{};
	size_t used = simEnvelope::writeInputBundleSetHeader(out);
	REQUIRE(used == 3u);
	used = simEnvelope::appendInputBundle(out, used, 3u, bundle3.payload());
	REQUIRE(used == 12u);
	used = simEnvelope::appendInputBundle(out, used, 5u, bundle5.payload());
	CHECK(toU8(std::span<const std::byte>(out.data(), used)) == hex(kSetHex));

	used = simEnvelope::writeRelayRing(out, 4u, ring.payload());
	CHECK(toU8(std::span<const std::byte>(out.data(), used)) == hex(kRingHex));

	used = simEnvelope::writeCorrectionState(out, 2u, CorrectionStateBuffer::kWireFormatVersion, correction.payload());
	CHECK(toU8(std::span<const std::byte>(out.data(), used)) == hex(kCorrectionHex));
}

TEST_CASE("SimEnvelope.InputBundleSetGoldenVectorDecodesToItsBundles", "[WireFormat]")
{
	const Decoded decoded = decodeAll(SimLane::SimInput, kUp, hex(kSetHex));
	REQUIRE(decoded.refusal == DecodeRefusal::None);
	CHECK(decoded.kind == SimMessageKind::InputBundleSet);
	CHECK(decoded.count == 2u);
	CHECK(decoded.simId == 0u);
	CHECK(decoded.body == hex("02  03 03 01 10 00 00 00 AA BB  05 03 02 11 00 00 00 01 02 12 00 00 00 03 04"));

	const std::vector<std::byte> body = toBytes(decoded.body);
	DecodeRefusal refusal = DecodeRefusal::Version;
	const std::vector<SeenBundle> seen = collectBundles(body, refusal);
	CHECK(refusal == DecodeRefusal::None);
	REQUIRE(seen.size() == 2u);
	CHECK(seen[0].simId == 3u);
	CHECK(seen[0].bytes == hex("03 01 10 00 00 00 AA BB"));
	CHECK(seen[1].simId == 5u);
	CHECK(seen[1].bytes == hex("03 02 11 00 00 00 01 02 12 00 00 00 03 04"));

	InputBundleBuffer received;
	REQUIRE(received.assignFromWire(toBytes(seen[1].bytes)));
	std::vector<std::pair<uint32_t, EnvInput>> slots;
	received.forEachSlot<EnvInput>([&](uint32_t tick, const EnvInput& input) { slots.push_back({tick, input}); });
	REQUIRE(slots.size() == 2u);
	CHECK(slots[0].first == 0x11u);
	CHECK(slots[0].second.a == 0x01u);
	CHECK(slots[1].first == 0x12u);
	CHECK(slots[1].second.b == 0x04u);
}

TEST_CASE("SimEnvelope.RelayRingGoldenVectorDecodesToItsRing", "[WireFormat]")
{
	const Decoded decoded = decodeAll(SimLane::SimRelay, kDown, hex(kRingHex));
	REQUIRE(decoded.refusal == DecodeRefusal::None);
	CHECK(decoded.kind == SimMessageKind::RelayRing);
	CHECK(decoded.simId == 4u);
	CHECK(decoded.count == 1u);
	CHECK(decoded.body == hex("03 01 20 00 00 00 02 CC DD"));

	RelayRingBuffer received;
	REQUIRE(received.assignFromWire(toBytes(decoded.body)));
	uint8_t dA = 0u;
	EnvInput input;
	REQUIRE(received.findEntry(0x20u, dA, input));
	CHECK(dA == 2u);
	CHECK(input.a == 0xCCu);
	CHECK(input.b == 0xDDu);
}

TEST_CASE("SimEnvelope.CorrectionStateGoldenVectorDecodesToItsPayload", "[WireFormat]")
{
	const Decoded decoded = decodeAll(SimLane::SimState, kDown, hex(kCorrectionHex));
	REQUIRE(decoded.refusal == DecodeRefusal::None);
	CHECK(decoded.kind == SimMessageKind::CorrectionState);
	CHECK(decoded.simId == 2u);
	CHECK(decoded.corrVersion == 0x0Fu);
	CHECK(decoded.tick == 0x30u);
	CHECK(decoded.body == hex("30 00 00 00 2F 00 00 00 01 02 03 04"));

	CorrectionStateBuffer received;
	REQUIRE(received.assignFromWire(toBytes(decoded.body)));
	CHECK(received.usedBytes() == 12u);
	EnvComposite state;
	CHECK(received.readInto(state) == 0x30u);
	CHECK(state.get<EnvState>().x == 0x04030201);
	CHECK(received.getAppliedCaptureTick() == 0x2Fu);
}

TEST_CASE("SimEnvelope.ClockFloorAndTierGoldenVectorsWriteAndDecode", "[WireFormat]")
{
	std::array<std::byte, 8> out{};
	size_t used = simEnvelope::writeClockPublish(out, 0x12345678u);
	CHECK(toU8(std::span<const std::byte>(out.data(), used)) == hex(kClockHex));
	const Decoded clock = decodeAll(SimLane::SimClock, kDown, hex(kClockHex));
	REQUIRE(clock.refusal == DecodeRefusal::None);
	CHECK(clock.kind == SimMessageKind::ClockPublish);
	CHECK(clock.tick == 0x12345678u);
	CHECK(clock.body == hex("78 56 34 12"));

	used = simEnvelope::writeRelayFloor(out, 6u);
	CHECK(toU8(std::span<const std::byte>(out.data(), used)) == hex(kFloorHex));
	const Decoded floor = decodeAll(SimLane::Control, kDown, hex(kFloorHex));
	REQUIRE(floor.refusal == DecodeRefusal::None);
	CHECK(floor.kind == SimMessageKind::RelayFloor);
	CHECK(floor.value == 6u);

	used = simEnvelope::writeConnectionTier(out, 2u);
	CHECK(toU8(std::span<const std::byte>(out.data(), used)) == hex(kTierHex));
	const Decoded tier = decodeAll(SimLane::Control, kDown, hex(kTierHex));
	REQUIRE(tier.refusal == DecodeRefusal::None);
	CHECK(tier.kind == SimMessageKind::ConnectionTier);
	CHECK(tier.value == 2u);
}

TEST_CASE("SimEnvelope.TheReservedKindsAreRefusedAsKindOnTheirOwnRoute", "[WireFormat]")
{
	for (const SimMessageKind kind : {SimMessageKind::ClockPing, SimMessageKind::ClockPong, SimMessageKind::ArrivalMarginSignal})
	{
		const Route route = routeOf(kind);
		const Decoded decoded = decodeAll(route.lane, route.direction, {0x01u, static_cast<uint8_t>(kind), 0x00u, 0x01u});
		const int kindNumber = static_cast<int>(kind);
		CAPTURE(kindNumber);
		CHECK(decoded.refusal == DecodeRefusal::Kind);
		CHECK(decoded.kind == kind);
		CHECK(decoded.body.empty());
	}
}

TEST_CASE("SimEnvelope.RosterGoldenVectorsTravelByteForByteInKind7", "[WireFormat]")
{
	struct RosterCase
	{
		const char* roster;
		DecodeRefusal envelopeVerdict;
	};
	const std::array<RosterCase, 7> cases = {{
		{"01 01 05 00 02 01 00 FF 02 01 00", DecodeRefusal::None},
		{"01 02 06 00 03 02 01", DecodeRefusal::None},
		{"01 03 07 00 02 01", DecodeRefusal::None},
		{"02 01 05 00 00", DecodeRefusal::None},
		{"01 01 05 00 01 00 00 FF", DecodeRefusal::None},
		{"01 02 06 00 03 08 01", DecodeRefusal::None},
		{"01 03 07 00 02", DecodeRefusal::Truncated},
	}};

	for (const RosterCase& rosterCase : cases)
	{
		CAPTURE(std::string(rosterCase.roster));
		const std::vector<uint8_t> roster = hex(rosterCase.roster);
		std::array<std::byte, 64> out{};
		const size_t used = simEnvelope::writeRoster(out, toBytes(roster));
		REQUIRE(used == roster.size() + 2u);
		std::vector<uint8_t> expected = {0x01u, 0x07u};
		expected.insert(expected.end(), roster.begin(), roster.end());
		CHECK(toU8(std::span<const std::byte>(out.data(), used)) == expected);

		const Decoded decoded = decodeAll(SimLane::Control, kDown, expected);
		CHECK(decoded.refusal == rosterCase.envelopeVerdict);
		CHECK(decoded.kind == SimMessageKind::Roster);
		if (rosterCase.envelopeVerdict == DecodeRefusal::None)
			CHECK(decoded.body == roster);
	}
	CHECK(hex(kSnapshotHex) == hex("01 07  01 01 05 00 02 01 00 FF 02 01 00"));
	CHECK(hex(kAddHex) == hex("01 07  01 02 06 00 03 02 01"));
	CHECK(hex(kRemoveHex) == hex("01 07  01 03 07 00 02 01"));
}

TEST_CASE("SimEnvelope.EveryProperPrefixOfEveryGoldenVectorIsTruncated", "[WireFormat]")
{
	std::vector<GoldenVector> vectors = goldenVectors();

	std::vector<uint8_t> brawlerSet = {0x01u, 0x01u, 0x04u};
	for (uint8_t player = 1u; player <= 4u; ++player)
	{
		brawlerSet.push_back(player);
		const std::vector<uint8_t> bundle = brawlerBundle(player);
		brawlerSet.insert(brawlerSet.end(), bundle.begin(), bundle.end());
	}
	REQUIRE(brawlerSet.size() == 1391u);

	size_t prefixes = 0u;
	for (const GoldenVector& golden : vectors)
	{
		CAPTURE(golden.name);
		const Route route = routeOf(golden.kind);
		REQUIRE(decodeAll(route.lane, route.direction, golden.bytes).refusal == DecodeRefusal::None);
		for (size_t length = 0u; length < golden.bytes.size(); ++length)
		{
			CAPTURE(length);
			const Decoded zeroGuard = decodeGuarded(route.lane, route.direction, golden.bytes, length, 0x00u);
			const Decoded fullGuard = decodeGuarded(route.lane, route.direction, golden.bytes, length, 0xFFu);
			CHECK(zeroGuard.refusal == DecodeRefusal::Truncated);
			CHECK(fullGuard.refusal == DecodeRefusal::Truncated);
			CHECK(zeroGuard == fullGuard);
			CHECK(zeroGuard.bodyInSpan);
			CHECK(fullGuard.bodyInSpan);
			++prefixes;
		}
	}

	REQUIRE(decodeAll<BrawlerStrideShape>(SimLane::SimInput, kUp, brawlerSet).refusal == DecodeRefusal::None);
	size_t brawlerFailures = 0u;
	for (size_t length = 0u; length < brawlerSet.size(); ++length)
	{
		const Decoded zeroGuard = decodeGuarded<BrawlerStrideShape>(SimLane::SimInput, kUp, brawlerSet, length, 0x00u);
		const Decoded fullGuard = decodeGuarded<BrawlerStrideShape>(SimLane::SimInput, kUp, brawlerSet, length, 0xFFu);
		if (zeroGuard.refusal != DecodeRefusal::Truncated || !(zeroGuard == fullGuard))
			++brawlerFailures;
		++prefixes;
	}
	CHECK(brawlerFailures == 0u);
	CHECK(prefixes == 1391u + 12u + 27u + 16u + 6u + 13u + 9u + 8u + 3u + 3u);
}

TEST_CASE("SimEnvelope.EachRefusalKindHasACase", "[WireFormat]")
{
	CHECK(decodeAll(SimLane::SimClock, kDown, hex("02 04 78 56 34 12")).refusal == DecodeRefusal::Version);
	CHECK(decodeAll(SimLane::SimClock, kDown, hex("00 04 78 56 34 12")).refusal == DecodeRefusal::Version);
	CHECK(decodeAll(SimLane::SimClock, kDown, hex("01 0B 78 56 34 12")).refusal == DecodeRefusal::Kind);
	CHECK(decodeAll(SimLane::SimClock, kDown, hex("01 00 78 56 34 12")).refusal == DecodeRefusal::Kind);
	CHECK(decodeAll(SimLane::SimState, kDown, hex(kClockHex)).refusal == DecodeRefusal::WrongLane);
	CHECK(decodeAll(SimLane::SimClock, kDown, hex("01 04 78 56 34")).refusal == DecodeRefusal::Truncated);
	CHECK(decodeAll(SimLane::SimClock, kDown, hex("01 04 78 56 34 12 00")).refusal == DecodeRefusal::Overlong);
	CHECK(decodeAll(SimLane::Control, kDown, hex("01 08 06 00")).refusal == DecodeRefusal::Overlong);
	CHECK(decodeAll(SimLane::SimRelay, kDown, hex("01 02 00  03 01 20 00 00 00 02 CC DD")).refusal == DecodeRefusal::SimIdZero);
	CHECK(decodeAll(SimLane::SimState, kDown, hex("01 03 00 0F  30 00 00 00 2F 00 00 00 01 02 03 04")).refusal
		== DecodeRefusal::SimIdZero);
	CHECK(decodeAll(SimLane::SimInput, kUp, hex("01 01 01  00 03 01 10 00 00 00 AA BB")).refusal == DecodeRefusal::SimIdZero);
	CHECK(decodeAll(SimLane::SimInput, kUp, hex("01 01 00")).refusal == DecodeRefusal::CountZero);
	CHECK(decodeAll(SimLane::SimInput, kUp, hex("01 01 09  03 03 01 10 00 00 00 AA BB")).refusal == DecodeRefusal::CountOverMax);
	CHECK(decodeAll(SimLane::SimInput, kUp, hex("01 01 01  03 03 09 10 00 00 00 AA BB")).refusal == DecodeRefusal::CountOverMax);
	CHECK(decodeAll(SimLane::SimRelay, kDown, hex("01 02 04  03 09 20 00 00 00 02 CC DD")).refusal == DecodeRefusal::CountOverMax);
	CHECK(decodeAll(SimLane::SimInput, kUp, hex(std::string(kSetHex) + " 00")).refusal == DecodeRefusal::Overlong);
	CHECK(decodeAll(SimLane::SimRelay, kDown, hex(std::string(kRingHex) + " 00")).refusal == DecodeRefusal::Overlong);
	CHECK(decodeAll(SimLane::SimState, kDown, hex(std::string(kCorrectionHex) + " 00")).refusal == DecodeRefusal::Overlong);
	CHECK(decodeAll(SimLane::Control, kDown, hex(std::string(kAddHex) + " 00")).refusal == DecodeRefusal::Overlong);
	CHECK(decodeAll(SimLane::SimInput, kUp, {}).refusal == DecodeRefusal::Truncated);
}

TEST_CASE("SimEnvelope.InnerVersionMismatchesPassThroughToTheirLatches", "[WireFormat]")
{
	const Decoded correction = decodeAll(SimLane::SimState, kDown, hex("01 03 02 0E  AA BB CC"));
	REQUIRE(correction.refusal == DecodeRefusal::None);
	CHECK(correction.simId == 2u);
	CHECK(correction.corrVersion == 0x0Eu);
	CHECK(correction.tick == 0u);
	CHECK(correction.body == hex("AA BB CC"));
	CHECK(decodeAll(SimLane::SimState, kDown, hex("01 03 02 0E")).refusal == DecodeRefusal::None);

	const Decoded ring = decodeAll(SimLane::SimRelay, kDown, hex("01 02 04  02 05 01"));
	REQUIRE(ring.refusal == DecodeRefusal::None);
	CHECK(ring.simId == 4u);
	CHECK(ring.count == 0u);
	CHECK(ring.body == hex("02 05 01"));

	CHECK(decodeAll(SimLane::SimInput, kUp, hex("01 01 01  03 02 01 10 00 00 00 AA BB")).refusal == DecodeRefusal::Version);

	std::vector<uint8_t> longCorrection = hex("01 03 02 0E");
	longCorrection.resize(4u + CorrectionStateBuffer::kBufferBytes, 0x11u);
	CHECK(decodeAll(SimLane::SimState, kDown, longCorrection).refusal == DecodeRefusal::None);
	longCorrection.push_back(0x11u);
	CHECK(decodeAll(SimLane::SimState, kDown, longCorrection).refusal == DecodeRefusal::Overlong);

	std::vector<uint8_t> longRing = hex("01 02 04 02");
	longRing.resize(3u + relayedInputRing::kMaxWireBytes, 0x22u);
	CHECK(decodeAll(SimLane::SimRelay, kDown, longRing).refusal == DecodeRefusal::None);
	longRing.push_back(0x22u);
	CHECK(decodeAll(SimLane::SimRelay, kDown, longRing).refusal == DecodeRefusal::Overlong);
}

TEST_CASE("SimEnvelope.AKindOffItsLaneOrDirectionIsWrongLane", "[WireFormat]")
{
	size_t combinations = 0u;
	size_t legal = 0u;
	for (uint8_t kindByte = simEnvelope::kFirstKind; kindByte <= simEnvelope::kLastKind; ++kindByte)
	{
		const SimMessageKind kind = static_cast<SimMessageKind>(kindByte);
		const std::vector<uint8_t> message = kindVector(kind);
		for (size_t laneIndex = 0u; laneIndex < kSimLaneCount; ++laneIndex)
		{
			for (const SimDirection direction : {kUp, kDown})
			{
				const SimLane lane = static_cast<SimLane>(laneIndex);
				const int kindNumber = kindByte;
				const int directionNumber = static_cast<int>(direction);
				CAPTURE(kindNumber, laneIndex, directionNumber);
				const Route route = routeOf(kind);
				const bool expectedLegal = route.lane == lane && route.direction == direction;
				CHECK(simEnvelope::isLegal(kind, lane, direction) == expectedLegal);

				const Decoded decoded = decodeAll(lane, direction, message);
				if (!simEnvelope::isLegal(kind, lane, direction))
				{
					CHECK(decoded.refusal == DecodeRefusal::WrongLane);
					CHECK(decoded.kind == kind);
				}
				else if (!simEnvelope::hasBodyLayout(kind))
				{
					CHECK(decoded.refusal == DecodeRefusal::Kind);
					++legal;
				}
				else
				{
					CHECK(decoded.refusal == DecodeRefusal::None);
					++legal;
				}
				++combinations;
			}
		}
	}
	CHECK(combinations == 100u);
	CHECK(legal == 10u);
}

TEST_CASE("SimEnvelope.AFourBundleSetOfTheBrawlerStrideRoundTripsAndACountOf9IsOverMax", "[WireFormat]")
{
	std::vector<std::byte> out(BrawlerStrideShape::inputBundleSetMaxBytes(simEnvelope::kMaxInputBundlesPerSet + 1u));
	size_t used = simEnvelope::writeInputBundleSetHeader(out);
	std::vector<std::vector<uint8_t>> bundles;
	for (uint8_t player = 1u; player <= 4u; ++player)
	{
		bundles.push_back(brawlerBundle(player));
		REQUIRE(bundles.back().size() == BrawlerStrideShape::kMaxBundleBytes);
		used = simEnvelope::appendInputBundle(out, used, static_cast<uint8_t>(10u + player), toBytes(bundles.back()));
		REQUIRE(used != 0u);
	}
	CHECK(used == 1391u);

	const std::vector<uint8_t> message = toU8(std::span<const std::byte>(out.data(), used));
	const Decoded decoded = decodeAll<BrawlerStrideShape>(SimLane::SimInput, kUp, message);
	REQUIRE(decoded.refusal == DecodeRefusal::None);
	CHECK(decoded.count == 4u);

	DecodeRefusal refusal = DecodeRefusal::Version;
	const std::vector<SeenBundle> seen = collectBundles<BrawlerStrideShape>(toBytes(decoded.body), refusal);
	CHECK(refusal == DecodeRefusal::None);
	REQUIRE(seen.size() == 4u);
	for (size_t i = 0u; i < 4u; ++i)
	{
		CHECK(seen[i].simId == 11u + i);
		CHECK(seen[i].bytes == bundles[i]);
	}

	for (uint8_t player = 5u; player <= 8u; ++player)
	{
		used = simEnvelope::appendInputBundle(out, used, static_cast<uint8_t>(10u + player), toBytes(brawlerBundle(player)));
		REQUIRE(used != 0u);
	}
	CHECK(used == BrawlerStrideShape::inputBundleSetMaxBytes(8u));
	CHECK(simEnvelope::appendInputBundle(out, used, 19u, toBytes(brawlerBundle(9u))) == 0u);

	std::vector<uint8_t> nine = toU8(std::span<const std::byte>(out.data(), used));
	CHECK(decodeAll<BrawlerStrideShape>(SimLane::SimInput, kUp, nine).refusal == DecodeRefusal::None);
	nine[2] = 9u;
	const std::vector<uint8_t> ninth = brawlerBundle(9u);
	nine.push_back(19u);
	nine.insert(nine.end(), ninth.begin(), ninth.end());
	CHECK(decodeAll<BrawlerStrideShape>(SimLane::SimInput, kUp, nine).refusal == DecodeRefusal::CountOverMax);
	CHECK(simEnvelope::forEachInputBundle<BrawlerStrideShape>(toBytes(std::vector<uint8_t>(nine.begin() + 2, nine.end())),
		[](uint8_t, std::span<const std::byte>) {}) == DecodeRefusal::CountOverMax);
}

TEST_CASE("SimEnvelope.ForEachInputBundleVisitsNothingInARefusedSet", "[WireFormat]")
{
	std::vector<uint8_t> body = hex("02  03 03 01 10 00 00 00 AA BB  05 03 02 11 00 00 00 01 02 12 00 00 00 03");
	DecodeRefusal refusal = DecodeRefusal::None;
	CHECK(collectBundles(toBytes(body), refusal).empty());
	CHECK(refusal == DecodeRefusal::Truncated);

	body = hex("02  03 03 01 10 00 00 00 AA BB  00 03 01 11 00 00 00 01 02");
	CHECK(collectBundles(toBytes(body), refusal).empty());
	CHECK(refusal == DecodeRefusal::SimIdZero);

	body = hex("01  03 03 01 10 00 00 00 AA BB  FF");
	CHECK(collectBundles(toBytes(body), refusal).empty());
	CHECK(refusal == DecodeRefusal::Overlong);
}

TEST_CASE("SimEnvelope.WritersReturnZeroAndWriteNothingWhenTheMessageCannotBeWritten", "[WireFormat]")
{
	const std::vector<std::byte> ring = toBytes(hex("03 01 20 00 00 00 02 CC DD"));
	const std::vector<std::byte> payload = toBytes(hex("30 00 00 00 2F 00 00 00 01 02 03 04"));
	const std::vector<std::byte> roster = toBytes(hex("01 02 06 00 03 02 01"));
	const std::vector<std::byte> bundle = toBytes(hex("03 01 10 00 00 00 AA BB"));

	std::vector<std::byte> small(ring.size() + 2u, std::byte{0x5Au});
	const std::vector<std::byte> untouched = small;
	CHECK(simEnvelope::writeRelayRing(small, 4u, ring) == 0u);
	CHECK(simEnvelope::writeRelayRing(small, 0u, ring) == 0u);
	CHECK(toU8(small) == toU8(untouched));
	std::vector<std::byte> roomy(64u);
	CHECK(simEnvelope::writeRelayRing(roomy, 4u, ring) == ring.size() + 3u);
	CHECK(simEnvelope::writeRelayRing(roomy, 4u, {}) == 0u);

	small.assign(payload.size() + 3u, std::byte{0x5Au});
	CHECK(simEnvelope::writeCorrectionState(small, 2u, 0x0Fu, payload) == 0u);
	CHECK(simEnvelope::writeCorrectionState(roomy, 0u, 0x0Fu, payload) == 0u);
	CHECK(simEnvelope::writeCorrectionState(roomy, 2u, 0x0Fu, payload) == payload.size() + 4u);

	small.assign(5u, std::byte{0x5Au});
	CHECK(simEnvelope::writeClockPublish(small, 1u) == 0u);
	small.assign(2u, std::byte{0x5Au});
	CHECK(simEnvelope::writeRelayFloor(small, 1u) == 0u);
	CHECK(simEnvelope::writeConnectionTier(small, 1u) == 0u);
	CHECK(simEnvelope::writeInputBundleSetHeader(small) == 0u);
	CHECK(simEnvelope::writeRoster(small, roster) == 0u);
	CHECK(simEnvelope::writeRoster(roomy, {}) == 0u);
	CHECK(toU8(small) == std::vector<uint8_t>(2u, 0x5Au));

	std::vector<std::byte> set(12u, std::byte{0x00u});
	size_t used = simEnvelope::writeInputBundleSetHeader(set);
	REQUIRE(used == 3u);
	CHECK(simEnvelope::appendInputBundle(set, used, 0u, bundle) == 0u);
	CHECK(simEnvelope::appendInputBundle(set, used, 3u, toBytes(hex("03"))) == 0u);
	CHECK(simEnvelope::appendInputBundle(set, 2u, 3u, bundle) == 0u);
	CHECK(simEnvelope::appendInputBundle(set, 13u, 3u, bundle) == 0u);
	used = simEnvelope::appendInputBundle(set, used, 3u, bundle);
	CHECK(used == 12u);
	const std::vector<std::byte> full = set;
	CHECK(simEnvelope::appendInputBundle(set, used, 5u, bundle) == 0u);
	CHECK(toU8(set) == toU8(full));

	std::vector<std::byte> notASet = roomy;
	const size_t ringBytes = simEnvelope::writeRelayRing(notASet, 4u, ring);
	CHECK(simEnvelope::appendInputBundle(notASet, ringBytes, 3u, bundle) == 0u);
}

#endif
