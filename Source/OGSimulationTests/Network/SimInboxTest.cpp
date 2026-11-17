// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/SimInbox.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <span>
#include <thread>
#include <vector>

namespace
{
std::vector<std::byte> patternBytes(size_t size, uint8_t seed)
{
	std::vector<std::byte> out(size);
	for (size_t i = 0; i < size; ++i)
	{
		out[i] = static_cast<std::byte>(static_cast<uint8_t>(seed + i * 7u));
	}
	return out;
}

constexpr uint64_t checkOf(uint64_t sequence) { return ~(sequence * 0x9E3779B97F4A7C15ull); }

constexpr size_t kSequencedHeaderBytes = 16u;

size_t sequencedSize(uint64_t sequence) { return kSequencedHeaderBytes + static_cast<size_t>(sequence % 97u); }

std::array<std::byte, kMaxSimMessageBytes> sequencedBytes(uint64_t sequence)
{
	std::array<std::byte, kMaxSimMessageBytes> out{};
	const uint64_t check = checkOf(sequence);
	std::memcpy(out.data(), &sequence, sizeof(sequence));
	std::memcpy(out.data() + sizeof(sequence), &check, sizeof(check));
	for (size_t i = kSequencedHeaderBytes; i < sequencedSize(sequence); ++i)
	{
		out[i] = static_cast<std::byte>(static_cast<uint8_t>(sequence + i));
	}
	return out;
}

bool readSequenced(const InboundSimRecord& record, uint64_t& sequence)
{
	if (record.kind != InboundKind::Message || record.size < kSequencedHeaderBytes)
	{
		return false;
	}
	uint64_t check = 0u;
	std::memcpy(&sequence, record.bytes.data(), sizeof(sequence));
	std::memcpy(&check, record.bytes.data() + sizeof(sequence), sizeof(check));
	if (check != checkOf(sequence) || record.size != sequencedSize(sequence))
	{
		return false;
	}
	for (size_t i = kSequencedHeaderBytes; i < record.size; ++i)
	{
		if (record.bytes[i] != static_cast<std::byte>(static_cast<uint8_t>(sequence + i)))
		{
			return false;
		}
	}
	return true;
}

bool sameBytes(const std::vector<std::byte>& a, const std::vector<std::byte>& b)
{
	return a == b;
}

uint64_t sumOf(const std::array<uint64_t, kSimLaneCount>& perLane)
{
	uint64_t total = 0u;
	for (uint64_t value : perLane)
	{
		total += value;
	}
	return total;
}

struct SeenRecord
{
	InboundKind kind;
	SimLane lane;
	SimConnectionId conn;
	double recvSeconds;
	std::vector<std::byte> bytes;
};
} // namespace

TEST_CASE("SimInbox.MessagesAndEventsComeOutInFifoOrderWithByteExactPayloads", "[Network]")
{
	auto inbox = std::make_unique<DefaultSimInbox>();
	const std::vector<std::byte> input = patternBytes(347u, 3u);
	const std::vector<std::byte> relay = patternBytes(357u, 11u);
	const std::vector<std::byte> largest = patternBytes(kMaxSimMessageBytes, 29u);
	const std::vector<std::byte> roster = patternBytes(31u, 41u);

	inbox->pushEvent(InboundKind::Connected, 7u, 1.0);
	REQUIRE(inbox->pushMessage(7u, SimLane::SimInput, 1.25, input));
	REQUIRE(inbox->pushMessage(9u, SimLane::SimRelay, 1.5, relay));
	REQUIRE(inbox->pushMessage(7u, SimLane::Control, 1.75, std::span<const std::byte>()));
	inbox->pushEvent(InboundKind::Disconnected, 9u, 2.0);
	REQUIRE(inbox->pushMessage(7u, SimLane::SimState, 2.25, largest));
	REQUIRE(inbox->pushMessage(7u, SimLane::Control, 2.5, roster));

	std::vector<SeenRecord> seen;
	const size_t count = inbox->drain([&](const InboundSimRecord& record) {
		const std::span<const std::byte> payload = record.payload();
		seen.push_back({record.kind, record.lane, record.conn, record.recvSeconds, std::vector<std::byte>(payload.begin(), payload.end())});
	});

	REQUIRE(count == 7u);
	REQUIRE(seen.size() == 7u);

	CHECK(seen[0].kind == InboundKind::Connected);
	CHECK(seen[0].conn == 7u);
	CHECK(seen[0].recvSeconds == 1.0);
	CHECK(seen[0].lane == SimLane::Control);
	CHECK(seen[0].bytes.empty());

	CHECK(seen[1].kind == InboundKind::Message);
	CHECK(seen[1].lane == SimLane::SimInput);
	CHECK(seen[1].conn == 7u);
	CHECK(seen[1].recvSeconds == 1.25);
	CHECK(sameBytes(seen[1].bytes, input));

	CHECK(seen[2].kind == InboundKind::Message);
	CHECK(seen[2].lane == SimLane::SimRelay);
	CHECK(seen[2].conn == 9u);
	CHECK(seen[2].recvSeconds == 1.5);
	CHECK(sameBytes(seen[2].bytes, relay));

	CHECK(seen[3].kind == InboundKind::Message);
	CHECK(seen[3].lane == SimLane::Control);
	CHECK(seen[3].bytes.empty());

	CHECK(seen[4].kind == InboundKind::Disconnected);
	CHECK(seen[4].conn == 9u);
	CHECK(seen[4].recvSeconds == 2.0);
	CHECK(seen[4].bytes.empty());

	CHECK(seen[5].kind == InboundKind::Message);
	CHECK(seen[5].lane == SimLane::SimState);
	CHECK(seen[5].recvSeconds == 2.25);
	CHECK(sameBytes(seen[5].bytes, largest));

	CHECK(seen[6].kind == InboundKind::Message);
	CHECK(seen[6].lane == SimLane::Control);
	CHECK(seen[6].recvSeconds == 2.5);
	CHECK(sameBytes(seen[6].bytes, roster));

	const SimInboxStats stats = inbox->stats();
	CHECK(stats.pushed[static_cast<size_t>(SimLane::Control)] == 2u);
	CHECK(stats.pushed[static_cast<size_t>(SimLane::SimClock)] == 0u);
	CHECK(stats.pushed[static_cast<size_t>(SimLane::SimInput)] == 1u);
	CHECK(stats.pushed[static_cast<size_t>(SimLane::SimRelay)] == 1u);
	CHECK(stats.pushed[static_cast<size_t>(SimLane::SimState)] == 1u);
	CHECK(sumOf(stats.dropped) == 0u);
	CHECK(stats.tooLarge == 0u);
	CHECK(stats.events == 2u);
	CHECK(stats.maxOccupancy == 7u);

	CHECK(inbox->drain([](const InboundSimRecord&) {}) == 0u);
}

TEST_CASE("SimInbox.TheLast64SlotsAreReservedForConnectionEvents", "[Network]")
{
	auto inbox = std::make_unique<DefaultSimInbox>();
	REQUIRE(DefaultSimInbox::kCapacity == 2048u);
	const std::vector<std::byte> bytes = patternBytes(40u, 5u);

	size_t accepted = 0u;
	for (size_t i = 0; i < 1984u; ++i)
	{
		if (inbox->pushMessage(3u, SimLane::SimInput, 0.0, bytes))
		{
			++accepted;
		}
	}
	REQUIRE(accepted == 1984u);

	CHECK_FALSE(inbox->pushMessage(3u, SimLane::SimInput, 0.0, bytes));
	CHECK_FALSE(inbox->pushMessage(4u, SimLane::SimClock, 0.0, bytes));
	SimInboxStats stats = inbox->stats();
	CHECK(stats.pushed[static_cast<size_t>(SimLane::SimInput)] == 1984u);
	CHECK(stats.dropped[static_cast<size_t>(SimLane::SimInput)] == 1u);
	CHECK(stats.dropped[static_cast<size_t>(SimLane::SimClock)] == 1u);
	CHECK(sumOf(stats.dropped) == 2u);

	for (SimConnectionId conn = 1u; conn <= 64u; ++conn)
	{
		inbox->pushEvent((conn % 2u) == 0u ? InboundKind::Disconnected : InboundKind::Connected, conn, 1.0);
	}
	stats = inbox->stats();
	CHECK(stats.events == 64u);

	size_t messages = 0u;
	size_t events = 0u;
	SimConnectionId lastEventConn = 0u;
	bool eventsInOrder = true;
	const size_t drained = inbox->drain([&](const InboundSimRecord& record) {
		if (record.kind == InboundKind::Message)
		{
			eventsInOrder = eventsInOrder && events == 0u;
			++messages;
			return;
		}
		eventsInOrder = eventsInOrder && record.conn == lastEventConn + 1u;
		lastEventConn = record.conn;
		++events;
	});
	CHECK(drained == 2048u);
	CHECK(messages == 1984u);
	CHECK(events == 64u);
	CHECK(eventsInOrder);
	CHECK(inbox->stats().maxOccupancy == 2048u);

	CHECK(inbox->pushMessage(3u, SimLane::SimInput, 0.0, bytes));
}

TEST_CASE("SimInbox.AMessageOver512BytesIsRefusedAsTooLarge", "[Network]")
{
	auto inbox = std::make_unique<DefaultSimInbox>();
	const std::vector<std::byte> over = patternBytes(kMaxSimMessageBytes + 1u, 1u);
	const std::vector<std::byte> exact = patternBytes(kMaxSimMessageBytes, 2u);

	CHECK_FALSE(inbox->pushMessage(2u, SimLane::SimRelay, 0.0, over));
	SimInboxStats stats = inbox->stats();
	CHECK(stats.tooLarge == 1u);
	CHECK(sumOf(stats.pushed) == 0u);
	CHECK(sumOf(stats.dropped) == 0u);

	CHECK(inbox->pushMessage(2u, SimLane::SimRelay, 0.0, exact));
	stats = inbox->stats();
	CHECK(stats.tooLarge == 1u);
	CHECK(stats.pushed[static_cast<size_t>(SimLane::SimRelay)] == 1u);

	size_t sizeSeen = 0u;
	CHECK(inbox->drain([&](const InboundSimRecord& record) { sizeSeen = record.size; }) == 1u);
	CHECK(sizeSeen == kMaxSimMessageBytes);
}

TEST_CASE("SimInbox.OneProducerAndOneConsumerOnTwoThreadsKeepOrderAndCountEveryRecord", "[Network]")
{
	constexpr uint64_t kRecords = 1000000u;
	auto inbox = std::make_unique<DefaultSimInbox>();
	std::atomic<bool> producerDone{false};
	uint64_t producerRefused = 0u;

	std::thread producer([&] {
		for (uint64_t sequence = 0; sequence < kRecords; ++sequence)
		{
			const std::array<std::byte, kMaxSimMessageBytes> bytes = sequencedBytes(sequence);
			const SimLane lane = static_cast<SimLane>(sequence % kSimLaneCount);
			if (!inbox->pushMessage(static_cast<SimConnectionId>(1u + sequence % 8u), lane, static_cast<double>(sequence),
					std::span<const std::byte>(bytes.data(), sequencedSize(sequence))))
			{
				++producerRefused;
			}
		}
		producerDone.store(true, std::memory_order_release);
	});

	uint64_t drained = 0u;
	uint64_t badRecords = 0u;
	uint64_t outOfOrder = 0u;
	bool haveLast = false;
	uint64_t last = 0u;
	auto consume = [&](const InboundSimRecord& record) {
		uint64_t sequence = 0u;
		if (!readSequenced(record, sequence) || record.lane != static_cast<SimLane>(sequence % kSimLaneCount)
			|| record.conn != 1u + sequence % 8u || record.recvSeconds != static_cast<double>(sequence))
		{
			++badRecords;
		}
		if (haveLast && sequence <= last)
		{
			++outOfOrder;
		}
		haveLast = true;
		last = sequence;
		++drained;
	};
	for (;;)
	{
		const bool doneBefore = producerDone.load(std::memory_order_acquire);
		const size_t got = inbox->drain(consume);
		if (doneBefore && got == 0u)
		{
			break;
		}
	}
	producer.join();

	const SimInboxStats stats = inbox->stats();
	CHECK(badRecords == 0u);
	CHECK(outOfOrder == 0u);
	CHECK(drained + producerRefused == kRecords);
	CHECK(drained + sumOf(stats.dropped) == kRecords);
	CHECK(sumOf(stats.pushed) == drained);
	CHECK(sumOf(stats.dropped) == producerRefused);
	CHECK(stats.tooLarge == 0u);
	CHECK(stats.events == 0u);
	CHECK(stats.maxOccupancy <= 2048u);
	CHECK(drained > 0u);
}

TEST_CASE("SimInbox.ConsumersAlternatingBetweenTwoThreadsAcrossDrainsSeeEveryRecordOnce", "[Network]")
{
	constexpr uint64_t kRecords = 200000u;
	auto inbox = std::make_unique<DefaultSimInbox>();
	std::atomic<bool> producerDone{false};
	bool producerGaveUp = false;
	const auto giveUpAt = std::chrono::steady_clock::now() + std::chrono::seconds(30);

	std::thread producer([&] {
		for (uint64_t sequence = 0; sequence < kRecords && !producerGaveUp; ++sequence)
		{
			const std::array<std::byte, kMaxSimMessageBytes> bytes = sequencedBytes(sequence);
			while (!inbox->pushMessage(5u, SimLane::SimRelay, 0.0, std::span<const std::byte>(bytes.data(), sequencedSize(sequence))))
			{
				if (std::chrono::steady_clock::now() > giveUpAt)
				{
					producerGaveUp = true;
					break;
				}
				std::this_thread::yield();
			}
		}
		producerDone.store(true, std::memory_order_release);
	});

	std::vector<uint32_t> timesSeen(kRecords, 0u);
	uint64_t badRecords = 0u;
	uint64_t outOfOrder = 0u;
	bool haveLast = false;
	uint64_t last = 0u;
	std::array<uint64_t, 2> drainsPerThread{};
	std::array<uint64_t, 2> recordsPerThread{};
	std::array<std::thread::id, 2> threadIds{};
	size_t turn = 0u;
	for (;;)
	{
		const bool doneBefore = producerDone.load(std::memory_order_acquire);
		size_t got = 0u;
		std::thread consumer([&] {
			threadIds[turn] = std::this_thread::get_id();
			got = inbox->drain([&](const InboundSimRecord& record) {
				uint64_t sequence = 0u;
				if (!readSequenced(record, sequence) || sequence >= kRecords)
				{
					++badRecords;
					return;
				}
				if (haveLast && sequence <= last)
				{
					++outOfOrder;
				}
				haveLast = true;
				last = sequence;
				++timesSeen[sequence];
			});
		});
		consumer.join();
		++drainsPerThread[turn];
		recordsPerThread[turn] += got;
		turn ^= 1u;
		if (doneBefore && got == 0u)
		{
			break;
		}
	}
	producer.join();

	REQUIRE_FALSE(producerGaveUp);
	uint64_t seenOnce = 0u;
	uint64_t seenOther = 0u;
	for (uint32_t times : timesSeen)
	{
		(times == 1u ? seenOnce : seenOther) += 1u;
	}
	CHECK(badRecords == 0u);
	CHECK(outOfOrder == 0u);
	CHECK(seenOnce == kRecords);
	CHECK(seenOther == 0u);
	CHECK(recordsPerThread[0] + recordsPerThread[1] == kRecords);
	CHECK(drainsPerThread[0] >= 1u);
	CHECK(drainsPerThread[1] >= 1u);
	CHECK(recordsPerThread[0] > 0u);
	CHECK(recordsPerThread[1] > 0u);
	CHECK(threadIds[0] != threadIds[1]);
	CHECK(sumOf(inbox->stats().pushed) == kRecords);
}

TEST_CASE("SimInbox.Death.AnEventIntoAFullInboxFiresTheCheck", "[.][OGSimDeath]")
{
	auto inbox = std::make_unique<DefaultSimInbox>();
	const std::vector<std::byte> bytes = patternBytes(8u, 1u);
	for (size_t i = 0; i < 1984u; ++i)
	{
		REQUIRE(inbox->pushMessage(1u, SimLane::SimInput, 0.0, bytes));
	}
	for (SimConnectionId conn = 1u; conn <= 64u; ++conn)
	{
		inbox->pushEvent(InboundKind::Connected, conn, 0.0);
	}
	REQUIRE(inbox->stats().events == 64u);
	inbox->pushEvent(InboundKind::Disconnected, 65u, 0.0);
	FAIL("SimInbox::pushEvent into a full inbox did NOT fire its OG_CHECK");
}

TEST_CASE("SimInbox.Death.TwoOverlappingDrainsFireTheCheck", "[.][OGSimDeath]")
{
	auto inbox = std::make_unique<DefaultSimInbox>();
	const std::vector<std::byte> bytes = patternBytes(8u, 1u);
	REQUIRE(inbox->pushMessage(1u, SimLane::SimInput, 0.0, bytes));
	REQUIRE(inbox->pushMessage(1u, SimLane::SimInput, 0.0, bytes));
	inbox->drain([&](const InboundSimRecord&) {
		inbox->drain([](const InboundSimRecord&) {});
	});
	FAIL("a drain overlapping another drain did NOT fire SimInbox's single-consumer OG_CHECK");
}

#endif
