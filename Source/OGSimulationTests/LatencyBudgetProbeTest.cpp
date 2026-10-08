// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/LatencyBudgetProbe.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <random>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// LatencyBudgetProbe: the per-hop latency instrument (og-simulationscheduler-withjolt
// task 1). Every case drives the probe with synthetic timestamps; the clock is the
// caller's, so nothing here sleeps or reads a wall clock.
// ---------------------------------------------------------------------------

using namespace latencyBudget;

namespace
{
    constexpr double kMs = 0.001;
    constexpr uint32_t kLane = 7u;

    ProbeConfig smallConfig()
    {
        ProbeConfig config;
        config.windowSeconds    = 10.0;
        config.ticksPerLane     = 8u;
        config.maxLanes         = 2u;
        config.maxSamplesPerHop = 256u;
        return config;
    }

    const HopSummary* findHop(const WindowReport& report, Hop hop)
    {
        for (std::size_t index = 0; index < report.hopCount; ++index)
        {
            if (report.hops[index].hop == hop)
                return &report.hops[index];
        }
        return nullptr;
    }

    WindowReport closeNow(LatencyBudgetProbe& probe, double start = 0.0)
    {
        WindowReport report;
        REQUIRE_FALSE(probe.closeWindowIfDue(start, report));
        REQUIRE(probe.closeWindowIfDue(start + 10.0, report));
        return report;
    }

    void stampH2(LatencyBudgetProbe& probe, uint32_t tick, double captured, double handed)
    {
        probe.stamp(Stream::Input, kLane, tick, Point::Captured, captured);
        probe.stamp(Stream::Input, kLane, tick, Point::Handed, handed);
    }
}

TEST_CASE("LatencyBudgetProbe.PercentilesOnAKnownDistribution", "[LatencyProbe]")
{
    LatencyBudgetProbe probe(kClientHops, smallConfig());
    WindowReport report;
    REQUIRE_FALSE(probe.closeWindowIfDue(0.0, report));

    std::vector<int> millis(100);
    for (int index = 0; index < 100; ++index)
        millis[static_cast<std::size_t>(index)] = index + 1;
    std::mt19937 rng(12345u);
    std::shuffle(millis.begin(), millis.end(), rng);

    for (const int value : millis)
        probe.addSample(Hop::Wire, value * kMs);

    REQUIRE(probe.closeWindowIfDue(10.0, report));
    const HopSummary* wire = findHop(report, Hop::Wire);
    REQUIRE(wire != nullptr);
    CHECK(wire->n == 100u);
    CHECK(wire->p50 == Catch::Approx(50 * kMs));
    CHECK(wire->p95 == Catch::Approx(95 * kMs));
    CHECK(wire->p99 == Catch::Approx(99 * kMs));
    CHECK(wire->max == Catch::Approx(100 * kMs));

    SECTION("nearest rank on small counts")
    {
        CHECK(nearestRankIndex(1u, 500u) == 0u);
        CHECK(nearestRankIndex(1u, 990u) == 0u);
        CHECK(nearestRankIndex(2u, 500u) == 0u);
        CHECK(nearestRankIndex(2u, 950u) == 1u);
        CHECK(nearestRankIndex(10u, 500u) == 4u);
        CHECK(nearestRankIndex(10u, 990u) == 9u);
        CHECK(nearestRankIndex(1000u, 990u) == 989u);
    }
}

TEST_CASE("LatencyBudgetProbe.StampPairsYieldTheHopDuration", "[LatencyProbe]")
{
    LatencyBudgetProbe probe(kClientHops, smallConfig());

    SECTION("stamps in hop order")
    {
        stampH2(probe, 3u, 1.000, 1.004);
    }
    SECTION("the end stamp lands first: the timestamps decide, not the arrival order")
    {
        probe.stamp(Stream::Input, kLane, 3u, Point::Handed, 1.004);
        probe.stamp(Stream::Input, kLane, 3u, Point::Captured, 1.000);
    }

    const WindowReport report = closeNow(probe);
    const HopSummary* h2 = findHop(report, Hop::H2);
    REQUIRE(h2 != nullptr);
    CHECK(h2->n == 1u);
    CHECK(h2->p50 == Catch::Approx(4 * kMs));
    CHECK(h2->noStart == 0u);
    CHECK(h2->outOfOrder == 0u);
}

TEST_CASE("LatencyBudgetProbe.WindowRollover", "[LatencyProbe]")
{
    LatencyBudgetProbe probe(kClientHops, smallConfig());
    WindowReport report;

    REQUIRE_FALSE(probe.closeWindowIfDue(100.0, report));
    probe.addSample(Hop::Wire, 10 * kMs);
    probe.addSample(Hop::Wire, 20 * kMs);
    stampH2(probe, 1u, 100.0, 100.002);

    CHECK_FALSE(probe.closeWindowIfDue(109.999, report));

    REQUIRE(probe.closeWindowIfDue(110.0, report));
    CHECK(report.windowStartSeconds == 100.0);
    CHECK(report.windowEndSeconds == 110.0);
    CHECK(findHop(report, Hop::Wire)->n == 2u);
    CHECK(findHop(report, Hop::Wire)->max == Catch::Approx(20 * kMs));
    CHECK(findHop(report, Hop::H2)->n == 1u);

    SECTION("the next window starts empty at the close time")
    {
        probe.addSample(Hop::Wire, 30 * kMs);
        CHECK_FALSE(probe.closeWindowIfDue(119.999, report));
        REQUIRE(probe.closeWindowIfDue(120.0, report));
        CHECK(report.windowStartSeconds == 110.0);
        const HopSummary* wire = findHop(report, Hop::Wire);
        CHECK(wire->n == 1u);
        CHECK(wire->p50 == Catch::Approx(30 * kMs));
        CHECK(findHop(report, Hop::H2)->n == 0u);
        CHECK(findHop(report, Hop::H2)->p99 == 0.0);
    }

    SECTION("a report lists every enabled hop, in Hop order, and nothing else")
    {
        std::vector<Hop> expected;
        for (std::size_t index = 0; index < kHopCount; ++index)
        {
            if ((kClientHops & hopBit(static_cast<Hop>(index))) != 0u)
                expected.push_back(static_cast<Hop>(index));
        }
        REQUIRE(report.hopCount == expected.size());
        for (std::size_t index = 0; index < expected.size(); ++index)
            CHECK(report.hops[index].hop == expected[index]);
        CHECK(findHop(report, Hop::H4) == nullptr);
    }
}

TEST_CASE("LatencyBudgetProbe.MissingHopIsDroppedAndCounted", "[LatencyProbe]")
{
    const ProbeConfig config = smallConfig();
    LatencyBudgetProbe probe(kClientHops, config);

    SECTION("an end with no start is never a sample; eviction counts it as noStart")
    {
        probe.stamp(Stream::Input, kLane, 5u, Point::Handed, 1.0);
        probe.stamp(Stream::Input, kLane, 5u + config.ticksPerLane, Point::Captured, 2.0);

        const WindowReport report = closeNow(probe);
        const HopSummary* h2 = findHop(report, Hop::H2);
        CHECK(h2->n == 0u);
        CHECK(h2->noStart == 1u);
        CHECK(h2->noEnd == 0u);
    }

    SECTION("a start with no end counts as noEnd when its lane is forgotten")
    {
        probe.stamp(Stream::Input, kLane, 5u, Point::Captured, 1.0);
        probe.forgetLane(kLane);

        const WindowReport report = closeNow(probe);
        CHECK(findHop(report, Hop::H2)->n == 0u);
        CHECK(findHop(report, Hop::H2)->noEnd == 1u);
    }

    SECTION("a hop the role does not measure is never counted as missing")
    {
        LatencyBudgetProbe server(kServerHops, config);
        server.stamp(Stream::Input, kLane, 5u, Point::ServerReceived, 1.0);
        server.stamp(Stream::Input, kLane, 5u, Point::Handed, 1.010);
        server.forgetLane(kLane);

        const WindowReport report = closeNow(server);
        CHECK(findHop(report, Hop::H2) == nullptr);
        const HopSummary* h4r = findHop(report, Hop::H4r);
        CHECK(h4r->n == 1u);
        CHECK(h4r->p50 == Catch::Approx(10 * kMs));
        CHECK(findHop(report, Hop::H3)->noEnd == 1u);
        CHECK(findHop(report, Hop::H4)->noEnd == 1u);
    }
}

TEST_CASE("LatencyBudgetProbe.OutOfOrderTickIsDroppedNotMisattributed", "[LatencyProbe]")
{
    const ProbeConfig config = smallConfig();
    LatencyBudgetProbe probe(kClientHops, config);

    SECTION("a late stamp for an older tick that shares a ring slot is stale, not joined")
    {
        const uint32_t newer = 3u + config.ticksPerLane;
        probe.stamp(Stream::Input, kLane, 3u, Point::Captured, 1.000);
        probe.stamp(Stream::Input, kLane, newer, Point::Captured, 2.000);
        probe.stamp(Stream::Input, kLane, 3u, Point::Handed, 1.500);
        probe.stamp(Stream::Input, kLane, newer, Point::Handed, 2.006);

        const WindowReport report = closeNow(probe);
        CHECK(report.staleStamps == 1u);
        const HopSummary* h2 = findHop(report, Hop::H2);
        REQUIRE(h2->n == 1u);
        CHECK(h2->p50 == Catch::Approx(6 * kMs));
        CHECK(h2->max == Catch::Approx(6 * kMs));
        CHECK(h2->noEnd == 1u);
    }

    SECTION("an end stamped before its start is dropped and counted")
    {
        stampH2(probe, 4u, 2.000, 1.990);

        const WindowReport report = closeNow(probe);
        const HopSummary* h2 = findHop(report, Hop::H2);
        CHECK(h2->n == 0u);
        CHECK(h2->outOfOrder == 1u);
    }

    SECTION("a negative or non-finite direct sample is dropped and counted")
    {
        probe.addSample(Hop::Wire, -0.001);
        probe.addSample(Hop::Wire, std::numeric_limits<double>::quiet_NaN());

        const WindowReport report = closeNow(probe);
        CHECK(findHop(report, Hop::Wire)->n == 0u);
        CHECK(findHop(report, Hop::Wire)->outOfOrder == 2u);
    }
}

TEST_CASE("LatencyBudgetProbe.FirstStampWins", "[LatencyProbe]")
{
    LatencyBudgetProbe probe(kClientHops, smallConfig());

    probe.stamp(Stream::Input, kLane, 9u, Point::Captured, 1.000);
    probe.stamp(Stream::Input, kLane, 9u, Point::Handed, 1.016);
    probe.stamp(Stream::Input, kLane, 9u, Point::Handed, 1.033);
    probe.stamp(Stream::Input, kLane, 9u, Point::Handed, 1.050);

    const WindowReport report = closeNow(probe);
    CHECK(report.duplicateStamps == 2u);
    const HopSummary* h2 = findHop(report, Hop::H2);
    REQUIRE(h2->n == 1u);
    CHECK(h2->p50 == Catch::Approx(16 * kMs));
}

TEST_CASE("LatencyBudgetProbe.PendingStamps", "[LatencyProbe]")
{
    LatencyBudgetProbe probe(kServerHops, smallConfig());

    SECTION("stampPending closes every open hop whose start is not later than the stamp")
    {
        probe.stamp(Stream::Input, kLane, 1u, Point::Handed, 1.000);
        probe.stamp(Stream::Input, kLane, 2u, Point::Handed, 1.002);
        probe.stamp(Stream::Input, kLane, 3u, Point::Handed, 1.009);
        probe.stampPending(Stream::Input, Point::Handed, Point::LeftProcess, 1.005);
        probe.stampPending(Stream::Input, Point::Handed, Point::LeftProcess, 1.010);

        const WindowReport report = closeNow(probe);
        const HopSummary* h3 = findHop(report, Hop::H3);
        REQUIRE(h3->n == 3u);
        CHECK(h3->max == Catch::Approx(5 * kMs));
        CHECK(h3->p50 == Catch::Approx(3 * kMs));
        CHECK(findHop(report, Hop::H3c)->n == 0u);
    }

    SECTION("stampNewestPending closes only the newest open tick of each lane")
    {
        probe.stamp(Stream::State, kLane, 1u, Point::StepEnd, 1.000);
        probe.stamp(Stream::State, kLane, 2u, Point::StepEnd, 1.016);
        probe.stamp(Stream::State, 11u, 2u, Point::StepEnd, 1.017);
        probe.stampNewestPending(Stream::State, Point::StepEnd, Point::Rendered, 1.020);
        probe.forgetLane(kLane);
        probe.forgetLane(11u);

        const WindowReport report = closeNow(probe);
        const HopSummary* h7 = findHop(report, Hop::H7);
        REQUIRE(h7->n == 2u);
        CHECK(h7->p50 == Catch::Approx(3 * kMs));
        CHECK(h7->max == Catch::Approx(4 * kMs));
        CHECK(h7->noEnd == 1u);
    }

    SECTION("Event::apply routes each kind to its method")
    {
        probe.apply(Event::stamp(Stream::State, kLane, 4u, Point::StepEnd, 2.000));
        probe.apply(Event::stamp(Stream::State, kLane, 4u, Point::Handed, 2.007));
        probe.apply(Event::stampPending(Stream::State, Point::Handed, Point::LeftProcess, 2.008));
        probe.apply(Event::sample(Hop::Wire, 0.025));

        const WindowReport report = closeNow(probe);
        CHECK(findHop(report, Hop::H5)->p50 == Catch::Approx(7 * kMs));
        CHECK(findHop(report, Hop::H3c)->p50 == Catch::Approx(1 * kMs));
        CHECK(findHop(report, Hop::Wire)->p50 == Catch::Approx(25 * kMs));
    }
}

TEST_CASE("LatencyBudgetProbe.LanesAndCapacity", "[LatencyProbe]")
{
    ProbeConfig config = smallConfig();
    config.maxSamplesPerHop = 3u;
    LatencyBudgetProbe probe(kClientHops, config);

    SECTION("lanes beyond maxLanes are refused and counted, and a forgotten lane is reused")
    {
        stampH2(probe, 1u, 1.0, 1.001);
        probe.stamp(Stream::Input, 20u, 1u, Point::Captured, 1.0);
        probe.stamp(Stream::Input, 30u, 1u, Point::Captured, 1.0);
        probe.forgetLane(20u);
        probe.stamp(Stream::Input, 30u, 1u, Point::Captured, 1.0);
        probe.stamp(Stream::Input, 30u, 1u, Point::Handed, 1.002);

        const WindowReport report = closeNow(probe);
        CHECK(report.laneOverflow == 1u);
        CHECK(findHop(report, Hop::H2)->n == 2u);
        CHECK(findHop(report, Hop::H2)->noEnd == 1u);
    }

    SECTION("samples beyond maxSamplesPerHop are counted, not stored")
    {
        for (int index = 1; index <= 5; ++index)
            probe.addSample(Hop::Wire, index * kMs);

        const WindowReport report = closeNow(probe);
        CHECK(findHop(report, Hop::Wire)->n == 3u);
        CHECK(findHop(report, Hop::Wire)->overflow == 2u);
        CHECK(findHop(report, Hop::Wire)->max == Catch::Approx(3 * kMs));
    }
}

TEST_CASE("LatencyBudgetProbe.SpscMailbox", "[LatencyProbe]")
{
    SECTION("drains in push order and counts what a full mailbox refused")
    {
        SpscMailbox<uint32_t, 4> mailbox;
        for (uint32_t value = 0; value < 6u; ++value)
            mailbox.tryPush(value);

        std::vector<uint32_t> drained;
        CHECK(mailbox.drain([&](uint32_t value) { drained.push_back(value); }) == 4u);
        CHECK(drained == std::vector<uint32_t>{ 0u, 1u, 2u, 3u });
        CHECK(mailbox.takeDroppedCount() == 2u);
        CHECK(mailbox.takeDroppedCount() == 0u);
        CHECK(mailbox.tryPush(9u));
        CHECK(mailbox.drain([&](uint32_t value) { drained.push_back(value); }) == 1u);
        CHECK(drained.back() == 9u);
    }

    SECTION("one producer thread and one consumer thread lose and reorder nothing")
    {
        static SpscMailbox<Event, 64> mailbox;
        constexpr uint32_t kCount = 20000u;

        std::thread producer([] {
            for (uint32_t tick = 0; tick < kCount; ++tick)
            {
                while (!mailbox.tryPush(Event::stamp(Stream::Input, 1u, tick, Point::Captured, 0.0)))
                    std::this_thread::yield();
            }
        });

        uint32_t expected = 0u;
        bool inOrder = true;
        while (expected < kCount)
        {
            mailbox.drain([&](const Event& event) {
                inOrder = inOrder && event.tick == expected;
                ++expected;
            });
        }
        producer.join();

        CHECK(inOrder);
        CHECK(expected == kCount);
        mailbox.takeDroppedCount();
    }
}

TEST_CASE("LatencyBudgetProbe.ScalarWindowStats", "[LatencyProbe]")
{
    ScalarWindowStats stats;
    CHECK(stats.mean() == 0.0);
    stats.add(1.0);
    stats.add(0.75);
    stats.add(1.25);
    stats.add(std::numeric_limits<double>::infinity());
    CHECK(stats.samples == 3u);
    CHECK(stats.min == 0.75);
    CHECK(stats.max == 1.25);
    CHECK(stats.mean() == Catch::Approx(1.0));
    stats.reset();
    CHECK(stats.samples == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS
