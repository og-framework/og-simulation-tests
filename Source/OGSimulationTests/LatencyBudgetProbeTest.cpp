// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/LatencyBudgetProbe.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
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

TEST_CASE("LatencyBudgetProbe.LateHealHopH6L", "[LatencyProbe]")
{
    LatencyBudgetProbe probe(kClientHops, smallConfig());
    REQUIRE(probe.isEnabled(Hop::H6L));
    REQUIRE_FALSE(LatencyBudgetProbe(kServerHops, smallConfig()).isEnabled(Hop::H6L));

    SECTION("a resim-served stamp closes H6L and leaves H6 open")
    {
        probe.stamp(Stream::Input, kLane, 5u, Point::ClientReceived, 1.000);
        probe.apply(Event::stampHealedIfPending(kLane, 5u, 1.040));
        probe.forgetLane(kLane);

        const WindowReport report = closeNow(probe);
        const HopSummary* h6l = findHop(report, Hop::H6L);
        REQUIRE(h6l != nullptr);
        CHECK(h6l->n == 1u);
        CHECK(h6l->p50 == Catch::Approx(40 * kMs));
        CHECK(h6l->healedByUse == 1u);
        CHECK(h6l->healedByCorrection == 0u);
        CHECK(h6l->noEnd == 0u);
        CHECK(findHop(report, Hop::H6)->n == 0u);
        CHECK(findHop(report, Hop::H6)->noEnd == 1u);
        CHECK(report.duplicateStamps == 0u);
    }

    SECTION("a prediction Hit closes both H6 and H6L")
    {
        probe.stamp(Stream::Input, kLane, 5u, Point::ClientReceived, 1.000);
        probe.apply(Event::stamp(Stream::Input, kLane, 5u, Point::Consumed, 1.012));
        probe.apply(Event::stamp(Stream::Input, kLane, 5u, Point::Healed, 1.012));
        probe.apply(Event::stampHealedIfPending(kLane, 5u, 1.050));
        probe.forgetLane(kLane);

        const WindowReport report = closeNow(probe);
        const HopSummary* h6l = findHop(report, Hop::H6L);
        CHECK(findHop(report, Hop::H6)->n == 1u);
        CHECK(findHop(report, Hop::H6)->p50 == Catch::Approx(12 * kMs));
        CHECK(h6l->n == 1u);
        CHECK(h6l->p50 == Catch::Approx(12 * kMs));
        CHECK(h6l->healedByUse == 1u);
        CHECK(report.duplicateStamps == 0u);
    }

    SECTION("a correction at tick X closes H6L for every pending capture <= X - dA and for none above it")
    {
        for (uint32_t capture = 10u; capture <= 14u; ++capture)
            probe.stamp(Stream::Input, kLane, capture, Point::ClientReceived, 1.000 + capture * kMs);
        probe.stamp(Stream::Input, kLane, 10u, Point::Healed, 1.015);
        probe.stamp(Stream::Input, 11u, 12u, Point::ClientReceived, 1.000);

        constexpr uint32_t kCorrectionTick = 18u;
        constexpr uint32_t kDelayTicks     = 6u;
        probe.stampHealedByCorrection(kLane, kCorrectionTick, kDelayTicks, 1.100);
        probe.forgetLane(kLane);
        probe.forgetLane(11u);

        const WindowReport report = closeNow(probe);
        const HopSummary* h6l = findHop(report, Hop::H6L);
        CHECK(h6l->n == 3u);
        CHECK(h6l->healedByUse == 1u);
        CHECK(h6l->healedByCorrection == 2u);
        CHECK(h6l->healedByUse + h6l->healedByCorrection == h6l->n);
        CHECK(h6l->p50 == Catch::Approx(88 * kMs));
        CHECK(h6l->max == Catch::Approx(89 * kMs));
        CHECK(h6l->noEnd == 3u);
    }

    SECTION("a correction older than the delay stamp heals nothing")
    {
        probe.stamp(Stream::Input, kLane, 2u, Point::ClientReceived, 1.000);
        probe.stampHealedByCorrection(kLane, 5u, 6u, 1.100);
        probe.forgetLane(kLane);

        const WindowReport report = closeNow(probe);
        CHECK(findHop(report, Hop::H6L)->n == 0u);
        CHECK(findHop(report, Hop::H6L)->noEnd == 1u);
    }

    SECTION("a late capture with no later use or correction stays noEnd")
    {
        probe.stamp(Stream::Input, kLane, 20u, Point::ClientReceived, 1.000);
        probe.stampHealedByCorrection(kLane, 25u, 6u, 1.100);
        probe.apply(Event::stampHealedIfPending(kLane, 21u, 1.120));
        probe.forgetLane(kLane);

        const WindowReport report = closeNow(probe);
        const HopSummary* h6l = findHop(report, Hop::H6L);
        CHECK(h6l->n == 0u);
        CHECK(h6l->noEnd == 1u);
        CHECK(h6l->healedByUse == 0u);
        CHECK(h6l->healedByCorrection == 0u);
    }

    SECTION("a heal stamp never starts a hop: no ClientReceived, or a receipt after it, heals nothing")
    {
        probe.apply(Event::stampHealedIfPending(kLane, 3u, 1.000));
        probe.stamp(Stream::Input, kLane, 4u, Point::ClientReceived, 1.200);
        probe.stampHealedByCorrection(kLane, 30u, 6u, 1.100);
        probe.forgetLane(kLane);

        const WindowReport report = closeNow(probe);
        CHECK(findHop(report, Hop::H6L)->n == 0u);
        CHECK(findHop(report, Hop::H6L)->noStart == 0u);
        CHECK(findHop(report, Hop::H6L)->noEnd == 1u);
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

namespace
{
    std::vector<std::string> reportedHopNames(const WindowReport& report)
    {
        std::vector<std::string> names;
        for (std::size_t index = 0; index < report.hopCount; ++index)
            names.emplace_back(hopName(report.hops[index].hop));
        return names;
    }

    struct NetHopCase
    {
        const char* label;
        HopMask     mask;
        Hop         hop;
        Stream      stream;
        Point       end;
    };

    const NetHopCase kNetHopCases[] = {
        { "N4",  kServerHops | kServerNetHops, Hop::N4,  Stream::Input, Point::ServerReceived },
        { "N6",  kClientHops | kClientNetHops, Hop::N6,  Stream::Input, Point::ClientReceived },
        { "N6c", kClientHops | kClientNetHops, Hop::N6c, Stream::State, Point::ClientReceived },
    };
}

TEST_CASE("LatencyBudgetProbe.NetHopNamesAndRoutes", "[LatencyProbe]")
{
    CHECK(std::string(hopName(Hop::N4)) == "N4");
    CHECK(std::string(hopName(Hop::N6)) == "N6");
    CHECK(std::string(hopName(Hop::N6c)) == "N6c");
    CHECK(std::string(hopName(Hop::N6u)) == "N6u");
    CHECK(std::string(hopName(Hop::Wire)) == "WIRE");
    CHECK(static_cast<std::size_t>(Point::NetReceived) + 1u == kPointCount);
    CHECK(static_cast<std::size_t>(Hop::Wire) + 1u == kHopCount);
    CHECK(kServerNetHops == hopBit(Hop::N4));
    CHECK(kClientNetHops == (hopBit(Hop::N6) | hopBit(Hop::N6c) | hopBit(Hop::N6u)));
    CHECK((kClientHops & (kServerNetHops | kClientNetHops)) == 0u);
    CHECK((kServerHops & (kServerNetHops | kClientNetHops)) == 0u);
}

TEST_CASE("LatencyBudgetProbe.NetHopIsTheDrainResidence", "[LatencyProbe]")
{
    for (const NetHopCase& netCase : kNetHopCases)
    {
        DYNAMIC_SECTION(netCase.label << ": NetReceived then the decode stamp")
        {
            LatencyBudgetProbe probe(netCase.mask, smallConfig());
            probe.stamp(netCase.stream, kLane, 12u, Point::NetReceived, 2.000);
            probe.stamp(netCase.stream, kLane, 12u, netCase.end, 2.0135);

            const WindowReport report = closeNow(probe);
            const HopSummary* net = findHop(report, netCase.hop);
            REQUIRE(net != nullptr);
            CHECK(net->n == 1u);
            CHECK(net->p50 == Catch::Approx(13.5 * kMs));
            CHECK(net->outOfOrder == 0u);
        }
        DYNAMIC_SECTION(netCase.label << ": the decode stamp lands first")
        {
            LatencyBudgetProbe probe(netCase.mask, smallConfig());
            probe.stamp(netCase.stream, kLane, 12u, netCase.end, 2.0135);
            probe.stamp(netCase.stream, kLane, 12u, Point::NetReceived, 2.000);

            const WindowReport report = closeNow(probe);
            const HopSummary* net = findHop(report, netCase.hop);
            REQUIRE(net != nullptr);
            CHECK(net->n == 1u);
            CHECK(net->p50 == Catch::Approx(13.5 * kMs));
        }
        DYNAMIC_SECTION(netCase.label << ": through apply(Event::stamp)")
        {
            LatencyBudgetProbe probe(netCase.mask, smallConfig());
            probe.apply(Event::stamp(netCase.stream, kLane, 12u, netCase.end, 2.0135));
            probe.apply(Event::stamp(netCase.stream, kLane, 12u, Point::NetReceived, 2.000));

            const WindowReport report = closeNow(probe);
            const HopSummary* net = findHop(report, netCase.hop);
            REQUIRE(net != nullptr);
            CHECK(net->n == 1u);
            CHECK(net->p50 == Catch::Approx(13.5 * kMs));
        }
        DYNAMIC_SECTION(netCase.label << ": NetReceived posted from another thread and drained on the owner")
        {
            LatencyBudgetProbe probe(netCase.mask, smallConfig());
            SpscMailbox<Event, 8> mailbox;
            const NetHopCase posted = netCase;
            bool pushed = false;
            std::thread producer([&mailbox, &pushed, posted] {
                pushed = mailbox.tryPush(Event::stamp(posted.stream, kLane, 12u, Point::NetReceived, 2.000));
            });
            producer.join();
            REQUIRE(pushed);
            probe.stamp(netCase.stream, kLane, 12u, netCase.end, 2.0135);
            CHECK(mailbox.drain([&](const Event& event) { probe.apply(event); }) == 1u);
            CHECK(mailbox.takeDroppedCount() == 0u);

            const WindowReport report = closeNow(probe);
            const HopSummary* net = findHop(report, netCase.hop);
            REQUIRE(net != nullptr);
            CHECK(net->n == 1u);
            CHECK(net->p50 == Catch::Approx(13.5 * kMs));
            CHECK(report.duplicateStamps == 0u);
        }
    }
}

TEST_CASE("LatencyBudgetProbe.DefaultMasksReportTodaysHops", "[LatencyProbe]")
{
    SECTION("client")
    {
        LatencyBudgetProbe probe(kClientHops, smallConfig());
        CHECK(reportedHopNames(closeNow(probe))
            == std::vector<std::string>{ "H1", "H2", "H3", "H6", "H6c", "H7", "H6L", "WIRE" });
    }
    SECTION("server")
    {
        LatencyBudgetProbe probe(kServerHops, smallConfig());
        CHECK(reportedHopNames(closeNow(probe))
            == std::vector<std::string>{ "H3", "H3c", "H4", "H4r", "H5", "H7", "WIRE" });
    }
    SECTION("the net masks add their hops before WIRE, in Hop order")
    {
        LatencyBudgetProbe client(kClientHops | kClientNetHops, smallConfig());
        CHECK(reportedHopNames(closeNow(client))
            == std::vector<std::string>{ "H1", "H2", "H3", "H6", "H6c", "H7", "H6L", "N6", "N6c", "N6u", "WIRE" });
        LatencyBudgetProbe server(kServerHops | kServerNetHops, smallConfig());
        CHECK(reportedHopNames(closeNow(server))
            == std::vector<std::string>{ "H3", "H3c", "H4", "H4r", "H5", "H7", "N4", "WIRE" });
    }
}

TEST_CASE("LatencyBudgetProbe.NetHopsCountMissingStampsOnlyWhenEnabled", "[LatencyProbe]")
{
    const auto stampUnpaired = [](LatencyBudgetProbe& probe) {
        probe.stamp(Stream::Input, kLane, 1u, Point::NetReceived, 1.000);
        probe.stamp(Stream::Input, kLane, 2u, Point::ServerReceived, 1.010);
        probe.stamp(Stream::Input, kLane, 3u, Point::ClientReceived, 1.020);
        probe.stamp(Stream::State, kLane, 4u, Point::NetReceived, 1.030);
        probe.stamp(Stream::State, kLane, 5u, Point::ClientReceived, 1.040);
        probe.forgetLane(kLane);
    };

    SECTION("enabled: each net hop counts its own unpaired starts and ends")
    {
        LatencyBudgetProbe probe(kServerNetHops | kClientNetHops, smallConfig());
        stampUnpaired(probe);

        const WindowReport report = closeNow(probe);
        REQUIRE(reportedHopNames(report) == std::vector<std::string>{ "N4", "N6", "N6c", "N6u" });
        const HopSummary* n4 = findHop(report, Hop::N4);
        CHECK(n4->n == 0u);
        CHECK(n4->noEnd == 1u);
        CHECK(n4->noStart == 1u);
        const HopSummary* n6 = findHop(report, Hop::N6);
        CHECK(n6->n == 0u);
        CHECK(n6->noEnd == 1u);
        CHECK(n6->noStart == 1u);
        const HopSummary* n6c = findHop(report, Hop::N6c);
        CHECK(n6c->n == 0u);
        CHECK(n6c->noEnd == 1u);
        CHECK(n6c->noStart == 1u);
        const HopSummary* n6u = findHop(report, Hop::N6u);
        CHECK(n6u->n == 0u);
        CHECK(n6u->noEnd == 1u);
        CHECK(n6u->noStart == 0u);
    }

    SECTION("one net hop enabled: the others count nothing")
    {
        LatencyBudgetProbe probe(kServerNetHops, smallConfig());
        stampUnpaired(probe);

        const WindowReport report = closeNow(probe);
        REQUIRE(reportedHopNames(report) == std::vector<std::string>{ "N4" });
        CHECK(findHop(report, Hop::N4)->noEnd == 1u);
        CHECK(findHop(report, Hop::N4)->noStart == 1u);
    }

    SECTION("default masks: a NetReceived stamp changes no reported counter")
    {
        LatencyBudgetProbe withNet(kServerHops, smallConfig());
        stampUnpaired(withNet);
        LatencyBudgetProbe withoutNet(kServerHops, smallConfig());
        withoutNet.stamp(Stream::Input, kLane, 2u, Point::ServerReceived, 1.010);
        withoutNet.stamp(Stream::Input, kLane, 3u, Point::ClientReceived, 1.020);
        withoutNet.stamp(Stream::State, kLane, 5u, Point::ClientReceived, 1.040);
        withoutNet.forgetLane(kLane);

        const WindowReport with    = closeNow(withNet);
        const WindowReport without = closeNow(withoutNet);
        REQUIRE(reportedHopNames(with) == reportedHopNames(without));
        for (std::size_t index = 0; index < with.hopCount; ++index)
        {
            CHECK(with.hops[index].n == without.hops[index].n);
            CHECK(with.hops[index].noStart == without.hops[index].noStart);
            CHECK(with.hops[index].noEnd == without.hops[index].noEnd);
        }
        CHECK(findHop(with, Hop::H4)->noEnd == 1u);
        CHECK(findHop(with, Hop::N4) == nullptr);
    }
}

TEST_CASE("LatencyBudgetProbe.N6uIsEachRelayedInputsArrivalToItsUse", "[LatencyProbe]")
{
    const HopMask clientMask = kClientHops | kClientNetHops;

    SECTION("per input, N6u is N6 + H6 exactly: og-net arrival -> the drain -> the use step")
    {
        LatencyBudgetProbe probe(clientMask, smallConfig());
        const double arrivals[] = { 2.000, 2.010, 2.012 };
        const double drains[]   = { 2.008, 2.017, 2.017 };
        const double uses[]     = { 2.030, 2.050, 2.067 };
        for (uint32_t index = 0; index < 3u; ++index)
        {
            const uint32_t tick = 20u + index;
            probe.stamp(Stream::Input, kLane, tick, Point::NetReceived, arrivals[index]);
            probe.stamp(Stream::Input, kLane, tick, Point::ClientReceived, drains[index]);
            probe.stamp(Stream::Input, kLane, tick, Point::Consumed, uses[index]);
        }

        const WindowReport report = closeNow(probe);
        const HopSummary* n6u = findHop(report, Hop::N6u);
        REQUIRE(n6u != nullptr);
        CHECK(n6u->n == 3u);
        CHECK(n6u->p50 == Catch::Approx(40.0 * kMs));
        CHECK(n6u->max == Catch::Approx(55.0 * kMs));
        CHECK(findHop(report, Hop::N6)->p50 == Catch::Approx(7.0 * kMs));
        CHECK(findHop(report, Hop::H6)->p50 == Catch::Approx(33.0 * kMs));
        CHECK(n6u->noStart == 0u);
        CHECK(n6u->noEnd == 0u);
        CHECK(n6u->outOfOrder == 0u);
    }

    SECTION("the first arrival and the first use count; a redundant copy or a re-read does not move either")
    {
        LatencyBudgetProbe probe(clientMask, smallConfig());
        probe.stamp(Stream::Input, kLane, 30u, Point::NetReceived, 3.000);
        probe.stamp(Stream::Input, kLane, 30u, Point::NetReceived, 3.016);
        probe.stamp(Stream::Input, kLane, 30u, Point::Consumed, 3.050);
        probe.stamp(Stream::Input, kLane, 30u, Point::Consumed, 3.067);

        const WindowReport report = closeNow(probe);
        const HopSummary* n6u = findHop(report, Hop::N6u);
        REQUIRE(n6u != nullptr);
        CHECK(n6u->n == 1u);
        CHECK(n6u->p50 == Catch::Approx(50.0 * kMs));
        CHECK(report.duplicateStamps == 2u);
    }

    SECTION("the use stamp may be applied before the arrival (two mailboxes): the sample is the same")
    {
        LatencyBudgetProbe probe(clientMask, smallConfig());
        probe.apply(Event::stamp(Stream::Input, kLane, 40u, Point::Consumed, 4.040));
        probe.apply(Event::stamp(Stream::Input, kLane, 40u, Point::NetReceived, 4.000));

        const WindowReport report = closeNow(probe);
        const HopSummary* n6u = findHop(report, Hop::N6u);
        REQUIRE(n6u != nullptr);
        CHECK(n6u->n == 1u);
        CHECK(n6u->p50 == Catch::Approx(40.0 * kMs));
    }

    SECTION("a local input (no og-net arrival) and a correction are not N6u samples")
    {
        LatencyBudgetProbe probe(clientMask, smallConfig());
        probe.stamp(Stream::Input, kLane, 50u, Point::Captured, 5.000);
        probe.stamp(Stream::Input, kLane, 50u, Point::Consumed, 5.001);
        probe.stamp(Stream::State, kLane, 51u, Point::NetReceived, 5.002);
        probe.stamp(Stream::State, kLane, 51u, Point::Consumed, 5.030);
        probe.forgetLane(kLane);

        const WindowReport report = closeNow(probe);
        const HopSummary* n6u = findHop(report, Hop::N6u);
        REQUIRE(n6u != nullptr);
        CHECK(n6u->n == 0u);
        CHECK(n6u->noStart == 1u);
        CHECK(n6u->noEnd == 0u);
    }

    SECTION("off in the default client mask: the window reports no N6u")
    {
        LatencyBudgetProbe probe(kClientHops, smallConfig());
        probe.stamp(Stream::Input, kLane, 60u, Point::NetReceived, 6.000);
        probe.stamp(Stream::Input, kLane, 60u, Point::Consumed, 6.040);

        const WindowReport report = closeNow(probe);
        CHECK(findHop(report, Hop::N6u) == nullptr);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
