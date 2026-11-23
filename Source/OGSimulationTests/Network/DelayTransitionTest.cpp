// SPDX-License-Identifier: MPL-2.0
// docs/DelayTransitionTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/Network/ReplicatedTierConsumer.h"
#include "OGSimulation/Network/ServerReceptionCoordinator.h"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "MockSimulatablesForNetworkTests.h"
#include "StandaloneTestHandle.h"

namespace delayTransitionTest
{
    using Coordinator = ServerReceptionCoordinator<FStandaloneTestHandle, MockSimA>;

    constexpr unsigned int kSenderId      = 7u;
    constexpr std::uint8_t kPlayerSlot    = 0u;
    constexpr std::int32_t kLeadTicks     = 2;
    constexpr std::int32_t kUplinkTicks   = 1;
    constexpr std::int32_t kDownlinkTicks = 3;
    constexpr std::int32_t kSwitchTick    = 100;
    constexpr std::int32_t kLastTick      = kSwitchTick + 700;
    constexpr double       kNoRttReading  = -1.0;
    constexpr double       kLoopbackRttMs = 1.0;

    struct Release
    {
        std::int32_t serverTick  = 0;
        std::int32_t captureTick = 0;
        std::int32_t late        = 0;
    };

    struct TierSink
    {
        std::vector<std::uint8_t> tiers;

        void sendConnectionTierToOwningClient(unsigned int, std::uint8_t tier) { tiers.push_back(tier); }
    };

    struct Server
    {
        TimeConfig            config;
        Coordinator           coordinator{ config };
        FStandaloneTestHandle wire{ 1u, true };
        TierSink              tierSink;
        std::vector<Release>  releases;
        std::vector<std::int32_t> emptyTicks;

        explicit Server(std::int32_t floorTicks) { config.relayDelayFloorTicks = floorTicks; }

        std::int32_t delay() const { return coordinator.delayQueue().effectiveDelay(wire); }

        void sampleRtt(std::int32_t serverTick, double rttMs)
        {
            coordinator.noteRttSample(wire, kSenderId, serverTick, rttMs, tierSink);
        }

        void receive(std::int32_t captureTick)
        {
            (void)coordinator.receiveRemoteInput<MockSimA>(kSenderId, wire, kPlayerSlot, captureTick,
                                                           static_cast<int>(captureTick));
        }

        void drain(std::int32_t serverTick)
        {
            const std::int32_t delayTicks = delay();
            const std::size_t  before     = releases.size();
            coordinator.releaseDelayedInputs<MockSimA>(serverTick, 1,
                [&](unsigned int, std::uint32_t captureTick, const int&)
                {
                    const std::int32_t capture = static_cast<std::int32_t>(captureTick);
                    releases.push_back(Release{ serverTick, capture, serverTick - (capture + delayTicks) });
                    return true;
                });
            if (releases.size() == before && !releases.empty())
                emptyTicks.push_back(serverTick);
        }
    };

    // One server tick at a time: the client's captures for that tick go up the link, the arrivals due now are
    // received, `beforeDrain` runs, then the server drains the tick.
    struct Session
    {
        Server server;
        std::map<std::int32_t, std::vector<std::int32_t>> inFlight;

        explicit Session(std::int32_t floorTicks) : server(floorTicks) {}

        void run(const std::function<std::vector<std::int32_t>(std::int32_t)>& capturesAt,
                 const std::function<void(std::int32_t)>& beforeDrain)
        {
            for (std::int32_t tick = 1; tick <= kLastTick; ++tick)
            {
                for (const std::int32_t capture : capturesAt(tick))
                    inFlight[tick + kUplinkTicks].push_back(capture);
                const auto due = inFlight.find(tick);
                if (due != inFlight.end())
                {
                    for (const std::int32_t capture : due->second)
                        server.receive(capture);
                    inFlight.erase(due);
                }
                beforeDrain(tick);
                server.drain(tick);
            }
        }

        std::size_t releasesFrom(std::int32_t serverTick, std::int32_t late) const
        {
            std::size_t n = 0u;
            for (const Release& release : server.releases)
                n += (release.serverTick >= serverTick && release.late == late) ? 1u : 0u;
            return n;
        }

        std::size_t releasesFrom(std::int32_t serverTick) const
        {
            std::size_t n = 0u;
            for (const Release& release : server.releases)
                n += release.serverTick >= serverTick ? 1u : 0u;
            return n;
        }
    };

    std::vector<std::int32_t> one(std::int32_t capture) { return { capture }; }

    TimeConfig deadBandClockConfig()
    {
        TimeConfig cfg;
        cfg.minTicksBeforeDriftCheck = 0u;
        cfg.softDriftThresholdTicks  = 1000u;
        cfg.hardResyncThresholdTicks = 5000u;
        cfg.recentreSettleSamples    = 0u;
        return cfg;
    }

    // A client whose captures come from a real ClientPredictionClock in the dead band, kLeadTicks ahead of the
    // server: one advance per server tick, a capture at the frontier unless the advance stalled.
    struct ClockClient
    {
        TimeConfig            config = deadBandClockConfig();
        NetworkTimeEstimator  estimator{ config, nullptr };
        ClientPredictionClock clock{ config, estimator, nullptr };

        ClockClient()
        {
            for (std::int32_t i = 0; i < kLeadTicks; ++i)
            {
                estimator.recordAuthorityTick(0u);
                clock.advancePrediction();
            }
        }

        std::vector<std::int32_t> capturesAt(std::int32_t serverTick)
        {
            estimator.recordAuthorityTick(static_cast<unsigned int>(serverTick));
            if (clock.advancePrediction() == ClientPredictionClock::AdvanceResult::Stall)
                return {};
            return one(static_cast<std::int32_t>(clock.getPredictionTick()));
        }
    };

    // The client's tier half: the consumer over its own TimeConfig (the floor already arrived), its last published
    // delay, and the transition it requests for each arrival, as one adapter's recompute does.
    struct TierClient
    {
        ClockClient            client;
        TimeConfig             tierConfig;
        ReplicatedTierConsumer consumer{ tierConfig };
        std::int32_t           published = 0;
        std::size_t            tiersSeen = 0u;

        explicit TierClient(std::int32_t floorTicks)
        {
            tierConfig.relayDelayFloorTicks = floorTicks;
            published = consumer.effectiveInputDelayTicks();
        }

        void onTier(std::uint8_t tier)
        {
            consumer.onReplicatedTierReceived(tier);
            const std::int32_t now = consumer.effectiveInputDelayTicks();
            client.clock.requestInputDelayTransition(now - published);
            published = now;
        }
    };

    // The tier path: the server samples the wire's RTT on every arrival (`firstReadingTick` and later carry a
    // reading, earlier ones the "no reading yet" sentinel), and each tier it publishes reaches the client
    // kDownlinkTicks later.
    void runTierSession(Session& session, TierClient& tierClient, std::int32_t firstReadingTick)
    {
        std::map<std::int32_t, std::vector<std::uint8_t>> tiersInFlight;
        session.run([&](std::int32_t tick) { return tierClient.client.capturesAt(tick); },
            [&](std::int32_t tick)
            {
                session.server.sampleRtt(tick, tick >= firstReadingTick ? kLoopbackRttMs : kNoRttReading);
                const std::vector<std::uint8_t>& sent = session.server.tierSink.tiers;
                for (; tierClient.tiersSeen < sent.size(); ++tierClient.tiersSeen)
                    tiersInFlight[tick + kDownlinkTicks].push_back(sent[tierClient.tiersSeen]);
                const auto due = tiersInFlight.find(tick);
                if (due != tiersInFlight.end())
                {
                    for (const std::uint8_t tier : due->second)
                        tierClient.onTier(tier);
                    tiersInFlight.erase(due);
                }
            });
    }
}

using namespace delayTransitionTest;

TEST_CASE("DelayTransition.TodayALoweredFloorWithAGapFreeStreamStaysOneTickLate", "[Network][DelayTransition]")
{
    Session session(6);
    session.run([](std::int32_t tick) { return one(tick + kLeadTicks); },
        [&](std::int32_t tick)
        {
            if (tick == kSwitchTick)
                session.server.config.relayDelayFloorTicks = 5;
        });

    REQUIRE(session.server.delay() == 5);
    CHECK(session.releasesFrom(1) == session.releasesFrom(1, 0) + session.releasesFrom(kSwitchTick, 1));
    CHECK(session.releasesFrom(kSwitchTick + 6) >= 600u);
    CHECK(session.releasesFrom(kSwitchTick + 6, 1) == session.releasesFrom(kSwitchTick + 6));
}

TEST_CASE("DelayTransition.OneMissingCaptureTickDrainsTheLoweredFloorsBacklog", "[Network][DelayTransition]")
{
    constexpr std::int32_t kSkipServerTick = kSwitchTick + kDownlinkTicks;
    constexpr std::int32_t kMissingCapture = kSkipServerTick + kLeadTicks;
    Session session(6);
    session.run([](std::int32_t tick) { return one(tick + kLeadTicks + (tick >= kSkipServerTick ? 1 : 0)); },
        [&](std::int32_t tick)
        {
            if (tick == kSwitchTick)
                session.server.config.relayDelayFloorTicks = 5;
        });

    CHECK(session.releasesFrom(kSwitchTick, 1) > 0u);
    CHECK(session.releasesFrom(kMissingCapture + 6) >= 600u);
    CHECK(session.releasesFrom(kMissingCapture + 6, 0) == session.releasesFrom(kMissingCapture + 6));
    CHECK(session.server.emptyTicks.empty());
}

TEST_CASE("DelayTransition.ARaisedFloorAndOneStallLeaveOneTickWithoutARelease", "[Network][DelayTransition]")
{
    constexpr std::int32_t kStallServerTick = kSwitchTick + kDownlinkTicks;
    Session session(5);
    session.run(
        [](std::int32_t tick)
        {
            if (tick == kStallServerTick)
                return std::vector<std::int32_t>{};
            return one(tick + kLeadTicks - (tick > kStallServerTick ? 1 : 0));
        },
        [&](std::int32_t tick)
        {
            if (tick == kSwitchTick)
                session.server.config.relayDelayFloorTicks = 6;
        });

    REQUIRE(session.server.delay() == 6);
    CHECK(session.server.emptyTicks == std::vector<std::int32_t>{ kSwitchTick });
    CHECK(session.releasesFrom(1, 0) == session.releasesFrom(1));
}

TEST_CASE("DelayTransition.TheClocksSkipDrainsTheLoweredFloorsBacklog", "[Network][DelayTransition]")
{
    ClockClient client;
    Session     session(6);
    session.run([&](std::int32_t tick) { return client.capturesAt(tick); },
        [&](std::int32_t tick)
        {
            if (tick == kSwitchTick)
                session.server.config.relayDelayFloorTicks = 5;
            if (tick == kSwitchTick + kDownlinkTicks)
                client.clock.requestInputDelayTransition(-1);
        });

    REQUIRE(client.clock.getDiagnostics().skipCount() == 1u);
    const std::int32_t missingCapture = static_cast<std::int32_t>(client.clock.getDiagnostics().lastSkipTick()) - 1;
    CAPTURE(missingCapture);
    CHECK(session.releasesFrom(kSwitchTick, 1) > 0u);
    CHECK(session.releasesFrom(missingCapture + 6) >= 600u);
    CHECK(session.releasesFrom(missingCapture + 6, 0) == session.releasesFrom(missingCapture + 6));
}

TEST_CASE("DelayTransition.TheFirstTierArrivalsChangeIsTheFallbacksToTheTiers", "[Network][DelayTransition]")
{
    const std::int32_t expected[4] = { -3, -2, -1, 0 };
    for (std::int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
    {
        CAPTURE(tier);
        TimeConfig             cfg;
        ReplicatedTierConsumer consumer(cfg);
        const std::int32_t     before = consumer.effectiveInputDelayTicks();
        consumer.onReplicatedTierReceived(tier);
        CHECK(consumer.effectiveInputDelayTicks() - before == expected[tier]);
    }
}

TEST_CASE("DelayTransition.ATierPathJoinBelowFloorFourEndsWithNoLastingLateness", "[Network][DelayTransition]")
{
    for (const std::int32_t floorTicks : { 3, 2 })
    {
        CAPTURE(floorTicks);
        constexpr std::int32_t kFirstReadingTick = 40;
        TierClient tierClient(floorTicks);
        Session    session(floorTicks);
        REQUIRE(session.server.delay() == 4);
        REQUIRE(tierClient.published == 4);

        runTierSession(session, tierClient, kFirstReadingTick);

        const std::int32_t lowered = 4 - floorTicks;
        CHECK(session.server.tierSink.tiers == std::vector<std::uint8_t>{ 0u });
        CHECK(session.server.delay() == floorTicks);
        CHECK(tierClient.published == session.server.delay());
        CHECK(tierClient.client.clock.getDiagnostics().skipCount() == static_cast<unsigned int>(lowered));
        CHECK(session.releasesFrom(kFirstReadingTick, lowered) > 0u);
        CHECK(session.releasesFrom(kFirstReadingTick + 100) >= 600u);
        CHECK(session.releasesFrom(kFirstReadingTick + 100, 0) == session.releasesFrom(kFirstReadingTick + 100));
    }
}

TEST_CASE("DelayTransition.ATierPathJoinWhoseFirstSampleLeadsItsInputsPaysOnlyEmptyTicks", "[Network][DelayTransition]")
{
    for (const std::int32_t floorTicks : { 3, 2 })
    {
        CAPTURE(floorTicks);
        TierClient tierClient(floorTicks);
        Session    session(floorTicks);

        runTierSession(session, tierClient, 1);

        const std::int32_t lowered = 4 - floorTicks;
        CHECK(session.server.tierSink.tiers == std::vector<std::uint8_t>{ 0u });
        CHECK(tierClient.published == session.server.delay());
        CHECK(tierClient.client.clock.getDiagnostics().skipCount() == static_cast<unsigned int>(lowered));
        CHECK(session.releasesFrom(1, 0) == session.releasesFrom(1));
        CHECK(session.server.emptyTicks.size() == static_cast<std::size_t>(lowered));
    }
}

#endif // WITH_LOW_LEVEL_TESTS
