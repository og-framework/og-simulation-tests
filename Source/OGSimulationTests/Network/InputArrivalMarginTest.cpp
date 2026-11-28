// SPDX-License-Identifier: MPL-2.0
// docs/InputArrivalMarginTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/Network/InputArrivalMargin.h"
#include "OGSimulation/Network/ServerReceptionCoordinator.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "LinkModel.h"
#include "MockSimulatablesForNetworkTests.h"
#include "StandaloneTestHandle.h"

namespace arrivalMarginTest
{
    struct BundleInput
    {
        std::int32_t captureTick = 0;
    };
}

template <>
struct SerializableFields<arrivalMarginTest::BundleInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(arrivalMarginTest::BundleInput, captureTick));
    }
};

namespace arrivalMarginTest
{
    using linkModel::LinkModel;
    using Coordinator = ServerReceptionCoordinator<FStandaloneTestHandle, MockSimA>;
    using Margin      = InputArrivalMargin<FStandaloneTestHandle>;

    constexpr unsigned int  kSenderId   = 7u;
    constexpr std::uint8_t  kPlayerSlot = 0u;
    constexpr std::int32_t  kDelayTicks = 6;
    constexpr std::int32_t  kNoLeadStep = -1;

    TimeConfig makeConfig()
    {
        TimeConfig config;
        config.relayDelayFloorTicks = kDelayTicks;
        return config;
    }

    TimeConfig makeUniformDelayConfig(std::int32_t delayTicks)
    {
        TimeConfig config;
        config.relayDelayFloorTicks = 0;
        for (std::int32_t& tierDelay : config.rttTierInputDelays)
        {
            tierDelay = delayTicks;
        }
        return config;
    }

    LinkModel::Payload encodeTicks(const std::vector<std::int32_t>& ticks)
    {
        LinkModel::Payload bytes(ticks.size() * sizeof(std::int32_t));
        if (!ticks.empty())
        {
            std::memcpy(bytes.data(), ticks.data(), bytes.size());
        }
        return bytes;
    }

    std::vector<std::int32_t> decodeTicks(const LinkModel::Payload& bytes)
    {
        std::vector<std::int32_t> ticks(bytes.size() / sizeof(std::int32_t));
        if (!ticks.empty())
        {
            std::memcpy(ticks.data(), bytes.data(), ticks.size() * sizeof(std::int32_t));
        }
        return ticks;
    }

    std::int32_t nearestRank(std::vector<std::int32_t> values, std::uint32_t percent)
    {
        std::sort(values.begin(), values.end());
        const std::size_t rank = std::max<std::size_t>((percent * values.size() + 99u) / 100u, 1u);
        return values[rank - 1u];
    }

    struct Scenario
    {
        std::int32_t  numSteps        = 1;
        std::int32_t  leadTicks       = 2;
        std::int32_t  redundancyDepth = 1;
        std::int32_t  firstSendTick   = 0;
        std::int32_t  lastServerTick  = 0;
        std::int32_t  leadStepTick    = kNoLeadStep;
        std::int32_t  leadAfterStep   = 0;
        linkModel::Config link;
        std::function<double(std::int32_t)> extraDelayAt;

        std::int32_t leadAt(std::int32_t tick) const
        {
            return (leadStepTick != kNoLeadStep && tick >= leadStepTick) ? leadAfterStep : leadTicks;
        }

        double extraDelayTicks(std::int32_t sendTick) const
        {
            return extraDelayAt ? extraDelayAt(sendTick) : 0.0;
        }
    };

    struct SenderPlan
    {
        unsigned int  id   = kSenderId;
        std::uint8_t  slot = kPlayerSlot;
        Scenario      scenario;
    };

    struct Copy
    {
        unsigned int owner       = 0u;
        std::int32_t captureTick = 0;
        std::int32_t frame       = 0;
        bool         firstCopy   = false;
        bool         late        = false;
    };

    struct Oracle
    {
        std::uint32_t             lateFirstSight = 0u;
        std::uint32_t             lateResent     = 0u;
        std::vector<std::int32_t> m0;
    };

    struct Rig
    {
        TimeConfig                config;
        Coordinator               coordinator{ config };
        FStandaloneTestHandle     wire{ 1u, true };
        std::vector<std::string>  lines;
        std::vector<std::int32_t> arrivedCaptures;
        std::vector<std::int32_t> arrivalFrames;
        std::vector<Copy>         copies;
        std::map<unsigned int, std::set<std::int32_t>> seenByOwner;
        std::vector<ArrivalMarginSignal> signals;
        std::int32_t              receivingFrame = 0;
        std::size_t               delivered = 0u;

        Rig()
            : Rig(makeConfig())
        {
        }

        explicit Rig(const TimeConfig& withConfig)
            : config(withConfig)
        {
            coordinator.setLogger([this](const char* line) { lines.emplace_back(line); });
        }

        void receive(unsigned int owner, std::uint8_t slot, const LinkModel::Payload& payload)
        {
            for (const std::int32_t captureTick : decodeTicks(payload))
            {
                const bool firstCopy = seenByOwner[owner].insert(captureTick).second;
                if (owner == kSenderId && firstCopy)
                {
                    arrivedCaptures.push_back(captureTick);
                    arrivalFrames.push_back(receivingFrame);
                }
                const std::size_t lateBefore = coordinator.lateDroppedCount();
                (void)coordinator.receiveRemoteInput<MockSimA>(owner, wire, slot, captureTick,
                                                               static_cast<int>(captureTick));
                copies.push_back(Copy{ owner, captureTick, receivingFrame, firstCopy,
                                       coordinator.lateDroppedCount() != lateBefore });
            }
        }

        void frame(std::int32_t firstUpcomingSimTick, std::int32_t numSteps)
        {
            coordinator.releaseDelayedInputs<MockSimA>(firstUpcomingSimTick, numSteps,
                [this](unsigned int, std::uint32_t, const int&)
                {
                    ++delivered;
                    return true;
                });
            coordinator.reapConnections(firstUpcomingSimTick);
            const std::optional<ArrivalMarginSignal> s = coordinator.arrivalMargin().signal(wire);
            if (s.has_value() && (signals.empty() || signals.back().windowEndServerTick != s->windowEndServerTick))
            {
                signals.push_back(*s);
            }
        }

        void run(const std::vector<SenderPlan>& plans, std::int32_t lastServerTick, std::int32_t numSteps)
        {
            std::vector<LinkModel> uplinks;
            for (const SenderPlan& plan : plans)
            {
                uplinks.emplace_back(plan.scenario.link);
            }
            for (std::int32_t t = 0; t <= lastServerTick; ++t)
            {
                for (std::size_t i = 0; i < plans.size(); ++i)
                {
                    const Scenario& scenario = plans[i].scenario;
                    if (t < scenario.firstSendTick)
                    {
                        continue;
                    }
                    std::vector<std::int32_t> bundle;
                    const std::int32_t newest = t + scenario.leadAt(t);
                    for (std::int32_t c = newest - scenario.redundancyDepth + 1; c <= newest; ++c)
                    {
                        if (c >= scenario.firstSendTick + scenario.leadTicks)
                        {
                            bundle.push_back(c);
                        }
                    }
                    uplinks[i].send(encodeTicks(bundle), static_cast<double>(t) + scenario.extraDelayTicks(t));
                }
                if (t % numSteps == 0)
                {
                    receivingFrame = t;
                    for (std::size_t i = 0; i < plans.size(); ++i)
                    {
                        const SenderPlan& plan = plans[i];
                        uplinks[i].deliverDue(static_cast<double>(t),
                            [this, &plan](const LinkModel::Payload& payload) { receive(plan.id, plan.slot, payload); });
                    }
                    frame(t, numSteps);
                }
            }
            lostSends.clear();
            for (const LinkModel& uplink : uplinks)
            {
                lostSends.push_back(uplink.lostSends().size());
            }
        }

        void run(const Scenario& scenario)
        {
            SenderPlan plan;
            plan.scenario = scenario;
            run({ plan }, scenario.lastServerTick, scenario.numSteps);
        }

        Oracle oracle() const
        {
            Oracle out;
            for (const Copy& copy : copies)
            {
                if (copy.late)
                {
                    ++(copy.firstCopy ? out.lateFirstSight : out.lateResent);
                }
                if (copy.firstCopy && copy.frame > 0)
                {
                    out.m0.push_back(copy.captureTick - copy.frame);
                }
            }
            return out;
        }

        std::vector<std::string> arrivalLines() const
        {
            std::vector<std::string> out;
            for (const std::string& line : lines)
            {
                if (line.find("[InputArrival.Window]") != std::string::npos)
                {
                    out.push_back(line);
                }
            }
            return out;
        }

        std::vector<std::string> backDateLines() const
        {
            std::vector<std::string> out;
            for (const std::string& line : lines)
            {
                if (line.find("[InputArrival.BackDate]") != std::string::npos)
                {
                    out.push_back(line);
                }
            }
            return out;
        }

        std::string expectedBackDateLine(std::int32_t m0P50, std::uint32_t windows, std::int32_t delay) const
        {
            char buffer[256];
            std::snprintf(buffer, sizeof(buffer),
                "[Warning][InputArrival.BackDate] conn=%zu m0P50=%d windows=%u threshold=%d D=%d",
                static_cast<std::size_t>(std::hash<FStandaloneTestHandle>{}(wire)), m0P50,
                static_cast<unsigned int>(windows), config.backDateM0P50ThresholdTicks, delay);
            return buffer;
        }

        std::string expectedLine(const InputArrivalWindowSummary& s, const ArrivalMarginSignal& a) const
        {
            char buffer[384];
            std::snprintf(buffer, sizeof(buffer),
                "[Warning][InputArrival.Window] conn=%zu samples=%u marginP05=%d marginP50=%d marginP95=%d min=%d "
                "lateRefused=%u lateFirstSight=%u lateResent=%u m0P05=%d m0P50=%d stepsPerBundle=%d",
                static_cast<std::size_t>(std::hash<FStandaloneTestHandle>{}(wire)),
                static_cast<unsigned int>(s.samples), s.marginP05Ticks, s.marginP50Ticks, s.marginP95Ticks,
                s.marginMinTicks, static_cast<unsigned int>(s.lateRefused),
                static_cast<unsigned int>(a.lateFirstSight), static_cast<unsigned int>(a.lateResent), a.m0P05Ticks,
                a.m0P50Ticks, a.stepsPerBundleP50);
            return buffer;
        }

        std::vector<std::size_t> lostSends;
    };

    std::int32_t firstFrameAtOrAfter(double tick, std::int32_t numSteps)
    {
        const std::int32_t whole = static_cast<std::int32_t>(std::ceil(tick));
        return ((whole + numSteps - 1) / numSteps) * numSteps;
    }

    linkModel::Config fixedLink(std::int32_t delayTicks)
    {
        linkModel::Config config;
        config.seed           = 11u;
        config.baseDelayTicks = static_cast<double>(delayTicks);
        return config;
    }

    void checkSplitAgainstOracle(const Rig& rig)
    {
        const Oracle expected = rig.oracle();
        const std::optional<InputArrivalWindowSummary> latest = rig.coordinator.arrivalMargin().latest(rig.wire);
        const std::optional<ArrivalMarginSignal>       signal = rig.coordinator.arrivalMargin().signal(rig.wire);
        REQUIRE(latest.has_value());
        REQUIRE(signal.has_value());
        CAPTURE(latest->lateRefused, signal->lateFirstSight, signal->lateResent, expected.lateFirstSight,
                expected.lateResent, signal->samples, expected.m0.size());
        CHECK(latest->lateRefused == signal->lateFirstSight + signal->lateResent);
        CHECK(latest->lateRefused == rig.coordinator.lateDroppedCount());
        CHECK(signal->lateFirstSight == expected.lateFirstSight);
        CHECK(signal->lateResent == expected.lateResent);
        REQUIRE(signal->samples == expected.m0.size());
        REQUIRE_FALSE(expected.m0.empty());
        CHECK(signal->m0P05Ticks == nearestRank(expected.m0, 5u));
        CHECK(signal->m0P50Ticks == nearestRank(expected.m0, 50u));
    }

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

    struct BundleSim
    {
        using Input = BundleInput;
    };

    using BundleCoordinator = ServerReceptionCoordinator<FStandaloneTestHandle, BundleSim>;

    struct NoUndelayedDelivery
    {
        void deliverRemoteInput(unsigned int, std::uint32_t, const BundleInput&) {}
    };

    std::optional<ArrivalMarginSignal> runBundles(std::int32_t sendEveryTicks, std::int32_t redundancyDepth)
    {
        TimeConfig            config = makeConfig();
        BundleCoordinator     coordinator(config);
        const FStandaloneTestHandle wire(1u, true);
        LinkModel             uplink(fixedLink(3));
        NoUndelayedDelivery   undelayed;
        const std::int32_t    window = coordinator.arrivalMargin().windowTicks();

        for (std::int32_t t = 0; t <= window; ++t)
        {
            if (t % sendEveryTicks == 0)
            {
                WireBuffer bundle;
                const std::int32_t newest = t + 2;
                for (std::int32_t c = std::max(2, newest - redundancyDepth + 1); c <= newest; ++c)
                {
                    inputRedundancyBundle::appendSlot<BundleInput>(bundle, static_cast<std::uint32_t>(c),
                                                                   BundleInput{ c });
                }
                uplink.send(bundle.bytes, static_cast<double>(t));
            }
            uplink.deliverDue(static_cast<double>(t),
                [&](const LinkModel::Payload& payload)
                {
                    const WireBuffer received(payload);
                    coordinator.receiveInputBundle<BundleSim>(kSenderId, wire, kPlayerSlot, received, undelayed);
                });
            coordinator.releaseDelayedInputs<BundleSim>(t, 1,
                [](unsigned int, std::uint32_t, const BundleInput&) { return true; });
            coordinator.reapConnections(t);
        }
        return coordinator.arrivalMargin().signal(wire);
    }

    constexpr std::uint32_t kSpecGraceWindows   = 5u;
    constexpr std::uint32_t kSpecSustainWindows = 5u;
    constexpr std::uint32_t kSpecMinSamples     = 60u;

    struct QualifyingWindows
    {
        std::uint32_t count      = 0u;
        std::uint32_t longestRun = 0u;
        std::size_t   firstIndex = 0u;
    };

    QualifyingWindows qualifyingWindows(const std::vector<ArrivalMarginSignal>& signals, std::int32_t threshold)
    {
        QualifyingWindows out;
        std::uint32_t run         = 0u;
        std::uint32_t withSamples = 0u;
        for (std::size_t i = 0; i < signals.size(); ++i)
        {
            const ArrivalMarginSignal& s = signals[i];
            if (s.samples == 0u)
            {
                continue;
            }
            ++withSamples;
            const bool qualifies = withSamples > kSpecGraceWindows && s.samples >= kSpecMinSamples
                                && s.m0P50Ticks < threshold;
            run = qualifies ? run + 1u : 0u;
            if (qualifies && out.count++ == 0u)
            {
                out.firstIndex = i;
            }
            out.longestRun = std::max(out.longestRun, run);
        }
        return out;
    }

    std::function<double(std::int32_t)> pathSwing(std::uint64_t seed, std::int32_t segmentTicks, double swingTicks)
    {
        return [seed, segmentTicks, swingTicks](std::int32_t sendTick)
        {
            linkModel::SplitMix64 stream(seed + static_cast<std::uint64_t>(sendTick / segmentTicks));
            return (2.0 * stream.unit() - 1.0) * swingTicks;
        };
    }

    std::uint32_t episodesSoFar(const Rig& rig)
    {
        const std::optional<ArrivalMarginSignal> s = rig.coordinator.arrivalMargin().signal(rig.wire);
        return s.has_value() ? s->backDateEpisodes : 0u;
    }

TEST_CASE("InputArrivalMargin: a fixed delay and a fixed lead give margin = captureTick + D - nextUndrainedTick, exactly",
          "[Network][InputArrivalMargin]")
{
    const std::int32_t numSteps = GENERATE(1, 2);
    CAPTURE(numSteps);

    constexpr std::int32_t kLinkDelay = 3;
    Scenario scenario;
    scenario.numSteps        = numSteps;
    scenario.leadTicks       = 2;
    scenario.redundancyDepth = 1;
    scenario.link            = fixedLink(kLinkDelay);

    Rig rig;
    const std::int32_t window = rig.coordinator.arrivalMargin().windowTicks();
    REQUIRE(window == static_cast<std::int32_t>(2.0 * rig.config.tickFrequency));
    REQUIRE(window % numSteps == 0);
    scenario.lastServerTick = window;

    std::vector<std::int32_t> expected;
    for (std::int32_t sendTick = 0; sendTick <= scenario.lastServerTick; ++sendTick)
    {
        const std::int32_t arrivalFrame = firstFrameAtOrAfter(sendTick + kLinkDelay, numSteps);
        if (arrivalFrame > scenario.lastServerTick)
        {
            continue;
        }
        const std::int32_t captureTick       = sendTick + scenario.leadTicks;
        const std::int32_t nextUndrainedTick = arrivalFrame;
        expected.push_back(captureTick + kDelayTicks - nextUndrainedTick);
    }
    REQUIRE(expected.size() > 100u);

    rig.run(scenario);

    REQUIRE(rig.coordinator.delayQueue().effectiveDelay(rig.wire) == kDelayTicks);
    CHECK(rig.coordinator.lateDroppedCount() == 0u);
    CHECK(rig.coordinator.relayOooSkipCount() == 0u);

    const std::optional<InputArrivalWindowSummary> latest = rig.coordinator.arrivalMargin().latest(rig.wire);
    REQUIRE(latest.has_value());
    CHECK(latest->samples == expected.size());
    CHECK(latest->marginMinTicks == *std::min_element(expected.begin(), expected.end()));
    CHECK(latest->marginP05Ticks == nearestRank(expected, 5u));
    CHECK(latest->marginP50Ticks == nearestRank(expected, 50u));
    CHECK(latest->marginP95Ticks == nearestRank(expected, 95u));
    CHECK(latest->lateRefused == 0u);

    const std::optional<ArrivalMarginSignal> signal = rig.coordinator.arrivalMargin().signal(rig.wire);
    REQUIRE(signal.has_value());
    CHECK(signal->samples == latest->samples);
    CHECK(signal->m0P05Ticks == latest->marginP05Ticks - kDelayTicks);
    CHECK(signal->m0P50Ticks == latest->marginP50Ticks - kDelayTicks);
    CHECK(signal->lateFirstSight == 0u);
    CHECK(signal->lateResent == 0u);
    CHECK(signal->effectiveDelayTicks == kDelayTicks);
    CHECK(signal->windowEndServerTick == window);

    const std::vector<std::string> lines = rig.arrivalLines();
    REQUIRE(lines.size() == 1u);
    CHECK(lines[0] == rig.expectedLine(*latest, *signal));
}

TEST_CASE("InputArrivalMargin: the hand-derived margins of the exact case", "[Network][InputArrivalMargin]")
{
    SECTION("numSteps 1: L + D - d = 2 + 6 - 3 = 5 for every capture")
    {
        Scenario scenario;
        scenario.numSteps       = 1;
        scenario.leadTicks      = 2;
        scenario.link           = fixedLink(3);
        scenario.lastServerTick = 120;
        Rig rig;
        rig.run(scenario);
        const std::optional<InputArrivalWindowSummary> latest = rig.coordinator.arrivalMargin().latest(rig.wire);
        REQUIRE(latest.has_value());
        CHECK(latest->samples == 118u);
        CHECK(latest->marginMinTicks == 5);
        CHECK(latest->marginP05Ticks == 5);
        CHECK(latest->marginP95Ticks == 5);
        const std::optional<ArrivalMarginSignal> signal = rig.coordinator.arrivalMargin().signal(rig.wire);
        REQUIRE(signal.has_value());
        CHECK(signal->m0P05Ticks == -1);
        CHECK(signal->m0P50Ticks == -1);
        CHECK(signal->m0EwmaQ4 == -16);
    }

    SECTION("numSteps 2: an even send waits one tick for its frame (4), an odd one does not (5)")
    {
        Scenario scenario;
        scenario.numSteps       = 2;
        scenario.leadTicks      = 2;
        scenario.link           = fixedLink(3);
        scenario.lastServerTick = 120;
        Rig rig;
        rig.run(scenario);
        const std::optional<InputArrivalWindowSummary> latest = rig.coordinator.arrivalMargin().latest(rig.wire);
        REQUIRE(latest.has_value());
        CHECK(latest->samples == 118u);
        CHECK(latest->marginMinTicks == 4);
        CHECK(latest->marginP05Ticks == 4);
        CHECK(latest->marginP50Ticks == 4);
        CHECK(latest->marginP95Ticks == 5);
    }
}

TEST_CASE("InputArrivalMargin: uniform jitter gives p05 within 1 tick of the analytic value",
          "[Network][InputArrivalMargin]")
{
    constexpr std::int32_t kBaseDelay = 3;
    constexpr double       kSpread    = 4.0;

    Scenario scenario;
    scenario.numSteps                 = 1;
    scenario.leadTicks                = 4;
    scenario.redundancyDepth          = 1;
    scenario.link.seed                = 21u;
    scenario.link.baseDelayTicks      = static_cast<double>(kBaseDelay);
    scenario.link.jitterShape         = linkModel::JitterShape::Uniform;
    scenario.link.jitterSpreadTicks   = kSpread;

    Rig rig;
    scenario.lastServerTick = rig.coordinator.arrivalMargin().windowTicks();
    rig.run(scenario);

    const std::int32_t analyticP05 =
        scenario.leadTicks + kDelayTicks - kBaseDelay - static_cast<std::int32_t>(std::ceil(0.95 * kSpread));
    REQUIRE(analyticP05 == 3);

    const std::optional<InputArrivalWindowSummary> latest = rig.coordinator.arrivalMargin().latest(rig.wire);
    REQUIRE(latest.has_value());
    CAPTURE(latest->samples, latest->marginP05Ticks, latest->marginP50Ticks, latest->marginP95Ticks,
            latest->marginMinTicks, rig.coordinator.relayOooSkipCount(), rig.arrivedCaptures.size());
    CHECK(latest->marginP05Ticks >= analyticP05 - 1);
    CHECK(latest->marginP05Ticks <= analyticP05 + 1);
    CHECK(latest->marginMinTicks >= analyticP05);
    CHECK(latest->lateRefused == 0u);
    CHECK(latest->samples == rig.arrivedCaptures.size());
    CHECK(rig.coordinator.relayOooSkipCount() > 0u);
}

TEST_CASE("InputArrivalMargin: under reorder every new park is sampled, and p50 equals the analytic value",
          "[Network][InputArrivalMargin]")
{
    constexpr std::int32_t kBaseDelay   = 3;
    constexpr std::int32_t kSpreadTicks = 6;

    Scenario scenario;
    scenario.numSteps                     = 1;
    scenario.leadTicks                    = 4;
    scenario.redundancyDepth              = 1;
    scenario.link.seed                    = 31u;
    scenario.link.baseDelayTicks          = static_cast<double>(kBaseDelay);
    scenario.link.jitterShape             = linkModel::JitterShape::TwoPoint;
    scenario.link.jitterSpreadTicks       = static_cast<double>(kSpreadTicks);
    scenario.link.twoPointHighProbability = 0.65;

    Rig rig;
    scenario.lastServerTick = rig.coordinator.arrivalMargin().windowTicks();
    rig.run(scenario);

    const std::int32_t fastMargin = scenario.leadTicks + kDelayTicks - kBaseDelay;
    const std::int32_t slowMargin = fastMargin - kSpreadTicks;
    REQUIRE(slowMargin == 1);
    REQUIRE(fastMargin == 7);
    REQUIRE(scenario.link.twoPointHighProbability > 0.5);
    const std::int32_t analyticP50 = slowMargin;

    REQUIRE(rig.arrivalFrames.size() == rig.arrivedCaptures.size());
    std::vector<std::int32_t> expected;
    std::vector<std::int32_t> expectedInOrderOnly;
    std::size_t slowArrivals = 0u;
    std::int32_t newestSoFar = 0;
    for (std::size_t i = 0; i < rig.arrivedCaptures.size(); ++i)
    {
        REQUIRE(rig.arrivalFrames[i] > 0);
        expected.push_back(rig.arrivedCaptures[i] + kDelayTicks - rig.arrivalFrames[i]);
        REQUIRE((expected.back() == slowMargin || expected.back() == fastMargin));
        slowArrivals += (expected.back() == slowMargin) ? 1u : 0u;
        if (i == 0u || rig.arrivedCaptures[i] > newestSoFar)
        {
            expectedInOrderOnly.push_back(expected.back());
            newestSoFar = rig.arrivedCaptures[i];
        }
    }
    REQUIRE(expected.size() > 100u);
    REQUIRE(2u * slowArrivals > expected.size());
    REQUIRE(nearestRank(expected, 50u) == analyticP50);
    REQUIRE(expectedInOrderOnly.size() < expected.size());
    REQUIRE(nearestRank(expectedInOrderOnly, 50u) == fastMargin);

    const std::optional<InputArrivalWindowSummary> latest = rig.coordinator.arrivalMargin().latest(rig.wire);
    REQUIRE(latest.has_value());
    CAPTURE(latest->samples, latest->marginP05Ticks, latest->marginP50Ticks, latest->marginP95Ticks,
            latest->marginMinTicks, rig.coordinator.relayOooSkipCount(), expected.size(), slowArrivals,
            expectedInOrderOnly.size());
    CHECK(rig.coordinator.relayOooSkipCount() == expected.size() - expectedInOrderOnly.size());
    CHECK(rig.coordinator.lateDroppedCount() == 0u);
    CHECK(latest->samples == expected.size());
    CHECK(latest->marginP50Ticks == analyticP50);
    CHECK(latest->marginMinTicks == slowMargin);
    CHECK(latest->marginP95Ticks == fastMargin);
    CHECK(latest->lateRefused == 0u);
}

TEST_CASE("InputArrivalMargin: a late capture is counted as lateRefused and gives no sample",
          "[Network][InputArrivalMargin]")
{
    Scenario scenario;
    scenario.numSteps        = 1;
    scenario.leadTicks       = -5;
    scenario.redundancyDepth = 1;
    scenario.firstSendTick   = 10;
    scenario.link            = fixedLink(3);

    Rig rig;
    scenario.lastServerTick = rig.coordinator.arrivalMargin().windowTicks();
    rig.run(scenario);

    REQUIRE(scenario.leadTicks + kDelayTicks - 3 < 0);
    const std::optional<InputArrivalWindowSummary> latest = rig.coordinator.arrivalMargin().latest(rig.wire);
    REQUIRE(latest.has_value());
    CHECK(latest->samples == 0u);
    CHECK(latest->lateRefused == rig.arrivedCaptures.size());
    CHECK(latest->lateRefused == rig.coordinator.lateDroppedCount());
    CHECK(rig.delivered == 0u);

    const std::optional<ArrivalMarginSignal> signal = rig.coordinator.arrivalMargin().signal(rig.wire);
    REQUIRE(signal.has_value());
    CHECK(signal->lateFirstSight == latest->lateRefused);
    CHECK(signal->lateResent == 0u);
    CHECK(signal->samples == latest->lateRefused);
    CHECK(signal->m0P50Ticks == scenario.leadTicks - 3);

    const std::vector<std::string> lines = rig.arrivalLines();
    REQUIRE(lines.size() == 1u);
    CHECK(lines[0] == rig.expectedLine(*latest, *signal));
}

TEST_CASE("InputArrivalMargin: a redundancy re-send of a still-parked capture is a duplicate, not lateRefused",
          "[Network][InputArrivalMargin]")
{
    Scenario scenario;
    scenario.numSteps        = 1;
    scenario.leadTicks       = 2;
    scenario.redundancyDepth = 3;
    scenario.link            = fixedLink(3);

    Rig rig;
    scenario.lastServerTick = rig.coordinator.arrivalMargin().windowTicks();
    rig.run(scenario);

    const std::optional<InputArrivalWindowSummary> latest = rig.coordinator.arrivalMargin().latest(rig.wire);
    REQUIRE(latest.has_value());
    CHECK(latest->samples == 118u);
    CHECK(latest->marginMinTicks == 5);
    CHECK(latest->marginP95Ticks == 5);
    CHECK(latest->lateRefused == 0u);
    CHECK(rig.coordinator.lateDroppedCount() == 0u);
}

TEST_CASE("InputArrivalMargin: (a) a lost bundle's recovered ticks are first sight, not resent",
          "[Network][InputArrivalMargin][FirstSight]")
{
    Scenario scenario;
    scenario.numSteps             = 1;
    scenario.leadTicks            = -3;
    scenario.firstSendTick        = 10;
    scenario.redundancyDepth      = 3;
    scenario.link                 = fixedLink(3);
    scenario.link.seed            = 41u;
    scenario.link.lossProbability = 0.15;

    Rig rig;
    scenario.lastServerTick = rig.coordinator.arrivalMargin().windowTicks();
    rig.run(scenario);
    REQUIRE(scenario.leadTicks + kDelayTicks - 3 == 0);

    std::uint32_t recoveredLate = 0u;
    for (const Copy& copy : rig.copies)
    {
        if (copy.firstCopy && copy.frame > copy.captureTick + kDelayTicks)
        {
            REQUIRE(copy.late);
            ++recoveredLate;
        }
    }
    REQUIRE(rig.lostSends.size() == 1u);
    REQUIRE(rig.lostSends[0] > 5u);
    REQUIRE(recoveredLate > 5u);

    checkSplitAgainstOracle(rig);
    const std::optional<ArrivalMarginSignal> signal = rig.coordinator.arrivalMargin().signal(rig.wire);
    CHECK(signal->lateFirstSight == recoveredLate);
    CHECK(signal->lateResent > 50u);
}

TEST_CASE("InputArrivalMargin: (b) with less slack than the re-send span, re-sends of released ticks are lateResent",
          "[Network][InputArrivalMargin][FirstSight]")
{
    Scenario scenario;
    scenario.numSteps        = 1;
    scenario.leadTicks       = -2;
    scenario.firstSendTick   = 10;
    scenario.redundancyDepth = 3;
    scenario.link            = fixedLink(3);

    Rig rig;
    scenario.lastServerTick = rig.coordinator.arrivalMargin().windowTicks();
    rig.run(scenario);
    REQUIRE(scenario.leadTicks + kDelayTicks - 3 == 1);

    checkSplitAgainstOracle(rig);
    const std::optional<InputArrivalWindowSummary> latest = rig.coordinator.arrivalMargin().latest(rig.wire);
    const std::optional<ArrivalMarginSignal>       signal = rig.coordinator.arrivalMargin().signal(rig.wire);
    CHECK(signal->lateFirstSight == 0u);
    CHECK(signal->lateResent > 50u);
    CHECK(signal->lateResent == latest->lateRefused);
    CHECK(latest->samples == signal->samples);
}

TEST_CASE("InputArrivalMargin: (c) a reordered late first copy is lateFirstSight and adds a negative m0",
          "[Network][InputArrivalMargin][FirstSight]")
{
    constexpr std::int32_t kBaseDelay   = 3;
    constexpr std::int32_t kSpreadTicks = 3;

    Scenario scenario;
    scenario.numSteps                     = 1;
    scenario.leadTicks                    = -2;
    scenario.firstSendTick                = 10;
    scenario.redundancyDepth              = 1;
    scenario.link.seed                    = 51u;
    scenario.link.baseDelayTicks          = static_cast<double>(kBaseDelay);
    scenario.link.jitterShape             = linkModel::JitterShape::TwoPoint;
    scenario.link.jitterSpreadTicks       = static_cast<double>(kSpreadTicks);
    scenario.link.twoPointHighProbability = 0.35;

    Rig rig;
    scenario.lastServerTick = rig.coordinator.arrivalMargin().windowTicks();
    rig.run(scenario);

    const std::int32_t fastM0 = scenario.leadTicks - kBaseDelay;
    const std::int32_t slowM0 = fastM0 - kSpreadTicks;
    REQUIRE(fastM0 + kDelayTicks == 1);
    REQUIRE(slowM0 + kDelayTicks == -2);

    std::uint32_t reorderedLate = 0u;
    std::int32_t  newestSoFar   = 0;
    for (const Copy& copy : rig.copies)
    {
        REQUIRE(copy.firstCopy);
        if (copy.late && copy.captureTick < newestSoFar)
        {
            ++reorderedLate;
        }
        newestSoFar = std::max(newestSoFar, copy.captureTick);
    }
    REQUIRE(reorderedLate > 10u);

    checkSplitAgainstOracle(rig);
    const std::optional<ArrivalMarginSignal> signal = rig.coordinator.arrivalMargin().signal(rig.wire);
    CHECK(signal->lateResent == 0u);
    CHECK(signal->lateFirstSight >= reorderedLate);
    CHECK(signal->m0P05Ticks == slowM0);
    CHECK(signal->m0P50Ticks == fastM0);
}

TEST_CASE("InputArrivalMargin: (d) m0 is identical at D = 1 and D = 6", "[Network][InputArrivalMargin][FirstSight]")
{
    Scenario scenario;
    scenario.numSteps               = 1;
    scenario.leadTicks              = 4;
    scenario.redundancyDepth        = 3;
    scenario.link.seed              = 21u;
    scenario.link.baseDelayTicks    = 3.0;
    scenario.link.jitterShape       = linkModel::JitterShape::Uniform;
    scenario.link.jitterSpreadTicks = 4.0;

    Rig lowDelay(makeUniformDelayConfig(1));
    Rig shippedDelay(makeUniformDelayConfig(6));
    scenario.lastServerTick = lowDelay.coordinator.arrivalMargin().windowTicks();
    lowDelay.run(scenario);
    shippedDelay.run(scenario);
    REQUIRE(lowDelay.coordinator.delayQueue().effectiveDelay(lowDelay.wire) == 1);
    REQUIRE(shippedDelay.coordinator.delayQueue().effectiveDelay(shippedDelay.wire) == 6);

    checkSplitAgainstOracle(lowDelay);
    checkSplitAgainstOracle(shippedDelay);

    const ArrivalMarginSignal low     = *lowDelay.coordinator.arrivalMargin().signal(lowDelay.wire);
    const ArrivalMarginSignal shipped = *shippedDelay.coordinator.arrivalMargin().signal(shippedDelay.wire);
    CAPTURE(low.samples, low.m0P05Ticks, low.m0P50Ticks, low.m0EwmaQ4, low.lateFirstSight, low.lateResent,
            shipped.samples, shipped.m0P05Ticks, shipped.m0P50Ticks, shipped.m0EwmaQ4);
    REQUIRE(low.lateFirstSight > 0u);
    REQUIRE(shipped.lateFirstSight == 0u);
    CHECK(low.samples == shipped.samples);
    CHECK(low.m0P05Ticks == shipped.m0P05Ticks);
    CHECK(low.m0P50Ticks == shipped.m0P50Ticks);
    CHECK(low.m0EwmaQ4 == shipped.m0EwmaQ4);
    CHECK(low.effectiveDelayTicks == 1);
    CHECK(shipped.effectiveDelayTicks == 6);

    const InputArrivalWindowSummary lowMargin     = *lowDelay.coordinator.arrivalMargin().latest(lowDelay.wire);
    const InputArrivalWindowSummary shippedMargin = *shippedDelay.coordinator.arrivalMargin().latest(shippedDelay.wire);
    CHECK(lowMargin.samples < shippedMargin.samples);
}

TEST_CASE("InputArrivalMargin: (e) stepsPerBundle is the newest capture's advance per bundle",
          "[Network][InputArrivalMargin][FirstSight]")
{
    const std::optional<ArrivalMarginSignal> oneCapture  = runBundles(1, 3);
    const std::optional<ArrivalMarginSignal> twoCaptures = runBundles(2, 3);
    REQUIRE(oneCapture.has_value());
    REQUIRE(twoCaptures.has_value());
    CHECK(oneCapture->stepsPerBundleP50 == 1);
    CHECK(twoCaptures->stepsPerBundleP50 == 2);
    CHECK(oneCapture->lateFirstSight == 0u);
    CHECK(twoCaptures->lateFirstSight == 0u);
}

TEST_CASE("InputArrivalMargin: (f) the m0 EWMA converges to a fixed m0 within 0.1 tick after 5 s",
          "[Network][InputArrivalMargin][FirstSight]")
{
    constexpr std::int32_t kStepTick = 60;

    Scenario scenario;
    scenario.numSteps        = 1;
    scenario.leadTicks       = 2;
    scenario.leadStepTick    = kStepTick;
    scenario.leadAfterStep   = 5;
    scenario.redundancyDepth = 1;
    scenario.link            = fixedLink(3);

    Rig rig;
    const std::int32_t window = rig.coordinator.arrivalMargin().windowTicks();
    scenario.lastServerTick   = 4 * window;
    rig.run(scenario);

    const std::int32_t m0Before = scenario.leadTicks - 3;
    const std::int32_t m0After  = scenario.leadAfterStep - 3;
    REQUIRE(m0After - m0Before == 3);
    CHECK(std::fabs(rig.coordinator.arrivalMargin().ewmaAlpha() - 1.0 / rig.config.tickFrequency) < 1e-12);

    const std::int32_t settledBy = kStepTick + static_cast<std::int32_t>(5.0 * rig.config.tickFrequency);
    REQUIRE(rig.signals.size() == 4u);
    bool sawSettled = false;
    for (const ArrivalMarginSignal& s : rig.signals)
    {
        const double ewma = static_cast<double>(s.m0EwmaQ4) / 16.0;
        CAPTURE(s.windowEndServerTick, ewma);
        if (s.windowEndServerTick >= settledBy)
        {
            CHECK(std::fabs(ewma - static_cast<double>(m0After)) <= 0.1);
            CHECK(s.m0P50Ticks == m0After);
            sawSettled = true;
        }
        else
        {
            CHECK(std::fabs(ewma - static_cast<double>(m0After)) > 0.1);
        }
    }
    CHECK(sawSettled);
}

TEST_CASE("InputArrivalMargin: (g) two owners on one wire keep separate seen masks",
          "[Network][InputArrivalMargin][FirstSight]")
{
    SenderPlan inTime;
    inTime.id                       = kSenderId;
    inTime.slot                     = 0u;
    inTime.scenario.leadTicks       = -2;
    inTime.scenario.firstSendTick   = 10;
    inTime.scenario.redundancyDepth = 3;
    inTime.scenario.link            = fixedLink(3);

    SenderPlan lateFirst = inTime;
    lateFirst.id            = kSenderId + 1u;
    lateFirst.slot          = 1u;
    lateFirst.scenario.link = fixedLink(5);
    lateFirst.scenario.link.seed = 12u;

    Rig rig;
    const std::int32_t window = rig.coordinator.arrivalMargin().windowTicks();
    rig.run({ inTime, lateFirst }, window, 1);

    std::uint32_t lateFirstCopies = 0u;
    std::uint32_t inTimeResentLate = 0u;
    for (const Copy& copy : rig.copies)
    {
        lateFirstCopies  += (copy.owner == lateFirst.id && copy.firstCopy && copy.late) ? 1u : 0u;
        inTimeResentLate += (copy.owner == inTime.id && !copy.firstCopy && copy.late) ? 1u : 0u;
    }
    REQUIRE(lateFirstCopies > 50u);
    REQUIRE(inTimeResentLate > 50u);

    checkSplitAgainstOracle(rig);
    const std::optional<ArrivalMarginSignal> signal = rig.coordinator.arrivalMargin().signal(rig.wire);
    CHECK(signal->lateFirstSight == lateFirstCopies);
    CHECK(rig.coordinator.arrivalMargin().connectionCount() == 1u);
    CHECK(rig.coordinator.arrivalMargin().ownerCount() == 2u);
}

TEST_CASE("InputArrivalMargin: forgetOwner drops the owner's seen mask", "[Network][InputArrivalMargin][FirstSight]")
{
    Rig rig;
    rig.frame(0, 1);
    (void)rig.coordinator.receiveRemoteInput<MockSimA>(kSenderId, rig.wire, kPlayerSlot, 4, 4);
    (void)rig.coordinator.receiveRemoteInput<MockSimA>(kSenderId + 1u, rig.wire, 1u, 4, 4);
    REQUIRE(rig.coordinator.arrivalMargin().ownerCount() == 2u);
    rig.coordinator.forgetOwner(kSenderId);
    CHECK(rig.coordinator.arrivalMargin().ownerCount() == 1u);
}

TEST_CASE("InputArrivalMargin: no sample before the first drain", "[Network][InputArrivalMargin]")
{
    Rig rig;
    (void)rig.coordinator.receiveRemoteInput<MockSimA>(kSenderId, rig.wire, kPlayerSlot, 5, 5);
    CHECK(rig.coordinator.arrivalMargin().openWindowSamples(rig.wire) == 0u);
    CHECK(rig.coordinator.arrivalMargin().connectionCount() == 0u);

    rig.frame(0, 1);
    (void)rig.coordinator.receiveRemoteInput<MockSimA>(kSenderId, rig.wire, kPlayerSlot, 6, 6);
    CHECK(rig.coordinator.arrivalMargin().openWindowSamples(rig.wire) == 1u);
}

TEST_CASE("InputArrivalMargin: the window closes on the [InputStats] reap and emits once per carrying wire",
          "[Network][InputArrivalMargin]")
{
    Rig rig;
    const FStandaloneTestHandle quietWire(2u, true);
    const std::int32_t window = rig.coordinator.arrivalMargin().windowTicks();

    rig.frame(0, 1);
    (void)rig.coordinator.receiveRemoteInput<MockSimA>(kSenderId, rig.wire, kPlayerSlot, 8, 8);
    for (std::int32_t t = 1; t < window; ++t)
    {
        rig.frame(t, 1);
    }
    CHECK(rig.arrivalLines().empty());
    CHECK_FALSE(rig.coordinator.arrivalMargin().latest(rig.wire).has_value());

    rig.frame(window, 1);
    const std::vector<std::string> lines = rig.arrivalLines();
    REQUIRE(lines.size() == 1u);
    CHECK(lines[0].find("samples=1 marginP05=13 marginP50=13 marginP95=13 min=13 lateRefused=0") != std::string::npos);
    CHECK(lines[0].find("lateRefused=0 lateFirstSight=0 lateResent=0 m0P05=7 m0P50=7 stepsPerBundle=0")
          != std::string::npos);
    CHECK(rig.coordinator.arrivalMargin().latest(rig.wire).has_value());
    CHECK_FALSE(rig.coordinator.arrivalMargin().latest(quietWire).has_value());

    for (std::int32_t t = window + 1; t <= 2 * window; ++t)
    {
        rig.frame(t, 1);
    }
    CHECK(rig.arrivalLines().size() == 1u);
    CHECK(rig.coordinator.arrivalMargin().latest(rig.wire)->samples == 1u);
}

TEST_CASE("InputArrivalMargin: a dead wire is forgotten on the reap path", "[Network][InputArrivalMargin]")
{
    Rig rig;
    const FStandaloneTestHandle deadWire(3u, false);
    rig.frame(0, 1);
    (void)rig.coordinator.receiveRemoteInput<MockSimA>(kSenderId, deadWire, kPlayerSlot, 4, 4);
    (void)rig.coordinator.receiveRemoteInput<MockSimA>(kSenderId + 1u, rig.wire, kPlayerSlot, 4, 4);
    REQUIRE(rig.coordinator.arrivalMargin().connectionCount() == 2u);

    rig.frame(rig.config.tierMinDwellTicks, 1);
    CHECK(rig.coordinator.arrivalMargin().connectionCount() == 1u);
    CHECK(rig.coordinator.arrivalMargin().openWindowSamples(rig.wire) == 1u);
    CHECK(rig.coordinator.arrivalMargin().openWindowSamples(deadWire) == 0u);
}

TEST_CASE("InputArrivalMargin: the class alone, nearest-rank over the window", "[Network][InputArrivalMargin]")
{
    Margin margin(10, 60.0, TimeConfig{}.backDateM0P50ThresholdTicks);
    const FStandaloneTestHandle wire(1u, true);
    Margin::WindowLines out;

    margin.noteArrival(wire, 10, 6, Margin::kNoDrainYet, true);
    CHECK(margin.connectionCount() == 0u);

    CHECK_FALSE(margin.closeWindowIfDue(100, out));
    for (std::int32_t m = 1; m <= 20; ++m)
    {
        margin.noteArrival(wire, m, 0, 0, true);
    }
    margin.noteLateRefused(wire);
    margin.noteLateResent(wire);
    CHECK_FALSE(margin.closeWindowIfDue(109, out));
    CHECK(margin.closeWindowIfDue(110, out));
    REQUIRE(out.size() == 1u);
    CHECK(out[0].address == wire);
    CHECK(out[0].summary.samples == 20u);
    CHECK(out[0].summary.marginP05Ticks == 1);
    CHECK(out[0].summary.marginP50Ticks == 10);
    CHECK(out[0].summary.marginP95Ticks == 19);
    CHECK(out[0].summary.marginMinTicks == 1);
    CHECK(out[0].summary.lateRefused == 1u);
    CHECK(out[0].arrivalSignal.samples == 20u);
    CHECK(out[0].arrivalSignal.m0P05Ticks == 1);
    CHECK(out[0].arrivalSignal.m0P50Ticks == 10);
    CHECK(out[0].arrivalSignal.lateResent == 1u);
    CHECK(out[0].arrivalSignal.windowEndServerTick == 110);

    CHECK(margin.closeWindowIfDue(120, out));
    CHECK(out.empty());
    REQUIRE(margin.latest(wire).has_value());
    CHECK(margin.latest(wire)->samples == 20u);
    REQUIRE(margin.signal(wire).has_value());
    CHECK(margin.signal(wire)->samples == 20u);

    margin.forget(wire);
    CHECK_FALSE(margin.latest(wire).has_value());
    CHECK_FALSE(margin.signal(wire).has_value());
}

TEST_CASE("InputArrivalMargin: the class alone, a parked capture that is not a first sight adds no m0",
          "[Network][InputArrivalMargin][FirstSight]")
{
    Margin margin(10, 60.0, TimeConfig{}.backDateM0P50ThresholdTicks);
    const FStandaloneTestHandle wire(1u, true);
    Margin::WindowLines out;
    CHECK_FALSE(margin.closeWindowIfDue(0, out));
    margin.noteArrival(wire, 5, 6, 3, true);
    margin.noteArrival(wire, 6, 9, 3, false);
    CHECK(margin.closeWindowIfDue(10, out));
    REQUIRE(out.size() == 1u);
    CHECK(out[0].summary.samples == 2u);
    CHECK(out[0].arrivalSignal.samples == 1u);
    CHECK(out[0].arrivalSignal.m0P50Ticks == 2);
    CHECK(out[0].arrivalSignal.effectiveDelayTicks == 6);
}

TEST_CASE("InputArrivalMargin: the class alone, noteSighting is true exactly once per (owner, tick)",
          "[Network][InputArrivalMargin][FirstSight]")
{
    Margin margin(10, 60.0, TimeConfig{}.backDateM0P50ThresholdTicks);
    CHECK(margin.noteSighting(1u, 100));
    CHECK_FALSE(margin.noteSighting(1u, 100));
    CHECK(margin.noteSighting(2u, 100));
    CHECK(margin.noteSighting(1u, 98));
    CHECK_FALSE(margin.noteSighting(1u, 98));
    CHECK(margin.noteSighting(1u, 99));
    CHECK(margin.noteSighting(1u, 100 + Margin::kSeenMaskTicks - 1));
    CHECK_FALSE(margin.noteSighting(1u, 100));
    CHECK(margin.noteSighting(1u, 101));

    const std::int32_t anchor = 100 + Margin::kSeenMaskTicks - 1;
    CHECK(margin.noteSighting(1u, anchor - Margin::kSeenMaskTicks));
    CHECK(margin.noteSighting(1u, anchor + 1));

    std::uint32_t firsts = 0u;
    for (std::int32_t round = 0; round < 3; ++round)
    {
        for (std::int32_t tick = 1000; tick < 1041; ++tick)
        {
            firsts += margin.noteSighting(3u, tick) ? 1u : 0u;
        }
    }
    CHECK(firsts == 41u);

    margin.forgetOwner(1u);
    CHECK(margin.ownerCount() == 2u);
    CHECK(margin.noteSighting(1u, 100));
}

TEST_CASE("InputArrivalMargin: the class alone, steps per bundle are sampled in [1, 16]",
          "[Network][InputArrivalMargin][FirstSight]")
{
    Margin margin(10, 60.0, TimeConfig{}.backDateM0P50ThresholdTicks);
    const FStandaloneTestHandle wire(1u, true);
    Margin::WindowLines out;
    CHECK_FALSE(margin.closeWindowIfDue(0, out));
    margin.noteArrival(wire, 20, 6, 15, true);
    margin.noteBundleNewest(wire, 1u, 10);
    margin.noteBundleNewest(wire, 1u, 12);
    margin.noteBundleNewest(wire, 1u, 11);
    margin.noteBundleNewest(wire, 1u, 14);
    margin.noteBundleNewest(wire, 1u, 14 + Margin::kStepsPerBundleMax + 1);
    margin.noteBundleNewest(wire, 1u, 31 + Margin::kStepsPerBundleMax);
    margin.noteBundleNewest(wire, 2u, 500);
    CHECK(margin.closeWindowIfDue(10, out));
    REQUIRE(out.size() == 1u);
    CHECK(out[0].arrivalSignal.stepsPerBundleP50 == 2);
}

TEST_CASE("InputArrivalMargin: the class alone, the EWMA steps by 1/tickFrequency and quantizes to 1/16 tick",
          "[Network][InputArrivalMargin][FirstSight]")
{
    Margin margin(10, 4.0, TimeConfig{}.backDateM0P50ThresholdTicks);
    const FStandaloneTestHandle wire(1u, true);
    Margin::WindowLines out;
    CHECK_FALSE(margin.closeWindowIfDue(0, out));
    margin.noteArrival(wire, 8, 0, 0, true);
    margin.noteArrival(wire, 0, 0, 0, true);
    margin.noteLateRefused(wire);
    margin.noteLateFirstSight(wire, -8, 0);
    CHECK(margin.closeWindowIfDue(10, out));
    REQUIRE(out.size() == 1u);
    CHECK(out[0].arrivalSignal.m0EwmaQ4 == static_cast<std::int16_t>(std::lround(16.0 * (6.0 - 14.0 / 4.0))));
    CHECK(out[0].arrivalSignal.samples == 3u);
    CHECK(out[0].arrivalSignal.m0P05Ticks == -8);
    CHECK(out[0].arrivalSignal.lateFirstSight == 1u);
    CHECK(out[0].summary.lateRefused == 1u);
}

TEST_CASE("InputArrivalMargin: an honest session with uniform jitter and a lead parked 3 ticks low opens no "
          "back-dating episode in 10 minutes", "[Network][InputArrivalMargin][BackDate]")
{
    Scenario scenario;
    scenario.numSteps               = 1;
    scenario.leadTicks              = 5;
    scenario.redundancyDepth        = 1;
    scenario.link.seed              = 71u;
    scenario.link.baseDelayTicks    = 5.0;
    scenario.link.jitterShape       = linkModel::JitterShape::Uniform;
    scenario.link.jitterSpreadTicks = 5.0;

    Rig rig;
    const std::int32_t window = rig.coordinator.arrivalMargin().windowTicks();
    scenario.extraDelayAt   = pathSwing(0x5EEDu, window, 3.0);
    scenario.lastServerTick = static_cast<std::int32_t>(600.0 * rig.config.tickFrequency);
    rig.run(scenario);

    const Oracle expected = rig.oracle();
    const QualifyingWindows below = qualifyingWindows(rig.signals, rig.config.backDateM0P50ThresholdTicks);
    CAPTURE(rig.signals.size(), nearestRank(expected.m0, 50u), below.count, below.longestRun);
    REQUIRE(rig.signals.size() >= 299u);
    REQUIRE(nearestRank(expected.m0, 50u) == -3);
    REQUIRE(below.count > 0u);
    REQUIRE(below.longestRun < kSpecSustainWindows);

    CHECK(episodesSoFar(rig) == 0u);
    CHECK(rig.backDateLines().empty());
}

TEST_CASE("InputArrivalMargin: captures stamped 6 ticks old open exactly one back-dating episode, in the 5th "
          "qualifying window", "[Network][InputArrivalMargin][BackDate]")
{
    constexpr std::int32_t kLinkDelay = 3;
    constexpr std::int32_t kStamp     = 6;

    Rig rig;
    const std::int32_t window = rig.coordinator.arrivalMargin().windowTicks();

    Scenario scenario;
    scenario.numSteps        = 1;
    scenario.redundancyDepth = 1;
    scenario.link            = fixedLink(kLinkDelay);
    scenario.lastServerTick  = 60 * window;

    SECTION("from the wire's first window")
    {
        scenario.leadTicks = kLinkDelay - kStamp;
        rig.run(scenario);
    }
    SECTION("after an honest start")
    {
        scenario.leadTicks     = kLinkDelay;
        scenario.leadStepTick  = 15 * window;
        scenario.leadAfterStep = kLinkDelay - kStamp;
        rig.run(scenario);
    }

    const QualifyingWindows below = qualifyingWindows(rig.signals, rig.config.backDateM0P50ThresholdTicks);
    CAPTURE(rig.signals.size(), below.count, below.longestRun, below.firstIndex);
    REQUIRE(rig.signals.size() >= 59u);
    REQUIRE(below.count > 2u * kSpecSustainWindows);
    REQUIRE(rig.signals[below.firstIndex].m0P50Ticks == -kStamp);
    REQUIRE(below.firstIndex >= kSpecGraceWindows);

    const std::size_t opensAt = below.firstIndex + kSpecSustainWindows - 1u;
    for (std::size_t i = 0; i < rig.signals.size(); ++i)
    {
        CAPTURE(i, rig.signals[i].m0P50Ticks, rig.signals[i].samples);
        CHECK(rig.signals[i].backDateEpisodes == (i < opensAt ? 0u : 1u));
    }

    const std::vector<std::string> backDate = rig.backDateLines();
    REQUIRE(backDate.size() == 1u);
    CHECK(backDate[0] == rig.expectedBackDateLine(-kStamp, kSpecSustainWindows, kDelayTicks));

    std::size_t windowLinesBefore = 0u;
    for (const std::string& line : rig.lines)
    {
        if (line == backDate[0])
        {
            break;
        }
        windowLinesBefore += (line.find("[InputArrival.Window]") != std::string::npos) ? 1u : 0u;
    }
    CHECK(windowLinesBefore == opensAt + 1u);
}

TEST_CASE("InputArrivalMargin: a dip of 3 or 4 windows opens no back-dating episode, a dip of 5 opens one",
          "[Network][InputArrivalMargin][BackDate]")
{
    constexpr std::int32_t kLinkDelay = 3;
    constexpr std::int32_t kDipStart  = 10;
    const std::int32_t     dipWindows = GENERATE(3, 4, 5);
    CAPTURE(dipWindows);

    Rig rig;
    const std::int32_t window  = rig.coordinator.arrivalMargin().windowTicks();
    const std::int32_t dipFrom = kDipStart * window;
    const std::int32_t dipTo   = (kDipStart + dipWindows) * window;

    Scenario scenario;
    scenario.numSteps        = 1;
    scenario.leadTicks       = kLinkDelay;
    scenario.redundancyDepth = 1;
    scenario.link            = fixedLink(kLinkDelay);
    scenario.lastServerTick  = 30 * window;
    scenario.extraDelayAt    = [dipFrom, dipTo](std::int32_t sendTick)
    {
        return (sendTick >= dipFrom && sendTick < dipTo) ? 6.0 : 0.0;
    };
    rig.run(scenario);

    const QualifyingWindows below = qualifyingWindows(rig.signals, rig.config.backDateM0P50ThresholdTicks);
    CAPTURE(rig.signals.size(), below.count, below.longestRun, below.firstIndex);
    REQUIRE(below.count == static_cast<std::uint32_t>(dipWindows));
    REQUIRE(below.longestRun == static_cast<std::uint32_t>(dipWindows));
    REQUIRE(rig.signals[below.firstIndex].m0P50Ticks == -6);

    const std::uint32_t expectedEpisodes = (static_cast<std::uint32_t>(dipWindows) >= kSpecSustainWindows) ? 1u : 0u;
    CHECK(episodesSoFar(rig) == expectedEpisodes);
    CHECK(rig.backDateLines().size() == expectedEpisodes);
}

TEST_CASE("InputArrivalMargin: a join with m0 -10 for its first 10 s opens no back-dating episode",
          "[Network][InputArrivalMargin][BackDate]")
{
    constexpr std::int32_t kLinkDelay = 3;
    constexpr std::int32_t kJoinTick  = 37;
    constexpr std::int32_t kJoinM0    = -10;

    Rig rig;
    const std::int32_t window = rig.coordinator.arrivalMargin().windowTicks();

    Scenario scenario;
    scenario.numSteps        = 1;
    scenario.firstSendTick   = kJoinTick;
    scenario.leadTicks       = kLinkDelay + kJoinM0;
    scenario.leadStepTick    = kJoinTick + static_cast<std::int32_t>(10.0 * rig.config.tickFrequency);
    scenario.leadAfterStep   = kLinkDelay;
    scenario.redundancyDepth = 1;
    scenario.link            = fixedLink(kLinkDelay);
    scenario.lastServerTick  = 60 * window;
    rig.run(scenario);

    CAPTURE(rig.signals.size());
    REQUIRE(rig.signals.size() >= 59u);
    for (std::size_t i = 0; i < kSpecGraceWindows; ++i)
    {
        CAPTURE(i, rig.signals[i].samples, rig.signals[i].m0P50Ticks);
        REQUIRE(rig.signals[i].samples >= kSpecMinSamples);
        REQUIRE(rig.signals[i].m0P50Ticks == kJoinM0);
    }
    REQUIRE(rig.signals[kSpecGraceWindows].m0P50Ticks >= rig.config.backDateM0P50ThresholdTicks);

    CHECK(episodesSoFar(rig) == 0u);
    CHECK(rig.backDateLines().empty());
}

TEST_CASE("InputArrivalMargin: the class alone, the back-dating grace, sample floor, sustain and close",
          "[Network][InputArrivalMargin][BackDate]")
{
    constexpr std::int32_t kWindow   = 10;
    constexpr std::int32_t kDelay    = 6;
    const std::int32_t     threshold = TimeConfig{}.backDateM0P50ThresholdTicks;

    Margin margin(kWindow, 60.0, threshold);
    const FStandaloneTestHandle wire(1u, true);
    Margin::WindowLines out;
    std::int32_t closeTick = 0;
    REQUIRE_FALSE(margin.closeWindowIfDue(closeTick, out));

    const auto window = [&](std::uint32_t samples, std::int32_t m0) -> std::optional<ArrivalBackDateOpening>
    {
        for (std::uint32_t i = 0; i < samples; ++i)
        {
            margin.noteArrival(wire, m0, kDelay, 0, true);
        }
        closeTick += kWindow;
        REQUIRE(margin.closeWindowIfDue(closeTick, out));
        REQUIRE(out.size() == 1u);
        return out[0].backDateOpening;
    };
    const auto episodes = [&]() { return margin.signal(wire)->backDateEpisodes; };

    CHECK(margin.backDateThresholdTicks() == threshold);
    const std::int32_t below = threshold - 1;

    for (std::uint32_t w = 1; w <= kSpecGraceWindows; ++w)
    {
        CHECK_FALSE(window(kSpecMinSamples, threshold - 15).has_value());
    }
    for (std::uint32_t w = 1; w < kSpecSustainWindows; ++w)
    {
        CHECK_FALSE(window(kSpecMinSamples, below).has_value());
    }
    CHECK_FALSE(window(kSpecMinSamples - 1u, below).has_value());
    for (std::uint32_t w = 1; w < kSpecSustainWindows; ++w)
    {
        CHECK_FALSE(window(kSpecMinSamples, below).has_value());
    }
    CHECK_FALSE(window(kSpecMinSamples, threshold).has_value());
    for (std::uint32_t w = 1; w < kSpecSustainWindows; ++w)
    {
        CHECK_FALSE(window(kSpecMinSamples, below).has_value());
    }
    CHECK(episodes() == 0u);

    const std::optional<ArrivalBackDateOpening> opened = window(kSpecMinSamples, below);
    REQUIRE(opened.has_value());
    CHECK(opened->m0P50Ticks == below);
    CHECK(opened->sustainedWindows == kSpecSustainWindows);
    CHECK(opened->thresholdTicks == threshold);
    CHECK(opened->effectiveDelayTicks == kDelay);
    CHECK(episodes() == 1u);

    CHECK_FALSE(window(200u, threshold - 25).has_value());
    margin.noteLateRefused(wire);
    margin.noteLateResent(wire);
    closeTick += kWindow;
    REQUIRE(margin.closeWindowIfDue(closeTick, out));
    REQUIRE(out.size() == 1u);
    CHECK(out[0].arrivalSignal.samples == 0u);
    CHECK_FALSE(out[0].backDateOpening.has_value());
    CHECK_FALSE(window(kSpecMinSamples, below).has_value());
    CHECK(episodes() == 1u);

    CHECK_FALSE(window(1u, threshold).has_value());
    for (std::uint32_t w = 1; w < kSpecSustainWindows; ++w)
    {
        CHECK_FALSE(window(kSpecMinSamples, below).has_value());
    }
    CHECK(window(kSpecMinSamples, below).has_value());
    CHECK(episodes() == 2u);

    margin.forget(wire);
    CHECK_FALSE(margin.signal(wire).has_value());
}

TEST_CASE("InputArrivalMargin: the class alone, the grace counts each wire's own windows with samples",
          "[Network][InputArrivalMargin][BackDate]")
{
    constexpr std::int32_t kWindow   = 10;
    const std::int32_t     threshold = TimeConfig{}.backDateM0P50ThresholdTicks;

    Margin margin(kWindow, 60.0, threshold);
    const FStandaloneTestHandle early(1u, true);
    const FStandaloneTestHandle late(2u, true);
    Margin::WindowLines out;
    std::int32_t closeTick = 0;
    REQUIRE_FALSE(margin.closeWindowIfDue(closeTick, out));

    std::vector<std::uint32_t> earlyOpens;
    std::vector<std::uint32_t> lateOpens;
    for (std::uint32_t w = 1; w <= 3u * kSpecGraceWindows; ++w)
    {
        for (std::uint32_t i = 0; i < kSpecMinSamples; ++i)
        {
            margin.noteArrival(early, threshold - 1, 6, 0, true);
            if (w > kSpecGraceWindows)
            {
                margin.noteArrival(late, threshold - 1, 6, 0, true);
            }
        }
        closeTick += kWindow;
        REQUIRE(margin.closeWindowIfDue(closeTick, out));
        for (const Margin::WindowLine& line : out)
        {
            if (line.backDateOpening.has_value())
            {
                (line.address == early ? earlyOpens : lateOpens).push_back(w);
            }
        }
    }
    CHECK(earlyOpens == std::vector<std::uint32_t>{ kSpecGraceWindows + kSpecSustainWindows });
    CHECK(lateOpens == std::vector<std::uint32_t>{ 2u * kSpecGraceWindows + kSpecSustainWindows });
}
} // namespace arrivalMarginTest

#endif // WITH_LOW_LEVEL_TESTS
