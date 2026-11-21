// SPDX-License-Identifier: MPL-2.0
// docs/ClockDilationLoopTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "catch_amalgamated.hpp"
#include "ClockLoopHarness.h"

namespace clockDilationLoopTest
{
    using namespace clockLoopHarness;
    using Advance = ClientPredictionClock::AdvanceResult;

    constexpr double kSteadyAfterSettleSeconds = 10.0;
    constexpr double kPairWindowSeconds        = 120.0;
    constexpr double kDriftRunSeconds          = 602.0;
    constexpr double kHitchAtSeconds           = 30.0;
    constexpr double kHitchSeconds             = 0.6;
    constexpr double kHitchRunSeconds          = 60.0;
    constexpr double kHitchSettleSeconds       = 10.0;
    constexpr double kLawCheckFromSeconds      = 6.0;
    constexpr double kLawCheckUntilSeconds     = 6.5;
    constexpr double kLawCheckAtSeconds        = 0.5 * (kLawCheckFromSeconds + kLawCheckUntilSeconds);
    constexpr double kLawCheckMinResidualTicks = 0.5;
    constexpr double kLawToleranceTicks        = 0.09;
    constexpr double kAfterSettleMaxErrorTicks = 0.13;
    constexpr double kFortyPairMedianP95Ticks  = 0.11;
    constexpr double kThirtyFps                = 30.0;
    constexpr double kThirtyFpsPairP95Ticks    = 0.30;
    constexpr uint32_t kOffSteadyStallsPerClient = 1u;
    constexpr double kFrameJitter              = 0.2;
    constexpr size_t   kExecutionLeadClients     = 20u;
    constexpr uint64_t kExecutionLeadSeedBase    = 7001u;
    constexpr double   kHundredFpsGridTicks      = 0.2;
    constexpr double   kThirtyFpsGridTicks       = 1.0;
    constexpr double   kClientStairMarginTicks   = 0.05;

    constexpr std::pair<uint64_t, uint64_t> kGuardPairs[] = {
        { 1u, 101u }, { 1001u, 5003u }, { 1018u, 5034u }, { 1035u, 5065u },
        { 2u, 102u }, { 3u, 103u }, { 1052u, 5096u }, { 1069u, 5127u },
    };
    constexpr std::pair<uint64_t, uint64_t> kTargetPairs[] = { { 1u, 101u }, { 1001u, 5003u } };
    constexpr bool kDrainOrders[] = { true, false };
    constexpr uint64_t kHitchSeeds[] = { 1u, 2001u, 2008u, 2015u, 2022u, 3035u, 3048u, 3156u };
    constexpr uint32_t kHitchLawSeeds = 4u;
    constexpr double kCentreBiasToleranceTicks   = 0.05;
    constexpr double kChangesPerSecondClientMax  = 0.6;
    constexpr double kChangesPerSecondPooledMax  = 0.45;
    constexpr double kThirtyFpsChangesClientMax  = 2.0;
    constexpr double kThirtyFpsChangesPooledMax  = 1.3;
    constexpr double kLoopbackChangesPooledMax   = 0.3;
    constexpr uint32_t kThirtyFpsMinGuardBoundFrames = 180u;

    const char* orderWord(bool drainAtStepStart) { return drainAtStepStart ? "step-start" : "frame-start"; }

    Link loopbackLink() { return Link{ 0.0003, 0.0015 }; }
    Link cleanLink() { return Link{ 0.0003, 0.0002 }; }

    ClientSpec b0Client(uint64_t seed, double joinSeconds, ClockDilationMode mode)
    {
        ClientSpec spec;
        spec.seed               = seed;
        spec.joinSeconds        = joinSeconds;
        spec.clockOffsetSeconds = 1000.0 + 0.001 * static_cast<double>(seed % 997u);
        spec.dilation.mode      = mode;
        return spec;
    }

    std::vector<ClientSpec> pairOf(uint64_t seedA, uint64_t seedB, ClockDilationMode mode)
    {
        return { b0Client(seedA, 2.0, mode), b0Client(seedB, 2.375, mode) };
    }

    std::vector<ClientSpec> pairAt(const std::pair<uint64_t, uint64_t>& seeds, ClockDilationMode mode, double framesPerSecond,
                                   double frameJitter = 0.0, bool drainAtStepStart = ClientSpec{}.drainAtStepStart)
    {
        std::vector<ClientSpec> clients = pairOf(seeds.first, seeds.second, mode);
        for (ClientSpec& client : clients)
        {
            client.framesPerSecond  = framesPerSecond;
            client.frameJitter      = frameJitter;
            client.drainAtStepStart = drainAtStepStart;
        }
        return clients;
    }

    struct PairRun
    {
        double   steadyFrom  = 0.0;
        double   steadyUntil = 0.0;
        size_t   samples     = 0u;
        double   meanDiff    = 0.0;
        double   p95AbsDiff  = 0.0;
        uint32_t graduated   = 0u;
        uint32_t hardResyncs = 0u;
    };

    double settledSeconds(const World& world, size_t clients)
    {
        double settled = 0.0;
        for (size_t index = 0u; index < clients; ++index)
            settled = std::max(settled, world.client(index).joinSettledSeconds().value_or(1e9));
        return settled;
    }

    uint32_t graduatedAfter(const LoopClient& client, double seconds)
    {
        return client.countAfter(seconds, Advance::Skip) + client.countAfter(seconds, Advance::Stall);
    }

    PairRun runPair(World& world, double windowSeconds)
    {
        world.runUntil(20.0);
        PairRun run;
        run.steadyFrom  = settledSeconds(world, 2u) + kSteadyAfterSettleSeconds;
        run.steadyUntil = run.steadyFrom + windowSeconds;
        world.runUntil(run.steadyUntil);

        std::vector<double> diffs;
        std::vector<double> absDiffs;
        for (const PairSample& row : world.samples())
        {
            if (row.seconds < run.steadyFrom || row.seconds > run.steadyUntil || !row.started[0] || !row.started[1])
                continue;
            diffs.push_back(row.leadTicks[0] - row.leadTicks[1]);
            absDiffs.push_back(std::abs(row.leadTicks[0] - row.leadTicks[1]));
        }
        run.samples    = diffs.size();
        run.meanDiff   = meanOf(diffs);
        run.p95AbsDiff = percentileOf(absDiffs, 95.0);
        for (size_t index = 0u; index < 2u; ++index)
        {
            run.graduated += graduatedAfter(world.client(index), run.steadyFrom);
            run.hardResyncs += world.client(index).countAfter(run.steadyFrom, Advance::HardResync);
        }
        return run;
    }

    struct ClientSteady
    {
        uint32_t skips            = 0u;
        uint32_t stalls           = 0u;
        uint32_t hardResyncs      = 0u;
        uint32_t guardBoundFrames = 0u;
        double   leadMean         = 0.0;

        uint32_t corrections() const { return skips + stalls + hardResyncs; }
    };

    ClientSteady steadyOf(const LoopClient& client, const PairRun& run)
    {
        ClientSteady steady;
        steady.skips       = client.countAfter(run.steadyFrom, Advance::Skip);
        steady.stalls      = client.countAfter(run.steadyFrom, Advance::Stall);
        steady.hardResyncs = client.countAfter(run.steadyFrom, Advance::HardResync);
        std::vector<double> leads;
        for (const FrameRecord& frame : client.frames())
        {
            if (frame.seconds < run.steadyFrom || frame.seconds > run.steadyUntil)
                continue;
            leads.push_back(frame.trueLeadTicks);
            steady.guardBoundFrames += frame.out.guardBound ? 1u : 0u;
        }
        steady.leadMean = meanOf(leads);
        return steady;
    }

    const char* fieldStart(const std::string& line, const char* key)
    {
        const char* const found = std::strstr(line.c_str(), key);
        return found == nullptr ? nullptr : found + std::strlen(key);
    }

    double fieldPart(const std::string& line, const char* key, int part)
    {
        const char* cursor = fieldStart(line, key);
        if (cursor == nullptr)
            return std::nan("");
        for (int skip = 0; skip < part; ++skip)
        {
            cursor = std::strchr(cursor, '/');
            if (cursor == nullptr)
                return std::nan("");
            ++cursor;
        }
        return std::strtod(cursor, nullptr);
    }

    std::vector<const WindowLine*> windowsWithin(const LoopClient& client, double from, double until)
    {
        std::vector<const WindowLine*> lines;
        for (const WindowLine& line : client.windowLines())
        {
            if (line.seconds - clockDilation::kWindowSeconds >= from && line.seconds <= until)
                lines.push_back(&line);
        }
        return lines;
    }

    ClockDilationOutput outputOf(double scale, bool apply, double lead, double target, DilationFreeze freeze,
                                 bool guardBound = false)
    {
        ClockDilationOutput out;
        out.scale       = scale;
        out.apply       = apply;
        out.leadTicks   = lead;
        out.targetTicks = target;
        out.errorTicks  = lead - target;
        out.freeze      = freeze;
        out.guardBound  = guardBound;
        return out;
    }

    ClockDilationSensorCounters sensorOf(uint32_t accepted, uint32_t rejected, uint32_t reseeds, double lastSampleSeconds)
    {
        ClockDilationSensorCounters counters;
        counters.acceptedSamples   = accepted;
        counters.rejectedSamples   = rejected;
        counters.reseeds           = reseeds;
        counters.lastSampleSeconds = lastSampleSeconds;
        return counters;
    }

    double stepLagOf(const ClockDilationOutput& out) { return out.stepLagTicks; }

    struct ExecutionLeadSet
    {
        std::vector<double> offsets;
        double              meanOffset  = 0.0;
        uint32_t            corrections = 0u;
    };

    ExecutionLeadSet executionLeadSet(double framesPerSecond, uint64_t seedBase)
    {
        std::vector<ClientSpec> clients;
        for (size_t index = 0u; index < kExecutionLeadClients; ++index)
        {
            const double phase = static_cast<double>(index) / static_cast<double>(kExecutionLeadClients);
            ClientSpec spec = b0Client(seedBase + index, 2.0 + phase / framesPerSecond, ClockDilationMode::On);
            spec.framesPerSecond = framesPerSecond;
            spec.frameJitter     = kFrameJitter;
            clients.push_back(spec);
        }
        World world(WorldSpec{}, clients);
        world.runUntil(20.0);
        const double from  = settledSeconds(world, clients.size()) + kSteadyAfterSettleSeconds;
        const double until = from + kPairWindowSeconds;
        world.runUntil(until);

        ExecutionLeadSet set;
        for (size_t index = 0u; index < clients.size(); ++index)
        {
            const LoopClient& client = world.client(index);
            std::vector<double> executionLeads;
            for (const StepRecord& step : client.steps())
            {
                if (step.seconds < from || step.seconds > until)
                    continue;
                executionLeads.push_back(step.executionLeadTicks);
                set.corrections += step.result != Advance::Normal ? 1u : 0u;
            }
            std::vector<double> executionTargets;
            for (const FrameRecord& frame : client.frames())
            {
                if (frame.seconds >= from && frame.seconds <= until && frame.dilated
                    && frame.out.freeze == DilationFreeze::None)
                    executionTargets.push_back(frame.out.targetTicks - stepLagOf(frame.out));
            }
            set.offsets.push_back(meanOf(executionLeads) - meanOf(executionTargets));
        }
        set.meanOffset = meanOf(set.offsets);
        return set;
    }

    double doorChangesPerSecond(const LoopClient& client, const PairRun& run)
    {
        double   previous = 1.0;
        uint32_t changes  = 0u;
        for (const FrameRecord& frame : client.frames())
        {
            if (frame.seconds > run.steadyUntil)
                break;
            changes += frame.seconds >= run.steadyFrom && frame.rateScale != previous ? 1u : 0u;
            previous = frame.rateScale;
        }
        return static_cast<double>(changes) / (run.steadyUntil - run.steadyFrom);
    }

    struct CentreBias
    {
        double   mean   = 0.0;
        uint32_t frames = 0u;
    };

    CentreBias centreBiasOf(const LoopClient& client, const PairRun& run)
    {
        std::vector<double> biases;
        for (const FrameRecord& frame : client.frames())
        {
            if (frame.seconds < run.steadyFrom || frame.seconds > run.steadyUntil || frame.steps == 0u || !frame.dilated
                || frame.out.freeze != DilationFreeze::None)
                continue;
            biases.push_back(frame.out.guardCentreTicks - frame.readCentreTicks);
        }
        return CentreBias{ meanOf(biases), static_cast<uint32_t>(biases.size()) };
    }
} // namespace clockDilationLoopTest

using namespace clockDilationLoopTest;

TEST_CASE("PCTM.ClockDilationLoop.TwoClientsHoldEqualDeadlineLeads", "[PCTM][Dilation][ClockDilationLoop]")
{
    World world(WorldSpec{}, pairOf(1u, 101u, ClockDilationMode::On));
    const PairRun run = runPair(world, kPairWindowSeconds);
    CAPTURE(run.steadyFrom, run.meanDiff, run.p95AbsDiff, run.graduated, run.hardResyncs);

    REQUIRE(run.samples >= static_cast<size_t>(kPairWindowSeconds / WorldSpec{}.sampleEverySeconds));
    CHECK(run.p95AbsDiff <= 0.10);
    CHECK(run.graduated == 0u);
    CHECK(run.hardResyncs == 0u);
}

TEST_CASE("PCTM.ClockDilationLoop.EqualLeadsAcrossFortySeedPairs", "[PCTM][Dilation][ClockDilationLoop]")
{
    std::vector<double> p95s;
    std::vector<double> means;
    for (uint64_t seed = 1u; seed <= 40u; ++seed)
    {
        World world(WorldSpec{}, pairOf(seed, seed + 100u, ClockDilationMode::On));
        const PairRun run = runPair(world, kPairWindowSeconds);
        CAPTURE(seed, run.meanDiff, run.p95AbsDiff);
        CHECK(std::abs(run.meanDiff) <= 0.10);
        CHECK(run.p95AbsDiff <= 0.25);
        CHECK(run.graduated == 0u);
        p95s.push_back(run.p95AbsDiff);
        means.push_back(run.meanDiff);
    }
    const double median      = percentileOf(p95s, 50.0);
    const double meanOfMeans = meanOf(means);
    CAPTURE(median, percentileOf(p95s, 100.0), meanOfMeans);
    CHECK(median <= kFortyPairMedianP95Ticks);
    CHECK(std::abs(meanOfMeans) <= 0.02);
}

TEST_CASE("PCTM.ClockDilationLoop.OffKeepsTheJoinResiduals", "[PCTM][Dilation][ClockDilationLoop]")
{
    World world(WorldSpec{}, pairOf(1u, 101u, ClockDilationMode::Off));
    const PairRun run = runPair(world, kPairWindowSeconds);
    CAPTURE(run.meanDiff, run.p95AbsDiff);

    CHECK(std::abs(run.meanDiff) >= 0.4);
    CHECK(run.p95AbsDiff > 0.10);
    CHECK(run.graduated == 0u);
    CHECK(world.client(0).windowLines().empty());
}

TEST_CASE("PCTM.ClockDilationLoop.HundredPpmDriftIsHeldWithoutGraduatedCorrections", "[PCTM][Dilation][ClockDilationLoop]")
{
    for (ClockDilationMode mode : { ClockDilationMode::On, ClockDilationMode::Off })
    {
        std::vector<ClientSpec> clients = pairOf(1u, 101u, mode);
        clients[0].clockRatePpm         = 100.0;
        World world(WorldSpec{}, clients);
        world.runUntil(20.0);
        const double from = settledSeconds(world, 2u) + kSteadyAfterSettleSeconds;
        world.runUntil(kDriftRunSeconds);
        const LoopClient& drifting = world.client(0);
        CAPTURE(clockDilation::modeWord(mode), from);

        if (mode == ClockDilationMode::Off)
        {
            CHECK(drifting.countAfter(from, Advance::Stall) >= 1u);
            continue;
        }

        std::vector<double> errors;
        for (const FrameRecord& frame : drifting.frames())
        {
            if (frame.seconds >= from && frame.dilated)
                errors.push_back(frame.out.errorTicks);
        }
        const double meanError = meanOf(errors);
        std::vector<double> absErrors;
        for (double error : errors)
            absErrors.push_back(std::abs(error));
        CAPTURE(meanError, errors.size(), percentileOf(absErrors, 95.0), percentileOf(absErrors, 100.0));
        CHECK(std::abs(meanError) <= 0.05);
        CHECK(percentileOf(absErrors, 95.0) <= 0.10);
        for (size_t index = 0u; index < 2u; ++index)
        {
            CHECK(graduatedAfter(world.client(index), from) == 0u);
            CHECK(world.client(index).countAfter(from, Advance::HardResync) == 0u);
        }
    }
}

TEST_CASE("PCTM.ClockDilationLoop.SixHundredMillisecondHitchRecovers", "[PCTM][Dilation][ClockDilationLoop]")
{
    const ClockDilationConfig defaults;
    const double saturationTicks = defaults.maxDeviation / defaults.gainPerTick;
    uint32_t lawChecks = 0u;
    uint32_t runs      = 0u;
    for (bool drainAtStepStart : kDrainOrders)
    {
        for (uint64_t seed : kHitchSeeds)
        {
            ClientSpec spec       = b0Client(seed, 2.0, ClockDilationMode::On);
            spec.hitch.atSeconds  = kHitchAtSeconds;
            spec.hitch.seconds    = kHitchSeconds;
            spec.drainAtStepStart = drainAtStepStart;
            World world(WorldSpec{}, { spec });
            world.runUntil(kHitchRunSeconds);
            const LoopClient& client = world.client(0);

            CHECK(client.lostHostSeconds() == Catch::Approx(kHitchSeconds - kMaxUndilatedFrameSeconds).margin(1e-9));
            double lastCorrection = -1.0;
            uint32_t corrections  = 0u;
            for (const CorrectionEvent& event : client.corrections())
            {
                if (event.seconds >= kHitchAtSeconds)
                {
                    lastCorrection = event.seconds;
                    ++corrections;
                }
            }
            CAPTURE(orderWord(drainAtStepStart), seed, corrections, lastCorrection,
                client.countAfter(kHitchAtSeconds, Advance::Skip), client.countAfter(kHitchAtSeconds, Advance::Stall),
                client.countAfter(kHitchAtSeconds, Advance::HardResync));
            REQUIRE(corrections >= 1u);
            REQUIRE(lastCorrection + kLawCheckFromSeconds < kHitchRunSeconds - kHitchSettleSeconds);

            double residual    = 0.0;
            double sumInWindow = 0.0;
            uint32_t inWindowN = 0u;
            double afterTen    = 0.0;
            for (const FrameRecord& frame : client.frames())
            {
                if (frame.seconds <= lastCorrection || !frame.dilated)
                    continue;
                if (residual == 0.0 && frame.out.freeze == DilationFreeze::None)
                    residual = std::abs(frame.out.errorTicks);
                if (frame.seconds >= lastCorrection + kLawCheckFromSeconds
                    && frame.seconds <= lastCorrection + kLawCheckUntilSeconds)
                {
                    sumInWindow += std::abs(frame.out.errorTicks);
                    ++inWindowN;
                }
                if (frame.seconds >= lastCorrection + kHitchSettleSeconds)
                    afterTen = std::max(afterTen, std::abs(frame.out.errorTicks));
            }
            REQUIRE(inWindowN > 0u);
            const double meanInWindow = sumInWindow / static_cast<double>(inWindowN);
            const double lawAtMid     = residual * std::exp(-kLawCheckAtSeconds * kTickHz * defaults.gainPerTick);
            CAPTURE(residual, lawAtMid, meanInWindow, inWindowN, afterTen);
            CHECK(residual < saturationTicks);
            if (residual >= kLawCheckMinResidualTicks)
            {
                CHECK(std::abs(meanInWindow - lawAtMid) <= kLawToleranceTicks);
                ++lawChecks;
            }
            CHECK(afterTen <= kAfterSettleMaxErrorTicks);
            ++runs;
        }
    }
    CHECK(runs == 2u * std::size(kHitchSeeds));
    CHECK(lawChecks == 2u * kHitchLawSeeds);
}

TEST_CASE("PCTM.ClockDilationLoop.TheAppliedScaleStaysInsideOnePercent", "[PCTM][Dilation][ClockDilationLoop]")
{
    World pair(WorldSpec{}, pairOf(1u, 101u, ClockDilationMode::On));
    const PairRun run = runPair(pair, kPairWindowSeconds);

    ClientSpec hitch      = b0Client(1u, 2.0, ClockDilationMode::On);
    hitch.hitch.atSeconds = kHitchAtSeconds;
    hitch.hitch.seconds   = kHitchSeconds;
    World hitched(WorldSpec{}, { hitch });
    hitched.runUntil(kHitchRunSeconds);

    for (const LoopClient* client : { &pair.client(0), &pair.client(1), &hitched.client(0) })
    {
        double low  = 1.0;
        double high = 1.0;
        for (const FrameRecord& frame : client->frames())
        {
            low  = std::min(low, frame.rateScale);
            high = std::max(high, frame.rateScale);
        }
        CAPTURE(low, high);
        CHECK(low >= 0.99);
        CHECK(high <= 1.01);
        CHECK(client->refusedRateChanges() == 0u);
    }

    for (size_t index = 0u; index < 2u; ++index)
    {
        std::vector<double> deviations;
        for (const FrameRecord& frame : pair.client(index).frames())
        {
            if (frame.seconds >= run.steadyFrom && frame.seconds <= run.steadyUntil)
                deviations.push_back(std::abs(frame.out.scale - 1.0) * 100.0);
        }
        CAPTURE(index, percentileOf(deviations, 99.0));
        CHECK(percentileOf(deviations, 99.0) <= 0.05);
        const std::vector<const WindowLine*> lines = windowsWithin(pair.client(index), run.steadyFrom, run.steadyUntil);
        CHECK(lines.size() >= 11u);
        for (const WindowLine* line : lines)
        {
            CAPTURE(line->text);
            CHECK(fieldPart(line->text, " wouldScaleDevP99=", 0) <= 0.05);
        }
    }
}

TEST_CASE("PCTM.ClockDilationLoop.ThirtyFpsOnHoldsTheBand", "[PCTM][Dilation][ClockDilationLoop]")
{
    uint32_t ordersRun = 0u;
    for (bool drainAtStepStart : kDrainOrders)
    {
        CAPTURE(orderWord(drainAtStepStart));
        uint32_t offCorrections = 0u;
        uint32_t onCorrections  = 0u;
        for (const std::pair<uint64_t, uint64_t>& seeds : kGuardPairs)
        {
            double offLead[2] = { 0.0, 0.0 };
            for (ClockDilationMode mode : { ClockDilationMode::Off, ClockDilationMode::Shadow, ClockDilationMode::On })
            {
                World world(WorldSpec{}, pairAt(seeds, mode, kThirtyFps, 0.0, drainAtStepStart));
                const PairRun run = runPair(world, kPairWindowSeconds);
                CAPTURE(seeds.first, seeds.second, clockDilation::modeWord(mode), run.steadyFrom, run.meanDiff,
                    run.p95AbsDiff);

                for (size_t index = 0u; index < 2u; ++index)
                {
                    const ClientSteady steady = steadyOf(world.client(index), run);
                    CAPTURE(index, steady.leadMean, steady.skips, steady.stalls, steady.hardResyncs,
                        steady.guardBoundFrames);
                    CHECK(steady.hardResyncs == 0u);
                    if (mode == ClockDilationMode::On)
                    {
                        CHECK(steady.skips == 0u);
                        CHECK(steady.stalls == 0u);
                        CHECK(steady.guardBoundFrames >= kThirtyFpsMinGuardBoundFrames);
                        CHECK(steady.leadMean <= offLead[index]);
                        onCorrections += steady.corrections();
                        continue;
                    }
                    CHECK(steady.skips == 0u);
                    CHECK(steady.stalls <= kOffSteadyStallsPerClient);
                    if (mode == ClockDilationMode::Off)
                    {
                        offLead[index] = steady.leadMean;
                        offCorrections += steady.corrections();
                    }
                    else
                    {
                        CHECK(steady.leadMean == Catch::Approx(offLead[index]).margin(1e-9));
                    }
                }
                if (mode == ClockDilationMode::On)
                    CHECK(run.p95AbsDiff <= kThirtyFpsPairP95Ticks);
            }
        }
        CAPTURE(offCorrections, onCorrections);
        CHECK(onCorrections <= offCorrections);
        ++ordersRun;
    }
    CHECK(ordersRun == 2u);
}

TEST_CASE("PCTM.ClockDilationLoop.TheGuardHoldsAnyTarget", "[PCTM][Dilation][ClockDilationLoop]")
{
    uint32_t armsRun = 0u;
    for (bool drainAtStepStart : kDrainOrders)
    {
        for (double framesPerSecond : { kThirtyFps, 100.0 })
        {
            for (int arm = 0; arm < 6; ++arm)
            {
                const double offsets[] = { -0.5, -1.0, -2.0, -3.0 };
                for (const std::pair<uint64_t, uint64_t>& seeds : kTargetPairs)
                {
                    std::vector<ClientSpec> clients
                        = pairAt(seeds, ClockDilationMode::On, framesPerSecond, 0.0, drainAtStepStart);
                    for (ClientSpec& client : clients)
                    {
                        if (arm < 4)
                        {
                            client.dilation.targetOffsetTicks = offsets[arm];
                            continue;
                        }
                        client.dilation.setpoint        = ClockDilationSetpoint::Common;
                        client.dilation.commonLeadTicks = arm == 4 ? 0.5 : 1.5;
                    }
                    World world(WorldSpec{}, clients);
                    const PairRun run = runPair(world, kPairWindowSeconds);
                    CAPTURE(orderWord(drainAtStepStart), framesPerSecond, arm, clients[0].dilation.targetOffsetTicks,
                        clients[0].dilation.commonLeadTicks, clockDilation::setpointWord(clients[0].dilation.setpoint),
                        seeds.first, seeds.second, run.p95AbsDiff);
                    CHECK(run.graduated == 0u);
                    CHECK(run.hardResyncs == 0u);
                    ++armsRun;
                }
            }
        }
    }
    CHECK(armsRun == 48u);
}

TEST_CASE("PCTM.ClockDilationLoop.OtherFrameRatesHoldTheBand", "[PCTM][Dilation][ClockDilationLoop]")
{
    uint32_t runs = 0u;
    for (bool drainAtStepStart : kDrainOrders)
    {
        for (double framesPerSecond : { 45.0, 60.0, 144.0 })
        {
            for (const std::pair<uint64_t, uint64_t>& seeds : kGuardPairs)
            {
                World world(WorldSpec{}, pairAt(seeds, ClockDilationMode::On, framesPerSecond, 0.0, drainAtStepStart));
                const PairRun run = runPair(world, kPairWindowSeconds);
                CAPTURE(orderWord(drainAtStepStart), framesPerSecond, seeds.first, seeds.second, run.p95AbsDiff);
                CHECK(run.graduated == 0u);
                CHECK(run.hardResyncs == 0u);
                ++runs;
            }
        }
    }
    CHECK(runs == 48u);
}

TEST_CASE("PCTM.ClockDilationLoop.FrameJitterLeavesOnNoWorseThanOff", "[PCTM][Dilation][ClockDilationLoop]")
{
    for (bool drainAtStepStart : kDrainOrders)
    {
        uint32_t corrections[2] = { 0u, 0u };
        for (ClockDilationMode mode : { ClockDilationMode::Off, ClockDilationMode::On })
        {
            for (const std::pair<uint64_t, uint64_t>& seeds : kGuardPairs)
            {
                World world(WorldSpec{}, pairAt(seeds, mode, kThirtyFps, kFrameJitter, drainAtStepStart));
                const PairRun run = runPair(world, kPairWindowSeconds);
                corrections[mode == ClockDilationMode::On ? 1 : 0] += run.graduated + run.hardResyncs;
            }
        }
        CAPTURE(orderWord(drainAtStepStart), corrections[0], corrections[1]);
        CHECK(corrections[0] >= 1u);
        CHECK(corrections[1] <= corrections[0]);
    }
}

TEST_CASE("PCTM.ClockDilationLoop.TheGuardCentreIsTheClocksRead", "[PCTM][Dilation][ClockDilationLoop]")
{
    uint32_t cells = 0u;
    for (bool drainAtStepStart : kDrainOrders)
    {
        for (bool clean : { false, true })
        {
            for (double framesPerSecond : { kThirtyFps, 60.0, 100.0, 144.0 })
            {
                std::vector<double> clientMeans;
                for (const std::pair<uint64_t, uint64_t>& seeds : kGuardPairs)
                {
                    std::vector<ClientSpec> clients
                        = pairAt(seeds, ClockDilationMode::On, framesPerSecond, 0.0, drainAtStepStart);
                    for (ClientSpec& client : clients)
                    {
                        if (!clean)
                            continue;
                        client.up   = cleanLink();
                        client.down = cleanLink();
                    }
                    World world(WorldSpec{}, clients);
                    const PairRun run = runPair(world, kPairWindowSeconds);
                    for (size_t index = 0u; index < 2u; ++index)
                    {
                        const CentreBias bias = centreBiasOf(world.client(index), run);
                        REQUIRE(bias.frames > 1000u);
                        clientMeans.push_back(bias.mean);
                    }
                }
                const double mean = meanOf(clientMeans);
                CAPTURE(orderWord(drainAtStepStart), clean, framesPerSecond, mean, percentileOf(clientMeans, 0.0),
                    percentileOf(clientMeans, 100.0));
                CHECK(std::abs(mean) <= kCentreBiasToleranceTicks);
                ++cells;
            }
        }
    }
    CHECK(cells == 16u);
}

TEST_CASE("PCTM.ClockDilationLoop.QuantumChangesStayUnderTheBound", "[PCTM][Dilation][ClockDilationLoop]")
{
    struct Arm
    {
        double framesPerSecond;
        bool   loopback;
        double clientMax;
        double pooledMax;
    };
    const Arm arms[] = {
        { 100.0, false, kChangesPerSecondClientMax, kChangesPerSecondPooledMax },
        { 60.0, false, kChangesPerSecondClientMax, kChangesPerSecondPooledMax },
        { 144.0, false, kChangesPerSecondClientMax, kChangesPerSecondPooledMax },
        { kThirtyFps, false, kThirtyFpsChangesClientMax, kThirtyFpsChangesPooledMax },
        { 100.0, true, std::numeric_limits<double>::infinity(), kLoopbackChangesPooledMax },
    };
    uint32_t armsRun = 0u;
    for (const Arm& arm : arms)
    {
        std::vector<double> rates;
        for (const std::pair<uint64_t, uint64_t>& seeds : kGuardPairs)
        {
            std::vector<ClientSpec> clients = pairAt(seeds, ClockDilationMode::On, arm.framesPerSecond);
            for (ClientSpec& client : clients)
            {
                if (!arm.loopback)
                    continue;
                client.up   = loopbackLink();
                client.down = loopbackLink();
            }
            World world(WorldSpec{}, clients);
            const PairRun run = runPair(world, kPairWindowSeconds);
            for (size_t index = 0u; index < 2u; ++index)
            {
                const double rate = doorChangesPerSecond(world.client(index), run);
                CAPTURE(arm.framesPerSecond, arm.loopback, seeds.first, seeds.second, index, rate);
                CHECK(rate <= arm.clientMax);
                rates.push_back(rate);
            }
        }
        CAPTURE(arm.framesPerSecond, arm.loopback, meanOf(rates), percentileOf(rates, 100.0));
        CHECK(meanOf(rates) <= arm.pooledMax);
        ++armsRun;
    }
    CHECK(armsRun == 5u);
}

TEST_CASE("PCTM.ClockDilationLoop.RenderTimeStaysBracketedUnderDilation", "[PCTM][Dilation][ClockDilationLoop]")
{
    std::vector<ClientSpec> clients = pairOf(1u, 101u, ClockDilationMode::On);
    for (ClientSpec& client : clients)
        client.render = true;
    World world(WorldSpec{}, clients);
    world.runUntil(150.0);

    for (size_t index = 0u; index < 2u; ++index)
    {
        const RenderStats& render = world.client(index).renderStats();
        CAPTURE(index, render.checkedFrames, render.beyondOldest, render.renderTimeBacks, render.minAdvanceSeconds);
        CHECK(render.checkedFrames >= 14000u);
        CHECK(render.beyondOldest == 0u);
        CHECK(render.renderTimeBacks == 0u);
    }
}

TEST_CASE("PCTM.ClockDilationLoop.AnInstantScaleStepLeavesTheBracket", "[PCTM][Dilation][ClockDilationLoop]")
{
    ClientSpec spec   = b0Client(1u, 2.0, ClockDilationMode::On);
    spec.render       = true;
    for (int trial = 0; trial < 10; ++trial)
    {
        spec.forcedScales.emplace_back(20.0 + trial, 1.25);
        spec.forcedScales.emplace_back(20.5 + trial, 0.75);
    }
    World world(WorldSpec{}, { spec });
    world.runUntil(31.0);

    const RenderStats& render = world.client(0).renderStats();
    CAPTURE(render.checkedFrames, render.beyondOldest, render.renderTimeBacks, render.minAdvanceSeconds);
    CHECK(world.client(0).scheduler().rateScale() == 0.75);
    CHECK(render.beyondOldest >= 1u);
    CHECK(render.renderTimeBacks >= 1u);
}

TEST_CASE("PCTM.ClockDilationLoop.ShadowNeverMovesTheDoor", "[PCTM][Dilation][ClockDilationLoop]")
{
    World world(WorldSpec{}, pairOf(1u, 101u, ClockDilationMode::Shadow));
    world.runUntil(60.0);

    for (size_t index = 0u; index < 2u; ++index)
    {
        const LoopClient& client = world.client(index);
        bool scaleMoved = false;
        for (const FrameRecord& frame : client.frames())
        {
            CHECK(frame.rateScale == 1.0);
            CHECK_FALSE(frame.out.apply);
            scaleMoved = scaleMoved || frame.out.scale != 1.0;
        }
        CHECK(scaleMoved);
        REQUIRE_FALSE(client.windowLines().empty());
        for (const WindowLine& line : client.windowLines())
        {
            CAPTURE(line.text);
            CHECK(std::strstr(line.text.c_str(), " applied=0 ") != nullptr);
            CHECK(std::strstr(line.text.c_str(), " rateScale=1.00000/1.00000 ") != nullptr);
            CHECK(std::strstr(line.text.c_str(), "[Dilation.Window] mode=Shadow setpoint=OwnRtt ") == line.text.c_str());
        }
    }
}

TEST_CASE("PCTM.ClockDilationLoop.ExecutionLeadMeetsTheTarget", "[PCTM][Dilation][ClockDilationLoop]")
{
    struct Arm
    {
        double framesPerSecond;
        double meanToleranceTicks;
        double gridTicks;
    };
    const Arm arms[] = { { 100.0, 0.03, kHundredFpsGridTicks }, { kThirtyFps, 0.10, kThirtyFpsGridTicks } };
    uint32_t armsRun = 0u;
    for (const Arm& arm : arms)
    {
        const ExecutionLeadSet set = executionLeadSet(arm.framesPerSecond, kExecutionLeadSeedBase);
        const double clientTolerance = arm.gridTicks / 2.0 + kClientStairMarginTicks;
        CAPTURE(arm.framesPerSecond, set.meanOffset, percentileOf(set.offsets, 0.0), percentileOf(set.offsets, 100.0),
            set.corrections);
        REQUIRE(set.offsets.size() == kExecutionLeadClients);
        CHECK(std::abs(set.meanOffset) <= arm.meanToleranceTicks);
        for (size_t index = 0u; index < set.offsets.size(); ++index)
        {
            CAPTURE(index, set.offsets[index]);
            CHECK(std::abs(set.offsets[index]) <= clientTolerance);
        }
        ++armsRun;
    }
    CHECK(armsRun == 2u);
}

TEST_CASE("PCTM.ClockDilationLoop.WouldChangesCountsTheControllersScaleChanges", "[PCTM][Dilation][ClockDilationLoop]")
{
    for (ClockDilationMode mode : { ClockDilationMode::On, ClockDilationMode::Shadow })
    {
        World world(WorldSpec{}, pairOf(1u, 101u, mode));
        world.runUntil(60.0);
        for (size_t index = 0u; index < 2u; ++index)
        {
            const LoopClient& client = world.client(index);
            REQUIRE_FALSE(client.windowLines().empty());
            double lineChanges = 0.0;
            for (const WindowLine& line : client.windowLines())
            {
                CAPTURE(line.text);
                lineChanges += fieldPart(line.text, " wouldChanges=", 0);
                const double applied = fieldPart(line.text, " applied=", 0);
                if (mode == ClockDilationMode::On)
                    CHECK(fieldPart(line.text, " wouldChanges=", 0) == applied);
                else
                    CHECK(applied == 0.0);
                const auto at = std::find_if(client.frames().begin(), client.frames().end(),
                    [&](const FrameRecord& frame) { return frame.seconds == line.seconds; });
                REQUIRE(at != client.frames().end());
                CHECK(std::abs(fieldPart(line.text, " stepLag=", 0) - stepLagOf(at->out)) <= 0.0005);
            }

            const double lastLineSeconds = client.windowLines().back().seconds;
            double previous      = 1.0;
            uint32_t frameChanges = 0u;
            for (const FrameRecord& frame : client.frames())
            {
                if (frame.seconds > lastLineSeconds || !frame.dilated)
                    continue;
                frameChanges += frame.out.scale != previous ? 1u : 0u;
                previous      = frame.out.scale;
            }
            CAPTURE(clockDilation::modeWord(mode), index, lineChanges, frameChanges);
            CHECK(frameChanges > 0u);
            CHECK(lineChanges == static_cast<double>(frameChanges));
        }
    }
}

TEST_CASE("PCTM.ClockDilationLoop.TheSensorEstimatesThePublishedTick", "[PCTM][Dilation][ClockDilationLoop]")
{
    for (double stepSeconds : { 0.0, 0.0015 })
    {
        WorldSpec spec;
        spec.serverStepSeconds = stepSeconds;
        World world(spec, { b0Client(1u, 2.0, ClockDilationMode::On) });
        world.runUntil(60.0);

        std::vector<double> errors;
        for (const FrameRecord& frame : world.client(0).frames())
        {
            if (frame.seconds >= 15.0 && frame.sensorValid)
                errors.push_back(frame.sensorErrorTicks);
        }
        const double bias = meanOf(errors);
        CAPTURE(stepSeconds, bias, errors.size());
        CHECK(bias == Catch::Approx(-stepSeconds * kTickHz).margin(0.01));
    }
}

TEST_CASE("PCTM.ClockDilationLoop.KeptSeriesDecimatesAndRanksNearest", "[PCTM][Dilation][ClockDilationLoop]")
{
    clockDilation::KeptSeries series;
    for (uint32_t value = 0u; value < 4096u; ++value)
        series.add(static_cast<double>(value));
    CHECK(series.count() == 1024u);
    CHECK(series.stride() == 4u);

    clockDilation::KeptValues scratch{};
    const clockDilation::KeptValues& sorted = series.sortedInto(scratch);
    CHECK(sorted[0] == 0.0);
    CHECK(sorted[1] == 4.0);
    CHECK(sorted[1023] == 4092.0);
    CHECK(clockDilation::nearestRank(sorted, series.count(), 50u) == 2044.0);
    CHECK(clockDilation::nearestRank(sorted, series.count(), 99u) == 4052.0);
    CHECK(clockDilation::nearestRank(sorted, 0u, 50u) == 0.0);

    series.add(4096.0);
    series.add(4097.0);
    CHECK(series.count() == 513u);
    CHECK(series.stride() == 8u);
    series.clear();
    series.add(7.0);
    CHECK(series.count() == 1u);
    CHECK(series.stride() == 1u);
}

TEST_CASE("PCTM.ClockDilationLoop.AScriptedWindowIsTheGoldenLine", "[PCTM][Dilation][ClockDilationLoop]")
{
    ClockDilationConfig config;
    config.mode = ClockDilationMode::On;
    ClockDilationWindow window(config);
    CHECK_FALSE(window.isDue(100.0));

    window.note(outputOf(1.0, false, 0.0, 3.497, DilationFreeze::SensorInvalid), 1.0, sensorOf(5u, 1u, 0u, 99.98), 100.00,
                6000.25, 6000.5);
    window.note(outputOf(0.9998, true, 2.5, 2.4, DilationFreeze::None), 0.9998, sensorOf(6u, 1u, 0u, 100.005), 100.01,
                6000.85, 6001.1);
    window.note(outputOf(0.9996, true, 2.45, 2.4, DilationFreeze::None, true), 0.9996, sensorOf(7u, 2u, 1u, 100.015),
                100.02, 6001.45, 6001.7);
    window.note(outputOf(0.9998, true, 2.6, 2.4, DilationFreeze::AfterResync), 0.9998, sensorOf(7u, 2u, 1u, 100.015),
                100.03, 6002.05, 6002.3);
    window.note(outputOf(1.0002, true, 2.3, 2.45, DilationFreeze::None), 1.0002, sensorOf(8u, 2u, 1u, 100.035), 100.04,
                6002.65, 6002.9);

    CHECK(window.calls() == 5u);
    CHECK_FALSE(window.isDue(109.99));
    CHECK(window.isDue(110.0));
    CHECK(std::string(window.formatLine())
          == "[Dilation.Window] mode=On setpoint=OwnRtt calls=5 lead=2.300/2.450/2.500 target=2.417 "
             "err=-0.150/0.050/0.100 errAbsMax=0.150 wouldScale=0.99960/0.99988/1.00020 wouldScaleDevP99=0.0400 "
             "rateScale=0.99960/1.00020 applied=4 frozen=2 frozenBy=sensor:1,recentre:0,debt:0,firstRtt:0,resync:1 "
             "guardBound=1 sensorSamples=8 sensorRejected=2 sensorReseeds=1 sensorAgeMsMax=20.0 sHatClock=100.040000 "
             "sHat=6002.650 sHatNoAge=6002.900 wouldChanges=4 stepLag=0.000");

    window.restart();
    CHECK(window.calls() == 0u);
    CHECK_FALSE(window.isDue(110.03));
    CHECK(window.isDue(110.05));
    window.note(outputOf(1.0002, false, 2.4, 2.45, DilationFreeze::None), 1.0002, sensorOf(10u, 3u, 1u, 100.045), 100.05,
                6003.25, 6003.5);
    CHECK(std::string(window.formatLine())
          == "[Dilation.Window] mode=On setpoint=OwnRtt calls=1 lead=2.400/2.400/2.400 target=2.450 "
             "err=-0.050/-0.050/-0.050 errAbsMax=0.050 wouldScale=1.00020/1.00020/1.00020 wouldScaleDevP99=0.0200 "
             "rateScale=1.00020/1.00020 applied=0 frozen=0 frozenBy=sensor:0,recentre:0,debt:0,firstRtt:0,resync:0 "
             "guardBound=0 sensorSamples=2 sensorRejected=1 sensorReseeds=0 sensorAgeMsMax=5.0 sHatClock=100.050000 "
             "sHat=6003.250 sHatNoAge=6003.500 wouldChanges=0 stepLag=0.000");
}

#endif // WITH_LOW_LEVEL_TESTS
