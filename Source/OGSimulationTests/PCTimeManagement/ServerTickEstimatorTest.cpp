// SPDX-License-Identifier: MPL-2.0
// docs/ServerTickEstimatorTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"
#include "OGSimulation/PCTimeManagement/ServerTickEstimator.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

namespace serverTickEstimatorTest
{
    constexpr double kPingIntervalSeconds     = 1.0 / 20.0;
    constexpr double kTurnaroundSeconds       = 0.001;
    constexpr double kServerClockAheadSeconds = 123.456;
    constexpr double kFirstPingClientSeconds  = 1.0;

    class SplitMix64
    {
    public:
        explicit SplitMix64(uint64_t state)
            : m_state(state)
        {
        }

        uint64_t next()
        {
            m_state += 0x9E3779B97F4A7C15ull;
            uint64_t z = m_state;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
            return z ^ (z >> 31);
        }

        double unit() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }

        double uniform(double lo, double hi) { return lo + (hi - lo) * unit(); }

    private:
        uint64_t m_state;
    };

    struct Trace
    {
        uint64_t seed                = 1u;
        double   durationSeconds     = 60.0;
        double   upMinSeconds        = 0.030;
        double   upMaxSeconds        = 0.030;
        double   downMinSeconds      = 0.030;
        double   downMaxSeconds      = 0.030;
        double   clientRate          = 1.0;
        double   lockedPhase         = -1.0;
        double   stepLossAtSeconds   = -1.0;
        double   stepLossTicks       = 0.0;
        double   lossProbability     = 0.0;
        double   burstProbability    = 0.0;
        double   burstExtraSeconds   = 0.200;
        double   evaluateFromSeconds = 10.0;
    };

    struct Sample
    {
        uint32_t serverTick     = 0u;
        double   ageSeconds     = 0.0;
        double   rttSeconds     = 0.0;
        double   t3Seconds      = 0.0;
        bool     burst          = false;
        bool     afterStepLoss  = false;
    };

    class World
    {
    public:
        explicit World(const Trace& trace)
            : m_trace(trace)
        {
            const double f = TimeConfig{}.tickFrequency;
            SplitMix64 rng(trace.seed);
            m_originTicks = 1000.0 + 1000.0 * rng.unit();
            if (trace.lockedPhase >= 0.0)
            {
                const double firstAnswer = serverSecondsAt(kFirstPingClientSeconds) + trace.upMinSeconds + kTurnaroundSeconds;
                const double raw         = firstAnswer * f;
                m_originTicks            = 1000.0 + trace.lockedPhase - (raw - std::floor(raw));
            }

            const uint32_t pings = static_cast<uint32_t>(trace.durationSeconds / kPingIntervalSeconds);
            for (uint32_t k = 0; k < pings; ++k)
            {
                const double clientSend = kFirstPingClientSeconds + k * kPingIntervalSeconds;
                const double up         = rng.uniform(trace.upMinSeconds, trace.upMaxSeconds);
                double       down       = rng.uniform(trace.downMinSeconds, trace.downMaxSeconds);
                const bool   burst      = rng.unit() < trace.burstProbability;
                const bool   lost       = rng.unit() < trace.lossProbability;
                if (burst)
                    down += trace.burstExtraSeconds;
                if (lost)
                    continue;

                const double serverReceive = serverSecondsAt(clientSend) + up;
                const double serverAnswer  = serverReceive + kTurnaroundSeconds;
                const double continuous    = continuousTickAtServer(serverAnswer);
                const double published     = std::floor(continuous);
                const double clientReceive = clientSecondsAt(serverAnswer + down);

                Sample sample;
                sample.serverTick    = static_cast<uint32_t>(published);
                sample.ageSeconds    = (continuous - published) / f;
                sample.rttSeconds    = (clientReceive - clientSend) - (serverAnswer - serverReceive);
                sample.t3Seconds     = clientReceive;
                sample.burst         = burst;
                sample.afterStepLoss = trace.stepLossAtSeconds >= 0.0 && serverAnswer >= trace.stepLossAtSeconds;
                m_samples.push_back(sample);
            }
            std::stable_sort(m_samples.begin(), m_samples.end(),
                             [](const Sample& a, const Sample& b) { return a.t3Seconds < b.t3Seconds; });
        }

        const std::vector<Sample>& samples() const { return m_samples; }

        double truthAt(double clientSeconds) const { return continuousTickAtServer(serverSecondsAt(clientSeconds)); }

    private:
        double serverSecondsAt(double clientSeconds) const
        {
            return clientSeconds / m_trace.clientRate + kServerClockAheadSeconds;
        }

        double clientSecondsAt(double serverSeconds) const
        {
            return (serverSeconds - kServerClockAheadSeconds) * m_trace.clientRate;
        }

        double continuousTickAtServer(double serverSeconds) const
        {
            const bool lost = m_trace.stepLossAtSeconds >= 0.0 && serverSeconds >= m_trace.stepLossAtSeconds;
            return serverSeconds * TimeConfig{}.tickFrequency + m_originTicks - (lost ? m_trace.stepLossTicks : 0.0);
        }

        Trace               m_trace;
        double              m_originTicks = 0.0;
        std::vector<Sample> m_samples;
    };

    class Feeder
    {
    public:
        explicit Feeder(ServerTickEstimator& estimator)
            : m_rtt(m_config, nullptr)
            , m_estimator(estimator)
        {
        }

        void feed(const Sample& sample)
        {
            m_rtt.updateRTT(sample.rttSeconds);
            m_estimator.noteSample(sample.serverTick, sample.ageSeconds, sample.rttSeconds, sample.t3Seconds,
                                   m_rtt.getSmoothedRTT());
        }

    private:
        TimeConfig           m_config;
        NetworkTimeEstimator m_rtt;
        ServerTickEstimator& m_estimator;
    };

    struct Errors
    {
        std::vector<double> ticks;
        std::vector<double> phaseBlindTicks;
        uint32_t            delivered       = 0u;
        uint32_t            burstsDelivered = 0u;

        double mean() const
        {
            double sum = 0.0;
            for (double e : ticks)
                sum += e;
            return ticks.empty() ? std::numeric_limits<double>::quiet_NaN() : sum / ticks.size();
        }

        double phaseBlindMean() const
        {
            double sum = 0.0;
            for (double e : phaseBlindTicks)
                sum += e;
            return phaseBlindTicks.empty() ? std::numeric_limits<double>::quiet_NaN() : sum / phaseBlindTicks.size();
        }

        double absP95() const
        {
            std::vector<double> sorted;
            for (double e : ticks)
                sorted.push_back(std::abs(e));
            std::sort(sorted.begin(), sorted.end());
            if (sorted.empty())
                return std::numeric_limits<double>::infinity();
            const size_t rank = static_cast<size_t>(std::ceil(0.95 * sorted.size()));
            return sorted[rank - 1u];
        }

        double absMax() const
        {
            double worst = 0.0;
            for (double e : ticks)
                worst = std::max(worst, std::abs(e));
            return ticks.empty() ? std::numeric_limits<double>::infinity() : worst;
        }

        double absMaxDeviationFrom(double centre) const
        {
            double worst = 0.0;
            for (double e : ticks)
                worst = std::max(worst, std::abs(e - centre));
            return ticks.empty() ? std::numeric_limits<double>::infinity() : worst;
        }
    };

    Errors run(const Trace& trace, ServerTickEstimator& estimator)
    {
        const World world(trace);
        Feeder      feeder(estimator);
        Errors      errors;
        for (const Sample& sample : world.samples())
        {
            feeder.feed(sample);
            ++errors.delivered;
            if (sample.burst)
                ++errors.burstsDelivered;
            const double now = sample.t3Seconds;
            if (now < trace.evaluateFromSeconds + kFirstPingClientSeconds || !estimator.isValid(now))
                continue;
            const double truth = world.truthAt(now);
            errors.ticks.push_back(estimator.serverTickAt(now) - truth);
            errors.phaseBlindTicks.push_back(estimator.phaseBlindServerTickAt(now) - truth);
        }
        return errors;
    }

    Trace b0Like(uint64_t seed, double durationSeconds)
    {
        Trace trace;
        trace.seed            = seed;
        trace.durationSeconds = durationSeconds;
        trace.upMinSeconds    = 0.015;
        trace.upMaxSeconds    = 0.050;
        trace.downMinSeconds  = 0.015;
        trace.downMaxSeconds  = 0.050;
        return trace;
    }

    void noteExact(ServerTickEstimator& estimator, uint32_t k, double offsetTicks = 0.0, double t3Shift = 0.0)
    {
        const double t3 = 10.0 + k / 16.0 + t3Shift;
        const double continuous = 1000.0 + t3 * 60.0 + offsetTicks;
        const double published  = std::floor(continuous);
        estimator.noteSample(static_cast<uint32_t>(published), (continuous - published) / 60.0, 0.0, t3, 0.040);
    }

    ServerTickEstimatorConfig at60Hz()
    {
        ServerTickEstimatorConfig config;
        config.tickFrequency = 60.0;
        return config;
    }
} // namespace serverTickEstimatorTest

TEST_CASE("PCTM.ServerTickEstimator.SymmetricJitterWithTheTrueAgeIsUnbiased", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    ServerTickEstimator estimator;
    const Errors errors = run(b0Like(11u, 600.0), estimator);

    INFO("p95 " << errors.absP95() << " mean " << errors.mean() << " points " << errors.ticks.size()
                << " rejected " << estimator.rejectedSamples() << " reseeds " << estimator.reseeds());
    REQUIRE(errors.ticks.size() > 10000u);
    CHECK(std::abs(errors.mean()) <= 0.01);
    CHECK(errors.absP95() <= 0.07);
    CHECK(estimator.reseeds() == 0u);
}

TEST_CASE("PCTM.ServerTickEstimator.PhaseLockedPingsNeedTheAge", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    Trace trace;
    trace.seed        = 12u;
    trace.lockedPhase = 0.9;
    ServerTickEstimator estimator;
    const Errors errors = run(trace, estimator);

    INFO("max " << errors.absMax() << " phase-blind mean " << errors.phaseBlindMean());
    REQUIRE(errors.ticks.size() > 900u);
    CHECK(errors.absMax() <= 0.05);
    CHECK(errors.phaseBlindMean() <= -0.3);
    for (double blind : errors.phaseBlindTicks)
        REQUIRE(std::abs(blind) >= 0.3);
}

TEST_CASE("PCTM.ServerTickEstimator.AnAsymmetricLinkReadsHalfTheAsymmetry", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    Trace trace;
    trace.seed           = 13u;
    trace.upMinSeconds   = 0.050;
    trace.upMaxSeconds   = 0.050;
    trace.downMinSeconds = 0.020;
    trace.downMaxSeconds = 0.020;
    ServerTickEstimator estimator;
    const Errors errors = run(trace, estimator);

    const double expected = (0.050 - 0.020) / 2.0 * TimeConfig{}.tickFrequency;
    INFO("mean " << errors.mean() << " expected " << expected);
    REQUIRE(errors.ticks.size() > 900u);
    CHECK(std::abs(errors.mean() - 0.90) <= 0.05);
    CHECK(errors.absMaxDeviationFrom(expected) <= 0.05);
}

TEST_CASE("PCTM.ServerTickEstimator.AHundredPpmClockRateMismatchStaysWithinTheLag", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    for (const double rate : { 1.0 + 100e-6, 1.0 - 100e-6 })
    {
        Trace trace;
        trace.seed            = 14u;
        trace.durationSeconds = 600.0;
        trace.upMinSeconds    = 0.025;
        trace.upMaxSeconds    = 0.025;
        trace.downMinSeconds  = 0.025;
        trace.downMaxSeconds  = 0.025;
        trace.clientRate      = rate;
        ServerTickEstimator estimator;
        const Errors errors = run(trace, estimator);

        INFO("rate " << rate << " max " << errors.absMax());
        REQUIRE(errors.ticks.size() > 11000u);
        CHECK(errors.absMax() <= 0.06);
    }
}

TEST_CASE("PCTM.ServerTickEstimator.AServerStepLossReSeedsWithinFiveSamples", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    Trace trace;
    trace.seed              = 15u;
    trace.durationSeconds   = 40.0;
    trace.stepLossAtSeconds = 30.0 + kServerClockAheadSeconds;
    trace.stepLossTicks     = 12.0;
    const World world(trace);

    ServerTickEstimator estimator;
    Feeder              feeder(estimator);
    uint32_t            afterLoss      = 0u;
    double              reseedAt       = -1.0;
    bool                checkedLater   = false;
    for (const Sample& sample : world.samples())
    {
        feeder.feed(sample);
        if (!sample.afterStepLoss)
            continue;
        ++afterLoss;
        const double error = estimator.serverTickAt(sample.t3Seconds) - world.truthAt(sample.t3Seconds);
        INFO("sample " << afterLoss << " after the loss, error " << error << " reseeds " << estimator.reseeds());
        if (afterLoss < 5u)
        {
            REQUIRE(estimator.reseeds() == 0u);
            REQUIRE(std::abs(error - 12.0) <= 0.05);
        }
        else if (afterLoss == 5u)
        {
            REQUIRE(estimator.reseeds() == 1u);
            reseedAt = sample.t3Seconds;
        }
        else if (!checkedLater && sample.t3Seconds >= reseedAt + 0.5)
        {
            checkedLater = true;
            CHECK(std::abs(error) <= 0.1);
        }
    }
    CHECK(checkedLater);
    CHECK(estimator.reseeds() == 1u);
}

TEST_CASE("PCTM.ServerTickEstimator.LossAndDelayBurstsAreGatedAndCounted", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    Trace trace             = b0Like(16u, 600.0);
    trace.lossProbability   = 0.05;
    trace.burstProbability  = 0.10;
    ServerTickEstimator estimator;
    const Errors errors = run(trace, estimator);

    INFO("p95 " << errors.absP95() << " delivered " << errors.delivered << " bursts " << errors.burstsDelivered
                << " rejected " << estimator.rejectedSamples() << " reseeds " << estimator.reseeds());
    REQUIRE(errors.burstsDelivered > 900u);
    CHECK(errors.absP95() <= 0.08);
    CHECK(estimator.rejectedSamples() >= errors.burstsDelivered * 9u / 10u);
    CHECK(estimator.rejectedSamples() <= errors.burstsDelivered + errors.delivered / 100u);
    CHECK(estimator.acceptedSamples() + estimator.rejectedSamples() == errors.delivered);
}

TEST_CASE("PCTM.ServerTickEstimator.ValidAfterTwentySamplesAndStaleOneSecondAfterTheLast", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    ServerTickEstimator estimator(at60Hz());
    CHECK_FALSE(estimator.isValid(10.0));
    for (uint32_t k = 0; k < 19u; ++k)
        noteExact(estimator, k);
    CHECK(estimator.acceptedSamples() == 19u);
    CHECK_FALSE(estimator.isValid(estimator.lastSampleSeconds()));

    noteExact(estimator, 19u);
    const double last = estimator.lastSampleSeconds();
    CHECK(last == 10.0 + 19.0 / 16.0);
    CHECK(estimator.isValid(last));
    CHECK(estimator.isValid(last + 1.0));
    CHECK_FALSE(estimator.isValid(last + 1.0 + 1.0 / 1024.0));

    estimator.noteSample(1000u, 0.0, 1.0, last + 2.0, 0.040);
    CHECK(estimator.rejectedSamples() == 1u);
    CHECK(estimator.lastSampleSeconds() == last);
    CHECK_FALSE(estimator.isValid(last + 2.0));
}

TEST_CASE("PCTM.ServerTickEstimator.IgnoredSamplesMoveNothing", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    ServerTickEstimator estimator(at60Hz());
    estimator.noteSample(ServerTickEstimator::kNoServerTick, 0.001, 0.040, 10.0, 0.040);
    estimator.noteSample(1000u, nan, 0.040, 10.0, 0.040);
    estimator.noteSample(1000u, 0.001, inf, 10.0, 0.040);
    estimator.noteSample(1000u, 0.001, 0.040, nan, 0.040);
    estimator.noteSample(1000u, 0.001, 0.040, 10.0, nan);
    estimator.noteSample(1000u, -1e-9, 0.040, 10.0, 0.040);
    estimator.noteSample(1000u, 4294967295.0 / 1e6, 0.040, 10.0, 0.040);
    estimator.noteSample(1000u, ServerTickEstimator::kMaxServerTickAgeSeconds, 0.040, 10.0, 0.040);
    CHECK(estimator.acceptedSamples() == 0u);
    CHECK(estimator.rejectedSamples() == 0u);
    CHECK(estimator.lastSampleSeconds() == 0.0);
    CHECK(estimator.serverTickAt(10.0) == 600.0);

    estimator.noteSample(1000u, 0.0, 0.500, 10.0, 0.0);
    CHECK(estimator.acceptedSamples() == 1u);
    CHECK(estimator.rejectedSamples() == 0u);
}

TEST_CASE("PCTM.ServerTickEstimator.OnlyAConsecutiveRunOfJumpsReSeeds", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    ServerTickEstimator estimator(at60Hz());
    uint32_t k = 0u;
    for (; k < 20u; ++k)
        noteExact(estimator, k);
    const double settled = estimator.serverTickAt(20.0);

    for (uint32_t i = 0; i < 4u; ++i, ++k)
        noteExact(estimator, k, 8.0);
    noteExact(estimator, k++);
    for (uint32_t i = 0; i < 4u; ++i, ++k)
        noteExact(estimator, k, 8.0);
    CHECK(estimator.reseeds() == 0u);
    CHECK(std::abs(estimator.serverTickAt(20.0) - settled) <= 1e-6);
    CHECK(estimator.isValid(estimator.lastSampleSeconds()));

    noteExact(estimator, k++, 8.0);
    CHECK(estimator.reseeds() == 1u);
    CHECK(std::abs(estimator.serverTickAt(20.0) - settled - 8.0) <= 1e-6);
    CHECK_FALSE(estimator.isValid(estimator.lastSampleSeconds()));
}

TEST_CASE("PCTM.ServerTickEstimator.AnOutOfOrderSampleHasNoWeight", "[PCTM][Dilation][ServerTickEstimator]")
{
    using namespace serverTickEstimatorTest;
    ServerTickEstimator estimator(at60Hz());
    for (uint32_t k = 0; k < 20u; ++k)
        noteExact(estimator, k);
    const double settled = estimator.serverTickAt(20.0);

    noteExact(estimator, 20u, 1.0, -0.25);
    CHECK(estimator.serverTickAt(20.0) == settled);
    CHECK(estimator.acceptedSamples() == 21u);
}

#endif // WITH_LOW_LEVEL_TESTS
