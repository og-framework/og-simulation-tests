// SPDX-License-Identifier: MPL-2.0
// docs/ClockDilationControllerTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/ClockDilationController.h"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"

namespace clockDilationControllerTest
{
    constexpr double   kCallSeconds      = 1.0 / 64.0;
    constexpr double   kCallStepLagTicks = kCallSeconds * 60.0 / 2.0;
    constexpr double   kServerTick       = 1000.0;
    constexpr uint32_t kAuthorityTick    = 998u;
    constexpr uint32_t kPredOffsetTicks  = 4u;
    constexpr double   kCommonLeadTicks  = 2.5;
    constexpr double   kGuardCentreTicks = 2.0;
    constexpr double   kBelowEveryEdge   = -10.0;
    constexpr double   kThirtyFpsSeconds = 1.0 / 30.0;
    constexpr double   kHundredFpsSeconds = 1.0 / 100.0;
    constexpr double   kHitchSeconds     = 0.6;
    constexpr double   kMidBandServerTickFraction = 0.5;
    constexpr double   kStepLagSeconds   = 30.0;
    constexpr double   kStepLagTolerance = 0.002;
    constexpr double   kLongFrameSeconds = 0.040;
    constexpr double   kOverCapFrameSeconds = 0.060;
    constexpr double   kServerTicksPerThirtyFpsCall = 2.0;
    constexpr uint32_t kCentreCalls      = 1800u;
    constexpr double   kAboveEveryEdge   = 10.0;

    ClockDilationConfig commonConfig(ClockDilationMode mode)
    {
        ClockDilationConfig config;
        config.mode            = mode;
        config.setpoint        = ClockDilationSetpoint::Common;
        config.commonLeadTicks = kCommonLeadTicks;
        return config;
    }

    ClockDilationConfig lowEdgeConfig()
    {
        ClockDilationConfig config = commonConfig(ClockDilationMode::On);
        config.commonLeadTicks     = kBelowEveryEdge;
        return config;
    }

    ClockDilationInputs inputsWithError(double clockNowSeconds, double errorTicks)
    {
        ClockDilationInputs in;
        in.clockNowSeconds       = clockNowSeconds;
        in.sensorValid           = true;
        in.serverTick            = kServerTick;
        in.predictionTick        = kServerTick + kCommonLeadTicks + errorTicks;
        in.smoothedRttSeconds    = 0.030;
        in.smoothedJitterSeconds = 0.002;
        in.jitterMultiplier      = 0.5;
        in.predOffsetFloorTicks  = 4u;
        in.tickFrequency         = 60.0;
        in.authorityTick         = kAuthorityTick;
        in.predictionOffsetTicks = kPredOffsetTicks;
        return in;
    }

    struct Door
    {
        double   rate    = 1.0;
        uint32_t applied = 0u;
    };

    struct Step
    {
        ClockDilationOutput out;
        double              doorRate = 1.0;
    };

    class Rig
    {
    public:
        explicit Rig(const ClockDilationConfig& config)
            : m_controller(config)
        {
        }

        Step call(ClockDilationInputs in)
        {
            in.currentScale = m_door.rate;
            if (in.clockRead.advances == 0u)
            {
                in.clockRead.advances   = ++m_reads;
                in.clockRead.targetTick = in.authorityTick + in.predictionOffsetTicks;
            }
            Step step;
            step.out = m_controller.update(in);
            if (step.out.apply)
            {
                m_door.rate = step.out.scale;
                ++m_door.applied;
            }
            step.doorRate = m_door.rate;
            m_nowSeconds  = in.clockNowSeconds;
            ++m_calls;
            return step;
        }

        Step callWithError(double errorTicks)
        {
            const double lagTicks = m_calls > 0u ? kCallStepLagTicks : 0.0;
            return call(inputsWithError(m_nowSeconds + kCallSeconds, errorTicks + lagTicks));
        }

        Step callWith(ClockDilationInputs in)
        {
            in.clockNowSeconds = m_nowSeconds + kCallSeconds;
            return call(in);
        }

        Step callAfter(double seconds)
        {
            return call(inputsWithError(m_nowSeconds + seconds, 0.0));
        }

        Step callAfterWith(double seconds, ClockDilationInputs in)
        {
            in.clockNowSeconds = m_nowSeconds + seconds;
            return call(in);
        }

        Step callEvery(double seconds, int calls)
        {
            Step last;
            for (int i = 0; i < calls; ++i)
                last = callAfter(seconds);
            return last;
        }

        void settle(double errorTicks, int calls)
        {
            for (int i = 0; i < calls; ++i)
                callWithError(errorTicks);
        }

        double nowSeconds() const { return m_nowSeconds; }
        const Door& door() const { return m_door; }
        const ClockDilationController& controller() const { return m_controller; }

    private:
        ClockDilationController m_controller;
        Door                    m_door;
        double                  m_nowSeconds = 10.0;
        uint32_t                m_calls      = 0u;
        uint32_t                m_reads      = 0u;
    };

    double stepLagOf(const ClockDilationOutput& out) { return out.stepLagTicks; }

    ClockDilationInputs midBandOwnRttInputs()
    {
        ClockDilationInputs in = inputsWithError(0.0, 0.0);
        in.serverTick          = kServerTick + kMidBandServerTickFraction;
        in.predictionTick      = in.serverTick + kCommonLeadTicks;
        return in;
    }

    double ownRttRawTargetOf(const ClockDilationInputs& in)
    {
        const double f = in.tickFrequency;
        return std::max(static_cast<double>(in.predOffsetFloorTicks),
                        (in.smoothedRttSeconds + in.jitterMultiplier * in.smoothedJitterSeconds) * f + 0.5)
               - (in.smoothedRttSeconds / 2.0) * f - 0.5;
    }

    Step runFrames(Rig& rig, const ClockDilationInputs& in, double frameTicks, double seconds)
    {
        const double frameSeconds = frameTicks / in.tickFrequency;
        Step last;
        for (double elapsed = 0.0; elapsed < seconds; elapsed += frameSeconds)
            last = rig.callAfterWith(frameSeconds, in);
        return last;
    }

    ClockDilationInputs centreInputs(uint32_t call, bool drainAtStepStart)
    {
        ClockDilationInputs in  = inputsWithError(10.0 + kThirtyFpsSeconds * call, 0.0);
        const double     moved  = kServerTicksPerThirtyFpsCall * call;
        const uint32_t   fresh  = kAuthorityTick + static_cast<uint32_t>(moved);
        const uint32_t   stale  = fresh - static_cast<uint32_t>(kServerTicksPerThirtyFpsCall);
        in.serverTick           = kServerTick + moved;
        in.predictionTick       = in.serverTick + kCommonLeadTicks;
        in.authorityTick        = drainAtStepStart ? stale : fresh;
        in.clockRead.advances   = call;
        in.clockRead.targetTick = stale + kPredOffsetTicks;
        return in;
    }

    struct CentreRun
    {
        ClockDilationOutput last;
        ClockDilationOutput held;
        double              readMinusPreviousSensor = 0.0;
    };

    CentreRun runCentre(bool drainAtStepStart, double commonLeadTicks)
    {
        ClockDilationConfig config = commonConfig(ClockDilationMode::On);
        config.commonLeadTicks     = commonLeadTicks;
        ClockDilationController controller(config);
        CentreRun run;
        for (uint32_t call = 0u; call < kCentreCalls; ++call)
            run.last = controller.update(centreInputs(call, drainAtStepStart));
        const ClockDilationInputs lastIn     = centreInputs(kCentreCalls - 1u, drainAtStepStart);
        const ClockDilationInputs previousIn = centreInputs(kCentreCalls - 2u, drainAtStepStart);
        run.readMinusPreviousSensor = static_cast<double>(lastIn.clockRead.targetTick) - previousIn.serverTick;

        ClockDilationInputs noNewRead = centreInputs(kCentreCalls, drainAtStepStart);
        noNewRead.clockRead           = lastIn.clockRead;
        noNewRead.serverTick         += 7.25;
        noNewRead.authorityTick      -= 5u;
        run.held = controller.update(noNewRead);
        return run;
    }

    std::vector<ClockDilationInputs> variedInputs()
    {
        std::vector<ClockDilationInputs> all;
        for (double error : {-3.0, -1.0, 0.0, 1.0, 3.0, 25.0})
        {
            ClockDilationInputs in = inputsWithError(10.0 + error, error);
            all.push_back(in);
            in.sensorValid = false;
            all.push_back(in);
            in.sensorValid           = true;
            in.phase.recentring      = true;
            in.phase.transitionDebt  = true;
            in.phase.hardResyncs     = 3u;
            in.currentScale          = 0.99;
            all.push_back(in);
        }
        return all;
    }
} // namespace clockDilationControllerTest

using namespace clockDilationControllerTest;

TEST_CASE("PCTM.ClockDilationController.OffIsScaleOneAndNeverApplies", "[PCTM][Dilation][ClockDilationController]")
{
    ClockDilationController controller(commonConfig(ClockDilationMode::Off));
    for (const ClockDilationInputs& in : variedInputs())
    {
        const ClockDilationOutput out = controller.update(in);
        CHECK(out.scale == 1.0);
        CHECK_FALSE(out.apply);
        CHECK(out.freeze == DilationFreeze::ModeOff);
        CHECK_FALSE(out.guardBound);
    }
    CHECK(controller.scale() == 1.0);

    ClockDilationConfig ownRtt = commonConfig(ClockDilationMode::Off);
    ownRtt.setpoint            = ClockDilationSetpoint::OwnRtt;
    ClockDilationController ownRttController(ownRtt);
    for (const ClockDilationInputs& in : variedInputs())
    {
        const ClockDilationOutput out = ownRttController.update(in);
        CHECK(out.scale == 1.0);
        CHECK_FALSE(out.apply);
        CHECK(out.freeze == DilationFreeze::ModeOff);
    }
}

TEST_CASE("PCTM.ClockDilationController.OneTickAheadSlewsToPointNineNineFive", "[PCTM][Dilation][ClockDilationController]")
{
    Rig rig(commonConfig(ClockDilationMode::On));
    const double expected[] = {0.998, 0.996, 0.995, 0.995, 0.995, 0.995, 0.995, 0.995};
    double       previous   = 1.0;
    for (double scale : expected)
    {
        const Step step = rig.callWithError(1.0);
        CHECK(step.out.errorTicks == Catch::Approx(1.0).margin(1e-12));
        CHECK(step.out.freeze == DilationFreeze::None);
        CHECK(step.out.scale == Catch::Approx(scale).margin(1e-12));
        CHECK(step.out.apply == (step.out.scale != previous));
        CHECK(step.doorRate == step.out.scale);
        previous = step.out.scale;
    }
    CHECK(rig.door().applied == 3u);

    rig.settle(1.0, 600);
    CHECK(rig.controller().scale() == Catch::Approx(0.995).margin(1e-12));
    CHECK(rig.door().applied == 3u);
}

TEST_CASE("PCTM.ClockDilationController.ThreeTicksSaturateAtTheDeviation", "[PCTM][Dilation][ClockDilationController]")
{
    Rig ahead(commonConfig(ClockDilationMode::On));
    ahead.settle(3.0, 20);
    const Step aheadStep = ahead.callWithError(3.0);
    CHECK(aheadStep.out.scale == Catch::Approx(0.990).margin(1e-12));
    CHECK_FALSE(aheadStep.out.apply);
    CHECK(aheadStep.doorRate == Catch::Approx(0.990).margin(1e-12));

    Rig behind(commonConfig(ClockDilationMode::On));
    behind.settle(-3.0, 20);
    const Step behindStep = behind.callWithError(-3.0);
    CHECK(behindStep.out.scale == Catch::Approx(1.010).margin(1e-12));
    CHECK_FALSE(behindStep.out.apply);
    CHECK(behindStep.doorRate == Catch::Approx(1.010).margin(1e-12));
}

TEST_CASE("PCTM.ClockDilationController.EachFreezeCauseSlewsBackToOneAndHolds", "[PCTM][Dilation][ClockDilationController]")
{
    struct Cause
    {
        DilationFreeze freeze;
        void (*apply)(ClockDilationInputs&);
    };
    const Cause causes[] = {
        {DilationFreeze::SensorInvalid, [](ClockDilationInputs& in) { in.sensorValid = false; }},
        {DilationFreeze::Recentre, [](ClockDilationInputs& in) { in.phase.recentreArmed = true; }},
        {DilationFreeze::Recentre, [](ClockDilationInputs& in) { in.phase.recentring = true; }},
        {DilationFreeze::TransitionDebt, [](ClockDilationInputs& in) { in.phase.transitionDebt = true; }},
        {DilationFreeze::AwaitFirstRtt, [](ClockDilationInputs& in) { in.phase.awaitsFirstRtt = true; }},
    };
    uint32_t causesRun = 0u;
    for (const Cause& cause : causes)
    {
        Rig rig(commonConfig(ClockDilationMode::On));
        rig.settle(3.0, 20);
        REQUIRE(rig.controller().scale() == Catch::Approx(0.990).margin(1e-12));

        const double expected[] = {0.992, 0.994, 0.996, 0.998, 1.0, 1.0, 1.0, 1.0};
        for (double scale : expected)
        {
            ClockDilationInputs in = inputsWithError(0.0, 3.0);
            cause.apply(in);
            const Step step = rig.callWith(in);
            CHECK(step.out.freeze == cause.freeze);
            CHECK(step.out.scale == Catch::Approx(scale).margin(1e-12));
            CHECK(step.doorRate == step.out.scale);
        }
        const Step released = rig.callWithError(3.0);
        CHECK(released.out.freeze == DilationFreeze::None);
        CHECK(released.out.scale == Catch::Approx(0.998).margin(1e-12));
        ++causesRun;
    }
    CHECK(causesRun == 5u);
}

TEST_CASE("PCTM.ClockDilationController.AfterResyncFreezesForExactlyTheWindow", "[PCTM][Dilation][ClockDilationController]")
{
    Rig rig(commonConfig(ClockDilationMode::On));
    rig.settle(3.0, 20);
    REQUIRE(rig.controller().scale() == Catch::Approx(0.990).margin(1e-12));

    ClockDilationInputs resynced = inputsWithError(0.0, 3.0);
    resynced.phase.hardResyncs   = 1u;
    const Step first             = rig.callWith(resynced);
    const double resyncSeconds   = rig.nowSeconds();
    CHECK(first.out.freeze == DilationFreeze::AfterResync);
    CHECK(first.out.scale == Catch::Approx(0.992).margin(1e-12));

    int frozenCalls = 1;
    for (;;)
    {
        const Step step = rig.callWith(resynced);
        if (step.out.freeze != DilationFreeze::AfterResync)
        {
            CHECK(step.out.freeze == DilationFreeze::None);
            CHECK(step.out.scale == Catch::Approx(0.998).margin(1e-12));
            break;
        }
        CHECK(rig.nowSeconds() - resyncSeconds < 1.0);
        ++frozenCalls;
        REQUIRE(frozenCalls < 1000);
    }
    CHECK(rig.nowSeconds() - resyncSeconds == 1.0);
    CHECK(frozenCalls == 64);
    CHECK(rig.door().rate == Catch::Approx(0.998).margin(1e-12));

    for (int i = 0; i < 200; ++i)
        REQUIRE(rig.callWith(resynced).out.freeze == DilationFreeze::None);
    resynced.phase.hardResyncs = 2u;
    CHECK(rig.callWith(resynced).out.freeze == DilationFreeze::AfterResync);
}

TEST_CASE("PCTM.ClockDilationController.TheFirstMatchingFreezeCauseIsReported", "[PCTM][Dilation][ClockDilationController]")
{
    Rig rig(commonConfig(ClockDilationMode::On));
    ClockDilationInputs in   = inputsWithError(0.0, 3.0);
    in.sensorValid           = false;
    in.phase.recentreArmed   = true;
    in.phase.transitionDebt  = true;
    in.phase.awaitsFirstRtt  = true;
    in.phase.hardResyncs     = 1u;
    CHECK(rig.callWith(in).out.freeze == DilationFreeze::SensorInvalid);
    in.sensorValid = true;
    CHECK(rig.callWith(in).out.freeze == DilationFreeze::Recentre);
    in.phase.recentreArmed = false;
    CHECK(rig.callWith(in).out.freeze == DilationFreeze::TransitionDebt);
    in.phase.transitionDebt = false;
    CHECK(rig.callWith(in).out.freeze == DilationFreeze::AwaitFirstRtt);
    in.phase.awaitsFirstRtt = false;
    CHECK(rig.callWith(in).out.freeze == DilationFreeze::AfterResync);
}

TEST_CASE("PCTM.ClockDilationController.ARealClocksSkipDebtFreezesItAsAStallDebtDoes", "[PCTM][Dilation][ClockDilationController]")
{
    for (const int32_t debt : {-2, +2})
    {
        CAPTURE(debt);
        TimeConfig clockConfig;
        clockConfig.recentreSettleSamples    = 0u;
        clockConfig.minTicksBeforeDriftCheck = 0u;
        clockConfig.softDriftThresholdTicks  = 1000u;
        clockConfig.hardResyncThresholdTicks = 5000u;
        NetworkTimeEstimator  estimator(clockConfig, nullptr);
        ClientPredictionClock clock(clockConfig, estimator, nullptr);
        Rig rig(commonConfig(ClockDilationMode::On));

        ClockDilationInputs in = inputsWithError(0.0, 3.0);
        in.phase               = clock.publishedCorrectionPhase();
        REQUIRE(rig.callWith(in).out.freeze == DilationFreeze::None);

        clock.requestInputDelayTransition(debt);
        in.phase = clock.publishedCorrectionPhase();
        CHECK(rig.callWith(in).out.freeze == DilationFreeze::TransitionDebt);

        for (int32_t paid = 0; paid < (debt < 0 ? -debt : debt); ++paid)
        {
            estimator.recordAuthorityTick(0u);
            clock.advancePrediction();
        }
        REQUIRE(clock.getPendingInputDelayTransitionTicks() == 0);
        in.phase = clock.publishedCorrectionPhase();
        CHECK(rig.callWith(in).out.freeze == DilationFreeze::None);
    }
}

TEST_CASE("PCTM.ClockDilationController.OwnRttReproducesTheMeasuredMeanLead", "[PCTM][Dilation][ClockDilationController]")
{
    ClockDilationConfig config = commonConfig(ClockDilationMode::Shadow);
    config.setpoint            = ClockDilationSetpoint::OwnRtt;

    ClockDilationInputs emulated   = inputsWithError(10.0, 0.0);
    emulated.smoothedRttSeconds    = 0.0725;
    emulated.smoothedJitterSeconds = 0.0095;
    emulated.jitterMultiplier      = 0.5;
    emulated.predOffsetFloorTicks  = 4u;
    ClockDilationController emulatedController(config);
    const ClockDilationOutput emulatedOut = emulatedController.update(emulated);
    CHECK(emulatedOut.targetTicks == Catch::Approx(2.461).margin(0.001));
    CHECK(emulatedOut.targetTicks == Catch::Approx(2.460).margin(1e-9));
    CHECK_FALSE(emulatedOut.guardBound);

    ClockDilationInputs loopback   = emulated;
    loopback.smoothedRttSeconds    = 0.0001;
    loopback.smoothedJitterSeconds = 0.0;
    ClockDilationController loopbackController(config);
    const ClockDilationOutput loopbackOut = loopbackController.update(loopback);
    CHECK(loopbackOut.targetTicks == Catch::Approx(3.497).margin(0.001));
    CHECK_FALSE(loopbackOut.guardBound);

    ClockDilationInputs later = loopback;
    later.clockNowSeconds     = 20.0;
    CHECK(loopbackController.update(later).targetTicks == Catch::Approx(3.497).margin(0.001));
}

TEST_CASE("PCTM.ClockDilationController.TheTargetLowPassesAtItsTau", "[PCTM][Dilation][ClockDilationController]")
{
    ClockDilationController controller(commonConfig(ClockDilationMode::Shadow));
    ClockDilationInputs in = inputsWithError(10.0, 0.0);
    in.sensorValid         = false;
    CHECK(controller.update(in).targetTicks == Catch::Approx(kCommonLeadTicks).margin(1e-12));

    ClockDilationConfig ownRtt = commonConfig(ClockDilationMode::Shadow);
    ownRtt.setpoint            = ClockDilationSetpoint::OwnRtt;
    ClockDilationController filtered(ownRtt);
    ClockDilationInputs seed   = inputsWithError(10.0, 0.0);
    seed.smoothedRttSeconds    = 0.0001;
    seed.smoothedJitterSeconds = 0.0;
    const double seedTarget    = filtered.update(seed).targetTicks;

    ClockDilationInputs stepped = seed;
    stepped.clockNowSeconds     = 20.0;
    stepped.smoothedRttSeconds  = 0.1;
    const double raw            = std::max(4.0, 0.1 * 60.0 + 0.5) - 0.05 * 60.0 - 0.5;
    const double expected       = seedTarget + (1.0 - std::exp(-10.0 / 10.0)) * (raw - seedTarget);
    CHECK(filtered.update(stepped).targetTicks == Catch::Approx(expected).margin(1e-9));
}

TEST_CASE("PCTM.ClockDilationController.TheCommonTargetCarriesNoJitterTerm", "[PCTM][Dilation][ClockDilationController]")
{
    ClockDilationConfig ownRtt = commonConfig(ClockDilationMode::Shadow);
    ownRtt.setpoint            = ClockDilationSetpoint::OwnRtt;
    uint32_t setpointsRun = 0u;
    for (const ClockDilationConfig& config : { commonConfig(ClockDilationMode::Shadow), ownRtt })
    {
        ClockDilationInputs quiet   = inputsWithError(10.0, 0.0);
        quiet.smoothedRttSeconds    = 0.080;
        quiet.smoothedJitterSeconds = 0.0;
        quiet.jitterMultiplier      = 0.0;
        ClockDilationInputs jittery   = quiet;
        jittery.smoothedJitterSeconds = 0.010;
        jittery.jitterMultiplier      = 2.0;

        ClockDilationController quietController(config);
        ClockDilationController jitteryController(config);
        const double quietTarget   = quietController.update(quiet).targetTicks;
        const double jitteryTarget = jitteryController.update(jittery).targetTicks;
        CAPTURE(clockDilation::setpointWord(config.setpoint), quietTarget, jitteryTarget);
        REQUIRE_FALSE(quietController.update(quiet).guardBound);
        REQUIRE_FALSE(jitteryController.update(jittery).guardBound);
        if (config.setpoint == ClockDilationSetpoint::Common)
        {
            CHECK(quietTarget == Catch::Approx(kCommonLeadTicks).margin(1e-12));
            CHECK(jitteryTarget == quietTarget);
        }
        else
        {
            CHECK(jitteryTarget - quietTarget == Catch::Approx(2.0 * 0.010 * 60.0).margin(1e-9));
        }
        ++setpointsRun;
    }
    CHECK(setpointsRun == 2u);
}

TEST_CASE("PCTM.ClockDilationController.TheBandGuardClampsACommonTarget", "[PCTM][Dilation][ClockDilationController]")
{
    ClockDilationConfig config = commonConfig(ClockDilationMode::On);
    config.commonLeadTicks     = 6.0;
    ClockDilationController controller(config);

    ClockDilationInputs in = inputsWithError(10.0, 0.0);
    REQUIRE(static_cast<double>(in.predictionOffsetTicks) - (in.serverTick - in.authorityTick) == kGuardCentreTicks);
    const ClockDilationOutput out = controller.update(in);
    CHECK(out.targetTicks == Catch::Approx(4.5).margin(1e-12));
    CHECK(out.guardBound);

    ClockDilationConfig low = commonConfig(ClockDilationMode::On);
    low.commonLeadTicks     = -1.0;
    ClockDilationController lowController(low);
    const ClockDilationOutput lowOut = lowController.update(in);
    CHECK(lowOut.targetTicks == Catch::Approx(1.5).margin(1e-12));
    CHECK(lowOut.guardBound);

    ClockDilationController inside(commonConfig(ClockDilationMode::On));
    const ClockDilationOutput insideOut = inside.update(in);
    CHECK(insideOut.targetTicks == Catch::Approx(kCommonLeadTicks).margin(1e-12));
    CHECK_FALSE(insideOut.guardBound);
}

TEST_CASE("PCTM.ClockDilationController.TheLowEdgeFollowsTheFramePeriod", "[PCTM][Dilation][ClockDilationController]")
{
    const double periods[]  = {kThirtyFpsSeconds, kHundredFpsSeconds, 1.0 / 144.0};
    const double expected[] = {kGuardCentreTicks + 1.0, kGuardCentreTicks - 0.2, kGuardCentreTicks - 0.5 + 30.0 / 144.0};
    for (size_t index = 0u; index < 3u; ++index)
    {
        Rig rig(lowEdgeConfig());
        const Step first = rig.callAfter(periods[index]);
        CHECK(first.out.targetTicks == Catch::Approx(kGuardCentreTicks - 0.5).margin(1e-12));
        CHECK(first.out.guardBound);

        const Step second = rig.callAfter(periods[index]);
        const Step later  = rig.callEvery(periods[index], 600);
        CAPTURE(index, second.out.targetTicks, later.out.targetTicks);
        CHECK(second.out.targetTicks == Catch::Approx(expected[index]).margin(1e-9));
        CHECK(later.out.targetTicks == Catch::Approx(expected[index]).margin(1e-9));
        CHECK(later.out.guardBound);
    }

    ClockDilationConfig shadowLow = lowEdgeConfig();
    shadowLow.mode                = ClockDilationMode::Shadow;
    Rig shadowEdge(shadowLow);
    shadowEdge.callEvery(kThirtyFpsSeconds, 60);
    CHECK(shadowEdge.callAfter(kThirtyFpsSeconds).out.targetTicks == Catch::Approx(kGuardCentreTicks + 1.0).margin(1e-9));
}

TEST_CASE("PCTM.ClockDilationController.TheFramePeakDecaysAtTheGuardTau", "[PCTM][Dilation][ClockDilationController]")
{
    Rig rig(lowEdgeConfig());
    rig.callEvery(kThirtyFpsSeconds, 120);
    REQUIRE(rig.callAfter(kThirtyFpsSeconds).out.targetTicks == Catch::Approx(kGuardCentreTicks + 1.0).margin(1e-9));

    const Step afterOne = rig.callAfter(kHundredFpsSeconds);
    CHECK(afterOne.out.targetTicks < kGuardCentreTicks + 1.0);
    CHECK(afterOne.out.targetTicks > kGuardCentreTicks + 0.99);

    const Step afterTwoSeconds = rig.callEvery(kHundredFpsSeconds, 199);
    const double peakTicks     = 0.6 + 1.4 * std::exp(-1.0);
    CAPTURE(afterTwoSeconds.out.targetTicks, peakTicks);
    CHECK(afterTwoSeconds.out.targetTicks == Catch::Approx(kGuardCentreTicks + 0.115).margin(0.005));
    CHECK(afterTwoSeconds.out.targetTicks == Catch::Approx(kGuardCentreTicks + (peakTicks + 0.5 - 1.0) - 0.5).margin(1e-6));

    const Step settled = rig.callEvery(kHundredFpsSeconds, 3000);
    CHECK(settled.out.targetTicks == Catch::Approx(kGuardCentreTicks - 0.2).margin(1e-6));
}

TEST_CASE("PCTM.ClockDilationController.AHitchFrameIsNotSampled", "[PCTM][Dilation][ClockDilationController]")
{
    Rig rig(lowEdgeConfig());
    const Step before = rig.callEvery(kHundredFpsSeconds, 300);
    REQUIRE(before.out.targetTicks == Catch::Approx(kGuardCentreTicks - 0.2).margin(1e-9));

    const Step hitch = rig.callAfter(kHitchSeconds);
    CHECK(hitch.out.targetTicks == Catch::Approx(before.out.targetTicks).margin(1e-9));
    const Step next = rig.callAfter(kHundredFpsSeconds);
    CHECK(next.out.targetTicks == Catch::Approx(before.out.targetTicks).margin(1e-9));
    const Step later = rig.callEvery(kHundredFpsSeconds, 100);
    CHECK(later.out.targetTicks == Catch::Approx(before.out.targetTicks).margin(1e-9));
}

TEST_CASE("PCTM.ClockDilationController.AShortRunOfLongFramesIsNotSampled", "[PCTM][Dilation][ClockDilationController]")
{
    const double hundredFpsEdge = kGuardCentreTicks - 0.2;
    const double longFrameEdge  = kGuardCentreTicks + (kLongFrameSeconds * 60.0 + 0.5 - 1.0) - 0.5;
    const uint32_t correctionReads = TimeConfig{}.gradualCorrectionRate;
    uint32_t runsChecked = 0u;
    for (uint32_t run = 1u; run < correctionReads; ++run)
    {
        Rig rig(lowEdgeConfig());
        REQUIRE(rig.callEvery(kHundredFpsSeconds, 300).out.targetTicks == Catch::Approx(hundredFpsEdge).margin(1e-9));
        for (uint32_t frame = 0u; frame < run; ++frame)
        {
            const Step longFrame = rig.callAfter(kLongFrameSeconds);
            CAPTURE(run, frame, longFrame.out.targetTicks);
            CHECK(longFrame.out.targetTicks == Catch::Approx(hundredFpsEdge).margin(1e-9));
        }
        const Step after = rig.callEvery(kHundredFpsSeconds, 100);
        CHECK(after.out.targetTicks == Catch::Approx(hundredFpsEdge).margin(1e-9));
        ++runsChecked;
    }
    CHECK(runsChecked == correctionReads - 1u);

    Rig sustained(lowEdgeConfig());
    sustained.callEvery(kHundredFpsSeconds, 300);
    Step longFrame;
    for (uint32_t frame = 1u; frame < correctionReads; ++frame)
        longFrame = sustained.callAfter(kLongFrameSeconds);
    CHECK(longFrame.out.targetTicks == Catch::Approx(hundredFpsEdge).margin(1e-9));
    const Step entered = sustained.callAfter(kLongFrameSeconds);
    CAPTURE(entered.out.targetTicks, longFrameEdge);
    CHECK(entered.out.targetTicks == Catch::Approx(longFrameEdge).margin(1e-9));

    Rig interleaved(lowEdgeConfig());
    interleaved.callEvery(kHundredFpsSeconds, 300);
    Step alternate;
    for (uint32_t frame = 0u; frame < 2u * correctionReads; ++frame)
        alternate = interleaved.callAfter(frame % 2u == 0u ? kHundredFpsSeconds : kLongFrameSeconds);
    CAPTURE(alternate.out.targetTicks);
    CHECK(alternate.out.targetTicks == Catch::Approx(longFrameEdge).margin(1e-9));
}

TEST_CASE("PCTM.ClockDilationController.ShadowFollowsOnsTrajectoryWithoutApplying", "[PCTM][Dilation][ClockDilationController]")
{
    Rig on(commonConfig(ClockDilationMode::On));
    Rig shadow(commonConfig(ClockDilationMode::Shadow));

    std::vector<ClockDilationInputs> script;
    double                           now = 10.0;
    for (int i = 0; i < 400; ++i)
    {
        now += kCallSeconds;
        const double error     = 2.5 * std::sin(i * 0.05) + (i % 7) * 0.01;
        ClockDilationInputs in = inputsWithError(now, error);
        in.sensorValid         = !(i >= 120 && i < 130);
        in.phase.transitionDebt = i >= 200 && i < 205;
        in.phase.hardResyncs   = i >= 300 ? 1u : 0u;
        script.push_back(in);
    }

    uint32_t onApplied    = 0u;
    uint32_t onChanges    = 0u;
    double   previousOn   = 1.0;
    for (const ClockDilationInputs& in : script)
    {
        const Step onStep     = on.call(in);
        const Step shadowStep = shadow.call(in);
        CHECK(shadowStep.out.scale == onStep.out.scale);
        CHECK(shadowStep.out.freeze == onStep.out.freeze);
        CHECK(shadowStep.out.errorTicks == onStep.out.errorTicks);
        CHECK_FALSE(shadowStep.out.apply);
        CHECK(shadowStep.doorRate == 1.0);
        onApplied += onStep.out.apply ? 1u : 0u;
        onChanges += onStep.out.scale != previousOn ? 1u : 0u;
        previousOn = onStep.out.scale;
    }
    CHECK(onApplied == onChanges);
    CHECK(onApplied > 20u);
    CHECK(shadow.door().applied == 0u);
}

TEST_CASE("PCTM.ClockDilationController.TheTargetGainsHalfTheMeanFramePeriod", "[PCTM][Dilation][ClockDilationController]")
{
    ClockDilationConfig config;
    config.mode = ClockDilationMode::Shadow;
    const ClockDilationInputs in = midBandOwnRttInputs();
    const double raw             = ownRttRawTargetOf(in);

    uint32_t framePeriodsRun = 0u;
    for (double frameTicks : { 0.6, 1.0, 2.0, 60.0 / 144.0 })
    {
        Rig rig(config);
        const Step last     = runFrames(rig, in, frameTicks, kStepLagSeconds);
        const double offset = last.out.targetTicks - raw;
        CAPTURE(frameTicks, raw, last.out.targetTicks, offset, stepLagOf(last.out));
        CHECK_FALSE(last.out.guardBound);
        CHECK(stepLagOf(last.out) == Catch::Approx(frameTicks / 2.0).margin(kStepLagTolerance));
        CHECK(offset == Catch::Approx(frameTicks / 2.0).margin(kStepLagTolerance));
        CHECK(last.out.targetTicks - stepLagOf(last.out) == Catch::Approx(raw).margin(1e-9));
        ++framePeriodsRun;
    }
    CHECK(framePeriodsRun == 4u);

    Rig longFrame(config);
    const Step steady   = runFrames(longFrame, in, 0.6, kStepLagSeconds);
    const Step afterOne = longFrame.callAfterWith(kLongFrameSeconds, in);
    const double moved  = stepLagOf(afterOne.out) - stepLagOf(steady.out);
    CAPTURE(stepLagOf(steady.out), stepLagOf(afterOne.out), moved);
    CHECK(moved > 0.0);
    CHECK(moved < 0.01);

    Rig overCap(config);
    const Step before   = runFrames(overCap, in, 0.6, kStepLagSeconds);
    const Step hitch    = overCap.callAfterWith(kOverCapFrameSeconds, in);
    CHECK(stepLagOf(hitch.out) == stepLagOf(before.out));
    CHECK(hitch.out.targetTicks == before.out.targetTicks);

    ClockDilationController off(commonConfig(ClockDilationMode::Off));
    for (const ClockDilationInputs& varied : variedInputs())
    {
        const ClockDilationOutput out = off.update(varied);
        CHECK(out.scale == 1.0);
        CHECK_FALSE(out.apply);
        CHECK(out.leadTicks == 0.0);
        CHECK(out.targetTicks == 0.0);
        CHECK(out.errorTicks == 0.0);
        CHECK(out.freeze == DilationFreeze::ModeOff);
        CHECK_FALSE(out.guardBound);
        CHECK(stepLagOf(out) == 0.0);
    }
}


TEST_CASE("PCTM.ClockDilationController.TheGuardCentreIsTheClocksReadInEitherDrainOrder", "[PCTM][Dilation][ClockDilationController]")
{
    uint32_t runs = 0u;
    for (double commonLeadTicks : {kBelowEveryEdge, kAboveEveryEdge})
    {
        const CentreRun stepStart  = runCentre(true, commonLeadTicks);
        const CentreRun frameStart = runCentre(false, commonLeadTicks);
        CAPTURE(commonLeadTicks, stepStart.last.guardCentreTicks, frameStart.last.guardCentreTicks,
            stepStart.last.targetTicks, frameStart.last.targetTicks);
        REQUIRE(stepStart.readMinusPreviousSensor == kGuardCentreTicks);
        REQUIRE(frameStart.readMinusPreviousSensor == kGuardCentreTicks);
        CHECK(stepStart.last.guardCentreTicks == Catch::Approx(kGuardCentreTicks).margin(1e-9));
        CHECK(frameStart.last.guardCentreTicks == Catch::Approx(kGuardCentreTicks).margin(1e-9));
        CHECK(stepStart.last.guardCentreTicks == Catch::Approx(frameStart.last.guardCentreTicks).margin(1e-9));
        CHECK(stepStart.last.guardBound);
        CHECK(frameStart.last.guardBound);
        const double edge = commonLeadTicks == kBelowEveryEdge ? kGuardCentreTicks + 1.0 : kGuardCentreTicks + 2.5;
        CHECK(stepStart.last.targetTicks == Catch::Approx(edge).margin(1e-9));
        CHECK(frameStart.last.targetTicks == Catch::Approx(edge).margin(1e-9));

        for (const CentreRun* run : {&stepStart, &frameStart})
        {
            CHECK(run->held.guardCentreTicks == run->last.guardCentreTicks);
            CHECK(run->held.targetTicks == Catch::Approx(run->last.targetTicks).margin(1e-9));
            ++runs;
        }
    }
    CHECK(runs == 4u);
}

TEST_CASE("PCTM.ClockDilationController.TheScaleHoldsUnderTwoQuantaExceptWhileFrozen", "[PCTM][Dilation][ClockDilationController]")
{
    const ClockDilationConfig defaults;
    REQUIRE(defaults.changeHysteresisQuanta == 2.0);

    Rig rig(commonConfig(ClockDilationMode::On));
    const Step fiveQuanta = rig.callWithError(0.1);
    CHECK(fiveQuanta.out.scale == Catch::Approx(0.9995).margin(1e-12));
    CHECK(fiveQuanta.out.apply);

    const Step oneQuantum = rig.callWithError(0.08);
    CHECK(oneQuantum.out.scale == fiveQuanta.out.scale);
    CHECK_FALSE(oneQuantum.out.apply);
    const Step oneAndAHalf = rig.callWithError(0.13);
    CHECK(oneAndAHalf.out.scale == fiveQuanta.out.scale);
    CHECK_FALSE(oneAndAHalf.out.apply);
    for (int i = 0; i < 100; ++i)
        REQUIRE(rig.callWithError(0.08).out.scale == fiveQuanta.out.scale);
    CHECK(rig.door().applied == 1u);

    const Step threeQuanta = rig.callWithError(0.04);
    CHECK(threeQuanta.out.scale == Catch::Approx(0.9998).margin(1e-12));
    CHECK(threeQuanta.out.apply);
    CHECK(rig.door().applied == 2u);

    Rig frozenRig(commonConfig(ClockDilationMode::On));
    frozenRig.callWithError(0.1);
    const Step nearOne = frozenRig.callWithError(0.02);
    REQUIRE(nearOne.out.scale == Catch::Approx(0.9999).margin(1e-12));
    ClockDilationInputs frozen = inputsWithError(0.0, 0.02 + kCallStepLagTicks);
    frozen.phase.transitionDebt = true;
    const Step released        = frozenRig.callWith(frozen);
    CAPTURE(nearOne.out.scale, released.out.scale);
    CHECK(released.out.freeze == DilationFreeze::TransitionDebt);
    CHECK(released.out.scale == 1.0);
    CHECK(released.out.apply);
    CHECK(frozenRig.callWith(frozen).out.scale == 1.0);
}

#endif // WITH_LOW_LEVEL_TESTS
