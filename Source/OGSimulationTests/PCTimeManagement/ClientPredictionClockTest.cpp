// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/NetworkTimeEstimator.h"

// ---------------------------------------------------------------------------
// Test helpers
//
// Strategy: NetworkTimeEstimator has no RTT sample on construction →
//   getPredictionOffsetTicks() == 0  →  getTargetPredictionTick() == authorityTick.
// We control the "target" simply by calling recordAuthorityTick() before each
// advancePrediction().  TimeConfig is stored by const-ref, so mutating the
// local cfg object is immediately visible to both estimator and clock.
// ---------------------------------------------------------------------------

// Advance the clock N times with a huge soft-drift threshold (dead-band) so no
// correction fires.  Used to wind the prediction tick up to a known starting value.
static void advanceNoDrift(ClientPredictionClock& clock, NetworkTimeEstimator& est,
                           unsigned int targetOffset, unsigned int n)
{
    for (unsigned int i = 0; i < n; ++i)
    {
        est.recordAuthorityTick(clock.getPredictionTick() + targetOffset);
        clock.advancePrediction();
    }
}

// ---------------------------------------------------------------------------
// AC1 — Zero drift: every call increments by exactly 1
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.ZeroDrift", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck  = 0;    // no startup guard
    cfg.softDriftThresholdTicks   = 5;
    cfg.hardResyncThresholdTicks  = 15;
    cfg.gradualCorrectionRate     = 4;
    cfg.tickFrequency             = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // Keep authorityTick == predictionTick → drift == 0 → always dead-band
    for (unsigned int i = 0; i < 20; ++i)
    {
        const unsigned int before = clock.getPredictionTick();
        est.recordAuthorityTick(clock.getPredictionTick());   // drift = 0
        clock.advancePrediction();
        REQUIRE(clock.getPredictionTick() == before + 1);
    }
}

// ---------------------------------------------------------------------------
// AC2 — +5 drift (client behind), rate=4: every 4th call advances by 2
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.SkipCorrection", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck  = 0;
    cfg.softDriftThresholdTicks   = 3;
    cfg.hardResyncThresholdTicks  = 15;
    cfg.gradualCorrectionRate     = 4;
    cfg.tickFrequency             = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // Advance 4 calls with drift = +5 (client behind target by 5)
    // Expected: calls 1-3 advance by 1; call 4 advances by 2 (skip)
    unsigned int totalAdvance = 0;
    for (unsigned int call = 1; call <= 4; ++call)
    {
        const unsigned int before = clock.getPredictionTick();
        est.recordAuthorityTick(clock.getPredictionTick() + 5);
        clock.advancePrediction();
        const unsigned int delta = clock.getPredictionTick() - before;

        if (call < 4)
            REQUIRE(delta == 1u);
        else
            REQUIRE(delta == 2u);

        totalAdvance += delta;
    }
    REQUIRE(totalAdvance == 5u);
}

// ---------------------------------------------------------------------------
// AC3 — -5 drift (client ahead), rate=4: every 4th call does not advance
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.StallCorrection", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck  = 0;
    cfg.softDriftThresholdTicks   = 3;
    cfg.hardResyncThresholdTicks  = 15;
    cfg.gradualCorrectionRate     = 4;
    cfg.tickFrequency             = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // Wind prediction tick up to 20 in dead-band so target=predTick-5 doesn't underflow
    cfg.softDriftThresholdTicks = 100;
    advanceNoDrift(clock, est, 0, 20);
    cfg.softDriftThresholdTicks = 3;    // restore real threshold

    // Now predictionTick == 20. Keep authorityTick = predictionTick - 5 → drift = -5 (client ahead)
    unsigned int totalAdvance = 0;
    for (unsigned int call = 1; call <= 4; ++call)
    {
        const unsigned int before = clock.getPredictionTick();
        // authorityTick = predTick - 5 → target = predTick - 5 → drift = -5
        const int authTick = static_cast<int>(clock.getPredictionTick()) - 5;
        est.recordAuthorityTick(static_cast<unsigned int>(authTick));
        clock.advancePrediction();
        const unsigned int delta = clock.getPredictionTick() - before;

        if (call < 4)
            REQUIRE(delta == 1u);
        else
            REQUIRE(delta == 0u);

        totalAdvance += delta;
    }
    REQUIRE(totalAdvance == 3u);
}

// ---------------------------------------------------------------------------
// AC4 — +20 drift: hard resync fires, predictionTick jumps, callbacks called
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.HardResync", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck  = 0;
    cfg.softDriftThresholdTicks   = 3;
    cfg.hardResyncThresholdTicks  = 15;
    cfg.gradualCorrectionRate     = 4;
    cfg.tickFrequency             = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    bool callbackFired = false;
    unsigned int callbackTick = 0;

    clock.registerResyncCallback([&](unsigned int newTick)
    {
        callbackFired = true;
        callbackTick  = newTick;
    });

    // Set drift to +20 (well above hardResyncThreshold=15)
    const unsigned int targetTick = clock.getPredictionTick() + 20;
    est.recordAuthorityTick(targetTick);
    clock.advancePrediction();

    REQUIRE(callbackFired);
    REQUIRE(callbackTick == targetTick);
    REQUIRE(clock.getPredictionTick() == targetTick);
    REQUIRE(!clock.isResimulating());
}

// ---------------------------------------------------------------------------
// AC5 — Resimulation cursor: startResimulation → advance → isResimulating
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ClientPredictionClock.Resimulation", "[PCTM][ClientPredictionClock]")
{
    TimeConfig cfg;
    cfg.minTicksBeforeDriftCheck = 0;
    cfg.softDriftThresholdTicks  = 100;   // dead-band for all setup advances
    cfg.tickFrequency            = 60.0;

    NetworkTimeEstimator est(cfg, nullptr);
    ClientPredictionClock clock(cfg, est, nullptr);

    // Advance to tick 10 in dead-band
    advanceNoDrift(clock, est, 0, 10);
    REQUIRE(!clock.isResimulating());

    // Start resimulation from tick 5 (5 ticks behind prediction frontier)
    clock.startResimulation(5);
    REQUIRE(clock.isResimulating());
    REQUIRE(clock.getResimulationTick() == 5u);

    // Advance resimulation up to the frontier
    for (unsigned int i = 5; i < 10; ++i)
    {
        REQUIRE(clock.isResimulating());
        clock.advanceResimulation();
    }

    // finishResimulation explicitly
    REQUIRE(!clock.isResimulating());

    // Verify getResimulationStep reports isResimulating == true during resim
    clock.startResimulation(7);
    REQUIRE(clock.getResimulationStep().getIsResimulating());

    clock.finishResimulation();
    REQUIRE(!clock.isResimulating());
    REQUIRE(clock.getResimulationTick() == clock.getPredictionTick());
}

#endif // WITH_LOW_LEVEL_TESTS
