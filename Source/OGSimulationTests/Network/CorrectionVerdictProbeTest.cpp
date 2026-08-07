// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/Network/CorrectionVerdictProbe.h"

#include <cstdint>
#include <string>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T24: the CORRECTION-VERDICT probe.
//
// WHAT THIS TASK IS. Exposure, not construction. The verdict being counted here —
// `m_stateBuffer[i].isSimilarTo(state)` — has always been computed inside
// `StateCorrectionCache::tryInsertingCorrectState`, on every correction, for every
// character, on every tick, and has always been stored. It was invisible purely
// because its log line carries no tag and therefore falls to the `LogOG` fallback,
// which ships at Warning. Zero occurrences in the clean floor-0 run. Nothing in
// this file or in the header it tests performs a comparison.
//
// WHAT THESE CASES EXIST TO PIN, in order of how easy each is to get wrong:
//
//   1. THE TWO CLASSES ARE NEVER POOLED. A pooled counter is the implementation
//      someone would actually write, it passes every "the probe counts" assertion,
//      and it destroys the only thing T23 scenario 4 can read: remote proxies are
//      the sole half that can move with the relay delay floor, and a locally
//      predicted population that cannot move would dilute their rate by roughly a
//      third in the tested topology. Every case below therefore drives the two
//      classes with DIFFERENT rates, so a pooled implementation lands on neither.
//
//   2. THE WINDOW CLOSES ON THE COMBINED COUNT, not per class. Two class lines
//      that closed on their own counters would describe different, unstated time
//      intervals, and comparing their rates would be comparing different sessions.
//      The case below completes a window with an ASYMMETRIC class split so a
//      per-class window boundary produces a visibly different answer.
//
//   3. AN EMPTY CLASS REPORTS rate 0 — and the CALLER, not the probe, is what must
//      keep that out of a log. `corrections == 0` means "nothing observed", which
//      would read as "a perfect record" if it were printed. The probe's job is to
//      report the zero honestly; SimulationNetSync's emitter is what skips the
//      block, and the wiring for that lives in og-brawler-tests.
//
//   4. A CLOSED WINDOW RESETS BOTH CLASSES. A reset that missed one class would
//      make that class's counters monotonically accumulate across the session
//      while the other's did not, and the resulting rate would silently converge
//      on a session average that nothing asked for.
//
// NOTE ON WHAT THIS CANNOT SHOW. These cases feed the probe BY HAND. That the
// SHIPPED correction callback actually feeds it, and classifies with the same
// provider-presence test the rest of SimulationNetSync uses, needs
// SimulatableOwnerTraits bound to concrete owners and therefore lives in
// og-brawler-tests ([DivergenceProbeWiring]). A probe wired to nothing passes
// every case in this file and reports all-zero counters forever.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    constexpr PredictedCharacterClass kLocal  = PredictedCharacterClass::LocallyPredicted;
    constexpr PredictedCharacterClass kRemote = PredictedCharacterClass::RemoteProxy;

    // Feed `count` corrections of one class, of which `disagreeing` disagreed.
    // Returns how many of them closed a window.
    std::uint32_t feed(CorrectionVerdictProbe& probe,
                       PredictedCharacterClass characterClass,
                       std::uint32_t count,
                       std::uint32_t disagreeing)
    {
        std::uint32_t closed = 0u;
        CorrectionVerdictWindowSummary summary;
        for (std::uint32_t i = 0u; i < count; ++i)
        {
            const bool predictionWasCorrect = (i >= disagreeing);
            if (probe.noteCorrection(characterClass, predictionWasCorrect, summary))
            {
                ++closed;
            }
        }
        return closed;
    }
}

// ---------------------------------------------------------------------------
// 1 — THE CLASS SPLIT. The two populations are counted separately, and the
// disagreement rate of one says nothing about the other.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.CorrectionVerdictProbe.CountsTheTwoClassesSeparately",
          "[Network][DivergenceProbe]")
{
    // A window far larger than the sample count, so nothing closes and the
    // in-progress counters are what is under test.
    CorrectionVerdictProbe probe(1000u);

    // LOCAL: 10 corrections, 1 disagreement. REMOTE: 4 corrections, 3
    // disagreements. Deliberately different in BOTH count and rate, so no single
    // pooled number coincides with either class's answer.
    feed(probe, kLocal,  10u, 1u);
    feed(probe, kRemote,  4u, 3u);

    REQUIRE(probe.correctionsFor(kLocal)     == 10u);
    REQUIRE(probe.disagreementsFor(kLocal)   == 1u);
    REQUIRE(probe.correctionsFor(kRemote)    == 4u);
    REQUIRE(probe.disagreementsFor(kRemote)  == 3u);

    // The combined count is the WINDOW's size, and is the only place the two are
    // ever added together.
    REQUIRE(probe.sampleCount() == 14u);

    CorrectionVerdictWindowSummary summary;
    probe.fillSummary(summary);

    REQUIRE(summary.samples == 14u);
    REQUIRE(summary.local.corrections   == 10u);
    REQUIRE(summary.local.disagreements == 1u);
    REQUIRE(summary.remote.corrections   == 4u);
    REQUIRE(summary.remote.disagreements == 3u);

    // 1/10 = 100 per mille; 3/4 = 750 per mille. THE POOLED ANSWER WOULD BE
    // 4/14 = 286 — equal to neither, which is exactly why the split exists.
    REQUIRE(summary.local.disagreementRatePerMille  == 100u);
    REQUIRE(summary.remote.disagreementRatePerMille == 750u);
    REQUIRE_FALSE(summary.remote.disagreementRatePerMille == 286u);
}

// ---------------------------------------------------------------------------
// 2 — THE WINDOW IS COMBINED-COUNT DRIVEN, so both class lines describe ONE
// interval.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.CorrectionVerdictProbe.WindowClosesOnTheCombinedCount",
          "[Network][DivergenceProbe]")
{
    CorrectionVerdictProbe probe(10u);
    CorrectionVerdictWindowSummary summary;

    // SEVEN local corrections. A per-class window of 10 would not be close to
    // closing; the combined window is 7/10 of the way there.
    REQUIRE(feed(probe, kLocal, 7u, 2u) == 0u);
    REQUIRE(probe.sampleCount() == 7u);

    // Two remote corrections take the combined count to 9 — still short.
    REQUIRE(probe.noteCorrection(kRemote, false, summary) == false);
    REQUIRE(probe.noteCorrection(kRemote, true,  summary) == false);
    REQUIRE(probe.sampleCount() == 9u);

    // The TENTH correction closes it, and the summary carries BOTH classes over
    // the SAME interval — 7 local and 3 remote, not 10 of either.
    REQUIRE(probe.noteCorrection(kRemote, false, summary) == true);

    REQUIRE(summary.samples == 10u);
    REQUIRE(summary.local.corrections    == 7u);
    REQUIRE(summary.local.disagreements  == 2u);
    REQUIRE(summary.remote.corrections   == 3u);
    REQUIRE(summary.remote.disagreements == 2u);
    REQUIRE(summary.samples == summary.local.corrections + summary.remote.corrections);

    // 2/7 = 285.7 -> 286 rounded to nearest; 2/3 = 666.7 -> 667.
    REQUIRE(summary.local.disagreementRatePerMille  == 286u);
    REQUIRE(summary.remote.disagreementRatePerMille == 667u);
}

// ---------------------------------------------------------------------------
// 3 — AN EMPTY CLASS. The probe reports zero honestly; keeping that out of the
// log is the caller's job.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.CorrectionVerdictProbe.EmptyClassReportsZeroRatherThanAPerfectRecord",
          "[Network][DivergenceProbe]")
{
    // The single-remote-character client with no OTHER remote proxies is not
    // hypothetical — it is the shipped two-client topology, where one class of a
    // window is routinely empty.
    CorrectionVerdictProbe probe(4u);
    CorrectionVerdictWindowSummary summary;

    REQUIRE(feed(probe, kLocal, 3u, 0u) == 0u);
    REQUIRE(probe.noteCorrection(kLocal, true, summary) == true);

    REQUIRE(summary.local.corrections   == 4u);
    REQUIRE(summary.local.disagreements == 0u);

    // Zero corrections and zero disagreements produce rate 0 — INDISTINGUISHABLE
    // from a perfect record on the numbers alone. `corrections == 0` is the field
    // that separates them, and it is what the emitter gates on.
    REQUIRE(summary.remote.corrections   == 0u);
    REQUIRE(summary.remote.disagreements == 0u);
    REQUIRE(summary.remote.disagreementRatePerMille == 0u);
    REQUIRE(summary.local.disagreementRatePerMille  == 0u);
    REQUIRE(summary.local.corrections != summary.remote.corrections);
}

// ---------------------------------------------------------------------------
// 4 — A CLOSED WINDOW RESETS BOTH CLASSES.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.CorrectionVerdictProbe.ClosingAWindowResetsBothClasses",
          "[Network][DivergenceProbe]")
{
    CorrectionVerdictProbe probe(4u);
    CorrectionVerdictWindowSummary first;

    // Window 1: 2 local (both disagreeing), 2 remote (neither disagreeing).
    REQUIRE(probe.noteCorrection(kLocal,  false, first) == false);
    REQUIRE(probe.noteCorrection(kLocal,  false, first) == false);
    REQUIRE(probe.noteCorrection(kRemote, true,  first) == false);
    REQUIRE(probe.noteCorrection(kRemote, true,  first) == true);

    REQUIRE(first.local.corrections    == 2u);
    REQUIRE(first.local.disagreements  == 2u);
    REQUIRE(first.remote.corrections   == 2u);
    REQUIRE(first.remote.disagreements == 0u);

    // Everything is back to zero, on BOTH classes.
    REQUIRE(probe.sampleCount()             == 0u);
    REQUIRE(probe.correctionsFor(kLocal)    == 0u);
    REQUIRE(probe.disagreementsFor(kLocal)  == 0u);
    REQUIRE(probe.correctionsFor(kRemote)   == 0u);
    REQUIRE(probe.disagreementsFor(kRemote) == 0u);

    // Window 2 inverts the rates. If the reset had missed a class, window 2's
    // numbers for it would still carry window 1's, and the local rate below would
    // read 2/6 rather than 0/4.
    CorrectionVerdictWindowSummary second;
    REQUIRE(probe.noteCorrection(kLocal,  true,  second) == false);
    REQUIRE(probe.noteCorrection(kLocal,  true,  second) == false);
    REQUIRE(probe.noteCorrection(kLocal,  true,  second) == false);
    REQUIRE(probe.noteCorrection(kLocal,  true,  second) == true);

    REQUIRE(second.samples             == 4u);
    REQUIRE(second.local.corrections   == 4u);
    REQUIRE(second.local.disagreements == 0u);
    REQUIRE(second.local.disagreementRatePerMille == 0u);
    REQUIRE(second.remote.corrections  == 0u);
}

// ---------------------------------------------------------------------------
// 5 — THE SHIPPED WINDOW SIZE, and the degenerate-argument guard.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.CorrectionVerdictProbe.WindowSizeDefaultAndZeroGuard",
          "[Network][DivergenceProbe]")
{
    CorrectionVerdictProbe shipped;
    REQUIRE(shipped.windowSamples() == kCorrectionVerdictProbeWindowSamples);
    REQUIRE(kCorrectionVerdictProbeWindowSamples == 120u);

    // A zero window would close on every single correction — one Warning line per
    // correction per character, i.e. precisely the per-tick volume class T19 was
    // filed to remove. Clamped to the default rather than accepted.
    CorrectionVerdictProbe degenerate(0u);
    REQUIRE(degenerate.windowSamples() == kCorrectionVerdictProbeWindowSamples);

    CorrectionVerdictWindowSummary summary;
    REQUIRE(degenerate.noteCorrection(kRemote, false, summary) == false);
    REQUIRE(degenerate.sampleCount() == 1u);
}

// ---------------------------------------------------------------------------
// 6 — THE CLASS NAMES an operator greps for are defined once, in the header.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.CorrectionVerdictProbe.ClassNamesAreStableStrings",
          "[Network][DivergenceProbe]")
{
    REQUIRE(std::string(predictedCharacterClassName(kRemote)) == "RemoteProxy");
    REQUIRE(std::string(predictedCharacterClassName(kLocal))  == "LocallyPredicted");
}

#endif // WITH_LOW_LEVEL_TESTS
