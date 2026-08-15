// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/ResimGateProbe.h"

#include <cstdint>
#include <string>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / item 42: THE RESIM-GATE PROBE.
//
// WHAT THIS TASK IS. Instrumentation, not repair. Item 31 established that resim
// triggers were gated by the prediction-frontier slot's INHERITED `m_isResimulated`
// bit — `getLastResimulationTick` scanned from the frontier at offset 0, so that bit
// shadowed every older corrected slot, and the gate re-opened only when a correction
// landed exactly ON the frontier. Nothing in this file or in the header it tests
// changed that. These cases pin what the counters MEAN.
//
// ⚠ [item 45, 2026-08-11] THE REPAIR HAS SINCE LANDED, in a separate change, and
// this suite was left almost entirely alone — deliberately, and the reason is the
// point of the split: THESE CASES ARE ABOUT ARITHMETIC, NOT ABOUT THE GATE. A
// counter that divides `requested` by `checks` is right or wrong independently of
// what opens the gate. Two things did change:
//   * the gate is now EDGE-TRIGGERED on a pending anchor tick, with the trigger
//     CONFIGURED (`TimeConfig::resimTriggerPolicy`) and shipped defaulted to
//     reproduce the mechanism described above. So the paragraph above is the
//     HISTORICAL mechanism AND the shipped policy's behaviour, which is why the
//     baselines in the header still bind;
//   * one field was added, `deepAnchorExclusions`, with a case of its own below.
// The gate's own semantics are pinned in CorrectionCache/ResimGateSemanticsTest.cpp
// and its policy in CorrectionCache/ResimGatePolicyTest.cpp; if you are here looking
// for "does a behind-frontier correction trigger a resim", it is those files.
//
// ⭐ WHY THIS SUITE IS WRITTEN THE WAY IT IS. Item 42's own framing: "a metric that
// cannot fail is not a metric". This initiative has shipped five instruments that
// measured the wrong quantity under the right name — most recently a loss counter
// that reported 122 per mille on a run that was working perfectly. The live
// falsification (PIE runs, cvar flips) is recorded in the impl note; what THIS file
// can do that a PIE run cannot is pin the arithmetic against the specific wrong
// implementations that would still look healthy:
//
//   1. A WINDOW THAT CLEARS ITS STRADDLING STATE. The natural `resetWindow()` wipes
//      every member. Three pieces of state here deliberately survive the boundary —
//      the previous request's anchor, a request awaiting its grant, and a resim
//      awaiting its apply edge — because all three describe sequences that CROSS
//      windows, and they are exactly the interesting ones. A wiping reset
//      under-reports `repeatRequests` and `clampedGrants` once per window and looks
//      perfect on every single-window test.
//   2. A REFUSAL COUNT THAT CANNOT SEE A REFUSAL. `refusedFrames` is
//      `requests - grants` and is only meaningful because a refusal leaves
//      `needsResimulation()` true so the request repeats (finding §4a). An
//      implementation that clamped it, or computed it per-frame, would read 0
//      forever and 0 is also the healthy value.
//   3. A LANDING SPLIT THAT DROPS DISCARDS. The discard bucket is item 41's
//      `aboveNewest` population and is the one place this probe's sample set MUST
//      differ from its CorrectionVerdictProbe neighbour's. Excluding discards makes
//      `atFrontierPerMille` rise for a reason that has nothing to do with the gate.
//   4. A CLASS SPLIT DRIVEN SYMMETRICALLY. Every partition case below drives its
//      partitions at DIFFERENT counts, so a swapped or pooled implementation lands
//      on neither. One-of-each would certify the exact defect it exists to catch.
//
// NO CACHE, NO SIMULATABLE, NO OWNER, NO LOGGER. The header is `<cstdint>` plus one
// other STL-only header, which is what makes this whole file possible here rather
// than only in the engine-coupled suite.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // A window small enough to close inside a readable test body. Every case that
    // does not care about closing uses a large one instead, so an accidental flush
    // cannot silently reset the counters mid-assertion.
    constexpr std::uint32_t kTinyWindow = 4u;
    constexpr std::uint32_t kBigWindow  = 1000u;

    // Drive `count` declining checks (the overwhelmingly common outcome under the
    // mechanism). Returns how many windows closed.
    std::uint32_t declineChecks(ResimGateProbe& probe, std::uint32_t count)
    {
        std::uint32_t closed = 0u;
        ResimGateWindowSummary window;
        for (std::uint32_t i = 0u; i < count; ++i)
            if (probe.noteCheck(/*requestedResim=*/false, window))
                ++closed;
        return closed;
    }
}

// ---------------------------------------------------------------------------
// I1 — THE DENOMINATOR.
// ---------------------------------------------------------------------------

TEST_CASE("Reconciliation.ResimGateProbe.WindowClosesOnTheNthCheckAndOnlyThen",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kTinyWindow);
    ResimGateWindowSummary window;

    REQUIRE(probe.noteCheck(false, window) == false);
    REQUIRE(probe.noteCheck(false, window) == false);
    REQUIRE(probe.noteCheck(false, window) == false);
    // The Nth check — and only the Nth — closes it.
    REQUIRE(probe.noteCheck(false, window) == true);
    REQUIRE(window.checks == kTinyWindow);

    // ...and the window is genuinely reset, not merely reported. An implementation
    // that reported and kept accumulating would fire on every subsequent check.
    REQUIRE(probe.checkCount() == 0u);
    REQUIRE(probe.noteCheck(false, window) == false);
}

TEST_CASE("Reconciliation.ResimGateProbe.GateSplitsCheckOutcomesAndRatesThem",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);

    // ASYMMETRIC ON PURPOSE — 97 declines against 3 requests. A swapped
    // declined/requested pair is invisible at 50/50 and unmissable here, and the
    // ratio is deliberately near the measured baseline (59 trigger frames over
    // ~2,700 ticks ~= 22 per mille).
    declineChecks(probe, 97u);
    ResimGateWindowSummary ignored;
    probe.noteCheck(true, ignored);
    probe.noteCheck(true, ignored);
    probe.noteCheck(true, ignored);

    ResimGateWindowSummary window;
    probe.fillSummary(window);
    REQUIRE(window.checks    == 100u);
    REQUIRE(window.declined  == 97u);
    REQUIRE(window.requested == 3u);
    // 3/100 = 30 per mille exactly. Per-MILLE and not per-cent because the
    // interesting rates here are single-digit percentages and a percentage would
    // round most windows to 0 and throw the signal away.
    REQUIRE(window.requestRatePerMille == 30u);
}

TEST_CASE("Reconciliation.ResimGateProbe.RatesRoundToNearestRatherThanTruncating",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);

    // 1 request in 3 checks = 333.33... per mille. Truncation gives 333, round-to-
    // nearest gives 333 as well — so the discriminating case is one that rounds UP.
    // 2 in 3 = 666.67 -> 667 rounded, 666 truncated.
    ResimGateWindowSummary window;
    probe.noteCheck(true,  window);
    probe.noteCheck(true,  window);
    probe.noteCheck(false, window);

    probe.fillSummary(window);
    REQUIRE(window.requested == 2u);
    REQUIRE(window.checks    == 3u);
    REQUIRE(window.requestRatePerMille == 667u);
}

TEST_CASE("Reconciliation.ResimGateProbe.EmptyDenominatorsRateZeroRatherThanDividing",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    ResimGateWindowSummary window;
    probe.fillSummary(window);

    // Zero means NO OBSERVATION, not a perfect record. The caller is the thing that
    // decides whether to print it; the probe must not divide by zero to get here.
    REQUIRE(window.checks              == 0u);
    REQUIRE(window.requestRatePerMille == 0u);
    REQUIRE(window.requests            == 0u);
    REQUIRE(window.refusedRatePerMille == 0u);
    REQUIRE(window.minRequestDepth     == 0u);
    REQUIRE(window.maxRequestDepth     == 0u);
}

// ---------------------------------------------------------------------------
// I3 — THE CHAOS REQUEST / REFUSAL LEDGER.
// ---------------------------------------------------------------------------

TEST_CASE("Reconciliation.ResimGateProbe.RefusalIsRequestsMinusGrantsAndNeverUnderflows",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    ResimGateWindowSummary window;

    // Five requests, three of which Chaos executed. The two it did not are the
    // refusals — silent in any normal build, because every refusal diagnostic in
    // FRewindData / FPBDRigidsSolver is compiled out behind DEBUG_REWIND_DATA.
    for (std::uint32_t i = 0u; i < 5u; ++i)
        probe.noteRequest(/*anchorTick=*/100u + i, /*lastCompletedStep=*/200, /*requestedChaosFrame=*/199);
    probe.noteGrant(199);
    probe.noteGrant(199);
    probe.noteGrant(199);

    probe.fillSummary(window);
    REQUIRE(window.requests      == 5u);
    REQUIRE(window.grants        == 3u);
    REQUIRE(window.refusedFrames == 2u);
    REQUIRE(window.refusedRatePerMille == 400u);

    // MORE GRANTS THAN REQUESTS IS LEGAL, and it must not wrap into 4 billion.
    // Finding §3: FNetworkPhysicsCallback merges our frame with engine-side
    // requesters (RewindData->GetResimFrame(), CompareTargetsToLastFrame), so Chaos
    // can rewind on its own initiative with nothing of ours on record.
    probe.noteGrant(199);
    probe.noteGrant(199);
    probe.noteGrant(199);
    probe.fillSummary(window);
    REQUIRE(window.grants        == 6u);
    REQUIRE(window.refusedFrames == 0u);
}

TEST_CASE("Reconciliation.ResimGateProbe.RepeatRequestsNeedTheSameAnchorNotJustAnotherRequest",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    ResimGateWindowSummary window;

    // THE REFUSAL-RUN SIGNATURE from finding §4a, reproduced literally: the same
    // correctionTick asked for three times because nothing cleared
    // needsResimulation(), then a newer correction finally moves the anchor.
    probe.noteRequest(137u, 200, 199);   // first sighting — not a repeat
    probe.noteRequest(137u, 201, 199);   // repeat 1
    probe.noteRequest(137u, 202, 199);   // repeat 2
    probe.noteRequest(141u, 203, 202);   // anchor moved — not a repeat

    probe.fillSummary(window);
    REQUIRE(window.requests       == 4u);
    // Three would mean "every request after the first"; one would mean the run was
    // only counted once. Two is the number of consecutive same-anchor retries.
    REQUIRE(window.repeatRequests == 2u);
}

// ---------------------------------------------------------------------------
// I4 — THE DOMAIN-CONVERSION PIN.
// ---------------------------------------------------------------------------

TEST_CASE("Reconciliation.ResimGateProbe.ClampedGrantsCountAnyRequestedVsGrantedMismatch",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    ResimGateWindowSummary window;

    // Asked for frame 190, Chaos started at 190 — no mismatch.
    probe.noteRequest(100u, 200, 190);
    probe.noteGrant(190);
    // Asked for 191, Chaos started at 194 — PURE ARITHMETIC driving the
    // direction-agnostic detector 0→1 on a requested-vs-granted mismatch.
    // ⚠ NOT the engine's live behaviour: a grant LATER than requested (a shallow
    // clamp) is structurally impossible on this wiring — FindValidResimFrame walks
    // DOWNWARD (return ≤ requested), the engine's FMath::Min merge can only
    // DEEPEN, and the replay-loop push-data skip is dead code, so the observed
    // PhysicsStep always equals the solver's ResimStep ≤ the requested frame
    // (item 42 review §2). What the counter would catch live is an engine-side
    // requester DEEPENING the grant; its live reading is a constant 0 by
    // construction, and any nonzero is an engine-behaviour-change alarm.
    probe.noteRequest(101u, 201, 191);
    probe.noteGrant(194);

    probe.fillSummary(window);
    REQUIRE(window.grants        == 2u);
    REQUIRE(window.clampedGrants == 1u);
}

TEST_CASE("Reconciliation.ResimGateProbe.AGrantWithNoRequestOnRecordIsNotAClamp",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    ResimGateWindowSummary window;

    // An engine-initiated rewind — no request of ours preceded it. Charging it as a
    // clamp would manufacture domain-skew evidence out of a mechanism that has
    // nothing to do with our mapper, which is precisely the misreading I4 exists to
    // support rather than create.
    probe.noteGrant(500);
    probe.fillSummary(window);
    REQUIRE(window.grants        == 1u);
    REQUIRE(window.clampedGrants == 0u);

    // And a request is consumed by its grant: a SECOND unmatched grant after a
    // matched pair is likewise not a clamp.
    probe.noteRequest(1u, 100, 90);
    probe.noteGrant(90);
    probe.noteGrant(77);
    probe.fillSummary(window);
    REQUIRE(window.grants        == 3u);
    REQUIRE(window.clampedGrants == 0u);
}

TEST_CASE("Reconciliation.ResimGateProbe.RequestDepthTracksMinAndMaxAndFloorsTheSkew",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    ResimGateWindowSummary window;

    probe.noteRequest(1u, /*lastCompletedStep=*/200, /*requestedChaosFrame=*/199);  // depth 1
    probe.noteRequest(2u, 200, 196);                                               // depth 4
    probe.noteRequest(3u, 200, 198);                                               // depth 2

    probe.fillSummary(window);
    REQUIRE(window.minRequestDepth == 1u);
    REQUIRE(window.maxRequestDepth == 4u);

    // A NEGATIVE depth — we asked to rewind to a frame at or ahead of the last
    // completed one — is the ±1 ChaosTickMapper skew finding §3 describes. It floors
    // at 0 rather than wrapping to ~4 billion, which would destroy the max as well
    // as the min.
    probe.noteRequest(4u, 200, 203);
    probe.fillSummary(window);
    REQUIRE(window.minRequestDepth == 0u);
    REQUIRE(window.maxRequestDepth == 4u);
}

// ---------------------------------------------------------------------------
// I5 + I6 — THE APPLY EDGE AND THE REPLAY SPAN.
// ---------------------------------------------------------------------------

TEST_CASE("Reconciliation.ResimGateProbe.ApplyLedgerChargesAbandonedToPreparesNotFinishes",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    ResimGateWindowSummary window;

    // Three granted resims, two of which reached the apply edge — the measured
    // ~20 % shortfall present in every archived run.
    probe.notePrepare(100u); probe.noteReplayTick(); probe.noteReplayTick(); probe.noteFinish();
    probe.notePrepare(110u); probe.noteReplayTick(); probe.noteFinish();
    probe.notePrepare(120u); probe.noteReplayTick();   // stranded

    probe.fillSummary(window);
    REQUIRE(window.prepares    == 3u);
    REQUIRE(window.finishes    == 2u);
    REQUIRE(window.abandoned   == 1u);
    REQUIRE(window.replayTicks == 4u);

    // finishes - prepares would be 0 here (and would underflow the other way round);
    // an extra finish must not produce a negative-wrapped `abandoned`.
    probe.noteFinish();
    probe.noteFinish();
    probe.fillSummary(window);
    REQUIRE(window.finishes  == 4u);
    REQUIRE(window.abandoned == 0u);
}

TEST_CASE("Reconciliation.ResimGateProbe.StrandedReportIsOneShotPerEpisodeNotPerStuckFrame",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    StrandedResimEpisode episode;

    // A resim that prepared at 120 and replayed twice, then never applied.
    probe.notePrepare(120u);
    probe.noteReplayTick();
    probe.noteReplayTick();

    // The clock's cursor is now stranded, so normal prediction frames keep arriving
    // with isResimulating() still true. The FIRST one reports.
    REQUIRE(probe.noteStuckResimFrame(/*predictionTick=*/123u, episode) == true);
    REQUIRE(episode.anchorTick     == 120u);
    REQUIRE(episode.predictionTick == 123u);
    REQUIRE(episode.replayedTicks  == 2u);
    // 3 needed, 2 executed — the one-short signature of the ±1 domain skew, as
    // opposed to a far-short reading which would point at the push-data history
    // shortfall instead.
    REQUIRE(episode.catchUpDeficit == 3u);

    // ...and every subsequent frame of the SAME episode counts but does not report.
    // A per-frame Verbose line here is the T19 volume class: the stuck state
    // persists for as long as it takes the next resim to reset the cursor.
    REQUIRE(probe.noteStuckResimFrame(124u, episode) == false);
    REQUIRE(probe.noteStuckResimFrame(125u, episode) == false);

    ResimGateWindowSummary window;
    probe.fillSummary(window);
    REQUIRE(window.stuckResimFrames == 3u);   // all three counted
    REQUIRE(window.prepares         == 1u);
    REQUIRE(window.finishes         == 0u);

    // A NEW episode reports again — the one-shot is per episode, not per session.
    probe.notePrepare(130u);
    probe.noteReplayTick();
    REQUIRE(probe.noteStuckResimFrame(132u, episode) == true);
    REQUIRE(episode.anchorTick     == 130u);
    REQUIRE(episode.replayedTicks  == 1u);
    REQUIRE(episode.catchUpDeficit == 2u);
}

TEST_CASE("Reconciliation.ResimGateProbe.AFinishedEpisodeIsNeverReportedAsStranded",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    StrandedResimEpisode episode;

    probe.notePrepare(100u);
    probe.noteReplayTick();
    probe.noteFinish();

    // A stuck frame with no open episode is still COUNTED — the counter is the
    // boundary-noise-free reading and must not be gated on episode bookkeeping —
    // but there is nothing to report about.
    REQUIRE(probe.noteStuckResimFrame(105u, episode) == false);

    ResimGateWindowSummary window;
    probe.fillSummary(window);
    REQUIRE(window.stuckResimFrames == 1u);
    REQUIRE(window.abandoned        == 0u);
}

TEST_CASE("Reconciliation.ResimGateProbe.ReplayOverrunsAccumulatePerSweepNotPerTick",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kBigWindow);
    ResimGateWindowSummary window;

    // postResimulationAll sweeps EVERY character, so one replay tick can discard
    // more than one slot. Taking a count rather than being a per-event call is what
    // keeps a three-character session from reporting a third of its overruns.
    probe.noteReplayTick();  probe.noteReplayOverruns(0u);
    probe.noteReplayTick();  probe.noteReplayOverruns(3u);
    probe.noteReplayTick();  probe.noteReplayOverruns(1u);

    probe.fillSummary(window);
    REQUIRE(window.replayTicks    == 3u);
    REQUIRE(window.replayOverruns == 4u);
}

// ---------------------------------------------------------------------------
// [item 45] THE DEPTH-POLICY EXCLUSION COUNT — the one field item 45 added.
//
// It counts CHARACTER-FRAMES, not distinct anchors: `checkDivergenceAll` sweeps
// every character and re-examines a stranded deep anchor on every frame it stays
// stranded, so a sustained nonzero is the reading that matters ("an anchor is stuck
// out of reach"), exactly as `refusedFrames` is a frame count and not a distinct-
// request count. A per-anchor counter would report 1 for a permanently stuck anchor
// and look healthy.
//
// ⚠ IT IS A PER-WINDOW COUNTER AND MUST BE CLEARED AT THE BOUNDARY, unlike the four
// straddling values in the next case: an exclusion is a completed observation about
// one frame, not a sequence awaiting its other half.
// ---------------------------------------------------------------------------
TEST_CASE("Reconciliation.ResimGateProbe.DeepAnchorExclusionsAccumulatePerSweepAndClearPerWindow",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateWindowSummary window;

    SECTION("a sweep can exclude more than one character in one frame")
    {
        ResimGateProbe probe(kBigWindow);

        probe.noteDeepAnchorSkips(0u);
        probe.noteDeepAnchorSkips(2u);   // two characters stranded on one frame
        probe.noteDeepAnchorSkips(1u);

        probe.fillSummary(window);
        REQUIRE(window.deepAnchorExclusions == 3u);
    }

    SECTION("the shipped configuration cannot produce one")
    {
        // The depth policy is consulted only under `OnDisagreement`, and the compiled
        // default is `FrontierExact`, so on a default build the manager passes a
        // maxDepth of 0 and `checkDivergenceAll` excludes nothing. A structural 0,
        // not an unexercised counter — this is what makes a nonzero in an item-46 log
        // attributable to the flip rather than to noise.
        ResimGateProbe probe(kBigWindow);
        probe.noteDeepAnchorSkips(0u);
        probe.noteDeepAnchorSkips(0u);

        probe.fillSummary(window);
        REQUIRE(window.deepAnchorExclusions == 0u);
    }

    SECTION("it clears at the window boundary")
    {
        ResimGateProbe probe(kTinyWindow);

        probe.noteDeepAnchorSkips(5u);
        // Close the window: the flush must report the 5 and then forget it.
        bool closed = false;
        for (std::uint32_t i = 0u; i < kTinyWindow; ++i)
            closed = probe.noteCheck(false, window) || closed;
        REQUIRE(closed);
        REQUIRE(window.deepAnchorExclusions == 5u);

        probe.fillSummary(window);
        REQUIRE(window.deepAnchorExclusions == 0u);
    }
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 47] THE HOLLOW-ANCHOR LEDGER —
// `freshClobbersAvoided` / `staleClobbersAvoided`.
//
// Replay ticks whose slot carried a correction and was therefore NOT overwritten.
// Character-SLOT events, on the `replayOverruns` pattern and fed from the same
// sweep, for the same reason: one replay tick sweeps every character and can
// protect more than one slot.
//
// ⚠ THE TWO ARE PASSED IN ONE CALL, and that is deliberate rather than tidy: the
// pair is only meaningful together. `fresh` is the live rate of the defect item 47
// repairs; `stale` is hygiene AND a wiring check. A caller that could report one
// and forget the other would produce a line whose two halves describe different
// replay ticks.
//
// ⛔ AND `replayOverruns` KEEPS ITS POPULATION. A protected slot is NOT a discard —
// it EXISTS, which is exactly what that counter is about. The archived baseline
// (1-2 per RUN) still binds, and the last section pins the two apart.
// ---------------------------------------------------------------------------
TEST_CASE("Reconciliation.ResimGateProbe.ProtectionsSplitFreshFromStaleAndAreNotOverruns",
          "[Reconciliation][ResimGateProbe][ReplayProtect]")
{
    ResimGateWindowSummary window;

    SECTION("one replay tick can protect several characters' slots")
    {
        ResimGateProbe probe(kBigWindow);

        probe.noteReplayTick();  probe.noteCorrectionProtections(0u, 0u);
        probe.noteReplayTick();  probe.noteCorrectionProtections(2u, 1u);   // 3 characters
        probe.noteReplayTick();  probe.noteCorrectionProtections(1u, 0u);

        probe.fillSummary(window);
        REQUIRE(window.replayTicks          == 3u);
        REQUIRE(window.freshClobbersAvoided == 3u);
        REQUIRE(window.staleClobbersAvoided == 1u);
    }

    SECTION("a protection is not an overrun — the two counters never bleed")
    {
        ResimGateProbe probe(kBigWindow);

        probe.noteReplayTick();
        probe.noteReplayOverruns(2u);
        probe.noteCorrectionProtections(5u, 4u);

        probe.fillSummary(window);
        REQUIRE(window.replayOverruns       == 2u);
        REQUIRE(window.freshClobbersAvoided == 5u);
        REQUIRE(window.staleClobbersAvoided == 4u);
    }

    SECTION("the single-character structural zero, as the probe would see it")
    {
        // For ONE character the replay span is `anchor+1..frontier` and "stale"
        // needs `tick < capturedAnchor` — disjoint ranges, so every protection a
        // one-character session can produce is FRESH. A nonzero `stale` in such a
        // log means the classifier is comparing against the folded min instead of
        // the per-cache capture; it does NOT mean a new population appeared. The
        // reachability argument is at `resimGate::classifyResimSlotWrite`.
        ResimGateProbe probe(kBigWindow);

        probe.noteReplayTick();  probe.noteCorrectionProtections(1u, 0u);
        probe.noteReplayTick();  probe.noteCorrectionProtections(1u, 0u);

        probe.fillSummary(window);
        REQUIRE(window.staleClobbersAvoided == 0u);
        // ...and NOT the trivial zero of "nothing was protected".
        REQUIRE(window.freshClobbersAvoided == 2u);
    }

    SECTION("both clear at the window boundary")
    {
        // Per-window like `deepAnchorExclusions`, and for the same reason: each is
        // a completed observation about one replay tick, not a sequence awaiting
        // its other half (contrast the four straddling values in the next case).
        ResimGateProbe probe(kTinyWindow);

        probe.noteCorrectionProtections(3u, 2u);

        bool closed = false;
        for (std::uint32_t i = 0u; i < kTinyWindow; ++i)
            closed = probe.noteCheck(false, window) || closed;
        REQUIRE(closed);
        REQUIRE(window.freshClobbersAvoided == 3u);
        REQUIRE(window.staleClobbersAvoided == 2u);

        probe.fillSummary(window);
        REQUIRE(window.freshClobbersAvoided == 0u);
        REQUIRE(window.staleClobbersAvoided == 0u);
    }
}

// ---------------------------------------------------------------------------
// THE WINDOW BOUNDARY — what must be cleared and what must not.
// ---------------------------------------------------------------------------

TEST_CASE("Reconciliation.ResimGateProbe.CrossWindowSequencesSurviveTheBoundary",
          "[Reconciliation][ResimGateProbe]")
{
    ResimGateProbe probe(kTinyWindow);
    ResimGateWindowSummary window;

    // A refusal run, a request awaiting its grant, and a resim awaiting its apply
    // edge — all three straddling the boundary, which is the case the natural
    // "clear every member" reset silently loses. Under such a reset this test reads
    // repeat=0 / clamped=0 / abandoned=0, i.e. three perfectly healthy numbers.
    probe.noteRequest(137u, 200, 199);
    probe.notePrepare(137u);
    REQUIRE(probe.noteCheck(false, window) == false);
    REQUIRE(probe.noteCheck(false, window) == false);
    REQUIRE(probe.noteCheck(false, window) == false);
    REQUIRE(probe.noteCheck(false, window) == true);   // window closes HERE
    REQUIRE(window.requests  == 1u);
    REQUIRE(window.prepares  == 1u);
    REQUIRE(window.abandoned == 1u);                   // finish has not happened yet

    // Now the next window sees the continuation. The repeat is detected against an
    // anchor recorded in the PREVIOUS window; the grant mismatch (pure arithmetic
    // here, like the I4 case above — not live engine behaviour) is detected against
    // a chaos frame requested in the PREVIOUS window.
    probe.noteRequest(137u, 204, 199);
    probe.noteGrant(202);
    probe.noteFinish();
    for (std::uint32_t i = 0u; i < kTinyWindow; ++i)
        probe.noteCheck(false, window);

    REQUIRE(window.repeatRequests == 1u);
    REQUIRE(window.clampedGrants  == 1u);
    // The counters themselves DID reset: this window saw one request, one grant,
    // no prepare and one finish.
    REQUIRE(window.requests == 1u);
    REQUIRE(window.grants   == 1u);
    REQUIRE(window.prepares == 0u);
    REQUIRE(window.finishes == 1u);
}

// ---------------------------------------------------------------------------
// I2 — THE FRONTIER-LANDING SPLIT.
// ---------------------------------------------------------------------------

TEST_CASE("Reconciliation.ResimGateProbe.ClassifyPutsDiscardsInTheirOwnBucketRegardlessOfTick",
          "[Reconciliation][ResimGateProbe]")
{
    // A DISCARD HAS NO SLOT, so comparing its tick against the frontier would
    // classify an event that never happened. `landed == false` must win outright —
    // including in the case that makes the mistake invisible, where the discarded
    // tick happens to EQUAL the frontier.
    REQUIRE(classifyCorrectionLanding(/*landed=*/false, 40u, 40u) == CorrectionLandingSite::Discarded);
    REQUIRE(classifyCorrectionLanding(false, 9000u, 40u)          == CorrectionLandingSite::Discarded);
    REQUIRE(classifyCorrectionLanding(false, 1u, 40u)             == CorrectionLandingSite::Discarded);

    // THE ONLY EVENT THAT RE-OPENS THE GATE IN PLAY.
    REQUIRE(classifyCorrectionLanding(/*landed=*/true, 40u, 40u) == CorrectionLandingSite::AtFrontier);

    // Behind the frontier: sets its own slot's flag, is shadowed by the frontier's
    // inherited resim bit, is never replayed through, never touches live state.
    REQUIRE(classifyCorrectionLanding(true, 39u, 40u) == CorrectionLandingSite::Behind);
    REQUIRE(classifyCorrectionLanding(true,  1u, 40u) == CorrectionLandingSite::Behind);
}

TEST_CASE("Reconciliation.ResimGateProbe.LandingSplitsByClassAndNeverPools",
          "[Reconciliation][ResimGateProbe]")
{
    CorrectionLandingProbe probe(kBigWindow);
    CorrectionLandingWindowSummary window;

    // ASYMMETRIC IN BOTH DIMENSIONS — different totals per class AND a different
    // bucket mix within each. A pooled implementation matches neither class; a
    // swapped one matches neither; and neither could be detected from a one-of-each
    // drive, which is how T24's class-split case had to be rewritten.
    for (int i = 0; i < 7; ++i)
        probe.noteLanding(PredictedCharacterClass::LocallyPredicted, CorrectionLandingSite::Behind, window);
    for (int i = 0; i < 2; ++i)
        probe.noteLanding(PredictedCharacterClass::LocallyPredicted, CorrectionLandingSite::AtFrontier, window);
    probe.noteLanding(PredictedCharacterClass::LocallyPredicted, CorrectionLandingSite::Discarded, window);

    for (int i = 0; i < 3; ++i)
        probe.noteLanding(PredictedCharacterClass::RemoteProxy, CorrectionLandingSite::Behind, window);
    probe.noteLanding(PredictedCharacterClass::RemoteProxy, CorrectionLandingSite::AtFrontier, window);
    for (int i = 0; i < 4; ++i)
        probe.noteLanding(PredictedCharacterClass::RemoteProxy, CorrectionLandingSite::Discarded, window);

    probe.fillSummary(window);
    REQUIRE(window.local.landedBehind     == 7u);
    REQUIRE(window.local.landedAtFrontier == 2u);
    REQUIRE(window.local.discarded        == 1u);
    REQUIRE(window.remote.landedBehind     == 3u);
    REQUIRE(window.remote.landedAtFrontier == 1u);
    REQUIRE(window.remote.discarded        == 4u);
    REQUIRE(window.samples == 18u);
    REQUIRE(window.samples == window.local.total() + window.remote.total());

    // Introspection agrees with the summary — the accessor the wiring suite asserts
    // through must not be a second, drifting view of the same counters.
    REQUIRE(probe.countFor(PredictedCharacterClass::LocallyPredicted,
                           CorrectionLandingSite::Behind) == 7u);
    REQUIRE(probe.countFor(PredictedCharacterClass::RemoteProxy,
                           CorrectionLandingSite::Discarded) == 4u);
}

TEST_CASE("Reconciliation.ResimGateProbe.DiscardsAreSamplesAndSitInTheFrontierRateDenominator",
          "[Reconciliation][ResimGateProbe]")
{
    CorrectionLandingProbe probe(kBigWindow);
    CorrectionLandingWindowSummary window;

    // 1 at-frontier, 1 behind, 2 discarded. The frontier-touch rate is 1/4 = 250 per
    // mille. An implementation that excluded discards from the denominator would
    // report 1/2 = 500 — DOUBLE, and in the healthy-looking direction, on a client
    // whose corrections are mostly landing outside the cache window at all. Item 41
    // measured one client at 70.76 % aboveNewest; under a landed-only denominator
    // that client would report the best frontier-touch rate in the run.
    probe.noteLanding(PredictedCharacterClass::RemoteProxy, CorrectionLandingSite::AtFrontier, window);
    probe.noteLanding(PredictedCharacterClass::RemoteProxy, CorrectionLandingSite::Behind, window);
    probe.noteLanding(PredictedCharacterClass::RemoteProxy, CorrectionLandingSite::Discarded, window);
    probe.noteLanding(PredictedCharacterClass::RemoteProxy, CorrectionLandingSite::Discarded, window);

    probe.fillSummary(window);
    REQUIRE(window.samples == 4u);
    REQUIRE(window.remote.total() == 4u);
    REQUIRE(window.remote.atFrontierRatePerMille == 250u);

    // A class with nothing in it rates 0 — no observation, not a perfect record.
    // The emitter SKIPS such a block rather than printing it.
    REQUIRE(window.local.total() == 0u);
    REQUIRE(window.local.atFrontierRatePerMille == 0u);
}

TEST_CASE("Reconciliation.ResimGateProbe.LandingWindowClosesOnTheCombinedCount",
          "[Reconciliation][ResimGateProbe]")
{
    CorrectionLandingProbe probe(kTinyWindow);
    CorrectionLandingWindowSummary window;

    // TOTAL-DRIVEN, NOT PER-CLASS-DRIVEN — the CorrectionVerdictProbe rule, and for
    // the same reason: if each class closed on its own count the two summary lines
    // would describe different and unstated intervals, and comparing their rates
    // would be comparing different sessions.
    REQUIRE(probe.noteLanding(PredictedCharacterClass::LocallyPredicted,
                              CorrectionLandingSite::Behind, window) == false);
    REQUIRE(probe.noteLanding(PredictedCharacterClass::RemoteProxy,
                              CorrectionLandingSite::Discarded, window) == false);
    REQUIRE(probe.noteLanding(PredictedCharacterClass::RemoteProxy,
                              CorrectionLandingSite::AtFrontier, window) == false);
    REQUIRE(probe.noteLanding(PredictedCharacterClass::LocallyPredicted,
                              CorrectionLandingSite::Behind, window) == true);

    REQUIRE(window.samples == kTinyWindow);
    REQUIRE(window.local.landedBehind == 2u);
    REQUIRE(window.remote.total()     == 2u);

    // And it reset.
    REQUIRE(probe.sampleCount() == 0u);
    REQUIRE(probe.countFor(PredictedCharacterClass::LocallyPredicted,
                           CorrectionLandingSite::Behind) == 0u);
}

TEST_CASE("Reconciliation.ResimGateProbe.BothProbesShareTheDivergenceProbeWindowLength",
          "[Reconciliation][ResimGateProbe]")
{
    // Item 42: "mirror LogOGDivergenceProbe's window mechanism and window length
    // exactly (same constant source, do not invent a second window size)". A second
    // literal here would drift silently the day the first one moves, and the two
    // probe families' lines would then describe different intervals while looking
    // like a matched pair in a log.
    REQUIRE(kResimGateProbeWindowSamples == kCorrectionVerdictProbeWindowSamples);

    ResimGateProbe        gate;
    CorrectionLandingProbe landing;
    REQUIRE(gate.windowSamples()    == kResimGateProbeWindowSamples);
    REQUIRE(landing.windowSamples() == kResimGateProbeWindowSamples);

    // A zero window would never close — it is coerced to the default rather than
    // silently disabling the whole instrument, which is the failure mode this task
    // exists to stop.
    REQUIRE(ResimGateProbe(0u).windowSamples()         == kResimGateProbeWindowSamples);
    REQUIRE(CorrectionLandingProbe(0u).windowSamples() == kResimGateProbeWindowSamples);
}

TEST_CASE("Reconciliation.ResimGateProbe.SiteNamesAreTheStringsAnOperatorGreps",
          "[Reconciliation][ResimGateProbe]")
{
    // Defined once beside the enum so the spellings in a shipped log line and in a
    // validation pack's grep cannot drift apart.
    REQUIRE(std::string(correctionLandingSiteName(CorrectionLandingSite::Behind))     == "Behind");
    REQUIRE(std::string(correctionLandingSiteName(CorrectionLandingSite::AtFrontier)) == "AtFrontier");
    REQUIRE(std::string(correctionLandingSiteName(CorrectionLandingSite::Discarded))  == "Discarded");
}

#endif // WITH_LOW_LEVEL_TESTS
