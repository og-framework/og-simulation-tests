// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/Network/RelayWritePathProbe.h"

#include <cstdint>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T22: the SERVER-side write-path probes.
//
// WHAT THESE PIN, in order of how easy each is to get wrong:
//
//   1. THE WINDOW CLOSES ON A COMPLETED RUN, NEVER ON A WRITE. The run in progress
//      has an unknown length until a later frame's first write ends it. An
//      implementation that closed on write count would count a truncated run once
//      per window — biasing exactly the mean this probe exists to measure, in the
//      direction that makes coalescing look smaller than it is. The cases below
//      arrange for the truncation to be VISIBLE (a long run straddling the window
//      edge) so the wrong implementation reports a different number rather than
//      the same one.
//
//   2. COALESCING LOSS AND UPSTREAM LOSS ARE DIFFERENT CHANNELS AND ARE NEVER
//      SUMMED. Two writes in one frame is coalescing (fixable by depth). A write
//      whose capture tick skipped one is a tick the server never received
//      (not fixable by depth). A single "loss" counter would make the two
//      indistinguishable and would make a depth decision on the wrong evidence.
//
//   3. `observableX1000` IS THE NUMBER THE WHOLE TASK TURNS ON. It is compared
//      directly against the client's measured delivered fraction (§9.11: 593‰), so
//      an off-by-one in either `runs` or `writes` silently moves the verdict. It is
//      asserted here on hand-computed distributions, not on round numbers.
//
//   4. QueuedBits IS NEGATIVE WHEN THERE IS HEADROOM. It is a debt counter, so
//      `min` is the most headroom and `max` is the closest to saturation. Getting
//      that backwards would report a healthy connection as a saturated one.
//
// Pure telemetry: nothing here feeds back into any simulated value.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    constexpr unsigned int kOwnerA = 11u;
    constexpr unsigned int kOwnerB = 22u;

    // -----------------------------------------------------------------------
    // [og-netcode-v2-input-relay T34] THE TWO WRITE REGIMES, BOTH KEPT.
    //
    // `observableX1000` answers "of what the server received, how much could a
    // once-per-poll publish ever see". Its DEFINITION did not change with item 34;
    // its arithmetic did, because the publish step did:
    //
    //   kReplaceLatestStage  the RETIRED path — one write per frame reached the
    //                        wire, so the ceiling was runs/writes. Every case below
    //                        that was written for T22 keeps driving this, unchanged
    //                        and unweakened. They are not dead: they are the model
    //                        of the regime the archived T22/T33/T39 measurements
    //                        were taken in, and `replaceLatestObservableX1000`
    //                        reports that same number live so the two eras stay
    //                        comparable. A migration's evidence cannot outlive what
    //                        it migrates from while the comparison is still made.
    //   kFlushStage          the SHIPPED path — bare C1 publishes the whole staged
    //                        burst, so the ceiling is Sum min(run, 8)/writes. The
    //                        cases at the bottom of this file drive it, and they are
    //                        what item 34's `observableX1000 >= 990` gate is read
    //                        against.
    //
    // The capacity is a NAMED TYPE, so a pre-T34 `RelayWriteProbe probe(4u)` is a
    // compile error rather than a silently reinterpreted window size.
    // -----------------------------------------------------------------------
    constexpr RelayStageCapacity kReplaceLatestStage{ 1u };

    // 8 mirrors `relayedInputRing::kMaxDepth`. Written as a literal because this
    // file is deliberately free of the codec header (the probe is STL-only, which
    // is what makes it testable without a simulatable); the production wiring in
    // SimulationManagerUImpl.h passes the real constant, and the ring suite pins
    // the constant's value.
    constexpr RelayStageCapacity kFlushStage{ 8u };
} // namespace

// ---------------------------------------------------------------------------
// RelayWriteProbe
// ---------------------------------------------------------------------------

TEST_CASE("RelayWriteProbe: one write per frame is no coalescing at all",
          "[Network][RelayWriteProbe]")
{
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/4u);
    RelayWriteWindowSummary summary;

    // Five writes on five consecutive frames. The fifth write is what CLOSES the
    // fourth run, so the window reports 4 runs / 4 writes — the fifth is the open
    // run and is deliberately not in it.
    bool closed = false;
    for (std::uint32_t i = 0u; i < 5u; ++i)
    {
        closed = probe.noteWrite(kOwnerA, 100u + i, 500u + i, summary);
    }

    REQUIRE(closed);
    REQUIRE(summary.ownerId == kOwnerA);
    REQUIRE(summary.runs == 4u);
    REQUIRE(summary.writes == 4u);
    // Capture ticks 500..503 are the four writes the four counted runs carried;
    // 504 opens the next window and is deliberately outside this span.
    REQUIRE(summary.firstCaptureTick == 500u);
    REQUIRE(summary.lastCaptureTick == 503u);
    REQUIRE(summary.captureSpan == 4u);
    REQUIRE(summary.receivedX1000 == 1000u);     // nothing lost upstream
    REQUIRE(summary.observableX1000 == 1000u);   // every write is observable
    REQUIRE(summary.deliverableX1000 == 1000u);
    REQUIRE(summary.p50 == 1u);
    REQUIRE(summary.p99 == 1u);
    REQUIRE(summary.maxRun == 1u);
    REQUIRE(summary.emptyFrames == 0u);
    REQUIRE(summary.nonConsecutiveWrites == 0u);
    REQUIRE(summary.missedCaptureTicks == 0u);
    REQUIRE(summary.discontinuities == 0u);
}

TEST_CASE("RelayWriteProbe: two writes in one frame make the first unobservable",
          "[Network][RelayWriteProbe]")
{
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary summary;

    // Frame 1: capture ticks 500 and 501 — 501 overwrites 500 in a depth-1 ring.
    // Frame 2: capture tick 502.
    // Frame 3: capture tick 503 (closes run 2, completing the window).
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 500u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 501u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 502u, summary));
    REQUIRE(probe.noteWrite(kOwnerA, 3u, 503u, summary));

    REQUIRE(summary.runs == 2u);
    REQUIRE(summary.writes == 3u);
    REQUIRE(summary.maxRun == 2u);
    // 2 observable out of 3 written = 666‰, and the one lost write is NOT an
    // upstream loss — every capture tick was received.
    REQUIRE(summary.captureSpan == 3u);          // 500..502
    REQUIRE(summary.receivedX1000 == 1000u);
    REQUIRE(summary.observableX1000 == 666u);
    REQUIRE(summary.deliverableX1000 == 666u);
    REQUIRE(summary.nonConsecutiveWrites == 0u);
    REQUIRE(summary.missedCaptureTicks == 0u);

    // [T34] At a stage capacity of 1 the two ceilings are the same number by
    // construction — pinned here so the historical field is anchored to the regime
    // it names rather than to whatever the live one happens to compute.
    REQUIRE(summary.replaceLatestObservableX1000 == summary.observableX1000);
    REQUIRE(summary.observableWrites == summary.runs);
}

TEST_CASE("RelayWriteProbe: the observable fraction reproduces the measured 59% baseline",
          "[Network][RelayWriteProbe]")
{
    // THE CASE THIS PROBE EXISTS FOR. §9.11 measured a delivered fraction of 593‰
    // (35.6 arrivals/s against 60 captures/s). If write coalescing is the whole
    // mechanism, the server's own write pattern must already impose that ceiling.
    // Here is a distribution that does: 10 runs carrying 17 writes.
    //
    //   run lengths: 1,1,1,1,1,1,2,2,3,4  ->  10 runs, 17 writes
    //   observable  = 10/17 = 588‰, i.e. within a whisker of the measured 593‰
    //
    // Built explicitly rather than randomly so the expected p50/p99/max are
    // hand-checkable: sorted, the 10 samples are 1,1,1,1,1,1,2,2,3,4.
    //   p50 nearest-rank = ceil(0.50 * 10) = rank 5  -> 1
    //   p99 nearest-rank = ceil(0.99 * 10) = rank 10 -> 4
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/10u);
    RelayWriteWindowSummary summary;

    const std::uint32_t runLengths[10] = { 1u, 1u, 1u, 1u, 1u, 1u, 2u, 2u, 3u, 4u };

    std::uint64_t frame       = 1000u;
    std::uint32_t captureTick = 5000u;
    bool          closed      = false;

    for (std::uint32_t run = 0u; run < 10u; ++run)
    {
        for (std::uint32_t w = 0u; w < runLengths[run]; ++w)
        {
            closed = probe.noteWrite(kOwnerA, frame, captureTick, summary);
            ++captureTick;
        }
        ++frame;
    }
    // One more write on a fresh frame to CLOSE the tenth run.
    closed = probe.noteWrite(kOwnerA, frame, captureTick, summary);

    REQUIRE(closed);
    REQUIRE(summary.runs == 10u);
    REQUIRE(summary.writes == 17u);
    REQUIRE(summary.captureSpan == 17u);
    REQUIRE(summary.observableX1000 == 588u);
    // ⭐ THE COMPARISON THE WHOLE TASK TURNS ON. `deliverableX1000` is what the
    // client's arrival rate is read against; 588 against the measured 593 would
    // say the server's own write pattern accounts for the entire loss.
    REQUIRE(summary.deliverableX1000 == 588u);
    REQUIRE(summary.p50 == 1u);
    REQUIRE(summary.p99 == 4u);
    REQUIRE(summary.maxRun == 4u);
    // Every capture tick was consecutive: this loss is ENTIRELY coalescing, and
    // the upstream channel reads clean. That separation is the finding.
    REQUIRE(summary.receivedX1000 == 1000u);
    REQUIRE(summary.nonConsecutiveWrites == 0u);
    REQUIRE(summary.missedCaptureTicks == 0u);
}

TEST_CASE("RelayWriteProbe: a skipped capture tick is upstream loss, not coalescing",
          "[Network][RelayWriteProbe]")
{
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/3u);
    RelayWriteWindowSummary summary;

    // One write per frame — zero coalescing — but capture ticks 700, 703, 704, 705.
    // The 3-tick jump means 701 and 702 never reached the server at all. Depth
    // cannot recover those; only the input redundancy bundle can.
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 700u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 703u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 3u, 704u, summary));
    REQUIRE(probe.noteWrite(kOwnerA, 4u, 705u, summary));

    REQUIRE(summary.runs == 3u);
    REQUIRE(summary.writes == 3u);
    REQUIRE(summary.observableX1000 == 1000u);   // nothing was coalesced away
    REQUIRE(summary.nonConsecutiveWrites == 1u);
    REQUIRE(summary.missedCaptureTicks == 2u);   // 701 and 702
    REQUIRE(summary.discontinuities == 0u);
    // 700..704 is a span of 5 carrying only 3 received capture ticks. So the
    // ceiling on delivery is 600‰ and NONE of that deficit is coalescing —
    // raising depth would not recover a single one of it. Reading the three
    // fractions as one number is exactly how that mistake gets made.
    REQUIRE(summary.captureSpan == 5u);
    REQUIRE(summary.receivedX1000 == 600u);
    REQUIRE(summary.deliverableX1000 == 600u);
}

TEST_CASE("RelayWriteProbe: coalescing and upstream loss multiply into the delivered fraction",
          "[Network][RelayWriteProbe]")
{
    // Both channels active at once, which is the case the log line has to keep
    // legible. Capture ticks 10, 11, 13, 14 (12 never arrived), with 10+11 sharing
    // a frame:
    //   frame 1: caps 10, 11   -> run of 2
    //   frame 2: cap  13       -> run of 1   (12 missing)
    //   frame 3: cap  14       -> closes run 2 and the window
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary summary;

    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 10u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 11u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 13u, summary));
    REQUIRE(probe.noteWrite(kOwnerA, 3u, 14u, summary));

    REQUIRE(summary.runs == 2u);
    REQUIRE(summary.writes == 3u);
    REQUIRE(summary.captureSpan == 4u);            // 10..13
    REQUIRE(summary.missedCaptureTicks == 1u);     // 12
    REQUIRE(summary.receivedX1000 == 750u);        // 3 of 4 reached the server
    REQUIRE(summary.observableX1000 == 666u);      // 2 of those 3 are pollable
    REQUIRE(summary.deliverableX1000 == 500u);     // 2 of 4 end to end
}

TEST_CASE("RelayWriteProbe: a capture-tick discontinuity restarts the window and says so",
          "[Network][RelayWriteProbe]")
{
    // A jump past kRelayWriteDiscontinuityTicks is a re-join or a tick domain
    // being re-established, not a coverage hole. Absorbing it would put a
    // four-digit `missedCaptureTicks` and a nonsense `captureSpan` into the
    // window; the fractions computed from that span would be garbage and would
    // look like data. So the window RESTARTS — and the restart is REPORTED, so a
    // reader knows to discard the shortened window rather than average it in.
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary summary;

    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 100u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 101u, summary));

    // +5000 capture ticks in one write.
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 3u, 5101u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 4u, 5102u, summary));
    REQUIRE(probe.noteWrite(kOwnerA, 5u, 5103u, summary));

    REQUIRE(summary.discontinuities == 1u);
    // Anchored at the discontinuity, not before it: 5101..5102, two clean writes.
    REQUIRE(summary.firstCaptureTick == 5101u);
    REQUIRE(summary.captureSpan == 2u);
    REQUIRE(summary.writes == 2u);
    REQUIRE(summary.missedCaptureTicks == 0u);
    REQUIRE(summary.receivedX1000 == 1000u);
}

TEST_CASE("RelayWriteProbe: frames with no write are counted, and are not runs",
          "[Network][RelayWriteProbe]")
{
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary summary;

    // Writes on frames 10, 14, 15. Frames 11/12/13 carried nothing for this owner.
    // Empty frames are the signature of CLUMPED packet arrival, which is a
    // different picture from steady one-per-frame delivery even at the same rate.
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 10u, 900u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 14u, 901u, summary));
    REQUIRE(probe.noteWrite(kOwnerA, 15u, 902u, summary));

    REQUIRE(summary.runs == 2u);
    REQUIRE(summary.writes == 2u);
    REQUIRE(summary.emptyFrames == 3u);   // 11, 12, 13
}

TEST_CASE("RelayWriteProbe: a run straddling the window edge is not truncated into it",
          "[Network][RelayWriteProbe]")
{
    // THE FAILURE MODE THIS CASE TARGETS. An implementation that closed the window
    // on write count would cut the 5-write run in half and report a max of 2 or 3.
    // The correct behaviour is that the long run belongs ENTIRELY to whichever
    // window observes its completion.
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary first;
    RelayWriteWindowSummary second;

    // Runs of 1, 1 (window 1 closes on the third frame's first write), then a run
    // of 5, then 1, then a closing write.
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 10u, first));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 11u, first));
    REQUIRE(probe.noteWrite(kOwnerA, 3u, 12u, first));      // closes runs {1,1}
    REQUIRE(first.runs == 2u);
    REQUIRE(first.writes == 2u);
    REQUIRE(first.maxRun == 1u);

    // Frame 3's run continues: 4 more writes, total length 5.
    for (std::uint32_t i = 0u; i < 4u; ++i)
    {
        REQUIRE_FALSE(probe.noteWrite(kOwnerA, 3u, 13u + i, second));
    }
    REQUIRE(probe.openRunLength(kOwnerA) == 5u);

    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 4u, 17u, second));   // closes the run of 5
    REQUIRE(probe.noteWrite(kOwnerA, 5u, 18u, second));         // closes the run of 1

    REQUIRE(second.runs == 2u);
    REQUIRE(second.writes == 6u);     // 5 + 1, whole, in ONE window
    REQUIRE(second.maxRun == 5u);
    REQUIRE(second.observableX1000 == 333u);
}

TEST_CASE("RelayWriteProbe: the capture-tick watermark survives a window reset",
          "[Network][RelayWriteProbe]")
{
    // A watermark reset at the window edge would report a spurious discontinuity or
    // a spurious gap once per 120 runs — the same order of magnitude as the effect
    // being measured, which is why this is a case and not a comment.
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary summary;

    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 300u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 301u, summary));
    REQUIRE(probe.noteWrite(kOwnerA, 3u, 302u, summary));
    REQUIRE(summary.missedCaptureTicks == 0u);

    // First writes of the NEXT window, consecutive with 302. If the watermark had
    // been reset it would look like a jump from 0. The run open at the window edge
    // is carried across too, so the second write here already closes the new
    // window's second run.
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 4u, 303u, summary));
    REQUIRE(probe.noteWrite(kOwnerA, 5u, 304u, summary));

    REQUIRE(summary.runs == 2u);
    REQUIRE(summary.writes == 2u);
    REQUIRE(summary.missedCaptureTicks == 0u);
    REQUIRE(summary.discontinuities == 0u);
}

TEST_CASE("RelayWriteProbe: owners keep separate windows and are dropped on unregister",
          "[Network][RelayWriteProbe]")
{
    // Two characters relayed on the same server share the game-thread frame
    // counter. Pooling them would report one character's coalescing as the other's
    // — and the whole comparison is against a PER-COMPONENT client measurement.
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary summary;

    // Owner A coalesces two-per-frame; owner B does not. Same frames.
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 10u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 11u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerB, 1u, 10u, summary));

    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 12u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 13u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerB, 2u, 11u, summary));

    RelayWriteWindowSummary aSummary;
    REQUIRE(probe.noteWrite(kOwnerA, 3u, 14u, aSummary));
    REQUIRE(aSummary.ownerId == kOwnerA);
    REQUIRE(aSummary.writes == 4u);
    REQUIRE(aSummary.runs == 2u);
    REQUIRE(aSummary.observableX1000 == 500u);

    RelayWriteWindowSummary bSummary;
    REQUIRE(probe.noteWrite(kOwnerB, 3u, 12u, bSummary));
    REQUIRE(bSummary.ownerId == kOwnerB);
    REQUIRE(bSummary.writes == 2u);
    REQUIRE(bSummary.runs == 2u);
    REQUIRE(bSummary.observableX1000 == 1000u);

    REQUIRE(probe.trackedOwnerCount() == 2u);
    probe.forgetOwner(kOwnerA);
    REQUIRE(probe.trackedOwnerCount() == 1u);
    REQUIRE_FALSE(probe.peekSummary(kOwnerA, summary));
    REQUIRE(probe.peekSummary(kOwnerB, summary));
}

TEST_CASE("RelayWriteProbe: a run longer than the tracked range saturates p99 but not max",
          "[Network][RelayWriteProbe]")
{
    RelayWriteProbe probe(kReplaceLatestStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary summary;

    // A 20-write run — beyond kRelayWriteMaxTrackedRun (16). The histogram
    // saturates; `maxRun` must still report the true 20, because the depth rule
    // reads the max and a silently clamped one would size depth short.
    for (std::uint32_t i = 0u; i < 20u; ++i)
    {
        REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 100u + i, summary));
    }
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 120u, summary));
    REQUIRE(probe.noteWrite(kOwnerA, 3u, 121u, summary));

    REQUIRE(summary.runs == 2u);
    REQUIRE(summary.writes == 21u);
    REQUIRE(summary.maxRun == 20u);
    REQUIRE(summary.p99Saturated);
    REQUIRE(summary.p99 == kRelayWriteMaxTrackedRun + 1u);
}

// ---------------------------------------------------------------------------
// ⭐ [og-netcode-v2-input-relay T34] THE FLUSH REGIME — the ceiling this task moved.
//
// Item 34's acceptance gate is `observableX1000 >= 990` on a live run. That gate is
// only meaningful if the metric moved with the mechanism: under the retired
// replace-latest path a writing frame published exactly one entry, so a burst of
// two reported 666 no matter how healthy the wire was, and the gate would have been
// unpassable by construction while the design worked perfectly. These cases pin the
// new arithmetic against the SAME hand-computed distributions the T22 cases above
// use, so the two regimes can be read side by side.
// ---------------------------------------------------------------------------

TEST_CASE("RelayWriteProbe: under flush-on-poll a two-write frame loses nothing",
          "[Network][RelayWriteProbe]")
{
    // Byte-for-byte the same drive as "two writes in one frame make the first
    // unobservable" above — the only difference is the publish step.
    RelayWriteProbe probe(kFlushStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary summary;

    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 500u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 501u, summary));
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 502u, summary));
    REQUIRE(probe.noteWrite(kOwnerA, 3u, 503u, summary));

    REQUIRE(summary.runs == 2u);
    REQUIRE(summary.writes == 3u);
    REQUIRE(summary.observableWrites == 3u);
    REQUIRE(summary.observableX1000 == 1000u);
    REQUIRE(summary.deliverableX1000 == 1000u);

    // ...and the historical field still reports what the retired path WOULD have
    // imposed on this same window, which is how a run reports the improvement as a
    // measurement rather than as a claim.
    REQUIRE(summary.replaceLatestObservableX1000 == 666u);
}

TEST_CASE("RelayWriteProbe: the flush ceiling clears item 34's gate on the measured baseline distribution",
          "[Network][RelayWriteProbe]")
{
    // The §9.11 baseline distribution again — 10 runs carrying 17 writes, run
    // lengths 1,1,1,1,1,1,2,2,3,4 — which the replace-latest ceiling scored at 588.
    // Every one of those runs fits a stage of 8, so all 17 writes are published.
    RelayWriteProbe probe(kFlushStage, /*windowRuns=*/10u);
    RelayWriteWindowSummary summary;

    const std::uint32_t runLengths[10] = { 1u, 1u, 1u, 1u, 1u, 1u, 2u, 2u, 3u, 4u };

    std::uint64_t frame       = 1000u;
    std::uint32_t captureTick = 5000u;
    bool          closed      = false;

    for (std::uint32_t run = 0u; run < 10u; ++run)
    {
        for (std::uint32_t w = 0u; w < runLengths[run]; ++w)
        {
            closed = probe.noteWrite(kOwnerA, frame, captureTick, summary);
            ++captureTick;
        }
        ++frame;
    }
    closed = probe.noteWrite(kOwnerA, frame, captureTick, summary);

    REQUIRE(closed);
    REQUIRE(summary.writes == 17u);
    REQUIRE(summary.observableWrites == 17u);
    REQUIRE(summary.observableX1000 == 1000u);
    REQUIRE(summary.observableX1000 >= 990u);            // item 34's pass condition
    REQUIRE(summary.replaceLatestObservableX1000 == 588u);
}

TEST_CASE("RelayWriteProbe: a burst longer than the stage still loses its oldest, and says so",
          "[Network][RelayWriteProbe]")
{
    // Flush-on-poll does NOT raise the ring's hard ceiling: kMaxDepth is a wire
    // bound, and a burst past it drops its oldest entries exactly as the ring would
    // have. This is the residual loss the ceiling has to keep reporting — a metric
    // that reads 1000 unconditionally would be a metric that cannot fail.
    RelayWriteProbe probe(kFlushStage, /*windowRuns=*/2u);
    RelayWriteWindowSummary summary;

    // Frame 1: a 12-write burst against a stage of 8 -> 4 lost.
    for (std::uint32_t i = 0u; i < 12u; ++i)
    {
        REQUIRE_FALSE(probe.noteWrite(kOwnerA, 1u, 100u + i, summary));
    }
    REQUIRE_FALSE(probe.noteWrite(kOwnerA, 2u, 112u, summary));   // closes the 12-run
    REQUIRE(probe.noteWrite(kOwnerA, 3u, 113u, summary));         // closes the 1-run

    REQUIRE(summary.runs == 2u);
    REQUIRE(summary.writes == 13u);
    REQUIRE(summary.observableWrites == 9u);       // min(12,8) + min(1,8)
    REQUIRE(summary.maxRun == 12u);
    REQUIRE(summary.observableX1000 == 692u);      // 9/13
    REQUIRE(summary.observableX1000 < 990u);       // the gate would FAIL here
    REQUIRE(summary.replaceLatestObservableX1000 == 153u);   // 2/13
}

// ---------------------------------------------------------------------------
// ConnectionBudgetProbe
// ---------------------------------------------------------------------------

TEST_CASE("ConnectionBudgetProbe: occupancy is measured against the negotiated net speed",
          "[Network][RelayWriteProbe]")
{
    // The shipped configuration, with the numbers the engine derivation predicts:
    //   CurrentNetSpeed  = 250000 B/s (MaxClientRate clamp)
    //   DesiredTickRate  = 60 Hz      (NetServerMaxTickRate on a dedicated server)
    //   allowance        = 4166 B/tick
    // and a modelled 3-character round of 1414 B/tick -> 33.9 % occupancy.
    ConnectionBudgetProbe probe(/*windowSamples=*/4u);
    ConnectionBudgetWindowSummary summary;

    std::uint64_t bytes   = 0u;
    std::uint64_t packets = 0u;
    std::uint64_t micros  = 0u;
    bool closed = false;

    for (std::uint32_t i = 0u; i < 5u; ++i)
    {
        closed = probe.noteSample(/*connectionId=*/7u,
                                  /*netSpeedBps=*/250000,
                                  /*queuedBits=*/-60000,
                                  bytes, packets, /*outTotalPacketsLost=*/0u,
                                  /*tickRateHz=*/60u, micros, summary);
        bytes   += 1414u;
        packets += 2u;
        micros  += 16667u;
    }

    REQUIRE(closed);
    REQUIRE(summary.connectionId == 7u);
    REQUIRE(summary.samples == 4u);
    REQUIRE(summary.netSpeedBps == 250000);
    REQUIRE(summary.allowanceBytesPerTick == 4166u);
    REQUIRE(summary.outBytes == 4u * 1414u);
    REQUIRE(summary.bytesPerSample == 1414u);
    REQUIRE(summary.bytesPerPacket == 707u);
    REQUIRE(summary.occupancyPctX10 == 339u);      // 33.9 %
    REQUIRE(summary.notReadySamples == 0u);
}

TEST_CASE("ConnectionBudgetProbe: QueuedBits is a debt counter, so min is the MOST headroom",
          "[Network][RelayWriteProbe]")
{
    ConnectionBudgetProbe probe(/*windowSamples=*/3u);
    ConnectionBudgetWindowSummary summary;

    const std::int32_t queued[4] = { -66000, -30000, 500, -30000 };

    bool closed = false;
    for (std::uint32_t i = 0u; i < 4u; ++i)
    {
        closed = probe.noteSample(/*connectionId=*/1u, /*netSpeedBps=*/250000,
                                  queued[i], /*outTotalBytes=*/0u,
                                  /*outTotalPackets=*/0u, /*outTotalPacketsLost=*/0u,
                                  /*tickRateHz=*/60u, /*nowMicros=*/i * 16667u,
                                  summary);
    }

    REQUIRE(closed);
    // The seeding sample is not measured, so the window covers queued[1..3].
    REQUIRE(summary.samples == 3u);
    REQUIRE(summary.queuedBitsMin == -30000);
    REQUIRE(summary.queuedBitsMax == 500);
    // One sample at or above zero: that is one frame on which IsNetReady() was
    // false and Iris wrote NOTHING. A non-zero count here is the send-path
    // deferral candidate's evidence.
    REQUIRE(summary.notReadySamples == 1u);
}

TEST_CASE("ConnectionBudgetProbe: outgoing packet loss is reported as an ack-derived delta",
          "[Network][RelayWriteProbe]")
{
    ConnectionBudgetProbe probe(/*windowSamples=*/2u);
    ConnectionBudgetWindowSummary summary;

    REQUIRE_FALSE(probe.noteSample(3u, 250000, -60000, 1000u, 10u, 5u, 60u, 0u, summary));
    REQUIRE_FALSE(probe.noteSample(3u, 250000, -60000, 2000u, 20u, 6u, 60u, 16667u, summary));
    REQUIRE(probe.noteSample(3u, 250000, -60000, 3000u, 30u, 8u, 60u, 33334u, summary));

    REQUIRE(summary.outBytes == 2000u);
    REQUIRE(summary.outPackets == 20u);
    REQUIRE(summary.outPacketsLost == 3u);   // 8 - 5, measured not configured
    REQUIRE(summary.elapsedMs == 33u);
}

TEST_CASE("ConnectionBudgetProbe: a reconnect re-anchors instead of reporting a huge delta",
          "[Network][RelayWriteProbe]")
{
    ConnectionBudgetProbe probe(/*windowSamples=*/2u);
    ConnectionBudgetWindowSummary summary;

    REQUIRE_FALSE(probe.noteSample(9u, 250000, -60000, 5000u, 50u, 0u, 60u, 0u, summary));
    // Counters go BACKWARDS — a new UNetConnection on the same id. Differencing
    // would underflow into a multi-gigabyte "delta" and poison the window.
    REQUIRE_FALSE(probe.noteSample(9u, 250000, -60000, 10u, 1u, 0u, 60u, 16667u, summary));
    REQUIRE_FALSE(probe.noteSample(9u, 250000, -60000, 110u, 2u, 0u, 60u, 33334u, summary));
    REQUIRE(probe.noteSample(9u, 250000, -60000, 210u, 3u, 0u, 60u, 50001u, summary));

    REQUIRE(summary.outBytes == 200u);
    REQUIRE(summary.outPackets == 2u);
}

#endif // WITH_LOW_LEVEL_TESTS
