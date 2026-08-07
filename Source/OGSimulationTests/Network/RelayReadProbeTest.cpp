// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/Network/RelayReadProbe.h"
#include "OGSimulation/Network/RelayedInputStore.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationNetSync.h"      // resolveScheduledRelayedInput

#include <cstdint>
#include <cstring>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T19: the CLIENT-side relay probes.
//
// WHAT THESE TESTS EXIST TO PIN, in order of how easy each is to get wrong:
//
//   1. FOUR OUTCOMES, NOT TWO. `resolveScheduledRelayedInput` serves `fallback()`
//      from three different situations that behave identically and mean completely
//      different things. VERIFY-FAIL (a candidate was there, stamped against a
//      delay that has since moved) is a delay TRANSITION; MISS (nothing resident)
//      is STARVATION; rung 0 is the JOIN WINDOW. An implementation that collapsed
//      them to hit/miss would pass every behavioural test in the tree, because the
//      returned INPUT is the same in all three — only the classification differs.
//      So these cases assert the outcome, not the value.
//
//   2. THE CADENCE GAP IS IN CAPTURE TICKS, NOT LOCAL TICKS. `depth` is
//      denominated in capture ticks, so a p99 measured in the receiver's local
//      ticks cannot legitimately be compared against it. The cases below are built
//      so their LOCAL spacing DIFFERS from their CAPTURE spacing in BOTH
//      directions — a local-tick implementation gets a different number and fails,
//      rather than passing because the two happened to coincide.
//
//   3. RUNG 0 IS EXCLUDED FROM THE STALE RUN. Rung 0 also serves `fallback()`, so
//      the naive "count every fallback" implementation is the one someone would
//      actually write. The stale-run case carries a LEADING rung-0 prefix that is
//      LONGER than the real runs that follow it, so the naive implementation
//      reports the prefix and fails; an implementation that counted rung-0 as a
//      break rather than an exclusion still passes, which is intentional — the
//      header documents the two as observationally identical and says why.
//
//   4. THE PROBES ARE PURE TELEMETRY. Nothing here feeds resolution: the
//      report-pointer overload of the ladder is asserted to return byte-identical
//      inputs to the no-report call on every rung.
//
// These run against the REAL production types — the real store, the real codec, the
// real `populateRelayedInputStore` and the real ladder. The only thing standing in
// for production is the byte buffer behind the ring (a std::vector rather than the
// USTRUCT's TArray), per the codec's BUFFER CONCEPT.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // Same load-bearing asymmetry RelayedInputStoreTest.cpp uses: `forward` is 0
    // when value-initialised and 1 in the "game zero", so an assertion can tell the
    // injected neutral from `InputT{}`.
    struct ProbeTestInput
    {
        std::int32_t forward = 0;
        std::int32_t tag     = 0;

        bool operator==(const ProbeTestInput& other) const
        {
            return forward == other.forward && tag == other.tag;
        }
    };
} // namespace

template <>
struct SerializableFields<ProbeTestInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(ProbeTestInput, forward),
                               SIM_MEMBER(ProbeTestInput, tag));
    }
};

namespace
{
    ProbeTestInput gameZero()
    {
        ProbeTestInput zero;
        zero.forward = 1;
        return zero;
    }

    ProbeTestInput tagged(std::int32_t tag)
    {
        ProbeTestInput input = gameZero();
        input.tag = tag;
        return input;
    }

    // UE-free byte buffer satisfying the codec's BUFFER CONCEPT.
    struct ProbeTestRing
    {
        std::vector<std::uint8_t> bytes;

        std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

        void bundleAddZeroedBytes(std::int32_t count)
        {
            bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u);
        }

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        {
            std::memcpy(bytes.data() + off, &value, sizeof(T));
        }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        {
            T value;
            std::memcpy(&value, bytes.data() + off, sizeof(T));
            return value;
        }
    };

    ProbeTestRing makeRing(const std::vector<std::uint32_t>& captureTicks,
                           std::uint8_t                      dA,
                           std::int32_t                      depth)
    {
        ProbeTestRing ring;
        for (std::uint32_t tick : captureTicks)
        {
            relayedInputRing::writeLatest<ProbeTestInput>(
                ring, tick, dA, tagged(static_cast<std::int32_t>(tick)), depth);
        }
        return ring;
    }

    // Runs the ladder BOTH ways and requires the returned input to be identical,
    // then hands back the classification. Every outcome case below goes through
    // this, so "the report pointer does not change the answer" is asserted on every
    // rung rather than once.
    ScheduledRelayedReadOutcome classify(const RelayedInputStore<ProbeTestInput>& store,
                                         std::uint32_t                            tick,
                                         ProbeTestInput&                          outInput)
    {
        const ProbeTestInput withoutReport = resolveScheduledRelayedInput(store, tick);

        ScheduledRelayedReadReport report;
        const ProbeTestInput withReport = resolveScheduledRelayedInput(store, tick, &report);

        REQUIRE(withReport == withoutReport);
        outInput = withReport;
        return report.outcome;
    }
} // namespace

// ---------------------------------------------------------------------------
// 1. PROBE 1 — the four outcomes of the scheduled read.
// ---------------------------------------------------------------------------

TEST_CASE("RelayProbe: rung 0 reports NoProbe, and it is NOT a miss",
          "[Network][RelayProbe]")
{
    const RelayedInputStore<ProbeTestInput> store(gameZero());

    ProbeTestInput served;
    REQUIRE(classify(store, 500u, served) == ScheduledRelayedReadOutcome::NoProbe);

    // Rung 0 answers the INJECTED game zero, never ProbeTestInput{} — the same
    // distinction T5's store tests pin, re-asserted here because the classification
    // must not have been bought by changing what is served.
    REQUIRE(served == gameZero());
    REQUIRE(served.forward == 1);
}

TEST_CASE("RelayProbe: a scheduled read whose stamp verifies reports Hit",
          "[Network][RelayProbe]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());

    // dLatest = 4, and the entry at tick 104-4 = 100 carries the SAME stamp.
    store.push(100u, 4u, tagged(100));
    store.push(104u, 4u, tagged(104));

    ProbeTestInput served;
    ScheduledRelayedReadReport report;
    REQUIRE(resolveScheduledRelayedInput(store, 104u, &report).tag == 100);
    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Hit);
    REQUIRE(report.probeTick == 100u);
    REQUIRE(report.dLatest == 4u);
    REQUIRE(report.candidateDA == 4u);

    REQUIRE(classify(store, 104u, served) == ScheduledRelayedReadOutcome::Hit);
    REQUIRE(served.tag == 100);
}

TEST_CASE("RelayProbe: a probe that finds nothing reports Miss (starvation)",
          "[Network][RelayProbe]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());
    store.push(104u, 4u, tagged(104));

    // Read tick 110 probes capture 106, which is not resident and does not alias
    // 104's slot (106 % 64 = 42, 104 % 64 = 40).
    ProbeTestInput served;
    ScheduledRelayedReadReport report;
    resolveScheduledRelayedInput(store, 110u, &report);
    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(report.probeTick == 106u);
    REQUIRE(report.dLatest == 4u);

    REQUIRE(classify(store, 110u, served) == ScheduledRelayedReadOutcome::Miss);
    // The MISS still serves last-known, which is why the outcome — not the value —
    // is the only thing that can distinguish this from the Hit above.
    REQUIRE(served.tag == 104);
}

TEST_CASE("RelayProbe: a candidate stamped against a STALE delay reports VerifyFail, not Miss",
          "[Network][RelayProbe]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());

    // THE DELAY REGIME SHIFTED. Capture 100 was relayed while the wire was held at
    // dA=2; by the time capture 104 arrived the server had moved the wire to dA=4.
    // The read at tick 104 therefore probes 100 — which IS resident — and finds a
    // stamp that no longer matches the current schedule.
    store.push(100u, 2u, tagged(100));
    store.push(104u, 4u, tagged(104));

    ScheduledRelayedReadReport report;
    const ProbeTestInput served = resolveScheduledRelayedInput(store, 104u, &report);

    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::VerifyFail);
    REQUIRE(report.probeTick == 100u);
    REQUIRE(report.dLatest == 4u);
    REQUIRE(report.candidateDA == 2u);          // the number that makes it diagnosable

    // A CANDIDATE WAS FOUND AND DELIBERATELY NOT USED — the fallback is served
    // instead, so the returned value is indistinguishable from the Miss case above.
    // This assertion pair is the whole reason the two outcomes must not be merged.
    REQUIRE(served.tag == 104);
    REQUIRE_FALSE(served.tag == 100);

    // And the contrast, on the SAME store: a read whose probe lands on the
    // consistently-stamped entry still hits. The classification is per read, not a
    // property the store acquires.
    ScheduledRelayedReadReport hitReport;
    resolveScheduledRelayedInput(store, 108u, &hitReport);
    REQUIRE(hitReport.outcome == ScheduledRelayedReadOutcome::Hit);
    REQUIRE(hitReport.probeTick == 104u);
}

TEST_CASE("RelayProbe: the tick < dA underflow guard reports Miss, not NoProbe",
          "[Network][RelayProbe]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());
    store.push(1u, 8u, tagged(1));

    // A session younger than the delay: no probe tick can be formed. Data HAS
    // arrived, so `findLatest().valid` is true — which is the literal condition the
    // stale-run rule is stated in terms of, and therefore the reason this is a Miss
    // rather than being folded into rung 0.
    ProbeTestInput served;
    REQUIRE(classify(store, 3u, served) == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(served.tag == 1);
    REQUIRE(store.findLatest().valid);
}

// ---------------------------------------------------------------------------
// 2. PROBE 1 — the two call sites are counted SEPARATELY.
// ---------------------------------------------------------------------------

TEST_CASE("RelayProbe: prediction and resim reads never share a counter",
          "[Network][RelayProbe]")
{
    RelayReadProbe probe;

    probe.notePredictionRead(1u, ScheduledRelayedReadOutcome::Hit);
    probe.notePredictionRead(1u, ScheduledRelayedReadOutcome::Hit);
    probe.notePredictionRead(1u, ScheduledRelayedReadOutcome::Miss);
    probe.notePredictionRead(1u, ScheduledRelayedReadOutcome::NoProbe);

    probe.noteResimRead(ScheduledRelayedReadOutcome::VerifyFail);
    probe.noteResimRead(ScheduledRelayedReadOutcome::Hit);

    REQUIRE(probe.predictionCounters().hit == 2u);
    REQUIRE(probe.predictionCounters().miss == 1u);
    REQUIRE(probe.predictionCounters().noProbe == 1u);
    REQUIRE(probe.predictionCounters().verifyFail == 0u);
    REQUIRE(probe.predictionCounters().total() == 4u);

    REQUIRE(probe.resimCounters().hit == 1u);
    REQUIRE(probe.resimCounters().verifyFail == 1u);
    REQUIRE(probe.resimCounters().miss == 0u);
    REQUIRE(probe.resimCounters().noProbe == 0u);
    REQUIRE(probe.resimCounters().total() == 2u);

    // Resim reads do not touch the stale run either — a run is a property of the
    // monotonic prediction stream.
    REQUIRE(probe.windowMaxFallbackRun() == 1u);        // the single prediction Miss
}

// ---------------------------------------------------------------------------
// 3. PROBE 3 — the D4 stale window, and the rung-0 exclusion.
// ---------------------------------------------------------------------------

TEST_CASE("RelayProbe: the max consecutive fallback run EXCLUDES a leading rung-0 prefix",
          "[Network][RelayProbe]")
{
    RelayReadProbe probe;

    using O = ScheduledRelayedReadOutcome;

    // THE SEQUENCE IS BUILT TO FAIL A NAIVE IMPLEMENTATION. The leading rung-0 run
    // is FIVE long and is immediately followed by two more fallback serves, so
    // "count every fallback" reports 7. Excluding rung 0 — and only rung 0 — the
    // true answer is the 3-long Miss/VerifyFail run later in the sequence.
    const O sequence[] = {
        O::NoProbe, O::NoProbe, O::NoProbe, O::NoProbe, O::NoProbe,   // join window
        O::Miss, O::Miss,                                             // run of 2
        O::Hit,
        O::Miss, O::VerifyFail, O::Miss,                              // run of 3
        O::Hit,
        O::Miss,                                                      // run of 1
        O::Hit,
    };

    for (O outcome : sequence)
    {
        probe.notePredictionRead(7u, outcome);
    }

    REQUIRE(probe.windowMaxFallbackRun() == 3u);
    // Explicitly NOT the naive answers: 7 (count everything) or 5 (the prefix).
    REQUIRE_FALSE(probe.windowMaxFallbackRun() == 7u);
    REQUIRE_FALSE(probe.windowMaxFallbackRun() == 5u);

    // The trailing Hit closed the last run.
    REQUIRE(probe.currentFallbackRun(7u) == 0u);

    // Rung 0 is still COUNTED as an outcome — it is excluded from the RUN, not
    // from the tally. Losing the tally would hide the join window entirely.
    REQUIRE(probe.predictionCounters().noProbe == 5u);
    REQUIRE(probe.predictionCounters().miss == 5u);
    REQUIRE(probe.predictionCounters().verifyFail == 1u);
    REQUIRE(probe.predictionCounters().hit == 3u);
}

TEST_CASE("RelayProbe: a strictly alternating hit/fallback stream has a max run of 1",
          "[Network][RelayProbe]")
{
    RelayReadProbe probe;

    for (int i = 0; i < 20; ++i)
    {
        probe.notePredictionRead(3u, (i % 2 == 0) ? ScheduledRelayedReadOutcome::Miss
                                                  : ScheduledRelayedReadOutcome::Hit);
    }

    REQUIRE(probe.windowMaxFallbackRun() == 1u);
    REQUIRE(probe.predictionCounters().miss == 10u);
    REQUIRE(probe.predictionCounters().hit == 10u);
}

TEST_CASE("RelayProbe: stale runs are per character and are dropped on unregister",
          "[Network][RelayProbe]")
{
    RelayReadProbe probe;

    // Two proxies interleaved. Character 1 is starving; character 2 is healthy.
    // A single global run counter would report 6 for the interleaving.
    for (int i = 0; i < 3; ++i)
    {
        probe.notePredictionRead(1u, ScheduledRelayedReadOutcome::Miss);
        probe.notePredictionRead(2u, ScheduledRelayedReadOutcome::Hit);
    }

    REQUIRE(probe.currentFallbackRun(1u) == 3u);
    REQUIRE(probe.currentFallbackRun(2u) == 0u);
    REQUIRE(probe.windowMaxFallbackRun() == 3u);
    REQUIRE(probe.trackedOwnerCount() == 2u);

    probe.forgetOwner(1u);
    REQUIRE(probe.currentFallbackRun(1u) == 0u);
    REQUIRE(probe.trackedOwnerCount() == 1u);
}

// ---------------------------------------------------------------------------
// 4. PROBE 1/3 — window mechanics.
// ---------------------------------------------------------------------------

TEST_CASE("RelayProbe: the read window closes on the PREDICTION tick and only when it carried reads",
          "[Network][RelayProbe]")
{
    RelayReadProbe probe(/*windowTicks=*/10u);
    RelayReadWindowSummary summary;

    REQUIRE_FALSE(probe.maybeCloseWindow(100u, summary));       // starts the window
    probe.notePredictionRead(1u, ScheduledRelayedReadOutcome::Hit);
    REQUIRE_FALSE(probe.maybeCloseWindow(105u, summary));       // still open

    REQUIRE(probe.maybeCloseWindow(110u, summary));
    REQUIRE(summary.prediction.hit == 1u);
    REQUIRE(summary.windowStartTick == 100u);
    REQUIRE(summary.windowEndTick == 110u);

    // Counters reset with the window.
    REQUIRE(probe.predictionCounters().total() == 0u);

    // AN EMPTY WINDOW IS SILENT — an authority allocates no relay stores and never
    // reaches either call site, so without this it would emit a Warning line every
    // two seconds forever with all-zero counts.
    REQUIRE_FALSE(probe.maybeCloseWindow(120u, summary));
}

TEST_CASE("RelayProbe: a backwards prediction tick restarts the window instead of hanging it open",
          "[Network][RelayProbe]")
{
    RelayReadProbe probe(/*windowTicks=*/10u);
    RelayReadWindowSummary summary;

    REQUIRE_FALSE(probe.maybeCloseWindow(4000u, summary));

    // A HARD RESYNC jumps the prediction clock backwards. An unsigned subtraction
    // would compute a ~4-billion-tick "elapsed" and close the window immediately,
    // every tick, for the rest of the session.
    REQUIRE_FALSE(probe.maybeCloseWindow(3000u, summary));
    probe.notePredictionRead(1u, ScheduledRelayedReadOutcome::Hit);
    REQUIRE_FALSE(probe.maybeCloseWindow(3005u, summary));
    REQUIRE(probe.maybeCloseWindow(3010u, summary));
    REQUIRE(summary.windowStartTick == 3000u);
}

// ---------------------------------------------------------------------------
// 5. PROBE 2 — the cadence gap is in CAPTURE TICKS.
// ---------------------------------------------------------------------------

TEST_CASE("RelayProbe: the arrival gap is measured in CAPTURE ticks, not local ticks (slower capture)",
          "[Network][RelayProbe]")
{
    RelayArrivalProbe probe(/*windowSamples=*/8u);
    RelayArrivalWindowSummary summary;

    // LOCAL SPACING 1, CAPTURE SPACING 3. `localTick` advances once per arrival —
    // it is exactly what a local-tick implementation would subtract — while the
    // sender's newest capture tick advances by 3. The two answers are 1 and 3, so
    // this case cannot be passed by both.
    std::uint32_t localTick   = 1000u;
    std::uint32_t captureTick = 500u;
    bool          closed      = false;
    for (int i = 0; i < 9; ++i)       // 1 seeding arrival + 8 samples
    {
        closed = probe.noteArrival(1u, captureTick, summary) || closed;
        ++localTick;
        captureTick += 3u;
    }

    REQUIRE(closed);
    REQUIRE(localTick == 1009u);        // the local spacing really was 1
    REQUIRE(summary.samples == 8u);
    REQUIRE(summary.p50 == 3u);
    REQUIRE(summary.p99 == 3u);
    REQUIRE(summary.maxGap == 3u);
    // THE ASSERTION THAT KILLS A LOCAL-TICK IMPLEMENTATION: it would report 1 here,
    // because the arrivals are one local tick apart.
    REQUIRE_FALSE(summary.p99 == 1u);

    // The window really did reset on close.
    RelayArrivalWindowSummary peek;
    probe.peekSummary(peek);
    REQUIRE(peek.samples == 0u);
}

TEST_CASE("RelayProbe: the arrival gap is measured in CAPTURE ticks, not local ticks (faster capture)",
          "[Network][RelayProbe]")
{
    RelayArrivalProbe probe(/*windowSamples=*/6u);
    RelayArrivalWindowSummary summary;

    // THE INVERSE, so neither direction can be passed by coincidence. Arrivals are
    // 4 LOCAL ticks apart (a laggy OnRep cadence) but each carries only ONE new
    // capture tick, because the sender is producing captures no faster than that.
    // A local-tick implementation reports 4; the capture-tick answer is 1.
    std::uint32_t localTick   = 0u;
    std::uint32_t captureTick = 900u;
    bool closed = false;
    for (int i = 0; i < 7; ++i)
    {
        closed = probe.noteArrival(2u, captureTick, summary) || closed;
        localTick += 4u;
        captureTick += 1u;
    }

    REQUIRE(closed);
    REQUIRE(localTick == 28u);      // the local spacing really was 4
    REQUIRE(summary.samples == 6u);
    REQUIRE(summary.p50 == 1u);
    REQUIRE(summary.p99 == 1u);
    REQUIRE(summary.maxGap == 1u);
    REQUIRE_FALSE(summary.p99 == 4u);
}

TEST_CASE("RelayProbe: the cadence p99 reports the TAIL a mean would hide",
          "[Network][RelayProbe]")
{
    RelayArrivalProbe probe(/*windowSamples=*/100u);
    RelayArrivalWindowSummary summary;

    // A KNOWN DISTRIBUTION: 98 gaps of 1 and 2 gaps of 20. The mean is ~1.4 — which
    // would say "depth 2 is plenty" — while the nearest-rank p99 is 20, which is the
    // number `depth >= gap_p99 + margin` has to clear. This is the whole reason the
    // task asks for a percentile rather than an average.
    std::uint32_t captureTick = 10u;
    probe.noteArrival(5u, captureTick, summary);        // seed, not a sample

    bool closed = false;
    for (int i = 0; i < 100; ++i)
    {
        // Put the two long gaps in the middle, so a "last sample wins" bug does not
        // pass by accident.
        const std::uint32_t gap = (i == 40 || i == 41) ? 20u : 1u;
        captureTick += gap;
        closed = probe.noteArrival(5u, captureTick, summary) || closed;
    }

    REQUIRE(closed);
    REQUIRE(summary.samples == 100u);
    REQUIRE(summary.p50 == 1u);
    REQUIRE(summary.p99 == 20u);
    REQUIRE(summary.maxGap == 20u);
    REQUIRE_FALSE(summary.p99Saturated);
    REQUIRE(summary.saturatedSamples == 0u);
}

TEST_CASE("RelayProbe: a gap beyond the tracked range saturates the histogram but not the max",
          "[Network][RelayProbe]")
{
    RelayArrivalProbe probe(/*windowSamples=*/4u);
    RelayArrivalWindowSummary summary;

    std::uint32_t captureTick = 1u;
    probe.noteArrival(1u, captureTick, summary);        // seed

    bool closed = false;
    const std::uint32_t gaps[] = { 1u, 1u, 1u, 500u };
    for (std::uint32_t gap : gaps)
    {
        captureTick += gap;
        closed = probe.noteArrival(1u, captureTick, summary) || closed;
    }

    REQUIRE(closed);
    REQUIRE(summary.samples == 4u);
    REQUIRE(summary.saturatedSamples == 1u);
    REQUIRE(summary.p99 == kRelayArrivalMaxTrackedGap + 1u);
    REQUIRE(summary.p99Saturated);
    // THE EXACT NUMBER SURVIVES. A saturated percentile is always accompanied by a
    // real max, so an operator is never left with only ">= 64".
    REQUIRE(summary.maxGap == 500u);
}

TEST_CASE("RelayProbe: arrivals that advance no capture tick are not cadence samples",
          "[Network][RelayProbe]")
{
    RelayArrivalProbe probe(/*windowSamples=*/2u);
    RelayArrivalWindowSummary summary;

    probe.noteArrival(1u, 300u, summary);       // seed
    // A dA RE-STAMP of an already-resident tick: the ring changed, so OnRep fired,
    // but no new capture tick was delivered. Counting it as a gap of 0 would drag
    // every percentile down and understate the depth requirement.
    REQUIRE_FALSE(probe.noteArrival(1u, 300u, summary));
    REQUIRE_FALSE(probe.noteArrival(1u, 300u, summary));
    REQUIRE(probe.sampleCount() == 0u);
    REQUIRE(probe.noAdvance() == 2u);

    // A BACKWARDS newest tick is treated the same way rather than producing a
    // nonsense negative gap, and it must not rewind the watermark either.
    REQUIRE_FALSE(probe.noteArrival(1u, 290u, summary));
    REQUIRE(probe.sampleCount() == 0u);
    REQUIRE(probe.noAdvance() == 3u);

    probe.noteArrival(1u, 302u, summary);       // gap 2 from 300, not 12 from 290
    REQUIRE(probe.sampleCount() == 1u);
    REQUIRE(probe.maxGap() == 2u);
}

TEST_CASE("RelayProbe: capture-tick watermarks are per component",
          "[Network][RelayProbe]")
{
    RelayArrivalProbe probe(/*windowSamples=*/100u);
    RelayArrivalWindowSummary summary;

    // TWO SENDERS AT DIFFERENT CAPTURE-TICK OFFSETS — the couch-coop / late-join
    // shape. A single shared watermark would compute an enormous cross-component
    // gap on every alternation.
    probe.noteArrival(1u, 1000u, summary);
    probe.noteArrival(2u, 40u, summary);
    probe.noteArrival(1u, 1001u, summary);
    probe.noteArrival(2u, 41u, summary);

    REQUIRE(probe.sampleCount() == 2u);
    REQUIRE(probe.maxGap() == 1u);

    probe.forgetOwner(2u);
    // After the unregister, id 2's next arrival seeds afresh instead of measuring
    // against a watermark that belonged to a character that no longer exists.
    REQUIRE_FALSE(probe.noteArrival(2u, 9000u, summary));
    REQUIRE(probe.sampleCount() == 2u);
}

// ---------------------------------------------------------------------------
// 6. PROBE 2's DATA SOURCE — the ingest report, through the REAL codec.
// ---------------------------------------------------------------------------

TEST_CASE("RelayProbe: the ingest report carries the newest capture tick THIS ring held",
          "[Network][RelayProbe]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());
    const ProbeTestRing ring = makeRing({ 100u, 101u, 102u }, /*dA=*/3u, /*depth=*/3);

    const RelayedInputIngestReport report =
        populateRelayedInputStore<ProbeTestInput>(store, ring);

    REQUIRE(report.outcome == RelayedInputIngestOutcome::Consumed);
    REQUIRE(report.entriesIngested == 3u);
    REQUIRE(report.newestCaptureTickValid);
    REQUIRE(report.newestCaptureTick == 102u);

    // NOT `store.findLatest()`. A SECOND arrival that carried only already-resident
    // ticks reports what THAT RING held, while the store's own latest is unchanged
    // — the store accumulates, the ring does not. Reading the store here would make
    // a stalled sender look like a healthy one whose gap is always 0.
    const ProbeTestRing restamp = makeRing({ 101u }, /*dA=*/9u, /*depth=*/1);
    const RelayedInputIngestReport second =
        populateRelayedInputStore<ProbeTestInput>(store, restamp);

    REQUIRE(second.newestCaptureTickValid);
    REQUIRE(second.newestCaptureTick == 101u);
    REQUIRE(store.findLatest().captureTick == 102u);
}

TEST_CASE("RelayProbe: a ring that delivered nothing reports no newest capture tick",
          "[Network][RelayProbe]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());

    // NEVER WRITTEN — version byte 0. Nothing was read, so there is no newest tick,
    // and the flag (rather than a 0 sentinel) is what says so: 0 is a perfectly
    // ordinary capture tick.
    const ProbeTestRing empty;
    const RelayedInputIngestReport neverWritten =
        populateRelayedInputStore<ProbeTestInput>(store, empty);
    REQUIRE(neverWritten.outcome == RelayedInputIngestOutcome::NeverWritten);
    REQUIRE_FALSE(neverWritten.newestCaptureTickValid);

    // VERSION MISMATCH — the whole ring is dropped, so nothing structural is read
    // and the cadence probe must get no sample from it.
    ProbeTestRing bad = makeRing({ 200u }, /*dA=*/1u, /*depth=*/1);
    bad.bytes[relayedInputRing::kVersionOffset] =
        static_cast<std::uint8_t>(relayedInputRing::kWireFormatVersion + 1u);
    const RelayedInputIngestReport mismatch =
        populateRelayedInputStore<ProbeTestInput>(store, bad);
    REQUIRE(mismatch.outcome == RelayedInputIngestOutcome::VersionMismatch);
    REQUIRE_FALSE(mismatch.newestCaptureTickValid);
}

TEST_CASE("RelayProbe: the ingest report drives the cadence probe end to end",
          "[Network][RelayProbe]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());
    RelayArrivalProbe                 probe(/*windowSamples=*/3u);
    RelayArrivalWindowSummary         summary;

    // THE WHOLE CHAIN, as the OnRep callback runs it: ring -> real ingest -> report
    // -> probe. The capture ticks advance by 5 per arrival while the arrivals
    // themselves are consecutive, so once more the capture answer (5) and any
    // local-tick answer (1) differ.
    bool closed = false;
    std::uint32_t captureTick = 700u;
    for (int i = 0; i < 4; ++i)
    {
        const ProbeTestRing ring = makeRing({ captureTick }, /*dA=*/2u, /*depth=*/1);
        const RelayedInputIngestReport report =
            populateRelayedInputStore<ProbeTestInput>(store, ring);

        REQUIRE(report.newestCaptureTickValid);
        closed = probe.noteArrival(11u, report.newestCaptureTick, summary) || closed;
        captureTick += 5u;
    }

    REQUIRE(closed);
    REQUIRE(summary.samples == 3u);
    REQUIRE(summary.p50 == 5u);
    REQUIRE(summary.p99 == 5u);
    REQUIRE(summary.maxGap == 5u);
}

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T20 — WHY the miss missed, and what the SERVER's
// frame cadence was while it happened.
//
// WHAT THESE PIN, and why each one is the one that would otherwise be got wrong:
//
//   1. SIX CLASSES, EACH CONSTRUCTED SEPARATELY. rung0 / hit / verifyFail are T19's
//      and are re-asserted here only where T20 touches them. The three new ones —
//      missInSpan, missAboveNewest, missBelowOldest — are the whole point: they have
//      DIFFERENT REMEDIES (raise the depth / depth is irrelevant / clock or
//      capacity), so an implementation that lumped any two of them together would
//      send the next task after the wrong fix. Every case asserts the CLASS, and the
//      served VALUE is identical in all three, so the class is the only thing that
//      can distinguish them.
//
//   2. THE SPAN IS THE STORE'S, NOT THE RING'S. This is the single easiest thing to
//      get wrong and it would invalidate the whole probe: at depth 1 a ring carries
//      one entry, so a ring-span implementation would classify essentially every
//      miss as out-of-span and `missInSpan` would read 0 forever — which is exactly
//      the answer that would refute the depth hypothesis. The case below drives TWO
//      depth-1 arrivals through the REAL codec and the REAL ingest, then asserts the
//      hole BETWEEN them classifies as in-span.
//
//   3. THE RATIO IS HOOK-INDEPENDENT. ServerFrameProbe is fed a GLOBAL frame counter
//      precisely so that a hook firing twice per frame, or skipping frames, still
//      yields the right ticks-per-frame. Two cases drive exactly those cadences and
//      assert the aggregate is unchanged while the cadence counters report the
//      difference.
//
//   4. SUB-STEPPING AND FRAME SHORTFALL ARE NOT THE SAME DEFECT. Both produce a
//      ratio of 2. The pair of cases below produce the SAME ratio from the two
//      different causes and assert that `numStepsAboveOne` separates them — if it
//      did not, the live run would report "ticks-per-frame = 2" and the reader would
//      have no way to tell which fix it implied.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // Classifies a read and hands back the whole report, having first asserted that
    // asking for the report did not change the answer. Same contract as `classify`
    // above, widened to the T20 fields.
    ScheduledRelayedReadReport classifyFully(const RelayedInputStore<ProbeTestInput>& store,
                                             std::uint32_t                            tick,
                                             ProbeTestInput&                          outInput)
    {
        const ProbeTestInput withoutReport = resolveScheduledRelayedInput(store, tick);

        ScheduledRelayedReadReport report;
        const ProbeTestInput withReport = resolveScheduledRelayedInput(store, tick, &report);

        REQUIRE(withReport == withoutReport);
        outInput = withReport;
        return report;
    }
} // namespace

// ---------------------------------------------------------------------------
// 7. THE RESIDENT SPAN — the store surface the classification is built on.
// ---------------------------------------------------------------------------

TEST_CASE("RelayMissClass: an empty store has no resident span",
          "[Network][RelayMissClass]")
{
    const RelayedInputStore<ProbeTestInput> store(gameZero());

    const auto span = store.residentSpan();
    REQUIRE_FALSE(span.valid);
    REQUIRE(span.count == 0u);
}

TEST_CASE("RelayMissClass: the resident span reports oldest, newest and count",
          "[Network][RelayMissClass]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());
    store.push(300u, 2u, tagged(300));
    store.push(305u, 2u, tagged(305));
    store.push(301u, 2u, tagged(301));

    const auto span = store.residentSpan();
    REQUIRE(span.valid);
    REQUIRE(span.oldest == 300u);
    REQUIRE(span.newest == 305u);
    REQUIRE(span.count == 3u);

    // `newest` is the same derivation findLatest() performs — asserted rather than
    // assumed, because the ladder reads the delta from one and the class from the
    // other and a divergence there would be invisible.
    REQUIRE(store.findLatest().captureTick == span.newest);

    // A single entry is a degenerate span, not an invalid one.
    RelayedInputStore<ProbeTestInput> one(gameZero());
    one.push(42u, 0u, tagged(42));
    const auto single = one.residentSpan();
    REQUIRE(single.valid);
    REQUIRE(single.oldest == 42u);
    REQUIRE(single.newest == 42u);
    REQUIRE(single.count == 1u);
}

// ---------------------------------------------------------------------------
// 8. THE SIX CLASSES, one constructed scenario each.
// ---------------------------------------------------------------------------

TEST_CASE("RelayMissClass: rung 0 and hit and verify-fail are NOT misses",
          "[Network][RelayMissClass]")
{
    ProbeTestInput served;

    // CLASS 1 — rung 0. No probe, no span, no delta.
    const RelayedInputStore<ProbeTestInput> empty(gameZero());
    const ScheduledRelayedReadReport rung0 = classifyFully(empty, 500u, served);
    REQUIRE(rung0.outcome == ScheduledRelayedReadOutcome::NoProbe);
    REQUIRE(rung0.missClass == ScheduledRelayedReadMissClass::NotAMiss);
    REQUIRE_FALSE(rung0.deltaToNewestValid);

    RelayedInputStore<ProbeTestInput> store(gameZero());
    store.push(100u, 4u, tagged(100));
    store.push(104u, 4u, tagged(104));

    // CLASS 2 — hit. The delta IS set here, deliberately: the hit deltas are the
    // calibration the miss deltas are read against. Probe 100, newest 104 => -4.
    const ScheduledRelayedReadReport hit = classifyFully(store, 104u, served);
    REQUIRE(hit.outcome == ScheduledRelayedReadOutcome::Hit);
    REQUIRE(hit.missClass == ScheduledRelayedReadMissClass::NotAMiss);
    REQUIRE(hit.deltaToNewestValid);
    REQUIRE(hit.deltaToNewest == -4);

    // CLASS 3 — verify-fail. Capture 200 was relayed while the wire was held at
    // dA=2; by the time 204 arrived the server had moved to dA=4. The read at 204
    // therefore probes 200 — RESIDENT, but stamped against a schedule that has moved.
    RelayedInputStore<ProbeTestInput> shifted(gameZero());
    shifted.push(200u, 2u, tagged(200));
    shifted.push(204u, 4u, tagged(204));
    const ScheduledRelayedReadReport verify = classifyFully(shifted, 204u, served);
    REQUIRE(verify.outcome == ScheduledRelayedReadOutcome::VerifyFail);
    REQUIRE(verify.missClass == ScheduledRelayedReadMissClass::NotAMiss);
    REQUIRE(verify.deltaToNewestValid);
    REQUIRE(verify.deltaToNewest == -4);        // probed 200, newest 204
}

TEST_CASE("RelayMissClass: a hole INSIDE the resident span is missInSpan",
          "[Network][RelayMissClass]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());

    // THE COVERAGE HOLE. 100 and 104 arrived; 102 never did. 102 % 64 = 38, and
    // neither 100 % 64 = 36 nor 104 % 64 = 40 aliases it, so this is a genuine
    // absence rather than a slot collision.
    store.push(100u, 4u, tagged(100));
    store.push(104u, 4u, tagged(104));

    ProbeTestInput served;
    const ScheduledRelayedReadReport report = classifyFully(store, 106u, served);

    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(report.probeTick == 102u);
    REQUIRE(report.missClass == ScheduledRelayedReadMissClass::InSpan);
    REQUIRE(report.spanValid);
    REQUIRE(report.oldestResident == 100u);
    REQUIRE(report.newestResident == 104u);
    REQUIRE(report.residentCount == 2u);
    REQUIRE(report.deltaToNewest == -2);

    // THE REMEDY THIS CLASS IMPLIES IS THE OPPOSITE OF THE NEXT ONE'S, which is the
    // whole reason the two must not be merged: the requested tick is one the sender
    // already produced and the receiver is asking INSIDE what it has been sent, so
    // more ring depth would have delivered it.
    REQUIRE_FALSE(report.missClass == ScheduledRelayedReadMissClass::AboveNewest);
    REQUIRE(served.tag == 104);                 // still serves last-known
}

TEST_CASE("RelayMissClass: asking newer than anything resident is missAboveNewest",
          "[Network][RelayMissClass]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());
    store.push(100u, 4u, tagged(100));
    store.push(104u, 4u, tagged(104));

    // Read tick 110 probes 106, which is NEWER than the newest arrival. No ring
    // depth delivers a capture the sender has not produced, so this class is the one
    // the depth hypothesis cannot explain and must not be allowed to claim.
    ProbeTestInput served;
    const ScheduledRelayedReadReport report = classifyFully(store, 110u, served);

    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(report.probeTick == 106u);
    REQUIRE(report.missClass == ScheduledRelayedReadMissClass::AboveNewest);
    REQUIRE(report.newestResident == 104u);
    REQUIRE(report.deltaToNewest == 2);         // POSITIVE — reading ahead of the data
    REQUIRE_FALSE(report.missClass == ScheduledRelayedReadMissClass::InSpan);
}

TEST_CASE("RelayMissClass: asking older than anything resident is missBelowOldest",
          "[Network][RelayMissClass]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());

    // The store holds 200 and 204 only; the read at 150 probes 146. 146 % 64 = 18,
    // 200 % 64 = 8, 204 % 64 = 12 — no aliasing, so the absence is real.
    store.push(200u, 4u, tagged(200));
    store.push(204u, 4u, tagged(204));

    ProbeTestInput served;
    const ScheduledRelayedReadReport report = classifyFully(store, 150u, served);

    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(report.probeTick == 146u);
    REQUIRE(report.missClass == ScheduledRelayedReadMissClass::BelowOldest);
    REQUIRE(report.oldestResident == 200u);
    REQUIRE(report.deltaToNewest == -58);
    REQUIRE_FALSE(report.missClass == ScheduledRelayedReadMissClass::InSpan);
}

TEST_CASE("RelayMissClass: the tick < dA underflow guard is its OWN class, not belowOldest",
          "[Network][RelayMissClass]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());
    store.push(1u, 8u, tagged(1));

    // T19 pins this as a Miss rather than rung 0 and that is unchanged. T20 adds the
    // reason: no probe tick could be FORMED, so there is nothing to compare against a
    // span and no delta to report. Folding it into BelowOldest — which it resembles,
    // since both mean "asking too early" — would put an early-session artefact in the
    // bucket whose remedy is a clock investigation.
    ProbeTestInput served;
    const ScheduledRelayedReadReport report = classifyFully(store, 3u, served);

    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(report.missClass == ScheduledRelayedReadMissClass::NoProbeTick);
    REQUIRE_FALSE(report.missClass == ScheduledRelayedReadMissClass::BelowOldest);
    REQUIRE_FALSE(report.deltaToNewestValid);
    REQUIRE_FALSE(report.spanValid);
}

// ---------------------------------------------------------------------------
// 9. THE TRAP: the span is the STORE'S, not the RING'S — through the real codec.
// ---------------------------------------------------------------------------

TEST_CASE("RelayMissClass: at DEPTH 1 a clobbered tick is still INSIDE the store's span",
          "[Network][RelayMissClass]")
{
    RelayedInputStore<ProbeTestInput> store(gameZero());

    // TWO ARRIVALS OF A DEPTH-1 RING, exactly as the shipped configuration produces
    // them: the server wrote capture 100, replicated, then wrote 101 and 102 before
    // the next replication — so 101 was clobbered and never transmitted, and the
    // client's store ends up holding 100 and 102 with a hole at 101.
    const ProbeTestRing first = makeRing({ 100u }, /*dA=*/2u, /*depth=*/1);
    REQUIRE(populateRelayedInputStore<ProbeTestInput>(store, first).entriesIngested == 1u);

    const ProbeTestRing second = makeRing({ 101u, 102u }, /*dA=*/2u, /*depth=*/1);
    REQUIRE(populateRelayedInputStore<ProbeTestInput>(store, second).entriesIngested == 1u);

    // THE RING CARRIED ONE ENTRY. THE STORE SPANS THREE TICKS. If the classification
    // read the RING's span it would see a single tick, every miss would land outside
    // it, and `missInSpan` would read zero for the entire session — which is exactly
    // the reading that would be taken as refuting the depth hypothesis.
    const auto span = store.residentSpan();
    REQUIRE(span.oldest == 100u);
    REQUIRE(span.newest == 102u);
    REQUIRE(span.count == 2u);              // 101 is the hole
    REQUIRE_FALSE(store.has(101u));

    ProbeTestInput served;
    const ScheduledRelayedReadReport report = classifyFully(store, 103u, served);

    REQUIRE(report.probeTick == 101u);
    REQUIRE(report.outcome == ScheduledRelayedReadOutcome::Miss);
    REQUIRE(report.missClass == ScheduledRelayedReadMissClass::InSpan);
    REQUIRE(report.deltaToNewest == -1);
}

// ---------------------------------------------------------------------------
// 10. COUNTING — the partition, and the two call sites.
// ---------------------------------------------------------------------------

TEST_CASE("RelayMissClass: the four miss sub-counters always partition `miss`",
          "[Network][RelayMissClass]")
{
    RelayReadProbe probe;

    RelayedInputStore<ProbeTestInput> store(gameZero());
    store.push(100u, 4u, tagged(100));
    store.push(104u, 4u, tagged(104));

    // One of each class that this store can produce, plus a hit, driven through the
    // REAL ladder so the counters cannot pass on a hand-written report.
    const std::uint32_t reads[] = { 104u /*hit*/, 106u /*inSpan*/, 110u /*aboveNewest*/ };
    for (std::uint32_t tick : reads)
    {
        ScheduledRelayedReadReport report;
        resolveScheduledRelayedInput(store, tick, &report);
        probe.notePredictionRead(7u, report);
    }

    // ...and one belowOldest and one noProbeTick, each from a store that can make it.
    RelayedInputStore<ProbeTestInput> far(gameZero());
    far.push(200u, 4u, tagged(200));
    ScheduledRelayedReadReport below;
    resolveScheduledRelayedInput(far, 150u, &below);
    probe.notePredictionRead(7u, below);

    RelayedInputStore<ProbeTestInput> young(gameZero());
    young.push(1u, 8u, tagged(1));
    ScheduledRelayedReadReport underflow;
    resolveScheduledRelayedInput(young, 3u, &underflow);
    probe.notePredictionRead(7u, underflow);

    const RelayReadCounters& counters = probe.predictionCounters();
    REQUIRE(counters.hit == 1u);
    REQUIRE(counters.miss == 4u);
    REQUIRE(counters.missInSpan == 1u);
    REQUIRE(counters.missAboveNewest == 1u);
    REQUIRE(counters.missBelowOldest == 1u);
    REQUIRE(counters.missNoProbeTick == 1u);

    // THE PARTITION PROPERTY. Every miss is classified, so an unclassified arm would
    // show up here as a shortfall rather than being silently absorbed.
    REQUIRE(counters.missClassTotal() == counters.miss);
}

TEST_CASE("RelayMissClass: prediction and resim keep SEPARATE miss classes and deltas",
          "[Network][RelayMissClass]")
{
    RelayReadProbe probe;

    RelayedInputStore<ProbeTestInput> store(gameZero());
    store.push(100u, 4u, tagged(100));
    store.push(104u, 4u, tagged(104));

    ScheduledRelayedReadReport inSpan;
    resolveScheduledRelayedInput(store, 106u, &inSpan);
    probe.notePredictionRead(3u, inSpan);

    ScheduledRelayedReadReport above;
    resolveScheduledRelayedInput(store, 110u, &above);
    probe.noteResimRead(above);

    // T19 requires the two call sites never to share a counter; T20's fields inherit
    // that and it is re-asserted rather than assumed, because a single shared
    // histogram would be the natural mistake when adding one.
    REQUIRE(probe.predictionCounters().missInSpan == 1u);
    REQUIRE(probe.predictionCounters().missAboveNewest == 0u);
    REQUIRE(probe.resimCounters().missAboveNewest == 1u);
    REQUIRE(probe.resimCounters().missInSpan == 0u);

    RelayDeltaSummary predictionDelta;
    probe.predictionCounters().delta.fillSummary(predictionDelta);
    RelayDeltaSummary resimDelta;
    probe.resimCounters().delta.fillSummary(resimDelta);

    REQUIRE(predictionDelta.samples == 1u);
    REQUIRE(predictionDelta.p50 == -2);
    REQUIRE(resimDelta.samples == 1u);
    REQUIRE(resimDelta.p50 == 2);
}

// ---------------------------------------------------------------------------
// 11. THE SIGNED-DELTA HISTOGRAM.
// ---------------------------------------------------------------------------

TEST_CASE("RelayDelta: the percentiles are nearest-rank over SIGNED values",
          "[Network][RelayMissClass]")
{
    RelaySignedDeltaHistogram histogram;

    // A deliberately asymmetric distribution straddling zero: 10 at -5, 70 at -1,
    // 20 at +3. p10 must land in the negative tail, p90 in the positive one — an
    // implementation that sorted by magnitude, or that used unsigned buckets, gets
    // both wrong.
    for (int i = 0; i < 10; ++i) { histogram.note(-5); }
    for (int i = 0; i < 70; ++i) { histogram.note(-1); }
    for (int i = 0; i < 20; ++i) { histogram.note(3); }

    RelayDeltaSummary summary;
    histogram.fillSummary(summary);

    REQUIRE(summary.samples == 100u);
    REQUIRE(summary.p10 == -5);
    REQUIRE(summary.p50 == -1);
    REQUIRE(summary.p90 == 3);
    REQUIRE(summary.minDelta == -5);
    REQUIRE(summary.maxDelta == 3);
    REQUIRE(summary.saturatedLow == 0u);
    REQUIRE(summary.saturatedHigh == 0u);
}

TEST_CASE("RelayDelta: values beyond the tracked range saturate but the extremes stay exact",
          "[Network][RelayMissClass]")
{
    RelaySignedDeltaHistogram histogram;

    // A fat tail at each end: 20 far below the tracked range, 60 at zero, 20 far
    // above. The tails are big enough that p10 and p90 must land IN them, which is
    // what makes this a test of the saturating buckets' ORDERING rather than of a
    // single outlier that any implementation would ignore.
    for (int i = 0; i < 20; ++i) { histogram.note(-4000); }
    for (int i = 0; i < 60; ++i) { histogram.note(0); }
    for (int i = 0; i < 20; ++i) { histogram.note(9000); }

    RelayDeltaSummary summary;
    histogram.fillSummary(summary);

    REQUIRE(summary.samples == 100u);
    REQUIRE(summary.saturatedLow == 20u);
    REQUIRE(summary.saturatedHigh == 20u);

    // A SATURATED PERCENTILE IS ALWAYS ACCOMPANIED BY A REAL NUMBER — the same
    // contract the arrival-gap histogram carries. The extremes are exact even though
    // the buckets are not, so a saturated p10 is never the only thing the reader has.
    REQUIRE(summary.minDelta == -4000);
    REQUIRE(summary.maxDelta == 9000);
    REQUIRE(summary.p50 == 0);

    // The saturating buckets sort OUTSIDE the exact range in the right DIRECTION, so
    // the ordering the percentile scan walks is monotonic in delta rather than in
    // magnitude. They report the first value beyond the range — a bound, never a
    // claim of precision the bucket does not have.
    REQUIRE(summary.p10 == -(kRelayDeltaHistogramRange + 1));
    REQUIRE(summary.p90 == kRelayDeltaHistogramRange + 1);

    // A SINGLE far outlier, by contrast, must NOT drag a percentile: one sample in
    // fifty is below the 10th percentile's rank, so p10 is the ordinary value.
    RelaySignedDeltaHistogram thinTail;
    for (int i = 0; i < 50; ++i) { thinTail.note(0); }
    thinTail.note(-4000);

    RelayDeltaSummary thin;
    thinTail.fillSummary(thin);
    REQUIRE(thin.p10 == 0);
    REQUIRE(thin.minDelta == -4000);
}

TEST_CASE("RelayDelta: an empty histogram reports nothing rather than zero samples of zero",
          "[Network][RelayMissClass]")
{
    RelaySignedDeltaHistogram histogram;

    RelayDeltaSummary summary;
    histogram.fillSummary(summary);
    REQUIRE(summary.samples == 0u);

    // The emitter is gated on `samples`, not on the percentile values, because 0 is a
    // perfectly ordinary delta and could not be used as an absent marker.
    REQUIRE(summary.p50 == 0);
}

// ---------------------------------------------------------------------------
// 12. PROBE A — server sim ticks per game-thread frame.
// ---------------------------------------------------------------------------

TEST_CASE("ServerFrameProbe: a once-per-frame hook reports the ratio directly",
          "[Network][ServerFrameProbe]")
{
    ServerFrameProbe probe(/*windowSamples=*/10u);
    ServerFrameWindowSummary summary;

    // 60 Hz sim on a 30 fps server: one frame, two sim ticks, every time.
    bool closed = false;
    std::uint64_t frame = 1000u;
    std::uint32_t tick  = 500u;
    std::uint64_t micros = 0u;
    for (int i = 0; i <= 10; ++i)
    {
        closed = probe.noteFrame(frame, tick, /*numSteps=*/2u, micros, summary) || closed;
        frame += 1u;
        tick  += 2u;
        micros += 33333u;                       // 33.3 ms == 30 fps
    }

    REQUIRE(closed);
    REQUIRE(summary.p50 == 2u);
    REQUIRE(summary.p99 == 2u);
    REQUIRE(summary.maxTicksPerFrame == 2u);
    REQUIRE(summary.meanTicksPerFrameX100 == 200u);
    REQUIRE(summary.oncePerFrameSamples == 10u);
    REQUIRE(summary.sameFrameSamples == 0u);
    REQUIRE(summary.skippedFrameSamples == 0u);
    REQUIRE(summary.meanFrameMicros == 33333u);
}

TEST_CASE("ServerFrameProbe: a hook firing TWICE per frame still reports the right ratio",
          "[Network][ServerFrameProbe]")
{
    ServerFrameProbe probe(/*windowSamples=*/20u);
    ServerFrameWindowSummary summary;

    // THE HOOK-INDEPENDENCE PROPERTY, and the reason the metric is defined against a
    // GLOBAL frame counter rather than an invocation count. This hook fires once per
    // SUB-STEP: two invocations per frame, one sim tick each. The true answer is
    // still two ticks per frame, and an implementation that counted invocations
    // would report one.
    bool closed = false;
    std::uint64_t frame = 10u;
    std::uint32_t tick  = 0u;
    for (int i = 0; i < 11; ++i)
    {
        closed = probe.noteFrame(frame, tick, 1u, 0u, summary) || closed;
        tick += 1u;
        closed = probe.noteFrame(frame, tick, 1u, 0u, summary) || closed;
        tick += 1u;
        frame += 1u;
    }

    REQUIRE(closed);
    REQUIRE(summary.meanTicksPerFrameX100 == 200u);
    REQUIRE_FALSE(summary.meanTicksPerFrameX100 == 100u);   // the invocation-count answer

    // AND IT SAYS SO OUT LOUD. Half the samples landed in the same frame as their
    // predecessor, which is the signature of a per-sub-step hook and is exactly the
    // exception the task requires to be reported rather than assumed away.
    REQUIRE(summary.sameFrameSamples > 0u);
}

TEST_CASE("ServerFrameProbe: a hook that SKIPS frames still reports the right ratio",
          "[Network][ServerFrameProbe]")
{
    ServerFrameProbe probe(/*windowSamples=*/10u);
    ServerFrameWindowSummary summary;

    // The mirror of the case above: this hook fires every THIRD frame, and three
    // frames advance three sim ticks. The true ratio is 1.
    bool closed = false;
    std::uint64_t frame = 0u;
    std::uint32_t tick  = 0u;
    for (int i = 0; i <= 10; ++i)
    {
        closed = probe.noteFrame(frame, tick, 1u, 0u, summary) || closed;
        frame += 3u;
        tick  += 3u;
    }

    REQUIRE(closed);
    REQUIRE(summary.meanTicksPerFrameX100 == 100u);
    REQUIRE_FALSE(summary.meanTicksPerFrameX100 == 300u);   // the per-invocation answer
    REQUIRE(summary.skippedFrameSamples == 10u);

    // No sample had exactly one frame between it and its predecessor, so the
    // histogram is empty and the percentiles must not invent a number.
    REQUIRE(summary.samples == 0u);
    REQUIRE(summary.p50 == 0u);
}

TEST_CASE("ServerFrameProbe: SUB-STEPPING and FRAME SHORTFALL give the same ratio and are still told apart",
          "[Network][ServerFrameProbe]")
{
    // TWO DIFFERENT DEFECTS, ONE IDENTICAL RATIO. Chaos running two fixed sub-steps
    // inside a healthy 60 fps frame, and a 30 fps frame running one step that covers
    // two ticks, both read as "two sim ticks per frame". They need different fixes,
    // so a probe that reported only the ratio would send the reader the wrong way
    // half the time.
    ServerFrameProbe substepping(/*windowSamples=*/5u);
    ServerFrameWindowSummary substepSummary;
    ServerFrameProbe shortfall(/*windowSamples=*/5u);
    ServerFrameWindowSummary shortfallSummary;

    std::uint64_t frame = 1u;
    std::uint32_t tick  = 0u;
    bool substepClosed = false;
    bool shortfallClosed = false;
    for (int i = 0; i <= 5; ++i)
    {
        substepClosed = substepping.noteFrame(frame, tick, /*numSteps=*/2u, 0u,
                                              substepSummary) || substepClosed;
        shortfallClosed = shortfall.noteFrame(frame, tick, /*numSteps=*/1u, 0u,
                                              shortfallSummary) || shortfallClosed;
        frame += 1u;
        tick  += 2u;
    }

    REQUIRE(substepClosed);
    REQUIRE(shortfallClosed);

    // Identical ratios...
    REQUIRE(substepSummary.p50 == shortfallSummary.p50);
    REQUIRE(substepSummary.meanTicksPerFrameX100 == shortfallSummary.meanTicksPerFrameX100);
    REQUIRE(substepSummary.p50 == 2u);

    // ...and the sub-step count is the only thing that separates them.
    REQUIRE(substepSummary.numStepsAboveOne == 6u);
    REQUIRE(substepSummary.maxNumSteps == 2u);
    REQUIRE(shortfallSummary.numStepsAboveOne == 0u);
    REQUIRE(shortfallSummary.maxNumSteps == 1u);
}

TEST_CASE("ServerFrameProbe: the p99 reports the hitch a mean would hide",
          "[Network][ServerFrameProbe]")
{
    ServerFrameProbe probe(/*windowSamples=*/100u);
    ServerFrameWindowSummary summary;

    // 98 healthy frames and two 12-tick hitches placed mid-sequence, not at the
    // edges. The mean is near 1.2 and the depth rule cares about the 12.
    std::uint64_t frame = 5u;
    std::uint32_t tick  = 0u;
    bool closed = false;
    for (int i = 0; i <= 100; ++i)
    {
        const std::uint32_t advance = (i == 40 || i == 70) ? 12u : 1u;
        closed = probe.noteFrame(frame, tick, 1u, 0u, summary) || closed;
        frame += 1u;
        tick  += advance;
    }

    REQUIRE(closed);
    REQUIRE(summary.p50 == 1u);
    REQUIRE(summary.p99 == 12u);
    REQUIRE(summary.maxTicksPerFrame == 12u);
    REQUIRE_FALSE(summary.p99 == 1u);
}

TEST_CASE("ServerFrameProbe: a discontinuity is re-seeded, counted, and never sampled",
          "[Network][ServerFrameProbe]")
{
    ServerFrameProbe probe(/*windowSamples=*/6u);
    ServerFrameWindowSummary summary;

    std::uint64_t frame = 100u;
    std::uint32_t tick  = 1000u;
    bool closed = false;
    for (int i = 0; i < 4; ++i)
    {
        closed = probe.noteFrame(frame, tick, 1u, 0u, summary) || closed;
        frame += 1u;
        tick  += 1u;
    }

    // A 50000-tick jump: the mapper offset being established, or a level transition.
    // Sampling it would put a five-digit outlier in `max` and hide every real number
    // behind it for the rest of the window.
    tick += 50000u;
    frame += 1u;
    closed = probe.noteFrame(frame, tick, 1u, 0u, summary) || closed;
    REQUIRE(probe.discontinuities() == 1u);

    for (int i = 0; i < 8; ++i)
    {
        frame += 1u;
        tick  += 1u;
        closed = probe.noteFrame(frame, tick, 1u, 0u, summary) || closed;
    }

    REQUIRE(closed);
    REQUIRE(summary.maxTicksPerFrame == 1u);
    REQUIRE_FALSE(summary.maxTicksPerFrame == 50000u);
    REQUIRE(summary.p50 == 1u);

    // A BACKWARDS tick is the same situation and takes the same route.
    ServerFrameProbe backwards(/*windowSamples=*/4u);
    ServerFrameWindowSummary backwardsSummary;
    backwards.noteFrame(10u, 900u, 1u, 0u, backwardsSummary);
    backwards.noteFrame(11u, 400u, 1u, 0u, backwardsSummary);
    REQUIRE(backwards.discontinuities() == 1u);
    REQUIRE(backwards.ratioSampleCount() == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS
