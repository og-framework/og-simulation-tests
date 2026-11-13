<!-- SPDX-License-Identifier: MPL-2.0 -->
# `InputArrivalMarginTest.cpp` — rationale

Source: `InputArrivalMarginTest.cpp`, tags `[Network][InputArrivalMargin]`. It tests og-simulation's
`InputArrivalMargin` through the real `ServerReceptionCoordinator`, fed by the seeded `LinkModel`
(`docs/LinkModel-rationale.md`). The margin is `captureTick + D − nextUndrainedTick`, recorded when the
server parks a new input (og-simulation `docs/InputArrivalMargin-rationale.md`).

This file has no guards document: it states no prohibition.

---

## §1 The rig

`Rig` holds one coordinator over `FStandaloneTestHandle` and `MockSimA`, one live wire, and a logger
that keeps every line. By default the configuration sets `relayDelayFloorTicks = 6`; `Rig(config)`
takes another (case (d) of §6). No RTT sample is ever fed,
so every connection takes the no-tier fallback, floored to 6: `D = 6` for every capture. The exact
case checks that with `effectiveDelay(wire) == kDelayTicks` before it trusts any number.

`Rig::run` plays one client against the server, in server ticks `t = 0 … lastServerTick`. Its other
overload plays several owners (`SenderPlan`: an id, a slot, a scenario each), each on its own uplink,
into the same wire; the frames are the same.

1. From `firstSendTick` on, the client captures tick `t + leadTicks` and sends, at time `t`, a
   payload carrying the last `redundancyDepth` capture ticks (oldest first) on the uplink.
2. On every `numSteps`-th tick the server runs a frame, in a host's order: the uplink delivers what
   is due and each capture goes to `receiveRemoteInput`; then `releaseDelayedInputs(t, numSteps)`;
   then `reapConnections(t)`.

The rig records each capture tick the first time it arrives (`arrivedCaptures`) and the frame it
arrived in (`arrivalFrames`), from the link's deliveries alone, so a case can derive the margins it
expects without reading the coordinator. It also records every copy for the first-sight oracle (§6),
and after each frame the window's `ArrivalMarginSignal` when a new one closed (`signals`).

A scenario's `extraDelayAt(sendTick)`, empty by default, adds ticks of path delay to each send (it shifts
the send time handed to the link). The P3-6 cases use it for a slow path swing and for a delay spike (§8).
Empty, it adds exactly 0.0, so no earlier case moved.

So inputs arrive between frames, the drain serves `numSteps` ticks per frame, and the reap gets the
frame's first tick, as on a host. The payload is just the capture ticks: the bundle codec is not
under test here, and `LinkModelPinsTodayTest.cpp` already drives it.

`lastServerTick` is one window (`windowTicks()`, 120 at 60 Hz). The windows open on the reap at tick
0 and close on the reap at tick 120, so `latest(wire)` after the run is the summary of everything
that arrived up to and including that frame.

## §2 The exact case, and the value it derives

A fixed link of `d = 3` ticks and a lead of `L = 2`. A capture sent at `t` is due at `t + 3`, so it
arrives in the first frame `F` at or after `t + 3`. The drain before that frame served through
`F − 1`, so `nextUndrainedTick = F`. The setup computes, for every send whose `F` is inside the
window, `captureTick + D − nextUndrainedTick = (t + 2) + 6 − F`, and compares the coordinator's
summary with its own nearest-rank p05, p50, p95, minimum and count, field by field. It also checks the
line's text exactly, `conn=` included.

- `numSteps` 1: `F = t + 3`, so every margin is 5.
- `numSteps` 2: frames are on even ticks. An even `t` waits one extra tick for its frame
  (`F = t + 4`, margin 4); an odd one does not (`F = t + 3`, margin 5). 59 of each.

The second test case states those numbers by hand, so a mistake in the setup's own formula would
show up as a disagreement between the two cases.

**Seen failing.** With the coordinator's reference replaced by `m_serverTick` (the first tick of the
last drained frame, which is that drain's `firstUpcomingSimTick`), every margin grew by `numSteps`:
5 → 6 at `numSteps` 1, and 4/5 → 6/7 at `numSteps` 2. The exact case failed on all four fields at
both step counts. The task notes quote the run.

## §3 The jitter case

Uniform jitter of 4 ticks on a base of 3, a lead of 4, one capture per send. The arrival frame is
`t + 3 + ceil(4u)` for a uniform `u`, so the margin is `4 + 6 − 3 − k` with `k` uniform on 1…4: the
values 3 to 6, a quarter each. Its 5th percentile is `L + D − d − ceil(0.95 · 4) = 3`, and the case
checks the summary's p05 is within one tick of it.

The jitter reorders packets. A capture that arrives after a newer one is parked and sampled like any
other new park (og-simulation `docs/InputArrivalMargin-rationale.md` §3). The case checks that the
samples account for every capture that arrived, and that some were reordered, so the check is not
vacuous. With seed 21: 116 arrived and were sampled, 40 reordered, p05 3, p50 4, p95 6, minimum 3.

## §4 The reorder case

Two-point jitter: each packet takes the base 3 ticks, or 3 + 6 = 9 with probability 0.65; a lead of 4,
one capture per send, `numSteps` 1. A fast capture's margin is `4 + 6 − 3 = 7`, a slow one's
`7 − 6 = 1`. A slow packet is overtaken by any fast packet sent in the next five ticks, so a slow capture
arrives after a newer one unless those five are slow too. Such a capture is parked, not relayed,
and counted in `relayOooSkipCount()`.

With more than half of the arrivals slow, the nearest-rank p50 of all arrivals is the slow margin, 1:
the analytic value. The case derives each arrival's margin from the rig's arrival record
(`captureTick + D − arrivalFrame`), requires that every one is 1 or 7, that more than half are 1 and
that their p50 is 1. It also requires that the arrivals that came in order (each newer than every
capture before it) have a p50 of 7, so the case can tell the two samplings apart on its seed. Then it
checks the coordinator's summary: every arrival sampled, p50 1, minimum 1, p95 7, nothing late, and
`relayOooSkipCount()` equal to the arrivals that came out of order. With seed 31: 115 arrived, 82 slow,
67 out of order; the 48 in order have p50 7.

**Seen failing.** With the sample narrowed to the `[Park]` arm (`if (queuedNewEntry)` →
`if (acceptedNew && queuedNewEntry)`), the coordinator sampled 48 of 115 and read p50 7: the case
failed on `samples` and on p50 (`7 == 1`), and the jitter case failed on `samples` (76 of 116). The task
notes quote the run.

## §5 The other cases

- **Late.** A lead of −5 gives `2 − 5 + 6 − 3 = −2`: every capture arrives after its release tick.
  The window then has no samples, `lateRefused` equals the arrivals and the queue's late count, and
  the line is still written.
- **Re-sends.** Redundancy depth 3 with a margin of 5: every capture is re-sent twice while it is still
  parked. Those are duplicates, so `lateRefused` stays 0. Counting `!queuedNewEntry` as late instead
  reports 233 (seen failing).
- **No sample before the first drain.** An input received before any frame records nothing; after
  one frame, the next one does.
- **The window.** One input at margin 13 (`8 + 6 − 1`), then frames up to the window: no line before
  the reap at tick 120, exactly one at it, and a wire with no input gets neither a line nor a
  `latest`. A second, empty window writes no line and keeps the old `latest`.
- **Dead wire.** An input on a handle that reports dead, then a reap on a dwell boundary: that
  connection is forgotten and the live one keeps its sample.
- **The class alone.** Margins 1…20 in a window of 10 ticks: nearest-rank p05 1, p50 10, p95 19; the
  first close arms the window, a close one tick early does nothing, an empty window returns no line.
  The same samples are first sightings, so the signal reads m0 p05 1 and p50 10, and the one late
  refusal, a re-send, is `lateResent`.
- **The appended fields.** The exact case and the late case compare the whole line, the five appended
  fields included; the window case checks `lateFirstSight=0 lateResent=0 m0P05=7 m0P50=7
  stepsPerBundle=0` (the rig feeds `receiveRemoteInput` directly, so no step is sampled). The exact case
  also checks `m0 = margin − D` field by field, and the late case that every late capture is a late first
  sight with m0 `L − d = −8`.

## §6 The first-sight cases (og-input-latency P3-5), tag `[FirstSight]`

These check the late split (`lateFirstSight`, `lateResent`), the D-free `m0 = captureTick − lateGate`,
its EWMA and the steps per bundle (og-simulation `docs/InputArrivalMargin-rationale.md` §8–§11).

**The oracle.** The rig's `receive` records every copy it hands the coordinator (`Copy`): the owner, the
tick, the frame, whether it is the rig's own first copy of that (owner, tick), and whether the queue's
late count moved across that one call. `oracle()` derives from that record, and from nothing the margin
object computes: `lateFirstSight` = late first copies, `lateResent` = late other copies, and the m0
samples = `captureTick − frame` of every first copy after frame 0. (At `numSteps` 1, `lateGate` is the
frame: the drain before it served through `frame − 1`.) `checkSplitAgainstOracle` then requires the
window's `lateRefused` to equal `lateFirstSight + lateResent` and the queue's late count, each count to
equal the oracle's, the m0 sample count to match, and p05 and p50 to equal the oracle's nearest-rank
values. The sum alone could not catch a wrong split; the per-count comparison does.

All of them run one window (120 ticks), D = 6 unless stated, sends from tick 10.

- **(a) Loss.** A fixed link of 3 ticks, loss 0.15, redundancy depth 3, a lead of −3, so a newest copy
  arrives on its release tick (margin 0, parked) and each re-send after it (late). When a bundle is lost,
  its newest tick first arrives in the next bundle, past its release tick: a late **first** sight. Seed
  41: 22 sends lost; 20 first copies arrived late, all of them recovered ticks; the split read 20 late
  first sights and 153 late re-sends (173 late in all); 108 m0 samples, p05 −7, p50 −6.
- **(b) Less slack than the re-send span.** No loss, depth 3, a lead of −2 (margin 1). The first re-send
  arrives while its capture is still parked (a duplicate); the second after its release (late). 106 late
  re-sends, 0 late first sights; 108 m0 samples, p05 and p50 −5.
- **(c) Reorder.** Depth 1, a two-point link (3 ticks, or 6 with probability 0.35), a lead of −2: a fast
  copy has m0 −5 (margin 1, parked), a slow one m0 −8 (margin −2, late). Each tick is sent once, so every
  copy is a first copy. Seed 51: 49 late first copies, 36 of them older than a capture already seen;
  the split read 49 and 0; 107 m0 samples, p05 −8 (the slow ones), p50 −5.
  **Seen failing:** with `firstSight` taken from the owner's watermark (`acceptedNew`), the split read
  13 late first sights and 36 re-sends, with 71 m0 samples of 107.
- **(d) Two delays.** Uniform jitter (3 + 0–4 ticks), lead 4, depth 3, run at D = 1 and at D = 6
  (`makeUniformDelayConfig`: every tier delay set, the floor 0). At D = 1, 266 copies were late: 47 first
  sights and 219 re-sends. At D = 6, none. Both read 117 m0 samples, p05 −3, p50 −1 and the same EWMA,
  while the D-inclusive margin sampled 70 and 117: it is censored at the late gate, m0 is not.
- **(e) Steps per bundle.** The real bundle codec over a fixed link (`runBundles`), depth 3: a send every
  tick reads `stepsPerBundle` 1, a send every second tick (two new captures per send) reads 2.
- **(f) The EWMA.** A fixed link of 3 and depth 1, the lead stepping from 2 to 5 at tick 60, so m0 steps
  from −1 to 2. Four windows. α is `1 / tickFrequency`. The windows that end before tick 360 (60 plus 5 s
  of ticks) are more than 0.1 tick from 2 (1.125 at 120, 0.125 at 240), so the check is not met by a
  filter that only reports its last sample. Those at 360 and 480 read 2.0, with p50 2.
- **(g) Two owners on one wire.** Owners 7 (slot 0, a link of 3) and 8 (slot 1, a link of 5) send the same
  capture ticks, lead −2, depth 3. Owner 8's first copies all arrive late; owner 7's re-sends past their
  release are late. The split read 106 late first sights (owner 8's) and 315 re-sends; 214 m0 samples;
  one connection, two owners. **Seen failing:** with the mask keyed by the wire's hash, the split read 0
  late first sights and 421 re-sends, with 108 m0 samples.
- **forgetOwner.** Two owners sighted, then `forgetOwner` on the coordinator: one owner left in the margin.

## §7 The class alone, P3-5

- **A parked capture that is not a first sight** keeps its margin sample and adds no m0, and does not
  move `effectiveDelayTicks`.
- **`noteSighting`** is true exactly once per (owner, tick), separately per owner; an older tick inside
  the mask is tested by its bit; a tick 63 ahead shifts the mask and keeps the old anchor at bit 63; a
  tick 64 or more away either way restarts the mask and reads as first; 41 ticks sighted three times
  give 41 first sightings; `forgetOwner` forgets.
- **Steps per bundle**: from 10, the newest ticks 12, 11, 14, 31, 47 sample 2, 2 and 16. The reordered
  11 and the jump of 17 are not sampled, and the jump moves the mark. p50 2.
- **The EWMA** at a tick frequency of 4 (α = 1/4): samples 8, 0, −8 (the last a late first sight) give
  8 → 6 → 2.5, reported as 40 (2.5 × 16).

## §8 The back-dating cases (og-input-latency P3-6), tag `[BackDate]`

They check og-simulation's back-dating detector (`InputArrivalMargin-rationale.md` §14) through the real
coordinator. The test holds its own copy of the rule's design values (`kSpecGraceWindows`,
`kSpecSustainWindows`, `kSpecMinSamples`: 5, 5, 60) rather than reading the class's constants, so a change
of a constant in the header fails a case instead of moving the test with it. The threshold is read from
`TimeConfig`, never restated. `qualifyingWindows` applies the rule's qualifying test to the rig's
`signals` (from the 6th window with samples on, at least 60 samples, m0 p50 below the threshold) and
reports how many windows qualified, the longest run, and the first one. Each case first requires what
makes it non-vacuous, then checks the episodes.

- **Honest, jittered, 10 minutes.** Depth 1, a link of 5 ticks plus a uniform 0–5 ticks of jitter, and a
  path swing drawn uniformly in ±3 ticks once per 2 s window of send time (`pathSwing`, seed `0x5EED`).
  The lead is 5, so a capture's m0 runs from −1 down to −5 at a swing of 0. 36000 server ticks: 300
  windows, m0 p50 over the session −3, the lead "parked 3 ticks low". The swing is what moves a window's
  p50: per-packet jitter averages out over a 120-sample window. 12 windows qualified, never two in a row.
  0 episodes, no `[InputArrival.BackDate]` line.
- **Back-dated by 6.** A fixed link of 3. Section one: the lead is −3 from the start (an honest lead would
  be 3), so every m0 is −6. Section two: an honest lead of 3, stepping to −3 at tick 1800 (window 15).
  60 windows each. The first qualifying window is index 5 (the 6th window, the first after the grace)
  in section one and index 15 in section two. In both, `backDateEpisodes` is 0 up to the window before
  the 5th qualifying one, and 1 from it to the end: exactly one episode, opened in the 5th qualifying
  window, and kept open, not re-reported, while m0 stays low. The one line reads
  `[Warning][InputArrival.BackDate] conn=… m0P50=-6 windows=5 threshold=… D=6`, and it follows the
  `[InputArrival.Window]` line of the window that opened it.
- **Dips.** An honest fixed link (m0 0) with 6 ticks of extra path delay for 3, 4 or 5 windows of send
  time from window 10: m0 −6 for exactly that many windows (the case requires the count and the run).
  3 and 4 open no episode; 5 opens one. The case pins the sustain at exactly 5.
- **A join.** Sends start at tick 37 with m0 −10 (lead −7 over a link of 3; every capture late) and the
  lead steps to 3 after 10 s. The first five windows qualify on everything but the grace: 81, then 120
  samples, all at p50 −10. The case requires the 6th window's p50 to be at or above the threshold. 0
  episodes.

**Seen failing** (`kBackDateSustainWindows` set to 1 in the header, one replacement): the honest case
FAILED with `episodesSoFar(rig) == 0u` expanding to `12 == 0`, and the 3- and 4-window dips with
`1 == 0`. The back-dated case failed in both sections (its line followed the 6th and the 16th window's
line, not the 10th and the 20th), and both class-alone cases failed: 5 of 29 cases. With the grace
removed (`windowsWithSamples <= 0u`, one replacement), the join case FAILED with `1 == 0`, the
back-dated case's first section opened after the 5th window's line, and both class-alone cases failed:
4 of 29.

## §9 The class alone, P3-6

- **Grace, sample floor, sustain and close**, on one wire with 10-tick windows and D 6 in each sample:
  five windows far below the threshold open nothing (grace); then four qualifying windows and one with
  59 samples (reset), four and one at the threshold exactly (reset, the test is strict), and four more:
  the fifth opens, reporting its p50, `sustainedWindows` 5, the threshold and D 6. A window of 200
  samples far below reports nothing (open). A window with only a late refusal and a late re-send has no
  m0 sample: it gets a line with `samples=0`, reports nothing and does not close the episode, so the next
  qualifying window still reports nothing. One sample at the threshold closes it (the floor does not
  apply to closing), and five more qualifying windows open a second episode: `backDateEpisodes` 2.
  `forget` drops it with the wire.
- **The grace is per wire.** Two wires, every window qualifying; the second wire starts five windows
  late. The first opens in window 10, the second in window 15: each counts its own windows with samples.
