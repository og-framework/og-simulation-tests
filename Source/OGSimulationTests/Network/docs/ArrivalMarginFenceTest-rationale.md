<!-- SPDX-License-Identifier: MPL-2.0 -->
# `ArrivalMarginFenceTest.cpp` — rationale

Source: `ArrivalMarginFenceTest.cpp`, tags `[Network][InputArrivalMargin][ArrivalMarginFence]`. It is the
fence of og-simulation's `docs/InputArrivalMargin-rationale.md` §13: no decision of the
`ServerReceptionCoordinator` may read its `InputArrivalMargin`, its back-dating episode state included
(§14 there, and guard G-07 of its guards document).

This file has no guards document: it states no prohibition.

---

## §1 The session

`runSession` drives the real coordinator over `FStandaloneTestHandle` and a two-field input
(`FenceInput`: the capture tick and the owner), through the real bundle codec and `receiveInputBundle`,
for 1200 server ticks (ten windows), one frame per tick: send, deliver, drain, reap. D = 6
(`relayDelayFloorTicks`).

Two owners share one wire, each with redundancy depth 3 and its own seeded `LinkModel` uplink: owner 7
(slot 0, lead −2, seed 61) and owner 8 (slot 1, lead −1, seed 62). Each uplink adds 3 ticks plus a
uniform 0–4 ticks of jitter and loses 10 % of its sends; owner 8's also reorders 10 % of them, held for up
to two later deliveries. The margin of a first copy is then about 2 down to −3 ticks, so the session
exercises every path the margin reads: parks, out-of-order parks, late first copies, late re-sends,
duplicates and lost bundles. It is heavy on lateness by design: of about 2400 captures, 923 are released, and
the queue counts 5352 late refusals (2668 by tick 600). Its m0 sits low enough that the clean run opens one
back-dating episode, at its last window.

## §2 What is compared

The session runs twice, the second time calling `editArrivalMarginForFenceTest().scribbleForFenceTest`
with a fixed seed at tick 600, before that frame's sends. Each run records:

- every release from the drain (owner, capture tick, and the input's two fields);
- every relay call (owner, capture tick, the stamp `dA`, the input's tick);
- every undelayed delivery (none expected; compared anyway);
- every `[Park]` and `[Release]` line, as text;
- `lateDroppedCount()` after every frame.

Every one must be equal between the runs. The case first requires that the session is not vacuous: more
than 500 releases and relay calls, more than 100 late refusals, late refusals after tick 600, ten
`[InputArrival.Window]` lines, and that those lines **differ** between the runs. That last check proves
the scribble reached the margin object; without it, a scribble that changed nothing would pass the
fence. Since og-input-latency P3-6 the scribble also overwrites the back-dating episode state, and the
case requires that it reached it: the final `backDateEpisodes` differ (clean 1, scribbled 2) and so do
the `[InputArrival.BackDate]` lines (clean 1, scribbled 0).

## §3 Seen failing

A probe that lets the mask steer the queue (`enqueue` given the "no late gate" sentinel whenever
`noteSighting` returned true) failed the fence: the releases, the relay calls, the `[Park]`/`[Release]`
lines and the per-frame late counts all differed. The probe also failed the behaviour cases of
`InputArrivalMarginTest.cpp`, as it should: it changes what the queue refuses.

A second probe, for the episode state (og-input-latency P3-6): `enqueue` given the "no late gate"
sentinel whenever the wire's `signal()` reported `backDateEpisodes > 0`. The fence FAILED on the
releases, the relay calls, the `[Park]`/`[Release]` lines and the per-frame late counts; it was the only
case that failed.
