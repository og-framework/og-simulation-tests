<!-- SPDX-License-Identifier: MPL-2.0 -->
# `DelayTransitionTest.cpp` — rationale

Source: `DelayTransitionTest.cpp`, tags `[Network][DelayTransition]` (og-input-latency P6-2). It pins what the server's
input drain does when a client's input delay changes mid-session, and that the client's prediction clock pays the change:
a raised delay as Stalls (T11), a lowered one as Skips (P6-2). The clock's own cases are in
`PCTimeManagement/ClientPredictionClockTest.cpp` under `[DelayTransition]`.

This file has no guards document: it states no prohibition.

---

## §1 The session the cases share

`Session` runs one sending client and the real `ServerReceptionCoordinator` over `FStandaloneTestHandle` and one mock
simulatable (`MockSimA`, an `int` input), at one server tick per loop. Each server tick `t`, in order:

1. The client's captures for `t` go up the link (zero or one capture tick; the case decides). The link is a fixed
   `kUplinkTicks` = 1 tick, so a capture sent at `t` is received at `t + 1`.
2. The arrivals due at `t` are fed to the coordinator's per-slot `receiveRemoteInput`, one capture per call.
3. The case's `beforeDrain` runs: a floor change, an RTT sample, a clock request.
4. The server drains `t`: `releaseDelayedInputs(t, 1, …)`.

**Lateness is the drain's own quantity.** Each release records `t − (captureTick + delay)`, with `delay` the queue's
`effectiveDelay` for the wire as the drain reads it: the same arithmetic as the `[Release] … late=` line the coordinator
logs. A tick on which nothing is released, after the first release, is recorded as an empty tick.

**The client is `kLeadTicks` = 2 ahead**: in the scripted cases its capture at server tick `t` is `t + 2`; in the clock
cases it comes from a real `ClientPredictionClock` advanced twice before the first server tick. So a capture `c` arrives at
`c − 1` and is due at `c + D`: every input arrives `D + 1` ticks early, and lateness can only come from the drain's backlog,
never from the link.

The floor is the server's `TimeConfig::relayDelayFloorTicks`, which the test owns and writes as the manager's setter would.
With no RTT sample the wire has no tier entry, so its delay is the floored fallback `max(floor, rttTierInputDelays[3])`, which
is the floor itself at floors 6 and 5.

## §2 Why a lowered delay stays late, and what a Skip does

The queue releases one parked input per slot per drained tick, the smallest due capture first
(`ServerInputDelayQueue::tryDequeueForTick`). Lowering the delay by Δ at tick `T` makes the captures `T − D` … `T − D + Δ`
due at once; one is released, the rest wait, and with one capture per tick the backlog never shrinks: every later release is
Δ late. Raising the delay instead leaves Δ ticks with nothing due (empty ticks), and nothing is late.

A Skip leaves one capture tick unproduced. When the release front reaches it, that tick has no input of its own and the
waiting one is released on time: the backlog drains by one tick per Skip, about `D` ticks after the Skip, and no input is
dropped. With no backlog to drain, the same gap is one empty tick.

## §3 The cases

| case | the scene | what it pins |
|---|---|---|
| `TodayALoweredFloorWithAGapFreeStreamStaysOneTickLate` | floor 6 → 5 at tick 100, a gap-free stream | (i), the defect: every release from tick 100 is `late=1`, 600 and more of them after tick 106, and every release before it is on time |
| `OneMissingCaptureTickDrainsTheLoweredFloorsBacklog` | the same, and the client's capture numbering jumps by one at tick 103 (capture 105 is never produced) | (ii): releases are late from tick 100, on time for every release after tick 110 (= 105 + 5), and no tick is empty |
| `ARaisedFloorAndOneStallLeaveOneTickWithoutARelease` | floor 5 → 6 at tick 100, and the client produces nothing at tick 103 and numbers its captures one lower after it | (iii): exactly one empty tick, tick 100, and no late release |
| `TheClocksSkipDrainsTheLoweredFloorsBacklog` | (ii) with the captures from a real clock in its dead band, and `requestInputDelayTransition(−1)` at tick 103, the floor path's request | (iv): exactly one Skip, late releases after tick 100, and on time for every release after the Skip's unproduced capture + 5 |
| `TheFirstTierArrivalsChangeIsTheFallbacksToTheTiers` | a `ReplicatedTierConsumer` at floor 0, before and after its first tier | the quantity a first tier arrival pays: −3, −2, −1, 0 for tiers 0–3 (`ConnectionTierTable-rationale.md` §8's worked table, its TRUE delay change column) |
| `ATierPathJoinBelowFloorFourEndsWithNoLastingLateness` | floors 3 and 2: the server samples the wire's RTT on every tick, with no reading until tick 40, so the first inputs park at the fallback (4); at tick 40 the first reading (1 ms, tier 0) lowers the wire's delay to the floor; the tier it publishes reaches a `ReplicatedTierConsumer` 3 ticks later, which requests the change from a real clock | the tier path, both ends: one publish, tier 0; the client's delay ends equal to the server's; `4 − floor` Skips; releases `4 − floor` late after the switch, and every release from tick 140 on time |
| `ATierPathJoinWhoseFirstSampleLeadsItsInputsPaysOnlyEmptyTicks` | floors 3 and 2, with an RTT reading from the first tick, so the server switches before it parks anything | the cost when there is no backlog to drain: no late release, `4 − floor` Skips, and exactly `4 − floor` empty ticks |

The tier cases are the engine-free model of one adapter's session (`UESimulationSession::onConnectionTierReceived`: the
consumer's delay before and after the arrival, requested as the clock's transition); they cannot reach the adapter itself.

## §4 Seen failing (og-input-latency P6-2, 2026-11-22)

Each arm was applied to the shipped source, built, run over `[DelayTransition]` (these cases and the clock's three), and
reverted.

- **The clock ignores a negative request** (`requestInputDelayTransition` acting only on `delta > 0`, the pre-P6-2
  behaviour): 6 of 10 cases fail. `TheClocksSkipDrainsTheLoweredFloorsBacklog`: `skipCount() == 1u` → `0 == 1`. The tier
  join at floors 3 and 2: `skipCount()` → `0 == 1` / `0 == 2`, and every release from tick 140 on time → `0 == 661`. The
  no-backlog join: `0 == 1` Skips, `0 == 1` empty ticks. The clock's own `ANegativeDebtIsPaidAsExactlyThatManySkips`:
  `getPendingInputDelayTransitionTicks() == -2` → `0 == -2`. `TodayALoweredFloorWithAGapFreeStreamStaysOneTickLate` passes:
  it is today's pin.
- **The coordinator's pre-P6-2 dedup** (a missing entry read as "told tier 0", so a first tier 0 is never published): both
  tier cases fail, at both floors: `tierSink.tiers == { 0 }` → `{  } == { 0 }`, the client's delay against the server's →
  `4 == 3` / `4 == 2`, `0 == 1` / `0 == 2` Skips, and `0 == 661` on-time releases from tick 140.
- **The phase bit set for a positive debt only** (its pre-P6-2 meaning): the clock's `ANegativeDebtIsPaidAsExactlyThatManySkips`
  and `AHardResyncClearsEitherSign` fail on `publishedCorrectionPhase().transitionDebt` → `false`, and
  `PCTimeManagement/ClockDilationControllerTest.cpp`'s `ARealClocksSkipDebtFreezesItAsAStallDebtDoes` on `freeze == TransitionDebt` → `0 == 4`
  (`None` against `TransitionDebt`).
