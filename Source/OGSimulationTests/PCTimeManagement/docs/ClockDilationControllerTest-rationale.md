<!-- SPDX-License-Identifier: MPL-2.0 -->
# `ClockDilationControllerTest.cpp` — rationale

Source: `ClockDilationControllerTest.cpp`, tags `[PCTM][Dilation][ClockDilationController]`. It tests og-simulation's
`ClockDilationController` (og-simulation `docs/ClockDilationController-rationale.md`) value by value: every input is set
by hand, so every expected scale is known exactly. Added by og-time-dilation task 6 (2026-11-21); its acceptance criteria
are named (a)–(g) below. Task 7b (2026-11-21) made the guard's low edge frame-aware and added the three frame-period cases
(§2, report §4 item 6). Task 10b (2026-11-21) added the target's step lag and its case (§2, research topic
`lead-definition`'s report §6 AC1). Task 10c (2026-11-21) made the guard centre the clock's published read and added the
change hysteresis: the rig now supplies a clock read, and two cases were added (§1, §2; research topic `on-readiness`'s
report §7 AC2 and AC4).

This file has no guards document: it states no prohibition.

---

## §1 The rig

- **`inputsWithError(now, error)`** builds a valid-sensor input whose lead is the `Common` target (2.5 ticks) plus `error`.
  The guard centre is `predictionOffsetTicks − (serverTick − authorityTick)` = 4 − (1000 − 998) = 2. The high edge is
  2 + 2.5 = 4.5. The low edge is frame-aware (og-simulation `docs/ClockDilationController-rationale.md` §5): 2 − 0.5 = 1.5 on
  a controller's first call (no frame sampled yet), and 2 + (1 + 0.9375/2 − 1) − 0.5 = 1.96875 once the 1/64 s call period
  (0.9375 tick) is sampled. Either way the interval contains the target, so the guard never binds unless a case asks for it.
  Most cases use `Common` so the error is exactly the one asked for; (e) uses `OwnRtt`.
- **The step lag (task 10b).** From a controller's second call the target carries the step lag, half the mean frame
  period: 0.9375 / 2 = 0.46875 tick at the 1/64 s call period, exact in binary. `Rig::callWithError(error)` adds it to the
  lead from the second call on (`kCallStepLagTicks`), so the error the controller computes is still exactly the one asked
  for and (b)'s and (c)'s numbers are unchanged. The cases that build inputs with `inputsWithError` themselves keep the
  plain lead: the freeze cases' errors of 3 ticks become 2.53, still saturated at ±1 %.
- **The clock read (task 10c).** The guard centre's sample is the clock's published read against the previous call's
  sensor (og-simulation `docs/ClockDilationController-rationale.md` §5.4). `Rig::call` gives every call whose input carries
  no read (`clockRead.advances == 0`) a new one, `{call count, authorityTick + predictionOffsetTicks}`, consistent with the
  input's A and O: one advance between calls. With the server tick constant (1000) the sample is 1002 − 1000 = 2, the
  centre every case already asserted; at `midBandOwnRttInputs()`'s 1000.5 it is 1.5, as before. The cases that call
  `ClockDilationController::update` without the rig (`TheBandGuardClampsACommonTarget`, `OwnRttReproducesTheMeasuredMeanLead`,
  `TheTargetLowPassesAtItsTau`, the Off cases) pass no read: their first call seeds the centre from the input's
  `O − (Ŝ − A)` as before, and a later call with the same count holds it. No asserted number changed.
- **`centreInputs(call, drainAtStepStart)`** (task 10c) is a 30 fps client whose server moves 2 ticks per call: Ŝ = 1000 + 2n,
  the fresh authority tick 998 + 2n. The read visible at call n is the one frame n − 1's step made, `(998 + 2(n − 1)) + 4`.
  The stage's own A is that stale tick in the step-start order and the fresh one in the frame-start order, so the formula
  before task 10c gives centres 2 ticks apart, and the read against the previous call's Ŝ gives 2.0 in both.
- **`midBandOwnRttInputs()`** is `inputsWithError`'s input with the server tick at 1000.5, so the guard centre is
  4 − 2.5 = 1.5 and the `OwnRtt` setpoint 2.6 (R 30 ms, J 2 ms, m 0.5, floor 4). With or without the lag the target stays
  inside the edges at every frame period the step-lag case uses (at 2 ticks: low 2.5, high 4.0, target 2.6 + 1.0).
- **`Rig`** owns one controller and a model of the tick driver's door: it hands the door's scale in as `currentScale`, and on
  `apply` sets the door to the returned scale and counts the call. Calls are 1/64 s apart, so every clock time is exact in
  binary and "exactly 1.0 s" can be asserted with `==`. `callAfter(seconds)` and `callEvery(seconds, n)` make calls at another
  period, for the frame-period cases.
- **`lowEdgeConfig()`** is `On` with a `Common` target of −10 ticks, below every low edge, so the clamped target the
  controller reports is the low edge itself.
- **`variedInputs()`** mixes errors from −3 to +25 ticks, an invalid sensor, a re-centre, a stall debt, a resync count and a
  door at 0.99, for the Off case.

## §2 The cases

| case | acceptance criterion | what it asserts |
|---|---|---|
| `OffIsScaleOneAndNeverApplies` | (a) | every varied input, either setpoint: scale 1, apply false, freeze `ModeOff`; the controller's scale is still 1 after |
| `OneTickAheadSlewsToPointNineNineFive` | (b) | +1 tick: 0.998, 0.996, 0.995, then 0.995 for 600 more calls; apply true exactly on the 3 changes |
| `ThreeTicksSaturateAtTheDeviation` | (c) | +3 → 0.990 and −3 → 1.010 after 20 calls, apply false once settled |
| `EachFreezeCauseSlewsBackToOneAndHolds` | (d) | from 0.990, each of the four sensor and phase causes (the re-centre twice, armed and recentring: five runs) reports itself and slews 0.992 … 1.0 and holds; released, the scale steps 0.998 |
| `AfterResyncFreezesForExactlyTheWindow` | (d) | a hard-resync count change freezes for 64 calls (1.0 s), releases at exactly 1.0 s; an unchanged count does not freeze, the next change does |
| `TheFirstMatchingFreezeCauseIsReported` | the Description's order | all causes on, then removed one by one in order |
| `OwnRttReproducesTheMeasuredMeanLead` | (e) | R 72.5 ms, J 9.5 ms, m 0.5, F 4 → 2.461 ± 0.001 and exactly 2.460 ± 1e-9; R 0.1 ms, J 0 → 3.497 ± 0.001, also 10 s later |
| `TheTargetLowPassesAtItsTau` | the Description's low-pass | before the first valid sensor the raw target is reported; a step after 10 s moves by `1 − e^(−1)` of the gap |
| `TheBandGuardClampsACommonTarget` | (f) | a `Common` target of 6 with c = 2 → 4.5, `guardBound`; −1 → 1.5 (the first call's low edge, c − 0.5; task 6 asserted 0.5 = c − 1.5 before task 7b), bound; 2.5 → not bound |
| `TheLowEdgeFollowsTheFramePeriod` | 7b, report §4 item 6 | the first call's edge is c − 0.5; from the second call, 1/30 s calls (2 ticks, φ̄ = 2.5) give c + 1.0, 1/100 s (0.6 tick, φ̄ = 1.3) c − 0.2, 1/144 s (0.417 tick, φ̄ = 1.208) c − 0.292, each ± 1e-9 and unchanged 600 calls later; a Shadow controller's edge equals On's (c + 1.0 at 1/30 s) |
| `TheFramePeakDecaysAtTheGuardTau` | 7b, report §4 item 6 | from 1/30 s to 1/100 s: one call later the edge is just below c + 1.0 (the peak decays, it does not drop); after 2.0 s (200 calls) the peak is 0.6 + 1.4/e = 1.115, so the edge is c + 0.115 ± 0.005 (and the exact form ± 1e-6); after 30 s more, c − 0.2 |
| `AHitchFrameIsNotSampled` | 7b, report §4 item 6 | at 1/100 s (edge c − 0.2) one call 0.6 s late (36 ticks > `kMaxFrameTicks`) leaves the edge unchanged on that call, the next and 100 after |
| `TheTargetGainsHalfTheMeanFramePeriod` | 10b, report §6 AC1 | Shadow, `OwnRtt`, mid-band inputs, 30 s of calls at constant frames of 0.6, 1.0, 2.0 and 60/144 ticks: unbound; `stepLagTicks` = F/2 ± 0.002, the target minus the raw setpoint (computed from the inputs) = F/2 ± 0.002, and target − `stepLagTicks` = the setpoint ± 1e-9. After 30 s at 0.6 tick one 40 ms frame (2.4 ticks) moves the lag by more than 0 and less than 0.01 (a mean with τ 10 s: 0.0036; the peak would move 0.9); one 60 ms frame (3.6 ticks, over `kMaxFrameTicks`) leaves lag and target unchanged. Off: every field of the output is its default (0, false, scale 1) with `ModeOff`, `stepLagTicks` 0 |
| `TheGuardCentreIsTheClocksReadInEitherDrainOrder` | 10c, report §7 AC2 | 1,800 calls at 1/30 s with `centreInputs`, in each drain order, with a `Common` target below every edge and one above: the centre is 2.0 (the read minus the previous call's Ŝ) ± 1e-9 in both orders and equal between them; the target is the low edge c + 1.0 = 3.0 or the high edge c + 2.5 = 4.5 in both, bound. One more call with the read's count unchanged and Ŝ + 7.25, A − 5 leaves the centre **bit-identical** (and the target within 1e-9: the frame peak's arithmetic moves it in the last bits) |
| `TheScaleHoldsUnderTwoQuantaExceptWhileFrozen` | 10c, report §7 AC4 | the default `changeHysteresisQuanta` is 2. Not frozen, from 0.9995 (one 5-quantum move): a slewed value 1 quantum away (error 0.08) and 1.5 quanta away (0.13) hold the scale with `apply` false, for 100 more calls too; 3 quanta (0.04) moves it to 0.9998 with `apply`. Frozen (a stall debt) at 0.9999: the next call returns exactly 1.0 with `apply`, and stays. Task 6's (b), (c), (d) and (g) are unchanged: every step they take is at least 10 quanta |
| `ShadowFollowsOnsTrajectoryWithoutApplying` | (g) | a 400-call script (a sine error, an invalid stretch, a stall debt, a resync): Shadow's scale, freeze and error equal On's every call, Shadow never applies and its door stays 1.0; On applies exactly on its changes |

(e)'s 2.461 is the redesign's value from unrounded per-run means; at the rounded inputs the rule gives 2.460, on the edge of
± 0.001 (0.00099999999999989 in float64). The second assertion pins the exact value so a last-bit change cannot pass silently.

## §3 Seen failing

Run red first on a temporary edit of `ClockDilationController.h`, restored byte-identical (sha256 checked):

- **(b) with the error's sign flipped** (`out.errorTicks = out.targetTicks - out.leadTicks;`): 4 of 10 cases red,
  23 assertions. `OneTickAheadSlewsToPointNineNineFive`: `-1.0 == Approx( 1.0 )` and `1.002 == Approx( 0.998 )`,
  `1.004 == Approx( 0.996 )`, `1.005 == Approx( 0.995 )`; also (c) and both freeze cases.
- **(e) without the two `0.5` terms** (`std::max(floorTicks, margin) - halfRtt + m_config.targetOffsetTicks`): 1 of 10 red,
  2 assertions, both on the loopback arm: `3.997 == Approx( 3.497 )`. The emulated arm stays green: above the floor the two
  halves cancel.
- **The hitch skip removed** (task 7b: `if (frameTicks <= clockDilation::kMaxFrameTicks)` → `if (frameTicks <= 1e9)`): 3 of
  13 cases red, 5 assertions. `AHitchFrameIsNotSampled`: `37.0 == Approx( 1.8 )` on the hitch call, then `36.8234417634` and
  `22.9640973701` (the 36-tick peak decaying at 2 s). Two task-6 cases also go red, because their calls are 10 s apart
  (600 ticks): `OwnRttReproducesTheMeasuredMeanLead` and `TheTargetLowPassesAtItsTau` report `601.0`: the low edge
  2 + (600 + 0.5 − 1) − 0.5, far above the high edge 4.5.
- **The header before the step lag** (task 10b: the og-simulation header of task 7b, restored afterwards; the test's
  `stepLagOf` read 0 because `stepLagTicks` did not exist): `TheTargetGainsHalfTheMeanFramePeriod` red, 9 assertions:
  `CHECK( stepLagOf(last.out) == Catch::Approx(frameTicks / 2.0)… )` and `CHECK( offset == … )` with `offset := 0.0` at
  `frameTicks := 0.6`, `1.0`, `2.0` and `0.4166666667`, and `moved := 0.0` for the 40 ms frame.
- **Task 10c, the guard sample as before** (`guardSample` returning the stage's `O − (Ŝ − A)` on every call):
  `TheGuardCentreIsTheClocksReadInEitherDrainOrder` red, 14 assertions: `stepStart.last.guardCentreTicks := 0.0` against
  `frameStart.last.guardCentreTicks := 2.0` (the centres differ by 2.000), the step-start targets `1.0 == Approx( 3.0 )` and
  `2.5 == Approx( 4.5 )`, and the no-new-read call moves the centre (`-0.2024746907 == 0.0`, `1.7975253093 == 2.0`) and the
  target with it.
- **Task 10c, the read paired with this call's Ŝ** (`- in.serverTick`): the same case red, 8 assertions: the centre
  `0.0 == Approx( 2.0 )` in **both** orders, low by the call's Ŝ step of 2 ticks, and the edges with it (`1.0`, `2.5`).
- **Task 10c, the hysteresis without the freeze exemption** (`frozen ||` removed): `TheScaleHoldsUnderTwoQuantaExceptWhileFrozen`
  red, 3 assertions: `released.out.scale == 1.0` with `0.9999 == 1.0`, `released.out.apply` false, and the next frozen call
  still `0.9999 == 1.0`.
- The compile-time checks are in the header and are recorded in its rationale (§12).
