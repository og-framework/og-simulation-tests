<!-- SPDX-License-Identifier: MPL-2.0 -->
# `ClockDilationLoopTest.cpp` — rationale

Source: `ClockDilationLoopTest.cpp`, tags `[PCTM][Dilation][ClockDilationLoop]`. It tests og-simulation's per-frame client
stage `clockDilation::beforeClientPump` and its `ClockDilationWindow` (og-simulation
`docs/ClockDilationController-rationale.md` §13, §14) in a closed loop built by `ClockLoopHarness.h`
(`docs/ClockLoopHarness-rationale.md`), plus the window's line and kept series value by value. Added by og-time-dilation
task 7 (2026-11-21); its acceptance criteria are named (a)–(g) below. Task 7b (2026-11-21) made the band guard's low edge
frame-aware: it replaced task 7's 30 fps characterisation with a pass case and added the frame-rate, any-target and
frame-jitter cases (§2, §3.1). Task 10b (2026-11-21) added the target's step lag: the execution-lead case, the window's
`wouldChanges` and `stepLag` fields, and a characterisation of the Unreal drain order (§2, §3.2). Task 10c (2026-11-21)
made the guard centre the clock's published read and added the change hysteresis: the harness now runs the Unreal
immediate client's step-start drain order by default, every guard case runs in both drain orders, the characterisation case
is retired into the step-start arm of `ThirtyFpsOnHoldsTheBand`, and two cases were added (§2, §3.3). The whole file runs
in about 2.4 s for its 20 cases (2.39 s measured, task 10c).

This file has no guards document: it states no prohibition.

---

## §1 The scenario

`pairOf(seedA, seedB, mode)` builds two b0-like clients (the harness's default links, 100 frames per second). The first
joins at 2.0 s, the second at 2.375 s: 142.5 server ticks, so its join lands half a tick away from the first's. That is the
"different residuals" the criteria ask for, by construction rather than by seed. **Steady** starts 10 s after the later of
the two `joinSettledSeconds()`, the end of both joins' re-centre debts. The pair's lead difference is the harness's true
deadline lead of each client, sampled at the same true instant every 10 ms.

Every client drains in the harness's default order, the step-start one (`docs/ClockLoopHarness-rationale.md` §2), unless
a case passes the frame-start order; `kDrainOrders` is the pair the guard cases loop over.

The seed pair of the single-seed cases is 1/101, the first pair tried. The frame-rate cases use
`pairAt(seeds, mode, fps, jitter, drainAtStepStart)`, the same pair at another frame rate, frame jitter and drain order, over the eight pairs `kGuardPairs` (1/101, 2/102, 3/103 and
1001 + 17k against 5003 + 31k for k = 0–4, i.e. 1001/5003 … 1069/5127; research topic `thirty-fps-guard`'s pairs) or the
two `kTargetPairs` (1/101, 1001/5003).

## §2 The cases

| case | criterion | what it asserts |
|---|---|---|
| `TwoClientsHoldEqualDeadlineLeads` | (a) | On, seeds 1/101: over 120 s of steady state the difference's p95 ≤ 0.10 tick, with 0 Skip, Stall or hard resync after steady starts |
| `EqualLeadsAcrossFortySeedPairs` | (a), derived | On, seeds 1–40 against 101–140: every \|mean difference\| ≤ 0.10, the mean of the 40 mean differences within ±0.02, every p95 ≤ 0.25 (G1's bound), 0 graduated corrections, and the median p95 ≤ 0.11 (§3) |
| `OffKeepsTheJoinResiduals` | (b) | Off, seeds 1/101: \|mean difference\| ≥ 0.4, p95 > 0.10 (so (a)'s bound fails in Off), and no window line |
| `HundredPpmDriftIsHeldWithoutGraduatedCorrections` | (c) | the pair, client 0 at +100 ppm, to 602 s. On: client 0's mean error over the steady frames within ±0.05 tick, its \|error\| p95 ≤ 0.10, and 0 graduated corrections or resyncs on either client. Off: client 0 fires ≥ 1 graduated Stall |
| `SixHundredMillisecondHitchRecovers` | (d), derived; 10c in both orders | one client per seed (1, 2001, 2008, 2015, 2022, and since task 10c 3035, 3048, 3156), each in both drain orders, a 600 ms frame at 30 s: 0.2 s of host time lost; ≥ 1 clock correction; the first unfrozen residual after the last correction below the controller's saturation (maxDeviation / gainPerTick = 2 ticks, so the proportional law governs its decay); when that residual is ≥ 0.5 tick, the mean \|error\| over [+6, +6.5] s after the last correction within ±0.09 (|mean| + 3σ of the measured spread under the hysteresis, §3.3; ±0.07 before task 10c) of the law at the window's midpoint, `residual · exp(−6.25 · f · K_p)`, and exactly 8 such checks run (4 seeds × 2 orders); from 10 s after it \|error\| ≤ 0.13 (§3) |
| `TheAppliedScaleStaysInsideOnePercent` | (e) | the door's scale stays in [0.99, 1.01] on both clients of (a)'s run and in (d)'s; no refused rate change; in (a)'s steady state the p99 of \|scale − 1\| ≤ 0.05 %, and every steady window line's `wouldScaleDevP99` ≤ 0.05 |
| `ThirtyFpsOnHoldsTheBand` | 7b, report §4 item 1 (§3.1); 10c AC3 | the eight `kGuardPairs` at 30 frames per second, 120 s of steady state each, in Off, Shadow and On, in **each drain order**. Every client, every mode: 0 hard resyncs. Off and Shadow: 0 Skips and at most 1 Stall per client (the unchanged clock, lead ruling 02:44 L4); Shadow's mean lead equals Off's. On: 0 Skips, 0 Stalls, the guard binds on ≥ 180 of the steady frames (5 %; ≥ 1 before task 10c, §3.3), each client's mean lead ≤ its Off lead, the pair's p95 ≤ 0.30; over the 16 clients, On's steady corrections ≤ Off's. The step-start arm is the retired `StepStartDrainLetsOnSkipAtThirtyFps_Finding` |
| `TheGuardHoldsAnyTarget` | 7b, report §4 item 2; 10c AC3 | On, the two `kTargetPairs` at 30 and at 100 fps, with `targetOffsetTicks` −0.5, −1, −2, −3 and `Common` at 0.5 and 1.5 ticks, in each drain order: 0 steady Skips, Stalls and hard resyncs in each of the 48 runs |
| `OtherFrameRatesHoldTheBand` | 7b, report §4 item 3; 10c AC3 | On, the eight `kGuardPairs` at 45, 60 and 144 fps, in each drain order: 0 steady Skips, Stalls and hard resyncs in each of the 48 runs. 20 fps is recorded, not asserted (§3.1) |
| `FrameJitterLeavesOnNoWorseThanOff` | 7b, report §4 item 4; 10c AC3 | the eight `kGuardPairs` at 30 fps with each frame period × (1 + 0.2 · U(−1, 1)), in each drain order: Off has ≥ 1 steady correction (the knob is live), and On's total ≤ Off's |
| `TheGuardCentreIsTheClocksRead` | 10c AC3 | On, the eight `kGuardPairs` at 30, 60, 100 and 144 fps, on the b0 link and on a clean one (0.3 ms + 0.2 ms each way), in each drain order (16 cells): per client the mean over the steady stepping unfrozen frames of the guard's centre minus the frame's read centre (the harness's `readCentreTicks`); the mean over a cell's 16 clients within ±0.05, with > 1,000 such frames per client |
| `QuantumChangesStayUnderTheBound` | 10c AC4 | On, the eight `kGuardPairs`, the default (step-start) order, door scale changes per client per second over 120 s of steady state: at 100, 60 and 144 fps on b0 every client ≤ 0.6 and the 16 clients' mean ≤ 0.45; at 30 fps every client ≤ 2.0 (G-DIL-1's G4) and the mean ≤ 1.3; on a loopback link (0.3 ms + 1.5 ms each way) at 100 fps the mean ≤ 0.3 |
| `RenderTimeStaysBracketedUnderDilation` | (f) | (a)'s pair with render on, to 150 s: once 4 snapshots exist, 0 `beyondOldest` frames and the render time never decreases, over ≥ 14,000 frames per client |
| `AnInstantScaleStepLeavesTheBracket` | (f)'s control | forced scale steps 1.25 → 0.75 ten times (no controller): ≥ 1 `beyondOldest` frame and ≥ 1 render-time step backwards. It keeps (f)'s two counters able to fail |
| `ShadowNeverMovesTheDoor` | the stage's apply gate | Shadow pair, 60 s: the door's scale is 1.0 on every frame, no call applies, the would-be scale moves, and every window line is Shadow's with `applied=0` and `rateScale=1.00000/1.00000` |
| `TheSensorEstimatesThePublishedTick` | the truth convention | the steady mean of `serverTickAt − trueTickAt` is −D · f within 0.01 tick, for a publication D = 0 and 1.5 ms after the step's start |
| `KeptSeriesDecimatesAndRanksNearest` | the window's percentiles | 4,096 values keep 1,024 at stride 4; nearest-rank p50 and p99; a full series halves and keeps the new sample only on its new stride; `clear` resets the stride |
| `AScriptedWindowIsTheGoldenLine` | (g) | five scripted calls (a sensor freeze, an after-resync freeze, a guard clamp, applies) give one exact line; after `restart`, one call gives a second exact line with the sensor deltas from the restart's base; `isDue` before and after the 10 s. Since task 10b each line ends in `wouldChanges=` (4, then 0: the second window's one call repeats the first window's last scale) and `stepLag=0.000` (the scripted outputs carry no lag) |
| `ExecutionLeadMeetsTheTarget` | 10b, report §6 AC2 | 20 clients On at 100 and at 30 fps, joins spread over one frame period, ±20 % frame jitter, seeds 7001 + i: each client's mean execution lead (the harness's per-step `tick − S(t)`) minus its mean `target − stepLag` (the execution-lead setpoint) is the client's offset. The mean over the 20 is within ±0.03 (100 fps) / ±0.10 (30 fps); every client within ±(g/2 + 0.05), g the frame grid's period in ticks: ±0.15 / ±0.55 (§3.2) |
| `WouldChangesCountsTheControllersScaleChanges` | 10b, F-c | On and Shadow pairs, 60 s: per client, the window lines' `wouldChanges` sum to the number of frames whose controller scale changed (up to the last line); in On each line's `wouldChanges` equals its `applied`, in Shadow `applied` is 0; each line's `stepLag` is that frame's `stepLagTicks` to 0.0005 |

## §3 Where a criterion's number does not hold, and what is asserted instead

- **(a) across seeds.** The single-seed case passes (0.077). Over 40 seed pairs the p95 is 0.042–0.144, median 0.084, and
  **11 of 40 exceed 0.10**. Each client's sensor error has a standard deviation of about 0.03 tick (the estimator's 5 s
  filter of the link's `(u − d)/2`; with this harness at D = 0 the signed error's p95 is 0.043, about 0.026 standard
  deviation), and the two clients' errors are independent, so the difference alone has a p95 of about
  1.96 · √2 · 0.03 ≈ 0.08. The 0.10 bound is therefore near the
  median outcome, not a tail. The task 7 review ran 24 further pairs (seeds 1001 + 17k against 5003 + 31k, k = 0–23, the
  same `pairOf` scenario): p95 0.058–**0.216** (seeds 1035/5065), median 0.094, 11 of 24 over 0.10, every \|mean\| ≤ 0.047,
  0 corrections. Over all 64 pairs the worst p95 is 0.216, so a bound fitted to the first 40 (their max is 0.144) would sit
  inside the tail. The derived case therefore asserts each seed against G1's own bound, p95 ≤ 0.25, and the median
  ≤ 0.11 (lead ruling 2026-11-21 01:50, widened from 0.10 by the 02:44 ruling, L3 of the task 7 review's round 3). The
  review's three fresh 40-pair sets (41011 + 59k against 43013 + 61k, 47017 + 67k against 49019 + 71k, 53023 + 73k against
  59029 + 79k) gave medians 0.0951, 0.0973 and 0.0912, and a bootstrap of a 40-pair median has mean 0.0949 and standard
  deviation 0.0051. So 0.10 failed about 19 % of fresh sets; 0.11 is about 3 standard deviations. The case's 40 give 0.084
  (0.0844 since task 7b, 0.0841 before).
  - **The bias bounds are derived from the spread of the per-pair mean difference** (lead ruling 2026-11-21 02:22). Over
    168 pairs, its standard deviation is **0.020** (0.017–0.024 per set) and its largest magnitude 0.061. The pairs are
    the 40, the review's 24 above, the review's second 24 (13001 + 19k against 17011 + 29k) and 80 pairs not used for any
    bound (21001 + 23k against 23003 + 37k, and 31013 + 41k against 37019 + 47k, k = 0–39). The earlier per-pair bound
    \|mean\| ≤ 0.05 sat at about 2.5 standard deviations and failed on 13229/17359 (+0.056) and 31669/37771 (−0.061).
  - The case asserts each pair's \|mean\| ≤ **0.10** (about 4–5 standard deviations) and a pooled check, \|mean of the 40
    means\| ≤ **0.02**. With the largest per-set standard deviation, 0.024, the pooled mean's standard error is 0.024 / √40 ≈ 0.004,
    so 0.02 is about 5 standard errors.
    The 40 give −0.003. The five 24- or 40-pair sets give −0.003 to +0.004.
  - **The per-pair p95 ≤ 0.25 and the zero-corrections check are not free of tails either.** Of the 80 unused pairs,
    two exceed 0.25 and two have one steady correction:
    - 31669/37771: p95 0.331, no correction. Its pair is still 0.73 apart when steady starts.
    - 21093/23151: p95 0.256, with a Skip on its second client at 65.2 s.
    - 31423/37489: a Stall on its second client at 127.7 s.

    The case's 40 pairs include none of these.
- **(c)'s "|ε| ≤ 0.05".** The steady mean error is +0.020 tick, the design's δ / K_p at 100 ppm; the per-frame \|error\| p95 is
  0.055 and its max 0.11, the same sensor noise as (a). The case asserts the mean (the drift error the criterion is about)
  within 0.05 and the p95 within 0.10.
- **(d) "the clock's Skips, then |ε| ≤ 0.10 within 6 s of the last Skip".** Two parts do not hold:
  - A 600 ms frame gives **1 hard resync and 23 Stalls, no Skips.** The catch-up batch's first advance already reads every
    authority tick that arrived during the hitch: the clock's drift is about 37 ticks, past the hard threshold 21. It resyncs
    onto its target, the rest of the batch overshoots by 23 steps, and the graduated Stalls and the re-armed re-centre pay
    them back. (A 300 ms frame, measured separately, gives 3 Skips inside the batch and leaves the lead 3 ticks high.)
  - After the last correction a residual remains, and the proportional law removes it with τ = 1 / (60 · 0.005) = 3.3 s.
    Seed 1 leaves 0.95 tick: \|error\| reaches 0.10 at about 7.5 s. 0.10 within 6 s needs a residual below about 0.6 tick.
  - The case compares the law with a **short mean**: the mean \|error\| over [+6, +6.5] s after the last correction (50
    frames). A maximum over everything after +6 s would measure the sensor noise's maximum instead of the law.
    - The law is taken at the window's **midpoint**, `residual · exp(−6.25 · f · K_p)`. Over the window the law falls by
      about 14 %, so its value at +6 s sits above the window's mean (at +6.25 s it is 0.93 × the value at +6 s). Against
      +6 s the differences had a mean of −0.012.
    - The check applies only when the residual is ≥ 0.5 tick. Below that, the law at +6.25 s (≤ 0.5 · e^−1.875 = 0.077) is
      under the sensor noise (§3 (a): about 0.03 standard deviation per client, and up to 0.07 per frame), so the
      comparison would test the noise.
    - It also asserts that the residual is below the controller's saturation (maxDeviation / K_p = 2 ticks), where the
      exponential law applies.
  - **The tolerance was ±0.07 until task 10c, about three standard deviations of the measured difference** Δ = mean − law
    (lead rulings 2026-11-21 02:22 and 02:44, L1 of the task 7 review's round 3). Task 10c's hysteresis biased it and it is
    now ±0.09 (§3.3).
    - 213 seeds: the case's five, the review's 3001 + 37k (k = 0–15) and 5001 + 43k (k = 0–191). 30 of them leave a
      residual ≥ 0.5 tick. Their Δ has mean **+0.000** and standard deviation **0.020**, and lies in [−0.046, +0.041].
      3 × 0.020 = 0.060 was task 7's tolerance.
    - 64 seeds not used for the bound (7001 + 53k): 11 are checked, with \|Δ\| ≤ 0.024.
    - The review's round 3 checked 256 fresh seeds (50021 + 71k): 34 leave a residual ≥ 0.5, Δ has mean +0.0003 and
      standard deviation 0.0248, and seed 54423 (residual 0.904) gives Δ +0.0627, outside 0.06. Pooled with the 30, the 64
      checked seeds give a standard deviation of **0.0225**, so 3σ = 0.068 and 0.07 is about 3.1σ.
    - The bound still fails a law with the wrong gain. Halving K_p gives \|Δ\| 0.19–0.29 over the 41 checked seeds, and
      doubling the exponent gives 0.09–0.17.
    - The earlier ±0.05 at +6 s failed on the review's seed 3038 (Δ −0.058). At the midpoint, 3038's Δ is −0.046.
  - **"≤ 0.13 from +10 s"** (the criterion's 0.10, widened by the 02:44 ruling, L2 of the task 7 review's round 3). The
    case's five seeds reach 0.073. It is a maximum of the sensor noise over the run's last 18 s (about 1,800 frames), not
    a law. Over task 7's 277 surveyed seeds the largest is 0.101 (seed 11193). Of the review's 256 fresh seeds, 5 exceed
    0.10 (about 2 %) and the largest is **0.124** (57618), so 0.10 sat inside that maximum's tail and 0.13 sits above it.

    | seed | corrections | residual | law at +6.25 s | mean \|ε\| [+6, +6.5] s (Δ) | law checked | max \|ε\| from +10 s |
    |---|---|---|---|---|---|---|
    | 1 | 1 HR + 23 Stalls | 0.953 | 0.146 | 0.165 (+0.019) | yes | 0.062 |
    | 2001 | 1 HR + 25 Stalls | 0.888 | 0.136 | 0.117 (−0.019) | yes | 0.073 |
    | 2008 | 1 HR + 24 Stalls | 0.072 | 0.011 | 0.055 | no (below 0.5) | 0.056 |
    | 2015 | 1 HR + 24 Stalls | 0.057 | 0.009 | 0.003 | no (below 0.5) | 0.057 |
    | 2022 | 1 HR + 23 Stalls | 1.045 | 0.160 | 0.158 (−0.002) | yes | 0.072 |

    Seeds 2001–2022 are the review's. Seed 2008 is the one that failed the earliest form (the maximum after +6 s, 0.072,
    against the law + 0.05 = 0.062). Seed 2022's residual of 1.045 is why the residual check is the saturation bound,
    not "≤ 1 tick". Every one of the 277 seeds gives 1 hard resync, 22–25 Stalls and no Skip.

    The table is task 7's tree. **With task 10b's step lag** the target sits 0.3 tick higher, and the residual, the error
    left once the resync and its Stalls have landed the lead on whole ticks, moves with the target modulo one tick: the
    five residuals are 0.047 / 0.890 / 0.069 / 0.058 / 0.050, so the law is checked on seed 2001 only (mean 0.116 against
    the law's 0.136, Δ −0.020). The maxima from +10 s are 0.055 / 0.073 / 0.056 / 0.057 / 0.072.
- **30 frames per second.** Task 7 pinned On at 30 fps as a finding, F4, in a characterisation case that task 7b removed:
  4–8 Skips per client per 120 s, as many 1-step frames, leads 0.9–1.5 below Off's, pair p95 0.66–0.91, and a guard that
  never bound. Task 7b fixed the cause (og-simulation `docs/ClockDilationController-rationale.md` §5) and replaced the case
  with `ThirtyFpsOnHoldsTheBand` (§3.1).

### §3.1 The frame-aware guard's cases (task 7b)

Every number below was measured on the tree that holds the change, by a hidden survey test built and removed inside one
build lock hold. "Fresh" seeds are pairs used for no bound: 83003 + 37k against 89009 + 41k (k = 0–15, or k = 0–7 for the
any-target arms). "Corrections" are steady Skips, Stalls and hard resyncs.

- **`ThirtyFpsOnHoldsTheBand`** (report §4 item 1).
  - On, the eight pairs: **0 corrections** on all 16 clients, and the guard binds on every steady frame (3,600 of 3,600).
    The leads are 3.31–3.46, mean **3.376**, against Off's 4.000 / 3.500 (mean 3.750). The pair's p95 is 0.179–**0.262**.
  - The 16 fresh pairs: 0 corrections on 32 clients, p95 max 0.234, mean lead 3.378.
  - **The p95 bound 0.30** is derived from all 24 pairs: their p95 has mean 0.207 and standard deviation 0.027, so
    mean + 3σ = 0.287. 0.30 is about 3.5σ; a bound at the largest measured value, 0.262, would have been fitted to the seeds.
  - Off and Shadow: 0 corrections on the eight pairs and on the fresh 16. The case allows **one steady Stall per client**
    because the clock is unchanged by this initiative and occasionally Stalls at 30 fps in Off: the task 7 review's round 3
    found 1 steady Off Stall on 1 of 16 fresh pairs (61604/67642, lead 4.707). That is the lead's 02:44 ruling (L4). It
    also asks that On's corrections stay ≤ Off's: 0 ≤ 0 here.
  - Shadow's lead equals Off's on every client (Shadow never moves the door). Its would-be guard binds on every frame too,
    as On's does.
- **`TheGuardHoldsAnyTarget`** (report §4 item 2). On the two pairs, all 24 runs give 0 corrections; on 8 fresh pairs per
  arm, all 96 runs give 0. With the guard as task 7 wrote it, the same arms gave, summed over the two pairs: at 30 fps
  47 / 110 / 183 / 183 corrections for the offsets −0.5 / −1 / −2 / −3 and 183 / 84 for `Common` 0.5 / 1.5; at 100 fps
  0 / 18 / 40 / 40 and 40 / 12.
- **`OtherFrameRatesHoldTheBand`** (report §4 item 3). 45, 60 and 144 fps: 0 corrections over the eight pairs and over the
  16 fresh pairs (as written, 45 fps gave 14 and 36). The mean leads are 2.690 (45 fps), 2.407 (60) and 2.369 (144).
  **20 fps is recorded, not asserted:** On gives 4 steady corrections over the eight pairs and 5 over the fresh 16, against
  Off's 5 and 16, and as written 419 and 846. Below 20 fps the band cannot be held by this guard (the longest sampled frame
  is `kMaxFrameTicks` = 3 ticks, 20 fps).
- **`FrameJitterLeavesOnNoWorseThanOff`** (report §4 item 4). The harness's `frameJitter` (`docs/ClockLoopHarness-rationale.md`
  §2) at 0.2, 30 fps: On gives 2 corrections over the eight pairs (one client: a Skip and a Stall), Off 9; on the fresh 16,
  On 1, Off 12. As written, On gave 153 and 314. The case also asserts Off ≥ 1, so a knob that stopped jittering would fail
  it (at j = 0 Off has none).
- **The 100 fps cases (report §4 item 5).** Every task 7 case passes. Its numbers against task 7's: (a) 0.0766 (equal);
  40 pairs p95 max 0.1442 (equal), median **0.0844** (0.0841), \|mean\| max **0.0451** (0.0448), mean of means −0.0031
  (−0.0028); the review's 24 pairs max 0.2158 (equal); (c) mean +0.0204 (equal), p95 **0.0553** (0.0552), max 0.1102;
  (d) all five seeds identical to four decimals; (e) [0.9982, 1.0003], p99 0.0400 %; (f) 0 / 0. The largest move is 0.0003.
  19 of task 7's 72 100 fps scenarios have byte-identical traces (door scale, would-be scale, steps, corrections and the
  pair samples); the other 53 differ, which research topic `thirty-fps-guard` attributes to the frame-aware edge
  (c − 0.2 at 100 fps) binding briefly on some calls.

### §3.2 The step lag's cases (task 10b)

Measured by a hidden survey test, added and removed while the task was open, against the header with the lag and against
the header before it (og-simulation `docs/ClockDilationController-rationale.md` §18 has the full tables).

- **`ExecutionLeadMeetsTheTarget`** (report §6 AC2). The bounds are the report's, derived from the mechanism: a client
  parked at one frame-grid phase holds an execution lead within ±g/2 of its setpoint, g = 0.2 tick at 100 fps and 1 tick
  at 30 fps, plus the report's 0.05 margin; the mean over clients is unbiased over the phase. Five seed sets of
  20 clients (7001 + i, the case's, and 7101 + i, 7201 + i, 9001 + i, 9101 + i, used for no bound):
  - with the lag, the set means are −0.003 … +0.003 at 100 fps and −0.003 … +0.000 at 30 fps; per-client standard deviation
    0.008–0.013, the 100 clients within −0.033 … +0.028. The mean bounds are at least 11× (100 fps) and 33× (30 fps) the
    set mean's standard error (the largest per-client sd over √20: 0.0027 and 0.0030); the client bounds are 4.5× and 17×
    the largest client offset. With ±20 % jitter the frame grid's
    phase moves every frame, so the stair is averaged away and every client sits near 0;
  - before the lag (the seen-failing arm, §4): −0.305 at 100 fps and −1.012 at 30 fps for the case's set, −0.300 … −0.306
    and −1.012 … −1.015 over the five, which is the mean step lag itself, 0.3 and 1.0 times 1.013 (the ±20 % jitter's
    long-frame weighting, og-simulation rationale §3.1). The 30 fps arm's guard binds on every steady frame, so it checks the
    lag against the clamped target.
- **The 100 fps and 30 fps cases of tasks 7 and 7b** pass with the lag, every assertion unchanged. The numbers that moved
  (before → with the lag): (a) 1/101 p95 0.0766 → 0.0768; 40 pairs median 0.0844 → 0.0839, max 0.1442 → 0.1443, mean of
  means −0.0031 → −0.0028, \|mean\| max 0.0451 → 0.0449; (c) mean +0.0204 → +0.0203, p95 0.0553 → 0.0549, max
  0.110 → 0.102; (d) the residuals of §3; 30 fps On, 8 pairs, mean lead 3.376 → 3.409, p95 max 0.262 → 0.205, 0
  corrections; any target 0 corrections. No assertion's level had to move: the cases compare two clients, count
  corrections or bound an error against the controller's own target.
- **The drain order (F-b).** The harness's default delivers every message at the frame's start, before the stage; an
  Unreal client drains at each step's start, after it (`docs/ClockLoopHarness-rationale.md` §2). With `drainAtStepStart`,
  task 7b's two guard cases fail with the header before the lag and with it:
  - `ThirtyFpsOnHoldsTheBand`'s On arm: 92 Skips on all 16 clients before the lag, 31 on 8 of 16 with it (Off: 0 either
    way); the pair p95 reaches 0.913 / 0.847;
  - `TheGuardHoldsAnyTarget`: at 30 fps every arm Skips (9–111 per arm with the lag), at 100 fps the offsets −1, −2, −3 and
    `Common` 0.5 (5–18).

  task 10b's brief ruled this out of scope ("do not retune the guard"): the guard cases kept the frame-start order, and
  `StepStartDrainLetsOnSkipAtThirtyFps_Finding` pinned the 30 fps arm in the Unreal order for the lead. Task 10c fixed it
  and retired that case (§3.3).

### §3.3 The clock's read and the change hysteresis (task 10c)

Measured on the tree that holds both changes by a hidden survey test built and removed inside one build lock hold; the
full tables are og-simulation `docs/ClockDilationController-rationale.md` §19. "Held out" = 16 pairs 83003 + 37k against
89009 + 41k, and 5,800 hitch seeds 3201–9000, used for no bound except where said.

- **The guard cases in both orders** (report §7 AC3). `ThirtyFpsOnHoldsTheBand`: 0 corrections in both orders, pair p95
  0.122–0.222 (step-start) and 0.131–0.222 (frame-start), mean lead 3.409 in both, every client's lead below its Off lead
  (Off 3.750). `TheGuardHoldsAnyTarget`, `OtherFrameRatesHoldTheBand`: 0 in every run of both orders.
  `FrameJitterLeavesOnNoWorseThanOff`: On 2 against Off 9 in both. With the stage's own A (the seen-failing arm, §4) the
  step-start order gives 31 Skips on 8 of 16 clients, 345 / 59 any-target corrections and 50 jittered.
- **The 30 fps guard-bound floor.** The research report's AC asked for ≥ 512 bound frames per client: that is the smallest
  value over the eight pairs (512 frame-start, 513 step-start; mean 1,876, standard deviation 1,086, maximum 3,196), so a
  bound there is fitted to the seeds, and the 16 held-out pairs reach **338** (step-start) and 340. The spread is too wide
  for a 3σ rule (mean − 3σ < 0). The case asserts ≥ **180**, 5 % of the 3,600 steady frames: about 1.9× under the
  lowest of the 96 clients measured. It still fails a guard that never binds, the 30 fps failure task 7b fixed.
- **`TheGuardCentreIsTheClocksRead`** (AC3). Cell means −0.006 … +0.001 on b0 and −0.040 … 0.000 on the clean link in both
  orders (every client of a cell within 0.006 of its cell's mean on b0, 0.003 on the clean link); held out, b0 at 30 / 60 /
  100 / 144 fps −0.001 / −0.001 / −0.007 / −0.000, clean 100 fps −0.040. As before: −2.000 / −1.000 / −0.847 / −0.725 on
  b0 in the step-start order. The ±0.05 is the AC's; the closest cell, clean 100 fps, is 0.010 inside it in both orders and
  on the held-out pairs.
- **`QuantumChangesStayUnderTheBound`** (AC4). Measured mean (worst client): 100 fps 0.29 (0.35), 60 fps 0.28 (0.37),
  144 fps 0.27 (0.35), 30 fps 0.99 (1.70), loopback 0.08 (0.16); held out 0.27 (0.37), 0.27 (0.34), 0.28 (0.40),
  1.00 (1.80), 0.07 (0.17). The bounds are the AC's, about 1.5–1.7× the measured values and inside G4's 2/s; the closest
  is the 30 fps worst held-out client, 1.80 against 2.0, which is G4's own bound, not a tolerance. With the rule inert
  (`changeHysteresisQuanta` 0.5): 3.37 (3.70), 3.32, 3.40, 7.89 (12.69), 0.80.
- **The hitch's law tolerance, 0.07 → 0.09.** The hysteresis holds the scale within ±0.04 tick of ε (og-simulation
  rationale §6), which biases the law check: over the 186 held-out seeds with a residual ≥ 0.5 the difference has mean
  −0.014 and standard deviation 0.025 (range −0.079 … +0.062), and 2 of them lie beyond the earlier ±0.07 (lead ruling
  02:44 L1, then 3.1σ of a zero-mean spread). |mean| + 3σ = 0.089, so the case asserts ±0.09. **New seeds (report §8 F5):**
  the first three from 3001 up whose residual lands ≥ 0.5, 3035, 3048 and 3156 (residuals 1.02 / 0.97 / 1.09, differences
  +0.000 / −0.027 / −0.065), beside 2001 (0.886, −0.036); the other four seeds land under 0.5 (0.039–0.064). The case also
  counts the law checks (8), so a change that moved every residual under 0.5 would not pass silently. From 10 s after the
  last correction the eight seeds reach 0.051–0.090; over the held-out seeds 2 of 5,800 exceed the 0.13 bound (max 0.147).
- **The other cases, now in the step-start order** (AC5). Every assertion passes unchanged. The numbers that moved, against
  task 10b's frame-start ones: (a) 1/101 p95 0.0768 → 0.0791; 40 pairs median 0.0839 → 0.0909, max 0.1443 → 0.1446, mean of
  means −0.0028 → −0.0057, \|mean\| max 0.0449 → 0.0447; (c) mean +0.0203 → +0.0208, p95 0.0549 → 0.0612, max 0.102 →
  0.105; (e) steady p99 0.03 / 0.02 %; (f) 0 / 0; the execution-lead set's mean offset −0.0027 → −0.0007 (100 fps) and
  +0.0008 → +0.0051 (30 fps), the latter 0.004, more than the report's inferred 0.002 and 20× inside the case's ±0.10.
  `WouldChangesCountsTheControllersScaleChanges` and `ShadowNeverMovesTheDoor` pass unchanged (Shadow's would-be changes
  on pair 1/101: 0.20 and 0.18 per second).

## §4 Seen failing

Each probe was a temporary edit, applied and restored inside one build lock hold, restored byte-identical (sha256 checked):

| probe | result |
|---|---|
| (a) run on the Off arm (`ClockDilationMode::Off` in `TwoClientsHoldEqualDeadlineLeads`) | `CHECK( run.p95AbsDiff <= 0.10 )`: `0.5 <= 0.1`, mean difference 0.5 |
| (c)'s On assertions run on the Off arm | `CHECK( graduatedAfter(world.client(index), from) == 0u )`: `3 == 0` (and the error checks see no dilated frame) |
| (f) with three 1.25 → 0.75 steps forced in `RenderTimeStaysBracketedUnderDilation` | `CHECK( render.beyondOldest == 0u )`: `1 == 0` and `2 == 0`; `CHECK( render.renderTimeBacks == 0u )`: `3 == 0` on both clients |
| the stage's `if (out.apply)` removed (og-simulation `ClockDilationController.h`, guard G-05) | `ShadowNeverMovesTheDoor`: `CHECK( frame.rateScale == 1.0 )`: `0.9982 == 1.0`, 7,926 failed assertions |
| (d)'s law with K_p halved (`0.5 * defaults.gainPerTick`) | `CHECK( std::abs(meanInWindow - lawAtMid) <= kLawToleranceTicks )`: `0.2083`, `0.2305`, `0.2507 <= 0.06` (seeds 1, 2001, 2022, the three with a residual ≥ 0.5); task 7's run, at its tolerance 0.06, and all three are as far outside 0.07 |
| (a)'s 40-seed case on the Off arm | `CHECK( std::abs(run.meanDiff) <= 0.10 )`: `0.5 <= 0.1` on 36 seeds, `1.5` on 2, `1.4353` and `0.458` on one each; `CHECK( std::abs(meanOfMeans) <= 0.02 )`: `0.3277 <= 0.02` |
| task 7b: the guard's low edge as task 7 wrote it (og-simulation `ClockDilationController.h`: `const double low  = m_guardTicks + -1.5;`) | 4 of 4 new cases red, 72 assertions. `ThirtyFpsOnHoldsTheBand`: `CHECK( steady.skips == 0u )`: `4 == 0` … `8 == 0` on all 16 On clients, `CHECK( steady.guardBoundFrames >= 1u )`: `0 >= 1` on all 16, `CHECK( run.p95AbsDiff <= kThirtyFpsPairP95Ticks )`: `0.6576` … `0.9136 <= 0.3` on all 8 pairs, `CHECK( onCorrections <= offCorrections )`: `92 <= 0`. `TheGuardHoldsAnyTarget`: `CHECK( run.graduated == 0u )` on 22 of 24 runs, `5` … `93 == 0`, including 10 of the 12 100 fps runs (`5` … `23`; every arm but the offset −0.5). `OtherFrameRatesHoldTheBand`: `1` … `4 == 0` on 8 of the 24 runs (45 fps). `FrameJitterLeavesOnNoWorseThanOff`: `153 <= 9` |
| task 10b: the og-simulation header as it was before the step lag (the test's `stepLagOf` reads 0, the field did not exist) | `ExecutionLeadMeetsTheTarget` red, 42 assertions: `CHECK( std::abs(set.meanOffset) <= arm.meanToleranceTicks )`: `-0.304783619` at 100 fps and `-1.0123481073` at 30 fps; every client outside its bound at 30 fps (`-1.030` … `-0.992` against ±0.55) and at 100 fps (`-0.332` … `-0.285` against ±0.15) |
| task 10c: the guard sample as before (og-simulation `ClockDilationController.h`: `guardSample` returns the stage's `O − (Ŝ − A)`) | 6 of this file's cases red, 71 assertions (and the controller unit case's 14), every one in the step-start order but one. `ThirtyFpsOnHoldsTheBand`: `CHECK( steady.skips == 0u )` `3 == 0` … `5 == 0` on 8 clients, the pair p95 `0.6754` … `0.8450 <= 0.3` on all 8 pairs, `CHECK( onCorrections <= offCorrections )` `31 <= 0`. `TheGuardHoldsAnyTarget`: 20 runs, at 30 fps 4 … 58 corrections per run (9 / 20 / 108 / 114 and 87 / 16 per arm), at 100 fps 2 … 11 (0 / 5 / 19 / 18 and 19 / 0). `FrameJitterLeavesOnNoWorseThanOff`: `50 <= 9`. `TheGuardCentreIsTheClocksRead`: `-2.0001`, `-1.0001`, `-0.8474`, `-0.7255` (b0) and `-2.0000`, `-1.0000`, `-0.8006`, `-0.7089` (clean) against ±0.05, and in the frame-start order `0.0726` at 144 fps. `SixHundredMillisecondHitchRecovers`: `CHECK( residual < saturationTicks )` `2.685` … `4.884 < 2.0` on all eight seeds, the law `0.092` … `0.728 <= 0.09`, `CHECK( lawChecks == 2u * kHitchLawSeeds )` `12 == 8`. `QuantumChangesStayUnderTheBound`: 30 fps `2.13` … `2.31 <= 2.0` on 6 clients, the mean `1.6833 <= 1.3` |
| task 10c: `changeHysteresisQuanta = 0.5` (the rule inert) | `QuantumChangesStayUnderTheBound` red, 69 assertions: every b0 client at 100 / 60 / 144 fps `3.06` … `3.76 <= 0.6` and at 30 fps `3.48` … `12.69 <= 2.0`; the means `3.3661`, `3.3167`, `3.4026 <= 0.45`, 30 fps `7.8932 <= 1.3` (worst client 12.69), loopback `0.8036 <= 0.3` |
| task 7b: `guardLowTicks = -1.0` with the headroom `static_assert` disabled (`true || …`) | `ThirtyFpsOnHoldsTheBand` red, 17 assertions: `CHECK( steady.skips == 0u )`: `3 == 0` … `5 == 0` on 8 clients, `CHECK( run.p95AbsDiff <= kThirtyFpsPairP95Ticks )`: `0.6818` … `0.8465 <= 0.3` on all 8 pairs, `CHECK( onCorrections <= offCorrections )`: `32 <= 0`. With the `static_assert` in place the value does not compile (og-simulation `docs/ClockDilationController-rationale.md` §12) |
