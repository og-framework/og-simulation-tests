<!-- SPDX-License-Identifier: MPL-2.0 -->
# `ServerTickEstimatorTest.cpp` — rationale

Source: `ServerTickEstimatorTest.cpp`, tags `[PCTM][Dilation][ServerTickEstimator]`. It tests og-simulation's
`ServerTickEstimator` (og-simulation `docs/ServerTickEstimator-rationale.md`) on synthetic clock ping/pong traces whose truth
is known exactly. Added by og-time-dilation task 3 (2026-11-20); its acceptance criteria are named (i)–(vii) below.
Case (viii), the estimator inside `NetworkTimeEstimator`, is in `NetworkTimeEstimatorTest.cpp`
(`PCTM.NetworkTimeEstimator.ServerTick.*`).

This file has no guards document: it states no prohibition.

---

## §1 The rig

- **`SplitMix64`** is written out (the reference splitmix64 step), so every trace is the same on every compiler and
  standard library; `std::uniform_real_distribution` is not specified bit for bit.
- **`Trace`** is one scenario: the seed, the duration, uniform uplink and downlink delays, the client's clock rate against
  the server's, a locked pong phase, a server step loss, loss and burst probabilities, and where evaluation starts (10 s
  after the first ping).
- **`World`** draws the trace. The server's clock reads `kServerClockAheadSeconds` (123.456 s) ahead of the client's, and the
  client's clock runs `clientRate` times as fast as the server's; its continuous tick is `serverSeconds · 60 + origin` (origin a random 1000–2000 ticks),
  minus the step loss after it happens. A ping leaves the client every 1/20 s (og-sim-net's SimClock rate); the server
  answers 1 ms after receipt with the tick it last published, `floor` of its continuous tick, and that tick's age; the pong
  arrives one downlink later. Per ping the RNG is drawn in one fixed order (up, down, burst, loss) whether or not the ping
  is lost. The samples are then sorted by arrival, as a drain would hand them over, so reordered pongs arrive reordered.
- **`Feeder`** passes each sample to the estimator with the smoothed RTT of a real `NetworkTimeEstimator` fed that sample
  first through `updateRTT` (the order `noteSimClockSample` uses), so the gate sees the RTT a client would.
- **`Errors`** records, at each sample's arrival once the estimate is valid and evaluation has started, `serverTickAt(t3) −
  truth(t3)` and the phase-blind equivalent; p95 is nearest-rank over |error|.

`noteExact` feeds hand-built samples (t3 on a 1/16 s grid, so time sums are exact) for the unit cases.

## §2 The cases

| case | trace | asserts |
|---|---|---|
| (i) `SymmetricJitterWithTheTrueAgeIsUnbiased` | seed 11, 600 s, u and d each U(15, 50) ms | \|mean\| ≤ 0.01, p95 ≤ **0.07**, no re-seed |
| (ii) `PhaseLockedPingsNeedTheAge` | 30/30 ms, every pong at phase 0.9 | max \|error\| ≤ 0.05; phase-blind mean ≤ −0.3 and \|error\| ≥ 0.3 on every sample |
| (iii) `AnAsymmetricLinkReadsHalfTheAsymmetry` | u = 50, d = 20 ms | mean 0.90 ± 0.05, every sample within 0.05 of (u − d)/2 · f |
| (iv) `AHundredPpmClockRateMismatchStaysWithinTheLag` | 25/25 ms, 600 s, client rate 1 ± 1e-4 | max \|error\| ≤ 0.06 |
| (v) `AServerStepLossReSeedsWithinFiveSamples` | 30/30 ms, the server loses 12 ticks at 30 s | no re-seed and error 12 for samples 1–4 after it; a re-seed at the 5th; \|error\| ≤ 0.1 at the first sample 0.5 s later |
| (vi) `LossAndDelayBurstsAreGatedAndCounted` | (i)'s link, 5 % loss, a 200 ms downlink delay on 10 % of samples | p95 ≤ 0.08; rejected ≥ 90 % of the burst samples delivered and ≤ those plus 1 % of all; accepted + rejected = delivered |
| (vii) `ValidAfterTwentySamplesAndStaleOneSecondAfterTheLast` | `noteExact` | invalid at 19, valid at 20, valid exactly 1 s after the last sample and not 1/1024 s more; a rejected sample neither counts nor refreshes |
| `IgnoredSamplesMoveNothing` | the none tick, a NaN or infinite input, a negative age, a wrapped u32 age (2³² − 1 µs) and an age of exactly `kMaxServerTickAgeSeconds` | nothing moves; a smoothed RTT of 0 disables the gate |
| `OnlyAConsecutiveRunOfJumpsReSeeds` | 4 jumps, 1 in-band sample, 4 jumps, 1 jump | no re-seed until the 5th consecutive jump; then ô moves by the jump and validity restarts |
| `AnOutOfOrderSampleHasNoWeight` | a sample 0.25 s before the last | ô unmoved, counted |

**Why these traces.** (iv) and (v) use constant delays so they measure the drift and the re-seed alone: with b0 jitter a
re-seed lands on one sample's 0.43-tick error (the estimator's rationale §8, limit 2), so (v)'s 0.1 tick at 0.5 s holds only
without jitter. (vi)'s bursts are single samples: the acceptance criterion names no burst length, and a run of several 200 ms
samples raises the smoothed RTT the gate reads (limit 3).

**(i)'s bound.** (i) asserts p95 ≤ **0.07** tick, the initiative lead's ruling of 2026-11-21, which replaced the first
criterion's 0.05 and kept τ at 5 s. The reason: under the estimator's compiled τ = 5 s the stationary error is a standard
deviation of 0.030 tick, and the p95 of |error| is about 1.96 σ = 0.059; this trace measured **0.063**. The bound is not
tuned by seed: every case asserts its criterion's number, and the first seed tried was kept for every case.

## §3 Measured (2026-11-20, Development build, `-s`)

(i) p95 0.0630, mean −0.0024, 11,800 points; (ii) max 1.8e-12, phase-blind mean −0.400; (iii) mean 0.900; (iv) max 0.0300
(+100 ppm) and 0.0300 (−100 ppm); (v) error 12 for samples 1–4, 0 from the 5th; (vi) p95 0.0742, 11,421 delivered,
1,155 burst samples, 1,155 rejected, no re-seed.

**Seen failing** (the probe edit in `ServerTickEstimator.h`, built and run, then restored byte for byte):
- (i) with `halfRtt = rttSeconds` instead of `rttSeconds / 2.0`: mean 2.054, p95 2.118 (`CHECK( std::abs(errors.mean()) <=
  0.01 )` and `CHECK( errors.absP95() <= 0.07 )` FAILED);
- (v) with the re-seed disabled (`if (++jumpRun < config.reseedRunSamples || true)`): `REQUIRE( estimator.reseeds() == 1u )`
  FAILED at the 5th sample, `0 == 1`, error 12.
