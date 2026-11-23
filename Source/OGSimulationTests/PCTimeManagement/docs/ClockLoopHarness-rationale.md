<!-- SPDX-License-Identifier: MPL-2.0 -->
# `ClockLoopHarness.h` — rationale

Source: `ClockLoopHarness.h`, test-only, used by `ClockDilationLoopTest.cpp`. It closes og-simulation's client clock loop
without an engine or a network: a 60 Hz server on its own clock, and per client a real `SimulationScheduler`,
`NetworkTimeEstimator` and `ClientPredictionClock` on `TimeConfig` defaults, a `TickRateDoor` over the scheduler, and the
per-frame stage `clockDilation::beforeClientPump` (og-simulation `docs/ClockDilationController-rationale.md` §13). Added by
og-time-dilation task 7 (2026-11-21); task 7b (2026-11-21) added the frame-jitter knob (§1); task 10b (2026-11-21) added
the step-start drain order (§2) and the per-step execution-lead record (§4); task 10c (2026-11-21) made the step-start
order the default and added the per-frame read centre (§4); task 14 (2026-11-22) added the uniform lag (§3).

This file has no guards document: it states no prohibition. Everything is deterministic: one SplitMix64 stream per client,
seeded from `ClientSpec::seed`.

---

## §1 Time

- **True time** is the server's clock, in seconds. Server tick n's step **starts** at `(n − serverTickAtZero) / 60` and is
  **published** `serverStepSeconds` later (default 0). `Server::trueTickAt(t)` is the step-start timeline, the one a host's
  before-step stamp gives; `publishedTickAt(t)` is what a pong carries (§3).
- **A client's clock** is `clockOffsetSeconds + (1 + clockRatePpm · 1e-6) · t`. Frames, pings and the SimClock stamps t0 and
  t3 are on it; `clockNowSeconds` passed to the stage is the frame's time on it.
- **A client's host time** (the scheduler's time base) starts at the first frame's clock time and advances by each frame's
  clock delta clamped to 0.4 s (`kMaxUndilatedFrameSeconds`, the engine's undilated frame clamp). The excess is counted in
  `lostHostSeconds()`. Without a hitch host time is the client clock minus a constant.
- **Frames** run every `1 / framesPerSecond` of client clock. A `Hitch` makes the first frame at or after `atSeconds` (true
  time) last `seconds` instead.
- **Frame jitter.** With `ClientSpec::frameJitter` = j > 0, each non-hitch frame lasts `(1 / framesPerSecond) · (1 + j · U)`,
  U uniform in [−1, 1) drawn from the client's own stream after the frame's link draws (research topic `thirty-fps-guard`'s
  probe model, unchanged). At j = 0 (the default) nothing is drawn and the frame period is exactly the one before the knob
  existed: task 7b fingerprinted 194 scenarios (every frame's times, lead, scales and step count, every correction and every
  pair sample) before and after adding it, and all 194 were identical. The knob models a frame-paced host only roughly: the
  engine's real `t.MaxFPS` pacing was not measured.
- **Long frames** (og-time-dilation task 7c). With `ClientSpec::longFrames.fromSeconds` ≥ 0, the first frame at or after
  that true time, and every later one at or after the next due time, lasts `minSeconds + (maxSeconds − minSeconds) · U`
  instead of the frame period; a run of `burstFrames` (default 1) such frames follows back to back, and after the run the
  next one is due `everySeconds · (0.5 + U)` later, so runs land at a mean interval of `everySeconds` and never on a fixed
  frame phase. With `everySeconds` = 0 every frame from `fromSeconds` on is long: a sustained frame-rate drop when
  `minSeconds = maxSeconds`. The draws U come from a second stream of the client's own, `SplitMix64(seed ^
  kLongFrameSeedSalt)`, so the link and frame-jitter draws are the same with and without long frames; a hitch frame takes
  precedence. `longFrameCount()` counts the long frames. At the default `fromSeconds` = −1 nothing is drawn and every
  frame period is the one before the knob existed, by construction; every older `[ClockDilationLoop]` case passes
  unchanged. It models a host that stalls now and then for a few tens of milliseconds (G-DIL-1's F2: 22.9–38.2 ms frames,
  with up to 3 multi-step frames within 8 frames), not the engine's own pacing after such a frame, which was not measured.

`World::runUntil` runs every client's frames in true-time order, and records both clients' true deadline leads every 10 ms of
true time (`PairSample`), so two clients are compared at one instant.

## §2 The frame

In order, as the tick driver runs a client frame:

1. Generate the authority messages published up to now; deliver every message (authority tick or pong) that has arrived, in
   arrival order: `recordAuthorityTick`, `noteSimClockSample`.
2. Advance host time; send a ping if one is due (20 Hz on the client clock, at most one per frame: a hitch skips pings).
3. Build the door (`TickRateDoor(scheduler, tickOffset, false, refused)`); record the true deadline lead (`trueLeadTicks`:
   the door's deadline tick minus `trueTickAt(now)`) and the sensor's error (`serverTickAt(clockNow) − trueTickAt(now)`).
4. Unless forced scales are scripted: call the stage when the mode is not `Off`; log and restart the window when due.
5. `pump(hostNow)`; per step, `advancePrediction()` once, then the driver's tick offset becomes
   `getPredictionTick() − physicsStep`, as the driver's after-tick hook publishes it. Non-Normal advances are recorded
   (`CorrectionEvent`, with whether a re-centre debt was being paid).
6. With `render`: one snapshot per step at its deadline into a `SnapshotChannel<RenderSnapshotT<1>, 4>`, then one
   `interpolateRenderPoses` per frame at `hostNow − 2 · dt / rateScale`, the presenter's render time.

**The drain order.** An immediate Unreal client, the default one, drains its messages at each step's start (og-tickdriver's
`receiveForStep` hook), after the stage: every step of a frame runs at the frame's time, so the steps see the same messages
in either order, but the stage sees them one call later, and a frame with no step drains nothing.
**`ClientSpec::drainAtStepStart`** runs that order and is **true by default** since task 10c: step 1 then delivers nothing,
and each step of step 5 first delivers everything that has arrived by the frame's time. Set false, messages are delivered at
the frame's start (step 1), before the stage, which is the order of a non-immediate Unreal client (it drains at the game
thread's net dispatch, before the tick driver's `beforePump`; that this dispatch precedes `beforePump` in the frame is
inferred from where the engine ticks its net driver, not measured). With false the frame is exactly the one before the knob
existed. Tasks 7 to 10b ran their cases in that order; with the stage's own A, task 7b's guard cases failed in the
step-start order (`ClockDilationLoopTest-rationale.md` §3.2). Task 10c's guard reads the clock's published read and runs
every guard case in both orders (§3.3 there).

## §3 The network

- **Links** (`Link`): one-way delay `baseSeconds` + an exponential tail of mean `jitterMeanSeconds` (b0-like defaults 28 ms
  + 7.5 ms, so R ≈ 71 ms), optional loss and bursts (`burstExtraSeconds` on a fraction of messages).
- **A uniform lag** (og-time-dilation task 14): `uniformWidthSeconds` > 0 adds `uniformWidthSeconds · U(0, 1)` to each
  message, so `baseSeconds` 15 ms and a width of 35 ms is og-net's shaper b0 (`lagMinMs` 15, `lagMaxMs` 50 each way, og-net
  `EmulatorProfile.h`), whose tail is bounded, unlike the exponential one. The draw is taken only when the width is set, so
  every link without it draws the same sequence as before and every older case's numbers are unchanged.
- **In-order lanes.** Each lane (authority ticks down, pings up, pongs down) delivers in order: a message's arrival is the
  later of its own draw and the previous message's arrival.
- **Authority ticks**: one message per server tick published after the client's join.
- **SimClock**: the server answers a ping 50 µs after it arrives (`kTurnaroundSeconds`) with the tick published at that
  instant and its age since publication, rounded to a microsecond as the pong's u32 carries it. The client builds the sample
  as og-sim-net does: `rtt = (t3 − t0) − (t2 − t1)`, `theta = ((t1 − t0) + (t2 − t3)) / 2`.
- **The join**: pings start 0.3 s before the client's first frame (the connection's clock exchange runs before the world),
  and their pongs are delivered at the first frame, ahead of the first authority tick. So the join's hard resync already has
  an RTT reading and takes no second resync, as an engine host that holds its first sample does.

## §4 What it records

`frames()` (one `FrameRecord` per frame: times, true lead, sensor error and validity, the stage's output, the clock's
published phase, the door's scale, the step count, and on a frame that steps its **read centre** `readCentreTicks`: the
clock's published target after the frame's first step minus `serverTickAt(clockNow)` after the frame's steps, the
instantaneous centre the guard's filtered one is compared with; NaN on a 0-step frame), `steps()` (one `StepRecord` per step: its true time, the frame's, its
**execution lead** `getPredictionTick() − trueTickAt(now)` after the advance, and the advance's result), `corrections()`, `windowLines()` (each with its true time),
`renderStats()`, `firstHardResyncSeconds()` and `joinSettledSeconds()`: the first frame after the first hard resync whose
phase shows no re-centre, debt or first-RTT wait. Tests take their steady state from it.

## §5 What it leaves out

Engine-owned threads (it runs the immediate client's order on one thread; a step's run time is its frame's, with no
delay from the stage to the step), packet sizes and bandwidth, the server's own
scheduler (the server steps exactly on its grid), the input path, and any frame-time jitter other than the `frameJitter`
knob's uniform model (§1). The step-start versus
publication offset is a parameter (`serverStepSeconds`), so a test can measure the truth convention.

**Frame rate.** `ClientSpec::framesPerSecond` is a knob. Task 7's criteria (a)–(g) are asserted at 100 fps; task 7b's
frame-aware guard is asserted at 30, 45, 60 and 144 fps and at 30 fps with 20 % frame jitter (`ClockDilationLoopTest-rationale.md`
§3.1). Before task 7b, On at 30 fps Skipped steadily (4–8 per client per 120 s, finding F4) because the guard's low edge
ignored the frame period (og-simulation `docs/ClockDilationController-rationale.md` §5). The harness's 30 fps Off landing
(4.000 / 3.500, the re-centre's integer decision at a frame phase that never moves) sits 1.1–1.6 ticks above the OwnRtt
target, larger than Ph5's measured +0.46 / +0.50, so the model may overstate the 30 fps lead cost of the guard (On parks at
the frame-aware edge, about 3.38 against Off's 3.75).
