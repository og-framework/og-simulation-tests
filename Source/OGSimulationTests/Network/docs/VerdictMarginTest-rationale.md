<!-- SPDX-License-Identifier: MPL-2.0 -->
# `VerdictMarginTest.cpp` — rationale

Source: `VerdictMarginTest.cpp`, tags `[Network][VerdictMargin]`. It pins og-input-latency task 2h, the verdict-margin
diagnostic: how close each landed correction verdict came to the tolerance (`measureVerdictMargin` in og-simulation's
`SimulationComparison.h`, the ratio overloads in `SimulationComparisonGlm.h`, the call site in `CorrectionCache.h` and
the `[DivergenceProbe.Margin]` line in `NetSyncTelemetry.h`), and task 2i's agreeing max on that line. Their own rationale is og-simulation's
`SimulationComparison-rationale.md` §8, `SimulationComparisonGlm-rationale.md` §6, `CorrectionCache-rationale.md` §14
and `Telemetry-rationale.md` §15.

This file has no guards document: it states no prohibition.

---

## §1 The mock state

`verdictMarginMock::MotionState` holds one leaf of each kind the walk distinguishes: an enum (`phase`), an integer
(`counter`), a float (`timer`), a nested `Serializable` aggregate that ships in og-simulation (`LinearBodyState`, two
`glm::vec3`), a `glm::vec2` (`aim`), a `glm::quat` (`facing`) and a `std::vector<float>` (`trail`, through
`SIM_VECTOR`). `PoolState` is a second composite element with a member `isSimilarTo`, the shape of the brawler's
projectile pool. The mocks live in a named namespace so the walk's path reads `verdictMarginMock::MotionState…`, as
`CorrectionFieldDivergenceTest.cpp` explains for its own mocks.

Every verdict goes through the real `StateCorrectionCache::tryInsertingCorrectState` with an observer, so the test
reads what production reads: the `CorrectionInsertVerdict`'s `predictionWasCorrect` and `verdictMargin`.

## §2 What each case pins

- **The buckets** (the task's AC): no offset gives b0; 5e-10 on a zero velocity component gives b1; 5e-8 b2; 5e-6 b3;
  5e-5, ratio 0.5, b4; 2e-4 b5, and that one disagrees. Two more checks: the largest field wins and names itself, and a
  ratio of 0 names no field.
- **Discrete leaves:** an enum, an integer (with a float offset beside it that sets the bucket), a vector's size, and an
  element its own `isSimilarTo` decides all set `discrete` and stay out of the ratio.
- **A `std::vector<float>` field compares exactly today.** A 5e-8 offset on one element disagrees. The fold never sees
  the vector overload, so it reaches the generic `a == b` (og-simulation's `SimulationComparison-rationale.md` §8). The
  case pins what the walk reports for it, `discrete`. If that lookup is ever fixed, this case fails, and should then
  expect an agreeing verdict with the element outside the ratio.
- **glm leaves:** `q` against `−q` gives 0; a 1e-4 rad rotation is below 1 and names `facing`; 5e-4 rad is b5; a vec2
  offset of 5e-5 is b4.
- **Agreement with the fold:** over 9 offsets (0 to 2e-4) on 6 fields, a verdict agrees exactly when
  `maxRatio < 1` and no discrete leaf disagreed. The grid includes both outcomes (the counts are checked), so the
  property is not vacuous.
- **The gate** (the task's AC): over ten landed corrections (five of each verdict) and two discards: unset, no walk;
  a predicate returning false is consulted ten times and walks nothing; true walks ten times, which is what stops the
  zeros being a dead instrument; true with no observer walks nothing; and the margin gate does not open the field diff.
- **Verdicts identical with the gate on and off** (the task's AC): one ten-correction sequence, run with the predicate
  false and then true. The verdicts, the pending resim anchor after each correction, the landing stamp and every slot's
  state compare equal; the sequence has three disagreements.
- **The line:** both classes, the buckets, `exactFail`, `discrete`, the max and its field, the agreeing max and its
  field, quoted whole; `verdicts` equals the Window line's `corrections`; the next window starts empty and prints
  `maxRatio=0 maxField=- maxAgreeRatio=0 maxAgreeField=-`; with the gate off the Window line prints and no Margin line
  does.
- **The agreeing max** (og-input-latency task 2i's AC): one window holds an agreement on `timer` at r ≈ 0.5, a smaller
  agreement on the velocity, a float disagreement on the velocity at r = 2, and a discrete disagreement (`counter`)
  with the velocity at r = 0.9. `maxField` names the velocity at 2 and `maxAgreeField` names `timer` at 0.5. The 0.9
  verdict is below 1 but disagrees, so it shows that agreement is read from the verdict, not from `r < 1`. A second
  window holds only that 0.9 disagreement among zeros, and prints `maxAgreeRatio=0 maxAgreeField=-` beside a non-zero
  `maxRatio`. The case first checks the ratios' order and each verdict's outcome, so the line checks are not vacuous.
- **Length:** a maximal line, with a 95-character path for both the overall and the agreeing max, holds the whole
  overall path and ends with the whole agreeing path inside its 512-byte buffer; the widest format is checked to be
  exactly 456 characters, so more than 256 (SIMLOG's buffer, which would have cut it) and less than 512.
- **The buckets' edges and a non-finite leaf:** each boundary value, and an infinite `timer` delta, which disagrees and
  lands in b5.

## §3 Seen failing (og-input-latency task 2h)

Each arm: one counted replacement in `CorrectionCache.h`, build, run `[VerdictMargin]`, restore the bytes with a new
mtime, rebuild, run again. The quoted results are in the task's impl notes.

1. **The walk skips agreeing verdicts** (the diff's `!predictionWasCorrect` added to the margin gate): the bucket case
   fails for b0 to b4.
2. **The gate dropped** (the walk runs whatever the predicate says): the gate case fails in its unset and false arms.

The compile-time pins were also checked by a `cl` build outside UBT before the first UBT build.

**og-input-latency task 2i:** one counted replacement in `NetSyncTelemetry.h`, the agreeing max taken over all
verdicts (`agreed && ` made always true): the agreeing-max case fails in both windows (it names the velocity at 2, and
the 0.9 disagreement in the second), and so do the line and length cases.
