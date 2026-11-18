<!-- SPDX-License-Identifier: MPL-2.0 -->
# `SimTrigTest.cpp` — rationale

Source: `SimTrigTest.cpp`, tags `[SimTrig]` and `[SimTrig][Jolt]`. It tests og-simulation's `simTrig`
(`OGSimulation/SimTrig.h`, og-input-latency task 22), the trigonometry the simulation uses so that every platform
computes the same angles and rotations. The file sits under `Jolt/` because its first case compares `simTrig`
with Jolt's own trigonometry, and only that directory links Jolt.

This file has no guards document: it states no prohibition.

<!-- lint-external-ref: Jolt/Math/Trigonometry.h -- Jolt Physics v5.6.0, vendored by og-simulation-jolt under ThirdParty/, which the lint does not index -->

---

## §1 The cases

| case | what it shows |
|---|---|
| `SimTrig.MatchesJoltBitForBit` | each of `sin`, `cos`, `sinCos`, `tan`, `asin`, `acos`, `atan` and `atan2` returns the bits of the Jolt function it was ported from (`Jolt/Math/Trigonometry.h`), over a sparse sweep of float bit patterns (every 997th up to ±1000 for the angles, every 401st up to ±1.5 for the inverse sine and cosine, so clamped inputs too, every 10,007th over the whole float range for `atan`) and 200,000 random `atan2` pairs with zeros, negative zeros and equal arguments mixed in |
| `SimTrig.RotateAndOrientedAngleEqualGlmWhereTheTrigAgrees` | `simTrig::rotate` and `simTrig::orientedAngle` are glm's `rotate` and `orientedAngle` with only the trigonometry replaced: on every random case where the C library and `simTrig` return the same cosine and sine (or the same `acos`), the two give the same bits |
| `SimTrig.AccuracyAgainstDoublePrecision` | the maximum absolute error against double-precision `std::` functions stays below a bound per function (1e-7 for `sin` and `cos` up to ±100, 2e-7 for `asin` and `atan`, 5e-7 for `acos`, 4e-7 for `atan2`) |
| `SimTrig.InverseTrigClampsInputsOutsideTheUnitInterval` | `asin` and `acos` of a value outside [−1, 1] return the value at ±1, not NaN |
| `SimTrig.RecordedOutputsAreFixed` | eleven outputs keep the bits recorded on x64 at task 22 (the arm64 probe gave the same bits for the nine of them it covered), so a change that moves `simTrig` and Jolt together, such as a Jolt upgrade, still fails a case |

## §2 Why the comparison with Jolt is the reference

`simTrig` is a port: each operation is Jolt's, in Jolt's order. A reordered expression, a changed constant or a
dropped branch changes some outputs' bits, and the first case finds them. The probe that measured the port
(task 22's notes) compared 3.1 × 10⁹ evaluations; this case compares about 2 × 10⁷, a fraction of a second.

## §3 Why the accuracy bounds are what they are

Each bound sits above the maximum the task 22 probe measured over a much denser sweep of the same or a wider
range: `sin` and `cos` 7.8e-8 (up to ±1000), `asin` 1.7e-7, `acos` 3.3e-7, `atan` 1.4e-7, `atan2` 2.8e-7. The case
samples other points (other strides, another random seed), so the bounds keep a margin rather than repeat the
probe's maxima.
