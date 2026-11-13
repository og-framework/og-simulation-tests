<!-- SPDX-License-Identifier: MPL-2.0 -->
# `GlmFieldSimilarityTest.cpp` — rationale

Source: `GlmFieldSimilarityTest.cpp`, tags `[Network][GlmFieldSimilarity]`. It pins og-input-latency task 2e: the
correction comparator compares `glm::vec3`, `glm::vec2` and `glm::quat` fields with the epsilon overloads in
og-simulation's `SimulationComparisonGlm.h`, not with an exact `==`. It also pins task 2g: the quaternion rule is the
vectors' rule on four components after sign alignment, and a glm matrix field is rejected at compile time. The comparator's own rationale is og-simulation's
`SimulationComparisonGlm-rationale.md` and `SimulationTypes-rationale.md`.

This file has no guards document: it states no prohibition.

---

## §1 The compile-time pins

- **ADL alone finds each overload.** `glmFieldSimilarityAdlOnly` declares an `isSimilarToField` that takes a private
  type (`Hider`). It hides every outer declaration from ordinary lookup, so in `FoundByAdlAlone<T>` only
  argument-dependent lookup can supply a two-argument candidate. The comparator's dependent call reaches the glm
  overloads only that way under `/permissive-`, so the three `static_assert`s fail if the overloads leave
  `namespace glm`. A fourth asserts the concept is false for `float`, which has no associated namespace, so the three
  cannot pass vacuously.
- **Why not a deleted hider.** The usual form, `void isSimilarToField() = delete;`, does not work here: with it, MSVC
  (`/std:c++20 /permissive-`) evaluates the concept as false even when the overloads are in `namespace glm`, while an
  ordinary function template in the same namespace does find them. Measured in task 2e; the non-deleted hider is true
  with the overloads in `namespace glm` and false with them in the global namespace.
- **The generic template's guard recognises the glm types, and only them.** `simulationTypesDetail::GlmFloatType`
  must hold for `glm::vec3`, `glm::vec2`, `glm::quat`, `glm::vec4` and `glm::dvec3`, and (since task 2g) for
  `glm::mat3`, `glm::mat4`, `glm::mat2x3` and `glm::dmat3`. It must not hold for `float`, `std::int32_t`, `glm::ivec3`,
  an aggregate, `std::array<float, 3>` (out of the pin's scope: it compares exactly) or `std::vector<float>` (it has its
  own overload). If the concept stopped matching, the `static_assert` in the generic `isSimilarToField` could never
  fire, and a glm field without an overload would compare exactly again without a compile error.
- **A `glm::mat3` field trips the pin.** That is a compile failure, so it is checked by a probe outside the suite
  (og-input-latency task 2g): a `Serializable` aggregate with a `glm::mat3` field, compared with
  `fieldwiseIsSimilarTo`, fails with the generic template's `static_assert`, and compiles with the field removed.

## §2 The runtime cases

`ProbeState` is a `Serializable` aggregate carrying the shipped `LinearBodyState` (two `glm::vec3`), a `glm::vec2` and a
`glm::quat`. Its baseline (`slopeIdle`) is task 2c's diverging character: the slope spawn point (13.0513, 5.2657,
−16.7586) and a near-zero velocity of 1e-9 scale.

`verdictOf` asks the same question three ways, each one a layer the production verdict goes through:
1. `fieldwiseIsSimilarTo` on the aggregate;
2. `SimulationComposite::isSimilarTo` on a one-element composite (the `SimulationStateComposite` alias);
3. `StateCorrectionCache::tryInsertingCorrectState` with the verdict out-pointer: the call that decides whether a
   resim runs. The case also reads the slot back: an agreeing correction leaves the prediction in place, a disagreeing
   one overwrites it with the authority's state.

The cases:
- a 1-ULP difference (`std::nextafter`) on a `position` component, on every `linearVelocity` component, on a `glm::vec2`
  component and on the quaternion's `w`: similar;
- `linearVelocity` 4.76837e-07 apart, the largest delta task 2c measured: similar;
- 5e-5 apart (inside the 1e-4 epsilon): similar; 2e-4 apart on `linearVelocity`, `position` or the `glm::vec2`:
  dissimilar;
- the quaternion rotated by a further 0.05 rad about its axis: dissimilar.

The quaternion cases of task 2g, through the same three layers:
- 1 ULP on every component: similar;
- `q` against `-q` (the same rotation), and `-q` with every component 1 ULP up: similar;
- rotated by a further 1e-4 rad: similar; by 5e-4 rad: dissimilar;
- rotated by a further 0.01 rad: **dissimilar since task 2g.** Under task 2e's dot-product rule, which forgave up to
  about 0.028 rad, this case was similar; the case was kept and its expectation inverted.

**The angle bounds** (task 2g), on the leaf `isSimilarToField` directly. og-simulation's
`SimulationComparisonGlm-rationale.md` §2 derives them: a rotation by θ moves the four components by `2·sin(θ/4) ≈ θ/2`
in length, and the largest component by between θ/4 and θ/2, so with the 1e-4 epsilon the rule is always similar below
2e-4 rad and never similar from 4e-4 rad. The case checks:
- the rationale's numbers, in `double`: `2·asin(1e-4)` ≈ 2.0e-4 rad ≈ 0.011459°, 4e-4 rad ≈ 0.022918°, and the old
  `2·acos(1 − 1e-4)` ≈ 0.028285 rad ≈ 1.6206°;
- from the identity, about each axis: that axis's component is `sin(θ/2)`; 0.95 × 2e-4 rad is similar and 1.05 × 2e-4
  rad is not;
- over 6 bases and 5 axes (30 pairs), both argument orders: 0.95 × 2e-4 rad is always similar and 1.05 × 4e-4 rad
  never;
- the base `(½, ½, ½, ½)·conj(x)`, where a rotation about x moves all four components by θ/4: 0.95 × 4e-4 rad is still
  similar there (and not from the identity), and 1.05 × 4e-4 rad is not. So the 4e-4 rad end is reached.

`leafIsSimilarToAsFieldwiseDoes`, the leaf shim the divergence walk uses, must give the same answers for the glm types
as the fold, so its case repeats the 1-ULP and above-epsilon checks through it, and for the quaternion `q` against `-q`
and a 5e-4 rad rotation.

## §3 Seen failing (task 2e)

- With the `glm::vec3` overload's body replaced by `return a == b;`, the 1-ULP and inside-epsilon `glm::vec3` cases
  fail.
- With the overloads moved back to the global namespace, the build fails: the `FoundByAdlAlone` `static_assert`s here,
  and the generic template's `static_assert` in every translation unit that compares a glm field.

## §4 Seen failing (task 2g)

- With task 2e's dot-product rule restored, the 5e-4 rad and 0.01 rad cases fail, as do the angle-bound case's
  dissimilar checks (in the identity, grid and even-spread sections) and the shim's 5e-4 rad check.
- With the sign alignment dropped, the two `-q` cases and the shim's `-q` check fail.
- With the concept's member-`x` requirement restored, this file does not compile: the matrix `static_assert` fires.
