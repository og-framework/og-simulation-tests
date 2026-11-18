<!-- SPDX-License-Identifier: MPL-2.0 -->
# `JoltSortTotalOrderTest.cpp` — rationale

Source: `JoltSortTotalOrderTest.cpp`, tags `[Jolt][StaticGeometry]`. It tests that the sorts in og-simulation-jolt
give the same order on every standard library (og-input-latency task 23). `std::sort` leaves elements that compare
equal in an order it does not specify, and MSVC's STL and libc++ need not agree, so a sort whose comparison can
return "equal" for two elements that differ can order them differently on Win64 and on Linux or Android.

This file has no guards document: it states no prohibition.

---

## §1 Which sorts have a case

Of the five sorts in og-simulation-jolt, only `JoltStaticWorldBuilder::build`'s element sort could tie two
different elements, and only it changed. Each source's rationale says why the other four cannot tie:

| Sort | Key | A tie between two different elements |
|---|---|---|
| `JoltStaticWorldBuilder::build` | layer key, surface, `stableKey`, content hash, then the content words | only through a 64-bit content-hash collision; the content words now break it (`JoltStaticWorldBuilder-rationale.md` §5) |
| `JoltSpatialQueryAdapter::collideAt` | hit key, depth, sub-shape id | none: the key and the sub-shape id are unique per hit (`JoltSpatialQueryAdapter-rationale.md` §3) |
| `JoltSpatialQueryAdapter::excludeStaticBodiesFromQueries` | the body index itself | none: equal indices are the same value (§8 of the same file) |
| `JoltContactCounter::drain` | simulatable id | none: each id fills one slot (`JoltContactCounter-rationale.md` §4) |
| `JoltLayerTable`'s constructor | the whole `JoltLayerKey` | none: equal keys are the same value (`JoltLayerTable-rationale.md` §3) |

## §2 The colliding pair

The case needs two elements that tie on everything before the content words: the same layer key, surface and
`stableKey`, and the same content hash while their contents differ. Both are 0.25 m boxes with an identity
rotation, categories `bit(kWorld)`, blocking `kAllMappedCategories`, friction 0.7, restitution 0.3 and one shared
`stableKey`. They differ only in the translation, whose bits were searched offline so that the two FNV-1a 64
content hashes are equal (`0xA07C98885AC3FE5D`):

| | x | y | z |
|---|---|---|---|
| `kFirst` | `0x3F98E296` (1.19441485) | `0x3FC37A19` (1.52716362) | `0x3FC149E0` (1.51006699) |
| `kSecond` | `0x3FDA4036` (1.70508456) | `0x3FFFC041` (1.99805462) | `0x3FDD5B80` (1.72935486) |

The search was a parallel distinguished-point birthday search over the 64 mantissa bits of the translation:
about 4.5 × 10⁹ hash evaluations. Every word after the translation is equal for the two boxes, and each FNV-1a
byte step is a bijection on the state, so equal states after the translation mean equal final hashes.

The case first checks its own fixture. `boxContentWords` lists the words `JoltStaticWorldBuilder`'s content hash
consumes for a box, in the same order: the shape's variant index, the half extents, the 16 matrix entries column by
column, both category sets, then friction and restitution. `fnv1a64OfWords` hashes them. The case requires that
the word lists differ, the hashes are equal, `kFirst`'s words compare lower, and the boxes are more than 0.1 m apart.
⚠ The two helpers are copies of the builder's private hash. If the builder's hash or its word order changes, the
pair no longer collides in the builder, and the case would pass with or without the tie-break; the copies have to
change with it, and a new pair has to be searched.

## §3 What the case checks

It builds the pair in both description orders (`kFirst, kSecond` and the reverse), each in a fresh `JoltWorld`
with `maxShapesPerChunk = 1`, so each element becomes its own static body and the bodies are created in sorted
order. In both builds the first body's world bounds must be centred on `kFirst` and the second's on `kSecond`
(within 1 mm): the element with the lower content words comes first, whatever the description order.

**Seen failing (task 23, MSVC):** with the content tie-break replaced by `return false`, the case passed on the
first order and failed on the reversed one: both position checks read `0.72843f < 0.001f`, the distance between the
two boxes, so `kSecond` was built first. With the tie left alone the two elements kept their description order.
