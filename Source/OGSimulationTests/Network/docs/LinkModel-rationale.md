<!-- SPDX-License-Identifier: MPL-2.0 -->
# `LinkModel.h` — rationale

Source: `LinkModel.h`, a header-only test helper: a seeded, tick-domain model of one direction of a lossy link.
It carries byte vectors only, so the suites that use it drive the real codecs and the real receive paths
through it with no engine present. Everything is in namespace `linkModel`.

This file has no guards document. Its two contracts are `static_assert`s (§5), so nothing in it needs a `⛔`
tag.

---

## §1 The API

| member | does |
|---|---|
| `linkModel::Config` | `seed`, `lossProbability`, `baseDelayTicks`, `jitterShape` + `jitterSpreadTicks` (+ `twoPointHighProbability`), `reorderProbability` + `reorderWindowPackets` |
| `send(payload, nowTick)` | takes one packet. It is either lost (recorded in `lostSends`) or put in flight, due at `nowTick + baseDelayTicks + jitter` |
| `deliverDue(nowTick, callback)` | calls `callback(const Payload&)` for every packet that is deliverable at `nowTick`, in delivery order, and returns how many it delivered |
| `sentCount`, `deliveredCount`, `inFlightCount` | counters |
| `lostSends`, `reorderedSends` | the 0-based send indices that were lost, and that were held for reordering |

`Payload` is `std::vector<std::uint8_t>` and `Tick` is `double`. A tick may be fractional: a delay of 1.5 ticks
sent at tick 10 is due at 11.5, so a caller polling on whole ticks receives it at tick 12.

## §2 Jitter

`JitterShape::Uniform` adds `u * jitterSpreadTicks`, with `u` uniform in [0, 1), so the delay lies in
[`baseDelayTicks`, `baseDelayTicks + jitterSpreadTicks`). `JitterShape::TwoPoint` adds either nothing or the whole
`jitterSpreadTicks`, the latter with probability `twoPointHighProbability`.

## §3 Delivery order and the reorder window

A packet is deliverable when its due tick is at or before `nowTick`. Among the deliverable packets, the one
with the smallest due tick goes first; a tie goes to the earlier send. So jitter alone can reorder packets.

The reorder window reorders on purpose. On each send, with probability `reorderProbability` (and only when
`reorderWindowPackets` is above 0), the packet is HELD. A held packet becomes deliverable only once its due tick
has passed **and** `reorderWindowPackets` packets sent after it have been delivered. From then on it competes by
due tick like any other packet, in the same `deliverDue` call. With a window of 1 and a constant delay its due tick
is older than its successor's, so it arrives immediately after its successor.

A held packet that never sees enough later deliveries stays in flight (`inFlightCount`). A caller that ends its
run with a held packet in flight must send more packets to release it.

## §4 Seeding: why the helper has its own generator

The suites pin exact outcomes ("send 4 is lost") as literals. That needs a sequence of draws that is the same on
every compiler and standard library. The `std::` distributions do not promise that: their algorithms are
implementation-defined. So the helper carries `SplitMix64`, a fixed 64-bit generator, and turns a draw into a
unit double as `(next() >> 11) * 2^-53`, which is exact in any IEEE double arithmetic.

There are three independent streams, one per feature, each seeded with `seed` XOR its own salt
(`kLossStreamSalt`, `kJitterStreamSalt`, `kReorderStreamSalt`). Every `send` makes exactly one draw from each
stream, in the order loss, jitter, reorder, and it makes all three even for a packet that is lost. So the loss
pattern of a seed depends only on the seed, the loss probability and the number of sends. Turning jitter or
reordering on or off does not move it.

To choose a seed for a pinned pattern, replay the same arithmetic off-line: splitmix64 with the state
initialized to `seed ^ salt`, the same unit conversion, and an event when the draw is below the probability.
The `static_assert` on `SplitMix64(0u).next()` pins the generator to the reference splitmix64 sequence, so an
edit that changes the generator fails to compile instead of silently moving every seeded pin.

## §5 What the two `static_assert`s prove, and what they cannot

The task asked for a compile-time proof that the helper names no UE or Jolt symbol, with includes from the STL
and `OGSimulation/` only. Two asserts carry it.

1. **The surface.** `Payload` is `std::vector<std::uint8_t>`, `Tick` is `double`, and `send` takes exactly those.
   An engine container or an engine time type in the interface fails to compile. Witnessed: with `Tick` changed
   to `float` the build fails with `error C2338` on this assert.
2. **The include set, for Jolt.** The header records whether `JPH_VERSION_MAJOR` (defined by Jolt's core header)
   is defined before its includes and after them, and asserts that the includes did not change it. Witnessed:
   with `#include "Jolt/Jolt.h"` added among the includes, the build fails with `error C2338` on this assert.

**The limit, for UE.** The UBT test target force-includes the Core module's shared precompiled header into every
translation unit, before the first line of this file. Every UE Core name is therefore already declared when
the header starts, and no assertion inside it can tell a Core name it uses from one it does not. The UE half of
the requirement is carried by the surface assert and by the include list itself, which is STL only: the helper
includes no `OGSimulation/` header, because it needs none.
