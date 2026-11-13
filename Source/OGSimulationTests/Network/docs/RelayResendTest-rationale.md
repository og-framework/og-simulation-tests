<!-- SPDX-License-Identifier: MPL-2.0 -->
# `RelayResendTest.cpp` — rationale

Source: `RelayResendTest.cpp`, tags `[Network][RelayResend]`. It tests the relay flush's re-send of the last
flushed entries (`relayedInputRing::flushStagedInto`'s `resendEntries`, og-input-latency task 8), on the real
codec and the real client-side ingest, through `LinkModel` (`docs/LinkModel-rationale.md`). The codec's own
rationale is og-simulation's `RelayedInputRingCodec-rationale.md` §15.

This file has no guards document: it states no prohibition.

---

## §1 The downlink the cases share

`runDownlink` models the server's relay host and one peer, one server frame per tick. Each frame, in order:

1. The relay tap stages `capturesPerFlush` new capture ticks with `stageArrival`, as the coordinator's relay
   sink does, until tick `lastCaptureTick`. The ticks increase by one, so each frame's arrivals are newer than
   everything flushed before, as in production.
2. `flushStagedInto(ring, stage, resendEntries)` publishes the round. When it published anything, the ring's
   bytes go out on the link, except for the one flush a case chose to drop (`droppedFlush`).
3. The link delivers what is due, and the peer ingests each arrival with `populateRemoteInputCache`, once or,
   with `deliverTwice`, twice.

The loop runs four frames past the last capture with an empty stage, then drains the link. A flush is dropped
by not sending it rather than through the link's loss draw, so "any single lost flush" can be enumerated.
`capturesPerFlush = 1` is a 60 Hz sender against the 60 Hz server; `capturesPerFlush = 2` is a 30 frames per
second sender, whose two captures per packet the server stages in one frame (the shape task 3's case (iii)
pins).

`ResendInput` carries two `int32` fields, so one entry is 13 bytes (`captureTick`, `dA`, the input).

## §2 The cases

- **The byte pin.** `runPinRounds` stages a fixed sequence of rounds (1, 2, 0, 3, 0, 0, 9, 1, 2, 0, 1 arrivals,
  with varying `dA`; the 9 overflows the stage) and records every flush's return value and the ring's bytes
  after it. The expected vectors were captured by running this same sequence against the codec **before**
  task 8 changed it. At `resendEntries = 0`, through the two-argument call, an explicit 0 and a negative value
  (which clamps to 0), the trace must be identical. This is the default's equivalence: the ring bytes on the
  wire at the shipped config do not move.
- **Recovery at r = 1.** For every flush but the last (the last has no later flush to carry its tail), drop
  that one flush: the peer's store must still hold all 20 ticks. Its constant `kRecoveryResendEntries` set to
  0 makes this case fail (the seen-failing run of task 8).
- **Today at r = 0.** The same drop at r = 0 leaves exactly the dropped flush's ticks missing, at one and at
  two captures per flush: task 3's case (iii), restated over this harness.
- **Two entries per flush.** At r = 1 a dropped two-entry flush gets back its newer tick only; at r = 2 it is
  recovered whole. A re-sent entry is one entry, not one flush.
- **Random loss.** At 30 % seeded loss over 60 flushes (fewer than the store's 64 slots, so no tick is evicted
  by a later one), a tick is missing at r = 1 exactly when its flush
  and the next one were both lost (or its flush was the last and was lost); at r = 0 every lost flush loses its
  tick. That is the loss → loss² claim, counted.
- **Duplicates and reordering.** At r = 1 and 2, with one and two captures per flush, a link with jitter and a
  reorder window, delivering every payload twice, leaves the store with the same resident count and, tick by
  tick, the same `dA` and input as a clean in-order run. The store's push is last-wins and a re-sent entry
  carries the bytes it was first published with, so ingest is idempotent.
- **The bound.** For every previous ring size 1–8, every staged count 1–9 and requests −3, 0, 1, 2, 3, 99,
  the flush publishes `staged + min(clampResendEntries(request), kMaxDepth − staged, previous)` entries, never
  more than `kMaxDepth`, in exactly that many strides, and `isAcceptableWireLength` accepts the result.
- **The tail.** The re-sent entries are the newest of the ring the last flush published, laid out oldest
  first before the staged ones. A staged capture tick is not also taken from the tail (the staged value wins),
  and a ring that is never-written or carries another wire version yields no tail.
- **The empty stage.** After flushes with a re-send, five empty frames return 0 and leave the ring
  byte-identical: suppress-clear-on-empty holds at r > 0.

A file-scope `static_assert` holds the codec's `relayedInputRing::kMaxResendEntries` equal to the
`TimeConfig` knob's `latencyKnobs::kMaxRelayResendEntries`.
