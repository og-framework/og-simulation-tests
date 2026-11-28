<!-- SPDX-License-Identifier: MPL-2.0 -->
# `RedundancyWindowTest.cpp` — rationale

Source: `RedundancyWindowTest.cpp`, tags `[Network][RedundancyWindow]`. It drives the client's real input send,
`SimulationNetSync::sendLocalInputToAuthorityAll`, over `LinkModel` (`docs/LinkModel-rationale.md`) into a real
`ServerReceptionCoordinator`, and checks the redundancy window counted in frames (og-simulation
`docs/SimulationNetSync-rationale.md` §17).

This file has no guards document: it states no prohibition.

---

## §1 The rig

* **Client.** `ClientRig` holds a real storage, reconciliation, input-resolution peer and net-sync, and registers one
  local sender (`kSenderId`, 21) through the real `registerSimulatable` facade, so the sender map, the pending queue
  and the cadence record are production's. A capture is an `enqueue` straight into the sender's `PendingInputQueue`,
  found through `findPendingInputQueue`; the rig does not run a prediction step, because the send reads only the queue.
* **Owner.** `WinOwner::sendLocalInputToAuthority` does what the engine adapter's bundle builder does: it caps the
  scalar it is handed at `inputRedundancyBundle::kMaxSlots`, walks `forEachRecent` and appends each slot with the
  codec. It records the scalar (`slots`), the bytes and the slot ticks of every bundle.
* **Server.** `Server` feeds each delivered bundle to `receiveInputBundle` and drains one tick per server tick with
  `releaseDelayedInputs`. Its delay is set to 8 ticks (`kServerDelayTicks`, every `rttTierInputDelays` entry, so the
  no-tier fallback returns 8). §3 says why.
* **Session.** `runSession` takes a frame schedule (ticks stepped per frame). Each tick the client captures, sends if
  a frame ends on it (a 0-step frame sends again at the same tick), the uplink (seed 7, one-tick delay, no random loss)
  delivers, and the server drains. A lost bundle is modelled by not handing send number `droppedSend` to the link. The
  server runs 40 ticks past the last capture so every parked input is released.
* **Measures.** `neverArrived`: capture ticks no delivered bundle carried. `neverReleased`: capture ticks the server
  never released (the "never-parked" ticks). `retainedAfterSend`: entries left in the pending queue after each send.

## §2 The cases

1. **Bytes at 0 frames equal the pre-task path's.** Three sessions over one irregular schedule (1, 2, 0, 1, 3, 1, 0,
   2, 4, …, so 0-step frames and up to 4 steps per frame occur) at depths 3, 5 and 10. Each case pins the slot ticks
   of every bundle, the total byte count and an FNV-1a digest of all bytes. The values were recorded by running this
   case against the code before og-input-latency task 7 (the two-argument send), and the case passed there before the
   send changed. Depth 10 also checks that the queue keeps 10 entries, more than the wire's 8, as it did.
2. **`redundancySlots`' bounds.** At 0 and negative frames it returns the depth for every EMA and depth tried,
   including depths 0 and 10. At 1 to 12 frames it never exceeds `kMaxSlots` and never falls below
   min(depth, `kMaxSlots`). Spot values: 3 frames at 1, 2, 3 and 4 ticks per send give 3, 6, 8, 8; 1.9 and 2.1 both
   round to 6.
3. **1 to 4 steps per frame at `redundancyFrames = 3`.** Schedule: one 1-step frame, then 14 frames of k steps. The
   clean run checks every bundle's `slots` (the depth on the first send, then max(depth, min(3k, 8))), the queue
   retention after each send (min(`slots`, captured)), and the cap. Then each send except the last is dropped in
   turn: no tick may be never-arrived or never-released, and nothing may stay pending.
4. **0 frames, two steps per frame (task 3 case (i)).** Dropping any interior send loses exactly the middle tick of
   that bundle (its send tick minus one), both as never-arrived and never-released.
5. **The log line and re-registration.** A sender at two ticks per send logs
   `[SendLocalInputToServer] id=21 tick=6 depth=3 slots=6`; the same tick at 0 frames logs `depth=3 slots=3`. After
   `unregisterSimulatable` and a fresh `registerSimulatable`, the first two sends give 3 slots: the cadence started
   over. With the erase removed, both give 5: the old cadence (an EMA of about 1.8 ticks per send) carries over.

## §3 Why the server delay is 8

A tick recovered from the next bundle arrives one frame late. With k steps per frame and a one-tick link, the oldest
tick of a lost bundle reaches the server 2k ticks after its capture, and the delay queue parks it only if the delay is
at least 2k. Otherwise it is dropped as late, whatever the client sends. At 4 steps per frame that needs 8. The
default no-tier fallback is 4, which covers 1 and 2 steps per frame only. One delay for every k keeps the cases
comparable: they differ in the client's window alone. A case that wants to show the server's deadline is a different
test.

The delay queue also counts an arriving copy of an already-released tick as late (`lateDroppedCount()`), so the
cases do not check that counter: a window of 6 re-sends ticks the server has released, with no input lost.

## §4 Seen failing

* `kRedundancyFrames` set to 0 in case 3 (one replacement): sections "steps per frame 2", 3 and 4 fail and 1 passes
  (at one step per frame the depth of 3 already overlaps). At 4 steps per frame even the clean run fails, because a
  depth of 3 never sends one of each frame's 4 ticks.
* In `SimulationNetSync.h`, the release given `redundancyDepth` instead of `slots`: case 3 fails its retention check
  and loses ticks at every drop. The cadence erase removed from unregister step 2: case 5's two re-registration
  checks fail (`5 == 3`).
