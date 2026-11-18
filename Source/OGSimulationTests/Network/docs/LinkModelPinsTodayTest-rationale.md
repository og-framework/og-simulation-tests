<!-- SPDX-License-Identifier: MPL-2.0 -->
# `LinkModelPinsTodayTest.cpp` — rationale

Source: `LinkModelPinsTodayTest.cpp`, tags `[Network][LinkModel]`. It drives today's real input path, both
directions, through `LinkModel` (`docs/LinkModel-rationale.md`), and pins what that path does when one packet is
lost or reordered. The pins are **today's behaviour**, not a target: a later change that improves loss
robustness is expected to move them, and must move them on purpose.

This file has no guards document: it states no prohibition.

---

## §1 The session the cases share

`Session` runs one sending client, the server, and one peer, at a 60 Hz sim tick. Each server tick `t`, in
order:

1. The client captures tick `t` into its `PendingInputQueue`. On a send frame it builds a bundle and sends it
   on the uplink at `t`. A 30 fps client (`capturesPerSend = 2`) sends on even ticks; a 100 fps client
   (`capturesPerSend = 1`) sends every tick. Captures stop at tick 20; the loop runs to tick 40, so every
   parked input is released.
2. The uplink delivers what is due, and the server feeds each arrival to the coordinator's
   `receiveInputBundle`.
3. The server drains one tick: `releaseDelayedInputs(t, 1, …)`.
4. The relay stage is flushed into the replicated ring with `flushStagedInto`. When it published anything, the
   ring's bytes are sent on the downlink at `t`.
5. The downlink delivers what is due, and the peer ingests each arrival with `populateRemoteInputCache`.

Both links have a constant delay of one tick.

**The client builds bundles as production does.** `UplinkClient::buildAndRelease` is the engine-free
equivalent of `SimNetChannel::sendLocalInputToAuthority` followed by the release in `sendLocalInputToAuthorityAll`: the depth is
clamped to `kMaxSlots`, `forEachRecent` walks the newest entries into `appendSlot`, and then
`releaseAllButRecent` keeps the **unclamped** depth. The depth is `TimeConfig::redundancyDepthTicks`, 3 today;
case (i) asserts that.

**The server is the real coordinator.** `ServerReceptionCoordinator` over `FStandaloneTestHandle`, with a
default `TimeConfig`. No RTT sample is fed, so there is no tier entry and every input is parked for the fallback
delay, `rttTierInputDelays[kMaxConnectionTierIndex]` = 4 ticks (the relay floor is 0). The coordinator's relay
sink is the stage: every relayed input goes through `stageArrival`.

**"Parked" is measured as "released by the drain".** A capture tick that is never parked is never released,
and `neverReleased()` lists the capture ticks 1 to 20 that the drain never handed out. The undelayed fallback
sink must stay empty, and (i), (ii) and (iv) check that it does: no input takes the malformed-slot path.

## §2 The four pins

The bundle a 30 fps client sends at tick `2k` carries the three newest pending ticks, `2k-2`, `2k-1` and `2k`.
The bundle before it ends at `2k-2` and the bundle after it starts at `2k`. So **tick `2k-1` travels in exactly
one bundle**, and losing that bundle loses that tick. At 100 fps the bundle sent at `t` carries `t-2` to `t`,
so every tick travels in three consecutive bundles.

| case | link | pinned |
|---|---|---|
| (i) 30 fps, depth 3 | uplink loss 0.1, seed 8: send 4 is lost (the bundle sent at tick 10: 8, 9, 10) | `neverReleased() == {9}`; 19 released; no relay out-of-order skip, no late drop |
| (ii) 100 fps, depth 3 | uplink loss 0.1, seed 96: send 9 is lost (the bundle sent at tick 10) | nothing missing: 20 released |
| (iii) downlink | no uplink loss; downlink loss 0.1, seed 8: flush 4 is lost | the peer's store lacks exactly 9 and 10, and holds the other 18 |
| (iv) reorder | reorder 0.1, window 1, seed 29: send 4 (sent at tick 10) is held, so it arrives straight after send 5 (sent at tick 12) | every tick released once, with the first arrival's value; `relayOooSkipCount() == 1`; tick 9 is not relayed |

Each case also checks the link's own record (`lostSends`, `reorderedSends`), so a seed that no longer draws the
pattern the case was written for fails on that line first.

**(iii), why exactly two ticks.** A 30 fps client's bundle arrives every second tick and adds two new ticks, so
each published flush carries two entries: flush `j` carries ticks `2j+1` and `2j+2`. Flush 4 carries 9 and 10.
The flush publishes only what was staged since the last poll and keeps nothing (`flushStagedInto` empties the
stage), so nothing re-carries those entries. The model does not include a replication system re-sending a
property whose packet was lost: the next flush has already replaced the ring's contents, so it has nothing
older to re-send. The stored stamp `dA` is the server's delay at receipt, 4 (§1).

**(iv), three populations in one late bundle.** When the bundle sent at tick 10 (8, 9, 10) arrives at tick 13,
after the bundle sent at tick 12 (10, 11, 12):
- tick 8 was released at tick 12 (captured at 8, delay 4), so its re-send is a **late drop**:
  `lateDroppedCount() == 1`;
- tick 9 is new to the queue but older than the id's watermark (12): it is parked and released, and counted as
  a **relay out-of-order skip**, not relayed;
- tick 10 is still parked from the earlier arrival: a **duplicate**, and the first value stays.

To make first-writer-wins observable, (iv) stamps each arrival's ordinal into its inputs before the coordinator
sees them (`stampArrival`: the bundle is decoded with `forEachSlot` and re-encoded with `appendSlot`). A
production client never revises a sent tick, so without a stamp both copies would be identical. Every released
input carries the ordinal of the first arrival that carried its tick; tick 10's is the bundle sent at tick 12.

## §3 Seen failing

With the loss probability set to 0 in (i) and in (iii), both cases fail: (i) with `{  } == { 9 }` on
`neverReleased()` and (iii) with `{  } == { 9, 10 }` on the missing set, besides the link-record lines. Both
cases assert today's loss, not merely that the link dropped a packet.
