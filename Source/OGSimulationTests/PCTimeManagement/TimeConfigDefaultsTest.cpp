// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

// ---------------------------------------------------------------------------
// TimeConfig default-drift gate (R-P1) + R-D3 ordering sanity test.
//
// These two tests guard the WHOLE TimeConfig struct against silent default
// drift and lock in the bounded-depth ordering invariant.
//
//   * Default-drift gate (risks_and_plan §6.2): asserts every TimeConfig field
//     default matches the synthesis-recommended initial value. If anyone edits
//     a default without updating the configurability surface (the R-P1 lint
//     BLACKLIST + this test), this test fails loudly. One REQUIRE per field.
//
//   * R-D3 ordering sanity (proposal §4.2): the HardResync failsafe backstop
//     must fire strictly LATER than the RollbackWindow soft cap, i.e.
//     `hardResyncThresholdTicks > rollbackWindowHardCap`. With T1's values
//     (21 > 20) this passes by a margin of 1.
//
// Stage hand-off note: `redundancyDepthTicks == 3` is the ratified 60 Hz target
// value (active since Stage 2 / T15). The historical Stage 1 interim value was 5
// (matched the 100 Hz runtime). Stage 2 task T15 flipped both the TimeConfig.h
// default AND this assertion 5 -> 3 in the same atomic change, when the runtime
// moved to the ratified 60 Hz target (proposal §11).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Default-drift gate — closes the default-drift surface for the whole struct.
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.TimeConfig.DefaultsMatchSynthesisRecommendation", "[PCTM][TimeConfig]")
{
    TimeConfig tc;

    // --- Network estimation (NetworkTimeEstimator) -------------------------
    REQUIRE(tc.rttSmoothingAlpha == 0.15);
    REQUIRE(tc.jitterSmoothingAlpha == 0.15);
    REQUIRE(tc.jitterMultiplier == 2.0);
    REQUIRE(tc.predOffsetFloorTicks == 4);

    // --- Outlier RTT rejection (T26b) --------------------------------------
    // The plausibility bound is `4.0 * smoothedRTT + 0.030 s`, with a 0.5 s
    // absolute ceiling standing in on the cold-start seed. `rttOutlierConsecutiveLimit`
    // is the escape hatch: 30 consecutive implausible samples are read as a
    // GENUINE step change and force a re-seed, which is what keeps the gate a
    // filter rather than a permanent lock. If any of these five ever changes,
    // the change must come WITH a focus-swap PIE trace showing what it bought —
    // they were sized against the ~8 s post-hitch offset transient, not derived.
    REQUIRE(tc.rttOutlierMultiplier == 4.0);
    REQUIRE(tc.rttOutlierMarginSeconds == 0.030);
    REQUIRE(tc.rttOutlierColdStartCeilingSeconds == 0.5);
    REQUIRE(tc.rttOutlierConsecutiveLimit == 30);
    REQUIRE(tc.rttOutlierLogWindowSamples == 600);

    // --- Drift correction (ClientPredictionClock) --------------------------
    REQUIRE(tc.softDriftThresholdTicks == 3);
    REQUIRE(tc.gradualCorrectionRate == 4);
    REQUIRE(tc.minTicksBeforeDriftCheck == 60);

    // --- Tick frequency ----------------------------------------------------
    // Struct default is the long-term 60 Hz target. The runtime overrides this
    // at session construction (config.tickFrequency = 1.0 / asyncDeltaTime);
    // T15 changes that async-DeltaTime setup, not this struct default — so this
    // assertion stays == 60.0 across Stage 1 and Stage 2.
    REQUIRE(tc.tickFrequency == 60.0);

    // --- Bounded-depth prediction (RollbackWindow) -------------------------
    REQUIRE(tc.rollbackWindowTicks == 12);
    REQUIRE(tc.rollbackWindowHardCap == 20);

    // --- HardResync failsafe backstop (bumped 15 -> 21 in T1) --------------
    REQUIRE(tc.hardResyncThresholdTicks == 21);

    // --- Input redundancy (FInputRedundancyBundle) -------------------------
    // 60 Hz ratified target (active default since Stage 2 / T15). The historical
    // 100 Hz interim value was 5; T15 flipped both this assertion and the
    // TimeConfig.h default to 3 in the same atomic change.
    REQUIRE(tc.redundancyDepthTicks == 3);

    // --- Outbound input relay (FRelayedInputRing) --------------------------
    // ⛔ THERE IS DELIBERATELY NO relay-ring retention-depth ASSERTION here,
    // because there is no such field any more (og-netcode-v2-input-relay item
    // 63 / RN-13, 2026-08-16: retired — its old identifier is on record in
    // RN-13, ReviewNotes.md). Item 34's bare-C1 flush-on-poll write path had
    // already made the field inert; its stage capacity is the compile-time
    // constant `relayedInputRing::kMaxDepth`, pinned (and machine-fenced
    // against ever becoming configurable again) by
    // `Network/RelayRedundancyDepthTest.cpp`.

    // [T11] The floor lever ships OFF. 0 is the degenerate value at which
    // `max(floor, tier-or-fallback)` is the identity, i.e. at which the whole
    // relay-delay-spectrum feature reproduces pre-T11 behaviour exactly. Sizing
    // (§3.3: ~7-8 to cover 80 ms) waits on playtest + the T9 cadence probe — if
    // this assertion ever fails, the change must come WITH that evidence.
    REQUIRE(tc.relayDelayFloorTicks == 0);

    // [T39] The correction-state rotation width. A DECIDED number, not a
    // placeholder. [T34] LOWERED 2 -> 1 FOR THE PRE-DIET WINDOW, and it goes back
    // to 2 in the same item-40 change that deletes `kPreDietCharacterCap`.
    //
    // The reason is an engine fork, not a byte budget: at K=2 with un-dieted 316 B
    // states and bare C1's variable-length rings, the SECOND state batch fails
    // inside Iris's huge-object window (`SplitHugeObject`, > 1,536 bits free) on
    // roughly a third of frames at four characters, which chunks it and blocks that
    // character's newer snapshots for ~1 RTT. At K=1 and N <= 4 the residual
    // failures land below the window => clean Abort => the state ships in packet 2
    // of the same tick (design_task38_input_first_replication.md §16.2).
    //
    // ⚠ THE COST, RECORDED: T39 chose 2 so that two-character sessions kept the
    // pre-T39 every-frame cadence and the archived two-character baselines stayed
    // comparable. At K=1 a two-character session corrects at 30 Hz, so that
    // comparison must now expect exactly half. If this assertion ever fails, the
    // change must come WITH the packet-budget arithmetic that justifies it — the
    // round-vs-packet LLT in og-brawler-tests is the fence, and its pre-diet table
    // is asserted at this same value.
    REQUIRE(tc.correctionRotationK == 1);

    // --- The resim gate (item 45) ------------------------------------------
    // ⛔ THE POLICY DEFAULT IS THE WHOLE OF ITEM 45's LANDING CONTRACT. The item
    // replaced the level-triggered resim gate with an edge-triggered one and shipped
    // it DEFAULTED TO REPRODUCE THE OLD OBSERVABLE BEHAVIOUR, so that nothing
    // changes by default and the mechanism can be validated separately from the
    // policy. Flipping this to `OnDisagreement` is backlog item 46 and is
    // HARD-BLOCKED on item 30 (a non-degenerate verdict): with today's
    // always-false verdict, "disagrees" is EVERY landing, which is the modelled
    // 3-6x sustained physics-cost storm of design §4.
    //
    // If this assertion ever fails, the change is item 46 and must come WITH the
    // pre-flip cost preview that item requires — not as a tidy-up.
    REQUIRE(tc.resimTriggerPolicy == TimeConfig::ResimTriggerPolicy::FrontierExact);

    // ⛔ AND THERE IS DELIBERATELY NO `resimCooldownTicks` ASSERTION, because there is
    // no such field. A trigger-rate ceiling was built and REMOVED on a user ruling
    // (2026-08-11): it defers acting on a correction already known to disagree, which
    // is the defect item 45 repairs. The throttle is structural instead — at most one
    // resim in flight and one pending. Backlog items 45/46 and design §4 still name
    // the knob; the ruling block on `TimeConfig::resimTriggerPolicy` is the authority.

    // --- Test harness mode selector ----------------------------------------
    REQUIRE(tc.harnessMode == TimeConfig::HarnessMode::Production);

    // --- C.2 tiered input delay (Stage 5) ----------------------------------
    // ⛔ THERE IS DELIBERATELY NO dedicated no-tier-baseline field ASSERTION
    // here, because there is no such field any more (og-netcode-v2-input-relay
    // item 62 / RN-12, 2026-08-16: retired — its old identifier is on record in
    // RN-12, ReviewNotes.md). The no-tier fallback is now
    // `rttTierInputDelays[kMaxConnectionTierIndex]` — pinned by the array
    // assertion below, and by name in RelayDelayFloorTest / ServerInputDelayQueueTest
    // / ReplicatedTierConsumptionTest.

    REQUIRE(tc.rttTierBoundariesMs[0] == 30);
    REQUIRE(tc.rttTierBoundariesMs[1] == 80);
    REQUIRE(tc.rttTierBoundariesMs[2] == 150);
    REQUIRE(tc.rttTierBoundariesMs[3] == 999);

    REQUIRE(tc.rttTierInputDelays[0] == 1);
    REQUIRE(tc.rttTierInputDelays[1] == 2);
    REQUIRE(tc.rttTierInputDelays[2] == 3);
    REQUIRE(tc.rttTierInputDelays[3] == 4);

    REQUIRE(tc.rttTierRollbackCeilings[0] == 6);
    REQUIRE(tc.rttTierRollbackCeilings[1] == 9);
    REQUIRE(tc.rttTierRollbackCeilings[2] == 12);
    REQUIRE(tc.rttTierRollbackCeilings[3] == 20);

    REQUIRE(tc.tierHysteresisMs == 10);
    REQUIRE(tc.tierMinDwellTicks == 60);

    // `muteEchoOnDegradedTier` has NO consumer this initiative (optional task
    // T15 owns the behaviour). The default is asserted anyway so that if T15
    // flips it, the flip is a deliberate, visible change rather than a silent
    // drift.
    REQUIRE(tc.muteEchoOnDegradedTier == true);
    REQUIRE(tc.lanZeroDelayOverride == false);

    // --- Stage 4 observability (fields only, no consumer yet) --------------
    REQUIRE(tc.sn1BroadcastPolicy == TimeConfig::Sn1BroadcastPolicy::RttTiered);
    REQUIRE(tc.sn1IdleBroadcastIntervalTicks == 6);
    REQUIRE(tc.hashBroadcastPolicy == TimeConfig::HashBroadcastPolicy::EveryTick);
    REQUIRE(tc.hashBroadcastIntervalTicks == 1);
    REQUIRE(tc.hashMismatchTickThreshold == 5);
    REQUIRE(tc.hashMismatchReaction == TimeConfig::HashMismatchReaction::LogOnly);
    REQUIRE(tc.sparseSaveMode == false);
    REQUIRE(tc.recordSkipEvents == true);
    REQUIRE(tc.recordStallEvents == true);
    REQUIRE(tc.recordSubstitutionEvents == true);
    REQUIRE(tc.recordRedundancyHits == false);
    REQUIRE(tc.aggregateSiblingInputBundles == false);
    REQUIRE(tc.hashLogRingCapacity == 600);
}

// NOTE: the R-D3 HardResync ordering invariant that previously lived here has
// MOVED to the sibling TimeConfigOrderingTest.cpp (D3.10 completion), which is
// the dedicated home for TimeConfig ordering/relational invariants. The test
// name, tags and assertion are preserved verbatim there — this is a relocation,
// not a removal, and the file was split so the ordering invariants sit together
// rather than being buried in the default-drift gate.

#endif // WITH_LOW_LEVEL_TESTS
