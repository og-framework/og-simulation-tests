// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionCache.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / item 45: THE EDGE-TRIGGERED RESIM GATE.
// (Was item 44's CHARACTERIZATION of the shipped level-triggered gate. Every case
//  below is either that case updated deliberately, or a new case item 45 requires;
//  the case-by-case ledger is in `impl/impl_notes_task45.md` and summarised here.)
//
// ── WHAT CHANGED, AND WHAT DID NOT ─────────────────────────────────────────
// The gate WAS level-triggered on per-slot bits: `getLastResimulationTick()`
// scanned newest->oldest STARTING AT THE FRONTIER and returned the first slot with
// `m_isResimulated || m_containsCorrectTick`, and `pushPredictionTick` copied the
// frontier's `m_isResimulated` bit forward so a completed resim stayed consumed.
// That inheritance was a LOAD-BEARING GUARD, not the bug: without it the
// just-replayed slot one below an unflagged frontier re-triggered a 1-tick resim
// every tick, forever (design §3 candidates A and B, both DEAD). The bug was the
// RE-OPEN condition — only a landing EXACTLY on the frontier slot cleared the
// shadowing bit, so triggers measured clock alignment (~2 events against ~8,759
// behind-frontier corrections).
//
// The gate IS NOW edge-triggered: a landed correction SETS a pending anchor tick
// when `resimGate::shouldSetPendingAnchor` says so, `needsResimulation()` is
// `anchor != 0 && anchor != frontier`, and the resim-completion edge CONSUMES the
// anchor with a CAS. `m_isResimulated`, the scan and the inheritance are retired,
// and the inheritance line was deleted BECAUSE ITS ONLY READER WAS GONE — never on
// its own.
//
// ⭐ THE ONE PROPERTY THAT MUST NOT MOVE: **a consumed correction must not
// re-trigger.** That is what the inheritance protected, it is now protected
// STRUCTURALLY (events set, completion consumes, no rescan of replayed state can
// re-open it), and `ACompletedResimClosesTheGateAndItStaysClosed` below is its
// tripwire. That case KEEPS ITS ITEM 44 NAME AND SHAPE on purpose: if it goes red,
// the fix reintroduced candidate A/B's storm and the fix is wrong — do not rewrite
// the case to accommodate it.
//
// ── HOW ITEM 44's EIGHT CASES WERE UPDATED ─────────────────────────────────
//   AnUncorrectedCacheHasNoAnchorAndNoGate              KEPT (accessor renamed)
//   ScanStartsAtTheFrontierAndAcceptsTheCorrectFlagThere RENAMED — there is no
//       scan; the surviving behaviour is "a frontier-exact landing anchors the
//       frontier and the gate stays shut until it moves".
//   AFrontierExactLandingReopensTheGateOneTickLater      KEPT verbatim in intent —
//       it is now the `FrontierExact` policy's defining case.
//   InheritanceCarriesTheResimFlagForward                REPLACED. It pinned the
//       inheritance as CORRECT-FOR-NOW; the guard is retired, so the case asserts
//       what replaced it (a consumed resim leaves NO anchor, and frontier advance
//       touches no gate state at all).
//   ACompletedResimClosesTheGateAndItStaysClosed         KEPT — the tripwire.
//   ABehindFrontierLandingIsShadowedByTheInheritedFlag_DocumentsADefect_…
//       REWRITTEN as `ABehindFrontierDisagreeingLandingSetsThePendingAnchor` (the
//       replacement the backlog names). The defect is fixed; the case now pins the
//       fix, plus the legacy policy's arm.
//   RepeatedBehindFrontierLandingsNeverReopenTheGate_DocumentsADefect_…
//       REWRITTEN as `RepeatedBehindFrontierLandingsCoalesceIntoOneNewestAnchor`.
//   WipeCacheClearsAllGateState                          KEPT (+ anchor assertion)
// New: the consume CAS both ways, the mid-replay landing, the policy-equivalence
// pair, and the verdict gate. The three POLICY predicates and the manager/config
// wiring are swept in `ResimGatePolicyTest.cpp` — same tags.
//
// ⛔ ITEM 44's REVIEW APPENDED TWO REQUIRED CHARACTERIZATION CASES, both present:
//   1. the legacy trigger is VERDICT-BLIND —
//      `AnAgreeingLandingIsIgnoredOnlyUnderOnDisagreement`'s `FrontierExact` section;
//   2. coalescing-to-newest under the LEGACY policy —
//      `RepeatedFrontierExactLandingsCoalesceIntoOneNewestAnchor` (added on the item
//      45 review; the `OnDisagreement` coalescing pair does not cover it, see that
//      case's own block for why).
//
// ⚠ THE DEFAULT POLICY IS `FrontierExact`, so a `landCorrection` behind the
// frontier sets NOTHING unless a case opts into `OnDisagreement`. That is not test
// scaffolding: it is the shipped configuration, and item 45's whole landing claim
// is that it reproduces the old observable behaviour.
//
// ── HOW THESE CASES ARE DRIVEN ─────────────────────────────────────────────
// A bare `StateCorrectionCache`, exactly as the neighbouring `[CorrectionCache]`
// suites drive it, with helpers that mirror the four production lifecycle sites
// (predict / land / prepare / replay+finish) so a case cannot accidentally model a
// sequence production cannot produce. NO PRODUCTION CODE WAS ADDED to make this
// file possible; the gate's whole surface is public because production reads it.
//
// Tagged `[CorrectionCache][ResimGate]` — unchanged from item 44, so the same
// isolation command still recovers this suite's exact contribution.
//////////////////////////////////////////////////////////////////////////////

namespace
{
	struct GateState
	{
		std::int32_t value = 0;

		// The cache compares prediction against correction on the hit path.
		bool isSimilarTo(const GateState& other) const { return value == other.value; }
	};

	struct GateInput
	{
		std::int32_t value = 0;
	};

	using GateCache = StateCorrectionCache<GateState, GateInput>;

	// A value no predicted tick below ever uses, so a correction carrying it lands
	// with `predictionWasCorrect == false`. That is the LIVE regime, not a
	// contrivance: the verdict is degenerately always-wrong today (item 30), which
	// is exactly why `OnDisagreement` is not the shipped default.
	constexpr std::int32_t kAuthorityValue   = 4242;
	constexpr std::int32_t kResimulatedValue = 7777;

	// One predicted tick, exactly as collectInputAll + postPredictionAll produce
	// it on the client.
	void predictTick(GateCache& cache, std::uint32_t tick, std::int32_t value)
	{
		cache.pushPredictionTick(tick);
		cache.pushPredictionState(GateState{ value });
	}

	// A landed, DISAGREEING correction for `tick` — the OnRep path
	// (`SimulationReconciliation::injectCorrectionState`).
	void landCorrection(GateCache& cache, std::uint32_t tick)
	{
		cache.tryInsertingCorrectState(GateState{ kAuthorityValue }, tick);
	}

	// A landed correction that AGREES with the prediction at `tick`: the caller
	// passes the value it predicted there. Exists because the two trigger policies
	// differ on exactly this input — `OnDisagreement` must ignore it and
	// `FrontierExact` must not (the legacy gate never read the verdict).
	void landAgreeingCorrection(GateCache& cache, std::uint32_t tick, std::int32_t predictedValue)
	{
		cache.tryInsertingCorrectState(GateState{ predictedValue }, tick);
	}

	// `SimulationReconciliation::prepareResimAll`'s gate half: capture the anchor
	// this resim is being prepared with, so the completion edge can CAS against it.
	// (The real method also restores live state from the anchor slot, which the gate
	// is indifferent to.)
	void prepareResimulation(GateCache& cache)
	{
		cache.captureResimAnchorForConsume();
	}

	// ONE replay tick, mirroring one `postResimulationAll` sweep for one character,
	// and REPORTING WHAT THE WRITE DID.
	//
	// [item 47] The outcome out-pointer is the only new production surface these
	// cases need: `Written` / `ProtectedFresh` / `ProtectedStale` / `Discarded`.
	// It is defaulted in production, so the cases that do not care read exactly
	// like item 45's did.
	resimGate::ResimSlotWriteOutcome replayTick(GateCache& cache, std::uint32_t tick)
	{
		resimGate::ResimSlotWriteOutcome outcome = resimGate::ResimSlotWriteOutcome::Discarded;
		cache.tryInsertingResimulatedState(GateState{ kResimulatedValue }, tick, &outcome);
		return outcome;
	}

	// The replay half, mirroring `postResimulationAll` — called ONCE PER REPLAYED
	// TICK, over the span `anchor+1 .. frontier`.
	//
	// ⚠⚠ [ITEM 47 CORRECTED THE SPAN, and it was NOT cosmetic.] Item 45's version
	// of this helper started AT the anchor. Production does not, and the ordering
	// is unambiguous in source: `SimulationManager::prepareResimulation` calls
	// `startResimulation(simTick)` (cursor := simTick) and restores live state from
	// slot `simTick`; then EVERY `onGameSimulationResimulation` calls
	// `advanceResimulation()` FIRST (cursor := simTick+1) before integrating and
	// stamping `m_lastStep`, which is the tick `postResimulationAll` then writes.
	// So the anchor slot is the restore SOURCE and is NEVER a write target.
	//
	// Under item 45 the extra tick was harmless — the replay wrote state nothing
	// asserted on. Under item 47 it is material: the anchor slot is corrected by
	// definition, so a helper that replayed it would exercise a PROTECTION that
	// production can never perform, and every fresh/stale count taken through this
	// helper would be one too high. Fixed rather than worked around, because a
	// case that models a sequence production cannot produce is the exact failure
	// this suite's own header warns about.
	//
	// ⚠ IT NO LONGER TOUCHES GATE STATE, and that is the item 45 change most likely
	// to be misread: under the old gate this loop was what re-closed the gate (it
	// set `m_isResimulated` on every replayed slot). Now it writes state only, and
	// the gate is closed by the explicit consume below.
	void replayResimulationFrom(GateCache& cache, std::uint32_t anchorTick)
	{
		const std::uint32_t frontierTick = cache.getPredictionTick();
		for (std::uint32_t tick = anchorTick + 1u; tick <= frontierTick; ++tick)
			replayTick(cache, tick);
	}

	// One complete resim, in production order: prepare (capture) -> replay ->
	// `[Resim.Finish]` (consume). Returns whether the consume actually cleared the
	// gate, which is false exactly when a newer anchor arrived mid-replay.
	bool completeResimulationFrom(GateCache& cache, std::uint32_t anchorTick)
	{
		prepareResimulation(cache);
		replayResimulationFrom(cache, anchorTick);
		return cache.consumeCapturedResimAnchor();
	}

	// Brings a cache to the state every interesting case starts from: three
	// predicted ticks, one frontier-exact correction, one COMPLETED resim — i.e.
	// the post-first-resim steady state a live client spends its whole session in.
	// Leaves the frontier at 103, the anchor CONSUMED and the gate CLOSED.
	//
	// ⚠ ITEM 44's VERSION OF THIS HELPER WAS NAMED FOR WHAT IT LEFT BEHIND: a
	// frontier carrying a set `m_isResimulated` bit, which was the precondition for
	// the shadowing. There is no such bit now and no shadow to set up, so the
	// helper is named for the lifecycle instead. It is still load-bearing: several
	// cases below are about the state AFTER a resim, which is where termination
	// lives.
	void driveToClosedGateAfterACompletedResim(GateCache& cache)
	{
		predictTick(cache, 100u, 1);
		predictTick(cache, 101u, 2);
		predictTick(cache, 102u, 3);

		landCorrection(cache, 102u);   // lands ON the frontier -> anchors 102
		predictTick(cache, 103u, 4);   // the gate opens here

		REQUIRE(cache.needsResimulation());
		REQUIRE(cache.getPendingResimAnchorTick() == 102u);

		REQUIRE(completeResimulationFrom(cache, 102u));   // replays 102..103, consumes

		REQUIRE_FALSE(cache.needsResimulation());
		REQUIRE(cache.getPendingResimAnchorTick() == 0u);
		REQUIRE(cache.getPredictionTick() == 103u);
	}
} // namespace

// ---------------------------------------------------------------------------
// THE FLOOR. Nothing corrected ⇒ no anchor ⇒ no gate. Under the old gate this was
// `getLastResimulationTick`'s `lastCorrectTick == 0` early-out; it is now simply
// the anchor's initial value, which is a strictly smaller thing to be sure of. It
// is still the precondition every other case rests on: a `false` below is only
// meaningful because it is NOT this trivial false.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AnUncorrectedCacheHasNoAnchorAndNoGate",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	// Fresh, before anything is predicted at all.
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);
	REQUIRE_FALSE(cache.needsResimulation());

	// Prediction alone never opens the gate, however far it runs — and now for a
	// structural reason rather than an arithmetic one: `pushPredictionTick` does not
	// touch gate state at all.
	for (std::uint32_t tick = 100u; tick <= 130u; ++tick)
	{
		predictTick(cache, tick, static_cast<std::int32_t>(tick));
		REQUIRE(cache.getPendingResimAnchorTick() == 0u);
		REQUIRE_FALSE(cache.needsResimulation());
	}
}

// ---------------------------------------------------------------------------
// [ITEM 45 RENAMED THIS CASE] was
// `ScanStartsAtTheFrontierAndAcceptsTheCorrectFlagThere`.
//
// There is no scan any more, so the old name described a mechanism rather than a
// behaviour. THE BEHAVIOUR SURVIVES UNCHANGED and is worth pinning for itself: a
// correction landing on the frontier anchors the frontier tick, and
// `needsResimulation()`'s second clause (`anchor != predictionTick`) keeps the gate
// SHUT on that same tick. That is why the re-open in the next case is "one tick
// LATER" and not immediate — the same one-tick lag the old inheritance produced,
// reproduced deliberately rather than incidentally.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AFrontierExactLandingAnchorsTheFrontierAndTheGateStaysShut",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	predictTick(cache, 100u, 1);
	predictTick(cache, 101u, 2);
	predictTick(cache, 102u, 3);

	landCorrection(cache, 102u);

	REQUIRE(cache.getPredictionTick() == 102u);
	REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(102u)));

	// Anchor == frontier ⇒ gate shut.
	REQUIRE(cache.getPendingResimAnchorTick() == 102u);
	REQUIRE_FALSE(cache.needsResimulation());
}

// ---------------------------------------------------------------------------
// THE LEGACY POLICY'S DEFINING CASE — kept from item 44, because under
// `FrontierExact` this is still exactly what happens, one tick later and all.
//
// What changed underneath: the old mechanism needed THREE steps to get here
// (`tryInsertingCorrectState` clears the frontier's bit -> `pushPredictionTick`
// inherits the cleared bit -> the scan steps down one slot). Now the landing sets
// the anchor outright and the frontier advance is irrelevant to the gate except
// through the `anchor != frontier` comparison. Same observable, three fewer moving
// parts.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AFrontierExactLandingReopensTheGateOneTickLater",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);   // frontier 103, gate shut

	// A correction lands EXACTLY on the frontier.
	landCorrection(cache, 103u);

	// Still shut on this tick — anchor is the frontier itself (previous case).
	REQUIRE(cache.getPendingResimAnchorTick() == 103u);
	REQUIRE_FALSE(cache.needsResimulation());

	// One tick later the frontier moves past the anchor and the gate opens.
	predictTick(cache, 104u, 5);

	REQUIRE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 103u);
	REQUIRE(cache.getPredictionTick() == 104u);
}

// ---------------------------------------------------------------------------
// ⭐ [ITEM 45 REPLACED THIS CASE] was `InheritanceCarriesTheResimFlagForward`,
// which pinned the frontier-bit inheritance as CORRECT-FOR-NOW (item 44 was
// explicit that it was not pinning a defect there). THE GUARD IS RETIRED, so the
// old assertions cannot hold and must not be made to: the old case asserted that
// after a resim replaying 102..103, a push to 104 made the anchor read 104 — A TICK
// `tryInsertingResimulatedState` NEVER TOUCHED — which was the only way to observe
// the inheritance through the public surface.
//
// WHAT REPLACES IT, and why this is the honest successor rather than a deletion:
// the inheritance existed to make "everything up to the frontier is reconciled"
// survive frontier advance. The edge-triggered gate states the same thing by
// having NO gate state to inherit — a consumed anchor is 0, and 0 stays 0 however
// far the frontier runs, because `pushPredictionTick` writes no gate state at all.
// So this case asserts the ABSENCE the old case asserted the presence of, over the
// same tick range, which is the same property observed from the other side.
//
// ⚠ THE FAILURE THIS GUARDS. If a future edit gives `pushPredictionTick` any anchor
// write — the obvious "carry the anchor forward" instinct — the gate becomes
// derivable from frontier advance again and candidate A's unterminating 1-tick
// resim comes back with it. The next case is the storm tripwire; this one names the
// specific line that would cause it.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.FrontierAdvanceTouchesNoGateStateAfterAConsumedResim",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);   // resim replayed 102 and 103

	predictTick(cache, 104u, 5);

	// 104 was never corrected and never replayed. Under the old gate it answered
	// the scan at offset 0 by INHERITING the frontier's resim bit, and the anchor
	// read 104. There is nothing to inherit now.
	REQUIRE(cache.getPredictionTick() == 104u);
	REQUIRE_FALSE(cache.containsCorrectTick(cache.getCacheIndex(104u)));
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);
	REQUIRE_FALSE(cache.needsResimulation());

	// And it keeps touching nothing, tick after tick.
	predictTick(cache, 105u, 6);
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);
	REQUIRE_FALSE(cache.needsResimulation());
}

// ---------------------------------------------------------------------------
// TERMINATION — the property the retired guard existed to provide, and the one
// property item 45 had to preserve BY DIFFERENT MEANS.
//
// A completed resim closes the gate, and it STAYS closed across an arbitrary run
// of further ticks with no new authority information. A design that re-opens here
// resimulates the same already-consumed correction forever.
//
// ⭐ THIS CASE IS THE STORM TRIPWIRE AND KEEPS ITS ITEM 44 NAME. If it goes red,
// the gate has reintroduced candidate A/B's unterminating resim — do not rewrite
// this case to accommodate that. The ONLY line item 45 changed inside it is the
// anchor-value assertion: item 44 asserted `anchor == getPredictionTick()` (the
// inherited flag answering the scan at the frontier), and the same property is now
// `anchor == 0` (consumed). The `REQUIRE_FALSE(needsResimulation())` that carries
// the actual property is untouched, and it is what must never move.
//
// It also now proves termination against a STRICTLY STRONGER precondition: slot 102
// still carries `m_containsCorrectTick`, and under `OnDisagreement` — set below for
// exactly this reason — a mechanism that re-derived the gate from corrected slots
// would re-trigger on every one of these ticks.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.ACompletedResimClosesTheGateAndItStaysClosed",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);   // the resim completed here

	REQUIRE_FALSE(cache.needsResimulation());

	// The most demanding regime, not the shipped one: under the designed trigger
	// every landed disagreement sets the anchor, so if anything in the frontier
	// advance re-derived the gate, this is where it would show.
	cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

	// The corrected slot 102 is still flagged `m_containsCorrectTick` and will
	// remain so until the ring recycles it ~60 ticks from now. Under a gate that
	// could see past the frontier it would re-trigger on every one of these ticks.
	for (std::uint32_t tick = 104u; tick <= 140u; ++tick)
	{
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

		REQUIRE_FALSE(cache.needsResimulation());
		REQUIRE(cache.getPendingResimAnchorTick() == 0u);
	}
}

// ---------------------------------------------------------------------------
// ⭐⭐ [ITEM 45 REWROTE THIS CASE] was
// `ABehindFrontierLandingIsShadowedByTheInheritedFlag_DocumentsADefect_Item45WillChangeThis`,
// and it is the DEFECT ITEM 45 EXISTS TO FIX. Item 44 pinned it as current
// behaviour with a comment saying so; the backlog named this replacement outright.
//
// THE DEFECT, for the record: a correction landing BEHIND the frontier is real
// authority information that disagrees with prediction (verdict false, state
// adopted into the slot) — and nothing happened. The frontier's inherited
// `m_isResimulated` bit answered the scan first at offset 0, so the corrected slot
// was never reached, the anchor stayed pinned to the frontier, and
// `needsResimulation()` stayed FALSE. That is the whole of the
// ~8,759-corrections-against-2-triggers measurement.
//
// BOTH POLICIES ARE PINNED HERE, IN ONE CASE, BECAUSE THE PAIR IS THE POINT:
// `OnDisagreement` (the fix) sets the anchor and opens the gate; `FrontierExact`
// (the SHIPPED default) still sets nothing, which is what makes item 45's landing
// behaviour-neutral. Reading either arm alone would be misleading in opposite
// directions.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.ABehindFrontierDisagreeingLandingSetsThePendingAnchor",
	"[CorrectionCache][ResimGate]")
{
	SECTION("under OnDisagreement — the fix: the gate opens on the corrected tick")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);   // frontier 103, gate shut

		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

		predictTick(cache, 104u, 5);
		predictTick(cache, 105u, 6);
		REQUIRE(cache.getPredictionTick() == 105u);

		// Authority disagrees about tick 104 — one single tick behind the frontier.
		landCorrection(cache, 104u);

		// It landed, the cache took the state, AND the gate is now open on it. This
		// assertion is the inverse of the one item 44 pinned.
		REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(104u)));
		REQUIRE(cache.getPendingResimAnchorTick() == 104u);
		REQUIRE(cache.needsResimulation());

		// It stays open across frontier advance — the trigger is not lost if the
		// physics frame that would consume it is refused (item 42's 15-31 % class).
		predictTick(cache, 106u, 7);
		REQUIRE(cache.needsResimulation());
		REQUIRE(cache.getPendingResimAnchorTick() == 104u);

		// And a completed resim from that anchor closes it, once.
		REQUIRE(completeResimulationFrom(cache, 104u));
		REQUIRE_FALSE(cache.needsResimulation());
	}

	SECTION("under FrontierExact — the SHIPPED default still sets nothing")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);

		predictTick(cache, 104u, 5);
		predictTick(cache, 105u, 6);

		landCorrection(cache, 104u);

		// Same landing, same disagreement, no anchor: the legacy policy triggers on
		// POSITION, never on the verdict. This is the behaviour-neutrality claim in
		// four lines — and the reason the flip is item 46 and not this item.
		REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(104u)));
		REQUIRE(cache.getPendingResimAnchorTick() == 0u);
		REQUIRE_FALSE(cache.needsResimulation());

		predictTick(cache, 106u, 7);
		REQUIRE_FALSE(cache.needsResimulation());
	}
}

// ---------------------------------------------------------------------------
// ⭐ [ITEM 45 REWROTE THIS CASE] was
// `RepeatedBehindFrontierLandingsNeverReopenTheGate_DocumentsADefect_Item45WillChangeThis`.
//
// Item 44's version showed that the shadow did not wear off: an unbroken run of
// behind-frontier landings produced ZERO triggers, nothing accumulated, and no
// depth built up. The fix's answer is NOT "one trigger per landing" — it is
// COALESCING TO THE NEWEST ANCHOR, which is the designed outcome and worth pinning
// precisely because "we now trigger on everything" is the failure mode the cost
// model (design §4) is afraid of.
//
// WHY NEWEST-ONLY IS CORRECT AND NOT A LOST-CORRECTION BUG: corrections arrive in
// tick order, so the authority state at the newest corrected tick SUBSUMES every
// older correction on the same trajectory. Restoring at the newest and replaying
// forward consumes the whole backlog at depth = `frontier - anchor`, i.e. at
// correction transit latency. Ten landings between two divergence checks cost ONE
// resim, not ten.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.RepeatedBehindFrontierLandingsCoalesceIntoOneNewestAnchor",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);   // frontier 103, gate shut
	cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

	// Predict out to 115, then feed corrections for 105..114 — every one of them
	// strictly behind the frontier, every one of them disagreeing.
	for (std::uint32_t tick = 104u; tick <= 115u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	REQUIRE(cache.getPredictionTick() == 115u);

	for (std::uint32_t tick = 105u; tick <= 114u; ++tick)
	{
		landCorrection(cache, tick);

		REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(tick)));

		// The gate is open, and the anchor tracks the NEWEST landing — one pending
		// resim, growing shallower as the corrections catch up to the frontier.
		REQUIRE(cache.needsResimulation());
		REQUIRE(cache.getPendingResimAnchorTick() == tick);
	}

	// One resim consumes the whole backlog: it restores at 114 (the newest) and
	// replays to the frontier.
	REQUIRE(completeResimulationFrom(cache, 114u));
	REQUIRE_FALSE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);

	// And the older corrected slots do not resurrect it as the frontier advances.
	predictTick(cache, 116u, 16);
	REQUIRE_FALSE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);
}

// ---------------------------------------------------------------------------
// COALESCING IS A MAX, NOT A LATEST-WINS. An OUT-OF-ORDER landing — a correction
// for an older tick arriving after a newer one, which the wire permits and item
// 41's discard population is adjacent to — must not pull the anchor BACKWARDS.
//
// If it did, the resim would restore deeper than necessary (paying rewind depth for
// state the newer correction already superseded) and, worse, an alternating pair of
// ticks could hold the gate open indefinitely by re-anchoring each other.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AnOlderLandingNeverPullsTheAnchorBackwards",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);
	cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

	for (std::uint32_t tick = 104u; tick <= 112u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	landCorrection(cache, 110u);
	REQUIRE(cache.getPendingResimAnchorTick() == 110u);

	// Older, and it lands (the slot exists) — but the anchor holds.
	landCorrection(cache, 106u);
	REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(106u)));
	REQUIRE(cache.getPendingResimAnchorTick() == 110u);

	// Newer still raises it.
	landCorrection(cache, 111u);
	REQUIRE(cache.getPendingResimAnchorTick() == 111u);
}

// ---------------------------------------------------------------------------
// ⛔ REQUIRED CASE (item 44's review, coverage gap 2): COALESCING-TO-NEWEST UNDER
// THE **LEGACY** POLICY. The two cases above pin it under `OnDisagreement`; this
// one pins it under `FrontierExact`, which is the SHIPPED default and therefore
// the only policy whose behaviour item 45 promised not to change.
//
// It reproduces what the legacy SCAN did: it started at offset 0 (the frontier)
// and walked backwards, so the NEWEST flagged slot won and a second correction
// simply moved the tick the resim would restore from. Here the same outcome comes
// out of `raisePendingResimAnchorTo`'s CAS-max instead: gate open at anchor 103 /
// frontier 104, a second frontier-exact landing at 104 raises the anchor to 104,
// and the gate RE-SHUTS on that tick (`anchor != predictionTick` is false again)
// until the next `pushPredictionTick`. One resim, from the newest — never two.
//
// ⚠ WHY THIS CASE EXISTS EVEN THOUGH IT PASSES TODAY FOR FREE: the raise is one
// shared, POLICY-INDEPENDENT `raisePendingResimAnchorTo`, so the `OnDisagreement`
// cases above currently pin the max for both policies. This case is the guard
// against a FUTURE POLICY-CONDITIONAL RAISE PATH — split the raise per policy and
// this is the case that goes red while every `OnDisagreement` case stays green.
// Do not "simplify" it away as a duplicate of
// `RepeatedBehindFrontierLandingsCoalesceIntoOneNewestAnchor`: it is a duplicate
// only for as long as the raise stays shared, which is exactly the property it is
// here to hold.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.RepeatedFrontierExactLandingsCoalesceIntoOneNewestAnchor",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);   // frontier 103, gate shut

	// NO policy call: this case is about the compiled default, and saying so is the
	// point of it.
	REQUIRE(cache.getResimTriggerPolicy() == TimeConfig::ResimTriggerPolicy::FrontierExact);

	// First frontier-exact landing: anchors 103, gate shut (anchor == frontier)...
	landCorrection(cache, 103u);
	REQUIRE(cache.getPendingResimAnchorTick() == 103u);
	REQUIRE_FALSE(cache.needsResimulation());

	// ...and opens one tick later, which is the legacy lag.
	predictTick(cache, 104u, 5);
	REQUIRE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 103u);
	REQUIRE(cache.getPredictionTick() == 104u);

	// THE CLAIM: a second frontier-exact landing, arriving while the gate is already
	// open, does not queue a second resim — it RAISES the one pending anchor to the
	// newest corrected tick, and because that tick is the frontier the gate re-shuts.
	landCorrection(cache, 104u);
	REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(103u)));
	REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(104u)));
	REQUIRE(cache.getPendingResimAnchorTick() == 104u);
	REQUIRE_FALSE(cache.needsResimulation());

	// Re-opens on the next push, at the NEWEST anchor — 103 is subsumed, not queued.
	predictTick(cache, 105u, 6);
	REQUIRE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 104u);

	// One resim consumes both landings, and nothing re-triggers behind it.
	REQUIRE(completeResimulationFrom(cache, 104u));
	REQUIRE_FALSE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);

	predictTick(cache, 106u, 7);
	REQUIRE_FALSE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);
}

// ---------------------------------------------------------------------------
// THE CONSUME EDGE IS A CAS, PROVEN BOTH WAYS, SINGLE-THREADED.
//
// This is the acceptance criterion the design calls out as "design §7.2's
// interleaving case becomes deterministic": the rule that makes a mid-replay
// landing survive is a `compare_exchange_strong(preparedAnchor, 0)`, and because
// the expected value is an explicit parameter, the interleaving needs no threads to
// exercise — a STALE expected value IS the interleaving.
//
// A compare-then-store would pass the first section and fail the second, which is
// exactly the silently-lost trigger this design exists to make impossible.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.TheConsumeEdgeIsACompareAndSwap",
	"[CorrectionCache][ResimGate]")
{
	SECTION("the matching prepared anchor consumes and closes the gate")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);
		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

		predictTick(cache, 104u, 5);
		predictTick(cache, 105u, 6);
		landCorrection(cache, 104u);
		REQUIRE(cache.needsResimulation());

		REQUIRE(cache.consumeResimAnchor(104u));
		REQUIRE(cache.getPendingResimAnchorTick() == 0u);
		REQUIRE_FALSE(cache.needsResimulation());
	}

	SECTION("a STALE prepared anchor consumes nothing and leaves the gate open")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);
		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

		// The frontier runs to 106 so that BOTH landings are strictly behind it —
		// otherwise the newer one would anchor the frontier itself and the gate's
		// `anchor != predictionTick` clause would close it for an unrelated reason,
		// hiding what this section is about.
		predictTick(cache, 104u, 5);
		predictTick(cache, 105u, 6);
		predictTick(cache, 106u, 7);
		landCorrection(cache, 104u);

		// The resim was prepared with 104; a newer correction has since landed.
		landCorrection(cache, 105u);
		REQUIRE(cache.getPendingResimAnchorTick() == 105u);

		REQUIRE_FALSE(cache.consumeResimAnchor(104u));
		REQUIRE(cache.getPendingResimAnchorTick() == 105u);
		REQUIRE(cache.needsResimulation());
	}

	SECTION("consuming with no anchor prepared is a no-op, not a wipe")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);
		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

		predictTick(cache, 104u, 5);
		predictTick(cache, 105u, 6);
		landCorrection(cache, 104u);

		// An engine-side rewind we never asked for reaches the completion edge with
		// nothing captured. It must not clear a trigger it never consumed.
		REQUIRE_FALSE(cache.consumeResimAnchor(0u));
		REQUIRE(cache.getPendingResimAnchorTick() == 104u);
		REQUIRE(cache.needsResimulation());
	}
}

// ---------------------------------------------------------------------------
// THE MID-REPLAY LANDING, end to end through the real lifecycle helpers: a
// correction lands on the game thread while the physics thread is replaying, and
// the resim it interrupted must NOT swallow it.
//
// This is the case design §7's open question 2 asked for, and the reason the
// consume compares rather than clears. Its failure mode is invisible in
// production: the correction is absorbed into the cache, the gate closes, and the
// client free-runs on state the authority already disagreed with — which is the
// original defect's exact signature.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AMidReplayLandingSurvivesTheResimAndRetriggers",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);
	cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

	for (std::uint32_t tick = 104u; tick <= 108u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	landCorrection(cache, 105u);
	REQUIRE(cache.getPendingResimAnchorTick() == 105u);

	// prepareResimAll captures 105 and the replay starts.
	prepareResimulation(cache);
	REQUIRE(cache.getCapturedResimAnchorTick() == 105u);
	replayResimulationFrom(cache, 105u);

	// GAME THREAD, mid-replay: authority disagrees about a NEWER tick.
	landCorrection(cache, 107u);

	// The `[Resim.Finish]` edge now fails to consume — and that is the designed
	// outcome, not an error.
	REQUIRE_FALSE(cache.consumeCapturedResimAnchor());
	REQUIRE(cache.getPendingResimAnchorTick() == 107u);
	REQUIRE(cache.needsResimulation());

	// The next resim consumes it properly.
	REQUIRE(completeResimulationFrom(cache, 107u));
	REQUIRE_FALSE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);
}

// ---------------------------------------------------------------------------
// THE VERDICT GATE BITES — and only under `OnDisagreement`.
//
// An AGREEING correction is authority information that says "your prediction was
// right". Resimulating from it is pure cost: the replay re-integrates the same
// inputs from a state it already matched. `OnDisagreement` therefore ignores it.
//
// ⚠ `FrontierExact` DOES NOT, AND THAT IS DELIBERATE — the legacy gate never read
// the verdict (it is stored and consulted by nothing, item 30), so a verdict test
// on the legacy policy would make it trigger STRICTLY LESS often than the mechanism
// it has to reproduce, on the one setting item 45's behaviour-neutrality rests on.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AnAgreeingLandingIsIgnoredOnlyUnderOnDisagreement",
	"[CorrectionCache][ResimGate]")
{
	SECTION("OnDisagreement ignores an agreeing frontier-exact landing")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);
		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

		predictTick(cache, 104u, 5);
		landAgreeingCorrection(cache, 104u, 5);   // authority agrees

		REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(104u)));
		REQUIRE(cache.getPendingResimAnchorTick() == 0u);

		predictTick(cache, 105u, 6);
		REQUIRE_FALSE(cache.needsResimulation());
	}

	SECTION("FrontierExact anchors it anyway — the legacy gate ignored the verdict")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);

		predictTick(cache, 104u, 5);
		landAgreeingCorrection(cache, 104u, 5);

		REQUIRE(cache.getPendingResimAnchorTick() == 104u);

		predictTick(cache, 105u, 6);
		REQUIRE(cache.needsResimulation());
		REQUIRE(cache.getPendingResimAnchorTick() == 104u);
	}
}

// ---------------------------------------------------------------------------
// ⚠ THE ONE KNOWN, DELIBERATE DIVERGENCE FROM THE LEGACY GATE — pinned so it is a
// recorded decision rather than a surprise in a future bisect.
//
// On a VIRGIN cache (no resim has completed, no correction has landed on the
// frontier) every `m_isResimulated` bit was false, so the legacy scan could see a
// BEHIND-frontier corrected slot and would open the gate once. `FrontierExact` does
// not. The cost is at most one trigger per character per virgin cache — session
// start, and after each hard resync, since `wipeCache` cleared those bits too —
// against a measured population of thousands of corrections per run. That is why
// item 45's acceptance criterion asks for statistically indistinguishable
// `[ResimProbe.Gate]` ratios rather than equality.
//
// Reproducing it exactly would mean keeping the retired bitset alive purely to
// emulate its pre-steady-state corner, which is the stale-truth trap the
// retirement exists to close.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.OnAVirginCacheFrontierExactSkipsTheLegacyGatesOneFreeTrigger",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	// No resim has ever run on this cache — the exact state the legacy shadow did
	// not yet exist in.
	predictTick(cache, 100u, 1);
	predictTick(cache, 101u, 2);
	predictTick(cache, 102u, 3);

	// A behind-frontier disagreeing landing. The legacy gate would have returned
	// anchor 101 here (nothing shadowed it) and opened. The shipped policy sets no
	// anchor, so the first trigger of the session waits for a frontier-exact
	// landing instead.
	landCorrection(cache, 101u);

	REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(101u)));
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);
	REQUIRE_FALSE(cache.needsResimulation());

	// The steady state still arms normally on the next frontier-exact landing, so
	// the divergence is one deferred trigger, not a disabled gate.
	landCorrection(cache, 102u);
	predictTick(cache, 103u, 4);
	REQUIRE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 102u);
}

// ---------------------------------------------------------------------------
// WIPE — the hard-resync path (`SimulationNetSync::wipeAllForResync` ->
// `wipeCache`). A resync renumbers the prediction clock, so an anchor that survived
// it would name a tick that no longer exists: at best a resim to nowhere, at worst
// — if the new numbering is LOWER — an anchor permanently above the frontier that
// the gate can never close, because `anchor != predictionTick` would stay true
// forever.
//
// The wipe is driven from a gate-OPEN state deliberately: a wipe that cleared only
// the bitsets (or only the tick buffer) would still look clean from a closed start.
// [item 45] The assertion set grew by the anchor itself and by the captured
// consume-expectation, which is the other word a resync must not leave behind.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.WipeCacheClearsAllGateState",
	"[CorrectionCache][ResimGate]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);

	// Re-open the gate so the wipe has something to clear, and leave a captured
	// consume-expectation behind too (a resync can land mid-resim).
	landCorrection(cache, 103u);
	predictTick(cache, 104u, 5);
	prepareResimulation(cache);
	REQUIRE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 103u);
	REQUIRE(cache.getCapturedResimAnchorTick() == 103u);

	cache.wipeCache(900u);

	REQUIRE(cache.getPredictionTick() == 900u);
	REQUIRE(cache.getLastCorrectTick() == 0u);
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);
	REQUIRE(cache.getCapturedResimAnchorTick() == 0u);
	REQUIRE_FALSE(cache.needsResimulation());

	// No stale anchor can be resurrected by the next push either.
	predictTick(cache, 901u, 1);
	REQUIRE(cache.getPendingResimAnchorTick() == 0u);
	REQUIRE_FALSE(cache.needsResimulation());

	// The gate re-arms normally afterwards — the wipe clears state, it does not
	// disable the mechanism.
	landCorrection(cache, 901u);
	predictTick(cache, 902u, 2);
	REQUIRE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 901u);
}

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / ITEM 47: A REPLAY MUST NOT CLOBBER A FRESH
// AUTHORITY CORRECTION — THE HOLLOW-ANCHOR DEFECT.
//
// Tagged `[CorrectionCache][ResimGate][ReplayProtect]`. The third tag is purely
// for delta isolation (`[CorrectionCache]~[ReplayProtect]` recovers item 45's
// exact contribution); the first two keep these cases inside the suites the
// backlog names.
//
// ── THE DEFECT, IN ONE PARAGRAPH ───────────────────────────────────────────
// `tryInsertingCorrectState` wrote authority state into slot T on the GAME
// thread; an in-flight replay on the PHYSICS thread then reached tick T and
// `tryInsertingResimulatedState` overwrote slot T with the REPLAYED state,
// unconditionally, while `m_containsCorrectTick` stayed SET. Item 45's surviving
// anchor then correctly triggered a follow-up resim at T, `prepareResimAll`
// restored the CLOBBERED state, and the replay faithfully reproduced the same
// prediction. **The trigger was real; the data it pointed at was gone.**
//
// ── WHAT ITEM 47 SHIPPED ───────────────────────────────────────────────────
// PROTECT-ALL-CORRECTED: a replay never overwrites a slot with
// `m_containsCorrectTick` set, so no bit is ever cleared and "bit set ⇒
// authority-grade state" holds by construction. The FRESH/STALE two-clause
// discriminator survives as CLASSIFICATION ONLY, feeding the probe's
// `freshClobbersAvoided` / `staleClobbersAvoided` split. Both live in
// `resimGate::classifyResimSlotWrite`, swept exhaustively as a pure predicate in
// `ResimGatePolicyTest.cpp`; what these cases prove is that the WIRING through a
// real cache lifecycle produces the three exposure populations the item names.
//
// ⚠ EVERY CASE BELOW CARRIES ITS OWN FAILING TWIN. The rule "a test that cannot
// fail is not a test" is enforced structurally here: each case contains, in the
// same replay, an input where the mechanism protects AND one where it writes, so
// a regression that deleted the protection would flip an assertion rather than
// leave the case vacuously green. The two readings are recorded per case in
// `impl/impl_notes_task47.md` §3.
//////////////////////////////////////////////////////////////////////////////

// ---------------------------------------------------------------------------
// ⭐⭐ POPULATION (a) — THE MID-REPLAY LANDING, AND THE HOLLOW-TRIGGER REPRO
// RUN TO ITS END: the follow-up resim now restores AUTHORITY state.
//
// This is the acceptance criterion's first arm, driven entirely through item 45's
// prepare / replay / consume lifecycle helpers. The final assertion is the one
// that matters: what `prepareResimAll` would restore from slot 107 on the
// follow-up resim is the AUTHORITY state, not a re-derivation of the prediction.
// Before item 47 that read `kResimulatedValue` and the whole follow-up resim was
// a no-op costing a full Chaos rewind.
//
// ⚠ THE FAILING TWIN IS SLOT 106. It is replayed by the SAME loop, in the same
// resim, one tick earlier, and differs from 107 in exactly one respect: no
// correction landed in it. It must read `kResimulatedValue`. If a future edit
// made `tryInsertingResimulatedState` skip writes for some unrelated reason,
// this case would go red on 106 rather than passing for the wrong reason.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.ReplayNeverClobbersAFreshCorrection_TheHollowAnchorRepro",
	"[CorrectionCache][ResimGate][ReplayProtect]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);   // frontier 103, gate shut
	cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

	for (std::uint32_t tick = 104u; tick <= 108u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	landCorrection(cache, 105u);
	REQUIRE(cache.getPendingResimAnchorTick() == 105u);

	// `prepareResimAll`: captures the anchor for the consume CAS AND the
	// landing-sequence baseline for the classifier, in one instant.
	prepareResimulation(cache);
	REQUIRE(cache.getCapturedResimAnchorTick() == 105u);
	const std::uint32_t preparedSeq = cache.getCapturedLandingSeq();

	// The replay runs 106..108. It gets as far as 106 — uncorrected, so written.
	REQUIRE(replayTick(cache, 106u) == resimGate::ResimSlotWriteOutcome::Written);

	// GAME THREAD, MID-REPLAY: authority disagrees about 107, which the replay
	// cursor has not reached yet. THIS is the tick the defect used to eat.
	landCorrection(cache, 107u);
	REQUIRE(cache.getSlotLandingSeq(cache.getCacheIndex(107u)) > preparedSeq);
	REQUIRE(cache.getPendingResimAnchorTick() == 107u);

	// The replay reaches 107 and is REFUSED. Fresh by the SEQUENCE clause (it
	// landed after prepare) and, here, by the TICK clause too (107 >= 105) —
	// either alone would classify it fresh, which is why the case that separates
	// the two clauses is a different one.
	REQUIRE(replayTick(cache, 107u) == resimGate::ResimSlotWriteOutcome::ProtectedFresh);
	REQUIRE(replayTick(cache, 108u) == resimGate::ResimSlotWriteOutcome::Written);

	// ⭐ THE PROTECTION ITSELF, and its failing twin one tick below it.
	REQUIRE(cache.getState(cache.getCacheIndex(107u)).value == kAuthorityValue);
	REQUIRE(cache.getState(cache.getCacheIndex(106u)).value == kResimulatedValue);

	// The provenance bit was never touched — nothing was overwritten, so nothing
	// had to be cleared.
	REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(107u)));

	// The `[Resim.Finish]` edge fails to consume (item 45's CAS), so the anchor
	// survives and re-triggers — unchanged by item 47.
	REQUIRE_FALSE(cache.consumeCapturedResimAnchor());
	REQUIRE(cache.getPendingResimAnchorTick() == 107u);
	REQUIRE(cache.needsResimulation());

	// ⭐⭐ THE TRIGGER IS NO LONGER HOLLOW. `prepareResimAll` restores live state
	// from the anchor slot; that slot now holds what the authority said.
	prepareResimulation(cache);
	REQUIRE(cache.getCapturedResimAnchorTick() == 107u);
	REQUIRE(cache.getState(cache.getCacheIndex(107u)).value == kAuthorityValue);

	replayResimulationFrom(cache, 107u);
	REQUIRE(cache.consumeCapturedResimAnchor());
	REQUIRE_FALSE(cache.needsResimulation());
}

// ---------------------------------------------------------------------------
// ⭐ POPULATION (a), THE HALF THE TICK CLAUSE CANNOT SEE — a mid-replay landing
// BELOW this cache's captured anchor. **This case is why the sequence clause
// exists.**
//
// Reachable because the restore tick is a MIN ACROSS CHARACTERS: another
// character's deeper anchor drags this cache's replay span below its own anchor,
// so a landing inside that span can sit below the captured value. The tick clause
// (`slotTick >= capturedAnchorTick`) reads FALSE there; only
// `slotLandingSeq > preparedLandingSeq` catches it.
//
// ⚠ THE FAILING TWIN IS THE SECOND SECTION: the SAME slot, the SAME tick, the
// SAME span — corrected BEFORE the prepare instead of after. It classifies
// ProtectedStale. The two sections differ in one thing only, the order of the
// landing against the prepare, which is exactly what the sequence clause
// measures. Delete the sequence clause and section 1 turns into section 2.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AMidReplayLandingBelowTheCapturedAnchorIsFreshBySequenceAlone",
	"[CorrectionCache][ResimGate][ReplayProtect]")
{
	// The shared restore tick another character forced. Named rather than
	// inlined because the whole case is about it being BELOW this cache's anchor.
	constexpr std::uint32_t kSharedMinRestoreTick = 105u;
	constexpr std::uint32_t kOwnAnchorTick        = 110u;
	constexpr std::uint32_t kLandingTick          = 107u;   // in span, below the anchor

	SECTION("landed AFTER prepare — fresh by sequence, invisible to the tick clause")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);
		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

		for (std::uint32_t tick = 104u; tick <= 112u; ++tick)
			predictTick(cache, tick, static_cast<std::int32_t>(tick));

		landCorrection(cache, kOwnAnchorTick);
		REQUIRE(cache.getPendingResimAnchorTick() == kOwnAnchorTick);

		prepareResimulation(cache);
		REQUIRE(cache.getCapturedResimAnchorTick() == kOwnAnchorTick);

		// The replay runs from the SHARED min, not from this cache's anchor.
		REQUIRE(replayTick(cache, 106u) == resimGate::ResimSlotWriteOutcome::Written);

		// Mid-replay landing, below the captured anchor. It does not raise the
		// anchor either (CAS-MAX, item 45) — 107 < 110.
		landCorrection(cache, kLandingTick);
		REQUIRE(cache.getPendingResimAnchorTick() == kOwnAnchorTick);
		REQUIRE(cache.getSlotLandingSeq(cache.getCacheIndex(kLandingTick))
		        > cache.getCapturedLandingSeq());
		REQUIRE(kLandingTick < cache.getCapturedResimAnchorTick());   // tick clause is FALSE here

		REQUIRE(replayTick(cache, kLandingTick) == resimGate::ResimSlotWriteOutcome::ProtectedFresh);
		REQUIRE(cache.getState(cache.getCacheIndex(kLandingTick)).value == kAuthorityValue);
	}

	SECTION("landed BEFORE prepare — the same slot in the same span is STALE")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);
		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

		for (std::uint32_t tick = 104u; tick <= 112u; ++tick)
			predictTick(cache, tick, static_cast<std::int32_t>(tick));

		// Order reversed: the older correction lands first, then the newer one
		// takes the anchor (CAS-max), then the resim is prepared.
		landCorrection(cache, kLandingTick);
		landCorrection(cache, kOwnAnchorTick);
		REQUIRE(cache.getPendingResimAnchorTick() == kOwnAnchorTick);

		prepareResimulation(cache);
		REQUIRE(cache.getSlotLandingSeq(cache.getCacheIndex(kLandingTick))
		        <= cache.getCapturedLandingSeq());

		REQUIRE(replayTick(cache, 106u) == resimGate::ResimSlotWriteOutcome::Written);
		REQUIRE(replayTick(cache, kLandingTick) == resimGate::ResimSlotWriteOutcome::ProtectedStale);

		// STALE IS STILL PROTECTED — that is the whole of amendment 2's
		// protect-all shape. The classification splits a COUNTER, never the
		// action: a stale corrected slot holds server truth the replay does not
		// supersede, because the replay derives from this character's OLD
		// PREDICTION at the shared min, not from newer authority.
		REQUIRE(cache.getState(cache.getCacheIndex(kLandingTick)).value == kAuthorityValue);
		REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(kLandingTick)));
	}

	// Kept referenced so the constant cannot rot into a comment-only fact.
	REQUIRE(kSharedMinRestoreTick < kOwnAnchorTick);
}

// ---------------------------------------------------------------------------
// ⭐⭐ POPULATION (b) — THE MULTI-CHARACTER MIN-FOLD, IN THE TWO-CACHE
// CONSTRUCTION THE ITEM REQUIRES. **This case is why the tick clause exists**,
// and the stale arm CANNOT be produced with one cache (see the third section).
//
// `checkDivergenceAll` folds the per-character anchors with `std::min`, because a
// Chaos rewind is global: one restore tick has to serve everybody. So character B,
// whose anchor is NEWER than the min, is restored at A's tick and replayed
// forward THROUGH its own corrected slots — the population that used to be
// clobbered without ever being applied, while B's anchor was consumed by the
// per-cache CAS as if it had been.
//
// Two distinct sub-populations inside B, and they classify differently, which is
// the point of the case:
//   * B's slot AT/ABOVE its own captured anchor -> FRESH by the tick clause (the
//     sequence clause calls it stale: it landed BEFORE prepare).
//   * B's older corrections in `(min, A_B)` -> STALE by both clauses, and still
//     protected.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.TheMinFoldProtectsTheNewerAnchoredCharactersCorrectedSlots",
	"[CorrectionCache][ResimGate][ReplayProtect]")
{
	GateCache cacheA(nullptr);
	GateCache cacheB(nullptr);

	for (GateCache* cache : { &cacheA, &cacheB })
	{
		driveToClosedGateAfterACompletedResim(*cache);
		cache->setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);
		for (std::uint32_t tick = 104u; tick <= 112u; ++tick)
			predictTick(*cache, tick, static_cast<std::int32_t>(tick));
	}

	// A anchors SHALLOW, B anchors DEEP-in-time (newer). B additionally carries an
	// older correction at 107, which its anchor at 110 subsumed (CAS-max) — the
	// "previously-consumed out-of-order correction" the item names.
	landCorrection(cacheA, 105u);
	landCorrection(cacheB, 107u);
	landCorrection(cacheB, 110u);
	REQUIRE(cacheA.getPendingResimAnchorTick() == 105u);
	REQUIRE(cacheB.getPendingResimAnchorTick() == 110u);

	// The fold, done exactly as `checkDivergenceAll` does it rather than hard-coded.
	const std::uint32_t sharedRestoreTick =
		std::min(cacheA.getPendingResimAnchorTick(), cacheB.getPendingResimAnchorTick());
	REQUIRE(sharedRestoreTick == 105u);

	prepareResimulation(cacheA);
	prepareResimulation(cacheB);
	REQUIRE(cacheA.getCapturedResimAnchorTick() == sharedRestoreTick);   // A IS the min
	REQUIRE(cacheB.getCapturedResimAnchorTick() == 110u);                // B is not

	SECTION("B's slot at its own anchor is FRESH — caught by the TICK clause only")
	{
		// It landed BEFORE the prepare, so the sequence clause reads stale...
		REQUIRE(cacheB.getSlotLandingSeq(cacheB.getCacheIndex(110u))
		        <= cacheB.getCapturedLandingSeq());
		// ...and the tick clause is what rescues it: 110 >= 110.
		REQUIRE(replayTick(cacheB, 110u) == resimGate::ResimSlotWriteOutcome::ProtectedFresh);
		REQUIRE(cacheB.getState(cacheB.getCacheIndex(110u)).value == kAuthorityValue);
	}

	SECTION("B's older correction inside (min, ownAnchor) is STALE — and still protected")
	{
		REQUIRE(replayTick(cacheB, 107u) == resimGate::ResimSlotWriteOutcome::ProtectedStale);
		REQUIRE(cacheB.getState(cacheB.getCacheIndex(107u)).value == kAuthorityValue);
		REQUIRE(cacheB.containsCorrectTick(cacheB.getCacheIndex(107u)));
	}

	SECTION("the whole shared replay: B keeps both, A keeps its own, uncorrected slots are written")
	{
		std::uint32_t fresh = 0u;
		std::uint32_t stale = 0u;
		std::uint32_t written = 0u;
		for (std::uint32_t tick = sharedRestoreTick + 1u; tick <= 112u; ++tick)
		{
			for (GateCache* cache : { &cacheA, &cacheB })
			{
				switch (replayTick(*cache, tick))
				{
				case resimGate::ResimSlotWriteOutcome::ProtectedFresh: ++fresh;   break;
				case resimGate::ResimSlotWriteOutcome::ProtectedStale: ++stale;   break;
				case resimGate::ResimSlotWriteOutcome::Written:        ++written; break;
				default: break;
				}
			}
		}

		// B contributed both protections; A contributed none (its only corrected
		// slot, 105, is the restore SOURCE and sits below the span).
		REQUIRE(fresh == 1u);
		REQUIRE(stale == 1u);
		REQUIRE(written == 12u);   // 7 ticks x 2 caches, minus B's two protected slots

		REQUIRE(cacheB.getState(cacheB.getCacheIndex(107u)).value == kAuthorityValue);
		REQUIRE(cacheB.getState(cacheB.getCacheIndex(110u)).value == kAuthorityValue);
		// The failing twin, in the same sweep: A's slots have no corrections in the
		// span and are all overwritten.
		REQUIRE(cacheA.getState(cacheA.getCacheIndex(110u)).value == kResimulatedValue);

		// ⭐ P1, CONSUME-ALL, AS SHIPPED AND DELIBERATELY UNCHANGED BY ITEM 47:
		// each cache CASes against ITS OWN captured anchor, so B's anchor is
		// consumed by a resim that restored at A's tick and never applied B's
		// correction. One resim; B is uncorrected by it and self-heals on its next
		// rotation landing (<= ceil(N/K) ticks). P2 (capture-the-restore-tick,
		// which would fail B's CAS and cascade <= N convergent resims) is only
		// MEANINGFUL with this item's protect-all — pre-47, round 1 clobbered the
		// very slots the follow-ups restore from, so every follow-up was hollow —
		// but it is a cost/quality call priced by item 46's per-resim cost data,
		// not by this item. ASSERTED, not implied.
		REQUIRE(cacheA.consumeCapturedResimAnchor());
		REQUIRE(cacheB.consumeCapturedResimAnchor());
		REQUIRE(cacheA.getPendingResimAnchorTick() == 0u);
		REQUIRE(cacheB.getPendingResimAnchorTick() == 0u);
		REQUIRE_FALSE(cacheB.needsResimulation());
	}
}

// ---------------------------------------------------------------------------
// ⭐ THE SINGLE-CHARACTER STRUCTURAL ZERO — the free classifier-wiring check.
//
// For ONE character the replay span is `capturedAnchor+1..frontier` and "stale"
// requires `tick < capturedAnchor`: DISJOINT RANGES, so a stale classification is
// unreachable no matter what lands. `staleClobbersAvoided` reading nonzero in a
// single-character session therefore means the classifier is comparing against
// the wrong anchor (the folded min instead of the per-cache capture) — which is
// the exact mistake item 47's amendment 1 forbids by name.
//
// The span here is loaded with corrected slots on purpose: an agreeing landing
// above the anchor, a disagreeing landing that raised it, and a corrected slot
// BELOW the anchor that the span never reaches. If the count could be nonzero,
// this is the shape that would produce it.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.ASingleCharacterReplayProducesNoStaleClassificationsAtAll",
	"[CorrectionCache][ResimGate][ReplayProtect]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);
	cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

	for (std::uint32_t tick = 104u; tick <= 112u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	landCorrection(cache, 105u);                              // raises the anchor...
	landCorrection(cache, 106u);                              // ...and then this does
	landAgreeingCorrection(cache, 109u, 109);                 // corrected, sets NO anchor
	REQUIRE(cache.getPendingResimAnchorTick() == 106u);

	prepareResimulation(cache);
	REQUIRE(cache.getCapturedResimAnchorTick() == 106u);

	std::uint32_t fresh = 0u;
	std::uint32_t stale = 0u;
	for (std::uint32_t tick = 107u; tick <= 112u; ++tick)
	{
		const resimGate::ResimSlotWriteOutcome outcome = replayTick(cache, tick);
		if (outcome == resimGate::ResimSlotWriteOutcome::ProtectedFresh) ++fresh;
		if (outcome == resimGate::ResimSlotWriteOutcome::ProtectedStale) ++stale;
	}

	// ⭐ THE STRUCTURAL ZERO.
	REQUIRE(stale == 0u);
	// ...and it is NOT the trivial zero of "nothing was protected at all": the
	// agreeing landing at 109 sits in the span and was protected.
	REQUIRE(fresh == 1u);

	// The corrected slot BELOW the anchor is untouched because the span never
	// reaches it — protected by ARITHMETIC, not by the rule.
	REQUIRE(cache.getState(cache.getCacheIndex(105u)).value == kAuthorityValue);
}

// ---------------------------------------------------------------------------
// ⭐ POPULATION (c) — AN AGREEING LANDING ABOVE THE ANCHOR, which item 46's flip
// makes the COMMON case rather than a corner.
//
// Under `OnDisagreement` an agreeing correction sets no anchor (item 45, pinned by
// `AnAgreeingLandingIsIgnoredOnlyUnderOnDisagreement`) — but it DOES set
// `m_containsCorrectTick`, and it takes the SKIP-THE-COPY branch, so the slot
// keeps the PREDICTED value. Protect-all covers it, and the reading that makes
// that correct is the one amendment 2 spells out: "authority-grade" means
// ADOPTED-AUTHORITY **OR** WITHIN-TOLERANCE-OF-AUTHORITY. The authority looked at
// this slot and certified it. A replay's re-derivation carries no such warrant.
//
// ⚠ THE FAILING TWIN IS SLOT 108 — same span, same replay, uncorrected, written.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AnAgreeingLandingAboveTheAnchorIsProtectedAndClassifiedFresh",
	"[CorrectionCache][ResimGate][ReplayProtect]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);
	cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

	for (std::uint32_t tick = 104u; tick <= 110u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	landCorrection(cache, 105u);
	landAgreeingCorrection(cache, 107u, 107);   // predicted 107 at tick 107

	// It set no anchor — the verdict gate bit, unchanged by item 47.
	REQUIRE(cache.getPendingResimAnchorTick() == 105u);
	REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(107u)));

	prepareResimulation(cache);

	REQUIRE(replayTick(cache, 106u) == resimGate::ResimSlotWriteOutcome::Written);
	// FRESH by the TICK clause (107 >= 105); the sequence clause reads stale,
	// because it landed before the prepare.
	REQUIRE(cache.getSlotLandingSeq(cache.getCacheIndex(107u)) <= cache.getCapturedLandingSeq());
	REQUIRE(replayTick(cache, 107u) == resimGate::ResimSlotWriteOutcome::ProtectedFresh);
	REQUIRE(replayTick(cache, 108u) == resimGate::ResimSlotWriteOutcome::Written);

	// ⭐ THE SLOT KEEPS THE CERTIFIED PREDICTION, not the replay's re-derivation.
	REQUIRE(cache.getState(cache.getCacheIndex(107u)).value == 107);
	// The failing twin.
	REQUIRE(cache.getState(cache.getCacheIndex(108u)).value == kResimulatedValue);
}

// ---------------------------------------------------------------------------
// ⭐ AMENDMENT 3 — THE FRONTIER RULING, ASSERTED ON WHAT `[Resim.Finish]`
// PUBLISHES rather than implied.
//
// `SimulationReconciliation::applyResimAll` reads the FRONTIER SLOT and copies it
// into live simulatable state — that is the resim's publication. Protecting the
// frontier slot therefore means a resim that ends on a corrected frontier
// publishes the AUTHORITY state instead of its own replay result. That is
// authority-beats-a-re-derivation applied at the one slot where the difference is
// externally visible, and it IS a deliberate behaviour change in what
// `[Resim.Finish]` publishes.
//
// Both sections read the same slot the same way, and the ONLY difference between
// them is whether a correction landed on the frontier. That contrast is the
// case's failing twin.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AFrontierExactLandingAtReplayEndIsWhatResimPublishes",
	"[CorrectionCache][ResimGate][ReplayProtect]")
{
	SECTION("frontier slot corrected at replay end — the resim publishes AUTHORITY")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);
		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

		for (std::uint32_t tick = 104u; tick <= 108u; ++tick)
			predictTick(cache, tick, static_cast<std::int32_t>(tick));

		landCorrection(cache, 106u);
		prepareResimulation(cache);

		REQUIRE(replayTick(cache, 107u) == resimGate::ResimSlotWriteOutcome::Written);

		// Mid-replay, the authority corrects the FRONTIER itself.
		landCorrection(cache, 108u);
		REQUIRE(cache.getPredictionTick() == 108u);

		REQUIRE(replayTick(cache, 108u) == resimGate::ResimSlotWriteOutcome::ProtectedFresh);

		// What `applyResimAll` reads — `cache.getState(getCacheIndex(frontier))`.
		const std::int32_t published =
			cache.getState(cache.getCacheIndex(cache.getPredictionTick())).value;
		REQUIRE(published == kAuthorityValue);
	}

	SECTION("frontier slot uncorrected — the resim publishes its own replay result")
	{
		GateCache cache(nullptr);
		driveToClosedGateAfterACompletedResim(cache);
		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

		for (std::uint32_t tick = 104u; tick <= 108u; ++tick)
			predictTick(cache, tick, static_cast<std::int32_t>(tick));

		landCorrection(cache, 106u);
		prepareResimulation(cache);
		replayResimulationFrom(cache, 106u);

		const std::int32_t published =
			cache.getState(cache.getCacheIndex(cache.getPredictionTick())).value;
		REQUIRE(published == kResimulatedValue);
	}
}

// ---------------------------------------------------------------------------
// ⭐⭐ THE PROVENANCE HALF OF THE INVARIANT: **`m_containsCorrectTick` CANNOT
// LIE.** Two-part, and both parts need pinning:
//   1. a replay never overwrites a slot carrying a fresh, unconsumed correction;
//   2. the bit must not remain set on a slot whose contents are a re-derivation.
//
// Under protect-all, (2) holds because nothing authority-marked is ever
// overwritten, so NO bit ever has to be cleared. The committed pre-47 state —
// bit SET, contents REPLAYED — was the worst of both, and this case is what makes
// its return visible: it walks the whole span and asserts, slot by slot, that the
// bit and the contents agree.
//
// ⚠ THE FAILING TWIN IS BUILT INTO THE SWEEP: every uncorrected slot must hold
// `kResimulatedValue`. A regression that protected everything would go red here,
// and one that protected nothing would go red on the corrected slots — so the
// case cannot pass by accident in either direction.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.TheProvenanceBitNeverLiesAcrossAReplay",
	"[CorrectionCache][ResimGate][ReplayProtect]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);
	cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);

	for (std::uint32_t tick = 104u; tick <= 112u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	// A span carrying all three kinds of slot.
	landCorrection(cache, 105u);                 // the anchor (restore source)
	landAgreeingCorrection(cache, 108u, 108);    // corrected, keeps the CERTIFIED prediction
	// 106, 107, 109..112 are uncorrected.

	// The bits, counted BEFORE the replay so the "no bit was cleared" half is a
	// measurement rather than an assumption.
	std::uint32_t correctedBefore = 0u;
	for (std::uint32_t tick = 104u; tick <= 112u; ++tick)
		if (cache.containsCorrectTick(cache.getCacheIndex(tick)))
			++correctedBefore;
	REQUIRE(correctedBefore == 2u);

	prepareResimulation(cache);

	// Mid-replay, one more correction lands ahead of the cursor.
	REQUIRE(replayTick(cache, 106u) == resimGate::ResimSlotWriteOutcome::Written);
	landCorrection(cache, 110u);
	for (std::uint32_t tick = 107u; tick <= 112u; ++tick)
		replayTick(cache, tick);

	std::uint32_t correctedAfter = 0u;
	for (std::uint32_t tick = 104u; tick <= 112u; ++tick)
	{
		const std::uint32_t idx = cache.getCacheIndex(tick);
		if (cache.containsCorrectTick(idx))
		{
			++correctedAfter;
			// (1) + (2): the bit is set, so the contents must be AUTHORITY-GRADE —
			// adopted authority, or the prediction the authority certified.
			const std::int32_t expected =
				(tick == 108u) ? 108 : kAuthorityValue;
			REQUIRE(cache.getState(idx).value == expected);
			REQUIRE(cache.getState(idx).value != kResimulatedValue);
		}
		else if (tick > 105u)
		{
			// The twin: everything the replay was allowed to touch, it touched.
			REQUIRE(cache.getState(idx).value == kResimulatedValue);
		}
	}

	// One bit gained (the mid-replay landing at 110); NONE cleared.
	REQUIRE(correctedAfter == correctedBefore + 1u);
}

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / ITEM 48: PER-SLOT **STATE PROVENANCE**.
// Everything below this banner is item 48; tagged
// `[CorrectionCache][ResimGate][Provenance]`, where the third tag exists for
// DELTA ISOLATION ONLY — exactly the bargain item 47 struck with
// `[ReplayProtect]`, so `[CorrectionCache]~[Provenance]` still recovers items
// 44/45/47's suites unchanged.
//
// ⚠ WHY THESE CASES LIVE IN THE GATE-SEMANTICS FILE RATHER THAN A FILE NAMED
// FOR PROVENANCE, given item 48's whole first fence is "this is not the gate".
// The alternative was a second file with a second copy of the four lifecycle
// helpers at the top of this one, and one of those helpers — `replayResimulationFrom`
// — carries a correction that ITEM 47 HAD TO MAKE (the replay span is
// `anchor+1..frontier`; the anchor slot is the restore SOURCE, never a write
// target). A duplicated copy of that helper is a duplicated copy of a fidelity
// bug waiting to be reintroduced, and a provenance map taken through a wrong
// span reads like a production defect rather than like a test defect. Sharing
// the corrected helper removes that whole failure mode. The FENCE IS ON THE
// COLUMN'S NAME AND ITS READERS, not on which file exercises it.
//
// ── WHAT THIS COLUMN IS, IN FOUR LINES ─────────────────────────────────────
// `SlotStateProvenance`, one byte per slot, written at five sites and read by
// NOTHING IN PRODUCTION. `Replayed` answers everything the retired
// `m_isResimulated` bool could; `AuthorityAgreedKeptPrediction` and
// `ReplayedOverCorrection` answer the two things it could not. The full
// contract and the three fences are in `SlotStateProvenance.h`; the item 45
// retirement block in `CorrectionCache.h` carries the annotation explaining why
// this is an override of its warning rather than an oversight of it.
//
// ⛔ THE COLUMN MUST NEVER FEED A DECISION. That is not a comment here — it is
// `TheProvenanceColumnCannotReachAnyProductionOutput` below, which garbage-fills
// the column mid-lifecycle and asserts every production output is byte-identical.
// If you are about to make the gate read provenance, that case is what you have
// to delete on purpose.
//////////////////////////////////////////////////////////////////////////////

namespace
{
	// The provenance of the slot holding `tick`. Fails loudly rather than
	// returning a sentinel: a case asking about a tick outside the window has
	// mis-modelled its own scenario, and `Empty` would look like a real answer.
	SlotStateProvenance provenanceAt(const GateCache& cache, std::uint32_t tick)
	{
		const std::uint32_t idx = cache.getCacheIndex(tick);
		REQUIRE(idx != GateCache::InvalidCacheIndex);
		return cache.getDiagnosticStateProvenance(idx);
	}

	// The provenance map over a TICK RANGE, as the alphabet the shipped
	// `[ResimProbe.SlotMap]` line prints (`.PACRX`). Tick-ordered rather than
	// slot-ordered because a CASE reasons in ticks; the shipped line is
	// slot-ordered because a LOG should show the literal ring. `!` marks a tick
	// with no slot, so a window mistake cannot silently read as `Empty`.
	std::string provenanceRun(const GateCache& cache,
	                          std::uint32_t    firstTick,
	                          std::uint32_t    lastTick)
	{
		std::string run;
		for (std::uint32_t tick = firstTick; tick <= lastTick; ++tick)
		{
			const std::uint32_t idx = cache.getCacheIndex(tick);
			run.push_back(idx == GateCache::InvalidCacheIndex
				? '!'
				: slotStateProvenanceChar(cache.getDiagnosticStateProvenance(idx)));
		}
		return run;
	}

	// How many of the ring's 60 slots carry `value`. Whole-ring rather than
	// per-span on purpose: the `ReplayedOverCorrection == 0` assertions have to
	// cover slots no case names, or they only prove the value is absent where it
	// was already not expected.
	std::uint32_t countProvenanceInRing(const GateCache& cache, SlotStateProvenance value)
	{
		std::uint32_t count = 0u;
		for (std::uint32_t slot = 0u; slot < static_cast<std::uint32_t>(GateCache::StateBufferSize); ++slot)
			if (cache.getDiagnosticStateProvenance(slot) == value)
				++count;
		return count;
	}

	// =======================================================================
	// FENCE 2's INSTRUMENT — "every production output", captured as a number
	// sequence so two runs can be compared for byte-identity.
	//
	// ⭐ WHAT IS IN HERE IS THE WHOLE STRENGTH OF THE FENCE, so it is deliberately
	// EXHAUSTIVE over `StateCorrectionCache`'s const surface rather than a
	// selection: the gate and its anchor, the prepare-time captures, the landing
	// counter, the trigger policy, EVERY slot's correction bit / applied-capture
	// ref / landing stamp / STATE, the tick-buffer contents (through
	// `getCacheIndex` over a range that brackets every tick any case uses), the
	// anomalous-miss log gate, and — deliberately — `compute_checksum`, which is
	// how the ⛔ "provenance never enters the determinism comparison" prohibition
	// becomes a test rather than a comment.
	//
	// ⛔ `getDiagnosticStateProvenance` IS THE ONE THING NOT IN HERE, and its
	// absence is the point: the scribbled run and the clean run differ in exactly
	// that column and in nothing else, so an equal trace means no production path
	// can see it. Do not "complete" this function by adding it.
	// =======================================================================
	struct ProductionTrace
	{
		std::vector<std::uint32_t> values;
		bool operator==(const ProductionTrace& other) const { return values == other.values; }
	};

	void recordProductionOutputs(const GateCache& cache, ProductionTrace& trace)
	{
		trace.values.push_back(cache.getPredictionTick());
		trace.values.push_back(cache.getLastCorrectTick());
		trace.values.push_back(cache.getPendingResimAnchorTick());
		trace.values.push_back(cache.needsResimulation() ? 1u : 0u);
		trace.values.push_back(cache.getCapturedResimAnchorTick());
		trace.values.push_back(cache.getCapturedLandingSeq());
		trace.values.push_back(cache.getLandingSeq());
		trace.values.push_back(static_cast<std::uint32_t>(cache.getResimTriggerPolicy()));
		// ⛔ THE CHECKSUM PROHIBITION, MACHINE-CHECKED. If provenance ever leaks
		// into `compute_checksum`, two peers that agree on state and disagree on
		// lineage would hash differently — and this line goes red.
		trace.values.push_back(cache.compute_checksum(cache.getPredictionTick()));

		for (std::uint32_t slot = 0u; slot < static_cast<std::uint32_t>(GateCache::StateBufferSize); ++slot)
		{
			trace.values.push_back(cache.containsCorrectTick(slot) ? 1u : 0u);
			trace.values.push_back(cache.getAppliedCaptureTick(slot));
			trace.values.push_back(cache.getSlotLandingSeq(slot));
			trace.values.push_back(static_cast<std::uint32_t>(cache.getState(slot).value));
		}

		// The tick buffer and the log gate, over a range that brackets every tick
		// the lifecycle below touches (and some that it does not, so a slot
		// mapping that moved would show up too).
		for (std::uint32_t tick = 90u; tick <= 130u; ++tick)
		{
			trace.values.push_back(cache.getCacheIndex(tick));
			trace.values.push_back(cache.isAnomalousMiss(tick) ? 1u : 0u);
		}
	}

	// ONE FIXED GATE LIFECYCLE, driven identically every time, recording the
	// production trace after every step. Four knobs, and all of them exist to
	// make the comparison honest rather than to vary the scenario:
	//
	//   `seedGateOpen` / `seedMidPrepare` / `seedPostConsume`
	//                         garbage-fill the provenance column at the THREE
	//                         lifecycle points item 48 names. 0 means "do not
	//                         scribble at this point".
	//   `extraLandingTick != 0`  land one EXTRA REAL correction mid-replay. This
	//                         is the SENSITIVITY TWIN: it must move the trace. A
	//                         trace that captured nothing would pass the fence
	//                         vacuously, and this is what proves it does not.
	//
	// ⛔⛔ THE THREE SEEDS ARE INDEPENDENT, AND THAT IS A CORRECTION MADE BY A
	// MUTATION RUN — DO NOT COLLAPSE THEM BACK INTO ONE. The first draft took a
	// single seed and derived the other two from it (`seed+3`, `seed+7`). A
	// planted leak — `getLastCorrectTick` skipping any corrected slot whose
	// provenance read `Replayed` — SURVIVED that version green. The reason is
	// arithmetic, not luck: the generator writes `(seed + slot) % 6`, so ONE seed
	// gives each slot exactly ONE of the six values, and the newest corrected
	// slot simply never drew `Replayed` on the three seeds a single base value
	// produced. A fence that poisons each slot with one value out of six is not a
	// fence. With three independent seeds the caller sweeps all 6^3 combinations,
	// which is EXHAUSTIVE over this generator: every slot takes every value at
	// every one of the three points somewhere in the sweep.
	void driveTracedLifecycle(GateCache&       cache,
	                          ProductionTrace& trace,
	                          std::uint32_t    seedGateOpen,
	                          std::uint32_t    seedMidPrepare,
	                          std::uint32_t    seedPostConsume,
	                          std::uint32_t    extraLandingTick)
	{
		cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);
		for (std::uint32_t tick = 100u; tick <= 108u; ++tick)
			predictTick(cache, tick, static_cast<std::int32_t>(tick));
		recordProductionOutputs(cache, trace);

		cache.tryInsertingCorrectState(GateState{ kAuthorityValue }, 104u, 77u);
		landAgreeingCorrection(cache, 106u, 106);
		recordProductionOutputs(cache, trace);

		// ── SCRIBBLE POINT 1: THE GATE IS OPEN AND UNCONSUMED. ──────────────
		REQUIRE(cache.needsResimulation());
		if (seedGateOpen != 0u)
			cache.scribbleDiagnosticStateProvenanceForFenceTest(seedGateOpen);
		recordProductionOutputs(cache, trace);

		prepareResimulation(cache);
		recordProductionOutputs(cache, trace);

		// The replay span, `capturedAnchor+1 .. frontier` == 105..108.
		for (std::uint32_t tick = 105u; tick <= 108u; ++tick)
		{
			trace.values.push_back(static_cast<std::uint32_t>(replayTick(cache, tick)));
			if (tick == 105u)
			{
				if (extraLandingTick != 0u)
					landCorrection(cache, extraLandingTick);
				// ── SCRIBBLE POINT 2: MID-PREPARE, cursor inside the span. ──
				if (seedMidPrepare != 0u)
					cache.scribbleDiagnosticStateProvenanceForFenceTest(seedMidPrepare);
			}
			recordProductionOutputs(cache, trace);
		}

		trace.values.push_back(cache.consumeCapturedResimAnchor() ? 1u : 0u);
		recordProductionOutputs(cache, trace);

		// ── SCRIBBLE POINT 3: POST-CONSUME. ─────────────────────────────────
		if (seedPostConsume != 0u)
			cache.scribbleDiagnosticStateProvenanceForFenceTest(seedPostConsume);

		// And the lifecycle CONTINUES past the scribble, because "the column is
		// invisible" has to hold for everything that happens AFTER it is poisoned,
		// not merely for the reading taken at the instant of poisoning.
		predictTick(cache, 109u, 109);
		landCorrection(cache, 109u);
		recordProductionOutputs(cache, trace);

		CorrectionInsertVerdict verdict{};
		cache.tryInsertingCorrectState(GateState{ 107 }, 107u, 88u, &verdict);
		trace.values.push_back(verdict.landed ? 1u : 0u);
		trace.values.push_back(verdict.predictionWasCorrect ? 1u : 0u);
		trace.values.push_back(verdict.tick);
		recordProductionOutputs(cache, trace);

		// One discarded correction, so the miss path's log gate is on the trace too.
		cache.tryInsertingCorrectState(GateState{ kAuthorityValue }, 900u);
		recordProductionOutputs(cache, trace);
	}
} // namespace

// ---------------------------------------------------------------------------
// THE ALPHABET. `[ResimProbe.SlotMap]` prints one character per slot, and an
// operator greps those characters — so a value added to the enum without a
// character, or two values sharing one, is a silently unreadable map. Swept as a
// unit because the enum and its alphabet are pure and need no cache.
//
// ⭐ THE `static_assert`s ON `isAuthorityGradeProvenance` ARE THE SHARP ONES.
// That predicate is what the replay write site consults to decide between
// `Replayed` and the `ReplayedOverCorrection` ALARM, so its truth table is the
// alarm's own definition:
//   * the two authority-grade values are exactly the population
//     `m_containsCorrectTick` marks — no more, or protect-all and the alarm would
//     disagree about which slots are protected;
//   * `ReplayedOverCorrection` itself is NOT authority-grade, deliberately. See
//     the erasure-window note on the case below.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.TheProvenanceAlphabetIsOneCharPerValueAndAllSixAreDistinct",
	"[CorrectionCache][ResimGate][Provenance]")
{
	static constexpr SlotStateProvenance kAll[] = {
		SlotStateProvenance::Empty,
		SlotStateProvenance::Predicted,
		SlotStateProvenance::AuthorityAdopted,
		SlotStateProvenance::AuthorityAgreedKeptPrediction,
		SlotStateProvenance::Replayed,
		SlotStateProvenance::ReplayedOverCorrection,
	};
	static_assert(std::size(kAll) == kSlotStateProvenanceCount,
		"a value was added to SlotStateProvenance without extending this sweep");

	// The scribble generator walks `% kSlotStateProvenanceCount`, so the
	// enumerators must be the contiguous range [0, count) or it would skip values
	// and the fence test would poison the column less thoroughly than it claims.
	for (std::uint8_t i = 0u; i < kSlotStateProvenanceCount; ++i)
		REQUIRE(static_cast<SlotStateProvenance>(i) == kAll[i]);

	std::string alphabet;
	for (const SlotStateProvenance value : kAll)
	{
		const char c = slotStateProvenanceChar(value);
		// The twin: '?' is the out-of-enumeration fallback, so hitting it here
		// would mean a value the shipped emitter cannot print.
		REQUIRE(c != '?');
		REQUIRE(std::string(slotStateProvenanceName(value)) != "Unknown");
		alphabet.push_back(c);
	}
	REQUIRE(alphabet == ".PACRX");

	std::string sorted = alphabet;
	std::sort(sorted.begin(), sorted.end());
	REQUIRE(std::unique(sorted.begin(), sorted.end()) == sorted.end());

	static_assert(isAuthorityGradeProvenance(SlotStateProvenance::AuthorityAdopted));
	static_assert(isAuthorityGradeProvenance(SlotStateProvenance::AuthorityAgreedKeptPrediction));
	static_assert(!isAuthorityGradeProvenance(SlotStateProvenance::Empty));
	static_assert(!isAuthorityGradeProvenance(SlotStateProvenance::Predicted));
	static_assert(!isAuthorityGradeProvenance(SlotStateProvenance::Replayed));
	static_assert(!isAuthorityGradeProvenance(SlotStateProvenance::ReplayedOverCorrection));
}

// ---------------------------------------------------------------------------
// SCENARIO 1 — A PLAIN PREDICTION RUN. Every written slot reads `Predicted`;
// every slot the ring has not reached yet reads `Empty`.
//
// ⚠ THE `Empty` HALF IS THE LOAD-BEARING HALF, and it is why this column can say
// something the tick buffer cannot: both constructors `fill(0)` the tick buffer,
// so an UNWRITTEN slot claims tick 0 (the tick-0 phantom). `Empty` is the only
// place that distinction exists in data. The count assertion is the failing twin
// — a write site that stamped `Predicted` too eagerly would show up as 60.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.APlainPredictionRunIsAllPredictedAndTheRestStaysEmpty",
	"[CorrectionCache][ResimGate][Provenance]")
{
	GateCache cache(nullptr);

	// A virgin cache has written no state anywhere.
	REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::Empty)
	        == static_cast<std::uint32_t>(GateCache::StateBufferSize));

	for (std::uint32_t tick = 100u; tick <= 104u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	REQUIRE(provenanceRun(cache, 100u, 104u) == "PPPPP");
	REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::Predicted) == 5u);
	REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::Empty)
	        == static_cast<std::uint32_t>(GateCache::StateBufferSize) - 5u);
	REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::Replayed) == 0u);
	REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::ReplayedOverCorrection) == 0u);
}

// ---------------------------------------------------------------------------
// SCENARIO 2 — THE TWO LANDING ARMS, AND ⭐ THE PAYLOAD OF THE WHOLE ITEM.
//
// `m_containsCorrectTick` is set on BOTH verdicts, so the retired bool could say
// "authority-grade" and stop. It could not say WHICH KIND, and the difference is
// real: `tryInsertingCorrectState` copies the authority state only
// `if (!predictionWasCorrect)`, so an AGREEING landing leaves a slot that is
// authority-grade while physically holding the PREDICTED value (item 47's header
// nuance, in prose there and in data here for the first time).
//
// ⚠ STATE AND PROVENANCE ARE ASSERTED TOGETHER ON BOTH ARMS, deliberately: the
// provenance write and the state copy are ONE decision expressed twice, twenty
// lines apart in `CorrectionCache.h`, and this is what stops them drifting.
// The two arms are each other's failing twin — they run in the same cache, in
// the same ring pass, and differ only in the value the correction carries.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.ADisagreeingLandingAdoptsAuthorityAndAnAgreeingOneCertifiesThePrediction",
	"[CorrectionCache][ResimGate][Provenance]")
{
	GateCache cache(nullptr);

	for (std::uint32_t tick = 100u; tick <= 104u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	landCorrection(cache, 102u);                 // disagrees -> authority ADOPTED
	landAgreeingCorrection(cache, 103u, 103);    // agrees    -> prediction CERTIFIED

	REQUIRE(provenanceRun(cache, 100u, 104u) == "PPACP");

	const std::uint32_t adopted   = cache.getCacheIndex(102u);
	const std::uint32_t certified = cache.getCacheIndex(103u);

	// The state each arm actually left behind — the half a provenance byte alone
	// could be wrong about.
	REQUIRE(cache.getState(adopted).value   == kAuthorityValue);
	REQUIRE(cache.getState(certified).value == 103);

	// ⭐ THE BIT CANNOT TELL THEM APART. It is set on both, and
	// `isAuthorityGradeProvenance` agrees with it on both — which is required, or
	// item 47's protect-all and item 48's alarm would disagree about which slots
	// are protected. The ENUM is what separates them.
	REQUIRE(cache.containsCorrectTick(adopted));
	REQUIRE(cache.containsCorrectTick(certified));
	REQUIRE(isAuthorityGradeProvenance(cache.getDiagnosticStateProvenance(adopted)));
	REQUIRE(isAuthorityGradeProvenance(cache.getDiagnosticStateProvenance(certified)));
	REQUIRE(cache.getDiagnosticStateProvenance(adopted)
	        != cache.getDiagnosticStateProvenance(certified));
}

// ---------------------------------------------------------------------------
// SCENARIO 3 — A COMPLETED RESIM. The span reads `Replayed`; the ANCHOR SLOT
// STAYS `AuthorityAdopted`.
//
// ⚠ THE ANCHOR SLOT IS THE RESTORE **SOURCE**, NEVER A WRITE TARGET — the replay
// span is `anchor+1..frontier` (item 47's fidelity fix to items 44/45's helper;
// `startResimulation` sets the cursor and `advanceResimulation` runs before the
// first integrate). This case is driven through the CORRECTED helper.
//
// ⭐ A NOTE FOR WHOEVER READS THIS MAP AGAINST A SPAN BUG: under protect-all the
// anchor slot's PROVENANCE is invariant to the one-tick overstatement, because
// replaying a corrected slot is refused and refusal writes nothing. What the
// overstatement moves is the write OUTCOME (a `ProtectedFresh` production can
// never perform), so this case asserts the outcomes tick by tick as well as the
// map. Do not read the map alone as proof the span is right.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.ACompletedResimReadsReplayedAcrossTheSpanAndLeavesTheAnchorAuthorityAdopted",
	"[CorrectionCache][ResimGate][Provenance]")
{
	GateCache cache(nullptr);   // shipped FrontierExact policy

	predictTick(cache, 100u, 1);
	predictTick(cache, 101u, 2);
	predictTick(cache, 102u, 3);
	landCorrection(cache, 102u);          // frontier-exact -> anchors 102
	predictTick(cache, 103u, 4);
	predictTick(cache, 104u, 5);

	REQUIRE(cache.needsResimulation());
	REQUIRE(cache.getPendingResimAnchorTick() == 102u);
	REQUIRE(provenanceRun(cache, 100u, 104u) == "PPAPP");

	prepareResimulation(cache);
	// The span, one tick at a time, asserting what each write DID.
	REQUIRE(replayTick(cache, 103u) == resimGate::ResimSlotWriteOutcome::Written);
	REQUIRE(replayTick(cache, 104u) == resimGate::ResimSlotWriteOutcome::Written);
	REQUIRE(cache.consumeCapturedResimAnchor());

	REQUIRE(provenanceRun(cache, 100u, 104u) == "PPARR");
	REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::Replayed) == 2u);
	REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::ReplayedOverCorrection) == 0u);

	// The failing twin, inside the same replay: the anchor slot keeps AUTHORITY
	// state and its lineage, while everything the replay was allowed to touch
	// holds the replay value and says so.
	REQUIRE(cache.getState(cache.getCacheIndex(102u)).value == kAuthorityValue);
	REQUIRE(cache.getState(cache.getCacheIndex(103u)).value == kResimulatedValue);
	REQUIRE(cache.getState(cache.getCacheIndex(104u)).value == kResimulatedValue);
}

// ---------------------------------------------------------------------------
// SCENARIO 4 — THE MID-REPLAY LANDING, i.e. item 47's hollow-anchor repro read
// through the provenance column.
//
// ⭐⭐ THE FLIP, STATED NOW SO A FUTURE READER DOES NOT HAVE TO RECONSTRUCT IT.
// **PRE-ITEM-47** this scenario produced the provenance LIE: the replay reached
// 107 after the correction had landed there, overwrote the authority state, and
// left `m_containsCorrectTick` SET over a re-derivation. Had this column existed
// then, slot 107 would have read `ReplayedOverCorrection` — and the case would
// have asserted it appears EXACTLY there, because that was the shipped truth.
// **POST-ITEM-47** protect-all refuses the write, so the same case flips to
// asserting the value NEVER appears and the slot still reads `AuthorityAdopted`.
// The value is retained precisely because it can express the regression: a
// column that cannot say "this lied" cannot report the lie coming back.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.AMidReplayLandingStaysAuthorityAdoptedAndNeverReadsReplayedOverCorrection",
	"[CorrectionCache][ResimGate][Provenance]")
{
	GateCache cache(nullptr);

	driveToClosedGateAfterACompletedResim(cache);
	cache.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);
	for (std::uint32_t tick = 104u; tick <= 110u; ++tick)
		predictTick(cache, tick, static_cast<std::int32_t>(tick));

	landCorrection(cache, 105u);                     // anchors 105
	REQUIRE(cache.getPendingResimAnchorTick() == 105u);

	prepareResimulation(cache);
	REQUIRE(replayTick(cache, 106u) == resimGate::ResimSlotWriteOutcome::Written);

	// THE MID-REPLAY LANDING — game thread, ahead of the physics-thread cursor.
	landCorrection(cache, 107u);
	REQUIRE(provenanceAt(cache, 107u) == SlotStateProvenance::AuthorityAdopted);

	// ...and the replay now reaches it.
	REQUIRE(replayTick(cache, 107u) == resimGate::ResimSlotWriteOutcome::ProtectedFresh);
	for (std::uint32_t tick = 108u; tick <= 110u; ++tick)
		REQUIRE(replayTick(cache, tick) == resimGate::ResimSlotWriteOutcome::Written);

	//        105 106 107 108 109 110
	REQUIRE(provenanceRun(cache, 105u, 110u) == "ARARRR");

	// ⛔ THE ASSERTION THE VALUE EXISTS FOR — and it is whole-ring, not
	// span-local, so it covers slots this case never names.
	REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::ReplayedOverCorrection) == 0u);

	// The lie's two halves, both refuted at the one slot where it would have
	// appeared: the bit is set AND the contents are authority's, not the replay's.
	REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(107u)));
	REQUIRE(cache.getState(cache.getCacheIndex(107u)).value == kAuthorityValue);
	// The twin in the same span: 106 was uncorrected, so it holds the replay value
	// and says `R`. The case therefore cannot pass by protecting everything.
	REQUIRE(cache.getState(cache.getCacheIndex(106u)).value == kResimulatedValue);
}

// ---------------------------------------------------------------------------
// SCENARIO 5 — THE TWO-CACHE MIN-FOLD, the OTHER population item 47's
// reachability note names, read through the column.
//
// A non-min character B is restored at the SHARED min, so its replay span dips
// below its own anchor and passes corrected slots on both sides of it: one at
// its own anchor (fresh by the tick clause) and an older one inside
// `(min, ownAnchor)` (stale, and still protected). ⭐ BOTH are protected, so
// NEITHER produces `ReplayedOverCorrection` — the post-47 flip applies to this
// population exactly as it does to the mid-replay one. Pre-47 this construction
// produced the lie at BOTH slots.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.TheTwoCacheMinFoldProducesNoReplayedOverCorrectionEither",
	"[CorrectionCache][ResimGate][Provenance]")
{
	GateCache cacheA(nullptr);
	GateCache cacheB(nullptr);

	for (GateCache* cache : { &cacheA, &cacheB })
	{
		driveToClosedGateAfterACompletedResim(*cache);
		cache->setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);
		for (std::uint32_t tick = 104u; tick <= 112u; ++tick)
			predictTick(*cache, tick, static_cast<std::int32_t>(tick));
	}

	landCorrection(cacheA, 105u);
	landCorrection(cacheB, 107u);   // the older correction, subsumed by CAS-max
	landCorrection(cacheB, 110u);   // B's own, NEWER anchor

	const std::uint32_t sharedRestoreTick =
		std::min(cacheA.getPendingResimAnchorTick(), cacheB.getPendingResimAnchorTick());
	REQUIRE(sharedRestoreTick == 105u);

	prepareResimulation(cacheA);
	prepareResimulation(cacheB);

	for (std::uint32_t tick = sharedRestoreTick + 1u; tick <= 112u; ++tick)
	{
		replayTick(cacheA, tick);
		replayTick(cacheB, tick);
	}

	//                       106 107 108 109 110 111 112
	REQUIRE(provenanceRun(cacheB, 106u, 112u) == "RARRARR");
	// A's own anchor at 105 is the restore SOURCE and below the span, so A is
	// uniformly replayed across it — the failing twin at the cache level.
	REQUIRE(provenanceRun(cacheA, 106u, 112u) == "RRRRRRR");
	REQUIRE(provenanceAt(cacheA, 105u) == SlotStateProvenance::AuthorityAdopted);

	REQUIRE(countProvenanceInRing(cacheA, SlotStateProvenance::ReplayedOverCorrection) == 0u);
	REQUIRE(countProvenanceInRing(cacheB, SlotStateProvenance::ReplayedOverCorrection) == 0u);
	REQUIRE(cacheB.getState(cacheB.getCacheIndex(107u)).value == kAuthorityValue);
	REQUIRE(cacheB.getState(cacheB.getCacheIndex(110u)).value == kAuthorityValue);
}

// ---------------------------------------------------------------------------
// SCENARIO 6 — THE TWO RETIREMENT POINTS. A slot's lineage must die with the
// tick it described, or the column becomes the stale second copy of the truth
// T16 exists to forbid — the exact failure mode that makes re-adding a per-slot
// column dangerous in the first place.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.WipeAndRingRecycleBothRetireTheProvenanceColumn",
	"[CorrectionCache][ResimGate][Provenance]")
{
	GateCache cache(nullptr);

	predictTick(cache, 100u, 1);
	predictTick(cache, 101u, 2);
	predictTick(cache, 102u, 3);
	landCorrection(cache, 102u);
	predictTick(cache, 103u, 4);
	REQUIRE(provenanceRun(cache, 100u, 103u) == "PPAP");

	SECTION("wipeCache retires EVERY slot to Empty, including the renumbered frontier")
	{
		cache.wipeCache(500u);

		REQUIRE(cache.getPredictionTick() == 500u);
		// ⚠ ALL 60, not "all but the frontier": `wipeCache` sets the frontier
		// slot's TICK, never its state, so no state has been written for tick 500
		// either. The next `pushPredictionState` is what fills it.
		REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::Empty)
		        == static_cast<std::uint32_t>(GateCache::StateBufferSize));
		REQUIRE(provenanceRun(cache, 500u, 500u) == ".");
	}

	SECTION("a ring recycle retires the old lineage at the same instant as the bit")
	{
		const std::uint32_t recycledIndex = cache.getCacheIndex(102u);
		REQUIRE(cache.getDiagnosticStateProvenance(recycledIndex)
		        == SlotStateProvenance::AuthorityAdopted);
		REQUIRE(cache.containsCorrectTick(recycledIndex));

		// One full lap of the ring, so slot `recycledIndex` is reallocated.
		for (std::uint32_t tick = 104u; tick <= 163u; ++tick)
			predictTick(cache, tick, static_cast<std::int32_t>(tick));

		// ⭐ `Predicted`, not `Empty`: `pushPredictionTick` allocates a PREDICTION
		// slot and production always follows it with `pushPredictionState`.
		REQUIRE(cache.getDiagnosticStateProvenance(recycledIndex)
		        == SlotStateProvenance::Predicted);
		// The twin: the correction BIT retired at the same instant, so the column
		// and the bit cannot disagree about whether a correction survived a lap.
		REQUIRE_FALSE(cache.containsCorrectTick(recycledIndex));
		REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::AuthorityAdopted) == 0u);
		REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::Empty) == 0u);
	}
}

// ---------------------------------------------------------------------------
// ⭐⭐ `ReplayedOverCorrection` — UNREACHABLE UNDER THE SHIPPED DEFAULT, AND
// THE ALARM PROVEN LIVE ANYWAY.
//
// This is the case that stops "the value reads zero everywhere" from being a
// TEST THAT CANNOT FAIL. Section A drives a full lifecycle and asserts zero.
// Section B FORGES the guard-failure precondition the shipped code makes
// unreachable — authority-grade PROVENANCE over a CLEAR `m_containsCorrectTick`,
// i.e. the bit and the column disagreeing — and asserts the write site stamps
// the alarm. Without section B, section A proves only that nothing happened.
//
// ⚠ THE FORGERY IS NOT A BACKDOOR INTO PRODUCTION. It writes the DIAGNOSTIC
// COLUMN through the same fence-test seam the independence case uses, and the
// replay write is then decided by `m_containsCorrectTick` exactly as always
// (`Written`, because the bit is clear). What the forgery exercises is the
// SECOND, INDEPENDENT question the write site asks — "was the bit telling the
// truth?" — which is the only thing that can ever produce this value.
//
// ⚠ ONE KNOWN BOUND, recorded rather than defended against: the value is NOT
// STICKY. `isAuthorityGradeProvenance(ReplayedOverCorrection)` is false (pinned
// by the alphabet case), so a LATER replay over the same still-uncorrected slot
// restamps it `Replayed` and the alarm is erased. That window is closed in
// practice by WHERE the dump fires: `[ResimProbe.SlotMap]` is emitted at
// `[Resim.Finish]`, i.e. at the end of the very resim that stamped the `X`, and
// no second resim can run before it. Stickiness was rejected because it would
// make the column describe HISTORY rather than the CURRENT lineage of the state
// in the slot, which is the whole contract.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.ReplayedOverCorrectionIsUnreachableButTheAlarmIsWired",
	"[CorrectionCache][ResimGate][Provenance]")
{
	GateCache cache(nullptr);

	SECTION("A — a full lifecycle with corrections on both sides of the span produces none")
	{
		ProductionTrace ignored;
		driveTracedLifecycle(cache, ignored, 0u, 0u, 0u, /*extraLandingTick*/ 107u);

		REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::ReplayedOverCorrection) == 0u);
		// Not vacuous: the lifecycle really did replay, and really did protect.
		REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::Replayed) > 0u);
		REQUIRE(countProvenanceInRing(cache, SlotStateProvenance::AuthorityAdopted) > 0u);
	}

	SECTION("B — forge the bit/column disagreement and the write site raises the alarm")
	{
		for (std::uint32_t tick = 100u; tick <= 108u; ++tick)
			predictTick(cache, tick, static_cast<std::int32_t>(tick));
		landCorrection(cache, 104u);
		prepareResimulation(cache);

		// The alarm slot and its twin, TWO SLOTS APART so the scribble's cycle
		// gives them different forged values. Both are uncorrected, so protect-all
		// lets the replay into both — the ONLY thing that differs between them is
		// what the column claims about the state already there.
		const std::uint32_t forgedIndex = cache.getCacheIndex(108u);
		const std::uint32_t twinIndex   = cache.getCacheIndex(106u);
		REQUIRE_FALSE(cache.containsCorrectTick(forgedIndex));
		REQUIRE_FALSE(cache.containsCorrectTick(twinIndex));

		// Choose the seed so slot `forgedIndex` lands on AuthorityAdopted: the
		// generator writes `(seed + i) % kSlotStateProvenanceCount`, and
		// 96 == 16 * 6 keeps the subtraction non-negative for every index.
		const std::uint32_t seed = 96u
			+ static_cast<std::uint32_t>(SlotStateProvenance::AuthorityAdopted)
			- forgedIndex;
		cache.scribbleDiagnosticStateProvenanceForFenceTest(seed);

		// The forged state: the column claims authority lineage, the BIT — which
		// is what protect-all actually reads — says the slot is unprotected.
		REQUIRE(cache.getDiagnosticStateProvenance(forgedIndex)
		        == SlotStateProvenance::AuthorityAdopted);

		// ⭐ THE TWIN, in the same scribbled cache and the same replay sweep: the
		// forgery left this slot NON-authority-grade, so the alarm must NOT fire
		// on it. Without this, section B would only prove the write site stamps
		// something — not that it discriminates.
		REQUIRE_FALSE(isAuthorityGradeProvenance(cache.getDiagnosticStateProvenance(twinIndex)));
		REQUIRE(replayTick(cache, 106u) == resimGate::ResimSlotWriteOutcome::Written);
		REQUIRE(cache.getDiagnosticStateProvenance(twinIndex) == SlotStateProvenance::Replayed);

		// And the alarm slot, replayed in the same ascending sweep: allowed
		// through by the bit, exactly as in shipped code...
		REQUIRE(replayTick(cache, 108u) == resimGate::ResimSlotWriteOutcome::Written);
		// ...and the second, independent question fires the alarm.
		REQUIRE(cache.getDiagnosticStateProvenance(forgedIndex)
		        == SlotStateProvenance::ReplayedOverCorrection);
		REQUIRE(slotStateProvenanceChar(cache.getDiagnosticStateProvenance(forgedIndex)) == 'X');
	}
}

// ---------------------------------------------------------------------------
// ⛔⛔ FENCE 2 — **THIS CASE IS THE FENCE.** THE PROVENANCE COLUMN CANNOT REACH
// ANY PRODUCTION OUTPUT.
//
// Item 45 retired `m_isResimulated` because production logic derived from a
// per-slot column had failed SILENTLY for months, and `CorrectionCache.h`'s
// retirement block warns that re-adding such a column re-creates that hazard.
// Item 48 overrides that warning, and THIS is what makes the override safe:
// re-deriving anything in production from this column is now a RED TEST rather
// than a discouraged practice.
//
// HOW IT WORKS. One fixed gate lifecycle — predict, land (disagreeing and
// agreeing), open the gate, prepare, replay the span with a mid-replay landing,
// consume, then keep going past the consume — is driven once CLEAN and then
// again for EVERY combination of garbage the generator can produce at the three
// points item 48 names (gate OPEN, MID-PREPARE, POST-CONSUME). After every step
// each run records a trace of the cache's ENTIRE production surface: the gate,
// the anchor, both prepare-time captures, the landing counter and every per-slot
// stamp, the trigger policy, every slot's correction bit / applied-capture ref /
// STATE, the tick buffer, the anomalous-miss log gate, the correction verdict,
// every replay write outcome, and the DETERMINISM CHECKSUM.
//
// Every trace must be byte-identical to the clean one.
// `getDiagnosticStateProvenance` is the one thing deliberately absent from the
// trace — that is the only column the runs differ in, so equality means nothing
// else can see it.
//
// ⛔⛔ THE 6^3 SWEEP IS NOT THOROUGHNESS THEATRE — A MUTATION RUN PROVED A SINGLE
// SEED INSUFFICIENT. The first version scribbled one seed (deriving the other
// two points from it) and a planted leak SURVIVED IT GREEN: `getLastCorrectTick`
// was made to skip any corrected slot whose provenance read `Replayed`, and the
// case did not notice. The generator writes `(seed + slot) % 6`, so one seed
// gives each slot exactly ONE of six values, and the newest corrected slot never
// drew the poisoned one. Sweeping the three points independently over all six
// rotations is EXHAUSTIVE over this generator — every slot takes every value at
// every point somewhere in the sweep — and the same leak then fails on the first
// combination that reaches it. If you shrink this loop, re-run that mutant.
//
// ⭐ AND THE LAST RUN IS WHY THE EQUALITY MEANS ANYTHING. A trace that captured
// nothing would compare equal forever. The sensitivity run changes ONE REAL
// INPUT (an extra correction landing mid-replay) and the case REQUIRES the trace
// to MOVE. So the instrument is proven sensitive in the same case that proves
// the column is invisible.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionCache.ResimGate.TheProvenanceColumnCannotReachAnyProductionOutput",
	"[CorrectionCache][ResimGate][Provenance]")
{
	ProductionTrace clean;
	{
		GateCache cache(nullptr);
		driveTracedLifecycle(cache, clean, 0u, 0u, 0u, /*extraLandingTick*/ 0u);
	}
	REQUIRE(clean.values.size() > 1000u);   // the trace is substantial, not a stub

	// ⛔ THE FENCE, swept exhaustively over the generator's whole output space.
	// The `+ 1u` keeps every seed nonzero (0 means "do not scribble") while the
	// three loops still cover all six rotations at each point.
	std::uint32_t poisonedRuns = 0u;
	for (std::uint32_t gateOpen = 0u; gateOpen < kSlotStateProvenanceCount; ++gateOpen)
	for (std::uint32_t midPrepare = 0u; midPrepare < kSlotStateProvenanceCount; ++midPrepare)
	for (std::uint32_t postConsume = 0u; postConsume < kSlotStateProvenanceCount; ++postConsume)
	{
		ProductionTrace scribbled;
		GateCache       cache(nullptr);
		driveTracedLifecycle(cache, scribbled,
			gateOpen + 1u, midPrepare + 1u, postConsume + 1u, /*extraLandingTick*/ 0u);

		REQUIRE(scribbled == clean);

		// The scribble really did poison the column on this combination —
		// otherwise the comparison above would compare two identical runs and
		// prove nothing. `X` can only appear because a REPLAY landed on a slot the
		// forgery had marked authority-grade, so its presence is direct evidence
		// the garbage reached the write path.
		if (countProvenanceInRing(cache, SlotStateProvenance::ReplayedOverCorrection) > 0u)
			++poisonedRuns;
	}
	REQUIRE(poisonedRuns > 0u);

	// ⭐ THE SENSITIVITY PROOF: one real input moved, and the trace noticed.
	ProductionTrace sensitivity;
	{
		GateCache cache(nullptr);
		driveTracedLifecycle(cache, sensitivity, 0u, 0u, 0u, /*extraLandingTick*/ 107u);
	}
	REQUIRE(sensitivity.values.size() == clean.values.size());
	REQUIRE_FALSE(sensitivity == clean);
}

#endif // WITH_LOW_LEVEL_TESTS
