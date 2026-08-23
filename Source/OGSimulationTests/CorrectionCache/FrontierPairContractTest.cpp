// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionCache.h"
#include "OGSimulation/SimulationTimeContext.h"

#include <cstdint>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / item 84: THE FRONTIER-PAIR CONTRACT, HARDENED
// IN PLACE (design step 0; design §B.4, §E step 0; review B-1, B-2, B-3, §F).
//
// WHAT THIS PINS. `StateCorrectionCache::pushPredictionTick` allocates a
// frontier slot; `pushPredictionState` completes it. The pair is a
// cross-phase CONVENTION (design §B.3 — it cannot be made structural without
// re-timing the resim gate's trigger, §F), held today by two independently
// written `!= StepKind::Stall` gates in different peers. Item 84 single-
// sources the gate (`stepAllocatesFrontierSlot`, `SimulationTimeContext.h`)
// and adds a loud, cheap detector (`m_frontierSlotAwaitingState`) that catches
// the failure mode that matters — an allocation abandoned without its state
// push — one allocation late.
//
// ⛔ NO DEATH TEST, AND NONE IS COMING (design G Q4, review B-5, Backlog item
// 84 ruling): the LLT harness cannot trap a failed `OG_CHECK` — standalone it
// is a bare `assert` (`OGAssert.h`), which aborts the process. This file pins
// the MECHANICS (set-on-allocation / clear-on-state / unchanged-on-early-
// return / reset-on-wipe) through the diagnostics read seam
// (`getDiagnostics().frontierSlotAwaitingState()`) instead of the check
// itself firing. The `OG_CHECK` remains present-but-untested-by-firing, which
// is the honestly-stated residual this suite does not paper over.
//
// RED/GREEN: this case is the reason the detector exists to be reasoned
// about, not just described. With the three `m_frontierSlotAwaitingState`
// writes (set-true in `pushPredictionTick`'s allocation path, set-false in
// `pushPredictionState`, reset-false in `wipeCache`) locally reverted, every
// `REQUIRE` below that asserts the bit's value goes RED — the case can fail,
// and that failure is quoted in `impl/impl_notes_task84.md` alongside the
// restored GREEN run.
//
// Tagged `[CorrectionCache][FrontierPair]` — `[CorrectionCache]` is already
// an `[@og]` top-level alias (`OgTagAliases.cpp`), so no alias-table edit is
// needed for this file to run under the full suite.
//////////////////////////////////////////////////////////////////////////////

namespace
{
	struct FPState
	{
		std::int32_t value = 0;
	};

	struct FPInput
	{
		std::int32_t value = 0;
	};

	using FPCache = StateCorrectionCache<FPState, FPInput>;

	// Mirrors `SimulationReconciliation::backfillSkippedTick`'s body exactly
	// (`cache.pushPredictionTick(skippedTick); cache.pushPredictionState(priorState);`)
	// — the internal pairing the `Skip` backfill gate performs, on the bare
	// cache this suite drives directly (no `SimulationReconciliation`
	// instance, matching the neighbouring `[CorrectionCache][ResimGate]`
	// suite's own convention).
	void backfillSkippedTickMirror(FPCache& cache, std::uint32_t skippedTick, const FPState& priorState)
	{
		cache.pushPredictionTick(skippedTick);
		cache.pushPredictionState(priorState);
	}

	// THE VALUE-IDENTITY PIN — one static_assert per StepKind enumerator,
	// checked at compile time regardless of which test filter runs. If a
	// fifth StepKind is ever added, this fails to compile until its author
	// makes the one decision `stepAllocatesFrontierSlot` exists to force.
	static_assert(stepAllocatesFrontierSlot(StepKind::Normal)     == (StepKind::Normal     != StepKind::Stall),
		"stepAllocatesFrontierSlot must be value-identical to != StepKind::Stall for Normal");
	static_assert(stepAllocatesFrontierSlot(StepKind::Stall)      == (StepKind::Stall      != StepKind::Stall),
		"stepAllocatesFrontierSlot must be value-identical to != StepKind::Stall for Stall");
	static_assert(stepAllocatesFrontierSlot(StepKind::Skip)       == (StepKind::Skip       != StepKind::Stall),
		"stepAllocatesFrontierSlot must be value-identical to != StepKind::Stall for Skip");
	static_assert(stepAllocatesFrontierSlot(StepKind::HardResync) == (StepKind::HardResync != StepKind::Stall),
		"stepAllocatesFrontierSlot must be value-identical to != StepKind::Stall for HardResync");
} // namespace

TEST_CASE("CorrectionCache.FrontierPair.AnAllocatedSlotMustReceiveItsStateBeforeTheNextAllocation",
	"[CorrectionCache][FrontierPair]")
{
	FPCache cache(nullptr);

	// False at construction — nothing has been allocated yet.
	REQUIRE_FALSE(cache.getDiagnostics().frontierSlotAwaitingState());

	// True after an ALLOCATING pushPredictionTick — the slot is now awaiting
	// its state push.
	cache.pushPredictionTick(100u);
	REQUIRE(cache.getDiagnostics().frontierSlotAwaitingState());

	// False after pushPredictionState — the pair's completing half.
	cache.pushPredictionState(FPState{ 1 });
	REQUIRE_FALSE(cache.getDiagnostics().frontierSlotAwaitingState());

	// A second allocate/complete cycle behaves identically — this is not a
	// one-shot latch.
	cache.pushPredictionTick(101u);
	REQUIRE(cache.getDiagnostics().frontierSlotAwaitingState());
	cache.pushPredictionState(FPState{ 2 });
	REQUIRE_FALSE(cache.getDiagnostics().frontierSlotAwaitingState());

	// UNCHANGED across the tick-equality early-return — stall re-entry does
	// not arm (and, tested here on a bit that IS already armed, does not
	// disarm either): pushPredictionTick with the SAME tick must be a true
	// no-op on this bit, not merely "leaves an already-false bit false".
	cache.pushPredictionTick(102u);
	REQUIRE(cache.getDiagnostics().frontierSlotAwaitingState());
	cache.pushPredictionTick(102u);   // tick == predictionTick -> early return
	REQUIRE(cache.getDiagnostics().frontierSlotAwaitingState());   // still true, untouched
	cache.pushPredictionState(FPState{ 3 });
	REQUIRE_FALSE(cache.getDiagnostics().frontierSlotAwaitingState());

	// False after wipeCache — exercised on a bit that IS mid-pair (an
	// abandoned allocation), which is coverage blind spot (iii)'s own
	// mechanism: the wipe resets the bit by design rather than reporting the
	// abandonment.
	cache.pushPredictionTick(103u);
	REQUIRE(cache.getDiagnostics().frontierSlotAwaitingState());
	cache.wipeCache(200u);
	REQUIRE_FALSE(cache.getDiagnostics().frontierSlotAwaitingState());

	// False after backfillSkippedTick's internal pairing returns (mirrored
	// above) — the Skip backfill gate is a DIFFERENT predicate and pairs
	// tick+state internally in one call, same as any other allocate/complete
	// cycle from the detector's point of view.
	backfillSkippedTickMirror(cache, 201u, FPState{ 4 });
	REQUIRE_FALSE(cache.getDiagnostics().frontierSlotAwaitingState());
}

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / item 91 part A: THE FOURTH BLIND SPOT WAS AN
// OVER-DETECTION FALSE POSITIVE, AND IT IS NOW FIXED.
//
// `save_snapshot`'s existing-slot path (the branch taken whenever `tick` is
// already present in the cache — true whenever a `pushPredictionTick(tick)`
// call for that exact tick already ran) used to fall straight to
// `m_stateBuffer[slot] = state;` without ever touching
// `m_frontierSlotAwaitingState` — functionally identical to
// `pushPredictionState` on that path, but a separately-written one that
// skipped the clear. Left as-is, a `save_snapshot` call completing a
// `pushPredictionTick`-opened pairing would leave the bit falsely `true`,
// arming a spurious `OG_CHECK` crash at the NEXT legitimate
// `pushPredictionTick` allocation — on a cache whose state is entirely
// correct. All three PRE-EXISTING blind spots (see the coverage statement at
// `m_frontierSlotAwaitingState`'s declaration) are UNDER-detection; this one
// was OVER-detection, the more damaging direction and precisely the class
// the coverage statement claims to enumerate.
//
// ⛔ NO DEATH TEST HERE EITHER, same reason as the file banner above: a
// second `pushPredictionTick` call after an unfixed `save_snapshot` would
// trip the `OG_CHECK` and abort the whole process (a bare `assert`
// standalone), not fail this one case. The diagnostics-seam read
// immediately after `save_snapshot` is the full, safe falsification.
//////////////////////////////////////////////////////////////////////////////

TEST_CASE("CorrectionCache.FrontierPair.SaveSnapshotOnAnExistingSlotClearsTheAwaitingBit",
	"[CorrectionCache][FrontierPair]")
{
	FPCache cache(nullptr);

	// Open a pairing the ordinary way...
	cache.pushPredictionTick(300u);
	REQUIRE(cache.getDiagnostics().frontierSlotAwaitingState());

	// ...then complete it via save_snapshot instead of pushPredictionState.
	// save_snapshot's own doc comment calls itself "the externally-driven
	// twin of pushPredictionTick + pushPredictionState" — completing the pair
	// this way must clear the bit exactly as pushPredictionState does.
	//
	// ⛔ RED AGAINST TODAY'S (PRE-FIX) CODE: this REQUIRE_FALSE fails, because
	// save_snapshot's existing-slot path never touches the bit at all.
	cache.save_snapshot(300u, FPState{ 42 });
	REQUIRE_FALSE(cache.getDiagnostics().frontierSlotAwaitingState());
}

TEST_CASE("CorrectionCache.FrontierPair.SaveSnapshotOnAnOlderResidentTickDoesNotSwallowAGenuinelyOpenPairing",
	"[CorrectionCache][FrontierPair]")
{
	// The fix is guarded on `tick == getPredictionTick()`, not an
	// unconditional clear like pushPredictionState's own (which always
	// targets the frontier slot by construction — save_snapshot's tick is
	// caller-supplied, and getCacheIndex scans the WHOLE ring, so an
	// existing-slot hit can legitimately name an OLDER resident tick, not the
	// frontier). Pin that an unrelated save_snapshot call cannot swallow a
	// still-genuinely-open frontier pairing — the same swallowing failure
	// mode blind spot (iii) describes for wipeCache, in a new disguise.
	FPCache cache(nullptr);

	cache.pushPredictionTick(300u);
	cache.save_snapshot(300u, FPState{ 1 });   // closes 300's pairing, bit false
	REQUIRE_FALSE(cache.getDiagnostics().frontierSlotAwaitingState());

	cache.pushPredictionTick(400u);   // opens 400's pairing; 300's slot stays resident (different ring index)
	REQUIRE(cache.getDiagnostics().frontierSlotAwaitingState());

	cache.save_snapshot(300u, FPState{ 2 });   // existing slot, but NOT the frontier tick (400)
	REQUIRE(cache.getDiagnostics().frontierSlotAwaitingState());   // still true — 400's pairing is untouched

	cache.pushPredictionState(FPState{ 3 });
	REQUIRE_FALSE(cache.getDiagnostics().frontierSlotAwaitingState());
}

#endif // WITH_LOW_LEVEL_TESTS
