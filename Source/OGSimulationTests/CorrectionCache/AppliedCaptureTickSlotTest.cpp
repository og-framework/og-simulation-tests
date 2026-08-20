// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionCache.h"

#include <cstdint>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T4 (design finding D3): WHERE the per-tick
// applied-capture-tick reference is stored on the client.
//
// The ruling is the CORRECTION-CACHE SLOT, parallel to the state it corrects —
// and the reason it must be a slot and not one scalar per character is T6: resim
// resolves remote input for EVERY tick it replays, so it needs the reference
// that belonged to each of those ticks. A single "latest ref" stash would answer
// the newest correction's question for all of them and silently feed the wrong
// input into every earlier resim tick.
//
// These cases pin exactly that property, plus the three ways a slot can hold NO
// reference (never corrected / the D1 underrun sentinel / recycled by the
// prediction ring), which are deliberately indistinguishable to a reader: all
// three mean "do not try to look a capture up for this tick".
//////////////////////////////////////////////////////////////////////////////

namespace
{
	struct RefState
	{
		std::int32_t value = 0;

		// The cache compares prediction against correction on the hit path.
		bool isSimilarTo(const RefState& other) const { return value == other.value; }
	};

	struct RefInput
	{
		std::int32_t value = 0;
	};

	using RefCache = StateCorrectionCache<RefState, RefInput>;

	// One predicted tick, exactly as prepareSimulationStep + postPredictionAll produce it.
	//
	// [og-netcode-v2-input-relay T16] The `pushPredictionInput(RefInput{ value })`
	// between these two lines is gone with the cache's input column — as is the
	// call in prepareSimulationStep this helper mirrors. Nothing below asserted on it:
	// every assertion in this file reads the applied-capture-tick REF, which is
	// what the column's retirement leaves the slot carrying. Assertion count
	// unchanged.
	void predictTick(RefCache& cache, std::uint32_t tick, std::int32_t value)
	{
		cache.pushPredictionTick(tick);
		cache.pushPredictionState(RefState{ value });
	}

	// The reference the cache holds for `tick`, or the sentinel when that tick has
	// no slot at all.
	std::uint32_t refAt(const RefCache& cache, std::uint32_t tick)
	{
		const std::uint32_t idx = cache.getCacheIndex(tick);
		if (idx == RefCache::InvalidCacheIndex)
			return kNoInputCaptureTick;
		return cache.getAppliedCaptureTick(idx);
	}
} // namespace

// ---------------------------------------------------------------------------
// THE headline property: PER TICK, not per character. Three corrected ticks,
// three different references, all readable at once.
// ---------------------------------------------------------------------------
TEST_CASE("Correction cache stores a distinct applied capture tick per tick",
	"[CorrectionCache][InputRelay]")
{
	RefCache cache(nullptr);

	predictTick(cache, 100u, 1);
	predictTick(cache, 101u, 2);
	predictTick(cache, 102u, 3);

	// The authority applied a different capture at each tick — the ordinary case
	// once input delay and network jitter are in play.
	cache.tryInsertingCorrectState(RefState{ 1 }, 100u, 93u);
	cache.tryInsertingCorrectState(RefState{ 2 }, 101u, 94u);
	cache.tryInsertingCorrectState(RefState{ 3 }, 102u, 96u);

	REQUIRE(refAt(cache, 100u) == 93u);
	REQUIRE(refAt(cache, 101u) == 94u);
	REQUIRE(refAt(cache, 102u) == 96u);

	// A single-scalar stash would answer 96 (the newest) to all three — this is
	// the assertion that fails under that implementation.
	REQUIRE_FALSE(refAt(cache, 100u) == refAt(cache, 102u));

	// The reference is not the tick it is stored at: they are two different
	// clocks' numbering of the same event.
	REQUIRE_FALSE(refAt(cache, 100u) == 100u);
}

// ---------------------------------------------------------------------------
// The three "no reference" situations.
// ---------------------------------------------------------------------------
TEST_CASE("Correction cache reports the sentinel wherever no capture is known",
	"[CorrectionCache][InputRelay]")
{
	RefCache cache(nullptr);

	predictTick(cache, 200u, 1);

	// (1) Predicted but never corrected — nothing has told us anything yet.
	REQUIRE(refAt(cache, 200u) == kNoInputCaptureTick);

	// (2) Corrected, but the authority substituted an input (D1). The correction
	// says so explicitly rather than repeating a stale capture tick.
	cache.tryInsertingCorrectState(RefState{ 1 }, 200u, kNoInputCaptureTick);
	REQUIRE(refAt(cache, 200u) == kNoInputCaptureTick);

	// A real reference lands normally afterwards — the sentinel is a per-tick
	// classification, never a latch.
	predictTick(cache, 201u, 2);
	cache.tryInsertingCorrectState(RefState{ 2 }, 201u, 195u);
	REQUIRE(refAt(cache, 201u) == 195u);

	// (3) A tick with no slot at all is outside the window — refAt's own guard,
	// mirroring SimulationReconciliation::getAppliedCaptureTick returning nullopt.
	REQUIRE(cache.getCacheIndex(9999u) == RefCache::InvalidCacheIndex);

	// Capture tick 0 is an ORDINARY session-start capture and must survive as
	// itself — the same discriminator the wire and netsync suites pin.
	predictTick(cache, 202u, 3);
	cache.tryInsertingCorrectState(RefState{ 3 }, 202u, 0u);
	REQUIRE(refAt(cache, 202u) == 0u);
	REQUIRE_FALSE(refAt(cache, 202u) == kNoInputCaptureTick);
}

// ---------------------------------------------------------------------------
// Slot recycling. The ring reuses a slot every StateBufferSize ticks; the
// previous occupant's reference must retire with its tick, or a resim through
// the fresh tick would resolve input from a capture 60 ticks old and be
// confidently wrong rather than merely uninformed.
// ---------------------------------------------------------------------------
TEST_CASE("Correction cache retires the applied capture tick when a slot recycles",
	"[CorrectionCache][InputRelay]")
{
	RefCache cache(nullptr);

	predictTick(cache, 300u, 1);
	cache.tryInsertingCorrectState(RefState{ 1 }, 300u, 291u);
	const std::uint32_t recycledIndex = cache.getCacheIndex(300u);
	REQUIRE(cache.getAppliedCaptureTick(recycledIndex) == 291u);

	// Walk the ring all the way round so tick 300's slot is handed to a new tick.
	for (std::uint32_t tick = 301u; tick <= 300u + RefCache::StateBufferSize; ++tick)
		predictTick(cache, tick, 9);

	const std::uint32_t newTick = 300u + static_cast<std::uint32_t>(RefCache::StateBufferSize);
	REQUIRE(cache.getCacheIndex(newTick) == recycledIndex);
	REQUIRE(cache.getAppliedCaptureTick(recycledIndex) == kNoInputCaptureTick);
	REQUIRE_FALSE(cache.getAppliedCaptureTick(recycledIndex) == 291u);
}

// ---------------------------------------------------------------------------
// A hard resync renumbers the prediction clock, so every surviving reference
// describes a tick numbering that no longer exists — same reasoning as the
// input-delay-line clear in SimulationNetSync::wipeAllForResync.
// ---------------------------------------------------------------------------
TEST_CASE("Correction cache drops applied capture ticks on a resync wipe",
	"[CorrectionCache][InputRelay]")
{
	RefCache cache(nullptr);

	predictTick(cache, 400u, 1);
	predictTick(cache, 401u, 2);
	cache.tryInsertingCorrectState(RefState{ 1 }, 400u, 388u);
	cache.tryInsertingCorrectState(RefState{ 2 }, 401u, 389u);
	REQUIRE(refAt(cache, 400u) == 388u);

	cache.wipeCache(900u);

	for (std::uint32_t idx = 0u; idx < RefCache::StateBufferSize; ++idx)
		REQUIRE(cache.getAppliedCaptureTick(idx) == kNoInputCaptureTick);
}

#endif // WITH_LOW_LEVEL_TESTS
