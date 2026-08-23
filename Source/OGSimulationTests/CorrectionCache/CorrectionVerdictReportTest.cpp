// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionCache.h"

#include <cstdint>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T24: the correction cache REPORTS the verdict it
// has always computed.
//
// THE VERDICT IS NOT NEW AND MUST NOT BE. `tryInsertingCorrectState` has always
// evaluated `m_stateBuffer[cacheIndex].isSimilarTo(state)`, always stored it in
// `m_predictionWasCorrect`, and always used it to decide whether to overwrite the
// slot. T24 adds a DEFAULTED out-pointer so a caller that knows which character
// the cache belongs to — which this class deliberately does not — can attribute
// it. These cases pin two things:
//
//   1. THE REPORT MATCHES THE BEHAVIOUR. `predictionWasCorrect == false` and "the
//      slot was overwritten with the authority's state" must be the SAME event. A
//      report derived independently of the overwrite decision could drift from it
//      under a later edit, and then the telemetry would describe a divergence the
//      simulation did not act on (or miss one it did). Every case below asserts
//      the reported verdict AND the state the slot ended up holding.
//
//   2. A DISCARDED CORRECTION IS NOT A VERDICT. When the tick has no slot, no
//      comparison happens at all. Reporting `landed == false` — rather than
//      leaving the caller to read a stale or value-initialised
//      `predictionWasCorrect` — is what stops a discard being counted as an
//      agreement. Discards are ROUTINE and self-healing (the cache's own
//      isAnomalousMiss block enumerates three benign causes), so counting them as
//      agreements would put a large, permanently-correct population under the
//      denominator and drive the measured disagreement rate toward zero for
//      reasons that have nothing to do with prediction quality.
//
// BEHAVIOUR-NEUTRALITY, stated as a testable claim rather than an assurance: the
// out-pointer is defaulted, so every pre-T24 call site compiles and behaves
// identically. The case at the bottom exercises the three-argument form and
// asserts the slot outcome is the same as the four-argument form's.
//////////////////////////////////////////////////////////////////////////////

namespace
{
	struct VerdictState
	{
		std::int32_t value = 0;

		bool isSimilarTo(const VerdictState& other) const { return value == other.value; }
	};

	struct VerdictInput
	{
		std::int32_t value = 0;
	};

	using VerdictCache = StateCorrectionCache<VerdictState, VerdictInput>;

	void predictTick(VerdictCache& cache, std::uint32_t tick, std::int32_t value)
	{
		cache.pushPredictionTick(tick);
		cache.pushPredictionState(VerdictState{ value });
	}

	std::int32_t stateAt(const VerdictCache& cache, std::uint32_t tick)
	{
		return cache.getState(cache.getCacheIndex(tick)).value;
	}
} // namespace

// ---------------------------------------------------------------------------
// AGREEMENT — reported, and the slot is left alone.
// ---------------------------------------------------------------------------
TEST_CASE("Correction cache reports an AGREEING prediction and leaves the slot",
	"[CorrectionCache][DivergenceProbe]")
{
	VerdictCache cache(nullptr);
	predictTick(cache, 100u, 7);

	CorrectionInsertVerdict verdict;
	cache.tryInsertingCorrectState(VerdictState{ 7 }, 100u, 93u, &verdict);

	REQUIRE(verdict.landed);
	REQUIRE(verdict.predictionWasCorrect);
	REQUIRE(verdict.tick == 100u);

	// The behaviour half of the claim: an agreeing correction does not overwrite.
	REQUIRE(stateAt(cache, 100u) == 7);
	// ...and the existing bookkeeping is untouched by the report.
	REQUIRE(cache.containsCorrectTick(cache.getCacheIndex(100u)));
	REQUIRE(cache.getAppliedCaptureTick(cache.getCacheIndex(100u)) == 93u);
}

// ---------------------------------------------------------------------------
// DISAGREEMENT — reported, and the slot takes the authority's state. Same event,
// asserted from both sides.
// ---------------------------------------------------------------------------
TEST_CASE("Correction cache reports a DISAGREEING prediction and overwrites the slot",
	"[CorrectionCache][DivergenceProbe]")
{
	VerdictCache cache(nullptr);
	predictTick(cache, 200u, 7);

	CorrectionInsertVerdict verdict;
	cache.tryInsertingCorrectState(VerdictState{ 42 }, 200u, 194u, &verdict);

	REQUIRE(verdict.landed);
	REQUIRE_FALSE(verdict.predictionWasCorrect);
	REQUIRE(verdict.tick == 200u);

	// THE TWO HALVES ARE ONE EVENT. If the reported verdict were derived
	// separately from the overwrite decision, these two assertions could disagree
	// after a future edit — and the telemetry would then describe a divergence the
	// simulation never acted on.
	REQUIRE(stateAt(cache, 200u) == 42);
}

// ---------------------------------------------------------------------------
// A DISCARDED CORRECTION IS NOT A VERDICT — the case that keeps the denominator
// honest.
// ---------------------------------------------------------------------------
TEST_CASE("Correction cache reports a DISCARDED correction as not landed",
	"[CorrectionCache][DivergenceProbe]")
{
	VerdictCache cache(nullptr);
	predictTick(cache, 300u, 7);

	// Tick 9000 has no slot — the routine, self-healing miss the cache's
	// isAnomalousMiss block characterises. No comparison happens.
	CorrectionInsertVerdict verdict;
	cache.tryInsertingCorrectState(VerdictState{ 42 }, 9000u, 8993u, &verdict);

	REQUIRE_FALSE(verdict.landed);
	REQUIRE(verdict.tick == 9000u);

	// `predictionWasCorrect` is meaningless when nothing landed, and it is
	// deliberately left FALSE rather than TRUE: a caller that ignored `landed`
	// would then over-report disagreement (visible, investigated, corrected)
	// rather than silently inflate the agreeing population, which is the failure
	// that would look like a benefit.
	REQUIRE_FALSE(verdict.predictionWasCorrect);

	// Nothing was written anywhere.
	REQUIRE(stateAt(cache, 300u) == 7);
	REQUIRE(cache.getCacheIndex(9000u) == VerdictCache::InvalidCacheIndex);
}

// ---------------------------------------------------------------------------
// A STALE VERDICT IS NEVER LEFT STANDING. The out-parameter is reset on entry,
// so re-using one across calls cannot carry the previous answer forward.
// ---------------------------------------------------------------------------
TEST_CASE("Correction cache resets the verdict on every call",
	"[CorrectionCache][DivergenceProbe]")
{
	VerdictCache cache(nullptr);
	predictTick(cache, 400u, 7);

	CorrectionInsertVerdict verdict;
	cache.tryInsertingCorrectState(VerdictState{ 7 }, 400u, 393u, &verdict);
	REQUIRE(verdict.landed);
	REQUIRE(verdict.predictionWasCorrect);

	// The SAME object, re-used for a correction that misses. Both fields must move.
	cache.tryInsertingCorrectState(VerdictState{ 7 }, 9999u, 9993u, &verdict);
	REQUIRE_FALSE(verdict.landed);
	REQUIRE_FALSE(verdict.predictionWasCorrect);
	REQUIRE(verdict.tick == 9999u);
}

// ---------------------------------------------------------------------------
// BEHAVIOUR-NEUTRALITY — the pre-T24 call shapes are unchanged.
// ---------------------------------------------------------------------------
TEST_CASE("Correction cache behaves identically with no verdict out-pointer",
	"[CorrectionCache][DivergenceProbe]")
{
	VerdictCache reported(nullptr);
	VerdictCache silent(nullptr);

	for (VerdictCache* cache : { &reported, &silent })
	{
		predictTick(*cache, 500u, 7);
		predictTick(*cache, 501u, 8);
	}

	CorrectionInsertVerdict verdict;
	reported.tryInsertingCorrectState(VerdictState{ 42 }, 500u, 493u, &verdict);
	reported.tryInsertingCorrectState(VerdictState{ 8 },  501u, 494u, &verdict);

	// The three-argument form (T4's shape) and the two-argument form (the pre-T4
	// shape, still used by receiveCorrectionState and by several test harnesses).
	silent.tryInsertingCorrectState(VerdictState{ 42 }, 500u, 493u);
	silent.tryInsertingCorrectState(VerdictState{ 8 },  501u, 494u);

	REQUIRE(stateAt(reported, 500u) == stateAt(silent, 500u));
	REQUIRE(stateAt(reported, 501u) == stateAt(silent, 501u));
	REQUIRE(reported.getAppliedCaptureTick(reported.getCacheIndex(500u))
	        == silent.getAppliedCaptureTick(silent.getCacheIndex(500u)));
	REQUIRE(reported.getDiagnostics().lastCorrectTick() == silent.getDiagnostics().lastCorrectTick());
}

#endif // WITH_LOW_LEVEL_TESTS
