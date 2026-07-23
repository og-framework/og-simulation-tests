// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionCache.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// Stage 5 / Task 18: correction-miss log gating.
//
// A correction whose tick has no cache slot is discarded. The 2026-07-20
// dedicated-server PIE session showed that is a ROUTINE, self-healing outcome
// (`impl/research_correction_discards.md`): 111 miss events across two clients,
// all on three benign paths — freshly registered remote proxies, the connect
// transient, and post-Skip holes. Reconciliation was demonstrably healthy
// throughout (88 ResimCheck.Divergence resims against 22 steady-state discards).
// Logging every one of those at Warning cost real diagnostic time because the
// shape resembled the v1 T23/T24 hard-lock signature.
//
// StateCorrectionCache::isAnomalousMiss now gates the severity: a miss is
// anomalous only when the missed tick is FURTHER than rollbackWindowHardCap from
// the prediction frontier, and a cache with no frontier at all (registration
// transient) is never anomalous. Routine misses drop to Verbose.
//
// These cases pin the gate on both sides — that the observed benign shapes are
// silenced AND that a genuine window overrun still warns — plus the fact that
// insertion behaviour itself is untouched.
//
// R-P1: the threshold is always read from TimeConfig, never written as a literal,
// so a retune of the field moves these expectations with it.
//////////////////////////////////////////////////////////////////////////////

namespace
{
	struct GateState
	{
		std::int32_t value = 0;

		// tryInsertingCorrectState calls this on the HIT path only; the miss path
		// under test never reaches it. Present so the cache instantiates.
		bool isSimilarTo(const GateState& other) const { return value == other.value; }
	};

	struct GateInput
	{
		std::int32_t value = 0;
	};

	using GateCache = StateCorrectionCache<GateState, GateInput>;

	// Collects every SIMLOG line the cache emits so a case can assert on severity.
	struct LogSink
	{
		std::vector<std::string> lines;

		std::function<void(const char*)> functor()
		{
			return [this](const char* msg) { lines.emplace_back(msg); };
		}

		void clear() { lines.clear(); }

		bool sawWarning() const { return countPrefixed("[Warning]") > 0; }
		bool sawVerbose() const { return countPrefixed("[Verbose]") > 0; }

		std::size_t countPrefixed(const std::string& prefix) const
		{
			std::size_t n = 0;
			for (const std::string& line : lines)
				if (line.rfind(prefix, 0) == 0)
					++n;
			return n;
		}

		// Only miss lines matter to these cases; the hit path also logs.
		std::size_t countMisses() const
		{
			std::size_t n = 0;
			for (const std::string& line : lines)
				if (line.find("discarding") != std::string::npos)
					++n;
			return n;
		}
	};

	// rollbackWindowHardCap as the cache's default gate reads it. Never a literal.
	std::uint32_t hardCap()
	{
		const TimeConfig cfg;
		return static_cast<std::uint32_t>(cfg.rollbackWindowHardCap);
	}

	// Drives the cache to a live prediction frontier at `tick`, then drops the
	// log lines that produced so a case sees only the miss it provokes.
	void seedFrontier(GateCache& cache, LogSink& sink, std::uint32_t tick)
	{
		cache.pushPredictionTick(tick);
		cache.pushPredictionInput(GateInput{ 1 });
		cache.pushPredictionState(GateState{ 1 });
		sink.clear();
	}
} // namespace

TEST_CASE("Correction miss on a fresh cache with no frontier logs Verbose, not Warning",
	"[CorrectionCache][MissGating]")
{
	// The observed `predictionTick=0` events (tick=941 and tick=959 in the
	// 2026-07-20 logs): a remote proxy registered mid-session whose cache has
	// never been pushed. The raw distance is enormous (941 - 0) and would trip any
	// pure-distance gate, so the no-frontier case needs its own exemption.
	LogSink sink;
	GateCache cache(sink.functor());

	REQUIRE(cache.getPredictionTick() == 0);
	REQUIRE(cache.isAnomalousMiss(941u) == false);

	cache.insertCorrectionInput(GateInput{ 7 }, 941u);
	cache.tryInsertingCorrectState(GateState{ 7 }, 941u);

	REQUIRE(sink.countMisses() == 2u);
	REQUIRE(sink.sawWarning() == false);
	REQUIRE(sink.countPrefixed("[Verbose]") == 2u);
}

TEST_CASE("Correction miss within rollbackWindowHardCap logs Verbose",
	"[CorrectionCache][MissGating]")
{
	// The steady-state plus-one/minus-one shapes: tick=618 pred=617,
	// ticks 829/830/831 pred=827, tick=6407 pred=6412. All well inside the cap.
	LogSink sink;
	GateCache cache(sink.functor());
	seedFrontier(cache, sink, 617u);

	REQUIRE(cache.isAnomalousMiss(618u) == false);   // ahead by 1
	REQUIRE(cache.isAnomalousMiss(612u) == false);   // behind by 5

	cache.insertCorrectionInput(GateInput{ 7 }, 618u);
	cache.tryInsertingCorrectState(GateState{ 7 }, 618u);

	REQUIRE(sink.countMisses() == 2u);
	REQUIRE(sink.sawWarning() == false);
	REQUIRE(sink.sawVerbose() == true);
}

TEST_CASE("Correction miss at exactly rollbackWindowHardCap is still routine",
	"[CorrectionCache][MissGating]")
{
	// The single worst observed event — tick=57 predictionTick=37, distance
	// exactly 20 — sits ON the boundary, inside the connect transient that the
	// clock then resolved with a HardResync. The gate is strictly `>` so this
	// stays Verbose; a `>=` gate would have re-armed the very warning this task
	// exists to silence.
	LogSink sink;
	GateCache cache(sink.functor());
	seedFrontier(cache, sink, 37u);

	const std::uint32_t cap = hardCap();
	REQUIRE(cache.isAnomalousMiss(37u + cap) == false);
	REQUIRE(cache.isAnomalousMiss(37u - cap) == false);

	cache.insertCorrectionInput(GateInput{ 7 }, 37u + cap);

	REQUIRE(sink.countMisses() == 1u);
	REQUIRE(sink.sawWarning() == false);
}

TEST_CASE("Correction miss beyond rollbackWindowHardCap still logs Warning",
	"[CorrectionCache][MissGating]")
{
	// The gate must not be vacuous: one tick past the cap is beyond anything the
	// reconciler could ever resimulate, and TimeConfig pins
	// hardResyncThresholdTicks > rollbackWindowHardCap, so a miss out here means
	// the clock should have resynced and did not.
	LogSink sink;
	GateCache cache(sink.functor());
	seedFrontier(cache, sink, 1000u);

	const std::uint32_t beyond = hardCap() + 1u;
	REQUIRE(cache.isAnomalousMiss(1000u + beyond) == true);
	REQUIRE(cache.isAnomalousMiss(1000u - beyond) == true);

	cache.insertCorrectionInput(GateInput{ 7 }, 1000u + beyond);
	cache.tryInsertingCorrectState(GateState{ 7 }, 1000u - beyond);

	REQUIRE(sink.countMisses() == 2u);
	REQUIRE(sink.countPrefixed("[Warning]") == 2u);
	REQUIRE(sink.sawVerbose() == false);
}

TEST_CASE("Miss gating is severity only — insertion behaviour is unchanged",
	"[CorrectionCache][MissGating]")
{
	// AC: "No behavioural change to correction insertion itself." A miss must stay
	// a pure no-op, and a hit must still land, regardless of which severity the
	// gate selected.
	LogSink sink;
	GateCache cache(sink.functor());
	seedFrontier(cache, sink, 500u);

	const std::uint32_t landedTickBefore = cache.getPredictionTick();

	// Routine miss (Verbose) — nothing written.
	cache.insertCorrectionInput(GateInput{ 42 }, 501u);
	REQUIRE(cache.getPredictionTick() == landedTickBefore);
	REQUIRE(cache.getLastCorrectTick() == 0u);

	// Anomalous miss (Warning) — also nothing written.
	cache.insertCorrectionInput(GateInput{ 42 }, 500u + hardCap() + 5u);
	REQUIRE(cache.getPredictionTick() == landedTickBefore);
	REQUIRE(cache.getLastCorrectTick() == 0u);

	// A correction that DOES hit the window still lands, unchanged.
	cache.tryInsertingCorrectState(GateState{ 99 }, 500u);
	REQUIRE(cache.getLastCorrectTick() == 500u);

	const std::uint32_t cacheIndex = cache.getCacheIndex(500u);
	REQUIRE(cacheIndex != GateCache::InvalidCacheIndex);
	REQUIRE(cache.getState(cacheIndex).value == 99);
}

TEST_CASE("The configured gate distance is honoured, not a baked-in constant",
	"[CorrectionCache][MissGating]")
{
	// Pins the setter seam and proves the threshold is read from a member rather
	// than hardcoded at the comparison site.
	LogSink sink;
	GateCache cache(sink.functor());
	seedFrontier(cache, sink, 200u);

	REQUIRE(cache.isAnomalousMiss(200u + hardCap()) == false);

	cache.setAnomalousMissDistanceTicks(2u);
	REQUIRE(cache.isAnomalousMiss(203u) == true);
	REQUIRE(cache.isAnomalousMiss(202u) == false);
}

#endif // WITH_LOW_LEVEL_TESTS
