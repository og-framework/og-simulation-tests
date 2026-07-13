// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <tuple>
#include <type_traits>

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/StorageView.h"

// ---------------------------------------------------------------------------
// Coverage for StorageView<...> + SimulationObjectStorage::projectTo<>()
// (tasks 2 & 3 of the ogsim-system-api initiative — see system_api_design.md
// §3.11 / §4.1 / §5.3).
//
// StorageView reaches into a storage it doesn't fully know via an opaque handle
// + per-type function-pointer thunks. Its constructor is PRIVATE with a single
// `friend class SimulationObjectStorage` grant (design §4.1), so the ONLY way to
// mint a view is through the real production entry point,
// SimulationObjectStorage::projectTo<SimulatableList<...>>(). This TU therefore
// constructs a real SimulationObjectStorage, projects it, and exercises the view
// end-to-end — validating the friend grant, the thunk binding to the promoted-
// public per-type forEachSimulatable<T>, and the projectTo<> constraint gates.
//
// NOTE (task 3 history): task 2 shipped StorageView with a temporary PUBLIC ctor
// and a MockStorage stand-in, because projectTo<> didn't exist yet. Task 3
// restored the design's friend-only ctor and retired the mock in favour of the
// real storage. A view is trivially destructible (opaque pointer + a tuple of
// function-pointer PODs), so it owns no heap — proven at compile time by the
// static_asserts below; this is a stronger guarantee than the mock's former
// runtime allocation counter (which could only observe a single call path).
// ---------------------------------------------------------------------------

namespace
{
	// --- Mock simulatable types --------------------------------------------
	struct MockBrawler { int hp = 0; };
	struct MockVehicle { int fuel = 0; };
	struct MockUnrelated { int x = 0; };  // deliberately NOT in any storage below

	using BrawlerStorage = SimulationObjectStorage<MockBrawler>;
	using BrawlerVehicleStorage = SimulationObjectStorage<MockBrawler, MockVehicle>;

	// --- Concept-diagnostic probes -----------------------------------------
	// SFINAE-friendly detection of whether a view exposes get<T> /
	// forEachSimulatable<T> for a given T. Because the accessors are constrained by
	// a requires-clause, naming them for an out-of-pack T is an unsatisfied
	// constraint (a clean substitution failure), so these concepts resolve to false
	// rather than a hard error — exactly the "legible requires-clause diagnostic".
	template <typename V, typename T>
	concept ViewHasGet = requires(V v, unsigned int id) { v.template get<T>(id); };

	template <typename V, typename T>
	concept ViewHasForEach =
		requires(V v) { v.template forEachSimulatable<T>([](unsigned int, T&) {}); };

	// SFINAE-friendly detection of whether projectTo<W>() is well-formed on a given
	// storage. Used to pin the two projectTo<> constraint gates (checks 1 & 2)
	// without tripping list_contains<>'s primary-template hard static_assert.
	template <typename S, typename W>
	concept CanProjectTo = requires(S s) { s.template projectTo<W>(); };
}

// --- StorageView concept-diagnostic checks (task 2 property preserved): get<X> /
//     forEachSimulatable<X> for X not in the view's pack fails on the
//     requires-clause, not a spooky instantiation error ------------------------
static_assert(ViewHasGet<StorageView<MockBrawler>, MockBrawler>);
static_assert(!ViewHasGet<StorageView<MockBrawler>, MockVehicle>);
static_assert(!ViewHasGet<StorageView<MockBrawler>, MockUnrelated>);
static_assert(ViewHasGet<StorageView<MockBrawler, MockVehicle>, MockVehicle>);

static_assert(ViewHasForEach<StorageView<MockBrawler>, MockBrawler>);
static_assert(!ViewHasForEach<StorageView<MockBrawler>, MockVehicle>);

// --- No-owning-storage property: a trivially-destructible value type owns no
//     heap memory, so by-value passing of the view is allocation-free -----------
static_assert(std::is_trivially_destructible_v<StorageView<MockBrawler>>);
static_assert(std::is_trivially_destructible_v<StorageView<MockBrawler, MockVehicle>>);

// --- projectTo<> return type (compile-test: projectTo<SimulatableList<T>>()
//     returns a working StorageView<T>) -----------------------------------------
static_assert(std::is_same_v<
	decltype(std::declval<BrawlerStorage&>().projectTo<SimulatableList<MockBrawler>>()),
	StorageView<MockBrawler>>);
static_assert(std::is_same_v<
	decltype(std::declval<BrawlerVehicleStorage&>().projectTo<SimulatableList<MockBrawler, MockVehicle>>()),
	StorageView<MockBrawler, MockVehicle>>);

// --- projectTo<> constraint gates ------------------------------------------
// Well-formed: a SimulatableList<> subset of the storage's pack projects.
static_assert(CanProjectTo<BrawlerStorage, SimulatableList<MockBrawler>>);
static_assert(CanProjectTo<BrawlerVehicleStorage, SimulatableList<MockBrawler>>);
static_assert(CanProjectTo<BrawlerVehicleStorage, SimulatableList<MockBrawler, MockVehicle>>);

// Concept-diagnostic check 1 (missing wrap): projectTo<MockBrawler>() — a RAW type,
// not wrapped in SimulatableList<> — fails on the FIRST requires conjunct
// (IsSimulatableList). Because that conjunct short-circuits, list_contains<>'s
// primary-template hard static_assert is never instantiated: the mere fact that
// this TU compiles proves the failure is on the concept, NOT the static_assert.
static_assert(!CanProjectTo<BrawlerStorage, MockBrawler>);

// Concept-diagnostic check 2 (SB-4 — well-formed wrap but not a subset):
// projectTo<SimulatableList<MockUnrelated>>() where MockUnrelated is not in the
// storage's pack. IsSimulatableList passes (it IS a SimulatableList), so the
// failure lands on the list_contains_v conjunct — which resolves through
// list_contains<>'s SPECIALIZATION (both args are SimulatableList<>) to `false`,
// again without touching the primary's static_assert.
static_assert(!CanProjectTo<BrawlerStorage, SimulatableList<MockUnrelated>>);
static_assert(!CanProjectTo<BrawlerVehicleStorage, SimulatableList<MockUnrelated>>);

TEST_CASE("StorageView.GetAndForEach", "[StorageView]")
{
	BrawlerStorage storage;
	storage.add<MockBrawler>(7, MockBrawler{ 100 });
	storage.add<MockBrawler>(9, MockBrawler{ 55 });
	storage.add<MockBrawler>(4, MockBrawler{ 30 });

	// The ONLY construction path — projectTo<> through the friend grant.
	StorageView<MockBrawler> view = storage.projectTo<SimulatableList<MockBrawler>>();

	SECTION("get<T> resolves through the thunk to the live object")
	{
		REQUIRE(view.get<MockBrawler>(7).hp == 100);
		REQUIRE(view.get<MockBrawler>(9).hp == 55);
		REQUIRE(view.get<MockBrawler>(4).hp == 30);

		// Mutations through the view reach the underlying storage (T& is live).
		view.get<MockBrawler>(7).hp = 1;
		REQUIRE(storage.get<MockBrawler>(7).hp == 1);
	}

	SECTION("forEachSimulatable<T> visits every object of that type")
	{
		int total = 0;
		int count = 0;
		view.forEachSimulatable<MockBrawler>([&](unsigned int, MockBrawler& b)
		{
			total += b.hp;
			++count;
		});
		REQUIRE(count == 3);
		REQUIRE(total == 100 + 55 + 30);
	}
}

TEST_CASE("StorageView.ForEachAny", "[StorageView]")
{
	BrawlerVehicleStorage storage;
	storage.add<MockBrawler>(1, MockBrawler{ 10 });
	storage.add<MockBrawler>(2, MockBrawler{ 20 });
	storage.add<MockVehicle>(3, MockVehicle{ 5 });

	StorageView<MockBrawler, MockVehicle> view =
		storage.projectTo<SimulatableList<MockBrawler, MockVehicle>>();

	// forEachAny folds the generic lambda across BOTH types in the pack. The body
	// must compile for every type in the view — here an `if constexpr` gate reads a
	// type-specific field, exercising the design's optional-per-type-behaviour idiom.
	int entities = 0;
	int hpSum = 0;
	int fuelSum = 0;
	view.forEachAny([&](unsigned int, auto& simulatable)
	{
		using S = std::remove_reference_t<decltype(simulatable)>;
		++entities;
		if constexpr (std::is_same_v<S, MockBrawler>)
			hpSum += simulatable.hp;
		else if constexpr (std::is_same_v<S, MockVehicle>)
			fuelSum += simulatable.fuel;
	});

	REQUIRE(entities == 3);   // 2 brawlers + 1 vehicle
	REQUIRE(hpSum == 30);
	REQUIRE(fuelSum == 5);
}

#endif // WITH_LOW_LEVEL_TESTS
