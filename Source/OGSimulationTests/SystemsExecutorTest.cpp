// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <tuple>
#include <type_traits>

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SystemsExecutor.h"
#include "OGSimulation/SimulationTimeContext.h"

// ---------------------------------------------------------------------------
// Coverage for the SimulationSystem concept + SimulationSystemsExecutor +
// NullSystemsExecutor (task 5 of the ogsim-system-api initiative — see
// system_api_design.md §3.11 / §4.1 / §8.3).
//
// task-5-LOCAL isolation (CB-1 review finding): this TU uses ONLY scratch mock
// types — a MockStaticData, a MockSimulatable, and ad-hoc SimulatableList<...>s.
// It deliberately does NOT reference BrawlerSimulatables /
// simulatableBrawler::StaticData / brawlerHitRouting::System — those don't exist
// until task 6, and task 5 must compile/verify in complete isolation.
// ---------------------------------------------------------------------------

namespace
{
	// --- task-5-LOCAL mock types -------------------------------------------
	struct MockStaticData {};

	struct MockSimulatable { int hp = 0; };
	struct MockOther       { int fuel = 0; };  // a second simulatable for subset tests

	using MockList  = SimulatableList<MockSimulatable>;
	using MockView  = StorageView<MockSimulatable>;
	using MockStore = SimulationObjectStorage<MockSimulatable>;

	// --- A well-formed system: satisfies SimulationSystem<_, MockStaticData> -
	// Records which hooks fired (and with what) so the executor's fold-firing can
	// be observed from the outside via get<MockSystem>(). Hooks take the view
	// BY VALUE (the design's convention).
	struct MockSystem
	{
		using RequiredSimulatables = MockList;

		int  preCount  = 0;
		int  postCount = 0;
		int  regCount  = 0;
		int  unregCount = 0;
		unsigned int lastId  = 0;
		unsigned int lastTick = 0;
		int  observedHpSum = 0;   // written by postIntegrate — proves the view reaches storage

		void preIntegrate(const SimulationTimeStep& step, MockView /*view*/, const MockStaticData&)
		{
			++preCount;
			lastTick = step.getTick();
		}

		void postIntegrate(const SimulationTimeStep& step, MockView view, const MockStaticData&)
		{
			++postCount;
			lastTick = step.getTick();
			observedHpSum = 0;
			view.forEachSimulatable<MockSimulatable>(
				[&](unsigned int, MockSimulatable& m) { observedHpSum += m.hp; });
		}

		void onCharacterRegistered(unsigned int id, MockView, const MockStaticData&)
		{
			++regCount;
			lastId = id;
		}

		void onCharacterUnregistered(unsigned int id, MockView, const MockStaticData&)
		{
			++unregCount;
			lastId = id;
		}
	};

	// --- A system taking the view BY CONST-REF (NEW-4 honest scope): still
	//     satisfies the concept -----------------------------------------------
	struct ConstRefViewSystem
	{
		using RequiredSimulatables = MockList;
		void preIntegrate(const SimulationTimeStep&, const MockView&, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, const MockView&, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, const MockView&, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, const MockView&, const MockStaticData&) {}
	};

	// --- BAD system 1: missing the RequiredSimulatables alias entirely -------
	struct NoRequiredSimulatables
	{
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&) {}
	};

	// --- BAD system 2: RequiredSimulatables is NOT a SimulatableList<> --------
	struct RequiredIsNotAList
	{
		using RequiredSimulatables = MockSimulatable;  // raw type — should trip the S9 gate
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&) {}
	};

	// --- BAD system 3: a hook signature mismatch (postIntegrate returns int) --
	struct HookSignatureMismatch
	{
		using RequiredSimulatables = MockList;
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		int  postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) { return 0; }  // not -> void
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&) {}
	};

	// --- BAD system 4: missing a hook entirely (no onCharacterUnregistered) ---
	struct MissingHook
	{
		using RequiredSimulatables = MockList;
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
	};

	// SFINAE-friendly detection of whether a SimulationSystemsExecutor<...> is
	// instantiable for a given system — the specialization's requires-clause makes
	// an ill-formed system select the UNDEFINED primary template (an incomplete
	// type), so `sizeof` is ill-formed and this concept resolves to false rather
	// than a hard error.
	template <typename Sys>
	concept ExecutorInstantiable =
		requires { sizeof(SimulationSystemsExecutor<MockList, MockStaticData, Sys>); };
}

// --- Concept satisfaction (positive) ---------------------------------------
static_assert(SimulationSystem<MockSystem, MockStaticData>);
static_assert(SimulationSystem<ConstRefViewSystem, MockStaticData>);   // by-const-ref view is honest scope (NEW-4)

// --- Concept diagnostics (negative) — each bad system fails the concept ----
static_assert(!SimulationSystem<NoRequiredSimulatables, MockStaticData>);  // no RequiredSimulatables
static_assert(!SimulationSystem<RequiredIsNotAList, MockStaticData>);      // RequiredSimulatables not a SimulatableList<> (S9)
static_assert(!SimulationSystem<HookSignatureMismatch, MockStaticData>);   // postIntegrate not -> void (S8)
static_assert(!SimulationSystem<MissingHook, MockStaticData>);             // onCharacterUnregistered absent

// --- Executor instantiation gate: only a well-formed system instantiates ---
static_assert(ExecutorInstantiable<MockSystem>);
static_assert(!ExecutorInstantiable<NoRequiredSimulatables>);
static_assert(!ExecutorInstantiable<RequiredIsNotAList>);
static_assert(!ExecutorInstantiable<HookSignatureMismatch>);
static_assert(!ExecutorInstantiable<MissingHook>);

// --- NullSystemsExecutor is a valid (vacuous) executor type ----------------
static_assert(std::is_same_v<
	NullSystemsExecutor<MockList, MockStaticData>,
	SimulationSystemsExecutor<MockList, MockStaticData>>);

TEST_CASE("SystemsExecutor.FiresAllFourHooks", "[SystemsExecutor]")
{
	MockStore storage;
	storage.add<MockSimulatable>(1, MockSimulatable{ 10 });
	storage.add<MockSimulatable>(2, MockSimulatable{ 20 });
	MockStaticData sd;

	SimulationSystemsExecutor<MockList, MockStaticData, MockSystem> exec;

	SECTION("firePreIntegrate calls preIntegrate once, threading the step")
	{
		exec.firePreIntegrate(SimulationTimeStep(42, false), storage, sd);
		const MockSystem& s = exec.get<MockSystem>();
		REQUIRE(s.preCount == 1);
		REQUIRE(s.postCount == 0);
		REQUIRE(s.lastTick == 42);
	}

	SECTION("firePostIntegrate calls postIntegrate; the view reaches live storage")
	{
		exec.firePostIntegrate(SimulationTimeStep(7, false), storage, sd);
		const MockSystem& s = exec.get<MockSystem>();
		REQUIRE(s.postCount == 1);
		REQUIRE(s.lastTick == 7);
		REQUIRE(s.observedHpSum == 30);   // 10 + 20 through the projected view
	}

	SECTION("notifyCharacterRegistered / Unregistered fan out with the id, no step")
	{
		exec.notifyCharacterRegistered(5, storage, sd);
		exec.notifyCharacterUnregistered(9, storage, sd);
		const MockSystem& s = exec.get<MockSystem>();
		REQUIRE(s.regCount == 1);
		REQUIRE(s.unregCount == 1);
		REQUIRE(s.lastId == 9);   // last write wins
	}
}

TEST_CASE("SystemsExecutor.PiecewiseConstruct", "[SystemsExecutor]")
{
	// A system whose only ctor takes an int — reached via std::piecewise_construct.
	struct SeededSystem
	{
		using RequiredSimulatables = MockList;
		int seed;
		explicit SeededSystem(int s) : seed(s) {}
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&) {}
	};

	static_assert(SimulationSystem<SeededSystem, MockStaticData>);

	SimulationSystemsExecutor<MockList, MockStaticData, SeededSystem> exec(
		std::piecewise_construct, std::make_tuple(1337));

	REQUIRE(exec.get<SeededSystem>().seed == 1337);
}

TEST_CASE("SystemsExecutor.NullExecutorAllFourFireMethodsAreNoOps", "[SystemsExecutor]")
{
	// SB-3 fix: verify the zero-systems flavor is a real, callable executor whose
	// four fire methods are empty folds — they compile and produce no observable
	// side effects (the storage is untouched by any system).
	MockStore storage;
	storage.add<MockSimulatable>(1, MockSimulatable{ 100 });
	MockStaticData sd;

	NullSystemsExecutor<MockList, MockStaticData> nullExec;

	// All four fire methods compile and run as empty folds.
	nullExec.firePreIntegrate(SimulationTimeStep(1, false), storage, sd);
	nullExec.firePostIntegrate(SimulationTimeStep(2, false), storage, sd);
	nullExec.notifyCharacterRegistered(3, storage, sd);
	nullExec.notifyCharacterUnregistered(4, storage, sd);

	// No system exists, so nothing mutated the storage.
	REQUIRE(storage.get<MockSimulatable>(1).hp == 100);
	SUCCEED("NullSystemsExecutor fire methods are no-ops");
}

#endif // WITH_LOW_LEVEL_TESTS
