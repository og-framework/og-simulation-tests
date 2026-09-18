// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <tuple>
#include <type_traits>

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SystemsExecutor.h"
#include "OGSimulation/SystemRoleAffinity.h"
#include "OGSimulation/SimulationTimeContext.h"

// ---------------------------------------------------------------------------
// Coverage for the SimulationSystem concept + SimulationSystemsExecutor +
// NullSystemsExecutor (task 5 of the ogsim-system-api initiative — see
// system_api_design.md §3.11 / §4.1 / §8.3).
//
// [ringout task 19] SystemRoleAffinity: every system now DECLARES whether it is part of the
// step (AllRoles) or a side effect of it (AuthorityOnly), the concept REQUIRES the declaration,
// and the executor gates all four hooks on the role its caller passes. Three things are pinned
// below: that seven malformed declarations are rejected by the concept and cannot instantiate an
// executor, that an AuthorityOnly system is inert off the authority through all four hooks and
// all three step shapes while an AllRoles system beside it still fires on every one of them,
// and that the exactly-once tripwire is not vacuous. See OGSimulation/SystemRoleAffinity.h.
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
		static constexpr SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AllRoles;

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
		static constexpr SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AllRoles;
		void preIntegrate(const SimulationTimeStep&, const MockView&, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, const MockView&, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, const MockView&, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, const MockView&, const MockStaticData&) {}
	};

	// --- BAD system 1: missing the RequiredSimulatables alias entirely -------
	// [ringout task 19] Each BAD system below carries a WELL-FORMED kRoleAffinity, so it still
	// fails the concept for exactly one reason - the one its name states. Without that line the
	// four negatives would all be failing on the affinity gate instead, and the assertions
	// underneath them would stop testing what they say they test.
	struct NoRequiredSimulatables
	{
		static constexpr SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AllRoles;
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&) {}
	};

	// --- BAD system 2: RequiredSimulatables is NOT a SimulatableList<> --------
	struct RequiredIsNotAList
	{
		static constexpr SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AllRoles;
		using RequiredSimulatables = MockSimulatable;  // raw type — should trip the S9 gate
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&) {}
	};

	// --- BAD system 3: a hook signature mismatch (postIntegrate returns int) --
	struct HookSignatureMismatch
	{
		static constexpr SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AllRoles;
		using RequiredSimulatables = MockList;
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		int  postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) { return 0; }  // not -> void
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&) {}
	};

	// --- BAD system 4: missing a hook entirely (no onCharacterUnregistered) ---
	struct MissingHook
	{
		static constexpr SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AllRoles;
		using RequiredSimulatables = MockList;
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
	};

	// --- THE SEVEN MALFORMED AFFINITY DECLARATIONS ---------------------------
	// Everything else about these is well-formed: the alias, all four hooks. The ONLY defect is
	// how kRoleAffinity is spelled, so each assertion below isolates one shape of wrong. The
	// first four are caught by the compound requirement (`{ T::kRoleAffinity } -> same_as<const
	// SystemRoleAffinity&>`), the fifth and sixth by it too, and the seventh by the nested
	// requirement that names both enumerators.
	#define OG_TEST_SYSTEM_HOOKS()                                                       \
		using RequiredSimulatables = MockList;                                            \
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}   \
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}  \
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}       \
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&) {}

	enum class NotTheAffinityEnum : uint8_t { AllRoles, AuthorityOnly };

	struct AffinityIsABool          { OG_TEST_SYSTEM_HOOKS() static constexpr bool kRoleAffinity = true; };
	struct AffinityIsAnInt          { OG_TEST_SYSTEM_HOOKS() static constexpr int  kRoleAffinity = 0; };
	struct AffinityIsAnotherEnum    { OG_TEST_SYSTEM_HOOKS() static constexpr NotTheAffinityEnum kRoleAffinity = NotTheAffinityEnum::AllRoles; };
	struct AffinityIsNonStatic      { OG_TEST_SYSTEM_HOOKS() SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AllRoles; };
	struct AffinityIsMutableStatic  { OG_TEST_SYSTEM_HOOKS() static inline SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AllRoles; };
	struct AffinityIsAFunction      { OG_TEST_SYSTEM_HOOKS() static constexpr SystemRoleAffinity kRoleAffinity() { return SystemRoleAffinity::AllRoles; } };
	// The one the compound requirement CANNOT catch: right type, static, const, constant - and a
	// value that is not an enumerator. Only the nested requirement rejects it, which is why that
	// requirement exists rather than trusting the type.
	struct AffinityIsACastToANonEnumerator { OG_TEST_SYSTEM_HOOKS() static constexpr SystemRoleAffinity kRoleAffinity = static_cast<SystemRoleAffinity>(7); };
	// A TYPO is the realistic form of "forgot to declare it", and it must fail the same way.
	struct TypoedAffinityName       { OG_TEST_SYSTEM_HOOKS() static constexpr SystemRoleAffinity kRoleAffinty = SystemRoleAffinity::AllRoles; };

	// POSITIVE, and recorded rather than assumed: `static const` with an in-class initialiser is
	// a constant expression and an lvalue of type `const SystemRoleAffinity`, so it satisfies the
	// requirement exactly as `static constexpr` does. The concept does not, and need not, insist
	// on the `constexpr` spelling.
	struct StaticConstAffinity      { OG_TEST_SYSTEM_HOOKS() static const SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AuthorityOnly; };

	// --- Recording systems for the role-gate cases ---------------------------
	// One of each affinity, same shape, so a single executor can be fired once and BOTH answers
	// read off it. Counting every hook separately is the point: a gate that only covered the two
	// step hooks would look identical on the scores and different on the roster.
	struct HookCounts
	{
		int pre = 0, post = 0, reg = 0, unreg = 0;
		bool operator==(const HookCounts&) const = default;
	};

	template <SystemRoleAffinity Affinity>
	struct CountingSystem
	{
		using RequiredSimulatables = MockList;
		static constexpr SystemRoleAffinity kRoleAffinity = Affinity;

		HookCounts counts;

		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&)  { ++counts.pre; }
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) { ++counts.post; }
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&)     { ++counts.reg; }
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&)   { ++counts.unreg; }
	};

	using AllRolesSystem      = CountingSystem<SystemRoleAffinity::AllRoles>;
	using AuthorityOnlySystem = CountingSystem<SystemRoleAffinity::AuthorityOnly>;

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
static_assert(SimulationSystem<StaticConstAffinity, MockStaticData>);  // `static const` + in-class init is accepted
static_assert(SimulationSystem<AllRolesSystem, MockStaticData>);
static_assert(SimulationSystem<AuthorityOnlySystem, MockStaticData>);

// --- THE SEVEN MALFORMED AFFINITY DECLARATIONS, rejected -------------------
static_assert(!SimulationSystem<AffinityIsABool, MockStaticData>);                   // 1. a bool
static_assert(!SimulationSystem<AffinityIsAnInt, MockStaticData>);                   // 2. an int
static_assert(!SimulationSystem<AffinityIsAnotherEnum, MockStaticData>);             // 3. a different enum type
static_assert(!SimulationSystem<AffinityIsNonStatic, MockStaticData>);               // 4. a non-static member
static_assert(!SimulationSystem<AffinityIsMutableStatic, MockStaticData>);           // 5. a non-const static
static_assert(!SimulationSystem<AffinityIsAFunction, MockStaticData>);               // 6. a function, not a value
static_assert(!SimulationSystem<AffinityIsACastToANonEnumerator, MockStaticData>);   // 7a. a value that is not an enumerator
static_assert(!SimulationSystem<TypoedAffinityName, MockStaticData>);                // 7b. the member is simply absent

// ...and none of the seven can instantiate an executor. ⚠ THIS IS THE HALF THAT MATTERS: a
// concept that says false but still lets the executor compile would gate nothing. The primary
// template is UNDEFINED, so a rejected system makes `sizeof` ill-formed here - and, at a real
// instantiation site with no SFINAE around it, an incomplete type (MSVC `C2079`).
static_assert(!ExecutorInstantiable<AffinityIsABool>);
static_assert(!ExecutorInstantiable<AffinityIsAnInt>);
static_assert(!ExecutorInstantiable<AffinityIsAnotherEnum>);
static_assert(!ExecutorInstantiable<AffinityIsNonStatic>);
static_assert(!ExecutorInstantiable<AffinityIsMutableStatic>);
static_assert(!ExecutorInstantiable<AffinityIsAFunction>);
static_assert(!ExecutorInstantiable<AffinityIsACastToANonEnumerator>);
static_assert(!ExecutorInstantiable<TypoedAffinityName>);
static_assert(ExecutorInstantiable<StaticConstAffinity>);   // the positive control for the eight above

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
		exec.firePreIntegrate(SimulationTimeStep(42, false), storage, sd, /*isAuthority=*/true);
		const MockSystem& s = exec.get<MockSystem>();
		REQUIRE(s.preCount == 1);
		REQUIRE(s.postCount == 0);
		REQUIRE(s.lastTick == 42);
	}

	SECTION("firePostIntegrate calls postIntegrate; the view reaches live storage")
	{
		exec.firePostIntegrate(SimulationTimeStep(7, false), storage, sd, /*isAuthority=*/true);
		const MockSystem& s = exec.get<MockSystem>();
		REQUIRE(s.postCount == 1);
		REQUIRE(s.lastTick == 7);
		REQUIRE(s.observedHpSum == 30);   // 10 + 20 through the projected view
	}

	SECTION("notifyCharacterRegistered / Unregistered fan out with the id, no step")
	{
		exec.notifyCharacterRegistered(5, storage, sd, /*isAuthority=*/true);
		exec.notifyCharacterUnregistered(9, storage, sd, /*isAuthority=*/true);
		const MockSystem& s = exec.get<MockSystem>();
		REQUIRE(s.regCount == 1);
		REQUIRE(s.unregCount == 1);
		REQUIRE(s.lastId == 9);   // last write wins
	}
}

// ---------------------------------------------------------------------------
// [ringout task 19] THE ROLE GATE, driven over BOTH affinities in ONE executor so the two
// answers are read off the same thirteen fires. An AuthorityOnly system must be inert off the
// authority through all four hooks AND all three step shapes; an AllRoles system beside it must
// still fire on every one of them, because a rollback replay that skips it diverges (D4).
//
// ⛔ THE ALLROLES ARM IS NOT DECORATION. Without it this case would pass against an executor
// that fired nothing at all - which is exactly what a `firesOnRole` returning false for
// everything would produce, and is a likelier defect than the one the AuthorityOnly arm catches.
// ---------------------------------------------------------------------------
TEST_CASE("SystemsExecutor.AuthorityOnlySystemIsInertOffTheAuthority", "[SystemsExecutor]")
{
	MockStore storage;
	storage.add<MockSimulatable>(1, MockSimulatable{ 10 });
	MockStaticData sd;

	SimulationSystemsExecutor<MockList, MockStaticData, AllRolesSystem, AuthorityOnlySystem> exec;

	auto fireStep = [&](unsigned int tick, bool isAuthority, bool isResim, StepKind kind)
	{
		const SimulationTimeStep step(tick, isResim, kind);
		exec.firePreIntegrate(step, storage, sd, isAuthority);
		exec.firePostIntegrate(step, storage, sd, isAuthority);
	};

	SECTION("off the authority: AllRoles fires 13/13/1/1, AuthorityOnly 0/0/0/0")
	{
		// One forward prediction tick.
		fireStep(100u, /*isAuthority=*/false, /*isResim=*/false, StepKind::Normal);
		// Eleven replayed ticks of one resim. ⚠ The role gate is taken BEFORE the exactly-once
		// tripwire, so these are silent rather than assertions - that ordering is the property.
		for (unsigned int t = 101u; t <= 111u; ++t)
			fireStep(t, /*isAuthority=*/false, /*isResim=*/true, StepKind::Normal);
		// One HardResync step: a client reaches the hooks on this shape too.
		fireStep(112u, /*isAuthority=*/false, /*isResim=*/false, StepKind::HardResync);
		// And both lifecycle notifications.
		exec.notifyCharacterRegistered(1, storage, sd, /*isAuthority=*/false);
		exec.notifyCharacterUnregistered(1, storage, sd, /*isAuthority=*/false);

		REQUIRE(exec.get<AllRolesSystem>().counts == HookCounts{ 13, 13, 1, 1 });
		REQUIRE(exec.get<AuthorityOnlySystem>().counts == HookCounts{ 0, 0, 0, 0 });
	}

	SECTION("on the authority: both fire, on every hook")
	{
		for (unsigned int t = 200u; t < 205u; ++t)
			fireStep(t, /*isAuthority=*/true, /*isResim=*/false, StepKind::Normal);
		exec.notifyCharacterRegistered(1, storage, sd, /*isAuthority=*/true);
		exec.notifyCharacterUnregistered(1, storage, sd, /*isAuthority=*/true);

		REQUIRE(exec.get<AllRolesSystem>().counts == HookCounts{ 5, 5, 1, 1 });
		REQUIRE(exec.get<AuthorityOnlySystem>().counts == HookCounts{ 5, 5, 1, 1 });
	}
}

// ---------------------------------------------------------------------------
// [ringout task 19] THE EXACTLY-ONCE TRIPWIRE, and what this case can honestly assert about it.
//
// `checkAuthorityOnlyStep` is an `OG_CHECK`, which in this target is UE's `checkf`: firing it
// does not throw and cannot be caught, so a case CANNOT assert "and here it fires" and stay
// green. What it can assert is the half that is reachable without firing anything - that the
// role gate is taken FIRST, so every client replay tick is silent - plus the positive control
// that an authority NON-resim step does reach the hook with the check present and passing.
//
// ⛔ THE OTHER HALF IS A HIDDEN CASE, NOT A MISSING ONE.
// `SystemsExecutor.TheAuthorityOnlyResimTripwireFires` below carries `[.]`, so it is excluded
// from every filter that does not name it - `[SystemsExecutor]` and `[@og]` both skip it and
// their counts do not move. Run it by name to see the tripwire fire:
//
//     OGSimulationTests.exe "SystemsExecutor.TheAuthorityOnlyResimTripwireFires"
//
// ⚠ A check that cannot be made to fire proves nothing, and this initiative has shipped two
// vacuous controls already. That hidden case is this one's vacuity arm, and ring-out task 19's
// notes record what it printed and with what exit code.
// ---------------------------------------------------------------------------
TEST_CASE("SystemsExecutor.TheRoleGateIsTakenBeforeTheResimTripwire", "[SystemsExecutor]")
{
	MockStore storage;
	storage.add<MockSimulatable>(1, MockSimulatable{ 10 });
	MockStaticData sd;

	SimulationSystemsExecutor<MockList, MockStaticData, AuthorityOnlySystem> exec;

	// Twelve resimulated steps off the authority. If the tripwire were consulted before the
	// gate - or if the gate were dropped - this line would abort the process rather than fail.
	for (unsigned int t = 100u; t < 112u; ++t)
		exec.firePostIntegrate(SimulationTimeStep(t, /*isResimulating=*/true), storage, sd,
							  /*isAuthority=*/false);
	REQUIRE(exec.get<AuthorityOnlySystem>().counts == HookCounts{ 0, 0, 0, 0 });

	// The positive control: an authority step is NOT a resim step (ServerTickClock builds
	// isResimulating=false), so the hook is entered and the check passes.
	exec.firePostIntegrate(SimulationTimeStep(200u, /*isResimulating=*/false), storage, sd,
						   /*isAuthority=*/true);
	REQUIRE(exec.get<AuthorityOnlySystem>().counts == HookCounts{ 0, 1, 0, 0 });
}

// ⛔ THE VACUITY ARM. Hidden (`[.]`), so no default run and no tag count includes it. It
// exists to be run by name, and it is EXPECTED to fire the check - see the case above.
TEST_CASE("SystemsExecutor.TheAuthorityOnlyResimTripwireFires", "[.][SystemsExecutorTripwire]")
{
	MockStore storage;
	MockStaticData sd;
	SimulationSystemsExecutor<MockList, MockStaticData, AuthorityOnlySystem> exec;

	// isAuthority=true on a RESIMULATED step: the combination the authority can never produce.
	exec.firePostIntegrate(SimulationTimeStep(1u, /*isResimulating=*/true), storage, sd,
						   /*isAuthority=*/true);
	FAIL("the OG_CHECK tripwire did NOT fire - checkAuthorityOnlyStep is vacuous in this target");
}

// ⛔ [ringout task 19] MOVED OUT OF THE TEST_CASE, AND THAT IS AN API CONSEQUENCE WORTH
// RECORDING, not a tidy-up: `kRoleAffinity` is a static data member, and C++ forbids those in a
// function-LOCAL class (MSVC `C2246`). Requiring the declaration therefore means a system can no
// longer be declared inside a function body. Measured here — this case's `SeededSystem` was
// local until the concept gained the requirement.
namespace
{
	// A system whose only ctor takes an int — reached via std::piecewise_construct.
	struct SeededSystem
	{
		using RequiredSimulatables = MockList;
		static constexpr SystemRoleAffinity kRoleAffinity = SystemRoleAffinity::AllRoles;
		int seed;
		explicit SeededSystem(int s) : seed(s) {}
		void preIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void postIntegrate(const SimulationTimeStep&, MockView, const MockStaticData&) {}
		void onCharacterRegistered(unsigned int, MockView, const MockStaticData&) {}
		void onCharacterUnregistered(unsigned int, MockView, const MockStaticData&) {}
	};
}

static_assert(SimulationSystem<SeededSystem, MockStaticData>);

TEST_CASE("SystemsExecutor.PiecewiseConstruct", "[SystemsExecutor]")
{

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
	nullExec.firePreIntegrate(SimulationTimeStep(1, false), storage, sd, /*isAuthority=*/true);
	nullExec.firePostIntegrate(SimulationTimeStep(2, false), storage, sd, /*isAuthority=*/false);
	nullExec.notifyCharacterRegistered(3, storage, sd, /*isAuthority=*/true);
	nullExec.notifyCharacterUnregistered(4, storage, sd, /*isAuthority=*/false);

	// No system exists, so nothing mutated the storage.
	REQUIRE(storage.get<MockSimulatable>(1).hp == 100);
	SUCCEED("NullSystemsExecutor fire methods are no-ops");
}

#endif // WITH_LOW_LEVEL_TESTS
