// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionCache.h"
#include "OGSimulation/NetSyncTelemetry.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/SimulationComparison.h"

#include <cstring>
#include <string>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-field-defects / task 6 — NAME THE DISAGREEING FIELD, AND PROVE
// THE NAMING IS FREE AT THE SHIPPED VERBOSITY.
//
// `isSimilarTo` is a boolean fold that discards the distance. The 2026-09-21
// knockback analysis reached a wall no line in any log could get past: which
// field kept disagreeing for 36 and then 70 consecutive corrections. This file
// covers the instrument that answers it, and it covers TWO claims that pull in
// opposite directions, which is why they are in one file:
//
//   1. WHEN IT IS ON, it names the right field with the right magnitude.
//   2. WHEN IT IS OFF — the SHIPPED `LogOGDivergenceProbe=Warning` — NOTHING IS
//      WALKED AT ALL. Not "the line is dropped later"; not "the cost is small".
//      Zero field reads.
//
// THE CONTROL RUNS THROUGH EVERY CASE. `isSimilarTo`'s verdict is asserted in
// every arm below, gate open and gate shut alike, against the same pairs. The
// feature is additive or it is a regression, and the verdict is what says which.
//
// HOW CLAIM 2 IS MEASURED RATHER THAN ARGUED. `correctionFieldDiff::walkCount()`
// counts entries to `describeFirstDivergingField` — the walk itself, not log
// lines, not formatted strings. A probe that counted lines would pass while the
// walk ran and its result was discarded, which is precisely the regression this
// has to catch. The third arm (gate OPEN, count MUST MOVE) is what stops the
// zeros in arms one and two being a dead instrument reporting nothing.
//////////////////////////////////////////////////////////////////////////////

// A NAMED namespace, not an anonymous one, and the reason is MEASURED.
//
// ⚠ MSVC 14.38.33130 spells a type declared in an anonymous namespace
// `` `anonymous-namespace'::MockMovementState `` inside `__FUNCSIG__`, so the walk
// faithfully reports that as the path root — correct, and unreadable. It is not a
// defect in the walk and there is nothing to fix in it: every type the walk names
// in production (`dAttackRadialSimulation::State`, `brawlerMovementSimulation::State`)
// is in a named namespace. Putting the mocks in one keeps the ASSERTIONS BELOW
// legible AND keeps them an honest sample of the production shape. The first run of
// this file asserted the unqualified spelling and failed with exactly that expansion.
namespace fieldDivergenceMock
{
	// A state slice shaped like a real one: an enum, a scalar, and a nested
	// Serializable aggregate that is a SHIPPED og-simulation type rather than a
	// mock — `LinearBodyState`, whose `position` is the field the brawler's
	// movement slice carries and the field the AC plants into.
	enum class MockPhase : std::uint8_t { Idle = 0, Moving = 1, Stunned = 7 };

	struct MockMovementState
	{
		MockPhase phase = MockPhase::Idle;
		float     timer = 0.f;
		LinearBodyState bodyState;
	};

	struct MockScoreState
	{
		std::uint32_t score = 0u;
	};

	struct MockInput
	{
		std::int32_t value = 0;
	};
} // namespace fieldDivergenceMock

template <>
struct SerializableFields<fieldDivergenceMock::MockMovementState>
{
	static constexpr auto get()
	{
		using S = fieldDivergenceMock::MockMovementState;
		return std::make_tuple(
			SIM_MEMBER(S, phase),
			SIM_MEMBER(S, timer),
			SIM_MEMBER(S, bodyState));
	}
};

template <>
struct SerializableFields<fieldDivergenceMock::MockScoreState>
{
	static constexpr auto get()
	{
		return std::make_tuple(SIM_MEMBER(fieldDivergenceMock::MockScoreState, score));
	}
};

namespace
{
	using fieldDivergenceMock::MockPhase;
	using MockState = SimulationStateComposite<
		fieldDivergenceMock::MockMovementState, fieldDivergenceMock::MockScoreState>;
	using MockCache = StateCorrectionCache<MockState, fieldDivergenceMock::MockInput>;

	// RAII for the two process-global knobs. ⛔ Catch2 runs these cases in one
	// process alongside every other suite; leaving a predicate installed would make
	// a later suite pay for walks it never asked for, and leaving the counter dirty
	// would make the next case's delta meaningless.
	class GateScope
	{
	public:
		explicit GateScope(std::function<bool()> predicate)
		{
			correctionFieldDiff::setEnabledPredicate(std::move(predicate));
			correctionFieldDiff::resetWalkCount();
		}
		~GateScope()
		{
			correctionFieldDiff::clearEnabledPredicate();
			correctionFieldDiff::resetWalkCount();
		}
		GateScope(const GateScope&) = delete;
		GateScope& operator=(const GateScope&) = delete;
	};

	void predictTick(MockCache& cache, std::uint32_t tick, const MockState& state)
	{
		cache.pushPredictionTick(tick);
		cache.pushPredictionState(state);
	}

	MockState stateWith(MockPhase phase, float timer, float positionX, std::uint32_t score)
	{
		MockState s;
		s.edit<fieldDivergenceMock::MockMovementState>().phase = phase;
		s.edit<fieldDivergenceMock::MockMovementState>().timer = timer;
		s.edit<fieldDivergenceMock::MockMovementState>().bodyState.position.x = positionX;
		s.edit<fieldDivergenceMock::MockScoreState>().score = score;
		return s;
	}

	std::string tailOf(const FieldDivergence& divergence)
	{
		char buffer[160];
		formatFieldDivergence(divergence, buffer, sizeof(buffer));
		return std::string(buffer);
	}
} // namespace

// ---------------------------------------------------------------------------
// THE WIRE, AT COMPILE TIME. Task 6 gave every field descriptor a `name`, and the
// wire layout is a function of the descriptors — so "the wire is untouched" is a
// claim this file has to hold, not a sentence in a note.
//
// ⛔ THE TYPE-IDENTITY ASSERTION IS THE LOAD-BEARING ONE AND IT IS WHY `name` IS A
// MEMBER RATHER THAN A SECOND TEMPLATE PARAMETER. Nine APPEND-ONLY wire fences
// across the games pin `decltype(SerializableFields<S>::get())` against an exact
// `std::tuple<MemberFieldDesc<&S::f>, ...>`. An NTTP would have changed every one
// of those types and fired every one of those fences — in four files this task is
// forbidden to touch. Reintroduce a template parameter and this line goes first.
// ---------------------------------------------------------------------------

static_assert(
	std::is_same_v<decltype(SIM_MEMBER(PhysicsBodyState, position)),
	               MemberFieldDesc<&PhysicsBodyState::position>>,
	"SIM_MEMBER must not change a field descriptor's TYPE. The APPEND-ONLY wire fences in "
	"BrawlerMovementSimulation.h, BrawlerRingoutSimulation.h and their siblings compare "
	"decltype(SerializableFields<S>::get()) against a tuple of bare MemberFieldDesc<&S::f>, "
	"so a descriptor carrying its name in a template parameter fires all of them. Task 6.");

// ...and the name reaches the descriptor, which the assertion above cannot say.
// ⛔ A PAIR, DELIBERATELY: identity alone is satisfied by dropping the name entirely.
static_assert(SIM_MEMBER(PhysicsBodyState, position).name != nullptr,
	"SIM_MEMBER must carry the member's spelling; the divergence walk has no other source "
	"for it (a pointer-to-member NTTP prints as `pointer-to-member(0x0)` under MSVC). Task 6.");

static_assert(syncSize<PhysicsBodyState>() == 52u,
	"PhysicsBodyState wire size moved. Task 6 added a `name` member to every field "
	"descriptor; FieldDesc::Size is computed from Value alone and must not have noticed.");
static_assert(syncSize<LinearBodyState>() == 24u,
	"LinearBodyState wire size moved. Same reason as PhysicsBodyState above. Task 6.");

// ⛔ THE REPORT MUST NEVER BECOME WIRE. Giving `FieldDivergence` a SerializableFields
// specialization would silently put a 96-byte path string into every correction
// packet and into the determinism checksum.
// ⛔ PAIRED WITH A `sizeof`, because an absence check alone passes for a type that
// was deleted, renamed or emptied.
static_assert(!Serializable<FieldDivergence>,
	"FieldDivergence is a DIAGNOSTIC value and must never be serializable. Task 6.");
static_assert(!Serializable<CorrectionInsertVerdict>,
	"CorrectionInsertVerdict is a DIAGNOSTIC value and must never be serializable. Task 6.");
static_assert(sizeof(FieldDivergence) >= kFieldPathCapacity,
	"FieldDivergence lost its path buffer. Task 6.");

// ---------------------------------------------------------------------------
// ARM 1 — THE DEFAULT SHIPPED STATE OF THIS SUBMODULE: no predicate installed.
//
// This is the arm that covers every consumer of og-simulation that never opts
// in, including a game that has never heard of this feature. It is not the same
// claim as arm 2 and must not be folded into it: arm 2 proves the gate answers
// correctly, this proves there is a CLOSED gate to answer with in the first
// place.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: an UNCONFIGURED host walks no field, "
          "and the boolean verdict is unchanged",
          "[Network][DivergenceProbe]")
{
	correctionFieldDiff::clearEnabledPredicate();
	correctionFieldDiff::resetWalkCount();
	REQUIRE_FALSE(correctionFieldDiff::enabled());

	MockCache cache(nullptr);
	const MockState predicted = stateWith(MockPhase::Moving, 1.5f, 4.f, 3u);
	const MockState authority = stateWith(MockPhase::Stunned, 1.5f, 4.f, 3u);

	for (std::uint32_t tick = 100u; tick < 105u; ++tick)
		predictTick(cache, tick, predicted);

	for (std::uint32_t tick = 100u; tick < 105u; ++tick)
	{
		CorrectionInsertVerdict verdict;
		MockState arriving = authority;
		cache.tryInsertingCorrectState(std::move(arriving), tick, kNoInputCaptureTick, &verdict);

		// THE CONTROL. Five landed, five disagreeing — the verdict is exactly what
		// it was before this feature existed.
		REQUIRE(verdict.landed);
		REQUIRE_FALSE(verdict.predictionWasCorrect);

		// ...and not one field was looked at.
		REQUIRE_FALSE(verdict.fieldDivergence.evaluated);
		REQUIRE_FALSE(verdict.fieldDivergence.named());
		REQUIRE(tailOf(verdict.fieldDivergence).empty());
	}

	REQUIRE(correctionFieldDiff::walkCount() == 0u);
}

// ---------------------------------------------------------------------------
// ARM 2 — THE SHIPPED VERBOSITY. A predicate IS installed, standing in for
// `UE_LOG_ACTIVE(LogOGDivergenceProbe, Verbose)` on a host whose ini says
// Warning. Five disagreeing corrections, zero walks.
//
// ⛔ The predicate COUNTS ITS OWN INVOCATIONS, which is the assertion that stops
// this case passing for the wrong reason: a "gate" that was never consulted
// would also produce walkCount() == 0, and would then also produce zero under a
// future edit that removed the gate and the whole feature with it.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: at Warning the gate is CONSULTED and NO diff is computed, "
          "and the boolean verdict is unchanged",
          "[Network][DivergenceProbe]")
{
	unsigned int gateQueries = 0u;
	GateScope gate([&gateQueries]() { ++gateQueries; return false; });

	MockCache cache(nullptr);
	const MockState predicted = stateWith(MockPhase::Moving, 1.5f, 4.f, 3u);
	const MockState authority = stateWith(MockPhase::Stunned, 1.5f, 4.f, 3u);

	for (std::uint32_t tick = 200u; tick < 205u; ++tick)
		predictTick(cache, tick, predicted);

	for (std::uint32_t tick = 200u; tick < 205u; ++tick)
	{
		CorrectionInsertVerdict verdict;
		MockState arriving = authority;
		cache.tryInsertingCorrectState(std::move(arriving), tick, kNoInputCaptureTick, &verdict);

		REQUIRE(verdict.landed);
		REQUIRE_FALSE(verdict.predictionWasCorrect);
		REQUIRE_FALSE(verdict.fieldDivergence.evaluated);
	}

	// The gate was asked once per DISAGREEING landed correction...
	REQUIRE(gateQueries == 5u);
	// ...and answered no every time, so nothing walked.
	REQUIRE(correctionFieldDiff::walkCount() == 0u);
}

// ---------------------------------------------------------------------------
// ARM 3 — THE ANTI-VACUITY ARM. Same pairs, same cache, gate OPEN. The counter
// MUST move, or the two zeros above are a dead instrument.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: at Verbose the walk RUNS and names the planted field, "
          "and the boolean verdict is still unchanged",
          "[Network][DivergenceProbe]")
{
	GateScope gate([]() { return true; });

	MockCache cache(nullptr);
	const MockState predicted = stateWith(MockPhase::Moving, 1.5f, 4.f, 3u);
	const MockState authority = stateWith(MockPhase::Stunned, 1.5f, 4.f, 3u);

	for (std::uint32_t tick = 300u; tick < 305u; ++tick)
		predictTick(cache, tick, predicted);

	for (std::uint32_t tick = 300u; tick < 305u; ++tick)
	{
		CorrectionInsertVerdict verdict;
		MockState arriving = authority;
		cache.tryInsertingCorrectState(std::move(arriving), tick, kNoInputCaptureTick, &verdict);

		// THE CONTROL, ARM 3. Identical to arms 1 and 2 — the feature changed the
		// REPORT, never the verdict.
		REQUIRE(verdict.landed);
		REQUIRE_FALSE(verdict.predictionWasCorrect);

		REQUIRE(verdict.fieldDivergence.evaluated);
		REQUIRE(verdict.fieldDivergence.kind == FieldDivergenceKind::Discrete);
		REQUIRE(std::string(verdict.fieldDivergence.path) == "fieldDivergenceMock::MockMovementState.phase");
		REQUIRE(verdict.fieldDivergence.oldValue == 1);   // Moving,  the PREDICTION
		REQUIRE(verdict.fieldDivergence.newValue == 7);   // Stunned, the AUTHORITY
	}

	REQUIRE(correctionFieldDiff::walkCount() == 5u);
}

// ---------------------------------------------------------------------------
// THE VACUITY CONTROL — AN AGREEING CORRECTION NAMES NOTHING, and it does not
// even pay for the question. The gate is wide open here, so the zero below is
// the CALL SITE's second condition (`!predictionWasCorrect`) and nothing else.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: an AGREEING correction names nothing and walks nothing "
          "even with the gate open",
          "[Network][DivergenceProbe]")
{
	GateScope gate([]() { return true; });

	MockCache cache(nullptr);
	const MockState identical = stateWith(MockPhase::Moving, 1.5f, 4.f, 3u);

	predictTick(cache, 400u, identical);

	CorrectionInsertVerdict verdict;
	MockState arriving = identical;
	cache.tryInsertingCorrectState(std::move(arriving), 400u, kNoInputCaptureTick, &verdict);

	REQUIRE(verdict.landed);
	REQUIRE(verdict.predictionWasCorrect);          // the control
	REQUIRE_FALSE(verdict.fieldDivergence.evaluated);
	REQUIRE_FALSE(verdict.fieldDivergence.named());
	REQUIRE(tailOf(verdict.fieldDivergence).empty());
	REQUIRE(correctionFieldDiff::walkCount() == 0u);
}

// ---------------------------------------------------------------------------
// A DISCARDED CORRECTION IS NOT A VERDICT AND NOT A WALK. No slot, no
// comparison, nothing to name — and the existing `landed == false` report is
// untouched.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: a DISCARDED correction walks nothing",
          "[Network][DivergenceProbe]")
{
	GateScope gate([]() { return true; });

	MockCache cache(nullptr);
	predictTick(cache, 500u, stateWith(MockPhase::Moving, 1.5f, 4.f, 3u));

	CorrectionInsertVerdict verdict;
	MockState arriving = stateWith(MockPhase::Stunned, 9.f, 9.f, 9u);
	cache.tryInsertingCorrectState(std::move(arriving), 9999u, kNoInputCaptureTick, &verdict);

	REQUIRE_FALSE(verdict.landed);                  // the control
	REQUIRE_FALSE(verdict.fieldDivergence.evaluated);
	REQUIRE(correctionFieldDiff::walkCount() == 0u);
}

// ---------------------------------------------------------------------------
// THE NESTED NUMERIC CASE — `LinearBodyState.position`, the AC's third planted
// field, reached through a nested Serializable aggregate. The path must descend
// INTO the aggregate rather than stopping at `bodyState`, and the magnitude is
// the MAX COMPONENT delta, not a norm.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: a nested LinearBodyState.position is named with its magnitude",
          "[Network][DivergenceProbe]")
{
	GateScope gate([]() { return true; });

	MockState predicted = stateWith(MockPhase::Moving, 1.5f, 0.f, 3u);
	MockState authority = predicted;
	authority.edit<fieldDivergenceMock::MockMovementState>().bodyState.position.z = -2.5f;

	// THE CONTROL FIRST: the fold says these differ. Everything below describes
	// THIS verdict; if the fold ever said they agree, the description would be
	// describing nothing.
	REQUIRE_FALSE(predicted.isSimilarTo(authority));

	FieldDivergence divergence;
	describeFirstDivergingField(predicted, authority, divergence);

	REQUIRE(divergence.evaluated);
	REQUIRE(divergence.kind == FieldDivergenceKind::Numeric);
	REQUIRE(std::string(divergence.path) == "fieldDivergenceMock::MockMovementState.bodyState.position");
	REQUIRE(divergence.delta == Catch::Approx(2.5f));
	REQUIRE(tailOf(divergence) == " field=fieldDivergenceMock::MockMovementState.bodyState.position delta=2.5");
}

// ---------------------------------------------------------------------------
// FIRST, IN DECLARATION ORDER — the same order and the same stopping rule
// `fieldwiseIsSimilarTo` uses. With TWO fields planted the earlier one wins, and
// the SECOND composite element is only reached when the first agrees.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: the FIRST differing field wins, in declaration order",
          "[Network][DivergenceProbe]")
{
	GateScope gate([]() { return true; });

	MockState predicted = stateWith(MockPhase::Moving, 1.5f, 0.f, 3u);

	SECTION("two planted in the same element — the earlier descriptor wins")
	{
		MockState authority = predicted;
		authority.edit<fieldDivergenceMock::MockMovementState>().timer = 9.5f;                  // descriptor 1
		authority.edit<fieldDivergenceMock::MockMovementState>().bodyState.position.x = 1.f;    // descriptor 2

		FieldDivergence divergence;
		describeFirstDivergingField(predicted, authority, divergence);
		REQUIRE(std::string(divergence.path) == "fieldDivergenceMock::MockMovementState.timer");
		REQUIRE(divergence.delta == Catch::Approx(8.f));
	}

	SECTION("the second composite element is reached when the first agrees")
	{
		MockState authority = predicted;
		authority.edit<fieldDivergenceMock::MockScoreState>().score = 11u;

		FieldDivergence divergence;
		describeFirstDivergingField(predicted, authority, divergence);
		REQUIRE(std::string(divergence.path) == "fieldDivergenceMock::MockScoreState.score");
		REQUIRE(divergence.kind == FieldDivergenceKind::Discrete);
		REQUIRE(divergence.oldValue == 3);
		REQUIRE(divergence.newValue == 11);
	}
}

// ---------------------------------------------------------------------------
// THE WALK AND THE FOLD AGREE, WHATEVER THE FOLD SAYS.
//
// This is the case for the subtlest claim in the change, and the one no other
// case here can reach: that the walk's leaf question resolves at
// `fieldwiseIsSimilarTo`'s DEFINITION POINT rather than at its own. Every other
// case plants a difference far larger than `kDefaultSimilarityEpsilon`, where
// every plausible predicate agrees; only a SUB-EPSILON plant can tell them apart.
//
// ⛔ IT ASSERTS A RELATION, NEVER A DIRECTION, AND THAT IS DELIBERATE. Under
// /permissive- the glm overloads of `isSimilarToField` are not found from
// `fieldwiseIsSimilarTo` (they are in the global namespace, not in `glm`), so a
// vector field compares EXACTLY while a bare float compares with the 1e-4
// epsilon. That asymmetry is a known, recorded property of this tree that is
// deliberately NOT being fixed here and deliberately NOT pinned: a test asserting
// "a 1e-5 vector difference IS a divergence" would turn the eventual fix into a
// false regression. What must hold either way is that ONE verdict comes out of
// this codebase — `named() == !isSimilarTo()` — and that survives the fix.
//
// ⛔ SO A RE-DERIVED LEAF PREDICATE IN THE WALK BREAKS THIS CASE IN BOTH
// DIRECTIONS: an exact `==` names the sub-epsilon float the fold forgave, and a
// hand-rolled epsilon forgives the sub-epsilon vector the fold flagged.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: the walk NAMES a field exactly when the boolean fold "
          "disagrees, including sub-epsilon plants",
          "[Network][DivergenceProbe]")
{
	GateScope gate([]() { return true; });

	// Comfortably inside kDefaultSimilarityEpsilon (1e-4), on both axes tried.
	const float subEpsilon = 5e-5f;

	SECTION("a sub-epsilon SCALAR plant")
	{
		const MockState predicted = stateWith(MockPhase::Moving, 1.f, 0.f, 3u);
		MockState authority = predicted;
		authority.edit<fieldDivergenceMock::MockMovementState>().timer = 1.f + subEpsilon;

		FieldDivergence divergence;
		describeFirstDivergingField(predicted, authority, divergence);
		REQUIRE(divergence.named() == !predicted.isSimilarTo(authority));
	}

	SECTION("a sub-epsilon VECTOR plant, inside a nested aggregate")
	{
		const MockState predicted = stateWith(MockPhase::Moving, 1.f, 0.f, 3u);
		MockState authority = predicted;
		authority.edit<fieldDivergenceMock::MockMovementState>().bodyState.position.y = subEpsilon;

		FieldDivergence divergence;
		describeFirstDivergingField(predicted, authority, divergence);
		REQUIRE(divergence.named() == !predicted.isSimilarTo(authority));
	}

	// ⛔ THE ARM THAT ACTUALLY BITES, and the reason the two sections above are not
	// enough. MEASURED: with the leaf predicate poisoned to a bare `==`, both
	// sections above still PASSED. They plant ONE difference, so the COMPOSITE-LEVEL
	// fold (`compositeDetail::compareElement`) forgives the whole element and the
	// walk never descends into it — the poisoned leaf is unreachable. To reach the
	// leaf the element must ALREADY differ, so this section plants TWO: a
	// sub-epsilon one in an EARLIER descriptor and a real one in a LATER one. The
	// fold forgives the first and flags the second, and the walk must name the
	// SECOND. A re-derived exact leaf names the first instead.
	SECTION("a sub-epsilon plant in an EARLIER field is not the first DIFFERING field")
	{
		const MockState predicted = stateWith(MockPhase::Moving, 1.f, 0.f, 3u);
		MockState authority = predicted;
		authority.edit<fieldDivergenceMock::MockMovementState>().timer = 1.f + subEpsilon;
		authority.edit<fieldDivergenceMock::MockMovementState>().bodyState.position.x = 5.f;

		REQUIRE_FALSE(predicted.isSimilarTo(authority));

		FieldDivergence divergence;
		describeFirstDivergingField(predicted, authority, divergence);
		REQUIRE(std::string(divergence.path)
			== "fieldDivergenceMock::MockMovementState.bodyState.position");
		REQUIRE(divergence.delta == Catch::Approx(5.f));
	}

	SECTION("an IDENTICAL pair — the degenerate end of the same relation")
	{
		const MockState predicted = stateWith(MockPhase::Moving, 1.f, 0.f, 3u);
		const MockState authority = predicted;

		FieldDivergence divergence;
		describeFirstDivergingField(predicted, authority, divergence);
		REQUIRE(divergence.named() == !predicted.isSimilarTo(authority));
		REQUIRE_FALSE(divergence.named());
	}
}

// ---------------------------------------------------------------------------
// THE LINE ITSELF. The tail has to reach `[DivergenceProbe.Correction]`, and it
// has to be ABSENT when the walk did not run — a `field=` token on a line the
// gate suppressed the walk for would assert an observation nothing made.
//
// ⛔ THE EXISTING TOKENS ARE ASSERTED CHARACTER FOR CHARACTER. `[Verbose]`, the
// tag, `id=`, `tick=`, `class=` and `correct=` are what operators grep for and
// what `Config/DefaultEngine.ini`'s [T24] block documents. This case is the
// control for the LINE the way the verdict is the control for the DECISION.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: the tail rides the existing "
          "[DivergenceProbe.Correction] line and is absent when the walk did not run",
          "[Network][DivergenceProbe]")
{
	std::vector<std::string> lines;
	NetSyncTelemetry telemetry;
	telemetry.setLogger([&lines](const char* msg) { lines.emplace_back(msg); });

	const auto correctionLine = [&lines]() -> std::string {
		for (const std::string& line : lines)
			if (line.find("[DivergenceProbe.Correction]") != std::string::npos)
				return line;
		return std::string();
	};

	SECTION("the walk did not run — the line is byte-for-byte its pre-task-6 self")
	{
		CorrectionArrivalDecision decision;
		decision.characterClass       = PredictedCharacterClass::LocallyPredicted;
		decision.landingSite          = CorrectionLandingSite::AtFrontier;
		decision.landed               = true;
		decision.tick                 = 1523u;
		decision.predictionWasCorrect = false;
		// decision.fieldDivergence left unevaluated, as at Warning.

		telemetry.emitCorrectionArrival(42357u, decision);

		REQUIRE(correctionLine() ==
			"[Verbose][DivergenceProbe.Correction] id=42357 tick=1523 class=LocallyPredicted correct=0");
	}

	SECTION("the walk ran — the tail is appended, nothing before it moves")
	{
		CorrectionArrivalDecision decision;
		decision.characterClass       = PredictedCharacterClass::LocallyPredicted;
		decision.landingSite          = CorrectionLandingSite::AtFrontier;
		decision.landed               = true;
		decision.tick                 = 1523u;
		decision.predictionWasCorrect = false;
		decision.fieldDivergence.evaluated = true;
		decision.fieldDivergence.kind      = FieldDivergenceKind::Numeric;
		decision.fieldDivergence.delta     = 12.25f;
		std::strncpy(decision.fieldDivergence.path,
			"dAttackRadialSimulation::State.bodyState.angularVelocity",
			kFieldPathCapacity - 1u);

		telemetry.emitCorrectionArrival(42357u, decision);

		REQUIRE(correctionLine() ==
			"[Verbose][DivergenceProbe.Correction] id=42357 tick=1523 class=LocallyPredicted correct=0"
			" field=dAttackRadialSimulation::State.bodyState.angularVelocity delta=12.25");
	}

	SECTION("a DISCARDED correction still carries no tail — it never had a comparison")
	{
		CorrectionArrivalDecision decision;
		decision.landed = false;
		decision.tick   = 1523u;

		telemetry.emitCorrectionArrival(42357u, decision);

		// The verdict line is not emitted at all on a discard — the pre-existing
		// gate, unchanged.
		REQUIRE(correctionLine().empty());
	}
}

// ---------------------------------------------------------------------------
// THE PATH BUFFER TRUNCATES, IT DOES NOT OVERFLOW, and the line stays inside
// SIMLOG's `char[256]`. A silent truncation of `delta=` would turn the
// instrument into a field name with no magnitude — the exact half-answer the
// task exists to replace.
// ---------------------------------------------------------------------------
TEST_CASE("Correction field divergence: a maximal line fits inside the SIMLOG buffer",
          "[Network][DivergenceProbe]")
{
	std::vector<std::string> lines;
	NetSyncTelemetry telemetry;
	telemetry.setLogger([&lines](const char* msg) { lines.emplace_back(msg); });

	CorrectionArrivalDecision decision;
	decision.characterClass       = PredictedCharacterClass::LocallyPredicted;
	decision.landingSite          = CorrectionLandingSite::AtFrontier;
	decision.landed               = true;
	decision.tick                 = 4294967295u;
	decision.predictionWasCorrect = false;
	decision.fieldDivergence.evaluated = true;
	decision.fieldDivergence.kind      = FieldDivergenceKind::Discrete;
	decision.fieldDivergence.oldValue  = -9223372036854775807LL;
	decision.fieldDivergence.newValue  = 9223372036854775807LL;
	std::memset(decision.fieldDivergence.path, 'x', kFieldPathCapacity - 1u);
	decision.fieldDivergence.path[kFieldPathCapacity - 1u] = '\0';

	telemetry.emitCorrectionArrival(4294967295u, decision);

	std::string line;
	for (const std::string& candidate : lines)
		if (candidate.find("[DivergenceProbe.Correction]") != std::string::npos)
			line = candidate;

	REQUIRE_FALSE(line.empty());
	// 255 characters plus the NUL is all SIMLOG's buffer can carry; the widest
	// line this feature can produce must land strictly inside it, or the tail it
	// exists to add is the part that gets cut.
	REQUIRE(line.size() < 255u);
	REQUIRE(line.find("new=9223372036854775807") != std::string::npos);
}

#endif // WITH_LOW_LEVEL_TESTS
