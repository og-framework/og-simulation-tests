// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/ResimGatePolicy.h"
#include "OGSimulation/SimulationManager.h"
// [item 71] ResimSweepDiagnostics — MockReconciliation::postResimulationAll's
// return type, needed to drive the real onPostGameSimulation `[Resim.Finish]`
// path below.
#include "OGSimulation/SimulationReconciliation.h"

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / item 45: THE RESIM-GATE POLICY KERNEL AND ITS
// WIRING.
// (design_task43_resim_gate_fix.md §3 candidate D, §4; the gate's own semantics
//  are in the sibling `ResimGateSemanticsTest.cpp`.)
//
// TWO HALVES, AND NEITHER IS SUFFICIENT ALONE — the `correctionRotation` split,
// for the same reason:
//   * THE KERNEL. Two pure predicates in `resimGate::`, swept exhaustively here
//     because they are the whole of the policy and can be asked every question
//     without a cache, a storage tuple or a clock.
//   * THE WIRING. That the manager actually consults them, with the values from
//     ITS TimeConfig — proven with the duck-typed manager rig the sibling knobs'
//     suites use. A green kernel with an unwired caller ships the legacy gate on
//     every setting, which is indistinguishable from success on a default build.
//
// WHAT THIS SUITE DELIBERATELY DOES NOT COVER: the ini intake and the
// `[ResimGate]` proof line, which are UE-side composition-root code
// (SimulationManagerUImpl) and invisible to a pure-C++ target; the per-character
// depth-skip sweep inside `checkDivergenceAll`, which needs a real storage and a
// real simulatable and therefore lives in og-brawler-tests'
// SimulationReconciliationTest; and the anchor semantics themselves.
//
// Tagged `[CorrectionCache][ResimGate]` — the same pair the semantics suite uses,
// so one isolation command still recovers item 45's whole test contribution.
//////////////////////////////////////////////////////////////////////////////

namespace
{
	// The peers SimulationManager holds by reference. Duck-typed and instantiated
	// only where used — mirrors the rigs in CorrectionRotationTest.cpp and
	// RelayRedundancyDepthTest.cpp deliberately, so the four knobs' setter cases
	// read the same.
	//
	// [item 45] `MockReconciliation` now RECORDS what the manager passed it. That is
	// the only way to see the depth policy's derivation from this side of the
	// boundary: the manager's job is to turn (policy, rollbackWindowTicks) into the
	// one number `checkDivergenceAll` obeys, and the number is otherwise invisible
	// until a real cache is involved.
	// [item 71] The no-op members below (both structs) are only reached by the
	// prediction/resim drive the new wiring case uses to get a real
	// `ClientPredictionClock` (`shouldRunPrediction=true`) into a state where
	// `onPostGameSimulation` takes the `[Resim.Finish]` branch — `onCheckIsSimilar`
	// (every other case in this file) never calls any of them. The `inputs` type is
	// an unused placeholder (`int`); nothing here reads its content, matching the
	// duck-typed pass-through `SimulationManager` itself does.
	struct MockIntegrationExec
	{
		void captureBodyStatesAll() {}
		void integrateAll(const SimulationTimeStep&, int) {}
		void firstResimStepAll(int32_t) {}
	};
	// [item 87] `collectInputAll` (RENAMED `prepareSimulationStep` at item 90)
	// / `collectResimInputAll` / `wipeAllForResync` LEFT MockNetSync for
	// MockInputResolution below — they moved off the real `SimulationNetSync`
	// onto the resolution peer at item 87, and
	// the manager now reads them off `InputResolutionT`, not `NetSyncT`. What
	// remains here is NetSync's own shrunk tick surface.
	struct MockNetSync
	{
		// [item 71] Not called by anything this file's cases reach at runtime —
		// `onGameSimulationAuthority()` is the `!m_runsPrediction` branch of
		// `onGameSimulation`, never taken once `shouldRunPrediction=true`. But a
		// runtime `if` (unlike `if constexpr`) still requires BOTH branches to
		// compile once the enclosing template member function is instantiated at
		// all, so this is needed to build even though it is never invoked.
		void setAuthorityGuardContext(unsigned int, int32_t) {}
	};

	// [item 87] The resolution peer's mock — `prepareSimulationStep`
	// (RENAMED from `collectInputAll` at item 90) /
	// `collectResimInputAll` / `wipeAllForResync`, matching
	// `SimulationInputResolutionTickConcept`. All three are reachable the
	// same way MockNetSync's methods used to be: `onGameSimulation()`
	// dispatches to the prediction/resim branches too, so their bodies must
	// compile even though only the authority branch runs in this file
	// (`shouldRunPrediction=false` everywhere but the one case below that
	// flips it); `wipeAllForResync` is reachable from the ctor's own
	// resync-callback lambda, compiled regardless of the runtime branch.
	struct MockInputResolution
	{
		void wipeAllForResync(unsigned int) {}
		int  prepareSimulationStep(const SimulationTimeStep&) { return 0; }
		int  collectResimInputAll(unsigned int) { return 0; }
	};

	struct MockReconciliation
	{
		void wipeAllForResync(unsigned int) {}

		// What the manager asked for, in call order.
		std::vector<std::uint32_t> receivedDepthPolicies;
		// What we answer with; 0 == "no character needs a resim".
		std::uint32_t             anchorToReport = 0u;
		unsigned int              deepSkipsToReport = 0u;

		unsigned int checkDivergenceAll(std::uint32_t maxAnchorDepthTicks,
		                                unsigned int* outDiagnosticDeepAnchorSkips = nullptr)
		{
			receivedDepthPolicies.push_back(maxAnchorDepthTicks);
			if (outDiagnosticDeepAnchorSkips != nullptr)
				*outDiagnosticDeepAnchorSkips = deepSkipsToReport;
			return anchorToReport;
		}

		// [item 71] Configurable, unlike `anchorToReport` / `deepSkipsToReport`'s
		// long-standing sibling here used to be: a hardcoded `{ return 0u; }` that
		// could never drive `SimulationManager::onPostGameSimulation`'s
		// `[Resim.Finish]` consume edge both ways (item 57 review, finding 1).
		// ⛔ CAS-consume semantics: this is a one-shot value per call, matching the
		// real `consumeResimAnchorsAll`'s "calling it twice is not safe" contract —
		// the wiring case below calls it exactly once per resim, same as the real
		// completion edge.
		unsigned int survivingToReturn = 0u;
		unsigned int consumeResimAnchorsAll() { return survivingToReturn; }

		TimeConfig::ResimTriggerPolicy lastPolicyPushed = TimeConfig{}.resimTriggerPolicy;
		unsigned int                   policyPushCount  = 0u;
		void setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy policy)
		{
			lastPolicyPushed = policy;
			++policyPushCount;
		}

		// [item 71] The remaining `SimulationReconciliationConcept` members this
		// mock had never needed until now — `onCheckIsSimilar`'s duck-typed rig
		// only ever reached `checkDivergenceAll`/`consumeResimAnchorsAll`/
		// `setResimTriggerPolicy`/`wipeAllForResync`. Reaching
		// `onPostGameSimulation`'s `[Resim.Finish]` block additionally needs the
		// prepare/apply/post-sweep trio and the diagnostics view its
		// `logSlotProvenanceAll()` call reads — all no-ops here since none of
		// them feed anything this file asserts on.
		void prepareResimAll(std::uint32_t) {}
		void applyResimAll() {}
		void postPredictionAll(const SimulationTimeStep&) {}
		ResimSweepDiagnostics postResimulationAll(const SimulationTimeStep&)
		{
			return ResimSweepDiagnostics{};
		}

		struct Diagnostics
		{
			void logSlotProvenanceAll() const {}
		};
		Diagnostics getDiagnostics() const { return Diagnostics{}; }
	};

	// [item 71] Templated (rather than typed on the concrete Mock storage/static
	// data) so `MockSystemsExec` needs no forward declaration of, or dependency
	// on, `MockStorage`/`MockStaticData`'s declaration order — it just forwards
	// whatever `SimulationManager` itself was built with, exactly as the real
	// `SimulationSystemsExecutor` peer is duck-typed over both.
	struct MockSystemsExec
	{
		template <typename StorageT, typename StaticDataT>
		void firePreIntegrate(const SimulationTimeStep&, StorageT&, const StaticDataT&) {}
		template <typename StorageT, typename StaticDataT>
		void firePostIntegrate(const SimulationTimeStep&, StorageT&, const StaticDataT&) {}
	};
	struct MockStorage {};
	struct MockStaticData {};

	using TestManager = SimulationManager<
		MockIntegrationExec, MockNetSync, MockInputResolution, MockReconciliation, MockSystemsExec,
		MockStorage, MockStaticData>;

	struct ManagerRig
	{
		MockIntegrationExec integration{};
		MockNetSync         netSync{};
		MockInputResolution inputResolution{};
		MockReconciliation  reconciliation{};
		MockSystemsExec     systemsExec{};
		MockStorage         storage{};
		MockStaticData      staticData{};

		// `shouldRunPrediction = false` (the default) so the ctor builds a
		// ServerTickClock and no ClientPredictionClock, which is all
		// `onCheckIsSimilar` needs: it reads no clock at all. The adapter
		// short-circuits this method on !runsPrediction in production, so calling
		// it here exercises the path a client takes without having to drive a
		// real prediction clock.
		//
		// [item 71] `shouldRunPrediction = true` is the one case in this file that
		// DOES need a real `ClientPredictionClock` — reaching
		// `onPostGameSimulation`'s `[Resim.Finish]` branch requires
		// `m_runsPrediction` true, which only exists behind that flag. Same
		// `TestManager`/Mock types as every other case here — only the ctor
		// argument differs — per the review's "extend the rig, don't duplicate
		// it" instruction.
		explicit ManagerRig(bool shouldRunPrediction = false)
			: manager{
				shouldRunPrediction,
				/*tickFrequency (fixed dt, seconds)=*/1.0 / 60.0,
				TestManager::Params{ integration, netSync, inputResolution, reconciliation, systemsExec,
				                     storage, staticData, nullptr } }
		{}

		TestManager manager;
	};

} // namespace

// ---------------------------------------------------------------------------
// THE KERNEL — `shouldSetPendingAnchor`, swept over every input combination.
//
// FOUR ROWS PER POLICY, and the interesting cell is the one that looks like an
// omission: `FrontierExact` IGNORES THE VERDICT. The legacy gate never read
// `predictionWasCorrect` (it is stored and consulted by nothing — item 30), so a
// verdict test there would make the legacy policy trigger strictly LESS often than
// the mechanism it exists to reproduce, on the very setting item 45's
// behaviour-neutral landing rests on.
// ---------------------------------------------------------------------------
TEST_CASE("ResimGate.Policy.ShouldSetPendingAnchorIsExhaustive",
	"[CorrectionCache][ResimGate]")
{
	using Policy = TimeConfig::ResimTriggerPolicy;

	// FrontierExact — POSITION only.
	REQUIRE(resimGate::shouldSetPendingAnchor(Policy::FrontierExact, /*atFrontier=*/true,  /*wasCorrect=*/false));
	REQUIRE(resimGate::shouldSetPendingAnchor(Policy::FrontierExact, /*atFrontier=*/true,  /*wasCorrect=*/true));
	REQUIRE_FALSE(resimGate::shouldSetPendingAnchor(Policy::FrontierExact, false, false));
	REQUIRE_FALSE(resimGate::shouldSetPendingAnchor(Policy::FrontierExact, false, true));

	// OnDisagreement — VERDICT only. Position is irrelevant, which is the whole
	// repair: a behind-frontier disagreement is what the legacy gate could not see.
	REQUIRE(resimGate::shouldSetPendingAnchor(Policy::OnDisagreement, /*atFrontier=*/false, /*wasCorrect=*/false));
	REQUIRE(resimGate::shouldSetPendingAnchor(Policy::OnDisagreement, /*atFrontier=*/true,  /*wasCorrect=*/false));
	REQUIRE_FALSE(resimGate::shouldSetPendingAnchor(Policy::OnDisagreement, false, true));
	REQUIRE_FALSE(resimGate::shouldSetPendingAnchor(Policy::OnDisagreement, true,  true));

	// constexpr, so the policy can be folded at compile time where it matters.
	static_assert(resimGate::shouldSetPendingAnchor(Policy::FrontierExact, true, true));
	static_assert(!resimGate::shouldSetPendingAnchor(Policy::OnDisagreement, true, true));
}

// ---------------------------------------------------------------------------
// THE DEPTH CEILING IS POLICY-SCOPED — the design decision most likely to be
// mistaken for an oversight, pinned so it reads as intent.
//
// It exists to bound the DISAGREEMENT trigger's worst-case depth. Under the legacy
// policy the trigger rate is already bounded by frontier-exact coincidence (~2
// events per archived run), so applying it there would bound nothing and would COST
// behaviour-neutrality: the depth skip would ABANDON an anchor the legacy gate
// retries (reachable via a long stranded-resim episode, item 42's I5 class).
//
// ⛔ AND THERE IS NO SECOND CEILING. A `resimCooldownTicks` trigger-rate limit was
// built and REMOVED on a user ruling (2026-08-11): deferring action on a correction
// already known to disagree is the defect item 45 repairs, with a smaller constant.
// The rate is instead bounded STRUCTURALLY — the gate is consulted only on non-resim
// frames and the anchor is consumed only on the completion edge, so at most one
// resim is in flight and at most one more is pending, and mid-replay landings
// coalesce into a single deeper replay. That bound is asserted by the termination
// tripwire `ACompletedResimClosesTheGateAndItStaysClosed` in the sibling semantics
// suite, which is therefore the COST bound as well as the correctness bound.
// ---------------------------------------------------------------------------
TEST_CASE("ResimGate.Policy.TheDepthCeilingAppliesOnlyToTheDisagreementTrigger",
	"[CorrectionCache][ResimGate]")
{
	using Policy = TimeConfig::ResimTriggerPolicy;

	REQUIRE_FALSE(resimGate::policyEnforcesDepthCeiling(Policy::FrontierExact));
	REQUIRE(resimGate::policyEnforcesDepthCeiling(Policy::OnDisagreement));

	// And the SHIPPED default is the one that enforces neither — the single fact
	// item 45's "lands behaviour-neutral" claim reduces to.
	REQUIRE(TimeConfig{}.resimTriggerPolicy == Policy::FrontierExact);
	REQUIRE_FALSE(resimGate::policyEnforcesDepthCeiling(TimeConfig{}.resimTriggerPolicy));
}

// ---------------------------------------------------------------------------
// THE DEPTH POLICY — SKIP, NOT CLAMP, and the boundary is INCLUSIVE at exactly
// `maxDepthTicks`.
//
// Inclusive because `rollbackWindowTicks` is documented as the maximum depth the
// reconciler will resimulate, so a resim of exactly that depth is the deepest
// PERMITTED one, not the first forbidden one. The `isAnomalousMiss` log gate one
// header over reads its own bound the same way, and the two being consistent is
// worth more than either convention on its own.
// ---------------------------------------------------------------------------
TEST_CASE("ResimGate.Policy.DepthPolicyAdmitsExactlyTheWindowAndSkipsBeyondIt",
	"[CorrectionCache][ResimGate]")
{
	// A neutral local, NOT named after the TimeConfig field: the R-P1
	// configurability lint flags a non-member lvalue named for a config field and
	// assigned its magic value, and a fixture is exactly the false positive it
	// cannot distinguish. The value is read from TimeConfig so the case tracks a
	// retune.
	const std::uint32_t kWindow = static_cast<std::uint32_t>(TimeConfig{}.rollbackWindowTicks);

	// 0 == NO POLICY. This is the shipped configuration's value and it must admit
	// everything, however deep.
	REQUIRE(resimGate::isAnchorWithinDepthPolicy(1u, 100000u, 0u));

	// Depth 1 .. kWindow are admitted; kWindow + 1 is not.
	REQUIRE(resimGate::isAnchorWithinDepthPolicy(999u, 1000u, kWindow));
	REQUIRE(resimGate::isAnchorWithinDepthPolicy(1000u - kWindow, 1000u, kWindow));
	REQUIRE_FALSE(resimGate::isAnchorWithinDepthPolicy(1000u - kWindow - 1u, 1000u, kWindow));
	REQUIRE_FALSE(resimGate::isAnchorWithinDepthPolicy(900u, 1000u, kWindow));

	// Depth 0 (anchor == frontier) and an anchor AHEAD of the frontier are both
	// admitted rather than treated as infinitely deep. Neither can be produced by a
	// landing — a correction above the frontier has no slot and is discarded before
	// it reaches the anchor — and the gate's own `anchor != frontier` clause already
	// handles equality, so an unsigned underflow here would turn a structurally
	// impossible value into a silent drop.
	REQUIRE(resimGate::isAnchorWithinDepthPolicy(1000u, 1000u, kWindow));
	REQUIRE(resimGate::isAnchorWithinDepthPolicy(1001u, 1000u, kWindow));

	static_assert(resimGate::isAnchorWithinDepthPolicy(50u, 1000u, 0u));
}

// ---------------------------------------------------------------------------
// THE WIRING, 1 of 2 — THE SETTER DOOR, and its second effect.
//
// ONE door, not two: the sibling `setResimCooldownTicks` was removed with the
// cooldown itself (ruling above).
//
// The setter half of the four-step config path (ini intake -> parse/validate ->
// setter -> proof line); the intake and the proof line are UE-side.
// `setResimTriggerPolicy` must ALSO fan the value out to the caches, which is what
// makes it the only legal door: a caller writing TimeConfig directly would leave
// every cache on the compiled default while the proof line reported the override.
// ---------------------------------------------------------------------------
TEST_CASE("ResimGate.Policy.TheManagerSetterIsTheOneWritableDoor",
	"[CorrectionCache][ResimGate]")
{
	ManagerRig rig;

	// The compiled default, read from the same source of truth rather than typed;
	// the defaults gate in TimeConfigDefaultsTest owns the value itself.
	REQUIRE(rig.manager.getTimeConfig().resimTriggerPolicy == TimeConfig{}.resimTriggerPolicy);

	rig.manager.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);
	REQUIRE(rig.manager.getTimeConfig().resimTriggerPolicy
	        == TimeConfig::ResimTriggerPolicy::OnDisagreement);
	// THE SECOND EFFECT: it reached reconciliation, which is what reaches the caches.
	REQUIRE(rig.reconciliation.policyPushCount == 1u);
	REQUIRE(rig.reconciliation.lastPolicyPushed == TimeConfig::ResimTriggerPolicy::OnDisagreement);

	rig.manager.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::FrontierExact);
	REQUIRE(rig.reconciliation.policyPushCount == 2u);
	REQUIRE(rig.reconciliation.lastPolicyPushed == TimeConfig::ResimTriggerPolicy::FrontierExact);
}

// ---------------------------------------------------------------------------
// THE WIRING, 2 of 2 — THE DEPTH POLICY IS DERIVED FROM THE LIVE CONFIG AND HANDED
// DOWN PER CALL.
//
// Read live rather than cached, for the reason the `[T39]` note on
// `sendCorrectionAll` gives about `correctionRotationK`: a value captured once
// would make an ini-driven setting silently ineffective. What the mock records is
// the ONE number reconciliation obeys, so this case pins both halves of the
// derivation — off under the legacy policy, `rollbackWindowTicks` under the
// designed one.
// ---------------------------------------------------------------------------
TEST_CASE("ResimGate.Policy.ManagerDerivesTheDepthPolicyFromPolicyAndWindow",
	"[CorrectionCache][ResimGate]")
{
	ManagerRig rig;
	rig.reconciliation.anchorToReport = 0u;   // no trigger; only the argument matters

	// Legacy default: NO depth policy, so the fold is byte-for-byte the
	// pre-item-45 one.
	rig.manager.onCheckIsSimilar();
	REQUIRE(rig.reconciliation.receivedDepthPolicies.size() == 1u);
	REQUIRE(rig.reconciliation.receivedDepthPolicies.back() == 0u);

	// The designed trigger: the window becomes the ceiling.
	rig.manager.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::OnDisagreement);
	rig.manager.onCheckIsSimilar();
	REQUIRE(rig.reconciliation.receivedDepthPolicies.back()
	        == static_cast<std::uint32_t>(TimeConfig{}.rollbackWindowTicks));

	// And back — the derivation is a function of the live value, not a latch.
	rig.manager.setResimTriggerPolicy(TimeConfig::ResimTriggerPolicy::FrontierExact);
	rig.manager.onCheckIsSimilar();
	REQUIRE(rig.reconciliation.receivedDepthPolicies.back() == 0u);
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 47] THE REPLAY WRITE RULE, SWEPT AS A PURE
// PREDICATE — one bit decides, four parameters only label.
//
// The wiring through a live cache lifecycle is proven case-by-population in
// `ResimGateSemanticsTest.cpp`'s `[ReplayProtect]` block. What belongs HERE is
// the same thing this file already does for the other three predicates: the
// exhaustive sweep, in the STL-only kernel where it can be swept at all.
//
// ⭐ THE PROPERTY THIS CASE EXISTS FOR, and it is the one a future reader is most
// likely to break: **`Written` depends on `slotContainsCorrectTick` and NOTHING
// ELSE.** That is what keeps the per-slot landing stamps OBSERVATIONAL — they are
// game-thread-written and physics-thread-read, riding the cache's pre-existing
// state-buffer race, and the whole safety argument for leaving them as plain
// `uint32`s is that a torn stamp can mislabel a COUNTER and can never mis-decide
// a WRITE. The first section below sweeps the other four parameters across their
// extremes with the bit clear and asserts `Written` every time; if that ever goes
// red, the plain stamps have become load-bearing and need the atomic treatment
// the anchor gets.
// ---------------------------------------------------------------------------
TEST_CASE("ResimGate.Policy.TheReplayWriteRuleIsOneBitWideAndTheRestIsLabelling",
	"[CorrectionCache][ResimGate][ReplayProtect]")
{
	using Outcome = resimGate::ResimSlotWriteOutcome;

	SECTION("bit CLEAR ⇒ Written, whatever the other four parameters say")
	{
		const std::uint32_t values[] = { 0u, 1u, 105u, 4294967295u };
		for (std::uint32_t slotSeq : values)
			for (std::uint32_t preparedSeq : values)
				for (std::uint32_t slotTick : values)
					for (std::uint32_t anchorTick : values)
						REQUIRE(resimGate::classifyResimSlotWrite(
							/*contains=*/false, slotSeq, preparedSeq, slotTick, anchorTick)
							== Outcome::Written);
	}

	SECTION("bit SET ⇒ never Written; the two clauses OR into FRESH")
	{
		// Truth table over the two clauses. `seqFresh` is slotSeq > preparedSeq;
		// `tickFresh` is slotTick >= capturedAnchor.
		//
		//   seq  tick | expected
		//    F    F   | ProtectedStale     <- the ONLY stale cell
		//    F    T   | ProtectedFresh     <- population (b) / (c): the tick clause
		//    T    F   | ProtectedFresh     <- population (a) below the anchor: the seq clause
		//    T    T   | ProtectedFresh
		REQUIRE(resimGate::classifyResimSlotWrite(true, /*slotSeq=*/3u, /*prepared=*/7u,
			/*slotTick=*/104u, /*anchor=*/110u) == Outcome::ProtectedStale);
		REQUIRE(resimGate::classifyResimSlotWrite(true, 3u, 7u, 110u, 110u) == Outcome::ProtectedFresh);
		REQUIRE(resimGate::classifyResimSlotWrite(true, 9u, 7u, 104u, 110u) == Outcome::ProtectedFresh);
		REQUIRE(resimGate::classifyResimSlotWrite(true, 9u, 7u, 110u, 110u) == Outcome::ProtectedFresh);
	}

	SECTION("the two clause BOUNDARIES, which are deliberately different")
	{
		// SEQUENCE is STRICT: a slot stamped at exactly the prepared value landed
		// BEFORE this resim was prepared, not during it. Equality is stale.
		REQUIRE(resimGate::classifyResimSlotWrite(true, 7u, 7u, 0u, 1u) == Outcome::ProtectedStale);
		REQUIRE(resimGate::classifyResimSlotWrite(true, 8u, 7u, 0u, 1u) == Outcome::ProtectedFresh);

		// TICK is INCLUSIVE: the slot AT the captured anchor is the tick this resim
		// was asked to act on, so it is fresh. (In a single-character resim that
		// slot is the restore SOURCE and never reaches this function at all; under
		// the multi-character min fold it very much does — see the two-cache case.)
		REQUIRE(resimGate::classifyResimSlotWrite(true, 0u, 0u, 109u, 110u) == Outcome::ProtectedStale);
		REQUIRE(resimGate::classifyResimSlotWrite(true, 0u, 0u, 110u, 110u) == Outcome::ProtectedFresh);
	}

	SECTION("a resim prepared with NO anchor classifies everything FRESH")
	{
		// `capturedAnchorTick == 0` is "this resim consumed nothing" — an
		// engine-side rewind we did not ask for (the same sentinel
		// `consumeResimAnchor` rejects up front). The tick clause is then
		// vacuously true. DELIBERATE OVER-PROTECTION in the safe direction, and it
		// is also what keeps the single-character structural zero honest: with no
		// anchor there is no `tick < anchor` for anything to fall into.
		REQUIRE(resimGate::classifyResimSlotWrite(true, 0u, 0u, 0u, 0u) == Outcome::ProtectedFresh);
		REQUIRE(resimGate::classifyResimSlotWrite(true, 0u, 5u, 1u, 0u) == Outcome::ProtectedFresh);
	}

	// constexpr, like its three neighbours — the classifier is usable in a
	// static_assert, which is the cheapest possible regression net on the one-bit
	// property above.
	static_assert(resimGate::classifyResimSlotWrite(false, 9u, 0u, 0u, 999u)
		== resimGate::ResimSlotWriteOutcome::Written);
	static_assert(resimGate::classifyResimSlotWrite(true, 0u, 0u, 0u, 999u)
		== resimGate::ResimSlotWriteOutcome::ProtectedStale);
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay task 59 / RN-9] THE RESIM-GATE PROBE ACCESSOR —
// WIRING, NOT ARITHMETIC. Same shape as the "WIRING, 1 of 2 / 2 of 2" cases
// above, and the same reason this lives beside them rather than in
// Reconciliation/ResimGateProbeTest.cpp: that file drives `ResimGateProbe` BY
// HAND (its own header says so) and can never show that a real
// `SimulationManager` call site feeds it. This case is the missing proof.
//
// ⚠ WHY THIS DOES NOT NEED SimulatableOwnerTraits, UNLIKE ITS FOUR SIBLINGS.
// `SimulationNetSync`'s four probe accessors (task 59, same backlog item) are
// fed from deep inside `prepareSimulationStep` / `collectResimInputAll` /
// `registerPredictionOwner`, which are variadic over a simulatable pack — a
// mock cannot drive them, so THEIR proof lives in og-brawler-tests against
// concrete owners. `m_resimGateProbe`'s feeders — `onCheckIsSimilar`,
// `noteDivergenceCheck` — take only primitives (a tick, a bool, an
// out-pointer) and are already proven reachable through the same duck-typed
// `ManagerRig` the two wiring cases above use. Nothing about them needs a
// concrete owner, so the cheaper mock-based proof is the correct one, not a
// shortcut.
//
// `fillSummary` (a live snapshot, never called by shipped code — see its own
// doc comment) is what lets this case read the window WITHOUT having to drive
// `kResimGateProbeWindowSamples` checks just to force a flush.
// ---------------------------------------------------------------------------
TEST_CASE("ResimGate.Policy.TheResimGateProbeAccessorObservesTheShippedFeed",
	"[CorrectionCache][ResimGate]")
{
	ManagerRig rig;

	// A declined check: `onCheckIsSimilar` -> `noteDivergenceCheck` ->
	// `m_resimGateProbe.noteCheck(false, ...)`, live-read via
	// `getDiagnostics().resimGateProbe()` with no window flush needed.
	rig.reconciliation.anchorToReport    = 0u;
	rig.reconciliation.deepSkipsToReport = 0u;
	rig.manager.onCheckIsSimilar();

	ResimGateWindowSummary snap;
	rig.manager.getDiagnostics().resimGateProbe().fillSummary(snap);
	REQUIRE(snap.checks == 1u);
	REQUIRE(snap.declined == 1u);
	REQUIRE(snap.requested == 0u);
	REQUIRE(snap.deepAnchorExclusions == 0u);

	// A requesting check with a nonzero depth-skip count: proves BOTH
	// `m_resimGateProbe.noteDeepAnchorSkips(...)` (called before `noteCheck`,
	// per the ordering note on that method) and `noteCheck(true, ...)` are
	// reached from the same shipped call site.
	rig.reconciliation.anchorToReport    = 42u;
	rig.reconciliation.deepSkipsToReport = 3u;
	rig.manager.onCheckIsSimilar();

	rig.manager.getDiagnostics().resimGateProbe().fillSummary(snap);
	REQUIRE(snap.checks == 2u);
	REQUIRE(snap.declined == 1u);
	REQUIRE(snap.requested == 1u);
	REQUIRE(snap.deepAnchorExclusions == 3u);
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 71 / RN-6 rework] THE SURVIVING-ANCHOR
// COUNT'S PRODUCTION CALL SITE — WIRING, NOT ARITHMETIC. Same shape and same
// reason as the case immediately above; this is its sibling for the apply
// edge rather than the check edge.
//
// Item 57 proved `survivingAnchors` MOVES (`SimulationReconciliationTest.cpp`,
// built on a bare `SimulationReconciliation` + `ResimGateProbe` pair calling
// `probe.noteSurvivingAnchors(...)` directly) but never drove the line it
// actually shipped:
//     SimulationManager.h:390
//     m_resimGateProbe.noteSurvivingAnchors(m_reconciliation.consumeResimAnchorsAll());
// — the `[Resim.Finish]` apply edge inside `onPostGameSimulation`. The item 57
// review (finding 1) named this file's `MockReconciliation::
// consumeResimAnchorsAll()` — then a hardcoded `{ return 0u; }` — as the
// missing half: give it a configurable value, the same way
// `anchorToReport`/`deepSkipsToReport` already let `checkDivergenceAll` be
// driven both ways, and reach `onPostGameSimulation` — not `onCheckIsSimilar`
// — with it. `survivingToReturn` above is that field.
//
// UNLIKE THE CASE ABOVE, this one needs `shouldRunPrediction = true`: the
// `[Resim.Finish]` branch is gated on `m_runsPrediction` and reads a real
// `ClientPredictionClock`, which exists only behind that flag —
// `onCheckIsSimilar` reads no clock at all, which is why every other case in
// this file gets away with `false`. The sequence below is the minimum real
// drive that reaches it, through the SAME public doors production uses:
//   1. Two ordinary prediction advances (`onGameSimulation`,
//      `isResimulation=false`) via the REAL clock's `advancePrediction()` —
//      not poked directly — bring `m_predictionTick` to 2. No authority tick
//      is ever fed to the estimator, so `getTargetPredictionTick()` stays
//      near 0 — far under the default `minTicksBeforeDriftCheck` (60) — so
//      every advance takes the plain dead-band Normal path
//      (`ClientPredictionClock.cpp`'s `doNormalAdvance`), which also advances
//      `m_resimulationTick` in lockstep while the two are in sync. This is
//      exact, deterministic arithmetic, not a race against drift correction.
//   2. `prepareResimulation(chaosStep=0, simTick=1)` — the SAME public entry
//      point Chaos's rewind hook calls in production — sets the resim cursor
//      to 1, one tick behind the frontier (2), so `isResimulating()` becomes
//      true.
//   3. One resim replay tick (`onGameSimulation`, `isResimulation=true` →
//      `onGameSimulationResimulation()`) calls the clock's real
//      `advanceResimulation()`, bringing the resim cursor to 2 — EQUAL to the
//      frontier. That is "our clock has caught up", the production comment's
//      own words at the `[Resim.Finish]` site: `isResimulating()` now reads
//      false while Chaos (via `updateInfo`) still reports a resim step.
//   4. `onPostGameSimulation(isResimulation=true)` — `chaosIsResim (true) &&
//      !clockIsResim (true)` — takes the `[Resim.Finish]` branch and reaches
//      line 390.
//
// `survivingToReturn` is set to a value (7u) this sequence cannot produce by
// accident anywhere else (not 0, not any tick/count in play), so a wiring gap
// that quietly ships 0 cannot pass by coincidence.
// ---------------------------------------------------------------------------
TEST_CASE("ResimGate.Policy.OnPostGameSimulationFeedsSurvivingAnchorsToTheProbe",
	"[CorrectionCache][ResimGate]")
{
	ManagerRig rig(/*shouldRunPrediction=*/true);

	// Step 1: advance the prediction frontier to tick 2. The clocks start in
	// sync at 0, so `advancePrediction()` keeps `m_resimulationTick` in
	// lockstep on every call — see the file comment above for why this stays
	// a plain Normal advance with no estimator input in play.
	rig.manager.onGameSimulation(SimulationUpdateInfo(/*isResimulation=*/false, /*isFirstResimulationStep=*/false));
	rig.manager.onGameSimulation(SimulationUpdateInfo(/*isResimulation=*/false, /*isFirstResimulationStep=*/false));

	// Step 2: prepare a resim from tick 1 — one behind the frontier (2) — the
	// same door `FSimulationManagerAsyncCallback::TriggerRewindIfNeeded_Internal`
	// calls in production.
	rig.manager.prepareResimulation(/*chaosStep=*/0, /*simTick=*/1u);

	// Step 3: one resim replay tick brings the resim cursor to 2, equal to the
	// frontier — the clock has caught up mid-Chaos-resim.
	rig.manager.onGameSimulation(SimulationUpdateInfo(/*isResimulation=*/true, /*isFirstResimulationStep=*/true));

	// Precondition this whole sequence exists to set up — named explicitly so
	// a future change to the arithmetic above fails HERE, with a clear cause,
	// rather than as a confusing failure on the real assertion below.
	REQUIRE_FALSE(rig.manager.editClientClock().isResimulating());

	rig.reconciliation.survivingToReturn = 7u;

	// THE LINE UNDER TEST: reached only if `onPostGameSimulation` takes the
	// `[Resim.Finish]` branch and does not discard `consumeResimAnchorsAll`'s
	// return. ⛔ Called exactly once here — `consumeResimAnchorsAll` is a
	// one-shot CAS-consume in production and this rig does not pretend a
	// second call is safe (see the mock's own comment).
	rig.manager.onPostGameSimulation(SimulationUpdateInfo(/*isResimulation=*/true, /*isFirstResimulationStep=*/false));

	ResimGateWindowSummary snap;
	rig.manager.getDiagnostics().resimGateProbe().fillSummary(snap);
	REQUIRE(snap.survivingAnchors == 7u);
}

#endif // WITH_LOW_LEVEL_TESTS
