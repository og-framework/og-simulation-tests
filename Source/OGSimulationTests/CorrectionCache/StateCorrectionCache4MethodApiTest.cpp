// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionCache.h"
#include "OGSimulation/SimulatableList.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationIntegrationExecutor.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/SystemsExecutor.h"

#include <cstdint>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// Stage 3 / Task 7 (D3.7): StateCorrectionCache 4-method external API.
//
// Covers the additive external surface on StateCorrectionCache — save_snapshot,
// load_snapshot, advance_frame, compute_checksum (proposal §2.2, GGRS Pattern B
// + GekkoNet Pattern G). The legacy internal API (tryInsertingCorrectState /
// prepareResimAll / applyResimAll) is unchanged and untested here.
//
// The load-bearing case is AdvanceFrameMatchesManagerPath: it drives the SAME
// SimulationIntegrationExecutor::integrateAll through BOTH the externally-driven
// advance_frame path and a real SimulationManager authority tick loop, then
// asserts compute_checksum agrees tick-for-tick. A divergence between the two
// paths is exactly the bug this task exists to prevent.
//
// TICK-0 SENTINEL CAVEAT (applies to every case below): the cache's tick buffer
// is value-initialized to 0 and getCacheIndex() matches by tick VALUE, so slot 0
// of a fresh cache legitimately answers to tick 0. Tests therefore use non-zero
// ticks whenever "not present" is the property under test.
//
// LOCAL isolation: this TU uses only scratch mock types (mock simulatable, mock
// physics/query adapters, mock net-sync/reconciliation peers). It deliberately
// references no brawler types — og-simulation core must verify standalone.
//////////////////////////////////////////////////////////////////////////////

namespace
{
	// --- Serializable state/input payloads ---------------------------------
	// Two int32 fields so ComputeChecksumChangesOnStateChange can perturb one
	// field and leave the other alone.
	struct CounterState
	{
		std::int32_t position = 0;
		std::int32_t velocity = 0;
	};

	struct CounterInput
	{
		std::int32_t accel = 0;
	};
} // namespace

template <>
struct SerializableFields<CounterState>
{
	static constexpr auto get()
	{
		return std::make_tuple(
			SIM_MEMBER(CounterState, position),
			SIM_MEMBER(CounterState, velocity));
	}
};

template <>
struct SerializableFields<CounterInput>
{
	static constexpr auto get()
	{
		return std::make_tuple(SIM_MEMBER(CounterInput, accel));
	}
};

namespace
{
	// The task pins these type names: StateT / InputT are the semantic aliases of
	// SimulationComposite<Ts...> (SimulationComposite.h:56-57).
	using StateComposite = SimulationStateComposite<CounterState>;
	using InputComposite = SimulationInputComposite<CounterInput>;
	using TestCache      = StateCorrectionCache<StateComposite, InputComposite>;

	constexpr unsigned int kCapacity = static_cast<unsigned int>(TestCache::StateBufferSize);
	constexpr unsigned int kCharacterId = 1;
	constexpr float        kDeltaSeconds = 1.0f / 60.0f;

	StateComposite makeState(std::int32_t position, std::int32_t velocity)
	{
		return StateComposite(CounterState{ position, velocity });
	}

	// --- Mock adapters ------------------------------------------------------
	// Minimal shapes satisfying PhysicsBodyAdapter / SpatialQueryAdapter. The mock
	// simulatable never touches them; they exist to instantiate the executor.
	struct MockPhysicsAdapter
	{
		glm::mat4 getBodyTransform(BodyId) const { return glm::mat4(1.f); }
		void setBodyTransform(BodyId, const glm::mat4&) {}
		void addBodyTorque(BodyId, const glm::vec3&) {}
		void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
		void setBodyLinearVelocity(BodyId, const glm::vec3&) {}
		glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
		PhysicsBodyState captureBodyState(BodyId) const { return PhysicsBodyState{}; }
	};

	struct MockQueryAdapter
	{
		SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) { return SpatialQueryReport{}; }
		void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
		void enableShape(ShapeId) {}
		void disableShape(ShapeId) {}
	};

	static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);
	static_assert(SpatialQueryAdapter<MockQueryAdapter>);

	struct MockStaticData {};

	// --- Mock simulatable ---------------------------------------------------
	// Deterministic integer integration, deliberately tick-DEPENDENT: the tick
	// contribution means AdvanceFrameMatchesManagerPath would fail if either path
	// threaded the wrong SimulationTimeStep, not just the wrong input.
	struct MockSimulatable
	{
		using InputType = CounterInput;

		CounterState state{};

		void integrate(
			const SimulationTimeStep& step,
			const CounterInput&       input,
			MockPhysicsAdapter&,
			MockQueryAdapter&,
			const MockStaticData&)
		{
			state.velocity += input.accel;
			state.position += state.velocity + static_cast<std::int32_t>(step.getTick());
		}

		void firstResimStep(MockPhysicsAdapter&, std::int32_t) {}
	};

	using MockStorage  = SimulationObjectStorage<MockSimulatable>;
	using MockExecutor = SimulationIntegrationExecutor<
		MockStaticData, MockPhysicsAdapter, MockQueryAdapter, MockSimulatable>;
	using MockResolvedInputs = ResolvedInputs<MockSimulatable>;

	MockResolvedInputs makeResolvedInputs(const CounterInput& input)
	{
		MockResolvedInputs inputs;
		std::get<std::unordered_map<unsigned int, CounterInput>>(inputs)[kCharacterId] = input;
		return inputs;
	}

	// The per-tick input schedule both paths replay. Varying, non-monotonic, and
	// including a zero so a path that silently drops or reorders inputs diverges.
	std::int32_t accelForStep(int stepIndex)
	{
		constexpr std::int32_t schedule[] = { 3, -1, 0, 7, 2, -5, 1, 4 };
		return schedule[stepIndex % (sizeof(schedule) / sizeof(schedule[0]))];
	}

	// --- Mock manager peers -------------------------------------------------
	// SimulationManager is duck-typed on its NetSync / Reconciliation / Systems
	// peers, and member functions of a class template instantiate only when used —
	// so the authority tick path needs only what it actually calls.
	struct MockNetSync
	{
		MockResolvedInputs nextInputs;

		MockResolvedInputs collectInputAll(const SimulationTimeStep&) const { return nextInputs; }

		template <typename TickT, typename WindowT>
		void setAuthorityGuardContext(TickT, WindowT) {}

		void wipeAllForResync(unsigned int) {}
	};

	// Never invoked on the authority path, but wipeAllForResync must EXIST: the
	// ctor's resync-callback lambda body is compiled regardless of the runtime
	// shouldRunPrediction branch it sits behind.
	struct MockReconciliation
	{
		void wipeAllForResync(unsigned int) {}

		// Same reason: onGameSimulation() dispatches to the prediction/resim
		// branches too, so their bodies must compile even though only the
		// authority branch ever runs here.
		MockResolvedInputs collectResimInputAll(unsigned int) const { return MockResolvedInputs{}; }
	};

	using MockSystemsExec = NullSystemsExecutor<SimulatableList<MockSimulatable>, MockStaticData>;

	using MockManager = SimulationManager<
		MockExecutor, MockNetSync, MockReconciliation, MockSystemsExec, MockStorage, MockStaticData>;

	// Owns a storage + adapters + executor triple so the two paths in
	// AdvanceFrameMatchesManagerPath each get an independent, identically-seeded rig.
	struct SimRig
	{
		MockStorage        storage;
		MockStaticData     staticData;
		MockPhysicsAdapter physics;
		MockQueryAdapter   query;
		MockExecutor       executor{ storage, staticData, physics, query };

		SimRig() { storage.add<MockSimulatable>(kCharacterId, MockSimulatable{}); }

		MockSimulatable& sim() { return storage.get<MockSimulatable>(kCharacterId); }
	};
} // namespace

TEST_CASE("CorrectionCache.SaveThenLoadRoundTrip", "[CorrectionCache]")
{
	TestCache cache(nullptr);

	cache.save_snapshot(100, makeState(42, -7));

	StateComposite loaded = makeState(0, 0);
	REQUIRE(cache.load_snapshot(100, loaded));
	REQUIRE(loaded.get<CounterState>().position == 42);
	REQUIRE(loaded.get<CounterState>().velocity == -7);

	// Byte-for-byte equality via the serialized-bytes checksum, not just field-wise.
	TestCache reference(nullptr);
	reference.save_snapshot(100, loaded);
	REQUIRE(reference.compute_checksum(100) == cache.compute_checksum(100));
}

TEST_CASE("CorrectionCache.LoadOnEmptySlotReturnsFalse", "[CorrectionCache]")
{
	TestCache cache(nullptr);

	StateComposite loaded = makeState(11, 13);
	REQUIRE_FALSE(cache.load_snapshot(100, loaded));

	// A failed load leaves out_state untouched (caller's value survives).
	REQUIRE(loaded.get<CounterState>().position == 11);
	REQUIRE(loaded.get<CounterState>().velocity == 13);

	// A save at a DIFFERENT tick does not make the queried tick loadable.
	cache.save_snapshot(101, makeState(1, 2));
	REQUIRE_FALSE(cache.load_snapshot(100, loaded));
	REQUIRE(cache.load_snapshot(101, loaded));
}

TEST_CASE("CorrectionCache.RingWrapOverwrite", "[CorrectionCache]")
{
	TestCache cache(nullptr);

	// Save ticks [0, kCapacity + 5) — ticks kCapacity..kCapacity+4 wrap onto ring
	// slots 0..4, evicting ticks 0..4. Tick 5 (slot 5) is the first survivor.
	for (unsigned int tick = 0; tick < kCapacity + 5; ++tick)
		cache.save_snapshot(tick, makeState(static_cast<std::int32_t>(tick), 0));

	StateComposite loaded = makeState(0, 0);

	// Overwritten: tick 0's slot now holds tick kCapacity.
	REQUIRE_FALSE(cache.load_snapshot(0, loaded));

	// Survivor: tick 5 was never remapped (tick kCapacity+5 was not saved).
	REQUIRE(cache.load_snapshot(5, loaded));
	REQUIRE(loaded.get<CounterState>().position == 5);

	// The wrapping writer itself is loadable and holds ITS state, not the evicted one.
	REQUIRE(cache.load_snapshot(kCapacity, loaded));
	REQUIRE(loaded.get<CounterState>().position == static_cast<std::int32_t>(kCapacity));

	// Newest tick survives.
	REQUIRE(cache.load_snapshot(kCapacity + 4, loaded));
	REQUIRE(loaded.get<CounterState>().position == static_cast<std::int32_t>(kCapacity + 4));
}

TEST_CASE("CorrectionCache.AdvanceFrameMatchesManagerPath", "[CorrectionCache]")
{
	constexpr int kSteps = 12;

	// --- Path B: the real SimulationManager authority tick loop --------------
	SimRig             managerRig;
	MockNetSync        netSync;
	MockReconciliation reconciliation;
	MockSystemsExec    systemsExec;

	MockManager manager(
		/*shouldRunPrediction=*/false,
		/*tickFrequency (= fixed physics dt, per the ctor's own note)=*/ static_cast<double>(kDeltaSeconds),
		MockManager::Params{
			managerRig.executor, netSync, reconciliation, systemsExec,
			managerRig.storage, managerRig.staticData, nullptr });

	TestCache managerCache(nullptr);
	managerCache.save_snapshot(0, makeState(0, 0));   // seed: pre-tick state at tick 0

	std::vector<unsigned int> ticks;
	ticks.reserve(kSteps);

	for (int i = 0; i < kSteps; ++i)
	{
		netSync.nextInputs = makeResolvedInputs(CounterInput{ accelForStep(i) });

		// The production dispatch entry point (what FSimulationManagerAsyncCallback
		// calls); with shouldRunPrediction=false it routes to the authority tick.
		manager.onGameSimulation(SimulationUpdateInfo(
			/*isResimulation=*/false, /*isFirstResimulationStep=*/false));

		const unsigned int tick = manager.getServerClock().getTick();
		ticks.push_back(tick);
		managerCache.save_snapshot(
			tick, StateComposite(managerRig.sim().state));
	}

	// --- Path A: externally-driven advance_frame ----------------------------
	// The integrate functor restores prevState into storage, runs the SAME
	// SimulationIntegrationExecutor::integrateAll, and reads the result back —
	// so the cache, not the storage, is the source of truth between steps.
	SimRig advanceRig;

	TestCache::IntegrateFn integrateFn =
		[&advanceRig](uint32 tick, const StateComposite& prevState, const InputComposite& input)
		{
			advanceRig.sim().state = prevState.get<CounterState>();
			advanceRig.executor.integrateAll(
				SimulationTimeStep(tick, /*isResimulating=*/false, StepKind::Normal, kDeltaSeconds),
				makeResolvedInputs(input.get<CounterInput>()));
			return StateComposite(advanceRig.sim().state);
		};

	TestCache advanceCache(nullptr, integrateFn);
	advanceCache.save_snapshot(0, makeState(0, 0));   // identical seed

	for (int i = 0; i < kSteps; ++i)
		advanceCache.advance_frame(ticks[i], InputComposite(CounterInput{ accelForStep(i) }));

	// --- The two paths agree tick-for-tick ----------------------------------
	for (int i = 0; i < kSteps; ++i)
	{
		INFO("step " << i << " tick " << ticks[i]);
		REQUIRE(advanceCache.compute_checksum(ticks[i]) == managerCache.compute_checksum(ticks[i]));
	}

	// Guard against the vacuous pass where both paths produced identical
	// zero-states (or every checksum collapsed to the not-in-cache 0 sentinel).
	REQUIRE(advanceRig.sim().state.position != 0);
	REQUIRE(advanceRig.sim().state.position == managerRig.sim().state.position);
	REQUIRE(advanceRig.sim().state.velocity == managerRig.sim().state.velocity);
	REQUIRE(advanceCache.compute_checksum(ticks.back()) != 0u);
}

TEST_CASE("CorrectionCache.ComputeChecksumStableAcrossRuns", "[CorrectionCache]")
{
	const StateComposite state = makeState(123, -456);

	// Same state, two independent caches, different ring slots — the checksum
	// covers the serialized state bytes only, never slot identity or cache history.
	TestCache first(nullptr);
	TestCache second(nullptr);
	first.save_snapshot(10, state);
	second.save_snapshot(70, state);   // 70 % capacity == 10, different tick

	const uint32 checksum = first.compute_checksum(10);
	REQUIRE(checksum == second.compute_checksum(70));

	// Repeated computation is pure.
	REQUIRE(first.compute_checksum(10) == checksum);

	// Re-saving the same value re-derives the same checksum.
	first.save_snapshot(10, state);
	REQUIRE(first.compute_checksum(10) == checksum);

	// Not-in-window ticks answer 0 rather than hashing a stale slot.
	REQUIRE(first.compute_checksum(999) == 0u);
}

TEST_CASE("CorrectionCache.ComputeChecksumChangesOnStateChange", "[CorrectionCache]")
{
	TestCache cache(nullptr);

	cache.save_snapshot(20, makeState(5, 9));
	const uint32 baseline = cache.compute_checksum(20);

	// Perturb ONE field; the other is unchanged.
	cache.save_snapshot(20, makeState(6, 9));
	REQUIRE(cache.compute_checksum(20) != baseline);

	// Perturb the OTHER field from the same baseline.
	cache.save_snapshot(20, makeState(5, 10));
	REQUIRE(cache.compute_checksum(20) != baseline);

	// Restoring the original value restores the original checksum.
	cache.save_snapshot(20, makeState(5, 9));
	REQUIRE(cache.compute_checksum(20) == baseline);
}

#endif // WITH_LOW_LEVEL_TESTS
