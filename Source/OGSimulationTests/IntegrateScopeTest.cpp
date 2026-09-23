// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <iterator>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationIntegrationExecutor.h"
#include "OGSimulation/SimulationLog.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/SpatialQueryResult.h"

// ---------------------------------------------------------------------------
// [og-netcode-v2-field-defects task 16] simulationLog::IntegrateScope — the ambient
// (id, tick) log context. integrateAll is its ONE set site: every simulatable sees its
// own storage id and the step's tick while it integrates, and nothing else runs inside
// a scope — not firstResimStep, not the code after integrateAll returns.
// ---------------------------------------------------------------------------

namespace integratescopetests
{
	struct MockPhysicsAdapter
	{
		glm::mat4 getBodyTransform(BodyId) const { return glm::mat4(1.f); }
		void setBodyTransform(BodyId, const glm::mat4&) {}
		void addBodyTorque(BodyId, const glm::vec3&) {}
		void setBodyAngularVelocity(BodyId, const glm::vec3&) {}
		void setBodyLinearVelocity(BodyId, const glm::vec3&) {}
		void addBodyAcceleration(BodyId, const glm::vec3&) {}
		void addBodyVelocityChange(BodyId, const glm::vec3&) {}
		glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
		PhysicsBodyState captureBodyState(BodyId) const { return PhysicsBodyState{}; }
	};

	struct MockQueryAdapter
	{
		SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) { return SpatialQueryReport{}; }
		SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) { return SweepHit{}; }
		void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
		void enableShape(ShapeId) {}
		void disableShape(ShapeId) {}
	};

	static_assert(PhysicsBodyAdapter<MockPhysicsAdapter>);
	static_assert(SpatialQueryAdapter<MockQueryAdapter>);

	struct MockStaticData {};
	struct MockInput { int unused = 0; };
	struct OtherMockInput { int unused = 0; };

	// Records what currentIntegrateScope() reads from INSIDE each hook the executor calls.
	template <typename InputT>
	struct ScopeRecording
	{
		using InputType = InputT;

		std::vector<std::optional<simulationLog::IntegrateScopeInfo>> seenInIntegrate;
		std::vector<std::optional<simulationLog::IntegrateScopeInfo>> seenInFirstResimStep;

		void integrate(const SimulationTimeStep&, const InputT&,
					   MockPhysicsAdapter&, MockQueryAdapter&, const MockStaticData&)
		{
			seenInIntegrate.push_back(simulationLog::currentIntegrateScope());
		}

		void firstResimStep(MockPhysicsAdapter&, std::int32_t)
		{
			seenInFirstResimStep.push_back(simulationLog::currentIntegrateScope());
		}
	};

	using ScopeRecordingSimulatable = ScopeRecording<MockInput>;
	// A second simulatable TYPE, so the executor's fold over types is covered too.
	using OtherScopeRecordingSimulatable = ScopeRecording<OtherMockInput>;

	using Storage  = SimulationObjectStorage<ScopeRecordingSimulatable, OtherScopeRecordingSimulatable>;
	using Executor = SimulationIntegrationExecutor<MockStaticData, MockPhysicsAdapter, MockQueryAdapter,
												   ScopeRecordingSimulatable, OtherScopeRecordingSimulatable>;

	constexpr unsigned int kFirstId  = 3u;
	constexpr unsigned int kSecondId = 11u;
	constexpr unsigned int kOtherId  = 29u;
}

TEST_CASE("IntegrationExecutor.IntegrateScopeIsTheIntegratedIdAndTickAndNothingElse", "[IntegrationExecutor]")
{
	using namespace integratescopetests;

	Storage storage;
	storage.add<ScopeRecordingSimulatable>(kFirstId, ScopeRecordingSimulatable{});
	storage.add<ScopeRecordingSimulatable>(kSecondId, ScopeRecordingSimulatable{});
	storage.add<OtherScopeRecordingSimulatable>(kOtherId, OtherScopeRecordingSimulatable{});

	MockStaticData staticData;
	MockPhysicsAdapter physics;
	MockQueryAdapter query;
	Executor executor(storage, staticData, physics, query);

	Executor::ResolvedInputsType inputs;
	std::get<std::unordered_map<unsigned int, MockInput>>(inputs)[kFirstId]  = MockInput{};
	std::get<std::unordered_map<unsigned int, MockInput>>(inputs)[kSecondId] = MockInput{};
	std::get<std::unordered_map<unsigned int, OtherMockInput>>(inputs)[kOtherId] = OtherMockInput{};

	REQUIRE_FALSE(simulationLog::currentIntegrateScope().has_value());   // before: no scope

	const std::uint32_t ticks[] = { 9u, 10u, 4000000000u };
	for (const std::uint32_t tick : ticks)
	{
		executor.integrateAll(SimulationTimeStep(tick, /*isResimulating=*/false), inputs);
		CHECK_FALSE(simulationLog::currentIntegrateScope().has_value());  // after integrateAll returns
	}

	executor.firstResimStepAll(0);
	CHECK_FALSE(simulationLog::currentIntegrateScope().has_value());

	const auto checkRecord = [&](unsigned int id, const auto& sim)
	{
		INFO("id " << id);
		REQUIRE(sim.seenInIntegrate.size() == std::size(ticks));
		for (std::size_t i = 0; i < std::size(ticks); ++i)
		{
			INFO("step " << i);
			REQUIRE(sim.seenInIntegrate[i].has_value());
			CHECK(sim.seenInIntegrate[i]->id == id);
			CHECK(sim.seenInIntegrate[i]->tick == ticks[i]);
		}
		REQUIRE(sim.seenInFirstResimStep.size() == 1u);
		CHECK_FALSE(sim.seenInFirstResimStep[0].has_value());   // firstResimStep is NOT a set site
	};
	checkRecord(kFirstId,  storage.get<ScopeRecordingSimulatable>(kFirstId));
	checkRecord(kSecondId, storage.get<ScopeRecordingSimulatable>(kSecondId));
	checkRecord(kOtherId,  storage.get<OtherScopeRecordingSimulatable>(kOtherId));
}

TEST_CASE("IntegrationExecutor.IntegrateScopeClearsOnDestruction", "[IntegrationExecutor]")
{
	REQUIRE_FALSE(simulationLog::currentIntegrateScope().has_value());
	{
		simulationLog::IntegrateScope scope(7u, 9u);
		const auto info = simulationLog::currentIntegrateScope();
		REQUIRE(info.has_value());
		CHECK(info->id == 7u);
		CHECK(info->tick == 9u);
	}
	CHECK_FALSE(simulationLog::currentIntegrateScope().has_value());

	// Sequential (not nested) scopes are fine: the destructor really released the slot.
	{
		simulationLog::IntegrateScope scope(8u, 10u);
		CHECK(simulationLog::currentIntegrateScope()->id == 8u);
	}
	CHECK_FALSE(simulationLog::currentIntegrateScope().has_value());
}

// ⛔ THE VACUITY ARM for the no-nesting OG_CHECK. Hidden (`[.]`): OG_CHECK is UE's checkf in
// this target, which aborts rather than throws, so no default run or tag alias includes it.
// Run it by name; it is EXPECTED to fire the check and never reach the FAIL:
//     OGSimulationTests.exe "IntegrationExecutor.NestedIntegrateScopeTripsTheCheck"
TEST_CASE("IntegrationExecutor.NestedIntegrateScopeTripsTheCheck", "[.][IntegrationExecutorNestedScope]")
{
	simulationLog::IntegrateScope outer(1u, 1u);
	simulationLog::IntegrateScope inner(2u, 2u);
	FAIL("the IntegrateScope no-nesting OG_CHECK did NOT fire - it is vacuous in this target");
}

#endif // WITH_LOW_LEVEL_TESTS
