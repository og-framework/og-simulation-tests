// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/BodyId.h"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationIntegrationExecutor.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/SpatialQueryAdapter.h"
#include "OGSimulation/SpatialQueryResult.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{
namespace pushStartOfStep
{
    struct PoseWrite
    {
        glm::vec3 position{ 0.f };
        glm::quat rotation{ 1.f, 0.f, 0.f, 0.f };

        bool operator==(const PoseWrite&) const = default;
    };

    struct BodyWrites
    {
        std::vector<glm::mat4> transforms;
        std::vector<PoseWrite> poses;
        std::vector<glm::vec3> linearVelocities;
        std::vector<glm::vec3> angularVelocities;

        size_t count() const { return transforms.size() + poses.size() + linearVelocities.size() + angularVelocities.size(); }
    };

    // captureBodyState returns `stale`: the phase an engine's post-solve capture reads, which a
    // rollback does not restore (Chaos's P/Q).
    struct CaptureOnlyAdapter
    {
        glm::mat4 getBodyTransform(BodyId) const { return glm::mat4(1.f); }
        void setBodyTransform(BodyId id, const glm::mat4& transform) { writes[id.value].transforms.push_back(transform); }
        void setBodyPose(BodyId id, const glm::vec3& position, const glm::quat& rotation) { writes[id.value].poses.push_back(PoseWrite{ position, rotation }); }
        void addBodyTorque(BodyId, const glm::vec3&) {}
        void setBodyAngularVelocity(BodyId id, const glm::vec3& velocity) { writes[id.value].angularVelocities.push_back(velocity); }
        void setBodyLinearVelocity(BodyId id, const glm::vec3& velocity) { writes[id.value].linearVelocities.push_back(velocity); }
        void addBodyAcceleration(BodyId, const glm::vec3&) {}
        void addBodyVelocityChange(BodyId, const glm::vec3&) {}
        glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
        PhysicsBodyState captureBodyState(BodyId id) const
        {
            ++staleReads;
            return lookUp(stale, id);
        }

        static PhysicsBodyState lookUp(const std::map<uint32_t, PhysicsBodyState>& states, BodyId id)
        {
            const auto it = states.find(id.value);
            return it != states.end() ? it->second : PhysicsBodyState{};
        }

        std::map<uint32_t, PhysicsBodyState> stale;
        std::map<uint32_t, PhysicsBodyState> startOfStep;
        std::map<uint32_t, BodyWrites> writes;
        mutable int staleReads = 0;
        mutable int startOfStepReads = 0;
    };

    // captureStartOfStepBodyState returns `startOfStep`: the state the rollback restored and the
    // replay starts from (Chaos's X/R/V/W).
    struct StartOfStepAdapter : CaptureOnlyAdapter
    {
        PhysicsBodyState captureStartOfStepBodyState(BodyId id) const
        {
            ++startOfStepReads;
            return lookUp(startOfStep, id);
        }
    };

    static_assert(PhysicsBodyAdapter<CaptureOnlyAdapter>);
    static_assert(PhysicsBodyPoseWriter<CaptureOnlyAdapter>);
    static_assert(PhysicsBodyAdapter<StartOfStepAdapter>);
    static_assert(PhysicsBodyPoseWriter<StartOfStepAdapter>);
    static_assert(!PhysicsBodyStartOfStepReader<CaptureOnlyAdapter>);
    static_assert(PhysicsBodyStartOfStepReader<StartOfStepAdapter>);

    struct NoQueries
    {
        SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) { return SpatialQueryReport{}; }
        SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) { return SweepHit{}; }
        void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
        void enableShape(ShapeId) {}
        void disableShape(ShapeId) {}
    };

    static_assert(SpatialQueryAdapter<NoQueries>);

    struct FullPart
    {
        PhysicsBodyState bodyState;
    };

    struct LinearPart
    {
        LinearBodyState bodyState;
    };

    struct PartsState
    {
        FullPart full;
        LinearPart linear;

        template <typename S>
        const S& get() const
        {
            if constexpr (std::is_same_v<S, FullPart>)
                return full;
            else
                return linear;
        }
    };

    struct PartsAllState
    {
        PartsState state;
        const PartsState& getState() const { return state; }
    };

    struct OwnBodyBindings
    {
        BodyId ownBodyId;
    };

    struct FullDeclaration
    {
        using StateType = FullPart;
        static const PhysicsBodyState& bodyStateOf(const FullPart& part) { return part.bodyState; }
        OwnBodyBindings bindings;
    };

    struct LinearDeclaration
    {
        using StateType = LinearPart;
        static const LinearBodyState& bodyStateOf(const LinearPart& part) { return part.bodyState; }
        OwnBodyBindings bindings;
    };

    struct PartsComposite
    {
        FullDeclaration full;
        LinearDeclaration linear;

        template <typename F>
        void forEach(F&& f) const
        {
            f(full);
            f(linear);
        }
    };

    struct NoStaticData
    {
    };

    struct NoInput
    {
        int unused = 0;
    };

    template <typename AdapterT>
    struct PartsSimulatable
    {
        using InputType = NoInput;

        void integrate(const SimulationTimeStep&, const NoInput&, AdapterT&, NoQueries&, const NoStaticData&) {}
        void firstResimStep(AdapterT&, int32_t) {}
        const PartsAllState& getAllState() const { return allState; }
        const PartsComposite& getPhysicsComposite() const { return composite; }

        PartsAllState allState;
        PartsComposite composite;
    };

    constexpr uint32_t kFullBody = 3u;
    constexpr uint32_t kLinearBody = 5u;

    const PhysicsBodyState kFullState{ glm::vec3(-7.5f, 2.25f, 0.125f), glm::normalize(glm::quat(0.8f, -0.2f, 0.4f, 0.1f)), glm::vec3(1.f, -5.f, 6.f),
        glm::vec3(-0.3f, 0.2f, 0.1f) };
    const LinearBodyState kLinearState{ glm::vec3(30.f, -40.f, 88.5f), glm::vec3(2.f, -4.f, 8.f) };
    const glm::quat kRewoundRotation = glm::angleAxis(0.7f, glm::vec3(0.f, 0.f, 1.f));
    const glm::quat kStaleRotation = glm::angleAxis(-1.9f, glm::vec3(1.f, 0.f, 0.f));
    const glm::vec3 kStaleOffset(4.f, -3.f, 2.f);

    PhysicsBodyState linearBody(const glm::vec3& position, const glm::quat& rotation)
    {
        return PhysicsBodyState{ position, rotation, kLinearState.linearVelocity, glm::vec3(0.f, 3.f, 0.f) };
    }

    template <typename AdapterT>
    struct PushRig
    {
        PushRig()
        {
            PartsSimulatable<AdapterT> simulatable;
            simulatable.composite.full.bindings.ownBodyId = BodyId{ kFullBody };
            simulatable.composite.linear.bindings.ownBodyId = BodyId{ kLinearBody };
            simulatable.allState.state.full.bodyState = kFullState;
            simulatable.allState.state.linear.bodyState = kLinearState;
            storage.template add<PartsSimulatable<AdapterT>>(1u, std::move(simulatable));
        }

        void push()
        {
            SimulationIntegrationExecutor<NoStaticData, AdapterT, NoQueries, PartsSimulatable<AdapterT>> executor(storage, staticData, adapter, queries);
            executor.pushCorrectedBodyStatesAll();
        }

        SimulationObjectStorage<PartsSimulatable<AdapterT>> storage;
        NoStaticData staticData;
        AdapterT adapter;
        NoQueries queries;
    };

    // The full body was not corrected: the rollback restored exactly its declared state, while the
    // stale read still holds an older pose. The linear body WAS corrected: the rollback restored a
    // different position, while the stale read happens to equal the declaration.
    template <typename AdapterT>
    void stageUncorrectedFullAndCorrectedLinear(PushRig<AdapterT>& rig)
    {
        PhysicsBodyState staleFull = kFullState;
        staleFull.position += kStaleOffset;
        rig.adapter.stale[kFullBody] = staleFull;
        rig.adapter.startOfStep[kFullBody] = kFullState;
        rig.adapter.stale[kLinearBody] = linearBody(kLinearState.position, kStaleRotation);
        rig.adapter.startOfStep[kLinearBody] = linearBody(kLinearState.position + glm::vec3(0.f, 0.f, 1.f), kRewoundRotation);
    }
} // namespace pushStartOfStep
} // namespace

TEST_CASE("IntegrationExecutor.StartOfStep.TheRestoreSkipComparesWithTheStartOfStepState", "[IntegrationExecutor]")
{
    using namespace pushStartOfStep;
    PushRig<StartOfStepAdapter> rig;
    stageUncorrectedFullAndCorrectedLinear(rig);
    rig.push();

    INFO("the uncorrected full body equals its start-of-step state, so it is skipped");
    REQUIRE(rig.adapter.writes.count(kFullBody) == 0u);
    INFO("the corrected linear body differs from its start-of-step state, so it is written");
    REQUIRE(rig.adapter.writes.count(kLinearBody) == 1u);
    const BodyWrites& linear = rig.adapter.writes.at(kLinearBody);
    REQUIRE(linear.poses.size() == 1u);
    REQUIRE(linear.poses[0].position == kLinearState.position);
    REQUIRE(linear.linearVelocities == std::vector<glm::vec3>{ kLinearState.linearVelocity });
    REQUIRE(linear.angularVelocities.empty());
    REQUIRE(linear.transforms.empty());
}

TEST_CASE("IntegrationExecutor.StartOfStep.ALinearDeclarationsPoseTakesTheStartOfStepRotation", "[IntegrationExecutor]")
{
    using namespace pushStartOfStep;
    PushRig<StartOfStepAdapter> rig;
    stageUncorrectedFullAndCorrectedLinear(rig);
    rig.push();

    REQUIRE(rig.adapter.writes.count(kLinearBody) == 1u);
    const BodyWrites& linear = rig.adapter.writes.at(kLinearBody);
    REQUIRE(linear.poses == std::vector<PoseWrite>{ PoseWrite{ kLinearState.position, kRewoundRotation } });
    REQUIRE(rig.adapter.staleReads == 0);
    REQUIRE(rig.adapter.startOfStepReads == 2);
}

TEST_CASE("IntegrationExecutor.StartOfStep.WithoutTheCapabilityThePushStillReadsCaptureBodyState", "[IntegrationExecutor]")
{
    using namespace pushStartOfStep;
    PushRig<CaptureOnlyAdapter> rig;
    stageUncorrectedFullAndCorrectedLinear(rig);
    rig.push();

    REQUIRE(rig.adapter.staleReads == 2);
    REQUIRE(rig.adapter.startOfStepReads == 0);
    INFO("judged by captureBodyState: the full body differs from it and is written, the linear body equals it and is skipped");
    REQUIRE(rig.adapter.writes.count(kLinearBody) == 0u);
    REQUIRE(rig.adapter.writes.count(kFullBody) == 1u);
    const BodyWrites& full = rig.adapter.writes.at(kFullBody);
    REQUIRE(full.poses == std::vector<PoseWrite>{ PoseWrite{ kFullState.position, kFullState.rotation } });
    REQUIRE(full.linearVelocities == std::vector<glm::vec3>{ kFullState.linearVelocity });
    REQUIRE(full.angularVelocities == std::vector<glm::vec3>{ kFullState.angularVelocity });
}

#endif // WITH_LOW_LEVEL_TESTS
