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

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{
    struct PoseWrite
    {
        glm::vec3 position{ 0.f };
        glm::quat rotation{ 1.f, 0.f, 0.f, 0.f };

        bool operator==(const PoseWrite&) const = default;
    };

    struct RestoreWrites
    {
        std::vector<glm::mat4> transforms;
        std::vector<PoseWrite> poses;
        std::vector<glm::vec3> linearVelocities;
        std::vector<glm::vec3> angularVelocities;

        size_t count() const { return transforms.size() + poses.size() + linearVelocities.size() + angularVelocities.size(); }
    };

    struct MatrixOnlyAdapter
    {
        glm::mat4 getBodyTransform(BodyId id) const
        {
            const PhysicsBodyState state = captureBodyState(id);
            glm::mat4 transform = glm::mat4_cast(state.rotation);
            transform[3] = glm::vec4(state.position, 1.f);
            return transform;
        }
        void setBodyTransform(BodyId id, const glm::mat4& transform) { writes[id.value].transforms.push_back(transform); }
        void addBodyTorque(BodyId, const glm::vec3&) {}
        void setBodyAngularVelocity(BodyId id, const glm::vec3& velocity) { writes[id.value].angularVelocities.push_back(velocity); }
        void setBodyLinearVelocity(BodyId id, const glm::vec3& velocity) { writes[id.value].linearVelocities.push_back(velocity); }
        void addBodyAcceleration(BodyId, const glm::vec3&) {}
        void addBodyVelocityChange(BodyId, const glm::vec3&) {}
        glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
        PhysicsBodyState captureBodyState(BodyId id) const
        {
            const auto it = restored.find(id.value);
            return it != restored.end() ? it->second : PhysicsBodyState{};
        }

        size_t writeCount() const
        {
            size_t total = 0;
            for (const auto& [id, bodyWrites] : writes)
                total += bodyWrites.count();
            return total;
        }

        std::map<uint32_t, PhysicsBodyState> restored;
        std::map<uint32_t, RestoreWrites> writes;
    };

    struct PoseWriterAdapter : MatrixOnlyAdapter
    {
        void setBodyPose(BodyId id, const glm::vec3& position, const glm::quat& rotation) { writes[id.value].poses.push_back(PoseWrite{ position, rotation }); }
    };

    static_assert(PhysicsBodyAdapter<MatrixOnlyAdapter>);
    static_assert(!PhysicsBodyPoseWriter<MatrixOnlyAdapter>);
    static_assert(PhysicsBodyAdapter<PoseWriterAdapter>);
    static_assert(PhysicsBodyPoseWriter<PoseWriterAdapter>);

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

    struct RestoreState
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

    struct RestoreAllState
    {
        RestoreState state;
        const RestoreState& getState() const { return state; }
    };

    struct RestoreBindings
    {
        BodyId ownBodyId;
    };

    struct FullDeclaration
    {
        using StateType = FullPart;
        static const PhysicsBodyState& bodyStateOf(const FullPart& part) { return part.bodyState; }
        RestoreBindings bindings;
    };

    struct LinearDeclaration
    {
        using StateType = LinearPart;
        static const LinearBodyState& bodyStateOf(const LinearPart& part) { return part.bodyState; }
        RestoreBindings bindings;
    };

    struct RestoreComposite
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
    struct RestoreSimulatable
    {
        using InputType = NoInput;

        void integrate(const SimulationTimeStep&, const NoInput&, AdapterT&, NoQueries&, const NoStaticData&) {}
        void firstResimStep(AdapterT&, int32_t) {}
        const RestoreAllState& getAllState() const { return allState; }
        const RestoreComposite& getPhysicsComposite() const { return composite; }

        RestoreAllState allState;
        RestoreComposite composite;
    };

    constexpr uint32_t kFullBody = 7u;
    constexpr uint32_t kLinearBody = 9u;

    const PhysicsBodyState kFullState{ glm::vec3(-0.0225f, 0.03125f, -0.015f),
        glm::normalize(glm::quat(0.9f, 0.3f, -0.2f, 0.1f)), glm::vec3(0.05f, -0.06f, -0.04f),
        glm::vec3(0.2f, -0.3f, 0.1f) };
    const LinearBodyState kLinearState{ glm::vec3(0.2f, 0.965f, -0.1f), glm::vec3(-0.02f, -0.03f, 0.01f) };
    const glm::quat kLinearBodyOwnRotation = glm::angleAxis(1.2f, glm::vec3(-1.f, -0.f, 0.f));
    const glm::vec3 kLinearBodyOwnSpin = glm::vec3(-0.f, -7.f, 0.f);

    PhysicsBodyState capturedLinearBody()
    {
        return PhysicsBodyState{ kLinearState.position, kLinearBodyOwnRotation, kLinearState.linearVelocity, kLinearBodyOwnSpin };
    }

    template <typename AdapterT>
    struct RestoreRig
    {
        RestoreRig()
        {
            RestoreSimulatable<AdapterT> simulatable;
            simulatable.composite.full.bindings.ownBodyId = BodyId{ kFullBody };
            simulatable.composite.linear.bindings.ownBodyId = BodyId{ kLinearBody };
            simulatable.allState.state.full.bodyState = kFullState;
            simulatable.allState.state.linear.bodyState = kLinearState;
            storage.template add<RestoreSimulatable<AdapterT>>(1u, std::move(simulatable));
            adapter.restored[kFullBody] = kFullState;
            adapter.restored[kLinearBody] = capturedLinearBody();
        }

        void push()
        {
            SimulationIntegrationExecutor<NoStaticData, AdapterT, NoQueries, RestoreSimulatable<AdapterT>> executor(storage, staticData, adapter, queries);
            executor.pushCorrectedBodyStatesAll();
        }

        SimulationObjectStorage<RestoreSimulatable<AdapterT>> storage;
        NoStaticData staticData;
        AdapterT adapter;
        NoQueries queries;
    };

    static_assert(SimulationIntegrationExecutorConcept<
        SimulationIntegrationExecutor<NoStaticData, PoseWriterAdapter, NoQueries, RestoreSimulatable<PoseWriterAdapter>>>);

    float nextFloatUp(float value)
    {
        return std::nextafter(value, std::numeric_limits<float>::infinity());
    }
} // namespace

TEST_CASE("IntegrationExecutor.RestoreSkip.ADeclarationEqualToTheRestoredBodyIsNotWritten", "[IntegrationExecutor]")
{
    RestoreRig<PoseWriterAdapter> poseRig;
    poseRig.push();
    REQUIRE(poseRig.adapter.writeCount() == 0u);

    RestoreRig<MatrixOnlyAdapter> matrixRig;
    matrixRig.push();
    REQUIRE(matrixRig.adapter.writeCount() == 0u);
}

TEST_CASE("IntegrationExecutor.RestoreSkip.AFullDeclarationDifferingInAnyFieldByOneUlpIsWritten", "[IntegrationExecutor]")
{
    const int field = GENERATE(0, 1, 2, 3);
    RestoreRig<PoseWriterAdapter> rig;
    PhysicsBodyState& restored = rig.adapter.restored[kFullBody];
    switch (field)
    {
    case 0:
        restored.position.y = nextFloatUp(restored.position.y);
        break;
    case 1:
        restored.rotation.w = nextFloatUp(restored.rotation.w);
        break;
    case 2:
        restored.linearVelocity.z = nextFloatUp(restored.linearVelocity.z);
        break;
    default:
        restored.angularVelocity.x = nextFloatUp(restored.angularVelocity.x);
        break;
    }
    INFO("field " << field);
    rig.push();

    REQUIRE(rig.adapter.writes.count(kLinearBody) == 0u);
    const RestoreWrites& full = rig.adapter.writes.at(kFullBody);
    REQUIRE(full.poses == std::vector<PoseWrite>{ PoseWrite{ kFullState.position, kFullState.rotation } });
    REQUIRE(full.linearVelocities == std::vector<glm::vec3>{ kFullState.linearVelocity });
    REQUIRE(full.angularVelocities == std::vector<glm::vec3>{ kFullState.angularVelocity });
    REQUIRE(full.transforms.empty());
}

TEST_CASE("IntegrationExecutor.RestoreSkip.ALinearDeclarationDifferingInPositionOrVelocityIsWritten", "[IntegrationExecutor]")
{
    const int field = GENERATE(0, 1);
    RestoreRig<PoseWriterAdapter> rig;
    PhysicsBodyState& restored = rig.adapter.restored[kLinearBody];
    if (field == 0)
        restored.position.x = nextFloatUp(restored.position.x);
    else
        restored.linearVelocity.y = nextFloatUp(restored.linearVelocity.y);
    INFO("field " << field);
    rig.push();

    REQUIRE(rig.adapter.writes.count(kFullBody) == 0u);
    const RestoreWrites& linear = rig.adapter.writes.at(kLinearBody);
    REQUIRE(linear.poses == std::vector<PoseWrite>{ PoseWrite{ kLinearState.position, kLinearBodyOwnRotation } });
    REQUIRE(linear.linearVelocities == std::vector<glm::vec3>{ kLinearState.linearVelocity });
    REQUIRE(linear.angularVelocities.empty());
    REQUIRE(linear.transforms.empty());
}

TEST_CASE("IntegrationExecutor.RestoreSkip.ALinearDeclarationIgnoresTheBodysRotationAndAngularVelocity", "[IntegrationExecutor]")
{
    RestoreRig<PoseWriterAdapter> rig;
    PhysicsBodyState& restored = rig.adapter.restored[kLinearBody];
    REQUIRE(restored.rotation != glm::quat(1.f, 0.f, 0.f, 0.f));
    REQUIRE(restored.angularVelocity != glm::vec3(0.f));
    restored.rotation = glm::angleAxis(-2.f, glm::vec3(-0.f, -0.f, 1.f));
    restored.angularVelocity = glm::vec3(9.f, -9.f, 9.f);
    rig.push();
    REQUIRE(rig.adapter.writeCount() == 0u);
}

TEST_CASE("IntegrationExecutor.PosePath.WithSetBodyPoseThePushNeverWritesAMatrix", "[IntegrationExecutor]")
{
    RestoreRig<PoseWriterAdapter> rig;
    rig.adapter.restored[kFullBody] = PhysicsBodyState{};
    rig.adapter.restored[kLinearBody].position += glm::vec3(0.f, 0.f, -0.05f);
    rig.push();

    const RestoreWrites& full = rig.adapter.writes.at(kFullBody);
    REQUIRE(full.transforms.empty());
    REQUIRE(full.poses == std::vector<PoseWrite>{ PoseWrite{ kFullState.position, kFullState.rotation } });
    REQUIRE(full.linearVelocities == std::vector<glm::vec3>{ kFullState.linearVelocity });
    REQUIRE(full.angularVelocities == std::vector<glm::vec3>{ kFullState.angularVelocity });

    const RestoreWrites& linear = rig.adapter.writes.at(kLinearBody);
    REQUIRE(linear.transforms.empty());
    REQUIRE(linear.poses == std::vector<PoseWrite>{ PoseWrite{ kLinearState.position, kLinearBodyOwnRotation } });
    REQUIRE(linear.linearVelocities == std::vector<glm::vec3>{ kLinearState.linearVelocity });
    REQUIRE(linear.angularVelocities.empty());
}

TEST_CASE("IntegrationExecutor.PosePath.WithoutSetBodyPoseThePushKeepsTheMatrixPath", "[IntegrationExecutor]")
{
    RestoreRig<MatrixOnlyAdapter> rig;
    rig.adapter.restored[kFullBody] = PhysicsBodyState{};
    rig.adapter.restored[kLinearBody].position += glm::vec3(0.f, 0.f, -0.05f);
    const glm::mat4 linearLiveTransform = rig.adapter.getBodyTransform(BodyId{ kLinearBody });
    rig.push();

    const RestoreWrites& full = rig.adapter.writes.at(kFullBody);
    REQUIRE(full.poses.empty());
    glm::mat4 expectedFull = glm::mat4_cast(kFullState.rotation);
    expectedFull[3] = glm::vec4(kFullState.position, 1.f);
    REQUIRE(full.transforms == std::vector<glm::mat4>{ expectedFull });
    REQUIRE(full.linearVelocities == std::vector<glm::vec3>{ kFullState.linearVelocity });
    REQUIRE(full.angularVelocities == std::vector<glm::vec3>{ kFullState.angularVelocity });

    const RestoreWrites& linear = rig.adapter.writes.at(kLinearBody);
    REQUIRE(linear.poses.empty());
    glm::mat4 expectedLinear = linearLiveTransform;
    expectedLinear[3] = glm::vec4(kLinearState.position, 1.f);
    REQUIRE(linear.transforms == std::vector<glm::mat4>{ expectedLinear });
    REQUIRE(linear.linearVelocities == std::vector<glm::vec3>{ kLinearState.linearVelocity });
    REQUIRE(linear.angularVelocities.empty());
}

#endif // WITH_LOW_LEVEL_TESTS
