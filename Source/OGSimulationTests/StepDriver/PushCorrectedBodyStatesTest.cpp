// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <map>
#include <vector>

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

// ---------------------------------------------------------------------------
// SimulationIntegrationExecutor::pushCorrectedBodyStatesAll over the REAL executor: the
// engine-free write of og-sim's restored body states into the physics world after a rollback.
// A full PhysicsBodyState declaration writes position, rotation and both velocities; a
// LinearBodyState declaration writes position and linear velocity only, and keeps the body's own
// rotation and angular velocity.
// ---------------------------------------------------------------------------

namespace
{
    struct BodyWrites
    {
        std::vector<glm::mat4> transforms;
        std::vector<glm::vec3> linearVelocities;
        std::vector<glm::vec3> angularVelocities;
    };

    struct RecordingPhysicsAdapter
    {
        glm::mat4 getBodyTransform(BodyId id) const
        {
            const auto it = current.find(id.value);
            return it != current.end() ? it->second : glm::mat4(1.f);
        }
        void setBodyTransform(BodyId id, const glm::mat4& m) { writes[id.value].transforms.push_back(m); }
        void addBodyTorque(BodyId, const glm::vec3&) {}
        void setBodyAngularVelocity(BodyId id, const glm::vec3& v) { writes[id.value].angularVelocities.push_back(v); }
        void setBodyLinearVelocity(BodyId id, const glm::vec3& v) { writes[id.value].linearVelocities.push_back(v); }
        void addBodyAcceleration(BodyId, const glm::vec3&) {}
        void addBodyVelocityChange(BodyId, const glm::vec3&) {}
        glm::vec3 getBodyInertiaTensor(BodyId) const { return glm::vec3(1.f); }
        PhysicsBodyState captureBodyState(BodyId) const { return PhysicsBodyState{}; }

        std::map<uint32_t, glm::mat4> current;
        std::map<uint32_t, BodyWrites> writes;
    };

    struct NoQueries
    {
        SpatialQueryReport overlap(const std::vector<QueryVolumeId>&) { return SpatialQueryReport{}; }
        SweepHit sweep(QueryVolumeId, const glm::mat4&, const glm::vec3&) { return SweepHit{}; }
        void setVolumeParentTransform(QueryVolumeId, const glm::mat4&) {}
        void enableShape(ShapeId) {}
        void disableShape(ShapeId) {}
    };

    static_assert(PhysicsBodyAdapter<RecordingPhysicsAdapter>);
    static_assert(SpatialQueryAdapter<NoQueries>);

    struct FullBodyPart { PhysicsBodyState bodyState; };
    struct LinearBodyPart { LinearBodyState bodyState; };

    struct PushState
    {
        FullBodyPart full;
        LinearBodyPart linear;

        template <typename S>
        const S& get() const
        {
            if constexpr (std::is_same_v<S, FullBodyPart>)
                return full;
            else
                return linear;
        }
    };

    struct PushAllState
    {
        PushState state;
        const PushState& getState() const { return state; }
    };

    struct Bindings { BodyId ownBodyId; };

    struct FullDeclaration
    {
        using StateType = FullBodyPart;
        static const PhysicsBodyState& bodyStateOf(const FullBodyPart& s) { return s.bodyState; }
        Bindings bindings;
    };

    struct LinearDeclaration
    {
        using StateType = LinearBodyPart;
        static const LinearBodyState& bodyStateOf(const LinearBodyPart& s) { return s.bodyState; }
        Bindings bindings;
    };

    struct PushComposite
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

    struct NoStaticData {};
    struct NoInput { int unused = 0; };

    struct PushSimulatable
    {
        using InputType = NoInput;

        void integrate(const SimulationTimeStep&, const NoInput&, RecordingPhysicsAdapter&, NoQueries&, const NoStaticData&) {}
        void firstResimStep(RecordingPhysicsAdapter&, int32_t) {}
        const PushAllState& getAllState() const { return allState; }
        const PushComposite& getPhysicsComposite() const { return composite; }

        PushAllState allState;
        PushComposite composite;
    };

    using PushExecutor = SimulationIntegrationExecutor<NoStaticData, RecordingPhysicsAdapter, NoQueries, PushSimulatable>;
    static_assert(SimulationIntegrationExecutorConcept<PushExecutor>);

    constexpr uint32_t kFullBody = 7u;
    constexpr uint32_t kLinearBody = 9u;

    constexpr glm::vec3 kFullPosition = glm::vec3(0.02f, 0.03f, -0.01f);
    constexpr glm::vec3 kFullRotationAxis = glm::vec3(-0.f, -1.f, 0.f);
    constexpr glm::vec3 kFullLinearVelocity = glm::vec3(0.05f, 0.06f, -0.04f);
    constexpr glm::vec3 kFullAngularVelocity = glm::vec3(-0.2f, -0.3f, 0.1f);
    constexpr glm::vec3 kLinearPosition = glm::vec3(0.2f, 0.3f, -0.1f);
    constexpr glm::vec3 kLinearLinearVelocity = glm::vec3(-0.02f, -0.03f, 0.01f);
    constexpr glm::vec3 kLiveRotationAxis = glm::vec3(-1.f, -0.f, 0.f);
    constexpr glm::vec3 kLiveTranslation = glm::vec3(-0.08f, -0.09f, 0.07f);
}

TEST_CASE("StepDriver.PushCorrectedBodyStates.FullWritesEverythingLinearKeepsRotationAndSpin", "[StepDriver][IntegrationExecutor]")
{
    PushSimulatable simulatable;
    simulatable.composite.full.bindings.ownBodyId = BodyId{ kFullBody };
    simulatable.composite.linear.bindings.ownBodyId = BodyId{ kLinearBody };
    simulatable.allState.state.full.bodyState = PhysicsBodyState{
        kFullPosition,
        glm::angleAxis(0.5f, kFullRotationAxis),
        kFullLinearVelocity,
        kFullAngularVelocity };
    simulatable.allState.state.linear.bodyState = LinearBodyState{ kLinearPosition, kLinearLinearVelocity };

    SimulationObjectStorage<PushSimulatable> storage;
    storage.add<PushSimulatable>(1u, std::move(simulatable));
    NoStaticData staticData;
    RecordingPhysicsAdapter physics;
    NoQueries queries;

    // The linear body's live transform carries a rotation the push must keep.
    const glm::quat liveRotation = glm::angleAxis(1.2f, kLiveRotationAxis);
    glm::mat4 liveTransform = glm::mat4_cast(liveRotation);
    liveTransform[3] = glm::vec4(kLiveTranslation, 1.f);
    physics.current[kLinearBody] = liveTransform;

    PushExecutor executor(storage, staticData, physics, queries);
    executor.pushCorrectedBodyStatesAll();

    // Full: position and rotation in one transform, plus both velocities.
    const BodyWrites& full = physics.writes.at(kFullBody);
    REQUIRE(full.transforms.size() == 1u);
    glm::mat4 expectedFull = glm::mat4_cast(glm::angleAxis(0.5f, kFullRotationAxis));
    expectedFull[3] = glm::vec4(kFullPosition, 1.f);
    CHECK(full.transforms[0] == expectedFull);
    REQUIRE(full.linearVelocities == std::vector<glm::vec3>{ kFullLinearVelocity });
    REQUIRE(full.angularVelocities == std::vector<glm::vec3>{ kFullAngularVelocity });

    // Linear: the translation is replaced, the rotation columns are the body's own, and the
    // angular velocity is never written.
    const BodyWrites& linear = physics.writes.at(kLinearBody);
    REQUIRE(linear.transforms.size() == 1u);
    glm::mat4 expectedLinear = liveTransform;
    expectedLinear[3] = glm::vec4(kLinearPosition, 1.f);
    CHECK(linear.transforms[0] == expectedLinear);
    REQUIRE(linear.linearVelocities == std::vector<glm::vec3>{ kLinearLinearVelocity });
    REQUIRE(linear.angularVelocities.empty());
}

#endif // WITH_LOW_LEVEL_TESTS
