// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsFactory.h"
#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"
#include "OGSimulationJolt/JoltStaticWorldBuilder.h"

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/SimulatableList.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationInputResolution.h"
#include "OGSimulation/SimulationIntegrationExecutor.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationNetSync.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SimulationStepDriver.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StaticGeometry.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/SystemsExecutor.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <type_traits>
#include <vector>

namespace joltSuite
{
    struct ProbeInput
    {
        int32_t value = 0;
    };
} // namespace joltSuite

template <>
struct SerializableFields<joltSuite::ProbeInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(joltSuite::ProbeInput, value));
    }
};

namespace joltSuite
{
    using namespace joltTestRig;

    inline constexpr uint32_t kSlots = 8;
    inline constexpr uint32_t kDeclarations = 6;
    inline constexpr float kQueryRadiusCm = 200.f;

    template <uint32_t Index>
    struct ProbeSpherePart
    {
        PhysicsBodyState bodyState;
    };

    struct ProbeCapsulePart
    {
        LinearBodyState bodyState;
    };

    struct ProbeState
    {
        ProbeSpherePart<0> radial;
        ProbeSpherePart<1> guard;
        ProbeSpherePart<2> projectile0;
        ProbeSpherePart<3> projectile1;
        ProbeSpherePart<4> projectile2;
        ProbeCapsulePart capsule;
        int32_t queryHits = 0;
        uint32_t integrations = 0;
        int32_t lastInput = 0;

        template <typename S>
        S& edit()
        {
            if constexpr (std::is_same_v<S, ProbeSpherePart<0>>)
                return radial;
            else if constexpr (std::is_same_v<S, ProbeSpherePart<1>>)
                return guard;
            else if constexpr (std::is_same_v<S, ProbeSpherePart<2>>)
                return projectile0;
            else if constexpr (std::is_same_v<S, ProbeSpherePart<3>>)
                return projectile1;
            else if constexpr (std::is_same_v<S, ProbeSpherePart<4>>)
                return projectile2;
            else
            {
                static_assert(std::is_same_v<S, ProbeCapsulePart>, "ProbeState has no such part");
                return capsule;
            }
        }

        template <typename S>
        const S& get() const
        {
            return const_cast<ProbeState*>(this)->template edit<S>();
        }

        bool isSimilarTo(const ProbeState& other) const
        {
            return std::memcmp(this, &other, sizeof(ProbeState)) == 0;
        }
    };

    static_assert(sizeof(ProbeState) == 5u * sizeof(PhysicsBodyState) + sizeof(LinearBodyState) + 3u * 4u,
        "ProbeState must have no padding: isSimilarTo compares its bytes");
    static_assert(std::is_trivially_copyable_v<ProbeState>);

    class ProbeAllState
    {
    public:
        const ProbeState& getState() const { return m_state; }
        ProbeState& editState() { return m_state; }

    private:
        ProbeState m_state;
    };

    struct ProbeBindings
    {
        BodyId ownBodyId;
    };

    template <typename PartT>
    struct ProbeDeclaration
    {
        using StateType = PartT;

        static const auto& bodyStateOf(const PartT& part) { return part.bodyState; }
        static auto& bodyStateOf(PartT& part) { return part.bodyState; }

        ProbeBindings bindings;
    };

    struct ProbeComposite
    {
        std::tuple<ProbeDeclaration<ProbeSpherePart<0>>, ProbeDeclaration<ProbeSpherePart<1>>, ProbeDeclaration<ProbeSpherePart<2>>,
            ProbeDeclaration<ProbeSpherePart<3>>, ProbeDeclaration<ProbeSpherePart<4>>, ProbeDeclaration<ProbeCapsulePart>>
            declarations;

        template <typename F>
        void forEach(F&& f) const
        {
            std::apply([&](const auto&... declaration) { (f(declaration), ...); }, declarations);
        }

        template <typename F>
        void forEach(F&& f)
        {
            std::apply([&](auto&... declaration) { (f(declaration), ...); }, declarations);
        }

        BodyId bodyOf(uint32_t declarationIndex) const
        {
            BodyId result;
            uint32_t index = 0;
            forEach([&](const auto& declaration) {
                if (index++ == declarationIndex)
                    result = declaration.bindings.ownBodyId;
            });
            return result;
        }

        void bind(uint32_t declarationIndex, BodyId body)
        {
            uint32_t index = 0;
            forEach([&](auto& declaration) {
                if (index++ == declarationIndex)
                    declaration.bindings.ownBodyId = body;
            });
        }
    };

    struct IntegrateRecord
    {
        uint32_t id = 0;
        SimTick tick = 0;
        StepKind kind = StepKind::Normal;
        bool resim = false;
        int32_t input = 0;
    };

    struct ProbeStaticData
    {
        std::vector<IntegrateRecord>* log = nullptr;
    };

    inline float scriptedAcceleration(uint32_t tick, uint32_t id, uint32_t axis, int32_t input)
    {
        const int32_t pattern = int32_t((tick * 7u + id * 13u + axis * 5u) % 11u) - 5;
        return float(pattern) * 300.f + float(input) * 20.f;
    }

    inline glm::vec3 homeOf(uint32_t slot)
    {
        return glm::vec3(float(slot % 4u) * 150.f - 225.f, float(slot / 4u) * 150.f - 75.f, 97.f);
    }

    class ProbeSim
    {
    public:
        using StateType = ProbeState;
        using InputType = ProbeInput;

        uint32_t id = 0;
        uint32_t slot = 0;
        QueryVolumeId volume;
        ProbeComposite composite;

        const ProbeAllState& getAllState() const { return m_all; }
        ProbeAllState& editAllState() { return m_all; }
        void updateVizState() { m_viz = m_all; }
        const ProbeAllState& getVizState() const { return m_viz; }

        const ProbeComposite& getPhysicsComposite() const { return composite; }
        ProbeComposite& editPhysicsComposite() { return composite; }

        void firstResimStep(JoltPhysicsBodyAdapter&, int32_t) {}

        void integrate(const SimulationTimeStep& step, const ProbeInput& input, JoltPhysicsBodyAdapter& physics,
            JoltSpatialQueryAdapter& query, const ProbeStaticData& staticData)
        {
            if (staticData.log != nullptr)
            {
                staticData.log->push_back(IntegrateRecord{ id, step.getTick(), step.getStepKind(), step.getIsResimulating(), input.value });
            }

            ProbeState& state = m_all.editState();
            const uint32_t tick = step.getTick();
            const glm::vec3 capsulePosition = state.capsule.bodyState.position;
            const glm::vec3 capsuleVelocity = state.capsule.bodyState.linearVelocity;

            query.setVolumeParentTransform(volume, glm::translate(glm::mat4(1.f), capsulePosition));
            state.queryHits = int32_t(query.overlap({ volume }).size());

            const glm::vec3 home = homeOf(slot);
            glm::vec3 acceleration(scriptedAcceleration(tick, id, 0, input.value), scriptedAcceleration(tick, id, 1, input.value), 0.f);
            acceleration.x += -3.f * capsuleVelocity.x - 4.f * (capsulePosition.x - home.x) + 150.f * float(state.queryHits);
            acceleration.y += -3.f * capsuleVelocity.y - 4.f * (capsulePosition.y - home.y);
            physics.addBodyAcceleration(composite.bodyOf(5), acceleration);

            glm::mat4 radialTransform = glm::mat4_cast(state.radial.bodyState.rotation);
            radialTransform[3] = glm::vec4(capsulePosition + glm::vec3(0.f, 70.f, 30.f), 1.f);
            physics.setBodyTransform(composite.bodyOf(0), radialTransform);
            const float angularAcceleration = ((tick / 20u + id) % 2u == 0u ? 1.f : -1.f) * 12.f;
            physics.addBodyTorque(composite.bodyOf(0), physics.getBodyInertiaTensor(composite.bodyOf(0)) * glm::vec3(0.2f, 0.f, 1.f) * angularAcceleration);

            const glm::vec3 guardOffset = state.guard.bodyState.position - capsulePosition;
            if (glm::dot(guardOffset, guardOffset) > 300.f * 300.f)
            {
                physics.setBodyTransform(composite.bodyOf(1), glm::translate(glm::mat4(1.f), capsulePosition + glm::vec3(0.f, -70.f, 30.f)));
                physics.setBodyLinearVelocity(composite.bodyOf(1), glm::vec3(0.f));
            }
            physics.addBodyVelocityChange(composite.bodyOf(1), glm::vec3(scriptedAcceleration(tick, id, 2, input.value) * 0.01f, 0.f, 0.f));

            const std::array<glm::vec3, 3> projectilePositions{ state.projectile0.bodyState.position, state.projectile1.bodyState.position,
                state.projectile2.bodyState.position };
            for (uint32_t projectile = 0; projectile < 3u; ++projectile)
            {
                const BodyId body = composite.bodyOf(2u + projectile);
                const glm::vec3 offset = projectilePositions[projectile] - capsulePosition;
                if (glm::dot(offset, offset) > 1500.f * 1500.f)
                {
                    physics.setBodyTransform(body, glm::translate(glm::mat4(1.f), capsulePosition + glm::vec3(60.f, 0.f, 40.f * float(projectile))));
                    physics.setBodyLinearVelocity(body, glm::vec3(200.f * float(projectile + 1u), 0.f, 0.f));
                }
                physics.addBodyAcceleration(body, glm::vec3(scriptedAcceleration(tick, id, 3u + projectile, input.value),
                    scriptedAcceleration(tick, id, 6u + projectile, input.value), 0.f));
            }

            ++state.integrations;
            state.lastInput = input.value;
        }

    private:
        ProbeAllState m_all;
        ProbeAllState m_viz;
    };

    struct ProbeStateSyncBuffer
    {
        ProbeState payload;
        uint32_t lastTick = 0;
        uint32_t lastAppliedCaptureTick = kNoInputCaptureTick;

        void write(const ProbeState&, uint32_t tick, uint32_t appliedCaptureTick)
        {
            lastTick = tick;
            lastAppliedCaptureTick = appliedCaptureTick;
        }
        void write(const ProbeState& state, uint32_t tick) { write(state, tick, kNoInputCaptureTick); }

        uint32_t readInto(ProbeState& state) const
        {
            state = payload;
            return lastTick;
        }
        uint32_t getAppliedCaptureTick() const { return lastAppliedCaptureTick; }

        template <typename T>
        T readFromBuffer(uint32_t) const { return T{}; }
        template <typename T>
        void writeToBuffer(uint32_t, T) {}
    };

    struct ProbeRelayedInputRing
    {
        std::vector<uint8_t> bytes;

        int32_t bundleByteNum() const { return static_cast<int32_t>(bytes.size()); }
        void bundleAddZeroedBytes(int32_t count) { bytes.resize(bytes.size() + static_cast<size_t>(count), 0u); }

        template <typename T>
        void writeToBuffer(uint32_t offset, const T& value) { std::memcpy(bytes.data() + offset, &value, sizeof(T)); }

        template <typename T>
        T readFromBuffer(uint32_t offset) const
        {
            T value;
            std::memcpy(&value, bytes.data() + offset, sizeof(T));
            return value;
        }
    };

    struct ProbeOwner
    {
        using SyncedCorrectionBufferType = ProbeStateSyncBuffer;
        using RelayedInputRingType = ProbeRelayedInputRing;

        std::function<void(const ProbeStateSyncBuffer&)> onCorrectionStateReceived;
        std::function<void(const ProbeRelayedInputRing&)> onRelayedInputReceived;
        ProbeRelayedInputRing relayedInputRing;
        ProbeStateSyncBuffer correctionStateBuffer;
        std::function<void(uint32_t, const ProbeInput&)> onRemoteMoveReceived;

        void setOnCorrectionStateReceivedCallback(std::function<void(const ProbeStateSyncBuffer&)> fn) { onCorrectionStateReceived = std::move(fn); }
        void clearOnCorrectionStateReceivedCallback() { onCorrectionStateReceived = nullptr; }
        void setOnRelayedInputReceivedCallback(std::function<void(const ProbeRelayedInputRing&)> fn) { onRelayedInputReceived = std::move(fn); }
        void clearOnRelayedInputReceivedCallback() { onRelayedInputReceived = nullptr; }
        const ProbeRelayedInputRing& getRelayedInputRing() const { return relayedInputRing; }
        void sendLocalInputToAuthority(const PendingInputQueue<ProbeInput>&, uint32_t, uint32_t) {}
        ProbeStateSyncBuffer& getSyncedCorrectionStateBuffer() { return correctionStateBuffer; }
        void setOnRemoteMoveReceivedCallback(std::function<void(uint32_t, const ProbeInput&)> fn) { onRemoteMoveReceived = std::move(fn); }
        void clearOnRemoteMoveReceivedCallback() { onRemoteMoveReceived = nullptr; }
    };
} // namespace joltSuite

template <>
struct SimulatableOwnerTraits<joltSuite::ProbeSim>
{
    using PredictionOwnerType = joltSuite::ProbeOwner;
    using AuthorityOwnerType = joltSuite::ProbeOwner;
};

namespace joltSuite
{
    static_assert(SimulatableState<ProbeSim>);
    static_assert(PredictionSyncedBufferOwnerConcept<ProbeOwner, ProbeState, ProbeInput>);

    using ProbeStorage = SimulationObjectStorage<ProbeSim>;
    using ProbeReconciliation = SimulationReconciliation<ProbeSim>;
    using ProbeResolution = SimulationInputResolution<ProbeSim>;
    using ProbeNetSync = SimulationNetSync<ProbeSim>;
    using ProbeExecutor = SimulationIntegrationExecutor<ProbeStaticData, JoltPhysicsBodyAdapter, JoltSpatialQueryAdapter, ProbeSim>;
    using ProbeSystems = NullSystemsExecutor<SimulatableList<ProbeSim>, ProbeStaticData>;
    using ProbeManager = SimulationManager<ProbeExecutor, ProbeNetSync, ProbeResolution, ProbeReconciliation, ProbeSystems, ProbeStorage, ProbeStaticData>;

    inline ProbeInput scriptedInput(const SimulationTimeStep& step, uint32_t id)
    {
        return ProbeInput{ int32_t((step.getTick() * 31u + id * 7u) % 17u) - 8 };
    }

    inline StaticShapeDescriptor staticBox(const glm::vec3& centreCm, const glm::vec3& halfExtentsCm, uint64_t stableKey)
    {
        StaticShapeDescriptor descriptor;
        descriptor.shape = StaticBox{ halfExtentsCm };
        descriptor.localToWorld = glm::translate(glm::mat4(1.f), centreCm);
        descriptor.categories = CollisionCategories{ bit(kWorld) };
        descriptor.blockingCategories = CollisionCategories{ kAllMappedCategories };
        descriptor.friction = 0.7f;
        descriptor.restitution = 0.3f;
        descriptor.stableKey = stableKey;
        return descriptor;
    }

    inline StaticWorldDescription arenaWithExtraStatics(uint32_t extraStatics)
    {
        StaticWorldDescription description;
        description.shapes.push_back(staticBox(glm::vec3(0.f, 0.f, -50.f), glm::vec3(1500.f, 1500.f, 50.f), 1u));
        for (uint32_t index = 0; index < extraStatics; ++index)
        {
            const float angle = float(index) * 0.7f;
            const float radius = 800.f + 20.f * float(index % 13u);
            description.shapes.push_back(staticBox(glm::vec3(radius * std::cos(angle), radius * std::sin(angle), 40.f + float(index % 5u) * 30.f),
                glm::vec3(20.f + float(index % 3u) * 5.f, 20.f, 40.f), 1000u + index));
        }
        return description;
    }

    struct SceneConfig
    {
        uint32_t ringDepthTicks = 22;
        uint32_t extraStatics = 0;
        uint32_t maxShapesPerChunk = 256;
        JoltPhysicsFactoryOptions factoryOptions;
    };

    struct JoltScene
    {
        JoltScene(JoltRuntime& runtime, const SceneConfig& sceneConfig)
            : accessScope()
            , description(arenaWithExtraStatics(sceneConfig.extraStatics))
            , world(runtime, worldConfigOf(sceneConfig, description), &logJoltProblems)
            , bodies(world)
            , query(world, &logJoltProblems, JoltSpatialQueryConfig{ kAllMappedCategories })
            , factoryOptions(sceneConfig.factoryOptions)
        {
            JoltStaticWorldBuilderOptions options;
            options.maxShapesPerChunk = sceneConfig.maxShapesPerChunk;
            JoltStaticWorldBuilder builder(world, [](const char*) {}, options);
            builder.build(description);
            staticBodies = builder.stats().bodies;
        }

        static JoltWorldConfig worldConfigOf(const SceneConfig& sceneConfig, const StaticWorldDescription& description)
        {
            JoltWorldConfig config;
            config.simulatableSlots = kSlots;
            config.slotTemplate = brawlerSlotTemplate();
            config.staticLayers = staticLayerKeysOf(description);
            config.ringDepthTicks = sceneConfig.ringDepthTicks;
            config.tempAllocatorBytes = 8u << 20;
            return config;
        }

        ProbeSim bindProbe(uint32_t slot, uint32_t id)
        {
            ProbeSim sim;
            sim.id = id;
            sim.slot = slot;
            JoltPhysicsFactory factory(bodies, brawlerSlotTemplate(), slot, id, factoryOptions,
                [this](BodyId body, uint32_t shapeIndex, std::optional<BodyId> root) { return query.registerShape(body, shapeIndex, root); });
            uint32_t declarationIndex = 0;
            for (const SlotBodyTemplate& slotTemplate : brawlerSlotTemplate())
            {
                sim.composite.bind(declarationIndex++, factory.createPhysicalObject(slotTemplate.descriptor, "probe").bodyId);
            }
            sim.volume = query.registerVolume(
                QueryVolumeDescriptor{ SphereGeometry{ kQueryRadiusCm }, CollisionCategories{ bit(kCharacter) }, glm::mat4(1.f), kQueryRouting },
                factory.parentBodyId());

            const glm::vec3 home = homeOf(slot);
            bodies.setBodyTransform(sim.composite.bodyOf(5), glm::translate(glm::mat4(1.f), home));
            for (uint32_t declaration = 0; declaration < 5u; ++declaration)
            {
                bodies.setBodyTransform(sim.composite.bodyOf(declaration),
                    glm::translate(glm::mat4(1.f), home + glm::vec3(60.f, 10.f * float(declaration), 30.f)));
                bodies.setBodyLinearVelocity(sim.composite.bodyOf(declaration), glm::vec3(0.f));
                bodies.setBodyAngularVelocity(sim.composite.bodyOf(declaration), glm::vec3(0.f));
            }
            bodies.setBodyLinearVelocity(sim.composite.bodyOf(5), glm::vec3(0.f));

            ProbeState& state = sim.editAllState().editState();
            sim.composite.forEach([&](const auto& declaration) {
                using D = std::decay_t<decltype(declaration)>;
                D::bodyStateOf(state.template edit<typename D::StateType>()) = bodies.captureBodyState(declaration.bindings.ownBodyId);
            });
            return sim;
        }

        JoltWorldAccessScope accessScope;
        StaticWorldDescription description;
        JoltWorld world;
        JoltPhysicsBodyAdapter bodies;
        JoltSpatialQueryAdapter query;
        JoltPhysicsFactoryOptions factoryOptions;
        std::vector<JPH::BodyID> staticBodies;
    };

    struct BodyBits
    {
        std::array<uint32_t, 13> words{};

        bool operator==(const BodyBits&) const = default;
    };

    inline BodyBits bodyBitsOf(const JoltWorld& world, JPH::BodyID id)
    {
        const JPH::BodyInterface& bodies = world.bodies();
        const JPH::RVec3 position = bodies.GetCenterOfMassPosition(id);
        const JPH::Quat rotation = bodies.GetRotation(id);
        const JPH::Vec3 linear = bodies.GetLinearVelocity(id);
        const JPH::Vec3 angular = bodies.GetAngularVelocity(id);
        const std::array<float, 13> values{ float(position.GetX()), float(position.GetY()), float(position.GetZ()), rotation.GetX(),
            rotation.GetY(), rotation.GetZ(), rotation.GetW(), linear.GetX(), linear.GetY(), linear.GetZ(), angular.GetX(), angular.GetY(),
            angular.GetZ() };
        BodyBits bits;
        for (size_t index = 0; index < values.size(); ++index)
        {
            bits.words[index] = std::bit_cast<uint32_t>(values[index]);
        }
        return bits;
    }

    class ObservedJoltWorld
    {
    public:
        static constexpr SnapshotCoverage coverage = JoltWorld::coverage;

        explicit ObservedJoltWorld(JoltWorld& world) : m_world(world) {}

        void step(float dt)
        {
            if (beforeStep)
                beforeStep(lastRestoredTick, stepsSinceRestore);
            m_world.step(dt);
            ++steps;
            if (afterStep)
                afterStep(lastRestoredTick, stepsSinceRestore);
            ++stepsSinceRestore;
        }

        void saveTick(SimTick tick)
        {
            m_world.saveTick(tick);
            savedTicks.push_back(tick);
        }

        bool restoreTick(SimTick tick)
        {
            const bool restored = m_world.restoreTick(tick);
            ++restores;
            lastRestoredTick = tick;
            stepsSinceRestore = 0;
            if (!restored)
                ++failedRestores;
            return restored;
        }

        bool hasTick(SimTick tick) const { return m_world.hasTick(tick); }
        std::optional<SimTick> oldestHeldTick() const { return m_world.oldestHeldTick(); }
        void invalidateAllTicks() { m_world.invalidateAllTicks(); }
        void saveScratch() { m_world.saveScratch(); }
        void commitScratch(SimTick tick) { m_world.commitScratch(tick); }
        void applyOccupancy(const BodySlotOccupancy& occupancy) { m_world.applyOccupancy(occupancy); }
        std::optional<uint64_t> stateHash(SimTick tick) const { return m_world.stateHash(tick); }

        std::function<void(std::optional<SimTick>, uint32_t)> beforeStep;
        std::function<void(std::optional<SimTick>, uint32_t)> afterStep;
        std::optional<SimTick> lastRestoredTick;
        uint32_t stepsSinceRestore = 0;
        uint64_t steps = 0;
        uint64_t restores = 0;
        uint64_t failedRestores = 0;
        std::vector<SimTick> savedTicks;

    private:
        JoltWorld& m_world;
    };

    static_assert(PhysicsWorldAdapter<ObservedJoltWorld>);

    struct PushCheck
    {
        uint64_t pushes = 0;
        uint64_t declarationsChecked = 0;
        uint64_t declarationsExact = 0;
        uint64_t mismatches = 0;
        uint64_t declarationsMovedByPush = 0;
        float maxPositionErrorCm = 0.f;
        float maxVelocityErrorCmPerS = 0.f;
        float maxRotationError = 0.f;
        float maxAngularVelocityError = 0.f;
    };

    inline float maxAbs(const glm::vec3& value)
    {
        return std::max({ std::fabs(value.x), std::fabs(value.y), std::fabs(value.z) });
    }

    inline void checkPushedBodies(const ProbeStorage& storage, const JoltPhysicsBodyAdapter& bodies, PushCheck& check)
    {
        ++check.pushes;
        storage.forEachSimulatable([&](unsigned int, const auto& sim) {
            const ProbeState& state = sim.getAllState().getState();
            sim.getPhysicsComposite().forEach([&](const auto& declaration) {
                using D = std::decay_t<decltype(declaration)>;
                using BodyStateT = std::remove_cvref_t<decltype(D::bodyStateOf(state.template get<typename D::StateType>()))>;
                const PhysicsBodyState pushed = static_cast<PhysicsBodyState>(D::bodyStateOf(state.template get<typename D::StateType>()));
                const PhysicsBodyState live = bodies.captureBodyState(declaration.bindings.ownBodyId);
                const float positionError = maxAbs(live.position - pushed.position);
                const float velocityError = maxAbs(live.linearVelocity - pushed.linearVelocity);
                float rotationError = 0.f;
                float angularError = 0.f;
                if constexpr (std::is_same_v<BodyStateT, PhysicsBodyState>)
                {
                    const float dot = std::fabs(glm::dot(live.rotation, pushed.rotation));
                    rotationError = 1.f - std::min(dot, 1.f);
                    angularError = maxAbs(live.angularVelocity - pushed.angularVelocity);
                }
                ++check.declarationsChecked;
                const bool exact = positionError == 0.f && velocityError == 0.f && rotationError == 0.f && angularError == 0.f;
                if (exact)
                    ++check.declarationsExact;
                const float positionTolerance = 1e-5f * std::max(1.f, maxAbs(pushed.position));
                const float velocityTolerance = 1e-5f * std::max(1.f, maxAbs(pushed.linearVelocity));
                if (positionError > positionTolerance || velocityError > velocityTolerance || rotationError > 1e-6f || angularError != 0.f)
                    ++check.mismatches;
                check.maxPositionErrorCm = std::max(check.maxPositionErrorCm, positionError);
                check.maxVelocityErrorCmPerS = std::max(check.maxVelocityErrorCmPerS, velocityError);
                check.maxRotationError = std::max(check.maxRotationError, rotationError);
                check.maxAngularVelocityError = std::max(check.maxAngularVelocityError, angularError);
            });
        });
    }

    class PushProbeExecutor
    {
    public:
        PushProbeExecutor(ProbeExecutor& executor, ProbeStorage& storage, JoltPhysicsBodyAdapter& bodies)
            : m_executor(executor), m_storage(storage), m_bodies(bodies)
        {
        }

        void firstResimStepAll(int32_t physicsStep) { m_executor.firstResimStepAll(physicsStep); }
        void captureBodyStatesAll() { m_executor.captureBodyStatesAll(); }
        void pushCorrectedBodyStatesAll()
        {
            std::vector<PhysicsBodyState> before;
            m_storage.forEachSimulatable([&](unsigned int, const auto& sim) {
                sim.getPhysicsComposite().forEach([&](const auto& declaration) { before.push_back(m_bodies.captureBodyState(declaration.bindings.ownBodyId)); });
            });
            if (!skipPush)
                m_executor.pushCorrectedBodyStatesAll();
            size_t index = 0;
            m_storage.forEachSimulatable([&](unsigned int, const auto& sim) {
                sim.getPhysicsComposite().forEach([&](const auto& declaration) {
                    const PhysicsBodyState after = m_bodies.captureBodyState(declaration.bindings.ownBodyId);
                    if (std::memcmp(&after, &before[index++], sizeof(PhysicsBodyState)) != 0)
                        ++check.declarationsMovedByPush;
                });
            });
            checkPushedBodies(m_storage, m_bodies, check);
        }

        PushCheck check;
        bool skipPush = false;

    private:
        ProbeExecutor& m_executor;
        ProbeStorage& m_storage;
        JoltPhysicsBodyAdapter& m_bodies;
    };

    static_assert(SimulationIntegrationExecutorConcept<PushProbeExecutor>);

    struct ProbeHooks
    {
        void beforeTick(const UpcomingTick&) {}
        void beforeSimulate(const UpcomingTick&) {}
        void beforePhysics(SimTick) {}
        void afterTick(const TickOutcome& outcome) { outcomes.push_back(outcome); }

        std::vector<TickOutcome> outcomes;
    };

    struct DriverRigConfig
    {
        SceneConfig scene;
        ResimPolicy policy = ResimPolicy::Always;
        uint32_t depthTicks = 8;
        int32_t inputDelayTicks = 2;
        TimeConfig::ResimTriggerPolicy triggerPolicy = TimeConfig::ResimTriggerPolicy::OnDisagreement;
    };

    using ProbeDriver = SimulationStepDriver<ProbeManager, ObservedJoltWorld, PushProbeExecutor, ProbeHooks>;

    struct DriverRig
    {
        DriverRig(JoltRuntime& runtime, const DriverRigConfig& rigConfig)
            : config(rigConfig)
            , scene(runtime, rigConfig.scene)
            , observed(scene.world)
            , staticData{ &log }
            , reconciliation(storage)
            , resolution(storage, reconciliation)
            , netSync(storage, reconciliation, resolution)
            , executor(storage, staticData, scene.bodies, scene.query)
            , pushProbe(executor, storage, scene.bodies)
            , manager(true, double(kDt), ProbeManager::Params{ executor, netSync, resolution, reconciliation, systems, storage, staticData, nullptr })
            , driver(manager, observed, pushProbe, hooks, StepDriverConfig{ kDt, rigConfig.policy, rigConfig.depthTicks })
        {
            resolution.setNeutralInput<ProbeSim>(ProbeInput{ 0 });
            resolution.setClientEffectiveInputDelayTicks(rigConfig.inputDelayTicks);
            manager.setResimTriggerPolicy(rigConfig.triggerPolicy);
            reconciliation.setLogger([this](const char* line) {
                if (std::strstr(line, "[ResimCheck.Check]") != nullptr && std::strstr(line, "needsResim=1") != nullptr
                    && std::strstr(line, "withinDepth=0") != nullptr)
                    ++deepAnchorSkips;
            });
        }

        void join(uint32_t slot, uint32_t id)
        {
            ProbeSim sim = scene.bindProbe(slot, id);
            registerSimulatable<ProbeSim>(storage, reconciliation, resolution, netSync, id, std::move(sim), owners[slot],
                [id](const SimulationTimeStep& step, const LocalInputCache<ProbeInput>&) { return scriptedInput(step, id); });
            manager.notifyCharacterRegistered(id);
            driver.noteOccupancy(slot, true);
            idOfSlot[slot] = id;
        }

        void leave(uint32_t slot)
        {
            const uint32_t id = idOfSlot[slot];
            manager.notifyCharacterUnregistered(id);
            unregisterSimulatable<ProbeSim>(storage, reconciliation, resolution, netSync, id, &owners[slot]);
            driver.noteOccupancy(slot, false);
            scene.bodies.bindTable().releaseSlot(slot);
            idOfSlot[slot] = 0;
        }

        void injectCorrection(uint32_t slot, SimTick tick, const ProbeState& corrected)
        {
            ProbeStateSyncBuffer buffer;
            buffer.payload = corrected;
            buffer.lastTick = tick;
            owners[slot].onCorrectionStateReceived(buffer);
        }

        void holdDrift(int32_t drift)
        {
            NetworkTimeEstimator& estimator = manager.editNetworkEstimator();
            const int64_t target = static_cast<int64_t>(present()) + drift;
            estimator.recordAuthorityTick(static_cast<unsigned int>(target - estimator.getPredictionOffsetTicks()));
        }

        SimTick present() const { return manager.getClientClock().getPredictionTick(); }

        TickOutcome tick(int32_t drift = 1)
        {
            holdDrift(drift);
            return driver.runTick(++physicsStep);
        }

        const ProbeSim* simOf(uint32_t id) const
        {
            return storage.has<ProbeSim>(id) ? &storage.get<ProbeSim>(id) : nullptr;
        }

        DriverRigConfig config;
        JoltScene scene;
        ObservedJoltWorld observed;
        std::vector<IntegrateRecord> log;
        ProbeStaticData staticData;
        ProbeStorage storage;
        ProbeReconciliation reconciliation;
        ProbeResolution resolution;
        ProbeNetSync netSync;
        ProbeExecutor executor;
        PushProbeExecutor pushProbe;
        ProbeSystems systems;
        ProbeManager manager;
        ProbeHooks hooks;
        ProbeDriver driver;
        std::array<ProbeOwner, kSlots> owners;
        std::array<uint32_t, kSlots> idOfSlot{};
        uint64_t physicsStep = 0;
        uint64_t deepAnchorSkips = 0;
    };

    struct ReferenceTick
    {
        bool resimGranted = false;
        bool resimRefused = false;
        uint32_t replayedTicks = 0;
    };

    struct ReferenceRig
    {
        ReferenceRig(JoltRuntime& runtime, const DriverRigConfig& rigConfig)
            : config(rigConfig)
            , scene(runtime, rigConfig.scene)
            , executor(storage, staticData, scene.bodies, scene.query)
        {
            scene.world.applyOccupancy(live);
            applied = live;
        }

        void join(uint32_t slot, uint32_t id)
        {
            storage.add<ProbeSim>(id, scene.bindProbe(slot, id));
            live.occupied.set(slot);
            applyOccupancy(live);
            idOfSlot[slot] = id;
        }

        void leave(uint32_t slot)
        {
            const uint32_t id = idOfSlot[slot];
            storage.remove<ProbeSim>(id);
            live.occupied.reset(slot);
            for (auto& [tick, occupancy] : timeline)
                occupancy.occupied.reset(slot);
            applyOccupancy(live);
            scene.bodies.bindTable().releaseSlot(slot);
            idOfSlot[slot] = 0;
        }

        void injectCorrection(uint32_t slot, SimTick tick, const ProbeState& corrected)
        {
            const auto it = states.find(tick);
            if (it != states.end() && it->second.count(idOfSlot[slot]) != 0u)
                it->second[idOfSlot[slot]] = corrected;
        }

        void applyOccupancy(const BodySlotOccupancy& occupancy)
        {
            scene.world.applyOccupancy(occupancy);
            applied = occupancy;
        }

        BodySlotOccupancy occupancyAt(SimTick tick) const
        {
            const auto it = timeline.find(tick);
            return it != timeline.end() ? it->second : live;
        }

        void record(SimTick tick)
        {
            timeline[tick] = applied;
            const std::optional<SimTick> oldest = scene.world.oldestHeldTick();
            if (!oldest.has_value())
            {
                timeline.clear();
                return;
            }
            timeline.erase(timeline.begin(), timeline.lower_bound(*oldest));
        }

        void feed(const std::vector<IntegrateRecord>& records)
        {
            for (const IntegrateRecord& record : records)
                pending[record.id].push_back(record);
        }

        void integrateAll(SimTick tick, StepKind kind, bool resim, const std::set<uint32_t>* only = nullptr)
        {
            ProbeResolvedInputs inputs;
            auto& map = std::get<0>(inputs);
            storage.forEachSimulatable([&](unsigned int id, auto&) {
                if (only != nullptr && only->count(id) == 0u)
                    return;
                std::deque<IntegrateRecord>& queue = pending[id];
                const bool expected = !queue.empty() && queue.front().tick == tick && queue.front().kind == kind && queue.front().resim == resim;
                if (!expected)
                {
                    ++recordMismatches;
                    map[id] = ProbeInput{ 0 };
                    return;
                }
                map[id] = ProbeInput{ queue.front().input };
                queue.pop_front();
            });
            executor.integrateAll(SimulationTimeStep(tick, resim, kind, kDt), inputs);
        }

        void storeStates(SimTick tick, bool onlyExistingSlots)
        {
            storage.forEachSimulatable([&](unsigned int id, auto& sim) {
                std::map<uint32_t, ProbeState>& slot = states[tick];
                if (onlyExistingSlots && slot.count(id) == 0u)
                    return;
                slot[id] = sim.getAllState().getState();
            });
        }

        std::optional<SimTick> anchorFor(std::optional<SimTick> requestedAnchor) const
        {
            if (config.policy == ResimPolicy::Always)
                return present > config.depthTicks ? std::optional<SimTick>(present - config.depthTicks) : std::nullopt;
            if (requestedAnchor.has_value() && *requestedAnchor != 0u && *requestedAnchor < present)
                return requestedAnchor;
            return std::nullopt;
        }

        ReferenceTick tick(StepKind kind, SimTick expectedTick, bool hardResync, std::optional<SimTick> requestedAnchor = std::nullopt)
        {
            ReferenceTick result;
            if (const std::optional<SimTick> requested = anchorFor(requestedAnchor); requested.has_value())
            {
                const SimTick anchor = *requested;
                if (!scene.world.hasTick(anchor))
                {
                    result.resimRefused = true;
                }
                else
                {
                    result.resimGranted = true;
                    const bool restored = scene.world.restoreTick(anchor);
                    if (!restored)
                        ++failedRestores;
                    applyOccupancy(occupancyAt(anchor));
                    const auto anchorStates = states.find(anchor);
                    storage.forEachSimulatable([&](unsigned int id, auto& sim) {
                        if (anchorStates == states.end())
                            return;
                        const auto it = anchorStates->second.find(id);
                        if (it != anchorStates->second.end())
                            sim.editAllState().editState() = it->second;
                    });
                    executor.pushCorrectedBodyStatesAll();
                    for (SimTick replayTick = anchor + 1u; replayTick <= present; ++replayTick)
                    {
                        const BodySlotOccupancy replayOccupancy = occupancyAt(replayTick);
                        applyOccupancy(replayOccupancy);
                        std::set<uint32_t> withSlot;
                        if (const auto it = states.find(replayTick); it != states.end())
                        {
                            for (const auto& [id, state] : it->second)
                                withSlot.insert(id);
                        }
                        integrateAll(replayTick, StepKind::Normal, true, &withSlot);
                        scene.world.step(kDt);
                        executor.captureBodyStatesAll();
                        scene.world.saveTick(replayTick);
                        timeline[replayTick] = replayOccupancy;
                        const std::optional<SimTick> oldest = scene.world.oldestHeldTick();
                        if (oldest.has_value())
                            timeline.erase(timeline.begin(), timeline.lower_bound(*oldest));
                        storeStates(replayTick, true);
                        ++result.replayedTicks;
                    }
                    storage.forEachSimulatable([&](unsigned int id, auto& sim) {
                        const auto it = states.find(present);
                        if (it == states.end())
                            return;
                        const auto found = it->second.find(id);
                        if (found != it->second.end())
                            sim.editAllState().editState() = found->second;
                    });
                }
            }

            if (applied != live)
                applyOccupancy(live);
            scene.world.saveScratch();

            if (hardResync)
            {
                scene.world.invalidateAllTicks();
                timeline.clear();
                states.clear();
            }

            const SimTick tick = expectedTick;
            if (kind == StepKind::Skip)
            {
                storeStates(tick - 1u, false);
                scene.world.commitScratch(tick - 1u);
                record(tick - 1u);
            }
            integrateAll(tick, kind, false);
            scene.world.step(kDt);
            executor.captureBodyStatesAll();
            if (stepAllocatesFrontierSlot(kind))
            {
                scene.world.saveTick(tick);
                record(tick);
                storeStates(tick, false);
            }
            present = tick;
            return result;
        }

        using ProbeResolvedInputs = ResolvedInputs<ProbeSim>;

        DriverRigConfig config;
        JoltScene scene;
        ProbeStaticData staticData;
        ProbeStorage storage;
        ProbeExecutor executor;
        BodySlotOccupancy live;
        BodySlotOccupancy applied;
        std::map<SimTick, BodySlotOccupancy> timeline;
        std::map<SimTick, std::map<uint32_t, ProbeState>> states;
        std::map<uint32_t, std::deque<IntegrateRecord>> pending;
        std::array<uint32_t, kSlots> idOfSlot{};
        SimTick present = 0;
        uint64_t recordMismatches = 0;
        uint64_t failedRestores = 0;
    };

    struct Comparison
    {
        bool bytesEqual = false;
        bool liveHashEqual = false;
        bool ringHashEqual = false;
        bool simStatesEqual = false;
        size_t stateBytes = 0;

        bool allEqual() const { return bytesEqual && liveHashEqual && ringHashEqual && simStatesEqual; }
    };

    inline bool sameSimStates(const ProbeStorage& first, const ProbeStorage& second)
    {
        bool same = true;
        size_t firstCount = 0;
        size_t secondCount = 0;
        first.forEachSimulatable([&](unsigned int, const auto&) { ++firstCount; });
        second.forEachSimulatable([&](unsigned int, const auto&) { ++secondCount; });
        if (firstCount != secondCount)
            return false;
        first.forEachSimulatable([&](unsigned int id, const auto& sim) {
            if (!second.has<ProbeSim>(id) || !sim.getAllState().getState().isSimilarTo(second.get<ProbeSim>(id).getAllState().getState()))
                same = false;
        });
        return same;
    }

    inline Comparison compare(DriverRig& run, ReferenceRig& reference, SimTick tick, bool tickSaved)
    {
        Comparison comparison;
        const std::vector<uint8_t> runBytes = fullState(run.scene.world);
        comparison.stateBytes = runBytes.size();
        comparison.bytesEqual = sameBytes(runBytes, fullState(reference.scene.world));
        comparison.liveHashEqual = run.scene.world.liveStateHash() == reference.scene.world.liveStateHash();
        const std::optional<uint64_t> runHash = run.scene.world.stateHash(tick);
        const std::optional<uint64_t> referenceHash = reference.scene.world.stateHash(tick);
        comparison.ringHashEqual = tickSaved ? (runHash.has_value() && runHash == referenceHash) : (runHash == referenceHash);
        comparison.simStatesEqual = sameSimStates(run.storage, reference.storage);
        return comparison;
    }

    inline std::string describe(const PushCheck& check)
    {
        char line[512];
        std::snprintf(line, sizeof(line),
            "pushes=%llu declarations=%llu exact=%llu mismatches=%llu movedByPush=%llu maxPosErrCm=%.3g maxVelErrCmPerS=%.3g maxRotErr=%.3g maxAngVelErr=%.3g",
            (unsigned long long)check.pushes, (unsigned long long)check.declarationsChecked, (unsigned long long)check.declarationsExact,
            (unsigned long long)check.mismatches, (unsigned long long)check.declarationsMovedByPush, double(check.maxPositionErrorCm), double(check.maxVelocityErrorCmPerS),
            double(check.maxRotationError), double(check.maxAngularVelocityError));
        return line;
    }
    inline std::string describe(const Comparison& comparison, const TickOutcome& outcome)
    {
        char line[256];
        std::snprintf(line, sizeof(line), "tick=%u kind=%d hardResync=%d replayed=%u refused=%d bytes=%d liveHash=%d ringHash=%d simStates=%d",
            outcome.tick, int(outcome.kind), outcome.hardResync ? 1 : 0, outcome.replayedTicks, outcome.resimRefused ? 1 : 0,
            comparison.bytesEqual ? 1 : 0, comparison.liveHashEqual ? 1 : 0, comparison.ringHashEqual ? 1 : 0, comparison.simStatesEqual ? 1 : 0);
        return line;
    }

    struct Totals
    {
        uint64_t ticks = 0;
        uint64_t granted = 0;
        uint64_t refused = 0;
        uint64_t hardResyncs = 0;
        uint64_t hashComparisons = 0;
        uint64_t byteComparisons = 0;
        std::map<StepKind, uint64_t> kinds;
        std::vector<SimTick> refusedAtPresent;
        uint32_t maxSavedTickBytes = 0;
        uint64_t savedTickBytesSum = 0;
        uint64_t savedTickSamples = 0;
        double runSeconds = 0.0;
    };

    struct Lockstep
    {
        DriverRig& run;
        ReferenceRig& reference;
        Totals totals;
        std::optional<SimTick> nextAnchor;

        void inject(uint32_t slot, SimTick tick, const ProbeState& corrected, bool referenceResims = true)
        {
            run.injectCorrection(slot, tick, corrected);
            reference.injectCorrection(slot, tick, corrected);
            if (referenceResims)
                nextAnchor = nextAnchor.has_value() ? std::min(*nextAnchor, tick) : tick;
        }

        TickOutcome tick(int32_t drift = 1)
        {
            const SimTick presentBefore = run.present();
            const size_t logBefore = run.log.size();
            const auto start = std::chrono::steady_clock::now();
            const TickOutcome outcome = run.tick(drift);
            totals.runSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            reference.feed(std::vector<IntegrateRecord>(run.log.begin() + std::ptrdiff_t(logBefore), run.log.end()));
            const ReferenceTick referenceTick = reference.tick(outcome.kind, outcome.tick, outcome.hardResync, nextAnchor);
            nextAnchor.reset();
            ++totals.ticks;
            ++totals.kinds[outcome.kind];
            if (outcome.replayedTicks > 0u)
                ++totals.granted;
            if (outcome.resimRefused)
            {
                ++totals.refused;
                totals.refusedAtPresent.push_back(presentBefore);
            }
            if (outcome.hardResync)
                ++totals.hardResyncs;

            INFO("run: replayed=" << outcome.replayedTicks << " refused=" << outcome.resimRefused << " / reference: replayed="
                                  << referenceTick.replayedTicks << " refused=" << referenceTick.resimRefused);
            REQUIRE(referenceTick.replayedTicks == outcome.replayedTicks);
            REQUIRE(referenceTick.resimRefused == outcome.resimRefused);
            REQUIRE(reference.recordMismatches == 0u);

            const bool saved = stepAllocatesFrontierSlot(outcome.kind);
            const Comparison comparison = compare(run, reference, outcome.tick, saved);
            INFO(describe(comparison, outcome));
            REQUIRE(comparison.ringHashEqual);
            REQUIRE(comparison.liveHashEqual);
            REQUIRE(comparison.bytesEqual);
            REQUIRE(comparison.simStatesEqual);
            ++totals.hashComparisons;
            ++totals.byteComparisons;

            if (saved)
            {
                const JoltStateSlot* slot = run.scene.world.ring().find(outcome.tick);
                REQUIRE(slot != nullptr);
                totals.maxSavedTickBytes = std::max(totals.maxSavedTickBytes, slot->byteCount);
                totals.savedTickBytesSum += slot->byteCount;
                ++totals.savedTickSamples;
            }
            return outcome;
        }

        void ticks(uint32_t count, int32_t drift = 1)
        {
            for (uint32_t index = 0; index < count; ++index)
                tick(drift);
        }
    };

    inline ProbeState perturbed(ProbeState state)
    {
        state.capsule.bodyState.position.x += 5.f;
        state.capsule.bodyState.linearVelocity.y -= 40.f;
        state.radial.bodyState.rotation = glm::normalize(glm::angleAxis(0.3f, glm::vec3(0.f, 0.f, 1.f)) * state.radial.bodyState.rotation);
        state.radial.bodyState.angularVelocity += glm::vec3(0.f, 0.f, 1.f);
        state.guard.bodyState.linearVelocity += glm::vec3(10.f, 0.f, 0.f);
        state.queryHits += 1;
        return state;
    }

    inline void printSummary(const char* name, const Totals& totals, const DriverRig& run)
    {
        std::printf("[JoltSuite] %s: ticks=%llu normal=%llu stall=%llu skip=%llu hardResync=%llu granted=%llu refused=%llu "
                    "replayedTicks=%llu restores=%llu failedRestores=%llu hashComparisons=%llu byteComparisons=%llu "
                    "savedTickBytes(max=%u mean=%.0f) runMsPerTick=%.3f\n",
            name, (unsigned long long)totals.ticks, (unsigned long long)(totals.kinds.count(StepKind::Normal) ? totals.kinds.at(StepKind::Normal) : 0),
            (unsigned long long)(totals.kinds.count(StepKind::Stall) ? totals.kinds.at(StepKind::Stall) : 0),
            (unsigned long long)(totals.kinds.count(StepKind::Skip) ? totals.kinds.at(StepKind::Skip) : 0), (unsigned long long)totals.hardResyncs,
            (unsigned long long)totals.granted, (unsigned long long)totals.refused,
            (unsigned long long)run.driver.getDiagnostics().replayedTicks(), (unsigned long long)run.observed.restores,
            (unsigned long long)run.observed.failedRestores, (unsigned long long)totals.hashComparisons, (unsigned long long)totals.byteComparisons,
            totals.maxSavedTickBytes, totals.savedTickSamples ? double(totals.savedTickBytesSum) / double(totals.savedTickSamples) : 0.0,
            totals.ticks ? 1000.0 * totals.runSeconds / double(totals.ticks) : 0.0);
        std::printf("[JoltSuite] %s: push %s\n", name, joltSuite::describe(run.pushProbe.check).c_str());
    }

    inline uint64_t countOf(const Totals& totals, StepKind kind)
    {
        return totals.kinds.count(kind) ? totals.kinds.at(kind) : 0u;
    }
} // namespace joltSuite
