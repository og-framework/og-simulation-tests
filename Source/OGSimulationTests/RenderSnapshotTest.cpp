// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StepHooks.h"

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{
    struct FullBodyPart { PhysicsBodyState bodyState; };
    struct LinearBodyPart { LinearBodyState bodyState; };

    struct TwoBodyState
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

    struct OneBodyState
    {
        LinearBodyPart linear;

        template <typename S>
        const S& get() const
        {
            static_assert(std::is_same_v<S, LinearBodyPart>);
            return linear;
        }
    };

    template <typename StateT>
    struct AllStateOf
    {
        StateT state;
        const StateT& getState() const { return state; }
    };

    struct FullDeclaration
    {
        using StateType = FullBodyPart;
        static const PhysicsBodyState& bodyStateOf(const FullBodyPart& s) { return s.bodyState; }
    };

    struct LinearDeclaration
    {
        using StateType = LinearBodyPart;
        static const LinearBodyState& bodyStateOf(const LinearBodyPart& s) { return s.bodyState; }
    };

    struct TwoDeclarationComposite
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

    struct OneDeclarationComposite
    {
        LinearDeclaration linear;

        template <typename F>
        void forEach(F&& f) const
        {
            f(linear);
        }
    };

    struct NoInput { int unused = 0; };

    struct TwoBodySimulatable
    {
        using InputType = NoInput;

        const AllStateOf<TwoBodyState>& getAllState() const { return allState; }
        const TwoDeclarationComposite& getPhysicsComposite() const { return composite; }

        AllStateOf<TwoBodyState> allState;
        TwoDeclarationComposite composite;
    };

    struct OneBodySimulatable
    {
        using InputType = NoInput;

        const AllStateOf<OneBodyState>& getAllState() const { return allState; }
        const OneDeclarationComposite& getPhysicsComposite() const { return composite; }

        AllStateOf<OneBodyState> allState;
        OneDeclarationComposite composite;
    };

    glm::vec3 fullPositionOf(uint32_t id) { return glm::vec3(float(id), 10.f + float(id), -5.f * float(id)); }
    glm::vec3 linearPositionOf(uint32_t id) { return glm::vec3(-float(id), 0.5f * float(id), 300.f + float(id)); }
    glm::quat fullRotationOf(uint32_t id) { return glm::quat(0.5f, -0.5f, 0.25f, float(id)); }

    TwoBodySimulatable twoBodySimulatable(uint32_t id)
    {
        TwoBodySimulatable simulatable;
        simulatable.allState.state.full.bodyState = PhysicsBodyState{ fullPositionOf(id), fullRotationOf(id), glm::vec3(1.f, 0.f, 0.f), glm::vec3(0.f, 0.f, 2.f) };
        simulatable.allState.state.linear.bodyState.position = linearPositionOf(id);
        simulatable.allState.state.linear.bodyState.linearVelocity = glm::vec3(0.f, 4.f, 0.f);
        return simulatable;
    }

    OneBodySimulatable oneBodySimulatable(uint32_t id)
    {
        OneBodySimulatable simulatable;
        simulatable.allState.state.linear.bodyState.position = linearPositionOf(id);
        return simulatable;
    }

    using TwoBodyStorage = SimulationObjectStorage<TwoBodySimulatable>;
    using MixedStorage = SimulationObjectStorage<TwoBodySimulatable, OneBodySimulatable>;
    using Snapshot = RenderSnapshotT<16>;

    template <typename StorageT>
    std::vector<uint32_t> storageWalkOrder(const StorageT& storage)
    {
        std::vector<uint32_t> ids;
        storage.forEachSimulatable([&ids](unsigned int id, const auto&) { ids.push_back(id); });
        return ids;
    }

    TickOutcome someOutcome()
    {
        TickOutcome outcome;
        outcome.tick = 4242u;
        outcome.kind = StepKind::Skip;
        outcome.hardResync = true;
        outcome.replayedTicks = 7u;
        outcome.resimRefused = true;
        outcome.physicsStep = 123456789012ull;
        return outcome;
    }

    bool isIdentity(const glm::quat& rotation)
    {
        return rotation.w == 1.f && rotation.x == 0.f && rotation.y == 0.f && rotation.z == 0.f;
    }
}

TEST_CASE("RenderSnapshot.BodiesAreInKeyOrderOneEntryPerDeclaration", "[RenderSnapshot]")
{
    TwoBodyStorage storage;
    for (const uint32_t id : { 1000u, 40u, 17u, 8u, 3u })
    {
        storage.add(id, twoBodySimulatable(id));
    }
    const std::vector<uint32_t> walk = storageWalkOrder(storage);
    INFO("the storage walk must not already be in key order, or this case cannot see a missing sort");
    REQUIRE_FALSE(std::is_sorted(walk.begin(), walk.end()));

    Snapshot snapshot;
    fillRenderSnapshot(snapshot, storage, someOutcome(), 0.0);

    REQUIRE(snapshot.bodyCount == 10u);
    const std::vector<uint32_t> sortedIds{ 3u, 8u, 17u, 40u, 1000u };
    for (size_t index = 0; index < snapshot.bodyCount; ++index)
    {
        const RenderBody& body = snapshot.bodies[index];
        INFO("entry " << index);
        CHECK(body.simulatableId == sortedIds[index / 2u]);
        CHECK(body.declarationIndex == uint8_t(index % 2u));
        CHECK(body.positionCm == (index % 2u == 0u ? fullPositionOf(body.simulatableId) : linearPositionOf(body.simulatableId)));
    }
}

TEST_CASE("RenderSnapshot.HasRotationOnlyForAFullBodyState", "[RenderSnapshot]")
{
    TwoBodyStorage storage;
    TwoBodySimulatable simulatable = twoBodySimulatable(5u);
    simulatable.allState.state.linear.bodyState.position = glm::vec3(7.f, 8.f, 9.f);
    storage.add(5u, std::move(simulatable));

    Snapshot snapshot;
    fillRenderSnapshot(snapshot, storage, someOutcome(), 0.0);

    REQUIRE(snapshot.bodyCount == 2u);
    const RenderBody& full = snapshot.bodies[0];
    const RenderBody& linear = snapshot.bodies[1];
    REQUIRE_FALSE(isIdentity(fullRotationOf(5u)));
    CHECK(full.declarationIndex == 0u);
    CHECK(full.hasRotation == 1u);
    CHECK(full.rotation == fullRotationOf(5u));
    CHECK(full.positionCm == fullPositionOf(5u));
    CHECK(linear.declarationIndex == 1u);
    CHECK(linear.hasRotation == 0u);
    CHECK(isIdentity(linear.rotation));
    CHECK(linear.positionCm == glm::vec3(7.f, 8.f, 9.f));
}

TEST_CASE("RenderSnapshot.OutcomeFieldsAndTheDeadlineAreCopied", "[RenderSnapshot]")
{
    TwoBodyStorage storage;
    storage.add(1u, twoBodySimulatable(1u));
    const TickOutcome outcome = someOutcome();

    Snapshot snapshot;
    fillRenderSnapshot(snapshot, storage, outcome, 12.5);

    CHECK(snapshot.physicsStep == outcome.physicsStep);
    CHECK(snapshot.stepDeadlineSeconds == 12.5);
    CHECK(snapshot.tick == outcome.tick);
    CHECK(snapshot.kind == StepKind::Skip);
    CHECK(snapshot.hardResync);
    CHECK(snapshot.replayedTicks == 7u);
    CHECK(snapshot.bodyCount == 2u);

    TickOutcome normal;
    normal.tick = 9u;
    normal.physicsStep = 11u;
    fillRenderSnapshot(snapshot, storage, normal, 0.25);
    CHECK(snapshot.physicsStep == 11u);
    CHECK(snapshot.stepDeadlineSeconds == 0.25);
    CHECK(snapshot.tick == 9u);
    CHECK(snapshot.kind == StepKind::Normal);
    CHECK_FALSE(snapshot.hardResync);
    CHECK(snapshot.replayedTicks == 0u);
}

TEST_CASE("RenderSnapshot.BodyCountFollowsTheStorageOnARefill", "[RenderSnapshot]")
{
    Snapshot snapshot;
    {
        TwoBodyStorage storage;
        for (const uint32_t id : { 1u, 2u, 3u, 4u })
        {
            storage.add(id, twoBodySimulatable(id));
        }
        fillRenderSnapshot(snapshot, storage, someOutcome(), 0.0);
        REQUIRE(snapshot.bodyCount == 8u);
    }

    TwoBodyStorage smaller;
    smaller.add(9u, twoBodySimulatable(9u));
    fillRenderSnapshot(snapshot, smaller, someOutcome(), 0.0);
    CHECK(snapshot.bodyCount == 2u);
    CHECK(snapshot.bodies[0].simulatableId == 9u);
    CHECK(snapshot.bodies[1].simulatableId == 9u);

    TwoBodyStorage empty;
    fillRenderSnapshot(snapshot, empty, someOutcome(), 0.0);
    CHECK(snapshot.bodyCount == 0u);
}

TEST_CASE("RenderSnapshot.KeyOrderSpansSimulatableTypes", "[RenderSnapshot]")
{
    MixedStorage storage;
    storage.add(20u, twoBodySimulatable(20u));
    storage.add(5u, twoBodySimulatable(5u));
    storage.add(10u, oneBodySimulatable(10u));
    storage.add(1u, oneBodySimulatable(1u));

    Snapshot snapshot;
    fillRenderSnapshot(snapshot, storage, someOutcome(), 0.0);

    REQUIRE(snapshot.bodyCount == 6u);
    const std::vector<std::pair<uint32_t, uint8_t>> expected{ { 1u, 0u }, { 5u, 0u }, { 5u, 1u }, { 10u, 0u }, { 20u, 0u }, { 20u, 1u } };
    const std::vector<uint8_t> expectedRotation{ 0u, 1u, 0u, 0u, 1u, 0u };
    for (size_t index = 0; index < expected.size(); ++index)
    {
        INFO("entry " << index);
        CHECK(snapshot.bodies[index].simulatableId == expected[index].first);
        CHECK(snapshot.bodies[index].declarationIndex == expected[index].second);
        CHECK(snapshot.bodies[index].hasRotation == expectedRotation[index]);
    }
    CHECK(snapshot.bodies[0].positionCm == linearPositionOf(1u));
    CHECK(snapshot.bodies[3].positionCm == linearPositionOf(10u));
}

#endif // WITH_LOW_LEVEL_TESTS
