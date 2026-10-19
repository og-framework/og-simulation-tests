// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/PhysicsDeclaration.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/TickDriverConcepts.h"
#include "OGSimulation/TickDriverLog.h"
#include "OGSimulation/TickDriverPresenter.h"
#include "OGSimulationJolt/JoltTickWorldHost.h"
#include "LegacyRenderModel.h"
#include "TickDriverFakes.h"

namespace tickDriverPresenterTest
{
    using tickDriverFakes::FakeEngine;
    using tickDriverFakes::FakeEngineNoDiagnostics;
    using tickDriverFakes::FakeRenderSink;
    using tickDriverFakes::LoggedLine;

    struct PresenterAllocationCounter
    {
        static inline uint64_t allocations = 0u;
    };

    template <typename T>
    struct PresenterCountingAllocator
    {
        using value_type = T;

        PresenterCountingAllocator() = default;

        template <typename U>
        PresenterCountingAllocator(const PresenterCountingAllocator<U>&) noexcept
        {
        }

        T* allocate(std::size_t count)
        {
            ++PresenterAllocationCounter::allocations;
            return std::allocator<T>{}.allocate(count);
        }

        void deallocate(T* pointer, std::size_t count) noexcept { std::allocator<T>{}.deallocate(pointer, count); }

        template <typename U>
        bool operator==(const PresenterCountingAllocator<U>&) const noexcept
        {
            return true;
        }
    };

    struct NoInput
    {
        int unused = 0;
    };

    struct RootPart { LinearBodyState bodyState; };
    struct WeaponPart { PhysicsBodyState bodyState; };
    struct ProjectilePart { PhysicsBodyState bodyState; };

    using TestState = SimulationComposite<RootPart, WeaponPart, ProjectilePart>;

    struct TestAllState
    {
        TestState state;
        const TestState& getState() const { return state; }
        TestState& editState() { return state; }
    };

    struct TestBodyDescriptor
    {
        struct Body
        {
            bool isRoot = false;
        };
        Body body;
    };

    struct RootDeclaration
    {
        using StateType = RootPart;
        static const LinearBodyState& bodyStateOf(const RootPart& part) { return part.bodyState; }
        static LinearBodyState& bodyStateOf(RootPart& part) { return part.bodyState; }
        static TestBodyDescriptor descriptor() { return TestBodyDescriptor{ { true } }; }
    };

    struct WeaponDeclaration
    {
        using StateType = WeaponPart;
        static constexpr RenderCorrectionMode renderCorrection = RenderCorrectionMode::SnapInRootFrame;
        static const PhysicsBodyState& bodyStateOf(const WeaponPart& part) { return part.bodyState; }
        static PhysicsBodyState& bodyStateOf(WeaponPart& part) { return part.bodyState; }
        static TestBodyDescriptor descriptor() { return TestBodyDescriptor{ { false } }; }
    };

    struct ProjectileDeclaration
    {
        using StateType = ProjectilePart;
        static const PhysicsBodyState& bodyStateOf(const ProjectilePart& part) { return part.bodyState; }
        static PhysicsBodyState& bodyStateOf(ProjectilePart& part) { return part.bodyState; }
        static TestBodyDescriptor descriptor() { return TestBodyDescriptor{ { false } }; }
    };

    using TestComposite = SimulationComposite<RootDeclaration, WeaponDeclaration, ProjectileDeclaration>;

    class TestSimulatable
    {
    public:
        using StateType = TestState;
        using InputType = NoInput;

        const TestAllState& getAllState() const { return m_all; }
        TestAllState& editAllState() { return m_all; }
        void updateVizState() { m_viz = m_all; }
        const TestAllState& getVizState() const { return m_viz; }
        TestAllState& editVizState() { return m_viz; }
        const TestComposite& getPhysicsComposite() const { return m_composite; }

    private:
        TestAllState m_all;
        TestAllState m_viz;
        TestComposite m_composite;
    };

    using Storage = SimulationObjectStorage<TestSimulatable>;

    constexpr size_t kSlots = 4u;
    constexpr uint32_t kDeclarations = 3u;
    using Snapshot = RenderSnapshotT<kSlots * kDeclarations>;

    struct TestSource
    {
        using Storage = tickDriverPresenterTest::Storage;
        using PhysicsComposite = TestComposite;
        static constexpr size_t kMaxSimulatableSlots = kSlots;

        Storage& storage() { return *store; }
        bool savesTicks() const { return saves; }
        uint64_t grantedResims() const { return resims; }
        int64_t integratedTick() const { return tick; }

        Storage* store = nullptr;
        bool saves = true;
        uint64_t resims = 0u;
        int64_t tick = 0;
    };

    struct RestoreNote
    {
        double seconds = 0.0;
        bool refused = false;

        bool operator==(const RestoreNote&) const = default;
    };

    struct TestClock
    {
        double hostNowSeconds() const { return host; }
        double stepIntervalSeconds() const { return interval; }
        void noteShadowRestore(double restoreSeconds, bool refused) { notes.push_back(RestoreNote{ restoreSeconds, refused }); }
        void noteShadowRestore_GameThread(double restoreSeconds, bool refused) { notes.push_back(RestoreNote{ restoreSeconds, refused }); }

        uint32_t refusedNotes() const
        {
            uint32_t count = 0u;
            for (const RestoreNote& note : notes)
                count += note.refused ? 1u : 0u;
            return count;
        }

        double host = 0.0;
        double interval = 1.0 / 60.0;
        std::vector<RestoreNote> notes;
    };

    static_assert(TickDriverPresentClock<TestClock>, "TestClock models the presenter's clock, the driver's doors included");

    class UeLikeRenderSink
    {
    public:
        using FVector = legacyRenderUe::FVector;
        using FQuat = legacyRenderUe::FQuat;

        struct Write
        {
            uint32_t id = 0u;
            glm::dvec3 position{ 0.0 };
            glm::dquat rotation{ 1.0, 0.0, 0.0, 0.0 };

            bool operator==(const Write& other) const
            {
                return id == other.id && position == other.position && rotation == other.rotation;
            }
        };

        class Actor
        {
        public:
            struct Transform
            {
                FVector translation;
                FVector GetTranslation() const { return translation; }
            };

            FVector GetActorLocation() const { return location; }
            FQuat GetActorQuat() const { return rotation; }

            void SetActorLocationAndRotation(const FVector& newLocation, const FQuat& newRotation, bool, std::nullptr_t,
                legacyRenderUe::ETeleportType)
            {
                sink->write(id, newLocation, newRotation);
            }

            const Actor* GetCapsuleComponent() const { return this; }
            Transform GetComponentTransform() const { return Transform{ location }; }

            UeLikeRenderSink* sink = nullptr;
            unsigned int id = 0u;
            FVector location;
            FQuat rotation{ 1.0, 0.0, 0.0, 0.0 };
            std::optional<double> floorZ;
            std::string displayName;
        };

        using Character = Actor;

        UeLikeRenderSink() = default;
        UeLikeRenderSink(const UeLikeRenderSink&) = delete;
        UeLikeRenderSink& operator=(const UeLikeRenderSink&) = delete;

        void spawn(unsigned int id, const glm::dvec3& location)
        {
            Actor& actor = actors[id];
            actor.sink = this;
            actor.id = id;
            actor.location = FVector{ location.x, location.y, location.z };
            actor.displayName = "Brawler_" + std::to_string(id);
        }

        void destroy(unsigned int id) { actors.erase(id); }

        void nudge(unsigned int id, const glm::dvec3& delta)
        {
            Actor& actor = actors.at(id);
            actor.location.X += delta.x;
            actor.location.Y += delta.y;
            actor.location.Z += delta.z;
        }

        void setFloor(unsigned int id, double z) { actors.at(id).floorZ = z; }

        Actor* Cast(unsigned int id)
        {
            const auto found = actors.find(id);
            return found == actors.end() ? nullptr : &found->second;
        }

        legacyRenderUe::TWeakObjectPtr<Actor> weakOf(Actor* character, unsigned int id)
        {
            if (character == nullptr)
                return {};
            return legacyRenderUe::TWeakObjectPtr<Actor>{ &UeLikeRenderSink::lookup, this, id };
        }

        std::optional<glm::dvec3> applyRootPose(uint32_t id, const glm::vec3& position, const std::optional<glm::quat>& rotation)
        {
            Actor* const actor = Cast(id);
            if (actor == nullptr)
                return std::nullopt;
            write(id, legacyRenderUe::uglm::toFVector(position), rotation.has_value() ? FQuat(*rotation) : actor->rotation);
            return glm::dvec3(actor->location.X, actor->location.Y, actor->location.Z);
        }

        std::optional<glm::dvec3> readAnchor(uint32_t id) const
        {
            const auto found = actors.find(id);
            if (found == actors.end())
                return std::nullopt;
            const FVector& location = found->second.location;
            return glm::dvec3(location.X, location.Y, location.Z);
        }

        const char* name(uint32_t id) const
        {
            const auto found = actors.find(id);
            return found == actors.end() ? "?" : found->second.displayName.c_str();
        }

        std::map<unsigned int, Actor> actors;
        std::vector<Write> writes;

    private:
        static Actor* lookup(void* self, unsigned int id) { return static_cast<UeLikeRenderSink*>(self)->Cast(id); }

        static bool sameWithin(const FQuat& first, const FQuat& second, double tolerance)
        {
            return (std::abs(first.x - second.x) <= tolerance && std::abs(first.y - second.y) <= tolerance
                       && std::abs(first.z - second.z) <= tolerance && std::abs(first.w - second.w) <= tolerance)
                || (std::abs(first.x + second.x) <= tolerance && std::abs(first.y + second.y) <= tolerance
                    && std::abs(first.z + second.z) <= tolerance && std::abs(first.w + second.w) <= tolerance);
        }

        void write(unsigned int id, const FVector& location, const FQuat& rotation)
        {
            writes.push_back(Write{ id, glm::dvec3(location.X, location.Y, location.Z), rotation });
            Actor& actor = actors.at(id);
            const double tolerance = static_cast<double>(legacyRenderUe::kUeKindaSmallNumber);
            if (!actor.location.Equals(location, tolerance) || !sameWithin(actor.rotation, rotation, tolerance))
            {
                actor.location = location;
                actor.rotation = rotation;
            }
            if (actor.floorZ.has_value() && actor.location.Z < *actor.floorZ)
                actor.location.Z = *actor.floorZ;
        }
    };

    static_assert(TickDriverRenderSink<UeLikeRenderSink>,
        "UeLikeRenderSink: a sink that returns double-precision positions models TickDriverRenderSink");

    template <typename EngineT>
    class TestShadowWorld
    {
    public:
        using BodyKey = uint32_t;

        TestShadowWorld(const EngineT& engine, bool hasShadow)
            : transforms(kSlots * kDeclarations, glm::mat4(1.f))
            , bound(kSlots * kDeclarations, 0u)
            , readBackOffset(kSlots * kDeclarations, glm::vec3(0.f))
            , m_engine(&engine)
            , m_hasShadow(hasShadow)
        {
        }

        bool hasShadowWorld() const { return m_hasShadow; }

        template <typename OnRestoreF>
        void restore_GameThread(uint64_t frameNumber, OnRestoreF&& onRestore)
        {
            restoreAt(frameNumber);
            if (m_newestSequence > m_restoredSequence)
            {
                const bool restored = restoreFromSnapshot(m_newestSequence);
                const double restoreSeconds = restoreSecondsOf(m_newestSequence);
                m_restoredSequence = m_newestSequence;
                onRestore(restoreSeconds, !restored);
            }
        }

        template <typename TargetsT, typename VisitorT>
        void overwriteShadowPoses_GameThread(const Snapshot& poses, TargetsT&& targetSlots, VisitorT&& visitor)
        {
            OG_CHECK(m_restoredFrame == m_engine->frameNumber(),
                "TestShadowWorld: the shadow world was not restored this frame before the render poses were written into it");
            targetSlots([&](uint32_t id, uint32_t slot)
            {
                for (uint32_t declarationIndex = 0u; declarationIndex < kDeclarations; ++declarationIndex)
                {
                    const RenderBody* const pose = findRenderBody(poses, id, static_cast<uint8_t>(declarationIndex));
                    if (pose == nullptr)
                        continue;
                    const BodyKey body = shadowSlotBody(slot, declarationIndex);
                    if (!shadowIsBound(body))
                        continue;

                    glm::mat4 transform = pose->hasRotation != 0u ? glm::mat4_cast(pose->rotation) : shadowGetBodyTransform(body);
                    transform[3] = glm::vec4(pose->positionCm, 1.f);
                    shadowSetBodyTransform(body, transform);
                    visitor(id, declarationIndex, *pose, [this, body]() { return shadowReaderGetBodyTransform(body); });
                }
            });
        }

        template <typename FrameHostT>
        void restoreShadowWorld_GameThread(FrameHostT& m_frameHost)
        {
            restoreAt(m_engine->frameNumber());
            if (m_newestSequence > m_restoredSequence)
            {
                const bool restored = restoreFromSnapshot(m_newestSequence);
                const double restoreSeconds = restoreSecondsOf(m_newestSequence);
                m_restoredSequence = m_newestSequence;
                m_frameHost.noteShadowRestore_GameThread(restoreSeconds, !restored);
            }
        }

        void publishShadowState() { ++m_newestSequence; }
        uint64_t shadowRestoredFrame() const { return m_restoredFrame; }

        template <typename CharacterT>
        std::optional<uint32_t> slotByOwner(const CharacterT* character) const
        {
            const auto found = m_slotById.find(character->id);
            if (found == m_slotById.end())
                return std::nullopt;
            return found->second;
        }

        uint32_t shadowBodiesPerSlot() const { return kDeclarations; }
        BodyKey shadowSlotBody(uint32_t slot, uint32_t declarationIndex) const { return slot * kDeclarations + declarationIndex; }
        bool shadowIsBound(BodyKey body) const { return bound[body] != 0u; }
        glm::mat4 shadowGetBodyTransform(BodyKey body) const { return transforms[body]; }
        void shadowSetBodyTransform(BodyKey body, const glm::mat4& transform) { transforms[body] = transform; }

        glm::mat4 shadowReaderGetBodyTransform(BodyKey body) const
        {
            glm::mat4 queried = transforms[body];
            queried[3] += glm::vec4(readBackOffset[body], 0.f);
            return queried;
        }

        void bindSlot(unsigned int id, uint32_t slot, uint32_t boundDeclarations)
        {
            m_slotById[id] = slot;
            for (uint32_t declarationIndex = 0u; declarationIndex < kDeclarations; ++declarationIndex)
                bound[shadowSlotBody(slot, declarationIndex)] = declarationIndex < boundDeclarations ? 1u : 0u;
        }

        void releaseSlot(unsigned int id)
        {
            const auto found = m_slotById.find(id);
            if (found == m_slotById.end())
                return;
            for (uint32_t declarationIndex = 0u; declarationIndex < kDeclarations; ++declarationIndex)
                bound[shadowSlotBody(found->second, declarationIndex)] = 0u;
            m_slotById.erase(found);
        }

        uint32_t restores = 0u;
        std::vector<glm::mat4> transforms;
        std::vector<uint8_t> bound;
        std::vector<glm::vec3> readBackOffset;

    private:
        static bool restoreFromSnapshot(uint64_t sequence) { return sequence % 7u != 3u; }
        static double restoreSecondsOf(uint64_t sequence) { return 1.0e-5 * static_cast<double>(sequence % 5u + 1u); }

        void restoreAt(uint64_t frameNumber)
        {
            ++restores;
            m_restoredFrame = frameNumber;
            for (uint32_t body = 0u; body < transforms.size(); ++body)
            {
                if (bound[body] == 0u)
                    continue;
                glm::mat4 newest = glm::rotate(glm::mat4(1.f), 0.25f * static_cast<float>(body), glm::vec3(0.f, 0.f, 1.f));
                newest[3] = glm::vec4(1000.f + static_cast<float>(body), -50.f, 7.f, 1.f);
                transforms[body] = newest;
            }
        }

        const EngineT* m_engine;
        bool m_hasShadow;
        uint64_t m_restoredFrame = ~uint64_t{ 0 };
        uint64_t m_newestSequence = 0u;
        uint64_t m_restoredSequence = 0u;
        std::map<unsigned int, uint32_t> m_slotById;
    };

    template <typename EngineT>
    using JoltHost = JoltTickWorldHost<TestSimulatable, tickDriverFakes::WrapperOverLockUnlockOnly, EngineT>;

    static_assert(TickDriverPresentWorldHost<JoltHost<FakeEngine>, Snapshot>,
        "JoltTickWorldHost (diagnostics on) models the presenter's world host: restore_GameThread takes the restore note");
    static_assert(TickDriverPresentWorldHost<JoltHost<FakeEngineNoDiagnostics>, Snapshot>,
        "JoltTickWorldHost (diagnostics off) models the presenter's world host: restore_GameThread takes the restore note");

    struct RigConfig
    {
        bool runsPrediction = true;
        bool hasShadow = true;
        double frameSeconds = 0.01;
        double stepSeconds = 1.0 / 60.0;
        float parkSnap = 1000.f;
        double wallClockOffset = 1000.0;
        double lengthReportScale = 1.0;
        LegacyRenderCvars cvars;
    };

    struct BodyMotion
    {
        glm::vec3 velocity{ 0.f };
        float weaponTurnRate = 0.f;
    };

    template <typename EngineT>
    class DifferentialRig
    {
    public:
        using Presenter = BasicTickDriverPresenter<Snapshot, EngineT, TestSource, PresenterCountingAllocator<char>>;
        using Legacy = LegacyRenderModel<Snapshot, EngineT, TestSource, TestClock, UeLikeRenderSink, TestShadowWorld<EngineT>>;

        static constexpr bool kDiagnostics = static_cast<bool>(EngineT::kHostDiagnostics);

        explicit DifferentialRig(const RigConfig& rigConfig)
            : config(rigConfig)
            , cvars(rigConfig.cvars)
            , presenterWorld(presenterEngine, rigConfig.hasShadow)
            , legacyWorld(legacyEngine, rigConfig.hasShadow)
        {
            source.store = &storage;
            source.saves = rigConfig.runsPrediction;
            clock.interval = rigConfig.stepSeconds;
            legacyClock.interval = rigConfig.stepSeconds;
            const uint64_t before = PresenterAllocationCounter::allocations;
            presenter = std::make_unique<Presenter>(presenterEngine, source,
                TickDriverPresenterConfig{ rigConfig.runsPrediction, static_cast<float>(rigConfig.stepSeconds), 0u });
            constructionAllocations = PresenterAllocationCounter::allocations - before;
            legacy = std::make_unique<Legacy>(legacyEngine, source, legacyClock, legacySink, legacyWorld, cvars,
                rigConfig.runsPrediction, static_cast<float>(rigConfig.stepSeconds), static_cast<uint8_t>(0u), rigConfig.parkSnap);
        }

        PresentControls controls() const
        {
            PresentControls present;
            present.delaySteps = cvars.InterpolationDelaySteps;
            present.newestOnly = cvars.NewestSnapshot != 0;
            present.correctionSeconds = cvars.ErrorCorrectionSeconds;
            present.maxCorrection = cvars.ErrorCorrectionMaxCm;
            present.desyncSeconds = cvars.ErrorCorrectionDesyncSeconds;
            present.traceFrames = cvars.TraceFrames != 0;
            present.parkSnap = config.parkSnap;
            present.lengthReportScale = config.lengthReportScale;
            return present;
        }

        void join(unsigned int id, uint32_t slot, const glm::vec3& at, const BodyMotion& motion, uint32_t boundDeclarations = kDeclarations)
        {
            TestSimulatable simulatable;
            TestState& state = simulatable.editAllState().state;
            state.edit<RootPart>().bodyState.position = at;
            state.edit<RootPart>().bodyState.linearVelocity = motion.velocity;
            state.edit<WeaponPart>().bodyState.position = at + glm::vec3(40.f, 0.f, 10.f);
            state.edit<ProjectilePart>().bodyState.position = at + glm::vec3(0.f, 0.f, 200.f);
            state.edit<ProjectilePart>().bodyState.linearVelocity = motion.velocity * 2.f;
            storage.add(id, std::move(simulatable));
            motions[id] = motion;

            const glm::dvec3 spawnedAt(at.x - 300.0, at.y + 125.0, at.z + 90.0);
            presenterSink.spawn(id, spawnedAt);
            legacySink.spawn(id, spawnedAt);
            presenterWorld.bindSlot(id, slot, boundDeclarations);
            legacyWorld.bindSlot(id, slot, boundDeclarations);
            presenter->onRegistered(id, slot);
            legacy->onRegistered_GameThread(id);
            slots[id] = slot;
        }

        void leave(unsigned int id)
        {
            const uint64_t before = PresenterAllocationCounter::allocations;
            presenter->onUnregistered(id, slots.at(id));
            unregisterAllocations += PresenterAllocationCounter::allocations - before;
            legacy->onUnregistered_GameThread(id);
            presenterWorld.releaseSlot(id);
            legacyWorld.releaseSlot(id);
            presenterSink.destroy(id);
            legacySink.destroy(id);
            storage.remove<TestSimulatable>(id);
            motions.erase(id);
            slots.erase(id);
        }

        void destroyActorOnly(unsigned int id)
        {
            presenterSink.destroy(id);
            legacySink.destroy(id);
        }

        void moveStorage(unsigned int id, const glm::vec3& offset)
        {
            TestState& state = storage.get<TestSimulatable>(id).editAllState().state;
            state.edit<RootPart>().bodyState.position += offset;
            state.edit<WeaponPart>().bodyState.position += offset;
            state.edit<ProjectilePart>().bodyState.position += offset;
        }

        void markResimulation(const std::vector<std::pair<unsigned int, glm::vec3>>& corrections)
        {
            ++source.resims;
            for (const auto& [id, offset] : corrections)
                moveStorage(id, offset);
        }

        void capture(uint64_t physicsStep)
        {
            countStepAllocations([&] { presenter->captureCorrection_Step(physicsStep); });
            legacy->captureRenderCorrection_Step(physicsStep);
        }

        void integrate()
        {
            const float dt = static_cast<float>(config.stepSeconds);
            storage.forEachSimulatable([&](unsigned int id, TestSimulatable& simulatable)
            {
                const BodyMotion& motion = motions.at(id);
                TestState& state = simulatable.editAllState().state;
                LinearBodyState& root = state.edit<RootPart>().bodyState;
                root.position += motion.velocity * dt;
                PhysicsBodyState& weapon = state.edit<WeaponPart>().bodyState;
                weapon.rotation = glm::normalize(glm::angleAxis(motion.weaponTurnRate * dt, glm::vec3(0.f, 0.f, 1.f)) * weapon.rotation);
                weapon.position = root.position + glm::vec3(weapon.rotation * glm::vec3(40.f, 0.f, 10.f));
                PhysicsBodyState& projectile = state.edit<ProjectilePart>().bodyState;
                projectile.position += projectile.linearVelocity * dt;
            });
            ++source.tick;
        }

        void publish()
        {
            TickOutcome outcome;
            outcome.physicsStep = nextStep;
            outcome.tick = static_cast<SimTick>(source.tick);
            outcome.kind = stepKind;
            const double deadline = static_cast<double>(nextStep) * config.stepSeconds;
            countStepAllocations([&] { presenter->publish_Step(outcome, deadline); });
            legacy->publishRenderSnapshot_Step(outcome, deadline);
            presenterWorld.publishShadowState();
            legacyWorld.publishShadowState();
        }

        void step()
        {
            if (beforeEachStep)
                beforeEachStep(nextStep);
            capture(nextStep);
            integrate();
            publish();
            ++nextStep;
        }

        void frame()
        {
            ++frameIndex;
            clock.host = static_cast<double>(frameIndex) * config.frameSeconds;
            legacyClock.host = clock.host;
            presenterEngine.now = clock.host + config.wallClockOffset;
            legacyEngine.now = clock.host + config.wallClockOffset;
            presenterEngine.advanceFrame();
            legacyEngine.advanceFrame();

            while (static_cast<double>(nextStep - 1u) * config.stepSeconds <= clock.host)
                step();

            if (beforePresent)
                beforePresent(frameIndex);

            const size_t presenterLinesBefore = presenterEngine.lines.size();
            const size_t legacyLinesBefore = legacyEngine.lines.size();
            const size_t presenterWritesBefore = presenterSink.writes.size();
            const size_t legacyWritesBefore = legacySink.writes.size();

            const PresentControls present = controls();
            countStepAllocations([&] { presenter->present_GameThread(clock, present, presenterSink, presenterWorld); });
            legacy->beforePresent_GameThread();

            countStepAllocations([&] { presenter->updateVisualization_GameThread(storage); });
            const std::vector<glm::vec3> presenterViz = vizPositions();
            const std::vector<glm::quat> presenterVizRotations = vizRotations();
            legacy->updateVisualization_GameThread();
            const std::vector<glm::vec3> legacyViz = vizPositions();
            const std::vector<glm::quat> legacyVizRotations = vizRotations();

            std::ostringstream diff;
            compareWrites(diff, presenterWritesBefore, legacyWritesBefore);
            compareLines(diff, presenterLinesBefore, legacyLinesBefore);
            compareSnapshots(diff, "poses", presenter->renderPoses(), legacy->renderPoses());
            compareFrames(diff);
            compareShadow(diff);
            compareRestoreNotes(diff);
            if (presenterViz != legacyViz || presenterVizRotations != legacyVizRotations)
                diff << "viz state differs; ";
            if (presenter->renderPosesValid() != legacy->renderPosesValid())
                diff << "renderPosesValid differs; ";
            lastDiff = diff.str();
            if (!lastDiff.empty() && firstDiff.empty())
            {
                firstDiff = "frame " + std::to_string(frameIndex) + ": " + lastDiff;
            }
        }

        uint32_t countLines(const std::vector<LoggedLine>& lines, const std::string& prefix) const
        {
            uint32_t count = 0u;
            for (const LoggedLine& line : lines)
                count += line.text.rfind(prefix, 0) == 0u ? 1u : 0u;
            return count;
        }

        uint32_t legacyLines(const std::string& prefix) const { return countLines(legacyEngine.lines, prefix); }

        RigConfig config;
        LegacyRenderCvars cvars;
        EngineT presenterEngine;
        EngineT legacyEngine;
        Storage storage;
        TestSource source;
        TestClock clock;
        TestClock legacyClock;
        UeLikeRenderSink presenterSink;
        UeLikeRenderSink legacySink;
        TestShadowWorld<EngineT> presenterWorld;
        TestShadowWorld<EngineT> legacyWorld;
        std::unique_ptr<Presenter> presenter;
        std::unique_ptr<Legacy> legacy;
        std::map<unsigned int, BodyMotion> motions;
        std::map<unsigned int, uint32_t> slots;
        std::function<void(uint64_t)> beforeEachStep;
        std::function<void(uint32_t)> beforePresent;
        StepKind stepKind = StepKind::Normal;
        uint64_t nextStep = 1u;
        uint32_t frameIndex = 0u;
        uint64_t constructionAllocations = 0u;
        uint64_t steadyAllocations = 0u;
        uint64_t unregisterAllocations = 0u;
        std::string firstDiff;
        std::string lastDiff;

    private:
        template <typename F>
        void countStepAllocations(F&& call)
        {
            const uint64_t before = PresenterAllocationCounter::allocations;
            call();
            steadyAllocations += PresenterAllocationCounter::allocations - before;
        }

        std::vector<glm::vec3> vizPositions()
        {
            std::vector<glm::vec3> positions;
            storage.forEachSimulatable([&](unsigned int, TestSimulatable& simulatable)
            {
                const TestState& viz = simulatable.getVizState().state;
                positions.push_back(viz.get<RootPart>().bodyState.position);
                positions.push_back(viz.get<WeaponPart>().bodyState.position);
                positions.push_back(viz.get<ProjectilePart>().bodyState.position);
            });
            return positions;
        }

        std::vector<glm::quat> vizRotations()
        {
            std::vector<glm::quat> rotations;
            storage.forEachSimulatable([&](unsigned int, TestSimulatable& simulatable)
            {
                const TestState& viz = simulatable.getVizState().state;
                rotations.push_back(viz.get<WeaponPart>().bodyState.rotation);
                rotations.push_back(viz.get<ProjectilePart>().bodyState.rotation);
            });
            return rotations;
        }

        void compareWrites(std::ostringstream& diff, size_t presenterBefore, size_t legacyBefore) const
        {
            const std::vector<UeLikeRenderSink::Write> presenterWrites(presenterSink.writes.begin() + static_cast<std::ptrdiff_t>(presenterBefore),
                presenterSink.writes.end());
            const std::vector<UeLikeRenderSink::Write> legacyWrites(legacySink.writes.begin() + static_cast<std::ptrdiff_t>(legacyBefore),
                legacySink.writes.end());
            if (presenterWrites != legacyWrites)
            {
                diff << "sink writes differ (presenter " << presenterWrites.size() << ", legacy " << legacyWrites.size() << "); ";
            }
            for (const auto& [id, actor] : presenterSink.actors)
            {
                const auto other = legacySink.actors.find(id);
                if (other == legacySink.actors.end() || !(actor.location == other->second.location) || actor.rotation != other->second.rotation)
                    diff << "actor " << id << " differs; ";
            }
        }

        void compareLines(std::ostringstream& diff, size_t presenterBefore, size_t legacyBefore) const
        {
            const size_t presenterCount = presenterEngine.lines.size() - presenterBefore;
            const size_t legacyCount = legacyEngine.lines.size() - legacyBefore;
            if (presenterCount != legacyCount)
            {
                diff << "line count differs (presenter " << presenterCount << ", legacy " << legacyCount << "); ";
            }
            for (size_t index = 0u; index < presenterCount && index < legacyCount; ++index)
            {
                const LoggedLine& mine = presenterEngine.lines[presenterBefore + index];
                const LoggedLine& theirs = legacyEngine.lines[legacyBefore + index];
                if (mine.level != theirs.level || mine.text != theirs.text)
                {
                    diff << "line differs:\n  presenter: " << mine.text << "\n  legacy:    " << theirs.text << "\n";
                    return;
                }
            }
            if (presenterCount != legacyCount)
            {
                const std::vector<LoggedLine>& longer = presenterCount > legacyCount ? presenterEngine.lines : legacyEngine.lines;
                const size_t from = (presenterCount > legacyCount ? presenterBefore : legacyBefore) + std::min(presenterCount, legacyCount);
                diff << "first extra line (" << (presenterCount > legacyCount ? "presenter" : "legacy") << "): " << longer[from].text << "\n";
            }
        }

        static void compareSnapshots(std::ostringstream& diff, const char* what, const Snapshot& mine, const Snapshot& theirs)
        {
            bool same = mine.physicsStep == theirs.physicsStep && mine.stepDeadlineSeconds == theirs.stepDeadlineSeconds
                && mine.tick == theirs.tick && mine.kind == theirs.kind && mine.bodyCount == theirs.bodyCount;
            for (uint16_t index = 0u; same && index < mine.bodyCount; ++index)
            {
                const RenderBody& a = mine.bodies[index];
                const RenderBody& b = theirs.bodies[index];
                same = a.simulatableId == b.simulatableId && a.declarationIndex == b.declarationIndex && a.hasRotation == b.hasRotation
                    && a.positionCm == b.positionCm && a.rotation == b.rotation;
            }
            if (!same)
                diff << what << " differ; ";
        }

        void compareFrames(std::ostringstream& diff) const
        {
            const RenderInterpolationFrame& a = presenter->renderFrame();
            const RenderInterpolationFrame& b = legacy->renderFrame();
            if (a.renderTimeSeconds != b.renderTimeSeconds || a.alpha != b.alpha || a.prevStep != b.prevStep || a.nextStep != b.nextStep
                || a.newestStep != b.newestStep || a.nextBack != b.nextBack || a.snaps != b.snaps || a.hasPrev != b.hasPrev
                || a.newestOnly != b.newestOnly || a.atNewest != b.atNewest || a.beyondOldest != b.beyondOldest)
            {
                diff << "interpolation frame differs (render " << a.renderTimeSeconds << " vs " << b.renderTimeSeconds << "); ";
            }
            const RenderCorrectionFrame& c = presenter->renderCorrectionFrame();
            const RenderCorrectionFrame& d = legacy->renderCorrectionFrame();
            if (c.corrections != d.corrections || c.smoothedBodies != d.smoothedBodies || c.snappedBodies != d.snappedBodies
                || c.late != d.late || c.finished != d.finished || c.active != d.active || c.maxErrorCm != d.maxErrorCm
                || c.maxAppliedCm != d.maxAppliedCm || c.policySnaps != d.policySnaps || c.straddleSnaps != d.straddleSnaps)
            {
                diff << "correction frame differs; ";
            }
        }

        void compareShadow(std::ostringstream& diff) const
        {
            if (presenterWorld.transforms != legacyWorld.transforms || presenterWorld.restores != legacyWorld.restores)
                diff << "shadow world differs; ";
        }

        void compareRestoreNotes(std::ostringstream& diff) const
        {
            if (clock.notes != legacyClock.notes)
            {
                diff << "shadow restore notes differ (presenter " << clock.notes.size() << ", legacy " << legacyClock.notes.size()
                    << "); ";
            }
        }
    };

    template <typename EngineT>
    using Script = std::function<void(DifferentialRig<EngineT>&)>;

    template <typename EngineT>
    void expectSameAsLegacy(DifferentialRig<EngineT>& rig)
    {
        INFO(rig.firstDiff);
        CHECK(rig.firstDiff.empty());
        CHECK(rig.steadyAllocations == 0u);
        CHECK(rig.unregisterAllocations == 0u);
        if (!rig.config.hasShadow)
        {
            CHECK(rig.clock.notes.empty());
            CHECK(rig.legacyClock.notes.empty());
        }
        if constexpr (!DifferentialRig<EngineT>::kDiagnostics)
        {
            CHECK(rig.presenterEngine.lines.empty());
            CHECK(rig.legacyEngine.lines.empty());
        }
    }

    template <typename EngineT>
    void joinThreeOf(DifferentialRig<EngineT>& rig)
    {
        rig.join(3u, 0u, glm::vec3(100.f, 0.f, 50.f), BodyMotion{ glm::vec3(240.f, 30.f, 0.f), 2.5f });
        rig.join(17u, 2u, glm::vec3(-400.f, 220.f, 50.f), BodyMotion{ glm::vec3(-60.f, 310.f, 0.f), -4.f });
        rig.join(1u, 1u, glm::vec3(0.f, -500.f, 50.f), BodyMotion{ glm::vec3(0.f, 0.f, 0.f), 0.f });
    }
} // namespace tickDriverPresenterTest

using namespace tickDriverPresenterTest;

TEST_CASE("TickDriverPresenter.Differential.InterpolationAt100FpsAgainst60HzSteps", "[TickDriverPresenter]")
{
    const auto run = [](auto engineTag)
    {
        using EngineT = typename decltype(engineTag)::type;
        RigConfig config;
        config.cvars.TraceFrames = 1;
        DifferentialRig<EngineT> rig(config);
        joinThreeOf(rig);
        for (int frame = 0; frame < 1300; ++frame)
            rig.frame();
        expectSameAsLegacy(rig);
        CHECK(rig.legacyClock.notes.size() > 700u);
        CHECK(rig.legacyClock.notes.size() < 1300u);
        CHECK(rig.legacyClock.refusedNotes() > 100u);
        if constexpr (DifferentialRig<EngineT>::kDiagnostics)
        {
            CHECK(rig.legacyLines("[SimHost.RenderApply] role=") == 2u);
            CHECK(rig.legacyLines("[SimHost.RenderSync] role=") == 2u);
            CHECK(rig.legacyLines("[SimHost.RenderCorrection] role=") == 2u);
            CHECK(rig.legacyLines("[SimHost.RenderApply] first") == 3u);
            CHECK(rig.legacyLines("[SimHost.RenderTrace]") > 3000u);
        }
    };
    SECTION("kHostDiagnostics true") { run(std::type_identity<FakeEngine>{}); }
    SECTION("kHostDiagnostics false") { run(std::type_identity<FakeEngineNoDiagnostics>{}); }
}

TEST_CASE("TickDriverPresenter.Differential.ResimCorrectionCapturedSmoothedAndFinished", "[TickDriverPresenter]")
{
    const auto run = [](auto engineTag)
    {
        using EngineT = typename decltype(engineTag)::type;
        RigConfig config;
        config.cvars.TraceFrames = 1;
        DifferentialRig<EngineT> rig(config);
        joinThreeOf(rig);
        rig.beforeEachStep = [&rig](uint64_t step)
        {
            if (step == 1u)
                rig.markResimulation({});
            if (step == 40u)
                rig.markResimulation({ { 3u, glm::vec3(35.f, -20.f, 0.f) }, { 17u, glm::vec3(0.f, 12.f, 0.f) } });
            if (step == 90u)
                rig.markResimulation({ { 3u, glm::vec3(0.01f, 0.f, 0.f) } });
            if (step == 120u)
            {
                rig.source.tick += 5;
                rig.markResimulation({ { 1u, glm::vec3(30.f, 0.f, 0.f) } });
            }
            if (step == 121u)
                rig.source.tick -= 5;
            if (step == 200u)
                rig.markResimulation({ { 1u, glm::vec3(400.f, 0.f, 0.f) } });
        };
        for (int frame = 0; frame < 700; ++frame)
            rig.frame();
        expectSameAsLegacy(rig);
        if constexpr (DifferentialRig<EngineT>::kDiagnostics)
        {
            CHECK(rig.legacyLines("[SimHost.RenderCorrection] event") >= 2u);
            CHECK(rig.legacyLines("[SimHost.RenderCorrection] role=Client") == 1u);
            const LoggedLine* window = nullptr;
            for (const LoggedLine& line : rig.legacyEngine.lines)
                window = line.text.rfind("[SimHost.RenderCorrection] role=Client", 0) == 0u ? &line : window;
            REQUIRE(window != nullptr);
            INFO(window->text);
            CHECK(window->text.find("resims=5 captured=2 withoutError=1 tickMismatches=1") != std::string::npos);
            CHECK(window->text.find("decays=0") == std::string::npos);
        }
    };
    SECTION("kHostDiagnostics true") { run(std::type_identity<FakeEngine>{}); }
    SECTION("kHostDiagnostics false") { run(std::type_identity<FakeEngineNoDiagnostics>{}); }
}

TEST_CASE("TickDriverPresenter.Differential.LateCorrection", "[TickDriverPresenter]")
{
    const auto run = [](auto engineTag)
    {
        using EngineT = typename decltype(engineTag)::type;
        RigConfig config;
        config.cvars.InterpolationDelaySteps = 0.f;
        DifferentialRig<EngineT> rig(config);
        joinThreeOf(rig);
        rig.beforePresent = [&rig](uint32_t frame)
        {
            if (frame == 60u || frame == 61u)
            {
                rig.markResimulation({ { 17u, glm::vec3(25.f, 0.f, 0.f) } });
                rig.capture(rig.nextStep - 3u);
            }
        };
        for (int frame = 0; frame < 650; ++frame)
            rig.frame();
        expectSameAsLegacy(rig);
        if constexpr (DifferentialRig<EngineT>::kDiagnostics)
        {
            CHECK(rig.legacyLines("[SimHost.RenderCorrection] LATE") == 1u);
        }
    };
    SECTION("kHostDiagnostics true") { run(std::type_identity<FakeEngine>{}); }
    SECTION("kHostDiagnostics false") { run(std::type_identity<FakeEngineNoDiagnostics>{}); }
}

TEST_CASE("TickDriverPresenter.Differential.ParkSnap", "[TickDriverPresenter]")
{
    const auto run = [](auto engineTag)
    {
        using EngineT = typename decltype(engineTag)::type;
        RigConfig config;
        DifferentialRig<EngineT> rig(config);
        joinThreeOf(rig);
        rig.beforeEachStep = [&rig](uint64_t step)
        {
            if (step == 30u || step == 31u)
            {
                PhysicsBodyState& projectile = rig.storage.template get<TestSimulatable>(3u).editAllState().state.template edit<ProjectilePart>().bodyState;
                projectile.position.z += step == 30u ? -5000.f : 5000.f;
            }
        };
        for (int frame = 0; frame < 650; ++frame)
            rig.frame();
        expectSameAsLegacy(rig);
        if constexpr (DifferentialRig<EngineT>::kDiagnostics)
        {
            const LoggedLine* window = nullptr;
            for (const LoggedLine& line : rig.legacyEngine.lines)
                window = line.text.rfind("[SimHost.RenderApply] role=", 0) == 0u ? &line : window;
            REQUIRE(window != nullptr);
            INFO(window->text);
            CHECK(window->text.find(" snaps=0 ") == std::string::npos);
        }
    };
    SECTION("kHostDiagnostics true") { run(std::type_identity<FakeEngine>{}); }
    SECTION("kHostDiagnostics false") { run(std::type_identity<FakeEngineNoDiagnostics>{}); }
}

TEST_CASE("TickDriverPresenter.Differential.JoinAndLeave", "[TickDriverPresenter]")
{
    const auto run = [](auto engineTag)
    {
        using EngineT = typename decltype(engineTag)::type;
        RigConfig config;
        DifferentialRig<EngineT> rig(config);
        joinThreeOf(rig);
        rig.beforePresent = [&rig](uint32_t frame)
        {
            if (frame == 100u)
                rig.join(9u, 3u, glm::vec3(600.f, 600.f, 50.f), BodyMotion{ glm::vec3(-90.f, -90.f, 0.f), 1.f }, 2u);
            if (frame == 260u)
                rig.destroyActorOnly(1u);
            if (frame == 400u)
                rig.leave(3u);
            if (frame == 420u)
                rig.leave(1u);
            if (frame == 500u)
                rig.join(3u, 0u, glm::vec3(10.f, 10.f, 50.f), BodyMotion{ glm::vec3(50.f, 0.f, 0.f), 0.5f });
        };
        for (int frame = 0; frame < 1250; ++frame)
            rig.frame();
        expectSameAsLegacy(rig);
        if constexpr (DifferentialRig<EngineT>::kDiagnostics)
        {
            CHECK(rig.legacyLines("[SimHost.RenderApply] first") == 5u);
            const LoggedLine* window = nullptr;
            for (const LoggedLine& line : rig.legacyEngine.lines)
                window = window == nullptr && line.text.rfind("[SimHost.RenderApply] role=", 0) == 0u ? &line : window;
            REQUIRE(window != nullptr);
            INFO(window->text);
            CHECK(window->text.find(" missing=0 ") == std::string::npos);
        }
    };
    SECTION("kHostDiagnostics true") { run(std::type_identity<FakeEngine>{}); }
    SECTION("kHostDiagnostics false") { run(std::type_identity<FakeEngineNoDiagnostics>{}); }
}

TEST_CASE("TickDriverPresenter.Differential.NewestOnly", "[TickDriverPresenter]")
{
    const auto run = [](auto engineTag)
    {
        using EngineT = typename decltype(engineTag)::type;
        RigConfig config;
        config.cvars.NewestSnapshot = 1;
        config.cvars.TraceFrames = 1;
        DifferentialRig<EngineT> rig(config);
        joinThreeOf(rig);
        for (int frame = 0; frame < 650; ++frame)
            rig.frame();
        expectSameAsLegacy(rig);
        if constexpr (DifferentialRig<EngineT>::kDiagnostics)
        {
            CHECK(rig.legacyLines("[SimHost.RenderTrace]") > 0u);
            CHECK(rig.legacyEngine.lines.back().text.find("mode=newest") != std::string::npos);
        }
    };
    SECTION("kHostDiagnostics true") { run(std::type_identity<FakeEngine>{}); }
    SECTION("kHostDiagnostics false") { run(std::type_identity<FakeEngineNoDiagnostics>{}); }
}

TEST_CASE("TickDriverPresenter.Differential.DelayZeroTwoAndOutOfRange", "[TickDriverPresenter]")
{
    const auto run = [](auto engineTag, float delaySteps)
    {
        using EngineT = typename decltype(engineTag)::type;
        RigConfig config;
        config.cvars.InterpolationDelaySteps = delaySteps;
        config.cvars.TraceFrames = 1;
        DifferentialRig<EngineT> rig(config);
        joinThreeOf(rig);
        for (int frame = 0; frame < 650; ++frame)
            rig.frame();
        expectSameAsLegacy(rig);
    };
    for (const float delaySteps : { 0.f, 2.f, 3.5f, -1.f })
    {
        DYNAMIC_SECTION("delay " << delaySteps << ", kHostDiagnostics true") { run(std::type_identity<FakeEngine>{}, delaySteps); }
        DYNAMIC_SECTION("delay " << delaySteps << ", kHostDiagnostics false") { run(std::type_identity<FakeEngineNoDiagnostics>{}, delaySteps); }
    }
}

TEST_CASE("TickDriverPresenter.Differential.ChecksMovedMismatchAndShadow", "[TickDriverPresenter]")
{
    const auto run = [](auto engineTag, bool runsPrediction)
    {
        using EngineT = typename decltype(engineTag)::type;
        RigConfig config;
        config.runsPrediction = runsPrediction;
        config.hasShadow = runsPrediction;
        DifferentialRig<EngineT> rig(config);
        joinThreeOf(rig);
        rig.presenterWorld.readBackOffset[2u * kDeclarations + 1u] = glm::vec3(0.f, 0.002f, 0.f);
        rig.legacyWorld.readBackOffset[2u * kDeclarations + 1u] = glm::vec3(0.f, 0.002f, 0.f);
        rig.beforePresent = [&rig](uint32_t frame)
        {
            if (frame == 50u || frame == 51u)
            {
                rig.presenterSink.nudge(17u, glm::dvec3(5.0e-5, 0.0, 0.0));
                rig.legacySink.nudge(17u, glm::dvec3(5.0e-5, 0.0, 0.0));
            }
            if (frame == 80u)
            {
                rig.presenterSink.setFloor(1u, 50.5);
                rig.legacySink.setFloor(1u, 50.5);
            }
        };
        for (int frame = 0; frame < 650; ++frame)
            rig.frame();
        expectSameAsLegacy(rig);
        if constexpr (DifferentialRig<EngineT>::kDiagnostics)
        {
            CHECK(rig.legacyLines("[SimHost.RenderApply] MOVED") == 1u);
            CHECK(rig.legacyLines("[SimHost.RenderApply] MISMATCH") == 2u);
            CHECK(rig.legacyLines("[SimHost.RenderSync] SHADOW") == (runsPrediction ? 2u : 0u));
            CHECK(rig.legacyLines("[SimHost.RenderSync] role=") == (runsPrediction ? 1u : 0u));
            CHECK(rig.legacyLines("[SimHost.RenderCorrection] role=") == (runsPrediction ? 1u : 0u));
        }
    };
    SECTION("worker client, kHostDiagnostics true") { run(std::type_identity<FakeEngine>{}, true); }
    SECTION("worker client, kHostDiagnostics false") { run(std::type_identity<FakeEngineNoDiagnostics>{}, true); }
    SECTION("authority, kHostDiagnostics true") { run(std::type_identity<FakeEngine>{}, false); }
    SECTION("authority, kHostDiagnostics false") { run(std::type_identity<FakeEngineNoDiagnostics>{}, false); }
}

namespace tickDriverPresenterTest
{
    struct ParsedLine
    {
        std::string skeleton;
        std::vector<std::string> names;
        std::vector<double> lengths;
    };

    ParsedLine parseLengthFields(const std::string& text)
    {
        ParsedLine parsed;
        size_t position = 0u;
        while (position < text.size())
        {
            const size_t equals = text.find('=', position);
            if (equals == std::string::npos)
            {
                parsed.skeleton += text.substr(position);
                break;
            }
            parsed.skeleton += text.substr(position, equals + 1u - position);
            size_t nameStart = text.find_last_of(" [", equals);
            const std::string name = text.substr(nameStart == std::string::npos ? 0u : nameStart + 1u,
                equals - (nameStart == std::string::npos ? 0u : nameStart + 1u));
            size_t valueEnd = text.find_first_of(" :", equals + 1u);
            if (valueEnd == std::string::npos)
                valueEnd = text.size();
            const std::string value = text.substr(equals + 1u, valueEnd - equals - 1u);
            if (name.size() > 2u && name.compare(name.size() - 2u, 2u, "Cm") == 0)
            {
                std::stringstream parts(value);
                std::string part;
                while (std::getline(parts, part, '/'))
                {
                    parsed.names.push_back(name);
                    parsed.lengths.push_back(std::strtod(part.c_str(), nullptr));
                    parsed.skeleton += "#/";
                }
            }
            else
            {
                parsed.skeleton += value;
            }
            position = valueEnd;
        }
        return parsed;
    }
} // namespace tickDriverPresenterTest

TEST_CASE("TickDriverPresenter.Units.LengthReportScaleMultipliesEveryCmFieldOnce", "[TickDriverPresenter]")
{
    const auto runWithScale = [](double scale)
    {
        RigConfig config;
        config.lengthReportScale = scale;
        config.cvars.TraceFrames = 1;
        auto rig = std::make_unique<DifferentialRig<FakeEngine>>(config);
        joinThreeOf(*rig);
        rig->beforeEachStep = [rigPointer = rig.get()](uint64_t step)
        {
            if (step == 40u)
                rigPointer->markResimulation({ { 3u, glm::vec3(35.f, -20.f, 0.f) } });
        };
        rig->presenterWorld.readBackOffset[1u] = glm::vec3(0.f, 0.002f, 0.f);
        rig->legacyWorld.readBackOffset[1u] = glm::vec3(0.f, 0.002f, 0.f);
        rig->beforePresent = [rigPointer = rig.get()](uint32_t frame)
        {
            if (frame == 50u)
            {
                rigPointer->presenterSink.nudge(17u, glm::dvec3(5.0e-5, 0.0, 0.0));
                rigPointer->legacySink.nudge(17u, glm::dvec3(5.0e-5, 0.0, 0.0));
            }
            if (frame == 80u)
            {
                rigPointer->presenterSink.setFloor(1u, 50.5);
                rigPointer->legacySink.setFloor(1u, 50.5);
            }
        };
        for (int frame = 0; frame < 650; ++frame)
            rig->frame();
        return rig;
    };

    const auto unscaled = runWithScale(1.0);
    const auto scaled = runWithScale(2.0);
    INFO(unscaled->firstDiff);
    REQUIRE(unscaled->firstDiff.empty());

    const std::vector<LoggedLine>& one = unscaled->presenterEngine.lines;
    const std::vector<LoggedLine>& two = scaled->presenterEngine.lines;
    REQUIRE(one.size() == two.size());
    uint32_t lengthFields = 0u;
    uint32_t nonZeroLengthFields = 0u;
    std::set<std::string> fieldsSeen;
    for (size_t index = 0u; index < one.size(); ++index)
    {
        const ParsedLine a = parseLengthFields(one[index].text);
        const ParsedLine b = parseLengthFields(two[index].text);
        INFO(one[index].text);
        INFO(two[index].text);
        REQUIRE(a.skeleton == b.skeleton);
        REQUIRE(a.lengths.size() == b.lengths.size());
        for (size_t field = 0u; field < a.lengths.size(); ++field)
        {
            ++lengthFields;
            nonZeroLengthFields += a.lengths[field] != 0.0 ? 1u : 0u;
            const size_t tagEnd = one[index].text.find_first_of(" =", one[index].text.find(']') + 2u);
            fieldsSeen.insert(one[index].text.substr(0u, tagEnd) + " " + a.names[field]);
            const double expected = 2.0 * a.lengths[field];
            CHECK(std::abs(b.lengths[field] - expected) <= 2.0e-3 + 1.0e-6 * std::abs(expected));
            if (a.lengths[field] >= 0.01)
                CHECK(std::abs(b.lengths[field] / a.lengths[field] - 2.0) <= 0.1);
        }
    }
    CHECK(lengthFields > 15u);
    CHECK(nonZeroLengthFields > 10u);
    CHECK(fieldsSeen == std::set<std::string>{
        "[SimHost.RenderApply] role maxErrorCm", "[SimHost.RenderApply] role maxHeldDriftCm", "[SimHost.RenderApply] role pathCm",
        "[SimHost.RenderApply] role dispCm", "[SimHost.RenderSync] role shadowMaxErrorCm",
        "[SimHost.RenderCorrection] role maxErrorCm", "[SimHost.RenderCorrection] role maxAppliedCm",
        "[SimHost.RenderCorrection] role maxCm", "[SimHost.RenderCorrection] role maxPolicySnapCm",
        "[SimHost.RenderApply] MOVED driftCm", "[SimHost.RenderApply] MISMATCH errorCm", "[SimHost.RenderSync] SHADOW errorCm",
        "[SimHost.RenderCorrection] event maxErrorCm" });
}

TEST_CASE("TickDriverPresenter.Kit.FakeRenderSinkReceivesTheRootPoses", "[TickDriverPresenter]")
{
    FakeEngine engine;
    Storage storage;
    TestSource source;
    source.store = &storage;
    source.saves = false;
    TickDriverPresenter<Snapshot, FakeEngine, TestSource> presenter(engine, source, TickDriverPresenterConfig{ false, 1.f / 60.f, 0u });
    TestShadowWorld<FakeEngine> world(engine, false);
    FakeRenderSink sink;
    TestClock clock;

    TestSimulatable simulatable;
    simulatable.editAllState().state.edit<RootPart>().bodyState.position = glm::vec3(1.f, 2.f, 3.f);
    storage.add(5u, std::move(simulatable));
    sink.addTarget(5u, glm::vec3(0.f), "Brawler_5");
    presenter.onRegistered(5u, 0u);

    TickOutcome outcome;
    outcome.physicsStep = 1u;
    outcome.tick = 1;
    presenter.publish_Step(outcome, 1.0 / 60.0);
    clock.host = 0.5;
    PresentControls controls;
    controls.delaySteps = 0.f;
    presenter.present_GameThread(clock, controls, sink, world);

    REQUIRE(sink.applies.size() == 1u);
    CHECK(sink.applies[0].id == 5u);
    CHECK(sink.applies[0].position == glm::vec3(1.f, 2.f, 3.f));
    CHECK_FALSE(sink.applies[0].rotation.has_value());
    CHECK(sink.targets.at(5u).position == glm::vec3(1.f, 2.f, 3.f));
    REQUIRE(engine.lines.size() == 1u);
    CHECK(engine.lines[0].text == "[SimHost.RenderApply] first role=Authority id=5 from=(0.000,0.000,0.000) to=(1.000,2.000,3.000) physicsStep=1 tick=1");
}

#endif
