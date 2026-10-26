// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"
#include "../TickDriver/TickDriverFakes.h"

#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsBodyReaderAdapter.h"
#include "OGSimulationJolt/JoltSpatialQueryAdapter.h"
#include "OGSimulationJolt/JoltStaticWorldBuilder.h"
#include "OGSimulationJolt/JoltTickWorldHost.h"

#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/PCTimeManagement/ClientPredictionClock.h"
#include "OGSimulation/PCTimeManagement/ServerTickClock.h"
#include "OGSimulation/PhysicsDeclaration.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/ResimGateProbe.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationManager.h"
#include "OGSimulation/SimulationStepDriver.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/TickDriverThreadContext.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
    using tickDriverFakes::FakeEngine;
    using tickDriverFakes::FakeEngineNoDiagnostics;
    using tickDriverFakes::LoggedLine;

    std::atomic<uint64_t> g_lineAllocations{ 0u };

    template <typename T>
    struct CountingLineAllocator
    {
        using value_type = T;

        CountingLineAllocator() = default;

        template <typename U>
        CountingLineAllocator(const CountingLineAllocator<U>&)
        {
        }

        T* allocate(std::size_t count)
        {
            g_lineAllocations.fetch_add(1u);
            return std::allocator<T>{}.allocate(count);
        }

        void deallocate(T* pointer, std::size_t count) { std::allocator<T>{}.deallocate(pointer, count); }

        template <typename U>
        bool operator==(const CountingLineAllocator<U>&) const
        {
            return true;
        }
    };

    struct HostStaticData
    {
        glm::vec3 radialOffset = glm::vec3(0.7f, 0.3f, -0.f);
        float queryRadius = 0.5f;
    };

    struct HostRadialPart
    {
        PhysicsBodyState bodyState;
    };

    struct HostCapsulePart
    {
        PhysicsBodyState bodyState;
    };

    struct HostRadialDeclaration
    {
        using StateType = HostRadialPart;
        static constexpr const char* name = "hostRadial";

        static const PhysicalObjectDescriptor& descriptor()
        {
            static const PhysicalObjectDescriptor radial =
                joltTestRig::sphereTemplate(0.3f, joltTestRig::bit(joltTestRig::kBody), 0u, 0).descriptor;
            return radial;
        }

        static const HostStaticData& staticDataOf(const HostStaticData& staticData) { return staticData; }

        static std::vector<QueryVolumeDescriptor> queryVolumes(const HostStaticData& staticData)
        {
            return { QueryVolumeDescriptor{ SphereGeometry{ staticData.queryRadius },
                CollisionCategories{ joltTestRig::bit(joltTestRig::kCharacter) }, glm::mat4(1.f), joltTestRig::kQueryRouting } };
        }

        static glm::vec3 attachmentOffset(const HostStaticData& staticData) { return staticData.radialOffset; }

        static PhysicsBodyState& bodyStateOf(StateType& state) { return state.bodyState; }
        static const PhysicsBodyState& bodyStateOf(const StateType& state) { return state.bodyState; }

        PhysicsRuntimeBindings bindings;
    };

    struct HostCapsuleDeclaration
    {
        using StateType = HostCapsulePart;
        static constexpr const char* name = "hostCapsule";

        static const PhysicalObjectDescriptor& descriptor()
        {
            static const PhysicalObjectDescriptor capsule = joltTestRig::capsuleTemplate(joltTestRig::bit(joltTestRig::kCharacter),
                joltTestRig::bit(joltTestRig::kWorld) | joltTestRig::bit(joltTestRig::kCharacter), 1).descriptor;
            return capsule;
        }

        static const HostStaticData& staticDataOf(const HostStaticData& staticData) { return staticData; }
        static std::vector<QueryVolumeDescriptor> queryVolumes(const HostStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const HostStaticData&) { return glm::vec3(0.f); }

        static PhysicsBodyState& bodyStateOf(StateType& state) { return state.bodyState; }
        static const PhysicsBodyState& bodyStateOf(const StateType& state) { return state.bodyState; }

        PhysicsRuntimeBindings bindings;
    };

    static_assert(PhysicsDeclaration<HostRadialDeclaration, HostStaticData>);
    static_assert(PhysicsDeclaration<HostCapsuleDeclaration, HostStaticData>);

    using HostComposite = SimulationComposite<HostRadialDeclaration, HostCapsuleDeclaration>;

    class HostSim
    {
    public:
        const HostComposite& getPhysicsComposite() const { return m_composite; }
        HostComposite& editPhysicsComposite() { return m_composite; }

    private:
        HostComposite m_composite;
    };

    constexpr uint8_t kRadialDeclaration = 0u;
    constexpr uint8_t kCapsuleDeclaration = 1u;
    constexpr uint32_t kClientRingDepthTicks = 8u;

    using HostLock = tickDriverFakes::WrapperOverLockUnlockOnly;

    template <typename EngineT>
    using Host = JoltTickWorldHost<HostSim, HostLock, EngineT, CountingLineAllocator<char>>;

    static_assert(TickDriverWorldHost<Host<FakeEngine>>);
    static_assert(TickDriverWorldHost<Host<FakeEngineNoDiagnostics>>);

    StaticShapeDescriptor staticBoxOf(const glm::vec3& centre, const glm::vec3& halfExtents, uint64_t stableKey)
    {
        StaticShapeDescriptor descriptor;
        descriptor.shape = StaticBox{ halfExtents };
        descriptor.localToWorld = glm::translate(glm::mat4(1.f), centre);
        descriptor.categories = CollisionCategories{ joltTestRig::bit(joltTestRig::kWorld) };
        descriptor.blockingCategories = CollisionCategories{ joltTestRig::kAllMappedCategories };
        descriptor.friction = 0.7f;
        descriptor.restitution = 0.3f;
        descriptor.stableKey = stableKey;
        return descriptor;
    }

    StaticWorldDescription importedStatics()
    {
        StaticWorldDescription description;
        description.shapes.push_back(staticBoxOf(glm::vec3(0.f, -0.5f, -0.f), glm::vec3(15.f, 0.5f, 15.f), 1u));
        description.shapes.push_back(staticBoxOf(glm::vec3(0.f, 2.f, -9.f), glm::vec3(0.4f, 2.f, 0.4f), 2u));
        description.shapes.push_back(staticBoxOf(glm::vec3(0.f, 1.f, 9.f), glm::vec3(0.6f, 1.f, 0.2f), 3u));
        return description;
    }

    const std::vector<uint8_t> kImportedPhysicsOnly{ 0u, 1u, 0u };

    JoltTickWorldHostConfig configOf(bool isAuthority, bool runInline, bool contactCounts = false)
    {
        JoltTickWorldHostConfig config;
        config.isAuthority = isAuthority;
        config.runInline = runInline;
        config.clientRingDepthTicks = kClientRingDepthTicks;
        config.gravity = glm::vec3(0.f, -9.8f, -0.f);
        config.contactCounts = contactCounts;
        config.mappedCategories = joltTestRig::kAllMappedCategories;
        config.log = &joltTestRig::logJoltProblems;
        return config;
    }

    template <typename EngineT>
    struct HostRig
    {
        explicit HostRig(const JoltTickWorldHostConfig& config)
            : statics(joltTickWorldHost::splitStatics(importedStatics(), kImportedPhysicsOnly))
            , host(engine)
        {
            host.build(config, statics.queryable, statics.physicsOnly, bodyAdapter, readerAdapter, queryAdapter);
        }

        std::vector<std::string> texts() const
        {
            std::vector<std::string> result;
            for (const LoggedLine& line : engine.lines)
                result.push_back(line.text);
            return result;
        }

        std::vector<std::string> textsStartingWith(const std::string& prefix) const
        {
            std::vector<std::string> result;
            for (const LoggedLine& line : engine.lines)
            {
                if (line.text.rfind(prefix, 0) == 0)
                    result.push_back(line.text);
            }
            return result;
        }

        JoltTickStatics statics;
        EngineT engine;
        Host<EngineT> host;
        std::optional<JoltPhysicsBodyAdapter> bodyAdapter;
        std::optional<JoltPhysicsBodyReaderAdapter> readerAdapter;
        std::optional<JoltSpatialQueryAdapter> queryAdapter;
    };

    struct PublishRecorder
    {
        void operator()(uint32_t slot) { published.push_back(slot); }

        std::vector<uint32_t> published;
    };

    struct BodySlotOccupancyRecorder
    {
        void operator()(uint32_t slot, bool occupied) { pushes.emplace_back(slot, occupied); }

        std::vector<std::pair<uint32_t, bool>> pushes;
    };

    template <typename F>
    auto onOtherThread(F function)
    {
        decltype(function()) result{};
        std::thread worker([&] { result = function(); });
        worker.join();
        return result;
    }

    template <typename RigT>
    bool bindTablesAgree(const RigT& rig)
    {
        const JoltBodyBindTable& stepTable = rig.bodyAdapter->bindTable();
        const JoltPhysicsBodyAdapter* const shadow = rig.host.getDiagnostics().shadowBodyAdapter();
        if (shadow == nullptr)
            return false;
        const JoltBodyBindTable& shadowTable = shadow->bindTable();
        const JoltWorld& stepWorld = rig.host.stepWorld();
        const JoltWorld& shadowWorld = *rig.host.getDiagnostics().shadowWorld();
        for (uint32_t slot = 0u; slot < stepWorld.simulatableSlots(); ++slot)
        {
            for (uint32_t index = 0u; index < stepWorld.bodiesPerSlot(); ++index)
            {
                const BodyId stepBody = JoltPhysicsBodyAdapter::bodyIdOf(stepWorld.slotBodyId(slot, index));
                const BodyId shadowBody = JoltPhysicsBodyAdapter::bodyIdOf(shadowWorld.slotBodyId(slot, index));
                if (stepBody != shadowBody || stepTable.bindingOf(stepBody) != shadowTable.bindingOf(shadowBody))
                    return false;
                if (stepTable.isBound(stepBody)
                    && stepTable.lockedRotationInertiaOf(stepBody) != shadowTable.lockedRotationInertiaOf(shadowBody))
                    return false;
            }
        }
        return true;
    }

    bool nearPose(const glm::vec3& first, const glm::vec3& second)
    {
        return glm::distance(first, second) < 1.0000001e-5f;
    }

    TickOutcome outcomeOf(SimTick tick, StepKind kind)
    {
        TickOutcome outcome;
        outcome.tick = tick;
        outcome.kind = kind;
        return outcome;
    }

    template <typename RigT>
    void saveAndHandOver(RigT& rig, SimTick tick, StepKind kind, bool save = true)
    {
        auto worldLock = rig.host.lockForStep();
        StepScope step(rig.host.threadContext());
        if (save)
            rig.host.stepWorld().saveTick(tick);
        rig.host.afterTick_Step(outcomeOf(tick, kind));
    }

    class HostManager
    {
    public:
        explicit HostManager(float dtSeconds)
            : m_serverClock(dtSeconds)
            , m_dtSeconds(dtSeconds)
        {
        }

        struct Clock
        {
            unsigned int getPredictionTick() const { return predictionTick; }

            unsigned int registerResyncCallback(ClientPredictionClock::ResyncCallback callback)
            {
                callbacks.push_back(std::move(callback));
                return static_cast<unsigned int>(callbacks.size() - 1u);
            }

            void unregisterResyncCallback(unsigned int) {}

            unsigned int predictionTick = 0u;
            unsigned int resimulationTick = 0u;
            std::vector<ClientPredictionClock::ResyncCallback> callbacks;
        };

        bool runsPrediction() const { return true; }
        Clock& editClientClock() { return m_clock; }
        const Clock& getClientClock() const { return m_clock; }
        const ServerTickClock& getServerClock() const { return m_serverClock; }
        ResimGateProbe& editResimGateProbe() { return m_probe; }

        void onGameSimulation(const SimulationUpdateInfo& info)
        {
            if (info.isResimulation())
            {
                ++m_clock.resimulationTick;
                m_lastStep = SimulationTimeStep(m_clock.resimulationTick, true, StepKind::Normal, m_dtSeconds);
                return;
            }
            if (m_clock.predictionTick == m_clock.resimulationTick)
                ++m_clock.resimulationTick;
            ++m_clock.predictionTick;
            m_lastStep = SimulationTimeStep(m_clock.predictionTick, false, StepKind::Normal, m_dtSeconds);
        }

        void onPostGameSimulation(const SimulationUpdateInfo&) {}
        unsigned int onCheckIsSimilar() { return std::numeric_limits<unsigned int>::max(); }

        void prepareResimulation(int32_t, uint32_t simTick) { m_clock.resimulationTick = simTick; }

        uint32_t currentIntegratedTick() const { return m_lastStep.has_value() ? m_lastStep->getTick() : 0u; }
        const std::optional<SimulationTimeStep>& lastIntegratedStep() const { return m_lastStep; }

    private:
        Clock m_clock;
        ServerTickClock m_serverClock;
        ResimGateProbe m_probe;
        float m_dtSeconds;
        std::optional<SimulationTimeStep> m_lastStep;
    };

    template <typename HostT>
    struct HostHooks
    {
        void beforeTick(const UpcomingTick&) {}
        void beforeSimulate(const UpcomingTick&) {}
        void beforePhysics(SimTick) {}
        void afterTick(const TickOutcome& outcome) { host->afterTick_Step(outcome); }

        HostT* host = nullptr;
    };

    template <typename RigT>
    void bindAt(RigT& rig, HostSim& sim, uint64_t key, const char* name, uint32_t slot, unsigned int id, bool isAuthority)
    {
        PublishRecorder publish;
        const std::optional<uint32_t> acquired =
            rig.host.acquire(key, name, isAuthority, isAuthority ? std::nullopt : std::optional<uint32_t>(slot), publish);
        REQUIRE(acquired == std::optional<uint32_t>(slot));
        rig.host.bind(slot, id, sim, HostStaticData{}, [](double) {});
    }
} // namespace

TEST_CASE("JoltTickWorldHost.Build.InlineHostRegistersAVolumeFromTheConfiguringThreadWithoutTheLock", "[JoltTickWorldHost]")
{
    const bool isAuthority = GENERATE(true, false);
    HostRig<FakeEngine> rig(configOf(isAuthority, /*runInline=*/true));
    const TickDriverThreadContext& context = rig.host.threadContext();

    CHECK(context.stepsInline());
    CHECK(context.isOwnerThreadForRole());
    CHECK_FALSE(context.isWorldLockHeldOnThisThread());
    CHECK_FALSE(context.isStepRunningOnThisThread());
    CHECK(rig.queryAdapter->callerMayAccessWorld());
    CHECK_FALSE(rig.host.hasShadowWorld());

    const BodyId root = joltTickWorldHost::slotRootBodyId(rig.host.stepWorld(), rig.host.getDiagnostics().slotTemplate(), 0u);
    const QueryVolumeDescriptor volume{ SphereGeometry{ 0.8f }, CollisionCategories{ joltTestRig::bit(joltTestRig::kCharacter) },
        glm::mat4(1.f), joltTestRig::kQueryRouting };
    const uint32_t volumesBefore = rig.queryAdapter->volumeCount();
    rig.queryAdapter->registerVolume(volume, root);
    CHECK(rig.queryAdapter->volumeCount() == volumesBefore + 1u);

    PublishRecorder publish;
    const std::optional<QueryVolumeId> viz = rig.host.registerVizVolume(volume, 41u, "viz", isAuthority,
        isAuthority ? std::nullopt : std::optional<uint32_t>(3u), std::ref(publish));
    CHECK(viz.has_value());
    CHECK(rig.queryAdapter->volumeCount() == volumesBefore + 2u);
    CHECK(rig.host.slotOf(41u) == std::optional<uint32_t>(isAuthority ? 0u : 3u));
    CHECK(publish.published == (isAuthority ? std::vector<uint32_t>{ 0u } : std::vector<uint32_t>{}));
    CHECK(&rig.host.editVizQuery() == &*rig.queryAdapter);
    CHECK(&rig.host.getVizReader() == &*rig.readerAdapter);
    CHECK(rig.host.getDiagnostics().shadowWindow().vizQueryStep == 1u);
    CHECK(rig.host.getDiagnostics().shadowWindow().vizReaderStep == 1u);
}

TEST_CASE("JoltTickWorldHost.Build.WorkerClientBuildsAShadowWorldWithTheSameStatics", "[JoltTickWorldHost]")
{
    HostRig<FakeEngine> rig(configOf(/*isAuthority=*/false, /*runInline=*/false));
    REQUIRE(rig.host.hasShadowWorld());
    CHECK_FALSE(rig.host.threadContext().stepsInline());

    const std::vector<std::string> built = rig.textsStartingWith("[SimHost.Shadow] built");
    REQUIRE(built.size() == 1u);
    CHECK(built.front()
        == "[SimHost.Shadow] built bodies step=18 shadow=18 sameStatics=1 slots=8 bodiesPerSlot=2 ringDepthTicks=0 tempAllocatorBytes=65536");

    joltTestRig::RuntimeLease lease;
    JoltWorldConfig referenceConfig;
    referenceConfig.simulatableSlots = kMaxSimulatableSlots;
    referenceConfig.slotTemplate = joltTickWorldHost::slotTemplateOf<HostSim>();
    referenceConfig.staticLayers = staticLayerKeysOf(rig.statics.queryable);
    const std::vector<JoltLayerKey> physicsOnlyLayers = staticLayerKeysOf(rig.statics.physicsOnly);
    referenceConfig.staticLayers.insert(referenceConfig.staticLayers.end(), physicsOnlyLayers.begin(), physicsOnlyLayers.end());
    referenceConfig.ringDepthTicks = kClientRingDepthTicks;
    referenceConfig.tempAllocatorBytes = 4u << 20;
    JoltWorld reference(lease.runtime, referenceConfig, &joltTestRig::logJoltProblems);
    JoltStaticWorldBuilder queryableBuilder(reference, &joltTestRig::logJoltProblems);
    queryableBuilder.build(rig.statics.queryable);
    JoltStaticWorldBuilder physicsOnlyBuilder(reference, &joltTestRig::logJoltProblems);
    physicsOnlyBuilder.build(rig.statics.physicsOnly);
    std::vector<JPH::BodyID> referenceStatics = queryableBuilder.stats().bodies;
    referenceStatics.insert(referenceStatics.end(), physicsOnlyBuilder.stats().bodies.begin(), physicsOnlyBuilder.stats().bodies.end());
    REQUIRE(referenceStatics.size() == 2u);

    const JoltWorld& step = rig.host.stepWorld();
    const JoltWorld& shadow = *rig.host.getDiagnostics().shadowWorld();
    uint32_t staticsCompared = 0u;
    for (const JPH::BodyID id : referenceStatics)
    {
        const JPH::RVec3 expected = reference.bodies().GetCenterOfMassPosition(id);
        CHECK(step.bodies().GetCenterOfMassPosition(id) == expected);
        CHECK(shadow.bodies().GetCenterOfMassPosition(id) == expected);
        CHECK(step.bodies().GetObjectLayer(id) == reference.bodies().GetObjectLayer(id));
        ++staticsCompared;
    }
    CHECK(staticsCompared == 2u);
}

TEST_CASE("JoltTickWorldHost.Bind.BindReleaseAndRebindLeaveTheTwoBindTablesEqual", "[JoltTickWorldHost]")
{
    HostRig<FakeEngine> rig(configOf(/*isAuthority=*/false, /*runInline=*/false));
    REQUIRE(rig.host.hasShadowWorld());
    CHECK(bindTablesAgree(rig));

    HostSim first;
    std::vector<double> waits;
    PublishRecorder publish;
    REQUIRE(rig.host.acquire(101u, "first", false, std::optional<uint32_t>(2u), std::ref(publish)) == std::optional<uint32_t>(2u));
    rig.host.bind(2u, 7u, first, HostStaticData{}, [&waits](double seconds) { waits.push_back(seconds); });
    CHECK(waits == std::vector<double>{ 0.0 });
    CHECK(publish.published.empty());
    CHECK(bindTablesAgree(rig));
    CHECK(rig.bodyAdapter->bindTable().isBound(first.getPhysicsComposite().get<HostCapsuleDeclaration>().bindings.ownBodyId));
    CHECK(first.getPhysicsComposite().get<HostRadialDeclaration>().bindings.queryVolumeIds.size() == 1u);
    CHECK(first.getPhysicsComposite().get<HostRadialDeclaration>().bindings.parentBodyId
        == first.getPhysicsComposite().get<HostCapsuleDeclaration>().bindings.ownBodyId);

    BodySlotOccupancyRecorder occupancy;
    CHECK(rig.host.release(101u, 7u, std::ref(occupancy)) == std::optional<uint32_t>(2u));
    CHECK(occupancy.pushes == std::vector<std::pair<uint32_t, bool>>{ { 2u, false } });
    CHECK(bindTablesAgree(rig));
    CHECK_FALSE(rig.bodyAdapter->bindTable().isBound(first.getPhysicsComposite().get<HostCapsuleDeclaration>().bindings.ownBodyId));
    CHECK(rig.host.release(101u, 7u, std::ref(occupancy)) == std::nullopt);
    CHECK(occupancy.pushes.size() == 1u);

    HostSim second;
    REQUIRE(rig.host.acquire(102u, "second", false, std::optional<uint32_t>(2u), std::ref(publish)) == std::optional<uint32_t>(2u));
    rig.host.bind(2u, 8u, second, HostStaticData{}, [&waits](double seconds) { waits.push_back(seconds); });
    CHECK(bindTablesAgree(rig));

    CHECK(rig.host.getDiagnostics().shadowBindCompares() == 3u);
    CHECK(rig.host.getDiagnostics().shadowBindMismatches() == 0u);
    CHECK(rig.textsStartingWith("[SimHost.Shadow] bindCompare") == std::vector<std::string>{
        "[SimHost.Shadow] bindCompare after=bind id=7 slot=2 equal=1 boundBodies=2 compares=1 mismatches=0",
        "[SimHost.Shadow] bindCompare after=release id=0 slot=2 equal=1 boundBodies=0 compares=2 mismatches=0",
        "[SimHost.Shadow] bindCompare after=bind id=8 slot=2 equal=1 boundBodies=2 compares=3 mismatches=0",
    });
}

TEST_CASE("JoltTickWorldHost.Bind.TheAuthorityPublishesOnlyANewAssignment", "[JoltTickWorldHost]")
{
    HostRig<FakeEngine> rig(configOf(/*isAuthority=*/true, /*runInline=*/false));
    PublishRecorder publish;
    CHECK(rig.host.acquire(11u, "a", true, std::nullopt, std::ref(publish)) == std::optional<uint32_t>(0u));
    CHECK(rig.host.acquire(11u, "a", true, std::nullopt, std::ref(publish)) == std::optional<uint32_t>(0u));
    CHECK(rig.host.acquire(12u, "b", true, std::nullopt, std::ref(publish)) == std::optional<uint32_t>(1u));
    CHECK(publish.published == std::vector<uint32_t>{ 0u, 1u });
    CHECK(rig.host.slotsInUse() == 2u);
}

TEST_CASE("JoltTickWorldHost.Shadow.HandOverPublishesOnlyOnFrontierSavingStepsAndRestoresOncePerSequence", "[JoltTickWorldHost]")
{
    HostRig<FakeEngine> rig(configOf(/*isAuthority=*/false, /*runInline=*/false));
    HostSim sim;
    bindAt(rig, sim, 5u, "pawn", 4u, 9u, false);
    const auto diagnostics = rig.host.getDiagnostics();

    saveAndHandOver(rig, 1u, StepKind::Normal);
    CHECK(diagnostics.shadowSlotsPublished() == 1u);
    saveAndHandOver(rig, 2u, StepKind::Stall);
    CHECK(diagnostics.shadowSlotsPublished() == 1u);
    saveAndHandOver(rig, 3u, StepKind::Skip);
    CHECK(diagnostics.shadowSlotsPublished() == 2u);
    saveAndHandOver(rig, 4u, StepKind::Normal, /*save=*/false);
    CHECK(diagnostics.shadowSlotsPublished() == 2u);
    CHECK(diagnostics.shadowSlotsMissing() == 1u);
    saveAndHandOver(rig, 5u, StepKind::HardResync);
    CHECK(diagnostics.shadowSlotsPublished() == 3u);

    std::vector<std::pair<double, bool>> notes;
    const auto note = [&notes](double seconds, bool refused) { notes.emplace_back(seconds, refused); };

    rig.engine.advanceFrame();
    rig.host.restore_GameThread(rig.engine.frameNumber(), note);
    CHECK(diagnostics.shadowWindow().restores == 1u);
    CHECK(diagnostics.shadowWindow().lastRestoredTick == 5u);
    CHECK(diagnostics.shadowRestoredSequence() == 3u);

    rig.engine.advanceFrame();
    rig.host.restore_GameThread(rig.engine.frameNumber(), note);
    CHECK(diagnostics.shadowWindow().restores == 1u);
    CHECK(diagnostics.shadowWindow().unchanged == 1u);

    saveAndHandOver(rig, 6u, StepKind::Normal);
    rig.engine.advanceFrame();
    rig.host.restore_GameThread(rig.engine.frameNumber(), note);
    CHECK(diagnostics.shadowWindow().restores == 2u);
    CHECK(diagnostics.shadowWindow().lastRestoredTick == 6u);
    CHECK(diagnostics.shadowWindow().refused == 0u);
    CHECK(diagnostics.shadowWindow().hashEqual == 2u);
    CHECK(diagnostics.shadowWindow().frames == 3u);
    CHECK(notes == std::vector<std::pair<double, bool>>{ { 0.0, false }, { 0.0, false } });
    CHECK(rig.host.stepWorld().liveStateHash() == rig.host.getDiagnostics().shadowWorld()->liveStateHash());
}

TEST_CASE("JoltTickWorldHost.Shadow.InlineHostHasNoHandOverAndNoRestore", "[JoltTickWorldHost]")
{
    HostRig<FakeEngine> rig(configOf(/*isAuthority=*/false, /*runInline=*/true));
    saveAndHandOver(rig, 1u, StepKind::Normal);
    rig.host.restore_GameThread(rig.engine.frameNumber());
    CHECK(rig.host.getDiagnostics().shadowSlotsPublished() == 0u);
    CHECK(rig.host.getDiagnostics().shadowWindow().frames == 0u);
}

TEST_CASE("JoltTickWorldHost.Shadow.OverwriteWritesTheRenderPosesIntoTheRestoredShadow", "[JoltTickWorldHost]")
{
    const auto run = [](auto& rig, uint32_t expectedVisits)
    {
        HostSim sim;
        bindAt(rig, sim, 5u, "pawn", 4u, 9u, false);
        saveAndHandOver(rig, 1u, StepKind::Normal);
        rig.engine.advanceFrame();
        rig.host.restore_GameThread(rig.engine.frameNumber());

        RenderSnapshotT<16> poses;
        poses.bodyCount = 2u;
        poses.bodies[0] = RenderBody{ 9u, kRadialDeclaration, 1u, 0u, 0u, glm::vec3(0.2f, 0.3f, -0.1f),
            glm::angleAxis(0.5f, glm::vec3(-0.f, -1.f, 0.f)) };
        poses.bodies[1] = RenderBody{ 9u, kCapsuleDeclaration, 0u, 0u, 1u, glm::vec3(0.5f, 1.6f, -0.4f) };

        std::vector<std::pair<uint32_t, glm::vec3>> visits;
        rig.host.overwriteShadowPoses_GameThread(poses,
            [](auto&& visit) { visit(9u, 4u); },
            [&visits](unsigned int id, uint32_t declaration, const RenderBody& pose, auto&& readBack)
            {
                const glm::mat4 queried = readBack();
                CHECK(id == 9u);
                CHECK(nearPose(glm::vec3(queried[3]), pose.position));
                visits.emplace_back(declaration, glm::vec3(queried[3]));
            });

        const JoltPhysicsBodyAdapter& shadowBodies = *rig.host.getDiagnostics().shadowBodyAdapter();
        const JoltWorld& shadow = *rig.host.getDiagnostics().shadowWorld();
        const glm::mat4 radial = shadowBodies.getBodyTransform(JoltPhysicsBodyAdapter::bodyIdOf(shadow.slotBodyId(4u, kRadialDeclaration)));
        const glm::mat4 capsule = shadowBodies.getBodyTransform(JoltPhysicsBodyAdapter::bodyIdOf(shadow.slotBodyId(4u, kCapsuleDeclaration)));
        CHECK(nearPose(glm::vec3(radial[3]), glm::vec3(0.2f, 0.3f, -0.1f)));
        CHECK(nearPose(glm::vec3(capsule[3]), glm::vec3(0.5f, 1.6f, -0.4f)));
        CHECK(visits.size() == expectedVisits);
    };

    SECTION("diagnostics on: the visitor sees each written body")
    {
        HostRig<FakeEngine> rig(configOf(/*isAuthority=*/false, /*runInline=*/false));
        run(rig, 2u);
    }
    SECTION("diagnostics off: the visitor is not called")
    {
        HostRig<FakeEngineNoDiagnostics> rig(configOf(/*isAuthority=*/false, /*runInline=*/false));
        run(rig, 0u);
    }
}

TEST_CASE("JoltTickWorldHost.Contacts.OneRunTickWithAFiveTickReplayRecordsSixContactSteps", "[JoltTickWorldHost]")
{
    HostRig<FakeEngine> rig(configOf(/*isAuthority=*/false, /*runInline=*/true, /*contactCounts=*/true));
    rig.engine.minimumLevel = HostLogLevel::Verbose;
    REQUIRE(rig.host.getDiagnostics().contactStepsReserved() == kClientRingDepthTicks + 1u);

    HostManager manager(joltTestRig::kDt);
    rig.host.bindIntegratedTickSource(manager);
    using RigHost = Host<FakeEngine>;
    HostHooks<RigHost> hooks{ &rig.host };
    SimulationStepDriver<HostManager, JoltWorld, HostHooks<RigHost>> driver(manager, rig.host.stepWorld(), hooks,
        StepDriverConfig{ joltTestRig::kDt, ResimPolicy::Always, 5u });

    HostSim sim;
    bindAt(rig, sim, 5u, "pawn", 0u, 3u, false);
    driver.noteBodySlotOccupancy(0u, true);
    const BodyId capsule = sim.getPhysicsComposite().get<HostCapsuleDeclaration>().bindings.ownBodyId;
    const BodyId radial = sim.getPhysicsComposite().get<HostRadialDeclaration>().bindings.ownBodyId;
    {
        auto worldLock = rig.host.lockForStep();
        rig.bodyAdapter->setBodyTransform(capsule, glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.95f, -0.f)));
        rig.bodyAdapter->setBodyTransform(radial, glm::translate(glm::mat4(1.f), glm::vec3(0.f, 4.f, -3.f)));
    }

    uint64_t physicsStep = 0u;
    const auto runTick = [&]()
    {
        auto worldLock = rig.host.lockForStep();
        StepScope step(rig.host.threadContext());
        const TickOutcome outcome = driver.runTick(++physicsStep);
        rig.host.afterRunTick_Step(physicsStep, outcome);
        return outcome;
    };

    for (int forward = 0; forward < 6; ++forward)
    {
        const TickOutcome outcome = runTick();
        REQUIRE(outcome.replayedTicks == 0u);
    }

    const size_t linesBefore = rig.engine.lines.size();
    const uint64_t allocationsBefore = g_lineAllocations.load();
    const TickOutcome replay = runTick();
    REQUIRE(replay.replayedTicks == 5u);
    CHECK(g_lineAllocations.load() == allocationsBefore);
    CHECK(rig.host.getDiagnostics().contactStepsReserved() == kClientRingDepthTicks + 1u);

    std::vector<std::string> lines;
    for (size_t index = linesBefore; index < rig.engine.lines.size(); ++index)
    {
        CHECK(rig.engine.lines[index].level == HostLogLevel::Verbose);
        CHECK(rig.engine.lines[index].text.size() < joltTickWorldHost::kReservedLineChars);
        lines.push_back(rig.engine.lines[index].text);
    }
    REQUIRE(lines.size() == 6u);
    for (size_t index = 0u; index < lines.size(); ++index)
    {
        const std::string tick = "tick=" + std::to_string(2u + index) + " step=7 ";
        CHECK(lines[index].rfind("[Verbose][SimHost.Contacts] " + tick, 0) == 0u);
        CHECK(lines[index].find(index + 1u < lines.size() ? " replay=1 " : " replay=0 ") != std::string::npos);
    }
    CHECK(lines[1] == "[Verbose][SimHost.Contacts] tick=3 step=7 worldStep=8 replay=1 manifolds=1 added=0 rootRoot=0 "
                      "rootChild=0 rootStatic=1 childChild=0 childStatic=0 staticStatic=0 ids=3:0/0/1/0/0:0x0 idsDropped=0 "
                      "declarationsDropped=0");

    SECTION("at Log the records are drained and nothing is formatted")
    {
        rig.engine.minimumLevel = HostLogLevel::Log;
        const size_t quietBefore = rig.engine.lines.size();
        const TickOutcome quiet = runTick();
        CHECK(quiet.replayedTicks == 5u);
        CHECK(rig.engine.lines.size() == quietBefore);
        CHECK(g_lineAllocations.load() == allocationsBefore);
    }
}

TEST_CASE("JoltTickWorldHost.Predicates.RefuseANonStepNonLockCallerInWorkerMode", "[JoltTickWorldHost]")
{
    HostRig<FakeEngine> rig(configOf(/*isAuthority=*/false, /*runInline=*/false));
    const TickDriverThreadContext& context = rig.host.threadContext();
    const TickDriverThreadContext::AccessPredicate step = context.stepWorldPredicate();
    const TickDriverThreadContext::AccessPredicate shadow = context.shadowPredicate();

    const uint64_t ownerThreadAccessesBefore = TickDriverThreadContext::stepQueryOwnerThreadAccessesOnThisThread();
    CHECK_FALSE(step());
    CHECK_FALSE(rig.queryAdapter->callerMayAccessWorld());
    CHECK(TickDriverThreadContext::stepQueryOwnerThreadAccessesOnThisThread() == ownerThreadAccessesBefore + 2u);
    CHECK_FALSE(onOtherThread([step] { return step(); }));
    {
        auto worldLock = rig.host.lockForStep();
        CHECK(step());
        CHECK(rig.queryAdapter->callerMayAccessWorld());
        CHECK_FALSE(onOtherThread([step] { return step(); }));
    }
    {
        StepScope running(context);
        CHECK(step());
    }
    CHECK_FALSE(step());

    CHECK(shadow());
    CHECK(rig.host.getDiagnostics().shadowWorld() != nullptr);
    CHECK_FALSE(onOtherThread([shadow] { return shadow(); }));
    CHECK(&rig.host.editVizQuery() != &*rig.queryAdapter);
    CHECK(rig.host.getDiagnostics().shadowWindow().vizQueryShadow == 1u);
}

TEST_CASE("JoltTickWorldHost.Lines.MatchTheGoldens", "[JoltTickWorldHost]")
{
    SECTION("an inline authority")
    {
        HostRig<FakeEngine> rig(configOf(/*isAuthority=*/true, /*runInline=*/false));
        HostSim sim;
        bindAt(rig, sim, 21u, "BP_Brawler_C_0", 0u, 4u, true);
        BodySlotOccupancyRecorder occupancy;
        rig.host.release(21u, 4u, std::ref(occupancy));
        REQUIRE(rig.texts() == std::vector<std::string>{
            "[SimHost.Mode] role=Authority stepping=inline viz=stepWorld",
            "[SimHost.World] slots=8 bodiesPerSlot=2 ringDepthTicks=0 statics=2 physicsOnlyStatics=1 staticBodies=1 physicsOnlyBodies=1",
            "[SimHost.Contacts] installed=0 stepListener=null shadowListener=none reserved=0",
            "[SimHost.Slot] assigned role=Authority owner=BP_Brawler_C_0 slot=0 lowestFreeLocally=0",
            "[SimHost.Bind] bind id=4 slot=0 body0=1 boundBodies=2 slotsInUse=1",
            "[SimHost.Bind] release id=4 slot=0 boundBodies=0 slotsInUse=0",
        });
        CHECK(rig.engine.lines[0].level == HostLogLevel::Warning);
        CHECK(rig.engine.lines[1].level == HostLogLevel::Warning);
        CHECK(rig.engine.lines[2].level == HostLogLevel::Warning);
        CHECK(rig.engine.lines[4].level == HostLogLevel::Warning);
        CHECK(rig.engine.lines[5].level == HostLogLevel::Warning);
    }

    SECTION("an inline client with the contact counts")
    {
        HostRig<FakeEngine> rig(configOf(/*isAuthority=*/false, /*runInline=*/true, /*contactCounts=*/true));
        REQUIRE(rig.texts() == std::vector<std::string>{
            "[SimHost.Mode] role=Client stepping=inline viz=stepWorld",
            "[SimHost.World] slots=8 bodiesPerSlot=2 ringDepthTicks=8 statics=2 physicsOnlyStatics=1 staticBodies=1 physicsOnlyBodies=1",
            "[SimHost.Contacts] installed=1 stepListener=set shadowListener=none reserved=9",
        });
    }

    SECTION("a worker client: shadow, bind mirror, mismatch and the window")
    {
        HostRig<FakeEngine> rig(configOf(/*isAuthority=*/false, /*runInline=*/false));
        HostSim first;
        bindAt(rig, first, 31u, "BP_Brawler_C_1", 6u, 12u, false);

        const BodyId stray = JoltPhysicsBodyAdapter::bodyIdOf(rig.host.stepWorld().slotBodyId(1u, kRadialDeclaration));
        rig.bodyAdapter->bindTable().bind(stray, JoltBodyBinding{ 99u, kRadialDeclaration }, glm::vec3(1e-4f, 1e-4f, 1e-4f));
        HostSim second;
        bindAt(rig, second, 32u, "BP_Brawler_C_2", 3u, 13u, false);

        REQUIRE(rig.texts() == std::vector<std::string>{
            "[SimHost.Mode] role=Client stepping=worker viz=shadowWorld",
            "[SimHost.World] slots=8 bodiesPerSlot=2 ringDepthTicks=8 statics=2 physicsOnlyStatics=1 staticBodies=1 physicsOnlyBodies=1",
            "[SimHost.Shadow] built bodies step=18 shadow=18 sameStatics=1 slots=8 bodiesPerSlot=2 ringDepthTicks=0 tempAllocatorBytes=65536",
            "[SimHost.Contacts] installed=0 stepListener=null shadowListener=null reserved=0",
            "[SimHost.Slot] assigned role=Client owner=BP_Brawler_C_1 slot=6 lowestFreeLocally=0",
            "[SimHost.Bind] bind id=12 slot=6 body0=13 boundBodies=2 slotsInUse=1",
            "[SimHost.Shadow] bindCompare after=bind id=12 slot=6 equal=1 boundBodies=2 compares=1 mismatches=0",
            "[SimHost.Slot] assigned role=Client owner=BP_Brawler_C_2 slot=3 lowestFreeLocally=0",
            "[SimHost.Bind] bind id=13 slot=3 body0=7 boundBodies=5 slotsInUse=2",
            "[SimHost.Shadow] bindCompare after=bind id=13 slot=3 equal=0 boundBodies=5 differing=1 sameShape=1 sameResults=1 "
            "compares=2 mismatches=1: the shadow world's bind table differs from the step world's",
        });
        CHECK(rig.engine.lines[6].level == HostLogLevel::Log);
        CHECK(rig.engine.lines[9].level == HostLogLevel::Warning);

        saveAndHandOver(rig, 1u, StepKind::Normal);
        for (uint32_t frame = 0u; frame < joltTickWorldHost::kShadowWindowFrames; ++frame)
        {
            rig.engine.advanceFrame();
            rig.host.restore_GameThread(rig.engine.frameNumber());
        }
        const std::vector<std::string> firstWindow = rig.textsStartingWith("[SimHost.Shadow] role=Client");
        REQUIRE(firstWindow.size() == 1u);

        rig.host.editVizQuery();
        rig.host.editVizQuery();
        rig.host.getVizReader();
        const uint64_t shadowAccessesBefore = TickDriverThreadContext::shadowQueryAccessesOnThisThread();
        PublishRecorder publish;
        const QueryVolumeDescriptor volume{ SphereGeometry{ 0.8f }, CollisionCategories{ joltTestRig::bit(joltTestRig::kCharacter) },
            glm::mat4(1.f), joltTestRig::kQueryRouting };
        CHECK(rig.host.registerVizVolume(volume, 31u, "BP_Brawler_C_1", false, std::optional<uint32_t>(6u), std::ref(publish)).has_value());
        CHECK(rig.host.editVizQuery().callerMayAccessWorld());
        const uint64_t shadowAccesses = TickDriverThreadContext::shadowQueryAccessesOnThisThread() - shadowAccessesBefore;
        CHECK(shadowAccesses >= 1u);
        CHECK_FALSE(rig.queryAdapter->callerMayAccessWorld());
        saveAndHandOver(rig, 2u, StepKind::Normal);
        saveAndHandOver(rig, 3u, StepKind::Normal, /*save=*/false);
        for (uint32_t frame = 0u; frame < joltTickWorldHost::kShadowWindowFrames; ++frame)
        {
            rig.engine.advanceFrame();
            rig.host.restore_GameThread(rig.engine.frameNumber());
        }
        const std::vector<std::string> windows = rig.textsStartingWith("[SimHost.Shadow] role=Client");
        REQUIRE(windows.size() == 2u);
        CHECK(windows[1] == "[SimHost.Shadow] role=Client frames=600 restores=1 unchanged=599 refused=0 published=1 missing=1 "
                            "hashEqual=1 hashDiffer=0 occupiedSlots=0 lastTick=2 maxRestoreMs=0.0000 vizQuery=[shadow:3 step:0] "
                            "vizReader=[shadow:1 step:0] shadowQueryAccesses=" + std::to_string(shadowAccesses)
                            + " stepQueryGameThreadAccesses=1 bindCompares=2 bindMismatches=1 drops=0");
    }

    SECTION("diagnostics off: no bind compare and no window")
    {
        HostRig<FakeEngineNoDiagnostics> rig(configOf(/*isAuthority=*/false, /*runInline=*/false));
        HostSim sim;
        bindAt(rig, sim, 31u, "BP_Brawler_C_1", 6u, 12u, false);
        saveAndHandOver(rig, 1u, StepKind::Normal);
        for (uint32_t frame = 0u; frame < joltTickWorldHost::kShadowWindowFrames; ++frame)
        {
            rig.engine.advanceFrame();
            rig.host.restore_GameThread(rig.engine.frameNumber());
        }
        CHECK(rig.textsStartingWith("[SimHost.Shadow] bindCompare").empty());
        CHECK(rig.textsStartingWith("[SimHost.Shadow] role=Client").empty());
        CHECK(rig.host.getDiagnostics().shadowBindCompares() == 0u);
        CHECK(rig.host.getDiagnostics().shadowWindow().restores == 1u);
        CHECK(rig.host.getDiagnostics().shadowWindow().hashEqual == 0u);
        CHECK(rig.textsStartingWith("[SimHost.Shadow] built").size() == 1u);
    }
}

TEST_CASE("JoltTickWorldHost.Statics.SplitByThePhysicsOnlyFlags", "[JoltTickWorldHost]")
{
    const JoltTickStatics statics = joltTickWorldHost::splitStatics(importedStatics(), kImportedPhysicsOnly);
    REQUIRE(statics.queryable.shapes.size() == 2u);
    REQUIRE(statics.physicsOnly.shapes.size() == 1u);
    CHECK(statics.queryable.shapes[0].stableKey == 1u);
    CHECK(statics.queryable.shapes[1].stableKey == 3u);
    CHECK(statics.physicsOnly.shapes[0].stableKey == 2u);
}

#endif // WITH_LOW_LEVEL_TESTS
