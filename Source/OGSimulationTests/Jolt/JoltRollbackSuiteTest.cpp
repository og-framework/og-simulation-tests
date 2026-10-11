// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltRollbackSuiteRig.h"

#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

#include <cstdio>
#include <random>

using namespace joltSuite;

namespace
{
    constexpr uint32_t kFirstId = 101;
    constexpr uint32_t kCorrectedSlot = 2;

    void joinAll(DriverRig& run, ReferenceRig& reference, uint32_t slots = kSlots)
    {
        for (uint32_t slot = 0; slot < slots; ++slot)
        {
            run.join(slot, kFirstId + slot);
            reference.join(slot, kFirstId + slot);
        }
    }
} // namespace

TEST_CASE("JoltSuite.AlwaysResimForAThousandTicksMatchesTheOneStepPerTickReference", "[Jolt]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::Always;
    config.depthTicks = 8;
    DriverRig run(lease.runtime, config);
    ReferenceRig reference(lease.runtime, config);
    joinAll(run, reference);
    REQUIRE(sameBytes(fullState(run.scene.world), fullState(reference.scene.world)));

    Lockstep lockstep{ run, reference };
    lockstep.ticks(1000);

    const Totals& totals = lockstep.totals;
    REQUIRE(countOf(totals, StepKind::Normal) == 1000u);
    REQUIRE(totals.granted == 1000u - (config.depthTicks + 1u));
    REQUIRE(totals.refused == 0u);
    REQUIRE(run.driver.getDiagnostics().replayedTicks() == totals.granted * config.depthTicks);
    REQUIRE(run.driver.getDiagnostics().occupancyTimelineMisses() == 0u);
    REQUIRE(run.observed.failedRestores == 0u);
    REQUIRE(reference.failedRestores == 0u);
    REQUIRE(run.present() == 1000u);
    REQUIRE(run.pushProbe.check.pushes == totals.granted);
    REQUIRE(run.pushProbe.check.mismatches == 0u);
    REQUIRE(sameBytes(fullState(run.scene.world), fullState(reference.scene.world)));
    REQUIRE(run.scene.world.stateHash(1000u) == reference.scene.world.stateHash(1000u));
    REQUIRE(totals.maxSavedTickBytes > 48u * 100u);
    REQUIRE(totals.maxSavedTickBytes < JoltWorldConfig{}.ringSlotBytes);
    printSummary("A always depth 8", totals, run);

    DriverRigConfig quietConfig = config;
    quietConfig.policy = ResimPolicy::OnRequest;
    DriverRig quiet(lease.runtime, quietConfig);
    for (uint32_t slot = 0; slot < kSlots; ++slot)
        quiet.join(slot, kFirstId + slot);
    for (int index = 0; index < 1000; ++index)
        REQUIRE(quiet.tick().replayedTicks == 0u);
    const bool alwaysEqualsNoResim = sameBytes(fullState(run.scene.world), fullState(quiet.scene.world));
    const bool alwaysEqualsNoResimHash = run.scene.world.liveStateHash() == quiet.scene.world.liveStateHash();
    std::printf("[JoltSuite] A always depth 8 vs a no-resim run of the same 1000 ticks: bytesEqual=%d hashEqual=%d simStatesEqual=%d\n",
        alwaysEqualsNoResim ? 1 : 0, alwaysEqualsNoResimHash ? 1 : 0, sameSimStates(run.storage, quiet.storage) ? 1 : 0);
}

TEST_CASE("JoltSuite.AlwaysResimsOfDepthTwelveAndTwentyAreBitExactAgainstTheReference", "[Jolt]")
{
    const uint32_t depth = GENERATE(12u, 20u);
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::Always;
    config.depthTicks = depth;
    DriverRig run(lease.runtime, config);
    ReferenceRig reference(lease.runtime, config);
    joinAll(run, reference);

    Lockstep lockstep{ run, reference };
    lockstep.ticks(300);

    const Totals& totals = lockstep.totals;
    REQUIRE(countOf(totals, StepKind::Normal) == 300u);
    REQUIRE(totals.granted == 300u - (depth + 1u));
    REQUIRE(run.driver.getDiagnostics().replayedTicks() == totals.granted * depth);
    REQUIRE(run.observed.failedRestores == 0u);
    REQUIRE(run.pushProbe.check.mismatches == 0u);
    printSummary(depth == 12u ? "B always depth 12" : "B always depth 20", totals, run);
}

TEST_CASE("JoltSuite.LinearCastCharactersAreBitExactUnderAlwaysResims", "[Jolt]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::Always;
    config.depthTicks = 12;
    config.scene.factoryOptions.linearCastCharacters = true;
    DriverRig run(lease.runtime, config);
    ReferenceRig reference(lease.runtime, config);
    joinAll(run, reference);
    REQUIRE(run.scene.world.bodies().GetMotionQuality(run.scene.world.slotBodyId(0, kTemplateCapsule)) == JPH::EMotionQuality::LinearCast);

    Lockstep lockstep{ run, reference };
    lockstep.ticks(300);
    REQUIRE(lockstep.totals.granted == 300u - (config.depthTicks + 1u));
    REQUIRE(run.observed.failedRestores == 0u);
    printSummary("LinearCast characters, always depth 12", lockstep.totals, run);
}

TEST_CASE("JoltSuite.CorrectionDrivenResimsUnderTheShippedPolicyMatchTheReference", "[Jolt]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::OnRequest;
    DriverRig run(lease.runtime, config);
    REQUIRE(run.manager.getTimeConfig().resimTriggerPolicy == TimeConfig::ResimTriggerPolicy::OnDisagreement);
    REQUIRE(run.manager.getTimeConfig().rollbackWindowTicks == 12);
    ReferenceRig reference(lease.runtime, config);
    joinAll(run, reference);

    Lockstep lockstep{ run, reference };
    lockstep.ticks(60);
    REQUIRE(lockstep.totals.granted == 0u);

    auto correctAt = [&](uint32_t depth, bool expectResim) {
        const SimTick anchor = run.present() - depth;
        const uint32_t id = run.idOfSlot[kCorrectedSlot];
        REQUIRE(reference.states.count(anchor) == 1u);
        REQUIRE(reference.states.at(anchor).count(id) == 1u);
        const ProbeState predicted = reference.states.at(anchor).at(id);
        const ProbeState corrected = perturbed(predicted);
        REQUIRE_FALSE(corrected.isSimilarTo(predicted));
        lockstep.inject(kCorrectedSlot, anchor, corrected, expectResim);
        return lockstep.tick();
    };

    const TickOutcome twelve = correctAt(12u, true);
    REQUIRE(twelve.replayedTicks == 12u);
    REQUIRE_FALSE(twelve.resimRefused);
    lockstep.ticks(30);

    REQUIRE(run.deepAnchorSkips == 0u);
    const TickOutcome twenty = correctAt(20u, false);
    REQUIRE(twenty.replayedTicks == 0u);
    REQUIRE_FALSE(twenty.resimRefused);
    REQUIRE(run.deepAnchorSkips == 1u);
    lockstep.ticks(30);
    REQUIRE(run.deepAnchorSkips == 31u);

    REQUIRE(lockstep.totals.granted == 1u);
    REQUIRE(run.pushProbe.check.pushes == 1u);
    REQUIRE(run.pushProbe.check.mismatches == 0u);
    REQUIRE(run.observed.failedRestores == 0u);
    printSummary("B correction-driven depth 12 (granted) then 20 (beyond the shipped ceiling)", lockstep.totals, run);
}

TEST_CASE("JoltSuite.StallSkipAndHardResyncInsideReplayWindowsAgreeWithTheReference", "[Jolt]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::Always;
    config.depthTicks = 12;
    DriverRig run(lease.runtime, config);
    ReferenceRig reference(lease.runtime, config);
    joinAll(run, reference);

    Lockstep lockstep{ run, reference };
    lockstep.ticks(40);

    run.manager.editClientClock().requestInputDelayIncreaseStall(1);
    REQUIRE(lockstep.tick().kind == StepKind::Stall);
    lockstep.ticks(10);

    run.manager.editClientClock().requestInputDelayIncreaseStall(3);
    uint64_t tierStalls = 0;
    for (int index = 0; index < 10 && tierStalls < 3u; ++index)
    {
        if (lockstep.tick().kind == StepKind::Stall)
            ++tierStalls;
    }
    REQUIRE(tierStalls == 3u);
    lockstep.ticks(20);

    auto skipOnce = [&]() {
        for (int index = 0; index < 16; ++index)
        {
            if (lockstep.tick(8).kind == StepKind::Skip)
                return true;
        }
        return false;
    };
    REQUIRE(skipOnce());
    lockstep.ticks(20);

    const SimTick presentBeforeResync = run.present();
    const TickOutcome resync = lockstep.tick(500);
    REQUIRE(resync.hardResync);
    REQUIRE(resync.tick > presentBeforeResync + 100u);
    const uint64_t refusedBefore = lockstep.totals.refused;
    lockstep.ticks(30);
    REQUIRE(lockstep.totals.refused - refusedBefore == config.depthTicks);
    for (size_t index = lockstep.totals.refusedAtPresent.size() - config.depthTicks; index < lockstep.totals.refusedAtPresent.size(); ++index)
    {
        const SimTick present = lockstep.totals.refusedAtPresent[index];
        REQUIRE(present >= resync.tick);
        REQUIRE(present < resync.tick + config.depthTicks);
    }

    REQUIRE(skipOnce());
    lockstep.ticks(20);

    const Totals& totals = lockstep.totals;
    REQUIRE(countOf(totals, StepKind::Stall) >= 4u);
    REQUIRE(countOf(totals, StepKind::Skip) >= 2u);
    REQUIRE(totals.hardResyncs == 1u);
    REQUIRE(totals.refused == config.depthTicks);
    REQUIRE(run.driver.getDiagnostics().refusedResims() == config.depthTicks);
    REQUIRE(run.driver.getDiagnostics().hardResyncInvalidations() == 1u);
    REQUIRE(run.driver.getDiagnostics().skipBackfills() == countOf(totals, StepKind::Skip));
    REQUIRE(run.observed.failedRestores == 0u);
    REQUIRE(reference.failedRestores == 0u);
    REQUIRE(run.pushProbe.check.mismatches == 0u);
    printSummary("B2 stall/skip/hardResync depth 12", totals, run);
}

TEST_CASE("JoltSuite.ACorrectionAtASkipsBackfilledTickReplaysFromThePreStepWorld", "[Jolt]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::OnRequest;
    DriverRig run(lease.runtime, config);
    ReferenceRig reference(lease.runtime, config);
    joinAll(run, reference);
    Lockstep lockstep{ run, reference };
    lockstep.ticks(40);

    TickOutcome skip;
    for (int index = 0; index < 16 && skip.kind != StepKind::Skip; ++index)
        skip = lockstep.tick(8);
    REQUIRE(skip.kind == StepKind::Skip);
    REQUIRE(run.scene.world.hasTick(skip.tick - 1u));
    REQUIRE(run.driver.getDiagnostics().skipBackfills() == 1u);

    const SimTick backfilled = skip.tick - 1u;
    const uint32_t id = run.idOfSlot[kCorrectedSlot];
    REQUIRE(reference.states.at(backfilled).count(id) == 1u);
    lockstep.inject(kCorrectedSlot, backfilled, perturbed(reference.states.at(backfilled).at(id)));
    const TickOutcome outcome = lockstep.tick();
    REQUIRE(outcome.replayedTicks == 1u);
    lockstep.ticks(10);
    REQUIRE(lockstep.totals.granted == 1u);
    REQUIRE(run.observed.failedRestores == 0u);
    printSummary("B2 correction at a skip's backfilled tick", lockstep.totals, run);
}

TEST_CASE("JoltSuite.EveryBodyEqualsItsPushedStateBeforeTheFirstReplayStep", "[Jolt]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::OnRequest;
    DriverRig run(lease.runtime, config);
    ReferenceRig reference(lease.runtime, config);
    joinAll(run, reference);
    Lockstep lockstep{ run, reference };
    lockstep.ticks(40);

    const SimTick anchor = run.present() - 12u;
    const uint32_t id = run.idOfSlot[kCorrectedSlot];
    lockstep.inject(kCorrectedSlot, anchor, perturbed(reference.states.at(anchor).at(id)));
    REQUIRE(lockstep.tick().replayedTicks == 12u);

    const PushCheck& check = run.pushProbe.check;
    REQUIRE(check.pushes == 1u);
    REQUIRE(check.declarationsChecked == kSlots * kDeclarations);
    REQUIRE(check.mismatches == 0u);
    REQUIRE(check.maxAngularVelocityError == 0.f);
    printSummary("F push check", lockstep.totals, run);

    SECTION("a dropped push is seen by the same check")
    {
        DriverRig dropped(lease.runtime, config);
        for (uint32_t slot = 0; slot < kSlots; ++slot)
            dropped.join(slot, kFirstId + slot);
        for (int index = 0; index < 40; ++index)
            dropped.tick();
        dropped.pushProbe.skipPush = true;
        dropped.injectCorrection(kCorrectedSlot, anchor, perturbed(reference.states.at(anchor).at(id)));
        REQUIRE(dropped.tick().replayedTicks == 12u);
        REQUIRE(dropped.pushProbe.check.pushes == 1u);
        REQUIRE(dropped.pushProbe.check.mismatches >= 1u);
        REQUIRE(dropped.pushProbe.check.maxPositionErrorCm >= 4.f);
    }
}

namespace
{
    struct PushFidelity
    {
        int firstDifferingTick = -1;
        float maxCapsuleDifferenceCm = 0.f;
        uint64_t movedByPush = 0;
    };

    PushFidelity alwaysAgainstUnreplayed(JoltRuntime& runtime, bool skipPush, int ticks)
    {
        DriverRigConfig config;
        config.policy = ResimPolicy::Always;
        config.depthTicks = 8;
        DriverRigConfig quietConfig = config;
        quietConfig.policy = ResimPolicy::OnRequest;
        DriverRig run(runtime, config);
        DriverRig quiet(runtime, quietConfig);
        run.pushProbe.skipPush = skipPush;
        for (uint32_t slot = 0; slot < kSlots; ++slot)
        {
            run.join(slot, kFirstId + slot);
            quiet.join(slot, kFirstId + slot);
        }
        PushFidelity fidelity;
        for (int index = 1; index <= ticks; ++index)
        {
            run.tick();
            REQUIRE(quiet.tick().replayedTicks == 0u);
            const bool same = sameBytes(fullState(run.scene.world), fullState(quiet.scene.world))
                && run.scene.world.liveStateHash() == quiet.scene.world.liveStateHash() && sameSimStates(run.storage, quiet.storage);
            if (!same && fidelity.firstDifferingTick < 0)
                fidelity.firstDifferingTick = index;
            for (uint32_t slot = 0; slot < kSlots; ++slot)
            {
                const glm::vec3 a = run.simOf(kFirstId + slot)->getAllState().getState().capsule.bodyState.position;
                const glm::vec3 b = quiet.simOf(kFirstId + slot)->getAllState().getState().capsule.bodyState.position;
                fidelity.maxCapsuleDifferenceCm = std::max(fidelity.maxCapsuleDifferenceCm, maxAbs(a - b));
            }
        }
        fidelity.movedByPush = run.pushProbe.check.declarationsMovedByPush;
        REQUIRE(run.driver.getDiagnostics().grantedResims() == uint64_t(ticks) - (config.depthTicks + 1u));
        return fidelity;
    }
} // namespace

TEST_CASE("JoltSuite.AlwaysResimsReproduceTheUnreplayedRunByteForByte", "[Jolt]")
{
    RuntimeLease lease;
    const PushFidelity withoutPush = alwaysAgainstUnreplayed(lease.runtime, true, 300);
    REQUIRE(withoutPush.firstDifferingTick == -1);
    REQUIRE(withoutPush.maxCapsuleDifferenceCm == 0.f);
    REQUIRE(withoutPush.movedByPush == 0u);

    const PushFidelity withPush = alwaysAgainstUnreplayed(lease.runtime, false, 300);
    REQUIRE(withPush.firstDifferingTick == -1);
    REQUIRE(withPush.movedByPush == 0u);
    REQUIRE(withPush.maxCapsuleDifferenceCm == 0.f);
    std::printf("[JoltSuite] push fidelity, always depth 8 vs an unreplayed run over 300 ticks: without push firstDiff=%d maxCapsuleDiffCm=%g; "
                "with push firstDiff=%d maxCapsuleDiffCm=%g movedByPush=%llu\n",
        withoutPush.firstDifferingTick, double(withoutPush.maxCapsuleDifferenceCm), withPush.firstDifferingTick,
        double(withPush.maxCapsuleDifferenceCm), (unsigned long long)withPush.movedByPush);
}

TEST_CASE("JoltSuite.Finding.AResimWithoutACorrectionReproducesTheUnreplayedRun", "[Jolt]")
{
    RuntimeLease lease;
    const PushFidelity withPush = alwaysAgainstUnreplayed(lease.runtime, false, 60);
    INFO("first differing tick " << withPush.firstDifferingTick << ", max capsule difference " << withPush.maxCapsuleDifferenceCm << " cm");
    REQUIRE(withPush.firstDifferingTick == -1);
}

namespace
{
    constexpr float kOffsetCentreOfMassMetres = 0.25f;
    constexpr uint32_t kSeamWriteSamples = 1000;
    constexpr uint32_t kSeamWriteSeed = 59;

    constexpr const char* kSeamWriteTripwireExplanation =
        "TRIPWIRE: this test is EXPECTED to fail one day, and that failure is the signal, not a bug.\n"
        "Today the simulations work in centimetres and Jolt works in metres. Writing a body state into Jolt (cm -> m, plus Jolt's "
        "centre-of-mass offset) does not give back the same bits.\n"
        "Because of that, SimulationIntegrationExecutor::pushCorrectedBodyStatesAll compares each body with the restored Jolt snapshot "
        "and skips the write when they are equal (the restore skip). Without it, every resim would perturb bodies nobody corrected and "
        "start disagreement storms.\n"
        "The simulations are planned to move to metres, with Jolt's centre-of-mass convention. When that lands, the write is exact and "
        "this test fails.\n"
        "When it fails: remove the restore-skip comparison from pushCorrectedBodyStatesAll (keep setBodyPose), then delete this test. "
        "To keep the skip on purpose instead, delete only this test and record why in og-simulation-jolt's docs/JoltUnits-rationale.md "
        "\xC2\xA7"
        "5.";

    void offsetCentreOfMass(JoltWorld& world, JPH::BodyID body)
    {
        JPH::BodyInterface& bodies = world.bodies();
        bodies.SetShape(body, new JPH::OffsetCenterOfMassShape(bodies.GetShape(body), JPH::Vec3(kOffsetCentreOfMassMetres, 0.f, 0.f)), false,
            JPH::EActivation::DontActivate);
    }

    void offsetRadialCentresOfMass(JoltScene& scene)
    {
        for (uint32_t slot = 0; slot < kSlots; ++slot)
            offsetCentreOfMass(scene.world, scene.world.slotBodyId(slot, kTemplateRadial));
    }

    float centreOfMassOffsetMetres(const JoltWorld& world, JPH::BodyID body)
    {
        return world.bodies().GetShape(body)->GetCenterOfMass().Length();
    }

    class SeamRandom
    {
    public:
        explicit SeamRandom(uint32_t seed) : m_engine(seed) {}

        float uniform(float low, float high) { return low + (high - low) * float(double(m_engine()) / 4294967296.0); }

        glm::vec3 vector(float magnitude) { return glm::vec3(uniform(-magnitude, magnitude), uniform(-magnitude, magnitude), uniform(-magnitude, magnitude)); }

        glm::quat unitQuaternion()
        {
            for (;;)
            {
                const glm::vec4 candidate(uniform(-1.f, 1.f), uniform(-1.f, 1.f), uniform(-1.f, 1.f), uniform(-1.f, 1.f));
                const float lengthSquared = glm::dot(candidate, candidate);
                if (lengthSquared > 0.01f && lengthSquared <= 1.f)
                    return glm::normalize(glm::quat(candidate.w, candidate.x, candidate.y, candidate.z));
            }
        }

        uint32_t index(uint32_t count) { return uint32_t(m_engine() % count); }

    private:
        std::mt19937 m_engine;
    };

    std::vector<PhysicsBodyState> lossySeamStates(uint32_t seed)
    {
        SeamRandom random(seed);
        std::vector<PhysicsBodyState> states(kSeamWriteSamples);
        for (PhysicsBodyState& state : states)
        {
            state.position = random.vector(1000.f) + glm::vec3(0.f, 0.f, 1000.f);
            state.rotation = random.unitQuaternion();
            state.linearVelocity = random.vector(1500.f);
            state.angularVelocity = random.vector(10.f);
        }
        return states;
    }

    bool centimetresRoundTripExactly(float centimetres)
    {
        const float metres = joltUnits::centimetresToMetres(centimetres);
        return metres * joltSeamUnits::kCentimetresPerMetre == centimetres;
    }

    std::vector<PhysicsBodyState> exactSeamStates(uint32_t seed)
    {
        std::vector<float> exactCentimetres;
        for (int32_t step = 1; step <= 400; ++step)
        {
            for (const float centimetres : { 25.f * float(step), -25.f * float(step) })
            {
                if (centimetresRoundTripExactly(centimetres))
                    exactCentimetres.push_back(centimetres);
            }
        }
        REQUIRE(exactCentimetres.size() > 100u);
        SeamRandom random(seed);
        std::vector<PhysicsBodyState> states(kSeamWriteSamples);
        for (PhysicsBodyState& state : states)
        {
            const uint32_t count = uint32_t(exactCentimetres.size());
            state.position = glm::vec3(exactCentimetres[random.index(count)], exactCentimetres[random.index(count)], exactCentimetres[random.index(count)]);
            state.rotation = random.unitQuaternion();
        }
        return states;
    }

    uint32_t seamWritesThatChangedBits(JoltPhysicsBodyAdapter& bodies, BodyId body, const std::vector<PhysicsBodyState>& states)
    {
        uint32_t changed = 0;
        for (const PhysicsBodyState& written : states)
        {
            bodies.setBodyPose(body, written.position, written.rotation);
            bodies.setBodyLinearVelocity(body, written.linearVelocity);
            bodies.setBodyAngularVelocity(body, written.angularVelocity);
            const PhysicsBodyState captured = bodies.captureBodyState(body);
            if (std::memcmp(&captured, &written, sizeof(PhysicsBodyState)) != 0)
                ++changed;
        }
        return changed;
    }

    void requireTheSeamWriteIsLossy(uint32_t lossyCount)
    {
        INFO(kSeamWriteTripwireExplanation);
        REQUIRE(lossyCount > 0u);
    }

    struct TripwireBodies
    {
        explicit TripwireBodies(JoltRuntime& runtime)
            : scene(runtime, SceneConfig{})
            , offsetBody(scene.world.slotBodyId(0, kTemplateRadial))
            , centredBody(scene.world.slotBodyId(0, kTemplateGuard))
        {
            offsetCentreOfMass(scene.world, offsetBody);
        }

        JoltScene scene;
        JPH::BodyID offsetBody;
        JPH::BodyID centredBody;
    };
} // namespace

TEST_CASE("JoltSuite.Tripwire.TheSeamWriteIsLossyUntilTheSimulationsUseMetres", "[Jolt]")
{
    RuntimeLease lease;
    TripwireBodies tripwire(lease.runtime);
    REQUIRE(centreOfMassOffsetMetres(tripwire.scene.world, tripwire.offsetBody) == kOffsetCentreOfMassMetres);
    REQUIRE(centreOfMassOffsetMetres(tripwire.scene.world, tripwire.centredBody) == 0.f);

    const uint32_t lossyCount = seamWritesThatChangedBits(tripwire.scene.bodies, JoltPhysicsBodyAdapter::bodyIdOf(tripwire.offsetBody),
        lossySeamStates(kSeamWriteSeed));
    std::printf("[JoltSuite] tripwire: %u of 1000 seam writes changed bits (0 = remove the restore skip in pushCorrectedBodyStatesAll)\n", lossyCount);

    const uint32_t exactCount = seamWritesThatChangedBits(tripwire.scene.bodies, JoltPhysicsBodyAdapter::bodyIdOf(tripwire.centredBody),
        exactSeamStates(kSeamWriteSeed));
    REQUIRE(exactCount == 0u);

    requireTheSeamWriteIsLossy(lossyCount);
}

TEST_CASE("JoltSuite.Tripwire.Poison.ExactRoundTripsTripTheTripwire", "[.][JoltSuitePoison]")
{
    RuntimeLease lease;
    TripwireBodies tripwire(lease.runtime);
    requireTheSeamWriteIsLossy(seamWritesThatChangedBits(tripwire.scene.bodies, JoltPhysicsBodyAdapter::bodyIdOf(tripwire.centredBody),
        exactSeamStates(kSeamWriteSeed)));
}

TEST_CASE("JoltSuite.SetBodyPoseWritesTheQuaternionBitForBitAndTheMatrixPathDoesNot", "[Jolt]")
{
    RuntimeLease lease;
    JoltScene scene(lease.runtime, SceneConfig{});
    const BodyId body = JoltPhysicsBodyAdapter::bodyIdOf(scene.world.slotBodyId(0, kTemplateRadial));
    SeamRandom random(kSeamWriteSeed);
    uint32_t poseChanged = 0;
    uint32_t matrixChanged = 0;
    for (uint32_t sample = 0; sample < kSeamWriteSamples; ++sample)
    {
        const glm::quat rotation = random.unitQuaternion();
        const glm::vec3 position = random.vector(1000.f);

        scene.bodies.setBodyPose(body, position, rotation);
        const glm::quat poseCaptured = scene.bodies.captureBodyState(body).rotation;
        if (std::memcmp(&poseCaptured, &rotation, sizeof(glm::quat)) != 0)
            ++poseChanged;

        glm::mat4 transform = glm::mat4_cast(rotation);
        transform[3] = glm::vec4(position, 1.f);
        scene.bodies.setBodyTransform(body, transform);
        const glm::quat matrixCaptured = scene.bodies.captureBodyState(body).rotation;
        if (std::memcmp(&matrixCaptured, &rotation, sizeof(glm::quat)) != 0)
            ++matrixChanged;
    }
    std::printf("[JoltSuite] pose write: setBodyPose changed %u of 1000 quaternions, the setBodyTransform matrix path changed %u\n", poseChanged,
        matrixChanged);
    REQUIRE(poseChanged == 0u);
    REQUIRE(matrixChanged > 0u);
}

namespace
{
    constexpr uint32_t kSwingLeadTicks = 30;
    constexpr uint32_t kSwingAnchorDepth = 6;
    constexpr uint32_t kSwingComparedTicks = 120;

    struct SwingFidelity
    {
        int firstDifferingTick = -1;
        uint32_t differingTicks = 0;
        uint64_t movedByPush = 0;
        uint64_t granted = 0;
        float anchorAngularSpeed = 0.f;
        float maxCapsuleSpeed = 0.f;
    };

    bool sameCaptures(const DriverRig& first, const DriverRig& second)
    {
        for (uint32_t slot = 0; slot < kSlots; ++slot)
        {
            for (uint32_t templateIndex = 0; templateIndex < kBodiesPerBrawler; ++templateIndex)
            {
                const BodyId body = JoltPhysicsBodyAdapter::bodyIdOf(first.scene.world.slotBodyId(slot, templateIndex));
                const PhysicsBodyState a = first.scene.bodies.captureBodyState(body);
                const PhysicsBodyState b = second.scene.bodies.captureBodyState(body);
                if (std::memcmp(&a, &b, sizeof(PhysicsBodyState)) != 0)
                    return false;
            }
        }
        return true;
    }

    SwingFidelity resimMidSwingAgainstUnreplayed(JoltRuntime& runtime)
    {
        DriverRigConfig config;
        config.policy = ResimPolicy::OnRequest;
        config.drive = ProbeDrive::Swing;
        DriverRig run(runtime, config);
        DriverRig quiet(runtime, config);
        offsetRadialCentresOfMass(run.scene);
        offsetRadialCentresOfMass(quiet.scene);
        REQUIRE(centreOfMassOffsetMetres(run.scene.world, run.scene.world.slotBodyId(kCorrectedSlot, kTemplateRadial)) == kOffsetCentreOfMassMetres);
        for (uint32_t slot = 0; slot < kSlots; ++slot)
        {
            run.join(slot, kFirstId + slot);
            quiet.join(slot, kFirstId + slot);
        }

        SwingFidelity fidelity;
        std::map<SimTick, std::map<uint32_t, ProbeState>> predicted;
        for (uint32_t index = 0; index < kSwingLeadTicks; ++index)
        {
            REQUIRE(run.tick().replayedTicks == 0u);
            REQUIRE(quiet.tick().replayedTicks == 0u);
            for (uint32_t slot = 0; slot < kSlots; ++slot)
                predicted[run.present()][kFirstId + slot] = run.simOf(kFirstId + slot)->getAllState().getState();
        }

        const SimTick anchor = run.present() - kSwingAnchorDepth;
        REQUIRE(anchor / kSwingPhaseTicks == run.present() / kSwingPhaseTicks);
        const uint32_t correctedId = run.idOfSlot[kCorrectedSlot];
        ProbeState corrected = predicted.at(anchor).at(correctedId);
        fidelity.anchorAngularSpeed = glm::length(corrected.radial.bodyState.angularVelocity);
        corrected.queryHits += 1;
        run.injectCorrection(kCorrectedSlot, anchor, corrected);

        for (uint32_t index = 1; index <= kSwingComparedTicks; ++index)
        {
            const TickOutcome outcome = run.tick();
            REQUIRE(quiet.tick().replayedTicks == 0u);
            if (index == 1u)
                REQUIRE(outcome.replayedTicks == kSwingAnchorDepth);
            const bool same = sameCaptures(run, quiet) && sameBytes(fullState(run.scene.world), fullState(quiet.scene.world))
                && run.scene.world.liveStateHash() == quiet.scene.world.liveStateHash() && sameSimStates(run.storage, quiet.storage);
            if (!same)
            {
                ++fidelity.differingTicks;
                if (fidelity.firstDifferingTick < 0)
                    fidelity.firstDifferingTick = int(index);
            }
            for (uint32_t slot = 0; slot < kSlots; ++slot)
                fidelity.maxCapsuleSpeed = std::max(fidelity.maxCapsuleSpeed,
                    glm::length(run.simOf(kFirstId + slot)->getAllState().getState().capsule.bodyState.linearVelocity));
        }
        fidelity.movedByPush = run.pushProbe.check.declarationsMovedByPush;
        fidelity.granted = run.driver.getDiagnostics().grantedResims();
        return fidelity;
    }
} // namespace

TEST_CASE("JoltSuite.AResimWithNoBodyCorrectionMidSwingReproducesTheUnreplayedRunBitForBit", "[Jolt]")
{
    RuntimeLease lease;
    const SwingFidelity fidelity = resimMidSwingAgainstUnreplayed(lease.runtime);
    std::printf("[JoltSuite] mid-swing resim vs an unreplayed run over %u ticks: granted=%llu firstDiff=%d differingTicks=%u movedByPush=%llu "
                "anchorAngularSpeed=%g maxCapsuleSpeedCmPerS=%g\n",
        kSwingComparedTicks, (unsigned long long)fidelity.granted, fidelity.firstDifferingTick, fidelity.differingTicks,
        (unsigned long long)fidelity.movedByPush, double(fidelity.anchorAngularSpeed), double(fidelity.maxCapsuleSpeed));
    REQUIRE(fidelity.granted == 1u);
    REQUIRE(fidelity.anchorAngularSpeed > 1.f);
    REQUIRE(fidelity.maxCapsuleSpeed > 10.f);
    REQUIRE(fidelity.movedByPush == 0u);
    REQUIRE(fidelity.firstDifferingTick == -1);
    REQUIRE(fidelity.differingTicks == 0u);
}

#endif // WITH_LOW_LEVEL_TESTS
