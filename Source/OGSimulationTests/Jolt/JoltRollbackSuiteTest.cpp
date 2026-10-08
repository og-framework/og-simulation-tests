// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltRollbackSuiteRig.h"

#include <cstdio>

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

TEST_CASE("JoltSuite.WithoutThePushAlwaysResimsReproduceTheUnreplayedRunByteForByte", "[Jolt]")
{
    RuntimeLease lease;
    const PushFidelity withoutPush = alwaysAgainstUnreplayed(lease.runtime, true, 300);
    REQUIRE(withoutPush.firstDifferingTick == -1);
    REQUIRE(withoutPush.maxCapsuleDifferenceCm == 0.f);
    REQUIRE(withoutPush.movedByPush == 0u);

    const PushFidelity withPush = alwaysAgainstUnreplayed(lease.runtime, false, 300);
    REQUIRE(withPush.firstDifferingTick == 10);
    REQUIRE(withPush.movedByPush > 0u);
    REQUIRE(withPush.maxCapsuleDifferenceCm > 0.f);
    REQUIRE(withPush.maxCapsuleDifferenceCm < 0.1f);
    std::printf("[JoltSuite] push fidelity, always depth 8 vs an unreplayed run over 300 ticks: without push firstDiff=%d maxCapsuleDiffCm=%g; "
                "with push firstDiff=%d maxCapsuleDiffCm=%g movedByPush=%llu\n",
        withoutPush.firstDifferingTick, double(withoutPush.maxCapsuleDifferenceCm), withPush.firstDifferingTick,
        double(withPush.maxCapsuleDifferenceCm), (unsigned long long)withPush.movedByPush);
}

TEST_CASE("JoltSuite.Finding.AResimWithoutACorrectionReproducesTheUnreplayedRun", "[.][JoltSuiteFinding]")
{
    RuntimeLease lease;
    const PushFidelity withPush = alwaysAgainstUnreplayed(lease.runtime, false, 60);
    INFO("first differing tick " << withPush.firstDifferingTick << ", max capsule difference " << withPush.maxCapsuleDifferenceCm << " cm");
    REQUIRE(withPush.firstDifferingTick == -1);
}

#endif // WITH_LOW_LEVEL_TESTS
