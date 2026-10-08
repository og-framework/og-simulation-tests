// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltRollbackSuiteRig.h"

#include <cstdio>

using namespace joltSuite;

namespace
{
    constexpr uint32_t kFirstId = 101;
    constexpr uint32_t kInitialSlots = 6;
    constexpr uint32_t kJoiningSlot = 6;
    constexpr uint32_t kLeavingSlot = 3;
    constexpr uint32_t kCorrectedSlot = 2;

    bool slotIsParked(const JoltWorld& world, uint32_t slot)
    {
        for (uint32_t index = 0; index < world.bodiesPerSlot(); ++index)
        {
            if (world.bodies().GetObjectLayer(world.slotBodyId(slot, index)) != world.layers().parkedLayer())
                return false;
        }
        return true;
    }

    bool slotIsAtRest(const JoltWorld& world, uint32_t slot)
    {
        for (uint32_t index = 0; index < world.bodiesPerSlot(); ++index)
        {
            const JPH::BodyID id = world.slotBodyId(slot, index);
            if (world.bodies().GetLinearVelocity(id) != JPH::Vec3::sZero() || world.bodies().GetAngularVelocity(id) != JPH::Vec3::sZero())
                return false;
        }
        return true;
    }

    std::vector<BodyBits> slotBits(const JoltWorld& world, uint32_t slot)
    {
        std::vector<BodyBits> bits;
        for (uint32_t index = 0; index < world.bodiesPerSlot(); ++index)
            bits.push_back(bodyBitsOf(world, world.slotBodyId(slot, index)));
        return bits;
    }

    struct ReplayTracker
    {
        uint32_t replayCount = 0;

        std::optional<SimTick> replayTickOf(std::optional<SimTick> lastRestored, uint32_t stepsSinceRestore) const
        {
            if (!lastRestored.has_value() || stepsSinceRestore >= replayCount)
                return std::nullopt;
            return *lastRestored + 1u + stepsSinceRestore;
        }
    };

    TickOutcome correctAndTick(Lockstep& lockstep, ReplayTracker& tracker, uint32_t depth)
    {
        const SimTick anchor = lockstep.run.present() - depth;
        const uint32_t id = lockstep.run.idOfSlot[kCorrectedSlot];
        REQUIRE(lockstep.reference.states.count(anchor) == 1u);
        REQUIRE(lockstep.reference.states.at(anchor).count(id) == 1u);
        lockstep.inject(kCorrectedSlot, anchor, perturbed(lockstep.reference.states.at(anchor).at(id)));
        tracker.replayCount = depth;
        const TickOutcome outcome = lockstep.tick();
        tracker.replayCount = 0;
        REQUIRE(outcome.replayedTicks == depth);
        return outcome;
    }

    float distance(const glm::vec3& first, const glm::vec3& second)
    {
        return glm::length(first - second);
    }
} // namespace

TEST_CASE("JoltSuite.JoinAcrossCorrectionResimsMatchesTheReferenceAndIsParkedBeforeItsFirstSlot", "[Jolt]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::OnRequest;
    DriverRig run(lease.runtime, config);
    ReferenceRig reference(lease.runtime, config);
    for (uint32_t slot = 0; slot < kInitialSlots; ++slot)
    {
        run.join(slot, kFirstId + slot);
        reference.join(slot, kFirstId + slot);
    }

    Lockstep lockstep{ run, reference };
    lockstep.ticks(40);
    REQUIRE(slotIsParked(run.scene.world, kJoiningSlot));

    const SimTick joinTick = run.present() + 1u;
    const uint32_t joiningId = kFirstId + kJoiningSlot;
    run.join(kJoiningSlot, joiningId);
    reference.join(kJoiningSlot, joiningId);

    ReplayTracker tracker;
    uint64_t parkedReplaySteps = 0;
    uint64_t liveReplaySteps = 0;
    uint64_t violations = 0;
    run.observed.beforeStep = [&](std::optional<SimTick> lastRestored, uint32_t stepsSinceRestore) {
        const std::optional<SimTick> replayTick = tracker.replayTickOf(lastRestored, stepsSinceRestore);
        if (!replayTick.has_value())
            return;
        const bool parked = slotIsParked(run.scene.world, kJoiningSlot);
        if (*replayTick < joinTick)
        {
            ++parkedReplaySteps;
            violations += parked ? 0u : 1u;
        }
        else
        {
            ++liveReplaySteps;
            violations += parked ? 1u : 0u;
        }
    };

    const size_t logBeforeJoin = run.log.size();
    std::vector<float> joinedJumpCm;
    std::vector<float> otherJumpCm;
    std::vector<float> joinedStepCm;
    auto correctMeasuringJumps = [&](uint32_t depth) {
        const glm::vec3 joinedTwoBefore = run.simOf(joiningId)->getAllState().getState().capsule.bodyState.position;
        lockstep.tick();
        joinedStepCm.push_back(distance(run.simOf(joiningId)->getAllState().getState().capsule.bodyState.position, joinedTwoBefore));
        const glm::vec3 joinedBefore = run.simOf(joiningId)->getAllState().getState().capsule.bodyState.position;
        const glm::vec3 otherBefore = run.simOf(kFirstId)->getAllState().getState().capsule.bodyState.position;
        correctAndTick(lockstep, tracker, depth);
        joinedJumpCm.push_back(distance(run.simOf(joiningId)->getAllState().getState().capsule.bodyState.position, joinedBefore));
        otherJumpCm.push_back(distance(run.simOf(kFirstId)->getAllState().getState().capsule.bodyState.position, otherBefore));
    };
    lockstep.ticks(3);
    correctMeasuringJumps(8u);
    lockstep.ticks(3);
    correctMeasuringJumps(12u);
    lockstep.ticks(10);
    std::printf("[JoltSuite] E join: capsule displacement over the corrected tick: joined=%.2f / %.2f cm, an unrelated character=%.2f / %.2f cm; joined over the tick before=%.2f / %.2f cm\n",
        double(joinedJumpCm[0]), double(joinedJumpCm[1]), double(otherJumpCm[0]), double(otherJumpCm[1]),
        double(joinedStepCm[0]), double(joinedStepCm[1]));
    run.observed.beforeStep = nullptr;

    REQUIRE(violations == 0u);
    REQUIRE(parkedReplaySteps == 4u + 3u);
    REQUIRE(liveReplaySteps == 4u + 9u);
    uint64_t joinedResimIntegrations = 0;
    for (size_t index = logBeforeJoin; index < run.log.size(); ++index)
    {
        const IntegrateRecord& record = run.log[index];
        if (record.id != joiningId)
            continue;
        REQUIRE(record.tick >= joinTick);
        if (record.resim)
            ++joinedResimIntegrations;
    }
    REQUIRE(joinedResimIntegrations == 4u + 9u);
    REQUIRE(lockstep.totals.granted == 2u);
    REQUIRE(run.observed.failedRestores == 0u);
    REQUIRE(run.driver.getDiagnostics().occupancyTimelineMisses() == 0u);
    REQUIRE(run.pushProbe.check.mismatches == 0u);
    printSummary("E join across correction resims", lockstep.totals, run);
}

TEST_CASE("JoltSuite.LeaveIsRetroactiveAndNoReplayTouchesTheDepartedSlot", "[Jolt]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::Always;
    config.depthTicks = 12;
    DriverRig run(lease.runtime, config);
    ReferenceRig reference(lease.runtime, config);
    for (uint32_t slot = 0; slot < kSlots; ++slot)
    {
        run.join(slot, kFirstId + slot);
        reference.join(slot, kFirstId + slot);
    }

    Lockstep lockstep{ run, reference };
    lockstep.ticks(40);

    const uint32_t leavingId = run.idOfSlot[kLeavingSlot];
    run.leave(kLeavingSlot);
    reference.leave(kLeavingSlot);
    const SimTick lastTickWithTheSlot = run.present();
    const std::optional<SimTick> oldest = run.scene.world.oldestHeldTick();
    REQUIRE(oldest.has_value());
    uint32_t heldTicksChecked = 0;
    for (SimTick tick = *oldest; tick <= lastTickWithTheSlot; ++tick)
    {
        if (!run.scene.world.hasTick(tick))
            continue;
        const std::optional<BodySlotOccupancy> occupancy = run.driver.getDiagnostics().occupancyAt(tick);
        REQUIRE(occupancy.has_value());
        REQUIRE_FALSE(occupancy->occupied.test(kLeavingSlot));
        REQUIRE(occupancy->occupied.count() == kSlots - 1u);
        ++heldTicksChecked;
    }
    REQUIRE(heldTicksChecked >= config.depthTicks + 1u);

    ReplayTracker tracker{ config.depthTicks };
    uint64_t replaySteps = 0;
    uint64_t violations = 0;
    std::vector<BodyBits> bitsBefore;
    run.observed.beforeStep = [&](std::optional<SimTick> lastRestored, uint32_t stepsSinceRestore) {
        if (!tracker.replayTickOf(lastRestored, stepsSinceRestore).has_value())
            return;
        ++replaySteps;
        if (!slotIsParked(run.scene.world, kLeavingSlot) || !slotIsAtRest(run.scene.world, kLeavingSlot))
            ++violations;
        bitsBefore = slotBits(run.scene.world, kLeavingSlot);
    };
    run.observed.afterStep = [&](std::optional<SimTick> lastRestored, uint32_t stepsSinceRestore) {
        if (!tracker.replayTickOf(lastRestored, stepsSinceRestore).has_value())
            return;
        if (slotBits(run.scene.world, kLeavingSlot) != bitsBefore)
            ++violations;
    };

    const size_t logBeforeLeave = run.log.size();
    lockstep.ticks(30);
    run.observed.beforeStep = nullptr;
    run.observed.afterStep = nullptr;
    REQUIRE(replaySteps == 30u * config.depthTicks);
    REQUIRE(violations == 0u);
    for (size_t index = logBeforeLeave; index < run.log.size(); ++index)
        REQUIRE(run.log[index].id != leavingId);
    REQUIRE(slotIsParked(run.scene.world, kLeavingSlot));
    REQUIRE(run.observed.failedRestores == 0u);
    REQUIRE(run.driver.getDiagnostics().occupancyTimelineMisses() == 0u);
    printSummary("E leave under always depth 12", lockstep.totals, run);
}

TEST_CASE("JoltSuite.Finding.AJoinFollowedByAResimInTheNextTickKeepsTheJoinedCharactersState", "[.][JoltSuiteFinding]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::Always;
    config.depthTicks = 12;
    DriverRig run(lease.runtime, config);
    for (uint32_t slot = 0; slot < kInitialSlots; ++slot)
        run.join(slot, kFirstId + slot);
    for (int index = 0; index < 40; ++index)
        run.tick();

    const uint32_t joiningId = kFirstId + kJoiningSlot;
    run.join(kJoiningSlot, joiningId);
    const glm::vec3 boundRadial = run.simOf(joiningId)->getAllState().getState().radial.bodyState.position;
    const glm::vec3 boundCapsule = run.simOf(joiningId)->getAllState().getState().capsule.bodyState.position;
    REQUIRE(distance(boundCapsule, homeOf(kJoiningSlot)) < 1.f);

    const TickOutcome outcome = run.tick();
    REQUIRE(outcome.replayedTicks == config.depthTicks);
    const ProbeState& state = run.simOf(joiningId)->getAllState().getState();
    INFO("radial after its first tick: (" << state.radial.bodyState.position.x << ", " << state.radial.bodyState.position.y << ", "
                                          << state.radial.bodyState.position.z << "); bound at (" << boundRadial.x << ", " << boundRadial.y
                                          << ", " << boundRadial.z << ")");
    CHECK(state.integrations == 1u);
    REQUIRE(distance(state.radial.bodyState.position, homeOf(kJoiningSlot) + glm::vec3(0.f, 70.f, 30.f)) < 20.f);
}

TEST_CASE("JoltSuite.Finding.AResimCrossingAJoinDoesNotReplayTheJoinedCharacterFromItsPresentState", "[.][JoltSuiteFinding]")
{
    RuntimeLease lease;
    DriverRigConfig config;
    config.policy = ResimPolicy::OnRequest;
    DriverRig run(lease.runtime, config);
    ReferenceRig reference(lease.runtime, config);
    for (uint32_t slot = 0; slot < kInitialSlots; ++slot)
    {
        run.join(slot, kFirstId + slot);
        reference.join(slot, kFirstId + slot);
    }
    Lockstep lockstep{ run, reference };
    lockstep.ticks(40);

    const uint32_t joiningId = kFirstId + kJoiningSlot;
    run.join(kJoiningSlot, joiningId);
    reference.join(kJoiningSlot, joiningId);
    lockstep.ticks(3);
    const glm::vec3 twoBefore = run.simOf(joiningId)->getAllState().getState().capsule.bodyState.position;
    lockstep.tick();
    const glm::vec3 before = run.simOf(joiningId)->getAllState().getState().capsule.bodyState.position;
    const float normalStepCm = distance(before, twoBefore);

    ReplayTracker tracker;
    correctAndTick(lockstep, tracker, 8u);
    const float correctedStepCm = distance(run.simOf(joiningId)->getAllState().getState().capsule.bodyState.position, before);
    INFO("joined capsule moved " << correctedStepCm << " cm over the corrected tick; " << normalStepCm << " cm over the tick before");
    REQUIRE(correctedStepCm < 2.f * normalStepCm + 0.5f);
}

#endif // WITH_LOW_LEVEL_TESTS
