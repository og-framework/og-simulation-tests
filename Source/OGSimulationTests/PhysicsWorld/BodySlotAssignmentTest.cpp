// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <array>
#include <cstdint>
#include <initializer_list>
#include <optional>

#include "catch_amalgamated.hpp"
#include "OGSimulation/BodySlotAssignment.h"

namespace
{
using TakenSlots = std::array<bool, kMaxSimulatableSlots>;

TakenSlots bodySlotTakenOf(std::initializer_list<uint32_t> slots)
{
    TakenSlots taken{};
    for (const uint32_t slot : slots)
    {
        taken[slot] = true;
    }
    return taken;
}

TakenSlots bodySlotTakenFromMask(uint32_t mask)
{
    TakenSlots taken{};
    for (uint32_t slot = 0; slot < kMaxSimulatableSlots; ++slot)
    {
        taken[slot] = ((mask >> slot) & 1u) != 0u;
    }
    return taken;
}

std::optional<uint32_t> bodySlotLowestFree(const TakenSlots& taken)
{
    for (uint32_t slot = 0; slot < kMaxSimulatableSlots; ++slot)
    {
        if (!taken[slot])
        {
            return slot;
        }
    }
    return std::nullopt;
}

constexpr uint32_t kAllTakenMasks = 1u << kMaxSimulatableSlots;
}

TEST_CASE("BodySlotAssignment authority takes the lowest free slot and reports Full when none is free", "[PhysicsWorld]")
{
    STATIC_REQUIRE(kMaxSimulatableSlots == 8);

    CHECK(decideBodySlot(true, std::nullopt, bodySlotTakenOf({}))
          == BodySlotDecision{ BodySlotOutcome::Assigned, 0u });
    CHECK(decideBodySlot(true, std::nullopt, bodySlotTakenOf({ 0, 1 }))
          == BodySlotDecision{ BodySlotOutcome::Assigned, 2u });
    CHECK(decideBodySlot(true, std::nullopt, bodySlotTakenOf({ 0, 2, 3 }))
          == BodySlotDecision{ BodySlotOutcome::Assigned, 1u });
    CHECK(decideBodySlot(true, std::nullopt, bodySlotTakenOf({ 0, 1, 2, 3, 4, 5, 6 }))
          == BodySlotDecision{ BodySlotOutcome::Assigned, 7u });
    CHECK(decideBodySlot(true, std::nullopt, bodySlotTakenOf({ 0, 1, 2, 3, 4, 5, 6, 7 })).outcome
          == BodySlotOutcome::Full);

    SECTION("the authority ignores a replicated slot: its own table is the only input")
    {
        CHECK(decideBodySlot(true, std::optional<uint32_t>(5u), bodySlotTakenOf({}))
              == BodySlotDecision{ BodySlotOutcome::Assigned, 0u });
        CHECK(decideBodySlot(true, std::optional<uint32_t>(5u), bodySlotTakenOf({ 0, 1, 2, 3, 4, 5, 6, 7 })).outcome
              == BodySlotOutcome::Full);
    }

    SECTION("every taken set: the lowest free slot, or Full")
    {
        for (uint32_t mask = 0; mask < kAllTakenMasks; ++mask)
        {
            const TakenSlots taken = bodySlotTakenFromMask(mask);
            const BodySlotDecision decision = decideBodySlot(true, std::nullopt, taken);
            const std::optional<uint32_t> lowestFree = bodySlotLowestFree(taken);
            INFO("taken mask " << mask);
            if (lowestFree.has_value())
            {
                CHECK(decision == BodySlotDecision{ BodySlotOutcome::Assigned, *lowestFree });
            }
            else
            {
                CHECK(decision.outcome == BodySlotOutcome::Full);
            }
        }
    }
}

TEST_CASE("BodySlotAssignment client waits for the replicated slot before it binds", "[PhysicsWorld]")
{
    CHECK(decideBodySlot(false, std::nullopt, bodySlotTakenOf({})).outcome
          == BodySlotOutcome::WaitForReplication);
    CHECK(decideBodySlot(false, std::nullopt, bodySlotTakenOf({ 0, 1, 2 })).outcome
          == BodySlotOutcome::WaitForReplication);
    CHECK(decideBodySlot(false, std::nullopt, bodySlotTakenOf({ 0, 1, 2, 3, 4, 5, 6, 7 })).outcome
          == BodySlotOutcome::WaitForReplication);

    for (uint32_t mask = 0; mask < kAllTakenMasks; ++mask)
    {
        INFO("taken mask " << mask);
        CHECK(decideBodySlot(false, std::nullopt, bodySlotTakenFromMask(mask)).outcome
              == BodySlotOutcome::WaitForReplication);
    }
}

TEST_CASE("BodySlotAssignment client waits while the replicated slot is still held locally", "[PhysicsWorld]")
{
    CHECK(decideBodySlot(false, std::optional<uint32_t>(3u), bodySlotTakenOf({ 3 }))
          == BodySlotDecision{ BodySlotOutcome::WaitForRelease, 3u });
    CHECK(decideBodySlot(false, std::optional<uint32_t>(0u), bodySlotTakenOf({ 0, 1, 2, 3, 4, 5, 6, 7 }))
          == BodySlotDecision{ BodySlotOutcome::WaitForRelease, 0u });

    uint32_t witnessesWithALowerFreeSlot = 0;
    for (uint32_t replicated = 0; replicated < kMaxSimulatableSlots; ++replicated)
    {
        for (uint32_t mask = 0; mask < kAllTakenMasks; ++mask)
        {
            const TakenSlots taken = bodySlotTakenFromMask(mask);
            if (!taken[replicated])
            {
                continue;
            }
            const std::optional<uint32_t> lowestFree = bodySlotLowestFree(taken);
            if (lowestFree.has_value() && *lowestFree < replicated)
            {
                ++witnessesWithALowerFreeSlot;
            }
            INFO("replicated slot " << replicated << ", taken mask " << mask);
            CHECK(decideBodySlot(false, std::optional<uint32_t>(replicated), taken)
                  == BodySlotDecision{ BodySlotOutcome::WaitForRelease, replicated });
        }
    }
    CHECK(witnessesWithALowerFreeSlot > 0u);
}

TEST_CASE("BodySlotAssignment client is assigned the replicated slot even when a lower slot is free locally", "[PhysicsWorld]")
{
    CHECK(decideBodySlot(false, std::optional<uint32_t>(1u), bodySlotTakenOf({}))
          == BodySlotDecision{ BodySlotOutcome::Assigned, 1u });
    CHECK(decideBodySlot(false, std::optional<uint32_t>(5u), bodySlotTakenOf({ 0, 2 }))
          == BodySlotDecision{ BodySlotOutcome::Assigned, 5u });
    CHECK(decideBodySlot(false, std::optional<uint32_t>(7u), bodySlotTakenOf({ 0, 1, 2, 3, 4, 5, 6 }))
          == BodySlotDecision{ BodySlotOutcome::Assigned, 7u });

    uint32_t witnessesWithALowerFreeSlot = 0;
    for (uint32_t replicated = 0; replicated < kMaxSimulatableSlots; ++replicated)
    {
        for (uint32_t mask = 0; mask < kAllTakenMasks; ++mask)
        {
            const TakenSlots taken = bodySlotTakenFromMask(mask);
            if (taken[replicated])
            {
                continue;
            }
            const std::optional<uint32_t> lowestFree = bodySlotLowestFree(taken);
            if (lowestFree.has_value() && *lowestFree < replicated)
            {
                ++witnessesWithALowerFreeSlot;
            }
            INFO("replicated slot " << replicated << ", taken mask " << mask);
            CHECK(decideBodySlot(false, std::optional<uint32_t>(replicated), taken)
                  == BodySlotDecision{ BodySlotOutcome::Assigned, replicated });
        }
    }
    CHECK(witnessesWithALowerFreeSlot > 0u);
}

#endif
