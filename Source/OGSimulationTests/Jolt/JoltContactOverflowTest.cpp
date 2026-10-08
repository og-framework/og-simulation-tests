// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include <string>
#include <vector>

using namespace joltTestRig;

namespace
{
    double g_fakeNowSeconds = 0.0;

    double fakeClock()
    {
        return g_fakeNowSeconds;
    }

    uint32_t linesNaming(const std::vector<std::string>& lines, const char* flag)
    {
        uint32_t count = 0;
        for (const std::string& line : lines)
        {
            count += line.find(flag) != std::string::npos ? 1u : 0u;
        }
        return count;
    }
}

TEST_CASE("JoltWorld.ContactOverflowIsLoggedOncePerFlagPerSecond", "[Jolt]")
{
#ifdef JPH_ENABLE_ASSERTS
    SKIP("with JPH_ENABLE_ASSERTS, PhysicsSystem::Update asserts on any EPhysicsUpdateError before JoltWorld can log it");
#endif
    RuntimeLease lease;
    JoltWorldConfig config = brawlerWorldConfig(0);
    config.maxBodyPairs = 4;
    config.maxContactConstraints = 2;
    config.clock = &fakeClock;
    g_fakeNowSeconds = 100.0;

    std::vector<std::string> lines;
    JoltWorld world(lease.runtime, config, [&lines](const char* line) { lines.emplace_back(line); });
    addFloor(world);
    world.applyOccupancy(allOccupied());
    for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
    {
        world.bodies().SetPosition(world.slotBodyId(slot, kTemplateCapsule), JPH::RVec3(0.05f * float(slot), 0.f, 0.9f), JPH::EActivation::Activate);
    }

    for (uint32_t index = 0; index < 50; ++index)
    {
        world.step(kDt);
        g_fakeNowSeconds += 0.01;
    }

    const char* flags[] = { "ManifoldCacheFull", "BodyPairCacheFull", "ContactConstraintsFull" };
    const JoltContactOverflow kinds[] = { JoltContactOverflow::ManifoldCacheFull, JoltContactOverflow::BodyPairCacheFull, JoltContactOverflow::ContactConstraintsFull };
    for (size_t index = 0; index < 3; ++index)
    {
        INFO(flags[index]);
        const uint64_t occurrences = world.contactOverflowCount(kinds[index]);
        CHECK(linesNaming(lines, flags[index]) == (occurrences > 0u ? 1u : 0u));
    }
    CHECK(world.contactOverflowCount(JoltContactOverflow::ContactConstraintsFull) > 1u);
    CHECK(world.contactOverflowCount(JoltContactOverflow::BodyPairCacheFull) > 1u);
    CHECK(lines.size() == linesNaming(lines, "[Warning] JoltWorld: Jolt dropped contacts"));
    REQUIRE_FALSE(lines.empty());
    CHECK(lines.front().find("physics step 1 ") != std::string::npos);

    const size_t linesWithinWindow = lines.size();
    g_fakeNowSeconds += 1.5;
    world.step(kDt);
    CHECK(linesNaming(lines, "ContactConstraintsFull") == 2u);
    CHECK(lines.size() > linesWithinWindow);
    CHECK(lines.back().find("since the last line") != std::string::npos);
}

#endif // WITH_LOW_LEVEL_TESTS
