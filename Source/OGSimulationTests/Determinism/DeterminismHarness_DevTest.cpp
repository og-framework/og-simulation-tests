// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "DeterminismHarness.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <span>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// Determinism harness — DevTest-mode tests (proposal §9.1).
//
// Tagged [Determinism][DevTest]. This tag is intentionally NOT in the [@og] alias
// (OgTagAliases.cpp), so these are OPT-IN only — `oglltest simulation` (default
// [@og]) does NOT pick them up. Run them explicitly:
//     oglltest simulation -Filter '[Determinism][DevTest]'
//
// These are longer / heavier seeded-grid runs plus a negative control proving the
// divergence detector actually fires on injected non-determinism. Trivial POD
// State/Input types, as in the Production file.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    struct TestState { std::int64_t accumulator = 0; };
    struct TestInput { std::int64_t increment = 0; };

    using Cache = StateCorrectionCache<TestState, TestInput>;

    // Seed shared by all DevTest seeded grids (Backlog Task 3).
    constexpr std::uint32_t kDevTestSeed = 0x0FB1AC1Eu;

    std::function<void(const char*)> nullLogger()
    {
        return [](const char*) {};
    }

    Cache::IntegrateFn accumulateIntegrate()
    {
        return [](std::uint32_t, const TestState& prev, const TestInput& in)
        {
            return TestState{ prev.accumulator + in.increment };
        };
    }

    // Deterministic pseudo-random input grid: a fresh std::mt19937 seeded with the
    // same value always yields the same sequence, so two calls with the same seed
    // produce identical grids.
    std::vector<TestInput> seededGrid(std::size_t count, std::uint32_t seed)
    {
        std::mt19937 rng(seed);
        std::uniform_int_distribution<std::int64_t> dist(-1000, 1000);

        std::vector<TestInput> grid;
        grid.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
            grid.push_back(TestInput{ dist(rng) });
        return grid;
    }

    std::span<const TestInput> asSpan(const std::vector<TestInput>& v)
    {
        return std::span<const TestInput>(v.data(), v.size());
    }
} // namespace

TEST_CASE("Determinism.Cache.RandomInputGrid600Ticks", "[Determinism][DevTest]")
{
    // 600-tick seeded grid; two runs with the same seed must produce identical
    // (per-tick) checksum sequences.
    const std::vector<TestInput> firstGrid = seededGrid(600, kDevTestSeed);
    const std::vector<TestInput> secondGrid = seededGrid(600, kDevTestSeed);
    REQUIRE(firstGrid.size() == 600u);

    Cache first(nullLogger(), accumulateIntegrate());
    std::vector<std::uint32_t> firstChecksums;
    og::determinism::runDeterminismLoop(first, asSpan(firstGrid), 1u, firstChecksums);

    Cache second(nullLogger(), accumulateIntegrate());
    std::vector<std::uint32_t> secondChecksums;
    og::determinism::runDeterminismLoop(second, asSpan(secondGrid), 1u, secondChecksums);

    REQUIRE(firstChecksums.size() == 600u);
    REQUIRE_FALSE(og::determinism::firstChecksumDivergence(firstChecksums, secondChecksums).has_value());
}

TEST_CASE("Determinism.Cache.DivergenceDetectionWhenIntegrateNonDeterministic", "[Determinism][DevTest]")
{
    // Negative control: inject a NON-deterministic integrate functor and confirm
    // the harness's divergence detector actually fires. The functor's output
    // depends on hidden mutable state (a shared counter that is NEVER reset
    // between the two runs), so the same inputs produce different states across
    // runs. A shared counter is used instead of std::random_device so the
    // divergence is GUARANTEED (non-flaky) while still exercising the exact
    // same-inputs / different-output path random_device would.
    const std::vector<TestInput> grid = seededGrid(120, kDevTestSeed);

    auto counter = std::make_shared<std::int64_t>(0);
    Cache::IntegrateFn nonDeterministic =
        [counter](std::uint32_t, const TestState& prev, const TestInput& in)
        {
            ++(*counter); // hidden state carried across both runs via shared_ptr
            return TestState{ prev.accumulator + in.increment + *counter };
        };

    // Both caches receive copies of the SAME functor, which share the same
    // underlying counter — so run two continues where run one left off.
    Cache first(nullLogger(), nonDeterministic);
    std::vector<std::uint32_t> firstChecksums;
    og::determinism::runDeterminismLoop(first, asSpan(grid), 1u, firstChecksums);

    Cache second(nullLogger(), nonDeterministic);
    std::vector<std::uint32_t> secondChecksums;
    og::determinism::runDeterminismLoop(second, asSpan(grid), 1u, secondChecksums);

    const std::optional<std::size_t> divergence =
        og::determinism::firstChecksumDivergence(firstChecksums, secondChecksums);
    REQUIRE(divergence.has_value());
}

TEST_CASE("Determinism.Cache.LongTorturePass", "[Determinism][DevTest]")
{
    // 6000-tick run on the seeded grid; the checksum at the final tick must be
    // stable across consecutive reruns (and the whole sequence must match).
    const std::vector<TestInput> grid = seededGrid(6000, kDevTestSeed);

    Cache first(nullLogger(), accumulateIntegrate());
    std::vector<std::uint32_t> firstChecksums;
    og::determinism::runDeterminismLoop(first, asSpan(grid), 1u, firstChecksums);

    Cache second(nullLogger(), accumulateIntegrate());
    std::vector<std::uint32_t> secondChecksums;
    og::determinism::runDeterminismLoop(second, asSpan(grid), 1u, secondChecksums);

    REQUIRE(firstChecksums.size() == 6000u);
    REQUIRE(secondChecksums.size() == 6000u);
    REQUIRE(firstChecksums.back() == secondChecksums.back()); // checksum @ tick 6000
    REQUIRE_FALSE(og::determinism::firstChecksumDivergence(firstChecksums, secondChecksums).has_value());
}

#endif // WITH_LOW_LEVEL_TESTS
