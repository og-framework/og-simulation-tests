// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "DeterminismHarness.h"

#include <cstdint>
#include <functional>
#include <span>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// Determinism harness — Production-mode tests (proposal §9.1).
//
// Tagged [Determinism][Production]; the [Determinism][Production] AND-spec is part
// of the [@og] alias (OgTagAliases.cpp), so these run by default under
// `oglltest simulation`. They use TRIVIAL POD State/Input types — this is
// API-level validation of the cache + checksum + integrate-driver mechanism, NOT
// the brawler simulation (Stage 3).
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // Trivial Plain-Old-Data state/input (Backlog Task 3): int64 accumulator /
    // int64 increment. Not Serializable and not a SimulationComposite, so
    // compute_checksum CRCs their raw bytes (the deterministic fallback path).
    struct TestState { std::int64_t accumulator = 0; };
    struct TestInput { std::int64_t increment = 0; };

    using Cache = StateCorrectionCache<TestState, TestInput>;

    std::function<void(const char*)> nullLogger()
    {
        return [](const char*) {};
    }

    // Deterministic accumulate: newState = prevState + input.
    Cache::IntegrateFn accumulateIntegrate()
    {
        return [](std::uint32_t, const TestState& prev, const TestInput& in)
        {
            return TestState{ prev.accumulator + in.increment };
        };
    }

    // Fixed, non-random input grid (deterministic, no seed needed).
    std::vector<TestInput> fixedGrid(std::size_t count)
    {
        std::vector<TestInput> grid;
        grid.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            // A spread of small positive values; +1 keeps every increment non-zero.
            grid.push_back(TestInput{ static_cast<std::int64_t>((i * 7u) % 251u) + 1 });
        }
        return grid;
    }

    std::span<const TestInput> asSpan(const std::vector<TestInput>& v)
    {
        return std::span<const TestInput>(v.data(), v.size());
    }
} // namespace

TEST_CASE("Determinism.Cache.AdvanceFrameDeterministic", "[Determinism][Production]")
{
    // Drive 60 ticks through advance_frame; two independent runs over the same
    // input grid must produce bit-identical checksum sequences.
    const std::vector<TestInput> grid = fixedGrid(60);

    Cache first(nullLogger(), accumulateIntegrate());
    std::vector<std::uint32_t> firstChecksums;
    og::determinism::runDeterminismLoop(first, asSpan(grid), 1u, firstChecksums);

    Cache second(nullLogger(), accumulateIntegrate());
    std::vector<std::uint32_t> secondChecksums;
    og::determinism::runDeterminismLoop(second, asSpan(grid), 1u, secondChecksums);

    REQUIRE(firstChecksums.size() == 60u);
    REQUIRE(secondChecksums.size() == 60u);
    REQUIRE(firstChecksums == secondChecksums);
    REQUIRE_FALSE(og::determinism::firstChecksumDivergence(firstChecksums, secondChecksums).has_value());
}

TEST_CASE("Determinism.Cache.SaveLoadRoundTrip", "[Determinism][Production]")
{
    Cache cache(nullLogger());

    const TestState saved{ 1234567890123LL };
    cache.save_snapshot(7u, saved);

    TestState loaded{};
    REQUIRE(cache.load_snapshot(7u, loaded));
    REQUIRE(loaded.accumulator == saved.accumulator);

    // A tick that was never stored is reported as a miss.
    TestState missOut{};
    REQUIRE_FALSE(cache.load_snapshot(9999u, missOut));
}

TEST_CASE("Determinism.Cache.AdvanceFrameMatchesManualIntegrate", "[Determinism][Production]")
{
    // 30 ticks via advance_frame must yield the same final-state checksum as 30
    // manual pushPredictionTick + pushPredictionInput + pushPredictionState calls
    // using the same integrate functor.
    const std::vector<TestInput> grid = fixedGrid(30);
    const Cache::IntegrateFn integrate = accumulateIntegrate();

    Cache driven(nullLogger(), integrate);
    std::vector<std::uint32_t> drivenChecksums;
    og::determinism::runDeterminismLoop(driven, asSpan(grid), 1u, drivenChecksums);

    // Manual path: mirror advance_frame's read-integrate-push sequence exactly.
    Cache manual(nullLogger());
    std::uint32_t tick = 1u;
    for (const TestInput& in : grid)
    {
        const std::uint32_t prevIndex = manual.getCacheIndex(manual.getPredictionTick());
        const TestState prev = manual.getState(prevIndex);
        const TestState next = integrate(tick, prev, in);
        manual.pushPredictionTick(tick);
        manual.pushPredictionInput(in);
        manual.pushPredictionState(next);
        ++tick;
    }

    const std::uint32_t finalTick = 30u; // startTick(1) + 30 ticks - 1
    REQUIRE(driven.compute_checksum(finalTick) == manual.compute_checksum(finalTick));
    REQUIRE(drivenChecksums.back() == manual.compute_checksum(finalTick));
}

TEST_CASE("Determinism.Cache.ChecksumIsSensitiveToStateBits", "[Determinism][Production]")
{
    // Flipping one bit in the integrate-functor's output state must change the
    // resulting checksum.
    const std::vector<TestInput> grid = fixedGrid(16);

    Cache baseline(nullLogger(), accumulateIntegrate());
    std::vector<std::uint32_t> baselineChecksums;
    og::determinism::runDeterminismLoop(baseline, asSpan(grid), 1u, baselineChecksums);

    auto bitFlippedIntegrate = [](std::uint32_t, const TestState& prev, const TestInput& in)
    {
        // Same accumulate, but flip one bit of the produced state.
        return TestState{ (prev.accumulator + in.increment) ^ std::int64_t(1) };
    };
    Cache flipped(nullLogger(), Cache::IntegrateFn(bitFlippedIntegrate));
    std::vector<std::uint32_t> flippedChecksums;
    og::determinism::runDeterminismLoop(flipped, asSpan(grid), 1u, flippedChecksums);

    // The one-bit difference in the integrate output must surface as a checksum
    // difference at some tick. (We assert divergence-somewhere rather than only on
    // the final tick: with a pure-accumulate integrator the toggled low bit can
    // coincidentally re-cancel at the last tick, but the per-tick sequence still
    // diverges — which is exactly the checksum sensitivity we're verifying.)
    REQUIRE(og::determinism::firstChecksumDivergence(baselineChecksums, flippedChecksums).has_value());
}

TEST_CASE("Determinism.Cache.ChecksumIsSensitiveToInputBits", "[Determinism][Production]")
{
    // Flipping one bit in a single input value must change the resulting state's
    // checksum.
    const std::vector<TestInput> baseGrid = fixedGrid(16);

    Cache baseline(nullLogger(), accumulateIntegrate());
    std::vector<std::uint32_t> baselineChecksums;
    og::determinism::runDeterminismLoop(baseline, asSpan(baseGrid), 1u, baselineChecksums);

    std::vector<TestInput> mutatedGrid = baseGrid;
    mutatedGrid[5].increment ^= std::int64_t(1); // flip one bit of one input

    Cache mutated(nullLogger(), accumulateIntegrate());
    std::vector<std::uint32_t> mutatedChecksums;
    og::determinism::runDeterminismLoop(mutated, asSpan(mutatedGrid), 1u, mutatedChecksums);

    // The flipped input bit changes the integrated state from that tick onward, so
    // the checksum sequences must diverge.
    REQUIRE(og::determinism::firstChecksumDivergence(baselineChecksums, mutatedChecksums).has_value());
}

#endif // WITH_LOW_LEVEL_TESTS
