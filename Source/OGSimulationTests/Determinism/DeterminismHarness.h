#pragma once
// SPDX-License-Identifier: MPL-2.0

//////////////////////////////////////////////////////////////////////////////
// DeterminismHarness — Stage 0 determinism test facade (proposal §9.1).
//
// A tiny header-only driver that pushes an input grid through the
// StateCorrectionCache 4-method external API (proposal §2.2 — pulled forward to
// Stage 0 by Task 2) and collects a per-tick CRC-32 checksum sequence. The
// Production and DevTest TEST_CASEs (DeterminismHarness_Production.cpp /
// DeterminismHarness_DevTest.cpp) use this to assert that:
//   - the same inputs produce the same checksum sequence across reruns
//     (determinism), and
//   - injected non-determinism is actually detectable (the divergence signal is
//     real, not a no-op).
//
// The harness is intentionally agnostic to the State/Input types — Stage 0 uses
// trivial POD test types (struct { int64 } accumulator/increment). It validates
// the cache + checksum + integrate-driver mechanism, NOT the brawler simulation;
// brawler-coupled determinism tests follow in Stage 3.
//////////////////////////////////////////////////////////////////////////////

#include "OGSimulation/CorrectionCache.h"

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace og::determinism
{
    namespace detail
    {
        // Compile-time proof that the harness can reach the StateCorrectionCache
        // 4-method external API (proposal §2.2). If any of save_snapshot /
        // load_snapshot / advance_frame / compute_checksum is renamed, removed, or
        // re-signed, this concept stops being satisfied and the static_assert below
        // fails loudly at header-parse time.
        template <typename S, typename I>
        concept HasFourMethodApi = requires(
            StateCorrectionCache<S, I>& cache,
            const StateCorrectionCache<S, I>& constCache,
            S state, const I input, std::uint32_t tick)
        {
            cache.save_snapshot(tick, state);
            { constCache.load_snapshot(tick, state) } -> std::convertible_to<bool>;
            cache.advance_frame(tick, input);
            { constCache.compute_checksum(tick) } -> std::convertible_to<std::uint32_t>;
        };

        // Trivial POD probe types used solely to instantiate the concept check at
        // header-parse time (no TEST_CASE needs to compile for this to fire).
        struct ProbeState { std::int64_t value; };
        struct ProbeInput { std::int64_t value; };

        static_assert(HasFourMethodApi<ProbeState, ProbeInput>,
            "DeterminismHarness requires the StateCorrectionCache 4-method external "
            "API (save_snapshot / load_snapshot / advance_frame / compute_checksum). "
            "See proposal §2.2 / Backlog Task 2.");
    } // namespace detail

    // Drives `inputs.size()` externally-triggered sim steps through the cache via
    // advance_frame, starting at `startTick` and incrementing by one each step.
    // After each step the current tick's CRC-32 checksum is appended to
    // `outChecksums` (cleared first). The injected integrate functor lives on the
    // cache (2-arg constructor) — the harness owns no integration logic.
    //
    // Determinism contract: two caches constructed with equivalent integrate
    // functors, driven with the same `inputs` + `startTick`, MUST yield identical
    // `outChecksums`. A mismatch means the (cache, integrate, checksum) pipeline is
    // non-deterministic — exactly what the harness is built to surface.
    template <typename S, typename I>
    void runDeterminismLoop(StateCorrectionCache<S, I>& cache,
                            std::span<const I> inputs,
                            std::uint32_t startTick,
                            std::vector<std::uint32_t>& outChecksums)
    {
        static_assert(detail::HasFourMethodApi<S, I>,
            "runDeterminismLoop requires the StateCorrectionCache 4-method API.");

        outChecksums.clear();
        outChecksums.reserve(inputs.size());

        std::uint32_t tick = startTick;
        for (const I& input : inputs)
        {
            cache.advance_frame(tick, input);
            outChecksums.push_back(cache.compute_checksum(tick));
            ++tick;
        }
    }

    // Returns the index of the first differing checksum between two runs, or the
    // common length if one run is a strict prefix of the other, or std::nullopt if
    // the sequences are identical. This is the harness's divergence detector: the
    // DevTest non-determinism case asserts it returns a value; the determinism
    // cases assert it returns nullopt.
    inline std::optional<std::size_t> firstChecksumDivergence(
        const std::vector<std::uint32_t>& a,
        const std::vector<std::uint32_t>& b)
    {
        const std::size_t common = std::min(a.size(), b.size());
        for (std::size_t i = 0; i < common; ++i)
        {
            if (a[i] != b[i])
                return i;
        }
        if (a.size() != b.size())
            return common;
        return std::nullopt;
    }
} // namespace og::determinism
