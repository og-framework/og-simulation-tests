// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "OGSimulation/SimulationManagerConcept.h"

// ---------------------------------------------------------------------------
// FStandaloneTestHandle + StandaloneTestNetConfig — the ENGINE-FREE
// NetConfig::Address binding used by the Catch2 suite.
// (Stage 3 / D3.2; proposal_ogbrawler_netcode.md §2.1.)
//
// The per-connection templates this initiative builds (ConnectionTierTable in
// T4, ServerInputDelayQueue in T5) are engine-agnostic and live in the sim
// core, but their only PRODUCTION Address binding — FUEConnectionHandle — drags
// in UNetConnection and the whole engine. This suite does not link
// OGSimulationUnreal, so it supplies its own Address instead: a plain POD pair
// of an id and an explicit liveness bit.
//
// The explicit `aliveBit` is the point. FUEConnectionHandle's liveness is
// decided by the engine's object lifetime and cannot be scripted from a test;
// here a test flips a bool to stage exactly the connection-drop and
// stale-entry-reaping scenarios T4/T5 need, deterministically and with no
// engine present. Two handles with the same id but different aliveBit are
// deliberately DISTINCT values, so a table cannot silently conflate a live
// connection with the dead entry it replaced.
// ---------------------------------------------------------------------------
struct FStandaloneTestHandle
{
    std::uint32_t id = 0;
    bool aliveBit = false;

    FStandaloneTestHandle() = default;

    FStandaloneTestHandle(std::uint32_t inId, bool inAlive)
        : id(inId)
        , aliveBit(inAlive)
    {
    }

    bool isAlive() const
    {
        return aliveBit;
    }

    // Defaulted: gives std::equality_comparable (and operator!= via the C++20
    // rewrite) with memberwise semantics.
    friend bool operator==(const FStandaloneTestHandle&, const FStandaloneTestHandle&) = default;
};

namespace std
{
    template <>
    struct hash<FStandaloneTestHandle>
    {
        std::size_t operator()(const FStandaloneTestHandle& handle) const noexcept
        {
            // Both members participate, matching operator==: equal values must
            // hash equal, and the two states of one id must be able to differ.
            const std::size_t idHash = std::hash<std::uint32_t>{}(handle.id);
            const std::size_t aliveHash = std::hash<bool>{}(handle.aliveBit);
            return idHash ^ (aliveHash + 0x9e3779b9u + (idHash << 6) + (idHash >> 2));
        }
    };
}

// The engine-free counterpart to UEBrawlerNetConfig. Same tick rate, so a test
// asserting against tickFrequencyHz exercises the same value production does.
struct StandaloneTestNetConfig
{
    using Address = FStandaloneTestHandle;
    static constexpr int tickFrequencyHz = 60;
};

static_assert(NetConfig<StandaloneTestNetConfig>,
    "StandaloneTestNetConfig must satisfy the NetConfig concept");
