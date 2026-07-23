// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <cstdint>
#include <functional>
#include <type_traits>
#include <unordered_map>

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationManagerConcept.h"
// Sibling-relative, matching the Determinism/ harness convention — the test
// tree is not on an include root under either UBT or the standalone CMake build.
#include "StandaloneTestHandle.h"

// ---------------------------------------------------------------------------
// Coverage for the NetConfig<C> concept + FStandaloneTestHandle
// (task 2 of og-netcode-v2-arch-latency — see proposal §2.1 / D3.2).
//
// Follows the compile-time-probe shape of SimulatableListTest / StorageViewTest
// / SystemsExecutorTest: the static_asserts ARE the coverage, and the TEST_CASEs
// re-affirm the key relations as runtime REQUIREs so the module reports
// observable assertions under [@og].
//
// The negative probes below are what make this test worth more than the
// static_assert already sitting in StandaloneTestHandle.h. A concept that is
// accidentally vacuous — one whose requires-clause silently accepts anything —
// would pass every positive assertion in this file. Each broken adapter drops
// exactly ONE requirement, so a regression that loosens the concept shows up as
// a specific failing negative rather than as silence.
// ---------------------------------------------------------------------------

namespace netConfigProbes
{
    // --- Broken adapter A: Address has no isAlive() ------------------------
    // The AC's named case. Everything else is well-formed, so this isolates the
    // liveness requirement — without which the per-connection tables could
    // never reap dropped connections.
    struct NoIsAliveHandle
    {
        std::uint32_t id = 0;
        friend bool operator==(const NoIsAliveHandle&, const NoIsAliveHandle&) = default;
    };

    // --- Broken adapter B: Address is not hashable -------------------------
    // No std::hash specialization. Isolates the requirement that makes the
    // handle usable as an unordered_map key.
    struct UnhashableHandle
    {
        std::uint32_t id = 0;
        bool isAlive() const { return true; }
        friend bool operator==(const UnhashableHandle&, const UnhashableHandle&) = default;
    };

    // --- Broken adapter C: Address is not std::regular ---------------------
    // Move-only (copy deleted), so it fails std::copyable and therefore
    // std::regular, even though it is hashable and has isAlive().
    struct MoveOnlyHandle
    {
        std::uint32_t id = 0;

        MoveOnlyHandle() = default;
        MoveOnlyHandle(const MoveOnlyHandle&) = delete;
        MoveOnlyHandle& operator=(const MoveOnlyHandle&) = delete;
        MoveOnlyHandle(MoveOnlyHandle&&) = default;
        MoveOnlyHandle& operator=(MoveOnlyHandle&&) = default;

        bool isAlive() const { return true; }
        friend bool operator==(const MoveOnlyHandle&, const MoveOnlyHandle&) = default;
    };
}

namespace std
{
    template <>
    struct hash<netConfigProbes::NoIsAliveHandle>
    {
        std::size_t operator()(const netConfigProbes::NoIsAliveHandle& h) const noexcept
        {
            return std::hash<std::uint32_t>{}(h.id);
        }
    };

    template <>
    struct hash<netConfigProbes::MoveOnlyHandle>
    {
        std::size_t operator()(const netConfigProbes::MoveOnlyHandle& h) const noexcept
        {
            return std::hash<std::uint32_t>{}(h.id);
        }
    };
}

namespace netConfigProbes
{
    struct NoIsAliveNetConfig
    {
        using Address = NoIsAliveHandle;
        static constexpr int tickFrequencyHz = 60;
    };

    struct UnhashableNetConfig
    {
        using Address = UnhashableHandle;
        static constexpr int tickFrequencyHz = 60;
    };

    struct MoveOnlyNetConfig
    {
        using Address = MoveOnlyHandle;
        static constexpr int tickFrequencyHz = 60;
    };

    // --- Broken adapter D: no Address member type at all --------------------
    struct NoAddressNetConfig
    {
        static constexpr int tickFrequencyHz = 60;
    };

    // --- Broken adapter E: no tickFrequencyHz -------------------------------
    struct NoTickFrequencyNetConfig
    {
        using Address = FStandaloneTestHandle;
    };
}

// --- Positive: the engine-free binding satisfies the concept ---------------
static_assert(NetConfig<StandaloneTestNetConfig>);

// --- Negatives: each broken adapter is rejected for exactly one reason -----
static_assert(!NetConfig<netConfigProbes::NoIsAliveHandle>);          // a handle is not a config
static_assert(!NetConfig<netConfigProbes::NoIsAliveNetConfig>);       // Address lacks isAlive()
static_assert(!NetConfig<netConfigProbes::UnhashableNetConfig>);      // Address lacks std::hash
static_assert(!NetConfig<netConfigProbes::MoveOnlyNetConfig>);        // Address is not std::regular
static_assert(!NetConfig<netConfigProbes::NoAddressNetConfig>);       // no Address member type
static_assert(!NetConfig<netConfigProbes::NoTickFrequencyNetConfig>); // no tickFrequencyHz
static_assert(!NetConfig<int>);                      // a scalar is not a config

// --- The Address contract holds on the standalone handle itself ------------
static_assert(std::regular<FStandaloneTestHandle>);
static_assert(std::default_initializable<FStandaloneTestHandle>);
static_assert(std::is_same_v<StandaloneTestNetConfig::Address, FStandaloneTestHandle>);

TEST_CASE("NetConfig.ConceptAcceptsStandaloneBinding", "[Network][NetConfig]")
{
    // Runtime anchor for the compile-time asserts above.
    REQUIRE(NetConfig<StandaloneTestNetConfig>);
    REQUIRE(std::regular<FStandaloneTestHandle>);

    // T1 coherence check: the concept's compile-time tick rate must agree with
    // the runtime TimeConfig::tickFrequency default (60 Hz, Stage 2). If a
    // future task changes one and not the other, this fails.
    REQUIRE(StandaloneTestNetConfig::tickFrequencyHz == 60);
}

TEST_CASE("NetConfig.ConceptRejectsBrokenAdapters", "[Network][NetConfig]")
{
    // The AC's named negative: dropping isAlive() must break the concept.
    // Evaluated as a requires-expression yielding bool, so the rejection is
    // observable at runtime rather than only as a compile error.
    REQUIRE_FALSE(NetConfig<netConfigProbes::NoIsAliveNetConfig>);
    REQUIRE_FALSE(NetConfig<netConfigProbes::UnhashableNetConfig>);
    REQUIRE_FALSE(NetConfig<netConfigProbes::MoveOnlyNetConfig>);
    REQUIRE_FALSE(NetConfig<netConfigProbes::NoAddressNetConfig>);
    REQUIRE_FALSE(NetConfig<netConfigProbes::NoTickFrequencyNetConfig>);

    // Guard against a vacuous concept: the same probe that accepts the good
    // binding must reject a plain scalar.
    REQUIRE(NetConfig<StandaloneTestNetConfig>);
    REQUIRE_FALSE(NetConfig<int>);
}

TEST_CASE("NetConfig.StandaloneHandleIsRegular", "[Network][NetConfig]")
{
    SECTION("default construction yields the sentinel, which is not alive")
    {
        const FStandaloneTestHandle sentinel;
        REQUIRE(sentinel.id == 0u);
        REQUIRE_FALSE(sentinel.isAlive());
        // Sentinels collapse to one value — the local/no-wire table entry.
        REQUIRE(sentinel == FStandaloneTestHandle{});
    }

    SECTION("copy preserves value and equality")
    {
        const FStandaloneTestHandle original{7u, true};
        const FStandaloneTestHandle copy = original;

        REQUIRE(copy == original);
        REQUIRE(copy.id == 7u);
        REQUIRE(copy.isAlive());
    }

    SECTION("assignment overwrites both members")
    {
        FStandaloneTestHandle target{1u, true};
        const FStandaloneTestHandle source{2u, false};
        target = source;

        REQUIRE(target == source);
        REQUIRE(target.id == 2u);
        REQUIRE_FALSE(target.isAlive());
    }

    SECTION("equality is memberwise — aliveBit is part of identity")
    {
        const FStandaloneTestHandle live{3u, true};
        const FStandaloneTestHandle dead{3u, false};

        REQUIRE_FALSE(live == dead);
        REQUIRE(live != dead);
        REQUIRE(live == FStandaloneTestHandle{3u, true});
    }

    SECTION("hash is consistent with equality")
    {
        const std::hash<FStandaloneTestHandle> hasher;
        const FStandaloneTestHandle a{42u, true};
        const FStandaloneTestHandle b{42u, true};

        // The binding contract: equal values MUST hash equal.
        REQUIRE(hasher(a) == hasher(b));
        // Not a correctness requirement (collisions are legal), but a cheap
        // canary that both members actually reach the hash — a hash ignoring
        // aliveBit would make these collide and defeat stale-entry detection.
        REQUIRE(hasher(a) != hasher(FStandaloneTestHandle{42u, false}));
    }

    SECTION("usable as an unordered_map key — the reason hashing is required")
    {
        // Stands in for the ConnectionTierTable<Address> that T4 builds.
        std::unordered_map<FStandaloneTestHandle, int> tierByConnection;
        tierByConnection[FStandaloneTestHandle{1u, true}] = 0;
        tierByConnection[FStandaloneTestHandle{2u, true}] = 3;

        REQUIRE(tierByConnection.size() == 2u);
        REQUIRE(tierByConnection.at(FStandaloneTestHandle{1u, true}) == 0);
        REQUIRE(tierByConnection.at(FStandaloneTestHandle{2u, true}) == 3);

        // Re-inserting an equal key updates rather than duplicates.
        tierByConnection[FStandaloneTestHandle{1u, true}] = 2;
        REQUIRE(tierByConnection.size() == 2u);
        REQUIRE(tierByConnection.at(FStandaloneTestHandle{1u, true}) == 2);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
