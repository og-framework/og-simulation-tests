// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <cstdint>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/CorrectionRotation.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/SimulationManager.h"

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay / T39] THE STATE-CADENCE KERNEL.
//
// `SimulationNetSync::sendCorrectionAll` writes K of the N registered authority
// writers' correction buffers per tick, round-robin. This suite pins the pure
// arithmetic that decides WHO — coverage, wrap, the K >= N degeneracy, and the
// shared clamp — plus the four-step config path's setter half.
//
// WHY THE ARITHMETIC LIVES IN ITS OWN NAMESPACE AND ITS OWN SUITE. The method
// that calls it is on the `SimulationNetSync` class template, which can only be
// instantiated by a suite that binds `SimulatableOwnerTraits` to concrete owners
// — that is og-brawler-tests, and every existing sendCorrectionAll case lives
// there for exactly that reason. Pushing the selection rule down into a free
// function lets THIS suite exhaustively sweep (N, K) pairs with no mock
// character at all, while og-brawler-tests keeps one integration case proving
// the send path actually honours it. Neither half is sufficient alone: a green
// kernel with an unwired caller ships every-frame state, and a green integration
// case at one (N, K) proves nothing about the wrap.
//
// WHAT THIS SUITE DELIBERATELY DOES NOT COVER: the ini intake and the
// `[StateRotation]` proof line, both of which are UE-side composition-root code
// (SimulationManagerUImpl) and invisible to a pure-C++ target; and the packet
// arithmetic that motivated the cadence, which is the round-vs-packet LLT in
// og-brawler-tests.
// ---------------------------------------------------------------------------

namespace
{
    // Replays the exact loop shape sendCorrectionAll runs: walk positions
    // 0..N-1, ask the predicate, then advance the monotonic base once. Returns,
    // per round, the set of positions that were written.
    std::vector<std::vector<std::size_t>> simulateRounds(
        std::size_t writerCount, std::int32_t k, std::size_t rounds,
        std::size_t initialBase = 0u)
    {
        std::vector<std::vector<std::size_t>> written;
        std::size_t base = initialBase;

        for (std::size_t r = 0; r < rounds; ++r)
        {
            std::vector<std::size_t> thisRound;
            for (std::size_t i = 0; i < writerCount; ++i)
            {
                if (correctionRotation::isInRound(i, base, writerCount, k))
                    thisRound.push_back(i);
            }
            written.push_back(std::move(thisRound));
            base = correctionRotation::advanceRound(base, k);
        }

        return written;
    }

    // The coverage bound the caller relies on: with N writers and an effective
    // window of min(clampK(K), N), every position is written at least once within
    // this many consecutive rounds.
    std::size_t coverageBound(std::size_t writerCount, std::int32_t k)
    {
        const std::size_t clamped = static_cast<std::size_t>(correctionRotation::clampK(k));
        const std::size_t effective = (clamped > writerCount) ? writerCount : clamped;
        return (writerCount + effective - 1u) / effective;
    }

    // The peers SimulationManager holds by reference. Duck-typed and instantiated
    // only where used, so an authority manager needs only what the ctor touches —
    // mirrors the rig in RelayRedundancyDepthTest.cpp, deliberately, so the two
    // sibling knobs' setter cases read the same.
    //
    // [item 87] `wipeAllForResync` LEFT MockNetSync for MockInputResolution —
    // it moved off the real `SimulationNetSync` onto the resolution peer at
    // the same item, and the manager's ctor now calls it there instead.
    struct MockIntegrationExec {};
    struct MockNetSync {};
    struct MockInputResolution { void wipeAllForResync(unsigned int) {} };
    struct MockReconciliation{ void wipeAllForResync(unsigned int) {} };
    struct MockSystemsExec {};
    struct MockStorage {};
    struct MockStaticData {};

    using TestManager = SimulationManager<
        MockIntegrationExec, MockNetSync, MockInputResolution, MockReconciliation, MockSystemsExec,
        MockStorage, MockStaticData>;

    struct ManagerRig
    {
        MockIntegrationExec integration{};
        MockNetSync         netSync{};
        MockInputResolution inputResolution{};
        MockReconciliation  reconciliation{};
        MockSystemsExec     systemsExec{};
        MockStorage         storage{};
        MockStaticData      staticData{};

        TestManager manager{
            /*shouldRunPrediction=*/false,
            /*tickFrequency (fixed dt, seconds)=*/1.0 / 60.0,
            TestManager::Params{ integration, netSync, inputResolution, reconciliation, systemsExec,
                                 storage, staticData, nullptr } };
    };
}

TEST_CASE("CorrectionRotation: clampK is the shared intake guard and is idempotent",
          "[Network][CorrectionRotation]")
{
    // The three cases the config path must survive. 0 and negatives clamp UP:
    // a K of 0 is not "off", it is a correction channel that never publishes,
    // which is a permanent desync — the same ruling relayedInputRing::clampDepth
    // makes for a depth-0 ring.
    REQUIRE(correctionRotation::clampK(0) == 1);
    REQUIRE(correctionRotation::clampK(-1) == 1);
    REQUIRE(correctionRotation::clampK(-999) == 1);

    // The ceiling only has to stop a typo becoming nonsense; K >= N is the
    // legitimate every-frame setting, so 16 is deliberately above any supported
    // character count rather than tight against it.
    REQUIRE(correctionRotation::clampK(99) == 16);
    REQUIRE(correctionRotation::clampK(17) == 16);

    // In-range values pass through untouched, including both bounds.
    REQUIRE(correctionRotation::clampK(1) == 1);
    REQUIRE(correctionRotation::clampK(2) == 2);
    REQUIRE(correctionRotation::clampK(16) == 16);

    // IDEMPOTENCE is what makes calling this at the ini intake, at the setter AND
    // inside the predicate safe rather than merely redundant.
    for (std::int32_t raw : { -5, 0, 1, 2, 7, 16, 40 })
    {
        const std::int32_t once = correctionRotation::clampK(raw);
        REQUIRE(correctionRotation::clampK(once) == once);
    }

    // constexpr, so the clamp can guard a compile-time constant too.
    static_assert(correctionRotation::clampK(0) == 1);
    static_assert(correctionRotation::clampK(99) == 16);
}

TEST_CASE("CorrectionRotation: setCorrectionRotationK is the one writable door and clamps",
          "[Network][CorrectionRotation]")
{
    // The setter half of the four-step config path (ini intake -> shared clamp ->
    // setter -> proof line). The intake and the proof line are UE-side; this is
    // the part a pure-C++ target can reach.
    ManagerRig rig;

    // [T34] The compiled default is 1 for the pre-diet window (was 2 at T39) — see
    // TimeConfig's block. This case is about the SETTER, so the starting value is
    // read from the same source of truth rather than typed, and the defaults gate
    // in TimeConfigDefaultsTest owns the value itself.
    REQUIRE(rig.manager.getTimeConfig().correctionRotationK == TimeConfig{}.correctionRotationK);
    REQUIRE(rig.manager.getTimeConfig().correctionRotationK == 1);

    rig.manager.setCorrectionRotationK(4);
    REQUIRE(rig.manager.getTimeConfig().correctionRotationK == 4);

    // Out of range in both directions lands on the bound, not on the request —
    // the same guard the intake calls, so an operator typo cannot install a
    // cadence the predicate would then silently re-clamp behind the log line.
    rig.manager.setCorrectionRotationK(0);
    REQUIRE(rig.manager.getTimeConfig().correctionRotationK == 1);

    rig.manager.setCorrectionRotationK(-1);
    REQUIRE(rig.manager.getTimeConfig().correctionRotationK == 1);

    rig.manager.setCorrectionRotationK(99);
    REQUIRE(rig.manager.getTimeConfig().correctionRotationK == correctionRotation::kMaxK);
}

TEST_CASE("CorrectionRotation: K >= N degenerates to every-frame",
          "[Network][CorrectionRotation]")
{
    // THE BASELINE-PRESERVING PROPERTY. It is why T39's compiled default was 2:
    // at two characters K=2 writes both every tick, i.e. the pre-T39 behaviour
    // byte-for-byte. [T34] The default is 1 for the pre-diet window, so the
    // property is now exercised here rather than shipped — item 40 restores it.
    for (std::size_t n : { std::size_t{1}, std::size_t{2}, std::size_t{3} })
    {
        const auto rounds = simulateRounds(n, static_cast<std::int32_t>(n), 5u);
        for (const auto& r : rounds)
            REQUIRE(r.size() == n);
    }

    // Strictly greater than N is the same statement, and it must not wrap round
    // and double-count: the predicate returns true for every position, once.
    const auto rounds = simulateRounds(/*writerCount=*/3u, /*k=*/16, /*rounds=*/4u);
    for (const auto& r : rounds)
    {
        REQUIRE(r.size() == 3u);
        REQUIRE(r[0] == 0u);
        REQUIRE(r[1] == 1u);
        REQUIRE(r[2] == 2u);
    }
}

TEST_CASE("CorrectionRotation: the shipped cadences are exactly what the design claims",
          "[Network][CorrectionRotation]")
{
    // The design's cadence table is `tickFrequency * K / N` per character
    // (design_task38 §6 Candidate A / §13.2), expressed here as the exact count of
    // writes per character over one full 60-tick second.
    //
    // [T34] AT THE SHIPPED K = 1 (pre-diet) that is 30/20/15 Hz at N = 2/3/4 — the
    // floor of the accepted 15-20 Hz band, and deliberately so: at K = 2 with
    // un-dieted states the second state batch enters Iris's huge-object window
    // (§16.2). The K = 2 column is kept beside it because item 40 restores it and
    // because it is what the archived T39 baselines were taken at; asserting both
    // means the restore cannot land without this table agreeing.
    const TimeConfig defaults;
    const std::int32_t k = defaults.correctionRotationK;
    REQUIRE(k == 1);

    struct Expectation { std::size_t characters; std::size_t writesPerCharacterPerSecond; };
    const Expectation table[] = {
        { 2u, 30u },
        { 3u, 20u },
        { 4u, 15u },
        { 6u, 10u },
    };

    // The post-diet configuration, asserted at the same time so the two are one
    // fact rather than two that can drift.
    const Expectation kTwoTable[] = {
        { 2u, 60u },
        { 3u, 40u },
        { 4u, 30u },
        { 6u, 20u },
    };

    for (const Expectation& e : kTwoTable)
    {
        const auto rounds = simulateRounds(e.characters, /*k=*/2, /*rounds=*/60u);

        std::vector<std::size_t> writesPerCharacter(e.characters, 0u);
        for (const auto& r : rounds)
            for (std::size_t position : r)
                ++writesPerCharacter[position];

        for (std::size_t position = 0; position < e.characters; ++position)
        {
            INFO("K=2 characters=" << e.characters << " position=" << position);
            REQUIRE(writesPerCharacter[position] == e.writesPerCharacterPerSecond);
        }
    }

    for (const Expectation& e : table)
    {
        const auto rounds = simulateRounds(e.characters, k, /*rounds=*/60u);

        std::vector<std::size_t> writesPerCharacter(e.characters, 0u);
        for (const auto& r : rounds)
            for (std::size_t position : r)
                ++writesPerCharacter[position];

        for (std::size_t position = 0; position < e.characters; ++position)
        {
            INFO("characters=" << e.characters << " position=" << position);
            REQUIRE(writesPerCharacter[position] == e.writesPerCharacterPerSecond);
        }
    }
}

TEST_CASE("CorrectionRotation: every writer is written within ceil(N/K) rounds",
          "[Network][CorrectionRotation]")
{
    // ⭐ THE LOAD-BEARING GUARANTEE. The cadence claim "60 * K / N Hz" is only
    // honest if no character can be starved, so this sweeps the whole supported
    // (N, K) space rather than sampling it — including the awkward ones where K
    // does not divide N (N=5,K=2 phases 0,2,4,1,3) and where the two share a
    // factor (N=4,K=2 alternates two fixed halves).
    for (std::size_t n = 1u; n <= 8u; ++n)
    {
        for (std::int32_t k = correctionRotation::kMinK; k <= correctionRotation::kMaxK; ++k)
        {
            const std::size_t bound = coverageBound(n, k);

            // Start from several phases, not just 0: the cursor is monotonic and
            // survives registration changes, so a fresh character must not be
            // starved just because it joined mid-phase.
            for (std::size_t initialBase = 0u; initialBase < n + 3u; ++initialBase)
            {
                const auto rounds = simulateRounds(n, k, bound, initialBase);

                std::vector<bool> seen(n, false);
                for (const auto& r : rounds)
                    for (std::size_t position : r)
                        seen[position] = true;

                for (std::size_t position = 0; position < n; ++position)
                {
                    INFO("n=" << n << " k=" << k << " base=" << initialBase
                         << " bound=" << bound << " position=" << position);
                    REQUIRE(seen[position]);
                }
            }
        }
    }
}

TEST_CASE("CorrectionRotation: the window wraps rather than truncating at the end",
          "[Network][CorrectionRotation]")
{
    // N=3, K=2. Bases 0,2,4 => window starts 0,2,1. The middle round's window
    // starts at the LAST position, so it must WRAP round to position 0 rather
    // than emit a short round — a truncating implementation would write only {2}
    // and starve position 0 by half.
    //
    // The vectors are in ASCENDING POSITION order, not window order, because
    // that is the order sendCorrectionAll's loop visits the map in: it walks
    // positions 0..N-1 and asks the predicate. The wrap is therefore visible as
    // set membership, which is exactly the property that matters.
    const auto rounds = simulateRounds(/*writerCount=*/3u, /*k=*/2, /*rounds=*/3u);

    REQUIRE(rounds[0] == std::vector<std::size_t>{ 0u, 1u });   // start 0
    REQUIRE(rounds[1] == std::vector<std::size_t>{ 0u, 2u });   // start 2, WRAPPED to 0
    REQUIRE(rounds[2] == std::vector<std::size_t>{ 1u, 2u });   // start 1

    // Every round writes exactly K while K < N — no short rounds anywhere in a
    // long run, which is what makes the per-tick BYTE cost of the state channel a
    // constant `K * stateBytes` rather than a variable the packet budget would
    // have to be pessimistic about. That constancy is what the round-vs-packet
    // LLT budgets against.
    const auto longRun = simulateRounds(/*writerCount=*/6u, /*k=*/2, /*rounds=*/50u);
    for (const auto& r : longRun)
        REQUIRE(r.size() == 2u);
}

TEST_CASE("CorrectionRotation: an empty writer set is a no-op, not a divide by zero",
          "[Network][CorrectionRotation]")
{
    // sendCorrectionAll runs unconditionally every tick, including on a pure
    // client (where m_authorityWriters is empty for every type) and on the server
    // before the first character registers. The predicate must answer false
    // rather than take a modulo by zero.
    REQUIRE_FALSE(correctionRotation::isInRound(/*index=*/0u, /*roundBase=*/0u,
                                                /*writerCount=*/0u, /*k=*/2));
    REQUIRE_FALSE(correctionRotation::isInRound(0u, 7u, 0u, 16));

    // Advancing is still well-defined with no writers — the base is monotonic and
    // type-independent, so it must not depend on any one type map's size.
    REQUIRE(correctionRotation::advanceRound(0u, 2) == 2u);
    REQUIRE(correctionRotation::advanceRound(2u, 2) == 4u);

    // The clamp applies to the advance too, so an out-of-range K cannot make the
    // base and the window disagree about the step size — which would break
    // coverage silently.
    REQUIRE(correctionRotation::advanceRound(0u, 0) == 1u);
    REQUIRE(correctionRotation::advanceRound(0u, -3) == 1u);
    REQUIRE(correctionRotation::advanceRound(0u, 99) == 16u);
}

#endif // WITH_LOW_LEVEL_TESTS
