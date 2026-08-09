// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationManager.h"

// ---------------------------------------------------------------------------
// THE RELAY RING DEPTH INTAKE (og-netcode-v2-input-relay T35).
//
// `TimeConfig::relayRedundancyDepthTicks` sizes the OUTBOUND relay ring: how many
// of a character's most recent (captureTick, dA, input) entries the server keeps
// replicated. It shipped as a compiled constant that nothing could write — the ini
// key existed but was inert — so the ring sat at depth 1 (replace-latest) for the
// whole initiative. T35 gives it the missing intake:
//
//     [OGNetcode] RelayRedundancyDepthTicks  (server ini, composition root)
//         -> relayedInputRing::clampDepth    (the shared guard)
//         -> SimulationManager::setRelayRedundancyDepthTicks
//         -> TimeConfig::relayRedundancyDepthTicks
//         -> read per relay write, clamped once more by the codec
//
// This suite pins the engine-free half of that: the guard, and the setter. What it
// deliberately does NOT cover is the GConfig read and the startup log line at the
// composition root — UE surface with no engine-coupled LLT target to drive it (see
// docs/low-level-tests.md), verified in PIE instead.
//
// ONE GUARD, NOT THREE RULES. The intake, the setter and the ring's write site all
// call the SAME `relayedInputRing::clampDepth`, which is why it was promoted out of
// `detail`. Idempotence is what makes calling it three times safe, and a drifting
// second clamp is the specific hazard the idempotence cases below exist to prevent
// — the same shape, and the same reasoning, as
// `RelayDelayFloorTest.cpp`'s "clampRelayDelayFloorTicks is the shared intake
// guard" case for the sibling floor knob.
//
// WHY 0 CLAMPS *UP*. Depth 0 is not "off". A ring that retains nothing is a relay
// that is silently disabled — every scheduled read misses and every peer falls back
// to its last-known input, with no error anywhere. Clamping up to 1 degrades to the
// documented replace-latest behaviour instead.
// ---------------------------------------------------------------------------

namespace
{
    // SimulationManager is duck-typed on its peers and its member functions
    // instantiate only when used, so constructing one needs only what the CTOR
    // touches. On the authority path (shouldRunPrediction = false) that is the
    // server clock and nothing else — except that the ctor's resync-callback
    // lambda BODY is compiled regardless of the branch it sits behind, so NetSync
    // and Reconciliation must each expose wipeAllForResync.
    struct MockIntegrationExec {};
    struct MockNetSync       { void wipeAllForResync(unsigned int) {} };
    struct MockReconciliation{ void wipeAllForResync(unsigned int) {} };
    struct MockSystemsExec {};
    struct MockStorage {};
    struct MockStaticData {};

    using TestManager = SimulationManager<
        MockIntegrationExec, MockNetSync, MockReconciliation, MockSystemsExec,
        MockStorage, MockStaticData>;

    // Owns the peers the manager holds by reference, so a test can construct an
    // authority manager in one line without dangling anything.
    struct ManagerRig
    {
        MockIntegrationExec integration{};
        MockNetSync         netSync{};
        MockReconciliation  reconciliation{};
        MockSystemsExec     systemsExec{};
        MockStorage         storage{};
        MockStaticData      staticData{};

        TestManager manager{
            /*shouldRunPrediction=*/false,
            /*tickFrequency (fixed dt, seconds)=*/1.0 / 60.0,
            TestManager::Params{ integration, netSync, reconciliation, systemsExec,
                                 storage, staticData, nullptr } };
    };

    constexpr std::int32_t kMaxDepthI = static_cast<std::int32_t>(relayedInputRing::kMaxDepth);
}

// ---------------------------------------------------------------------------
// 1. THE SHARED GUARD.
// ---------------------------------------------------------------------------

TEST_CASE("RelayRedundancyDepth: clampDepth is the shared intake guard",
          "[Network][RelayRedundancyDepth]")
{
    // In range: the identity, including both boundaries.
    for (std::int32_t d = 1; d <= kMaxDepthI; ++d)
    {
        REQUIRE(static_cast<std::int32_t>(relayedInputRing::clampDepth(d)) == d);
    }

    // Above: capped at the wire budget, not wrapped, not rejected.
    REQUIRE(relayedInputRing::clampDepth(kMaxDepthI + 1) == relayedInputRing::kMaxDepth);
    REQUIRE(relayedInputRing::clampDepth(99)             == relayedInputRing::kMaxDepth);
    REQUIRE(relayedInputRing::clampDepth(1000000)        == relayedInputRing::kMaxDepth);

    // Below: UP to 1, never down to 0. This is the direction that matters — a
    // depth-0 ring retains nothing, which disables the relay without saying so.
    REQUIRE(relayedInputRing::clampDepth(0)     == 1);
    REQUIRE(relayedInputRing::clampDepth(-1)    == 1);
    REQUIRE(relayedInputRing::clampDepth(-1000) == 1);

    // No input of any sign or magnitude can produce a depth outside the usable
    // range — stated as a sweep rather than as spot values, because the failure
    // this guards against is a ring that is unwritable or over the wire budget,
    // and both are properties of the whole domain.
    bool allInRange = true;
    const std::int32_t probes[] = { -1000000, -9, -1, 0, 1, 2, 3, 7, 8, 9, 64, 255, 1000000 };
    for (const std::int32_t d : probes)
    {
        const std::uint8_t clamped = relayedInputRing::clampDepth(d);
        allInRange &= (clamped >= 1 && clamped <= relayedInputRing::kMaxDepth);
    }
    REQUIRE(allInRange);

    // IDEMPOTENT, which is what makes it safe to call at the ini intake, again in
    // SimulationManager::setRelayRedundancyDepthTicks, and once more on every ring
    // write. Two clamps that could drift is the hazard; one that can be applied
    // repeatedly is the fix.
    bool idempotent = true;
    for (const std::int32_t d : probes)
    {
        const std::uint8_t once = relayedInputRing::clampDepth(d);
        idempotent &= (relayedInputRing::clampDepth(static_cast<std::int32_t>(once)) == once);
    }
    REQUIRE(idempotent);
}

// ---------------------------------------------------------------------------
// 2. THE SETTER — the one writable location for the session depth.
// ---------------------------------------------------------------------------

TEST_CASE("RelayRedundancyDepth: the compiled default is 1 and the setter stores",
          "[Network][RelayRedundancyDepth]")
{
    ManagerRig rig;

    // The COMPILED default is unchanged by T35 — only the ini can move it. This
    // is the same claim TimeConfigDefaultsTest pins on a bare TimeConfig, asserted
    // here on the config a real manager owns, which is what the relay tap reads.
    REQUIRE(rig.manager.getTimeConfig().relayRedundancyDepthTicks == 1);

    // The shipped ini value, and the whole point of the task: it takes.
    rig.manager.setRelayRedundancyDepthTicks(2);
    REQUIRE(rig.manager.getTimeConfig().relayRedundancyDepthTicks == 2);

    // Every in-range value round-trips, both boundaries included.
    for (std::int32_t d = 1; d <= kMaxDepthI; ++d)
    {
        rig.manager.setRelayRedundancyDepthTicks(d);
        REQUIRE(rig.manager.getTimeConfig().relayRedundancyDepthTicks == d);
    }
}

TEST_CASE("RelayRedundancyDepth: the setter clamps, in both directions",
          "[Network][RelayRedundancyDepth]")
{
    ManagerRig rig;

    // Up: a configured 0 or negative can never disable the relay.
    rig.manager.setRelayRedundancyDepthTicks(0);
    REQUIRE(rig.manager.getTimeConfig().relayRedundancyDepthTicks == 1);

    rig.manager.setRelayRedundancyDepthTicks(-1);
    REQUIRE(rig.manager.getTimeConfig().relayRedundancyDepthTicks == 1);

    rig.manager.setRelayRedundancyDepthTicks(-1000);
    REQUIRE(rig.manager.getTimeConfig().relayRedundancyDepthTicks == 1);

    // Down: a configured 99 can never blow the per-character wire budget.
    rig.manager.setRelayRedundancyDepthTicks(99);
    REQUIRE(rig.manager.getTimeConfig().relayRedundancyDepthTicks == kMaxDepthI);

    rig.manager.setRelayRedundancyDepthTicks(kMaxDepthI + 1);
    REQUIRE(rig.manager.getTimeConfig().relayRedundancyDepthTicks == kMaxDepthI);

    // A repeated set is not cumulative, and a valid value after a clamped one is
    // still honoured — the setter stores the clamp of its ARGUMENT, never of the
    // previously stored value.
    rig.manager.setRelayRedundancyDepthTicks(2);
    REQUIRE(rig.manager.getTimeConfig().relayRedundancyDepthTicks == 2);
}

TEST_CASE("RelayRedundancyDepth: the setter and the guard cannot disagree",
          "[Network][RelayRedundancyDepth]")
{
    // THE ANTI-DRIFT CASE. The setter must not carry a private rule: whatever it
    // stores has to be exactly what clampDepth answers, for every input. If some
    // future edit gives the setter its own bounds, the composition root's log line
    // (which clamps with clampDepth before reporting the effective depth) would
    // start naming a depth the ring does not use — a lying proof line, which is
    // the one failure mode this whole path exists to avoid.
    ManagerRig rig;

    bool agrees = true;
    const std::int32_t probes[] = { -1000, -1, 0, 1, 2, 3, 7, 8, 9, 99, 1000000 };
    for (const std::int32_t d : probes)
    {
        rig.manager.setRelayRedundancyDepthTicks(d);
        agrees &= (rig.manager.getTimeConfig().relayRedundancyDepthTicks
                   == static_cast<std::int32_t>(relayedInputRing::clampDepth(d)));
    }
    REQUIRE(agrees);
}

#endif // WITH_LOW_LEVEL_TESTS
