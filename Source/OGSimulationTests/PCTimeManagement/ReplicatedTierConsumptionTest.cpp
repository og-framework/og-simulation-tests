// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ConnectionTierTable.h"
#include "OGSimulation/Network/ReplicatedTierConsumer.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"
// Sibling-relative include of a header that lives in the Network/ test dir —
// the test tree is not on an include root under either UBT or the standalone
// CMake build, so this must be reached by relative path (see the same note in
// ConnectionTierTableTest.cpp / ServerTierDerivationTest.cpp).
#include "../Network/StandaloneTestHandle.h"

// ---------------------------------------------------------------------------
// CLIENT-SIDE REPLICATED-TIER CONSUMPTION (task 9 of og-netcode-v2-arch-latency
// — D5.1 client half, Option A).
//
// WHAT THIS SUITE COVERS. Under Option A the client does not derive a tier at
// all: it receives one integer from the server and turns it into behaviour.
// Everything engine-free about that is here:
//
//   1. The C2 REPLACES formula on the client side — effective delay is
//      `rttTierInputDelays[tier]`, NOT `forcedInputLatencyTicks + tier delay`.
//   2. The pre-arrival fallback, and specifically that it is keyed on ARRIVAL
//      rather than on the tier VALUE (tier 0 and "nothing yet" must not be
//      conflated — the replicated property defaults to 0).
//   3. SERVER/CLIENT AGREEMENT — the point of extracting the shared lookups.
//      A real ConnectionTierTable is driven with RTT samples, its tier index is
//      handed to the consumer as though it had replicated, and both ends are
//      asserted to produce the identical delay/ceiling/mute answers.
//   4. Wire-value robustness: an out-of-range tier byte must clamp, not index
//      the TimeConfig arrays out of bounds.
//
// WHAT IT DELIBERATELY DOES NOT COVER. The UE transport itself — the
// COND_OwnerOnly UPROPERTY and OnRep_ConnectionTier on USimmableUpdateComponent
// — is engine surface this suite links none of. That is PIE-only. This suite
// pins the contract that sits UNDER the transport.
//
// ALL ASSERTIONS READ `cfg` RATHER THAN LITERALS, so a future TimeConfig default
// change moves the expectation with it and the R-P1 configurability lint stays
// clean.
// ---------------------------------------------------------------------------

namespace
{
    // Drives `table` with enough samples at `rttMs` to clear BOTH R-A2 gates
    // (hysteresis band + minimum dwell) as many times as needed to settle, then
    // returns the tier it landed in. The table steps at most one tier per sample
    // and refuses any transition inside the dwell window, so reaching tier 3 from
    // tier 0 genuinely requires several dwell periods.
    template <typename TableT, typename AddrT>
    int32_t settleTier(TableT& table, const AddrT& addr, const TimeConfig& cfg, double rttMs)
    {
        int32_t tick = 0;
        // (tier count) dwell periods is enough to walk the full ladder end to end.
        const int32_t samples = cfg.tierMinDwellTicks * static_cast<int32_t>(kConnectionTierCount + 1);
        for (int32_t i = 0; i < samples; ++i)
            table.onRttSample(addr, tick++, rttMs);
        return table.lookupTierIndex(addr);
    }
}

TEST_CASE("ReplicatedTierConsumer: pre-arrival falls back to forcedInputLatencyTicks",
          "[PCTM][ReplicatedTier]")
{
    const TimeConfig cfg;
    ReplicatedTierConsumer consumer(cfg);

    REQUIRE_FALSE(consumer.hasReceivedTier());
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.forcedInputLatencyTicks);

    // The un-escalated configured window, not a tier ceiling: the tier can only
    // ever RAISE the soft ceiling, so "no tier" must mean "not raised".
    REQUIRE(consumer.effectiveRollbackCeiling() == cfg.rollbackWindowTicks);

    // Never mute before hearing from the server — muting is a response to a
    // degraded link, and no information is not evidence of one.
    REQUIRE_FALSE(consumer.shouldMuteEcho());
}

TEST_CASE("ReplicatedTierConsumer: tier 0 arrival is distinguishable from no arrival",
          "[PCTM][ReplicatedTier]")
{
    // THE CASE THAT JUSTIFIES THE hasReceivedTier FLAG. The replicated uint8
    // defaults to 0, which is also a perfectly legal tier. If arrival were
    // inferred from the VALUE, a client that had never heard from the server
    // would silently adopt tier-0 timing. These two states must differ, and they
    // differ only because the defaults do — hence the guard below, which makes
    // this case fail loudly rather than pass vacuously if the defaults ever
    // converge.
    const TimeConfig cfg;
    REQUIRE(cfg.forcedInputLatencyTicks != cfg.rttTierInputDelays[0]);
    REQUIRE_FALSE(cfg.lanZeroDelayOverride);

    ReplicatedTierConsumer consumer(cfg);
    const int32_t beforeArrival = consumer.effectiveInputDelayTicks();

    consumer.onReplicatedTierReceived(0);

    REQUIRE(consumer.hasReceivedTier());
    REQUIRE(consumer.currentTierIndex() == 0);
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.rttTierInputDelays[0]);
    REQUIRE(consumer.effectiveInputDelayTicks() != beforeArrival);
}

TEST_CASE("ReplicatedTierConsumer: delay tracks rttTierInputDelays and REPLACES the baseline",
          "[PCTM][ReplicatedTier]")
{
    const TimeConfig cfg;
    ReplicatedTierConsumer consumer(cfg);

    for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
    {
        INFO("tier " << tier);
        consumer.onReplicatedTierReceived(tier);

        REQUIRE(consumer.currentTierIndex() == tier);
        REQUIRE(consumer.effectiveInputDelayTicks() == cfg.rttTierInputDelays[tier]);

        // C2, locked 2026-07-19: REPLACES, never ADDS. This is the assertion that
        // catches an additive re-reading of the formula — the same probe the
        // server-side suites carry, restated on the client half so the two
        // cannot drift into disagreeing about the semantics.
        REQUIRE(consumer.effectiveInputDelayTicks()
                != cfg.forcedInputLatencyTicks + cfg.rttTierInputDelays[tier]);

        REQUIRE(consumer.effectiveRollbackCeiling() == cfg.rttTierRollbackCeilings[tier]);
    }
}

TEST_CASE("ReplicatedTierConsumer: honours lanZeroDelayOverride at tier 0 only",
          "[PCTM][ReplicatedTier]")
{
    TimeConfig cfg;
    cfg.lanZeroDelayOverride = true;
    ReplicatedTierConsumer consumer(cfg);

    consumer.onReplicatedTierReceived(0);
    REQUIRE(consumer.effectiveInputDelayTicks() == 0);

    // A bad connection inside a LAN session still gets its own tier's delay.
    consumer.onReplicatedTierReceived(1);
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.rttTierInputDelays[1]);
}

TEST_CASE("ReplicatedTierConsumer: out-of-range wire values clamp instead of indexing OOB",
          "[PCTM][ReplicatedTier]")
{
    // The tier arrives off the wire as a uint8. A corrupt, hostile or
    // version-mismatched byte must not index the TimeConfig tier arrays out of
    // bounds — that is a read past the end of a fixed-size array, not a merely
    // wrong answer.
    const TimeConfig cfg;
    ReplicatedTierConsumer consumer(cfg);

    consumer.onReplicatedTierReceived(255);
    REQUIRE(consumer.currentTierIndex() == kMaxConnectionTierIndex);
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.rttTierInputDelays[kMaxConnectionTierIndex]);

    consumer.onReplicatedTierReceived(-7);
    REQUIRE(consumer.currentTierIndex() == 0);
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.rttTierInputDelays[0]);
}

TEST_CASE("ReplicatedTierConsumer: reset returns to the pre-arrival state",
          "[PCTM][ReplicatedTier]")
{
    const TimeConfig cfg;
    ReplicatedTierConsumer consumer(cfg);

    consumer.onReplicatedTierReceived(2);
    REQUIRE(consumer.hasReceivedTier());

    consumer.reset();

    REQUIRE_FALSE(consumer.hasReceivedTier());
    REQUIRE(consumer.currentTierIndex() == 0);
    REQUIRE(consumer.effectiveInputDelayTicks() == cfg.forcedInputLatencyTicks);
}

TEST_CASE("Shared lookups: server table and client consumer agree for every tier",
          "[PCTM][ReplicatedTier]")
{
    // THE POINT OF THE SHARED-HELPER EXTRACTION. A REAL ConnectionTierTable is
    // driven with RTT samples until it settles; its tier index is then handed to
    // the consumer exactly as replication would, and both ends must answer
    // identically for all three tier-derived quantities. If either side ever
    // re-inlines its own copy of the lookup math, this case fails.
    //
    // SWEEPS lanZeroDelayOverride AND muteEchoOnDegradedTier RATHER THAN USING
    // THE DEFAULTS, and that is load-bearing rather than thoroughness theatre.
    // A first draft of this case ran on the default config only, and a verified
    // negative that re-inlined the server's delay lookup as a bare
    // `rttTierInputDelays[tier]` — dropping the lanZeroDelayOverride branch —
    // PASSED it: with the override defaulted off, the correct and the re-inlined
    // implementations agree on every tier. The flags are precisely where the two
    // can disagree, so agreement has to be asserted with them on.
    for (bool lanOverride : { false, true })
    {
        for (bool muteEcho : { false, true })
        {
            TimeConfig cfg;
            cfg.lanZeroDelayOverride   = lanOverride;
            cfg.muteEchoOnDegradedTier = muteEcho;

            // One RTT comfortably inside each tier's band, clear of the
            // hysteresis dead-band on both sides. Derived from the configured
            // boundaries rather than written as literals.
            std::vector<double> rttPerTier;
            rttPerTier.push_back(0.0);
            for (std::size_t t = 1; t < kConnectionTierCount; ++t)
            {
                rttPerTier.push_back(
                    static_cast<double>(cfg.rttTierBoundariesMs[t - 1] + 2 * cfg.tierHysteresisMs));
            }

            for (std::size_t t = 0; t < kConnectionTierCount; ++t)
            {
                INFO("lan=" << lanOverride << " mute=" << muteEcho
                     << " target tier " << t << " via rtt " << rttPerTier[t]);

                ConnectionTierTable<FStandaloneTestHandle> table(cfg);
                const FStandaloneTestHandle addr(static_cast<std::uint32_t>(100 + t), /*inAlive=*/true);

                const int32_t serverTier = settleTier(table, addr, cfg, rttPerTier[t]);
                REQUIRE(serverTier == static_cast<int32_t>(t));

                // Replicate it.
                ReplicatedTierConsumer consumer(cfg);
                consumer.onReplicatedTierReceived(serverTier);

                REQUIRE(consumer.effectiveInputDelayTicks() == table.lookupInputDelayTicks(addr));
                REQUIRE(consumer.effectiveRollbackCeiling()  == table.lookupRollbackCeiling(addr));
                REQUIRE(consumer.shouldMuteEcho()            == table.shouldMuteEcho(addr));
            }
        }
    }
}

TEST_CASE("Shared lookups: free helpers and table members are the same function",
          "[PCTM][ReplicatedTier]")
{
    // Pins the delegation directly, independent of any table state, across both
    // settings of every flag that the lookups branch on.
    for (bool lanOverride : { false, true })
    {
        for (bool muteEcho : { false, true })
        {
            TimeConfig cfg;
            cfg.lanZeroDelayOverride  = lanOverride;
            cfg.muteEchoOnDegradedTier = muteEcho;

            for (int32_t tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
            {
                INFO("lan=" << lanOverride << " mute=" << muteEcho << " tier=" << tier);

                const int32_t expectedDelay =
                    (tier == 0 && lanOverride) ? 0 : cfg.rttTierInputDelays[tier];

                REQUIRE(tierInputDelayTicks(tier, cfg) == expectedDelay);
                REQUIRE(tierRollbackCeiling(tier, cfg) == cfg.rttTierRollbackCeilings[tier]);
                REQUIRE(tierShouldMuteEcho(tier, cfg)
                        == (muteEcho && tier == kMaxConnectionTierIndex));
            }
        }
    }
}

#endif // WITH_LOW_LEVEL_TESTS
