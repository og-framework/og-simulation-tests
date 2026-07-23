// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/DesyncDiagnosticSink.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

// ---------------------------------------------------------------------------
// Coverage for the desync diagnostic sink boundary
// (task 8 of og-netcode-v2-arch-latency — see proposal §2.4 / D3.8).
//
// There is no production consumer yet by design (the hash-broadcast wire hookup
// is the Stage 4 initiative), so this file is the ONLY thing compiling and
// exercising the header. That makes two of the cases below load-bearing beyond
// their nominal assertions: DispatchesThroughInterface is what proves the
// virtual seam actually works polymorphically rather than only via the concrete
// type, and ThresholdReadsConfigNotDefault is what proves the helper consults
// the passed config rather than having the default baked in — a mistake that
// every assertion using a default-constructed TimeConfig would happily pass.
// ---------------------------------------------------------------------------

namespace
{
    DesyncDiagnosticEvent makeEvent(int32_t tick, int32_t run)
    {
        DesyncDiagnosticEvent ev;
        ev.tick                   = tick;
        ev.localChecksum          = 0xAAAA0000u + static_cast<uint32_t>(tick);
        ev.remoteChecksum         = 0xBBBB0000u + static_cast<uint32_t>(tick);
        ev.consecutiveMismatchRun = run;
        return ev;
    }
}

TEST_CASE("DefaultSinkStashesCounts", "[Reconciliation][Desync]")
{
    LogOnlyDesyncDiagnosticSink sink(nullptr);

    REQUIRE(sink.getHashMismatchCount() == 0);
    REQUIRE(sink.getConfirmedDivergenceCount() == 0);

    sink.onHashMismatch(makeEvent(10, 1));
    sink.onHashMismatch(makeEvent(11, 2));
    sink.onHashMismatch(makeEvent(12, 3));
    sink.onConfirmedDivergence(makeEvent(12, 3));

    REQUIRE(sink.getHashMismatchCount() == 3);
    REQUIRE(sink.getConfirmedDivergenceCount() == 1);

    // The two tallies are independent — onConfirmedDivergence must not also
    // bump the mismatch count, or a Stage 4 dashboard would double-count the
    // escalating tick.
    sink.onConfirmedDivergence(makeEvent(13, 4));
    REQUIRE(sink.getHashMismatchCount() == 3);
    REQUIRE(sink.getConfirmedDivergenceCount() == 2);
}

TEST_CASE("DefaultSinkLoggerInvoked", "[Reconciliation][Desync]")
{
    std::vector<std::string> messages;
    LogOnlyDesyncDiagnosticSink sink(
        [&messages](const char* msg) { messages.emplace_back(msg); });

    sink.onHashMismatch(makeEvent(42, 1));
    REQUIRE(messages.size() == 1);
    REQUIRE(messages[0].find("Desync.HashMismatch") != std::string::npos);
    REQUIRE(messages[0].find("tick=42") != std::string::npos);

    sink.onConfirmedDivergence(makeEvent(47, 5));
    REQUIRE(messages.size() == 2);
    REQUIRE(messages[1].find("Desync.ConfirmedDivergence") != std::string::npos);
    REQUIRE(messages[1].find("run=5") != std::string::npos);

    // One message per call, not per event field / not batched.
    sink.onHashMismatch(makeEvent(48, 1));
    sink.onHashMismatch(makeEvent(49, 2));
    REQUIRE(messages.size() == 4);

    // A null logger is the documented "logging disabled" contract, and the
    // counters must still work — SIMLOG's null test is what keeps this from
    // being a crash.
    LogOnlyDesyncDiagnosticSink silent(nullptr);
    silent.onHashMismatch(makeEvent(1, 1));
    silent.onConfirmedDivergence(makeEvent(1, 1));
    REQUIRE(silent.getHashMismatchCount() == 1);
    REQUIRE(silent.getConfirmedDivergenceCount() == 1);
}

TEST_CASE("EscalationThresholdGate", "[Reconciliation][Desync]")
{
    // Deliberately the DEFAULT-constructed config: hashMismatchTickThreshold
    // defaults to 5, so this exercises the AC's 4-then-5 boundary against the
    // real shipped default without re-declaring the literal (which the
    // configurability lint guards).
    const TimeConfig cfg;

    REQUIRE(shouldEscalateToLayer2(4, cfg) == false);
    REQUIRE(shouldEscalateToLayer2(5, cfg) == true);

    // Escalation is monotone in the run length — once escalated, a longer run
    // stays escalated.
    REQUIRE(shouldEscalateToLayer2(6, cfg) == true);
    REQUIRE(shouldEscalateToLayer2(600, cfg) == true);

    // Degenerate/never-mismatched runs never escalate.
    REQUIRE(shouldEscalateToLayer2(0, cfg) == false);
    REQUIRE(shouldEscalateToLayer2(1, cfg) == false);
}

TEST_CASE("ThresholdReadsConfigNotDefault", "[Reconciliation][Desync]")
{
    // Guards against the helper hardcoding the default instead of reading the
    // field. Both configs below use NON-default thresholds, so a hardcoded 5
    // fails here while passing every assertion in EscalationThresholdGate.
    TimeConfig tight;
    tight.hashMismatchTickThreshold = 2;
    REQUIRE(shouldEscalateToLayer2(1, tight) == false);
    REQUIRE(shouldEscalateToLayer2(2, tight) == true);

    TimeConfig loose;
    loose.hashMismatchTickThreshold = 12;
    REQUIRE(shouldEscalateToLayer2(11, loose) == false);
    REQUIRE(shouldEscalateToLayer2(12, loose) == true);

    // The two configs disagree at the same run length — the answer is a
    // function of the config, not of the run alone.
    REQUIRE(shouldEscalateToLayer2(5, tight) != shouldEscalateToLayer2(5, loose));
}

TEST_CASE("DispatchesThroughInterface", "[Reconciliation][Desync]")
{
    // Stage 4 will hold the sink as IDesyncDiagnosticSink& / unique_ptr, never
    // as the concrete type. Exercise it the way it will actually be used.
    std::vector<std::string> messages;
    std::unique_ptr<IDesyncDiagnosticSink> sink =
        std::make_unique<LogOnlyDesyncDiagnosticSink>(
            [&messages](const char* msg) { messages.emplace_back(msg); });

    sink->onHashMismatch(makeEvent(7, 1));
    sink->onConfirmedDivergence(makeEvent(7, 5));
    REQUIRE(messages.size() == 2);

    // Counts are reachable after downcasting back — i.e. the base-pointer calls
    // above landed on the derived overrides, not on some other object.
    auto* concrete = static_cast<LogOnlyDesyncDiagnosticSink*>(sink.get());
    REQUIRE(concrete->getHashMismatchCount() == 1);
    REQUIRE(concrete->getConfirmedDivergenceCount() == 1);

    // Destroying through the base pointer must run the derived destructor —
    // relies on IDesyncDiagnosticSink's virtual dtor. Under MSVC debug CRT a
    // missing virtual dtor here is a heap-corruption report, not a silent pass.
    sink.reset();
    REQUIRE(messages.size() == 2);
}

#endif // WITH_LOW_LEVEL_TESTS
