// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <cstdint>
#include <string>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/ClientInputDelayLine.h"
#include "OGSimulation/Network/ReplicatedTierConsumer.h"
#include "OGSimulation/PCTimeManagement/TimeConfig.h"

// ---------------------------------------------------------------------------
// CLIENT INPUT DELAY LINE — unit suite (T9 parts 3+4 of
// og-netcode-v2-arch-latency; D5.2 client half).
//
// This covers the CONTAINER and the offset rule in isolation. The end-to-end
// wiring — the real SimulationNetSync::collectInputAll provider branch, with the
// real SimulatableBrawler — is covered in og-brawler-tests
// (SimulationNetSyncTest.cpp, [SimulationNetSync][ClientInputDelay]), because
// that is the suite that links a concrete simulatable. The split is the same one
// ServerInputDelayQueueTest / ServerInputDelayIntegrationTest already use.
//
// WHAT THIS SUITE EXISTS TO PIN, beyond "the ring stores things":
//   * absence is answered with the NEUTRAL input, not with a stale neighbour
//     slot — the [0, effectiveDelay) window (part 4) IS the absence path;
//   * the neutral is INJECTED, because the game's zero input is not InputT{};
//   * `effectiveDelay <= 0` returns the LIVE capture and never touches the ring,
//     so zero delay is byte-identical to the pre-T9 behaviour;
//   * the client's delay equals the server's for the same tier — the agreement
//     that the whole Option A design rests on.
// ---------------------------------------------------------------------------

namespace
{
    // A deliberately NON-value-initialisable-to-neutral input, mirroring the real
    // hazard: simulatableBrawler::getZeroPlayerInput() builds (0,0,1) forward
    // vectors, so `PlayerInput{}` is NOT the game's zero input. A test that used
    // `int` here would be blind to a delay line that quietly returned InputT{}.
    struct TaggedInput
    {
        int         tick    = -999;     // deliberately not 0
        std::string marker  = "UNSET";  // deliberately not empty

        bool operator==(const TaggedInput& other) const
        {
            return tick == other.tick && marker == other.marker;
        }
    };

    TaggedInput captureAt(int tick)
    {
        return TaggedInput{ tick, "capture" };
    }

    TaggedInput neutralInput()
    {
        return TaggedInput{ 0, "NEUTRAL" };
    }

    using Line = ClientInputDelayLine<TaggedInput>;
}

TEST_CASE("ClientInputDelayLine: at() returns the capture taken at that tick",
          "[Network][ClientInputDelay]")
{
    Line line(neutralInput());

    for (int tick = 0; tick < 10; ++tick)
    {
        line.push(tick, captureAt(tick));
    }

    for (int tick = 0; tick < 10; ++tick)
    {
        REQUIRE(line.has(tick));
        REQUIRE(line.at(tick) == captureAt(tick));
    }
}

// PART 4, directly. The pre-session window is not an error state — it is reached
// on every session start, and after every hard resync.
TEST_CASE("ClientInputDelayLine: never-captured ticks read as the neutral input",
          "[Network][ClientInputDelay]")
{
    Line line(neutralInput());
    line.push(5, captureAt(5));

    SECTION("negative ticks — the [0, effectiveDelay) window at session start")
    {
        REQUIRE_FALSE(line.has(-1));
        REQUIRE_FALSE(line.has(-4));
        REQUIRE(line.at(-1) == neutralInput());
        REQUIRE(line.at(-4) == neutralInput());
    }

    SECTION("non-negative ticks that were simply never pushed")
    {
        REQUIRE_FALSE(line.has(4));
        REQUIRE(line.at(4) == neutralInput());
    }

    // The captured tick still answers correctly — absence handling must not
    // swallow presence.
    REQUIRE(line.at(5) == captureAt(5));
}

// The ring is indexed modulo capacity, so an evicted tick lands on a slot that
// now holds a DIFFERENT tick's capture. Reporting that neighbour would be a
// silent wrong-input bug; the stored-tick validation is what prevents it.
TEST_CASE("ClientInputDelayLine: an evicted tick reads neutral, not its ring neighbour",
          "[Network][ClientInputDelay]")
{
    constexpr std::size_t kCapacity = 8u;
    Line line(neutralInput(), kCapacity);

    // Tick 0 and tick 8 collide on slot 0.
    line.push(0, captureAt(0));
    REQUIRE(line.at(0) == captureAt(0));

    line.push(8, captureAt(8));

    REQUIRE(line.at(8) == captureAt(8));
    REQUIRE_FALSE(line.has(0));
    REQUIRE(line.at(0) == neutralInput());      // NOT captureAt(8)
    REQUIRE_FALSE(line.at(0) == captureAt(8));
}

TEST_CASE("ClientInputDelayLine: capacity comfortably exceeds the worst tier delay",
          "[Network][ClientInputDelay]")
{
    const TimeConfig cfg;
    Line line(neutralInput());

    // Whatever the configured tiers say, a capture must still be resident
    // `worstDelay` ticks later — otherwise the offset-read would fall into the
    // neutral window during normal play rather than only at session start.
    int worstDelay = 0;
    for (int tier = 0; tier <= kMaxConnectionTierIndex; ++tier)
    {
        worstDelay = std::max(worstDelay, tierInputDelayTicks(tier, cfg));
    }
    REQUIRE(worstDelay > 0);
    REQUIRE(static_cast<std::size_t>(worstDelay) < line.capacity());

    for (int tick = 0; tick <= worstDelay; ++tick)
    {
        line.push(tick, captureAt(tick));
    }
    REQUIRE(line.at(0) == captureAt(0));
    REQUIRE(line.at(worstDelay) == captureAt(worstDelay));
}

// The delay line is keyed to the pre-resync prediction clock. Surviving captures
// would be read at ticks that no longer mean what they meant.
TEST_CASE("ClientInputDelayLine: clear() re-enters the neutral window and keeps the neutral",
          "[Network][ClientInputDelay]")
{
    Line line(neutralInput());
    for (int tick = 0; tick < 5; ++tick)
    {
        line.push(tick, captureAt(tick));
    }
    REQUIRE(line.residentCount() == 5u);

    line.clear();

    REQUIRE(line.residentCount() == 0u);
    for (int tick = 0; tick < 5; ++tick)
    {
        REQUIRE_FALSE(line.has(tick));
        REQUIRE(line.at(tick) == neutralInput());
    }
    // The neutral survives the wipe — it is configuration, not data.
    REQUIRE(line.getNeutralInput() == neutralInput());
}

TEST_CASE("ClientInputDelayLine: the neutral is injected, not value-initialised",
          "[Network][ClientInputDelay]")
{
    // The defaulted line answers InputT{} — which for the real PlayerInput would
    // mean a (0,0,0) forward vector reaching normalisation.
    Line defaulted;
    REQUIRE(defaulted.at(3) == TaggedInput{});
    REQUIRE_FALSE(defaulted.at(3) == neutralInput());

    // Injection after construction must take effect, so the composition root does
    // not have to order itself before every registration.
    defaulted.setNeutralInput(neutralInput());
    REQUIRE(defaulted.at(3) == neutralInput());
}

TEST_CASE("ClientInputDelayLine: last capture wins for a repeated tick",
          "[Network][ClientInputDelay]")
{
    // Unlike the server's first-wins enqueue (which guards against a redundancy
    // bundle re-sending an already-consumed input), the only writer here is the
    // local provider, so a repeated tick means a Stall/Skip re-entry and the
    // freshest capture is the correct one.
    Line line(neutralInput());
    line.push(7, captureAt(7));
    line.push(7, TaggedInput{ 7, "fresher" });

    REQUIRE(line.at(7) == TaggedInput{ 7, "fresher" });
}

// ---------------------------------------------------------------------------
// resolveDelayedInput — THE offset rule. Production (collectInputAll) and the
// og-brawler-tests integration cases both call this same function, so these
// assertions constrain the shipped expression rather than a paraphrase of it.
// ---------------------------------------------------------------------------

TEST_CASE("resolveDelayedInput: reads the capture from tick - effectiveDelay",
          "[Network][ClientInputDelay]")
{
    Line line(neutralInput());
    for (int tick = 0; tick <= 20; ++tick)
    {
        line.push(tick, captureAt(tick));
    }

    const TaggedInput live = captureAt(20);
    REQUIRE(resolveDelayedInput(line, 20, 1, live) == captureAt(19));
    REQUIRE(resolveDelayedInput(line, 20, 4, live) == captureAt(16));
}

// A correctness requirement, not an optimisation: a Stall tick does not push into
// the line, so `at(currentTick)` could answer neutral on exactly the ticks that
// must keep the live input. Zero delay must be indistinguishable from pre-T9.
TEST_CASE("resolveDelayedInput: zero delay returns the live capture without touching the ring",
          "[Network][ClientInputDelay]")
{
    Line line(neutralInput());       // deliberately EMPTY — nothing was pushed
    const TaggedInput live = captureAt(42);

    REQUIRE(resolveDelayedInput(line, 42, 0, live) == live);
    REQUIRE(resolveDelayedInput(line, 42, -3, live) == live);   // defensive
    REQUIRE(line.residentCount() == 0u);
}

TEST_CASE("resolveDelayedInput: the session-start window resolves to the neutral input",
          "[Network][ClientInputDelay]")
{
    Line line(neutralInput());
    const int delay = 3;

    // Ticks 0..2: `tick - delay` is negative, i.e. before the session began.
    for (int tick = 0; tick < delay; ++tick)
    {
        line.push(tick, captureAt(tick));
        REQUIRE(resolveDelayedInput(line, tick, delay, captureAt(tick)) == neutralInput());
    }

    // Tick 3 is the first tick whose delayed read lands on a real capture.
    line.push(delay, captureAt(delay));
    REQUIRE(resolveDelayedInput(line, delay, delay, captureAt(delay)) == captureAt(0));
}

// THE agreement assertion. Option A's entire justification is that one tier value
// produces one delay on both ends. The client reaches it through
// ReplicatedTierConsumer; the server reaches it through ConnectionTierTable's
// delegation to the same free lookup (T9 part 2).
TEST_CASE("resolveDelayedInput: the offset equals the tier delay the consumer reports",
          "[Network][ClientInputDelay]")
{
    TimeConfig cfg;
    // Sweep the config flags rather than trusting defaults — the T9 part-2 notes
    // record a verified negative that was invisible under default flags because
    // lanZeroDelayOverride was false there.
    const bool lanOverride = GENERATE(false, true);
    cfg.lanZeroDelayOverride = lanOverride;

    const int tier = GENERATE(0, 1, 2, 3);

    ReplicatedTierConsumer consumer(cfg);
    consumer.onReplicatedTierReceived(tier);
    const int delay = consumer.effectiveInputDelayTicks();

    REQUIRE(delay == tierInputDelayTicks(tier, cfg));

    Line line(neutralInput());
    constexpr int kNow = 30;
    for (int tick = 0; tick <= kNow; ++tick)
    {
        line.push(tick, captureAt(tick));
    }

    const TaggedInput resolved = resolveDelayedInput(line, kNow, delay, captureAt(kNow));
    REQUIRE(resolved == captureAt(kNow - delay));
}

#endif // WITH_LOW_LEVEL_TESTS
