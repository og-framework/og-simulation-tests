// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <cstring>
#include <tuple>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"

// ---------------------------------------------------------------------------
// THE FLUSH STAGE'S CAPACITY (og-netcode-v2-input-relay item 63 / RN-13,
// 2026-08-16 — ReviewNotes.md).
//
// This file used to pin the intake chain of a session-configurable relay-ring
// retention-depth knob (its old identifier is on record in RN-13,
// ReviewNotes.md, if a future reader needs it). Item 34's bare-C1
// flush-on-poll replaced the write path that knob sized, so the quantity it
// named ("entries retained by the replace-latest write path") stopped
// existing on the live relay path; item 63 then deleted the knob outright —
// the TimeConfig field, its ini key, its clamp intake, its setter, and its
// startup proof line.
//
// ⛔⛔ THE RULING THIS FILE FOLLOWS: repoint the cases that pinned the deleted
// chain at the property that SURVIVES the deletion, rather than deleting them
// — RN-13 declined a tombstone comment at `relayedInputRing::kMaxDepth`
// (RelayedInputRingCodec.h), so a design-doc-only warning would be silent.
// THE TRAP: wiring any depth knob into the flush stage's capacity degenerates
// bare-C1 back into replace-latest — no assert, no log line, looks like a
// config change, behaves like a regression, and cost T37 plus item 34 to find
// the first time. A machine-checked fence is what the declined comment could
// not be.
//
// TWO FACTS, KEPT DELIBERATELY SEPARATE:
//   1. `relayedInputRing::clampDepth` is UNRELATED to the deleted knob and
//      remains genuinely live: `stageArrival` (the flush-path write site)
//      calls it — via `writeLatest` — with the literal constant `kMaxDepth`,
//      and every direct `writeLatest` caller (this file's own §1, plus every
//      case in `WireFormat/RelayedInputRingTest.cpp`) still clamps through
//      it. §1 below is UNCHANGED IN SUBSTANCE from before this task; only its
//      framing moved, because it no longer doubles as "the session's intake
//      guard" for a knob that no longer exists.
//   2. `stageArrival` has NO depth parameter at all, and
//      `RelayedInputRingTest.cpp`'s
//      `RelayRing.StagedBurstSurvivesAtTheSessionDefaultDepth` already proves
//      its capacity behaves as `kMaxDepth` at RUNTIME. What THAT case cannot
//      show — and what THIS file now owns — is that no call shape exists for
//      a future intake chain to feed a configurable value into in the first
//      place. §2 below is a COMPILE-TIME fence for exactly that: a case that
//      fails to compile, not merely to pass, if someone gives `stageArrival` a
//      depth parameter to receive one.
// ---------------------------------------------------------------------------

namespace
{
    constexpr std::int32_t kMaxDepthI = static_cast<std::int32_t>(relayedInputRing::kMaxDepth);

    // Minimal UE-free byte buffer satisfying the codec's BUFFER CONCEPT — the
    // same four methods `WireFormat/RelayedInputRingTest.cpp`'s RingTestBuffer
    // exposes, duplicated locally rather than shared because that file is
    // WireFormat-suite scoped and this fence belongs to the Network suite's
    // knob-retirement story.
    struct FenceTestBuffer
    {
        std::vector<std::uint8_t> bytes;

        std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

        void bundleAddZeroedBytes(std::int32_t count)
        {
            bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u);
        }

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        {
            std::memcpy(bytes.data() + off, &value, sizeof(T));
        }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        {
            T value;
            std::memcpy(&value, bytes.data() + off, sizeof(T));
            return value;
        }
    };

    // Trivial Serializable input, mirroring WireFormat/RelayedInputRingTest.cpp's
    // RingTestInput — only used to make `stageArrival<FenceTestInput>` a real,
    // instantiable call for the compile-time fence and its runtime companion.
    struct FenceTestInput
    {
        std::int32_t value = 0;
    };
} // namespace

template <>
struct SerializableFields<FenceTestInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(FenceTestInput, value));
    }
};

namespace
{
    // THE COMPILE-TIME FENCE ITSELF. Must be a template — a `requires-expression`
    // over a CONCRETE type hard-errors (C2039) instead of evaluating false on
    // this tree's MSVC toolchain (see the LLT authoring notes on
    // "MSVC non-dependent requires"), so the detection is hoisted into a
    // concept, at namespace scope (a concept cannot live inside a TEST_CASE
    // body).
    template <typename InputType, typename Buffer, typename... ExtraArgs>
    concept StageArrivalAccepts = requires(Buffer& buf, std::uint32_t tick, std::uint8_t dA,
                                            const InputType& input, ExtraArgs... extra)
    {
        relayedInputRing::stageArrival<InputType>(buf, tick, dA, input, extra...);
    };

    // POSITIVE CONTROL. Without this, a typo'd concept (e.g. the wrong function
    // name) would satisfy the negative assertion below vacuously forever — a
    // fence that cannot fail is not a fence.
    static_assert(StageArrivalAccepts<FenceTestInput, FenceTestBuffer>,
        "positive control: stageArrival's real 4-argument call must stay valid");

    // THE ASSERTION. No overload of stageArrival accepts a fifth, depth-shaped
    // argument, so there is no call shape a future intake chain could resurrect
    // to feed a configurable value into the stage's capacity. If this line ever
    // fails to compile, someone gave stageArrival a depth parameter and
    // reopened T43 finding 1 — see the file banner above.
    static_assert(!StageArrivalAccepts<FenceTestInput, FenceTestBuffer, std::int32_t>,
        "stageArrival must never accept a depth argument - the stage capacity is "
        "relayedInputRing::kMaxDepth, taken as a constant, and nothing configurable "
        "may reach it (item 63 / RN-13)");
} // namespace

// ---------------------------------------------------------------------------
// 1. THE SHARED GUARD — unrelated to the deleted knob; see the file banner.
// ---------------------------------------------------------------------------

TEST_CASE("RelayRedundancyDepth: clampDepth remains the shared depth guard for direct writeLatest callers",
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

    // IDEMPOTENT, which is what makes it safe for `stageArrival` to clamp
    // (via `writeLatest`) on every flush-path write while direct test callers
    // also clamp arbitrary depths through the same guard. Two clamps that
    // could drift is the hazard; one that can be applied repeatedly is the fix.
    bool idempotent = true;
    for (const std::int32_t d : probes)
    {
        const std::uint8_t once = relayedInputRing::clampDepth(d);
        idempotent &= (relayedInputRing::clampDepth(static_cast<std::int32_t>(once)) == once);
    }
    REQUIRE(idempotent);
}

// ---------------------------------------------------------------------------
// 2. THE STAGE CAPACITY — the property that survives the knob's deletion.
// ---------------------------------------------------------------------------

TEST_CASE("RelayRedundancyDepth: the flush stage capacity is kMaxDepth - a constant nothing configurable reaches",
          "[Network][RelayRedundancyDepth]")
{
    // The compile-time half of this fence is the file-scope static_assert pair
    // above. This case is the runtime half: a burst well above kMaxDepth still
    // resides at exactly kMaxDepth afterwards, because there is no smaller
    // "requested" depth for anything to have supplied in the first place.
    FenceTestBuffer stage;
    bool droppedBeforeCapacity = false;

    for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(kMaxDepthI); ++i)
    {
        const relayedInputRing::StageArrivalOutcome outcome =
            relayedInputRing::stageArrival<FenceTestInput>(
                stage, 900u + i, 1u, FenceTestInput{ static_cast<std::int32_t>(i) });
        droppedBeforeCapacity |= outcome.droppedOldest;
    }
    REQUIRE_FALSE(droppedBeforeCapacity);
    REQUIRE(relayedInputRing::entryCount(stage) == relayedInputRing::kMaxDepth);

    // The (kMaxDepth + 1)-th arrival is the first one anything could ever have
    // evicted for. There is no depth knob left to have moved this boundary —
    // it is exactly kMaxDepth, every time.
    const relayedInputRing::StageArrivalOutcome overflow =
        relayedInputRing::stageArrival<FenceTestInput>(
            stage, 900u + static_cast<std::uint32_t>(kMaxDepthI), 1u, FenceTestInput{ -1 });
    REQUIRE(overflow.accepted);
    REQUIRE(overflow.droppedOldest);
    REQUIRE(relayedInputRing::entryCount(stage) == relayedInputRing::kMaxDepth);

    // kMaxDepth itself is a compile-time constant, not a TimeConfig read —
    // restated here as a REQUIRE (not just a static_assert) so a reviewer
    // scanning only pass/fail output, without reading source, still sees the
    // number this fence is anchored to.
    REQUIRE(relayedInputRing::kMaxDepth == 8u);
}

#endif // WITH_LOW_LEVEL_TESTS
