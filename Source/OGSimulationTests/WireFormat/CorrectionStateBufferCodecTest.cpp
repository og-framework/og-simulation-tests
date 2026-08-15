// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"

#include <cstdint>
#include <cstring>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / T4: the CORRECTION STATE payload now carries the
// per-tick APPLIED-CAPTURE-TICK REFERENCE — the join key between the two
// now-independently-cadenced channels.
//
// WHY THESE TESTS EXIST. Input is relayed at receipt keyed by CAPTURE tick;
// state is corrected keyed by the AUTHORITY tick. Without a reference riding the
// state message, a client resimulating a corrected tick cannot say which relayed
// input the authority actually fed into it — the whole initiative's correlation
// would be guesswork off a delay that is only the INTENDED schedule
// (RelayDelaySpectrumDesign.md §5.3).
//
// ⚠ [T39] THIS BLOCK USED TO SAY "corrected EVERY FRAME". It is not: T39's
// write-site rotation means a character's state ships at `tickFrequency * K / N`
// Hz (`TimeConfig::correctionRotationK`, shipped at 2 — 60 Hz at two characters,
// 20 Hz at six). NOTHING IN THIS FILE CHANGES AS A RESULT, and that is the point
// worth recording: these cases pin the codec's LAYOUT and its sentinel, which are
// cadence-independent by construction. The correction is to the prose only, so a
// future reader does not take the mirrored premise as a live statement about the
// channel — see the same correction at the head of CorrectionStateBufferCodec.h.
//
// WHAT IS PINNED HERE:
//   * the ref survives the wire round-trip verbatim, at a FIXED offset, ahead of
//     the state composite (so the composite's own offsets stay a pure function
//     of its field list);
//   * the D1 SENTINEL (kNoInputCaptureTick — "the authority substituted an
//     input, no client capture stands behind this tick") round-trips as itself
//     and is distinguishable from every real capture tick, INCLUDING tick 0,
//     which is an ordinary session-start capture;
//   * the version fence is at 2 (bumped 1 -> 2 by this task) — the SINGLE fence
//     of the increment.
//
// These run against the ENGINE-AGNOSTIC codec, which is the same code the UE
// USTRUCT (FSimulationStateSyncBuffer) delegates to — a std::vector-backed
// buffer here versus its TArray-backed one, per the codec's BUFFER CONCEPT. The
// USTRUCT's UE-only half (NetSerialize's version byte + watermark trim, and the
// OnRep refusal path that compares the byte against kWireFormatVersion) is not
// reachable from this target and follows the existing deferral for
// engine-coupled wire tests (WireFormat_Bundle.cpp / docs/low-level-tests.md
// "Future: testing UE-coupled code").
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // Two trivial Serializable parts, mirroring the other WireFormat test files
    // so every codec is exercised against the same fixed-stride shape.
    struct CorrTestPosition
    {
        std::int32_t x = 0;
    };

    struct CorrTestHealth
    {
        float value = 0.f;
    };
} // namespace

template <>
struct SerializableFields<CorrTestPosition>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(CorrTestPosition, x));
    }
};

template <>
struct SerializableFields<CorrTestHealth>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(CorrTestHealth, value));
    }
};

namespace
{
    using CorrTestState = SimulationComposite<CorrTestPosition, CorrTestHealth>;

    // UE-free byte buffer satisfying the codec's BUFFER CONCEPT — the same two
    // methods FSimulationStateSyncBuffer exposes over its TArray<uint8>. Fixed
    // capacity, like the real one (kBufferBytes), so there is no grow path here
    // either.
    struct CorrTestBuffer
    {
        std::vector<std::uint8_t> bytes = std::vector<std::uint8_t>(384, 0u);

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        {
            std::memcpy(bytes.data() + off, &value, sizeof(T));
        }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        {
            T value{};
            std::memcpy(&value, bytes.data() + off, sizeof(T));
            return value;
        }
    };

    CorrTestState makeState(std::int32_t x, float health)
    {
        CorrTestState state;
        state.edit<CorrTestPosition>().x     = x;
        state.edit<CorrTestHealth>().value   = health;
        return state;
    }
} // namespace

// ---------------------------------------------------------------------------
// The headline round-trip: state + tick + ref, all three intact.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionStateBuffer.RoundTripCarriesTheAppliedCaptureTick",
          "[WireFormat][CorrectionStateRef]")
{
    CorrTestBuffer buffer;

    // Authority tick and capture tick are deliberately DIFFERENT numbers: the
    // whole point of the field is that the state's tick does not tell you which
    // capture produced it (the server applies a client capture some ticks after
    // it was taken).
    constexpr std::uint32_t kStateTick   = 900u;
    constexpr std::uint32_t kCaptureTick = 893u;

    correctionStateBuffer::write(buffer, makeState(42, 7.5f), kStateTick, kCaptureTick);

    CorrTestState out;
    std::uint32_t readCaptureTick = 0u;
    const std::uint32_t readTick = correctionStateBuffer::readInto(buffer, out, readCaptureTick);

    REQUIRE(readTick == kStateTick);
    REQUIRE(readCaptureTick == kCaptureTick);
    REQUIRE(readCaptureTick != readTick);

    // The state payload is unaffected by the new field sitting in front of it.
    REQUIRE(out.get<CorrTestPosition>().x == 42);
    REQUIRE(out.get<CorrTestHealth>().value == Catch::Approx(7.5f));

    // The ref-only read agrees with the full read — it is the same bytes, not a
    // second source of truth.
    REQUIRE(correctionStateBuffer::readAppliedCaptureTick(buffer) == kCaptureTick);
}

// ---------------------------------------------------------------------------
// THE SENTINEL. On a RemoteMoveQueue underrun the authority applies a
// substituted input, so there is no capture tick to name. The wire must carry
// that fact explicitly rather than repeat a stale number the client would then
// resolve into a real (wrong) relayed input.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionStateBuffer.SentinelRoundTripsAsItself",
          "[WireFormat][CorrectionStateRef]")
{
    CorrTestBuffer buffer;

    correctionStateBuffer::write(buffer, makeState(-1, 0.f), 1200u, kNoInputCaptureTick);

    CorrTestState out;
    std::uint32_t readCaptureTick = 0u;
    const std::uint32_t readTick = correctionStateBuffer::readInto(buffer, out, readCaptureTick);

    REQUIRE(readTick == 1200u);
    REQUIRE(readCaptureTick == kNoInputCaptureTick);
    // Not silently clamped, truncated, or turned into a plausible tick.
    REQUIRE(readCaptureTick == 0xFFFFFFFFu);
    REQUIRE(out.get<CorrTestPosition>().x == -1);
}

// ---------------------------------------------------------------------------
// The discriminator: capture tick 0 is an ORDINARY session-start capture and
// must not read back as "no input applied". This is the wire-side twin of
// SimulationNetSyncTest's RemoteBranchRealCaptureTickZeroIsNotTheSentinel — a
// codec that used 0 as its "unset" marker would pass every other case here.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionStateBuffer.RealCaptureTickZeroIsNotTheSentinel",
          "[WireFormat][CorrectionStateRef]")
{
    CorrTestBuffer buffer;

    correctionStateBuffer::write(buffer, makeState(3, 1.f), 5u, 0u);

    CorrTestState out;
    std::uint32_t readCaptureTick = kNoInputCaptureTick;
    correctionStateBuffer::readInto(buffer, out, readCaptureTick);

    REQUIRE(readCaptureTick == 0u);
    REQUIRE(readCaptureTick != kNoInputCaptureTick);

    // ...and a later publish on the SAME buffer can flip it back to the
    // sentinel, so the field is per-publish, never a latch.
    correctionStateBuffer::write(buffer, makeState(3, 1.f), 6u, kNoInputCaptureTick);
    REQUIRE(correctionStateBuffer::readAppliedCaptureTick(buffer) == kNoInputCaptureTick);
}

// ---------------------------------------------------------------------------
// Layout + fence. Both halves are wire contracts a future edit must not move
// silently: the ref sits between the tick and the composite, and the version
// fence is at 2.
// ---------------------------------------------------------------------------
TEST_CASE("CorrectionStateBuffer.LayoutAndVersionFence",
          "[WireFormat][CorrectionStateRef]")
{
    REQUIRE(correctionStateBuffer::kTickOffset == 0u);
    REQUIRE(correctionStateBuffer::kAppliedCaptureTickOffset == 4u);
    REQUIRE(correctionStateBuffer::kPayloadOffset == 8u);
    REQUIRE(correctionStateBuffer::kHeaderBytes == correctionStateBuffer::kPayloadOffset);

    // The SINGLE wire fence of the input-relay increment. It was 1 before T4;
    // if this ever reads 1 again, mismatched builds stop being detected.
    REQUIRE(correctionStateBuffer::kWireFormatVersion == 2u);
    REQUIRE(correctionStateBuffer::kWireFormatVersion != 1u);

    // The two header scalars really do precede the composite: the first
    // composite field lands at kPayloadOffset, and writing the header does not
    // touch it.
    CorrTestBuffer buffer;
    correctionStateBuffer::write(buffer, makeState(0x0BADF00D, 2.f), 11u, 12u);

    REQUIRE(buffer.readFromBuffer<std::uint32_t>(correctionStateBuffer::kTickOffset) == 11u);
    REQUIRE(buffer.readFromBuffer<std::uint32_t>(correctionStateBuffer::kAppliedCaptureTickOffset) == 12u);
    REQUIRE(buffer.readFromBuffer<std::int32_t>(correctionStateBuffer::kPayloadOffset) == 0x0BADF00D);
}

#endif // WITH_LOW_LEVEL_TESTS
