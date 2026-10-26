// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/RelayedInputRingCodec.h"
#include "OGSimulation/SimulationFrame.h"

#include <cstdint>
#include <set>

namespace
{
    constexpr AxisFrame kZ = AxisFrame::UeZUpLeftHanded;
    constexpr AxisFrame kY = AxisFrame::YUpRightHanded;
    constexpr float kCm = 100.f;
    constexpr float kM = 1.f;
}

TEST_CASE("WireVersionKnob.TheLegacyFrameVersionsAreTheOnesOnMainWhenTheKnobsLanded", "[WireFormat][WireVersionKnob]")
{
    REQUIRE(correctionStateBuffer::kLegacyFrameWireFormatVersion == 6u);
    REQUIRE(relayedInputRing::kLegacyFrameWireFormatVersion == 2u);
    REQUIRE(inputRedundancyBundle::kLegacyFrameWireFormatVersion == 2u);
}

TEST_CASE("WireVersionKnob.TheCorrectionVersionIsDistinctAtEachOfTheFourKnobCombinations", "[WireFormat][WireVersionKnob]")
{
    using correctionStateBuffer::kLegacyFrameWireFormatVersion;
    using correctionStateBuffer::wireFormatVersionFor;

    REQUIRE(wireFormatVersionFor(kZ, kCm) == kLegacyFrameWireFormatVersion);
    REQUIRE(wireFormatVersionFor(kZ, kM) == kLegacyFrameWireFormatVersion + 1);
    REQUIRE(wireFormatVersionFor(kY, kM) == kLegacyFrameWireFormatVersion + 2);
    REQUIRE(wireFormatVersionFor(kY, kCm) == kLegacyFrameWireFormatVersion + 3);

    const std::set<std::uint8_t> distinct{ wireFormatVersionFor(kZ, kCm), wireFormatVersionFor(kZ, kM),
        wireFormatVersionFor(kY, kM), wireFormatVersionFor(kY, kCm) };
    REQUIRE(distinct.size() == 4u);
}

TEST_CASE("WireVersionKnob.TheRingAndBundleVersionsChangeWithTheAxisOnly", "[WireFormat][WireVersionKnob]")
{
    REQUIRE(relayedInputRing::wireFormatVersionFor(kZ, kCm) == relayedInputRing::kLegacyFrameWireFormatVersion);
    REQUIRE(relayedInputRing::wireFormatVersionFor(kZ, kM) == relayedInputRing::kLegacyFrameWireFormatVersion);
    REQUIRE(relayedInputRing::wireFormatVersionFor(kY, kCm) == relayedInputRing::kLegacyFrameWireFormatVersion + 1);
    REQUIRE(relayedInputRing::wireFormatVersionFor(kY, kM) == relayedInputRing::kLegacyFrameWireFormatVersion + 1);

    REQUIRE(inputRedundancyBundle::wireFormatVersionFor(kZ, kCm) == inputRedundancyBundle::kLegacyFrameWireFormatVersion);
    REQUIRE(inputRedundancyBundle::wireFormatVersionFor(kZ, kM) == inputRedundancyBundle::kLegacyFrameWireFormatVersion);
    REQUIRE(inputRedundancyBundle::wireFormatVersionFor(kY, kCm)
        == inputRedundancyBundle::kLegacyFrameWireFormatVersion + 1);
    REQUIRE(inputRedundancyBundle::wireFormatVersionFor(kY, kM)
        == inputRedundancyBundle::kLegacyFrameWireFormatVersion + 1);
}

TEST_CASE("WireVersionKnob.TheShippedVersionsAreTheFunctionsAtTheShippedKnobs", "[WireFormat][WireVersionKnob]")
{
    REQUIRE(correctionStateBuffer::kWireFormatVersion
        == correctionStateBuffer::wireFormatVersionFor(kAxisFrame, kUnitsPerMetre));
    REQUIRE(relayedInputRing::kWireFormatVersion == relayedInputRing::wireFormatVersionFor(kAxisFrame, kUnitsPerMetre));
    REQUIRE(inputRedundancyBundle::kWireFormatVersion
        == inputRedundancyBundle::wireFormatVersionFor(kAxisFrame, kUnitsPerMetre));
}

#endif
