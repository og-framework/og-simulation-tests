// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/CorrectionStateBufferCodec.h"
#include "OGSimulation/InputRedundancyBundleCodec.h"
#include "OGSimulation/RelayedInputRingCodec.h"

TEST_CASE("WireVersionKnob.TheVersionsAreTheOnesTheFlipsReached", "[WireFormat][WireVersionKnob]")
{
    REQUIRE(correctionStateBuffer::kWireFormatVersion == 10u);
    REQUIRE(relayedInputRing::kWireFormatVersion == 3u);
    REQUIRE(inputRedundancyBundle::kWireFormatVersion == 3u);
}

#endif
