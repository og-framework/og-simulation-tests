// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltDeterminismFingerprint.h"

using namespace joltTestRig;

#if defined(_M_X64)
TEST_CASE("JoltDeterminismFingerprint.PinnedValueWin64", "[Jolt]")
{
    RuntimeLease lease;
    const JoltDeterminismFingerprint fingerprint = determinismFingerprint(lease.runtime);
    CHECK(fingerprint.value() == 0xe54d2bc380288c01ull);
}
#endif

#endif
