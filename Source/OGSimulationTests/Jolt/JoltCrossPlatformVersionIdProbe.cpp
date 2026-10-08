// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#define JPH_CROSS_PLATFORM_DETERMINISTIC
#include <Jolt/Jolt.h>

#include <cstdint>

uint64_t joltVersionIdWithCrossPlatformDeterminism()
{
    using JPH::uint64;
    return JPH_VERSION_ID;
}

#endif // WITH_LOW_LEVEL_TESTS
