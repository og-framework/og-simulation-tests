// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

TEST_CASE("DAttack.Catch2Bridge.SmokePass", "[DAttack][Catch2Bridge]")
{
    REQUIRE(1 + 1 == 2);
}

#endif // WITH_LOW_LEVEL_TESTS
