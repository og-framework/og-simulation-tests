// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <tuple>
#include <type_traits>

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulatableList.h"

// ---------------------------------------------------------------------------
// Compile-time coverage for the SimulatableList<...> toolkit (task 1 of the
// ogsim-system-api initiative — see system_api_design.md §3.10 / §4.1).
//
// The whole surface is compile-time metaprogramming, so these static_asserts
// ARE the test: if the header regresses, this translation unit fails to
// compile. The TEST_CASE below exists only so the module has a runtime anchor
// (and so [@og] reports a passing case) — the real coverage is the asserts.
// ---------------------------------------------------------------------------

// --- IsSimulatableList concept ---------------------------------------------
static_assert(IsSimulatableList<SimulatableList<int>>);
static_assert(IsSimulatableList<SimulatableList<>>);
static_assert(IsSimulatableList<SimulatableList<int, char, double>>);
static_assert(!IsSimulatableList<int>);
static_assert(!IsSimulatableList<std::tuple<int>>);

// --- list_contains_v subset check ------------------------------------------
static_assert(list_contains_v<SimulatableList<int, char>, SimulatableList<int>>);
static_assert(list_contains_v<SimulatableList<int, char>, SimulatableList<int, char>>);
static_assert(list_contains_v<SimulatableList<int, char>, SimulatableList<char, int>>);
static_assert(!list_contains_v<SimulatableList<int>, SimulatableList<char>>);
static_assert(!list_contains_v<SimulatableList<int>, SimulatableList<int, char>>);
// Empty subset is trivially contained (vacuous fold over an empty pack).
static_assert(list_contains_v<SimulatableList<int>, SimulatableList<>>);
static_assert(list_contains_v<SimulatableList<>, SimulatableList<>>);

// --- apply_t metafunction --------------------------------------------------
static_assert(std::is_same_v<apply_t<std::tuple, SimulatableList<int, char>>, std::tuple<int, char>>);
static_assert(std::is_same_v<apply_t<std::tuple, SimulatableList<>>, std::tuple<>>);

// --- is_type_in_pack_v helper ----------------------------------------------
static_assert(is_type_in_pack_v<int, int, char>);
static_assert(!is_type_in_pack_v<double, int, char>);
static_assert(!is_type_in_pack_v<int>); // empty pack

TEST_CASE("SimulatableList.CompileTimeToolkit", "[SimulatableList]")
{
    // Runtime anchor for a header whose real coverage is the static_asserts
    // above. Re-affirm a couple of the key relations as runtime REQUIREs so
    // the case has an observable assertion.
    REQUIRE(IsSimulatableList<SimulatableList<int>>);
    REQUIRE_FALSE(IsSimulatableList<int>);
    REQUIRE((list_contains_v<SimulatableList<int, char>, SimulatableList<int>>));
    REQUIRE_FALSE((list_contains_v<SimulatableList<int>, SimulatableList<char>>));
}

#endif // WITH_LOW_LEVEL_TESTS
