// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"

#include <cstdint>
#include <type_traits>

//////////////////////////////////////////////////////////////////////////////
// SimulationDerivedComposite<Ts...> -- the OFF-WIRE composite
// (brawler-movement-simulation task 23).
//
// A game's DerivedState is per-simulatable LOCAL scratch: recomputed or reset
// every tick, never serialized, never corrected, never checksummed. That rule
// (the brawler initiative records it as D1) used to live in a COMMENT on the
// slice. `SimulationDerivedComposite` is the same underlying SimulationComposite
// with `requires (!(Serializable<Ts> || ...))` attached, so the rule is now the
// compiler's problem instead of a review catch.
//
// WHAT THESE CASES HAVE TO DISCRIMINATE, and why a naive spelling would not.
// The failure this alias exists to prevent is somebody giving a derived slice a
// `SerializableFields` specialization -- the single edit that would silently
// start costing wire bytes and checksum coverage. So it is not enough to show
// that a composite of non-serializable mocks compiles: an UNCONSTRAINED alias
// would pass that too, identically. The load-bearing assertions are the
// NEGATIVE ones below -- `HasDerivedComposite<WireSlice>` must be FALSE -- and
// each is paired with a positive control on the same machinery so a typo cannot
// satisfy it vacuously.
//
// WHY `HasDerivedComposite` IS A CONCEPT TEMPLATE AND NOT AN INLINE `requires`.
// MSVC 14.38.33130 (the toolchain UBT selects here) mis-evaluates an inline
// `requires { ... }` written over a CONCRETE instantiation -- task 22 captured a
// build where it reported the expression as satisfied while simultaneously
// printing "the constraint was not satisfied". Making the detection dependent on
// the concept's own parameters is the standing workaround in this suite.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // A derived slice with actual scratch in it -- the shape of the brawler's
    // radial / projectile / inbound-hit slices. NO SerializableFields.
    struct ScratchA
    {
        std::int32_t counter = 0;
        bool         flag    = false;
    };

    // The OTHER production shape: an intentionally-empty slice kept for
    // structural symmetry (the brawler's guard and movement slices are exactly
    // this). Present so both kinds are shown to sit in one composite.
    struct ScratchB
    {
    };

    // THE NEGATIVE CONTROL -- a slice that DOES cross the wire. Nothing about it
    // is unusual; the only thing that disqualifies it is the SerializableFields
    // specialization below, which is precisely the edit the alias must reject.
    struct WireSlice
    {
        std::int32_t value = 0;
    };
} // namespace

template <>
struct SerializableFields<WireSlice>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(WireSlice, value));
    }
};

// Whether SimulationDerivedComposite<Ts...> can be NAMED at all. Hoisted into a
// concept template on purpose -- see the MSVC note in the header comment.
template <typename... Ts>
concept HasDerivedComposite = requires { typename SimulationDerivedComposite<Ts...>; };

// The same detection over the UNCONSTRAINED alias. This is the anti-vacuity
// guard for the negative assertions: it proves the rejections below come from
// the requires-clause and not from `WireSlice` being unusable in a composite for
// some unrelated reason.
template <typename... Ts>
concept HasPlainComposite = requires { typename SimulationComposite<Ts...>; };

TEST_CASE("SimulationDerivedComposite rejects any element that is Serializable",
          "[SimulationComposite][DerivedComposite]")
{
    // --- Anti-vacuity: the mocks really are on the sides of the line I claim ---
    STATIC_REQUIRE_FALSE(Serializable<ScratchA>);
    STATIC_REQUIRE_FALSE(Serializable<ScratchB>);
    STATIC_REQUIRE(Serializable<WireSlice>);

    // --- Anti-vacuity: WireSlice is a perfectly good PLAIN composite element ---
    // so a rejection below can only be the derived alias's constraint talking.
    STATIC_REQUIRE(HasPlainComposite<WireSlice>);
    STATIC_REQUIRE(HasPlainComposite<ScratchA, WireSlice>);

    // --- The positive case: a list of pure scratch is accepted ---
    STATIC_REQUIRE(HasDerivedComposite<ScratchA, ScratchB>);
    STATIC_REQUIRE(HasDerivedComposite<ScratchA>);

    // --- THE POINT OF THE TASK: a Serializable element is a COMPILE ERROR ---
    // Alone, first in the list, and last in the list -- the fold is `||` over the
    // whole pack, so a single wire slice anywhere disqualifies the composite.
    STATIC_REQUIRE_FALSE(HasDerivedComposite<WireSlice>);
    STATIC_REQUIRE_FALSE(HasDerivedComposite<WireSlice, ScratchA>);
    STATIC_REQUIRE_FALSE(HasDerivedComposite<ScratchA, ScratchB, WireSlice>);

    // The empty pack is accepted: `(... || false)` is false, so `!false` holds.
    // Not a hypothetical -- a game with no derived state at all should compile.
    STATIC_REQUIRE(HasDerivedComposite<>);
}

TEST_CASE("SimulationDerivedComposite is a SimulationComposite: get / edit / forEach",
          "[SimulationComposite][DerivedComposite]")
{
    // The alias must not be a new type with new semantics -- it is the SAME class
    // template, so the access idiom a game already uses on State works unchanged.
    STATIC_REQUIRE(std::is_same_v<SimulationDerivedComposite<ScratchA, ScratchB>,
                                  SimulationComposite<ScratchA, ScratchB>>);

    SimulationDerivedComposite<ScratchA, ScratchB> derived;

    REQUIRE(derived.get<ScratchA>().counter == 0);
    REQUIRE_FALSE(derived.get<ScratchA>().flag);

    derived.edit<ScratchA>().counter = 42;
    derived.edit<ScratchA>().flag    = true;

    REQUIRE(derived.get<ScratchA>().counter == 42);
    REQUIRE(derived.get<ScratchA>().flag);

    // A COPY carries the mutated scratch. This is the property the brawler's
    // per-render-step AllState copy (updateVizState) depends on, and it is the
    // one thing a naked class gave for free that a reviewer would want pinned
    // when it becomes a tuple.
    const SimulationDerivedComposite<ScratchA, ScratchB> copy = derived;
    REQUIRE(copy.get<ScratchA>().counter == 42);
    REQUIRE(copy.get<ScratchA>().flag);

    // forEach -- the option value the conversion buys (a generic per-tick
    // reset/dump/viz pass). Counts BOTH elements, so an empty slice is visited
    // too rather than being silently skipped.
    int visited = 0;
    derived.forEach([&visited](auto&) { ++visited; });
    REQUIRE(visited == 2);

    // The const overload, over the const copy.
    int visitedConst = 0;
    copy.forEach([&visitedConst](const auto&) { ++visitedConst; });
    REQUIRE(visitedConst == 2);
}

#endif // WITH_LOW_LEVEL_TESTS
