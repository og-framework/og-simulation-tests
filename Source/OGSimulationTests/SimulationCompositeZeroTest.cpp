// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"

#include <cstdint>

//////////////////////////////////////////////////////////////////////////////
// SimulationComposite<Ts...>::zero() -- the NEUTRAL INPUT fold
// (brawler-movement-simulation task 22).
//
// The `SimulationInput` role concept now requires `{ T::zero() } -> same_as<T>`,
// and the composite folds those: `SimulationComposite(Ts::zero()...)`. A game no
// longer hand-builds one argument per element to say what its neutral input is;
// each element type owns its own neutral value, beside the type.
//
// WHAT MAKES THESE CASES DISCRIMINATE, and why it matters here more than usual.
// The obvious wrong implementation of this fold is `return SimulationComposite{};`
// -- value-initialising every element instead of asking each for its zero(). A
// mock whose `zero()` happens to equal `T{}` CANNOT tell the two apart: it passes
// against the correct fold and against the broken one alike. So `TaggedZeroInput`
// below has `zero() != TaggedZeroInput{}` in EVERY field, deliberately, and the
// cases assert both halves: the element equals `T::zero()` AND differs from `T{}`.
//
// That is not a hypothetical. In production the brawler's radial, machine and
// guard `PlayerInput` types carry a (0,0,1) forward aim as their neutral value
// while a value-initialised one carries (0,0,0) -- a difference that is both a
// normalize() guard and the tag the input-resolution anti-vacuity tests use.
// `NeutralZeroInput` is the OTHER production shape (projectile / movement, whose
// zero() is a value-initialised element) and is present so the fold is shown to
// handle both kinds in one composite.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // THE DISCRIMINATING MOCK. Every field of zero() differs from the field's
    // default member initialiser, so an element that came from value-initialisation
    // rather than from zero() is detectable on any of them.
    struct TaggedZeroInput
    {
        float        aim    = 0.f;   // TaggedZeroInput{} carries 0.f
        std::int32_t marker = 0;     // TaggedZeroInput{} carries 0

        static TaggedZeroInput zero() { return TaggedZeroInput{ 1.f, 7 }; }
    };

    // The other production shape: a neutral value that IS the value-initialised one.
    struct NeutralZeroInput
    {
        std::int32_t value = 0;

        static NeutralZeroInput zero() { return NeutralZeroInput{}; }
    };

    // NEGATIVE CONTROL 1 -- Serializable, but no zero() at all.
    struct NoZeroInput
    {
        std::int32_t value = 0;
    };

    // NEGATIVE CONTROL 2 -- has a zero(), but it does not return T. The concept
    // spells `-> std::same_as<T>` rather than merely requiring the call to be
    // valid, and this is what holds it to that.
    struct WrongReturnZeroInput
    {
        std::int32_t value = 0;

        static std::int32_t zero() { return 0; }
    };
} // namespace

template <>
struct SerializableFields<TaggedZeroInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(TaggedZeroInput, aim),
                               SIM_MEMBER(TaggedZeroInput, marker));
    }
};

template <>
struct SerializableFields<NeutralZeroInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(NeutralZeroInput, value));
    }
};

template <>
struct SerializableFields<NoZeroInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(NoZeroInput, value));
    }
};

template <>
struct SerializableFields<WrongReturnZeroInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(WrongReturnZeroInput, value));
    }
};

// Whether SimulationComposite<Ts...>::zero() is callable at all. Hoisted into a
// concept template ON PURPOSE: an inline `requires { SimulationComposite<A, B>::zero(); }`
// over a CONCRETE instantiation is mis-evaluated by MSVC 14.38 (the toolchain UBT
// selects here) -- it reported the expression as satisfied while simultaneously
// printing "the constraint was not satisfied". Making the detection dependent on the
// concept's own parameters fixes it. Same workaround as the absence-assertions in
// the relay store tests.
template <typename... Ts>
concept HasCompositeZero = requires { SimulationComposite<Ts...>::zero(); };

TEST_CASE("SimulationComposite::zero() folds each element's own zero(), not T{}",
          "[SimulationComposite][ZeroFold]")
{
    // THE ANTI-VACUITY GUARD FOR THIS CASE ITSELF. If the tagged mock's zero()
    // ever became equal to a value-initialised one, every assertion below would
    // still pass while no longer able to see the defect it exists to catch.
    REQUIRE(TaggedZeroInput::zero().aim != TaggedZeroInput{}.aim);
    REQUIRE(TaggedZeroInput::zero().marker != TaggedZeroInput{}.marker);

    using Composite = SimulationComposite<TaggedZeroInput, NeutralZeroInput>;
    const Composite neutral = Composite::zero();

    // 1. The tagged element IS its type's zero()...
    REQUIRE(neutral.get<TaggedZeroInput>().aim == TaggedZeroInput::zero().aim);
    REQUIRE(neutral.get<TaggedZeroInput>().marker == TaggedZeroInput::zero().marker);

    // 2. ...and is NOT a value-initialised element. This pair is the whole case:
    //    `return SimulationComposite{};` satisfies (1) for NeutralZeroInput but
    //    fails here.
    REQUIRE(neutral.get<TaggedZeroInput>().aim != TaggedZeroInput{}.aim);
    REQUIRE(neutral.get<TaggedZeroInput>().marker != TaggedZeroInput{}.marker);

    // 3. The other shape folds too -- an element whose zero() equals T{} is not a
    //    special case in the fold, it just happens to coincide.
    REQUIRE(neutral.get<NeutralZeroInput>().value == NeutralZeroInput::zero().value);
    REQUIRE(neutral.get<NeutralZeroInput>().value == NeutralZeroInput{}.value);

    // 4. The fold reads element-wise, not positionally: swapping the pack order
    //    must move the values with the types.
    using Swapped = SimulationComposite<NeutralZeroInput, TaggedZeroInput>;
    const Swapped swapped = Swapped::zero();
    REQUIRE(swapped.get<TaggedZeroInput>().aim == TaggedZeroInput::zero().aim);
    REQUIRE(swapped.get<TaggedZeroInput>().marker == TaggedZeroInput::zero().marker);
}

TEST_CASE("SimulationInput requires a zero() returning the type itself",
          "[SimulationComposite][ZeroFold]")
{
    // POSITIVE: both mocks are accepted, so the negatives below are rejected for
    // the zero() requirement and not because the mocks are unserializable.
    STATIC_REQUIRE(Serializable<TaggedZeroInput>);
    STATIC_REQUIRE(Serializable<NoZeroInput>);
    STATIC_REQUIRE(Serializable<WrongReturnZeroInput>);
    STATIC_REQUIRE(SimulationInput<TaggedZeroInput>);
    STATIC_REQUIRE(SimulationInput<NeutralZeroInput>);

    // NEGATIVE 1 -- no zero(). Under the pre-task concept (`Serializable<T>` alone)
    // this was TRUE, so this line is what the widening bought.
    STATIC_REQUIRE_FALSE(SimulationInput<NoZeroInput>);

    // NEGATIVE 2 -- a zero() that returns something else. Distinguishes
    // `-> std::same_as<T>` from a bare `requires { T::zero(); }`.
    STATIC_REQUIRE_FALSE(SimulationInput<WrongReturnZeroInput>);

    // The composite's zero() is CONSTRAINED, not merely broken-on-use: a composite
    // holding an element without a zero() has no zero() to call at all. Detected
    // through the HasCompositeZero concept above rather than an inline
    // requires-expression over the concrete instantiation -- MSVC 14.38 evaluates
    // that shape wrongly (see the note at the concept).
    STATIC_REQUIRE_FALSE(HasCompositeZero<NoZeroInput>);
    STATIC_REQUIRE_FALSE(HasCompositeZero<TaggedZeroInput, NoZeroInput>);
    STATIC_REQUIRE(HasCompositeZero<TaggedZeroInput, NeutralZeroInput>);

    // The OTHER two role concepts were NOT widened -- they still mean
    // `Serializable<T>`, so a state or an initial-conditions type owes no zero().
    STATIC_REQUIRE(SimulationState<NoZeroInput>);
    STATIC_REQUIRE(SimulationInitialConditions<NoZeroInput>);
}

#endif // WITH_LOW_LEVEL_TESTS
