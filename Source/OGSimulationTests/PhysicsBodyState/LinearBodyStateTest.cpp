// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/PhysicsDeclaration.h"
#include "OGSimulation/QueryGeometry.h"
#include "OGSimulation/SimulationSerialization.h"

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// LinearBodyState — the slim, rotation-locked body-state wire shape, and the
// PhysicsDeclaration concept that finally makes the body-state contract
// checkable instead of duck-typed.
//
// WHY THESE TESTS EXIST. LinearBodyState is a LOSSY bridge on purpose: widening
// it back to PhysicsBodyState FABRICATES an identity rotation and a zero angular
// velocity. That is sound only for a body whose descriptor locks rotation, and
// it is exactly the class of failure this codebase has already paid for twice
// (a sign convention documented backwards, an enum inferred wrongly). So the
// lossy behaviour is PINNED here rather than left to be discovered:
//
//   * the fabricated values are asserted, not merely described;
//   * the widening is asserted to be IMPLICIT — the engine's rewind push
//     `pushBodyState(ownBodyId, D::bodyStateOf(state))` takes a
//     `const PhysicsBodyState&` at a generic call site, and that implicit
//     conversion is the zero-edit property the whole design rests on. Marking
//     the operator `explicit` breaks `is_convertible` below, loudly;
//   * the narrowing direction is asserted to be ASSIGNMENT-ONLY. There is no
//     converting constructor, so `LinearBodyState x = someFullState;` and
//     silent lossy pass-by-value do not compile.
//
// The PhysicsDeclaration concept is pinned here for the same reason, by MOCK
// declarations only — no real declaration is asserted anywhere yet. Two of its
// requirements are TIGHTER than "the member exists", and each has a control
// below that goes false the moment the requirement is relaxed:
//   * the CONST bodyStateOf must widen to PhysicsBodyState, because the rewind
//     push reads through it and converts;
//   * `bindings` must BE the shared PhysicsRuntimeBindings, as a mutable
//     lvalue, because the creation fold writes all five of its members.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // --- compile-time pins for the two bridge directions --------------------

    // Widening MUST be implicit. is_convertible tests exactly that: it fails the
    // moment someone marks `operator PhysicsBodyState()` explicit.
    static_assert(std::is_convertible_v<LinearBodyState, PhysicsBodyState>,
                  "the widening operator must stay IMPLICIT — the rewind push relies on it");

    // Narrowing is assignment-only: assignable, but NOT constructible and NOT
    // implicitly convertible from a full body state.
    static_assert(std::is_assignable_v<LinearBodyState&, const PhysicsBodyState&>);
    static_assert(!std::is_constructible_v<LinearBodyState, const PhysicsBodyState&>);
    static_assert(!std::is_convertible_v<const PhysicsBodyState&, LinearBodyState>);

    // Both body-state types model the pair the generic sites need.
    static_assert(BodyStateLike<PhysicsBodyState>);
    static_assert(BodyStateLike<LinearBodyState>);

    static_assert(syncSize<LinearBodyState>() == 24u, "LinearBodyState is a 24-byte wire shape");
    static_assert(syncSize<PhysicsBodyState>() == 52u, "PhysicsBodyState is a 52-byte wire shape");

    // --- a full body state with NOTHING at its default ----------------------
    // Every field is distinguishable, so a dropped or fabricated one is visible.
    PhysicsBodyState makeFullState()
    {
        PhysicsBodyState full;
        full.position        = glm::vec3(1.f, 2.f, 3.f);
        full.rotation        = glm::quat(0.5f, 0.5f, 0.5f, 0.5f);   // NOT identity
        full.linearVelocity  = glm::vec3(4.f, 5.f, 6.f);
        full.angularVelocity = glm::vec3(7.f, 8.f, 9.f);            // NOT zero
        return full;
    }

    // --- UE-free byte buffer satisfying the serializer's BUFFER CONCEPT ------
    struct LinearTestBuffer
    {
        std::vector<std::uint8_t> bytes = std::vector<std::uint8_t>(128, 0u);

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

    // --- mock game/sub static data and declarations -------------------------
    // Shaped exactly like a real sub-simulation's, so what the concept accepts
    // here is what it accepts in the game.
    struct MockSubStaticData { float radius = 30.f; };
    struct MockGameStaticData { MockSubStaticData movement; };

    const PhysicalObjectDescriptor& mockDescriptor()
    {
        static const PhysicalObjectDescriptor descriptor{
            .body   = BodyDescriptor{ .simulatePhysics = true, .enableGravity = false },
            .shapes = std::vector<ShapeDescriptor>{} };
        return descriptor;
    }

    // The shape task 5 gives the movement skeleton: a slim, rotation-locked body.
    struct MockLinearState { LinearBodyState bodyState; };
    struct MockLinearDecl
    {
        static const PhysicalObjectDescriptor& descriptor() { return mockDescriptor(); }
        static constexpr const char* name = "MockMovementBody";
        static const MockSubStaticData& staticDataOf(const MockGameStaticData& gsd) { return gsd.movement; }
        static std::vector<QueryVolumeDescriptor> queryVolumes(const MockSubStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const MockSubStaticData&) { return glm::vec3(0.f, 0.f, 30.f); }

        using StateType = MockLinearState;
        static       LinearBodyState& bodyStateOf(      StateType& s) { return s.bodyState; }
        static const LinearBodyState& bodyStateOf(const StateType& s) { return s.bodyState; }

        PhysicsRuntimeBindings bindings;
    };

    // The shape the four EXISTING sub-simulations already have: a full body
    // state. The concept must accept both, or it is not a widening at all.
    struct MockFullState { PhysicsBodyState bodyState; };
    struct MockFullDecl
    {
        static const PhysicalObjectDescriptor& descriptor() { return mockDescriptor(); }
        static constexpr const char* name = "MockFullBody";
        static const MockSubStaticData& staticDataOf(const MockGameStaticData& gsd) { return gsd.movement; }
        static std::vector<QueryVolumeDescriptor> queryVolumes(const MockSubStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const MockSubStaticData&) { return glm::vec3(0.f); }

        using StateType = MockFullState;
        static       PhysicsBodyState& bodyStateOf(      StateType& s) { return s.bodyState; }
        static const PhysicsBodyState& bodyStateOf(const StateType& s) { return s.bodyState; }

        PhysicsRuntimeBindings bindings;
    };

    // --- negative controls --------------------------------------------------
    // NOTHING selects these but the concept; they exist to prove it can say no.

    // (1) bodyStateOf returns something that is not a body state at all.
    struct NotABodyState { int x = 0; };
    struct MockWrongState { NotABodyState bodyState; };
    struct MockWrongBodyStateDecl
    {
        static const PhysicalObjectDescriptor& descriptor() { return mockDescriptor(); }
        static constexpr const char* name = "MockWrongBodyState";
        static const MockSubStaticData& staticDataOf(const MockGameStaticData& gsd) { return gsd.movement; }
        static std::vector<QueryVolumeDescriptor> queryVolumes(const MockSubStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const MockSubStaticData&) { return glm::vec3(0.f); }

        using StateType = MockWrongState;
        static       NotABodyState& bodyStateOf(      StateType& s) { return s.bodyState; }
        static const NotABodyState& bodyStateOf(const StateType& s) { return s.bodyState; }

        PhysicsRuntimeBindings bindings;
    };

    // (2) the exact gap the concept was written for: no staticDataOf, so the
    //     creation fold cannot ask the declaration for its own static-data slice
    //     and needs a hand-written engine-side branch instead.
    struct MockNoStaticDataOfDecl
    {
        static const PhysicalObjectDescriptor& descriptor() { return mockDescriptor(); }
        static constexpr const char* name = "MockNoStaticDataOf";
        static std::vector<QueryVolumeDescriptor> queryVolumes(const MockSubStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const MockSubStaticData&) { return glm::vec3(0.f); }

        using StateType = MockLinearState;
        static       LinearBodyState& bodyStateOf(      StateType& s) { return s.bodyState; }
        static const LinearBodyState& bodyStateOf(const StateType& s) { return s.bodyState; }

        PhysicsRuntimeBindings bindings;
    };

    // (3) no bindings member — the handles the folds read back after creation.
    struct MockNoBindingsDecl
    {
        static const PhysicalObjectDescriptor& descriptor() { return mockDescriptor(); }
        static constexpr const char* name = "MockNoBindings";
        static const MockSubStaticData& staticDataOf(const MockGameStaticData& gsd) { return gsd.movement; }
        static std::vector<QueryVolumeDescriptor> queryVolumes(const MockSubStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const MockSubStaticData&) { return glm::vec3(0.f); }

        using StateType = MockLinearState;
        static       LinearBodyState& bodyStateOf(      StateType& s) { return s.bodyState; }
        static const LinearBodyState& bodyStateOf(const StateType& s) { return s.bodyState; }
    };

    // --- negative controls added by task 10a (F-1 / F-2 tightenings) ---------

    // (4) ⚠ F-2, AND THE SHAPE ALL FOUR EXISTING SUB-SIMULATIONS ARE IN TODAY:
    //     a `bindings` whose type is field-for-field identical to
    //     PhysicsRuntimeBindings — same members, same order, same types — but is
    //     a DISTINCT struct. The creation fold writes through `decl.bindings`,
    //     and a look-alike is not the type that fold was compiled against. This
    //     control is why adopting the shared type is MANDATORY rather than
    //     cosmetic: the tree is in this shape right now, and it does NOT satisfy
    //     the concept.
    struct LookAlikeRuntimeBindings
    {
        BodyId                     ownBodyId;
        BodyId                     parentBodyId;
        glm::vec3                  attachmentOffset;
        std::vector<ShapeId>       shapeIds;
        std::vector<QueryVolumeId> queryVolumeIds;
    };
    struct MockDistinctBindingsStructDecl
    {
        static const PhysicalObjectDescriptor& descriptor() { return mockDescriptor(); }
        static constexpr const char* name = "MockDistinctBindings";
        static const MockSubStaticData& staticDataOf(const MockGameStaticData& gsd) { return gsd.movement; }
        static std::vector<QueryVolumeDescriptor> queryVolumes(const MockSubStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const MockSubStaticData&) { return glm::vec3(0.f); }

        using StateType = MockLinearState;
        static       LinearBodyState& bodyStateOf(      StateType& s) { return s.bodyState; }
        static const LinearBodyState& bodyStateOf(const StateType& s) { return s.bodyState; }

        LookAlikeRuntimeBindings bindings;   // everything else conforms
    };

    // (5) F-1: the const overload EXISTS but does not WIDEN. Everything else is
    //     sound — the MUTABLE overload's referent is a perfectly good
    //     BodyStateLike, which is all the concept asked of the pair before this
    //     tightening. So this declaration used to SATISFY the concept and would
    //     still have failed inside the rewind push, which reads through the
    //     const overload and converts it to `const PhysicsBodyState&`.
    struct MockSplitState
    {
        LinearBodyState bodyState;
        int             notABodyState = 0;
    };
    struct MockNonWideningConstBodyStateDecl
    {
        static const PhysicalObjectDescriptor& descriptor() { return mockDescriptor(); }
        static constexpr const char* name = "MockNonWideningConstBodyState";
        static const MockSubStaticData& staticDataOf(const MockGameStaticData& gsd) { return gsd.movement; }
        static std::vector<QueryVolumeDescriptor> queryVolumes(const MockSubStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const MockSubStaticData&) { return glm::vec3(0.f); }

        using StateType = MockSplitState;
        static       LinearBodyState& bodyStateOf(      StateType& s) { return s.bodyState; }
        static const int&             bodyStateOf(const StateType& s) { return s.notABodyState; }

        PhysicsRuntimeBindings bindings;
    };

    // (6) The DISCRIMINATION control for F-2, and the reason it is here: control
    //     (4) is false under the OLD requirement too (a look-alike is not
    //     convertible to `const PhysicsRuntimeBindings&` either), so on its own
    //     it cannot show that the tightening changed anything. This one can: a
    //     read-only `bindings` SATISFIED
    //     `convertible_to<const PhysicsRuntimeBindings&>` and fails
    //     `same_as<PhysicsRuntimeBindings&>`. The creation fold writes all five
    //     members, so read-only was false confidence.
    struct MockConstBindingsDecl
    {
        static const PhysicalObjectDescriptor& descriptor() { return mockDescriptor(); }
        static constexpr const char* name = "MockConstBindings";
        static const MockSubStaticData& staticDataOf(const MockGameStaticData& gsd) { return gsd.movement; }
        static std::vector<QueryVolumeDescriptor> queryVolumes(const MockSubStaticData&) { return {}; }
        static glm::vec3 attachmentOffset(const MockSubStaticData&) { return glm::vec3(0.f); }

        using StateType = MockLinearState;
        static       LinearBodyState& bodyStateOf(      StateType& s) { return s.bodyState; }
        static const LinearBodyState& bodyStateOf(const StateType& s) { return s.bodyState; }

        const PhysicsRuntimeBindings bindings{};
    };
} // namespace

//////////////////////////////////////////////////////////////////////////////

TEST_CASE("LinearBodyState capture bridge drops rotation and angular velocity",
          "[LinearBodyState][CaptureBridge]")
{
    // This IS the executor's capture loop, spelled out:
    //     D::bodyStateOf(state) = adapter.captureBodyState(id)
    const PhysicsBodyState captured = makeFullState();

    LinearBodyState slim;
    slim = captured;

    CHECK(slim.position.x == 1.f);
    CHECK(slim.position.y == 2.f);
    CHECK(slim.position.z == 3.f);
    CHECK(slim.linearVelocity.x == 4.f);
    CHECK(slim.linearVelocity.y == 5.f);
    CHECK(slim.linearVelocity.z == 6.f);

    // The dropped half is not stored anywhere: LinearBodyState has exactly two
    // members, so the narrowing is total by construction.
    CHECK(sizeof(LinearBodyState) == sizeof(glm::vec3) * 2);

    // Assigning again from a DIFFERENT full state fully overwrites — no residue.
    PhysicsBodyState second;
    second.position       = glm::vec3(-1.f, -2.f, -3.f);
    second.linearVelocity = glm::vec3(0.f);
    slim = second;
    CHECK(slim.position.x == -1.f);
    CHECK(slim.linearVelocity.x == 0.f);
}

TEST_CASE("LinearBodyState push bridge fabricates identity rotation, zero angular",
          "[LinearBodyState][PushBridge]")
{
    LinearBodyState slim;
    slim.position       = glm::vec3(10.f, 20.f, 30.f);
    slim.linearVelocity = glm::vec3(-1.f, -2.f, -3.f);

    // Implicit — exactly as at `pushBodyState(id, D::bodyStateOf(state))`.
    const PhysicsBodyState widened = slim;

    CHECK(widened.position.x == 10.f);
    CHECK(widened.position.y == 20.f);
    CHECK(widened.position.z == 30.f);
    CHECK(widened.linearVelocity.x == -1.f);
    CHECK(widened.linearVelocity.y == -2.f);
    CHECK(widened.linearVelocity.z == -3.f);

    // ⚠ THE FABRICATION. Sound ONLY for a body whose descriptor locks rotation.
    CHECK(widened.rotation.w == 1.f);
    CHECK(widened.rotation.x == 0.f);
    CHECK(widened.rotation.y == 0.f);
    CHECK(widened.rotation.z == 0.f);
    CHECK(widened.angularVelocity.x == 0.f);
    CHECK(widened.angularVelocity.y == 0.f);
    CHECK(widened.angularVelocity.z == 0.f);

    // Full -> slim -> full: the surviving half survives, the rest is fabricated.
    LinearBodyState roundTrip;
    roundTrip = makeFullState();
    const PhysicsBodyState back = roundTrip;
    CHECK(back.position.x == 1.f);
    CHECK(back.linearVelocity.z == 6.f);
    CHECK(back.rotation.w == 1.f);          // the original was (0.5, 0.5, 0.5, 0.5)
    CHECK(back.angularVelocity.y == 0.f);   // the original was (7, 8, 9)
}

TEST_CASE("LinearBodyState bridges by assignment only, never by construction",
          "[LinearBodyState][NoConvertingCtor]")
{
    // The compile-time half of this case is the static_assert block at the top
    // of this file; it fails the build the day someone adds a converting
    // constructor. Restated here so a reader of the case sees the claim:
    STATIC_REQUIRE(std::is_assignable_v<LinearBodyState&, const PhysicsBodyState&>);
    STATIC_REQUIRE_FALSE(std::is_constructible_v<LinearBodyState, const PhysicsBodyState&>);
    STATIC_REQUIRE_FALSE(std::is_convertible_v<const PhysicsBodyState&, LinearBodyState>);

    // ... and the widening direction stays implicit, which is what buys the
    // zero-edit rewind push.
    STATIC_REQUIRE(std::is_convertible_v<LinearBodyState, PhysicsBodyState>);

    // Assignment into an EXISTING member of an existing state — the only shape
    // capture ever uses.
    MockLinearState state;
    MockLinearDecl::bodyStateOf(state) = makeFullState();
    CHECK(state.bodyState.position.y == 2.f);
    CHECK(state.bodyState.linearVelocity.y == 5.f);
}

TEST_CASE("LinearBodyState is a 24-byte wire shape with an exact round-trip",
          "[LinearBodyState][WireShape]")
{
    STATIC_REQUIRE(syncSize<LinearBodyState>() == 24u);
    STATIC_REQUIRE(syncSize<PhysicsBodyState>() == 52u);
    // The saving the swap buys per body, stated as the subtraction:
    STATIC_REQUIRE(syncSize<PhysicsBodyState>() - syncSize<LinearBodyState>() == 28u);

    LinearBodyState written;
    written.position       = glm::vec3(1.5f, -2.5f, 3.25f);
    written.linearVelocity = glm::vec3(-4.75f, 5.f, -6.125f);

    LinearTestBuffer buffer;
    const std::uint32_t bytes = writeToSyncedBuffer(written, buffer, 0u);
    CHECK(bytes == 24u);

    LinearBodyState read;
    readFromSyncedBuffer(read, buffer, 0u);

    CHECK(read.position.x == 1.5f);
    CHECK(read.position.y == -2.5f);
    CHECK(read.position.z == 3.25f);
    CHECK(read.linearVelocity.x == -4.75f);
    CHECK(read.linearVelocity.y == 5.f);
    CHECK(read.linearVelocity.z == -6.125f);

    // Fieldwise similarity comes from SerializableFields, like every other state.
    CHECK(fieldwiseIsSimilarTo(written, read));

    LinearBodyState different = read;
    different.linearVelocity.z = 99.f;
    CHECK_FALSE(fieldwiseIsSimilarTo(written, different));
}

TEST_CASE("PhysicsDeclaration accepts both body-state shapes and rejects gaps",
          "[LinearBodyState][PhysicsDeclaration]")
{
    // A declaration carrying the SLIM shape and one carrying the FULL shape are
    // both valid: the concept constrains the contract, not the wire size.
    STATIC_REQUIRE(PhysicsDeclaration<MockLinearDecl, MockGameStaticData>);
    STATIC_REQUIRE(PhysicsDeclaration<MockFullDecl,   MockGameStaticData>);

    // Negative controls — each removes exactly one requirement.
    STATIC_REQUIRE_FALSE(PhysicsDeclaration<MockWrongBodyStateDecl, MockGameStaticData>);
    STATIC_REQUIRE_FALSE(PhysicsDeclaration<MockNoStaticDataOfDecl, MockGameStaticData>);
    STATIC_REQUIRE_FALSE(PhysicsDeclaration<MockNoBindingsDecl,     MockGameStaticData>);

    // staticDataOf is what lets the creation fold reach a declaration's own
    // static-data slice generically, without an engine-side per-declaration arm.
    const MockGameStaticData gameStaticData;
    CHECK(MockLinearDecl::staticDataOf(gameStaticData).radius == 30.f);
    CHECK(MockLinearDecl::queryVolumes(MockLinearDecl::staticDataOf(gameStaticData)).empty());
    CHECK(MockLinearDecl::attachmentOffset(MockLinearDecl::staticDataOf(gameStaticData)).z == 30.f);

    // One shared bindings struct replaces the four byte-identical per-sim copies.
    // A ROOT body is its own parent rather than carrying a null id.
    MockLinearDecl declaration;
    declaration.bindings.ownBodyId    = BodyId{7u};
    declaration.bindings.parentBodyId = BodyId{7u};
    CHECK(declaration.bindings.ownBodyId == declaration.bindings.parentBodyId);
    CHECK(declaration.bindings.shapeIds.empty());
    CHECK(declaration.bindings.queryVolumeIds.empty());
}

TEST_CASE("PhysicsDeclaration requires the const bodyStateOf overload to widen",
          "[LinearBodyState][ConstWidening]")
{
    // F-1. The rewind push is
    //     pushBodyState(decl.bindings.ownBodyId, D::bodyStateOf(state))
    // against `pushBodyState(BodyId, const PhysicsBodyState&)`. It reads through
    // the CONST overload and lets it convert. Requiring that overload merely to
    // EXIST left the conversion assumed; it is now checked.
    STATIC_REQUIRE_FALSE(PhysicsDeclaration<MockNonWideningConstBodyStateDecl, MockGameStaticData>);

    // ...and it is false for exactly ONE reason, which is what makes it a
    // discriminating control rather than merely a false one. The mutable
    // overload's referent still models the body-state pair — every requirement
    // the concept made of this declaration before the tightening still holds.
    MockSplitState splitState;
    splitState.bodyState.position = glm::vec3(1.f, 2.f, 3.f);
    STATIC_REQUIRE(BodyStateLike<std::remove_cvref_t<
        decltype(MockNonWideningConstBodyStateDecl::bodyStateOf(splitState))>>);
    CHECK(MockNonWideningConstBodyStateDecl::bodyStateOf(splitState).position.y == 2.f);

    // The broken half: what the push would have been handed cannot become a
    // PhysicsBodyState at all.
    const MockSplitState& constState = splitState;
    STATIC_REQUIRE_FALSE(std::is_convertible_v<
        decltype(MockNonWideningConstBodyStateDecl::bodyStateOf(constState)), PhysicsBodyState>);

    // Both shipped shapes widen through their const overload — the slim one by
    // the fabricating operator, the full one by copy — so the tightening costs
    // the real declarations nothing.
    STATIC_REQUIRE(std::is_convertible_v<const LinearBodyState&,  PhysicsBodyState>);
    STATIC_REQUIRE(std::is_convertible_v<const PhysicsBodyState&, PhysicsBodyState>);
    STATIC_REQUIRE(PhysicsDeclaration<MockLinearDecl, MockGameStaticData>);
    STATIC_REQUIRE(PhysicsDeclaration<MockFullDecl,   MockGameStaticData>);
}

TEST_CASE("PhysicsDeclaration requires bindings to BE the shared writable type",
          "[LinearBodyState][SharedBindings]")
{
    // F-2. ⚠ THIS IS THE SHAPE ALL FOUR EXISTING SUB-SIMULATIONS ARE IN TODAY:
    // a per-sim `RuntimeBindings` struct, byte-identical to the shared one and
    // still a different type. Adopting `PhysicsRuntimeBindings` is therefore
    // MANDATORY, not cosmetic — this assertion is the reason why.
    STATIC_REQUIRE_FALSE(PhysicsDeclaration<MockDistinctBindingsStructDecl, MockGameStaticData>);

    // It is the bindings alone: same members, same order, same size.
    STATIC_REQUIRE(sizeof(LookAlikeRuntimeBindings) == sizeof(PhysicsRuntimeBindings));
    STATIC_REQUIRE_FALSE(std::is_same_v<LookAlikeRuntimeBindings, PhysicsRuntimeBindings>);
    MockDistinctBindingsStructDecl lookAlike;
    STATIC_REQUIRE_FALSE(std::is_same_v<decltype((lookAlike.bindings)), PhysicsRuntimeBindings&>);
    lookAlike.bindings.ownBodyId = BodyId{7u};
    CHECK(lookAlike.bindings.ownBodyId == BodyId{7u});

    // ⚠ The look-alike above cannot show that the requirement got TIGHTER: it is
    // false under `convertible_to<const PhysicsRuntimeBindings&>` too. This is
    // the control that isolates the change — read-only bindings SATISFIED the
    // loose form and fail `same_as<PhysicsRuntimeBindings&>`, even though the
    // creation fold writes all five members after it makes the body.
    STATIC_REQUIRE_FALSE(PhysicsDeclaration<MockConstBindingsDecl, MockGameStaticData>);
    MockConstBindingsDecl readOnly;
    STATIC_REQUIRE(std::is_convertible_v<decltype((readOnly.bindings)), const PhysicsRuntimeBindings&>);
    STATIC_REQUIRE_FALSE(std::is_same_v<decltype((readOnly.bindings)), PhysicsRuntimeBindings&>);
    CHECK(readOnly.bindings.shapeIds.empty());

    // The conforming shape: a mutable member OF the shared type, which is what
    // the fold needs in order to write back the handles it just created.
    MockLinearDecl conforming;
    STATIC_REQUIRE(std::is_same_v<decltype((conforming.bindings)), PhysicsRuntimeBindings&>);
    conforming.bindings.ownBodyId        = BodyId{11u};
    conforming.bindings.parentBodyId     = BodyId{11u};
    conforming.bindings.attachmentOffset = glm::vec3(0.f, 0.f, 30.f);
    conforming.bindings.shapeIds.push_back(ShapeId{2u});
    conforming.bindings.queryVolumeIds.push_back(QueryVolumeId{3u});
    CHECK(conforming.bindings.attachmentOffset.z == 30.f);
    CHECK(conforming.bindings.shapeIds.size() == 1u);
    CHECK(conforming.bindings.queryVolumeIds.size() == 1u);
}

#endif // WITH_LOW_LEVEL_TESTS
