// SPDX-License-Identifier: MPL-2.0
// docs/GlmFieldSimilarityTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include <array>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "glm/mat2x3.hpp"
#include "glm/mat3x3.hpp"
#include "glm/mat4x4.hpp"
#include "OGSimulation/CorrectionCache.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/SimulationComparisonGlm.h"
#include "OGSimulation/SimulationComposite.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationSerialization.h"

namespace glmFieldSimilarity
{
    struct ProbeState
    {
        std::int32_t    phase = 0;
        LinearBodyState bodyState;
        glm::vec2       aim{0.f, 1.f};
        glm::quat       facing{1.f, 0.f, 0.f, 0.f};
    };

    struct ProbeInput
    {
        std::int32_t value = 0;
    };
}

template <>
struct SerializableFields<glmFieldSimilarity::ProbeState>
{
    static constexpr auto get()
    {
        using S = glmFieldSimilarity::ProbeState;
        return std::make_tuple(SIM_MEMBER(S, phase), SIM_MEMBER(S, bodyState), SIM_MEMBER(S, aim),
                               SIM_MEMBER(S, facing));
    }
};

namespace glmFieldSimilarityAdlOnly
{
    struct Hider {};
    void isSimilarToField(Hider);

    template <typename T>
    concept FoundByAdlAlone = requires(const T& v) {
        { isSimilarToField(v, v) } -> std::same_as<bool>;
    };
}

static_assert(glmFieldSimilarityAdlOnly::FoundByAdlAlone<glm::vec3>,
    "the glm::vec3 overload of isSimilarToField must be found by ADL alone (namespace glm), or the correction "
    "comparator's dependent call compares vec3 fields exactly. og-input-latency task 2e");
static_assert(glmFieldSimilarityAdlOnly::FoundByAdlAlone<glm::vec2>,
    "the glm::vec2 overload of isSimilarToField must be found by ADL alone. og-input-latency task 2e");
static_assert(glmFieldSimilarityAdlOnly::FoundByAdlAlone<glm::quat>,
    "the glm::quat overload of isSimilarToField must be found by ADL alone. og-input-latency task 2e");
static_assert(!glmFieldSimilarityAdlOnly::FoundByAdlAlone<float>,
    "FoundByAdlAlone must be false for a type with no associated namespace, or the three asserts above prove nothing. "
    "og-input-latency task 2e");

static_assert(simulationTypesDetail::GlmFloatType<glm::vec3>
    && simulationTypesDetail::GlmFloatType<glm::vec2>
    && simulationTypesDetail::GlmFloatType<glm::quat>
    && simulationTypesDetail::GlmFloatType<glm::vec4>
    && simulationTypesDetail::GlmFloatType<glm::dvec3>,
    "the generic isSimilarToField's static_assert must recognise every glm float vector and quaternion, or a glm "
    "field without an overload compares exactly again. og-input-latency task 2e");
static_assert(simulationTypesDetail::GlmFloatType<glm::mat3>
    && simulationTypesDetail::GlmFloatType<glm::mat4>
    && simulationTypesDetail::GlmFloatType<glm::mat2x3>
    && simulationTypesDetail::GlmFloatType<glm::dmat3>,
    "the generic isSimilarToField's static_assert must recognise every glm float matrix, or a glm::mat field "
    "compares exactly with no compile error. Was: the concept required a member x, which no glm::mat has "
    "(og-input-latency task 2g)");
static_assert(!simulationTypesDetail::GlmFloatType<float>
    && !simulationTypesDetail::GlmFloatType<std::int32_t>
    && !simulationTypesDetail::GlmFloatType<glm::ivec3>
    && !simulationTypesDetail::GlmFloatType<glmFieldSimilarity::ProbeState>,
    "the generic isSimilarToField's static_assert must not reject scalars, integer vectors or aggregates. "
    "og-input-latency task 2e");
static_assert(!simulationTypesDetail::GlmFloatType<std::array<float, 3>>
    && !simulationTypesDetail::GlmFloatType<std::vector<float>>,
    "std::array<float, N> is out of the pin's scope (it compares exactly), and std::vector has its own overload: "
    "neither is a glm type. og-input-latency task 2g");

namespace
{
    using glmFieldSimilarity::ProbeState;
    using ProbeComposite = SimulationStateComposite<ProbeState>;
    using ProbeCache     = StateCorrectionCache<ProbeComposite, glmFieldSimilarity::ProbeInput>;

    constexpr float kSubEpsilon   = 5e-5f;
    constexpr float kAboveEpsilon = 2e-4f;

    const glm::vec3 kFacingAxis(0.f, 1.f, 0.f);
    constexpr float kFacingAngle = 0.7f;

    ProbeState slopeIdle()
    {
        ProbeState s;
        s.phase                    = 1;
        s.bodyState.position       = glm::vec3(13.0513f, 5.2657f, -16.7586f);
        s.bodyState.linearVelocity = glm::vec3(0.f, -1.5e-9f, 4.2e-10f);
        s.aim                      = glm::vec2(0.6f, 0.8f);
        s.facing                   = glm::angleAxis(kFacingAngle, kFacingAxis);
        return s;
    }

    float ulpUp(float v) { return std::nextafter(v, std::numeric_limits<float>::infinity()); }

    glm::quat everyComponentUlpUp(const glm::quat& q)
    {
        return glm::quat(ulpUp(q.w), ulpUp(q.x), ulpUp(q.y), ulpUp(q.z));
    }

    ProbeComposite composite(const ProbeState& s)
    {
        ProbeComposite c;
        c.edit<ProbeState>() = s;
        return c;
    }

    bool sameBits(const ProbeState& a, const ProbeState& b)
    {
        return a.phase == b.phase && a.bodyState.position == b.bodyState.position
            && a.bodyState.linearVelocity == b.bodyState.linearVelocity && a.aim == b.aim && a.facing == b.facing;
    }

    struct Verdict
    {
        bool fieldwise       = false;
        bool composite       = false;
        bool cache           = false;
        bool slotIsPredicted = false;
        bool slotIsAuthority = false;
    };

    Verdict verdictOf(const ProbeState& predicted, const ProbeState& authority)
    {
        REQUIRE_FALSE(sameBits(predicted, authority));

        Verdict v;
        v.fieldwise = fieldwiseIsSimilarTo(predicted, authority);
        v.composite = composite(predicted).isSimilarTo(composite(authority));

        ProbeCache cache(nullptr);
        cache.pushPredictionTick(500u);
        cache.pushPredictionState(composite(predicted));
        CorrectionInsertVerdict report;
        cache.tryInsertingCorrectState(composite(authority), 500u, kNoInputCaptureTick, &report);
        REQUIRE(report.landed);
        v.cache = report.predictionWasCorrect;
        const ProbeState& held = cache.getState(cache.getCacheIndex(500u)).get<ProbeState>();
        v.slotIsPredicted = sameBits(held, predicted);
        v.slotIsAuthority = sameBits(held, authority);
        return v;
    }

    void requireSimilar(const ProbeState& predicted, const ProbeState& authority)
    {
        const Verdict v = verdictOf(predicted, authority);
        CHECK(v.fieldwise);
        CHECK(v.composite);
        CHECK(v.cache);
        CHECK(v.slotIsPredicted);
    }

    void requireDissimilar(const ProbeState& predicted, const ProbeState& authority)
    {
        const Verdict v = verdictOf(predicted, authority);
        CHECK_FALSE(v.fieldwise);
        CHECK_FALSE(v.composite);
        CHECK_FALSE(v.cache);
        CHECK(v.slotIsAuthority);
    }

    ProbeState facingRotatedFurtherBy(const ProbeState& s, float radians)
    {
        ProbeState r = s;
        r.facing = glm::angleAxis(kFacingAngle + radians, kFacingAxis);
        return r;
    }

    glm::quat rotatedBy(const glm::quat& base, float radians, const glm::vec3& unitAxis)
    {
        return base * glm::angleAxis(radians, unitAxis);
    }

    float component(const glm::quat& q, int i)
    {
        return i == 0 ? q.x : i == 1 ? q.y : i == 2 ? q.z : q.w;
    }
}

TEST_CASE("Glm field similarity: a vec3 field 1 ULP apart is similar, through the fold, the composite and the "
          "correction cache",
          "[Network][GlmFieldSimilarity]")
{
    const ProbeState predicted = slopeIdle();

    SECTION("position, one component")
    {
        ProbeState authority = predicted;
        authority.bodyState.position.x = ulpUp(predicted.bodyState.position.x);
        REQUIRE(authority.bodyState.position.x != predicted.bodyState.position.x);
        requireSimilar(predicted, authority);
    }

    SECTION("linear velocity, every component")
    {
        ProbeState authority = predicted;
        authority.bodyState.linearVelocity = glm::vec3(ulpUp(predicted.bodyState.linearVelocity.x),
                                                       ulpUp(predicted.bodyState.linearVelocity.y),
                                                       ulpUp(predicted.bodyState.linearVelocity.z));
        REQUIRE(authority.bodyState.linearVelocity.y != predicted.bodyState.linearVelocity.y);
        requireSimilar(predicted, authority);
    }

    SECTION("linear velocity, the largest delta task 2c measured (4.76837e-07)")
    {
        ProbeState authority = predicted;
        authority.bodyState.linearVelocity.y = predicted.bodyState.linearVelocity.y + 4.76837e-07f;
        requireSimilar(predicted, authority);
    }
}

TEST_CASE("Glm field similarity: a vec3 field inside the epsilon is similar, above it dissimilar",
          "[Network][GlmFieldSimilarity]")
{
    const ProbeState predicted = slopeIdle();

    SECTION("inside")
    {
        ProbeState authority = predicted;
        authority.bodyState.linearVelocity.z += kSubEpsilon;
        requireSimilar(predicted, authority);
    }

    SECTION("above")
    {
        ProbeState authority = predicted;
        authority.bodyState.linearVelocity.y += kAboveEpsilon;
        requireDissimilar(predicted, authority);
    }

    SECTION("above, on position")
    {
        ProbeState authority = predicted;
        authority.bodyState.position.x += kAboveEpsilon;
        requireDissimilar(predicted, authority);
    }
}

TEST_CASE("Glm field similarity: vec2 and quat fields use their own overloads' tolerances",
          "[Network][GlmFieldSimilarity]")
{
    const ProbeState predicted = slopeIdle();

    SECTION("vec2 1 ULP apart")
    {
        ProbeState authority = predicted;
        authority.aim.x = ulpUp(predicted.aim.x);
        REQUIRE(authority.aim.x != predicted.aim.x);
        requireSimilar(predicted, authority);
    }

    SECTION("vec2 above the epsilon")
    {
        ProbeState authority = predicted;
        authority.aim.x += kAboveEpsilon;
        requireDissimilar(predicted, authority);
    }

    SECTION("quat 1 ULP apart")
    {
        ProbeState authority = predicted;
        authority.facing.w = ulpUp(predicted.facing.w);
        REQUIRE(authority.facing.w != predicted.facing.w);
        requireSimilar(predicted, authority);
    }

    SECTION("quat rotated by 0.05 rad, above it")
    {
        requireDissimilar(predicted, facingRotatedFurtherBy(predicted, 0.05f));
    }
}

TEST_CASE("Glm field similarity: a quat field compares per component after sign alignment, on the vectors' scale",
          "[Network][GlmFieldSimilarity]")
{
    const ProbeState predicted = slopeIdle();

    SECTION("1 ULP apart on every component")
    {
        ProbeState authority = predicted;
        authority.facing = everyComponentUlpUp(predicted.facing);
        REQUIRE(authority.facing.y != predicted.facing.y);
        REQUIRE(authority.facing.w != predicted.facing.w);
        requireSimilar(predicted, authority);
    }

    SECTION("q against -q, the same rotation")
    {
        ProbeState authority = predicted;
        authority.facing = -predicted.facing;
        requireSimilar(predicted, authority);
    }

    SECTION("q against -q 1 ULP apart on every component")
    {
        ProbeState authority = predicted;
        authority.facing = everyComponentUlpUp(-predicted.facing);
        requireSimilar(predicted, authority);
    }

    SECTION("rotated by 1e-4 rad: similar")
    {
        requireSimilar(predicted, facingRotatedFurtherBy(predicted, 1e-4f));
    }

    SECTION("rotated by 5e-4 rad: dissimilar")
    {
        requireDissimilar(predicted, facingRotatedFurtherBy(predicted, 5e-4f));
    }

    SECTION("rotated by 0.01 rad: dissimilar since task 2g (task 2e's dot-product rule called it similar)")
    {
        requireDissimilar(predicted, facingRotatedFurtherBy(predicted, 0.01f));
    }
}

TEST_CASE("Glm field similarity: the quat rule's angle bounds, 2e-4 rad from identity and 4e-4 rad at most",
          "[Network][GlmFieldSimilarity]")
{
    const float perAxisBound  = 2.f * kDefaultSimilarityEpsilon;
    const float anyAxisBound  = 4.f * kDefaultSimilarityEpsilon;
    const float justInside    = 0.95f;
    const float justOutside   = 1.05f;
    const glm::quat identity(1.f, 0.f, 0.f, 0.f);
    const glm::vec3 unitX(1.f, 0.f, 0.f);

    SECTION("the rationale's numbers")
    {
        const double eps = static_cast<double>(kDefaultSimilarityEpsilon);
        const double degPerRad = 180.0 / 3.14159265358979323846;
        CHECK(2.0 * std::asin(eps) == Catch::Approx(2.0e-4).epsilon(1e-6));
        CHECK(2.0 * std::asin(eps) * degPerRad == Catch::Approx(0.011459).epsilon(1e-4));
        CHECK(4.0 * eps * degPerRad == Catch::Approx(0.022918).epsilon(1e-4));
        CHECK(2.0 * std::acos(1.0 - eps) == Catch::Approx(0.028285).epsilon(1e-4));
        CHECK(2.0 * std::acos(1.0 - eps) * degPerRad == Catch::Approx(1.6206).epsilon(1e-4));
    }

    SECTION("from identity, about one axis: that axis's component changes by sin(theta/2), so the bound is 2e-4 rad")
    {
        const glm::vec3 axes[] = {glm::vec3(1.f, 0.f, 0.f), glm::vec3(0.f, 1.f, 0.f), glm::vec3(0.f, 0.f, 1.f)};
        for (int k = 0; k < 3; ++k)
        {
            CAPTURE(k);
            const float inside  = justInside * perAxisBound;
            const float outside = justOutside * perAxisBound;
            const glm::quat in  = rotatedBy(identity, inside, axes[k]);
            CHECK(component(in, k) == Catch::Approx(std::sin(inside / 2.f)).epsilon(1e-5));
            CHECK(isSimilarToField(identity, in));
            CHECK_FALSE(isSimilarToField(identity, rotatedBy(identity, outside, axes[k])));
        }
    }

    SECTION("any base and axis: always similar below 2e-4 rad, never similar from 4e-4 rad")
    {
        const glm::quat bases[] = {
            identity,
            glm::angleAxis(kFacingAngle, kFacingAxis),
            glm::angleAxis(1.9f, glm::normalize(glm::vec3(1.f, 2.f, 3.f))),
            glm::angleAxis(3.0f, glm::normalize(glm::vec3(-1.f, 0.5f, 2.f))),
            glm::angleAxis(2.5f, unitX),
            glm::angleAxis(-1.2f, glm::normalize(glm::vec3(0.3f, -0.8f, 0.1f))),
        };
        const glm::vec3 axes[] = {
            glm::vec3(1.f, 0.f, 0.f), glm::vec3(0.f, 1.f, 0.f), glm::vec3(0.f, 0.f, 1.f),
            glm::normalize(glm::vec3(1.f, 1.f, 1.f)), glm::normalize(glm::vec3(-2.f, 1.f, 0.5f)),
        };
        int pairs = 0;
        for (const glm::quat& base : bases)
        {
            for (const glm::vec3& axis : axes)
            {
                const glm::quat in  = rotatedBy(base, justInside * perAxisBound, axis);
                const glm::quat out = rotatedBy(base, justOutside * anyAxisBound, axis);
                CHECK(isSimilarToField(base, in));
                CHECK(isSimilarToField(in, base));
                CHECK_FALSE(isSimilarToField(base, out));
                CHECK_FALSE(isSimilarToField(out, base));
                ++pairs;
            }
        }
        CHECK(pairs == 30);
    }

    SECTION("the 4e-4 rad bound is reached: a base where the rotation spreads evenly over all four components")
    {
        const glm::quat pureX(0.f, 1.f, 0.f, 0.f);
        const glm::quat worst = glm::quat(0.5f, 0.5f, 0.5f, 0.5f) * glm::conjugate(pureX);
        const float inside  = justInside * anyAxisBound;
        const float outside = justOutside * anyAxisBound;
        const glm::quat in  = rotatedBy(worst, inside, unitX);
        for (int i = 0; i < 4; ++i)
        {
            CAPTURE(i);
            CHECK(std::abs(component(in, i) - component(worst, i)) == Catch::Approx(inside / 4.f).epsilon(0.01));
        }
        CHECK(isSimilarToField(worst, in));
        CHECK_FALSE(isSimilarToField(worst, rotatedBy(worst, outside, unitX)));
        CHECK_FALSE(isSimilarToField(identity, rotatedBy(identity, inside, unitX)));
    }
}

TEST_CASE("Glm field similarity: the walk's leaf shim resolves the same glm overload as the fold",
          "[Network][GlmFieldSimilarity]")
{
    const glm::vec3 a(13.0513f, 5.2657f, -16.7586f);
    const glm::vec3 b(ulpUp(a.x), a.y, a.z);
    CHECK(leafIsSimilarToAsFieldwiseDoes<glm::vec3>(a, b));
    CHECK_FALSE(leafIsSimilarToAsFieldwiseDoes<glm::vec3>(a, a + glm::vec3(kAboveEpsilon, 0.f, 0.f)));

    const glm::vec2 c(0.6f, 0.8f);
    CHECK(leafIsSimilarToAsFieldwiseDoes<glm::vec2>(c, glm::vec2(ulpUp(c.x), c.y)));

    const glm::quat q = glm::angleAxis(kFacingAngle, kFacingAxis);
    CHECK(leafIsSimilarToAsFieldwiseDoes<glm::quat>(q, glm::quat(ulpUp(q.w), q.x, q.y, q.z)));
    CHECK(leafIsSimilarToAsFieldwiseDoes<glm::quat>(q, -q));
    CHECK_FALSE(leafIsSimilarToAsFieldwiseDoes<glm::quat>(q, glm::angleAxis(kFacingAngle + 5e-4f, kFacingAxis)));
}

#endif // WITH_LOW_LEVEL_TESTS
