// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/Mailbox.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/RenderCorrectionSmoothing.h"
#include "OGSimulation/RenderInterpolation.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTimeContext.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <random>
#include <type_traits>
#include <vector>

namespace
{
    using Snapshot = RenderSnapshotT<4>;
    using Correction = RenderCorrectionT<4>;
    using Smoother = RenderCorrectionSmoother<4>;
    using CorrectionRing = SpscRing<Correction, 8>;
    using Channel = SnapshotChannel<Snapshot, 4>;

    constexpr float kStepSeconds = 0.016667f;
    constexpr double kStepDeadlineSeconds = 1.0 / 60.0;
    const glm::quat kIdentity(1.f, 0.f, 0.f, 0.f);

    RenderBody makeBody(uint32_t id, uint8_t declaration, bool hasRotation, glm::vec3 position, glm::quat rotation = kIdentity)
    {
        RenderBody body;
        body.simulatableId = id;
        body.declarationIndex = declaration;
        body.hasRotation = hasRotation ? uint8_t{ 1 } : uint8_t{ 0 };
        body.positionCm = position;
        body.rotation = rotation;
        return body;
    }

    void fillSnapshot(Snapshot& snapshot, uint64_t step, std::initializer_list<RenderBody> bodies)
    {
        snapshot = Snapshot{};
        snapshot.physicsStep = step;
        snapshot.stepDeadlineSeconds = static_cast<double>(step) * kStepDeadlineSeconds;
        snapshot.tick = static_cast<SimTick>(step);
        for (const RenderBody& body : bodies)
        {
            snapshot.bodies[snapshot.bodyCount++] = body;
        }
        std::sort(snapshot.bodies.begin(), snapshot.bodies.begin() + snapshot.bodyCount, &renderSnapshotDetail::keyLess);
    }

    Correction oneBodyCorrection(uint64_t step, glm::dvec3 errorCm, glm::dquat errorRotation = glm::dquat(1.0, 0.0, 0.0, 0.0),
        float speedCmPerSecond = 0.f, uint32_t id = 1u, uint8_t declaration = 0u, bool hasRotation = false)
    {
        Correction correction;
        correction.physicsStep = step;
        correction.bodyCount = 1u;
        RenderCorrectionBody& body = correction.bodies[0];
        body.simulatableId = id;
        body.declarationIndex = declaration;
        body.hasRotation = hasRotation ? uint8_t{ 1 } : uint8_t{ 0 };
        body.speedCmPerSecond = speedCmPerSecond;
        body.errorCm = errorCm;
        body.errorRotation = errorRotation;
        return correction;
    }

    RenderCorrectionParams defaultParams()
    {
        RenderCorrectionParams params;
        params.stepSeconds = kStepSeconds;
        return params;
    }

    RenderInterpolationFrame frameAt(uint64_t nextStep, double alpha, std::optional<uint64_t> prevStep = std::nullopt)
    {
        RenderInterpolationFrame frame;
        frame.nextStep = nextStep;
        frame.alpha = alpha;
        frame.hasPrev = prevStep.has_value();
        frame.prevStep = prevStep.value_or(0u);
        return frame;
    }

    Snapshot posesWithOneBody(uint64_t step, glm::vec3 position = glm::vec3(0.f), bool hasRotation = false, glm::quat rotation = kIdentity)
    {
        Snapshot poses;
        fillSnapshot(poses, step, { makeBody(1u, 0u, hasRotation, position, rotation) });
        return poses;
    }

    // Engine-free transcription of Chaos's linear error decay (FProxyInterpolationError in PhysicsProxyBase.h,
    // UE 5.6) and of the snap decision in FSingleParticlePhysicsProxy::PullFromPhysicsState, kept in the
    // engine's own shape (int32 ticks, member names, the decay loop) so the smoother is checked against it.
    struct ChaosErrorReference
    {
        int32_t LastSimTick = 0;
        int32_t SimTicks = 0;
        int32_t EndDecayTick = 0;
        glm::dvec3 ErrorX{ 0.0 };
        glm::dvec3 ErrorXPrev{ 0.0 };
        glm::dquat ErrorR{ 1.0, 0.0, 0.0, 0.0 };
        glm::dquat ErrorRPrev{ 1.0, 0.0, 0.0, 0.0 };

        static glm::dquat Slerp(const glm::dquat& Quat1, const glm::dquat& Quat2, double Slerp)
        {
            double RawCosom = Quat1.x * Quat2.x + Quat1.y * Quat2.y + Quat1.z * Quat2.z + Quat1.w * Quat2.w;
            const double Sign = RawCosom >= 0.0 ? 1.0 : -1.0;
            RawCosom *= Sign;
            double Scale0 = 1.0 - Slerp;
            double Scale1 = Slerp * Sign;
            if (RawCosom < 0.9999)
            {
                const double Omega = std::acos(RawCosom);
                const double InvSin = 1.0 / std::sin(Omega);
                Scale0 = std::sin(Scale0 * Omega) * InvSin;
                Scale1 = std::sin(Scale1 * Omega) * InvSin;
            }
            glm::dquat Result(Scale0 * Quat1.w + Scale1 * Quat2.w, Scale0 * Quat1.x + Scale1 * Quat2.x,
                Scale0 * Quat1.y + Scale1 * Quat2.y, Scale0 * Quat1.z + Scale1 * Quat2.z);
            const double SquareSum = Result.x * Result.x + Result.y * Result.y + Result.z * Result.z + Result.w * Result.w;
            if (SquareSum >= 1e-8)
            {
                const double Scale = 1.0 / std::sqrt(SquareSum);
                return glm::dquat(Result.w * Scale, Result.x * Scale, Result.y * Scale, Result.z * Scale);
            }
            return glm::dquat(1.0, 0.0, 0.0, 0.0);
        }

        bool IsErrorSmoothing() const { return LastSimTick < EndDecayTick; }
        glm::dvec3 GetErrorX(float Alpha) const { return ErrorXPrev + static_cast<double>(Alpha) * (ErrorX - ErrorXPrev); }
        glm::dquat GetErrorR(float Alpha) const { return Slerp(ErrorRPrev, ErrorR, static_cast<double>(Alpha)); }

        void AccumlateErrorXR(const glm::dvec3& X, const glm::dquat& R, int32_t CurrentSimTick, int32_t ErrorSmoothDuration)
        {
            LastSimTick = CurrentSimTick - 1;
            EndDecayTick = LastSimTick + ErrorSmoothDuration;
            SimTicks = 0;
            if (IsErrorSmoothing())
            {
                ErrorX += X;
                ErrorXPrev = ErrorX;
                ErrorR = ErrorR * R;
                ErrorRPrev = ErrorR;
            }
            else
            {
                Reset();
            }
        }

        bool UpdateError(int32_t CurrentSimTick)
        {
            SimTicks = CurrentSimTick - LastSimTick;
            LastSimTick = CurrentSimTick;
            if (SimTicks > 0)
            {
                return DecayError();
            }
            return false;
        }

        bool DecayError()
        {
            if (SimTicks <= 0)
            {
                return false;
            }
            if (!IsErrorSmoothing())
            {
                Reset();
                return false;
            }
            const int32_t ErrorSmoothingCount = EndDecayTick - LastSimTick;
            int32_t Ticks = std::clamp(SimTicks, 0, ErrorSmoothingCount);
            for (; Ticks > 0; Ticks--)
            {
                float Alpha = static_cast<float>(ErrorSmoothingCount - 1) / static_cast<float>(ErrorSmoothingCount);
                Alpha = std::clamp(Alpha, 0.f, 1.f);
                ErrorXPrev = ErrorX;
                ErrorX *= static_cast<double>(Alpha);
                ErrorRPrev = ErrorR;
                ErrorR = Slerp(glm::dquat(1.0, 0.0, 0.0, 0.0), ErrorR, static_cast<double>(Alpha));
            }
            return true;
        }

        void Reset()
        {
            ErrorX = glm::dvec3(0.0);
            ErrorXPrev = glm::dvec3(0.0);
            ErrorR = glm::dquat(1.0, 0.0, 0.0, 0.0);
            ErrorRPrev = glm::dquat(1.0, 0.0, 0.0, 0.0);
            EndDecayTick = 0;
            LastSimTick = 0;
            SimTicks = 0;
        }

        static int32_t DurationTicks(const RenderCorrectionBody& Error, float ErrorCorrectionDuration, float MaxErrorCorrectionCm,
            float MaxErrorDesyncTime, float AsyncFixedTimeStep)
        {
            const double ErrorDistanceSqr = glm::dot(Error.errorCm, Error.errorCm);
            const double MaxErrorCorrection = static_cast<double>(MaxErrorCorrectionCm) * static_cast<double>(MaxErrorCorrectionCm);
            const double DesyncCm = static_cast<double>(Error.speedCmPerSecond) * static_cast<double>(MaxErrorDesyncTime);
            const bool VelocityWithinLimit = MaxErrorDesyncTime > 0.f && ErrorDistanceSqr < DesyncCm * DesyncCm;
            if (ErrorDistanceSqr < MaxErrorCorrection || VelocityWithinLimit)
            {
                return static_cast<int32_t>(std::floor(static_cast<double>(ErrorCorrectionDuration) / static_cast<double>(AsyncFixedTimeStep)));
            }
            return 0;
        }
    };

    bool sameBits(const glm::dvec3& first, const glm::dvec3& second)
    {
        return first.x == second.x && first.y == second.y && first.z == second.z;
    }

    bool sameBits(const glm::dquat& first, const glm::dquat& second)
    {
        return first.x == second.x && first.y == second.y && first.z == second.z && first.w == second.w;
    }

    struct FullBodyPart { PhysicsBodyState bodyState; };
    struct LinearBodyPart { LinearBodyState bodyState; };

    struct TwoBodyState
    {
        FullBodyPart full;
        LinearBodyPart linear;

        template <typename S>
        const S& get() const
        {
            if constexpr (std::is_same_v<S, FullBodyPart>)
                return full;
            else
                return linear;
        }
    };

    struct TwoBodyAllState
    {
        TwoBodyState state;
        const TwoBodyState& getState() const { return state; }
    };

    struct FullDeclaration
    {
        using StateType = FullBodyPart;
        static const PhysicsBodyState& bodyStateOf(const FullBodyPart& s) { return s.bodyState; }
    };

    struct LinearDeclaration
    {
        using StateType = LinearBodyPart;
        static const LinearBodyState& bodyStateOf(const LinearBodyPart& s) { return s.bodyState; }
    };

    struct TwoDeclarationComposite
    {
        FullDeclaration full;
        LinearDeclaration linear;

        template <typename F>
        void forEach(F&& f) const
        {
            f(full);
            f(linear);
        }
    };

    struct NoInput { int unused = 0; };

    struct TwoBodySimulatable
    {
        using InputType = NoInput;

        const TwoBodyAllState& getAllState() const { return allState; }
        const TwoDeclarationComposite& getPhysicsComposite() const { return composite; }

        TwoBodyAllState allState;
        TwoDeclarationComposite composite;
    };
}

TEST_CASE("RenderCorrection.Capture.ErrorIsShownMinusCorrectedWithTheRotationInTheCorrectedBodysFrame", "[RenderInterpolation]")
{
    const glm::quat shownRotation = glm::angleAxis(0.4f, glm::normalize(glm::vec3(0.f, 0.f, 1.f)));
    const glm::quat correctedRotation = glm::angleAxis(0.1f, glm::normalize(glm::vec3(0.f, 0.f, 1.f)));

    Snapshot shown;
    fillSnapshot(shown, 41u, { makeBody(1u, 0u, false, glm::vec3(100.f, 20.f, 112.f)),
        makeBody(1u, 1u, true, glm::vec3(5.f, 5.f, 5.f), shownRotation) });
    Snapshot corrected;
    fillSnapshot(corrected, 41u, { makeBody(1u, 0u, false, glm::vec3(97.f, 20.f, 112.5f)),
        makeBody(1u, 1u, true, glm::vec3(5.f, 5.f, 5.f), correctedRotation) });

    Correction correction;
    REQUIRE(captureRenderCorrection(shown, corrected, 42u, correction) == 2u);
    CHECK(correction.physicsStep == 42u);
    CHECK(correction.tick == corrected.tick);

    const RenderCorrectionBody& root = correction.bodies[0];
    CHECK(root.simulatableId == 1u);
    CHECK(root.declarationIndex == 0u);
    CHECK(root.hasRotation == 0u);
    CHECK(root.errorCm.x == Catch::Approx(3.0));
    CHECK(root.errorCm.y == Catch::Approx(0.0));
    CHECK(root.errorCm.z == Catch::Approx(-0.5));
    CHECK(sameBits(root.errorRotation, glm::dquat(1.0, 0.0, 0.0, 0.0)));

    const RenderCorrectionBody& turned = correction.bodies[1];
    CHECK(turned.declarationIndex == 1u);
    CHECK(turned.hasRotation == 1u);
    const glm::dquat restored = glm::dquat(correctedRotation) * turned.errorRotation;
    CHECK(std::abs(glm::dot(restored, glm::dquat(shownRotation))) == Catch::Approx(1.0).margin(1e-7));
    CHECK(turned.errorRotation.w >= 0.0);
    CHECK(2.0 * std::acos(std::min(1.0, turned.errorRotation.w)) == Catch::Approx(0.3).margin(1e-6));
}

TEST_CASE("RenderCorrection.Capture.ChaosThresholdsATenthOfACentimetrePerAxisOrTwoHundredthsOfAQuaternionComponent", "[RenderInterpolation]")
{
    const glm::vec3 base(10.f, 10.f, 10.f);
    Snapshot shown;
    fillSnapshot(shown, 7u, { makeBody(1u, 0u, false, base + glm::vec3(0.0625f, -0.0625f, 0.f)),
        makeBody(2u, 0u, false, base + glm::vec3(0.f, 0.f, 0.125f)),
        makeBody(3u, 0u, true, base, glm::angleAxis(0.039f, glm::vec3(1.f, 0.f, 0.f))),
        makeBody(4u, 0u, true, base, glm::angleAxis(0.041f, glm::vec3(1.f, 0.f, 0.f))) });
    Snapshot corrected;
    fillSnapshot(corrected, 7u, { makeBody(1u, 0u, false, base), makeBody(2u, 0u, false, base),
        makeBody(3u, 0u, true, base, kIdentity), makeBody(4u, 0u, true, base, kIdentity) });

    Correction correction;
    REQUIRE(captureRenderCorrection(shown, corrected, 8u, correction) == 2u);
    CHECK(correction.bodies[0].simulatableId == 2u);
    CHECK(correction.bodies[0].errorCm.z == Catch::Approx(0.125));
    CHECK(correction.bodies[1].simulatableId == 4u);
    CHECK(correction.bodies[1].errorRotation.x > kRenderCorrectionMinRotationError);
}

TEST_CASE("RenderCorrection.Capture.ABodyInOnlyOneOfTheTwoSnapshotsCarriesNoError", "[RenderInterpolation]")
{
    Snapshot shown;
    fillSnapshot(shown, 3u, { makeBody(1u, 0u, false, glm::vec3(50.f)), makeBody(2u, 0u, false, glm::vec3(50.f)) });
    Snapshot corrected;
    fillSnapshot(corrected, 3u, { makeBody(2u, 0u, false, glm::vec3(40.f)), makeBody(3u, 0u, false, glm::vec3(0.f)) });

    Correction correction;
    REQUIRE(captureRenderCorrection(shown, corrected, 4u, correction) == 1u);
    CHECK(correction.bodies[0].simulatableId == 2u);
    CHECK(correction.bodies[0].errorCm.x == Catch::Approx(10.0));
}

TEST_CASE("RenderCorrection.Speeds.RecordedFromTheStoragesLinearVelocityForTheCorrectedBodiesOnly", "[RenderInterpolation]")
{
    SimulationObjectStorage<TwoBodySimulatable> storage;
    TwoBodySimulatable first;
    first.allState.state.full.bodyState.linearVelocity = glm::vec3(300.f, 400.f, 0.f);
    first.allState.state.linear.bodyState.linearVelocity = glm::vec3(0.f, 0.f, 7.f);
    TwoBodySimulatable second;
    second.allState.state.full.bodyState.linearVelocity = glm::vec3(9.f, 0.f, 0.f);
    second.allState.state.linear.bodyState.linearVelocity = glm::vec3(0.f, 12.f, 0.f);
    storage.add(5u, std::move(first));
    storage.add(6u, std::move(second));

    Correction correction;
    correction.bodyCount = 2u;
    correction.bodies[0].simulatableId = 5u;
    correction.bodies[0].declarationIndex = 0u;
    correction.bodies[1].simulatableId = 6u;
    correction.bodies[1].declarationIndex = 1u;
    correction.bodies[2].simulatableId = 6u;
    correction.bodies[2].declarationIndex = 0u;
    correction.bodies[2].speedCmPerSecond = -1.f;

    recordRenderCorrectionSpeeds(correction, storage);
    CHECK(correction.bodies[0].speedCmPerSecond == Catch::Approx(500.f));
    CHECK(correction.bodies[1].speedCmPerSecond == Catch::Approx(12.f));
    CHECK(correction.bodies[2].speedCmPerSecond == -1.f);
}

TEST_CASE("RenderCorrection.Duration.FloorOfTheSecondsOverTheStepConvertedAsChaosDoes", "[RenderInterpolation]")
{
    CHECK(renderCorrectionDurationSteps(0.3f, 0.016667f) == 17);
    CHECK(renderCorrectionDurationSteps(0.3f, 1.f / 60.f) == 17);
    CHECK(renderCorrectionDurationSteps(0.3f, 0.01f) == 30);
    CHECK(renderCorrectionDurationSteps(0.f, 0.016667f) == 0);
    CHECK(renderCorrectionDurationSteps(0.3f, 0.f) == 0);
    CHECK(renderCorrectionDurationSteps(-0.1f, 0.016667f) == -6);
}

TEST_CASE("RenderCorrection.Decay.EveryStepMatchesChaossLinearDecayBitForBit", "[RenderInterpolation]")
{
    std::mt19937 random(5701u);
    std::uniform_real_distribution<double> errorComponent(-60.0, 60.0);
    std::uniform_real_distribution<float> unit(0.f, 1.f);
    std::uniform_int_distribution<int> advance(-1, 3);

    uint64_t compared = 0;
    uint64_t smoothingFrames = 0;
    uint64_t snaps = 0;
    for (int run = 0; run < 40; ++run)
    {
        RenderCorrectionParams params = defaultParams();
        params.correctionSeconds = run % 8 == 0 ? 0.1f : 0.3f;
        params.maxCorrectionCm = run % 5 == 0 ? 40.f : 250.f;
        params.desyncSeconds = run % 3 == 0 ? -1.f : 0.6f;

        Smoother smoother;
        CorrectionRing ring;
        ChaosErrorReference reference;
        uint64_t nextStep = 100u;
        for (int frameIndex = 0; frameIndex < 600; ++frameIndex)
        {
            const int step = advance(random);
            nextStep = static_cast<uint64_t>(static_cast<int64_t>(nextStep) + (step < 0 && unit(random) < 0.8f ? 0 : step));

            std::optional<RenderCorrectionBody> error;
            if (unit(random) < 0.06f)
            {
                const glm::dvec3 errorCm(errorComponent(random), errorComponent(random), 0.25 * errorComponent(random));
                const glm::dquat errorRotation = glm::normalize(glm::dquat(1.0, 0.02 * errorComponent(random) / 60.0,
                    0.1 * errorComponent(random) / 60.0, 0.05 * errorComponent(random) / 60.0));
                const Correction correction = oneBodyCorrection(nextStep, errorCm, errorRotation, 200.f * unit(random), 1u, 0u, true);
                REQUIRE(ring.tryPush(correction));
                error = correction.bodies[0];
            }

            const float alpha = unit(random);
            const RenderInterpolationFrame frame = frameAt(nextStep, static_cast<double>(alpha));
            const glm::quat poseRotation = glm::angleAxis(0.5f, glm::vec3(0.f, 1.f, 0.f));
            Snapshot poses = posesWithOneBody(nextStep, glm::vec3(1000.f, -200.f, 112.f), true, poseRotation);
            RenderCorrectionFrame summary;
            smoother.correctPoses(ring, frame, params, poses, summary);

            if (error.has_value())
            {
                const int32_t ticks = ChaosErrorReference::DurationTicks(*error, params.correctionSeconds, params.maxCorrectionCm,
                    params.desyncSeconds, params.stepSeconds);
                snaps += ticks == 0 ? 1u : 0u;
                reference.AccumlateErrorXR(error->errorCm, error->errorRotation, static_cast<int32_t>(nextStep), ticks);
            }
            reference.UpdateError(static_cast<int32_t>(nextStep));

            const Smoother::BodyError* const state = smoother.findError(1u, 0u);
            if (reference.IsErrorSmoothing())
            {
                ++smoothingFrames;
                REQUIRE(state != nullptr);
                REQUIRE(state->isSmoothing());
                REQUIRE(sameBits(state->errorCm, reference.ErrorX));
                REQUIRE(sameBits(state->errorCmPrev, reference.ErrorXPrev));
                REQUIRE(sameBits(state->errorRotation, reference.ErrorR));
                REQUIRE(sameBits(state->errorRotationPrev, reference.ErrorRPrev));
                REQUIRE(sameBits(smoother.appliedErrorCm(frame, 1u, 0u), reference.GetErrorX(alpha)));

                const glm::dvec3 expectedPosition = glm::dvec3(glm::vec3(1000.f, -200.f, 112.f)) + reference.GetErrorX(alpha);
                REQUIRE(poses.bodies[0].positionCm == glm::vec3(expectedPosition));
                const glm::quat expectedRotation = glm::quat(glm::dquat(poseRotation) * reference.GetErrorR(alpha));
                REQUIRE(poses.bodies[0].rotation == expectedRotation);
            }
            else
            {
                REQUIRE((state == nullptr || !state->isSmoothing()));
                REQUIRE(poses.bodies[0].positionCm == glm::vec3(1000.f, -200.f, 112.f));
                REQUIRE(poses.bodies[0].rotation == poseRotation);
            }
            ++compared;
        }
    }
    CHECK(compared == 24000u);
    CHECK(smoothingFrames > 8000u);
    CHECK(snaps > 10u);
}

TEST_CASE("RenderCorrection.Decay.LinearToZeroOverTheDurationMinusOneSteps", "[RenderInterpolation]")
{
    const RenderCorrectionParams params = defaultParams();
    REQUIRE(renderCorrectionDurationSteps(params.correctionSeconds, params.stepSeconds) == 17);

    Smoother smoother;
    CorrectionRing ring;
    REQUIRE(ring.tryPush(oneBodyCorrection(50u, glm::dvec3(16.0, 0.0, 0.0))));

    for (uint64_t step = 50u; step <= 67u; ++step)
    {
        Snapshot poses = posesWithOneBody(step);
        RenderCorrectionFrame summary;
        smoother.correctPoses(ring, frameAt(step, 1.0), params, poses, summary);
        const double expected = step <= 65u ? 16.0 * static_cast<double>(65u - step) / 16.0 : 0.0;
        CHECK(poses.bodies[0].positionCm.x == Catch::Approx(expected).margin(1e-4));
        if (step == 50u)
        {
            CHECK(summary.corrections == 1u);
            CHECK(summary.smoothedBodies == 1u);
            CHECK(summary.maxErrorCm == Catch::Approx(16.0));
        }
        if (step == 66u)
        {
            CHECK(summary.finished == 1u);
            CHECK(summary.finishedSteps == 16u);
            CHECK(summary.maxFinishedSteps == 16u);
            CHECK(summary.active == 0u);
            CHECK(smoother.findError(1u, 0u) == nullptr);
        }
        else if (step < 66u)
        {
            CHECK(summary.active == 1u);
        }
    }
}

TEST_CASE("RenderCorrection.Snap.AtOrAboveTheMaximumOrWithZeroSecondsTheWholeErrorIsDropped", "[RenderInterpolation]")
{
    RenderCorrectionParams params = defaultParams();
    params.desyncSeconds = 0.f;

    SECTION("an error exactly at the maximum snaps, one just under it smooths")
    {
        Smoother smoother;
        CorrectionRing ring;
        REQUIRE(ring.tryPush(oneBodyCorrection(10u, glm::dvec3(0.0, 250.0, 0.0))));
        Snapshot poses = posesWithOneBody(10u);
        RenderCorrectionFrame summary;
        smoother.correctPoses(ring, frameAt(10u, 0.0), params, poses, summary);
        CHECK(summary.snappedBodies == 1u);
        CHECK(summary.smoothedBodies == 0u);
        CHECK(poses.bodies[0].positionCm == glm::vec3(0.f));

        REQUIRE(ring.tryPush(oneBodyCorrection(11u, glm::dvec3(0.0, 249.99, 0.0))));
        poses = posesWithOneBody(11u);
        smoother.correctPoses(ring, frameAt(11u, 0.0), params, poses, summary);
        CHECK(summary.smoothedBodies == 1u);
        CHECK(poses.bodies[0].positionCm.y == Catch::Approx(249.99f));
    }

    SECTION("a snap also drops the error still decaying from an earlier correction")
    {
        Smoother smoother;
        CorrectionRing ring;
        REQUIRE(ring.tryPush(oneBodyCorrection(10u, glm::dvec3(30.0, 0.0, 0.0))));
        Snapshot poses = posesWithOneBody(10u);
        RenderCorrectionFrame summary;
        smoother.correctPoses(ring, frameAt(10u, 1.0), params, poses, summary);
        REQUIRE(poses.bodies[0].positionCm.x > 20.f);

        REQUIRE(ring.tryPush(oneBodyCorrection(12u, glm::dvec3(0.0, 0.0, 400.0))));
        poses = posesWithOneBody(12u);
        smoother.correctPoses(ring, frameAt(12u, 1.0), params, poses, summary);
        CHECK(summary.snappedBodies == 1u);
        CHECK(summary.active == 0u);
        CHECK(poses.bodies[0].positionCm == glm::vec3(0.f));
    }

    SECTION("zero seconds snaps every correction: the pop")
    {
        params.correctionSeconds = 0.f;
        Smoother smoother;
        CorrectionRing ring;
        REQUIRE(ring.tryPush(oneBodyCorrection(10u, glm::dvec3(0.5, 0.0, 0.0))));
        Snapshot poses = posesWithOneBody(10u);
        RenderCorrectionFrame summary;
        smoother.correctPoses(ring, frameAt(10u, 0.0), params, poses, summary);
        CHECK(summary.corrections == 1u);
        CHECK(summary.snappedBodies == 1u);
        CHECK(poses.bodies[0].positionCm == glm::vec3(0.f));
    }
}

TEST_CASE("RenderCorrection.Snap.TheDesyncTimeTimesTheSpeedKeepsAFastBodySmoothing", "[RenderInterpolation]")
{
    RenderCorrectionParams params = defaultParams();
    Smoother smoother;
    CorrectionRing ring;

    REQUIRE(ring.tryPush(oneBodyCorrection(20u, glm::dvec3(299.0, 0.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), 500.f)));
    Snapshot poses = posesWithOneBody(20u);
    RenderCorrectionFrame summary;
    smoother.correctPoses(ring, frameAt(20u, 0.0), params, poses, summary);
    CHECK(summary.smoothedBodies == 1u);

    smoother.reset();
    REQUIRE(ring.tryPush(oneBodyCorrection(21u, glm::dvec3(300.01, 0.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), 500.f)));
    poses = posesWithOneBody(21u);
    smoother.correctPoses(ring, frameAt(21u, 0.0), params, poses, summary);
    CHECK(summary.snappedBodies == 1u);

    params.desyncSeconds = 0.f;
    smoother.reset();
    REQUIRE(ring.tryPush(oneBodyCorrection(22u, glm::dvec3(299.0, 0.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), 500.f)));
    poses = posesWithOneBody(22u);
    smoother.correctPoses(ring, frameAt(22u, 0.0), params, poses, summary);
    CHECK(summary.snappedBodies == 1u);
}

TEST_CASE("RenderCorrection.Accumulate.ASecondCorrectionAddsToTheRemainingErrorAndRestartsTheWindow", "[RenderInterpolation]")
{
    const RenderCorrectionParams params = defaultParams();
    Smoother smoother;
    CorrectionRing ring;
    RenderCorrectionFrame summary;

    REQUIRE(ring.tryPush(oneBodyCorrection(30u, glm::dvec3(16.0, 0.0, 0.0))));
    for (uint64_t step = 30u; step <= 37u; ++step)
    {
        Snapshot poses = posesWithOneBody(step);
        smoother.correctPoses(ring, frameAt(step, 1.0), params, poses, summary);
    }
    const Smoother::BodyError* state = smoother.findError(1u, 0u);
    REQUIRE(state != nullptr);
    const double remaining = state->errorCm.x;
    CHECK(remaining == Catch::Approx(16.0 * 8.0 / 16.0).margin(1e-5));

    REQUIRE(ring.tryPush(oneBodyCorrection(38u, glm::dvec3(4.0, 0.0, 0.0))));
    Snapshot poses = posesWithOneBody(38u);
    smoother.correctPoses(ring, frameAt(38u, 0.0), params, poses, summary);
    state = smoother.findError(1u, 0u);
    REQUIRE(state != nullptr);
    CHECK(state->errorCmPrev.x == Catch::Approx(remaining + 4.0));
    CHECK(state->errorCm.x == Catch::Approx((remaining + 4.0) * 15.0 / 16.0));
    CHECK(state->endDecayStep == 38 - 1 + 17);
}

TEST_CASE("RenderCorrection.Straddle.TheRenderedPathStaysContinuousAcrossTheCorrectedStepAndMatchesChaossCorrectedPrev", "[RenderInterpolation]")
{
    const glm::vec3 errorCm(-30.f, 12.f, 0.f);
    const uint64_t correctedStep = 40u;
    auto oldTimeline = [](uint64_t step) { return glm::vec3(5.f * static_cast<float>(step), 0.f, 112.f); };
    auto newTimeline = [&](uint64_t step) { return oldTimeline(step) - errorCm; };

    auto renderRun = [&](float correctionSeconds, std::vector<glm::vec3>& rendered, std::vector<double>& maxChaosGap)
    {
        RenderCorrectionParams params = defaultParams();
        params.correctionSeconds = correctionSeconds;
        Channel channel;
        CorrectionRing ring;
        Smoother smoother;
        Snapshot poses;
        RenderInterpolationFrame frame;
        double chaosGap = 0.0;
        uint32_t straddleFrames = 0u;

        uint64_t published = 0u;
        for (int frameIndex = 0; frameIndex < 120; ++frameIndex)
        {
            const double hostSeconds = 0.5 + 0.01 * static_cast<double>(frameIndex);
            while (static_cast<double>(published + 1u) * kStepDeadlineSeconds <= hostSeconds)
            {
                ++published;
                if (published == correctedStep)
                {
                    Snapshot shown;
                    fillSnapshot(shown, published - 1u, { makeBody(1u, 0u, false, oldTimeline(published - 1u)) });
                    Snapshot corrected;
                    fillSnapshot(corrected, published - 1u, { makeBody(1u, 0u, false, newTimeline(published - 1u)) });
                    Correction correction;
                    REQUIRE(captureRenderCorrection(shown, corrected, published, correction) == 1u);
                    REQUIRE(ring.tryPush(correction));
                }
                Snapshot* const slot = channel.beginWrite();
                REQUIRE(slot != nullptr);
                fillSnapshot(*slot, published,
                    { makeBody(1u, 0u, false, published < correctedStep ? oldTimeline(published) : newTimeline(published)) });
                channel.commit();
            }

            RenderInterpolationParams interpolation;
            interpolation.renderTimeSeconds = hostSeconds - 2.0 * kStepDeadlineSeconds;
            REQUIRE(interpolateRenderPoses(channel, interpolation, poses, frame));
            RenderCorrectionFrame summary;
            smoother.correctPoses(ring, frame, params, poses, summary);
            rendered.push_back(poses.bodies[0].positionCm);

            if (correctionSeconds > 0.f && frame.hasPrev && frame.prevStep + 1u == correctedStep)
            {
                const float alpha = static_cast<float>(frame.alpha);
                const glm::vec3 chaosPrev = newTimeline(frame.prevStep);
                const glm::vec3 chaosPose = chaosPrev + (newTimeline(frame.nextStep) - chaosPrev) * alpha;
                const glm::vec3 chaosError = glm::vec3(errorCm) + (glm::vec3(errorCm) * (15.f / 16.f) - glm::vec3(errorCm)) * alpha;
                chaosGap = std::max(chaosGap, static_cast<double>(glm::distance(chaosPose + chaosError, poses.bodies[0].positionCm)));
                ++straddleFrames;
            }
            CHECK(summary.late == 0u);
        }
        CHECK((correctionSeconds == 0.f || straddleFrames >= 1u));
        maxChaosGap.push_back(chaosGap);
    };

    auto maxJumpAgainstMotion = [&](const std::vector<glm::vec3>& rendered)
    {
        double maxExtra = 0.0;
        for (size_t index = 1; index < rendered.size(); ++index)
        {
            const glm::vec3 motion = rendered[index] - rendered[index - 1];
            maxExtra = std::max(maxExtra, static_cast<double>(glm::length(motion - glm::vec3(3.f, 0.f, 0.f))));
        }
        return maxExtra;
    };

    std::vector<glm::vec3> smoothed;
    std::vector<glm::vec3> popped;
    std::vector<double> chaosGaps;
    renderRun(0.3f, smoothed, chaosGaps);
    renderRun(0.f, popped, chaosGaps);

    const double smoothedJump = maxJumpAgainstMotion(smoothed);
    const double poppedJump = maxJumpAgainstMotion(popped);
    INFO("smoothed " << smoothedJump << " popped " << poppedJump);
    CHECK(smoothedJump < 0.1 * static_cast<double>(glm::length(errorCm)));
    CHECK(poppedJump > 0.4 * static_cast<double>(glm::length(errorCm)));
    CHECK(chaosGaps[0] < 1e-3);
    CHECK(glm::distance(smoothed.back(), popped.back()) < 1e-3f);
}

TEST_CASE("RenderCorrection.Pending.ACorrectionWaitsForTheBracketToReachItsStepAndALateOneIsCounted", "[RenderInterpolation]")
{
    const RenderCorrectionParams params = defaultParams();
    Smoother smoother;
    CorrectionRing ring;
    RenderCorrectionFrame summary;

    REQUIRE(ring.tryPush(oneBodyCorrection(60u, glm::dvec3(8.0, 0.0, 0.0))));
    Snapshot poses = posesWithOneBody(58u);
    smoother.correctPoses(ring, frameAt(58u, 0.5, 57u), params, poses, summary);
    CHECK(summary.corrections == 0u);
    CHECK(smoother.pendingCount() == 1u);
    CHECK(poses.bodies[0].positionCm == glm::vec3(0.f));

    poses = posesWithOneBody(59u);
    smoother.correctPoses(ring, frameAt(59u, 0.5, 58u), params, poses, summary);
    CHECK(summary.corrections == 0u);

    poses = posesWithOneBody(60u);
    smoother.correctPoses(ring, frameAt(60u, 0.5, 59u), params, poses, summary);
    CHECK(summary.corrections == 1u);
    CHECK(summary.late == 0u);
    CHECK(smoother.pendingCount() == 0u);
    CHECK(poses.bodies[0].positionCm.x == Catch::Approx(0.5 * 8.0 * 15.0 / 16.0).margin(1e-5));

    REQUIRE(ring.tryPush(oneBodyCorrection(60u, glm::dvec3(1.0, 0.0, 0.0))));
    poses = posesWithOneBody(61u);
    smoother.correctPoses(ring, frameAt(61u, 0.5, 60u), params, poses, summary);
    CHECK(summary.late == 1u);
    CHECK(summary.corrections == 1u);
}

TEST_CASE("RenderCorrection.Pose.AtTheCorrectionTheRenderedPoseIsTheShownOneAndARotationlessBodyKeepsItsRotation", "[RenderInterpolation]")
{
    const glm::quat shownRotation = glm::angleAxis(0.7f, glm::normalize(glm::vec3(1.f, 2.f, 0.5f)));
    const glm::quat correctedRotation = glm::angleAxis(0.5f, glm::normalize(glm::vec3(1.f, 2.f, 0.4f)));
    const glm::vec3 shownPosition(400.f, -30.f, 120.f);
    const glm::vec3 correctedPosition(385.f, -26.f, 118.f);

    Snapshot shown;
    fillSnapshot(shown, 70u, { makeBody(1u, 0u, true, shownPosition, shownRotation), makeBody(1u, 1u, false, shownPosition) });
    Snapshot corrected;
    fillSnapshot(corrected, 70u, { makeBody(1u, 0u, true, correctedPosition, correctedRotation),
        makeBody(1u, 1u, false, correctedPosition, glm::angleAxis(1.f, glm::vec3(0.f, 0.f, 1.f))) });
    Correction correction;
    REQUIRE(captureRenderCorrection(shown, corrected, 71u, correction) == 2u);

    Smoother smoother;
    CorrectionRing ring;
    REQUIRE(ring.tryPush(correction));
    Snapshot poses = corrected;
    poses.physicsStep = 71u;
    RenderCorrectionFrame summary;
    smoother.correctPoses(ring, frameAt(71u, 0.0), defaultParams(), poses, summary);

    CHECK(glm::distance(poses.bodies[0].positionCm, shownPosition) < 1e-4f);
    CHECK(std::abs(glm::dot(poses.bodies[0].rotation, shownRotation)) == Catch::Approx(1.f).margin(1e-6f));
    CHECK(glm::distance(poses.bodies[1].positionCm, shownPosition) < 1e-4f);
    CHECK(poses.bodies[1].rotation == corrected.bodies[1].rotation);
}

TEST_CASE("RenderCorrection.Lifetime.AnErrorIsReleasedWhenItsBodyLeavesThePoseSetAndNoneIsCreatedForAnAbsentBody", "[RenderInterpolation]")
{
    const RenderCorrectionParams params = defaultParams();
    Smoother smoother;
    CorrectionRing ring;
    RenderCorrectionFrame summary;

    REQUIRE(ring.tryPush(oneBodyCorrection(80u, glm::dvec3(10.0, 0.0, 0.0))));
    REQUIRE(ring.tryPush(oneBodyCorrection(80u, glm::dvec3(10.0, 0.0, 0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), 0.f, 9u)));
    Snapshot poses = posesWithOneBody(80u);
    smoother.correctPoses(ring, frameAt(80u, 0.0), params, poses, summary);
    CHECK(smoother.findError(1u, 0u) != nullptr);
    CHECK(smoother.findError(9u, 0u) == nullptr);

    Snapshot empty;
    fillSnapshot(empty, 81u, {});
    smoother.correctPoses(ring, frameAt(81u, 0.0), params, empty, summary);
    CHECK(smoother.findError(1u, 0u) == nullptr);

    poses = posesWithOneBody(82u);
    smoother.correctPoses(ring, frameAt(82u, 0.0), params, poses, summary);
    CHECK(poses.bodies[0].positionCm == glm::vec3(0.f));
}

TEST_CASE("RenderCorrection.NoAllocation.TheCorrectionTheSmootherAndItsFrameAreFlatValues", "[RenderInterpolation]")
{
    STATIC_REQUIRE(std::is_trivially_copyable_v<Correction>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<RenderCorrectionBody>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<RenderCorrectionParams>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<RenderCorrectionFrame>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Smoother::BodyError>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Smoother>);
}

#endif
