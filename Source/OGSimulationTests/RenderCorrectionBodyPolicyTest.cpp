// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/Mailbox.h"
#include "OGSimulation/PhysicsDeclaration.h"
#include "OGSimulation/RenderCorrectionSmoothing.h"
#include "OGSimulation/RenderInterpolation.h"
#include "OGSimulation/RenderSnapshot.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <vector>

namespace renderCorrectionBodyPolicyTest
{
    constexpr size_t kBodies = 8;
    using Snapshot = RenderSnapshotT<kBodies>;
    using Correction = RenderCorrectionT<kBodies>;
    using Smoother = RenderCorrectionSmoother<kBodies>;
    using Ring = SpscRing<Correction, 8>;

    constexpr float kStepSeconds = 0.016667f;
    constexpr uint8_t kSnapInRootFrame = static_cast<uint8_t>(RenderCorrectionMode::SnapInRootFrame);
    constexpr uint8_t kChild = 0u;
    constexpr uint8_t kFree = 1u;
    constexpr uint8_t kRoot = 2u;
    constexpr std::array<uint32_t, 2> kSimulatables{ 1u, 2u };
    const glm::quat kIdentity(1.f, 0.f, 0.f, 0.f);

    glm::vec3 fromForwardRightUp(const glm::vec3& forwardRightUp)
    {
        return glm::vec3(forwardRightUp.y, forwardRightUp.z, -forwardRightUp.x);
    }

    glm::dvec3 fromForwardRightUp(const glm::dvec3& forwardRightUp)
    {
        return glm::dvec3(forwardRightUp.y, forwardRightUp.z, -forwardRightUp.x);
    }

    RenderBody makeBody(uint32_t id, uint8_t declaration, glm::vec3 position, glm::quat rotation = kIdentity)
    {
        RenderBody body;
        body.simulatableId = id;
        body.declarationIndex = declaration;
        body.hasRotation = declaration == kRoot ? uint8_t{ 0 } : uint8_t{ 1 };
        body.renderCorrection = declaration == kChild ? kSnapInRootFrame : uint8_t{ 0 };
        body.isRoot = declaration == kRoot ? uint8_t{ 1 } : uint8_t{ 0 };
        body.position = position;
        body.rotation = declaration == kRoot ? kIdentity : rotation;
        return body;
    }

    void sortBodies(Snapshot& snapshot)
    {
        std::sort(snapshot.bodies.begin(), snapshot.bodies.begin() + snapshot.bodyCount, &renderSnapshotDetail::keyLess);
    }

    Snapshot posesOf(uint64_t step, std::initializer_list<RenderBody> bodies)
    {
        Snapshot poses;
        poses.physicsStep = step;
        for (const RenderBody& body : bodies)
        {
            poses.bodies[poses.bodyCount++] = body;
        }
        sortBodies(poses);
        return poses;
    }

    RenderCorrectionBody correctionBody(uint32_t id, uint8_t declaration, glm::dvec3 errorPosition,
        glm::dquat errorRotation = glm::dquat(1.0, 0.0, 0.0, 0.0), float speed = 0.f)
    {
        RenderCorrectionBody body;
        body.simulatableId = id;
        body.declarationIndex = declaration;
        body.hasRotation = declaration == kRoot ? uint8_t{ 0 } : uint8_t{ 1 };
        body.speed = speed;
        body.errorPosition = errorPosition;
        body.errorRotation = declaration == kRoot ? glm::dquat(1.0, 0.0, 0.0, 0.0) : errorRotation;
        return body;
    }

    Correction correctionOf(uint64_t step, std::initializer_list<RenderCorrectionBody> bodies)
    {
        Correction correction;
        correction.physicsStep = step;
        for (const RenderCorrectionBody& body : bodies)
        {
            correction.bodies[correction.bodyCount++] = body;
        }
        return correction;
    }

    RenderCorrectionParams paramsOf(float correctionSeconds, float maxCorrection, float desyncSeconds)
    {
        RenderCorrectionParams params;
        params.correctionSeconds = correctionSeconds;
        params.maxCorrection = maxCorrection;
        params.desyncSeconds = desyncSeconds;
        params.stepSeconds = kStepSeconds;
        return params;
    }

    const std::array<RenderCorrectionParams, 3> kParameterSets{
        paramsOf(0.3f, 2.5f, 0.6f),
        paramsOf(0.1f, 0.4f, -1.f),
        paramsOf(0.5f, 1.2f, 0.6f),
    };

    RenderInterpolationFrame frameAt(uint64_t nextStep, double alpha, std::optional<uint64_t> prevStep = std::nullopt)
    {
        RenderInterpolationFrame frame;
        frame.nextStep = nextStep;
        frame.alpha = alpha;
        frame.hasPrev = prevStep.has_value();
        frame.prevStep = prevStep.value_or(0u);
        return frame;
    }

    enum class Compare
    {
        Bytes,
        Values
    };

    template <typename T>
    bool same(const T& first, const T& second, Compare compare)
    {
        return compare == Compare::Bytes ? std::memcmp(&first, &second, sizeof(T)) == 0 : first == second;
    }

    bool samePose(const RenderBody& first, const RenderBody& second, Compare compare)
    {
        return same(first.position, second.position, compare) && same(first.rotation, second.rotation, compare);
    }

    template <typename ErrorA, typename ErrorB>
    bool sameError(const ErrorA* first, const ErrorB* second, Compare compare)
    {
        const auto sameValue = [compare](const auto& a, const auto& b) { return same(a, b, compare); };
        if (first == nullptr || second == nullptr)
        {
            return first == nullptr && second == nullptr;
        }
        return first->simulatableId == second->simulatableId && first->declarationIndex == second->declarationIndex
            && first->hasRotation == second->hasRotation && first->decaying == second->decaying
            && first->lastStep == second->lastStep && first->stepsSinceUpdate == second->stepsSinceUpdate
            && first->endDecayStep == second->endDecayStep && sameValue(first->errorPosition, second->errorPosition)
            && sameValue(first->errorPositionPrev, second->errorPositionPrev) && sameValue(first->errorRotation, second->errorRotation)
            && sameValue(first->errorRotationPrev, second->errorRotationPrev) && first->correctionStep == second->correctionStep
            && sameValue(first->errorPositionBeforeCorrection, second->errorPositionBeforeCorrection)
            && sameValue(first->errorRotationBeforeCorrection, second->errorRotationBeforeCorrection)
            && first->smoothingStartStep == second->smoothingStartStep;
    }

    bool sameSmoothingSummary(const RenderCorrectionFrame& first, const RenderCorrectionFrame& second)
    {
        return first.corrections == second.corrections && first.smoothedBodies == second.smoothedBodies
            && first.snappedBodies == second.snappedBodies && first.late == second.late && first.pendingDrops == second.pendingDrops
            && first.finished == second.finished && first.finishedSteps == second.finishedSteps
            && first.maxFinishedSteps == second.maxFinishedSteps && first.active == second.active
            && first.maxError == second.maxError && first.maxApplied == second.maxApplied;
    }

    template <size_t MaxBodies, size_t MaxPending = 8>
    class PrePolicySmoother
    {
    public:
        using CorrectionT = RenderCorrectionT<MaxBodies>;
        using SnapshotT = RenderSnapshotT<MaxBodies>;

        struct Error
        {
            uint32_t simulatableId = 0;
            uint8_t declarationIndex = 0;
            uint8_t hasRotation = 0;
            bool inUse = false;
            bool decaying = false;
            int64_t lastStep = 0;
            int64_t stepsSinceUpdate = 0;
            int64_t endDecayStep = 0;
            glm::dvec3 errorPosition{ 0.0 };
            glm::dvec3 errorPositionPrev{ 0.0 };
            glm::dquat errorRotation{ 1.0, 0.0, 0.0, 0.0 };
            glm::dquat errorRotationPrev{ 1.0, 0.0, 0.0, 0.0 };
            uint64_t correctionStep = 0;
            glm::dvec3 errorPositionBeforeCorrection{ 0.0 };
            glm::dquat errorRotationBeforeCorrection{ 1.0, 0.0, 0.0, 0.0 };
            int64_t smoothingStartStep = 0;

            bool smoothing() const { return lastStep < endDecayStep; }
        };

        template <typename RingT>
        void correctPoses(RingT& corrections, const RenderInterpolationFrame& frame, const RenderCorrectionParams& params,
            SnapshotT& poses, RenderCorrectionFrame& summary)
        {
            RenderCorrectionFrame result;
            while (auto correction = corrections.tryPop())
            {
                if (m_hasLastNextStep && correction->physicsStep <= m_lastNextStep)
                {
                    ++result.late;
                }
                if (m_pendingCount == MaxPending)
                {
                    ++result.pendingDrops;
                    continue;
                }
                m_pending[m_pendingCount++] = *correction;
            }

            for (Error& error : m_errors)
            {
                if (error.inUse && findRenderBody(poses, error.simulatableId, error.declarationIndex) == nullptr)
                {
                    error = Error{};
                }
            }

            const int64_t currentStep = static_cast<int64_t>(frame.nextStep);
            const int32_t durationSteps = renderCorrectionDurationSteps(params.correctionSeconds, params.stepSeconds);
            size_t kept = 0;
            for (size_t index = 0; index < m_pendingCount; ++index)
            {
                if (m_pending[index].physicsStep <= frame.nextStep)
                {
                    accumulate(m_pending[index], currentStep, durationSteps, params, poses, result);
                }
                else
                {
                    m_pending[kept++] = m_pending[index];
                }
            }
            m_pendingCount = kept;

            for (Error& error : m_errors)
            {
                if (error.inUse)
                {
                    update(error, currentStep, result);
                }
            }
            m_lastNextStep = frame.nextStep;
            m_hasLastNextStep = true;

            const double alpha = static_cast<double>(static_cast<float>(frame.alpha));
            for (const Error& error : m_errors)
            {
                if (!error.inUse || !error.smoothing())
                {
                    continue;
                }
                RenderBody* const body = const_cast<RenderBody*>(findRenderBody(static_cast<const SnapshotT&>(poses),
                    error.simulatableId, error.declarationIndex));
                if (body == nullptr)
                {
                    continue;
                }
                ++result.active;
                const bool straddles = frame.hasPrev && frame.prevStep < error.correctionStep;
                const glm::dvec3& fromPosition = straddles ? error.errorPositionBeforeCorrection : error.errorPositionPrev;
                const glm::dvec3 offset = fromPosition + alpha * (error.errorPosition - fromPosition);
                body->position = glm::vec3(glm::dvec3(body->position) + offset);
                if (body->hasRotation != 0u)
                {
                    const glm::dquat& fromRotation = straddles ? error.errorRotationBeforeCorrection : error.errorRotationPrev;
                    body->rotation = glm::quat(glm::dquat(body->rotation)
                        * renderCorrectionDetail::slerp(fromRotation, error.errorRotation, alpha));
                }
                result.maxApplied = std::max(result.maxApplied, glm::length(offset));
            }
            summary = result;
        }

        const Error* findError(uint32_t simulatableId, uint8_t declarationIndex) const
        {
            for (const Error& error : m_errors)
            {
                if (error.inUse && error.simulatableId == simulatableId && error.declarationIndex == declarationIndex)
                {
                    return &error;
                }
            }
            return nullptr;
        }

    private:
        void accumulate(const CorrectionT& correction, int64_t currentStep, int32_t durationSteps, const RenderCorrectionParams& params,
            const SnapshotT& poses, RenderCorrectionFrame& result)
        {
            ++result.corrections;
            const double maxCorrectionSquared = static_cast<double>(params.maxCorrection) * static_cast<double>(params.maxCorrection);
            for (uint16_t index = 0; index < correction.bodyCount; ++index)
            {
                const RenderCorrectionBody& body = correction.bodies[index];
                if (findRenderBody(poses, body.simulatableId, body.declarationIndex) == nullptr)
                {
                    continue;
                }
                Error* error = nullptr;
                Error* free = nullptr;
                for (Error& candidate : m_errors)
                {
                    if (candidate.inUse && candidate.simulatableId == body.simulatableId && candidate.declarationIndex == body.declarationIndex)
                    {
                        error = &candidate;
                        break;
                    }
                    if (!candidate.inUse && free == nullptr)
                    {
                        free = &candidate;
                    }
                }
                if (error == nullptr)
                {
                    REQUIRE(free != nullptr);
                    *free = Error{};
                    free->simulatableId = body.simulatableId;
                    free->declarationIndex = body.declarationIndex;
                    free->inUse = true;
                    error = free;
                }

                const double errorSquared = glm::dot(body.errorPosition, body.errorPosition);
                const double desyncDistance = static_cast<double>(body.speed) * static_cast<double>(params.desyncSeconds);
                const bool withinDesync = params.desyncSeconds > 0.f && errorSquared < desyncDistance * desyncDistance;
                const int32_t steps = errorSquared < maxCorrectionSquared || withinDesync ? durationSteps : 0;
                result.maxError = std::max(result.maxError, std::sqrt(errorSquared));

                const bool wasSmoothing = error->smoothing();
                error->errorPositionBeforeCorrection = wasSmoothing ? error->errorPosition : glm::dvec3(0.0);
                error->errorRotationBeforeCorrection = wasSmoothing ? error->errorRotation : glm::dquat(1.0, 0.0, 0.0, 0.0);
                error->correctionStep = correction.physicsStep;
                error->smoothingStartStep = currentStep;
                error->hasRotation = body.hasRotation;
                error->lastStep = currentStep - 1;
                error->endDecayStep = error->lastStep + steps;
                error->stepsSinceUpdate = 0;
                if (error->smoothing())
                {
                    error->errorPosition += body.errorPosition;
                    error->errorPositionPrev = error->errorPosition;
                    error->errorRotation = error->errorRotation * body.errorRotation;
                    error->errorRotationPrev = error->errorRotation;
                    error->decaying = true;
                    ++result.smoothedBodies;
                }
                else
                {
                    error->errorPosition = glm::dvec3(0.0);
                    error->errorPositionPrev = glm::dvec3(0.0);
                    error->errorRotation = glm::dquat(1.0, 0.0, 0.0, 0.0);
                    error->errorRotationPrev = glm::dquat(1.0, 0.0, 0.0, 0.0);
                    error->endDecayStep = 0;
                    error->lastStep = 0;
                    error->stepsSinceUpdate = 0;
                    error->decaying = false;
                    ++result.snappedBodies;
                }
            }
        }

        static void update(Error& error, int64_t currentStep, RenderCorrectionFrame& result)
        {
            error.stepsSinceUpdate = currentStep - error.lastStep;
            error.lastStep = currentStep;
            if (error.stepsSinceUpdate <= 0)
            {
                return;
            }
            if (!error.smoothing())
            {
                if (error.decaying)
                {
                    const int64_t steps = std::max<int64_t>(currentStep - error.smoothingStartStep, 0);
                    ++result.finished;
                    result.finishedSteps += static_cast<uint64_t>(steps);
                    result.maxFinishedSteps = std::max(result.maxFinishedSteps, static_cast<uint32_t>(steps));
                }
                error = Error{};
                return;
            }
            const int64_t smoothingCount = error.endDecayStep - error.lastStep;
            for (int64_t ticks = std::clamp<int64_t>(error.stepsSinceUpdate, 0, smoothingCount); ticks > 0; --ticks)
            {
                const float alpha = std::clamp(static_cast<float>(smoothingCount - 1) / static_cast<float>(smoothingCount), 0.f, 1.f);
                error.errorPositionPrev = error.errorPosition;
                error.errorPosition *= static_cast<double>(alpha);
                error.errorRotationPrev = error.errorRotation;
                error.errorRotation = renderCorrectionDetail::slerp(glm::dquat(1.0, 0.0, 0.0, 0.0), error.errorRotation,
                    static_cast<double>(alpha));
            }
        }

        std::array<Error, MaxBodies> m_errors{};
        std::array<CorrectionT, MaxPending> m_pending{};
        size_t m_pendingCount = 0;
        uint64_t m_lastNextStep = 0;
        bool m_hasLastNextStep = false;
    };

    struct FrameInput
    {
        RenderInterpolationFrame frame;
        Snapshot poses;
        Snapshot next;
        std::optional<Correction> correction;
        bool rootSnapGenerated = false;
    };

    class RandomDriver
    {
    public:
        explicit RandomDriver(uint32_t seed) : m_random(seed), m_nextRandom(seed + 7600u) {}

        FrameInput next(const RenderCorrectionParams& params)
        {
            FrameInput input;
            const int advance = m_advance(m_random);
            m_nextStep = static_cast<uint64_t>(static_cast<int64_t>(m_nextStep) + (advance < 0 && unit() < 0.8f ? 0 : advance));
            std::optional<uint64_t> prevStep;
            if (unit() < 0.5f)
            {
                prevStep = m_nextStep - (unit() < 0.8f ? 1u : 2u);
            }
            input.frame = frameAt(m_nextStep, static_cast<double>(unit()), prevStep);

            input.poses.physicsStep = m_nextStep;
            for (const uint32_t id : kSimulatables)
            {
                const glm::vec3 rootPosition = fromForwardRightUp(glm::vec3(position(15.f), position(15.f), position(3.f)));
                input.poses.bodies[input.poses.bodyCount++] = makeBody(id, kRoot, rootPosition);
                input.poses.bodies[input.poses.bodyCount++] = makeBody(id, kChild,
                    rootPosition + fromForwardRightUp(glm::vec3(position(0.6f), position(0.6f), 0.3f + position(0.1f))), rotation());
                input.poses.bodies[input.poses.bodyCount++] = makeBody(id, kFree,
                    fromForwardRightUp(glm::vec3(position(15.f), position(15.f), position(3.f))), rotation());
            }
            sortBodies(input.poses);
            input.next = input.poses;
            for (uint16_t index = 0; index < input.next.bodyCount; ++index)
            {
                RenderBody& body = input.next.bodies[index];
                body.position += fromForwardRightUp(glm::vec3(0.2f * nextSignedUnit(), 0.2f * nextSignedUnit(), 0.05f * nextSignedUnit()));
                if (body.hasRotation != 0u)
                {
                    body.rotation = glm::normalize(glm::quat(nextSignedUnit(), nextSignedUnit(), nextSignedUnit(), nextSignedUnit() + 0.01f));
                }
            }

            if (unit() < 0.1f)
            {
                Correction correction;
                correction.physicsStep = m_nextStep + static_cast<uint64_t>(unit() < 0.6f ? 0 : (unit() < 0.5f ? 1 : 2));
                for (const uint32_t id : kSimulatables)
                {
                    for (const uint8_t declaration : { kChild, kFree, kRoot })
                    {
                        if (unit() < 0.25f)
                        {
                            continue;
                        }
                        const bool rootSnap = declaration == kRoot && unit() < 0.15f;
                        const double scale = rootSnap ? 4.0 * static_cast<double>(params.maxCorrection) : 0.3;
                        glm::dvec3 errorPosition = fromForwardRightUp(glm::dvec3(scale * signedUnit(), scale * signedUnit(), 0.25 * scale * signedUnit()));
                        if (rootSnap)
                        {
                            errorPosition = glm::normalize(errorPosition)
                                * (1.5 * static_cast<double>(params.maxCorrection) + 0.1);
                            input.rootSnapGenerated = true;
                        }
                        const float speed = rootSnap ? 0.f : 2.f * unit();
                        correction.bodies[correction.bodyCount++] =
                            correctionBody(id, declaration, errorPosition, glm::dquat(errorRotation()), speed);
                    }
                }
                std::sort(correction.bodies.begin(), correction.bodies.begin() + correction.bodyCount,
                    [](const RenderCorrectionBody& first, const RenderCorrectionBody& second) {
                        return first.simulatableId != second.simulatableId ? first.simulatableId < second.simulatableId
                                                                           : first.declarationIndex < second.declarationIndex;
                    });
                input.correction = correction;
            }
            return input;
        }

    private:
        float unit() { return m_unit(m_random); }
        float nextSignedUnit() { return 2.f * m_nextUnit(m_nextRandom) - 1.f; }
        double signedUnit() { return 2.0 * static_cast<double>(unit()) - 1.0; }
        float position(float range) { return range * (2.f * unit() - 1.f); }

        glm::quat rotation()
        {
            return glm::normalize(glm::quat(2.f * unit() - 1.f, 2.f * unit() - 1.f, 2.f * unit() - 1.f, 2.f * unit() - 1.f + 0.01f));
        }

        glm::dquat errorRotation()
        {
            const double angle = std::numbers::pi * static_cast<double>(unit());
            const glm::dvec3 axis = glm::normalize(glm::dvec3(signedUnit(), signedUnit(), signedUnit() + 0.01));
            glm::dquat result = glm::angleAxis(angle, axis);
            return result.w < 0.0 ? -result : result;
        }

        std::mt19937 m_random;
        std::mt19937 m_nextRandom;
        std::uniform_real_distribution<float> m_unit{ 0.f, 1.f };
        std::uniform_real_distribution<float> m_nextUnit{ 0.f, 1.f };
        std::uniform_int_distribution<int> m_advance{ -1, 3 };
        uint64_t m_nextStep = 100u;
    };

    Snapshot withoutChildren(const Snapshot& poses)
    {
        Snapshot result = poses;
        result.bodyCount = 0;
        for (uint16_t index = 0; index < poses.bodyCount; ++index)
        {
            if (poses.bodies[index].declarationIndex != kChild)
            {
                result.bodies[result.bodyCount++] = poses.bodies[index];
            }
        }
        return result;
    }

    Correction withoutChildren(const Correction& correction)
    {
        Correction result = correction;
        result.bodyCount = 0;
        for (uint16_t index = 0; index < correction.bodyCount; ++index)
        {
            if (correction.bodies[index].declarationIndex != kChild)
            {
                result.bodies[result.bodyCount++] = correction.bodies[index];
            }
        }
        return result;
    }

    double rotationDegrees(const glm::dquat& rotation)
    {
        return 2.0 * std::acos(std::min(1.0, std::abs(rotation.w))) * 180.0 / std::numbers::pi;
    }

    class CarrierStepModel
    {
    public:
        void noteCorrection(const Correction& correction)
        {
            for (uint16_t index = 0; index < correction.bodyCount; ++index)
            {
                if (correction.bodies[index].declarationIndex == kChild)
                {
                    m_pending.push_back({ correction.bodies[index].simulatableId, correction.physicsStep });
                }
            }
        }

        void advance(const RenderInterpolationFrame& frame)
        {
            std::vector<Pending> kept;
            for (const Pending& pending : m_pending)
            {
                if (pending.step <= frame.nextStep)
                {
                    m_carrierStep[pending.simulatableId] = pending.step;
                }
                else
                {
                    kept.push_back(pending);
                }
            }
            m_pending = kept;
            for (std::optional<uint64_t>& step : m_carrierStep)
            {
                if (step.has_value() && (!frame.hasPrev || frame.prevStep >= *step))
                {
                    step.reset();
                }
            }
            m_frame = frame;
        }

        bool takesTheCarrierRotation(uint32_t simulatableId) const
        {
            const std::optional<uint64_t>& step = m_carrierStep[simulatableId];
            return step.has_value() && m_frame.hasPrev && m_frame.prevStep < *step && *step <= m_frame.nextStep;
        }

    private:
        struct Pending
        {
            uint32_t simulatableId = 0;
            uint64_t step = 0;
        };

        std::vector<Pending> m_pending;
        std::array<std::optional<uint64_t>, kSimulatables.back() + 1u> m_carrierStep{};
        RenderInterpolationFrame m_frame;
    };
} // namespace renderCorrectionBodyPolicyTest

using namespace renderCorrectionBodyPolicyTest;

TEST_CASE("RenderCorrection.BodyPolicy.ASnappedBodyCarriesNoErrorAndItsRootStillSmooths", "[RenderInterpolation]")
{
    const glm::vec3 rootPosition = glm::vec3(-0.3f, 1.2f, -4.f);
    const glm::vec3 childPosition = glm::vec3(-0.22f, 1.51f, -4.15f);
    const glm::quat childRotation = glm::angleAxis(0.4f, glm::normalize(glm::vec3(-0.1f, -1.f, 0.2f)));
    const glm::dvec3 rootError = glm::dvec3(-0.016, 0.0, -0.012);
    const glm::dvec3 childError = glm::dvec3(0.012, 0.016, 0.0);
    const glm::dquat childErrorRotation = glm::angleAxis(117.0 * std::numbers::pi / 180.0,
        glm::normalize(glm::dvec3(-1.0, -0.2, 0.3)));
    REQUIRE(glm::length(rootError) == Catch::Approx(0.02));
    REQUIRE(glm::length(childError) == Catch::Approx(0.02));

    const RenderCorrectionParams params = kParameterSets[0];
    const RenderInterpolationFrame frame = frameAt(71u, 0.0);

    Smoother smoother;
    Ring ring;
    REQUIRE(ring.tryPush(correctionOf(71u, { correctionBody(1u, kChild, childError, childErrorRotation), correctionBody(1u, kRoot, rootError) })));
    Snapshot poses = posesOf(71u, { makeBody(1u, kChild, childPosition, childRotation), makeBody(1u, kRoot, rootPosition) });
    RenderCorrectionFrame summary;
    smoother.correctPoses(ring, frame, params, poses, summary);

    Smoother alone;
    Ring aloneRing;
    REQUIRE(aloneRing.tryPush(correctionOf(71u, { correctionBody(1u, kRoot, rootError) })));
    Snapshot alonePoses = posesOf(71u, { makeBody(1u, kRoot, rootPosition) });
    RenderCorrectionFrame aloneSummary;
    alone.correctPoses(aloneRing, frame, params, alonePoses, aloneSummary);

    const RenderBody* const child = findRenderBody(poses, 1u, kChild);
    const RenderBody* const root = findRenderBody(poses, 1u, kRoot);
    REQUIRE(child != nullptr);
    REQUIRE(root != nullptr);

    CHECK(smoother.findError(1u, kChild) == nullptr);
    CHECK(child->rotation == childRotation);
    const glm::dvec3 rootOffset = smoother.appliedErrorPosition(frame, 1u, kRoot);
    REQUIRE(glm::length(rootOffset) > 0.019);
    CHECK(child->position == glm::vec3(glm::dvec3(childPosition) + rootOffset));
    CHECK(smoother.appliedErrorPosition(frame, 1u, kChild) == glm::dvec3(0.0));

    CHECK(summary.corrections == 1u);
    CHECK(summary.policySnaps == 1u);
    CHECK(summary.rootFramed == 1u);
    CHECK(summary.maxPolicySnap == Catch::Approx(0.02).margin(1.0000000000000001e-11));
    CHECK(summary.maxPolicySnapDeg == Catch::Approx(117.0).margin(0.1));
    CHECK(summary.smoothedBodies == 1u);

    REQUIRE(alone.findError(1u, kRoot) != nullptr);
    CHECK(sameError(smoother.findError(1u, kRoot), alone.findError(1u, kRoot), Compare::Bytes));
    CHECK(samePose(*root, alonePoses.bodies[0], Compare::Bytes));
    CHECK(sameSmoothingSummary(summary, aloneSummary));
}

TEST_CASE("RenderCorrection.BodyPolicy.TheSnappedBodyKeepsItsOffsetFromItsRootOnEveryFrame", "[RenderInterpolation]")
{
    uint64_t frames = 0;
    uint64_t rootFramedFrames = 0;
    uint64_t straddledRootFrames = 0;
    uint64_t rootSnaps = 0;
    uint64_t policySnaps = 0;
    uint64_t carrierRotationFrames = 0;
    double largestRootOffset = 0.0;
    for (int run = 0; run < 10; ++run)
    {
        const RenderCorrectionParams& params = kParameterSets[static_cast<size_t>(run) % kParameterSets.size()];
        RandomDriver driver(7400u + static_cast<uint32_t>(run));
        Smoother smoother;
        Ring ring;
        CarrierStepModel carriers;
        for (int frameIndex = 0; frameIndex < 500; ++frameIndex)
        {
            FrameInput input = driver.next(params);
            if (input.correction.has_value())
            {
                REQUIRE(ring.tryPush(*input.correction));
                carriers.noteCorrection(*input.correction);
                rootSnaps += input.rootSnapGenerated ? 1u : 0u;
            }
            Snapshot poses = input.poses;
            RenderCorrectionFrame summary;
            smoother.correctPoses(ring, input.frame, params, poses, summary, &input.next);
            carriers.advance(input.frame);
            policySnaps += summary.policySnaps;
            rootFramedFrames += summary.rootFramed > 0u ? 1u : 0u;
            REQUIRE(summary.pendingDrops == 0u);
            uint32_t expectedStraddleSnaps = 0;

            for (const uint32_t id : kSimulatables)
            {
                INFO("run " << run << " frame " << frameIndex << " simulatable " << id);
                const RenderBody* const suppliedChild = findRenderBody(input.poses, id, kChild);
                const RenderBody* const suppliedRoot = findRenderBody(input.poses, id, kRoot);
                const RenderBody* const nextChild = findRenderBody(input.next, id, kChild);
                const RenderBody* const child = findRenderBody(poses, id, kChild);
                const RenderBody* const root = findRenderBody(poses, id, kRoot);
                REQUIRE((suppliedChild != nullptr && suppliedRoot != nullptr && nextChild != nullptr && child != nullptr
                    && root != nullptr));

                const glm::dvec3 rendered = glm::dvec3(child->position) - glm::dvec3(root->position);
                const glm::dvec3 supplied = glm::dvec3(suppliedChild->position) - glm::dvec3(suppliedRoot->position);
                REQUIRE(glm::length(rendered - supplied) <= 1e-5);
                const bool carrierRotation = carriers.takesTheCarrierRotation(id);
                expectedStraddleSnaps += carrierRotation ? 1u : 0u;
                carrierRotationFrames += carrierRotation ? 1u : 0u;
                REQUIRE(child->rotation == (carrierRotation ? nextChild->rotation : suppliedChild->rotation));
                REQUIRE(smoother.findError(id, kChild) == nullptr);

                const double rootOffset = glm::length(glm::dvec3(root->position) - glm::dvec3(suppliedRoot->position));
                largestRootOffset = std::max(largestRootOffset, rootOffset);
                const Smoother::BodyError* const rootError = smoother.findError(id, kRoot);
                if (rootError != nullptr && rootError->isSmoothing() && input.frame.hasPrev && input.frame.prevStep < rootError->correctionStep
                    && rootOffset > 0.0001)
                {
                    ++straddledRootFrames;
                }
            }
            REQUIRE(summary.straddleSnaps == expectedStraddleSnaps);
            ++frames;
        }
    }
    CHECK(frames == 5000u);
    CHECK(rootFramedFrames > 1500u);
    CHECK(straddledRootFrames > 50u);
    CHECK(carrierRotationFrames > 50u);
    CHECK(rootSnaps > 10u);
    CHECK(policySnaps > 200u);
    CHECK(largestRootOffset > 0.2);
}

TEST_CASE("RenderCorrection.BodyPolicy.BodiesWithoutThePolicyAreBitExactWithAndWithoutASnappedNeighbour", "[RenderInterpolation]")
{
    uint64_t compared = 0;
    uint64_t smoothingComparisons = 0;
    uint64_t rootFramedFrames = 0;
    for (int run = 0; run < 10; ++run)
    {
        const RenderCorrectionParams& params = kParameterSets[static_cast<size_t>(run) % kParameterSets.size()];
        RandomDriver driver(7400u + static_cast<uint32_t>(run));
        Smoother withChildren;
        Ring withChildrenRing;
        Smoother withoutChildrenSmoother;
        Ring withoutChildrenRing;
        PrePolicySmoother<kBodies> reference;
        Ring referenceRing;
        for (int frameIndex = 0; frameIndex < 500; ++frameIndex)
        {
            const FrameInput input = driver.next(params);
            if (input.correction.has_value())
            {
                REQUIRE(withChildrenRing.tryPush(*input.correction));
                REQUIRE(withoutChildrenRing.tryPush(withoutChildren(*input.correction)));
                REQUIRE(referenceRing.tryPush(withoutChildren(*input.correction)));
            }
            Snapshot posesA = input.poses;
            Snapshot posesB = withoutChildren(input.poses);
            Snapshot posesReference = withoutChildren(input.poses);
            RenderCorrectionFrame summaryA;
            RenderCorrectionFrame summaryB;
            RenderCorrectionFrame summaryReference;
            const Snapshot nextB = withoutChildren(input.next);
            withChildren.correctPoses(withChildrenRing, input.frame, params, posesA, summaryA, &input.next);
            withoutChildrenSmoother.correctPoses(withoutChildrenRing, input.frame, params, posesB, summaryB, &nextB);
            reference.correctPoses(referenceRing, input.frame, params, posesReference, summaryReference);
            rootFramedFrames += summaryA.rootFramed > 0u ? 1u : 0u;

            INFO("run " << run << " frame " << frameIndex);
            REQUIRE(sameSmoothingSummary(summaryA, summaryB));
            REQUIRE(sameSmoothingSummary(summaryA, summaryReference));
            REQUIRE(summaryB.policySnaps == 0u);
            REQUIRE(summaryB.rootFramed == 0u);
            REQUIRE(summaryB.straddleSnaps == 0u);
            for (const uint32_t id : kSimulatables)
            {
                for (const uint8_t declaration : { kFree, kRoot })
                {
                    INFO("simulatable " << id << " declaration " << int(declaration));
                    const Smoother::BodyError* const errorA = withChildren.findError(id, declaration);
                    REQUIRE(sameError(errorA, withoutChildrenSmoother.findError(id, declaration), Compare::Bytes));
                    REQUIRE(sameError(errorA, reference.findError(id, declaration), Compare::Values));
                    smoothingComparisons += errorA != nullptr && errorA->isSmoothing() ? 1u : 0u;

                    const RenderBody* const bodyA = findRenderBody(posesA, id, declaration);
                    const RenderBody* const bodyB = findRenderBody(posesB, id, declaration);
                    const RenderBody* const bodyReference = findRenderBody(posesReference, id, declaration);
                    REQUIRE((bodyA != nullptr && bodyB != nullptr && bodyReference != nullptr));
                    REQUIRE(samePose(*bodyA, *bodyB, Compare::Bytes));
                    REQUIRE(samePose(*bodyA, *bodyReference, Compare::Values));
                    ++compared;
                }
            }
        }
    }
    CHECK(compared == 20000u);
    CHECK(smoothingComparisons > 5000u);
    CHECK(rootFramedFrames > 1500u);
}

TEST_CASE("RenderCorrection.BodyPolicy.OnTheStraddleASnappedBodyTakesTheCarrierPose", "[RenderInterpolation]")
{
    const RenderCorrectionParams params = kParameterSets[0];
    const glm::vec3 childOffset = glm::vec3(0.08f, 0.31f, -0.15f);
    const glm::vec3 rootError = glm::vec3(-0.016f, 0.f, -0.012f);
    const glm::quat oldRotation = glm::angleAxis(1.4f, glm::normalize(glm::vec3(-1.f, -0.2f, 0.f)));
    const auto rootAt = [](uint64_t step) { return glm::vec3(-0.3f, 1.2f, -(4.f + 0.02f * static_cast<float>(step - 70u))); };
    const auto swingAt = [](uint64_t step) {
        return glm::angleAxis(0.26f * static_cast<float>(step - 71u), glm::vec3(-0.f, -1.f, 0.f))
            * glm::angleAxis(0.3f, glm::vec3(-0.f, -0.f, 1.f));
    };
    const auto correctedAt = [&](uint64_t step) {
        return posesOf(step, { makeBody(1u, kChild, rootAt(step) + childOffset, swingAt(step)), makeBody(1u, kRoot, rootAt(step)) });
    };
    const Snapshot oldTimeline = posesOf(70u,
        { makeBody(1u, kChild, rootAt(70u) + rootError + childOffset, oldRotation), makeBody(1u, kRoot, rootAt(70u) + rootError) });

    Smoother smoother;
    Ring ring;
    REQUIRE(ring.tryPush(correctionOf(71u, { correctionBody(1u, kChild, glm::dvec3(0.012, 0.016, 0.0),
        glm::dquat(glm::conjugate(swingAt(71u)) * oldRotation)), correctionBody(1u, kRoot, glm::dvec3(rootError)) })));

    struct Bracket
    {
        uint64_t nextStep;
        double alpha;
        bool straddle;
    };
    const std::array<Bracket, 6> brackets{ {
        { 71u, 0.3, true },
        { 71u, 0.8, true },
        { 72u, 0.3, false },
        { 73u, 0.3, false },
        { 74u, 0.6, false },
        { 75u, 0.3, false },
    } };
    for (size_t index = 0; index < brackets.size(); ++index)
    {
        const Bracket& bracket = brackets[index];
        INFO("frame " << index << " next " << bracket.nextStep << " alpha " << bracket.alpha);
        const Snapshot prev = bracket.nextStep == 71u ? oldTimeline : correctedAt(bracket.nextStep - 1u);
        const Snapshot next = correctedAt(bracket.nextStep);
        const RenderInterpolationFrame frame = frameAt(bracket.nextStep, bracket.alpha, bracket.nextStep - 1u);
        Snapshot poses;
        REQUIRE(blendRenderPoses(prev, next, bracket.alpha, std::numeric_limits<float>::infinity(), poses) == 0u);
        const Snapshot supplied = poses;
        RenderCorrectionFrame summary;
        smoother.correctPoses(ring, frame, params, poses, summary, &next);

        const RenderBody* const suppliedChild = findRenderBody(supplied, 1u, kChild);
        const RenderBody* const suppliedRoot = findRenderBody(supplied, 1u, kRoot);
        const RenderBody* const nextChild = findRenderBody(next, 1u, kChild);
        const RenderBody* const child = findRenderBody(poses, 1u, kChild);
        const RenderBody* const root = findRenderBody(poses, 1u, kRoot);
        REQUIRE((suppliedChild != nullptr && suppliedRoot != nullptr && nextChild != nullptr && child != nullptr && root != nullptr));
        REQUIRE(rotationDegrees(glm::dquat(glm::conjugate(suppliedChild->rotation) * nextChild->rotation)) > 3.0);
        INFO("rendered to carrier rotation " << rotationDegrees(glm::dquat(glm::conjugate(child->rotation) * nextChild->rotation))
            << " deg, blend to carrier " << rotationDegrees(glm::dquat(glm::conjugate(suppliedChild->rotation) * nextChild->rotation)) << " deg");

        if (bracket.straddle)
        {
            CHECK(same(child->rotation, nextChild->rotation, Compare::Bytes));
            CHECK(summary.straddleSnaps == 1u);
        }
        else
        {
            CHECK(same(child->rotation, suppliedChild->rotation, Compare::Bytes));
            CHECK(summary.straddleSnaps == 0u);
        }
        CHECK(summary.policySnaps == (index == 0u ? 1u : 0u));
        CHECK(summary.rootFramed == 1u);

        const glm::dvec3 rootOffset = smoother.appliedErrorPosition(frame, 1u, kRoot);
        REQUIRE(glm::length(rootOffset) > 0.004);
        CHECK(child->position == glm::vec3(glm::dvec3(suppliedChild->position) + rootOffset));
        CHECK(glm::distance(child->position, nextChild->position) > 0.004f);
        const glm::dvec3 rendered = glm::dvec3(child->position) - glm::dvec3(root->position);
        const glm::dvec3 interpolated = glm::dvec3(suppliedChild->position) - glm::dvec3(suppliedRoot->position);
        CHECK(glm::length(rendered - interpolated) <= 1e-5);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
