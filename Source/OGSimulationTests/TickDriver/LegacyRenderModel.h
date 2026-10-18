// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "glm/geometric.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/mat3x3.hpp"
#include "glm/mat4x4.hpp"
#include "glm/vec3.hpp"
#include "OGSimulation/LatencyBudgetProbe.h"
#include "OGSimulation/Mailbox.h"
#include "OGSimulation/OGAssert.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/RenderCorrectionSmoothing.h"
#include "OGSimulation/RenderInterpolation.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StepHooks.h"
#include "OGSimulation/TickDriverLog.h"

namespace legacyRenderUe
{
    inline constexpr float kUeKindaSmallNumber = 1.e-4f;

    struct FMath
    {
        template <typename T>
        static constexpr T Max(T A, T B)
        {
            return (B < A) ? A : B;
        }

        template <typename T>
        static constexpr T Min(T A, T B)
        {
            return (A < B) ? A : B;
        }

        template <typename T>
        static constexpr T Clamp(const T X, const T MinValue, const T MaxValue)
        {
            return Max(Min(X, MaxValue), MinValue);
        }

        template <typename T>
        static constexpr T Abs(const T A)
        {
            return (A < (T)0) ? -A : A;
        }

        static double Atan2(double Y, double X)
        {
            if (X == 0.0 && Y == 0.0)
            {
                return 0.0;
            }
            return std::atan2(Y, X);
        }

        static double Sqrt(double Value) { return std::sqrt(Value); }

        template <typename T>
        static constexpr T Square(const T A)
        {
            return A * A;
        }
    };

    struct FVector
    {
        double X = 0.0;
        double Y = 0.0;
        double Z = 0.0;

        static FVector ZeroVector() { return FVector{}; }

        static double DistSquared(const FVector& V1, const FVector& V2)
        {
            return FMath::Square(V2.X - V1.X) + FMath::Square(V2.Y - V1.Y) + FMath::Square(V2.Z - V1.Z);
        }

        static double Dist(const FVector& V1, const FVector& V2) { return FMath::Sqrt(DistSquared(V1, V2)); }

        bool operator==(const FVector& V) const { return X == V.X && Y == V.Y && Z == V.Z; }

        bool Equals(const FVector& V, double Tolerance) const
        {
            return FMath::Abs(X - V.X) <= Tolerance && FMath::Abs(Y - V.Y) <= Tolerance && FMath::Abs(Z - V.Z) <= Tolerance;
        }
    };

    using FQuat = glm::dquat;

    enum class ETeleportType : uint8_t
    {
        None,
        TeleportPhysics,
        ResetPhysics
    };

    namespace uglm
    {
        inline FVector toFVector(const glm::vec3& value)
        {
            return FVector{ static_cast<double>(value.x), static_cast<double>(value.y), static_cast<double>(value.z) };
        }

        inline FQuat toFQuat(const glm::quat& value) { return FQuat(value); }
    } // namespace uglm

    template <typename CharacterT>
    struct TWeakObjectPtr
    {
        CharacterT* Get() const { return lookup != nullptr ? lookup(owner, id) : nullptr; }

        CharacterT* (*lookup)(void*, unsigned int) = nullptr;
        void* owner = nullptr;
        unsigned int id = 0u;
    };
} // namespace legacyRenderUe

struct LegacyRenderCvars
{
    float InterpolationDelaySteps = 2.f;
    int32_t NewestSnapshot = 0;
    float ErrorCorrectionSeconds = 0.3f;
    float ErrorCorrectionMaxCm = 250.f;
    float ErrorCorrectionDesyncSeconds = 0.6f;
    int32_t TraceFrames = 0;
};

template <typename Snapshot, typename EngineT, typename SourceT, typename ClockT, typename ActorsT, typename WorldT>
class LegacyRenderModel
{
public:
    using FMath = legacyRenderUe::FMath;
    using FVector = legacyRenderUe::FVector;
    using FQuat = legacyRenderUe::FQuat;
    using ETeleportType = legacyRenderUe::ETeleportType;
    using AOGBrawlerUECharacter = typename ActorsT::Character;
    using RenderSnapshot = Snapshot;
    using uint8 = uint8_t;
    using uint32 = uint32_t;
    using uint64 = uint64_t;
    using int32 = int32_t;

    static constexpr bool kNonShipping = static_cast<bool>(EngineT::kHostDiagnostics);

    LegacyRenderModel(EngineT& engine, SourceT& source, ClockT& frameHost, ActorsT& actors, WorldT& worlds,
        const LegacyRenderCvars& cvars, bool runsPrediction, float renderCorrectionStepSeconds, uint8 joltRootDeclarationIndex,
        float renderParkSnapCm)
        : m_engine(engine)
        , m_source(source)
        , m_frameHost(frameHost)
        , m_actors(actors)
        , m_worlds(worlds)
        , m_cvars(cvars)
        , m_runsPrediction(runsPrediction)
        , kRenderParkSnapCm(renderParkSnapCm)
    {
        m_joltRootDeclarationIndex = joltRootDeclarationIndex;
        m_renderCorrectionStepSeconds = renderCorrectionStepSeconds;
    }

    LegacyRenderModel(const LegacyRenderModel&) = delete;
    LegacyRenderModel& operator=(const LegacyRenderModel&) = delete;

    void onRegistered_GameThread(unsigned int id)
    {
        AOGBrawlerUECharacter* const character = m_actors.Cast(id);
        m_renderTargetsById[id] = RenderTargetUImpl{ m_actors.weakOf(character, id) };
    }

    void onUnregistered_GameThread(unsigned int id)
    {
        m_renderTargetsById.erase(id);
    }

    void beforePresent_GameThread()
    {
        syncRenderPoses_GameThread();
    }

    void updateVisualization_GameThread()
    {
        updateVisualizationAtRenderPoses_GameThread();
    }

    const Snapshot& renderPoses() const { return m_renderPoses; }
    bool renderPosesValid() const { return m_renderPosesValid; }
    const RenderInterpolationFrame& renderFrame() const { return m_renderFrame; }
    const RenderCorrectionFrame& renderCorrectionFrame() const { return m_renderCorrectionFrame; }

    void publishRenderSnapshot_Step(const TickOutcome& outcome, double stepDeadlineSeconds)
    {
        static_assert(std::is_trivially_copyable_v<RenderSnapshot>,
            "RenderSnapshot must stay a flat value with no heap-owning member: the J5 channel's four slots are built with "
            "the manager and filled in place on every step, which allocates nothing only while the snapshot owns no heap "
            "storage. Was task 55's 'no allocation on the publish path after warm-up'.");

        RenderSnapshot* const snapshot = m_renderSnapshots.beginWrite();
        OG_CHECK(snapshot != nullptr,
            "publishRenderSnapshot_Step: the render snapshot channel (J5) has no free slot. The game thread holds at "
            "most one snapshot (the newest, for one apply), and a 4-slot channel keeps a free slot for up to two.");
        if (snapshot == nullptr)
            return;

        fillRenderSnapshot(*snapshot, m_source.storage(), outcome, stepDeadlineSeconds);
        if (m_source.savesTicks())
            m_renderCorrectionCapture_Step.noteShownSnapshot(*snapshot);
        m_renderSnapshots.commit();
    }

    void captureRenderCorrection_Step(uint64 physicsStep)
    {
        RenderCorrection correction;
        const RenderCorrectionCaptureOutcome outcome = m_renderCorrectionCapture_Step.onStepAfterReplay(
            m_source.grantedResims(), static_cast<SimTick>(m_source.integratedTick()),
            physicsStep, m_source.storage(), correction);
        switch (outcome)
        {
        case RenderCorrectionCaptureOutcome::NoNewResim:
            return;
        case RenderCorrectionCaptureOutcome::NoShownHead:
            m_renderCorrectionResims.fetch_add(1u, std::memory_order_relaxed);
            return;
        case RenderCorrectionCaptureOutcome::TickMismatch:
            m_renderCorrectionResims.fetch_add(1u, std::memory_order_relaxed);
            m_renderCorrectionTickMismatches.fetch_add(1u, std::memory_order_relaxed);
            return;
        case RenderCorrectionCaptureOutcome::NoError:
            m_renderCorrectionResims.fetch_add(1u, std::memory_order_relaxed);
            m_renderCorrectionsWithoutError.fetch_add(1u, std::memory_order_relaxed);
            return;
        case RenderCorrectionCaptureOutcome::Captured:
            m_renderCorrectionResims.fetch_add(1u, std::memory_order_relaxed);
            if (m_renderCorrections.tryPush(correction))
                m_renderCorrectionsCaptured.fetch_add(1u, std::memory_order_relaxed);
            return;
        }
    }

private:
    static constexpr double kRenderMaxInterpolationDelaySteps = 2.0;
    static constexpr double kRenderApplyToleranceCm  = 1.0e-6;
    static constexpr double kShadowPoseToleranceCm   = 1.0e-3;
    static constexpr double kShadowPoseToleranceRad  = 1.0e-4;
    static constexpr uint32 kRenderApplyWindowFrames = 600u;

    static const char* stepKindText(StepKind kind)
    {
        switch (kind)
        {
        case StepKind::Normal:     return "Normal";
        case StepKind::Stall:      return "Stall";
        case StepKind::Skip:       return "Skip";
        case StepKind::HardResync: return "HardResync";
        }
        return "?";
    }

    struct RenderTargetUImpl
    {
        legacyRenderUe::TWeakObjectPtr<AOGBrawlerUECharacter> character;
        FVector                                               appliedCm = FVector::ZeroVector();
        bool                                                  applied   = false;
    };

    struct RenderApplyWindowUImpl
    {
        uint32 frames          = 0u;
        uint32 appliedFrames   = 0u;
        uint32 applied         = 0u;
        uint32 missing         = 0u;
        uint32 exact           = 0u;
        uint32 outOfTolerance  = 0u;
        uint32 held            = 0u;
        uint32 moved           = 0u;
        double maxErrorCm      = 0.0;
        double maxHeldDriftCm  = 0.0;
        double pathCm          = 0.0;
        uint64 firstAppliedStep = 0u;
        uint64 lastAppliedStep  = 0u;
        uint32 interpolated    = 0u;
        uint32 newestMode      = 0u;
        uint32 atNewest        = 0u;
        uint32 beyondOldest    = 0u;
        uint32 snaps           = 0u;
        uint32 maxNextBack     = 0u;
        uint32 shadowBodies    = 0u;
        uint32 shadowExact     = 0u;
        uint32 shadowOutOfTolerance = 0u;
        double shadowMaxErrorCm     = 0.0;
        double shadowMaxAngleRad    = 0.0;
        std::vector<float> frameDisplacementCm;
    };

    struct RenderCorrectionWindowUImpl
    {
        uint32 corrections       = 0u;
        uint32 smoothedBodies    = 0u;
        uint32 snappedBodies     = 0u;
        uint32 late              = 0u;
        uint32 pendingDrops      = 0u;
        uint32 finished          = 0u;
        uint64 finishedSteps     = 0u;
        uint32 maxFinishedSteps  = 0u;
        uint32 activeFrames      = 0u;
        double maxErrorCm        = 0.0;
        double maxAppliedCm      = 0.0;
        uint32 policySnaps       = 0u;
        uint32 rootFramedFrames  = 0u;
        double maxPolicySnapCm   = 0.0;
        double maxPolicySnapDeg  = 0.0;
        uint32 straddleSnaps     = 0u;
        uint64 resimsAtStart     = 0u;
        uint64 capturedAtStart   = 0u;
        uint64 withoutErrorAtStart  = 0u;
        uint64 tickMismatchAtStart  = 0u;
        uint64 ringDropsAtStart  = 0u;
    };

    using RenderCorrection         = RenderCorrectionT<RenderSnapshot::kMaxBodies>;
    using RenderCorrectionSmoothing = RenderCorrectionSmoother<RenderSnapshot::kMaxBodies>;
    using RenderCorrectionCapturing = RenderCorrectionCapture<RenderSnapshot::kMaxBodies>;

    void ueLog(HostLogLevel level, const char* format, ...)
    {
        if (!m_engine.logEnabled(level))
            return;
        va_list args;
        va_start(args, format);
        va_list sized;
        va_copy(sized, args);
        const int length = std::vsnprintf(nullptr, 0, format, sized);
        va_end(sized);
        std::string line(static_cast<size_t>(length < 0 ? 0 : length), '\0');
        std::vsnprintf(line.data(), line.size() + 1u, format, args);
        va_end(args);
        m_engine.log(level, line.c_str());
    }

    static std::string Printf(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        va_list sized;
        va_copy(sized, args);
        const int length = std::vsnprintf(nullptr, 0, format, sized);
        va_end(sized);
        std::string text(static_cast<size_t>(length < 0 ? 0 : length), '\0');
        std::vsnprintf(text.data(), text.size() + 1u, format, args);
        va_end(args);
        return text;
    }

    bool buildRenderPoses_GameThread()
    {
        using RenderChannel = decltype(m_renderSnapshots);
        static_assert(kRenderMaxInterpolationDelaySteps <= static_cast<double>(RenderChannel::kCapacity - 2),
            "og.Render.InterpolationDelaySteps is clamped to what the J5 channel can bracket: the step before the render time is "
            "at most floor(delay) + 1 snapshots behind the newest, so a delay of N - 2 steps is the most an N-slot channel "
            "reaches. Raise the channel's N with the clamp (task 19).");

        RenderInterpolationParams params;
        params.newestOnly = m_cvars.NewestSnapshot != 0;
        params.snapDistanceCm = kRenderParkSnapCm;
        const double delaySteps = FMath::Clamp(static_cast<double>(m_cvars.InterpolationDelaySteps),
            0.0, kRenderMaxInterpolationDelaySteps);
        params.renderTimeSeconds =
            m_frameHost.hostNowSeconds() - delaySteps * m_frameHost.stepIntervalSeconds();

        if (!interpolateRenderPoses(m_renderSnapshots, params, m_renderPoses, m_renderFrame, &m_renderNext))
            return false;

        RenderCorrectionParams correction;
        correction.correctionSeconds = m_cvars.ErrorCorrectionSeconds;
        correction.maxCorrectionCm   = m_cvars.ErrorCorrectionMaxCm;
        correction.desyncSeconds     = m_cvars.ErrorCorrectionDesyncSeconds;
        correction.stepSeconds       = m_renderCorrectionStepSeconds;
        m_renderCorrectionSmoother.correctPoses(m_renderCorrections, m_renderFrame, correction, m_renderPoses,
            m_renderCorrectionFrame, &m_renderNext);

        if constexpr (kNonShipping)
        {
        const RenderCorrectionFrame& corrected = m_renderCorrectionFrame;
        RenderCorrectionWindowUImpl& window = m_renderCorrectionWindow;
        window.corrections      += corrected.corrections;
        window.smoothedBodies   += corrected.smoothedBodies;
        window.snappedBodies    += corrected.snappedBodies;
        window.pendingDrops     += corrected.pendingDrops;
        window.finished         += corrected.finished;
        window.finishedSteps    += corrected.finishedSteps;
        window.maxFinishedSteps  = FMath::Max(window.maxFinishedSteps, corrected.maxFinishedSteps);
        window.activeFrames     += corrected.active > 0u ? 1u : 0u;
        window.maxErrorCm        = FMath::Max(window.maxErrorCm, corrected.maxErrorCm);
        window.maxAppliedCm      = FMath::Max(window.maxAppliedCm, corrected.maxAppliedCm);
        window.policySnaps      += corrected.policySnaps;
        window.rootFramedFrames += corrected.rootFramed > 0u ? 1u : 0u;
        window.maxPolicySnapCm   = FMath::Max(window.maxPolicySnapCm, corrected.maxPolicySnapCm);
        window.maxPolicySnapDeg  = FMath::Max(window.maxPolicySnapDeg, corrected.maxPolicySnapDeg);
        window.straddleSnaps    += corrected.straddleSnaps;
        if (corrected.late > 0u && window.late == 0u)
        {
            ueLog(HostLogLevel::Warning,
                "[SimHost.RenderCorrection] LATE frame=%llu next=%llu late=%u: a resimulation's correction reached the game "
                "thread after the corrected step it rides had already been drawn, so that step was drawn uncorrected (a pop)",
                static_cast<unsigned long long>(m_engine.frameNumber()), static_cast<unsigned long long>(m_renderFrame.nextStep),
                corrected.late);
        }
        window.late += corrected.late;
        if (corrected.corrections > 0u && m_cvars.TraceFrames != 0)
        {
            ueLog(HostLogLevel::Log,
                "[SimHost.RenderCorrection] event frame=%llu prev=%llu next=%llu alpha=%.4f corrections=%u smoothed=%u "
                "snapped=%u maxErrorCm=%.4f durationSteps=%d policySnaps=%u rootFramed=%u",
                static_cast<unsigned long long>(m_engine.frameNumber()), static_cast<unsigned long long>(m_renderFrame.prevStep),
                static_cast<unsigned long long>(m_renderFrame.nextStep), m_renderFrame.alpha, corrected.corrections,
                corrected.smoothedBodies, corrected.snappedBodies, corrected.maxErrorCm,
                renderCorrectionDurationSteps(correction.correctionSeconds, correction.stepSeconds), corrected.policySnaps,
                corrected.rootFramed);
        }
        }

        m_renderPosesValid = true;
        return true;
    }

    void applyRenderSnapshot_GameThread()
    {
        if constexpr (kNonShipping)
        {
        ++m_renderApplyWindow.frames;
        checkRenderTargetsHeld_GameThread();
        }

        if (buildRenderPoses_GameThread())
        {
            for (auto& [id, target] : m_renderTargetsById)
            {
                AOGBrawlerUECharacter* const character = target.character.Get();
                const RenderBody* const body = findRenderBody(m_renderPoses, id, m_joltRootDeclarationIndex);
                if (character == nullptr || body == nullptr)
                    continue;

                const FVector location = legacyRenderUe::uglm::toFVector(body->positionCm);
                const FQuat rotation = body->hasRotation != 0u ? legacyRenderUe::uglm::toFQuat(body->rotation) : character->GetActorQuat();
                [[maybe_unused]] FVector before;
                if constexpr (kNonShipping)
                {
                before = character->GetActorLocation();
                }
                character->SetActorLocationAndRotation(location, rotation, /*bSweep*/ false, /*OutSweepHitResult*/ nullptr,
                    ETeleportType::TeleportPhysics);

                if constexpr (kNonShipping)
                {
                if (target.applied)
                {
                    const double displacementCm = FVector::Dist(target.appliedCm, character->GetActorLocation());
                    m_renderApplyWindow.pathCm += displacementCm;
                    m_renderApplyWindow.frameDisplacementCm.push_back(static_cast<float>(displacementCm));
                }
                else
                {
                    ueLog(HostLogLevel::Log,
                        "[SimHost.RenderApply] first role=%s id=%u from=(%.3f,%.3f,%.3f) to=(%.3f,%.3f,%.3f) physicsStep=%llu tick=%u",
                        m_runsPrediction ? "Client" : "Authority", id, before.X, before.Y, before.Z,
                        location.X, location.Y, location.Z, static_cast<unsigned long long>(m_renderPoses.physicsStep),
                        static_cast<uint32>(m_renderPoses.tick));
                }
                }
                target.appliedCm = character->GetActorLocation();
                target.applied   = true;
            }

            if constexpr (kNonShipping)
            {
            checkRenderApply_GameThread();
            traceRenderFrame_GameThread();
            }
        }

        if constexpr (kNonShipping)
        {
        if (m_renderApplyWindow.frames >= kRenderApplyWindowFrames)
            logRenderApplyWindow_GameThread();
        }
    }

    void syncRenderPoses_GameThread()
    {
        if (m_worlds.hasShadowWorld())
            m_worlds.restoreShadowWorld_GameThread(m_frameHost);
        applyRenderSnapshot_GameThread();
        overwriteShadowPoses_GameThread();
    }

    void updateVisualizationAtRenderPoses_GameThread()
    {
        updateVisualizationAll(m_source.storage());
        if (!m_renderPosesValid)
            return;

        m_source.storage().forEachSimulatable([this](unsigned int id, auto& simulatable)
        {
            auto& vizState = simulatable.editVizState().editState();
            uint8 declarationIndex = 0;
            simulatable.getPhysicsComposite().forEach([&](const auto& declaration)
            {
                using D = std::decay_t<decltype(declaration)>;
                using S = typename D::StateType;
                const uint8 index = declarationIndex++;
                const RenderBody* const pose = findRenderBody(m_renderPoses, id, index);
                if (pose == nullptr || index == m_joltRootDeclarationIndex)
                    return;

                auto& bodyState = D::bodyStateOf(vizState.template edit<S>());
                bodyState.position = pose->positionCm;
                if constexpr (std::is_same_v<std::remove_cvref_t<decltype(bodyState)>, PhysicsBodyState>)
                    bodyState.rotation = pose->rotation;
            });
        });
    }

    void overwriteShadowPoses_GameThread()
    {
        if (!m_worlds.hasShadowWorld() || !m_renderPosesValid)
            return;
        OG_CHECK(m_worlds.shadowRestoredFrame() == m_engine.frameNumber(),
            "overwriteShadowPoses_GameThread: the shadow world was not restored this frame before the render poses were "
            "written into it. The restore loads the newest saved tick into every slot body, so a restore after this "
            "overwrite would leave the visualizations querying the newest tick instead of the rendered pose.");

        for (const auto& [id, target] : m_renderTargetsById)
        {
            const AOGBrawlerUECharacter* const character = target.character.Get();
            if (character == nullptr)
                continue;
            const std::optional<uint32> slot = m_worlds.slotByOwner(character);
            if (!slot.has_value())
                continue;

            for (uint32 declarationIndex = 0u; declarationIndex < m_worlds.shadowBodiesPerSlot(); ++declarationIndex)
            {
                const RenderBody* const pose = findRenderBody(m_renderPoses, id, static_cast<uint8>(declarationIndex));
                if (pose == nullptr)
                    continue;
                const auto body = m_worlds.shadowSlotBody(*slot, declarationIndex);
                if (!m_worlds.shadowIsBound(body))
                    continue;

                glm::mat4 transform = pose->hasRotation != 0u ? glm::mat4_cast(pose->rotation)
                                                              : m_worlds.shadowGetBodyTransform(body);
                transform[3] = glm::vec4(pose->positionCm, 1.f);
                m_worlds.shadowSetBodyTransform(body, transform);

                if constexpr (kNonShipping)
                {
                RenderApplyWindowUImpl& window = m_renderApplyWindow;
                const glm::mat4 queried = m_worlds.shadowReaderGetBodyTransform(body);
                const double errorCm = glm::distance(glm::vec3(queried[3]), pose->positionCm);
                double angleRad = 0.0;
                if (pose->hasRotation != 0u)
                {
                    const glm::dquat queriedRotation(glm::normalize(glm::quat_cast(glm::mat3(queried))));
                    const glm::dquat relative = glm::conjugate(glm::dquat(pose->rotation)) * queriedRotation;
                    angleRad = 2.0 * FMath::Atan2(glm::length(glm::dvec3(relative.x, relative.y, relative.z)), FMath::Abs(relative.w));
                }
                ++window.shadowBodies;
                window.shadowExact += errorCm == 0.0 ? 1u : 0u;
                window.shadowMaxErrorCm  = FMath::Max(window.shadowMaxErrorCm, errorCm);
                window.shadowMaxAngleRad = FMath::Max(window.shadowMaxAngleRad, angleRad);
                if ((errorCm > kShadowPoseToleranceCm || angleRad > kShadowPoseToleranceRad) && window.shadowOutOfTolerance++ == 0u)
                {
                    ueLog(HostLogLevel::Warning,
                        "[SimHost.RenderSync] SHADOW id=%u declaration=%u queried=(%.6f,%.6f,%.6f) rendered=(%.6f,%.6f,%.6f) "
                        "errorCm=%.9f angleRad=%.9f: the shadow world the visualizations query is not at the rendered pose",
                        id, declarationIndex, queried[3].x, queried[3].y, queried[3].z, pose->positionCm.x, pose->positionCm.y,
                        pose->positionCm.z, errorCm, angleRad);
                }
                }
            }
        }
    }

    void checkRenderTargetsHeld_GameThread()
    {
        for (const auto& [id, target] : m_renderTargetsById)
        {
            const AOGBrawlerUECharacter* const character = target.character.Get();
            if (!target.applied || character == nullptr)
                continue;

            const FVector planePoint = character->GetCapsuleComponent()->GetComponentTransform().GetTranslation();
            const double driftCm = FVector::Dist(planePoint, target.appliedCm);
            ++m_renderApplyWindow.held;
            m_renderApplyWindow.maxHeldDriftCm = FMath::Max(m_renderApplyWindow.maxHeldDriftCm, driftCm);
            if (driftCm > kRenderApplyToleranceCm && m_renderApplyWindow.moved++ == 0u)
            {
                ueLog(HostLogLevel::Warning,
                    "[SimHost.RenderApply] MOVED role=%s id=%u capsule=(%.6f,%.6f,%.6f) applied=(%.6f,%.6f,%.6f) driftCm=%.9f: "
                    "something moved the capsule between the last render apply and this one",
                    m_runsPrediction ? "Client" : "Authority", id, planePoint.X, planePoint.Y, planePoint.Z,
                    target.appliedCm.X, target.appliedCm.Y, target.appliedCm.Z, driftCm);
            }
        }
    }

    void checkRenderApply_GameThread()
    {
        RenderApplyWindowUImpl& window = m_renderApplyWindow;
        const RenderInterpolationFrame& bracket = m_renderFrame;
        if (window.appliedFrames++ == 0u)
            window.firstAppliedStep = m_renderPoses.physicsStep;
        window.lastAppliedStep = m_renderPoses.physicsStep;
        window.interpolated += bracket.hasPrev ? 1u : 0u;
        window.newestMode   += bracket.newestOnly ? 1u : 0u;
        window.atNewest     += bracket.atNewest ? 1u : 0u;
        window.beyondOldest += bracket.beyondOldest ? 1u : 0u;
        window.snaps        += bracket.snaps;
        window.maxNextBack   = FMath::Max(window.maxNextBack, bracket.nextBack);

        for (const auto& [id, target] : m_renderTargetsById)
        {
            const AOGBrawlerUECharacter* const character = target.character.Get();
            const RenderBody* const body = findRenderBody(m_renderPoses, id, m_joltRootDeclarationIndex);
            if (character == nullptr || body == nullptr)
            {
                ++window.missing;
                continue;
            }

            const FVector planePoint = character->GetCapsuleComponent()->GetComponentTransform().GetTranslation();
            const FVector pose = legacyRenderUe::uglm::toFVector(body->positionCm);
            const double errorCm = FVector::Dist(planePoint, pose);
            ++window.applied;
            window.exact += planePoint == pose ? 1u : 0u;
            window.maxErrorCm = FMath::Max(window.maxErrorCm, errorCm);
            if (!planePoint.Equals(pose, legacyRenderUe::kUeKindaSmallNumber) && window.outOfTolerance++ == 0u)
            {
                ueLog(HostLogLevel::Warning,
                    "[SimHost.RenderApply] MISMATCH role=%s id=%u capsule=(%.6f,%.6f,%.6f) rendered=(%.6f,%.6f,%.6f) errorCm=%.9f physicsStep=%llu",
                    m_runsPrediction ? "Client" : "Authority", id, planePoint.X, planePoint.Y, planePoint.Z,
                    pose.X, pose.Y, pose.Z, errorCm, static_cast<unsigned long long>(m_renderPoses.physicsStep));
            }
        }
    }

    void traceRenderFrame_GameThread()
    {
        if (m_cvars.TraceFrames == 0)
            return;

        const RenderInterpolationFrame& bracket = m_renderFrame;
        for (const auto& [id, target] : m_renderTargetsById)
        {
            const RenderBody* const nextBody = findRenderBody(m_renderNext, id, m_joltRootDeclarationIndex);
            if (!target.applied || nextBody == nullptr)
                continue;
            const glm::dvec3 errorCm = m_renderCorrectionSmoother.appliedErrorCm(bracket, id, m_joltRootDeclarationIndex);
            ueLog(HostLogLevel::Log,
                "[SimHost.RenderTrace] role=%s frame=%llu id=%u pos=(%.4f,%.4f,%.4f) nextPos=(%.4f,%.4f,%.4f) host=%.6f "
                "render=%.6f mode=%s prev=%llu/%s next=%llu/%s back=%u alpha=%.4f newest=%llu err=(%.4f,%.4f,%.4f)",
                m_runsPrediction ? "Client" : "Authority", static_cast<unsigned long long>(m_engine.frameNumber()), id,
                target.appliedCm.X, target.appliedCm.Y, target.appliedCm.Z,
                nextBody->positionCm.x, nextBody->positionCm.y, nextBody->positionCm.z,
                m_frameHost.hostNowSeconds(), bracket.renderTimeSeconds,
                bracket.newestOnly ? "newest" : "interp",
                static_cast<unsigned long long>(bracket.prevStep), bracket.hasPrev ? stepKindText(bracket.prevKind) : "-",
                static_cast<unsigned long long>(bracket.nextStep), stepKindText(bracket.nextKind), bracket.nextBack,
                bracket.alpha, static_cast<unsigned long long>(bracket.newestStep), errorCm.x, errorCm.y, errorCm.z);
        }
    }

    void logRenderApplyWindow_GameThread()
    {
        RenderApplyWindowUImpl& window = m_renderApplyWindow;
        std::string displacement = "-";
        if (!window.frameDisplacementCm.empty())
        {
            std::vector<float>& values = window.frameDisplacementCm;
            std::sort(values.begin(), values.end());
            const uint32 n = static_cast<uint32>(values.size());
            displacement = Printf("%.4f/%.4f/%.4f", values[latencyBudget::nearestRankIndex(n, 500u)],
                values[latencyBudget::nearestRankIndex(n, 990u)], values.back());
        }

        ueLog(HostLogLevel::Log,
            "[SimHost.RenderApply] role=%s frames=%u appliedFrames=%u characters=%d applied=%u missing=%u exact=%u "
            "outOfTolerance=%u maxErrorCm=%.9f held=%u moved=%u maxHeldDriftCm=%.9f pathCm=%.3f steps=[%llu,%llu] drops=%llu "
            "mode=%s delaySteps=%.2f interpolated=%u atNewest=%u beyondOldest=%u snaps=%u maxNextBack=%u dispCm=%s",
            m_runsPrediction ? "Client" : "Authority", window.frames, window.appliedFrames,
            static_cast<int32>(m_renderTargetsById.size()), window.applied, window.missing, window.exact,
            window.outOfTolerance, window.maxErrorCm, window.held, window.moved, window.maxHeldDriftCm, window.pathCm,
            static_cast<unsigned long long>(window.firstAppliedStep), static_cast<unsigned long long>(window.lastAppliedStep),
            static_cast<unsigned long long>(m_renderSnapshots.drops()),
            window.newestMode > 0u ? "newest" : "interp",
            FMath::Clamp(static_cast<double>(m_cvars.InterpolationDelaySteps), 0.0,
                kRenderMaxInterpolationDelaySteps),
            window.interpolated, window.atNewest, window.beyondOldest, window.snaps, window.maxNextBack, displacement.c_str());

        if (m_worlds.hasShadowWorld())
        {
            ueLog(HostLogLevel::Log,
                "[SimHost.RenderSync] role=Client frames=%u shadowBodies=%u shadowExact=%u shadowOutOfTolerance=%u "
                "shadowMaxErrorCm=%.6f shadowMaxAngleRad=%.6f",
                window.frames, window.shadowBodies, window.shadowExact, window.shadowOutOfTolerance, window.shadowMaxErrorCm,
                window.shadowMaxAngleRad);
        }

        if (m_runsPrediction)
            logRenderCorrectionWindow_GameThread();

        std::vector<float> samples = std::move(window.frameDisplacementCm);
        samples.clear();
        window = RenderApplyWindowUImpl{};
        window.frameDisplacementCm = std::move(samples);
    }

    void logRenderCorrectionWindow_GameThread()
    {
        RenderCorrectionWindowUImpl& window = m_renderCorrectionWindow;
        const uint64 resims       = m_renderCorrectionResims.load(std::memory_order_relaxed);
        const uint64 captured     = m_renderCorrectionsCaptured.load(std::memory_order_relaxed);
        const uint64 withoutError = m_renderCorrectionsWithoutError.load(std::memory_order_relaxed);
        const uint64 tickMismatch = m_renderCorrectionTickMismatches.load(std::memory_order_relaxed);
        const uint64 ringDrops    = m_renderCorrections.drops();
        const float  seconds      = m_cvars.ErrorCorrectionSeconds;
        const double stepSeconds  = static_cast<double>(m_renderCorrectionStepSeconds);
        const double decayMeanS   = window.finished > 0u
            ? static_cast<double>(window.finishedSteps) / static_cast<double>(window.finished) * stepSeconds : 0.0;

        ueLog(HostLogLevel::Log,
            "[SimHost.RenderCorrection] role=Client frames=%u resims=%llu captured=%llu withoutError=%llu tickMismatches=%llu "
            "ringDrops=%llu corrections=%u smoothedBodies=%u snappedBodies=%u maxErrorCm=%.3f maxAppliedCm=%.3f decays=%u "
            "decayMeanS=%.3f decayMaxS=%.3f activeFrames=%u late=%u pendingDrops=%u seconds=%.3f durationSteps=%d "
            "maxCm=%.1f desyncS=%.2f policySnaps=%u maxPolicySnapCm=%.3f maxPolicySnapDeg=%.1f rootFramedFrames=%u "
            "straddleSnaps=%u",
            m_renderApplyWindow.frames, static_cast<unsigned long long>(resims - window.resimsAtStart),
            static_cast<unsigned long long>(captured - window.capturedAtStart),
            static_cast<unsigned long long>(withoutError - window.withoutErrorAtStart),
            static_cast<unsigned long long>(tickMismatch - window.tickMismatchAtStart),
            static_cast<unsigned long long>(ringDrops - window.ringDropsAtStart), window.corrections, window.smoothedBodies,
            window.snappedBodies, window.maxErrorCm, window.maxAppliedCm, window.finished, decayMeanS,
            static_cast<double>(window.maxFinishedSteps) * stepSeconds, window.activeFrames, window.late, window.pendingDrops,
            seconds, renderCorrectionDurationSteps(seconds, m_renderCorrectionStepSeconds),
            m_cvars.ErrorCorrectionMaxCm, m_cvars.ErrorCorrectionDesyncSeconds,
            window.policySnaps, window.maxPolicySnapCm, window.maxPolicySnapDeg, window.rootFramedFrames, window.straddleSnaps);

        window = RenderCorrectionWindowUImpl{};
        window.resimsAtStart       = resims;
        window.capturedAtStart     = captured;
        window.withoutErrorAtStart = withoutError;
        window.tickMismatchAtStart = tickMismatch;
        window.ringDropsAtStart    = ringDrops;
    }

    EngineT& m_engine;
    SourceT& m_source;
    ClockT& m_frameHost;
    ActorsT& m_actors;
    WorldT& m_worlds;
    const LegacyRenderCvars& m_cvars;
    bool m_runsPrediction = false;
    const float kRenderParkSnapCm;

    SnapshotChannel<RenderSnapshot, 4>                   m_renderSnapshots;
    std::unordered_map<unsigned int, RenderTargetUImpl>  m_renderTargetsById;
    uint8                                                m_joltRootDeclarationIndex = 0;
    RenderSnapshot                                       m_renderPoses{};
    bool                                                 m_renderPosesValid = false;
    RenderInterpolationFrame                             m_renderFrame;
    RenderSnapshot                                       m_renderNext{};
    RenderApplyWindowUImpl                               m_renderApplyWindow;

    float                                                m_renderCorrectionStepSeconds = 0.f;
    RenderCorrectionCapturing                            m_renderCorrectionCapture_Step{};
    SpscRing<RenderCorrection, 16>                       m_renderCorrections;
    std::atomic<uint64>                                  m_renderCorrectionResims{0u};
    std::atomic<uint64>                                  m_renderCorrectionsCaptured{0u};
    std::atomic<uint64>                                  m_renderCorrectionsWithoutError{0u};
    std::atomic<uint64>                                  m_renderCorrectionTickMismatches{0u};
    RenderCorrectionSmoothing                            m_renderCorrectionSmoother;
    RenderCorrectionFrame                                m_renderCorrectionFrame;
    RenderCorrectionWindowUImpl                          m_renderCorrectionWindow;
};
