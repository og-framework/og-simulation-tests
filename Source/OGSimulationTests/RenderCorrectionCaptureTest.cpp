// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/Mailbox.h"
#include "OGSimulation/PhysicsBodyState.h"
#include "OGSimulation/RenderCorrectionSmoothing.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StepHooks.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <random>
#include <type_traits>
#include <utility>

namespace
{
    constexpr size_t kBodies = 12;
    using Snapshot = RenderSnapshotT<kBodies>;
    using Correction = RenderCorrectionT<kBodies>;
    using Capture = RenderCorrectionCapture<kBodies>;
    using Outcome = RenderCorrectionCaptureOutcome;

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

    using Storage = SimulationObjectStorage<TwoBodySimulatable>;

    TwoBodySimulatable makeSimulatable(glm::vec3 fullPosition, glm::vec3 linearPosition, glm::vec3 fullVelocity = glm::vec3(0.f))
    {
        TwoBodySimulatable simulatable;
        simulatable.allState.state.full.bodyState.position = fullPosition;
        simulatable.allState.state.full.bodyState.linearVelocity = fullVelocity;
        simulatable.allState.state.linear.bodyState.position = linearPosition;
        return simulatable;
    }

    PhysicsBodyState& fullBody(Storage& storage, unsigned int id)
    {
        return storage.get<TwoBodySimulatable>(id).allState.state.full.bodyState;
    }

    void publish(Snapshot& snapshot, const Storage& storage, SimTick tick, uint64_t physicsStep)
    {
        TickOutcome outcome;
        outcome.tick = tick;
        outcome.physicsStep = physicsStep;
        fillRenderSnapshot(snapshot, storage, outcome, static_cast<double>(physicsStep) / 60.0);
    }

    Correction sentinelCorrection()
    {
        Correction correction;
        correction.physicsStep = 0xDEADu;
        correction.tick = 0xBEEFu;
        correction.bodyCount = 7u;
        return correction;
    }

    bool isSentinel(const Correction& correction)
    {
        return correction.physicsStep == 0xDEADu && correction.tick == 0xBEEFu && correction.bodyCount == 7u;
    }

    template <typename T>
    bool sameBytes(const T& first, const T& second)
    {
        static_assert(std::is_trivially_copyable_v<T>);
        return std::memcmp(&first, &second, sizeof(T)) == 0;
    }

    bool sameBody(const RenderCorrectionBody& first, const RenderCorrectionBody& second)
    {
        return first.simulatableId == second.simulatableId && first.declarationIndex == second.declarationIndex
            && first.hasRotation == second.hasRotation && sameBytes(first.speed, second.speed)
            && sameBytes(first.errorPosition.x, second.errorPosition.x) && sameBytes(first.errorPosition.y, second.errorPosition.y)
            && sameBytes(first.errorPosition.z, second.errorPosition.z) && sameBytes(first.errorRotation.x, second.errorRotation.x)
            && sameBytes(first.errorRotation.y, second.errorRotation.y) && sameBytes(first.errorRotation.z, second.errorRotation.z)
            && sameBytes(first.errorRotation.w, second.errorRotation.w);
    }

    bool sameCorrection(const Correction& first, const Correction& second)
    {
        if (first.physicsStep != second.physicsStep || first.tick != second.tick || first.bodyCount != second.bodyCount)
        {
            return false;
        }
        for (uint16_t index = 0; index < first.bodyCount; ++index)
        {
            if (!sameBody(first.bodies[index], second.bodies[index]))
            {
                return false;
            }
        }
        return true;
    }

    struct HostCounters
    {
        uint64_t resims = 0;
        uint64_t captured = 0;
        uint64_t withoutError = 0;
        uint64_t tickMismatches = 0;

        bool operator==(const HostCounters&) const = default;
    };

    using HostRing = SpscRing<Correction, 4>;

    struct LooseMemberHostReference
    {
        Snapshot shownHead;
        bool shownHeadValid = false;
        Snapshot correctedHead;
        Correction correction;
        uint64_t resimsSeen = 0u;
        HostCounters counters;
        HostRing ring;

        void published(const Snapshot& snapshot, bool savesTicks)
        {
            if (savesTicks)
            {
                shownHead = snapshot;
                shownHeadValid = true;
            }
        }

        Outcome capture(uint64_t grantedResims, uint32_t integratedTick, uint64_t physicsStep, const Storage& storage)
        {
            if (grantedResims == resimsSeen)
                return Outcome::NoNewResim;
            resimsSeen = grantedResims;
            ++counters.resims;
            if (!shownHeadValid)
                return Outcome::NoShownHead;

            TickOutcome head;
            head.tick = static_cast<SimTick>(integratedTick);
            head.physicsStep = physicsStep;
            fillRenderSnapshot(correctedHead, storage, head, 0.0);
            if (correctedHead.tick != shownHead.tick)
            {
                ++counters.tickMismatches;
                return Outcome::TickMismatch;
            }
            if (captureRenderCorrection(shownHead, correctedHead, physicsStep, correction) == 0u)
            {
                ++counters.withoutError;
                return Outcome::NoError;
            }
            recordRenderCorrectionSpeeds(correction, storage);
            if (ring.tryPush(correction))
                ++counters.captured;
            return Outcome::Captured;
        }
    };

    struct CaptureObjectHost
    {
        Capture capture_Step;
        HostCounters counters;
        HostRing ring;

        void published(const Snapshot& snapshot, bool savesTicks)
        {
            if (savesTicks)
                capture_Step.noteShownSnapshot(snapshot);
        }

        Outcome capture(uint64_t grantedResims, uint32_t integratedTick, uint64_t physicsStep, const Storage& storage)
        {
            Correction correction;
            const Outcome outcome = capture_Step.onStepAfterReplay(grantedResims, static_cast<SimTick>(integratedTick),
                physicsStep, storage, correction);
            switch (outcome)
            {
            case Outcome::NoNewResim:
                return outcome;
            case Outcome::NoShownHead:
                ++counters.resims;
                return outcome;
            case Outcome::TickMismatch:
                ++counters.resims;
                ++counters.tickMismatches;
                return outcome;
            case Outcome::NoError:
                ++counters.resims;
                ++counters.withoutError;
                return outcome;
            case Outcome::Captured:
                ++counters.resims;
                if (ring.tryPush(correction))
                    ++counters.captured;
                return outcome;
            }
            return outcome;
        }
    };

    glm::vec3 nudge(std::mt19937& rng, float scale)
    {
        std::uniform_real_distribution<float> unit(-1.f, 1.f);
        return glm::vec3(unit(rng), unit(rng), unit(rng)) * scale;
    }

    void correctStorage(std::mt19937& rng, Storage& storage, const std::array<bool, 5>& present)
    {
        std::uniform_int_distribution<int> kind(0, 5);
        std::uniform_real_distribution<float> angle(-0.5f, 0.5f);
        for (unsigned int id = 1u; id <= present.size(); ++id)
        {
            if (!present[id - 1u])
                continue;
            TwoBodySimulatable& simulatable = storage.get<TwoBodySimulatable>(id);
            PhysicsBodyState& full = simulatable.allState.state.full.bodyState;
            LinearBodyState& linear = simulatable.allState.state.linear.bodyState;
            switch (kind(rng))
            {
            case 0:
                break;
            case 1:
                full.position += nudge(rng, 0.0005f);
                linear.position += nudge(rng, 0.0005f);
                break;
            case 2:
                full.position += nudge(rng, 0.4f);
                full.linearVelocity = nudge(rng, 9.f);
                break;
            case 3:
                full.rotation = glm::normalize(full.rotation * glm::angleAxis(angle(rng), glm::vec3(-0.f, -1.f, 0.f)));
                break;
            case 4:
                full.rotation = glm::normalize(full.rotation * glm::angleAxis(0.01f, glm::vec3(-0.f, -0.f, 1.f)));
                break;
            default:
                linear.position += nudge(rng, 3.f);
                linear.linearVelocity = nudge(rng, 0.5f);
                break;
            }
        }
    }

    void advanceStorage(std::mt19937& rng, Storage& storage, const std::array<bool, 5>& present)
    {
        for (unsigned int id = 1u; id <= present.size(); ++id)
        {
            if (!present[id - 1u])
                continue;
            TwoBodySimulatable& simulatable = storage.get<TwoBodySimulatable>(id);
            simulatable.allState.state.full.bodyState.position += nudge(rng, 0.03f);
            simulatable.allState.state.linear.bodyState.position += nudge(rng, 0.03f);
        }
    }
}

TEST_CASE("RenderCorrection.CaptureObject.NoNewResimWhenTheGrantedCountHasNotMovedAndOutIsUntouched", "[RenderInterpolation]")
{
    Storage storage;
    storage.add(1u, makeSimulatable(glm::vec3(0.1f, 0.1f, -0.1f), glm::vec3(0.2f, 0.2f, -0.2f)));
    Snapshot published;
    publish(published, storage, 5u, 5u);

    Capture capture;
    capture.noteShownSnapshot(published);
    Correction out = sentinelCorrection();
    CHECK(capture.onStepAfterReplay(0u, 5u, 6u, storage, out) == Outcome::NoNewResim);
    CHECK(isSentinel(out));

    fullBody(storage, 1u).position = glm::vec3(0.9f, 0.9f, -0.9f);
    REQUIRE(capture.onStepAfterReplay(3u, 5u, 6u, storage, out) == Outcome::Captured);
    out = sentinelCorrection();
    CHECK(capture.onStepAfterReplay(3u, 5u, 7u, storage, out) == Outcome::NoNewResim);
    CHECK(isSentinel(out));
    CHECK(capture.resimsSeen() == 3u);
}

TEST_CASE("RenderCorrection.CaptureObject.NoShownHeadConsumesTheResimWithoutTouchingOut", "[RenderInterpolation]")
{
    Storage storage;
    storage.add(1u, makeSimulatable(glm::vec3(0.1f, 0.1f, -0.1f), glm::vec3(0.2f, 0.2f, -0.2f)));

    Capture capture;
    CHECK_FALSE(capture.hasShownSnapshot());
    Correction out = sentinelCorrection();
    CHECK(capture.onStepAfterReplay(1u, 0u, 1u, storage, out) == Outcome::NoShownHead);
    CHECK(isSentinel(out));
    CHECK(capture.resimsSeen() == 1u);
    CHECK(capture.onStepAfterReplay(1u, 0u, 2u, storage, out) == Outcome::NoNewResim);

    Snapshot published;
    publish(published, storage, 2u, 2u);
    capture.noteShownSnapshot(published);
    CHECK(capture.hasShownSnapshot());
    CHECK(capture.onStepAfterReplay(2u, 2u, 3u, storage, out) == Outcome::NoError);
}

TEST_CASE("RenderCorrection.CaptureObject.TickMismatchWhenTheIntegratedTickIsNotTheShownSnapshotsTick", "[RenderInterpolation]")
{
    Storage storage;
    storage.add(1u, makeSimulatable(glm::vec3(0.1f, 0.1f, -0.1f), glm::vec3(0.2f, 0.2f, -0.2f)));
    Snapshot published;
    publish(published, storage, 41u, 41u);

    Capture capture;
    capture.noteShownSnapshot(published);
    fullBody(storage, 1u).position = glm::vec3(0.8f, 0.8f, -0.8f);
    Correction out = sentinelCorrection();
    CHECK(capture.onStepAfterReplay(1u, 44u, 42u, storage, out) == Outcome::TickMismatch);
    CHECK(isSentinel(out));
    CHECK(capture.onStepAfterReplay(2u, 40u, 42u, storage, out) == Outcome::TickMismatch);
    CHECK(isSentinel(out));
    CHECK(capture.onStepAfterReplay(3u, 41u, 42u, storage, out) == Outcome::Captured);
}

TEST_CASE("RenderCorrection.CaptureObject.NoErrorWhenEveryBodyIsWithinTheThresholds", "[RenderInterpolation]")
{
    Storage storage;
    storage.add(1u, makeSimulatable(glm::vec3(0.1f, 0.1f, -0.1f), glm::vec3(0.2f, 0.2f, -0.2f)));
    storage.add(2u, makeSimulatable(glm::vec3(-0.1f, -0.1f, 0.1f), glm::vec3(-0.2f, -0.2f, 0.2f)));
    Snapshot published;
    publish(published, storage, 9u, 9u);

    Capture capture;
    capture.noteShownSnapshot(published);
    fullBody(storage, 1u).position += glm::vec3(0.f, -0.000625f, -0.000625f);
    fullBody(storage, 2u).linearVelocity = glm::vec3(0.f, 0.f, -50.f);
    Correction out = sentinelCorrection();
    CHECK(capture.onStepAfterReplay(1u, 9u, 10u, storage, out) == Outcome::NoError);
    CHECK(out.bodyCount == 0u);
    CHECK(out.physicsStep == 10u);
    CHECK(out.tick == 9u);
}

TEST_CASE("RenderCorrection.CaptureObject.CapturedIsShownMinusCorrectedAtTheIntegratedTickWithTheCorrectedSpeeds", "[RenderInterpolation]")
{
    Storage storage;
    storage.add(4u, makeSimulatable(glm::vec3(0.f, 0.f, -1.f), glm::vec3(0.f, 0.5f, -0.f)));
    storage.add(7u, makeSimulatable(glm::vec3(3.f, 0.f, -0.f), glm::vec3(0.01f, 0.01f, -0.01f)));
    Snapshot published;
    publish(published, storage, 120u, 130u);

    Capture capture;
    capture.noteShownSnapshot(published);
    fullBody(storage, 4u).position = glm::vec3(0.f, 0.f, -0.97f);
    fullBody(storage, 4u).linearVelocity = glm::vec3(4.f, 0.f, -3.f);
    storage.get<TwoBodySimulatable>(7u).allState.state.linear.bodyState.position = glm::vec3(0.01f, 0.11f, -0.01f);
    storage.get<TwoBodySimulatable>(7u).allState.state.linear.bodyState.linearVelocity = glm::vec3(0.12f, 0.f, -0.f);

    Correction out;
    REQUIRE(capture.onStepAfterReplay(1u, 120u, 131u, storage, out) == Outcome::Captured);
    CHECK(out.physicsStep == 131u);
    CHECK(out.tick == 120u);
    REQUIRE(out.bodyCount == 2u);
    CHECK(out.bodies[0].simulatableId == 4u);
    CHECK(out.bodies[0].declarationIndex == 0u);
    CHECK(out.bodies[0].errorPosition.z == Catch::Approx(-0.03));
    CHECK(out.bodies[0].speed == Catch::Approx(5.f));
    CHECK(out.bodies[1].simulatableId == 7u);
    CHECK(out.bodies[1].declarationIndex == 1u);
    CHECK(out.bodies[1].errorPosition.y == Catch::Approx(-0.1));
    CHECK(out.bodies[1].speed == Catch::Approx(0.12f));

    Correction direct;
    Snapshot corrected;
    TickOutcome head;
    head.tick = 120u;
    head.physicsStep = 131u;
    fillRenderSnapshot(corrected, storage, head, 0.0);
    REQUIRE(captureRenderCorrection(published, corrected, 131u, direct) == 2u);
    recordRenderCorrectionSpeeds(direct, storage);
    CHECK(sameCorrection(out, direct));
}

TEST_CASE("RenderCorrection.CaptureObject.ResetForgetsTheShownSnapshotAndTheResimCount", "[RenderInterpolation]")
{
    Storage storage;
    storage.add(1u, makeSimulatable(glm::vec3(0.1f, 0.1f, -0.1f), glm::vec3(0.2f, 0.2f, -0.2f)));
    Snapshot published;
    publish(published, storage, 3u, 3u);

    Capture capture;
    capture.noteShownSnapshot(published);
    fullBody(storage, 1u).position = glm::vec3(0.6f, 0.6f, -0.6f);
    Correction out;
    REQUIRE(capture.onStepAfterReplay(5u, 3u, 4u, storage, out) == Outcome::Captured);

    capture.reset();
    CHECK_FALSE(capture.hasShownSnapshot());
    CHECK(capture.resimsSeen() == 0u);
    out = sentinelCorrection();
    CHECK(capture.onStepAfterReplay(0u, 3u, 4u, storage, out) == Outcome::NoNewResim);
    CHECK(capture.onStepAfterReplay(5u, 3u, 4u, storage, out) == Outcome::NoShownHead);
    CHECK(isSentinel(out));
}

TEST_CASE("RenderCorrection.CaptureObject.NoAllocationTheCaptureIsAFlatValue", "[RenderInterpolation]")
{
    STATIC_REQUIRE(std::is_trivially_copyable_v<Capture>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<RenderCorrectionCapture<48>>);
    STATIC_REQUIRE(std::is_same_v<std::underlying_type_t<Outcome>, uint8_t>);
    STATIC_REQUIRE(sizeof(Capture) >= 2u * sizeof(Snapshot));
}

TEST_CASE("RenderCorrection.CaptureObject.MatchesTheLooseMemberHostLogicOverRandomPublishResimAndTickSequences", "[RenderInterpolation]")
{
    std::array<uint64_t, 5> outcomesSeen{};
    uint64_t poppedCompared = 0u;
    uint64_t ringDrops = 0u;
    uint64_t authorityRuns = 0u;

    for (uint32_t run = 0u; run < 120u; ++run)
    {
        std::mt19937 rng(0x68u + run);
        std::uniform_real_distribution<float> chance(0.f, 1.f);
        const bool savesTicks = run % 5u != 0u;
        authorityRuns += savesTicks ? 0u : 1u;

        Storage storage;
        std::array<bool, 5> present{};
        for (unsigned int id = 1u; id <= present.size(); ++id)
        {
            present[id - 1u] = id <= 3u;
            if (present[id - 1u])
                storage.add(id, makeSimulatable(nudge(rng, 5.f), nudge(rng, 5.f)));
        }

        LooseMemberHostReference reference;
        CaptureObjectHost host;
        Snapshot channelSlot;
        SimTick tick = 0u;
        uint64_t physicsStep = 1u;
        uint64_t grantedResims = 0u;
        const float publishChance = run % 3u == 0u ? 0.6f : 0.95f;

        for (uint32_t step = 0u; step < 400u; ++step)
        {
            if (chance(rng) < 0.3f)
            {
                grantedResims += chance(rng) < 0.85f ? 1u : 2u;
                if (chance(rng) < 0.1f)
                {
                    const unsigned int id = 1u + static_cast<unsigned int>(rng() % present.size());
                    if (present[id - 1u])
                        storage.remove<TwoBodySimulatable>(id);
                    else
                        storage.add(id, makeSimulatable(nudge(rng, 5.f), nudge(rng, 5.f)));
                    present[id - 1u] = !present[id - 1u];
                }
                correctStorage(rng, storage, present);
            }

            uint32_t integratedTick = tick;
            const float tickRoll = chance(rng);
            if (tickRoll < 0.04f)
                integratedTick = tick + 1u;
            else if (tickRoll < 0.07f)
                integratedTick = tick > 3u ? tick - 3u : tick + 3u;

            const Outcome expected = reference.capture(grantedResims, integratedTick, physicsStep, storage);
            const Outcome actual = host.capture(grantedResims, integratedTick, physicsStep, storage);
            REQUIRE(actual == expected);
            REQUIRE(host.counters == reference.counters);
            ++outcomesSeen[static_cast<size_t>(actual)];

            advanceStorage(rng, storage, present);
            ++tick;
            if (chance(rng) < publishChance)
            {
                publish(channelSlot, storage, tick, physicsStep);
                reference.published(channelSlot, savesTicks);
                host.published(channelSlot, savesTicks);
            }
            ++physicsStep;

            const uint32_t pops = chance(rng) < 0.5f ? 0u : static_cast<uint32_t>(rng() % 3u);
            for (uint32_t pop = 0u; pop < pops; ++pop)
            {
                std::optional<Correction> expectedPop = reference.ring.tryPop();
                std::optional<Correction> actualPop = host.ring.tryPop();
                REQUIRE(expectedPop.has_value() == actualPop.has_value());
                if (expectedPop.has_value())
                {
                    REQUIRE(sameCorrection(*actualPop, *expectedPop));
                    ++poppedCompared;
                }
            }
        }

        for (;;)
        {
            std::optional<Correction> expectedPop = reference.ring.tryPop();
            std::optional<Correction> actualPop = host.ring.tryPop();
            REQUIRE(expectedPop.has_value() == actualPop.has_value());
            if (!expectedPop.has_value())
                break;
            REQUIRE(sameCorrection(*actualPop, *expectedPop));
            ++poppedCompared;
        }
        REQUIRE(host.ring.drops() == reference.ring.drops());
        ringDrops += host.ring.drops();
    }

    CHECK(authorityRuns > 0u);
    CHECK(outcomesSeen[static_cast<size_t>(Outcome::NoNewResim)] > 1000u);
    CHECK(outcomesSeen[static_cast<size_t>(Outcome::NoShownHead)] > 50u);
    CHECK(outcomesSeen[static_cast<size_t>(Outcome::TickMismatch)] > 50u);
    CHECK(outcomesSeen[static_cast<size_t>(Outcome::NoError)] > 50u);
    CHECK(outcomesSeen[static_cast<size_t>(Outcome::Captured)] > 1000u);
    CHECK(poppedCompared > 1000u);
    CHECK(ringDrops > 0u);
}

#endif
