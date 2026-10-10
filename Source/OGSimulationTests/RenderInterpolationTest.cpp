// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/Mailbox.h"
#include "OGSimulation/RenderInterpolation.h"
#include "OGSimulation/RenderSnapshot.h"
#include "OGSimulation/SimulationTimeContext.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <type_traits>
#include <vector>

namespace
{
    using Snapshot = RenderSnapshotT<4>;
    using Channel = SnapshotChannel<Snapshot, 4>;

    constexpr double kStepSeconds = 1.0 / 60.0;

    RenderBody makeBody(uint32_t id, uint8_t declaration, bool hasRotation, glm::vec3 position,
        glm::quat rotation = glm::quat(1.f, 0.f, 0.f, 0.f))
    {
        RenderBody body;
        body.simulatableId = id;
        body.declarationIndex = declaration;
        body.hasRotation = hasRotation ? uint8_t{ 1 } : uint8_t{ 0 };
        body.positionCm = position;
        body.rotation = rotation;
        return body;
    }

    void fillSnapshot(Snapshot& snapshot, uint64_t step, double deadline, std::initializer_list<RenderBody> bodies,
        StepKind kind = StepKind::Normal)
    {
        snapshot = Snapshot{};
        snapshot.physicsStep = step;
        snapshot.stepDeadlineSeconds = deadline;
        snapshot.tick = static_cast<SimTick>(step);
        snapshot.kind = kind;
        for (const RenderBody& body : bodies)
        {
            snapshot.bodies[snapshot.bodyCount++] = body;
        }
        std::sort(snapshot.bodies.begin(), snapshot.bodies.begin() + snapshot.bodyCount, &renderSnapshotDetail::keyLess);
    }

    glm::vec3 positionAtStep(uint64_t step) { return glm::vec3(10.f * static_cast<float>(step), -3.f, 100.f + static_cast<float>(step)); }

    template <typename ChannelT>
    void publishStep(ChannelT& channel, uint64_t step, StepKind kind = StepKind::Normal)
    {
        Snapshot* slot = channel.beginWrite();
        REQUIRE(slot != nullptr);
        fillSnapshot(*slot, step, static_cast<double>(step) * kStepSeconds, { makeBody(1u, 0u, false, positionAtStep(step)) }, kind);
        channel.commit();
    }

    class CountingChannel
    {
    public:
        static constexpr size_t kCapacity = Channel::kCapacity;
        static constexpr size_t kMaxHeldKeepingNewest = Channel::kMaxHeldKeepingNewest;

        const Snapshot* peekNewest(size_t back = 0)
        {
            const Snapshot* snapshot = channel.peekNewest(back);
            if (snapshot != nullptr && std::find(held.begin(), held.end(), snapshot) == held.end())
            {
                held.push_back(snapshot);
                maxHeld = std::max(maxHeld, held.size());
            }
            ++peeks;
            if (commitAfterBackOne && back == 1u && snapshot != nullptr)
            {
                publishStep(channel, writerStep++);
            }
            return snapshot;
        }

        void release(const Snapshot* snapshot)
        {
            const auto found = std::find(held.begin(), held.end(), snapshot);
            if (found != held.end())
            {
                held.erase(found);
            }
            ++releases;
            channel.release(snapshot);
        }

        void resetCounts()
        {
            maxHeld = held.size();
            peeks = 0;
            releases = 0;
        }

        Channel channel;
        std::vector<const Snapshot*> held;
        size_t maxHeld = 0;
        size_t peeks = 0;
        size_t releases = 0;
        bool commitAfterBackOne = false;
        uint64_t writerStep = 0;
    };

    template <typename ChannelT>
    RenderInterpolationFrame interpolateAt(ChannelT& channel, double renderTimeSeconds, Snapshot& poses, bool newestOnly = false)
    {
        RenderInterpolationParams params;
        params.renderTimeSeconds = renderTimeSeconds;
        params.newestOnly = newestOnly;
        RenderInterpolationFrame frame;
        REQUIRE(interpolateRenderPoses(channel, params, poses, frame));
        return frame;
    }

    void requireNear(glm::vec3 actual, glm::vec3 expected)
    {
        INFO("actual (" << actual.x << ", " << actual.y << ", " << actual.z << ") expected (" << expected.x << ", " << expected.y
            << ", " << expected.z << ")");
        REQUIRE(glm::distance(actual, expected) < 1e-3f);
    }

    void requireNear(glm::quat actual, glm::quat expected)
    {
        REQUIRE(std::abs(glm::dot(actual, expected)) > 1.f - 1e-6f);
    }

    void requireBodyAt(const Snapshot& poses, glm::vec3 expected)
    {
        const RenderBody* body = findRenderBody(poses, 1u, 0u);
        REQUIRE(body != nullptr);
        requireNear(body->positionCm, expected);
    }
}

TEST_CASE("RenderInterpolation.Bracket.RenderTimeInsideThePublishedSteps", "[RenderInterpolation]")
{
    Channel channel;
    for (uint64_t step = 1; step <= 4; ++step)
    {
        publishStep(channel, step);
    }
    Snapshot poses;

    const double betweenTwoAndThree = 2.25 * kStepSeconds;
    RenderInterpolationFrame frame = interpolateAt(channel, betweenTwoAndThree, poses);
    REQUIRE(frame.hasPrev);
    REQUIRE(frame.prevStep == 2u);
    REQUIRE(frame.nextStep == 3u);
    REQUIRE(frame.newestStep == 4u);
    REQUIRE(frame.nextBack == 1u);
    REQUIRE_FALSE(frame.atNewest);
    REQUIRE_FALSE(frame.beyondOldest);
    REQUIRE_FALSE(frame.newestOnly);
    REQUIRE(frame.renderTimeSeconds == betweenTwoAndThree);
    REQUIRE(frame.alpha == Catch::Approx(0.25).margin(1e-9));
    REQUIRE(poses.physicsStep == 3u);
    REQUIRE(poses.bodyCount == 1u);
    const float alpha = static_cast<float>(frame.alpha);
    requireBodyAt(poses, positionAtStep(2) + (positionAtStep(3) - positionAtStep(2)) * alpha);

    const double betweenOneAndTwo = 1.5 * kStepSeconds;
    frame = interpolateAt(channel, betweenOneAndTwo, poses);
    REQUIRE(frame.prevStep == 1u);
    REQUIRE(frame.nextStep == 2u);
    REQUIRE(frame.nextBack == 2u);
    REQUIRE(frame.alpha == Catch::Approx(0.5).margin(1e-9));

    const double betweenThreeAndFour = 3.75 * kStepSeconds;
    frame = interpolateAt(channel, betweenThreeAndFour, poses);
    REQUIRE(frame.prevStep == 3u);
    REQUIRE(frame.nextStep == 4u);
    REQUIRE(frame.nextBack == 0u);
}

TEST_CASE("RenderInterpolation.Bracket.PastTheNewestRendersTheNewestWithoutExtrapolating", "[RenderInterpolation]")
{
    Channel channel;
    for (uint64_t step = 1; step <= 4; ++step)
    {
        publishStep(channel, step);
    }
    Snapshot poses;

    RenderInterpolationFrame frame = interpolateAt(channel, 4.5 * kStepSeconds, poses);
    REQUIRE(frame.atNewest);
    REQUIRE_FALSE(frame.hasPrev);
    REQUIRE_FALSE(frame.beyondOldest);
    REQUIRE(frame.nextStep == 4u);
    REQUIRE(frame.alpha == 1.0);
    requireBodyAt(poses, positionAtStep(4));

    frame = interpolateAt(channel, 4.0 * kStepSeconds, poses);
    REQUIRE_FALSE(frame.atNewest);
    REQUIRE_FALSE(frame.hasPrev);
    REQUIRE(frame.nextStep == 4u);
    requireBodyAt(poses, positionAtStep(4));
}

TEST_CASE("RenderInterpolation.Bracket.BeforeTheOldestRendersTheOldestReachable", "[RenderInterpolation]")
{
    Channel channel;
    for (uint64_t step = 1; step <= 6; ++step)
    {
        publishStep(channel, step);
    }
    Snapshot poses;

    const RenderInterpolationFrame frame = interpolateAt(channel, 0.5 * kStepSeconds, poses);
    REQUIRE(frame.beyondOldest);
    REQUIRE_FALSE(frame.hasPrev);
    REQUIRE_FALSE(frame.atNewest);
    REQUIRE(frame.nextStep == 3u);
    REQUIRE(frame.nextBack == 3u);
    REQUIRE(frame.newestStep == 6u);
    REQUIRE(frame.alpha == 1.0);
    requireBodyAt(poses, positionAtStep(3));
}

TEST_CASE("RenderInterpolation.Bracket.EqualDeadlinesTakeTheOldestAtOrAfterTheRenderTime", "[RenderInterpolation]")
{
    Channel channel;
    for (uint64_t step = 1; step <= 4; ++step)
    {
        Snapshot* slot = channel.beginWrite();
        REQUIRE(slot != nullptr);
        const double deadline = (step == 3u ? 2.0 : static_cast<double>(step)) * kStepSeconds;
        fillSnapshot(*slot, step, deadline, { makeBody(1u, 0u, false, positionAtStep(step)) });
        channel.commit();
    }
    Snapshot poses;

    const RenderInterpolationFrame frame = interpolateAt(channel, 2.0 * kStepSeconds, poses);
    REQUIRE(frame.hasPrev);
    REQUIRE(frame.prevStep == 1u);
    REQUIRE(frame.nextStep == 2u);
    REQUIRE(frame.nextBack == 2u);
    REQUIRE(frame.alpha == 1.0);
    requireBodyAt(poses, positionAtStep(2));
}

TEST_CASE("RenderInterpolation.Bracket.ASingleSnapshotAndAnEmptyChannel", "[RenderInterpolation]")
{
    Channel channel;
    Snapshot poses;
    poses.physicsStep = 77u;
    RenderInterpolationFrame frame;
    frame.nextStep = 55u;
    RenderInterpolationParams params;
    REQUIRE_FALSE(interpolateRenderPoses(channel, params, poses, frame));
    REQUIRE(poses.physicsStep == 77u);
    REQUIRE(frame.nextStep == 55u);

    publishStep(channel, 9u);
    frame = interpolateAt(channel, 8.0 * kStepSeconds, poses);
    REQUIRE(frame.beyondOldest);
    REQUIRE_FALSE(frame.hasPrev);
    REQUIRE(frame.nextStep == 9u);
    requireBodyAt(poses, positionAtStep(9));

    frame = interpolateAt(channel, 10.0 * kStepSeconds, poses);
    REQUIRE(frame.atNewest);
    REQUIRE_FALSE(frame.beyondOldest);
    REQUIRE(frame.nextStep == 9u);
    requireBodyAt(poses, positionAtStep(9));
}

TEST_CASE("RenderInterpolation.Bracket.NewestOnlyIgnoresTheRenderTime", "[RenderInterpolation]")
{
    Channel channel;
    for (uint64_t step = 1; step <= 4; ++step)
    {
        publishStep(channel, step, step == 4u ? StepKind::Skip : StepKind::Normal);
    }
    Snapshot poses;

    for (const double renderTime : { 0.5 * kStepSeconds, 2.5 * kStepSeconds, 9.0 * kStepSeconds })
    {
        const RenderInterpolationFrame frame = interpolateAt(channel, renderTime, poses, true);
        REQUIRE(frame.newestOnly);
        REQUIRE_FALSE(frame.hasPrev);
        REQUIRE_FALSE(frame.atNewest);
        REQUIRE_FALSE(frame.beyondOldest);
        REQUIRE(frame.nextStep == 4u);
        REQUIRE(frame.nextKind == StepKind::Skip);
        REQUIRE(frame.nextBack == 0u);
        requireBodyAt(poses, positionAtStep(4));
    }
}

TEST_CASE("RenderInterpolation.Alpha.FromTheTwoDeadlinesClampedToTheBracket", "[RenderInterpolation]")
{
    REQUIRE(renderInterpolationAlpha(1.0, 2.0, 1.25) == 0.25);
    REQUIRE(renderInterpolationAlpha(1.0, 2.0, 1.0) == 0.0);
    REQUIRE(renderInterpolationAlpha(1.0, 2.0, 2.0) == 1.0);
    REQUIRE(renderInterpolationAlpha(1.0, 2.0, 0.5) == 0.0);
    REQUIRE(renderInterpolationAlpha(1.0, 2.0, 3.0) == 1.0);
    REQUIRE(renderInterpolationAlpha(2.0, 2.0, 2.0) == 1.0);
    REQUIRE(renderInterpolationAlpha(2.0, 1.0, 1.5) == 1.0);

    Channel channel;
    Snapshot* slot = channel.beginWrite();
    fillSnapshot(*slot, 10u, 0.100, { makeBody(1u, 0u, false, glm::vec3(0.f)) }, StepKind::Stall);
    channel.commit();
    slot = channel.beginWrite();
    fillSnapshot(*slot, 11u, 0.130, { makeBody(1u, 0u, false, glm::vec3(30.f, 0.f, 0.f)) }, StepKind::Skip);
    channel.commit();
    Snapshot poses;
    const RenderInterpolationFrame frame = interpolateAt(channel, 0.106, poses);
    REQUIRE(frame.alpha == (0.106 - 0.100) / (0.130 - 0.100));
    REQUIRE(frame.prevKind == StepKind::Stall);
    REQUIRE(frame.nextKind == StepKind::Skip);
}

TEST_CASE("RenderInterpolation.Blend.PositionLerpRotationSlerpOnlyWhenTheBodyCarriesOne", "[RenderInterpolation]")
{
    const glm::quat prevRotation = glm::angleAxis(0.2f, glm::vec3(0.f, 0.f, 1.f));
    const glm::quat nextRotation = glm::angleAxis(1.4f, glm::vec3(0.f, 0.f, 1.f));
    const glm::quat linearPrevRotation = glm::angleAxis(0.7f, glm::vec3(1.f, 0.f, 0.f));
    const glm::quat linearNextRotation = glm::angleAxis(-0.9f, glm::vec3(1.f, 0.f, 0.f));

    Snapshot prev;
    fillSnapshot(prev, 1u, 1.0, {
        makeBody(1u, 0u, true, glm::vec3(0.f, 0.f, 0.f), prevRotation),
        makeBody(1u, 1u, false, glm::vec3(10.f, 20.f, 30.f), linearPrevRotation),
        makeBody(3u, 0u, false, glm::vec3(5.f)),
    });
    Snapshot next;
    fillSnapshot(next, 2u, 2.0, {
        makeBody(1u, 0u, true, glm::vec3(100.f, -50.f, 8.f), nextRotation),
        makeBody(1u, 1u, false, glm::vec3(20.f, 40.f, 60.f), linearNextRotation),
        makeBody(2u, 0u, true, glm::vec3(-7.f, 7.f, 7.f), nextRotation),
    });

    Snapshot poses;
    const double alpha = 0.3;
    const float bodyAlpha = static_cast<float>(alpha);
    REQUIRE(blendRenderPoses(prev, next, alpha, std::numeric_limits<float>::infinity(), poses) == 0u);
    REQUIRE(poses.physicsStep == 2u);
    REQUIRE(poses.bodyCount == 3u);

    const RenderBody* full = findRenderBody(poses, 1u, 0u);
    REQUIRE(full != nullptr);
    requireNear(full->positionCm, glm::vec3(30.f, -15.f, 2.4f));
    requireNear(full->rotation, glm::slerp(prevRotation, nextRotation, bodyAlpha));
    REQUIRE(std::abs(glm::angle(full->rotation) - (0.2f + 1.2f * bodyAlpha)) < 1e-5f);

    const RenderBody* linear = findRenderBody(poses, 1u, 1u);
    REQUIRE(linear != nullptr);
    REQUIRE(linear->hasRotation == 0u);
    requireNear(linear->positionCm, glm::vec3(13.f, 26.f, 39.f));
    REQUIRE(linear->rotation == linearNextRotation);

    const RenderBody* joined = findRenderBody(poses, 2u, 0u);
    REQUIRE(joined != nullptr);
    REQUIRE(joined->positionCm == glm::vec3(-7.f, 7.f, 7.f));
    REQUIRE(joined->rotation == nextRotation);
    REQUIRE(findRenderBody(poses, 3u, 0u) == nullptr);

    REQUIRE(blendRenderPoses(prev, next, 0.0, std::numeric_limits<float>::infinity(), poses) == 0u);
    REQUIRE(findRenderBody(poses, 1u, 0u)->positionCm == glm::vec3(0.f));
    REQUIRE(findRenderBody(poses, 1u, 0u)->rotation == prevRotation);
    REQUIRE(blendRenderPoses(prev, next, 1.0, std::numeric_limits<float>::infinity(), poses) == 0u);
    REQUIRE(findRenderBody(poses, 1u, 0u)->positionCm == glm::vec3(100.f, -50.f, 8.f));
    REQUIRE(findRenderBody(poses, 1u, 0u)->rotation == nextRotation);
}

TEST_CASE("RenderInterpolation.Blend.ABodyFartherThanTheSnapDistanceTakesNextUnblended", "[RenderInterpolation]")
{
    constexpr float kSnapCm = 50000.f;
    Snapshot prev;
    fillSnapshot(prev, 1u, 1.0, {
        makeBody(1u, 0u, false, glm::vec3(0.f, 0.f, 0.f)),
        makeBody(1u, 1u, false, glm::vec3(0.f, 0.f, 0.f)),
        makeBody(1u, 2u, true, glm::vec3(0.f, 0.f, -100000.f), glm::angleAxis(0.5f, glm::vec3(0.f, 1.f, 0.f))),
    });
    Snapshot next;
    fillSnapshot(next, 2u, 2.0, {
        makeBody(1u, 0u, false, glm::vec3(kSnapCm, 0.f, 0.f)),
        makeBody(1u, 1u, false, glm::vec3(0.f, kSnapCm - 1.f, 0.f)),
        makeBody(1u, 2u, true, glm::vec3(30.f, 0.f, 120.f), glm::angleAxis(1.5f, glm::vec3(0.f, 1.f, 0.f))),
    });

    Snapshot poses;
    REQUIRE(blendRenderPoses(prev, next, 0.5, kSnapCm, poses) == 1u);
    requireNear(findRenderBody(poses, 1u, 0u)->positionCm, glm::vec3(0.5f * kSnapCm, 0.f, 0.f));
    requireNear(findRenderBody(poses, 1u, 1u)->positionCm, glm::vec3(0.f, 0.5f * (kSnapCm - 1.f), 0.f));
    REQUIRE(findRenderBody(poses, 1u, 2u)->positionCm == glm::vec3(30.f, 0.f, 120.f));
    REQUIRE(findRenderBody(poses, 1u, 2u)->rotation == glm::angleAxis(1.5f, glm::vec3(0.f, 1.f, 0.f)));

    REQUIRE(blendRenderPoses(prev, next, 0.5, std::numeric_limits<float>::infinity(), poses) == 0u);
    REQUIRE(findRenderBody(poses, 1u, 2u)->positionCm.z == Catch::Approx(-49940.0).margin(1e-2));

    Channel channel;
    *channel.beginWrite() = prev;
    channel.commit();
    *channel.beginWrite() = next;
    channel.commit();
    RenderInterpolationParams params;
    params.renderTimeSeconds = 1.5;
    params.snapDistanceCm = kSnapCm;
    RenderInterpolationFrame frame;
    REQUIRE(interpolateRenderPoses(channel, params, poses, frame));
    REQUIRE(frame.hasPrev);
    REQUIRE(frame.snaps == 1u);
    REQUIRE(findRenderBody(poses, 1u, 2u)->positionCm == glm::vec3(30.f, 0.f, 120.f));
}

TEST_CASE("RenderInterpolation.Slots.AtMostTwoHeldAndAllReleasedOnEveryPath", "[RenderInterpolation]")
{
    CountingChannel channel;
    Snapshot poses;
    RenderInterpolationParams params;
    RenderInterpolationFrame frame;
    params.renderTimeSeconds = 1.0;
    REQUIRE_FALSE(interpolateRenderPoses(channel, params, poses, frame));
    REQUIRE(channel.held.empty());

    for (uint64_t step = 1; step <= 6; ++step)
    {
        publishStep(channel.channel, step);
    }

    struct Path { double renderSteps; bool newestOnly; bool hasPrev; bool atNewest; bool beyondOldest; uint32_t nextBack; };
    const std::array<Path, 7> paths{ {
        { 5.5, false, true, false, false, 0u },
        { 4.5, false, true, false, false, 1u },
        { 3.5, false, true, false, false, 2u },
        { 1.0, false, false, false, true, 3u },
        { 7.0, false, false, true, false, 0u },
        { 6.0, false, false, false, false, 0u },
        { 3.5, true, false, false, false, 0u },
    } };
    for (const Path& path : paths)
    {
        channel.resetCounts();
        params.renderTimeSeconds = path.renderSteps * kStepSeconds;
        params.newestOnly = path.newestOnly;
        REQUIRE(interpolateRenderPoses(channel, params, poses, frame));
        REQUIRE(frame.hasPrev == path.hasPrev);
        REQUIRE(frame.atNewest == path.atNewest);
        REQUIRE(frame.beyondOldest == path.beyondOldest);
        REQUIRE(frame.nextBack == path.nextBack);
        REQUIRE(channel.maxHeld <= 2u);
        REQUIRE(channel.held.empty());
        REQUIRE(channel.releases >= 1u);
    }

    params.newestOnly = false;
    for (uint64_t step = 7; step <= 2000; ++step)
    {
        publishStep(channel.channel, step);
        channel.resetCounts();
        params.renderTimeSeconds = (static_cast<double>(step) - 1.6) * kStepSeconds;
        REQUIRE(interpolateRenderPoses(channel, params, poses, frame));
        REQUIRE(frame.hasPrev);
        REQUIRE(channel.maxHeld <= 2u);
        REQUIRE(channel.held.empty());
    }
    REQUIRE(channel.channel.drops() == 0u);

    for (uint64_t step = 2001; step <= 2004; ++step)
    {
        publishStep(channel.channel, step);
    }
    for (size_t back = 0; back < Channel::kCapacity; ++back)
    {
        const Snapshot* snapshot = channel.channel.peekNewest(back);
        REQUIRE(snapshot != nullptr);
        REQUIRE(snapshot->physicsStep == 2004u - back);
        channel.channel.release(snapshot);
    }
}

TEST_CASE("RenderInterpolation.Slots.AWriterCommitWhileTwoAreHeldNeitherDropsNorShiftsTheBracket", "[RenderInterpolation]")
{
    CountingChannel channel;
    for (uint64_t step = 1; step <= 4; ++step)
    {
        publishStep(channel.channel, step);
    }
    channel.commitAfterBackOne = true;
    channel.writerStep = 5;
    Snapshot poses;
    RenderInterpolationParams params;
    RenderInterpolationFrame frame;

    for (int frameIndex = 0; frameIndex < 500; ++frameIndex)
    {
        const uint64_t newest = channel.writerStep - 1u;
        channel.resetCounts();
        params.renderTimeSeconds = (static_cast<double>(newest) - 1.6) * kStepSeconds;
        REQUIRE(interpolateRenderPoses(channel, params, poses, frame));
        REQUIRE(channel.writerStep == newest + 2u);
        REQUIRE(channel.maxHeld == 2u);
        REQUIRE(channel.held.empty());
        REQUIRE(frame.hasPrev);
        REQUIRE(frame.newestStep == newest);
        REQUIRE(frame.nextStep == newest - 1u);
        REQUIRE(frame.prevStep == newest - 2u);
        REQUIRE(frame.alpha == Catch::Approx(0.4).margin(1e-6));
        requireBodyAt(poses, positionAtStep(newest - 2u) + (positionAtStep(newest - 1u) - positionAtStep(newest - 2u)) * 0.4f);
    }
    REQUIRE(channel.channel.drops() == 0u);
}

TEST_CASE("RenderInterpolation.NoAllocation.ThePoseSetAndTheFrameAreFlatCallerOwnedValues", "[RenderInterpolation]")
{
    STATIC_REQUIRE(std::is_trivially_copyable_v<RenderInterpolationParams>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<RenderInterpolationFrame>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<Snapshot>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<RenderSnapshotT<48>>);

    Channel channel;
    for (uint64_t step = 1; step <= 4; ++step)
    {
        publishStep(channel, step);
    }
    Snapshot poses;
    Snapshot nextCopy;
    const RenderBody* const posesStorage = poses.bodies.data();
    RenderInterpolationParams params;
    params.renderTimeSeconds = 2.5 * kStepSeconds;
    RenderInterpolationFrame frame;
    REQUIRE(interpolateRenderPoses(channel, params, poses, frame, &nextCopy));
    REQUIRE(poses.bodies.data() == posesStorage);
    REQUIRE(nextCopy.physicsStep == 3u);
    REQUIRE(findRenderBody(nextCopy, 1u, 0u)->positionCm == positionAtStep(3));
    REQUIRE(frame.nextStep == 3u);
}

#endif // WITH_LOW_LEVEL_TESTS
