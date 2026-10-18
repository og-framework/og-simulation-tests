// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationScheduler.h"
#include "OGSimulation/TickDriverConcepts.h"
#include "OGSimulation/TickDriverLog.h"
#include "OGSimulation/TickDriverThreadContext.h"
#include "TickDriverFakes.h"

namespace
{
    using namespace tickDriverFakes;
    using tickDriverThread::kAuthorityRoleBit;
    using tickDriverThread::kClientRoleBit;

    struct AllocationCounter
    {
        static inline uint64_t allocations = 0u;
    };

    template <typename T>
    struct CountingAllocator
    {
        using value_type = T;

        CountingAllocator() = default;

        template <typename U>
        CountingAllocator(const CountingAllocator<U>&) noexcept
        {
        }

        T* allocate(std::size_t count)
        {
            ++AllocationCounter::allocations;
            return std::allocator<T>{}.allocate(count);
        }

        void deallocate(T* pointer, std::size_t count) noexcept { std::allocator<T>{}.deallocate(pointer, count); }

        template <typename U>
        bool operator==(const CountingAllocator<U>&) const noexcept
        {
            return true;
        }
    };

    using CountingHostLine = BasicHostLine<CountingAllocator<char>>;

    constexpr const char* kLineFormat = "[SimHost.Test] frame=%llu steps=%u ratio=%.4f text=%s";

    std::string fillerFor(std::size_t lineLength)
    {
        const int prefix = std::snprintf(nullptr, 0, kLineFormat, 123456789ull, 7u, 0.3125, "");
        REQUIRE(prefix > 0);
        REQUIRE(lineLength > static_cast<std::size_t>(prefix));
        std::string filler;
        for (std::size_t index = 0; index < lineLength - static_cast<std::size_t>(prefix); ++index)
            filler.push_back(static_cast<char>('a' + index % 26u));
        return filler;
    }

    std::string snprintfLine(const std::string& filler)
    {
        const int length = std::snprintf(nullptr, 0, kLineFormat, 123456789ull, 7u, 0.3125, filler.c_str());
        std::vector<char> bytes(static_cast<std::size_t>(length) + 1u);
        std::snprintf(bytes.data(), bytes.size(), kLineFormat, 123456789ull, 7u, 0.3125, filler.c_str());
        return std::string(bytes.data(), static_cast<std::size_t>(length));
    }

    template <typename WorldLockT>
    concept WorldLockScopeAccepts = requires { typename WorldLockScope<WorldLockT>; };

    static_assert(WorldLockScopeAccepts<WrapperOverLockUnlockOnly>,
        "WorldLockScope takes a lock/unlock wrapper over a Lock/Unlock-only mutex");
    static_assert(!WorldLockScopeAccepts<LockUnlockOnly>,
        "WorldLockScope refuses a Lock/Unlock-only mutex: it requires BasicLockable");

    bool callOnOtherThread(TickDriverThreadContext::AccessPredicate predicate)
    {
        bool result = true;
        std::thread other([&result, predicate] { result = predicate(); });
        other.join();
        return result;
    }
} // namespace

TEST_CASE("TickDriverLog.FormatHostLine.EqualsSnprintfWithoutTruncationOrAllocation", "[TickDriver]")
{
    CountingHostLine line;
    line.reserve(1024u);
    REQUIRE(line.capacity() >= 1024u);
    const std::size_t reserved = line.capacity();

    for (const std::size_t length : { std::size_t{ 100u }, std::size_t{ 600u } })
    {
        CAPTURE(length);
        const std::string filler = fillerFor(length);
        const std::string expected = snprintfLine(filler);
        REQUIRE(expected.size() == length);

        const uint64_t before = AllocationCounter::allocations;
        const char* const text = formatHostLine(line, kLineFormat, 123456789ull, 7u, 0.3125, filler.c_str());
        CHECK(AllocationCounter::allocations == before);
        CHECK(line.capacity() == reserved);
        CHECK(line.size() == length);
        CHECK(std::string(text) == expected);
        CHECK(std::string(line.c_str(), line.size()) == expected);
    }

    const std::string longFiller = fillerFor(5000u);
    const std::string longExpected = snprintfLine(longFiller);
    REQUIRE(longExpected.size() == 5000u);
    const uint64_t beforeLong = AllocationCounter::allocations;
    formatHostLine(line, kLineFormat, 123456789ull, 7u, 0.3125, longFiller.c_str());
    CHECK(AllocationCounter::allocations > beforeLong);
    CHECK(line.size() == 5000u);
    CHECK(std::string(line.c_str(), line.size()) == longExpected);

    const std::string shortFiller = fillerFor(600u);
    const uint64_t afterGrowth = AllocationCounter::allocations;
    formatHostLine(line, kLineFormat, 123456789ull, 7u, 0.3125, shortFiller.c_str());
    CHECK(AllocationCounter::allocations == afterGrowth);
    CHECK(std::string(line.c_str(), line.size()) == snprintfLine(shortFiller));
}

TEST_CASE("TickDriverThreadContext.Configure.PredicateHoldsForTheOwnerThreadBeforeAnyDriver", "[TickDriver]")
{
    SECTION("an inline authority")
    {
        TickDriverThreadContext context;
        context.configure<true>(kAuthorityRoleBit, true);
        REQUIRE(context.stepWorldPredicate() != nullptr);
        CHECK(context.isOwnerThreadForRole());
        CHECK(context.stepWorldPredicate()());
    }
    SECTION("an inline client")
    {
        TickDriverThreadContext context;
        context.configure<true>(kClientRoleBit, true);
        REQUIRE(context.stepWorldPredicate() != nullptr);
        CHECK(context.stepWorldPredicate()());
    }
    SECTION("a worker client's shadow world")
    {
        TickDriverThreadContext context;
        context.configure<true>(kClientRoleBit, false);
        REQUIRE(context.shadowPredicate() != nullptr);
        CHECK(context.shadowPredicate()());
        CHECK_FALSE(context.stepWorldPredicate()());
    }
    SECTION("a fake world host configures in its constructor")
    {
        Events events;
        FakeWorldHost host(events, kAuthorityRoleBit, true);
        CHECK(host.threadContext().stepWorldPredicate()());
    }
}

TEST_CASE("TickDriverThreadContext.Predicates.TwoRoleBitsOnOneThreadStayApart", "[TickDriver]")
{
    Events events;
    FakeWorldHost authority(events, kAuthorityRoleBit, false);
    const TickDriverThreadContext::AccessPredicate authorityStep = authority.threadContext().stepWorldPredicate();
    const TickDriverThreadContext::AccessPredicate clientWorker = TickDriverThreadContext::predicateFor<true>(kClientRoleBit, false);
    const TickDriverThreadContext::AccessPredicate clientInline = TickDriverThreadContext::predicateFor<true>(kClientRoleBit, true);
    const TickDriverThreadContext::AccessPredicate clientShadow = TickDriverThreadContext::shadowPredicateFor<true>(kClientRoleBit);

    CHECK_FALSE(authorityStep());
    {
        const auto worldLock = authority.lockWorld();
        CHECK(authorityStep());
        CHECK_FALSE(clientWorker());
        CHECK_FALSE(clientInline());
    }
    {
        const auto stepLock = authority.lockForStep();
        CHECK(authorityStep());
        CHECK_FALSE(clientWorker());
        CHECK_FALSE(clientInline());
    }
    CHECK_FALSE(clientShadow());

    FakeWorldHost client(events, kClientRoleBit, false);
    CHECK(clientShadow());
    CHECK_FALSE(clientWorker());
    {
        const auto stepLock = client.lockForStep();
        CHECK(clientWorker());
        CHECK_FALSE(authorityStep());
    }
    CHECK_FALSE(authorityStep());
}

TEST_CASE("TickDriverThreadContext.IsOwnerThread.OnlyOnTheConfiguringThread", "[TickDriver]")
{
    TickDriverThreadContext context;
    context.configure<true>(kAuthorityRoleBit, true);
    CHECK(tickDriverThread::isOwnerThread<kAuthorityRoleBit>());
    CHECK_FALSE(tickDriverThread::isOwnerThread<kClientRoleBit>());
    CHECK_FALSE(callOnOtherThread(&tickDriverThread::isOwnerThread<kAuthorityRoleBit>));
    CHECK_FALSE(callOnOtherThread(context.stepWorldPredicate()));
    CHECK(tickDriverThread::isOwnerThread<kAuthorityRoleBit>());
}

TEST_CASE("TickDriverThreadContext.OwnerMark.ClearedWhenTheContextGoes", "[TickDriver]")
{
    {
        TickDriverThreadContext context;
        context.configure<true>(kClientRoleBit, true);
        CHECK(tickDriverThread::isOwnerThread<kClientRoleBit>());
    }
    CHECK_FALSE(tickDriverThread::isOwnerThread<kClientRoleBit>());
}

TEST_CASE("TickDriverThreadContext.OwnerMark.SameRoleContextsOnOneThreadAreCounted", "[TickDriver]")
{
    SECTION("an inline authority")
    {
        auto first = std::make_unique<TickDriverThreadContext>();
        first->configure<true>(kAuthorityRoleBit, true);
        auto second = std::make_unique<TickDriverThreadContext>();
        second->configure<true>(kAuthorityRoleBit, true);
        const TickDriverThreadContext::AccessPredicate step = second->stepWorldPredicate();
        const TickDriverThreadContext::AccessPredicate shadow = second->shadowPredicate();

        first.reset();
        CHECK(second->isOwnerThreadForRole());
        CHECK(step());
        CHECK(shadow());

        second.reset();
        CHECK_FALSE(tickDriverThread::isOwnerThread<kAuthorityRoleBit>());
        CHECK_FALSE(step());
        CHECK_FALSE(shadow());
    }
    SECTION("an inline client")
    {
        auto first = std::make_unique<TickDriverThreadContext>();
        first->configure<true>(kClientRoleBit, true);
        auto second = std::make_unique<TickDriverThreadContext>();
        second->configure<true>(kClientRoleBit, true);
        const TickDriverThreadContext::AccessPredicate step = second->stepWorldPredicate();
        const TickDriverThreadContext::AccessPredicate shadow = second->shadowPredicate();

        first.reset();
        CHECK(second->isOwnerThreadForRole());
        CHECK(step());
        CHECK(shadow());

        second.reset();
        CHECK_FALSE(tickDriverThread::isOwnerThread<kClientRoleBit>());
        CHECK_FALSE(step());
        CHECK_FALSE(shadow());
    }
}

TEST_CASE("TickDriverThreadContext.OwnerMark.DestroyedOnAnotherThreadLeavesBothThreadsAlone", "[TickDriver]")
{
    bool ownerBeforeHandOff = false;
    bool ownerAfterHandOff = false;
    bool stepAfterHandOff = false;
    bool destroyingThreadOwnerAfter = true;
    std::thread owner([&]
    {
        auto context = std::make_unique<TickDriverThreadContext>();
        context->configure<true>(kClientRoleBit, true);
        const TickDriverThreadContext::AccessPredicate step = context->stepWorldPredicate();
        ownerBeforeHandOff = tickDriverThread::isOwnerThread<kClientRoleBit>();
        std::thread destroyer([&]
        {
            context.reset();
            destroyingThreadOwnerAfter = tickDriverThread::isOwnerThread<kClientRoleBit>();
        });
        destroyer.join();
        ownerAfterHandOff = tickDriverThread::isOwnerThread<kClientRoleBit>();
        stepAfterHandOff = step();
    });
    owner.join();

    CHECK(ownerBeforeHandOff);
    CHECK(ownerAfterHandOff);
    CHECK(stepAfterHandOff);
    CHECK_FALSE(destroyingThreadOwnerAfter);
    CHECK_FALSE(tickDriverThread::isOwnerThread<kClientRoleBit>());
}

TEST_CASE("TickDriverThreadContext.Diagnostics.CountersOnlyWhenEnabled", "[TickDriver]")
{
    SECTION("enabled")
    {
        Events events;
        FakeWorldHost client(events, kClientRoleBit, false);
        const TickDriverThreadContext& context = client.threadContext();
        const uint64_t shadowBefore = TickDriverThreadContext::shadowQueryAccessesOnThisThread();
        const uint64_t stepBefore = TickDriverThreadContext::stepQueryOwnerThreadAccessesOnThisThread();

        CHECK_FALSE(context.stepWorldPredicate()());
        CHECK(TickDriverThreadContext::stepQueryOwnerThreadAccessesOnThisThread() == stepBefore + 1u);
        {
            const auto worldLock = client.lockWorld();
            CHECK(context.stepWorldPredicate()());
        }
        CHECK(TickDriverThreadContext::stepQueryOwnerThreadAccessesOnThisThread() == stepBefore + 1u);
        CHECK_FALSE(callOnOtherThread(context.stepWorldPredicate()));
        CHECK(TickDriverThreadContext::stepQueryOwnerThreadAccessesOnThisThread() == stepBefore + 1u);

        CHECK(context.shadowPredicate()());
        CHECK(context.shadowPredicate()());
        CHECK(TickDriverThreadContext::shadowQueryAccessesOnThisThread() == shadowBefore + 2u);
    }
    SECTION("disabled")
    {
        Events events;
        FakeWorldHostNoDiagnostics client(events, kClientRoleBit, false);
        const TickDriverThreadContext& context = client.threadContext();
        const uint64_t shadowBefore = TickDriverThreadContext::shadowQueryAccessesOnThisThread();
        const uint64_t stepBefore = TickDriverThreadContext::stepQueryOwnerThreadAccessesOnThisThread();

        CHECK_FALSE(context.stepWorldPredicate()());
        CHECK(context.shadowPredicate()());
        CHECK(TickDriverThreadContext::stepQueryOwnerThreadAccessesOnThisThread() == stepBefore);
        CHECK(TickDriverThreadContext::shadowQueryAccessesOnThisThread() == shadowBefore);
    }
}

TEST_CASE("TickDriverThreadContext.Scopes.MarkTheRoleAndOpenTheAccessScope", "[TickDriver]")
{
    Events events;
    FakeWorldHost client(events, kClientRoleBit, false);
    const TickDriverThreadContext& context = client.threadContext();

    CHECK_FALSE(context.isWorldLockHeldOnThisThread());
    CHECK_FALSE(FakeAccessScope::isOpenOnThisThread());
    {
        const auto worldLock = client.lockWorld();
        CHECK(context.isWorldLockHeldOnThisThread());
        CHECK_FALSE(context.isStepRunningOnThisThread());
        CHECK(FakeAccessScope::isOpenOnThisThread());
    }
    CHECK_FALSE(context.isWorldLockHeldOnThisThread());
    CHECK_FALSE(FakeAccessScope::isOpenOnThisThread());
    {
        const auto stepLock = client.lockForStep();
        CHECK(context.isWorldLockHeldOnThisThread());
        CHECK_FALSE(context.isStepRunningOnThisThread());
    }
    CHECK_FALSE(context.isStepRunningOnThisThread());
    CHECK(client.lock().locks() == 2u);
}

TEST_CASE("TickRateDoor.ForwardsToTheScheduler", "[TickDriver]")
{
    SimulationScheduler scheduler(SchedulerConfig{ 1.0 / 60.0, 4u }, 0.0);
    TickRateDoor door(scheduler);

    door.setRateScale(1.05);
    CHECK(scheduler.rateScale() == 1.05);
    CHECK(door.rateScale() == 1.05);

    scheduler.pump(1.0);
    CHECK(door.physicsStepCount() == scheduler.physicsStepCount());
    CHECK(door.physicsStepCount() == 4u);
    CHECK(door.lostTime().steps == scheduler.lostTime().steps);
    CHECK(door.lostTime().steps > 0u);
    CHECK(door.lostTime().seconds == scheduler.lostTime().seconds);
}

TEST_CASE("TickDriverFakes.Engine.ManualExecutorRunsInOrderAndWaitsRunTheQueue", "[TickDriver]")
{
    FakeEngine engine;
    std::vector<int> ran;

    const FakeEngine::Handle first = engine.dispatchAfter(FakeEngine::Handle{}, [&ran] { ran.push_back(1); });
    const FakeEngine::Handle second = engine.dispatchAfter(first, [&ran] { ran.push_back(2); });
    CHECK(engine.isPending(first));
    CHECK(engine.isPending(second));
    CHECK(engine.dispatchedAfter == std::vector<uint64_t>{ 0u, first->id });
    CHECK(ran.empty());

    FakeCompletion completion;
    bool continued = false;
    CHECK(engine.deferUntil(completion, second, [&continued] { continued = true; }));
    CHECK(engine.runDeferred() == 0u);
    CHECK_FALSE(continued);

    engine.waitFor(second);
    CHECK(ran == std::vector<int>{ 1, 2 });
    CHECK_FALSE(engine.isPending(second));
    CHECK(engine.waits == std::vector<uint64_t>{ second->id });
    CHECK(engine.runDeferred() == 1u);
    CHECK(continued);
    CHECK(completion.continuations.size() == 1u);

    engine.deferralSupported = false;
    CHECK_FALSE(engine.deferUntil(completion, second, [] {}));
    CHECK(engine.pendingDeferrals() == 0u);

    engine.minimumLevel = HostLogLevel::Log;
    CHECK_FALSE(engine.logEnabled(HostLogLevel::Verbose));
    CHECK(engine.logEnabled(HostLogLevel::Warning));
    engine.log(HostLogLevel::Warning, "[SimHost.Test] line");
    REQUIRE(engine.lines.size() == 1u);
    CHECK(engine.lines[0].level == HostLogLevel::Warning);
    CHECK(engine.lines[0].text == "[SimHost.Test] line");

    engine.advance(0.5);
    engine.advanceFrame();
    CHECK(engine.nowSeconds() == 0.5);
    CHECK(engine.frameNumber() == 1u);
}

TEST_CASE("TickDriverFakes.Engine.ThreadModeRunsTasksOffTheCallingThread", "[TickDriver]")
{
    FakeEngine engine;
    engine.startWorkerThread();
    std::thread::id ranOn;
    const FakeEngine::Handle task = engine.dispatchAfter(FakeEngine::Handle{}, [&ranOn] { ranOn = std::this_thread::get_id(); });
    engine.waitFor(task);
    CHECK_FALSE(engine.isPending(task));
    CHECK(ranOn == engine.workerThreadId());
    CHECK(ranOn != std::this_thread::get_id());
    engine.stopWorkerThread();
}

TEST_CASE("TickDriverFakes.RecordingGameAndHosts.RecordIntoOneOrderedList", "[TickDriver]")
{
    Events events;
    FakeManager manager;
    FakeStorage storage;
    RecordingGame game(events, manager, storage);
    FakePresenter presenter(events);
    FakeWorldHost host(events, kAuthorityRoleBit, true);
    SimulationScheduler scheduler(SchedulerConfig{ 1.0 / 60.0, 4u }, 0.0);
    TickRateDoor door(scheduler);

    TickOutcome outcome;
    outcome.tick = 12u;
    game.beforePump(0.25, door);
    game.onFrameStepsDue(12, 2);
    game.releaseInputsForAuthorityTick(12u);
    game.onStepStarted(0.26);
    host.stepWorld().step(1.f / 60.f);
    game.onStepEnded();
    presenter.publish_Step(outcome, 0.3);
    host.afterTick_Step(outcome);
    host.afterRunTick_Step(5u, outcome);
    presenter.captureCorrection_Step(5u);
    game.afterInlineBatch(12);
    game.onPostPhysics();

    CHECK(events == Events{ "beforePump", "onFrameStepsDue:12,2", "releaseInputs:12", "onStepStarted", "step",
        "onStepEnded", "publish:12", "hostAfterTick:12", "afterRunTick:5", "captureCorrection:5", "afterInlineBatch:12",
        "onPostPhysics" });
    CHECK(game.beforePumpHostSeconds == std::vector<double>{ 0.25 });
    CHECK(&game.manager() == &manager);
    CHECK(&game.storage() == &storage);
    CHECK(presenter.publishedDeadlines == std::vector<double>{ 0.3 });

    FakeRenderSink sink;
    sink.addTarget(3u, glm::vec3(1.f, 2.f, 3.f), "three");
    CHECK(sink.readAnchor(3u) == glm::vec3(1.f, 2.f, 3.f));
    CHECK(sink.applyRootPose(3u, glm::vec3(4.f, 5.f, 6.f), std::nullopt) == glm::vec3(4.f, 5.f, 6.f));
    CHECK(sink.readAnchor(3u) == glm::vec3(4.f, 5.f, 6.f));
    CHECK_FALSE(sink.applyRootPose(9u, glm::vec3(0.f), std::nullopt).has_value());
    CHECK(std::string(sink.name(3u)) == "three");
    CHECK(std::string(sink.name(9u)) == "?");
    CHECK(sink.applies.size() == 2u);
}

#endif // WITH_LOW_LEVEL_TESTS
