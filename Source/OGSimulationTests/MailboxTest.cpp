// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/Mailbox.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <thread>
#include <type_traits>
#include <vector>

// ---------------------------------------------------------------------------
// Mailbox: the engine-free SPSC ring and triple buffer (og-simulationscheduler-withjolt
// task 4) and the pooled snapshot channel (task 44). The single-thread cases pin the
// semantics; the [og][Stress] cases run one producer and one consumer on two real
// threads. Catch2 assertions are not made from the worker threads: each thread counts
// its own findings and the test thread asserts on them after join(), which is also what
// makes those counts visible to it.
// ---------------------------------------------------------------------------

namespace
{
    using Clock = std::chrono::steady_clock;
    constexpr auto kStressBudget = std::chrono::seconds(5);

    struct LiveCounter
    {
        explicit LiveCounter(int v) : value(v) { ++live; }
        LiveCounter(const LiveCounter& other) : value(other.value) { ++live; }
        LiveCounter(LiveCounter&& other) noexcept : value(other.value) { ++live; }
        ~LiveCounter() { --live; }
        LiveCounter& operator=(const LiveCounter&) = delete;

        static inline int live = 0;
        int value;
    };

    static_assert(!std::is_default_constructible_v<LiveCounter>,
        "LiveCounter must stay non-default-constructible: it proves SpscRing does not need T()");

    struct RingItem
    {
        uint64_t sequence = 0;
        uint64_t check = 0;
    };

    constexpr uint64_t checkOf(uint64_t sequence) { return ~(sequence * 0x9E3779B97F4A7C15ull); }

    // A struct far wider than one atomic store, so a reader that shares a slot with
    // the writer sees a mix of two publishes and fails the checksum.
    struct TornProbe
    {
        uint64_t sequence = 0;
        std::array<uint64_t, 30> words{};
        uint64_t checksum = 0;
    };

    uint64_t checksumOf(const TornProbe& probe)
    {
        uint64_t hash = probe.sequence ^ 0xCBF29CE484222325ull;
        for (const uint64_t word : probe.words)
        {
            hash = (hash ^ word) * 0x100000001B3ull;
            hash ^= hash >> 29;
        }
        return hash;
    }

    void fillProbe(TornProbe& probe, uint64_t sequence)
    {
        probe.sequence = sequence;
        for (size_t i = 0; i < probe.words.size(); ++i)
        {
            probe.words[i] = sequence * 0x9E3779B97F4A7C15ull + i;
        }
        probe.checksum = checksumOf(probe);
    }
}

TEST_CASE("Mailbox.SpscRing.EmptyRingPopsNothing", "[Mailbox]")
{
    SpscRing<int, 4> ring;

    REQUIRE(SpscRing<int, 4>::kCapacity == 4u);
    REQUIRE(ring.sizeApprox() == 0u);
    REQUIRE_FALSE(ring.tryPop().has_value());
    REQUIRE(ring.drops() == 0u);
}

TEST_CASE("Mailbox.SpscRing.OneSlotRingHoldsOneItem", "[Mailbox]")
{
    SpscRing<int, 1> ring;

    REQUIRE(ring.tryPush(5));
    REQUIRE_FALSE(ring.tryPush(6));
    REQUIRE(ring.drops() == 1u);
    REQUIRE(ring.sizeApprox() == 1u);
    REQUIRE(ring.tryPop() == std::optional<int>(5));
    REQUIRE(ring.tryPush(7));
    REQUIRE(ring.tryPop() == std::optional<int>(7));
    REQUIRE_FALSE(ring.tryPop().has_value());
}

TEST_CASE("Mailbox.SpscRing.FifoOrderAcrossManyWraps", "[Mailbox]")
{
    SpscRing<int, 4> ring;
    int nextPushed = 0;
    int nextExpected = 0;

    for (int round = 0; round < 1000; ++round)
    {
        const int batch = round % 5;
        for (int i = 0; i < batch; ++i)
        {
            REQUIRE(ring.tryPush(nextPushed));
            ++nextPushed;
        }
        REQUIRE(ring.sizeApprox() == static_cast<size_t>(batch));
        for (int i = 0; i < batch; ++i)
        {
            const std::optional<int> popped = ring.tryPop();
            REQUIRE(popped.has_value());
            REQUIRE(*popped == nextExpected);
            ++nextExpected;
        }
        REQUIRE_FALSE(ring.tryPop().has_value());
    }

    REQUIRE(nextPushed == 2000);
    REQUIRE(nextExpected == 2000);
    REQUIRE(ring.drops() == 0u);
}

TEST_CASE("Mailbox.SpscRing.FullRingRejectsWithoutConsumingAndCountsTheDrop", "[Mailbox]")
{
    SpscRing<std::unique_ptr<int>, 4> ring;
    for (int i = 0; i < 4; ++i)
    {
        REQUIRE(ring.tryPush(std::make_unique<int>(i)));
    }
    REQUIRE(ring.sizeApprox() == 4u);

    auto rejected = std::make_unique<int>(99);
    REQUIRE_FALSE(ring.tryPush(std::move(rejected)));
    REQUIRE(rejected != nullptr);
    REQUIRE(*rejected == 99);
    REQUIRE(ring.drops() == 1u);

    REQUIRE_FALSE(ring.tryPush(std::move(rejected)));
    REQUIRE(rejected != nullptr);
    REQUIRE(ring.drops() == 2u);
    REQUIRE(ring.sizeApprox() == 4u);

    const std::optional<std::unique_ptr<int>> first = ring.tryPop();
    REQUIRE(first.has_value());
    REQUIRE(**first == 0);

    REQUIRE(ring.tryPush(std::move(rejected)));
    REQUIRE(rejected == nullptr);
    REQUIRE(ring.drops() == 2u);

    for (const int expected : {1, 2, 3, 99})
    {
        const std::optional<std::unique_ptr<int>> popped = ring.tryPop();
        REQUIRE(popped.has_value());
        REQUIRE(**popped == expected);
    }
    REQUIRE_FALSE(ring.tryPop().has_value());
    REQUIRE(ring.drops() == 2u);
}

TEST_CASE("Mailbox.SpscRing.ElementsAreDestroyedOnPopAndWithTheRing", "[Mailbox]")
{
    LiveCounter::live = 0;
    {
        SpscRing<LiveCounter, 4> ring;
        const LiveCounter copied(1);
        REQUIRE(ring.tryPush(copied));
        REQUIRE(ring.tryPush(LiveCounter(2)));
        REQUIRE(ring.tryPush(LiveCounter(3)));
        REQUIRE(LiveCounter::live == 4);

        {
            const std::optional<LiveCounter> popped = ring.tryPop();
            REQUIRE(popped.has_value());
            REQUIRE(popped->value == 1);
            REQUIRE(LiveCounter::live == 4);
        }
        REQUIRE(LiveCounter::live == 3);
    }
    REQUIRE(LiveCounter::live == 0);
}

TEST_CASE("Mailbox.TripleBuffer.LatestWinsAndTheDirtyFlag", "[Mailbox]")
{
    TripleBuffer<int> buffer;

    REQUIRE_FALSE(buffer.dirty());
    REQUIRE(buffer.acquireLatest() == nullptr);

    buffer.write() = 7;
    REQUIRE_FALSE(buffer.dirty());
    REQUIRE(buffer.acquireLatest() == nullptr);

    buffer.publish();
    REQUIRE(buffer.dirty());
    const int* first = buffer.acquireLatest();
    REQUIRE(first != nullptr);
    REQUIRE(*first == 7);
    REQUIRE_FALSE(buffer.dirty());

    REQUIRE(buffer.acquireLatest() == first);
    REQUIRE(*buffer.acquireLatest() == 7);

    for (int value = 10; value <= 12; ++value)
    {
        buffer.write() = value;
        buffer.publish();
    }
    REQUIRE(buffer.dirty());
    REQUIRE(*buffer.acquireLatest() == 12);
    REQUIRE_FALSE(buffer.dirty());

    buffer.write() = 13;
    REQUIRE(*buffer.acquireLatest() == 12);
}

TEST_CASE("Mailbox.TripleBuffer.WriterAndReaderSlotsNeverAlias", "[Mailbox]")
{
    TripleBuffer<int> buffer;
    std::set<const int*> slotsSeen;
    int published = 0;

    for (int step = 0; step < 300; ++step)
    {
        const int publishes = step % 4;
        for (int i = 0; i < publishes; ++i)
        {
            buffer.write() = ++published;
            buffer.publish();
        }
        if (published == 0)
        {
            REQUIRE(buffer.acquireLatest() == nullptr);
            continue;
        }
        const int* front = buffer.acquireLatest();
        const int* back = &buffer.write();
        REQUIRE(front != nullptr);
        REQUIRE(*front == published);
        REQUIRE(front != back);
        slotsSeen.insert(front);
        slotsSeen.insert(back);
    }

    REQUIRE(slotsSeen.size() == 3u);
}

TEST_CASE("Mailbox.SpscRing.StressMillionItemsInOrderWithNoLoss", "[Mailbox][og][Stress]")
{
    constexpr uint64_t kItems = 1'000'000;
    auto ring = std::make_unique<SpscRing<RingItem, 64>>();
    std::atomic<bool> start{false};
    std::atomic<bool> producerDone{false};
    uint64_t producerRejections = 0;

    std::thread producer([&] {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (uint64_t sequence = 0; sequence < kItems;)
        {
            if (ring->tryPush(RingItem{sequence, checkOf(sequence)}))
            {
                ++sequence;
            }
            else
            {
                ++producerRejections;
                std::this_thread::yield();
            }
        }
        producerDone.store(true, std::memory_order_release);
    });

    uint64_t received = 0;
    uint64_t outOfOrder = 0;
    uint64_t corrupt = 0;
    const Clock::time_point began = Clock::now();
    start.store(true, std::memory_order_release);
    while (received < kItems)
    {
        const bool done = producerDone.load(std::memory_order_acquire);
        if (const std::optional<RingItem> item = ring->tryPop())
        {
            outOfOrder += item->sequence != received ? 1u : 0u;
            corrupt += item->check != checkOf(item->sequence) ? 1u : 0u;
            ++received;
        }
        else if (done)
        {
            break;
        }
        else
        {
            std::this_thread::yield();
        }
    }
    producer.join();
    const Clock::duration elapsed = Clock::now() - began;

    INFO("full-ring rejections: " << producerRejections);
    REQUIRE(received == kItems);
    REQUIRE(outOfOrder == 0u);
    REQUIRE(corrupt == 0u);
    REQUIRE_FALSE(ring->tryPop().has_value());
    REQUIRE(ring->drops() == producerRejections);
    REQUIRE(elapsed < kStressBudget);
}

TEST_CASE("Mailbox.TripleBuffer.StressReaderNeverSeesATornStruct", "[Mailbox][og][Stress]")
{
    constexpr uint64_t kPublishes = 1'000'000;
    auto buffer = std::make_unique<TripleBuffer<TornProbe>>();
    std::atomic<bool> start{false};
    std::atomic<bool> writerDone{false};

    std::thread writer([&] {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (uint64_t sequence = 1; sequence <= kPublishes; ++sequence)
        {
            fillProbe(buffer->write(), sequence);
            buffer->publish();
        }
        writerDone.store(true, std::memory_order_release);
    });

    uint64_t reads = 0;
    uint64_t torn = 0;
    uint64_t wentBackwards = 0;
    uint64_t distinct = 0;
    uint64_t lastSequence = 0;
    const Clock::time_point began = Clock::now();
    start.store(true, std::memory_order_release);
    for (;;)
    {
        const bool done = writerDone.load(std::memory_order_acquire);
        if (const TornProbe* latest = buffer->acquireLatest())
        {
            ++reads;
            torn += latest->checksum != checksumOf(*latest) ? 1u : 0u;
            wentBackwards += latest->sequence < lastSequence ? 1u : 0u;
            distinct += latest->sequence != lastSequence ? 1u : 0u;
            lastSequence = latest->sequence;
        }
        if (done)
        {
            break;
        }
    }
    writer.join();
    const Clock::duration elapsed = Clock::now() - began;

    INFO("reads: " << reads << ", distinct publishes observed: " << distinct);
    REQUIRE(torn == 0u);
    REQUIRE(wentBackwards == 0u);
    REQUIRE(lastSequence == kPublishes);
    REQUIRE(distinct >= 2u);
    REQUIRE_FALSE(buffer->dirty());
    REQUIRE(elapsed < kStressBudget);
}

namespace
{
    struct SnapshotAllocations
    {
        static inline uint64_t count = 0;
    };

    template <typename U>
    struct CountingAllocator
    {
        using value_type = U;

        CountingAllocator() = default;
        template <typename V>
        CountingAllocator(const CountingAllocator<V>&) noexcept
        {
        }

        U* allocate(size_t n)
        {
            ++SnapshotAllocations::count;
            return std::allocator<U>{}.allocate(n);
        }

        void deallocate(U* pointer, size_t n) { std::allocator<U>{}.deallocate(pointer, n); }

        template <typename V>
        bool operator==(const CountingAllocator<V>&) const noexcept
        {
            return true;
        }
    };

    // Allocates in its constructor and can be neither copied nor moved, so the channel
    // compiles only if it never copies or moves a snapshot, and every allocation a
    // snapshot makes is counted.
    struct AllocatingSnapshot
    {
        AllocatingSnapshot() : payload(64, 0u) {}
        AllocatingSnapshot(const AllocatingSnapshot&) = delete;
        AllocatingSnapshot& operator=(const AllocatingSnapshot&) = delete;

        std::vector<uint64_t, CountingAllocator<uint64_t>> payload;
        uint64_t sequence = 0;
    };

    template <typename Channel>
    void commitValue(Channel& channel, int value)
    {
        int* slot = channel.beginWrite();
        REQUIRE(slot != nullptr);
        *slot = value;
        channel.commit();
    }

    template <typename Channel>
    std::vector<int> readAll(Channel& channel)
    {
        std::vector<int> values;
        std::vector<const int*> held;
        for (size_t back = 0; back < Channel::kCapacity; ++back)
        {
            const int* snapshot = channel.peekNewest(back);
            if (snapshot == nullptr)
            {
                break;
            }
            values.push_back(*snapshot);
            held.push_back(snapshot);
        }
        for (const int* snapshot : held)
        {
            channel.release(snapshot);
        }
        return values;
    }
}

TEST_CASE("Mailbox.SnapshotChannel.NothingBeforeTheFirstCommit", "[Mailbox]")
{
    SnapshotChannel<int, 4> channel;

    STATIC_REQUIRE(SnapshotChannel<int, 4>::kCapacity == 4u);
    STATIC_REQUIRE(SnapshotChannel<int, 4>::kMaxHeldKeepingNewest == 2u);
    STATIC_REQUIRE(SnapshotChannel<int, 3>::kMaxHeldKeepingNewest == 1u);
    REQUIRE(channel.peekNewest() == nullptr);
    REQUIRE(channel.peekNewest(1) == nullptr);

    channel.commit();
    REQUIRE(channel.peekNewest() == nullptr);

    int* slot = channel.beginWrite();
    REQUIRE(slot != nullptr);
    REQUIRE(channel.beginWrite() == slot);
    *slot = 7;
    REQUIRE(channel.peekNewest() == nullptr);

    channel.commit();
    const int* newest = channel.peekNewest();
    REQUIRE(newest == slot);
    REQUIRE(*newest == 7);
    REQUIRE(channel.peekNewest(1) == nullptr);
    channel.release(newest);
    REQUIRE(channel.drops() == 0u);
}

TEST_CASE("Mailbox.SnapshotChannel.NewestFirstAndBackIndexing", "[Mailbox]")
{
    SnapshotChannel<int, 4> channel;
    for (int value = 1; value <= 3; ++value)
    {
        commitValue(channel, value);
    }

    REQUIRE(*channel.peekNewest(0) == 3);
    REQUIRE(*channel.peekNewest(1) == 2);
    REQUIRE(*channel.peekNewest(2) == 1);
    REQUIRE(channel.peekNewest(3) == nullptr);
    REQUIRE(channel.peekNewest(4) == nullptr);
    REQUIRE(channel.peekNewest(1000) == nullptr);
    REQUIRE(channel.peekNewest(0) == channel.peekNewest());
    channel.release(channel.peekNewest(0));
    channel.release(channel.peekNewest(1));
    channel.release(channel.peekNewest(2));

    for (int value = 4; value <= 9; ++value)
    {
        commitValue(channel, value);
    }
    REQUIRE(readAll(channel) == std::vector<int>{9, 8, 7, 6});
    REQUIRE(channel.drops() == 0u);
}

TEST_CASE("Mailbox.SnapshotChannel.WriterRecyclesTheOldestUnheldSnapshot", "[Mailbox]")
{
    SnapshotChannel<int, 4> channel;
    for (int value = 1; value <= 4; ++value)
    {
        commitValue(channel, value);
    }
    const int* four = channel.peekNewest(0);
    const int* three = channel.peekNewest(1);
    const int* two = channel.peekNewest(2);
    const int* one = channel.peekNewest(3);
    channel.release(four);
    channel.release(three);
    channel.release(two);

    REQUIRE(*one == 1);
    int* recycled = channel.beginWrite();
    REQUIRE(recycled == two);
    *recycled = 5;
    channel.commit();
    REQUIRE(*one == 1);

    recycled = channel.beginWrite();
    REQUIRE(recycled == three);
    *recycled = 6;
    channel.commit();

    recycled = channel.beginWrite();
    REQUIRE(recycled == four);
    *recycled = 7;
    channel.commit();
    REQUIRE(*one == 1);

    channel.release(one);
    REQUIRE(readAll(channel) == std::vector<int>{7, 6, 5, 1});
    REQUIRE(channel.beginWrite() == one);
    REQUIRE(channel.drops() == 0u);
}

TEST_CASE("Mailbox.SnapshotChannel.HeldSnapshotIsNeverRecycled", "[Mailbox]")
{
    SnapshotChannel<int, 3> channel;
    commitValue(channel, 1);
    const int* held = channel.peekNewest();
    REQUIRE(*held == 1);

    for (int value = 2; value <= 200; ++value)
    {
        int* slot = channel.beginWrite();
        REQUIRE(slot != nullptr);
        REQUIRE(slot != held);
        *slot = value;
        channel.commit();
        REQUIRE(*held == 1);
    }

    channel.release(held);
    REQUIRE(readAll(channel) == std::vector<int>{200, 199, 1});
    REQUIRE(channel.drops() == 0u);
}

TEST_CASE("Mailbox.SnapshotChannel.SizingRuleKeepsTheNewestWhileTheReaderHoldsTwo", "[Mailbox]")
{
    SECTION("N = 4 holding two: a new read always sees the last commit, even mid-write")
    {
        SnapshotChannel<int, 4> channel;
        commitValue(channel, 1);
        commitValue(channel, 2);
        for (int value = 3; value <= 100; ++value)
        {
            const int* newest = channel.peekNewest(0);
            const int* previous = channel.peekNewest(1);
            int* slot = channel.beginWrite();
            REQUIRE(slot != nullptr);
            channel.release(previous);
            channel.release(newest);

            const int* fresh = channel.peekNewest(0);
            REQUIRE(*fresh == value - 1);
            channel.release(fresh);

            *slot = value;
            channel.commit();
        }
        REQUIRE(channel.drops() == 0u);
    }

    SECTION("N = 3 holding two: never drops, but the writer recycles the newest")
    {
        SnapshotChannel<int, 3> channel;
        commitValue(channel, 1);
        commitValue(channel, 2);
        commitValue(channel, 3);
        const int* two = channel.peekNewest(1);
        const int* one = channel.peekNewest(2);
        REQUIRE(*two == 2);
        REQUIRE(*one == 1);

        int* slot = channel.beginWrite();
        REQUIRE(slot != nullptr);
        channel.release(two);
        channel.release(one);
        REQUIRE(*channel.peekNewest(0) == 2);
        channel.release(channel.peekNewest(0));

        *slot = 4;
        channel.commit();
        REQUIRE(readAll(channel) == std::vector<int>{4, 2, 1});
        REQUIRE(channel.drops() == 0u);
    }
}

TEST_CASE("Mailbox.SnapshotChannel.DropCountedWhenEverySlotIsHeld", "[Mailbox]")
{
    SnapshotChannel<int, 3> channel;
    for (int value = 1; value <= 3; ++value)
    {
        commitValue(channel, value);
    }
    const int* three = channel.peekNewest(0);
    const int* two = channel.peekNewest(1);
    const int* one = channel.peekNewest(2);

    REQUIRE(channel.beginWrite() == nullptr);
    REQUIRE(channel.drops() == 1u);
    REQUIRE(channel.beginWrite() == nullptr);
    REQUIRE(channel.drops() == 2u);
    channel.commit();
    REQUIRE(*three == 3);
    REQUIRE(*two == 2);
    REQUIRE(*one == 1);

    channel.release(two);
    int* recycled = channel.beginWrite();
    REQUIRE(recycled == two);
    REQUIRE(channel.drops() == 2u);
    *recycled = 4;
    channel.commit();

    channel.release(three);
    channel.release(one);
    REQUIRE(readAll(channel) == std::vector<int>{4, 3, 1});
    REQUIRE(channel.drops() == 2u);
}

TEST_CASE("Mailbox.SnapshotChannel.OneReadSeesOneConsistentView", "[Mailbox]")
{
    SnapshotChannel<int, 4> channel;

    SECTION("a commit between two peeks of one read is not seen until the read ends")
    {
        commitValue(channel, 1);
        commitValue(channel, 2);
        const int* newest = channel.peekNewest(0);
        REQUIRE(*newest == 2);

        commitValue(channel, 3);
        const int* previous = channel.peekNewest(1);
        REQUIRE(previous != nullptr);
        REQUIRE(*previous == 1);
        REQUIRE(channel.peekNewest(0) == newest);

        channel.release(newest);
        REQUIRE(*channel.peekNewest(0) == 2);
        channel.release(previous);
        channel.release(channel.peekNewest(0));

        REQUIRE(*channel.peekNewest(0) == 3);
        REQUIRE(*channel.peekNewest(1) == 2);
        channel.release(channel.peekNewest(0));
        channel.release(channel.peekNewest(1));
    }

    SECTION("peeking the older one first with no recycle (only free slots written): back = 0 is still the read-start newest")
    {
        commitValue(channel, 1);
        commitValue(channel, 2);
        const int* previous = channel.peekNewest(1);
        REQUIRE(*previous == 1);

        commitValue(channel, 3);
        commitValue(channel, 4);
        const int* newest = channel.peekNewest(0);
        REQUIRE(newest != nullptr);
        REQUIRE(*newest == 2);
        REQUIRE(channel.peekNewest(2) == nullptr);

        channel.release(previous);
        channel.release(newest);
        REQUIRE(readAll(channel) == std::vector<int>{4, 3, 2, 1});
    }

    SECTION("a repeated peek returns the same snapshot, and one release frees it")
    {
        commitValue(channel, 1);
        const int* first = channel.peekNewest(0);
        REQUIRE(channel.peekNewest(0) == first);
        channel.release(first);

        for (int value = 2; value <= 5; ++value)
        {
            commitValue(channel, value);
        }
        REQUIRE(readAll(channel) == std::vector<int>{5, 4, 3, 2});
    }
}

TEST_CASE("Mailbox.SnapshotChannel.PeekBelowEveryHeldBackMaySeeACommitFromTheRead", "[Mailbox]")
{
    SnapshotChannel<int, 3> channel;
    commitValue(channel, 1);
    commitValue(channel, 2);
    commitValue(channel, 3);

    SECTION("back = 1 first, then two recycling commits: back = 0 returns one committed during the read, still newer than the held one")
    {
        const int* two = channel.peekNewest(1);
        REQUIRE(*two == 2);

        commitValue(channel, 4);
        commitValue(channel, 5);
        const int* four = channel.peekNewest(0);
        REQUIRE(four != nullptr);
        REQUIRE(*four == 4);
        REQUIRE(*four > *two);
        REQUIRE(channel.peekNewest(1) == two);
        REQUIRE(channel.peekNewest(0) == four);
        REQUIRE(channel.peekNewest(2) == nullptr);

        channel.release(two);
        channel.release(four);
        REQUIRE(readAll(channel) == std::vector<int>{5, 4, 2});
        REQUIRE(channel.drops() == 0u);
    }

    SECTION("back = 2 first, writer mid-recycle: back = 0 returns nullptr, then the next commit")
    {
        const int* one = channel.peekNewest(2);
        REQUIRE(*one == 1);

        int* slot = channel.beginWrite();
        REQUIRE(slot != nullptr);
        REQUIRE(channel.peekNewest(0) == nullptr);
        REQUIRE(channel.peekNewest(2) == one);

        *slot = 4;
        channel.commit();
        const int* four = channel.peekNewest(0);
        REQUIRE(four != nullptr);
        REQUIRE(*four == 4);
        REQUIRE(*four > *one);

        channel.release(one);
        channel.release(four);
        REQUIRE(readAll(channel) == std::vector<int>{4, 3, 1});
    }

    SECTION("back = 0 first fixes the read-start view: the same commits stay invisible")
    {
        const int* three = channel.peekNewest(0);
        REQUIRE(*three == 3);

        commitValue(channel, 4);
        commitValue(channel, 5);
        REQUIRE(channel.peekNewest(0) == three);
        REQUIRE(channel.peekNewest(1) == nullptr);

        channel.release(three);
        REQUIRE(readAll(channel) == std::vector<int>{5, 4, 3});
        REQUIRE(channel.drops() == 0u);
    }
}

TEST_CASE("Mailbox.SnapshotChannel.ReleaseIgnoresForeignAndUnheldPointers", "[Mailbox]")
{
    SnapshotChannel<int, 3> channel;
    for (int value = 1; value <= 3; ++value)
    {
        commitValue(channel, value);
    }
    const int* held = channel.peekNewest(2);
    REQUIRE(*held == 1);

    const int outside = 0;
    channel.release(nullptr);
    channel.release(&outside);
    const int* three = channel.peekNewest(0);
    channel.release(three);
    channel.release(three);

    int* slot = channel.beginWrite();
    REQUIRE(slot != held);
    *slot = 4;
    channel.commit();
    slot = channel.beginWrite();
    REQUIRE(slot != held);
    *slot = 5;
    channel.commit();
    REQUIRE(*held == 1);

    channel.release(held);
    channel.release(held);
    REQUIRE(readAll(channel) == std::vector<int>{5, 4, 1});
    REQUIRE(channel.drops() == 0u);
}

TEST_CASE("Mailbox.SnapshotChannel.SmallestAndLargestChannelsKeepTheirOrder", "[Mailbox]")
{
    SnapshotChannel<int, 2> two;
    for (int value = 1; value <= 5; ++value)
    {
        commitValue(two, value);
    }
    REQUIRE(readAll(two) == std::vector<int>{5, 4});

    SnapshotChannel<int, 12> twelve;
    for (int value = 1; value <= 12; ++value)
    {
        commitValue(twelve, value);
    }
    REQUIRE(readAll(twelve) == std::vector<int>{12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1});

    const int* oldest = twelve.peekNewest(11);
    REQUIRE(*oldest == 1);
    for (int value = 13; value <= 40; ++value)
    {
        commitValue(twelve, value);
    }
    REQUIRE(*oldest == 1);
    twelve.release(oldest);
    REQUIRE(readAll(twelve) == std::vector<int>{40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 1});
    REQUIRE(twelve.drops() == 0u);
}

TEST_CASE("Mailbox.SnapshotChannel.NoAllocationAfterConstruction", "[Mailbox]")
{
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<AllocatingSnapshot>);
    STATIC_REQUIRE_FALSE(std::is_move_constructible_v<AllocatingSnapshot>);

    SnapshotAllocations::count = 0;
    auto channel = std::make_unique<SnapshotChannel<AllocatingSnapshot, 4>>();
    REQUIRE(SnapshotAllocations::count == 4u);

    std::array<const uint64_t*, 4> payloads{};
    for (size_t back = 0; back < 4; ++back)
    {
        AllocatingSnapshot* slot = channel->beginWrite();
        REQUIRE(slot != nullptr);
        payloads[back] = slot->payload.data();
        channel->commit();
    }

    uint64_t sequence = 0;
    for (int cycle = 0; cycle < 10'000; ++cycle)
    {
        const AllocatingSnapshot* newest = channel->peekNewest(0);
        const AllocatingSnapshot* previous = channel->peekNewest(1);
        AllocatingSnapshot* slot = channel->beginWrite();
        REQUIRE(slot != nullptr);
        ++sequence;
        slot->sequence = sequence;
        for (uint64_t& word : slot->payload)
        {
            word = sequence;
        }
        channel->commit();
        channel->release(previous);
        channel->release(newest);
        if (cycle % 7 == 0)
        {
            REQUIRE(channel->beginWrite() != nullptr);
        }
    }

    REQUIRE(SnapshotAllocations::count == 4u);
    for (size_t back = 0; back < 4; ++back)
    {
        const AllocatingSnapshot* snapshot = channel->peekNewest(back);
        REQUIRE(snapshot != nullptr);
        REQUIRE(std::find(payloads.begin(), payloads.end(), snapshot->payload.data()) != payloads.end());
    }
    REQUIRE(channel->peekNewest(0)->sequence == sequence);
    REQUIRE(channel->drops() == 0u);
}

TEST_CASE("Mailbox.SnapshotChannel.StressPairedPeeksNeverTornAndAlwaysOrdered", "[Mailbox][og][Stress]")
{
    constexpr uint64_t kCommits = 1'000'000;
    auto channel = std::make_unique<SnapshotChannel<TornProbe, 4>>();
    std::atomic<bool> start{false};
    std::atomic<bool> writerDone{false};
    uint64_t writerNulls = 0;

    std::thread writer([&] {
        while (!start.load(std::memory_order_acquire))
        {
        }
        for (uint64_t sequence = 1; sequence <= kCommits;)
        {
            if (TornProbe* slot = channel->beginWrite())
            {
                fillProbe(*slot, sequence);
                channel->commit();
                ++sequence;
            }
            else
            {
                ++writerNulls;
                std::this_thread::yield();
            }
        }
        writerDone.store(true, std::memory_order_release);
    });

    uint64_t reads = 0;
    uint64_t pairs = 0;
    uint64_t torn = 0;
    uint64_t changedWhileHeld = 0;
    uint64_t pairNotOlder = 0;
    uint64_t newestWentBackwards = 0;
    uint64_t distinct = 0;
    uint64_t lastNewest = 0;
    const Clock::time_point began = Clock::now();
    start.store(true, std::memory_order_release);
    for (;;)
    {
        const bool done = writerDone.load(std::memory_order_acquire);
        const TornProbe* newest = channel->peekNewest(0);
        const TornProbe* previous = channel->peekNewest(1);
        if (newest != nullptr)
        {
            ++reads;
            const uint64_t newestSequence = newest->sequence;
            torn += newest->checksum != checksumOf(*newest) ? 1u : 0u;
            newestWentBackwards += newestSequence < lastNewest ? 1u : 0u;
            distinct += newestSequence != lastNewest ? 1u : 0u;
            lastNewest = newestSequence;
            if (previous != nullptr)
            {
                ++pairs;
                torn += previous->checksum != checksumOf(*previous) ? 1u : 0u;
                pairNotOlder += previous->sequence >= newestSequence ? 1u : 0u;
            }
            changedWhileHeld += newest->sequence != newestSequence ? 1u : 0u;
        }
        channel->release(previous);
        channel->release(newest);
        if (done)
        {
            break;
        }
    }
    writer.join();
    const Clock::duration elapsed = Clock::now() - began;

    const TornProbe* last = channel->peekNewest(0);
    INFO("reads: " << reads << ", pairs: " << pairs << ", distinct newest: " << distinct);
    REQUIRE(torn == 0u);
    REQUIRE(changedWhileHeld == 0u);
    REQUIRE(pairNotOlder == 0u);
    REQUIRE(newestWentBackwards == 0u);
    REQUIRE(pairs > 0u);
    REQUIRE(distinct >= 2u);
    REQUIRE(last != nullptr);
    REQUIRE(last->sequence == kCommits);
    REQUIRE(writerNulls == 0u);
    REQUIRE(channel->drops() == 0u);
    REQUIRE(elapsed < kStressBudget);
}

#endif // WITH_LOW_LEVEL_TESTS
