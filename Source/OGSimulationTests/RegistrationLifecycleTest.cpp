// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/DeferredLifecycleQueue.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationInputResolution.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SimulationTimeContext.h"

// SimulationNetSync.h declares SimulatableOwnerTraits primary; it must come before
// the specialization below.
#include "OGSimulation/SimulationNetSync.h"

#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-field-defects / task 3: the REGISTRATION RACE.
//
// The defect (impl/bugreport_registration_race_local_input_cache.md): the game
// thread ran `registerSimulatable`, which published the id into storage BEFORE
// the input resolver had a delay line for it, while the physics thread was
// inside `collectInputAll` iterating that same storage. The physics thread
// found the id, found its provider, and threw `std::out_of_range` out of
// `m_localInputCaches.at(id)`.
//
// This file pins all three halves of the fix:
//
//   B  ORDER. `registerSimulatable` publishes to storage LAST and
//      `registerLocalCharacter` creates the delay line BEFORE the provider, so
//      an id that is visible to the collect loop always has everything the
//      collect loop will ask it for. Pinned by observing the exact instant of
//      publication — the simulatable's move into storage.
//
//   C  DEGRADE. `collectInputForCharacter` / `collectResimInputForCharacter`
//      look the line up with `find` and answer the injected neutral on a miss,
//      emitting ONE `NOCACHE` line per tick rather than one per character.
//      That state is unreachable through the production API once B and A are in
//      place, which is exactly why it is synthesised here — a tripwire nobody
//      has watched fire is not a tripwire.
//
//   A  MARSHALLING. `DeferredLifecycleQueue`: lifecycle changes are ENQUEUED on
//      the game thread and APPLIED on the physics thread between steps. A
//      registration is invisible to the collect on the step it was queued and
//      fully visible on the next; an unregistration likewise.
//
//      ⛔ NO ADAPTER IN THE TREE USES THE QUEUE. It was wired into the
//      og-brawler-unreal composition root, measured, and HELD - adopting it
//      inverts the game/physics crossing rather than closing it, because two
//      per-tick GAME-thread readers ITERATE the maps registration would then
//      mutate from the physics thread. The ruling and the full reader list are
//      in the queue's own banner and in og-simulation's
//      docs/ThreadingCrossings.md row 11. These cases pin the primitive's
//      semantics so that adoption, when it happens, is a wiring change against
//      something already tested - they assert nothing about production wiring.
//
//      ⛔ RENAMED 2026-09-19 (task 3 rework 1), AND THE OLD NAMES ARE THE POINT.
//      They were `RegistrationAppliesOnThePhysicsThreadBetweenSteps` and
//      `UnregistrationAppliesOnThePhysicsThreadBetweenSteps`. Neither case has a
//      thread or a physics step in it - both drive the queue directly, on the
//      test's one thread - so the names asserted the very wiring the ruling above
//      says does not exist. They are now
//      `ADeferredRegistrationIsInvisibleUntilApplied` and
//      `ADeferredUnregistrationIsInvisibleUntilApplied`, which is what they check:
//      invisible before `applyPending`, whole after it. The assertions are
//      unchanged. Tag `[DeferredLifecycle]` is kept - it names the primitive, and
//      that name is accurate.
//
// LOCAL isolation, the convention this module uses everywhere: a scratch mock
// simulatable and scratch mock owners. Nothing here names a brawler or UE type.
//
// FRONTIER-PAIR DISCIPLINE: no case in this file calls
// `reconciliation.postPredictionAll`, so none of them opens a frontier pair and
// none needs the `collectInputAll` -> `allocateFrontierSlotsAll` pairing
// SimulationInputResolutionTest.cpp's banner describes.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // --- Serializable state/input payloads ---------------------------------
    struct RegState
    {
        std::int32_t position = 0;
        bool isSimilarTo(const RegState& other) const { return position == other.position; }
    };

    struct RegInput
    {
        std::int32_t value = 0;
    };
} // namespace

// Required because `registerPredictionOwner`'s REMOTE arm instantiates
// `ingestRelayRing<RegSimulatable>`, which drives the real
// RelayedInputRingCodec and derives its wire stride from the SIM_MEMBER
// machinery. Both arms compile even when only the local one is taken.
template <>
struct SerializableFields<RegInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(RegInput, value));
    }
};

namespace
{
    // --- Mock simulatable ---------------------------------------------------
    class RegAllState
    {
    public:
        const RegState& getState() const { return m_state; }
        RegState&       editState()       { return m_state; }
    private:
        RegState m_state;
    };

    // Armed around exactly one `registerSimulatable` call. The move below is the
    // ONE the whole file turns on: `SimulationObjectStorage::add` is
    // `make_unique<T>(std::forward<T>(simulatable))`, so a simulatable is moved
    // exactly once, at the instant it becomes visible to `forEachSimulatable`,
    // and the map holds unique_ptrs so a later rehash never moves it again.
    std::function<void()> g_onPublishedToStorage;

    struct RegSimulatable
    {
        using StateType = RegState;
        using InputType = RegInput;

        RegAllState m_allState;
        RegAllState m_vizState;

        RegSimulatable() = default;
        RegSimulatable(const RegSimulatable&) = default;
        RegSimulatable& operator=(const RegSimulatable&) = default;
        RegSimulatable& operator=(RegSimulatable&&) = default;

        // ⛔ NOTHING THAT CAN THROW MAY RUN IN HERE. A Catch2 `REQUIRE` throws on
        //   failure and this constructor is `noexcept`, so the hook RECORDS and the
        //   assertions are made by the case, after the call returns.
        RegSimulatable(RegSimulatable&& other) noexcept
            : m_allState(other.m_allState)
            , m_vizState(other.m_vizState)
        {
            if (g_onPublishedToStorage)
                g_onPublishedToStorage();
        }

        const RegAllState& getAllState() const { return m_allState; }
        RegAllState&       editAllState()       { return m_allState; }
        void updateVizState() { m_vizState = m_allState; }
        const RegAllState& getVizState() const { return m_vizState; }
    };

    static_assert(SimulatableState<RegSimulatable>,
        "RegSimulatable must satisfy the concept SimulationObjectStorage/"
        "SimulationReconciliation require");

    // --- Mock synced buffers + owners, the minimum the owner concepts demand --
    struct RegStateSyncBuffer
    {
        std::uint32_t lastTick = 0;
        std::uint32_t lastAppliedCaptureTick = kNoInputCaptureTick;

        void write(const RegState&, std::uint32_t tick, std::uint32_t appliedCaptureTick)
        {
            lastTick = tick;
            lastAppliedCaptureTick = appliedCaptureTick;
        }
        void write(const RegState& state, std::uint32_t tick)
        { write(state, tick, kNoInputCaptureTick); }

        std::uint32_t readInto(RegState&) const { return lastTick; }
        std::uint32_t getAppliedCaptureTick() const { return lastAppliedCaptureTick; }

        template <typename T> T readFromBuffer(std::uint32_t) const { return T{}; }
        template <typename T> void writeToBuffer(std::uint32_t, T) {}
    };

    struct RegInputSyncBuffer
    {
        std::uint32_t lastTick = 0;
        RegInput      lastInput{};

        void write(const RegInput& input, std::uint32_t tick)
        { lastInput = input; lastTick = tick; }

        std::uint32_t readInto(RegInput& outInput) const
        { outInput = lastInput; return lastTick; }

        template <typename T> T readFromBuffer(std::uint32_t) const { return T{}; }
        template <typename T> void writeToBuffer(std::uint32_t, T) {}
    };

    struct RegRelayedInputRing
    {
        std::vector<std::uint8_t> bytes;

        std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

        void bundleAddZeroedBytes(std::int32_t count)
        { bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u); }

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        { std::memcpy(bytes.data() + off, &value, sizeof(T)); }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        { T value; std::memcpy(&value, bytes.data() + off, sizeof(T)); return value; }
    };

    struct RegPredictionOwner
    {
        using SyncedCorrectionBufferType  = RegStateSyncBuffer;
        using SyncedRemoteInputBufferType = RegInputSyncBuffer;
        using RelayedInputRingType        = RegRelayedInputRing;

        std::function<void(const RegStateSyncBuffer&)>   onCorrectionStateReceived;
        std::function<void(const RegRelayedInputRing&)>  onRelayedInputReceived;
        RegInputSyncBuffer                               outgoingInputBuffer;
        RegRelayedInputRing                              relayedInputRing;

        void setOnCorrectionStateReceivedCallback(std::function<void(const RegStateSyncBuffer&)> fn)
        { onCorrectionStateReceived = std::move(fn); }
        void clearOnCorrectionStateReceivedCallback() { onCorrectionStateReceived = nullptr; }

        void setOnRelayedInputReceivedCallback(std::function<void(const RegRelayedInputRing&)> fn)
        { onRelayedInputReceived = std::move(fn); }
        void clearOnRelayedInputReceivedCallback() { onRelayedInputReceived = nullptr; }

        const RegRelayedInputRing& getRelayedInputRing() const { return relayedInputRing; }

        RegInputSyncBuffer* getClientToServerInputSyncedBuffer() { return &outgoingInputBuffer; }

        void sendLocalInputToAuthority(const PendingInputQueue<RegInput>&,
                                       std::uint32_t, std::uint32_t) {}

        // The AUTHORITY half. One mock stands for both owner roles here because no
        // case in this file registers an authority character; what the roles need is
        // that `clearOwnerCallbacks` compiles against both.
        RegStateSyncBuffer                                 correctionStateBuffer;
        std::function<void(std::uint32_t, const RegInput&)> onRemoteMoveReceived;

        RegStateSyncBuffer& getSyncedCorrectionStateBuffer() { return correctionStateBuffer; }

        void setOnRemoteMoveReceivedCallback(
            std::function<void(std::uint32_t, const RegInput&)> fn)
        { onRemoteMoveReceived = std::move(fn); }
        void clearOnRemoteMoveReceivedCallback() { onRemoteMoveReceived = nullptr; }
    };

    static_assert(PredictionSyncedBufferOwnerConcept<RegPredictionOwner, RegState, RegInput>,
        "RegPredictionOwner must satisfy PredictionSyncedBufferOwnerConcept");
    static_assert(AuthoritySyncedBufferOwnerConcept<RegPredictionOwner, RegState, RegInput>,
        "RegPredictionOwner must satisfy AuthoritySyncedBufferOwnerConcept");
} // namespace

template <>
struct SimulatableOwnerTraits<RegSimulatable>
{
    using PredictionOwnerType = RegPredictionOwner;
    using AuthorityOwnerType  = RegPredictionOwner;
};

// ---------------------------------------------------------------------------
// The tear probe, declared as the resolver's one friend and DEFINED ONLY HERE.
//
// `SimulationInputResolution`'s registration order (delay line before provider)
// makes "provider present, delay line absent" unreachable through any COMPLETED
// call of the public API. Synthesising it is the only way to witness the NOCACHE
// tripwire actually firing. ⚠ Ordering is the whole of that argument: nothing
// marshals registration onto the collect's thread, so the concurrent-mutation
// route is open and unaddressed (task 4).
// ---------------------------------------------------------------------------
template <typename... Ts>
struct LocalInputCacheTearProbe
{
    template <typename T>
    static void eraseLine(SimulationInputResolution<Ts...>& resolution, unsigned int id)
    {
        std::get<LocalInputCacheMapFor<T>>(resolution.m_localInputCaches).erase(id);
    }
};

namespace
{
    using RegStorage        = SimulationObjectStorage<RegSimulatable>;
    using RegReconciliation = SimulationReconciliation<RegSimulatable>;
    using RegResolution     = SimulationInputResolution<RegSimulatable>;
    using RegNetSync        = SimulationNetSync<RegSimulatable>;
    using RegResolvedInputs = ResolvedInputs<RegSimulatable>;

    constexpr unsigned int kIdA = 11u;
    constexpr unsigned int kIdB = 12u;
    constexpr float        kDeltaSeconds = 1.0f / 60.0f;
    constexpr std::int32_t kNeutralValue = 7;

    SimulationTimeStep normalStep(std::uint32_t tick)
    {
        return SimulationTimeStep(tick, /*isResimulating=*/false, StepKind::Normal, kDeltaSeconds);
    }

    // storage + a REAL reconciliation + a REAL resolution peer + a REAL net-sync,
    // driven through the REAL `registerSimulatable` / `unregisterSimulatable`
    // facades — the whole point is that the ordering under test is production's.
    struct RegistrationRig
    {
        RegStorage        storage;
        RegReconciliation reconciliation{ storage };
        RegResolution     resolution{ storage, reconciliation };
        RegNetSync        netSync{ storage, reconciliation, resolution };

        std::vector<std::string> log;

        RegistrationRig()
        {
            auto sink = [this](const char* msg) { log.emplace_back(msg); };
            resolution.setLogger(sink);
            netSync.setLogger(sink);
            resolution.setNeutralInput<RegSimulatable>(RegInput{ kNeutralValue });
        }

        std::size_t countLinesContaining(const char* needle) const
        {
            std::size_t n = 0;
            for (const std::string& line : log)
                if (line.find(needle) != std::string::npos)
                    ++n;
            return n;
        }
    };

    std::function<RegInput(const SimulationTimeStep&, const LocalInputCache<RegInput>&)>
    constantProvider(std::int32_t value)
    {
        return [value](const SimulationTimeStep&, const LocalInputCache<RegInput>&)
        { return RegInput{ value }; };
    }

    void registerLocal(RegistrationRig& rig, unsigned int id, RegPredictionOwner& owner,
                       std::int32_t providerValue)
    {
        registerSimulatable<RegSimulatable>(
            rig.storage, rig.reconciliation, rig.resolution, rig.netSync,
            id, RegSimulatable{}, owner, constantProvider(providerValue));
    }

    bool hasInputFor(const RegResolvedInputs& inputs, unsigned int id)
    {
        const auto& map = std::get<std::unordered_map<unsigned int, RegInput>>(inputs);
        return map.find(id) != map.end();
    }

    std::int32_t inputValueFor(const RegResolvedInputs& inputs, unsigned int id)
    {
        const auto& map = std::get<std::unordered_map<unsigned int, RegInput>>(inputs);
        const auto it = map.find(id);
        REQUIRE(it != map.end());
        return it->second.value;
    }
} // namespace

// ===========================================================================
// B — PUBLICATION ORDER
// ===========================================================================

TEST_CASE("ASimulatableIsNeverVisibleToCollectWithoutItsInputCache",
          "[Registration][RegistrationOrder]")
{
    RegistrationRig    rig;
    RegPredictionOwner owner;

    bool observedPublication = false;
    bool lineExistedAtPublication = false;
    bool providerExistedAtPublication = false;

    g_onPublishedToStorage = [&]()
    {
        observedPublication = true;
        lineExistedAtPublication =
            rig.resolution.getDiagnostics().localInputCache<RegSimulatable>(kIdA) != nullptr;
        providerExistedAtPublication =
            rig.resolution.isLocallyControlled<RegSimulatable>(kIdA);
    };

    registerLocal(rig, kIdA, owner, /*providerValue=*/3);

    g_onPublishedToStorage = nullptr;

    // The hook has to have run, or the case below asserts about nothing.
    REQUIRE(observedPublication);

    // THE INVARIANT: at the instant the id becomes reachable from
    // `forEachSimulatable`, everything `collectInputForCharacter` will ask it
    // for already exists. Provider present with NO delay line is precisely the
    // shipped crash.
    CHECK(providerExistedAtPublication);
    CHECK(lineExistedAtPublication);
}

TEST_CASE("ARegisteredLocalCharacterHasBothItsProviderAndItsLine",
          "[Registration][RegistrationOrder]")
{
    RegistrationRig    rig;
    RegPredictionOwner owner;

    registerLocal(rig, kIdA, owner, /*providerValue=*/3);

    CHECK(rig.storage.has<RegSimulatable>(kIdA));
    CHECK(rig.resolution.isLocallyControlled<RegSimulatable>(kIdA));
    CHECK(rig.resolution.getDiagnostics().localInputCache<RegSimulatable>(kIdA) != nullptr);

    const RegResolvedInputs inputs = rig.resolution.collectInputAll(normalStep(100u));
    CHECK(inputValueFor(inputs, kIdA) == 3);
}

// ===========================================================================
// C — THE NOCACHE TRIPWIRE
// ===========================================================================

TEST_CASE("CollectDegradesToNeutralWhenTheCacheIsMissing",
          "[Registration][NoCache]")
{
    RegistrationRig    rig;
    RegPredictionOwner owner;

    registerLocal(rig, kIdA, owner, /*providerValue=*/3);

    LocalInputCacheTearProbe<RegSimulatable>::eraseLine<RegSimulatable>(rig.resolution, kIdA);

    REQUIRE(rig.storage.has<RegSimulatable>(kIdA));
    REQUIRE(rig.resolution.isLocallyControlled<RegSimulatable>(kIdA));
    REQUIRE(rig.resolution.getDiagnostics().localInputCache<RegSimulatable>(kIdA) == nullptr);

    // RED BEFORE THE FIX: this line throws std::out_of_range out of
    // `m_localInputCaches.at(id)` — on the physics thread, in the shipped build.
    const RegResolvedInputs inputs = rig.resolution.collectInputAll(normalStep(100u));

    // The id still resolves, and it resolves to the INJECTED neutral, never a
    // value-initialised input.
    REQUIRE(hasInputFor(inputs, kIdA));
    CHECK(inputValueFor(inputs, kIdA) == kNeutralValue);

    CHECK(rig.countLinesContaining("NOCACHE") == 1u);
}

TEST_CASE("TheNoCacheTripwireIsRateLimitedToOneLinePerTick",
          "[Registration][NoCache]")
{
    RegistrationRig    rig;
    RegPredictionOwner ownerA;
    RegPredictionOwner ownerB;

    registerLocal(rig, kIdA, ownerA, /*providerValue=*/3);
    registerLocal(rig, kIdB, ownerB, /*providerValue=*/4);

    LocalInputCacheTearProbe<RegSimulatable>::eraseLine<RegSimulatable>(rig.resolution, kIdA);
    LocalInputCacheTearProbe<RegSimulatable>::eraseLine<RegSimulatable>(rig.resolution, kIdB);

    // ONE tick, TWO torn characters: the tripwire is a per-tick line, not a
    // per-call one, so an ongoing tear costs one line per tick no matter how
    // many characters are in it.
    rig.resolution.collectInputAll(normalStep(200u));
    CHECK(rig.countLinesContaining("NOCACHE") == 1u);

    // A second tick is a second line — the tripwire is rate-limited, not latched.
    rig.resolution.collectInputAll(normalStep(201u));
    CHECK(rig.countLinesContaining("NOCACHE") == 2u);

    // Both misses of the first tick are accounted for: the second character's is
    // carried as the suppressed count on the NEXT emitted line.
    CHECK(rig.countLinesContaining("suppressed=1") == 1u);
}

// ===========================================================================
// A — THE MARSHALLING PRIMITIVE'S SEMANTICS (the queue alone; HELD, NOT WIRED)
// ===========================================================================

TEST_CASE("ADeferredRegistrationIsInvisibleUntilApplied",
          "[Registration][DeferredLifecycle]")
{
    RegistrationRig       rig;
    RegPredictionOwner    owner;
    DeferredLifecycleQueue queue;

    // GAME THREAD: enqueue only. Nothing the collect loop reads has changed.
    queue.enqueue([&rig, &owner]()
    {
        registerLocal(rig, kIdA, owner, /*providerValue=*/3);
    });

    CHECK(queue.hasPending());

    // STEP N — the queued registration is invisible.
    const RegResolvedInputs stepN = rig.resolution.collectInputAll(normalStep(300u));
    CHECK_FALSE(rig.storage.has<RegSimulatable>(kIdA));
    CHECK_FALSE(hasInputFor(stepN, kIdA));

    // The apply call — where an adopting adapter would put it is between steps on
    // the physics thread; this case runs it on the test's one thread and asserts
    // only the queue's own before/after semantics.
    CHECK(queue.applyPending() == 1u);
    CHECK_FALSE(queue.hasPending());

    // STEP N+1 — fully visible: in storage, with a provider AND a delay line.
    const RegResolvedInputs stepNPlus1 = rig.resolution.collectInputAll(normalStep(301u));
    CHECK(rig.storage.has<RegSimulatable>(kIdA));
    CHECK(rig.resolution.isLocallyControlled<RegSimulatable>(kIdA));
    CHECK(rig.resolution.getDiagnostics().localInputCache<RegSimulatable>(kIdA) != nullptr);
    CHECK(inputValueFor(stepNPlus1, kIdA) == 3);

    // …and the collect never took the degrade branch on the way through.
    CHECK(rig.countLinesContaining("NOCACHE") == 0u);
}

TEST_CASE("ADeferredUnregistrationIsInvisibleUntilApplied",
          "[Registration][DeferredLifecycle]")
{
    RegistrationRig       rig;
    RegPredictionOwner    owner;
    DeferredLifecycleQueue queue;

    registerLocal(rig, kIdA, owner, /*providerValue=*/3);
    REQUIRE(rig.storage.has<RegSimulatable>(kIdA));

    // GAME THREAD: the owner-facing half runs here, while the owner is provably
    // alive; only the container half is queued, and it captures the id alone.
    rig.netSync.clearOwnerCallbacks<RegSimulatable>(&owner, /*authorityOwner=*/nullptr);
    queue.enqueue([&rig]()
    {
        unregisterSimulatableContainers<RegSimulatable>(
            rig.storage, rig.reconciliation, rig.resolution, rig.netSync, kIdA);
    });

    // STEP N — still fully visible.
    const RegResolvedInputs stepN = rig.resolution.collectInputAll(normalStep(400u));
    CHECK(rig.storage.has<RegSimulatable>(kIdA));
    CHECK(inputValueFor(stepN, kIdA) == 3);

    CHECK(queue.applyPending() == 1u);

    // STEP N+1 — gone, and gone WHOLE: no id in storage and nothing left behind
    // for the collect loop to trip over.
    const RegResolvedInputs stepNPlus1 = rig.resolution.collectInputAll(normalStep(401u));
    CHECK_FALSE(rig.storage.has<RegSimulatable>(kIdA));
    CHECK_FALSE(hasInputFor(stepNPlus1, kIdA));
    CHECK_FALSE(rig.resolution.isLocallyControlled<RegSimulatable>(kIdA));
    CHECK(rig.resolution.getDiagnostics().localInputCache<RegSimulatable>(kIdA) == nullptr);
    CHECK(rig.countLinesContaining("NOCACHE") == 0u);
}

TEST_CASE("TheDeferredQueueAppliesInEnqueueOrder", "[Registration][DeferredLifecycle]")
{
    DeferredLifecycleQueue queue;
    std::vector<int>       order;

    queue.enqueue([&order]() { order.push_back(1); });
    queue.enqueue([&order]() { order.push_back(2); });
    queue.enqueue([&order]() { order.push_back(3); });

    CHECK(queue.applyPending() == 3u);
    CHECK(order == std::vector<int>{ 1, 2, 3 });

    // Draining an empty queue is free and answers zero.
    CHECK(queue.applyPending() == 0u);
    CHECK_FALSE(queue.hasPending());
}

TEST_CASE("TheDeferredQueueMovesNonCopyablePayloads", "[Registration][DeferredLifecycle]")
{
    // The registration job owns the simulatable by value and `SimulatableBrawler`
    // is not copyable, so the queue must accept a MOVE-ONLY callable —
    // `std::function` cannot, which is why the queue owns typed job nodes.
    DeferredLifecycleQueue     queue;
    std::unique_ptr<int>       payload = std::make_unique<int>(42);
    int                        seen    = 0;

    queue.enqueue([p = std::move(payload), &seen]() { seen = *p; });

    CHECK(queue.applyPending() == 1u);
    CHECK(seen == 42);
}

#endif // WITH_LOW_LEVEL_TESTS
