// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/SimulationInputResolution.h"
#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SimulationFieldDescriptors.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/RelayedInputRingCodec.h"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// og-netcode-v2-input-relay / item 86 (step 2 of the input-resolution
// migration): SimulationInputResolutionTest.
//
// THE ACCEPTANCE CRITERION THIS FILE EXISTS TO SATISFY: construct and drive
// `SimulationInputResolution` with NO `SimulationNetSync` ANYWHERE IN THIS
// TRANSLATION UNIT — registration lifecycle, the by-id ingest doors, a REAL
// `SimulationReconciliation` instance (not a mock), and both prediction and
// resim resolution, all driven directly against the peer's own public
// surface. If the peer could not stand alone, this file would fail to
// compile or fail red — that is the point of it, not an incidental property.
//
// LOCAL isolation, same convention as StateCorrectionCache4MethodApiTest.cpp:
// a scratch mock simulatable only. og-simulation core must verify standalone;
// this file references no brawler / UE / owner types at all.
//
// FRONTIER-PAIR DISCIPLINE (item 84's detector). [item 94] `resolution.
// collectInputAll` (named `prepareSimulationStep` between item 90 and item
// 94) NO LONGER ALLOCATES — frontier allocation moved to
// `reconciliation.allocateFrontierSlotsAll`. Every call this file makes to
// `collectInputAll` on a step that should allocate a frontier slot is
// followed by `reconciliation.allocateFrontierSlotsAll` for the SAME step,
// which is in turn followed by `reconciliation.postPredictionAll` — mirroring
// what `SimulationManager::onGameSimulationPrediction` always does within one
// manager tick (collect -> allocate -> ... -> capture), exactly the pairing
// item 84's fixture repair (SimulationNetSyncTest.cpp, og-brawler-tests) had
// to add for the same reason. Skipping the allocate call before capture would
// abort the process on the NEXT allocation (`OG_CHECK` in CorrectionCache.h),
// not silently misbehave.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // --- Serializable state/input payloads ---------------------------------
    struct MockState
    {
        std::int32_t position = 0;

        // StateCorrectionCache::tryInsertingCorrectState compares slots via
        // `compare()` (SimulationSerialization.h), which prefers a member
        // isSimilarTo when one exists — this is cheaper than wrapping the
        // payload in a SimulationComposite<...> purely to inherit its fold,
        // and this file never calls compute_checksum/save_snapshot, so no
        // SerializableFields specialization is needed for the state type.
        bool isSimilarTo(const MockState& other) const { return position == other.position; }
    };

    struct MockInput
    {
        std::int32_t value = 0;
    };
} // namespace

// SerializableFields IS required for MockInput — RelayedInputRingCodec.h's
// writeLatest<InputType> derives the wire stride from the SIM_MEMBER
// machinery, and one case below drives the REAL ring-codec ingest path
// through `ingestRelayRing`.
template <>
struct SerializableFields<MockInput>
{
    static constexpr auto get()
    {
        return std::make_tuple(SIM_MEMBER(MockInput, value));
    }
};

namespace
{
    // --- Mock simulatable ----------------------------------------------------
    // Minimal SimulatableState-concept satisfier (SimulationObjectStorage.h):
    // StateType/InputType + getAllState/editAllState/updateVizState/getVizState.
    // No integrate() / firstResimStep(): this file never touches
    // SimulationIntegrationExecutor, so the mock does not need them.
    class MockAllState
    {
    public:
        const MockState& getState() const { return m_state; }
        MockState&       editState()       { return m_state; }
    private:
        MockState m_state;
    };

    struct MockSimulatable
    {
        using StateType = MockState;
        using InputType = MockInput;

        MockAllState m_allState;
        MockAllState m_vizState;

        const MockAllState& getAllState() const { return m_allState; }
        MockAllState&       editAllState()       { return m_allState; }
        void updateVizState() { m_vizState = m_allState; }
        const MockAllState& getVizState() const { return m_vizState; }
    };

    static_assert(SimulatableState<MockSimulatable>,
        "MockSimulatable must satisfy the concept SimulationObjectStorage/"
        "SimulationReconciliation require");

    using MockStorage       = SimulationObjectStorage<MockSimulatable>;
    using MockReconciliation = SimulationReconciliation<MockSimulatable>;
    using MockResolution     = SimulationInputResolution<MockSimulatable>;
    using MockResolvedInputs = ResolvedInputs<MockSimulatable>;

    constexpr unsigned int kLocalId  = 1u;
    constexpr unsigned int kRemoteId = 2u;
    constexpr unsigned int kAuthId   = 3u;
    constexpr float        kDeltaSeconds = 1.0f / 60.0f;

    SimulationTimeStep normalStep(std::uint32_t tick)
    {
        return SimulationTimeStep(tick, /*isResimulating=*/false, StepKind::Normal, kDeltaSeconds);
    }

    // --- The rig: storage + a REAL reconciliation + a REAL resolution peer ---
    // No SimulationNetSync anywhere in this file — this IS the acceptance
    // criterion, not incidental to it.
    struct ResolutionRig
    {
        MockStorage        storage;
        MockReconciliation reconciliation{ storage };
        MockResolution     resolution{ storage, reconciliation };
    };

    // A no-op input provider — collectInputAll's local-provider branch
    // only needs SOME callable of the right shape; most cases here care about
    // the registration/resolution plumbing, not the provider's own arithmetic.
    MockInput constantProvider(std::int32_t value, const SimulationTimeStep&,
                               const LocalInputCache<MockInput>&)
    {
        return MockInput{ value };
    }

    // --- Ring codec test buffer, same shape as WireFormat/RelayedInputRingTest.cpp's
    // RingTestBuffer — the std::vector-backed satisfier of RelayedInputRingCodec.h's
    // BUFFER CONCEPT (bundleByteNum / bundleAddZeroedBytes / writeToBuffer /
    // readFromBuffer / bundleTruncateTo).
    struct RingTestBuffer
    {
        std::vector<std::uint8_t> bytes;

        std::int32_t bundleByteNum() const { return static_cast<std::int32_t>(bytes.size()); }

        void bundleAddZeroedBytes(std::int32_t count)
        {
            bytes.resize(bytes.size() + static_cast<std::size_t>(count), 0u);
        }

        template <typename T>
        void writeToBuffer(std::uint32_t off, const T& value)
        {
            std::memcpy(bytes.data() + off, &value, sizeof(T));
        }

        template <typename T>
        T readFromBuffer(std::uint32_t off) const
        {
            T value;
            std::memcpy(&value, bytes.data() + off, sizeof(T));
            return value;
        }

        void bundleTruncateTo(std::int32_t byteCount)
        {
            if (byteCount < 0) byteCount = 0;
            if (static_cast<std::size_t>(byteCount) < bytes.size())
                bytes.resize(static_cast<std::size_t>(byteCount));
        }
    };

    // Minimal correction-STATE buffer satisfying the two operations
    // SimulationReconciliation::injectCorrectionState actually calls
    // (readInto / getAppliedCaptureTick) — enough to drive a REAL correction
    // landing with no owner/adapter/codec type involved.
    struct MockCorrectionBuffer
    {
        MockState state{};
        std::uint32_t tick = 0;
        std::uint32_t appliedCaptureTick = kNoInputCaptureTick;

        std::uint32_t readInto(MockState& out) const { out = state; return tick; }
        std::uint32_t getAppliedCaptureTick() const { return appliedCaptureTick; }
    };

    MockInput inputFor(const MockResolvedInputs& inputs, unsigned int id)
    {
        const auto& map = std::get<std::unordered_map<unsigned int, MockInput>>(inputs);
        const auto it = map.find(id);
        REQUIRE(it != map.end());
        return it->second;
    }

    bool hasInputFor(const MockResolvedInputs& inputs, unsigned int id)
    {
        const auto& map = std::get<std::unordered_map<unsigned int, MockInput>>(inputs);
        return map.find(id) != map.end();
    }
} // namespace

// ---------------------------------------------------------------------------
// Registration lifecycle + local-prediction collect (provider branch).
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.LocalCharacterCollectInputAllRunsTheProviderThenReconciliationAllocatesTheFrontierPair",
    "[InputResolution]")
{
    ResolutionRig rig;
    // collectInputAll/collectResimInputAll/allocateFrontierSlotsAll/postPredictionAll
    // all iterate m_storage.forEachSimulatable — the id must be present in
    // STORAGE, not just registered on the resolution/reconciliation peers, or
    // none of the per-character bodies this file exercises ever run for it.
    rig.storage.add<MockSimulatable>(kLocalId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kLocalId);
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(7, step, line);
        });

    REQUIRE(rig.resolution.isLocallyControlled<MockSimulatable>(kLocalId));

    const auto step = normalStep(10u);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(step);
    // [item 94] collectInputAll no longer opens the frontier pair itself —
    // allocation is a separate call on reconciliation now, mirroring
    // SimulationManager::onGameSimulationPrediction's collect -> allocate
    // sequence.
    rig.reconciliation.allocateFrontierSlotsAll(step);
    // Complete the frontier pair — see the file banner.
    rig.reconciliation.postPredictionAll(step);

    REQUIRE(hasInputFor(inputs, kLocalId));
    REQUIRE(inputFor(inputs, kLocalId).value == 7);

    // The tick push actually allocated a slot in a REAL reconciliation
    // instance — findInputCache is non-null and the ref for this tick is
    // NoRef (a slot exists, nothing has corrected it yet).
    const AppliedCaptureRef ref =
        rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kLocalId, 10u);
    REQUIRE(ref.kind == AppliedCaptureRefKind::NoRef);
}

// ---------------------------------------------------------------------------
// Remote (proxy) prediction collect — resolves via the scheduled-read ladder
// against a pre-populated relay store.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.RemoteCharacterCollectInputAllResolvesFromTheRelayStore",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);

    REQUIRE_FALSE(rig.resolution.isLocallyControlled<MockSimulatable>(kRemoteId));

    auto* store = rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId);
    REQUIRE(store != nullptr);
    // dA = 0 so the ladder's probe (tick - dLatest) lands exactly on tick 5.
    REQUIRE(store->push(5u, /*dA=*/0u, MockInput{ 42 }));

    const auto step = normalStep(5u);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(step);
    rig.reconciliation.allocateFrontierSlotsAll(step);
    rig.reconciliation.postPredictionAll(step);

    REQUIRE(hasInputFor(inputs, kRemoteId));
    REQUIRE(inputFor(inputs, kRemoteId).value == 42);
}

// ---------------------------------------------------------------------------
// Authority remote-move queue: queueRemoteMove door, non-underrun consumption,
// and the join-key accessor.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.QueueRemoteMoveThenCollectInputAllConsumesItAndRecordsTheJoinKey",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kAuthId, MockSimulatable{});
    rig.resolution.registerAuthorityCharacter<MockSimulatable>(kAuthId);

    REQUIRE(rig.resolution.getLastUsedCaptureTick<MockSimulatable>(kAuthId) == kNoInputCaptureTick);

    const QueueMoveResult queued = rig.resolution.queueRemoteMove<MockSimulatable>(
        kAuthId, /*captureTick=*/9u, MockInput{ 11 }, /*currentAuthorityTick=*/9u,
        /*rollbackWindowTicks=*/-1);
    REQUIRE(queued == QueueMoveResult::Enqueued);

    // The authority path never allocates a correction cache (design §A.2) —
    // NO createCacheFor call here, matching production's server-overload
    // registerSimulatable exactly. collectInputAll's remote-move branch
    // does not touch reconciliation at all, and [item 94]
    // `onGameSimulationAuthority` never calls `allocateFrontierSlotsAll`
    // either — no pair to complete.
    const auto step = normalStep(9u);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(step);

    REQUIRE(hasInputFor(inputs, kAuthId));
    REQUIRE(inputFor(inputs, kAuthId).value == 11);
    REQUIRE(rig.resolution.getLastUsedCaptureTick<MockSimulatable>(kAuthId) == 9u);
}

TEST_CASE("SimulationInputResolution.AuthorityQueueUnderrunSubstitutesTheInjectedNeutral",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kAuthId, MockSimulatable{});
    rig.resolution.registerAuthorityCharacter<MockSimulatable>(kAuthId);
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -1 });

    const auto step = normalStep(1u);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(step);

    REQUIRE(hasInputFor(inputs, kAuthId));
    REQUIRE(inputFor(inputs, kAuthId).value == -1);
    // Underrun -> sentinel join key, never the queued tick.
    REQUIRE(rig.resolution.getLastUsedCaptureTick<MockSimulatable>(kAuthId) == kNoInputCaptureTick);
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 92, RE-DISPOSITIONED AT ITEM 94] THE
// REGISTRATION-ORDERING WINDOW.
// ---------------------------------------------------------------------------
// SimulationNetSync.h's server `registerSimulatable` overload allocates NO
// correction cache for an authority id, ever (see `AuthorityQueueUnderrunSub
// stitutesTheInjectedNeutral` above — no `createCacheFor` call there either,
// matching production exactly). Pre-item-92-fix, that overload called
// `storage.add` BEFORE `registerAuthorityOwner` (-> `registerAuthorityCharacter`,
// the ONLY call that populates `queueMap`) — a window during which a
// concurrent physics tick's frontier-allocation sweep would see the id in
// storage, miss it in `queueMap`, and fall through to `pushPredictionTick`
// for an id that will NEVER have a cache. `SimulationReconciliation::
// getCacheFor`'s bare `.at(id)` then threw — the exact crash captured in
// `runs/t91_hardresync_pie/client3_probe.log:86..95`.
//
// [item 94] ⚠ THE FAILURE MODE THIS WINDOW PRODUCES HAS CHANGED, THE WINDOW
// HAS NOT. Frontier allocation moved to `SimulationReconciliation::
// allocateFrontierSlotsAll`, which no longer reads `queueMap` at all — it
// filters on `findInputCache<T>(id) != nullptr` directly, and item 92's loud
// `OG_CHECK` guard is DELETED along with the resolution-side sweep that used
// to carry it (traded away, priced at `allocateFrontierSlotsAll`'s own
// banner). So this same window (an authority id exposed to storage before it
// has a cache — no cache ever, on this role) is now a SILENT SKIP, not a
// crash: `findInputCache` answers nullptr and the sweep moves on. The
// registration-ordering invariant (`registerAuthorityOwner` before
// `storage.add`) is STILL worth keeping — it protects sweep 1's own
// `queueMap`-based branch dispatch in `collectInputForCharacter`, a
// DIFFERENT correctness property than the crash this window used to
// threaten — but this specific case's file-banner framing ("the precondition
// the guard tests") no longer applies: there is no guard left to test the
// precondition of. What survives is the PRECONDITION ITSELF, now read as
// "the exact state the new nullable filter must correctly skip".
//
// This file has no `SimulationNetSync` (see the `ResolutionRig` comment
// above), so it cannot drive the free-function facade's ordering directly —
// it drives the SAME window at the core level instead, by calling
// `storage.add` and `resolution.registerAuthorityCharacter` in each order in
// turn.
//
// RED/GREEN (quoted in impl/impl_notes_task92.md, for the item-92 guard this
// window originally motivated): with the loud-failure guard temporarily
// reverted AND a scratch case added that actually called
// `rig.resolution.prepareSimulationStep(step)` on this exact window state,
// the case went RED — Catch2 reported an uncaught `std::out_of_range`
// escaping the test, the same exception class the production crash's SEH
// translation (`0xe06d7363`) wraps. [item 94] That RED/GREEN pinned the
// GUARD, which is now gone; the case below instead pins the FILTER'S
// precondition directly, and is verified against today's code by the second
// case's clean pass through the real `allocateFrontierSlotsAll` call.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.RegistrationWindowLeavesAnAuthorityIdWithNoQueueEntryAndNoCache",
    "[InputResolution]")
{
    constexpr unsigned int kWindowId = 41u;
    ResolutionRig rig;

    // Mirrors the pre-fix server overload's step 1 (storage.add) WITHOUT its
    // step 3 (registerAuthorityOwner -> registerAuthorityCharacter) having run
    // yet — the exact mid-registration window. No createCacheFor either,
    // matching the server overload's permanent design (authority ids never
    // get a cache, fixed or not).
    rig.storage.add<MockSimulatable>(kWindowId, MockSimulatable{});

    // [item 94] THE PRECONDITION `allocateFrontierSlotsAll`'s nullable filter
    // now exploits directly: no correction cache exists for this id.
    // (queueMap also misses it, by construction — registerAuthorityCharacter,
    // the only thing that inserts into queueMap, was never called — but the
    // new sweep no longer reads queueMap at all, so only the cache absence is
    // load-bearing here.) This is exactly the state that made pre-94's
    // pushPredictionTick's getCacheFor(id).at(id) throw, and is now the state
    // the filter silently skips.
    REQUIRE(rig.reconciliation.findInputCache<MockSimulatable>(kWindowId) == nullptr);
}

// The other side of the same window: the FIXED ordering (registerAuthority
// Character before storage.add — SimulationNetSync.h's server overload now
// does this) never produces the state above, and both collectInputAll and
// the separate allocateFrontierSlotsAll call run clean for a
// freshly-registered authority id — the AC's "(a) a storage-exposed id with
// no cache is swept by BOTH sweeps without crash, without allocation, without
// a detector fire" case.
TEST_CASE("SimulationInputResolution.AuthorityCharacterRegisteredBeforeStorageExposureAllocatesNoFrontierSlotSafely",
    "[InputResolution]")
{
    constexpr unsigned int kOrderedId = 42u;
    ResolutionRig rig;

    // Fixed order: registerAuthorityCharacter (populates queueMap) BEFORE
    // storage.add exposes the id to forEachSimulatable — mirrors item 92's
    // reordering of SimulationNetSync.h's server registerSimulatable overload.
    rig.resolution.registerAuthorityCharacter<MockSimulatable>(kOrderedId);
    rig.storage.add<MockSimulatable>(kOrderedId, MockSimulatable{});
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -1 });

    const auto step = normalStep(50u);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(step);
    // [item 94] BOTH sweeps now, explicitly: collect (above) and the
    // separate reconciliation call (below) — if the ordering fix or
    // `allocateFrontierSlotsAll`'s nullable filter regressed, this line would
    // throw instead of silently skipping. No postPredictionAll call: mirrors
    // production exactly — it is a prediction-role-only call
    // (onGameSimulationAuthority never makes either call, see
    // SimulationManager.h), and this id has no cache to complete a pair in
    // anyway (server overload's permanent design, unchanged by this fix).
    rig.reconciliation.allocateFrontierSlotsAll(step);

    REQUIRE(hasInputFor(inputs, kOrderedId));
    REQUIRE(inputFor(inputs, kOrderedId).value == -1); // underrun -> injected neutral
    // [item 94] The nullable-filter skip still holds: no frontier slot opened.
    const AppliedCaptureRef ref =
        rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kOrderedId, 50u);
    REQUIRE(ref.kind == AppliedCaptureRefKind::NoSlot);
    // "Without a detector fire" (AC (a)'s third clause) is true by
    // construction, not merely by observation: `m_frontierSlotAwaitingState`
    // is a per-CACHE bit (CorrectionCache.h), and this id has no cache at all
    // (see above) — there is no bit here that could have fired, and no
    // accessor to probe one through. The absence of a cache is itself the
    // proof; `allocateFrontierSlotsAll` returning without throwing (the line
    // above) is what confirms the filter reached that conclusion safely.
    REQUIRE(rig.reconciliation.findInputCache<MockSimulatable>(kOrderedId) == nullptr);
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 93, RE-DISPOSITIONED AT ITEM 94] THE
// UNREGISTRATION-ORDERING WINDOW — item 92's MIRROR.
// ---------------------------------------------------------------------------
// SimulationNetSync.h's free-function `unregisterSimulatable` facade used to
// call `netSync.unregisterSimulatable` (-> `SimulationInputResolution::
// unregisterCharacter`, which erases `queueMap`) BEFORE `storage.remove` — a
// window in which a concurrent physics tick's frontier-allocation sweep would
// see the id STILL in storage (still visible to forEachSimulatable) but no
// longer in `queueMap`, fall through the guard, and call `pushPredictionTick`
// for an id that (server overload's permanent design) has NEVER had a
// correction cache. This is the exact same precondition item 92 fixed on the
// REGISTRATION side, reached from the opposite direction: publish-last on
// the way in (item 92) means unpublish-first on the way out (item 93) —
// storage.remove must run BEFORE the queueMap/telemetry teardown, not after.
// ⚠ [item 94, Part F] THAT REORDER IS **STILL LANDED AND MUST NOT BE
// "SIMPLIFIED" BACK.** This task's own existence (a nullable, storage-driven
// allocation filter that no longer falls through to a throwing `.at(id)` at
// all) removes the reorder's ORIGINAL stated motivation — but that makes item
// 93's reorder BELT-AND-BRACES, not unnecessary: it still protects sweep 1's
// `queueMap`-based branch dispatch in `collectInputForCharacter` (a
// storage-exposed-but-not-yet-queueMap'd id would misclassify as a simulated
// proxy rather than an authority id, a real if lower-severity defect this
// task does not touch), and removing it would have zero upside and one real
// downside.
//
// This file has no `SimulationNetSync` (see the `ResolutionRig` comment
// above), so — same as item 92's pair above — it drives the SAME window at
// the core level: calling `resolution.unregisterCharacter` and
// `storage.remove` in each order in turn, rather than the free-function
// facade directly.
//
// [item 94] ⚠ THE FAILURE MODE THIS WINDOW PRODUCES HAS CHANGED, THE SAME WAY
// item 92's pair's did (see that section's own item-94 paragraph): the
// nullable `findInputCache` filter on `allocateFrontierSlotsAll` silently
// skips this window's exposed-but-cache-less id instead of falling through to
// a throwing `.at(id)` — no guard left to test the precondition of, and the
// case below pins the precondition itself instead.
//
// RED/GREEN (quoted in impl/impl_notes_task93.md, for the item-92/93 guard
// this window originally motivated): with the guard temporarily reverted AND
// a scratch case added that drove `prepareSimulationStep` on this exact
// window state (id unregistered from `queueMap` but still in storage — the
// OLD facade order), the case went RED — the same `std::out_of_range` class
// ("invalid unordered_map<K, T> key") item 92's RED demonstration produced.
// [item 94] That RED/GREEN pinned the GUARD, which is now gone; the two cases
// below instead pin the FILTER's precondition and its clean, silent pass —
// this IS the AC's "(b)"-shaped case for the registration direction (the
// authority-role half; the client-role, cache-outlives-storage half is a NEW
// case further below, immediately after the two-sweep-agreement suite).
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.UnregistrationWindowLeavesAnAuthorityIdInStorageWithNoQueueEntryAndNoCache",
    "[InputResolution]")
{
    constexpr unsigned int kUnregWindowId = 43u;
    ResolutionRig rig;

    // Fully registered, fixed order (item 92): queueMap populated before
    // storage exposes the id. No cache — matches the server overload's
    // permanent "authority ids never get a cache" design.
    rig.resolution.registerAuthorityCharacter<MockSimulatable>(kUnregWindowId);
    rig.storage.add<MockSimulatable>(kUnregWindowId, MockSimulatable{});

    // Mirrors the OLD (pre-fix) free-function facade's step order: the
    // queueMap/telemetry teardown (unregisterCharacter) ran BEFORE
    // storage.remove. The id is still visible to forEachSimulatable here.
    rig.resolution.unregisterCharacter<MockSimulatable>(kUnregWindowId);

    // [item 94] THE PRECONDITION `allocateFrontierSlotsAll`'s nullable filter
    // now exploits directly: still in storage (so the sweep would visit it)
    // and no correction cache (server overload never allocates one) —
    // together, exactly the state that made pre-94's pushPredictionTick's
    // getCacheFor(id).at(id) throw, and is now the state the filter silently
    // skips. (queueMap no longer matters to this sweep at all — only listed
    // here as the historical trigger.)
    REQUIRE(rig.storage.has<MockSimulatable>(kUnregWindowId));
    REQUIRE(rig.reconciliation.findInputCache<MockSimulatable>(kUnregWindowId) == nullptr);
}

// The other side of the same window: the FIXED ordering (storage.remove
// BEFORE the queueMap/telemetry teardown — SimulationNetSync.h's
// unregisterSimulatable free-function facade now does this) removes the id
// from storage before anything erases queueMap, so it is never visible to
// forEachSimulatable during teardown at all — neither to collectInputAll nor
// to the separate allocateFrontierSlotsAll call.
TEST_CASE("SimulationInputResolution.StorageRemovedBeforeQueueEntryErasureIsInvisibleToCollectInputAllAndAllocation",
    "[InputResolution]")
{
    constexpr unsigned int kUnregOrderedId = 44u;
    ResolutionRig rig;

    rig.resolution.registerAuthorityCharacter<MockSimulatable>(kUnregOrderedId);
    rig.storage.add<MockSimulatable>(kUnregOrderedId, MockSimulatable{});
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -1 });

    // Fixed order: storage.remove FIRST — mirrors item 93's reordering of
    // SimulationNetSync.h's unregisterSimulatable free-function facade.
    rig.storage.remove<MockSimulatable>(kUnregOrderedId);
    REQUIRE_FALSE(rig.storage.has<MockSimulatable>(kUnregOrderedId));

    // queueMap still has the id at this point (unregisterCharacter has not
    // run yet) — but that no longer matters: collectInputAll and
    // allocateFrontierSlotsAll are BOTH storage-driven
    // (m_storage.forEachSimulatable), so an id storage no longer has is
    // simply never visited by either, regardless of what queueMap/cache say
    // about it. [item 94] Runs BOTH calls explicitly now — if this ordering
    // regressed, either line could throw or abort.
    const auto step = normalStep(60u);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(step);
    REQUIRE_FALSE(hasInputFor(inputs, kUnregOrderedId));
    rig.reconciliation.allocateFrontierSlotsAll(step);
    REQUIRE(rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kUnregOrderedId, 60u).kind
        == AppliedCaptureRefKind::NoSlot);

    // Complete the teardown, matching the fixed facade's remaining two
    // steps, so the rig is left consistent (not load-bearing for the
    // assertions above, but avoids leaving a half-torn-down id in scope).
    rig.resolution.unregisterCharacter<MockSimulatable>(kUnregOrderedId);
}

// ---------------------------------------------------------------------------
// design §C.3 — the by-id doors' benign-lookup-miss behaviour, replacing the
// pre-cut captured-reference dangling-UB hazard.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.QueueRemoteMoveOnAnUnregisteredIdIsABenignLookupMiss",
    "[InputResolution]")
{
    ResolutionRig rig;
    // kAuthId was never registered at all.
    const QueueMoveResult result = rig.resolution.queueRemoteMove<MockSimulatable>(
        kAuthId, 3u, MockInput{ 1 }, 3u, -1);
    REQUIRE(result == QueueMoveResult::IdNotRegistered);
}

TEST_CASE("SimulationInputResolution.IngestRelayRingOnAnUnregisteredIdIsABenignLookupMiss",
    "[InputResolution]")
{
    ResolutionRig rig;
    RingTestBuffer ring; // never written — the lookup miss returns before touching it.
    const RelayedInputIngestReport report =
        rig.resolution.ingestRelayRing<MockSimulatable>(kRemoteId, ring);
    REQUIRE(report.outcome == RelayedInputIngestOutcome::NeverWritten);
}

// ---------------------------------------------------------------------------
// The real ring-codec ingest path — proves ingestRelayRing actually consumes
// a wire-shaped ring, not just a by-id lookup.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.IngestRelayRingConsumesARealRingIntoTheStore",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);

    RingTestBuffer ring;
    REQUIRE(relayedInputRing::writeLatest<MockInput>(ring, /*captureTick=*/20u, /*dA=*/2u,
        MockInput{ 99 }, /*depth=*/1));

    const RelayedInputIngestReport report =
        rig.resolution.ingestRelayRing<MockSimulatable>(kRemoteId, ring);
    REQUIRE(report.outcome == RelayedInputIngestOutcome::Consumed);
    REQUIRE(report.newestCaptureTickValid);
    REQUIRE(report.newestCaptureTick == 20u);

    auto* store = rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId);
    REQUIRE(store != nullptr);
    REQUIRE(store->has(20u));

    const auto latest = rig.resolution.getLastRelayedInput<MockSimulatable>(kRemoteId);
    REQUIRE(latest.has_value());
    REQUIRE(latest->value == 99);
}

// ---------------------------------------------------------------------------
// collectResimInputAll — the resolution table's rungs, driven against a REAL
// reconciliation instance (no NetSync anywhere in this TU).
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.ResimNoSlotForAnIdWithNoCorrectionCache",
    "[InputResolution]")
{
    ResolutionRig rig;
    // Nothing registered at all for kLocalId.
    const MockResolvedInputs inputs = rig.resolution.collectResimInputAll(5u);
    REQUIRE_FALSE(hasInputFor(inputs, kLocalId));
}

TEST_CASE("SimulationInputResolution.ResimLocalRefRungReplaysTheDelayLineEntryACorrectionNamed",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kLocalId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kLocalId);
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(5, step, line);
        });

    const auto step = normalStep(3u);
    rig.resolution.collectInputAll(step);             // delay line now holds tick 3's capture
    rig.reconciliation.allocateFrontierSlotsAll(step); // [item 94] opens the frontier pair
    rig.reconciliation.postPredictionAll(step);       // complete the frontier pair

    // Before any correction lands, the resim ref is NoRef.
    {
        const MockResolvedInputs resim = rig.resolution.collectResimInputAll(3u);
        REQUIRE(hasInputFor(resim, kLocalId)); // NoRef/local re-derives from the delay line too
    }

    // Land a REAL correction naming tick 3 as the applied capture — driven
    // directly against reconciliation, no NetSync involved.
    MockCorrectionBuffer buffer;
    buffer.state = MockState{ 100 };
    buffer.tick = 3u;
    buffer.appliedCaptureTick = 3u;
    rig.reconciliation.injectCorrectionState<MockSimulatable>(kLocalId, buffer);

    const AppliedCaptureRef ref =
        rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kLocalId, 3u);
    REQUIRE(ref.kind == AppliedCaptureRefKind::Ref);
    REQUIRE(ref.captureTick == 3u);

    const MockResolvedInputs resimAfter = rig.resolution.collectResimInputAll(3u);
    REQUIRE(hasInputFor(resimAfter, kLocalId));
    REQUIRE(inputFor(resimAfter, kLocalId).value == 5); // replayed from the delay line, not re-derived
}

TEST_CASE("SimulationInputResolution.ResimSentinelRungResolvesToTheInjectedNeutralForBothCharacterClasses",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kLocalId, MockSimulatable{});
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -7 });

    rig.reconciliation.createCacheFor<MockSimulatable>(kLocalId);
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(1, step, line);
        });
    const auto step = normalStep(4u);
    rig.resolution.collectInputAll(step);
    rig.reconciliation.allocateFrontierSlotsAll(step); // [item 94] opens the frontier pair
    rig.reconciliation.postPredictionAll(step);

    // A correction landing with the SENTINEL applied-capture-tick — the
    // authority substituted an input, named no real capture.
    MockCorrectionBuffer buffer;
    buffer.state = MockState{ 0 };
    buffer.tick = 4u;
    buffer.appliedCaptureTick = kNoInputCaptureTick;
    rig.reconciliation.injectCorrectionState<MockSimulatable>(kLocalId, buffer);

    const AppliedCaptureRef ref =
        rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kLocalId, 4u);
    REQUIRE(ref.kind == AppliedCaptureRefKind::Sentinel);

    const MockResolvedInputs resim = rig.resolution.collectResimInputAll(4u);
    REQUIRE(hasInputFor(resim, kLocalId));
    REQUIRE(inputFor(resim, kLocalId).value == -7);
}

TEST_CASE("SimulationInputResolution.ResimRemoteNoStoreRungResolvesToTheNeutral",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ -3 });
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    // Deliberately no registerRemoteCharacter — a cache exists but no relay
    // store does, exercising the REMOTE/NoStore rung directly. Allocate the
    // frontier slot manually, the way collectInputForCharacter's proxy branch
    // would (this id has no provider and no relay store), then complete the
    // pair — same discipline as every other case in this file.
    const auto step = normalStep(2u);
    rig.reconciliation.pushPredictionTick<MockSimulatable>(kRemoteId, 2u);
    rig.reconciliation.postPredictionAll(step);

    const AppliedCaptureRef ref =
        rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kRemoteId, 2u);
    REQUIRE(ref.kind == AppliedCaptureRefKind::NoRef);

    const MockResolvedInputs resim = rig.resolution.collectResimInputAll(2u);
    REQUIRE(hasInputFor(resim, kRemoteId));
    REQUIRE(inputFor(resim, kRemoteId).value == -3);
}

TEST_CASE("SimulationInputResolution.ResimRemoteRefRungReplaysFromTheStoreWithSelfHealOnMiss",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kRemoteId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kRemoteId);
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);
    auto* store = rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId);
    REQUIRE(store != nullptr);
    REQUIRE(store->push(6u, /*dA=*/0u, MockInput{ 55 }));

    rig.reconciliation.pushPredictionTick<MockSimulatable>(kRemoteId, 6u);
    const auto step = normalStep(6u);
    rig.reconciliation.postPredictionAll(step);

    MockCorrectionBuffer buffer;
    buffer.state = MockState{ 0 };
    buffer.tick = 6u;
    buffer.appliedCaptureTick = 6u; // names a capture tick THIS store has resident
    rig.reconciliation.injectCorrectionState<MockSimulatable>(kRemoteId, buffer);

    const MockResolvedInputs resim = rig.resolution.collectResimInputAll(6u);
    REQUIRE(hasInputFor(resim, kRemoteId));
    REQUIRE(inputFor(resim, kRemoteId).value == 55);
}

// ---------------------------------------------------------------------------
// wipeAllForResync — the celebrated non-wipe: pending + local wiped, remote
// store retained.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.WipeAllForResyncWipesPendingAndLocalButNotTheRemoteStore",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.storage.add<MockSimulatable>(kLocalId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kLocalId);
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(3, step, line);
        });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);
    auto* store = rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId);
    REQUIRE(store != nullptr);
    REQUIRE(store->push(1u, 0u, MockInput{ 8 }));

    const auto step = normalStep(1u);
    rig.resolution.collectInputAll(step); // populates the local delay line + pending queue
    rig.reconciliation.allocateFrontierSlotsAll(step); // [item 94] opens the frontier pair
    rig.reconciliation.postPredictionAll(step);

    auto* pending = rig.resolution.findPendingInputQueue<MockSimulatable>(kLocalId);
    REQUIRE(pending != nullptr);
    REQUIRE_FALSE(pending->empty()); // collectInputAll enqueued tick 1's capture

    rig.resolution.wipeAllForResync(0u);

    // The pending queue was wiped — directly observable via the same door.
    REQUIRE(pending->empty());

    // The remote store survives — the design's deliberate non-wipe.
    REQUIRE(store->has(1u));
}

// ---------------------------------------------------------------------------
// unregisterCharacter — erases all five container families' entries plus the
// join key.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.UnregisterCharacterErasesEveryContainerEntryAndTheJoinKey",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.resolution.registerAuthorityCharacter<MockSimulatable>(kAuthId);
    rig.resolution.queueRemoteMove<MockSimulatable>(kAuthId, 2u, MockInput{ 1 }, 2u, -1);
    REQUIRE(rig.resolution.getLastUsedCaptureTick<MockSimulatable>(kAuthId) == kNoInputCaptureTick);

    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);
    REQUIRE(rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId) != nullptr);

    rig.resolution.unregisterCharacter<MockSimulatable>(kAuthId);
    rig.resolution.unregisterCharacter<MockSimulatable>(kRemoteId);

    // The queue is gone — a subsequent remote move is a benign lookup miss.
    REQUIRE(rig.resolution.queueRemoteMove<MockSimulatable>(kAuthId, 2u, MockInput{ 1 }, 2u, -1)
        == QueueMoveResult::IdNotRegistered);
    REQUIRE(rig.resolution.getLastUsedCaptureTick<MockSimulatable>(kAuthId) == kNoInputCaptureTick);
    REQUIRE(rig.resolution.findRemoteInputCache<MockSimulatable>(kRemoteId) == nullptr);
}

// ---------------------------------------------------------------------------
// setNeutralInput / hasNeutralInput / getNeutralInput.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.SetNeutralInputIsVisibleThroughHasAndGetNeutralInput",
    "[InputResolution]")
{
    ResolutionRig rig;
    REQUIRE_FALSE(rig.resolution.hasNeutralInput<MockSimulatable>());

    rig.resolution.setNeutralInput<MockSimulatable>(MockInput{ 21 });

    REQUIRE(rig.resolution.hasNeutralInput<MockSimulatable>());
    REQUIRE(rig.resolution.getNeutralInput<MockSimulatable>().value == 21);
}

// ---------------------------------------------------------------------------
// findPendingInputQueue — the send-side by-id door, nullable, and populated
// only for provider-owning (local) ids.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.FindPendingInputQueueIsPopulatedForLocalCharactersOnly",
    "[InputResolution]")
{
    ResolutionRig rig;
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(2, step, line);
        });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteId);

    REQUIRE(rig.resolution.findPendingInputQueue<MockSimulatable>(kLocalId) != nullptr);
    REQUIRE(rig.resolution.findPendingInputQueue<MockSimulatable>(kRemoteId) == nullptr);
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 90, RE-POINTED AT ITEM 94] TWO-CALL
// AGREEMENT (the item-90 LLT, re-pointed rather than renamed away: the
// property it pins survives the relocation intact).
//
// Pre-94: `prepareSimulationStep` resolved every character's input in sweep 1
// and allocated the frontier for every prediction-owned id in sweep 2 — two
// separate `forEachSimulatable` passes inside ONE call. Post-94: `collectInputAll`
// resolves (one pass) and `reconciliation.allocateFrontierSlotsAll` allocates
// (a second pass, on a DIFFERENT class, called separately) — mirroring
// `SimulationManager::onGameSimulationPrediction`'s own collect-then-allocate
// sequence exactly. This is the pin that the two calls stay in agreement:
// from one collect + one allocate, every registered prediction-owned id
// (local-provider AND simulated-proxy alike) gets BOTH a resolved input AND
// its frontier advanced exactly once, and the StepKind matrix that gates
// allocation (Normal/Skip allocate, Stall does not) behaves exactly as it did
// before either relocation.
//
// "Frontier advanced" is observed the same way
// LocalCharacterCollectInputAllRunsTheProviderThenReconciliationAllocatesTheFrontierPair
// already does above: `getAppliedCaptureTickRefKind` answers `NoRef` (a slot
// exists, uncorrected) once a tick has been pushed and `NoSlot` (the
// `AppliedCaptureRef{}` default) if it never was — `getCacheIndex` rejects a
// tick outside the ring's allocated window. "Exactly once" is enforced
// structurally rather than counted: `allocateFrontierSlotsAll` visits each id
// exactly once per `forEachSimulatable` pass, and a SECOND `pushPredictionTick`
// for the same id in the same call would trip item 84's `OG_CHECK` in
// `StateCorrectionCache::pushPredictionTick` and abort the process — so a
// case that runs to completion and passes its `postPredictionAll` pairing
// discipline (the file banner) has already proven at-most-once by not
// crashing; these cases prove at-least-once with the `NoRef` checks below.
//
// ⛔ RED/GREEN, RE-DEMONSTRATED AT ITEM 94 (quoted in impl/impl_notes_task94.md):
// with the `reconciliation.allocateFrontierSlotsAll(step)` call commented out
// of the Normal-step case below — the direct analogue of the manager's own
// allocate call — every `AppliedCaptureRefKind::NoRef` REQUIRE goes RED (the
// query answers `NoSlot` instead). Restored, every case is GREEN.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.CollectThenAllocateNormalStepAdvancesEveryPredictionOwnedIdsFrontierExactlyOnce",
    "[InputResolution]")
{
    constexpr unsigned int kLocalIdA  = 11u;
    constexpr unsigned int kLocalIdB  = 12u;
    constexpr unsigned int kRemoteIdA = 21u;
    constexpr unsigned int kRemoteIdB = 22u;
    const unsigned int allIds[] = { kLocalIdA, kLocalIdB, kRemoteIdA, kRemoteIdB };

    ResolutionRig rig;
    for (unsigned int id : allIds)
    {
        rig.storage.add<MockSimulatable>(id, MockSimulatable{});
        rig.reconciliation.createCacheFor<MockSimulatable>(id);
    }
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalIdA,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(101, step, line);
        });
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalIdB,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(102, step, line);
        });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteIdA);
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteIdB);

    const auto step = normalStep(50u);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(step);
    // [item 94] The manager's own collect -> allocate sequence, reproduced
    // here explicitly rather than folded into one call.
    rig.reconciliation.allocateFrontierSlotsAll(step);
    // Complete the frontier pair for all four ids — file banner discipline.
    rig.reconciliation.postPredictionAll(step);

    for (unsigned int id : allIds)
    {
        INFO("id=" << id);
        REQUIRE(hasInputFor(inputs, id));

        const AppliedCaptureRef ref =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 50u);
        REQUIRE(ref.kind == AppliedCaptureRefKind::NoRef);
    }
}

TEST_CASE("SimulationInputResolution.CollectThenAllocateSkipStepBackfillsAndAdvancesEveryPredictionOwnedIdsFrontierExactlyOnce",
    "[InputResolution]")
{
    constexpr unsigned int kLocalIdA  = 11u;
    constexpr unsigned int kRemoteIdA = 21u;
    const unsigned int allIds[] = { kLocalIdA, kRemoteIdA };

    ResolutionRig rig;
    for (unsigned int id : allIds)
    {
        rig.storage.add<MockSimulatable>(id, MockSimulatable{});
        rig.reconciliation.createCacheFor<MockSimulatable>(id);
    }
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalIdA,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(201, step, line);
        });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteIdA);

    // Establish tick 9 as the last completed tick, same as every other case's
    // opening Normal step.
    const auto firstStep = normalStep(9u);
    rig.resolution.collectInputAll(firstStep);
    rig.reconciliation.allocateFrontierSlotsAll(firstStep);
    rig.reconciliation.postPredictionAll(firstStep);

    // A Skip step landing at tick 12 (StepKind::Skip: "sim tick jumps by >1;
    // previous tick must be back-filled" — SimulationTimeContext.h).
    // `allocateFrontierSlotsAll` must both backfill tick 11
    // (step.getTick() - 1, today's single-tick backfill — tick 10 is a
    // genuine, documented gap, unchanged by this task) AND push the new
    // frontier tick 12, from the SAME call.
    const SimulationTimeStep skipStep(12u, /*isResimulating=*/false, StepKind::Skip, kDeltaSeconds);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(skipStep);
    rig.reconciliation.allocateFrontierSlotsAll(skipStep);
    rig.reconciliation.postPredictionAll(skipStep);

    for (unsigned int id : allIds)
    {
        INFO("id=" << id);
        REQUIRE(hasInputFor(inputs, id));

        const AppliedCaptureRef frontierRef =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 12u);
        REQUIRE(frontierRef.kind == AppliedCaptureRefKind::NoRef);

        const AppliedCaptureRef backfilledRef =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 11u);
        REQUIRE(backfilledRef.kind == AppliedCaptureRefKind::NoRef);

        // The gap tick — today's documented single-tick backfill limit,
        // unchanged by this task's restructuring.
        const AppliedCaptureRef gapRef =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 10u);
        REQUIRE(gapRef.kind == AppliedCaptureRefKind::NoSlot);
    }
}

TEST_CASE("SimulationInputResolution.CollectThenAllocateStallStepResolvesInputButAllocatesNoFrontierSlot",
    "[InputResolution]")
{
    constexpr unsigned int kLocalIdA  = 11u;
    constexpr unsigned int kRemoteIdA = 21u;
    const unsigned int allIds[] = { kLocalIdA, kRemoteIdA };

    ResolutionRig rig;
    for (unsigned int id : allIds)
    {
        rig.storage.add<MockSimulatable>(id, MockSimulatable{});
        rig.reconciliation.createCacheFor<MockSimulatable>(id);
    }
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalIdA,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(301, step, line);
        });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kRemoteIdA);

    // A fresh rig, no prior Normal step: tick 5 has never been allocated, so
    // if `allocateFrontierSlotsAll` (wrongly) allocated on Stall, tick 5
    // would read NoRef; if it (correctly) does not, tick 5 stays NoSlot.
    // `stepAllocatesFrontierSlot` is `kind != StepKind::Stall` — this is the
    // one StepKind the predicate answers false for.
    const SimulationTimeStep stallStep(5u, /*isResimulating=*/false, StepKind::Stall, kDeltaSeconds);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(stallStep);
    // [item 94] Called explicitly, even on Stall — proving the ALLOCATION
    // CALL ITSELF is a safe no-op on this StepKind (gated by the predicate
    // inside), not merely that the test declined to call it.
    rig.reconciliation.allocateFrontierSlotsAll(stallStep);
    // No postPredictionAll call — item 84's pair never opened on a Stall
    // step, so there is nothing to complete (postPredictionAll's own early
    // return would make the call a no-op anyway).

    for (unsigned int id : allIds)
    {
        INFO("id=" << id);
        // collectInputAll still resolves an input — resolution is
        // unconditional on StepKind; only frontier allocation is gated.
        REQUIRE(hasInputFor(inputs, id));

        const AppliedCaptureRef ref =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 5u);
        REQUIRE(ref.kind == AppliedCaptureRefKind::NoSlot);
    }
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 91 part I2] SWEEP 1 THROWING PARTWAY — RETIRED
// AT ITEM 94. GRAVESTONE, NOT SILENTLY DELETED.
// ---------------------------------------------------------------------------
// This case used to pin: post-item-90, sweep 2 (frontier allocation) ran only
// after sweep 1's (resolution's) ENTIRE `forEachSimulatable` returned without
// throwing, INSIDE ONE FUNCTION (`prepareSimulationStep`) — so a throwing
// provider for one id left EVERY registered id, including ones whose own
// sweep-1 work had already completed, with NO frontier slot allocated for
// that tick, a state pre-item-90's per-character-paired shape could not
// produce.
//
// [item 94] THE PROPERTY THIS PINNED NO LONGER EXISTS AT THIS LAYER, BECAUSE
// THE BOUNDARY IT PINNED NO LONGER EXISTS HERE. `collectInputAll` (this
// class's method) does not allocate anything at all any more, throw or no
// throw — the boundary between "resolve" and "allocate" moved from an
// internal two-sweep split inside one function to two ordinary, separate
// statements in `SimulationManager::onGameSimulationPrediction`
// (`m_inputResolution.collectInputAll(step);` followed by
// `m_reconciliation.allocateFrontierSlotsAll(step);`). Re-deriving this case
// against `collectInputAll` alone would be a test that PASSES REGARDLESS OF
// WHETHER THE PROVIDER THROWS — this class structurally cannot allocate a
// frontier slot any more, so the assertion "no frontier slot allocated"
// holds unconditionally and proves nothing: exactly the "test which cannot
// fail" class item 94's own dispatch forbids leaving behind (Part F, applied
// here on the implementer's own initiative to a case Part F did not name).
//
// THE DECISION ITSELF (accepted as documented debt: this path is reachable
// only via an already-broken registration invariant, a second bug required)
// CARRIES FORWARD UNCHANGED and is now stated where the two statements it
// concerns actually sit: `SimulationManager.h`'s `onGameSimulationPrediction`,
// at the sweep-boundary fence between `collectInputAll` and
// `allocateFrontierSlotsAll`. The REACHABILITY argument (one of
// `collectInputForCharacter`'s two remaining `.at(id)` throwing lookups —
// see that method's own corrected-attribution comment, 91-I's misattribution
// fix folded in by this task) is unchanged and still lives there too. An
// equivalent manager-level regression test would need a full
// `SimulationManager` rig with a throwing mock resolution peer; not written
// here, because the property it would prove — "an uncaught C++ exception
// skips the following statement" — is ordinary language guarantee, not a
// custom mechanism this codebase has otherwise felt the need to
// regress-test, and no manager-level LLT rig for this exists elsewhere in
// this file's family to extend cheaply.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 91 part I3, RE-DISPOSITIONED AT ITEM 94]
// MIXED AUTHORITY + PREDICTION-OWNED POPULATION — the three collect-then-
// allocate cases above register only prediction-owned ids (local-provider /
// simulated-proxy), never a true authority id (`registerAuthorityCharacter`,
// the remote-move-queue branch). None of them directly re-proves that
// `allocateFrontierSlotsAll`'s nullable filter correctly excludes
// authority-owned ids when mixed with prediction-owned ids in the SAME call
// — this case does.
//
// [item 94] ⚠ THE SETUP INVERTS FROM ITS PRE-94 SHAPE, AND THAT INVERSION IS
// THE POINT. Pre-94, this case deliberately created a correction cache for
// the authority ids too — defensive, so a future regression that forgot the
// `queueMap` early-return would surface as a wrongly-allocated `NoRef`
// instead of silently reading `NoSlot` for the right reason by accident.
// Post-94 that setup would be actively WRONG: the filter IS cache existence
// now (`findInputCache<T>(id) != nullptr`), so giving an authority id a
// cache would make `allocateFrontierSlotsAll` correctly, legitimately
// allocate a slot for it — not a regression, a direct contradiction of the
// case's own premise ("prediction-owned id" ⇔ "id with a cache", item 94's
// central established fact). The authority ids below get NO cache, matching
// `AuthorityQueueUnderrunSubstitutesTheInjectedNeutral` and every other
// authority-role case in this file, and it is precisely the absence of a
// cache — not a `queueMap` lookup any more — that the assertions below prove
// stays exclusive even when mixed with cache-bearing ids in one call.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.CollectThenAllocateMixedAuthorityAndPredictionOwnedPopulationAdvancesOnlyThePredictionOwnedFrontiers",
    "[InputResolution]")
{
    constexpr unsigned int kMixedLocalId  = 11u;
    constexpr unsigned int kMixedRemoteId = 21u;
    constexpr unsigned int kAuthorityIdA  = 31u;
    constexpr unsigned int kAuthorityIdB  = 32u;
    const unsigned int predictionOwnedIds[] = { kMixedLocalId, kMixedRemoteId };
    const unsigned int authorityIds[]       = { kAuthorityIdA, kAuthorityIdB };

    ResolutionRig rig;
    for (unsigned int id : predictionOwnedIds)
    {
        rig.storage.add<MockSimulatable>(id, MockSimulatable{});
        rig.reconciliation.createCacheFor<MockSimulatable>(id);
    }
    for (unsigned int id : authorityIds)
    {
        rig.storage.add<MockSimulatable>(id, MockSimulatable{});
        // [item 94] NO createCacheFor HERE — see the file-section comment
        // above for why creating one would now be wrong rather than merely
        // unnecessary. Matches the server overload's permanent design: an
        // authority id never gets a cache.
    }
    rig.resolution.registerLocalCharacter<MockSimulatable>(kMixedLocalId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(601, step, line);
        });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kMixedRemoteId);
    for (unsigned int id : authorityIds)
        rig.resolution.registerAuthorityCharacter<MockSimulatable>(id);

    const auto step = normalStep(70u);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(step);
    // [item 94] The manager's own collect -> allocate sequence.
    rig.reconciliation.allocateFrontierSlotsAll(step);
    // [item 94] ⚠ NO postPredictionAll CALL HERE, DELIBERATELY — discovered
    // while writing this case, not carried over from the pre-94 version.
    // `postPredictionAll` (SimulationReconciliation.h, untouched by this
    // task) sweeps storage UNCONDITIONALLY and pushes state through every
    // id's cache via a bare, throwing `.at(id)` — it has no nullable filter
    // at all, unlike `allocateFrontierSlotsAll`. That is safe in every real
    // production configuration because a manager that calls it
    // (`onGameSimulationPrediction`'s caller) never shares its storage with
    // authority-only ids — a listen server runs the prediction role and the
    // authority role on SEPARATE `ASimulationManagerUImpl` instances, each
    // with its own storage/reconciliation (`SimulationManagerUImpl.cpp`'s
    // `s_instances[0]`/authority vs the prediction-role instance). This
    // TEST's rig deliberately mixes both id classes in ONE storage to pin the
    // ALLOCATION filter in isolation — a configuration production never
    // produces — so calling `postPredictionAll` here would throw on the very
    // first authority id it swept, not because of a defect this task
    // introduced, but because the artificial mix violates a precondition
    // `postPredictionAll` has always had and this task does not touch. Left
    // as an unplanned discovery, not filed as a defect: no known path
    // constructs this storage shape in production. The prediction-owned ids'
    // frontier state is still fully verified below via `getAppliedCaptureTickRef`
    // (allocation), which does not require capture to have run.

    for (unsigned int id : predictionOwnedIds)
    {
        INFO("prediction-owned id=" << id);
        REQUIRE(hasInputFor(inputs, id));
        const AppliedCaptureRef ref =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 70u);
        REQUIRE(ref.kind == AppliedCaptureRefKind::NoRef);
    }

    for (unsigned int id : authorityIds)
    {
        INFO("authority id=" << id);
        // collectInputAll still resolves an input for authority ids
        // (underrun -> injected neutral, same as
        // AuthorityQueueUnderrunSubstitutesTheInjectedNeutral); this case's
        // own concern is the FRONTIER side, which must stay untouched even
        // mixed in with prediction-owned ids in one allocate call.
        REQUIRE(hasInputFor(inputs, id));
        const AppliedCaptureRef ref =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 70u);
        REQUIRE(ref.kind == AppliedCaptureRefKind::NoSlot);
    }
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 94] THE CLIENT-SIDE TEARDOWN WINDOW —
// "CACHE PRESENT, STORAGE ABSENT" (acceptance criterion (b)'s shape).
// ---------------------------------------------------------------------------
// The item-92/93 pair above drives the AUTHORITY-side window: an id sits in
// storage with NO cache (the server overload's permanent design). This case
// drives its CLIENT-side mirror: a PREDICTION-owned id, WITH a real cache,
// sits OUTSIDE storage — item 93's fixed teardown order
// (`storage.remove` -> `netSync.unregisterSimulatable` ->
// `reconciliation.removeCacheFor`, `SimulationNetSync.h`) opens exactly this
// window for the width of the middle call, which this case reproduces at the
// core level (no `SimulationNetSync` in this file — see `ResolutionRig`).
//
// Unlike the authority window, "not swept" here cannot be proven merely by
// absence of a crash (there IS a cache, so a bare `.at(id)` would have
// succeeded even pre-92/93) — it is proven by both sweeps being
// STORAGE-DRIVEN (`m_storage.forEachSimulatable`), so an id storage does not
// have is never visited by either, regardless of what its cache says. "Arms
// nothing" is checked directly against the frontier-pair detector
// (`m_frontierSlotAwaitingState`, read through `getDiagnostics()`), not
// inferred from an absence of side effects — the AC's own third clause.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.CacheOutlivesStorageWindowIsNotSweptByAllocationAndArmsNoDetector",
    "[InputResolution]")
{
    constexpr unsigned int kMidTeardownId = 45u;
    ResolutionRig rig;

    rig.storage.add<MockSimulatable>(kMidTeardownId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kMidTeardownId);
    rig.resolution.registerLocalCharacter<MockSimulatable>(kMidTeardownId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(901, step, line);
        });

    // One full, closed pair at tick 80 — mirrors an ordinary live tick before
    // the character leaves.
    const auto firstStep = normalStep(80u);
    rig.resolution.collectInputAll(firstStep);
    rig.reconciliation.allocateFrontierSlotsAll(firstStep);
    rig.reconciliation.postPredictionAll(firstStep);

    const auto* cacheBefore = rig.reconciliation.findInputCache<MockSimulatable>(kMidTeardownId);
    REQUIRE(cacheBefore != nullptr);
    REQUIRE_FALSE(cacheBefore->getDiagnostics().frontierSlotAwaitingState());
    REQUIRE(rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kMidTeardownId, 80u).kind
        == AppliedCaptureRefKind::NoRef);

    // [item 93] THE FIXED TEARDOWN ORDER, DRIVEN THROUGH THE REAL SEQUENCE:
    // storage.remove FIRST — the cache still exists (unregisterCharacter,
    // which would erase it, has not run yet).
    rig.storage.remove<MockSimulatable>(kMidTeardownId);
    REQUIRE_FALSE(rig.storage.has<MockSimulatable>(kMidTeardownId));
    REQUIRE(rig.reconciliation.findInputCache<MockSimulatable>(kMidTeardownId) != nullptr);

    // A further tick's collect + allocate — both storage-driven — must not
    // touch this id at all: NOT SWEPT.
    const auto secondStep = normalStep(81u);
    const MockResolvedInputs inputs = rig.resolution.collectInputAll(secondStep);
    REQUIRE_FALSE(hasInputFor(inputs, kMidTeardownId));
    rig.reconciliation.allocateFrontierSlotsAll(secondStep);

    // ARMS NOTHING: the detector bit is exactly as it was before this window
    // — untouched, not merely false again by coincidence.
    const auto* cacheAfter = rig.reconciliation.findInputCache<MockSimulatable>(kMidTeardownId);
    REQUIRE(cacheAfter != nullptr);
    REQUIRE_FALSE(cacheAfter->getDiagnostics().frontierSlotAwaitingState());

    // NOT SWEPT, restated at the frontier: the tick-81 slot was never opened
    // (NoSlot), and the tick-80 slot this character's real registration
    // produced is untouched (still NoRef, not overwritten or advanced).
    REQUIRE(rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kMidTeardownId, 81u).kind
        == AppliedCaptureRefKind::NoSlot);
    REQUIRE(rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kMidTeardownId, 80u).kind
        == AppliedCaptureRefKind::NoRef);

    // Complete the teardown, matching the fixed facade's remaining step, so
    // the rig is left consistent.
    rig.resolution.unregisterCharacter<MockSimulatable>(kMidTeardownId);
    rig.reconciliation.removeCacheFor<MockSimulatable>(kMidTeardownId);
}

#endif // WITH_LOW_LEVEL_TESTS
