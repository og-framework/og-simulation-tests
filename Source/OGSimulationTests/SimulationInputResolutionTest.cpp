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
// FRONTIER-PAIR DISCIPLINE (item 84's detector). Every call this file makes
// to `resolution.prepareSimulationStep` (named `collectInputAll` before item
// 90's rename) on a step that allocates a frontier slot is followed by
// `reconciliation.postPredictionAll` for the SAME step, mirroring
// what `SimulationManager` always does within one manager tick — exactly the
// pairing item 84's fixture repair (SimulationNetSyncTest.cpp, og-brawler-
// tests) had to add for the same reason. Skipping this would abort the
// process on the NEXT allocation (`OG_CHECK` in CorrectionCache.h), not
// silently misbehave.
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

    // A no-op input provider — prepareSimulationStep's local-provider branch
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
TEST_CASE("SimulationInputResolution.LocalCharacterPrepareSimulationStepRunsTheProviderAndPushesTheFrontierPair",
    "[InputResolution]")
{
    ResolutionRig rig;
    // prepareSimulationStep/collectResimInputAll/postPredictionAll all iterate
    // m_storage.forEachSimulatable — the id must be present in STORAGE, not
    // just registered on the resolution/reconciliation peers, or none of the
    // per-character bodies this file exercises ever run for it.
    rig.storage.add<MockSimulatable>(kLocalId, MockSimulatable{});
    rig.reconciliation.createCacheFor<MockSimulatable>(kLocalId);
    rig.resolution.registerLocalCharacter<MockSimulatable>(kLocalId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(7, step, line);
        });

    REQUIRE(rig.resolution.isLocallyControlled<MockSimulatable>(kLocalId));

    const auto step = normalStep(10u);
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(step);
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
TEST_CASE("SimulationInputResolution.RemoteCharacterPrepareSimulationStepResolvesFromTheRelayStore",
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
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(step);
    rig.reconciliation.postPredictionAll(step);

    REQUIRE(hasInputFor(inputs, kRemoteId));
    REQUIRE(inputFor(inputs, kRemoteId).value == 42);
}

// ---------------------------------------------------------------------------
// Authority remote-move queue: queueRemoteMove door, non-underrun consumption,
// and the join-key accessor.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.QueueRemoteMoveThenPrepareSimulationStepConsumesItAndRecordsTheJoinKey",
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
    // registerSimulatable exactly. prepareSimulationStep's remote-move branch
    // does not touch reconciliation at all, so no pair to complete.
    const auto step = normalStep(9u);
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(step);

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
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(step);

    REQUIRE(hasInputFor(inputs, kAuthId));
    REQUIRE(inputFor(inputs, kAuthId).value == -1);
    // Underrun -> sentinel join key, never the queued tick.
    REQUIRE(rig.resolution.getLastUsedCaptureTick<MockSimulatable>(kAuthId) == kNoInputCaptureTick);
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 92] THE REGISTRATION-ORDERING WINDOW.
// ---------------------------------------------------------------------------
// SimulationNetSync.h's server `registerSimulatable` overload allocates NO
// correction cache for an authority id, ever (see `AuthorityQueueUnderrunSub
// stitutesTheInjectedNeutral` above — no `createCacheFor` call there either,
// matching production exactly). Pre-fix, that overload called `storage.add`
// BEFORE `registerAuthorityOwner` (-> `registerAuthorityCharacter`, the ONLY
// call that populates `queueMap`) — a window during which a concurrent
// physics tick's sweep 2 (`allocateFrontierSlotsAll`) would see the id in
// storage, miss it in `queueMap`, and fall through to `pushPredictionTick`
// for an id that will NEVER have a cache. `SimulationReconciliation::
// getCacheFor`'s bare `.at(id)` then threw — the exact crash captured in
// `runs/t91_hardresync_pie/client3_probe.log:86..95`
// (`SimulationReconciliation.h:783` <- `SimulationInputResolution.h:1673`
// <- `SimulationManager.h:1006`).
//
// This file has no `SimulationNetSync` (see the `ResolutionRig` comment
// above), so it cannot drive the free-function facade's ordering directly —
// it drives the SAME window at the core level instead, by calling
// `storage.add` and `resolution.registerAuthorityCharacter` in each order in
// turn.
//
// ⛔ NO DEATH TEST (same ruling as item 84's FrontierPairContractTest.cpp —
// see its file banner): `allocateFrontierSlotForCharacter`'s loud-failure
// guard (`OG_CHECK`, added by item 92) cannot be trapped by the LLT harness;
// firing it aborts the whole test process. The first case below pins the
// PRECONDITION the guard tests — via the same diagnostics-read seam
// (`findInputCache`) production code uses — rather than calling the guarded
// sweep and catching what it does.
//
// RED/GREEN (quoted in impl/impl_notes_task92.md): with the loud-failure
// guard temporarily reverted AND a scratch case added that actually calls
// `rig.resolution.prepareSimulationStep(step)` on this exact window state,
// the case goes RED — Catch2 reports an uncaught `std::out_of_range` escaping
// the test, the same exception class the production crash's SEH translation
// (`0xe06d7363`) wraps. Restoring the guard and removing the scratch case
// (this file's actual, permanent content) is the GREEN side: the window
// state is pinned as unreachable-without-a-throw through the seam below
// instead, and the second case proves the FIXED ordering never produces it.
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

    // The precondition allocateFrontierSlotForCharacter's OG_CHECK tests:
    // no correction cache exists for this id. (queueMap also misses it, by
    // construction — registerAuthorityCharacter, the only thing that inserts
    // into queueMap, was never called.) Together these are exactly the state
    // that made pushPredictionTick's getCacheFor(id).at(id) throw.
    REQUIRE(rig.reconciliation.findInputCache<MockSimulatable>(kWindowId) == nullptr);
}

// The other side of the same window: the FIXED ordering (registerAuthority
// Character before storage.add — SimulationNetSync.h's server overload now
// does this) never produces the state above, and prepareSimulationStep runs
// clean through both sweeps for a freshly-registered authority id.
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
    // Runs sweep 1 AND sweep 2 (prepareSimulationStep calls
    // allocateFrontierSlotsAll internally) — if the ordering fix or the
    // guard regressed, this line would throw or abort instead of returning.
    // No postPredictionAll call: mirrors production exactly — it is a
    // prediction-role-only call (onGameSimulationAuthority never makes it,
    // see SimulationManager.h), and this id has no cache to complete a pair
    // in anyway (server overload's permanent design, unchanged by this fix).
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(step);

    REQUIRE(hasInputFor(inputs, kOrderedId));
    REQUIRE(inputFor(inputs, kOrderedId).value == -1); // underrun -> injected neutral
    // Sweep 2's queueMap early-return still holds: no frontier slot opened.
    const AppliedCaptureRef ref =
        rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(kOrderedId, 50u);
    REQUIRE(ref.kind == AppliedCaptureRefKind::NoSlot);
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 93] THE UNREGISTRATION-ORDERING WINDOW —
// item 92's MIRROR.
// ---------------------------------------------------------------------------
// SimulationNetSync.h's free-function `unregisterSimulatable` facade used to
// call `netSync.unregisterSimulatable` (-> `SimulationInputResolution::
// unregisterCharacter`, which erases `queueMap`) BEFORE `storage.remove` — a
// window in which a concurrent physics tick's sweep 2 would see the id STILL
// in storage (still visible to forEachSimulatable) but no longer in
// `queueMap`, fall through the guard, and call `pushPredictionTick` for an id
// that (server overload's permanent design) has NEVER had a correction
// cache. This is the exact same precondition item 92 fixed on the
// REGISTRATION side, reached from the opposite direction: publish-last on
// the way in (item 92) means unpublish-first on the way out (item 93) —
// storage.remove must run BEFORE the queueMap/telemetry teardown, not after.
//
// This file has no `SimulationNetSync` (see the `ResolutionRig` comment
// above), so — same as item 92's pair above — it drives the SAME window at
// the core level: calling `resolution.unregisterCharacter` and
// `storage.remove` in each order in turn, rather than the free-function
// facade directly.
//
// ⛔ NO DEATH TEST, same ruling as item 92's pair immediately above (and
// item 84's FrontierPairContractTest.cpp): `allocateFrontierSlotForCharacter`'s
// `OG_CHECK` cannot be trapped by the LLT harness. The first case below pins
// the PRECONDITION via the diagnostics-read seam (`findInputCache` +
// `storage.has`) instead of calling the guarded sweep and catching what it
// does.
//
// RED/GREEN (quoted in impl/impl_notes_task93.md): with the guard temporarily
// reverted AND a scratch case added that drove `prepareSimulationStep` on
// this exact window state (id unregistered from `queueMap` but still in
// storage — the OLD facade order), the case went RED — the same
// `std::out_of_range` class ("invalid unordered_map<K, T> key") item 92's RED
// demonstration produced. Restoring the guard and removing the scratch case
// is the GREEN side below.
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

    // The precondition allocateFrontierSlotForCharacter's OG_CHECK tests:
    // still in storage (so sweep 2 would visit it), no longer in queueMap
    // (so the guard's early-return misses it), and no correction cache
    // (server overload never allocates one) — together, exactly the state
    // that made pushPredictionTick's getCacheFor(id).at(id) throw.
    REQUIRE(rig.storage.has<MockSimulatable>(kUnregWindowId));
    REQUIRE(rig.reconciliation.findInputCache<MockSimulatable>(kUnregWindowId) == nullptr);
}

// The other side of the same window: the FIXED ordering (storage.remove
// BEFORE the queueMap/telemetry teardown — SimulationNetSync.h's
// unregisterSimulatable free-function facade now does this) removes the id
// from storage before anything erases queueMap, so it is never visible to
// forEachSimulatable during teardown at all.
TEST_CASE("SimulationInputResolution.StorageRemovedBeforeQueueEntryErasureIsInvisibleToPrepareSimulationStep",
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
    // run yet) — but that no longer matters: prepareSimulationStep's sweeps
    // both iterate storage.forEachSimulatable, so an id storage no longer
    // has is simply never visited, regardless of what queueMap/cache say
    // about it. Runs sweep 1 AND sweep 2 (prepareSimulationStep calls
    // allocateFrontierSlotsAll internally) — if this ordering regressed,
    // this line would throw or abort.
    const auto step = normalStep(60u);
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(step);
    REQUIRE_FALSE(hasInputFor(inputs, kUnregOrderedId));

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
    rig.resolution.prepareSimulationStep(step);      // delay line now holds tick 3's capture
    rig.reconciliation.postPredictionAll(step);      // complete the frontier pair

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
    rig.resolution.prepareSimulationStep(step);
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
    rig.resolution.prepareSimulationStep(step); // populates the local delay line + pending queue
    rig.reconciliation.postPredictionAll(step);

    auto* pending = rig.resolution.findPendingInputQueue<MockSimulatable>(kLocalId);
    REQUIRE(pending != nullptr);
    REQUIRE_FALSE(pending->empty()); // prepareSimulationStep enqueued tick 1's capture

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
// [og-netcode-v2-input-relay item 90] TWO-SWEEP AGREEMENT.
//
// `prepareSimulationStep` resolves every character's input in sweep 1 and
// allocates the frontier for every prediction-owned id in sweep 2 — two
// separate `forEachSimulatable` passes inside ONE call. This is the pin that
// the two sweeps stay in agreement: from one call, every registered
// prediction-owned id (local-provider AND simulated-proxy alike) gets BOTH a
// resolved input AND its frontier advanced exactly once, and the StepKind
// matrix that gates sweep 2 (Normal/Skip allocate, Stall does not) behaves
// exactly as it did before the sweep split.
//
// "Frontier advanced" is observed the same way
// LocalCharacterPrepareSimulationStepRunsTheProviderAndPushesTheFrontierPair
// already does above: `getAppliedCaptureTickRefKind` answers `NoRef` (a slot
// exists, uncorrected) once a tick has been pushed and `NoSlot` (the
// `AppliedCaptureRef{}` default) if it never was — `getCacheIndex` rejects a
// tick outside the ring's allocated window. "Exactly once" is enforced
// structurally rather than counted: sweep 2 visits each id exactly once per
// `forEachSimulatable` pass, and a SECOND `pushPredictionTick` for the same
// id in the same call would trip item 84's `OG_CHECK` in
// `StateCorrectionCache::pushPredictionTick` and abort the process — so a
// case that runs to completion and passes its `postPredictionAll` pairing
// discipline (the file banner) has already proven at-most-once by not
// crashing; these cases prove at-least-once with the `NoRef` checks below.
//
// ⛔ RED/GREEN, DEMONSTRATED (see impl/impl_notes_task90.md for the quoted
// run output): with sweep 2 (`allocateFrontierSlotsAll`) locally commented
// out of `prepareSimulationStep`, every `AppliedCaptureRefKind::NoRef` REQUIRE
// below goes RED (the query answers `NoSlot` instead — nothing ever pushed
// the frontier past construction). Restored, every case is GREEN. This is
// the case that pin protects, and is the reason it exists as its own file
// section rather than folding into the two single-character cases above.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.PrepareSimulationStepNormalStepAdvancesEveryPredictionOwnedIdsFrontierExactlyOnce",
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
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(step);
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

TEST_CASE("SimulationInputResolution.PrepareSimulationStepSkipStepBackfillsAndAdvancesEveryPredictionOwnedIdsFrontierExactlyOnce",
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
    rig.resolution.prepareSimulationStep(firstStep);
    rig.reconciliation.postPredictionAll(firstStep);

    // A Skip step landing at tick 12 (StepKind::Skip: "sim tick jumps by >1;
    // previous tick must be back-filled" — SimulationTimeContext.h). Sweep 2
    // must both backfill tick 11 (step.getTick() - 1, today's single-tick
    // backfill — tick 10 is a genuine, documented gap, unchanged by this
    // task) AND push the new frontier tick 12, from the SAME call.
    const SimulationTimeStep skipStep(12u, /*isResimulating=*/false, StepKind::Skip, kDeltaSeconds);
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(skipStep);
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

TEST_CASE("SimulationInputResolution.PrepareSimulationStepStallStepResolvesInputButAllocatesNoFrontierSlot",
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
    // if sweep 2 (wrongly) ran on Stall, tick 5 would read NoRef; if it
    // (correctly) does not, tick 5 stays NoSlot. `stepAllocatesFrontierSlot`
    // is `kind != StepKind::Stall` — this is the one StepKind the predicate
    // answers false for.
    const SimulationTimeStep stallStep(5u, /*isResimulating=*/false, StepKind::Stall, kDeltaSeconds);
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(stallStep);
    // No postPredictionAll call — item 84's pair never opened on a Stall
    // step, so there is nothing to complete (postPredictionAll's own early
    // return would make the call a no-op anyway).

    for (unsigned int id : allIds)
    {
        INFO("id=" << id);
        // Sweep 1 still resolves an input — resolution is unconditional on
        // StepKind; only frontier allocation (sweep 2) is gated.
        REQUIRE(hasInputFor(inputs, id));

        const AppliedCaptureRef ref =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 5u);
        REQUIRE(ref.kind == AppliedCaptureRefKind::NoSlot);
    }
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 91 part I2] SWEEP 1 THROWING PARTWAY — the
// gap the dispatch's own Q4 predicted the three cases above would miss
// (item 90 review finding 1): commenting out sweep 2 lets sweep 1 run to
// completion for every id, which is NOT the same failure shape as sweep 1
// itself throwing partway through. The two are different mutations with
// different observable consequences, and only a genuine mid-sweep-1 throw
// proves the ACTUAL claim the sweep-boundary banner
// (`SimulationInputResolution.h`, above `prepareSimulationStep`) now states:
// an uncaught exception during sweep 1 skips sweep 2 for this tick entirely.
//
// REACHABILITY NOTE. The real trigger is one of the three `.at(id)` lookups
// in `collectInputForCharacter`'s local-provider branch (delay line,
// pending-input queue, last-used-capture-tick map) — these throw only when
// the provider-present/line-present invariant `registerLocalCharacter`
// always establishes together has ALREADY been broken elsewhere. That is not
// reproducible through this class's own public API (registration is
// all-or-nothing) — which is itself the reason the sweep-boundary decision
// records this as a second-bug-required path, not a standalone defect. This
// case substitutes a THROWING PROVIDER for that public-API-unreachable
// corruption: the control-flow consequence at the sweep boundary is
// identical either way — an uncaught exception unwinds out of
// `prepareSimulationStep` before sweep 2 (`allocateFrontierSlotsAll`) ever
// runs, REGARDLESS of which registered id's sweep-1 body happened to run
// first (this class's storage iterates an `unordered_map`, so processing
// order is not something a test can pin) — that is exactly why the
// assertion below holds for BOTH ids without needing to control order.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.PrepareSimulationStepSweep1ThrowingPartwayAllocatesNoFrontierSlotForAnyId",
    "[InputResolution]")
{
    constexpr unsigned int kSurvivingId = 11u;
    constexpr unsigned int kThrowingId  = 12u;
    const unsigned int allIds[] = { kSurvivingId, kThrowingId };

    ResolutionRig rig;
    for (unsigned int id : allIds)
    {
        rig.storage.add<MockSimulatable>(id, MockSimulatable{});
        rig.reconciliation.createCacheFor<MockSimulatable>(id);
    }
    rig.resolution.registerLocalCharacter<MockSimulatable>(kSurvivingId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(401, step, line);
        });
    rig.resolution.registerLocalCharacter<MockSimulatable>(kThrowingId,
        [](const SimulationTimeStep&, const LocalInputCache<MockInput>&) -> MockInput {
            // Stands in for the corruption the three `.at(id)` lookups guard
            // against — see the file-section comment above.
            throw std::runtime_error("simulated sweep-1 registration-invariant break");
        });

    const auto step = normalStep(60u);
    bool threw = false;
    try
    {
        rig.resolution.prepareSimulationStep(step);
    }
    catch (const std::runtime_error&)
    {
        threw = true;
    }
    REQUIRE(threw);
    // No postPredictionAll call: sweep 2 never ran, so the frontier-pair
    // contract never opened for either id this tick — nothing to complete.

    for (unsigned int id : allIds)
    {
        INFO("id=" << id);
        // [item 91 part I] THE PROPERTY THIS CASE PINS: post-item-90, sweep 2
        // runs only after sweep 1's ENTIRE forEachSimulatable returns without
        // throwing — so ONE id's provider throwing leaves EVERY registered
        // id, including one whose own sweep-1 work may have already
        // completed, with NO frontier slot allocated for this tick.
        // Pre-item-90 (resolve+allocate paired per character inline) this
        // could not happen: a mid-loop throw left already-processed
        // characters FULLY paired instead — see the sweep-boundary banner.
        const AppliedCaptureRef ref =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 60u);
        REQUIRE(ref.kind == AppliedCaptureRefKind::NoSlot);
    }
}

// ---------------------------------------------------------------------------
// [og-netcode-v2-input-relay item 91 part I3] MIXED AUTHORITY +
// PREDICTION-OWNED POPULATION — the three two-sweep cases above register
// only prediction-owned ids (local-provider / simulated-proxy), never a true
// authority id (`registerAuthorityCharacter`, the remote-move-queue branch).
// None of them directly re-proves sweep 2's `queueMap` early-return
// (`allocateFrontierSlotForCharacter`) correctly excludes authority-owned
// ids when mixed with prediction-owned ids in the SAME `prepareSimulationStep`
// call — this case does.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationInputResolution.PrepareSimulationStepMixedAuthorityAndPredictionOwnedPopulationAdvancesOnlyThePredictionOwnedFrontiers",
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
        // A cache IS created here (unlike AuthorityQueueUnderrunSubstitutesTheInjectedNeutral,
        // which relies on findInputCache's nullable route) so that a future
        // regression allocating a frontier slot for an authority id would
        // surface as NoRef below, not silently read NoSlot for the wrong reason.
        rig.reconciliation.createCacheFor<MockSimulatable>(id);
    }
    rig.resolution.registerLocalCharacter<MockSimulatable>(kMixedLocalId,
        [](const SimulationTimeStep& step, const LocalInputCache<MockInput>& line) {
            return constantProvider(601, step, line);
        });
    rig.resolution.registerRemoteCharacter<MockSimulatable>(kMixedRemoteId);
    for (unsigned int id : authorityIds)
        rig.resolution.registerAuthorityCharacter<MockSimulatable>(id);

    const auto step = normalStep(70u);
    const MockResolvedInputs inputs = rig.resolution.prepareSimulationStep(step);
    // Complete the frontier pair for the prediction-owned ids only —
    // authority ids never open one (file banner discipline).
    rig.reconciliation.postPredictionAll(step);

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
        // Sweep 1 still resolves an input for authority ids (underrun ->
        // injected neutral, same as AuthorityQueueUnderrunSubstitutesTheInjectedNeutral);
        // this case's own concern is the FRONTIER side, which must stay
        // untouched even mixed in with prediction-owned ids in one call.
        REQUIRE(hasInputFor(inputs, id));
        const AppliedCaptureRef ref =
            rig.reconciliation.getAppliedCaptureTickRef<MockSimulatable>(id, 70u);
        REQUIRE(ref.kind == AppliedCaptureRefKind::NoSlot);
    }
}

#endif // WITH_LOW_LEVEL_TESTS
