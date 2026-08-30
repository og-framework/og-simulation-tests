// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"

#include "OGSimulation/SimulationObjectStorage.h"
#include "OGSimulation/SimulationReconciliation.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/SlotStateProvenance.h"

#include <cstdint>
#include <optional>

//////////////////////////////////////////////////////////////////////////////
// THE PER-TICK PROVENANCE READ SEAM on `SimulationReconciliation`'s diagnostic
// view: `getDiagnostics().slotStateProvenance<T>(id, simTick)`.
//
// WHY IT EXISTS. `logSlotProvenanceAll` FORMATS the whole slot map into one
// Verbose line and answers nothing about a single tick. Before this seam the
// only way to read one tick's lineage was `findCorrectionCache` plus a
// hand-computed cache index at the call site — a reader reaching through a
// production accessor whose whole point is the prediction-ownership test.
//
// WHAT THESE CASES PROVE, following the precedent named in
// `docs/DiagnosticsConventions.md` §5 —
//     ResimGate.Policy.TheResimGateProbeAccessorObservesTheShippedFeed
// — namely that an accessor observes the REAL feed rather than merely
// compiling: every provenance value asserted below is written by a SHIPPED
// write site driven through this class's own public sweeps
// (`allocateFrontierSlotsAll` -> `pushPredictionTick`, and
// `injectCorrectionState` -> `tryInsertingCorrectState`). Nothing here
// scribbles the column directly.
//
// THE AUTHORITY IS THE HAZARD. The server allocates no correction cache at
// all, so every id must answer "nothing here" rather than fault — the same
// safety `getAppliedCaptureTickRef` already carries, which is why this seam
// routes through the same nullable `findCorrectionCache`.
//
// LOCAL isolation: a scratch mock simulatable only. No brawler, UE or owner
// type is named in this translation unit.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    struct ProvState
    {
        std::int32_t position = 0;

        // `tryInsertingCorrectState` compares slots via `compare()`, which prefers a
        // member `isSimilarTo` when one exists — enough to drive both correction arms.
        bool isSimilarTo(const ProvState& other) const { return position == other.position; }
    };

    struct ProvInput
    {
        std::int32_t value = 0;
    };

    class ProvAllState
    {
    public:
        const ProvState& getState() const { return m_state; }
        ProvState&       editState()       { return m_state; }
    private:
        ProvState m_state;
    };

    struct ProvSimulatable
    {
        using StateType = ProvState;
        using InputType = ProvInput;

        ProvAllState m_allState;
        ProvAllState m_vizState;

        const ProvAllState& getAllState() const { return m_allState; }
        ProvAllState&       editAllState()       { return m_allState; }
        void updateVizState() { m_vizState = m_allState; }
        const ProvAllState& getVizState() const { return m_vizState; }
    };

    static_assert(SimulatableState<ProvSimulatable>,
        "ProvSimulatable must satisfy the concept SimulationObjectStorage/"
        "SimulationReconciliation require");

    using ProvStorage        = SimulationObjectStorage<ProvSimulatable>;
    using ProvReconciliation = SimulationReconciliation<ProvSimulatable>;

    constexpr unsigned int kId           = 7u;
    constexpr float        kDeltaSeconds = 1.0f / 60.0f;

    // A tick far outside the 60-slot ring these cases ever reach, so `getCacheIndex`
    // cannot find it. NOT tick 0: an unwritten slot claims tick 0 (the tick-0
    // phantom), and the last case below pins that on purpose.
    constexpr std::uint32_t kUnreachedTick = 5000u;

    SimulationTimeStep normalStep(std::uint32_t tick)
    {
        return SimulationTimeStep(tick, /*isResimulating=*/false, StepKind::Normal, kDeltaSeconds);
    }

    struct ProvRig
    {
        ProvStorage        storage;
        ProvReconciliation reconciliation{ storage };

        std::optional<SlotStateProvenance> provenanceAt(std::uint32_t tick) const
        {
            return reconciliation.getDiagnostics().slotStateProvenance<ProvSimulatable>(kId, tick);
        }

        // One complete frontier pair, exactly as `SimulationManager::
        // onGameSimulationPrediction` drives it: allocate, then complete with state.
        void predictTick(std::uint32_t tick)
        {
            const auto step = normalStep(tick);
            reconciliation.allocateFrontierSlotsAll(step);
            reconciliation.postPredictionAll(step);
        }
    };

    // The two operations `injectCorrectionState` actually calls, and nothing else.
    struct ProvCorrectionBuffer
    {
        ProvState     state{};
        std::uint32_t tick = 0;
        std::uint32_t appliedCaptureTick = kNoInputCaptureTick;

        std::uint32_t readInto(ProvState& out) const { out = state; return tick; }
        std::uint32_t getAppliedCaptureTick() const { return appliedCaptureTick; }
    };
} // namespace

// ---------------------------------------------------------------------------
// THE WIRING PROOF. Every value read below was written by a shipped write site
// driven through this class's own sweeps — and the two ticks answer
// DIFFERENTLY, which is what a per-character "latest provenance" stash could
// never do.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationReconciliation.TheSlotStateProvenanceAccessorObservesTheShippedProvenanceFeed",
    "[Reconciliation][Provenance]")
{
    ProvRig rig;
    rig.storage.add<ProvSimulatable>(kId, ProvSimulatable{});
    rig.reconciliation.createCacheFor<ProvSimulatable>(kId);

    // Two ordinary predicted ticks. `pushPredictionTick` is the shipped writer of
    // `SlotStateProvenance::Predicted`.
    rig.predictTick(30u);
    rig.predictTick(31u);

    REQUIRE(rig.provenanceAt(30u).has_value());
    REQUIRE(*rig.provenanceAt(30u) == SlotStateProvenance::Predicted);
    REQUIRE(*rig.provenanceAt(31u) == SlotStateProvenance::Predicted);

    // A correction that DISAGREES with the prediction (the mock predicted
    // position 0): `tryInsertingCorrectState` adopts the authority's state.
    ProvCorrectionBuffer disagreeing;
    disagreeing.state = ProvState{ 100 };
    disagreeing.tick  = 30u;
    rig.reconciliation.injectCorrectionState<ProvSimulatable>(kId, disagreeing);

    REQUIRE(*rig.provenanceAt(30u) == SlotStateProvenance::AuthorityAdopted);

    // A correction that AGREES: the state copy is skipped and the prediction is
    // certified instead. The OTHER arm of the same shipped write.
    ProvCorrectionBuffer agreeing;
    agreeing.state = ProvState{ 0 };
    agreeing.tick  = 31u;
    rig.reconciliation.injectCorrectionState<ProvSimulatable>(kId, agreeing);

    REQUIRE(*rig.provenanceAt(31u) == SlotStateProvenance::AuthorityAgreedKeptPrediction);

    // PER TICK, NOT PER CHARACTER — the assertion a "latest provenance" scalar fails.
    REQUIRE_FALSE(rig.provenanceAt(30u) == rig.provenanceAt(31u));

    // A tick this cache never reached has no slot at all.
    REQUIRE_FALSE(rig.provenanceAt(kUnreachedTick).has_value());
}

// ---------------------------------------------------------------------------
// SAFE WHERE THERE IS NOTHING TO READ. The authority allocates no correction
// cache, so `findCorrectionCache` answers nullptr for every id — the seam must
// answer nullopt rather than fault, exactly as `getAppliedCaptureTickRef`
// answers `NoSlot`.
// ---------------------------------------------------------------------------
TEST_CASE("SimulationReconciliation.TheSlotStateProvenanceAccessorAnswersSafelyOnTheAuthorityAndForAnAbsentId",
    "[Reconciliation][Provenance]")
{
    ProvRig rig;

    // An id this reconciliation has never heard of.
    REQUIRE_FALSE(rig.provenanceAt(30u).has_value());

    // THE AUTHORITY, driven exactly as production drives it: the character is in
    // storage, and `createCacheFor` is deliberately NEVER called.
    rig.storage.add<ProvSimulatable>(kId, ProvSimulatable{});
    REQUIRE(rig.reconciliation.findCorrectionCache<ProvSimulatable>(kId) == nullptr);

    // Tick 0 is asked FIRST — it is the tick an unwritten slot would claim, so it
    // is the value most likely to be answered by accident.
    REQUIRE_FALSE(rig.provenanceAt(0u).has_value());
    REQUIRE_FALSE(rig.provenanceAt(30u).has_value());

    // The join key it mirrors agrees, on the same id and the same tick.
    REQUIRE(rig.reconciliation.getAppliedCaptureTickRef<ProvSimulatable>(kId, 30u).kind
        == AppliedCaptureRefKind::NoSlot);

    // The lifetime is the cache's: created, it answers; removed, it stops.
    rig.reconciliation.createCacheFor<ProvSimulatable>(kId);
    rig.predictTick(30u);
    REQUIRE(rig.provenanceAt(30u).has_value());

    rig.reconciliation.removeCacheFor<ProvSimulatable>(kId);
    REQUIRE_FALSE(rig.provenanceAt(30u).has_value());
}

// ---------------------------------------------------------------------------
// nullopt AND `Empty` ARE DIFFERENT ANSWERS, and the tick-0 phantom is why the
// distinction has to be carried in data rather than inferred. A fresh cache's
// tick buffer is all zeroes, so tick 0 FINDS a slot — and that slot must report
// `Empty` ("nothing was ever written here"), never be collapsed into "no slot".
// ---------------------------------------------------------------------------
TEST_CASE("SimulationReconciliation.TheSlotStateProvenanceAccessorKeepsNoSlotAndAnEmptySlotDistinguishable",
    "[Reconciliation][Provenance]")
{
    ProvRig rig;
    rig.storage.add<ProvSimulatable>(kId, ProvSimulatable{});
    rig.reconciliation.createCacheFor<ProvSimulatable>(kId);

    // Nothing predicted yet: the tick-0 phantom slot is found, and reports Empty.
    REQUIRE(rig.provenanceAt(0u).has_value());
    REQUIRE(*rig.provenanceAt(0u) == SlotStateProvenance::Empty);

    // Any other tick has no slot at all — a different answer, not the same one.
    REQUIRE_FALSE(rig.provenanceAt(7u).has_value());
}

#endif // WITH_LOW_LEVEL_TESTS
