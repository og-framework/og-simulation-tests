// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "OGSimulation/SimulatableList.h"

// ---------------------------------------------------------------------------
// MockSimA / MockSimB — the engine-free, game-free simulatable pack the
// Network/ suite instantiates per-simulatable templates against.
// (Stage 3 / D3.5; supports ServerInputDelayQueue in task 5.)
//
// WHY MOCKS RATHER THAN SimulatableBrawler. The templates under test live in the
// sim CORE and are variadic over the simulatable pack; binding the tests to
// OGBrawler's concrete simulatable would couple a core test to the game module,
// which this suite does not link and the layering does not permit. Two mocks is
// the minimum that can prove the pack is genuinely per-type: with one, a
// "variadic" container is indistinguishable from a single-type one.
//
// Their Input types are deliberately DIFFERENT (`int` vs `float`) and not
// implicitly interchangeable in a way a test could miss — if the container ever
// collapsed the pack into shared storage, the type mismatch would surface as a
// compile error rather than as a subtly wrong value.
//
// The Input alias is spelled `Input` here, matching the D3.5 task contract.
// Production simulatables spell it `InputType` (SimulatableBrawler.h:23 et al);
// ServerInputDelayQueue's `SimulatableInputOf` trait resolves BOTH, so these
// mocks and the real simulatables are both valid pack members. See the trait's
// comment in ServerInputDelayQueue.h.
// ---------------------------------------------------------------------------

struct MockSimA
{
    using Input = int;
};

struct MockSimB
{
    using Input = float;
};

// The pack in the marker form the rest of the OGSim core consumes. Not used by
// ServerInputDelayQueue itself (which takes a bare variadic pack), but declared
// here so a future core template taking a SimulatableList<...> can reuse these
// mocks without redeclaring the pair.
using MockNetworkSimulatables = SimulatableList<MockSimA, MockSimB>;

static_assert(IsSimulatableList<MockNetworkSimulatables>,
    "MockNetworkSimulatables must be a SimulatableList<...>");
