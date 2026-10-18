<!-- SPDX-License-Identifier: MPL-2.0 -->
# `TickDriverFakes.h` — rationale

Source: `TickDriverFakes.h`, the tick-driver suites' own fake host kit. This file has no guards document:
the kit's contracts are its `static_assert`s (every fake models its concept), so nothing in it needs a
`⛔` tag.

Everything is in namespace `tickDriverFakes`. The kit includes `StepDriverTestSupport.h` and reuses its
`LoggedWorld` and `Events`; it does not copy them, and it does not use the anonymous-namespace rigs of
the step-driver suites.

---

## §1 The API, one line per fake

| fake | models | API |
|---|---|---|
| `FakeEngine` (`BasicFakeEngine<true>`), `FakeEngineNoDiagnostics` (`<false>`) | `TickDriverEngine` | public `now`/`advance(s)`, `frame`/`advanceFrame()`, `minimumLevel`, `lines` (`LoggedLine` level + text), `deferralSupported`; executor: `dispatchAfter`, `isPending`, `waitFor`, `deferUntil`, plus `runNextTask()`, `runAllTasks()`, `queuedTasks()`, `runDeferred()`, `pendingDeferrals()`, `startWorkerThread()`/`stopWorkerThread()`/`workerThreadId()`; records `dispatchedAfter`, `waits`, `deferrals`, `tasksRun` |
| `RecordingGame` (`BasicRecordingGame<ManagerT, StorageT, RenderSnapshotT>`) | `TickDriverGame` | constructed over `(Events&, Manager&, Storage&)`; every hook appends one event; typed records `beforePumpHostSeconds`, `frameStepsDue`, `afterInlineBatchTicks`, `releasedTicks`, `stepStartedSeconds`, `postPhysicsCalls`; injectable `onBeforePump` |
| `FakePresenter` | `TickDriverPresenterSteps` | constructed over `Events&`; records `capturedSteps`, `published`, `publishedDeadlines` |
| `FakeWorldHost` (`BasicFakeWorldHost<true>`), `FakeWorldHostNoDiagnostics` (`<false>`) | `TickDriverWorldHost`, `TickBackendWorldHost`, `TickBackendWorldBuild` (over two `FakeAdapter` optionals) | constructed over `(Events&, roleBit, stepsInline, ringDepthTicks = 8)`, which configures its `TickDriverThreadContext` in the constructor, or over `(Events&, kConfigureAtBuild, ringDepthTicks = 8)`, which leaves it to `build`; `stepWorld()` is a `LoggedWorld`; `lockForStep()` and `lockWorld()` (both the world lock and the access scope, no step scope; §5), `lock()` (the lock, with its `locks()` count); the backend's members: `build(config, queryable, physicsOnly, bodyAdapter, queryAdapter)` (`Config` is `FakeWorldHostConfig`: `isAuthority`, `runInline`), `bindIntegratedTickSource`, `rootDeclarationIndex()` (returns `rootIndex`), `acquire` (the authority takes the next slot and publishes it; a client takes its replicated slot or waits), `bind` (under `lockWorld()`, calls the wait note with `bindLockWaitSeconds`), `release` (pushes the vacated slot), `slotOf`, `editVizQuery()`/`getVizReader()` (`vizQuery`, `vizReader`), `registerVizVolume`; the presenter's three doors: `hasShadowWorld()` (`shadowWorld`), `restore_GameThread` (calls the note with `restoreSeconds`, `restoreRefused`), `overwriteShadowPoses_GameThread`; records `afterTickOutcomes`, `afterRunTickSteps`, `builtStatics`, `integratedTick` |
| `FakeAdapter` | (an adapter optional's value) | constructed over `(Events&, kind, context)` by the fake's `build`; keeps the context's `stepWorldPredicate` it saw, and appends `emplace:<kind>`, or `emplace:<kind>:unconfigured` when the context had no predicate yet |
| `FakeRenderSink` | `TickDriverRenderSink` | `addTarget(id, position, displayName)`, `removeTarget(id)`, `targets`; records `applies`; optional `holdRule(held, requested)` for an engine that does not take every write |

Helpers: `LockUnlockOnly` (a mutex with only `Lock`/`Unlock`, the shape of an engine mutex; not
`BasicLockable`), `WrapperOverLockUnlockOnly` (its `lock`/`unlock` wrapper; the world host's lock),
`FakeAccessScope` (an RAII access scope with `isOpenOnThisThread()`), and `FakeManager`, `FakeStorage`,
`FakeRenderSnapshot` (empty default types for the game).

## §2 The event list

Every fake that records an event appends it to one `Events` list (a `std::vector<std::string>`) that
the case owns, the pattern of `StepDriverTestSupport.h`. `LoggedWorld` writes its own events (`step`,
`save:N`, `restore:N`, `occ`, …) to the same list, so a case asserts the interleaving of the game's
hooks, the world, the presenter and the world host in one comparison. The spellings:

| source | events |
|---|---|
| `RecordingGame` | `beforePump`, `onFrameStepsDue:<firstTick>,<steps>`, `afterInlineBatch:<firstTick>`, `releaseInputs:<tick>`, `onStepStarted`, `onStepEnded`, `onPostPhysics` |
| `FakePresenter` | `captureCorrection:<physicsStep>`, `publish:<tick>` |
| `FakeWorldHost` | `hostAfterTick:<tick>`, `afterRunTick:<physicsStep>`; and, for the backend's calls, `configure:<role>:<stepping>` (role `authority` or `client`, stepping `inline` or `worker`), `bindIntegratedTickSource`, `rootDeclarationIndex`, `acquire:<key>:<slot>` or `acquire:<key>:wait`, `bind:<slot>:<id>`, `release:<key>:<slot>:<id>`, `registerVizVolume:<slot>`, `editVizQuery`, `getVizReader`, `restore:<frame>`, `overwriteShadowPoses` |
| `FakeAdapter` | `emplace:<kind>`, `emplace:<kind>:unconfigured` |

## §3 The executor

`FakeEngine`'s executor has two modes.

- **Manual** (the default). `dispatchAfter` queues the task and returns its handle; nothing runs until
  the case calls `runNextTask()` or `runAllTasks()`, which run the queue in order. The queue is FIFO, so
  a task chained after an earlier one runs after it. `waitFor(handle)` runs queued tasks until the
  handle's task is done, the way an engine's blocking wait can run the task on the waiting thread.
  `dispatchedAfter` records, per dispatch, the id of the previous task when it was still pending, or 0.
- **Thread** (`startWorkerThread()`). A `std::thread` runs the queue in order, and `waitFor` blocks
  until the task is done. `stopWorkerThread()` (also run by the destructor) drains the queue and joins.

`deferUntil(completion, handle, continuation)` returns `false` when `deferralSupported` is false (the
engine has no completion event). Otherwise it stores the continuation; `runDeferred()` runs every stored
continuation whose task is done, after appending it to the `FakeCompletion`.

⚠ In thread mode the worker and the case's thread can both append to the shared `Events` list and
`LoggedWorld` does so without a lock. A case that uses thread mode must not let the two threads record at
the same time: wait for the dispatched task before the case's thread records again, or keep the events
the worker writes in a list of their own.

## §4 Thread context in the fakes

`FakeWorldHost` calls `TickDriverThreadContext::configure` in its constructor, the way a world host
configures at world build, so its predicates hold for the constructing thread at once. Built with
`kConfigureAtBuild` it calls it in `build` instead, before it emplaces the two adapters, as a real world host's
`build` does (BL1); a case then checks that order through the events and the adapters' predicates. Its destructor
(through the context's) takes its count off that role's owner count again, so cases that run one after
another on the same thread do not inherit each other's counts. Two hosts of different roles can live on
one thread, which is how `TickDriverThreadContext.Predicates.TwoRoleBitsOnOneThreadStayApart` sets up an
authority and a client side by side. Two hosts of the same role can too: each one adds its own count,
so destroying one leaves the other's predicates holding. Destroy a host on the thread that built it;
destroyed anywhere else it leaves its count behind on the building thread.

## §5 The step-free lock (`lockForStep`)

`lockForStep()` returns the same `WorldLock` as `lockWorld()`: the world lock and the access scope, and no
`StepScope`. That is what a real world host's `lockForStep()` returns, and the tick driver opens its own `StepScope`
inside it for each step, so a kit lock that also opened one made the two nest, which `StepScope` refuses ("steps
never nest"). Until the kit was aligned the driver's suites derived a world host whose `lockForStep()` returned
`lockWorld()` (`TraceWorldHost` in `SimulationTickDriverTest.cpp`, which still does, now to the same effect). A rig
that runs the step driver without the tick driver opens the step scope itself after the lock, as the step hooks'
rig does. `SimulationTickBackend.KitFix.TheDriverStepsOverTheKitsWorldHostWithoutNestingStepScopes` runs the
driver over the kit's own world host.
