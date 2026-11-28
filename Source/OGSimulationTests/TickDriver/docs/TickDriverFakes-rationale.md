<!-- SPDX-License-Identifier: MPL-2.0 -->
# `TickDriverFakes.h` — rationale

Source: `TickDriverFakes.h`, the tick-driver suites' own fake host kit. This file has no guards document:
the kit's contracts are its `static_assert`s (every fake models its concept), so nothing in it needs a
`⛔` tag. The kit's own cases are in `TickDriverConceptsTest.cpp` (`TickDriverFakes.Engine.*`,
`TickDriverFakes.RecordingGameAndHosts.*`) and, for the `FreeRunning` fakes of §6, in `TickDriverFakesTest.cpp`. The
sampling games (§1) have no case of their own; `SimulationTickDriverSampleTest.cpp` runs them. Nor do the receiving
games (§1); `SimulationTickDriverReceiveTest.cpp` runs them.

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
| `FakeWorldHost` (`BasicFakeWorldHost<true>`), `FakeWorldHostNoDiagnostics` (`<false>`) | `TickDriverWorldHost`, `TickBackendWorldHost`, `TickBackendWorldBuild` (over two `FakeAdapter` optionals) | constructed over `(Events&, roleBit, stepsInline, ringDepthTicks = 8)` or `(Events&, roleBit, StepThreading, ringDepthTicks = 8)`, which configure its `TickDriverThreadContext` in the constructor, or over `(Events&, kConfigureAtBuild, ringDepthTicks = 8)`, which leaves it to `build`; `stepWorld()` is a `LoggedWorld`; `lockForStep()` and `lockWorld()` (both the world lock and the access scope, no step scope; §5), `lock()` (the lock, with its `locks()` count); the backend's members: `build(config, queryable, physicsOnly, bodyAdapter, queryAdapter)` (`Config` is `FakeWorldHostConfig`: `isAuthority`, `runInline`, and an optional `threading`; §4), `bindIntegratedTickSource`, `rootDeclarationIndex()` (returns `rootIndex`), `acquire` (the authority takes the next slot and publishes it; a client takes its replicated slot or waits), `bind` (under `lockWorld()`, calls the wait note with `bindLockWaitSeconds`), `release` (pushes the vacated slot), `slotOf`, `editVizQuery()`/`getVizReader()` (`vizQuery`, `vizReader`), `registerVizVolume`; the presenter's three doors: `hasShadowWorld()` (`shadowWorld`), `restore_GameThread` (calls the note with `restoreSeconds`, `restoreRefused`), `overwriteShadowPoses_GameThread`; records `afterTickOutcomes`, `afterRunTickSteps`, `builtStatics`, `integratedTick` |
| `FakeAdapter` | (an adapter optional's value) | constructed over `(Events&, kind, context)` by the fake's `build`; keeps the context's `stepWorldPredicate` it saw, and appends `emplace:<kind>`, or `emplace:<kind>:unconfigured` when the context had no predicate yet |
| `RealClockFakeEngine` | `TickDriverEngine` | `nowSeconds()` reads the real clock (§6.1); atomic `frameNumber()`/`advanceFrame()`; `minimumLevel` (atomic); `log` appends under a mutex and `snapshotLines()` copies the lines out; the executor never defers (§6.1); counts `inlineDispatches` |
| `FakeRunnerHost` (`BasicFakeRunnerHost<true>`), `FakeRunnerHostNoDiagnostics` (`<false>`) | `TickDriverRunnerHost` | constructed over `(BasicFakeEngine&, Events* = nullptr)`; `sleepUntil(s)` records `s` in `sleeps`, appends `sleepUntil` when it has an event list, and advances the engine's `now` to `max(now, s)` |
| `RealRunnerHost` | `TickDriverRunnerHost` | `sleepUntil(s)` sleeps the calling thread until the real clock reaches `s`; counts `sleepCalls` |
| `RecordingIdleGame` (`BasicRecordingIdleGame<true, ManagerT, StorageT, RenderSnapshotT>`), `RecordingIdleGameNoDiagnostics` (`<false>`) | `TickDriverIdleGame` | a `BasicRecordingGame` with `idleUntil(deadline)`, which follows an `IdleScript` (§6.2) and records an `IdleCall` in `idleCalls`; constructed over `(Events&, Manager&, Storage&, BasicFakeEngine&, IdleScript = HonourDeadline)`, or over `(Events&, Manager&, Storage&, IdleScript)` for the real-clock scripts; `setScript`, `wake()`, `beforePumpHook`, `setStopAfterSteps(stop, n)`, the atomic `stepsEnded`, `scriptExhaustedCalls` |
| `SamplingGame` (`BasicSamplingGame<RecordingGame>`), `SamplingIdleGame` (`BasicSamplingGame<RecordingIdleGame>`); `BasicSamplingGame<GameT>` over any recording game | `TickDriverSampleGame` (and `TickDriverIdleGame` over an idle game) | derives from `GameT` and adds `sampleInput()`, which appends one event and counts `sampleInputCalls`; constructed over `(Events&, …)`, the rest forwarded to `GameT`'s constructor, so a case picks the base game's manager type, as `BasicSamplingGame<BasicRecordingGame<TestManager>>`. `RecordingGame` and `RecordingIdleGame` themselves do not sample (a `static_assert`), so no existing trace gains an event |
| `ReceivingGame` (`BasicReceivingGame<RecordingGame>`); `BasicReceivingGame<GameT>` over any recording game | `TickDriverReceiveGame` (and `TickDriverIdleGame` over an idle game) | derives from `GameT` and adds `receiveForStep()`, which appends one event, counts `receiveForStepCalls` and then calls the injectable `onReceive` (a case's "network": it writes what the step should then read); constructed like `BasicSamplingGame`. `RecordingGame`, `RecordingIdleGame` and `SamplingGame` do not receive (a `static_assert`), so no existing trace gains an event |
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
| `RecordingIdleGame` | the `RecordingGame` events, and `idleUntil:<wake>` (`Deadline`, `Activity` or `Unavailable`) after each idle; `beforePumpHook` runs after `beforePump` is recorded |
| `SamplingGame`, `SamplingIdleGame` | the base game's events, and `sampleInput` for each `sampleInput()` |
| `ReceivingGame` | the base game's events, and `receiveForStep` for each `receiveForStep()`, before `onReceive` runs |
| `FakeRunnerHost` | `sleepUntil`, when constructed with an event list |
| `FakePresenter` | `captureCorrection:<physicsStep>`, `publish:<tick>` |
| `FakeWorldHost` | `hostAfterTick:<tick>`, `afterRunTick:<physicsStep>`; and, for the backend's calls, `configure:<role>:<stepping>` (role `authority` or `client`, stepping `inline`, `worker` or `runner`), `bindIntegratedTickSource`, `rootDeclarationIndex`, `acquire:<key>:<slot>` or `acquire:<key>:wait`, `bind:<slot>:<id>`, `release:<key>:<slot>:<id>`, `registerVizVolume:<slot>`, `editVizQuery`, `getVizReader`, `restore:<frame>`, `overwriteShadowPoses` |
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
configures at world build, so its predicates hold for the constructing thread at once. The constructor takes
the stepping either as today's `stepsInline` bool or as a `StepThreading`. Built with
`kConfigureAtBuild` it calls it in `build` instead, before it emplaces the two adapters, as a real world host's
`build` does (BL1); a case then checks that order through the events and the adapters' predicates. `build` configures
`FakeWorldHostConfig::resolvedThreading()`: the config's `threading` when it is set, otherwise the value today's
fields give (`Inline` for an authority or a `runInline` client, else `FrameBounded`). The default is resolved in
`build`, not in a member initializer, because cases set `isAuthority` and `runInline` after constructing the
config. So a config without a `threading` configures the context exactly as before, with the same event. With
`StepThreading::FreeRunning` the host is configured on the thread that calls `build` (the runner, in a
`FreeRunning` case), its `entryCheck()` is that thread's step-world predicate, and the event's stepping is
`runner`. A `FreeRunning` client is refused by the context's own check. Its destructor
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

## §6 The `FreeRunning` kit

The fakes for the tick driver's `FreeRunning` mode: a runner thread that pumps, steps inline and idles in the
game's `idleUntil` hook between deadlines. Each fake models its concept by a `static_assert` at the end of the
header.

### §6.1 Two clocks

- **Virtual time** is `BasicFakeEngine::now`. `FakeRunnerHost` and the virtual idle scripts move it forward by
  assignment and never backward, so a single-thread case decides every wake time exactly.
- **Real time** is `realClock::seconds()`: `std::chrono::steady_clock` in seconds since the first call in the
  process (`realClock::origin()`). Counting from a process origin keeps the values small, so a `double` holds them
  to well under a nanosecond. `RealClockFakeEngine::nowSeconds()`, `RealRunnerHost`, and the `RealDeadline` and
  `RealSpin` scripts all read this one clock, so their deadlines agree.
- `realClock::at(seconds)` turns an engine-clock time into a `steady_clock` time point. It rounds up and adds one
  tick, so a wait that ends at that point reads back as at or after the deadline, even after the
  `double`-to-ticks-to-`double` round trip.

`RealClockFakeEngine` is the engine a real-thread `FreeRunning` case runs over. Its clock, frame counter, log level
and log lines are safe to use from the runner and the test thread at once. Read the lines through
`snapshotLines()` while the runner runs. Its executor is the one a `FreeRunning` authority never uses (it steps
inline), reduced to the simplest shape that models the concept: `dispatchAfter` runs the work at once on the
calling thread and returns a handle that is already done, `isPending` is false, `waitFor` does nothing, and
`deferUntil` returns false. `inlineDispatches` counts the dispatches, so a case can show there were none.

### §6.2 The idle scripts

`BasicRecordingIdleGame::idleUntil(deadline)` records `IdleCall{deadline, now, returnedAt, wake}` and then
follows its `IdleScript`, one of the types in namespace `idleScript`:

| script | clock | what `idleUntil` does |
|---|---|---|
| `HonourDeadline` | virtual | if the deadline is ahead, sets `now` to exactly the deadline; otherwise advances one pass (`kOnePassSeconds`, 50 µs). Returns `Deadline` |
| `Wakes{times}` | virtual | advances to the next scripted time, whatever the deadline. Returns `Activity` if that is before the deadline, else `Deadline`. When the script runs out it honours the deadline and counts `scriptExhaustedCalls` |
| `ActivityAt{times}` | virtual | the end is the deadline if it is ahead, else one pass. If the next scripted time is before the end, advances to it, consumes it and returns `Activity`; otherwise advances to the end and returns `Deadline`. A time equal to the deadline is left for the next call, which returns `Activity` at once |
| `Unavailable` | virtual | returns `Unavailable` at once, without advancing |
| `EarlyDeadline` | virtual | the broken idle: advances 1 µs (`kEarlyDeadlineSeconds`) and returns `Deadline` |
| `RealDeadline` | real | waits on a condition variable until the deadline or a `wake()`. Returns `Activity` on a wake and `Deadline` otherwise. A deadline that has passed returns at once |
| `RealSpin` | real | spins on the real clock until it reaches the deadline. Returns `Deadline`. Nothing interrupts it (see the bullet below) |

- The virtual scripts need the `BasicFakeEngine` they advance, so they use the constructor that takes it. The game
  `OG_CHECK`s that it has one.
- The two real scripts read the real clock, so the constructor without an engine serves them.
- `setScript` swaps the script and restarts its list. `wake()` may be called from any thread. A wake that arrives
  while the game is not idling is kept, and the next `RealDeadline` idle returns `Activity` at once.
- `RealSpin` reads only the real clock: it spins until its deadline, and neither `wake()` nor a stop flag
  interrupts it, so a runner idling in it cannot be stopped before that deadline. A `wake()` that arrives during
  the spin is kept for the next `RealDeadline` idle. A case that wants a prompt stop or wake uses `RealDeadline`.
- `TickDriverFakes.RealDeadline.AWakeFromAnotherThreadReturnsActivity` mostly exercises the kept-wake path: the waker thread
  usually calls `wake()` before `idleUntil` starts to wait. Both paths return `Activity`.

The idle game also carries two controls for loop cases:

- `beforePumpHook`, a `std::function<void()>` run inside `beforePump` after its event is recorded. A case uses it
  for work that has to run on the runner before a pump.
- `setStopAfterSteps(stop, n)`, which sets the given `std::atomic<bool>` once `onStepEnded` has run `n` times.
  `stepsEnded` counts those calls atomically, so a test thread can wait on it while the runner steps.

`idleCalls`, the event list and the base game's records are written on the thread that runs the game, without a
lock. Read them after the runner has been joined, as §3's warning says for the worker thread.

### §6.3 The runner hosts

`FakeRunnerHost::sleepUntil(s)` stands for the runner's bounded sleep in virtual time. It records `s` and moves
the engine to `max(now, s)`: a sleep never moves the clock backward, and a sleep to a time already passed returns
at once. `RealRunnerHost::sleepUntil(s)` is `std::this_thread::sleep_until` on the real clock.

### §6.4 Tests

`TickDriverFakesTest.cpp` pins each script and host:
- `TickDriverFakes.HonourDeadline.LandsExactlyOnTheDeadline`;
- `TickDriverFakes.Wakes.GiveActivityForAnEarlyScriptedTimeAndDeadlineOtherwise`;
- `TickDriverFakes.ActivityAt.StopsAtTheEarlierOfTheActivityAndTheDeadline`;
- `TickDriverFakes.Unavailable.ReturnsAtOnceWithoutAdvancing`;
- `TickDriverFakes.EarlyDeadline.AdvancesOneMicrosecondAndReturnsDeadline`;
- `TickDriverFakes.FakeRunnerHost.SleepUntilAdvancesTheEngineClock`;
- `TickDriverFakes.RealDeadline.NeverReturnsDeadlineBeforeTheDeadline`, which checks the lower bound only (no upper
  bound, so it cannot flake), and `TickDriverFakes.RealDeadline.AWakeFromAnotherThreadReturnsActivity`;
- `TickDriverFakes.RealSpin.ReturnsDeadlineAtOrAfterTheDeadline` and
  `TickDriverFakes.RealRunnerHost.SleepUntilReturnsAtOrAfterTheTime`;
- `TickDriverFakes.RealClockFakeEngine.RunsDispatchedWorkInlineAndDefersNothing`;
- `TickDriverFakes.RecordingIdleGame.RunsTheBeforePumpHookAndSetsTheStopAfterNSteps`;
- `TickDriverFakes.FakeWorldHost.BuildConfiguresTheConfigsThreading`: a config without a `threading` gives the
  same stepping as before, and `FreeRunning` gives `runner`.
