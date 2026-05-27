<!-- SPDX-License-Identifier: MPL-2.0 -->
# og-simulation-tests

Catch2 test source for [og-simulation](https://github.com/og-framework/og-simulation).

This repo is a **pure source distribution** — it does not have a root `CMakeLists.txt` and is not directly buildable on its own. Consumers compose it via their own build systems.

## Position in the og-framework graph

```
og-simulation  (submoduled by consumers alongside these tests)
og-simulation-tests  (this repo — pure Catch2 test source)
    ↓ consumed by
og-tests-cmake-runner  — CMake assembly; runs ctest
og-brawler-unreal      — UE project; compiles as an LLT target
```

## Related repos

| Repo | Role |
|---|---|
| [og-simulation](https://github.com/og-framework/og-simulation) | The library under test |
| [og-tests-cmake-runner](https://github.com/og-framework/og-tests-cmake-runner) | CMake harness that builds and runs these tests |
| [og-brawler-unreal](https://github.com/og-framework/og-brawler-unreal) | UE project; `Source/OGSimulationTests/` LLT target submodules this repo |
| [og-brawler-tests](https://github.com/og-framework/og-brawler-tests) | Sibling test repo for the brawler layer |

## Quickstart

To build and run these tests, clone [og-tests-cmake-runner](https://github.com/og-framework/og-tests-cmake-runner) — it submodules this repo alongside og-simulation and Catch2 and assembles the full CMake build:

```bash
git clone --recurse-submodules https://github.com/og-framework/og-tests-cmake-runner
cd og-tests-cmake-runner
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: **218 assertions in 11 test cases** from `og_simulation_tests.exe`.

## Layout

```
Source/OGSimulationTests/   Catch2 test source (.cpp files + CMakeLists.txt)
Source/ThirdParty/Catch2/   Catch2 v3.14.0 amalgamation (used by og-tests-cmake-runner)
```

## Canonical workflow

See [`og-brawler-unreal/docs/cross-repo-dev-loop.md`](https://github.com/og-framework/og-brawler-unreal/blob/main/docs/cross-repo-dev-loop.md) for the multi-repo development workflow.

## License and contributing

[MPL-2.0](LICENSES/MPL-2.0.txt). Inbound = outbound.

See [CONTRIBUTING.md](https://github.com/og-framework/og-brawler-unreal/blob/main/CONTRIBUTING.md) for the decision tree on where to make your change.

