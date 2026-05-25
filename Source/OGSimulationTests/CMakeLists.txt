# SPDX-License-Identifier: MPL-2.0
# Catch2, og_simulation, and glm::glm-header-only must be provided by the
# parent CMake (e.g. og-tests-cmake-runner). Standalone configure will fail
# without them — this is the intended honest-failure-mode for a source dist.

file(GLOB_RECURSE OGSIM_TESTS_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)

add_executable(og_simulation_tests ${OGSIM_TESTS_SOURCES})

target_compile_definitions(og_simulation_tests PRIVATE
    WITH_LOW_LEVEL_TESTS=1
)

target_link_libraries(og_simulation_tests PRIVATE og_simulation Catch2 glm::glm-header-only)

add_test(NAME og_simulation_tests COMMAND og_simulation_tests)
