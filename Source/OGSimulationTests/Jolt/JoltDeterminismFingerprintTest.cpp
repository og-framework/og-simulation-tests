// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltDeterminismFingerprint.h"

#include <string>

uint64_t joltVersionIdWithCrossPlatformDeterminism();

using namespace joltTestRig;

namespace
{
    constexpr uint32_t kFeatureShift = 24;
    constexpr uint64_t kCrossPlatformDeterministicBit = uint64_t(1) << (kFeatureShift + 1u);
    constexpr uint64_t kEnableAssertsBit = uint64_t(1) << (kFeatureShift + 9u);

    bool contains(const std::string& text, const char* part)
    {
        return text.find(part) != std::string::npos;
    }
}

TEST_CASE("JoltDeterminismFingerprint.StableAcrossTwoRunsOfTheSameBinary", "[Jolt]")
{
    RuntimeLease lease;
    const JoltDeterminismFingerprint first = determinismFingerprint(lease.runtime);
    const JoltDeterminismFingerprint second = determinismFingerprint(lease.runtime);

    CHECK(first.value() == second.value());
    CHECK(first.probeStateHash == second.probeStateHash);
    CHECK(first.probeStateHash != 0u);
    CHECK(std::string(first.engineName) == "Jolt");
    CHECK(first.versionMajor == 5u);
    CHECK(first.versionMinor == 6u);
    CHECK(first.joltVersionId == JoltRuntime::libraryVersionId());
    CHECK(first.simdWidthBits >= 128u);
    CHECK(first.stepFlushToZero);
    CHECK(first.stepDenormalsAreZero);
    CHECK(first.stepRoundToNearest);
    CHECK(contains(describeFingerprintMismatch(first, second), "match"));
}

TEST_CASE("JoltDeterminismFingerprint.ChangesWhenCrossPlatformDeterminismIsToggled", "[Jolt]")
{
    const uint64_t libraryId = JoltRuntime::libraryVersionId();
    const uint64_t toggledId = joltVersionIdWithCrossPlatformDeterminism();
    REQUIRE((libraryId & kCrossPlatformDeterministicBit) == 0u);
    CHECK((libraryId ^ toggledId) == kCrossPlatformDeterministicBit);

    RuntimeLease lease;
    const JoltDeterminismFingerprint local = determinismFingerprint(lease.runtime);
    JoltDeterminismFingerprint toggled = local;
    toggled.joltVersionId = toggledId;
    CHECK(toggled.value() != local.value());

    const std::string message = describeFingerprintMismatch(local, toggled);
    INFO(message);
    CHECK(contains(message, "mismatch"));
    CHECK(contains(message, "JPH_CROSS_PLATFORM_DETERMINISTIC off locally, on remotely"));
}

TEST_CASE("JoltDeterminismFingerprint.MismatchMessageNamesTheBuildConfiguration", "[Jolt]")
{
    RuntimeLease lease;
    const JoltDeterminismFingerprint local = determinismFingerprint(lease.runtime);
    JoltDeterminismFingerprint debugPeer = local;
    debugPeer.joltVersionId = local.joltVersionId ^ kEnableAssertsBit;

    const bool localAsserts = (local.joltVersionId & kEnableAssertsBit) != 0u;
    const std::string message = describeFingerprintMismatch(local, debugPeer);
    INFO(message);
    CHECK(contains(message, "Debug configuration"));
    CHECK(contains(message, "Development/Shipping configuration"));
    CHECK(contains(message, localAsserts ? "JPH_ENABLE_ASSERTS on locally, off remotely" : "JPH_ENABLE_ASSERTS off locally, on remotely"));
    CHECK(contains(joltBuildConfigurationOf(local.joltVersionId), localAsserts ? "Debug" : "Development/Shipping"));

    JoltDeterminismFingerprint otherProbe = local;
    otherProbe.probeStateHash ^= 1u;
    CHECK(contains(describeFingerprintMismatch(local, otherProbe), "probe simulation state hash"));
}

#endif // WITH_LOW_LEVEL_TESTS
