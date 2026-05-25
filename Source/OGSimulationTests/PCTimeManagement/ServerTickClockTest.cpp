// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "OGSimulation/PCTimeManagement/ServerTickClock.h"

// ---------------------------------------------------------------------------
// Minimal mock sync-buffer — mirrors the interface used by writeToSyncedBuffer
// and readFromSyncedBuffer. Uses a fixed 64-byte backing array.
// ---------------------------------------------------------------------------
struct FMockSyncBuffer
{
    unsigned char data[64] = {};

    template<typename T>
    void writeToBuffer(unsigned int byteOffset, const T& value)
    {
        static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
        memcpy(data + byteOffset, &value, sizeof(T));
    }

    template<typename T>
    T readFromBuffer(unsigned int byteOffset) const
    {
        static_assert(std::is_trivially_copyable<T>::value, "T must be trivially copyable");
        T value{};
        memcpy(&value, data + byteOffset, sizeof(T));
        return value;
    }
};

// ---------------------------------------------------------------------------
// Test: advance N times, getTick() == N
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ServerTickClock.AdvanceTick", "[PCTM][ServerTickClock]")
{
    ServerTickClock clock(/*deltaSeconds=*/1.0f / 60.0f);
    REQUIRE(clock.getTick() == 0u);

    constexpr unsigned int N = 100;
    for (unsigned int i = 0; i < N; ++i)
        clock.advanceTick();

    REQUIRE(clock.getTick() == N);

    // getSimulationStep() reflects the same tick, never resimulating on server
    const SimulationTimeStep step = clock.getSimulationStep();
    REQUIRE(step.getTick() == N);
    REQUIRE(!step.getIsResimulating());
}

// ---------------------------------------------------------------------------
// Test: writeToSyncedBuffer / readFromSyncedBuffer round-trip
// ---------------------------------------------------------------------------
TEST_CASE("PCTM.ServerTickClock.SerializationRoundTrip", "[PCTM][ServerTickClock]")
{
    // SyncSize must be 4 bytes
    REQUIRE(ServerTickClock::SyncSize() == 4u);

    ServerTickClock clock(/*deltaSeconds=*/1.0f / 60.0f);
    constexpr unsigned int AdvancedTick = 42;
    for (unsigned int i = 0; i < AdvancedTick; ++i)
        clock.advanceTick();

    FMockSyncBuffer buffer;
    const unsigned int bytesWritten = ServerTickClock::writeToSyncedBuffer(clock, buffer, 0);
    REQUIRE(bytesWritten == ServerTickClock::SyncSize());

    const SimulationTimeStep recovered = ServerTickClock::readFromSyncedBuffer(buffer, 0);
    REQUIRE(recovered.getTick() == AdvancedTick);
    REQUIRE(!recovered.getIsResimulating());
}

#endif // WITH_LOW_LEVEL_TESTS
