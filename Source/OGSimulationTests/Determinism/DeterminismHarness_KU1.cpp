// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "DeterminismHarness.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iterator>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <utility>
#include <vector>

//////////////////////////////////////////////////////////////////////////////
// Determinism harness — KU1CrossArch-mode tests (proposal §9.1 / Backlog Task 4).
//
// Tagged [Determinism][KU1CrossArch]. This tag is intentionally NOT in the [@og]
// alias (OgTagAliases.cpp), so these are OPT-IN only — `oglltest simulation`
// (default [@og]) does NOT pick them up. Run them explicitly:
//     oglltest simulation -Filter '[Determinism][KU1CrossArch]'
//
// Purpose (KU-1 cross-arch determinism scaffolding):
//   - HashLogToFile      — runs the SAME seeded 600-tick input grid as DevTest's
//                          RandomInputGrid600Ticks (seed 0x0FB1AC1E, same trivial
//                          POD types + integrate functor) and writes a binary
//                          hash log {build_id header + (tick, checksum) records}
//                          to env OG_KU1_HASH_LOG (default ./ku1_hash_log_<arch>_
//                          <compiler>.bin). The build_id + default filename are
//                          derived from compile-time macros, so an MSVC-Win64 run
//                          and a Clang-ARM64-NDK run write DISTINCT files.
//   - HashLogReplayMatch — reads two hash-log paths from env OG_KU1_HASH_LOG_A /
//                          OG_KU1_HASH_LOG_B, compares their (tick, checksum)
//                          records, and reports the first divergent tick. The
//                          same path on both env vars produces zero divergences.
//
// SCOPE BOUNDARY (FINAL doc §6): this file delivers the harness MODE only. Actual
// cross-arch measurement (running the harness on real ARM64 hardware to validate
// KU-1) is parallel-track research and OUT OF SCOPE here. Under Option 4 (current
// strategy) cross-arch divergence is EXPECTED — HashLogReplayMatch therefore
// reports magnitude (and does not fail) when the two paths are distinct files,
// and only asserts zero divergence for the same-file self-comparison.
// See docs/cross-arch-determinism.md for the full flow.
//////////////////////////////////////////////////////////////////////////////

namespace
{
    // --- Test sim types + integrate functor (parity with the DevTest grid) -----
    struct TestState { std::int64_t accumulator = 0; };
    struct TestInput { std::int64_t increment = 0; };

    using Cache = StateCorrectionCache<TestState, TestInput>;

    // Same seed as DeterminismHarness_DevTest.cpp's RandomInputGrid600Ticks, so a
    // KU1 hash log and a DevTest run share the exact same input stream (Task 4).
    constexpr std::uint32_t kKu1Seed = 0x0FB1AC1Eu;
    constexpr std::size_t   kKu1TickCount = 600u;

    std::function<void(const char*)> nullLogger()
    {
        return [](const char*) {};
    }

    Cache::IntegrateFn accumulateIntegrate()
    {
        return [](std::uint32_t, const TestState& prev, const TestInput& in)
        {
            return TestState{ prev.accumulator + in.increment };
        };
    }

    // Deterministic pseudo-random input grid (mirrors seededGrid in the DevTest
    // file): a fresh std::mt19937 seeded with the same value always yields the
    // same sequence. Kept file-local to avoid coupling the test files.
    std::vector<TestInput> seededGrid(std::size_t count, std::uint32_t seed)
    {
        std::mt19937 rng(seed);
        std::uniform_int_distribution<std::int64_t> dist(-1000, 1000);

        std::vector<TestInput> grid;
        grid.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
            grid.push_back(TestInput{ dist(rng) });
        return grid;
    }

    std::span<const TestInput> asSpan(const std::vector<TestInput>& v)
    {
        return std::span<const TestInput>(v.data(), v.size());
    }

    // Drives the canonical KU1 grid through the cache and returns the per-tick
    // CRC-32 checksum sequence. Deterministic for a given (seed, integrate).
    std::vector<std::uint32_t> computeReferenceChecksums()
    {
        const std::vector<TestInput> grid = seededGrid(kKu1TickCount, kKu1Seed);
        Cache cache(nullLogger(), accumulateIntegrate());
        std::vector<std::uint32_t> checksums;
        og::determinism::runDeterminismLoop(cache, asSpan(grid), 1u, checksums);
        return checksums;
    }

    // --- Compile-time build identity ------------------------------------------
    // build_id encodes a 16-bit compiler tag in the high bits + a version number
    // in the low bits, so distinct toolchains produce distinct ids automatically.
    // __clang__ is checked BEFORE _MSC_VER because clang-cl defines both.
    constexpr std::uint64_t computeBuildId()
    {
#if defined(__clang__)
        return (std::uint64_t(0x434Cu) << 48) // 'CL'
             | std::uint64_t(__clang_major__ * 10000 + __clang_minor__ * 100 + __clang_patchlevel__);
#elif defined(_MSC_VER)
        return (std::uint64_t(0x4D53u) << 48) // 'MS'
             | std::uint64_t(_MSC_FULL_VER);
#elif defined(__GNUC__)
        return (std::uint64_t(0x4743u) << 48) // 'GC'
             | std::uint64_t(__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__);
#else
        return std::uint64_t(0xFFFFu) << 48;
#endif
    }

    // <arch>_<compiler> tag for the default hash-log filename. Guarantees that an
    // MSVC-Win64 run ("x64_msvc") and a Clang-ARM64-NDK run ("arm64_clang") write
    // to different default files.
    inline const char* archCompilerTag()
    {
#if defined(__clang__)
    #if defined(__aarch64__)
        return "arm64_clang";
    #elif defined(__x86_64__)
        return "x64_clang";
    #else
        return "unknownarch_clang";
    #endif
#elif defined(_MSC_VER)
    #if defined(_M_ARM64)
        return "arm64_msvc";
    #elif defined(_M_X64)
        return "x64_msvc";
    #else
        return "unknownarch_msvc";
    #endif
#elif defined(__GNUC__)
    #if defined(__aarch64__)
        return "arm64_gcc";
    #elif defined(__x86_64__)
        return "x64_gcc";
    #else
        return "unknownarch_gcc";
    #endif
#else
        return "unknownarch_unknowncc";
#endif
    }

    // --- Portable environment read (avoids MSVC C4996 on std::getenv) ----------
    std::optional<std::string> getEnvVar(const char* name)
    {
#if defined(_MSC_VER)
        char* buf = nullptr;
        std::size_t len = 0;
        if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr)
            return std::nullopt;
        std::string value(buf);
        std::free(buf);
        return value;
#else
        const char* v = std::getenv(name);
        if (v == nullptr)
            return std::nullopt;
        return std::string(v);
#endif
    }

    std::string resolveDefaultHashLogPath()
    {
        if (const std::optional<std::string> env = getEnvVar("OG_KU1_HASH_LOG"))
            return *env;
        return std::string("./ku1_hash_log_") + archCompilerTag() + ".bin";
    }

    // --- Binary hash-log format ------------------------------------------------
    // Layout (little-endian, both target archs are LE):
    //   [0]  magic           "KU1H"        (4 bytes)
    //   [4]  formatVersion   u32           (=kHashLogFormatVersion)
    //   [8]  buildId         u64
    //   [16] tickCount       u32
    //   [20] records[tickCount]: { tick u32, checksum u32 }  (8 bytes each)
    constexpr std::uint32_t kHashLogFormatVersion = 1u;
    constexpr std::size_t   kHashLogHeaderSize = 20u;

    void putU32(std::vector<std::uint8_t>& out, std::uint32_t v)
    {
        for (int i = 0; i < 4; ++i)
            out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFu));
    }

    void putU64(std::vector<std::uint8_t>& out, std::uint64_t v)
    {
        for (int i = 0; i < 8; ++i)
            out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFu));
    }

    std::uint32_t getU32(const std::uint8_t* p)
    {
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i)
            v |= static_cast<std::uint32_t>(p[i]) << (8 * i);
        return v;
    }

    std::uint64_t getU64(const std::uint8_t* p)
    {
        std::uint64_t v = 0;
        for (int i = 0; i < 8; ++i)
            v |= static_cast<std::uint64_t>(p[i]) << (8 * i);
        return v;
    }

    struct ParsedLog
    {
        std::uint64_t buildId = 0;
        std::vector<std::pair<std::uint32_t, std::uint32_t>> records; // (tick, checksum)
    };

    bool writeHashLog(const std::string& path,
                      std::uint64_t buildId,
                      const std::vector<std::uint32_t>& checksums)
    {
        std::vector<std::uint8_t> bytes;
        bytes.reserve(kHashLogHeaderSize + checksums.size() * 8u);
        bytes.push_back(static_cast<std::uint8_t>('K'));
        bytes.push_back(static_cast<std::uint8_t>('U'));
        bytes.push_back(static_cast<std::uint8_t>('1'));
        bytes.push_back(static_cast<std::uint8_t>('H'));
        putU32(bytes, kHashLogFormatVersion);
        putU64(bytes, buildId);
        putU32(bytes, static_cast<std::uint32_t>(checksums.size()));

        std::uint32_t tick = 1u;
        for (const std::uint32_t checksum : checksums)
        {
            putU32(bytes, tick);
            putU32(bytes, checksum);
            ++tick;
        }

        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out)
            return false;
        out.write(reinterpret_cast<const char*>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
        return static_cast<bool>(out);
    }

    std::optional<ParsedLog> readHashLog(const std::string& path)
    {
        std::ifstream in(path, std::ios::binary);
        if (!in)
            return std::nullopt;

        const std::vector<std::uint8_t> bytes(
            (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

        if (bytes.size() < kHashLogHeaderSize)
            return std::nullopt;
        if (!(bytes[0] == 'K' && bytes[1] == 'U' && bytes[2] == '1' && bytes[3] == 'H'))
            return std::nullopt;
        if (getU32(&bytes[4]) != kHashLogFormatVersion)
            return std::nullopt;

        ParsedLog log;
        log.buildId = getU64(&bytes[8]);
        const std::uint32_t count = getU32(&bytes[16]);

        std::size_t offset = kHashLogHeaderSize;
        log.records.reserve(count);
        for (std::uint32_t i = 0; i < count; ++i)
        {
            if (offset + 8u > bytes.size())
                return std::nullopt; // truncated / malformed
            const std::uint32_t tick = getU32(&bytes[offset]);
            const std::uint32_t checksum = getU32(&bytes[offset + 4u]);
            log.records.emplace_back(tick, checksum);
            offset += 8u;
        }
        return log;
    }

    // First tick at which the two logs' checksums diverge (or differ in length).
    // nullopt iff the two record sequences are identical.
    std::optional<std::uint32_t> firstDivergentTick(const ParsedLog& a, const ParsedLog& b)
    {
        const std::size_t common = std::min(a.records.size(), b.records.size());
        for (std::size_t i = 0; i < common; ++i)
        {
            if (a.records[i].second != b.records[i].second)
                return a.records[i].first;
        }
        if (a.records.size() != b.records.size())
        {
            const ParsedLog& longer = (a.records.size() > b.records.size()) ? a : b;
            return longer.records[common].first; // first tick past the common prefix
        }
        return std::nullopt;
    }
} // namespace

TEST_CASE("Determinism.KU1.HashLogToFile", "[Determinism][KU1CrossArch]")
{
    // Run the canonical KU1 grid and persist a binary hash log. The build_id +
    // default filename are compile-time-derived, so distinct toolchains write
    // distinct files automatically (the cross-arch comparison reads them back).
    const std::vector<std::uint32_t> checksums = computeReferenceChecksums();
    REQUIRE(checksums.size() == kKu1TickCount);

    const std::string path = resolveDefaultHashLogPath();
    INFO("KU1 hash log path: " << path << " (build_id=" << computeBuildId() << ")");

    REQUIRE(writeHashLog(path, computeBuildId(), checksums));

    // The file must be non-empty and round-trip back to the same records.
    const std::optional<ParsedLog> readBack = readHashLog(path);
    REQUIRE(readBack.has_value());
    REQUIRE(readBack->buildId == computeBuildId());
    REQUIRE(readBack->records.size() == kKu1TickCount);
    REQUIRE(readBack->records.front().first == 1u);           // first tick
    REQUIRE(readBack->records.back().first == kKu1TickCount);  // last tick (600)
}

TEST_CASE("Determinism.KU1.HashLogReplayMatch", "[Determinism][KU1CrossArch]")
{
    // Compare two hash logs supplied via env. When both env vars are unset (the
    // local self-test path), write a fresh log and compare it against itself —
    // this is order-independent (does NOT rely on HashLogToFile having run) and
    // must report zero divergences.
    const std::optional<std::string> envA = getEnvVar("OG_KU1_HASH_LOG_A");
    const std::optional<std::string> envB = getEnvVar("OG_KU1_HASH_LOG_B");

    std::string pathA;
    std::string pathB;
    if (envA && envB)
    {
        pathA = *envA;
        pathB = *envB;
    }
    else
    {
        const std::string selfPath = "./ku1_hash_log_replay_selftest.bin";
        REQUIRE(writeHashLog(selfPath, computeBuildId(), computeReferenceChecksums()));
        pathA = selfPath;
        pathB = selfPath;
    }

    const std::optional<ParsedLog> logA = readHashLog(pathA);
    const std::optional<ParsedLog> logB = readHashLog(pathB);
    REQUIRE(logA.has_value());
    REQUIRE(logB.has_value());

    const std::optional<std::uint32_t> divergentTick = firstDivergentTick(*logA, *logB);
    const bool samePath = (pathA == pathB);

    if (samePath)
    {
        // Identical input → byte-identical checksum records → zero divergence.
        REQUIRE_FALSE(divergentTick.has_value());
    }
    else
    {
        // Genuine cross-arch comparison. Under Option 4 (current strategy)
        // divergence is EXPECTED; the harness reports magnitude rather than
        // failing the build (a future Option 3 commitment would gate-fail here).
        if (divergentTick.has_value())
        {
            WARN("KU1 cross-arch divergence: first divergent tick = " << *divergentTick
                 << " (build_id A=" << logA->buildId << " B=" << logB->buildId
                 << "). Under Option 4 this is expected — reporting magnitude, not failing.");
        }
        else
        {
            WARN("KU1 cross-arch logs matched with no divergence (build_id A="
                 << logA->buildId << " B=" << logB->buildId << ").");
        }
        SUCCEED("Cross-arch comparison completed; divergence reported as magnitude (Option 4).");
    }
}

#endif // WITH_LOW_LEVEL_TESTS
