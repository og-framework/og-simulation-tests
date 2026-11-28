// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <optional>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/CorrectionStaleFilter.h"

namespace correctionStaleFilterTest
{
    constexpr std::uint8_t sim(unsigned id) { return static_cast<std::uint8_t>(id); }

    template <class Filter, class... Key>
    std::vector<std::uint32_t> admittedOf(Filter& filter, const std::vector<std::uint32_t>& ticks, Key... key)
    {
        std::vector<std::uint32_t> admitted;
        for (const std::uint32_t tick : ticks)
        {
            if (filter.admit(key..., tick))
            {
                admitted.push_back(tick);
            }
        }
        return admitted;
    }
}

using correctionStaleFilterTest::admittedOf;
using correctionStaleFilterTest::sim;

TEST_CASE("CorrectionStaleFilter.InOrderTicksAreAdmitted", "[Network][CorrectionStaleFilter]")
{
    CorrectionStaleFilter filter;
    CHECK( filter.newest(sim(7)) == std::nullopt );

    const std::vector<std::uint32_t> ticks{0u, 1u, 2u, 5u, 6u, 100u};
    CHECK( admittedOf(filter, ticks, sim(7)) == ticks );
    CHECK( filter.newest(sim(7)) == std::optional<std::uint32_t>{100u} );
    CHECK( filter.staleDropped() == 0u );
    CHECK( filter.duplicateDropped() == 0u );
}

TEST_CASE("CorrectionStaleFilter.DuplicateAndOlderTicksAreRefusedAndCountedApart", "[Network][CorrectionStaleFilter]")
{
    CorrectionStaleFilter filter;
    REQUIRE( filter.admit(sim(3), 40u) );

    SECTION("a duplicate")
    {
        CHECK_FALSE( filter.admit(sim(3), 40u) );
        CHECK( filter.duplicateDropped() == 1u );
        CHECK( filter.staleDropped() == 0u );
    }
    SECTION("an older tick")
    {
        CHECK_FALSE( filter.admit(sim(3), 39u) );
        CHECK( filter.staleDropped() == 1u );
        CHECK( filter.duplicateDropped() == 0u );
    }
    SECTION("both, repeated")
    {
        CHECK_FALSE( filter.admit(sim(3), 40u) );
        CHECK_FALSE( filter.admit(sim(3), 0u) );
        CHECK_FALSE( filter.admit(sim(3), 40u) );
        CHECK_FALSE( filter.admit(sim(3), 12u) );
        CHECK_FALSE( filter.admit(sim(3), 39u) );
        CHECK( filter.duplicateDropped() == 2u );
        CHECK( filter.staleDropped() == 3u );
    }
    CHECK( filter.newest(sim(3)) == std::optional<std::uint32_t>{40u} );
    CHECK( filter.admit(sim(3), 41u) );
}

TEST_CASE("CorrectionStaleFilter.SimIdsAreIndependent", "[Network][CorrectionStaleFilter]")
{
    CorrectionStaleFilter filter;
    REQUIRE( filter.admit(sim(1), 500u) );

    CHECK( filter.admit(sim(2), 10u) );
    CHECK( filter.admit(sim(2), 11u) );
    CHECK_FALSE( filter.admit(sim(1), 499u) );
    CHECK( filter.admit(sim(0), 3u) );
    CHECK( filter.admit(sim(255), 4u) );
    CHECK_FALSE( filter.admit(sim(255), 4u) );

    CHECK( filter.newest(sim(0)) == std::optional<std::uint32_t>{3u} );
    CHECK( filter.newest(sim(1)) == std::optional<std::uint32_t>{500u} );
    CHECK( filter.newest(sim(2)) == std::optional<std::uint32_t>{11u} );
    CHECK( filter.newest(sim(254)) == std::nullopt );
    CHECK( filter.newest(sim(255)) == std::optional<std::uint32_t>{4u} );
    CHECK( filter.staleDropped() == 1u );
    CHECK( filter.duplicateDropped() == 1u );
}

TEST_CASE("CorrectionStaleFilter.ForgetReArms", "[Network][CorrectionStaleFilter]")
{
    CorrectionStaleFilter filter;
    REQUIRE( filter.admit(sim(9), 1000u) );
    REQUIRE( filter.admit(sim(10), 2000u) );
    REQUIRE_FALSE( filter.admit(sim(9), 999u) );

    filter.forget(sim(9));
    CHECK( filter.newest(sim(9)) == std::nullopt );
    CHECK( filter.newest(sim(10)) == std::optional<std::uint32_t>{2000u} );
    CHECK( filter.admit(sim(9), 999u) );
    CHECK( filter.admit(sim(9), 1000u) );
    CHECK_FALSE( filter.admit(sim(9), 1000u) );
    CHECK_FALSE( filter.admit(sim(10), 1999u) );
    CHECK( filter.staleDropped() == 2u );
    CHECK( filter.duplicateDropped() == 1u );
}

TEST_CASE("CorrectionStaleFilter.ReorderTraceAdmitsTenTwelveThirteen", "[Network][CorrectionStaleFilter]")
{
    CorrectionStaleFilter filter;
    CHECK( admittedOf(filter, {10u, 12u, 11u, 13u}, sim(4)) == std::vector<std::uint32_t>{10u, 12u, 13u} );
    CHECK( filter.staleDropped() == 1u );
    CHECK( filter.duplicateDropped() == 0u );
    CHECK( filter.newest(sim(4)) == std::optional<std::uint32_t>{13u} );
}

TEST_CASE("CorrectionStaleFilter.NewestTickFilterKeepsTheSameRuleForOneKey", "[Network][CorrectionStaleFilter]")
{
    NewestTickFilter filter;
    CHECK( filter.newest() == std::nullopt );

    CHECK( admittedOf(filter, {10u, 12u, 11u, 12u, 13u}) == std::vector<std::uint32_t>{10u, 12u, 13u} );
    CHECK( filter.staleDropped() == 1u );
    CHECK( filter.duplicateDropped() == 1u );
    CHECK( filter.newest() == std::optional<std::uint32_t>{13u} );

    filter.forget();
    CHECK( filter.newest() == std::nullopt );
    CHECK( filter.admit(2u) );
    CHECK( filter.newest() == std::optional<std::uint32_t>{2u} );
}

#endif
