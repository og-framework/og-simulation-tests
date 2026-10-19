// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstddef>
#include <iterator>
#include <string>

#include "catch_amalgamated.hpp"
#include "OGSimulation/SimulationLogRoutes.h"

namespace simulationLogRoutesTest
{
    struct PinnedRow
    {
        const char*        prefix;
        SimulationLogRoute route;
        HostLogLevel       plainLevel;
    };

    constexpr PinnedRow kPinnedRows[] = {
        { "[Resim.Input]",                  SimulationLogRoute::SimTick,         HostLogLevel::Verbose },
        { "[TimeResync.",                   SimulationLogRoute::Sim,             HostLogLevel::Log },
        { "[Resim.",                        SimulationLogRoute::Sim,             HostLogLevel::Log },
        { "[ResimCheck.Divergence]",        SimulationLogRoute::Sim,             HostLogLevel::Log },
        { "[ResimCheck.PrepareRestore]",    SimulationLogRoute::Sim,             HostLogLevel::Log },
        { "[ResimCheck.Check]",             SimulationLogRoute::SimTick,         HostLogLevel::Verbose },
        { "[ResimCheck.IsSimilar]",         SimulationLogRoute::SimTick,         HostLogLevel::Verbose },
        { "[AuthoritySimulation]",          SimulationLogRoute::SimTick,         HostLogLevel::Verbose },
        { "[PredictionSimulation]",         SimulationLogRoute::SimTick,         HostLogLevel::Verbose },
        { "[PostPrediction]",               SimulationLogRoute::SimTick,         HostLogLevel::Verbose },
        { "[CollectInput]",                 SimulationLogRoute::SimTick,         HostLogLevel::Verbose },
        { "[ReceiveLocalInput] DISCARD",    SimulationLogRoute::Net,             HostLogLevel::Log },
        { "[ReceiveLocalInput]",            SimulationLogRoute::Net,             HostLogLevel::Verbose },
        { "[SendCorrectionStateToClients]", SimulationLogRoute::Net,             HostLogLevel::Verbose },
        { "[SendLocalInputToServer]",       SimulationLogRoute::Net,             HostLogLevel::Verbose },
        { "[InjectCorrectionState]",        SimulationLogRoute::Net,             HostLogLevel::Verbose },
        { "[InputGap]",                     SimulationLogRoute::Net,             HostLogLevel::Log },
        { "[InputDrop]",                    SimulationLogRoute::Net,             HostLogLevel::Log },
        { "[DelayShift]",                   SimulationLogRoute::Net,             HostLogLevel::Log },
        { "[InputStats]",                   SimulationLogRoute::Net,             HostLogLevel::Log },
        { "[Park]",                         SimulationLogRoute::Net,             HostLogLevel::Verbose },
        { "[Release]",                      SimulationLogRoute::Net,             HostLogLevel::Verbose },
        { "[InputDomain]",                  SimulationLogRoute::Net,             HostLogLevel::Log },
        { "[RelaySkip]",                    SimulationLogRoute::Net,             HostLogLevel::Log },
        { "[DivergenceProbe",               SimulationLogRoute::DivergenceProbe, HostLogLevel::Log },
    };

    const SimulationLogRouteEntry* firstMatch(const char* line)
    {
        for (const SimulationLogRouteEntry& entry : kSimulationLogRoutes)
        {
            if (simulationLogRoutes::startsWithIgnoringCase(line, entry.prefix))
                return &entry;
        }
        return nullptr;
    }

    std::string prefixOfFirstMatch(const char* line)
    {
        const SimulationLogRouteEntry* entry = firstMatch(line);
        return entry != nullptr ? std::string(entry->prefix) : std::string("<none>");
    }
} // namespace simulationLogRoutesTest

TEST_CASE("SimulationLogRoutes.Rows.PrefixRouteAndPlainLevelArePinnedInOrder", "[TickDriver]")
{
    using namespace simulationLogRoutesTest;

    REQUIRE(std::size(kSimulationLogRoutes) == std::size(kPinnedRows));
    REQUIRE(std::size(kSimulationLogRoutes) == 25u);
    for (std::size_t row = 0; row < std::size(kPinnedRows); ++row)
    {
        INFO("row " << row << " pinned " << kPinnedRows[row].prefix);
        CHECK(std::string(kSimulationLogRoutes[row].prefix) == kPinnedRows[row].prefix);
        CHECK(kSimulationLogRoutes[row].route == kPinnedRows[row].route);
        CHECK(kSimulationLogRoutes[row].plainLevel == kPinnedRows[row].plainLevel);
    }
}

TEST_CASE("SimulationLogRoutes.Checks.HoldForTheTableAndRejectAShadowedRowOrAWarningLevel", "[TickDriver]")
{
    STATIC_REQUIRE(simulationLogRoutes::allReachable(kSimulationLogRoutes));
    STATIC_REQUIRE(simulationLogRoutes::plainLevelsAreLogOrVerbose(kSimulationLogRoutes));

    constexpr SimulationLogRouteEntry catchAllAboveItsSpecificRow[] = {
        { "[Resim.",       SimulationLogRoute::Sim,     HostLogLevel::Log },
        { "[Resim.Input]", SimulationLogRoute::SimTick, HostLogLevel::Verbose },
    };
    STATIC_REQUIRE_FALSE(simulationLogRoutes::allReachable(catchAllAboveItsSpecificRow));

    constexpr SimulationLogRouteEntry discardBelowItsCatchAll[] = {
        { "[ReceiveLocalInput]",         SimulationLogRoute::Net, HostLogLevel::Verbose },
        { "[ReceiveLocalInput] DISCARD", SimulationLogRoute::Net, HostLogLevel::Log },
    };
    STATIC_REQUIRE_FALSE(simulationLogRoutes::allReachable(discardBelowItsCatchAll));

    constexpr SimulationLogRouteEntry lowerCaseDuplicate[] = {
        { "[Resim.Input]", SimulationLogRoute::SimTick, HostLogLevel::Verbose },
        { "[resim.input]", SimulationLogRoute::Sim,     HostLogLevel::Log },
    };
    STATIC_REQUIRE_FALSE(simulationLogRoutes::allReachable(lowerCaseDuplicate));

    constexpr SimulationLogRouteEntry specificRowFirst[] = {
        { "[Resim.Input]", SimulationLogRoute::SimTick, HostLogLevel::Verbose },
        { "[Resim.",       SimulationLogRoute::Sim,     HostLogLevel::Log },
    };
    STATIC_REQUIRE(simulationLogRoutes::allReachable(specificRowFirst));

    constexpr SimulationLogRouteEntry warningRow[] = {
        { "[Park]", SimulationLogRoute::Net, HostLogLevel::Warning },
    };
    STATIC_REQUIRE_FALSE(simulationLogRoutes::plainLevelsAreLogOrVerbose(warningRow));
}

TEST_CASE("SimulationLogRoutes.FirstMatch.TakesTheSpecificRowIgnoresCaseAndLeavesHostTagsUnrouted", "[TickDriver]")
{
    using namespace simulationLogRoutesTest;

    CHECK(prefixOfFirstMatch("[Resim.Input] id=3 tick=120") == "[Resim.Input]");
    CHECK(prefixOfFirstMatch("[resim.input] id=3") == "[Resim.Input]");
    CHECK(prefixOfFirstMatch("[Resim.Post] tick=120") == "[Resim.");
    CHECK(prefixOfFirstMatch("[ResimCheck.Check] tick=120") == "[ResimCheck.Check]");
    CHECK(prefixOfFirstMatch("[ReceiveLocalInput] DISCARD tick=900") == "[ReceiveLocalInput] DISCARD");
    CHECK(prefixOfFirstMatch("[ReceiveLocalInput] tick=900") == "[ReceiveLocalInput]");
    CHECK(prefixOfFirstMatch("[DivergenceProbe.Window] n=4") == "[DivergenceProbe");
    CHECK(prefixOfFirstMatch("[TimeResync.Wipe] from=1") == "[TimeResync.");

    CHECK(prefixOfFirstMatch("[ResimProbe.Gate] x=1") == "<none>");
    CHECK(prefixOfFirstMatch("[RelayProbe.Write] x=1") == "<none>");
    CHECK(prefixOfFirstMatch("[ServerReceive] id=1") == "<none>");
    CHECK(prefixOfFirstMatch("[ReleaseBatch] first=1") == "<none>");
    CHECK(prefixOfFirstMatch("[ResimCheck.TriggerRewind] rewind=0") == "<none>");
    CHECK(prefixOfFirstMatch("[Resim") == "<none>");
    CHECK(prefixOfFirstMatch("") == "<none>");

    const SimulationLogRouteEntry* resimInput = firstMatch("[Resim.Input] id=3");
    REQUIRE(resimInput != nullptr);
    CHECK(resimInput->route == SimulationLogRoute::SimTick);
    CHECK(resimInput->plainLevel == HostLogLevel::Verbose);
}

#endif // WITH_LOW_LEVEL_TESTS
