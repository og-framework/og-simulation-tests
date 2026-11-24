// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <cstdint>
#include <string>
#include <string_view>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Network/SimWireToken.h"

namespace simWireTokenTest
{
consteval simWireToken::Token sixFieldToken(uint8_t correction, uint8_t ring, uint8_t bundle, uint8_t envelope,
	uint8_t roster, uint8_t clock)
{
	return simWireToken::join({ simWireToken::tokenOf({ { "c", correction }, { "r", ring }, { "b", bundle }, { "e", envelope } }),
		simWireToken::tokenOf({ { "ro", roster }, { "ck", clock } }) });
}

static_assert(simWireToken::kSimFragment.view() == "c15.r3.b3.e1",
	"og-session-runtime task 21: og-simulation's wire-token fragment is c<correction>.r<ring>.b<bundle>.e<envelope> "
	"from its own four version constants. A version bump changes this string and the game's whole-token pin.");

static_assert(sixFieldToken(15, 3, 3, 1, 1, 2).view() == "c15.r3.b3.e1.ro1.ck2"
		&& sixFieldToken(6, 2, 2, 1, 1, 1).view() == "c6.r2.b2.e1.ro1.ck1"
		&& sixFieldToken(10, 2, 3, 0, 12, 0).view() == "c10.r2.b3.e0.ro12.ck0"
		&& sixFieldToken(255, 100, 0, 9, 1, 10).view() == "c255.r100.b0.e9.ro1.ck10"
		&& sixFieldToken(255, 255, 255, 255, 255, 255).view() == "c255.r255.b255.e255.ro255.ck255"
		&& sixFieldToken(255, 255, 255, 255, 255, 255).view().size() == 31,
	"Was the brawler's wireTokenFor static_assert (og-sim-metres-yup task 15's c<correction>.r<ring>.b<bundle>, "
	"og-input-latency P5-6's .e<envelope>.ro<roster>, og-time-dilation task 2's .ck<clock>): each version in decimal "
	"without leading zeros, and the longest token (31 characters) fits the 32-character buffer.");

static_assert(simWireToken::join({ simWireToken::tokenOf({ { "c", 1 } }), simWireToken::Token{},
				  simWireToken::tokenOf({ { "ro", 2 } }) })
			.view()
		== "c1.ro2",
	"og-session-runtime task 21: an empty fragment adds no field and no separator, so a part with no wire version "
	"of its own can take part in the join.");
} // namespace simWireTokenTest

TEST_CASE("SimWireToken.TheSimFragmentIsTheFourOgSimulationVersions", "[Network][SimWireToken]")
{
	CHECK(std::string(simWireToken::kSimFragment.view()) == "c15.r3.b3.e1");
	CHECK(simWireTokenTest::sixFieldToken(255, 255, 255, 255, 255, 255).view().size() < simWireToken::kCapacity);
}

#endif // WITH_LOW_LEVEL_TESTS
