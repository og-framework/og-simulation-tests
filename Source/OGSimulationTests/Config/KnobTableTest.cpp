// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <system_error>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/Config/KnobTable.h"

namespace knobTableTest
{
using knobTable::KnobLevel;
using knobTable::KnobRole;
using knobTable::KnobRow;
using knobTable::KnobWorld;

class ScriptedConfig
{
public:
	explicit ScriptedConfig(std::map<std::string, std::string> values = {})
		: m_values(std::move(values))
	{
	}

	std::optional<int32_t> readInt(const char* key) const
	{
		const std::string* text = find(key);
		if (text == nullptr)
			return std::nullopt;
		int32_t value = 0;
		const char* const end = text->data() + text->size();
		const std::from_chars_result parsed = std::from_chars(text->data(), end, value);
		if (parsed.ec != std::errc() || parsed.ptr != end)
			return std::nullopt;
		return value;
	}

	std::optional<double> readDouble(const char* key) const
	{
		const std::string* text = find(key);
		if (text == nullptr || text->empty())
			return std::nullopt;
		char* end = nullptr;
		const double value = std::strtod(text->c_str(), &end);
		if (end != text->c_str() + text->size())
			return std::nullopt;
		return value;
	}

	const std::vector<std::string>& reads() const { return m_reads; }

private:
	const std::string* find(const char* key) const
	{
		m_reads.emplace_back(key);
		const auto found = m_values.find(key);
		return found != m_values.end() ? &found->second : nullptr;
	}

	std::map<std::string, std::string> m_values;
	mutable std::vector<std::string>   m_reads;
};

struct ConfigWithoutReadDouble
{
	std::optional<int32_t> readInt(const char*) const;
};

struct ConfigWithAnIntWithoutAbsence
{
	int32_t                readInt(const char*) const;
	std::optional<double> readDouble(const char*) const;
};

static_assert(knobTable::KnobConfig<ScriptedConfig>);
static_assert(!knobTable::KnobConfig<ConfigWithoutReadDouble>);
static_assert(!knobTable::KnobConfig<ConfigWithAnIntWithoutAbsence>);
static_assert(!std::is_default_constructible_v<knobTable::KnobFn<void(int&, int32_t)>>);
static_assert(!std::is_constructible_v<knobTable::KnobFn<double(double)>, std::nullptr_t>);

struct Owner
{
	int32_t ticks      = 4;
	double  gain       = 0.5;
	int32_t serverOnly = 2;
	int32_t clientOnly = 6;
};

constexpr int32_t clampTicks(int32_t requested) { return requested < 0 ? 0 : (requested > 10 ? 10 : requested); }
constexpr double clampGain(double requested) { return requested < 0.0 ? 0.0 : (requested > 1.0 ? 1.0 : requested); }
constexpr int32_t clampAtLeastOne(int32_t requested) { return requested < 1 ? 1 : requested; }

inline constexpr KnobRow<Owner, int32_t> kTicksRow{
	.intake = { .section = "TestSection", .key = "TestTicks" },
	.tag    = "Test.Ticks",
	.name   = "ticks",
	.clamp  = &clampTicks,
	.apply  = [](Owner& owner, int32_t ticks) { owner.ticks = ticks; },
	.stored = [](const Owner& owner) { return owner.ticks; },
};

inline constexpr KnobRow<Owner, double> kGainRow{
	.intake     = { .section = "TestSection", .key = "TestGain" },
	.tag        = "Test.Gain",
	.name       = "gain",
	.proofLevel = KnobLevel::Log,
	.clamp      = &clampGain,
	.apply      = [](Owner& owner, double gain) { owner.gain = gain; },
	.stored     = [](const Owner& owner) { return owner.gain; },
};

inline constexpr KnobRow<Owner, int32_t> kServerOnlyRow{
	.intake = { .section = "TestSection", .key = "TestServerOnly", .role = KnobRole::Authority },
	.tag    = "Test.ServerOnly",
	.name   = "serverOnly",
	.clamp  = &clampAtLeastOne,
	.apply  = [](Owner& owner, int32_t value) { owner.serverOnly = value; },
	.stored = [](const Owner& owner) { return owner.serverOnly; },
};

inline constexpr KnobRow<Owner, int32_t> kClientOnlyRow{
	.intake = { .section = "TestSection", .key = "TestClientOnly", .role = KnobRole::Prediction },
	.tag    = "Test.ClientOnly",
	.name   = "clientOnly",
	.clamp  = &clampAtLeastOne,
	.apply  = [](Owner& owner, int32_t value) { owner.clientOnly = value; },
	.stored = [](const Owner& owner) { return owner.clientOnly; },
};

inline constexpr std::tuple kRows{ kTicksRow, kGainRow, kServerOnlyRow, kClientOnlyRow };

struct Run
{
	Owner                    owner;
	std::vector<std::string> lines;
	std::vector<std::string> reads;
};

Run applyAll(std::map<std::string, std::string> values, KnobWorld world)
{
	const ScriptedConfig config(std::move(values));
	Run run;
	knobTable::applyKnobs(kRows, config, run.owner, world, [&run](KnobLevel level, const char* line)
		{ run.lines.push_back(std::string(level == KnobLevel::Warning ? "Warning " : "Log ") + line); });
	run.reads = config.reads();
	return run;
}
} // namespace knobTableTest

using knobTableTest::applyAll;
using knobTableTest::Run;

TEST_CASE("KnobTable.EveryAbsentKeyProvesItsCompiledValueInRowOrder", "[KnobTable]")
{
	const Run authority = applyAll({}, knobTable::KnobWorld::Authority);
	CHECK(authority.reads == std::vector<std::string>{ "TestTicks", "TestGain", "TestServerOnly" });
	CHECK(authority.lines == std::vector<std::string>{ "Warning [Test.Ticks] ticks=4 (compiled)",
		"Log [Test.Gain] gain=0.5000 (compiled)", "Warning [Test.ServerOnly] serverOnly=2 (compiled)" });

	const Run prediction = applyAll({}, knobTable::KnobWorld::Prediction);
	CHECK(prediction.reads == std::vector<std::string>{ "TestTicks", "TestGain", "TestClientOnly" });
	CHECK(prediction.lines == std::vector<std::string>{ "Warning [Test.Ticks] ticks=4 (compiled)",
		"Log [Test.Gain] gain=0.5000 (compiled)", "Warning [Test.ClientOnly] clientOnly=6 (compiled)" });
}

TEST_CASE("KnobTable.AnInRangeValueIsAppliedAndProvedAsIni", "[KnobTable]")
{
	const Run run = applyAll({ { "TestTicks", "7" }, { "TestGain", "0.25" }, { "TestServerOnly", "3" } }, knobTable::KnobWorld::Authority);
	CHECK(run.owner.ticks == 7);
	CHECK(run.owner.gain == 0.25);
	CHECK(run.owner.serverOnly == 3);
	CHECK(run.lines == std::vector<std::string>{ "Warning [Test.Ticks] ticks=7 (ini)", "Log [Test.Gain] gain=0.2500 (ini)",
		"Warning [Test.ServerOnly] serverOnly=3 (ini)" });
}

TEST_CASE("KnobTable.AnOutOfRangeValueIsReportedClampedAndApplied", "[KnobTable]")
{
	const Run low = applyAll({ { "TestTicks", "-3" }, { "TestGain", "-0.5" }, { "TestServerOnly", "0" } }, knobTable::KnobWorld::Authority);
	CHECK(low.owner.ticks == 0);
	CHECK(low.owner.gain == 0.0);
	CHECK(low.owner.serverOnly == 1);
	CHECK(low.lines == std::vector<std::string>{
		"Warning [Test.Ticks] ini [TestSection] TestTicks=-3 out of range, clamped to 0", "Warning [Test.Ticks] ticks=0 (ini)",
		"Warning [Test.Gain] ini [TestSection] TestGain=-0.5000 out of range, clamped to 0.0000", "Log [Test.Gain] gain=0.0000 (ini)",
		"Warning [Test.ServerOnly] ini [TestSection] TestServerOnly=0 out of range, clamped to 1",
		"Warning [Test.ServerOnly] serverOnly=1 (ini)" });

	const Run high = applyAll({ { "TestTicks", "2147483647" }, { "TestGain", "2.5" } }, knobTable::KnobWorld::Prediction);
	CHECK(high.owner.ticks == 10);
	CHECK(high.owner.gain == 1.0);
	CHECK(high.lines == std::vector<std::string>{
		"Warning [Test.Ticks] ini [TestSection] TestTicks=2147483647 out of range, clamped to 10", "Warning [Test.Ticks] ticks=10 (ini)",
		"Warning [Test.Gain] ini [TestSection] TestGain=2.5000 out of range, clamped to 1.0000", "Log [Test.Gain] gain=1.0000 (ini)",
		"Warning [Test.ClientOnly] clientOnly=6 (compiled)" });

	const Run lowest = applyAll({ { "TestTicks", "-2147483648" } }, knobTable::KnobWorld::Prediction);
	CHECK(lowest.lines.front() == "Warning [Test.Ticks] ini [TestSection] TestTicks=-2147483648 out of range, clamped to 0");
}

TEST_CASE("KnobTable.ANonFiniteDoubleIsReportedAndTheCompiledValueKept", "[KnobTable]")
{
	for (const char* text : { "inf", "-inf", "nan" })
	{
		INFO(text);
		const Run run = applyAll({ { "TestGain", text } }, knobTable::KnobWorld::Authority);
		CHECK(run.owner.gain == 0.5);
		CHECK(run.lines == std::vector<std::string>{ "Warning [Test.Ticks] ticks=4 (compiled)",
			"Warning [Test.Gain] ini [TestSection] TestGain is not a finite number, keeping the compiled default",
			"Log [Test.Gain] gain=0.5000 (compiled)", "Warning [Test.ServerOnly] serverOnly=2 (compiled)" });
	}
}

TEST_CASE("KnobTable.ARowOfTheOtherRoleIsNeitherReadNorApplied", "[KnobTable]")
{
	const Run prediction = applyAll({ { "TestServerOnly", "9" } }, knobTable::KnobWorld::Prediction);
	CHECK(prediction.owner.serverOnly == 2);
	CHECK(prediction.reads == std::vector<std::string>{ "TestTicks", "TestGain", "TestClientOnly" });

	const Run authority = applyAll({ { "TestClientOnly", "9" } }, knobTable::KnobWorld::Authority);
	CHECK(authority.owner.clientOnly == 6);
	CHECK(authority.reads == std::vector<std::string>{ "TestTicks", "TestGain", "TestServerOnly" });
}

TEST_CASE("KnobTable.AValueTheSourceCannotParseIsAbsent", "[KnobTable]")
{
	const Run run = applyAll({ { "TestTicks", "4abc" }, { "TestGain", "" } }, knobTable::KnobWorld::Authority);
	CHECK(run.owner.ticks == 4);
	CHECK(run.owner.gain == 0.5);
	CHECK(run.lines.front() == "Warning [Test.Ticks] ticks=4 (compiled)");
}

TEST_CASE("KnobTable.ReadKnobThenApplyKnobIsApplyKnobs", "[KnobTable]")
{
	const std::map<std::string, std::string> values = { { "TestTicks", "12" }, { "TestGain", "nan" }, { "TestServerOnly", "5" } };
	const Run whole = applyAll(values, knobTable::KnobWorld::Authority);

	const knobTableTest::ScriptedConfig config(values);
	knobTableTest::Owner owner;
	std::vector<std::string> lines;
	auto log = [&lines](knobTable::KnobLevel level, const char* line)
	{ lines.push_back(std::string(level == knobTable::KnobLevel::Warning ? "Warning " : "Log ") + line); };
	const std::optional<int32_t> ticks = knobTable::readKnob(knobTableTest::kTicksRow.intake, config, knobTable::KnobWorld::Authority);
	const std::optional<double> gain = knobTable::readKnob(knobTableTest::kGainRow.intake, config, knobTable::KnobWorld::Authority);
	const std::optional<int32_t> serverOnly =
		knobTable::readKnob(knobTableTest::kServerOnlyRow.intake, config, knobTable::KnobWorld::Authority);
	const std::optional<int32_t> clientOnly =
		knobTable::readKnob(knobTableTest::kClientOnlyRow.intake, config, knobTable::KnobWorld::Authority);
	CHECK_FALSE(clientOnly.has_value());
	knobTable::applyKnob(knobTableTest::kTicksRow, ticks, owner, knobTable::KnobWorld::Authority, log);
	knobTable::applyKnob(knobTableTest::kGainRow, gain, owner, knobTable::KnobWorld::Authority, log);
	knobTable::applyKnob(knobTableTest::kServerOnlyRow, serverOnly, owner, knobTable::KnobWorld::Authority, log);
	knobTable::applyKnob(knobTableTest::kClientOnlyRow, clientOnly, owner, knobTable::KnobWorld::Authority, log);

	CHECK(lines == whole.lines);
	CHECK(config.reads() == whole.reads);
	CHECK(owner.ticks == whole.owner.ticks);
	CHECK(owner.serverOnly == whole.owner.serverOnly);
}

#endif // WITH_LOW_LEVEL_TESTS
