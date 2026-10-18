// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "catch_amalgamated.hpp"
#include "OGSimulation/BodySlotAssignment.h"
#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/TickDriverLog.h"
#include "OGSimulation/TickSlotTable.h"
#include "TickDriverFakes.h"

namespace
{
    using tickDriverFakes::FakeEngine;
    using tickDriverFakes::LoggedLine;

    std::string formatted(const char* fmt, va_list args)
    {
        va_list copy;
        va_copy(copy, args);
        const int length = std::vsnprintf(nullptr, 0u, fmt, copy);
        va_end(copy);
        std::string text(static_cast<size_t>(length), '\0');
        std::vsnprintf(text.data(), text.size() + 1u, fmt, args);
        return text;
    }

    struct FakeBindTable
    {
        void bindSlot(uint32_t slot) { bound.insert(slot); }
        void releaseSlot(uint32_t slot) { bound.erase(slot); }
        uint32_t boundBodies() const { return static_cast<uint32_t>(bound.size()) * 3u; }

        std::set<uint32_t> bound;
    };

    struct FakePhysAdapter
    {
        FakeBindTable& bindTable() { return table; }

        FakeBindTable table;
    };

    struct FakeFrameHost
    {
        void pushOccupancy_GameThread(uint32_t slot, bool occupied)
        {
            events.push_back("occupancy:" + std::to_string(slot) + ":" + (occupied ? "1" : "0"));
        }

        std::vector<std::string> events;
    };

    namespace legacyJoltSlot
    {
        using uint8 = uint8_t;
        using uint32 = uint32_t;
        using int32 = int32_t;

        enum ENetMode
        {
            NM_DedicatedServer,
            NM_Client,
        };

        enum LogVerbosity
        {
            Log,
            Warning,
        };

        struct LogCategory
        {
        };

        inline LogCategory LogOGSimHost;

        struct FString : std::string
        {
            FString(const char* text) : std::string(text) {}
            FString(const std::string& text) : std::string(text) {}
            const char* operator*() const { return c_str(); }
        };

        struct AOGBrawlerUECharacter;

        struct AActor
        {
            explicit AActor(std::string actorName) : name(std::move(actorName)) {}
            virtual ~AActor() = default;
            virtual AOGBrawlerUECharacter* asCharacter() { return nullptr; }
            FString GetName() const { return FString(name); }

            std::string name;
        };

        struct AOGBrawlerUECharacter : AActor
        {
            AOGBrawlerUECharacter(std::string actorName, bool authority) : AActor(std::move(actorName)), hasAuthority(authority) {}

            AOGBrawlerUECharacter* asCharacter() override { return this; }

            std::optional<uint32> GetPhysicsSlot() const
            {
                if (PhysicsSlotValue == 0u)
                    return std::nullopt;
                return static_cast<uint32>(PhysicsSlotValue) - 1u;
            }

            void SetAuthoritativePhysicsSlot(uint32 Slot)
            {
                REQUIRE(hasAuthority);
                REQUIRE(Slot < kMaxSimulatableSlots);
                REQUIRE(PhysicsSlotValue == 0u);
                PhysicsSlotValue = static_cast<uint8>(Slot + 1u);
                published.push_back(Slot);
            }

            bool hasAuthority;
            uint8 PhysicsSlotValue = 0u;
            std::vector<uint32> published;
        };

        template <typename T>
        T* Cast(AActor* actor)
        {
            static_assert(std::is_same_v<T, AOGBrawlerUECharacter>, "the reference model casts to its character only");
            return actor->asCharacter();
        }

        struct FakeOwner
        {
            ENetMode GetNetMode() const { return netMode; }

            ENetMode netMode;
            FakePhysAdapter* m_physAdapter;
        };

        struct Worlds
        {
            std::unordered_map<const AActor*, uint32> m_joltSlotByOwner;
            std::unordered_map<const AActor*, uint8> m_joltSlotWaitsLoggedByOwner;
            std::optional<int> m_shadowWorld;
        };

        class JoltBackendUImpl
        {
        public:
            JoltBackendUImpl(ENetMode netMode, bool withShadowWorld)
                : m_owner{ netMode, &m_physAdapter }
            {
                if (withShadowWorld)
                    m_worlds.m_shadowWorld.emplace(0);
            }

            std::optional<uint32> tryAcquireJoltSlot(AActor& owner);
            void releaseJoltSlot(const AActor& owner, unsigned int id);

            std::vector<LoggedLine> lines;
            std::vector<std::string> events;
            FakePhysAdapter m_physAdapter;
            FakeFrameHost m_frameHost;
            Worlds m_worlds;

        private:
            uint32 boundJoltBodyCount() const { return m_physAdapter.table.boundBodies(); }
            void releaseShadowSlot(uint32 slot) { events.push_back("shadowRelease:" + std::to_string(slot)); }

            void ueLog(LogCategory&, LogVerbosity verbosity, const char* fmt, ...)
            {
                va_list args;
                va_start(args, fmt);
                lines.push_back(LoggedLine{ verbosity == Warning ? HostLogLevel::Warning : HostLogLevel::Log, formatted(fmt, args) });
                va_end(args);
            }

            static void checkfRef(bool condition, const char*, ...) { REQUIRE(condition); }

            FakeOwner m_owner;
        };

        std::optional<uint32> JoltBackendUImpl::tryAcquireJoltSlot(AActor& owner)
        {
            if (const auto found = m_worlds.m_joltSlotByOwner.find(&owner); found != m_worlds.m_joltSlotByOwner.end())
                return found->second;

            AOGBrawlerUECharacter* const character = Cast<AOGBrawlerUECharacter>(&owner);
            checkfRef(character != nullptr,
                "tryAcquireJoltSlot: '%s' is not an AOGBrawlerUECharacter. The slot the authority chooses reaches the "
                "clients on that class's replicated PhysicsSlotValue, so only that class can hold a Jolt slot.",
                *owner.GetName());

            std::array<bool, kMaxSimulatableSlots> taken{};
            for (const auto& entry : m_worlds.m_joltSlotByOwner)
                taken[entry.second] = true;
            const auto lowestFree = std::find(taken.begin(), taken.end(), false);

            const bool isAuthority = m_owner.GetNetMode() != NM_Client;
            const BodySlotDecision decision = decideBodySlot(isAuthority, character->GetPhysicsSlot(), taken);

            if (decision.outcome == BodySlotOutcome::WaitForReplication || decision.outcome == BodySlotOutcome::WaitForRelease)
            {
                const bool forReplication = decision.outcome == BodySlotOutcome::WaitForReplication;
                uint8& loggedWaits = m_worlds.m_joltSlotWaitsLoggedByOwner[&owner];
                const uint8 waitBit = forReplication ? 1u : 2u;
                if ((loggedWaits & waitBit) == 0u)
                {
                    loggedWaits |= waitBit;
                    FString heldBy = "-";
                    for (const auto& entry : m_worlds.m_joltSlotByOwner)
                    {
                        if (!forReplication && entry.second == decision.slot)
                            heldBy = entry.first->GetName();
                    }
                    ueLog(LogOGSimHost, Log, "[SimHost.Slot] wait=%s owner=%s slot=%d heldBy=%s",
                        forReplication ? "WaitForReplication" : "WaitForRelease", *owner.GetName(),
                        forReplication ? -1 : static_cast<int32>(decision.slot), *heldBy);
                }
                return std::nullopt;
            }

            checkfRef(decision.outcome == BodySlotOutcome::Assigned,
                "tryAcquireJoltSlot: all %u Jolt slots are taken; '%s' cannot get one. The Jolt world has a fixed "
                "body set of kMaxSimulatableSlots characters (design D7, task 10).",
                kMaxSimulatableSlots, *owner.GetName());
            if (decision.outcome != BodySlotOutcome::Assigned)
                return std::nullopt;

            if (isAuthority)
                character->SetAuthoritativePhysicsSlot(decision.slot);
            m_worlds.m_joltSlotByOwner.emplace(&owner, decision.slot);
            m_worlds.m_joltSlotWaitsLoggedByOwner.erase(&owner);

            ueLog(LogOGSimHost, Log, "[SimHost.Slot] assigned role=%s owner=%s slot=%u lowestFreeLocally=%d",
                isAuthority ? "Authority" : "Client", *owner.GetName(), decision.slot,
                lowestFree != taken.end() ? static_cast<int32>(lowestFree - taken.begin()) : -1);
            return decision.slot;
        }

        void JoltBackendUImpl::releaseJoltSlot(const AActor& owner, unsigned int id)
        {
            m_worlds.m_joltSlotWaitsLoggedByOwner.erase(&owner);

            const auto found = m_worlds.m_joltSlotByOwner.find(&owner);
            if (found == m_worlds.m_joltSlotByOwner.end())
                return;

            const uint32 slot = found->second;
            m_frameHost.pushOccupancy_GameThread(slot, false);
            m_owner.m_physAdapter->bindTable().releaseSlot(slot);
            m_worlds.m_joltSlotByOwner.erase(found);

            ueLog(LogOGSimHost, Warning, "[SimHost.Bind] release id=%u slot=%u boundBodies=%u slotsInUse=%d",
                id, slot, boundJoltBodyCount(), static_cast<int32>(m_worlds.m_joltSlotByOwner.size()));

            if (m_worlds.m_shadowWorld.has_value())
                releaseShadowSlot(slot);
        }
    } // namespace legacyJoltSlot

    using legacyJoltSlot::AActor;
    using Character = legacyJoltSlot::AOGBrawlerUECharacter;
    using LegacyBackend = legacyJoltSlot::JoltBackendUImpl;

    uint64_t ownerKey(const AActor& owner)
    {
        return static_cast<uint64_t>(reinterpret_cast<uintptr_t>(&owner));
    }

    class TableBackend
    {
    public:
        TableBackend(bool isAuthorityPeer, bool withShadowWorld)
            : m_isAuthorityPeer(isAuthorityPeer)
            , m_withShadowWorld(withShadowWorld)
            , m_table(engine)
        {
            m_line.reserve(256u);
        }

        std::optional<uint32_t> tryAcquire(AActor& owner)
        {
            Character* const character = legacyJoltSlot::Cast<Character>(&owner);
            REQUIRE(character != nullptr);
            const uint64_t key = ownerKey(owner);
            const bool alreadyHeld = m_table.slotOf(key).has_value();
            const std::optional<uint32_t> slot = m_table.acquire(key, owner.name.c_str(), m_isAuthorityPeer, character->GetPhysicsSlot());
            if (slot.has_value() && m_isAuthorityPeer && !alreadyHeld)
                character->SetAuthoritativePhysicsSlot(*slot);
            return slot;
        }

        void release(const AActor& owner, unsigned int id)
        {
            const std::optional<uint32_t> slot = m_table.release(ownerKey(owner));
            if (!slot.has_value())
                return;

            frameHost.pushOccupancy_GameThread(*slot, false);
            physAdapter.bindTable().releaseSlot(*slot);
            engine.log(HostLogLevel::Warning, formatHostLine(m_line, "[SimHost.Bind] release id=%u slot=%u boundBodies=%u slotsInUse=%d",
                id, *slot, physAdapter.table.boundBodies(), static_cast<int32_t>(m_table.size())));
            if (m_withShadowWorld)
                events.push_back("shadowRelease:" + std::to_string(*slot));
        }

        const TickSlotTable& table() const { return m_table; }

        FakeEngine engine;
        FakePhysAdapter physAdapter;
        FakeFrameHost frameHost;
        std::vector<std::string> events;

    private:
        bool m_isAuthorityPeer;
        bool m_withShadowWorld;
        TickSlotTable m_table;
        HostLine m_line;
    };

    bool sameLines(const std::vector<LoggedLine>& a, const std::vector<LoggedLine>& b)
    {
        if (a.size() != b.size())
            return false;
        for (size_t index = 0; index < a.size(); ++index)
        {
            if (a[index].level != b[index].level || a[index].text != b[index].text)
                return false;
        }
        return true;
    }

    std::vector<std::string> texts(const std::vector<LoggedLine>& lines)
    {
        std::vector<std::string> result;
        for (const LoggedLine& line : lines)
            result.push_back(line.text);
        return result;
    }

    struct Witnesses
    {
        uint32_t authorityAssigned = 0u;
        uint32_t clientAssigned = 0u;
        uint32_t clientAssignedAboveALowerFreeSlot = 0u;
        uint32_t waitForReplication = 0u;
        uint32_t waitForRelease = 0u;
        uint32_t waitRepeatedWithoutALine = 0u;
        uint32_t heldAcquire = 0u;
        uint32_t releasesWithASlot = 0u;
        uint32_t releasesWithoutASlot = 0u;
    };

    struct ActorState
    {
        bool aliveOnAuthority = false;
        bool knownOnClient = false;
        bool leftClient = false;
    };

    class Differential
    {
    public:
        explicit Differential(uint32_t seed)
            : m_random(seed)
            , m_legacyAuthority(legacyJoltSlot::NM_DedicatedServer, false)
            , m_legacyClient(legacyJoltSlot::NM_Client, true)
            , m_tableAuthority(true, false)
            , m_tableClient(false, true)
        {
        }

        void run(uint32_t events)
        {
            for (uint32_t index = 0; index < events; ++index)
            {
                step();
                compareState();
            }
            CHECK(sameLines(m_legacyAuthority.lines, m_tableAuthority.engine.lines));
            CHECK(sameLines(m_legacyClient.lines, m_tableClient.engine.lines));
            CHECK(texts(m_legacyClient.lines) == texts(m_tableClient.engine.lines));
        }

        Witnesses witnesses;
        size_t linesWritten() const { return m_legacyAuthority.lines.size() + m_legacyClient.lines.size(); }

    private:
        uint32_t pick(uint32_t count) { return std::uniform_int_distribution<uint32_t>(0u, count - 1u)(m_random); }

        template <typename Predicate>
        std::optional<size_t> pickActor(Predicate predicate)
        {
            std::vector<size_t> candidates;
            for (size_t index = 0; index < m_states.size(); ++index)
            {
                if (predicate(m_states[index], index))
                    candidates.push_back(index);
            }
            if (candidates.empty())
                return std::nullopt;
            return candidates[pick(static_cast<uint32_t>(candidates.size()))];
        }

        size_t aliveOnAuthority() const
        {
            return static_cast<size_t>(std::count_if(m_states.begin(), m_states.end(), [](const ActorState& s) { return s.aliveOnAuthority; }));
        }

        void step()
        {
            switch (pick(8u))
            {
            case 0:
            case 1:
                join();
                break;
            case 2:
                if (const auto actor = pickActor([](const ActorState& s, size_t) { return s.aliveOnAuthority; }))
                    acquireOnAuthority(*actor);
                break;
            case 3:
                if (const auto actor = pickActor([](const ActorState& s, size_t) { return !s.knownOnClient; }))
                    m_states[*actor].knownOnClient = true;
                break;
            case 4:
                if (const auto actor = pickActor([this](const ActorState& s, size_t index)
                    { return s.knownOnClient && m_tableClientActors[index].PhysicsSlotValue != m_tableAuthorityActors[index].PhysicsSlotValue; }))
                {
                    m_legacyClientActors[*actor].PhysicsSlotValue = m_legacyAuthorityActors[*actor].PhysicsSlotValue;
                    m_tableClientActors[*actor].PhysicsSlotValue = m_tableAuthorityActors[*actor].PhysicsSlotValue;
                }
                break;
            case 5:
            case 6:
                if (const auto actor = pickActor([](const ActorState& s, size_t) { return s.knownOnClient && !s.leftClient; }))
                    acquireOnClient(*actor);
                break;
            default:
                if (pick(2u) == 0u)
                {
                    if (const auto actor = pickActor([](const ActorState& s, size_t) { return s.aliveOnAuthority; }))
                    {
                        m_states[*actor].aliveOnAuthority = false;
                        releaseOn(m_legacyAuthority, m_tableAuthority, m_legacyAuthorityActors[*actor], m_tableAuthorityActors[*actor], *actor);
                    }
                }
                else if (const auto actor = pickActor([](const ActorState& s, size_t)
                    { return !s.aliveOnAuthority && s.knownOnClient && !s.leftClient; }))
                {
                    m_states[*actor].leftClient = true;
                    releaseOn(m_legacyClient, m_tableClient, m_legacyClientActors[*actor], m_tableClientActors[*actor], *actor);
                }
                break;
            }
        }

        void join()
        {
            if (aliveOnAuthority() >= kMaxSimulatableSlots || m_states.size() >= 96u)
                return;
            const std::string name = "BP_Brawler_C_" + std::to_string(2147482000u + m_states.size());
            m_legacyAuthorityActors.emplace_back(name, true);
            m_legacyClientActors.emplace_back(name, false);
            m_tableAuthorityActors.emplace_back(name, true);
            m_tableClientActors.emplace_back(name, false);
            m_states.push_back(ActorState{ true, false, false });
            acquireOnAuthority(m_states.size() - 1u);
        }

        void acquireOnAuthority(size_t actor)
        {
            acquireOn(m_legacyAuthority, m_tableAuthority, m_legacyAuthorityActors[actor], m_tableAuthorityActors[actor], true);
        }

        void acquireOnClient(size_t actor)
        {
            acquireOn(m_legacyClient, m_tableClient, m_legacyClientActors[actor], m_tableClientActors[actor], false);
        }

        static std::array<bool, kMaxSimulatableSlots> legacyTaken(const LegacyBackend& backend)
        {
            std::array<bool, kMaxSimulatableSlots> taken{};
            for (const auto& entry : backend.m_worlds.m_joltSlotByOwner)
                taken[entry.second] = true;
            return taken;
        }

        static std::array<bool, kMaxSimulatableSlots> tableTaken(const TableBackend& backend, const std::deque<Character>& actors)
        {
            std::array<bool, kMaxSimulatableSlots> taken{};
            for (const Character& actor : actors)
            {
                if (const auto slot = backend.table().slotOf(ownerKey(actor)))
                    taken[*slot] = true;
            }
            return taken;
        }

        void acquireOn(LegacyBackend& legacy, TableBackend& table, Character& legacyActor, Character& tableActor, bool isAuthority)
        {
            const std::deque<Character>& tableActors = isAuthority ? m_tableAuthorityActors : m_tableClientActors;
            const auto takenLegacy = legacyTaken(legacy);
            REQUIRE(takenLegacy == tableTaken(table, tableActors));
            REQUIRE(legacyActor.GetPhysicsSlot() == tableActor.GetPhysicsSlot());
            const bool held = legacy.m_worlds.m_joltSlotByOwner.count(&legacyActor) != 0u;
            const BodySlotDecision decision = decideBodySlot(isAuthority, legacyActor.GetPhysicsSlot(), takenLegacy);
            REQUIRE(decision == decideBodySlot(isAuthority, tableActor.GetPhysicsSlot(), tableTaken(table, tableActors)));

            const size_t linesBefore = legacy.lines.size();
            const std::optional<uint32_t> legacySlot = legacy.tryAcquireJoltSlot(legacyActor);
            const std::optional<uint32_t> tableSlot = table.tryAcquire(tableActor);
            REQUIRE(legacySlot == tableSlot);
            REQUIRE(sameLines(legacy.lines, table.engine.lines));

            if (held)
                ++witnesses.heldAcquire;
            else if (decision.outcome == BodySlotOutcome::Assigned)
            {
                ++(isAuthority ? witnesses.authorityAssigned : witnesses.clientAssigned);
                const auto lowestFree = std::find(takenLegacy.begin(), takenLegacy.end(), false);
                if (!isAuthority && static_cast<uint32_t>(lowestFree - takenLegacy.begin()) < decision.slot)
                    ++witnesses.clientAssignedAboveALowerFreeSlot;
            }
            else if (decision.outcome == BodySlotOutcome::WaitForReplication)
                ++witnesses.waitForReplication;
            else if (decision.outcome == BodySlotOutcome::WaitForRelease)
                ++witnesses.waitForRelease;
            if (!held && decision.outcome != BodySlotOutcome::Assigned && legacy.lines.size() == linesBefore)
                ++witnesses.waitRepeatedWithoutALine;

            if (legacySlot.has_value())
            {
                legacy.m_physAdapter.bindTable().bindSlot(*legacySlot);
                table.physAdapter.bindTable().bindSlot(*tableSlot);
            }
        }

        void releaseOn(LegacyBackend& legacy, TableBackend& table, Character& legacyActor, Character& tableActor, size_t actor)
        {
            const bool held = legacy.m_worlds.m_joltSlotByOwner.count(&legacyActor) != 0u;
            const unsigned int id = static_cast<unsigned int>(actor) + 1u;
            legacy.releaseJoltSlot(legacyActor, id);
            table.release(tableActor, id);
            ++(held ? witnesses.releasesWithASlot : witnesses.releasesWithoutASlot);
            REQUIRE(sameLines(legacy.lines, table.engine.lines));
            REQUIRE(legacy.m_frameHost.events == table.frameHost.events);
            REQUIRE(legacy.events == table.events);
            REQUIRE(legacy.m_physAdapter.table.bound == table.physAdapter.table.bound);
        }

        void compareState()
        {
            REQUIRE(sameBackend(m_legacyAuthority, m_tableAuthority, m_legacyAuthorityActors, m_tableAuthorityActors));
            REQUIRE(sameBackend(m_legacyClient, m_tableClient, m_legacyClientActors, m_tableClientActors));
        }

        static bool sameBackend(const LegacyBackend& legacy, const TableBackend& table, const std::deque<Character>& legacyActors,
            const std::deque<Character>& tableActors)
        {
            if (legacy.m_worlds.m_joltSlotByOwner.size() != table.table().size())
                return false;
            for (size_t index = 0; index < legacyActors.size(); ++index)
            {
                const auto found = legacy.m_worlds.m_joltSlotByOwner.find(&legacyActors[index]);
                const std::optional<uint32_t> legacySlot =
                    found != legacy.m_worlds.m_joltSlotByOwner.end() ? std::optional<uint32_t>(found->second) : std::nullopt;
                if (legacySlot != table.table().slotOf(ownerKey(tableActors[index]))
                    || legacyActors[index].published != tableActors[index].published
                    || legacyActors[index].PhysicsSlotValue != tableActors[index].PhysicsSlotValue)
                    return false;
            }
            return true;
        }

        std::mt19937 m_random;
        LegacyBackend m_legacyAuthority;
        LegacyBackend m_legacyClient;
        TableBackend m_tableAuthority;
        TableBackend m_tableClient;
        std::deque<Character> m_legacyAuthorityActors;
        std::deque<Character> m_legacyClientActors;
        std::deque<Character> m_tableAuthorityActors;
        std::deque<Character> m_tableClientActors;
        std::vector<ActorState> m_states;
    };

    struct LineRecorder
    {
        bool logEnabled(HostLogLevel level) const { return static_cast<int>(level) >= static_cast<int>(minimumLevel); }
        void log(HostLogLevel level, const char* line) { lines.push_back(LoggedLine{ level, line }); }

        HostLogLevel minimumLevel = HostLogLevel::Verbose;
        std::vector<LoggedLine> lines;
    };

    static_assert(TickSlotLogTarget<FakeEngine>, "the kit's FakeEngine must be a TickSlotLogTarget, as every TickDriverEngine is");
    static_assert(TickSlotLogTarget<LineRecorder>, "LineRecorder must be a TickSlotLogTarget");
}

TEST_CASE("TickSlotTable.Differential.EqualsTheLegacyJoltSlotAcquireOverRandomOrders", "[TickSlotTable]")
{
    Witnesses total;
    size_t lines = 0u;
    for (uint32_t seed = 1u; seed <= 24u; ++seed)
    {
        INFO("seed " << seed);
        Differential differential(seed);
        differential.run(500u);
        lines += differential.linesWritten();
        const Witnesses& w = differential.witnesses;
        total.authorityAssigned += w.authorityAssigned;
        total.clientAssigned += w.clientAssigned;
        total.clientAssignedAboveALowerFreeSlot += w.clientAssignedAboveALowerFreeSlot;
        total.waitForReplication += w.waitForReplication;
        total.waitForRelease += w.waitForRelease;
        total.waitRepeatedWithoutALine += w.waitRepeatedWithoutALine;
        total.heldAcquire += w.heldAcquire;
        total.releasesWithASlot += w.releasesWithASlot;
        total.releasesWithoutASlot += w.releasesWithoutASlot;
    }

    INFO("authorityAssigned=" << total.authorityAssigned << " clientAssigned=" << total.clientAssigned
        << " clientAssignedAboveALowerFreeSlot=" << total.clientAssignedAboveALowerFreeSlot
        << " waitForReplication=" << total.waitForReplication << " waitForRelease=" << total.waitForRelease
        << " waitRepeatedWithoutALine=" << total.waitRepeatedWithoutALine << " heldAcquire=" << total.heldAcquire
        << " releasesWithASlot=" << total.releasesWithASlot << " releasesWithoutASlot=" << total.releasesWithoutASlot
        << " lines=" << lines);
    CHECK(total.authorityAssigned > 0u);
    CHECK(total.clientAssigned > 0u);
    CHECK(total.clientAssignedAboveALowerFreeSlot > 0u);
    CHECK(total.waitForReplication > 0u);
    CHECK(total.waitForRelease > 0u);
    CHECK(total.waitRepeatedWithoutALine > 0u);
    CHECK(total.heldAcquire > 0u);
    CHECK(total.releasesWithASlot > 0u);
    CHECK(total.releasesWithoutASlot > 0u);
}

TEST_CASE("TickSlotTable.Lines.MatchTheGoldens", "[TickSlotTable]")
{
    LineRecorder recorder;
    TickSlotTable authority(recorder);
    CHECK(authority.acquire(11u, "BP_Brawler_C_1", true, std::nullopt) == std::optional<uint32_t>(0u));
    CHECK(authority.acquire(12u, "BP_Brawler_C_2", true, std::optional<uint32_t>(6u)) == std::optional<uint32_t>(1u));

    TickSlotTable client(recorder);
    CHECK(client.acquire(21u, "BP_Brawler_C_1", false, std::optional<uint32_t>(5u)) == std::optional<uint32_t>(5u));
    CHECK_FALSE(client.acquire(22u, "BP_Brawler_C_2", false, std::nullopt).has_value());
    CHECK_FALSE(client.acquire(23u, "BP_Brawler_C_3", false, std::optional<uint32_t>(5u)).has_value());

    const std::vector<std::string> expected{
        "[SimHost.Slot] assigned role=Authority owner=BP_Brawler_C_1 slot=0 lowestFreeLocally=0",
        "[SimHost.Slot] assigned role=Authority owner=BP_Brawler_C_2 slot=1 lowestFreeLocally=1",
        "[SimHost.Slot] assigned role=Client owner=BP_Brawler_C_1 slot=5 lowestFreeLocally=0",
        "[SimHost.Slot] wait=WaitForReplication owner=BP_Brawler_C_2 slot=-1 heldBy=-",
        "[SimHost.Slot] wait=WaitForRelease owner=BP_Brawler_C_3 slot=5 heldBy=BP_Brawler_C_1",
    };
    CHECK(texts(recorder.lines) == expected);
    for (const LoggedLine& line : recorder.lines)
        CHECK(line.level == HostLogLevel::Log);
}

TEST_CASE("TickSlotTable.Waits.EachKindIsLoggedOncePerOwner", "[TickSlotTable]")
{
    LineRecorder recorder;
    TickSlotTable client(recorder);
    REQUIRE(client.acquire(1u, "holder", false, std::optional<uint32_t>(2u)) == std::optional<uint32_t>(2u));
    recorder.lines.clear();

    SECTION("a repeated wait of one kind writes one line")
    {
        for (int attempt = 0; attempt < 4; ++attempt)
            CHECK_FALSE(client.acquire(2u, "waiter", false, std::nullopt).has_value());
        for (int attempt = 0; attempt < 4; ++attempt)
            CHECK_FALSE(client.acquire(2u, "waiter", false, std::optional<uint32_t>(2u)).has_value());
        for (int attempt = 0; attempt < 2; ++attempt)
            CHECK_FALSE(client.acquire(2u, "waiter", false, std::nullopt).has_value());

        CHECK(texts(recorder.lines) == std::vector<std::string>{
            "[SimHost.Slot] wait=WaitForReplication owner=waiter slot=-1 heldBy=-",
            "[SimHost.Slot] wait=WaitForRelease owner=waiter slot=2 heldBy=holder",
        });
        CHECK(client.size() == 1u);
        CHECK_FALSE(client.slotOf(2u).has_value());
    }

    SECTION("each owner has its own record")
    {
        CHECK_FALSE(client.acquire(2u, "first", false, std::nullopt).has_value());
        CHECK_FALSE(client.acquire(3u, "second", false, std::nullopt).has_value());
        CHECK_FALSE(client.acquire(2u, "first", false, std::nullopt).has_value());
        CHECK(texts(recorder.lines) == std::vector<std::string>{
            "[SimHost.Slot] wait=WaitForReplication owner=first slot=-1 heldBy=-",
            "[SimHost.Slot] wait=WaitForReplication owner=second slot=-1 heldBy=-",
        });
    }

    SECTION("an assignment ends the record, so a later wait is logged again")
    {
        CHECK_FALSE(client.acquire(2u, "waiter", false, std::optional<uint32_t>(2u)).has_value());
        CHECK(client.release(1u) == std::optional<uint32_t>(2u));
        CHECK(client.acquire(2u, "waiter", false, std::optional<uint32_t>(2u)) == std::optional<uint32_t>(2u));
        CHECK(client.release(2u) == std::optional<uint32_t>(2u));
        CHECK_FALSE(client.acquire(2u, "waiter", false, std::nullopt).has_value());
        CHECK(texts(recorder.lines) == std::vector<std::string>{
            "[SimHost.Slot] wait=WaitForRelease owner=waiter slot=2 heldBy=holder",
            "[SimHost.Slot] assigned role=Client owner=waiter slot=2 lowestFreeLocally=0",
            "[SimHost.Slot] wait=WaitForReplication owner=waiter slot=-1 heldBy=-",
        });
    }

    SECTION("a release ends the record of an owner that never got a slot")
    {
        CHECK_FALSE(client.acquire(2u, "waiter", false, std::nullopt).has_value());
        CHECK_FALSE(client.release(2u).has_value());
        CHECK_FALSE(client.acquire(2u, "waiter", false, std::nullopt).has_value());
        CHECK(texts(recorder.lines) == std::vector<std::string>{
            "[SimHost.Slot] wait=WaitForReplication owner=waiter slot=-1 heldBy=-",
            "[SimHost.Slot] wait=WaitForReplication owner=waiter slot=-1 heldBy=-",
        });
    }

    SECTION("a wait is recorded while the log level is off, and is not logged later")
    {
        recorder.minimumLevel = HostLogLevel::Warning;
        CHECK_FALSE(client.acquire(2u, "waiter", false, std::nullopt).has_value());
        recorder.minimumLevel = HostLogLevel::Verbose;
        CHECK_FALSE(client.acquire(2u, "waiter", false, std::nullopt).has_value());
        CHECK(recorder.lines.empty());
    }
}

TEST_CASE("TickSlotTable.Client.NeverAssignedWithoutTheReplicatedSlot", "[TickSlotTable]")
{
    LineRecorder recorder;
    TickSlotTable client(recorder);
    for (int attempt = 0; attempt < 3; ++attempt)
        CHECK_FALSE(client.acquire(7u, "pawn", false, std::nullopt).has_value());
    CHECK(client.size() == 0u);
    CHECK_FALSE(client.slotOf(7u).has_value());

    CHECK(client.acquire(7u, "pawn", false, std::optional<uint32_t>(4u)) == std::optional<uint32_t>(4u));
    CHECK(client.slotOf(7u) == std::optional<uint32_t>(4u));
    CHECK(client.acquire(7u, "pawn", false, std::nullopt) == std::optional<uint32_t>(4u));
    CHECK(texts(recorder.lines).back() == "[SimHost.Slot] assigned role=Client owner=pawn slot=4 lowestFreeLocally=0");
}

TEST_CASE("TickSlotTable.Assigned.LowestFreeLocallyIsFromTheSlotsHeldBeforeTheAssignment", "[TickSlotTable]")
{
    LineRecorder recorder;
    TickSlotTable authority(recorder);
    for (uint64_t key = 1u; key <= 3u; ++key)
        REQUIRE(authority.acquire(key, "a", true, std::nullopt) == std::optional<uint32_t>(static_cast<uint32_t>(key - 1u)));
    REQUIRE(authority.release(2u) == std::optional<uint32_t>(1u));
    CHECK(authority.acquire(4u, "b", true, std::nullopt) == std::optional<uint32_t>(1u));
    CHECK(texts(recorder.lines).back() == "[SimHost.Slot] assigned role=Authority owner=b slot=1 lowestFreeLocally=1");

    TickSlotTable client(recorder);
    REQUIRE(client.acquire(1u, "c", false, std::optional<uint32_t>(0u)) == std::optional<uint32_t>(0u));
    CHECK(client.acquire(2u, "d", false, std::optional<uint32_t>(6u)) == std::optional<uint32_t>(6u));
    CHECK(texts(recorder.lines).back() == "[SimHost.Slot] assigned role=Client owner=d slot=6 lowestFreeLocally=1");
}

TEST_CASE("TickSlotTable.Release.ErasesTheOwnerAndReturnsItsSlot", "[TickSlotTable]")
{
    LineRecorder recorder;
    TickSlotTable authority(recorder);
    REQUIRE(authority.acquire(5u, "e", true, std::nullopt) == std::optional<uint32_t>(0u));
    REQUIRE(authority.acquire(6u, "f", true, std::nullopt) == std::optional<uint32_t>(1u));
    CHECK(authority.size() == 2u);

    CHECK(authority.release(5u) == std::optional<uint32_t>(0u));
    CHECK(authority.size() == 1u);
    CHECK_FALSE(authority.slotOf(5u).has_value());
    CHECK_FALSE(authority.release(5u).has_value());
    CHECK_FALSE(authority.release(99u).has_value());
    CHECK(authority.slotOf(6u) == std::optional<uint32_t>(1u));
    CHECK(recorder.lines.size() == 2u);
}

TEST_CASE("TickSlotTable.Lines.AreFormattedIntoTheReservedLine", "[TickSlotTable]")
{
    LineRecorder recorder;
    TickSlotTable client(recorder);
    const std::string longName(600u, 'n');
    CHECK(client.acquire(1u, longName.c_str(), false, std::optional<uint32_t>(3u)) == std::optional<uint32_t>(3u));
    CHECK_FALSE(client.acquire(2u, "w", false, std::optional<uint32_t>(3u)).has_value());
    CHECK(texts(recorder.lines) == std::vector<std::string>{
        "[SimHost.Slot] assigned role=Client owner=" + longName + " slot=3 lowestFreeLocally=0",
        "[SimHost.Slot] wait=WaitForRelease owner=w slot=3 heldBy=" + longName,
    });
}

#endif // WITH_LOW_LEVEL_TESTS
