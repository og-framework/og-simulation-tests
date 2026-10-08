// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/PhysicsWorldAdapter.h"
#include "OGSimulation/SimulationTimeContext.h"
#include "OGSimulation/StepHooks.h"
#include "../Mocks/MockPhysicsWorld.h"

// Shared by the step-driver suites. Every call the driver makes on the world, the hooks and the
// integration executor lands in ONE ordered event list, so a case can assert the interleaving
// across all of them, not just each one's own order.
namespace stepDriverTestSupport
{
    using Events = std::vector<std::string>;

    inline std::string tickEvent(const char* name, SimTick tick)
    {
        return std::string(name) + ":" + std::to_string(tick);
    }

    // MockPhysicsWorld plus an ordered log. Queries (hasTick, oldestHeldTick, stateHash) are NOT
    // logged: they change nothing, and the driver asks them at points the call-order cases do
    // not pin.
    class LoggedWorld
    {
    public:
        static constexpr SnapshotCoverage coverage{};

        LoggedWorld(uint32_t ringDepthTicks, Events& events)
            : m_inner(ringDepthTicks)
            , m_events(&events)
        {
        }

        void step(float dt)
        {
            m_inner.step(dt);
            ++m_steps;
            log("step");
        }

        void saveTick(SimTick tick)
        {
            m_inner.saveTick(tick);
            log(tickEvent("save", tick));
        }

        bool restoreTick(SimTick tick)
        {
            const bool restored = m_inner.restoreTick(tick);
            if (!restored)
            {
                ++m_failedRestores;
            }
            log(tickEvent("restore", tick));
            return restored;
        }

        bool hasTick(SimTick tick) const { return m_inner.hasTick(tick); }

        std::optional<SimTick> oldestHeldTick() const { return m_inner.oldestHeldTick(); }

        void invalidateAllTicks()
        {
            m_inner.invalidateAllTicks();
            log("invalidate");
        }

        void saveScratch()
        {
            m_inner.saveScratch();
            log("scratch");
        }

        void commitScratch(SimTick tick)
        {
            m_inner.commitScratch(tick);
            log(tickEvent("commit", tick));
        }

        void applyOccupancy(const BodySlotOccupancy& occupancy)
        {
            m_inner.applyOccupancy(occupancy);
            log("occ");
        }

        std::optional<uint64_t> stateHash(SimTick tick) const { return m_inner.stateHash(tick); }

        MockPhysicsWorld& inner() { return m_inner; }
        const MockPhysicsWorld& inner() const { return m_inner; }

        std::set<SimTick> heldTicks(SimTick upTo) const
        {
            std::set<SimTick> held;
            for (SimTick tick = 0; tick <= upTo; ++tick)
            {
                if (m_inner.hasTick(tick))
                {
                    held.insert(tick);
                }
            }
            return held;
        }

        uint64_t steps() const { return m_steps; }
        uint64_t failedRestores() const { return m_failedRestores; }

    private:
        void log(std::string event) { m_events->push_back(std::move(event)); }

        MockPhysicsWorld m_inner;
        Events* m_events;
        uint64_t m_steps = 0;
        uint64_t m_failedRestores = 0;
    };

    static_assert(PhysicsWorldAdapter<LoggedWorld>, "LoggedWorld must model PhysicsWorldAdapter");

    class RecordingHooks
    {
    public:
        explicit RecordingHooks(Events& events) : m_events(&events) {}

        void beforeTick(const UpcomingTick& upcoming)
        {
            m_events->push_back("beforeTick");
            upcomingTicks.push_back(upcoming);
            if (onBeforeTick)
            {
                onBeforeTick(upcoming);
            }
        }

        void beforeSimulate(const UpcomingTick& upcoming)
        {
            m_events->push_back("beforeSimulate");
            simulateUpcomingTicks.push_back(upcoming);
            if (onBeforeSimulate)
            {
                onBeforeSimulate(upcoming);
            }
        }

        void beforePhysics(SimTick tick)
        {
            m_events->push_back("beforePhysics");
            beforePhysicsTicks.push_back(tick);
        }

        void afterTick(const TickOutcome& outcome)
        {
            m_events->push_back("afterTick");
            outcomes.push_back(outcome);
        }

        std::function<void(const UpcomingTick&)> onBeforeTick;
        std::function<void(const UpcomingTick&)> onBeforeSimulate;
        std::vector<UpcomingTick> upcomingTicks;
        std::vector<UpcomingTick> simulateUpcomingTicks;
        std::vector<SimTick> beforePhysicsTicks;
        std::vector<TickOutcome> outcomes;

    private:
        Events* m_events;
    };

    static_assert(StepHooks<RecordingHooks>, "RecordingHooks must model StepHooks");

    class RecordingIntegration
    {
    public:
        explicit RecordingIntegration(Events& events) : m_events(&events) {}

        void firstResimStepAll(int32_t) {}
        void captureBodyStatesAll() {}
        void pushCorrectedBodyStatesAll()
        {
            ++pushes;
            m_events->push_back("push");
        }

        uint32_t pushes = 0;

    private:
        Events* m_events;
    };

    // The interleaving of one resim of `anchor` up to `present`, followed by the normal tick
    // `present + 1`: the sequence the task's acceptance criteria spell out.
    inline Events resimThenNormalTick(SimTick anchor, SimTick present)
    {
        Events expected{ "beforeTick", tickEvent("restore", anchor), "occ", tickEvent("prepare", anchor), "push" };
        for (SimTick tick = anchor + 1u; tick <= present; ++tick)
        {
            expected.insert(expected.end(), { "occ", "simResim", "step", "postResim", tickEvent("save", tick) });
        }
        expected.insert(expected.end(),
            { "beforeSimulate", "scratch", "sim", "beforePhysics", "step", "post", tickEvent("save", present + 1u),
              "afterTick" });
        return expected;
    }

    inline std::vector<SimTick> savedTicksIn(const Events& events)
    {
        std::vector<SimTick> saved;
        for (const std::string& event : events)
        {
            if (event.rfind("save:", 0) == 0)
            {
                saved.push_back(static_cast<SimTick>(std::stoul(event.substr(5))));
            }
        }
        return saved;
    }

    inline std::size_t countOf(const Events& events, const std::string& name)
    {
        return static_cast<std::size_t>(std::count(events.begin(), events.end(), name));
    }

    inline Events normalTick(SimTick tick)
    {
        return { "beforeTick", "beforeSimulate", "scratch", "sim", "beforePhysics", "step", "post",
                 tickEvent("save", tick), "afterTick" };
    }

    inline bool sameUpcoming(const UpcomingTick& lhs, const UpcomingTick& rhs)
    {
        return lhs.authority == rhs.authority && lhs.authorityTick == rhs.authorityTick
            && lhs.physicsStep == rhs.physicsStep;
    }
} // namespace stepDriverTestSupport
