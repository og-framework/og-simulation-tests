// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

#include "OGSimulation/BodySlotOccupancy.h"
#include "OGSimulation/PhysicsWorldAdapter.h"
#include "OGSimulation/StaticGeometry.h"

enum class MockWorldCallKind : uint8_t
{
    Step,
    SaveTick,
    RestoreTick,
    HasTick,
    OldestHeldTick,
    InvalidateAllTicks,
    SaveScratch,
    CommitScratch,
    ApplyOccupancy,
    StateHash
};

struct MockWorldCall
{
    MockWorldCallKind kind = MockWorldCallKind::Step;
    SimTick tick = 0;
    float dt = 0.f;
    BodySlotOccupancy occupancy;
    bool result = false;
};

struct MockWorldState
{
    int64_t value = 0;
    BodySlotOccupancy occupancy;

    bool operator==(const MockWorldState&) const = default;
};

class MockPhysicsWorld
{
public:
    static constexpr SnapshotCoverage coverage{};

    explicit MockPhysicsWorld(uint32_t ringDepthTicks)
        : m_ringDepthTicks(ringDepthTicks)
    {
    }

    void step(float dt)
    {
        ++m_live.value;
        record({ MockWorldCallKind::Step, 0, dt, {}, false });
    }

    void saveTick(SimTick tick)
    {
        store(tick, m_live);
        record({ MockWorldCallKind::SaveTick, tick, 0.f, {}, false });
    }

    bool restoreTick(SimTick tick)
    {
        const HeldTick* held = find(tick);
        if (held != nullptr)
        {
            m_live = held->state;
        }
        record({ MockWorldCallKind::RestoreTick, tick, 0.f, {}, held != nullptr });
        return held != nullptr;
    }

    bool hasTick(SimTick tick) const
    {
        const bool held = find(tick) != nullptr;
        record({ MockWorldCallKind::HasTick, tick, 0.f, {}, held });
        return held;
    }

    std::optional<SimTick> oldestHeldTick() const
    {
        std::optional<SimTick> oldest;
        for (const HeldTick& held : m_ring)
        {
            if (!oldest || held.tick < *oldest)
            {
                oldest = held.tick;
            }
        }
        record({ MockWorldCallKind::OldestHeldTick, oldest.value_or(0), 0.f, {}, oldest.has_value() });
        return oldest;
    }

    void invalidateAllTicks()
    {
        m_ring.clear();
        record({ MockWorldCallKind::InvalidateAllTicks, 0, 0.f, {}, false });
    }

    void saveScratch()
    {
        m_scratch = m_live;
        record({ MockWorldCallKind::SaveScratch, 0, 0.f, {}, false });
    }

    void commitScratch(SimTick tick)
    {
        store(tick, m_scratch);
        record({ MockWorldCallKind::CommitScratch, tick, 0.f, {}, false });
    }

    void applyOccupancy(const BodySlotOccupancy& occupancy)
    {
        m_live.occupancy = occupancy;
        record({ MockWorldCallKind::ApplyOccupancy, 0, 0.f, occupancy, false });
    }

    std::optional<uint64_t> stateHash(SimTick tick) const
    {
        const HeldTick* held = find(tick);
        std::optional<uint64_t> hash;
        if (held != nullptr)
        {
            hash = hashOf(held->state);
        }
        record({ MockWorldCallKind::StateHash, tick, 0.f, {}, hash.has_value() });
        return hash;
    }

    static uint64_t hashOf(const MockWorldState& state)
    {
        uint64_t hash = 14695981039346656037ull;
        const auto mix = [&hash](uint64_t byte)
        {
            hash ^= byte & 0xffu;
            hash *= 1099511628211ull;
        };
        const uint64_t value = static_cast<uint64_t>(state.value);
        for (int shift = 0; shift < 64; shift += 8)
        {
            mix(value >> shift);
        }
        mix(state.occupancy.occupied.to_ulong());
        return hash;
    }

    uint32_t ringDepthTicks() const { return m_ringDepthTicks; }
    std::size_t heldTickCount() const { return m_ring.size(); }

    const MockWorldState& live() const { return m_live; }
    void setLiveValue(int64_t value) { m_live.value = value; }

    const MockWorldState& scratch() const { return m_scratch; }

    std::optional<MockWorldState> heldState(SimTick tick) const
    {
        const HeldTick* held = find(tick);
        return held != nullptr ? std::optional<MockWorldState>(held->state) : std::nullopt;
    }

    const std::vector<MockWorldCall>& calls() const { return m_calls; }
    void clearCalls() { m_calls.clear(); }

    std::vector<MockWorldCall> callsOf(MockWorldCallKind kind) const
    {
        std::vector<MockWorldCall> matching;
        for (const MockWorldCall& call : m_calls)
        {
            if (call.kind == kind)
            {
                matching.push_back(call);
            }
        }
        return matching;
    }

private:
    struct HeldTick
    {
        SimTick tick = 0;
        MockWorldState state;
    };

    const HeldTick* find(SimTick tick) const
    {
        const auto it = std::find_if(m_ring.begin(), m_ring.end(),
            [tick](const HeldTick& held) { return held.tick == tick; });
        return it != m_ring.end() ? &*it : nullptr;
    }

    void store(SimTick tick, const MockWorldState& state)
    {
        const auto existing = std::find_if(m_ring.begin(), m_ring.end(),
            [tick](const HeldTick& held) { return held.tick == tick; });
        if (existing != m_ring.end())
        {
            existing->state = state;
            return;
        }
        m_ring.push_back({ tick, state });
        if (m_ring.size() > m_ringDepthTicks)
        {
            m_ring.erase(std::min_element(m_ring.begin(), m_ring.end(),
                [](const HeldTick& a, const HeldTick& b) { return a.tick < b.tick; }));
        }
    }

    void record(const MockWorldCall& call) const { m_calls.push_back(call); }

    uint32_t m_ringDepthTicks = 0;
    MockWorldState m_live;
    MockWorldState m_scratch;
    std::vector<HeldTick> m_ring;
    mutable std::vector<MockWorldCall> m_calls;
};

class MockStaticWorldBuilder
{
public:
    StaticWorldBuildReport build(const StaticWorldDescription& description)
    {
        m_received.push_back(description);
        StaticWorldBuildReport report;
        for (const StaticShapeDescriptor& descriptor : description.shapes)
        {
            ++report.shapeCountByType[descriptor.shape.index()];
        }
        return report;
    }

    const std::vector<StaticWorldDescription>& received() const { return m_received; }

private:
    std::vector<StaticWorldDescription> m_received;
};

static_assert(PhysicsWorldAdapter<MockPhysicsWorld>, "MockPhysicsWorld must model PhysicsWorldAdapter");
static_assert(StaticWorldBuilder<MockStaticWorldBuilder>, "MockStaticWorldBuilder must model StaticWorldBuilder");
