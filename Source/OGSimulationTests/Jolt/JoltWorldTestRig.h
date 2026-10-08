// SPDX-License-Identifier: MPL-2.0

#pragma once

#include "OGSimulationJolt/JoltFpEnvironment.h"
#include "OGSimulationJolt/JoltRuntime.h"
#include "OGSimulationJolt/JoltUnits.h"
#include "OGSimulationJolt/JoltStateRing.h"
#include "OGSimulationJolt/JoltWorld.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace joltTestRig
{
    inline constexpr uint32_t kBody = 0;
    inline constexpr uint32_t kGuard = 1;
    inline constexpr uint32_t kQueryRouting = 2;
    inline constexpr uint32_t kProjectile = 3;
    inline constexpr uint32_t kWorld = 4;
    inline constexpr uint32_t kCharacter = 5;

    inline constexpr uint32_t bit(uint32_t category) { return 1u << category; }

    inline constexpr uint32_t kAllMappedCategories =
        bit(kBody) | bit(kGuard) | bit(kQueryRouting) | bit(kProjectile) | bit(kWorld) | bit(kCharacter);

    inline constexpr uint32_t kTemplateRadial = 0;
    inline constexpr uint32_t kTemplateGuard = 1;
    inline constexpr uint32_t kTemplateProjectileFirst = 2;
    inline constexpr uint32_t kTemplateCapsule = 5;
    inline constexpr uint32_t kBodiesPerBrawler = 6;

    inline constexpr float kDt = 1.f / 60.f;

    inline const JoltLayerKey kWorldStaticKey{ JoltBroadPhaseClass::Static, bit(kWorld), kAllMappedCategories };

    inline SlotBodyTemplate sphereTemplate(float radiusCm, uint32_t categories, uint32_t blocking, uint8_t declarationIndex)
    {
        SlotBodyTemplate slotTemplate;
        slotTemplate.descriptor.body = BodyDescriptor{ .simulatePhysics = true, .enableGravity = false };
        slotTemplate.descriptor.shapes = { ShapeDescriptor{ SphereGeometry{ radiusCm }, CollisionCategories{ categories }, CollisionCategories{ blocking } } };
        slotTemplate.declarationIndex = declarationIndex;
        return slotTemplate;
    }

    inline SlotBodyTemplate capsuleTemplate(uint32_t categories, uint32_t blocking, uint8_t declarationIndex)
    {
        SlotBodyTemplate slotTemplate;
        slotTemplate.descriptor.body = BodyDescriptor{ .simulatePhysics = true, .enableGravity = true, .isRoot = true, .lockRotation = true,
            .resimPolicy = BodyResimPolicy::Resimulate };
        slotTemplate.descriptor.shapes = { ShapeDescriptor{ CapsuleGeometry{ 42.f, 96.f }, CollisionCategories{ categories }, CollisionCategories{ blocking } } };
        slotTemplate.declarationIndex = declarationIndex;
        return slotTemplate;
    }

    inline std::vector<SlotBodyTemplate> brawlerSlotTemplate(uint32_t projectileBlocking = 0u)
    {
        return {
            sphereTemplate(30.f, bit(kBody), 0u, 0),
            sphereTemplate(40.f, bit(kGuard), 0u, 1),
            sphereTemplate(30.f, bit(kProjectile), projectileBlocking, 2),
            sphereTemplate(30.f, bit(kProjectile), projectileBlocking, 3),
            sphereTemplate(30.f, bit(kProjectile), projectileBlocking, 4),
            capsuleTemplate(bit(kCharacter), bit(kWorld) | bit(kCharacter), 5)
        };
    }

    inline JoltWorldConfig brawlerWorldConfig(uint32_t ringDepthTicks)
    {
        JoltWorldConfig config;
        config.slotTemplate = brawlerSlotTemplate();
        config.staticLayers = { kWorldStaticKey };
        config.ringDepthTicks = ringDepthTicks;
        config.tempAllocatorBytes = 4u << 20;
        return config;
    }

    inline void logJoltProblems(const char* line)
    {
        if (std::strncmp(line, "JoltRuntime: ", 13) != 0)
        {
            std::fprintf(stderr, "%s\n", line);
        }
    }

    struct RuntimeLease
    {
        RuntimeLease() : runtime(JoltRuntime::acquire(&logJoltProblems)) {}
        ~RuntimeLease() { JoltRuntime::release(); }
        RuntimeLease(const RuntimeLease&) = delete;
        RuntimeLease& operator=(const RuntimeLease&) = delete;

        JoltRuntime& runtime;
    };

    inline JPH::BodyID addFloor(JoltWorld& world, const JoltLayerKey& key = kWorldStaticKey, float topZMetres = 0.f)
    {
        JPH::BodyCreationSettings floor(new JPH::BoxShape(JPH::Vec3(20.f, 20.f, 0.5f)), JPH::RVec3(0.f, 0.f, topZMetres - 0.5f),
            JPH::Quat::sIdentity(), JPH::EMotionType::Static, world.layers().layerOf(key));
        const JPH::BodyID id = world.createStaticBody(floor);
        world.bodies().AddBody(id, JPH::EActivation::DontActivate);
        world.physics().OptimizeBroadPhase();
        return id;
    }

    inline BodySlotOccupancy occupancyOf(std::initializer_list<uint32_t> slots)
    {
        BodySlotOccupancy occupancy;
        for (const uint32_t slot : slots)
        {
            occupancy.occupied.set(slot);
        }
        return occupancy;
    }

    inline BodySlotOccupancy allOccupied()
    {
        BodySlotOccupancy occupancy;
        occupancy.occupied.set();
        return occupancy;
    }

    inline void placeBrawler(JoltWorld& world, uint32_t slot, float xMetres, float yMetres, float zMetres)
    {
        JPH::BodyInterface& bodies = world.bodies();
        bodies.SetPosition(world.slotBodyId(slot, kTemplateCapsule), JPH::RVec3(xMetres, yMetres, zMetres), JPH::EActivation::Activate);
        for (uint32_t index = 0; index < kTemplateCapsule; ++index)
        {
            bodies.SetPosition(world.slotBodyId(slot, index), JPH::RVec3(xMetres + 0.6f, yMetres + 0.1f * float(index), zMetres), JPH::EActivation::Activate);
        }
    }

    inline void placeAllBrawlers(JoltWorld& world)
    {
        for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
        {
            placeBrawler(world, slot, float(slot % 4u) * 1.5f - 2.25f, float(slot / 4u) * 1.5f - 0.75f, 1.0f + 0.1f * float(slot));
        }
    }

    inline float scriptedForce(uint32_t tick, uint32_t slot, uint32_t templateIndex, uint32_t axis)
    {
        const int32_t pattern = int32_t((tick * 7u + slot * 13u + templateIndex * 5u + axis * 3u) % 11u) - 5;
        return float(pattern) * (templateIndex == kTemplateCapsule ? 400.f : 20.f);
    }

    inline void applyScriptedForces(JoltWorld& world, uint32_t tick)
    {
        JPH::BodyInterface& bodies = world.bodies();
        for (uint32_t slot = 0; slot < world.simulatableSlots(); ++slot)
        {
            if (!world.appliedOccupancy().occupied.test(slot))
            {
                continue;
            }
            for (uint32_t index = 0; index < world.bodiesPerSlot(); ++index)
            {
                bodies.AddForce(world.slotBodyId(slot, index),
                    JPH::Vec3(scriptedForce(tick, slot, index, 0), scriptedForce(tick, slot, index, 1), 0.f));
            }
        }
    }

    inline std::vector<uint8_t> fullState(const JoltWorld& world)
    {
        std::vector<uint8_t> buffer(1u << 20);
        JoltFixedStateRecorder recorder;
        recorder.beginWrite(buffer.data(), buffer.size());
        world.physics().SaveState(recorder, JPH::EStateRecorderState::All, nullptr);
        buffer.resize(recorder.IsFailed() ? 0u : recorder.bytesWritten());
        return buffer;
    }

    inline bool sameBytes(const std::vector<uint8_t>& first, const std::vector<uint8_t>& second)
    {
        return !first.empty() && first == second;
    }

    inline bool restoreFullState(JoltWorld& world, const std::vector<uint8_t>& bytes)
    {
        JoltFixedStateRecorder recorder;
        recorder.beginRead(bytes.data(), bytes.size());
        return world.physics().RestoreState(recorder) && !recorder.IsFailed() && recorder.fullyRead();
    }

    class CountingContactListener final : public JPH::ContactListener
    {
    public:
        void OnContactAdded(const JPH::Body&, const JPH::Body&, const JPH::ContactManifold&, JPH::ContactSettings&) override { ++added; }
        void OnContactPersisted(const JPH::Body&, const JPH::Body&, const JPH::ContactManifold&, JPH::ContactSettings&) override { ++persisted; }

        uint32_t added = 0;
        uint32_t persisted = 0;
    };
} // namespace joltTestRig
