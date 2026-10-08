// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltCollisionRule.h"
#include "OGSimulationJolt/JoltLayerTable.h"

#include <algorithm>
#include <bit>
#include <random>
#include <string>
#include <vector>

using namespace joltTestRig;

namespace
{
    enum class ChaosResponse : uint8_t
    {
        Overlap,
        Block
    };

    struct BrawlerShape
    {
        const char* name = "";
        uint32_t categories = 0;
        uint32_t blockingCategories = 0;
        bool blocksEveryChannel = false;
    };

    const std::vector<BrawlerShape>& brawlerShapes()
    {
        static const std::vector<BrawlerShape> shapes = {
            { "capsule (character, world|character)", bit(kCharacter), bit(kWorld) | bit(kCharacter), false },
            { "weapon/radial (body, none)", bit(kBody), 0u, false },
            { "guard (guard, none)", bit(kGuard), 0u, false },
            { "projectile (projectile, none)", bit(kProjectile), 0u, false },
            { "world static, BlockAll (world, every mapped channel)", bit(kWorld), kAllMappedCategories, true }
        };
        return shapes;
    }

    uint32_t chaosObjectChannel(const BrawlerShape& shape)
    {
        return static_cast<uint32_t>(std::countr_zero(shape.categories));
    }

    ChaosResponse chaosResponse(const BrawlerShape& shape, uint32_t channel)
    {
        if (shape.blocksEveryChannel)
        {
            return ChaosResponse::Block;
        }
        return (shape.blockingCategories & bit(channel)) != 0u ? ChaosResponse::Block : ChaosResponse::Overlap;
    }

    bool chaosCollides(const BrawlerShape& a, const BrawlerShape& b)
    {
        return chaosResponse(a, chaosObjectChannel(b)) == ChaosResponse::Block
            && chaosResponse(b, chaosObjectChannel(a)) == ChaosResponse::Block;
    }

    uint32_t contactsBetweenSlotCapsuleAndStatic(uint32_t capsuleIn, uint32_t capsuleWith, const JoltLayerKey& staticKey, bool occupied)
    {
        RuntimeLease lease;
        JoltWorldConfig config;
        config.simulatableSlots = 1;
        config.slotTemplate = { capsuleTemplate(capsuleIn, capsuleWith, 0) };
        config.staticLayers = { staticKey };
        config.parkingPositionCm = glm::vec3(0.f, 0.f, 90.f);
        config.tempAllocatorBytes = 4u << 20;
        JoltWorld world(lease.runtime, config, nullptr);
        addFloor(world, staticKey);
        if (occupied)
        {
            world.applyOccupancy(occupancyOf({ 0 }));
        }

        CountingContactListener contacts;
        world.physics().SetContactListener(&contacts);
        for (uint32_t index = 0; index < 30; ++index)
        {
            world.step(kDt);
        }
        world.physics().SetContactListener(nullptr);
        return contacts.added + contacts.persisted;
    }
}

TEST_CASE("JoltCollisionRule.ChaosAndTruthTable", "[Jolt]")
{
    CHECK(joltCollisionRule::shouldCollide(0b01u, 0b10u, 0b10u, 0b01u));
    CHECK_FALSE(joltCollisionRule::shouldCollide(0b01u, 0b10u, 0b10u, 0b00u));
    CHECK_FALSE(joltCollisionRule::shouldCollide(0b01u, 0b00u, 0b10u, 0b01u));
    CHECK_FALSE(joltCollisionRule::shouldCollide(0b01u, 0b00u, 0b10u, 0b00u));
    CHECK_FALSE(joltCollisionRule::shouldCollide(0u, 0u, 0b10u, ~0u));
    CHECK(joltCollisionRule::queryMatches(0b1000u, 0b1100u));
    CHECK(joltCollisionRule::queryMatches(0b0100u, 0b1100u));
    CHECK_FALSE(joltCollisionRule::queryMatches(0b0010u, 0b1100u));
    CHECK_FALSE(joltCollisionRule::queryMatches(0u, ~0u));
}

TEST_CASE("JoltCollisionRule.ContactsInAWorldFollowTheAndRule", "[Jolt]")
{
    const uint32_t a = bit(kCharacter);
    const uint32_t b = bit(kWorld);

    SECTION("(i) both list each other: contact")
    {
        CHECK(contactsBetweenSlotCapsuleAndStatic(a, b, JoltLayerKey{ JoltBroadPhaseClass::Static, b, a }, true) > 0u);
    }
    SECTION("(ii) only the capsule lists the static: no contact (Chaos overlap)")
    {
        CHECK(contactsBetweenSlotCapsuleAndStatic(a, b, JoltLayerKey{ JoltBroadPhaseClass::Static, b, 0u }, true) == 0u);
    }
    SECTION("(ii) only the static lists the capsule: no contact (Chaos overlap)")
    {
        CHECK(contactsBetweenSlotCapsuleAndStatic(a, 0u, JoltLayerKey{ JoltBroadPhaseClass::Static, b, a }, true) == 0u);
    }
    SECTION("(iii) neither lists the other: no contact")
    {
        CHECK(contactsBetweenSlotCapsuleAndStatic(a, 0u, JoltLayerKey{ JoltBroadPhaseClass::Static, b, 0u }, true) == 0u);
    }
    SECTION("(iv) parked: no contact, even with a static that blocks everything")
    {
        CHECK(contactsBetweenSlotCapsuleAndStatic(a, b, JoltLayerKey{ JoltBroadPhaseClass::Static, b, ~0u }, false) == 0u);
    }
}

TEST_CASE("JoltCollisionRule.M1EquivalenceWithChaosForEveryBrawlerShapePair", "[Jolt]")
{
    const std::vector<BrawlerShape>& shapes = brawlerShapes();
    for (const BrawlerShape& shape : shapes)
    {
        INFO(shape.name);
        CHECK(std::popcount(shape.categories) == 1);
    }

    uint32_t pairs = 0;
    uint32_t colliding = 0;
    for (size_t first = 0; first < shapes.size(); ++first)
    {
        for (size_t second = first; second < shapes.size(); ++second)
        {
            const BrawlerShape& a = shapes[first];
            const BrawlerShape& b = shapes[second];
            const bool chaos = chaosCollides(a, b);
            const bool jolt = joltCollisionRule::shouldCollide(a.categories, a.blockingCategories, b.categories, b.blockingCategories);
            INFO(a.name << " x " << b.name);
            CHECK(jolt == chaos);
            ++pairs;
            colliding += jolt ? 1u : 0u;
        }
    }
    CHECK(pairs == 15u);
    CHECK(colliding == 3u);
    CHECK(joltCollisionRule::shouldCollide(shapes[0].categories, shapes[0].blockingCategories, shapes[0].categories, shapes[0].blockingCategories));
    CHECK(joltCollisionRule::shouldCollide(shapes[0].categories, shapes[0].blockingCategories, shapes[4].categories, shapes[4].blockingCategories));
}

TEST_CASE("JoltLayerTable.AllocatesDeterministicallyAndIncludesParked", "[Jolt]")
{
    std::vector<JoltLayerKey> keys = {
        JoltLayerKey{ JoltBroadPhaseClass::Moving, bit(kCharacter), bit(kWorld) | bit(kCharacter) },
        JoltLayerKey{ JoltBroadPhaseClass::Moving, bit(kBody), 0u },
        JoltLayerKey{ JoltBroadPhaseClass::Moving, bit(kGuard), 0u },
        JoltLayerKey{ JoltBroadPhaseClass::Moving, bit(kProjectile), 0u },
        JoltLayerKey{ JoltBroadPhaseClass::Moving, bit(kProjectile), 0u },
        kWorldStaticKey,
        JoltLayerKey{ JoltBroadPhaseClass::Static, bit(kWorld), bit(kCharacter) }
    };
    std::vector<JoltLayerKey> shuffled = keys;
    std::mt19937 random(1234u);
    std::shuffle(shuffled.begin(), shuffled.end(), random);

    const JoltLayerTable table(keys);
    const JoltLayerTable shuffledTable(shuffled);
    CHECK(table.layerCount() == 7u);
    REQUIRE(shuffledTable.layerCount() == table.layerCount());
    for (JPH::ObjectLayer layer = 0; layer < table.layerCount(); ++layer)
    {
        CHECK(table.keyOf(layer) == shuffledTable.keyOf(layer));
        CHECK(table.layerOf(table.keyOf(layer)) == layer);
    }
    for (JPH::ObjectLayer layer = 1; layer < table.layerCount(); ++layer)
    {
        CHECK(table.keyOf(layer - 1u) < table.keyOf(layer));
    }

    CHECK(table.keyOf(table.parkedLayer()) == kParkedLayerKey);
    CHECK_FALSE(table.findLayer(JoltLayerKey{ JoltBroadPhaseClass::Moving, bit(kWorld), 0u }).has_value());

    const JPH::ObjectLayer capsule = table.layerOf(keys[0]);
    const JPH::ObjectLayer worldStatic = table.layerOf(kWorldStaticKey);
    CHECK(table.broadPhaseLayers().GetNumBroadPhaseLayers() == 2u);
    CHECK(table.broadPhaseLayers().GetBroadPhaseLayer(capsule) == JPH::BroadPhaseLayer(static_cast<uint8_t>(JoltBroadPhaseClass::Moving)));
    CHECK(table.broadPhaseLayers().GetBroadPhaseLayer(worldStatic) == JPH::BroadPhaseLayer(static_cast<uint8_t>(JoltBroadPhaseClass::Static)));
    CHECK(table.broadPhaseLayers().GetBroadPhaseLayer(table.parkedLayer()) == JPH::BroadPhaseLayer(static_cast<uint8_t>(JoltBroadPhaseClass::Moving)));

    CHECK(table.objectLayerPairFilter().ShouldCollide(capsule, capsule));
    CHECK(table.objectLayerPairFilter().ShouldCollide(capsule, worldStatic));
    CHECK_FALSE(table.objectLayerPairFilter().ShouldCollide(capsule, table.layerOf(keys[1])));
    CHECK_FALSE(table.objectLayerPairFilter().ShouldCollide(table.parkedLayer(), worldStatic));

    for (uint8_t broadPhase = 0; broadPhase < kJoltBroadPhaseClassCount; ++broadPhase)
    {
        CHECK_FALSE(table.objectVsBroadPhaseFilter().ShouldCollide(table.parkedLayer(), JPH::BroadPhaseLayer(broadPhase)));
        CHECK_FALSE(table.objectVsBroadPhaseFilter().ShouldCollide(table.layerOf(keys[1]), JPH::BroadPhaseLayer(broadPhase)));
        CHECK(table.objectVsBroadPhaseFilter().ShouldCollide(capsule, JPH::BroadPhaseLayer(broadPhase)));
    }

    CHECK(table.queryMatches(capsule, bit(kCharacter) | bit(kWorld)));
    CHECK_FALSE(table.queryMatches(capsule, bit(kBody)));
    CHECK_FALSE(table.queryMatches(table.parkedLayer(), ~0u));
    const JoltQueryLayerFilter worldOnly(table, bit(kWorld));
    CHECK(worldOnly.ShouldCollide(worldStatic));
    CHECK_FALSE(worldOnly.ShouldCollide(capsule));
}

TEST_CASE("JoltLayerTable.StaticKeysComeFromTheStaticDescription", "[Jolt]")
{
    StaticWorldDescription description;
    description.shapes.push_back(StaticShapeDescriptor{ StaticBox{ glm::vec3(100.f) }, glm::mat4(1.f), CollisionCategories{ bit(kWorld) }, CollisionCategories{ kAllMappedCategories } });
    description.shapes.push_back(StaticShapeDescriptor{ StaticSphere{ 50.f }, glm::mat4(1.f), CollisionCategories{ bit(kWorld) }, CollisionCategories{ kAllMappedCategories } });
    description.shapes.push_back(StaticShapeDescriptor{ StaticSphere{ 50.f }, glm::mat4(1.f), CollisionCategories{ bit(kWorld) }, CollisionCategories{ bit(kCharacter) } });

    const std::vector<JoltLayerKey> keys = staticLayerKeysOf(description);
    REQUIRE(keys.size() == 3u);
    for (const JoltLayerKey& key : keys)
    {
        CHECK(key.broadPhaseClass == JoltBroadPhaseClass::Static);
    }
    const JoltLayerTable table(keys);
    CHECK(table.layerCount() == 3u);
}

#endif // WITH_LOW_LEVEL_TESTS
