// SPDX-License-Identifier: MPL-2.0
// docs/JoltSortTotalOrderTest-rationale.md
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltLayerTable.h"
#include "OGSimulationJolt/JoltStaticWorldBuilder.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>

#include "glm/geometric.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <tuple>
#include <vector>

using namespace joltTestRig;

namespace
{
    constexpr uint64_t kSharedStableKey = 0x00000000'7A570023ull;

    struct TranslationBits
    {
        uint32_t x = 0;
        uint32_t y = 0;
        uint32_t z = 0;

        glm::vec3 metres() const { return glm::vec3(std::bit_cast<float>(x), std::bit_cast<float>(y), std::bit_cast<float>(z)); }
    };

    constexpr TranslationBits kFirst{ 0x3F98E296u, 0x3FC37A19u, 0x3FC149E0u };
    constexpr TranslationBits kSecond{ 0x3FDA4036u, 0x3FFFC041u, 0x3FDD5B80u };

    StaticShapeDescriptor collidingBox(const TranslationBits& translation)
    {
        StaticShapeDescriptor descriptor;
        descriptor.shape = StaticBox{ glm::vec3(0.25f, 0.25f, 0.25f) };
        descriptor.localToWorld = glm::mat4(1.f);
        descriptor.localToWorld[3] = glm::vec4(translation.metres(), 1.f);
        descriptor.categories = CollisionCategories{ bit(kWorld) };
        descriptor.blockingCategories = CollisionCategories{ kAllMappedCategories };
        descriptor.friction = 0.7f;
        descriptor.restitution = 0.3f;
        descriptor.stableKey = kSharedStableKey;
        return descriptor;
    }

    uint64_t fnv1a64OfWords(const std::vector<uint32_t>& words)
    {
        uint64_t hash = 14695981039346656037ull;
        for (const uint32_t word : words)
        {
            for (int i = 0; i < 4; ++i)
            {
                hash ^= static_cast<uint8_t>(word >> (8 * i));
                hash *= 1099511628211ull;
            }
        }
        return hash;
    }

    std::vector<uint32_t> boxContentWords(const StaticShapeDescriptor& descriptor)
    {
        const StaticBox& box = std::get<StaticBox>(descriptor.shape);
        std::vector<uint32_t> words{ static_cast<uint32_t>(descriptor.shape.index()) };
        for (int axis = 0; axis < 3; ++axis)
        {
            words.push_back(std::bit_cast<uint32_t>(box.halfExtents[axis]));
        }
        for (int column = 0; column < 4; ++column)
        {
            for (int row = 0; row < 4; ++row)
            {
                words.push_back(std::bit_cast<uint32_t>(descriptor.localToWorld[column][row]));
            }
        }
        words.push_back(descriptor.categories.bits);
        words.push_back(descriptor.blockingCategories.bits);
        words.push_back(std::bit_cast<uint32_t>(descriptor.friction));
        words.push_back(std::bit_cast<uint32_t>(descriptor.restitution));
        return words;
    }

    std::vector<glm::vec3> staticBodyCentresInCreationOrder(const StaticWorldDescription& description)
    {
        RuntimeLease lease;
        JoltWorldConfig config = brawlerWorldConfig(4);
        config.staticLayers = staticLayerKeysOf(description);
        JoltWorld world(lease.runtime, config, &logJoltProblems);
        JoltStaticWorldBuilderOptions options;
        options.maxShapesPerChunk = 1;
        JoltStaticWorldBuilder builder(world, [](const char*) {}, options);
        builder.build(description);

        std::vector<glm::vec3> centres;
        const JPH::BodyLockInterface& locks = world.physics().GetBodyLockInterfaceNoLock();
        for (const JPH::BodyID& id : builder.stats().bodies)
        {
            JPH::BodyLockRead lock(locks, id);
            REQUIRE(lock.Succeeded());
            const JPH::Vec3 centre(lock.GetBody().GetWorldSpaceBounds().GetCenter());
            centres.push_back(glm::vec3(centre.GetX(), centre.GetY(), centre.GetZ()));
        }
        return centres;
    }
} // namespace

TEST_CASE("JoltSortTotalOrder.StaticElementsWithCollidingContentHashesSortByContent", "[Jolt][StaticGeometry]")
{
    const StaticShapeDescriptor first = collidingBox(kFirst);
    const StaticShapeDescriptor second = collidingBox(kSecond);
    const std::vector<uint32_t> firstWords = boxContentWords(first);
    const std::vector<uint32_t> secondWords = boxContentWords(second);
    REQUIRE(firstWords != secondWords);
    REQUIRE(fnv1a64OfWords(firstWords) == fnv1a64OfWords(secondWords));
    REQUIRE(firstWords < secondWords);
    REQUIRE(glm::distance(kFirst.metres(), kSecond.metres()) > 0.1f);

    StaticWorldDescription inOrder;
    inOrder.shapes = { first, second };
    StaticWorldDescription reversed;
    reversed.shapes = { second, first };

    for (const StaticWorldDescription* description : { &inOrder, &reversed })
    {
        const std::vector<glm::vec3> centres = staticBodyCentresInCreationOrder(*description);
        REQUIRE(centres.size() == 2u);
        CHECK(glm::distance(centres[0], kFirst.metres()) < 1e-3f);
        CHECK(glm::distance(centres[1], kSecond.metres()) < 1e-3f);
    }
}

#endif
