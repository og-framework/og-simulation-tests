// SPDX-License-Identifier: MPL-2.0
#if WITH_LOW_LEVEL_TESTS

#include "catch_amalgamated.hpp"
#include "JoltWorldTestRig.h"

#include "OGSimulationJolt/JoltBodyDefaults.h"
#include "OGSimulationJolt/JoltPhysicsBodyAdapter.h"
#include "OGSimulationJolt/JoltPhysicsBodyReaderAdapter.h"
#include "OGSimulationJolt/JoltPhysicsFactory.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>

#include "glm/ext/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "OGSimulation/PhysicsBodyAdapter.h"
#include "OGSimulation/PhysicsBodyReaderAdapter.h"
#include "OGSimulation/PhysicsObjectFactory.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace joltTestRig;

namespace
{
    constexpr uint32_t kCategoryBody = 0;
    constexpr uint32_t kCategoryGuard = 1;
    constexpr uint32_t kCategoryProjectile = 3;
    constexpr uint32_t kCategoryWorld = 4;
    constexpr uint32_t kCategoryCharacter = 5;

    constexpr uint32_t kRadial = 0;
    constexpr uint32_t kGuardBody = 1;
    constexpr uint32_t kCapsule = 5;

    constexpr uint32_t kSimulatableId = 7;

    SlotBodyTemplate declaration(const BodyDescriptor& body, const QueryGeometry& geometry, uint32_t categories, uint32_t blocking, uint8_t index)
    {
        SlotBodyTemplate slotTemplate;
        slotTemplate.descriptor.body = body;
        slotTemplate.descriptor.shapes = { ShapeDescriptor{ geometry, CollisionCategories{ categories }, CollisionCategories{ blocking } } };
        slotTemplate.declarationIndex = index;
        return slotTemplate;
    }

    std::vector<SlotBodyTemplate> brawlerDeclarations()
    {
        const BodyDescriptor sphereBody{ .simulatePhysics = true, .enableGravity = false };
        const BodyDescriptor capsuleBody{ .simulatePhysics = true, .enableGravity = false, .isRoot = true, .lockRotation = true,
            .resimPolicy = BodyResimPolicy::Resimulate };
        return {
            declaration(sphereBody, SphereGeometry{ 30.f }, bit(kCategoryBody), 0u, 0),
            declaration(sphereBody, SphereGeometry{ 40.f }, bit(kCategoryGuard), 0u, 1),
            declaration(sphereBody, SphereGeometry{ 30.f }, bit(kCategoryProjectile), 0u, 2),
            declaration(sphereBody, SphereGeometry{ 30.f }, bit(kCategoryProjectile), 0u, 3),
            declaration(sphereBody, SphereGeometry{ 30.f }, bit(kCategoryProjectile), 0u, 4),
            declaration(capsuleBody, CapsuleGeometry{ 42.f, 96.f }, bit(kCategoryCharacter), bit(kCategoryWorld) | bit(kCategoryCharacter), 5)
        };
    }

    struct ChaosMass
    {
        double massKilograms;
        std::array<double, 3> inertiaKilogramSquareCentimetres;
    };

    constexpr ChaosMass kChaosSphere30{ 34.6808138, { 12485.093, 12485.093, 12485.093 } };
    constexpr ChaosMass kChaosSphere40{ 66.2523729, { 42401.5186, 42401.5186, 42401.5186 } };
    constexpr ChaosMass kChaosCapsule42x96{ 165.527146, { 454866.174, 454866.174, 136024.556 } };

    const std::array<ChaosMass, 6> kChaosMassPerDeclaration{ kChaosSphere30, kChaosSphere40, kChaosSphere30, kChaosSphere30, kChaosSphere30, kChaosCapsule42x96 };

    JoltWorldConfig brawlerDeclarationWorld(uint32_t ringDepthTicks)
    {
        JoltWorldConfig config;
        config.slotTemplate = brawlerDeclarations();
        config.staticLayers = { kWorldStaticKey };
        config.ringDepthTicks = ringDepthTicks;
        config.tempAllocatorBytes = 4u << 20;
        return config;
    }

    struct BoundBrawler
    {
        JoltWorld world;
        JoltPhysicsBodyAdapter adapter;
        JoltPhysicsBodyReaderAdapter reader;
        std::vector<BodyId> bodies;

        BoundBrawler(JoltRuntime& runtime, uint32_t ringDepthTicks = 0, JoltPhysicsFactoryOptions options = {})
            : world(runtime, brawlerDeclarationWorld(ringDepthTicks), nullptr)
            , adapter(world)
            , reader(world, adapter.bindTable())
        {
            JoltPhysicsFactory factory(adapter, brawlerDeclarations(), 0, kSimulatableId, options);
            for (const SlotBodyTemplate& slotTemplate : brawlerDeclarations())
            {
                bodies.push_back(factory.createPhysicalObject(slotTemplate.descriptor, "declaration").bodyId);
            }
            world.applyOccupancy(occupancyOf({ 0 }));
        }
    };

    bool withinRelative(double actual, double expected, double relative)
    {
        return std::abs(actual - expected) <= relative * std::abs(expected);
    }

    void overrideAngularDamping(JoltWorld& world, BodyId body, float damping)
    {
        JPH::BodyLockWrite lock(world.physics().GetBodyLockInterfaceNoLock(), JoltPhysicsBodyAdapter::joltBodyIdOf(body));
        REQUIRE(lock.Succeeded());
        lock.GetBody().GetMotionProperties()->SetAngularDamping(damping);
    }

    struct AuthoredSequence
    {
        std::vector<uint32_t> segmentStartTicks;
        std::vector<double> accelerations;
        uint32_t endTick = 0;

        double accelerationAtTick(uint32_t tick) const
        {
            for (size_t segment = 0; segment + 1 < segmentStartTicks.size(); ++segment)
            {
                if (tick >= segmentStartTicks[segment] && tick < segmentStartTicks[segment + 1])
                {
                    return accelerations[segment];
                }
            }
            if (tick >= segmentStartTicks.back() && tick < endTick)
            {
                return accelerations.back();
            }
            return 0.0;
        }
    };

    AuthoredSequence leftToLeftSequence()
    {
        constexpr double pi = 3.14159265358979323846;
        const std::vector<std::pair<uint32_t, double>> points{
            { 0u, 4.0 * pi / 8.0 }, { 6u, 6.0 * pi / 8.0 }, { 9u, 8.0 * pi / 8.0 }, { 12u, 10.0 * pi / 8.0 },
            { 15u, 12.0 * pi / 8.0 }, { 18u, 14.0 * pi / 8.0 }, { 24u, 17.0 * pi / 8.0 } };
        constexpr uint32_t ticksToReachZeroVelocity = 6;
        const double dt = 1.0 / 60.0;

        AuthoredSequence sequence;
        double velocity = 0.0;
        for (size_t index = 0; index + 1 < points.size(); ++index)
        {
            const double segmentSeconds = double(points[index + 1].first - points[index].first) * dt;
            const double acceleration = 2.0 * ((points[index + 1].second - points[index].second) - velocity * segmentSeconds) / (segmentSeconds * segmentSeconds);
            sequence.segmentStartTicks.push_back(points[index].first);
            sequence.accelerations.push_back(acceleration);
            velocity += acceleration * segmentSeconds;
        }
        sequence.segmentStartTicks.push_back(points.back().first);
        sequence.accelerations.push_back(-velocity / (double(ticksToReachZeroVelocity) * dt));
        sequence.endTick = points.back().first + ticksToReachZeroVelocity;
        return sequence;
    }

    struct SpinRun
    {
        double maxAbsoluteError = 0.0;
        double peakOmega = 0.0;
        double finalOmega = 0.0;
        double maxOffAxisOmega = 0.0;
    };

    SpinRun spinRadialWithAuthoredTorque(BoundBrawler& rig, uint32_t ticks)
    {
        const AuthoredSequence sequence = leftToLeftSequence();
        const BodyId radial = rig.bodies[kRadial];
        const BodyId capsule = rig.bodies[kCapsule];
        rig.adapter.setBodyTransform(capsule, glm::translate(glm::mat4(1.f), glm::vec3(500.f, 0.f, 200.f)));
        rig.adapter.setBodyTransform(radial, glm::translate(glm::mat4(1.f), glm::vec3(500.f, 0.f, 200.f)));
        rig.adapter.setBodyAngularVelocity(radial, glm::vec3(0.f));

        SpinRun run;
        double analyticOmega = 0.0;
        const glm::vec4 localSequenceAxis(0.f, 0.f, 1.f, 0.f);
        for (uint32_t tick = 0; tick < ticks; ++tick)
        {
            glm::mat4 childTransform = rig.adapter.getBodyTransform(radial);
            const glm::vec3 parentPosition(rig.adapter.getBodyTransform(capsule)[3]);
            childTransform[3] = glm::vec4(parentPosition, 1.f);
            rig.adapter.setBodyTransform(radial, childTransform);

            const double acceleration = sequence.accelerationAtTick(tick);
            const glm::vec3 worldAxis(rig.adapter.getBodyTransform(radial) * localSequenceAxis);
            const glm::vec3& inertia = rig.adapter.getBodyInertiaTensor(radial);
            rig.adapter.addBodyTorque(radial, worldAxis * float(acceleration) * inertia.z);

            rig.world.step(kDt);
            analyticOmega += acceleration * (1.0 / 60.0);

            const glm::vec3 omega = rig.adapter.captureBodyState(radial).angularVelocity;
            run.maxAbsoluteError = std::max(run.maxAbsoluteError, std::abs(double(omega.z) - analyticOmega));
            run.maxOffAxisOmega = std::max(run.maxOffAxisOmega, double(std::max(std::abs(omega.x), std::abs(omega.y))));
            run.peakOmega = std::max(run.peakOmega, std::abs(double(omega.z)));
            run.finalOmega = double(omega.z);
        }
        return run;
    }
}

TEST_CASE("JoltBodyAdapter.SatisfiesTheThreeSeamConcepts", "[Jolt]")
{
    STATIC_REQUIRE(PhysicsBodyAdapter<JoltPhysicsBodyAdapter>);
    STATIC_REQUIRE(PhysicsBodyReaderAdapter<JoltPhysicsBodyReaderAdapter>);
    STATIC_REQUIRE(PhysicsObjectFactory<JoltPhysicsFactory>);
}

TEST_CASE("JoltBodyAdapter.UnitAndQuaternionRoundTrips", "[Jolt]")
{
    const glm::vec3 centimetres(123.5f, -42.25f, 980.f);
    const JPH::Vec3 metres = joltSeamUnits::centimetreVectorToJolt(centimetres);
    CHECK(metres.GetX() == Catch::Approx(1.235f));
    CHECK(metres.GetZ() == Catch::Approx(9.8f));
    const glm::vec3 back = joltSeamUnits::metreVectorToSeam(metres);
    CHECK_THAT(back.x, Catch::Matchers::WithinRel(centimetres.x, 1e-6f));
    CHECK_THAT(back.y, Catch::Matchers::WithinRel(centimetres.y, 1e-6f));
    CHECK_THAT(back.z, Catch::Matchers::WithinRel(centimetres.z, 1e-6f));

    const glm::vec3 inertia(12485.093f, 42401.52f, 136024.56f);
    const JPH::Vec3 inertiaJolt = joltSeamUnits::inertiaToJolt(inertia);
    CHECK_THAT(inertiaJolt.GetX(), Catch::Matchers::WithinRel(1.2485093f, 1e-6f));
    const glm::vec3 inertiaBack = joltSeamUnits::inertiaToSeam(inertiaJolt);
    CHECK_THAT(inertiaBack.y, Catch::Matchers::WithinRel(inertia.y, 1e-6f));
    CHECK_THAT(inertiaBack.z, Catch::Matchers::WithinRel(inertia.z, 1e-6f));
    CHECK_THAT(joltSeamUnits::torqueToJolt(glm::vec3(0.f, 0.f, 50000.f)).GetZ(), Catch::Matchers::WithinRel(5.f, 1e-6f));

    const glm::quat seamRotation = glm::normalize(glm::quat(0.9f, 0.1f, -0.3f, 0.2f));
    const JPH::Quat joltRotation = joltSeamUnits::rotationToJolt(seamRotation);
    CHECK(joltRotation.GetW() == seamRotation.w);
    CHECK(joltRotation.GetX() == seamRotation.x);
    CHECK(joltRotation.GetY() == seamRotation.y);
    CHECK(joltRotation.GetZ() == seamRotation.z);
    const glm::quat rotationBack = joltSeamUnits::rotationToSeam(joltRotation);
    CHECK(rotationBack == seamRotation);

    RuntimeLease lease;
    BoundBrawler rig(lease.runtime);
    const BodyId radial = rig.bodies[kRadial];
    const glm::vec3 positionCm(250.f, -75.5f, 130.f);
    const glm::mat4 transform = glm::translate(glm::mat4(1.f), positionCm) * glm::mat4_cast(seamRotation);
    rig.adapter.setBodyTransform(radial, transform);
    const glm::mat4 read = rig.adapter.getBodyTransform(radial);
    for (int column = 0; column < 4; ++column)
    {
        for (int row = 0; row < 4; ++row)
        {
            CHECK_THAT(read[column][row], Catch::Matchers::WithinAbs(transform[column][row], 1e-4f));
        }
    }
    const PhysicsBodyState state = rig.adapter.captureBodyState(radial);
    CHECK_THAT(state.position.x, Catch::Matchers::WithinAbs(positionCm.x, 1e-4f));
    CHECK_THAT(state.position.y, Catch::Matchers::WithinAbs(positionCm.y, 1e-4f));
    CHECK_THAT(state.position.z, Catch::Matchers::WithinAbs(positionCm.z, 1e-4f));
    CHECK(std::abs(glm::dot(state.rotation, seamRotation)) == Catch::Approx(1.f).epsilon(1e-6f));
    CHECK(state.rotation.w == Catch::Approx(seamRotation.w).margin(1e-6f));
    CHECK(state.rotation.x == Catch::Approx(seamRotation.x).margin(1e-6f));

    rig.adapter.setBodyLinearVelocity(radial, glm::vec3(300.f, -150.f, 25.f));
    rig.adapter.setBodyAngularVelocity(radial, glm::vec3(1.5f, -2.f, 3.f));
    const PhysicsBodyState moving = rig.adapter.captureBodyState(radial);
    CHECK_THAT(moving.linearVelocity.x, Catch::Matchers::WithinRel(300.f, 1e-6f));
    CHECK_THAT(moving.linearVelocity.y, Catch::Matchers::WithinRel(-150.f, 1e-6f));
    CHECK(moving.angularVelocity == glm::vec3(1.5f, -2.f, 3.f));
}

TEST_CASE("JoltBodyAdapter.AnAccelerationOverOneStepYieldsTheExpectedVelocityChange", "[Jolt]")
{
    RuntimeLease lease;
    BoundBrawler rig(lease.runtime);
    const double c = joltBodyDefaults::kLinearDamping;
    const double dt = kDt;

    const glm::vec3 acceleration(600.f, -300.f, 120.f);
    for (const uint32_t declaration : { kRadial, kGuardBody, kCapsule })
    {
        const BodyId body = rig.bodies[declaration];
        rig.adapter.setBodyLinearVelocity(body, glm::vec3(0.f));
        rig.adapter.addBodyAcceleration(body, acceleration);
    }
    rig.world.step(kDt);
    for (const uint32_t declaration : { kRadial, kGuardBody, kCapsule })
    {
        INFO("declaration " << declaration);
        const glm::vec3 velocity = rig.adapter.captureBodyState(rig.bodies[declaration]).linearVelocity;
        for (int axis = 0; axis < 3; ++axis)
        {
            const double expected = double(acceleration[axis]) * dt * (1.0 - c * dt);
            CHECK_THAT(double(velocity[axis]), Catch::Matchers::WithinAbs(expected, 1e-5));
        }
    }

    const BodyId radial = rig.bodies[kRadial];
    const glm::vec3 before = rig.adapter.captureBodyState(radial).linearVelocity;
    const glm::vec3 velocityChange(-40.f, 10.f, 5.f);
    rig.adapter.addBodyVelocityChange(radial, velocityChange);
    rig.world.step(kDt);
    const glm::vec3 after = rig.adapter.captureBodyState(radial).linearVelocity;
    for (int axis = 0; axis < 3; ++axis)
    {
        const double expected = (double(before[axis]) + double(velocityChange[axis])) * (1.0 - c * dt);
        CHECK_THAT(double(after[axis]), Catch::Matchers::WithinAbs(expected, 1e-5));
    }
}

TEST_CASE("JoltBodyAdapter.ATorqueInSeamUnitsYieldsTauOverInertiaTimesDt", "[Jolt]")
{
    RuntimeLease lease;
    BoundBrawler rig(lease.runtime);
    const BodyId radial = rig.bodies[kRadial];

    const glm::vec3 inertia = rig.reader.getBodyInertiaTensor(radial);
    for (int axis = 0; axis < 3; ++axis)
    {
        CHECK(withinRelative(inertia[axis], kChaosSphere30.inertiaKilogramSquareCentimetres[size_t(axis)], 1e-5));
    }
    CHECK(rig.adapter.getBodyInertiaTensor(radial) == inertia);

    const glm::vec3 torque(20000.f, -35000.f, 50000.f);
    rig.adapter.setBodyAngularVelocity(radial, glm::vec3(0.f));
    rig.adapter.addBodyTorque(radial, torque);
    rig.world.step(kDt);
    const glm::vec3 omega = rig.adapter.captureBodyState(radial).angularVelocity;
    for (int axis = 0; axis < 3; ++axis)
    {
        const double expected = double(torque[axis]) / kChaosSphere30.inertiaKilogramSquareCentimetres[size_t(axis)] * double(kDt);
        CHECK_THAT(double(omega[axis]), Catch::Matchers::WithinAbs(expected, 1e-5));
    }

    const BodyId capsule = rig.bodies[kCapsule];
    rig.adapter.addBodyTorque(capsule, torque);
    rig.world.step(kDt);
    CHECK(rig.adapter.captureBodyState(capsule).angularVelocity == glm::vec3(0.f));
}

TEST_CASE("JoltPhysicsFactory.EveryBrawlerDeclarationBindsAChaosParityBody", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerDeclarationWorld(0), nullptr);
    JoltPhysicsBodyAdapter adapter(world);
    JoltPhysicsBodyReaderAdapter reader(world, adapter.bindTable());
    const std::vector<SlotBodyTemplate> declarations = brawlerDeclarations();
    constexpr uint32_t kSlot = 2;
    JoltPhysicsFactory factory(adapter, declarations, kSlot, kSimulatableId);

    CHECK(factory.parentBodyId() == JoltPhysicsBodyAdapter::bodyIdOf(world.slotBodyId(kSlot, kCapsule)));
    CHECK(world.physics().GetCombineFriction() == &joltBodyDefaults::combineFrictionAverage);
    CHECK(world.physics().GetCombineRestitution() == &joltBodyDefaults::combineRestitutionAverage);

    for (uint32_t index = 0; index < declarations.size(); ++index)
    {
        INFO("declaration " << index);
        const BodyId expectedId = JoltPhysicsBodyAdapter::bodyIdOf(world.slotBodyId(kSlot, index));
        CHECK_FALSE(adapter.isBodyResolvable(expectedId));
        CHECK_FALSE(reader.isBodyResolvable(expectedId));

        const JoltPhysicsFactory::PhysicalObjectResult result = factory.createPhysicalObject(declarations[index].descriptor, "declaration");
        REQUIRE(result.bodyId == expectedId);
        REQUIRE(result.shapeIds.size() == 1u);
        CHECK(result.shapeIds[0] == JoltPhysicsFactory::defaultShapeIdOf(expectedId, 0));
        CHECK(result.shapeIds[0].value < kShapeEnableCapacity);
        CHECK(adapter.isBodyResolvable(result.bodyId));
        CHECK(reader.isBodyResolvable(result.bodyId));
        CHECK(adapter.bindTable().bindingOf(result.bodyId) == JoltBodyBinding{ kSimulatableId, uint8_t(index) });
        CHECK(joltBodyUserData::decode(world.bodies().GetUserData(world.slotBodyId(kSlot, index))) == JoltBodyBinding{ kSimulatableId, uint8_t(index) });

        const ChaosMass& chaos = kChaosMassPerDeclaration[index];
        CHECK(withinRelative(reader.getBodyMass(result.bodyId), chaos.massKilograms, 1e-5));
        CHECK(withinRelative(adapter.getBodyMass(result.bodyId), chaos.massKilograms, 1e-5));
        const glm::vec3 inertia = reader.getBodyInertiaTensor(result.bodyId);
        for (int axis = 0; axis < 3; ++axis)
        {
            CHECK(withinRelative(inertia[axis], chaos.inertiaKilogramSquareCentimetres[size_t(axis)], 1e-5));
        }

        JPH::BodyLockRead lock(world.physics().GetBodyLockInterfaceNoLock(), world.slotBodyId(kSlot, index));
        REQUIRE(lock.Succeeded());
        const JPH::Body& body = lock.GetBody();
        const joltBodyDefaults::JoltBodyMaterial material =
            index == kCapsule ? joltBodyDefaults::kAdoptedRootMaterial : joltBodyDefaults::kCreatedBodyMaterial;
        CHECK(body.GetFriction() == material.friction);
        CHECK(body.GetRestitution() == material.restitution);
        CHECK(body.IsDynamic());
        CHECK_FALSE(body.GetAllowSleeping());
        const JPH::MotionProperties& motion = *body.GetMotionProperties();
        CHECK(motion.GetLinearDamping() == joltBodyDefaults::kLinearDamping);
        CHECK(motion.GetAngularDamping() == joltBodyDefaults::kAngularDamping);
        CHECK(motion.GetMaxAngularVelocity() == joltBodyDefaults::kMaxAngularVelocityRadiansPerSecond);
        CHECK(motion.GetMaxLinearVelocity() == joltBodyDefaults::kMaxLinearVelocityMetresPerSecond);
        CHECK(motion.GetMotionQuality() == JPH::EMotionQuality::Discrete);
        CHECK(motion.GetAllowedDOFs() == (index == kCapsule
            ? JPH::EAllowedDOFs::TranslationX | JPH::EAllowedDOFs::TranslationY | JPH::EAllowedDOFs::TranslationZ
            : JPH::EAllowedDOFs::All));
        CHECK(body.GetObjectLayer() == world.layers().parkedLayer());
    }
    CHECK(factory.boundBodyCount() == declarations.size());
    CHECK(joltBodyDefaults::kMaxAngularVelocityRadiansPerSecond == Catch::Approx(62.8318530718));

    world.applyOccupancy(occupancyOf({ kSlot }));
    for (uint32_t index = 0; index < declarations.size(); ++index)
    {
        CHECK(world.bodies().GetObjectLayer(world.slotBodyId(kSlot, index)) == world.templateLayer(index));
        CHECK(world.bodies().GetGravityFactor(world.slotBodyId(kSlot, index)) == 0.f);
    }

    JPH::BodyLockRead capsuleLock(world.physics().GetBodyLockInterfaceNoLock(), world.slotBodyId(kSlot, kCapsule));
    JPH::BodyLockRead radialLock(world.physics().GetBodyLockInterfaceNoLock(), world.slotBodyId(kSlot, kRadial));
    const JPH::SubShapeID wholeShape;
    CHECK(world.physics().GetCombineFriction()(capsuleLock.GetBody(), wholeShape, radialLock.GetBody(), wholeShape) == Catch::Approx(0.35f));
    CHECK(world.physics().GetCombineRestitution()(capsuleLock.GetBody(), wholeShape, radialLock.GetBody(), wholeShape) == Catch::Approx(0.15f));
}

TEST_CASE("JoltPhysicsFactory.TheCharacterLinearCastOptionAppliesToTheRootOnly", "[Jolt]")
{
    RuntimeLease lease;
    BoundBrawler rig(lease.runtime, 0, JoltPhysicsFactoryOptions{ .linearCastCharacters = true });
    for (uint32_t index = 0; index < rig.bodies.size(); ++index)
    {
        INFO("declaration " << index);
        CHECK(rig.world.bodies().GetMotionQuality(JoltPhysicsBodyAdapter::joltBodyIdOf(rig.bodies[index]))
            == (index == kCapsule ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete));
    }
}

TEST_CASE("JoltPhysicsFactory.DescriptorComparisonSeesEveryField", "[Jolt]")
{
    const PhysicalObjectDescriptor capsule = brawlerDeclarations()[kCapsule].descriptor;
    CHECK(JoltPhysicsFactory::sameDescriptor(capsule, capsule));

    auto differs = [&](auto mutate)
    {
        PhysicalObjectDescriptor changed = capsule;
        mutate(changed);
        return !JoltPhysicsFactory::sameDescriptor(capsule, changed);
    };
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.body.simulatePhysics = false; }));
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.body.enableGravity = true; }));
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.body.isRoot = false; }));
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.body.lockRotation = false; }));
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.body.resimPolicy = BodyResimPolicy::ReplayRecordedHistory; }));
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.shapes[0].geometry = CapsuleGeometry{ 42.f, 95.f }; }));
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.shapes[0].geometry = SphereGeometry{ 42.f }; }));
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.shapes[0].categories = CollisionCategories{ bit(kCategoryBody) }; }));
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.shapes[0].blockingCategories = CollisionCategories{ 0u }; }));
    CHECK(differs([](PhysicalObjectDescriptor& d) { d.shapes.push_back(d.shapes[0]); }));
}

TEST_CASE("JoltBodyAdapter.ResolvableMeansBoundNeverOccupied", "[Jolt]")
{
    RuntimeLease lease;
    JoltWorld world(lease.runtime, brawlerDeclarationWorld(0), nullptr);
    JoltPhysicsBodyAdapter adapter(world);
    JoltPhysicsBodyReaderAdapter reader(world, adapter.bindTable());
    const BodyId slot1Capsule = JoltPhysicsBodyAdapter::bodyIdOf(world.slotBodyId(1, kCapsule));
    const BodyId slot3Capsule = JoltPhysicsBodyAdapter::bodyIdOf(world.slotBodyId(3, kCapsule));

    world.applyOccupancy(occupancyOf({ 3 }));
    CHECK_FALSE(adapter.isBodyResolvable(slot3Capsule));
    CHECK_FALSE(reader.isBodyResolvable(slot3Capsule));

    JoltPhysicsFactory factory(adapter, brawlerDeclarations(), 1, kSimulatableId);
    for (const SlotBodyTemplate& slotTemplate : brawlerDeclarations())
    {
        (void)factory.createPhysicalObject(slotTemplate.descriptor, "declaration");
    }
    CHECK_FALSE(world.appliedOccupancy().occupied.test(1));
    CHECK(adapter.isBodyResolvable(slot1Capsule));
    CHECK(reader.isBodyResolvable(slot1Capsule));

    adapter.bindTable().releaseSlot(1);
    CHECK_FALSE(adapter.isBodyResolvable(slot1Capsule));
    CHECK_FALSE(reader.isBodyResolvable(slot1Capsule));
    CHECK(reader.captureBodyState(slot1Capsule).position == glm::vec3(0.f));
    CHECK(reader.getBodyMass(slot1Capsule) == 0.f);
    CHECK(adapter.getBodyMass(slot1Capsule) > 0.f);
    CHECK(world.bodies().GetUserData(world.slotBodyId(1, kCapsule)) == joltBodyUserData::encode(JoltBodyBinding{ kSimulatableId, 5 }));

    CHECK_FALSE(adapter.isBodyResolvable(BodyId{ 0 }));
    CHECK_FALSE(adapter.isBodyResolvable(BodyId{ 1u + world.simulatableSlots() * world.bodiesPerSlot() }));
}

TEST_CASE("JoltBodyAdapter.BoundBodyConfigurationSurvivesARestore", "[Jolt]")
{
    RuntimeLease lease;
    BoundBrawler rig(lease.runtime, 8);
    const BodyId radial = rig.bodies[kRadial];
    const float massBefore = rig.adapter.getBodyMass(radial);
    const glm::vec3 inertiaBefore = rig.adapter.getBodyInertiaTensor(radial);

    rig.adapter.setBodyAngularVelocity(radial, glm::vec3(0.f, 0.f, 4.f));
    rig.world.step(kDt);
    rig.world.saveTick(1);
    rig.adapter.addBodyTorque(radial, glm::vec3(0.f, 0.f, 1.0e6f));
    rig.world.step(kDt);
    rig.world.saveTick(2);
    REQUIRE(rig.world.restoreTick(1));

    CHECK(rig.adapter.getBodyMass(radial) == massBefore);
    CHECK(rig.adapter.getBodyInertiaTensor(radial) == inertiaBefore);
    CHECK(rig.adapter.captureBodyState(radial).angularVelocity.z == Catch::Approx(4.f));
    CHECK(rig.adapter.isBodyResolvable(radial));
    CHECK(rig.world.bodies().GetFriction(JoltPhysicsBodyAdapter::joltBodyIdOf(radial)) == joltBodyDefaults::kCreatedBodyMaterial.friction);
}

TEST_CASE("JoltBodyAdapter.TheRadialSpunByItsAuthoredTorqueFollowsTheAnalyticOmega", "[Jolt]")
{
    RuntimeLease lease;
    constexpr uint32_t kTicks = 60;

    BoundBrawler parity(lease.runtime);
    const SpinRun run = spinRadialWithAuthoredTorque(parity, kTicks);
    INFO("max |omega - analytic| = " << run.maxAbsoluteError << " rad/s, peak omega = " << run.peakOmega);
    CHECK(run.maxAbsoluteError <= 1e-5);
    CHECK(run.peakOmega == Catch::Approx(15.7079632679).epsilon(1e-5));
    CHECK(run.peakOmega < double(joltBodyDefaults::kMaxAngularVelocityRadiansPerSecond));
    CHECK(std::abs(run.finalOmega) <= 1e-5);
    CHECK(run.maxOffAxisOmega <= 1e-5);

    BoundBrawler joltDefaultDamping(lease.runtime);
    overrideAngularDamping(joltDefaultDamping.world, joltDefaultDamping.bodies[kRadial], 0.05f);
    const SpinRun drifting = spinRadialWithAuthoredTorque(joltDefaultDamping, kTicks);
    INFO("with Jolt's default angular damping 0.05: max error = " << drifting.maxAbsoluteError);
    CHECK(drifting.maxAbsoluteError > 1e-3);
}

#endif // WITH_LOW_LEVEL_TESTS
