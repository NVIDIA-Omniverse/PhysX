// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3 AC-5
 *
 * @implements REQ-PARSE-NEWTON-JOINT-001
 * @covers AC-2 AC-4
 */
#pragma once
#include "ParseApi.h"
#include "PreparedAttribute.h"
#include "DescriptorBatchAllocator.h"
#include <array>

namespace omni::physics::parse
{

// Interned once per source context; no names are constructed during worker reads.
struct JointInstanceTokens
{
    TokenId limitAPI;
    TokenId axisAPI;
    TokenId envelopeAPI;
    TokenId stateAPI;
    TokenId physicsLimitAPI;
    TokenId physicsDriveAPI;
    TokenId restitution;
    TokenId bounceThreshold;
    TokenId stiffness;
    TokenId damping;
    TokenId armature;
    TokenId maxJointVelocity;
    TokenId staticFrictionEffort;
    TokenId dynamicFrictionEffort;
    TokenId viscousFrictionCoefficient;
    TokenId maxActuatorVelocity;
    TokenId velocityDependentResistance;
    TokenId speedEffortGradient;
    TokenId position;
    TokenId velocity;
    TokenId low;
    TokenId high;
    TokenId driveStiffness;
    TokenId driveDamping;
    TokenId driveTargetPosition;
    TokenId driveTargetVelocity;
    TokenId driveMaxForce;
    TokenId driveType;
};
static_assert(eDistance == 0 && eTransX == 1 && eTransY == 2 && eTransZ == 3 &&
              eRotX == 4 && eRotY == 5 && eRotZ == 6, "Joint attribute table must follow JointAxis");
struct JointReadTokens
{
    static constexpr size_t angular = 7, linear = 8, cone = 9;
    std::array<JointInstanceTokens, 10> instances;
    TokenId gearRatio, rackRatio;
    explicit JointReadTokens(const IPhysicsSource& source);
};

struct PreparedJointInstance
{
    bool limit = false, axis = false, envelope = false, state = false;
    PreparedAttribute<float> restitution, bounceThreshold, stiffness, damping;
    PreparedAttribute<float> armature, maxJointVelocity, staticFrictionEffort,
        dynamicFrictionEffort, viscousFrictionCoefficient;
    PreparedAttribute<float> maxActuatorVelocity, velocityDependentResistance, speedEffortGradient;
    PreparedAttribute<float> position, velocity;
};

struct PreparedJoint
{
    bool prepared = false;
    ObjectKey key;
    JointInfo info;
    // First seven entries match JointAxis, then angular, linear and cone.
    std::array<PreparedJointInstance, 10> instances;
    PreparedAttribute<float> newtonVelocity, newtonArmature, newtonFriction, newtonDamping;
    bool distanceSpring = false;
    PreparedAttribute<bool> springEnabled;
    PreparedAttribute<float> springStiffness, springDamping, ratio;
    ObjectKey reference0, reference1;
};

template <typename Reader>
void readJointInstanceAttributes(const Reader& src, ObjectKey key, const JointInstanceTokens& tok,
                                 PreparedJointInstance& input)
{
    if (input.limit)
    {
        input.restitution = prepareAttribute<float>(src, key, tok.restitution);
        input.bounceThreshold = prepareAttribute<float>(src, key, tok.bounceThreshold);
        input.stiffness = prepareAttribute<float>(src, key, tok.stiffness);
        input.damping = prepareAttribute<float>(src, key, tok.damping);
    }
    if (input.axis)
    {
        input.armature = prepareAttribute<float>(src, key, tok.armature, true);
        input.maxJointVelocity = prepareAttribute<float>(src, key, tok.maxJointVelocity, true);
        input.staticFrictionEffort = prepareAttribute<float>(src, key, tok.staticFrictionEffort, true);
        input.dynamicFrictionEffort = prepareAttribute<float>(src, key, tok.dynamicFrictionEffort, true);
        input.viscousFrictionCoefficient = prepareAttribute<float>(src, key, tok.viscousFrictionCoefficient, true);
    }
    if (input.envelope)
    {
        input.maxActuatorVelocity = prepareAttribute<float>(src, key, tok.maxActuatorVelocity);
        input.velocityDependentResistance = prepareAttribute<float>(src, key, tok.velocityDependentResistance);
        input.speedEffortGradient = prepareAttribute<float>(src, key, tok.speedEffortGradient);
    }
    if (input.state)
    {
        input.position = prepareAttribute<float>(src, key, tok.position);
        input.velocity = prepareAttribute<float>(src, key, tok.velocity);
    }
}

template <typename Reader>
void readJointAttributes(const Reader& src, const KnownTokens& tok, const JointReadTokens& names, PreparedJoint& input)
{
    const ObjectKey key = input.key;
    const auto type = input.info.type;
    const bool hasAxis = type == eJointRevolute || type == eJointPrismatic || type == eJointSpherical || type == eJointD6;
    if (hasAxis)
    {
        input.newtonVelocity = prepareAttribute<float>(src, key, tok.newtonVelocityLimit, true);
        input.newtonArmature = prepareAttribute<float>(src, key, tok.newtonArmature, true);
        input.newtonFriction = prepareAttribute<float>(src, key, tok.newtonFriction, true);
        input.newtonDamping = prepareAttribute<float>(src, key, tok.newtonDamping, true);
    }
    for (size_t i = 0; i < input.instances.size(); ++i)
        readJointInstanceAttributes(src, key, names.instances[i], input.instances[i]);
    if (input.distanceSpring)
    {
        input.springEnabled = prepareAttribute<bool>(src, key, tok.physxPhysicsDistanceJointSpringEnabled);
        input.springStiffness = prepareAttribute<float>(src, key, tok.physxPhysicsDistanceJointSpringStiffness, true);
        input.springDamping = prepareAttribute<float>(src, key, tok.physxPhysicsDistanceJointSpringDamping, true);
    }
    if (type == eJointGear || type == eJointRackAndPinion)
        input.ratio = prepareAttribute<float>(src, key, type == eJointGear ? names.gearRatio : names.rackRatio);
    input.prepared = true;
}

PreparedJoint prepareJointStructure(ParseContext& context, ObjectKey key, const JointInfo& info);

PreparedJoint prepareJoint(ParseContext& context, ObjectKey key, const JointInfo& info);
void prepareJointStorage(const PreparedJoint& input, PhysxJointDesc& output);
bool fillJoint(const PreparedJoint& input, PhysxJointDesc& output) noexcept;
std::vector<DescPtr<PhysxJointDesc>> allocateJointBatch(DescriptorBatchAllocator& allocator,
                                                     ObjectType type, size_t count);

} // namespace omni::physics::parse
