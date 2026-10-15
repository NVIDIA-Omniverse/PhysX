// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3 AC-5
 */
#pragma once
#include <omni/physics/parse/PreparedJoint.h>
#include <cmath>

namespace omni::physics::ovstage
{
using namespace omni::physics::parse;

struct JointColumnTokens
{
    TokenId enabled, collisionEnabled, breakForce, breakTorque, excludeArticulation;
    TokenId pos0, rot0, pos1, rot1, axis, lower, upper, cone0, cone1, minDistance, maxDistance;
    TokenId axisY, axisZ, acceleration;
    std::vector<TokenId> attributes(ObjectType type = eUndefined) const
    {
        std::vector<TokenId> result{enabled, collisionEnabled, breakForce, breakTorque, excludeArticulation,
                                   pos0, rot0, pos1, rot1};
        if (type == eJointRevolute || type == eJointPrismatic || type == eUndefined)
            result.insert(result.end(), {lower, upper});
        if (type == eJointSpherical || type == eUndefined) result.insert(result.end(), {cone0, cone1});
        if (type == eJointDistance || type == eUndefined) result.insert(result.end(), {minDistance, maxDistance});
        if (type == eJointRevolute || type == eJointPrismatic || type == eJointSpherical || type == eUndefined)
            result.push_back(axis);
        return result;
    }
};

// Membership and vector sizes are fixed before dispatch. Every iteration owns
// its input/output row, so filling an existing D6 entry needs no allocation.
template <typename Reader>
void readOvstageJointAttributes(const Reader& row, const KnownTokens& tok, const JointReadTokens& names,
                               const JointColumnTokens& fields, PreparedJoint& input)
{
    auto& info = input.info;
    const ObjectKey key = input.key;
    row.getAttribute(key, fields.enabled, info.jointEnabled);
    row.getAttribute(key, fields.collisionEnabled, info.collisionEnabled);
    info.breakForce = info.breakTorque = FLT_MAX;
    row.getAttribute(key, fields.breakForce, info.breakForce);
    row.getAttribute(key, fields.breakTorque, info.breakTorque);
    row.getAttribute(key, fields.excludeArticulation, info.excludeFromArticulation);
    row.getAttribute(key, fields.pos0, info.localPose0Position);
    row.getAttribute(key, fields.rot0, info.localPose0Orientation);
    row.getAttribute(key, fields.pos1, info.localPose1Position);
    row.getAttribute(key, fields.rot1, info.localPose1Orientation);
    if (info.type == eJointRevolute || info.type == eJointPrismatic || info.type == eJointSpherical)
    {
        TokenId axis;
        row.getAttribute(key, fields.axis, axis);
        info.axis = axis == fields.axisY ? eY : axis == fields.axisZ ? eZ : eX;
    }
    auto readLimit = [&](TokenId low, TokenId high, JointLimitInfo& limit)
    {
        row.getAttribute(key, low, limit.lower);
        row.getAttribute(key, high, limit.upper);
        constexpr float sentinel = 0.5e38f;
        limit.enabled = (std::isfinite(limit.lower) && limit.lower > -sentinel) ||
                        (std::isfinite(limit.upper) && limit.upper < sentinel);
    };
    auto readDrive = [&](const JointInstanceTokens& tokens, JointDriveInfo& drive)
    {
        row.getAttribute(key, tokens.driveStiffness, drive.stiffness);
        row.getAttribute(key, tokens.driveDamping, drive.damping);
        row.getAttribute(key, tokens.driveTargetPosition, drive.targetPosition);
        row.getAttribute(key, tokens.driveTargetVelocity, drive.targetVelocity);
        row.getAttribute(key, tokens.driveMaxForce, drive.forceLimit);
        TokenId type;
        if (row.getAttribute(key, tokens.driveType, type)) drive.acceleration = type == fields.acceleration;
    };
    if (info.type == eJointRevolute || info.type == eJointPrismatic)
    {
        readLimit(fields.lower, fields.upper, info.limit);
        if (info.drive.enabled) readDrive(names.instances[info.type == eJointRevolute ? JointReadTokens::angular : JointReadTokens::linear], info.drive);
    }
    else if (info.type == eJointSpherical)
    {
        row.getAttribute(key, fields.cone0, info.limit.lower);
        row.getAttribute(key, fields.cone1, info.limit.upper);
        info.limit.enabled = std::isfinite(info.limit.lower) && std::isfinite(info.limit.upper) &&
                             info.limit.lower >= 0.0f && info.limit.upper >= 0.0f;
    }
    else if (info.type == eJointDistance)
    {
        float minimum = -1.0f, maximum = -1.0f;
        info.minEnabled = row.getAttribute(key, fields.minDistance, minimum) && minimum >= 0.0f;
        info.maxEnabled = row.getAttribute(key, fields.maxDistance, maximum) && maximum >= 0.0f;
        info.limit.lower = minimum;
        info.limit.upper = maximum;
    }
    else if (info.type == eJointD6)
    {
        for (auto& limit : info.jointLimits)
        {
            const auto& tokens = names.instances[size_t(limit.first)];
            readLimit(tokens.low, tokens.high, limit.second);
        }
        for (auto& drive : info.jointDrives) readDrive(names.instances[size_t(drive.first)], drive.second);
    }
    readJointAttributes(row, tok, names, input);
}

// Declare only schema families used in this wave, including known-missing
// values. The read declaration derives from the same structural flags as fill.
inline void appendJointAttributeTokens(const KnownTokens& tok, const JointReadTokens& names,
                                      const std::vector<PreparedJoint>& inputs,
                                      std::vector<TokenId>& attributes, std::vector<TokenId>& tokenAttributes)
{
    if (inputs.empty()) return;
    const ObjectType type = inputs.front().info.type;
    const bool hasAxis = type == eJointRevolute || type == eJointPrismatic || type == eJointSpherical || type == eJointD6;
    if (hasAxis) attributes.insert(attributes.end(), {tok.newtonVelocityLimit, tok.newtonArmature,
        tok.newtonFriction, tok.newtonDamping});
    bool spring = false;
    std::array<PreparedJointInstance, 10> used{};
    std::array<bool, 10> limits{}, drives{};
    for (const auto& input : inputs)
    {
        spring |= input.distanceSpring;
        for (size_t i = 0; i < used.size(); ++i)
        {
            used[i].limit |= input.instances[i].limit;
            used[i].axis |= input.instances[i].axis;
            used[i].envelope |= input.instances[i].envelope;
            used[i].state |= input.instances[i].state;
        }
        for (const auto& limit : input.info.jointLimits) limits[size_t(limit.first)] = true;
        for (const auto& drive : input.info.jointDrives) drives[size_t(drive.first)] = true;
        if (input.info.drive.enabled) drives[type == eJointRevolute ? JointReadTokens::angular : JointReadTokens::linear] = true;
    }
    if (spring) attributes.insert(attributes.end(), {tok.physxPhysicsDistanceJointSpringEnabled,
        tok.physxPhysicsDistanceJointSpringStiffness, tok.physxPhysicsDistanceJointSpringDamping});
    if (type == eJointGear || type == eJointRackAndPinion) attributes.push_back(type == eJointGear ? names.gearRatio : names.rackRatio);
    for (size_t i = 0; i < used.size(); ++i)
    {
        const auto& t = names.instances[i];
        if (used[i].limit) attributes.insert(attributes.end(), {t.restitution, t.bounceThreshold, t.stiffness, t.damping});
        if (used[i].axis) attributes.insert(attributes.end(), {t.armature, t.maxJointVelocity,
            t.staticFrictionEffort, t.dynamicFrictionEffort, t.viscousFrictionCoefficient});
        if (used[i].envelope) attributes.insert(attributes.end(), {t.maxActuatorVelocity, t.velocityDependentResistance, t.speedEffortGradient});
        if (used[i].state) attributes.insert(attributes.end(), {t.position, t.velocity});
        if (limits[i]) attributes.insert(attributes.end(), {t.low, t.high});
        if (drives[i])
        {
            attributes.insert(attributes.end(), {t.driveStiffness, t.driveDamping, t.driveTargetPosition,
                t.driveTargetVelocity, t.driveMaxForce, t.driveType});
            tokenAttributes.push_back(t.driveType);
        }
    }
}
} // namespace omni::physics::ovstage
