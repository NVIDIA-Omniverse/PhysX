// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-MIMIC-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

// Per-joint mimic readers (PhysxMimicJointAPI multi-apply +
// NewtonMimicAPI single-apply). All attribute / relationship / schema
// lookups go through IPhysicsSource. Joint-kind classification
// (revolute / prismatic / D6 / unsupported) is walker-resolved; the
// joint kind of the *reference* joint is resolved through that lookup, with a
// source type-name fallback for partial resyncs where the reference joint is
// outside the scanned subtree.

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <cmath>
#include <cfloat>
#include <string>

namespace omni::physics::parse
{

namespace
{

// Sentinel used by the joint schema as "no finite limit set" — values
// at or above this threshold are treated as the unset case.
constexpr float kFiniteLimitSentinel = 0.5e38f;

// Read a float scalar from `key.attrName`. Returns the unchanged `out`
// when the attribute is unauthored / wrong-kind.
void readFloat(IPhysicsSource& src, ObjectKey key, const std::string& attrName, float& out)
{
    src.getAttribute(key, src.internToken(attrName), out);
}

bool readBool(IPhysicsSource& src, ObjectKey key, const std::string& attrName, bool defaultValue)
{
    bool v = defaultValue;
    src.getAttribute(key, src.internToken(attrName), v);
    return v;
}

ObjectType jointTypeFromSource(IPhysicsSource& src, ObjectKey key)
{
    const std::string_view typeName = src.tokenToString(src.getTypeName(key));
    if (typeName == "PhysicsRevoluteJoint")
        return eJointRevolute;
    if (typeName == "PhysicsPrismaticJoint")
        return eJointPrismatic;
    if (typeName == "PhysicsSphericalJoint")
        return eJointSpherical;
    if (typeName == "PhysicsDistanceJoint")
        return eJointDistance;
    if (typeName == "PhysicsFixedJoint")
        return eJointFixed;
    if (typeName == "PhysicsJoint")
        return eJointD6;

    return eUndefined;
}

ObjectType resolveJointType(IPhysicsSource& src, const JointTypeLookup& jointTypeOf, ObjectKey key)
{
    const ObjectType lookupType = jointTypeOf ? jointTypeOf(key) : eUndefined;
    return lookupType != eUndefined ? lookupType : jointTypeFromSource(src, key);
}

// Check whether a revolute joint has a finite limit set — mirrors
// usdLoad/MimicJoint.cpp::revoluteHasLimitSet. Reads `physics:lowerLimit`
// and `physics:upperLimit` (the schema attributes of UsdPhysicsRevoluteJoint).
bool revoluteHasFiniteLimits(IPhysicsSource& src, ObjectKey key)
{
    float lower = kFiniteLimitSentinel;
    float upper = kFiniteLimitSentinel;
    readFloat(src, key, "physics:lowerLimit", lower);
    readFloat(src, key, "physics:upperLimit", upper);
    return std::isfinite(lower) && std::isfinite(upper) &&
           (lower > -kFiniteLimitSentinel) && (upper < kFiniteLimitSentinel);
}

// Per-axis lock state for a generic (D6) joint, taken from the scanned
// JointLimitInfo (lower > upper is the USD "locked" convention). The lookup
// returns nullptr for an axis with no limit authored, which reads as not
// locked. Using the parsed limit avoids the raw UsdPhysicsLimitAPI attribute
// reads the source does not surface for an arbitrary reference joint.
bool isAxisLocked(const JointLimitLookup& jointLimitOf, ObjectKey key, JointAxis axis)
{
    const JointLimitInfo* limit = jointLimitOf ? jointLimitOf(key, axis) : nullptr;
    return limit && limit->lower > limit->upper;
}

bool isAxisNotLocked(const JointLimitLookup& jointLimitOf, ObjectKey key, JointAxis axis)
{
    const JointLimitInfo* limit = jointLimitOf ? jointLimitOf(key, axis) : nullptr;
    return !limit || limit->lower < limit->upper;
}

// JointAxis enum mirror for the D6 path — three rotational axes.
constexpr int kAxisRotX = 0; // matches JointAxis::eRotX
constexpr int kAxisRotY = 1;
constexpr int kAxisRotZ = 2;

// Validate a joint's degree-of-freedom for mimic. `jointType` is the
// joint kind for `jointKey`; `targetAxis` is the mimic-API axis
// instance ("rotX" / "rotY" / "rotZ"). On success, writes the resolved
// internal axis (`MimicJointDesc::eDEFAULT_AXIS` for revolute /
// prismatic; the rotational axis for D6) into `outAxis`.
bool checkDegreeOfFreedom(IPhysicsSource& src, const JointLimitLookup& jointLimitOf,
                          ObjectKey key, ObjectType jointType,
                          const std::string& targetAxis, std::string_view ownerPath,
                          int& outAxis)
{
    if (jointType == eJointRevolute)
    {
        outAxis = MimicJointDesc::eDEFAULT_AXIS;
        if (revoluteHasFiniteLimits(src, key))
            return true;
        CARB_LOG_ERROR(
            "Usd Physics: the revolute joint at prim %s needs a finite limit set to be used by the mimic joint feature.",
            std::string(ownerPath).c_str());
        return false;
    }
    if (jointType == eJointPrismatic)
    {
        outAxis = MimicJointDesc::eDEFAULT_AXIS;
        return true;
    }
    // Reject joint kinds without a free rotational DOF. Gear and rack-and-pinion
    // joints (eJointGear / eJointRackAndPinion) are rejected here intentionally --
    // neither exposes the single free rotational DOF the mimic-joint feature requires.
    if (jointType == eJointSpherical || jointType == eJointDistance ||
        jointType == eJointFixed || jointType == eJointGear ||
        jointType == eJointRackAndPinion)
    {
        CARB_LOG_ERROR(
            "Usd Physics: the joint at prim %s has a type that is not supported by the mimic joint feature. "
            "Please refer to the USD documentation for a list of supported joint types.",
            std::string(ownerPath).c_str());
        return false;
    }

    // D6 / Custom — all translation axes must be locked, and the target
    // rotational axis must not be locked.
    const bool transXLocked = isAxisLocked(jointLimitOf, key, eTransX);
    const bool transYLocked = isAxisLocked(jointLimitOf, key, eTransY);
    const bool transZLocked = isAxisLocked(jointLimitOf, key, eTransZ);

    if (!(transXLocked && transYLocked && transZLocked))
    {
        CARB_LOG_ERROR(
            "Usd Physics: a generic joint needs all linear degrees of freedom locked to support the mimic joint feature. "
            "The joint at prim %s does not meet the requirement.",
            std::string(ownerPath).c_str());
        return false;
    }

    if (targetAxis != "rotX" && targetAxis != "rotY" && targetAxis != "rotZ")
    {
        CARB_LOG_ERROR(
            "Usd Physics: mimic joint axis tokens have to be one of {rotX, rotY, rotZ}. Provided token: \"%s\".",
            targetAxis.c_str());
        return false;
    }

    const JointAxis targetJointAxis =
        (targetAxis == "rotX") ? eRotX : ((targetAxis == "rotY") ? eRotY : eRotZ);
    if (!isAxisNotLocked(jointLimitOf, key, targetJointAxis))
    {
        CARB_LOG_ERROR(
            "Usd Physics: the mimic joint feature needs a non locked degree of freedom. The joint "
            "at prim %s has the axis \"%s\" locked.",
            std::string(ownerPath).c_str(), targetAxis.c_str());
        return false;
    }

    if (targetAxis == "rotX")      outAxis = kAxisRotX;
    else if (targetAxis == "rotY") outAxis = kAxisRotY;
    else                           outAxis = kAxisRotZ;
    return true;
}

// Read a relationship's single target. Logs an error and returns
// invalid when the relationship has 0 or more than 1 target.
ObjectKey readSingleRelTarget(IPhysicsSource& src, ObjectKey key, const std::string& relName,
                              std::string_view ownerPath, const char* errMsg)
{
    std::vector<ObjectKey> targets;
    src.getRelationshipTargets(key, src.internToken(relName), targets);
    if (targets.size() != 1)
    {
        CARB_LOG_ERROR("%s", errMsg);
        (void)ownerPath;
        return ObjectKey{};
    }
    return targets[0];
}

[[maybe_unused]] ObjectType mimicJointAxisToType(const std::string& axis)
{
    if (axis == "rotX") return eMimicJointRotX;
    if (axis == "rotY") return eMimicJointRotY;
    return eMimicJointRotZ;
}

} // namespace

// @implements REQ-PARSE-MIMIC-001
// @covers AC-1 AC-2
void parseMimicJoints(ParseContext& ctx, ObjectKey jointKey,
                     const MimicJointParseInfo& info,
                     const JointTypeLookup& jointTypeOf,
                     const JointLimitLookup& jointLimitOf,
                     std::vector<DescPtr<MimicJointDesc>>& out)
{
    if (info.excludeFromArticulation)
    {
        // The error is only emitted when an instance is applied;
        // skip silently when no instance is applied (checked below).
    }

    if (!info.jointEnabled)
        return;

    IPhysicsSource& src = ctx.source();
    const std::string ownerPathStr(src.sourceKeyToString(jointKey));

    static const char* const kAxisTokens[]   = { "rotX", "rotY", "rotZ" };
    static const ObjectType kAxisObjectTypes[] = { eMimicJointRotX, eMimicJointRotY, eMimicJointRotZ };

    uint32_t appliedMimicAPICount = 0;
    for (uint32_t i = 0; i < 3; ++i)
    {
        const std::string axisToken = kAxisTokens[i];
        const std::string apiName = std::string("PhysxMimicJointAPI:") + axisToken;
        if (!src.hasSchema(jointKey, src.internToken(apiName)))
            continue;

        if (info.excludeFromArticulation)
        {
            CARB_LOG_ERROR(
                "Usd Physics: PhysxMimicJointAPI is only supported on articulation joints. The joint at %s has "
                "\"excludeFromArticulation\" set to true.",
                ownerPathStr.c_str());
            return;
        }

        DescPtr<MimicJointDesc> desc = allocateDesc<MimicJointDesc>(ctx.descriptorAllocator());
        desc->type             = kAxisObjectTypes[i];
        desc->mimicJointKey   = jointKey;
        desc->gearing          = 0.0f;
        desc->offset           = 0.0f;
        desc->naturalFrequency = 0.0f;
        desc->dampingRatio     = 0.0f;

        const std::string attrBase = std::string("physxMimicJoint:") + axisToken + ":";
        readFloat(src, jointKey, attrBase + "gearing",          desc->gearing);
        readFloat(src, jointKey, attrBase + "offset",           desc->offset);
        readFloat(src, jointKey, attrBase + "naturalFrequency", desc->naturalFrequency);
        readFloat(src, jointKey, attrBase + "dampingRatio",     desc->dampingRatio);

        std::string refJointAxis;
        TokenId refAxisTok;
        if (src.getAttribute(jointKey, src.internToken(attrBase + "referenceJointAxis"), refAxisTok))
            refJointAxis = std::string(src.tokenToString(refAxisTok));

        if (refJointAxis != "rotX" && refJointAxis != "rotY" && refJointAxis != "rotZ")
        {
            CARB_LOG_ERROR(
                "Usd Physics: PhysxMimicJointAPI at %s has unsupported token \"%s\" in attribute \"referenceJointAxis\". "
                "Supported tokens are: {rotX, rotY, rotZ}.",
                ownerPathStr.c_str(), refJointAxis.c_str());
            continue;
        }

        if (!checkDegreeOfFreedom(src, jointLimitOf, jointKey, info.jointType, axisToken, ownerPathStr, desc->mimicJointAxis))
            continue;

        // Legacy: prismatic/revolute joints can only have ONE PhysxMimicJointAPI instance applied.
        // (The first one sets mimicJointAxis = eDEFAULT_AXIS; a second one fails this check.)
        if (desc->mimicJointAxis == MimicJointDesc::eDEFAULT_AXIS && appliedMimicAPICount > 0)
        {
            CARB_LOG_ERROR(
                "Usd Physics: PhysxMimicJointAPI at %s: prismatic and revolute joints can only have one PhysxMimicJointAPI "
                "instance applied.",
                ownerPathStr.c_str());
            continue;
        }

        const std::string refRelName = attrBase + "referenceJoint";
        std::vector<ObjectKey> refTargets;
        src.getRelationshipTargets(jointKey, src.internToken(refRelName), refTargets);
        if (refTargets.size() != 1)
        {
            CARB_LOG_ERROR(
                "Usd Physics: PhysxMimicJointAPI at %s must have exactly 1 \"referenceJoint\" relationship defined.",
                ownerPathStr.c_str());
            continue;
        }
        const ObjectKey refKey = refTargets[0];
        const std::string refPathStr(src.sourceKeyToString(refKey));

        if (!src.exists(refKey))
        {
            CARB_LOG_ERROR(
                "Usd Physics: PhysxMimicJointAPI at %s points to a non existing prim at %s in attribute \"referenceJoint\".",
                ownerPathStr.c_str(), refPathStr.c_str());
            continue;
        }

        const ObjectType refJointType = resolveJointType(src, jointTypeOf, refKey);
        if (refJointType == eUndefined)
        {
            CARB_LOG_ERROR(
                "Usd Physics: PhysxMimicJointAPI at %s points to prim %s that is not a UsdPhysicsJoint (see attribute \"referenceJoint\").",
                ownerPathStr.c_str(), refPathStr.c_str());
            continue;
        }

        const bool refExcluded = readBool(src, refKey, "physics:excludeFromArticulation", false);
        if (refExcluded)
        {
            CARB_LOG_ERROR(
                "Usd Physics: PhysxMimicJointAPI is only supported on articulation joints. The mimic joint at %s points to reference "
                "joint %s which has \"excludeFromArticulation\" set to true.",
                ownerPathStr.c_str(), refPathStr.c_str());
            continue;
        }

        const bool refEnabled = readBool(src, refKey, "physics:jointEnabled", true);
        if (!refEnabled)
            continue; // disabled reference joint — ignore silently

        if (!checkDegreeOfFreedom(src, jointLimitOf, refKey, refJointType, refJointAxis, refPathStr, desc->referenceJointAxis))
            continue;

        if (refKey == jointKey)
        {
            CARB_LOG_ERROR(
                "Usd Physics: PhysxMimicJointAPI at %s has same joint for mimic and reference. The native ovruntime "
                "mimic path does not support using the same joint as both mimic and reference.",
                ownerPathStr.c_str());
            continue;
        }

        desc->referenceJointKey = refKey;
        out.push_back(std::move(desc));
        ++appliedMimicAPICount;
    }
}

namespace
{

// Newton-side: NewtonMimicAPI is supported only on single-DOF joints.
// Returns true for revolute (with finite limit) and prismatic.
bool isNewtonSingleDof(IPhysicsSource& src, ObjectKey key, ObjectType jointType,
                      const char* role, std::string_view ownerPath)
{
    if (jointType == eJointRevolute)
    {
        if (revoluteHasFiniteLimits(src, key))
            return true;
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI %s joint at %s is a revolute joint without a finite limit set. "
            "A finite limit is required.",
            role, std::string(ownerPath).c_str());
        return false;
    }
    if (jointType == eJointPrismatic)
        return true;
    CARB_LOG_ERROR(
        "Usd Physics: NewtonMimicAPI is only supported on single-DOF joints (PhysicsRevoluteJoint with a "
        "finite limit or PhysicsPrismaticJoint). The %s joint at %s is not supported.",
        role, std::string(ownerPath).c_str());
    return false;
}

} // namespace

// @implements REQ-PARSE-MIMIC-001
// @covers AC-3
void parseNewtonMimicJoints(ParseContext& ctx, ObjectKey jointKey,
                            const MimicJointParseInfo& info,
                            const JointTypeLookup& jointTypeOf,
                            std::vector<DescPtr<MimicJointDesc>>& out)
{
    IPhysicsSource& src = ctx.source();

    if (!src.hasSchema(jointKey, src.internToken("NewtonMimicAPI")))
        return;

    const std::string ownerPathStr(src.sourceKeyToString(jointKey));

    if (info.excludeFromArticulation)
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI is only supported on articulation joints. The joint at %s has "
            "\"excludeFromArticulation\" set to true.",
            ownerPathStr.c_str());
        return;
    }
    if (!info.jointEnabled)
        return;

    if (!readBool(src, jointKey, "newton:mimicEnabled", true))
        return;

    if (!isNewtonSingleDof(src, jointKey, info.jointType, "follower", ownerPathStr))
        return;

    std::vector<ObjectKey> refTargets;
    src.getRelationshipTargets(jointKey, src.internToken("newton:mimicJoint"), refTargets);
    if (refTargets.size() != 1)
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI at %s must have exactly 1 \"newton:mimicJoint\" relationship defined.",
            ownerPathStr.c_str());
        return;
    }
    const ObjectKey refKey = refTargets[0];
    const std::string refPathStr(src.sourceKeyToString(refKey));

    if (refKey == jointKey)
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI at %s has the same joint for follower and leader, which is not allowed.",
            ownerPathStr.c_str());
        return;
    }

    const ObjectType refJointType = resolveJointType(src, jointTypeOf, refKey);
    if (refJointType == eUndefined)
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI at %s points to prim %s that is not a UsdPhysicsJoint (see \"newton:mimicJoint\").",
            ownerPathStr.c_str(), refPathStr.c_str());
        return;
    }

    const bool refExcluded = readBool(src, refKey, "physics:excludeFromArticulation", false);
    if (refExcluded)
    {
        CARB_LOG_ERROR(
            "Usd Physics: NewtonMimicAPI is only supported on articulation joints. The mimic joint at %s points "
            "to reference joint %s which has \"excludeFromArticulation\" set to true.",
            ownerPathStr.c_str(), refPathStr.c_str());
        return;
    }
    const bool refEnabled = readBool(src, refKey, "physics:jointEnabled", true);
    if (!refEnabled)
        return;

    if (!isNewtonSingleDof(src, refKey, refJointType, "leader", refPathStr))
        return;

    float coef0 = 0.0f;
    float coef1 = 1.0f;
    readFloat(src, jointKey, "newton:mimicCoef0", coef0);
    readFloat(src, jointKey, "newton:mimicCoef1", coef1);

    DescPtr<MimicJointDesc> desc = allocateDesc<MimicJointDesc>(ctx.descriptorAllocator());
    desc->type                 = eNewtonMimicJoint;
    desc->mimicJointKey       = jointKey;
    desc->mimicJointAxis       = MimicJointDesc::eDEFAULT_AXIS;
    desc->referenceJointKey   = refKey;
    desc->referenceJointAxis   = MimicJointDesc::eDEFAULT_AXIS;
    // Newton: joint0 = coef0 + coef1 * joint1
    // PhysX: joint0 + gearing * joint1 + offset = 0
    desc->gearing              = -coef1;
    desc->offset               = -coef0;
    desc->naturalFrequency     = 0.0f;
    desc->dampingRatio         = 0.0f;

    out.push_back(std::move(desc));
}

} // namespace omni::physics::parse
