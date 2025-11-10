// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>
#include "Joint.h"


using namespace pxr;

namespace omni
{
namespace physics
{
namespace schema
{

static const float sentinelLimit = 0.5e38f;

static JointDesc* createJointDesc(const UsdPrim& usdPrim, const TfTokenVector& customTokens, uint64_t typeFlags)
{
    if (typeFlags & PrimType::eUsdPhysicsRevoluteJoint)
    {
        return new RevoluteJointDesc();
    }
    else if (typeFlags & PrimType::eUsdPhysicsPrismaticJoint)
    {
        return new PrismaticJointDesc();
    }
    else if (typeFlags & PrimType::eUsdPhysicsSphericalJoint)
    {
        return new SphericalJointDesc();
    }
    else if (typeFlags & PrimType::eUsdPhysicsDistanceJoint)
    {
        return new DistanceJointDesc();
    }
    else if (typeFlags & PrimType::eUsdPhysicsFixedJoint)
    {
        return new FixedJointDesc();
    }
    else
    {
        const TfToken& primType = usdPrim.GetTypeName();
        for (size_t i = 0; i < customTokens.size(); i++)
        {
            if (primType == customTokens[i])
            {
                return new CustomJointDesc();                
            }
        }
    }

    return new D6JointDesc();
}

static void parseLimitAPI(JointLimit* dst, const UsdPhysicsLimitAPI& limitAPI)
{
    limitAPI.GetLowAttr().Get(&dst->lower);
    limitAPI.GetHighAttr().Get(&dst->upper);
    if ((isfinite(dst->lower) && dst->lower > -sentinelLimit) || (isfinite(dst->upper) && dst->upper < sentinelLimit))
        dst->enabled = true;
}

static void parseDriveAPI(JointDrive* dst, const UsdPhysicsDriveAPI& driveAPI)
{
    driveAPI.GetTargetPositionAttr().Get(&dst->targetPosition);
    driveAPI.GetTargetVelocityAttr().Get(&dst->targetVelocity);
    driveAPI.GetMaxForceAttr().Get(&dst->forceLimit);    

    driveAPI.GetDampingAttr().Get(&dst->damping);
    driveAPI.GetStiffnessAttr().Get(&dst->stiffness);    

    TfToken typeToken;
    driveAPI.GetTypeAttr().Get(&typeToken);
    if (typeToken == UsdPhysicsTokens->acceleration)
        dst->acceleration = true;
    dst->enabled = true;
}

static void parseAngularDrive(JointDrive* dst, const UsdStageWeakPtr stage, const UsdPrim& usdPrim)
{
    CARB_ASSERT(dst != nullptr);

    dst->enabled = false;

    if (usdPrim.HasAPI<UsdPhysicsDriveAPI>(UsdPhysicsTokens->angular))
    {
        const UsdPhysicsDriveAPI driveAPI = UsdPhysicsDriveAPI::Get(usdPrim, UsdPhysicsTokens->angular);

        parseDriveAPI(dst, driveAPI);
    }  
}


static void parseLinearDrive(JointDrive* dst, const UsdStageWeakPtr stage, const UsdPrim& usdPrim)
{
    CARB_ASSERT(dst != nullptr);

    dst->enabled = false;

    if (usdPrim.HasAPI<UsdPhysicsDriveAPI>(UsdPhysicsTokens->linear))
    {
        const UsdPhysicsDriveAPI driveAPI = UsdPhysicsDriveAPI::Get(usdPrim, UsdPhysicsTokens->linear);

        parseDriveAPI(dst, driveAPI);
    }
}


static SdfPath GetRel(const pxr::UsdRelationship& ref, const pxr::UsdPrim& jointPrim)
{
    pxr::SdfPathVector targets;
    ref.GetTargets(&targets);

    if (targets.size() == 0)
    {
        return SdfPath();
    }
    if (targets.size() > 1)
    {
        CARB_LOG_WARN("Joint prim does have relationship to multiple bodies, jointPrim %s", jointPrim.GetPrimPath().GetText());
        return targets.at(0);
    }

    return targets.at(0);
}

bool checkJointRel(const UsdStageWeakPtr stage, const SdfPath& relPath, const UsdPrim& jointPrim)
{
    if (relPath == SdfPath())
        return true;

    const UsdPrim relPrim = stage->GetPrimAtPath(relPath);
    if (!relPrim)
    {
        CARB_LOG_ERROR("Joint (%s) body relationship %s points to a non existent prim, joint will not be created.", jointPrim.GetPrimPath().GetText(), relPath.GetText());
        return false;
    }
    return true;
}


JointDesc* parseJoint(const UsdStageWeakPtr stage, const UsdPrim& usdPrim, const TfTokenVector& customTokens, uint64_t typeFlags)
{
    JointDesc* desc = nullptr;

    CARB_ASSERT(usdPrim.IsA<UsdPhysicsJoint>());
    UsdPhysicsJoint jointPrim(usdPrim);
    if (jointPrim)
    {                
        JointDesc* jointBaseDesc = createJointDesc(usdPrim, customTokens, typeFlags);
        desc = jointBaseDesc;        

        // parse the joint common parameters
        {
            jointPrim.GetJointEnabledAttr().Get(&jointBaseDesc->jointEnabled);
            jointPrim.GetCollisionEnabledAttr().Get(&jointBaseDesc->collisionEnabled);
            jointPrim.GetBreakForceAttr().Get(&jointBaseDesc->breakForce);            
            jointPrim.GetBreakTorqueAttr().Get(&jointBaseDesc->breakTorque);
            jointPrim.GetExcludeFromArticulationAttr().Get(&jointBaseDesc->excludeFromArticulation);

            jointBaseDesc->rel0 = GetRel(jointPrim.GetBody0Rel(), usdPrim);
            jointBaseDesc->rel1 = GetRel(jointPrim.GetBody1Rel(), usdPrim);
        }

        // check rel validity
        {
            if (!checkJointRel(stage, jointBaseDesc->rel0, usdPrim) || !checkJointRel(stage, jointBaseDesc->rel1, usdPrim))
            {
                delete jointBaseDesc;
                return nullptr;
            }
        }

        // revolute/hinge joint type, all degree of freedom locked except for one rotation axis
        if (jointBaseDesc->type == ObjectType::eJointRevolute)
        {
            RevoluteJointDesc* revoluteDesc = (RevoluteJointDesc*)jointBaseDesc;

            const UsdPhysicsRevoluteJoint revoluteJoint(usdPrim);

            Axis::Enum jointAxis = Axis::eX;
            TfToken axis = UsdPhysicsTokens->x;
            revoluteJoint.GetAxisAttr().Get(&axis);

            if (axis == UsdPhysicsTokens->y)
                jointAxis = Axis::eY;
            else if (axis == UsdPhysicsTokens->z)
                jointAxis = Axis::eZ;
            revoluteDesc->axis = jointAxis;

            revoluteDesc->limit.enabled = false;
            // A.B replace by sentinel values
            revoluteJoint.GetLowerLimitAttr().Get(&revoluteDesc->limit.lower);
            revoluteJoint.GetUpperLimitAttr().Get(&revoluteDesc->limit.upper);
            if (isfinite(revoluteDesc->limit.lower) && isfinite(revoluteDesc->limit.upper)
                && revoluteDesc->limit.lower > -sentinelLimit && revoluteDesc->limit.upper < sentinelLimit)
            {
                revoluteDesc->limit.enabled = true;
            }

            parseAngularDrive(&revoluteDesc->drive, stage, usdPrim);
        }
        // prismatic joint - all degree of freedom locked except for one linear degree of freedom
        else if (jointBaseDesc->type == ObjectType::eJointPrismatic)
        {
            PrismaticJointDesc* prismaticDesc = (PrismaticJointDesc*)jointBaseDesc;

            const UsdPhysicsPrismaticJoint prismaticJoint(usdPrim);

            Axis::Enum jointAxis = Axis::eX;
            TfToken axis = UsdPhysicsTokens->x;            
            prismaticJoint.GetAxisAttr().Get(&axis);

            if (axis == UsdPhysicsTokens->y)
                jointAxis = Axis::eY;
            else if (axis == UsdPhysicsTokens->z)
                jointAxis = Axis::eZ;
            prismaticDesc->axis = jointAxis;

            prismaticDesc->limit.enabled = false;
            prismaticJoint.GetLowerLimitAttr().Get(&prismaticDesc->limit.lower);
            prismaticJoint.GetUpperLimitAttr().Get(&prismaticDesc->limit.upper);
            if ((isfinite(prismaticDesc->limit.lower) && (prismaticDesc->limit.lower > -sentinelLimit)) || 
                (isfinite(prismaticDesc->limit.upper) && (prismaticDesc->limit.upper < sentinelLimit)))
            {
                prismaticDesc->limit.enabled = true;
            }

            parseLinearDrive(&prismaticDesc->drive, stage, usdPrim);
        }
        // spherical joint/ball and socket - liner degree of freedom locked, all angular degree's of freedom opened
        // cone limit can be applied
        else if (jointBaseDesc->type == ObjectType::eJointSpherical)
        {
            SphericalJointDesc* sphericalDesc = (SphericalJointDesc*)jointBaseDesc;

            const UsdPhysicsSphericalJoint sphericalJoint(usdPrim);

            Axis::Enum jointAxis = Axis::eX;
            TfToken axis = UsdPhysicsTokens->x;
            sphericalJoint.GetAxisAttr().Get(&axis);

            if (axis == UsdPhysicsTokens->y)
                jointAxis = Axis::eY;
            else if (axis == UsdPhysicsTokens->z)
                jointAxis = Axis::eZ;
            sphericalDesc->axis = jointAxis;

            sphericalDesc->limit.enabled = false;
            sphericalJoint.GetConeAngle0LimitAttr().Get(&sphericalDesc->limit.angle0);
            sphericalJoint.GetConeAngle1LimitAttr().Get(&sphericalDesc->limit.angle1);

            if (isfinite(sphericalDesc->limit.angle0) && isfinite(sphericalDesc->limit.angle1)
                && sphericalDesc->limit.angle0 >= 0.0 && sphericalDesc->limit.angle1 >= 0.0)
            {
                sphericalDesc->limit.enabled = true;
            }
        }
        // distance joint - prevents movement between bodies based on its distance. Can have
        // either min distance or max distance defined or both
        else if (jointBaseDesc->type == ObjectType::eJointDistance)
        {
            DistanceJointDesc* distanceDesc = (DistanceJointDesc*)jointBaseDesc;

            const UsdPhysicsDistanceJoint distanceJoint(usdPrim);

            distanceDesc->maxEnabled = false;
            distanceDesc->minEnabled = false;
            distanceJoint.GetMinDistanceAttr().Get(&distanceDesc->limit.minDist);
            distanceJoint.GetMaxDistanceAttr().Get(&distanceDesc->limit.maxDist);
            
            if (distanceDesc->limit.minDist >= 0.0f)
            {
                distanceDesc->minEnabled = true;
                
            }
            if (distanceDesc->limit.maxDist >= 0.0f)
            {
                distanceDesc->maxEnabled = true;
                
            }
        }
        else if ((jointBaseDesc->type != ObjectType::eJointFixed) && (jointBaseDesc->type != ObjectType::eJointCustom))
        {
            // D6 joint
            D6JointDesc* d6Joint = (D6JointDesc*)jointBaseDesc;

            std::vector<std::pair<JointAxis::Enum, TfToken>> axisVector = {
                std::make_pair(JointAxis::eDistance, UsdPhysicsTokens->distance), std::make_pair(JointAxis::eTransX, UsdPhysicsTokens->transX),
                std::make_pair(JointAxis::eTransY, UsdPhysicsTokens->transY),     std::make_pair(JointAxis::eTransZ, UsdPhysicsTokens->transZ),
                std::make_pair(JointAxis::eRotX, UsdPhysicsTokens->rotX),         std::make_pair(JointAxis::eRotY, UsdPhysicsTokens->rotY),
                std::make_pair(JointAxis::eRotZ, UsdPhysicsTokens->rotZ)
            };

            for (size_t i = 0; i < axisVector.size(); i++)
            {
                const TfToken& axisToken = axisVector[i].second;

                const UsdPhysicsLimitAPI limitAPI = UsdPhysicsLimitAPI::Get(usdPrim, axisToken);
                if (limitAPI)
                {
                    JointLimit limit;
                    parseLimitAPI(&limit, limitAPI);
                    d6Joint->jointLimits.push_back(std::make_pair(axisVector[i].first, limit));
                }

                const UsdPhysicsDriveAPI driveAPI = UsdPhysicsDriveAPI::Get(usdPrim, axisToken);
                if (driveAPI)
                {
                    JointDrive drive;
                    parseDriveAPI(&drive, driveAPI);
                    d6Joint->jointDrives.push_back(std::make_pair(axisVector[i].first, drive));
                }
            }
        }
    }
    else
    {
        CARB_LOG_WARN("Usd Physics: primitive %s unknown joint\n", usdPrim.GetTypeName().GetText());
    }

    return desc;
}

pxr::UsdPrim getBodyPrim(const pxr::UsdStageWeakPtr stage, const pxr::SdfPath& relPath, pxr::UsdPrim& relPrim)
{
    UsdPrim parent = stage->GetPrimAtPath(relPath);
    relPrim = parent;
    UsdPrim collisionPrim = UsdPrim();
    while (parent && parent != stage->GetPseudoRoot())
    {        
        if (parent.HasAPI<UsdPhysicsRigidBodyAPI>())
        {
            return parent;
        }
        if (parent.HasAPI<UsdPhysicsCollisionAPI>())
        {
            collisionPrim = parent;
        }
        parent = parent.GetParent();
    }

    return collisionPrim;
}

SdfPath getLocalPose(const pxr::UsdStageWeakPtr stage,
    pxr::UsdGeomXformCache& xfCache, const SdfPath& relPath, GfVec3f& t, GfQuatf& q)
{
    UsdPrim relPrim;    
    UsdPrim body = getBodyPrim(stage, relPath, relPrim);    

    // get scale and apply it into localPositions vectors
    GfMatrix4d worldRel = xfCache.GetLocalToWorldTransform(relPrim);

    // we need to apply scale to the localPose, the scale comes from the rigid body
    GfVec3f sc;
    // if we had a rel not to rigid body, we need to recompute the localPose
    if (relPrim != body)
    {
        GfMatrix4d localAnchor;
        localAnchor.SetIdentity();
        localAnchor.SetTranslate(GfVec3d(t));
        localAnchor.SetRotateOnly(GfQuatd(q));

        GfMatrix4d bodyMat;
        if (body)
            bodyMat = xfCache.GetLocalToWorldTransform(body);
        else
            bodyMat.SetIdentity();
        
        const GfMatrix4d worldAnchor = localAnchor * worldRel;
        GfMatrix4d bodyLocalAnchor = worldAnchor * bodyMat.GetInverse();
        bodyLocalAnchor = bodyLocalAnchor.RemoveScaleShear();

        t = GfVec3f(bodyLocalAnchor.ExtractTranslation());
        q = GfQuatf(bodyLocalAnchor.ExtractRotationQuat());
        q.Normalize();

        const GfTransform tr(bodyMat);
        sc = GfVec3f(tr.GetScale());
    }
    else
    {
        const GfTransform tr(worldRel);
        sc = GfVec3f(tr.GetScale());
    }

    // apply the scale, this is not obvious, but in physics there is no scale, so we need to
    // apply it before its send to physics
    for (int i = 0; i < 3; i++)
    {
        t[i] *= sc[i];
    }

    return body ? body.GetPrimPath() : SdfPath();
}

void finalizeJoint(const pxr::UsdStageWeakPtr stage, JointDesc* jointDesc,
    pxr::UsdGeomXformCache& xfCache)
{
    // joint bodies anchor point local transforms
    pxr::UsdPhysicsJoint jointPrim(jointDesc->usdPrim);
    GfVec3f t0(0.f);
    GfVec3f t1(0.f);
    GfQuatf q0(1.f);
    GfQuatf q1(1.f);
    jointPrim.GetLocalPos0Attr().Get(&t0);
    jointPrim.GetLocalRot0Attr().Get(&q0);
    jointPrim.GetLocalPos1Attr().Get(&t1);
    jointPrim.GetLocalRot1Attr().Get(&q1);

    q0.Normalize();
    q1.Normalize();

    // get scale and apply it into localPositions vectors
    if (jointDesc->rel0 != SdfPath())
    {
        jointDesc->body0 = getLocalPose(stage, xfCache, jointDesc->rel0, t0, q0);
    }

    if (jointDesc->rel1 != SdfPath())
    {
        jointDesc->body1 = getLocalPose(stage, xfCache, jointDesc->rel1, t1, q1);
    }

    jointDesc->localPose0Position = t0;
    jointDesc->localPose0Orientation = q0;
    jointDesc->localPose1Position = t1;
    jointDesc->localPose1Orientation = q1;
}

} // namespace schema
} // namespace physics
} // namespace omni
