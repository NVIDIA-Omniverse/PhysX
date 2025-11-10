// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/PrimUtilities.h>
#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <PhysXTools.h>
#include <PhysXCustomJoint.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/foundation/TypeCast.h>
#include "LoadUsd.h"
#include "LoadTools.h"
#include "Joint.h"
#include "PhysicsBody.h"
#include "AttributeHelpers.h"
#include <physicsSchemaTools/physicsSchemaTokens.h>


using namespace pxr;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

static TfToken g_cone("cone");

ObjectId createJoint(AttachedStage& attachedStage, const SdfPath& primPath, PhysxJointDesc* desc, ObjectId body0, bool body0Dynamic,
    ObjectId body1, bool body1Dynamic)
{
    if (desc != nullptr)
    {
        ObjectDb* objectDb = attachedStage.getObjectDatabase();
        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();

        if (desc->jointEnabled && (body0 == kInvalidObjectId && body1 == kInvalidObjectId))
        {
            // scristiano: if in forced parsing single scene mode, the bodies may have not been created
            if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
            {
                REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - no bodies defined at body0 and body1, joint prim: %s", primPath.GetText());
            }
        }
        else
        {
            if (desc->jointEnabled && body0 != kInvalidObjectId && body1 == kInvalidObjectId)
            {
                if (!body0Dynamic)
                {
                    // scristiano: if in forced parsing single scene mode, the bodies may have not been created
                    if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
                    {
                        REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: %s", primPath.GetText());
                    }                    
                    return kInvalidObjectId;
                }
            }
            else if (desc->jointEnabled && body1 != kInvalidObjectId && body0 == kInvalidObjectId)
            {
                if (!body1Dynamic)
                {
                    // scristiano: if in forced parsing single scene mode, the bodies may have not been created
                    if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
                    {
                        REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: %s", primPath.GetText());
                    }                    
                    return kInvalidObjectId;
                }
            }
            else if (desc->jointEnabled && (!body0Dynamic) && (!body1Dynamic))
            {
                REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - cannot create a joint between static bodies, joint prim: %s", primPath.GetText());                
                return kInvalidObjectId;
            }

            if ((body0 != kInvalidObjectId) && (body1 != kInvalidObjectId))
            {
                if (body0 == body1)
                {
                    REPORT_PHYSICS_ERROR("PhysicsUSD: CreateJoint - you cannot create a joint between a body and itself (both joint bodies must be unique) for joint prim: %s", primPath.GetText());                    
                    return kInvalidObjectId;
                }
            }

            if (!desc->validBodyTransformations)
            {
                CARB_LOG_WARN(
                    "PhysicsUSD: CreateJoint - found a joint with disjointed body transforms, the simulation will most likely snap objects together: %s",
                    primPath.GetText());
            }

            const ObjectId id = physInt->createJoint(attachedStage, primPath, *desc, body0, body1);

            if (id != kInvalidObjectId)
            {
                objectDb->findOrCreateEntry(primPath, eJoint, id);             
            }
            return id;
        }        
    }

    return kInvalidObjectId;
}

PhysxJointDesc* createJointDesc(const UsdPrim& usdPrim)
{
    if (usdPrim.IsA<PhysxSchemaPhysxPhysicsGearJoint>())
    {
        return ICE_PLACEMENT_NEW(GearPhysxJointDesc)();
    }
    else if (usdPrim.IsA<PhysxSchemaPhysxPhysicsRackAndPinionJoint>())
    {
        return ICE_PLACEMENT_NEW(RackPhysxJointDesc)();
    }

    return nullptr;
}

bool checkJointBodySimulationOwners(AttachedStage& attachedStage, pxr::UsdStageWeakPtr stage, const omni::physics::schema::JointDesc& inDesc)
{
    SdfPath simOwner0 = getRigidBodySimulationOwner(stage, inDesc.body0);
    SdfPath simOwner1 = getRigidBodySimulationOwner(stage, inDesc.body1);

    if ((inDesc.body0 != SdfPath() && inDesc.body1 != SdfPath()) && (simOwner0 != simOwner1))
    {
        CARB_LOG_ERROR("Cannot create joint between bodies that belong to different owners. Joint: %s", inDesc.usdPrim.GetPrimPath().GetText());
        return false;
    }
    else
    {
        SdfPath owner = simOwner0 != SdfPath() ? simOwner0 : simOwner1;
        if (owner != SdfPath())
        {
            // check if they belong to a scene we do simulate
            const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(owner, eScene);
            if (entry == kInvalidObjectId)
            {                
                return false;
            }
        }
    }

    return true;
}

void setToDefault(PhysxJointDesc& desc)
{
    desc.enableCollision = false;
    desc.jointFriction = 0.0f;
    desc.enableResidualReporting = false;
}

void fillJointDesc(const omni::physics::schema::JointDesc& inDesc, PhysxJointDesc& desc)
{
    setToDefault(desc);

    desc.body0 = inDesc.body0;
    desc.body1 = inDesc.body1;

    desc.rel0 = inDesc.rel0;
    desc.rel1 = inDesc.rel1;

    desc.jointEnabled = inDesc.jointEnabled;
    desc.jointPrimPath = inDesc.usdPrim.GetPrimPath();
    desc.breakForce = inDesc.breakForce;
    desc.breakTorque = inDesc.breakTorque;
    desc.enableCollision = inDesc.collisionEnabled;
    desc.excludedFromArticulation = inDesc.excludeFromArticulation;
    desc.validBodyTransformations = true;

    GfVec3ToFloat3(inDesc.localPose0Position, desc.localPose0Position);
    GfQuatToFloat4(inDesc.localPose0Orientation, desc.localPose0Orientation);
    GfVec3ToFloat3(inDesc.localPose1Position, desc.localPose1Position);
    GfQuatToFloat4(inDesc.localPose1Orientation, desc.localPose1Orientation);    

    // physx bits
    const PhysxSchemaPhysxJointAPI physxJointAPI = PhysxSchemaPhysxJointAPI::Get(inDesc.usdPrim.GetStage(), inDesc.usdPrim.GetPrimPath());
    if (physxJointAPI)
    {        
        getAttribute(desc.jointFriction, physxJointAPI.GetJointFrictionAttr(), 0.0f, FLT_MAX, nullptr);
    }


    // residual reporting
    const PhysxSchemaPhysxResidualReportingAPI residualReportingAPI = PhysxSchemaPhysxResidualReportingAPI::Get(inDesc.usdPrim.GetStage(), inDesc.usdPrim.GetPrimPath());
    if (residualReportingAPI)
    {
        desc.enableResidualReporting = true;
    }
}

void setToDefault(PhysxJointLimit& limit)
{
    limit.restitution = 0.0f;
    limit.bounceThreshold = 0.0f;
    limit.stiffness = 0.0f;
    limit.damping = 0.0f;
}

void setToDefault(PhysxJointAxisProperties& properties)
{
    properties.armature = 0.0f;
    properties.maxJointVelocity = FLT_MAX;
    properties.staticFrictionEffort = 0.0f;
    properties.dynamicFrictionEffort = 0.0f;
    properties.viscousFrictionCoefficient = 0.0f;
}

void setToDefault(PhysxJointDrive& drive)
{
    drive.isEnvelopeUsed = false;
    drive.maxActuatorVelocity = FLT_MAX;
    drive.velocityDependentResistance= 0.0f;
    drive.speedEffortGradient = 0.0f;
}

void fillPhysxLimitDesc(const omni::physics::schema::JointDesc& inDesc, PhysxJointLimit& limit, const pxr::TfToken& axis)
{
    setToDefault(limit);

    // physx bits
    const PhysxSchemaPhysxLimitAPI physxLimitAPI = PhysxSchemaPhysxLimitAPI::Get(inDesc.usdPrim, axis);
    if (physxLimitAPI)
    {
        getAttribute(limit.restitution, physxLimitAPI.GetRestitutionAttr(), 0.0f, FLT_MAX, nullptr);
        getAttribute(limit.bounceThreshold, physxLimitAPI.GetBounceThresholdAttr(), 0.0f, FLT_MAX, nullptr);
        getAttribute(limit.stiffness, physxLimitAPI.GetStiffnessAttr(), 0.0f, FLT_MAX, nullptr);
        getAttribute(limit.damping, physxLimitAPI.GetDampingAttr(), 0.0f, FLT_MAX, nullptr);
    }
}

void fillPhysxJointAxisPropertiesDesc(const omni::physics::schema::JointDesc& inDesc, PhysxJointAxisProperties& properties, const pxr::TfToken& axis)
{
    TfType type = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->PhysxJointAxisAPI);

    setToDefault(properties);
    
    if (inDesc.usdPrim.HasAPI(type, axis))
    {
        if (axis == pxr::UsdPhysicsTokens->linear)
        {
            getAttribute(properties.armature, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->armatureLinear), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.maxJointVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxJointVelocityLinear), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.staticFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->staticFrictionEffortLinear), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.dynamicFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->dynamicFrictionEffortLinear), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.viscousFrictionCoefficient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->viscousFrictionCoefficientLinear), 0.0f, FLT_MAX, nullptr);
        }
        else if (axis == pxr::UsdPhysicsTokens->angular)
        {
            getAttribute(properties.armature, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->armatureAngular), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.maxJointVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxJointVelocityAngular), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.staticFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->staticFrictionEffortAngular), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.dynamicFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->dynamicFrictionEffortAngular), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.viscousFrictionCoefficient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->viscousFrictionCoefficientAngular), 0.0f, FLT_MAX, nullptr);
        }
        else if (axis == pxr::UsdPhysicsTokens->rotX)
        {
            getAttribute(properties.armature, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->armatureRotX), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.maxJointVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxJointVelocityRotX), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.staticFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->staticFrictionEffortRotX), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.dynamicFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->dynamicFrictionEffortRotX), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.viscousFrictionCoefficient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->viscousFrictionCoefficientRotX), 0.0f, FLT_MAX, nullptr);
        }
        else if (axis == pxr::UsdPhysicsTokens->rotY)
        {
            getAttribute(properties.armature, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->armatureRotY), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.maxJointVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxJointVelocityRotY), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.staticFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->staticFrictionEffortRotY), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.dynamicFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->dynamicFrictionEffortRotY), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.viscousFrictionCoefficient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->viscousFrictionCoefficientRotY), 0.0f, FLT_MAX, nullptr);
        }
        else if (axis == pxr::UsdPhysicsTokens->rotZ)
        {
            getAttribute(properties.armature, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->armatureRotZ), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.maxJointVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxJointVelocityRotZ), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.staticFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->staticFrictionEffortRotZ), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.dynamicFrictionEffort, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->dynamicFrictionEffortRotZ), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.viscousFrictionCoefficient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->viscousFrictionCoefficientRotZ), 0.0f, FLT_MAX, nullptr);
        }
    }

    else
    {
        const PhysxSchemaPhysxJointAPI physxJointAPI = PhysxSchemaPhysxJointAPI::Get(inDesc.usdPrim.GetStage(), inDesc.usdPrim.GetPrimPath());
        if (physxJointAPI)
        {
            getAttribute(properties.maxJointVelocity, physxJointAPI.GetMaxJointVelocityAttr(), 0.0f, FLT_MAX, nullptr);
            getAttribute(properties.armature, physxJointAPI.GetArmatureAttr(), 0.0f, FLT_MAX, nullptr);
        
        }
    }
}

void fillPhysxPerformanceEnvelopeDesc(const omni::physics::schema::JointDesc& inDesc, PhysxJointDrive& drive, const pxr::TfToken& axis)
{
    TfType type = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI);
    setToDefault(drive);
    if (inDesc.usdPrim.HasAPI(type, axis))
    {
        drive.isEnvelopeUsed = true;
        bool convertToRad = true;
        if (axis == pxr::UsdPhysicsTokens->linear)
        {
            convertToRad = false;
            getAttribute(drive.maxActuatorVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxActuatorVelocityLinear), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.velocityDependentResistance, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->velocityDependentResistanceLinear), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.speedEffortGradient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->speedEffortGradientLinear), 0.0f, FLT_MAX, nullptr);
        }
        else if (axis == pxr::UsdPhysicsTokens->angular)
        {
            getAttribute(drive.maxActuatorVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxActuatorVelocityAngular), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.velocityDependentResistance, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->velocityDependentResistanceAngular), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.speedEffortGradient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->speedEffortGradientAngular), 0.0f, FLT_MAX, nullptr);
        }
        else if (axis == pxr::UsdPhysicsTokens->rotX)
        {
            getAttribute(drive.maxActuatorVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxActuatorVelocityRotX), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.velocityDependentResistance, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->velocityDependentResistanceRotX), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.speedEffortGradient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->speedEffortGradientRotX), 0.0f, FLT_MAX, nullptr);
        }
        else if (axis == pxr::UsdPhysicsTokens->rotY)
        {
            getAttribute(drive.maxActuatorVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxActuatorVelocityRotY), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.velocityDependentResistance, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->velocityDependentResistanceRotY), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.speedEffortGradient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->speedEffortGradientRotY), 0.0f, FLT_MAX, nullptr);
        }
        else if (axis == pxr::UsdPhysicsTokens->rotZ)
        {
            getAttribute(drive.maxActuatorVelocity, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->maxActuatorVelocityRotZ), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.velocityDependentResistance, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->velocityDependentResistanceRotZ), 0.0f, FLT_MAX, nullptr);
            getAttribute(drive.speedEffortGradient, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->speedEffortGradientRotZ), 0.0f, FLT_MAX, nullptr);
        }


        if (convertToRad)
        {
            drive.maxActuatorVelocity = degToRad(drive.maxActuatorVelocity);
            drive.velocityDependentResistance = radToDeg(drive.velocityDependentResistance); // torque * second / degrees
            drive.speedEffortGradient =  degToRad(drive.speedEffortGradient);
        }
    }
}

void fillPhysxDriveDesc(const JointDrive& driveIn, PhysxJointDrive& driveOut, bool convertToRad)
{
    driveOut.acceleration = driveIn.acceleration;    
    driveOut.enabled = driveIn.enabled;
    driveOut.forceLimit = isfinite(driveIn.forceLimit) ? driveIn.forceLimit : FLT_MAX;
    // stiffness and damping are in 1/deg in usd, so converting via radToDeg will convert correctly,
    // i.e. 1 / (pi / 180) = 180 / pi
    driveOut.stiffness = convertToRad ? radToDeg(driveIn.stiffness) : driveIn.stiffness;
    driveOut.damping = convertToRad ? radToDeg(driveIn.damping) : driveIn.damping;
    driveOut.targetPosition = convertToRad ? degToRad(driveIn.targetPosition) : driveIn.targetPosition;
    driveOut.targetVelocity = convertToRad ? degToRad(driveIn.targetVelocity) : driveIn.targetVelocity;
}

static bool isBodyTmEq(UsdStageWeakPtr stage, const PhysxJointDesc* desc, UsdGeomXformCache& xfCache, bool checkPosition, bool checkRotation, unsigned char axis = 0xff)
{

    return primutils::isBodyTransformEqual( stage,desc->body0, desc->body1, 
                                            toVec3f(desc->localPose0Position), 
                                            toQuatf(desc->localPose0Orientation),
                                            toVec3f(desc->localPose1Position), 
                                            toQuatf(desc->localPose1Orientation), 
                                            xfCache, OmniPhysX::getInstance().getCachedSettings().jointBodyTransformCheckTolerance,
                                            checkPosition, checkRotation, axis);
}

void fillPhysicsJointStateDesc(const omni::physics::schema::JointDesc& inDesc, PhysicsJointState& state, const pxr::TfToken& axis)
{
    // physx bits
    const PhysxSchemaJointStateAPI physicsJointStateAPI = PhysxSchemaJointStateAPI::Get(inDesc.usdPrim, axis);
    if (physicsJointStateAPI)
    {   
        state.enabled = true;
        getAttribute(state.position, physicsJointStateAPI.GetPositionAttr(), -FLT_MAX, FLT_MAX, nullptr);
        getAttribute(state.velocity, physicsJointStateAPI.GetVelocityAttr(), -FLT_MAX, FLT_MAX, nullptr);
    }
}

PhysxJointDesc* parseJoint(pxr::UsdStageWeakPtr stage, const omni::physics::schema::JointDesc& inDesc, pxr::UsdGeomXformCache& xfCache)
{
    switch (inDesc.type)
    {
    case omni::physics::schema::ObjectType::eJointSpherical:
    {
        const SphericalJointDesc& sphericalJointDesc = (const SphericalJointDesc&)inDesc;
        SphericalPhysxJointDesc* desc = ICE_PLACEMENT_NEW(SphericalPhysxJointDesc)();
        fillJointDesc(inDesc, *desc);

        desc->axis = Axis(sphericalJointDesc.axis);
        fillPhysxLimitDesc(inDesc, desc->limit, g_cone);

        desc->limit.enabled = sphericalJointDesc.limit.enabled;
        desc->limit.angle0 = degToRad(sphericalJointDesc.limit.angle0);
        desc->limit.angle1 = degToRad(sphericalJointDesc.limit.angle1);
        desc->validBodyTransformations = isBodyTmEq(stage, desc, xfCache, true, false);

        const std::array<std::pair<JointAxis, TfToken>, 3> axisVector = {
            std::make_pair(JointAxis::eRotX, UsdPhysicsTokens->rotX),         
            std::make_pair(JointAxis::eRotY, UsdPhysicsTokens->rotY),
            std::make_pair(JointAxis::eRotZ, UsdPhysicsTokens->rotZ)
        };

        for (size_t i = 0; i < axisVector.size(); i++)
        {
            const TfToken& axisToken = axisVector[i].second;
            PhysicsJointState state;
            PhysxJointAxisProperties properties;
            fillPhysxJointAxisPropertiesDesc(inDesc, properties, axisToken);
            properties.maxJointVelocity = degToRad( properties.maxJointVelocity);
            properties.viscousFrictionCoefficient = radToDeg( properties.viscousFrictionCoefficient); // torque * second / degrees
            desc->jointProperties.push_back(std::make_pair(axisVector[i].first, properties));
        }
        return desc;
    }
    break;
    case omni::physics::schema::ObjectType::eJointD6:
    {
        const D6JointDesc& d6JointDesc = (const D6JointDesc&)inDesc;
        D6PhysxJointDesc* desc = ICE_PLACEMENT_NEW(D6PhysxJointDesc)();
        fillJointDesc(inDesc, *desc);

        for (size_t i = 0; i < d6JointDesc.jointLimits.size(); i++)
        {
            const JointLimit& limit = d6JointDesc.jointLimits[i].second;
            PhysxJointLimit physxLimit;
            switch(d6JointDesc.jointLimits[i].first)
            {
            case JointAxis::eRotX:
            {
                fillPhysxLimitDesc(inDesc, physxLimit, UsdPhysicsTokens->rotX);

                physxLimit.enabled = limit.enabled;
                physxLimit.lower = degToRad(limit.lower);
                physxLimit.upper = degToRad(limit.upper);

                desc->jointLimits.push_back(std::make_pair(eRotX, physxLimit));
            }
            break;
            case JointAxis::eRotY:
            {
                fillPhysxLimitDesc(inDesc, physxLimit, UsdPhysicsTokens->rotY);

                physxLimit.enabled = limit.enabled;
                physxLimit.lower = degToRad(limit.lower);
                physxLimit.upper = degToRad(limit.upper);

                desc->jointLimits.push_back(std::make_pair(eRotY, physxLimit));
            }
            break;
            case JointAxis::eRotZ:
            {
                fillPhysxLimitDesc(inDesc, physxLimit, UsdPhysicsTokens->rotZ);

                physxLimit.enabled = limit.enabled;
                physxLimit.lower = degToRad(limit.lower);
                physxLimit.upper = degToRad(limit.upper);

                desc->jointLimits.push_back(std::make_pair(eRotZ, physxLimit));
            }
            break;
            case JointAxis::eTransX:
            {
                fillPhysxLimitDesc(inDesc, physxLimit, UsdPhysicsTokens->transX);

                physxLimit.enabled = limit.enabled;
                physxLimit.lower = limit.lower;
                physxLimit.upper = limit.upper;

                desc->jointLimits.push_back(std::make_pair(eTransX, physxLimit));
            }
            break;
            case JointAxis::eTransY:
            {
                fillPhysxLimitDesc(inDesc, physxLimit, UsdPhysicsTokens->transY);

                physxLimit.enabled = limit.enabled;
                physxLimit.lower = limit.lower;
                physxLimit.upper = limit.upper;

                desc->jointLimits.push_back(std::make_pair(eTransY, physxLimit));
            }
            break;
            case JointAxis::eTransZ:
            {
                fillPhysxLimitDesc(inDesc, physxLimit, UsdPhysicsTokens->transZ);

                physxLimit.enabled = limit.enabled;
                physxLimit.lower = limit.lower;
                physxLimit.upper = limit.upper;

                desc->jointLimits.push_back(std::make_pair(eTransZ, physxLimit));
            }
            break;
            case JointAxis::eDistance:
            {
                fillPhysxLimitDesc(inDesc, physxLimit, UsdPhysicsTokens->distance);

                physxLimit.enabled = limit.enabled;
                physxLimit.lower = limit.lower;
                physxLimit.upper = limit.upper;

                desc->jointLimits.push_back(std::make_pair(eDistance, physxLimit));
            }
            break;
            default:
                break;
            }            
        }

        for (size_t i = 0; i < d6JointDesc.jointDrives.size(); i++)
        {
            const JointDrive& limit = d6JointDesc.jointDrives[i].second;
            PhysxJointDrive physxDrive;
            switch (d6JointDesc.jointDrives[i].first)
            {
            case JointAxis::eRotX:
            {
                fillPhysxDriveDesc(d6JointDesc.jointDrives[i].second, physxDrive, true);
                fillPhysxPerformanceEnvelopeDesc(inDesc, physxDrive, UsdPhysicsTokens->rotX);
                desc->jointDrives.push_back(std::make_pair(eRotX, physxDrive));
            }
            break;
            case JointAxis::eRotY:
            {
                fillPhysxDriveDesc(d6JointDesc.jointDrives[i].second, physxDrive, true);
                fillPhysxPerformanceEnvelopeDesc(inDesc, physxDrive, UsdPhysicsTokens->rotY);
                desc->jointDrives.push_back(std::make_pair(eRotY, physxDrive));

            }
            break;
            case JointAxis::eRotZ:
            {
                fillPhysxDriveDesc(d6JointDesc.jointDrives[i].second, physxDrive, true);
                fillPhysxPerformanceEnvelopeDesc(inDesc, physxDrive, UsdPhysicsTokens->rotZ);
                desc->jointDrives.push_back(std::make_pair(eRotZ, physxDrive));

            }
            break;
            case JointAxis::eTransX:
            {
                fillPhysxDriveDesc(d6JointDesc.jointDrives[i].second, physxDrive, false);
                desc->jointDrives.push_back(std::make_pair(eTransX, physxDrive));

            }
            break;
            case JointAxis::eTransY:
            {
                fillPhysxDriveDesc(d6JointDesc.jointDrives[i].second, physxDrive, false);
                desc->jointDrives.push_back(std::make_pair(eTransY, physxDrive));

            }
            break;
            case JointAxis::eTransZ:
            {
                fillPhysxDriveDesc(d6JointDesc.jointDrives[i].second, physxDrive, false);
                desc->jointDrives.push_back(std::make_pair(eTransZ, physxDrive));

            }
            break;
            default:
                break;
            }
        }         

        const std::array<std::pair<JointAxis, TfToken>, 6> axisVector = {
            std::make_pair(JointAxis::eTransX, UsdPhysicsTokens->transX),
            std::make_pair(JointAxis::eTransY, UsdPhysicsTokens->transY),     
            std::make_pair(JointAxis::eTransZ, UsdPhysicsTokens->transZ),
            std::make_pair(JointAxis::eRotX, UsdPhysicsTokens->rotX),         
            std::make_pair(JointAxis::eRotY, UsdPhysicsTokens->rotY),
            std::make_pair(JointAxis::eRotZ, UsdPhysicsTokens->rotZ)
        };

        for (size_t i = 0; i < axisVector.size(); i++)
        {
            const TfToken& axisToken = axisVector[i].second;
            PhysicsJointState state;
            PhysxJointAxisProperties properties;
            const PhysxSchemaJointStateAPI physicsJointStateAPI = PhysxSchemaJointStateAPI::Get(inDesc.usdPrim, axisToken);
            if (physicsJointStateAPI)
            {
                state.enabled = true;
                getAttribute(state.position, physicsJointStateAPI.GetPositionAttr(), -FLT_MAX, FLT_MAX, nullptr);
                getAttribute(state.velocity, physicsJointStateAPI.GetVelocityAttr(), -FLT_MAX, FLT_MAX, nullptr);
                if (i >= 3)
                {
                    //convert to radians eRot*
                    state.position = degToRad(state.position);
                    state.velocity = degToRad(state.velocity);
                }
                desc->jointStates.push_back(std::make_pair(axisVector[i].first, state));
            }

            if (i >= 3)
                {
                    fillPhysxJointAxisPropertiesDesc(inDesc, properties, axisToken);
                    properties.maxJointVelocity = degToRad( properties.maxJointVelocity);
                    properties.viscousFrictionCoefficient = radToDeg( properties.viscousFrictionCoefficient); // torque * second / degrees
                    desc->jointProperties.push_back(std::make_pair(axisVector[i].first, properties));
                }
        }
        return desc;
    }
    break;
    case omni::physics::schema::ObjectType::eJointDistance:
    {
        const DistanceJointDesc& distanceJointDesc = (const DistanceJointDesc&)inDesc;
        DistancePhysxJointDesc* desc = ICE_PLACEMENT_NEW(DistancePhysxJointDesc)();
        fillJointDesc(inDesc, *desc);
        
        fillPhysxLimitDesc(inDesc, desc->limit, UsdPhysicsTokens->distance);

        desc->minEnabled = distanceJointDesc.minEnabled;
        desc->maxEnabled = distanceJointDesc.maxEnabled;
        desc->limit.minDist = distanceJointDesc.limit.minDist;
        desc->limit.maxDist = distanceJointDesc.limit.maxDist;

        const PhysxSchemaPhysxPhysicsDistanceJointAPI distJointAPI = PhysxSchemaPhysxPhysicsDistanceJointAPI::Get(stage, inDesc.usdPrim.GetPrimPath());
        if (distJointAPI)
        {
            getBoolAttribute(desc->springEnabled, distJointAPI.GetSpringEnabledAttr(), nullptr);
            getAttribute(desc->damping, distJointAPI.GetSpringDampingAttr(), 0.0f, FLT_MAX, nullptr);
            getAttribute(desc->stiffness, distJointAPI.GetSpringStiffnessAttr(), 0.0f, FLT_MAX, nullptr);
        }
        return desc;
    }
    break;
    case omni::physics::schema::ObjectType::eJointFixed:
    {        
        FixedPhysxJointDesc* desc = ICE_PLACEMENT_NEW(FixedPhysxJointDesc)();
        fillJointDesc(inDesc, *desc);
        desc->validBodyTransformations = isBodyTmEq(stage, desc, xfCache, true, true);
        return desc;
    }
    break;
    case omni::physics::schema::ObjectType::eJointPrismatic:
    {
        const PrismaticJointDesc& prismaticJointDesc = (const PrismaticJointDesc&)inDesc;
        PrismaticPhysxJointDesc* desc = ICE_PLACEMENT_NEW(PrismaticPhysxJointDesc)();
        fillJointDesc(inDesc, *desc);

        desc->axis = Axis(prismaticJointDesc.axis);

        fillPhysxJointAxisPropertiesDesc(inDesc, desc->properties, UsdPhysicsTokens->linear);
        fillPhysxLimitDesc(inDesc, desc->limit, UsdPhysicsTokens->linear);

        desc->limit.enabled = prismaticJointDesc.limit.enabled;
        desc->limit.lower = prismaticJointDesc.limit.lower;
        desc->limit.upper = prismaticJointDesc.limit.upper;
        desc->validBodyTransformations = isBodyTmEq(stage, desc, xfCache, true, true, desc->axis);

        // drive
        fillPhysxDriveDesc(prismaticJointDesc.drive, desc->drive, false);
        fillPhysxPerformanceEnvelopeDesc(inDesc, desc->drive, UsdPhysicsTokens->linear);

        // joint state
        fillPhysicsJointStateDesc(inDesc, desc->state, UsdPhysicsTokens->linear);
        return desc;
    }
    break;
    case omni::physics::schema::ObjectType::eJointRevolute:
    {
        const RevoluteJointDesc& revoluteJointDesc = (const RevoluteJointDesc&)inDesc;
        RevolutePhysxJointDesc* desc = ICE_PLACEMENT_NEW(RevolutePhysxJointDesc)();
        fillJointDesc(inDesc, *desc);
        desc->axis = Axis(revoluteJointDesc.axis);

        fillPhysxJointAxisPropertiesDesc(inDesc, desc->properties, UsdPhysicsTokens->angular);
        desc->properties.maxJointVelocity = degToRad(desc->properties.maxJointVelocity);
        desc->properties.viscousFrictionCoefficient = radToDeg( desc->properties.viscousFrictionCoefficient); // torque * second / degrees
        fillPhysxLimitDesc(inDesc, desc->limit, UsdPhysicsTokens->angular);

        desc->limit.enabled = revoluteJointDesc.limit.enabled;
        desc->limit.lower = degToRad(revoluteJointDesc.limit.lower);
        desc->limit.upper = degToRad(revoluteJointDesc.limit.upper);

        desc->validBodyTransformations = isBodyTmEq(stage, desc, xfCache, true, false);

        // drive
        fillPhysxDriveDesc(revoluteJointDesc.drive, desc->drive, true);
        fillPhysxPerformanceEnvelopeDesc(inDesc, desc->drive, UsdPhysicsTokens->angular);

        // joint state
        fillPhysicsJointStateDesc(inDesc, desc->state, UsdPhysicsTokens->angular);
        desc->state.position = degToRad(desc->state.position);
        desc->state.velocity = degToRad(desc->state.velocity);
        return desc;
    }
    break;
    case omni::physics::schema::ObjectType::eJointCustom:
    {
        if (inDesc.usdPrim.IsA<PhysxSchemaPhysxPhysicsGearJoint>())
        {            
            GearPhysxJointDesc* gearDesc = ICE_PLACEMENT_NEW(GearPhysxJointDesc)();
            fillJointDesc(inDesc, *gearDesc);
            
            const PhysxSchemaPhysxPhysicsGearJoint gearJoint(inDesc.usdPrim);

            gearJoint.GetGearRatioAttr().Get(&gearDesc->gearRatio);

            if (pxr::UsdRelationship rel = gearJoint.GetHinge0Rel())
            {
                pxr::SdfPathVector paths;
                rel.GetTargets(&paths);
                if (!paths.empty())
                    gearDesc->hingePrimPath0 = paths.front();
            }

            if (pxr::UsdRelationship rel = gearJoint.GetHinge1Rel())
            {
                pxr::SdfPathVector paths;
                rel.GetTargets(&paths);
                if (!paths.empty())
                    gearDesc->hingePrimPath1 = paths.front();
            }
            return gearDesc;
        }
        else if (inDesc.usdPrim.IsA<PhysxSchemaPhysxPhysicsRackAndPinionJoint>())
        {
            RackPhysxJointDesc* rackDesc = ICE_PLACEMENT_NEW(RackPhysxJointDesc)();
            fillJointDesc(inDesc, *rackDesc);

            const PhysxSchemaPhysxPhysicsRackAndPinionJoint rackJoint(inDesc.usdPrim);

            rackJoint.GetRatioAttr().Get(&rackDesc->ratio);

            if (pxr::UsdRelationship rel = rackJoint.GetHingeRel())
            {
                pxr::SdfPathVector paths;
                rel.GetTargets(&paths);
                if (!paths.empty())
                    rackDesc->hingePrimPath = paths.front();
            }

            if (pxr::UsdRelationship rel = rackJoint.GetPrismaticRel())
            {
                pxr::SdfPathVector paths;
                rel.GetTargets(&paths);
                if (!paths.empty())
                    rackDesc->prismaticPrimPath = paths.front();
            }
            return rackDesc;
        }
        else
        {
            const CustomJointTypeMap& customJointMap = OmniPhysX::getInstance().getCustomJointManager().getCustomJointTypeMap();
            CustomJointTypeMap::const_iterator fit = customJointMap.find(inDesc.usdPrim.GetTypeName());
            if (fit != customJointMap.end())
            {
                CustomPhysxJointDesc* customDesc = ICE_PLACEMENT_NEW(CustomPhysxJointDesc)();
                fillJointDesc(inDesc, *customDesc);
                customDesc->customJointToken = inDesc.usdPrim.GetTypeName();
                return customDesc;
            }
        }
    }
    break;
    default:
        break;
    }

    return nullptr;
}


} // namespace usdparser
} // namespace physx
} // namespace omni
