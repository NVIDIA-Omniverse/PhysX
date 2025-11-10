// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/PrimUtilities.h>

#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

#include <physicsSchemaTools/physicsSchemaTokens.h>


#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <PhysXTools.h>
#include <OmniPhysX.h>
#include <ChangeRegister.h>
#include <CookingDataAsync.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "Mass.h"
#include "Material.h"
#include "CollisionGroup.h"
#include "AttributeHelpers.h"


using namespace pxr;
using namespace carb;

namespace
{

UsdAttribute getPosePurposesAttr(UsdPrim posePrim, TfToken instanceName)
{
    TfToken attrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
        OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_purposes, instanceName);
    return posePrim.GetAttribute(attrName);
}

TfToken getPoseNameFromPurpose(const UsdPrim prim, const TfToken posePurposeToken)
{
    TfTokenVector allAPIs = prim.GetAppliedSchemas();

    TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
    TfToken poseTypeName = UsdSchemaRegistry::GetAPISchemaTypeName(poseType);

    for (const auto& api : allAPIs)
    {
        std::pair<TfToken, TfToken> typeNameAndInstance = UsdSchemaRegistry::GetTypeNameAndInstance(api);
        if (typeNameAndInstance.first == poseTypeName)
        {
            VtArray<TfToken> candTokens;
            getPosePurposesAttr(prim, typeNameAndInstance.second).Get(&candTokens);
            for (const TfToken candToken : candTokens)
            {
                if (candToken == posePurposeToken)
                {
                    return typeNameAndInstance.second;
                }
            }
        }
    }

    return TfToken();
}

} // namespace

namespace omni
{
namespace physx
{
namespace usdparser
{

void setToDefault(PhysxForceDesc& desc)
{
    desc.enabled = false;
    desc.accelerationMode = true;
    desc.force = { 0.0f, 0.0f, 0.0f };
    desc.torque = { 0.0f, 0.0f, 0.0f };
    desc.worldFrame = false;
    desc.body = kInvalidObjectId;
    desc.scene = kInvalidObjectId;
    desc.localRot = { 0.0f, 0.0f, 0.0f, 1.0f };
}

PhysxForceDesc* parsePhysxForce(AttachedStage& attachedStage, const UsdPrim& prim, UsdGeomXformCache& xformCache)
{
    PhysxForceDesc* desc = ICE_PLACEMENT_NEW(PhysxForceDesc)();
    setToDefault(*desc);

    const PhysxSchemaPhysxForceAPI forceAPI = PhysxSchemaPhysxForceAPI::Get(prim.GetStage(), prim.GetPrimPath());
    if (forceAPI)
    {
        getAttribute(desc->enabled, forceAPI.GetForceEnabledAttr(), updatePhysxForceEnabled);
        getAttribute(desc->worldFrame, forceAPI.GetWorldFrameEnabledAttr(), updatePhysxForceWorldFrameEnabled);

        GfVec3f val{ 0.f };
        getAttribute(val, forceAPI.GetForceAttr(), updatePhysxForce);
        GfVec3ToFloat3(val, desc->force);
        
        getAttribute(val, forceAPI.GetTorqueAttr(), updatePhysxTorque);
        GfVec3ToFloat3(val, desc->torque);

        TfToken mode;
        getAttribute(mode, forceAPI.GetModeAttr(), updatePhysxForceMode);
        if (mode == PhysxSchemaTokens->acceleration)
        {
            desc->accelerationMode = true;
        }
        else
        {
            desc->accelerationMode = false;
        }

        const GfMatrix4d pose = xformCache.GetLocalToWorldTransform(prim);
        GfVec3ToFloat3(pose.ExtractTranslation(), desc->worldPos);

        bool isTimeVarying = false;
        UsdPrim parent = prim;
        const UsdPrim root = prim.GetStage()->GetPseudoRoot();
        while (!isTimeVarying && parent != root)
        {
            UsdGeomXformable xform(parent);
            bool resetsXformStack;
            for (auto xformOp : xform.GetOrderedXformOps(&resetsXformStack))
            {
                if (xformOp.GetNumTimeSamples() > 1)
                {
                    isTimeVarying = true;
                    break;
                }
            }

            parent = parent.GetParent();
        }

        if (isTimeVarying)
        {
            attachedStage.getAnimatedKinematicBodies()[prim.GetPrimPath()] = prim;
        }
    }

    return desc;
}

SdfPath getRigidBodySimulationOwner(UsdStageWeakPtr stage, const SdfPath& bodyPath)
{
    if (bodyPath == SdfPath())
        return SdfPath();

    const UsdPhysicsRigidBodyAPI rboAPI = UsdPhysicsRigidBodyAPI::Get(stage, bodyPath);
    if (rboAPI)
    {
        SdfPathVector owners;
        rboAPI.GetSimulationOwnerRel().GetTargets(&owners);
        if (!owners.empty())
        {
            return owners[0];
        }
    }
    else
    {
        const UsdPhysicsCollisionAPI collisionAPI = UsdPhysicsCollisionAPI::Get(stage, bodyPath);
        if (collisionAPI)
        {
            SdfPathVector owners;
            collisionAPI.GetSimulationOwnerRel().GetTargets(&owners);
            if (!owners.empty())
            {
                return owners[0];
            }
        }
    }
    return SdfPath();
}

void finalizePhysxForce(AttachedStage& attachedStage, const UsdPrim& usdPrim, PhysxForceDesc& desc, UsdGeomXformCache& xformCache)
{
    // search if the force belongs to some dynamic body
    ObjectId bodyId = kInvalidObjectId;
    UsdPrim parent = usdPrim;
    while (parent && !parent.IsPseudoRoot())
    {
        bodyId = attachedStage.getObjectDatabase()->findEntry(parent.GetPrimPath(), eBody);
        if (bodyId != kInvalidObjectId)
        {
            break;
        }

        bodyId = attachedStage.getObjectDatabase()->findEntry(parent.GetPrimPath(), eArticulationLink);
        if (bodyId != kInvalidObjectId)
        {
            break;
        }
        parent = parent.GetParent();
    }

    if (bodyId != kInvalidObjectId)
    {
        desc.body = bodyId;
        UsdPhysicsRigidBodyAPI rboAPI = UsdPhysicsRigidBodyAPI::Get(parent.GetStage(), parent.GetPrimPath());
        if (rboAPI)
        {
            SdfPathVector owners;
            rboAPI.GetSimulationOwnerRel().GetTargets(&owners);
            if (!owners.empty())
            {
                const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(owners[0], eScene);
                desc.scene = entry;
            }

            // local rotation between force and body
            GfVec3f localPos;
            GfVec3f localScale;
            GfQuatf localRot;
            omni::physics::schema::getCollisionShapeLocalTransfrom(xformCache, usdPrim, parent,
                localPos, localRot, localScale);
            GfQuatToFloat4(localRot, desc.localRot);
        }
    }
}

ObjectId getRigidBody(AttachedStage& attachedStage, const SdfPath& shapePath, PhysxShapeDesc& shapeDesc)
{
    const SdfPath& path = shapeDesc.rigidBody;
    if (path != SdfPath())
        return attachedStage.getObjectDatabase()->findEntry(path, eBody);
    else
    {
        UsdStageWeakPtr stage = attachedStage.getStage();
        UsdPrim parent = stage->GetPrimAtPath(shapePath);
        while (parent != stage->GetPseudoRoot())
        {
            ObjectId bodyId = attachedStage.getObjectDatabase()->findEntry(parent.GetPrimPath(), eBody);
            if (bodyId != kInvalidObjectId)
            {
                shapeDesc.rigidBody = parent.GetPrimPath();
                return bodyId;
            }
            else
            {
                bodyId = attachedStage.getObjectDatabase()->findEntry(parent.GetPrimPath(), eArticulationLink);
                if (bodyId != kInvalidObjectId)
                {
                    shapeDesc.rigidBody = parent.GetPrimPath();
                    return bodyId;
                }
            }

            parent = parent.GetParent();
        }
        return kInvalidObjectId;
    }
}

PhysxRigidBodyDesc* createStaticBody()
{
    StaticPhysxRigidBodyDesc* desc = ICE_PLACEMENT_NEW(StaticPhysxRigidBodyDesc)();

    return desc;
}

bool fillRigidBodyDesc(AttachedStage& attachedStage, const omni::physics::schema::RigidBodyDesc& inDesc, PhysxRigidBodyDesc& desc, CollisionPairVector& filteredPairs)
{
    GfVec3ToFloat3(inDesc.position, desc.position);
    GfQuatToFloat4(inDesc.rotation, desc.rotation);
    GfVec3ToFloat3(inDesc.scale, desc.scale);

    const SdfPath& primPath = inDesc.usdPrim.GetPrimPath();
    for (size_t i = 0; i < inDesc.filteredCollisions.size(); i++)
    {
        filteredPairs.push_back(std::make_pair(primPath, inDesc.filteredCollisions[i]));
    }


    for (size_t i = 0; i < inDesc.simulationOwners.size(); i++)
    {
        const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(inDesc.simulationOwners[i], eScene);
        if (entry != kInvalidObjectId)
            desc.sceneIds.push_back(entry);
    }

    if (!inDesc.simulationOwners.empty() && desc.sceneIds.empty())
    {
        return false;
    }

    return true;
}

void finalizeRigidBody(AttachedStage& attachedStage, BodyDescAndColliders& bodyAndColliders)
{
    for (const SdfPath& collisionPath : bodyAndColliders.collisions)
    {
        if (collisionPath != SdfPath())
        {
            const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(collisionPath);
            if (entries && !entries->empty())
            {
                auto it = entries->begin();
                while (it != entries->end())
                {
                    if (it->first == eShape)
                    {
                        const ObjectId collisionId = it->second;
                        if (collisionId != kInvalidObjectId)
                        {
                            bodyAndColliders.desc->shapes.push_back(collisionId);
                        }
                    }
                    it++;
                }

            }
        }
    }
}

void setToDefault(UsdStageWeakPtr stage, DynamicPhysxRigidBodyDesc& desc)
{
    double metersPerUnit = UsdGeomGetStageMetersPerUnit(stage);
    float tolerancesSpeed = float(10.0f / metersPerUnit);

    desc.linearDamping = 0.0f;
    desc.angularDamping = 0.05f;
    desc.maxLinearVelocity = SQRT_FLT_MAX;
    desc.maxAngularVelocity = 100.0f;
    desc.maxContactImpulse = FLT_MAX;
    desc.sleepThreshold = 5e-5f * tolerancesSpeed * tolerancesSpeed;
    desc.stabilizationThreshold = 1e-5f * tolerancesSpeed * tolerancesSpeed;
    desc.maxDepenetrationVelocity = float(3.0f / metersPerUnit);
    desc.contactSlopCoefficient = 0.0f;
    desc.solverPositionIterationCount = 16;
    desc.solverVelocityIterationCount = 1;
    desc.cfmScale = 0.025f;

    desc.enableCCD = false;
    desc.enableSpeculativeCCD = false;
    desc.disableGravity = false;
    desc.retainAccelerations = false;
    desc.enableGyroscopicForces = true;
    desc.localSpaceVelocities = false;
    desc.solveContacts = true;

    desc.lockedPosAxis = 0;
    desc.lockedRotAxis = 0;

    desc.surfaceVelocityEnabled = false;
    desc.surfaceLinearVelocity = { 0.0f, 0.0f, 0.0f };
    desc.surfaceAngularVelocity = { 0.0f, 0.0f, 0.0f };

    desc.splinesSurfaceVelocityEnabled = false;
    desc.splinesSurfaceVelocityMagnitude = 0.0f;
    desc.splinesCurvePrimPath = SdfPath();
}

inline bool getLocalSpaceVelocitiesValue(const UsdPrim& prim, bool& value)
{
    TfToken gLocalSpaceVelocitiesToken(omni::physx::kLocalSpaceVelocitiesMetadataAttributeName);
    VtValue v;

    if (prim.GetMetadataByDictKey(SdfFieldKeys->CustomData, gLocalSpaceVelocitiesToken, &v))
    {
        value = v.Get<bool>();
        return true;
    }

    return false;
}

PhysxRigidBodyDesc* parseRigidBody(AttachedStage& attachedStage, UsdGeomXformCache& xformCache, const omni::physics::schema::RigidBodyDesc& inDesc,
    CollisionPairVector& filteredPairs, bool ignoreOwners)
{
    PhysxRigidBodyDesc* desc = nullptr;
    if (inDesc.rigidBodyEnabled)
    {
        // dynamic body
        // static body
        DynamicPhysxRigidBodyDesc* dynamicBodyDesc = ICE_PLACEMENT_NEW(DynamicPhysxRigidBodyDesc)();
        desc = dynamicBodyDesc;
        setToDefault(attachedStage.getStage(), *dynamicBodyDesc);

        // Check for output velocities in local space setting
        bool outputVelocitiesLocalSpace = OmniPhysX::getInstance().getISettings()->getAsBool(kSettingOutputVelocitiesLocalSpace);
        // Check if output velocities in local space setting are overridden for this body
        bool localSpaceVelocitiesRigidBody;
        if (getLocalSpaceVelocitiesValue(inDesc.usdPrim, localSpaceVelocitiesRigidBody))
            outputVelocitiesLocalSpace = localSpaceVelocitiesRigidBody;
        dynamicBodyDesc->localSpaceVelocities = outputVelocitiesLocalSpace;

        GfVec3f transformedVelocity = inDesc.linearVelocity;
        GfVec3f transformedAngularVelocity = inDesc.angularVelocity;
        if (outputVelocitiesLocalSpace)
        {
            // Local velocity:
            // rotate by the localToWorld and scale linear velocity based on scale
            const GfMatrix4d localToWorld = xformCache.GetLocalToWorldTransform(inDesc.usdPrim);
            const GfTransform tr(localToWorld);
            const GfRotation rot(tr.GetRotation());
            const GfVec3f scale(tr.GetScale());

            transformedVelocity = rot.TransformDir(transformedVelocity);
            transformedVelocity = GfCompMult(scale, transformedVelocity);
            transformedAngularVelocity = rot.TransformDir(transformedAngularVelocity);
        }

        // generic bits
        dynamicBodyDesc->kinematicBody = inDesc.kinematicBody;
        dynamicBodyDesc->startsAsleep = inDesc.startsAsleep;
        GfVec3ToFloat3(transformedVelocity, dynamicBodyDesc->linearVelocity);
        GfVec3ToFloat3(degToRad(transformedAngularVelocity), dynamicBodyDesc->angularVelocity);

        // physx bits
        const PhysxSchemaPhysxRigidBodyAPI physxRigidBodyAPI = PhysxSchemaPhysxRigidBodyAPI::Get(attachedStage.getStage(), inDesc.usdPrim.GetPrimPath());
        if (physxRigidBodyAPI)
        {
            getAttribute(dynamicBodyDesc->linearDamping, physxRigidBodyAPI.GetLinearDampingAttr(), 0.0f, FLT_MAX, updateBodyLinearDamping);
            getAttribute(dynamicBodyDesc->angularDamping, physxRigidBodyAPI.GetAngularDampingAttr(), 0.0f, FLT_MAX, updateBodyAngularDamping);
            getAttribute(dynamicBodyDesc->maxLinearVelocity, physxRigidBodyAPI.GetMaxLinearVelocityAttr(), 0.0f, SQRT_FLT_MAX, updateBodyMaxLinearVelocity);
            if (getAttribute(dynamicBodyDesc->maxAngularVelocity, physxRigidBodyAPI.GetMaxAngularVelocityAttr(), 0.0f, SQRT_FLT_MAX, updateBodyMaxAngularVelocity))
            {
                dynamicBodyDesc->maxAngularVelocity = degToRad(dynamicBodyDesc->maxAngularVelocity);
            }
            getAttribute(dynamicBodyDesc->sleepThreshold, physxRigidBodyAPI.GetSleepThresholdAttr(), 0.0f, FLT_MAX, updateBodySleepThreshold);
            getAttribute(dynamicBodyDesc->stabilizationThreshold, physxRigidBodyAPI.GetStabilizationThresholdAttr(), 0.0f, FLT_MAX, updateBodyStabilizationThreshold);
            getAttribute(dynamicBodyDesc->maxDepenetrationVelocity, physxRigidBodyAPI.GetMaxDepenetrationVelocityAttr(), 0.0f, FLT_MAX, updateBodyMaxDepenetrationVelocity);
            getAttribute(dynamicBodyDesc->contactSlopCoefficient, physxRigidBodyAPI.GetContactSlopCoefficientAttr(), 0.0f, FLT_MAX, updateBodyContactSlopCoefficient);
            getAttribute(dynamicBodyDesc->maxContactImpulse, physxRigidBodyAPI.GetMaxContactImpulseAttr(), 0.0f, FLT_MAX, updateBodyMaxContactImpulse);
            getAttribute(dynamicBodyDesc->cfmScale, physxRigidBodyAPI.GetCfmScaleAttr(), 0.0f, 1.0f, updateBodyCfmScale);

            getAttribute(dynamicBodyDesc->solverPositionIterationCount, physxRigidBodyAPI.GetSolverPositionIterationCountAttr(), 1, 255, updateBodySolverPositionIterationCount);
            getAttribute(dynamicBodyDesc->solverVelocityIterationCount, physxRigidBodyAPI.GetSolverVelocityIterationCountAttr(), 0, 255, updateBodySolverVelocityIterationCount);

            getBoolAttribute(dynamicBodyDesc->enableCCD, physxRigidBodyAPI.GetEnableCCDAttr(), updateBodyEnableCCD);
            getBoolAttribute(dynamicBodyDesc->enableSpeculativeCCD, physxRigidBodyAPI.GetEnableSpeculativeCCDAttr(), updateBodyEnableSpeculativeCCD);
            getBoolAttribute(dynamicBodyDesc->disableGravity, physxRigidBodyAPI.GetDisableGravityAttr(), updateBodyDisableGravity);
            getBoolAttribute(dynamicBodyDesc->retainAccelerations, physxRigidBodyAPI.GetRetainAccelerationsAttr(), updateBodyRetainAccelerations);
            getBoolAttribute(dynamicBodyDesc->enableGyroscopicForces, physxRigidBodyAPI.GetEnableGyroscopicForcesAttr(), updateBodyGyroscopicForces);
            getBoolAttribute(dynamicBodyDesc->solveContacts, physxRigidBodyAPI.GetSolveContactAttr(), updateBodySolveContacts);

            getAttribute(dynamicBodyDesc->lockedPosAxis, physxRigidBodyAPI.GetLockedPosAxisAttr(), 0, 7, updateBodyLockedPosAxis);
            getAttribute(dynamicBodyDesc->lockedRotAxis, physxRigidBodyAPI.GetLockedRotAxisAttr(), 0, 7, updateBodyLockedRotAxis);
        }

        if (dynamicBodyDesc->kinematicBody && (
            !GfIsClose((const GfVec3f&)(dynamicBodyDesc->linearVelocity), GfVec3f(0.0f), 1e-4) ||
            !GfIsClose((const GfVec3f&)(dynamicBodyDesc->angularVelocity), GfVec3f(0.0f), 1e-4)
            ))
        {
            CARB_LOG_INFO("Defining surface velocity through kinematic body is deprecated, please use new PhysxSchemaPhysxSurfaceVelocityAPI, prim: %s", inDesc.usdPrim.GetPrimPath().GetText());
            dynamicBodyDesc->surfaceVelocityEnabled = true;
            dynamicBodyDesc->surfaceVelocityLocalSpace = false;
            dynamicBodyDesc->surfaceLinearVelocity = dynamicBodyDesc->linearVelocity;
            dynamicBodyDesc->surfaceAngularVelocity = dynamicBodyDesc->angularVelocity;
        }

        const PhysxSchemaPhysxSurfaceVelocityAPI physxSurfaceVelocityAPI = PhysxSchemaPhysxSurfaceVelocityAPI::Get(attachedStage.getStage(), inDesc.usdPrim.GetPrimPath());
        if (physxSurfaceVelocityAPI)
        {
            dynamicBodyDesc->surfaceVelocityEnabled = true;
            dynamicBodyDesc->surfaceVelocityLocalSpace = true;
            getBoolAttribute(dynamicBodyDesc->surfaceVelocityEnabled, physxSurfaceVelocityAPI.GetSurfaceVelocityEnabledAttr(), updateBodySurfaceVelocityEnabled);
            getBoolAttribute(dynamicBodyDesc->surfaceVelocityLocalSpace, physxSurfaceVelocityAPI.GetSurfaceVelocityLocalSpaceAttr(), updateBodySurfaceVelocityLocalSpace);
            getAttribute((GfVec3f&)dynamicBodyDesc->surfaceLinearVelocity, physxSurfaceVelocityAPI.GetSurfaceVelocityAttr(), updateBodySurfaceLinearVelocity);
            getAttribute((GfVec3f&)dynamicBodyDesc->surfaceAngularVelocity, physxSurfaceVelocityAPI.GetSurfaceAngularVelocityAttr(), updateBodySurfaceAngularVelocity);
            (GfVec3f&)dynamicBodyDesc->surfaceAngularVelocity = degToRad((GfVec3f&)dynamicBodyDesc->surfaceAngularVelocity);

            if (dynamicBodyDesc->surfaceVelocityLocalSpace)
            {
                const GfMatrix4d localToWorld = xformCache.GetLocalToWorldTransform(inDesc.usdPrim);
                const GfTransform tr(localToWorld);
                const GfVec3f scale(tr.GetScale());

                (GfVec3f&)dynamicBodyDesc->surfaceLinearVelocity = GfCompMult(scale, (GfVec3f&)dynamicBodyDesc->surfaceLinearVelocity);
            }
        }

        const bool hasSplinesSurfaceVelocity = inDesc.usdPrim.HasAPI(
            UsdSchemaRegistry::GetTypeFromSchemaTypeName(PhysxAdditionAPITokens->PhysxSplinesSurfaceVelocityAPI));
        if (hasSplinesSurfaceVelocity)
        {
            dynamicBodyDesc->splinesSurfaceVelocityEnabled = true;
            getBoolAttribute(dynamicBodyDesc->splinesSurfaceVelocityEnabled,
                             inDesc.usdPrim.GetAttribute(
                                 PhysxAdditionAttrTokens->surfaceVelocityEnabled),
                             updateBodySplineSurfaceVelocityEnabled);

            if (dynamicBodyDesc->surfaceVelocityEnabled && dynamicBodyDesc->splinesSurfaceVelocityEnabled)
            {
                CARB_LOG_ERROR(
                    "Detected rigid body (%s) with both surface velocity and splines surface velocity, please disable one.",
                    inDesc.usdPrim.GetPrimPath().GetText());

                dynamicBodyDesc->splinesSurfaceVelocityEnabled = false;
            }
            else if (dynamicBodyDesc->splinesSurfaceVelocityEnabled)
            {
                getAttribute(dynamicBodyDesc->splinesSurfaceVelocityMagnitude,
                             inDesc.usdPrim.GetAttribute(
                                 PhysxAdditionAttrTokens->surfaceVelocityMagnitude),
                             updateBodySplineSurfaceVelocityMagnitude);

                UsdRelationship splinesRel = inDesc.usdPrim.GetRelationship(
                    PhysxAdditionAttrTokens->surfaceVelocityCurve);
                SdfPathVector splinesList;
                if (splinesRel)
                {                    
                    splinesRel.GetTargets(&splinesList);
                }

                if (!splinesRel || splinesList.empty())
                {
                    CARB_LOG_ERROR(
                        "Splines surface velocity %s does not have a valid spline curve defined.",
                        inDesc.usdPrim.GetPrimPath().GetText());

                    dynamicBodyDesc->splinesSurfaceVelocityEnabled = false;
                }
                else
                {
                    const SdfPath& splinePath = splinesList[0];
                    const UsdPrim splinePrim = attachedStage.getStage()->GetPrimAtPath(splinePath);
                    if (!splinePrim || !splinePrim.IsA<UsdGeomBasisCurves>())
                    {
                        CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined.",
                                       inDesc.usdPrim.GetPrimPath().GetText());

                        dynamicBodyDesc->splinesSurfaceVelocityEnabled = false;
                    }
                    else
                    {
                        bool parentBodyFound = false;
                        UsdPrim parentPrim = splinePrim.GetParent();
                        while (parentPrim != attachedStage.getStage()->GetPseudoRoot())
                        {
                            if (parentPrim == inDesc.usdPrim)
                            {
                                parentBodyFound = true;
                                break;
                            }
                            parentPrim = parentPrim.GetParent();
                        }
                        if (!parentBodyFound)
                        {
                            CARB_LOG_ERROR("Splines surface velocity %s spline curve is not a child of the rigid body.",
                                           inDesc.usdPrim.GetPrimPath().GetText());

                            dynamicBodyDesc->splinesSurfaceVelocityEnabled = false;
                        }
                        else
                        {
                            dynamicBodyDesc->splinesCurvePrimPath = splinePath;
                        }
                    }
                }
            }
        }

        // checking for animated transformations
        const bool isTimeVarying = primutils::IsTransformTimeVarying(inDesc.usdPrim);
        if (isTimeVarying)
        {
            dynamicBodyDesc->hasTimeSampledXform = true;
            if (dynamicBodyDesc->kinematicBody)
            {
                // TODO: investigate whether this might be sufficient
                // isTimeVarying = xform.TransformMightBeTimeVarying()
                attachedStage.getAnimatedKinematicBodies()[inDesc.usdPrim.GetPrimPath()] = inDesc.usdPrim;                
            }
            else
            {
                CARB_LOG_WARN("Detected rigid body that is not kinematic but does have parents with animated xformOps. Prim: %s, converting the body to kinematic body", inDesc.usdPrim.GetPrimPath().GetText());
                dynamicBodyDesc->kinematicBody = true;
                attachedStage.getAnimatedKinematicBodies()[inDesc.usdPrim.GetPrimPath()] = inDesc.usdPrim;
            }
        }

        // check for time samples on velocity attribute
        UsdAttribute velAttr = inDesc.usdPrim.GetAttribute(UsdPhysicsTokens->physicsVelocity);
        if (velAttr.ValueMightBeTimeVarying())
        {
            UsdLoad::getUsdLoad()->registerTimeSampledAttribute(velAttr, updateBodyLinearVelocity);
        }
    }
    else
    {
        // static body
        desc = ICE_PLACEMENT_NEW(StaticPhysxRigidBodyDesc)();
    }

    if (!fillRigidBodyDesc(attachedStage, inDesc, *desc, filteredPairs) && !ignoreOwners)
    {
        ICE_FREE(desc);
        desc = nullptr;
    }

    return desc;
}

void setToDefault(UsdStageWeakPtr stage, PhysxDeformableBodyDesc& desc)
{
    double metersPerUnit = UsdGeomGetStageMetersPerUnit(stage);
    float tolerancesSpeed = float(10.0f / metersPerUnit);

    desc.sceneId = kInvalidObjectId;
    desc.simMeshMaterial = kInvalidObjectId;
    desc.transform = GfMatrix4d(1.0f);
    desc.bodyEnabled = false;
    desc.kinematicBody = false;
    desc.startsAsleep = false;
    desc.mass = -1.0f;
    desc.enableSpeculativeCCD = false;
    desc.selfCollision = false;
    desc.disableGravity = false;
    desc.sleepThreshold = 5e-5f * tolerancesSpeed * tolerancesSpeed;
    desc.linearDamping = 0.005f;
    desc.maxLinearVelocity = SQRT_FLT_MAX;
    desc.settlingThreshold = float(0.1f / metersPerUnit);
    desc.settlingDamping = 10.0f;
    desc.maxDepenetrationVelocity = float(3.0f / metersPerUnit);
    desc.contactOffset = -1.0f;
    desc.restOffset = float(0.02f / metersPerUnit);
    desc.selfCollisionFilterDistance = -1.0f;
    desc.solverPositionIterationCount = 16;
    desc.hasAutoAPI = false;
    desc.isAutoMeshSimplificationEnabled = false;
    desc.isAutoRemeshingEnabled = false;
    desc.hasAutoForceConforming = false;
    desc.autoRemeshingResolution = 0;
    desc.autoTriangleTargetCount = 0;
}

void setToDefault(UsdStageWeakPtr stage, PhysxVolumeDeformableBodyDesc& desc)
{
    setToDefault(stage, static_cast<PhysxDeformableBodyDesc&>(desc));

    desc.isAutoHexahedralMeshEnabled = false;
    desc.autoHexahedralResolution = 0;
}

void setToDefault(UsdStageWeakPtr stage, PhysxSurfaceDeformableBodyDesc& desc)
{
    setToDefault(stage, static_cast<PhysxDeformableBodyDesc&>(desc));

    desc.restBendAnglesDefault = OmniPhysicsDeformableAttrTokens->flatDefault;
    desc.collisionPairUpdateFrequency = 1;
    desc.collisionIterationMultiplier = 1;
}

const SdfPath* findFirstNonEmptyPath(const SdfPathVector& paths)
{
    auto it = std::find_if(paths.begin(), paths.end(), [](const SdfPath path) { return !path.IsEmpty(); });
    return (it != paths.end()) ? &(*it) : nullptr;
}

const TfToken* findFirstNonEmptyToken(const TfTokenVector& tokens)
{
    auto it = std::find_if(tokens.begin(), tokens.end(), [](const TfToken token) { return !token.IsEmpty(); });
    return (it != tokens.end()) ? &(*it) : nullptr;
}

PhysxDeformableBodyDesc* parseDeformableBody(AttachedStage& attachedStage, UsdGeomXformCache& xformCache, const SdfPath& path,
    const omni::physics::schema::DeformableBodyDesc& inDesc, CollisionPairVector& filteredPairs, SdfPath& simMeshMaterial)
{
    uint64_t schemaFlags = attachedStage.getObjectDatabase()->getSchemaAPIs(path);
    PhysxDeformableBodyDesc* desc = nullptr;
    if (inDesc.type == omni::physics::schema::ObjectType::eVolumeDeformableBody)
    {
        PhysxVolumeDeformableBodyDesc* volumeDesc = ICE_PLACEMENT_NEW(PhysxVolumeDeformableBodyDesc)();
        setToDefault(attachedStage.getStage(), *volumeDesc);
        desc = volumeDesc;
    }
    else if (inDesc.type == omni::physics::schema::ObjectType::eSurfaceDeformableBody)
    {
        PhysxSurfaceDeformableBodyDesc* surfaceDesc = ICE_PLACEMENT_NEW(PhysxSurfaceDeformableBodyDesc)();
        setToDefault(attachedStage.getStage(), *surfaceDesc);
        desc = surfaceDesc;
    }

    if (!desc)
    {
        return nullptr;
    }

    if ((schemaFlags & SchemaAPIFlag::eAutoDeformableBodyAPI) > 0)
    {
        getAttribute(desc->hasAutoAPI, inDesc.usdPrim.GetAttribute(PhysxAdditionAttrTokens->autoDeformableBodyEnabled), nullptr);
        UsdRelationship cookingSourceMeshRel = inDesc.usdPrim.GetRelationship(PhysxAdditionAttrTokens->cookingSourceMesh);
        if (cookingSourceMeshRel)
        {
            SdfPathVector pathVector;
            cookingSourceMeshRel.GetTargets(&pathVector);
            if (pathVector.size() > 0)
            {
                desc->cookingSrcMeshPath = pathVector[0];
                UsdGeomMesh cookingSrcMesh = UsdGeomMesh::Get(attachedStage.getStage(), desc->cookingSrcMeshPath);
                if (cookingSrcMesh)
                {
                    pxr::TfToken bindPoseToken = getPoseNameFromPurpose(cookingSrcMesh.GetPrim(), OmniPhysicsDeformableAttrTokens->bindPose);
                    desc->cookingSrcMeshBindPoseToken = bindPoseToken;
                }
            }
        }

        if ((schemaFlags & SchemaAPIFlag::eAutoDeformableMeshSimplificationAPI) > 0)
        {
            getAttribute(desc->isAutoMeshSimplificationEnabled, inDesc.usdPrim, PhysxAdditionAttrTokens->autoDeformableMeshSimplificationEnabled, nullptr);
            getAttribute(desc->isAutoRemeshingEnabled, inDesc.usdPrim, PhysxAdditionAttrTokens->remeshingEnabled, nullptr);
            getAttribute(desc->autoRemeshingResolution, inDesc.usdPrim, PhysxAdditionAttrTokens->remeshingResolution, nullptr);
            getAttribute(desc->autoTriangleTargetCount, inDesc.usdPrim, PhysxAdditionAttrTokens->targetTriangleCount, nullptr);
            getAttribute(desc->hasAutoForceConforming, inDesc.usdPrim, PhysxAdditionAttrTokens->forceConforming, nullptr);
        }

        if ((schemaFlags & SchemaAPIFlag::eAutoDeformableHexahedralMeshAPI) > 0)
        {
            PhysxVolumeDeformableBodyDesc* volumeDesc = (PhysxVolumeDeformableBodyDesc*)desc;
            volumeDesc->isAutoHexahedralMeshEnabled = true;
            getAttribute(volumeDesc->autoHexahedralResolution, inDesc.usdPrim, PhysxAdditionAttrTokens->resolution, nullptr);
        }
    }

    const TfType physxDBType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->BaseDeformableBodyAPI);
    if (inDesc.usdPrim.HasAPI(physxDBType))
    {
        getAttribute(desc->linearDamping, inDesc.usdPrim, PhysxAdditionAttrTokens->linearDamping, 0.0f, FLT_MAX, nullptr);
        getAttribute(desc->maxLinearVelocity, inDesc.usdPrim, PhysxAdditionAttrTokens->maxLinearVelocity, -SQRT_FLT_MAX, SQRT_FLT_MAX, nullptr);
        getAttribute(desc->sleepThreshold, inDesc.usdPrim, PhysxAdditionAttrTokens->sleepThreshold, 0.0f, FLT_MAX, nullptr);
        getAttribute(desc->settlingThreshold, inDesc.usdPrim, PhysxAdditionAttrTokens->settlingThreshold, 0.0f, FLT_MAX, nullptr);
        getAttribute(desc->settlingDamping, inDesc.usdPrim, PhysxAdditionAttrTokens->settlingDamping, 0.0f, FLT_MAX, nullptr);
        getAttribute(desc->maxDepenetrationVelocity, inDesc.usdPrim, PhysxAdditionAttrTokens->maxDepenetrationVelocity, 0.0f, FLT_MAX, nullptr);
        getAttribute(desc->solverPositionIterationCount, inDesc.usdPrim, PhysxAdditionAttrTokens->solverPositionIterationCount, 1u, 255u, nullptr);
        getAttribute(desc->selfCollisionFilterDistance, inDesc.usdPrim, PhysxAdditionAttrTokens->selfCollisionFilterDistance, 0.0f, FLT_MAX, nullptr);
        getBoolAttribute(desc->enableSpeculativeCCD, inDesc.usdPrim, PhysxAdditionAttrTokens->enableSpeculativeCCD, nullptr);
        getBoolAttribute(desc->selfCollision, inDesc.usdPrim, PhysxAdditionAttrTokens->selfCollision, nullptr);
        getBoolAttribute(desc->disableGravity, inDesc.usdPrim, PhysxAdditionAttrTokens->disableGravity, nullptr);
    }

    const TfType surfaceDBType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->SurfaceDeformableBodyAPI);
    if (inDesc.usdPrim.HasAPI(surfaceDBType))
    {
        PhysxSurfaceDeformableBodyDesc* surfaceDesc = (PhysxSurfaceDeformableBodyDesc*)desc;
        getAttribute(surfaceDesc->collisionPairUpdateFrequency, inDesc.usdPrim, PhysxAdditionAttrTokens->collisionPairUpdateFrequency, nullptr);
        getAttribute(surfaceDesc->collisionIterationMultiplier, inDesc.usdPrim, PhysxAdditionAttrTokens->collisionIterationMultiplier, nullptr);
    }

    //as opposed to rigid bodies, we need to pass on the bodyEnabled flag
    //so we can support live update. Rigid bodies have a dynamic and a static type instead.
    desc->bodyEnabled = inDesc.bodyEnabled;
    desc->kinematicBody = inDesc.kinematicBody;
    desc->startsAsleep = inDesc.startsAsleep;

    desc->transform = inDesc.transform;
    desc->mass = (inDesc.mass <= 0.0f) ? -1.0f : inDesc.mass;

    desc->simMeshPath = inDesc.simMeshPath;

    if (inDesc.type == omni::physics::schema::ObjectType::eSurfaceDeformableBody)
    {
        const UsdPrim simMeshPrim = attachedStage.getStage()->GetPrimAtPath(inDesc.simMeshPath);
        if (simMeshPrim && simMeshPrim.HasAPI(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI))
        {
            UsdAttribute restBendAnglesDefaultAttr = simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restBendAnglesDefault);
            if (restBendAnglesDefaultAttr)
            {
                PhysxSurfaceDeformableBodyDesc* surfaceDesc = (PhysxSurfaceDeformableBodyDesc*)desc;
                restBendAnglesDefaultAttr.Get(&surfaceDesc->restBendAnglesDefault);
            }
        }
    }

    if (inDesc.collisionGeomPaths.empty())
    {
        CARB_LOG_WARN("No UsdPhysics.CollisionAPI found on deformable body prim or sub-tree, which is currently unsupported. "
                      "Parsing failed. Prim: %s",
                      inDesc.usdPrim.GetPrimPath().GetText());
        ICE_FREE(desc);
        return nullptr;
    }
    if (inDesc.collisionGeomPaths.size() > 1)
    {
        CARB_LOG_WARN("More than one UsdPhysics.CollisionAPI found on deformable body prim or sub-tree, which is currently unsupported. "
                      "Ignoring all but first. Prim: %s",
                      inDesc.usdPrim.GetPrimPath().GetText());
    }
    if (desc->type == ObjectType::eVolumeDeformableBody)
    {
        UsdPrim collMeshPrim = attachedStage.getStage()->GetPrimAtPath(inDesc.collisionGeomPaths[0]);
        if (!collMeshPrim.IsA<UsdGeomTetMesh>())
        {
            CARB_LOG_WARN("UsdPhysics.CollisionAPI on UsdGeomPointBased that are not UsdGeomTetMesh "
                          "is currently not supported for volume deformables. Parsing failed. Prim: %s",
                          inDesc.usdPrim.GetPrimPath().GetText());
            return nullptr;
        }
    }
    else if (desc->type == ObjectType::eSurfaceDeformableBody)
    {
        UsdPrim collMeshPrim = attachedStage.getStage()->GetPrimAtPath(inDesc.collisionGeomPaths[0]);
        UsdGeomMesh collMesh(collMeshPrim);
        if (!collMesh)
        {
            CARB_LOG_WARN("UsdPhysics.CollisionAPI on UsdGeomPointBased that are not UsdGeomMesh "
                "is currently not supported for surface deformables. Parsing failed. Prim: %s",
                inDesc.usdPrim.GetPrimPath().GetText());
            return nullptr;
        }
        if (inDesc.collisionGeomPaths[0] != inDesc.simMeshPath)
        {
            CARB_LOG_WARN("UsdPhysics.CollisionAPI found on different prim than UsdPhysics.SurfaceDeformableSimAPI, "
                "which is currently not supported for surface deformables. Parsing failed. Prim: %s",
                inDesc.usdPrim.GetPrimPath().GetText());
            return nullptr;
        }
    }
    desc->collisionMeshPath = inDesc.collisionGeomPaths[0];
    desc->skinGeomPaths = inDesc.skinGeomPaths;

    // collider
    if (!desc->collisionMeshPath.IsEmpty())
    {
        const UsdPrim collMeshPrim = attachedStage.getStage()->GetPrimAtPath(desc->collisionMeshPath);
        const PhysxSchemaPhysxCollisionAPI physxCollisionAPI(collMeshPrim);
        if (physxCollisionAPI)
        {
            // currently more or less a copy of code in Collision.cpp, TODO refactor
            float contactOffset = desc->contactOffset;
            float restOffset = desc->restOffset;
            {
                UsdAttribute attribute = physxCollisionAPI.GetContactOffsetAttr();
                if (attribute && attribute.HasAuthoredValue())
                {
                    float attrVal;
                    attribute.Get(&attrVal);
                    if (!isinf((float)attrVal))
                    {
                        if (attrVal >= 0.0f && attrVal <= FLT_MAX)
                        {
                            contactOffset = attrVal;
                        }
                    }

                    // check for time samples
                    if (attribute.GetNumTimeSamples() > 1)
                    {
                        attachedStage.registerTimeSampledAttribute(attribute.GetPath(), updateDeformableContactOffset);
                    }
                }
            }

            {
                UsdAttribute attribute = physxCollisionAPI.GetRestOffsetAttr();
                if (attribute && attribute.HasAuthoredValue())
                {
                    float attrVal;
                    attribute.Get(&attrVal);
                    if (!isinf((float)attrVal))
                    {
                        if (attrVal >= -FLT_MAX && attrVal <= FLT_MAX)
                        {
                            restOffset = attrVal;
                        }
                    }

                    // check for time samples
                    if (attribute.GetNumTimeSamples() > 1)
                    {
                        attachedStage.registerTimeSampledAttribute(attribute.GetPath(), updateDeformableRestOffset);
                    }
                }
            }

            if (contactOffset >= restOffset)
                desc->contactOffset = contactOffset;

            if (restOffset < contactOffset)
                desc->restOffset = restOffset;
        }

        desc->collisionGroup = getCollisionGroup(attachedStage, desc->collisionMeshPath);
    }

    // materials, note that the material gets set in finalizeDeformableBody
    simMeshMaterial = inDesc.simMeshMaterialPath;

    const SdfPath* firstCollMatPathPtr = findFirstNonEmptyPath(inDesc.collisionGeomMaterialPaths);
    const SdfPath* firstSkinMatPathPtr = findFirstNonEmptyPath(inDesc.skinGeomMaterialPaths);
    if (simMeshMaterial.IsEmpty())
    {
        if (firstCollMatPathPtr || firstSkinMatPathPtr)
        {
            CARB_LOG_WARN("No deformable material found for simulation mesh, but deformable materials found on "
                          "collision or graphical meshes, which are currently ignored. Prim: %s",
                          inDesc.usdPrim.GetPrimPath().GetText());
        }
    }

    const SdfPath& primPath = inDesc.usdPrim.GetPrimPath();
    for (size_t i = 0; i < inDesc.filteredCollisions.size(); i++)
    {
        filteredPairs.push_back(std::make_pair(primPath, inDesc.filteredCollisions[i]));
    }

    //ignore all but first simulation owner for now: TODO warn about it
    if (inDesc.simulationOwners.size())
    {
        const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(inDesc.simulationOwners[0], eScene);
        if (entry != kInvalidObjectId)
        {
            desc->sceneId = entry;
        }
    }

    //This would fail for parsing during cooking.
    //TODO maybe translate to sceneId after parsing? 
    //if (!inDesc.simulationOwners.empty() && desc->sceneId == kInvalidObjectId)
    //{
    //    ICE_FREE(desc);
    //    return nullptr;
    //}

    desc->simMeshBindPoseToken = inDesc.simMeshBindPoseToken;
    desc->collisionMeshBindPoseToken = inDesc.collisionGeomBindPoseTokens[0];
    const TfToken* fistCollTokenPtr = findFirstNonEmptyToken(inDesc.collisionGeomSelfCollisionFilterPoseTokens);
    if (fistCollTokenPtr)
    {
        CARB_LOG_WARN("Deformable self collision filter pose found for collision mesh, which is not supported. "
            "Using deformable bind pose instead. Prim: %s",
            inDesc.usdPrim.GetPrimPath().GetText());
    }
    desc->skinGeomBindPoseTokens = inDesc.skinGeomBindPoseTokens;

    return desc;
}

void finalizeDeformableBody(AttachedStage& attachedStage, PhysxDeformableBodyDesc* desc, const SdfPath simMeshMaterial)
{
    ObjectCategory type;
    if (desc->type == eVolumeDeformableBody)
    {
        type = eDeformableMaterial;
    }
    else if (desc->type == eSurfaceDeformableBody)
    {
        type = eSurfaceDeformableMaterial;
    }
    else
    {
        return;
    }

    desc->simMeshMaterial = getMaterial(attachedStage, simMeshMaterial, type);

    registerDeformablePoseChangeParams(attachedStage, desc->simMeshBindPoseToken);
    registerDeformablePoseChangeParams(attachedStage, desc->collisionMeshBindPoseToken);
    for (const TfToken instanceToken : desc->skinGeomBindPoseTokens)
    {
        registerDeformablePoseChangeParams(attachedStage, instanceToken);
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
