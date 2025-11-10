// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <internal/Internal.h>
#include <internal/InternalScene.h>
#include <internal/InternalDeformable.h>
#include <PhysXTools.h>
#include <private/omni/physx/PhysxUsd.h>
#include <usdLoad/AttachedStage.h>

#include <PxPhysicsAPI.h>

using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

bool omni::physx::updateDeformableBody(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType type;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(type, objectId);
    if (!objectRecord || !(type == PhysXType::ePTDeformableSurface || type == PhysXType::ePTDeformableVolume))
    {
        return false;
    }

    InternalDeformableBody* intDeformableBody = (InternalDeformableBody*)objectRecord->mInternalPtr;
    if (!intDeformableBody)
    {
        return false;
    }

    if (property == OmniPhysicsDeformableAttrTokens->mass)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        intDeformableBody->mBodyMass = data;
        attachedStage.getPhysXPhysicsInterface()->updateDeformableBodyMass(attachedStage, objectId);
    }
    else if (property == UsdGeomTokens.Get()->points)
    {
        attachedStage.getPhysXPhysicsInterface()->updateDeformableBodyPositions(attachedStage, objectId);
    }
    else if (property == UsdGeomTokens.Get()->velocities)
    {
        attachedStage.getPhysXPhysicsInterface()->updateDeformableBodyVelocities(attachedStage, objectId);
    }

    PxDeformableBody* deformableBody = nullptr;
    PxDeformableSurface* deformableSurface = nullptr;
    if (type == PhysXType::ePTDeformableSurface)
    {
        deformableBody = ((InternalSurfaceDeformableBody*)intDeformableBody)->mDeformableSurface;
        deformableSurface = ((InternalSurfaceDeformableBody*)intDeformableBody)->mDeformableSurface;
    }
    else if (type == PhysXType::ePTDeformableVolume)
    {
        deformableBody = ((InternalVolumeDeformableBody*)intDeformableBody)->mDeformableVolume;
    }

    if (!deformableBody)
    {
        return false;
    }

    if (property == PhysxAdditionAttrTokens.Get()->solverPositionIterationCount)
    {
        //TOTO switch schema to int for consistency with rigid bodies.
        uint32_t data;
        if (!getValue<uint32_t>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (intDeformableBody->mPhysXScene && intDeformableBody->mPhysXScene->getInternalScene())
        {
            data = intDeformableBody->mPhysXScene->getInternalScene()->clampPosIterationCount(data);
        }

        deformableBody->setSolverIterationCounts(data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->linearDamping)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        deformableBody->setLinearDamping(data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->maxLinearVelocity)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        deformableBody->setMaxVelocity(data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->settlingDamping)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        deformableBody->setSettlingDamping(data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->sleepThreshold)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        const bool disableSleeping = OmniPhysX::getInstance().getCachedSettings().disableSleeping;
        const float sleepThreshold = disableSleeping ? 0.0f : data;
        deformableBody->setSleepThreshold(sleepThreshold);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->settlingThreshold)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        const bool disableSleeping = OmniPhysX::getInstance().getCachedSettings().disableSleeping;
        const float settlingThreshold = disableSleeping ? 0.0f : data;
        deformableBody->setSettlingThreshold(settlingThreshold);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->maxDepenetrationVelocity)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        deformableBody->setMaxDepenetrationVelocity(data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->selfCollision)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        deformableBody->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, !data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->selfCollisionFilterDistance)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        attachedStage.getPhysXPhysicsInterface()->updateDeformableSelfCollisionFilterDistance(attachedStage, objectId, data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->enableSpeculativeCCD)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        deformableBody->setDeformableBodyFlag(PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD, data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->disableGravity)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        deformableBody->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->collisionPairUpdateFrequency)
    {
        uint32_t data;
        if (!getValue<uint32_t>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (deformableSurface)
            deformableSurface->setNbCollisionPairUpdatesPerTimestep(data);
    }
    else if (property == PhysxAdditionAttrTokens.Get()->collisionIterationMultiplier)
    {
        uint32_t data;
        if (!getValue<uint32_t>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (deformableSurface)
            deformableSurface->setNbCollisionSubsteps(data);
    }

    return true;
}

bool omni::physx::updateDeformableRestOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    InternalDeformableBody* internalBody = (InternalDeformableBody*)objectRecord->mInternalPtr;
    if (internalBody)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        attachedStage.getPhysXPhysicsInterface()->updateDeformableRestOffset(attachedStage, objectId, data);
    }
    return true;
}

bool omni::physx::updateDeformableContactOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    InternalDeformableBody* internalBody = (InternalDeformableBody*)objectRecord->mInternalPtr;
    if (internalBody)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        attachedStage.getPhysXPhysicsInterface()->updateDeformableContactOffset(attachedStage, objectId, data);
    }
    return true;
}

bool omni::physx::updateDeformableMaterial(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    PhysXType type;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(type, objectId);
    if (!objectRecord || !(type == PhysXType::ePTDeformableSurfaceMaterial || type == PhysXType::ePTDeformableVolumeMaterial))
    {
        return false;
    }

    PxDeformableMaterial* material = (PxDeformableMaterial*)objectRecord->mPtr;
    if (!material)
    {
        return false;
    }
    PxDeformableSurfaceMaterial* surfaceMaterial = nullptr;
    if (type == PhysXType::ePTDeformableSurfaceMaterial)
    {
        surfaceMaterial = static_cast<PxDeformableSurfaceMaterial*>(material);
    }

    if (property == OmniPhysicsDeformableAttrTokens->density)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        InternalDeformableMaterial* intMaterial = (InternalDeformableMaterial*)objectRecord->mInternalPtr;
        if (!intMaterial)
            return true;

        intMaterial->mDensity = data;
        for (usdparser::ObjectId bodyId : intMaterial->mDeformableIds)
        {
            attachedStage.getPhysXPhysicsInterface()->updateDeformableBodyMass(attachedStage, bodyId);
        }

        return true;
    }
    else if (property == OmniPhysicsDeformableAttrTokens->dynamicFriction)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        material->setDynamicFriction(data);
        return true;
    }
    else if (property == OmniPhysicsDeformableAttrTokens->staticFriction)
    {
        CARB_LOG_WARN("Static friction is not supported on deformable bodies, prim: %s", objectRecord->mPath.GetText());
    }
    else if (property == OmniPhysicsDeformableAttrTokens->youngsModulus)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        material->setYoungsModulus(data);
        return true;
    }
    else if (property == OmniPhysicsDeformableAttrTokens->poissonsRatio)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        material->setPoissons(data);
        return true;
    }
    else if (property == PhysxAdditionAttrTokens->elasticityDamping)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        material->setElasticityDamping(data);
        return true;
    }
    else if (property == OmniPhysicsDeformableAttrTokens->surfaceThickness)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (surfaceMaterial)
        {
            surfaceMaterial->setThickness(data);

            InternalDeformableMaterial* intMaterial = (InternalDeformableMaterial*)objectRecord->mInternalPtr;
            if (!intMaterial)
                return true;

            for (usdparser::ObjectId bodyId : intMaterial->mDeformableIds)
            {
                attachedStage.getPhysXPhysicsInterface()->updateDeformableBodyMass(attachedStage, bodyId);
            }
        }
        return true;
    }
    else if (property == OmniPhysicsDeformableAttrTokens->surfaceStretchStiffness)
    {
        CARB_LOG_WARN("Surface Stretch Stiffness is currently not supported. Prim: %s",
            objectRecord->mPath.GetText());
    }
    else if (property == OmniPhysicsDeformableAttrTokens->surfaceShearStiffness)
    {
        CARB_LOG_WARN("Surface Shear Stiffness is currently not supported. Prim: %s",
            objectRecord->mPath.GetText());
    }
    else if (property == OmniPhysicsDeformableAttrTokens->surfaceBendStiffness)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (surfaceMaterial)
            surfaceMaterial->setBendingStiffness(data);

        return true;
    }
    else if (property == PhysxAdditionAttrTokens->bendDamping)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (surfaceMaterial)
            surfaceMaterial->setBendingDamping(data);

        return true;
    }
    return false;
}
