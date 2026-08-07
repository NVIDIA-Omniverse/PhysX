// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"
#include "../usdLoad/NewtonCompat.h"

#include <internal/Internal.h>
#include <internal/InternalScene.h>
#include <internal/InternalDeformable.h>
#include <PhysXTools.h>
#include <private/omni/physx/PhysxUsd.h>
#include <usdLoad/AttachedStage.h>

#include <PxPhysicsAPI.h>

using namespace ::physx;
using namespace carb;
using namespace PXR_NS;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

bool omni::physx::updateDeformableBody(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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

    if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsMass)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
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

    if (property == PhysxSchemaTokens->physxDeformableBodySolverPositionIterationCount)
    {
        //TOTO switch schema to int for consistency with rigid bodies.
        uint32_t data;
        if (!getValue<uint32_t>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        if (intDeformableBody->mPhysXScene && intDeformableBody->mPhysXScene->getInternalScene())
        {
            data = intDeformableBody->mPhysXScene->getInternalScene()->clampPosIterationCount(data);
        }

        deformableBody->setSolverIterationCounts(data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodyLinearDamping)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        deformableBody->setLinearDamping(data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodyMaxLinearVelocity)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        deformableBody->setMaxLinearVelocity(data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodySettlingDamping)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        deformableBody->setSettlingDamping(data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodySleepThreshold)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        deformableBody->setSleepThreshold(data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodySettlingThreshold)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        deformableBody->setSettlingThreshold(data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodyMaxDepenetrationVelocity)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        deformableBody->setMaxDepenetrationVelocity(data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodySelfCollision)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        deformableBody->setDeformableBodyFlag(PxDeformableBodyFlag::eDISABLE_SELF_COLLISION, !data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodySelfCollisionFilterDistance)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        attachedStage.getPhysXPhysicsInterface()->updateDeformableSelfCollisionFilterDistance(attachedStage, objectId, data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodyEnableSpeculativeCCD)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        deformableBody->setDeformableBodyFlag(PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD, data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodyDisableGravity)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        deformableBody->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodyCollisionPairUpdateFrequency)
    {
        uint32_t data;
        if (!getValue<uint32_t>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        if (deformableSurface)
            deformableSurface->setNbCollisionPairUpdatesPerTimestep(data);
    }
    else if (property == PhysxSchemaTokens->physxDeformableBodyCollisionIterationMultiplier)
    {
        uint32_t data;
        if (!getValue<uint32_t>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        if (deformableSurface)
            deformableSurface->setNbCollisionSubsteps(data);
    }

    return true;
}

bool omni::physx::updateDeformableRestOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        attachedStage.getPhysXPhysicsInterface()->updateDeformableRestOffset(attachedStage, objectId, data);
    }
    return true;
}

bool omni::physx::updateDeformableContactOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        attachedStage.getPhysXPhysicsInterface()->updateDeformableContactOffset(attachedStage, objectId, data);
    }
    return true;
}

// Resolve the current effective rest offset for a deformable prim, following the
// PhysX > Newton priority (mirrors parse-time logic in Collision.cpp).
// Time-sampled attributes are read at the supplied timeCode.
static float resolveDeformableRestOffset(const AttachedStage& attachedStage,
                                         omni::physics::parse::ObjectKey key,
                                         const PXR_NS::UsdTimeCode& timeCode)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return 0.0f;
    float v = 0.0f;
    if (src->hasAuthoredAttribute(key, src->internToken(PhysxSchemaTokens.Get()->physxCollisionRestOffset.GetString()))
        && getValue<float>(attachedStage, key, PhysxSchemaTokens.Get()->physxCollisionRestOffset, timeCode, v))
    {
        return v;
    }
    if (src->hasAuthoredAttribute(key, src->internToken(NewtonSchemaTokens->newtonContactMargin.GetString()))
        && getValue<float>(attachedStage, key, NewtonSchemaTokens->newtonContactMargin, timeCode, v))
    {
        if (v >= 0.0f)
            return v;
    }
    return 0.0f;
}

// Newton fallback: newton:contactMargin -> physxCollision:restOffset (deformable).
// When physxCollision:contactOffset is also fallback-driven, keep it = margin + gap
// in sync so the parse-time invariant survives runtime edits.
bool omni::physx::updateNewtonDeformableContactMargin(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    // Authored-value and Newton-fallback reads go through the source rather than
    // reaching into USD via the prim.
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::ObjectKey key = objectRecord->mKey;
    if (!src || !src->exists(key))
        return true;
    if (src->hasAuthoredAttribute(key, src->internToken(PhysxSchemaTokens.Get()->physxCollisionRestOffset.GetString())))
        return true;

    InternalDeformableBody* internalBody = (InternalDeformableBody*)objectRecord->mInternalPtr;
    if (!internalBody)
        return true;

    // Clamp negative margins (matches parse-time non-negative filtering in resolveDeformableRestOffset).
    float newMargin;
    if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, newMargin))
        return true;
    if (!isfinite(newMargin) || newMargin < 0.0f)
        return true;

    attachedStage.getPhysXPhysicsInterface()->updateDeformableRestOffset(attachedStage, objectId, newMargin);

    // Keep contact offset coupled when it is fallback-driven too.
    if (src->hasAuthoredAttribute(key, src->internToken(PhysxSchemaTokens.Get()->physxCollisionContactOffset.GetString())))
        return true;
    float gap = 0.0f;
    if (!src->hasAuthoredAttribute(key, src->internToken(NewtonSchemaTokens->newtonContactGap.GetString()))
        || !getValue<float>(attachedStage, key, NewtonSchemaTokens->newtonContactGap, PXR_NS::UsdTimeCode(), gap))
        return true;
    if (!isfinite(gap) || gap < 0.0f)
        return true;
    const float newContactOffset = newMargin + gap;
    attachedStage.getPhysXPhysicsInterface()->updateDeformableContactOffset(attachedStage, objectId, newContactOffset);
    return true;
}

// Newton fallback: newton:contactGap -> physxCollision:contactOffset (deformable),
// where contactOffset = restOffset + gap.
bool omni::physx::updateNewtonDeformableContactGap(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(objectRecord->mKey))
        return true;
    if (src->hasAuthoredAttribute(objectRecord->mKey,
                                  src->internToken(PhysxSchemaTokens.Get()->physxCollisionContactOffset.GetString())))
        return true;

    InternalDeformableBody* internalBody = (InternalDeformableBody*)objectRecord->mInternalPtr;
    if (!internalBody)
        return true;

    float gap;
    if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, gap))
        return true;
    if (!isfinite(gap) || gap < 0.0f)
        return true;

    const float restOffset = resolveDeformableRestOffset(attachedStage, objectRecord->mKey, timeCode);
    const float newContactOffset = restOffset + gap;
    attachedStage.getPhysXPhysicsInterface()->updateDeformableContactOffset(attachedStage, objectId, newContactOffset);
    return true;
}

bool omni::physx::updateDeformableMaterial(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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

    if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsDensity)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    else if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsDynamicFriction)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        material->setDynamicFriction(data);
        return true;
    }
    else if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsStaticFriction)
    {
        CARB_LOG_WARN("Static friction is not supported on deformable bodies, prim: %s", attachedStage.textFor(objectRecord->mKey));
    }
    else if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsYoungsModulus)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        material->setYoungsModulus(data);
        return true;
    }
    else if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsPoissonsRatio)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        material->setPoissons(data);
        return true;
    }
    else if (property == PhysxSchemaTokens->physxDeformableMaterialElasticityDamping)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        material->setElasticityDamping(data);
        return true;
    }
    else if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSurfaceThickness)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    else if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSurfaceStretchStiffness)
    {
        CARB_LOG_WARN("Surface Stretch Stiffness is currently not supported. Prim: %s",
            attachedStage.textFor(objectRecord->mKey));
    }
    else if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSurfaceShearStiffness)
    {
        CARB_LOG_WARN("Surface Shear Stiffness is currently not supported. Prim: %s",
            attachedStage.textFor(objectRecord->mKey));
    }
    else if (property == OmniUsdPhysicsDeformableSchemaTokens->omniphysicsSurfaceBendStiffness)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        if (surfaceMaterial)
            surfaceMaterial->setBendingStiffness(data);

        return true;
    }
    else if (property == PhysxSchemaTokens->physxDeformableMaterialBendDamping)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        if (surfaceMaterial)
            surfaceMaterial->setBendingDamping(data);

        return true;
    }
    return false;
}
