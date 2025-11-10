// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <internal/Internal.h>
#include <internal/InternalScene.h>
#include <internal/InternalDeformableDeprecated.h>
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

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Deformable body material
bool omni::physx::updateDeformableBodyMaterialDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTSoftBodyMaterialDeprecated)
    {
        PxFEMSoftBodyMaterial* physxDeformableMaterial = reinterpret_cast<PxFEMSoftBodyMaterial*>(objectRecord->mPtr);
        if (physxDeformableMaterial)
        {
            if (property == PhysxSchemaTokens.Get()->physxDeformableBodyMaterialDensity)
            {
                float materialDensity;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, materialDensity))
                    return true;

                InternalDeformableMaterial* intMat = (InternalDeformableMaterial*)objectRecord->mInternalPtr;
                intMat->mDensity = materialDensity;

                for (size_t i = 0; i < intMat->mDeformableIds.size(); i++)
                {
                    OmniPhysX::getInstance().getInternalPhysXDatabase().addDirtyMassDeformableBodyDeprecated(intMat->mDeformableIds[i]);
                }
            }
            else if (property == PhysxSchemaTokens.Get()->physxDeformableBodyMaterialElasticityDamping)
            {
                float elasticityDamping;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, elasticityDamping))
                    return true;
                physxDeformableMaterial->setDamping(elasticityDamping);
            }
            else if (property == PhysxSchemaTokens.Get()->physxDeformableBodyMaterialDampingScale)
            {
                float dampingScale;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, dampingScale))
                    return true;
                physxDeformableMaterial->setDampingScale(dampingScale);
            }
            else if (property == PhysxSchemaTokens.Get()->physxDeformableBodyMaterialYoungsModulus)
            {
                float youngsModulus;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, youngsModulus))
                    return true;
                physxDeformableMaterial->setYoungsModulus(youngsModulus);
            }
            else if (property == PhysxSchemaTokens.Get()->physxDeformableBodyMaterialPoissonsRatio)
            {
                float poissonsRatio;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, poissonsRatio))
                    return true;
                physxDeformableMaterial->setPoissons(poissonsRatio);
            }
            else if (property == PhysxSchemaTokens.Get()->physxDeformableBodyMaterialDynamicFriction)
            {
                float dynamicFriction;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, dynamicFriction))
                    return true;
                physxDeformableMaterial->setDynamicFriction(dynamicFriction);
            }
        }
    }

    return true;
}

bool omni::physx::updateDeformableBodyDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    if (property == UsdPhysicsTokens.Get()->physicsMass ||
        property == UsdPhysicsTokens.Get()->physicsDensity ||
        property == PhysxSchemaTokens.Get()->physxDeformableSurfaceMaterialDensity)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        db.addDirtyMassDeformableBodyDeprecated(size_t(objectId));
    }
    else if (property == PhysxSchemaTokens.Get()->physxDeformableSimulationVelocities)
    {
        attachedStage.getPhysXPhysicsInterface()->updateDeformableBodyVelocitiesDeprecated(attachedStage, objectId);
    }
    else if (property == PhysxSchemaTokens.Get()->physxDeformableSimulationPoints)
    {
        attachedStage.getPhysXPhysicsInterface()->updateDeformableBodyPositionsDeprecated(attachedStage, objectId);
    }

    return true;
}

bool omni::physx::warnDeformableBodyNoRuntimeCookingDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    PhysXType internalType = ePTSoftBodyDeprecated;
    const InternalDeformableBodyDeprecated* internalDeformableBody = reinterpret_cast<const InternalDeformableBodyDeprecated*>(db.getInternalTypedRecord(internalType, objectId));
    if (internalDeformableBody && internalDeformableBody->mPrim)
    {
        CARB_LOG_WARN("Changing deformable body mesh or meshing related PhysxSchemaPhysxDeformableBodyAPI parameter during simulation is not supported. Prim: %s",
            internalDeformableBody->mPrim.GetPath().GetText());
    }
    return true;
}

// Deformable surface material
bool omni::physx::updateDeformableSurfaceMaterialDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;

    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);

    if (!objectRecord)
        return true;

    static TfToken bendDampingToken("physxDeformableSurfaceMaterial:bendDamping");
    static TfToken elasticityDampingToken("physxDeformableSurfaceMaterial:elasticityDamping");
    static TfToken bendStiffnessToken("physxDeformableSurfaceMaterial:bendStiffness");

    if (internalType == ePTFEMClothMaterialDeprecated)
    {
        PxDeformableSurfaceMaterial* physxDeformableMaterial = reinterpret_cast<PxDeformableSurfaceMaterial*>(objectRecord->mPtr);

        if (physxDeformableMaterial)
        {
            if (property == PhysxSchemaTokens.Get()->physxDeformableSurfaceMaterialDensity)
            {
                float materialDensity;

                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, materialDensity))
                    return true;

                InternalDeformableMaterial* intMat = (InternalDeformableMaterial*)objectRecord->mInternalPtr;
                intMat->mDensity = materialDensity;

                for (size_t i = 0; i < intMat->mDeformableIds.size(); i++)
                {
                    OmniPhysX::getInstance().getInternalPhysXDatabase().addDirtyMassDeformableSurfaceDeprecated(intMat->mDeformableIds[i]);
                }
            }
            else if (property == PhysxSchemaTokens.Get()->physxDeformableSurfaceMaterialThickness)
            {
                float thickness;

                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, thickness))
                    return true;

                physxDeformableMaterial->setThickness(thickness);
            }
            else if (property == PhysxSchemaTokens.Get()->physxDeformableSurfaceMaterialYoungsModulus)
            {
                float youngsModulus;

                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, youngsModulus))
                    return true;

                physxDeformableMaterial->setYoungsModulus(youngsModulus);
            }
            else if (property == PhysxSchemaTokens.Get()->physxDeformableSurfaceMaterialPoissonsRatio)
            {
                float poissonsRatio;

                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, poissonsRatio))
                    return true;

                physxDeformableMaterial->setPoissons(poissonsRatio);
            }
            else if (property == PhysxSchemaTokens.Get()->physxDeformableSurfaceMaterialDynamicFriction)
            {
                float dynamicFriction;

                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, dynamicFriction))
                    return true;

                physxDeformableMaterial->setDynamicFriction(dynamicFriction);
            }
            else if (property == bendStiffnessToken)
            {
                float bendStiffness;

                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, bendStiffness))
                    return true;

                physxDeformableMaterial->setBendingStiffness(bendStiffness);
            }
            else if (property == bendDampingToken)
            {
                float bendDamping;

                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, bendDamping))
                    return true;

                physxDeformableMaterial->setBendingDamping(bendDamping);
            }
            else if (property == elasticityDampingToken)
            {
                float elasticityDamping;

                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, elasticityDamping))
                    return true;

                physxDeformableMaterial->setElasticityDamping(elasticityDamping);
            }
        }
    }

    return true;
}

bool omni::physx::updateDeformableSurfaceDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    if (property == UsdPhysicsTokens.Get()->physicsMass ||
        property == UsdPhysicsTokens.Get()->physicsDensity ||
        property == PhysxSchemaTokens.Get()->physxDeformableSurfaceMaterialDensity)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        db.addDirtyMassDeformableSurfaceDeprecated(size_t(objectId));
    }
    else if (property == PhysxSchemaTokens.Get()->physxDeformableSimulationVelocities)
    {
        attachedStage.getPhysXPhysicsInterface()->updateDeformableSurfaceVelocitiesDeprecated(attachedStage, objectId);
    }
    else if (property == UsdGeomTokens.Get()->points)
    {
        attachedStage.getPhysXPhysicsInterface()->updateDeformableSurfacePositionsDeprecated(attachedStage, objectId);
    }

    return true;
}

