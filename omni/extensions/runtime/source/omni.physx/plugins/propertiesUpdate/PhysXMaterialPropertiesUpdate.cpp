// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

// physx material
bool omni::physx::updateMaterialFrictionCombineMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        TfToken data;
        if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        if (PhysxSchemaTokens.Get()->average == data)
            material->setFrictionCombineMode(PxCombineMode::eAVERAGE);
        else if (PhysxSchemaTokens.Get()->min == data)
            material->setFrictionCombineMode(PxCombineMode::eMIN);
        else if (PhysxSchemaTokens.Get()->max == data)
            material->setFrictionCombineMode(PxCombineMode::eMAX);
        else if (PhysxSchemaTokens.Get()->multiply == data)
            material->setFrictionCombineMode(PxCombineMode::eMULTIPLY);
    }
    return true;
}

bool omni::physx::updateMaterialRestitutionCombineMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        TfToken data;
        if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        if (PhysxSchemaTokens.Get()->average == data)
            material->setRestitutionCombineMode(PxCombineMode::eAVERAGE);
        else if (PhysxSchemaTokens.Get()->min == data)
            material->setRestitutionCombineMode(PxCombineMode::eMIN);
        else if (PhysxSchemaTokens.Get()->max == data)
            material->setRestitutionCombineMode(PxCombineMode::eMAX);
        else if (PhysxSchemaTokens.Get()->multiply == data)
            material->setRestitutionCombineMode(PxCombineMode::eMULTIPLY);
    }
    return true;
}

bool omni::physx::updateMaterialDampingCombineMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        TfToken data;
        if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        if (PhysxSchemaTokens.Get()->average == data)
            material->setDampingCombineMode(PxCombineMode::eAVERAGE);
        else if (PhysxSchemaTokens.Get()->min == data)
            material->setDampingCombineMode(PxCombineMode::eMIN);
        else if (PhysxSchemaTokens.Get()->max == data)
            material->setDampingCombineMode(PxCombineMode::eMAX);
        else if (PhysxSchemaTokens.Get()->multiply == data)
            material->setDampingCombineMode(PxCombineMode::eMULTIPLY);
    }
    return true;
}

bool omni::physx::updateCompliantMaterial(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
                                          const pxr::TfToken& property,
                                          const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        PxMaterial* material = reinterpret_cast<PxMaterial*>(objectRecord->mPtr);
        if (material)
        {
            if (property == PhysxSchemaTokens.Get()->physxMaterialCompliantContactStiffness)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                if (data > 0.0f)
                {
                    material->setRestitution(-data);  // negative restitution is interpreted as compliant stiffness
                }
                else
                {
                    // disable compliance and restore restitution from USD:
                    float restitution = 0.0f;  // fallback to 0.0
                    getValue<float>(attachedStage, objectRecord->mPath, UsdPhysicsTokens.Get()->physicsRestitution, timeCode, restitution);
                    material->setRestitution(restitution);
                }
            }
            else if (property == PhysxSchemaTokens.Get()->physxMaterialCompliantContactDamping)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                if(material->getRestitution() >= 0.0f) // restitution < 0 implies compliant contact behavior
                {
                    CARB_LOG_WARN(
                        "Updating compliant contact damping on material %s, but compliant stiffness is zero. Set stiffness >0 first to enable compliance.",
                        objectRecord->mPath.GetText());
                    return true;
                }
                material->setDamping(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxMaterialCompliantContactAccelerationSpring)
            {
                bool data;
                if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                if(material->getRestitution() >= 0.0f) // restitution < 0 implies compliant contact behavior
                {
                    CARB_LOG_WARN(
                        "Updating compliant contact acceleration spring on material %s, but compliant stiffness is zero. Set stiffness >0 first to enable compliance.",
                        objectRecord->mPath.GetText());
                    return true;
                }
                material->setFlag(PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING, data);
            }
        }
    }
    return true;
}
