// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PhysXPropertiesUpdate.h"

#include <omni/physics/parse/KnownTokens.h>

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

// physx material
bool omni::physx::updateMaterialFrictionCombineMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        if (!source)
            return true;

        omni::physics::parse::TokenId data;
        if (!source->getAttribute(objectRecord->mKey, property, data))
            return true;

        omni::physics::parse::KnownTokens tok;
        tok.intern(*source);

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        if (tok.average == data)
            material->setFrictionCombineMode(PxCombineMode::eAVERAGE);
        else if (tok.min == data)
            material->setFrictionCombineMode(PxCombineMode::eMIN);
        else if (tok.max == data)
            material->setFrictionCombineMode(PxCombineMode::eMAX);
        else if (tok.multiply == data)
            material->setFrictionCombineMode(PxCombineMode::eMULTIPLY);
    }
    return true;
}

bool omni::physx::updateMaterialRestitutionCombineMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        if (!source)
            return true;

        omni::physics::parse::TokenId data;
        if (!source->getAttribute(objectRecord->mKey, property, data))
            return true;

        omni::physics::parse::KnownTokens tok;
        tok.intern(*source);

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        if (tok.average == data)
            material->setRestitutionCombineMode(PxCombineMode::eAVERAGE);
        else if (tok.min == data)
            material->setRestitutionCombineMode(PxCombineMode::eMIN);
        else if (tok.max == data)
            material->setRestitutionCombineMode(PxCombineMode::eMAX);
        else if (tok.multiply == data)
            material->setRestitutionCombineMode(PxCombineMode::eMULTIPLY);
    }
    return true;
}

bool omni::physx::updateMaterialDampingCombineMode(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        if (!source)
            return true;

        omni::physics::parse::TokenId data;
        if (!source->getAttribute(objectRecord->mKey, property, data))
            return true;

        omni::physics::parse::KnownTokens tok;
        tok.intern(*source);

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        if (tok.average == data)
            material->setDampingCombineMode(PxCombineMode::eAVERAGE);
        else if (tok.min == data)
            material->setDampingCombineMode(PxCombineMode::eMIN);
        else if (tok.max == data)
            material->setDampingCombineMode(PxCombineMode::eMAX);
        else if (tok.multiply == data)
            material->setDampingCombineMode(PxCombineMode::eMULTIPLY);
    }
    return true;
}

bool omni::physx::updateCompliantMaterial(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
                                          omni::physics::parse::TokenId property,
                                          omni::physics::parse::ReadTime timeCode)
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
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.compliantContactStiffness)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                if (data > 0.0f)
                {
                    material->setRestitution(-data);  // negative restitution is interpreted as compliant stiffness
                }
                else
                {
                    // disable compliance and restore restitution from USD:
                    float restitution = 0.0f;
                    getValue<float>(attachedStage, objectRecord->mKey, tok.restitution, timeCode, restitution);
                    material->setRestitution(restitution);
                }
            }
            else if (property == tok.compliantContactDamping)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                if(material->getRestitution() >= 0.0f) // restitution < 0 implies compliant contact behavior
                {
                    CARB_LOG_WARN(
                        "Updating compliant contact damping on material %s, but compliant stiffness is zero. Set stiffness >0 first to enable compliance.",
                        attachedStage.textFor(objectRecord->mKey));
                    return true;
                }
                material->setDamping(data);
            }
            else if (property == tok.compliantContactAccelerationSpring)
            {
                bool data;
                if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                if(material->getRestitution() >= 0.0f) // restitution < 0 implies compliant contact behavior
                {
                    CARB_LOG_WARN(
                        "Updating compliant contact acceleration spring on material %s, but compliant stiffness is zero. Set stiffness >0 first to enable compliance.",
                        attachedStage.textFor(objectRecord->mKey));
                    return true;
                }
                material->setFlag(PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING, data);
            }
        }
    }
    return true;
}
