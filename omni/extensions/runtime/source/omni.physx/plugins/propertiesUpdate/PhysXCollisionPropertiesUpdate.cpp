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

////////////////////////////////////////////////////////////////////////////////////////////////////////
// shape
bool omni::physx::updateShapeEnabled(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;


    auto setCollisionsEnabledForShape = [&omniPhysX](PxShape* shape, bool enabled) {
        if (!shape)
            return;

        shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, enabled);
        shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, enabled);
        if (enabled && omniPhysX.isDebugVisualizationEnabled()) {
            shape->setFlag(PxShapeFlag::eVISUALIZATION, true);
        } else {
            shape->setFlag(PxShapeFlag::eVISUALIZATION, false);
        }

        PxRigidActor* actor = shape->getActor();
        if (actor && actor->getScene()) {
            actor->getScene()->resetFiltering(*actor);
        }
    };

    if (internalType == ePTShape)
    {
        PxShape* shape = (PxShape*)objectRecord->mPtr;
        if (shape && shape->isExclusive())
        {
            bool data;
            if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
                return true;

            setCollisionsEnabledForShape(shape, data);
        }
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        if (cShape && !cShape->getShapes().empty())
        {
            bool data;
            if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
                return true;

            for(auto shape : cShape->getShapes()) {
                setCollisionsEnabledForShape(shape, data);
            }
        }
    }
    return true;
}


bool omni::physx::updateShapeDensity(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTShape)
    {
        const PxShape* shape = (const PxShape*)objectRecord->mPtr;
        const PxRigidActor* actor = shape->getActor();
        if(actor)
        {
            db.addDirtyMassActor(size_t(actor->userData));
        };
    }
    return true;
}

// physx shape
bool omni::physx::updateShapeContactOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTShape)
    {
        PxShape* shape = (PxShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;        

        if (data >= 0.0f && data > shape->getRestOffset())
        {
            shape->setContactOffset(data);
        }
        else
        {
            CARB_LOG_ERROR("Collision contact offset must be positive and greater then restOffset, prim: %s", objectRecord->mPath.GetText());
        }
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (!cShape->getShapes().empty())
        {
            if (data >= 0.0f && data > cShape->getShapes()[0]->getRestOffset())
            {
                for (auto shape : cShape->getShapes()) {
                    shape->setContactOffset(data);
                }
            }
            else
            {
                CARB_LOG_ERROR("Collision contact offset must be positive and greater then restOffset, prim: %s", objectRecord->mPath.GetText());
            }
        }
    }

    return true;
}

bool omni::physx::updateShapeRestOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTShape)
    {
        PxShape* shape = (PxShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (isfinite(data) && data < shape->getContactOffset())
        {
            shape->setRestOffset(data);
        }
        else
        {
            CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", objectRecord->mPath.GetText());
        }
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (!cShape->getShapes().empty())
        {
            if (isfinite(data) && data < cShape->getShapes()[0]->getContactOffset())
            {
                for (auto shape : cShape->getShapes()) {
                    shape->setRestOffset(data);
                }
            }
            else
            {
                CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", objectRecord->mPath.GetText());
            }
        }
    }

    return true;
}

bool omni::physx::updateShapeTorsionalPatchRadius(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTShape)
    {
        PxShape* shape = (PxShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        shape->setTorsionalPatchRadius(data);
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        for(auto shape : cShape->getShapes()) {
            shape->setTorsionalPatchRadius(data);
        }
    }

    return true;
}

bool omni::physx::updateShapeMinTorsionalPatchRadius(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTShape)
    {
        PxShape* shape = (PxShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        shape->setMinTorsionalPatchRadius(data);
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        for(auto shape : cShape->getShapes()) {
            shape->setMinTorsionalPatchRadius(data);
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// material
bool omni::physx::updateMaterialDynamicFriction(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    float data;
    if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
        return true;

    if (internalType == ePTMaterial)
    {
        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        material->setDynamicFriction(data);
    }

    return true;
}

bool omni::physx::updateMaterialStaticFriction(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        material->setStaticFriction(data);
    }
    else if (internalType == ePTDeformableVolumeMaterial || internalType == ePTDeformableSurfaceMaterial)
    {
        CARB_LOG_WARN("Static friction is not supported on deformable bodies, prim: %s", objectRecord->mPath.GetText());
    }
    return true;
}

bool omni::physx::updateMaterialRestitution(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTMaterial)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        material->setRestitution(data);
    }
    return true;
}
