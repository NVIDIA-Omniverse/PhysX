// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"
#include "../usdLoad/NewtonCompat.h"

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace PXR_NS;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// shape
bool omni::physx::updateShapeEnabled(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
            if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
            if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
                return true;

            for(auto shape : cShape->getShapes()) {
                setCollisionsEnabledForShape(shape, data);
            }
        }
    }
    return true;
}


bool omni::physx::updateShapeDensity(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;        

        if (data >= 0.0f && data > shape->getRestOffset())
        {
            shape->setContactOffset(data);
        }
        else
        {
            CARB_LOG_ERROR("Collision contact offset must be positive and greater then restOffset, prim: %s", attachedStage.textFor(objectRecord->mKey));
        }
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
                CARB_LOG_ERROR("Collision contact offset must be positive and greater then restOffset, prim: %s", attachedStage.textFor(objectRecord->mKey));
            }
        }
    }

    return true;
}

bool omni::physx::updateShapeRestOffset(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        if (isfinite(data) && data < shape->getContactOffset())
        {
            shape->setRestOffset(data);
        }
        else
        {
            CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", attachedStage.textFor(objectRecord->mKey));
        }
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
                CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", attachedStage.textFor(objectRecord->mKey));
            }
        }
    }

    return true;
}

// Read the current effective rest offset from a PxShape / PhysXCompoundShape record.
// Returns true if a value was retrieved.
static bool getCurrentRestOffset(const InternalDatabase::Record* objectRecord,
                                 PhysXType internalType, float& outRestOffset)
{
    if (internalType == ePTShape)
    {
        PxShape* shape = (PxShape*)objectRecord->mPtr;
        outRestOffset = shape->getRestOffset();
        return true;
    }
    if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        if (!cShape->getShapes().empty())
        {
            outRestOffset = cShape->getShapes()[0]->getRestOffset();
            return true;
        }
    }
    return false;
}

// Newton fallback: newton:contactMargin -> physxCollision:restOffset.
//
// When physxCollision:contactOffset is *also* fallback-driven (i.e. unauthored and
// newton:contactGap is the effective source), the contact offset must move with the
// margin so the parse-time invariant `contactOffset = restOffset + gap` stays true
// at runtime — otherwise PxShape can reject the new rest because the stale contact
// offset is no longer greater than it.
bool omni::physx::updateNewtonShapeContactMargin(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    // PhysX wins: skip when physxCollision:restOffset is authored. Authored-value
    // and Newton-fallback reads go through the source (hasAuthoredAttribute +
    // getValue) rather than reaching into USD via the prim.
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::ObjectKey key = objectRecord->mKey;
    if (!src || !src->exists(key))
        return true;
    if (src->hasAuthoredAttribute(key, src->internToken(PhysxSchemaTokens.Get()->physxCollisionRestOffset.GetString())))
        return true;

    // Is the contact offset also fallback-driven? If so we need to keep it = margin + gap.
    const bool physxContactAuthored =
        src->hasAuthoredAttribute(key, src->internToken(PhysxSchemaTokens.Get()->physxCollisionContactOffset.GetString()));
    float gap = 0.0f;
    const bool hasGapFallback = !physxContactAuthored
        && src->hasAuthoredAttribute(key, src->internToken(NewtonSchemaTokens->newtonContactGap.GetString()))
        && getValue<float>(attachedStage, key, NewtonSchemaTokens->newtonContactGap, PXR_NS::UsdTimeCode(), gap)
        && isfinite(gap) && gap >= 0.0f;
    if (!hasGapFallback)
    {
        // No coupled contactOffset to maintain — plain rest-offset path.
        return updateShapeRestOffset(attachedStage, objectId, property, timeCode);
    }

    float newRest;
    if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, newRest))
        return true;
    if (!isfinite(newRest) || newRest < 0.0f)
        return true;
    const float newContact = newRest + gap;

    // Apply rest+contact in an order that always preserves rest < contact:
    // grow contact first if it needs to grow, then set rest; otherwise set rest first.
    auto applyShape = [&](PxShape* shape) {
        const bool contactGrowsFirst = (newContact > shape->getContactOffset());
        if (contactGrowsFirst)
        {
            if (newContact > shape->getRestOffset())
                shape->setContactOffset(newContact);
        }
        if (newRest < shape->getContactOffset())
            shape->setRestOffset(newRest);
        else
            CARB_LOG_ERROR("Collision rest offset must be lesser then contact offset, prim: %s", attachedStage.textFor(objectRecord->mKey));
        if (!contactGrowsFirst)
        {
            if (newContact > shape->getRestOffset())
                shape->setContactOffset(newContact);
            else
                CARB_LOG_ERROR("Collision contact offset must be greater than restOffset, prim: %s", attachedStage.textFor(objectRecord->mKey));
        }
    };

    if (internalType == ePTShape)
    {
        applyShape((PxShape*)objectRecord->mPtr);
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        for (auto shape : cShape->getShapes())
            applyShape(shape);
    }
    return true;
}

// Newton fallback: newton:contactGap -> physxCollision:contactOffset, where
// contactOffset = restOffset + gap (Newton's gap is additive on top of the margin).
bool omni::physx::updateNewtonShapeContactGap(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    // PhysX wins: skip when physxCollision:contactOffset is authored.
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(objectRecord->mKey))
        return true;
    if (src->hasAuthoredAttribute(objectRecord->mKey,
                                  src->internToken(PhysxSchemaTokens.Get()->physxCollisionContactOffset.GetString())))
        return true;

    float gap;
    if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, gap))
        return true;
    // Newton sentinel for "use default".
    if (!isfinite(gap) || gap < 0.0f)
        return true;

    float restOffset = 0.0f;
    if (!getCurrentRestOffset(objectRecord, internalType, restOffset))
        return true;
    const float newContactOffset = restOffset + gap;

    if (internalType == ePTShape)
    {
        PxShape* shape = (PxShape*)objectRecord->mPtr;
        if (newContactOffset > shape->getRestOffset())
            shape->setContactOffset(newContactOffset);
        else
            CARB_LOG_ERROR("Collision contact offset must be greater than restOffset, prim: %s", attachedStage.textFor(objectRecord->mKey));
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        if (!cShape->getShapes().empty() && newContactOffset > cShape->getShapes()[0]->getRestOffset())
        {
            for (auto shape : cShape->getShapes())
                shape->setContactOffset(newContactOffset);
        }
        else
        {
            CARB_LOG_ERROR("Collision contact offset must be greater than restOffset, prim: %s", attachedStage.textFor(objectRecord->mKey));
        }
    }
    return true;
}

bool omni::physx::updateShapeTorsionalPatchRadius(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        shape->setTorsionalPatchRadius(data);
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        for(auto shape : cShape->getShapes()) {
            shape->setTorsionalPatchRadius(data);
        }
    }

    return true;
}

bool omni::physx::updateShapeMinTorsionalPatchRadius(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        shape->setMinTorsionalPatchRadius(data);
    }
    else if (internalType == ePTCompoundShape)
    {
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)objectRecord->mPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        for(auto shape : cShape->getShapes()) {
            shape->setMinTorsionalPatchRadius(data);
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// material
bool omni::physx::updateMaterialDynamicFriction(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    float data;
    if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
        return true;

    if (internalType == ePTMaterial)
    {
        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        material->setDynamicFriction(data);
    }

    return true;
}

bool omni::physx::updateMaterialStaticFriction(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        material->setStaticFriction(data);
    }
    else if (internalType == ePTDeformableVolumeMaterial || internalType == ePTDeformableSurfaceMaterial)
    {
        CARB_LOG_WARN("Static friction is not supported on deformable bodies, prim: %s", attachedStage.textFor(objectRecord->mKey));
    }
    return true;
}

bool omni::physx::updateMaterialRestitution(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxMaterial* material = (PxMaterial*)objectRecord->mPtr;
        material->setRestitution(data);
    }
    return true;
}
