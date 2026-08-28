// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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
static const InternalDatabase::Record* getObjectRecord(ObjectId objectId)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    return internalType == ePTScene ? objectRecord : nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// scene
bool omni::physx::updateGravityMagnitude(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        PxScene* scene = (PxScene*)objectRecord->mPtr;
        InternalScene* intScene = (InternalScene*)objectRecord->mInternalPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;
        PxVec3 gravity = intScene->mGravityDirection;
        gravity.normalize();
        if (data < -0.5e38f)
        {
            const float metersPerUnit = attachedStage.getSource()->getSourceUnits().metersPerUnit;
            data = 9.81f / metersPerUnit;
        }
        intScene->mGravityMagnitude = data;
        gravity *= data;
        scene->setGravity(gravity);
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omni::physx::updateGravityDirection(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        PxScene* scene = (PxScene*)objectRecord->mPtr;
        InternalScene* intScene = (InternalScene*)objectRecord->mInternalPtr;
        GfVec3f data;
        if (!getValue<GfVec3f>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;
        const float magn = intScene->mGravityMagnitude;
        if (data.GetLengthSq() < 0.001f)
        {
            // USD up-axis is Y or Z only (UsdGeomGetStageUpAxis never returns X),
            // which is exactly what SourceUnits::upAxis encodes.
            const omni::physics::parse::UpAxis upAxis = attachedStage.getSource()->getSourceUnits().upAxis;
            if (upAxis == omni::physics::parse::UpAxis::eY)
                data = GfVec3f(0.0f, -1.0f, 0.0f);
            else
                data = GfVec3f(0.0f, 0.0f, -1.0f);
        }
        intScene->mGravityDirection = toPhysX(data);
        data = data * magn;
        scene->setGravity(PxVec3(data[0], data[1], data[2]));
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omni::physx::updateTimeStepsPerSecond(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        uint32_t data;
        if (!getValue<uint32_t>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        PhysXScene* scene = omniPhysX.getPhysXSetup().getPhysXScene(size_t(objectId));

        if (data > 0 && scene)
            scene->setTimeStepsPerSeconds(data);
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Newton fallback: newton:timeStepsPerSecond -> physxScene:timeStepsPerSecond
bool omni::physx::updateNewtonTimeStepsPerSecond(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        // PhysX wins: skip when physxScene:timeStepsPerSecond is authored.
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        if (!src || !src->exists(objectRecord->mKey))
            return true;
        if (src->hasAuthoredAttribute(objectRecord->mKey,
                                      src->internToken(PhysxSchemaTokens.Get()->physxSceneTimeStepsPerSecond.GetString())))
            return true;

        // Newton schema declares newton:timeStepsPerSecond as int — read it as int and
        // promote to uint32_t. Reading directly as uint32_t would fail on the typed attr.
        int data;
        if (!getValue<int>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PhysXScene* scene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(size_t(objectId));
        if (data > 0 && scene)
            scene->setTimeStepsPerSeconds(static_cast<uint32_t>(data));
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Newton: newton:gravityEnabled — when false, gravity magnitude is zeroed.
// When flipped back to true, the magnitude is re-resolved from physicsGravityMagnitude.
bool omni::physx::updateNewtonGravityEnabled(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        // PhysX wins: skip when physics:gravityMagnitude is authored.
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        if (!src || !src->exists(objectRecord->mKey))
            return true;
        if (src->hasAuthoredAttribute(objectRecord->mKey,
                                      src->internToken(UsdPhysicsTokens.Get()->physicsGravityMagnitude.GetString())))
            return true;

        PxScene* scene = (PxScene*)objectRecord->mPtr;
        InternalScene* intScene = (InternalScene*)objectRecord->mInternalPtr;
        if (!scene || !intScene)
            return true;

        bool gravityEnabled = true;
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, gravityEnabled))
            return true;

        if (!gravityEnabled)
        {
            intScene->mGravityMagnitude = 0.0f;
            PxVec3 dir = intScene->mGravityDirection;
            dir.normalize();
            scene->setGravity(dir * 0.0f);
        }
        else
        {
            // Re-apply the USD-authored gravity magnitude via the standard PhysX path.
            updateGravityMagnitude(attachedStage, objectId, UsdPhysicsTokens.Get()->physicsGravityMagnitude, timeCode);
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omni::physx::updateSceneUpdateType(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        TfToken data;
        if (!getValue<TfToken>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        PhysXScene* scene = omniPhysX.getPhysXSetup().getPhysXScene(size_t(objectId));
        if (scene && PhysxSchemaTokens.Get()->Asynchronous == data)
            scene->setUpdateType(eAsynchronous);
        else if (scene && PhysxSchemaTokens.Get()->Synchronous == data)
            scene->setUpdateType(eSynchronous);
        else if (scene && PhysxSchemaTokens.Get()->Disabled == data)
            scene->setUpdateType(eDisabled);
    }
    return true;
}

bool omni::physx::updateQuasistaticEnabled(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        if (objectRecord->mType == ePTScene)
        {
            bool data;
            if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
                return true;

            InternalScene* sc = (InternalScene*)objectRecord->mInternalPtr;
            sc->getSceneDesc().enableQuasistatic = data;
        }
    }
    return true;
}

bool omni::physx::updateQuasistaticCollection(AttachedStage& attachedStage, ObjectId objectId, const PXR_NS::TfToken& property, const PXR_NS::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        if (objectRecord->mType == ePTScene)
        {
            const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
            // Gate on the PhysxSceneQuasistaticAPI being applied; when absent,
            // leave the existing quasistaticActors set untouched.
            if (src && src->hasSchema(objectRecord->mKey, src->internToken("PhysxSceneQuasistaticAPI")))
            {
                InternalScene* sc = (InternalScene*)objectRecord->mInternalPtr;
                // The QuasistaticActorsCollectionAPI instance name is
                // PhysxSchemaTokens->quasistaticactors ("quasistaticactors").
                // resolveCollection performs the include/exclude membership walk
                // internally; an empty result clears the set.
                std::vector<omni::physics::parse::ObjectKey> members;
                src->resolveCollection(objectRecord->mKey, src->internToken("quasistaticactors"), members);
                std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>& qActors =
                    sc->getSceneDesc().quasistaticActors;
                qActors.clear();
                qActors.reserve(members.size());
                for (const omni::physics::parse::ObjectKey& k : members)
                    qActors.insert(k);
            }
        }
    }
    return true;
}
