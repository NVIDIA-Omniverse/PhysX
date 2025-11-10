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
bool omni::physx::updateGravityMagnitude(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        PxScene* scene = (PxScene*)objectRecord->mPtr;
        InternalScene* intScene = (InternalScene*)objectRecord->mInternalPtr;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;
        PxVec3 gravity = intScene->mGravityDirection;
        gravity.normalize();
        if (data < -0.5e38f)
        {
            float metersPerUnit = (float)pxr::UsdGeomGetStageMetersPerUnit(OmniPhysX::getInstance().getStage());
            data = 9.81f / metersPerUnit;
        }
        intScene->mGravityMagnitude = data;
        gravity *= data;
        scene->setGravity(gravity);
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omni::physx::updateGravityDirection(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        PxScene* scene = (PxScene*)objectRecord->mPtr;
        InternalScene* intScene = (InternalScene*)objectRecord->mInternalPtr;
        GfVec3f data;
        if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;
        const float magn = intScene->mGravityMagnitude;
        if (data.GetLengthSq() < 0.001f)
        {
            TfToken upAxis = pxr::UsdGeomGetStageUpAxis(OmniPhysX::getInstance().getStage());
            if (upAxis == pxr::UsdGeomTokens.Get()->x)
                data = GfVec3f(-1.0f, 0.0f, 0.0f);
            else if (upAxis == pxr::UsdGeomTokens.Get()->y)
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
bool omni::physx::updateTimeStepsPerSecond(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        uint32_t data;
        if (!getValue<uint32_t>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        OmniPhysX& omniPhysX = OmniPhysX::getInstance();        
        PhysXScene* scene = omniPhysX.getPhysXSetup().getPhysXScene(size_t(objectId));

        if (data > 0 && scene)
            scene->setTimeStepsPerSeconds(data);
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omni::physx::updateSceneUpdateType(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        TfToken data;
        if (!getValue<TfToken>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        PhysXScene* scene = omniPhysX.getPhysXSetup().getPhysXScene(size_t(objectId));
        if (scene && PhysxSchemaTokens.Get()->Asynchronous == data)
            scene->setUpdateType(Asynchronous);
        else if (scene && PhysxSchemaTokens.Get()->Synchronous == data)
            scene->setUpdateType(Synchronous);
        else if (scene && PhysxSchemaTokens.Get()->Disabled == data)
            scene->setUpdateType(Disabled);
    }
    return true;
}

bool omni::physx::updateQuasistaticEnabled(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        if (objectRecord->mType == ePTScene)
        {
            bool data;
            if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
                return true;

            InternalScene* sc = (InternalScene*)objectRecord->mInternalPtr;
            sc->getSceneDesc().enableQuasistatic = data;
        }
    }
    return true;
}

bool omni::physx::updateQuasistaticCollection(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        if (objectRecord->mType == ePTScene)
        {
            const PhysxSchemaPhysxSceneQuasistaticAPI physxSceneQuasistaticAPI = PhysxSchemaPhysxSceneQuasistaticAPI::Get(attachedStage.getStage(), objectRecord->mPath);
            if (physxSceneQuasistaticAPI)
            {
                InternalScene* sc = (InternalScene*)objectRecord->mInternalPtr;
                const UsdCollectionAPI collection = physxSceneQuasistaticAPI.GetQuasistaticActorsCollectionAPI();
                SdfPathVector targets;
                collection.GetIncludesRel().GetTargets(&targets);
                if (!targets.empty())
                {
                    const UsdCollectionMembershipQuery query = collection.ComputeMembershipQuery();
                    const SdfPathSet collectionPaths = UsdCollectionAPI::ComputeIncludedPaths(query,
                        attachedStage.getStage(), UsdTraverseInstanceProxies());
                    sc->getSceneDesc().quasistaticActors = collectionPaths;
                }
                else
                {
                    sc->getSceneDesc().quasistaticActors.clear();
                }
            }            
        }
    }
    return true;
}
