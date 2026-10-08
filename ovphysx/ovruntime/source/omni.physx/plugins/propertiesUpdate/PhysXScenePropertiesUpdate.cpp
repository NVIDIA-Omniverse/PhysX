// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CORE-003
 * @covers AC-2
 */

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
bool omni::physx::updateGravityMagnitude(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
bool omni::physx::updateGravityDirection(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        PxScene* scene = (PxScene*)objectRecord->mPtr;
        InternalScene* intScene = (InternalScene*)objectRecord->mInternalPtr;
        carb::Float3 data;
        if (!getValue<carb::Float3>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;
        PxVec3 direction = toPhysX(data);
        const float magn = intScene->mGravityMagnitude;
        if (direction.magnitudeSquared() < 0.001f)
        {
            // USD up-axis is Y or Z only (UsdGeomGetStageUpAxis never returns X),
            // which is exactly what SourceUnits::upAxis encodes.
            const omni::physics::parse::UpAxis upAxis = attachedStage.getSource()->getSourceUnits().upAxis;
            if (upAxis == omni::physics::parse::UpAxis::eY)
                direction = PxVec3(0.0f, -1.0f, 0.0f);
            else
                direction = PxVec3(0.0f, 0.0f, -1.0f);
        }
        intScene->mGravityDirection = direction;
        scene->setGravity(direction * magn);
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omni::physx::updateTimeStepsPerSecond(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
bool omni::physx::updateNewtonTimeStepsPerSecond(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        // PhysX wins: skip when physxScene:timeStepsPerSecond is authored.
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        if (!src || !src->exists(objectRecord->mKey))
            return true;
        omni::physics::parse::KnownTokens tok;
        tok.intern(*src);
        if (src->hasAuthoredAttribute(objectRecord->mKey, tok.physxSceneTimeStepsPerSecond))
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
//
// Applied unconditionally: unlike the other Newton mappings (timeStepsPerSecond,
// contactMargin, contactGap) this is not a fallback spelling of a PhysX attribute,
// it is an independent on/off toggle with no PhysX equivalent. So there is no
// precedence to arbitrate — an authored physics:gravityMagnitude says how strong
// gravity is, and newton:gravityEnabled says whether it applies at all.
//
// This used to skip when physics:gravityMagnitude was authored ("PhysX wins",
// f9932cee68). That guard was dropped: it made the toggle silently inert on any
// stage that set a magnitude, it never matched the load path in ParseScene.cpp
// (which has always applied this unconditionally), and it was expressed with
// hasAuthoredAttribute, which a resolved-value backend cannot answer at all
// (ADR-0020) — so on ovstage it suppressed the toggle on every stage.
bool omni::physx::updateNewtonGravityEnabled(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
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
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);
            updateGravityMagnitude(attachedStage, objectId, tok.gravityMagnitude, timeCode);
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omni::physx::updateSceneUpdateType(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    if (const InternalDatabase::Record* objectRecord = getObjectRecord(objectId))
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        if (!source)
            return true;

        // Token-valued attribute: read the raw TokenId directly (mirrors what the
        // TfToken-typed getValue overload does internally) rather than round-tripping
        // through a materialized TfToken just to compare against the KnownTokens
        // constants below.
        omni::physics::parse::TokenId data;
        if (!source->getAttribute(objectRecord->mKey, property, data))
            return true;

        omni::physics::parse::KnownTokens tok;
        tok.intern(*source);

        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        PhysXScene* scene = omniPhysX.getPhysXSetup().getPhysXScene(size_t(objectId));
        if (scene && tok.sceneUpdateAsynchronous == data)
            scene->setUpdateType(eAsynchronous);
        else if (scene && tok.sceneUpdateSynchronous == data)
            scene->setUpdateType(eSynchronous);
        else if (scene && tok.sceneUpdateDisabled == data)
            scene->setUpdateType(eDisabled);
    }
    return true;
}

bool omni::physx::updateQuasistaticEnabled(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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

bool omni::physx::updateQuasistaticCollection(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
