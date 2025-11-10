// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>
#include <Raycast.h>
#include <PhysXScene.h>
#include <ContactReport.h>
#include <PhysXMirror.h>
#include <usdInterface/UsdInterface.h>
#include <common/utilities/MemoryMacros.h>

#include <private/omni/physics/schema/IUsdPhysicsListener.h>

#include <usdLoad/LoadTools.h>
#include <usdLoad/LoadUsd.h>
#include <usdLoad/PhysicsBody.h>

#include <carb/logging/Log.h>

#include <private/omni/physx/PhysxUsd.h>

#include <usdrt/gf/matrix.h>
#include <omni/fabric/stage/StageReaderWriter.h>

#include <physicsSchemaTools/physicsSchemaTokens.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// body

void wakeActor(PxRigidActor& actor)
{
    if (actor.getType() == PxConcreteType::eRIGID_DYNAMIC)
    {
        ((PxRigidDynamic&)actor).wakeUp();
    }
    else if (actor.getType() == PxConcreteType::eARTICULATION_LINK)
    {
        ((PxArticulationLink&)actor).getArticulation().wakeUp();
    }
}

void changeRigidActorType(PxRigidActor* sourceActor, PxRigidActor* destActor, const SdfPath& actorPath)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    // move shapes
    const PxU32 numShapes = sourceActor->getNbShapes();
    std::vector<PxShape*> shapes;
    shapes.resize(size_t(numShapes));
    sourceActor->getShapes(shapes.data(), numShapes);
    for (size_t i = 0; i < shapes.size(); i++)
    {
        PxShape& shape = *shapes[i];
        sourceActor->detachShape(shape);
        destActor->attachShape(shape);
    }

    // move constaints
    const PxU32 numConstraints = sourceActor->getNbConstraints();
    std::vector<PxConstraint*> constaints;
    constaints.resize(size_t(numConstraints));
    sourceActor->getConstraints(constaints.data(), numConstraints);
    for (size_t i = 0; i < constaints.size(); i++)
    {
        PxConstraint& constraint = *constaints[i];
        PxU32 typeId;
        PxJoint* joint = reinterpret_cast<PxJoint*>(constraint.getExternalReference(typeId));
        if (typeId == PxConstraintExtIDs::eJOINT)
        {
            PxRigidActor* actor0 = nullptr;
            PxRigidActor* actor1 = nullptr;
            joint->getActors(actor0, actor1);
            if (actor0 == sourceActor)
                actor0 = destActor;
            if (actor1 == sourceActor)
                actor1 = destActor;
            joint->setActors(actor0, actor1);
        }
    }

    destActor->setName(sourceActor->getName());
    destActor->userData = sourceActor->userData;

    omniPhysX.getRaycastManager().clearPicker(sourceActor);
    if (sourceActor->getScene())
    {
        PhysXScene* physxScene = omniPhysX.getPhysXSetup().getPhysXScene((size_t)sourceActor->getScene()->userData);

        physxScene->getContactReport()->swapActor(sourceActor, destActor);
        physxScene->getContactReport()->removeActor(sourceActor, actorPath);

        physxScene->getInternalScene()->swapForceActors(sourceActor, destActor);        

        physxScene->getInternalScene()->swapActorRigidAttachmentsDeprecated(sourceActor, destActor);
        physxScene->getInternalScene()->swapDeformableAttachmentsRigidActor(sourceActor, destActor);
        physxScene->getInternalScene()->swapDeformableCollisionFiltersRigidActor(sourceActor, destActor);
    }
}

void applyPhysxProps(AttachedStage& attachedStage, pxr::UsdPrim prim, PxRigidDynamic* rigidDynamic)
{
    CollisionPairVector filteredPairs;
    omni::physics::schema::RigidBodyDesc inDesc;
    inDesc.usdPrim = prim;
    inDesc.rigidBodyEnabled = true;

    const UsdPhysicsRigidBodyAPI rigidBodyAPI(inDesc.usdPrim);
    if (rigidBodyAPI)
    {
        rigidBodyAPI.GetKinematicEnabledAttr().Get(&inDesc.kinematicBody);
        rigidBodyAPI.GetVelocityAttr().Get(&inDesc.linearVelocity);
        rigidBodyAPI.GetAngularVelocityAttr().Get(&inDesc.angularVelocity);
    }

    pxr::UsdGeomXformCache cache;
    usdparser::DynamicPhysxRigidBodyDesc* desc = (usdparser::DynamicPhysxRigidBodyDesc* )usdparser::parseRigidBody(attachedStage, cache, inDesc, filteredPairs);
    if (desc)
    {
        PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(desc->sceneIds.empty() ? kInvalidObjectId : desc->sceneIds[0]);

        applyRigidDynamicPhysxDesc(physxScene, *desc, *rigidDynamic);

        usdparser::releaseDesc(desc);
    }
}


bool omni::physx::updateBodyEnabled(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (objectRecord->mType == ePTActor)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxActor* actor = reinterpret_cast<PxActor*>(objectRecord->mPtr);
        InternalActor* internalActor = reinterpret_cast<InternalActor*>(objectRecord->mInternalPtr);
        PxRigidDynamic* dynamicActor = actor->is<PxRigidDynamic>();
        const bool isDynamic = (dynamicActor != nullptr);
        if (isDynamic && !data)
        {
            // switch from dynamic to static

            // check if the body is part of a vehicle. If so, ignore the change and send error message.
            InternalScene* internalScene = internalActor->mPhysXScene->getInternalScene();
            if (internalScene->getVehicleBody(*dynamicActor))
            {
                CARB_LOG_ERROR("Setting rigidBodyEnabled to false is not supported if the rigid body is part of a vehicle. Body: %s", objectRecord->mPath.GetText());
                return true;
            }

            PxScene* scene = dynamicActor->getScene();
            attachedStage.getPhysXPhysicsInterface()->sendObjectDestructionNotification(objectRecord->mPath, objectId, ePTActor);
            PxRigidStatic* rigidStatic = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createRigidStatic(dynamicActor->getGlobalPose());
            changeRigidActorType(dynamicActor, rigidStatic, objectRecord->mPath);
            internalActor->mActor = rigidStatic;
            objectRecord->mPtr = rigidStatic;
            dynamicActor->release();
            if (scene)
                scene->addActor(*rigidStatic);
            attachedStage.getPhysXPhysicsInterface()->sendObjectCreationNotification(objectRecord->mPath, objectId, ePTActor);
        }
        else if (!isDynamic && data)
        {
            // switch from static to dynamic
            PxRigidActor* staticActor = actor->is<PxRigidActor>();
            PxScene* scene = staticActor->getScene();
            attachedStage.getPhysXPhysicsInterface()->sendObjectDestructionNotification(objectRecord->mPath, objectId, ePTActor);
            PxRigidDynamic* rigidDynamic = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createRigidDynamic(staticActor->getGlobalPose());
            changeRigidActorType(staticActor, rigidDynamic, objectRecord->mPath);
            applyPhysxProps(attachedStage, attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath), rigidDynamic);
            internalActor->mActor = rigidDynamic;
            objectRecord->mPtr = rigidDynamic;
            staticActor->release();
            if (scene)
                scene->addActor(*rigidDynamic);
            db.addDirtyMassActor(size_t(objectId));
            attachedStage.getPhysXPhysicsInterface()->sendObjectCreationNotification(objectRecord->mPath, objectId, ePTActor);
        }
    }
    else if (objectRecord->mType == ePTLink)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        if (!data)
        {
            CARB_LOG_ERROR("Setting rigidBodyEnabled to false is not supported if the rigid body is part of an articulation. Body: %s", objectRecord->mPath.GetText());
            return true;
        }
    }

    return true;
}

bool omni::physx::updateBodyDensity(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    db.addDirtyMassActor(size_t(objectId));
    return true;
}

void updateLinearVelocity(InternalActor* internalActor, PxRigidActor* actor, bool localSpaceVelocities, pxr::GfVec3f& outVelocity)
{
    if (localSpaceVelocities)
    {
        PxTransform tf = actor->getGlobalPose();

        const pxr::GfRotation rot(pxr::GfQuaternion(tf.q.w, pxr::GfVec3f(tf.q.x, tf.q.y, tf.q.z)));
        outVelocity = rot.TransformDir(outVelocity);
        const pxr::GfVec3f scale(internalActor->mScale.x, internalActor->mScale.y, internalActor->mScale.z);
        outVelocity = GfCompMult(scale, outVelocity);
    }
}

bool omni::physx::updateBodyLinearVelocity(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        GfVec3f data;
        if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->getConcreteType() == PxConcreteType::eRIGID_DYNAMIC)
        {
            InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
            CARB_ASSERT(internalActor);
            updateLinearVelocity(internalActor, actor, internalActor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES, data);

            PxRigidDynamic* rbo = (PxRigidDynamic*)actor;
            if (rbo->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
            {
                internalActor->mSurfaceVelocity = toPhysX(data);
                if (GfIsClose(data, GfVec3f(0.0f), 1e-4) &&
                    GfIsClose(toVec3f(internalActor->mSurfaceAngularVelocity), GfVec3f(0.0f), 1e-4))
                    internalActor->enableSurfaceVelocity(false, *actor);
                else
                    internalActor->enableSurfaceVelocity(true, *actor);
            }
            else
            {
                rbo->setLinearVelocity(PxVec3(data[0], data[1], data[2]));
            }
        }
    }
    else if (internalType == ePTLink)
    {
        PxArticulationLink* actor = (PxArticulationLink*)objectRecord->mPtr;
        if (actor->getLinkIndex())
        {
            return true;
        }
        else
        {
            GfVec3f velocity;
            if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, velocity))
                return true;

            actor->getArticulation().setRootLinearVelocity(PxVec3(velocity[0], velocity[1], velocity[2]));
            // preist todo: Check if articulation->updatekinematic is needed here.
        }
    }
    return true;
}

void updateAngularVelocity(InternalActor* internalActor,
                           PxRigidActor* actor,
                           bool localSpaceVelocities,
                           pxr::GfVec3f& outVelocity)
{
    if (localSpaceVelocities)
    {
        PxTransform tf = actor->getGlobalPose();

        const pxr::GfRotation rot(pxr::GfQuaternion(tf.q.w, pxr::GfVec3f(tf.q.x, tf.q.y, tf.q.z)));
        outVelocity = rot.TransformDir(outVelocity);
    }
}

bool omni::physx::updateBodyAngularVelocity(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        GfVec3f data;
        if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        CARB_ASSERT(internalActor);
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        updateAngularVelocity(internalActor, actor, internalActor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES, data);
        data = degToRad(data);
        
        PxRigidDynamic* rbo = actor->is<PxRigidDynamic>();
        if (rbo)
        {
            if (rbo->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
            {
                internalActor->mSurfaceAngularVelocity = toPhysX(data);
                internalActor->mSurfaceAngularVelocityPivot = actor->getGlobalPose();
                if (GfIsClose(data, GfVec3f(0.0f), 1e-4) &&
                    GfIsClose(toVec3f(internalActor->mSurfaceVelocity), GfVec3f(0.0f), 1e-4))
                    internalActor->enableSurfaceVelocity(false, *actor);
                else
                    internalActor->enableSurfaceVelocity(true, *actor);
            }
            else
            {
                rbo->setAngularVelocity(PxVec3(data[0], data[1], data[2]));
            }
        }
    }
    else if (internalType == ePTLink)
    {
        PxArticulationLink* actor = (PxArticulationLink*)objectRecord->mPtr;
        if (actor->getLinkIndex())
        {
            return true;
        }
        else
        {
            GfVec3f angularVelocity;
            if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, angularVelocity))
                return true;

            angularVelocity = degToRad(angularVelocity);
            actor->getArticulation().setRootAngularVelocity(
                PxVec3(angularVelocity[0], angularVelocity[1], angularVelocity[2]));
            // preist todo: Check if articulation->updatekinematic is needed here.
        }
    }
    return true;
}

// physx body
bool updateRigidBody(bool force, AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);

    if (objectRecord)
    {
        if (internalType == ePTActor ||
            internalType == ePTLink)
        {
            GfVec3f data;

            if (getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, data))
            {
                PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
                PxRigidBody* rigidBody = actor->is<PxRigidBody>();

                const PxVec3 vec3Data = PxVec3(data[0], data[1], data[2]);

                if (rigidBody &&
                    (rigidBody->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) == false &&
                    (rigidBody->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION) == false)
                {
                    if (force)
                    {
                        rigidBody->addForce(vec3Data);
                    }
                    else
                    {
                        rigidBody->addTorque(vec3Data);
                    }
                }
            }
        }
    }

    return true;
}

bool omni::physx::updateBodyWorldForce(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const bool force = true;

    return updateRigidBody(force, attachedStage, objectId, property, timeCode);
}

bool omni::physx::updateBodyWorldTorque(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const bool force = false;

    return updateRigidBody(force, attachedStage, objectId, property, timeCode);
}

bool omni::physx::updateBodyLinearDamping(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setLinearDamping(data);
    }
    return true;
}

bool omni::physx::updateBodyAngularDamping(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setAngularDamping(data);
    }
    return true;
}

bool omni::physx::updateBodyMaxLinearVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>() && data >= 0.0f && data <= SQRT_FLT_MAX)
            actor->is<PxRigidDynamic>()->setMaxLinearVelocity(data);
    }
    return true;
}

bool omni::physx::updateBodyMaxAngularVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        data = degToRad(data);
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>() && data >= 0.0f && data <= SQRT_FLT_MAX)
            actor->is<PxRigidDynamic>()->setMaxAngularVelocity(data);
    }
    return true;
}

bool omni::physx::updateBodyMaxContactImpulse(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setMaxContactImpulse(data);
    }
    return true;
}

bool omni::physx::updateBodySleepThreshold(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        const float toleranceSpeed = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getTolerancesScale().speed;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        data = data * toleranceSpeed * toleranceSpeed;
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setSleepThreshold(data);
    }
    return true;
}

bool omni::physx::updateBodyStabilizationThreshold(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        const float toleranceSpeed = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getTolerancesScale().speed;
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;
        data = data * toleranceSpeed * toleranceSpeed;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setStabilizationThreshold(data);
    }
    return true;
}

bool omni::physx::updateBodyMaxDepenetrationVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setMaxDepenetrationVelocity(data);
    }
    return true;
}

bool omni::physx::updateBodyContactSlopCoefficient(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setContactSlopCoefficient(data);
    }
    return true;
}

bool omni::physx::updateBodySolverPositionIterationCount(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        int data;
        if (!getValue<int>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        InternalActor* intActor = (InternalActor*)objectRecord->mInternalPtr;
        if (actor->is<PxRigidDynamic>())
        {
            PxU32 posIt, velIt;
            actor->is<PxRigidDynamic>()->getSolverIterationCounts(posIt, velIt);
            const InternalScene* internalScene = intActor->mPhysXScene->getInternalScene();
            data = internalScene->clampPosIterationCount(data);
            actor->is<PxRigidDynamic>()->setSolverIterationCounts(data, velIt);
        }
    }
    return true;
}

bool omni::physx::updateBodySolverVelocityIterationCount(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        int data;
        if (!getValue<int>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        InternalActor* intActor = (InternalActor*)objectRecord->mInternalPtr;
        if (actor->is<PxRigidDynamic>())
        {
            PxU32 posIt, velIt;
            actor->is<PxRigidDynamic>()->getSolverIterationCounts(posIt, velIt);
            const InternalScene* internalScene = intActor->mPhysXScene->getInternalScene();
            data = internalScene->clampVelIterationCount(data);
            actor->is<PxRigidDynamic>()->setSolverIterationCounts(posIt, data);
        }
    }
    return true;
}

bool omni::physx::updateBodyEnableKinematics(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        PxRigidDynamic* dyn = actor->is<PxRigidDynamic>();
        InternalActor* intActor = (InternalActor*)objectRecord->mInternalPtr;

        // if trying to disable kinematic for a body with time sampled xform
        if(intActor->mFlags & InternalActorFlag::eHAS_TIME_SAMPLED_XFORM && data == false)
        {
            CARB_LOG_WARN("Prim: %s, failed to switch from kinematic body to dynamic body as prim has parents with animated xformOps.", objectRecord->mPath.GetText());
            return false;
        }

        if (dyn)
        {
            dyn->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, (data));
            if (!data)
            {
                // Need to block USD update, we can get a resync operation from the xformOp sanitation code 
                UsdLoad::getUsdLoad()->blockUSDUpdate(true);
                intActor->switchFromKinematic();
                UsdLoad::getUsdLoad()->blockUSDUpdate(false);
                dyn->wakeUp();
            }
            else
            {
                // check if the body is part of an enabled vehicle. If so, disable the vehicle and send error message.
                InternalScene* internalScene = intActor->mPhysXScene->getInternalScene();
                InternalVehicle* internalVehicle = internalScene->getVehicleBody(*dyn);
                if (internalVehicle && internalScene->isVehicleEnabled(*internalVehicle))
                {
                    CARB_LOG_ERROR("Setting kinematicEnabled to true on a rigid body that is part of an enabled vehicle is illegal. "
                        "Please disable the vehicle first (see vehicleEnabled on the PhysxVehicleAPI USD API schema). The vehicle will "
                        "be disabled internally now. Body: %s", objectRecord->mPath.GetText());

                    internalScene->setVehicleEnabledState(*internalVehicle, false);

                    return true;
                }
            }
        }
    }
    return true;
}

bool omni::physx::updateBodyEnableCCD(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
        {
            actor->is<PxRigidDynamic>()->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, (data));
        }
    }
    return true;
}

bool omni::physx::updateBodyEnableSpeculativeCCD(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
        {
            actor->is<PxRigidDynamic>()->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, (data));
        }
    }
    return true;
}

bool omni::physx::updateBodyGyroscopicForces(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
        {
            actor->is<PxRigidDynamic>()->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES, (data));
        }
    }
    return true;
}


bool omni::physx::updateBodyRetainAccelerations(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
        {
            actor->is<PxRigidDynamic>()->setRigidBodyFlag(PxRigidBodyFlag::eRETAIN_ACCELERATIONS, (data));
        }
    }
    return true;
}

bool omni::physx::updateBodyDisableGravity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor)
        {
            actor->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, (data));
            PxRigidDynamic* rd = actor->is<PxRigidDynamic>();
            if (!data && rd && !(rd->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
            {
                rd->wakeUp();
            }
        }
    }
    return true;
}

bool omni::physx::updateBodyLockedPosAxis(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        int data;
        if (!getValue<int>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
        {
            actor->is<PxRigidDynamic>()->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_X, (data & 1 << 0));
            actor->is<PxRigidDynamic>()->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Y, (data & 1 << 1));
            actor->is<PxRigidDynamic>()->setRigidDynamicLockFlag(PxRigidDynamicLockFlag::eLOCK_LINEAR_Z, (data & 1 << 2));
        }
    }
    return true;
}

bool omni::physx::updateBodyLockedRotAxis(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        int data;
        if (!getValue<int>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
        {
            actor->is<PxRigidDynamic>()->setRigidDynamicLockFlag(
                PxRigidDynamicLockFlag::eLOCK_ANGULAR_X, (data & 1 << 0));
            actor->is<PxRigidDynamic>()->setRigidDynamicLockFlag(
                PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y, (data & 1 << 1));
            actor->is<PxRigidDynamic>()->setRigidDynamicLockFlag(
                PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z, (data & 1 << 2));
        }
    }
    return true;
}

bool omni::physx::updateBodySolveContacts(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        bool data = true;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        InternalActor* intActor = (InternalActor*)objectRecord->mInternalPtr;
        intActor->enableContactSolve(data, actor);
    }
    return true;
}

bool omni::physx::updateBodyTransformStack(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    if (objectId < db.getRecords().size())
    {
        const InternalDatabase::Record& objectRec = db.getRecords()[objectId];
        if (objectRec.mType == ePTActor)
        {
            InternalActor* internalActor = (InternalActor*)objectRec.mInternalPtr;
            if (getParentXform(attachedStage.getStage()->GetPrimAtPath(objectRec.mPath), internalActor->mParentXformPrim))
            {
                internalActor->mFlags |= InternalActorFlag::eHAS_PARENT_XFORM;
                internalActor->mFlags |= InternalActorFlag::ePARENT_XFORM_DIRTY;
            }
            else
            {
                internalActor->mFlags &= ~InternalActorFlag::eHAS_PARENT_XFORM;
            }
        }
    }
    return true;
}

bool omni::physx::updateBodyCfmScale(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTLink)
    {
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        PxArticulationLink* actor = (PxArticulationLink*)objectRecord->mPtr;
        if (actor)
        {
            actor->setCfmScale(data);
        }
    }
    return true;
}

void clearSimulationOwners(InternalActor& internalActor, omni::physx::usdparser::ObjectId sceneId)
{
    for (MirrorActor& mirror : internalActor.mMirrors)
    {
        mirror.release();
    }
    internalActor.mMirrors.clear();
    SAFE_RELEASE(internalActor.mMirrorSharedCollection);
    internalActor.mMirrorMemsize = 0;
    if (internalActor.mMirrorMemory)
    {
        free(internalActor.mMirrorMemory);
        internalActor.mMirrorMemory = nullptr;
    }

    // get default scene
    PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(sceneId);
    if (!physxScene)
    {
        return;
    }

    PhysXScene* oldPhysXScene = internalActor.mPhysXScene;
    if (internalActor.mPhysXScene != physxScene || internalActor.mActor->getScene() != physxScene->getScene())
    {
        if (internalActor.mActor->getScene())
        {
            internalActor.mActor->getScene()->removeActor(*internalActor.mActor);
        }        
        physxScene->getScene()->addActor(*internalActor.mActor);
        internalActor.mPhysXScene = physxScene;
    }

    std::vector<InternalActor*>& mirrorActors = oldPhysXScene->getInternalScene()->mMirorredActors;
    for (size_t i = mirrorActors.size(); i--;)
    {
        if (mirrorActors[i] == &internalActor)
        {
            mirrorActors[i] = mirrorActors.back();
            mirrorActors.pop_back();
            break;
        }
    }
}

void swapMainSimulationOwner(AttachedStage& attachedStage, InternalActor& internalActor, const SdfPath& ownerPath)
{
    omni::physx::usdparser::ObjectId sceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownerPath, ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(sceneId);
    if (!physxScene)
    {
        CARB_LOG_ERROR("No PhysX Scene found for path %s", ownerPath.GetText());
        return;
    }
    if (internalActor.mPhysXScene == physxScene && internalActor.mActor->getScene() == physxScene->getScene())
        return;

    internalActor.mActor->getScene()->removeActor(*internalActor.mActor);
    physxScene->getScene()->addActor(*internalActor.mActor);
    internalActor.mPhysXScene = physxScene;
}

void createMirrorSimulationOwner(AttachedStage& attachedStage, InternalActor& internalActor, const SdfPathVector& ownersPaths)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    void* mirrorMemBlock = nullptr;
    uint32_t memSize = 0;
    PxCollection* sharedCollection = nullptr;
    mirrorActor(*internalActor.mActor, mirrorMemBlock, memSize, *physxSetup.getSerializationRegistry(), sharedCollection);
    CARB_ASSERT(sharedCollection);
    internalActor.mMirrorSharedCollection = sharedCollection;
    internalActor.mMirrorMemsize = memSize;
    internalActor.mMirrorMemory = mirrorMemBlock;

    omni::physx::usdparser::ObjectId sceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownersPaths[0], ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);    
    physxScene->getInternalScene()->mMirorredActors.push_back(&internalActor);

    for (size_t i = 1; i < ownersPaths.size(); i++)
    {
        void* nm = copyAlignedMemory(mirrorMemBlock, memSize);
        omni::physx::usdparser::ObjectId mirrorSceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownersPaths[i], ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
        PhysXScene* mirrorScene = physxSetup.getPhysXScene(mirrorSceneId);
        PxCollection* col = nullptr;
        PxRigidActor* actor = (PxRigidActor*)instantiateMirrorActor(nm, *physxSetup.getSerializationRegistry(),
            *mirrorScene->getScene(), col, sharedCollection);
        if (actor)
        {
            // mirrored body is always kinematic
            PxRigidDynamic* dynamicBody = actor->is<PxRigidDynamic>();
            if (dynamicBody)
            {
                dynamicBody->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
            }
            internalActor.mMirrors.push_back({ nm, col, actor });
        }
    }

}

void swapMirrorSimulationOwner(AttachedStage& attachedStage, InternalActor& internalActor, const SdfPath& ownerPath, size_t mirrorIndex)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    omni::physx::usdparser::ObjectId sceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownerPath, ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);    

    if (!physxScene)
    {
        CARB_LOG_ERROR("No PhysX Scene found for path %s", ownerPath.GetText());
        return;
    }
    if (internalActor.mMirrors.empty())
    {
        CARB_ASSERT(0);
        return;
    }

    if (mirrorIndex < internalActor.mMirrors.size())
    {
        MirrorActor& mirror = internalActor.mMirrors[mirrorIndex];
        if (mirror.actor->getScene() == physxScene->getScene())
            return;

        mirror.actor->getScene()->removeActor(*mirror.actor);
        physxScene->getScene()->addActor(*mirror.actor);
    }
    else
    {
        // Add mirror
        void* nm = copyAlignedMemory(internalActor.mMirrorMemory, internalActor.mMirrorMemsize);
        omni::physx::usdparser::ObjectId mirrorSceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownerPath, ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
        PhysXScene* mirrorScene = physxSetup.getPhysXScene(mirrorSceneId);
        PxCollection* col = nullptr;
        PxRigidActor* actor = (PxRigidActor*)instantiateMirrorActor(nm, *physxSetup.getSerializationRegistry(),
            *mirrorScene->getScene(), col, internalActor.mMirrorSharedCollection);
        if (actor)
        {
            // mirrored body is always kinematic
            PxRigidDynamic* dynamicBody = actor->is<PxRigidDynamic>();
            if (dynamicBody)
            {
                dynamicBody->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
            }
            internalActor.mMirrors.push_back({ nm, col, actor });
        }
    }
}

bool omni::physx::updateBodySimulationOwner(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        if (internalActor)
        {
            UsdRelationship rel = attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath).GetRelationship(property);
            if (rel)
            {
                SdfPathVector owners;
                rel.GetTargets(&owners);
                if (owners.empty())
                {
                    clearSimulationOwners(*internalActor, kInvalidObjectId);
                }
                else
                {
                    if (owners.size() == 1)
                    {
                        const omni::physx::usdparser::ObjectId sceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(owners[0], ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
                        clearSimulationOwners(*internalActor, sceneId);
                    }
                    else
                    {
                        swapMainSimulationOwner(attachedStage, *internalActor, owners[0]);
                        if (internalActor->mMirrors.empty())
                        {
                            createMirrorSimulationOwner(attachedStage, *internalActor, owners);
                        }
                        else
                        {
                            for (size_t i = 1; i < owners.size(); i++)
                            {
                                swapMirrorSimulationOwner(attachedStage, *internalActor, owners[i], i-1);
                            }
                            CARB_ASSERT(internalActor->mMirrors.size() >= owners.size() - 1);
                            const size_t mirrorsToDelete = internalActor->mMirrors.size() - (owners.size() - 1);
                            for (size_t i = 0; i < mirrorsToDelete; i++)
                            {
                                internalActor->mMirrors.back().release();
                                internalActor->mMirrors.pop_back();
                            }
                        }
                    }
                }
            }
            else
            {
                clearSimulationOwners(*internalActor, kInvalidObjectId);
            }
        }
    }
    return true;
}

bool omni::physx::updateBodySurfaceVelocityEnabled(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    bool data;
    if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        internalActor->enableSurfaceVelocity(data, *actor);
        wakeActor(*actor);
    }

    return true;
}

bool omni::physx::updateBodySurfaceVelocityLocalSpace(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    bool data;
    if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        internalActor->mSurfaceVelocityLocalSpace = data;        
    }

    return true;
}

bool omni::physx::updateBodySurfaceLinearVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    GfVec3f data;
    if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, data))
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        if (internalActor->mSurfaceVelocityLocalSpace)
            internalActor->mSurfaceVelocity = toPhysX(GfCompMult((const GfVec3f&)internalActor->mScale, data));
        else
            internalActor->mSurfaceVelocity = toPhysX(data);
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        wakeActor(*actor);
    }

    return true;
}


bool omni::physx::updateBodySplineSurfaceVelocityEnabled(AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId objectId,
                                                   const pxr::TfToken& property,
                                                   const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    bool data;
    if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        if (data)
        {
            UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(objectRecord->mPath);
            UsdRelationship splinesRel = prim.GetRelationship(PhysxAdditionAttrTokens->surfaceVelocityCurve);
            SdfPathVector splinesList;
            if (splinesRel)
            {
                splinesRel.GetTargets(&splinesList);
            }

            if (!splinesRel || splinesList.empty())
            {
                CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined.",
                               prim.GetPrimPath().GetText());

                return true;
            }
            else
            {
                const SdfPath& splinePath = splinesList[0];
                const UsdPrim splinePrim = attachedStage.getStage()->GetPrimAtPath(splinePath);
                if (!splinePrim || !splinePrim.IsA<UsdGeomBasisCurves>())
                {
                    CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined.",
                                   prim.GetPrimPath().GetText());

                    return true;
                }
                else
                {
                    internalActor->enableSplineSurfaceVelocity(data, *actor, UsdGeomBasisCurves(splinePrim));
                }
            }            
        }
        else
        {
            internalActor->enableSplineSurfaceVelocity(data, *actor, UsdGeomBasisCurves());
        }        
        wakeActor(*actor);
    }

    return true;
}

bool omni::physx::updateBodySplineSurfaceVelocityMagnitude(AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId objectId,
                                                  const pxr::TfToken& property,
                                                  const pxr::UsdTimeCode& timeCode)
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

    if (internalType == ePTActor || internalType == ePTLink)
    {
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        internalActor->mSplinesSurfaceVelocityMagnitude = data;
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        wakeActor(*actor);
    }

    return true;
}

bool omni::physx::updateBodySurfaceAngularVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    GfVec3f data;
    if (!getValue<GfVec3f>(attachedStage, objectRecord->mPath, property, timeCode, data))
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        internalActor->mSurfaceAngularVelocity = toPhysX(degToRad(data));
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        wakeActor(*actor);
    }

    return true;
}

bool omni::physx::updatePhysxContactReportThreshold(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {        
        float data;
        if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
            return true;

        const PxRigidActor* actor = (const PxRigidActor*)objectRecord->mPtr;
        if (actor && actor->getScene())
        {
            PhysXScene* physxScene = omniPhysX.getPhysXSetup().getPhysXScene((size_t)actor->getScene()->userData);
            if (physxScene)
            {
                physxScene->getContactReport()->setBodyThreshold(actor, data);
            }
        }
    }
    return true;
}
