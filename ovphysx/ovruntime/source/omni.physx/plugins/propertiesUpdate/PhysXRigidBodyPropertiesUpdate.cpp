// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BODY-001
 * @covers AC-5
 *
 * @implements REQ-SIM-MULTISCENE-001
 * @covers AC-4
 *
 * @implements REQ-SPLINE-TARGET-001
 * @covers AC-1 AC-2 AC-4
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-2
 */

/**
 * @implements REQ-SIM-ACTIVEACTOR-001
 * @covers AC-1
 */

#include "PhysXPropertiesUpdate.h"

#include <omni/physics/parse/KnownTokens.h>

#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>
#include <Raycast.h>
#include <PhysXScene.h>
#include <ContactReport.h>
#include <PhysXMirror.h>
#include <usdInterface/UsdInterface.h>
#include <common/utilities/MemoryMacros.h>


#include <usdLoad/LoadTools.h>
#include <usdLoad/LoadUsd.h>
#include <usdLoad/IceDescriptorAllocator.h>

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <carb/logging/Log.h>

#include <private/omni/physx/PhysxUsd.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
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

// actorKey is ObjectKey-typed (ADR-0019); ContactReport::removeActor
// (ContactReport.h) has both an ObjectKey overload and an SdfPath sibling for
// callers that still hold a live USD path, so this routes through the former.
void changeRigidActorType(PxRigidActor* sourceActor, PxRigidActor* destActor, omni::physics::parse::ObjectKey actorKey)
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
        physxScene->getContactReport()->removeActor(sourceActor, actorKey);

        physxScene->getInternalScene()->swapForceActors(sourceActor, destActor);        

        physxScene->getInternalScene()->swapDeformableAttachmentsRigidActor(sourceActor, destActor);
        physxScene->getInternalScene()->swapDeformableCollisionFiltersRigidActor(sourceActor, destActor);
    }
}

void applyPhysxProps(AttachedStage& attachedStage, omni::physics::parse::ObjectKey key,
                     PxRigidDynamic* rigidDynamic, PhysXScene* physxScene)
{
    // Re-parse the body's PhysX-extension properties from the active source and apply them to the
    // freshly created dynamic actor. parseDynamicBody is source-agnostic (works for USD and ovstage
    // alike). The body's scene is unchanged by the enable switch, so the caller passes the existing
    // PhysXScene rather than re-resolving simulationOwner.
    omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(key))
        return;

    omni::physics::parse::ParseContext ctx(*src, omni::physx::usdparser::iceDescriptorAllocator());
    ctx.adoptKnownTokens(attachedStage.getKnownTokens());
    omni::physics::parse::DescPtr<omni::physics::parse::DynamicPhysxRigidBodyDesc> dyn =
        omni::physics::parse::parseDynamicBody(ctx, key);
    if (!dyn || dyn->type != omni::physics::parse::eDynamicBody)
        return;

    applyRigidDynamicPhysxDesc(physxScene, *dyn, *rigidDynamic);
}


bool omni::physx::updateBodyEnabled(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
                CARB_LOG_ERROR("Setting rigidBodyEnabled to false is not supported if the rigid body is part of a vehicle. Body: %s", attachedStage.textFor(objectRecord->mKey));
                return true;
            }

            PxScene* scene = dynamicActor->getScene();
            attachedStage.getPhysXPhysicsInterface()->sendObjectDestructionNotification(objectRecord->mKey, objectId, ePTActor);
            PxRigidStatic* rigidStatic = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createRigidStatic(dynamicActor->getGlobalPose());
            changeRigidActorType(dynamicActor, rigidStatic, objectRecord->mKey);
            internalActor->mActor = rigidStatic;
            objectRecord->mPtr = rigidStatic;
            internalScene->trackReleasedActiveActor(dynamicActor);
            dynamicActor->release();
            if (scene)
                scene->addActor(*rigidStatic);
            attachedStage.getPhysXPhysicsInterface()->sendObjectCreationNotification(objectRecord->mKey, objectId, ePTActor);
        }
        else if (!isDynamic && data)
        {
            // switch from static to dynamic
            PxRigidActor* staticActor = actor->is<PxRigidActor>();
            PxScene* scene = staticActor->getScene();
            attachedStage.getPhysXPhysicsInterface()->sendObjectDestructionNotification(objectRecord->mKey, objectId, ePTActor);
            PxRigidDynamic* rigidDynamic = OmniPhysX::getInstance().getPhysXSetup().getPhysics()->createRigidDynamic(staticActor->getGlobalPose());
            changeRigidActorType(staticActor, rigidDynamic, objectRecord->mKey);
            applyPhysxProps(attachedStage, objectRecord->mKey, rigidDynamic, internalActor->mPhysXScene);
            internalActor->mActor = rigidDynamic;
            objectRecord->mPtr = rigidDynamic;
            internalActor->mPhysXScene->getInternalScene()->trackReleasedActiveActor(staticActor);
            staticActor->release();
            if (scene)
                scene->addActor(*rigidDynamic);
            db.addDirtyMassActor(size_t(objectId));
            attachedStage.getPhysXPhysicsInterface()->sendObjectCreationNotification(objectRecord->mKey, objectId, ePTActor);
        }
    }
    else if (objectRecord->mType == ePTLink)
    {
        bool data;
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        if (!data)
        {
            CARB_LOG_ERROR("Setting rigidBodyEnabled to false is not supported if the rigid body is part of an articulation. Body: %s", attachedStage.textFor(objectRecord->mKey));
            return true;
        }
    }

    return true;
}

bool omni::physx::updateBodyDensity(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    db.addDirtyMassActor(size_t(objectId));
    return true;
}

// zero test matching GfIsClose(v, zero, 1e-4)
static bool isVelocityZero(const PxVec3& v)
{
    constexpr double kTolerance = 1e-4;
    return double(v.magnitudeSquared()) <= kTolerance * kTolerance;
}

void updateLinearVelocity(InternalActor* internalActor, PxRigidActor* actor, bool localSpaceVelocities, PxVec3& outVelocity)
{
    if (localSpaceVelocities)
    {
        const PxTransform tf = actor->getGlobalPose();

        outVelocity = tf.q.rotate(outVelocity).multiply(toPhysX(internalActor->mScale));
    }
}

bool omni::physx::updateBodyLinearVelocity(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        carb::Float3 data;
        if (!getValue<carb::Float3>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;
        PxVec3 velocity = toPhysX(data);

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->getConcreteType() == PxConcreteType::eRIGID_DYNAMIC)
        {
            InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
            CARB_ASSERT(internalActor);
            updateLinearVelocity(internalActor, actor, internalActor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES, velocity);

            PxRigidDynamic* rbo = (PxRigidDynamic*)actor;
            if (rbo->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
            {
                // Legacy path: a kinematic body's authored physics:velocity acts as the
                // surface velocity. Parsing pins surfaceVelocityLocalSpace to false for
                // it, so the effective and authored values are the same vector here.
                internalActor->mSurfaceVelocity = velocity;
                internalActor->mSurfaceVelocityAuthored = internalActor->mSurfaceVelocity;
                if (isVelocityZero(velocity) && isVelocityZero(internalActor->mSurfaceAngularVelocity))
                    internalActor->enableSurfaceVelocity(false, *actor);
                else
                    internalActor->enableSurfaceVelocity(true, *actor);
            }
            else
            {
                rbo->setLinearVelocity(velocity);
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
            carb::Float3 velocity;
            if (!getValue<carb::Float3>(attachedStage, objectRecord->mKey, property, timeCode, velocity))
                return true;

            actor->getArticulation().setRootLinearVelocity(toPhysX(velocity));
            // preist todo: Check if articulation->updatekinematic is needed here.
        }
    }
    return true;
}

void updateAngularVelocity(InternalActor* internalActor,
                           PxRigidActor* actor,
                           bool localSpaceVelocities,
                           PxVec3& outVelocity)
{
    if (localSpaceVelocities)
    {
        const PxTransform tf = actor->getGlobalPose();

        outVelocity = tf.q.rotate(outVelocity);
    }
}

bool omni::physx::updateBodyAngularVelocity(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTActor)
    {
        carb::Float3 data;
        if (!getValue<carb::Float3>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;
        PxVec3 angularVelocity = toPhysX(data);

        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        CARB_ASSERT(internalActor);
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        updateAngularVelocity(internalActor, actor, internalActor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES, angularVelocity);
        angularVelocity = PxVec3(degToRad(angularVelocity.x), degToRad(angularVelocity.y), degToRad(angularVelocity.z));

        PxRigidDynamic* rbo = actor->is<PxRigidDynamic>();
        if (rbo)
        {
            if (rbo->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
            {
                internalActor->mSurfaceAngularVelocity = angularVelocity;
                internalActor->mSurfaceAngularVelocityPivot = actor->getGlobalPose();
                if (isVelocityZero(angularVelocity) && isVelocityZero(internalActor->mSurfaceVelocity))
                    internalActor->enableSurfaceVelocity(false, *actor);
                else
                    internalActor->enableSurfaceVelocity(true, *actor);
            }
            else
            {
                rbo->setAngularVelocity(angularVelocity);
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
            carb::Float3 angularVelocity;
            if (!getValue<carb::Float3>(attachedStage, objectRecord->mKey, property, timeCode, angularVelocity))
                return true;

            actor->getArticulation().setRootAngularVelocity(degToRad(toPhysX(angularVelocity)));
            // preist todo: Check if articulation->updatekinematic is needed here.
        }
    }
    return true;
}

bool omni::physx::updateBodyLinearDamping(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setLinearDamping(data);
    }
    return true;
}

bool omni::physx::updateBodyAngularDamping(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setAngularDamping(data);
    }
    return true;
}

bool omni::physx::updateBodyMaxLinearVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>() && data >= 0.0f && data <= SQRT_FLT_MAX)
            actor->is<PxRigidDynamic>()->setMaxLinearVelocity(data);
    }
    return true;
}

bool omni::physx::updateBodyMaxAngularVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        data = degToRad(data);
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>() && data >= 0.0f && data <= SQRT_FLT_MAX)
            actor->is<PxRigidDynamic>()->setMaxAngularVelocity(data);
    }
    return true;
}

bool omni::physx::updateBodyMaxContactImpulse(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setMaxContactImpulse(data);
    }
    return true;
}

bool omni::physx::updateBodySleepThreshold(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        data = data * toleranceSpeed * toleranceSpeed;
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setSleepThreshold(data);
    }
    return true;
}

bool omni::physx::updateBodyStabilizationThreshold(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;
        data = data * toleranceSpeed * toleranceSpeed;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setStabilizationThreshold(data);
    }
    return true;
}

bool omni::physx::updateBodyMaxDepenetrationVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setMaxDepenetrationVelocity(data);
    }
    return true;
}

bool omni::physx::updateBodyContactSlopCoefficient(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        if (actor->is<PxRigidDynamic>())
            actor->is<PxRigidDynamic>()->setContactSlopCoefficient(data);
    }
    return true;
}

bool omni::physx::updateBodySolverPositionIterationCount(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<int>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<int>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        PxRigidDynamic* dyn = actor->is<PxRigidDynamic>();
        InternalActor* intActor = (InternalActor*)objectRecord->mInternalPtr;

        // if trying to disable kinematic for a body with time sampled xform
        if(intActor->mFlags & InternalActorFlag::eHAS_TIME_SAMPLED_XFORM && data == false)
        {
            CARB_LOG_WARN("Prim: %s, failed to switch from kinematic body to dynamic body as prim has parents with animated xformOps.", attachedStage.textFor(objectRecord->mKey));
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
                        "be disabled internally now. Body: %s", attachedStage.textFor(objectRecord->mKey));

                    internalScene->setVehicleEnabledState(*internalVehicle, false);

                    return true;
                }
            }
        }
    }
    return true;
}

bool omni::physx::updateBodyEnableCCD(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<int>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<int>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        InternalActor* intActor = (InternalActor*)objectRecord->mInternalPtr;
        intActor->enableContactSolve(data, actor);
    }
    return true;
}

bool omni::physx::updateBodyTransformStack(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    // No-op: transform write-back goes through IPhysicsDataWrite, which resolves each
    // body's parent frame per-frame, so no cached parent state needs refreshing here.
    return true;
}

bool omni::physx::updateBodyCfmScale(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
            return true;

        PxArticulationLink* actor = (PxArticulationLink*)objectRecord->mPtr;
        if (actor)
        {
            actor->setCfmScale(data);
        }
    }
    return true;
}

// Moves the InternalActor between the owning scenes' bookkeeping. InternalScene::mActors owns the
// entry and the removal paths look the list up through InternalActor::mPhysXScene, so the two have to
// be updated together - leaving the entry in the previous scene's list makes it dangle as soon as the
// actor is deleted, and the teardown loop in InternalScene::release() then walks it (NVBugs 6504495).
static void moveInternalActorToScene(InternalActor& internalActor, PhysXScene& newPhysXScene)
{
    PhysXScene* oldPhysXScene = internalActor.mPhysXScene;
    if (oldPhysXScene == &newPhysXScene)
    {
        return;
    }

    if (oldPhysXScene && oldPhysXScene->getInternalScene())
    {
        oldPhysXScene->getInternalScene()->removeActor(internalActor);
    }
    if (newPhysXScene.getInternalScene())
    {
        newPhysXScene.getInternalScene()->addActor(internalActor);
    }
    internalActor.mPhysXScene = &newPhysXScene;
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
        moveInternalActorToScene(internalActor, *physxScene);
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

void swapMainSimulationOwner(AttachedStage& attachedStage, InternalActor& internalActor, omni::physics::parse::ObjectKey ownerKey)
{
    omni::physx::usdparser::ObjectId sceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownerKey, ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(sceneId);
    if (!physxScene)
    {
        CARB_LOG_ERROR("No PhysX Scene found for path %s", attachedStage.textFor(ownerKey));
        return;
    }
    if (internalActor.mPhysXScene == physxScene && internalActor.mActor->getScene() == physxScene->getScene())
        return;

    internalActor.mActor->getScene()->removeActor(*internalActor.mActor);
    physxScene->getScene()->addActor(*internalActor.mActor);
    moveInternalActorToScene(internalActor, *physxScene);
}

void createMirrorSimulationOwner(AttachedStage& attachedStage, InternalActor& internalActor, const std::vector<omni::physics::parse::ObjectKey>& ownersKeys)
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

    omni::physx::usdparser::ObjectId sceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownersKeys[0], ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);
    physxScene->getInternalScene()->mMirorredActors.push_back(&internalActor);

    for (size_t i = 1; i < ownersKeys.size(); i++)
    {
        void* nm = copyAlignedMemory(mirrorMemBlock, memSize);
        omni::physx::usdparser::ObjectId mirrorSceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownersKeys[i], ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
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
            internalActor.mMirrors.push_back({ nm, col, actor, mirrorScene->getInternalScene() });
        }
    }

}

void swapMirrorSimulationOwner(AttachedStage& attachedStage, InternalActor& internalActor, omni::physics::parse::ObjectKey ownerKey, size_t mirrorIndex)
{
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();

    omni::physx::usdparser::ObjectId sceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownerKey, ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);

    if (!physxScene)
    {
        CARB_LOG_ERROR("No PhysX Scene found for path %s", attachedStage.textFor(ownerKey));
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
        mirror.internalScene = physxScene->getInternalScene();
    }
    else
    {
        // Add mirror
        void* nm = copyAlignedMemory(internalActor.mMirrorMemory, internalActor.mMirrorMemsize);
        omni::physx::usdparser::ObjectId mirrorSceneId = omni::physx::usdparser::ObjectId(getObjectDataOrID<ObjectDataQueryType::eOBJECT_ID>(ownerKey, ePTScene, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
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
            internalActor.mMirrors.push_back({ nm, col, actor, mirrorScene->getInternalScene() });
        }
    }
}

bool omni::physx::updateBodySimulationOwner(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
            if (hasRelationship(attachedStage, objectRecord->mKey, property))
            {
                std::vector<omni::physics::parse::ObjectKey> owners;
                getRelationshipValue(attachedStage, objectRecord->mKey, property, owners);
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    bool data;
    if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    bool data;
    if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        internalActor->mSurfaceVelocityLocalSpace = data;

        // The flag reaches the simulation only through mSurfaceVelocity, which had the
        // body's scale folded in (or not) when the surface velocity was last derived.
        // Flipping the flag on its own would leave the actor driving the previously
        // derived vector, so re-derive it the way parsing does.
        //
        // The authored velocity comes from mSurfaceVelocityAuthored rather than from a
        // read of the surfaceVelocity attribute: this callback is draining one specific
        // change, and a cross-property read would resolve at the source's latest state.
        // If surfaceVelocity is authored after this flag but drained later, that read
        // would apply the newer velocity here, ahead of its own change.
        const PxVec3 authored = internalActor->mSurfaceVelocityAuthored;
        if (data)
            internalActor->mSurfaceVelocity = authored.multiply(toPhysX(internalActor->mScale));
        else
            internalActor->mSurfaceVelocity = authored;

        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        wakeActor(*actor);
    }

    return true;
}

bool omni::physx::updateBodySurfaceLinearVelocity(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    carb::Float3 data;
    if (!getValue<carb::Float3>(attachedStage, objectRecord->mKey, property, timeCode, data))
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        // Cache the authored value alongside the derived one, so a later change to
        // surfaceVelocityLocalSpace alone can re-derive without re-reading the source.
        const PxVec3 velocity = toPhysX(data);
        internalActor->mSurfaceVelocityAuthored = velocity;
        if (internalActor->mSurfaceVelocityLocalSpace)
            internalActor->mSurfaceVelocity = velocity.multiply(toPhysX(internalActor->mScale));
        else
            internalActor->mSurfaceVelocity = velocity;
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        wakeActor(*actor);
    }

    return true;
}


bool omni::physx::updateBodySplineSurfaceVelocityEnabled(AttachedStage& attachedStage,
                                                   omni::physx::usdparser::ObjectId objectId,
                                                   omni::physics::parse::TokenId property,
                                                   omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    bool data;
    if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        if (data)
        {
            const omni::physics::parse::ObjectKey bodyKey = objectRecord->mKey;
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);
            // An absent relationship and a defined-but-empty one are both invalid: a spline curve
            // target must be present.
            const bool hasSplinesRel = hasRelationship(attachedStage, bodyKey, tok.physxSplinesSurfaceVelocityCurve);
            std::vector<omni::physics::parse::ObjectKey> splinesList;
            if (hasSplinesRel)
            {
                getRelationshipValue(attachedStage, bodyKey, tok.physxSplinesSurfaceVelocityCurve, splinesList);
            }

            if (!hasSplinesRel || splinesList.empty())
            {
                CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined: "
                               "the physxSplinesSurfaceVelocity:surfaceVelocityCurve relationship has no target.",
                               attachedStage.textFor(bodyKey));

                return true;
            }
            else
            {
                // Same cause-naming split as the load-time validation in LoadStage.cpp.
                const omni::physics::parse::ObjectKey splineKey = splinesList[0];
                const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
                if (!(src && src->exists(splineKey)))
                {
                    CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined: "
                                   "the curve target %s is not present in the attached stage. When populating "
                                   "with ovstage, the physics population must include the referenced BasisCurves prim.",
                                   attachedStage.textFor(bodyKey), attachedStage.textFor(splineKey));

                    return true;
                }
                if (!src->isA(splineKey, tok.basisCurvesType))
                {
                    const std::string_view typeText = src->tokenToString(src->getTypeName(splineKey));
                    CARB_LOG_ERROR("Splines surface velocity %s does not have a valid spline curve defined: "
                                   "the curve target %s is not a BasisCurves prim (type '%.*s').",
                                   attachedStage.textFor(bodyKey), attachedStage.textFor(splineKey),
                                   int(typeText.size()), typeText.data());

                    return true;
                }
                // A body authored with splines disabled skipped the parse-time checks, so the
                // descendant rule has to be applied here too (same walk as LoadStage.cpp).
                bool parentBodyFound = false;
                const omni::physics::parse::ObjectKey root = src->getRootKey();
                for (omni::physics::parse::ObjectKey p = src->getParent(splineKey); p.valid() && p != root;
                     p = src->getParent(p))
                {
                    if (p == bodyKey)
                    {
                        parentBodyFound = true;
                        break;
                    }
                }
                if (!parentBodyFound)
                {
                    CARB_LOG_ERROR("Splines surface velocity %s spline curve is not a child of the rigid body.",
                                   attachedStage.textFor(bodyKey));

                    return true;
                }
                else
                {
                    internalActor->enableSplineSurfaceVelocity(data, *actor, attachedStage, splineKey);
                }
            }            
        }
        else
        {
            internalActor->enableSplineSurfaceVelocity(data, *actor, attachedStage, omni::physics::parse::ObjectKey{});
        }        
        wakeActor(*actor);
    }

    return true;
}

bool omni::physx::updateBodySplineSurfaceVelocityMagnitude(AttachedStage& attachedStage,
                                                  omni::physx::usdparser::ObjectId objectId,
                                                  omni::physics::parse::TokenId property,
                                                  omni::physics::parse::ReadTime timeCode)
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
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    carb::Float3 data;
    if (!getValue<carb::Float3>(attachedStage, objectRecord->mKey, property, timeCode, data))
        return true;

    if (internalType == ePTActor || internalType == ePTLink)
    {
        InternalActor* internalActor = (InternalActor*)objectRecord->mInternalPtr;
        internalActor->mSurfaceAngularVelocity = degToRad(toPhysX(data));
        PxRigidActor* actor = (PxRigidActor*)objectRecord->mPtr;
        wakeActor(*actor);
    }

    return true;
}

bool omni::physx::updatePhysxContactReportThreshold(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId,
    omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
