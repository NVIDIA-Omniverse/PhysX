// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "InternalDeformableAttachment.h"
#include "InternalDeformable.h"
#include "InternalScene.h"
#include <PhysXTools.h>

#include <common/utilities/MemoryMacros.h>

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace carb;
using namespace ::physx;

extern ObjectId getObjectId(const pxr::SdfPath& path, PhysXType type);

bool checkScenes(const PxActor* actor0, const PxActor* actor1)
{
    if (!actor0 || !actor1)
        return false;

    PxScene* s0 = actor0->getScene();
    PxScene* s1 = actor1->getScene();

    if (!s0 && !s1)
        return false;

    if (!s0 || !s1)
        return true;

    return s0 == s1;
}

bool matchingRigidBody(pxr::SdfPath path, const PxActor* deformableActor, PxRigidActor*& rigidActor)
{
    PhysXType internalType = ePTRemoved;
    PxRigidActor* actor = nullptr;

    {
        PxArticulationLink* ptr = omni::physx::getPtr<PxArticulationLink>(ePTLink, getObjectId(path, ePTLink));
        if (ptr)
        {
            internalType = ePTLink;
            actor = ptr;
        }
    }

    {
        InternalActor* internalPtr = omni::physx::getInternalPtr<InternalActor>(ePTActor, getObjectId(path, ePTActor));
        if (internalPtr)
        {
            internalType = ePTActor;
            actor = internalPtr->mActor;
        }
    }

    {
        PxShape* ptr = omni::physx::getPtr<PxShape>(ePTShape, getObjectId(path, ePTShape));
        if (ptr)
        {
            internalType = ePTShape;
            actor = ptr->getActor();
        }
    }

    if (actor)
    {
        if (checkScenes(actor, deformableActor))
        {
            rigidActor = actor;
            return true;
        }
        else
        {
            const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            const omni::physx::internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
            const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, (ObjectId)actor->userData);

            InternalActor* intActor = (InternalActor*)objectRecord->mInternalPtr;

            // Loop through mirrored scenes
            for (int i = 0; i < intActor->mMirrors.size(); i++)
            {
                if (checkScenes(intActor->mMirrors[i].actor, deformableActor))
                {
                    rigidActor = intActor->mMirrors[i].actor;
                    return true;
                }
            }
        }
    }

    return false;
}

PxActor* getPhysxActorFromPath(pxr::SdfPath path, PhysXType& type)
{
    {
        InternalVolumeDeformableBody* internalPtr = omni::physx::getInternalPtr<InternalVolumeDeformableBody>(ePTDeformableVolume, getObjectId(path, ePTDeformableVolume));
        if (internalPtr)
        {
            type = ePTDeformableVolume;
            return internalPtr->mDeformableVolume;
        }
    }

    {
        InternalSurfaceDeformableBody* internalPtr = omni::physx::getInternalPtr<InternalSurfaceDeformableBody>(ePTDeformableSurface, getObjectId(path, ePTDeformableSurface));
        if (internalPtr)
        {
            type = ePTDeformableSurface;
            return internalPtr->mDeformableSurface;
        }
    }

    {
        if (getObjectId(path, ePTXformActor) != kInvalidObjectId)
        {
            type = ePTXformActor;
            return nullptr;
        }
    }

    {
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        pxr::UsdGeomXformable xformable = pxr::UsdGeomXformable::Get(omniPhysX.getStage(), path);

        if (xformable)
        {
            type = ePTXformActor;
            return nullptr;
        }
    }

    type = ePTRemoved;
    return nullptr;
}

ObjectId InternalDeformableAttachment::createXformActor(pxr::SdfPath path)
{
    PhysXType physxType;

    getPhysxActorFromPath(path, physxType);
    ObjectId objId = getObjectId(path, physxType);

    if (!(physxType == ePTXformActor && objId != kInvalidObjectId))
    {
        // Add xform actor record
        objId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTXformActor, nullptr, nullptr, path);

        const uint64_t stageId = OmniPhysX::getInstance().getStageId();
        AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
        attachedStage->getObjectDatabase()->findOrCreateEntry(path, eXformActor, objId);
    }

    return objId;
}

void InternalDeformableAttachment::setupXformAttachment()
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    pxr::UsdGeomImageable imageable = pxr::UsdGeomImageable::Get(omniPhysX.getStage(), mData[1].path);
    pxr::GfMatrix4d xformToWorld;
    xformToWorld.SetIdentity();
    if (imageable)
    {
        xformToWorld = imageable.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
    }

    const pxr::GfTransform tr(xformToWorld);
    mLocalTransform = toPhysX(tr);
    mScale = toPhysX(tr.GetScale());

    mData[1].objId = createXformActor(mData[1].path);

    pxr::UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    pxr::UsdPrim usdPrim = stage->GetPrimAtPath(mData[1].path);

    // Check if the xform is a child of a rigid body
    while (usdPrim)
    {
        pxr::SdfPath path = usdPrim.GetPath();

        {
            PxArticulationLink* ptr = omni::physx::getPtr<PxArticulationLink>(ePTLink, getObjectId(path, ePTLink));
            if (ptr)
            {
                mData[1].actor = ptr;
                mData[1].rootPath = path;
                mData[1].rootObjId = (ObjectId)mData[1].actor->userData;
                break;
            }
        }

        {
            InternalActor* internalPtr = omni::physx::getInternalPtr<InternalActor>(ePTActor, getObjectId(path, ePTActor));
            if (internalPtr)
            {
                mData[1].actor = internalPtr->mActor;
                mData[1].rootPath = path;
                mData[1].rootObjId = (ObjectId)mData[1].actor->userData;
                break;
            }
        }

        usdPrim = usdPrim.GetParent();
    }

    if (mData[1].actor)
    {
        // Find matching rigid actor or mirrored actor
        PxRigidActor* rigidActor = nullptr;
        if (matchingRigidBody(usdPrim.GetPath(), mData[0].actor, rigidActor))
        {
            setRigidActor(rigidActor);

            // Find the child to parent transform
            pxr::UsdGeomImageable imageable = pxr::UsdGeomImageable::Get(omniPhysX.getStage(), usdPrim.GetPath());
            pxr::GfMatrix4d rigidToWorld;
            rigidToWorld.SetIdentity();
            if (imageable)
            {
                rigidToWorld = imageable.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
            }

            const pxr::GfTransform temp(rigidToWorld);
            PxVec3 scale = toPhysX(temp.GetScale());

            pxr::GfMatrix4d xformToRigid = xformToWorld * rigidToWorld.GetInverse();
            const pxr::GfTransform tr(xformToRigid);
            mLocalTransform = toPhysX(tr);
            mLocalTransform.p = mLocalTransform.p.multiply(scale);
        }
    }
}

InternalDeformableAttachment::InternalDeformableAttachment(pxr::SdfPath path, const PhysxDeformableAttachmentDesc& desc)
{
    mType = desc.type;
    mPath = path;

    mData[0].path = desc.src0;
    mData[1].path = desc.src1;

    for (uint32_t i = 0; i < 2; i++)
    {
        mData[i].actor = getPhysxActorFromPath(mData[i].path, mData[i].physxType);
        mData[i].objId = getObjectId(mData[i].path, mData[i].physxType);
    }

    // Sanity check that deformable actor is not null.
    if (mData[0].actor == nullptr)
    {
        // Sanity check that both actors are not null
        if (mData[1].actor == nullptr)
        {
            std::string errorStr = "Physics Deformable Attachment " + mPath.GetString() + " cannot have 2 invalid actors.";
            PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, errorStr.c_str());
        }
        else
        {
            std::string errorStr = "Physics Deformable Attachment " + mPath.GetString() + " cannot have an invalid deformable actor.";
            PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, errorStr.c_str());
        }

        return;
    }

    if (mData[1].physxType == ePTXformActor)
    {
        setupXformAttachment();
    }

    // For attachments that have a root prim, we need to track the root obj id also.
    for (uint32_t i = 0; i < 2; i++)
    {
        if (mData[i].physxType == ePTDeformableVolume || mData[i].physxType == ePTDeformableSurface)
        {
            mData[i].rootObjId = (ObjectId)mData[i].actor->userData;
        }
    }

    if (mData[1].actor == nullptr || mData[0].actor->getScene() == mData[1].actor->getScene())
    {
        mInternalScene = getInternalPtr<InternalScene>(ePTScene, omni::physx::usdparser::ObjectId(mData[0].actor->getScene()->userData));
    }
}

InternalDeformableAttachment::~InternalDeformableAttachment()
{
    SAFE_RELEASE(mDeformableAttachment);
}

bool InternalDeformableAttachment::isValid()
{
    if (mInternalScene)
    {
        switch (mType)
        {
            case eAttachmentVtxXform:
            {
                if ((mData[0].physxType == ePTDeformableSurface || mData[0].physxType == ePTDeformableVolume) &&
                    (mData[1].physxType == ePTXformActor))
                {
                    return true;
                }
                break;
            }

            case eAttachmentTetXform:
            {
                if ((mData[0].physxType == ePTDeformableVolume) &&
                    (mData[1].physxType == ePTXformActor))
                {
                    return true;
                }
                break;
            }

            case eAttachmentVtxVtx:
            {
                if ((mData[0].physxType == ePTDeformableSurface || mData[0].physxType == ePTDeformableVolume) &&
                    (mData[1].physxType == ePTDeformableSurface || mData[1].physxType == ePTDeformableVolume))
                {
                    return true;
                }
                break;
            }

            case eAttachmentVtxTri:
            {
                if ((mData[0].physxType == ePTDeformableSurface || mData[0].physxType == ePTDeformableVolume) &&
                    (mData[1].physxType == ePTDeformableSurface))
                {
                    return true;
                }
                break;
            }

            case eAttachmentVtxTet:
            {
                if ((mData[0].physxType == ePTDeformableSurface || mData[0].physxType == ePTDeformableVolume) &&
                    (mData[1].physxType == ePTDeformableVolume))
                {
                    return true;
                }
                break;
            }
        }
    }

    return false;
}

void InternalDeformableAttachment::create()
{
    switch (mType)
    {
        case eAttachmentVtxXform:
        case eAttachmentTetXform:
        case eAttachmentVtxVtx:
        case eAttachmentVtxTri:
        case eAttachmentVtxTet:
        {
            PxDeformableAttachmentData data;

            data.actor[0] = mData[0].actor;
            data.type[0] = mData[0].targetType;
            data.indices[0].data = (PxU32*)mData[0].indices.data();
            data.indices[0].count = (PxU32)mData[0].indices.size();
            data.coords[0].data = (PxVec4*)mData[0].coords.data();
            data.coords[0].count = (PxU32)mData[0].coords.size();

            data.actor[1] = mData[1].actor;
            data.type[1] = mData[1].targetType;
            data.indices[1].data = (PxU32*)mData[1].indices.data();
            data.indices[1].count = (PxU32)mData[1].indices.size();
            data.coords[1].data = (PxVec4*)mData[1].coords.data();
            data.coords[1].count = (PxU32)mData[1].coords.size();

            if (mType == eAttachmentVtxXform || mType == eAttachmentTetXform)
            {
                data.pose[1] = mLocalTransform;
            }

            mDeformableAttachment = mInternalScene->getScene()->getPhysics().createDeformableAttachment(data);

            break;
        }

        default:
        {
            CARB_ASSERT(0);
            break;
        }
    }

    if (mDeformableAttachment == nullptr)
    {
        std::string errorStr = "Failed to create Physics Deformable Attachment " + mPath.GetString();
        PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, errorStr.c_str());
    }
    else
    {
        // Wake up the deformable actors
        for (uint32_t i = 0; i < 2; i++)
        {
            switch (mData[i].physxType)
            {
                case ePTDeformableSurface:
                case ePTDeformableVolume:
                {
                    PxDeformableBody* actor = (PxDeformableBody*)mData[i].actor;
                    if (actor->isSleeping())
                    {
                        actor->setWakeCounter(actor->getWakeCounter());
                    }
                    break;
                }
            }
        }
    }
}

void InternalDeformableAttachment::update()
{
    if (mDirtyEvent & eRefreshAttachment)
    {
        if (mDirtyEvent & eRemoveAttachment)
        {
            SAFE_RELEASE(mDeformableAttachment);
        }

        if (mDirtyEvent & eCreateAttachment)
        {
            create();
        }
    }
    else if (mDirtyEvent & eUpdateXform)
    {
        if (mDeformableAttachment)
        {
            if (mData[1].actor)
            {
                const OmniPhysX& omniPhysX = OmniPhysX::getInstance();

                pxr::UsdGeomImageable imageableXform = pxr::UsdGeomImageable::Get(omniPhysX.getStage(), mData[1].path);
                pxr::GfMatrix4d xformToWorld;
                xformToWorld.SetIdentity();
                if (imageableXform)
                {
                    xformToWorld = imageableXform.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
                }

                pxr::UsdGeomImageable imageableRigid = pxr::UsdGeomImageable::Get(omniPhysX.getStage(), mData[1].rootPath);
                pxr::GfMatrix4d rigidToWorld;
                rigidToWorld.SetIdentity();
                if (imageableRigid)
                {
                    rigidToWorld = imageableRigid.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
                }

                const pxr::GfTransform temp(rigidToWorld);
                PxVec3 scale = toPhysX(temp.GetScale());

                pxr::GfMatrix4d xformToRigid = xformToWorld * rigidToWorld.GetInverse();
                const pxr::GfTransform tr(xformToRigid);
                mLocalTransform = toPhysX(tr);
                mLocalTransform.p = mLocalTransform.p.multiply(scale);
            }
            else
            {
                const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
                pxr::UsdGeomImageable imageable = pxr::UsdGeomImageable::Get(omniPhysX.getStage(), mData[1].path);
                pxr::GfMatrix4d localToWorld;
                localToWorld.SetIdentity();
                if (imageable)
                {
                    localToWorld = imageable.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
                }
                mLocalTransform = toPhysX(pxr::GfTransform(localToWorld));
            }

            mDeformableAttachment->updatePose(mLocalTransform);
        }
    }

    mDirtyEvent = eNone;
}

void InternalDeformableAttachment::swapRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor)
{
    if (hasRigidActor(sourceActor))
    {
        // Remove attachment from old rigid actor
        mDirtyEvent = eRemoveAttachment;
        update();

        // Set pending event for new rigid actor
        setRigidActor(destActor);
        mDirtyEvent = eCreateAttachment;
    }
}

void InternalDeformableAttachment::setCreateAttachmentEvent()
{
    if (mDirtyEvent == eNone)
    {
        mDirtyEvent = eCreateAttachment;
    }
}

void InternalDeformableAttachment::setRefreshAttachmentEvent(::physx::PxRigidActor* rigidActor)
{
    if (hasRigidActor(rigidActor))
    {
        if (mDirtyEvent == eNone)
        {
            mDirtyEvent = eRefreshAttachment;
        }
    }
}

void InternalDeformableAttachment::setUpdateXformEvent(usdparser::ObjectId objId)
{
    if (hasXformActor(objId))
    {
        if (mDirtyEvent == eNone)
        {
            mDirtyEvent = eUpdateXform;
        }
    }
}

InternalDeformableCollisionFilter::InternalDeformableCollisionFilter(pxr::SdfPath path, const PhysxDeformableCollisionFilterDesc& desc)
{
    mPath = path;

    mData[0].path = desc.src0;
    mData[1].path = desc.src1;

    for (uint32_t i = 0; i < 2; i++)
    {
        mActorIndex[i] = i;

        mData[i].actor = getPhysxActorFromPath(mData[i].path, mData[i].physxType);

        if (mData[i].physxType == ePTRemoved)
        {
            std::string errorStr = "Physics Element Collision Filter " + mPath.GetString() + " has an invalid actor " + mData[i].path.GetString();
            PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, errorStr.c_str());
            return;
        }

        mData[i].objId = getObjectId(mData[i].path, mData[i].physxType);
    }

    // For ease of management, actor 0 is always a deformable
    if (mData[0].actor == nullptr || mData[0].actor->is<PxRigidBody>())
    {
        PxSwap(mActorIndex[0], mActorIndex[1]);
    }

    if (mData[mActorIndex[1]].physxType == ePTXformActor)
    {
        // Find matching rigid actor or mirrored actor
        PxRigidActor* rigidActor = nullptr;
        if (matchingRigidBody(mData[mActorIndex[1]].path, mData[mActorIndex[0]].actor, rigidActor))
        {
            setRigidActor(rigidActor);
        }
        else
        {
            // Actor cannot be null for collision filtering
            std::string errorStr = "Physics Element Collision Filter " + mPath.GetString() + " has an invalid actor " + mData[mActorIndex[1]].path.GetString();
            PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, errorStr.c_str());
            return;
        }
    }

    // For attachments that have a root prim, we need to track the root obj id also.
    for (uint32_t i = 0; i < 2; i++)
    {
        if (mData[i].physxType == ePTDeformableVolume || mData[i].physxType == ePTDeformableSurface || mData[i].physxType == ePTXformActor)
        {
            mData[i].rootObjId = (ObjectId)mData[i].actor->userData;
        }
    }

    if (mData[mActorIndex[0]].actor->getScene() == mData[mActorIndex[1]].actor->getScene())
    {
        mInternalScene = getInternalPtr<InternalScene>(ePTScene, omni::physx::usdparser::ObjectId(mData[mActorIndex[0]].actor->getScene()->userData));
    }
}

InternalDeformableCollisionFilter::~InternalDeformableCollisionFilter()
{
    SAFE_RELEASE(mDeformableElementFilter);
}

bool InternalDeformableCollisionFilter::isValid()
{
    if (mInternalScene)
    {
        // Check valid physx type for actor 0
        switch (mData[mActorIndex[0]].physxType)
        {
            case ePTDeformableSurface:
            case ePTDeformableVolume:
            {
                // Check valid physx type for actor 1
                switch (mData[mActorIndex[1]].physxType)
                {
                    case ePTDeformableSurface:
                    case ePTDeformableVolume:
                    case ePTXformActor:
                    {
                        return true;
                    }
                    break;
                }
            }
            break;
        }
    }

    return false;
}

void InternalDeformableCollisionFilter::create()
{
    PxDeformableElementFilterData data;

    data.actor[0] = mData[0].actor;
    data.groupElementCounts[0].data = mData[0].groupElementCounts.data();
    data.groupElementCounts[0].count = PxU32(mData[0].groupElementCounts.size());
    data.groupElementIndices[0].data = mData[0].groupElementIndices.data();
    data.groupElementIndices[0].count = PxU32(mData[0].groupElementIndices.size());

    data.actor[1] = mData[1].actor;
    data.groupElementCounts[1].data = mData[1].groupElementCounts.data();
    data.groupElementCounts[1].count = PxU32(mData[1].groupElementCounts.size());
    data.groupElementIndices[1].data = mData[1].groupElementIndices.data();
    data.groupElementIndices[1].count = PxU32(mData[1].groupElementIndices.size());

    mDeformableElementFilter = mInternalScene->getScene()->getPhysics().createDeformableElementFilter(data);

    if (mDeformableElementFilter == nullptr)
    {
        std::string errorStr = "Failed to create Physics Element Collision Filter " + mPath.GetString();
        PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eError, errorStr.c_str());
    }
}

void InternalDeformableCollisionFilter::update()
{
    if (mDirtyEvent & eRefreshCollisionFilter)
    {
        if (mDirtyEvent & eRemoveCollisionFilter)
        {
            SAFE_RELEASE(mDeformableElementFilter);
        }

        if (mDirtyEvent & eCreateCollisionFilter)
        {
            create();
        }
    }

    mDirtyEvent = eNone;
}

void InternalDeformableCollisionFilter::swapRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor)
{
    if (hasRigidActor(sourceActor))
    {
        // Remove collision filter from old rigid actor
        mDirtyEvent = eRemoveCollisionFilter;
        update();

        // Set pending event for new rigid actor
        setRigidActor(destActor);
        mDirtyEvent = eCreateCollisionFilter;
    }
}

void InternalDeformableCollisionFilter::setCreateCollisionFilterEvent()
{
    if (mDirtyEvent == eNone)
    {
        mDirtyEvent = eCreateCollisionFilter;
    }
}

void InternalDeformableCollisionFilter::setRefreshCollisionFilterEvent(::physx::PxRigidActor* rigidActor)
{
    if (hasRigidActor(rigidActor))
    {
        if (mDirtyEvent == eNone)
        {
            mDirtyEvent = eRefreshCollisionFilter;
        }
    }
}
