// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 */

#include "InternalDeformableAttachment.h"
#include "InternalDeformable.h"
#include "InternalScene.h"
#include "InternalActor.h"
#include "../PhysXScene.h"
#include <PhysXTools.h>

#include <PxArticulationLink.h>
#include <PxArticulationReducedCoordinate.h>

#include <common/utilities/MemoryMacros.h>

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using namespace ::physx;

extern ObjectId getObjectId(omni::physics::parse::ObjectKey key, PhysXType type);

using omni::physics::parse::ObjectKey;

namespace
{
// Diagnostic text for an endpoint key -- never null, empty string on miss (see
// AttachedStage::textFor's doc comment).
std::string textOf(ObjectKey key)
{
    AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    return as ? std::string(as->textFor(key)) : std::string();
}
} // namespace

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

bool matchingRigidBody(ObjectKey key, const PxActor* deformableActor, PxRigidActor*& rigidActor)
{
    PhysXType internalType = ePTRemoved;
    PxRigidActor* actor = nullptr;

    {
        PxArticulationLink* ptr = omni::physx::getPtr<PxArticulationLink>(ePTLink, getObjectId(key, ePTLink));
        if (ptr)
        {
            internalType = ePTLink;
            actor = ptr;
        }
    }

    {
        InternalActor* internalPtr = omni::physx::getInternalPtr<InternalActor>(ePTActor, getObjectId(key, ePTActor));
        if (internalPtr)
        {
            internalType = ePTActor;
            actor = internalPtr->mActor;
        }
    }

    {
        PxShape* ptr = omni::physx::getPtr<PxShape>(ePTShape, getObjectId(key, ePTShape));
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

PxActor* getPhysxActorFromPath(ObjectKey key, PhysXType& type)
{
    {
        InternalVolumeDeformableBody* internalPtr = omni::physx::getInternalPtr<InternalVolumeDeformableBody>(ePTDeformableVolume, getObjectId(key, ePTDeformableVolume));
        if (internalPtr)
        {
            type = ePTDeformableVolume;
            return internalPtr->mDeformableVolume;
        }
    }

    {
        InternalSurfaceDeformableBody* internalPtr = omni::physx::getInternalPtr<InternalSurfaceDeformableBody>(ePTDeformableSurface, getObjectId(key, ePTDeformableSurface));
        if (internalPtr)
        {
            type = ePTDeformableSurface;
            return internalPtr->mDeformableSurface;
        }
    }

    {
        if (getObjectId(key, ePTXformActor) != kInvalidObjectId)
        {
            type = ePTXformActor;
            return nullptr;
        }
    }

    {
        // Source-routed "is this endpoint an Xformable?" test. This used to be
        // UsdGeomXformable::Get(getActiveStage(), path), which needs a live UsdStage:
        // with none it posts a TF_CODING_ERROR ("Invalid stage") and hands back an
        // invalid schema object, so every xform-anchored endpoint fell through to
        // ePTRemoved and the attachment / element collision filter silently resolved
        // to kInvalidObjectId. This is the whole deformable<->rigid surface, not an
        // edge case: a UsdGeomCube carrying PhysicsRigidBodyAPI is an Xformable, so a
        // plain rigid body is classified here too and setupXformAttachment then walks
        // up to the real actor.
        //
        // isA() asks the same question of the parse source and is answered identically
        // by both backends (measured: Xform/Cube/Mesh -> true; UsdPhysicsScene,
        // Material, attachment prim and typeless prim -> false, on UsdSource and on
        // OvstageSource with no backing stage). IPhysicsSource::exists() is NOT a
        // substitute -- it is true for every live object, including the ones above.
        // "Xformable" is the registered USD schema-type name for UsdGeomXformable
        // (isA(key, internToken("Xformable")) is the same pxr-free pattern
        // OvstageWalker::isXformable uses for this exact check), so this needs no
        // pxr type at all -- schemaTypeToken<T>'s compile-time template is unnecessary here.
        const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        const omni::physics::parse::IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;

        if (src && src->isA(key, src->internToken("Xformable")))
        {
            type = ePTXformActor;
            return nullptr;
        }
    }

    type = ePTRemoved;
    return nullptr;
}

ObjectId InternalDeformableAttachment::createXformActor(ObjectKey key)
{
    PhysXType physxType;

    getPhysxActorFromPath(key, physxType);
    ObjectId objId = getObjectId(key, physxType);

    if (!(physxType == ePTXformActor && objId != kInvalidObjectId))
    {
        // No attached stage means there are no records to register against, so skip the add.
        AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        if (attachedStage)
        {
            objId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(
                ePTXformActor, nullptr, nullptr, key);

            attachedStage->getObjectDatabase()->findOrCreateEntry(key, attachedStage->textFor(key), eXformActor, objId);
        }
    }

    return objId;
}

void InternalDeformableAttachment::setupXformAttachment()
{
    AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage();
    const omni::physics::parse::IPhysicsSource* src = as ? as->getSource() : nullptr;

    PxMat44d xformToWorld(PxIdentity);
    if (as)
        xformToWorld = getWorldTransform(*as, mData[1].key, omni::physics::parse::ReadTime::defaultTime());

    decomposeMatrix(mLocalTransform, mScale, xformToWorld);

    mData[1].objId = createXformActor(mData[1].key);

    // Check if the xform is a child of a rigid body.
    //
    // Walks the ancestor chain by ObjectKey via IPhysicsSource::getParent (ADR-0019 decision 2:
    // hierarchy arithmetic routes through the source abstraction that already provides it), not
    // by SdfPath/UsdPrim. Checks the endpoint itself first, then each ancestor up to and
    // including the source's synthetic root -- getParent(key) returns that root for any
    // top-level object and the invalid sentinel once past it, so the walk terminates the same
    // way the old `== AbsoluteRootPath()` SdfPath walk did: the root is checked exactly once,
    // then the loop stops. No live UsdStage/UsdPrim involved.
    ObjectKey walkKey = mData[1].key;

    while (src && walkKey.valid())
    {
        {
            PxArticulationLink* ptr = omni::physx::getPtr<PxArticulationLink>(ePTLink, getObjectId(walkKey, ePTLink));
            if (ptr)
            {
                mData[1].actor = ptr;
                mData[1].rootKey = walkKey;
                mData[1].rootObjId = (ObjectId)mData[1].actor->userData;
                break;
            }
        }

        {
            InternalActor* internalPtr = omni::physx::getInternalPtr<InternalActor>(ePTActor, getObjectId(walkKey, ePTActor));
            if (internalPtr)
            {
                mData[1].actor = internalPtr->mActor;
                mData[1].rootKey = walkKey;
                mData[1].rootObjId = (ObjectId)mData[1].actor->userData;
                break;
            }
        }

        walkKey = src->getParent(walkKey);
    }

    if (mData[1].actor)
    {
        // Find matching rigid actor or mirrored actor
        PxRigidActor* rigidActor = nullptr;
        if (matchingRigidBody(mData[1].rootKey, mData[0].actor, rigidActor))
        {
            setRigidActor(rigidActor);

            // Find the child to parent transform
            PxMat44d rigidToWorld(PxIdentity);
            if (as)
                rigidToWorld = getWorldTransform(*as, mData[1].rootKey, omni::physics::parse::ReadTime::defaultTime());

            const PxVec3 scale = getScale(rigidToWorld);

            // Gf wrote this as `xformToWorld * rigidToWorld.GetInverse()`. Gf's
            // row-vector convention applies the left operand first, so that is
            // "xform-local -> world -> rigid-local". PhysX is column-vector, so the
            // same composition is written with the operands reversed; the matrices
            // themselves are the identical sixteen doubles and are NOT transposed.
            const PxMat44d xformToRigid = affineInverse(rigidToWorld) * xformToWorld;
            mLocalTransform = toTransform(xformToRigid);
            mLocalTransform.p = mLocalTransform.p.multiply(scale);
        }
    }
}

InternalDeformableAttachment::InternalDeformableAttachment(ObjectKey key, const PhysxDeformableAttachmentDesc& desc)
{
    mType = desc.type;
    mKey = key;

    mData[0].key = desc.src0;
    mData[1].key = desc.src1;

    for (uint32_t i = 0; i < 2; i++)
    {
        mData[i].actor = getPhysxActorFromPath(mData[i].key, mData[i].physxType);
        mData[i].objId = getObjectId(mData[i].key, mData[i].physxType);
    }

    if (mData[0].actor == nullptr)
    {
        if (mData[1].actor == nullptr)
        {
            std::string errorStr = "Physics Deformable Attachment " + textOf(key) + " cannot have 2 invalid actors.";
            PhysXUsdPhysicsInterface::reportLoadError(usdparser::ErrorCode::eError, errorStr.c_str());
        }
        else
        {
            std::string errorStr = "Physics Deformable Attachment " + textOf(key) + " cannot have an invalid deformable actor.";
            PhysXUsdPhysicsInterface::reportLoadError(usdparser::ErrorCode::eError, errorStr.c_str());
        }

        return;
    }

    if (mData[1].physxType == ePTXformActor)
    {
        setupXformAttachment();
    }

    if (mData[1].actor == nullptr)
    {
        const std::string errorStr = "Physics Deformable Attachment " + textOf(key) +
            ": no rigid actor at or above " + textOf(desc.src1) + ".";
        PhysXUsdPhysicsInterface::reportLoadError(usdparser::ErrorCode::eWarning, errorStr.c_str());
    }

    // For attachments that have a root prim, we need to track the root obj id also.
    for (uint32_t i = 0; i < 2; i++)
    {
        if (mData[i].physxType == ePTDeformableVolume || mData[i].physxType == ePTDeformableSurface)
        {
            mData[i].rootObjId = (ObjectId)mData[i].actor->userData;
        }
    }

    // [OMPE-92466] For an articulation link, PxActor::getScene() returns null
    // until finalizeArticulations runs (after createDeformableAttachments), so
    // relying on it alone would silently drop attachments whose rigid side is
    // an articulation link. Fall back to
    // PxArticulationLink::getArticulation().getScene(), then to the
    // omni.physx-side InternalLink/InternalActor mPhysXScene, which is valid
    // throughout load.
    auto resolveInternalScene = [](PxActor* actor, ObjectKey key) -> InternalScene* {
        if (actor)
        {
            if (PxScene* s = actor->getScene())
                return getInternalPtr<InternalScene>(ePTScene, omni::physx::usdparser::ObjectId(s->userData));
            if (actor->getConcreteType() == PxConcreteType::eARTICULATION_LINK)
            {
                PxArticulationLink* link = static_cast<PxArticulationLink*>(actor);
                if (PxScene* s = link->getArticulation().getScene())
                    return getInternalPtr<InternalScene>(ePTScene, omni::physx::usdparser::ObjectId(s->userData));
            }
        }
        if (InternalLink* link = getInternalPtr<InternalLink>(ePTLink, getObjectId(key, ePTLink)))
            if (link->mPhysXScene)
                return link->mPhysXScene->getInternalScene();
        if (InternalActor* a = getInternalPtr<InternalActor>(ePTActor, getObjectId(key, ePTActor)))
            if (a->mPhysXScene)
                return a->mPhysXScene->getInternalScene();
        return nullptr;
    };

    mInternalScene = resolveInternalScene(mData[0].actor, mData[0].key);
    if (!mInternalScene)
        mInternalScene = resolveInternalScene(mData[1].actor, mData[1].key);
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
        const std::string errorStr = "Failed to create Physics Deformable Attachment " + textOf(mKey);
        PhysXUsdPhysicsInterface::reportLoadError(usdparser::ErrorCode::eError, errorStr.c_str());
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
                PxMat44d xformToWorld(PxIdentity);
                PxMat44d rigidToWorld(PxIdentity);
                if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
                {
                    xformToWorld = getWorldTransform(*as, mData[1].key, omni::physics::parse::ReadTime::defaultTime());
                    rigidToWorld = getWorldTransform(*as, mData[1].rootKey, omni::physics::parse::ReadTime::defaultTime());
                }

                const PxVec3 scale = getScale(rigidToWorld);

                // Operands reversed relative to the old Gf `xformToWorld *
                // rigidToWorld.GetInverse()`: same composition order under PhysX's
                // column-vector convention, no transpose. See setupXformAttachment.
                const PxMat44d xformToRigid = affineInverse(rigidToWorld) * xformToWorld;
                mLocalTransform = toTransform(xformToRigid);
                mLocalTransform.p = mLocalTransform.p.multiply(scale);
            }
            else
            {
                PxMat44d localToWorld(PxIdentity);
                if (AttachedStage* as = UsdLoad::getUsdLoad()->getActiveAttachedStage())
                    localToWorld = getWorldTransform(*as, mData[1].key, omni::physics::parse::ReadTime::defaultTime());
                mLocalTransform = toTransform(localToWorld);
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

InternalDeformableCollisionFilter::InternalDeformableCollisionFilter(ObjectKey key, const PhysxDeformableCollisionFilterDesc& desc)
{
    mKey = key;

    const ObjectKey srcKeys[2] = { desc.src0, desc.src1 };
    mData[0].key = desc.src0;
    mData[1].key = desc.src1;

    for (uint32_t i = 0; i < 2; i++)
    {
        mActorIndex[i] = i;

        mData[i].actor = getPhysxActorFromPath(mData[i].key, mData[i].physxType);

        if (mData[i].physxType == ePTRemoved)
        {
            std::string errorStr = "Physics Element Collision Filter " + textOf(key) + " has an invalid actor " + textOf(srcKeys[i]);
            PhysXUsdPhysicsInterface::reportLoadError(usdparser::ErrorCode::eError, errorStr.c_str());
            return;
        }

        mData[i].objId = getObjectId(mData[i].key, mData[i].physxType);
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
        if (matchingRigidBody(mData[mActorIndex[1]].key, mData[mActorIndex[0]].actor, rigidActor))
        {
            setRigidActor(rigidActor);
        }
        else
        {
            // Actor cannot be null for collision filtering
            std::string errorStr = "Physics Element Collision Filter " + textOf(key) + " has an invalid actor " + textOf(srcKeys[mActorIndex[1]]);
            PhysXUsdPhysicsInterface::reportLoadError(usdparser::ErrorCode::eError, errorStr.c_str());
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
        switch (mData[mActorIndex[0]].physxType)
        {
            case ePTDeformableSurface:
            case ePTDeformableVolume:
            {
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
        const std::string errorStr = "Failed to create Physics Element Collision Filter " + textOf(mKey);
        PhysXUsdPhysicsInterface::reportLoadError(usdparser::ErrorCode::eError, errorStr.c_str());
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
