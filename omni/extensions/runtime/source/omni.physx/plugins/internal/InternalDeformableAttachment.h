// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#ifdef _MSC_VER
#    pragma warning(push)
#    define NOMINMAX // Make sure nobody #defines min or max
#endif

#ifdef __linux__
#    define __forceinline __attribute__((always_inline))
#endif

#include "UsdPCH.h"

#include <PxPhysicsAPI.h>

#include "Internal.h"
#include <PhysXDefines.h>

namespace omni
{
namespace physx
{
namespace internal
{

class InternalScene;

enum DirtyEventType
{
    eNone = 0,

    eShapeRemoved = 1 << 0,
    eShapeAdded = 1 << 1,

    eRemoveAttachment = 1 << 2,
    eCreateAttachment = 1 << 3,
    eRefreshAttachment = eRemoveAttachment | eCreateAttachment,

    // Mirror the attachment events for collision filter
    eRemoveCollisionFilter = eRemoveAttachment,
    eCreateCollisionFilter = eCreateAttachment,
    eRefreshCollisionFilter = eRefreshAttachment,

    eUpdateXform = 1 << 4,
};

struct ShapeRemovedEvent
{
    pxr::SdfPath shapePath;
    usdparser::ObjectId removeShapeId;
};

class InternalDeformableAttachment : public Allocateable
{
public:
    struct AttachmentData
    {
        PhysXType physxType{ ePTRemoved };
        pxr::SdfPath path;
        usdparser::ObjectId objId{ usdparser::kInvalidObjectId };

        ::physx::PxActor* actor;
        ::physx::PxDeformableAttachmentTargetType::Enum targetType;
        std::vector<int> indices;
        std::vector<carb::Float4> coords;

        usdparser::ObjectId rootObjId{ usdparser::kInvalidObjectId };
        pxr::SdfPath rootPath;
    };

    InternalDeformableAttachment(pxr::SdfPath path, const usdparser::PhysxDeformableAttachmentDesc& desc);
    virtual ~InternalDeformableAttachment();

    bool isValid();
    void create();
    void update();

    void swapRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor);
    void setCreateAttachmentEvent();
    void setRefreshAttachmentEvent(::physx::PxRigidActor* rigidActor);
    void setUpdateXformEvent(usdparser::ObjectId objId);

    usdparser::ObjectId createXformActor(pxr::SdfPath path);
    void setupXformAttachment();

    __forceinline bool hasRigidActor(const ::physx::PxRigidActor* rigidActor) const
    {
        return (mData[1].actor != nullptr && mData[1].actor == rigidActor) ? true : false;
    }

    __forceinline void setRigidActor(::physx::PxRigidActor* rigidActor)
    {
        if (mData[1].physxType == ePTXformActor)
            mData[1].actor = rigidActor;
    }

    __forceinline bool hasXformActor(const usdparser::ObjectId objId) const
    {
        return (mData[1].physxType == ePTXformActor && (mData[1].objId == objId || mData[1].rootObjId == objId)) ? true :
                                                                                                                   false;
    }

    AttachmentData mData[2];

    ::physx::PxTransform mLocalTransform{ ::physx::PxTransform(::physx::PxIdentity) };
    ::physx::PxVec3 mScale{ ::physx::PxVec3(1.0f) };

    usdparser::ObjectType mType{ usdparser::ObjectType::eDeformableAttachment };
    pxr::SdfPath mPath;
    usdparser::ObjectId mObjectId{ usdparser::kInvalidObjectId };

    ::physx::PxDeformableAttachment* mDeformableAttachment{ nullptr };
    InternalScene* mInternalScene{ nullptr };

    // Event handling
    DirtyEventType mDirtyEvent{ eNone };
    ShapeRemovedEvent mShapeRemovedEvent;
};

class InternalDeformableCollisionFilter : public Allocateable
{
public:
    struct CollisionFilterData
    {
        PhysXType physxType{ ePTRemoved };
        pxr::SdfPath path;
        usdparser::ObjectId objId{ usdparser::kInvalidObjectId };

        ::physx::PxActor* actor;
        std::vector<unsigned int> groupElementCounts;
        std::vector<unsigned int> groupElementIndices;

        usdparser::ObjectId rootObjId{ usdparser::kInvalidObjectId };
    };

    InternalDeformableCollisionFilter(pxr::SdfPath path, const usdparser::PhysxDeformableCollisionFilterDesc& desc);
    virtual ~InternalDeformableCollisionFilter();

    bool isValid();
    void create();
    void update();

    void swapRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor);
    void setCreateCollisionFilterEvent();
    void setRefreshCollisionFilterEvent(::physx::PxRigidActor* rigidActor);

    __forceinline bool hasRigidActor(const ::physx::PxRigidActor* rigidActor) const
    {
        return (mData[mActorIndex[1]].actor == rigidActor) ? true : false;
    }

    __forceinline void setRigidActor(::physx::PxRigidActor* rigidActor)
    {
        if (mData[mActorIndex[1]].physxType == ePTXformActor)
            mData[mActorIndex[1]].actor = rigidActor;
    }

    CollisionFilterData mData[2];
    int mActorIndex[2];

    pxr::SdfPath mPath;
    usdparser::ObjectId mObjectId{ usdparser::kInvalidObjectId };

    ::physx::PxDeformableElementFilter* mDeformableElementFilter{ nullptr };
    InternalScene* mInternalScene{ nullptr };

    // Event handling
    DirtyEventType mDirtyEvent{ eNone };
    ShapeRemovedEvent mShapeRemovedEvent;
};

} // namespace internal
} // namespace physx
} // namespace omni

#ifdef _MSC_VER
#    pragma warning(pop)
#endif
