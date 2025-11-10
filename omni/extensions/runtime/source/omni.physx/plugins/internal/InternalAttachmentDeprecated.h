// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
class InternalParticleClothDeprecated;
class InternalDeformableBodyDeprecated;
class InternalDeformableSurfaceDeprecated;

class InternalAttachmentDeprecated : public Allocateable
{
public:
    enum AttachmentType
    {
        eUndefined,
        eDeformableBodyRigidBody,
        eParticleClothRigidBody,
        eDeformableSurfaceRigidBody,
        eDeformableBodyDeformableBody,
        eDeformableBodyParticleCloth,
        eDeformableBodyDeformableSurface,
        eDeformableBodyStatic,
        eParticleClothStatic,
        eDeformableSurfaceStatic
    };

    enum DirtyEventType
    {
        eNone = 0,
        eShapeRemoved = 1 << 0,
        eShapeAdded = 1 << 1,
        eAttachmentRemove = 1 << 2,
        eAttachmentAdd = 1 << 3,
        eRefreshAttachment = eAttachmentRemove | eAttachmentAdd,
    };

    struct AttachmentShapeRemovedEvent
    {
        pxr::SdfPath shapePath;
        usdparser::ObjectId removeShapeId;
    };

    struct AttachmentActor
    {
        // Vertex attachment
        std::vector<unsigned int> mParticleAttachmentIds;
        std::vector<unsigned int> mParticleAttachmentHandles;
        std::vector<unsigned int> mParticleFilterIds;

        // Tetrahedron attachment
        std::vector<unsigned int> mTetraAttachmentIds;
        std::vector<pxr::GfVec4f> mTetraAttachmentBaryCoords;
        std::vector<unsigned int> mTetraAttachmentHandles;
        std::vector<unsigned int> mTetraFilterIds;

        // Deformable attachment
        std::vector<::physx::PxU32> mDeformableAttachmentIndices;
        std::vector<::physx::PxVec4> mDeformableAttachmentCoords;

        // Deformable Element Filter
        std::vector<::physx::PxU32> mDeformableFilterIndices;

        ::physx::PxRigidActor* mRigidActor;
        std::vector<pxr::GfVec3f> mRigidActorPositions;
        usdparser::ObjectId mShapeId{ usdparser::kInvalidObjectId };

        // Store the object id of the actor
        usdparser::ObjectId mObjectId{ usdparser::kInvalidObjectId };
        void* mObjectInternalPtr{ nullptr };

        void reserveRigidActorData(size_t numAttachmentIds)
        {
            mRigidActorPositions.reserve(numAttachmentIds);
        }

        void reserveVertexData(size_t numAttachmentIds, size_t numFilterIds)
        {
            mParticleAttachmentIds.reserve(numAttachmentIds);
            mParticleAttachmentHandles.reserve(numAttachmentIds);
            mParticleFilterIds.reserve(numFilterIds);
        }

        void reserveTetrahedronData(size_t numAttachmentIds, size_t numFilterIds)
        {
            mTetraAttachmentIds.reserve(numAttachmentIds);
            mTetraAttachmentBaryCoords.reserve(numAttachmentIds);
            mTetraAttachmentHandles.reserve(numAttachmentIds);
            mTetraFilterIds.reserve(numFilterIds);
        }
    };

    InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody,
                                 ::physx::PxRigidActor* rigidActor,
                                 usdparser::ObjectId shapeId);
    InternalAttachmentDeprecated(InternalParticleClothDeprecated* particleCloth,
                                 ::physx::PxRigidActor* rigidActor,
                                 usdparser::ObjectId shapeId);
    InternalAttachmentDeprecated(InternalDeformableSurfaceDeprecated* deformableSurface,
                                 ::physx::PxRigidActor* rigidActor,
                                 usdparser::ObjectId shapeId);

    InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody0,
                                 InternalDeformableBodyDeprecated* deformableBody1);
    InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody,
                                 InternalParticleClothDeprecated* particleCloth);
    InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody,
                                 InternalDeformableSurfaceDeprecated* deformableSurface);

    InternalAttachmentDeprecated(InternalParticleClothDeprecated* particleCloth);
    InternalAttachmentDeprecated(InternalDeformableBodyDeprecated* deformableBody);
    InternalAttachmentDeprecated(InternalDeformableSurfaceDeprecated* deformableSurface);

    virtual ~InternalAttachmentDeprecated();

    void releaseDeformableRigidBodyAttachment();

    bool hasActor(const ::physx::PxRigidActor* rigidActor) const;
    bool hasShape(const usdparser::ObjectId shapeId) const;
    void swapRigidActor(::physx::PxRigidActor* sourceActor, ::physx::PxRigidActor* destActor);
    void setRefreshAttachmentEvent(::physx::PxRigidActor* rigidActor);
    void refreshAttachment();
    bool isValidParticleSystem();

    AttachmentType mAttachmentType{ eUndefined };
    AttachmentActor mAttachmentActor[2];

    ::physx::PxDeformableAttachment* mDeformableAttachment;
    ::physx::PxDeformableElementFilter* mDeformableFilter;

    AttachmentShapeRemovedEvent mAttachmentShapeRemovedEvent;

    ::physx::PxTransform mInitialTransformLocal{ ::physx::PxTransform(::physx::PxIdentity) };
    ::physx::PxTransform mNewTransformLocal{ ::physx::PxTransform(::physx::PxIdentity) };

    DirtyEventType mDirtyEvent{ eNone };
    InternalScene* mInternalScene{ nullptr };

    usdparser::ObjectId mObjectId{ usdparser::kInvalidObjectId };
    pxr::SdfPath mPath;
};


} // namespace internal
} // namespace physx
} // namespace omni

#ifdef _MSC_VER
#    pragma warning(pop)
#endif
