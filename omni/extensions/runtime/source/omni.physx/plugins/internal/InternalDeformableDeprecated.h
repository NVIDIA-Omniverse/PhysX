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
#include <internal/InternalXformOpResetStorage.h>


namespace omni
{
namespace physx
{
class PhysXScene;

namespace internal
{

class InternalDeformableDeprecated : public Allocateable
{
public:
    InternalDeformableDeprecated() : mMaterialId(usdparser::kInvalidObjectId), mFlags(0)
    {
    }

    usdparser::ObjectId mMaterialId;
    uint32_t mFlags;

    XformOpResetStorage mXformOpStorage;
    bool mSkipUpdateTransform;

    PhysXScene* mPhysXScene;
};

class InternalDeformableBodyDeprecated : public InternalDeformableDeprecated
{
public:
    InternalDeformableBodyDeprecated(PhysXScene* ps, pxr::UsdPrim& prim);
    ~InternalDeformableBodyDeprecated();

    ::physx::PxVec4* getKinematicTargetsH();
    void uploadKinematicTargets(::physx::PxSoftBodyFlags flags);

    ::physx::PxSoftBody* mSoftBody;
    ::physx::PxSoftBodyMesh* mSoftBodyMesh;

    ::physx::PxVec4* mSimPositionInvMassH; // pinned host memory
    ::physx::PxVec4* mSimVelocityH; // pinned host memory
    ::physx::PxVec4* mCollPositionInvMassH; // pinned host memory
    ::physx::PxVec4* mCollRestPositionH; // pinned host memory

    ::physx::PxVec4* mKinematicTargetsH; // pinned host memory
    ::physx::PxVec4* mKinematicTargetsD; // device memory

    pxr::UsdPrim mPrim;
    pxr::GfMatrix4d mPrimToWorld;

    std::vector<carb::Uint3> mCollMeshSurfaceTriangles; // referencing collision mesh vertices

    unsigned int mNumCollMeshVertices;
    unsigned int mNumSimMeshVertices;
    unsigned int mNumSkinMeshVertices;
    unsigned int mNumCollSkinMeshVertices;

    pxr::GfMatrix4d mInitialPrimToParent;
    std::vector<carb::Float3> mInitialCollSkinMeshPositions;
    std::vector<carb::Float3> mInitialSimMeshPositions;
    std::vector<carb::Float3> mInitialSimMeshVelocities;
    pxr::VtVec3fArray mExtentSaveRestoreBuf;

    bool mIsPartiallyKinematic;

    std::vector<carb::Uint3> mCollisionVertexToSkinTriVertexIndices;
    std::vector<carb::Float3> mCollisionVertexToSkinTriBarycentrics;

    bool mKinematicTargetsSet;

    void findTetsFromPoints(const std::vector<::physx::PxVec3>& points,
                            std::vector<::physx::PxU32>& tetIds,
                            std::vector<::physx::PxVec4>& tetBarycentrics);
    void findTetsFromVtxIndices(const std::vector<::physx::PxU32>& vtxIds,
                                std::vector<::physx::PxU32>& tetIds,
                                std::vector<::physx::PxVec4>& tetBarycentrics);
    void convertToPhysxAttachmentTets(std::vector<::physx::PxU32>& tetIds, std::vector<::physx::PxVec4>& tetBarycentrics);

    ::physx::PxDeformableAttachment* addRigidAttachments(::physx::PxActor* actor,
                                                         std::vector<::physx::PxU32> indices,
                                                         std::vector<::physx::PxVec4> barycentrics,
                                                         std::vector<::physx::PxVec4> coords);
    ::physx::PxDeformableElementFilter* addRigidFilters(::physx::PxActor* actor, std::vector<::physx::PxU32> indices);

    ::physx::PxDeformableAttachment* addSoftBodyAttachments(::physx::PxActor* actor1,
                                                            std::vector<::physx::PxU32> indices1,
                                                            std::vector<::physx::PxVec4> barycentrics1,
                                                            std::vector<::physx::PxU32> indices0,
                                                            std::vector<::physx::PxVec4> barycentrics0);
    ::physx::PxDeformableElementFilter* addSoftBodyFilters(::physx::PxActor* actor1,
                                                           std::vector<::physx::PxU32> indices1,
                                                           std::vector<::physx::PxU32> indices0);
};

class InternalDeformableSurfaceDeprecated : public InternalDeformableDeprecated
{
public:
    InternalDeformableSurfaceDeprecated(PhysXScene* ps, pxr::UsdPrim& prim);
    ~InternalDeformableSurfaceDeprecated();

    ::physx::PxDeformableSurface* mDeformableSurface;
    ::physx::PxTriangleMesh* mTriangleMesh;

    ::physx::PxVec4* mPositionInvMassH; // pinned host memory
    ::physx::PxVec4* mVelocityH; // pinned host memory
    ::physx::PxVec4* mRestPositionH; // pinned host memory

    pxr::UsdPrim mPrim;
    pxr::GfMatrix4d mPrimToWorld;

    pxr::GfMatrix4d mInitialPrimToWorld;

    std::vector<uint32_t> mPhysxToUsdVtxMap;
    std::vector<uint32_t> mUsdToPhysxVtxMap;

    unsigned int mNumCollMeshVerticesWelded;
    unsigned int mNumCollMeshVerticesOrig;

    ::physx::PxFEMParameters mFEMParameters;
    int mSolverPositionIterations;

    std::vector<carb::Float3> mCollMeshPositionSaveRestoreBuf;
    std::vector<carb::Float3> mCollMeshVelocitySaveRestoreBuf;
    pxr::VtVec3fArray mExtentSaveRestoreBuf;

    void buildMap(const std::vector<uint32_t>& usdSimTriVtxIndices);
    void buildInverseMap(const std::vector<uint32_t>& usdSimTriVtxIndices);

    void findTrisFromPoints(const std::vector<::physx::PxVec3>& points,
                            std::vector<::physx::PxU32>& triIds,
                            std::vector<::physx::PxVec4>& triBarycentrics);
    void findTrisFromVtxIndices(const std::vector<::physx::PxU32>& vtxIds,
                                std::vector<::physx::PxU32>& triIds,
                                std::vector<::physx::PxVec4>& triBarycentrics);

    ::physx::PxDeformableAttachment* addRigidAttachments(::physx::PxActor* actor,
                                                         std::vector<::physx::PxU32> indices,
                                                         std::vector<::physx::PxVec4> coords);
    ::physx::PxDeformableAttachment* addRigidAttachments(::physx::PxActor* actor,
                                                         std::vector<::physx::PxU32> indices,
                                                         std::vector<::physx::PxVec4> barycentrics,
                                                         std::vector<::physx::PxVec4> coords);
    ::physx::PxDeformableElementFilter* addRigidFilters(::physx::PxActor* actor, std::vector<::physx::PxU32> indices);
};


} // namespace internal
} // namespace physx
} // namespace omni

#ifdef _MSC_VER
#    pragma warning(pop)
#endif
