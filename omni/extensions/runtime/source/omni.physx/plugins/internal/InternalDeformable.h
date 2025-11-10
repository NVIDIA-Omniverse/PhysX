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
#include <internal/InternalXformOpResetStorage.h>
#include "PxDeformableSkinning.h"


namespace omni
{
namespace physx
{
class PhysXScene;

namespace internal
{

class InternalDeformableMaterial : public Allocateable
{
public:
    InternalDeformableMaterial(float density) : mDensity(density)
    {
    }

    ~InternalDeformableMaterial() = default;

    void addDeformableId(usdparser::ObjectId id)
    {
        mDeformableIds.push_back(id);
    }

    void removeDeformableId(usdparser::ObjectId id)
    {
        for (size_t i = mDeformableIds.size(); i--;)
        {
            if (mDeformableIds[i] == id)
            {
                mDeformableIds[i] = mDeformableIds.back();
                mDeformableIds.pop_back();
                break;
            }
        }
    }

    float mDensity;
    std::vector<usdparser::ObjectId> mDeformableIds;
};

class InternalDeformableBody : public Allocateable
{
public:
    InternalDeformableBody();
    virtual ~InternalDeformableBody();

    usdparser::ObjectId mMaterialId;
    bool mIsKinematic;
    float mBodyMass;

    pxr::UsdPrim mBodyPrim;
    pxr::UsdPrim mSimMeshPrim;
    pxr::GfMatrix4f mWorldToSimMesh;
    std::vector<pxr::UsdPrim> mSkinMeshPrims;
    std::vector<pxr::GfMatrix4f> mWorldToSkinMeshTransforms;
    std::vector<carb::Uint2> mSkinMeshRanges;
    uint32_t mNumSkinMeshVertices; // all skin mesh vertices
    uint32_t mNumSimMeshVertices; // physx sim mesh!

    ::physx::PxVec4* mSimMeshPositionInvMassH; // physx sim mesh, pinned host memory
    ::physx::PxVec4* mSimMeshVelocityH; // physx sim mesh, pinned host memory

    std::vector<carb::Float3> mSimMeshPointsSaveRestoreBuf;
    std::vector<carb::Float3> mSimMeshVelocitiesSaveRestoreBuf;
    std::vector<carb::Float3> mAllSkinMeshPointsSaveRestoreBuf;

    PhysXScene* mPhysXScene;

    // deformable skinning mesh data (pinned host memory and device memory)
    ::physx::PxVec3* mAllSkinnedVerticesH;
    ::physx::PxVec3* mAllSkinnedVerticesD;
};

class InternalVolumeDeformableBody : public InternalDeformableBody
{
public:
    InternalVolumeDeformableBody();
    virtual ~InternalVolumeDeformableBody();

    bool findTetsFromVtxIndices(const std::vector<::physx::PxU32>& vtxIds,
                                std::vector<::physx::PxU32>& tetIds,
                                std::vector<::physx::PxVec4>& tetBarycentrics,
                                bool forSimMesh);
    bool findTetsFromPoints(const std::vector<::physx::PxVec3>& points,
                            std::vector<::physx::PxU32>& tetIds,
                            std::vector<::physx::PxVec4>& tetBarycentrics,
                            bool forSimMesh);
    void convertToPhysxAttachmentTets(std::vector<::physx::PxU32>& tetIds, std::vector<::physx::PxVec4>& tetBarycentrics);

    ::physx::PxDeformableAttachment* addRigidAttachments(::physx::PxActor* actor,
                                                         std::vector<::physx::PxU32> indices,
                                                         std::vector<::physx::PxVec4> barycentrics,
                                                         std::vector<::physx::PxVec4> coords);
    ::physx::PxDeformableElementFilter* addRigidFilters(::physx::PxActor* actor, std::vector<::physx::PxU32> indices);

    pxr::UsdPrim mCollMeshPrim;
    pxr::GfMatrix4f mWorldToCollMesh;
    std::vector<carb::Uint3> mCollMeshSurfaceTriangles;
    std::vector<uint32_t> mCollMeshSurfaceTriToTetMap;

    uint32_t mNumCollMeshVertices; // physx coll mesh!

    ::physx::PxVec4* mCollMeshPositionInvMassH; // physx coll mesh, pinned host memory

    std::vector<carb::Float3> mCollMeshPointsSaveRestoreBuf;
    pxr::VtVec3fArray mCollMeshExtentSaveRestoreBuf;

    ::physx::PxDeformableVolume* mDeformableVolume;
    ::physx::PxDeformableVolumeMesh* mDeformableVolumeMesh;

    // device volume deformable skinning mesh data
    ::physx::PxU32* mSimMeshTetIndicesD;
    ::physx::PxTetrahedronMeshEmbeddingInfo* mSkinningEmbeddingInfoD;
};

class InternalSurfaceDeformableBody : public InternalDeformableBody
{
public:
    InternalSurfaceDeformableBody();
    virtual ~InternalSurfaceDeformableBody();

    bool findTrisFromVtxIndices(const std::vector<::physx::PxU32>& vtxIds,
                                std::vector<::physx::PxU32>& triIds,
                                std::vector<::physx::PxVec4>& triBarycentrics);
    bool findTrisFromPoints(const std::vector<::physx::PxVec3>& points,
                            std::vector<::physx::PxU32>& triIds,
                            std::vector<::physx::PxVec4>& triBarycentrics);

    ::physx::PxDeformableAttachment* addRigidAttachments(::physx::PxActor* actor,
                                                         std::vector<::physx::PxU32> indices,
                                                         std::vector<::physx::PxVec4> barycentrics,
                                                         std::vector<::physx::PxVec4> coords);
    ::physx::PxDeformableElementFilter* addRigidFilters(::physx::PxActor* actor, std::vector<::physx::PxU32> indices);

    std::vector<uint32_t> mSimToPhysxTriMap;

    pxr::VtVec3fArray mSimMeshExtentSaveRestoreBuf;

    ::physx::PxDeformableSurface* mDeformableSurface;
    ::physx::PxTriangleMesh* mTriangleMesh;

    // device surface deformable skinning mesh data
    ::physx::PxU32* mSimMeshTriIndicesD;
    ::physx::PxTriangleMeshEmbeddingInfo* mSkinningEmbeddingInfoD;
    ::physx::PxVec3* mNormalVectorsD;
};


} // namespace internal
} // namespace physx
} // namespace omni

#ifdef _MSC_VER
#    pragma warning(pop)
#endif
