// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#ifdef _MSC_VER
#    pragma warning(push)
#    ifndef NOMINMAX
#        define NOMINMAX // Make sure nobody #defines min or max
#    endif
#endif

#ifdef __linux__
#    define __forceinline __attribute__((always_inline))
#endif

#include <PxPhysicsAPI.h>

#include "Internal.h"
#include <PhysXDefines.h>
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

    // Source-agnostic handles for the body / sim-mesh / skin-mesh prims; no
    // stored USD handles in runtime state.
    // The IPhysxPrivate ABI exposes these as SdfPaths (producer resolves
    // pathFor).
    omni::physics::parse::ObjectKey mBodyKey;
    omni::physics::parse::ObjectKey mSimMeshKey;
    // World-to-mesh-local inverses. Double precision: the source world
    // transforms these are inverted from are double, so nothing is narrowed on
    // the way in any more.
    ::physx::PxMat44d mWorldToSimMesh;
    std::vector<omni::physics::parse::ObjectKey> mSkinMeshKeys;
    std::vector<::physx::PxMat44d> mWorldToSkinMeshTransforms;
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

    omni::physics::parse::ObjectKey mCollMeshKey;
    ::physx::PxMat44d mWorldToCollMesh;
    std::vector<carb::Uint3> mCollMeshSurfaceTriangles;
    std::vector<uint32_t> mCollMeshSurfaceTriToTetMap;

    uint32_t mNumCollMeshVertices; // physx coll mesh!

    ::physx::PxVec4* mCollMeshPositionInvMassH; // physx coll mesh, pinned host memory

    std::vector<carb::Float3> mCollMeshPointsSaveRestoreBuf;
    std::vector<carb::Float3> mCollMeshExtentSaveRestoreBuf;

    ::physx::PxDeformableVolume* mDeformableVolume;
    ::physx::PxDeformableVolumeMesh* mDeformableVolumeMesh;

    // device volume deformable skinning mesh data
    ::physx::PxU32* mSimMeshTetIndicesD;
    ::physx::PxTetrahedronMeshEmbeddingInfo* mSkinningEmbeddingInfoD;

    // Runtime-owned per-sim-node kinematic-target buffer (one PxVec4 per sim-mesh vertex: xyz target +
    // w active-flag). PhysX's setKinematicTargetBufferD ADOPTS this pointer and reads it every step, so
    // it must outlive any single write -- hence the runtime owns it here, for the body's lifetime, rather
    // than an ovstage write session (whose column is freed at release). Lazily allocated on the first
    // kinematic-target write (undriven bodies never pay); freed in the dtor. Null when the body is not
    // driven. See OvxPhysicsWrite (populate + setKinematicTargetBufferD) and OvxPhysicsRead (gather).
    ::physx::PxVec4* mSimMeshKinematicTargetsD;
    // True once this body has been kinematically driven (setKinematicTargetBufferD handed a non-null buffer +
    // ePARTIALLY_KINEMATIC raised). Raised on the first driving write and NEVER cleared: driving is monotonic
    // (a node goes free->driven but never back), so a body, once driven, stays driven for its life. The read
    // omits a body that has never been driven (REQ-READ-ATTRS-001 AC-9).
    bool mKinematicTargetActive;
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

    std::vector<carb::Float3> mSimMeshExtentSaveRestoreBuf;

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
