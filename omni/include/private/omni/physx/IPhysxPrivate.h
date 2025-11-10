// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysx.h>
#include <omni/Span.h>

// PhysX includes, should not be here really
#include <foundation/PxVec4.h>
#include <foundation/PxVec3.h>
#include <foundation/PxMat44.h>
#include <foundation/PxSimpleTypes.h>
namespace physx
{
class PxScene;
class PxCudaContextManager;
} // namespace physx

namespace omni
{
namespace physx
{

/// Instanced data struct, holding information for point instanced objects
///
struct InstancedData
{
    uint64_t instancerPath; //!< SdfPath to the point instancer
    uint32_t instanceIndex; //!< Instance index of the object
};


struct InternalDeformableBodyData
{
    pxr::UsdPrim bodyPrim;
    pxr::UsdPrim simMeshPrim;
    pxr::GfMatrix4f worldToSimMesh;
    omni::span<pxr::UsdPrim> skinMeshPrims;
    omni::span<pxr::GfMatrix4f> worldToSkinMeshTransforms;
    omni::span<carb::Uint2> skinMeshRanges;
    uint32_t numSkinMeshVertices; // all skin mesh vertices
    uint32_t numSimMeshVertices; // physx sim mesh

    ::physx::PxVec4* simMeshPositionInvMassH; // physx sim mesh, pinned host memory

    ::physx::PxVec3* allSkinnedVerticesH;     // deformable skinning mesh data, pinned host memory
    ::physx::PxVec3* allSkinnedVerticesD;     // deformable skinning mesh data, device memory
};

struct InternalSurfaceDeformableBodyData : InternalDeformableBodyData
{
};

struct InternalVolumeDeformableBodyData : InternalDeformableBodyData
{
    pxr::UsdPrim collMeshPrim;
    pxr::GfMatrix4f worldToCollMesh;
    uint32_t numCollMeshVertices; // physx coll mesh!

    ::physx::PxVec4* collMeshPositionInvMassH; // physx coll mesh, pinned host memory
};

/// A private interface for physics extensions that need to be tightly coupled with omni.physx.
///
/// Subject to change without notice.
///
/// This interface should be considered internal to the omni.physx family of extensions and
/// should not be used by external clients.  Clients should rely on public interfaces like IPhysx
/// and IPhysxSimulation instead.
///
struct IPhysxPrivate
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxPrivate", 2, 0)

    /// TODO: move IPhysx::getPhysXPtr and friends here?

    /// TODO: move IPhysx::isReadbackSuppressed here?


    /// Get the PhysX scene for the active simulation.
    ///
    /// NB: this method may be changed or removed once multiple scenes are supported.
    ///
    /// \return The PxScene pointer or null if there is no active simulation.
    ::physx::PxScene*(CARB_ABI* getPhysXScene)();

    /// Get the point instancer information for given ids.
    ///
    /// \param[in] ids      Array of instanced object Ids.
    /// \param[in] numIds   Number of provided ids
    /// \param[out] data    Array of output data, the memory is allocated by the user and it needs to large enough
    ///                     to write information for each id, so size if numIds.
    void(CARB_ABI* getRigidBodyInstancedData)(usdparser::ObjectId* ids, uint32_t numIds, InstancedData* data);

    /// Get the PhysX Cuda context manager for the current PhysX instance.
    ///
    /// \return The PxCudaContextManager pointer.
    ::physx::PxCudaContextManager*(CARB_ABI* getCudaContextManager)();

    /// Get the instanced PhysX pointers
    ///
    /// This retrieves the pointers of all the physics objects created for a point instance proto.
    /// \param path Usd path where the physics objects were created
    /// \param data The output buffer of the instanced pointers
    /// \param dataSize The size of the output buffer
    /// \param type Physics type, note that there can be more than one object per path, so a type is required to return
    /// correct result
    ///
    /// \returns The number of the instanced pointers
    uint32_t(CARB_ABI* getPhysXPtrInstanced)(const pxr::SdfPath& path, void** data, uint32_t dataSize, PhysXType type);

    // Get the internal surface deformable body data for the given object id
    ///
    /// \param[in] deformableId Deformable object ID (see getObjectId() to get the ID from a USD path)
    /// \param[out] data        Output internal surface deformable body data
    void(CARB_ABI* getInternalSurfaceDeformableBodyData)(const usdparser::ObjectId deformableId,
                                                         InternalSurfaceDeformableBodyData& data);

    // Get the internal volume deformable body data for the given object id
    ///
    /// \param[in] deformableId Deformable object ID (see getObjectId() to get the ID from a USD path)
    /// \param[out] data        Output internal volume deformable body data
    void(CARB_ABI* getInternalVolumeDeformableBodyData)(const usdparser::ObjectId deformableId,
                                                        InternalVolumeDeformableBodyData& data);
};

} // namespace physx
} // namespace omni
