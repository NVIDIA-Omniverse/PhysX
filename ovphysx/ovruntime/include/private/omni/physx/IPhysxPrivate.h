// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-18 AC-19 AC-44
 */

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
    // 2026-08-29 (ADR-0018, breaking change): instancerPath is now an
    // `omni::physics::parse::ObjectKey::handle` value, not the legacy asInt(SdfPath)-bit
    // encoding -- mirrors InternalDeformableBodyData's ObjectKey fields above. A consumer that
    // needs a path resolves it on its own side (IPhysx::objectKeyToPath). A handle is only
    // valid against the Source instance that minted it (ADR-0021): one read across a
    // detach/reattach of the owning attach will not resolve to the same object.
    uint64_t instancerPath; //!< ObjectKey::handle of the point instancer (0 if none)
    uint32_t instanceIndex; //!< Instance index of the object
};


struct InternalDeformableBodyData
{
    // Mesh prims are named by ObjectKey, the ADR-0019 opaque object-identity
    // handle: omni.physx stores source-agnostic ObjectKeys internally, and this
    // ABI boundary hands them out directly rather than resolving to a path.
    // Consumers that need a path/prim resolve it on their own side (via
    // IPhysx::objectKeyToPath, or their own AttachedStage if they have one).
    // skinMeshKeys is owned by-value (lives in the consumer's data object)
    // rather than a span into omni.physx storage.
    omni::physics::parse::ObjectKey bodyKey;
    omni::physics::parse::ObjectKey simMeshKey;
    // World-to-mesh-local inverses, double precision (mirrors
    // InternalDeformableBody). ABI note: these were GfMatrix4f; a consumer that
    // needs a float Gf matrix must convert explicitly now.
    ::physx::PxMat44d worldToSimMesh;
    std::vector<omni::physics::parse::ObjectKey> skinMeshKeys;
    omni::span<::physx::PxMat44d> worldToSkinMeshTransforms;
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
    omni::physics::parse::ObjectKey collMeshKey;
    ::physx::PxMat44d worldToCollMesh;
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
    /// \param key ObjectKey where the physics objects were created
    /// \param data The output buffer of the instanced pointers
    /// \param dataSize The size of the output buffer
    /// \param type Physics type, note that there can be more than one object per key, so a type is required to return
    /// correct result
    ///
    /// \returns The number of the instanced pointers
    uint32_t(CARB_ABI* getPhysXPtrInstanced)(omni::physics::parse::ObjectKey key, void** data, uint32_t dataSize, PhysXType type);

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
