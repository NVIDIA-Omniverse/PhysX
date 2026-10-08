// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-15 AC-42 AC-44
 */

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <omni/physics/AttachHandle.h>
#include <omni/physics/parse/Handles.h> // ObjectKey

#include "PhysxConvexMesh.h"
#include "PhysxCookingParams.h"
#include "MeshKey.h"

#include "IPhysxCookingService.h"

namespace omni
{

namespace physx
{

/// Cooking finished callback
/// \param[in] attachHandle Attach the cooking request was issued for (matches @ref
/// IPhysxSimulation::getAttachHandle()); kNoAttach when the request was not tied to an attach
/// \param[in] primKey Handle of the cooked mesh object (@ref
/// omni::physics::parse::ObjectKey::handle), not a USD path encoding (ADR-0019)
/// \param[in] userData User data passed to IPhysxCookingCallback struct
typedef void (*PhysxCookingFinishedCallback)(AttachHandle attachHandle,
                                             uint64_t primKey,
                                             PhysxCookingResult::Enum result,
                                             void* userData);

/// Cooking result callback
/// \param[in] key Cooked data hash key
/// \param[in] dataArray Array of PhysxCookedDataSpan, each representing a cooking result buffer
/// \param[in] arraySize Number of PhysxCookedDataSpan elements in dataArray
/// \param[in] userData User data passed to IPhysxCookingCallback struct
/// Note: the data in dataArray is only valid during the call to this callback.  The described buffers must be
/// copied if they are to be used outside of this function.
typedef void (*PhysxCookingResultCallback)(usdparser::MeshKey key,
                                           const PhysxCookedDataSpan* dataArray,
                                           size_t arraySize,
                                           void* userData);

/// Cooking callback structure holding function pointers for callbacks
struct IPhysxCookingCallback
{
    void* userData = { nullptr };
    PhysxCookingFinishedCallback cookingFinishedCallback = { nullptr }; // Called when cooking is finished
    PhysxCookingResultCallback cookingResultCallback = { nullptr }; // Called when cooked data is available
};

/// Result of a call to IPhysxCooking::requestConvexCollisionRepresentation returned in the onResult callback
struct PhysxCollisionRepresentationResult
{
    enum Enum
    {
        eRESULT_VALID = 0, //!< The result is valid
        eRESULT_ERROR_NOT_READY = 1, //!< The system was not ready to provide a result
        eRESULT_ERROR_INVALID_PARSING = 2, //!< Parsing the input prim collision API failed
        eRESULT_ERROR_COOKING_FAILED = 3, //!< Computation of physics approximation for the prim failed
        eRESULT_ERROR_UNSUPPORTED_APPROXIMATION = 4, //!< The requested approximation is not supported on the given prim
        eRESULT_ERROR_INVALID_RESULT = 5, //!< An error occurred when retrieving result after cooking
    };
};

/// Resulting convexes returned in the onResult callback passed to IPhysxCooking::requestConvexCollisionRepresentation
struct PhysxCollisionRepresentationConvexResult
{
    using CallbackType = omni::function<void(PhysxCollisionRepresentationResult::Enum result, //!< Return code
                                             const PhysxCollisionRepresentationConvexResult& convexResult //!< Result
                                             )>;

    omni::span<const ConvexMeshData> convexes; //!< Span of convex meshes
};

/// Request for IPhysxCooking::requestConvexCollisionRepresentation indicating stage and prim to process
struct PhysxCollisionRepresentationRequest
{
    struct Options
    {
        enum Flags : uint64_t
        {
            //!< If true the operation will be run on background thread and onResult called later (during cooking pump)
            kComputeAsynchronously = 1 << 0,
        };

        Options& setFlag(Flags flag, bool value)
        {
            flags = value ? flags | flag : flags & ~flag;
            return *this;
        }
        bool hasFlag(Flags flag) const
        {
            return (flags & flag) != 0;
        }
        uint64_t flags = kComputeAsynchronously;
    };
    Options options; //!< Options for the request (async / no-async)


    // stageId and attachHandle are two different facts, not a redundancy (ADR-0016 Decision 8).
    // This request is genuinely mixed: the attach lookup names an attach, while the fallback path
    // builds a throwaway AttachedStage straight from the USD stage cache and the stage id is also
    // forwarded to the cooking request as PhysxCookingComputeRequest::primStageId.
    uint64_t stageId = 0; //!< USD stage to read the prim's mesh data from, and the stage-cache
                          //!< fallback when the attach lookup misses
    AttachHandle attachHandle = kNoAttach; //!< Attach to resolve the collision representation
                                           //!< against, from @ref IPhysxSimulation::getAttachHandle()
                                           //!< or kActiveAttach; kNoAttach falls back to stageId
    // 2026-08-29 (ADR-0018, breaking change): when attachHandle/stageId resolves to a live
    // attach, this is now an `omni::physics::parse::ObjectKey::handle` value (e.g. from @ref
    // IPhysx::resolveObjectKey()), NOT the legacy asInt(SdfPath)-bit encoding -- a handle is only
    // valid against the Source instance that minted it (ADR-0021), so caching one across a
    // detach/reattach and resubscribing afterwards fails closed rather than erroring. The
    // stageId-only fallback (no live attach: a stage nobody has attached, resolved straight from
    // the USD stage cache) is a deliberate exception and keeps the OLD legacy asInt(SdfPath)-bit
    // encoding -- there is no live Source such a caller's handle could ever have been minted
    // against.
    uint64_t collisionPrimId = 0; //!< Prim containing the Collision API: an ObjectKey::handle
                                  //!< when an attach is live, else a legacy asInt(SdfPath)-bit id
                                  //!< (see the comment above)
};

/// Task object returned by IPhysxCooking::requestConvexCollisionRepresentation that can potentially be used to
/// cancel the task by calling IPhysxCooking::cancelCollisionRepresentationTask
struct PhysxCollisionRepresentationTask
{
    void* handle = nullptr;
};

/// omni.physx cooking interface
struct IPhysxCooking
{
    // Creates a convex mesh approximation of a given mesh. The mesh is not scaled by the USD mesh scale.
    //
    //\param[in] key Mesh's ObjectKey
    //\param[in] vertexLimit Convex mesh vertex limit in a range <4, 255>
    //\param[out] meshData Convex mesh data output
    //\return True if cooking succeeded
    bool(CARB_ABI* createConvexMesh)(omni::physics::parse::ObjectKey key, uint32_t vertexLimit, ConvexMeshData& meshData);

    /// Empty the local mesh cache contents, that will force any mesh to be cooked again when requested 
    void(CARB_ABI* releaseLocalMeshCache)();

    // Create a conforming tetrahedral mesh from a closed source triangle mesh.
    //
    // The conforming tetrahedral mesh is defined as a tetrahedral mesh whose surface triangles align with
    // the closed source triangle mesh and whose internal vertices lie on the inside of the closed source triangle mesh.
    //
    //\param[out] dstTetPoints Output tetrahedral mesh vertices
    //\param[out] dstTetPointsSize Output number of tetrahedral mesh vertices
    //\param[out] dstTetIndices Output tetrahedral mesh vertex indices
    //\param[out] dstTetIndicesSize Output number of tetrahedral mesh vertex indices
    //\param[in] srcTriPoints Input triangle mesh vertices
    //\param[in] srcTripointsSize Input number of triangle mesh vertices
    //\param[in] srcTriIndices Input triangle mesh vertex indices
    //\param[in] srcTriIndicesSize Input number of triangle mesh vertex indices
    //\param[in] allocateBytes Input function to allocate memory for output
    //\return Whether tetrahedral mesh was created successfully
    bool(CARB_ABI* computeConformingTetrahedralMesh)(carb::Float3*& dstTetPoints,
                                                     uint32_t& dstTetPointsSize,
                                                     uint32_t*& dstTetIndices,
                                                     uint32_t& dstTetIndicesSize,
                                                     const carb::Float3* srcTriPoints,
                                                     const uint32_t srcTriPointsSize,
                                                     const uint32_t* srcTriIndices,
                                                     const uint32_t srcTriIndicesSize,
                                                     void* (*allocateBytes)(size_t));

    // Create a voxel tetrahedral mesh from a source tetrahedral mesh.
    //
    // The voxel tetrahedral mesh is made by voxelizing the source tetrahedra on a regular grid. The
    // resulting voxel tetrahedral mesh embeds all tetrahedra of the source mesh.
    // The provided voxel resolution may be lowered automatically in order to match a lower resolution detected in
    // the source mesh. This may help to avoid softbody convergence issues with high-resolution tetrahedra
    // embedding low resolution collision meshes.
    //
    //\param[out] dstTetPoints Output tetrahedral mesh vertices
    //\param[out] dstTetPointsSize Output number of tetrahedral mesh vertices
    //\param[out] dstTetIndices Output tetrahedral mesh vertex indices
    //\param[out] dstTetIndicesSize Output number of tetrahedral mesh vertex indices
    //\param[out] dstEmbedding Output table providing the index of the tetrahedron in the simulation mesh that contains
    // an input vertex \param[out] dstEmbeddingSize Output numbers of entries in the embedding table \param[in]
    // srcTetPoints Input tetrahedral mesh vertices \param[in] srcTetPointsSize Input number of tetrahedral mesh
    // vertices \param[in] srcTetIndices Input tetrahedral mesh vertex indices \param[in] srcTetIndicesSize Input number
    // of tetrahedral mesh vertex indices \param[in] srcScale Scale of source mesh used to determine resolution of the
    // resulting voxel mesh \param[in] voxelResolution Number of voxels along longest dimension of axis aligned bounding
    // box of source mesh \param[in] allocateBytes Input function to allocate memory for output \return Whether
    // tetrahedral mesh was created successfully
    bool(CARB_ABI* computeVoxelTetrahedralMesh)(carb::Float3*& dstTetPoints,
                                                uint32_t& dstTetPointsSize,
                                                uint32_t*& dstTetIndices,
                                                uint32_t& dstTetIndicesSize,
                                                int32_t*& dstEmbedding,
                                                uint32_t& dstEmbeddingSize,
                                                const carb::Float3* srcTetPoints,
                                                const uint32_t srcTetPointsSize,
                                                const uint32_t* srcTetIndices,
                                                const uint32_t srcTetIndicesSize,
                                                const carb::Float3& srcScale,
                                                const int voxelResolution,
                                                void* (*allocateBytes)(size_t));

    /// Precook mesh with a given cooking parameters
    //\param[in] attachHandle Attach holding the mesh, from IPhysxSimulation::getAttachHandle(), or
    //           kActiveAttach for the lone active attach
    //\param[in] meshKey ObjectKey of the UsdGeomMesh prim
    //\param[in] cookingParams cooking parameters
    //\param[in] cb Optional cooking/result callback, if nullptr is passed then the call is a blocking call
    bool(CARB_ABI* precookMesh)(AttachHandle attachHandle,
                                omni::physics::parse::ObjectKey meshKey,
                                const CookingParams& cookingParams,
                                IPhysxCookingCallback* cb);

    /// Cancel a collision representation request previously issued
    //\ param[in] task                  The non-null task returned by requestConvexCollisionRepresentation to cancel
    //\ param[in] invokeCallbackAnyway  If the onResult callback should be invoked anyway even if the task is cancelled
    void(CARB_ABI* cancelCollisionRepresentationTask)(PhysxCollisionRepresentationTask task, bool invokeCallbackAnyway);

    /// Request a convex collision representation for a given prim, both during and outside of simulation.
    /// The collision representation is a vector of convexes and works on prims with CollisionAPI marked with
    /// Convex Hull or Convex Decomposition approximation.
    /// Any other approximation will return an error result (in the callback)
    //\param[in] request    The request object with the stage and prim path
    //\param[in] onResult   The callback to be called with collision representation data
    //\returns              A null task handle if:
    //                          - The request has been fullfilled immediately (onResult has been called immediately)
    //                          - An error has occurred
    //                      A non-null task handle if:
    //                          - If the request has set kComputeAsynchronously and it cannot be completed immediately
    //                      A non-null task handle can be used to cancel the task with cancelCollisionRepresentationTask
    PhysxCollisionRepresentationTask(CARB_ABI* requestConvexCollisionRepresentation)(
        const PhysxCollisionRepresentationRequest& request,
        PhysxCollisionRepresentationConvexResult::CallbackType onResult);

    // Cooks deformable body with PhysxSchemaPhysxAutoDeformableBodyAPI for given object
    //\param[in] deformableBodyKey ObjectKey of the primitive with UsdPhysicsDeformableBodyAPI
    bool(CARB_ABI* cookAutoDeformableBody)(omni::physics::parse::ObjectKey deformableBodyKey);

};


} // namespace physx
} // namespace omni
