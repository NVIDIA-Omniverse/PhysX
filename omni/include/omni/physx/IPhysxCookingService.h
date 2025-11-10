// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include <carb/Defines.h>
#include <carb/Types.h>
#include <omni/Span.h>
#include <omni/Function.h>

#include "PhysxCookingParams.h"
#include "MeshKey.h"
namespace omni
{
namespace physx
{
struct PhysxCookingComputeResult;

typedef void* PhysxCookingOperationHandle;
typedef void* PhysxCookingAsyncContext;
struct PhysxCookingAsyncContextParameters
{
    omni::span<const char> contextName;
};

/// Cooking finished results
struct PhysxCookingResult
{
    enum Enum
    {
        eVALID = 0,
        eERROR_CANCELED = 1,
        eERROR_PRIM_INVALIDATION = 2,
        eERROR_COOKING_FAILED = 3,
        eERROR_INVALID_PRIM = 4,
        eERROR_INVALID_STAGE = 5,
        eERROR_INVALID_CONTEXT = 6,
        eERROR_CUDA_CONTEXT_MANAGER = 7,
    };
};

/// Enumerations holding the different types of data in the Cooking Local Cache
struct PhysxCookingDataType
{
    // This mirrors and extends the types defined at CookingParamsType
    enum Enum : uint32_t
    {
        eUNDEFINED = 0,
        eCONVEX_MESH = 1,
        eTRIANGLE_MESH = 2,
        eCONVEX_DECOMPOSITION = 3,
        eSDF_TRIANGLE_MESH = 4,
        eSPHERE_FILL = 5,
        eSOFT_BODY_DEPRECATED = 6,
        eDEFORMABLE_TETRAHEDRAL_MESH_DEPRECATED = 7,
        ePARTICLE_CLOTH_DEPRECATED = 8,
        ePARTICLE_POISSON_SAMPLING = 9,
        eDEFORMABLE_VOLUME_MESH = 10,
        eVOLUME_DEFORMABLE_BODY = 11,
        eSURFACE_DEFORMABLE_BODY = 12
    };
};

//!< A view over an un-triangulated mesh (for example as read from USD)
struct PhysxCookingMeshView
{
    omni::span<const carb::Float3> points;
    omni::span<const int32_t> indices;
    omni::span<const int32_t> faces;
    omni::span<const int32_t> holeIndices;
    bool rightHandedOrientation = true;

    omni::span<const uint16_t> faceMaterials;

    bool isEmpty() const
    {
        return points.empty() || indices.empty() || faces.empty();
    }
};


/// Cooked data buffer descriptor
struct PhysxCookedDataSpan
{
    PhysxCookedDataSpan() : data(nullptr), sizeInBytes(0)
    {
    }
    PhysxCookedDataSpan(void* _data, size_t _sizeInBytes) : data(_data), sizeInBytes(_sizeInBytes)
    {
    }

    void* data;
    size_t sizeInBytes;
};

/// Cooking Request
struct PhysxCookingComputeRequest
{
    // If data is in Fabric, it will be used before falling back to USD (only if forceDisableUSDAccess == false)
    // The caller is responsible for ensuring that mesh data is available for read for entire duration of request* calls
    enum DataInputMode : uint32_t //!< Controls if data comes from prim id or a mesh view
    {
        eINPUT_MODE_FROM_PRIM_ID = 0, //!< Input mesh is inferred from primId, primStageId and primTimeCode (Fabric or
                                      //!< USD)
        eINPUT_MODE_FROM_PRIM_MESH_VIEW = 1 //!< Input mesh is inferred from primMeshView, primMeshMetersPerUnit and
                                            //!< primMeshText
    };
    DataInputMode dataInputMode = eINPUT_MODE_FROM_PRIM_ID;

    // eINPUT_MODE_FROM_PRIM_ID
    uint64_t primStageId = 0; //!< Id of the stage holding the data to cook (used if dataInputMode ==
                              //!< eINPUT_MODE_FROM_PRIM_ID)
    uint64_t primId = 0; //!< Id of the mesh prim to cook (used if dataInputMode == eINPUT_MODE_FROM_PRIM_ID)
    double primTimeCode = 0.0; //!< UsdTimeCode to sample the mesh (used if dataInputMode == eINPUT_MODE_FROM_PRIM_ID)

    // eINPUT_MODE_FROM_PRIM_MESH_VIEW
    PhysxCookingMeshView primMeshView; //!< Input mesh (used if dataInputMode == eINPUT_MODE_FROM_PRIM_MESH_VIEW)
    double primMeshMetersPerUnit = 1.0; //!< Meters per unit (used if dataInputMode == eINPUT_MODE_FROM_PRIM_MESH_VIEW)
    omni::span<const char> primMeshText; //!< Prim Path used in debug messages (used if dataInputMode ==
                                         //!< eINPUT_MODE_FROM_PRIM_MESH_VIEW)

    omni::physx::usdparser::MeshKey meshKey; //!< OPTIONAL but if provided, it will skip hash recomputation for
                                             //!< vertices, triangles etc.
                                             //!< This value is provided from a previous call to IPhysxCooking, in the
                                             //!< response object (PhysxCookingComputeResponse::meshKey).
                                             //!< Be aware that this value may change between different versions of kit
                                             //!< physics or between different cooking backends used (UJITSO or Local)

    PhysxCookingDataType::Enum dataType = PhysxCookingDataType::eUNDEFINED; //!< Data Type of this compute request
    enum Mode : uint32_t
    {
        eMODE_COMPUTE_CRC = 0, //!< Computes only the meshCRC and meshKey fields, skipping retrieving / computing actual
                               //!< cooked data
        eMODE_REQUEST_COOKED_DATA = 1 //!< Same as eMODE_COMPUTE_CRC but also retrieves cooked data from cache or by
                                      //!< cooking the mesh if data is not in cache
    };
    Mode mode = eMODE_REQUEST_COOKED_DATA;

    omni::function<void(const PhysxCookingComputeResult& result)> onFinished; //!< Callback invoked when request on any
                                                                              //!< Mode has been fullfilled. onFinished
                                                                              //!< is always invoked, both when request
                                                                              //!< succeeds or when it fails and both
                                                                              //!< with kComputeAsynchronously set
                                                                              //!< (during pumpAsyncContext) and with
                                                                              //!< kComputeAsynchronously unset.
    struct Triangulation
    {
        bool needsVertices = false; //!< return triangulation mesh vertices in PhysxCookingMeshTriangulationView::points
        bool needsTriangles = false; //!< return triangulation indices in PhysxCookingMeshTriangulationView::triangles
        bool needsTriangleFaceMap = false; //!< return triangulation face mapping in
                                           //!< PhysxCookingMeshTriangulationView::trianglesFaceMap
        bool needsTriangleFaceMaterials = false; //!< return triangulation face materials in
                                                 //!< PhysxCookingMeshTriangulationView::faceMaterials
        bool needsMaxMaterialIndex = false; //!< Fill PhysxCookingComputeResult::triangulationMaxMaterialIndex

        bool isNeeded() const
        {
            return needsVertices || needsTriangles || needsTriangleFaceMap || needsTriangleFaceMaterials ||
                   needsMaxMaterialIndex;
        }
    };

    Triangulation triangulation; //!< Allows requesting specific buffers of the triangulation (triangles, vertices, face
                                 //!< mapping, face materials). Only supported for TriangleMesh approximation.

    struct Options
    {
        enum Flags : uint64_t
        {
            kComputeAsynchronously = 1 << 0, //!< If set cooking happens on a separate thread. If set, onFinished will
                                             //!< be dispatched during pumpAsyncContext called with same async context
                                             //!< used to create the task. Calling pumpAsyncContext is in charge to the
                                             //!< client.
            kComputeGPUCookingData = 1 << 1, //!< If set will generate GPU data
            kExecuteCookingOnGPU = 1 << 2, //!< If set will try to execute cooking on GPU (currently only SDF is
                                           //!< supported)
            kReservedFlag3 = 1 << 3, //!< Reserved, do not use
            kReservedFlag4 = 1 << 4, //!< Reserved, do not use
            kLoadCookedDataFromCache = 1 << 5, //!< If set will try to load cooked data from the cache
            kSaveCookedDataToCache = 1 << 6, //!< If set will save cooked data to cache
            kForceDisableUSDAccess = 1 << 7, //!< If set, the system will only read fabric and never fallback to USD
                                             //!< (useful for thread safety when running on a non-main thread)
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
        uint64_t flags =
            kComputeGPUCookingData | kExecuteCookingOnGPU | kLoadCookedDataFromCache | kSaveCookedDataToCache;
    };
    Options options;
};

//!< A view over a triangulated mesh, as read-only output of triangulation task
struct PhysxCookingMeshTriangulationView
{
    omni::span<const carb::Float3> points;
    omni::span<const uint32_t[3]> triangles;
    omni::span<const uint32_t> trianglesFaceMap;
    omni::span<const uint16_t> faceMaterials;
};

/// Cooking Result
struct PhysxCookingComputeResult
{
    PhysxCookingResult::Enum result = PhysxCookingResult::eERROR_COOKING_FAILED; //!< Return code for success or error
    omni::physx::usdparser::MeshKey meshKey; //!< Hash of mesh vertices, indices etc. excluding orientation
    omni::physx::usdparser::MeshKey cookedDataCRC; //!< meshKey + orientation + collision parameters

    const PhysxCookingComputeRequest* request = nullptr; //!< The request object that was used when requesting cooking

    const PhysxCookedDataSpan* cookedData = nullptr; //!< Pointer to first element of an array of cooked data elements
    uint64_t cookedDataNumElements = 0; //!< The number of elements in the array of cooked data

    PhysxCookingMeshTriangulationView triangulationView; //!< The triangulation data
    uint32_t pad; // Unused bytes for future expansion

    // These are some informative flags
    struct ResultWarning
    {
        enum Flags
        {
            NO_WARNINGS = 0, //!< No warnings
            FAILED_GPU_COMPATIBILITY = 1 << 0, //!< Cooked mesh will not be GPU compatible
            CONVEX_POLYGON_LIMITS_REACHED = 1 << 1, //!< Convex hull generation has reached polygon limits
        };

        ResultWarning& setFlag(Flags flag, bool value)
        {
            flags = value ? flags | flag : flags & ~flag;
            return *this;
        }
        bool hasFlag(Flags flag) const
        {
            return (flags & flag) != 0;
        }
        uint32_t flags = NO_WARNINGS;
    };
    ResultWarning resultWarnings;

    enum ResultSource
    {
        eRESULT_CACHE_MISS, //!< If the cooked data has been recomputed from scratch
        eRESULT_CACHE_HIT_LOCAL, //!< If the cooked data has been read from local cache
        eRESULT_CACHE_HIT_UJITSO, //!< If the cooked data has been read from UJITSO datastore
    };
    ResultSource resultSource = eRESULT_CACHE_MISS; //!< Indicates the source of data returned in this result object
    enum RequestSource
    {
        eREQUEST_SOURCE_USD, //!< If the input data was read from USD
        eREQUEST_SOURCE_FABRIC, //!< If the input data was read from Fabric
        eREQUEST_SOURCE_MESHVIEW, //!< If the input data was read from raw Mesh View Memory
    };
    RequestSource requestSource = eREQUEST_SOURCE_USD; //!< Indicates the source of input mesh
    bool isSynchronousResult = true; //!< Will be true when called before a ICookingComputeService::request* returns.
                                     //!< With kComputeAsynchronously set, isSynchronousResult will be true if request
                                     //!< hits a cache or in case of errors With kComputeAsynchronously unset,
                                     //!< isSynchronousResult will always be true
    uint16_t triangulationMaxMaterialIndex = 0; //!< available only if Triangulation::needsMaxMaterialIndex == true.
                                                //!< This is the highest index referenced in the faceMaterials array
};

struct IPhysxCookingService
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysxCookingService", 1, 0)

    /// Starts asynchronous tasks and invokes completion callbacks (onFinished) for tasks that have finished.
    /// Calling this in a carb.tasking thread allows you to receive those asynchronous cooking completion notifications
    /// in that same thread - therefore binding those notifications to the thread you invoked this into.
    /// NOTE: The thread where you call this MUST be a carb.tasking thread, not a regular std::thread (since we use
    /// carb.mutex fiber-based in the code)
    uint32_t(CARB_ABI* pumpAsyncContext)(PhysxCookingAsyncContext context);

    /// Cancels a task. if second parameter is true, it will also invoke onFinished with eERROR_CANCELED
    bool(CARB_ABI* cancelTask)(PhysxCookingOperationHandle handle, bool invokeCallbackAnyway);

    /// Cancels all tasks belonging to the given context, without invoking their onFinished (that will NOT be called
    /// even by pumpAsyncContext)
    uint32_t(CARB_ABI* cancelAllTasks)(PhysxCookingAsyncContext context);

    /// Waits synchronously for a given task handle to finish its operation.
    /// If timeoutMs is < 0 it will wait indefinitively until operation is finished
    /// Note: This method DOES NOT invoke the onFinished callback (that will be called with pumpAsyncContext)
    bool(CARB_ABI* waitForTaskToFinish)(PhysxCookingOperationHandle handle, int64_t timeoutMs);

    /// Create an async context needed when using pumpAsyncContextTasks
    PhysxCookingAsyncContext(CARB_ABI* createAsyncContext)(PhysxCookingAsyncContextParameters& parameters);

    /// Destroy a context, previously created with createAsyncContext
    void(CARB_ABI* destroyAsyncContext)(PhysxCookingAsyncContext context);

    /// Get cooked data for triangle mesh. Context must not be null if kComputeAsynchronously is set
    PhysxCookingOperationHandle(CARB_ABI* requestTriangleMeshCookedData)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams);

    /// Get cooked data for sdf mesh. Context must not be null if kComputeAsynchronously is set
    /// If kExecuteCookingOnGPU is set, a global CUDA context manager will be used.
    PhysxCookingOperationHandle(CARB_ABI* requestSdfMeshCookedData)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::TriangleMeshCookingParams& triangleMeshCookingParams,
        const omni::physx::SdfMeshCookingParams& sdfMeshCookingParams);

    /// Get cooked data for convex mesh. Context must not be null if kComputeAsynchronously is set
    PhysxCookingOperationHandle(CARB_ABI* requestConvexMeshCookedData)(PhysxCookingAsyncContext context,
                                                                       const PhysxCookingComputeRequest& request,
                                                                       const omni::physx::ConvexMeshCookingParams& desc);

    /// Get cooked data for convex mesh decomposition. Context must not be null if kComputeAsynchronously is set
    PhysxCookingOperationHandle(CARB_ABI* requestConvexMeshDecompositionCookedData)(
        PhysxCookingAsyncContext context,
        const PhysxCookingComputeRequest& request,
        const omni::physx::ConvexDecompositionCookingParams& desc);

    /// Get cooked data for sphere fill decomposition. Context must not be null if kComputeAsynchronously is set
    PhysxCookingOperationHandle(CARB_ABI* requestSphereFillCookedData)(PhysxCookingAsyncContext context,
                                                                       const PhysxCookingComputeRequest& request,
                                                                       const omni::physx::SphereFillCookingParams& desc);
};
} // namespace physx

} // namespace omni
