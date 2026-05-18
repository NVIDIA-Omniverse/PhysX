// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

// CookingTask defines the base class for all asynchronous physics cooking operations.
// Since the 'cooking' of physics data can take a substantial period of time, we need to
// perform as much of the operation in a background thread (using the carbonite tasking system)
// as possible.  Each 'task' has four phases.
//
// * Initialization : Called from the main thread (prepping any data needed for the cooking operation)
// * Pump : Called from the main thread which will schedule the task when ready, and return the completion state
// * Cooking : The actual cooking operation itself that runs in a background thread. USD is not thread safe, and
// therefore no USD operations can occur in this step
// * Finalization : Called from the main thread and stores the results of the cooking operation and updates USD prims as
// needed.
//


#include <extensions/PxDefaultStreams.h>
#include <carb/Types.h>
#include <vector>
#include <omni/physx/IPhysxCooking.h>
#include <PxPhysicsAPI.h>
#include <omni/Span.h>


// The maximum number of cooking tasks we allow to run simultaneously
#define MAX_ACTIVE_TASK_COUNT 8

// Forward reference various classes and structs used by the API
namespace omni
{
namespace physx
{
struct PhysxCookingMeshView;
struct TriangleMeshCookingParams;
struct SdfMeshCookingParams;
struct ConvexMeshCookingParams;
struct ConvexDecompositionCookingParams;
struct SphereFillCookingParams;
struct DeformableBodyTetMeshCookingParamsDeprecated;
struct SoftBodyMeshCookingParamsDeprecated;
struct ParticleClothMeshCookingParamsDeprecated;
struct ParticlePoissonSamplingCookingParams;
struct DeformableVolumeMeshCookingParams;
struct VolumeDeformableBodyCookingParams;
struct SurfaceDeformableBodyCookingParams;

typedef void* PhysxCookingAsyncContext;
namespace usdparser
{
class MeshKey; // The unique hash for a cooked collision primitive
} // namespace usdparser
struct PhysxCookingMeshTriangulationView;
struct PhysxCookingComputeResult;
struct PhysxCookingComputeRequest;

struct PhysxCookingDeformableBodyTetMeshDataDeprecated;
struct PhysxCookingParticleClothMeshDataDeprecated;
struct PhysxCookingVolumeDeformableBodyData;
struct PhysxCookingSurfaceDeformableBodyData;
struct PhysxCookingParticlePoissonSamplingData;

struct TriangulationMesh
{
    std::vector<carb::Float3> points;
    std::vector<uint32_t> indices;
    std::vector<uint32_t> trianglesToFacesMapping;
    std::vector<uint16_t> faceMaterials;
    uint16_t maxMaterialIndex = 0;
};
} // namespace physx
} // namespace omni

// Forward reference carbonite Float3 struct
namespace carb
{
struct Float3; // Carbonite vec3
}

// Forward reference two physx sdk classes
namespace physx
{
class PxDefaultMemoryOutputStream; // Forward reference the memory output stream class
class PxPhysics;
} // namespace physx

// Forward reference the carbonite tasking interface
namespace carb
{
namespace tasking
{
struct ITasking; // Carbonite tasking interface
}
} // namespace carb
namespace cookedcache
{
class CookedCache;
}
// All of the cookingtask related code resides in a 'cookingtask' namespace
namespace cookingtask
{
class CookingDataAsync;
class CookingTaskImpl; // Forward reference the implementation class that implements the CookingTask methods

// The base class for Cooking tasks, specific cooking implementations will inherit this
// base class.
class CookingTask
{
public:
    /**
     * Declare the default constructor
     */
    CookingTask(omni::physx::PhysxCookingComputeResult& result);

    /**
     * Declares the destructor as a pure virtual function. It must be declared as
     * virtual for the parent class to have it's destructor called first
     */
    virtual ~CookingTask(void);

    /**
     * Start performing the task from another thread. This will not be the
     * main thread and USD operations are not safe to do here.
     * Do work in the background thread and any results that need to be written in
     * the main thread should be processed when 'finalize' is called
     */
    virtual void performTask(void) = 0;

    /**
     * From the main thread, save out any results produced by the background cooking operation
     *
     */
    virtual void finalize(void) = 0;

    /**
     * Called from the main thread once per logical 'frame'.
     * This method will start the background task if it can and returns true if
     * the task has been fully processed and can be released.
     */
    bool pump(carb::tasking::ITasking* tasking);

    /**
     * This method attempts to load the source triangle mesh associated with this meshKey
     * if it exists in the local cache.
     * If it does not exist, it returns false
     * Most of the cooking methods operate on a UsdGeomMesh primitive. However, we cannot access USD from a
     * background thread and, as well, the triangle meshes in a UsdGeomMesh are not stored in the same format
     * we would want to use for geometric processing.
     * As an optimization, once we compute the indexed triangle mesh associated with a UsdGeomMesh we can store
     * that into the local-cache so that it doesn't have to be recomputed again each time in the main thread.
     *
     * @return : Returns true if the triangle mesh was successfully found and loaded from the local cache
     */
    bool loadTriangleMesh(void);

    /**
     * If the triangle mesh couldn't be found in the local mesh cache, then we need to triangulate from
     * the UsdGeomPrim itself.
     *
     * To avoid causing hangs or stalls in the main thread, we try to perform the triangulation process
     * in the background thread. However, since we cannot access USD from a separate thread, we need to
     * prepare the data which we intend to triangulate. Calling this method will stage the geometric data
     * associated with this UsdGeomMesh primitive so that it can be processed by the background thread.
     *
     * @param usdPrim : The UsdGeomMesh that we are triangulating to get the source data we need to cook
     */
    void initTriangulation(const pxr::UsdPrim& usdPrim, uint16_t& maxMaterialIndex);

    /**
     * This method is called from a background thread. It takes the polygon data from the source UsdGeomMesh
     * (read in initTriangulation) and converts it into an indexed triangle mesh.
     */
    void performTriangulation(void);

    /**
     * This method is called from the main thread. It stores the results from performTriangulation to the
     * local cache so that the next time this mesh is encountered we can just load the triangulated data
     * directly from the cache.
     */
    void saveTriangulation(::physx::PxDefaultMemoryOutputStream& stream);

    /**
     * This method is used to make sure that the source mesh meets the criteria of a minimum thickness.
     * If the source mesh is coplanar it will extrude the mesh to meet this 'minThickness' requirement
     * Without this convex hull approximations would be invalid.
     *
     * @param minThickness : The minimum thickness the mesh is allowed to be
     *
     * @return : Returns true if the mesh was adjusted because it was too small or coplanar
     */
    bool checkMeshThickness(float minThickness);

    /**
     * Returns the source vertices and vertex count of the source data we wish to cook
     *
     * @param vertexCount : A reference which will return the number of vertices in the source data
     *
     * @return : Returns a pointer to the vertex data
     */
    const float* getVertices(uint32_t& vertexCount);

    /**
     * Returns the source indices and triangle count of the source data we wish to cook
     *
     * @param triangleCount : A reference which will return the number of triangles in the source data
     *
     * @return : Returns the triangle indices
     */
    const uint32_t* getIndices(uint32_t& triangleCount);

    /**
     * Returns the source material indices and face count
     * When cooking a triangle mesh it is possible to have a different physics material assigned to each
     * triangle.
     *
     * @param faceCount : A reference which will return the number of faces
     *
     * @returns : Returns a pointer to the material indices associated with each polygon face
     */
    const uint16_t* getMaterialIndices(uint32_t& faceCount) const;

    /**
     * Returns something called a 'triangle face map'. This is a mapping from the face indices to the
     * triangle indices
     *
     * @param triangleCount : A reference which returns the number of triangles in the source mesh
     *
     * @return : A pointer to the remapping table
     */
    const uint32_t* getTriangleFaceMap(uint32_t& triangleCount) const;

    /***
     * Attempt to cancel this background cooking task if possible. Results will not be stored
     * Not all tasks will end immediately when 'cancel' is called. This is an outstanding todo item.
     */
    void cancel(bool invokeCallbackAnyway);

    /**
     * Safely wait for the background task to complete before returning. This is typically called from the
     * destructor of a cooking task so that it doesn't try to finalize the results until the background
     * thread is fully completed.
     */
    bool futureWait(int64_t timeoutMs = -1);

    /**
     * Returns true if this task was requested to be canceled, in which case results should be ignored.
     *
     * @return : Returns true if this cooking task has been canceled
     */
    bool isCanceled(void);

    /**
     * Get the unique hash key for this cooking task
     * @param result : The result object
     *
     */
    void getCRC(omni::physx::usdparser::MeshKey& crc) const;

    /**
     * Retrieve the fully qualified path name for the USD prim we are cooking
     *
     * @return : Returns the SdfPath of the primitive we are operating against
     */
    pxr::SdfPath getPrimPath(void) const;

    const std::string& getPrimPathText(void) const;

    bool setPrimPathText(const char* primPathText);

    pxr::UsdStageWeakPtr getStage() const;

    void fireFinishedCallback(omni::physx::PhysxCookingResult::Enum result);


    /**
     * Computes the bounding box for these vertices. When creating a debug visualization for
     * cooked data we include optionally the bounding box for it.
     * This is in support of the solid mesh visualization where we need to know the bounding box of
     * the parent mesh.
     *
     * @param vertexCount : The number of vertices that we need to compute the bounding box for
     * @param vertices : A pointer to the source vertices
     */
    void computeBoundingBox(uint32_t vertexCount, const float* vertices);

    /**
     * Gets the bounding box computed with computeBoundingBox.
     *
     * @param min : The min x,y,z
     * @param max : The max x,y,z
     */
    void getBoundingBox(carb::Float3& min, carb::Float3& max);


    /**
     * Set the finalized state. Indicates that the results for cooking have been
     * fully processed or not. Not to be confused with 'finished' which indicates that the
     * background thread has completed.  Finalized means we have fully written out all of
     * the cooked results and it is safe to delete this cooking task.
     *
     * @param state : The finalized state
     */
    void setFinalized(bool state);

    /**
     * @return : Returns true if the results have been finalized
     */
    bool isFinalized(void) const;

    /**
     * Sets a flag to indicate that the background thread operation is complete. Not to be confused
     * with the 'finalized' flag, which indicates that the results have been saved to completion.
     *
     * @param state : The background thread completion state
     */
    void setFinished(bool state);

    /**
     * @return : Returns true if the background thread has fully completed running.
     */
    bool isFinished(void) const;

    /**
     * Sets a flag to indicate that the background thread operation is succeeded.
     *
     * @param state : The background thread completion state
     */
    void setSucceeded(bool state);

    /**
     * @return : Returns true if the background thread was succeded.
     */
    bool isSucceeded(void) const;

    void setPxCudaAndGPUPointers(::physx::PxCudaContextManager* cudaContextManager, ::physx::PxPhysicsGpu* physicsGPU);

    ::physx::PxCudaContextManager* getPxCudaContextManager();

    ::physx::PxPhysicsGpu* getPxPhysicsGPU();

    /**
     * @return : Returns true if the task has been added to the background thread.
     */
    bool hasStarted(void) const;

    /**
    @ return : Returns true if we were able to successfully initialize the cooking task and it can be run.
    */
    virtual bool isValid(void);

    /**
     * Returns the current pending CookingTask (if there is one)
     * While we have a CookingTask scheduled it is quite possible that a new cooking request
     * can come in for the same primitive but with different collision settings. Typically this
     * happens when someone is rapidly changing properties in the property window using a slider.
     * If a CookingTask is running but a new one is requested, we have to cancel the old task and
     * set the new one as 'pending'. We only ever allow one 'pending' task at a time; as whatever the
     * most recent request coming in is considered valid. Older requests are thrown away.
     *
     * @return : Returns the pointer to the current pending cooking task for this UsdPrim if there is one.
     */
    CookingTask* getPendingTask(void);

    /**
     * Sets the current pending task
     *
     * @param ct : The new pending cooking task that is to be processed once the current task is completed
     */
    void addPendingTask(CookingTask* ct);

    ::physx::PxDefaultMemoryOutputStream& getTriangulationOutputStream();

    std::vector<std::unique_ptr<::physx::PxDefaultMemoryOutputStream>>& getCookedDataOutputStreams()
    {
        return mCookedDataOutputStreams;
    }

    void setMetersPerUnit(double metersPerUnit);

    double getMetersPerUnit() const;

    void setBuildGpuData(bool buildGpuData);

    bool getBuildGpuData() const;

    void setBuildTriangleAdjacencies(bool buildTriangleAdjacencies);

    bool getBuildTriangleAdjacencies() const;

    ::physx::PxCookingParams getDefaultCookingParams() const
    {
        return getCookingParams(getDefaultTolerances(getMetersPerUnit()));
    }

    static ::physx::PxTolerancesScale getDefaultTolerances(double metersPerUnit)
    {
        return ::physx::PxTolerancesScale(float(1.0 / metersPerUnit), float(10.0 / metersPerUnit));
    }

    ::physx::PxCookingParams getCookingParams(const ::physx::PxTolerancesScale& tolerances) const;

    static bool deserializeTriangleMesh(::physx::PxDefaultMemoryInputData idata,
                                        omni::physx::TriangulationMesh& triangulationMesh);

    static void fillTriangulationView(const omni::physx::TriangulationMesh& triangulationMesh,
                                      omni::physx::PhysxCookingMeshTriangulationView& triangulationView);

    omni::physx::PhysxCookingComputeResult& getResultObject();

    omni::physx::TriangulationMesh& getTriangulationMesh();

    bool setupTaskFromRequest(const omni::physx::PhysxCookingComputeRequest& request, bool skipMeshProcessing);

    omni::physx::PhysxCookingAsyncContext getAsyncContext() const
    {
        return mAsyncContext;
    }

    void setAsyncContext(omni::physx::PhysxCookingAsyncContext context)
    {
        mAsyncContext = context;
    }

    void saveCallbackFromRequest(const omni::physx::PhysxCookingComputeRequest& request);

private:
    omni::physx::PhysxCookingAsyncContext mAsyncContext = nullptr;
    std::vector<std::unique_ptr<::physx::PxDefaultMemoryOutputStream>> mCookedDataOutputStreams;

    // A pointer to the internal implementation of these methods
    CookingTaskImpl* mImpl{ nullptr };
};

/***
 * Create a CookingTask to cook a single convex hull
 *
 * @param desc : The descriptor for this convex mesh (how many verts, etc.)
 * @param result : The result object
 * @param cookedCache : Optional cooked cache to load / store triangulation and cooked data
 *
 * @return : Returns a pointer to the CookingTask for this operation
 */
CookingTask* createConvexMeshCookingTask(const omni::physx::ConvexMeshCookingParams& desc,
                                         omni::physx::PhysxCookingComputeResult& result);

/***
 * Create a CookingTask to cook a static triangle mesh
 *
 * @param triangleDesc : The descriptor for this triangle mesh (whether is should have mesh simplification or not, etc.)
 * @param sdfDesc : The descriptor for this sdf mesh ( used if sdfResolution > 0)
 * @param result : The result object
 * @param cookedCache : Optional cooked cache to load / store triangulation and cooked data
 *
 * @return : Returns a pointer to the CookingTask for this operation
 */
CookingTask* createTriangleMeshCookingTask(const omni::physx::TriangleMeshCookingParams& triangleDesc,
                                           const omni::physx::SdfMeshCookingParams& sdfDesc,
                                           omni::physx::PhysxCookingComputeResult& result);

/***
 * Create a CookingTask to cook for a convex decomposition
 *
 * @param desc : The descriptor for this convex decomposition (voxel resolution, number of hulls, etc.)
 * @param result : The result object
 * @param cookedCache : Optional cooked cache to load / store triangulation and cooked data
 *
 * @return : Returns a pointer to the CookingTask for this operation
 */
CookingTask* createConvexDecompositionCookingTask(const omni::physx::ConvexDecompositionCookingParams& desc,
                                                  omni::physx::PhysxCookingComputeResult& result);

/***
 * Create a CookingTask to cook for a soft body simulation
 *
 * @param params : The params needed for soft body mesh cooking
 * @param result : The result object
 * @param cookedCache : Optional cooked cache to load / store triangulation and cooked data
 *
 * @return : Returns a pointer to the CookingTask for this operation
 */
CookingTask* createSoftBodyMeshCookingTaskDeprecated(const omni::physx::SoftBodyMeshCookingParamsDeprecated& params,
                                                     omni::physx::PhysxCookingComputeResult& result);

/***
 * Create a CookingTask to cook for a tetrahedral mesh
 *
 * @param params : Struct holding parameters for creation of deformable body task
 * @param result : The result object
 * @param cookedCache : Optional cooked cache to load / store triangulation and cooked data
 *
 * @return : Returns a pointer to the cooking task interface for this operation
 */
CookingTask* createDeformableBodyTetMeshCookingTaskDeprecated(const omni::physx::DeformableBodyTetMeshCookingParamsDeprecated& params,
                                                              omni::physx::PhysxCookingComputeResult& result);

CookingTask* createDeformableVolumeMeshCookingTask(const omni::physx::DeformableVolumeMeshCookingParams& params,
                                                   omni::physx::PhysxCookingComputeResult& result);

CookingTask* createVolumeDeformableBodyCookingTask(const omni::physx::VolumeDeformableBodyCookingParams& params,
                                                   omni::physx::PhysxCookingComputeResult& result);

CookingTask* createSurfaceDeformableBodyCookingTask(const omni::physx::SurfaceDeformableBodyCookingParams& params,
                                                    omni::physx::PhysxCookingComputeResult& result);

/***
 * read deformable body tet mesh data from binary
 *
 * @param out : descriptor of cooked deformable body tet mesh data data
 * @param cookedData : binary data
 */
void readDeformableBodyTetMeshDataDeprecated(omni::physx::PhysxCookingDeformableBodyTetMeshDataDeprecated& out,
                                             const omni::physx::PhysxCookedDataSpan& cookedData);

/***
 * read particle clothdata from binary
 *
 * @param out : descriptor of cooked particle cloth data
 * @param cookedData : binary data
 */
void readParticleClothMeshDataDeprecated(omni::physx::PhysxCookingParticleClothMeshDataDeprecated& out,
                                         const omni::physx::PhysxCookedDataSpan& cookedData);

/***
 * read particle clothdata from binary
 *
 * @param out : descriptor of sampled data
 * @param cookedData : binary data
 */
void readParticlePoissonSamplingData(omni::physx::PhysxCookingParticlePoissonSamplingData& out,
                                     const omni::physx::PhysxCookedDataSpan& cookedData);

/***
 * Create a CookingTask to cook for a particle cloth
 *
 * @param params : particle cloth mesh parameters
 * @param result : The result object
 * @param cookedCache : Optional cooked cache to load / store triangulation and cooked data
 *
 * @return : Returns a pointer to the cooking task interface for this operation
 */
CookingTask* createParticleClothMeshCookingTaskDeprecated(const omni::physx::ParticleClothMeshCookingParamsDeprecated& params,
                                                          omni::physx::PhysxCookingComputeResult& result);

/***
 * load particle cloth mesh data from binary
 *
 * @param out : parsed binary data
 * @param cookedData : binary data
 */
void loadParticleClothMeshDataDeprecated(omni::physx::PhysxCookingParticleClothMeshDataDeprecated& out,
                                         const omni::physx::PhysxCookedDataSpan& cookedData);

void readVolumeDeformableBodyData(omni::physx::PhysxCookingVolumeDeformableBodyData& out,
                                  const omni::physx::PhysxCookedDataSpan& cookedData);

void readSurfaceDeformableBodyData(omni::physx::PhysxCookingSurfaceDeformableBodyData& out,
                                   const omni::physx::PhysxCookedDataSpan& cookedData);

/***
 * Create a cooking task to sample particle positions from a mesh
 *
 * @param params : the particle poisson sampling params
 * @param result : The result object
 * @param cookedCache : Optional cooked cache to load / store triangulation and cooked data
 *
 * @return : Returns a pointer to the cooking task interface for this operation
 */
CookingTask* createPoissonSamplingCookingTask(const omni::physx::ParticlePoissonSamplingCookingParams& params,
                                              omni::physx::PhysxCookingComputeResult& result);

/***
 * Create a CookingTask to compute a sphere fill approximation
 *
 * @param desc : The descriptor for this convex mesh (how many verts, etc.)
 * @param result : The result object
 * @param cookedCache : Optional cooked cache to load / store triangulation and cooked data
 *
 * @return : Returns a pointer to the CookingTask for this operation
 */
CookingTask* createSphereFillCookingTask(const omni::physx::SphereFillCookingParams& desc,
                                         omni::physx::PhysxCookingComputeResult& result);

} // namespace cookingtask
