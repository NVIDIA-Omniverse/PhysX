// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>
#include <carb/tasking/ITasking.h>

#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContextManager.h> // acquireReference()/release() on the owned manager
#include <common/foundation/Allocator.h>
#include <common/utilities/MemoryMacros.h>

#include "CookingTask.h"

#include "../utility/TriangulateUsdMeshPrim.h"
#include "../utility/MeshThickness.h"
#include "CookingComputeService.h"

namespace cookingtask
{


/**
* A helper method to initialize min-max bounding box values to known initial condition
*
* @param p : Initial position
* @param bmin : The destination minimum value
* @param bmax : The destination maximum value
*/
static void fm_initMinMax(const float* p, float* bmin, float* bmax)
{
    bmax[0] = bmin[0] = p[0];
    bmax[1] = bmin[1] = p[1];
    bmax[2] = bmin[2] = p[2];
}

/**
* Update the min/max bounding box values relative to this position
*
* @param p : Position to include in the min/max bounding box range
* @param bmin : The destination minimum value
* @param bmax : The destination maximum value
*/
static void fm_minmax(const float* p, float* bmin, float* bmax) 
{
    if (p[0] < bmin[0])
        bmin[0] = p[0];
    if (p[1] < bmin[1])
        bmin[1] = p[1];
    if (p[2] < bmin[2])
        bmin[2] = p[2];

    if (p[0] > bmax[0])
        bmax[0] = p[0];
    if (p[1] > bmax[1])
        bmax[1] = p[1];
    if (p[2] > bmax[2])
        bmax[2] = p[2];
}

static void computeBoundingBoxValues(uint32_t vertexCount, const float *vertices, carb::Float3 &bmin, carb::Float3 &bmax)
{
    if (vertices)
    {
        fm_initMinMax(vertices, &bmin.x, &bmax.x);
        for (uint32_t i = 1; i < vertexCount; i++)
        {
            const float *v = &vertices[i * 3];
            fm_minmax(v, &bmin.x, &bmax.x);
        }
    }
}

// The number of tasks currently running
static std::atomic<uint32_t> g_activeTaskCount{0};

// This is the base class for a single logical 'cooking task'. Each time
// we need to took a convex mesh, triangle mesh, or convex decomposition, we
// create an instance of the corresponding cooking task which will do as
// much of the work as possible in a background thread using the carbonite
// tasking system
class CookingTaskImpl
{
public:
    /**
    * This is the implementation class for a CookingTask. The 'parent' pointer is the pure-virtual interface class associated with this implementation.
    *
    * @param parent : The parent interface class associated with this implementation
    */
    CookingTaskImpl(CookingTask *parent, omni::physx::PhysxCookingComputeResult& result) : mParent(parent)
    {
        
        mRequestObject = *result.request;
        mResultObject = result;
        mResultObject.request = &mRequestObject;
        mPrimPathText.insert(
            mPrimPathText.begin(), result.request->primMeshText.data(), result.request->primMeshText.data() + result.request->primMeshText.size_bytes());
        m_taskKey = computeCookingTaskKey(result.request->primId, mPrimPathText);
    }

    ~CookingTaskImpl(void)
    {
        // If a background task was started, performTaskInternal() -> mParent->performTask() may still be
        // running on a carb::tasking worker thread against this object's own members (m_pending,
        // m_triangulate, mResultObject, etc.). m_future's destructor only releases a refcounted handle to
        // the shared task state (safe on its own -- the worker keeps the task alive independently) but does
        // not wait for it, so nothing below is safe to free until we know the worker is actually done.
        // completeOpenTask() already guards the synchronous-wait path this way (task.futureWait(-1) before
        // touching a started task); this mirrors it so every teardown path -- destroyAsyncContext(),
        // finalizeAllTasks()/~CookingComputeService(), and this destructor's other callers -- gets the same
        // guarantee instead of racing a delete against the worker mid-task.
        //
        futureWait(-1);

        delete m_pending; // if there was a pending task we nuke it too
        SAFE_RELEASE(m_triangulate); // Release the mesh triangulation interface if it was created
        releaseTriangleMeshData();  // Release the memory for any triangle mesh data that was loaded from the cache
        // Drop the CUDA context manager reference taken in setPxCudaAndGPUPointers(). This must
        // come after the futureWait() above: the worker thread dereferences the manager while it
        // cooks, so releasing earlier could destroy it out from under a task still running.
        SAFE_RELEASE(mPxCudaContextManager);
        // If we started a task, then decrement the global task counter
        if (m_taskStarted)
        {
            g_activeTaskCount--;
        }
    }

    /**
    * Returns true if we have found valid mesh data in the source mesh view to operate on.
    * If no mesh data was resolved, then there is nothing to actually cook and we return false.
    *
    * @return : Returns true if we have source mesh data to operate on.
    */
    bool isValid(void)
    {
        bool ret = false;

        uint32_t vertexCount = 0;
        if (getVertices(vertexCount)) // If we have source vertex data, that is enough to cook the data
        {
            ret = true;
        }
        return ret;
    }

    /**
    * Returns true if the background task is complete and it is safe to remove the cooking task
    *
    * return : Returns true if this task has completed
    */
    bool pump(carb::tasking::ITasking* tasking)
    {
        // If we have not started the task yet..and this operation was marked as
        // canceled, then just kill it
        if (!m_taskStarted && m_cancel)
        {
            return true;
        }
        // Only allow 'MAX_ACTIVE_TASK_COUNT' tasks to run at once (arbitrary gate to
        // avoid thousands of concurrent tasks).
        if (!m_taskStarted && g_activeTaskCount < MAX_ACTIVE_TASK_COUNT)
        {
            m_taskStarted = true;
            g_activeTaskCount++;
            // Set up the background task to actually generate the convex mesh
            m_future = tasking->addTask(carb::tasking::Priority::eHigh, {}, [this] { performTaskInternal(); });
        }
        return m_finished;
    }

    /**
    * This is a little convoluted due to how the implementation and the interface classes
    * have been separated. When it becomes time to perform the task, it actually occurs on
    * the pure virtual method exposed by the 'parent' (interface) class associated with this
    * implementation class.
    */
    void performTaskInternal(void)
    {
        if ( mParent )
        {
            // Perform the background cooking task relative to our logical parent interface class
            mParent->performTask();
        }
    }


    /**
    * Writes the triangulated mesh results to the cache so we don't have to
    * re-triangulate this geometry (identified by the 128-bit MeshKey) next time.
    *
    * The source mesh view can be huge and is only valid for the duration of the request, so the
    * source data is copied on the main thread but the actual triangulation runs in a background
    * thread to avoid blocking. The final cache write happens on the main thread
    * because writing from the background thread caused thread-safety issues.
    */
    void saveTriangulation(::physx::PxDefaultMemoryOutputStream& stream)
    {
        CARB_PROFILE_ZONE(0, "CookingTask::saveTriangulation");
        // Write the triangle mesh data to a memory stream
        // and then save is using the cooked cache interface
        // The triangulate is identified by the unique MeshKey not
        // the CRC. The CRC represents the key both for the source mesh data as
        // well as for the collision / cooking parameters.
        if (m_triangulate)
        {
            uint32_t version = omni::physx::PhysxCookingDataVersion_MeshTriangulation;
            stream.write(&version, sizeof(version));
            uint32_t vertexCount;
            const float* vertices = m_triangulate->getVertices(vertexCount);

            stream.write(&vertexCount, sizeof(vertexCount));
            if (vertexCount)
            {
                stream.write(vertices, sizeof(float) * 3 * vertexCount);
            }

            uint32_t triangleCount;
            const uint32_t* indices = m_triangulate->getIndices(triangleCount);
            stream.write(&triangleCount, sizeof(triangleCount));
            if (triangleCount)
            {
                stream.write(indices, sizeof(uint32_t) * 3 * triangleCount);
            }
            const uint32_t* triangleFaceMap = m_triangulate->getTriangleFaceMap(triangleCount);
            stream.write(&triangleCount, sizeof(triangleCount));
            if (triangleCount)
            {
                stream.write(triangleFaceMap, sizeof(uint32_t) * triangleCount);
            }
            uint32_t faceMaterialsCount;
            const uint16_t* faceMaterials = m_triangulate->getFaceMaterials(faceMaterialsCount);
            if (faceMaterials)
            {
                stream.write(&faceMaterialsCount, sizeof(faceMaterialsCount));
                stream.write(faceMaterials, sizeof(uint16_t) * faceMaterialsCount);
            }
            else
            {
                // Note: It happens sometimes that faceMaterialsCount > 0 but faceMaterials == nullptr.
                faceMaterialsCount = 0;
                stream.write(&faceMaterialsCount, sizeof(faceMaterialsCount));
            }

            uint16_t maxMaterialIndex = mResultObject.triangulationMaxMaterialIndex;
            stream.write(&maxMaterialIndex, sizeof(uint16_t));
           
            ::physx::PxDefaultMemoryInputData indata(stream.getData(), stream.getSize());
            auto meshKeyWithOrientation = mResultObject.meshKey;
            meshKeyWithOrientation.setRightHandedOrientation(mResultObject.request->primMeshView.rightHandedOrientation);
            auto meshTriangulationCRC = meshKeyWithOrientation;
            meshTriangulationCRC.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_MeshTriangulation);

            indata.seek(0);
            CookingTask::deserializeTriangleMesh(indata, mTriangulationMesh);
        }
    }

    /**
    * Attempt to load the triangulated mesh for this USD prim from the
    * cache. If the triangle mesh was already available in the local cache then
    * we don't need to triangulate it again.
    *
    * @return : Returns true if this triangle mesh was found in the local cache and loaded
    */
    bool loadTriangleMesh(void)
    {
        CARB_PROFILE_ZONE(0, "CookingTask::loadTriangleMesh");
        return false;
    }


    /**
    * Releases the triangle mesh buffers that got loaded from the cache as well as any collision meshes that were created
    */
    void releaseTriangleMeshData(void)
    {
        mTriangulationMesh = omni::physx::TriangulationMesh();
    }

    /**
    * Returns the current vertex buffer we are trying to cook.
    *
    * @param vertexCount : The a reference to return the number of vertices
    *
    * @return : Returns a pointer to the array of vertices
    */
    const float *getVertices(uint32_t &vertexCount)
    {
        const float *ret = nullptr;
        vertexCount = 0;

        // If we have a triangulation of the source mesh view we pull the vertices from it.
        if (m_triangulate)
        {
            ret = m_triangulate->getVertices(vertexCount);
        }
        else
        {
            // If we loaded the vertex data from the cache, then return our copy of it.
            vertexCount = static_cast<uint32_t>(mTriangulationMesh.points.size());
            ret = &mTriangulationMesh.points.data()->x;
        }

        return ret;
    }

    /**
    * Retrieves the triangulated index buffer
    *
    * @param triangleCount : A reference to return the number of triangles
    *
    * @return : Returns a pointer to the triangle indices
    */
    const uint32_t *getIndices(uint32_t &triangleCount)
    {
        const uint32_t *ret = nullptr;
        triangleCount = 0;

        if (m_triangulate)
        {
            ret = m_triangulate->getIndices(triangleCount);
        }
        else
        {
            triangleCount = static_cast<uint32_t>(mTriangulationMesh.indices.size() / 3);
            ret = mTriangulationMesh.indices.data();
        }
        return ret;
    }

    /**
    * Returns the per polygon face material indices. Since USD stores meshes as
    * polygons, not triangles, the material assignments correspond to polygons
    *
    * @param faceCount : The number of faces (polygons) in the source mesh
    *
    * @return : Returns an array of 'material indices' which ultimately may correspond to physics materials
    */
    const uint16_t* getMaterialIndices(uint32_t& faceCount) const
    {
        const uint16_t* ret = nullptr;
        faceCount = 0;

        if (m_triangulate)
        {
            ret = m_triangulate->getFaceMaterials(faceCount);
        }
        else
        {
            faceCount = static_cast<uint32_t>(mTriangulationMesh.faceMaterials.size());
            ret = mTriangulationMesh.faceMaterials.data();
        }
        return ret;
    }

    /**
    * Returns the mapping from triangle indices to polygon indices
    *
    * @param triangleCount : The number of triangles in the mesh
    *
    * @return : Returns a mapping from triangles to polygons
    */
    const uint32_t* getTriangleFaceMap(uint32_t& triangleCount) const
    {
        const uint32_t* ret = nullptr;
        triangleCount = 0;
        if (m_triangulate)
        {
            ret = m_triangulate->getTriangleFaceMap(triangleCount);
        }
        else
        {
            triangleCount = static_cast<uint32_t>(mTriangulationMesh.trianglesToFacesMapping.size());
            ret = mTriangulationMesh.trianglesToFacesMapping.data();
        }
        return ret;
    }

    /**
    * Raises the 'cancel' flag. Ideally the background thread would detect this and
    * abort the cooking process early if possible. Currently none of the existing cooking
    * tasks do this. However, once the cancel flag has been raised none of the cooking
    * results will be processed.
    */
    void cancel(bool invokeCallbackAnyway)
    {
        m_cancel = true;
        m_invokeCallbackAnyway = invokeCallbackAnyway;
    }

    /**
    * @return : Returns true if this task has been marked to be canceled
    */
    bool isCanceled(void) const
    {
        return m_cancel;
    }

    /**
    * When computing a convex hull or convex decomposition of a mesh
    * we cannot produce valid results for coplanar or extremely tiny objects.
    * This method will automatically detect coplanar meshes or meshes which are
    * extremely tiny and regenerate the triangle mesh such that it is at least 'minThickness'
    * in size. If the source mesh is coplanar, then the vertices will be projected one half
    * of minThickness along the normal of the plane. If the mesh is extremely tiny, then it
    * is represented as a simple bounding box of 'meshThickness' in size.
    *
    * @param minThickness : The minimum thickness allowed for a source mesh
    *
    * @return :Returns true if the mesh wasn't thick enough and had to be modified
    */
    bool checkMeshThickness(float minThickness)
    {
        CARB_PROFILE_ZONE(0, "CookingTask::checkMeshThickness");
        meshthickness::Mesh inputMesh;
        meshthickness::Mesh outputMesh;

        inputMesh.vertices = getVertices(inputMesh.vertexCount);
        inputMesh.indices = getIndices(inputMesh.triangleCount);


        carb::Float3 bmin = {0,0,0}, bmax = {0,0,0};
        computeBoundingBoxValues(inputMesh.vertexCount,inputMesh.vertices,bmin,bmax);
        float dx = bmax.x - bmin.x;
        float dy = bmax.y - bmin.y;
        float dz = bmax.z - bmin.z;
       
        float maxDim = fmaxf(dx, fmaxf(dy, dz));
        minThickness = fmaxf(minThickness, maxDim * 0.011f);

        bool isNewMesh = meshthickness::checkMeshThickness(inputMesh, outputMesh, minThickness);
        if (isNewMesh)
        {
            // If we have a new mesh, we need to release the old
            // mesh data and use this new one instead
            releaseTriangleMeshData();
            if (m_triangulate)
            {
                m_triangulate->release();
                m_triangulate = nullptr;
            }
            mTriangulationMesh.points.insert(mTriangulationMesh.points.begin(), 
                                            reinterpret_cast<const carb::Float3*>(outputMesh.vertices), 
                                            reinterpret_cast<const carb::Float3*>(outputMesh.vertices) + outputMesh.vertexCount);
            mTriangulationMesh.indices.insert(mTriangulationMesh.indices.begin(), outputMesh.indices, outputMesh.indices + outputMesh.triangleCount * 3);
            meshthickness::releaseMeshOutput(outputMesh);
        }
        return isNewMesh;
    }

    /**
    * The user has changed the collision properties on the prim and wants the data to
    * be recooked. However, we may currently be cooking the old data in a background
    * thread, so this represents the 'pending' task to be executed when the current one
    * is completed.
    *
    * @param ct : The new pending task 
    */
    void addPendingTask(CookingTaskImpl *ct)
    {
        // if we already had an existing pending task, this new one replaces it.
        // We cancel and delete the old pending task and then assign the new one.
        if (m_pending)
        {
            m_pending->cancel(false);
            delete m_pending;
        }
        m_pending = ct;
    }

    /**
    * @return : Returns the current pending task
    */
    CookingTaskImpl *getPendingTask(void)
    {
        CookingTaskImpl *ret = m_pending;
        m_pending = nullptr;
        return ret;
    }


    /**
    * A utility method to compute the bounding box on this set of vertices
    * and store the results in 'mBmin' and 'mBmax'.  This corresponds to the
    * centroid of the source mesh which is used to compute the 'explode view distance'
    * when the solid mesh debug visualization is enabled.
    *
    * @param vertexCount : The number of vertices
    * @param vertices : The array of vertices
    */
    void computeBoundingBox(uint32_t vertexCount, const float *vertices)
    {
        CARB_PROFILE_ZONE(0, "CookingTask::computeBoundingBox");
        computeBoundingBoxValues(vertexCount,vertices,mBmin,mBmax);
    }

    /**
    * Gets the bounding box computed with computeBoundingBox
    *
    * @param min : The min x,y,z
    * @param max : The max x,y,z
    */
    void getBoundingBox(carb::Float3& min, carb::Float3& max)
    {
        min = mBmin;
        max = mBmax;
    }


    /**
    * Initializes a triangulation process. In this initial state we perform a deep
    * copy of the caller-supplied mesh view so that we can perform the actual
    * triangulation of the polygon data in a background thread
    */
    void initTriangulation(const omni::physx::PhysxCookingMeshView& meshView)
    {
        CARB_PROFILE_ZONE(0, "CookingTask::initTriangulation");
        SAFE_RELEASE(m_triangulate);
        m_triangulate = triangulateusd::TriangulateUSDPrim::create(meshView);
    }

    /**
    * This method is called from a background thread. It takes the polygon data from the source mesh view
    * (read in initTriangulation) and converts it into an indexed triangle mesh.
    */
    void performTriangulation(void)
    {
        CARB_PROFILE_ZONE(0, "CookingTask::performTriangulation");
        if (m_triangulate)
        {
            m_triangulate->triangulate();
        }
    }

    /**
    * Safely wait for the background task to complete before returning. This is typically called from the
    * destructor of a cooking task so that it doesn't try to finalize the results until the background
    * thread is fully completed.
    */
    bool futureWait(int64_t timeoutMs)
    {
        // Wait until thread is finished
        if (m_taskStarted)
        {
            if(timeoutMs < 0)
            {
                m_future.wait();
                return true;
            }
            else
            {
                return m_future.wait_for(std::chrono::milliseconds(timeoutMs));
            }
        }
        return false;
    }

    /**
    * Retrieve this task's opaque identity key (see computeCookingTaskKey).
    *
    * @return : Returns the task key of the primitive we are operating against
    */
    const std::string& getTaskKey(void) const
    {
        return m_taskKey;
    }

    /**
    * Set the finalized state. Indicates that the results for cooking have been
    * fully processed or not. Not to be confused with 'finished' which indicates that the
    * background thread has completed.  Finalized means we have fully written out all of
    * the cooked results and it is safe to delete this cooking task.
    *
    * @param state : The finalized state
    */
    void setFinalized(bool state)
    {
        m_finalized = state;
    }

    /**
    * @return : Returns true if the results have been finalized
    */
    bool isFinalized(void) const
    {
        return m_finalized;
    }

    /**
    * Sets a flag to indicate that the background thread operation is complete. Not to be confused
    * with the 'finalized' flag, which indicates that the results have been saved to completion.
    *
    * @param state : The background thread completion state
    */
    void setFinished(bool state)
    {
        m_finished = state;
    }

    /**
    * @return : Returns true if the background thread has fully completed running.
    */
    bool isFinished(void) const
    {
        return m_finished;
    }

    void setSucceeded(bool state)
    {
        m_succeeded = state;
    }

    /**
    * @return : Returns true if the background thread has fully completed running.
    */
    bool isSucceeded(void) const
    {
        return m_succeeded;
    }

    void fireFinishedCallback(omni::physx::PhysxCookingResult::Enum result)
    {
            CARB_PROFILE_ZONE(0, "CookingTask::fireFinishedCallback");
            if(result == omni::physx::PhysxCookingResult::eVALID)
            {
                if(mResultObject.request->triangulation.isNeeded())
                {
                    CookingTask::fillTriangulationView(mTriangulationMesh, mResultObject.triangulationView);
                }
            }
            mResultObject.result = result;
            if(result != omni::physx::PhysxCookingResult::eERROR_CANCELED || m_invokeCallbackAnyway)
            {
                if (mResultObject.request->onFinished)
                {
                    mResultObject.request->onFinished(mResultObject);
                }
                for(auto& cb: mAdditionalRequestCallbacks)
                {
                    if(cb)
                    {
                        cb(mResultObject);
                    }
                }
            }
    }

    void saveCallbackFromRequest(const omni::physx::PhysxCookingComputeRequest& request)
    {
        mAdditionalRequestCallbacks.push_back(request.onFinished);
    }

    /**
    * @return : Returns true if the task has been added to the background thread.
    */
    bool hasStarted(void) const
    {
        return m_taskStarted;
    }

    // Owned reference, taken in CookingTask::setPxCudaAndGPUPointers() and released in
    // ~CookingTaskImpl(). Never a borrowed pointer: see REQ-COOK-CUDACTX-001.
    ::physx::PxCudaContextManager* mPxCudaContextManager = nullptr;
    ::physx::PxPhysicsGpu* mPxPhysicsGPU = nullptr;
    carb::Float3    mBmin{};    // The bounding box minimum for the source mesh
    carb::Float3    mBmax{};    // The bounding box maximum for the source mesh
    std::atomic<bool>   m_cancel{ false }; // Whether or not this task has been flagged to be canceled
    bool m_invokeCallbackAnyway{ false };  // If after cancelling we want to invoke the callback anyway
    std::atomic<bool>   m_finalized{ false }; // Whether or not the final results have bene saved out
    bool                m_taskStarted{ false }; // true if the task has been started
    std::atomic<bool> m_finished{ false };    // Set to true when the task is completed
    std::atomic<bool> m_succeeded{ false };    // Set to true when the task is succeded
    std::string m_taskKey; // opaque identity of the primitive we are cooking mesh data for (see computeCookingTaskKey).
    carb::tasking::Future<> m_future; // Pointer to the counter allocated for this task
    triangulateusd::TriangulateUSDPrim  *m_triangulate{ nullptr };
    // if the triangulation was loaded from the mesh cache...
    omni::physx::TriangulationMesh mTriangulationMesh;
    CookingTaskImpl     *m_pending{ nullptr };
    CookingTask *mParent{nullptr};
    ::physx::PxDefaultMemoryOutputStream mTriangulationStream;
    omni::physx::PhysxCookingComputeRequest mRequestObject;
    omni::physx::PhysxCookingComputeResult mResultObject;
    std::vector< omni::function<void(const omni::physx::PhysxCookingComputeResult& result)> > mAdditionalRequestCallbacks;

    double mMetersPerUnit = 1.0;
    bool mBuildGpuData = true;
    bool mBuildTriangleAdjacencies = true;
    std::string mPrimPathText;
};

/**
* Constructor for a CookingTask interface. We create an instance
* of the 'implementation' class here
*/
CookingTask::CookingTask(omni::physx::PhysxCookingComputeResult& result)
{
    mImpl = new CookingTaskImpl(this, result);
}

/**
* The destructor for a CookingTask interface will also delete the implemention class
*/
CookingTask::~CookingTask(void)
{
    delete mImpl;
}

// This method attempts to load the source triangle mesh associated with this meshKey
// if it exists in the local cache.
// If it does not exist, it returns false
bool CookingTask::loadTriangleMesh(void)
{
    return mImpl->loadTriangleMesh();
}

const float *CookingTask::getVertices(uint32_t &vertexCount)
{
    return mImpl->getVertices(vertexCount);
}

const uint32_t *CookingTask::getIndices(uint32_t &triangleCount)
{
    return mImpl->getIndices(triangleCount);
}

const uint16_t* CookingTask::getMaterialIndices(uint32_t& faceCount) const
{
    return mImpl->getMaterialIndices(faceCount);
}

/**
 * @implements REQ-COOK-CUDACTX-001
 * @covers AC-3
 */
void CookingTask::setPxCudaAndGPUPointers(::physx::PxCudaContextManager* cudaContextManager,
                                          ::physx::PxPhysicsGpu* physicsGPU)
{
    // A task can outlive the call that configured it - dispatchAsyncTasks() runs queued tasks on a
    // carb::tasking worker long afterwards, and GPU SDF cooking dereferences the manager there. The
    // host releases and recreates its manager from the main thread, so the task has to own a
    // reference for its whole lifetime rather than borrow the caller's.
    // Acquire before release so that re-setting the same manager cannot transiently drop it to zero.
    if (cudaContextManager)
    {
        cudaContextManager->acquireReference();
    }
    SAFE_RELEASE(mImpl->mPxCudaContextManager);
    mImpl->mPxCudaContextManager = cudaContextManager;
    mImpl->mPxPhysicsGPU = physicsGPU;
}

::physx::PxCudaContextManager* CookingTask::getPxCudaContextManager()
{
    return mImpl->mPxCudaContextManager;
}

::physx::PxPhysicsGpu* CookingTask::getPxPhysicsGPU()
{
    return mImpl->mPxPhysicsGPU;
}

void CookingTask::cancel(bool invokeCallbackAnyway)
{
    mImpl->cancel(invokeCallbackAnyway);
}

void CookingTask::performTriangulation(void) // called from another thread, perform the triangulation
{
    mImpl->performTriangulation();
}

void CookingTask::saveTriangulation(::physx::PxDefaultMemoryOutputStream& stream) // called from main thread, save the triangulation results
{
    mImpl->saveTriangulation(stream);
}

bool CookingTask::checkMeshThickness(float minThickness)
{
    return mImpl->checkMeshThickness(minThickness);
}

bool CookingTask::futureWait(int64_t timeoutMs)
{
    return mImpl->futureWait(timeoutMs);
}

bool CookingTask::isCanceled(void)
{
    return mImpl->isCanceled();
}

void CookingTask::getCRC(omni::physx::usdparser::MeshKey &crc) const
{
    crc = mImpl->mResultObject.cookedDataCRC;
}

const std::string& CookingTask::getTaskKey(void) const
{
    return mImpl->getTaskKey();
}

const std::string& CookingTask::getPrimPathText() const
{
    return mImpl->mPrimPathText;
}

bool CookingTask::setPrimPathText(const char* primPathText)
{
    if (primPathText)
    {
        mImpl->mPrimPathText = primPathText;
        return true;
    }
    return false;
}

void CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::Enum result)
{
    mImpl->fireFinishedCallback(result);
}

void CookingTask::computeBoundingBox(uint32_t vertexCount, const float *vertices)
{
    mImpl->computeBoundingBox(vertexCount,vertices);
}

void CookingTask::getBoundingBox(carb::Float3& min, carb::Float3& max)
{
    mImpl->getBoundingBox(min, max);
}

void CookingTask::setFinalized(bool state)
{
    mImpl->setFinalized(state);
}

bool CookingTask::isFinalized(void) const
{
    return mImpl->isFinalized();
}

void CookingTask::setFinished(bool state)
{
    mImpl->setFinished(state);
}

bool CookingTask::isFinished(void) const
{
    return mImpl->isFinished();
}

void CookingTask::setSucceeded(bool state)
{
    mImpl->setSucceeded(state);
}

bool CookingTask::isSucceeded(void) const
{
    return mImpl->isSucceeded();
}

bool CookingTask::hasStarted(void) const
{
    return mImpl->hasStarted();
}

const uint32_t *CookingTask::getTriangleFaceMap(uint32_t &tcount) const
{
    return mImpl->getTriangleFaceMap(tcount);
}

bool CookingTask::isValid(void)
{
    return mImpl->isValid();
}

CookingTask *CookingTask::getPendingTask(void)
{
    CookingTask *ret = nullptr;
    CookingTaskImpl *cti = mImpl->getPendingTask();
    if ( cti )
    {
        ret = cti->mParent;
    }
    return ret;
}

void CookingTask::addPendingTask(CookingTask *ct)
{
    mImpl->addPendingTask(ct->mImpl);
}

void CookingTask::setMetersPerUnit(double metersPerUnit)
{
    mImpl->mMetersPerUnit = metersPerUnit;
}

double CookingTask::getMetersPerUnit()const
{
    return mImpl->mMetersPerUnit;
}

void CookingTask::setBuildGpuData(bool buildGpuData)
{
    mImpl->mBuildGpuData = buildGpuData;
}

bool CookingTask::getBuildGpuData()const
{
    return mImpl->mBuildGpuData;
}

void CookingTask::setBuildTriangleAdjacencies(bool buildTriangleAdjacencies)
{
    mImpl->mBuildTriangleAdjacencies = buildTriangleAdjacencies;
}

bool CookingTask::getBuildTriangleAdjacencies()const
{
    return mImpl->mBuildTriangleAdjacencies;
}

::physx::PxCookingParams CookingTask::getCookingParams(const ::physx::PxTolerancesScale& tolerances) const
{
    ::physx::PxCookingParams params(tolerances);
    params.buildGPUData = mImpl->mBuildGpuData;
    params.buildTriangleAdjacencies = mImpl->mBuildTriangleAdjacencies;
    return params;
}

void CookingTask::saveCallbackFromRequest(const omni::physx::PhysxCookingComputeRequest& request)
{
    mImpl->saveCallbackFromRequest(request);
}

bool CookingTask::pump(carb::tasking::ITasking* tasking)
{
    return mImpl->pump(tasking);
}

::physx::PxDefaultMemoryOutputStream& CookingTask::getTriangulationOutputStream()
{
    return mImpl->mTriangulationStream;
}

omni::physx::PhysxCookingComputeResult& CookingTask::getResultObject()
{
    return mImpl->mResultObject;
}

omni::physx::TriangulationMesh& CookingTask::getTriangulationMesh()
{
    return mImpl->mTriangulationMesh;    
}

bool CookingTask::setupTaskFromRequest(const omni::physx::PhysxCookingComputeRequest& request, bool skipMeshProcessing)
{
    CARB_PROFILE_ZONE(0, "CookingTask::setupTaskFromRequest");
    setMetersPerUnit(request.primMeshMetersPerUnit);
    if (skipMeshProcessing)
    {
        // isValid() checks for mesh vertices being present -
        // we should refactor the base CookingTask to be independent of triangle meshes.
        return true;
    }
    setBuildGpuData(request.options.hasFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData));
    setBuildTriangleAdjacencies(request.options.hasFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData));
    if(!loadTriangleMesh())
    {
        mImpl->initTriangulation(request.primMeshView);
    }
    return isValid();
}

bool CookingTask::deserializeTriangleMesh(::physx::PxDefaultMemoryInputData idata, omni::physx::TriangulationMesh& triangulationMesh)
{
    CARB_PROFILE_ZONE(0, "CookingTask::deserializeTriangleMesh");
    // Version
    uint32_t version = 0;
    idata.read(&version, sizeof(version));
    if (version != omni::physx::PhysxCookingDataVersion_MeshTriangulation)
    {
        triangulationMesh = omni::physx::TriangulationMesh();
        return false;
    }

    uint32_t vsize, rsize;

    // Vertices
    uint32_t verticesCount = 0;
    idata.read(&verticesCount, sizeof(verticesCount));
    if(verticesCount > 0)
    {
        triangulationMesh.points.resize(verticesCount);
        vsize = sizeof(carb::Float3) * verticesCount;
        rsize = idata.read(triangulationMesh.points.data(), vsize);
        if (rsize != vsize)
        {
            triangulationMesh = omni::physx::TriangulationMesh();
            return false;        
        }
    }
    else
    {
        triangulationMesh = omni::physx::TriangulationMesh();
        return false;        
    }

    // Triangles
    uint32_t trianglesCount = 0;
    idata.read(&trianglesCount, sizeof(trianglesCount));
    if(trianglesCount > 0)
    {
        triangulationMesh.indices.resize(trianglesCount * 3);
        vsize = sizeof(uint32_t) * 3 * trianglesCount;
        rsize = idata.read(triangulationMesh.indices.data(), vsize);
        if (rsize != vsize)
        {
            triangulationMesh = omni::physx::TriangulationMesh();
            return false;        
        }
    }
    else
    {
        triangulationMesh = omni::physx::TriangulationMesh();
        return false;        
    }

    // Faces Triangles Mapping
    uint32_t trianglesMappingFaceCount = 0;
    idata.read(&trianglesMappingFaceCount, sizeof(trianglesMappingFaceCount));
    if(trianglesMappingFaceCount > 0)
    {
        vsize = sizeof(uint32_t) * trianglesMappingFaceCount;
        triangulationMesh.trianglesToFacesMapping.resize(trianglesMappingFaceCount);
        rsize = idata.read(triangulationMesh.trianglesToFacesMapping.data(), vsize);
        if (rsize != vsize)
        {
            triangulationMesh = omni::physx::TriangulationMesh();
            return false;        
        }
    }
    else
    {
        triangulationMesh = omni::physx::TriangulationMesh();
        return false;        
    }

    // Face Materials
    uint32_t faceMaterialsCount = 0;
    if(idata.read(&faceMaterialsCount, sizeof(faceMaterialsCount)) != sizeof(faceMaterialsCount))
    {
        triangulationMesh = omni::physx::TriangulationMesh();
        return false;
    }
    if (faceMaterialsCount > 0) // It's possible to get faceMaterialsCount == 0
    {
        vsize = sizeof(uint16_t) * faceMaterialsCount;
        triangulationMesh.faceMaterials.resize(faceMaterialsCount);
        rsize = idata.read(triangulationMesh.faceMaterials.data(), vsize);
        if (rsize != vsize)
        {
            triangulationMesh = omni::physx::TriangulationMesh();
            return false;
        }
    }

    // Number of used materials
    if(idata.read(&triangulationMesh.maxMaterialIndex, sizeof(uint16_t)) != sizeof(uint16_t))
    {
        triangulationMesh = omni::physx::TriangulationMesh();
        return false;
    }

    return true;
}

void CookingTask::fillTriangulationView(const omni::physx::TriangulationMesh& triangulationMesh, omni::physx::PhysxCookingMeshTriangulationView& triangulationView)
{
    triangulationView.points = {triangulationMesh.points.data(), triangulationMesh.points.size()};
    using triangle_t = uint32_t[3];
    triangulationView.triangles = {reinterpret_cast<const triangle_t*>(triangulationMesh.indices.data()), triangulationMesh.indices.size() / 3};
    triangulationView.trianglesFaceMap = {triangulationMesh.trianglesToFacesMapping.data(), triangulationMesh.trianglesToFacesMapping.size()};
    triangulationView.faceMaterials = {triangulationMesh.faceMaterials.data(), triangulationMesh.faceMaterials.size()};
}

} // end of cookingtask namespace
