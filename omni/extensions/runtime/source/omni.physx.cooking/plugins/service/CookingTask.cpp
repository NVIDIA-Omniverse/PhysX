// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>
#include <carb/tasking/ITasking.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/Utilities.h>
#include <common/utilities/MemoryMacros.h>

#include "CookingTask.h"

#include "../utility/TriangulateUsdMeshPrim.h"
#include "../utility/MeshThickness.h"
#include "CookingComputeService.h"

using namespace pxr;

// This code is in the 'cookingtask' namespace
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
        if(result.request->primId == 0)
        {
            if(SdfPath::IsValidPathString(mPrimPathText))
            {
                m_primPath = SdfPath(mPrimPathText);
            }
        }
        else
        {
            m_primPath = intToPath(result.request->primId);
        }
    }

    /**
    * The cooking task destructor
    */
    ~CookingTaskImpl(void)
    {
        delete m_pending; // if there was a pending task we nuke it too
        SAFE_RELEASE(m_triangulate); // Release the USD geom mesh triangulation interface if it was created
        releaseTriangleMeshData();  // Release the memory for any triangle mesh data that was loaded from the cache
        // If we started a task, then decrement the global task counter
        if (m_taskStarted)
        {
            g_activeTaskCount--;
        }
    }

    /**
    * Returns true if we have found valid UsdGeomMesh data in the source prim to operate on.
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
        // We only allow 'MAX_ACTIVE_TASK_COUNT' tasks to be running at once.
        // It's just an arbitrary gate to prevent having thousands of tasks running
        // at the same time which would be overkill.
        // If fewer than 'MAX_ACTIVE_TASK_COUNT' tasks are currently running then
        // we can start this one.
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
    * We write the triangulated mesh results to the cache so that the
    * next time we encounter this geometry in a USD prim we don't have
    * to triangulate it again.
    *
    * UsdGeomMesh prims can often times have millions of triangles in them.
    * Additionally, UsdGeomMeshes are not stored as triangle meshes but rather
    * as polygon meshes. Finally, accessing the data associated with a UsdGeomMesh
    * must happen in the main thread. Therefore, it can be extremely CPU intensive
    * and blocking on the main thread to triangulate a UsdGeomMesh.
    *
    * What we do here is we copy the source data from the UsdGeomMesh in the main
    * thread but actually perform the triangulation itself in a background thread
    * so as to be non-blocking.
    *
    * Once the triangulation has been computed, we then write it out to the local mesh cache
    * so that, in the future when we encounter the same geometry again (identified by
    * the 128 bit MeshKey) we don't have to perform the triangulation again.
    *
    * The last step is currently performed in the main thread because we found
    * some thread safety issues with storing data from the background thread to
    * the cache.
    *
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

        // If we have a triangulation of a UsdPrim we pull the vertices from it.
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
        meshthickness::Mesh inputMesh; // The input mesh data
        meshthickness::Mesh outputMesh; // The output mesh data

        inputMesh.vertices = getVertices(inputMesh.vertexCount);
        inputMesh.indices = getIndices(inputMesh.triangleCount);


        carb::Float3 bmin = {0,0,0}, bmax = {0,0,0};
        computeBoundingBoxValues(inputMesh.vertexCount,inputMesh.vertices,bmin,bmax);
        float dx = bmax.x - bmin.x;
        float dy = bmax.y - bmin.y;
        float dz = bmax.z - bmin.z;
       
        float maxDim = fmaxf(dx, fmaxf(dy, dz));
        minThickness = fmaxf(minThickness, maxDim * 0.011f);

        // Call the 'checkMeshThickness' method to compute the new mesh if needed.
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
            m_pending->cancel(false); // Cancel the old pending task
            delete m_pending;   // Release the old pending task
        }
        // This is the new pending task
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
    * copy of the source data in the UsdGeomMesh primitive so that we can
    * perform the actual triangulation of the polygon data in a background thread
    *
    * @param usdPrim : The source primitive we wish to triangulate
    */
    void initTriangulation(const UsdPrim &usdPrim, uint16_t& maxMaterialIndex)
    {
        CARB_PROFILE_ZONE(0, "CookingTask::initTriangulation");
        SAFE_RELEASE(m_triangulate); // release any previous instance of the triangulation class
        // Create in instance of the triangulation class relative to this UsdGeomMesh
        m_triangulate = triangulateusd::TriangulateUSDPrim::create(usdPrim, maxMaterialIndex);
    }

    void initTriangulation(const omni::physx::PhysxCookingMeshView& meshView)
    {
        CARB_PROFILE_ZONE(0, "CookingTask::initTriangulation");
        SAFE_RELEASE(m_triangulate);
        m_triangulate = triangulateusd::TriangulateUSDPrim::create(meshView);
    }

    /**
    * This method is called from a background thread. It takes the polygon data from the source UsdGeomMesh
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
    * Retrieve the fully qualified path name for the USD prim we are cooking
    *
    * @return : Returns the SdfPath of the primitive we are operating against
    */
    SdfPath getPrimPath(void) const
    {
        return m_primPath;
    }

    UsdStageWeakPtr getStage() const
    {
        return UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt((long)mResultObject.request->primStageId));
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
    SdfPath    m_primPath; // the name of the primitive we are cooking mesh data for.
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

void CookingTask::setPxCudaAndGPUPointers(::physx::PxCudaContextManager* cudaContextManager,
                                          ::physx::PxPhysicsGpu* physicsGPU)
{
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

void CookingTask::initTriangulation(const UsdPrim &usdPrim, uint16_t& maxMaterialIndex)
{
    mImpl->initTriangulation(usdPrim, maxMaterialIndex);
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

SdfPath CookingTask::getPrimPath(void) const
{
    return mImpl->getPrimPath();
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

UsdStageWeakPtr CookingTask::getStage() const
{
    return mImpl->getStage();
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
