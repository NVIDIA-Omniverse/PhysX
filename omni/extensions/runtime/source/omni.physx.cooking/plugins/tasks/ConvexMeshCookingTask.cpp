// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Implements the cooking task for single convex hulls

#include "UsdPCH.h"

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"

using namespace ::physx;
using namespace omni::physx;

namespace cookingtask
{

/**
* A helper method to clamp the convex hull vertex limit to valid values
*
* @param maxVertices : The number of maximum convex hull vertices in the descriptor
*
* @return : Returns the clamped max convex hull vertex limit
*/
static uint32_t clampConvexHullVertexLimit(uint32_t maxVertices)
{
    if (maxVertices > 64)
    {
        maxVertices = 64;
    }
    else if (maxVertices < 8)
    {
        maxVertices = 8;
    }
    return maxVertices;
}

/**
* This class handles cooking a single convex mesh in a background task.
*/
class ConvexMeshCookingTask : public cookingtask::CookingTask
{

public:
    ConvexMeshCookingTask(const omni::physx::ConvexMeshCookingParams& convexCookingParams,
                          omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
    {
        // Get the maximum number of convex hull vertices in the descriptor
        m_maxConvexHullVertices = convexCookingParams.maxHullVertices;
        // Get the minimum thickness of the source data
        m_minThickness = convexCookingParams.minThickness;
        m_signScale = convexCookingParams.signScale; // Make a copy of the mesh scale we are using
    }

    /**
    * The destructor for the cooking task. Make sure the background thread has completed
    * and that we have finalized the results
    */
    virtual ~ConvexMeshCookingTask(void)
    {
        CookingTask::futureWait();
        // Finalize the results; store them in the USD prim and mesh caches
        if(!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    /**
    * This operation happens in a background thread and, therefore, must be
    * completely thread safe.
    */
    virtual void performTask(void) final
    {
        CARB_PROFILE_ZONE(0, "ConvexMeshCookingTask::performTask");
        // If needed we triangulate the source mesh
        CookingTask::performTriangulation();
        // Here we detect coplanar and/or tiny meshes and make sure they
        // meet the 'm_minThickness' requirement
        bool thicknessAdjusted = CookingTask::checkMeshThickness(m_minThickness);
        if (thicknessAdjusted)
        {
            CARB_LOG_WARN("ConvexMeshCookingTask: adjusted the thickness of a very thin or very small mesh such that it meets the requirements for a GPU compatible convex hull collider. Prim %s", CookingTask::getPrimPathText().c_str());
        }

        // Here we must take into account negative mesh scale.
        // First retrieve the source vertices we are going to compute
        // the convex hull from.
        uint32_t vertexCount;
        const float *vertices = CookingTask::getVertices(vertexCount);
        std::vector<float> scaledVertices;
        // If the source mesh has a negative scale on any axis, we adjust the source vertices accordingly
        if (m_signScale.x < 0 || m_signScale.y < 0 || m_signScale.z < 0)
        {
            scaledVertices.insert(scaledVertices.begin(), vertices, vertices + vertexCount * 3);
            // When we are dealing with negative mesh scale we must actually
            // Modify the source vertex data to account for it.
            for (uint32_t i = 0; i < vertexCount; i++)
            {
                scaledVertices[i * 3 + 0] *= m_signScale.x;
                scaledVertices[i * 3 + 1] *= m_signScale.y;
                scaledVertices[i * 3 + 2] *= m_signScale.z;
            }
            vertices = scaledVertices.data();
        }

        // We compute the bounding box of the source mesh
        CookingTask::computeBoundingBox(vertexCount, vertices);

        // Pass the source vertices to the physx cooking library and store the results
        // in the output stream
        PxConvexMeshDesc convexDesc;
        convexDesc.points.count = vertexCount;  // Number of vertices
        convexDesc.points.stride = sizeof(PxVec3); // size of each vertex
        convexDesc.points.data = vertices; // Pointer to the vertex data
        // Raise the compute convex flag and make sure the results are GPU compatible
        convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
        // Make sure we pass in a valid maximum number of vertices in the convex hull
        convexDesc.vertexLimit = clampConvexHullVertexLimit(m_maxConvexHullVertices); // A.B. temp patch up seems to crash on GPU with 62 verts and a cylinder
        if(CookingTask::getCookedDataOutputStreams().empty())
        {
            CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
        }
        PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();
        // Here we actually cook the convex hull
        bool cookingSucceeded = outputStream.getSize() != 0;
        if(!cookingSucceeded)
        {
            PxConvexMeshCookingResult::Enum condition;
            const PxCookingParams cookingParams = CookingTask::getDefaultCookingParams();
            cookingSucceeded = PxCookConvexMesh(cookingParams, convexDesc, outputStream, &condition);
            if (condition == PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED)
            {
                CookingTask::getResultObject().resultWarnings.setFlag(PhysxCookingComputeResult::ResultWarning::CONVEX_POLYGON_LIMITS_REACHED, true);
                CARB_LOG_INFO("ConvexMeshCookingTask: polygon limit reached, resulting convex hull may be suboptimal. Prim %s", CookingTask::getPrimPathText().c_str());
            }
            else if (condition == PxConvexMeshCookingResult::eNON_GPU_COMPATIBLE)
            {
                CookingTask::getResultObject().resultWarnings.setFlag(PhysxCookingComputeResult::ResultWarning::FAILED_GPU_COMPATIBILITY, true);
                CARB_LOG_WARN("ConvexMeshCookingTask: failed to cook GPU-compatible mesh, collision detection will fall back to CPU. Collisions with particles and deformables will not work with this mesh. Prim %s", CookingTask::getPrimPathText().c_str());
            }
        }
        // If this task has not been canceled and the results are valid
        // we can then process the cooked data
        CookingTask::setSucceeded(cookingSucceeded);
        // Raise the flag which indicates that the background thread cooking process
        // has completed.
        CookingTask::setFinished(true);
    }

    /**
    * Once the task is complete, we now need to store the results into
    * the in memory mesh cache, the persistent storage mesh cache, and
    * to the USD prim itself.
    * This must all be done from the main thread since USD is not thread safe
    */
    virtual void finalize(void) final
    {
        CARB_PROFILE_ZONE(0, "ConvexMeshCookingTask::finalize");
        // If the task was canceled or already finalized, skip the saving step
        if (CookingTask::isCanceled() || CookingTask::isFinalized())
        {
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_CANCELED);
        }
        else if(!CookingTask::isSucceeded())
        {
            CookingTask::setFinalized(true); // Set the finalized semaphore to true
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_COOKING_FAILED);
        }
        else
        {
            // Set the finalized flag to true, indicating that we have fully
            // completed processing this cooked data
            CookingTask::setFinalized(true);

            // Save the triangulation to the cache in finalize, since saving it out in performTask
            // seemed to exhibit some thread safety issues
            CookingTask::saveTriangulation(getTriangulationOutputStream());
            // First we save the debug visual collision mesh representation to
            // the local cache
            PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();
            PxDefaultMemoryInputData inData(outputStream.getData(), outputStream.getSize());
            usdparser::MeshKey crc;
            CookingTask::getCRC(crc);

            omni::physx::PhysxCookingComputeResult& result = CookingTask::getResultObject();
            PxDefaultMemoryOutputStream& cookDataStream = outputStream;
            PhysxCookedDataSpan cookedDataSpan;
            result.cookedData = &cookedDataSpan;
            result.cookedDataNumElements = 1;
            cookedDataSpan.data = cookDataStream.getData();
            cookedDataSpan.sizeInBytes = cookDataStream.getSize();

            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eVALID);
        
        }
    }


    carb::Float3    m_signScale;
    uint32_t    m_maxConvexHullVertices{ 60 };
    float       m_minThickness{ 0.001f };
};

CookingTask* createConvexMeshCookingTask(const omni::physx::ConvexMeshCookingParams& desc,
                                         omni::physx::PhysxCookingComputeResult& result)
{
    return new ConvexMeshCookingTask(desc, result);
}

} // namespace cookingtask
