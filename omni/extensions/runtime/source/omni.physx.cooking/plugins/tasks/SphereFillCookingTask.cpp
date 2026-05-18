// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This code implements asynchronous cooking for a convex decomposition

#include "UsdPCH.h"

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>
#include <omni/convexdecomposition/ConvexDecomposition.h>

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"


using namespace ::physx;
using namespace omni::physx;

namespace cookingtask
{

// This task computes a convex decomposition of a source mesh
class SphereFillCookingTask : public cookingtask::CookingTask
{
public:
    /**
    * The constructor for a convex decomposition cooking task instance
    * @param desc : The descriptor for this convex decomposition operation
    */
    SphereFillCookingTask(const omni::physx::SphereFillCookingParams& desc,
                          omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
    {
        m_desc = desc; // make a copy of the descriptor
    }

    /**
    * The destructor for the convex decomposition cooking task
    */
    virtual ~SphereFillCookingTask(void)
    {
        // Wait until thread is finished
        CookingTask::futureWait();
        // Finalize the results; store them in the USD prim and mesh caches
        if(!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    /**
    * This is the portion of the cooking task which runs in a background thread
    */
    virtual void performTask(void) final
    {
        CookingTask::performTriangulation(); // Finish triangulation of the source mesh (if requested)

        if(CookingTask::getCookedDataOutputStreams().empty())
        {
            CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
        }
        PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();

        bool thicknessAdjusted = CookingTask::checkMeshThickness(defaultCookedMinThickness); // Make sure the source mesh meets the minimum thickness requirement
        if (thicknessAdjusted)
        {
            CARB_LOG_WARN("SphereFillCookingTask: adjusted the thickness of a very thin or very small mesh such that it meets the requirements for a GPU compatible convex hull collider. Prim %s", CookingTask::getPrimPathText().c_str());
        }

        uint32_t vertexCount;
        uint32_t triangleCount;
        const float *vertices = CookingTask::getVertices(vertexCount);
        const uint32_t *indices = CookingTask::getIndices(triangleCount);
        std::vector<float> scaledVertices;
        // If we have negative mesh scale we need to modify the source
        // vertices accordingly
        if (m_desc.signScale.x < 0 || m_desc.signScale.y < 0 || m_desc.signScale.z < 0)
        {
            scaledVertices.insert(scaledVertices.begin(), vertices, vertices + vertexCount * 3);
            for (size_t i = 0; i < vertexCount; i++)
            {
                scaledVertices[i * 3 + 0] *=  m_desc.signScale.x;
                scaledVertices[i * 3 + 1] *=  m_desc.signScale.y;
                scaledVertices[i * 3 + 2] *=  m_desc.signScale.z;
            }
            vertices = scaledVertices.data();
        }
        // Compute the bounding box of the source mesh. This
        // is actually used to implement the 'explode view' when
        // performing a solid shaded debug mesh visualization.
        // Explode view needs to know the center of the source mesh
        // and the center of the convex hull, to compute the distance
        // vector to move the child relative to the parent
        CookingTask::computeBoundingBox(vertexCount, vertices);

        omni::convexdecomposition::ConvexDecomposition* iConvexDecomposition = carb::getCachedInterface<omni::convexdecomposition::ConvexDecomposition>();

        // Create a VHACD instance
        omni::convexdecomposition::VHACDHANDLE cd = iConvexDecomposition->createVHACD();
        omni::convexdecomposition::Parameters cdParam;
        // Initialize the VHACD parameters from the descriptor
        cdParam.maxSpheres = m_desc.maxSpheres;
        cdParam.voxelFillMode = (omni::convexdecomposition::VoxelFillMode)(m_desc.fillMode);
        cdParam.voxelResolution = m_desc.voxelResolution;
        cdParam.maxSeedCount = m_desc.seedCount;

        // Copy the float vertices into double precision
        double *dvertices = new double[vertexCount * 3];
        for (uint32_t i = 0; i < vertexCount * 3; i++)
        {
            dvertices[i] = vertices[i];
        }
        // Initialize the source mesh data
        omni::convexdecomposition::SimpleMesh sm;
        sm.vertexCount = vertexCount;
        sm.vertices = dvertices;
        sm.indices = indices;
        sm.triangleCount = triangleCount;

        // Perform the convex decomposition
        // Note: this operation can take a long time. We should support the
        // ability to cancel it while 'in progress'
        uint32_t sphereCount;
        const omni::convexdecomposition::SimpleSphere *spheres = iConvexDecomposition->computeSphereApproximation(cd,cdParam,sm,sphereCount,true);
        delete[]dvertices;
        if (CookingTask::isCanceled())
        {
        }
        else
        {
            outputStream.write(&sphereCount,sizeof(sphereCount));
            for (uint32_t i=0; i<sphereCount; i++)
            {
                usdparser::SpherePhysxPoint p;
                const auto &s = spheres[i];
                p.position.x = (float)s.center.x;
                p.position.y = (float)s.center.y;
                p.position.z = (float)s.center.z;
                p.radius = (float)s.radius;
                outputStream.write(&p,sizeof(p));
            }
        }
        // Release the convex decomposition interface
        iConvexDecomposition->releaseVHACD(cd);
        CookingTask::setSucceeded(true);
        // Raise the flag indicating that the cooking task has completed
        CookingTask::setFinished(true);
    }

    /**
    * This is the finalize operation, called from the main thread, which will
    * write out the results to the UsdPrim and to the local cache
    */
    virtual void finalize(void) final
    {
        CARB_PROFILE_ZONE(0, "SphereFillCookingTask::finalize");
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
        {            // Set the finalized flag to true, indicating that we have fully
            // completed processing this cooked data
            CookingTask::setFinalized(true);

            // Save the triangulation to the cache in finalize, since saving it out in performTask
            // seemed to exhibit some thread safety issues
            CookingTask::saveTriangulation(getTriangulationOutputStream());

            PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();
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

    omni::physx::SphereFillCookingParams m_desc;
};

CookingTask* createSphereFillCookingTask(const omni::physx::SphereFillCookingParams& desc,
                                         omni::physx::PhysxCookingComputeResult& result)
{
    return new SphereFillCookingTask(desc, result);
}
}
