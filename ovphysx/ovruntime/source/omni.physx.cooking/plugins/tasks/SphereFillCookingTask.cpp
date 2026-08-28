// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

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
                          omni::physx::PhysxCookingComputeResult& result,
                          omni::convexdecomposition::ConvexDecomposition& convexDecomposition)
        : CookingTask(result), mConvexDecomposition(convexDecomposition)
    {
        m_desc = desc;
    }

    /**
    * The destructor for the convex decomposition cooking task
    */
    virtual ~SphereFillCookingTask(void)
    {
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
        // Bounding box of the source mesh, used by the 'explode view' solid-shaded
        // debug visualization to compute the offset from source-mesh center to hull center.
        CookingTask::computeBoundingBox(vertexCount, vertices);

        omni::convexdecomposition::VHACDHANDLE cd = mConvexDecomposition.createVHACD();
        omni::convexdecomposition::Parameters cdParam;
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
        omni::convexdecomposition::SimpleMesh sm;
        sm.vertexCount = vertexCount;
        sm.vertices = dvertices;
        sm.indices = indices;
        sm.triangleCount = triangleCount;

        // This operation can take a long time; TODO: support canceling it while in progress.
        uint32_t sphereCount;
        const omni::convexdecomposition::SimpleSphere *spheres =
            mConvexDecomposition.computeSphereApproximation(cd, cdParam, sm, sphereCount, true);
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
        mConvexDecomposition.releaseVHACD(cd);
        CookingTask::setSucceeded(true);
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
            CookingTask::setFinalized(true);
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_COOKING_FAILED);
        }
        else
        {
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
    omni::convexdecomposition::ConvexDecomposition& mConvexDecomposition;
};

CookingTask* createSphereFillCookingTask(const omni::physx::SphereFillCookingParams& desc,
                                         omni::physx::PhysxCookingComputeResult& result,
                                         omni::convexdecomposition::ConvexDecomposition& convexDecomposition)
{
    return new SphereFillCookingTask(desc, result, convexDecomposition);
}
}
