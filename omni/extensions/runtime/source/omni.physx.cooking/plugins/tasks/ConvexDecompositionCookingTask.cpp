// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This code implements asynchronous cooking for a convex decomposition

#include "UsdPCH.h"

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"

#include <omni/convexdecomposition/ConvexDecomposition.h>

#define SAVE_SOURCE_MESH_AS_OBJ 0

using namespace ::physx;
using namespace omni::physx;

namespace cookingtask
{

#if KEEP_POLYGONS
// A debug method to save a PxConvexMesh to disk as a triangulated
// wavefront OBJ file
void saveConvexMesh(const PxConvexMeshDesc &desc)
{
    static uint32_t gCount = 0;
    gCount++;
    char scratch[512];
    snprintf(scratch,sizeof(scratch),"d:\\convex%03d.obj", gCount);
    FILE *fph = fopen(scratch,"wb");
    if ( fph )
    {
        const float *vertices = (const float *)desc.points.data;
        for (uint32_t i=0; i<desc.points.count; i++)
        {
            fprintf(fph,"v %0.9f %0.9f %0.9f\n", vertices[i*3+0],vertices[i*3+1],vertices[i*3+2]);
        }
        const PxHullPolygon *polygons = (const PxHullPolygon *)desc.polygons.data;
        const uint32_t *polygonIndices = (const uint32_t *)desc.indices.data;
        for (uint32_t i=0; i<desc.polygons.count; i++)
        {
            const PxHullPolygon &p = polygons[i];
            const uint32_t *indices = &polygonIndices[p.mIndexBase];
            fprintf(fph,"f");
            for (uint32_t j=0; j<p.mNbVerts; j++)
            {
                fprintf(fph," %d", indices[j]+1);
            }
            fprintf(fph,"\n");
        }
        fclose(fph);
    }
}
#endif
/**
* Helper method to clamp the convex hull vertex limit into a known
* valid range.
*
* @param maxVertices : The user supplied maximum convex hull vertices
*
* @return : Returns the maximum number of vertices to use
*/
#if 0
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
#endif

// This task computes a convex decomposition of a source mesh
class ConvexDecompositionCookingTask : public cookingtask::CookingTask
{
public:
    /**
    * The constructor for a convex decomposition cooking task instance
    * @param desc : The descriptor for this convex decomposition operation
    */
    ConvexDecompositionCookingTask(const omni::physx::ConvexDecompositionCookingParams& desc,
                                   omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
    {
        m_desc = desc; // make a copy of the descriptor
    }

    /**
    * The destructor for the convex decomposition cooking task
    */
    virtual ~ConvexDecompositionCookingTask(void)
    {
        // Wait until thread is finished
        CookingTask::futureWait();
        // Finalize the results
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
        CARB_PROFILE_ZONE(0, "ConvexDecompositionCookingTask::performTask");
        CookingTask::performTriangulation(); // Finish triangulation of the source mesh (if requested)

        bool thicknessAdjusted = CookingTask::checkMeshThickness(m_desc.minThickness); // Make sure the source mesh meets the minimum thickness requirement
        if (thicknessAdjusted)
        {
            CARB_LOG_WARN("ConvexDecompositionCookingTask: adjusted the thickness of a very thin or very small mesh such that it meets the requirements for a GPU compatible convex hull collider. Prim %s", CookingTask::getPrimPathText().c_str());
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
                scaledVertices[i * 3 + 0] *= m_desc.signScale.x;
                scaledVertices[i * 3 + 1] *= m_desc.signScale.y;
                scaledVertices[i * 3 + 2] *= m_desc.signScale.z;
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

        omni::convexdecomposition::ConvexDecomposition* iConvexDecomposition =  carb::getCachedInterface<omni::convexdecomposition::ConvexDecomposition>();

        // Create a VHACD instance
        omni::convexdecomposition::VHACDHANDLE cd = iConvexDecomposition->createVHACD();
        omni::convexdecomposition::Parameters cdParam;
        // Initialize the VHACD parameters from the descriptor
        cdParam.maxConvexHullCount = m_desc.maxConvexHulls;
        cdParam.maxHullVertices = m_desc.maxHullVertices;
        cdParam.voxelResolution = m_desc.voxelResolution;
        cdParam.errorPercentage = m_desc.errorPercentage;
        cdParam.shrinkWrap = m_desc.shrinkWrap;
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
#if SAVE_SOURCE_MESH_AS_OBJ
        g_pConvexDecomposition->saveOBJ("d:\\source-mesh.obj",sm,false);
#endif

#if REPORT_PROGRESS
        report("computeVHACD:Begin");
#endif
        // Perform the convex decomposition
        // Note: this operation can take a long time. We should support the
        // ability to cancel it while 'in progress'
        iConvexDecomposition->computeVHACD(cd, cdParam, sm);
        delete[]dvertices;
        if (CookingTask::isCanceled())
        {
#if REPORT_PROGRESS
            report("computeVHACD complete:Throwing away results as a new approximation was requested");
#endif
        }
        else
        {
#if REPORT_PROGRESS
            report("computeVHACD complete:Keeping results");
#endif
            // Here we process the convex decomposition results by cooking
            // each individual convex hull
            bool validConvex = false;
            const PxU32 cdNumConvexes = iConvexDecomposition->getConvexHullCount(cd);
            bool cookingSucceeded = true;
            CookingTask::getCookedDataOutputStreams().clear();

            for (PxU32 c = 0; c < cdNumConvexes; c++)
            {
                omni::convexdecomposition::SimpleMesh convexHull;
                iConvexDecomposition->getConvexHull(cd, c, convexHull);

                if (convexHull.vertexCount < 4)
                    continue;

                validConvex = true;
                // Convert from double to single for PhysX convex.
                PxVec3* singlePrecisionPoints = new PxVec3[convexHull.vertexCount];

                for (PxU32 d = 0; d < convexHull.vertexCount; d++)
                {
                    singlePrecisionPoints[d].x = float(convexHull.vertices[d * 3]);
                    singlePrecisionPoints[d].y = float(convexHull.vertices[d * 3 + 1]);
                    singlePrecisionPoints[d].z = float(convexHull.vertices[d * 3 + 2]);
                }

                // Cook this convex hull
// By default we do *not* have the PhysX Cooking library recompute the convex hull from the source vertices
// since we already a full solution. Instead we copy the full polygon data for the convex hull and cook that.
#define KEEP_POLYGONS 0

                PxConvexMeshDesc convexMeshDesc;
#if !KEEP_POLYGONS
                convexMeshDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
#endif

                convexMeshDesc.points.count = convexHull.vertexCount;
                convexMeshDesc.points.data = singlePrecisionPoints;
                convexMeshDesc.points.stride = sizeof(PxVec3);

#if KEEP_POLYGONS
                convexMeshDesc.indices.data = convexHull.polygonIndices;
                convexMeshDesc.indices.count = convexHull.polygonIndexCount;
                convexMeshDesc.indices.stride = sizeof(uint32_t);

                PxHullPolygon *polygons = new PxHullPolygon[convexHull.polygonCount];
                for (uint32_t i=0; i<convexHull.polygonCount; i++)
                {
                    PxHullPolygon &dest = polygons[i];
                    const omni::convexdecomposition::PolygonData &source = convexHull.polygons[i];
                    dest.mIndexBase = source.startIndex;
                    dest.mNbVerts = source.pointCount;
                    dest.mPlane[0] = (float)source.planeEquation[0];
                    dest.mPlane[1] = (float)source.planeEquation[1];
                    dest.mPlane[2] = (float)source.planeEquation[2];
                    dest.mPlane[3] = (float)source.planeEquation[3];
                }
                convexMeshDesc.polygons.data = polygons;
                convexMeshDesc.polygons.count = convexHull.polygonCount;
                convexMeshDesc.polygons.stride = sizeof(PxHullPolygon);

                // For debugging purposes only, saves the convex hull to disk
                // as a wavefront OBJ 
                //saveConvexMesh(convexMeshDesc);

#endif
                convexMeshDesc.vertexLimit = PxMax(convexHull.vertexCount, 8u); //clampConvexHullVertexLimit(convexMeshDesc.vertexLimit);
                CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
                PxDefaultMemoryOutputStream* outStream = CookingTask::getCookedDataOutputStreams().back().get();

                CARB_ASSERT(convexMeshDesc.isValid());
                bool res = false;
                if (convexMeshDesc.isValid())
                {
                    PxConvexMeshCookingResult::Enum condition;
                    const PxCookingParams cookingParams = CookingTask::getDefaultCookingParams();
                    res = PxCookConvexMesh(cookingParams, convexMeshDesc, *outStream, &condition);
                    if (condition == PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED)
                    {
                        CookingTask::getResultObject().resultWarnings.setFlag(PhysxCookingComputeResult::ResultWarning::CONVEX_POLYGON_LIMITS_REACHED, true);
                        CARB_LOG_INFO("ConvexDecompositionTask: polygon limit reached, resulting convex hull may be suboptimal. Prim %s", CookingTask::getPrimPathText().c_str());
                    }
                    else if (condition == PxConvexMeshCookingResult::eNON_GPU_COMPATIBLE)
                    {
                        CookingTask::getResultObject().resultWarnings.setFlag(PhysxCookingComputeResult::ResultWarning::FAILED_GPU_COMPATIBILITY, true);
                        CARB_LOG_WARN("ConvexDecompositionTask: failed to cook GPU-compatible mesh, collision detection will fall back to CPU. Collisions with particles and deformables will not work with this mesh. Prim %s", CookingTask::getPrimPathText().c_str());
                    }
                }                

                if (!res)
                {
                    CookingTask::getCookedDataOutputStreams().pop_back();
                    cookingSucceeded = false;
                    CARB_LOG_ERROR("ConvexDecompositionTask: failed to create convex mesh, prim: %s", CookingTask::getPrimPathText().c_str());
                }
                delete[] singlePrecisionPoints;
#if KEEP_POLYGONS
                delete[]polygons;
#endif
            }            
            CookingTask::setSucceeded(cookingSucceeded && validConvex);
        }
        // Release the convex decomposition interface
        iConvexDecomposition->releaseVHACD(cd);
        // Raise the flag indicating that the cooking task has completed
        CookingTask::setFinished(true);
    }

    /**
    * This is the finalize operation, called from the main thread, which will
    * write out the results to the UsdPrim and to the local cache
    */
    virtual void finalize(void) final
    {
        CARB_PROFILE_ZONE(0, "ConvexDecompositionCookingTask::finalize");
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
            CookingTask::setFinalized(true); // Set the finalized semaphore to true

            // Save the triangulation to the cache in finalize, since saving it out in performTask
            // seemed to exhibit some thread safety issues
            CookingTask::saveTriangulation(getTriangulationOutputStream());

            // Write out the cooked data to a series or memory stream
            std::vector<PxDefaultMemoryInputData> inDataVector;
            inDataVector.reserve(CookingTask::getCookedDataOutputStreams().size());
            for (auto &i : CookingTask::getCookedDataOutputStreams())
            {
                inDataVector.push_back(PxDefaultMemoryInputData(i->getData(), i->getSize()));
            }
            
            usdparser::MeshKey crc;
            CookingTask::getCRC(crc);

            omni::physx::PhysxCookingComputeResult& result = CookingTask::getResultObject();
            std::vector<omni::physx::PhysxCookedDataSpan> cookedDataSpans;
            cookedDataSpans.reserve(CookingTask::getCookedDataOutputStreams().size());
            for (auto &i : CookingTask::getCookedDataOutputStreams())
            {
                cookedDataSpans.push_back(omni::physx::PhysxCookedDataSpan(i->getData(), i->getSize()));
            }
            result.cookedData = cookedDataSpans.data();
            result.cookedDataNumElements = cookedDataSpans.size();

            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eVALID);
        }
    }

    omni::physx::ConvexDecompositionCookingParams m_desc;
};

CookingTask* createConvexDecompositionCookingTask(const omni::physx::ConvexDecompositionCookingParams& desc,
                                                  omni::physx::PhysxCookingComputeResult& result)
{
    return new ConvexDecompositionCookingTask(desc, result);
}

}
