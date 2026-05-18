// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"

#include "gpu/PxPhysicsGpu.h"
#include "gpu/PxGpu.h"
#include "PxSDFBuilder.h"

#include "../utility/MeshSimplifyInternal.h"

#include "extensions/PxTetMakerExt.h"

using namespace ::physx;
using namespace omni::physx;

namespace cookingtask
{

/**
* This class handles creating a cooked triangle mesh.
* Triangle meshes are produced from either the original
* source graphics mesh, a quadratic simplification of it,
* or as a result of a convex decomposition operation.
*/
class TriangleMeshCookingTask : public cookingtask::CookingTask
{
public:
    /**
     * The constructor for a triangle mesh cooking task
     *
     * @param desc : The descriptor for this triangle mesh we are cooking
     */
    TriangleMeshCookingTask(const omni::physx::TriangleMeshCookingParams& triangleDesc,
                            const omni::physx::SdfMeshCookingParams& sdfDesc,
                            omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
    {
        triangleMeshCookingParams = triangleDesc;
        sdfMeshCookingParams = sdfDesc;
    }

    /**
    * The destructor for the triangle mesh cooking task
    */
    virtual ~TriangleMeshCookingTask(void)
    {
        // Wait until thread is finished
        CookingTask::futureWait();
        // Finalize the results in the main thread
        if(!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    /**
    * This is the method invoked to actually perform the cooking operation
    * in a background thread.
    */
    virtual void performTask(void) final
    {
        CARB_PROFILE_ZONE(0, "TriangleMeshCookingTask::performTask");

        // If requested, we perform the triangulation of the source mesh
        CookingTask::performTriangulation();

        PxSDFBuilder* gpuSdfBuilder = NULL;

        uint32_t triangleCount;
        uint32_t vertexCount;
        // Get the source vertices
        const float *vertices = CookingTask::getVertices(vertexCount);

        // Compute the bounding box for the source mesh
        CookingTask::computeBoundingBox(vertexCount, vertices);

        const uint32_t *indices = CookingTask::getIndices(triangleCount);

        std::vector<carb::Float3> inputVertices;
        std::vector<uint32_t> inputIndices;

        float metric = 1.0f;
        if (triangleMeshCookingParams.mode != TriangleMeshMode::eORIGINAL_TRIANGLES)
            metric = triangleMeshCookingParams.simplificationMetric;

        if (sdfMeshCookingParams.sdfTriangleCountReductionFactor < 1.0f)
            metric = sdfMeshCookingParams.sdfTriangleCountReductionFactor;

        // See if the user has requested that the source mesh be simplified
        if (metric < 1.0f)
        {
            if (metric < 0)
            {
                metric = 0;
            }
            else if (metric > 1)
            {
                metric = 1;
            }
            uint32_t targetTriangleCount = (uint32_t)(float(triangleCount)*metric);
            if (targetTriangleCount < 32)
            {
                targetTriangleCount = 32;
            }
            carb::Float3* simplifyVertices = nullptr;
            uint32_t simplifyVerticesSize = 0;
            uint32_t* simplifyIndices = nullptr;
            uint32_t simplifyIndicesSize = 0;
            simplifyTriangleMeshInternal(
                simplifyVertices, simplifyVerticesSize, simplifyIndices, simplifyIndicesSize,
                nullptr, nullptr, nullptr, nullptr,
                (const carb::Float3*)vertices, vertexCount, indices, triangleCount * 3,
                targetTriangleCount, 1000000.0f, false, false,
                [](size_t numBytes) { return (void*)ICE_ALLOC(numBytes); });

            vertexCount = simplifyVerticesSize;
            triangleCount = simplifyIndicesSize / 3;

            inputVertices.insert(inputVertices.begin(), simplifyVertices, simplifyVertices + vertexCount);
            inputIndices.insert(inputIndices.begin(), simplifyIndices, simplifyIndices + triangleCount * 3);

            ICE_FREE(simplifyVertices);
            ICE_FREE(simplifyIndices);

            indices = inputIndices.data();
            vertices = &inputVertices.data()->x;
        }
        // So long s the cooking task has not yet been canceled, we continue
        if (!CookingTask::isCanceled())
        {
            // resolve materials
            PxU16* materialIndices = nullptr;
            uint32_t faceCount = 0;
            uint32_t faceTriangleCount = 0;
            const uint16_t *faceMaterials = CookingTask::getMaterialIndices(faceCount);
            const uint32_t *triangleFaceMap = CookingTask::getTriangleFaceMap(faceTriangleCount);
            if (faceMaterials && faceCount && triangleFaceMap && faceTriangleCount && faceTriangleCount == triangleCount && (triangleMeshCookingParams.mode == TriangleMeshMode::eORIGINAL_TRIANGLES))
            {
                materialIndices = new PxU16[faceTriangleCount];
                memset(materialIndices, 0, sizeof(PxU16) * faceTriangleCount);
                for (uint32_t triangleIndex = 0; triangleIndex < faceTriangleCount; triangleIndex++)
                {
                    const uint32_t faceIndex = triangleFaceMap[triangleIndex];
                    if (faceIndex < faceCount)
                    {
                        const uint16_t materialIndex = faceMaterials[faceIndex];
                        materialIndices[triangleIndex] = materialIndex;
                    }
                }
            }
            PxCookingParams cookingParams = CookingTask::getDefaultCookingParams();

            // Increase num prims per leaf to fit in large meshes. 
            cookingParams.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = 15;

            // handle welding, if positive take the value, if zero disble welding enable check for small value
            // if negative autocompute the welding tolerance based on object size
            if (triangleMeshCookingParams.meshWeldTolerance > 0.0f)
            {
                cookingParams.meshWeldTolerance = triangleMeshCookingParams.meshWeldTolerance;
                cookingParams.meshPreprocessParams |= PxMeshPreprocessingFlag::eWELD_VERTICES;
            }
            else if (triangleMeshCookingParams.meshWeldTolerance == 0.0f)
            {
                const float weldingToleranceCheck = 1e-10f;
                cookingParams.meshWeldTolerance = weldingToleranceCheck;
                cookingParams.meshPreprocessParams &= ~(PxMeshPreprocessingFlags)(PxMeshPreprocessingFlag::eWELD_VERTICES);
            }
            else
            {
                const float baseWeldTolerance = 1e-5f / static_cast<float>(CookingTask::getMetersPerUnit());
                // 0.1% of the model's longest axis
                carb::Float3 min;
                carb::Float3 max;
                CookingTask::getBoundingBox(min, max);
                const float meshSizeWeldTolerance = 0.0001f * PxMax(PxMax(max.x - min.x, max.y - min.y), max.z - min.z);
                cookingParams.meshWeldTolerance = PxMin(baseWeldTolerance, meshSizeWeldTolerance);
                cookingParams.meshPreprocessParams |= PxMeshPreprocessingFlag::eWELD_VERTICES;
            }

            if(CookingTask::getCookedDataOutputStreams().empty())
            {
                CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
            }
            PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();

            // cooking triggered by missing debug-viz solid mesh
            // but tri-mesh data may still be in local disk cache or in the cooked data api
            // See synchronous cooking task creation in getTriangleMeshInternal
            bool cookingSucceeded = outputStream.getSize() != 0;

            if(sdfMeshCookingParams.sdfResolution > 0)
            {
                if (!cookingSucceeded)
                {

                    // cook sdf tri mesh:
                    carb::Float3 min, max;
                    CookingTask::getBoundingBox(min, max);
                    // compute spacing from max BB extent:
                    float maxExtent = max.x - min.x;
                    maxExtent = std::max(maxExtent, max.y - min.y);
                    maxExtent = std::max(maxExtent, max.z - min.z);
                    uint32_t maxSDFResolution = 1250;
                    if (sdfMeshCookingParams.sdfResolution > maxSDFResolution)
                    {
                        CARB_LOG_WARN("Limited the SDF resolution to 1250 for prim(%s).", CookingTask::getPrimPathText().c_str());
                    }
                    const float spacing = maxExtent / float(::physx::PxMin(maxSDFResolution, sdfMeshCookingParams.sdfResolution));

                    PxBounds3 sdfBounds(PxVec3(min.x, min.y, min.z), PxVec3(max.x, max.y, max.z));
                    sdfBounds.fattenFast(sdfMeshCookingParams.sdfMargin * (sdfBounds.maximum - sdfBounds.minimum).magnitude());

                    PxSDFDesc sdfDesc;
                    sdfDesc.spacing = spacing;
                    sdfDesc.subgridSize = sdfMeshCookingParams.sdfSubgridResolution;
                    sdfDesc.narrowBandThicknessRelativeToSdfBoundsDiagonal = sdfMeshCookingParams.sdfNarrowBandThickness;
                    sdfDesc.sdfBounds = sdfBounds;
                    sdfDesc.numThreadsForSdfConstruction = 16;

                    PxCudaContextManager* cudaContextManager = CookingTask::getPxCudaContextManager();
                    physx::PxPhysicsGpu* gpu = CookingTask::getPxPhysicsGPU();
                    if (cudaContextManager && gpu)
                    {
                        gpuSdfBuilder = gpu->createSDFBuilder(cudaContextManager);
                        sdfDesc.sdfBuilder = gpuSdfBuilder;
                    }

                    PxArray<PxVec3> remeshedVertices;
                    PxArray<PxU32> remeshedIndices;                   
                    if (sdfMeshCookingParams.sdfEnableRemeshing)
                    {
                        //Avoid a remeshing resolution smaller than 50 because the remesher might fill up too much empty space then. It will be very
                        //fast at a resolution of 50, so that seems to be a reasonable threshold. The SDF itself can still have a lower resolution in case that's desired.
                        PxU32 remeshingResolution = PxClamp(sdfMeshCookingParams.sdfResolution, 50u, maxSDFResolution); 

                        PxTetMaker::remeshTriangleMesh(reinterpret_cast<const PxVec3*>(vertices), vertexCount, indices, triangleCount * 3, remeshingResolution, remeshedVertices, remeshedIndices);

                        sdfDesc.baseMesh.points.count = remeshedVertices.size();
                        sdfDesc.baseMesh.points.stride = sizeof(PxVec3);
                        sdfDesc.baseMesh.points.data = remeshedVertices.begin();

                        sdfDesc.baseMesh.triangles.count = remeshedIndices.size() / 3;
                        sdfDesc.baseMesh.triangles.stride = sizeof(PxU32) * 3;
                        sdfDesc.baseMesh.triangles.data = remeshedIndices.begin();
                    }

                    switch (sdfMeshCookingParams.sdfBitsPerSubgridPixel)
                    {
                    case 8:
                        sdfDesc.bitsPerSubgridPixel = PxSdfBitsPerSubgridPixel::e8_BIT_PER_PIXEL;
                        break;
                    case 16:
                        sdfDesc.bitsPerSubgridPixel = PxSdfBitsPerSubgridPixel::e16_BIT_PER_PIXEL;
                        break;
                    case 32:
                        sdfDesc.bitsPerSubgridPixel = PxSdfBitsPerSubgridPixel::e32_BIT_PER_PIXEL;
                        break;
                    default:
                        sdfDesc.bitsPerSubgridPixel = PxSdfBitsPerSubgridPixel::e16_BIT_PER_PIXEL;
                        break;
                    }

                    PxTriangleMeshDesc meshDesc;
                    meshDesc.points.count = vertexCount;
                    meshDesc.points.stride = sizeof(PxVec3);
                    meshDesc.points.data = vertices;
                    meshDesc.triangles.count = triangleCount;
                    meshDesc.triangles.stride = sizeof(uint32_t) * 3;
                    meshDesc.triangles.data = indices;
                    meshDesc.sdfDesc = &sdfDesc;
                    CARB_LOG_INFO("Cooking SDF for triangle mesh %s with resolution %u and spacing %f - this may take a while\n", CookingTask::getPrimPathText().c_str(), sdfMeshCookingParams.sdfResolution, spacing);
                    cookingSucceeded = PxCookTriangleMesh(cookingParams, meshDesc, outputStream);

                }
                CookingTask::setSucceeded(cookingSucceeded);
            }
            else
            {
                PxTriangleMeshDesc meshDesc;
                meshDesc.points.count = vertexCount;
                meshDesc.points.stride = sizeof(PxVec3);
                meshDesc.points.data = vertices;
                meshDesc.triangles.count = triangleCount;
                meshDesc.triangles.stride = sizeof(uint32_t) * 3;
                meshDesc.triangles.data = indices;
                if (materialIndices)
                {
                    meshDesc.materialIndices.data = materialIndices;
                    meshDesc.materialIndices.stride = sizeof(PxU16);
                }
                // Cook the triangle mesh
                {
                    CARB_PROFILE_ZONE(0, "TriangleMeshCookingTask::performTask: PxCookTriangleMesh");
                    if (!cookingSucceeded)
                    {
                        cookingSucceeded = PxCookTriangleMesh(cookingParams, meshDesc, outputStream);
                    }
                    CookingTask::setSucceeded(cookingSucceeded);
                }
            }
            // If we created any scratch material indices, release them
            if (materialIndices)
            {
                delete[] materialIndices;
            }
        }
        if (gpuSdfBuilder)
        {
            gpuSdfBuilder->release();
            gpuSdfBuilder = NULL;          
        }
        // Indicate that the background cooking operation is finished
        CookingTask::setFinished(true);
    }


    /**
    * This method finalizes the results
    */
    virtual void finalize(void) final
    {
        CARB_PROFILE_ZONE(0, "TriangleMeshCookingTask::finalize");
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
    
    omni::physx::TriangleMeshCookingParams triangleMeshCookingParams;
    omni::physx::SdfMeshCookingParams sdfMeshCookingParams;
};

CookingTask* createTriangleMeshCookingTask(const omni::physx::TriangleMeshCookingParams& triangleDesc,
                                           const omni::physx::SdfMeshCookingParams& sdfDesc,
                                           omni::physx::PhysxCookingComputeResult& result)
{
    return new TriangleMeshCookingTask(triangleDesc, sdfDesc, result);
}


}
