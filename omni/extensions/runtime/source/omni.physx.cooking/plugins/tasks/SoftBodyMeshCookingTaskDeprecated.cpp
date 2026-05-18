// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>
#include <common/foundation/TypeCast.h>

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"


using namespace ::physx;
using namespace omni::physx;
using namespace pxr;

namespace cookingtask
{

// This task cooks a soft body mesh
class SoftBodyMeshCookingTask : public cookingtask::CookingTask
{
public:
    SoftBodyMeshCookingTask(
        const omni::physx::SoftBodyMeshCookingParamsDeprecated& params,
        omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
        , m_simulationResolution(params.simulationResolution)
        , m_simulationNumTetsPerVoxel(params.simulationNumTetsPerVoxel)
        , m_kinematicEnabled(params.kinematicEnabled)
    {
        m_softBodyToWorldScaleNormalized = *reinterpret_cast<const GfVec3f*>(&params.softBodyToWorldScaleNormalized);

        m_collisionRestPoints.resize(params.collisionRestPoints.size());
        memcpy(m_collisionRestPoints.data(), params.collisionRestPoints.data(),
               m_collisionRestPoints.size() * sizeof(GfVec3f));

        m_collisionIndices.resize(params.collisionIndices.size());
        memcpy(m_collisionIndices.data(), params.collisionIndices.data(),
               m_collisionIndices.size() * sizeof(uint32_t));

        m_simulationRestPoints.resize(params.simulationRestPoints.size());
        memcpy(m_simulationRestPoints.data(), params.simulationRestPoints.data(),
               m_simulationRestPoints.size() * sizeof(GfVec3f));

        m_simulationIndices.resize(params.simulationIndices.size());
        memcpy(m_simulationIndices.data(), params.simulationIndices.data(),
               m_simulationIndices.size() * sizeof(uint32_t));

        m_collisionVertexToSimulationTetIndices.resize(params.collisionVertexToSimulationTetIndices.size());
        memcpy(m_collisionVertexToSimulationTetIndices.data(), params.collisionVertexToSimulationTetIndices.data(),
               m_collisionVertexToSimulationTetIndices.size() * sizeof(uint32_t));

        m_restPoints.resize(params.restPoints.size());
        memcpy(m_restPoints.data(), params.restPoints.data(),
               m_restPoints.size() * sizeof(GfVec3f));
    }

    virtual ~SoftBodyMeshCookingTask(void)
    {
        // Wait until thread is finished
        CookingTask::futureWait();
        // Finalize the results; store them in the USD prim and mesh caches
        if(!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    bool isValid(void) const
    {
        return true;
    }

    bool cookSoftBodyMeshData()
    {
        const PxCookingParams cookingParams = CookingTask::getDefaultCookingParams();

        // Transform collision rest points and skin rest points into 'softbody' space and pre-scale with normalized softbody scale
        uint32_t numCollisionRestPoints = uint32_t(m_collisionRestPoints.size());
        uint32_t numRestPoints = uint32_t(m_restPoints.size());

        uint32_t numCollSkinPoints = numCollisionRestPoints + (m_kinematicEnabled ? 0 : numRestPoints);
        PxArray<PxVec3> collisionMeshVertices(numCollSkinPoints);
        {
            const GfVec3f* collisionMeshRestPositions = reinterpret_cast<const GfVec3f*>(&m_collisionRestPoints[0]);
            for (uint32_t i = 0; i < numCollisionRestPoints; ++i)
            {
                GfVec3f v = GfCompMult(collisionMeshRestPositions[i], m_softBodyToWorldScaleNormalized);
                collisionMeshVertices[i] = toPhysX(v);
            }
        }

        // Push skin mesh vertices to the back of the collision vertices for physx side visual deformation
        if (!m_kinematicEnabled && numRestPoints > 0)
        {
            // Transform skin mesh positions into 'softbody' space and pre-scale with normalized softbody scale
            const GfVec3f* skinMeshRestPositions = reinterpret_cast<const GfVec3f*>(&m_restPoints[0]);
            for (uint32_t i = 0; i < numRestPoints; ++i)
            {
                GfVec3f v = GfCompMult(skinMeshRestPositions[i], m_softBodyToWorldScaleNormalized);
                collisionMeshVertices[numCollisionRestPoints + i] = toPhysX(v);
            }
        }

        PxArray<PxU32> collisionMeshIndices;
        collisionMeshIndices.assign(&m_collisionIndices[0], &m_collisionIndices[0] + m_collisionIndices.size());
        PxTetrahedronMeshDesc collMeshDesc = PxTetrahedronMeshDesc(collisionMeshVertices, collisionMeshIndices);
        if (!collMeshDesc.isValid())
        {
            CARB_LOG_WARN("PxTetrahedronMeshDesc used as collision mesh input to PxCooking::cookSoftBodyMesh is not valid");
            return false;
        }

        // Transform simulation rest positions into 'softbody' space and pre-scale with normalized softbody scale
        PxArray<::PxVec3> simulationMeshVertices;
        simulationMeshVertices.reserve(static_cast<PxU32>(m_simulationRestPoints.size()));
        const GfVec3f* simulationMeshRestPositions = reinterpret_cast<const GfVec3f*>(&m_simulationRestPoints[0]);
        for (size_t i = 0u; i < m_simulationRestPoints.size(); ++i)
        {
            GfVec3f v = GfCompMult(simulationMeshRestPositions[i], m_softBodyToWorldScaleNormalized);
            simulationMeshVertices.pushBack(toPhysX(v));
        }
        PxArray<PxU32> simulationIndices;
        simulationIndices.assign(&m_simulationIndices[0], &m_simulationIndices[0] + m_simulationIndices.size());

        const PxTetrahedronMeshDesc::PxMeshFormat meshFormat = m_simulationResolution > 0 && !m_kinematicEnabled ?
            PxTetrahedronMeshDesc::PxMeshFormat::eHEX_MESH : PxTetrahedronMeshDesc::PxMeshFormat::eTET_MESH;
        PxTetrahedronMeshDesc simMeshDesc =
            PxTetrahedronMeshDesc(simulationMeshVertices, simulationIndices, meshFormat, m_simulationNumTetsPerVoxel);
        if (!simMeshDesc.isValid())
        {
            CARB_LOG_WARN("PxTetrahedronMeshDesc used as simulation mesh input to PxCooking::cookSoftBodyMesh is not valid");
            return false;
        }

        PxArray<PxI32> vertexToTet; // if this is empty, the sdk will compute the vertexToTet mapping internally.
        if (m_collisionVertexToSimulationTetIndices.size() == collisionMeshVertices.size())
        {
            vertexToTet.reserve(PxU32(m_collisionVertexToSimulationTetIndices.size()));
            for (PxU32 i = 0; i < m_collisionVertexToSimulationTetIndices.size(); ++i)
                vertexToTet.pushBack(m_collisionVertexToSimulationTetIndices[i]);

            //Verify that the vertexToTet mapping is actually valid - discard otherwise
            for (PxU32 i = 0; i < vertexToTet.size(); ++i)
            {
                if (vertexToTet[i] < 0 || 4 * vertexToTet[i] >= PxI32(simulationIndices.size()))
                {
                    vertexToTet.clear();
                    break;
                }
                const PxU32* tet = &simulationIndices[4 * vertexToTet[i]];
                if (tet[0] >= simulationMeshVertices.size() || tet[1] >= simulationMeshVertices.size() || tet[2] >= simulationMeshVertices.size() || tet[3] >= simulationMeshVertices.size())
                {
                    vertexToTet.clear();
                    break;
                }
            }
        }
        PxSoftBodySimulationDataDesc simDesc = PxSoftBodySimulationDataDesc(vertexToTet);
        if (!simDesc.isValid())
        {
            CARB_LOG_WARN("PxSoftBodySimulationDataDesc used as input to PxCooking::cookSoftBodyMesh is not valid");
            return false;
        }
        if(CookingTask::getCookedDataOutputStreams().empty())
        {
            CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
        }
        PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();
        // Here we actually cook the convex hull
        bool cookingSucceeded = outputStream.getSize() != 0;
        if(!cookingSucceeded)
        {
            // Get cooking, set params, and backup original state
            cookingSucceeded = PxCookSoftBodyMesh(cookingParams, simMeshDesc, collMeshDesc, simDesc, outputStream);
        }

        // Append surface mesh
        {
            PxArray<PxU32> surfaceTriangleIndices;
            PxTetrahedronMeshExt::extractTetMeshSurface(collisionMeshIndices.begin(), collisionMeshIndices.size()/4, false, surfaceTriangleIndices);
            uint32_t numSurfaceTriangles = surfaceTriangleIndices.size()/3;
            outputStream.write(&numSurfaceTriangles, sizeof(uint32_t));
            outputStream.write(&surfaceTriangleIndices[0], numSurfaceTriangles*sizeof(carb::Uint3));
        }
        CookingTask::setSucceeded(cookingSucceeded);
        return cookingSucceeded;
    }

    virtual void performTask(void) final
    {
        UsdStageWeakPtr stage = CookingTask::getStage();
        if (stage)
        {
            UsdPrim softBodyPrim = stage->GetPrimAtPath(CookingTask::getPrimPath());
            if (softBodyPrim)
            {
                if (!CookingTask::isCanceled())
                {
                    cookSoftBodyMeshData();
                }
            }
        }
        CookingTask::setFinished(true);
    }

    virtual void finalize(void) final
    {
        // If the task was canceled or already finalized, skip the saving step
        if (CookingTask::isCanceled() || CookingTask::isFinalized())
        {
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_CANCELED);
        }
        else if (!CookingTask::isSucceeded())
        {
            CookingTask::setFinalized(true); // Set the finalized semaphore to true
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_COOKING_FAILED);
        }
        else
        {
            // Set the finalized flag to true, indicating that we have fully
            // completed processing this cooked data
            CookingTask::setFinalized(true);

            PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();
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

    // input
    VtArray<GfVec3f> m_collisionRestPoints;
    VtArray<uint32_t> m_collisionIndices;
    VtArray<GfVec3f> m_simulationRestPoints;
    VtArray<uint32_t> m_simulationIndices;
    VtArray<uint32_t> m_collisionVertexToSimulationTetIndices;
    VtArray<GfVec3f> m_restPoints;
    GfVec3f m_softBodyToWorldScaleNormalized;
    uint32_t m_simulationResolution;
    uint32_t m_simulationNumTetsPerVoxel;
    bool m_kinematicEnabled;
};

/***
 * Create a CookingTask to cook for a soft body simulation
 *
 * @param softBodyDesc : The descriptor for this soft body
 * @param softBodyToWorldScaleNormalized : ???
 * @param softBodyPrim : The source UsdGeomMesh primitive we are cooking the soft body for.
 * @param physics : Associated PxPhysics instance.
 *
 * @return : Returns a pointer to the CookingTask for this operation
 */
CookingTask* createSoftBodyMeshCookingTaskDeprecated(
    const SoftBodyMeshCookingParamsDeprecated& params,
    omni::physx::PhysxCookingComputeResult& result)
{
    return new SoftBodyMeshCookingTask(params,
                                       result);
}
}
