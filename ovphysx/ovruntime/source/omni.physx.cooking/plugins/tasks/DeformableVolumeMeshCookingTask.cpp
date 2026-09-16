// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-COOK-TASK-001
 * @covers AC-1 AC-2
 */

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>
#include <common/foundation/CarbPhysXCast.h>

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"

using namespace ::physx;
using namespace omni::physx;

namespace
{
/*
 * mapSurfaceTrisToTets builds a surface triangle to tetrahedron map given a list of surface triangles and the
 * tet mesh. It's more work than extracting the surface itself from scratch, but the order of the given surface triangle
 * list needs to be preserved.
 */
bool mapSurfaceTrisToTets(uint32_t* surfaceTriToTetMap,
                          const uint32_t* surfaceTriIndices,
                          const uint32_t surfaceTriCount,
                          const uint32_t* tetIndices,
                          const uint32_t tetCount)
{
    // We could just build the data structure to build the map given the surface triangles from scratch
    // from the tet mesh, but instead we re-use PxTetrahedronMeshExt::extractTetMeshSurface, which
    // gives the surface triangles and the map from scratch.
    PxArray<PxU32> triToTetMap;
    PxArray<PxU32> triIndices;

    PxTetrahedronMeshExt::extractTetMeshSurface(tetIndices, tetCount, false, triIndices, &triToTetMap, false);

    // Convert PxTetrahedronMeshExt::extractTetMeshSurface triToTetMap output to actual "indices per tet".
    for (uint32_t i = 0; i < triToTetMap.size(); ++i)
    {
        triToTetMap[i] /= 4;
    }

    if (triToTetMap.size() != surfaceTriCount || triToTetMap.size() * 3 != triIndices.size())
        return false;

    // Now sort both triangle list according to a triangle key (consisting of its ordered vertex indices)
    // Instead of sorting the triangle lists directly, we sort corresponding index lists.
    const uint32_t triCount = surfaceTriCount;
    const uint32_t* toTriangles = surfaceTriIndices;
    const uint32_t* fromTriangles = triIndices.begin();

    std::vector<uint32_t> toTriOrder(triCount);
    std::vector<uint32_t> fromTriOrder(triCount);

    for (uint32_t i = 0; i < triCount; ++i)
        toTriOrder[i] = i;

    for (uint32_t i = 0; i < triCount; ++i)
        fromTriOrder[i] = i;

    const auto orderVerts = [](carb::Uint3 t) noexcept -> carb::Uint3 {
        if (t.x > t.y)
            std::swap(t.x, t.y);
        if (t.y > t.z)
            std::swap(t.y, t.z);
        if (t.x > t.y)
            std::swap(t.x, t.y);
        return t;
    };

    const auto cmpTriangles = [orderVerts](const carb::Uint3& a, const carb::Uint3& b) noexcept -> bool {
        carb::Uint3 ao = orderVerts(a);
        carb::Uint3 bo = orderVerts(b);
        if (ao.x != bo.x)
            return ao.x < bo.x;
        if (ao.y != bo.y)
            return ao.y < bo.y;
        return ao.z < bo.z; // false when keys are equal
    };

    const auto loadTriangle = [](const uint32_t* indices, uint32_t triOffset) -> carb::Uint3 {
        return { indices[triOffset * 3 + 0], indices[triOffset * 3 + 1], indices[triOffset * 3 + 2] };
    };

    auto fromCmp = [fromTriangles, loadTriangle, cmpTriangles](uint32_t i, uint32_t j) noexcept -> bool {
        return cmpTriangles(loadTriangle(fromTriangles, i), loadTriangle(fromTriangles, j));
    };

    auto toCmp = [toTriangles, loadTriangle, cmpTriangles](uint32_t i, uint32_t j) noexcept -> bool {
        return cmpTriangles(loadTriangle(toTriangles, i), loadTriangle(toTriangles, j));
    };

    std::sort(fromTriOrder.begin(), fromTriOrder.end(), fromCmp);
    std::sort(toTriOrder.begin(), toTriOrder.end(), toCmp);

    // Now create the output map by remapping triToTetMap using the two orderings.
    for (uint32_t i = 0; i < triCount; ++i)
    {
        uint32_t fromIndex = fromTriOrder[i];
        uint32_t toIndex = toTriOrder[i];

        // Check that both orders are pointing to same triangle
        const carb::Uint3 fromTri = orderVerts(loadTriangle(fromTriangles, fromIndex));
        const carb::Uint3 toTri = orderVerts(loadTriangle(toTriangles, toIndex));
        if (fromTri.x != toTri.x || fromTri.y != toTri.y || fromTri.z != toTri.z)
            return false;

        uint32_t tetIndex = triToTetMap[fromIndex];
        surfaceTriToTetMap[toIndex] = tetIndex;
    }
    return true;
}
} // namespace

namespace cookingtask
{

// This task cooks a PxDeformableVolumeMesh
class DeformableVolumeMeshCookingTask : public cookingtask::CookingTask
{
public:
    DeformableVolumeMeshCookingTask(
                            const DeformableVolumeMeshCookingParams& params,
                            omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
    {
        static_assert(sizeof(PxVec3) == sizeof(carb::Float3));

        m_simPoints.resize(params.simPoints.size());
        std::memcpy(m_simPoints.data(), params.simPoints.data(), sizeof(PxVec3) * m_simPoints.size());

        m_simBindPoints.resize(params.simBindPoints.size());
        std::memcpy(m_simBindPoints.data(), params.simBindPoints.data(), sizeof(PxVec3) * m_simBindPoints.size());

        m_simIndices.resize(params.simIndices.size());
        std::memcpy(m_simIndices.data(), params.simIndices.data(), sizeof(carb::Int4) * m_simIndices.size());

        m_collBindPointsInSim.resize(params.collBindPointsInSim.size());
        std::memcpy(m_collBindPointsInSim.data(), params.collBindPointsInSim.data(), sizeof(PxVec3) * m_collBindPointsInSim.size());

        m_collIndices.resize(params.collIndices.size());
        std::memcpy(m_collIndices.data(), params.collIndices.data(), sizeof(carb::Int4) * m_collIndices.size());

        m_collSurfaceIndices.resize(params.collSurfaceIndices.size());
        std::memcpy(m_collSurfaceIndices.data(), params.collSurfaceIndices.data(), sizeof(carb::Int3) * m_collSurfaceIndices.size());

        // params.simToCookingTransform is carb::Double4[4] holding the producer's
        // GfMatrix4d row by row. PxMat44d holds the same sixteen doubles column by
        // column, and Gf row i is PhysX column i, so this is an element copy with no
        // transpose -- see the mapping table in common/foundation/MatrixTools.h.
        static_assert(sizeof(m_simToCookingTransform) == sizeof(params.simToCookingTransform));
        const carb::Double4* s2c = params.simToCookingTransform;
        m_simToCookingTransform =
            PxMat44d(PxVec4d(s2c[0].x, s2c[0].y, s2c[0].z, s2c[0].w), PxVec4d(s2c[1].x, s2c[1].y, s2c[1].z, s2c[1].w),
                     PxVec4d(s2c[2].x, s2c[2].y, s2c[2].z, s2c[2].w), PxVec4d(s2c[3].x, s2c[3].y, s2c[3].z, s2c[3].w));

        m_numTetsPerElement = params.numTetsPerElement;
    }

    virtual ~DeformableVolumeMeshCookingTask(void)
    {
        // Wait until thread is finished
        CookingTask::futureWait();
        // Finalize the results; store them in the USD prim and mesh caches
        if (!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    bool isValid(void) override
    {
        return true;
    }

    bool cookDeformableVolumeMeshData(const PxCookingParams& cookingParams,
        const std::vector<PxVec3>& simPoints,
        const std::vector<PxVec3>& simBindPoints,
        const std::vector<carb::Int4>& simIndices,
        const std::vector<PxVec3>& collBindPointsInSim,
        const std::vector<carb::Int4>& collIndices,
        const std::vector<carb::Int3>& collSurfaceIndices,
        const ::physx::PxMat44d& simToCookingTransform,
        uint32_t numTetsPerElement,
        const usdparser::MeshKey& crc)
    {
        PxArray<PxU32> simMeshIndices;
        PxArray<PxVec3> simMeshVertices;
        {
            simMeshIndices.resize(uint32_t(simIndices.size()) * 4);
            PxMemCopy(&simMeshIndices[0], simIndices.data(), sizeof(PxU32) * simMeshIndices.size());

            // Transform sim points into cooking space
            uint32_t numSimPoints = uint32_t(simPoints.size());
            simMeshVertices.resize(numSimPoints);
            {
                for (uint32_t i = 0; i < numSimPoints; ++i)
                {
                    simMeshVertices[i] = toPhysXf(simToCookingTransform.transform(toPhysXd(simPoints[i])));
                }
            }
        }

        PxArray<PxU32> collMeshIndicesMem;
        PxArray<PxU32>* collMeshIndicesRef = nullptr;
        PxArray<PxVec3> collMeshVertices;
        {
            const bool separateCollision = collBindPointsInSim.size() > 0;
            if (separateCollision)
            {
                collMeshIndicesMem.resize(uint32_t(collIndices.size()) * 4);
                PxMemCopy(&collMeshIndicesMem[0], collIndices.data(), sizeof(PxU32) * collMeshIndicesMem.size());
                collMeshIndicesRef = &collMeshIndicesMem;
                collMeshVertices.resize(uint32_t(collBindPointsInSim.size()));

                PxArray<PxVec3> simMeshBindVertices;
                simMeshBindVertices.resize(uint32_t(simPoints.size()));

                // Transform coll bind points into cooking space
                for (uint32_t i = 0; i < collMeshVertices.size(); ++i)
                {
                    collMeshVertices[i] = toPhysXf(simToCookingTransform.transform(toPhysXd(collBindPointsInSim[i])));
                }

                // Transform sim bind points into cooking space
                for (uint32_t i = 0; i < simMeshBindVertices.size(); ++i)
                {
                    simMeshBindVertices[i] = toPhysXf(simToCookingTransform.transform(toPhysXd(simBindPoints[i])));
                }

                // Need to construct embedding for collision mesh:
                // (simulation bind pose, sim mesh indices == rest shape indices, collision bind pose) -> embedding
                // (embedding, sim rest shape points, rest shape topo> -> collision rest shape
                PxArray<PxVec4> embeddingBary;
                PxArray<PxU32> embeddingTets;
                PxTetrahedronMeshExt::createPointsToTetrahedronMap(simMeshBindVertices, simMeshIndices, collMeshVertices,
                                                                   embeddingBary, embeddingTets);

                if (embeddingBary.size() == collMeshVertices.size() && embeddingTets.size() == collMeshVertices.size())
                {
                    for (uint32_t i = 0; i < collMeshVertices.size(); ++i)
                    {
                        const PxU32 tetIndex = embeddingTets[i];
                        const PxVec4& bary = embeddingBary[i];
                        const PxU32* tet = simMeshIndices.begin() + tetIndex*4;
                        const PxVec3& a = simMeshVertices[tet[0]];
                        const PxVec3& b = simMeshVertices[tet[1]];
                        const PxVec3& c = simMeshVertices[tet[2]];
                        const PxVec3& d = simMeshVertices[tet[3]];
                        collMeshVertices[i] = a*bary.x + b*bary.y + c*bary.z + d*bary.w;
                    }
                }
            }
            else
            {
                collMeshIndicesRef = &simMeshIndices;
                collMeshVertices.resize(uint32_t(simPoints.size()));

                // Just copy from sim vertices in cooking space
                PxMemCopy(collMeshVertices.begin(), simMeshVertices.begin(), sizeof(PxVec3) * collMeshVertices.size());
            }
        }

        PxTetrahedronMeshDesc collMeshDesc = PxTetrahedronMeshDesc(collMeshVertices, *collMeshIndicesRef);
        if (!collMeshDesc.isValid())
        {
            CARB_LOG_WARN(
                "PxTetrahedronMeshDesc used as collision mesh input to PxCookDeformableVolumeMesh is not valid");
            return false;
        }

        const PxTetrahedronMeshDesc::PxMeshFormat meshFormat =
            (numTetsPerElement == 5 || numTetsPerElement == 6)
                ? PxTetrahedronMeshDesc::PxMeshFormat::eHEX_MESH
                : PxTetrahedronMeshDesc::PxMeshFormat::eTET_MESH;
        PxTetrahedronMeshDesc simMeshDesc = PxTetrahedronMeshDesc(
            simMeshVertices, simMeshIndices, meshFormat, numTetsPerElement);
        if (!simMeshDesc.isValid())
        {
            CARB_LOG_WARN(
                "PxTetrahedronMeshDesc used as simulation mesh input to PxCookDeformableVolumeMesh is not valid");
            return false;
        }

        PxArray<PxI32> vertexToTet; // if this is empty, the sdk will compute the vertexToTet mapping internally.
        PxDeformableVolumeSimulationDataDesc simDesc = PxDeformableVolumeSimulationDataDesc(vertexToTet);
        if (!simDesc.isValid())
        {
            CARB_LOG_WARN("PxDeformableVolumeSimulationDataDesc used as input to PxCookDeformableVolumeMesh is not valid");
            return false;
        }
        if (CookingTask::getCookedDataOutputStreams().empty())
        {
            CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
        }
        PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();

        bool cookingSucceeded = outputStream.getSize() != 0;
        if (!cookingSucceeded)
        {
            cookingSucceeded = PxCookDeformableVolumeMesh(cookingParams, simMeshDesc, collMeshDesc, simDesc, outputStream);
        }

        // Append surface mesh from collision tet mesh
        {
            // We need to keep original USD surface, and that's why we can't just create a new surface list which
            // might have a different order.
            uint32_t numSurfaceTris = uint32_t(collSurfaceIndices.size());
            PxArray<PxU32> surfaceTriToTetMap(numSurfaceTris);
            cookingSucceeded = mapSurfaceTrisToTets(surfaceTriToTetMap.begin(),
                                                    reinterpret_cast<const uint32_t*>(collSurfaceIndices.data()),
                                                    numSurfaceTris,
                                                    collMeshIndicesRef->begin(),
                                                    collMeshIndicesRef->size()/4);

            outputStream.write(&numSurfaceTris, sizeof(uint32_t));
            outputStream.write(surfaceTriToTetMap.begin(), numSurfaceTris * sizeof(PxU32));
        }
        CookingTask::setSucceeded(cookingSucceeded);
        return cookingSucceeded;
    }

    virtual void performTask(void) final
    {
        if (!CookingTask::isCanceled())
        {
            usdparser::MeshKey crc;
            CookingTask::getCRC(crc);
            const PxCookingParams cookingParams = CookingTask::getDefaultCookingParams();
            cookDeformableVolumeMeshData(cookingParams, m_simPoints, m_simBindPoints, m_simIndices,
                m_collBindPointsInSim, m_collIndices, m_collSurfaceIndices,
                m_simToCookingTransform, m_numTetsPerElement, crc);
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
            CookingTask::setFinalized(true);
            CookingTask::fireFinishedCallback(omni::physx::PhysxCookingResult::eERROR_COOKING_FAILED);
        }
        else
        {
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

    //inputs
    std::vector<PxVec3> m_simPoints;
    std::vector<PxVec3> m_simBindPoints;
    std::vector<carb::Int4> m_simIndices;
    std::vector<PxVec3> m_collBindPointsInSim;
    std::vector<carb::Int4> m_collIndices;
    std::vector<carb::Int3> m_collSurfaceIndices;
    ::physx::PxMat44d m_simToCookingTransform;
    uint32_t m_numTetsPerElement{ 1 };
};

CookingTask* createDeformableVolumeMeshCookingTask(
    const DeformableVolumeMeshCookingParams& params,
    omni::physx::PhysxCookingComputeResult& result)
{
    return  new DeformableVolumeMeshCookingTask(params, result);
}
}
