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

#include "../utility/MeshSimplifyInternal.h"

#include <common/foundation/Algorithms.h>
#include "extensions/PxTetMakerExt.h"

using namespace ::physx;
using namespace omni::physx;
using namespace pxr;

namespace cookingtask
{

static void writeDeformableBodyTetMeshDataDeprecated(PxDefaultMemoryOutputStream& outData,
    const carb::Float3* collisionRestPoints, const uint32_t collisionRestPointsSize,
    const uint32_t* collisionIndices, const uint32_t collisionIndicesSize, const carb::Float3* simulationRestPoints, const uint32_t simulationRestPointsSize,
    const uint32_t* simulationIndices, const uint32_t simulationIndicesSize,
    const int32_t* collisionVertexToSimulationTetIndices, const uint32_t collisionVertexToSimulationTetIndicesSize,
    const uint32_t* collisionVertexToSkinTriVertexIndices, const carb::Float3* collisionVertexToSkinTriBarycentrics, const uint32_t collisionVertexToSkinTriSize)
{
    outData.write(&collisionRestPointsSize, sizeof(uint32_t));
    outData.write(collisionRestPoints, sizeof(carb::Float3)*collisionRestPointsSize);

    outData.write(&collisionIndicesSize, sizeof(uint32_t));
    outData.write(collisionIndices, sizeof(uint32_t)*collisionIndicesSize);

    outData.write(&simulationRestPointsSize, sizeof(uint32_t));
    outData.write(simulationRestPoints, sizeof(carb::Float3)*simulationRestPointsSize);

    outData.write(&simulationIndicesSize, sizeof(uint32_t));
    outData.write(simulationIndices, sizeof(uint32_t)*simulationIndicesSize);

    outData.write(&collisionVertexToSimulationTetIndicesSize, sizeof(uint32_t));
    outData.write(collisionVertexToSimulationTetIndices, sizeof(int32_t)*collisionVertexToSimulationTetIndicesSize);

    outData.write(&collisionVertexToSkinTriSize, sizeof(uint32_t));
    outData.write(collisionVertexToSkinTriVertexIndices, sizeof(uint32_t)*collisionVertexToSkinTriSize*3);
    outData.write(collisionVertexToSkinTriBarycentrics, sizeof(carb::Float3)*collisionVertexToSkinTriSize);
}

void readDeformableBodyTetMeshDataDeprecated(PhysxCookingDeformableBodyTetMeshDataDeprecated& out,
    const PhysxCookedDataSpan& cookedData)
{
    const uint8_t* dataPtr = reinterpret_cast<const uint8_t*>(cookedData.data);

    out.collisionRestPointsSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.collisionRestPoints = reinterpret_cast<const carb::Float3*>(dataPtr);
    dataPtr += out.collisionRestPointsSize * sizeof(carb::Float3);

    out.collisionIndicesSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.collisionIndices = reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += out.collisionIndicesSize * sizeof(uint32_t);

    out.simulationRestPointsSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.simulationRestPoints = reinterpret_cast<const carb::Float3*>(dataPtr);
    dataPtr += out.simulationRestPointsSize * sizeof(carb::Float3);

    out.simulationIndicesSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.simulationIndices = reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += out.simulationIndicesSize * sizeof(uint32_t);

    out.collisionVertexToSimulationTetIndicesSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.collisionVertexToSimulationTetIndices = reinterpret_cast<const int32_t*>(dataPtr);
    dataPtr += out.collisionVertexToSimulationTetIndicesSize * sizeof(int32_t);

    out.collisionVertexToSkinTriSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.collisionVertexToSkinTriVertexIndices = reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += uint64_t(out.collisionVertexToSkinTriSize) * 3 * sizeof(uint32_t);

    out.collisionVertexToSkinTriBarycentrics = reinterpret_cast<const carb::Float3*>(dataPtr);
    dataPtr += out.collisionVertexToSkinTriSize * sizeof(carb::Float3);

    CARB_ASSERT(dataPtr == reinterpret_cast<const uint8_t*>(cookedData.data) + cookedData.sizeInBytes);
}

// This class handles cooking a single deformable body simulation mesh in a background task.
class DeformableBodyTetMeshCookingTaskDeprecated : public cookingtask::CookingTask
{
public:
    DeformableBodyTetMeshCookingTaskDeprecated(
        const omni::physx::DeformableBodyTetMeshCookingParamsDeprecated& params,
        omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
        , m_simpEnabled(params.simpParams.enabled)
        , m_simpRemeshing(params.simpParams.remeshing)
        , m_simpRemeshingResolution(params.simpParams.remeshingResolution)
        , m_simpTargetTriangleCount(params.simpParams.targetTriangleCount)
        , m_simpForceConforming(params.simpParams.forceConforming)
        , m_normalizedQuantizedScale(params.normalizedQuantizedScale)
        , m_simulationResolution(params.simulationResolution)
        , m_simulationNumTetsPerVoxel(params.simulationNumTetsPerVoxel)
        , m_kinematicEnabled(params.kinematicEnabled)
    {
        m_skinRestPoints.resize(params.skinRestPoints.size());
        for (uint32_t p = 0; p < m_skinRestPoints.size(); ++p)
        {
            const carb::Float3& src = params.skinRestPoints[p];
            m_skinRestPoints[p] = {
                src.x * m_normalizedQuantizedScale.x,
                src.y * m_normalizedQuantizedScale.y,
                src.z * m_normalizedQuantizedScale.z
            };
        }

        m_skinEarliestPoints.resize(params.skinEarliestPoints.size());
        for (uint32_t p = 0; p < m_skinEarliestPoints.size(); ++p)
        {
            const carb::Float3& src = params.skinEarliestPoints[p];
            m_skinEarliestPoints[p] = {
                src.x * m_normalizedQuantizedScale.x,
                src.y * m_normalizedQuantizedScale.y,
                src.z * m_normalizedQuantizedScale.z
            };
        }
    }

    virtual ~DeformableBodyTetMeshCookingTaskDeprecated(void)
    {
        // Wait until thread is finished
        CookingTask::futureWait();
        // Finalize the results; store them in the USD prim and mesh caches
        if(!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    inline float length(const carb::Float3& a)
    {
        return sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    }

    inline float distance(const carb::Float3& a, const carb::Float3& b)
    {
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float dz = b.z - a.z;
        return sqrtf(dx * dx + dy * dy + dz * dz);
    }

    inline carb::Float3 diff(const carb::Float3& a, const carb::Float3& b)
    {
        return carb::Float3{ a.x - b.x, a.y - b.y, a.z - b.z };
    }

    inline carb::Float3 cross(const carb::Float3& left, const carb::Float3& right)
    {
        return carb::Float3{ left.y * right.z - left.z * right.y,
                left.z * right.x - left.x * right.z,
                left.x * right.y - left.y * right.x };
    }

    float averageTriangleEdgeLength(const carb::Float3* vertices, const uint32_t* indices, const uint32_t indicesSize)
    {
        float triEdgeSum = 0.0f;
        int numTriangles = int(indicesSize) / 3;

        for (int i = 0; i < numTriangles; ++i)
        {
            const carb::Float3& a = vertices[indices[3 * i + 0]];
            const carb::Float3& b = vertices[indices[3 * i + 1]];
            const carb::Float3& c = vertices[indices[3 * i + 2]];
            triEdgeSum += distance(a, b);
            triEdgeSum += distance(a, c);
            triEdgeSum += distance(b, c);
        }

        return triEdgeSum / (3 * numTriangles);
    }

    float surfaceArea(const carb::Float3* vertices, const uint32_t* indices, const uint32_t indicesSize)
    {
        float areaSum = 0.0f;
        int numTriangles = int(indicesSize) / 3;

        for (int i = 0; i < numTriangles; ++i)
        {
            const carb::Float3& a = vertices[indices[3 * i + 0]];
            const carb::Float3& b = vertices[indices[3 * i + 1]];
            const carb::Float3& c = vertices[indices[3 * i + 2]];
            areaSum += length(cross(diff(b, a), diff(c, a)));

        }

        return 0.5f * areaSum;
    }

    void boundingBox(const carb::Float3* vertices, const uint32_t verticesSize, carb::Float3& min, carb::Float3& max)
    {
        min = carb::Float3{ FLT_MAX, FLT_MAX, FLT_MAX };
        max = carb::Float3{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
        for (uint32_t i = 0; i < verticesSize; ++i)
        {
            const carb::Float3& v = vertices[i];
            if (v.x > max.x) max.x = v.x;
            if (v.x < min.x) min.x = v.x;
            if (v.y > max.y) max.y = v.y;
            if (v.y < min.y) min.y = v.y;
            if (v.z > max.z) max.z = v.z;
            if (v.z < min.z) min.z = v.z;
        }
    }

    // adjusted from GuDistancePointTriangle.h, copy in PhysXTetFinder.cpp, PhysXTriFinder.cpp
    PX_INLINE PxReal distancePointTriangleSquared(PxVec3& bary, const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c,
                                                    const PxVec3& ab, const PxVec3& ac)
    {
        // Check if P in vertex region outside A
        //const PxVec3 ab = b - a;
        //const PxVec3 ac = c - a;
        const PxVec3 ap = p - a;
        const float d1 = ab.dot(ap);
        const float d2 = ac.dot(ap);
        if (d1<=0.0f && d2<=0.0f)
        {
            bary = PxVec3(1.0f, 0.0f, 0.0f);
            return ap.dot(ap);	// Barycentric coords 1,0,0
        }

        // Check if P in vertex region outside B
        const PxVec3 bp = p - b;
        const float d3 = ab.dot(bp);
        const float d4 = ac.dot(bp);
        if (d3>=0.0f && d4<=d3)
        {
            bary = PxVec3(0.0f, 1.0f, 0.0f);
            return bp.dot(bp); // Barycentric coords 0,1,0
        }

        // Check if P in edge region of AB, if so return projection of P onto AB
        const float vc = d1*d4 - d3*d2;
        if(vc<=0.0f && d1>=0.0f && d3<=0.0f)
        {
            const float v = d1 / (d1 - d3);
            PxVec3 diff = p - (a + v * ab);
            bary = PxVec3(1.0f - v, v, 0.0f);
            return diff.dot(diff); //barycentric coords (1-v, v, 0)
        }

        // Check if P in vertex region outside C
        const PxVec3 cp = p - c;
        const float d5 = ab.dot(cp);
        const float d6 = ac.dot(cp);
        if (d6>=0.0f && d5<=d6)
        {
            bary = PxVec3(0.0f, 0.0f, 1.0f);
            return cp.dot(cp); // Barycentric coords 0,0,1
        }

        // Check if P in edge region of AC, if so return projection of P onto AC
        const float vb = d5*d2 - d1*d6;
        if(vb<=0.0f && d2>=0.0f && d6<=0.0f)
        {
            const float w = d2 / (d2 - d6);
            PxVec3 diff = p - (a + w * ac);
            bary = PxVec3(1.0f - w, 0.0f, w);
            return diff.dot(diff); // barycentric coords (1-w, 0, w)
        }

        // Check if P in edge region of BC, if so return projection of P onto BC
        const float va = d3*d6 - d5*d4;
        if(va<=0.0f && (d4-d3)>=0.0f && (d5-d6)>=0.0f)
        {
            const float w = (d4-d3) / ((d4 - d3) + (d5-d6));
            PxVec3 diff = p - (b + w * (c-b));
            bary = PxVec3(0.0f, 1.0f - w, w);
            return diff.dot(diff); // barycentric coords (0, 1-w, w)
        }

        // P inside face region. Compute Q through its barycentric coords (u,v,w)
        const float denom = 1.0f / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        PxVec3 diff = p - (a + ab*v + ac*w);
        bary = PxVec3(1.0f - v - w, v, w);
        return diff.dot(diff);
    }

    inline carb::Float3 evaluateBarycentric(const carb::Uint3& triangle, const carb::Float3& bary, const float* vertices)
    {
        const ::physx::PxVec3* v = reinterpret_cast<const ::physx::PxVec3*>(vertices);
        return fromPhysX(bary.x * v[triangle.x] + bary.y * v[triangle.y] + bary.z * v[triangle.z]);
    }

    template <typename T = int>
    struct ResultBuffer
    {
        ~ResultBuffer()
        {
            if (ptr)
            {
                ICE_FREE(ptr);
                ptr = nullptr;
            }
            size = 0;
        }

        static void* allocate(size_t numBytes)
        {
            return ICE_ALLOC(numBytes);
        }

        T* ptr = nullptr;
        uint32_t size = 0;
    };

    // This operation happens in a background thread and, therefore, must be
    // completely thread safe.
    virtual void performTask(void) final
    {
        if (!CookingTask::isCanceled())
        {
            CookingTask::performTriangulation();
            uint32_t skinTriCount;
            std::vector<uint32_t> skinTriIndicesVector;
            {
                const uint32_t* indices = CookingTask::getIndices(skinTriCount);
                skinTriIndicesVector.insert(skinTriIndicesVector.begin(), indices, indices + skinTriCount * 3);
            }
            uint32_t* skinTriIndices = skinTriIndicesVector.data();
            uint32_t skinTriIndicesSize = skinTriCount * 3u;
            carb::Float3* skinRestPoints = m_skinRestPoints.data();
            uint32_t skinRestPointsSize = uint32_t(m_skinRestPoints.size());

            ResultBuffer<carb::Float3> collisionTriPoints;
            ResultBuffer<uint32_t> collisionTriIndices;
            ResultBuffer<uint32_t> skinToCollisionTriVertsMap;
            ResultBuffer<uint32_t> collisionVertsToSkinTriIds;

            bool kinematicEnabled = m_kinematicEnabled;

            // check for planar meshes
            {
                PxSimpleTriangleMesh skinMesh;
                skinMesh.points.count = skinRestPointsSize;
                skinMesh.points.data = skinRestPoints;
                skinMesh.triangles.count = skinTriIndicesSize / 3u;
                skinMesh.triangles.data = skinTriIndices;
                skinMesh.flags = PxMeshFlags(0u); // disable both flags
                PxTriangleMeshAnalysisResults skinMeshAnalysis = PxTetMaker::validateTriangleMesh(skinMesh);
                if (skinMeshAnalysis & PxTriangleMeshAnalysisResult::eZERO_VOLUME)
                {
                    CARB_LOG_ERROR("Mesh is flat, creating tetrahedral meshes for deformable simulation failed: %s",
                        CookingTask::getPrimPathText().c_str());
                    CookingTask::setSucceeded(false);
                    CookingTask::setFinished(true);
                    return;
                }
                else if (skinMeshAnalysis & PxTriangleMeshAnalysisResult::eMESH_IS_INVALID)
                {
                    CARB_LOG_ERROR("Mesh is invalid, creating tetrahedral meshes for deformable simulation failed: %s",
                        CookingTask::getPrimPathText().c_str());
                    CookingTask::setSucceeded(false);
                    CookingTask::setFinished(true);
                    return;
                }
            }
            bool cookingSucceeded = false;
            bool useConformingTetmesher = true;
            if (m_simpEnabled)
            {
                carb::Float3 min, max;
                boundingBox(skinRestPoints, skinRestPointsSize, min, max);
                carb::Float3 boundsDim{ max.x - min.x, max.y - min.y, max.z - min.z };
                float boundsDimMax = fmaxf(boundsDim.x, fmaxf(boundsDim.y, boundsDim.z));
                float simMeshVoxelEdgeLength = boundsDimMax / m_simulationResolution;

                uint32_t targetTriangleCount = m_simpTargetTriangleCount;
                if (targetTriangleCount == 0)
                {
                    //targetTriangleCount = (int)(sqrtf(averageTriangleEdgeLength(skinRestPoints, skinTriIndices, skinTriIndicesSize) / simMeshVoxelEdgeLength)*skinTriIndicesSize / 3) + 1;
                    //targetTriangleCount = (int)((averageTriangleEdgeLength(skinRestPoints, skinTriIndices, skinTriIndicesSize) / simMeshVoxelEdgeLength)*skinTriIndicesSize / 3) + 1;
                    targetTriangleCount = (int)(5.0f * surfaceArea(skinRestPoints, skinTriIndices, skinTriIndicesSize) / (simMeshVoxelEdgeLength * simMeshVoxelEdgeLength)) + 1;
                }

                if (m_simpRemeshing)
                {
                    uint32_t remeshingResolution = m_simpRemeshingResolution;
                    if (remeshingResolution == 0)
                    {
                        remeshingResolution = (uint32_t)(boundsDimMax / averageTriangleEdgeLength(skinRestPoints, skinTriIndices, skinTriIndicesSize)) + 1;
                        if (remeshingResolution < 100) remeshingResolution = 100;
                        if (remeshingResolution > 250) remeshingResolution = 250;
                    }

                    ResultBuffer<carb::Float3> remeshedTriPoints;
                    ResultBuffer<uint32_t> remeshedTriIndices;

                    //with remeshing enabled, we skip generating skinToCollisionTriVertsMap, cause a reliable mapping is currently impossible
                    remeshTriangleMeshInternal(
                        remeshedTriPoints.ptr, remeshedTriPoints.size, remeshedTriIndices.ptr, remeshedTriIndices.size, nullptr, nullptr,
                        skinRestPoints, skinRestPointsSize, skinTriIndices, skinTriIndicesSize, remeshingResolution,
                        ResultBuffer<>::allocate);

                    simplifyTriangleMeshInternal(
                        collisionTriPoints.ptr, collisionTriPoints.size, collisionTriIndices.ptr, collisionTriIndices.size,
                        nullptr, nullptr, nullptr, nullptr,
                        remeshedTriPoints.ptr, remeshedTriPoints.size, remeshedTriIndices.ptr, remeshedTriIndices.size,
                        targetTriangleCount, simMeshVoxelEdgeLength, false, false,
                        ResultBuffer<>::allocate);
                }
                else
                {
                    bool projectSimplifiedVerticesOntoInputMesh = kinematicEnabled;
                    bool removeDisconnectedPatches = kinematicEnabled;
                    uint32_t** skinToCollision = kinematicEnabled ? nullptr : &skinToCollisionTriVertsMap.ptr;
                    uint32_t* skinToCollisionSize = kinematicEnabled ? nullptr : &skinToCollisionTriVertsMap.size;
                    uint32_t** collisionToSkin = kinematicEnabled ? &collisionVertsToSkinTriIds.ptr : nullptr;
                    uint32_t* collisionToSkinSize = kinematicEnabled ? &collisionVertsToSkinTriIds.size : nullptr;

                    simplifyTriangleMeshInternal(
                        collisionTriPoints.ptr, collisionTriPoints.size, collisionTriIndices.ptr, collisionTriIndices.size,
                        skinToCollision, skinToCollisionSize, collisionToSkin, collisionToSkinSize,
                        skinRestPoints, skinRestPointsSize, skinTriIndices, skinTriIndicesSize,
                        targetTriangleCount, simMeshVoxelEdgeLength, projectSimplifiedVerticesOntoInputMesh, removeDisconnectedPatches,
                        ResultBuffer<>::allocate);
                }

                useConformingTetmesher = m_simpForceConforming || kinematicEnabled;
            }
            else
            {
                collisionTriPoints.ptr = skinRestPoints;
                collisionTriPoints.size = skinRestPointsSize;
                collisionTriIndices.ptr = skinTriIndices;
                collisionTriIndices.size = skinTriIndicesSize;
            }

            if (useConformingTetmesher)
            {
                cookingSucceeded = computeConformingTetrahedralMeshInternal(
                    m_collisionRestPoints.ptr, m_collisionRestPoints.size, m_collisionIndices.ptr, m_collisionIndices.size,
                    collisionTriPoints.ptr, collisionTriPoints.size, collisionTriIndices.ptr, collisionTriIndices.size,
                    ResultBuffer<>::allocate);
            }
            else
            {
                computeTreeBasedTetrahedralMeshInternal(
                    m_collisionRestPoints.ptr, m_collisionRestPoints.size, m_collisionIndices.ptr, m_collisionIndices.size,
                    collisionTriPoints.ptr, collisionTriPoints.size, collisionTriIndices.ptr, collisionTriIndices.size, true,
                    ResultBuffer<>::allocate);
                cookingSucceeded = true;
            }

            if (cookingSucceeded)
            {
                if (kinematicEnabled)
                {
                    //create embedding in default pose
                    m_collisionVertexToSimulationTetIndices.ptr = reinterpret_cast<int32_t*>(ResultBuffer<>::allocate(m_collisionRestPoints.size*sizeof(int32_t)));
                    m_collisionVertexToSimulationTetIndices.size = m_collisionRestPoints.size;
                    createPointsToTetrahedraMapInternal(
                        reinterpret_cast<uint32_t*>(m_collisionVertexToSimulationTetIndices.ptr), nullptr,
                        m_collisionRestPoints.ptr, m_collisionRestPoints.size, m_collisionRestPoints.ptr, m_collisionRestPoints.size, m_collisionIndices.ptr, m_collisionIndices.size);

                    bool needsCollisionToSkinMapping = collisionVertsToSkinTriIds.size > 0;
                    if (needsCollisionToSkinMapping)
                    {
                        //compute barycentric coordinates to embed the collision mesh surface in the skin surface
                        m_collisionVertexToSkinTriBarycentrics.ptr = reinterpret_cast<carb::Float3*>(ResultBuffer<>::allocate(collisionVertsToSkinTriIds.size * sizeof(carb::Float3)));
                        m_collisionVertexToSkinTriBarycentrics.size = collisionVertsToSkinTriIds.size;
                        for (uint32_t t = 0; t < collisionVertsToSkinTriIds.size; ++t)
                        {
                            uint32_t skinTriId = collisionVertsToSkinTriIds.ptr[t];
                            if (skinTriId*3 < skinTriIndicesSize)
                            {
                                const uint32_t* tri = skinTriIndices + skinTriId*3;
                                if (tri[0] < skinRestPointsSize && tri[1] < skinRestPointsSize && tri[2] < skinRestPointsSize)
                                {
                                    PxVec3 v0 = toPhysX(skinRestPoints[tri[0]]);
                                    PxVec3 v1 = toPhysX(skinRestPoints[tri[1]]);
                                    PxVec3 v2 = toPhysX(skinRestPoints[tri[2]]);
                                    PxVec3 p = toPhysX(m_collisionRestPoints.ptr[t]);
                                    PxVec3 e0 = v1 - v0;
                                    PxVec3 e1 = v2 - v0;

                                    //use robust distancePointTriangleSquared instead of vanilla barycentric coord computation
                                    //as it handles very small or degenerate triangles
                                    PxVec3 bary;
                                    distancePointTriangleSquared(bary, p, v0, v1, v2, e0, e1);
                                    m_collisionVertexToSkinTriBarycentrics.ptr[t] = fromPhysX(bary);
                                }
                            }
                        }
                        //copy the necessary surface triangles and remap triangle IDs
                        m_collisionVertexToSkinTriVertexIndices.ptr = reinterpret_cast<uint32_t*>(ResultBuffer<>::allocate(collisionVertsToSkinTriIds.size*3*sizeof(uint32_t)));
                        m_collisionVertexToSkinTriVertexIndices.size = collisionVertsToSkinTriIds.size*3;
                        for (uint32_t t = 0; t < collisionVertsToSkinTriIds.size; ++t)
                        {
                            uint32_t skinTriId = collisionVertsToSkinTriIds.ptr[t];
                            if (skinTriId*3+2 < skinTriIndicesSize)
                            {
                                const uint32_t* tri = skinTriIndices + skinTriId*3;
                                m_collisionVertexToSkinTriVertexIndices.ptr[t*3 + 0] = tri[0];
                                m_collisionVertexToSkinTriVertexIndices.ptr[t*3 + 1] = tri[1];
                                m_collisionVertexToSkinTriVertexIndices.ptr[t*3 + 2] = tri[2];
                            }
                        }
                    }

                    //relax to first animation frame
                    if (m_skinRestPoints.size() == m_skinEarliestPoints.size())
                    {
                        std::vector<carb::Float3> kinematicTargets;
                        if (needsCollisionToSkinMapping)
                        {
                            carb::Uint3* skinTriangles = reinterpret_cast<carb::Uint3*>(m_collisionVertexToSkinTriVertexIndices.ptr);
                            kinematicTargets.resize(m_collisionVertexToSkinTriBarycentrics.size);
                            for (size_t v = 0; v < kinematicTargets.size(); ++v)
                            {
                                kinematicTargets[v] = evaluateBarycentric(skinTriangles[v], m_collisionVertexToSkinTriBarycentrics.ptr[v], &m_skinEarliestPoints[0].x);
                            }
                        }
                        else
                        {
                            kinematicTargets.assign(m_skinEarliestPoints.begin(), m_skinEarliestPoints.end());
                        }

                        transformTetrahedralMeshPoseInternal(m_collisionRestPoints.ptr, m_collisionRestPoints.ptr, m_collisionRestPoints.size,
                            kinematicTargets.data(), uint32_t(kinematicTargets.size()), m_collisionIndices.ptr, m_collisionIndices.size);
                    }

                    m_simulationRestPoints.ptr = m_collisionRestPoints.ptr;
                    m_simulationRestPoints.size = m_collisionRestPoints.size;
                    m_simulationIndices.ptr = m_collisionIndices.ptr;
                    m_simulationIndices.size = m_collisionIndices.size;
                    cookingSucceeded = true;
                }
                else
                {
                    //Append skin mesh vertices after collision tetmesh vertices to get their embedding computed automatically inside the voxel tetmesher.
                    std::vector<carb::Float3> points;
                    points.reserve(m_collisionRestPoints.size + m_skinRestPoints.size());

                    std::vector<uint32_t> vertexMapExtended;
                    if (skinToCollisionTriVertsMap.size > 0)
                    {
                        vertexMapExtended.reserve(m_collisionRestPoints.size + m_skinRestPoints.size());
                        for (uint32_t i = 0; i < m_collisionRestPoints.size; ++i)
                            vertexMapExtended.push_back(i);

                        CARB_ASSERT(m_skinRestPoints.size() == skinToCollisionTriVertsMap.size);
                        //vertexMap points to skin mesh vertices. The tetmesh's vertices created by
                        // computeConformingTetrahedralMeshInternal and computeTreeBasedTetrahedralMeshInternal
                        // are guaranteed to share first N vertices with input triangle mesh
                        for (uint32_t i = 0; i < skinToCollisionTriVertsMap.size; ++i)
                            vertexMapExtended.push_back(skinToCollisionTriVertsMap.ptr[i]); 
                    }

                    for (uint32_t i = 0; i < m_collisionRestPoints.size; ++i)
                        points.push_back(m_collisionRestPoints.ptr[i]);

                    for (uint32_t i = 0; i < m_skinRestPoints.size(); ++i)
                        points.push_back(m_skinRestPoints[i]);

                    cookingSucceeded = computeVoxelTetrahedralMeshInternal(
                        m_simulationRestPoints.ptr, m_simulationRestPoints.size, m_simulationIndices.ptr, m_simulationIndices.size,
                        m_collisionVertexToSimulationTetIndices.ptr, m_collisionVertexToSimulationTetIndices.size,
                        points.data(), uint32_t(points.size()), m_collisionIndices.ptr, uint32_t(m_collisionIndices.size),
                        m_simulationResolution, m_simulationNumTetsPerVoxel, vertexMapExtended.data(), ResultBuffer<>::allocate);
                }
            }

            if (collisionTriPoints.ptr == skinRestPoints)
            {
                // in this case collisionTriPoints and collisionTriIndices are just references
                // to the skin data - this prevents auto deallocation
                collisionTriPoints.ptr = nullptr;
                collisionTriPoints.size = 0;
                collisionTriIndices.ptr = nullptr;
                collisionTriIndices.size = 0;
            }

            if (!cookingSucceeded)
            {
                CARB_LOG_ERROR("Creating tetrahedral meshes for deformable simulation failed: %s",
                    CookingTask::getPrimPathText().c_str());
            }
            CookingTask::setSucceeded(cookingSucceeded);
        }
        CookingTask::setFinished(true);
    }

    // Once the task is complete, we now need to store the results into USD.
    // This must all be done from the main thread since USD is not thread safe
    virtual void finalize(void) final
    {
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

            carb::Float3 scaleInv = { 1.0f / m_normalizedQuantizedScale.x, 1.0f / m_normalizedQuantizedScale.y, 1.0f / m_normalizedQuantizedScale.z };
            for (uint32_t p = 0; p < m_collisionRestPoints.size; ++p)
            {
                carb::Float3& v = m_collisionRestPoints.ptr[p];
                v = carb::Float3{ v.x * scaleInv.x, v.y * scaleInv.y, v.z * scaleInv.z };
            }
            if (m_simulationRestPoints.ptr != m_collisionRestPoints.ptr)
            {
                for (uint32_t p = 0; p < m_simulationRestPoints.size; ++p)
                {
                    carb::Float3& v = m_simulationRestPoints.ptr[p];
                    v = carb::Float3{ v.x * scaleInv.x, v.y * scaleInv.y, v.z * scaleInv.z };
                }
            }

            // Write output to stream, for both cache and direct consumption of the result
            if (CookingTask::getCookedDataOutputStreams().empty())
            {
                CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
            }
            PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();
            writeDeformableBodyTetMeshDataDeprecated(outputStream,
                m_collisionRestPoints.ptr, m_collisionRestPoints.size, m_collisionIndices.ptr, m_collisionIndices.size,
                m_simulationRestPoints.ptr, m_simulationRestPoints.size, m_simulationIndices.ptr, m_simulationIndices.size,
                m_collisionVertexToSimulationTetIndices.ptr, m_collisionVertexToSimulationTetIndices.size,
                m_collisionVertexToSkinTriVertexIndices.ptr, m_collisionVertexToSkinTriBarycentrics.ptr,
                m_collisionVertexToSkinTriBarycentrics.size);

            omni::physx::PhysxCookingComputeResult& result = CookingTask::getResultObject();
            PxDefaultMemoryOutputStream& cookDataStream = outputStream;
            PhysxCookedDataSpan cookedDataSpan;
            result.cookedData = &cookedDataSpan;
            result.cookedDataNumElements = 1;
            cookedDataSpan.data = cookDataStream.getData();
            cookedDataSpan.sizeInBytes = cookDataStream.getSize();

            CookingTask::fireFinishedCallback(PhysxCookingResult::eVALID);
        }

        //TODO add ownership to ResultsBuffer
        if (m_simulationRestPoints.ptr == m_collisionRestPoints.ptr)
        {
            m_simulationRestPoints.ptr = nullptr;
            m_simulationRestPoints.size = 0;
        }
        if (m_simulationIndices.ptr == m_collisionIndices.ptr)
        {
            m_simulationIndices.ptr = nullptr;
            m_simulationIndices.size = 0;
        }
    }

    //in
    bool m_simpEnabled;
    bool m_simpRemeshing;
    uint32_t m_simpRemeshingResolution;
    uint32_t m_simpTargetTriangleCount;
    bool m_simpForceConforming;
    carb::Float3 m_normalizedQuantizedScale;
    uint32_t m_simulationResolution;
    uint32_t m_simulationNumTetsPerVoxel;
    bool m_kinematicEnabled;
    std::vector<carb::Float3> m_skinRestPoints;
    std::vector<carb::Float3> m_skinEarliestPoints;

    //out
    ResultBuffer<carb::Float3> m_collisionRestPoints;
    ResultBuffer<uint32_t> m_collisionIndices;
    ResultBuffer<carb::Float3> m_simulationRestPoints;
    ResultBuffer<uint32_t> m_simulationIndices;
    ResultBuffer<int32_t> m_collisionVertexToSimulationTetIndices;
    ResultBuffer<uint32_t> m_collisionVertexToSkinTriVertexIndices;
    ResultBuffer<carb::Float3> m_collisionVertexToSkinTriBarycentrics;
};

CookingTask* createDeformableBodyTetMeshCookingTaskDeprecated(
    const omni::physx::DeformableBodyTetMeshCookingParamsDeprecated& params,
    omni::physx::PhysxCookingComputeResult& result)
{
    return new DeformableBodyTetMeshCookingTaskDeprecated(params,
                                                          result);
}

}
