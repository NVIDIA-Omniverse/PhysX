// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <physxSchema/physxCookedDataAPI.h>
#include <PxPhysicsAPI.h>
#include <common/foundation/Allocator.h>

#include "../service/CookingTask.h"
#include "../service/CookingComputeService.h"

#include "../utility/MeshSimplifyInternal.h"

#include <common/foundation/Algorithms.h>
#include <common/utilities/PrimUtilities.h>
#include "extensions/PxTetMakerExt.h"

using namespace ::physx;
using namespace omni::physx;

namespace
{
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
        if (d1 <= 0.0f && d2 <= 0.0f)
        {
            bary = PxVec3(1.0f, 0.0f, 0.0f);
            return ap.dot(ap);	// Barycentric coords 1,0,0
        }

        // Check if P in vertex region outside B
        const PxVec3 bp = p - b;
        const float d3 = ab.dot(bp);
        const float d4 = ac.dot(bp);
        if (d3 >= 0.0f && d4 <= d3)
        {
            bary = PxVec3(0.0f, 1.0f, 0.0f);
            return bp.dot(bp); // Barycentric coords 0,1,0
        }

        // Check if P in edge region of AB, if so return projection of P onto AB
        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
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
        if (d6 >= 0.0f && d5 <= d6)
        {
            bary = PxVec3(0.0f, 0.0f, 1.0f);
            return cp.dot(cp); // Barycentric coords 0,0,1
        }

        // Check if P in edge region of AC, if so return projection of P onto AC
        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        {
            const float w = d2 / (d2 - d6);
            PxVec3 diff = p - (a + w * ac);
            bary = PxVec3(1.0f - w, 0.0f, w);
            return diff.dot(diff); // barycentric coords (1-w, 0, w)
        }

        // Check if P in edge region of BC, if so return projection of P onto BC
        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        {
            const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            PxVec3 diff = p - (b + w * (c - b));
            bary = PxVec3(0.0f, 1.0f - w, w);
            return diff.dot(diff); // barycentric coords (0, 1-w, w)
        }

        // P inside face region. Compute Q through its barycentric coords (u,v,w)
        const float denom = 1.0f / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        PxVec3 diff = p - (a + ab * v + ac * w);
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
        static void* allocate(size_t numBytes)
        {
            return ICE_ALLOC(numBytes);
        }

        ResultBuffer() : mPtr(nullptr), mSize(0), mOwnsMemory(true) {}
        ResultBuffer(ResultBuffer<T>& other)
        {
            mPtr = other.mPtr;
            mSize = other.mSize;
            mOwnsMemory = false;
        }

        ~ResultBuffer()
        {
            release();
        }

        void referenceOther(T* otherPtr, uint32_t otherSize)
        {
            release();
            mPtr = otherPtr;
            mSize = otherSize;
            mOwnsMemory = false;
        }

        void referenceOther(ResultBuffer<T>& other)
        {
            referenceOther(other.mPtr, other.mSize);
        }

        void resize(uint32_t size)
        {
            release();
            mPtr = reinterpret_cast<T*>(allocate(size * sizeof(T)));
            mSize = size;
        }

        void release()
        {
            if (mOwnsMemory && mPtr != nullptr)
            {
                ICE_FREE(mPtr);
            }
            mPtr = nullptr;
            mSize = 0;
        }

        inline T*& ptr_ref() { return mPtr; }
        inline uint32_t& size_ref() { return mSize; }

        inline T* ptr() { return mPtr; }
        inline const uint32_t size() { return mSize; }

    private:
        T* mPtr;
        uint32_t mSize;
        bool mOwnsMemory;
    };
}

namespace cookingtask
{

static void writeVolumeDeformableBodyData(PxDefaultMemoryOutputStream& outData,
    const carb::Float3* simPoints, const uint32_t simPointsSize,
    const uint32_t* simIndices, const uint32_t simIndicesSize,
    const carb::Float3* collPoints, const uint32_t collPointsSize,
    const uint32_t* collIndices, const uint32_t collIndicesSize,
    const uint32_t* collSurfaceIndices, const uint32_t collSurfaceIndicesSize
)
{
    outData.write(&simPointsSize, sizeof(uint32_t));
    outData.write(simPoints, sizeof(carb::Float3) * simPointsSize);

    outData.write(&simIndicesSize, sizeof(uint32_t));
    outData.write(simIndices, sizeof(uint32_t) * simIndicesSize);

    outData.write(&collPointsSize, sizeof(uint32_t));
    if (collPointsSize > 0)
    {
        outData.write(collPoints, sizeof(carb::Float3) * collPointsSize);
    }

    outData.write(&collIndicesSize, sizeof(uint32_t));
    if (collIndicesSize > 0)
    {
        outData.write(collIndices, sizeof(uint32_t) * collIndicesSize);
    }

    // writing collision mesh surface indices even if no separate collision mesh.
    // it might just refer to the simulation mesh surface, doubling as collision mesh.
    outData.write(&collSurfaceIndicesSize, sizeof(uint32_t));
    outData.write(collSurfaceIndices, sizeof(uint32_t) * collSurfaceIndicesSize);
}

void readVolumeDeformableBodyData(PhysxCookingVolumeDeformableBodyData& out, const PhysxCookedDataSpan& cookedData)
{
    const uint8_t* dataPtr = reinterpret_cast<const uint8_t*>(cookedData.data);

    out.simPointsSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.simPoints = reinterpret_cast<const carb::Float3*>(dataPtr);
    dataPtr += out.simPointsSize * sizeof(carb::Float3);

    out.simIndicesSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.simIndices = reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += out.simIndicesSize * sizeof(uint32_t);

    out.collPointsSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.collPoints = nullptr;
    if (out.collPointsSize > 0)
    {
        out.collPoints = reinterpret_cast<const carb::Float3*>(dataPtr);
        dataPtr += out.collPointsSize * sizeof(carb::Float3);
    }

    out.collIndicesSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.collIndices = nullptr;
    if (out.collIndicesSize > 0)
    {
        out.collIndices = reinterpret_cast<const uint32_t*>(dataPtr);
        dataPtr += out.collIndicesSize * sizeof(uint32_t);
    }

    out.collSurfaceIndicesSize = *reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += sizeof(uint32_t);

    out.collSurfaceIndices = reinterpret_cast<const uint32_t*>(dataPtr);
    dataPtr += out.collSurfaceIndicesSize * sizeof(uint32_t);

    CARB_ASSERT(dataPtr == reinterpret_cast<const uint8_t*>(cookedData.data) + cookedData.sizeInBytes);
}

void removeOrphanedVertices(carb::Float3* points, uint32_t& pointsSize, uint32_t* tetVtxIndices, const uint32_t tetVtxIndicesSize)
{
    // count num tets per vtx
    std::vector<bool> isVtxOrphaned(pointsSize, true);
    for (uint32_t i = 0; i < tetVtxIndicesSize; ++i)
    {
        uint32_t vtxIndex = tetVtxIndices[i];
        if (vtxIndex < pointsSize)
        {
            isVtxOrphaned[vtxIndex] = false;
        }
    }

    // remove unreferenced points, tetsPerVtxCounts is consumed
    std::vector<uint32_t> vtxMovedTo(pointsSize);
    uint32_t r = 0; // read head
    uint32_t w = 0; // write head
    uint32_t skipped = 0;
    while (r < pointsSize)
    {
        // invariance: r always get's increased

        // advance write head until vertices are orphaned
        while (w < pointsSize && !isVtxOrphaned[w])
        {
            vtxMovedTo[w] = w;
            ++w;
        }

        // advance write head to read position if it's trailing
        if (r < w)
        {
            r = w;
        }

        // advance read head until vertices are not orphaned
        while (r < pointsSize && isVtxOrphaned[r])
        {
            ++skipped;
            ++r;
        }

        // advance both heads and copy
        while (r < pointsSize && !isVtxOrphaned[r] != 0 && isVtxOrphaned[w])
        {
            points[w] = points[r];
            //points[r] = { -1, -1, -1 }; //debug
            isVtxOrphaned[r] = true;
            vtxMovedTo[r] = w;
            ++r;
            ++w;
        }
    }
    pointsSize -= skipped;

    for (uint32_t i = 0; i < tetVtxIndicesSize; ++i)
    {
        uint32_t tetVtxIdx = tetVtxIndices[i];
        tetVtxIndices[i] = vtxMovedTo[tetVtxIdx];
    }
}

// This class handles cooking a single deformable body simulation mesh in a background task.
class VolumeDeformableBodyCookingTask : public cookingtask::CookingTask
{
public:
    VolumeDeformableBodyCookingTask(
        const VolumeDeformableBodyCookingParams& params,
        omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
    {
        static_assert(sizeof(m_simToCookingTransform) == sizeof(params.simToCookingTransform));
        m_simToCookingTransform = *reinterpret_cast<const pxr::GfMatrix4d*>(params.simToCookingTransform);

        static_assert(sizeof(m_simToCollTransform) == sizeof(params.simToCollTransform));
        m_simToCollTransform = *reinterpret_cast<const pxr::GfMatrix4d*>(params.simToCollTransform);

        m_srcPoints.resize(params.srcPointsInSim.size());
        for (uint32_t p = 0; p < m_srcPoints.size(); ++p)
        {
            const pxr::GfVec3f& src = *reinterpret_cast<const pxr::GfVec3f*>(&params.srcPointsInSim[p]);
            pxr::GfVec3f dst = pxr::GfVec3f(m_simToCookingTransform.Transform(src));
            m_srcPoints[p] = { dst[0], dst[1], dst[2] };
        }

        m_isAutoMeshSimplificationEnabled = params.isAutoMeshSimplificationEnabled;
        m_isAutoRemeshingEnabled = params.isAutoRemeshingEnabled;
        m_hasAutoForceConforming = params.hasAutoForceConforming;
        m_isAutoHexahedralMeshEnabled = params.isAutoHexahedralMeshEnabled;
        m_autoRemeshingResolution = params.autoRemeshingResolution;
        m_autoTriangleTargetCount = params.autoTriangleTargetCount;
        m_autoHexahedralResolution = params.autoHexahedralResolution;
    }

    virtual ~VolumeDeformableBodyCookingTask(void)
    {
        // Wait until thread is finished
        CookingTask::futureWait();
        // Finalize the results; store them in the USD prim and mesh caches
        if (!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    static void applyMeshSimplification(
        ResultBuffer<carb::Float3>& simpTriPoints,
        ResultBuffer<uint32_t>& simpTriIndices,
        ResultBuffer<uint32_t>& skinToSimpTriVertsMap,
        const carb::Float3* skinPoints, const uint32_t skinPointsSize,
        const uint32_t* skinTriIndices, const uint32_t skinTriIndicesSize,
        const uint32_t _targetResolution, const uint32_t _targetTriangleCount,
        const bool isRemeshingEnabled, const uint32_t _remeshingResolution)
    {
        float averageTriEdgeLength = averageTriangleEdgeLength(skinPoints, skinTriIndices, skinTriIndicesSize);
        carb::Float3 min, max;
        boundingBox(skinPoints, skinPointsSize, min, max);
        carb::Float3 boundsDim{ max.x - min.x, max.y - min.y, max.z - min.z };
        float boundsDimMax = fmaxf(boundsDim.x, fmaxf(boundsDim.y, boundsDim.z));

        uint32_t targetResolution = _targetResolution;
        if (targetResolution == 0)
        {
            targetResolution = uint32_t(std::ceil(boundsDimMax / averageTriEdgeLength));
        }
        float targetEdgeLength = boundsDimMax / targetResolution;

        uint32_t targetTriangleCount = _targetTriangleCount;
        if (targetTriangleCount == 0)
        {
            //targetTriangleCount = (int)(sqrtf(averageTriEdgeLength / targetEdgeLength)*skinTriIndicesSize / 3) + 1;
            //targetTriangleCount = (int)((averageTriEdgeLength / targetEdgeLength)*skinTriIndicesSize / 3) + 1;
            targetTriangleCount = (int)(5.0f * surfaceArea(skinPoints, skinTriIndices, skinTriIndicesSize) / (targetEdgeLength * targetEdgeLength)) + 1;
        }

        if (isRemeshingEnabled)
        {
            uint32_t remeshingResolution = _remeshingResolution;
            if (remeshingResolution == 0)
            {
                remeshingResolution = targetResolution;
                if (remeshingResolution < 100) remeshingResolution = 100;
                if (remeshingResolution > 250) remeshingResolution = 250;
            }

            ResultBuffer<carb::Float3> remeshedTriPoints;
            ResultBuffer<uint32_t> remeshedTriIndices;

            //with remeshing enabled, we skip generating skinToCollisionTriVertsMap, cause a reliable mapping is currently impossible
            remeshTriangleMeshInternal(
                remeshedTriPoints.ptr_ref(), remeshedTriPoints.size_ref(), remeshedTriIndices.ptr_ref(), remeshedTriIndices.size_ref(),
                nullptr, nullptr, skinPoints, skinPointsSize, skinTriIndices, skinTriIndicesSize, remeshingResolution,
                ResultBuffer<>::allocate);

            simplifyTriangleMeshInternal(
                simpTriPoints.ptr_ref(), simpTriPoints.size_ref(), simpTriIndices.ptr_ref(), simpTriIndices.size_ref(),
                nullptr, nullptr, nullptr, nullptr,
                remeshedTriPoints.ptr(), remeshedTriPoints.size(), remeshedTriIndices.ptr(), remeshedTriIndices.size(),
                targetTriangleCount, targetEdgeLength, false, false,
                ResultBuffer<>::allocate);
        }
        else
        {
            uint32_t** skinToSimp = &skinToSimpTriVertsMap.ptr_ref();
            uint32_t* skinToSimpSize = &skinToSimpTriVertsMap.size_ref();
            uint32_t** simpToSkin = nullptr;
            uint32_t* simpToSkinSize = nullptr;
            const bool removeDisconnectedPatches = false;

            simplifyTriangleMeshInternal(
                simpTriPoints.ptr_ref(), simpTriPoints.size_ref(), simpTriIndices.ptr_ref(), simpTriIndices.size_ref(),
                skinToSimp, skinToSimpSize, simpToSkin, simpToSkinSize,
                skinPoints, skinPointsSize, skinTriIndices, skinTriIndicesSize,
                targetTriangleCount, targetEdgeLength, false, removeDisconnectedPatches,
                ResultBuffer<>::allocate);
        }
    }

    // This operation happens in a background thread and, therefore, must be
    // completely thread safe.
    virtual void performTask(void) final
    {
        if (!CookingTask::isCanceled())
        {
            std::vector<uint32_t> srcTriIndices;
            {
                CookingTask::performTriangulation();
                uint32_t skinTriCount;
                const uint32_t* indices = CookingTask::getIndices(skinTriCount);
                srcTriIndices.insert(srcTriIndices.begin(), indices, indices + skinTriCount * 3);
            }
            uint32_t* skinTriIndices = srcTriIndices.data();
            uint32_t skinTriIndicesSize = uint32_t(srcTriIndices.size());
            carb::Float3* skinPoints = m_srcPoints.data();
            uint32_t skinPointsSize = uint32_t(m_srcPoints.size());

            // check for planar meshes
            {
                PxSimpleTriangleMesh skinMesh;
                skinMesh.points.count = skinPointsSize;
                skinMesh.points.data = skinPoints;
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

            // resolution parameter
            uint32_t resolution = 0;
            if (m_isAutoHexahedralMeshEnabled)
            {
                resolution = (m_autoHexahedralResolution > 0) ? std::max(m_autoHexahedralResolution, 4u) : 10;
            }

            ResultBuffer<carb::Float3> collisionTriPoints;
            ResultBuffer<uint32_t> collisionTriIndices;
            ResultBuffer<uint32_t> skinToCollisionTriVertsMap;
            bool useConformingTetmesher = true;
            if (m_isAutoMeshSimplificationEnabled)
            {
                applyMeshSimplification(collisionTriPoints, collisionTriIndices, skinToCollisionTriVertsMap,
                    skinPoints, skinPointsSize, skinTriIndices, skinTriIndicesSize,
                    resolution, m_autoTriangleTargetCount, m_isAutoRemeshingEnabled,
                    m_autoRemeshingResolution);

                useConformingTetmesher = m_hasAutoForceConforming;
            }
            else
            {
                collisionTriPoints.referenceOther(skinPoints, skinPointsSize);
                collisionTriIndices.referenceOther(skinTriIndices, skinTriIndicesSize);
            }

            bool cookingSucceeded = false;
            if (useConformingTetmesher)
            {
                cookingSucceeded = computeConformingTetrahedralMeshInternal(
                    m_collPoints.ptr_ref(), m_collPoints.size_ref(), m_collIndices.ptr_ref(), m_collIndices.size_ref(),
                    collisionTriPoints.ptr(), collisionTriPoints.size(), collisionTriIndices.ptr(), collisionTriIndices.size(),
                    ResultBuffer<>::allocate);
            }
            else
            {
                cookingSucceeded = computeTreeBasedTetrahedralMeshInternal(
                    m_collPoints.ptr_ref(), m_collPoints.size_ref(), m_collIndices.ptr_ref(), m_collIndices.size_ref(),
                    collisionTriPoints.ptr(), collisionTriPoints.size(), collisionTriIndices.ptr(), collisionTriIndices.size(), true,
                    ResultBuffer<>::allocate);
            }

            //remove vertices that are not referenced by any tets
            removeOrphanedVertices(m_collPoints.ptr(), m_collPoints.size_ref(), m_collIndices.ptr(), m_collIndices.size());

            if (cookingSucceeded)
            {
                if (m_isAutoHexahedralMeshEnabled)
                {
                    // Append skin mesh vertices after collision tetmesh vertices to get their embedding
                    // computed automatically inside the voxel tetmesher.
                    std::vector<carb::Float3> points;
                    points.reserve(m_collPoints.size() + m_srcPoints.size());

                    std::vector<uint32_t> vertexMapExtended;
                    if (skinToCollisionTriVertsMap.size() > 0)
                    {
                        vertexMapExtended.reserve(m_collPoints.size() + m_srcPoints.size());
                        for (uint32_t i = 0; i < m_collPoints.size(); ++i)
                            vertexMapExtended.push_back(i);

                        CARB_ASSERT(m_srcPoints.size() == skinToCollisionTriVertsMap.size());
                        // vertexMap points to skin mesh vertices. The tetmesh's vertices created by
                        // createConformingTetrahedralMesh and createTreeBasedTetmesh are guaranteed to share
                        // first N vertices with input triangle mesh
                        for (uint32_t i = 0; i < skinToCollisionTriVertsMap.size(); ++i)
                            vertexMapExtended.push_back(skinToCollisionTriVertsMap.ptr()[i]);
                    }

                    for (uint32_t i = 0; i < m_collPoints.size(); ++i)
                        points.push_back(m_collPoints.ptr()[i]);

                    for (uint32_t i = 0; i < m_srcPoints.size(); ++i)
                        points.push_back(m_srcPoints[i]);

                    // not writing collision to simulation mapping to output, so declaring this temporarily.
                    ResultBuffer<int32_t> collVertexToSimTetIndices;
                    const uint32_t simulationNumTetsPerVoxel = 6;
                    cookingSucceeded = computeVoxelTetrahedralMeshInternal(
                        m_simPoints.ptr_ref(), m_simPoints.size_ref(), m_simIndices.ptr_ref(), m_simIndices.size_ref(),
                        collVertexToSimTetIndices.ptr_ref(), collVertexToSimTetIndices.size_ref(),
                        points.data(), uint32_t(points.size()), m_collIndices.ptr(), uint32_t(m_collIndices.size()),
                        resolution, simulationNumTetsPerVoxel, vertexMapExtended.data(), ResultBuffer<>::allocate);
                }
                else
                {
                    m_simPoints.referenceOther(m_collPoints);
                    m_simIndices.referenceOther(m_collIndices);
                }
            }

            // computing surface triangles
            {
                ResultBuffer<uint32_t>& tetIndices = m_isAutoHexahedralMeshEnabled ? m_collIndices : m_simIndices;
                PxArray<PxU32> surfaceTriIndices;
                PxTetrahedronMeshExt::extractTetMeshSurface(tetIndices.ptr(), tetIndices.size() / 4,
                                                            false, surfaceTriIndices, nullptr);
                m_collSurfaceIndices.resize(surfaceTriIndices.size());
                std::memcpy(m_collSurfaceIndices.ptr(), surfaceTriIndices.begin(),
                            m_collSurfaceIndices.size() * sizeof(uint32_t));
            }

            if (!cookingSucceeded)
            {
                CARB_LOG_ERROR("Creating tetrahedral mesh for deformable simulation failed: %s",
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

            // Save the triangulation to the cache in finalize, since saving it out in performTask
            // seemed to exhibit some thread safety issues
            CookingTask::saveTriangulation(getTriangulationOutputStream());

            pxr::GfMatrix4d cookingToSimTransform(m_simToCookingTransform.GetInverse());
            for (uint32_t p = 0; p < m_simPoints.size(); ++p)
            {
                pxr::GfVec3f& v = reinterpret_cast<pxr::GfVec3f&>(m_simPoints.ptr()[p]);
                v = pxr::GfVec3f(cookingToSimTransform.Transform(v));
            }

            const bool simIsColl = (m_collPoints.ptr() == m_simPoints.ptr());
            if (!simIsColl)
            {
                pxr::GfMatrix4d cookingToCollTransform = cookingToSimTransform * m_simToCollTransform;
                for (uint32_t p = 0; p < m_collPoints.size(); ++p)
                {
                    pxr::GfVec3f& v = reinterpret_cast<pxr::GfVec3f&>(m_collPoints.ptr()[p]);
                    v = pxr::GfVec3f(cookingToCollTransform.Transform(v));
                }
            }

            // Write output to stream, for both cache and direct consumption of the result
            if (CookingTask::getCookedDataOutputStreams().empty())
            {
                CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
            }

            ResultBuffer<carb::Float3> emptyPoints;
            ResultBuffer<uint32_t> emptyIndices;
            ResultBuffer<carb::Float3> collPointsOut(simIsColl ? emptyPoints : m_collPoints);
            ResultBuffer<uint32_t> collIndicesOut(simIsColl ? emptyIndices : m_collIndices);

            PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();
            writeVolumeDeformableBodyData(outputStream,
                m_simPoints.ptr(), m_simPoints.size(), m_simIndices.ptr(), m_simIndices.size(),
                collPointsOut.ptr(), collPointsOut.size(), collIndicesOut.ptr(), collIndicesOut.size(),
                m_collSurfaceIndices.ptr(), m_collSurfaceIndices.size()
            );

            omni::physx::PhysxCookingComputeResult& result = CookingTask::getResultObject();
            PxDefaultMemoryOutputStream& cookDataStream = outputStream;
            PhysxCookedDataSpan cookedDataSpan;
            result.cookedData = &cookedDataSpan;
            result.cookedDataNumElements = 1;
            cookedDataSpan.data = cookDataStream.getData();
            cookedDataSpan.sizeInBytes = cookDataStream.getSize();

            CookingTask::fireFinishedCallback(PhysxCookingResult::eVALID);
        }
    }

    //in
    pxr::GfMatrix4d m_simToCookingTransform;
    pxr::GfMatrix4d m_simToCollTransform;
    std::vector<carb::Float3> m_srcPoints;
    bool m_isAutoMeshSimplificationEnabled;
    bool m_isAutoRemeshingEnabled;
    bool m_hasAutoForceConforming;
    bool m_isAutoHexahedralMeshEnabled;
    uint32_t m_autoRemeshingResolution;
    uint32_t m_autoTriangleTargetCount;
    uint32_t m_autoHexahedralResolution;

    //out
    ResultBuffer<carb::Float3> m_simPoints;
    ResultBuffer<uint32_t> m_simIndices;
    ResultBuffer<carb::Float3> m_collPoints;
    ResultBuffer<uint32_t> m_collIndices;
    ResultBuffer<uint32_t> m_collSurfaceIndices;

};

CookingTask* createVolumeDeformableBodyCookingTask(const VolumeDeformableBodyCookingParams& params,
                                                   omni::physx::PhysxCookingComputeResult& result)
{
    return new VolumeDeformableBodyCookingTask(params, result);
}

}
