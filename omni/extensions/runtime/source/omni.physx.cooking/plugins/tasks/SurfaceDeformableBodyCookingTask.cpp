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

using namespace ::physx;
using namespace omni::physx;

namespace
{
    class MeshCleaner
    {
    public:
        MeshCleaner(PxU32 nbVerts, const PxVec3* verts, PxU32 nbTris, const PxU32* indices, PxF32 meshWeldTolerance);
        ~MeshCleaner();

        PxU32	 mNbVerts;
        PxU32	 mNbTris;
        PxVec3* mVerts;
        PxU32* mIndices;
        PxU32* mRemapTriangles;
        PxU32* mRemapVertsToWeld;
        PxU32* mRemapVertsToOrig;
    };

    struct Indices
    {
        PxU32 mRef[3];

        PX_FORCE_INLINE bool operator!=(const Indices& v) const { return mRef[0] != v.mRef[0] || mRef[1] != v.mRef[1] || mRef[2] != v.mRef[2]; }
    };

    static PX_FORCE_INLINE PxU32 getHashValue(const PxVec3& v)
    {
        const PxU32* h = reinterpret_cast<const PxU32*>(&v.x);
        const PxU32 f = (h[0] + h[1] * 11 - (h[2] * 17)) & 0x7fffffff;	// avoid problems with +-0
        return (f >> 22) ^ (f >> 12) ^ (f);
    }

    static PX_FORCE_INLINE PxU32 getHashValue(const Indices& v)
    {
        //	const PxU32* h = v.mRef;
        //	const PxU32 f = (h[0]+h[1]*11-(h[2]*17)) & 0x7fffffff;	// avoid problems with +-0
        //	return (f>>22)^(f>>12)^(f);

        PxU32 a = v.mRef[0];
        PxU32 b = v.mRef[1];
        PxU32 c = v.mRef[2];
        a = a - b;  a = a - c;  a = a ^ (c >> 13);
        b = b - c;  b = b - a;  b = b ^ (a << 8);
        c = c - a;  c = c - b;  c = c ^ (b >> 13);
        a = a - b;  a = a - c;  a = a ^ (c >> 12);
        b = b - c;  b = b - a;  b = b ^ (a << 16);
        c = c - a;  c = c - b;  c = c ^ (b >> 5);
        a = a - b;  a = a - c;  a = a ^ (c >> 3);
        b = b - c;  b = b - a;  b = b ^ (a << 10);
        c = c - a;  c = c - b;  c = c ^ (b >> 15);
        return c;
    }

    MeshCleaner::MeshCleaner(PxU32 nbVerts, const PxVec3* srcVerts, PxU32 nbTris, const PxU32* srcIndices, PxF32 meshWeldTolerance)
    {
        PxVec3* cleanVerts = PX_ALLOCATE(PxVec3, nbVerts, "MeshCleaner");
        PX_ASSERT(cleanVerts);

        PxU32* indices = PX_ALLOCATE(PxU32, (nbTris * 3), "MeshCleaner");

        PxU32* remapTriangles = PX_ALLOCATE(PxU32, nbTris, "MeshCleaner");

        PxU32* vertexIndices = NULL;
        if (meshWeldTolerance != 0.0f)
        {
            vertexIndices = PX_ALLOCATE(PxU32, nbVerts, "MeshCleaner");
            const PxF32 weldTolerance = 1.0f / meshWeldTolerance;
            // snap to grid
            for (PxU32 i = 0; i < nbVerts; i++)
            {
                vertexIndices[i] = i;
                cleanVerts[i] = PxVec3(
                    PxFloor(srcVerts[i].x * weldTolerance + 0.5f),
                    PxFloor(srcVerts[i].y * weldTolerance + 0.5f),
                    PxFloor(srcVerts[i].z * weldTolerance + 0.5f));
            }
        }
        else
        {
            PxMemCopy(cleanVerts, srcVerts, nbVerts * sizeof(PxVec3));
        }

        const PxU32 maxNbElems = PxMax(nbTris, nbVerts);
        const PxU32 hashSize = PxNextPowerOfTwo(maxNbElems);
        const PxU32 hashMask = hashSize - 1;
        PxU32* hashTable = PX_ALLOCATE(PxU32, (hashSize + maxNbElems), "MeshCleaner");
        PX_ASSERT(hashTable);
        PxMemSet(hashTable, 0xff, hashSize * sizeof(PxU32));
        PxU32* const next = hashTable + hashSize;

        mRemapVertsToWeld = PX_ALLOCATE(PxU32, nbVerts, "MeshCleaner");
        PxMemSet(mRemapVertsToWeld, 0xff, nbVerts * sizeof(PxU32));

        for (PxU32 i = 0; i < nbTris * 3; i++)
        {
            const PxU32 vref = srcIndices[i];
            if (vref < nbVerts)
                mRemapVertsToWeld[vref] = 0;
        }

        PxU32 nbCleanedVerts = 0;
        for (PxU32 i = 0; i < nbVerts; i++)
        {
            if (mRemapVertsToWeld[i] == 0xffffffff)
                continue;

            const PxVec3& v = cleanVerts[i];
            const PxU32 hashValue = getHashValue(v) & hashMask;
            PxU32 offset = hashTable[hashValue];

            while (offset != 0xffffffff && cleanVerts[offset] != v)
                offset = next[offset];

            if (offset == 0xffffffff)
            {
                mRemapVertsToWeld[i] = nbCleanedVerts;
                cleanVerts[nbCleanedVerts] = v;
                if (vertexIndices)
                    vertexIndices[nbCleanedVerts] = i;
                next[nbCleanedVerts] = hashTable[hashValue];
                hashTable[hashValue] = nbCleanedVerts++;
            }
            else mRemapVertsToWeld[i] = offset;
        }

        PxU32 nbCleanedTris = 0;
        for (PxU32 i = 0; i < nbTris; i++)
        {
            PxU32 vref0 = *srcIndices++;
            PxU32 vref1 = *srcIndices++;
            PxU32 vref2 = *srcIndices++;
            if (vref0 >= nbVerts || vref1 >= nbVerts || vref2 >= nbVerts)
                continue;

            // PT: you can still get zero-area faces when the 3 vertices are perfectly aligned
            const PxVec3& p0 = srcVerts[vref0];
            const PxVec3& p1 = srcVerts[vref1];
            const PxVec3& p2 = srcVerts[vref2];
            const float area2 = ((p0 - p1).cross(p0 - p2)).magnitudeSquared();
            if (area2 == 0.0f)
                continue;

            PxU32 new_vref0 = mRemapVertsToWeld[vref0];
            PxU32 new_vref1 = mRemapVertsToWeld[vref1];
            PxU32 new_vref2 = mRemapVertsToWeld[vref2];
            if (new_vref0 == new_vref1 || new_vref1 == new_vref2 || new_vref2 == new_vref0)
                continue;

            indices[nbCleanedTris * 3 + 0] = new_vref0;
            indices[nbCleanedTris * 3 + 1] = new_vref1;
            indices[nbCleanedTris * 3 + 2] = new_vref2;
            remapTriangles[nbCleanedTris] = i;
            nbCleanedTris++;
        }

        PxU32 nbToGo = nbCleanedTris;
        nbCleanedTris = 0;
        PxMemSet(hashTable, 0xff, hashSize * sizeof(PxU32));

        Indices* const I = reinterpret_cast<Indices*>(indices);
        bool idtRemap = true;
        for (PxU32 i = 0; i < nbToGo; i++)
        {
            const Indices& v = I[i];
            const PxU32 hashValue = getHashValue(v) & hashMask;
            PxU32 offset = hashTable[hashValue];

            while (offset != 0xffffffff && I[offset] != v)
                offset = next[offset];

            if (offset == 0xffffffff)
            {
                const PxU32 originalIndex = remapTriangles[i];
                PX_ASSERT(nbCleanedTris <= i);
                remapTriangles[nbCleanedTris] = originalIndex;
                if (originalIndex != nbCleanedTris)
                    idtRemap = false;
                I[nbCleanedTris] = v;
                next[nbCleanedTris] = hashTable[hashValue];
                hashTable[hashValue] = nbCleanedTris++;
            }
        }
        PX_FREE(hashTable);

        if (vertexIndices)
        {
            for (PxU32 i = 0; i < nbCleanedVerts; i++)
                cleanVerts[i] = srcVerts[vertexIndices[i]];
            PX_FREE(vertexIndices);
        }
        mNbVerts = nbCleanedVerts;
        mNbTris = nbCleanedTris;
        mVerts = cleanVerts;
        mIndices = indices;
        if (idtRemap)
        {
            PX_FREE(remapTriangles);
            mRemapTriangles = NULL;
        }
        else
        {
            mRemapTriangles = remapTriangles;
        }

        mRemapVertsToOrig = PX_ALLOCATE(PxU32, nbCleanedVerts, "MeshCleaner");
        PxMemSet(mRemapVertsToOrig, 0xff, nbCleanedVerts * sizeof(PxU32));
        std::vector<PxF32> minDistToWeldedPoint(nbCleanedVerts, FLT_MAX);
        for (uint32_t origIndex = 0; origIndex < nbVerts; ++origIndex)
        {
            const uint32_t weldedIndex = mRemapVertsToWeld[origIndex];
            if (weldedIndex == 0xffffffff)
            {
                //vertices not referenced by triangles don't have a valid mRemapVertsToWeld entry
                continue;
            }
            const float dist = (srcVerts[origIndex] - mVerts[weldedIndex]).magnitude();
            if (dist < minDistToWeldedPoint[weldedIndex])
            {
                mRemapVertsToOrig[weldedIndex] = origIndex;
                minDistToWeldedPoint[weldedIndex] = dist;
            }
        }
    }

    MeshCleaner::~MeshCleaner()
    {
        PX_FREE(mRemapTriangles);
        PX_FREE(mIndices);
        PX_FREE(mVerts);
        PX_FREE(mRemapVertsToWeld);
        PX_FREE(mRemapVertsToOrig);
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

        template <typename U = T>
        typename std::enable_if<!std::is_const<U>::value, void>::type
        release()
        {
            if (mOwnsMemory && mPtr != nullptr)
            {
                ICE_FREE(mPtr);
            }
            mPtr = nullptr;
            mSize = 0;
        }

        template <typename U = T>
        typename std::enable_if<std::is_const<U>::value, void>::type
        release()
        {
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

static void writeSurfaceDeformableBodyData(PxDefaultMemoryOutputStream& outData,
    const carb::Float3* simPoints, const uint32_t simPointsSize,
    const uint32_t* simIndices, const uint32_t simIndicesSize
)
{
    outData.write(&simPointsSize, sizeof(uint32_t));
    outData.write(simPoints, sizeof(carb::Float3) * simPointsSize);

    outData.write(&simIndicesSize, sizeof(uint32_t));
    outData.write(simIndices, sizeof(uint32_t) * simIndicesSize);
}

void readSurfaceDeformableBodyData(PhysxCookingSurfaceDeformableBodyData& out, const PhysxCookedDataSpan& cookedData)
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

    CARB_ASSERT(dataPtr == reinterpret_cast<const uint8_t*>(cookedData.data) + cookedData.sizeInBytes);
}

// This class handles cooking a single surface deformable body in a background task.
class SurfaceDeformableBodyCookingTask : public cookingtask::CookingTask
{
public:
    SurfaceDeformableBodyCookingTask(
        const SurfaceDeformableBodyCookingParams& params,
        omni::physx::PhysxCookingComputeResult& result)
        : CookingTask(result)
    {
        static_assert(sizeof(m_simToCookingTransform) == sizeof(params.simToCookingTransform));
        m_simToCookingTransform = *reinterpret_cast<const pxr::GfMatrix4d*>(params.simToCookingTransform);

        m_srcPoints.resize(params.srcPointsInSim.size());
        for (uint32_t p = 0; p < m_srcPoints.size(); ++p)
        {
            const pxr::GfVec3f& src = *reinterpret_cast<const pxr::GfVec3f*>(&params.srcPointsInSim[p]);
            pxr::GfVec3f dst = pxr::GfVec3f(m_simToCookingTransform.Transform(src));
            m_srcPoints[p] = { dst[0], dst[1], dst[2] };
        }

        m_isAutoMeshSimplificationEnabled = params.isAutoMeshSimplificationEnabled;
        m_isAutoRemeshingEnabled = params.isAutoRemeshingEnabled;
        m_autoRemeshingResolution = params.autoRemeshingResolution;
        m_autoTriangleTargetCount = params.autoTriangleTargetCount;
    }

    virtual ~SurfaceDeformableBodyCookingTask(void)
    {
        // Wait until thread is finished
        CookingTask::futureWait();
        // Finalize the results; store them in the USD prim and mesh caches
        if (!CookingTask::isFinalized())
        {
            finalize();
        }
    }

    static void applyMeshSimplification(ResultBuffer<carb::Float3>& simpTriPoints,
                                        ResultBuffer<uint32_t>& simpTriIndices,
                                        ResultBuffer<uint32_t>& skinToSimpTriVertsMap,
                                        const carb::Float3* skinPoints,
                                        const uint32_t skinPointsSize,
                                        const uint32_t* skinTriIndices,
                                        const uint32_t skinTriIndicesSize,
                                        const uint32_t _targetResolution,
                                        const uint32_t _targetTriangleCount,
                                        const bool isRemeshingEnabled,
                                        const uint32_t _remeshingResolution)
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
            // targetTriangleCount = (int)(sqrtf(averageTriEdgeLength / targetEdgeLength)*skinTriIndicesSize / 3) + 1;
            // targetTriangleCount = (int)((averageTriEdgeLength / targetEdgeLength)*skinTriIndicesSize / 3) + 1;
            targetTriangleCount = (int)(5.0f * surfaceArea(skinPoints, skinTriIndices, skinTriIndicesSize) /
                                        (targetEdgeLength * targetEdgeLength)) +
                                  1;
        }

        if (isRemeshingEnabled)
        {
            uint32_t remeshingResolution = _remeshingResolution;
            if (remeshingResolution == 0)
            {
                remeshingResolution = targetResolution;
                if (remeshingResolution < 100)
                    remeshingResolution = 100;
                if (remeshingResolution > 250)
                    remeshingResolution = 250;
            }

            ResultBuffer<carb::Float3> remeshedTriPoints;
            ResultBuffer<uint32_t> remeshedTriIndices;

            // with remeshing enabled, we skip generating skinToCollisionTriVertsMap, cause a reliable mapping is
            // currently impossible
            remeshTriangleMeshInternal(remeshedTriPoints.ptr_ref(), remeshedTriPoints.size_ref(),
                                       remeshedTriIndices.ptr_ref(), remeshedTriIndices.size_ref(), nullptr, nullptr,
                                       skinPoints, skinPointsSize, skinTriIndices, skinTriIndicesSize,
                                       remeshingResolution, ResultBuffer<>::allocate);

            simplifyTriangleMeshInternal(simpTriPoints.ptr_ref(), simpTriPoints.size_ref(), simpTriIndices.ptr_ref(),
                                         simpTriIndices.size_ref(), nullptr, nullptr, nullptr, nullptr,
                                         remeshedTriPoints.ptr(), remeshedTriPoints.size(), remeshedTriIndices.ptr(),
                                         remeshedTriIndices.size(), targetTriangleCount, targetEdgeLength, false, false,
                                         ResultBuffer<>::allocate);
        }
        else
        {
            uint32_t** skinToSimp = &skinToSimpTriVertsMap.ptr_ref();
            uint32_t* skinToSimpSize = &skinToSimpTriVertsMap.size_ref();
            uint32_t** simpToSkin = nullptr;
            uint32_t* simpToSkinSize = nullptr;
            const bool removeDisconnectedPatches = false;

            simplifyTriangleMeshInternal(simpTriPoints.ptr_ref(), simpTriPoints.size_ref(), simpTriIndices.ptr_ref(),
                                         simpTriIndices.size_ref(), skinToSimp, skinToSimpSize, simpToSkin,
                                         simpToSkinSize, skinPoints, skinPointsSize, skinTriIndices, skinTriIndicesSize,
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

            // compute welding parameter
            // use bounds in cooking space to determine a reasonable weld tolerance
            carb::Float3 bMin, bMax;
            boundingBox(m_srcPoints.data(), uint32_t(m_srcPoints.size()), bMin, bMax);
            carb::Float3 boundsDim{ bMax.x - bMin.x, bMax.y - bMin.y, bMax.z - bMin.z };
            float boundsDimMin = fminf(boundsDim.x, fminf(boundsDim.y, boundsDim.z));
            float meshWeldTolerance = boundsDimMin / 1e6f;

            ResultBuffer<carb::Float3> simTriPoints;
            ResultBuffer<uint32_t> simTriIndices;
            if (m_isAutoMeshSimplificationEnabled)
            {
                ResultBuffer<uint32_t> skinToSimTriVertsMap;

                uint32_t resolution = 0;
                applyMeshSimplification(simTriPoints, simTriIndices, skinToSimTriVertsMap, m_srcPoints.data(),
                                        uint32_t(m_srcPoints.size()), srcTriIndices.data(),
                                        uint32_t(srcTriIndices.size()), resolution,
                                        m_autoTriangleTargetCount, m_isAutoRemeshingEnabled, m_autoRemeshingResolution);
            }
            else
            {
                simTriPoints.referenceOther(m_srcPoints.data(), uint32_t(m_srcPoints.size()));
                simTriIndices.referenceOther(srcTriIndices.data(), uint32_t(srcTriIndices.size()));
            }

            // welding
            // TODO, the MeshCleaner should probably remove isolated vertices initially
            // double execution of the MeshCleaner seems to work for removing isolated vertices.
            MeshCleaner cleaner_init(simTriPoints.size(), reinterpret_cast<PxVec3*>(simTriPoints.ptr()),
                                     simTriIndices.size() / 3, simTriIndices.ptr(), meshWeldTolerance);

            MeshCleaner cleaner(cleaner_init.mNbVerts, cleaner_init.mVerts, cleaner_init.mNbTris, cleaner_init.mIndices,
                                meshWeldTolerance);

            ResultBuffer<const carb::Float3> pointsSrc;
            ResultBuffer<const uint32_t> indicesSrc;
            if (m_isAutoMeshSimplificationEnabled)
            {
                pointsSrc.referenceOther(reinterpret_cast<const carb::Float3*>(cleaner.mVerts), cleaner.mNbVerts);
                indicesSrc.referenceOther(cleaner.mIndices, cleaner.mNbTris * 3);
            }
            else
            {
                // use origin mesh, if welded and original are the same, probably to avoid quantized values
                if (cleaner.mNbVerts == m_srcPoints.size() && cleaner.mNbTris == srcTriIndices.size()/3)
                {
                    pointsSrc.referenceOther(simTriPoints.ptr(), simTriPoints.size());
                    indicesSrc.referenceOther(simTriIndices.ptr(), simTriIndices.size());
                }
                else
                {
                    //we don't use source to sim mesh mapping for skinning anymore for simplicity.
                    //in the future we might want to reconstruct direct mapping from skin meshes to sim meshes
                    //as an optimization in the skinning manager itself.
#if 0
                    m_srcToSimMap.resize(uint32_t(m_srcPoints.size()));

                    // patch up vert to weld map
                    for (uint32_t i = 0; i < m_srcPoints.size(); ++i)
                    {
                        uint32_t a = cleaner_init.mRemapVertsToWeld[i];
                        uint32_t b = (a < cleaner_init.mNbVerts) ? cleaner.mRemapVertsToWeld[a] : 0xffffffff;
                        uint32_t c = (b < m_srcPoints.size()) ? b : 0; // map orphaned vertices to sim vertex 0
                        m_srcToSimMap.ptr()[i] = c;
                    }
#endif
                    pointsSrc.referenceOther(reinterpret_cast<const carb::Float3*>(cleaner.mVerts), cleaner.mNbVerts);
                    indicesSrc.referenceOther(cleaner.mIndices, cleaner.mNbTris * 3);
                }
            }

            m_simPoints.resize(pointsSrc.size());
            std::memcpy(m_simPoints.ptr(), pointsSrc.ptr(), sizeof(carb::Float3) * m_simPoints.size());
            m_simIndices.resize(indicesSrc.size());
            std::memcpy(m_simIndices.ptr(), indicesSrc.ptr(), sizeof(int32_t) * m_simIndices.size());

            bool cookingSucceeded = true;
            if (!cookingSucceeded)
            {
                CARB_LOG_ERROR("Creating triangle mesh for deformable simulation failed: %s",
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

            // Write output to stream, for both cache and direct consumption of the result
            if (CookingTask::getCookedDataOutputStreams().empty())
            {
                CookingTask::getCookedDataOutputStreams().push_back(std::make_unique<PxDefaultMemoryOutputStream>());
            }

            PxDefaultMemoryOutputStream& outputStream = *CookingTask::getCookedDataOutputStreams()[0].get();
            writeSurfaceDeformableBodyData(outputStream,
                m_simPoints.ptr(), m_simPoints.size(), m_simIndices.ptr(), m_simIndices.size()
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
    std::vector<carb::Float3> m_srcPoints;
    bool m_isAutoMeshSimplificationEnabled;
    bool m_isAutoRemeshingEnabled;
    uint32_t m_autoRemeshingResolution;
    uint32_t m_autoTriangleTargetCount;

    //out
    ResultBuffer<carb::Float3> m_simPoints;
    ResultBuffer<uint32_t> m_simIndices;
};

CookingTask* createSurfaceDeformableBodyCookingTask(const SurfaceDeformableBodyCookingParams& params,
                                                    omni::physx::PhysxCookingComputeResult& result)
{
    return new SurfaceDeformableBodyCookingTask(params, result);
}

}
