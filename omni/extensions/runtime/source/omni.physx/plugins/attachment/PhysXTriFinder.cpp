// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXTriFinder.h"

#include <PhysXTools.h>

#include "geometry/PxGjkQuery.h"
#include "extensions/PxGjkQueryExt.h"
#include <common/utilities/MemoryMacros.h>

using namespace physx;

namespace omni
{
    namespace trifinder
    {
        struct ProximityInfo
        {
            ProximityInfo() : triId(-1), separation(FLT_MAX) {}

            int32_t triId;

            carb::Float3 pointA;
            carb::Float3 pointB;
            carb::Float3 separatingAxis;
            float separation;
        };

        PxBounds3 triBound(const PxU32* tri, const PxTypedStridedData<const PxVec3>& vertices)
        {
            PxVec3 min = vertices.at(tri[0]);
            PxVec3 max = min;

            for (int i = 1; i < 3; ++i)
            {
                const PxVec3& p = vertices.at(tri[i]);
                if (p.x < min.x) min.x = p.x; if (p.y < min.y) min.y = p.y; if (p.z < min.z) min.z = p.z;
                if (p.x > max.x) max.x = p.x; if (p.y > max.y) max.y = p.y; if (p.z > max.z) max.z = p.z;
            }
            return PxBounds3(min, max);
        }

        PxBVH* createBVH(const PxTypedStridedData<const PxVec3>& vertices, const PxU32* tris, PxU32 numTris, PxBounds3& bvhBounds, PxReal fatten = 1e-6)
        {
            PxBounds3* bounds = new PxBounds3[numTris];
            bvhBounds.setEmpty();

            for (PxU32 i = 0; i < numTris; i++)
            {
                bounds[i] = triBound(&tris[3 * i], vertices);
                bounds[i].fattenFast(fatten);

                bvhBounds.include(bounds[i]);
            }

            PxBVHDesc bvhDesc;
            bvhDesc.bounds.count = numTris;
            bvhDesc.bounds.data = bounds;
            bvhDesc.bounds.stride = sizeof(PxBounds3);
            PxBVH* bvh = PxCreateBVH(bvhDesc);
            delete[] bounds;
            return bvh;
        }

        // adjusted from GuDistancePointTriangle.h, copy in PhysXTetFinder.cpp, DeformableBodyTetMeshCookingTask.cpp
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

        struct bvhOverlapCallback : PxBVH::OverlapCallback
        {
            virtual bool reportHit(PxU32 boundsIndex)
            {
                hitBuffer.pushBack(boundsIndex);
                return true;
            }
            PxArray<PxU32> hitBuffer;
        };

        struct TriFinder : Allocateable
        {
            PxBVH* mBvh;
            PxReal mAbsToleranceSqr;
            PxTypedStridedData<const PxVec3> mVertices;
            PxU32 mNumVertices;
            const PxU32* mTris;
            PxU32 mNumTris;
            PxBounds3 mBounds;

            PxArray<PxVec3> mVerticesCopy;
            PxArray<PxU32> mTrisCopy;

            TriFinder(const PxTypedStridedData<const PxVec3>& vertices, PxU32 numVertices, const PxU32* tris, PxU32 numTris, bool bMakeCopy, PxReal tolerance = 1e-5f);

            //Returns -1 if no tri was found
            int findTri(const PxVec3& queryPoint, PxVec3& bary);
            PxVec3 getPoint(const PxU32 triId, const PxVec3& bary);
            PxArray<ProximityInfo> overlap(const PxGeometry& geom, const PxTransform& geomPos, const PxReal overlapOffset, PxTriangleMeshPoissonSampler* triMeshSampler);
        };

        TriFinder::TriFinder(const PxTypedStridedData<const PxVec3>& vertices, PxU32 numVertices, const PxU32* tris, PxU32 numTris, bool bMakeCopy, float tolerance) :
            mVertices(vertices), mTris(tris), mNumTris(numTris)
        {
            if (bMakeCopy)
            {
                mVerticesCopy.resize(numVertices);
                for (PxU32 i = 0; i < numVertices; ++i)
                {
                    mVerticesCopy[i] = mVertices.at(i);
                }
                mVertices = PxTypedStridedData<const PxVec3>(mVerticesCopy.begin());
                mNumVertices = numVertices;

                mTrisCopy.assign(tris, tris + (PxU64(numTris) * 3));
                mTris = mTrisCopy.begin();
            }

            //compute tolerance based on max edge length
            PxReal maxEdgeLengthSq = 0.0f;
            for (PxU32 i = 0; i < mNumTris; ++i)
            {
                const PxVec3& a = mVertices.at(mTris[i*3 + 0]);
                const PxVec3& b = mVertices.at(mTris[i*3 + 1]);
                const PxVec3& c = mVertices.at(mTris[i*3 + 2]);
                maxEdgeLengthSq = PxMax(maxEdgeLengthSq, (a - b).magnitudeSquared());
                maxEdgeLengthSq = PxMax(maxEdgeLengthSq, (b - c).magnitudeSquared());
                maxEdgeLengthSq = PxMax(maxEdgeLengthSq, (c - a).magnitudeSquared());
            }
            mAbsToleranceSqr = maxEdgeLengthSq*tolerance*tolerance;
            mBvh = createBVH(mVertices, mTris, mNumTris, mBounds, PxSqrt(mAbsToleranceSqr));
        }

        //Returns -1 if no tri was found
        int TriFinder::findTri(const PxVec3& queryPoint, PxVec3& bary)
        {
            PxBounds3 box(queryPoint, queryPoint);

            bvhOverlapCallback callback;
            PxBoxGeometry bbox(box.getExtents());
            mBvh->overlap(bbox, PxTransform(queryPoint), callback);
            PxU32 numHits = callback.hitBuffer.size();
            PxU32* hitBuffer = callback.hitBuffer.begin();

            PxReal minDistSq = FLT_MAX;
            PxVec3 minBary(0.0f);
            PxU32 minHit(-1);
            for (PxU32 i = 0; i < numHits && minDistSq > 0.0f; ++i)
            {
                const PxU32* tri = &mTris[3 * hitBuffer[i]];
                const PxVec3& p0 = mVertices.at(tri[0]);
                const PxVec3& p1 = mVertices.at(tri[1]);
                const PxVec3& p2 = mVertices.at(tri[2]);

                PxVec3 testBary;
                PxReal testDistSq = distancePointTriangleSquared(testBary, queryPoint, p0, p1, p2, p1 - p0, p2 - p0);

                if (testDistSq < minDistSq)
                {
                    minDistSq = testDistSq;
                    minBary = testBary;
                    minHit = hitBuffer[i];
                }
            }

            if (minDistSq < mAbsToleranceSqr)
            {
                bary = minBary;
                return minHit;
            }
            return -1;
        }

        PxVec3 TriFinder::getPoint(const PxU32 triId, const PxVec3& bary)
        {
            const PxU32* tri = &mTris[3 * triId];
            return mVertices.at(tri[0]) * bary.x + mVertices.at(tri[1]) * bary.y + mVertices.at(tri[2]) * bary.z;
        }

        struct TriangleSupport : PxGjkQuery::Support
        {
            PxVec3 v0, v1, v2;
            PxReal margin;

            TriangleSupport(const PxVec3& _v0, const PxVec3& _v1, const PxVec3& _v2, PxReal _margin) :
                v0(_v0), v1(_v1), v2(_v2), margin(_margin)
            {
            }

            virtual PxReal getMargin() const
            {
                return margin;
            }

            virtual PxVec3 supportLocal(const PxVec3& dir) const
            {
                float d0 = dir.dot(v0), d1 = dir.dot(v1), d2 = dir.dot(v2);
                return (d0 > d1 && d0 > d2) ? v0 : (d1 > d2) ? v1 : v2;
            }
        };

        ProximityInfo shapeOverlapsTetrahedronGJK(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxGeometry& geom, const PxTransform& geomPos, PxReal contactDistance)
        {
            PxVec3 pointA, pointB, separatingAxis;
            PxReal separation = PX_MAX_REAL;

            TriangleSupport t(geomPos.transformInv(a), geomPos.transformInv(b), geomPos.transformInv(c), 0.0f);

            switch (geom.getType())
            {
            case PxGeometryType::eSPHERE:
            {
                const PxSphereGeometry& shapeGeom = static_cast<const PxSphereGeometry&>(geom);
                PxGjkQueryExt::SphereSupport s(shapeGeom);
                if (!PxGjkQuery::proximityInfo(t, s, PxTransform(PxIdentity), PxTransform(PxIdentity), contactDistance, 0.0f, pointA, pointB, separatingAxis, separation))
                {
                    // PxGjkQuery::proximityInfo could return false because of EPA_FAIL even if the shapes overlap
                    // if shapes overlap, set separation to -PX_MAX_REAL. The real separation distance is not important as long as it is less than contactDistance
                    if (PxGjkQuery::overlap(t, s, PxTransform(PxIdentity), PxTransform(PxIdentity)))
                        separation = -PX_MAX_REAL;
                }
                break;
            }
            case PxGeometryType::ePLANE:
            {
                const PxTransform pose0 = PxTransform(PxIdentity);
                const PxTransform pose1 = PxTransform(PxIdentity);
                //PxPlane plane = pose1.transform(PxPlane(1, 0, 0, 0));
                //PxPlane localPlane = pose0.inverseTransform(plane);
                PxPlane plane(1, 0, 0, 0);
                PxPlane localPlane = plane;
                PxVec3 point = t.supportLocal(-localPlane.n);
                float dist = localPlane.distance(point);
                float radius = t.getMargin();
                separation = dist - radius;

                break;
            }
            case PxGeometryType::eCAPSULE:
            {
                const PxCapsuleGeometry& shapeGeom = static_cast<const PxCapsuleGeometry&>(geom);
                PxGjkQueryExt::CapsuleSupport c(shapeGeom);
                if (!PxGjkQuery::proximityInfo(t, c, PxTransform(PxIdentity), PxTransform(PxIdentity), contactDistance, 0.0f, pointA, pointB, separatingAxis, separation))
                {
                    // PxGjkQuery::proximityInfo could return false because of EPA_FAIL even if the shapes overlap
                    // if shapes overlap, set separation to -PX_MAX_REAL. The real separation distance is not important as long as it is less than contactDistance
                    if (PxGjkQuery::overlap(t, c, PxTransform(PxIdentity), PxTransform(PxIdentity)))
                        separation = -PX_MAX_REAL;
                }
                break;
            }
            case PxGeometryType::eBOX:
            {
                const PxBoxGeometry& shapeGeom = static_cast<const PxBoxGeometry&>(geom);
                PxGjkQueryExt::BoxSupport b(shapeGeom);
                if (!PxGjkQuery::proximityInfo(t, b, PxTransform(PxIdentity), PxTransform(PxIdentity), contactDistance, 0.0f, pointA, pointB, separatingAxis, separation))
                {
                    // PxGjkQuery::proximityInfo could return false because of EPA_FAIL even if the shapes overlap
                    // if shapes overlap, set separation to -PX_MAX_REAL. The real separation distance is not important as long as it is less than contactDistance
                    if (PxGjkQuery::overlap(t, b, PxTransform(PxIdentity), PxTransform(PxIdentity)))
                        separation = -PX_MAX_REAL;
                }
                break;
            }
            case PxGeometryType::eCONVEXMESH:
            {
                const PxConvexMeshGeometry& shapeGeom = static_cast<const PxConvexMeshGeometry&>(geom);
                PxGjkQueryExt::ConvexMeshSupport c(shapeGeom);
                PxGjkQuery::proximityInfo(t, c, PxTransform(PxIdentity), PxTransform(PxIdentity), contactDistance, 0.0f, pointA, pointB, separatingAxis, separation);
                break;
            }
            case PxGeometryType::eTRIANGLEMESH:
            {
                const PxTriangleMeshGeometry& shapeGeom = static_cast<const PxTriangleMeshGeometry&>(geom);

                const PxU32 MAX_TRIANGLES = 1024;
                const PxReal meshContactMargin = 0.0f;

                PxBounds3 tetBounds;
                tetBounds.setEmpty();
                tetBounds.include(a); tetBounds.include(b); tetBounds.include(c);
                tetBounds.fattenSafe(contactDistance);
                const PxTransform& poseT = PxTransform(tetBounds.getCenter());
                PxBoxGeometry boxGeom(tetBounds.getExtents());

                PxU32 triangles[MAX_TRIANGLES];
                bool overflow = false;
                PxU32 triangleCount = PxMeshQuery::findOverlapTriangleMesh(boxGeom, poseT, shapeGeom, geomPos, triangles, MAX_TRIANGLES, 0, overflow);
                CARB_ASSERT(!overflow);

                PxReal minSeparation = PX_MAX_REAL;
                PxVec3 scale = shapeGeom.scale.scale;
                for (PxU32 i = 0; i < triangleCount; ++i)
                {
                    PxTriangle tri;
                    PxMeshQuery::getTriangle(shapeGeom, PxTransform(PxIdentity), triangles[i], tri);
                    TriangleSupport triSupport(tri.verts[0].multiply(scale), tri.verts[1].multiply(scale), tri.verts[2].multiply(scale), meshContactMargin);
                    PxGjkQuery::proximityInfo(t, triSupport, PxTransform(PxIdentity), PxTransform(PxIdentity), contactDistance, 0.0f, pointA, pointB, separatingAxis, separation);
                    if (separation < minSeparation)
                        minSeparation = separation;
                }
                separation = minSeparation;

                break;
            }
            default:
            {
                CARB_ASSERT(false);
                break;
            }
            }

            PxVec3 worldPosA = geomPos.transform(pointA);
            PxVec3 worldPosB = geomPos.transform(pointB);

            ProximityInfo info;
            info.pointA = { worldPosA.x, worldPosA.y, worldPosA.z };
            info.pointB = { worldPosB.x, worldPosB.y, worldPosB.z };
            info.separatingAxis = { separatingAxis.x, separatingAxis.y, separatingAxis.z };
            info.separation = separation;

            return info;
        }

        PxArray<ProximityInfo> TriFinder::overlap(const PxGeometry& geom, const PxTransform& geomPos, const PxReal overlapOffset, PxTriangleMeshPoissonSampler* triMeshSampler)
        {
            PxArray<ProximityInfo> result;
            if (!mBvh)
            {
                return result;
            }

            PxBounds3 box;
            PxGeometryQuery::computeGeomBounds(box, geom, geomPos, 0.0f, 1.01f);
            box.fattenSafe(overlapOffset);

            bvhOverlapCallback callback;
            PxBoxGeometry bbox(box.getExtents());
            mBvh->overlap(bbox, PxTransform(box.getCenter()), callback);
            PxU32 numHits = callback.hitBuffer.size();
            PxU32* hitBuffer = callback.hitBuffer.begin();

            ProximityInfo info;

            for (PxU32 i = 0; i < numHits; ++i)
            {
                const PxU32* tri = &mTris[3 * hitBuffer[i]];

                info = shapeOverlapsTetrahedronGJK(mVertices.at(tri[0]), mVertices.at(tri[1]), mVertices.at(tri[2]), geom, geomPos, overlapOffset);
                info.triId = hitBuffer[i];

                if (info.separation <= overlapOffset)
                {
                    result.pushBack(info);
                }
                else if (geom.getType() == PxGeometryType::eTRIANGLEMESH && triMeshSampler)
                {
                    for (PxU32 idx = 0; idx < 3; ++idx)
                    {
                        if (triMeshSampler->isPointInTriangleMesh(mVertices.at(tri[idx])))
                        {
                            result.pushBack(info);
                            break;
                        }
                    }
                }
            }

            return result;
        }

        // interface functions

        uint64_t createTriFinder(const carb::Float3* points, const uint32_t pointsSize, const uint32_t* indices, const uint32_t indicesSize, bool bMakeCopy)
        {
            CARB_ASSERT(indicesSize % 3 == 0);
            PxTypedStridedData<const PxVec3> stridedData(reinterpret_cast<const PxVec3*>(points));
            TriFinder* triFinder = ICE_NEW(TriFinder)(stridedData, pointsSize, (const PxU32*)indices, indicesSize / 3, bMakeCopy);
            return uint64_t(triFinder);
        }

        uint64_t createTriFinder(const carb::Float4* points, const uint32_t pointsSize, const uint32_t* indices, const uint32_t indicesSize, bool bMakeCopy)
        {
            CARB_ASSERT(indicesSize % 3 == 0);
            PxTypedStridedData<const PxVec3> stridedData(reinterpret_cast<const PxVec3*>(points), sizeof(carb::Float4));
            TriFinder* triFinder = ICE_NEW(TriFinder)(stridedData, pointsSize, (const PxU32*)indices, indicesSize / 3, bMakeCopy);
            return uint64_t(triFinder);
        }

        void releaseTriFinder(const uint64_t triFinder)
        {
            TriFinder* pf = reinterpret_cast<TriFinder*>(triFinder);
            SAFE_DELETE_SINGLE(pf);
        }

        void pointsToTriMeshLocal(int32_t* triIds, carb::Float3* barycentricCoords, const uint64_t triFinder, const carb::Float3* points, const uint32_t pointsSize)
        {
            TriFinder* tf = reinterpret_cast<TriFinder*>(triFinder);
            for (uint32_t p = 0; p < pointsSize; ++p)
            {
                int32_t& dstTriId = triIds[p];
                PxVec3& dstBary = (PxVec3&)barycentricCoords[p];
                const PxVec3& srcPoint = (const PxVec3&)points[p];
                dstTriId = tf->findTri(srcPoint, dstBary);
            }
        }

        void triMeshLocalToPoints(carb::Float3* points, const uint64_t triFinder, const int32_t* triIds, const carb::Float3* barycentricCoords, const uint32_t pointsSize)
        {
            TriFinder* tf = reinterpret_cast<TriFinder*>(triFinder);
            for (uint32_t p = 0; p < pointsSize; ++p)
            {
                const int32_t srcTetId = triIds[p];
                if (srcTetId >= 0)
                {
                    PxVec3& dstPoint = (PxVec3&)points[p];
                    const PxVec3& srcBary = (const PxVec3&)barycentricCoords[p];
                    dstPoint = tf->getPoint(srcTetId, srcBary);
                }
            }
        }

        void overlapTriMeshGeom(int32_t*& triIds, uint32_t& triIdsSize, const uint64_t triFinder, const PxGeometry& geom, const PxTransform& geomPos, const PxReal overlapOffset, void* (*allocateBytes)(size_t), void* triMeshSampler)
        {
            TriFinder* tf = reinterpret_cast<TriFinder*>(triFinder);
            PxArray<ProximityInfo> tmpInfo = tf->overlap(geom, geomPos, overlapOffset, (PxTriangleMeshPoissonSampler*)triMeshSampler);

            triIdsSize = uint32_t(tmpInfo.size());
            triIds = (int32_t*)allocateBytes(sizeof(uint32_t) * triIdsSize);

            for (uint32_t i = 0; i < triIdsSize; i++)
            {
                triIds[i] = tmpInfo[i].triId;
            }
        }

        const uint32_t* getIndices(uint32_t& indicesSize, const uint64_t triFinder)
        {
            TriFinder* tf = reinterpret_cast<TriFinder*>(triFinder);
            indicesSize = tf->mNumTris * 3;
            return tf->mTris;
        }

        const carb::Float3* getPoints(uint32_t& pointsSize, uint32_t& pointsByteStride, const uint64_t triFinder)
        {
            TriFinder* tf = reinterpret_cast<TriFinder*>(triFinder);
            pointsSize = tf->mNumVertices;
            pointsByteStride = tf->mVertices.stride;
            return reinterpret_cast<const carb::Float3*>(&tf->mVertices.at(0));
        }

        void getAdjacency(uint32_t* vertTriCounts, uint32_t*& vertTriIds, uint32_t& vertTriIdsSize, const uint64_t triFinder,
            const uint32_t* vertIds, const uint32_t vertIdsSize, void* (*allocateBytes)(size_t))
        {
            TriFinder* tf = reinterpret_cast<TriFinder*>(triFinder);

            //create map from vertId to vertIdIndex and 
            typedef std::unordered_map<uint32_t, uint32_t> VertMap;
            VertMap vertMap;
            vertMap.reserve(vertIdsSize);
            for (uint32_t i = 0; i < vertIdsSize; ++i)
            {
                vertMap[vertIds[i]] = i;
                vertTriCounts[i] = 0;
            }

            //count tris per vertex
            for (uint32_t i = 0; i < tf->mNumTris; ++i)
            {
                const uint32_t* tri = tf->mTris + i*3;
                for (uint32_t v = 0; v < 3; ++v)
                {
                    VertMap::iterator it = vertMap.find(tri[v]);
                    if (it != vertMap.end())
                    {
                        vertTriCounts[it->second]++;
                    }
                }
            }

            std::vector<uint32_t> vertTriOffsets(vertIdsSize);
            uint32_t vertTriOffset = 0;
            for (uint32_t i = 0; i < vertIdsSize; ++i)
            {
                vertTriOffsets[i] = vertTriOffset;
                vertTriOffset += vertTriCounts[i];
            }

            //allocate vertTriIds and set output size
            vertTriIds = reinterpret_cast<uint32_t*>(allocateBytes(sizeof(uint32_t)*vertTriOffset));
            vertTriIdsSize = vertTriOffset;

            //fill in triangle ids
            for (uint32_t i = 0; i < tf->mNumTris; ++i)
            {
                const uint32_t* tri = tf->mTris + i*3;
                for (uint32_t v = 0; v < 3; ++v)
                {
                    VertMap::iterator it = vertMap.find(tri[v]);
                    if (it != vertMap.end())
                    {
                        vertTriIds[vertTriOffsets[it->second]++] = i;
                    }
                }
            }

        }
    }
}
