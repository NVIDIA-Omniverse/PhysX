// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXTetFinder.h"

#include <PhysXTools.h>

#include "geometry/PxGjkQuery.h"
#include "extensions/PxGjkQueryExt.h"

#include <usdLoad/LoadUsd.h>
#include <CookingDataAsync.h>
#include <ConeCylinderConvexMesh.h>
#include <common/utilities/MemoryMacros.h>

using namespace pxr;
using namespace physx;
using namespace omni::physx;
using namespace omni::physx::usdparser;

namespace omni
{
    namespace tetfinder
    {
        struct ProximityInfo
        {
            ProximityInfo() : tetId(-1), separation(FLT_MAX) {}

            int32_t tetId;

            carb::Float3 pointA;
            carb::Float3 pointB;
            carb::Float3 separatingAxis;
            float separation;
        };

        PxBVH* createTetBVH(const PxTypedStridedData<const PxVec3>& vertices, const PxU32* tets, PxU32 numTets, PxBounds3& bvhBounds, PxReal fatten = 1e-6f)
        {
            if (numTets == 0)
            {
                return nullptr;
            }

            PxBounds3* bounds = new PxBounds3[numTets];
            bvhBounds.setEmpty();

            for (PxU32 i = 0; i < numTets; i++)
            {
                const PxVec3& p0 = vertices.at(tets[4 * i + 0]);
                const PxVec3& p1 = vertices.at(tets[4 * i + 1]);
                const PxVec3& p2 = vertices.at(tets[4 * i + 2]);
                const PxVec3& p3 = vertices.at(tets[4 * i + 3]);
                bounds[i] = PxBounds3::boundsOfPoints(p0, p1);
                bounds[i].include(p2);
                bounds[i].include(p3);
                bounds[i].fattenFast(fatten);

                bvhBounds.include(bounds[i]);
            }

            PxBVHDesc bvhDesc;
            bvhDesc.bounds.count = numTets;
            bvhDesc.bounds.data = bounds;
            bvhDesc.bounds.stride = sizeof(PxBounds3);
            PxBVH* bvh = PxCreateBVH(bvhDesc);
            delete[] bounds;
            return bvh;
        }

        PxBVH* createTriBVH(const PxTypedStridedData<const PxVec3>& vertices, const PxU32* tris, PxU32 numTris, PxBounds3& bvhBounds, PxReal fatten = 1e-6f)
        {
            if (numTris == 0)
            {
                return nullptr;
            }

            PxBounds3* bounds = new PxBounds3[numTris];
            bvhBounds.setEmpty();

            for (PxU32 i = 0; i < numTris; i++)
            {
                const PxVec3& p0 = vertices.at(tris[3 * i + 0]);
                const PxVec3& p1 = vertices.at(tris[3 * i + 1]);
                const PxVec3& p2 = vertices.at(tris[3 * i + 2]);
                bounds[i] = PxBounds3::boundsOfPoints(p0, p1);
                bounds[i].include(p2);
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

        PX_INLINE void computeBarycentricCoordinates(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxVec3& p, PxVec4& bary)
        {
            const PxVec3 ba = b - a;
            const PxVec3 ca = c - a;
            const PxVec3 da = d - a;
            const PxVec3 pa = p - a;

            const PxReal detBcd = ba.dot(ca.cross(da));
            const PxReal detPcd = pa.dot(ca.cross(da));

            bary.y = detPcd / detBcd;

            const PxReal detBpd = ba.dot(pa.cross(da));
            bary.z = detBpd / detBcd;

            const PxReal detBcp = ba.dot(ca.cross(pa));

            bary.w = detBcp / detBcd;
            bary.x = 1 - bary.y - bary.z - bary.w;
        }

        //adjusted from GuDistancePointTriangle.h, copy in PhysXTriFinder.cpp, DeformableBodyTetMeshCookingTask.cpp
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

        //adjusted from GuDistancePointTetrahedron.h
        PX_INLINE PxReal distancePointTetrahedronSurfaceSquared(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d)
        {
            const PxVec3 ab = b - a;
            const PxVec3 ac = c - a;
            const PxVec3 ad = d - a;
            const PxVec3 bc = c - b;
            const PxVec3 bd = d - b;
            PxVec3 bary;
            //point to face 0, 1, 2
            PxReal bestDistSq = distancePointTriangleSquared(bary, p, a, b, c, ab, ac);

            // 0, 2, 3
            PxReal distSq = distancePointTriangleSquared(bary, p, a, c, d, ac, ad);
            bestDistSq = PxMin(bestDistSq, distSq);

            // 0, 3, 1
            distSq = distancePointTriangleSquared(bary, p, a, d, b, ad, ab);
            bestDistSq = PxMin(bestDistSq, distSq);

            // 1, 3, 2
            distSq = distancePointTriangleSquared(bary, p, b, d, c, bd, bc);
            bestDistSq = PxMin(bestDistSq, distSq);

            return bestDistSq;
        }

        PX_INLINE PxReal distancePointTetrahedronSquared(PxVec4& bary, const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d)
        {
            computeBarycentricCoordinates(a, b, c, d, p, bary);

            if (bary.x >= 0.0f && bary.x <= 1.0f && bary.y >= 0.0f && bary.y <= 1.0f &&
                bary.z >= 0.0f && bary.z <= 1.0f && bary.w >= 0.0f && bary.w <= 1.0f)
            {
                return 0.0f;
            }
            else
            {
                return distancePointTetrahedronSurfaceSquared(p, a, b, c, d);
            }
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

        struct TetFinder : public Allocateable
        {
            PxBVH* mBvh = nullptr;
            PxReal mAbsToleranceSqr;
            PxTypedStridedData<const PxVec3> mVertices;
            PxU32 mNumVertices;
            const PxU32* mTets = nullptr;
            PxU32 mNumTets;
            PxBounds3 mBounds;

            // Used by createPointsToTetrahedronMap
            PxArray<PxVec3> mVerticesCopy;
            PxArray<PxU32> mTetsCopy;

            TetFinder(const PxTypedStridedData<const PxVec3>& vertices, PxU32 numVertices, const PxU32* tets, PxU32 numTets, float tolerance = 1e-5f);
            ~TetFinder();
            
            int findTet(const PxVec3& queryPoint, PxVec4& bary) const; // Returns -1 if no tet was found
            bool findClosestTets(PxU32* tetIds, PxVec4* barys, PxVec3* distanceDirs, const PxVec3* points, const PxU32 pointsSize) const;
            bool findAllTets(PxU32* tetIds, PxVec4* barys, const PxVec3* points, const PxU32 pointsSize) const;
            PxVec3 getPoint(const PxU32 tetId, const PxVec4& bary) const;

            PxArray<ProximityInfo> overlapDeprecated(const PxGeometry& geom, const PxTransform& geomPos, const PxReal overlapOffset, PxTriangleMeshPoissonSampler* triMeshSampler) const;
            
            void overlap(PxArray<ProximityInfo>& proximities,
                const PxGeometry& geom, const PxTransform& geomPos, const PxReal overlapOffset,
                PxTriangleMeshPoissonSampler* triMeshSampler) const;
            
            PxArray<PxPair<PxI32, PxI32>> overlapDeprecated(const TetFinder& tetFinder, const PxReal overlapOffset, const bool reportPairs);
            PxArray<PxPair<PxI32, PxI32>> overlapDeprecated(const PxVec3* triPoints, const PxU32* triIndices, PxU32 triIndicesSize, const PxReal overlapOffset, const bool reportPairs);

            void overlapPairs(PxArray<PxPair<PxI32, PxI32>>& tetIdPairs,
                const TetFinder& tetFinder, const PxReal overlapOffset) const;
            void overlapAny(PxArray<PxI32>& tetIdsThis, PxArray<PxI32>& tetIdsOther,
                const TetFinder& tetFinder, const PxReal overlapOffset) const;
            void overlapPairs(PxArray<PxPair<PxI32, PxI32>>& tetTriIdPairs,
                const PxVec3* triPoints, const PxU32* triIndices, PxU32 triIndicesSize, const PxReal overlapOffset) const;
            void overlapAny(PxArray<PxI32>& tetIds, PxArray<PxI32>& triIds,
                const PxVec3* triPoints, const PxU32* triIndices, PxU32 triIndicesSize, const PxReal overlapOffset) const;

        private:
            void tetTetQuery(PxArray<PxGeomIndexPair>& hitBuffer,
                const TetFinder& otherTetFinder, float overlapOffset) const;

            void tetTriQuery(PxArray<PxGeomIndexPair>& hitBuffer,
                const PxVec3* triPoints, const PxU32* triIndices, PxU32 triIndicesSize, const PxReal overlapOffset) const;
        };

        TetFinder::TetFinder(const PxTypedStridedData<const PxVec3>& vertices, PxU32 numVertices, const PxU32* tets, PxU32 numTets, float tolerance) :
            mVertices(vertices), mTets(tets), mNumTets(numTets)
        {
            //copy data
            mVerticesCopy.resize(numVertices);
            for (PxU32 i = 0; i < numVertices; ++i)
            {
                mVerticesCopy[i] = mVertices.at(i);
            }
            mVertices = PxTypedStridedData<const PxVec3>(mVerticesCopy.begin());
            mNumVertices = numVertices;

            mTetsCopy.assign(tets, tets + (PxU64(numTets) * 4));
            mTets = mTetsCopy.begin();

            //compute tolerance based on max edge length
            PxReal maxEdgeLengthSq = 0.0f;
            for (PxU32 i = 0; i < mNumTets; ++i)
            {
                const PxVec3& a = mVertices.at(mTets[i*4 + 0]);
                const PxVec3& b = mVertices.at(mTets[i*4 + 1]);
                const PxVec3& c = mVertices.at(mTets[i*4 + 2]);
                const PxVec3& d = mVertices.at(mTets[i*4 + 3]);
                maxEdgeLengthSq = PxMax(maxEdgeLengthSq, (a - b).magnitudeSquared());
                maxEdgeLengthSq = PxMax(maxEdgeLengthSq, (a - c).magnitudeSquared());
                maxEdgeLengthSq = PxMax(maxEdgeLengthSq, (a - d).magnitudeSquared());
                maxEdgeLengthSq = PxMax(maxEdgeLengthSq, (b - c).magnitudeSquared());
                maxEdgeLengthSq = PxMax(maxEdgeLengthSq, (c - d).magnitudeSquared());
                maxEdgeLengthSq = PxMax(maxEdgeLengthSq, (d - b).magnitudeSquared());
            }
            mAbsToleranceSqr = maxEdgeLengthSq*tolerance*tolerance;

            mBvh = createTetBVH(mVertices, mTets, mNumTets, mBounds, PxSqrt(mAbsToleranceSqr));
        }

        TetFinder::~TetFinder()
        {
            SAFE_RELEASE(mBvh);
        }

        //Returns -1 if no tet was found
        int TetFinder::findTet(const PxVec3& queryPoint, PxVec4& bary) const
        {
            if (!mBvh)
            {
                return -1;
            }

            PxBounds3 box(queryPoint, queryPoint);

            bvhOverlapCallback callback;
            PxBoxGeometry bbox(box.getExtents());
            mBvh->overlap(bbox, PxTransform(queryPoint), callback);
            PxU32 numHits = callback.hitBuffer.size();
            PxU32* hitBuffer = callback.hitBuffer.begin();

            PxReal minDistSq = FLT_MAX;
            PxVec4 minBary(0.0f);
            PxU32 minHit(-1);
            for (PxU32 i = 0; i < numHits && minDistSq > 0.0f; ++i)
            {
                const PxU32* tet = &mTets[4 * hitBuffer[i]];
                const PxVec3& p0 = mVertices.at(tet[0]);
                const PxVec3& p1 = mVertices.at(tet[1]);
                const PxVec3& p2 = mVertices.at(tet[2]);
                const PxVec3& p3 = mVertices.at(tet[3]);

                PxVec4 testBary;
                PxReal testDistSq = distancePointTetrahedronSquared(testBary, queryPoint, p0, p1, p2, p3);

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

        bool TetFinder::findClosestTets(PxU32* tetIds, PxVec4* barys, PxVec3* distanceDirs, const PxVec3* points, const PxU32 pointsSize) const
        {
            if (pointsSize == 0)
            {
                return false;
            }

            PxArray<PxVec3> queryPointsArray;
            queryPointsArray.assign((const PxVec3*)points, (const PxVec3*)points + pointsSize);

            PxArray<PxU32> tetIdsArray(pointsSize);
            PxArray<PxVec4> barysArray(pointsSize);
            PxTetrahedronMeshExt::createPointsToTetrahedronMap(mVerticesCopy, mTetsCopy, queryPointsArray, barysArray, tetIdsArray);

            if (barysArray.empty() || tetIdsArray.empty())
            {
                CARB_LOG_WARN("PxTetrahedronMeshExt::createPointsToTetrahedronMap failed!");
                return false;
            }             

            PxMemCopy(tetIds, tetIdsArray.begin(), sizeof(int32_t)*pointsSize);
            PxMemCopy(barys, barysArray.begin(), sizeof(PxVec4)*pointsSize);

            if (distanceDirs)
            {
                for (PxU32 i = 0; i < pointsSize; ++i)
                {
                    PxVec3 insidePoint = getPoint(tetIds[i], barys[i]);
                    PxVec3 distanceDir = points[i] - insidePoint;
                    distanceDirs[i] = (distanceDir.magnitudeSquared() < mAbsToleranceSqr) ? PxVec3(0.0f) : distanceDir;
                }
            }

            return true;
        }

        bool TetFinder::findAllTets(PxU32* tetIds, PxVec4* barys, const PxVec3* points, const PxU32 pointsSize) const
        {
            if (pointsSize == 0)
            {
                return false;
            }

            PxArray<PxVec3> queryPointsArray;
            queryPointsArray.assign((const PxVec3*)points, (const PxVec3*)points + pointsSize);

            PxArray<PxU32> tetIdsArray(pointsSize);
            PxArray<PxVec4> barysArray(pointsSize);
            PxTetrahedronMeshExt::createPointsToTetrahedronMap(mVerticesCopy, mTetsCopy, queryPointsArray, barysArray, tetIdsArray);

            if (barysArray.empty() || tetIdsArray.empty())
            {
                CARB_LOG_WARN("PxTetrahedronMeshExt::createPointsToTetrahedronMap failed!");
                return false;
            }

            PxMemCopy(tetIds, tetIdsArray.begin(), sizeof(int32_t) * pointsSize);
            PxMemCopy(barys, barysArray.begin(), sizeof(PxVec4) * pointsSize);

            for (PxU32 i = 0; i < pointsSize; ++i)
            {
                PxVec3 insidePoint = getPoint(tetIds[i], barys[i]);
                PxVec3 distanceDir = points[i] - insidePoint;
                if (distanceDir.magnitudeSquared() >= mAbsToleranceSqr)
                {
                    const PxU32* tet = &mTets[4 * tetIds[i]];
                    computeBarycentric(mVertices.at(tet[0]), mVertices.at(tet[1]), mVertices.at(tet[2]), mVertices.at(tet[3]), points[i], barys[i]);
                }
            }

            return true;
        }

        PxVec3 TetFinder::getPoint(const PxU32 tetId, const PxVec4& bary) const
        {
            const PxU32* tet = &mTets[4 * tetId];
            return mVertices.at(tet[0]) * bary.x + mVertices.at(tet[1]) * bary.y + mVertices.at(tet[2]) * bary.z + mVertices.at(tet[3]) * bary.w;
        }

        struct TetrahedronSupport : PxGjkQuery::Support
        {
            PxVec3 v0, v1, v2, v3;
            PxReal margin;
            TetrahedronSupport(const PxVec3& _v0, const PxVec3& _v1, const PxVec3& _v2, const PxVec3& _v3, PxReal _margin) :
                v0(_v0), v1(_v1), v2(_v2), v3(_v3), margin(_margin)
            {
            }

            virtual PxReal getMargin() const
            {
                return margin;
            }

            virtual PxVec3 supportLocal(const PxVec3& dir) const
            {
                float d0 = dir.dot(v0), d1 = dir.dot(v1), d2 = dir.dot(v2), d3 = dir.dot(v3);
                return (d0 > d1 && d0 > d2 && d0 > d3) ? v0 : (d1 > d2 && d1 > d3) ? v1 : (d2 > d3) ? v2 : v3;
            }
        };

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

        ProximityInfo shapeOverlapsTetrahedronGJK(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, const PxGeometry& geom, const PxTransform& geomPos, PxReal contactDistance)
        {
            PxVec3 pointA, pointB, separatingAxis;
            PxReal separation = PX_MAX_REAL;

            TetrahedronSupport t(geomPos.transformInv(a), geomPos.transformInv(b), geomPos.transformInv(c), geomPos.transformInv(d), 0.0f);

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
                    tetBounds.include(a); tetBounds.include(b); tetBounds.include(c); tetBounds.include(d);
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

        void TetFinder::overlap(PxArray<ProximityInfo>& proximities,
            const PxGeometry& geom, const PxTransform& geomPos, const PxReal overlapOffset, PxTriangleMeshPoissonSampler* triMeshSampler) const
        {
            proximities.clear();
            if (!mBvh)
            {
                return;
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
                const PxU32* tet = &mTets[4 * hitBuffer[i]];

                info = shapeOverlapsTetrahedronGJK(mVertices.at(tet[0]), mVertices.at(tet[1]), mVertices.at(tet[2]), mVertices.at(tet[3]), geom, geomPos, overlapOffset);
                info.tetId = hitBuffer[i];

                if (info.separation <= overlapOffset)
                {
                    proximities.pushBack(info);
                }
                else if (geom.getType() == PxGeometryType::eTRIANGLEMESH && triMeshSampler)
                {
                    for (PxU32 idx = 0; idx < 4; ++idx)
                    {
                        if (triMeshSampler->isPointInTriangleMesh(mVertices.at(tet[idx])))
                        {
                            proximities.pushBack(info);
                            break;
                        }
                    }
                }
            }
        }

        PxArray<PxPair<PxI32, PxI32>> TetFinder::overlapDeprecated(const TetFinder& tetFinder, const PxReal overlapOffset, const bool reportPairs)
        {
            PxArray<PxPair<PxI32, PxI32>> result;
            if (!mBvh || !tetFinder.mBvh)
            {
                return result;
            }

            PxArray<PxGeomIndexPair> hitBuffer;
            PxDynamicArrayReportCallback<PxGeomIndexPair> callback(hitBuffer);

            //fatten smaller bvh so we can do distance queries
            PxBounds3 tempBounds;
            PxU32 numBounds0 = mBvh->getNbBounds();
            PxU32 numBounds1 = tetFinder.mBvh->getNbBounds();
            PxBVH* bvh0 = nullptr;
            PxBVH* bvh1 = nullptr;
            PxBVH* bvhToRelease = nullptr;
            if (numBounds0 < numBounds1)
            {
                bvh0 = createTetBVH(mVertices, mTets, mNumTets, tempBounds, overlapOffset);
                bvhToRelease = bvh0;
                bvh1 = tetFinder.mBvh;
            }
            else
            {
                bvh0 = mBvh;
                bvh1 = createTetBVH(tetFinder.mVertices, tetFinder.mTets, tetFinder.mNumTets, tempBounds, overlapOffset);
                bvhToRelease = bvh1;
            }
            if (!bvhToRelease)
            {
                return result;
            }

            PxFindOverlap(callback, *bvh0, *bvh1);

            bvhToRelease->release();

            PxU32 numHits = hitBuffer.size();
            if (reportPairs)
            {
                //do full narrow phase in case we need to report all tet-tet pairs below the required distance
                for (PxU32 i = 0; i < numHits; ++i)
                {
                    PxU32 id0 = hitBuffer[i].id0;
                    PxU32 id1 = hitBuffer[i].id1;
                    const PxU32* tet0 = &mTets[4 * id0];
                    const PxU32* tet1 = &tetFinder.mTets[4 * id1];

                    PxVec3 pointA, pointB, separatingAxis;
                    PxReal separation = PX_MAX_REAL;

                    TetrahedronSupport t0(mVertices.at(tet0[0]), mVertices.at(tet0[1]), mVertices.at(tet0[2]), mVertices.at(tet0[3]), 0.0f);
                    TetrahedronSupport t1(tetFinder.mVertices.at(tet1[0]), tetFinder.mVertices.at(tet1[1]), tetFinder.mVertices.at(tet1[2]), tetFinder.mVertices.at(tet1[3]), 0.0f);

                    PxGjkQuery::proximityInfo(t0, t1, PxTransform(PxIdentity), PxTransform(PxIdentity), overlapOffset, 0.0f, pointA, pointB, separatingAxis, separation);
                    if (separation <= overlapOffset)
                    {
                        PxPair<PxI32, PxI32> pair(id0, id1);
                        result.pushBack(pair);
                    }
                }
            }
            else
            {
                //report a tet just once, if it's closer than the required distance to ANY tet of the other mesh
                result.reserve(numBounds0 + numBounds1);
                PxHashSet<PxU32> set0;
                PxHashSet<PxU32> set1;
                set0.reserve(numBounds0);
                set1.reserve(numBounds1);
                
                for (PxU32 i = 0; i < numHits; ++i)
                {
                    PxU32 id0 = hitBuffer[i].id0;
                    PxU32 id1 = hitBuffer[i].id1;
                    bool id0_contained = set0.contains(id0);
                    bool id1_contained = set1.contains(id1);
                    if (id0_contained && id1_contained)
                    {
                        continue;
                    }

                    const PxU32* tet0 = &mTets[4 * id0];
                    const PxU32* tet1 = &tetFinder.mTets[4 * id1];

                    PxVec3 pointA, pointB, separatingAxis;
                    PxReal separation = PX_MAX_REAL;

                    TetrahedronSupport t0(mVertices.at(tet0[0]), mVertices.at(tet0[1]), mVertices.at(tet0[2]), mVertices.at(tet0[3]), 0.0f);
                    TetrahedronSupport t1(tetFinder.mVertices.at(tet1[0]), tetFinder.mVertices.at(tet1[1]), tetFinder.mVertices.at(tet1[2]), tetFinder.mVertices.at(tet1[3]), 0.0f);

                    PxGjkQuery::proximityInfo(t0, t1, PxTransform(PxIdentity), PxTransform(PxIdentity), overlapOffset, 0.0f, pointA, pointB, separatingAxis, separation);
                    if (separation <= overlapOffset)
                    {
                        if (!id0_contained)
                        {
                            PxPair<PxI32, PxI32> pair(id0, -1);
                            result.pushBack(pair);
                            set0.insert(id0);
                        }
                        if (!id1_contained)
                        {
                            PxPair<PxI32, PxI32> pair(-1, id1);
                            result.pushBack(pair);
                            set1.insert(id1);
                        }
                    }
                }
            }

            return result;
        }

        PxArray<PxPair<PxI32, PxI32>> TetFinder::overlapDeprecated(const PxVec3* triPoints, const PxU32* triIndices, PxU32 triIndicesSize, const PxReal overlapOffset, const bool reportPairs)
        {
            PxArray<PxPair<PxI32, PxI32>> result;
            if (!mBvh || triPoints == nullptr || triIndices == nullptr || triIndicesSize == 0)
            {
                return result;
            }

            PxU32 numTris = triIndicesSize/3;

            PxArray<PxGeomIndexPair> hitBuffer;
            PxDynamicArrayReportCallback<PxGeomIndexPair> callback(hitBuffer);

            //create fattened BVH for triangle mesh so we can do distance queries
            PxBounds3 tempBounds;
            PxBVH* triBVH = createTriBVH(triPoints, triIndices, numTris, tempBounds, overlapOffset);

            if (!triBVH)
            {
                return result;
            }

            PxFindOverlap(callback, *mBvh, *triBVH);

            triBVH->release();

            PxU32 numHits = hitBuffer.size();
            if (reportPairs)
            {
                //do full narrow phase in case we need to report all tet-tet pairs below the required distance
                for (PxU32 i = 0; i < numHits; ++i)
                {
                    PxU32 tetId = hitBuffer[i].id0;
                    PxU32 triId = hitBuffer[i].id1;
                    const PxU32* tet = &mTets[4 * tetId];
                    const PxU32* tri = &triIndices[3 * triId];

                    PxVec3 pointA, pointB, separatingAxis;
                    PxReal separation = PX_MAX_REAL;

                    TetrahedronSupport tetS(mVertices.at(tet[0]), mVertices.at(tet[1]), mVertices.at(tet[2]), mVertices.at(tet[3]), 0.0f);
                    TriangleSupport triS(triPoints[tri[0]], triPoints[tri[1]], triPoints[tri[2]], 0.0f);

                    PxGjkQuery::proximityInfo(tetS, triS, PxTransform(PxIdentity), PxTransform(PxIdentity), overlapOffset, 0.0f, pointA, pointB, separatingAxis, separation);
                    if (separation <= overlapOffset)
                    {
                        PxPair<PxI32, PxI32> pair(tetId, triId);
                        result.pushBack(pair);
                    }
                }
            }
            else
            {
                //we only support 1:ALL filtering on tetrahedra
                //report a tri just once, if it's closer than the required distance to ANY tet
                result.reserve(numTris);

                PxHashSet<PxU32> triSet;
                triSet.reserve(numTris);

                for (PxU32 i = 0; i < numHits; ++i)
                {
                    PxU32 tetId = hitBuffer[i].id0;
                    PxU32 triId = hitBuffer[i].id1;
                    bool triId_contained = triSet.contains(triId);
                    if (triId_contained)
                    {
                        continue;
                    }

                    const PxU32* tet = &mTets[4 * tetId];
                    const PxU32* tri = &triIndices[3 * triId];

                    PxVec3 pointA, pointB, separatingAxis;
                    PxReal separation = PX_MAX_REAL;

                    TetrahedronSupport tetS(mVertices.at(tet[0]), mVertices.at(tet[1]), mVertices.at(tet[2]), mVertices.at(tet[3]), 0.0f);
                    TriangleSupport triS(triPoints[tri[0]], triPoints[tri[1]], triPoints[tri[2]], 0.0f);

                    PxGjkQuery::proximityInfo(tetS, triS, PxTransform(PxIdentity), PxTransform(PxIdentity), overlapOffset, 0.0f, pointA, pointB, separatingAxis, separation);
                    if (separation <= overlapOffset)
                    {
                        PxPair<PxI32, PxI32> pair(-1, triId);
                        result.pushBack(pair);
                        triSet.insert(triId);
                    }
                }
            }

            return result;
        }

        void TetFinder::tetTetQuery(PxArray<PxGeomIndexPair>& hitBuffer, const TetFinder& otherTetFinder, float overlapOffset) const
        {
            if (!mBvh || !otherTetFinder.mBvh)
            {
                return;
            }

            PxDynamicArrayReportCallback<PxGeomIndexPair> callback(hitBuffer);

            //fatten smaller bvh so we can do distance queries
            PxBounds3 tempBounds;
            PxU32 numBounds0 = mBvh->getNbBounds();
            PxU32 numBounds1 = otherTetFinder.mBvh->getNbBounds();
            PxBVH* bvh0 = nullptr;
            PxBVH* bvh1 = nullptr;
            PxBVH* bvhToRelease = nullptr;
            if (numBounds0 < numBounds1)
            {
                bvh0 = createTetBVH(mVertices, mTets, mNumTets, tempBounds, overlapOffset);
                bvhToRelease = bvh0;
                bvh1 = otherTetFinder.mBvh;
            }
            else
            {
                bvh0 = mBvh;
                bvh1 = createTetBVH(otherTetFinder.mVertices, otherTetFinder.mTets, otherTetFinder.mNumTets, tempBounds, overlapOffset);
                bvhToRelease = bvh1;
            }
            if (!bvhToRelease)
            {
                return;
            }

            PxFindOverlap(callback, *bvh0, *bvh1);

            bvhToRelease->release();
        }

        void TetFinder::overlapPairs(PxArray<PxPair<PxI32, PxI32>>& tetIdPairs, const TetFinder& tetFinder, const PxReal overlapOffset) const
        {
            tetIdPairs.clear();
            PxArray<PxGeomIndexPair> hitBuffer;
            tetTetQuery(hitBuffer, tetFinder, overlapOffset);

            PxU32 numHits = hitBuffer.size();

            //do full narrow phase in case we need to report all tet-tet pairs below the required distance
            for (PxU32 i = 0; i < numHits; ++i)
            {
                PxU32 id0 = hitBuffer[i].id0;
                PxU32 id1 = hitBuffer[i].id1;
                const PxU32* tet0 = &mTets[4 * id0];
                const PxU32* tet1 = &tetFinder.mTets[4 * id1];

                PxVec3 pointA, pointB, separatingAxis;
                PxReal separation = PX_MAX_REAL;

                TetrahedronSupport t0(mVertices.at(tet0[0]), mVertices.at(tet0[1]), mVertices.at(tet0[2]), mVertices.at(tet0[3]), 0.0f);
                TetrahedronSupport t1(tetFinder.mVertices.at(tet1[0]), tetFinder.mVertices.at(tet1[1]), tetFinder.mVertices.at(tet1[2]), tetFinder.mVertices.at(tet1[3]), 0.0f);

                PxGjkQuery::proximityInfo(t0, t1, PxTransform(PxIdentity), PxTransform(PxIdentity), overlapOffset, 0.0f, pointA, pointB, separatingAxis, separation);
                if (separation <= overlapOffset)
                {
                    PxPair<PxI32, PxI32> pair(id0, id1);
                    tetIdPairs.pushBack(pair);
                }
            }
        }

        void TetFinder::overlapAny(PxArray<PxI32>&tetIdsThis, PxArray<PxI32>& tetIdsOther, const TetFinder& tetFinder, const PxReal overlapOffset) const
        {
            tetIdsThis.clear();
            tetIdsOther.clear();

            PxArray<PxGeomIndexPair> hitBuffer;
            tetTetQuery(hitBuffer, tetFinder, overlapOffset);

            PxU32 numHits = hitBuffer.size();

            //report a tet just once, if it's closer than the required distance to ANY tet of the other mesh
            PxU32 numBounds0 = mBvh->getNbBounds();
            PxU32 numBounds1 = tetFinder.mBvh->getNbBounds();
            tetIdsThis.reserve(numBounds0);
            tetIdsOther.reserve(numBounds1);

            PxHashSet<PxU32> set0;
            PxHashSet<PxU32> set1;
            set0.reserve(numBounds0);
            set1.reserve(numBounds1);
                
            for (PxU32 i = 0; i < numHits; ++i)
            {
                PxU32 id0 = hitBuffer[i].id0;
                PxU32 id1 = hitBuffer[i].id1;
                bool id0_contained = set0.contains(id0);
                bool id1_contained = set1.contains(id1);
                if (id0_contained && id1_contained)
                {
                    continue;
                }

                const PxU32* tet0 = &mTets[4 * id0];
                const PxU32* tet1 = &tetFinder.mTets[4 * id1];

                PxVec3 pointA, pointB, separatingAxis;
                PxReal separation = PX_MAX_REAL;

                TetrahedronSupport t0(mVertices.at(tet0[0]), mVertices.at(tet0[1]), mVertices.at(tet0[2]), mVertices.at(tet0[3]), 0.0f);
                TetrahedronSupport t1(tetFinder.mVertices.at(tet1[0]), tetFinder.mVertices.at(tet1[1]), tetFinder.mVertices.at(tet1[2]), tetFinder.mVertices.at(tet1[3]), 0.0f);

                PxGjkQuery::proximityInfo(t0, t1, PxTransform(PxIdentity), PxTransform(PxIdentity), overlapOffset, 0.0f, pointA, pointB, separatingAxis, separation);
                if (separation <= overlapOffset)
                {
                    if (!id0_contained)
                    {
                        tetIdsThis.pushBack(id0);
                        set0.insert(id0);
                    }
                    if (!id1_contained)
                    {
                        tetIdsOther.pushBack(id1);
                        set1.insert(id1);
                    }
                }
            }
        }

        void TetFinder::tetTriQuery(PxArray<PxGeomIndexPair>& hitBuffer,
            const PxVec3* triPoints, const PxU32* triIndices, PxU32 triIndicesSize, const PxReal overlapOffset) const
        {
            if (!mBvh || triPoints == nullptr || triIndices == nullptr || triIndicesSize == 0)
            {
                return;
            }

            PxU32 numTris = triIndicesSize / 3;

            PxDynamicArrayReportCallback<PxGeomIndexPair> callback(hitBuffer);

            //create fattened BVH for triangle mesh so we can do distance queries
            PxBounds3 tempBounds;
            PxBVH* triBVH = createTriBVH(triPoints, triIndices, numTris, tempBounds, overlapOffset);

            if (!triBVH)
            {
                return;
            }

            PxFindOverlap(callback, *mBvh, *triBVH);

            triBVH->release();
        }

        void TetFinder::overlapPairs(PxArray<PxPair<PxI32, PxI32>>& tetTriIdPairs,
            const PxVec3* triPoints, const PxU32* triIndices, PxU32 triIndicesSize, const PxReal overlapOffset) const
        {
            tetTriIdPairs.clear();
            PxArray<PxGeomIndexPair> hitBuffer;
            tetTriQuery(hitBuffer, triPoints, triIndices, triIndicesSize, overlapOffset);

            PxU32 numHits = hitBuffer.size();

            //do full narrow phase in case we need to report all tet-tet pairs below the required distance
            for (PxU32 i = 0; i < numHits; ++i)
            {
                PxU32 tetId = hitBuffer[i].id0;
                PxU32 triId = hitBuffer[i].id1;
                const PxU32* tet = &mTets[4 * tetId];
                const PxU32* tri = &triIndices[3 * triId];

                PxVec3 pointA, pointB, separatingAxis;
                PxReal separation = PX_MAX_REAL;

                TetrahedronSupport tetS(mVertices.at(tet[0]), mVertices.at(tet[1]), mVertices.at(tet[2]), mVertices.at(tet[3]), 0.0f);
                TriangleSupport triS(triPoints[tri[0]], triPoints[tri[1]], triPoints[tri[2]], 0.0f);

                PxGjkQuery::proximityInfo(tetS, triS, PxTransform(PxIdentity), PxTransform(PxIdentity), overlapOffset, 0.0f, pointA, pointB, separatingAxis, separation);
                if (separation <= overlapOffset)
                {
                    PxPair<PxI32, PxI32> pair(tetId, triId);
                    tetTriIdPairs.pushBack(pair);
                }
            }
        }

        void TetFinder::overlapAny(PxArray<PxI32>& tetIds, PxArray<PxI32>& triIds,
            const PxVec3* triPoints, const PxU32* triIndices, PxU32 triIndicesSize, const PxReal overlapOffset) const
        {
            tetIds.clear();
            triIds.clear();
            PxArray<PxGeomIndexPair> hitBuffer;
            tetTriQuery(hitBuffer, triPoints, triIndices, triIndicesSize, overlapOffset);

            PxU32 numHits = hitBuffer.size();

            PxU32 numTris = triIndicesSize / 3;
            triIds.reserve(numTris);
            PxU32 numTetBounds = mBvh->getNbBounds();
            tetIds.reserve(numTetBounds);

            PxHashSet<PxU32> triSet;
            triSet.reserve(numTris);
            PxHashSet<PxU32> tetSet;
            tetSet.reserve(numTetBounds);

            for (PxU32 i = 0; i < numHits; ++i)
            {
                PxU32 tetId = hitBuffer[i].id0;
                PxU32 triId = hitBuffer[i].id1;
                bool triId_contained = triSet.contains(triId);
                bool tetId_contained = tetSet.contains(tetId);
                if (triId_contained && tetId_contained)
                {
                    continue;
                }

                const PxU32* tet = &mTets[4 * tetId];
                const PxU32* tri = &triIndices[3 * triId];

                PxVec3 pointA, pointB, separatingAxis;
                PxReal separation = PX_MAX_REAL;

                TetrahedronSupport tetS(mVertices.at(tet[0]), mVertices.at(tet[1]), mVertices.at(tet[2]), mVertices.at(tet[3]), 0.0f);
                TriangleSupport triS(triPoints[tri[0]], triPoints[tri[1]], triPoints[tri[2]], 0.0f);

                PxGjkQuery::proximityInfo(tetS, triS, PxTransform(PxIdentity), PxTransform(PxIdentity), overlapOffset, 0.0f, pointA, pointB, separatingAxis, separation);
                if (separation <= overlapOffset)
                {
                    if (!triId_contained)
                    {
                        triIds.pushBack(triId);
                        triSet.insert(triId);
                    }

                    if (!tetId_contained)
                    {
                        tetIds.pushBack(tetId);
                        tetSet.insert(tetId);
                    }
                }
            }
        }

        // interface functions

        uint64_t createTetFinder(const carb::Float3* points, const uint32_t pointsSize, const uint32_t pointsByteStride, const uint32_t* indices, const uint32_t indicesSize)
        {
            CARB_ASSERT(indicesSize % 4 == 0);
            PxTypedStridedData<const PxVec3> stridedData(reinterpret_cast<const PxVec3*>(points), pointsByteStride);
            TetFinder* tetFinder = ICE_NEW(TetFinder)(stridedData, pointsSize, (const PxU32*)indices, indicesSize / 4);
            return uint64_t(tetFinder);
        }

        uint64_t createTetFinder(const carb::Float3* points, const uint32_t pointsSize, const uint32_t* indices, const uint32_t indicesSize)
        {
            return createTetFinder(points, pointsSize, sizeof(carb::Float3), indices, indicesSize);
        }

        uint64_t createTetFinder(const carb::Float4* points, const uint32_t pointsSize, const uint32_t* indices, const uint32_t indicesSize)
        {
            return createTetFinder(reinterpret_cast<const carb::Float3*>(points), pointsSize, sizeof(carb::Float4), indices, indicesSize);
        }

        void releaseTetFinder(const uint64_t tetFinder)
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);
            SAFE_DELETE_SINGLE(tf);
        }

        /*
         * returns -1 entries in tetIds for points that are outside of tetrahedra
         */
        void pointsToTetMeshLocal(int32_t* tetIds, carb::Float4* bary, const uint64_t tetFinder, const carb::Float3* points, const uint32_t pointsSize)
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);

            for (uint32_t p = 0; p < pointsSize; ++p)
            {
                int32_t& dstTetId = tetIds[p];
                PxVec4& dstBary = (PxVec4&)bary[p];
                const PxVec3& srcPoint = (const PxVec3&)points[p];
                dstTetId = tf->findTet(srcPoint, dstBary);
            }
        }

        /*
         * returns closest tet for all points and closest point on that tet as barycentric coordinate. 
         */
        bool pointsToTetMeshLocalClosest(int32_t* tetIds, carb::Float4* barys, carb::Float3* distanceDirs, const uint64_t tetFinder, const carb::Float3* points, const uint32_t pointsSize)
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);
            return tf->findClosestTets(reinterpret_cast<PxU32*>(tetIds),
                                       reinterpret_cast<PxVec4*>(barys),
                                       reinterpret_cast<PxVec3*>(distanceDirs),
                                       reinterpret_cast<const PxVec3*>(points),
                                       pointsSize);
        }

        /*
         * returns tet for all points even for those outside of the tetrahedra.
         */
        bool pointsToTetMeshLocalAll(int32_t* tetIds, carb::Float4* barys, const uint64_t tetFinder, const carb::Float3* points, const uint32_t pointsSize)
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);
            return tf->findAllTets(reinterpret_cast<PxU32*>(tetIds),
                reinterpret_cast<PxVec4*>(barys),
                reinterpret_cast<const PxVec3*>(points),
                pointsSize);
        }

        void tetMeshLocalToPoints(carb::Float3* points, const uint64_t tetFinder, const int32_t* tetIds, const carb::Float4* barycentricCoords, const uint32_t pointsSize)
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);
            for (uint32_t p = 0; p < pointsSize; ++p)
            {
                const int32_t srcTetId = tetIds[p];
                if (srcTetId >= 0)
                {
                    PxVec3& dstPoint = (PxVec3&)points[p];
                    const PxVec4& srcBary = (const PxVec4&)barycentricCoords[p];
                    dstPoint = tf->getPoint(srcTetId, srcBary);
                }
            }
        }

        void overlapTetMeshGeom(int32_t*& tetIds, uint32_t& tetIdsSize, const uint64_t tetFinder, const PxGeometry& geom, const PxTransform& geomPos, const float overlapOffset, void* (*allocateBytes)(size_t), void* triMeshSampler)
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);
            PxArray<ProximityInfo> tmpInfo;
            tf->overlap(tmpInfo, geom, geomPos, overlapOffset, (PxTriangleMeshPoissonSampler*)triMeshSampler);

            tetIdsSize = uint32_t(tmpInfo.size());
            tetIds = (int32_t*)allocateBytes(sizeof(uint32_t) * tetIdsSize);

            for (uint32_t i = 0; i < tetIdsSize; i++)
            {
                tetIds[i] = tmpInfo[i].tetId;
            }
        }

        void overlapTetMeshSphere(int32_t*& tetIds, uint32_t& tetIdsSize, const uint64_t tetFinder, const carb::Float3& center, const float radius, void* (*allocateBytes)(size_t))
        {
            const PxSphereGeometry& geom = PxSphereGeometry(radius);
            const PxTransform& geomPos = PxTransform(physx::toPhysX(center));

            overlapTetMeshGeom(tetIds, tetIdsSize, tetFinder, geom, geomPos, 0.0f, allocateBytes);
        }

        void overlapTetMeshCapsule(int32_t*& tetIds, uint32_t& tetIdsSize, const uint64_t tetFinder, const carb::Float3& pos, const carb::Float3& axis, const float radius, const float halfHeight, void* (*allocateBytes)(size_t))
        {
            const PxCapsuleGeometry& geom = PxCapsuleGeometry(radius, halfHeight);
            const PxQuat& quat = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), physx::toPhysX(axis));
            const PxTransform& geomPos = PxTransform(physx::toPhysX(pos), quat);

            overlapTetMeshGeom(tetIds, tetIdsSize, tetFinder, geom, geomPos, 0.0f, allocateBytes);
        }

        void overlapTetMeshTetMeshDeprecated(carb::Int2*& tetIdPairs, uint32_t& tetIdPairsSize, const uint64_t tetFinder0, const uint64_t tetFinder1, const float overlapOffset, const bool reportPairs, void* (*allocateBytes)(size_t))
        {
            TetFinder* tf0 = reinterpret_cast<TetFinder*>(tetFinder0);
            TetFinder* tf1 = reinterpret_cast<TetFinder*>(tetFinder1);

            PxArray<PxPair<PxI32, PxI32>> tmpTetIds = tf0->overlapDeprecated(*tf1, overlapOffset, reportPairs);

            tetIdPairsSize = uint32_t(tmpTetIds.size());
            tetIdPairs = (carb::Int2*)allocateBytes(sizeof(carb::Int2)*tetIdPairsSize);
            memcpy(tetIdPairs, &tmpTetIds[0], sizeof(carb::Int2)*tetIdPairsSize);
        }

        void overlapTetMeshTriMeshDeprecated(carb::Int2*& tetTriIdPairs, uint32_t& tetTriIdPairsSize, const uint64_t tetFinder,
            const carb::Float3* triPoints, const uint32_t* triIndices, const uint32_t triIndicesSize,
            const float overlapOffset, const bool reportPairs, void* (*allocateBytes)(size_t))
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);

            PxArray<PxPair<PxI32, PxI32>> tmpTetTriIds = tf->overlapDeprecated(reinterpret_cast<const PxVec3*>(triPoints), triIndices, triIndicesSize, overlapOffset, reportPairs);

            tetTriIdPairsSize = uint32_t(tmpTetTriIds.size());
            tetTriIdPairs = (carb::Int2*)allocateBytes(sizeof(carb::Int2)*tetTriIdPairsSize);
            memcpy(tetTriIdPairs, &tmpTetTriIds[0], sizeof(carb::Int2)*tetTriIdPairsSize);
        }

        void overlapTetMeshTetMeshPairs(carb::Int2*& tetIdPairs, uint32_t& tetIdPairsSize,
            const uint64_t tetFinder0, const uint64_t tetFinder1, const float overlapOffset, void* (*allocateBytes)(size_t))
        {
            const TetFinder* tf0 = reinterpret_cast<const TetFinder*>(tetFinder0);
            const TetFinder* tf1 = reinterpret_cast<const TetFinder*>(tetFinder1);

            PxArray<PxPair<PxI32, PxI32>> tmpTetIdPairs;
            tf0->overlapPairs(tmpTetIdPairs, *tf1, overlapOffset);

            tetIdPairsSize = uint32_t(tmpTetIdPairs.size());
            tetIdPairs = (carb::Int2*)allocateBytes(sizeof(carb::Int2)*tetIdPairsSize);
            memcpy(tetIdPairs, tmpTetIdPairs.begin(), sizeof(carb::Int2)*tetIdPairsSize);
        }

        void overlapTetMeshTetMeshAny(int32_t*& tetIds0, uint32_t& tetIds0Size, int32_t*& tetIds1, uint32_t& tetIds1Size,
            const uint64_t tetFinder0, const uint64_t tetFinder1, const float overlapOffset, void* (*allocateBytes)(size_t))
        {
            TetFinder* tf0 = reinterpret_cast<TetFinder*>(tetFinder0);
            TetFinder* tf1 = reinterpret_cast<TetFinder*>(tetFinder1);

            PxArray<PxI32> tmpTetIds0;
            PxArray<PxI32> tmpTetIds1;
            tf0->overlapAny(tmpTetIds0, tmpTetIds1, *tf1, overlapOffset);

            tetIds0Size = uint32_t(tmpTetIds0.size());
            tetIds0 = (int32_t*)allocateBytes(sizeof(int32_t) * tetIds0Size);
            memcpy(tetIds0, tmpTetIds0.begin(), sizeof(int32_t) * tetIds0Size);

            tetIds1Size = uint32_t(tmpTetIds1.size());
            tetIds1 = (int32_t*)allocateBytes(sizeof(int32_t) * tetIds1Size);
            memcpy(tetIds1, tmpTetIds1.begin(), sizeof(int32_t) * tetIds1Size);
        }

        void overlapTetMeshTriMeshPairs(carb::Int2*& tetTriIdPairs, uint32_t& tetTriIdPairsSize,
            const uint64_t tetFinder, const carb::Float3* triPoints, const uint32_t* triIndices, const uint32_t triIndicesSize,
            const float overlapOffset, void* (*allocateBytes)(size_t))
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);

            PxArray<PxPair<PxI32, PxI32>> tmpTetTriIds;
            tf->overlapPairs(tmpTetTriIds, reinterpret_cast<const PxVec3*>(triPoints), triIndices, triIndicesSize, overlapOffset);

            tetTriIdPairsSize = uint32_t(tmpTetTriIds.size());
            tetTriIdPairs = (carb::Int2*)allocateBytes(sizeof(carb::Int2)*tetTriIdPairsSize);
            memcpy(tetTriIdPairs, tmpTetTriIds.begin(), sizeof(carb::Int2)*tetTriIdPairsSize);
        }

        void overlapTetMeshTriMeshAny(int32_t*& tetIds, uint32_t& tetIdsSize, int32_t*& triIds, uint32_t& triIdsSize,
            const uint64_t tetFinder, const carb::Float3* triPoints, const uint32_t* triIndices, const uint32_t triIndicesSize,
            const float overlapOffset, void* (*allocateBytes)(size_t))
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);

            PxArray<PxI32> tmpTetIds;
            PxArray<PxI32> tmpTriIds;
            tf->overlapAny(tmpTetIds, tmpTriIds, reinterpret_cast<const PxVec3*>(triPoints), triIndices, triIndicesSize, overlapOffset);

            tetIdsSize = uint32_t(tmpTetIds.size());
            tetIds = (int32_t*)allocateBytes(sizeof(int32_t)*tetIdsSize);
            memcpy(tetIds, tmpTetIds.begin(), sizeof(int32_t)*tetIdsSize);

            triIdsSize = uint32_t(tmpTriIds.size());
            triIds = (int32_t*)allocateBytes(sizeof(int32_t) * triIdsSize);
            memcpy(triIds, tmpTriIds.begin(), sizeof(int32_t) * triIdsSize);
        }

        void getClosestPoints(carb::Float3* closestPoints, float* dists, const carb::Float3* points, const uint32_t pointsSize, const pxr::SdfPath& rigidPath)
        {
            PhysxShapeDesc* shapeDesc = usdparser::parseCollision(OmniPhysX::getInstance().getStageId(), rigidPath, rigidPath);
            if (!shapeDesc)
            {
                return;
            }

            pxr::UsdPrim rigidPrim = OmniPhysX::getInstance().getStage()->GetPrimAtPath(rigidPath);
            GfMatrix4d mat = UsdGeomXformable(rigidPrim).ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
            PxTransform transform = toPhysX(mat);

            for (size_t i = 0; i < pointsSize; ++i)
            {
                carb::Float3& closestPoint = closestPoints[i];
                float& dist = dists[i];
                const carb::Float3& point = points[i];
                PxVec3 cp;
                switch (shapeDesc->type)
                {
                case eSphereShape:
                {
                    SpherePhysxShapeDesc* sphereDesc = (SpherePhysxShapeDesc*)shapeDesc;
                    PxSphereGeometry sphereGeom(sphereDesc->radius);
                    dist = PxGeometryQuery::pointDistance(toPhysX(point), sphereGeom, transform, &cp);
                }
                break;
                case eBoxShape:
                {
                    BoxPhysxShapeDesc* boxDesc = (BoxPhysxShapeDesc*)shapeDesc;
                    PxBoxGeometry boxGeom((const PxVec3&)boxDesc->halfExtents);
                    dist = PxGeometryQuery::pointDistance(toPhysX(point), boxGeom, transform, &cp);
                }
                break;
                case eCapsuleShape:
                {
                    CapsulePhysxShapeDesc* capsuleDesc = (CapsulePhysxShapeDesc*)shapeDesc;

                    const float hRt2 = sqrt(2.0f) / 2.0f;
                    PxQuat fixupQ(PxIdentity);
                    if (capsuleDesc->axis == eY)
                    {
                        fixupQ = PxQuat(hRt2, -hRt2, 0.0f, 0.0f);
                    }
                    else if (capsuleDesc->axis == eZ)
                    {
                        fixupQ = PxQuat(hRt2, 0.0f, -hRt2, 0.0f);
                    }
                    transform.q = transform.q * fixupQ;

                    PxCapsuleGeometry capsuleGeom(capsuleDesc->radius, capsuleDesc->halfHeight);
                    dist = PxGeometryQuery::pointDistance(toPhysX(point), capsuleGeom, transform, &cp);
                }
                break;
                case ePlaneShape:
                {
                    PlanePhysxShapeDesc* planeDesc = (PlanePhysxShapeDesc*)shapeDesc;

                    const float hRt2 = sqrt(2.0f) / 2.0f;
                    PxQuat fixupQ(PxIdentity);
                    if (planeDesc->axis == eY)
                    {
                        fixupQ = PxQuat(hRt2, hRt2, 0.0f, 0.0f);
                    }
                    else if (planeDesc->axis == eZ)
                    {
                        fixupQ = PxQuat(hRt2, 0.0f, hRt2, 0.0f);
                    }
                    transform.q = transform.q * fixupQ;

                    PxPlaneGeometry planeGeom;
                    dist = PxGeometryQuery::pointDistance(toPhysX(point), planeGeom, transform, &cp);
                }
                break;
                case eCylinderShape:
                {
                    CylinderPhysxShapeDesc* cylinderDesc = (CylinderPhysxShapeDesc*)shapeDesc;
                    PxConvexMesh* convexMesh = OmniPhysX::getInstance().getPhysXSetup().getCylinderConvexMesh(cylinderDesc->axis);
                    if (convexMesh)
                    {
                        const PxVec3 scale = getConeOrCylinderScale(cylinderDesc->halfHeight, cylinderDesc->radius, cylinderDesc->axis);
                        PxConvexMeshGeometry convexMeshGeom(convexMesh, scale);
                        dist = PxGeometryQuery::pointDistance(toPhysX(point), convexMeshGeom, transform, &cp);
                    }
                }
                break;
                case eConeShape:
                {
                    ConePhysxShapeDesc* coneDesc = (ConePhysxShapeDesc*)shapeDesc;
                    PxConvexMesh* convexMesh = OmniPhysX::getInstance().getPhysXSetup().getConeConvexMesh(coneDesc->axis);
                    if (convexMesh)
                    {
                        const PxVec3 scale = getConeOrCylinderScale(coneDesc->halfHeight, coneDesc->radius, coneDesc->axis);
                        PxConvexMeshGeometry convexMeshGeom(convexMesh, scale);
                        dist = PxGeometryQuery::pointDistance(toPhysX(point), convexMeshGeom, transform, &cp);
                    }
                }
                break;
                case eConvexMeshShape:
                {
                    ConvexMeshPhysxShapeDesc* convexDesc = (ConvexMeshPhysxShapeDesc*)shapeDesc;
                    cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
                    CARB_ASSERT(cookingDataAsync);
                    PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(*convexDesc, rigidPrim, false);
                    if (convexMesh)
                    {
                        PxConvexMeshGeometry convexMeshGeom(convexMesh, toPhysX(convexDesc->meshScale));
                        dist = PxGeometryQuery::pointDistance(toPhysX(point), convexMeshGeom, transform, &cp);
                    }
                }
                break;
                case eConvexMeshDecompositionShape:
                {
                    ConvexMeshDecompositionPhysxShapeDesc* convexDecompositionDesc = (ConvexMeshDecompositionPhysxShapeDesc*)shapeDesc;
                    cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
                    CARB_ASSERT(cookingDataAsync);
                    std::vector<PxConvexMesh*> convexMeshes = cookingDataAsync->getConvexMeshDecomposition(*convexDecompositionDesc, rigidPrim, false);
                    if (!convexMeshes.empty())
                    {
                        const PxVec3 scale(fabsf(convexDecompositionDesc->meshScale.x), fabsf(convexDecompositionDesc->meshScale.y), fabsf(convexDecompositionDesc->meshScale.z));
                        for (size_t j = 0; j < convexMeshes.size(); j++)
                        {
                            PxConvexMeshGeometry convexMeshGeom(convexMeshes[j], scale);
                            dist = PxGeometryQuery::pointDistance(toPhysX(point), convexMeshGeom, transform, &cp);
                        }
                    }
                }
                break;
                case eTriangleMeshShape:
                {
                    TriangleMeshPhysxShapeDesc* meshDesc = (TriangleMeshPhysxShapeDesc*)shapeDesc;
                    cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
                    CARB_ASSERT(cookingDataAsync);
                    PxTriangleMesh* triMesh = cookingDataAsync->getTriangleMesh(*meshDesc, rigidPrim, false);
                    if (triMesh)
                    {
                        PxTriangleMeshGeometry triangleMeshGeom(triMesh, toPhysX(meshDesc->meshScale));
                        dist = PxGeometryQuery::pointDistance(toPhysX(point), triangleMeshGeom, transform, &cp);
                    }
                }
                break;
                }

                closestPoint = fromPhysX(cp);
            }
        }

        const uint32_t* getIndices(uint32_t& indicesSize, const uint64_t tetFinder)
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);
            indicesSize = tf->mNumTets*4;
            return tf->mTets;
        }

        const carb::Float3* getPoints(uint32_t& pointsSize, uint32_t& pointsByteStride, const uint64_t tetFinder)
        {
            TetFinder* tf = reinterpret_cast<TetFinder*>(tetFinder);
            pointsSize = tf->mNumVertices;
            pointsByteStride = tf->mVertices.stride;
            return reinterpret_cast<const carb::Float3*>(&tf->mVertices.at(0));
        }

        bool tetMeshVtxToTet(uint32_t*& vtxToTetIndices, uint32_t& vtxToTetIndicesSize, uint32_t* vtxToTetCounts, uint32_t* vtxToTetOffsets,
            const uint32_t* tetVtxIndices, const uint32_t numTets, const uint32_t numVerts, void* (*allocateBytes)(size_t))
        {
            bool inputError = false;
            // count tets per vertex
            std::memset(vtxToTetCounts, 0, sizeof(uint32_t) * numVerts);
            for (uint32_t t = 0; t < numTets; ++t)
            {
                const uint32_t* tet = (tetVtxIndices + t * 4);
                for (uint32_t v = 0; v < 4; ++v)
                {
                    uint32_t vtxIndex = tet[v];
                    if (vtxIndex < numVerts)
                    {
                        vtxToTetCounts[vtxIndex]++;
                    }
                    else
                    {
                        inputError = true;
                    }
                }
            }
            if (inputError)
            {
                return false;
            }

            // count all tet refs and create offset table
            uint32_t totalOffset = 0;
            for (uint32_t v = 0; v < numVerts; ++v)
            {
                vtxToTetOffsets[v] = totalOffset;
                totalOffset += vtxToTetCounts[v];
                vtxToTetCounts[v] = 0;
            }

            // store tet indices per vertex
            vtxToTetIndices = reinterpret_cast<uint32_t*>(allocateBytes(sizeof(uint32_t) * totalOffset));
            if (!vtxToTetIndices)
            {
                return false;
            }
            vtxToTetIndicesSize = totalOffset;
            for (uint32_t t = 0; t < numTets; ++t)
            {
                const uint32_t* tet = (tetVtxIndices + t * 4);
                for (uint32_t v = 0; v < 4; ++v)
                {
                    uint32_t vtxIndex = tet[v];
                    uint32_t vtxTetIndex = vtxToTetOffsets[vtxIndex] + vtxToTetCounts[vtxIndex];
                    vtxToTetIndices[vtxTetIndex] = t;
                    vtxToTetCounts[vtxIndex]++;
                }
            }
            return true;
        }

        bool tetMeshTetToSurfaceTri(uint32_t*& tetToTriIndices, uint32_t& tetToTriIndicesSize, uint32_t* tetToTriCounts, uint32_t* tetToTriOffsets,
            const uint32_t numTets, const uint32_t* triToTetMap, const uint32_t numTris, void* (*allocateBytes)(size_t))
        {
            bool inputError = false;

            // count tris per tet
            std::memset(tetToTriCounts, 0, sizeof(uint32_t) * numTets);
            for (uint32_t t = 0; t < numTris; ++t)
            {
                const uint32_t tetIndex = triToTetMap[t];
                if (tetIndex < numTets)
                {
                    tetToTriCounts[tetIndex]++;
                }
                else
                {
                    inputError = true;
                }
            }
            if (inputError)
            {
                return false;
            }

            // count all tri refs and create offset table
            uint32_t totalOffset = 0;
            for (uint32_t v = 0; v < numTets; ++v)
            {
                tetToTriOffsets[v] = totalOffset;
                totalOffset += tetToTriCounts[v];
                tetToTriCounts[v] = 0;
            }

            // store tri indices per tet
            tetToTriIndices = reinterpret_cast<uint32_t*>(allocateBytes(sizeof(uint32_t) * totalOffset));
            if (!tetToTriIndices)
            {
                return false;
            }
            tetToTriIndicesSize = totalOffset;
            for (uint32_t t = 0; t < numTris; ++t)
            {
                const uint32_t tetIndex = triToTetMap[t];
                uint32_t tetTriIndex = tetToTriOffsets[tetIndex] + tetToTriCounts[tetIndex];
                tetToTriIndices[tetTriIndex] = t;
                tetToTriCounts[tetIndex]++;
            }
            return true;
        }

        bool tetMeshVtxToTetLocal(uint32_t* tetIndices, carb::Float4* tetBarycentrics, const uint32_t* vtxIndices, const uint32_t vtxIndicesSize,
            const uint32_t* meshTetVtxIndices, const uint32_t meshNumTets)
        {
            // setup sparse map from input vtx indices to the location they are stored in
            std::unordered_map<uint32_t, uint32_t> inputVtxMap;
            for (uint32_t v = 0; v < vtxIndicesSize; ++v)
            {
                inputVtxMap[vtxIndices[v]] = v;
            }

            // iterate over all tets as long as we found a tet for each input vertex
            for (uint32_t t = 0; t < meshNumTets && inputVtxMap.size(); ++t)
            {
                const uint32_t* tet = meshTetVtxIndices + t*4;
                for (uint32_t v = 0; v < 4; ++v)
                {
                    uint32_t vtxIndex = tet[v];
                    std::unordered_map<uint32_t, uint32_t>::iterator it = inputVtxMap.find(vtxIndex);
                    if (it != inputVtxMap.end())
                    {
                        tetIndices[it->second] = t;
                        inputVtxMap.erase(vtxIndex);
                    }
                }
            }

            if (inputVtxMap.size() > 0)
            {
                return false;
            }

            static const carb::Float4 sCoords[4] =
            {
                { 1.0f, 0.0f, 0.0f, 0.0f},
                { 0.0f, 1.0f, 0.0f, 0.0f},
                { 0.0f, 0.0f, 1.0f, 0.0f},
                { 0.0f, 0.0f, 0.0f, 1.0f}
            };

            // compute barycentric cordinates
            for (uint32_t v = 0; v < vtxIndicesSize; ++v)
            {
                uint32_t vtxIndex = vtxIndices[v];
                uint32_t tetIndex = tetIndices[v];
                const uint32_t* tet = meshTetVtxIndices + tetIndex * 4;
                for (uint32_t i = 0; i < 4; ++i)
                {
                    if (vtxIndex == tet[i])
                    {
                        tetBarycentrics[v] = sCoords[i];
                        break;
                    }
                }
            }

            return true;
        }

    } // namespace tetfinder
}
