// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved. 

#include "foundation/PxBounds3.h"
#include "foundation/PxMath.h"
#include "foundation/PxPlane.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"

#include "geometry/PxGeometry.h"

#include "PxgConvexConvexShape.h"
#include "PxgContactManager.h"
#include "PxgFEMCloth.h"
#include "PxgSoftBodyCore.h"
#include "PxgNpKernelIndices.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgSoftBody.h"
#include "PxsTransformCache.h"

#include "assert.h"
#include <vector_types.h>

#include "GuBV32.h"
#include "GuDistancePointTriangle.h"
#include "GuIntersectionTetrahedronTetrahedron.h"

#include "PxgCommonDefines.h"
#include "dataReadWriteHelper.cuh"
#include "femMidphaseScratch.cuh"
#include "utils.cuh"
#include "nputils.cuh"
#include "deformableElementFilter.cuh"
#include "deformableCollision.cuh"
#include "cudaNpCommon.h"

#include "bv32Traversal.cuh"
#include "triangleMesh.cuh"
#include "sdfCollision.cuh"
#include "reduction.cuh"

using namespace physx;
using namespace Gu;

extern "C" __host__ void initNarrowphaseKernels15() {}

__device__ PxReal minProject(const PxPlane& plane, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2)
{
	return PxMin(plane.distance(v0), PxMin(plane.distance(v1),
		plane.distance(v2)));
}

template <bool BackFaceCull>
__device__ PxReal satIntersect(const Tetrahedron& tet0, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxReal tolerance)
{
	const PxPlane plane(v0, v1, v2);
	
	if (BackFaceCull)
	{
		if (plane.distance(tet0.centroid) < 0.f)
			return PX_MAX_F32;
	}
	
	PxReal sep = minProject(plane, v0, v1, v2);
	if (sep > tolerance)
		return sep;

	sep = PxMax(sep, minProject(tet0.planes[0], v0, v1, v2));
	if (sep > tolerance)
		return sep;
	sep = PxMax(sep, minProject(tet0.planes[1], v0, v1, v2));
	if (sep > tolerance)
		return sep;
	sep = PxMax(sep, minProject(tet0.planes[2], v0, v1, v2));
	if (sep > tolerance)
		return sep;
	sep = PxMax(sep, minProject(tet0.planes[3], v0, v1, v2));
	if (sep > tolerance)
		return sep;

	return sep;
}


PX_INLINE PX_CUDA_CALLABLE PxReal minProjectABBB(const PxPlane& plane, const float4& min, const float4& max)
{
	return PxMin(min.x * plane.n.x, max.x * plane.n.x) + PxMin(min.y * plane.n.y, max.y * plane.n.y) + PxMin(min.z * plane.n.z, max.z * plane.n.z) + plane.d;
}


template <bool DoEdgeEdge>
struct TriangleLeafBoundMinMaxTraverser
{
	femMidphaseScratch* PX_RESTRICT s_warpScratch;
	Tetrahedron* mTet;
	const PxVec3 tMin;
	PxReal mBestDistance;
	const PxVec3 tMax;
	const PxReal mContactDistance;
	PxU32 mPrimIndex;

	PX_FORCE_INLINE __device__ TriangleLeafBoundMinMaxTraverser(
		femMidphaseScratch* PX_RESTRICT s_warpScratch,
		const PxBounds3 tetBound,
		const PxReal contactDistance, Tetrahedron* tet) :
		mContactDistance(contactDistance), mTet(tet), mBestDistance(PX_MAX_F32), mPrimIndex(0xFFFFFFFF), tMin(tetBound.minimum), tMax(tetBound.maximum),
		s_warpScratch(s_warpScratch)
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		if (primitiveIndex != 0xFFFFFFFF)
		{
			const float4 * PX_RESTRICT trimeshVerts = s_warpScratch->meshVerts;

			const uint4 triIdx = s_warpScratch->meshVertsIndices[primitiveIndex];

			const PxVec3 v0 = PxLoad3(trimeshVerts[triIdx.x]);
			const PxVec3 v1 = PxLoad3(trimeshVerts[triIdx.y]);
			const PxVec3 v2 = PxLoad3(trimeshVerts[triIdx.z]);

			PxPlane triPlane(v0, v1, v2);

			if (triPlane.distance(mTet->centroid) >= 0.f)
			{
				//KS - this version ot intersectTets is fully accurate 
				//intersect = intersectTets(tet0, tet1, 0.f);
				PxReal separation = satIntersect<DoEdgeEdge>(*mTet, v0, v1, v2, PxMin(mBestDistance, mContactDistance));

				if (separation < mBestDistance)
				{
					mBestDistance = separation;
					mPrimIndex = primitiveIndex;
				}
			}
		}

		//return mBestDistance < mContactDistance;
	}


	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox) {
			PxVec3 displacement;
			if (min.x > tMax.x)
			{
				displacement.x = min.x - tMax.x;
			}
			else if (max.x < tMin.x)
			{
				displacement.x = max.x - tMin.x;
			}
			else
				displacement.x = 0.f;
			if (min.y > tMax.y)
			{
				displacement.y = min.y - tMax.y;
			}
			else if (max.y < tMin.y)
			{
				displacement.y = max.y - tMin.y;
			}
			else
				displacement.y = 0.f;
			if (min.z > tMax.z)
			{
				displacement.z = min.z - tMax.z;
			}
			else if (max.z < tMin.z)
			{
				displacement.z = max.z - tMin.z;
			}
			else
				displacement.z = 0.f;

			return displacement.magnitudeSquared() < mContactDistance * mContactDistance;
		}
		else
			return false;
	}
};

static inline __device__ bool isPointOutsideOfPlane4(const PxVec3& p, const PxVec3& _a, const PxVec3& _b, const PxVec3& _c, const PxVec3& _d)
{
	PxVec4 r = PointOutsideOfPlane4(p, _a, _b, _c, _d);
	return ((r.x >= 0.f) && (r.y >= 0.f) && (r.z >= 0.f) && (r.w >= 0.f));
}

__device__ static void closestPtTetrahedron(const PxVec3& point, const PxU8 hint,
	const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d, PxVec3& normal,
	PxVec3& closestP, PxReal& sqDist)
{
	const PxVec3 ac = (c - a);
	const PxVec3 ab = (b - a);
	const PxVec3 ad = (d - a);
	

	if (hint & 1) //0111
	{
		

		const PxVec3 tPoint = closestPtPointTriangle2(point, a, b, c, ab, ac);
		const PxVec3 v = tPoint - point;
		const PxReal tSqDist = v.magnitudeSquared();
		if (tSqDist < sqDist)
		{
			normal = (ac.cross(ab)).getNormalized();
			if (ad.dot(normal) < 0.f)
				normal = -normal;
			//normal = v.getNormalized();

			/*const PxReal signDist = normal.dot(v);
			if (signDist < 0.f)
				normal = -normal;*/

			closestP = tPoint;
			sqDist = tSqDist;
		}
	}

	if (hint & 2)//1011
	{
		const PxVec3 tPoint = closestPtPointTriangle2(point, a, b, d, ab, ad);
		const PxVec3 v = tPoint - point;
		const PxReal tSqDist = v.magnitudeSquared();
		if (tSqDist < sqDist)
		{
			normal = (ab.cross(ad)).getNormalized();
			if (ac.dot(normal) < 0.f)
				normal = -normal;
			//normal = v.getNormalized();

			/*const PxReal signDist = normal.dot(v);
			if (signDist < 0.f)
				normal = -normal;*/

			closestP = tPoint;
			sqDist = tSqDist;
		}
	}

	if (hint & 4)//1101
	{
		const PxVec3 tPoint = closestPtPointTriangle2(point, a, c, d, ac, ad);
		const PxVec3 v = tPoint - point;
		const PxReal tSqDist = v.magnitudeSquared();
		if (tSqDist < sqDist)
		{
			normal = (ac.cross(ad)).getNormalized();
			if (ab.dot(normal) < 0.f)
				normal = -normal;
			//normal = v.getNormalized();

			/*const PxReal signDist = normal.dot(v);
			if (signDist < 0.f)
				normal = -normal;*/

			closestP = tPoint;
			sqDist = tSqDist;
		}
	}

	if (hint & 8)//1110
	{

		const PxVec3 bd = (d - b);
		const PxVec3 bc = (c - b);
		const PxVec3 tPoint = closestPtPointTriangle2(point, b, c, d, bc, bd);
		const PxVec3 v = tPoint - point;
		const PxReal tSqDist = v.magnitudeSquared();
		if (tSqDist < sqDist)
		{
			normal = (bc.cross(bd)).getNormalized();
			if (ab.dot(normal) > 0.f)
				normal = -normal;
			//normal = v.getNormalized();

			/*const PxReal signDist = normal.dot(v);
			if (signDist < 0.f)
				normal = -normal;*/

			closestP = tPoint;
			sqDist = tSqDist;
		}
	}
}


struct SbMeshTreeTraverser
{
	femMidphaseScratch* PX_RESTRICT		s_warpScratch;
	const PxU32							softbodyId;
	const PxVec3&						point;
	const PxVec3&						restP;
	const PxU32							vertIdx;
	const PxReal						distance;
	const PxU32*						vertToSuraceTetRemap;
	const PxU8*							surfaceTetHint;
	const float4*						restTetVerts;
	PxgSoftBodyContactWriter&			writer;

	const PxReal*						tetraStresses;
	const PxReal filterDistanceSq;
	const PxReal						selfCollisionStressTolerance;
	const PxgNonRigidFilterPair*		pairs;
	const PxU32							nbPairs;	
	PxReal bestDist = PX_MAX_F32;
	PxVec3 bestClosestP;
	PxVec3 bestNormal;
	PxU32 bestPrimIndex = 0;

	PX_FORCE_INLINE __device__ SbMeshTreeTraverser(
		femMidphaseScratch* PX_RESTRICT	s_warpScratch,
		const PxU32							softbodyId,
		const PxVec3&						point,
		const PxVec3&						restP,
		const PxU32							vertIdx,
		const PxReal						distance,
		const PxU32*						vertToSuraceTetRemap,
		const PxU8*							surfaceTetHint,
		const float4*						restTetVerts,
		PxgSoftBodyContactWriter&			writer,
		const PxReal*						tetraStresses,
		const PxReal						selfCollisionFilterDistance,
		const PxReal						selfCollisionStressTolerance,
		const PxgNonRigidFilterPair*		pairs,
		const PxU32							nbPairs
	) : s_warpScratch(s_warpScratch), softbodyId(softbodyId), point(point), restP(restP), vertIdx(vertIdx),
		distance(distance), vertToSuraceTetRemap(vertToSuraceTetRemap), surfaceTetHint(surfaceTetHint), restTetVerts(restTetVerts), writer(writer),		
		tetraStresses(tetraStresses), filterDistanceSq(selfCollisionFilterDistance*selfCollisionFilterDistance), selfCollisionStressTolerance(selfCollisionStressTolerance),
		pairs(pairs), nbPairs(nbPairs)
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		if (primitiveIndex != 0xFFFFFFFF)
		{
			bool bHasCollision = false;
			bool inside = false;

			PxReal dist = PX_MAX_F32;
			PxVec3 closestP;
			PxVec3 normal;

			const uint4 tetIdx1 = s_warpScratch->meshVertsIndices[primitiveIndex];

			const PxU8 hint = surfaceTetHint[primitiveIndex];

			//don't generate contacts with neighbour vert
			if (hint && (!((tetIdx1.x == vertIdx) || (tetIdx1.y == vertIdx) || (tetIdx1.z == vertIdx) || (tetIdx1.w == vertIdx))))
			{
				const PxU32 softBodyMask0 = PxEncodeSoftBodyIndex(softbodyId, vertIdx);
				const PxU32 softBodyMask1 = PxEncodeSoftBodyIndex(softbodyId, primitiveIndex);
				//If we found the tetrahedron index in the attachment list, we don't need to generate contacts between the rigid
				//body and the tetrahedron
				const bool filterPairs = find(pairs, nbPairs, softBodyMask0, softBodyMask1);
				if (!filterPairs)
				{

					PxVec3 a = PxLoad3(s_warpScratch->meshVerts[tetIdx1.x]);
					PxVec3 b = PxLoad3(s_warpScratch->meshVerts[tetIdx1.y]);
					PxVec3 c = PxLoad3(s_warpScratch->meshVerts[tetIdx1.z]);
					PxVec3 d = PxLoad3(s_warpScratch->meshVerts[tetIdx1.w]);

					if (isValidTet(a, b, c, d))
					{

						inside = isPointOutsideOfPlane4(point, a, b, c, d);

						PxReal sqDist2 = PX_MAX_F32;
						closestPtTetrahedron(point, hint, a, b, c, d, normal,
							closestP, sqDist2);

						PxVec3 v = (point - closestP);
						normal = -normal;

						if (inside)
						{
							bHasCollision = true;
							dist = v.dot(normal);
						}
						else if (v.dot(normal) > 0.f)
						{
							dist = v.normalize();

							if (normal.dot(v) > 0.707f)
								normal = v;
							bHasCollision = dist < distance;
						}


						if (bHasCollision)
						{
							//dist = PxSqrt(sqDist2);
							a = PxLoad3(restTetVerts[tetIdx1.x]);
							b = PxLoad3(restTetVerts[tetIdx1.y]);
							c = PxLoad3(restTetVerts[tetIdx1.z]);
							d = PxLoad3(restTetVerts[tetIdx1.w]);

							PxVec3 newNorm, newClosestP;
							PxReal newSqDist = PX_MAX_F32;

							bool inside2 = isPointOutsideOfPlane4(restP, a, b, c, d);

							closestPtTetrahedron(restP, 0xf, a, b, c, d, newNorm,
								newClosestP, newSqDist);

							//printf("sqDist = %f, NewSqDist = %f, distanceSq = %f\n", sqDist, newSqDist, distanceSq);

							bHasCollision = !inside2 && newSqDist > filterDistanceSq;

							if (bHasCollision)
							{

								const PxReal stress = tetraStresses[primitiveIndex];
								//printf("stress %f\n", stress);

								if (stress > selfCollisionStressTolerance)
									bHasCollision = false;

							}
						}
					}
				}
			}

			if (bHasCollision && dist < bestDist)
			{
				bestDist = dist;
				bestNormal = normal;
				bestClosestP = closestP;
				bestPrimIndex = primitiveIndex;
			}
		}
		//return bHasCollision;
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox) 
		{
			PxVec3 closest;
			closest.x = PxMax(min.x, PxMin(max.x, point.x));
			closest.y = PxMax(min.y, PxMin(max.y, point.y));
			closest.z = PxMax(min.z, PxMin(max.z, point.z));

			if ((closest - point).magnitudeSquared() < distance * distance)
				return true;

			return false;
		}
		else
			return false;
	}

	PX_FORCE_INLINE __device__ void finalizeFullWarp()
	{
		const PxU32 idxInWarp = threadIdx.x & 31;

		PxReal sep;
		PxU32 winnerLane = minIndex(bestDist, FULL_MASK, sep);

		if (winnerLane == idxInWarp && bestDist < distance)
		{
			PxU32 index = atomicAdd(writer.totalContactCount, 1);

			const PxReal pen = bestDist;
			const PxU32 tetrahedronIdx0 = vertToSuraceTetRemap[vertIdx];
			PxU32 pairId0 = PxEncodeSoftBodyIndex(softbodyId, tetrahedronIdx0);
			PxU32 pairId1 = PxEncodeSoftBodyIndex(softbodyId, bestPrimIndex);

			writer.writeContact(index, make_float4(bestClosestP.x, bestClosestP.y, bestClosestP.z, 0.f), make_float4(bestNormal.x, bestNormal.y, bestNormal.z, pen),
				make_float4(point.x, point.y, point.z, 0.f), make_float4(bestClosestP.x, bestClosestP.y, bestClosestP.z, 0.f), pairId0, pairId1);				
		}
	}
};

struct SbSbMeshTreeTraverser
{
	femMidphaseScratch* PX_RESTRICT	s_warpScratch;
	const PxU32							softbodyId0;
	const PxU32							softbodyId1;
	const PxVec3&						point;
	const PxU32							vertIdx;
	const PxReal						distance;
	const PxU32*						vertToSurfaceTetRemap0;
	const PxU8*							surfaceTetHint1;
	PxgSoftBodyContactWriter&			writer;
	const PxgNonRigidFilterPair*		pairs;
	const PxU32							nbPairs;

	PxReal bestDist = PX_MAX_F32;
	PxVec3 bestClosestP;
	PxVec3 bestNormal;
	PxU32 bestPrimIndex = 0;

	PX_FORCE_INLINE __device__ SbSbMeshTreeTraverser(
		femMidphaseScratch* PX_RESTRICT	s_warpScratch,
		const PxU32							softbodyId0,
		const PxU32							softbodyId1,
		const PxVec3&						point,
		const PxU32							vertIdx,
		const PxReal						distance,
		const PxU32*						vertToSurfaceTetRemap0,
		const PxU8*							surfaceTetHint1,
		PxgSoftBodyContactWriter&			writer,
		const PxgNonRigidFilterPair*		pairs,
		const PxU32							nbPairs)
		: s_warpScratch(s_warpScratch), softbodyId0(softbodyId0), softbodyId1(softbodyId1), point(point),
		  vertIdx(vertIdx), distance(distance), vertToSurfaceTetRemap0(vertToSurfaceTetRemap0), surfaceTetHint1(surfaceTetHint1),
		  writer(writer), pairs(pairs), nbPairs(nbPairs)
	{ }

	PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primitiveIndex, PxU32 idxInWarp)
	{
		if (primitiveIndex != 0xFFFFFFFF)
		{
			bool bHasCollision = false;
			bool inside = false;

			PxReal dist = PX_MAX_F32;
			PxVec3 closestP;
			PxVec3 normal;

			const uint4 tetIdx1 = s_warpScratch->meshVertsIndices[primitiveIndex];

			const PxU8 hint = surfaceTetHint1[primitiveIndex];

			//generate contact if the tetrahedron is surface tet
			if (hint)
			{
				const PxU32 softBodyMask0 = PxEncodeSoftBodyIndex(softbodyId0, vertIdx);
				const PxU32 softBodyMask1 = PxEncodeSoftBodyIndex(softbodyId1, primitiveIndex);

				//If we found the tetrahedron index in the attachment list, we don't need to generate contacts between the rigid
				//body and the tetrahedron

				if (!find(pairs, nbPairs, softBodyMask0, softBodyMask1))
				{
					PxVec3 a = PxLoad3(s_warpScratch->meshVerts[tetIdx1.x]);
					PxVec3 b = PxLoad3(s_warpScratch->meshVerts[tetIdx1.y]);
					PxVec3 c = PxLoad3(s_warpScratch->meshVerts[tetIdx1.z]);
					PxVec3 d = PxLoad3(s_warpScratch->meshVerts[tetIdx1.w]);

					if (isValidTet(a, b, c, d))
					{

						inside = isPointOutsideOfPlane4(point, a, b, c, d);

						PxReal sqDist2 = PX_MAX_F32;
						closestPtTetrahedron(point, hint, a, b, c, d, normal,
							closestP, sqDist2);

						PxVec3 v = (point - closestP);


						if (inside)
						{

							normal = -normal;
							dist = v.dot(normal);
							bHasCollision = true;
						}
						else if (v.dot(normal) < 0.f)
						{
							dist = v.normalize();
							normal = v.dot(normal) < -0.707f ? v : -normal;
							bHasCollision = dist < distance;
						}
					}
				}
				/*else
					{
						printf("softbodyId0 %i vertIdx %i softbodyId1 %i, primitiveIdx %i\n", softbodyId0, vertIdx, softbodyId1, primitiveIdx);
					}*/
			}

			if (bHasCollision && dist < bestDist)
			{
				bestDist = dist;
				bestNormal = normal;
				bestClosestP = closestP;
				bestPrimIndex = primitiveIndex;
			}
		}
		//return bHasCollision;
	}

	PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
	{
		if (hasBox)
		{
			PxVec3 closest;
			closest.x = PxMax(min.x, PxMin(max.x, point.x));
			closest.y = PxMax(min.y, PxMin(max.y, point.y));
			closest.z = PxMax(min.z, PxMin(max.z, point.z));

			if ((closest - point).magnitudeSquared() < distance * distance)
				return true;

			return false;
		}
		else
			return false;
	}

	PX_FORCE_INLINE __device__ void finalizeFullWarp()
	{
		const PxU32 idxInWarp = threadIdx.x & 31;

		PxReal sep;
		PxU32 winnerLane = minIndex(bestDist, FULL_MASK, sep);

		if (winnerLane == idxInWarp && bestDist < distance)
		{
			PxU32 index = atomicAdd(writer.totalContactCount, 1);

			const PxReal pen = bestDist;
			const PxU32 tetrahedronIdx0 = vertToSurfaceTetRemap0[vertIdx];
			PxU32 pairId0 = PxEncodeSoftBodyIndex(softbodyId0, tetrahedronIdx0);
			PxU32 pairId1 = PxEncodeSoftBodyIndex(softbodyId1, bestPrimIndex);

			writer.writeContact(index, make_float4(bestClosestP.x, bestClosestP.y, bestClosestP.z, 0.f), make_float4(bestNormal.x, bestNormal.y, bestNormal.z, pen),
				make_float4(point.x, point.y, point.z, 0.f), make_float4(bestClosestP.x, bestClosestP.y, bestClosestP.z, 0.f), pairId0, pairId1);
		}
	}
};

__device__ bool computeTetBoundWithTetrahedron(const PxVec3& worldV0, const PxVec3& worldV1, const PxVec3& worldV2, const PxVec3& worldV3,
	PxBounds3& tetBound, Tetrahedron& tet)
{
	if (!isValidTet(worldV0, worldV1, worldV2, worldV3))
	{
		return false;
	}

	tetBound = tetBoundingBox(worldV0, worldV1, worldV2, worldV3);

	if (threadIdx.x == 0)
	{
		constructTetrahedron(worldV0, worldV1, worldV2, worldV3, tet);
	}
	__syncwarp();
	return true;
}

__device__ bool computeTetBoundWithTetrahedron(const uint4* const PX_RESTRICT tetIndices, const PxU32 tetrahedronIdx, const float4* const PX_RESTRICT tetVerts,
	PxBounds3& tetBound, Tetrahedron& tet)
{
	const uint4 tetIdx = tetIndices[tetrahedronIdx];

	const PxVec3 worldV0 = PxLoad3(tetVerts[tetIdx.x]);
	const PxVec3 worldV1 = PxLoad3(tetVerts[tetIdx.y]);
	const PxVec3 worldV2 = PxLoad3(tetVerts[tetIdx.z]);
	const PxVec3 worldV3 = PxLoad3(tetVerts[tetIdx.w]);

	return computeTetBoundWithTetrahedron(worldV0, worldV1, worldV2, worldV3, tetBound, tet);
}

template<unsigned int WarpsPerBlock>
__device__ static inline void sb_tetmeshMidphaseCore2(
	const PxReal								toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxBounds3* PX_RESTRICT				bounds,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	femMidphaseScratch*							s_warpScratch,
	PxgSoftBodyContactWriter&					writer
)
{

	//__shared__ Tetrahedron tets[WarpsPerBlock];

	const PxU32 cmIdx = blockIdx.y;

	//we do bidirectional collision(A to B and B to A). This is to ensure we don't miss contacts when we have a soft body with 
	//an insufficiently tessellated tetrahedron mesh.
	bool flip = blockIdx.z;
	//each block deal with one pair
	//if (cmIdx < numNPWorkItems)
	{
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x*blockDim.y;

		PxgContactManagerInput npWorkItem;
		PxgContactManagerInput_ReadWarp(npWorkItem, cmInputs, cmIdx);

		PxgShape shape0;
		PxgShape_ReadWarp(shape0, gpuShapes + (flip ? npWorkItem.shapeRef1 : npWorkItem.shapeRef0));

		PxgShape shape1;
		PxgShape_ReadWarp(shape1, gpuShapes + (flip ? npWorkItem.shapeRef0 : npWorkItem.shapeRef1));

		assert(shape0.type == PxGeometryType::eTETRAHEDRONMESH && shape1.type == PxGeometryType::eTETRAHEDRONMESH);

		/*const PxgSoftBody* softbody[2];

		softbody[0] = &softbodies[shape0.particleOrSoftbodyId];
		softbody[1] = &softbodies[shape1.particleOrSoftbodyId];*/

		const PxgSoftBody& softbody0 = softbodies[shape0.particleOrSoftbodyId];
		const PxgSoftBody& softbody1 = softbodies[shape1.particleOrSoftbodyId];

		const PxU32 idx = threadIdx.x;

		if (idx == 0)
		{
			s_warpScratch->meshVerts = softbody1.mPosition_InvMass;
			s_warpScratch->meshVertsIndices = softbody1.mTetIndices;

			PxU8* tetmeshGeomPtr = reinterpret_cast<PxU8 *>(softbody1.mTetMeshData);

			//const uint4 nbVerts_nbTets_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(tetmeshGeomPtr);
			tetmeshGeomPtr += sizeof(uint4);

			//s_warpScratch->nbPrimitives[idx] = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.y;

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(tetmeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		}

		__syncthreads();

		const PxReal contactDist = contactDistance[npWorkItem.transformCacheRef0] + contactDistance[npWorkItem.transformCacheRef1];

		const PxU32 nbVerts = softbody0.mNumVerts;// s_warpScratch->nbPrimitives[0];

		const PxU32 NbWarps = blockDim.y*gridDim.x;

		const PxU32* vertToSurfaceTetRemap0 = softbody0.mSurfaceVertToTetRemap;
		//const uint4* tetIndices = softbody0.mTetIndices; //s_warpScratch->tetmeshTetIndices[0];
		const float4* tetVerts = softbody0.mPosition_InvMass; //s_warpScratch->tetmeshVerts[0];
		//const PxU8* surfaceHint = softbody0.mTetMeshSurfaceHint; // s_warpScratch->tetmeshTetSurfaceHint[0];
		const PxU8*  surfaceVertsHint = softbody0.mSurfaceVertsHint;
		//const PxU32* surfaceRemapTetIds1 = softbody1.mTetRemapClosestSurfaceTets;

		const PxU8* surfaceTetHint1 = softbody1.mTetMeshSurfaceHint;

		const PxgNonRigidFilterPair* pairs = softbody0.mFilteringPairs;
		const PxU32 nbFilterPairs = softbody0.mNumFilterPairs;

		for (PxU32 vertIdx = globalWarpIdx; vertIdx < nbVerts; vertIdx += NbWarps)
		{
			//avoid generate contacts if the tetrahedron isn'sb_sbMeshTreeTraversalt a surface vert
			PxU8 hint = surfaceVertsHint[vertIdx];
		
			if (hint)
			{

				const float4 vertf = tetVerts[vertIdx];
				const PxVec3 point(vertf.x, vertf.y, vertf.z);

				const PxU32 softBodyMask0 = PxEncodeSoftBodyIndex(shape0.particleOrSoftbodyId, vertIdx);
				const PxU32 softBodyMask1 = PxEncodeSoftBodyIndex(shape1.particleOrSoftbodyId, PX_MAX_NB_DEFORMABLE_VOLUME_TET);
				//If we found the tetrahedron index in the filtering list, we don't need to generate contacts between the rigid
				//body and the tetrahedron
				const bool filterPairs = find(pairs, nbFilterPairs, softBodyMask0, softBodyMask1);
				
	
				if (!filterPairs)
				{
					SbSbMeshTreeTraverser traverser(
						s_warpScratch,
						shape0.particleOrSoftbodyId, //soft body0 id
						shape1.particleOrSoftbodyId, //soft body1 id
						point,
						vertIdx,
						contactDist,
						vertToSurfaceTetRemap0,
						surfaceTetHint1,
						writer,
						pairs,
						nbFilterPairs);
					bv32TreeTraversal<SbSbMeshTreeTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, traverser);
					traverser.finalizeFullWarp();					
				}
			}
		}
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void sb_sbMidphaseGeneratePairsLaunch(
	const PxReal								tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT	cmInputs,
	const PxsCachedTransform* PX_RESTRICT		transformCache,
	const PxBounds3* PX_RESTRICT				bounds,
	const PxReal* PX_RESTRICT					contactDistance,
	const PxgShape* PX_RESTRICT					gpuShapes,
	const PxgSoftBody* PX_RESTRICT				softbodies,
	PxgSoftBodyContactWriter					writer
)
{

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 18];

	sb_tetmeshMidphaseCore2<MIDPHASE_WARPS_PER_BLOCK>(
		tolerenceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		softbodies,
		(femMidphaseScratch*)scratchMem[threadIdx.y],
		writer
		);
}

template<unsigned int WarpsPerBlock>
__device__ static inline void sb_selfCollisionMidphaseCore2(
	const PxU32*					activeSoftbodies,
	const PxReal* PX_RESTRICT		contactDistance,
	const PxgSoftBody* PX_RESTRICT	softbodies,

	femMidphaseScratch*				s_warpScratch,
	PxgSoftBodyContactWriter&		writer
)
{
	const PxU32 cmIdx = blockIdx.y;

	const PxU32 softbodyId = activeSoftbodies[cmIdx];

	const PxgSoftBody* softbody = &softbodies[softbodyId];

	if (softbody->mBodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION || softbody->mBodyFlags & PxDeformableBodyFlag::eKINEMATIC)
		return;

	__shared__ Tetrahedron tets[WarpsPerBlock];

	const PxU32 numVerts = softbody->mNumVerts;

	const PxReal selfCollisionDistance = softbody->mSelfCollisionFilterDistance;
	const PxReal selfCollisionStressTolerance = softbody->mSelfCollisionStressTolerance;

	//each block deal with one pair
	//if (cmIdx < numTets)
	{

		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x*blockDim.y;

		const PxU32 idx = threadIdx.x;

		if (idx == 0)
		{
			
			s_warpScratch->meshVerts = softbody->mPosition_InvMass;
			s_warpScratch->meshVertsIndices = softbody->mTetIndices;
			//s_warpScratch->tetmeshTetSurfaceHint[idx] = softbody->mTetMeshSurfaceHint;

			PxU8* tetmeshGeomPtr = reinterpret_cast<PxU8 *>(softbody->mTetMeshData);

			//const uint4 nbVerts_nbTets_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(tetmeshGeomPtr);
			tetmeshGeomPtr += sizeof(uint4);

			//s_warpScratch->nbPrimitives[idx] = nbVerts_nbTets_maxDepth_nbBv32TreeNodes.y;

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(tetmeshGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		}

		__syncthreads();

		const PxU32 elementInd = softbody->mElementIndex;

		const PxReal contactDist = (contactDistance[elementInd]) * 2.f;
		

		//const PxU32 nbTets = s_warpScratch->nbPrimitives[1];

		const PxU32 NbWarps = blockDim.y*gridDim.x;

		//const uint4* tetIndices = s_warpScratch->meshVertsIndices;
		const float4* tetVerts = s_warpScratch->meshVerts;

		const PxU8*  surfaceVertsHint = softbody->mSurfaceVertsHint;
		const PxU32* vertToSurfaceTetRemap = softbody->mSurfaceVertToTetRemap;
		const float4* restTetVerts = softbody->mRestPosition;
		const PxReal* tetraStresses = softbody->mTetraStressCoefficient;

		const PxgNonRigidFilterPair* pairs = softbody->mFilteringPairs;
		const PxU32 nbFilterPairs = softbody->mNumFilterPairs;

		const PxU8* surfaceHint = softbody->mTetMeshSurfaceHint;

		for (PxU32 vertIdx = globalWarpIdx; vertIdx < numVerts; vertIdx += NbWarps)
		{
			//avoid generate contacts if the tetrahedron isn't a surface tetrahedron
			PxU8 hint = surfaceVertsHint[vertIdx];
			if (hint)
			{

				const PxU32 softBodyMask0 = PxEncodeSoftBodyIndex(softbodyId, vertIdx);
				const PxU32 softBodyMask1 = PxEncodeSoftBodyIndex(softbodyId, PX_MAX_NB_DEFORMABLE_VOLUME_TET);
				//If we found the tetrahedron index in the filtering list, we don't need to generate contacts between the rigid
				//body and the tetrahedron
				const bool filterPairs = find(pairs, nbFilterPairs, softBodyMask0, softBodyMask1);

				if (!filterPairs)
				{
					const float4 vertf = tetVerts[vertIdx];
					const float4 restf = restTetVerts[vertIdx];
					const PxVec3 point(vertf.x, vertf.y, vertf.z);
					const PxVec3 restP(restf.x, restf.y, restf.z);

					SbMeshTreeTraverser traverser(s_warpScratch,
						softbodyId,
						point,
						restP,
						vertIdx,
						contactDist,
						vertToSurfaceTetRemap,
						surfaceHint,
						restTetVerts,
						writer,
						tetraStresses,
						selfCollisionDistance,
						selfCollisionStressTolerance,
						pairs,
						nbFilterPairs);
					bv32TreeTraversal<SbMeshTreeTraverser, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, traverser);
					traverser.finalizeFullWarp();					
				}
			}
		}
	}
}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void sb_selfCollisionMidphaseGeneratePairsLaunch(
	const PxU32*					activeSoftbodies,
	const PxReal* PX_RESTRICT		contactDistance,
	const PxgSoftBody* PX_RESTRICT	softbodies,
	PxgSoftBodyContactWriter		writer
)
{
	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 18];

	sb_selfCollisionMidphaseCore2<MIDPHASE_WARPS_PER_BLOCK>(
		activeSoftbodies,
		contactDistance,
		softbodies,
		(femMidphaseScratch*)scratchMem[threadIdx.y],
		writer
		);
}


template<unsigned int WarpsPerBlock>
__device__ static inline void sb_trimeshMidphaseCore(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgSoftBody* PX_RESTRICT softbodies,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,									//output
	PxU32* PX_RESTRICT midphasePairsNum,						//output

	femMidphaseScratch*	s_warpScratch
)
{
	__shared__ Tetrahedron tets[WarpsPerBlock];

	const PxU32 cmIdx = blockIdx.y;


	//each block deal with one pair
	//if (cmIdx < numNPWorkItems)
	{
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x*blockDim.y;

		PxgShape softbodyShape, trimeshShape;
		PxU32 softbodyCacheRef, trimeshCacheRef;
		LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH, PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
			softbodyShape, softbodyCacheRef, trimeshShape, trimeshCacheRef);


		PxsCachedTransform trimeshTransformCache;
		PxsCachedTransform_ReadWarp(trimeshTransformCache, transformCache + trimeshCacheRef);


		const PxTransform& trimeshToWorld = trimeshTransformCache.transform;
		const PxMeshScale& trimeshScale = trimeshShape.scale;

		const PxgSoftBody& softbody = softbodies[softbodyShape.particleOrSoftbodyId];

		const PxU32 idx = threadIdx.x;

		if (idx == 0)
		{
			//printf("%f, %f, %f, %f, %f, %f\n", meshToWorld.p.x, meshToWorld.p.y, meshToWorld.p.z, meshToWorld.q.x, meshToWorld.q.y, meshToWorld.q.z);

			{
				const PxU8* triGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);				
				readTriangleMesh(triGeomPtr, s_warpScratch->bv32PackedNodes, s_warpScratch->meshVerts, s_warpScratch->meshVertsIndices);
			}
		}

		__syncthreads();



		const PxReal worldContactDist = contactDistance[softbodyCacheRef] + contactDistance[trimeshCacheRef];
		// Use the smalest meshScale component to transform wordContactDist to the mesh space
		const PxReal contactDist = worldContactDist / trimeshScale.scale.abs().minElement();

		const PxU32 nbTets = softbody.mNumTets;// s_warpScratch->nbPrimitives[0];

		const PxU32 NbWarps = blockDim.y*gridDim.x;

		const uint4* tetIndices = softbody.mTetIndices;// s_warpScratch->tetmeshTetIndices[0];
		const float4* tetVerts = softbody.mPosition_InvMass;// s_warpScratch->tetmeshVerts[0];
		const PxU8* surfaceHint = softbody.mTetMeshSurfaceHint;// s_warpScratch->tetmeshTetSurfaceHint[0];

		PxBounds3 tetBoundInTrimeshVertexSpace;

		for (PxU32 tetrahedronIdx = globalWarpIdx; tetrahedronIdx < nbTets; tetrahedronIdx += NbWarps)
		{
			//avoid generate contacts if the tetrahedron isn't a surface tetrahedron
			PxU8 hint = surfaceHint[tetrahedronIdx];
			if (hint)
			{
				const uint4 tetIdx = tetIndices[tetrahedronIdx];

				const PxVec3 worldV0 = PxLoad3(tetVerts[tetIdx.x]);
				const PxVec3 worldV1 = PxLoad3(tetVerts[tetIdx.y]);
				const PxVec3 worldV2 = PxLoad3(tetVerts[tetIdx.z]);
				const PxVec3 worldV3 = PxLoad3(tetVerts[tetIdx.w]);

				//transform those point to triangle vertex space
				const PxVec3 v0 = shape2Vertex(trimeshToWorld.transformInv(worldV0), trimeshScale.scale, trimeshScale.rotation);
				const PxVec3 v1 = shape2Vertex(trimeshToWorld.transformInv(worldV1), trimeshScale.scale, trimeshScale.rotation);
				const PxVec3 v2 = shape2Vertex(trimeshToWorld.transformInv(worldV2), trimeshScale.scale, trimeshScale.rotation);
				const PxVec3 v3 = shape2Vertex(trimeshToWorld.transformInv(worldV3), trimeshScale.scale, trimeshScale.rotation);

				if (computeTetBoundWithTetrahedron(v0, v1, v2, v3, tetBoundInTrimeshVertexSpace, tets[threadIdx.y]))
				{

					
					uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
					const PxU32 stackSize = stackSizeBytes / sizeof(uint4);				
					TriangleLeafBoundMinMaxTraverser<true> minMaxTest(s_warpScratch,
						tetBoundInTrimeshVertexSpace,						
						contactDist, &tets[threadIdx.y]);
					bv32TreeTraversal<TriangleLeafBoundMinMaxTraverser<true>, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, minMaxTest);

					const PxU32 MaxContacts = 4;

					PxU32 mask = __ballot_sync(FULL_MASK, minMaxTest.mBestDistance < contactDist);

					PxU32 count = PxMin(PxU32(__popc(mask)), MaxContacts);

					if (count)
					{
						PxU32 index;
						if (threadIdx.x == 0)
						{
							index = atomicAdd(midphasePairsNum, count);
						}

						index = __shfl_sync(FULL_MASK, index, 0);

						PxReal bestSep = -1.f;
						for (PxU32 i = 0; i < count && bestSep < contactDist; i++)
						{
							PxU32 winnerLane = minIndex(minMaxTest.mBestDistance, FULL_MASK, bestSep);

							if (threadIdx.x == winnerLane)
							{
								if (minMaxTest.mBestDistance < contactDist)
								{
									if ((index + i) < stackSize)
										tStackPtr[index + i] = make_uint4(cmIdx, tetrahedronIdx, minMaxTest.mPrimIndex, 0);
									minMaxTest.mBestDistance = PX_MAX_F32;
								}
							}
						}
					}
				}
			}
		}
	}

}


extern "C" __global__
__launch_bounds__(1024, 0)
void sb_sdfMeshMidphaseGeneratePairsLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxReal* PX_RESTRICT					restDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgSoftBody* PX_RESTRICT softbodies,
	const PxNodeIndex* PX_RESTRICT shapeToRigidRemapTable,

	PxgRigidFilterPair*							filterPairs,
	const PxU32									nbFilterPairs,

	PxgFEMContactWriter writer
)
{
	__shared__ __align__(16) char sMesh[sizeof(PxgTriangleMesh)];
	PxgTriangleMesh& triangleMesh = reinterpret_cast<PxgTriangleMesh&>(*sMesh);

	__shared__ __align__(16) char sSdfTexture[sizeof(SparseSDFTexture)];
	SparseSDFTexture& sdfTexture = reinterpret_cast<SparseSDFTexture&>(*sSdfTexture);

	__shared__ PxU32 triangleIndicesS[1024];


	const PxU32 cmIdx = blockIdx.x;


	PxgShape softbodyShape, trimeshShape;
	PxU32 softbodyCacheRef, trimeshCacheRef;
	LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH, PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
		softbodyShape, softbodyCacheRef, trimeshShape, trimeshCacheRef);


	PxsCachedTransform trimeshTransformCache;
	PxsCachedTransform_ReadWarp(trimeshTransformCache, transformCache + trimeshCacheRef);

	const PxNodeIndex rigidId = shapeToRigidRemapTable[trimeshCacheRef];

	const PxTransform& trimeshToWorld = trimeshTransformCache.transform;
	const PxMeshScale& trimeshScale = trimeshShape.scale;

	const PxgSoftBody& softbody = softbodies[softbodyShape.particleOrSoftbodyId];

	if (threadIdx.x < 32)
	{
		readTriangleMesh(trimeshShape, triangleMesh);
		__syncwarp();
	}

	if (threadIdx.x == 0)
		sdfTexture.initialize(triangleMesh); //The texture is stored in shared memory - only one threads needs to initialize it

	__syncthreads();



	const PxReal worldContactDist = contactDistance[softbodyCacheRef] + contactDistance[trimeshCacheRef];
	// Use the smalest meshScale component to transform wordContactDist to the mesh space
	//const PxReal contactDist = worldContactDist / trimeshScale.scale.abs().minElement();
	const PxReal cullScale = worldContactDist / trimeshScale.scale.abs().minElement();

	const PxReal restDist = restDistance[cmIdx];


	const PxU32 nbTets = softbody.mNumTets;// s_warpScratch->nbPrimitives[0];

	const PxTransform softbodyToTrimeshTransform = trimeshToWorld.getInverse();

	//const PxU32 NbWarps = blockDim.y*gridDim.x;

	const uint4* tetIndices = softbody.mTetIndices;// s_warpScratch->tetmeshTetIndices[0];
	const float4* tetVerts = softbody.mPosition_InvMass;// s_warpScratch->tetmeshVerts[0];
	const PxU8* surfaceHint = softbody.mTetMeshSurfaceHint;// s_warpScratch->tetmeshTetSurfaceHint[0];
	PX_UNUSED(surfaceHint);

	//PxBounds3 tetBoundInTrimeshVertexSpace;

	for (PxU32 i = 0; i < nbTets;)
	{
		PxU32 nbFoundTets = findInterestingTets<32, 1024>(nbTets, tetIndices, tetVerts,
			trimeshShape.scale, cullScale, sdfTexture, softbodyToTrimeshTransform, i, triangleIndicesS/*, surfaceHint*/);

		PxU32 ind = threadIdx.x < nbFoundTets ? triangleIndicesS[threadIdx.x] : nbTets;
			
		PxReal sep;
		PxVec3 contactDir;
		PxVec3 contactPos;
		bool candidateContact = false;
		PxU32 pairId1;

		if (ind < nbTets)
		{
			const uint4 tet = tetIndices[ind];
			const PxVec3 v0 = shape2Vertex(softbodyToTrimeshTransform.transform(PxLoad3(tetVerts[tet.x])), trimeshShape.scale.scale, trimeshShape.scale.rotation);
			const PxVec3 v1 = shape2Vertex(softbodyToTrimeshTransform.transform(PxLoad3(tetVerts[tet.y])), trimeshShape.scale.scale, trimeshShape.scale.rotation);
			const PxVec3 v2 = shape2Vertex(softbodyToTrimeshTransform.transform(PxLoad3(tetVerts[tet.z])), trimeshShape.scale.scale, trimeshShape.scale.rotation);
			const PxVec3 v3 = shape2Vertex(softbodyToTrimeshTransform.transform(PxLoad3(tetVerts[tet.w])), trimeshShape.scale.scale, trimeshShape.scale.rotation);

			sep = doTetrahedronSDFCollision(sdfTexture, v0, v1, v2, v3,	contactPos, contactDir, cullScale);

			if (sep < cullScale)
			{
				
				contactDir = vertex2ShapeNormalVector(contactDir, trimeshShape.scale.scale, trimeshShape.scale.rotation);
				PxReal m = contactDir.magnitudeSquared();

				if (m > 0.0f)
				{
					m = 1.0f / PxSqrt(m);
					sep = sep * m;
					contactDir = contactDir * m;
				}

				candidateContact = sep < worldContactDist;

				if (candidateContact)
				{
					contactPos = trimeshToWorld.transform(vertex2Shape(contactPos, trimeshShape.scale.scale, trimeshShape.scale.rotation));

					contactDir = trimeshToWorld.rotate(contactDir);

					pairId1 = PxEncodeSoftBodyIndex(softbodyShape.particleOrSoftbodyId, ind);
					//If we found the tetrahedron index in the attachment list, we don't need to generate contacts between the rigid
					//body and the tetrahedron
					if (find(filterPairs, nbFilterPairs, rigidId.getInd(), pairId1))
						candidateContact = false;
				}
			}
		}

		PxU32 contactIndex = globalScanExclusive<32>(candidateContact, writer.totalContactCount);

		if (candidateContact)
		{
			PxU64 pairId0 = rigidId.getInd();

			writer.writeContactNoBarycentric(contactIndex, make_float4(contactPos.x, contactPos.y, contactPos.z, restDist),
				make_float4(contactDir.x, contactDir.y, contactDir.z, sep), pairId0, pairId1, rigidId.getInd());
		}
	}	
}


extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void sb_meshMidphaseGeneratePairsLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgSoftBody* PX_RESTRICT softbodies,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 18];

	sb_trimeshMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		tolerenceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		softbodies,
		stackSizeBytes,
		stackPtr,
		midphasePairsNum,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}

template<unsigned int WarpsPerBlock>
__device__ static inline void sb_clothMidphaseCore(
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgSoftBody* PX_RESTRICT softbodies,
	const PxgFEMCloth* PX_RESTRICT clothes,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,									//output
	PxU32* PX_RESTRICT midphasePairsNum,						//output

	femMidphaseScratch*	s_warpScratch
)
{
	__shared__ Tetrahedron tets[WarpsPerBlock];

	const PxU32 cmIdx = blockIdx.y;
	//each block deal with one pair
	//if (cmIdx < numNPWorkItems)
	{
		const PxU32 globalWarpIdx = threadIdx.y + blockIdx.x*blockDim.y;

		PxgShape softbodyShape, clothShape;
		PxU32 softbodyCacheRef, clothCacheRef;
		LoadShapePairWarp<PxGeometryType::eTETRAHEDRONMESH, PxGeometryType::eTRIANGLEMESH>(cmInputs, cmIdx, gpuShapes,
			softbodyShape, softbodyCacheRef, clothShape, clothCacheRef);

		PxsCachedTransform clothTransformCache;
		PxsCachedTransform_ReadWarp(clothTransformCache, transformCache + clothCacheRef);


		const PxTransform& clothToWorld = clothTransformCache.transform;
		//const PxMeshScale& trimeshScale = trimeshShape.scale;

		const PxgSoftBody& softbody = softbodies[softbodyShape.particleOrSoftbodyId];
		const PxgFEMCloth& cloth = clothes[clothShape.particleOrSoftbodyId];

		const PxU32 idx = threadIdx.x;

		if (idx == 0)
		{
			
			PxU8* triGeomPtr = reinterpret_cast<PxU8 *>(cloth.mTriMeshData);
			//const uint4 nbVerts_nbPrimitives_maxDepth_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(triGeomPtr);
			triGeomPtr += sizeof(uint4);

			Gu::BV32DataPacked* bv32PackedNodes = reinterpret_cast<Gu::BV32DataPacked*>(triGeomPtr);
			s_warpScratch->bv32PackedNodes = bv32PackedNodes;
		
			s_warpScratch->meshVerts = cloth.mPosition_InvMass;

			s_warpScratch->meshVertsIndices = cloth.mTriangleVertexIndices;

			
		}

		__syncthreads();


		const PxReal contactDist = contactDistance[softbodyCacheRef] + contactDistance[clothCacheRef];

		const PxU32 nbTets = softbody.mNumTets;// s_warpScratch->nbPrimitives[0];

		const PxU32 NbWarps = blockDim.y*gridDim.x;

		const uint4* tetIndices = softbody.mTetIndices;// s_warpScratch->tetmeshTetIndices[0];
		const float4* tetVerts = softbody.mPosition_InvMass;// s_warpScratch->tetmeshVerts[0];
		const PxU8* surfaceHint = softbody.mTetMeshSurfaceHint;// s_warpScratch->tetmeshTetSurfaceHint[0];

		PxBounds3 tetBoundInTrimeshVertexSpace;

		for (PxU32 tetrahedronIdx = globalWarpIdx; tetrahedronIdx < nbTets; tetrahedronIdx += NbWarps)
		{
			//avoid generate contacts if the tetrahedron isn't a surface tetrahedron
			PxU8 hint = surfaceHint[tetrahedronIdx];
			if (hint)
			{
				const uint4 tetIdx = tetIndices[tetrahedronIdx];

				const PxVec3 worldV0 = PxLoad3(tetVerts[tetIdx.x]);
				const PxVec3 worldV1 = PxLoad3(tetVerts[tetIdx.y]);
				const PxVec3 worldV2 = PxLoad3(tetVerts[tetIdx.z]);
				const PxVec3 worldV3 = PxLoad3(tetVerts[tetIdx.w]);

				//transform those point to triangle vertex space
				const PxVec3 v0 = clothToWorld.transformInv(worldV0);
				const PxVec3 v1 = clothToWorld.transformInv(worldV1);
				const PxVec3 v2 = clothToWorld.transformInv(worldV2);
				const PxVec3 v3 = clothToWorld.transformInv(worldV3);

				if (computeTetBoundWithTetrahedron(v0, v1, v2, v3, tetBoundInTrimeshVertexSpace, tets[threadIdx.y]))
				{


					uint4* tStackPtr = reinterpret_cast<uint4*>(stackPtr);
					const PxU32 stackSize = stackSizeBytes / sizeof(uint4);

					TriangleLeafBoundMinMaxTraverser<false> minMaxTest(s_warpScratch, tetBoundInTrimeshVertexSpace, contactDist, &tets[threadIdx.y]);
					bv32TreeTraversal<TriangleLeafBoundMinMaxTraverser<false>, WarpsPerBlock>(s_warpScratch->bv32PackedNodes, s_warpScratch->sBv32Nodes, minMaxTest);

					const PxU32 MaxContacts = 4;

					PxU32 mask = __ballot_sync(FULL_MASK, minMaxTest.mBestDistance < contactDist);

					PxU32 count = PxMin(PxU32(__popc(mask)), MaxContacts);

					if (count)
					{
						PxU32 index;
						if (threadIdx.x == 0)
						{
							index = atomicAdd(midphasePairsNum, count);
						}

						index = __shfl_sync(FULL_MASK, index, 0);

						PxReal bestSep = -1.f;
						for (PxU32 i = 0; i < count && bestSep < contactDist; i++)
						{
							PxU32 winnerLane = minIndex(minMaxTest.mBestDistance, FULL_MASK, bestSep);

							if (threadIdx.x == winnerLane)
							{
								if (minMaxTest.mBestDistance < contactDist)
								{
									if ((index + i) < stackSize)
										tStackPtr[index + i] = make_uint4(cmIdx, tetrahedronIdx, minMaxTest.mPrimIndex, 0);
									minMaxTest.mBestDistance = PX_MAX_F32;
								}
							}
						}
					}
				}

			}
		}
	}

}

extern "C" __global__
//__launch_bounds__(MIDPHASE_WARPS_PER_BLOCK * WARP_SIZE, 4)
void sb_clothMidphaseGeneratePairsLaunch(
	const PxReal tolerenceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxBounds3* PX_RESTRICT bounds,
	const PxReal* PX_RESTRICT contactDistance,
	const PxgShape* PX_RESTRICT gpuShapes,
	const PxgSoftBody* PX_RESTRICT softbodies,
	const PxgFEMCloth* PX_RESTRICT clothes,
	const PxU32 stackSizeBytes,
	PxU8* PX_RESTRICT stackPtr,							//output
	PxU32* PX_RESTRICT midphasePairsNum					//output
)
{

	__shared__ PxU32 scratchMem[MIDPHASE_WARPS_PER_BLOCK][WARP_SIZE * 18];

	sb_clothMidphaseCore<MIDPHASE_WARPS_PER_BLOCK>(
		tolerenceLength,
		cmInputs,
		transformCache,
		bounds,
		contactDistance,
		gpuShapes,
		softbodies,
		clothes,
		stackSizeBytes,
		stackPtr,
		midphasePairsNum,
		(femMidphaseScratch*)scratchMem[threadIdx.y]
		);
}
