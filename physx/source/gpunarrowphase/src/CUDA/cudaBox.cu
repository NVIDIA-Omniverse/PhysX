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


#include "foundation/PxVec3.h"
#include "foundation/PxMat34.h"
#include "foundation/PxTransform.h"
#include "foundation/PxAssert.h"
#include <stdio.h>


#include "cuda.h"
#include "cuda_runtime.h"
#include "convexFormat.h"
#include "cudaNpCommon.h"
#include "nputils.cuh"

#include "PxgCommonDefines.h"
#include "PxsTransformCache.h"
#include "PxsContactManagerState.h"
#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgPersistentContactManifold.h"

#include "PxgNpKernelIndices.h"

#include "PxsMaterialCore.h"

#include "PxContact.h"
#include "geometry/PxGeometry.h"

#include "SparseRemove.cuh"

#include "sphereCollision.cuh"
#include "contactPatchUtils.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels23() {}

__device__ PxVec3 getIncidentPolygon4(PxVec3& faceNormal, const PxVec3& axis, const PxMat34& transf1To0,
	const PxVec3& extents, const PxU32 threadIndexInGroup, const PxU32 groupStartThreadIndex, const PxU32 groupMask)
{
	//using namespace aos;

	PxReal d, absd;


	if (threadIndexInGroup < 3)
	{
		//calculate the insident face for b
		d = (&transf1To0.m.column0)[threadIndexInGroup].dot(axis);
		absd = PxAbs(d);
	}


	PxReal s0 = (threadIndexInGroup == 0 || threadIndexInGroup == 3) ? -1.f : 1.f;
	PxReal s1 = threadIndexInGroup & 2 ? 1.f : -1.f;

	PxVec3 sign;

	PxReal absd0 = __shfl_sync(groupMask, absd, groupStartThreadIndex);
	PxReal absd1 = __shfl_sync(groupMask, absd, groupStartThreadIndex + 1);
	PxReal absd2 = __shfl_sync(groupMask, absd, groupStartThreadIndex + 2);

	if (absd0 >= absd1 && absd0 >= absd2)
	{
		//the incident face is on u0
		sign.x = __shfl_sync(groupMask, d, groupStartThreadIndex) > 0.f ? -1.f : 1.f;
		sign.y = s0;
		sign.z = s1;

		faceNormal = transf1To0.m.column0 * sign.x;
	}
	else if (absd1 >= absd2)
	{
		//the incident face is on u1
		sign.y = __shfl_sync(groupMask, d, groupStartThreadIndex + 1) > 0.f ? -1.f : 1.f;
		sign.x = s0;
		sign.z = s1;
		faceNormal = transf1To0.m.column1 * sign.y;
	}
	else
	{
		//the incident face is on u2
		sign.z = __shfl_sync(groupMask, d, groupStartThreadIndex + 2) > 0.f ? -1.f : 1.f;
		sign.x = s0;
		sign.y = s1;
		faceNormal = transf1To0.m.column2 * sign.z;
	}

	const PxVec3 vert = extents.multiply(sign);

	return transf1To0.transform(vert);
}

template <PxU32 Count>
struct LocalContacts
{

	//We can derive localPointA from localPointB as they are aligned to the surface plane. The penetration is 
	//equal to -localPointB.w
	PxVec3 localPointB[Count];
};


static __device__ bool contains(const PxVec3* verts, const PxU32 numVerts, const PxVec3& p, const PxBounds3& bounds)
{
	if (p.x < bounds.minimum.x || p.x > bounds.maximum.x || p.y < bounds.minimum.y || p.y > bounds.maximum.y)
		return false;

	const PxReal tx = p.x;
	const PxReal ty = p.y;

	const PxReal eps = 1e-6f;
	const PxReal zero = 0.f;
	PxU32 intersectionPoints = 0;

	PxU32 i = 0, j = numVerts - 1;

	for (; i < numVerts; j = i++)
	{
		const PxReal jy = verts[j].y;
		const PxReal iy = verts[i].y;

		const PxReal jx = verts[j].x;
		const PxReal ix = verts[i].x;

		//if p is one of the end point, we will return intersect
		const bool con0 = (tx == jx && ty == jy);
		const bool con1 = (tx == ix && ty == iy);

		if (con0 || con1)
			return true;

		//(verts[i].y > test.y) != (points[j].y > test.y) 
		const bool yflag0 = jy > ty;
		const bool yflag1 = iy > ty;

		//ML: the only case the ray will intersect this segment is when the p's y is in between two segments y
		if (yflag0 != yflag1)
		{
			//ML: choose ray, which start at p and every points in the ray will have the same y component
			//t1 = (yp - yj)/(yi - yj)
			//qx = xj + t1*(xi-xj)
			//t = qx - xp > 0 for the ray and segment intersection happen
			const PxReal jix = ix - jx;
			const PxReal jiy = iy - jy;
			//const FloatV jtx = FSub(tx, jy);
			const PxReal jty = ty - jy;
			const PxReal part1 = jty * jix;
			//const FloatV part2 = FMul(jx, jiy);
			//const FloatV part3 = FMul(V3Sub(tx, eps), jiy);
			const PxReal part2 = (jx + eps) * jiy;
			const PxReal part3 = tx * jiy;

			const bool comp = jiy > zero;
			const PxReal tmp = part1 + part2;
			const PxReal comp1 = comp ? tmp : part3;
			const PxReal comp2 = comp ? part3 : tmp;

			if (comp1 >= comp2)
			{
				if (intersectionPoints == 1)
					return false;

				intersectionPoints++;
			}
		}
	}
	return intersectionPoints > 0;
}

static __device__ PxBounds3 calculateBounds4(const PxVec3& pt,
	const PxU32 syncMask)
{
	PxBounds3 bounds;
	bounds.minimum = bounds.maximum = pt;

	PxVec3 val(__shfl_xor_sync(syncMask, pt.x, 1, 4),
		__shfl_xor_sync(syncMask, pt.y, 1, 4),
		__shfl_xor_sync(syncMask, pt.z, 1, 4));
	bounds.include(val);

	bounds.minimum = bounds.minimum.minimum(PxVec3(__shfl_xor_sync(syncMask, bounds.minimum.x, 2, 4),
		__shfl_xor_sync(syncMask, bounds.minimum.y, 2, 4),
		__shfl_xor_sync(syncMask, bounds.minimum.z, 2, 4)));

	bounds.maximum = bounds.maximum.maximum(PxVec3(__shfl_xor_sync(syncMask, bounds.maximum.x, 2, 4),
		__shfl_xor_sync(syncMask, bounds.maximum.y, 2, 4),
		__shfl_xor_sync(syncMask, bounds.maximum.z, 2, 4)));

	return bounds;
}


//p0 and p1 is in the local space of AABB
static __device__ bool intersectSegmentAABB(const PxVec3& p0, const PxVec3& d, const PxVec3& max,
	const PxVec3& min, PxReal& tmin, PxReal& tmax)
{
	const PxReal eps(1e-6f);
	const PxVec3 absV = d.abs();
	const PxReal fMax(PX_MAX_F32);

	PxReal tminf = 0.f;
	PxReal tmaxf = 1.f;

	bool isParallelX = eps > absV.x;
	bool isParallelY = eps > absV.y;
	bool isParallelZ = eps > absV.z;

	if ((isParallelX && (p0.x > max.x || p0.x < min.x)) ||
		(isParallelY && (p0.y > max.y || p0.y < min.y)) ||
		(isParallelZ && (p0.z > max.z || p0.z < min.z)))
		return false;

	const PxVec3 odd(1.f / d.x, 1.f / d.y, 1.f / d.z);
	const PxVec3 t1(isParallelX ? 0.f : (min.x - p0.x)*odd.x,
		isParallelY ? 0.f : (min.y - p0.y)*odd.y,
		isParallelZ ? 0.f : (min.z - p0.z)*odd.z);

	const PxVec3 t2(isParallelX ? fMax : (max.x - p0.x)*odd.x,
		isParallelY ? fMax : (max.y - p0.y)*odd.y,
		isParallelZ ? fMax : (max.z - p0.z)*odd.z);

	const PxVec3 tt1 = t1.minimum(t2);
	const PxVec3 tt2 = t1.maximum(t2);


	const PxReal ft1 = tt1.maxElement();
	const PxReal ft2 = tt2.minElement();

	tminf = PxMax(ft1, tminf);
	tmaxf = PxMin(tmaxf, ft2);

	tmin = tminf;
	tmax = tmaxf;

	const bool con0 = tminf > tmaxf;
	const bool con1 = tminf > 1.f;

	return !(con0 || con1);
}



//pts, faceNormal and contact normal are in the local space of new space
static __device__ PxU32 calculateContacts(const PxReal extentX_, const PxReal extentY_, const PxVec3& pt,
	const PxVec3* pts, const PxVec3& incidentFaceNormalInNew, const PxVec3& localNormal, const PxReal contactDist,
	const PxU32 groupMask, const PxU32 threadIndexInGroup, const PxU32 groupThreadStartIndex, PxVec4* localPointB)
{
	const PxReal max = PX_MAX_F32;

	const PxReal extentX = 1.00001f*extentX_;
	const PxReal extentY = 1.00001f*extentY_;

	const PxReal nExtentX = -extentX;
	const PxReal nExtentY = -extentY;

	bool pPenetration;
	bool pArea;


	const PxVec3 ext(extentX, extentY, max);
	const PxVec3 negExt(nExtentX, nExtentY, -contactDist - 1e-7f);

	__syncwarp(groupMask);

	PxU32 count = 0;

	//get the projection point of pts
	{
		const PxReal z = -pt.z;
		if (contactDist > z)
		{
			pPenetration = true;
			//const BoolV con = V3IsGrtrOrEq(bound, absPt);
			if (ext.x >= PxAbs(pt.x) &&
				ext.y >= PxAbs(pt.y))
			{
				pArea = true;
				//contacts.localPointA[0] = PxVec3(pt.x, pt.y, 0.f);
				localPointB[0] = PxVec4(pt, z);

				//printf("%i: AREA (%f, %f, %f, %f)\n", threadIdx.x, pt.x, pt.y, pt.z, z);

				//contacts.separation[0] = z;
				count = 1;
			}
			else
			{
				pArea = false;
			}
		}
		else
		{
			pPenetration = false;
			pArea = false;
		}
	}

	//All points of A in B, so exit
	if (__all_sync(groupMask, pArea))
	{
		return count;
	}

	PxBounds3 bounds = calculateBounds4(pt, groupMask);

	bool pArea2 = false;

	//if(pPenetration[0] && pPenetration[1] && pPenetration[2] && pPenetration[3])
	{
		//if(!pArea[0] || !pArea[1] || !pArea[2] || !pArea[3])
		{
			const PxReal denom = incidentFaceNormalInNew.z;
			{
				//const PxVec3 q0(extentX, extentY, zero);
				const PxVec3 q0((threadIndexInGroup == 0 || threadIndexInGroup == 3) ? -extentX : extentX,
					(threadIndexInGroup & 2) ? -extentY : extentY, 0.f);

				PxReal t;

				if (contains(pts, 4, q0, bounds))
				{
					const PxReal nom = incidentFaceNormalInNew.dot(pts[0] - q0);
					t = nom / denom;

					if (contactDist > (-t))
					{
						pArea2 = true;
						//contacts.localPointA[0] = q0;
						localPointB[count * 4] = PxVec4(q0.x, q0.y, t, -t);

						//printf("%i: AREA2 (%f, %f, %f, %f)\n", threadIdx.x, q0.x, q0.y, t, -t);
						//contacts.separation[0] = -t;
						count++;
					}
				}

			}
		}
	}

	//All points of B inside A, so exit
	if (__all_sync(groupMask, pArea2))
	{
		return count;
	}



	//for (PxU32 rStart = 0, rEnd = 3; rStart < 4; rEnd = rStart++)
	{
		PxU32 rStart = threadIndexInGroup;
		PxU32 rEnd = (threadIndexInGroup + 1) & 3;
		const PxVec3 p0 = pts[rStart];
		const PxVec3 p1 = pts[rEnd];

		bool pPenStart = __shfl_sync(groupMask, pPenetration, groupThreadStartIndex + rStart);
		bool pPenEnd = __shfl_sync(groupMask, pPenetration, groupThreadStartIndex + rEnd);
		bool pAreaStart = __shfl_sync(groupMask, pArea, groupThreadStartIndex + rStart);
		bool pAreaEnd = __shfl_sync(groupMask, pArea, groupThreadStartIndex + rEnd);

		if (pPenStart || pPenEnd)
		{
			const bool con0 = pPenStart && pAreaStart;
			const bool con1 = pPenEnd && pAreaEnd;
			if (!(con0 && con1))
			{

				//intersect t value with x plane
				const PxVec3 p0p1 = p1 - p0;

				PxReal tmin, tmax;
				if (intersectSegmentAABB(p0, p0p1, ext, negExt, tmin, tmax))
				{
					if (!con0)
					{
						const PxVec3 intersectP = p0 + p0p1 * tmin;
						localPointB[count * 4] = PxVec4(intersectP, -intersectP.z);
						//printf("%i: edgeStart (%f, %f, %f, %f)\n", threadIdx.x, intersectP.x, intersectP.y, intersectP.z, -intersectP.z);
						count++;
					}
					if (!con1)
					{
						const PxVec3 intersectP = p0 + p0p1 * tmax;
						localPointB[count * 4] = PxVec4(intersectP, -intersectP.z);
						//printf("%i: edgeEnd (%f, %f, %f, %f)\n", threadIdx.x, intersectP.x, intersectP.y, intersectP.z, -intersectP.z);
						count++;
					}
				}
			}
		}
	}

	return count;
}

template <PxU32 NumWarps>
struct TempBoxBoxBuffer
{
	// space for 3 contacts per thread for all threads in the block
	PxVec4 tempBuff[NumWarps * 32 * 3];
};


template<PxU32 NumWarps>
__device__ PxU32 doBoxBoxGenerateContacts(const PxVec3& box0Extent, const PxVec3& box1Extent,
	const PxMat34& transform0, const PxMat34& transform1, const PxReal contactDist,
	TempBoxBoxBuffer<NumWarps>& tempBuffer, PxVec3& normal, const PxU32 threadIndexInGroup, const PxU32 groupMask)
{
	//4 threads per pair
	PxU32 groupStartIndex = threadIdx.x & (~3);

	PxVec4* localContacts = &tempBuffer.tempBuff[groupStartIndex * 3];
	PxMat33& abs0To1 = reinterpret_cast<PxMat33&>(*localContacts);

	const PxMat34 transform1To0 = transform0.transformTranspose(transform1);
	const PxMat33& rot1To0 = transform1To0.m;
	const PxMat33 rot0To1 = rot1To0.getTranspose();

	const PxVec3 uEps(1e-6f);

	PxReal t = 0.f;
	PxVec3 col(0.f);

	//__shared__ PxMat33 shAbs0To1[8*NumWarps];

	__shared__ PxVec3 shVerts[32 * NumWarps];

	PxReal ea;
	PxReal eb;

	//printf("%i: threadIndexInGroup = %i, groupMask = %i\n", threadIdx.x, threadIndexInGroup, groupMask);

	if (threadIndexInGroup < 3)
	{
		t = (&transform1To0.p.x)[threadIndexInGroup];
		col = (&rot1To0.column0)[threadIndexInGroup];

		abs0To1[threadIndexInGroup] = rot0To1[threadIndexInGroup].abs() + uEps;
		ea = box0Extent[threadIndexInGroup];
		eb = box1Extent[threadIndexInGroup];
	}

	__syncwarp(groupMask);

	PxReal sign = 0.f;
	PxReal minOverlap = PX_MAX_F32;

	PxReal ra, rb;
	PxReal radiusSum = PX_MAX_F32;



	PxReal s, overlap = 1.f;

	if (threadIndexInGroup < 3)
	{
		rb = abs0To1[threadIndexInGroup].dot(box1Extent);

		radiusSum = ea + rb;
		s = t;
		overlap = radiusSum - PxAbs(s) + contactDist;

		//printf("%i: overlap = %f\n", threadIdx.x, overlap);
	}

	if (__any_sync(groupMask, 0.f > overlap))
		return false;

	sign = s;
	minOverlap = overlap;
	PxU32 feature = threadIndexInGroup;

	PxVec3 abs1To0Col = col.abs() + uEps;

	//ub0 
	if (threadIndexInGroup < 3)
	{
		s = transform1To0.p.dot((&rot1To0.column0)[threadIndexInGroup]);

		ra = abs1To0Col.dot(box0Extent);

		radiusSum = ra + eb;
		overlap = radiusSum - PxAbs(s) + contactDist;
		//printf("%i: overlap2 = %f, ra = %f, eb = %f, s = %f, contactDist = %f\n", threadIdx.x, overlap,
		//	ra, eb, s, contactDist);
	}

	if (__any_sync(groupMask, 0.f > overlap))
		return false;

	if (minOverlap > overlap)
	{
		minOverlap = overlap;
		sign = s;
		feature += 3;
	}

	//printf("%i: feature = %i\n", threadIdx.x, feature);


	//ua0 X ub0

	PxReal absSign = 0.f;

	if (threadIndexInGroup < 3)
	{
		//B into A's space, ua0Xub0[0,-b3, b2]
		absSign = PxAbs((col.y * transform1To0.p.z) - (col.z * transform1To0.p.y));

		//B into A's space, ua0Xub0[0,-b3, b2]
		const PxReal vtemp0 = abs1To0Col.z * box0Extent.y;
		const PxReal vtemp1 = abs1To0Col.y * box0Extent.z;
		ra = vtemp0 + vtemp1;

		//A into B's space, ua0Xub0[0, a3, -a2]
		const PxReal vtemp01 = abs0To1.column0[threadIndexInGroup == 2 ? 1 : 2] * box1Extent[threadIndexInGroup == 0 ? 1 : 0];
		const PxReal vtemp02 = abs0To1.column0[threadIndexInGroup == 0 ? 1 : 0] * box1Extent[threadIndexInGroup == 2 ? 1 : 2];
		rb = vtemp01 + vtemp02;
		radiusSum = ra + rb + contactDist;
	}

	if (__any_sync(groupMask, absSign > radiusSum))
		return false;

	//ua1 X ub0
	if (threadIndexInGroup < 3)
	{
		//B into A's space, ua0Xub0[b3, 0, -b1]
		absSign = PxAbs((col.z * transform1To0.p.x) - (col.x * transform1To0.p.z));

		//B into A's space, ua0Xub0[b3, 0, -b1]
		const PxReal vtemp0 = abs1To0Col.z * box0Extent.x;
		const PxReal vtemp1 = abs1To0Col.x * box0Extent.z;
		ra = vtemp0 + vtemp1;

		//A into B's space, ua0Xub1[0, a3, -a2]
		const PxReal vtemp01 = abs0To1.column1[threadIndexInGroup == 2 ? 1 : 2] * box1Extent[threadIndexInGroup == 0 ? 1 : 0];
		const PxReal vtemp02 = abs0To1.column1[threadIndexInGroup == 0 ? 1 : 0] * box1Extent[threadIndexInGroup == 2 ? 1 : 2];
		rb = vtemp01 + vtemp02;

		radiusSum = ra + rb + contactDist;
	}

	if (__any_sync(groupMask, absSign > radiusSum))
		return false;

	//ua1 X ub0
	if (threadIndexInGroup < 3)
	{
		//B into A's space, ua0Xub0[b3, 0, -b1]
		absSign = PxAbs((col.x * transform1To0.p.y) - (col.y * transform1To0.p.x));

		//B into A's space, ua0Xub0[b3, 0, -b1]
		const PxReal vtemp0 = abs1To0Col.y * box0Extent.x;
		const PxReal vtemp1 = abs1To0Col.x * box0Extent.y;
		ra = vtemp0 + vtemp1;

		//A into B's space, ua0Xub1[0, a3, -a2]
		const PxReal vtemp01 = abs0To1.column2[threadIndexInGroup == 2 ? 1 : 2] * box1Extent[threadIndexInGroup == 0 ? 1 : 0];
		const PxReal vtemp02 = abs0To1.column2[threadIndexInGroup == 0 ? 1 : 0] * box1Extent[threadIndexInGroup == 2 ? 1 : 2];
		rb = vtemp01 + vtemp02;

		radiusSum = ra + rb + contactDist;
	}

	if (__any_sync(groupMask, absSign > radiusSum))
		return false;



	PxReal min0 = __shfl_sync(groupMask, minOverlap, groupStartIndex);
	PxReal min1 = __shfl_sync(groupMask, minOverlap, groupStartIndex + 1);
	PxReal min2 = __shfl_sync(groupMask, minOverlap, groupStartIndex + 2);

	if (min0 <= min1 && min0 <= min2)
	{
		feature = __shfl_sync(groupMask, feature, groupStartIndex);
		sign = __shfl_sync(groupMask, sign, groupStartIndex);
	}
	else if (min1 <= min2)
	{
		feature = __shfl_sync(groupMask, feature, groupStartIndex + 1);
		sign = __shfl_sync(groupMask, sign, groupStartIndex + 1);
	}
	else
	{
		feature = __shfl_sync(groupMask, feature, groupStartIndex + 2);
		sign = __shfl_sync(groupMask, sign, groupStartIndex + 2);
	}

	// now all threads in the group have the same feature and sign values

	PxVec3 mtd;

	//__shared__ PxMat34 shNewTransformV[8 * NumWarps];
	//PxMat34& newTransformV = shNewTransformV[threadIdx.x / 4];
	PxMat34 newTransformV;
	const PxVec3& axis00 = transform0.m.column0;
	const PxVec3& axis01 = transform0.m.column1;
	const PxVec3& axis02 = transform0.m.column2;
	const PxVec3& axis10 = transform1.m.column0;
	const PxVec3& axis11 = transform1.m.column1;
	const PxVec3& axis12 = transform1.m.column2;

	PxVec3 incidentFaceNormalInNew;

	PxReal e0, e1;

	//printf("%i: feature = %i\n", threadIdx.x, feature);

	switch (feature)
	{
	case 0: //ua0
	{
		if (0.f >= sign)
		{
			mtd = axis00;
			newTransformV.m.column0 = -axis02;
			newTransformV.m.column1 = axis01;
			newTransformV.m.column2 = axis00;

		}
		else
		{
			const PxVec3 nAxis00 = -axis00;
			mtd = nAxis00;
			newTransformV.m.column0 = axis02;
			newTransformV.m.column1 = axis01;
			newTransformV.m.column2 = nAxis00;
		}
		newTransformV.p = transform0.p - mtd * box0Extent.x;
		e0 = box0Extent.z;
		e1 = box0Extent.y;
		break;
	}
	case 1: //ua1
	{
		if (0.f >= sign)
		{
			mtd = axis01;
			newTransformV.m.column0 = axis00;
			newTransformV.m.column1 = -axis02;
			newTransformV.m.column2 = axis01;

		}
		else
		{
			const PxVec3 nAxis01 = -axis01;
			mtd = nAxis01;
			newTransformV.m.column0 = axis00;
			newTransformV.m.column1 = axis02;
			newTransformV.m.column2 = nAxis01;
		}
		newTransformV.p = transform0.p - mtd * box0Extent.y;
		e0 = box0Extent.x;
		e1 = box0Extent.z;
		break;
	};
	case 2: //ua2
	{
		if (0.f >= sign)
		{
			mtd = axis02;
			newTransformV.m.column0 = axis00;
			newTransformV.m.column1 = axis01;
			newTransformV.m.column2 = axis02;
		}
		else
		{
			const PxVec3 nAxis02 = -axis02;
			mtd = nAxis02;
			newTransformV.m.column0 = axis00;
			newTransformV.m.column1 = -axis01;
			newTransformV.m.column2 = nAxis02;
		}

		newTransformV.p = transform0.p - mtd * box0Extent.z;
		e0 = box0Extent.x;
		e1 = box0Extent.y;

		break;
	};
	case 3: //ub0
	{
		if (0.f >= sign)
		{
			mtd = axis10;
			newTransformV.m.column0 = axis12;
			newTransformV.m.column1 = axis11;
			newTransformV.m.column2 = -axis10;
		}
		else
		{
			mtd = -axis10;
			newTransformV.m.column0 = -axis12;
			newTransformV.m.column1 = axis11;
			newTransformV.m.column2 = axis10;
		}

		newTransformV.p = transform1.p + mtd * box1Extent.x;

		e0 = box1Extent.z;
		e1 = box1Extent.y;

		break;
	};
	case 4: //ub1;
	{
		if (0.f >= sign)
		{
			mtd = axis11;
			newTransformV.m.column0 = axis10;
			newTransformV.m.column1 = axis12;
			newTransformV.m.column2 = -axis11;
		}
		else
		{
			mtd = -axis11;

			newTransformV.m.column0 = axis10;
			newTransformV.m.column1 = -axis12;
			newTransformV.m.column2 = axis11;
		}

		newTransformV.p = transform1.p + mtd * box1Extent.y;

		e0 = box1Extent.x;
		e1 = box1Extent.z;

		break;
	}
	case 5: //ub2;
	{

		if (0.f >= sign)
		{
			mtd = axis12;
			newTransformV.m.column0 = axis10;
			newTransformV.m.column1 = -axis11;
			newTransformV.m.column2 = -axis12;
		}
		else
		{
			mtd = -axis12;

			newTransformV.m.column0 = axis10;
			newTransformV.m.column1 = axis11;
			newTransformV.m.column2 = axis12;
		}


		newTransformV.p = transform1.p + mtd * box1Extent.z;

		e0 = box1Extent.x;
		e1 = box1Extent.y;

		break;
	};
	default:
		return false;
	}


	const PxMat34 transform1ToNew = newTransformV.transformTranspose(feature < 3 ? transform1 : transform0);
	const PxVec3 localNormal = newTransformV.m.transformTranspose(mtd);
	PxVec3 point = getIncidentPolygon4(incidentFaceNormalInNew, feature < 3 ? -localNormal : localNormal, transform1ToNew,
		feature < 3 ? box1Extent : box0Extent, threadIndexInGroup, groupStartIndex, groupMask);

	//printf("shVerts[%i] = (%f, %f, %f), mtd = (%f, %f, %f)\n", threadIdx.x, point.x, point.y, point.z, mtd.x, mtd.y, mtd.z);

	shVerts[threadIdx.x] = point;

	normal = mtd;// feature < 3 ? mtd : -mtd;

	__syncwarp(groupMask);

	// every thread stores up to 3 contacts here: localContacts[4*contactNbOfThread + threadIndexInGroup]
	PxVec4* contacts = &localContacts[threadIndexInGroup]; // last Vec4 element is the penetration

	const PxU32 count = calculateContacts(e0, e1, point, &shVerts[groupStartIndex], incidentFaceNormalInNew, localNormal, contactDist, groupMask, threadIndexInGroup,
		groupStartIndex, contacts);
	assert(count <= 3);

	// Now every thread has between zero and three contacts.

	// runningContacts will the the inclusive cumulative sum of count for the group,
	// then shuffle the group total to every thread of the group
	PxU32 runningContacts = count;
	{
		PxU32 c0 = __shfl_sync(groupMask, count, (threadIdx.x & 31) - 1);
		if (threadIndexInGroup > 0)
			runningContacts += c0;

		PxU32 c1 = __shfl_sync(groupMask, runningContacts, (threadIdx.x & 31) - 2);
		if (threadIndexInGroup > 1)
			runningContacts += c1;
	}
	PxU32 totalContacts = __shfl_sync(groupMask, runningContacts, (threadIdx.x & 31) | 3);

	// origContacts are the contacts of this thread in local space
	const PxVec4 origContacts[3] = {contacts[0], contacts[4], contacts[8]};
	__syncwarp(groupMask);

	// Contact Reduction: We must not output more than 6 contacts for the patch.
	if(totalContacts <= PXG_MAX_NUM_POINTS_PER_CONTACT_PATCH)
	{
		// Store all contacts, every thread their own (zero to three), transformed
		// back into global space
		for(PxU32 i = 0; i < count; i++)
		{
			const PxVec3 transformedContact = newTransformV.transform(origContacts[i].getXYZ());
			localContacts[runningContacts - count + i] = PxVec4(transformedContact, origContacts[i].w);
		}
	}
	else
	{
		// Similar logic to contactReduce function:
		// We keep the contact with deepest pen and 3 others that maximize the area

		// 1. Find then store the deepest contact at position 0
		{
			PxU32 deepestContactIdxThisThread = 0xffFFffFF;
			PxReal deepestContactThisThread = PX_MAX_F32;
			for(PxU32 i = 0; i < count; i++)
			{
				if(origContacts[i].w < deepestContactThisThread)
				{
					deepestContactIdxThisThread = i;
					deepestContactThisThread = origContacts[i].w;
				}
			}
			const PxU32 deepestIdx = minIndex4(deepestContactThisThread, groupMask, threadIndexInGroup);
			if((threadIdx.x & 31) == deepestIdx)
			{
				localContacts[0] = origContacts[deepestContactIdxThisThread];
			}
		}
		__syncwarp(groupMask); // ensure shared mem written
		const PxVec3 deepestPt = localContacts[0].getXYZ();

		// 2. Find then store the point furthest away from the deepest
		{
			const PxVec3 deepestPtPlane = deepestPt - incidentFaceNormalInNew * deepestPt.dot(incidentFaceNormalInNew);

			PxU32 furthestContactIdxThisThread = 0xffFFffFF;
			PxReal furthestContactThisThread = -PX_MAX_F32;
			for(PxU32 i = 0; i < count; i++)
			{
				const PxReal distSq = (deepestPtPlane - origContacts[i].getXYZ()).magnitudeSquared();
				if(distSq > furthestContactThisThread)
				{
					furthestContactIdxThisThread = i;
					furthestContactThisThread = distSq;
				}
			}
			const PxU32 furthestIdx = minIndex4(-furthestContactThisThread, groupMask, threadIndexInGroup);
			if((threadIdx.x & 31) == furthestIdx)
			{
				localContacts[1] = origContacts[furthestContactIdxThisThread];
			}
		}
		__syncwarp(groupMask); // ensure shared mem written
		const PxVec3 furthestPt = localContacts[1].getXYZ();

		// 3. Now that we have two points, find two further ones as far as possible away from that line,
		// i.e., those that are extrema to either side of the line in the plane
		{
			const PxVec3 dir = incidentFaceNormalInNew.cross(furthestPt - deepestPt);

			PxU32 maxIdxThisThread = 0xffFFffFF;
			PxU32 minIdxThisThread = 0xffFFffFF;
			PxReal maxThisThread = -PX_MAX_F32;
			PxReal minThisThread = PX_MAX_F32;
			for(PxU32 i = 0; i < count; i++)
			{
				const PxReal dist = dir.dot(origContacts[i].getXYZ() - deepestPt);
				if(dist > maxThisThread)
				{
					maxIdxThisThread = i;
					maxThisThread = dist;
				}
				if(dist < minThisThread)
				{
					minIdxThisThread = i;
					minThisThread = dist;
				}
			}
			const PxU32 minIdx = minIndex4(minThisThread, groupMask, threadIndexInGroup);
			const PxU32 maxIdx = minIndex4(-maxThisThread, groupMask, threadIndexInGroup);

			if((threadIdx.x & 31) == minIdx)
			{
				localContacts[2] = origContacts[minIdxThisThread];
			}
			if((threadIdx.x & 31) == maxIdx)
			{
				localContacts[3] = origContacts[maxIdxThisThread];
			}
		}
		__syncwarp(groupMask); // ensure shared mem written

		// 4. We have stored exactly 4 contacts. Use all 4 threads to convert them into global space
		totalContacts = 4;
		{
			const PxVec4 localSpaceContact = localContacts[threadIndexInGroup];
			localContacts[threadIndexInGroup] = PxVec4(newTransformV.transform(localSpaceContact.getXYZ()), localSpaceContact.w);
		}
	}

	assert(totalContacts <= PXG_MAX_NUM_POINTS_PER_CONTACT_PATCH);
	return totalContacts;
}


// Box vs box collision detection.
// We assume a launch config such that 4 threads check one box-box pair together.
extern "C" __global__ void boxBoxNphase_Kernel(
	PxU32 numTests,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxgShape* PX_RESTRICT shapes,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* contactDistance,
	const PxsMaterialData* PX_RESTRICT materials,
	PxU8* PX_RESTRICT contactStream,
	PxU8* PX_RESTRICT patchStream,
	//PxgPersistentContactManifold* PX_RESTRICT contactManifolds, //KS - todo - add!
	PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
	PxU32* PX_RESTRICT touchChangeFlags,
	PxU32* PX_RESTRICT patchChangeFlags,
	PxU8* PX_RESTRICT startContactPatches,
	PxU8* PX_RESTRICT startContactPoints,
	PxU8* PX_RESTRICT startContactForces,
	PxU32 patchBytesLimit,
	PxU32 contactBytesLimit,
	PxU32 forceBytesLimit,
	const PxReal toleranceLength)
{
	const PxU32 globalThreadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	const PxU32 workIndex = globalThreadIndex / 4; //4 threads per pair

	assert((blockDim.x * gridDim.x) / 4 >= numTests); // ensure we're covering all tests

	if (workIndex >= numTests)
		return;


	PxgContactManagerInput contactInput = cmInputs[workIndex];

	PxU32 transformCacheRef0 = contactInput.transformCacheRef0;
	PxU32 transformCacheRef1 = contactInput.transformCacheRef1;

	PxsCachedTransform transformCache0 = transformCache[transformCacheRef0];
	PxsCachedTransform transformCache1 = transformCache[transformCacheRef1];

	const PxReal cDistance = contactDistance[transformCacheRef0] + contactDistance[transformCacheRef1];

	PxgShape& shape0 = shapes[contactInput.shapeRef0];
	PxgShape& shape1 = shapes[contactInput.shapeRef1];

	assert(shape0.type == PxGeometryType::eBOX);
	assert(shape1.type == PxGeometryType::eBOX);

	const PxVec3 box0Extent = shape0.scale.scale;
	const PxVec3 box1Extent = shape1.scale.scale;

	const PxMat34 transform0(transformCache0.transform);
	const PxMat34 transform1(transformCache1.transform);


	//Box-box collision...

	const PxU32 NumWarps = 2;
	assert(blockDim.x == NumWarps * WARP_SIZE);

	__shared__ __align__(16) char sContacts[sizeof(TempBoxBoxBuffer<NumWarps>)];

	TempBoxBoxBuffer<NumWarps>& tempBuff = reinterpret_cast<TempBoxBoxBuffer<NumWarps>&>(*sContacts);

	// One group consists of 4 threads that collaboratively check one box-box pair
	const PxU32 threadIndexInGroup = threadIdx.x & 3;
	const PxU32 threadGroupInWarp = (threadIdx.x & 31) >> 2;
	const PxU32 groupMask = 0xf << (threadGroupInWarp * 4);

	const PxU32 threadGroupStartIndex = threadIdx.x&(~3);

	PxVec3 normal;
	PxU32 nbContacts = doBoxBoxGenerateContacts<NumWarps>(box0Extent, box1Extent,
		transform0, transform1, cDistance,
		tempBuff, normal, threadIndexInGroup, groupMask);

	//Now output the contacts!

	//__syncwarp(groupMask);

	__syncwarp();

	PxU32 contactByteOffset = 0xFFFFFFFF;

	if (threadIndexInGroup == 3)
	{
		//PxsContactManagerOutput* output = &cmOutputs[workIndex];

		contactByteOffset = setContactPointAndForcePointers(cmOutputs, patchAndContactCounters, startContactPoints, startContactForces,
			contactBytesLimit, forceBytesLimit, workIndex, nbContacts);

		PxU32 patchIndex = registerContactPatch(cmOutputs, patchAndContactCounters, touchChangeFlags, patchChangeFlags, startContactPatches, patchBytesLimit, workIndex, nbContacts);

		if (nbContacts)
		{
			insertIntoPatchStream(materials, patchStream, shape0, shape1, patchIndex, normal, nbContacts);
		}
	}


	contactByteOffset = __shfl_sync(groupMask, contactByteOffset, (threadGroupStartIndex + 3) & 31);

	//write point and penetration to the contact stream
	if (contactByteOffset != 0xFFFFFFFF)
	{
		float4* baseContactStream = ((float4*)(contactStream + contactByteOffset));
		for (PxU32 i = threadIndexInGroup; i < nbContacts; i += 4)
		{
			float4& outPointPen = baseContactStream[i];
			float4 point = reinterpret_cast<float4&>(tempBuff.tempBuff[3 * threadGroupStartIndex + i]);
			outPointPen = point;

			//printf("%i: point = (%f, %f, %f, %f)\n", i, point.x, point.y, point.z, point.w);
		}
	}
}



