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

#include "foundation/PxMat34.h"
#include <stdio.h>

#include "PxgCommonDefines.h"
#include "MemoryAllocator.cuh"

#include "cuda.h"
#include "cuda_runtime.h"
#include "convexFormat.h"
#include "cudaNpCommon.h"

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
#include "contactPatchUtils.cuh"

#include "dataReadWriteHelper.cuh"
#include "schlockShared.h"

using namespace schlock;

#include "nputils.cuh"
#include "shuffle.cuh"
#include "gjk.cuh"
#include "epa.cuh"


using namespace physx;

extern "C" __host__ void initNarrowphaseKernels6() {}

__device__ __forceinline__ static
int writeContacts(float4* manifoldNormalPen, float4* manifoldA, float4* manifoldB,
				  int mask, PxVec3 pa, PxVec3 pb, PxVec3 n,  PxReal sep)
{
	if(mask & (1<<threadIdx.x))
	{
		int index = warpScanExclusive(mask, threadIdx.x);
		manifoldNormalPen[index] = make_float4(n.x, n.y, n.z, sep);
		manifoldA[index] = make_float4(pa.x, pa.y, pa.z, 0.0f);
		manifoldB[index] = make_float4(pb.x, pb.y, pb.z, 0.0f);
	}

	return __popc(mask);
}


struct TemporaryContacts
{
	PxVec3 pos[NUM_TMP_CONTACTS_PER_PAIR];
	PxReal sep[NUM_TMP_CONTACTS_PER_PAIR];
};

__device__ __forceinline__ 
void addContacts(int flags, PxVec3 pos, PxReal sep, PxReal maxSep, TemporaryContacts& contacts, int& nbContacts)
{
	flags &= __ballot_sync(FULL_MASK, sep < maxSep);

	int index = __popc(flags & ((1 << threadIdx.x) - 1)) + nbContacts;
	if (flags & (1 << threadIdx.x) && index < NUM_TMP_CONTACTS_PER_PAIR)
	{
		contacts.pos[index] = pos, contacts.sep[index] = sep;
		assert(PxIsFinite(pos.x));
		assert(PxIsFinite(pos.y));
		assert(PxIsFinite(pos.z));
		assert(PxIsFinite(sep));
	}

	nbContacts += __popc(flags);
}


__device__ __forceinline__
int polyClip(const PxPlane plane0, PxVec3 v0, PxU32 nbVerts0,
			 const PxPlane plane1, PxVec3 v1, PxU32 nbVerts1,
			 const PxVec3 axis, const PxReal contactDist,	
			 TemporaryContacts& contacts)
{
	int tI = threadIdx.x;


	//The plane projections onto the axis
	PxReal s0 = -plane1.distance(v0)/plane1.n.dot(axis);
	PxReal s1 = plane0.distance(v1);
	
	int in0 = 0, out0 = 0;	// each element represents a point of poly 0 - the nth bit is status regarding the nth plane of poly0
	int in1 = 0, out1 = 0;	// each element represents a plane of poly 1 - the nth bit is status regarding the nth point of poly1

	int nbContacts = 0;

	// points of poly1 against planes of poly 0
	const PxVec3 nextVert = shuffle(FULL_MASK, v0, tI + 1 == nbVerts0 ? 0 : tI + 1);
	PxVec3 e0 = nextVert - v0;
	PxVec3 e0planeN = e0.cross(axis);
	PxReal e0planeD = e0planeN.dot(v0);
	for(int j=0, bj=1; j<nbVerts1; j++, bj<<=1)
	{
		const PxVec3 vert = shuffle(FULL_MASK, v1, j);
		const PxReal d = vert.dot(e0planeN);
		PxReal t = d - e0planeD;
		//PxReal t = shuffleDot(v1, j, e0planeN) - e0planeD;
		in1  |=(t<0 ? bj : 0), out1 |=(t>0 ? bj : 0);
	}

	if(tI>=nbVerts0)
		in1 = out1 = 0;

	int b = (~out1) & __shfl_xor_sync(FULL_MASK, (~out1), 16); 
	b &= __shfl_xor_sync(FULL_MASK, b,8);
	b &= __shfl_xor_sync(FULL_MASK, b,4);
	b &= __shfl_xor_sync(FULL_MASK, b,2);
	b &= __shfl_xor_sync(FULL_MASK, b,1);

	addContacts(b&((1<<nbVerts1)-1), v1-s1*axis, s1, contactDist, contacts, nbContacts);

	if(!__any_sync(FULL_MASK, out1))										// all poly1 points inside poly0, so done
		return nbContacts;

	in1 |= in1<<nbVerts1, out1 |= out1<<nbVerts1;			// save poly1's edge crosses
	int e1cross = (in1 & out1>>1) | (in1>>1 & out1);

	//points of poly0 against planes of poly 1

	PxVec3 e1 = shuffle(FULL_MASK, v1, tI+1==nbVerts1 ? 0 : tI+1) - v1;
	PxVec3 e1planeN = axis.cross(e1);
	PxReal e1planeD = e1planeN.dot(v1);

	for(int j=0; j<nbVerts0; j++)
	{
		PxReal t = shuffleDot(FULL_MASK, v0, j, e1planeN) - e1planeD;
		int in = __ballot_sync(FULL_MASK, t<0), out = __ballot_sync(FULL_MASK, t>0);
		if(j == tI)
			in0 = in, out0 = out;
	}

	in0 &= (1<<nbVerts1)-1, out0  &= (1<<nbVerts1)-1;
	addContacts(__ballot_sync(FULL_MASK, !out0) & ((1<<nbVerts0)-1), v0, s0, contactDist, contacts, nbContacts);

	if(!__any_sync(FULL_MASK, out0))										// all poly0 points inside poly1, so done
		return nbContacts;

	int in0p = __shfl_sync(FULL_MASK, in0, tI+1 == nbVerts0 ? 0 : tI+1);	
	int out0p = __shfl_sync(FULL_MASK, out0, tI+1 == nbVerts0 ? 0 : tI+1);
	int e0cross = (in0 & out0p) | (in0p & out0);			// get poly0's edge crosses
	
	// edge-edge crossings
	int c = e0cross & e1cross;  
	//assert(c+1 < 1<<nbVerts1);

	PxReal s0d = __shfl_sync(FULL_MASK, s0, tI+1 == nbVerts0 ? 0 : tI+1) - s0;

	int a = c | __shfl_xor_sync(FULL_MASK, c,16); 
	a |= __shfl_xor_sync(FULL_MASK, a,8);
	a |= __shfl_xor_sync(FULL_MASK, a,4);
	a |= __shfl_xor_sync(FULL_MASK, a,2);
	a |= __shfl_xor_sync(FULL_MASK, a,1);

	for(; a; a &= (a-1))
	{
		int index = lowestSetIndex(a);
		PxVec3 m = shuffle(FULL_MASK, e1planeN, index);
		//ML: if m and e0 is perpendicular, m.dot(e0) is zero and lambda become Nan. This cause bug in the BP
		const PxReal mDote0 = m.dot(e0);

		PxReal lambda =-(m.dot(v0) - __shfl_sync(FULL_MASK, e1planeD, index)) / mDote0;

		const PxVec3 point = v0 + lambda*e0;
		//ML: if mDote0 is zero, we set the sep to be PX_MAX_F32, this will ensure we discard this contact in the addContacts method
		const PxReal sep = (mDote0 != 0.f) ? s0 + lambda*s0d : PX_MAX_F32;

		addContacts(__ballot_sync(FULL_MASK, c & lowestSetBit(a)), point, sep, contactDist, contacts, nbContacts);
	}

	return nbContacts;
}

#if 0
template<bool TClustering, bool TKeepAnotherDeepestForPCM, int TMaxPoints> __device__ __forceinline__ 
int reduce(PxVec3 point, PxReal separation, PxVec3 normal, int nbContacts, int allMask)
{
	PxReal v, w;
	int mask;
	
	int i0 = minIndex(separation, allMask, v);										// p0 - deepest contact
	mask = 1<<i0;

	PxVec3 nxp = normal.cross(point);
	int i1 = maxIndex((shuffle(nxp,i0)-nxp).magnitudeSquared(), allMask&~mask, v);	// p1 - furthest from p0, when projected onto normal plane
	mask |= 1<<i1;
	
	//Now we have the p0-p1 edge. We try to find the point furthest from it in the normal plane.
	//For that, we look for the 2 extreme points - one to the right and one to the left
	//One maximizes [(p1 - p0) x n] * (p - p0), the other one minimizes that
	//[(p1 - p0) x n] * (p - p0) = [n x (p - p0)] * (p1 - p0) = (n x p) * p1 - (n x p) * p0  - (n x p0) * p1 + (n x p0) * p0 =
	//= k1 - k0 - k1[0], as (n x p0) * p0 = 0

	PxReal k0 = shuffleDot(point, i0, nxp), k1 = shuffleDot(point, i1, nxp); 
	PxReal d = k1 - k0 - shfl(k1, i0);

	int f = maxIndex(d, allMask&~mask, v);
	int g = minIndex(d, allMask&~mask, w);

	bool chooseMinIndex = fabs(w) > fabs(v);
	
	int i2 = chooseMinIndex ? g : f;	// take the one that's more extreme
	mask |= 1<<i2;
	
	// This is the discarded candidate. We make sure the sign is positive, if the candidate is on the other side of the edge
	//i.e. we flip the sign
	PxReal u = chooseMinIndex ? v : -w;
	int i3 = chooseMinIndex ? f : g;

	//Now we need to test p1-p2 and p2-p0 edges
	//We maximize [(p2 - p1) x n] * (p - p1) and [(p0 - p2) x n] * (p - p2)
	//
	//[(p2 - p1) x n] * (p - p1) = [(p2 - p1) x n] * (p - p2) = [n x (p - p2)] * (p2 - p1) = 
	// = (n x p) * p2 - (n x p) * p1  - (n x p2) * p2 + (n x p2) * p1 =
	//= k2 - k1 + k1[2], as (n x p2) * p2 = 0
	//
	//[(p0 - p2) x n] * (p - p2) = [n x (p - p2)] * (p0 - p2) = (n x p) * p0 - (n x p) * p2  - (n x p2) * p0 + (n x p2) * p2 =
	//= k0 - k2 - k0[2], as (n x p2) * p2 = 0
	//
	//if p2 was on the +ve side of the p1-p0 edge, we reverse the order of the triangle, this flipping the signs
	//if fabs(w) > fabs(v), i.e. we chose the max direction, then p2 was on the +ve side   
	
	PxReal k2 = shuffleDot(point, i2, nxp);
	PxReal b0 = k1 - k2 - shfl(k1,i2);						
	PxReal b1 = k2 - k0 + shfl(k0,i2);						
	d = chooseMinIndex ? -fminf(b0,b1) : fmaxf(b0,b1);							
	
	PxReal u_;
	int i3_ = maxIndex(d, allMask&~mask, u_);

	//now account for the 3d edge 
	if (u_ > u)
	{
		u = u_;
		i3 = i3_;
	}
	
	if(u > 0)																		// if no point adds area, just 3 clusters
		mask |= 1 << i3;		
	
	if(TKeepAnotherDeepestForPCM && __popc(mask) == 4)
	{
		assert(nbContacts == 5);

		int notChosenIndex = lowestSetIndex(~mask);
		PxReal dummy;
		int furthestIndex = maxIndex((shuffle(point, notChosenIndex) - point).magnitudeSquared(), mask, dummy);	
		
		// if we are adbout to reject a deeper contact
		if (shfl(separation, notChosenIndex) < shfl(separation, furthestIndex))
		{
			mask |= 1 << notChosenIndex;
			mask &= ~(1 << furthestIndex);
		}
	}
	
	// post-cull clustering for mesh collisions
	if(TClustering)
	{	
		int nbClusters = 0, label;													// label each point with its closest cluster (distance measured orthogonal to the normal)
		for(PxReal t = FLT_MAX; mask; nbClusters++, mask &= (mask-1))
		{
			PxReal d = (nxp - shuffle(nxp, lowestSetIndex(mask))).magnitudeSquared();
			if(d<t)
				t = d, label = nbClusters;
		}

		for(int i=0;i<nbClusters;i++)												// find a point in each cluster (clusters can be empty if all input points are equal)
		{
			int cluster = __ballot(label == i)&allMask;
			if(cluster)
				mask |= 1<<minIndex(separation, cluster, v);
		}

		for(int i=nbClusters; i<TMaxPoints;i++)										// fill out the rest of the points
			mask |= 1<<minIndex(separation, allMask&~mask, v);
	}

	return mask;
}

#elif 0

/*
The logic in this funtion is:
(1)get the deepest point store in mContactPoints[0]
(2)get the furthest away point from mContactPoints[0] and store in mContactPoints[1]
(3)calculate the min and max distance point away the segment (mContactPoints[0] and mContactPoints[1]) and store the max distance point to mContactPoints[2]
(4)if the min and max distance on the same side of the segment, we need to chose the min distance point again and store this point to mContactPoints[3];
(5)cluster around that 4 points and chose the deepest points
(6)reassign contact points
*/
template<bool TClustering, bool TKeepAnotherDeepestForPCM, int TMaxPoints> __device__ __forceinline__
int reduce(PxVec3 point, PxReal separation, PxVec3 normal, int nbContacts, int allMask)
{
	PxReal v, w;
	int mask;

	int i0 = minIndex(separation, allMask, v);										// p0 - deepest contact
	mask = 1 << i0;


	const PxVec3 p0 = shuffle(FULL_MASK, point, i0);

	int i1 = maxIndex((p0 - point).magnitudeSquared(), allMask&~mask, v);	// p1 - furthest from p0, when projected onto normal plane
	mask |= 1 << i1;

	assert(i0 != i1);

	//Now we have the p0-p1 edge. We try to find the point furthest from it in the normal plane.
	//For that, we look for the 2 extreme points - one to the right and one to the left

	const PxVec3 p1 = shuffle(FULL_MASK, point, i1);
	const PxVec3 p01 = (p1 - p0);
	PxVec3 n = (p01.cross(normal)).getNormalized();

	const PxVec3 wDir = point - p0;
	PxReal d = n.dot(wDir);

	int f = maxIndex(d, allMask&~mask, v);
	int g = minIndex(d, allMask&~mask, w);

	bool chooseMinIndex = fabs(w) > fabs(v);

	int i2 = chooseMinIndex ? g : f;	// take the one that's more extreme
	mask |= 1 << i2;

	//ML: if those two extreme points are on the same side, we need to re-choose another point which will be on the side has maximum absolute value. 
	const PxReal wv = w *v;

	//ML: initialize i3 to be one of the extreme point. If this extreme point is on the same side of another extreme point, we need to overwrite this
	//value and get the second extreme point from the same side
	int i3 = chooseMinIndex ? f : g;

	//if chooseMinIndex is true, we flip the normal and calculate the projection again
	d = chooseMinIndex ? (-n).dot(wDir) : d;

	//recalcalute the min and max for the remainging contacts
	int h = maxIndex(d, allMask&~mask, v);
	
	i3 = wv > 0.f ? h : i3;

	mask |= 1 << i3;

	// post-cull clustering for mesh collisions
	if (TClustering)
	{
		
		int nbClusters = 0, label;													// label each point with its closest cluster
		for (PxReal t = FLT_MAX; mask; nbClusters++, mask &= (mask - 1))
		{
			PxReal d = (point - shuffle(FULL_MASK, point, lowestSetIndex(mask))).magnitudeSquared();
			if (d<t)
				t = d, label = nbClusters;
		}

		for (int i = 0; i<nbClusters; i++)												// find a point in each cluster (clusters can be empty if all input points are equal)
		{
			int cluster = __ballot_sync(FULL_MASK, label == i)&allMask;
			if (cluster)
				mask |= 1 << minIndex(separation, cluster, v);
		}

		for (int i = nbClusters; i<TMaxPoints; i++)										// fill out the rest of the points
			mask |= 1 << minIndex(separation, allMask&~mask, v);
	}

	return mask;
}
#else

template<bool TClustering, bool TKeepAnotherDeepestForPCM, int TMaxPoints>
static __device__ int reduce(PxVec3 point, PxReal separation, PxVec3 normal, int nbContacts, int allMask)
{
	PxReal v, w;
	int mask;

	/*bool imADupe = false;
	for(int m = allMask; m != 0; m &= (m-1))
	{
		int index = __ffs(m);
		PxReal d = (shuffle(FULL_MASK, point, index) - point).magnitudeSquared();
		PxReal sep = __shfl_sync(FULL_MASK, separation, index);
		if (d < 1e-8f && (sep < separation || (sep == separation && index > threadIdx.x)))
		{
			imADupe = true;
		}
	}

	allMask &= ~(__ballot_sync(FULL_MASK, imADupe));

	if (__popc(allMask) <= TMaxPoints)
	{
		return allMask;
	}*/


	int i0 = minIndex(separation, allMask, v);										// p0 - deepest contact
	mask = 1 << i0;

	int i1 = maxIndex((shuffle(FULL_MASK, point, i0) - point).magnitudeSquared(), allMask&~mask, v);	// p1 - furthest from p0, when projected onto normal plane
	mask |= 1 << i1;

	//Now we have the p0-p1 edge. We try to find the point furthest from it in the normal plane.
	//For that, we look for the 2 extreme points - one to the right and one to the left
	//One maximizes [(p1 - p0) x n] * (p - p0), the other one minimizes that
	//[(p1 - p0) x n] * (p - p0) = [n x (p - p0)] * (p1 - p0) = (n x p) * p1 - (n x p) * p0  - (n x p0) * p1 + (n x p0) * p0 =
	//= k1 - k0 - k1[0], as (n x p0) * p0 = 0

	PxVec3 dir = normal.cross(shuffle(FULL_MASK, point, i1) - shuffle(FULL_MASK, point, i0));

	PxReal d = dir.dot(point - shuffle(FULL_MASK, point, i0));

	int f = maxIndex(d, allMask&~mask, v);
	mask |= (1 << f);

	int g = minIndex(d, allMask&~mask, w);

	if (__shfl_sync(FULL_MASK, d, f) * __shfl_sync(FULL_MASK, d, g) > 0.f)
	{
		//We need to pick again...
		g = maxIndex(d, allMask&~mask, v);
	}

	mask |= (1 << g);

	// post-cull clustering for mesh collisions
	//unsigned mask_TClustering = __ballot_sync(syncMask, TClustering);
	if (TClustering)
	{
		int nbClusters = 0, label;													// label each point with its closest cluster (distance measured orthogonal to the normal)
		for (PxReal t = FLT_MAX; mask; nbClusters++, mask &= (mask - 1))
		{
			PxReal d = (point - shuffle(FULL_MASK, point, lowestSetIndex(mask))).magnitudeSquared();
			if (d < t)
				t = d, label = nbClusters;
		}

		for (int i = 0; i < nbClusters; i++)												// find a point in each cluster (clusters can be empty if all input points are equal)
		{
			int cluster = __ballot_sync(FULL_MASK, label == i)&allMask;
			if (cluster)
				mask |= 1 << minIndex(separation, cluster, v);
		}

		for (int i = nbClusters; i < TMaxPoints; i++)										// fill out the rest of the points
			mask |= 1 << minIndex(separation, allMask&~mask, v);
	}

	return mask;
}



#endif

template<bool TClustering, bool TKeepAnotherDeepestForPCM, int TMaxPoints> __device__ __forceinline__
int reduce(PxVec3 point, PxReal separation, PxVec3 normal, int nbContacts)
{
	return reduce<TClustering, TKeepAnotherDeepestForPCM, TMaxPoints>(point, separation, normal, nbContacts, (1 << nbContacts) - 1);
}


/*
	calculate local normal in the manifold. Manifold has maximum 4 contacts
*/

__device__ __forceinline__ static
PxVec3 calculateLocalNormal(float4* manifoldNormalPen, const PxU8 numContacts)
{
	assert(numContacts <= PXG_MAX_PCM_CONTACTS);

	const PxU32 tI = threadIdx.x;
	PxVec3 localNormal(0.f);
	if (tI < numContacts)
	{
		float4 localNormal4 = manifoldNormalPen[tI];
		localNormal = PxVec3(localNormal4.x, localNormal4.y, localNormal4.z);
	}

	unsigned mask_tI = __ballot_sync(FULL_MASK, tI < PXG_MAX_PCM_CONTACTS);
	if (tI < PXG_MAX_PCM_CONTACTS)
	{
		localNormal += shuffle(mask_tI, localNormal, tI + 1, PXG_MAX_PCM_CONTACTS);
		localNormal += shuffle(mask_tI, localNormal, tI + 2, PXG_MAX_PCM_CONTACTS);
	}
	
	// AD: this __syncwarp() fixes an issue where we had timeouts in debug UTs.
	// I had a look at the assembly and the version without syncwarp relies on a 
	// BSSY/BSYNC pattern for convergence, but I don't really know why this is not good enough.
	//
	// what I know is that __shfl_sync does not imply a memory barrier, so that might 
	// be part of the reason things break here. That does not explain why we deadlock though.
	//
	// At runtime, what I saw is that the threads that did not do the reduction above ran 
	// ahead and things go somehow out of sync, and I verified that it could also be fixed
	// by just having the whole warp do the reduction.
	__syncwarp();

	localNormal = shuffle(FULL_MASK, localNormal, 0);

	return localNormal.getNormalized();
}

/*
	If a new point and the exisitng point's distance are within some replace breaking threshold, we will replace the existing point with the new point. This is used for
	incremental manifold strategy.
	*/

__device__ __forceinline__ static
void addManifoldPoint(PxVec3 localPointA, PxVec3 localPointB, PxVec3 normal, PxReal sep, PxReal replaceBreakingThreshold,
					   float4* manifoldNormalPen,
					   float4* manifoldA,
					   float4* manifoldB,
					   PxU8& numContacts)
{
	const PxU32 tI = threadIdx.x;
	
	bool match = false;
	if(tI < numContacts)
	{
		const float4 b = manifoldB[tI];
		const float4 a = manifoldA[tI];
		const PxReal sqReplaceBreakingThreshold = replaceBreakingThreshold * replaceBreakingThreshold;
		const bool bMatch = (PxVec3(b.x, b.y, b.z) - localPointB).magnitudeSquared() < sqReplaceBreakingThreshold;
		const bool aMatch = (PxVec3(a.x, a.y, a.z) - localPointA).magnitudeSquared() < sqReplaceBreakingThreshold;
		match = aMatch | bMatch;
	}

	int matches = __ballot_sync(FULL_MASK, match);

	if(matches || numContacts < 4)
	{
		int index =  matches ? lowestSetIndex(matches) : numContacts++;
		
		if(tI == 0)
		{
			manifoldNormalPen[index] = make_float4(normal.x, normal.y, normal.z, sep);
			manifoldA[index] = make_float4(localPointA.x, localPointA.y, localPointA.z, 0.0f);
			manifoldB[index] = make_float4(localPointB.x, localPointB.y, localPointB.z, 0.0f);
		}

		return;
	}
	
	if(tI<4)
	{
		float4 a = manifoldA[tI], np =  manifoldNormalPen[tI];
		float4 b = manifoldB[tI];
		localPointA = PxVec3(a.x, a.y, a.z);
		localPointB = PxVec3(b.x, b.y, b.z);
		normal = PxVec3(np.x, np.y, np.z); 
		sep = np.w;
	}

	int result = reduce<false, false, 4>(localPointB, sep, normal, 5);
	assert(__popc(result) <= 4);
	numContacts = writeContacts(manifoldNormalPen, manifoldA, manifoldB, result, localPointA, localPointB, normal, sep);
	assert(numContacts <= 4);
}

__device__ __forceinline__ PxVec3 loadNormal(const float4& p, const PxVec3&scale, const PxQuat& rot)
{
	const float4 fplane = p;
	return shape2Vertex(PxVec3(fplane.x, fplane.y, fplane.z), scale, rot).getNormalized();
}


__device__ inline static
PxI32 getPolygonIndexFromLocalWitness(
	PxU32 polyData_NbPolygons,
	const float4* pPolygonPlanes,
	const PxVec3& scale, 
	const PxQuat& rot,
	const PxVec3& witnessPt,			//shapespace
	const PxVec3& n,					//shape space
	const PxReal radius,
	const PxReal toleranceLength)
{
	const PxReal margin = radius * 0.01;
	const PxReal lowerEps = toleranceLength * PCM_WITNESS_POINT_LOWER_EPS;
	const PxReal upperEps = toleranceLength * PCM_WITNESS_POINT_UPPER_EPS;
	const PxReal tolerance = PxClamp(margin, lowerEps, upperEps);

	const PxU32 threadIdxInGroup = threadIdx.x;

	//witnessPt is in shape space, need to transform the vertex space
	PxVec3 localWitness = shape2Vertex(witnessPt, scale, rot);

	//normal is in shape space, don't transfer to vertex space. If the shape has huge scale,
	//keep the normal in shape space and transfer plane normal to shape give us a much reliable
	//result

	PxReal bestProjection = PX_MAX_F32;
	PxReal minDist = PX_MAX_F32;
	PxReal maxDist = -PX_MAX_F32;
	PxU32 index = 0;

	const PxReal eps = -tolerance;

	for (PxU32 i = threadIdxInGroup; i < polyData_NbPolygons; i += WARP_SIZE)
	{
		float4 fplane = pPolygonPlanes[i];
		PxPlane plane = PxPlane(fplane.x, fplane.y, fplane.z, fplane.w);
		PxReal pd = plane.distance(localWitness);// abs(plane.d + plane.n.dot(localWitness));
		PxReal toleranceDist = pd >= eps ? pd : PX_MAX_F32;

	
		minDist = PxMin(toleranceDist, minDist);
		if (toleranceDist > maxDist)
		{
			maxDist = toleranceDist;
			index = i;
		}
	}

	PxU32 minDistLane;
	minDist = warpReduction<MinOpFloat, float>(FULL_MASK, minDist, minDistLane);

	if (minDist == PX_MAX_F32)
	{
		PxU32 maxDistLane;
		maxDist = warpReduction<MaxOpFloat, float>(FULL_MASK, maxDist, maxDistLane);
		index = __shfl_sync(FULL_MASK, (int)index, maxDistLane);
		return index;
	}

	for (PxU32 i = threadIdxInGroup; i < polyData_NbPolygons; i += WARP_SIZE)
	{
		float4 fplane = pPolygonPlanes[i];
		PxPlane plane = PxPlane(fplane.x, fplane.y, fplane.z, fplane.w);
		PxReal pd = plane.distance(localWitness);
		PxReal toleranceDist = pd >= eps ? pd : PX_MAX_F32;

		//if the difference between the minimum distance and the distance of p to plane i is within tolerance, we use the normal to chose the best plane 
		if (tolerance >(toleranceDist - minDist))
		{
			//transform plane normal to shape shapce
			PxVec3 planeN = shape2Vertex(plane.n, scale, rot).getNormalized();
			PxReal proj = planeN.dot(n);

			if (bestProjection > proj)
			{
				index = i;
				bestProjection = proj;
			}
		}
	}

	PxU32 bestProjLane;
	bestProjection = warpReduction<MinOpFloat, float>(FULL_MASK, bestProjection, bestProjLane);

	index = __shfl_sync(FULL_MASK, (int)index, bestProjLane);
	assert(index != 0xffffffff);
	return index;
}


struct CollideScratch
{
	PxVec3 vA[CONVEX_MAX_VERTICES_POLYGONS];
	PxVec3 vB[CONVEX_MAX_VERTICES_POLYGONS];
	GjkCachedData cachedData;
	GjkOutput gjkOutput;

	PxQuat rot0;
	PxVec3 scale0; 
	
	PxQuat rot1;
	PxVec3 scale1; 
		
	PxTransform aToB;
	PxTransform tB;
	
	float4* manifoldContactPointsLocalNormalPen;
	float4* manifoldContactPointsA;
	float4* manifoldContactPointsB;

	const PxU8* convexPtr0;
	const PxU8* convexPtr1; 

	PxU8 nbVertices0;
	PxU8 nbPolygons0;
	PxU16 nbEdges0;
	PxU8 nbVertices1;
	PxU8 nbPolygons1;
	PxU16 nbEdges1;

	PxReal contactDistance;
	PxReal toleranceLength;
	PxReal replaceBreakingThreshold;

	PxU32 initialNbContacts;

	PxReal inSphereRadius0;
	PxReal inSphereRadius1;
	PxU32 shape0MaterialIndex;
	PxU32 shape1MaterialIndex;
	
	//for debugging
	//PxU32 transformCache0;
	//PxU32 transformCache1;

	PxGeometryType::Enum type0;
	PxGeometryType::Enum type1;

	__device__ void Clear()
	{
		if(threadIdx.x == 0)
			cachedData.size = 0;
		__syncwarp();
	}
};

template<bool Cnd, int A, int B>
struct compileTimeSelect {};

template<int A, int B>
struct compileTimeSelect<true, A, B> {enum {value = A};};

template<int A, int B>
struct compileTimeSelect<false, A, B>{enum {value = B};};


template<int A, int B>
struct compileTimeSelectLargest
{
	enum {
		value = compileTimeSelect<(A > B), A, B > ::value
	};
};

struct EpaAndClipScratch
{
	PxU8 mem[compileTimeSelectLargest<sizeof(squawk::EpaScratch) , sizeof(TemporaryContacts)>::value];
};


__device__ inline static GjkResult::Enum runGJK(const PxVec3 initialDir, PxU32& persistentContactManifoldMoreDataW,
	CollideScratch& ss_scratch, const PxReal convergenceRatio)
{
	
	GjkResult::Enum gjkResult = squawk::gjk(
		ss_scratch.vA, ss_scratch.nbVertices0,
		ss_scratch.vB, ss_scratch.nbVertices1,
		initialDir,
		ss_scratch.contactDistance,
		convergenceRatio,
		ss_scratch.gjkOutput, ss_scratch.cachedData);

	__syncwarp();


	PxU32 warmStartAIndices = 0;
	PxU32 warmStartBIndices = 0;

	if (threadIdx.x < ss_scratch.cachedData.size)
	{
		CachedVertex cv = ss_scratch.cachedData.vertices[threadIdx.x];
		PxU8 aIndex = cv.a();
		warmStartAIndices |= (aIndex << (threadIdx.x << 3));
		PxU8 bIndex = cv.b();
		warmStartBIndices |= (bIndex << (threadIdx.x << 3));
	}

	warmStartAIndices = warpReduction<OrOpPxU32, int>(FULL_MASK, warmStartAIndices);
	warmStartBIndices = warpReduction<OrOpPxU32, int>(FULL_MASK, warmStartBIndices);

	__syncwarp();

	if (threadIdx.x >= 13 && threadIdx.x < 16)
	{
		persistentContactManifoldMoreDataW = threadIdx.x == 13 ? ss_scratch.cachedData.size :
			(threadIdx.x == 14 ? warmStartAIndices : warmStartBIndices);
	}

	return gjkResult;
}

__device__ static
bool generateConvexContacts(EpaAndClipScratch& ss_epa_clip_scratch, CollideScratch& ss_scratch,
	PxU32& persistentContactManifoldMoreDataW,
	PxU8& numManifoldContacts)
{
	PxVec3 normal = ss_scratch.gjkOutput.direction;

	const float4* pPolygonPlanes0 = (const float4*)(ss_scratch.convexPtr0 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices0);

	const PxU32* vRef8NbVertsMinIndex0 = (const PxU32*)(ss_scratch.convexPtr0 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices0
		+ sizeof(float4) * ss_scratch.nbPolygons0);

	const PxU8* polyData0_vertexData8 = (const PxU8*)(ss_scratch.convexPtr0 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices0
		+ (sizeof(float4) + sizeof(PxU32)) * ss_scratch.nbPolygons0
		+ (sizeof(PxU16) + sizeof(PxU8)) * 2 * ss_scratch.nbEdges0
		+ sizeof(PxU8) * 3 * ss_scratch.nbVertices0);


	const float4* pPolygonPlanes1 = (const float4*)(ss_scratch.convexPtr1 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices1);

	const PxU32* vRef8NbVertsMinIndex1 = (const PxU32*)(ss_scratch.convexPtr1 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices1
		+ sizeof(float4) * ss_scratch.nbPolygons1);

	const PxU8* polyData1_vertexData8 = (const PxU8*)(ss_scratch.convexPtr1 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices1
		+ (sizeof(float4) + sizeof(PxU32)) * ss_scratch.nbPolygons1
		+ (sizeof(PxU16) + sizeof(PxU8)) * 2 * ss_scratch.nbEdges1
		+ sizeof(PxU8) * 3 * ss_scratch.nbVertices1);



	//ML: we need to add the gjk/epa contacts to the manifold first. If the number of contacts after GJK/EPA in the manifold is still less than the initial number of
	//contacts, we need to trigger a full contact gen. Otherwise, we will lose contacts and will never regenerate contacts again.
	PxVec3 witnessA = ss_scratch.gjkOutput.closestPointA;
	PxVec3 witnessB = ss_scratch.gjkOutput.closestPointB;

	//calculate the local normal before GJK/EPA add to the manifold, we are using the number of contacts after refresh
	PxVec3 localNor(0.f);
	if (numManifoldContacts)
		localNor = calculateLocalNormal(ss_scratch.manifoldContactPointsLocalNormalPen, numManifoldContacts);

	assert(PxIsFinite(witnessB.x));
	assert(PxIsFinite(witnessB.y));
	assert(PxIsFinite(witnessB.z));

	addManifoldPoint(ss_scratch.aToB.transformInv(witnessA),
		witnessB,
		-normal,
		(witnessB - witnessA).dot(normal),
		ss_scratch.replaceBreakingThreshold,
		ss_scratch.manifoldContactPointsLocalNormalPen,
		ss_scratch.manifoldContactPointsA,
		ss_scratch.manifoldContactPointsB,
		numManifoldContacts);

	PxU32 nbContacts = 0;

	//ML: after we refresh the contacts(newContacts) and generate a GJK/EPA contacts(we will store that in the manifold), if the number of contacts is still less than the original contacts,
	//which means we lose too many contacts and we should regenerate all the contacts in the current configuration
	//const bool fullContactGen = (0.707106781f > localNor.dot(-normal)) || (numManifoldContacts < ss_scratch.initialNbContacts);
	const bool fullContactGen = (0.707106781f > localNor.dot(-normal)) || (numManifoldContacts < 4);
	
	if (fullContactGen)
	{
		const PxVec3 normalIn0 = ss_scratch.aToB.rotateInv(-normal);
		PxI32 bestPlaneAIdx = getPolygonIndexFromLocalWitness(
			ss_scratch.nbPolygons0,
			pPolygonPlanes0,
			ss_scratch.scale0, ss_scratch.rot0,
			ss_scratch.aToB.transformInv(ss_scratch.gjkOutput.closestPointA),
			ss_scratch.aToB.rotateInv(-normal),
			ss_scratch.inSphereRadius0,
			ss_scratch.toleranceLength);


		PxI32 bestPlaneBIdx = getPolygonIndexFromLocalWitness(
			ss_scratch.nbPolygons1,
			pPolygonPlanes1,
			ss_scratch.scale1, ss_scratch.rot1,
			ss_scratch.gjkOutput.closestPointB,
			normal,
			ss_scratch.inSphereRadius1,
			ss_scratch.toleranceLength);

		const float4 referencePolygon = pPolygonPlanes1[bestPlaneBIdx];
		PxPlane referencePolygon_plane(shape2Vertex(PxVec3(referencePolygon.x, referencePolygon.y, referencePolygon.z),
			ss_scratch.scale1, ss_scratch.rot1), referencePolygon.w);

		referencePolygon_plane.normalize();

		const float4 incidentPolygon = pPolygonPlanes0[bestPlaneAIdx];

		PxPlane incidentPolygon_plane(shape2Vertex(PxVec3(incidentPolygon.x, incidentPolygon.y, incidentPolygon.z),
			ss_scratch.scale0, ss_scratch.rot0), incidentPolygon.w);

		incidentPolygon_plane.normalize();

		const PxReal referenceProject = PxAbs(referencePolygon_plane.n.dot(normal));
		const PxReal incidentProject = PxAbs(incidentPolygon_plane.n.dot(normalIn0));

		const PxU32 polyDesc0 = vRef8NbVertsMinIndex0[bestPlaneAIdx];
		const PxU32 polyDesc1 = vRef8NbVertsMinIndex1[bestPlaneBIdx];

		//transform incidentPolygon_plane into referencePolygon_plane space
		PxTransform& aToB = ss_scratch.aToB;
		incidentPolygon_plane = incidentPolygon_plane.transform(aToB);
		incidentPolygon_plane.normalize();

		PxReal orthoEps = 1e-6;

		if (fabs(normal.dot(referencePolygon_plane.n)) >= orthoEps && fabs(normal.dot(incidentPolygon_plane.n)) >= orthoEps)
		{
			TemporaryContacts* contactsS = reinterpret_cast<TemporaryContacts*>(&ss_epa_clip_scratch);
			PxVec3 verts0, verts1;

			if (threadIdx.x < getNbVerts(polyDesc0))
				verts0 = ss_scratch.vA[polyData0_vertexData8[getVRef8(polyDesc0) + threadIdx.x]];

			if (threadIdx.x < getNbVerts(polyDesc1))
				verts1 = ss_scratch.vB[polyData1_vertexData8[getVRef8(polyDesc1) + threadIdx.x]];

			assert(getNbVerts(polyDesc0) <= 32);
			assert(getNbVerts(polyDesc1) <= 32);

			if ((referenceProject > incidentProject))
			{

				nbContacts = polyClip(referencePolygon_plane, verts1, getNbVerts(polyDesc1),
					incidentPolygon_plane, verts0, getNbVerts(polyDesc0),
					referencePolygon_plane.n, ss_scratch.contactDistance, *contactsS);

				__syncwarp();

				if (nbContacts)
				{
					PxVec3 pb;
					PxReal sep;

					if (threadIdx.x < nbContacts)
						pb = contactsS->pos[threadIdx.x], sep = contactsS->sep[threadIdx.x];

					int mask = (1 << nbContacts) - 1;

					if (nbContacts > 4)
						mask = reduce<false, false, 4>(pb, sep, referencePolygon_plane.n, nbContacts);

					assert(__popc(mask) <= 4);

					PxVec3 pa = pb + referencePolygon_plane.n * sep;


					numManifoldContacts = writeContacts(ss_scratch.manifoldContactPointsLocalNormalPen, ss_scratch.manifoldContactPointsA,
						ss_scratch.manifoldContactPointsB, mask, aToB.transformInv(pa), pb, referencePolygon_plane.n, sep);

					return true;
				}
			}
			else
			{
				//Both planes are in the B space
				PxPlane referencePlane = incidentPolygon_plane;
				PxPlane incidentPlane = referencePolygon_plane;

				nbContacts = polyClip(referencePlane, verts0, getNbVerts(polyDesc0), incidentPlane, verts1, getNbVerts(polyDesc1), referencePlane.n, ss_scratch.contactDistance, *contactsS);

				__syncwarp();

				if (nbContacts)
				{
					PxVec3 pa;
					PxReal sep;

					if (threadIdx.x < nbContacts)
						pa = contactsS->pos[threadIdx.x], sep = contactsS->sep[threadIdx.x];

					int mask = (1 << nbContacts) - 1;

					if (nbContacts > 4)
						mask = reduce<false, false, 4>(pa, sep, referencePlane.n, nbContacts);

					assert(__popc(mask) <= 4);

					PxVec3 pb = pa + referencePlane.n * sep;


					numManifoldContacts = writeContacts(ss_scratch.manifoldContactPointsLocalNormalPen, ss_scratch.manifoldContactPointsA,
						ss_scratch.manifoldContactPointsB, mask, aToB.transformInv(pa), pb, -referencePlane.n, sep);

					return true;
				}
			}
		}
	}

	return true;
}


__device__ bool intersectRayPolyhedron(const PxVec3& a, const PxVec3& dir, const PxU32 threadIndexInWarp,
	CollideScratch& ss_scratch, PxReal& tEnter, PxReal& tExit)
{
	const PxReal eps = 1e-7f;
	PxReal tFirst = 0.f;
	PxReal tLast = PX_MAX_F32;


	const float4* pPolygonPlanes1 = (const float4*)(ss_scratch.convexPtr1 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices1);

	const PxU32 nbPolygons = ss_scratch.nbPolygons1;

	for (PxU32 i = 0; i < nbPolygons; i += WARP_SIZE)
	{
		PxU32 k = i + threadIndexInWarp;

		bool separation = false;

		if (k < nbPolygons)
		{
			float4 plane = pPolygonPlanes1[k];
			const PxVec3 n = PxVec3(plane.x, plane.y, plane.z);

			PxReal d = plane.w;

			const PxReal denominator = n.dot(dir);
			const PxReal distToPlane = n.dot(a) + d;

			/*printf("%i: n = %f, %f, %f, d = %f\n", k, n.x, n.y, n.z, d);
			printf("%i: dir = %f, %f, %f, d = %f\n", k, dir.x, dir.y, dir.z, d);
			printf("%i: a = %f, %f, %f\n", k, a.x, a.y, a.z);

			printf("%i: denom = %f, distToPlane = %f\n",k, denominator, distToPlane);*/

			if (PxAbs(denominator) < eps)
			{
				separation = distToPlane > 0.f;
			}
			else
			{
				PxReal tTemp = -distToPlane / denominator;

				//ML: denominator < 0 means the ray is entering halfspace; denominator > 0 means the ray is exiting halfspace
				const bool con = 0.f > denominator;
				const bool con0 = tTemp > tFirst;
				const bool con1 = tLast > tTemp;

				tFirst = (con && con0) ? tTemp : tFirst;
				tLast = (con1 && (!con)) ? tTemp : tLast;

				//printf("k = %i, tFirst = %f, tLast = %f, tTemp = %f, con = %i, con0 = %i, con1 = %i\n", k, tFirst, tLast, tTemp, con, con0, con1);
			}

			
		}

		if (__ballot_sync(FULL_MASK, separation))
			return false;

		tFirst = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, tFirst);
		tLast = warpReduction<MinOpFloat, PxReal>(FULL_MASK, tLast);

		if (tFirst > tLast)
			return false;
	}

	//calculate the intersect p in the local space
	tEnter = tFirst;
	tExit = tLast;

	return true;
}

static __device__ PxU32 generatedFaceContacts(CollideScratch& ss_scratch, const PxU32 threadIndexInWarp)
{

	PxReal tEnter = 0.f, tExit = 0.f;
	const PxReal inflatedRadius = ss_scratch.contactDistance;
	const PxVec3 dir = -ss_scratch.gjkOutput.direction;

	PxVec3 vertexSpaceDir = shape2Vertex(-dir, ss_scratch.scale1, ss_scratch.rot1);
	const PxVec3 p0 = shape2Vertex(ss_scratch.vA[0], ss_scratch.scale1, ss_scratch.rot1);
	const PxVec3 p1 = shape2Vertex(ss_scratch.vA[1], ss_scratch.scale1, ss_scratch.rot1);

	PxU32 numContacts = 0;

	if (intersectRayPolyhedron(p0, vertexSpaceDir, threadIndexInWarp, ss_scratch, tEnter, tExit) && inflatedRadius>=tEnter)
	{
		if (threadIndexInWarp == 0)
		{
			PxVec3 localVa = ss_scratch.vA[0];
			PxVec3 va = ss_scratch.aToB.transformInv(localVa);
			PxVec3 vb = ss_scratch.vA[0] - dir * tEnter;
			ss_scratch.manifoldContactPointsA[numContacts] = make_float4(va.x, va.y, va.z, 0.f);
			ss_scratch.manifoldContactPointsB[numContacts] = make_float4(vb.x, vb.y, vb.z, 0.f);
			ss_scratch.manifoldContactPointsLocalNormalPen[numContacts] = make_float4(dir.x, dir.y, dir.z, tEnter);
		}
		numContacts++;
	}

	if (intersectRayPolyhedron(p1, vertexSpaceDir, threadIndexInWarp, ss_scratch, tEnter, tExit) && inflatedRadius>= tEnter)
	{
		if (threadIndexInWarp == 0)
		{
			PxVec3 localVa = ss_scratch.vA[1];
			PxVec3 va = ss_scratch.aToB.transformInv(localVa);
			PxVec3 vb = ss_scratch.vA[1] - dir * tEnter;
			ss_scratch.manifoldContactPointsA[numContacts] = make_float4(va.x, va.y, va.z, 0.f);
			ss_scratch.manifoldContactPointsB[numContacts] = make_float4(vb.x, vb.y, vb.z, 0.f);
			ss_scratch.manifoldContactPointsLocalNormalPen[numContacts] = make_float4(dir.x, dir.y, dir.z, tEnter);
		}
		numContacts++;
	}

	return numContacts;
}


static __device__ bool generateEE(const PxVec3& p, const PxVec3& q, const PxVec3& normal, const PxVec3& a, const PxVec3& b, 
	const PxTransform& aToB, const PxReal inflatedRadius, PxVec3& glocalPointA, PxVec3& glocalPointB, PxVec4& glocalNormalPen)
{
	const PxReal expandedRatio = 0.005f;
	const PxVec3 ab = b - a;
	const PxVec3 n = ab.cross(normal);
	const PxReal d = n.dot(a);
	const PxReal np = n.dot(p);
	const PxReal nq = n.dot(q);
	const PxReal signP = np - d;
	const PxReal signQ = nq - d;
	const PxReal temp = signP * signQ;
	if (temp > 0.f) 
		return false;

	// if colliding edge (p3,p4) and plane are parallel return no collision
	const PxVec3 pq = q - p;
	const PxReal npq = n.dot(pq);
	if (npq== 0.f)	
		return false;


	//calculate the intersect point in the segment pq with plane n(x - a).
	const PxReal segTValue = (d - np)/ npq;
	const PxVec3 localPointA = pq*segTValue + p;

	//ML: ab, localPointA and normal is in the same plane, so that we can do 2D segment segment intersection
	//calculate a normal perpendicular to ray localPointA + normal*t, then do 2D segment segment intersection
	const PxVec3 perNormal = normal.cross(pq);
	const PxVec3 ap = localPointA - a;
	const PxReal nom = perNormal.dot(ap);
	const PxReal denom = perNormal.dot(ab);

	const PxReal tValue = nom/denom;

	const PxReal max = 1.f + expandedRatio;
	const PxReal min = -expandedRatio;
	if (tValue > max || min > tValue)
		return false;

	const PxVec3 v = ap - ab*tValue;
	const PxReal signedDist = v.dot(normal);
	if (inflatedRadius >= signedDist)
	{
		glocalPointB = localPointA -v;
		glocalPointA = aToB.transformInv(localPointA);
		glocalNormalPen = PxVec4(normal.x, normal.y, normal.z, signedDist);
		return true;

	}

	return false;
}

__device__ PxU32 generatedContactsEEContacts(CollideScratch& ss_scratch, const PxU32 threadIndexInWarp, PxU32 numContacts)
{

	PxVec3 normal = ss_scratch.gjkOutput.direction;

	const float4* pPolygonPlanes1 = (const float4*)(ss_scratch.convexPtr1 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices1);

	const PxU32* vRef8NbVertsMinIndex1 = (const PxU32*)(ss_scratch.convexPtr1 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices1
		+ sizeof(float4) * ss_scratch.nbPolygons1);

	const PxU8* polyData1_vertexData8 = (const PxU8*)(ss_scratch.convexPtr1 + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * ss_scratch.nbVertices1
		+ (sizeof(float4) + sizeof(PxU32)) * ss_scratch.nbPolygons1
		+ (sizeof(PxU16) + sizeof(PxU8)) * 2 * ss_scratch.nbEdges1
		+ sizeof(PxU8) * 3 * ss_scratch.nbVertices1);

	const PxU32 nbPolygons = ss_scratch.nbPolygons1;

	PxI32 bestPlaneBIdx = getPolygonIndexFromLocalWitness(
		nbPolygons,
		pPolygonPlanes1,
		ss_scratch.scale1, ss_scratch.rot1,
		ss_scratch.gjkOutput.closestPointB,
		normal,
		ss_scratch.inSphereRadius1,
		ss_scratch.toleranceLength);

	const PxU32 polyDesc1 = vRef8NbVertsMinIndex1[bestPlaneBIdx];

	const PxU32 nbVerts = getNbVerts(polyDesc1);

	const PxReal inflatedRadius = ss_scratch.contactDistance;

	//for (PxU32 rStart = 0, rEnd = nbVerts - 1; rStart < nbVerts; rEnd = rStart++)
	for(PxU32 i = 0; i < nbVerts; i+= WARP_SIZE)
	{
		PxU32 start = i + threadIndexInWarp;
		PxVec3 localPointA, localPointB;
		PxVec4 localNormalPen;
		bool hasContacts = false;
		if(start < nbVerts)
		{
			PxU32 end = (start + 1)%nbVerts;

			PxVec3 v0 = ss_scratch.vB[polyData1_vertexData8[getVRef8(polyDesc1) + start]];
			PxVec3 v1 = ss_scratch.vB[polyData1_vertexData8[getVRef8(polyDesc1) + end]];

			hasContacts = generateEE(ss_scratch.vA[0], ss_scratch.vA[1], normal, v0, v1, ss_scratch.aToB, inflatedRadius,
				localPointA, localPointB, localNormalPen);
		}

		PxU32 mask = __ballot_sync(FULL_MASK, hasContacts);

		if (hasContacts)
		{
			PxU32 offset = warpScanExclusive(mask, threadIdx.x);
			if (offset < 4)
			{
				ss_scratch.manifoldContactPointsA[numContacts + offset] = make_float4(localPointA.x, localPointA.y, localPointA.z, 0.f);
				ss_scratch.manifoldContactPointsB[numContacts + offset] = make_float4(localPointB.x, localPointB.y, localPointB.z, 0.f);
				ss_scratch.manifoldContactPointsLocalNormalPen[numContacts + offset] = make_float4(-localNormalPen.x, -localNormalPen.y, -localNormalPen.z, -localNormalPen.w);

				/*printf("idx %i localPointA(%f, %f, %f)\n", threadIdx.x, localPointA.x, localPointA.y, localPointA.z);
				printf("idx %i localPointB(%f, %f, %f)\n", threadIdx.x, localPointB.x, localPointB.y, localPointB.z);
				printf("idx %i localNormalPen(%f, %f, %f, %f)\n", threadIdx.x, localNormalPen.x, localNormalPen.y, localNormalPen.z, localNormalPen.w);*/
			}
		}
		numContacts = PxMin(numContacts + __popc(mask), 4u);
	}

	return numContacts;
}


__device__ static
bool collide(
			const float4* pVertices0, 
			const float4* pVertices1, 
			PxU32& persistentContactManifoldMoreDataW,
			PxU8& numManifoldContacts, /*out warmStartIndex in GJK*/
			//shared mem temps
			EpaAndClipScratch& ss_epa_clip_scratch, 
			CollideScratch& ss_scratch,	
			const bool flip)
{

	const PxReal convergenceRatio = 1-0.000225f;

	assert(ss_scratch.nbVertices0 <= CONVEX_MAX_VERTICES_POLYGONS);
	assert(ss_scratch.nbVertices1 <= CONVEX_MAX_VERTICES_POLYGONS);

	assert(numManifoldContacts <= 4);
	

	prepareVertices(ss_scratch.aToB, ss_scratch.scale0, ss_scratch.rot0, ss_scratch.nbVertices0, pVertices0, ss_scratch.vA);
	prepareVertices(ss_scratch.scale1, ss_scratch.rot1, ss_scratch.nbVertices1, pVertices1, ss_scratch.vB);
	
	__syncwarp(); // this is to avoid having volatile vertex array
	

	const PxU32 numManifoldWarmStartPoints = __shfl_sync(FULL_MASK, (int)persistentContactManifoldMoreDataW, 13);
	PxU32 warmStartAIndices = __shfl_sync(FULL_MASK, (int)persistentContactManifoldMoreDataW, 14);
	PxU32 warmStartBIndices = __shfl_sync(FULL_MASK, (int)persistentContactManifoldMoreDataW, 15);
	
	assert(numManifoldWarmStartPoints <= 4);

	//bool rightPair = ss_scratch.transformCache0 == 302 && ss_scratch.transformCache1 == 16;
	//if (threadIdx.x == 0 && rightPair)
	//{
	//	printf("warmStartPoint = %i\n", numManifoldWarmStartPoints);
	//}


	if(threadIdx.x < numManifoldWarmStartPoints)
	{
		PxU8 aIndex = (warmStartAIndices >> (threadIdx.x << 3)) & 0xFF;
		PxU8 bIndex = (warmStartBIndices >> (threadIdx.x << 3)) & 0xFF;

		assert(aIndex < ss_scratch.nbVertices0);
		assert(bIndex < ss_scratch.nbVertices1);

		/*if (rightPair)
		{
			printf("aIndices[%i]=%i bIndices[%i] = %i\n", threadIdx.x, aIndex, threadIdx.x, bIndex);
		}*/
		ss_scratch.cachedData.vertices[threadIdx.x] = CachedVertex(aIndex, bIndex, ss_scratch.vA[aIndex] - ss_scratch.vB[bIndex]);
	}

	/*if (threadIdx.x < numManifoldWarmStartPoints)
	{
		PxU8 aIndices[4] = { 4, 1, 2, 5 };
		PxU8 bIndices[4] = { 3, 8, 12, 4 };

		PxU8 aIndex = aIndices[threadIdx.x];
		PxU8 bIndex = bIndices[threadIdx.x];
		ss_scratch.cachedData.vertices[threadIdx.x] = CachedVertex(aIndex, bIndex, ss_scratch.vA[aIndex] - ss_scratch.vB[bIndex]);
	}*/

	

	ss_scratch.cachedData.size = numManifoldWarmStartPoints;


	__syncwarp(); // this is to avoid having volatile vertex array

	PxVec3 initialDir = PxVec3(1, 0, 0);

	PxVec3 aToB_p = ss_scratch.aToB.p;

	if (aToB_p.magnitudeSquared() > 0)
		initialDir = aToB_p; // GJK's initial start dir is usually from the warm cache => lazy normalize inside GJK

	GjkResult::Enum gjkResult = runGJK(initialDir, persistentContactManifoldMoreDataW, ss_scratch, convergenceRatio);

	if(gjkResult==GjkResult::eDISTANT)
	{
		return false;
	}

	bool anomaly = gjkResult == GjkResult::eCLOSE && ss_scratch.gjkOutput.closestPointDir.dot(ss_scratch.gjkOutput.direction) < 0.999f;
	bool separated = gjkResult == GjkResult::eCLOSE;
	
	if(!separated || anomaly)
	{
		GjkResult::Enum epaResult = squawk::epa(*(squawk::EpaScratch*) ss_epa_clip_scratch.mem,
												ss_scratch.vA, ss_scratch.nbVertices0,
												ss_scratch.vB, ss_scratch.nbVertices1,
												&(ss_scratch.cachedData),
												convergenceRatio, 
												0.5f * (ss_scratch.inSphereRadius0 + ss_scratch.inSphereRadius1),
												ss_scratch.gjkOutput);

		separated = epaResult == GjkResult::eCLOSE;

		__syncwarp();

		if (ss_scratch.gjkOutput.degenerate)
		{
			//we need to re-run gjk epa with other configurations
			ss_scratch.cachedData.size = 0;

			GjkResult::Enum gjkResult = runGJK(initialDir, persistentContactManifoldMoreDataW, ss_scratch, convergenceRatio);

			GjkResult::Enum epaResult = squawk::epa(*(squawk::EpaScratch*) ss_epa_clip_scratch.mem,
				ss_scratch.vA, ss_scratch.nbVertices0,
				ss_scratch.vB, ss_scratch.nbVertices1,
				&(ss_scratch.cachedData),
				convergenceRatio,
				0.5f * (ss_scratch.inSphereRadius0 + ss_scratch.inSphereRadius1),
				ss_scratch.gjkOutput);

			separated = epaResult == GjkResult::eCLOSE;

			__syncwarp();
			
		}
	}

	if (ss_scratch.type0 == PxGeometryType::eSPHERE)
	{
		// ML: we need to add the gjk / epa contacts to the manifold first.If the number of contacts after GJK / EPA in the manifold is still less than the initial number of
		//contacts, we need to trigger a full contact gen. Otherwise, we will lose contacts and will never regenerate contacts again.
		//PxVec3 witnessA = ss_scratch.gjkOutput.closestPointA;
		//PxVec3 witnessB = ss_scratch.gjkOutput.closestPointB;
		//PxVec3 normal = ss_scratch.gjkOutput.direction;
		//PxReal sep = (witnessB - witnessA).dot(normal);

		//if (threadIdx.x == 12)
		//{
		//	ss_scratch.manifoldContactPointsA[0] = make_float4(0.f); //sphere center
		//	ss_scratch.manifoldContactPointsB[0] = make_float4(witnessB.x, witnessB.y, witnessB.z, 0.f);
		//	ss_scratch.manifoldContactPointsLocalNormalPen[0] = make_float4(-normal.x, -normal.y, -normal.z, sep);
		//	/*printf("witnessB(%f, %f, %f) sep %f\n", witnessB.x, witnessB.y, witnessB.z, sep);
		//	printf("normal(%f, %f, %f) \n", normal.x, normal.y, normal.z);*/
		//}

		PxReal witnessA = 0.f;
		PxReal witnessB = 0.f;
		PxReal normal = 0.f;
		if (threadIdx.x < 3)
		{
			const float* closestPointA = &ss_scratch.gjkOutput.closestPointA.x;
			const float* closestPointB = &ss_scratch.gjkOutput.closestPointB.x;
			const float* direction = &ss_scratch.gjkOutput.direction.x;
			witnessA = closestPointA[threadIdx.x];
			witnessB = closestPointB[threadIdx.x];
			normal = direction[threadIdx.x];
		}
		PxReal sep = (witnessB - witnessA) * normal;
		sep = __shfl_sync(FULL_MASK, sep, 0) + __shfl_sync(FULL_MASK, sep, 1) + __shfl_sync(FULL_MASK, sep, 2);

		if (threadIdx.x < 4)
		{
			float* pointA = &ss_scratch.manifoldContactPointsA[0].x;
			float* pointB = &ss_scratch.manifoldContactPointsB[0].x;
			float* normalPen = &ss_scratch.manifoldContactPointsLocalNormalPen[0].x;
			PxReal out = threadIdx.x < 3 ? -normal : sep;

			pointA[threadIdx.x] = 0.f;	//sphere center
			pointB[threadIdx.x] = witnessB;
			normalPen[threadIdx.x] = out;
		}


		numManifoldContacts = 1;
	}
	else if (ss_scratch.type0 == PxGeometryType::eCAPSULE)
	{
		PxU32 numContacts = generatedFaceContacts(ss_scratch, threadIdx.x);

		if (numContacts < 2)
		{
			numContacts = generatedContactsEEContacts(ss_scratch, threadIdx.x, numContacts);
		}

		numManifoldContacts = numContacts;
		assert(numManifoldContacts <= 4);
	}
	else
	{
		generateConvexContacts(ss_epa_clip_scratch, ss_scratch, persistentContactManifoldMoreDataW,
			numManifoldContacts);
	}

	return true;	
}


extern "C" __global__ void prepareLostFoundPairs_Stage1(const PxU32* PX_RESTRICT inputFlagsArray,
															PxU32* PX_RESTRICT	tempRunsumArray,
															 PxU32* PX_RESTRICT	blockAccumulationArray,
															 PxU32 numPairs)
	
{
  	ReadArrayFunctor<PxU32> readF(inputFlagsArray);
	WriteArrayFunctor<PxU32> writeF(tempRunsumArray);
	scanKernel1of2<PxgNarrowPhaseBlockDims::COMPACT_LOST_FOUND_PAIRS, PxgNarrowPhaseGridDims::COMPACT_LOST_FOUND_PAIRS,
													AddOpPxU32, PxU32, ReadArrayFunctor<PxU32>,	WriteArrayFunctor<PxU32> >(
														readF,
														writeF,
														2 * numPairs,
														blockAccumulationArray);
}

class CompactContactManagersFunctor
{
public:
	__host__ __device__
	CompactContactManagersFunctor(
									const PxU32* PX_RESTRICT inputFlags,
									const PxsContactManagerOutput* PX_RESTRICT allOutputManagers,
									PxsContactManagerOutputCounts* PX_RESTRICT compactedOutputManagers,
									PxsContactManager** PX_RESTRICT contactManagerPtrs,
									PxU32* PX_RESTRICT numTouchLostElements,
									PxU32 numPairs,
									PxsContactManager** contactManagerArray):
													mInputFlags(inputFlags),
													mAllOutputManagers(allOutputManagers),
													mCompactedOutputManagers(compactedOutputManagers),
													mContactManagerArray(contactManagerArray),
													mContactManagerPtrs(contactManagerPtrs),
													mNumTouchLostElements(numTouchLostElements),
													mNumPairs(numPairs)
													{}

	__host__ __device__
	void operator()(PxU32 idx, PxU32 outOffset) 
	{
		if (mInputFlags[idx])
		{
			const PxU32 index = idx % mNumPairs;
			mCompactedOutputManagers[outOffset].nbPatches = mAllOutputManagers[index].nbPatches;
			mCompactedOutputManagers[outOffset].prevPatches = mAllOutputManagers[index].prevPatches;
			mCompactedOutputManagers[outOffset].statusFlag = mAllOutputManagers[index].statusFlag;
			//mCompactedOutputManagers[outOffset].nbContacts = mAllOutputManagers[index].nbContacts;
			mContactManagerPtrs[outOffset] = mContactManagerArray[index];
		}

		if (idx == mNumPairs)
			*mNumTouchLostElements = outOffset;
	}

protected:
	const PxU32*					mInputFlags;
	const PxsContactManagerOutput*	mAllOutputManagers;
	PxsContactManagerOutputCounts*	mCompactedOutputManagers;
	PxsContactManager**				mContactManagerArray;
	PxsContactManager**				mContactManagerPtrs;
	PxU32*							mNumTouchLostElements;
	PxU32							mNumPairs;
};

extern "C" __global__ void prepareLostFoundPairs_Stage2(
															const PxU32* PX_RESTRICT inputFlagsArray,
															const PxU32* PX_RESTRICT tempRunsumArray,
															const PxsContactManagerOutput* PX_RESTRICT allOutputManagers,
															PxsContactManagerOutputCounts* PX_RESTRICT compactedOutputManagers,
															PxsContactManager** PX_RESTRICT outputManagersPtrs,
															uint2* PX_RESTRICT lostAndTotalReportedPairsCount,
															PxU32* PX_RESTRICT blockAccumulationArray,
															PxU32 numPairs,
															PxsContactManager** PX_RESTRICT contactManagerArray)
{
	ReadArrayFunctor<PxU32> readF(tempRunsumArray);
	CompactContactManagersFunctor contactF(inputFlagsArray,
											allOutputManagers,
											compactedOutputManagers,
											outputManagersPtrs,
											&(lostAndTotalReportedPairsCount->x),
											numPairs, contactManagerArray);
	WriteValueFunctor<PxU32> writeF(&(lostAndTotalReportedPairsCount->y));
	scanKernel2of2<PxgNarrowPhaseGridDims::COMPACT_LOST_FOUND_PAIRS, AddOpPxU32, PxU32,
									ReadArrayFunctor<PxU32>, CompactContactManagersFunctor, WriteValueFunctor<PxU32> >(
											readF,
											contactF,
											writeF,
											2 * numPairs,
											blockAccumulationArray);
}



struct ContactParams
{
	PxU16 startIdx;
	PxU8 nbContacts;
	PxU8 materialFlags;
	
};

struct MaterialParams
{
	PxU16 internalFlags;
	PxU16 materialIndex0;
};

struct MaterialParams2
{
	PxU16 materialIndex1;
	PxU16 pad;
};

union ContactUnion
{
	ContactParams params;
	PxU32 val;
};

union MaterialUnion
{
	MaterialParams params;
	PxU32 val;
};


struct FinishContactsWarpScratch
{
	PxU32 normx[WARP_SIZE];
	PxU32 normy[WARP_SIZE];
	PxU32 normz[WARP_SIZE];
	PxU32 restitution[WARP_SIZE];
	PxU32 dynamicFriction[WARP_SIZE];
	PxU32 staticFriction[WARP_SIZE];
	PxU32 damping[WARP_SIZE];
	union
	{
		ContactParams contactParams[WARP_SIZE];
		PxU32 contactUnion[WARP_SIZE];
	};
	union
	{
		MaterialParams materialParams[WARP_SIZE];
		PxU32 materialUnion[WARP_SIZE];
	};
	union
	{
		MaterialParams2 materialParams2[WARP_SIZE];
		PxU32 materialUnion2[WARP_SIZE];
	};
};


static __device__ inline void writeContactsToStream4threads(
															PxU32 numTests, 
															bool insertAveragePoint,
															PxU8* PX_RESTRICT contactStream,
															PxU8* PX_RESTRICT startContactPoints,
															PxU8* PX_RESTRICT startContactForces,
															PxU32 contactBytesLimit,
															PxU32 forceBytesLimit,
															PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
															const PxgContactManagerInput* PX_RESTRICT cmInputs,
															PxsContactManagerOutput* PX_RESTRICT cmOutputs,
															PxgPersistentContactManifold* PX_RESTRICT cmManifolds,
															PxgShape* PX_RESTRICT shapes,
															const PxsCachedTransform* PX_RESTRICT transformCache,
															const PxReal* PX_RESTRICT contactDistance,
															FinishContactsWarpScratch& s_scratch)
{
	const PxU32 warpBeg = blockIdx.x * blockDim.y * blockDim.x + threadIdx.y * blockDim.x;
	const PxU32 numElemsPerIteration = WARP_SIZE / 4;
	const PxU32 groupId = threadIdx.x / 4;
	const PxU32 tidInGroup = threadIdx.x % 4;
		
	#pragma unroll 4 
	for (PxU32 baseOffs = 0; baseOffs < WARP_SIZE; baseOffs += numElemsPerIteration)
	{
		//each 4 thread in a warp deal with a pair. Each thread in a group write to one contact to the output cm. 
		const PxU32 elementGlobalIndex = warpBeg + baseOffs + groupId;
		PxTransform t1;
		PxReal contactDist;
		PxGeometryType::Enum type0;
		PxGeometryType::Enum type1;
		PxReal radius = 0.f;

		bool flip = false;

		if (elementGlobalIndex < numTests)
		{
			PxgContactManagerInput input = cmInputs[elementGlobalIndex];

			PxgShape shape0 = shapes[input.shapeRef0];
			PxgShape shape1 = shapes[input.shapeRef1];

			type0 = PxGeometryType::Enum(shape0.type);
			type1 = PxGeometryType::Enum(shape1.type);

			PxU32 materialIndex0 = shape0.materialIndex;
			PxU32 materialIndex1 = shape1.materialIndex;

			PxU32 transformCacheRef0 = input.transformCacheRef0;
			PxU32 transformCacheRef1 = input.transformCacheRef1;

			PxVec3 scale0 = shape0.scale.scale;
			PxVec3 scale1 = shape1.scale.scale;

			flip = (type1 < type0);

			if (flip)
			{
				PxSwap(type0, type1);
				PxSwap(scale0, scale1);
				PxSwap(transformCacheRef0, transformCacheRef1);
			}

			if (type0 == PxGeometryType::eSPHERE || type0 == PxGeometryType::eCAPSULE)
				radius = scale0.y; 

			t1 = transformCache[transformCacheRef1].transform;
			//contactDistance = (shapes + input.shapeRef0)->contactOffset + (shapes + input.shapeRef1)->contactOffset;
			contactDist = contactDistance[transformCacheRef0] + contactDistance[transformCacheRef1];

			if (tidInGroup == 0)
			{
				s_scratch.materialParams[baseOffs + groupId].materialIndex0 = materialIndex0;
				s_scratch.materialParams2[baseOffs + groupId].materialIndex1 = materialIndex1;
			}
		}
		
		PxgPersistentContactManifold* manifoldPtr = cmManifolds + elementGlobalIndex;
	
		float4* manifoldContactPointsLocalNormalPen = ((float4*) manifoldPtr);
		float4* manifoldContactPointsB = ((float4*) manifoldPtr) + 2 * PXG_MAX_PCM_CONTACTS;

		PxU32 numManifoldContacts = 0;

		if (elementGlobalIndex < numTests)
		{
			numManifoldContacts = manifoldPtr->mNbContacts;
			manifoldPtr->mNbContacts = numManifoldContacts & ~0x80000000;

			if (numManifoldContacts & 0x80000000)
				numManifoldContacts = 0;
		}

		assert(numManifoldContacts <= 4);

		float4 normPen = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

		if (tidInGroup < numManifoldContacts)
		{
			normPen = manifoldContactPointsLocalNormalPen[tidInGroup];
		}

		PxVec3 avgNorm;
		avgNorm.x = normPen.x + __shfl_xor_sync(FULL_MASK, normPen.x, 2);
		avgNorm.x += __shfl_xor_sync(FULL_MASK, avgNorm.x, 1);
		avgNorm.y = normPen.y + __shfl_xor_sync(FULL_MASK, normPen.y, 2);
		avgNorm.y += __shfl_xor_sync(FULL_MASK, avgNorm.y, 1);
		avgNorm.z = normPen.z + __shfl_xor_sync(FULL_MASK, normPen.z, 2);
		avgNorm.z += __shfl_xor_sync(FULL_MASK, avgNorm.z, 1);

		

		PxReal magSq = avgNorm.magnitudeSquared();

		if(magSq >= 1e-5f)
		{
			avgNorm /= sqrtf(magSq);
		}
		else
		{
			avgNorm = PxVec3(normPen.x, normPen.y, normPen.z);
		}

		avgNorm = t1.rotate(avgNorm);

		if (flip)
			avgNorm = -avgNorm;
					
		if (tidInGroup == 0)
		{
			s_scratch.normx[baseOffs + groupId] = __float_as_int(avgNorm.x);
			s_scratch.normy[baseOffs + groupId] = __float_as_int(avgNorm.y);
			s_scratch.normz[baseOffs + groupId] = __float_as_int(avgNorm.z);
		}

		float4 pointB = make_float4(FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);
		
		if (tidInGroup < numManifoldContacts)
		{
			//if the type isn't sphere, radius is 0
			const PxReal sep = normPen.w - radius;

			pointB = manifoldContactPointsB[tidInGroup];

			PxVec3 p = t1.transform(PxVec3(pointB.x, pointB.y, pointB.z));
			pointB = make_float4(p.x, p.y, p.z, sep);


		}

		PxU32 keepContactW = __ballot_sync(FULL_MASK, pointB.w <= contactDist);
		numManifoldContacts = __popc(keepContactW & (0xF << (threadIdx.x & ~0x3)));
	
		//average contact has to go first to be solved by the solver. Otherwise, it doesn't benefit us. Therefore, tidInGroup has to be 0
		PxU32 extraContactsW =  __ballot_sync(FULL_MASK, tidInGroup == 0 && insertAveragePoint && numManifoldContacts > 1);

		PxU32 contactByteOffset = 0xFFFFFFFF;
		PxU32 forceAndIndiceByteOffset = 0xFFFFFFFF;

	
		if (threadIdx.x == 0 )
		{		
			PxU32 totalContacts = __popc(keepContactW) + __popc(extraContactsW);

			if (totalContacts)
			{

				contactByteOffset = atomicAdd(&(patchAndContactCounters->contactsBytes), sizeof(PxContact) * totalContacts);
				//force is PxReal and Indices is PxU32, however, both are 4 bytes so the size should be the same
				forceAndIndiceByteOffset = atomicAdd(&(patchAndContactCounters->forceAndIndiceBytes), sizeof(PxU32) * totalContacts);

				if ((contactByteOffset + sizeof(PxContact) * totalContacts) >= contactBytesLimit)
				{
					patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW);
					contactByteOffset = 0xFFFFFFFF; //overflow
				}
				else if ((forceAndIndiceByteOffset + sizeof(PxU32)* totalContacts) >= forceBytesLimit)
				{
					patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::FORCE_BUFFER_OVERFLOW);
					forceAndIndiceByteOffset = 0xFFFFFFFF; //overflow
				}
			}
			
		}

		//this is the start contactByte address
		contactByteOffset = __shfl_sync(FULL_MASK, contactByteOffset, 0);
		forceAndIndiceByteOffset = __shfl_sync(FULL_MASK, forceAndIndiceByteOffset, 0);
		
		if (contactByteOffset != 0xFFFFFFFF)
		{
			//this is the address each thread should write to
			const PxU32 mask = (1 << threadIdx.x) - 1;
			const PxU32 byteOffset = (__popc(keepContactW & mask) + __popc(extraContactsW & mask));
			contactByteOffset += sizeof(PxContact)* byteOffset;
			forceAndIndiceByteOffset += sizeof(PxReal)* byteOffset;

			size_t offset = 0;
			if (insertAveragePoint && numManifoldContacts > 1)
			{
				assert(numManifoldContacts <= 4);
				float4 avgPt = make_float4(0.0f, 0.0f, 0.0f, 0.0f);

				if (tidInGroup < numManifoldContacts)
					avgPt = pointB;

				avgPt.x += __shfl_xor_sync(FULL_MASK, avgPt.x, 2);
				avgPt.x += __shfl_xor_sync(FULL_MASK, avgPt.x, 1);
				avgPt.y += __shfl_xor_sync(FULL_MASK, avgPt.y, 2);
				avgPt.y += __shfl_xor_sync(FULL_MASK, avgPt.y, 1);
				avgPt.z += __shfl_xor_sync(FULL_MASK, avgPt.z, 2);
				avgPt.z += __shfl_xor_sync(FULL_MASK, avgPt.z, 1);
				avgPt.w += __shfl_xor_sync(FULL_MASK, avgPt.w, 2);
				avgPt.w += __shfl_xor_sync(FULL_MASK, avgPt.w, 1);

				if (tidInGroup == 0)
				{
					avgPt /= PxReal(numManifoldContacts);
					*((float4 *)(contactStream + contactByteOffset)) = avgPt; //avgPoint needs to be first in the buffer!
					offset += sizeof(PxContact);
				}
			}

			if (pointB.w <= contactDist)
			{
				*((float4 *)(contactStream + contactByteOffset+offset)) = pointB;
			}
	
		}
		else
		{
			numManifoldContacts = 0;
		}
		
		if (tidInGroup == 0)
		{

			if (!numManifoldContacts)
			{
				contactByteOffset = 0xFFFFFFFF;
				forceAndIndiceByteOffset = 0xFFFFFFFF;
			}
			if (elementGlobalIndex < numTests)
			{
				PxsContactManagerOutput* output = cmOutputs + elementGlobalIndex;

				output->contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
				output->contactPoints = startContactPoints + contactByteOffset;

			}		
			s_scratch.contactParams[baseOffs + groupId].nbContacts = numManifoldContacts + (numManifoldContacts > 1 && insertAveragePoint);
		}
	}
}




// AD: I don't know why it has to be this complex. Looks like this was optimized but then stuff was added and the optimizations weren't considered anymore.
// It seems overkill given the layout of PxContactPatch right now.

// Every thread is looking up whether there's a corresponding field that was filled with a value for their part of PxContactPatch using
// the patchOffsets array. For everything that is not filled, there is a 0 there which means we are taking the "fallback" into patchConstants.
// This only happens for the invMassScale and padding members though. Everything else is filled using the shared memory scratchpad.

// To make this consistent and somewhat followable I added the padding to the arrays now, with the added benefit of avoiding the branch
// inside the function. 

// if this assert hits, all of the code below needs to be updated.
// There's no perfect way to make sure nobody messes with PxContactPatch without changing everything here, but these two seem like things that should catch common issues.
PX_COMPILE_TIME_ASSERT(sizeof(PxContactPatch) == 64);
PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(PxContactPatch, pad) == 54); 

__constant__ __device__ PxReal patchConstants[] =	{1.0f,//mInvMassScale0
													 1.0f,//mInvMassScale1
													 1.0f,//mInvInertiaScale0
													 1.0f,//mInvInertiaScale1

													 0.0f,//normal x
													 0.0f,//normal y
													 0.0f,//normal z
													 0.0f,//restitution

													 0.0f,//dynamicfriction
													 0.0f,//staticfriction
													 0.0f,//damping
													 0.0f,//startContactIndex + nbcontacts + materialFlags

													 0.0f,//internalFlags + materialIndex0
													 0.0f,//materialIndex1 + padding
													 0.0f,//padding
													 0.0f,//padding
													};

PX_COMPILE_TIME_ASSERT(sizeof(patchConstants) == 16 * 4);

__constant__ __device__ PxU32 ((FinishContactsWarpScratch::*patchOffsets[16])[32]) = {
													 0,
													 0,
													 0,
													 0,

													 &FinishContactsWarpScratch::normx,
													 &FinishContactsWarpScratch::normy,
													 &FinishContactsWarpScratch::normz,
													 &FinishContactsWarpScratch::restitution,

													 &FinishContactsWarpScratch::dynamicFriction,
													 &FinishContactsWarpScratch::staticFriction,
													 &FinishContactsWarpScratch::damping,
													 &FinishContactsWarpScratch::contactUnion,

													 &FinishContactsWarpScratch::materialUnion,
													 &FinishContactsWarpScratch::materialUnion2,
													 0,
													 0,
													};



static __device__ inline void writePatchesToStream4threads(
															PxU32 numTests, 
															PxU32 patchMask,
															PxU32 patchByteOffsWarp,
															PxU8* PX_RESTRICT patchStream,
															FinishContactsWarpScratch& s_scratch)
{
	const PxU32 numElemsPerIteration = WARP_SIZE / 4;
	const PxU32 groupId = threadIdx.x / 4;
	const PxU32 tidInGroup = threadIdx.x % 4;

	#pragma unroll 4 
	for (PxU32 baseOffs = 0; baseOffs < WARP_SIZE; baseOffs += numElemsPerIteration)
	{
		PxU32(FinishContactsWarpScratch::*ptr0)[32] = patchOffsets[tidInGroup * 4 + 0];
		PxU32 val0;
		if (ptr0)
		{
			val0 = (s_scratch.*ptr0)[baseOffs + groupId];
		}
		else
		{
			val0 = __float_as_int(patchConstants[tidInGroup * 4 + 0]);
		}

		PxU32(FinishContactsWarpScratch::*ptr1)[32] = patchOffsets[tidInGroup * 4 + 1];
		PxU32 val1;
		if (ptr1)
		{
			val1 = (s_scratch.*ptr1)[baseOffs + groupId];
		}
		else
		{
			val1 = __float_as_int(patchConstants[tidInGroup * 4 + 1]);
		}

		PxU32(FinishContactsWarpScratch::*ptr2)[32] = patchOffsets[tidInGroup * 4 + 2];
		PxU32 val2;
		if (ptr2)
		{
			val2 = (s_scratch.*ptr2)[baseOffs + groupId];
		}
		else
		{
			val2 = __float_as_int(patchConstants[tidInGroup * 4 + 2]);
		}

		PxU32(FinishContactsWarpScratch::*ptr3)[32] = patchOffsets[tidInGroup * 4 + 3];
		PxU32 val3;
		if (ptr3)
		{
			val3 = (s_scratch.*ptr3)[baseOffs + groupId];
		}
		else
		{
			val3 = __float_as_int(patchConstants[tidInGroup * 4 + 3]);
		}
		
		PxContactPatch* patch = ((PxContactPatch*)(patchStream + patchByteOffsWarp)) + __popc(patchMask & ((1 << baseOffs + groupId) - 1));
		if (patchMask & (1ul << (baseOffs + groupId)))
		{
			uint4* out = ((uint4*)patch) + tidInGroup;
			*out = make_uint4(val0, val1, val2, val3);
		}
	}
}

extern "C"
__global__ void finishContactsKernel(PxU32 numTests, 
								const PxgContactManagerInput* PX_RESTRICT cmInputs,
								PxsContactManagerOutput* PX_RESTRICT cmOutputs,
								PxgPersistentContactManifold* PX_RESTRICT cmManifolds,
								PxgShape* PX_RESTRICT shapes,
								const PxsCachedTransform* PX_RESTRICT transformCache,
								const PxReal* PX_RESTRICT contactDistance,
								const PxsMaterialData* PX_RESTRICT materials,
								PxU8* PX_RESTRICT contactStream,
								PxU8* PX_RESTRICT patchStream,
								bool insertAveragePoint,
								PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
								PxU32* PX_RESTRICT touchChangeFlags,
								PxU32* PX_RESTRICT patchChangeFlags,
								PxU8* PX_RESTRICT startContactPatches,
								PxU8* PX_RESTRICT startContactPoints,
								PxU8* PX_RESTRICT startContactForces,
								PxU32 patchBytesLimit,
								PxU32 contactBytesLimit,
								PxU32 forceBytesLimit)
{

	__shared__ FinishContactsWarpScratch scratch[PxgNarrowPhaseBlockDims::FINISH_CONTACTS / WARP_SIZE];
	FinishContactsWarpScratch& sscratch = scratch[threadIdx.y];

	writeContactsToStream4threads(numTests, 
								insertAveragePoint,
								contactStream,
								startContactPoints,
								startContactForces,
								contactBytesLimit,
								forceBytesLimit,
								patchAndContactCounters,
								cmInputs,
								cmOutputs,
								cmManifolds,
								shapes,
								transformCache,
								contactDistance,
								sscratch);
	

	sscratch.contactParams[threadIdx.x].startIdx = 0;
	sscratch.materialParams[threadIdx.x].internalFlags = 0;

	__syncthreads();

	const PxU32 elementGlobalIndex = threadIdx.x + blockIdx.x * blockDim.y * blockDim.x + threadIdx.y * blockDim.x;
	
	PxU32 nbContacts = elementGlobalIndex < numTests ? sscratch.contactParams[threadIdx.x].nbContacts : 0;

	PxU32 allflags = 0;
	
	if (elementGlobalIndex < numTests)
		allflags = reinterpret_cast<PxU32*>(&((cmOutputs + elementGlobalIndex)->allflagsStart))[0];

	PxU8 oldStatusFlags = u16Low(u32High(allflags));
	PxU8 statusFlags = oldStatusFlags;

	statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);
	
	if (nbContacts != 0)
		statusFlags |= PxsContactManagerStatusFlag::eHAS_TOUCH;
	else
		statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

	PxU8 prevPatches = u16High(u32Low(allflags)); //Get out the current number of patches to store as the previous frame's number of patches
	bool previouslyHadTouch = oldStatusFlags & PxsContactManagerStatusFlag::eHAS_TOUCH;
	bool prevTouchKnown = oldStatusFlags & PxsContactManagerStatusFlag::eTOUCH_KNOWN;
	
	if (elementGlobalIndex < numTests)
	{
		/*touchLostFlags[elementGlobalIndex] = (oldStatusFlags & PxsContactManagerStatusFlag::eHAS_TOUCH) && !numManifoldContacts;
		touchFoundFlags[elementGlobalIndex] = !(oldStatusFlags & PxsContactManagerStatusFlag::eHAS_TOUCH) && numManifoldContacts;*/

		//Because convex-convex just has one patch so touchChange and patchChange should be the same

		PxU8 numPatches = nbContacts ? 1 : 0;

		bool currentlyHasTouch = nbContacts != 0;

		const bool change = (previouslyHadTouch ^ currentlyHasTouch) || (!prevTouchKnown);
		touchChangeFlags[elementGlobalIndex] = change;
		patchChangeFlags[elementGlobalIndex] = (prevPatches != numPatches);

		reinterpret_cast<PxU32*>(&((cmOutputs + elementGlobalIndex)->allflagsStart))[0] = merge(merge(prevPatches, statusFlags),
														merge(numPatches, PxU8(0)));
		cmOutputs[elementGlobalIndex].nbContacts = nbContacts;
	}
	
	PxU32 patchMask = __ballot_sync(FULL_MASK, nbContacts > 0);
	PxU32 patchIndex = 0xFFFFFFFF;

	if (patchMask && threadIdx.x == WARP_SIZE - 1)
	{
		PxU32 totalSz =  sizeof(PxContactPatch) * __popc(patchMask);
		patchIndex = atomicAdd(&(patchAndContactCounters->patchesBytes), totalSz);
		
		if ((patchIndex + totalSz) > patchBytesLimit)
		{
			patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW);
			patchIndex = 0xFFFFFFFF; //overflow
		}
	}

	patchIndex = __shfl_sync(FULL_MASK, patchIndex, WARP_SIZE - 1);
	
	if (patchIndex == 0xFFFFFFFF && patchMask) //overflow
	{
		patchMask = 0;
		sscratch.contactParams[threadIdx.x].nbContacts = 0;

		if (elementGlobalIndex < numTests)
		{
			statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);
			statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

			reinterpret_cast<PxU32*>(&((cmOutputs + elementGlobalIndex)->allflagsStart))[0] = merge(merge(prevPatches, statusFlags), 0);  
			cmOutputs[elementGlobalIndex].nbContacts = 0;

			touchChangeFlags[elementGlobalIndex] = previouslyHadTouch;
			patchChangeFlags[elementGlobalIndex] = prevPatches != 0;
		}
	}

	if (elementGlobalIndex < numTests && patchIndex != 0xFFFFFFFF)
	{
		(cmOutputs + elementGlobalIndex)->contactPatches = startContactPatches + patchIndex +
								sizeof(PxContactPatch) * __popc(patchMask & ((1 << threadIdx.x) - 1));
	}

	PxReal restitution, dynamicFriction, staticFriction, damping;
	PxU32 materialFlags;

	if (elementGlobalIndex < numTests)
		combineMaterials(materials, sscratch.materialParams[threadIdx.x].materialIndex0, 
								sscratch.materialParams2[threadIdx.x].materialIndex1,
								materialFlags,
								staticFriction,
								dynamicFriction,
								restitution,
								damping
								);

	sscratch.restitution[threadIdx.x] = __float_as_int(restitution);
	sscratch.dynamicFriction[threadIdx.x] = __float_as_int(dynamicFriction);
	sscratch.staticFriction[threadIdx.x] = __float_as_int(staticFriction);
	sscratch.damping[threadIdx.x] = __float_as_int(damping);
	sscratch.contactParams[threadIdx.x].materialFlags = static_cast<PxU8>(materialFlags);

	__syncthreads();

	writePatchesToStream4threads(numTests, 
								patchMask,
								patchIndex,
								patchStream,
								sscratch);
}


extern "C" __global__ void convexConvexNphase_stage1Kernel(
														PxU32 numTests,
														const PxReal toleranceLength,
														const PxgContactManagerInput* PX_RESTRICT cmInputs,
														const PxsCachedTransform* PX_RESTRICT transformCache,
														PxgShape* PX_RESTRICT shapes,
														PxgPersistentContactManifold* PX_RESTRICT cmManifolds,
														PxU32* PX_RESTRICT	tempKeepPairArray)
{

	PxU32 globalElementIndex = (blockIdx.x * blockDim.x * blockDim.y + threadIdx.y * blockDim.x + threadIdx.x) >> 2;
		
	if (globalElementIndex >= numTests)
		return;

	PxU32 tidInGroup = threadIdx.x % 4;

	PxgContactManagerInput input = cmInputs[globalElementIndex];

	PxgShape shape0 = shapes[input.shapeRef0];
	PxgShape shape1 = shapes[input.shapeRef1];

	PxU8* convexPtrA = reinterpret_cast<PxU8*>(shape0.hullOrMeshPtr);
	PxU8* convexPtrB = reinterpret_cast<PxU8*>(shape1.hullOrMeshPtr);

	PxVec3 scale0 = shape0.scale.scale;
	PxVec3 scale1 = shape1.scale.scale;

	PxGeometryType::Enum type0 = PxGeometryType::Enum(shape0.type);
	PxGeometryType::Enum type1 = PxGeometryType::Enum(shape1.type);
	PxU32 transformCacheRef0 = input.transformCacheRef0;
	PxU32 transformCacheRef1 = input.transformCacheRef1;

	const bool flip = (type1<type0);

	if (flip)
	{
		PxSwap(type0, type1);
		PxSwap(scale0, scale1);
		PxSwap(convexPtrA, convexPtrB);
		PxSwap(transformCacheRef0, transformCacheRef1);
	}

	PxReal convexMargin0;
	PxVec3 extents0;
	if (type0 == PxGeometryType::eSPHERE || type0 == PxGeometryType::eCAPSULE)
	{
		convexMargin0 = scale0.y; //sphere radius
		extents0 = PxVec3(0.f);
	}
	else
	{
		const float4 extents0_f4 = *reinterpret_cast<const float4*>((convexPtrA + sizeof(float4) + sizeof(uint4)));
		extents0 = PxVec3(extents0_f4.x * scale0.x, extents0_f4.y * scale0.y, extents0_f4.z * scale0.z);
		convexMargin0 = calculatePCMConvexMargin(extents0, toleranceLength);
	}

	const float4 extents1_f4 = *reinterpret_cast<const float4*>((convexPtrB + sizeof(float4) + sizeof(uint4)));
	const PxVec3 extents1 = PxVec3(extents1_f4.x * scale1.x, extents1_f4.y * scale1.y, extents1_f4.z * scale1.z);
	const PxReal convexMargin1 = calculatePCMConvexMargin(extents1, toleranceLength);

	const PxReal minMargin = fminf(convexMargin0, convexMargin1);
	
	PxTransform t0 = transformCache[transformCacheRef0].transform;
	assert(t0.isSane());
	PxTransform t1 = transformCache[transformCacheRef1].transform;
	assert(t1.isSane());
	//PxTransform bToA = t0.transformInv(t1);
	PxTransform aToB = t1.transformInv(t0);
	
	PxgPersistentContactManifold* manifoldPtr = cmManifolds + globalElementIndex;
	
	//ML: every four threads are processing a pair. In here, we are using four threads to read the data started from
	//mRelativeTransform in the PxgPersistentContactManifold. Each thread read 16 bytes. 
	//tidInGroup = 0 read mRelativePos.
	//tidInGroup = 1 read mQuatA
	//tidInGroup = 2 read mQuatB
	//tidInGroup = 3 read mNbContacts, mNbWarmStartPoints, mWarmStartA and mWarmStartB
	uint4* persistentContactManifoldMoreData = (uint4*) (((float4*) manifoldPtr) + 3 * PXG_MAX_PCM_CONTACTS);
	uint4 persistentContactManifoldMoreDataW;

	if (tidInGroup < ((sizeof(float4) + sizeof(PxAlignedQuat) * 2 + sizeof(PxU32) * 4) / sizeof(uint4)))
	{
		persistentContactManifoldMoreDataW = persistentContactManifoldMoreData[tidInGroup];
	}

	PxU32 initialNbContacts = __shfl_sync(FULL_MASK, persistentContactManifoldMoreDataW.x, (threadIdx.x & ~0x3) + 3);
				
	float4* manifoldContactPointsLocalNormalPen = ((float4*) manifoldPtr);
	float4* manifoldContactPointsA = ((float4*) manifoldPtr) + PXG_MAX_PCM_CONTACTS;
	float4* manifoldContactPointsB = ((float4*) manifoldPtr) + 2 * PXG_MAX_PCM_CONTACTS;
	
	float4 pointB = make_float4(FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);
	float4 pointA = make_float4(FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);
	float4 pointNormPen = make_float4(FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);
		
	if (tidInGroup < initialNbContacts)
	{
		pointB = manifoldContactPointsB[tidInGroup];
		pointA = manifoldContactPointsA[tidInGroup];
		pointNormPen = manifoldContactPointsLocalNormalPen[tidInGroup];
	}

	//ML: refresh manifold contacts based on new transform
	const PxVec3 localAInB = aToB.transform(PxVec3(pointA.x, pointA.y, pointA.z)); // from a to b
	const PxVec3 localBInB = PxVec3(pointB.x, pointB.y, pointB.z);
	const PxVec3 v = localAInB - localBInB; 

	const PxVec3 localNormal(pointNormPen.x, pointNormPen.y, pointNormPen.z); // normal in b space
	const PxReal dist = v.dot(localNormal);

	pointNormPen.w = dist;

	const PxVec3 projectedPoint = localAInB - localNormal * dist;
	const PxVec3 projectedDifference = localBInB - projectedPoint;

	const PxReal distance2d = projectedDifference.magnitudeSquared();
	const PxReal projectBreakingThreshold = minMargin * 0.8f;
	const PxReal sqProjectBreakingThreshold =  projectBreakingThreshold * projectBreakingThreshold; 
	
	bool keep = tidInGroup < initialNbContacts && distance2d <= sqProjectBreakingThreshold;
	PxU32 keepContactW = __ballot_sync(FULL_MASK, keep);
	PxU32 keepContactGroup = keepContactW & (0xF << (threadIdx.x & ~0x3));
	//ML: calculate number of manifold contacts after updating based on new transform
	PxU32 newNbContacts = __popc(keepContactGroup);

	//ML: this decides whether we should run contact gen at all
	bool bRunNarrowphase = (newNbContacts != initialNbContacts);

	PxU32 contactIndex = __popc(keepContactGroup & ((1 << threadIdx.x) - 1));

	if (keep)
	{
		manifoldContactPointsB[contactIndex] = pointB; 
		manifoldContactPointsA[contactIndex] = pointA; 
		manifoldContactPointsLocalNormalPen[contactIndex] = pointNormPen;
	}
	
	if (!bRunNarrowphase)
	{
		//thread 0
		PxU32 lane = (threadIdx.x & ~0x3);
		PxVec3 relativePos = shuffle(FULL_MASK, PxVec3(__int_as_float(persistentContactManifoldMoreDataW.x),
			__int_as_float(persistentContactManifoldMoreDataW.y),
			__int_as_float(persistentContactManifoldMoreDataW.z)),
			lane);

		//thread 1
		lane = (threadIdx.x & ~0x3) + 1;
		PxQuat quatA = PxQuat(__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.x), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.y), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.z), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.w), lane));

		//thread 2
		lane = (threadIdx.x & ~0x3) + 2;
		PxQuat quatB = PxQuat(__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.x), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.y), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.z), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.w), lane));
		
		const PxReal radiusB = PxSqrt(extents1.x * extents1.x + extents1.y * extents1.y + extents1.z * extents1.z);

		if (type0 == PxGeometryType::eSPHERE || type0 == PxGeometryType::eCAPSULE)
		{
			const PxReal radiusA = extents0.y;
			bRunNarrowphase = invalidate_BoxConvex(aToB.p, t0.q, t1.q, relativePos, quatA, quatB, minMargin,
				radiusA, radiusB, newNbContacts, local_invalidateThresholdsSphere, local_invalidateQuatThresholdsSphere);
		}
		else
		{
			const PxReal radiusA = PxSqrt(extents0.x * extents0.x + extents0.y * extents0.y + extents0.z * extents0.z);
			bRunNarrowphase = invalidate_BoxConvex(aToB.p, t0.q, t1.q, relativePos, quatA, quatB, minMargin,
				radiusA, radiusB, newNbContacts, local_invalidateThresholdsConvex, local_invalidateQuatThresholdsConvex);
		}
	}

	if (tidInGroup == 3)
	{
		//ML: the first 8 bit store new number of contacts and the second 8 bit store the old number of contacts
		persistentContactManifoldMoreDataW.x = newNbContacts | (bRunNarrowphase ? (initialNbContacts << 8) : 0);//((bRunNarrowphase && bRegenerateAll) ? 0x80000000 : 0);
		
		tempKeepPairArray[globalElementIndex] = bRunNarrowphase;
	}
	
	if (bRunNarrowphase)
	{
		assert(aToB.isSane());
		
		if (tidInGroup == 0)
		{
			//relative pos
			persistentContactManifoldMoreDataW.x = __float_as_int(aToB.p.x);
			persistentContactManifoldMoreDataW.y = __float_as_int(aToB.p.y);
			persistentContactManifoldMoreDataW.z = __float_as_int(aToB.p.z);
			persistentContactManifoldMoreDataW.w = 0;
		}
		else if (tidInGroup == 1)
		{
			//quat A
			persistentContactManifoldMoreDataW.x = __float_as_int(t0.q.x);
			persistentContactManifoldMoreDataW.y = __float_as_int(t0.q.y);
			persistentContactManifoldMoreDataW.z = __float_as_int(t0.q.z);
			persistentContactManifoldMoreDataW.w = __float_as_int(t0.q.w);
		}
		else if (tidInGroup == 2)
		{
			persistentContactManifoldMoreDataW.x = __float_as_int(t1.q.x);
			persistentContactManifoldMoreDataW.y = __float_as_int(t1.q.y);
			persistentContactManifoldMoreDataW.z = __float_as_int(t1.q.z);
			persistentContactManifoldMoreDataW.w = __float_as_int(t1.q.w);
		}
	}
	
	if (tidInGroup <((sizeof(float4) + sizeof(PxAlignedQuat) * 2 + sizeof(PxU32) * 4) / sizeof(uint4)))
	{
		persistentContactManifoldMoreData[tidInGroup] =	persistentContactManifoldMoreDataW;
	}
}

__device__ void convexConvexNphase_stage2Kernel_core(PxU32 globalThreadGroupIndex,
												const PxgContactManagerInput* PX_RESTRICT cmInputs,
												PxgPersistentContactManifold* PX_RESTRICT cmManifolds,
												PxgShape* PX_RESTRICT shapes,
												const PxsCachedTransform* PX_RESTRICT transformCache,
												const PxReal* PX_RESTRICT contactDistance,
												const PxReal toleranceLength)
{
	const PxU32 warpsPerBlock = GJK_EPA_WARPS_PER_BLOCK;
		
	__shared__ EpaAndClipScratch epa_clip_scratch[warpsPerBlock];
	EpaAndClipScratch& ss_epa_clip_scratch = epa_clip_scratch[threadIdx.y];

	__shared__ char tScratch[sizeof(CollideScratch) * warpsPerBlock];
	CollideScratch* scratch = reinterpret_cast<CollideScratch*>(tScratch);

	CollideScratch& ss_scratch = scratch[threadIdx.y];

	const PxU32 sharedMemoryPerWarps = sizeof(float4) * 4;

	__shared__ PxU8 sScratch[warpsPerBlock * sharedMemoryPerWarps];
	PxU8* msScratch = &sScratch[threadIdx.y * sharedMemoryPerWarps];
	ScratchMemoryAllocator sAlloc(msScratch, sharedMemoryPerWarps);

	ScratchMemoryMarker marker(sAlloc);

	ss_scratch.cachedData.Reset();

	PxU32 globalElementIndex = globalThreadGroupIndex;

	PxgContactManagerInput input;
	PxgContactManagerInput_ReadWarp(input, cmInputs, globalElementIndex);

	PxgShape shape0;
	PxgShape_ReadWarp(shape0, shapes + input.shapeRef0);

	PxgShape shape1;
	PxgShape_ReadWarp(shape1, shapes + input.shapeRef1);

	PxGeometryType::Enum type0 = PxGeometryType::Enum(shape0.type);
	PxGeometryType::Enum type1 = PxGeometryType::Enum(shape1.type);
	PxU32 transformCacheRef0 = input.transformCacheRef0;
	PxU32 transformCacheRef1 = input.transformCacheRef1;

	const bool flip = (type1<type0);

	if (flip)
	{
		PxSwap(shape0, shape1);
		PxSwap(transformCacheRef0, transformCacheRef1);
	}

	type0 = PxGeometryType::Enum(shape0.type);
	type1 = PxGeometryType::Enum(shape1.type);

	PxsCachedTransform transfCache0, transfCache1;
	PxsCachedTransform_ReadWarp(transfCache0, transformCache + transformCacheRef0);
	PxsCachedTransform_ReadWarp(transfCache1, transformCache + transformCacheRef1);

	const PxTransform& transf0 = transfCache0.transform;
	assert(transf0.isSane());
	const PxTransform& transf1 = transfCache1.transform;
	assert(transf1.isSane());

	PxTransform aToB = transf1.transformInv(transf0);
	PxgPersistentContactManifold* manifoldPtr = cmManifolds + globalElementIndex;
	
	float4* manifoldContactPointsLocalNormalPen = ((float4*) manifoldPtr);
	float4* manifoldContactPointsA = ((float4*) manifoldPtr) + PXG_MAX_PCM_CONTACTS;
	float4* manifoldContactPointsB = ((float4*) manifoldPtr) + 2 * PXG_MAX_PCM_CONTACTS;

	float4* pVertices0;
	float expandedMargin, convexMargin0;
	if (type0 == PxGeometryType::eSPHERE)
	{
		pVertices0 = sAlloc.allocAligned<float4>(sizeof(float4), 16);
		pVertices0[0] = make_float4(0.f, 0.f, 0.f, 0.f);
		const PxReal sphereRaduis = shape0.scale.scale.x;
		expandedMargin = shape0.scale.scale.x; //sphere radius
		convexMargin0 = sphereRaduis;
	}
	else if (type0 == PxGeometryType::eCAPSULE)
	{
		pVertices0 = sAlloc.allocAligned<float4>(sizeof(float4)*2, 16);
		//printf("Capsule halfHeight = %f\n", shape0.scale.scale.z);
		pVertices0[0] = make_float4(1.f, 0.f, 0.f, 0.f);
		pVertices0[1] = make_float4(-1.f, 0.f, 0.f, 0.f);

		const PxReal sphereRaduis = shape0.scale.scale.y;
		expandedMargin = sphereRaduis; //sphere radius
		convexMargin0 = sphereRaduis;
	}
	else
	{
		size_t hullPtr0 = shape0.hullOrMeshPtr;
		const PxU8* convexPtrA = (PxU8*)hullPtr0 + sizeof(float4);
		const float4* tVertices0 = (const float4*)(convexPtrA + sizeof(uint4) + sizeof(float4));
		pVertices0 = const_cast<float4*>(tVertices0);
		expandedMargin = 0.f;

		const float4 extents0 = *((float4*)(convexPtrA + sizeof(uint4)));
		convexMargin0 = calculatePCMConvexMargin(extents0, shape0.scale.scale, toleranceLength);
	}

	size_t hullPtr1 = shape1.hullOrMeshPtr;
	const PxU8* convexPtrB = (PxU8*)hullPtr1 + sizeof(float4);
	const float4 extents1 = *((float4*)(convexPtrB + sizeof(uint4)));
	const float4* pVertices1 = (const float4*)(convexPtrB + sizeof(uint4) + sizeof(float4));

	if (threadIdx.x == 0)
	{
		ss_scratch.aToB = aToB;
		ss_scratch.tB = transf1;

		ss_scratch.manifoldContactPointsLocalNormalPen = manifoldContactPointsLocalNormalPen;
		ss_scratch.manifoldContactPointsA = manifoldContactPointsA;
		ss_scratch.manifoldContactPointsB = manifoldContactPointsB;

		ss_scratch.scale0 = shape0.scale.scale;
		ss_scratch.rot0 = shape0.scale.rotation;
		ss_scratch.shape0MaterialIndex = shape0.materialIndex;

		if (type0 == PxGeometryType::eSPHERE)
		{
			ss_scratch.nbVertices0 = 1;
			ss_scratch.nbPolygons0 = 0;
			ss_scratch.nbEdges0 = 0;
			ss_scratch.inSphereRadius0 = 0.f;
			//printf("type0 == PxGeometryType::eSPHERE \n");
		}
		else if (type0 == PxGeometryType::eCAPSULE)
		{
			ss_scratch.nbVertices0 = 2;
			ss_scratch.nbPolygons0 = 0;
			ss_scratch.nbEdges0 = 0;
			ss_scratch.inSphereRadius0 = 0.f;
		}
		else
		{
			size_t hullPtr0 = shape0.hullOrMeshPtr;
			const PxU8* convexPtrA = (PxU8*)hullPtr0 + sizeof(float4);
			const float4 extents0 = *((float4*)(convexPtrA + sizeof(uint4)));

			const uint4 tmp = *((uint4*)convexPtrA);
			const PxU32 polyData0_NbEdgesNbHullVerticesNbPolygons = tmp.x;

			ss_scratch.convexPtr0 = convexPtrA;
			ss_scratch.nbVertices0 = getNbVerts(polyData0_NbEdgesNbHullVerticesNbPolygons);
			ss_scratch.nbPolygons0 = getNbPolygons(polyData0_NbEdgesNbHullVerticesNbPolygons);
			ss_scratch.nbEdges0 = getNbEdges(polyData0_NbEdgesNbHullVerticesNbPolygons);
			
			PxReal minScale0 = 1.f;
			if (shape1.type == PxGeometryType::eBOX)
			{
				const PxVec3 scale0 = shape0.scale.scale;
				minScale0 = PxMin(scale0.x, scale0.y);
				minScale0 = PxMin(scale0.z, minScale0);
			}

			ss_scratch.inSphereRadius0 = extents0.w * minScale0;
			//printf("nbVertices0 %i nbPolygons0 %i nbEdges0 %i\n", ss_scratch.nbVertices0, ss_scratch.nbPolygons0, ss_scratch.nbEdges0);
		}

		ss_scratch.type0 = type0;

		ss_scratch.scale1 = shape1.scale.scale;
		ss_scratch.rot1 = shape1.scale.rotation;
		ss_scratch.shape1MaterialIndex = shape1.materialIndex;
		ss_scratch.type1 = type1;

		const uint4 tmp2 = *((uint4*)convexPtrB);
		const PxU32 polyData1_NbEdgesNbHullVerticesNbPolygons = tmp2.x;

		ss_scratch.convexPtr1 = convexPtrB;
		ss_scratch.nbVertices1 = getNbVerts(polyData1_NbEdgesNbHullVerticesNbPolygons);
		ss_scratch.nbPolygons1 = getNbPolygons(polyData1_NbEdgesNbHullVerticesNbPolygons);
		ss_scratch.nbEdges1 = getNbEdges(polyData1_NbEdgesNbHullVerticesNbPolygons);
		
		PxReal minScale1 = 1.f;
		if (shape1.type == PxGeometryType::eBOX)
		{
			const PxVec3 scale1 = shape1.scale.scale;
			minScale1 = PxMin(scale1.x, scale1.y);
			minScale1 = PxMin(scale1.z, minScale1);
		}
		ss_scratch.inSphereRadius1 = extents1.w * minScale1;

		//printf("nbVertices1 %i nbPolygons1 %i nbEdges1 %i\n", ss_scratch.nbVertices1, ss_scratch.nbPolygons1, ss_scratch.nbEdges1);

		ss_scratch.contactDistance = contactDistance[transformCacheRef0] + contactDistance[transformCacheRef1] + expandedMargin;
		ss_scratch.toleranceLength = toleranceLength;

	}
	
	PxU32* persistentContactManifoldMoreData = (PxU32*) (((float4*) manifoldPtr) + 3 * PXG_MAX_PCM_CONTACTS);

	PxU32 persistentContactManifoldMoreDataW;

	//using 16 threads to read persistentContactManifoldMoreData
	if (threadIdx.x < (sizeof(float4) + sizeof(PxAlignedQuat) * 2 + sizeof(PxU32) * 4) / sizeof(PxU32))
	{
		persistentContactManifoldMoreDataW = persistentContactManifoldMoreData[threadIdx.x];
	}
	
	const PxReal convexMargin1 = calculatePCMConvexMargin(extents1, shape1.scale.scale, toleranceLength);
	const PxReal minMargin = fminf(convexMargin0, convexMargin1);
	const PxReal replaceBreakingThreshold = minMargin * 0.05f;
	
	//ML : the first 8 bits store the number of contacts after refreshing the manifold, 
	//the second 8 bits store the number of contacts before refreshing the manifold
	PxU32 numManifoldContacts32 = __shfl_sync(FULL_MASK, (int)persistentContactManifoldMoreDataW, 12);
									
	if (threadIdx.x == 0)
	{
		ss_scratch.replaceBreakingThreshold = replaceBreakingThreshold;
	
		ss_scratch.initialNbContacts = numManifoldContacts32 >> 8;
	}
	
	numManifoldContacts32 = numManifoldContacts32 & 0xFF;

	PxU8 numManifoldContacts = (PxU8) numManifoldContacts32;

	__syncwarp();

	bool bIntersection = false;
	if (transf0.isValid() && transf1.isValid())
	{
		bIntersection = collide(
			pVertices0,
			pVertices1,
			persistentContactManifoldMoreDataW,
			numManifoldContacts,
			//shared mem temps
			ss_epa_clip_scratch,
			ss_scratch,
			flip);
	}

	if (threadIdx.x == 12)
	{
		assert(numManifoldContacts <= 4);
		persistentContactManifoldMoreDataW = numManifoldContacts | (bIntersection ? 0 : 0x80000000);
	}

	const PxU32 constantDataSize = (sizeof(float) + sizeof(PxAlignedQuat) * 2);
	if ((threadIdx.x >= constantDataSize / sizeof(PxU32))
		&& (threadIdx.x < (constantDataSize + sizeof(PxU32) * 4) / sizeof(PxU32)))
	{
		persistentContactManifoldMoreData[threadIdx.x] = persistentContactManifoldMoreDataW;
	}
}

extern "C"
__launch_bounds__(GJK_EPA_WARPS_PER_BLOCK * WARP_SIZE)
__global__ void convexConvexNphase_stage2Kernel(
								PxU32 numTests,
								const PxReal toleranceLength,
								const PxU32* PX_RESTRICT tempKeepPairsArray,
								const PxgContactManagerInput* PX_RESTRICT cmInputs,
								PxgPersistentContactManifold* PX_RESTRICT cmManifolds,
								PxgShape* PX_RESTRICT shapes,
								const PxsCachedTransform* PX_RESTRICT transformCache,
								const PxReal* contactDistance)
{
	PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;
	
	if (globalWarpIndex >= numTests || (!tempKeepPairsArray[globalWarpIndex]))
		return;

	/*if (!tempKeepPairsArray[globalThreadGroupIndex])
		return;*/

	convexConvexNphase_stage2Kernel_core(globalWarpIndex,
												cmInputs,
												cmManifolds,
												shapes,
												transformCache,
												contactDistance,
												toleranceLength);
}

__device__ inline static
PxI32 getPolygonIndexFromPlaneNormal(
	PxU32 polyData_NbPolygons,
	const float4* pPolygonPlanes,
	const PxVec3& scale, 
	const PxQuat& rot,
	const PxVec3& n,						//shape space
	const PxU8* facesByEdges8,
	const PxU32 nbEdges,
	PxI32& plane2
	)
{
	//normal is in shape space, don't transfer to vertex space. If the shape has huge scale,
	//keep the normal in shape space and transfer plane normal to shape give us a much reliable
	//result

	PxReal bestProjection = -PX_MAX_F32;
	int index = 0;

	for (PxU32 i = threadIdx.x; i < polyData_NbPolygons; i += WARP_SIZE)
	{
		float4 fplane = pPolygonPlanes[i];
		PxPlane plane = PxPlane(fplane.x, fplane.y, fplane.z, fplane.w);
		
		//transform plane normal to shape shapce
		PxVec3 planeN = vertex2ShapeNormalVector(plane.n, scale, rot).getNormalized();
		PxReal proj = planeN.dot(n);

		if (bestProjection < proj)
		{
			index = i;
			bestProjection = proj;
		}
	}

	PxU32 bestProjLane;
	bestProjection = warpReduction<MaxOpFloat, float>(FULL_MASK, bestProjection, bestProjLane);

	index = __shfl_sync(FULL_MASK, (int)index, bestProjLane);

	if (1)
	{
		PxU32 closestEdge = 0xFFFFFFFF;

		PxReal bestEdge = -PX_MAX_F32;

		//if(threadIdx.x == 0)
		//	printf("%i: BestConvexFaceIndex = %i, minProj = %f, maxDpSq = %f, n = (%f, %f, %f)\n", threadIdx.y, polyFaceIndex, minProj, maxDpSq,
		//		featureNormal_vertexSpace.x, featureNormal_vertexSpace.y, featureNormal_vertexSpace.z);

		for (PxU32 i = threadIdx.x; i < nbEdges; i += WARP_SIZE)
		{
			PxU32 index = i * 2;
			const PxU8 f0 = facesByEdges8[index];
			const PxU8 f1 = facesByEdges8[index + 1];

			PxVec3 planeN0 = loadNormal(pPolygonPlanes[f0], scale, rot);
			PxVec3 planeN1 = loadNormal(pPolygonPlanes[f1], scale, rot);

			PxVec3 edgeN = (planeN0 + planeN1).getNormalized();
			PxReal dp = edgeN.dot(n);

			if (dp > bestEdge)
			{
				bestEdge = dp;
				closestEdge = i;
			}
		}

		//Add on bias because we prefer edge contacts - they generate contacts with 2 faces
		bestEdge = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, bestEdge, bestProjLane) +5e-4f; 
		closestEdge = __shfl_sync(FULL_MASK, closestEdge, bestProjLane);

		if (bestEdge > bestProjection)
		{
			//We found a better edge than the faces, so we pick both faces from the edge...
			PxU32 index = closestEdge * 2;
			const PxU8 f0 = facesByEdges8[index];
			const PxU8 f1 = facesByEdges8[index + 1];

			PxVec3 planeN0 = loadNormal(pPolygonPlanes[f0], scale, rot);
			PxVec3 planeN1 = loadNormal(pPolygonPlanes[f1], scale, rot);

			PxReal dp0 = planeN0.dot(n);
			PxReal dp1 = planeN1.dot(n);

			if (dp0 > dp1)
			{
				index = f0;
				plane2 = f1;
			}
			else
			{
				index = f1;
				plane2 = f0;
			}
		}
	}

	assert(index != 0xffffffff);
	return index;
}

//1 warp to deal with one pair, CONVEX_PLANE_WARPS_PER_BLOCK
__device__ void convexPlaneNphase_kernel_core(
	PxU32 globalElementIndex,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxgShape* PX_RESTRICT shapes,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* PX_RESTRICT contactDistance,
	const PxsMaterialData* PX_RESTRICT materials,
	PxU8* PX_RESTRICT contactStream,
	PxU8* PX_RESTRICT patchStream,
	PxgPersistentContactManifold* PX_RESTRICT contactManifolds,
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
	const PxU32 warpIndex = threadIdx.y;
	//32 contacts

	__shared__ char sContacts[sizeof(TemporaryContacts) * CONVEX_PLANE_WARPS_PER_BLOCK];

	TemporaryContacts* contactsS = reinterpret_cast<TemporaryContacts*>(sContacts);
	
	PxgShape planeShape, convexShape;
	PxU32 planeCacheRef, convexCacheRef;
	//convexShape type can apparently also be PxGeometryType::eBOX
	LoadShapePairWarp<PxGeometryType::ePLANE>(cmInputs, globalElementIndex, shapes,
		planeShape, planeCacheRef, convexShape, convexCacheRef);
	
	assert(PxGeometryType::Enum(convexShape.type) == PxGeometryType::eBOX ||
		PxGeometryType::Enum(convexShape.type) == PxGeometryType::eCONVEXMESH);

	PxsCachedTransform planeTransformCache, convexTransformCache;
	PxsCachedTransform_ReadWarp(planeTransformCache, transformCache + planeCacheRef);
	PxsCachedTransform_ReadWarp(convexTransformCache, transformCache + convexCacheRef);

	const PxTransform& planeTransform = planeTransformCache.transform;
	assert(planeTransform.isSane());
	const PxTransform& convexTransform = convexTransformCache.transform;
	assert(convexTransform.isSane());

	PxReal contactDist = contactDistance[planeCacheRef] + contactDistance[convexCacheRef];
	
	PxVec3 localNormal = PxVec3(1.0f, 0.f, 0.f);
	
	size_t hullPtr1 = convexShape.hullOrMeshPtr;
	const PxU8* convexPtrB = (PxU8*)hullPtr1 + sizeof(float4);
	const float4 extents1 = *((float4*)(convexPtrB + sizeof(uint4)));
	const float4* pVertices = (const float4*)(convexPtrB + sizeof(uint4) + sizeof(float4));

	const uint4 tmp2 = *((uint4*)convexPtrB);
	const PxU32 polyData_NbEdgesNbHullVerticesNbPolygons = tmp2.x;
	const PxU32 nbVertices = getNbVerts(polyData_NbEdgesNbHullVerticesNbPolygons);
	const PxU32 nbPolygons = getNbPolygons(polyData_NbEdgesNbHullVerticesNbPolygons);
	const PxU32 nbEdges = getNbEdges(polyData_NbEdgesNbHullVerticesNbPolygons);

	const float4* pPolygonPlanes = (const float4*)(convexPtrB + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * nbVertices);

	const PxU32* vRef8NbVertsMinIndex = (const PxU32*)(convexPtrB + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * nbVertices
		+ sizeof(float4) * nbPolygons);

	const PxU8* facesByEdges8 = (const PxU8*)(convexPtrB + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4)* nbVertices
		+ (sizeof(float4) + sizeof(PxU32)) * nbPolygons
		+ sizeof(PxU16) * 2 * nbEdges);

	const PxU8* polyData_vertexData8 = (const PxU8*)(convexPtrB + sizeof(uint4) + sizeof(float4)
		+ sizeof(float4) * nbVertices
		+ (sizeof(float4) + sizeof(PxU32)) * nbPolygons
		+ (sizeof(PxU16) + sizeof(PxU8)) * 2 * nbEdges
		+ sizeof(PxU8) * 3 * nbVertices);

	PxVec3 scale = convexShape.scale.scale;
	PxQuat rot = convexShape.scale.rotation;

	//convex to plane
	PxTransform bToA = planeTransform.transformInv(convexTransform);

	//rotate localPlane normal to convex hull shape space
	const PxVec3 n = (bToA.rotateInv(localNormal)).getNormalized();

	PxVec3 p;
	PxReal sep;
	
	bool bRunNarrowphase = true;

	const PxReal minMargin = calculatePCMConvexMargin(extents1, convexShape.scale.scale, toleranceLength);

	PxgPersistentContactManifold* PX_RESTRICT manifoldPtr = &contactManifolds[globalElementIndex];

	uint4* persistentContactManifoldMoreData = (uint4*)(((float4*)manifoldPtr) + 3 * PXG_MAX_PCM_CONTACTS);
	uint4 persistentContactManifoldMoreDataW;

	if (threadIdx.x < ((sizeof(float4) + sizeof(PxAlignedQuat) * 2 + sizeof(PxU32) * 4) / sizeof(uint4)))
	{
		persistentContactManifoldMoreDataW = persistentContactManifoldMoreData[threadIdx.x];
	}

	PxU32 initialNbContacts = __shfl_sync(FULL_MASK, persistentContactManifoldMoreDataW.x, 3);

	float4* manifoldContactPointsLocalNormalPen = ((float4*)manifoldPtr);
	float4* manifoldContactPointsA = ((float4*)manifoldPtr) + PXG_MAX_PCM_CONTACTS;
	float4* manifoldContactPointsB = ((float4*)manifoldPtr) + 2 * PXG_MAX_PCM_CONTACTS;

	float4 pointB = make_float4(FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);
	float4 pointA = make_float4(FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);
	//float4 pointNormPen = make_float4(FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);

	if (threadIdx.x < initialNbContacts)
	{
		pointB = manifoldContactPointsB[threadIdx.x];
		pointA = manifoldContactPointsA[threadIdx.x];
		//pointNormPen = manifoldContactPointsLocalNormalPen[threadIdx.x];
	}

	//ML: refresh manifold contacts based on new transform
	const PxVec3 localAInB = bToA.transformInv(PxVec3(pointA.x, pointA.y, pointA.z)); // from a to b
	const PxVec3 localBInB = PxVec3(pointB.x, pointB.y, pointB.z);
	const PxVec3 v = localBInB - localAInB;

	//const PxVec3 manifoldNorm(pointNormPen.x, pointNormPen.y, pointNormPen.z); // normal in b space
	const PxReal dist = v.dot(n);

	const PxVec3 projectedDifference = v - n*dist;

	const PxReal distance2d = projectedDifference.magnitudeSquared();
	const PxReal projectBreakingThreshold = minMargin * 0.8f;
	const PxReal sqProjectBreakingThreshold = projectBreakingThreshold * projectBreakingThreshold;

	bool keep = threadIdx.x < initialNbContacts && distance2d <= sqProjectBreakingThreshold;
	PxU32 keepContactW = __ballot_sync(FULL_MASK, keep);
	//ML: calculate number of manifold contacts after updating based on new transform
	PxU32 nbContacts = __popc(keepContactW);

	//ML: this decides whether we should run contact gen at all
	bRunNarrowphase = (nbContacts != initialNbContacts);

	int mask = 0;

	if (!bRunNarrowphase)
	{
		//thread 0
		PxU32 lane = 0;
		PxVec3 relativePos = shuffle(FULL_MASK, PxVec3(__int_as_float(persistentContactManifoldMoreDataW.x),
			__int_as_float(persistentContactManifoldMoreDataW.y),
			__int_as_float(persistentContactManifoldMoreDataW.z)),
			lane);

		//thread 1
		lane = 1;
		PxQuat quatA = PxQuat(__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.x), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.y), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.z), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.w), lane));

		//thread 2
		lane = 2;
		PxQuat quatB = PxQuat(__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.x), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.y), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.z), lane),
			__shfl_sync(FULL_MASK, __int_as_float(persistentContactManifoldMoreDataW.w), lane));

		const PxReal radiusB = PxSqrt(extents1.x * extents1.x + extents1.y * extents1.y + extents1.z * extents1.z);

		bRunNarrowphase = invalidate_BoxConvex(-bToA.p, planeTransform.q, convexTransform.q, relativePos, quatA, quatB, minMargin,
			radiusB, radiusB, nbContacts, local_invalidateThresholdsConvex, local_invalidateQuatThresholdsConvexPlane);
	}

	if (bRunNarrowphase)
	{
		//get the feature index
		PxI32 plane2 = -1;
		const PxU32 bestPlaneBIdx = getPolygonIndexFromPlaneNormal(nbPolygons, pPolygonPlanes, scale, rot, -n, facesByEdges8, nbEdges, plane2);

		//each thread read one vert

		PxU32 totalContacts = 0;

		for (PxU32 planeIndex = bestPlaneBIdx; planeIndex != 0xFFFFFFFF;)
		{
			PxVec3 pInPlaneSpace;
			PxReal signDist = PX_MAX_F32;
			const PxU32 polyDesc = vRef8NbVertsMinIndex[bestPlaneBIdx];
			if (threadIdx.x < getNbVerts(polyDesc))
			{
				const PxU32 index = polyData_vertexData8[getVRef8(polyDesc) + threadIdx.x];
				const float4 p = pVertices[index];
				const PxVec3 pInShapeSpace = vertex2Shape(PxVec3(p.x, p.y, p.z), scale, rot);
				pInPlaneSpace = bToA.transform(pInShapeSpace);
				signDist = pInPlaneSpace.x;
			}

			PxU32 flags = __ballot_sync(FULL_MASK, signDist < contactDist);

			PxU32 nbContacts = __popc(flags);

			//__syncwarp();

			PxVec3 p = pInPlaneSpace;
			PxReal sep = signDist;

			if (nbContacts)
			{
				if (nbContacts > 16)
				{
					flags = reduce<false, false, 4>(p, sep, localNormal, nbContacts, flags);
					//how many contacts we have now
					nbContacts = __popc(flags);
				}

				if (flags & (1 << threadIdx.x))
				{
					PxU32 index = __popc(flags & ((1 << threadIdx.x) - 1));
					contactsS[warpIndex].pos[totalContacts + index] = p;
					contactsS[warpIndex].sep[totalContacts + index] = sep;
				}

				totalContacts += nbContacts;
			}

			planeIndex = plane2;
			plane2 = 0xFFFFFFFF;
		}

		nbContacts = totalContacts;

		__syncwarp();

		if (nbContacts)
		{
			mask = (1 << totalContacts) - 1;

			if (threadIdx.x < nbContacts)
			{
				p = contactsS[warpIndex].pos[threadIdx.x];
				sep = contactsS[warpIndex].sep[threadIdx.x];
			}
			if (nbContacts > 4)
			{
				mask = reduce<false, false, 4>(p, sep, localNormal, nbContacts, mask);
				//how many contacts we have now
				nbContacts = __popc(mask);
			}

			if (mask & (1 << threadIdx.x))
			{
				//store in manifold...

				PxU32 offset = warpScanExclusive(mask, threadIdx.x);
				assert(offset < PXG_MAX_PCM_CONTACTS);

				//Write to manifold
				//p is in plane space, but it is on the surface of the convex. We need it to be on the surface of the plane.
				//additionally, we need to transform from plane space to convex space
				PxVec3 pInB = bToA.transformInv(p);
				PxVec3 pOnPlane = p;
				pOnPlane.x -= sep;

				manifoldContactPointsB[offset] = make_float4(pInB.x, pInB.y, pInB.z, 0.f);
				manifoldContactPointsA[offset] = make_float4(pOnPlane.x, pOnPlane.y, pOnPlane.z, 0.f);
				manifoldContactPointsLocalNormalPen[offset] = make_float4(n.x, n.y, n.z, sep);
			}

			if (threadIdx.x < 3)
			{
				float4 outVal = threadIdx.x == 0 ? make_float4(-bToA.p.x, -bToA.p.y, -bToA.p.z, 0.f) :
					threadIdx.x == 1 ? make_float4(planeTransform.q.x, planeTransform.q.y, planeTransform.q.z, planeTransform.q.w) :
					make_float4(convexTransform.q.x, convexTransform.q.y, convexTransform.q.z, convexTransform.q.w);

				(&manifoldPtr->mRelativePos)[threadIdx.x] = outVal;
			}
			if (threadIdx.x == 0)
			{
				manifoldPtr->mNbContacts = nbContacts;
			}
		}
	}
	else
	{
		mask = (1 << nbContacts) - 1;
		//Need to load p and sep from the PCM!
		if (threadIdx.x < nbContacts)
		{
			p = bToA.transform(PxVec3(pointB.x, pointB.y, pointB.z));
			sep = dist;
		}
	}

	PxU32 contactByteOffset = 0xFFFFFFFF;
	PxU32 forceAndIndiceByteOffset = 0xFFFFFFFF;

	if (nbContacts && threadIdx.x == 0)
	{
		contactByteOffset = atomicAdd(&(patchAndContactCounters->contactsBytes), sizeof(PxContact) * nbContacts);
		forceAndIndiceByteOffset = atomicAdd(&(patchAndContactCounters->forceAndIndiceBytes), sizeof(PxU32) * nbContacts);

		if ((contactByteOffset + sizeof(PxContact)) > contactBytesLimit)
		{
			patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW);
			contactByteOffset = 0xFFFFFFFF; //overflow
		}
		
		if ((forceAndIndiceByteOffset + sizeof(PxU32)) > forceBytesLimit)
		{
			patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::FORCE_BUFFER_OVERFLOW);
			forceAndIndiceByteOffset = 0xFFFFFFFF; //overflow
		}
	}

	contactByteOffset = __shfl_sync(FULL_MASK, contactByteOffset, 0);
	forceAndIndiceByteOffset = __shfl_sync(FULL_MASK, forceAndIndiceByteOffset, 0);

	PxsContactManagerOutput* output = cmOutputs + globalElementIndex;

	//write point and penetration to the contact stream
	if (contactByteOffset != 0xFFFFFFFF)
	{
		float4* baseContactStream = (float4*)(contactStream + contactByteOffset);

		if (mask & (1 << threadIdx.x))
		{
			int index = warpScanExclusive(mask, threadIdx.x);
			const PxVec3 worldP = planeTransform.transform(p);
			baseContactStream[index] = make_float4(worldP.x, worldP.y, worldP.z, sep);
		}

		output->contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
		output->contactPoints = startContactPoints + contactByteOffset;
	}
	else
	{
		output->contactForces = NULL;
		output->contactPoints = NULL;
	}

	PxU32 allflags = reinterpret_cast<PxU32*>(&((cmOutputs + globalElementIndex)->allflagsStart))[0];

	PxU8 oldStatusFlags = u16Low(u32High(allflags));
	PxU8 statusFlags = oldStatusFlags;

	statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);

	if (nbContacts)
		statusFlags |= PxsContactManagerStatusFlag::eHAS_TOUCH;
	else
		statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

	PxU8 prevPatches = u16High(u32Low(allflags)); //Get out the current number of patches to store as the previous frame's number of patches
	bool previouslyHadTouch = oldStatusFlags & PxsContactManagerStatusFlag::eHAS_TOUCH;
	bool prevTouchKnown = oldStatusFlags & PxsContactManagerStatusFlag::eTOUCH_KNOWN;

	PxU8 numPatches = (nbContacts > 0) ? 1 : 0;

	bool currentlyHasTouch = nbContacts > 0;

	const bool change = (previouslyHadTouch ^ currentlyHasTouch) || (!prevTouchKnown);
	touchChangeFlags[globalElementIndex] = change;
	patchChangeFlags[globalElementIndex] = (prevPatches != numPatches);

	if (threadIdx.x == 0)
	{
		reinterpret_cast<PxU32*>(&((cmOutputs + globalElementIndex)->allflagsStart))[0] = merge(merge(prevPatches, statusFlags),
			merge(numPatches, 0));
		cmOutputs[globalElementIndex].nbContacts = nbContacts;
	}

	//fill in patch information
	PxU32 patchIndex = 0xFFFFFFFF;
	if (nbContacts && threadIdx.x == 0)
	{
		patchIndex = atomicAdd(&(patchAndContactCounters->patchesBytes), sizeof(PxContactPatch));

		if ((patchIndex + sizeof(PxContactPatch)) > patchBytesLimit)
		{
			//patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW);
			//patchIndex = 0xFFFFFFFF; //overflow

			patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW);
			patchIndex = 0xFFFFFFFF; //overflow

			statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);
			statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

			reinterpret_cast<PxU32*>(&((cmOutputs + globalElementIndex)->allflagsStart))[0] = merge(merge(prevPatches, statusFlags), 0);
			cmOutputs[globalElementIndex].nbContacts = 0;

			touchChangeFlags[globalElementIndex] = previouslyHadTouch;
			patchChangeFlags[globalElementIndex] = prevPatches != 0;
		}
		else
		{
			(cmOutputs + globalElementIndex)->contactPatches = startContactPatches + patchIndex;
		}
	}

	if (threadIdx.x == 0)
		insertIntoPatchStream(materials, patchStream, planeShape, convexShape, patchIndex, planeTransform.rotate(localNormal), nbContacts);	
}

extern "C" __global__ void convexPlaneNphase_Kernel(
	PxU32 numTests,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxgShape* PX_RESTRICT shapes,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* contactDistance,
	const PxsMaterialData* PX_RESTRICT materials,
	PxU8* PX_RESTRICT contactStream,
	PxU8* PX_RESTRICT patchStream,
	PxgPersistentContactManifold* PX_RESTRICT contactManifolds,
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
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;

	if (globalWarpIndex >= numTests)
		return;

	convexPlaneNphase_kernel_core(
		globalWarpIndex,
		cmInputs,
		cmOutputs,
		shapes,
		transformCache,
		contactDistance,
		materials,
		contactStream,
		patchStream,
		contactManifolds,
		patchAndContactCounters,
		touchChangeFlags,
		patchChangeFlags,
		startContactPatches,
		startContactPoints,
		startContactForces,
		patchBytesLimit,
		contactBytesLimit,
		forceBytesLimit,
		toleranceLength);
}

