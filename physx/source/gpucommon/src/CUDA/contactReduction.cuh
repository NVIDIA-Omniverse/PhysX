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

#ifndef __CU_CONTACT_REDUCTION_CUH__
#define __CU_CONTACT_REDUCTION_CUH__

#include "utils.cuh"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "shuffle.cuh"
#include "nputils.cuh"

namespace physx
{

//See trunk/internaldocumentation/Solver/PhysX 3 constraint solver.doc
//* If the number of points in the patch is more than 6, for each patch
//* We find the most extreme point, p0.This is the point that is farthest from the origin. ALGORITHM BELOW TAKES DEEPEST CONTACT AS p0.
//* We find the point farthest from the most extreme point : p1.
//* We find point p2, which is the point farthest from the segment p0p1.
//* We find the direction from p2 to the closest point to p2 on segment p0p1.We then find point p3 : the point farthest 
//  from p0p1 in that direction.
//* These 4 points define the anchors for our clusters.We then assign the points to their respective clusters, i.e.the 
//  cluster which the contact point is closest to.In a case where a point is equidistant between 2 anchors, the earlier anchor 
//  in the array of anchors is arbitrarily chosen.
//* We choose the deepest point in each cluster.We slightly bias the initial points p0 - p3 by considering them to have deeper 
//  penetrations than they actually have; these are biased by an epsilon.This avoids oscillation between points when they have roughly 
//  equal depth, which can cause instability in the friction model.The deepest contact point in each cluster is selected.
//* Finally, we choose 2 remaining contacts.These contacts are the 2 deepest unselected contacts.

template<bool TClustering, bool TKeepAnotherDeepestForPCM, int TMaxPoints, bool TDoDupeTest = true>
static __device__ int contactReduceShared(const PxVec3& pointRaw, const PxReal separation, const PxVec3& normal, int allMask,
	PxReal clusterBias, PxReal distanceAdjustment, PxReal initialPointCriteria)
{
	PxReal v, w;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;



	if (TDoDupeTest)
	{
		bool imADupe = false;
		for (PxU32 m = allMask; m; m = clearLowestSetBit(m))
		{
			int i = lowestSetIndex(m);
			PxReal d = (shuffle(FULL_MASK, pointRaw, i) - pointRaw).magnitudeSquared();
			PxReal sep = __shfl_sync(FULL_MASK, separation, i);
			if (d < 1e-8f && (sep < separation || (sep == separation && i > threadIndexInWarp)))
			{
				imADupe = true;
			}
		}
		allMask &= ~(__ballot_sync(FULL_MASK, imADupe));
	}

	PxU32 newCount = __popc(allMask);

	if (newCount <= TMaxPoints)
	{
		return allMask;
	}

	const PxVec3 point = pointRaw - normal * pointRaw.dot(normal);

	//Distance calculation is altered by separation value to give further away contacts less weight
	int i0 = maxIndex(initialPointCriteria + distanceAdjustment, allMask, v);										// p0 - most extreme contact (furthest away from origin)
	int mask = 1 << i0;
	PxReal dist = (shuffle(FULL_MASK, point, i0) - point).magnitude();
	int i1 = maxIndex(dist + distanceAdjustment, allMask&~mask, v);	// p1 - furthest from p0, when projected onto normal plane
	mask |= 1 << i1;

	//Now we have the p0-p1 edge. We try to find the point furthest from it in the normal plane.
	//For that, we look for the 2 extreme points - one to the right and one to the left
	//One maximizes [(p1 - p0) x n] * (p - p0), the other one minimizes that
	//[(p1 - p0) x n] * (p - p0) = [n x (p - p0)] * (p1 - p0) = (n x p) * p1 - (n x p) * p0  - (n x p0) * p1 + (n x p0) * p0 =
	//= k1 - k0 - k1[0], as (n x p0) * p0 = 0

	PxVec3 dir = normal.cross(shuffle(FULL_MASK, point, i1) - shuffle(FULL_MASK, point, i0));

	PxReal d = dir.dot(point - shuffle(FULL_MASK, point, i0));

	int f = maxIndex(d + distanceAdjustment, allMask&~mask, v);
	mask |= (1 << f);

	int g = minIndex(d - distanceAdjustment, allMask&~mask, w);

	//if (__shfl_sync(FULL_MASK, d, f) * __shfl_sync(FULL_MASK, d, g) > 0.f)
	//{
	//	//We need to pick again...
	//	g = maxIndex(d, allMask&~mask, v);
	//}

	mask |= (1 << g);

	//if (TKeepAnotherDeepestForPCM && __popc(mask) == 4)
	bool predicate = (TKeepAnotherDeepestForPCM && __popc(mask) == 4);

	//unsigned mask_predicate = __ballot_sync(FULL_MASK, predicate);
	if (predicate && TMaxPoints > 4)
	{
		int i4 = minIndex(separation, allMask&~mask, v);
		mask |= (1 << i4);
	}

	// post-cull clustering for mesh collisions
	//unsigned mask_TClustering = __ballot_sync(syncMask, TClustering);
	if (TClustering)
	{

		PxReal sep = separation;
		if (mask & (1 << threadIndexInWarp))
			sep -= clusterBias;

		int nbClusters = 0, label = -1;													// label each point with its closest cluster (distance measured orthogonal to the normal)
		for (PxReal t = FLT_MAX; mask; nbClusters++, mask &= (mask - 1))
		{
			PxReal d = (point - shuffle(FULL_MASK, point, lowestSetIndex(mask))).magnitudeSquared();
			if (d < t)
				t = d, label = nbClusters;
		}

		mask = 0;

		for (int i = 0; i < nbClusters; i++)												// find a point in each cluster (clusters can be empty if all input points are equal)
		{
			int cluster = __ballot_sync(FULL_MASK, label == i)&allMask;
			if (cluster)
				mask |= 1 << minIndex(sep, cluster, v);
		}

		for (int i = nbClusters; i < TMaxPoints; i++)										// fill out the rest of the points
			mask |= 1 << minIndex(sep, allMask&~mask, v);
	}
	else
	{
		PxU32 count = __popc(mask);

		for (PxU32 i = count; i < TMaxPoints; ++i)
			mask |= 1 << minIndex(separation, allMask&~mask, v);
	}

	return mask;
}


template<bool TClustering, bool TKeepAnotherDeepestForPCM, int TMaxPoints, bool TDoDupeTest = true>
static __device__ int contactReduce(const PxVec3& pointRaw, const PxReal separation, const PxVec3& normal, int allMask,
	PxReal clusterBias)
{
	return contactReduceShared<TClustering, TKeepAnotherDeepestForPCM, TMaxPoints, TDoDupeTest>(pointRaw, separation, normal, allMask,
		clusterBias, 0.0f, separation);
}


template<bool TClustering, bool TKeepAnotherDeepestForPCM, int TMaxPoints, bool TDoDupeTest = true>
static __device__ int contactReduce2(const PxVec3& pointRaw, const PxReal separation, const PxVec3& normal, int allMask,
	PxReal clusterBias)
{	
	const PxVec3 point = pointRaw - normal * pointRaw.dot(normal);
	PxReal distToOrigin = point.magnitude();
	return contactReduceShared<TClustering, TKeepAnotherDeepestForPCM, TMaxPoints, TDoDupeTest>(pointRaw, separation, normal, allMask,
		clusterBias, -separation, distToOrigin);
}

}
#endif
