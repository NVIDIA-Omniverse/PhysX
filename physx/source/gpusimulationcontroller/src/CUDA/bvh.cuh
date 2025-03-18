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

#ifndef __CU_BVH_CUH__
#define __CU_BVH_CUH__

#include "foundation/PxVec3.h"
#include "foundation/PxBounds3.h"
#include "PxgBVH.h"
#include "GuDistancePointTriangle.h"
#include "foundation/PxMath.h"

using namespace physx;

PX_FORCE_INLINE __device__ PxU32 part1by2(PxU32 n)
{
	n = (n ^ (n << 16)) & 0xff0000ff;
	n = (n ^ (n << 8)) & 0x0300f00f;
	n = (n ^ (n << 4)) & 0x030c30c3;
	n = (n ^ (n << 2)) & 0x09249249;

	return n;
}

// Takes values in the range [0, 1] and assigns an index based Morton codes of length 3*log2(Dim) bits 
template <PxI32 Dim>
PX_FORCE_INLINE __device__ PxU32 morton3(PxReal x, PxReal y, PxReal z)
{
	PxU32 ux = PxClamp(PxI32(x*Dim), 0, Dim - 1);
	PxU32 uy = PxClamp(PxI32(y*Dim), 0, Dim - 1);
	PxU32 uz = PxClamp(PxI32(z*Dim), 0, Dim - 1);

	return (part1by2(uz) << 2) | (part1by2(uy) << 1) | part1by2(ux);
}


struct BvhTraversalControl
{
	enum Enum
	{
		eDontGoDeeper,
		eGoDeeper,
		eGoDeeperLowerFirst,
		eAbort
	};
};

template <typename Func>
PX_FORCE_INLINE __device__ void queryBVH(const PxgBVH& bvh, Func& f, PxI32* stack, PxU32 stackSize)
{
	if (bvh.mNumNodes == 0)
		return;

	PxI32 index = *bvh.mRootNode;
	PxI32 count = 0;

	const PxU32 maxIter = bvh.mMaxNodes;
	for(PxU32 iter = 0; iter < maxIter; ++iter)
	{
		// union to allow 128-bit loads
		//union { PxgPackedNodeHalf lower; float4 lowerf; };
		//union { PxgPackedNodeHalf upper; float4 upperf; };

		PxgPackedNodeHalf lower = bvh.mNodeLowers[index];
		PxgPackedNodeHalf upper = bvh.mNodeUppers[index];
		//lowerf = tex1Dfetch<float4>(bvh.mNodeLowersTex, index);
		//upperf = tex1Dfetch<float4>(bvh.mNodeUppersTex, index);		

		BvhTraversalControl::Enum control = f(lower, upper, index);
		if (control == BvhTraversalControl::eAbort)
			break;
		if (!lower.b && (control == BvhTraversalControl::eGoDeeper || control == BvhTraversalControl::eGoDeeperLowerFirst))
		{
			if (control == BvhTraversalControl::eGoDeeperLowerFirst)
			{
				if(count < stackSize)
					stack[count++] = upper.i;
				index = lower.i; //index gets processed next - assign lower index to it				
			}
			else
			{
				if (count < stackSize)
					stack[count++] = lower.i;
				index = upper.i; //index gets processed next - assign upper index to it	
			}
			continue;
		}
		if (count == 0)
			break;
		index = stack[--count];
	}
}

PX_FORCE_INLINE __device__ PxgPackedNodeHalf makeNode(const PxVec3& bound, PxI32 child, bool leaf)
{
	PxgPackedNodeHalf n;
	n.x = bound.x;
	n.y = bound.y;
	n.z = bound.z;
	n.i = (PxU32)child;
	n.b = (PxU32)(leaf ? 1 : 0);

	return n;
}

// variation of makeNode through volatile pointers used in BuildHierarchy
PX_FORCE_INLINE __device__ void makeNode(volatile PxgPackedNodeHalf* n, const PxVec3& bound, PxI32 child, bool leaf)
{
	n->x = bound.x;
	n->y = bound.y;
	n->z = bound.z;
	n->i = (PxU32)child;
	n->b = (PxU32)(leaf ? 1 : 0);
}

PX_FORCE_INLINE __device__ PxgPackedNodeHalf makeNode(const PxVec3& bound, PxReal w)
{
	PxgPackedNodeHalf n;
	n.x = bound.x;
	n.y = bound.y;
	n.z = bound.z;

	reinterpret_cast<float4&>(n).w = w;

	return n;
}

// this bottom-up process assigns left and right children and combines bounds to form internal nodes
// there is one thread launched per-leaf node, each thread calculates it's parent node and assigns
// itself to either the left or right parent slot, the last child to complete the parent and moves
// up the hierarchy
template <typename Func>
PX_FORCE_INLINE __device__ void buildHierarchy(PxI32 n, PxI32* root, PxU32* maxTreeDepth, const PxReal* PX_RESTRICT deltas, PxI32* PX_RESTRICT numChildren,
	volatile PxI32* PX_RESTRICT rangeLefts, volatile PxI32* PX_RESTRICT rangeRights, volatile PxgPackedNodeHalf* PX_RESTRICT lowers, volatile PxgPackedNodeHalf* PX_RESTRICT uppers, Func& f)
{
	PxI32 index = blockDim.x*blockIdx.x + threadIdx.x;

	PxU32 maxDepth = 0;

	if (index < n)
	{
		const PxI32 internalOffset = n;

		for (;;)
		{
			PxI32 left = rangeLefts[index];
			PxI32 right = rangeRights[index];

			// check if we are the root node, if so then store out our index and terminate
			if (left == 0 && right == n - 1)
			{
				*root = index;
				*maxTreeDepth = maxDepth;
				break;
			}

			PxI32 childCount = 0;

			PxI32 parent;

			if (left == 0 || (right != n - 1 && deltas[right] < deltas[left - 1]))
			{
				parent = right + internalOffset;

				// set parent left child
				lowers[parent].i = index;
				rangeLefts[parent] = left;

				childCount = atomicAdd(&numChildren[parent], 1);
			}
			else
			{
				parent = left + internalOffset - 1;

				// set parent right child
				uppers[parent].i = index;
				rangeRights[parent] = right;

				childCount = atomicAdd(&numChildren[parent], 1);
			}

			// ensure above writes are visible to all threads
			__threadfence();

			// if we have are the last thread (such that the parent node is now complete)
			// then update its bounds and move onto the the next parent in the hierarchy
			if (childCount == 1)
			{
				++maxDepth;

				const PxI32 leftChild = lowers[parent].i;
				const PxI32 rightChild = uppers[parent].i;

				//TODO: float4 loads as in queries?
				volatile PxgPackedNodeHalf& lowerLeft = lowers[leftChild];
				PxVec3 leftLower = PxVec3(lowerLeft.x,
					lowerLeft.y,
					lowerLeft.z);

				PxVec3 leftUpper = PxVec3(uppers[leftChild].x,
					uppers[leftChild].y,
					uppers[leftChild].z);

				volatile PxgPackedNodeHalf& lowerRight = lowers[rightChild];
				PxVec3 rightLower = PxVec3(lowerRight.x,
					lowerRight.y,
					lowerRight.z);

				PxVec3 rightUpper = PxVec3(uppers[rightChild].x,
					uppers[rightChild].y,
					uppers[rightChild].z);

				// union of child bounds
				PxVec3 lower = leftLower.minimum(rightLower);
				PxVec3 upper = leftUpper.maximum(rightUpper);

				// write new BVH nodes
				makeNode(lowers + parent, lower, leftChild, false);
				makeNode(uppers + parent, upper, rightChild, false);

				//Allows to compute additional data per node
				f(parent, leftChild, lowerLeft, rightChild, lowerRight);

				// move onto processing the parent
				index = parent;
			}
			else
			{
				// parent not ready (we are the first child), terminate thread
				break;
			}
		}
	}
}

struct EmptyBuilder
{
	PX_FORCE_INLINE __device__ EmptyBuilder() {}
	PX_FORCE_INLINE __device__ void operator()(PxI32 parentId, PxI32 childLeftId, volatile PxgPackedNodeHalf& childLeft, PxI32 childRightId, volatile PxgPackedNodeHalf& childRight)
	{}
};

PX_FORCE_INLINE __device__ void buildHierarchy(PxI32 n, PxI32* root, PxU32* maxTreeDepth, const PxReal* PX_RESTRICT deltas, PxI32* PX_RESTRICT numChildren,
	volatile PxI32* PX_RESTRICT rangeLefts, volatile PxI32* PX_RESTRICT rangeRights, volatile PxgPackedNodeHalf* PX_RESTRICT lowers, volatile PxgPackedNodeHalf* PX_RESTRICT uppers)
{
	EmptyBuilder e;
	buildHierarchy(n, root, maxTreeDepth, deltas, numChildren, rangeLefts, rangeRights, lowers, uppers, e);
}



__device__ inline bool intersectRayAABBFast(const PxVec3& pos, const PxVec3& rcp_dir, const PxVec3& min, const PxVec3& max, PxReal& lmin, PxReal& lmax)
{	
	PxReal l1 = (min.x - pos.x) * rcp_dir.x;
	PxReal l2 = (max.x - pos.x) * rcp_dir.x;
	lmin = PxMin(l1, l2);
	lmax = PxMax(l1, l2);

	l1 = (min.y - pos.y) * rcp_dir.y;
	l2 = (max.y - pos.y) * rcp_dir.y;
	lmin = PxMax(PxMin(l1, l2), lmin);
	lmax = PxMin(PxMax(l1, l2), lmax);

	l1 = (min.z - pos.z) * rcp_dir.z;
	l2 = (max.z - pos.z) * rcp_dir.z;
	lmin = PxMax(PxMin(l1, l2), lmin);
	lmax = PxMin(PxMax(l1, l2), lmax);

	//return ((lmax > 0.f) & (lmax >= lmin));
	//return ((lmax > 0.f) & (lmax > lmin));
	bool hit = ((lmax >= 0.f) & (lmax >= lmin));
	/*if (hit)
		t = lmin;*/
	return hit;
}

PX_FORCE_INLINE __device__ PxU32 maxAbsDim(PxVec3 dir)
{
	dir.x = PxAbs(dir.x);
	dir.y = PxAbs(dir.y);
	dir.z = PxAbs(dir.z);
	if (dir.x >= dir.y && dir.x >= dir.z)
		return 0;
	if (dir.y >= dir.x && dir.y >= dir.z)
		return 1;
	return 2;
}

//Specialized implementation guaranteeing watertightness taken from paper "Watertight Ray/Triangle Intersection"
//https://jcgt.org/published/0002/01/05/paper.pdf
__device__ inline bool intersectRayTriTwoSidedWatertight(const PxVec3& org, const PxVec3& dir, const PxVec3& a,
	const PxVec3& b, const PxVec3& c, PxReal& t, PxReal& u, PxReal& v, PxReal& w)
{
	//Claculate the dimension where the ray direction is maximal
	PxU32 kz = maxAbsDim(dir);
	PxU32 kx = kz + 1; if (kx == 3) kx = 0;
	PxU32 ky = kx + 1; if (ky == 3) ky = 0;

	//Swap kx and ky dimension to preserve winding direction of triangles
	if (dir[kz] < 0.0f) 
		PxSwap(kx, ky);

	//Calculate shear constants
	PxReal Sx = dir[kx] / dir[kz];
	PxReal Sy = dir[ky] / dir[kz];
	PxReal Sz = 1.0f / dir[kz];


	//Calculate vertices relative to ray origin
	const PxVec3 A = a - org;
	const PxVec3 B = b - org;
	const PxVec3 C = c - org;

	//Perform shear and scale of vertices
	const PxReal Ax = A[kx] - Sx * A[kz];
	const PxReal Ay = A[ky] - Sy * A[kz];
	const PxReal Bx = B[kx] - Sx * B[kz];
	const PxReal By = B[ky] - Sy * B[kz];
	const PxReal Cx = C[kx] - Sx * C[kz];
	const PxReal Cy = C[ky] - Sy * C[kz];

	//Calculate scaled barycentric coordinates
	PxReal U = Cx * By - Cy * Bx;
	PxReal V = Ax * Cy - Ay * Cx;
	PxReal W = Bx * Ay - By * Ax;

	//Fallback to test against edges using double precision
	//Happens only in about 1 case out of 1mio tests according to the paper "Watertight Ray/Triangle Intersection"
	if (U == 0.0f || V == 0.0f || W == 0.0f) 
	{
		double CxBy = (double)Cx*(double)By;
		double CyBx = (double)Cy*(double)Bx;
		U = (PxReal)(CxBy - CyBx);
		double AxCy = (double)Ax*(double)Cy;
		double AyCx = (double)Ay*(double)Cx;
		V = (PxReal)(AxCy - AyCx);
		double BxAy = (double)Bx*(double)Ay;
		double ByAx = (double)By*(double)Ax;
		W = (PxReal)(BxAy - ByAx);
	}

	//Perform edge tests. Moving this test before and at the end of the previous conditional gives higher performance
	if ((U < 0.0f || V < 0.0f || W < 0.0f) &&
		(U > 0.0f || V > 0.0f || W > 0.0f)) 
		return false;

	//Calculate determinant
	PxReal det = U + V + W;
	if (det == 0.0f) 
		return false;

	//Calculate scaled z-coordinates of vertices and use them to calculate the hit distance
	const PxReal Az = Sz * A[kz];
	const PxReal Bz = Sz * B[kz];
	const PxReal Cz = Sz * C[kz];
	const PxReal T = U * Az + V * Bz + W * Cz;
	
	//Normalize U, V, W and T
	const PxReal rcpDet = 1.0f / det;
	u = U * rcpDet;
	v = V * rcpDet;
	w = W * rcpDet;
	t = T * rcpDet;

	return true;
}

PX_FORCE_INLINE PxReal __device__ windingNumberForTriangle(const PxVec3& triA, const PxVec3& triB, const PxVec3& triC, const PxVec3& queryPoint)
{
	PxVec3 a = triA - queryPoint;
	PxVec3 b = triB - queryPoint;
	PxVec3 c = triC - queryPoint;
		
	PxReal y = a.dot(b.cross(c));

	PxReal la = a.magnitude();
	PxReal lb = b.magnitude();
	PxReal lc = c.magnitude();

	PxReal x = (la * lb * lc + a.dot(b) * lc + b.dot(c) * la + c.dot(a) * lb);
	PxReal omega = PxAtan2(y, x);

	return (0.5f / PxPi) * omega;
}

PX_FORCE_INLINE PxReal __device__ firstOrderClusterApproximation(const PxVec3& weightedCentroid, const PxVec3& weightedNormalSum,
	const PxVec3& evaluationPoint)
{
	const PxVec3 dir = weightedCentroid - evaluationPoint;
	const PxReal l = dir.magnitude();
	return ((0.25f / PxPi) / (l * l * l)) * weightedNormalSum.dot(dir);
}

PX_FORCE_INLINE PxReal __device__ radiusOfSphereContainingSubSpheres(const PxVec3& newSphereCenter, const PxVec3& centerA, PxReal radiusA, const PxVec3& centerB, PxReal radiusB)
{
	return PxMax((centerA - newSphereCenter).magnitude() + radiusA, (centerB - newSphereCenter).magnitude() + radiusB);
}

PX_FORCE_INLINE PxVec3 __device__ triangleNormal(const PxVec3& triA, const PxVec3& triB, const PxVec3& triC)
{
	return (triB - triA).cross(triC - triA);
}

PX_FORCE_INLINE PxVec3 __device__ triangleCentroid(const PxVec3& triA, const PxVec3& triB, const PxVec3& triC)
{
	const PxReal third = 1.0f / 3.0f;
	return third * (triA + triB + triC);
}

PX_FORCE_INLINE __device__ PxgWindingClusterApproximation createWindingClusterApproximation(const PxVec3* PX_RESTRICT vertices, const PxU32* PX_RESTRICT triangle)
{
	const PxVec3& triA = vertices[triangle[0]];
	const PxVec3& triB = vertices[triangle[1]];
	const PxVec3& triC = vertices[triangle[2]];

	PxgWindingClusterApproximation result;
	result.mWeightedNormalSum = 0.5f * triangleNormal(triA, triB, triC);
	result.mAreaSum = result.mWeightedNormalSum.magnitude();
	result.mCentroidTimesArea = triangleCentroid(triA, triB, triC);
	result.mRadius = PxSqrt(PxMax(PxMax((triA - result.mCentroidTimesArea).magnitudeSquared(),
		(triB - result.mCentroidTimesArea).magnitudeSquared()), (triC - result.mCentroidTimesArea).magnitudeSquared()));
	result.mCentroidTimesArea = result.mAreaSum * result.mCentroidTimesArea;
	return result;
}

PX_FORCE_INLINE __device__ PxVec3 clusterCentroid(const PxgWindingClusterApproximation& c)
{
	return c.mCentroidTimesArea * (1.0f / c.mAreaSum);
}

PX_FORCE_INLINE __device__ void combineClusters(const PxgWindingClusterApproximation& a, const PxgWindingClusterApproximation& b, PxgWindingClusterApproximation& result)
{
	result.mWeightedNormalSum = a.mWeightedNormalSum + b.mWeightedNormalSum;
	result.mAreaSum = a.mAreaSum + b.mAreaSum;
	result.mCentroidTimesArea = a.mCentroidTimesArea + b.mCentroidTimesArea;
	result.mRadius = radiusOfSphereContainingSubSpheres(clusterCentroid(result), clusterCentroid(a), a.mRadius, clusterCentroid(b), b.mRadius); //This is a conservative approximation (meaning the radius might b a bit too big) but that's fine for the winding number algorithm
}

//Clusters are not stored for child nodes!
PX_FORCE_INLINE __device__ PxI32 getClusterIndex(PxI32 bvhNodeIndex, PxU32 numTriangles)
{
	PxI32 result = bvhNodeIndex - numTriangles; //The tree is built such that the leave nodes are at the beginning of the array
	assert(result >= 0);
	assert(result < numTriangles);
	/*if (result < 0 || result >= numTriangles)
		printf("Winding cluster out of range access\n");*/
	return result;
}

//Can be passed to the buildHierarchy method to build a winding number hierarchy simultaneously
struct WindingClusterBuilder
{
	PxgWindingClusterApproximation* PX_RESTRICT clusters;
	const PxVec3* PX_RESTRICT vertices;
	const PxU32* PX_RESTRICT indices;
	PxU32 numTriangles;

	PX_FORCE_INLINE __device__ WindingClusterBuilder(PxgWindingClusterApproximation* PX_RESTRICT clusters, const PxVec3* PX_RESTRICT vertices, const PxU32* PX_RESTRICT indices, PxU32 numTriangles)
		: clusters(clusters), vertices(vertices), indices(indices), numTriangles(numTriangles)
	{
	}

	PX_FORCE_INLINE __device__ void operator()(PxI32 parentId, PxI32 childLeftId, volatile PxgPackedNodeHalf& childLeft, PxI32 childRightId, volatile PxgPackedNodeHalf& childRight)
	{
		PxgWindingClusterApproximation approxLeft = childLeft.b ? createWindingClusterApproximation(vertices, &indices[3 * childLeft.i]) : clusters[getClusterIndex(childLeftId, numTriangles)];
		PxgWindingClusterApproximation approxRight = childRight.b ? createWindingClusterApproximation(vertices, &indices[3 * childRight.i]) : clusters[getClusterIndex(childRightId, numTriangles)];
		combineClusters(approxLeft, approxRight, clusters[getClusterIndex(parentId, numTriangles)]);
	}
};


struct WindingNumberTraversal
{
public:
	PxReal mWindingNumber = 0;
	const PxU32* PX_RESTRICT mTriangles;
	PxU32 mNumTriangles;
	const PxVec3* PX_RESTRICT mPoints;
	const PxgWindingClusterApproximation* PX_RESTRICT  mClusters;
	PxVec3 mQueryPoint;
	PxReal mDistanceThresholdBeta;

	__device__ WindingNumberTraversal()
	{
	}

	__device__ WindingNumberTraversal(const PxU32* PX_RESTRICT triangles, PxU32 numTriangles, const PxVec3* PX_RESTRICT points,
		const PxgWindingClusterApproximation* PX_RESTRICT clusters, const PxVec3& queryPoint, PxReal distanceThresholdBeta = 2.0f)
		: mTriangles(triangles), mNumTriangles(numTriangles), mPoints(points), mClusters(clusters), mQueryPoint(queryPoint), mDistanceThresholdBeta(distanceThresholdBeta)
	{
	}

	__device__ inline BvhTraversalControl::Enum operator()(const PxgPackedNodeHalf& lower, const PxgPackedNodeHalf& upper, PxI32 nodeIndex)
	{
		if (lower.b)
		{
			const PxU32* tri = &mTriangles[3 * lower.i];
			mWindingNumber += windingNumberForTriangle(mPoints[tri[0]], mPoints[tri[1]], mPoints[tri[2]], mQueryPoint);
			return BvhTraversalControl::eDontGoDeeper;
		}
		const PxgWindingClusterApproximation& cluster = mClusters[getClusterIndex(nodeIndex, mNumTriangles)];
		const PxReal distSquared = (mQueryPoint - clusterCentroid(cluster)).magnitudeSquared();
		const PxReal threshold = mDistanceThresholdBeta * cluster.mRadius;
		if (distSquared > threshold * threshold)
		{
			mWindingNumber += firstOrderClusterApproximation(clusterCentroid(cluster), cluster.mWeightedNormalSum, mQueryPoint);
			return BvhTraversalControl::eDontGoDeeper;
		}
		return BvhTraversalControl::eGoDeeper;
	}
};


PX_FORCE_INLINE __device__ PxReal rayTriangleSign(const PxVec3& dir, const PxVec3& a,
	const PxVec3& b, const PxVec3& c, bool normalize)
{
	PxVec3 ab = b - a;
	PxVec3 ac = c - a;
	PxVec3 n = ab.cross(ac);

	if (normalize)
	{
		PxReal mag2 = n.magnitudeSquared();
		if (mag2 > 0.0f)
			n = n * (1.0f / PxSqrt(mag2));
	}

	return -(dir.dot(n));
}

struct ClosestRayIntersectionTraversal
{
	const PxVec3* PX_RESTRICT meshVertices;
	const PxU32* PX_RESTRICT meshIndices;

	const PxVec3 origin;
	const PxVec3 dir;
	const PxVec3 rcpDir;

	PxReal closestT;
	PxReal closestDotProduct;
	bool includeNegativeRayDirection;
	bool closestPointOnTriangleEdge;


	__device__ inline ClosestRayIntersectionTraversal(const PxVec3* PX_RESTRICT meshVertices, const PxU32* PX_RESTRICT meshIndices, const PxVec3& start, const PxVec3& dir, bool includeNegativeRayDirection) :
		meshVertices(meshVertices), meshIndices(meshIndices),
		origin(start),
		dir(dir),
		rcpDir(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z),
		closestT(FLT_MAX),
		closestDotProduct(0.0f),
		includeNegativeRayDirection(includeNegativeRayDirection),
		closestPointOnTriangleEdge(false)
	{
	}

	PX_FORCE_INLINE __device__ bool hasHit()
	{
		return closestT < FLT_MAX;
	}

	__device__ inline BvhTraversalControl::Enum operator()(const PxgPackedNodeHalf& lower, const PxgPackedNodeHalf& upper, PxI32 nodeIndex)
	{
		PxReal t;
		if (lower.b)
		{
			// test each element of the rigid body mesh
			PxU32 tri = lower.i;
			PxVec3 a = meshVertices[meshIndices[tri * 3 + 0]];
			PxVec3 b = meshVertices[meshIndices[tri * 3 + 1]];
			PxVec3 c = meshVertices[meshIndices[tri * 3 + 2]];

			PxReal u, v, w, s;
			PxVec3 n;


			if (intersectRayTriTwoSidedWatertight(origin, dir, a, b, c, t, u, v, w))
			{
				s = rayTriangleSign(dir, a, b, c, true);
				if (includeNegativeRayDirection)
				{
					if (t < 0.0f)
					{
						t = -t;
						s = -s;
					}
				}
				if (t > 0.0f && t < closestT)
				{
					closestT = t;
					closestDotProduct = s;
					closestPointOnTriangleEdge = u == 0.0f || v == 0.0f || w == 0.0f;
				}
			}

			return BvhTraversalControl::eDontGoDeeper;
		}

		//TODO: Does intersectRayAABBFast work for negative t?
		PxReal tMax;
		if (intersectRayAABBFast(origin, rcpDir, PxVec3(lower.x, lower.y, lower.z), PxVec3(upper.x, upper.y, upper.z), t, tMax))
		{
			if (includeNegativeRayDirection)
			{
				if (tMax < 0.0f)
					t = -tMax;
			}
			if (t < closestT)
				return BvhTraversalControl::eGoDeeper;
		}
		return BvhTraversalControl::eDontGoDeeper;
	}
};

struct ClosestDistanceToTriangleMeshTraversal
{
public:
	const PxU32* PX_RESTRICT mTriangles;
	const PxVec3* PX_RESTRICT mPoints;
	PxVec3 mQueryPoint;
	PxReal mClosestDistanceSquared;

	__device__ inline ClosestDistanceToTriangleMeshTraversal()
	{
	}

	__device__ inline ClosestDistanceToTriangleMeshTraversal(const PxU32* PX_RESTRICT triangles, const PxVec3* PX_RESTRICT points, const PxVec3& queryPoint)
		: mTriangles(triangles), mPoints(points), mQueryPoint(queryPoint), mClosestDistanceSquared(100000000000.0f)
	{
	}

	PX_FORCE_INLINE __device__ PxReal distancePointBoxSquared(const PxVec3& minimum, const PxVec3& maximum, const PxVec3& point)
	{
		PxVec3 closestPt = minimum.maximum(maximum.minimum(point));
		return (closestPt - point).magnitudeSquared();
	}

	__device__ inline BvhTraversalControl::Enum operator()(const PxgPackedNodeHalf& lower, const PxgPackedNodeHalf& upper, PxI32 nodeIndex)
	{
		if (distancePointBoxSquared(PxVec3(lower.x, lower.y, lower.z), PxVec3(upper.x, upper.y, upper.z), mQueryPoint) >= mClosestDistanceSquared)
			return BvhTraversalControl::eDontGoDeeper;

		if (lower.b)
		{
			const PxU32* tri = &mTriangles[3 * lower.i];
			const PxVec3 a = mPoints[tri[0]];
			const PxVec3 b = mPoints[tri[1]];
			const PxVec3 c = mPoints[tri[2]];

			//PxReal s, t;
			PxVec3 closestPt = Gu::closestPtPointTriangle2UnitBox(mQueryPoint, a, b, c); // closestPtPointTriangle(mQueryPoint, a, b, c, s, t);
			PxReal distSq = (closestPt - mQueryPoint).magnitudeSquared();
			if (distSq < mClosestDistanceSquared)
			{
				mClosestDistanceSquared = distSq;
			}

			return BvhTraversalControl::eDontGoDeeper;
		}

		return BvhTraversalControl::eGoDeeper;
	}
};

//Evaluates the winding number and the closest distance in a single query. Might be faster in some scenarios than two separate queries.
struct WindingNumberAndDistanceTraversal
{
public:
	const PxU32* PX_RESTRICT mTriangles;
	PxU32 mNumTriangles;
	PxReal mWindingNumber;
	const PxVec3* PX_RESTRICT mPoints;
	const PxgWindingClusterApproximation* mClusters;
	PxVec3 mQueryPoint;
	PxReal mDistanceThresholdBeta;

	PxReal mClosestDistance;

	__device__ WindingNumberAndDistanceTraversal()
	{
	}

	__device__ WindingNumberAndDistanceTraversal(const PxU32* PX_RESTRICT triangles, PxU32 numTriangles, const PxVec3* PX_RESTRICT points,
		const PxgWindingClusterApproximation* clusters, const PxVec3& queryPoint, PxReal distanceThresholdBeta = 2.0f)
		: mTriangles(triangles), mNumTriangles(numTriangles), mWindingNumber(0), mPoints(points), mClusters(clusters), mQueryPoint(queryPoint), mDistanceThresholdBeta(distanceThresholdBeta),
		mClosestDistance(10000000)
	{
	}

	__device__ inline void evaluateLeaf(PxU32 payloadIndex)
	{
		const PxU32* tri = &mTriangles[3 * payloadIndex];
		const PxVec3 a = mPoints[tri[0]];
		const PxVec3 b = mPoints[tri[1]];
		const PxVec3 c = mPoints[tri[2]];
		mWindingNumber += windingNumberForTriangle(a, b, c, mQueryPoint);

		//PxReal s, t;
		PxVec3 closestPt = Gu::closestPtPointTriangle2UnitBox(mQueryPoint, a, b, c); //closestPtPointTriangle(mQueryPoint, a, b, c, s, t);
		PxReal distSq = (closestPt - mQueryPoint).magnitudeSquared();
		if (distSq < mClosestDistance * mClosestDistance)
		{
			mClosestDistance = PxSqrt(distSq);
		}
	}

	//Do not pass leave nodes into that function
	__device__ inline BvhTraversalControl::Enum evaluateBranchNode(const PxgPackedNodeHalf& lower, const PxgPackedNodeHalf& upper, PxI32 nodeIndex)
	{
		const PxgWindingClusterApproximation& cluster = mClusters[getClusterIndex(nodeIndex, mNumTriangles)];
		const PxReal dist = (mQueryPoint - clusterCentroid(cluster)).magnitude();
		if (dist - cluster.mRadius < mClosestDistance)
		{
			//Deeper traversal is required
			return BvhTraversalControl::eGoDeeper;
		}
		else if (dist > mDistanceThresholdBeta * cluster.mRadius)
		{
			mWindingNumber += firstOrderClusterApproximation(clusterCentroid(cluster), cluster.mWeightedNormalSum, mQueryPoint);
			return BvhTraversalControl::eDontGoDeeper;
		}
		return BvhTraversalControl::eGoDeeper;
	}

	__device__ inline BvhTraversalControl::Enum operator()(const PxgPackedNodeHalf& lower, const PxgPackedNodeHalf& upper, PxI32 nodeIndex)
	{
		if (lower.b)
		{
			evaluateLeaf(lower.i);
			return BvhTraversalControl::eDontGoDeeper;
		}
		return evaluateBranchNode(lower, upper, nodeIndex);
	}
};


#endif
