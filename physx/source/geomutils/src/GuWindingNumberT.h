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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef GU_WINDING_NUMBER_T_H
#define GU_WINDING_NUMBER_T_H


#include "GuTriangle.h"
#include "foundation/PxArray.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxVec3.h"
#include "GuBVH.h"
#include "GuAABBTreeQuery.h"
#include "GuAABBTreeNode.h"
#include "GuWindingNumberCluster.h"

namespace physx
{
namespace Gu
{
	using Triangle = Gu::IndexedTriangleT<PxI32>;
	
	template<typename R, typename V3>
	struct SecondOrderClusterApproximationT : public ClusterApproximationT<R, V3>
	{
		PxMat33 WeightedOuterProductSum;

		PX_FORCE_INLINE SecondOrderClusterApproximationT() {}

		PX_FORCE_INLINE SecondOrderClusterApproximationT(R radius, R areaSum, const V3& weightedCentroid, const V3& weightedNormalSum, const PxMat33& weightedOuterProductSum) :
			ClusterApproximationT<R, V3>(radius, areaSum, weightedCentroid, weightedNormalSum), WeightedOuterProductSum(weightedOuterProductSum)
		{ }
	};
	   
	//Evaluates a first order winding number approximation for a given cluster (cluster = bunch of triangles)
	template<typename R, typename V3>
	PX_FORCE_INLINE R firstOrderClusterApproximation(const V3& weightedCentroid, const V3& weightedNormalSum,
		const V3& evaluationPoint)
	{
		const V3 dir = weightedCentroid - evaluationPoint;
		const R l = dir.magnitude();
		return (R(0.25 / 3.141592653589793238462643383) / (l * l * l)) * weightedNormalSum.dot(dir);
	}
	template<typename R, typename V3>
	PX_FORCE_INLINE R clusterApproximation(const ClusterApproximationT<R, V3>& c, const V3& evaluationPoint)
	{
		return firstOrderClusterApproximation(c.WeightedCentroid, c.WeightedNormalSum, evaluationPoint);
	}

	//Evaluates a second order winding number approximation for a given cluster (cluster = bunch of triangles)
	template<typename R, typename V3>
	PX_FORCE_INLINE R secondOrderClusterApproximation(const V3& weightedCentroid, const V3& weightedNormalSum,
		const PxMat33& weightedOuterProductSum, const V3& evaluationPoint)
	{
		const V3 dir = weightedCentroid - evaluationPoint;
		const R l = dir.magnitude();
		const R l2 = l * l;
		const R scaling = R(0.25 / 3.141592653589793238462643383) / (l2 * l);
		const R firstOrder = scaling * weightedNormalSum.dot(dir);

		const R scaling2 = -R(3.0) * scaling / l2;
		const R m11 = scaling + scaling2 * dir.x * dir.x, m12 = scaling2 * dir.x * dir.y, m13 = scaling2 * dir.x * dir.z;
		const R m21 = scaling2 * dir.y * dir.x, m22 = scaling + scaling2 * dir.y * dir.y, m23 = scaling2 * dir.y * dir.z;
		const R m31 = scaling2 * dir.z * dir.x, m32 = scaling2 * dir.z * dir.y, m33 = scaling + scaling2 * dir.z * dir.z;

		return firstOrder + (weightedOuterProductSum.column0.x * m11 + weightedOuterProductSum.column1.x * m12 + weightedOuterProductSum.column2.x * m13 +
			weightedOuterProductSum.column0.y * m21 + weightedOuterProductSum.column1.y * m22 + weightedOuterProductSum.column2.y * m23 +
			weightedOuterProductSum.column0.z * m31 + weightedOuterProductSum.column1.z * m32 + weightedOuterProductSum.column2.z * m33);
	}
	template<typename R, typename V3>
	PX_FORCE_INLINE R clusterApproximation(const SecondOrderClusterApproximationT<R, V3>& c, const V3& evaluationPoint)
	{
		return secondOrderClusterApproximation(c.WeightedCentroid, c.WeightedNormalSum, c.WeightedOuterProductSum, evaluationPoint);
	}

	//Computes parameters to approximately represent a cluster (cluster = bunch of triangles) to be used to compute a winding number approximation
	template<typename R, typename V3>
	void approximateCluster(const PxArray<PxI32>& triangleSet, PxU32 start, PxU32 end, const PxU32* triangles, const V3* points,
		const PxArray<R>& triangleAreas, const PxArray<V3>& triangleNormalsTimesTriangleArea, const PxArray<V3>& triangleCentroids, ClusterApproximationT<R, V3>& cluster)
	{
		V3 weightedCentroid(0., 0., 0.);
		R areaSum = 0;
		V3 weightedNormalSum(0., 0., 0.);

		for (PxU32 i = start; i < end; ++i)
		{
			PxI32 triId = triangleSet[i];
			areaSum += triangleAreas[triId];
			weightedCentroid += triangleCentroids[triId] * triangleAreas[triId];
			weightedNormalSum += triangleNormalsTimesTriangleArea[triId];
		}
		weightedCentroid = weightedCentroid / areaSum;

		R radiusSquared = 0;
		for (PxU32 i = start; i < end; ++i)
		{
			PxI32 triId = triangleSet[i];
			const PxU32* tri = &triangles[3 * triId];
			R d2 = (weightedCentroid - points[tri[0]]).magnitudeSquared();
			if (d2 > radiusSquared) radiusSquared = d2;
			d2 = (weightedCentroid - points[tri[1]]).magnitudeSquared();
			if (d2 > radiusSquared) radiusSquared = d2;
			d2 = (weightedCentroid - points[tri[2]]).magnitudeSquared();
			if (d2 > radiusSquared) radiusSquared = d2;
		}
		cluster = ClusterApproximationT<R, V3>(PxSqrt(radiusSquared), areaSum, weightedCentroid, weightedNormalSum/*, weightedOuterProductSum*/);
	}

	//Computes parameters to approximately represent a cluster (cluster = bunch of triangles) to be used to compute a winding number approximation
	template<typename R, typename V3>
	void approximateCluster(const PxArray<PxI32>& triangleSet, PxU32 start, PxU32 end, const PxU32* triangles, const V3* points,
		const PxArray<R>& triangleAreas, const PxArray<V3>& triangleNormalsTimesTriangleArea, const PxArray<V3>& triangleCentroids, SecondOrderClusterApproximationT<R, V3>& cluster)
	{
		V3 weightedCentroid(0., 0., 0.);
		R areaSum = 0;
		V3 weightedNormalSum(0., 0., 0.);

		for (PxU32 i = start; i < end; ++i)
		{
			PxI32 triId = triangleSet[i];
			areaSum += triangleAreas[triId];
			weightedCentroid += triangleCentroids[triId] * triangleAreas[triId];
			weightedNormalSum += triangleNormalsTimesTriangleArea[triId];
		}
		weightedCentroid = weightedCentroid / areaSum;

		R radiusSquared = 0;
		PxMat33 weightedOuterProductSum(PxZERO::PxZero);
		for (PxU32 i = start; i < end; ++i)
		{
			PxI32 triId = triangleSet[i];
			const PxU32* tri = &triangles[3 * triId];
			R d2 = (weightedCentroid - points[tri[0]]).magnitudeSquared();
			if (d2 > radiusSquared) radiusSquared = d2;
			d2 = (weightedCentroid - points[tri[1]]).magnitudeSquared();
			if (d2 > radiusSquared) radiusSquared = d2;
			d2 = (weightedCentroid - points[tri[2]]).magnitudeSquared();
			if (d2 > radiusSquared) radiusSquared = d2;

			weightedOuterProductSum = weightedOuterProductSum + PxMat33::outer(triangleCentroids[triId] - weightedCentroid, triangleNormalsTimesTriangleArea[triId]);
		}
		cluster = SecondOrderClusterApproximationT<R, V3>(PxSqrt(radiusSquared), areaSum, weightedCentroid, weightedNormalSum, weightedOuterProductSum);
	}

	//Exact winding number evaluation, needs to be called for every triangle close to the winding number query point
	template<typename R, typename V3>
	PX_FORCE_INLINE R evaluateExact(V3 a, V3 b, V3 c, const V3& p)
	{
		const R twoOver4PI = R(0.5 / 3.141592653589793238462643383);

		a -= p;
		b -= p;
		c -= p;

		const R la = a.magnitude(),
			lb = b.magnitude(),
			lc = c.magnitude();

		const R y = a.x * b.y * c.z - a.x * b.z * c.y - a.y * b.x * c.z + a.y * b.z * c.x + a.z * b.x * c.y - a.z * b.y * c.x;
		const R x = (la * lb * lc + (a.x * b.x + a.y * b.y + a.z * b.z) * lc +
			(b.x * c.x + b.y * c.y + b.z * c.z) * la + (c.x * a.x + c.y * a.y + c.z * a.z) * lb);
		return twoOver4PI * PxAtan2(y, x);
	}

	struct Section
	{
		PxI32 start;
		PxI32 end;

		Section(PxI32 s, PxI32 e) : start(s), end(e)
		{}
	};

	//Helper method that recursively traverses the given BVH tree and computes a cluster approximation for every node and links it to the node
	template<typename R, typename V3>
	void precomputeClusterInformation(PxI32 nodeId, const BVHNode* tree, const PxU32* triangles, const PxU32 numTriangles,
		const V3* points, PxHashMap<PxU32, ClusterApproximationT<R, V3>>& infos, const PxArray<R> triangleAreas,
		const PxArray<V3>& triangleNormalsTimesTriangleArea, const PxArray<V3>& triangleCentroids)
	{
		PxArray<PxI32> stack;
		stack.pushBack(nodeId);
		PxArray<Section> returnStack;

		PxArray<PxI32> triIndices;
		triIndices.reserve(numTriangles);
		infos.reserve(PxU32(1.2f*numTriangles));

		while (stack.size() > 0)
		{
			nodeId = stack.popBack();

			if (nodeId >= 0)
			{
				const BVHNode& node = tree[nodeId];
				if (node.isLeaf())
				{
					triIndices.pushBack(node.getPrimitiveIndex());
					returnStack.pushBack(Section(triIndices.size() - 1, triIndices.size()));
					continue;
				}

				stack.pushBack(-nodeId - 1); //Marker for return index
				stack.pushBack(node.getPosIndex());
				stack.pushBack(node.getPosIndex() + 1);
			}
			else
			{
				Section trianglesA = returnStack.popBack();
				Section trianglesB = returnStack.popBack();
				Section sum(trianglesB.start, trianglesA.end);

				nodeId = -nodeId - 1;
				ClusterApproximationT<R, V3> c;
				approximateCluster<R, V3>(triIndices, sum.start, sum.end, triangles, points, triangleAreas, triangleNormalsTimesTriangleArea, triangleCentroids, c);
				infos.insert(PxU32(nodeId), c);

				returnStack.pushBack(sum);
			}
		}
	}

	//Precomputes a cluster approximation for every node in the BVH tree
	template<typename R, typename V3>
	void precomputeClusterInformation(const BVHNode* tree, const PxU32* triangles, const PxU32 numTriangles,
		const V3* points, PxHashMap<PxU32, ClusterApproximationT<R, V3>>& result, PxI32 rootNodeIndex)
	{
		PxArray<R> triangleAreas;
		triangleAreas.resize(numTriangles);
		PxArray<V3> triangleNormalsTimesTriangleArea;
		triangleNormalsTimesTriangleArea.resize(numTriangles);
		PxArray<V3> triangleCentroids;
		triangleCentroids.resize(numTriangles);

		for (PxU32 i = 0; i < numTriangles; ++i)
		{
			const PxU32* tri = &triangles[3 * i];
			const V3& a = points[tri[0]];
			const V3& b = points[tri[1]];
			const V3& c = points[tri[2]];
			triangleNormalsTimesTriangleArea[i] = (b - a).cross(c - a) * R(0.5);
			triangleAreas[i] = triangleNormalsTimesTriangleArea[i].magnitude();
			triangleCentroids[i] = (a + b + c) * R(1.0 / 3.0);
		}

		result.clear();
		precomputeClusterInformation(rootNodeIndex, tree, triangles, numTriangles, points, result, triangleAreas, triangleNormalsTimesTriangleArea, triangleCentroids);
	}

	template<typename R, typename V3>
	class WindingNumberTraversalController
	{
	public:
		R mWindingNumber = 0;
	private:
		const PxU32* mTriangles;
		const V3* mPoints;
		const PxHashMap<PxU32, ClusterApproximationT<R, V3>>& mClusters;
		V3 mQueryPoint;
		R mDistanceThresholdBeta;

	public:
		PX_FORCE_INLINE WindingNumberTraversalController(const PxU32* triangles, const V3* points,
			const PxHashMap<PxU32, ClusterApproximationT<R, V3>>& clusters, const V3& queryPoint, R distanceThresholdBeta = 2)
			: mTriangles(triangles), mPoints(points), mClusters(clusters), mQueryPoint(queryPoint), mDistanceThresholdBeta(distanceThresholdBeta)
		{ }

		PX_FORCE_INLINE Gu::TraversalControl::Enum analyze(const BVHNode& node, PxI32 nodeIndex)
		{
			if (node.isLeaf())
			{
				PX_ASSERT(node.getNbPrimitives() == 1);
				const PxU32* tri = &mTriangles[3 * node.getPrimitiveIndex()];
				mWindingNumber += evaluateExact<R, V3>(mPoints[tri[0]], mPoints[tri[1]], mPoints[tri[2]], mQueryPoint);
				return Gu::TraversalControl::eDontGoDeeper;
			}
			const ClusterApproximationT<R, V3>& cluster = mClusters.find(nodeIndex)->second;
			const R distSquared = (mQueryPoint - cluster.WeightedCentroid).magnitudeSquared();
			const R threshold = mDistanceThresholdBeta * cluster.Radius;
			if (distSquared > threshold * threshold)
			{
				//mWindingNumber += secondOrderClusterApproximation(cluster.WeightedCentroid, cluster.WeightedNormalSum, cluster.WeightedOuterProductSum, mQueryPoint);
				mWindingNumber += firstOrderClusterApproximation<R, V3>(cluster.WeightedCentroid, cluster.WeightedNormalSum, mQueryPoint); // secondOrderClusterApproximation(cluster.WeightedCentroid, cluster.WeightedNormalSum, cluster.WeightedOuterProductSum, mQueryPoint);
				return Gu::TraversalControl::eDontGoDeeper;
			}
			return Gu::TraversalControl::eGoDeeper;
		}

	private:
		PX_NOCOPY(WindingNumberTraversalController)
	};

	template<typename R, typename V3>
	R computeWindingNumber(const BVHNode* tree, const V3& q, R beta, const PxHashMap<PxU32, ClusterApproximationT<R, V3>>& clusters,
		const PxU32* triangles, const V3* points)
	{
		WindingNumberTraversalController<R, V3> c(triangles, points, clusters, q, beta);
		traverseBVH<WindingNumberTraversalController<R, V3>>(tree, c);
		return c.mWindingNumber;
	}
}
}

#endif
