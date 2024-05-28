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

#include "extensions/PxSamplingExt.h"
#include "GuSDF.h"
#include "foundation/PxQuat.h"
#include "foundation/PxHashSet.h"
#include "GuDistancePointTriangle.h"
#include "extensions/PxRemeshingExt.h"
#include "geometry/PxGeometryQuery.h"
#include "PxQueryReport.h"
#include "foundation/PxHashMap.h"
#include "CmRandom.h"

#include "GuAABBTreeNode.h"
#include "GuAABBTree.h"
#include "GuAABBTreeBounds.h"
#include "GuWindingNumber.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxSort.h"

namespace physx
{
	namespace
	{
		using namespace Gu;
		using namespace Cm;	

		static const PxI32 neighborEdges[3][2] = { { 0, 1 }, { 2, 0 }, { 1, 2 } };
		
		struct Int3
		{
			PxI32 x;
			PxI32 y;
			PxI32 z;

			Int3(PxI32 x_, PxI32 y_, PxI32 z_) : x(x_), y(y_), z(z_) {}

			Int3() : x(0), y(0), z(0) {}
		};

		struct ActiveSample
		{
			PxI32 mIndex;
			PxArray<PxI32> mNearbyTriangles;
			PxArray<PxReal> mCumulativeTriangleAreas;

			ActiveSample() : mIndex(-1) {}

			ActiveSample(PxI32 index, const PxArray<PxI32>& nearbyTriangles, const PxArray<PxReal>& cumulativeTriangleAreas)
			{
				mIndex = index;
				mNearbyTriangles = nearbyTriangles;
				mCumulativeTriangleAreas = cumulativeTriangleAreas;
			}
		};

		struct PointWithNormal
		{
			PxVec3 mPoint;
			PxVec3 mNormal;

			PointWithNormal() {}

			PointWithNormal(const PxVec3& point, const PxVec3& normal) : mPoint(point), mNormal(normal)
			{
			}
		};

		struct IndexWithNormal
		{
			PxI32 mIndex;
			PxVec3 mNormal;

			IndexWithNormal(PxI32 index, const PxVec3& normal) : mIndex(index), mNormal(normal)
			{
			}
		};

		void getBoundsFromPoints(const PxVec3* points, const PxU32 numPoints, PxVec3& outMinExtents, PxVec3& outMaxExtents)
		{
			PxVec3 minExtents(FLT_MAX);
			PxVec3 maxExtents(-FLT_MAX);

			// calculate face bounds
			for (PxU32 i = 0; i < numPoints; ++i)
			{
				const PxVec3& a = points[i];

				minExtents = a.minimum(minExtents);
				maxExtents = a.maximum(maxExtents);
			}

			outMinExtents = minExtents;
			outMaxExtents = maxExtents;
		}

		PX_FORCE_INLINE PxReal triArea(const PxVec3& a, const PxVec3& b, const PxVec3& c)
		{
			return 0.5f * (b - a).cross(c - a).magnitude();
		}

		PX_FORCE_INLINE PxReal triArea(const PxU32* tri, const PxVec3* points)
		{
			return triArea(points[tri[0]], points[tri[1]], points[tri[2]]);
		}

		PX_FORCE_INLINE PxU64 edgeKey(PxI32 a, PxI32 b)
		{
			if (a < b)
				return ((PxU64(a)) << 32) | (PxU64(b));
			else
				return ((PxU64(b)) << 32) | (PxU64(a));
		}

		void buildTriangleAdjacency(const PxU32* tris, PxU32 numTriangles, PxArray<PxI32>& result)
		{
			PxU32 l = 4 * numTriangles; //4 elements per triangle - waste one entry per triangle to get a power of 2 which allows for bit shift usage instead of modulo	
			result.clear();
			result.resize(l, -1);

			for (PxU32 i = 3; i < l; i += 4)
				result[i] = -2;

			PxHashMap<PxU64, PxU32> edges;
			for (PxU32 i = 0; i < numTriangles; ++i)
			{
				const PxU32* tri = &tris[3 * i];

				for (PxU32 j = 0; j < 3; ++j)
				{
					const PxU64 edge = edgeKey(tri[neighborEdges[j][0]], tri[neighborEdges[j][1]]);
					const PxPair<const PxU64, PxU32>* it = edges.find(edge);
					if (it)
					{
						result[4u * i + j] = it->second;
						result[it->second] = 4u * i + j;
					}
					else
					{
						PxU32 v = 4u * i + j;
						edges.insert(edge, v);
					}
				}
			}
		}

		

		void collectTrianglesInSphere(const PxVec3& center, PxReal radius, PxI32 startTri, const PxU32* triangles, const PxVec3* points,
			const PxArray<PxI32>& adj, PxHashSet<PxI32>& result)
		{
			PxArray<PxI32> stack;
			stack.pushBack(startTri);

			result.clear();
			result.insert(startTri);

			while (stack.size() > 0)
			{
				PxI32 tri = stack.popBack() * 4;
				for (PxI32 i = 0; i < 3; ++i)
				{
					PxI32 n = adj[tri + i] >> 2;
					if (n >= 0 && !result.contains(n))
					{
						const PxU32* t = &triangles[3 * n];
						if (Gu::distancePointTriangleSquared(center, points[t[0]], points[t[1]] - points[t[0]], points[t[2]] - points[t[0]]) < radius * radius)
						{
							result.insert(n);
							stack.pushBack(n);
						}
					}
				}
			}
		}

		void createActiveSample(const PxArray<PxReal>& triangleAreaBuffer, PxI32 sampleIndex, const PxVec3& sample, PxReal radius, PxI32 startTri,
			const PxU32* triangles, const PxVec3* points, const PxArray<PxI32>& adj, ActiveSample& result)
		{
			PxHashSet<PxI32> nearbyTriangles;
			collectTrianglesInSphere(sample, radius, startTri, triangles, points, adj, nearbyTriangles);

			result.mNearbyTriangles.clear();
			result.mNearbyTriangles.reserve(nearbyTriangles.size());
			//for (PxI32 t : nearbyTriangles)
			for (PxHashSet<PxI32>::Iterator iter = nearbyTriangles.getIterator(); !iter.done(); ++iter)
				result.mNearbyTriangles.pushBack(*iter);

			result.mCumulativeTriangleAreas.clear();
			result.mCumulativeTriangleAreas.resize(nearbyTriangles.size());
			result.mCumulativeTriangleAreas[0] = triangleAreaBuffer[result.mNearbyTriangles[0]];
			for (PxU32 i = 1; i < nearbyTriangles.size(); ++i)
				result.mCumulativeTriangleAreas[i] = result.mCumulativeTriangleAreas[i - 1] + triangleAreaBuffer[result.mNearbyTriangles[i]];

			result.mIndex = sampleIndex;
		}

		//Returns the index of the element with value <= v
		PxU32 binarySearch(const PxArray<PxReal>& sorted, PxReal v)
		{
			PxU32 low = 0;
			PxU32 up = PxU32(sorted.size());

			while (up - low > 1)
			{
				PxU32 middle = (up + low) >> 1;
				PxReal m = sorted[middle];
				if (v <= m)
					up = middle;
				else
					low = middle;
			}
			return low;
		}

		PxVec3 randomPointOnTriangle(BasicRandom& rnd, const PxU32* tri, const PxVec3* points, PxVec3* barycentricCoordinates = NULL)
		{
			while (true)
			{
				PxReal a = rnd.rand(0.0f, 1.0f);
				PxReal b = rnd.rand(0.0f, 1.0f);

				PxReal sum = a + b;
				if (sum > 1)
					continue;

				PxReal c = 1 - a - b;
				if (barycentricCoordinates)
					(*barycentricCoordinates) = PxVec3(a, b, c);
				return points[tri[0]] * a + points[tri[1]] * b + points[tri[2]] * c;
			}
		}

		bool samplePointInBallOnSurface(BasicRandom& rnd, const PxArray<PxReal>& cumulativeAreas, const PxVec3* points, const PxU32* triangles, const PxArray<PxI32>& nearbyTriangles,
			const PxVec3& point, PxReal radius, PxVec3& sample, PxI32& triId, PxI32 numAttempts = 30, PxVec3* barycentricCoordinates = NULL)
		{
			triId = -1;

			//Use variable upper bound as described in http://extremelearning.com.au/an-improved-version-of-bridsons-algorithm-n-for-poisson-disc-sampling/
			PxReal step = radius / numAttempts;
			PxReal rUpper = radius + step;
			PxVec3 fallback;
			PxReal fallbackDist = FLT_MAX;
			PxI32 fallbackId = -1;
			PxVec3 fallbackBary;
			for (PxI32 i = 0; i < numAttempts; ++i)
			{
				PxReal totalArea = cumulativeAreas[cumulativeAreas.size() - 1];
				PxReal r = rnd.rand(0.0f, 1.0f) * totalArea;
				PxI32 id;
				id = binarySearch(cumulativeAreas, r);

				triId = nearbyTriangles[id];
				sample = randomPointOnTriangle(rnd, &triangles[3 * triId], points, barycentricCoordinates);

				const PxReal dist2 = (sample - point).magnitudeSquared();
				if (dist2 > radius * radius && dist2 < rUpper * rUpper)
					return true;
				if (dist2 > radius * radius && dist2 < 4 * radius * radius && dist2 < fallbackDist)
				{
					fallbackDist = dist2;
					fallbackId = triId;
					fallback = sample;
					if (barycentricCoordinates)
						fallbackBary = *barycentricCoordinates;
				}
				rUpper += step;
			}
			if (fallbackId >= 0)
			{
				sample = fallback;
				triId = fallbackId;
				if (barycentricCoordinates)
					*barycentricCoordinates = fallbackBary;
				return true;
			}

			return false;
		}
	}

	class PoissonSamplerShared 
	{
	private:
		struct SparseGridNode
		{
			PxI32 mPointIndex;
			PxI32 mExcessStartIndex;
			PxI32 mExcessEndIndex;

			SparseGridNode(PxI32 pointIndex_) : mPointIndex(pointIndex_), mExcessStartIndex(0), mExcessEndIndex(-1)
			{
			}

			SparseGridNode() : mPointIndex(-1), mExcessStartIndex(0), mExcessEndIndex(-1)
			{ }
		};

		//Returns true if successful. False if too many cells are required (overflow)
		bool rebuildSparseGrid();

	public:
	    PoissonSamplerShared() : maxNumSamples(0), currentSamplingRadius(0.0f) {}

		//Returns true if successful. False if too many cells are required (overflow)
		bool setSamplingRadius(PxReal r);

		void addSamples(const PxArray<PxVec3>& samples);

		PxU32 removeSamples(const PxArray<PxVec3>& samples);

		PxI32 findSample(const PxVec3& p);

		PxReal minDistanceToOtherSamplesSquared(const PxVec3& p) const;

		const PxArray<PxVec3>& getSamples() const { return result; }

		bool addPointToSparseGrid(const PxVec3& p, PxI32 pointIndex);

	protected:
		bool postAddPointToSparseGrid(const PxVec3& p, PxI32 pointIndex);
		bool preAddPointToSparseGrid(const PxVec3& p, PxI32 pointIndex);		

	public:
		//Input
		PxI32 numSampleAttemptsAroundPoint;
		PxVec3 size;
		PxVec3 min;
		PxU32 maxNumSamples;

		//Intermediate data
		PxReal currentSamplingRadius;
		Int3 resolution;
		PxReal cellSize;
		PxArray<PxU32> occupiedCellBits;
		PxHashMap<PxI32, SparseGridNode> sparseGrid3D;
		PxArray<PxI32> excessList;
		Cm::BasicRandom rnd;

		bool gridResolutionValid = false;

		//Output
		PxArray<PxVec3> result;

		PX_NOCOPY(PoissonSamplerShared)
	};

	struct PointInVolumeTester
	{
		virtual bool pointInVolume(const PxVec3& p) const = 0;

		virtual ~PointInVolumeTester() {}
	};

	PX_FORCE_INLINE bool pointInSphere(const PxVec3& p, const PxVec3& sphereCenter, PxReal sphereRadius)
	{
		return (p - sphereCenter).magnitudeSquared() < sphereRadius * sphereRadius;
	}

	struct AlwaysInsideTester : public PointInVolumeTester
	{
		AlwaysInsideTester() {}

		virtual bool pointInVolume(const PxVec3&) const
		{
			return true;
		}

		virtual ~AlwaysInsideTester() {}
	};

	struct PointInSphereTester : public PointInVolumeTester
	{
		PxVec3 mCenter;
		PxReal mRadius;

		PointInSphereTester(const PxVec3& center, const PxReal radius) : mCenter(center), mRadius(radius) {}

		virtual bool pointInVolume(const PxVec3& p) const
		{
			return pointInSphere(p, mCenter, mRadius);
		}

		virtual ~PointInSphereTester() {}
	};

	struct PointInOBBTester : public PointInVolumeTester
	{
		PxVec3 mBoxCenter;
		PxVec3 mBoxAxisAlignedExtents;
		PxQuat mBoxOrientation;

		PointInOBBTester(const PxVec3& boxCenter, const PxVec3& boxAxisAlignedExtents, const PxQuat boxOrientation) 
			: mBoxCenter(boxCenter), mBoxAxisAlignedExtents(boxAxisAlignedExtents), mBoxOrientation(boxOrientation) {}

		virtual bool pointInVolume(const PxVec3& p) const
		{
			PxVec3 localPoint = mBoxOrientation.rotateInv(p - mBoxCenter);
			return localPoint.x >= -mBoxAxisAlignedExtents.x && localPoint.x <= mBoxAxisAlignedExtents.x &&
				localPoint.y >= -mBoxAxisAlignedExtents.y && localPoint.y <= mBoxAxisAlignedExtents.y &&
				localPoint.z >= -mBoxAxisAlignedExtents.z && localPoint.z <= mBoxAxisAlignedExtents.z;
		}

		virtual ~PointInOBBTester() {}
	};

	class TriangleMeshPoissonSampler : public PxTriangleMeshPoissonSampler
	{
	public:
		TriangleMeshPoissonSampler(const PxU32* tris, PxU32 numTris, const PxVec3* pts_, PxU32 numPts, PxReal r, PxI32 numSampleAttemptsAroundPoint_ = 30, PxU32 maxNumSamples_ = 0);

		virtual void addSamplesInVolume(const PointInVolumeTester& pointInVolume, const PxVec3& sphereCenter, PxReal sphereRadius, bool createVolumeSamples);

		virtual void addSamplesInSphere(const PxVec3& sphereCenter, PxReal sphereRadius, bool createVolumeSamples);

		virtual void addSamplesInBox(const PxBounds3& axisAlignedBox, const PxQuat& boxOrientation, bool createVolumeSamples);

		virtual const PxArray<PxI32>& getSampleTriangleIds() const { return triangleIds; }

		virtual const PxArray<PxVec3>& getSampleBarycentrics() const { return barycentricCoordinates; }

		virtual bool setSamplingRadius(PxReal samplingRadius) { return poissonSamplerShared.setSamplingRadius(samplingRadius); }

		virtual void addSamples(const PxArray<PxVec3>& samples) { poissonSamplerShared.addSamples(samples); }

		void createVolumeSamples(const PointInVolumeTester& pointInVolume, const PxVec3& sphereCenter, PxReal sphereRadius, PxReal randomScale, PxReal r, bool addToSparseGrid = true);

		virtual PxU32 removeSamples(const PxArray<PxVec3>& samples) { return poissonSamplerShared.removeSamples(samples); }

		virtual const PxArray<PxVec3>& getSamples() const { return poissonSamplerShared.result; }

		virtual bool isPointInTriangleMesh(const PxVec3& p);

		virtual ~TriangleMeshPoissonSampler() { }

	public:
		bool pointInMesh(const PxVec3& p);

		PoissonSamplerShared poissonSamplerShared;

		//Input
		const PxVec3* originalPoints;
		const PxU32* originalTriangles;
		const PxU32 numOriginalTriangles;

		PxVec3 max;

		PxArray<PxVec3> points;
		PxArray<PxU32> triangles;
		PxArray<PxU32> triangleMap;
		PxArray<PxReal> triangleAreaBuffer;
		PxArray<PxI32> adj;

		PxArray<Gu::BVHNode> tree;
		PxHashMap<PxU32, Gu::ClusterApproximation> clusters;

		//Intermediate data
		PxArray<ActiveSample> activeSamples;

		//Output
		PxArray<PxI32> triangleIds;
		PxArray<PxVec3> barycentricCoordinates;

		PX_NOCOPY(TriangleMeshPoissonSampler)
	};

	class ShapePoissonSampler : public PxPoissonSampler
	{
	public:
		ShapePoissonSampler(const PxGeometry& geometry_, const PxTransform& transform_, const PxBounds3& worldBounds_, PxReal r, PxI32 numSampleAttemptsAroundPoint_ = 30, PxU32 maxNumSamples_ = 0);
		
		virtual void addSamplesInVolume(const PointInVolumeTester& pointInVolume, const PxVec3& sphereCenter, PxReal sphereRadius, bool createVolumeSamples);

		virtual void addSamplesInSphere(const PxVec3& sphereCenter, PxReal sphereRadius, bool createVolumeSamples);
		
		virtual void addSamplesInBox(const PxBounds3& axisAlignedBox, const PxQuat& boxOrientation, bool createVolumeSamples);

		virtual bool setSamplingRadius(PxReal samplingRadius) { return poissonSamplerShared.setSamplingRadius(samplingRadius); }

		virtual void addSamples(const PxArray<PxVec3>& samples) { poissonSamplerShared.addSamples(samples); }

		void createVolumeSamples(const PointInVolumeTester& pointInVolume, const PxVec3& sphereCenter, PxReal sphereRadius, PxReal randomScale, PxReal r, bool addToSparseGrid = true);

		virtual PxU32 removeSamples(const PxArray<PxVec3>& samples) { return poissonSamplerShared.removeSamples(samples); }

		virtual const PxArray<PxVec3>& getSamples() const { return poissonSamplerShared.result; }

		virtual ~ShapePoissonSampler() { }

	public:
		PoissonSamplerShared poissonSamplerShared;

		//Input
		const PxGeometry& shape;
		const PxTransform actorGlobalPose;

		//Intermediate data
		PxArray<IndexWithNormal> activeSamples;
	};

	bool TriangleMeshPoissonSampler::pointInMesh(const PxVec3& p)
	{
		return Gu::computeWindingNumber(tree.begin(), p, clusters, originalTriangles, originalPoints) > 0.5f;
	}


	PxPoissonSampler* PxCreateShapeSampler(const PxGeometry& geometry, const PxTransform& transform, const PxBounds3& worldBounds, PxReal r, PxI32 numSampleAttemptsAroundPoint)
	{
		return PX_NEW(ShapePoissonSampler)(geometry, transform, worldBounds, r, numSampleAttemptsAroundPoint);
	}

	PxTriangleMeshPoissonSampler* PxCreateTriangleMeshSampler(const PxU32* tris, PxU32 numTris, const PxVec3* pts, PxU32 numPts, PxReal r, PxI32 numSampleAttemptsAroundPoint)
	{
		return PX_NEW(TriangleMeshPoissonSampler)(tris, numTris, pts, numPts, r, numSampleAttemptsAroundPoint);
	}

	PxVec3 computeBarycentricCoordinates(PxVec3 p, const PxVec3& a, PxVec3 b, PxVec3 c)
	{
		PxVec4 bary;
		computeBarycentric(a, b, c, p, bary);
		return PxVec3(bary.x, bary.y, bary.z);
	}

	
	ShapePoissonSampler::ShapePoissonSampler(const PxGeometry& shape_, const PxTransform& actorGlobalPose_, const PxBounds3& worldBounds_, 
		PxReal r, PxI32 numSampleAttemptsAroundPoint_, PxU32 maxNumSamples_) : shape(shape_), actorGlobalPose(actorGlobalPose_)
	{
		poissonSamplerShared.size = worldBounds_.maximum - worldBounds_.minimum;
		poissonSamplerShared.min = worldBounds_.minimum;
		poissonSamplerShared.numSampleAttemptsAroundPoint = numSampleAttemptsAroundPoint_;
		poissonSamplerShared.maxNumSamples = maxNumSamples_;

		setSamplingRadius(r);
	}

	TriangleMeshPoissonSampler::TriangleMeshPoissonSampler(const PxU32* tris, PxU32 numTris, const PxVec3* pts_, PxU32 numPts, PxReal r, PxI32 numSampleAttemptsAroundPoint_, PxU32 maxNumSamples_)
		: originalPoints(pts_), originalTriangles(tris), numOriginalTriangles(numTris)
	{
		poissonSamplerShared.currentSamplingRadius = 0.0f;
		poissonSamplerShared.numSampleAttemptsAroundPoint = numSampleAttemptsAroundPoint_;
		poissonSamplerShared.maxNumSamples = maxNumSamples_;
		
		getBoundsFromPoints(originalPoints, numPts, poissonSamplerShared.min, max);
		poissonSamplerShared.size = max - poissonSamplerShared.min;

		points.assign(originalPoints, originalPoints + numPts);
		triangles.assign(tris, tris + 3 * numTris);			
		PxRemeshingExt::limitMaxEdgeLength(triangles, points, 2.0f * r, 100, &triangleMap, PxMax(10000u, 4 * numTris));

		PxU32 numTriangles = triangles.size() / 3;
		triangleAreaBuffer.resize(numTriangles);
		for (PxU32 i = 0; i < numTriangles; ++i)
			triangleAreaBuffer[i] = triArea(&triangles[3 * i], points.begin());

		buildTriangleAdjacency(triangles.begin(), numTriangles, adj);

		setSamplingRadius(r);
	}

	void PoissonSamplerShared::addSamples(const PxArray<PxVec3>& samples)
	{
		if (samples.size() > 0)
		{
			for (PxU32 i = 0; i < samples.size(); ++i)
			{
				result.pushBack(samples[i]);				
			}
			rebuildSparseGrid();
		}
	}

	PxI32 PoissonSamplerShared::findSample(const PxVec3& p)
	{
		PxI32 x = PxI32((p.x - min.x) / cellSize);
		PxI32 y = PxI32((p.y - min.y) / cellSize);
		PxI32 z = PxI32((p.z - min.z) / cellSize);
		if (x >= resolution.x) x = resolution.x - 1;
		if (y >= resolution.y) y = resolution.y - 1;
		if (z >= resolution.z) z = resolution.z - 1;

		PxReal minDist = FLT_MAX;
		PxI32 index = -1;
		for (PxI32 oX = -1; oX <= 1; ++oX)
		{
			for (PxI32 oY = -1; oY <= 1; ++oY)
			{
				for (PxI32 oZ = -1; oZ <= 1; ++oZ)
				{
					const PxI32 xx = x + oX;
					const PxI32 yy = y + oY;
					const PxI32 zz = z + oZ;

					if (xx >= 0 && xx < resolution.x && yy >= 0 && yy < resolution.y && zz >= 0 && zz < resolution.z)
					{
						PxI32 cellIndex = xx + resolution.x * yy + (resolution.x * resolution.y) * zz;
						if ((occupiedCellBits[cellIndex >> 5] & (1u << (cellIndex & 31))) != 0)
						{
							const PxPair<const PxI32, SparseGridNode>* it = sparseGrid3D.find(cellIndex);
							if (it)
							{
								const PxReal dist2 = (result[it->second.mPointIndex] - p).magnitudeSquared();
								if (dist2 < minDist) 
								{
									minDist = dist2;
									index = it->second.mPointIndex;
								}
								if (it->second.mExcessStartIndex >= 0)
								{
									for (PxI32 i = it->second.mExcessStartIndex; i < it->second.mExcessEndIndex; ++i)
									{
										const PxReal dist2_ = (result[excessList[i]] - p).magnitudeSquared();
										if (dist2_ < minDist) 
										{
											minDist = dist2_;
											index = it->second.mPointIndex;
										}
									}
								}
								if (minDist == 0.0f)
								{
									return index;
								}
							}
						}
					}
				}
			}
		}
		return -1;
	}

	PxU32 PoissonSamplerShared::removeSamples(const PxArray<PxVec3>& samples)
	{
		if (samples.size() > 0)
		{
			PxArray<PxI32> samplesToRemove;
			samplesToRemove.reserve(samples.size());
			for (PxU32 i = 0; i < samples.size(); ++i)
			{				
				PxI32 index = findSample(samples[i]);
				PX_ASSERT(samples[i] == result[index]);
				if (index >= 0) 				
					samplesToRemove.pushBack(index);				
			}

			PxSort(samplesToRemove.begin(), samplesToRemove.size());

			PxI32 counter = 0;
			for (PxI32 i = PxI32(samplesToRemove.size()) - 1; i >= 0; --i) 
			{
				result[samplesToRemove[i]] = result[result.size() - 1 - counter];
				++counter;
			}
			result.removeRange(result.size() - counter, counter);
			
			rebuildSparseGrid();
			return samplesToRemove.size();
		}
		return 0;
	}

	bool PoissonSamplerShared::setSamplingRadius(PxReal r)
	{
		if (r != currentSamplingRadius)
		{
			currentSamplingRadius = r;
			return rebuildSparseGrid();
		}
		return gridResolutionValid;
	}

	bool PoissonSamplerShared::rebuildSparseGrid()
	{
		const PxReal dimension = 3.0f;
		cellSize = (currentSamplingRadius / PxSqrt(dimension)) * 0.9999f;

		const PxF64 cellsX = PxF64(size.x) / PxF64(cellSize);
		const PxF64 cellsY = PxF64(size.y) / PxF64(cellSize);
		const PxF64 cellsZ = PxF64(size.z) / PxF64(cellSize);

		resolution = Int3(PxMax(1, PxI32(ceil(cellsX))), PxMax(1, PxI32(ceil(cellsY))), PxMax(1, PxI32(ceil(cellsZ))));
		
		const PxF64 numCellsDbl = PxF64(resolution.x) * PxF64(resolution.y) * PxI64(resolution.z);
		if (numCellsDbl >= (1u << 31) ||
			cellsX >= (1u << 31) || 
			cellsY >= (1u << 31) || 
			cellsZ >= (1u << 31))
		{
			gridResolutionValid = false;
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, "Internal grid resolution of sampler too high. Either a smaller mesh or a bigger radius must be used.");
			return false;
		}

		gridResolutionValid = true;

		PxU32 numCells = PxU32(resolution.x * resolution.y * resolution.z);

		occupiedCellBits.clear();
		occupiedCellBits.resize((numCells + 32 - 1) / 32, 0);
		sparseGrid3D.clear();

		for (PxU32 i = 0; i < result.size(); ++i)
			preAddPointToSparseGrid(result[i], i);

		PxI32 cumulativeSum = 0;
		for (PxHashMap<PxI32, SparseGridNode>::Iterator iter = sparseGrid3D.getIterator(); !iter.done(); ++iter)
		{
			if (iter->second.mExcessStartIndex > 0)
			{
				PxI32 start = cumulativeSum;
				cumulativeSum += iter->second.mExcessStartIndex;
				iter->second.mExcessStartIndex = start;
				iter->second.mExcessEndIndex = start - 1;
			}
			else
			{
				iter->second.mExcessStartIndex = -1;
			}
		}
		excessList.resize(cumulativeSum);
		for (PxU32 i = 0; i < result.size(); ++i)
			postAddPointToSparseGrid(result[i], i);

		return true;
	}

	bool PoissonSamplerShared::postAddPointToSparseGrid(const PxVec3& p, PxI32 pointIndex)
	{
		PxI32 x = PxI32((p.x - min.x) / cellSize);
		PxI32 y = PxI32((p.y - min.y) / cellSize);
		PxI32 z = PxI32((p.z - min.z) / cellSize);
		if (x >= resolution.x) x = resolution.x - 1;
		if (y >= resolution.y) y = resolution.y - 1;
		if (z >= resolution.z) z = resolution.z - 1;

		PxI32 cellIndex = x + resolution.x * y + (resolution.x * resolution.y) * z;
			
		SparseGridNode& n = sparseGrid3D[cellIndex];		
		if (n.mExcessStartIndex < 0) 
		{
			PX_ASSERT(n.mPointIndex == pointIndex);
			return true;
		}

		if (n.mExcessEndIndex < n.mExcessStartIndex)
		{
			PX_ASSERT(n.mPointIndex == pointIndex);
			n.mExcessEndIndex++;
			return true;
		}
		else
		{
			excessList[n.mExcessEndIndex] = pointIndex;
			n.mExcessEndIndex++;
			return true;
		}
	}
		
	bool PoissonSamplerShared::preAddPointToSparseGrid(const PxVec3& p, PxI32 pointIndex)
	{
		PxI32 x = PxI32((p.x - min.x) / cellSize);
		PxI32 y = PxI32((p.y - min.y) / cellSize);
		PxI32 z = PxI32((p.z - min.z) / cellSize);
		if (x >= resolution.x) x = resolution.x - 1;
		if (y >= resolution.y) y = resolution.y - 1;
		if (z >= resolution.z) z = resolution.z - 1;

		PxI32 cellIndex = x + resolution.x * y + (resolution.x * resolution.y) * z;

		if ((occupiedCellBits[cellIndex >> 5] & (1u << (cellIndex & 31))) != 0)
		{
			SparseGridNode& n = sparseGrid3D[cellIndex];
			n.mExcessStartIndex++;
			return true;
		}
		else
		{
			sparseGrid3D.insert(cellIndex, SparseGridNode(pointIndex));
			occupiedCellBits[cellIndex >> 5] ^= (1u << (cellIndex & 31));
			return true;
		}
	}

	bool PoissonSamplerShared::addPointToSparseGrid(const PxVec3& p, PxI32 pointIndex)
	{
		PxI32 x = PxI32((p.x - min.x) / cellSize);
		PxI32 y = PxI32((p.y - min.y) / cellSize);
		PxI32 z = PxI32((p.z - min.z) / cellSize);
		if (x >= resolution.x) x = resolution.x - 1;
		if (y >= resolution.y) y = resolution.y - 1;
		if (z >= resolution.z) z = resolution.z - 1;

		PxI32 cellIndex = x + resolution.x * y + (resolution.x * resolution.y) * z;

		//if (sparseGrid3D.ContainsKey(cellIndex))
		//    return false;

		sparseGrid3D.insert(cellIndex, pointIndex);
		occupiedCellBits[cellIndex >> 5] ^= (1u << (cellIndex & 31));
		return true;
	}

	PxReal PoissonSamplerShared::minDistanceToOtherSamplesSquared(const PxVec3& p) const
	{
		PxI32 x = PxI32((p.x - min.x) / cellSize);
		PxI32 y = PxI32((p.y - min.y) / cellSize);
		PxI32 z = PxI32((p.z - min.z) / cellSize);
		if (x >= resolution.x) x = resolution.x - 1;
		if (y >= resolution.y) y = resolution.y - 1;
		if (z >= resolution.z) z = resolution.z - 1;

		PxReal minDist = FLT_MAX;

		for (PxI32 oX = -2; oX <= 2; ++oX)
		{
			for (PxI32 oY = -2; oY <= 2; ++oY)
			{
				for (PxI32 oZ = -2; oZ <= 2; ++oZ)
				{
					const PxI32 xx = x + oX;
					const PxI32 yy = y + oY;
					const PxI32 zz = z + oZ;

					if (xx >= 0 && xx < resolution.x && yy >= 0 && yy < resolution.y && zz >= 0 && zz < resolution.z)
					{
						PxI32 cellIndex = xx + resolution.x * yy + (resolution.x * resolution.y) * zz;
						if ((occupiedCellBits[cellIndex >> 5] & (1u << (cellIndex & 31))) != 0)
						{
							const PxPair<const PxI32, SparseGridNode>* it = sparseGrid3D.find(cellIndex);
							if (it) 
							{
								const PxReal dist2 = (result[it->second.mPointIndex] - p).magnitudeSquared();
								if (dist2 < minDist)
									minDist = dist2;
								if (it->second.mExcessStartIndex >= 0)
								{
									for (PxI32 i = it->second.mExcessStartIndex; i < it->second.mExcessEndIndex; ++i)
									{
										const PxReal dist2_ = (result[excessList[i]] - p).magnitudeSquared();
										if (dist2_ < minDist)
											minDist = dist2_;
									}
								}
							}
						}
					}
				}
			}
		}

		return minDist;
	}
	

	void buildTree(const PxU32* triangles, const PxU32 numTriangles, const PxVec3* points, PxArray<Gu::BVHNode>& tree, PxF32 enlargement = 1e-4f)
	{
		//Computes a bounding box for every triangle in triangles
		Gu::AABBTreeBounds boxes;
		boxes.init(numTriangles);
		for (PxU32 i = 0; i < numTriangles; ++i)
		{
			const PxU32* tri = &triangles[3 * i];
			PxBounds3 box = PxBounds3::empty();
			box.include(points[tri[0]]);
			box.include(points[tri[1]]);
			box.include(points[tri[2]]);
			box.fattenFast(enlargement);
			boxes.getBounds()[i] = box;
		}

		Gu::buildAABBTree(numTriangles, boxes, tree);
	}

	bool TriangleMeshPoissonSampler::isPointInTriangleMesh(const PxVec3& p)
	{
		if (tree.size() == 0)
		{
			//Lazy initialization
			buildTree(originalTriangles, numOriginalTriangles, originalPoints, tree);
			Gu::precomputeClusterInformation(tree.begin(), originalTriangles, numOriginalTriangles, originalPoints, clusters);
		}

		return pointInMesh(p);
	}

	void TriangleMeshPoissonSampler::addSamplesInBox(const PxBounds3& axisAlignedBox, const PxQuat& boxOrientation, bool createVolumeSamples)
	{
		PointInOBBTester pointInOBB(axisAlignedBox.getCenter(), axisAlignedBox.getExtents(), boxOrientation);
		addSamplesInVolume(pointInOBB, axisAlignedBox.getCenter(), axisAlignedBox.getExtents().magnitude(), createVolumeSamples);
	}

	void TriangleMeshPoissonSampler::addSamplesInSphere(const PxVec3& sphereCenter, PxReal sphereRadius, bool createVolumeSamples)
	{
		PointInSphereTester pointInSphere(sphereCenter, sphereRadius);
		addSamplesInVolume(pointInSphere, sphereCenter, sphereRadius, createVolumeSamples);
	}

	//Ideally the sphere center is located on the mesh's surface
	void TriangleMeshPoissonSampler::addSamplesInVolume(const PointInVolumeTester& pointInVolume, const PxVec3& sphereCenter, PxReal sphereRadius, bool volumeSamples)
	{
		PxArray<PxU32> localActiveSamples;
		for (PxU32 i = 0; i < activeSamples.size();)
		{
			if (activeSamples[i].mIndex >= PxI32(poissonSamplerShared.result.size()))
			{
				activeSamples[i] = activeSamples[activeSamples.size() - 1];
				activeSamples.remove(activeSamples.size() - 1);
				continue;
			}

			if (pointInSphere(poissonSamplerShared.result[activeSamples[i].mIndex], sphereCenter, sphereRadius))
				localActiveSamples.pushBack(i);

			++i;
		}

		if (localActiveSamples.size() == 0)
		{
			const PxReal r = poissonSamplerShared.currentSamplingRadius;

			//Try to find a seed sample
			for (PxU32 i = 0; i < triangles.size(); i += 3)
			{
				PxVec3 p = (1.0f / 3.0f) * (points[triangles[i]] + points[triangles[i + 1]] + points[triangles[i + 2]]);
				PxReal triRadius = PxSqrt(PxMax((p - points[triangles[i]]).magnitudeSquared(), PxMax((p - points[triangles[i + 1]]).magnitudeSquared(), (p - points[triangles[i + 2]]).magnitudeSquared())));
				PxReal sum = triRadius + sphereRadius;
				if ((p - sphereCenter).magnitudeSquared() < sum * sum)
				{
					bool success = false;
					for (PxI32 j = 0; j < 30; ++j)
					{
						PxVec3 sample = randomPointOnTriangle(poissonSamplerShared.rnd, triangles.begin() + i, points.begin());
						if (poissonSamplerShared.minDistanceToOtherSamplesSquared(sample) > r * r)
						{
							if (pointInVolume.pointInVolume(sample))
							{
								PxI32 newSampleId = PxI32(poissonSamplerShared.result.size());
								PxU32 sampleTriId = i / 3;

								ActiveSample as;
								createActiveSample(triangleAreaBuffer, newSampleId, sample, 2 * r, sampleTriId, triangles.begin(), points.begin(), adj, as);
								localActiveSamples.pushBack(activeSamples.size());
								activeSamples.pushBack(as);

								poissonSamplerShared.result.pushBack(sample);
								triangleIds.pushBack(triangleMap[sampleTriId]);
								{
									const PxU32 triId = triangleMap[sampleTriId];
									const PxU32* origTri = &originalTriangles[3 * triId];
									barycentricCoordinates.pushBack(computeBarycentricCoordinates(sample, originalPoints[origTri[0]], originalPoints[origTri[1]], originalPoints[origTri[2]]));
								}

								poissonSamplerShared.addPointToSparseGrid(sample, newSampleId);

								success = true;

								if (poissonSamplerShared.maxNumSamples > 0 && poissonSamplerShared.result.size() >= poissonSamplerShared.maxNumSamples)
									return;

								break;
							}
						}
					}
					if (success)
						break;
				}
			}
		}

		//Start poisson sampling
		while (localActiveSamples.size() > 0)
		{
			const PxReal r = poissonSamplerShared.currentSamplingRadius;

			PxI32 localSampleIndex = poissonSamplerShared.rnd.rand32() % localActiveSamples.size();
			PxI32 selectedActiveSample = localActiveSamples[localSampleIndex];
			const ActiveSample& s = activeSamples[selectedActiveSample];

			bool successDist = false;
			bool success = false;
			for (PxI32 i = 0; i < poissonSamplerShared.numSampleAttemptsAroundPoint; ++i)
			{
				PxI32 sampleTriId;
				PxVec3 barycentricCoordinate;
				PxVec3 sample;
				if (samplePointInBallOnSurface(poissonSamplerShared.rnd, s.mCumulativeTriangleAreas, points.begin(), triangles.begin(), s.mNearbyTriangles, poissonSamplerShared.result[s.mIndex], r, sample, sampleTriId, 30, &barycentricCoordinate))
				{
					if (poissonSamplerShared.minDistanceToOtherSamplesSquared(sample) > r * r)
					{
						successDist = true;
						if (pointInVolume.pointInVolume(sample))
						{
							PxI32 newSampleId = PxI32(poissonSamplerShared.result.size());

							ActiveSample as;
							createActiveSample(triangleAreaBuffer, newSampleId, sample, 2 * r, sampleTriId, triangles.begin(), points.begin(), adj, as);
							localActiveSamples.pushBack(activeSamples.size());
							activeSamples.pushBack(as);

							poissonSamplerShared.result.pushBack(sample);
							triangleIds.pushBack(triangleMap[sampleTriId]);
							{
								const PxU32 triId = triangleMap[sampleTriId];
								const PxU32* origTri = &originalTriangles[3 * triId];
								barycentricCoordinates.pushBack(computeBarycentricCoordinates(sample, originalPoints[origTri[0]], originalPoints[origTri[1]], originalPoints[origTri[2]]));
							}

							poissonSamplerShared.addPointToSparseGrid(sample, newSampleId);
							success = true;

							if (poissonSamplerShared.maxNumSamples > 0 && poissonSamplerShared.result.size() >= poissonSamplerShared.maxNumSamples)
								return;

							break;
						}
					}
				}
			}
			if (!successDist)
			{
				PxU32 oldId = activeSamples.size() - 1;
				activeSamples[selectedActiveSample] = activeSamples[activeSamples.size() - 1];
				activeSamples.remove(activeSamples.size() - 1);

				for (PxU32 i = 0; i < localActiveSamples.size(); ++i)
				{
					if (localActiveSamples[i] == oldId)
					{
						localActiveSamples[i] = selectedActiveSample;
						break;
					}
				}
			}
			if (!success)
			{
				localActiveSamples[localSampleIndex] = localActiveSamples[localActiveSamples.size() - 1];
				localActiveSamples.remove(localActiveSamples.size() - 1);
			}
		}		

		if (volumeSamples)
		{
			PxReal randomScale = 0.1f; //Relative to particleSpacing
			PxReal r = (1.0f + 2.0f * randomScale) * 1.001f * poissonSamplerShared.currentSamplingRadius;
			createVolumeSamples(pointInVolume, sphereCenter, sphereRadius, randomScale, r);
		}
	}

	void TriangleMeshPoissonSampler::createVolumeSamples(const PointInVolumeTester& pointInVolume, const PxVec3& sphereCenter, PxReal sphereRadius, PxReal randomScale, PxReal r, bool addToSparseGrid)
	{
		if (tree.size() == 0)
		{
			//Lazy initialization
			buildTree(originalTriangles, numOriginalTriangles, originalPoints, tree);
			Gu::precomputeClusterInformation(tree.begin(), originalTriangles, numOriginalTriangles, originalPoints, clusters);
		}

		
		PxVec3 sphereBoxMin = PxVec3(sphereCenter.x - sphereRadius, sphereCenter.y - sphereRadius, sphereCenter.z - sphereRadius) - poissonSamplerShared.min;
		PxVec3 sphereBoxMax = PxVec3(sphereCenter.x + sphereRadius, sphereCenter.y + sphereRadius, sphereCenter.z + sphereRadius) - poissonSamplerShared.min;

		Int3 start(PxI32(PxFloor(sphereBoxMin.x / r)), PxI32(PxFloor(sphereBoxMin.y / r)), PxI32(PxFloor(sphereBoxMin.z / r)));
		Int3 end(PxI32(PxCeil(sphereBoxMax.x / r)), PxI32(PxCeil(sphereBoxMax.y / r)), PxI32(PxCeil(sphereBoxMax.z / r)));

		for (PxI32 x = start.x; x < end.x; ++x)
		{
			for (PxI32 y = start.y; y < end.y; ++y)
			{
				for (PxI32 z = start.z; z < end.z; ++z)
				{
					PxVec3 p = poissonSamplerShared.min + PxVec3(x * r, y * r, z * r);
					p += PxVec3(poissonSamplerShared.rnd.randomFloat32(-randomScale, randomScale) * poissonSamplerShared.currentSamplingRadius,
						poissonSamplerShared.rnd.randomFloat32(-randomScale, randomScale) * poissonSamplerShared.currentSamplingRadius,
						poissonSamplerShared.rnd.randomFloat32(-randomScale, randomScale) * poissonSamplerShared.currentSamplingRadius);
					if (pointInVolume.pointInVolume(p))
					{
						if (poissonSamplerShared.minDistanceToOtherSamplesSquared(p) > poissonSamplerShared.currentSamplingRadius * poissonSamplerShared.currentSamplingRadius && pointInMesh(p))
						{
							PxI32 newSampleId = PxI32(poissonSamplerShared.result.size());
							poissonSamplerShared.result.pushBack(p);

							if (addToSparseGrid)
								poissonSamplerShared.addPointToSparseGrid(p, newSampleId);

							triangleIds.pushBack(-1);
							barycentricCoordinates.pushBack(PxVec3(0.0f));

							if (poissonSamplerShared.maxNumSamples > 0 && poissonSamplerShared.result.size() >= poissonSamplerShared.maxNumSamples)
								return;
						}
					}
				}
			}
		}
	}



	PxU32 PX_FORCE_INLINE raycast(const PxGeometry& geometry, const PxTransform& transform,
		const PxVec3& rayOrigin, const PxVec3& rayDir, PxReal maxDist, PxHitFlags hitFlags,
		PxU32 maxHits, PxRaycastHit* rayHits)
	{
		return PxGeometryQuery::raycast(
			rayOrigin, rayDir, geometry, transform, maxDist, hitFlags, maxHits, rayHits);
	}

	bool PX_FORCE_INLINE pointInShape(const PxGeometry& geometry, const PxTransform& transform, const PxVec3& p)
	{
		//This is a bit a work-around solution: When a raycast starts inside of a shape, it returns the ray origin as hit point
		PxRaycastHit hit;
		return PxGeometryQuery::raycast(p, PxVec3(1.0f, 0.0f, 0.0f), geometry, transform,
			1.0f, PxHitFlag::ePOSITION, 1, &hit) > 0 && hit.position == p;
	}

	bool projectionCallback(PxReal targetDistance, const PxGeometry& geometry, const PxTransform& transform, PxReal boundingBoxDiagonalLength, const PxVec3& p, const PxVec3& n, PointWithNormal& result)
	{
		PxRaycastHit hitPos;
		PxU32 numHitsPos = raycast(geometry, transform, p - boundingBoxDiagonalLength * n, n, 2 * boundingBoxDiagonalLength, PxHitFlag::eMESH_BOTH_SIDES | PxHitFlag::ePOSITION | PxHitFlag::eNORMAL, 1, &hitPos);

		PxRaycastHit hitNeg;
		PxU32 numHitsNeg = raycast(geometry, transform, p + boundingBoxDiagonalLength * n, -n, 2 * boundingBoxDiagonalLength, PxHitFlag::eMESH_BOTH_SIDES | PxHitFlag::ePOSITION | PxHitFlag::eNORMAL, 1, &hitNeg);

		targetDistance *= 0.5f;

		if (numHitsPos && numHitsNeg)
		{
			if (PxAbs((hitPos.position - p).magnitude() - targetDistance) < PxAbs((hitNeg.position - p).magnitude() - targetDistance))
				result = PointWithNormal(hitPos.position, hitPos.normal);
			else
				result = PointWithNormal(hitNeg.position, hitNeg.normal);
			return true;
		}
		else if (numHitsPos)
		{
			result = PointWithNormal(hitPos.position, hitPos.normal);
			return true;
		}
		else if (numHitsNeg)
		{
			result = PointWithNormal(hitNeg.position, hitNeg.normal);
			return true;
		}
		return false;
	}

	PxVec3 randomDirection(BasicRandom& rnd)
	{
		PxVec3 dir;
		do
		{
			dir = PxVec3((rnd.rand(0.0f, 1.0f) - 0.5f), (rnd.rand(0.0f, 1.0f) - 0.5f), (rnd.rand(0.0f, 1.0f) - 0.5f));
		} while (dir.magnitudeSquared() < 1e-8f);
		return dir.getNormalized();
	}



	PxVec3 getPerpendicularVectorNormalized(const PxVec3& dir)
	{
		PxReal x = PxAbs(dir.x);
		PxReal y = PxAbs(dir.y);
		PxReal z = PxAbs(dir.z);
		if (x >= y && x >= z)
			return dir.cross(PxVec3(0, 1, 0)).getNormalized();
		else if (y >= x && y >= z)
			return dir.cross(PxVec3(0, 0, 1)).getNormalized();
		else
			return dir.cross(PxVec3(1, 0, 0)).getNormalized();
	}

	PxVec3 randomPointOnDisc(BasicRandom& rnd, const PxVec3& point, const PxVec3& normal, PxReal radius, PxReal rUpper)
	{
		//TODO: Use better random number generator
		PxReal r = radius + rnd.rand(0.0f, 1.0f) * (rUpper - radius);
		PxVec3 x = getPerpendicularVectorNormalized(normal);
		PxVec3 y = normal.cross(x).getNormalized();
		PxReal angle = rnd.rand(0.0f, 1.0f) * (2.0f * 3.1415926535898f);
		return point + x * (r * PxCos(angle)) + y * (r * PxSin(angle));
	}

	bool samplePointInBallOnSurface(BasicRandom& rnd, const PxGeometry& shape, const PxTransform& actorGlobalPose, const PxReal diagonalLength,
		const PxVec3& point, const PxVec3& normal, PxReal radius, PointWithNormal& sample, PxI32 numAttempts = 30)
	{
		for (PxI32 i = 0; i < numAttempts; ++i) {
			PxVec3 p = randomPointOnDisc(rnd, point, normal, radius, 2 * radius);

			//Distort the direction of the normal a bit
			PxVec3 n = normal;
			do
			{
				n.x += 0.5f * (rnd.rand(0.0f, 1.0f) - 0.5f);
				n.y += 0.5f * (rnd.rand(0.0f, 1.0f) - 0.5f);
				n.z += 0.5f * (rnd.rand(0.0f, 1.0f) - 0.5f);
			} while (n.magnitudeSquared() < 1e-8f);

			n.normalize();
			
			if (projectionCallback(radius, shape, actorGlobalPose, diagonalLength, p, n, sample)) 
			{
				PxReal d2 = (sample.mPoint - point).magnitudeSquared();
				if (d2 >= radius * radius && d2 < 4 * radius * radius)
					return true;
			}
		}
		return false;
	}


	void ShapePoissonSampler::addSamplesInBox(const PxBounds3& axisAlignedBox, const PxQuat& boxOrientation, bool createVolumeSamples)
	{
		PointInOBBTester pointInOBB(axisAlignedBox.getCenter(), axisAlignedBox.getExtents(), boxOrientation);
		addSamplesInVolume(pointInOBB, axisAlignedBox.getCenter(), axisAlignedBox.getExtents().magnitude(), createVolumeSamples);
	}

	void ShapePoissonSampler::addSamplesInSphere(const PxVec3& sphereCenter, PxReal sphereRadius, bool createVolumeSamples)
	{
		PointInSphereTester pointInSphere(sphereCenter, sphereRadius);
		addSamplesInVolume(pointInSphere, sphereCenter, sphereRadius, createVolumeSamples);
	}

	void ShapePoissonSampler::addSamplesInVolume(const PointInVolumeTester& pointInVolume, const PxVec3& sphereCenter, PxReal sphereRadius, bool volumeSamples)
	{
		PxReal boundingBoxDiagonalLength = poissonSamplerShared.size.magnitude();
		
		PxArray<PxU32> localActiveSamples;
		for (PxU32 i = 0; i < activeSamples.size();)
		{
			if (activeSamples[i].mIndex >= PxI32(poissonSamplerShared.result.size()))
			{
				activeSamples[i] = activeSamples[activeSamples.size() - 1];
				activeSamples.remove(activeSamples.size() - 1);
				continue;
			}

			if (pointInSphere(poissonSamplerShared.result[activeSamples[i].mIndex], sphereCenter, sphereRadius))
				localActiveSamples.pushBack(i);

			++i;
		}

		if (localActiveSamples.size() == 0)
		{
			PxVec3 center = poissonSamplerShared.min + 0.5f * poissonSamplerShared.size;
			PointWithNormal sample;
			PxVec3 arbitrarySeedPointOnSurface;
			
			PxVec3 reference = sphereCenter - center;
			reference.normalizeSafe();

			
			for (PxI32 i = 0; i < 1000; ++i)
			{
				PxVec3 dir = /*reference + 0.5f**/randomDirection(poissonSamplerShared.rnd);
				dir.normalize();
				if (projectionCallback(poissonSamplerShared.currentSamplingRadius, shape, actorGlobalPose, boundingBoxDiagonalLength, sphereCenter, dir, sample))
				{
					if (poissonSamplerShared.minDistanceToOtherSamplesSquared(sample.mPoint) > poissonSamplerShared.currentSamplingRadius * poissonSamplerShared.currentSamplingRadius)
					{
						if (pointInVolume.pointInVolume(sample.mPoint))
						{
							PxI32 newSampleId = PxI32(poissonSamplerShared.result.size());

							localActiveSamples.pushBack(activeSamples.size());
							activeSamples.pushBack(IndexWithNormal(newSampleId, sample.mNormal));

							poissonSamplerShared.result.pushBack(sample.mPoint);
							poissonSamplerShared.addPointToSparseGrid(sample.mPoint, newSampleId);
							
							if (poissonSamplerShared.maxNumSamples > 0 && poissonSamplerShared.result.size() >= poissonSamplerShared.maxNumSamples)
								return;

							break;
						}
					}
				}
			}
		}

		while (localActiveSamples.size() > 0)
		{
			PxI32 localSampleIndex = poissonSamplerShared.rnd.rand32() % localActiveSamples.size();
			PxI32 selectedActiveSample = localActiveSamples[localSampleIndex];
			const IndexWithNormal& s = activeSamples[selectedActiveSample];
			
			bool successDist = false;
			bool success = false;
			for (PxI32 i = 0; i < poissonSamplerShared.numSampleAttemptsAroundPoint; ++i)
			{
				PointWithNormal sample;
				if (samplePointInBallOnSurface(poissonSamplerShared.rnd, shape, actorGlobalPose, boundingBoxDiagonalLength, poissonSamplerShared.result[s.mIndex], s.mNormal, poissonSamplerShared.currentSamplingRadius, sample))
				{
					if (poissonSamplerShared.minDistanceToOtherSamplesSquared(sample.mPoint) > poissonSamplerShared.currentSamplingRadius * poissonSamplerShared.currentSamplingRadius)
					{
						successDist = true;
						if (pointInVolume.pointInVolume(sample.mPoint))
						{
							successDist = true;

							PxI32 newSampleId = PxI32(poissonSamplerShared.result.size());
							localActiveSamples.pushBack(activeSamples.size());
							activeSamples.pushBack(IndexWithNormal(newSampleId, sample.mNormal));

							poissonSamplerShared.result.pushBack(sample.mPoint);
							poissonSamplerShared.addPointToSparseGrid(sample.mPoint, newSampleId);
							success = true;

							if (poissonSamplerShared.maxNumSamples > 0 && poissonSamplerShared.result.size() >= poissonSamplerShared.maxNumSamples)
								return;

							break;
						}
					}
				}
			}
			if (!successDist)
			{
				PxU32 oldId = activeSamples.size() - 1;
				activeSamples[selectedActiveSample] = activeSamples[activeSamples.size() - 1];
				activeSamples.remove(activeSamples.size() - 1);

				for (PxU32 i = 0; i < localActiveSamples.size(); ++i)
				{
					if (localActiveSamples[i] == oldId)
					{
						localActiveSamples[i] = selectedActiveSample;
						break;
					}
				}
			}
			if (!success)
			{
				localActiveSamples[localSampleIndex] = localActiveSamples[localActiveSamples.size() - 1];
				localActiveSamples.remove(localActiveSamples.size() - 1);
			}
		}
		
		if (volumeSamples)
		{
			PxReal randomScale = 0.1f; //Relative to particleSpacing
			PxReal r = (1.0f + 2.0f * randomScale) * 1.001f * poissonSamplerShared.currentSamplingRadius;
			createVolumeSamples(pointInVolume, sphereCenter, sphereRadius, randomScale, r);
		}
	}	

	void ShapePoissonSampler::createVolumeSamples(const PointInVolumeTester& pointInVolume, const PxVec3& sphereCenter, PxReal sphereRadius, PxReal randomScale, PxReal r, bool addToSparseGrid)
	{
		PxVec3 sphereBoxMin = PxVec3(sphereCenter.x - sphereRadius, sphereCenter.y - sphereRadius, sphereCenter.z - sphereRadius) - poissonSamplerShared.min;
		PxVec3 sphereBoxMax = PxVec3(sphereCenter.x + sphereRadius, sphereCenter.y + sphereRadius, sphereCenter.z + sphereRadius) - poissonSamplerShared.min;

		Int3 start(PxI32(PxFloor(sphereBoxMin.x / r)), PxI32(PxFloor(sphereBoxMin.y / r)), PxI32(PxFloor(sphereBoxMin.z / r)));
		Int3 end(PxI32(PxCeil(sphereBoxMax.x / r)), PxI32(PxCeil(sphereBoxMax.y / r)), PxI32(PxCeil(sphereBoxMax.z / r)));

		for (PxI32 x = start.x; x < end.x; ++x)
		{
			for (PxI32 y = start.y; y < end.y; ++y)
			{
				for (PxI32 z = start.z; z < end.z; ++z)
				{
					PxVec3 p = poissonSamplerShared.min + PxVec3(x * r, y * r, z * r);
					p += PxVec3(poissonSamplerShared.rnd.randomFloat32(-randomScale, randomScale) * poissonSamplerShared.currentSamplingRadius,
						poissonSamplerShared.rnd.randomFloat32(-randomScale, randomScale) * poissonSamplerShared.currentSamplingRadius,
						poissonSamplerShared.rnd.randomFloat32(-randomScale, randomScale) * poissonSamplerShared.currentSamplingRadius);
					if (pointInVolume.pointInVolume(p) && pointInShape(shape, actorGlobalPose, p))
					{
						if (poissonSamplerShared.minDistanceToOtherSamplesSquared(p) > poissonSamplerShared.currentSamplingRadius * poissonSamplerShared.currentSamplingRadius)
						{
							PxI32 newSampleId = PxI32(poissonSamplerShared.result.size());
							poissonSamplerShared.result.pushBack(p);
							
							if (addToSparseGrid)
								poissonSamplerShared.addPointToSparseGrid(p, newSampleId);

							if (poissonSamplerShared.maxNumSamples > 0 && poissonSamplerShared.result.size() >= poissonSamplerShared.maxNumSamples)
								return;
						}
					}
				}
			}
		}
	}

	//Use for triangle meshes
	//https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
	bool PxSamplingExt::poissonSample(const PxSimpleTriangleMesh& mesh, PxReal r, PxArray<PxVec3>& result, PxReal rVolume, PxArray<PxI32>* triangleIds, PxArray<PxVec3>* barycentricCoordinates,
		const PxBounds3* axisAlignedBox, const PxQuat* boxOrientation, PxU32 maxNumSamples, PxU32 numSampleAttemptsAroundPoint)
	{
		TriangleMeshPoissonSampler sampler(reinterpret_cast<const PxU32*>(mesh.triangles.data), mesh.triangles.count, reinterpret_cast<const PxVec3*>(mesh.points.data), mesh.points.count, r, numSampleAttemptsAroundPoint, maxNumSamples);

		if (!sampler.poissonSamplerShared.gridResolutionValid)
		{
			return false;
		}

		PxVec3 center = 0.5f*(sampler.max + sampler.poissonSamplerShared.min);
		PxReal boundingSphereRadius = 1.001f * (sampler.max - sampler.poissonSamplerShared.min).magnitude() * 0.5f;

		if (axisAlignedBox == NULL || boxOrientation == NULL) 
		{			
			sampler.addSamplesInSphere(center, boundingSphereRadius, false);

			if (rVolume > 0.0f)
			{
				AlwaysInsideTester tester;
				sampler.createVolumeSamples(tester, center, boundingSphereRadius, 0.1f, rVolume, false);
			}
		}
		else
		{
			sampler.addSamplesInBox(*axisAlignedBox, *boxOrientation, false);

			if (rVolume > 0.0f)
			{
				PointInOBBTester tester(axisAlignedBox->getCenter(), axisAlignedBox->getExtents(), *boxOrientation);
				sampler.createVolumeSamples(tester, center, boundingSphereRadius, 0.1f, rVolume, false);
			}
		}

		result = sampler.getSamples();
		if (triangleIds)
			*triangleIds = sampler.getSampleTriangleIds();
		if (barycentricCoordinates)
			*barycentricCoordinates = sampler.getSampleBarycentrics();

		return true;
	}	

	bool PxSamplingExt::poissonSample(const PxGeometry& geometry, const PxTransform& transform, const PxBounds3& worldBounds, PxReal r, PxArray<PxVec3>& result, PxReal rVolume,
		const PxBounds3* axisAlignedBox, const PxQuat* boxOrientation, PxU32 maxNumSamples, PxU32 numSampleAttemptsAroundPoint)
	{
		PxVec3 center = worldBounds.getCenter();

		ShapePoissonSampler sampler(geometry, transform, worldBounds, r, numSampleAttemptsAroundPoint, maxNumSamples);
		PxReal boundingSphereRadius = 1.001f * worldBounds.getExtents().magnitude();

		if (!sampler.poissonSamplerShared.gridResolutionValid)
			return false;

		if (axisAlignedBox == NULL || boxOrientation == NULL)
		{
			sampler.addSamplesInSphere(center, worldBounds.getExtents().magnitude() * 1.001f, false);

			if (rVolume > 0.0f)
			{				
				AlwaysInsideTester tester;
				sampler.createVolumeSamples(tester, center, boundingSphereRadius, 0.1f, rVolume, false);
			}
		}
		else
		{
			sampler.addSamplesInBox(*axisAlignedBox, *boxOrientation, false);

			if (rVolume > 0.0f)
			{				
				PointInOBBTester tester(axisAlignedBox->getCenter(), axisAlignedBox->getExtents(), *boxOrientation);
				sampler.createVolumeSamples(tester, center, boundingSphereRadius, 0.1f, rVolume, false);
			}
		}

		result = sampler.getSamples();
		return true;
	}
}
