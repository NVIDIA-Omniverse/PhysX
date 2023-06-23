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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuSDF.h"

#include "GuAABBTreeNode.h"
#include "GuAABBTree.h"
#include "GuAABBTreeBounds.h"
#include "GuWindingNumber.h"

#include "GuAABBTreeNode.h"
#include "GuDistancePointBox.h"
#include "GuDistancePointTriangle.h"
#include "GuAABBTreeQuery.h"
#include "GuIntersectionRayTriangle.h"
#include "GuIntersectionRayBox.h"

#include "foundation/PxAtomic.h"
#include "foundation/PxThread.h"
#include "common/GuMeshAnalysis.h"
#include "GuMeshAnalysis.h"

namespace physx
{
namespace Gu
{
	SDF::~SDF()
	{
		PX_FREE(mSdf);
		PX_FREE(mSubgridStartSlots);
		PX_FREE(mSubgridSdf);
	}

	PxReal* SDF::allocateSdfs(const PxVec3& meshLower, const PxReal& spacing, const PxU32 dimX, const PxU32 dimY, const PxU32 dimZ, 
		const PxU32 subgridSize, const PxU32 sdfSubgrids3DTexBlockDimX, const PxU32 sdfSubgrids3DTexBlockDimY, const PxU32 sdfSubgrids3DTexBlockDimZ,
		PxReal minSdfValueSubgrids, PxReal maxSdfValueSubgrids, PxU32 sparsePixelNumBytes)
	{
		PX_ASSERT(!mSdf);
		PX_ASSERT(!mSubgridStartSlots);
		PX_ASSERT(!mSubgridSdf);

		mMeshLower = meshLower;
		mSpacing = spacing;
		mDims.x = dimX;
		mDims.y = dimY;
		mDims.z = dimZ;

		mSubgridSize = subgridSize;

		mSdfSubgrids3DTexBlockDim.x = sdfSubgrids3DTexBlockDimX;
		mSdfSubgrids3DTexBlockDim.y = sdfSubgrids3DTexBlockDimY;
		mSdfSubgrids3DTexBlockDim.z = sdfSubgrids3DTexBlockDimZ;

		mSubgridsMinSdfValue = minSdfValueSubgrids;
		mSubgridsMaxSdfValue = maxSdfValueSubgrids;
		mBytesPerSparsePixel = sparsePixelNumBytes;

		if (subgridSize > 0)
		{
			//Sparse sdf
			PX_ASSERT(dimX % subgridSize == 0);
			PX_ASSERT(dimY % subgridSize == 0);
			PX_ASSERT(dimZ % subgridSize == 0);

			PxU32 x = dimX / subgridSize;
			PxU32 y = dimY / subgridSize;
			PxU32 z = dimZ / subgridSize;

			mNumSdfs = (x + 1) * (y + 1) * (z + 1);
			mNumSubgridSdfs = mBytesPerSparsePixel * sdfSubgrids3DTexBlockDimX * (subgridSize + 1) * sdfSubgrids3DTexBlockDimY * (subgridSize + 1) * sdfSubgrids3DTexBlockDimZ * (subgridSize + 1);
			mNumStartSlots = x * y * z;
			
			mSubgridSdf = PX_ALLOCATE(PxU8, mNumSubgridSdfs, "PxU8");
			mSubgridStartSlots = PX_ALLOCATE(PxU32, mNumStartSlots, "PxU32");
			mSdf = PX_ALLOCATE(PxReal, mNumSdfs, "PxReal");
		}
		else
		{
			//Dense sdf - no sparse grid data required
			mSubgridStartSlots = NULL;
			mSubgridSdf = NULL;

			mNumSdfs = dimX * dimY*dimZ;
			mNumSubgridSdfs = 0;
			mNumStartSlots = 0;
			mSdf = PX_ALLOCATE(PxReal, mNumSdfs, "PxReal");
		}

		return mSdf;
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

	class LineSegmentTrimeshIntersectionTraversalController
	{
	private:
		const PxU32* mTriangles;
		const PxVec3* mPoints;
		PxVec3 mSegmentStart;
		PxVec3 mSegmentEnd;
		PxVec3 mDirection;
		bool mIntersects;

	public:
		LineSegmentTrimeshIntersectionTraversalController(const PxU32* triangles, const PxVec3* points, PxVec3 segmentStart, PxVec3 segmentEnd) 
			: mTriangles(triangles), mPoints(points), mSegmentStart(segmentStart), mSegmentEnd(segmentEnd), mDirection(segmentEnd - segmentStart), mIntersects(false)
		{
		}

		void reset(PxVec3 segmentStart, PxVec3 segmentEnd)
		{
			mSegmentStart = segmentStart;
			mSegmentEnd = segmentEnd;
			mDirection = segmentEnd - segmentStart;
			mIntersects = false;
		}

		bool intersectionDetected() const
		{
			return mIntersects;
		}

		PX_FORCE_INLINE Gu::TraversalControl::Enum analyze(const Gu::BVHNode& node, PxI32)
		{
			if (node.isLeaf())
			{
				PxI32 j = node.getPrimitiveIndex();				
				const PxU32* tri = &mTriangles[3 * j];

				PxReal at, au, av;
				if (Gu::intersectRayTriangle(mSegmentStart, mDirection, mPoints[tri[0]], mPoints[tri[1]], mPoints[tri[2]], at, au, av, false, 1e-4f) && at >= 0.0f && at <= 1.0f)
				{
					mIntersects = true;
					return TraversalControl::eAbort;
				}				

				return TraversalControl::eDontGoDeeper;
			}

			PxReal tnear, tfar;
			if (Gu::intersectRayAABB(node.mBV.minimum, node.mBV.maximum, mSegmentStart, mDirection, tnear, tfar) >= 0 && ((tnear >= 0.0f && tnear <= 1.0f) || (tfar >= 0.0f && tfar <= 1.0f) || node.mBV.contains(mSegmentStart)))
				return TraversalControl::eGoDeeper;
			return TraversalControl::eDontGoDeeper;
		}

	private:
		PX_NOCOPY(LineSegmentTrimeshIntersectionTraversalController)
	};

	class ClosestDistanceToTrimeshTraversalController
	{
	private:
		PxReal mClosestDistanceSquared;
		const PxU32* mTriangles;
		const PxVec3* mPoints;
		const Gu::BVHNode* mNodes;
		PxVec3 mQueryPoint;
		PxVec3 mClosestPoint;
		PxI32 mClosestTriId;

	public:
		PX_FORCE_INLINE ClosestDistanceToTrimeshTraversalController(){}

		PX_FORCE_INLINE ClosestDistanceToTrimeshTraversalController(const PxU32* triangles, const PxVec3* points, Gu::BVHNode* nodes) :
			mTriangles(triangles), mPoints(points), mNodes(nodes), mQueryPoint(0.0f), mClosestPoint(0.0f), mClosestTriId(-1)
		{
			initialize(triangles, points, nodes);
		}

		void initialize(const PxU32* triangles, const PxVec3* points, Gu::BVHNode* nodes)
		{
			mTriangles = triangles;
			mPoints = points;
			mNodes = nodes;
			mQueryPoint = PxVec3(0.0f);
			mClosestPoint = PxVec3(0.0f);
			mClosestTriId = -1;
			mClosestDistanceSquared = PX_MAX_F32;
		}

		PX_FORCE_INLINE void setQueryPoint(const PxVec3& queryPoint)
		{
			this->mQueryPoint = queryPoint;
			mClosestDistanceSquared = FLT_MAX;
			mClosestPoint = PxVec3(0.0f);
			mClosestTriId = -1;
		}

		PX_FORCE_INLINE const PxVec3& getClosestPoint() const
		{
			return mClosestPoint;
		}

		PX_FORCE_INLINE PxReal distancePointBoxSquared(const PxBounds3& box, const PxVec3& point)
		{
			PxVec3 closestPt = box.minimum.maximum(box.maximum.minimum(point));

			return (closestPt - point).magnitudeSquared();
		}

		PX_FORCE_INLINE Gu::TraversalControl::Enum analyze(const Gu::BVHNode& node, PxI32)
		{
			if (distancePointBoxSquared(node.mBV, mQueryPoint) >= mClosestDistanceSquared)
				return Gu::TraversalControl::eDontGoDeeper;

			if (node.isLeaf())
			{
				const PxI32 j = node.getPrimitiveIndex();
				const PxU32* tri = &mTriangles[3 * j];
				

				aos::FloatV t1, t2;
				aos::Vec3V q = V3LoadU(mQueryPoint);
				aos::Vec3V a = V3LoadU(mPoints[tri[0]]);
				aos::Vec3V b = V3LoadU(mPoints[tri[1]]);
				aos::Vec3V c = V3LoadU(mPoints[tri[2]]);
				aos::Vec3V cp;
				aos::FloatV d = Gu::distancePointTriangleSquared(q, a, b, c, t1, t2, cp);
				PxReal d2;
				FStore(d, &d2);
				PxVec3 closest;
				V3StoreU(cp, closest);

				//PxReal t1, t2;
				//const PxVec3 closest = Gu::closestPtPointTriangle(mQueryPoint, mPoints[tri[0]], mPoints[tri[1]], mPoints[tri[2]], t1, t2);
				//PxReal d2 = (closest - mQueryPoint).magnitudeSquared();
				if (d2 < mClosestDistanceSquared)
				{
					mClosestDistanceSquared = d2;
					mClosestTriId = j;
					mClosestPoint = closest;
				}
				return Gu::TraversalControl::eDontGoDeeper;
			}

			const Gu::BVHNode& nodePos = mNodes[node.getPosIndex()];
			const PxReal distSquaredPos = distancePointBoxSquared(nodePos.mBV, mQueryPoint);
			const Gu::BVHNode& nodeNeg = mNodes[node.getNegIndex()];
			const PxReal distSquaredNeg = distancePointBoxSquared(nodeNeg.mBV, mQueryPoint);

			if (distSquaredPos < distSquaredNeg)
			{
				if (distSquaredPos < mClosestDistanceSquared)
					return Gu::TraversalControl::eGoDeeper;
			}
			else
			{
				if (distSquaredNeg < mClosestDistanceSquared)
					return Gu::TraversalControl::eGoDeeperNegFirst;
			}
			return Gu::TraversalControl::eDontGoDeeper;
		}

		PxI32 getClosestTriId() const { return mClosestTriId; }

		void setClosestStart(const PxReal closestDistanceSquared, PxI32 closestTriangle, const PxVec3& closestPoint) 
		{ 
			mClosestDistanceSquared = closestDistanceSquared; 
			mClosestTriId = closestTriangle;
			mClosestPoint = closestPoint;
		}

	private:
		PX_NOCOPY(ClosestDistanceToTrimeshTraversalController)
	};

	class PointOntoTriangleMeshProjector : public PxPointOntoTriangleMeshProjector, public PxUserAllocated
	{
		PxArray<Gu::BVHNode> mNodes;
		ClosestDistanceToTrimeshTraversalController mEvaluator;
	public:
		PointOntoTriangleMeshProjector(const PxVec3* vertices, const PxU32* indices, PxU32 numTriangles)
		{
			buildTree(indices, numTriangles, vertices, mNodes);
			mEvaluator.initialize(indices, vertices, mNodes.begin());
		}

		virtual PxVec3 projectPoint(const PxVec3& point) PX_OVERRIDE
		{
			mEvaluator.setQueryPoint(point);
			Gu::traverseBVH(mNodes.begin(), mEvaluator);
			PxVec3 closestPoint = mEvaluator.getClosestPoint();
			return closestPoint;
		}

		virtual PxVec3 projectPoint(const PxVec3& point, PxU32& closetTriangleIndex) PX_OVERRIDE
		{
			mEvaluator.setQueryPoint(point);
			Gu::traverseBVH(mNodes.begin(), mEvaluator);
			PxVec3 closestPoint = mEvaluator.getClosestPoint();
			closetTriangleIndex = mEvaluator.getClosestTriId();
			return closestPoint;
		}

		virtual void release() PX_OVERRIDE
		{
			mNodes.reset();
			PX_FREE_THIS;
		}
	};

	PxPointOntoTriangleMeshProjector* PxCreatePointOntoTriangleMeshProjector(const PxVec3* vertices, const PxU32* indices, PxU32 numTriangleIndices)
	{
		return PX_NEW(PointOntoTriangleMeshProjector)(vertices, indices, numTriangleIndices);
	}

	void windingNumbers(const PxVec3* vertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth, 
		PxReal* windingNumbers, PxVec3 min, PxVec3 max, PxVec3* sampleLocations)
	{
		const PxVec3 extents(max - min);
		const PxVec3 delta(extents.x / width, extents.y / height, extents.z / depth);
		const PxVec3 offset = min + PxVec3(0.5f * delta.x, 0.5f * delta.y, 0.5f * delta.z);

		PxArray<Gu::BVHNode> tree;
		buildTree(indices, numTriangleIndices / 3, vertices, tree);

		PxHashMap<PxU32, Gu::ClusterApproximation> clusters;
		Gu::precomputeClusterInformation(tree.begin(), indices, numTriangleIndices / 3, vertices, clusters);

		for (PxU32 x = 0; x < width; ++x)
		{
			for (PxU32 y = 0; y < height; ++y)
			{
				for (PxU32 z = 0; z < depth; ++z)
				{
					PxVec3 queryPoint(x * delta.x + offset.x, y * delta.y + offset.y, z * delta.z + offset.z);
					PxReal windingNumber = Gu::computeWindingNumber(tree.begin(), queryPoint, clusters, indices, vertices);
					windingNumbers[z * width * height + y * width + x] = windingNumber; // > 0.5f ? PxU32(-1) : 0;
					if (sampleLocations)
						sampleLocations[z * width * height + y * width + x] = queryPoint;
				}
			}
		}
	}	

	struct Range
	{
		PxI32 mStart;
		PxI32 mEnd;
		bool mInsideStart;
		bool mInsideEnd;

		Range(PxI32 start, PxI32 end, bool insideStart, bool insideEnd) : mStart(start), mEnd(end), mInsideStart(insideStart), mInsideEnd(insideEnd) { }
	};

	class GridQueryPointSampler
	{
		PxVec3 mOrigin;
		PxVec3 mCellSize;
		PxI32 mOffsetX, mOffsetY, mOffsetZ;
		PxI32 mStepX, mStepY, mStepZ;

	public:
		GridQueryPointSampler() {}

		GridQueryPointSampler(const PxVec3& origin, const PxVec3& cellSize, bool cellCenteredSamples,
			PxI32 offsetX = 0, PxI32 offsetY = 0, PxI32 offsetZ = 0, PxI32 stepX = 1, PxI32 stepY = 1, PxI32 stepZ = 1)
			: mCellSize(cellSize), mOffsetX(offsetX), mOffsetY(offsetY), mOffsetZ(offsetZ), mStepX(stepX), mStepY(stepY), mStepZ(stepZ)
		{
			if (cellCenteredSamples)
				mOrigin = origin + 0.5f * cellSize;
			else
				mOrigin = origin;
		}

		PX_FORCE_INLINE PxVec3 getOrigin()
		{
			return mOrigin;
		}

		PX_FORCE_INLINE PxVec3 getActiveCellSize()
		{
			return PxVec3(mCellSize.x * mStepX, mCellSize.y * mStepY, mCellSize.z * mStepZ);
		}

		PX_FORCE_INLINE PxVec3 getPoint(PxI32 x, PxI32 y, PxI32 z)
		{
			return PxVec3(mOrigin.x + (x * mStepX + mOffsetX) * mCellSize.x,
				mOrigin.y + (y * mStepY + mOffsetY) * mCellSize.y,
				mOrigin.z + (z * mStepZ + mOffsetZ) * mCellSize.z);
		}
	};

	struct SDFCalculationData
	{
		const PxVec3* vertices;
		const PxU32* indices;
		PxU32 numTriangleIndices;
		PxU32 width;
		PxU32 height;
		PxU32 depth;
		PxReal* sdf;
		PxVec3* sampleLocations;
		GridQueryPointSampler* pointSampler;

		PxArray<Gu::BVHNode>* tree;
		PxHashMap<PxU32, Gu::ClusterApproximation>* clusters;
		PxI32 batchSize = 32;
		PxI32 end;
		PxI32* progress;

		bool optimizeInsideOutsideCalculation; //Toggle to enable an additional optimization for faster inside/outside classification
		bool signOnly;
	};



	void windingNumbersInsideCheck(const PxVec3* vertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
		bool* insideResult, PxVec3 min, PxVec3 max, PxVec3* sampleLocations)
	{
#if PX_DEBUG
		PxBounds3 bounds(min, max);
		for (PxU32 i = 0; i < numTriangleIndices; ++i)		
			PX_ASSERT(bounds.contains(vertices[indices[i]]));		
#endif

		const PxVec3 extents(max - min);
		const PxVec3 delta(extents.x / width, extents.y / height, extents.z / depth);
		const PxVec3 offset = min + PxVec3(0.5f * delta.x, 0.5f * delta.y, -0.5f * delta.z);

		PxArray<Gu::BVHNode> tree;
		buildTree(indices, numTriangleIndices / 3, vertices, tree);

		PxHashMap<PxU32, Gu::ClusterApproximation> clusters;
		Gu::precomputeClusterInformation(tree.begin(), indices, numTriangleIndices / 3, vertices, clusters);

		LineSegmentTrimeshIntersectionTraversalController intersector(indices, vertices, PxVec3(0.0f), PxVec3(0.0f));

		PxArray<Range> stack;
		for (PxU32 x = 0; x < width; ++x)
		{
			for (PxU32 y = 0; y < height; ++y)
			{
				stack.pushBack(Range(0, depth+2, false, false));
				while (stack.size() > 0)
				{
					Range r = stack.popBack();

					PxI32 center = (r.mStart + r.mEnd) / 2;	
					if (center == r.mStart)
					{
						if (r.mStart > 0 && r.mStart <= PxI32(depth)) 
						{
							insideResult[(r.mStart - 1) * width * height + y * width + x] = r.mInsideStart;
							if (sampleLocations)
								sampleLocations[(r.mStart - 1) * width * height + y * width + x] = PxVec3(x * delta.x + offset.x, y * delta.y + offset.y, r.mStart * delta.z + offset.z);
						}
						continue;
					}

					PxVec3 queryPoint = PxVec3(x * delta.x + offset.x, y * delta.y + offset.y, center * delta.z + offset.z);
					bool inside = Gu::computeWindingNumber(tree.begin(), queryPoint, clusters, indices, vertices) > 0.5f;
					
					if (inside != r.mInsideStart)
						stack.pushBack(Range(r.mStart, center, r.mInsideStart, inside));
					else
					{
						PxVec3 p = PxVec3(x * delta.x + offset.x, y * delta.y + offset.y, r.mStart * delta.z + offset.z);
						intersector.reset(p, queryPoint);
						Gu::traverseBVH(tree.begin(), intersector);
						if (!intersector.intersectionDetected())
						{
							PxI32 e = PxMin(center, PxI32(depth) + 1);
							for (PxI32 z = PxMax(1, r.mStart); z < e; ++z)
							{
								insideResult[(z - 1) * width * height + y * width + x] = inside;
								if (sampleLocations)
									sampleLocations[(z - 1) * width * height + y * width + x] = queryPoint;
							}
						}
						else
							stack.pushBack(Range(r.mStart, center, r.mInsideStart, inside));
					}
					
					
					if (inside != r.mInsideEnd)
						stack.pushBack(Range(center, r.mEnd, inside, r.mInsideEnd));
					else
					{
						PxVec3 p = PxVec3(x * delta.x + offset.x, y * delta.y + offset.y, r.mEnd * delta.z + offset.z);
						intersector.reset(queryPoint, p);
						Gu::traverseBVH(tree.begin(), intersector);
						if (!intersector.intersectionDetected())
						{
							PxI32 e = PxMin(r.mEnd, PxI32(depth) + 1);
							for (PxI32 z = PxMax(1, center); z < e; ++z)
							{
								insideResult[(z - 1) * width * height + y * width + x] = inside;
								if (sampleLocations)
									sampleLocations[(z - 1) * width * height + y * width + x] = queryPoint;
							}
						}
						else
							stack.pushBack(Range(center, r.mEnd, inside, r.mInsideEnd));
					}					
				}
			}
		}
	}
	
	

	void idToXYZ(PxU32 id, PxU32 sizeX, PxU32 sizeY, PxU32& xi, PxU32& yi, PxU32& zi)
	{
		xi = id % sizeX; id /= sizeX;
		yi = id % sizeY;
		zi = id / sizeY;
	}

	void idToXY(PxU32 id, PxU32 sizeX, PxU32& xi, PxU32& yi)
	{
		xi = id % sizeX;
		yi = id / sizeX;
	}

	void* computeSDFThreadJob(void* data)
	{
		SDFCalculationData& d = *reinterpret_cast<SDFCalculationData*>(data);

		PxI32 lastTriangle = -1;

		PxArray<Range> stack;
		LineSegmentTrimeshIntersectionTraversalController intersector(d.indices, d.vertices, PxVec3(0.0f), PxVec3(0.0f));

		PxI32 start = physx::PxAtomicAdd(d.progress, d.batchSize) - d.batchSize;
		while (start < d.end)
		{
			PxI32 end = PxMin(d.end, start + d.batchSize);

			PxU32 yStart, zStart;
			idToXY(start, d.height, yStart, zStart);
			for (PxI32 id = start; id < end; ++id)
			{
				PxU32 y, z;
				idToXY(id, d.height, y, z);
				if (y < yStart)
					yStart = 0;

				if (d.optimizeInsideOutsideCalculation)
				{
					stack.pushBack(Range(0, d.width + 2, false, false));
					while (stack.size() > 0)
					{
						Range r = stack.popBack();

						PxI32 center = (r.mStart + r.mEnd) / 2;
						if (center == r.mStart)
						{
							if (r.mStart > 0 && r.mStart <= PxI32(d.width))
							{
								if (r.mInsideStart)
									d.sdf[z * d.width * d.height + y * d.width + (r.mStart - 1)] *= -1.0f;
							}
							continue;
						}

						PxVec3 queryPoint = d.pointSampler->getPoint(center - 1, y, z); 


						bool inside = false;
						bool computeWinding = true;
						if (id > start && y > yStart)
						{
							PxReal s = d.sdf[z * d.width * d.height + (y - 1) * d.width + (center - 1)];
							if (PxAbs(s) > d.pointSampler->getActiveCellSize().y)
							{
								inside = s < 0.0f;
								computeWinding = false;
							}
						}

						if (computeWinding)
							inside = Gu::computeWindingNumber(d.tree->begin(), queryPoint, *d.clusters, d.indices, d.vertices) > 0.5f;


						if (inside != r.mInsideStart)
							stack.pushBack(Range(r.mStart, center, r.mInsideStart, inside));
						else
						{
							PxVec3 p = d.pointSampler->getPoint(r.mStart - 1, y, z);
							intersector.reset(p, queryPoint);
							Gu::traverseBVH(d.tree->begin(), intersector);
							if (!intersector.intersectionDetected())
							{
								PxI32 e = PxMin(center, PxI32(d.width) + 1);
								for (PxI32 x = PxMax(1, r.mStart); x < e; ++x)
								{
									if (inside)
										d.sdf[z * d.width * d.height + y * d.width + (x - 1)] *= -1.0f;
								}
							}
							else
								stack.pushBack(Range(r.mStart, center, r.mInsideStart, inside));
						}


						if (inside != r.mInsideEnd)
							stack.pushBack(Range(center, r.mEnd, inside, r.mInsideEnd));
						else
						{
							PxVec3 p = d.pointSampler->getPoint(r.mEnd - 1, y, z); 
							intersector.reset(queryPoint, p);
							Gu::traverseBVH(d.tree->begin(), intersector);
							if (!intersector.intersectionDetected())
							{
								PxI32 e = PxMin(r.mEnd, PxI32(d.width) + 1);
								for (PxI32 x = PxMax(1, center); x < e; ++x)
								{
									if (inside)
										d.sdf[z * d.width * d.height + y * d.width + (x - 1)] *= -1.0f;
								}
							}
							else
								stack.pushBack(Range(center, r.mEnd, inside, r.mInsideEnd));
						}
					}
				}

				if (!d.signOnly)
				{
					for (PxU32 x = 0; x < d.width; ++x)
					{
						const PxU32 index = z * d.width * d.height + y * d.width + x;

						PxVec3 queryPoint = d.pointSampler->getPoint(x, y, z);

						ClosestDistanceToTrimeshTraversalController cd(d.indices, d.vertices, d.tree->begin());
						cd.setQueryPoint(queryPoint);

						if (lastTriangle != -1)
						{
							//Warm-start the query with a lower-bound distance based on the triangle found by the previous query.
							//This helps to cull the tree traversal more effectively in the closest point query.
							PxU32 i0 = d.indices[3 * lastTriangle];
							PxU32 i1 = d.indices[3 * lastTriangle + 1];
							PxU32 i2 = d.indices[3 * lastTriangle + 2];

							PxReal s, t;

							const PxVec3 closest = Gu::closestPtPointTriangle(queryPoint, d.vertices[i0], d.vertices[i1], d.vertices[i2], s, t);
							PxReal d2 = (closest - queryPoint).magnitudeSquared();

							cd.setClosestStart(d2, lastTriangle, closest);
						}

						Gu::traverseBVH(d.tree->begin(), cd);
						PxVec3 closestPoint = cd.getClosestPoint();
						PxReal closestDistance = (closestPoint - queryPoint).magnitude();

						lastTriangle = cd.getClosestTriId();

						PxReal sign = 1.f;
						if (!d.optimizeInsideOutsideCalculation)
						{
							PxReal windingNumber = Gu::computeWindingNumber(d.tree->begin(), queryPoint, *d.clusters, d.indices, d.vertices);
							sign = windingNumber > 0.5f ? -1.f : 1.f;
						}

						d.sdf[index] *= closestDistance * sign;
						if (d.sampleLocations)
							d.sampleLocations[index] = queryPoint;
					}
				}
			}
			start = physx::PxAtomicAdd(d.progress, d.batchSize) - d.batchSize;
		}
		return NULL;
	}


	PxU32 idx(PxU32 x, PxU32 y, PxU32 z, PxU32 width, PxU32 height)
	{
		return z * (width + 1) * (height + 1) + y * (width + 1) + x;
	}

	PX_FORCE_INLINE PxU32 idxCompact(PxU32 x, PxU32 y, PxU32 z, PxU32 width, PxU32 height)
	{
		return z * (width) * (height)+y * (width)+x;
	}

	struct PxI32x3
	{
		PxI32x3(PxI32 x_, PxI32 y_, PxI32 z_) : x(x_), y(y_), z(z_)
		{}

		PxI32 x;
		PxI32 y;
		PxI32 z;
	};

	//Applies per pixel operations similar to the one uses by the fast marching methods to build SDFs out of binary image bitmaps
	//This allows to fill in correct distance values in regions where meshes habe holes
	struct PixelProcessor
	{
		PxVec3 mCellSize;
		PxI32 mWidth;
		PxI32 mHeight;
		PxI32 mDepth;

		PixelProcessor(PxVec3 cellSize, PxI32 width, PxI32 height, PxI32 depth) : 
			mCellSize(cellSize), mWidth(width), mHeight(height), mDepth(depth)
		{
		}

		//Estimates distance values near at mesh holes by estimating the location of the mesh surface. This can be done by analyzing
		//the sign change of the imperfect SDF. The signs are computed using winding numbers which are immune to meshes with holes.
		bool init(PxI32x3 p, const PxReal* sdf, PxReal& newValue) const
		{
			PxReal initialValue = sdf[idxCompact(p.x, p.y, p.z, mWidth, mHeight)];
			
			newValue = PxAbs(initialValue);
			
			for (PxI32 z = PxMax(0, p.z - 1); z <= PxMin(mDepth - 1, p.z + 1); ++z)
				for (PxI32 y = PxMax(0, p.y - 1); y <= PxMin(mHeight - 1, p.y + 1); ++y)
					for (PxI32 x = PxMax(0, p.x - 1); x <= PxMin(mWidth - 1, p.x + 1); ++x)
					{
						if (x == p.x && y == p.y && z == p.z)
							continue;

						PxReal value = sdf[idxCompact(x, y, z, mWidth, mHeight)];

						if (PxSign(initialValue) != PxSign(value))
						{
							PxReal distance = 0;
							if (x != p.x)
								distance += mCellSize.x*mCellSize.x;
							if (y != p.y)
								distance += mCellSize.y*mCellSize.y;
							if (z != p.z)
								distance += mCellSize.z*mCellSize.z;

							distance = PxSqrt(distance);

							PxReal delta = PxAbs(value - initialValue);
							
							if (0.99f*delta > distance)
							{
								PxReal scaling = distance / delta;
								PxReal v = 0.99f * scaling * initialValue;
								newValue = PxMin(newValue, PxAbs(v));
							}
						}
					}
			
			if (initialValue < 0)
				newValue = -newValue;

			if (newValue !=initialValue)
				return true;
			return false;
		}

		//Processes a pixel in a 3D sdf by applying the rule from the fast marching method. Only works on pixels with the same sign.
		bool process(PxI32x3 p, PxReal* sdf, PxReal& newValue) const
		{
			PxReal initialValue = sdf[idxCompact(p.x, p.y, p.z, mWidth, mHeight)];
			if (initialValue == 0.0f)
				return false;

			PxReal sign = PxSign(initialValue);
			newValue = PxAbs(initialValue);

			for (PxI32 z = PxMax(0, p.z - 1); z <= PxMin(mDepth - 1, p.z + 1); ++z)
				for (PxI32 y = PxMax(0, p.y - 1); y <= PxMin(mHeight - 1, p.y + 1); ++y)
					for (PxI32 x = PxMax(0, p.x - 1); x <= PxMin(mWidth - 1, p.x + 1); ++x)
					{
						if (x == p.x && y == p.y && z == p.z)
							continue;

						PxReal value = sdf[idxCompact(x, y, z, mWidth, mHeight)];

						if (sign == PxSign(value))
						{
							PxReal distance = 0;
							if (x != p.x)
								distance += mCellSize.x*mCellSize.x;
							if (y != p.y)
								distance += mCellSize.y*mCellSize.y;
							if (z != p.z)
								distance += mCellSize.z*mCellSize.z;

							distance = PxSqrt(distance);

							PxReal absValue = PxAbs(value);

							if(absValue + 1.01f*distance < newValue)
								newValue = absValue + distance;
						}
					}

			newValue = sign * newValue;
			if (newValue != initialValue)
			{
				sdf[idxCompact(p.x, p.y, p.z, mWidth, mHeight)] = newValue;
				return true;
			}
			return false;
		}
	};

	//Allows to store the new value of a SDF pixel to apply the change later. This avoids the need of double buffering the SDF data.
	struct Mutation
	{
		PxI32x3 mIndex;
		PxReal mNewValue;

		Mutation(const PxI32x3& index, PxReal newValue) : mIndex(index), mNewValue(newValue)
		{
		}
	};

	void applyMutations(PxArray<Mutation>& mutations, PxU32 start, PxU32 end, PxReal* sdfs, PxU32 width, PxU32 height)
	{
		for (PxU32 i = start; i < end; ++i)
		{
			Mutation m = mutations[i];
			sdfs[idxCompact(m.mIndex.x, m.mIndex.y, m.mIndex.z, width, height)] = m.mNewValue;
		}
	}

	//Approximates the solution of an Eikonal equation on a dense grid
	void fixSdfForNonClosedGeometry(PxU32 width, PxU32 height, PxU32 depth,
		PxReal* sdf, const PxVec3& cellSize)
	{
		PxArray<Mutation> mutations;

		PixelProcessor processor(cellSize, width, height, depth);

		for (PxU32 z = 0; z < depth; ++z)
			for (PxU32 y = 0; y < height; ++y)
				for (PxU32 x = 0; x < width; ++x)
				{
					//Process only cells where a sign change occurs
					PxReal newValue;
					if (processor.init(PxI32x3(x, y, z), sdf, newValue))
						mutations.pushBack(Mutation(PxI32x3(x, y, z), newValue));
				}

				
		applyMutations(mutations, 0, mutations.size(), sdf, width, height);

		PxU32 maxMutationLoops = 1000;
		PxU32 counter = 0;

		while (mutations.size() > 0 && counter  < maxMutationLoops)
		{
			PxU32 size = mutations.size();

			for (PxU32 i = 0; i < size; ++i)
			{
				PxI32x3 p = mutations[i].mIndex;

				//Process neighbors of item on stack
				for (PxI32 z = PxMax(0, p.z - 1); z <= PxMin(PxI32(depth) - 1, p.z + 1); ++z)
					for (PxI32 y = PxMax(0, p.y - 1); y <= PxMin(PxI32(height) - 1, p.y + 1); ++y)
						for (PxI32 x = PxMax(0, p.x - 1); x <= PxMin(PxI32(width) - 1, p.x + 1); ++x)
						{
							if (x == p.x && y == p.y && z == p.z)
								continue;
							PxReal newValue;
							if (processor.process(PxI32x3(x, y, z), sdf, newValue))
								mutations.pushBack(Mutation(PxI32x3(x, y, z), newValue));
						}
			}
			mutations.removeRange(0, size);
			++counter;
		}

		//For safety reasons: Check all cells again
		for (PxU32 z = 0; z < depth; ++z)
			for (PxU32 y = 0; y < height; ++y)
				for (PxU32 x = 0; x < width; ++x)
				{
					//Look at all neighbors
					PxReal newValue;
					if (processor.init(PxI32x3(x, y, z), sdf, newValue))
						mutations.pushBack(Mutation(PxI32x3(x, y, z), newValue));
				}

		counter = 0;
		while (mutations.size() > 0 && counter < maxMutationLoops)
		{
			PxU32 size = mutations.size();

			for (PxU32 i = 0; i < size; ++i)
			{
				PxI32x3 p = mutations[i].mIndex;

				//Process neighbors of item on stack
				for (PxI32 z = PxMax(0, p.z - 1); z <= PxMin(PxI32(depth) - 1, p.z + 1); ++z)
					for (PxI32 y = PxMax(0, p.y - 1); y <= PxMin(PxI32(height) - 1, p.y + 1); ++y)
						for (PxI32 x = PxMax(0, p.x - 1); x <= PxMin(PxI32(width) - 1, p.x + 1); ++x)
						{
							if (x == p.x && y == p.y && z == p.z)
								continue;
							PxReal newValue;
							if (processor.process(PxI32x3(x, y, z), sdf, newValue))
								mutations.pushBack(Mutation(PxI32x3(x, y, z), newValue));
						}
			}
			mutations.removeRange(0, size);
			++counter;
		}
	}



	void SDFUsingWindingNumbers(PxArray<Gu::BVHNode>& tree, PxHashMap<PxU32, Gu::ClusterApproximation>& clusters, const PxVec3* vertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
		PxReal* sdf, GridQueryPointSampler& sampler, PxVec3* sampleLocations, PxU32 numThreads, bool isWatertight, bool allVerticesInsideSamplingBox)
	{
		bool optimizeInsideOutsideCalculation = allVerticesInsideSamplingBox && isWatertight;
		numThreads = PxMax(numThreads, 1u);

		PxI32 progress = 0;

		PxArray<PxThread*> threads;
		PxArray<SDFCalculationData> perThreadData;

		for (PxU32 i = 0; i < numThreads; ++i)
		{
			perThreadData.pushBack(SDFCalculationData());

			SDFCalculationData& d = perThreadData[i];
			d.vertices = vertices;
			d.indices = indices;
			d.numTriangleIndices = numTriangleIndices;
			d.width = width;
			d.height = height;
			d.depth = depth;
			d.sdf = sdf;
			d.sampleLocations = sampleLocations;
			d.optimizeInsideOutsideCalculation = optimizeInsideOutsideCalculation;


			d.pointSampler = &sampler;

			d.progress = &progress;

			d.tree = &tree;

			d.clusters = &clusters;

			d.end = depth * height;

			d.signOnly = false;
		}

		PxU32 l = width * height * depth;
		for (PxU32 i = 0; i < l; ++i)
			sdf[i] = 1.0f;

		for (PxU32 i = 0; i < numThreads; ++i)
		{
			if (perThreadData.size() == 1)
				computeSDFThreadJob(&perThreadData[i]);
			else
			{
				threads.pushBack(PX_NEW(PxThread)(computeSDFThreadJob, &perThreadData[i], "thread"));
				threads[i]->start();
			}
		}

		for (PxU32 i = 0; i < threads.size(); ++i)
		{
			threads[i]->waitForQuit();
		}

		for (PxU32 i = 0; i < threads.size(); ++i)
		{
			threads[i]->~PxThreadT();
			PX_FREE(threads[i]);
		}

		if (!isWatertight)
			fixSdfForNonClosedGeometry(width, height, depth, sdf, sampler.getActiveCellSize());
	}
	//Helper class to extract surface triangles from a tetmesh
	struct SortedTriangle
	{
	public:
		PxI32 mA;
		PxI32 mB;
		PxI32 mC;
		bool mFlipped;
		PX_FORCE_INLINE SortedTriangle(PxI32 a, PxI32 b, PxI32 c)
		{
			mA = a; mB = b; mC = c; mFlipped = false;
			if (mA > mB) { PxSwap(mA, mB); mFlipped = !mFlipped; }
			if (mB > mC) { PxSwap(mB, mC); mFlipped = !mFlipped; }
			if (mA > mB) { PxSwap(mA, mB); mFlipped = !mFlipped; }
		}
	};
	struct TriangleHash
	{
		PX_FORCE_INLINE std::size_t operator()(const SortedTriangle& k) const
		{
			return k.mA ^ k.mB ^ k.mC;
		}
		PX_FORCE_INLINE bool equal(const SortedTriangle& first, const SortedTriangle& second) const
		{
			return first.mA == second.mA && first.mB == second.mB && first.mC == second.mC;
		}
	};
	PxReal signedVolume(const PxVec3* points, const PxU32* triangles, PxU32 numTriangles, const PxU32* triangleSubset = NULL, PxU32 setLength = 0)
	{
		PxReal signedVolume = 0;
		const PxU32 l = triangleSubset ? setLength : numTriangles;
		for (PxU32 j = 0; j < l; ++j)
		{
			const PxU32 i = triangleSubset ? triangleSubset[j] : j;
			const PxU32* tri = &triangles[3 * i];
			PxVec3 a = points[tri[0]];
			PxVec3 b = points[tri[1]];
			PxVec3 c = points[tri[2]];

			PxReal y = a.dot(b.cross(c));
			signedVolume += y;
		}
		signedVolume *= (1.0f / 6.0f);
		return signedVolume;
	}

	void analyzeAndFixMesh(const PxVec3* vertices, const PxU32* indicesOrig, PxU32 numTriangleIndices, PxArray<PxU32>& repairedIndices)
	{
		const PxU32* indices = indicesOrig;
		PxI32 numVertices = -1;
		for (PxU32 i = 0; i < numTriangleIndices; ++i)
			numVertices = PxMax(numVertices, PxI32(indices[i]));
		++numVertices;
		//Check for duplicate vertices
		PxArray<PxI32> map;
		MeshAnalyzer::mapDuplicatePoints<PxVec3, PxReal>(vertices, PxU32(numVertices), map, 0.0f);
		bool hasDuplicateVertices = false;
		for (PxU32 i = 0; i < map.size(); ++i)
		{
			if (map[i] != PxI32(i))
			{
				hasDuplicateVertices = true;
				break;
			}
		}
		if (hasDuplicateVertices)
		{
			repairedIndices.resize(numTriangleIndices);
			for (PxU32 i = 0; i < numTriangleIndices; ++i)
				repairedIndices[i] = map[indices[i]];
			indices = repairedIndices.begin();
		}
		//Check for duplicate triangles
		PxHashMap<SortedTriangle, PxI32, TriangleHash> tris;
		bool hasDuplicateTriangles = false;
		for (PxU32 i = 0; i < numTriangleIndices; i += 3)
		{
			SortedTriangle tri(indices[i], indices[i + 1], indices[i + 2]);
			if (const PxPair<const SortedTriangle, PxI32>* ptr = tris.find(tri))
			{
				tris[tri] = ptr->second + 1;
				hasDuplicateTriangles = true;
			}
			else
				tris.insert(tri, 1);
		}
		if (hasDuplicateTriangles)
		{
			repairedIndices.clear();
			for (PxHashMap<SortedTriangle, PxI32, TriangleHash>::Iterator iter = tris.getIterator(); !iter.done(); ++iter)
			{
				repairedIndices.pushBack(iter->first.mA);
				if (iter->first.mFlipped)
				{
					repairedIndices.pushBack(iter->first.mC);
					repairedIndices.pushBack(iter->first.mB);
				}
				else
				{
					repairedIndices.pushBack(iter->first.mB);
					repairedIndices.pushBack(iter->first.mC);
				}
			}
		}
		else
		{
			if (!hasDuplicateVertices) //reqairedIndices is already initialized if hasDuplicateVertices is true
			{
				repairedIndices.resize(numTriangleIndices);
				for (PxU32 i = 0; i < numTriangleIndices; ++i)
					repairedIndices[i] = indices[i];
			}
		}
		PxHashMap<PxU64, PxI32> edges;
		PxArray<bool> flipTriangle;
		PxArray<PxArray<PxU32>> connectedTriangleGroups;
		Triangle* triangles = reinterpret_cast<Triangle*>(repairedIndices.begin());
		bool success = MeshAnalyzer::buildConsistentTriangleOrientationMap(triangles, repairedIndices.size() / 3, flipTriangle, edges, connectedTriangleGroups);
		bool meshIsWatertight = true;
		for (PxHashMap<PxU64, PxI32>::Iterator iter = edges.getIterator(); !iter.done(); ++iter)
		{
			if (iter->second != -1)
			{
				meshIsWatertight = false;
				break;
			}
		}
		if (success)
		{
			if (hasDuplicateTriangles && meshIsWatertight && connectedTriangleGroups.size() == 1)
			{
				for (PxU32 i = 0; i < flipTriangle.size(); ++i)
				{
					Triangle& t = triangles[i];
					if (flipTriangle[i])
						PxSwap(t[0], t[1]);
				}

				if (signedVolume(vertices, repairedIndices.begin(), repairedIndices.size() / 3) < 0.0f)
				{
					PxU32 numTriangles = repairedIndices.size() / 3;
					for (PxU32 j = 0; j < numTriangles; ++j)
					{
						PxU32* tri = &repairedIndices[j * 3];
						PxSwap(tri[1], tri[2]);
					}
				}
			}
		}
		else
		{
			//Here it is not possible to guarantee that the mesh fixing can succeed
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "SDF creation: Fixing of the input mesh topology not possible. The computed SDF might not work as expected. Please try to improve the mesh structure by e. g. applying remeshing.");
			//connectedTriangleGroups won't have any elements, so return
			return;
		}

		if (!meshIsWatertight)
		{
			PxGetFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "SDF creation: Input mesh is not watertight. The SDF will try to close the holes to its best knowledge.");
		}
	}

	void SDFUsingWindingNumbers(const PxVec3* vertices, const PxU32* indicesOrig, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
		PxReal* sdf, PxVec3 minExtents, PxVec3 maxExtents, PxVec3* sampleLocations, bool cellCenteredSamples, PxU32 numThreads)
	{
		PxArray<PxU32> repairedIndices;
		//Analyze the mesh to catch and fix some special cases
		//There are meshes where every triangle is present once with cw and once with ccw orientation. Try to filter out only one set
		analyzeAndFixMesh(vertices, indicesOrig, numTriangleIndices, repairedIndices);
		const PxU32* indices = repairedIndices.size() > 0 ? repairedIndices.begin() : indicesOrig;
		if (repairedIndices.size() > 0)
			numTriangleIndices = repairedIndices.size();

		PxArray<Gu::BVHNode> tree;
		buildTree(indices, numTriangleIndices / 3, vertices, tree);

		PxHashMap<PxU32, Gu::ClusterApproximation> clusters;
		Gu::precomputeClusterInformation(tree.begin(), indices, numTriangleIndices / 3, vertices, clusters);

		const PxVec3 extents(maxExtents - minExtents);
		GridQueryPointSampler sampler(minExtents, PxVec3(extents.x / width, extents.y / height, extents.z / depth), cellCenteredSamples);


		bool isWatertight = MeshAnalyzer::checkMeshWatertightness(reinterpret_cast<const Triangle*>(indices), numTriangleIndices / 3);
		bool allSamplesInsideBox = true;
		PxBounds3 box(minExtents, maxExtents);
		for (PxU32 i = 0; i < numTriangleIndices; ++i)
		{
			PxVec3 v = vertices[indices[i]];
			if (!box.contains(v))
			{
				allSamplesInsideBox = false;
				break;
			}
		}

		SDFUsingWindingNumbers(tree, clusters, vertices, indices, numTriangleIndices, width, height, depth, sdf, sampler, sampleLocations, numThreads, isWatertight, allSamplesInsideBox);
	}

	PX_FORCE_INLINE void getCellCoords(PxU32 numCellsX, PxU32 numCellsY, PxU32 cellNr, PxU32& x, PxU32& y, PxU32& z)
	{
		x = cellNr % numCellsX;
		cellNr /= numCellsX;
		y = cellNr % numCellsY;
		z = cellNr / numCellsY;
	}

	PX_FORCE_INLINE PxU32 encodeTriple(PxU32 x, PxU32 y, PxU32 z)
	{
		PX_ASSERT(x >= 0 && x < 1024);
		PX_ASSERT(y >= 0 && y < 1024);
		PX_ASSERT(z >= 0 && z < 1024);
		return (z << 20) | (y << 10) | x;
	}


	struct Interval
	{
		PxReal min;
		PxReal max;

		Interval() : min(FLT_MAX), max(-FLT_MAX)
		{}

		Interval(PxReal min_, PxReal max_) : min(min_), max(max_)
		{}

		bool overlaps(const Interval& i)
		{
			return  !(min > i.max || i.min > max);
		}
	};

	void convertSparseSDFTo3DTextureLayout(PxU32 width, PxU32 height, PxU32 depth, PxU32 cellsPerSubgrid,
		PxU32* sdfFineStartSlots, const PxReal* sdfFineSubgridsIn, PxU32 sdfFineSubgridsSize, PxArray<PxReal>& subgrids3DTexFormat,
		PxU32& numSubgridsX, PxU32& numSubgridsY, PxU32& numSubgridsZ)
	{
		PxU32 valuesPerSubgrid = (cellsPerSubgrid + 1)*(cellsPerSubgrid + 1)*(cellsPerSubgrid + 1);
		PX_ASSERT(sdfFineSubgridsSize % valuesPerSubgrid == 0);
		PxU32 numSubgrids = sdfFineSubgridsSize / valuesPerSubgrid;

		PxReal cubicRoot = PxPow(PxReal(numSubgrids), 1.0f / 3.0f);
		//PX_UNUSED(cubicRoot);
		PxU32 up = PxMax(1u, PxU32(PxCeil(cubicRoot)));

		PxU32 debug = numSubgrids;

		//Arrange numSubgrids in a 3d layout
		numSubgridsX = PxMin(up, numSubgrids);
		numSubgrids = (numSubgrids + up - 1) / up;
		numSubgridsY = PxMin(up, numSubgrids);
		numSubgrids = (numSubgrids + up - 1) / up;
		numSubgridsZ = PxMin(up, numSubgrids);


		PxU32 debug2 = numSubgridsX * numSubgridsY * numSubgridsZ;
		PX_ASSERT(debug2 >= debug);
		PX_UNUSED(debug);
		PX_UNUSED(debug2);

		PxU32 size = valuesPerSubgrid * numSubgridsX * numSubgridsY * numSubgridsZ;
		PxReal placeholder = 1234567;
		subgrids3DTexFormat.resize(/*sdfFineSubgridsSize*/size, placeholder);

		PxU32 w = width / cellsPerSubgrid;
		PxU32 h = height / cellsPerSubgrid;
		PxU32 d = depth / cellsPerSubgrid;
		PxU32 l = (w)*(h)*(d);
		for (PxU32 i = 0; i < l; ++i)
		{
			PxU32 startSlot = sdfFineStartSlots[i];
			if (startSlot != 0xFFFFFFFF)
			{
				PxU32 baseIndex = startSlot * (cellsPerSubgrid + 1) * (cellsPerSubgrid + 1) * (cellsPerSubgrid + 1);
				const PxReal* sdfFine = &sdfFineSubgridsIn[baseIndex];

				PxU32 startSlotX, startSlotY, startSlotZ;
				getCellCoords(numSubgridsX, numSubgridsY, startSlot, startSlotX, startSlotY, startSlotZ);

				sdfFineStartSlots[i] = encodeTriple(startSlotX, startSlotY, startSlotZ);

				for (PxU32 zLocal = 0; zLocal <= cellsPerSubgrid; ++zLocal)
				{
					for (PxU32 yLocal = 0; yLocal <= cellsPerSubgrid; ++yLocal)
					{
						for (PxU32 xLocal = 0; xLocal <= cellsPerSubgrid; ++xLocal)
						{
							PxReal sdfValue = sdfFine[idx(xLocal, yLocal, zLocal, cellsPerSubgrid, cellsPerSubgrid)];
							PxU32 index = idxCompact(xLocal + startSlotX * (cellsPerSubgrid + 1), yLocal + startSlotY * (cellsPerSubgrid + 1), zLocal + startSlotZ * (cellsPerSubgrid + 1),
								numSubgridsX * (cellsPerSubgrid + 1), numSubgridsY * (cellsPerSubgrid + 1));
							PX_ASSERT(subgrids3DTexFormat[index] == placeholder);
							subgrids3DTexFormat[index] = sdfValue;
							PX_ASSERT(PxIsFinite(sdfValue));
						}
					}
				}
			}
		}
	}
	

	PX_FORCE_INLINE PxReal lerp(PxReal a, PxReal b, float t)
	{
		return a + t * (b - a);
	}

	PX_FORCE_INLINE PxReal bilerp(
		PxReal f00,
		PxReal f10,
		PxReal f01,
		PxReal f11,
		PxReal tx, PxReal ty)
	{
		PxReal a = lerp(f00, f10, tx);
		PxReal b = lerp(f01, f11, tx);
		return lerp(
			a,
			b,
			ty);
	}

	PX_FORCE_INLINE PxReal trilerp(
		PxReal f000,
		PxReal f100,
		PxReal f010,
		PxReal f110,
		PxReal f001,
		PxReal f101,
		PxReal f011,
		PxReal f111,
		PxReal tx,
		PxReal ty,
		PxReal tz)
	{
		PxReal a = bilerp(f000, f100, f010, f110, tx, ty);
		PxReal b = bilerp(f001, f101, f011, f111, tx, ty);
		return lerp(
			a,
			b,
			tz);
	}

	template<typename T>
	class DenseSDF
	{
	public:
		PxU32 width, height, depth;
	private:
		T* sdf;

	public:
		DenseSDF(PxU32 width, PxU32 height, PxU32 depth, T* sdf)
		{
			initialize(width, height, depth, sdf);
		}

		DenseSDF() {}

		void initialize(PxU32 width_, PxU32 height_, PxU32 depth_, T* sdf_)
		{
			this->width = width_;
			this->height = height_;
			this->depth = depth_;
			this->sdf = sdf_;
		}

		PxU32 memoryConsumption()
		{
			return (width + 1) * (height + 1) * (depth + 1) * sizeof(T);
		}

		PxReal sampleSDFDirect(const PxVec3& samplePoint)
		{
			const PxU32 xBase = PxClamp(PxU32(samplePoint.x), 0u, width - 1);
			const PxU32 yBase = PxClamp(PxU32(samplePoint.y), 0u, height - 1);
			const PxU32 zBase = PxClamp(PxU32(samplePoint.z), 0u, depth - 1);

			return trilerp(
				sdf[idx(xBase, yBase, zBase, width, height)],
				sdf[idx(xBase + 1, yBase, zBase, width, height)],
				sdf[idx(xBase, yBase + 1, zBase, width, height)],
				sdf[idx(xBase + 1, yBase + 1, zBase, width, height)],
				sdf[idx(xBase, yBase, zBase + 1, width, height)],
				sdf[idx(xBase + 1, yBase, zBase + 1, width, height)],
				sdf[idx(xBase, yBase + 1, zBase + 1, width, height)],
				sdf[idx(xBase + 1, yBase + 1, zBase + 1, width, height)], samplePoint.x - xBase, samplePoint.y - yBase, samplePoint.z - zBase);
		}
	};

	void SDFUsingWindingNumbersSparse(const PxVec3* vertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
		const PxVec3& minExtents, const PxVec3& maxExtents, PxReal narrowBandThickness, PxU32 cellsPerSubgrid,
		PxArray<PxReal>& sdfCoarse, PxArray<PxU32>& sdfFineStartSlots, PxArray<PxReal>& subgridData, PxArray<PxReal>& denseSdf,
		PxReal& subgridsMinSdfValue, PxReal& subgridsMaxSdfValue, PxU32 numThreads)
	{
		PX_ASSERT(width % cellsPerSubgrid == 0);
		PX_ASSERT(height % cellsPerSubgrid == 0);
		PX_ASSERT(depth % cellsPerSubgrid == 0);

		const PxVec3 extents(maxExtents - minExtents);
		const PxVec3 delta(extents.x / width, extents.y / height, extents.z / depth);

		PxU32 w = width / cellsPerSubgrid;
		PxU32 h = height / cellsPerSubgrid;
		PxU32 d = depth / cellsPerSubgrid;

		denseSdf.resize((width + 1) * (height + 1) * (depth + 1));
		SDFUsingWindingNumbers(vertices, indices, numTriangleIndices, width + 1, height + 1, depth + 1, denseSdf.begin(), minExtents, maxExtents + delta, NULL, false, numThreads);

		sdfCoarse.clear();
		sdfFineStartSlots.clear();
		subgridData.clear();

		sdfCoarse.reserve((w + 1) * (h + 1) * (d + 1));
		sdfFineStartSlots.reserve(w * h * d);

		for (PxU32 zBlock = 0; zBlock < d; ++zBlock)		
			for (PxU32 yBlock = 0; yBlock < h; ++yBlock)			
				for (PxU32 xBlock = 0; xBlock < w; ++xBlock)
				{
					sdfFineStartSlots.pushBack(0xFFFFFFFF);
				}

		for (PxU32 zBlock = 0; zBlock <= d; ++zBlock)		
			for (PxU32 yBlock = 0; yBlock <= h; ++yBlock)			
				for (PxU32 xBlock = 0; xBlock <= w; ++xBlock)
				{
					PxU32 x = xBlock * cellsPerSubgrid;
					PxU32 y = yBlock * cellsPerSubgrid;
					PxU32 z = zBlock * cellsPerSubgrid;
					const PxU32 index = idx(x, y, z, width, height);
					PX_ASSERT(index < denseSdf.size());
					PxReal sdfValue = denseSdf[index];
					sdfCoarse.pushBack(sdfValue);					
				}

		
		for (PxU32 zBlock = 0; zBlock <= d; ++zBlock)
			for (PxU32 yBlock = 0; yBlock <= h; ++yBlock)
				for (PxU32 xBlock = 0; xBlock <= w; ++xBlock)
				{
					PxU32 x = xBlock * cellsPerSubgrid;
					PxU32 y = yBlock * cellsPerSubgrid;
					PxU32 z = zBlock * cellsPerSubgrid;

					const PxU32 index = idx(x, y, z, width, height);
					const PxU32 indexCoarse = idx(xBlock, yBlock, zBlock, w, h);
					PX_ASSERT(sdfCoarse[indexCoarse] == denseSdf[index]);
					PX_UNUSED(indexCoarse);
					PX_UNUSED(index);
				}


		Interval narrowBandInterval(-narrowBandThickness, narrowBandThickness);

		DenseSDF<PxReal> coarseEval(w, h, d, sdfCoarse.begin());
		PxReal s = 1.0f / cellsPerSubgrid;

		const PxReal errorThreshold = 1e-6f * extents.magnitude();
		subgridsMaxSdfValue = -FLT_MAX;
		subgridsMinSdfValue = FLT_MAX;
		for (PxU32 zBlock = 0; zBlock < d; ++zBlock)
		{
			for (PxU32 yBlock = 0; yBlock < h; ++yBlock)
			{
				for (PxU32 xBlock = 0; xBlock < w; ++xBlock)
				{
					bool subgridRequired = false;
					Interval inverval;
					PxReal maxAbsError = 0.0f;
					for (PxU32 zLocal = 0; zLocal <= cellsPerSubgrid; ++zLocal)
					{
						for (PxU32 yLocal = 0; yLocal <= cellsPerSubgrid; ++yLocal)
						{
							for (PxU32 xLocal = 0; xLocal <= cellsPerSubgrid; ++xLocal)
							{
								PxU32 x = xBlock * cellsPerSubgrid + xLocal;
								PxU32 y = yBlock * cellsPerSubgrid + yLocal;
								PxU32 z = zBlock * cellsPerSubgrid + zLocal;

								const PxU32 index = idx(x, y, z, width, height);
								PxReal sdfValue = denseSdf[index];								
								inverval.max = PxMax(inverval.max, sdfValue);
								inverval.min = PxMin(inverval.min, sdfValue);

								maxAbsError = PxMax(maxAbsError, PxAbs(sdfValue - coarseEval.sampleSDFDirect(PxVec3(xBlock + xLocal * s, yBlock + yLocal * s, zBlock + zLocal * s))));							
							}
						}
					}			

					subgridRequired = narrowBandInterval.overlaps(inverval);
					if (maxAbsError < errorThreshold)
						subgridRequired = false; //No need for a subgrid if the coarse SDF is already almost exact

					if (subgridRequired)
					{
						subgridsMaxSdfValue = PxMax(subgridsMaxSdfValue, inverval.max);
						subgridsMinSdfValue = PxMin(subgridsMinSdfValue, inverval.min);						
					}
				}
			}
		}

		
		PxU32 subgridIndexer = 0;
		for (PxU32 zBlock = 0; zBlock < d; ++zBlock)
		{
			for (PxU32 yBlock = 0; yBlock < h; ++yBlock)
			{
				for (PxU32 xBlock = 0; xBlock < w; ++xBlock)
				{
					bool subgridRequired = false;
					Interval inverval;
					PxReal maxAbsError = 0.0f;
					for (PxU32 zLocal = 0; zLocal <= cellsPerSubgrid; ++zLocal)
					{
						for (PxU32 yLocal = 0; yLocal <= cellsPerSubgrid; ++yLocal)
						{
							for (PxU32 xLocal = 0; xLocal <= cellsPerSubgrid; ++xLocal)
							{
								PxU32 x = xBlock * cellsPerSubgrid + xLocal;
								PxU32 y = yBlock * cellsPerSubgrid + yLocal;
								PxU32 z = zBlock * cellsPerSubgrid + zLocal;

								const PxU32 index = idx(x, y, z, width, height); // z * (width + 1) * (height + 1) + y * (width + 1) + x;
								PxReal sdfValue = denseSdf[index];
								inverval.max = PxMax(inverval.max, sdfValue);
								inverval.min = PxMin(inverval.min, sdfValue);

								maxAbsError = PxMax(maxAbsError, PxAbs(sdfValue - coarseEval.sampleSDFDirect(PxVec3(xBlock + xLocal * s, yBlock + yLocal * s, zBlock + zLocal * s))));
							}
						}
					}

					subgridRequired = narrowBandInterval.overlaps(inverval);
					if (maxAbsError < errorThreshold)
						subgridRequired = false; //No need for a subgrid if the coarse SDF is already almost exact

					//For debugging
					if (cellsPerSubgrid == 1)
						subgridRequired = false;

					//subgridRequired = true;

					if (subgridRequired)
					{
						for (PxU32 zLocal = 0; zLocal <= cellsPerSubgrid; ++zLocal)
						{
							for (PxU32 yLocal = 0; yLocal <= cellsPerSubgrid; ++yLocal)
							{
								for (PxU32 xLocal = 0; xLocal <= cellsPerSubgrid; ++xLocal)
								{
									PxU32 x = xBlock * cellsPerSubgrid + xLocal;
									PxU32 y = yBlock * cellsPerSubgrid + yLocal;
									PxU32 z = zBlock * cellsPerSubgrid + zLocal;

									const PxU32 index = z * (width + 1) * (height + 1) + y * (width + 1) + x;
									PxReal sdfValue = denseSdf[index];

									subgridData.pushBack(sdfValue);
								}
							}
						}
						sdfFineStartSlots[idxCompact(xBlock, yBlock, zBlock, w, h)] = subgridIndexer;
						++subgridIndexer;
					}
				}
			}
		}
	}
	

	PX_FORCE_INLINE void decodeTriple(PxU32 id, PxU32& x, PxU32& y, PxU32& z)
	{
		x = id & 0x000003FF;
		id = id >> 10;
		y = id & 0x000003FF;
		id = id >> 10;
		z = id & 0x000003FF;
	}

	PX_FORCE_INLINE PxReal decode(PxU8* data, PxU32 bytesPerSparsePixel, PxReal subgridsMinSdfValue, PxReal subgridsMaxSdfValue)
	{
		switch (bytesPerSparsePixel)
		{
		case 1:
			return PxReal(data[0]) * (1.0f / 255.0f) * (subgridsMaxSdfValue - subgridsMinSdfValue) + subgridsMinSdfValue;
		case 2:
		{
			PxU16* ptr = reinterpret_cast<PxU16*>(data);
			return PxReal(ptr[0]) * (1.0f / 65535.0f) * (subgridsMaxSdfValue - subgridsMinSdfValue) + subgridsMinSdfValue;
		}
		case 4:
			//If 4 bytes per subgrid pixel are available, then normal floats are used. No need to 
			//de-normalize integer values since the floats already contain real distance values
			PxReal* ptr = reinterpret_cast<PxReal*>(data);
			return ptr[0];
		}
		return 0;
	}

	PX_FORCE_INLINE PxReal decode(const Gu::SDF& sdf, PxI32 xx, PxI32 yy, PxI32 zz)
	{
		if (xx < 0 || yy < 0 || zz < 0 || xx > PxI32(sdf.mDims.x) || yy > PxI32(sdf.mDims.y) || zz > PxI32(sdf.mDims.z))
			return 1.0f; //Return a value >0 that counts as outside

		const PxU32 nbX = sdf.mDims.x / sdf.mSubgridSize;
		const PxU32 nbY = sdf.mDims.y / sdf.mSubgridSize;
		const PxU32 nbZ = sdf.mDims.z / sdf.mSubgridSize;

		PxU32 xBase = xx / sdf.mSubgridSize;
		PxU32 yBase = yy / sdf.mSubgridSize;
		PxU32 zBase = zz / sdf.mSubgridSize;

		PxU32 x = xx % sdf.mSubgridSize;
		PxU32 y = yy % sdf.mSubgridSize;
		PxU32 z = zz % sdf.mSubgridSize;

		if (xBase == nbX)
		{
			--xBase;
			x = sdf.mSubgridSize;
		}
		if (yBase == nbY)
		{
			--yBase;
			y = sdf.mSubgridSize;
		}
		if (zBase == nbZ)
		{
			--zBase;
			z = sdf.mSubgridSize;
		}

		PxU32 startId = sdf.mSubgridStartSlots[zBase * (nbX) * (nbY)+yBase * (nbX)+xBase];
		if (startId != 0xFFFFFFFFu)
		{
			decodeTriple(startId, xBase, yBase, zBase);
			xBase *= (sdf.mSubgridSize + 1);
			yBase *= (sdf.mSubgridSize + 1);
			zBase *= (sdf.mSubgridSize + 1);

			const PxU32 w = sdf.mSdfSubgrids3DTexBlockDim.x * (sdf.mSubgridSize + 1);
			const PxU32 h = sdf.mSdfSubgrids3DTexBlockDim.y * (sdf.mSubgridSize + 1);
			const PxU32 index = idxCompact(xBase + x, yBase + y, zBase + z, w, h);
			return decode(&sdf.mSubgridSdf[sdf.mBytesPerSparsePixel * index],
				sdf.mBytesPerSparsePixel, sdf.mSubgridsMinSdfValue, sdf.mSubgridsMaxSdfValue);
		}
		else
		{
			DenseSDF<PxReal> coarseEval(nbX, nbY, nbZ, sdf.mSdf);

			PxReal s = 1.0f / sdf.mSubgridSize;
			return coarseEval.sampleSDFDirect(PxVec3(xBase + x * s, yBase + y * s, zBase + z * s));
		}
	}

	PX_FORCE_INLINE PxU64 key(PxI32 xId, PxI32 yId, PxI32 zId)
	{
		const PxI32 offset = 1 << 19;
		return (PxU64(zId + offset) << 42) | (PxU64(yId + offset) << 21) | (PxU64(xId + offset) << 0);
	}

	const PxI32 offsets[3][3][3] = { { {0,-1,0}, {0,-1,-1}, {0,0,-1} },
									 { {0,0,-1}, {-1,0,-1}, {-1,0,0} } ,
									 { {-1,0,0}, {-1,-1,0}, {0,-1,0} } };

	const PxI32 projections[3][2] = { {1, 2}, {2, 0}, {0, 1} };

	PX_FORCE_INLINE PxReal dirSign(PxI32 principalDirection, const PxVec3& start, const PxVec3& middle, const PxVec3& end)
	{
		PxReal a0 = middle[projections[principalDirection][0]] - start[projections[principalDirection][0]];
		PxReal a1 = middle[projections[principalDirection][1]] - start[projections[principalDirection][1]];

		PxReal b0 = end[projections[principalDirection][0]] - middle[projections[principalDirection][0]];
		PxReal b1 = end[projections[principalDirection][1]] - middle[projections[principalDirection][1]];

		return a0 * b1 - a1 * b0;
	}

	PX_FORCE_INLINE PxI32 indexOfMostConcaveCorner(PxI32 principalDirection, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d)
	{
		PxReal minimum = 0;
		PxI32 result = -1;
		PxReal s = dirSign(principalDirection, a, b, c);
		if (s <= minimum)
		{
			minimum = s;
			result = 1;
		}
		s = dirSign(principalDirection, b, c, d);
		if (s <= minimum)
		{
			minimum = s;
			result = 2;
		}
		s = dirSign(principalDirection, c, d, a);
		if (s <= minimum)
		{
			minimum = s;
			result = 3;
		}
		s = dirSign(principalDirection, d, a, b);
		if (s <= minimum)
		{
			minimum = s;
			result = 0;
		}
		return result;
	}

	bool generatePointInCell(const Gu::SDF& sdf, PxI32 x, PxI32 y, PxI32 z, PxVec3& point, PxReal corners[2][2][2])
	{
		const PxReal threshold = 0.0f;

		PxU32 positiveCounter = 0;
		PxU32 negativeCounter = 0;
		for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
		{
			PxReal v = corners[xx][yy][zz];
			if (v > 0)
				++positiveCounter;
			if (v < 0)
				++negativeCounter;
		}
		PxBounds3 box;
		box.minimum = sdf.mMeshLower + PxVec3(x * sdf.mSpacing, y * sdf.mSpacing, z * sdf.mSpacing);
		box.maximum = box.minimum + PxVec3(sdf.mSpacing);
		if (positiveCounter == 8 || negativeCounter == 8)
		{
			//Nothing to do because surface does not cross the current cell
		}
		else
		{
			//If point is not completely inside or outside, then find a point inside the cube that divides it into 8 cuboids
			PxU32 counter = 0;
			PxVec3 sum(0.0f);
			for (PxI32 a = 0; a <= 1; ++a) for (PxI32 b = 0; b <= 1; ++b)
			{
				PxReal p = corners[a][b][0];
				PxReal q = corners[a][b][1];

				if ((p <= threshold && q >= threshold) || (q <= threshold && p >= threshold))
				{
					PxReal t = (q != p) ? PxClamp((threshold - p) / (q - p), 0.0f, 1.0f) : 0.5f;
					sum += PxVec3(PxReal(a), PxReal(b), t);
					++counter;
				}
			}
			for (PxI32 a = 0; a <= 1; ++a) for (PxI32 b = 0; b <= 1; ++b)
			{
				PxReal p = corners[b][0][a];
				PxReal q = corners[b][1][a];

				if ((p <= threshold && q >= threshold) || (q <= threshold && p >= threshold))
				{
					PxReal t = (q != p) ? PxClamp((threshold - p) / (q - p), 0.0f, 1.0f) : 0.5f;
					sum += PxVec3(PxReal(b), t, PxReal(a));
					++counter;
				}
			}
			for (PxI32 a = 0; a <= 1; ++a) for (PxI32 b = 0; b <= 1; ++b)
			{
				PxReal p = corners[0][a][b];
				PxReal q = corners[1][a][b];

				if ((p <= threshold && q >= threshold) || (q <= threshold && p >= threshold))
				{
					PxReal t = (q != p) ? PxClamp((threshold - p) / (q - p), 0.0f, 1.0f) : 0.5f;
					sum += PxVec3(t, PxReal(a), PxReal(b));
					++counter;
				}
			}
			if (counter > 0)
			{
				point = box.minimum + sum * (sdf.mSpacing / counter);
				return true;
			}
		}

		return false;
	}

	PX_FORCE_INLINE bool generatePointInCell(const Gu::SDF& sdf, PxI32 x, PxI32 y, PxI32 z, PxVec3& point)
	{
		PxReal corners[2][2][2];
		for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
		{
			PxReal v = decode(sdf, x + xx, y + yy, z + zz);
			corners[xx][yy][zz] = v;
		}
		return generatePointInCell(sdf, x, y, z, point, corners);
	}

	PX_FORCE_INLINE bool generatePointInCellUsingCache(const Gu::SDF& sdf, PxI32 xBase, PxI32 yBase, PxI32 zBase, PxI32 x, PxI32 y, PxI32 z, PxVec3& point, const PxArray<PxReal>& cache)
	{
		PxReal corners[2][2][2];
		for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
		{
			PxReal v = cache[idxCompact(x + xx, y + yy, z + zz, sdf.mSubgridSize + 1, sdf.mSubgridSize + 1)];
			corners[xx][yy][zz] = v;
		}
		return generatePointInCell(sdf, xBase * sdf.mSubgridSize + x, yBase * sdf.mSubgridSize + y, zBase * sdf.mSubgridSize + z, point, corners);
	}

	PX_FORCE_INLINE PxReal getDenseSDFValue(const Gu::SDF& sdf, PxI32 x, PxI32 y, PxI32 z)
	{
		if (x < 0 || y < 0 || z < 0 || x >= PxI32(sdf.mDims.x) || y >= PxI32(sdf.mDims.y) || z >= PxI32(sdf.mDims.z))
			return 1.0; //Return a value >0 that counts as outside

		return sdf.mSdf[idxCompact(x, y, z, sdf.mDims.x, sdf.mDims.y)];
	}

	PX_FORCE_INLINE bool generatePointInCellDense(const Gu::SDF& sdf, PxI32 x, PxI32 y, PxI32 z, PxVec3& point)
	{
		PxReal corners[2][2][2];
		for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
		{
			PxReal v = getDenseSDFValue(sdf, x + xx, y + yy, z + zz);
			corners[xx][yy][zz] = v;
		}
		return generatePointInCell(sdf, x, y, z, point, corners);
	}

	PX_FORCE_INLINE bool canSkipSubgrid(const Gu::SDF& sdf, PxI32 i, PxI32 j, PxI32 k)
	{
		const PxReal t = 0.1f * sdf.mSpacing;
		const PxI32 nbX = sdf.mDims.x / sdf.mSubgridSize;
		const PxI32 nbY = sdf.mDims.y / sdf.mSubgridSize;
		const PxI32 nbZ = sdf.mDims.z / sdf.mSubgridSize;

		if (i < 0 || j < 0 || k < 0 || i >= nbX || j >= nbY || k >= nbZ)
			return false;

		if (sdf.mSubgridStartSlots[k * (nbX) * (nbY)+j * (nbX)+i] == 0xFFFFFFFFu)
		{
			PxU32 positiveCounter = 0;
			PxU32 negativeCounter = 0;
			for (PxI32 xx = 0; xx <= 1; ++xx) for (PxI32 yy = 0; yy <= 1; ++yy) for (PxI32 zz = 0; zz <= 1; ++zz)
			{
				PxReal v = decode(sdf, (i + xx)* sdf.mSubgridSize, (j + yy) * sdf.mSubgridSize, (k + zz) * sdf.mSubgridSize);
				if (v > t)
					++positiveCounter;
				if (v < t)
					++negativeCounter;
			}
			if (positiveCounter == 8 || negativeCounter == 8)
				return true;
		}
		return false;
	}

	void createTriangles(PxI32 xId, PxI32 yId, PxI32 zId, PxReal d0, PxReal ds[3], const PxHashMap<PxU64, PxU32>& cellToPoint, const PxArray<PxVec3>& points, PxArray<PxU32>& triangleIndices)
	{
		bool flipTriangleOrientation = false;
		const PxReal threshold = 0.0f;

		PxI32 num = 0;
		for (PxI32 dim = 0; dim < 3; dim++)
		{
			PxReal d = ds[dim];
			if ((d0 <= threshold && d >= threshold) || (d <= threshold && d0 >= threshold))
				num++;
		}
		if (num == 0)
			return;


		PxI32 buffer[4];
		const PxPair<const PxU64, PxU32>* f = cellToPoint.find(key(xId, yId, zId));
		if (!f)
			return;

		buffer[0] = f->second;
		PxVec3 v0 = points[buffer[0]];

		for (PxI32 dim = 0; dim < 3; dim++)
		{
			PxReal d = ds[dim];
			bool b1 = d0 <= threshold && d >= threshold;
			bool b2 = d <= threshold && d0 >= threshold;
			if (b1 || b2)
			{
				bool flip = flipTriangleOrientation == b1;
				bool skip = false;

				for (PxI32 ii = 0; ii < 3; ++ii)
				{
					f = cellToPoint.find(key(xId + offsets[dim][ii][0], yId + offsets[dim][ii][1], zId + offsets[dim][ii][2]));
					if (f)
						buffer[ii + 1] = f->second;
					else
						skip = true;
				}
				if (skip)
					continue;

				PxI32 shift = PxMax(0, indexOfMostConcaveCorner(dim, v0, points[buffer[1]], points[buffer[2]], points[buffer[3]])) % 2;

				//Split the quad into two triangles
				for (PxI32 ii = 0; ii < 2; ++ii)
				{
					triangleIndices.pushBack(buffer[shift]);
					if (flip)
					{
						for (PxI32 jj = 2; jj >= 1; --jj)
							triangleIndices.pushBack(buffer[(ii + jj + shift) % 4]);
					}
					else
					{
						for (PxI32 jj = 1; jj < 3; ++jj)
							triangleIndices.pushBack(buffer[(ii + jj + shift) % 4]);
					}
				}
			}
		}
	}


	void extractIsosurfaceFromSDF(const Gu::SDF& sdf, PxArray<PxVec3>& isosurfaceVertices, PxArray<PxU32>& isosurfaceTriangleIndices)
	{
		isosurfaceVertices.clear();
		isosurfaceTriangleIndices.clear();

		const PxI32 nbX = sdf.mDims.x / PxMax(1u, sdf.mSubgridSize);
		const PxI32 nbY = sdf.mDims.y / PxMax(1u, sdf.mSubgridSize);
		const PxI32 nbZ = sdf.mDims.z / PxMax(1u, sdf.mSubgridSize);

		PxHashMap<PxU64, PxU32> cellToPoint;

		if (sdf.mSubgridSize == 0)
		{
			//Dense SDF
			for (PxI32 k = -1; k <= nbZ; ++k)
				for (PxI32 j = -1; j <= nbY; ++j)
					for (PxI32 i = -1; i <= nbX; ++i)
					{
						PxVec3 p;
						if (generatePointInCellDense(sdf, i, j, k, p))
						{
							cellToPoint.insert(key(i, j, k), isosurfaceVertices.size());
							isosurfaceVertices.pushBack(p);
						}
					}
		}
		else
		{
			PxArray<PxReal> sdfCache;
			sdfCache.resize((sdf.mSubgridSize + 1) * (sdf.mSubgridSize + 1) * (sdf.mSubgridSize + 1));

			for (PxI32 k = -1; k <= nbZ; ++k)
			{
				for (PxI32 j = -1; j <= nbY; ++j)
				{
					for (PxI32 i = -1; i <= nbX; ++i)
					{
						if (canSkipSubgrid(sdf, i, j, k))
							continue;

						for (PxU32 z = 0; z <= sdf.mSubgridSize; ++z)
							for (PxU32 y = 0; y <= sdf.mSubgridSize; ++y)
								for (PxU32 x = 0; x <= sdf.mSubgridSize; ++x)
									sdfCache[idxCompact(x, y, z, sdf.mSubgridSize + 1, sdf.mSubgridSize + 1)] = decode(sdf, i * sdf.mSubgridSize + x, j * sdf.mSubgridSize + y, k * sdf.mSubgridSize + z);

						//Process the subgrid
						for (PxU32 z = 0; z < sdf.mSubgridSize; ++z)
						{
							for (PxU32 y = 0; y < sdf.mSubgridSize; ++y)
							{
								for (PxU32 x = 0; x < sdf.mSubgridSize; ++x)
								{
									PxVec3 p;
									PxU32 xId = i * sdf.mSubgridSize + x;
									PxU32 yId = j * sdf.mSubgridSize + y;
									PxU32 zId = k * sdf.mSubgridSize + z;
									if (generatePointInCellUsingCache(sdf, i, j, k, x, y, z, p, sdfCache))
									{
										cellToPoint.insert(key(xId, yId, zId), isosurfaceVertices.size());
										isosurfaceVertices.pushBack(p);
									}
								}
							}
						}
					}
				}
			}
		}

		if (sdf.mSubgridSize == 0)
		{
			for (PxI32 k = -1; k <= nbZ; ++k)
				for (PxI32 j = -1; j <= nbY; ++j)
					for (PxI32 i = -1; i <= nbX; ++i)
					{
						PxReal d0 = getDenseSDFValue(sdf, i, j, k);
						PxReal ds[3];
						ds[0] = getDenseSDFValue(sdf, i + 1, j, k);
						ds[1] = getDenseSDFValue(sdf, i, j + 1, k);
						ds[2] = getDenseSDFValue(sdf, i, j, k + 1);

						createTriangles(i, j, k, d0, ds, cellToPoint, isosurfaceVertices, isosurfaceTriangleIndices);
					}
		}
		else
		{
			for (PxI32 k = -1; k <= nbZ; ++k)
			{
				for (PxI32 j = -1; j <= nbY; ++j)
				{
					for (PxI32 i = -1; i <= nbX; ++i)
					{
						if (canSkipSubgrid(sdf, i, j, k))
							continue;

						//Process the subgrid
						for (PxU32 z = 0; z < sdf.mSubgridSize; ++z)
						{
							for (PxU32 y = 0; y < sdf.mSubgridSize; ++y)
							{
								for (PxU32 x = 0; x < sdf.mSubgridSize; ++x)
								{
									PxReal d0 = decode(sdf, i * sdf.mSubgridSize + x, j * sdf.mSubgridSize + y, k * sdf.mSubgridSize + z);
									PxReal ds[3];
									ds[0] = decode(sdf, i * sdf.mSubgridSize + x + 1, j * sdf.mSubgridSize + y, k * sdf.mSubgridSize + z);
									ds[1] = decode(sdf, i * sdf.mSubgridSize + x, j * sdf.mSubgridSize + y + 1, k * sdf.mSubgridSize + z);
									ds[2] = decode(sdf, i * sdf.mSubgridSize + x, j * sdf.mSubgridSize + y, k * sdf.mSubgridSize + z + 1);

									createTriangles(x + i * sdf.mSubgridSize, y + j * sdf.mSubgridSize, z + k * sdf.mSubgridSize, d0, ds, cellToPoint, isosurfaceVertices, isosurfaceTriangleIndices);
								}
							}
						}
					}
				}
			}
		}
	}
}

}
