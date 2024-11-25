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

#ifndef GU_AABBTREE_H
#define GU_AABBTREE_H

#include "foundation/PxMemory.h"
#include "foundation/PxArray.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxUserAllocated.h"
#include "common/PxPhysXCommonConfig.h"
#include "GuPrunerTypedef.h"
#include "GuAABBTreeQuery.h"
#include "GuDistancePointTriangle.h"
#include "GuDistancePointTetrahedron.h"

namespace physx
{
namespace Gu
{
	struct BVHNode;
	struct SAH_Buffers;
	class NodeAllocator;
	struct BuildStats;
	class AABBTreeBounds;

	// PT: TODO: sometimes we export member functions, sometimes we export the whole class. What's the story here?

#if PX_VC
#pragma warning(push)
#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif
	//! Contains AABB-tree build parameters
	class PX_PHYSX_COMMON_API AABBTreeBuildParams : public PxUserAllocated
	{
	public:
								AABBTreeBuildParams(PxU32 limit = 1, PxU32 nb_prims = 0, const AABBTreeBounds* bounds = NULL, BVHBuildStrategy bs = BVH_SPLATTER_POINTS) :
									mLimit			(limit),
									mNbPrimitives	(nb_prims),
									mBounds			(bounds),
									mCache			(NULL),
									mBuildStrategy	(bs)
								{
								}
								~AABBTreeBuildParams()
								{
									reset();
								}

		PX_FORCE_INLINE	void	reset()
								{
									mLimit = mNbPrimitives = 0;
									mBounds = NULL;
									PX_FREE(mCache);
								}

		PxU32					mLimit;			//!< Limit number of primitives / node. If limit is 1, build a complete tree (2*N-1 nodes)
		PxU32					mNbPrimitives;	//!< Number of (source) primitives.
		const AABBTreeBounds*	mBounds;		//!< Shortcut to an app-controlled array of AABBs.
		mutable PxVec3*			mCache;			//!< Cache for AABB centers - managed by build code.
		BVHBuildStrategy		mBuildStrategy;
	};

	//! AABB tree node used for building
	class PX_PHYSX_COMMON_API AABBTreeBuildNode : public PxUserAllocated
	{
	public:
		PX_FORCE_INLINE								AABBTreeBuildNode()		{}
		PX_FORCE_INLINE								~AABBTreeBuildNode()	{}

		PX_FORCE_INLINE	const PxBounds3&			getAABB()							const	{ return mBV;				}
		PX_FORCE_INLINE	const AABBTreeBuildNode*	getPos()							const	{ return mPos;				}
		PX_FORCE_INLINE	const AABBTreeBuildNode*	getNeg()							const	{ const AABBTreeBuildNode* P = mPos; return P ? P + 1 : NULL; }

		PX_FORCE_INLINE	bool						isLeaf()							const	{ return !getPos();			}

						PxBounds3					mBV;			//!< Global bounding-volume enclosing all the node-related primitives
						const AABBTreeBuildNode*	mPos;			//!< "Positive" & "Negative" children

						PxU32						mNodeIndex;		//!< Index of node-related primitives (in the tree's mIndices array)
						PxU32						mNbPrimitives;	//!< Number of primitives for this node

		PX_FORCE_INLINE	PxU32						getNbPrimitives()					const	{ return mNbPrimitives;		}

		PX_FORCE_INLINE	PxU32						getNbRuntimePrimitives()			const	{ return mNbPrimitives;		}
		PX_FORCE_INLINE void						setNbRunTimePrimitives(PxU32 val)			{ mNbPrimitives = val;		}
		PX_FORCE_INLINE	const PxU32*				getPrimitives(const PxU32* base)	const	{ return base + mNodeIndex; }
		PX_FORCE_INLINE	PxU32*						getPrimitives(PxU32* base)					{ return base + mNodeIndex; }

						void						subdivide(const AABBTreeBuildParams& params, BuildStats& stats, NodeAllocator& allocator, PxU32* const indices);
						void						subdivideSAH(const AABBTreeBuildParams& params, SAH_Buffers& sah, BuildStats& stats, NodeAllocator& allocator, PxU32* const indices);
	};

	//! For complete trees we can predict the final number of nodes and preallocate them. For incomplete trees we can't.
	//! But we don't want to allocate nodes one by one (which would be quite slow), so we use this helper class to
	//! allocate N nodes at once, while minimizing the amount of nodes allocated for nothing. An initial amount of
	//! nodes is estimated using the max number for a complete tree, and the user-defined number of primitives per leaf.
	//! In ideal cases this estimated number will be quite close to the final number of nodes. When that number is not
	//! enough though, slabs of N=1024 extra nodes are allocated until the build is complete.
	class PX_PHYSX_COMMON_API NodeAllocator : public PxUserAllocated
	{
	public:
									NodeAllocator();
									~NodeAllocator();

		void						release();
		void						init(PxU32 nbPrimitives, PxU32 limit);
		AABBTreeBuildNode*			getBiNode();

		AABBTreeBuildNode*			mPool;

		struct Slab
		{
			PX_FORCE_INLINE			Slab() {}
			PX_FORCE_INLINE			Slab(AABBTreeBuildNode* pool, PxU32 nbUsedNodes, PxU32 maxNbNodes) : mPool(pool), mNbUsedNodes(nbUsedNodes), mMaxNbNodes(maxNbNodes) {}
			AABBTreeBuildNode*		mPool;
			PxU32					mNbUsedNodes;
			PxU32					mMaxNbNodes;
		};
		PxArray<Slab>				mSlabs;
		PxU32						mCurrentSlabIndex;
		PxU32						mTotalNbNodes;
	};
#if PX_VC 
#pragma warning(pop) 
#endif

	/*
	*	\brief		Builds AABBtree from given parameters.
	*	\param		params				[in/out]	AABBTree build params
	*	\param		nodeAllocator		[in/out]	Node allocator
	*	\param		stats				[out]		Statistics
	*	\return		Indices buffer allocated during build, or NULL if failed
	*/
	PX_PHYSX_COMMON_API	PxU32* 	buildAABBTree(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, BuildStats& stats);

	// PT: TODO: explain how users should call these functions and maybe revisit this
	PX_PHYSX_COMMON_API	void	flattenTree(const NodeAllocator& nodeAllocator, BVHNode* dest, const PxU32* remap = NULL);

	PX_PHYSX_COMMON_API	void	buildAABBTree(PxU32 nbBounds, const AABBTreeBounds& bounds, PxArray<BVHNode>& tree);

	PxU32 reshuffle(PxU32 nb, PxU32* const PX_RESTRICT prims, const PxVec3* PX_RESTRICT centers, float splitValue, PxU32 axis);

	class BitArray
	{
		public:
										BitArray() : mBits(NULL), mSize(0)	{}
										BitArray(PxU32 nb_bits)				{ init(nb_bits);	}
										~BitArray()							{ PX_FREE(mBits);	}

						bool			init(PxU32 nb_bits);

		// Data management
		PX_FORCE_INLINE	void			setBit(PxU32 bit_number)
										{
											mBits[bit_number>>5] |= 1<<(bit_number&31);
										}
		PX_FORCE_INLINE	void			clearBit(PxU32 bit_number)
										{
											mBits[bit_number>>5] &= ~(1<<(bit_number&31));
										}
		PX_FORCE_INLINE	void			toggleBit(PxU32 bit_number)
										{
											mBits[bit_number>>5] ^= 1<<(bit_number&31);
										}

		PX_FORCE_INLINE	void			clearAll()			{ PxMemZero(mBits, mSize*4);		}
		PX_FORCE_INLINE	void			setAll()			{ PxMemSet(mBits, 0xff, mSize*4);	}

						void			resize(PxU32 maxBitNumber);

		// Data access
		PX_FORCE_INLINE	PxIntBool		isSet(PxU32 bit_number)	const
										{
											return PxIntBool(mBits[bit_number>>5] & (1<<(bit_number&31)));
										}

		PX_FORCE_INLINE	const PxU32*	getBits()	const	{ return mBits;		}
		PX_FORCE_INLINE	PxU32			getSize()	const	{ return mSize;		}

		protected:
						PxU32*			mBits;		//!< Array of bits
						PxU32			mSize;		//!< Size of the array in dwords
	};

	//! Contains AABB-tree merge parameters
	class AABBTreeMergeData
	{
	public:
		AABBTreeMergeData(PxU32 nbNodes, const BVHNode* nodes, PxU32 nbIndices, const PxU32* indices, PxU32 indicesOffset) :
			mNbNodes(nbNodes), mNodes(nodes), mNbIndices(nbIndices), mIndices(indices), mIndicesOffset(indicesOffset)
		{
		}

		~AABBTreeMergeData()		{}

		PX_FORCE_INLINE const BVHNode& getRootNode() const { return *mNodes; }

	public:
		PxU32			mNbNodes;		//!< Number of nodes of AABB tree merge
		const BVHNode*	mNodes;			//!< Nodes of AABB tree merge

		PxU32			mNbIndices;		//!< Number of indices of AABB tree merge
		const PxU32*	mIndices;		//!< Indices of AABB tree merge

		PxU32			mIndicesOffset;	//!< Indices offset from pruning pool
	};

	// Progressive building
	class FIFOStack;
	//~Progressive building

	// PT: base class used to share some data and code between Gu::AABBtree and Gu::BVH. This is WIP and subject to change.
	// Design dictated by refactoring necessities rather than a grand vision of something.
	class BVHCoreData : public PxUserAllocated
	{
		public:
												BVHCoreData() : mNbIndices(0), mNbNodes(0), mNodes(NULL), mIndices(NULL)	{}

		PX_FORCE_INLINE			PxU32			getNbIndices()		const	{ return mNbIndices;	}
		PX_FORCE_INLINE			const PxU32*	getIndices()		const	{ return mIndices;		}
		PX_FORCE_INLINE			PxU32*			getIndices()				{ return mIndices;		}
		PX_FORCE_INLINE			void			setIndices(PxU32* indices)	{ mIndices = indices;	}

		PX_FORCE_INLINE			PxU32			getNbNodes()		const	{ return mNbNodes;		}
		PX_FORCE_INLINE			const BVHNode*	getNodes()			const	{ return mNodes;		}
		PX_FORCE_INLINE			BVHNode*		getNodes()					{ return mNodes;		}

		PX_PHYSX_COMMON_API		void			fullRefit(const PxBounds3* boxes);

		// PT: I'm leaving the above accessors here to avoid refactoring the SQ code using them, but members became public.
								PxU32			mNbIndices;	//!< Nb indices
								PxU32			mNbNodes;	//!< Number of nodes in the tree.
								BVHNode*		mNodes;		//!< Linear pool of nodes.
								PxU32*			mIndices;	//!< Indices in the app list. Indices are reorganized during build (permutation).
	};

	class BVHPartialRefitData : public BVHCoreData
	{
		public:
		PX_PHYSX_COMMON_API						BVHPartialRefitData();
		PX_PHYSX_COMMON_API						~BVHPartialRefitData();

		PX_PHYSX_COMMON_API		void			releasePartialRefitData(bool clearRefitMap);
		// adds node[index] to a list of nodes to refit when refitMarkedNodes is called
		// Note that this includes updating the hierarchy up the chain
		PX_PHYSX_COMMON_API		void			markNodeForRefit(TreeNodeIndex nodeIndex);
		PX_PHYSX_COMMON_API		void			refitMarkedNodes(const PxBounds3* boxes);

		PX_FORCE_INLINE			PxU32*			getUpdateMap()	{ return mUpdateMap;	}

		protected:
								PxU32*			mParentIndices;		//!< PT: hot/cold split, keep parent data in separate array
								PxU32*			mUpdateMap;			//!< PT: Local index to tree node index
								BitArray		mRefitBitmask;		//!< bit is set for each node index in markForRefit
								PxU32			mRefitHighestSetWord;

								PxU32*			getParentIndices();
		public:
								void			createUpdateMap(PxU32 nbObjects);
	};

	//! AABB-tree, N primitives/leaf
	// PT: TODO: each PX_PHYSX_COMMON_API is a cross-DLL call, should we split that class in Gu/Sq parts to minimize this?
	class AABBTree : public BVHPartialRefitData
	{
		public:
		PX_PHYSX_COMMON_API						AABBTree();													
		PX_PHYSX_COMMON_API						~AABBTree();
		// Build
		PX_PHYSX_COMMON_API		bool			build(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator);
		// Progressive building
		PX_PHYSX_COMMON_API		PxU32			progressiveBuild(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, BuildStats& stats, PxU32 progress, PxU32 limit);
		//~Progressive building
		PX_PHYSX_COMMON_API		void			release(bool clearRefitMap=true);

		// Merge tree with another one
		PX_PHYSX_COMMON_API		void			mergeTree(const AABBTreeMergeData& tree);
		// Initialize tree from given merge data
		PX_PHYSX_COMMON_API		void			initTree(const AABBTreeMergeData& tree);

		// Data access
		PX_FORCE_INLINE			PxU32			getTotalPrims()		const	{ return mTotalPrims;	}

		PX_PHYSX_COMMON_API		void			shiftOrigin(const PxVec3& shift);

		// Shift indices of the tree by offset. Used for merged trees, when initial indices needs to be shifted to match indices in current pruning pool
		PX_PHYSX_COMMON_API		void			shiftIndices(PxU32 offset);
				
#if PX_DEBUG
								void			validate()	{}
#endif
		private:
								PxU32			mTotalPrims;		//!< Copy of final BuildStats::mTotalPrims
		// Progressive building
								FIFOStack*		mStack;
		//~Progressive building
								bool			buildInit(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, BuildStats& stats);
								void			buildEnd(const AABBTreeBuildParams& params, NodeAllocator& nodeAllocator, const BuildStats& stats);
		// tree merge
								void			mergeRuntimeNode(BVHNode& targetNode, const AABBTreeMergeData& tree, PxU32 targetNodeIndex);
								void			mergeRuntimeLeaf(BVHNode& targetNode, const AABBTreeMergeData& tree, PxU32 targetNodeIndex);
								void			addRuntimeChilds(PxU32& nodeIndex, const AABBTreeMergeData& tree);
								void			traverseRuntimeNode(BVHNode& targetNode, const AABBTreeMergeData& tree, PxU32 nodeIndex);
	};



	struct TinyBVH
	{
		PxArray<Gu::BVHNode> mTree;

		PX_PHYSX_COMMON_API static void constructFromTriangles(const PxU32* triangles, const PxU32 numTriangles, const PxVec3* points,
			TinyBVH& result, PxF32 enlargement = 1e-4f);

		PX_PHYSX_COMMON_API static void constructFromTetrahedra(const PxU32* tetrahedra, const PxU32 numTetrahedra, const PxVec3* points,
			TinyBVH& result, PxF32 enlargement = 1e-4f);

		template<typename T>
		void Traverse(T& traversalController, PxI32 rootNodeIndex = 0)
		{
			Gu::traverseBVH(mTree.begin(), traversalController, rootNodeIndex);
		}
	};

	class ClosestDistanceToTetmeshTraversalController
	{
	private:
		PxReal mClosestDistanceSquared;
		const PxU32* mTetrahedra;
		const PxVec3* mPoints;
		const Gu::BVHNode* mNodes;
		PxVec3 mQueryPoint;
		PxVec3 mClosestPoint;
		PxI32 mClosestTetId;

	public:
		PX_FORCE_INLINE ClosestDistanceToTetmeshTraversalController() {}

		PX_FORCE_INLINE ClosestDistanceToTetmeshTraversalController(const PxU32* tetrahedra, const PxVec3* points, Gu::BVHNode* nodes) :
			mClosestDistanceSquared(PX_MAX_F32), mTetrahedra(tetrahedra), mPoints(points), mNodes(nodes), mQueryPoint(0.0f), mClosestPoint(0.0f), mClosestTetId(-1)
		{
			initialize(tetrahedra, points, nodes);
		}

		void initialize(const PxU32* tetrahedra, const PxVec3* points, Gu::BVHNode* nodes)
		{
			mTetrahedra = tetrahedra;
			mPoints = points;
			mNodes = nodes;
			mQueryPoint = PxVec3(0.0f);
			mClosestPoint = PxVec3(0.0f);
			mClosestTetId = -1;
			mClosestDistanceSquared = PX_MAX_F32;
		}

		PX_FORCE_INLINE void setQueryPoint(const PxVec3& queryPoint)
		{
			mQueryPoint = queryPoint;
			mClosestDistanceSquared = PX_MAX_F32;
			mClosestPoint = PxVec3(0.0f);
			mClosestTetId = -1;
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
				const PxU32* tet = &mTetrahedra[4 * j];

				PxVec3 closest = closestPtPointTetrahedronWithInsideCheck(mQueryPoint,
					mPoints[tet[0]], mPoints[tet[1]], mPoints[tet[2]], mPoints[tet[3]]);

				PxReal d2 = (closest - mQueryPoint).magnitudeSquared();

				if (d2 < mClosestDistanceSquared)
				{
					mClosestDistanceSquared = d2;
					mClosestTetId = j;
					mClosestPoint = closest;
				}
				if (d2 == 0.0f)
					return Gu::TraversalControl::eAbort;

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

		PxI32 getClosestTetId() const { return mClosestTetId; }

		void setClosestStart(const PxReal closestDistanceSquared, PxI32 closestTetrahedron, const PxVec3& closestPoint)
		{
			mClosestDistanceSquared = closestDistanceSquared;
			mClosestTetId = closestTetrahedron;
			mClosestPoint = closestPoint;
		}

	private:
		PX_NOCOPY(ClosestDistanceToTetmeshTraversalController)
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
		PX_FORCE_INLINE ClosestDistanceToTrimeshTraversalController() {}

		PX_FORCE_INLINE ClosestDistanceToTrimeshTraversalController(const PxU32* triangles, const PxVec3* points, Gu::BVHNode* nodes) :
			mClosestDistanceSquared(PX_MAX_F32), mTriangles(triangles), mPoints(points), mNodes(nodes), mQueryPoint(0.0f), mClosestPoint(0.0f), mClosestTriId(-1)
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
			mQueryPoint = queryPoint;
			mClosestDistanceSquared = PX_MAX_F32;
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
				aos::FloatV d = Gu::distancePointTriangleSquared2UnitBox(q, a, b, c, t1, t2, cp);
				PxReal d2;
				FStore(d, &d2);
				PxVec3 closest;
				V3StoreU(cp, closest);

				//const PxVec3 closest = closestPtPointTriangle2UnitBox(mQueryPoint, mPoints[tri[0]], mPoints[tri[1]], mPoints[tri[2]]);
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



} // namespace Gu
}

#endif // GU_AABBTREE_H
