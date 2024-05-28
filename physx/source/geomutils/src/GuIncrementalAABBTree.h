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

#ifndef GU_INCREMENTAL_AABB_TREE_H
#define GU_INCREMENTAL_AABB_TREE_H

#include "foundation/PxBounds3.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxVecMath.h"
#include "foundation/PxPool.h"
#include "common/PxPhysXCommonConfig.h"
#include "GuAABBTree.h"
#include "GuPrunerTypedef.h"

namespace physx
{
	using namespace aos;

	namespace Gu
	{
		struct BVHNode;
		class BVH;

		#define INCR_NB_OBJECTS_PER_NODE	4

		// tree indices, can change in runtime
		struct AABBTreeIndices
		{
			PX_FORCE_INLINE AABBTreeIndices(PoolIndex index) : nbIndices(1)
			{
				indices[0] = index;
				for(PxU32 i=1; i<INCR_NB_OBJECTS_PER_NODE; i++)
					indices[i] = 0;
			}

			PxU32		nbIndices;
			PoolIndex	indices[INCR_NB_OBJECTS_PER_NODE];
		};

		// tree node, has parent information
		class IncrementalAABBTreeNode : public PxUserAllocated
		{
		public:
			PX_FORCE_INLINE									IncrementalAABBTreeNode() : mParent(NULL)
															{
																mChilds[0] = NULL;
																mChilds[1] = NULL;
															}
			PX_FORCE_INLINE									IncrementalAABBTreeNode(AABBTreeIndices* indices) : mParent(NULL)
															{
																mIndices = indices;
																mChilds[1] = NULL;
															}
			PX_FORCE_INLINE									~IncrementalAABBTreeNode() {}

			PX_FORCE_INLINE	PxU32							isLeaf()								const	{ return PxU32(mChilds[1]==0);	}

			PX_FORCE_INLINE	const PxU32*					getPrimitives(const PxU32*)				const	{ return &mIndices->indices[0]; }
			PX_FORCE_INLINE	PxU32*							getPrimitives(PxU32*)							{ return &mIndices->indices[0]; }
			PX_FORCE_INLINE	PxU32							getNbPrimitives()						const	{ return mIndices->nbIndices;	}
			PX_FORCE_INLINE	PxU32							getPrimitiveIndex()						const	{ return PX_INVALID_U32;		}

			PX_FORCE_INLINE	const IncrementalAABBTreeNode*	getPos(const IncrementalAABBTreeNode*)	const	{ return mChilds[0];			}
			PX_FORCE_INLINE	const IncrementalAABBTreeNode*	getNeg(const IncrementalAABBTreeNode*)	const	{ return mChilds[1];			}

			PX_FORCE_INLINE	IncrementalAABBTreeNode*		getPos(IncrementalAABBTreeNode*)				{ return mChilds[0];			}
			PX_FORCE_INLINE	IncrementalAABBTreeNode*		getNeg(IncrementalAABBTreeNode*)				{ return mChilds[1];			}

			// PT: TODO: these functions are duplicates from the regular AABB tree node
			PX_FORCE_INLINE	void							getAABBCenterExtentsV(physx::aos::Vec3V* center, physx::aos::Vec3V* extents) const
															{
																const float half = 0.5f;
																const FloatV halfV = FLoad(half);

																*extents = Vec3V_From_Vec4V((V4Scale(V4Sub(mBVMax, mBVMin), halfV)));
																*center = Vec3V_From_Vec4V((V4Scale(V4Add(mBVMax, mBVMin), halfV)));
															}

			PX_FORCE_INLINE	void							getAABBCenterExtentsV2(physx::aos::Vec3V* center, physx::aos::Vec3V* extents) const
															{
																*extents = Vec3V_From_Vec4V((V4Sub(mBVMax, mBVMin)));
																*center = Vec3V_From_Vec4V((V4Add(mBVMax, mBVMin)));
															}

							Vec4V							mBVMin;		// Global bounding-volume min enclosing all the node-related primitives
							Vec4V							mBVMax;		// Global bounding-volume max enclosing all the node-related primitives			
							IncrementalAABBTreeNode*		mParent;	// node parent
							union
							{
								IncrementalAABBTreeNode*	mChilds[2];		// childs of node if not a leaf
								AABBTreeIndices*			mIndices;		// if leaf, indices information 
							};			
		};

		struct IncrementalAABBTreeNodePair
		{
			IncrementalAABBTreeNode	mNode0;
			IncrementalAABBTreeNode	mNode1;
		};

		typedef PxArray<IncrementalAABBTreeNode*>	NodeList;

		// incremental AABB tree, all changes are immediatelly reflected to the tree
		class IncrementalAABBTree : public PxUserAllocated
		{
		public:
			PX_PHYSX_COMMON_API										IncrementalAABBTree();
			PX_PHYSX_COMMON_API										~IncrementalAABBTree();

			// Build the tree for the first time 
			PX_PHYSX_COMMON_API	bool								build(const AABBTreeBuildParams& params, PxArray<IncrementalAABBTreeNode*>& mapping);			

			// insert a new index into the tree
			PX_PHYSX_COMMON_API	IncrementalAABBTreeNode*			insert(const PoolIndex index, const PxBounds3* bounds, NodeList& changedLeaf);

			// update the object in the tree - full update insert/remove
			PX_PHYSX_COMMON_API	IncrementalAABBTreeNode*			update(IncrementalAABBTreeNode* node, const PoolIndex index, const PxBounds3* bounds, NodeList& changedLeaf);
			// update the object in the tree, faster method, that may unbalance the tree
			PX_PHYSX_COMMON_API	IncrementalAABBTreeNode*			updateFast(IncrementalAABBTreeNode* node, const PoolIndex index, const PxBounds3* bounds, NodeList& changedLeaf);

			// remove object from the tree
			PX_PHYSX_COMMON_API	IncrementalAABBTreeNode*			remove(IncrementalAABBTreeNode* node, const PoolIndex index, const PxBounds3* bounds);

			// fixup the tree indices, if we swapped the objects in the pruning pool
			PX_PHYSX_COMMON_API	void								fixupTreeIndices(IncrementalAABBTreeNode* node, const PoolIndex index, const PoolIndex newIndex);

			// origin shift
			PX_PHYSX_COMMON_API	void								shiftOrigin(const PxVec3& shift);
			
			// get the tree root node
			PX_FORCE_INLINE		const IncrementalAABBTreeNode*		getNodes() const { return mRoot; }

			// define this function so we can share the scene query code with regular AABBTree
			PX_FORCE_INLINE		const PxU32*						getIndices() const { return NULL; }

			// paranoia checks
			PX_PHYSX_COMMON_API	void								hierarchyCheck(PoolIndex maxIndex, const PxBounds3* bounds);
			PX_PHYSX_COMMON_API	void								hierarchyCheck(const PxBounds3* bounds);
			PX_PHYSX_COMMON_API	void								checkTreeLeaf(IncrementalAABBTreeNode* leaf, PoolIndex h);
			PX_PHYSX_COMMON_API	PxU32								getTreeLeafDepth(IncrementalAABBTreeNode* leaf);

			PX_PHYSX_COMMON_API	void								release();

			PX_PHYSX_COMMON_API	void								copy(const BVH& bvh, PxArray<IncrementalAABBTreeNode*>& mapping);
			
		private:
			// clone the tree from the generic AABB tree that was built
								void								clone(PxArray<IncrementalAABBTreeNode*>& mapping, const PxU32* indices, IncrementalAABBTreeNode** treeNodes);

								void								copyNode(IncrementalAABBTreeNode& destNode, const BVHNode& sourceNode, const BVHNode* nodeBase, 
																			IncrementalAABBTreeNode* parent, const PxU32* primitivesBase, PxArray<IncrementalAABBTreeNode*>& mapping);

			// split leaf node, the newly added object does not fit in
								IncrementalAABBTreeNode*			splitLeafNode(IncrementalAABBTreeNode* node, const PoolIndex index, const Vec4V& minV, const Vec4V& maxV, const PxBounds3* bounds);

								void								rotateTree(IncrementalAABBTreeNode* node, NodeList& changedLeaf, PxU32 largesRotateNode, const PxBounds3* bounds, bool rotateAgain);

								void								releaseNode(IncrementalAABBTreeNode* node);

								PxPool<AABBTreeIndices>				mIndicesPool;
								PxPool<IncrementalAABBTreeNodePair>	mNodesPool;
								IncrementalAABBTreeNode*			mRoot;

								NodeAllocator						mNodeAllocator;
		};
	}
}

#endif
