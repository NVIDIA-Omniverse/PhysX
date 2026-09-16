// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_AABB_TREE_UPDATE_MAP_H
#define GU_AABB_TREE_UPDATE_MAP_H

#include "common/PxPhysXCommonConfig.h"
#include "GuPrunerTypedef.h"
#include "foundation/PxArray.h"

namespace physx
{
namespace Gu
{
	class AABBTree;

	// Maps pruning pool indices to AABB-tree indices (i.e. locates the object's box in the aabb-tree nodes pool)
	// 
	// The map spans pool indices from 0..N-1, where N is the number of pool entries when the map was created from a tree.
	//
	// It maps: 
	//		to node indices in the range 0..M-1, where M is the number of nodes in the tree the map was created from,
	//   or to INVALID_NODE_ID if the pool entry was removed or pool index is outside input domain.
	//
	// The map is the inverse of the tree mapping: (node[map[poolID]].primitive == poolID) is true at all times.

	class AABBTreeUpdateMap 
	{
	public:
													AABBTreeUpdateMap()		{}
													~AABBTreeUpdateMap()	{}

							void					release()
													{
														mMapping.reset();
													}

						// indices offset used when indices are shifted from objects (used for merged trees)
		PX_PHYSX_COMMON_API	void					initMap(PxU32 numPoolObjects, const AABBTree& tree);

		PX_PHYSX_COMMON_API	void					invalidate(PoolIndex poolIndex, PoolIndex replacementPoolIndex, AABBTree& tree);

		PX_FORCE_INLINE		TreeNodeIndex			operator[](PxU32 poolIndex) const
													{
														return poolIndex < mMapping.size() ? mMapping[poolIndex] : INVALID_NODE_ID;
													}
	private:
		// maps from prunerIndex (index in the PruningPool) to treeNode index
		// this will only map to leaf tree nodes
							PxArray<TreeNodeIndex>	mMapping;
	};

}
}

#endif
