// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ExtBVH.h"
#include "ExtUtilities.h"

using namespace physx;
using namespace Ext;
using namespace Gu;

void BVHDesc::query(const PxBounds3& bounds, PxArray<PxI32>& items)
{
	items.clear();
	IntersectionCollectingTraversalController traversalController(bounds, items);
	traverseBVH(tree.begin(), traversalController, 0);
}
		
void BVHBuilder::build(BVHDesc& bvh, const PxBounds3* items, PxI32 n)
{
	AABBTreeBounds boxes;
	boxes.init(n);
	for (PxI32 i = 0; i < n; ++i)			
		boxes.getBounds()[i] = items[i];
	Gu::buildAABBTree(n, boxes, bvh.tree);
}
