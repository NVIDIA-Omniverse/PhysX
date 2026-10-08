// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef EXT_BVH_H
#define EXT_BVH_H

#include "foundation/PxBounds3.h"
#include "foundation/PxArray.h"
#include "GuAABBTreeNode.h"

namespace physx
{
	namespace Ext
	{
		struct BVHDesc
		{
			PxArray<Gu::BVHNode> tree;
			void query(const PxBounds3& bounds, PxArray<PxI32>& items);
		};

		class BVHBuilder
		{
		public:
			static void build(BVHDesc& bvh, const PxBounds3* items, PxI32 numItems);
		};
	}
}

#endif
