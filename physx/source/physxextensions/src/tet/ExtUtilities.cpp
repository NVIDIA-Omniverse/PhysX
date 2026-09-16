// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxAssert.h"
#include "ExtUtilities.h"
#include "GuAABBTreeBuildStats.h"
#include "foundation/PxFPU.h"

using namespace physx;
using namespace Ext;
using namespace Gu;
	
static PxVec3 toFloat(const PxVec3d& p)
{
	return PxVec3(PxReal(p.x), PxReal(p.y), PxReal(p.z));
}

void physx::Ext::buildTree(const PxU32* triangles, const PxU32 numTriangles, const PxVec3d* points, PxArray<Gu::BVHNode>& tree, PxF32 enlargement)
{
	//Computes a bounding box for every triangle in triangles
	AABBTreeBounds boxes;
	boxes.init(numTriangles);
	for (PxU32 i = 0; i < numTriangles; ++i)
	{
		const PxU32* tri = &triangles[3 * i];
		PxBounds3 box = PxBounds3::empty();
		box.include(toFloat(points[tri[0]]));
		box.include(toFloat(points[tri[1]]));
		box.include(toFloat(points[tri[2]]));
		box.fattenFast(enlargement);
		boxes.getBounds()[i] = box;
	}

	Gu::buildAABBTree(numTriangles, boxes, tree);
}
