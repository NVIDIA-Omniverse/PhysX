// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ExtFastWindingNumber.h"
#include "foundation/PxSort.h"

#include "foundation/PxMath.h"

#include "ExtUtilities.h"

//The following paper explains all the techniques used in this file
//http://www.dgp.toronto.edu/projects/fast-winding-numbers/fast-winding-numbers-for-soups-and-clouds-siggraph-2018-barill-et-al.pdf

namespace physx
{
namespace Ext
{
	PxF64 computeWindingNumber(const PxArray<Gu::BVHNode>& tree, const PxVec3d& q, PxF64 beta, const PxHashMap<PxU32, ClusterApproximationF64>& clusters,
		const PxArray<Triangle>& triangles, const PxArray<PxVec3d>& points)
	{
		return Gu::computeWindingNumber<PxF64, PxVec3d>(tree.begin(), q, beta, clusters, reinterpret_cast<const PxU32*>(triangles.begin()), points.begin());
	}

	void precomputeClusterInformation(PxArray<Gu::BVHNode>& tree, const PxArray<Triangle>& triangles,
		const PxArray<PxVec3d>& points, PxHashMap<PxU32, ClusterApproximationF64>& result, PxI32 rootNodeIndex)
	{
		Gu::precomputeClusterInformation<PxF64, PxVec3d>(tree.begin(), reinterpret_cast<const PxU32*>(triangles.begin()), triangles.size(), points.begin(), result, rootNodeIndex);
	}
}
}
