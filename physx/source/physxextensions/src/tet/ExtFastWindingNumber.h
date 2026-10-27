// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_FAST_WINDING_NUMBER_H
#define EXT_FAST_WINDING_NUMBER_H


#include "ExtVec3.h"
#include "GuWindingNumberT.h"

namespace physx
{
namespace Ext
{
	using Triangle = Gu::IndexedTriangleT<PxI32>;
	using Triangle16 = Gu::IndexedTriangleT<PxI16>;
	
	typedef Gu::ClusterApproximationT<PxF64, PxVec3d> ClusterApproximationF64;
	typedef Gu::SecondOrderClusterApproximationT<PxF64, PxVec3d> SecondOrderClusterApproximationF64;
	
	PxF64 computeWindingNumber(const PxArray<Gu::BVHNode>& tree, const PxVec3d& q, PxF64 beta, const PxHashMap<PxU32, ClusterApproximationF64>& clusters,
		const PxArray<Triangle>& triangles, const PxArray<PxVec3d>& points);

	void precomputeClusterInformation(PxArray<Gu::BVHNode>& tree, const PxArray<Triangle>& triangles,
		const PxArray<PxVec3d>& points, PxHashMap<PxU32, ClusterApproximationF64>& result, PxI32 rootNodeIndex = 0);
}
}

#endif

