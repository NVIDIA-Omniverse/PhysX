// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_WINDING_NUMBER_H
#define GU_WINDING_NUMBER_H


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"
#include "GuWindingNumberCluster.h"

namespace physx
{
namespace Gu
{
	struct BVHNode;

	typedef ClusterApproximationT<PxReal, PxVec3> ClusterApproximation;
	
	PX_PHYSX_COMMON_API PxF32 computeWindingNumber(const Gu::BVHNode* tree, const PxVec3& q, const PxHashMap<PxU32, ClusterApproximation>& clusters,
		const PxU32* triangles, const PxVec3* points);

	PX_PHYSX_COMMON_API PxF32 computeWindingNumber(const Gu::BVHNode* tree, const PxVec3& q, PxF32 beta, const PxHashMap<PxU32, ClusterApproximation>& clusters,
		const PxU32* triangles, const PxVec3* points);

	PX_PHYSX_COMMON_API void precomputeClusterInformation(const Gu::BVHNode* tree, const PxU32* triangles, const PxU32 numTriangles,
		const PxVec3* points, PxHashMap<PxU32, ClusterApproximation>& result, PxI32 rootNodeIndex = 0);	

	//Quite slow, only useful for few query points, otherwise it is worth to construct a tree for acceleration
	PX_PHYSX_COMMON_API PxF32 computeWindingNumber(const PxVec3& q, const PxU32* triangles, const PxU32 numTriangles, const PxVec3* points);
}
}

#endif
