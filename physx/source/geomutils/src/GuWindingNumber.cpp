// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuWindingNumberT.h"
#include "GuWindingNumber.h"

namespace physx
{
namespace Gu
{
	PxF32 computeWindingNumber(const Gu::BVHNode* tree, const PxVec3& q, PxF32 beta, const PxHashMap<PxU32, ClusterApproximation>& clusters,
		const PxU32* triangles, const PxVec3* points)
	{
		return Gu::computeWindingNumber<PxF32, PxVec3>(tree, q, beta, clusters, triangles, points);
	}

	PxF32 computeWindingNumber(const Gu::BVHNode* tree, const PxVec3& q, const PxHashMap<PxU32, ClusterApproximation>& clusters,
		const PxU32* triangles, const PxVec3* points)
	{
		return Gu::computeWindingNumber<PxF32, PxVec3>(tree, q, 2.0f, clusters, triangles, points);
	}

	void precomputeClusterInformation(const Gu::BVHNode* tree, const PxU32* triangles, const PxU32 numTriangles,
		const PxVec3* points, PxHashMap<PxU32, ClusterApproximation>& result, PxI32 rootNodeIndex)
	{
		Gu::precomputeClusterInformation<PxF32, PxVec3>(tree, triangles, numTriangles, points, result, rootNodeIndex);
	}

	PxF32 computeWindingNumber(const PxVec3& q, const PxU32* triangles, const PxU32 numTriangles, const PxVec3* points)
	{
		PxReal windingNumber = 0.0f;
		for (PxU32 i = 0; i < numTriangles; ++i) 
		{
			const PxU32* tri = &triangles[3 * i];
			windingNumber += Gu::evaluateExact<PxReal, PxVec3>(points[tri[0]], points[tri[1]], points[tri[2]], q);
		}
		return windingNumber;
	}
}
}
