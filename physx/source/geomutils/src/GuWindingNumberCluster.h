// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_WINDING_NUMBER_CLUSTER_H
#define GU_WINDING_NUMBER_CLUSTER_H


namespace physx
{
namespace Gu
{
	template<typename R, typename V3>
	struct ClusterApproximationT
	{
		R Radius;
		R AreaSum;
		V3 WeightedCentroid;
		V3 WeightedNormalSum;

		PX_FORCE_INLINE ClusterApproximationT() {}

		PX_FORCE_INLINE ClusterApproximationT(R radius, R areaSum, const V3& weightedCentroid, const V3& weightedNormalSum) :
			Radius(radius), AreaSum(areaSum), WeightedCentroid(weightedCentroid), WeightedNormalSum(weightedNormalSum)
		{ }
	};
}
}

#endif
