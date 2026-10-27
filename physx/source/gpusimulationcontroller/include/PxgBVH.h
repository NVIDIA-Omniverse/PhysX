// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_BVH_H
#define PXG_BVH_H

#include "foundation/PxVec3.h"
#include "GuSDF.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX

	PX_ALIGN_PREFIX(16)
	struct PxgPackedNodeHalf
	{
		PxReal x;
		PxReal y;
		PxReal z;
		PxU32 i : 31;
		PxU32 b : 1;

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 getXYZ()
		{
			return PxVec3(x, y, z);
		}
	}
	PX_ALIGN_SUFFIX(16);

	PX_ALIGN_PREFIX(16)
	struct PxgBVH
	{
		// for bottom up tree construction the root node does not appear in slot 0
		// this is a single int CUDA alloc that holds the index of the root
		PxU32* mRootNode;

		PxgPackedNodeHalf* mNodeLowers; // stores the lower spatial bound of the node's children, left child stored in i, leaf flag stored in b
		PxgPackedNodeHalf* mNodeUppers; // stores the upper spatial bound of the node's children, right child stored in i, flag is unused

		PxU32 mNumNodes;
		PxU32 mMaxNodes;

		PxgBVH() : mRootNode(NULL), mNodeLowers(NULL), mNodeUppers(NULL), mNumNodes(0), mMaxNodes(0)
		{}
	}
	PX_ALIGN_SUFFIX(16);

	PX_ALIGN_PREFIX(16)
	struct PxgBvhTriangleMesh
	{
		PxgBVH mBvh;

		PxVec3* mVertices;		
		PxU32* mTriangles;
		PxU32 mNumTriangles;
		PxU32 mNumVertices;
		PxU32 mPad[2];

		PxgBvhTriangleMesh() : mBvh(), mVertices(NULL), mTriangles(NULL), mNumTriangles(0), mNumVertices(0)
		{}
	}
	PX_ALIGN_SUFFIX(16);

	PX_ALIGN_PREFIX(16)
	struct PxgWindingClusterApproximation
	{
	public:
		PxVec3 mCentroidTimesArea;
		PxReal mRadius;
		PxVec3 mWeightedNormalSum;
		PxReal mAreaSum;	

		PX_CUDA_CALLABLE PxgWindingClusterApproximation() : mCentroidTimesArea(PxVec3(0.0f)), mRadius(0.0f), mWeightedNormalSum(PxVec3(0.0f)), mAreaSum(0.0f)
		{}
	}
	PX_ALIGN_SUFFIX(16);

	struct PxgBVHKernelBlockDim
	{
		enum
		{
			BUILD_HIERARCHY = 256,
			BUILD_SDF = 256,
			SDF_FIX_HOLES = 256
		};
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif