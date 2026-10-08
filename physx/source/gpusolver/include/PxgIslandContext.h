// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_ISLAND_CONTEXT_H
#define PXG_ISLAND_CONTEXT_H

#include "foundation/PxArray.h"

#include "DyCpuGpuBiasCoefficient.h"

namespace physx
{
	struct PxgIslandContext
	{
		PxU32 mBodyStartIndex;
		PxU32 mBodyCount;

		PxU32 mArticulationCount;

		PxU32 mDescStartIndex;
		PxU32 mDescCount;

		PxI32 mNumPositionIterations;
		PxI32 mNumVelocityIterations;

		PxU32 mStartPartitionIndex;
		PxU32 mNumPartitions;

		PxU32 mBatchStartIndex;
		PxU32 mBatchCount;

		PxU32 mArtiBatchStartIndex;
		PxU32 mArtiBatchCount;
		PxU32 mStaticArtiBatchCount;
		PxU32 mSelfArtiBatchCount;

		PxU32 mStaticRigidBatchCount;

		Dy::BiasCoefficientCollection mBiasCoefficients;
	};
}
#endif