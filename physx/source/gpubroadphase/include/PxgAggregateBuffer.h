// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_AGGREGATE_BUFFER_H
#define PXG_AGGREGATE_BUFFER_H

#include "foundation/PxUserAllocated.h"
#include "PxgCudaBuffer.h"

namespace physx
{
	class PxgHeapMemoryAllocator;

	class PxgAggregateBuffer : public PxUserAllocated
	{
	public:
		PxgAggregateBuffer(PxgHeapMemoryAllocator& deviceAlloc);

		PxgCudaBuffer		updateBoundIndices;
		PxgCudaBufferN<2>	boundIndices;
		PxgCudaBufferN<2>	sortedProjections;
		PxgCudaBufferN<2>	sortedHandles;
		PxgCudaBufferN<2>	sapBox1D;
		PxgCudaBufferN<2>	startMasks;
		PxgCudaBufferN<2>	comparisons;
	};
}

#endif
