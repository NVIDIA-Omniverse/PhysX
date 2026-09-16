// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_RADIXSORT_DESC_H
#define PXG_RADIXSORT_DESC_H

#include "foundation/PxSimpleTypes.h"

#define	NUM_RADIX_SORT_DESC	2

namespace physx
{
	
	struct PxgRadixSortDesc
	{
		PxU32*	inputKeys;
		PxU32*	inputRanks;
		PxU32*	outputKeys;
		PxU32*	outputRanks;
		PxU32*	radixBlockCounts;				//store the each radix's total number different different blocks
		PxU32	count;
	};

	struct PxgRadixSortBlockDesc : public PxgRadixSortDesc
	{
	public:
		PxU32* numKeys;
	};

}

#endif