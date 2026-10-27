// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_RADIX_SORT_KERNEL_INDICES_H
#define PXG_RADIX_SORT_KERNEL_INDICES_H

namespace physx
{

	struct PxgRadixSortKernelBlockDim
	{
		enum
		{
			RADIX_SORT = 1024,
		};
	};

	struct PxgRadixSortKernelGridDim
	{
		enum
		{
			RADIX_SORT = 32,
		};
	};

}

#endif
