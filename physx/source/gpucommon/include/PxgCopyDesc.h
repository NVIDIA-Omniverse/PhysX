// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_COPY_DESC_H
#define PXG_COPY_DESC_H

#include "foundation/PxPreprocessor.h"

#define COPY_KERNEL_WARPS_PER_BLOCK 4

namespace physx
{

// Lightweight struct for GPU copy operations - extracted from PxgCopyManager
// to avoid heavy header dependencies in CUDA files.
PX_ALIGN_PREFIX(16)
struct PxgCopyDesc
{
	size_t		dest;
	size_t		source; 
	size_t		bytes;
	size_t		pad;

	PX_CUDA_CALLABLE void operator= (const PxgCopyDesc& ref)
	{
		dest = ref.dest;
		source = ref.source;
		bytes = ref.bytes;
	}
} PX_ALIGN_SUFFIX(16);

}

#endif
