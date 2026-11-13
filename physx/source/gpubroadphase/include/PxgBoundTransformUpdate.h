// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_BOUND_TRANSFORM_UPDATE_H
#define PXG_BOUND_TRANSFORM_UPDATE_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
	struct PxgBoundTransformUpdate
	{
		// The top bit of index holds where the merge takes the bounds and the transform of the element from.
		enum Kind : PxU32
		{
			eCOMPUTE_FROM_GPU_POSE	= 0u,		// computed on the gpu from the pose of the dynamic rigid
			eCOPY_FROM_CPU			= 1u << 31,	// copied from the cpu array
			eKIND_MASK				= 1u << 31,
			eINDEX_MASK				= 0x7FFFFFFF
		};

		PxU32 index;
	};
}

#endif
