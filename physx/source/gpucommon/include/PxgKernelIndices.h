// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_KERNEL_INDICES_H
#define PXG_KERNEL_INDICES_H

namespace physx
{
	struct PxgKernelIds
	{
		enum
		{

#define KERNEL_DEF(id, name) id,
#include "PxgKernelNames.h"
#undef KERNEL_DEF

			KERNEL_COUNT
		};
	};
}
#endif
