// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_FEMCLOTH_CORE_KERNEL_INDICES_H
#define PXG_FEMCLOTH_CORE_KERNEL_INDICES_H

namespace physx
{

	struct PxgFEMClothKernelBlockDim
	{
		enum
		{
			CLOTH_PREINTEGRATION = 512,
			CLOTH_STEP = 1024,
			CLOTH_SOLVESHELL = 128
		};
	};
}

#endif
