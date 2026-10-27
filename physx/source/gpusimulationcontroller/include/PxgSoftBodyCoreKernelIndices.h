// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SOFTBODY_CORE_KERNEL_INDICES_H
#define PXG_SOFTBODY_CORE_KERNEL_INDICES_H

namespace physx
{

	struct PxgSoftBodyKernelBlockDim
	{
		enum
		{
			SB_PREINTEGRATION = 1024,
			SB_REFIT = 256,
			SB_INTERNALSOLVE = 256,
			SB_UPDATEROTATION = 256,
			SB_SOLVETETRA = 64,
			SB_SOLVETETRA_LOW = 32,
			SB_REORDERCONTACTS = 256,
			SB_ACCUMULATE_DELTA = 512,
		};
	};

	struct PxgSoftBodyKernelGridDim
	{
		enum
		{
			SB_REFIT = 32,
			SB_SBMIDPHASE = 1024,
			SB_SBCG = 1024,
			SB_MESHCG = 1024,
			SB_HFCG = 1024,
			SB_UPDATEROTATION = 1024,
			SB_SOLVETETRA = 64,
			SB_REORDERCONTACTS = 1024,
			SB_ACCUMULATE_DELTA = 32,
		};
	};


}

#endif
