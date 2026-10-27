// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_NP_KERNEL_INDICES_H
#define PXG_NP_KERNEL_INDICES_H

#define MIDPHASE_WARPS_PER_BLOCK              2
#define NP_TRIMESH_WARPS_PER_BLOCK            2
#define CORRELATE_WARPS_PER_BLOCK             2
#define PS_MIDPHASE_COLLISION_WAPRS_PER_BLOCK 1
#define SB_REFIT_WAPRS_PER_BLOCK              32

namespace physx
{

	struct PxgNarrowPhaseBlockDims
	{
		enum
		{
			REMOVE_CONTACT_MANAGERS		= 512,
			COMPACT_LOST_FOUND_PAIRS    = 512,
			FINISH_CONTACTS    = 128,
			EARLY_OUT_KERNEL = 128,
			COLLIDE_KERNEL = 128,
			INITIALIZE_MANIFOLDS = 512,
			COMPRESS_CONTACT = 256
		};
	};

	struct PxgNarrowPhaseGridDims
	{
		enum
		{
			REMOVE_CONTACT_MANAGERS		= 32,
			COMPACT_LOST_FOUND_PAIRS    = 32,
			EARLY_OUT_KERNEL = 32,
			COLLIDE_KERNEL = 32,
			INITIALIZE_MANIFOLDS = 128,
			COMPRESS_CONTACT = 32,

		};
	};
}

#endif

