// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_PARTICLE_SYSTEM_CORE_KERNEL_INDICES_H
#define PXG_PARTICLE_SYSTEM_CORE_KERNEL_INDICES_H

namespace physx
{

	struct PxgParticleSystemKernelBlockDim
	{
		enum
		{
			UPDATEBOUND = 1024, //can't change this. updateBound kernel is relied on numOfWarpPerBlock = 32
			UPDATEGRID = 1024,
			BOUNDCELLUPDATE = 512,
			PS_COLLISION = 256, //128,
			PS_MESH_COLLISION = 512,
			PS_HEIGHTFIELD_COLLISION = 64,
			ACCUMULATE_DELTA = 512,
			PS_SOLVE = 256,
			PS_CELL_RECOMPUTE = 256,
			SCAN = 512
		};
	};

	struct PxgParticleSystemKernelGridDim
	{
		enum
		{
			BOUNDCELLUPDATE = 32,
			PS_COLLISION = 1024,
			PS_MESH_COLLISION = 16384,
			PS_HEIGHTFIELD_COLLISION = 4096,
			ACCUMULATE_DELTA = 32,
			PS_CELL_RECOMPUTE = 32,
		};
	};
}

#endif
