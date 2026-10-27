// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgBroadPhase.h"

namespace physx
{

	extern "C" void initBroadphaseKernels0();
	extern "C" void initBroadphaseKernels1();

	void createPxgBroadphase()
	{
#if !PX_PHYSX_GPU_EXPORTS
		//this call is needed to force PhysXBroadphaseGpu linkage as Static Library!
		initBroadphaseKernels0();
		initBroadphaseKernels1();
#endif
	}

}
