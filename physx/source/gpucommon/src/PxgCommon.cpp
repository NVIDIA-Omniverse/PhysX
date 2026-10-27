// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgBroadPhase.h"

namespace physx
{

	extern "C" void initCommonKernels0();
	extern "C" void initCommonKernels1();
	extern "C" void initCommonKernels2();

	void createPxgCommon()
	{
#if !PX_PHYSX_GPU_EXPORTS
		//this call is needed to force PhysXCommonGpu linkage as Static Library!
		initCommonKernels0();
		initCommonKernels1();
		initCommonKernels2();
#endif
	}

}
