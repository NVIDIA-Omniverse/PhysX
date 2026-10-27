// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgBroadPhase.h"

namespace physx
{

	extern "C" void initNarrowphaseKernels0();
	extern "C" void initNarrowphaseKernels1();
	extern "C" void initNarrowphaseKernels2();
	extern "C" void initNarrowphaseKernels3();
	extern "C" void initNarrowphaseKernels4();
	extern "C" void initNarrowphaseKernels5();
	extern "C" void initNarrowphaseKernels6();
	extern "C" void initNarrowphaseKernels7();
	extern "C" void initNarrowphaseKernels8();
	extern "C" void initNarrowphaseKernels9();
	extern "C" void initNarrowphaseKernels10();
	extern "C" void initNarrowphaseKernels11();
	extern "C" void initNarrowphaseKernels12();
	extern "C" void initNarrowphaseKernels13();
	extern "C" void initNarrowphaseKernels14();
	extern "C" void initNarrowphaseKernels15();
	extern "C" void initNarrowphaseKernels16();
	extern "C" void initNarrowphaseKernels17();
	extern "C" void initNarrowphaseKernels18();
	extern "C" void initNarrowphaseKernels19();
	extern "C" void initNarrowphaseKernels20();
	extern "C" void initNarrowphaseKernels21();
	extern "C" void initNarrowphaseKernels22();
	extern "C" void initNarrowphaseKernels23();
	extern "C" void initNarrowphaseKernels24();

	void createPxgNarrowphase()
	{
#if !PX_PHYSX_GPU_EXPORTS
		//this call is needed to force PhysXNarrowphaseGpu linkage as Static Library!
		initNarrowphaseKernels0();
		initNarrowphaseKernels1();
		initNarrowphaseKernels2();
		initNarrowphaseKernels3();
		initNarrowphaseKernels4();
		initNarrowphaseKernels5();
		initNarrowphaseKernels6();
		initNarrowphaseKernels7();
		initNarrowphaseKernels8();
		initNarrowphaseKernels9();
		initNarrowphaseKernels10();
		initNarrowphaseKernels11();
		initNarrowphaseKernels12();
		initNarrowphaseKernels13();
		initNarrowphaseKernels14();
		initNarrowphaseKernels15();
		initNarrowphaseKernels16();
		initNarrowphaseKernels17();
		initNarrowphaseKernels18();
		initNarrowphaseKernels19();
		initNarrowphaseKernels20();
		initNarrowphaseKernels21();
		initNarrowphaseKernels22();
		initNarrowphaseKernels23();
		initNarrowphaseKernels24();
#endif
	}

}
