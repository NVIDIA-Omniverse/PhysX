// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgBroadPhase.h"

namespace physx
{

	extern "C" void initSolverKernels0();
	extern "C" void initSolverKernels1();
	extern "C" void initSolverKernels2();
	extern "C" void initSolverKernels3();
	extern "C" void initSolverKernels4();
	extern "C" void initSolverKernels5();
	extern "C" void initSolverKernels6();
	extern "C" void initSolverKernels7();
	extern "C" void initSolverKernels9();
	extern "C" void initSolverKernels10();
	extern "C" void initSolverKernels11();
	extern "C" void initSolverKernels13();

	void createPxgSolver()
	{
#if !PX_PHYSX_GPU_EXPORTS
		//this call is needed to force PhysXGpuSolver linkage as Static Library!
		initSolverKernels0();
		initSolverKernels1();
		initSolverKernels2();
		initSolverKernels3();
		initSolverKernels4();
		initSolverKernels5();
		initSolverKernels6();
		initSolverKernels7();
		initSolverKernels9();
		initSolverKernels10();
		initSolverKernels11();
		initSolverKernels13();
#endif
	}

}
