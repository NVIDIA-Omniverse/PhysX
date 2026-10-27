// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxvGlobals.h"

#if PX_SUPPORT_GPU_PHYSX
	#include "PxPhysXGpu.h"
	static physx::PxPhysXGpu* gPxPhysXGpu = NULL;
#endif

namespace physx
{

PxvOffsetTable gPxvOffsetTable;

void PxvInit(const PxvOffsetTable& offsetTable)
{
#if PX_SUPPORT_GPU_PHYSX
	gPxPhysXGpu = NULL;
#endif
	gPxvOffsetTable = offsetTable;
}

void PxvTerm()
{
#if PX_SUPPORT_GPU_PHYSX
	PX_RELEASE(gPxPhysXGpu);
#endif
}

}

#if PX_SUPPORT_GPU_PHYSX
namespace physx
{
	//forward declare stuff from PxPhysXGpuModuleLoader.cpp
	void PxLoadPhysxGPUModule(const char* appGUID);
	void PxUnloadPhysxGPUModule();
	typedef physx::PxPhysXGpu* (PxCreatePhysXGpu_FUNC)();
	extern PxCreatePhysXGpu_FUNC* g_PxCreatePhysXGpu_Func;

	PxPhysXGpu* PxvGetPhysXGpu(bool createIfNeeded)
	{
		if (!gPxPhysXGpu && createIfNeeded)
		{
#ifdef PX_PHYSX_GPU_STATIC
			gPxPhysXGpu = PxCreatePhysXGpu();
#else
			PxLoadPhysxGPUModule(NULL);
			if (g_PxCreatePhysXGpu_Func)
			{
				gPxPhysXGpu = g_PxCreatePhysXGpu_Func();
			}
#endif
		}
		
		return gPxPhysXGpu;
	}

	// PT: added for the standalone GPU BP but we may want to revisit this
	void PxvReleasePhysXGpu(PxPhysXGpu* gpu)
	{
		PX_ASSERT(gpu==gPxPhysXGpu);
		PxUnloadPhysxGPUModule();
		PX_RELEASE(gpu);
		gPxPhysXGpu = NULL;
	}
}
#endif
