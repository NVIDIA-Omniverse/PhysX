// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "common/windows/PxWindowsDelayLoadHook.h"
#include "foundation/windows/PxWindowsInclude.h"
#include "windows/CmWindowsLoadLibrary.h"

static const physx::PxDelayLoadHook* gCookingDelayLoadHook = NULL;

void physx::PxSetPhysXCookingDelayLoadHook(const physx::PxDelayLoadHook* hook)
{
	gCookingDelayLoadHook = hook;
}

// delay loading is enabled only for non static configuration
#if !defined PX_PHYSX_STATIC_LIB 

// Prior to Visual Studio 2015 Update 3, these hooks were non-const.
#define DELAYIMP_INSECURE_WRITABLE_HOOKS
#include <delayimp.h>

using namespace physx;

#pragma comment(lib, "delayimp")

FARPROC WINAPI cookingDelayHook(unsigned dliNotify, PDelayLoadInfo pdli)
{
	switch (dliNotify) {
	case dliStartProcessing :
		break;

	case dliNotePreLoadLibrary :
		{
			return Cm::physXCommonDliNotePreLoadLibrary(pdli->szDll,gCookingDelayLoadHook);
		}
		break;

	case dliNotePreGetProcAddress :
		break;

	case dliFailLoadLib :
		break;

	case dliFailGetProc :
		break;

	case dliNoteEndProcessing :
		break;

	default :

		return NULL;
	}

	return NULL;
}

PfnDliHook __pfnDliNotifyHook2 = cookingDelayHook;

#endif
