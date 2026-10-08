// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef CM_WINDOWS_LOADLIBRARY_H
#define CM_WINDOWS_LOADLIBRARY_H

#include "foundation/PxPreprocessor.h"
#include "foundation/windows/PxWindowsInclude.h"
#include "common/windows/PxWindowsDelayLoadHook.h"

namespace physx
{
namespace Cm
{
	EXTERN_C IMAGE_DOS_HEADER __ImageBase;

	PX_INLINE HMODULE WINAPI loadLibrary(const char* name)
	{
		return ::LoadLibraryA( name );
	};

	PX_INLINE FARPROC WINAPI physXCommonDliNotePreLoadLibrary(const char* libraryName, const physx::PxDelayLoadHook* delayLoadHook)
	{
		if(!delayLoadHook)
		{
			return (FARPROC)loadLibrary(libraryName);
		}
		else
		{
			if(strstr(libraryName, "PhysXFoundation"))
			{
				return (FARPROC)Cm::loadLibrary(delayLoadHook->getPhysXFoundationDllName());
			}

			if(strstr(libraryName, "PhysXCommon"))
			{
				return (FARPROC)Cm::loadLibrary(delayLoadHook->getPhysXCommonDllName());
			}
		}
		return NULL;
    }
} // namespace Cm
} // namespace physx


#endif	// CM_WINDOWS_LOADLIBRARY_H
