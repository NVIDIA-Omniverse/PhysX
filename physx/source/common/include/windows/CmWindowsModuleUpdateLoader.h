// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef CM_WINDOWS_MODULEUPDATELOADER_H
#define CM_WINDOWS_MODULEUPDATELOADER_H

#include "foundation/PxPreprocessor.h"
#include "foundation/windows/PxWindowsInclude.h"
#include "common/PxPhysXCommonConfig.h"


namespace physx
{
namespace Cm
{

#if PX_X64
#define UPDATE_LOADER_DLL_NAME "PhysXUpdateLoader64.dll"
#else
#define UPDATE_LOADER_DLL_NAME "PhysXUpdateLoader.dll"
#endif

class PX_PHYSX_COMMON_API CmModuleUpdateLoader
{
public:
	CmModuleUpdateLoader(const char* updateLoaderDllName);

	~CmModuleUpdateLoader();

	// Loads the given module through the update loader. Loads it from the path if 
	// the update loader doesn't find the requested module. Returns NULL if no
	// module found.
	HMODULE LoadModule(const char* moduleName, const char* appGUID);

protected:
	HMODULE mUpdateLoaderDllHandle;
	FARPROC mGetUpdatedModuleFunc;
};
} // namespace Cm
} // namespace physx


#endif	// CM_WINDOWS_MODULEUPDATELOADER_H
