// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

#include "PxPhysXConfig.h"

#if PX_SUPPORT_GPU_PHYSX

#include "foundation/Px.h"
#include "gpu/PxGpu.h"
#include "cudamanager/PxCudaContextManager.h"
#include "PxPhysics.h"

#if PX_WINDOWS
#include "common/windows/PxWindowsDelayLoadHook.h"
#include "foundation/windows/PxWindowsInclude.h"
#include "windows/CmWindowsModuleUpdateLoader.h"
#elif PX_LINUX
#include <dlfcn.h>
#endif // ~PX_LINUX

#include "stdio.h"


#if PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wundefined-reinterpret-cast"
#endif

#define STRINGIFY(x) #x
#define GETSTRING(x) STRINGIFY(x)

#define PHYSX_GPU_SHARED_LIB_NAME GETSTRING(PX_PHYSX_GPU_SHARED_LIB_NAME)
static const char*	gPhysXGpuLibraryName = PHYSX_GPU_SHARED_LIB_NAME;

#undef GETSTRING
#undef STRINGIFY

// Use reportError to handle cases where PxFoundation has not been created yet
static void reportError(const char* file, int line, const char* format, ...)
{
	va_list args;
	va_start(args, format);

	physx::PxFoundation* foundation = PxIsFoundationValid();
	if(foundation)
	{
		foundation->error(physx::PxErrorCode::eINTERNAL_ERROR, file, line, format, args);
	}
	else
	{
		fprintf(stderr, "Error in %s:%i: ", file, line);
		vfprintf(stderr, format, args);
	}

	va_end(args);
}


void PxSetPhysXGpuLoadHook(const PxGpuLoadHook* hook)
{
	gPhysXGpuLibraryName = hook->getPhysXGpuDllName();
}

namespace physx
{
#if PX_VC
#pragma warning(disable: 4191)	//'operator/operation' : unsafe conversion from 'type of expression' to 'type required'
#endif

	class PxFoundation;
	class PxPhysXGpu;
	class PxPhysicsGpu;

	typedef physx::PxPhysXGpu* (PxCreatePhysXGpu_FUNC)();
	typedef physx::PxCudaContextManager* (PxCreateCudaContextManager_FUNC)(physx::PxFoundation& foundation, const physx::PxCudaContextManagerDesc& desc, physx::PxProfilerCallback* profilerCallback, bool launchSynchronous);
	typedef int (PxGetSuggestedCudaDeviceOrdinal_FUNC)(physx::PxErrorCallback& errc);
	typedef void (PxSetPhysXGpuProfilerCallback_FUNC)(physx::PxProfilerCallback* cbk);
	typedef void (PxCudaRegisterFunction_FUNC)(int, const char*);
	typedef void** (PxCudaRegisterFatBinary_FUNC)(void*);
	typedef physx::PxKernelIndex* (PxGetCudaFunctionTable_FUNC)();
	typedef PxU32 (PxGetCudaFunctionTableSize_FUNC)();
	typedef void** PxGetCudaModuleTable_FUNC();
	typedef PxPhysicsGpu* PxCreatePhysicsGpu_FUNC();

	PxCreatePhysXGpu_FUNC* g_PxCreatePhysXGpu_Func = NULL;
	PxCreateCudaContextManager_FUNC* g_PxCreateCudaContextManager_Func = NULL;
	PxGetSuggestedCudaDeviceOrdinal_FUNC* g_PxGetSuggestedCudaDeviceOrdinal_Func = NULL;
	PxSetPhysXGpuProfilerCallback_FUNC* g_PxSetPhysXGpuProfilerCallback_Func = NULL;
	PxCudaRegisterFunction_FUNC* g_PxCudaRegisterFunction_Func = NULL;
	PxCudaRegisterFatBinary_FUNC* g_PxCudaRegisterFatBinary_Func = NULL;
	PxGetCudaFunctionTable_FUNC* g_PxGetCudaFunctionTable_Func = NULL;
	PxGetCudaFunctionTableSize_FUNC* g_PxGetCudaFunctionTableSize_Func = NULL;
	PxGetCudaFunctionTableSize_FUNC* g_PxGetCudaModuleTableSize_Func = NULL;
	PxGetCudaModuleTable_FUNC* g_PxGetCudaModuleTable_Func = NULL;
	PxCreatePhysicsGpu_FUNC* g_PxCreatePhysicsGpu_Func = NULL;

	void PxUnloadPhysxGPUModule()
	{
		g_PxCreatePhysXGpu_Func = NULL;
		g_PxCreateCudaContextManager_Func = NULL;
		g_PxGetSuggestedCudaDeviceOrdinal_Func = NULL;
		g_PxSetPhysXGpuProfilerCallback_Func = NULL;
		g_PxCudaRegisterFunction_Func = NULL;
		g_PxCudaRegisterFatBinary_Func = NULL;
		g_PxGetCudaFunctionTable_Func = NULL;
		g_PxGetCudaFunctionTableSize_Func = NULL;
		g_PxGetCudaModuleTableSize_Func = NULL;
		g_PxGetCudaModuleTable_Func = NULL;
		g_PxCreatePhysicsGpu_Func = NULL;
	}

#if PX_WINDOWS

	typedef void (PxSetPhysXGpuDelayLoadHook_FUNC)(const PxDelayLoadHook* delayLoadHook);	

#define DEFAULT_PHYSX_GPU_GUID    "D79FA4BF-177C-4841-8091-4375D311D6A3"
	   
	void PxLoadPhysxGPUModule(const char* appGUID)
	{
		HMODULE s_library = GetModuleHandle(gPhysXGpuLibraryName);

		bool freshlyLoaded = false;
		if (s_library == NULL)
		{
			Cm::CmModuleUpdateLoader moduleLoader(UPDATE_LOADER_DLL_NAME);
			s_library = moduleLoader.LoadModule(gPhysXGpuLibraryName, appGUID == NULL ? DEFAULT_PHYSX_GPU_GUID : appGUID);
			freshlyLoaded = true;
		}

		if (s_library && (freshlyLoaded || g_PxCreatePhysXGpu_Func == NULL))
		{			
			g_PxCreatePhysXGpu_Func = (PxCreatePhysXGpu_FUNC*)GetProcAddress(s_library, "PxCreatePhysXGpu");
			g_PxCreateCudaContextManager_Func = (PxCreateCudaContextManager_FUNC*)GetProcAddress(s_library, "PxCreateCudaContextManager");
			g_PxGetSuggestedCudaDeviceOrdinal_Func = (PxGetSuggestedCudaDeviceOrdinal_FUNC*)GetProcAddress(s_library, "PxGetSuggestedCudaDeviceOrdinal");
			g_PxSetPhysXGpuProfilerCallback_Func = (PxSetPhysXGpuProfilerCallback_FUNC*)GetProcAddress(s_library, "PxSetPhysXGpuProfilerCallback");
			g_PxCudaRegisterFunction_Func = (PxCudaRegisterFunction_FUNC*)GetProcAddress(s_library, "PxGpuCudaRegisterFunction");
			g_PxCudaRegisterFatBinary_Func = (PxCudaRegisterFatBinary_FUNC*)GetProcAddress(s_library, "PxGpuCudaRegisterFatBinary");
			g_PxGetCudaFunctionTable_Func  = (PxGetCudaFunctionTable_FUNC*)GetProcAddress(s_library, "PxGpuGetCudaFunctionTable");
			g_PxGetCudaFunctionTableSize_Func = (PxGetCudaFunctionTableSize_FUNC*)GetProcAddress(s_library, "PxGpuGetCudaFunctionTableSize");
			g_PxGetCudaModuleTableSize_Func = (PxGetCudaFunctionTableSize_FUNC*)GetProcAddress(s_library, "PxGpuGetCudaModuleTableSize");
			g_PxGetCudaModuleTable_Func = (PxGetCudaModuleTable_FUNC*)GetProcAddress(s_library, "PxGpuGetCudaModuleTable");
			g_PxCreatePhysicsGpu_Func = (PxCreatePhysicsGpu_FUNC*)GetProcAddress(s_library, "PxGpuCreatePhysicsGpu");
		}

		// Check for errors
		if (s_library == NULL)
		{
			reportError(PX_FL, "Failed to load %s!\n", gPhysXGpuLibraryName);
			return;
		}

		if (g_PxCreatePhysXGpu_Func == NULL || g_PxCreateCudaContextManager_Func == NULL || g_PxGetSuggestedCudaDeviceOrdinal_Func == NULL || g_PxSetPhysXGpuProfilerCallback_Func == NULL)
		{
			reportError(PX_FL, "PhysXGpu dll is incompatible with this version of PhysX!\n");
			return;
		}
	}

#elif PX_LINUX

	void PxLoadPhysxGPUModule(const char*)
	{
		static void* s_library = NULL;

		if (s_library == NULL)
		{
			// load libcuda.so here since gcc configured with --as-needed won't link to it
			// if there is no call from the binary to it.
			void* hLibCuda = dlopen("libcuda.so", RTLD_NOW | RTLD_GLOBAL);
			if (hLibCuda)
			{
				s_library = dlopen(gPhysXGpuLibraryName, RTLD_NOW);
			}
			else
			{
				char* error = dlerror();
				reportError(PX_FL, "Could not load libcuda.so: %s\n", error);
				return;
			}	
		}

		// no UpdateLoader
		if (s_library)
		{
			*reinterpret_cast<void**>(&g_PxCreatePhysXGpu_Func) = dlsym(s_library, "PxCreatePhysXGpu");
			*reinterpret_cast<void**>(&g_PxCreateCudaContextManager_Func) = dlsym(s_library, "PxCreateCudaContextManager");
			*reinterpret_cast<void**>(&g_PxGetSuggestedCudaDeviceOrdinal_Func) = dlsym(s_library, "PxGetSuggestedCudaDeviceOrdinal");
			*reinterpret_cast<void**>(&g_PxSetPhysXGpuProfilerCallback_Func) = dlsym(s_library, "PxSetPhysXGpuProfilerCallback");
			*reinterpret_cast<void**>(&g_PxCudaRegisterFunction_Func) = dlsym(s_library, "PxGpuCudaRegisterFunction");
			*reinterpret_cast<void**>(&g_PxCudaRegisterFatBinary_Func) = dlsym(s_library, "PxGpuCudaRegisterFatBinary");
			*reinterpret_cast<void**>(&g_PxGetCudaFunctionTable_Func)  = dlsym(s_library, "PxGpuGetCudaFunctionTable");
			*reinterpret_cast<void**>(&g_PxGetCudaFunctionTableSize_Func) = dlsym(s_library, "PxGpuGetCudaFunctionTableSize");
			*reinterpret_cast<void**>(&g_PxGetCudaModuleTableSize_Func) = dlsym(s_library, "PxGpuGetCudaModuleTableSize");
			*reinterpret_cast<void**>(&g_PxGetCudaModuleTable_Func) = dlsym(s_library, "PxGpuGetCudaModuleTable");
			*reinterpret_cast<void**>(&g_PxCreatePhysicsGpu_Func) = dlsym(s_library, "PxGpuCreatePhysicsGpu");
		}

		// Check for errors
		if (s_library == NULL)
		{
			char* error = dlerror();
			reportError(PX_FL, "Failed to load %s!: %s\n", gPhysXGpuLibraryName, error);
			return;
		}
		if (g_PxCreatePhysXGpu_Func == NULL || g_PxCreateCudaContextManager_Func == NULL || g_PxGetSuggestedCudaDeviceOrdinal_Func == NULL)
		{
			reportError(PX_FL, "%s is incompatible with this version of PhysX!\n", gPhysXGpuLibraryName);
			return;
		}
	}

#endif // PX_LINUX

} // end physx namespace

#if PX_CLANG
#pragma clang diagnostic pop
#endif

#endif // PX_SUPPORT_GPU_PHYSX
