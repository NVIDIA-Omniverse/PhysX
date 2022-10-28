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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.

#include "PxPhysXConfig.h"

#if PX_SUPPORT_GPU_PHYSX

#include "gpu/PxGpu.h"

#ifndef PX_PHYSX_GPU_STATIC

namespace physx
{
	//forward declare stuff from PxPhysXGpuModuleLoader.cpp
	void PxLoadPhysxGPUModule(const char* appGUID);

	typedef physx::PxCudaContextManager* (PxCreateCudaContextManager_FUNC)(physx::PxFoundation& foundation, const physx::PxCudaContextManagerDesc& desc, physx::PxProfilerCallback* profilerCallback);
	typedef int (PxGetSuggestedCudaDeviceOrdinal_FUNC)(physx::PxErrorCallback& errc);
	typedef void (PxSetPhysXGpuProfilerCallback_FUNC)(physx::PxProfilerCallback* cbk);
	typedef void (PxCudaRegisterFunction_FUNC)(int, const char*);
	typedef void** (PxCudaRegisterFatBinary_FUNC)(void*);
	typedef physx::PxKernelIndex* (PxGetCudaFunctionTable_FUNC)();
	typedef PxU32 (PxGetCudaFunctionTableSize_FUNC)();
	typedef void** PxGetCudaModuleTable_FUNC();

	extern PxCreateCudaContextManager_FUNC*  g_PxCreateCudaContextManager_Func;
	extern PxGetSuggestedCudaDeviceOrdinal_FUNC* g_PxGetSuggestedCudaDeviceOrdinal_Func;
	extern PxSetPhysXGpuProfilerCallback_FUNC* g_PxSetPhysXGpuProfilerCallback_Func;
	extern PxCudaRegisterFunction_FUNC* g_PxCudaRegisterFunction_Func;
	extern PxCudaRegisterFatBinary_FUNC* g_PxCudaRegisterFatBinary_Func;
	extern PxGetCudaFunctionTable_FUNC* g_PxGetCudaFunctionTable_Func;
	extern PxGetCudaFunctionTableSize_FUNC* g_PxGetCudaFunctionTableSize_Func;
	extern PxGetCudaFunctionTableSize_FUNC* g_PxGetCudaModuleTableSize_Func;
	extern PxGetCudaModuleTable_FUNC* g_PxGetCudaModuleTable_Func;

} // end of physx namespace



physx::PxCudaContextManager* PxCreateCudaContextManager(physx::PxFoundation& foundation, const physx::PxCudaContextManagerDesc& desc, physx::PxProfilerCallback* profilerCallback)
{
	if (!physx::g_PxCreateCudaContextManager_Func)
		physx::PxLoadPhysxGPUModule(desc.appGUID);

	if (physx::g_PxCreateCudaContextManager_Func)
		return physx::g_PxCreateCudaContextManager_Func(foundation, desc, profilerCallback);
	else
		return NULL;
}

int PxGetSuggestedCudaDeviceOrdinal(physx::PxErrorCallback& errc)
{
	if (!physx::g_PxGetSuggestedCudaDeviceOrdinal_Func)
		physx::PxLoadPhysxGPUModule(NULL);

	if (physx::g_PxGetSuggestedCudaDeviceOrdinal_Func)
		return physx::g_PxGetSuggestedCudaDeviceOrdinal_Func(errc);
	else
		return -1;
}

void PxSetPhysXGpuProfilerCallback(physx::PxProfilerCallback* profilerCallback)
{
	if (!physx::g_PxSetPhysXGpuProfilerCallback_Func)
		physx::PxLoadPhysxGPUModule(NULL);

	if (physx::g_PxSetPhysXGpuProfilerCallback_Func)
		physx::g_PxSetPhysXGpuProfilerCallback_Func(profilerCallback);
}

void PxCudaRegisterFunction(int moduleIndex, const char* functionName)
{
	if (!physx::g_PxCudaRegisterFunction_Func)
		physx::PxLoadPhysxGPUModule(NULL);

	if (physx::g_PxCudaRegisterFunction_Func)
		physx::g_PxCudaRegisterFunction_Func(moduleIndex, functionName);
}

void** PxCudaRegisterFatBinary(void* fatBin)
{
	if (!physx::g_PxCudaRegisterFatBinary_Func)
		physx::PxLoadPhysxGPUModule(NULL);

	if (physx::g_PxCudaRegisterFatBinary_Func)
		return physx::g_PxCudaRegisterFatBinary_Func(fatBin);

	return NULL;
}


physx::PxKernelIndex* PxGetCudaFunctionTable()
{
	if(!physx::g_PxGetCudaFunctionTable_Func)
		physx::PxLoadPhysxGPUModule(NULL);

	if(physx::g_PxGetCudaFunctionTable_Func)
		return physx::g_PxGetCudaFunctionTable_Func();

	return NULL;
}

physx::PxU32 PxGetCudaFunctionTableSize() {

	if(!physx::g_PxGetCudaFunctionTableSize_Func)
		physx::PxLoadPhysxGPUModule(NULL);

	if(physx::g_PxGetCudaFunctionTableSize_Func)
		return physx::g_PxGetCudaFunctionTableSize_Func();

	return 0;
}

void** PxGetCudaModuleTable() {
	if(!physx::g_PxGetCudaModuleTable_Func)
		physx::PxLoadPhysxGPUModule(NULL);

	if(physx::g_PxGetCudaModuleTable_Func)
		return physx::g_PxGetCudaModuleTable_Func();

	return NULL;
}


physx::PxU32 PxGetCudaModuleTableSize()
{
	if(!physx::g_PxGetCudaModuleTableSize_Func)
		physx::PxLoadPhysxGPUModule(NULL);

	if(physx::g_PxGetCudaModuleTableSize_Func)
		return physx::g_PxGetCudaModuleTableSize_Func();

	return 0;
}


#endif // PX_PHYSX_GPU_STATIC

#endif // PX_SUPPORT_GPU_PHYSX

