// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <stdio.h>

#include "foundation/PxPreprocessor.h"
#include "foundation/PxAssert.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxString.h"

// from the point of view of this source file the GPU library is linked statically
#ifndef PX_PHYSX_GPU_STATIC
	#define PX_PHYSX_GPU_STATIC
#endif
#include "PxPhysXGpu.h"

#if PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdisabled-macro-expansion"
#endif
#include <cuda.h>
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic pop
#endif

#include <texture_types.h>
#include <vector_types.h>

#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"

#include "CudaKernelWrangler.h"

using namespace physx;

const char* KernelWrangler::getCuFunctionName(uint16_t funcIndex) const
{
	// PT: this is wrong, functions there are not listed in the same order as mCuFunctions
	//return gFunctionTable[funcIndex].functionName;
	return mKernelNames[funcIndex];
}

KernelWrangler::KernelWrangler(PxCudaContextManager& cudaContextManager, PxErrorCallback& errorCallback, const char** funcNames, uint16_t numFuncs)
	: mError(false)
	, mKernelNames(NULL)
	, mCuFunctions("CuFunctions")
	, mCudaContextManager(cudaContextManager)
	, mCudaContext(cudaContextManager.getCudaContext())
	, mErrorCallback(errorCallback)
{
	// PT: TODO: consider revisiting this code so that the function order is kept the same between mCuFunctions and gFunctionTable.
	// That way the initial getCuFunctionName implementation could be kept and we could decouple the code from the external funcNames array again.

	// PT: proper names defined in PxgKernelWrangler.cpp. We assume the data there remains valid for the lifetime of the app and we don't need to do a copy.
	PX_ASSERT(funcNames);
	mKernelNames = funcNames;

	// matchup funcNames to CUDA modules, get CUfunction handles
	CUmodule* cuModules = cudaContextManager.getCuModules();
	PxKernelIndex* cuFunctionTable = PxGpuGetCudaFunctionTable();
	const PxU32 cuFunctionTableSize = PxGpuGetCudaFunctionTableSize();

	mCuFunctions.resize(numFuncs, NULL);

	if (mCudaContextManager.tryAcquireContext())
	{
		for (uint32_t i = 0; i < numFuncs; ++i)
		{
			// search through all known functions
			for (uint32_t j = 0; ; ++j)
			{
				if (j == cuFunctionTableSize)
				{
					// printf("Could not find registered CUDA function '%s'.\n", funcNames[i]);

					char buffer[256];
					Pxsnprintf(buffer, 256, "Could not find registered CUDA function '%s'.", funcNames[i]);
					mErrorCallback.reportError(PxErrorCode::eINTERNAL_ERROR, buffer, PX_FL);
					mError = true;
					break;
				}


				if (!Pxstrcmp(cuFunctionTable[j].functionName, funcNames[i]))
				{
					PxCUresult ret = mCudaContext->moduleGetFunction(&mCuFunctions[i], cuModules[cuFunctionTable[j].moduleIndex], funcNames[i]);
					if (ret != CUDA_SUCCESS)
					{
						char buffer[256];
						Pxsnprintf(buffer, 256, "Could not find CUDA module containing function '%s'.", funcNames[i]);
						mErrorCallback.reportError(PxErrorCode::eINTERNAL_ERROR, buffer, PX_FL);
						mError = true;
						// return;
					}
					break;
				}
			}
		}
		mCudaContextManager.releaseContext();
	}
	else 
	{
		char buffer[256];
		Pxsnprintf(buffer, 256, "Failed to acquire the cuda context.");
		mErrorCallback.reportError(PxErrorCode::eINTERNAL_ERROR, buffer, PX_FL);
		mError = true;
	}
}

/*
 * Workaround hacks for using nvcc --compiler output object files
 * without linking with CUDART.  We must implement our own versions
 * of these functions that the object files are hard-coded to call into.
 * These calls are all made _before_ main() during static initialization
 * of this DLL.
 */

#include <driver_types.h>

#if PX_WINDOWS_FAMILY
#define CUDARTAPI __stdcall
#endif

struct uint3;
struct dim3;

extern "C"
void** CUDARTAPI __cudaRegisterFatBinary(void* fatBin)
{
	return PxGpuCudaRegisterFatBinary(fatBin);
}

extern "C"
void CUDARTAPI __cudaRegisterFatBinaryEnd(void ** /*fatCubinHandle*/)
{
}

extern "C"
void CUDARTAPI __cudaUnregisterFatBinary(void** fatCubinHandle)
{
	// jcarius: not ideal because the module may still be loaded
	PxGpuGetCudaModuleTable()[(int)(size_t) fatCubinHandle] = 0;
}

extern "C"
void CUDARTAPI __cudaRegisterTexture(void**, const struct textureReference*, const void**, const char*, int, int, int)
{
}

extern "C" void CUDARTAPI __cudaRegisterVar(void**, char*, char*, const char*, int, int, int, int)
{
}


extern "C" void CUDARTAPI __cudaRegisterShared(void**, void**)
{
}

extern "C"
void CUDARTAPI __cudaRegisterFunction(void** fatCubinHandle, const char*, 
	char*, const char* deviceName, int, uint3*, uint3*, dim3*, dim3*, int*)
{
	PxGpuCudaRegisterFunction((int)(size_t) fatCubinHandle, deviceName);
}

/* These functions are implemented just to resolve link dependencies */

extern "C"
cudaError_t CUDARTAPI cudaLaunch(const char* entry)
{
	PX_UNUSED(entry);
	return cudaSuccess;
}

extern "C"
cudaError_t CUDARTAPI cudaLaunchKernel( const void* , dim3 , dim3 , void** , size_t , cudaStream_t  ) 
{
	return cudaSuccess;
}

extern "C"
cudaError_t CUDARTAPI cudaSetupArgument(const void*, size_t, size_t)
{
	return cudaSuccess;
}

extern "C"
struct cudaChannelFormatDesc CUDARTAPI cudaCreateChannelDesc(
    int x, int y, int z, int w, enum cudaChannelFormatKind f)
{
	struct cudaChannelFormatDesc desc;
	desc.x = x;
	desc.y = y;
	desc.z = z;
	desc.w = w;
	desc.f = f;
	return desc;
}

extern "C" 
cudaError_t CUDARTAPI __cudaPopCallConfiguration(
  dim3         *,
  dim3         *,
  size_t       *,
  void         *)
{
	return cudaSuccess;
}