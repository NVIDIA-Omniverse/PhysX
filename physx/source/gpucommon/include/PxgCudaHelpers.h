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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_CUDA_HELPERS_H
#define PXG_CUDA_HELPERS_H

#include "foundation/PxPreprocessor.h"

#if PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#endif
#include <cuda.h>
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic pop
#endif

#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "foundation/PxFoundation.h"

namespace physx
{

/**
*   Templated static functions to simplify common CUDA operations.
*
*   General guidelines:
*
*   - If you want automatic context acquisition, use the functions that take the context manager as a parameter.
*   - Otherwise, directly use the ones that take a cudaContext, but don't forget to acquire the context.
*   - For allocations/deallocations, see PxgCudaMemoryAllocator. Use the functions provided there.
*   - For anything outside the core SDK, use the helpers provided in the extensions.
*
*/

class PxgCudaHelpers
{

public:
    /**
	* \brief Copies a device buffer to the host
	*
	* The cuda context needs to be acquired by the user!
	*/
	template<typename T>
	static void copyDToH(PxCudaContext& cudaContext, T* hostBuffer, const T* deviceBuffer, PxU64 numElements)
	{
        if (!deviceBuffer || !hostBuffer)
    		return;

        PxU64 numBytes = numElements * sizeof(T);
	    PxCUresult result = cudaContext.memcpyDtoH(hostBuffer, CUdeviceptr(deviceBuffer), numBytes);
	    if (result != CUDA_SUCCESS)
	        PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyDtoH set failed with error code %i!\n", PxI32(result));
	}

	/**
	* \brief Copies a device buffer to the host
	*
	* The cuda context will get acquired automatically
	*/
	template<typename T>
	static void copyDToH(PxCudaContextManager& cudaContextManager, T* hostBuffer, const T* deviceBuffer, PxU64 numElements)
	{
        PxScopedCudaLock _lock(cudaContextManager);
		copyDToH(*cudaContextManager.getCudaContext(), hostBuffer, deviceBuffer, numElements);
	}

    /**
	* \brief Copies a host buffer to the device
	*
	* The cuda context needs to be acquired by the user!
	*/
	template<typename T>
	static void copyHToD(PxCudaContext& cudaContext, T* deviceBuffer, const T* hostBuffer, PxU64 numElements)
	{
		if (!deviceBuffer || !hostBuffer)
		    return;

        PxU64 numBytes = numElements * sizeof(T);

	    PxCUresult result = cudaContext.memcpyHtoD(CUdeviceptr(deviceBuffer), hostBuffer, numBytes);
	    if (result != CUDA_SUCCESS)
	    	PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyHtoD set failed with error code %i!\n", PxI32(result));
	}

	/**
	* \brief Copies a host buffer to the device
	*
	* The cuda context will get acquired automatically
	*/
	template<typename T>
	static void copyHToD(PxCudaContextManager& cudaContextManager, T* deviceBuffer, const T* hostBuffer, PxU64 numElements)
	{
        PxScopedCudaLock _lock(cudaContextManager);
		copyHToD(*cudaContextManager.getCudaContext(), deviceBuffer, hostBuffer, numElements);
	}

    /**
	* \brief Schedules device to host copy operation on the specified stream
	*
	* The cuda context needs to be acquired by the user!
	*/
	template<typename T>
	static void copyDToHAsync(PxCudaContext& cudaContext, T* hostBuffer, const T* deviceBuffer, PxU64 numElements, CUstream stream)
	{
        if (!deviceBuffer || !hostBuffer)
		    return;

        PxU64 numBytes = numElements * sizeof(T);
	    PxCUresult result = cudaContext.memcpyDtoHAsync(hostBuffer, CUdeviceptr(deviceBuffer), numBytes, stream);
	    if (result != CUDA_SUCCESS)
	    	PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyDtoHAsync set failed with error code %i!\n", PxI32(result));
	}

	/**
	* \brief Schedules device to host copy operation on the specified stream
	*
	* The cuda context will get acquired automatically
	*/
	template<typename T>
	static void copyDToHAsync(PxCudaContextManager& cudaContextManager, T* hostBuffer, const T* deviceBuffer, PxU64 numElements, CUstream stream)
	{
        PxScopedCudaLock _lock(cudaContextManager);
		copyDToHAsync(*cudaContextManager.getCudaContext(), hostBuffer, deviceBuffer, numElements, stream);
	}

    /**
	* \brief Schedules host to device copy operation on the specified stream
	*
	* The cuda context needs to be acquired by the user!
	*/
	template<typename T>
	static void copyHToDAsync(PxCudaContext& cudaContext, T* deviceBuffer, const T* hostBuffer, PxU64 numElements, CUstream stream)
	{
		if (!deviceBuffer || !hostBuffer)
		    return;

        PxU64 numBytes = numElements * sizeof(T);

	    PxCUresult result = cudaContext.memcpyHtoDAsync(CUdeviceptr(deviceBuffer), hostBuffer, numBytes, stream);
	    if (result != CUDA_SUCCESS)
	    	PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyHtoDAsync set failed with error code %i!\n", PxI32(result));
	}

	/**
	* \brief Schedules host to device copy operation on the specified stream
	*
	* The cuda context will get acquired automatically
	*/
	template<typename T>
	static void copyHToDAsync(PxCudaContextManager& cudaContextManager, T* deviceBuffer, const T* hostBuffer, PxU64 numElements, CUstream stream)
	{
        PxScopedCudaLock _lock(cudaContextManager);
		copyHToDAsync(*cudaContextManager.getCudaContext(), deviceBuffer, hostBuffer, numElements, stream);
	}

    /**
	* \brief Schedules device to device copy operation on the specified stream
	*
	* The cuda context needs to be acquired by the user!
	*/
	template<typename T>
	static void copyDToDAsync(PxCudaContext& cudaContext, T* dstDeviceBuffer, const T* srcDeviceBuffer, PxU64 numElements, CUstream stream)
	{
		if (!srcDeviceBuffer || !dstDeviceBuffer)
		    return;

        PxU64 numBytes = numElements * sizeof(T);

	    PxCUresult result = cudaContext.memcpyDtoDAsync(CUdeviceptr(dstDeviceBuffer), CUdeviceptr(srcDeviceBuffer), numBytes, stream);
	    if (result != CUDA_SUCCESS)
		    PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "copyDtoDAsync set failed with error code %i!\n", PxI32(result));
	}

	/**
	* \brief Schedules device to device copy operation on the specified stream
	*
	* The cuda context will get acquired automatically
	*/
	template<typename T>
	static void copyDToDAsync(PxCudaContextManager& cudaContextManager, T* dstDeviceBuffer, const T* srcDeviceBuffer, PxU64 numElements, CUstream stream)
	{
        PxScopedCudaLock _lock(cudaContextManager);
		copyDToDAsync(*cudaContextManager.getCudaContext(), dstDeviceBuffer, srcDeviceBuffer, numElements * sizeof(T), stream);
	}

    /**
	* \brief Schedules a memset operation on the device on the specified stream. Only supported for 1 and 4 byte types.
	*
	* The cuda context needs to be acquired by the user!
	*/
	template<typename T>
	static void memsetAsync(PxCudaContext& cudaContext, T* dstDeviceBuffer, const T& value, PxU64 numElements, CUstream stream)
	{
		PX_COMPILE_TIME_ASSERT(sizeof(T) == sizeof(PxU32) || sizeof(T) == sizeof(PxU8));

		if (!dstDeviceBuffer)
            return;

        PxU64 numBytes = numElements * sizeof(T);

        PxCUresult result = CUDA_SUCCESS;
	    if (sizeof(T) == sizeof(PxU32))
            result = cudaContext.memsetD32Async(CUdeviceptr(dstDeviceBuffer), reinterpret_cast<const PxU32&>(value), numBytes >> 2, stream);
	    else
            result = cudaContext.memsetD8Async(CUdeviceptr(dstDeviceBuffer), reinterpret_cast<const PxU8&>(value), numBytes, stream);

        if (result != CUDA_SUCCESS)
		    PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Memset failed with error code %i!\n", PxI32(result));
	}


	/**
	* \brief Schedules a memset operation on the device on the specified stream. Only supported for 1 byte or 4 byte data types.
	*
	* The cuda context will get acquired automatically
	*/
	template<typename T>
	static void memsetAsync(PxCudaContextManager& cudaContextManager, T* dstDeviceBuffer, const T& value, PxU64 numElements, CUstream stream)
	{
		PxScopedCudaLock _lock(cudaContextManager);
        memsetAsync(*cudaContextManager.getCudaContext(), dstDeviceBuffer, value, numElements, stream);		
	}
};

}; // namespace physx
#endif