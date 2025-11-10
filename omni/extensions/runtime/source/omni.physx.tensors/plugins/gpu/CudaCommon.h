// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/logging/Log.h>

#include "common/utilities/CudaHelpers.h"

#include <cuda.h>
#include <cuda_runtime.h>

namespace omni
{
namespace physx
{
namespace tensors
{

extern bool g_forceCudaDeviceSync;

#define CHECK_CUDA(code) cudaHelpers::checkCuda(code, __FILE__, __LINE__)
#define CHECK_CU(code) cudaHelpers::checkCu(code, __FILE__, __LINE__)

inline bool validateCudaContext(const char* file, int line)
{
    // Lovingly borrowed from PhysX CudaContextManager.cpp

    // Verify we can at least create a CUDA event in this context
    CUevent evt = nullptr;
    if (cudaHelpers::checkCu(cuEventCreate(&evt, CU_EVENT_DISABLE_TIMING), file, line))
    {
        cuEventDestroy(evt);
        return true;
    }
    else
    {
        CARB_LOG_ERROR("CUDA context validation failed");
        return false;
    }
}

inline void synchronizeCuda(const char* file, int line)
{
    if (g_forceCudaDeviceSync)
    {
        cudaHelpers::checkCuda(cudaDeviceSynchronize(), file, line);
    }
}

#define VALIDATE_CUDA_CONTEXT() validateCudaContext(__FILE__, __LINE__)
#define SYNCHRONIZE_CUDA() synchronizeCuda(__FILE__, __LINE__)


/**
    \brief Helper function to allocate device memory and initializing it with either existing host data or 0
    \param[in] dst is a pointer to the device pointer.
    \param[in] src is the host pointer to the data that needs to be copied. If the pointer is null then no copying is
   done \param[in] size is the size of the data to be allocated and/or copied from the host \param[in] name is a string
   to identify the buffer for debugging purposes \note if src is nullptr this function just zero initialize the device
   buffer bytes by 0
*/
inline bool prepareDeviceData(void** dst, const void* src, size_t size, const char* name)
{
    if (size == 0)
        CARB_LOG_ERROR("Zero memory size requested for %s allocation is incorrect", name);

    if (!CHECK_CUDA(cudaMalloc(dst, size)))
    {
        CARB_LOG_ERROR("Unable to allocate memory of size %zu for %s", size, name);
        return false;
    }
    if (src)
    {
        if (!CHECK_CUDA(cudaMemcpy(*dst, src, size, cudaMemcpyHostToDevice)))
        {
            CARB_LOG_ERROR("Unable to copy data from host to device for %s", name);
            return false;
        }
    }
    else if (size != 0)
    {
        if (!CHECK_CUDA(cudaMemset(*dst, 0, size)))
        {
            CARB_LOG_ERROR("Unable to initialize device memory for %s", name);
            return false;
        }
    }
    return true;
}

inline bool createPrimaryCudaContext()
{
    // Trigger primary context creation using the runtime API.
    // This assumes that cudaSetDevice has been called already.
    //
    // This should be compatible with PyTorch, since PyTorch uses the primary context on each device with the runtime
    // API. Thus, if we do this here, there should be no need to create the primary context in PyTorch beforehand.
    //
    // CUDA ref: https://docs.nvidia.com/cuda/cuda-runtime-api/group__CUDART__DRIVER.html
    //
    // PyTorch ref: https://github.com/pytorch/pytorch/issues/10832#issuecomment-419341200

    void* dummy = nullptr;
    if (CHECK_CUDA(cudaMalloc(&dummy, 4)))
    {
        cudaFree(dummy);
        return VALIDATE_CUDA_CONTEXT();
    }
    else
    {
        CARB_LOG_ERROR("Failed to create primary CUDA context");
        return false;
    }
}

class CudaContextGuard
{
public:
    explicit CudaContextGuard(CUcontext ctx) : mPushedCtx(0)
    {
        CUcontext oldCtx;
        if (ctx && CHECK_CU(cuCtxGetCurrent(&oldCtx)))
        {
            if (ctx != oldCtx && CHECK_CU(cuCtxPushCurrent(ctx)))
            {
                mPushedCtx = ctx;
            }
        }
    }

    ~CudaContextGuard()
    {
        if (mPushedCtx)
        {
            CUcontext ctx;
            CHECK_CU(cuCtxPopCurrent(&ctx));
        }
    }

private:
    CUcontext mPushedCtx;
};

} // namespace tensors
} // namespace physx
} // namespace omni
