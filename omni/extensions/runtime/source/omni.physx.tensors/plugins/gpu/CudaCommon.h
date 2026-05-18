// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/logging/Log.h>

// NOTE: CudaHelpers.h is intentionally NOT included here. It contains static
// functions (queryAllCudaDevices, selectBestPhysicsDevice) that reference cu*
// driver API symbols directly. Since omni.physx.tensors no longer links libcuda
// (all driver calls go through IOptionalCuda), including it causes link errors.
// We only need checkCuda() (CUDA runtime API) which is inlined below.
#include <common/utilities/CudaShimWrappers.h>

#include <cuda.h>
#include <cuda_runtime.h>

namespace omni
{
namespace physx
{

// Inline CUDA runtime error checker (uses cudart_static, no driver API link needed).
// Duplicated from CudaHelpers.h to avoid pulling in its static driver-API functions.
namespace cudaHelpers
{
inline bool checkCuda(cudaError_t code, const char* file, int line)
{
    if (code != cudaSuccess)
    {
        CARB_LOG_ERROR("CUDA error: %s: %s: %d", cudaGetErrorString(code), file, line);
        return false;
    }
    return true;
}
} // namespace cudaHelpers

namespace tensors
{

extern bool g_forceCudaDeviceSync;

using omni::physx::cudaShimWrappers::getCudaShim;
using omni::physx::cudaShimWrappers::shimCuEventCreate;

#define CHECK_CUDA(code) cudaHelpers::checkCuda(code, __FILE__, __LINE__)

// CHECK_CU wraps bool-returning IOptionalCuda shim calls (replaces old CUresult-based CHECK_CU).
// For shim calls that write through an out-pointer (memAlloc, eventCreate, streamCreate,
// ctxGetCurrent, ctxPopCurrent), use the SHIM_CU_* macros from CudaShimWrappers.h.
#define CHECK_CU(expr) CHECK_CUDA_SHIM(expr)

inline bool validateCudaContext(const char* file, int line)
{
    // Lovingly borrowed from PhysX CudaContextManager.cpp

    // Verify we can at least create a CUDA event in this context
    CUevent evt = nullptr;
    if (shimCuEventCreate(&evt, CU_EVENT_DISABLE_TIMING, file, line))
    {
        getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(evt), nullptr);
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
    // Trigger primary context creation using the CUDA runtime API (cudaMalloc/cudaFree).
    // NOTE: This intentionally uses the runtime API (linked via cudart_static) rather than
    // the IOptionalCuda driver shim. The runtime API is the standard mechanism for
    // creating the primary context, and cudart_static is statically linked (no DT_NEEDED).
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
    explicit CudaContextGuard(CUcontext ctx) : mPushedCtx(nullptr)
    {
        CUcontext oldCtx = nullptr;
        if (ctx && SHIM_CU_CTX_GET_CURRENT(&oldCtx))
        {
            if (ctx != oldCtx && CHECK_CU(getCudaShim()->ctxPushCurrent(reinterpret_cast<uintptr_t>(ctx), nullptr)))
            {
                mPushedCtx = ctx;
            }
        }
    }

    ~CudaContextGuard()
    {
        if (mPushedCtx)
        {
            CUcontext popped = nullptr;
            SHIM_CU_CTX_POP_CURRENT(&popped);
        }
    }

private:
    CUcontext mPushedCtx;
};

} // namespace tensors
} // namespace physx
} // namespace omni
