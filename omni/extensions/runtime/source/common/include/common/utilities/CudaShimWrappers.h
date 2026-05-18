// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Typed safe wrappers for IOptionalCuda functions that write through out-pointers.
//
// These avoid strict-aliasing UB (writing uintptr_t through CUevent*/CUdeviceptr*/etc.)
// by using a uintptr_t temporary, and log the CUDA status code on failure.
//
// Requires <cuda.h> for driver types (CUdeviceptr, CUevent, CUstream, CUcontext).
// Requires OptionalCudaShim.h for optionalCuda::get().

#pragma once

#include <common/utilities/OptionalCudaShim.h>

#include <carb/logging/Log.h>

#include <cuda.h>

namespace omni
{
namespace physx
{
namespace cudaShimWrappers
{

// -- getCudaShim() accessor (single definition) --

static inline omni::physx::IOptionalCuda* getCudaShim()
{
    return omni::physx::optionalCuda::get();
}

// -- Bool-returning check helper + macro --

static inline bool checkCudaShimOp(bool ok, const char* file, int line)
{
    if (!ok)
    {
        CARB_LOG_ERROR("CUDA driver shim call failed at %s:%d", file, line);
    }
    return ok;
}

#define CHECK_CUDA_SHIM(expr) omni::physx::cudaShimWrappers::checkCudaShimOp(expr, __FILE__, __LINE__)

// -- Typed safe wrappers --

static inline bool shimCuMemAlloc(CUdeviceptr* out, size_t bytes, const char* file, int line)
{
    if (out) *out = 0;
    uintptr_t tmp = 0;
    int st = 0;
    bool ok = getCudaShim()->memAlloc(&tmp, bytes, &st);
    if (!ok)
        CARB_LOG_ERROR("cuMemAlloc failed (CUresult %d) at %s:%d", st, file, line);
    if (out) *out = static_cast<CUdeviceptr>(tmp);
    return ok;
}

static inline bool shimCuEventCreate(CUevent* out, unsigned int flags, const char* file, int line)
{
    if (out) *out = nullptr;
    uintptr_t tmp = 0;
    int st = 0;
    bool ok = getCudaShim()->eventCreate(&tmp, flags, &st);
    if (!ok)
        CARB_LOG_ERROR("cuEventCreate failed (CUresult %d) at %s:%d", st, file, line);
    if (out) *out = reinterpret_cast<CUevent>(tmp);
    return ok;
}

static inline bool shimCuStreamCreate(CUstream* out, unsigned int flags, const char* file, int line)
{
    if (out) *out = nullptr;
    uintptr_t tmp = 0;
    int st = 0;
    bool ok = getCudaShim()->streamCreate(&tmp, flags, &st);
    if (!ok)
        CARB_LOG_ERROR("cuStreamCreate failed (CUresult %d) at %s:%d", st, file, line);
    if (out) *out = reinterpret_cast<CUstream>(tmp);
    return ok;
}

static inline bool shimCuCtxGetCurrent(CUcontext* out, const char* file, int line)
{
    if (out) *out = nullptr;
    uintptr_t tmp = 0;
    int st = 0;
    bool ok = getCudaShim()->ctxGetCurrent(&tmp, &st);
    if (!ok)
        CARB_LOG_ERROR("cuCtxGetCurrent failed (CUresult %d) at %s:%d", st, file, line);
    if (out) *out = reinterpret_cast<CUcontext>(tmp);
    return ok;
}

static inline bool shimCuCtxPopCurrent(CUcontext* out, const char* file, int line)
{
    if (out) *out = nullptr;
    uintptr_t tmp = 0;
    int st = 0;
    bool ok = getCudaShim()->ctxPopCurrent(&tmp, &st);
    if (!ok)
        CARB_LOG_ERROR("cuCtxPopCurrent failed (CUresult %d) at %s:%d", st, file, line);
    if (out) *out = reinterpret_cast<CUcontext>(tmp);
    return ok;
}

// -- Convenience macros that inject __FILE__, __LINE__ --

#define SHIM_CU_ALLOC(out, bytes) omni::physx::cudaShimWrappers::shimCuMemAlloc(out, bytes, __FILE__, __LINE__)
#define SHIM_CU_EVENT_CREATE(out, flags) omni::physx::cudaShimWrappers::shimCuEventCreate(out, flags, __FILE__, __LINE__)
#define SHIM_CU_STREAM_CREATE(out, flags) omni::physx::cudaShimWrappers::shimCuStreamCreate(out, flags, __FILE__, __LINE__)
#define SHIM_CU_CTX_GET_CURRENT(out) omni::physx::cudaShimWrappers::shimCuCtxGetCurrent(out, __FILE__, __LINE__)
#define SHIM_CU_CTX_POP_CURRENT(out) omni::physx::cudaShimWrappers::shimCuCtxPopCurrent(out, __FILE__, __LINE__)

} // namespace cudaShimWrappers
} // namespace physx
} // namespace omni
