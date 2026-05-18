// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Optional CUDA driver API interface.
//
// Provides runtime-loaded CUDA driver API wrappers so that plugins can call CUDA
// functions without linking libcuda (no DT_NEEDED libcuda.so.1). This allows
// plugins to load on CPU-only machines where the NVIDIA driver is absent.
//
// Design rules:
// - NO CUDA types (CUdeviceptr, CUcontext, etc.) -- only uintptr_t, int, size_t.
//   This allows consumers to include this header without cuda.h.
// - Only wraps cu* functions actually used by fabric, tensors, and ovphysx today.
//   Do not speculatively add wrappers for unused driver API functions.
// - If new wrappers are needed in the future, bump the minor version.
//
// Implemented by omni.physx.foundation.plugin via an internal dlopen-based shim.
// Consumers acquire via: carb::getFramework()->tryAcquireInterface<IOptionalCuda>()

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h> // CARB_PLUGIN_INTERFACE
#include <cstddef>
#include <cstdint>

namespace omni
{
namespace physx
{

struct IOptionalCuda
{
    // Initial version for the public optional CUDA interface.
    CARB_PLUGIN_INTERFACE("omni::physx::IOptionalCuda", 0, 1)

    // -- Availability ------------------------------------------------------
    // Returns true if a usable NVIDIA driver + at least one CUDA device is available.
    // Cached and thread-safe. If false, all other functions return failure.
    bool(CARB_ABI* cudaAvailable)();

    // -- Context management ------------------------------------------------
    // All context handles are represented as uintptr_t (CUcontext is a pointer type).
    // outStatus receives the CUresult value (0 = CUDA_SUCCESS) if non-null.
    bool(CARB_ABI* ctxGetCurrent)(uintptr_t* outCtx, int* outStatus);
    bool(CARB_ABI* ctxPushCurrent)(uintptr_t ctx, int* outStatus);
    bool(CARB_ABI* ctxPopCurrent)(uintptr_t* outCtx, int* outStatus);
    bool(CARB_ABI* ctxGetDevice)(int* outDevice, int* outStatus);
    bool(CARB_ABI* ctxSynchronize)(int* outStatus);

    // -- Device queries ----------------------------------------------------
    bool(CARB_ABI* deviceGetCount)(int* outCount, int* outStatus);
    bool(CARB_ABI* deviceGet)(int* outDevice, int ordinal, int* outStatus);

    // -- Memory management -------------------------------------------------
    // Device pointers are represented as uintptr_t (CUdeviceptr is a uint64_t / unsigned long long).
    bool(CARB_ABI* memAlloc)(uintptr_t* outDevicePtr, size_t bytes, int* outStatus);
    bool(CARB_ABI* memFree)(uintptr_t devicePtr, int* outStatus);
    bool(CARB_ABI* memcpyHtoD)(uintptr_t dstDevice, const void* srcHost, size_t bytes, int* outStatus);
    bool(CARB_ABI* memcpyDtoH)(void* dstHost, uintptr_t srcDevice, size_t bytes, int* outStatus);
    // 2D device-to-device copy (implemented via the CUDA driver 2D memcpy API with
    // CU_MEMORYTYPE_DEVICE src/dst). Used by e.g. ovphysx for AoS->SoA wrench conversion.
    // Parameters mirror the driver API: pitches/width are in bytes; height is in rows.
    bool(CARB_ABI* memcpy2DDeviceToDevice)(uintptr_t dstDevice, size_t dstPitch, uintptr_t srcDevice, size_t srcPitch, size_t widthInBytes, size_t height, int* outStatus);
    bool(CARB_ABI* memsetD8)(uintptr_t dstDevice, uint8_t value, size_t count, int* outStatus);
    bool(CARB_ABI* memsetD32)(uintptr_t dstDevice, uint32_t value, size_t count, int* outStatus);

    // -- Events ------------------------------------------------------------
    // Event handles are represented as uintptr_t (CUevent is a pointer type).
    bool(CARB_ABI* eventCreate)(uintptr_t* outEvent, unsigned int flags, int* outStatus);
    bool(CARB_ABI* eventDestroy)(uintptr_t event, int* outStatus);
    bool(CARB_ABI* eventRecord)(uintptr_t event, uintptr_t stream, int* outStatus);
    bool(CARB_ABI* eventSynchronize)(uintptr_t event, int* outStatus);

    // -- Streams -----------------------------------------------------------
    // Stream handles are represented as uintptr_t (CUstream is a pointer type).
    bool(CARB_ABI* streamCreate)(uintptr_t* outStream, unsigned int flags, int* outStatus);
    bool(CARB_ABI* streamDestroy)(uintptr_t stream, int* outStatus);
    bool(CARB_ABI* streamSynchronize)(uintptr_t stream, int* outStatus);
    bool(CARB_ABI* streamWaitEvent)(uintptr_t stream, uintptr_t event, unsigned int flags, int* outStatus);
};

} // namespace physx
} // namespace omni
