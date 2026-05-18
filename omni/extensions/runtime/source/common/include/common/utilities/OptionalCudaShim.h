// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Optional CUDA driver shim acquisition helper.
//
// Rationale:
// - Consumers (fabric, tensors) want to call cu* driver API functions without linking libcuda.
// - The driver API is accessed via omni::physx::IOptionalCuda (implemented by foundation).
// - Some code paths may call into GPU helpers before foundation is fully loaded. Also,
//   caching a nullptr interface pointer is a footgun.
//
// This helper:
// - Tries to acquire IOptionalCuda each time (no "cache nullptr forever").
// - Provides a stub interface when unavailable so call sites do not crash.
//
// NOTE: This header does NOT include <cuda.h>.
// It uses the canonical CUDA driver `CUresult` numeric values for status codes:
// - CUDA_SUCCESS == 0
// - CUDA_ERROR_NOT_INITIALIZED == 3

#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>

#include <carb/Framework.h>
#include <omni/physx/IOptionalCuda.h>

namespace omni
{
namespace physx
{
namespace optionalCuda
{

static constexpr int kCudaSuccess = 0;
static constexpr int kCudaErrorNotInitialized = 3;

static inline bool CARB_ABI stub_cudaAvailable()
{
    return false;
}

static inline bool CARB_ABI stub_ctxGetCurrent(uintptr_t* outCtx, int* outStatus)
{
    if (outCtx)
        *outCtx = 0;
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_ctxPushCurrent(uintptr_t, int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_ctxPopCurrent(uintptr_t* outCtx, int* outStatus)
{
    if (outCtx)
        *outCtx = 0;
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_ctxGetDevice(int* outDevice, int* outStatus)
{
    if (outDevice)
        *outDevice = -1;
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_ctxSynchronize(int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_deviceGetCount(int* outCount, int* outStatus)
{
    if (outCount)
        *outCount = 0;
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_deviceGet(int* outDevice, int, int* outStatus)
{
    if (outDevice)
        *outDevice = -1;
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_memAlloc(uintptr_t* outDevicePtr, size_t, int* outStatus)
{
    if (outDevicePtr)
        *outDevicePtr = 0;
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_memFree(uintptr_t devicePtr, int* outStatus)
{
    if (!devicePtr)
    {
        if (outStatus)
            *outStatus = kCudaSuccess;
        return true;
    }
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_memcpyHtoD(uintptr_t, const void*, size_t, int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_memcpyDtoH(void*, uintptr_t, size_t, int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_memcpy2DDeviceToDevice(uintptr_t, size_t, uintptr_t, size_t, size_t widthInBytes, size_t height, int* outStatus)
{
    // Match CUDA semantics: a 0-byte copy is a successful no-op.
    if (widthInBytes == 0 || height == 0)
    {
        if (outStatus)
            *outStatus = kCudaSuccess;
        return true;
    }
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_memsetD8(uintptr_t, uint8_t, size_t, int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_memsetD32(uintptr_t, uint32_t, size_t, int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_eventCreate(uintptr_t* outEvent, unsigned int, int* outStatus)
{
    if (outEvent)
        *outEvent = 0;
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_eventDestroy(uintptr_t event, int* outStatus)
{
    if (!event)
    {
        if (outStatus)
            *outStatus = kCudaSuccess;
        return true;
    }
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_eventRecord(uintptr_t, uintptr_t, int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_eventSynchronize(uintptr_t, int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_streamCreate(uintptr_t* outStream, unsigned int, int* outStatus)
{
    if (outStream)
        *outStream = 0;
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_streamDestroy(uintptr_t stream, int* outStatus)
{
    if (!stream)
    {
        if (outStatus)
            *outStatus = kCudaSuccess;
        return true;
    }
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_streamSynchronize(uintptr_t, int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline bool CARB_ABI stub_streamWaitEvent(uintptr_t, uintptr_t, unsigned int, int* outStatus)
{
    if (outStatus)
        *outStatus = kCudaErrorNotInitialized;
    return false;
}

static inline omni::physx::IOptionalCuda& getStubInterface()
{
    static omni::physx::IOptionalCuda iface = [] {
        omni::physx::IOptionalCuda i{};
        i.cudaAvailable = stub_cudaAvailable;
        i.ctxGetCurrent = stub_ctxGetCurrent;
        i.ctxPushCurrent = stub_ctxPushCurrent;
        i.ctxPopCurrent = stub_ctxPopCurrent;
        i.ctxGetDevice = stub_ctxGetDevice;
        i.ctxSynchronize = stub_ctxSynchronize;
        i.deviceGetCount = stub_deviceGetCount;
        i.deviceGet = stub_deviceGet;
        i.memAlloc = stub_memAlloc;
        i.memFree = stub_memFree;
        i.memcpyHtoD = stub_memcpyHtoD;
        i.memcpyDtoH = stub_memcpyDtoH;
        i.memsetD8 = stub_memsetD8;
        i.memsetD32 = stub_memsetD32;
        i.eventCreate = stub_eventCreate;
        i.eventDestroy = stub_eventDestroy;
        i.eventRecord = stub_eventRecord;
        i.eventSynchronize = stub_eventSynchronize;
        i.streamCreate = stub_streamCreate;
        i.streamDestroy = stub_streamDestroy;
        i.streamSynchronize = stub_streamSynchronize;
        i.streamWaitEvent = stub_streamWaitEvent;
        i.memcpy2DDeviceToDevice = stub_memcpy2DDeviceToDevice;
        return i;
    }();
    return iface;
}

static inline omni::physx::IOptionalCuda* get()
{
    // Cache only a successfully acquired interface pointer.
    // Do NOT cache "unavailable": foundation/plugins can be loaded later and we
    // want callers to start using the real interface once it becomes available.
    static std::atomic<omni::physx::IOptionalCuda*> s_cached{ nullptr };
    if (auto* cached = s_cached.load(std::memory_order_acquire))
        return cached;

    auto* framework = carb::getFramework();
    if (framework)
    {
        if (auto* s = framework->tryAcquireInterface<omni::physx::IOptionalCuda>())
        {
            s_cached.store(s, std::memory_order_release);
            return s;
        }
    }
    return &getStubInterface();
}

} // namespace optionalCuda
} // namespace physx
} // namespace omni
