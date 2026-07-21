// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/physx/IOptionalCuda.h>
#include "ovphysxTestHelpers.h"

#include <cstddef>
#include <cstdint>

namespace ovphysx
{
namespace test_cuda
{

// Get IOptionalCuda from the linked PhysX runtime through ovphysx. The runtime
// accessor is internal, so standalone test binaries use this test helper.
inline omni::physx::IOptionalCuda* getCuda()
{
    return static_cast<omni::physx::IOptionalCuda*>(ovphysx_get_optional_cuda_internal());
}

inline bool cudaAvailable()
{
    omni::physx::IOptionalCuda* cuda = getCuda();
    return cuda && cuda->cudaAvailable();
}

// Matches the CUDA driver error code for "not initialized" (used by IOptionalCuda shims).
inline constexpr int kCudaErrorNotInitialized = 3;

class ScopedCudaContextPush
{
public:
    ScopedCudaContextPush(omni::physx::IOptionalCuda* cuda, uintptr_t ctx)
        : mCuda(cuda)
    {
        if (!mCuda || !ctx)
        {
            mOk = false;
            mStatus = kCudaErrorNotInitialized;
            return;
        }

        // Only push if the desired context is not already current (avoid unnecessary
        // CUDA context stack depth changes).
        uintptr_t current = 0;
        if (!mCuda->ctxGetCurrent(&current, nullptr))
        {
            mOk = false;
            mStatus = kCudaErrorNotInitialized;
            return;
        }
        if (current == ctx)
        {
            mOk = true;
            mStatus = 0;
            mPushed = false;
            return;
        }

        int st = 0;
        mOk = mCuda->ctxPushCurrent(ctx, &st);
        mStatus = mOk ? 0 : st;
        mPushed = mOk;
    }

    ~ScopedCudaContextPush()
    {
        if (mCuda && mPushed)
        {
            (void)mCuda->ctxPopCurrent(nullptr, nullptr);
        }
    }

    // Non-copyable/movable: the destructor pops the CUDA context stack, so a
    // copy or move would cause a double-pop.
    ScopedCudaContextPush(const ScopedCudaContextPush&) = delete;
    ScopedCudaContextPush& operator=(const ScopedCudaContextPush&) = delete;
    ScopedCudaContextPush(ScopedCudaContextPush&&) = delete;
    ScopedCudaContextPush& operator=(ScopedCudaContextPush&&) = delete;

    bool ok() const { return mOk; }
    int status() const { return mStatus; }

private:
    omni::physx::IOptionalCuda* mCuda = nullptr;
    bool mOk = false;
    bool mPushed = false;
    int mStatus = 0;
};

// Small fixture-owned helper: holds the CUDA interface pointer and the PhysX CUDA context
// that test-side driver calls should execute under.
struct CudaOps
{
    omni::physx::IOptionalCuda* cuda = nullptr;
    uintptr_t ctx = 0;

    void reset(omni::physx::IOptionalCuda* c, uintptr_t context)
    {
        cuda = c;
        ctx = context;
    }

    bool available() const { return cuda && cuda->cudaAvailable(); }

    bool memAlloc(size_t bytes, uintptr_t* outDevicePtr, int* outStatus) const
    {
        if (outDevicePtr)
            *outDevicePtr = 0;
        if (!cuda)
        {
            if (outStatus)
                *outStatus = kCudaErrorNotInitialized;
            return false;
        }
        ScopedCudaContextPush guard(cuda, ctx);
        if (!guard.ok())
        {
            if (outStatus)
                *outStatus = guard.status();
            return false;
        }
        return cuda->memAlloc(outDevicePtr, bytes, outStatus);
    }

    bool memFree(uintptr_t devicePtr) const
    {
        if (!devicePtr)
            return true;
        if (!cuda)
            return false;
        ScopedCudaContextPush guard(cuda, ctx);
        if (!guard.ok())
            return false;
        return cuda->memFree(devicePtr, nullptr);
    }

    bool memcpyHtoD(uintptr_t dstDevice, const void* srcHost, size_t bytes) const
    {
        if (!cuda)
            return false;
        ScopedCudaContextPush guard(cuda, ctx);
        if (!guard.ok())
            return false;
        return cuda->memcpyHtoD(dstDevice, srcHost, bytes, nullptr);
    }

    bool memcpyDtoH(void* dstHost, uintptr_t srcDevice, size_t bytes) const
    {
        if (!cuda)
            return false;
        ScopedCudaContextPush guard(cuda, ctx);
        if (!guard.ok())
            return false;
        return cuda->memcpyDtoH(dstHost, srcDevice, bytes, nullptr);
    }

    bool memsetD32(uintptr_t dstDevice, uint32_t value, size_t count) const
    {
        if (!cuda)
            return false;
        ScopedCudaContextPush guard(cuda, ctx);
        if (!guard.ok())
            return false;
        return cuda->memsetD32(dstDevice, value, count, nullptr);
    }
};

} // namespace test_cuda
} // namespace ovphysx
