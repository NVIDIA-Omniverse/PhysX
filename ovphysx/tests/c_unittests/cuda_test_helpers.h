// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <omni/physx/IOptionalCuda.h>
#include "ovphysxTestHelpers.h"

#include <cstddef>
#include <cstdint>
#include <vector>

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

// True when the calling thread has no CUDA context current. Inside a scope that
// detached the stack, a call that pushes and pops symmetrically must leave it that
// way. A stranded push is otherwise invisible, since restoring the stack simply
// stacks the saved contexts on top of the leaked one and still succeeds.
inline bool noCudaContextCurrent(omni::physx::IOptionalCuda* cuda)
{
    uintptr_t current = 0;
    return cuda && cuda->ctxGetCurrent(&current, nullptr) && current == 0;
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

        // Push only if the desired context is not already current, to keep the
        // CUDA context stack depth unchanged.
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

// Removes every CUDA context from the calling thread for the scope, then rebuilds the
// stack. This is the test substitute for a caller context that is not the simulation's.
// IOptionalCuda has no context creation, so a test cannot make a foreign context.
class ScopedCudaContextDetach
{
public:
    explicit ScopedCudaContextDetach(omni::physx::IOptionalCuda* cuda)
        : mCuda(cuda)
    {
        if (!mCuda)
            return;
        for (;;)
        {
            uintptr_t current = 0;
            if (!mCuda->ctxGetCurrent(&current, nullptr) || current == 0)
                break;
            uintptr_t popped = 0;
            if (!mCuda->ctxPopCurrent(&popped, nullptr))
                break;
            mDetached.push_back(popped);
        }
    }

    ~ScopedCudaContextDetach() { restore(); }

    ScopedCudaContextDetach(const ScopedCudaContextDetach&) = delete;
    ScopedCudaContextDetach& operator=(const ScopedCudaContextDetach&) = delete;

    // Pushes the saved contexts back bottom-up. Returns false if any push failed, which
    // a test should surface: a half-restored stack corrupts later cases.
    bool restore()
    {
        bool ok = true;
        while (!mDetached.empty())
        {
            if (!mCuda->ctxPushCurrent(mDetached.back(), nullptr))
                ok = false;
            mDetached.pop_back();
        }
        return ok;
    }

private:
    omni::physx::IOptionalCuda* mCuda = nullptr;
    std::vector<uintptr_t> mDetached;
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
