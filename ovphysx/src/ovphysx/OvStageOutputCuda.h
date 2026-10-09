// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-4 AC-5 AC-6
 */

#pragma once

#include <ovstage/ovstage_api/ovstage_api_types.h>

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace omni::physx
{
struct IOptionalCuda;
}

namespace ovphysx::utils::detail
{
class OutputCudaBuffers;

class OutputCudaContextScope
{
public:
    // Preserve a cold stage read, which can migrate GPU-backed matrices to CPU.
    // Probes the driver when available but does not create a CUDA context.
    OutputCudaContextScope();
    // A null buffer creates a no-op scope and does not query CUDA.
    explicit OutputCudaContextScope(const OutputCudaBuffers* buffers);
    ~OutputCudaContextScope();
    OutputCudaContextScope(const OutputCudaContextScope&) = delete;
    OutputCudaContextScope& operator=(const OutputCudaContextScope&) = delete;

    bool active() const
    {
        return mReady;
    }
    omni::physx::IOptionalCuda* cuda() const
    {
        return mCuda;
    }
    bool restore(std::string& error);

private:
    bool restoreContext() noexcept;
    omni::physx::IOptionalCuda* mCuda = nullptr;
    bool mReady = false;
    bool mPushed = false;
    bool mRestoreNull = false;
};

class OutputCudaBuffers
{
public:
    OutputCudaBuffers() = default;
    ~OutputCudaBuffers();
    OutputCudaBuffers(const OutputCudaBuffers&) = delete;
    OutputCudaBuffers& operator=(const OutputCudaBuffers&) = delete;
    OutputCudaBuffers(OutputCudaBuffers&&) = delete;
    OutputCudaBuffers& operator=(OutputCudaBuffers&&) = delete;

    bool initialize(int device, size_t count, const double* hostScales, std::string& error);

    // The caller validates native float32 vector columns, row coverage and strides.
    // The read session stays alive through compose; destination writes must finish
    // before the buffers are reused or destroyed.
    bool compose(const DLTensor& positions,
                 const DLTensor& orientations,
                 ovstage_cuda_sync_t positionSync,
                 ovstage_cuda_sync_t orientationSync,
                 std::string& error);

    double* matrices() const
    {
        return mMatrices;
    }

private:
    friend class OutputCudaContextScope;
    uintptr_t mPrimary = 0;
    int mDevice = -1;
    size_t mCount = 0;
    double* mScales = nullptr;
    double* mMatrices = nullptr;
    int* mInvalidPose = nullptr;
    cudaStream_t mStream = nullptr;
};

} // namespace ovphysx::utils::detail
