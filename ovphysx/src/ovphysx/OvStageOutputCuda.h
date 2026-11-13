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
    // Preserve the caller context around work that selects a device.
    // Probes the driver when available but does not create a CUDA context.
    OutputCudaContextScope();
    // Select the output buffer context, or preserve the caller context for CPU output.
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

    bool initialize(int device, size_t count, std::string& error);

    // Each column is one native float32 vector column: position (3 lanes),
    // orientation (4) and world scale (3). The caller validates row coverage and
    // strides. The read session stays alive through compose; destination writes
    // must finish before the buffers are reused or destroyed.
    bool compose(const ovstage_read_column_t& positions,
                 const ovstage_read_column_t& orientations,
                 const ovstage_read_column_t& scales,
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
    double* mMatrices = nullptr;
    int* mInvalidPose = nullptr;
    cudaStream_t mStream = nullptr;
};

} // namespace ovphysx::utils::detail
