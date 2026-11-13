// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-2 AC-4 AC-5 AC-6
 */

#include "OvStageOutputCuda.h"
#include "OvStageOutputMath.h"

#include <omni/physx/IOptionalCuda.h>
#include <omni/physx/PhysXRuntime.h>
#include <carb/logging/Log.h>

#include <algorithm>
#include <limits>

namespace ovphysx::utils::detail
{
namespace
{

bool checkCuda(cudaError_t status, const char* operation, std::string& error)
{
    if (status == cudaSuccess)
        return true;
    error = std::string(operation) + ": " + cudaGetErrorString(status);
    return false;
}

} // namespace

OutputCudaContextScope::OutputCudaContextScope() : OutputCudaContextScope(nullptr)
{
}

OutputCudaContextScope::OutputCudaContextScope(const OutputCudaBuffers* buffers)
    : mCuda(omni::physx::runtime::tryGetOptionalCudaInterface())
{
    if (buffers)
    {
        mReady = mPushed = mCuda && buffers->mPrimary && mCuda->ctxPushCurrent(buffers->mPrimary, nullptr);
        return;
    }
    // CPU publication can overwrite a GPU-backed column and invoke CUDA inline.
    // Preserve the caller context even when no CUDA output buffer is needed.
    if (!mCuda || !mCuda->cudaAvailable())
    {
        mReady = true;
        return;
    }
    uintptr_t current = 0;
    if (!mCuda->ctxGetCurrent(&current, nullptr))
        return;
    if (current)
        mReady = mPushed = mCuda->ctxPushCurrent(current, nullptr);
    else
        mReady = mRestoreNull = true;
}

OutputCudaContextScope::~OutputCudaContextScope()
{
    if (!restoreContext())
        CARB_LOG_ERROR("Could not restore the caller CUDA context after world-transform output");
}

bool OutputCudaContextScope::restoreContext() noexcept
{
    if (mRestoreNull)
    {
        uintptr_t current = 0;
        if (!mCuda->ctxGetCurrent(&current, nullptr))
            return false;
        mRestoreNull = false;
        mPushed = current != 0;
    }
    if (!mPushed)
        return true;
    if (!mCuda->ctxPopCurrent(nullptr, nullptr))
        return false;
    mPushed = false;
    return true;
}

bool OutputCudaContextScope::restore(std::string& error)
{
    if (restoreContext())
        return true;
    error = "Could not restore the caller CUDA context after world-transform output";
    return false;
}

namespace
{
class StreamCompletion
{
public:
    explicit StreamCompletion(cudaStream_t stream) : mStream(stream)
    {
    }
    ~StreamCompletion()
    {
        if (mStream)
            cudaStreamSynchronize(mStream);
    }
    void completed()
    {
        mStream = nullptr;
    }

private:
    cudaStream_t mStream;
};

bool waitInput(cudaStream_t stream, ovstage_cuda_sync_t sync, std::string& error)
{
    if (sync.stream &&
        !checkCuda(cudaStreamSynchronize(sync.stream == 1 ? nullptr : reinterpret_cast<cudaStream_t>(sync.stream)),
                   "Wait for pose producer stream", error))
        return false;
    return !sync.wait_event || checkCuda(cudaStreamWaitEvent(stream, reinterpret_cast<cudaEvent_t>(sync.wait_event), 0),
                                         "Wait for pose producer event", error);
}

__global__ void composeMatrices(const float* positions,
                                size_t positionStride,
                                const float* orientations,
                                size_t orientationStride,
                                const float* scales,
                                size_t scaleStride,
                                double* matrices,
                                size_t count,
                                int* invalidPose)
{
    const size_t step = static_cast<size_t>(blockDim.x) * gridDim.x;
    for (size_t row = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x; row < count; row += step)
    {
        if (!composeWorldMatrix(positions + row * positionStride, orientations + row * orientationStride,
                                scales + row * scaleStride, matrices + row * 16))
            atomicExch(invalidPose, 1);
    }
}

const float* columnData(const DLTensor& tensor)
{
    return reinterpret_cast<const float*>(static_cast<const uint8_t*>(tensor.data) + tensor.byte_offset);
}

size_t columnStride(const DLTensor& tensor)
{
    return static_cast<size_t>(tensor.strides ? tensor.strides[0] : 1) * tensor.dtype.lanes;
}

} // namespace

bool OutputCudaBuffers::initialize(int device, size_t count, std::string& error)
{
    if (mPrimary || device < 0 || !count || count > std::numeric_limits<size_t>::max() / (16 * sizeof(double)))
    {
        error = "Invalid CUDA output buffer initialization";
        return false;
    }
    mDevice = device;
    mCount = count;

    // Select the tensor device's primary context while preserving the caller's
    // exact context, including a custom context or no previous context.
    OutputCudaContextScope scope;
    if (!scope.active() || !scope.cuda())
    {
        error = "Could not preserve the caller CUDA context";
        return false;
    }
    if (!checkCuda(cudaSetDevice(device), "Select output CUDA device", error))
        return false;
    if (!scope.cuda()->ctxGetCurrent(&mPrimary, nullptr) || !mPrimary)
    {
        error = "Could not inspect the output CUDA context";
        return false;
    }
    if (!(checkCuda(cudaStreamCreateWithFlags(&mStream, cudaStreamNonBlocking), "Create output CUDA stream", error) &&
          checkCuda(cudaMalloc(reinterpret_cast<void**>(&mMatrices), count * 16 * sizeof(double)),
                    "Allocate CUDA world matrices", error) &&
          checkCuda(cudaMalloc(reinterpret_cast<void**>(&mInvalidPose), sizeof(int)),
                    "Allocate CUDA pose validation flag", error)))
        return false;
    return scope.restore(error);
}

bool OutputCudaBuffers::compose(const ovstage_read_column_t& positionColumn,
                                const ovstage_read_column_t& orientationColumn,
                                const ovstage_read_column_t& scaleColumn,
                                std::string& error)
{
    const ovstage_read_column_t* columns[] = { &positionColumn, &orientationColumn, &scaleColumn };
    for (const ovstage_read_column_t* column : columns)
    {
        const DLDevice device = column->data.tensors[0].device;
        if (!mInvalidPose || device.device_type != kDLCUDA || device.device_id != mDevice)
        {
            error = "Pose and scale columns do not match the CUDA output buffer device";
            return false;
        }
    }
    OutputCudaContextScope scope(this);
    if (!scope.active())
    {
        error = "Could not bind the output device CUDA context";
        return false;
    }
    int invalidPose = 0;
    StreamCompletion completion(mStream);
    for (const ovstage_read_column_t* column : columns)
    {
        if (!waitInput(mStream, column->data.cuda_sync, error))
            return false;
    }
    if (!checkCuda(cudaMemsetAsync(mInvalidPose, 0, sizeof(int), mStream), "Reset CUDA pose validation", error))
        return false;

    const DLTensor& positions = positionColumn.data.tensors[0];
    const DLTensor& orientations = orientationColumn.data.tensors[0];
    const DLTensor& scales = scaleColumn.data.tensors[0];
    const unsigned int blocks = static_cast<unsigned int>(std::min<size_t>((mCount + 255) / 256, 65535));
    composeMatrices<<<blocks, 256, 0, mStream>>>(columnData(positions), columnStride(positions),
                                                 columnData(orientations), columnStride(orientations),
                                                 columnData(scales), columnStride(scales), mMatrices, mCount,
                                                 mInvalidPose);
    if (!checkCuda(cudaGetLastError(), "Launch CUDA world-matrix composition", error) ||
        !checkCuda(cudaMemcpyAsync(&invalidPose, mInvalidPose, sizeof(int), cudaMemcpyDeviceToHost, mStream),
                   "Read CUDA pose validation", error) ||
        !checkCuda(cudaStreamSynchronize(mStream), "Finish CUDA world-matrix composition", error))
        return false;
    completion.completed();
    if (invalidPose)
    {
        error = "Cannot compose a world matrix from a nonfinite pose or scale, or a zero quaternion";
        return false;
    }
    return scope.restore(error);
}

OutputCudaBuffers::~OutputCudaBuffers()
{
    if (!mPrimary)
        return;
    OutputCudaContextScope scope(this);
    if (scope.active())
    {
        if (mStream)
            cudaStreamSynchronize(mStream);
        if (mInvalidPose)
            cudaFree(mInvalidPose);
        if (mMatrices)
            cudaFree(mMatrices);
        if (mStream)
            cudaStreamDestroy(mStream);
    }
}

} // namespace ovphysx::utils::detail
