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

OutputCudaContextScope::OutputCudaContextScope() : mCuda(omni::physx::runtime::tryGetOptionalCudaInterface())
{
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

OutputCudaContextScope::OutputCudaContextScope(const OutputCudaBuffers* buffers)
{
    if (!buffers)
    {
        mReady = true;
        return;
    }
    mCuda = omni::physx::runtime::tryGetOptionalCudaInterface();
    mReady = mPushed = mCuda && buffers->mPrimary && mCuda->ctxPushCurrent(buffers->mPrimary, nullptr);
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
                                const double* scales,
                                double* matrices,
                                size_t count,
                                int* invalidPose)
{
    const size_t step = static_cast<size_t>(blockDim.x) * gridDim.x;
    for (size_t row = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x; row < count; row += step)
    {
        if (!composeWorldMatrix(positions + row * positionStride, orientations + row * orientationStride,
                                scales + row * 3, matrices + row * 16))
            atomicExch(invalidPose, 1);
    }
}

} // namespace

bool OutputCudaBuffers::initialize(int device, size_t count, const double* hostScales, std::string& error)
{
    if (mPrimary || device < 0 || !count || !hostScales ||
        count > std::numeric_limits<size_t>::max() / (16 * sizeof(double)))
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
    if (!checkCuda(cudaStreamCreateWithFlags(&mStream, cudaStreamNonBlocking), "Create output CUDA stream", error))
        return false;
    StreamCompletion completion(mStream);
    if (!(checkCuda(cudaMalloc(reinterpret_cast<void**>(&mScales), count * 3 * sizeof(double)),
                    "Allocate CUDA world scales", error) &&
          checkCuda(cudaMalloc(reinterpret_cast<void**>(&mMatrices), count * 16 * sizeof(double)),
                    "Allocate CUDA world matrices", error) &&
          checkCuda(cudaMalloc(reinterpret_cast<void**>(&mInvalidPose), sizeof(int)),
                    "Allocate CUDA pose validation flag", error) &&
          checkCuda(cudaMemcpyAsync(mScales, hostScales, count * 3 * sizeof(double), cudaMemcpyHostToDevice, mStream),
                    "Copy authored world scales to CUDA", error) &&
          checkCuda(cudaStreamSynchronize(mStream), "Finish CUDA world-scale copy", error)))
        return false;
    completion.completed();
    return scope.restore(error);
}

bool OutputCudaBuffers::compose(const DLTensor& positions,
                                const DLTensor& orientations,
                                ovstage_cuda_sync_t positionSync,
                                ovstage_cuda_sync_t orientationSync,
                                std::string& error)
{
    if (!mInvalidPose || positions.device.device_type != kDLCUDA || orientations.device.device_type != kDLCUDA ||
        positions.device.device_id != mDevice || orientations.device.device_id != mDevice)
    {
        error = "Pose columns do not match the CUDA output buffer device";
        return false;
    }
    OutputCudaContextScope scope(this);
    if (!scope.active())
    {
        error = "Could not bind the output device CUDA context";
        return false;
    }
    int invalidPose = 0;
    StreamCompletion completion(mStream);
    if (!waitInput(mStream, positionSync, error) || !waitInput(mStream, orientationSync, error) ||
        !checkCuda(cudaMemsetAsync(mInvalidPose, 0, sizeof(int), mStream), "Reset CUDA pose validation", error))
        return false;

    const float* positionData =
        reinterpret_cast<const float*>(static_cast<const uint8_t*>(positions.data) + positions.byte_offset);
    const float* orientationData =
        reinterpret_cast<const float*>(static_cast<const uint8_t*>(orientations.data) + orientations.byte_offset);
    const size_t positionStride = static_cast<size_t>(positions.strides ? positions.strides[0] : 1) * 3;
    const size_t orientationStride = static_cast<size_t>(orientations.strides ? orientations.strides[0] : 1) * 4;
    const unsigned int blocks = static_cast<unsigned int>(std::min<size_t>((mCount + 255) / 256, 65535));
    composeMatrices<<<blocks, 256, 0, mStream>>>(
        positionData, positionStride, orientationData, orientationStride, mScales, mMatrices, mCount, mInvalidPose);
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
        if (mScales)
            cudaFree(mScales);
        if (mStream)
            cudaStreamDestroy(mStream);
    }
}

} // namespace ovphysx::utils::detail
