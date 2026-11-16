// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "tensors/gpu/CudaKernels.h"
#include "tensors/gpu/ThrustUtils.h"
#include <cuda_runtime_api.h>
#include <cstdio>
#include <stdexcept>
#include <string>

int main()
{
    unsigned char destination[4] = { 1, 2, 3, 4 };
    const unsigned char source[4] = { 9, 9, 9, 9 };
    void* allocation = destination;
    if (cudaMalloc(&allocation, sizeof(destination)) == cudaSuccess || allocation != nullptr ||
        cudaMemcpy(destination, source, sizeof(destination), cudaMemcpyHostToDevice) == cudaSuccess ||
        cudaMemset(destination, 0, sizeof(destination)) != cudaErrorNoDevice ||
        cudaDeviceSynchronize() == cudaSuccess || destination[0] != 1 || destination[3] != 4)
        return 1;
    bool rejectedClear = false;
    try
    {
        omni::physx::tensors::clearContactBuffer(destination, sizeof(destination));
    }
    catch (const std::runtime_error& error)
    {
        rejectedClear = std::string(error.what()).find("no CUDA kernel: clearContactBuffer") != std::string::npos;
    }
    if (!rejectedClear || destination[0] != 1 || destination[3] != 4)
        return 2;
    omni::physx::tensors::SingleAllocPolicy policy;
    physx::PxU32 count = 123;
    bool rejectedCount = false;
    try
    {
        omni::physx::tensors::compactMaskToIndices(policy, nullptr, destination, 4, count);
    }
    catch (const std::runtime_error&)
    {
        rejectedCount = true;
    }
    if (!rejectedCount || count != 123)
        return 3;
    std::puts("PASS shared-stage CUDA provider: operations reject, kernels throw, caller data unchanged");
    return 0;
}
