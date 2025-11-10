// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/logging/Log.h>

#include <driver_types.h>
#include <cuda.h>
#include <cuda_runtime_api.h>

namespace omni
{
namespace physx
{
namespace cudaHelpers
{

inline bool checkCuda(cudaError_t code, const char* file, int line)
{
    if (code != cudaSuccess)
    {
        CARB_LOG_ERROR("CUDA error: %s: %s: %d", cudaGetErrorString(code), file, line);
        return false;
    }
    return true;
}

inline bool checkCu(CUresult code, const char* file, int line)
{
    if (code != CUDA_SUCCESS)
    {
        const char* err = nullptr;
        cuGetErrorString(code, &err);
        if (err)
        {
            CARB_LOG_ERROR("CUDA error: %s: %s: %d", err, file, line);
        }
        else
        {
            CARB_LOG_ERROR("CUDA unknown error: %s: %d", file, line);
        }
        return false;
    }
    return true;
}

} // namespace cudaHelpers
} // namespace physx
} // namespace omni
