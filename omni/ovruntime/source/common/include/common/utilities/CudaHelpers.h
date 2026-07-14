// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

struct CudaDeviceInfo
{
    int deviceId;
    char name[256];
    size_t totalMemoryMB;
    int computeCapabilityMajor;
    int computeCapabilityMinor;
    int multiProcessorCount;
    int maxThreadsPerBlock;
    bool isIntegrated;
};

static std::vector<CudaDeviceInfo> queryAllCudaDevices()
{
    std::vector<CudaDeviceInfo> devices;
    
    // Initialize CUDA Driver API (required before any device queries)
    // Note: cuInit() is idempotent - safe to call multiple times, will return CUDA_SUCCESS if already initialized
    CUresult initResult = cuInit(0);
    if (initResult != CUDA_SUCCESS)
    {
        const char* errStr = nullptr;
        cuGetErrorString(initResult, &errStr);
        CARB_LOG_INFO("CUDA initialization failed: %s (error code %d)", 
                     errStr ? errStr : "unknown error", initResult);
        return devices;
    }
    
    int deviceCount = 0;
    CUresult result = cuDeviceGetCount(&deviceCount);
    
    if (result != CUDA_SUCCESS)
    {
        const char* errStr = nullptr;
        cuGetErrorString(result, &errStr);
        CARB_LOG_INFO("cuDeviceGetCount failed: %s (error code %d)", 
                     errStr ? errStr : "unknown error", result);
        return devices;
    }
    
    if (deviceCount == 0)
    {
        CARB_LOG_INFO("No CUDA devices found");
        return devices;
    }
    
    CARB_LOG_INFO("Found %d CUDA device(s)", deviceCount);
    
    for (int i = 0; i < deviceCount; i++)
    {
        CUdevice device;
        if (cuDeviceGet(&device, i) != CUDA_SUCCESS)
            continue;
        
        CudaDeviceInfo info = {};
        info.deviceId = i;
        
        // Get device name
        cuDeviceGetName(info.name, sizeof(info.name), device);
        
        // Get total memory
        size_t totalMemory = 0;
        cuDeviceTotalMem(&totalMemory, device);
        info.totalMemoryMB = totalMemory / (1024 * 1024);
        
        // Get compute capability
        cuDeviceGetAttribute(&info.computeCapabilityMajor, 
                            CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, device);
        cuDeviceGetAttribute(&info.computeCapabilityMinor, 
                            CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, device);
        
        // Get SM count
        cuDeviceGetAttribute(&info.multiProcessorCount, 
                            CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, device);
        
        // Get max threads per block
        cuDeviceGetAttribute(&info.maxThreadsPerBlock, 
                            CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK, device);
        
        // Check if integrated GPU
        int integrated = 0;
        cuDeviceGetAttribute(&integrated, 
                            CU_DEVICE_ATTRIBUTE_INTEGRATED, device);
        info.isIntegrated = (integrated != 0);
        
        devices.push_back(info);
        
        CARB_LOG_INFO("Device %d: %s", i, info.name);
        CARB_LOG_INFO("    Memory: %zu MB", info.totalMemoryMB);
        CARB_LOG_INFO("    Compute Capability: %d.%d", info.computeCapabilityMajor, info.computeCapabilityMinor);
        CARB_LOG_INFO("    SM Count: %d", info.multiProcessorCount);
        CARB_LOG_INFO("    Integrated: %s", info.isIntegrated ? "Yes" : "No");
    }
    
    return devices;
}

// Select the best device for physics (prefer discrete GPUs with most memory)
static int selectBestPhysicsDevice(bool useFirstAvailable = true)
{
    auto devices = queryAllCudaDevices();
    
    if (devices.empty())
        return -1;

    // Otherwise, find the best device 
    
    // Prefer discrete GPUs
    int bestDevice = -1;
    size_t maxMemory = 0;

    if (useFirstAvailable)
    {
        return devices[0].deviceId;
    }
    
    for (const auto& dev : devices)
    {
        // Skip integrated GPUs if discrete ones are available
        if (dev.isIntegrated)
            continue;
            
        // Pick device with most memory
        if (dev.totalMemoryMB > maxMemory)
        {
            maxMemory = dev.totalMemoryMB;
            bestDevice = dev.deviceId;
        }
    }
    
    // If no discrete GPU found, use the first device
    if (bestDevice == -1 && !devices.empty())
        bestDevice = devices[0].deviceId;
    
    if (bestDevice >= 0)
    {
        CARB_LOG_INFO("Selected CUDA device %d for physics", bestDevice);
    }
    
    return bestDevice;
}

} // namespace cudaHelpers
} // namespace physx
} // namespace omni
