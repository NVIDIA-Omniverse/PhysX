// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector_types.h>

namespace omni
{
namespace physx
{

// Opaque handle wrappers used to transport CUDA driver handles without including <cuda.h>.
// These types are trivial / standard-layout so they can be safely embedded in ABI structs
// shared across translation units (e.g. between host .cpp and .cu files).

// Host-only handle to a CUDA stream. Not meaningful on device; used only by host launch code.
struct FabricCudaStreamHandle
{
    std::uintptr_t v = 0;
};

// Device pointer handle. Uses uint64_t to match CUdeviceptr (always 64-bit, regardless of
// host pointer width).
struct FabricCudaDevicePtrHandle
{
    std::uint64_t v = 0;
};

// compile-time checks to ensure we don't get surprises
static_assert(sizeof(void*) == 8, "this code assumes a 64-bit host build");
static_assert(sizeof(FabricCudaStreamHandle) == sizeof(std::uintptr_t), "stream handle must not change ABI");
static_assert(alignof(FabricCudaStreamHandle) == alignof(std::uintptr_t), "stream handle alignment must match");
static_assert(std::is_trivially_copyable_v<FabricCudaStreamHandle>, "stream handle must be trivially copyable");
static_assert(std::is_standard_layout_v<FabricCudaStreamHandle>, "stream handle must be standard layout");
static_assert(sizeof(FabricCudaDevicePtrHandle) == sizeof(std::uint64_t), "device ptr handle must not change ABI");
static_assert(alignof(FabricCudaDevicePtrHandle) == alignof(std::uint64_t), "device ptr handle alignment must match");
static_assert(std::is_trivially_copyable_v<FabricCudaDevicePtrHandle>, "device ptr handle must be trivially copyable");
static_assert(std::is_standard_layout_v<FabricCudaDevicePtrHandle>, "device ptr handle must be standard layout");

// DEPRECATED
// AD: pack the complete info into 1 GPU cache line.
// TODO: use this format for all of the conversion kernels.
// TODO: get rid of all the redundant code.
struct DeformableSurfaceGPUDataDeprecated
{
    float4 transform[4]; // 64B      (actually a pxr::GfMatrix4f)
    float4* srcPoints; // 8B  - 72 (actually a PxVec4)
    float3* dstPoints; // 8B  - 80 (either staging buffer for CPU fabric or device buffer for GPU fabric)
    int nbPoints; // 4B  - 84
    int padding[11]; // 11 * 4B - 128B
};

struct DeformableBodyGPUData
{
    float4 transform[4];    // 64B         (actually a pxr::GfMatrix4f)
    void* srcPoints;        // 8B     - 72 (either PxVec3 or PxVec4)
    float3* dstPoints;      // 8B     - 80 (either staging buffer for CPU fabric or device buffer for GPU fabric)
    int nbPoints;           // 4B     - 84
    int srcPointsElemSize;  // 4B     - 88 (either 3 for vec3 or 4 for vec4)
    int padding[10];        // 4B*10  - 128
};

void convertVec4fToVec3f(float3* out, const float4* in, const size_t numVerts, const float4* toWorld);

void convertVec4fToVec3fBlock(float3** out,
                              const float4** in,
                              const int2* offset,
                              const float* toWorld,
                              const size_t numBlocks,
                              const size_t numThreadsPerBlocks);

// DEPRECATED
void convertVec4ftoVec3fBlockParticleClothDeprecated(float3** out,
                                                     const float4** in,
                                                     const int* sizes,
                                                     const float* toWorld,
                                                     const int** remapTables,
                                                     const size_t numBlocks,
                                                     const size_t numElements);

// DEPRECATED
void convertVec4ftoVec3fBlockDeformableSurfaceDeprecated(DeformableSurfaceGPUDataDeprecated* surfaces,
                                                         int numCloths,
                                                         int maxPoints);

void convertToVec3fBlockDeformableBody(DeformableBodyGPUData* deformableBodies,
                                       int numDeformables,
                                       int maxPoints,
                                       FabricCudaStreamHandle stream = { 0 });

struct RigidBodyGpuData
{
    size_t numRigidBodies;
    FabricCudaDevicePtrHandle rigidBodyTransforms;
    FabricCudaDevicePtrHandle linearVelocities;
    FabricCudaDevicePtrHandle angularVelocities;
    FabricCudaDevicePtrHandle initialScales;
    double** worldMatMapping;
    double** worldPosMapping;
    float** worldOriMapping;
    float** worldSclMapping;
    float** linVelMapping;
    float** angVelMapping;
    bool updateTransforms;
    bool updateVelocities;
};

void copyRigidBodyDataToFabricGpu(const RigidBodyGpuData& rigidBodyData, FabricCudaStreamHandle cudaStream);

} // namespace physx
} // namespace omni
