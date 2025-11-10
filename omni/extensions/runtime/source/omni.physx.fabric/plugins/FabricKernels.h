// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <vector_types.h>

#include "common/utilities/CudaHelpers.h"
#include <cudamanager/PxCudaTypes.h>

namespace omni
{
namespace physx
{

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
                                       CUstream stream = 0);

struct RigidBodyGpuData
{
    size_t numRigidBodies;
    CUdeviceptr rigidBodyTransforms;
    CUdeviceptr linearVelocities;
    CUdeviceptr angularVelocities;
    CUdeviceptr initialScales;
    double** localMatMapping;
    double** parentWorldMatMapping;
    double** worldPosMapping;
    float** worldOriMapping;
    float** worldSclMapping;
    float** linVelMapping;
    float** angVelMapping;
    bool updateTransforms;
    bool updateVelocities;
};

void copyRigidBodyDataToFabricGpu(const RigidBodyGpuData& rigidBodyData, CUstream cudaStream);

} // namespace physx
} // namespace omni
