// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "FabricKernels.h"

#include <foundation/PxMat44.h>
#include <foundation/PxTransform.h>

#include <usdrt/gf/matrix.h>

#include <cuda.h>
#include <stdio.h>
#include <cuda_runtime_api.h>

namespace
{

__device__ inline float3 project(const float4& v)
{
    float inv = (v.w != 0.0f) ? 1.0f / v.w : 1.0f;
    return float3{ inv * v.x, inv * v.y, inv * v.z };
}

/// Transforms the row vector \e vec by the matrix, returning the result.
/// This treats the vector as a 4-component vector whose fourth component
/// is 1. This is an overloaded method; it differs from the other version
/// in that it returns a different value type.
__device__ inline float3 transform(const float4* mtx, const float3& vec)
{
    return (project(float4{ vec.x * mtx[0].x + vec.y * mtx[1].x + vec.z * mtx[2].x + mtx[3].x,
                            vec.x * mtx[0].y + vec.y * mtx[1].y + vec.z * mtx[2].y + mtx[3].y,
                            vec.x * mtx[0].z + vec.y * mtx[1].z + vec.z * mtx[2].z + mtx[3].z,
                            vec.x * mtx[0].w + vec.y * mtx[1].w + vec.z * mtx[2].w + mtx[3].w }));
}
} // namespace

namespace omni
{
namespace physx
{

__global__ static void convertVec4fToVec3fKernel(float3* out, const float4* in, const size_t numVerts, const float4* toWorld)
{
    const int globalThreadInd = blockIdx.x * blockDim.x + threadIdx.x;
    for (int i = globalThreadInd; i < numVerts; i += blockDim.x * gridDim.x)
    {
        float3 v = transform(toWorld, float3{ in[i].x, in[i].y, in[i].z });
        out[i].x = v.x;
        out[i].y = v.y;
        out[i].z = v.z;
    }
}

__global__ static void convertVec4fToVec3fBlockKernel(float3** out,
                                                      const float4** in,
                                                      const int2* offset,
                                                      const float* toWorld)
{

    const int idx = threadIdx.x + blockIdx.x * blockDim.x;
    float3* _out = out[blockIdx.y];
    const float4* _in = in[blockIdx.y];
    const int2 _offset = offset[blockIdx.y];
    const float4* _toWorld = (float4*)(&toWorld[blockIdx.y * 16]);

    if (idx >= _offset.x)
        return;

    float4 src = _in[idx + _offset.y];

    float3 v = transform(_toWorld, float3{ src.x, src.y, src.z });
    _out[idx].x = v.x;
    _out[idx].y = v.y;
    _out[idx].z = v.z;
}

// DEPRECATED
__global__ static void convertVec4fToVec3fBlockParticleClothKernel(
    float3** out, const float4** in, const int* sizes, const float* toWorld, const int** remapTables)
{
    const int idx = threadIdx.x + blockIdx.x * blockDim.x;
    float3* _out = out[blockIdx.y];
    const float4* _in = in[blockIdx.y];
    const int _numElements = sizes[blockIdx.y];
    const float4* _toWorld = (float4*)(&toWorld[blockIdx.y * 16]);

    if (idx >= _numElements)
        return;

    const int* _remapTable = remapTables[blockIdx.y];

    int index = _remapTable ? _remapTable[idx] : idx;
    float4 src = _in[index];

    float3 v = transform(_toWorld, float3{ src.x, src.y, src.z });
    _out[idx].x = v.x;
    _out[idx].y = v.y;
    _out[idx].z = v.z;
}

// DEPRECATED
__global__ static void convertVec4fToVec3fBlockDeformableSurfaceKernel(DeformableSurfaceGPUDataDeprecated* surfaces)
{
    const int idx = threadIdx.x + blockIdx.x * blockDim.x;

    DeformableSurfaceGPUDataDeprecated& data = surfaces[blockIdx.y];
    float3* out = data.dstPoints;
    const float4* in = data.srcPoints;
    const int numElements = data.nbPoints;

    if (idx >= numElements)
        return;

    const float4* toWorld = (float4*)(&data.transform);

    float4 src = in[idx];
    float3 v = transform(toWorld, float3{ src.x, src.y, src.z });
    out[idx] = v;
}

__global__ static void fabricDeformableBodyKernel(DeformableBodyGPUData* deformableBodies)
{
    const int idx = threadIdx.x + blockIdx.x * blockDim.x;

    DeformableBodyGPUData& data = deformableBodies[blockIdx.y];
    float3* out = data.dstPoints;
    const int numElements = data.nbPoints;

    if (idx >= numElements)
        return;

    const float4* toWorld = (float4*)(&data.transform);

    int srcIndex = idx;

    float3 v;
    const int srcPointsElemSize = data.srcPointsElemSize;
    if (srcPointsElemSize == 3)
    {
        const float3* in = (const float3*)data.srcPoints;
        float3 src = in[srcIndex];
        v = transform(toWorld, float3{ src.x, src.y, src.z });
    }
    else if (srcPointsElemSize == 4)
    {
        const float4* in = (const float4*)data.srcPoints;
        float4 src = in[srcIndex];
        v = transform(toWorld, float3{ src.x, src.y, src.z });
    }
    out[idx] = v;
}

//////////////////////
// Host launch code //
//////////////////////

void convertVec4fToVec3f(float3* out, const float4* in, const size_t numVerts, const float4* toWorld)
{
    int numSMs;
    cudaDeviceGetAttribute(&numSMs, cudaDevAttrMultiProcessorCount, 0);
    convertVec4fToVec3fKernel<<<32 * numSMs, 256>>>(out, in, numVerts, toWorld);
}

void convertVec4fToVec3fBlock(float3** out,
                              const float4** in,
                              const int2* offset,
                              const float* toWorld,
                              const size_t numBlocks,
                              const size_t numElements)
{
    const size_t numThreadsPerBlocks = 256;
    const size_t numBlocksX = (numElements + numThreadsPerBlocks - 1) / numThreadsPerBlocks;
    dim3 blockSize(numThreadsPerBlocks, 1, 1);
    dim3 gridSize(numBlocksX, numBlocks, 1);
    convertVec4fToVec3fBlockKernel<<<gridSize, blockSize>>>(out, in, offset, toWorld);
    // cudaError_t resErr = cudaDeviceSynchronize();
}

// DEPRECATED
void convertVec4ftoVec3fBlockParticleClothDeprecated(float3** out,
                                                     const float4** in,
                                                     const int* sizes,
                                                     const float* toWorld,
                                                     const int** remapTables,
                                                     const size_t numBlocks,
                                                     const size_t numElements)
{
    const size_t numThreadsPerBlocks = 256;
    const size_t numBlocksX = (numElements + numThreadsPerBlocks - 1) / numThreadsPerBlocks;
    dim3 blockSize(numThreadsPerBlocks, 1, 1);
    dim3 gridSize(numBlocksX, numBlocks, 1);
    convertVec4fToVec3fBlockParticleClothKernel<<<gridSize, blockSize>>>(out, in, sizes, toWorld, remapTables);
}

// DEPRECATED
void convertVec4ftoVec3fBlockDeformableSurfaceDeprecated(DeformableSurfaceGPUDataDeprecated* surfaces, int numCloths, int maxPoints)
{
    const size_t numThreadsPerBlocks = 256;
    const size_t numBlocksX = (maxPoints + numThreadsPerBlocks - 1) / numThreadsPerBlocks;
    dim3 blockSize(numThreadsPerBlocks, 1, 1);
    dim3 gridSize(numBlocksX, numCloths, 1);
    convertVec4fToVec3fBlockDeformableSurfaceKernel<<<gridSize, blockSize>>>(surfaces);
}

void convertToVec3fBlockDeformableBody(DeformableBodyGPUData* deformableBodies, int numDeformables, int maxPoints, CUstream stream)
{
    const size_t numThreadsPerBlocks = 256;
    const size_t numBlocksX = (maxPoints + numThreadsPerBlocks - 1) / numThreadsPerBlocks;
    dim3 blockSize(numThreadsPerBlocks, 1, 1);
    dim3 gridSize(numBlocksX, numDeformables, 1);

    fabricDeformableBodyKernel<<<gridSize, blockSize, 0, stream>>>(deformableBodies);
}

// debug code, temporary
__device__ void printMatrix(const char* text, double* mat)
{
    printf("fabric matrix %s: [(%f %f %f %f), (%f %f %f %f), (%f %f %f %f), (%f %f %f %f)]\n", text, mat[0], mat[1],
           mat[2], mat[3], mat[4], mat[5], mat[6], mat[7], mat[8], mat[9], mat[10], mat[11], mat[12], mat[13], mat[14],
           mat[15]);
}

__device__ inline ::usdrt::GfVec3d toVec3d(const ::physx::PxVec3& in)
{
    return ::usdrt::GfVec3d{in.x, in.y, in.z};
}

__device__ inline ::usdrt::GfVec3f toVec3f(const ::physx::PxVec3& in)
{
    return ::usdrt::GfVec3f{in.x, in.y, in.z};
}

__device__ inline ::usdrt::GfVec4f toVec4(const ::physx::PxQuat& in)
{
    return ::usdrt::GfVec4f{in.x, in.y, in.z, in.w};
}

__device__ inline ::usdrt::GfQuatd toQuatd(const ::usdrt::GfVec4f& in)
{
    return ::usdrt::GfQuatd{in[3], in[0], in[1], in[2]};
}

__device__ inline ::usdrt::GfMatrix4d composeMatrix(const ::usdrt::GfVec3d& position, const ::usdrt::GfVec4f& rotate, const ::usdrt::GfVec3f& scale)
{
    const ::usdrt::GfMatrix3d rotation(toQuatd(rotate));
    return ::usdrt::GfMatrix4d(rotation[0][0] * scale[0], rotation[0][1] * scale[0], rotation[0][2] * scale[0], 0,
                               rotation[1][0] * scale[1], rotation[1][1] * scale[1], rotation[1][2] * scale[1], 0,
                               rotation[2][0] * scale[2], rotation[2][1] * scale[2], rotation[2][2] * scale[2], 0,
                               position[0], position[1], position[2], 1);
}

__global__ void copyRigidBodyDataKernel(RigidBodyGpuData rb) {
    const int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx < rb.numRigidBodies)
    {
        if (rb.updateTransforms)
        {
            ::physx::PxTransform rdTransform = reinterpret_cast<::physx::PxTransform*>(rb.rigidBodyTransforms)[idx];
            ::usdrt::GfVec3d position = toVec3d(rdTransform.p);
            ::usdrt::GfVec4f rotate = toVec4(rdTransform.q);
            ::usdrt::GfVec3f scale = rb.initialScales ? reinterpret_cast<::usdrt::GfVec3f*>(rb.initialScales)[idx] : ::usdrt::GfVec3f(1, 1, 1);
            double* fabricLocalMatPtr = rb.localMatMapping ? rb.localMatMapping[idx] : nullptr;
            double* fabricParentWorldMatPtr = rb.parentWorldMatMapping ? rb.parentWorldMatMapping[idx] : nullptr;
            if (fabricLocalMatPtr)
            {
                ::usdrt::GfMatrix4d localMat = composeMatrix(position, rotate, scale);

                if (fabricParentWorldMatPtr)
                {
                    ::usdrt::GfMatrix4d parentWorldMat = *reinterpret_cast<::usdrt::GfMatrix4d*>(fabricParentWorldMatPtr);
                    localMat = localMat * parentWorldMat.GetInverse();
                }

                *reinterpret_cast<::usdrt::GfMatrix4d*>(fabricLocalMatPtr) = localMat;
            }
            double* worldPosPtr = rb.worldPosMapping ? rb.worldPosMapping[idx] : nullptr;
            if (worldPosPtr)
            {
                *reinterpret_cast<::usdrt::GfVec3d*>(worldPosPtr) = position;
            }
            float* worldOriPtr = rb.worldOriMapping ? rb.worldOriMapping[idx] : nullptr;
            if (worldOriPtr)
            {
                *reinterpret_cast<::usdrt::GfVec4f*>(worldOriPtr) = rotate;
            }
            float* worldSclPtr = rb.worldSclMapping ? rb.worldSclMapping[idx] : nullptr;
            if (worldSclPtr)
            {
                *reinterpret_cast<::usdrt::GfVec3f*>(worldSclPtr) = scale;
            }
        }
        if (rb.updateVelocities)
        {
            float* linVelPtr = rb.linVelMapping ? rb.linVelMapping[idx] : nullptr;
            if (linVelPtr)
            {
                ::usdrt::GfVec3f velocity = rb.linearVelocities ? toVec3f(reinterpret_cast<::physx::PxVec3*>(rb.linearVelocities)[idx]) : ::usdrt::GfVec3f(0, 0, 0);
                *reinterpret_cast<::usdrt::GfVec3f*>(linVelPtr) = velocity;
            }
            float* angVelPtr = rb.angVelMapping ? rb.angVelMapping[idx] : nullptr;
            if (angVelPtr)
            {
                ::usdrt::GfVec3f velocity = rb.angularVelocities ? toVec3f(reinterpret_cast<::physx::PxVec3*>(rb.angularVelocities)[idx]) : ::usdrt::GfVec3f(0, 0, 0);
                *reinterpret_cast<::usdrt::GfVec3f*>(angVelPtr) = velocity;
            }
        }
    }
}

void copyRigidBodyDataToFabricGpu(const RigidBodyGpuData& rigidBodyData, CUstream cudaStream)
{    
    const size_t numThreadsPerBlocks = 256;
    dim3 blockDim(numThreadsPerBlocks);
    dim3 gridDim((rigidBodyData.numRigidBodies + blockDim.x - 1) / blockDim.x);
    copyRigidBodyDataKernel<<<gridDim, blockDim, 0, cudaStream>>>(rigidBodyData);
}

} // namespace physx
} // namespace omni
