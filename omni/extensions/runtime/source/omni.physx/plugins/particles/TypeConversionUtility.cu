// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "TypeConversionUtility.cuh"
#include <cuda.h>
#include <stdio.h>
#include <cuda_runtime_api.h>

namespace omni
{
    namespace physx
    {
#if 0
        __global__ static void convertVec4fToVec3fKernel(float3* out, const float4* in, const size_t numVerts)
        {
            const int globalThreadInd = blockIdx.x * blockDim.x + threadIdx.x;
            for (int i = globalThreadInd; i < numVerts; i += blockDim.x * gridDim.x)
            {
                out[i].x = in[i].x;
                out[i].y = in[i].y;
                out[i].z = in[i].z;
            }
        }

        void convertVec4fToVec3f(float3* out, const float4* in, const size_t numVerts)
        {
            int numSMs;
            cudaDeviceGetAttribute(&numSMs, cudaDevAttrMultiProcessorCount, 0);
            convertVec4fToVec3fKernel << <32 * numSMs, 256 >> > (out, in, numVerts);
        }
#endif

        template<typename outType, typename inType>
        __global__ static void convertBlockKernel(outType** out, const inType** in, const int* offset)
        {

            const int idx = threadIdx.x + blockIdx.x * blockDim.x;
            outType* _out = out[blockIdx.y];
            const inType* _in = in[blockIdx.y];
            const int _offset = offset[blockIdx.y];

            if (idx >= _offset)
                return;

            inType src = _in[idx];
            
            _out[idx].x = src.x;
            _out[idx].y = src.y;
            _out[idx].z = src.z;
        }

        void convertVec4fToVec3fBlock(float3** out, const float4** in, const int* offset, const size_t numBlocks, const size_t numElements)
        {
            const size_t numThreadsPerBlocks = 256;
            const size_t numBlocksX = (numElements + numThreadsPerBlocks - 1) / numThreadsPerBlocks;
            dim3 blockSize(numThreadsPerBlocks, 1, 1);
            dim3 gridSize(numBlocksX, numBlocks, 1);
            convertBlockKernel<float3, float4> << < gridSize, blockSize >> > (out, in, offset);
        }

        void convertVec3fToVec4fBlock(float4** out, const float3** in, const int* offset, const size_t numBlocks, const size_t numElements)
        {
            const size_t numThreadsPerBlocks = 256;
            const size_t numBlocksX = (numElements + numThreadsPerBlocks - 1) / numThreadsPerBlocks;
            dim3 blockSize(numThreadsPerBlocks, 1, 1);
            dim3 gridSize(numBlocksX, numBlocks, 1);
            convertBlockKernel<float4, float3> << < gridSize, blockSize >> > (out, in, offset);
        }
    }
}
