// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "cuda.h"
#include "PxgAlgorithmsData.h"
#include "stdio.h"

using namespace physx;

extern "C" __host__ void initAlgorithmsKernels0() {}

struct int4x4
{
	int4 data[4];
};

PX_FORCE_INLINE PX_CUDA_CALLABLE int4x4 make_int16(const int4& a, const int4& b, const int4& c, const int4& d)
{
	int4x4 result;
	result.data[0] = a;
	result.data[1] = b;
	result.data[2] = c;
	result.data[3] = d;
	return result;
}

PX_FORCE_INLINE PX_CUDA_CALLABLE int4 operator+(const int4& lhs, const int4& rhs) { return make_int4(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w); }

PX_FORCE_INLINE PX_CUDA_CALLABLE int4x4 operator+(const int4x4& lhs, const int4x4& rhs) { return make_int16(lhs.data[0] + rhs.data[0], lhs.data[1] + rhs.data[1], lhs.data[2] + rhs.data[2], lhs.data[3] + rhs.data[3]); }

PX_FORCE_INLINE __device__ int4 shfl_up_sync(PxU32 mask, int4 var, PxU32 delta, int width)
{
	return make_int4(__shfl_up_sync(mask, var.x, delta, width), __shfl_up_sync(mask, var.y, delta, width), __shfl_up_sync(mask, var.z, delta, width), __shfl_up_sync(mask, var.w, delta, width));
}

PX_FORCE_INLINE __device__ int4x4 shfl_up_sync(PxU32 mask, int4x4 var, PxU32 delta, int width)
{
	return make_int16(shfl_up_sync(mask, var.data[0], delta, width), shfl_up_sync(mask, var.data[1], delta, width), shfl_up_sync(mask, var.data[2], delta, width), shfl_up_sync(mask, var.data[3], delta, width));
}

PX_FORCE_INLINE __device__ PxU32 shfl_up_sync(PxU32 mask, PxU32 var, PxU32 delta, int width)
{
	return __shfl_up_sync(mask, var, delta, width);
}

PX_FORCE_INLINE __device__ PxI32 shfl_up_sync(PxU32 mask, PxI32 var, PxU32 delta, int width)
{
	return __shfl_up_sync(mask, var, delta, width);
}

PX_FORCE_INLINE __device__ PxU64 shfl_up_sync(PxU32 mask, PxU64 var, PxU32 delta, int width)
{
	return __shfl_up_sync(mask, var, delta, width);
}

template<typename T>
PX_FORCE_INLINE __device__ T zero()
{
	return T();
}

template<>
PX_FORCE_INLINE __device__ PxU32 zero<PxU32>()
{
	return 0;
}

template<>
PX_FORCE_INLINE __device__ int4 zero<int4>()
{
	return make_int4(0, 0, 0, 0);
}

template<>
PX_FORCE_INLINE __device__ int4x4 zero<int4x4>()
{
	return make_int16(make_int4(0, 0, 0, 0), make_int4(0, 0, 0, 0), make_int4(0, 0, 0, 0), make_int4(0, 0, 0, 0));
}

template<typename T>
__device__ void warpScan(T& value, const int lane_id)
{
#pragma unroll
	for (int i = 1; i <= 32; i *= 2)
	{
		unsigned int mask = 0xffffffff;
		T n = shfl_up_sync(mask, value, i, 32);

		if (lane_id >= i)
			value = value + n;
	}
}

template<typename T>
__device__ T scanPerBlock(
	T				value,
	const PxU32		id,
	T*				sum)
{
	extern __shared__ PxU32 sumsMemory[];
	T* sums = reinterpret_cast<T*>(sumsMemory);
	int lane_id = id % warpSize;
	// determine a warp_id within a block
	int warp_id = threadIdx.x / warpSize;

	// Now accumulate in log steps up the chain
	// compute sums, with another thread's value who is
	// distance delta away (i).  Note
	// those threads where the thread 'i' away would have
	// been out of bounds of the warp are unaffected.  This
	// creates the scan sum.

	warpScan(value, lane_id);

	// value now holds the scan value for the individual thread
	// next sum the largest values for each warp

	__syncthreads(); //Required before accessing shared memory because this function can be called inside loops

	// write the sum of the warp to smem
	if (threadIdx.x % warpSize == warpSize - 1)
	{
		sums[warp_id] = value;
	}

	__syncthreads();

	//
	// scan sum the warp sums
	// the same shfl scan operation, but performed on warp sums
	//
	if (warp_id == 0 && lane_id < (blockDim.x / warpSize))
	{
		T warp_sum = sums[lane_id];

		int mask = (1 << (blockDim.x / warpSize)) - 1;
		for (int i = 1; i <= (blockDim.x / warpSize); i *= 2)
		{
			T n = shfl_up_sync(mask, warp_sum, i, (blockDim.x / warpSize));

			if (lane_id >= i)
				warp_sum = warp_sum + n;
		}

		sums[lane_id] = warp_sum;
	}

	__syncthreads();

	// perform a uniform add across warps in the block
	// read neighbouring warp's sum and add it to threads value
	T blockSum = zero<T>();

	if (warp_id > 0)
	{
		blockSum = sums[warp_id - 1];
	}

	value = value + blockSum;

	// last thread has sum, write write out the block's sum
	if (sum != NULL && threadIdx.x == blockDim.x - 1)
		*sum = value;

	return value;
}

template<typename T>
__device__ void scanPerBlockKernelShared(
	int				id,
	const T*		data,
	T*				result,
	T*				partialSums,
	const PxU32		length,
	const PxU32		exclusiveScan,
	T*				totalSum)
{
	T value = id < length ? data[id] : zero<T>();
	value = scanPerBlock(value, id, &partialSums[blockIdx.x]);
	if (totalSum && id == length - 1)
		*totalSum = value;

	// Now write out our result
	if (id < length && result)
	{
		if (exclusiveScan == 0)
			result[id] = value;
		else
		{
			if (threadIdx.x + 1 < blockDim.x && id + 1 < length)
				result[id + 1] = value;
			if (threadIdx.x == 0)
				result[id] = zero<T>();
		}
	}
}

extern "C" __global__ __launch_bounds__(1024, 1) void scanPerBlockKernel(
	const PxU32*	data,
	PxU32*			result,
	PxU32*			partialSums,
	const PxU32		length,
	const PxU32		exclusiveScan,
	PxU32*			totalSum)
{
	scanPerBlockKernelShared<PxU32>((blockIdx.x * blockDim.x) + threadIdx.x, data, result, partialSums, length, exclusiveScan, totalSum);
}

__device__ void exclusiveSumInt16(int4x4* values)
{
	__syncthreads();
	PxU32* ptr = reinterpret_cast<PxU32*>(values);
	PxU32 value = threadIdx.x < 16 ? ptr[threadIdx.x] : 0;
	warpScan(value, threadIdx.x % warpSize);
	__syncthreads();
	if (threadIdx.x < 15)
		ptr[threadIdx.x + 1] = value;
	if (threadIdx.x == 15)
		ptr[0] = 0;
}

extern "C" __global__ __launch_bounds__(512, 1) void scanPerBlockKernel4x4(
	const int4x4*	data,
	int4x4*			result,
	int4x4*			partialSums,
	const PxU32		length,
	const PxU32		exclusiveScan,
	int4x4*			totalSum)
{
	int id = ((blockIdx.x * blockDim.x) + threadIdx.x);
	int4x4* r = NULL;
	if (id < length && result)
	{
		if (exclusiveScan == 0)
			r = &result[id];
		else
		{
			if (threadIdx.x + 1 < blockDim.x && id + 1 < length)
				r = &result[id + 1];
			if (threadIdx.x == 0)
			{
				result[id].data[0] = zero<int4>();
				result[id].data[1] = zero<int4>();
				result[id].data[2] = zero<int4>();
				result[id].data[3] = zero<int4>();
			}
		}
	}

	int4 value;
#pragma unroll
	for (PxI32 i = 0; i < 4; ++i) 
	{
		value = id < length ? data[id].data[i] : zero<int4>();
		value = scanPerBlock(value, id, &partialSums[blockIdx.x].data[i]);
		if (r)
			r->data[i] = value;
		if (totalSum && id == length - 1)
			totalSum->data[i] = value;
	}

	if (totalSum && gridDim.x == 1)
	{
		exclusiveSumInt16(totalSum);
	}
}


template<typename T>
__device__ void addBlockSumsKernelShared(const T* partialSums, T* data, const PxU32 len, T* totalSum)
{
	const int id = ((blockIdx.x * blockDim.x) + threadIdx.x);

	if (id >= len)
		return;

	if (totalSum && id == len - 1)
		*totalSum = *totalSum + partialSums[blockIdx.x];

	if (data)
		data[id] = data[id] + partialSums[blockIdx.x];
}

extern "C" __global__ __launch_bounds__(1024, 1) void addBlockSumsKernel(const PxU32* partialSums, PxU32* data, const PxU32 length, PxU32* totalSum)
{
	addBlockSumsKernelShared<PxU32>(partialSums, data, length, totalSum);
}

extern "C" __global__ __launch_bounds__(1024, 1) void addBlockSumsKernel4x4(const int4x4* partialSums, int4x4* data, const PxU32 len, int4x4* totalSum)
{
	const int id = ((blockIdx.x * blockDim.x) + threadIdx.x);

	if (totalSum && id == len - 1)
	{
		(*totalSum).data[0] = (*totalSum).data[0] + partialSums[blockIdx.x].data[0];
		(*totalSum).data[1] = (*totalSum).data[1] + partialSums[blockIdx.x].data[1];
		(*totalSum).data[2] = (*totalSum).data[2] + partialSums[blockIdx.x].data[2];
		(*totalSum).data[3] = (*totalSum).data[3] + partialSums[blockIdx.x].data[3];
	}

	if (data && id < len)
	{
		data[id].data[0] = data[id].data[0] + partialSums[blockIdx.x].data[0];
		data[id].data[1] = data[id].data[1] + partialSums[blockIdx.x].data[1];
		data[id].data[2] = data[id].data[2] + partialSums[blockIdx.x].data[2];
		data[id].data[3] = data[id].data[3] + partialSums[blockIdx.x].data[3];
	}

	if (totalSum && blockIdx.x == gridDim.x - 1)
	{
		exclusiveSumInt16(totalSum);
	}
}

template<typename T>
__device__ void radixFourBitCountPerBlock(const T* data, PxU16* offsetsPerWarp, PxU32 passIndex, int4x4* partialSums, const PxU32 length, int4x4* totalSum)
{
	int* totalSum1 = reinterpret_cast<int*>(totalSum);
	int* partialSums1 = reinterpret_cast<int*>(&partialSums[blockIdx.x]);

	int id = ((blockIdx.x * blockDim.x) + threadIdx.x);
	int slot = 0;
	if (id < length)
		slot = (data[id] >> (passIndex * 4)) & 15;

	PxU64 value;
	PxU64 partial;
#pragma unroll
	for (int i = 0; i < 3; ++i)
	{
		value = 0;
		if (id < length && slot < 6 && slot >= 0)
		{
			value = ((PxU64)1) << (slot * 10);
		}

		value = scanPerBlock<PxU64>(value, id, &partial);
		if (threadIdx.x == blockDim.x - 1)
		{
			partialSums1[6 * i] = partial & 0x000003FF;
			partialSums1[6 * i + 1] = (partial >> 10) & 0x000003FF;
			partialSums1[6 * i + 2] = (partial >> 20) & 0x000003FF;
			partialSums1[6 * i + 3] = (partial >> 30) & 0x000003FF;
			if (i < 2)
			{
				partialSums1[6 * i + 4] = (partial >> 40) & 0x000003FF;
				partialSums1[6 * i + 5] = (partial >> 50) & 0x000003FF;
			}
		}

		if (totalSum && id == length - 1)
		{
			totalSum1[6 * i] = value & 0x000003FF;
			totalSum1[6 * i + 1] = (value >> 10) & 0x000003FF;
			totalSum1[6 * i + 2] = (value >> 20) & 0x000003FF;
			totalSum1[6 * i + 3] = (value >> 30) & 0x000003FF;
			if (i < 2)
			{
				totalSum1[6 * i + 4] = (value >> 40) & 0x000003FF;
				totalSum1[6 * i + 5] = (value >> 50) & 0x000003FF;
			}
		}

		if (id < length && slot < 6 && slot >= 0)
			offsetsPerWarp[id] = ((value >> (slot * 10)) & 0x000003FF) - 1;
		slot -= 6;
	}
}

extern "C" __global__ __launch_bounds__(512, 1) void radixFourBitCountPerBlockKernel(const PxU32* data, PxU16* offsetsPerWarp, PxU32 passIndex, int4x4* partialSums, const PxU32 length, int4x4* totalSum)
{
	radixFourBitCountPerBlock<PxU32>(data, offsetsPerWarp, passIndex, partialSums, length, totalSum);
}

template<typename T, typename U>
__device__ void radixFourBitReorder(const T* data, const PxU16* offsetsPerWarp, T* reordered, PxU32 passIndex, int4x4* partialSums, const PxU32 length, int4x4* cumulativeSum, U* dependentData = NULL, U* dependentDataReordered = NULL)
{
	int* partialSums1 = reinterpret_cast<int*>(partialSums);

	int id = ((blockIdx.x * blockDim.x) + threadIdx.x);
	if (id >= length)
		return;
	int* ptr = reinterpret_cast<int*>(cumulativeSum);
	int slot = (data[id] >> (passIndex * 4)) & 15;

	int newIndex = ptr[slot] + offsetsPerWarp[id] + partialSums1[16 * blockIdx.x + slot];

	if (newIndex < length) //This condition should always be met but in case everything goes wrong, it ensures that no out of bounds access happens
	{
		reordered[newIndex] = data[id];

		if (dependentData && dependentDataReordered)
			dependentDataReordered[newIndex] = passIndex == 0 ? id : dependentData[id];
	}
}

extern "C" __global__ __launch_bounds__(1024, 1) void radixFourBitReorderKernel(const PxU32* data, const PxU16* offsetsPerWarp, PxU32* reordered, PxU32 passIndex, int4x4* partialSums, const PxU32 length, int4x4* cumulativeSum, PxU32* dependentData, PxU32* dependentDataReordered)
{
	radixFourBitReorder<PxU32, PxU32>(data, offsetsPerWarp, reordered, passIndex, partialSums, length, cumulativeSum, dependentData, dependentDataReordered);
}

extern "C" __global__ __launch_bounds__(1024, 1) void reorderKernel(const float4* data, float4* reordered, const PxU32 length, const PxU32* reorderedToOriginalMap)
{
	int id = ((blockIdx.x * blockDim.x) + threadIdx.x);
	if (id >= length)
		return;

	reordered[id] = data[reorderedToOriginalMap[id]];
}

