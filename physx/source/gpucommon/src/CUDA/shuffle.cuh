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

#ifndef __CU_SHUFFLE_CUH__
#define __CU_SHUFFLE_CUH__

#include "cuda.h"
#include "PxgCommonDefines.h"
//#include "nputils.cuh"

static __device__ __forceinline__
physx::PxVec3 shuffle(const physx::PxU32 syncMask, const physx::PxVec3& v, int i, physx::PxU32 width = WARP_SIZE)
{
	return physx::PxVec3(__shfl_sync(syncMask, v.x, i, width), __shfl_sync(syncMask, v.y, i, width), __shfl_sync(syncMask, v.z, i, width));
}

static __device__ __forceinline__
float4 shuffle(const physx::PxU32 syncMask, const float4& v, const int lane)
{
	return make_float4(__shfl_sync(syncMask, v.x, lane), __shfl_sync(syncMask, v.y, lane), __shfl_sync(syncMask, v.z, lane), __shfl_sync(syncMask, v.w, lane));
}

static __device__ __forceinline__
physx::PxVec3 warpShuffleMin(physx::PxVec3 v)
{
	for (physx::PxU32 reductionRadius = 1; reductionRadius < WARP_SIZE; reductionRadius <<= 1)
	{
		v.x = fminf(v.x, __shfl_xor_sync(FULL_MASK, v.x, reductionRadius));
		v.y = fminf(v.y, __shfl_xor_sync(FULL_MASK, v.y, reductionRadius));
		v.z = fminf(v.z, __shfl_xor_sync(FULL_MASK, v.z, reductionRadius));
	}

	return v;
}

static __device__ __forceinline__
physx::PxVec3 warpShuffleMax(physx::PxVec3 v)
{
	for (physx::PxU32 reductionRadius = 1; reductionRadius < WARP_SIZE; reductionRadius <<= 1)
	{
		v.x = fmaxf(v.x, __shfl_xor_sync(FULL_MASK, v.x, reductionRadius));
		v.y = fmaxf(v.y, __shfl_xor_sync(FULL_MASK, v.y, reductionRadius));
		v.z = fmaxf(v.z, __shfl_xor_sync(FULL_MASK, v.z, reductionRadius));
	}

	return v;
}

//// experimentally, seems more register-efficient to coalesce this
//static __device__ __forceinline__
//physx::PxReal shuffleDot(const physx::PxU32 syncMask, const physx::PxVec3& v0, int shuffle0, const physx::PxVec3& v1)
//{
//	return __shfl_sync(syncMask, v0.x, shuffle0)*v1.x + __shfl_sync(syncMask, v0.y, shuffle0)*v1.y + __shfl_sync(syncMask, v0.z, shuffle0)*v1.z;
//}
//
//static __device__ __forceinline__
//physx::PxU32 maxIndex(physx::PxReal v, physx::PxU32 mask, physx::PxReal& maxV)
//{
//	maxV = mask & (1 << threadIdx.x) ? v : -FLT_MAX;
//
//	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 16));
//	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 8));
//	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 4));
//	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 2));
//	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 1));
//
//	return lowestSetIndex(__ballot_sync(FULL_MASK, maxV == v)&mask);
//}
//
//static __device__ __forceinline__
//physx::PxU32 minIndex(physx::PxReal v, physx::PxU32 mask, physx::PxReal& minV)
//{
//	minV = mask & (1 << threadIdx.x) ? v : FLT_MAX;
//
//	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 16));
//	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 8));
//	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 4));
//	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 2));
//	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 1));
//
//	return lowestSetIndex(__ballot_sync(FULL_MASK, minV == v)&mask);
//}


#endif //SHUFFLE_CUH
