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

#ifndef __NPUTILS_CUH__
#define __NPUTILS_CUH__

#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"
#include "PxgCommonDefines.h"
#include "utils.cuh"

template <typename T> __device__ static inline T ldS(const volatile T& val) { return val; }
template <typename T> __device__ static inline void stS(volatile T& dst, const T& src) { dst = src; }

template <> __device__ inline physx::PxVec3 ldS<physx::PxVec3>(const volatile physx::PxVec3& val) { return physx::PxVec3(val.x, val.y, val.z); }
template <> __device__ inline void stS<physx::PxVec3>(volatile physx::PxVec3& dst, const physx::PxVec3& src) { dst.x = src.x, dst.y = src.y, dst.z = src.z; }

template <> __device__ __forceinline__ physx::PxQuat ldS<physx::PxQuat>(const volatile physx::PxQuat& val) { return physx::PxQuat(val.x, val.y, val.z, val.w); }
template <> __device__ __forceinline__ void stS<physx::PxQuat>(volatile physx::PxQuat& dst, const physx::PxQuat& src) { dst.x = src.x, dst.y = src.y, dst.z = src.z, dst.w = src.w; }

template <> __device__ __forceinline__ physx::PxTransform ldS<physx::PxTransform>(const volatile physx::PxTransform& val) { return physx::PxTransform(ldS(val.p), ldS(val.q)); }
template <> __device__ __forceinline__ void stS<physx::PxTransform>(volatile physx::PxTransform& dst, const physx::PxTransform& src)
{ 
	stS(dst.p, src.p); 
	stS(dst.q, src.q); 
}

static __device__ __forceinline__ int next3(int x)
{
	return x==2 ? 0 : x+1;
}

static __device__ inline int prev3(int x)
{
	return x==0 ? 2 : x-1;
}


// operators dealing with 8 4-vectors in a warp
static __device__ inline float dot(float a, float b)	// leaves answer in x
{
	float x = a*b;
	return x + __shfl_down_sync(FULL_MASK, x,1) + __shfl_down_sync(FULL_MASK, x,2);
}

static __device__ inline float cross(float a, float b, int cp0, int cp1)
{
	return __shfl_sync(FULL_MASK, a, cp0) * __shfl_sync(FULL_MASK, b, cp1) - __shfl_sync(FULL_MASK, a, cp1) * __shfl_sync(FULL_MASK, b, cp0);
}

static __device__ inline float splatV(int index, float v)	// splat a vector across all 8 vectors
{
	return __shfl_sync(FULL_MASK, v, (index << 2) | threadIdx.x & 3);
}

static __device__ inline float splatX(float a)				// splat the x-component of each vector into yzw
{
	return __shfl_sync(FULL_MASK, a, threadIdx.x & 28);
}
	
static __device__ inline float normalize(float x)			// normalize each vector
{	
	return x * splatX(rsqrt(dot(x,x)));		// TODO: precision
}

static __device__ inline float magnitude(float x)			// normalize each vector
{	
	return splatX(sqrt(dot(x,x)));		// TODO: precision
}

template<int N>
static __device__ inline int packXFlags(bool a)				// pack a boolean from each x into a byte
{
	return __ballot_sync(FULL_MASK, __shfl_sync(FULL_MASK, a, threadIdx.x<<2)) & ((1<<N)-1);
}

template<int N>
static __device__ inline bool allX(bool a)					// check some number initial set of the x-flags are all set
{
	return packXFlags<N>(a) == ((1<<N)-1);
}

template<int N>
static __device__ inline bool anyX(bool a)					// check if any of an initial segment of x-flags are all set
{
	return packXFlags<N>(a) != 0;
}

static __device__ inline float loadV3Unsafe(const physx::PxVec3* v)	// NB requires padding (i.e. the 4th element is loaded)
{
	return reinterpret_cast<const float*>(v)[threadIdx.x&3];
}

static __device__ void storeV3(volatile physx::PxVec3* v, int index, float value)
{
	int lane = index<<2;
	volatile float* dest = reinterpret_cast<volatile float*>(v);
	if(threadIdx.x>=lane && threadIdx.x<lane+3)
	{
		dest[threadIdx.x&3] = value;
	}
}

// experimentally, seems more register-efficient to coalesce this
static __device__ __forceinline__
physx::PxReal shuffleDot(const physx::PxU32 syncMask, const physx::PxVec3& v0, int shuffle0, const physx::PxVec3& v1)
{
	return __shfl_sync(syncMask, v0.x, shuffle0)*v1.x + __shfl_sync(syncMask, v0.y, shuffle0)*v1.y + __shfl_sync(syncMask, v0.z, shuffle0)*v1.z;
}

static __device__ __forceinline__
physx::PxU32 maxIndex(physx::PxReal v, physx::PxU32 mask, physx::PxReal& maxV)
{
	maxV = mask & (1 << (threadIdx.x&31)) ? v : -FLT_MAX;

	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 16));
	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 8));
	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 4));
	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 2));
	maxV = fmaxf(maxV, __shfl_xor_sync(FULL_MASK, maxV, 1));

	return physx::lowestSetIndex(__ballot_sync(FULL_MASK, maxV == v)&mask);
}

static __device__ __forceinline__
physx::PxU32 minIndex(physx::PxReal v, physx::PxU32 mask, physx::PxReal& minV)
{
	minV = mask & (1 << (threadIdx.x & 31)) ? v : FLT_MAX;

	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 16));
	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 8));
	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 4));
	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 2));
	minV = fminf(minV, __shfl_xor_sync(FULL_MASK, minV, 1));

	return physx::lowestSetIndex(__ballot_sync(FULL_MASK, minV == v)&mask);
}

// similar as above but only with blocks of 4 threads
// mask must have exactly 4 consecutive bits set and the corresponding
// threads must execute this function together
static __device__ __forceinline__
physx::PxU32 minIndex4(physx::PxReal v, physx::PxU32 mask, physx::PxU32 threadIndexInGroup)
{
	// sanity checks that function has been called as expected
	assert(__popc(mask) == 4); // exactly 4 threads in mask
	assert(mask & (1 << (threadIdx.x & 31))); // executing thread must be in the mask
	assert(threadIndexInGroup == 0 || (mask & (1 << (threadIdx.x - 1 & 31)))); // threads consecutive

	physx::PxReal minV =  v;

	physx::PxReal minV_m1 = __shfl_sync(mask, minV, (threadIdx.x & 31) - 1);
	if(threadIndexInGroup > 0)
		minV = fminf(minV, minV_m1);

	physx::PxReal minV_m2 = __shfl_sync(mask, minV, (threadIdx.x & 31) - 2);
	if(threadIndexInGroup > 2)
		minV = fminf(minV, minV_m2);

	// Now the last thread (idx 3) of the group knows the min.
	// Send it back to all the threads.
	minV = __shfl_sync(mask, minV, (threadIdx.x & 31) | 3);

	return physx::lowestSetIndex(__ballot_sync(mask, minV == v)&mask);
}

#endif
