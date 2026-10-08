// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __CU_ATOMIC_CUH__
#define __CU_ATOMIC_CUH__

#include "cuda.h"
#include "foundation/PxVec3.h"
#include "foundation/PxSimpleTypes.h"
#include "PxgIntrinsics.h"
#include "PxgArticulationBlockData.h"

namespace physx
{

static __device__ inline void AtomicAdd(float4& a, const float4 b)
{
	atomicAdd(&a.x, b.x);
	atomicAdd(&a.y, b.y);
	atomicAdd(&a.z, b.z);
	atomicAdd(&a.w, b.w);
}

static __device__ inline void AtomicAdd(float4& a, const PxVec3 b, const PxReal w)
{
	atomicAdd(&a.x, b.x);
	atomicAdd(&a.y, b.y);
	atomicAdd(&a.z, b.z);
	atomicAdd(&a.w, w);
}

static __device__ inline void AtomicAdd(float4& a, const PxVec3 b)
{
	atomicAdd(&a.x, b.x);
	atomicAdd(&a.y, b.y);
	atomicAdd(&a.z, b.z);
}


__device__ inline void AtomicAdd(float* p, PxU32 i, const PxReal val)
{
	atomicAdd(&p[i], val);
}

__device__ inline void AtomicAdd(float4* p, PxU32 i, const PxVec3& v, PxReal w)
{
	atomicAdd(&p[i].x, v.x);
	atomicAdd(&p[i].y, v.y);
	atomicAdd(&p[i].z, v.z);
	atomicAdd(&p[i].w, w);
}

__device__ inline void AtomicAdd(float4* p, PxU32 i, const PxVec4& v)
{
	atomicAdd(&p[i].x, v.x);
	atomicAdd(&p[i].y, v.y);
	atomicAdd(&p[i].z, v.z);
	atomicAdd(&p[i].w, v.w);
}

__device__ inline void AtomicAdd(float4* p, PxU32 i, const PxVec3& v)
{
	atomicAdd(&p[i].x, v.x);
	atomicAdd(&p[i].y, v.y);
	atomicAdd(&p[i].z, v.z);
}

__device__ inline void AtomicAdd3(float4* p, PxU32 i, const float4& v)
{
	atomicAdd(&p[i].x, v.x);
	atomicAdd(&p[i].y, v.y);
	atomicAdd(&p[i].z, v.z);
}

__device__ inline void AtomicAdd3(PxVec3& p, const PxVec3& v)
{
	atomicAdd(&p.x, v.x);
	atomicAdd(&p.y, v.y);
	atomicAdd(&p.z, v.z);
}

__device__ inline float AtomicMin(float* address, float val)
{
	int *address_as_int = (int*)address;
	int old = *address_as_int, assumed;

	while (val < __int_as_float(old))
	{
		assumed = old;
		old = atomicCAS(address_as_int, assumed,
			__float_as_int(val));
	}

	return __int_as_float(old);
}

inline __device__ float AtomicMax(float* address, float val)
{
	int *address_as_int = (int*)address;
	int old = *address_as_int, assumed;

	while (val > __int_as_float(old))
	{
		assumed = old;
		old = atomicCAS(address_as_int, assumed,
			__float_as_int(val));
	}

	return __int_as_float(old);
}


//Some compiler was complaining about not supporting atomicOr on 64bit integers
PX_FORCE_INLINE static __device__ void AtomicOr(PxU64* address, const PxU64 mask)
{
	PxU32* address32 = reinterpret_cast<PxU32*>(address);
	const PxU32* maskPtr = reinterpret_cast<const PxU32*>(&mask);
	atomicOr(address32, maskPtr[0]);
	atomicOr(address32 + 1, maskPtr[1]);
}

/* use inline assembly with .global qualifier to perform the operation at the L2 cache
 * adds 20% performance in FLIP P2G compared to atomicAdd() or plain red.add.f32 */
PX_FORCE_INLINE __device__ void PxRedAddGlobal(float* addr, const float val)
{
#if __CUDA_ARCH__ >= 350
	asm volatile ("red.global.add.f32 [%0], %1;" :: __STG_PTR(addr) , "f"(val));
#else
#if __CUDA_ARCH__ >= 200
	atomicAdd(addr, val);
#else
	PX_UNUSED(addr);
	PX_UNUSED(val);
#endif
#endif
}

} // namespace physx

#endif