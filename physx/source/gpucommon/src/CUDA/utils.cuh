// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __CU_UTILS_CUH__
#define __CU_UTILS_CUH__

#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"

namespace physx
{
	__device__ PX_FORCE_INLINE PxVec3 PxLoad3(const float4& v) { float4 tmp = v; return PxVec3(tmp.x, tmp.y, tmp.z); }
	__device__ PX_FORCE_INLINE PxVec3 PxLoad3(const float4& v, float& w) { float4 tmp = v; w = tmp.w; return PxVec3(tmp.x, tmp.y, tmp.z); }
	__device__ PX_FORCE_INLINE PxVec4 PxLoad4(const float4& v) { float4 tmp = v; return PxVec4(tmp.x, tmp.y, tmp.z, tmp.w); }
	__device__ PX_FORCE_INLINE float4 PxSave3(const PxVec3& v) { return float4({ v.x, v.y, v.z, 0 }); }
	__device__ PX_FORCE_INLINE float4 PxSave4(const PxVec4& v) { return float4({ v.x, v.y, v.z, v.w }); }


	//Only works if val > 0
	__device__ PX_FORCE_INLINE int lowestSetIndex(int val) { return __ffs(val) - 1; }
	__device__ PX_FORCE_INLINE int highestSetIndex(int val) { return 31 - __clz(val); }
	__device__ PX_FORCE_INLINE int lowestSetBit(int val) { return val & -val; }
	__device__ PX_FORCE_INLINE bool testBit(int map, int index) { return (map & 1 << index) != 0; }

	//Returns the index of the lowest set bit. Returns 0xFFffFFff is not bit is set
	__device__ PX_FORCE_INLINE PxU32 lowestSetIndex(PxU32 val) { return __ffs(val) - 1; }
	__device__ PX_FORCE_INLINE PxU32 clearLowestSetBit(PxU32 val) { return val & (val - 1); }
}

#endif
