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
