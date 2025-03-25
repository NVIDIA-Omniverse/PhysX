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

#ifndef MATHS_EXTENSIONS_H
#define MATHS_EXTENSIONS_H

#include "cutil_math.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxQuat.h"
#include "foundation/PxVec3.h"

namespace physx
{

PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal dot3(const float4& x, const float4& y)
{
	return x.x * y.x + x.y * y.y + x.z * y.z;
}

PX_CUDA_CALLABLE PX_FORCE_INLINE float4 rotate(const PxQuat& r, const float4& v)
{
	const PxF32 vx = 2.0f*v.x;
	const PxF32 vy = 2.0f*v.y;
	const PxF32 vz = 2.0f*v.z;
	const PxF32 w2 = r.w*r.w-0.5f;
	const PxF32 dot2 = (r.x*vx + r.y*vy +r.z*vz);
	return make_float4
	(
		(vx*w2 + (r.y * vz - r.z * vy)*r.w + r.x*dot2), 
		(vy*w2 + (r.z * vx - r.x * vz)*r.w + r.y*dot2), 
		(vz*w2 + (r.x * vy - r.y * vx)*r.w + r.z*dot2),
		0.f
	);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE float4 cross3(const float4& v0, const float4& v2)
{
	return make_float4(v0.y * v2.z - v0.z * v2.y, 
					v0.z * v2.x - v0.x * v2.z, 
					v0.x * v2.y - v0.y * v2.x,
					0.f);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE float4 operator - (const float4& v)
{
	return make_float4(-v.x, -v.y, -v.z, -v.w);
}
}

#endif