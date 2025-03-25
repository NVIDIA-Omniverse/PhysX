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


#ifndef __TYPEHELPERS_H__
#define __TYPEHELPERS_H__

PX_CUDA_CALLABLE static inline PxVec3 float4ToVec3(const float4 & data)
{
	return PxVec3(data.x, data.y, data.z);
}
PX_CUDA_CALLABLE static inline float4 vec3ToFloat4(const PxVec3 & data)
{
	return make_float4(data.x, data.y, data.z, 0.0f);
}

PX_CUDA_CALLABLE static inline PxQuat float4ToQuat(const float4 & data)
{
	return PxQuat(data.x, data.y, data.z, data.w);
}
PX_CUDA_CALLABLE static inline float4 quatToFloat4(const PxQuat & data)
{
	return make_float4(data.x, data.y, data.z, data.w);
}

PX_CUDA_CALLABLE static inline void transformToFloat4s(float4 & pos, float4 & rot, const PxTransform & transform)
{
	pos = make_float4(transform.p.x, transform.p.y, transform.p.z, 0.0f);
	rot = make_float4(transform.q.x, transform.q.y, transform.q.z, transform.q.w);
}
PX_CUDA_CALLABLE static inline void float4sToTransform(PxTransform & transform, const float4 & pos, const float4 & rot)
{
	transform.p = PxVec3(pos.x, pos.y, pos.z);
	transform.q = PxQuat(rot.x, rot.y, rot.z, rot.w);
}

#endif
