// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


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
