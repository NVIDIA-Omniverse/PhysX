// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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