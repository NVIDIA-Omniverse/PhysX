// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __SOFT_BODY_CUH__
#define __SOFT_BODY_CUH__

#include "foundation/PxVecMath.h"
#include "atomic.cuh"

namespace physx
{

/**
TODO, remove. Already has been removed from softbody/softbody, softbody/femcloth and softbody/particle attachments.
*/
__device__ inline PxReal getSoftBodyInvMass(const PxReal baryMass, const float4& bary)
{
	PxReal scale = PxSqrt(bary.x*bary.x + bary.y*bary.y + bary.z*bary.z + bary.w*bary.w);
	return baryMass * scale;
}

static __device__ inline PxMat33 calculateDeformationGradient(
	const PxVec3& u1,
	const PxVec3& u2,
	const PxVec3& u3,
	const PxMat33& Qinv,
	const PxMat33& RTranspose)
{
	// calculate deformation gradient
	PxMat33 P = PxMat33(u1, u2, u3);
	PxMat33 F = P * Qinv;

	// remove rotation factor from strain, tranfrom into element space
	F = RTranspose * F;

	return F;
}

static __device__ float4 computeTetraContact(const float4* const vels, const uint4& tetrahedronId,
	const float4& barycentric, float4& invMass)
{
	const float4 v0 = vels[tetrahedronId.x];
	const float4 v1 = vels[tetrahedronId.y];
	const float4 v2 = vels[tetrahedronId.z];
	const float4 v3 = vels[tetrahedronId.w];

	invMass = make_float4(v0.w, v1.w, v2.w, v3.w);

	const float4 vel = v0 * barycentric.x + v1 * barycentric.y
		+ v2 * barycentric.z + v3 * barycentric.w;

	return vel;
}

static __device__ void updateTetraPosDelta(const float4& invMasses, const float4& barycentric, const uint4& tetrahedronId,
	const PxVec3& deltaPos, float4* outputDeltaPoses, const PxReal addition = 1.f)
{
	if (invMasses.x > 0.f && PxAbs(barycentric.x) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * (invMasses.x * barycentric.x);
		AtomicAdd(outputDeltaPoses[tetrahedronId.x], dP, addition);
	}

	if (invMasses.y > 0.f && PxAbs(barycentric.y) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * (invMasses.y * barycentric.y);
		AtomicAdd(outputDeltaPoses[tetrahedronId.y], dP, addition);
	}

	if (invMasses.z > 0.f && PxAbs(barycentric.z) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * (invMasses.z * barycentric.z);
		AtomicAdd(outputDeltaPoses[tetrahedronId.z], dP, addition);
	}

	if (invMasses.w > 0.f && PxAbs(barycentric.w) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * (invMasses.w * barycentric.w);
		AtomicAdd(outputDeltaPoses[tetrahedronId.w], dP, addition);
	}
}

} // namespace physx

#endif