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

#ifndef __SOFT_BODY_CUH__
#define __SOFT_BODY_CUH__

#include "foundation/PxVecMath.h"
#include "atomic.cuh"

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

static __device__ void updateTetPositionDelta(float4* outputDeltaPositions, const uint4& tetVertIndices,
	const PxVec3& deltaPosition, const float4& invMassBary, const PxReal constraintWeight)
{
	//testing inverse mass and barycentric product for > 0, assuming that barycentric coordinates where clamped on construction.
	if (invMassBary.x > 0.0f)
	{
		AtomicAdd(outputDeltaPositions[tetVertIndices.x], deltaPosition*invMassBary.x, constraintWeight);
	}

	if (invMassBary.y > 0.0f)
	{
		AtomicAdd(outputDeltaPositions[tetVertIndices.y], deltaPosition*invMassBary.y, constraintWeight);
	}

	if (invMassBary.z > 0.0f)
	{
		AtomicAdd(outputDeltaPositions[tetVertIndices.z], deltaPosition*invMassBary.z, constraintWeight);
	}

	if (invMassBary.w > 0.0f)
	{
		AtomicAdd(outputDeltaPositions[tetVertIndices.w], deltaPosition*invMassBary.w, constraintWeight);
	}
}

static __device__ void updateTriPositionDelta(float4* outputDeltaPositions, const uint4& triVertIndices,
	const PxVec3& deltaPosition, const float4& invMassBary, const PxReal constraintWeight)
{
	//testing inverse mass and barycentric product for > 0, assuming that barycentric coordinates where clamped on construction.
	if (invMassBary.x > 0.0f)
	{
		AtomicAdd(outputDeltaPositions[triVertIndices.x], deltaPosition*invMassBary.x, constraintWeight);
	}

	if (invMassBary.y > 0.0f)
	{
		AtomicAdd(outputDeltaPositions[triVertIndices.y], deltaPosition*invMassBary.y, constraintWeight);
	}

	if (invMassBary.z > 0.0f)
	{
		AtomicAdd(outputDeltaPositions[triVertIndices.z], deltaPosition*invMassBary.z, constraintWeight);
	}
}

#endif