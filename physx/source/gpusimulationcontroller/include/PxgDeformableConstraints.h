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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PXG_DEFORMABLE_CONSTRAINTS_H
#define PXG_DEFORMABLE_CONSTRAINTS_H

#include "foundation/PxSimpleTypes.h"
#include <vector_types.h>

namespace physx
{

// Constraint block structs used by CUDA kernels (shared between FEM cloth and soft body)
struct PxgFemRigidConstraintBlock
{
	// resp represents the rigid-body term in the denominator of the impulse calculation (also referred to as the velocity multiplier
	// internally). Also refer to PBD (Position-Based Dynamics) papers.
	float4 raXn_resp[32];
	float4 raXnF0_resp[32];
	float4 raXnF1_resp[32];

	float4 normal_errorW[32];

	// Friction tangent + invMass of the rigid body (avoids needing to read the mass)
	// Second tangent can be found by cross producting normal with fricTan0
	float4 fricTan0_invMass0[32];
	float4 barycentric[32];
	PxReal maxPenBias[32];
};

struct PxgFEMParticleConstraintBlock
{
	float4 normal_pen[32];
	float4 barycentric[32];
	PxReal velMultiplier[32];
};

struct PxgFEMRigidAttachmentConstraint
{
	float4	baryOrType[32];
	float4	raXn0_biasW[32];
	float4	raXn1_biasW[32];
	float4	raXn2_biasW[32];
	float4	velMultiplierXYZ_invMassW[32];
	float4  low_high_limits[32];
	float4  axis_angle[32];
	PxU32	elemId[32];
	PxU64	rigidId[32];		//node index
};

struct PxgFEMFEMAttachmentConstraint
{
	union
	{
		float4  low_high_limits[32];
		float4  low_high_angles[32];
	};

	union
	{
		float4  axis_angle[32];
		float4  attachmentBarycentric[32];
	};
	
	float4	barycentric0[32];
	float4	barycentric1[32];
	PxU64	elemId0[32]; //can be triangleId(cloth) or tetrahedron index
	PxU64	elemId1[32];//can be triangleId(cloth) or tetrahedron index
	float	constraintOffset[32];
};

// Soft body vs soft body constraint block
struct PxgSoftBodySoftBodyConstraintBlock
{
	float4 barycentric0[32];
	float4 barycentric1[32];
	float4 normal_pen[32];
	PxReal velMultiplier[32];
	PxReal friction[32];
};

} // namespace physx

#endif // PXG_DEFORMABLE_CONSTRAINTS_H
