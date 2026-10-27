// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
	PxU32	elemId[32];
	PxU64	rigidId[32];					//node index
	PxU32	rigidBodyReferenceCount[32];	//number of active attachments sharing this rigid; for mass-splitting
};

struct PxgFEMFEMAttachmentConstraint
{
	float4	barycentric0[32];
	float4	barycentric1[32];
	PxU64	elemId0[32]; //can be triangleId(cloth) or tetrahedron index
	PxU64	elemId1[32];//can be triangleId(cloth) or tetrahedron index
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
