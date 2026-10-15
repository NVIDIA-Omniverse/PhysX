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

// Contact block for DB-rigid pairs (SB-rigid, cloth-rigid).
struct PxgRigidDbContactBlock
{
	// resp represents the rigid-body term in the denominator of the impulse calculation (also referred to as the velocity multiplier
	// internally). Also refer to PBD (Position-Based Dynamics) papers.
	float4 raXn_resp[32];
	float4 raXnF0_resp[32];
	float4 raXnF1_resp[32];

	float4 normal_errorW[32];

	// Second tangent is normal x fricTan0.
	float4 fricTan0_invMass0[32];

	PxReal maxPenBiasClamp[32];
};

// Attachment block for DB-rigid pairs (SB-rigid, cloth-rigid).
struct PxgRigidDbAttachmentBlock
{
	float4	barycentric[32];       // unused where deformableRef names a vertex
	float4	raXn0_biasW[32];
	float4	raXn1_biasW[32];
	float4	raXn2_biasW[32];
	float4	velMultiplierXYZ_invMassW[32];
	PxU64	deformableRef[32];     // deformable participant: id, feature type and feature id
	PxU64	rigidId[32];           // node index
	PxU32	rigidBodyRefCount[32]; // active attachments sharing this rigid (mass-splitting)
};

// Contact block for DB-DB pairs (SB-SB, SB-cloth; cloth-cloth recomputes).
struct PxgDbDbContactBlock
{
	float4 normal_pen[32];

	PxReal maxPenBiasClamp[32];
};

// Attachment block for DB-DB pairs (SB-SB, SB-cloth, cloth-cloth).
struct PxgDbDbAttachmentBlock
{
	float4	barycentric0[32];
	float4	barycentric1[32];
	PxU64	deformableRef0[32]; // deformable participant: id, feature type and feature id
	PxU64	deformableRef1[32];
};

// Contact block for DB-particle pairs (SB-particle, cloth-particle).
struct PxgParticleDbContactBlock
{
	float4 normal_pen[32];
	float4 barycentric[32];

	PxReal maxPenBiasClamp[32];
};

} // namespace physx

#endif // PXG_DEFORMABLE_CONSTRAINTS_H
