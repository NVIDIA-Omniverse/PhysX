// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __SOLVER_BLOCK_COMMON_CUH__
#define __SOLVER_BLOCK_COMMON_CUH__

#include "PxgFrictionPatch.h"

namespace physx
{

template <typename FRICTION_HEADER, typename FRICTION>
static __device__ void writeBackContactBlockFriction(
	const PxU32 threadIndex, PxU32	numFrictionConstr, const FRICTION_HEADER* PX_RESTRICT frictionHeader,
	PxgBlockFrictionPatch& frictionPatchBlock, FRICTION* fric, PxgFrictionPatchGPU* frictionPatches
)
{
	PxU32 patchIndex = frictionPatchBlock.patchIndex[threadIndex];
	if (patchIndex != 0xFFFFFFFF)
	{
		PxgFrictionPatchGPU& frictionInfo = frictionPatches[patchIndex];

		float4 axis0 = frictionHeader->frictionNormals[0][threadIndex];
		float4 axis1 = frictionHeader->frictionNormals[1][threadIndex];

		frictionInfo.anchors = numFrictionConstr / 2;

		if (numFrictionConstr >= 2)
		{
			float4 anchor = frictionPatchBlock.anchorPoints[0][threadIndex];
			frictionInfo.points[0] = PxVec3(anchor.x, anchor.y, anchor.z);
			PxReal impulse0 = fric[0].appliedForce[threadIndex];
			PxReal impulse1 = fric[1].appliedForce[threadIndex];
			frictionInfo.impulses[0] = PxVec3(axis0.x, axis0.y, axis0.z) * impulse0 + PxVec3(axis1.x, axis1.y, axis1.z) * impulse1;
		}
		if (numFrictionConstr >= 4)
		{
			float4 anchor = frictionPatchBlock.anchorPoints[1][threadIndex];
			frictionInfo.points[1] = PxVec3(anchor.x, anchor.y, anchor.z);
			PxReal impulse0 = fric[2].appliedForce[threadIndex];
			PxReal impulse1 = fric[3].appliedForce[threadIndex];
			frictionInfo.impulses[1] = PxVec3(axis0.x, axis0.y, axis0.z) * impulse0 + PxVec3(axis1.x, axis1.y, axis1.z) * impulse1;
		}
	}
}

} // namespace physx

#endif
