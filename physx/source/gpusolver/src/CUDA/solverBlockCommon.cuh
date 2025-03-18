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

#ifndef __SOLVER_BLOCK_COMMON_CUH__
#define __SOLVER_BLOCK_COMMON_CUH__

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

#endif
