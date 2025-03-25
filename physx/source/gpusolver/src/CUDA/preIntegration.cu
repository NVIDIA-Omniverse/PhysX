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

#include "preIntegration.cuh"

extern "C" __host__ void initSolverKernels4() {}

extern "C" __global__ void preIntegrationLaunch(
	const uint32_t offset, const uint32_t nbSolverBodies, const PxReal dt, const PxVec3 gravity, PxgSolverBodyData* PX_RESTRICT solverBodyDataPool,
	PxgSolverBodySleepData* PX_RESTRICT solverBodySleepDataPool, PxgSolverTxIData* PX_RESTRICT solverTxIDataPool,
	const PxgBodySim* PX_RESTRICT bodySimPool, const PxNodeIndex* PX_RESTRICT islandNodeIndices,
	PxAlignedTransform* gTransforms, float4* gOutVelocityPool, PxU32* solverBodyIndices)
{
	preIntegration(offset, nbSolverBodies, dt, gravity, solverBodyDataPool, solverBodySleepDataPool, solverTxIDataPool, 
		bodySimPool, islandNodeIndices, gTransforms, gOutVelocityPool, solverBodyIndices);
}

extern "C" __global__ void initStaticKinematics(
	const uint32_t nbStaticKinematics, const uint32_t nbSolverBodies, PxgSolverBodyData* PX_RESTRICT solverBodyDataPool,
	PxgSolverTxIData* PX_RESTRICT solverTxIDataPool, PxAlignedTransform* gTransforms, float4* gOutVelocityPool, 
	PxNodeIndex* activeNodeIndices, PxU32* solverBodyIndices)
{
	const uint32_t idx = threadIdx.x + blockIdx.x * blockDim.x;

	if(idx < nbStaticKinematics)
	{
		//KS - TODO - Optimize these reads/writes
		const PxNodeIndex index = activeNodeIndices[idx];
		if (!index.isStaticBody())
		{
			solverBodyIndices[index.index()] = idx;
		}
		gTransforms[idx] = solverBodyDataPool[idx].body2World;
		gOutVelocityPool[idx] = solverBodyDataPool[idx].initialLinVelXYZ_invMassW;
		gOutVelocityPool[idx + nbSolverBodies] = solverBodyDataPool[idx].initialAngVelXYZ_penBiasClamp;
		solverTxIDataPool[idx].deltaBody2World = PxTransform(PxIdentity);
		solverTxIDataPool[idx].sqrtInvInertia = PxMat33(PxZero);
	}
}
