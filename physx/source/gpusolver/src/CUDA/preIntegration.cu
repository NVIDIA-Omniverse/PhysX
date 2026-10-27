// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "preIntegration.cuh"

using namespace physx;

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
