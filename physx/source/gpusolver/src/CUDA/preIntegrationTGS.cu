// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#define IS_TGS_SOLVER

#include "preIntegration.cuh"

using namespace physx;

extern "C" __host__ void initSolverKernels9() {}

extern "C" __global__ void preIntegrationLaunchTGS(
	const uint32_t offset, const uint32_t nbSolverBodies, const PxReal dt, const PxVec3 gravity, PxgSolverBodyData* PX_RESTRICT solverBodyDataPool,
	PxgSolverBodySleepData* PX_RESTRICT solverBodySleepDataPool, PxgSolverTxIData* PX_RESTRICT solverTxIDataPool,
	const PxgBodySim* const PX_RESTRICT bodySimPool, const PxNodeIndex* const PX_RESTRICT islandNodeIndices,
	PxAlignedTransform* gTransforms, float4* gOutVelocityPool, PxU32* solverBodyIndices, bool skipGravityApplication)
{
	preIntegration(offset, nbSolverBodies, dt, gravity, solverBodyDataPool, solverBodySleepDataPool, solverTxIDataPool, 
		bodySimPool, islandNodeIndices, gTransforms, gOutVelocityPool, solverBodyIndices, skipGravityApplication);
}
