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

#include "PxgBodySim.h"
#include "PxgSolverBody.h"
#include "PxgSolverCoreDesc.h"
#include "DySleepingConfigulation.h"
#include "PxvDynamics.h"
#include "PxsRigidBody.h"
#include "assert.h"
#include "stdio.h"
#include "integration.cuh"

using namespace physx;

extern "C" __host__ void initSolverKernels3() {}

extern "C" __global__ void integrateCoreParallelLaunch(
	const uint32_t offset, const PxgSolverCoreDesc* PX_RESTRICT solverCoreDesc,
	const PxgSolverSharedDesc<IterativeSolveData>* PX_RESTRICT sharedDesc,
	const PxU32* PX_RESTRICT islandIds, 
	const PxU32* PX_RESTRICT islandStaticTouchCounts,
	const PxU32* PX_RESTRICT numCountedInteractions)
{	
	//integrateCoreParallel(motionVelocity, solverBody, solverBodyData, numBodies, dt);
	uint32_t idx = threadIdx.x + blockIdx.x * blockDim.x;
	const float4* PX_RESTRICT motionVelocityArray = solverCoreDesc->motionVelocityArray;
	const uint32_t numSolverBodies = solverCoreDesc->numSolverBodies;
	const float4* PX_RESTRICT solverBodyVelocity = sharedDesc->iterativeData.solverBodyVelPool;

	const PxgSolverTxIData* PX_RESTRICT txIData = solverCoreDesc->solverBodyTxIDataPool;

	float4* PX_RESTRICT outSolverVelocity = solverCoreDesc->outSolverVelocity;
	PxAlignedTransform* PX_RESTRICT outBody2World = solverCoreDesc->outBody2World;

	//for(uint32_t a = idx+offset; a < numSolverBodies; a+=blockSize)
	uint32_t a = idx+offset;
	if(a < numSolverBodies)
	{
		const PxgSolverBodyData& data = solverCoreDesc->solverBodyDataPool[a];

		const PxU32 nodeIndex = data.islandNodeIndex.index();// >> 2;

		PxgBodySim&	bodySim = solverCoreDesc->mBodySimBufferDeviceData[nodeIndex];

		//KS - TODO - access all data via shared memory
		// PT: TODO: TGS version uses a copy here, what's better?
		const PxMat33& sqrtInvInertia = txIData[a].sqrtInvInertia;
		
		PxAlignedTransform body2World = bodySim.body2World;	// PT: TODO: TGS version uses outBody2World[a] here, why?
		const float4 inverseInertia = bodySim.inverseInertiaXYZ_contactReportThresholdW;

		const PxU32 index = solverCoreDesc->accumulatedBodyDeltaVOffset + a;

		// PT: TODO: TGS version uses tmp v0/v1 values here, why?
		float4 linVel = solverBodyVelocity[index];
		float4 angVel = solverBodyVelocity[index + numSolverBodies];

		const PxU32 staticTouchCount = islandStaticTouchCounts[islandIds[nodeIndex]];

		//printf("Integrating %i: index = %i, a = %i\n", nodeIndex, index, a);

		//we need to dma the sleep data back for post solver task
		PxgSolverBodySleepData& sleepData = solverCoreDesc->solverBodySleepDataPool[a];

		integrateCore(motionVelocityArray[a], motionVelocityArray[a + numSolverBodies], inverseInertia, linVel, angVel, body2World, data, bodySim, sleepData, sqrtInvInertia,
			sharedDesc->dt, sharedDesc->invDtF32, solverCoreDesc->enableStabilization, staticTouchCount != 0, numCountedInteractions[nodeIndex],
			nodeIndex);

		// PT: TODO: why do we write out the vels & pose to 2 different buffers?
		outSolverVelocity[a] = linVel;
		outSolverVelocity[a+numSolverBodies] = angVel;
		outBody2World[a] = body2World;

		// PT: for acceleration getters (eENABLE_BODY_ACCELERATIONS)
		PxgBodySimVelocities* prevVelocities = solverCoreDesc->mBodySimPrevVelocitiesBufferDeviceData;
		if(prevVelocities)
		{
			PxgBodySimVelocities& prev = prevVelocities[nodeIndex];
			prev.linearVelocity = bodySim.linearVelocityXYZ_inverseMassW;
			prev.angularVelocity = bodySim.angularVelocityXYZ_maxPenBiasW;
		}

		//write back linear velocity, angular velocity to pxgbodysim
		bodySim.linearVelocityXYZ_inverseMassW.x = linVel.x; bodySim.linearVelocityXYZ_inverseMassW.y = linVel.y; bodySim.linearVelocityXYZ_inverseMassW.z = linVel.z;
		bodySim.angularVelocityXYZ_maxPenBiasW.x = angVel.x; bodySim.angularVelocityXYZ_maxPenBiasW.y = angVel.y; bodySim.angularVelocityXYZ_maxPenBiasW.z = angVel.z;
		bodySim.body2World = body2World;
		assert(body2World.isSane());
	}
}
