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

#define IS_TGS_SOLVER

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

extern "C" __host__ void initSolverKernels11() {}

//KS - this will change dramatically once we have all the TGS functionality working!
extern "C" __global__ void integrateCoreParallelLaunchTGS(
	const uint32_t offset, const PxgSolverCoreDesc* PX_RESTRICT solverCoreDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* PX_RESTRICT sharedDesc,
	const PxU32* PX_RESTRICT islandIds,
	const PxU32* PX_RESTRICT islandStaticTouchCounts,
	const PxU32* PX_RESTRICT numCountedInteractions)
{
	//integrateCoreParallel(motionVelocity, solverBody, solverBodyData, numBodies, dt);
	uint32_t idx = threadIdx.x + blockIdx.x * blockDim.x;
	const float4* PX_RESTRICT motionVelocityArray = solverCoreDesc->motionVelocityArray;
	const uint32_t numSolverBodies = solverCoreDesc->numSolverBodies;
	const float4* PX_RESTRICT solverBodyVelocity = sharedDesc->iterativeData.solverBodyVelPool;

	const PxgSolverTxIData* PX_RESTRICT txIDatas = solverCoreDesc->solverBodyTxIDataPool;

	float4* PX_RESTRICT outSolverVelocity = solverCoreDesc->outSolverVelocity;
	PxAlignedTransform* PX_RESTRICT outBody2World = solverCoreDesc->outBody2World;

	//for(uint32_t a = idx+offset; a < numSolverBodies; a+=blockSize)
	uint32_t a = idx + offset;
	if (a < numSolverBodies)
	{
		const PxgSolverBodyData& data = solverCoreDesc->solverBodyDataPool[a];

		const PxU32 nodeIndex = data.islandNodeIndex.index();// >> 2;

		PxgBodySim&	bodySim = solverCoreDesc->mBodySimBufferDeviceData[nodeIndex];

		// PT: TODO: PGS version uses a reference here, what's better?
		const PxMat33 sqrtInvInertia = txIDatas[a].sqrtInvInertia;
		const PxTransform deltaTransform = txIDatas[a].deltaBody2World;

		PxAlignedTransform body2World = outBody2World[a];	// PT: TODO: PGS version uses bodySim.body2World here, why?

		const float4 inverseInertia = bodySim.inverseInertiaXYZ_contactReportThresholdW;

		const float4 motionLinVel = motionVelocityArray[a];
		const float4 motionAngVel = motionVelocityArray[a + numSolverBodies];

		//if (index == PxgSolverBody::InvalidHandle)
		//{
		//	const float4 zero4 = make_float4(0.f);
		//	linVel = motionLinVel;
		//	angVel = motionAngVel;
		//}
		//else
		//{
		//	//PxU32 ind = 3*(index&(~31)) + (index&31);
		//	PxU32 ind = index;
		//	float4 linxyz_angx = solverBodyVelocity[ind];
		//	float4 angyz_lindxy = solverBodyVelocity[ind + 32];
		//	linVel = make_float4(linxyz_angx.x, linxyz_angx.y, linxyz_angx.z, 0.f);
		//	angVel = make_float4(linxyz_angx.w, angyz_lindxy.x, angyz_lindxy.y, 0.f);
		//}

		const PxU32 readIndex = solverCoreDesc->accumulatedBodyDeltaVOffset + a;

		float4 v0 = solverBodyVelocity[readIndex];
		float4 v1 = solverBodyVelocity[readIndex + numSolverBodies];

		// PT: TODO: PGS version doesn't use tmp v0/v1 values here, why?
		float4 linVel = make_float4(v0.x, v0.y, v0.z, 0.f);
		float4 angVel = make_float4(v0.w, v1.x, v1.y, 0.f);

		const PxU32 staticTouchCount = islandStaticTouchCounts[islandIds[nodeIndex]];

		//this array need to be copied back to CPU
		PxgSolverBodySleepData& sleepData = solverCoreDesc->solverBodySleepDataPool[a];

		integrateCoreTGS(motionLinVel, motionAngVel, inverseInertia, linVel, angVel, body2World, deltaTransform, data, bodySim, sleepData, sqrtInvInertia,
			sharedDesc->dt, sharedDesc->invDtF32, solverCoreDesc->enableStabilization, staticTouchCount != 0, numCountedInteractions[nodeIndex],
			nodeIndex);

		// PT: TODO: why do we write out the vels & pose to 2 different buffers?
		outSolverVelocity[a] = linVel;
		outSolverVelocity[a + numSolverBodies] = angVel;
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
