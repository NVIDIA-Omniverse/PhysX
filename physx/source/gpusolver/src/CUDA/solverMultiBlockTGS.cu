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

#include "PxgCommonDefines.h"
#include "common/PxPhysXCommonConfig.h"
#include <cuda.h>
#include <sm_35_intrinsics.h>
#include "PxgSolverBody.h"
#include "PxgSolverConstraintBlock1D.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgConstraint.h"
#include "PxgConstraintBlock.h"
#include "PxgIslandContext.h"
#include "PxgSolverContext.h"
#include "cutil_math.h"
#include "PxgSolverCoreDesc.h"
#include "solverBlockTGS.cuh"
#include "solverBlock.cuh"
#include "PxgSolverKernelIndices.h"
#include "PxgDynamicsConfiguration.h"
#include "stdio.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgArticulation.h"
#include "solver.cuh"
#include "PxgBodySim.h"
#include "PxRigidDynamic.h"
#include "reduction.cuh"
#include "PxgBodySimManager.h"

using namespace physx;

extern "C" __host__ void initSolverKernels5() {}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 ComputeAverageBodyOutputIndex(const PxU32 bodyIndex)
{
	return 3*(bodyIndex & (~31)) + (bodyIndex &31);
}

extern "C" __global__ void ZeroBodiesTGS(const PxgSolverCoreDesc* constraintPrepDesc, const PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc)
{
	// The reference count used to be reset in this kernel.
	// This is no longer necessary, as it is now initialized immediately after allocation.

	__shared__ float4* bodyVelocities;
	__shared__ float4* motionVelocities;
	__shared__ uint totalNumBodies;
	__shared__ uint totalNumBodiesConstraints;

	if(threadIdx.x == 0)
	{
		bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;
		motionVelocities = constraintPrepDesc->motionVelocityArray;
		totalNumBodies = constraintPrepDesc->numSolverBodies*2; // *2 because of linear and angular components
		totalNumBodiesConstraints = (constraintPrepDesc->numArticBatches + constraintPrepDesc->numBatches) * 32 * 3 * 2
			+ constraintPrepDesc->numSolverBodies * 3;
	}
	__syncthreads();

	const uint blockStride = (blockDim.x * gridDim.x);
	const uint threadIndex = (threadIdx.x + blockIdx.x * blockDim.x);

	const float4 zero = make_float4(0.f);
	//(1) Set all motion velocities to zero
	for(uint a = threadIndex; a < totalNumBodies; a+=blockStride)
	{
		motionVelocities[a]					= zero;
	}
	
	//(2) Set all velocities to zero. Strictly, we only need to set the first instance of each body
	// in the solver to 0 but, for now, we'll just initialize them all to zero...
	for(uint a = threadIndex; a < totalNumBodiesConstraints; a+= blockStride)
	{
		bodyVelocities[a]					= zero;
	}
}

extern "C" __global__ void writebackBlocksTGS(
	PxgSolverCoreDesc* constraintPrepDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc, const PxU32 islandIndex)
{	
	PxgIslandContext& island = constraintPrepDesc->islandContextPool[islandIndex];

	const PxU32 startIndex = island.mBatchStartIndex;
	const PxU32 endIndex = island.mBatchCount + island.mArtiBatchCount + island.mStaticArtiBatchCount + island.mSelfArtiBatchCount 
		+ island.mStaticRigidBatchCount + startIndex;

	const uint warpSize = 32;

	//const uint blockStride = (blockDim.x * gridDim.x)/warpSize;

	//This identifies which warp a specific thread is in
	const uint warpIndex = (threadIdx.x + blockIdx.x * blockDim.x)/warpSize;

	const uint warpIndexInBlock = threadIdx.x/warpSize;

	//This identifies which thread within a warp a specific thread is
	const uint threadIndexInWarp = threadIdx.x&(warpSize-1);

	PxgBlockConstraintBatch* batchHeaders = sharedDesc->iterativeData.blockConstraintBatch;

	PxgTGSBlockSolverConstraint1DHeader* jointHeaders = sharedDesc->iterativeData.blockJointConstraintHeaders;
	PxgTGSBlockSolverConstraint1DCon* jointRowsCon = sharedDesc->iterativeData.blockJointConstraintRowsCon;
	//PxgBlockSolverConstraint1DMod* jointRowsMod = sharedDesc->iterativeData.blockJointConstraintRowsMod;

	PxgConstraintWriteback* constraintWriteBack = constraintPrepDesc->constraintWriteBack;

	PxgSolverBodyData* solverBodyDatas = constraintPrepDesc->solverBodyDataPool;

	PxgTGSBlockSolverContactHeader* contactHeaders = sharedDesc->iterativeData.blockContactHeaders;
	PxgTGSBlockSolverFrictionHeader* frictionHeaders = sharedDesc->iterativeData.blockFrictionHeaders;
	PxgTGSBlockSolverContactPoint* contactPoints = sharedDesc->iterativeData.blockContactPoints;
	PxgTGSBlockSolverContactFriction* frictions = sharedDesc->iterativeData.blockFrictions;

	PxgBlockFrictionPatch* baseFrictionPatches = sharedDesc->blockCurrentFrictionPatches;
	PxF32* baseWritebackForceBuffer = constraintPrepDesc->forceBuffer;

	PxgFrictionPatchGPU* frictionPatches = reinterpret_cast<PxgFrictionPatchGPU*>(constraintPrepDesc->frictionPatches);

	__shared__ PxU8 blob[PxgKernelBlockDim::WRITEBACK_BLOCKS*sizeof(Dy::ThresholdStreamElement)];

	__shared__  PxI32 index[PxgKernelBlockDim::WRITEBACK_BLOCKS/warpSize];

	Dy::ThresholdStreamElement* elems = reinterpret_cast<Dy::ThresholdStreamElement*>(&blob[0]);

	Dy::ThresholdStreamElement* startAddress = &elems[32*warpIndexInBlock];

	//for(uint k = startIndex + warpIndex; k < endIndex; k+=blockStride)
	uint k = startIndex + warpIndex;
	if(k < endIndex)
	{
		if(threadIndexInWarp == 0)
			index[warpIndexInBlock] = 0;

		__syncwarp();

		//Get block header
		const PxgBlockConstraintBatch& batch = batchHeaders[k];

		if(threadIndexInWarp < batch.mDescStride)
		{
			if(batch.constraintType==PxgSolverConstraintDesc::eCONTACT || batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONTACT)
			{
				writeBackContactBlockTGS(batch, threadIndexInWarp, solverBodyDatas, startAddress, &index[warpIndexInBlock], contactHeaders, frictionHeaders, contactPoints, frictions,
					baseWritebackForceBuffer, baseFrictionPatches[batch.mConstraintBatchIndex], frictionPatches);
			}
			else
			{
				//Do nothing (for now)
				writeBack1DBlockTGS(batch, threadIndexInWarp, jointHeaders, jointRowsCon, constraintWriteBack);
			}
		}

		//__syncthreads();

		__syncwarp();

		PxI32 ind = index[warpIndexInBlock];

		if(ind > 0)
		{
			__shared__ PxI32 startIndex[PxgKernelBlockDim::WRITEBACK_BLOCKS/warpSize];
			if(threadIndexInWarp == 0)
			{
				startIndex[warpIndexInBlock] = atomicAdd(&constraintPrepDesc->sharedThresholdStreamIndex, ind);
			}

			__syncwarp();

			if(threadIndexInWarp < ind)
			{
				//((float4*)constraintPrepDesc->thresholdStream)[startIndex[warpIndexInBlock]+threadIndexInWarp] = ((float4*)startAddress)[threadIndexInWarp];
				constraintPrepDesc->thresholdStream[startIndex[warpIndexInBlock]+threadIndexInWarp] = startAddress[threadIndexInWarp];
			}
		}
	}
}

extern "C" __global__ void concludeBlocksTGS(
	const PxgSolverCoreDesc* constraintPrepDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc, const PxU32 islandIndex)
{	
	PxgIslandContext& island = constraintPrepDesc->islandContextPool[islandIndex];

	const PxU32 startIndex = island.mBatchStartIndex;
	//const PxU32 endIndex = island.mBatchCount + startIndex;
	const PxU32 endIndex = island.mBatchCount + island.mArtiBatchCount + 
		island.mStaticArtiBatchCount + island.mSelfArtiBatchCount + startIndex + island.mStaticRigidBatchCount;

	const uint warpSize = 32;

	//const uint blockStride = (blockDim.x * gridDim.x)/warpSize;

	//This identifies which warp a specific thread is in
	const uint warpIndex = (threadIdx.x + blockIdx.x * blockDim.x)/warpSize;

	//This identifies which thread within a warp a specific thread is
	const uint threadIndexInWarp = threadIdx.x&(warpSize-1);

	PxgBlockConstraintBatch* batchHeaders = sharedDesc->iterativeData.blockConstraintBatch;

	PxgTGSBlockSolverConstraint1DHeader* jointHeaders = sharedDesc->iterativeData.blockJointConstraintHeaders;
	PxgTGSBlockSolverConstraint1DCon* jointRows = sharedDesc->iterativeData.blockJointConstraintRowsCon;

	PxgTGSBlockSolverContactHeader* contactHeaders = sharedDesc->iterativeData.blockContactHeaders;
	PxgTGSBlockSolverFrictionHeader* frictionHeaders = sharedDesc->iterativeData.blockFrictionHeaders;
	PxgTGSBlockSolverContactPoint* contactPoints = sharedDesc->iterativeData.blockContactPoints;
	PxgTGSBlockSolverContactFriction* frictions = sharedDesc->iterativeData.blockFrictions;

	//for(uint k = startIndex + warpIndex; k < endIndex; k+=blockStride)
	uint k = startIndex + warpIndex; 
	if(k < endIndex)
	{
		//Get block header
		const PxgBlockConstraintBatch& batch = batchHeaders[k];

		if(threadIndexInWarp < batch.mDescStride)
		{
			if(batch.constraintType==PxgSolverConstraintDesc::eCONTACT || batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONTACT)
			{
				concludeContactBlockTGS(batch, threadIndexInWarp, contactHeaders, frictionHeaders, contactPoints, frictions);
			}
			else
			{
				conclude1DBlockTGS(batch, threadIndexInWarp, jointHeaders, jointRows);
			}
		}
	}
}

extern "C" __global__ void writeBackBodiesTGS(
	const PxgSolverCoreDesc* solverCoreDesc ,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc, const PxU32 islandIndex)
{	
	__shared__ float4* bodyVelocities;
	__shared__ float4* motionVelocities;
	__shared__ PxU32 bodyOffset;
	__shared__ PxU32 numDynamicBodies;

	if(threadIdx.x == 0)
	{
		PxgIslandContext& island = solverCoreDesc->islandContextPool[islandIndex];
		bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;
		motionVelocities = solverCoreDesc->motionVelocityArray;
		bodyOffset = island.mBodyStartIndex;
		//outSolverVels = constraintPrepDesc->outSolverVelocity;
		numDynamicBodies = island.mBodyCount;

	}
	__syncthreads();

	const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;
	
	const PxU32 outputOffset = solverCoreDesc->accumulatedBodyDeltaVOffset;

	const PxReal invDt = sharedDesc->invDtF32;
	//const uint blockStride = (blockDim.x * gridDim.x);

	//This identifies which warp a specific thread is in
	const uint threadIndex = (threadIdx.x + blockIdx.x * blockDim.x);

	//for(uint a = threadIndex+bodyStartIndex; a < bodyEndIndex; a+=blockStride)
	uint a = threadIndex;// +bodyStartIndex;
	if(a < numDynamicBodies)
	{
		const float4 vel1 = bodyVelocities[bodyOffset + outputOffset + a + totalBodiesIncKinematics];
		const float4 vel2 = bodyVelocities[bodyOffset + outputOffset + a + totalBodiesIncKinematics + totalBodiesIncKinematics];

		motionVelocities[a + bodyOffset] = make_float4(vel1.z, vel1.w, vel2.x, 0.f)*invDt;
		motionVelocities[a + bodyOffset + totalBodiesIncKinematics] = make_float4(vel2.y, vel2.z, vel2.w, 0.f)*invDt;
	}
}

inline __device__ float dot4(float4 x, float4 y)
{
	return x.x * y.x + x.y*y.y + x.z*y.z + x.w*y.w;
}

extern "C" __global__ void computeAverageSolverBodyVelocityTGS(
	const PxgSolverCoreDesc* solverDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc)
{
	PxgSolverReferences* solverReferences = solverDesc->solverBodyReferences;
	float4* bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;

	PxU32* encodedReferenceCount = sharedDesc->iterativeData.solverEncodedReferenceCount;
	
	const PxU32 numDynamicBodies = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!
	
	const PxU32 numSlabs = solverDesc->numSlabs;
	const PxU32 numBatches = solverDesc->numBatches;
	const PxU32 numArticBatches = solverDesc->numArticBatches;
	const PxU32 bodyOffset = solverDesc->islandContextPool->mBodyStartIndex;

	const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;

	const PxU32 averageOutputOffsets = (numArticBatches + numBatches) * PXG_BATCH_SIZE * 3 * 2;

	//This identifies which warp a specific thread is in
	const uint warpIndex = (blockIdx.x * blockDim.y + threadIdx.y);

	const PxU32 numThreadsPerBody = PxMin(isPowerOfTwo(numSlabs) ? numSlabs : nextPowerOfTwo(numSlabs), 32u);
	const PxU32 numBodiesPerWarp = WARP_SIZE / numThreadsPerBody;

	const uint warpBodyStartIndex = warpIndex * numBodiesPerWarp;

	const PxU32 threadIdInWorkUnit = threadIdx.x&(numThreadsPerBody - 1);
	const PxU32 subWarpIndex = threadIdx.x / numThreadsPerBody;

	const PxU32 bodyInWarp = threadIdx.x/numThreadsPerBody;

	const uint bodyId = warpBodyStartIndex + bodyInWarp;

	const PxU32 outputOffset = solverDesc->accumulatedBodyDeltaVOffset;

	const PxU32 maskHigh = ((1 << ((subWarpIndex+1)*numThreadsPerBody)) - 1);
	const PxU32 maskLow = ((1 << ((subWarpIndex)*numThreadsPerBody)) - 1);
	const PxU32 mask = maskHigh - maskLow;

	if (bodyId < numDynamicBodies)
	{
		const float4 vel0 = bodyVelocities[bodyOffset + outputOffset + bodyId];
		const float4 vel1 = bodyVelocities[bodyOffset + outputOffset + bodyId + totalBodiesIncKinematics];

		float4 curVel0 = make_float4(0.f);
		float4 curVel1 = make_float4(0.f);

		bool hasRef = false;
		for (uint b = threadIdInWorkUnit; b < numSlabs; b += numThreadsPerBody)
		{
			PxU32 bodyIndex = bodyId * numSlabs + b;
			PxgSolverReferences reference = solverReferences[bodyIndex];

			PxU32 remappedBodyIndex = reference.mRemappedBodyIndex;

			if (remappedBodyIndex != 0xFFFFFFFF)
			{
				const uint velIndex = averageOutputOffsets + ComputeAverageBodyOutputIndex(bodyIndex);

				float4 delta0 = bodyVelocities[velIndex] - vel0;
				float4 delta1 = bodyVelocities[velIndex + 32];
				delta1.x -= vel1.x;
				delta1.y -= vel1.y;

				curVel0 += delta0;
				curVel1.x += delta1.x;
				curVel1.y += delta1.y;

				hasRef = true;
			}
		}

		//Now do the reduction...
#pragma unroll
		for (PxU32 reductionRadius = numThreadsPerBody >> 1; reductionRadius > 0; reductionRadius >>= 1)
		{

			curVel0.x += __shfl_xor_sync(mask, curVel0.x, reductionRadius, numThreadsPerBody);
			curVel0.y += __shfl_xor_sync(mask, curVel0.y, reductionRadius, numThreadsPerBody);
			curVel0.z += __shfl_xor_sync(mask, curVel0.z, reductionRadius, numThreadsPerBody);
			curVel0.w += __shfl_xor_sync(mask, curVel0.w, reductionRadius, numThreadsPerBody);

			curVel1.x += __shfl_xor_sync(mask, curVel1.x, reductionRadius, numThreadsPerBody);
			curVel1.y += __shfl_xor_sync(mask, curVel1.y, reductionRadius, numThreadsPerBody);
		}

		hasRef = (__ballot_sync(mask, hasRef) & mask);

		if (hasRef)
		{
			if (threadIdInWorkUnit == 0)
			{
				const PxU32 numTotalBodies =
				    bodyOffset + numDynamicBodies + solverDesc->islandContextPool->mArticulationCount;
				const PxU32 referenceCount = countActiveSlabs(bodyOffset + bodyId, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount);
				const PxReal recipRefs = 1.0f / static_cast<PxReal>(referenceCount);

				curVel0 = curVel0 * recipRefs;
				curVel1.x = curVel1.x * recipRefs;
				curVel1.y = curVel1.y * recipRefs;
				
				// Resetting rigid body reference count
				resetSlabCount(bodyOffset + bodyId, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount);

				curVel0 += vel0;
				curVel1 += vel1;

				//KS - we only need to write out to here if we had any references, otherwise we already
				//had these variables set in initializeSolverVelocitiesTGS 
				bodyVelocities[bodyOffset + outputOffset + bodyId] = curVel0;
				bodyVelocities[bodyOffset + outputOffset + bodyId + totalBodiesIncKinematics] = curVel1;
			}
		}
	}
}
	
extern "C" __global__ void propagateAverageSolverBodyVelocityTGS(
	const PxgSolverCoreDesc* solverDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc, bool isVelocityIteration, bool isLastPosIteration,
	PxReal biasCoefficient)
{
	//Buffer to store read data in. We then add on first 6 words to 2nd 6 words

	PxReal stepDt = isVelocityIteration ? 0.f : sharedDesc->stepDt;
	const uint MaxBodiesPerWarp = 32;

	PX_COMPILE_TIME_ASSERT(sizeof(PxgSolverTxIData) % sizeof(PxReal) == 0);

	__shared__ PxReal sIData[(sizeof(PxgSolverTxIData) / sizeof(PxReal)) * (PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY / WARP_SIZE) * MaxBodiesPerWarp];
	
	//__shared__ PxgSolverTxIData iData[PxgKernelBlockDim::COMPUTE_BODIES_AVERAGE_VELOCITY / 32][MaxBodiesPerWarp];
	PxgSolverTxIData* iData = reinterpret_cast<PxgSolverTxIData*>(sIData);
	//we need to fill in the writeIndex for the solver body data
	PxgSolverReferences* solverReferences = solverDesc->solverBodyReferences;
	float4* bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;
	const PxU32 numDynamicBodies = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!
	const PxU32 bodyOffset = solverDesc->islandContextPool->mBodyStartIndex;
	const PxU32 numSlabs = solverDesc->numSlabs;

	const PxU32 numThreadsPerBody = PxMin(isPowerOfTwo(numSlabs) ? numSlabs : nextPowerOfTwo(numSlabs), 32u);
	const PxU32 numBodiesPerWarp = WARP_SIZE / numThreadsPerBody;

	const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;

	PxgSolverTxIData* gIData = solverDesc->solverBodyTxIDataPool;

	//This identifies which warp a specific thread is in
	const uint warpIndex = (blockIdx.x * blockDim.y + threadIdx.y);

	//const uint threadIndex = threadIdx.x + warpIndex*blockDim.x;
	//const uint bodyId = threadIdx.x / 2 + warpIndex * 16;

	//We now do 3 threads per-pair, with threads 30 and 31 doing nothing...

	const uint warpBodyStartIndex = warpIndex * numBodiesPerWarp;

	const PxU32 bodyInWarp = threadIdx.x/numThreadsPerBody;

	const uint bodyId = warpBodyStartIndex + bodyInWarp;

	const uint bodyEndIndex = PxMin(totalBodiesIncKinematics, warpBodyStartIndex + numBodiesPerWarp);

	const PxU32 outputOffset = solverDesc->accumulatedBodyDeltaVOffset;

#if 1
	//(1) Read in shared data from txInertia!!!!
	int offsetIndex = threadIdx.x / 16;
	if (!isVelocityIteration)
	{
		for (uint i = warpBodyStartIndex + offsetIndex; i < bodyEndIndex; i += 32 / 16)
		{
			//volatile float* warpShData = reinterpret_cast<volatile float*>(&iData[threadIdx.y][i - warpBodyStartIndex]);
			const PxU32 ind = threadIdx.y * MaxBodiesPerWarp + (i - warpBodyStartIndex);

			volatile float* warpShData = reinterpret_cast<volatile float*>(&iData[ind]);

			float* warpData = reinterpret_cast<float*>(&gIData[i]);

			float tmp = warpData[threadIdx.x & 15];
			
			assert(PxIsFinite(tmp));

			warpShData[threadIdx.x & 15] = tmp;
		}
	}

	__syncwarp();
#endif

	if (bodyId < totalBodiesIncKinematics)
	{
		float4 curVel0 = bodyVelocities[outputOffset + bodyId];
		float4 curVel1 = bodyVelocities[outputOffset + bodyId + totalBodiesIncKinematics];
		float4 curVel2 = bodyVelocities[outputOffset + bodyId + totalBodiesIncKinematics + totalBodiesIncKinematics];

		/*printf("%i: BodyId = %i, index = %i, curVel0 = (%f, %f, %f)\n", 
			threadIdx.x, bodyId, outputOffset + bodyId, curVel0.x, curVel0.y, curVel0.z);*/

		//printf("bodyVelocities[%i] (%p) outputOffset = %i (%f, %f, %f, %f)(%f, %f, %f, %f)(%f, %f, %f, %f)\n", bodyId, 
		//	&bodyVelocities[outputOffset + bodyId], outputOffset, curVel0.x, curVel0.y, curVel0.z, curVel0.w,
		//	curVel1.x, curVel1.y, curVel1.z, curVel1.w, curVel2.x, curVel2.y, curVel2.z, curVel2.w);
#if 1
		//Integrate forwards rotation!!!! Quaternion is stored in by thread 0 in the group...

		const PxU32 threadIdInWorkUnit = threadIdx.x&(numThreadsPerBody - 1);

		PxU32 totalRefs = 0;
		//if (bodyId >= bodyOffset)
		{
			if (!isVelocityIteration)
			{
				//TODO - Bad memory access pattern below. Need to revisit!

				PxU32 lockFlags = 0;

				const PxgSolverBodyData& data = solverDesc->solverBodyDataPool[bodyId];
				const PxU32 nodeIndex = data.islandNodeIndex.index();// >> 2;
				
				PxVec3 angularMotionVel;

				PxVec3 linVel(curVel0.x, curVel0.y, curVel0.z);
				PxVec3 angVel(curVel0.w, curVel1.x, curVel1.y);

				if (bodyId > 0)
				{
					PxgBodySim&	bodySim = solverDesc->mBodySimBufferDeviceData[nodeIndex];

					lockFlags = bodySim.lockFlags;

					const PxU32 ind = threadIdx.y * MaxBodiesPerWarp + bodyInWarp;
					PxgSolverTxIData& txInertia = iData[ind];
					if(bodyId >= bodyOffset)
						angularMotionVel = txInertia.sqrtInvInertia * angVel;
					else
						angularMotionVel = angVel;

					if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X)
						angularMotionVel.x = 0.f;
					if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y)
						angularMotionVel.y = 0.f;
					if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)
						angularMotionVel.z = 0.f;

					if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X)
						linVel.x = 0.f;
					if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y)
						linVel.y = 0.f;
					if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z)
						linVel.z = 0.f;

					PxVec3 deltaPThisStep = linVel * stepDt;

					curVel1.z += deltaPThisStep.x; curVel1.w += deltaPThisStep.y; curVel2.x += deltaPThisStep.z;
					curVel2.y += angVel.x * stepDt; curVel2.z += angVel.y * stepDt; curVel2.w += angVel.z * stepDt;

					if (threadIdInWorkUnit == 0)
					{
						PxReal w2 = angularMotionVel.magnitudeSquared();

						assert(txInertia.deltaBody2World.p.isFinite());

						// Integrate the rotation using closed form quaternion integrator
						if (w2 != 0.0f)
						{
							PxReal w = PxSqrt(w2);

							const PxReal v = w * 0.5f*stepDt;
							PxReal s, q;
							__sincosf(v, &s, &q);
							s /= w;

							const PxVec3 pqr = angularMotionVel * s;
							const PxQuat quatVel(pqr.x, pqr.y, pqr.z, 0);
							PxQuat result = quatVel * txInertia.deltaBody2World.q;

							result += txInertia.deltaBody2World.q * q;

							txInertia.deltaBody2World.q = result.getNormalized();
						}

						txInertia.deltaBody2World.p += deltaPThisStep;
					}

					/*printf("%i: step = %f, bodywWorldD(%f, %f, %f)(%f, %f, %f, %f)\n", bodyId, stepDt, txInertia.deltaBody2World.p.x, txInertia.deltaBody2World.p.y, txInertia.deltaBody2World.p.z,
						txInertia.deltaBody2World.q.x, txInertia.deltaBody2World.q.y, txInertia.deltaBody2World.q.z, txInertia.deltaBody2World.q.w);*/

					//if (isLastPosIteration && bodyId >= bodyOffset)
					//{
					//	PxReal invDt = sharedDesc->invDtF32;
					//	//Averaging...
					//	PxVec3 deltaLinVel = PxVec3(curVel1.z, curVel1.w, curVel2.x) * invDt;
					//	PxVec3 deltaAngVel = PxVec3(curVel2.y, curVel2.z, curVel2.w) * invDt;

					//	if (deltaLinVel.magnitudeSquared() < linVel.magnitudeSquared() || deltaAngVel.magnitudeSquared() < angVel.magnitudeSquared())
					//	{
					//		PxReal ratio = PxMin(0.5f, biasCoefficient);
					//		PxReal otherRatio = 1.f - ratio;

					//		curVel0.x = curVel0.x * ratio + deltaLinVel.x * otherRatio;
					//		curVel0.y = curVel0.y * ratio + deltaLinVel.y * otherRatio;
					//		curVel0.z = curVel0.z * ratio + deltaLinVel.z * otherRatio;

					//		curVel0.w = curVel0.w * ratio + deltaAngVel.x * otherRatio;
					//		curVel1.x = curVel1.x * ratio + deltaAngVel.y * otherRatio;
					//		curVel1.y = curVel1.y * ratio + deltaAngVel.z * otherRatio;
					//	}
					//}
				}
			}

			__syncwarp();

			if (bodyId >= bodyOffset)
			{
				//copy the final velocity to the corresponding body velocity slot
				for (uint b = threadIdInWorkUnit; b < numSlabs; b+= numThreadsPerBody)
				{
					//uint startIndex = b * numDynamicBodies;
					//const uint bodyIndex = startIndex + bodyId - bodyOffset;
					const uint bodyIndex = (bodyId - bodyOffset)*numSlabs + b;
					PxgSolverReferences reference = solverReferences[bodyIndex];
					PxU32 remappedBodyIndex = reference.mRemappedBodyIndex;
					if (remappedBodyIndex != 0xFFFFFFFF)
					{
						bodyVelocities[remappedBodyIndex] = curVel0;
						bodyVelocities[remappedBodyIndex + 32] = curVel1;
						bodyVelocities[remappedBodyIndex + 64] = curVel2;
						totalRefs++;
					}
				}
			}
		}

		if (!isVelocityIteration)
		{
			//if (totalRefs == 0)
			if(threadIdInWorkUnit == 0)
			{
				//If no references, update the deltaP terms to update the positions/velocities. We technically
				//don't need this unless we have soft bodies or particles in the scene
				
				bodyVelocities[outputOffset + bodyId + totalBodiesIncKinematics] = curVel1;
				bodyVelocities[outputOffset + bodyId + totalBodiesIncKinematics + totalBodiesIncKinematics] = curVel2;
			}
		}
#endif
	}

#if 1
	if (!isVelocityIteration)
	{
		__syncwarp(); //Required (racecheck confirmed) because iData (Ptr warpShData points to iData and so does txInertia) is read below and written above

		for (uint i = warpBodyStartIndex + offsetIndex; i < bodyEndIndex; i += 32 / 16)
		{
			//volatile float* warpShData = reinterpret_cast<volatile float*>(&iData[threadIdx.y][i - warpBodyStartIndex]);
			const PxU32 ind = threadIdx.y * MaxBodiesPerWarp + (i - warpBodyStartIndex);
			volatile float* warpShData = reinterpret_cast<volatile float*>(&iData[ind]);
			float* warpData = reinterpret_cast<float*>(&gIData[i]);

			//float4 tmp = make_float4(warpShData[threadIdx.x & 3].x, warpShData[threadIdx.x & 3].y, warpShData[threadIdx.x & 3].z, warpShData[threadIdx.x & 3].w);

			warpData[threadIdx.x & 15] = warpShData[threadIdx.x & 15];
		}
	}
#endif
}

extern "C" __global__ void initializeSolverVelocitiesTGS(
	const PxgSolverCoreDesc* solverDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc)
{
	//Buffer to store read data in. We then add on first 6 words to 2nd 6 words

	//we need to fill in the writeIndex for the solver body data
	PxgSolverReferences* solverReferences = solverDesc->solverBodyReferences;
	float4* bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;
	const PxU32 bodyOffset = solverDesc->islandContextPool->mBodyStartIndex;
	const PxU32 numSlabs = solverDesc->numSlabs;
	const PxU32 allBodiesCount = solverDesc->numSolverBodies;

	const float4* velocities = solverDesc->outSolverVelocity;

	//This identifies which warp a specific thread is in
	const uint warpIndex = (blockIdx.x * blockDim.y + threadIdx.y);

	//const uint threadIndex = threadIdx.x + warpIndex*blockDim.x;
	//const uint bodyId = threadIdx.x / 2 + warpIndex * 16;

	//We now do 3 threads per-pair, with threads 30 and 31 doing nothing...

	const PxU32 numThreadsPerBody = PxMin(isPowerOfTwo(numSlabs) ? numSlabs : nextPowerOfTwo(numSlabs), 32u);
	const PxU32 numBodiesPerWarp = WARP_SIZE / numThreadsPerBody;
	const PxU32 threadIdxInWorkUnit = threadIdx.x&(numThreadsPerBody - 1);

	const uint warpBodyStartIndex = warpIndex * numBodiesPerWarp;

	const PxU32 bodyInWarp = threadIdx.x/numThreadsPerBody;

	const uint bodyId = warpBodyStartIndex + bodyInWarp;

	//const PxU32 outputOffset = (numArticBatches + numBatches) * PXG_BATCH_SIZE * 3 * 2
	//	+ ((numSlabs * activeBodyCount + 31)&(~31))*3;
	const PxU32 outputOffset = solverDesc->accumulatedBodyDeltaVOffset;

	if (bodyId < allBodiesCount)
	{		
		float4 linVel = velocities[bodyId];
		float4 angVel = velocities[bodyId + allBodiesCount];

		//printf("%i: bodyId = %i, linVel = (%f, %f, %f)\n", threadIdx.x, bodyId, linVel.x, linVel.y, linVel.z);

		//KS - temporarily zero these until the contact prep code will tolerate having actual pre-solver velocities in the 
		//velocity buffers
		//float4 linVel = make_float4(0.f);
		//float4 angVel = make_float4(0.f);

		float4 vel0 = make_float4(linVel.x, linVel.y, linVel.z, angVel.x);
		float4 vel1 = make_float4(angVel.y, angVel.z, 0.f, 0.f);
		float4 vel2 = make_float4(0.f);
		//get out velocity for the correponding body in a slab and put on weight to calculate the final velocity for that body
		if (bodyId >= bodyOffset)
		{
			for (uint b = threadIdxInWorkUnit; b < numSlabs; b+= numThreadsPerBody)
			{
				const uint bodyIndex = numSlabs * (bodyId - bodyOffset) + b;
				PxgSolverReferences reference = solverReferences[bodyIndex];

				/*printf("bodyId %i: b = %i: solverReferences[%i] = %i = (%f, %f, %f)\n", bodyId, b, bodyIndex, reference.mRemappedBodyIndex,
					vel0.x, vel0.y, vel0.z);*/

				const PxU32 remappedBodyIndex = reference.mRemappedBodyIndex;

				if (remappedBodyIndex != 0xFFFFFFFF)
				{
					bodyVelocities[remappedBodyIndex] = vel0;
					bodyVelocities[remappedBodyIndex + 32] = vel1;
					bodyVelocities[remappedBodyIndex + 64] = vel2;
				}
			}
		}

		if (threadIdxInWorkUnit == 0)
		{
			bodyVelocities[outputOffset + bodyId] = vel0;
			bodyVelocities[outputOffset + bodyId + allBodiesCount] = vel1;
			bodyVelocities[outputOffset + bodyId + 2 * allBodiesCount] = vel2;
		}
	}
}

__device__ void artiSolveBlockPartitionTGSInternal(
	PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveDataTGS>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 partitionIndex, const PxReal elapsedTime,
	const PxReal minPen, const PxgArticulationCoreDesc* const PX_RESTRICT artiDesc)
{
	const PxU32 warpIndex = threadIdx.y;
	PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;

	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];

	//PxgBodySim* gBodySims = sharedDesc->bodySims;
	PxgArticulationBlockData* gArticulations = artiDesc->mArticulationBlocks;
	PxgArticulationBlockLinkData* gArticulationLinks = artiDesc->mArticulationLinkBlocks;

	const PxU32 maxLinks = artiDesc->mMaxLinksPerArticulation;

	Cm::UnAlignedSpatialVector* deferredZ = sharedDesc->articulationDeferredZ;

	const PxU32 startPartitionIndex = island.mStartPartitionIndex;

	PxU32 startIndex = partitionIndex == 0 ? island.mArtiBatchStartIndex : solverDesc->artiConstraintsPerPartition[partitionIndex + startPartitionIndex - 1];
	//PxU32 startIndex = solverDesc->constraintsPerPartition[partitionIndex + startPartitionIndex];

	const PxU32 articulationBatchOffset = solverDesc->islandContextPool->mBatchCount;

	//const PxU32 nbArticulations = artiDesc->nbArticulations;

	PxU32 endIndex = solverDesc->artiConstraintsPerPartition[partitionIndex + startPartitionIndex] + articulationBatchOffset;

	//const PxU32 articOffset = solverDesc->numBatches * 32 * 2 * 2;

	uint2* isSlabDirty = artiDesc->slabHasChanges;

	//This identifies which thread within a warp a specific thread is
	const uint threadIndexInWarp = threadIdx.x;

	uint k = startIndex + globalWarpIndex + articulationBatchOffset;

	const PxgSolverTxIData* PX_RESTRICT iData = solverDesc->solverBodyTxIDataPool;

	PxgErrorAccumulator error;
	const bool accumulateError = solverDesc->contactErrorAccumulator.mCounter >= 0;

	if (k < endIndex)
	{
		const IterativeSolveDataTGS& msIterativeData = sharedDesc->iterativeData;

		const PxgBlockConstraintBatch& batch = msIterativeData.blockConstraintBatch[k];

		if (threadIndexInWarp >= batch.mDescStride)
			return;

		const PxU32 readIndex = k * 192 + threadIndexInWarp;

		PxgSolverTxIData iData0, iData1;

		PxU32 constraintType = batch.constraintType;

		const PxU32 nbArticulations = solverDesc->islandContextPool->mArticulationCount;

		const PxNodeIndex igNodeIndexA = batch.bodyANodeIndex[threadIndexInWarp];
		const PxNodeIndex igNodeIndexB = batch.bodyBNodeIndex[threadIndexInWarp];

		const PxU32 slabId = batch.slabId[threadIndexInWarp];

		const PxU32 nodeIndexA = igNodeIndexA.index();
		const PxU32 nodeIndexB = igNodeIndexB.index();

		const PxU32 solverBodyId0 = batch.bodyAIndex[threadIndexInWarp];
		const PxU32 solverBodyId1 = batch.bodyBIndex[threadIndexInWarp];

		PxU32 linkIndexA = igNodeIndexA.articulationLinkId();
		PxU32 linkIndexB = igNodeIndexB.articulationLinkId();

		Cm::UnAlignedSpatialVector vel0, vel1;
		Cm::UnAlignedSpatialVector delta0, delta1;
		PxQuat deltaQ0, deltaQ1;

		{
			float4 vec0 = Pxldcg(msIterativeData.solverBodyVelPool[readIndex]);
			float4 vec1 = Pxldcg(msIterativeData.solverBodyVelPool[readIndex + 32]);
			float4 vec2 = Pxldcg(msIterativeData.solverBodyVelPool[readIndex + 64]);

			vel0 = Cm::UnAlignedSpatialVector(PxVec3(vec0.w, vec1.x, vec1.y), PxVec3(vec0.x, vec0.y, vec0.z));
			delta0 = Cm::UnAlignedSpatialVector(PxVec3(vec2.y, vec2.z, vec2.w), PxVec3(vec1.z, vec1.w, vec2.x));
		}
		{
			float4 vec0 = Pxldcg(msIterativeData.solverBodyVelPool[readIndex + 96]);
			float4 vec1 = Pxldcg(msIterativeData.solverBodyVelPool[readIndex + 128]);
			float4 vec2 = Pxldcg(msIterativeData.solverBodyVelPool[readIndex + 160]);

			vel1 = Cm::UnAlignedSpatialVector(PxVec3(vec0.w, vec1.x, vec1.y), PxVec3(vec0.x, vec0.y, vec0.z));
			delta1 = Cm::UnAlignedSpatialVector(PxVec3(vec2.y, vec2.z, vec2.w), PxVec3(vec1.z, vec1.w, vec2.x));
		}

		PxReal curRef0 = 1.f;
		PxReal curRef1 = 1.f;

		const PxU32 bodyOffset = solverDesc->islandContextPool->mBodyStartIndex;
		const PxU32 numDynamicBodies = solverDesc->islandContextPool->mBodyCount; // nbBodies minus offset!

		const PxU32* const PX_RESTRICT encodedReferenceCount = sharedDesc->iterativeData.solverEncodedReferenceCount;
		const PxU32 numTotalBodies = bodyOffset + numDynamicBodies + nbArticulations;

		if(igNodeIndexA.isArticulation())
		{
			const PxU32 articulationBodyIdA = batch.remappedBodyAIndex[threadIndexInWarp];

			// Articulation IDs are at the back of rigid body IDs.
			const PxU32 globalBodyIdA = articulationBodyIdA + numDynamicBodies + bodyOffset;

			// Counting the number of active slabs
			curRef0 = static_cast<PxReal>(countActiveSlabs(globalBodyIdA, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount));
		}
		else if(solverBodyId0 >= bodyOffset)
		{
			// Counting the number of active slabs
			curRef0 = static_cast<PxReal>(countActiveSlabs(solverBodyId0, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount));
		}

		if(igNodeIndexB.isArticulation())
		{
			const PxU32 articulationBodyIdB = batch.remappedBodyBIndex[threadIndexInWarp];

			// Articulation IDs are at the back of rigid body IDs.
			const PxU32 globalBodyIdB = articulationBodyIdB + numDynamicBodies + bodyOffset;

			// Counting the number of active slabs
			curRef1 = static_cast<PxReal>(countActiveSlabs(globalBodyIdB, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount));
		}
		else if(solverBodyId1 >= bodyOffset)
		{
			// Counting the number of active slabs
			curRef1 = static_cast<PxReal>(countActiveSlabs(solverBodyId1, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount));
		}

		if(constraintType == PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D)
		{
			if(igNodeIndexA.isArticulation())
			{
				PxU32 articBlockId = solverBodyId0 / 32;
				const PxU32 linkID = articBlockId * maxLinks + linkIndexA;
				deltaQ0 = loadQuat(gArticulationLinks[linkID].mDeltaQ, solverBodyId0 & 31);
			}
			else
			{
				deltaQ0 = loadQuat(reinterpret_cast<const float4*>(&iData[solverBodyId0].deltaBody2World.q), 0);
			}

			if(igNodeIndexB.isArticulation())
			{
				PxU32 articBlockId = solverBodyId1 / 32;
				const PxU32 linkID = articBlockId * maxLinks + linkIndexB;
				deltaQ1 = loadQuat(gArticulationLinks[linkID].mDeltaQ, solverBodyId1 & 31);
			}
			else
			{
				deltaQ1 = loadQuat(reinterpret_cast<const float4*>(&iData[solverBodyId1].deltaBody2World.q), 0);
			}
		}

		Cm::UnAlignedSpatialVector impulse0(PxVec3(0.0f), PxVec3(0.0f)), impulse1(PxVec3(0.0f), PxVec3(0.0f));

		if (batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONTACT)
		{
			solveExtContactBlockTGS(batch, vel0, vel1, delta0, delta1, threadIndexInWarp,
				msIterativeData.blockContactHeaders, msIterativeData.blockFrictionHeaders, msIterativeData.blockContactPoints,
				msIterativeData.blockFrictions, msIterativeData.artiResponse, elapsedTime, minPen, impulse0, impulse1, 
				accumulateError ? &error : NULL, curRef0, curRef1);
		}
		else
		{
			solveExt1DBlockTGS(batch, vel0, vel1, delta0, delta1, threadIndexInWarp, msIterativeData.blockJointConstraintHeaders,
				msIterativeData.blockJointConstraintRowsCon, msIterativeData.artiResponse, deltaQ0, deltaQ1, elapsedTime, impulse0, impulse1,
				solverDesc->contactErrorAccumulator.mCounter >= 0, curRef0, curRef1);
		}

		//Pull impulse from threads 6-12
		if (!igNodeIndexA.isArticulation())
		{
			const PxU32 outIndex = (batch.remappedBodyAIndex[threadIndexInWarp]);
			{
				msIterativeData.solverBodyVelPool[outIndex] = make_float4(vel0.bottom.x, vel0.bottom.y, vel0.bottom.z, vel0.top.x);
				msIterativeData.solverBodyVelPool[outIndex + 32] = make_float4(vel0.top.y, vel0.top.z, delta0.bottom.x, delta0.bottom.y);
				msIterativeData.solverBodyVelPool[outIndex + 64] = make_float4(delta0.bottom.z, delta0.top.x, delta0.top.y, delta0.top.z);
				//printf("&msIterativeData.solverBodyVelPool[%i].x)[%i] = %f\n", outIndex, threadIndexInWarp%3, vel);
				//printf("Output rigid A to %i\n", outIndex);
			}
		}
		else
		{
			//Write to GPU articulation!
			const PxU32 index = computeDeltaVIndex(nbArticulations, maxLinks, solverBodyId0, linkIndexA, slabId);
			//deferredZ[index] = impulse0;
			//printf("A index %i\n", index);
			storeSpatialVector(deferredZ + index, impulse0);

			/*printf("A expected final LinkID = %i, articId = %i, vel = (%f, %f, %f, %f, %f, %f)\n", linkIndexA, solverBodyId0,
			vel0.top.x, vel0.top.y, vel0.top.z, vel0.bottom.x, vel0.bottom.y, vel0.bottom.z);*/

			//printf("%i: Output ArticA index = %i, articId = %i (%f, %f, %f, %f, %f, %f)\n", threadIndexInWarp, index, solverBodyId0,
			//	impulse0.top.x, impulse0.top.y, impulse0.top.z, impulse0.bottom.x, impulse0.bottom.y, impulse0.bottom.z);

			isSlabDirty[solverBodyId0 + slabId*nbArticulations].x = linkIndexA;
			PxU32 articBlockId = solverBodyId0 / 32;
			gArticulations[articBlockId].mStateDirty[solverBodyId0 & 31] = PxgArtiStateDirtyFlag::eHAS_IMPULSES | PxgArtiStateDirtyFlag::eVEL_DIRTY;

		}
		if (!igNodeIndexB.isArticulation())
		{
			//const PxU32 indexB = (2 * batch.remappedBodyBIndex);
			const PxU32 outIndex = (batch.remappedBodyBIndex[threadIndexInWarp]);
			{
				msIterativeData.solverBodyVelPool[outIndex] = make_float4(vel1.bottom.x, vel1.bottom.y, vel1.bottom.z, vel1.top.x);
				msIterativeData.solverBodyVelPool[outIndex + 32] = make_float4(vel1.top.y, vel1.top.z, delta1.bottom.x, delta1.bottom.y);
				msIterativeData.solverBodyVelPool[outIndex + 64] = make_float4(delta1.bottom.z, delta1.top.x, delta1.top.y, delta1.top.z);

				//printf("Output rigid B to %i\n", outIndex);
			}
		}
		else
		{
			//Write to GPU articulation!
			const PxU32 index = computeDeltaVIndex(nbArticulations, maxLinks, solverBodyId1, linkIndexB, slabId);
			//printf("B index %i\n", index);

			storeSpatialVector(deferredZ + index, impulse1);
			isSlabDirty[solverBodyId1 + slabId*nbArticulations].y = linkIndexB;
			PxU32 articBlockId = solverBodyId1 / 32;
			gArticulations[articBlockId].mStateDirty[solverBodyId1 & 31] = PxgArtiStateDirtyFlag::eHAS_IMPULSES | PxgArtiStateDirtyFlag::eVEL_DIRTY;

			/*printf("A expected final LinkID = %i, articId = %i, vel = (%f, %f, %f, %f, %f, %f)\n", linkIndexB, solverBodyId0,
			vel1.top.x, vel1.top.y, vel1.top.z, vel1.bottom.x, vel1.bottom.y, vel1.bottom.z);*/
			//printf("%i: Output ArticB index = %i, articId = %i (%f, %f, %f, %f, %f, %f)\n", threadIndexInWarp, index, solverBodyId1,
			//	impulse1.top.x, impulse1.top.y, impulse1.top.z, impulse1.bottom.x, impulse1.bottom.y, impulse1.bottom.z);
		}
	}

	if (accumulateError)
		error.accumulateErrorGlobalFullWarp(solverDesc->contactErrorAccumulator, threadIndexInWarp);
}

__device__ void solveBlockPartitionTGSInternal(
	PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveDataTGS>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 partitionIndex, const PxReal elapsedTime,
	const PxReal minPen, const PxU32 warpIndex, const PxU32 threadIndexInWarp)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];

	const PxU32 startPartitionIndex = island.mStartPartitionIndex;

	PxU32 startIndex = partitionIndex == 0 ? island.mBatchStartIndex : solverDesc->constraintsPerPartition[partitionIndex + startPartitionIndex - 1];

	PxU32 endIndex = solverDesc->constraintsPerPartition[partitionIndex + startPartitionIndex];

	//__shared__ IterativeSolveDataTGS iterativeData;

	//PxU32 idx = threadIdx.x;

	////if(threadIdx.x < sizeof(iterativeData)/sizeof(float))
	//while (idx < (sizeof(iterativeData) / sizeof(float)))
	//{
	//	float* iterData = reinterpret_cast<float*>(&iterativeData);

	//	iterData[idx] = reinterpret_cast<const float*>(&sharedDesc->iterativeData)[idx];
	//	idx += warpSize;
	//}

	//__syncthreads();

	bool residualAccumulationEnabled = solverDesc->contactErrorAccumulator.mCounter >= 0;
	PxgErrorAccumulator error;

	const IterativeSolveDataTGS& iterativeData = sharedDesc->iterativeData;

	const PxgSolverTxIData* PX_RESTRICT iData = solverDesc->solverBodyTxIDataPool;

	//for(uint k = startIndex + warpIndex; k < endIndex; k+=blockStride)
	uint k = startIndex + warpIndex;
	//unsigned mask_k = __ballot_sync(FULL_MASK, k < endIndex);
	if (k < endIndex)
	{
		const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[k];

		const PxU32 readIndex = k * 192 + threadIndexInWarp;

		PxgSolverTxIData iData0, iData1;

		//unsigned mask_eCONSTRAINT_1D = __ballot_sync(mask_k, batch.constraintType == PxgSolverConstraintDesc::eCONSTRAINT_1D);
		if (batch.constraintType == PxgSolverConstraintDesc::eCONSTRAINT_1D)
		{
			//printf("BodyAIdx = %i, bodyBIdx = %i\n", batch.bodyAIndex[threadIndexInWarp], batch.bodyBIndex[threadIndexInWarp]);
			loadTxInertia(FULL_MASK, iData, batch.bodyAIndex[threadIndexInWarp], batch.mDescStride, threadIndexInWarp, iData0);
			loadTxInertia(FULL_MASK, iData, batch.bodyBIndex[threadIndexInWarp], batch.mDescStride, threadIndexInWarp, iData1);
		}

		//Pull out shared memory into float4 format in registers to solve constraints
		if (threadIndexInWarp < batch.mDescStride)
		{
			float4 vec0 = Pxldcs(iterativeData.solverBodyVelPool[readIndex]);
			float4 vec1 = Pxldcs(iterativeData.solverBodyVelPool[readIndex + 32]);
			float4 vec2 = Pxldcs(iterativeData.solverBodyVelPool[readIndex + 64]);

			float4 vec3 = Pxldcs(iterativeData.solverBodyVelPool[readIndex + 96]);
			float4 vec4 = Pxldcs(iterativeData.solverBodyVelPool[readIndex + 128]);
			float4 vec5 = Pxldcs(iterativeData.solverBodyVelPool[readIndex + 160]);

			PxVec3 linVel0(vec0.x, vec0.y, vec0.z);
			PxVec3 angVel0(vec0.w, vec1.x, vec1.y);
			PxVec3 linDelta0(vec1.z, vec1.w, vec2.x);
			PxVec3 angDelta0(vec2.y, vec2.z, vec2.w);

			PxVec3 linVel1(vec3.x, vec3.y, vec3.z);
			PxVec3 angVel1(vec3.w, vec4.x, vec4.y);
			PxVec3 linDelta1(vec4.z, vec4.w, vec5.x);
			PxVec3 angDelta1(vec5.y, vec5.z, vec5.w);

			// Reference counts to be used for the current sub-timestep or iteration.
			PxReal curRef0 = 1.f;
			PxReal curRef1 = 1.f;

			const PxU32 bodyOffset = solverDesc->islandContextPool->mBodyStartIndex;
			
			const PxU32 numDynamicBodies = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!
			const PxU32 numArticulations = solverDesc->islandContextPool->mArticulationCount;
			const PxU32 numTotalBodies = bodyOffset + numDynamicBodies + numArticulations;

			const PxU32* const PX_RESTRICT encodedReferenceCount = sharedDesc->iterativeData.solverEncodedReferenceCount;

			if(batch.bodyAIndex[threadIndexInWarp] >= bodyOffset)
			{
				// Counting the number of active slabs
				curRef0 = static_cast<PxReal>(
					countActiveSlabs(batch.bodyAIndex[threadIndexInWarp], solverDesc->numSlabs, numTotalBodies, encodedReferenceCount));
			}

			if(batch.bodyBIndex[threadIndexInWarp] >= bodyOffset)
			{
				// Counting the number of active slabs
				curRef1 = static_cast<PxReal>(
					countActiveSlabs(batch.bodyBIndex[threadIndexInWarp], solverDesc->numSlabs, numTotalBodies, encodedReferenceCount));
			}

			const PxVec3 preLinVel0 = linVel0;
			const PxVec3 preAngVel0 = angVel0;

			const PxVec3 preLinVel1 = linVel1;
			const PxVec3 preAngVel1 = angVel1;

			if (batch.constraintType == PxgSolverConstraintDesc::eCONTACT)
				solveContactBlockTGS(batch, linVel0, angVel0, linVel1, angVel1, linDelta0, angDelta0, linDelta1, angDelta1, threadIndexInWarp,
					iterativeData.blockContactHeaders, iterativeData.blockFrictionHeaders, iterativeData.blockContactPoints,
					iterativeData.blockFrictions, elapsedTime, minPen, residualAccumulationEnabled ? &error : NULL, curRef0, curRef1);
			else
				solve1DBlockTGS(batch, linVel0, angVel0, linVel1, angVel1, linDelta0, angDelta0, linDelta1, angDelta1, threadIndexInWarp, iterativeData.blockJointConstraintHeaders, iterativeData.blockJointConstraintRowsCon,
					iData0, iData1, elapsedTime, solverDesc->contactErrorAccumulator.mCounter >= 0, curRef0, curRef1);

			vec0 = make_float4(linVel0.x, linVel0.y, linVel0.z, angVel0.x);
			vec1 = make_float4(angVel0.y, angVel0.z, linDelta0.x, linDelta0.y);
			vec2 = make_float4(linDelta0.z, angDelta0.x, angDelta0.y, angDelta0.z);

			vec3 = make_float4(linVel1.x, linVel1.y, linVel1.z, angVel1.x);
			vec4 = make_float4(angVel1.y, angVel1.z, linDelta1.x, linDelta1.y);
			vec5 = make_float4(linDelta1.z, angDelta1.x, angDelta1.y, angDelta1.z);

			const PxU32 outputA = batch.remappedBodyAIndex[threadIndexInWarp];
			const PxU32 outputB = batch.remappedBodyBIndex[threadIndexInWarp];

			Pxstcs(&iterativeData.solverBodyVelPool[outputA], vec0);
			Pxstcs(&iterativeData.solverBodyVelPool[outputA + 32], vec1);
			Pxstcs(&iterativeData.solverBodyVelPool[outputA + 64], vec2);

			Pxstcs(&iterativeData.solverBodyVelPool[outputB], vec3);
			Pxstcs(&iterativeData.solverBodyVelPool[outputB + 32], vec4);
			Pxstcs(&iterativeData.solverBodyVelPool[outputB + 64], vec5);
		}
	}

	if (residualAccumulationEnabled)
	{
		error.accumulateErrorGlobalFullWarp(solverDesc->contactErrorAccumulator, threadIndexInWarp);
	}
}

// Marking active slabs loosely following "solveBlockPartitionTGSInternal"
static __device__ void markActiveSlab_rigidBodyTGS(const PxgSolverCoreDesc* PX_RESTRICT solverDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* PX_RESTRICT sharedDesc,
	PxU32 islandIndex, PxU32 lastPartition, PxReal minPen, PxReal elapsedTime)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];
	const PxU32 startPartitionIndex = island.mStartPartitionIndex;

	const PxU32 startIndex = island.mBatchStartIndex;
	const PxU32 endIndex = solverDesc->constraintsPerPartition[lastPartition + startPartitionIndex];

	const IterativeSolveDataTGS& iterativeData = sharedDesc->iterativeData;

	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;
	const PxU32 threadIndexInWarp = threadIdx.x;
	const PxU32 batchIndex = startIndex + globalWarpIndex;

	if (batchIndex < endIndex)
	{
		const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[batchIndex];

		if (threadIndexInWarp < batch.mDescStride)
		{
			const PxU32 bodyOffset = island.mBodyStartIndex;
			const PxU32 slabId = batch.slabId[threadIndexInWarp];

			const PxU32 outputOffset = solverDesc->accumulatedBodyDeltaVOffset;
			const PxU32 numDynamicBodies = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!
			const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;

			const PxU32 bodyIdA = batch.bodyAIndex[threadIndexInWarp];
			const PxU32 bodyIdB = batch.bodyBIndex[threadIndexInWarp];

			const PxU32 numArticulations = solverDesc->islandContextPool->mArticulationCount;
			const PxU32 numTotalBodies = bodyOffset + numDynamicBodies + numArticulations;

			const PxU32 slabIndexOffset = (slabId / 32) * numTotalBodies;
			const PxU32 encodedSlabIndex = (1u << (slabId % 32));

			bool isActiveSlab = false;

			if (batch.constraintType == PxgSolverConstraintDesc::eCONSTRAINT_1D)
			{
				// For joint constraints, simply mark slabs as active for efficiency.
				// This does not significantly affect or increase the reference count in practice.
				isActiveSlab = true;
			}
			else
			{
				const PxU32 finalIdA = outputOffset + bodyIdA;
				const PxU32 finalIdB = outputOffset + bodyIdB;

				const float4 vec0 = Pxldcs(iterativeData.solverBodyVelPool[finalIdA]);
				const float4 vec1 = Pxldcs(iterativeData.solverBodyVelPool[finalIdA + totalBodiesIncKinematics]);
				const float4 vec2 = Pxldcs(iterativeData.solverBodyVelPool[finalIdA + 2 * totalBodiesIncKinematics]);

				const float4 vec3 = Pxldcs(iterativeData.solverBodyVelPool[finalIdB]);
				const float4 vec4 = Pxldcs(iterativeData.solverBodyVelPool[finalIdB + totalBodiesIncKinematics]);
				const float4 vec5 = Pxldcs(iterativeData.solverBodyVelPool[finalIdB + 2 * totalBodiesIncKinematics]);

				PxVec3 linVel0(vec0.x, vec0.y, vec0.z);
				PxVec3 angVel0(vec0.w, vec1.x, vec1.y);
				PxVec3 linDelta0(vec1.z, vec1.w, vec2.x);
				PxVec3 angDelta0(vec2.y, vec2.z, vec2.w);

				PxVec3 linVel1(vec3.x, vec3.y, vec3.z);
				PxVec3 angVel1(vec3.w, vec4.x, vec4.y);
				PxVec3 linDelta1(vec4.z, vec4.w, vec5.x);
				PxVec3 angDelta1(vec5.y, vec5.z, vec5.w);

				// Check if the contact/normal constraint is active.
				isActiveSlab = checkActiveContactBlockTGS(batch, linVel0, angVel0, linVel1, angVel1, linDelta0, angDelta0, linDelta1,
														  angDelta1, threadIndexInWarp, iterativeData.blockContactHeaders,
														  iterativeData.blockContactPoints, elapsedTime, minPen);
			}

			// Encode which slab is active in a 32-bit index. When querying reference counts, count the number of active
			// slabs encoded in solverEncodedReferenceCount. solverEncodedReferenceCount contains bitwise/slab-wise
			// activation information.
			if (isActiveSlab & (bodyIdA >= bodyOffset))
			{
				atomicOr(&iterativeData.solverEncodedReferenceCount[slabIndexOffset + bodyIdA], encodedSlabIndex);
			}
			if (isActiveSlab & (bodyIdB >= bodyOffset))
			{
				atomicOr(&iterativeData.solverEncodedReferenceCount[slabIndexOffset + bodyIdB], encodedSlabIndex);
			}

		}
	}
}

// Loosely following "artiSolveBlockPartitionTGSInternal"
static __device__ void markActiveSlab_articulationTGS(const PxgSolverCoreDesc* __restrict solverDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* __restrict sharedDesc,
	const PxgArticulationCoreDesc* const PX_RESTRICT artiDesc,
	const PxU32 islandIndex, const PxU32 lastPartition, 
	PxReal minPen, PxReal elapsedTime)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];
	const PxU32 startPartitionIndex = island.mStartPartitionIndex;

	const PxU32 startIndex = island.mArtiBatchStartIndex;
	const PxU32 articulationBatchOffset = solverDesc->islandContextPool->mBatchCount;
	const PxU32 endIndex = solverDesc->artiConstraintsPerPartition[lastPartition + startPartitionIndex] + articulationBatchOffset;

	const IterativeSolveDataTGS& iterativeData = sharedDesc->iterativeData;

	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;
	const PxU32 threadIndexInWarp = threadIdx.x;
	const PxU32 batchIndex = startIndex + globalWarpIndex + articulationBatchOffset;

	const PxU32 numDynamicBodies = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!
	const PxU32 bodyOffset = island.mBodyStartIndex;

	if (batchIndex < endIndex)
	{
		const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[batchIndex];

		if (threadIndexInWarp < batch.mDescStride)
		{
			const PxNodeIndex igNodeIndexA = batch.bodyANodeIndex[threadIndexInWarp];
			const PxNodeIndex igNodeIndexB = batch.bodyBNodeIndex[threadIndexInWarp];

			const PxU32 slabId = batch.slabId[threadIndexInWarp];

			const PxU32 nodeIndexA = igNodeIndexA.index();
			const PxU32 nodeIndexB = igNodeIndexB.index();

			const PxU32 bodyIdA = batch.bodyAIndex[threadIndexInWarp];
			const PxU32 bodyIdB = batch.bodyBIndex[threadIndexInWarp];

			PxU32 linkIndexA = igNodeIndexA.articulationLinkId();
			PxU32 linkIndexB = igNodeIndexB.articulationLinkId();

			const PxU32 numArticulations = solverDesc->islandContextPool->mArticulationCount;
			const PxU32 numTotalBodies = bodyOffset + numDynamicBodies + numArticulations;

			const PxU32 slabIndexOffset = (slabId / 32) * numTotalBodies;
			const PxU32 encodedSlabIndex = (1u << (slabId % 32));

			bool isActiveSlab = false;

			const PxU32 constraintType = batch.constraintType;
			if (constraintType == PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D) // joint
			{
				// For joint constraints, simply mark slabs as active for efficiency.
				// This does not significantly affect or increase the reference count in practice.
				isActiveSlab = true;
			}
			else // contact
			{
				const PxU32 readIndex = batchIndex * 192 + threadIndexInWarp;

				const PxU32 outputOffset = solverDesc->accumulatedBodyDeltaVOffset;
				const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;

				Cm::UnAlignedSpatialVector vel0, vel1;
				Cm::UnAlignedSpatialVector delta0, delta1;
				PxQuat deltaQ0, deltaQ1;

				if (igNodeIndexA.isArticulation())
				{
					// For articulations, read velocities using readIndex as done in artiSolveBlockPartition.
					const float4 vec0 = Pxldcg(iterativeData.solverBodyVelPool[readIndex]);
					const float4 vec1 = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 32]);
					const float4 vec2 = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 64]);

					vel0 = Cm::UnAlignedSpatialVector(PxVec3(vec0.w, vec1.x, vec1.y), PxVec3(vec0.x, vec0.y, vec0.z));
					delta0 = Cm::UnAlignedSpatialVector(PxVec3(vec2.y, vec2.z, vec2.w), PxVec3(vec1.z, vec1.w, vec2.x));
				}
				else
				{
					// For rigid bodies, use the original rigid body velocity, not a slab velocity, in case velocities
					// at readIndex are not set.
					const PxU32 finalIdA = outputOffset + bodyIdA;

					const float4 vec0 = Pxldcs(iterativeData.solverBodyVelPool[finalIdA]);
					const float4 vec1 = Pxldcs(iterativeData.solverBodyVelPool[finalIdA + totalBodiesIncKinematics]);
					const float4 vec2 = Pxldcs(iterativeData.solverBodyVelPool[finalIdA + 2 * totalBodiesIncKinematics]);

					vel0 = Cm::UnAlignedSpatialVector(PxVec3(vec0.w, vec1.x, vec1.y), PxVec3(vec0.x, vec0.y, vec0.z));
					delta0 = Cm::UnAlignedSpatialVector(PxVec3(vec2.y, vec2.z, vec2.w), PxVec3(vec1.z, vec1.w, vec2.x));
				}

				if (igNodeIndexB.isArticulation())
				{
					// For articulations, read velocities using readIndex as done in artiSolveBlockPartition.
					const float4 vec0 = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 96]);
					const float4 vec1 = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 128]);
					const float4 vec2 = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 160]);

					vel1 = Cm::UnAlignedSpatialVector(PxVec3(vec0.w, vec1.x, vec1.y), PxVec3(vec0.x, vec0.y, vec0.z));
					delta1 = Cm::UnAlignedSpatialVector(PxVec3(vec2.y, vec2.z, vec2.w), PxVec3(vec1.z, vec1.w, vec2.x));
				}
				else
				{
					// For rigid bodies, use the original rigid body velocity, not a slab velocity, in case velocities
					// at readIndex are not set.
					const PxU32 finalIdB = outputOffset + bodyIdB;

					const float4 vec0 = Pxldcs(iterativeData.solverBodyVelPool[finalIdB]);
					const float4 vec1 = Pxldcs(iterativeData.solverBodyVelPool[finalIdB + totalBodiesIncKinematics]);
					const float4 vec2 = Pxldcs(iterativeData.solverBodyVelPool[finalIdB + 2 * totalBodiesIncKinematics]);

					vel1 = Cm::UnAlignedSpatialVector(PxVec3(vec0.w, vec1.x, vec1.y), PxVec3(vec0.x, vec0.y, vec0.z));
					delta1 = Cm::UnAlignedSpatialVector(PxVec3(vec2.y, vec2.z, vec2.w), PxVec3(vec1.z, vec1.w, vec2.x));
				}

				// Check if the contact/normal constraint is active.
				isActiveSlab = checkExtActiveContactBlockTGS(
				    batch, vel0, vel1, delta0, delta1, threadIndexInWarp, iterativeData.blockContactHeaders,
				    iterativeData.blockContactPoints, iterativeData.artiResponse, elapsedTime, minPen);
			}

			if (isActiveSlab)
			{
				// Encode which slab is active in a 32-bit index. When querying reference counts, count the number of
				// active slabs encoded in solverEncodedReferenceCount. solverEncodedReferenceCount contains
				// bitwise/slab-wise activation information.
				if (igNodeIndexA.isArticulation()) // articulation
				{
					const PxU32 articulationBodyIdA = batch.remappedBodyAIndex[threadIndexInWarp];

					// Articulation IDs are at the back of rigid body IDs.
					const PxU32 globalBodyIdA = articulationBodyIdA + numDynamicBodies + bodyOffset;

					atomicOr(&iterativeData.solverEncodedReferenceCount[slabIndexOffset + globalBodyIdA], encodedSlabIndex);
				}
				else if (bodyIdA >= bodyOffset) // rigid
				{
					atomicOr(&iterativeData.solverEncodedReferenceCount[slabIndexOffset + bodyIdA], encodedSlabIndex);
				}

				if (igNodeIndexB.isArticulation()) // articulation
				{
					const PxU32 articulationBodyIdB = batch.remappedBodyBIndex[threadIndexInWarp];

					// Articulation IDs are at the back of rigid body IDs.
					const PxU32 globalBodyIdB = articulationBodyIdB + numDynamicBodies + bodyOffset;

					atomicOr(&iterativeData.solverEncodedReferenceCount[slabIndexOffset + globalBodyIdB], encodedSlabIndex);
				}
				else if (bodyIdB >= bodyOffset) // rigid
				{
					atomicOr(&iterativeData.solverEncodedReferenceCount[slabIndexOffset + bodyIdB], encodedSlabIndex);
				}
			}
		}
	}
}

// Marking active rigid body slabs, loosely following "solveBlockUnified"
extern "C" __global__
__launch_bounds__(PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 8)
void markActiveSlabTGS(const PxgSolverCoreDesc * PX_RESTRICT solverDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>*PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 lastPartition, PxReal minPen, PxReal elapsedTime,
	const PxgArticulationCoreDesc* const PX_RESTRICT artiDesc)
{
	if (blockIdx.y == 0)
	{
		markActiveSlab_rigidBodyTGS(solverDesc, sharedDesc, islandIndex, lastPartition, minPen, elapsedTime);
	}
	else
	{
		markActiveSlab_articulationTGS(solverDesc, sharedDesc, artiDesc, islandIndex, lastPartition, minPen, elapsedTime);
	}
}

extern "C" __global__
__launch_bounds__(PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 8)
void solveBlockUnified(
	PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveDataTGS>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 partitionIndex, bool doFriction, const PxReal elapsedTime,
	const PxReal minPen, const PxgArticulationCoreDesc* const PX_RESTRICT artiDesc
)
{
	if (blockIdx.y == 0)
	{
		PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;
		solveBlockPartitionTGSInternal(solverDesc, sharedDesc, islandIndex, partitionIndex, elapsedTime, minPen, globalWarpIndex, threadIdx.x);
	}
	else
	{
		artiSolveBlockPartitionTGSInternal(solverDesc, sharedDesc, islandIndex, partitionIndex, elapsedTime, minPen, artiDesc);
	}
}

extern "C" __global__
//__launch_bounds__(PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 16)
void solveStaticBlockTGS(
	PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveDataTGS>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 nbStaticSlabs, const PxU32 maxStaticPartitions, const PxReal elapsedTime, const PxReal minPen)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];
	const IterativeSolveDataTGS& iterativeData = sharedDesc->iterativeData;

	float4* PX_RESTRICT bodyVelocities = iterativeData.solverBodyVelPool;
	float4* PX_RESTRICT bodyOutVelocities = iterativeData.tempStaticBodyOutputs;
	const PxU32 numDynamicBodies = island.mBodyCount; //nbBodies minus offset!

	const PxU32 numDynamicBodiesRounded = (island.mBodyCount + 31)&(~31); //Rounded to a multiple of 32

	const PxU32 bodyOffset = island.mBodyStartIndex;
	const PxU32 deltaVOffset = solverDesc->accumulatedBodyDeltaVOffset;
	const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;

	const PxU32 nbDynamicBodiesToSolve = numDynamicBodiesRounded * nbStaticSlabs;

	//This identifies which warp a specific thread is in
	const uint globalThreadIdx = (threadIdx.x + blockIdx.x * blockDim.x);

	const uint warpSize = 32;

	const uint bodyIndex = globalThreadIdx % numDynamicBodiesRounded;
	const PxU32 slabIdx = (globalThreadIdx / numDynamicBodiesRounded);
	const PxU32 startIndex = slabIdx * maxStaticPartitions;

	//This identifies which thread within a warp a specific thread is
	const uint threadIndexInWarp = threadIdx.x&(warpSize - 1);

	PxgSolverTxIData iData0;

	PxgSolverTxIData iData1;

	bool residualAccumulationEnabled = solverDesc->contactErrorAccumulator.mCounter >= 0;
	PxgErrorAccumulator error;

	PxU32 contactCount = 0, jointCount = 0;

	const PxgSolverTxIData* PX_RESTRICT iData = solverDesc->solverBodyTxIDataPool;

	if (globalThreadIdx < nbDynamicBodiesToSolve)
	{
		if (bodyIndex < numDynamicBodies)
		{
			contactCount = PxMin(maxStaticPartitions, PxMax(startIndex, solverDesc->mRigidStaticContactCounts[bodyIndex]) - startIndex);
			jointCount = PxMin(maxStaticPartitions, PxMax(startIndex, solverDesc->mRigidStaticJointCounts[bodyIndex]) - startIndex);

			const PxU32 outputBody = slabIdx * totalBodiesIncKinematics * 2;

			/*printf("BodyIndex = %i, globalThreadIdx = %i, nbDynamicBodiesToSolve = %i, nbStaticSlabs = %i, contactCount = %i, startIndex = %i, outputBody = %i\n", bodyIndex, globalThreadIdx,
				nbDynamicBodiesToSolve, nbStaticSlabs, contactCount, startIndex, outputBody);*/

			if (contactCount != 0 || jointCount != 0)
			{
				//We have some constraints to solve...

				if (jointCount)
				{
					iData0 = iData[bodyIndex + bodyOffset];
					iData1.sqrtInvInertia = PxMat33(PxZero);
					iData1.deltaBody2World = PxTransform(PxIdentity);
				}

				const PxU32 startContactIndex = solverDesc->mRigidStaticContactStartIndices[bodyIndex] + startIndex;
				const PxU32 startJointIndex = solverDesc->mRigidStaticJointStartIndices[bodyIndex] + startIndex;

				assert(startContactIndex >= solverDesc->numBatches);

				const PxU32 inputBody = deltaVOffset + bodyIndex + bodyOffset;

				//Load in velocity data...
				float4 vel0 = bodyVelocities[inputBody];
				float4 vel1 = bodyVelocities[inputBody + totalBodiesIncKinematics];
				float4 vel2 = bodyVelocities[inputBody + totalBodiesIncKinematics + totalBodiesIncKinematics];

				PxVec3 lv0(vel0.x, vel0.y, vel0.z);
				PxVec3 av0(vel0.w, vel1.x, vel1.y);
				PxVec3 ld0(vel1.z, vel1.w, vel2.x);
				PxVec3 ad0(vel2.y, vel2.z, vel2.w);
				PxVec3 lv1(0.f);
				PxVec3 av1(0.f);
				PxVec3 ld1(0.f);
				PxVec3 ad1(0.f);

				for (PxU32 i = 0; i < jointCount; ++i)
				{
					const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[startJointIndex + i];

					assert(batch.constraintType == PxgSolverConstraintDesc::eCONSTRAINT_1D);

					PxU32 idx = warpScanExclusive(batch.mask, threadIndexInWarp);

					// For interaction with static objects, mass-splitting is not used; thus, reference counts are 1 (default).
					solve1DBlockTGS(batch, lv0, av0, lv1, av1, ld0, ad0, ld1, ad1, idx, iterativeData.blockJointConstraintHeaders, iterativeData.blockJointConstraintRowsCon,
						iData0, iData1, elapsedTime, solverDesc->contactErrorAccumulator.mCounter >= 0);
				}

				for (PxU32 i = 0; i < contactCount; ++i)
				{
					const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[startContactIndex + i];

					//printf("Solve batch %i\n", startContactIndex + i);

					assert(batch.constraintType == PxgSolverConstraintDesc::eCONTACT);

					PxU32 idx = warpScanExclusive(batch.mask, threadIndexInWarp);

					// For interaction with static objects, mass-splitting is not used; thus, reference counts are 1 (default).
					solveContactBlockTGS(batch, lv0, av0, lv1, av1, ld0, ad0, ld1, ad1, idx,
						iterativeData.blockContactHeaders, iterativeData.blockFrictionHeaders, iterativeData.blockContactPoints,
						iterativeData.blockFrictions, elapsedTime, minPen, residualAccumulationEnabled ? &error : NULL);
				}

				//if (globalThreadIdx == 33)
				////if(startContactIndex == (solverDesc->numBatches))
				////if(warpScanExclusive(iterativeData.blockConstraintBatch[startContactIndex].mask, threadIndexInWarp) == 0)
				//{
				//	printf("%i: NumContacts = %i, beforeVel = (%f, %f, %f), afterlinVel = (%f, %f, %f), lv1 (%f, %f, %f), av1(%f, %f, %f), startContactIndex = %i, numBatches = %i\n", 
				//		globalThreadIdx, contactCount, linVel.x, linVel.y, linVel.z, lv0.x, lv0.y, lv0.z, lv1.x, lv1.y, lv1.z, av1.x, av1.y, av1.z, startContactIndex, solverDesc->numBatches);
				//}

				vel0.x = lv0.x; vel0.y = lv0.y; vel0.z = lv0.z;
				vel0.w = av0.x; vel1.x = av0.y; vel1.y = av0.z;

				/*printf("BodyOutVelocities[%i] = (%f, %f, %f, %f), bodyOutVelocities[%i] = (%f, %f, %f, %f)\n",
					bodyIndex + outputBody, vel0.x, vel0.y, vel0.z, vel0.w,
					bodyIndex + outputBody + totalBodiesIncKinematics, vel1.x, vel1.y, vel1.z, vel1.w);*/
				bodyOutVelocities[bodyIndex + outputBody] = vel0;
				bodyOutVelocities[bodyIndex + outputBody + totalBodiesIncKinematics] = vel1;
			}
		}
	}

	if (residualAccumulationEnabled)
	{
		error.accumulateErrorGlobalFullWarp(solverDesc->contactErrorAccumulator, threadIndexInWarp);
	}
}

#pragma nv_diagnostic push
#pragma nv_diag_suppress 20054 // dynamic initialization is not supported for a function-scope static __shared__ variable within a __device__/__global__ function

PX_FORCE_INLINE __device__ void applyAcceleration(PxU32 bodyIndex, float4* bodyVelocities, const PxVec3& gravityTimesDt, const PxReal stepDt,
	const PxU16 disableGravity, const float4& linearAcceleration, const float4& angularAcceleration,
	const bool hasLinearAcceleration, const bool hasAngularAcceleration, const float accelScale, const PxU16 lockFlags, const PxgSolverTxIData& inertia, PxU32 angVelIndexOffset)
{
	float4 v = bodyVelocities[bodyIndex];
	v.x += accelScale * gravityTimesDt.x + linearAcceleration.x * stepDt;
	v.y += accelScale * gravityTimesDt.y + linearAcceleration.y * stepDt;
	v.z += accelScale * gravityTimesDt.z + linearAcceleration.z * stepDt;

	v.x = lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X ? 0.f : v.x;
	v.y = lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y ? 0.f : v.y;
	v.z = lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z ? 0.f : v.z;

	if (hasAngularAcceleration)
	{
		float4 a = bodyVelocities[bodyIndex + angVelIndexOffset];

		PxVec3 angAcc = inertia.sqrtInvInertia.getInverse() * PxVec3(angularAcceleration.x, angularAcceleration.y, angularAcceleration.z);

		v.w += angAcc.x * stepDt;
		a.x += angAcc.y * stepDt;
		a.y += angAcc.z * stepDt;

		v.w = lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X ? 0.f : v.w;
		a.x = lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y ? 0.f : a.x;
		a.y = lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z ? 0.f : a.y;

		bodyVelocities[bodyIndex + angVelIndexOffset] = a;
	}

	bodyVelocities[bodyIndex] = v;
}

extern "C" __global__ void applyTGSSubstepGravity(const PxgSolverCoreDesc* PX_RESTRICT solverDesc,
	const PxgSolverSharedDesc<IterativeSolveDataTGS>* PX_RESTRICT sharedDesc, PxVec3 gravity)
{
	const PxReal stepDt = sharedDesc->stepDt;

	const PxgSolverReferences* solverReferences = solverDesc->solverBodyReferences;
	float4* bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;
	const PxU32 bodyOffset = solverDesc->islandContextPool->mBodyStartIndex;
	const PxU32 numSlabs = solverDesc->numSlabs;
	const PxU32 allBodiesCount = solverDesc->numSolverBodies;

	const PxU32 warpIndex = blockIdx.x * blockDim.y + threadIdx.y;

	const PxU32 numThreadsPerBody = PxMin(isPowerOfTwo(numSlabs) ? numSlabs : nextPowerOfTwo(numSlabs), 32u);
	const PxU32 numBodiesPerWarp = WARP_SIZE / numThreadsPerBody;
	const PxU32 threadIdxInWorkUnit = threadIdx.x&(numThreadsPerBody - 1);

	const PxU32 warpBodyStartIndex = warpIndex * numBodiesPerWarp;

	const PxU32 bodyInWarp = threadIdx.x / numThreadsPerBody;

	const PxU32 bodyId = warpBodyStartIndex + bodyInWarp;

	const PxU32 outputOffset = solverDesc->accumulatedBodyDeltaVOffset;

	PxVec3 gravityTimesDt = stepDt * gravity;

	const PxgSolverTxIData* PX_RESTRICT iData = solverDesc->solverBodyTxIDataPool;

	if (bodyId < allBodiesCount)
	{
		if (bodyId >= bodyOffset)
		{
			const PxgSolverBodyData& data = solverDesc->solverBodyDataPool[bodyId];
			const PxU32 nodeIndex = data.islandNodeIndex.index();
			PxgBodySim&	bodySim = solverDesc->mBodySimBufferDeviceData[nodeIndex];
			const PxU16 disableGravity = bodySim.disableGravity;
			const float4 linearAcceleration = bodySim.externalLinearAcceleration;
			const float4 angularAcceleration = bodySim.externalAngularAcceleration;
			const bool hasLinearAcceleration = linearAcceleration.x != 0.0f || linearAcceleration.y != 0.0f || linearAcceleration.z != 0.0f;
			const bool hasAngularAcceleration = angularAcceleration.x != 0.0f || angularAcceleration.y != 0.0f || angularAcceleration.z != 0.0f;
			PxgSolverTxIData iData0;

			if (hasAngularAcceleration)
			{
				iData0 = iData[bodyId /*+ bodyOffset*/];
			}

			if (!disableGravity || hasLinearAcceleration || hasAngularAcceleration)
			{
				float accelScale = bodySim.sleepAngVelAccXYZ_accelScaleW.w;
				PxU16 lockFlags = bodySim.lockFlags;

				for (uint b = threadIdxInWorkUnit; b < numSlabs; b += numThreadsPerBody)
				{
					const uint bodyIndex = numSlabs * (bodyId - bodyOffset) + b;

					PxgSolverReferences reference = solverReferences[bodyIndex];

					const PxU32 remappedBodyIndex = reference.mRemappedBodyIndex;

					if (remappedBodyIndex != 0xFFFFFFFF)
					{
						applyAcceleration(remappedBodyIndex, bodyVelocities, gravityTimesDt, stepDt, disableGravity, linearAcceleration,
							angularAcceleration, hasLinearAcceleration, hasAngularAcceleration, accelScale, lockFlags, iData0, 32);
					}
				}

				if (threadIdxInWorkUnit == 0)
				{
					applyAcceleration(outputOffset + bodyId, bodyVelocities, gravityTimesDt, stepDt, disableGravity, linearAcceleration,
						angularAcceleration, hasLinearAcceleration, hasAngularAcceleration, accelScale, lockFlags, iData0, allBodiesCount);
				}
			}
		}
	}
}

extern "C" __global__
//__launch_bounds__(512, 1)
void solveWholeIslandTGS(
	PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveDataTGS>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, bool doFriction, const PxReal elapsedTime,
	const PxReal minPen)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];

	const PxU32 startPartitionIndex = island.mStartPartitionIndex;

	const PxU32 numDynamicBodies = island.mBodyCount; //nbBodies minus offset!

	const PxU32 bodyOffset = island.mBodyStartIndex;

	const IterativeSolveDataTGS& iterativeData = sharedDesc->iterativeData;

	const PxgSolverTxIData* PX_RESTRICT iData = solverDesc->solverBodyTxIDataPool;

	PxU32 startIndex = island.mBatchStartIndex;

	const PxU32 warpIndex = threadIdx.x / 32;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;
	const PxU32 numWarps = blockDim.x / 32;

	const PxU32 maxBodies = 944;

	__shared__ PxVec3 shLinVel[maxBodies];
	__shared__ PxVec3 shAngVel[maxBodies];
	__shared__ PxVec3 shLinDelta[maxBodies];
	__shared__ PxVec3 shAngDelta[maxBodies];

	const PxU32 deltaVOffset = solverDesc->accumulatedBodyDeltaVOffset;
	const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;

	const PxU32 numSlabs = solverDesc->numSlabs;
	const PxU32 numBatches = solverDesc->numBatches;
	const PxU32 numArticBatches = solverDesc->numArticBatches;

	const PxU32 averageOutputOffsets = (numArticBatches + numBatches) * PXG_BATCH_SIZE * 3 * 2;

	for(PxU32 index = threadIdx.x; index < (totalBodiesIncKinematics * numSlabs); index += blockDim.x)
	{
		const PxU32 inputBody = deltaVOffset + (index % totalBodiesIncKinematics);
		float4 vel0 = iterativeData.solverBodyVelPool[inputBody];
		float4 vel1 = iterativeData.solverBodyVelPool[inputBody + totalBodiesIncKinematics];
		float4 vel2 = iterativeData.solverBodyVelPool[inputBody + totalBodiesIncKinematics + totalBodiesIncKinematics];

		const PxU32 writeIndex = index;

		shLinVel[writeIndex] = PxVec3(vel0.x, vel0.y, vel0.z);
		shAngVel[writeIndex] = PxVec3(vel0.w, vel1.x, vel1.y);
		shLinDelta[writeIndex] = PxVec3(vel1.z, vel1.w, vel2.x);
		shAngDelta[writeIndex] = PxVec3(vel2.y, vel2.z, vel2.w);
	}

	__syncthreads();

	bool residualAccumulationEnabled = solverDesc->contactErrorAccumulator.mCounter >= 0;
	PxgErrorAccumulator error;

	for (PxU32 partitionIndex = 0; partitionIndex < island.mNumPartitions; ++partitionIndex)
	{
		 PxU32 endIndex = solverDesc->constraintsPerPartition[partitionIndex + startPartitionIndex];

		 for (PxU32 k = startIndex + warpIndex; k < endIndex; k += numWarps)
		 {
			 const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[k];

			// const PxU32 readIndex = k * 192 + threadIndexInWarp;

			 PxU32 bodyAIndex = batch.bodyAIndex[threadIndexInWarp];
			 PxU32 bodyBIndex = batch.bodyBIndex[threadIndexInWarp];

			 const PxU32 slabId = batch.slabId[threadIndexInWarp];

			 PxgSolverTxIData iData0, iData1;

			 //unsigned mask_eCONSTRAINT_1D = __ballot_sync(mask_k, batch.constraintType == PxgSolverConstraintDesc::eCONSTRAINT_1D);
			 if (batch.constraintType == PxgSolverConstraintDesc::eCONSTRAINT_1D)
			 {
				 //printf("BodyAIdx = %i, bodyBIdx = %i\n", batch.bodyAIndex[threadIndexInWarp], batch.bodyBIndex[threadIndexInWarp]);
				 loadTxInertia(FULL_MASK, iData, bodyAIndex, batch.mDescStride, threadIndexInWarp, iData0);
				 loadTxInertia(FULL_MASK, iData, bodyBIndex, batch.mDescStride, threadIndexInWarp, iData1);
			 }

			 //Pull out shared memory into float4 format in registers to solve constraints
			 if (threadIndexInWarp < batch.mDescStride)
			 {			
				 const PxU32 readIndexA = bodyAIndex + slabId * totalBodiesIncKinematics;
				 const PxU32 readIndexB = bodyBIndex + slabId * totalBodiesIncKinematics;

				 PxVec3 linVel0 = shLinVel[readIndexA];
				 PxVec3 angVel0 = shAngVel[readIndexA];
				 PxVec3 linDelta0 = shLinDelta[readIndexA];
				 PxVec3 angDelta0 = shAngDelta[readIndexA];

				 PxVec3 linVel1 = shLinVel[readIndexB];
				 PxVec3 angVel1 = shAngVel[readIndexB];
				 PxVec3 linDelta1 = shLinDelta[readIndexB];
				 PxVec3 angDelta1 = shAngDelta[readIndexB];

				 const PxVec3 preLinVel0 = linVel0;
				 const PxVec3 preAngVel0 = angVel0;

				 const PxVec3 preLinVel1 = linVel1;
				 const PxVec3 preAngVel1 = angVel1;

				 PxReal curRef0 = 1.f;
				 PxReal curRef1 = 1.f;
				 
				 const PxU32* const PX_RESTRICT encodedReferenceCount = sharedDesc->iterativeData.solverEncodedReferenceCount;
				 const PxU32 numTotalBodies = bodyOffset + numDynamicBodies + island.mArticulationCount;

				 if(bodyAIndex >= bodyOffset)
				 {
					 // Counting the number of active slabs
					 curRef0 = static_cast<PxReal>(countActiveSlabs(bodyAIndex, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount));
				 }

				 if(bodyBIndex >= bodyOffset)
				 {
					 // Counting the number of active slabs
					 curRef1 = static_cast<PxReal>(countActiveSlabs(bodyBIndex, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount));
				 }

				 if(batch.constraintType == PxgSolverConstraintDesc::eCONTACT)
					 solveContactBlockTGS(batch, linVel0, angVel0, linVel1, angVel1, linDelta0, angDelta0, linDelta1, angDelta1,
										  threadIndexInWarp, iterativeData.blockContactHeaders, iterativeData.blockFrictionHeaders,
										  iterativeData.blockContactPoints, iterativeData.blockFrictions, elapsedTime, minPen,
										  residualAccumulationEnabled ? &error : NULL, curRef0, curRef1);
				 else
					 solve1DBlockTGS(batch, linVel0, angVel0, linVel1, angVel1, linDelta0, angDelta0, linDelta1, angDelta1,
									 threadIndexInWarp, iterativeData.blockJointConstraintHeaders, iterativeData.blockJointConstraintRowsCon,
									 iData0, iData1, elapsedTime, solverDesc->contactErrorAccumulator.mCounter >= 0, curRef0, curRef1);

				 shLinVel[readIndexA] = linVel0;
				 shAngVel[readIndexA] = angVel0;

				 shLinVel[readIndexB] = linVel1;
				 shAngVel[readIndexB] = angVel1;
			 }
		 }

		 __syncthreads();

		 startIndex = endIndex;
	}

	//Get here, write out to accumulation buffers...

	//if (threadIdx.x < (numDynamicBodies * numSlabs))

	for (PxU32 index = threadIdx.x; index < (numDynamicBodies * numSlabs); index += blockDim.x)
	{
		PxU32 bodyId = index / numSlabs;
		PxU32 slabId = index % numSlabs;
		PxU32 bodyIndex = index;

		PxgSolverReferences reference = solverDesc->solverBodyReferences[bodyIndex];

		PxU32 remappedBodyIndex = reference.mRemappedBodyIndex;

		if (remappedBodyIndex != 0xFFFFFFFF)
		{

			const uint velIndex = averageOutputOffsets + ComputeAverageBodyOutputIndex(bodyIndex);

			PxU32 readIndex = (bodyId + bodyOffset + slabId * totalBodiesIncKinematics);

			PxVec3 linVel0 = shLinVel[readIndex];
			PxVec3 angVel0 = shAngVel[readIndex];
			PxVec3 linDelta0 = shLinDelta[readIndex];
			PxVec3 angDelta0 = shAngDelta[readIndex];

			iterativeData.solverBodyVelPool[velIndex] = make_float4(linVel0.x, linVel0.y, linVel0.z, angVel0.x);
			iterativeData.solverBodyVelPool[velIndex + 32] = make_float4(angVel0.y, angVel0.z, linDelta0.x, linDelta0.y);
			iterativeData.solverBodyVelPool[velIndex + 64] = make_float4(linDelta0.z, angDelta0.x, angDelta0.y, angDelta0.z);
		}
	}

	if (residualAccumulationEnabled)
	{
		error.accumulateErrorGlobalFullWarp(solverDesc->contactErrorAccumulator, threadIndexInWarp);
	}
}

#pragma nv_diagnostic pop

extern "C" __global__
//__launch_bounds__(PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 16)
void propagateStaticSolverBodyVelocitiesTGS(
	const PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveDataTGS>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 nbStaticSlabs, const PxU32 maxStaticPartitions)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];
	const IterativeSolveDataTGS& iterativeData = sharedDesc->iterativeData;

	float4* PX_RESTRICT bodyVelocities = iterativeData.solverBodyVelPool;
	float4* PX_RESTRICT bodyOutVelocities = iterativeData.tempStaticBodyOutputs;
	const PxU32 numDynamicBodies = island.mBodyCount; //nbBodies minus offset!
	const PxU32 bodyOffset = island.mBodyStartIndex;
	const PxU32 deltaVOffset = solverDesc->accumulatedBodyDeltaVOffset;
	const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;

	//This identifies which warp a specific thread is in
	const uint globalThreadIdx = (threadIdx.x + blockIdx.x * blockDim.x);

	PxU32 contactCount = 0, jointCount = 0;

	if (globalThreadIdx < numDynamicBodies)
	{
		contactCount = solverDesc->mRigidStaticContactCounts[globalThreadIdx];
		jointCount = solverDesc->mRigidStaticJointCounts[globalThreadIdx];

		if (contactCount || jointCount)
		{
			const PxU32 maxOutputs = PxMax(contactCount, jointCount);

			PxReal scale = 0.f;
			float4 vel0 = make_float4(0.f);
			float4 vel1 = make_float4(0.f);
			for (PxU32 i = 0, index = globalThreadIdx; i < maxOutputs; i += maxStaticPartitions,
				index += totalBodiesIncKinematics*2, scale += 1.0f)
			{
				//We have velocity changes we need to propagate!
				vel0 += bodyOutVelocities[index];
				vel1 += bodyOutVelocities[index + totalBodiesIncKinematics];
			}

			/*printf("BodyOutVelocities[%i] = (%f, %f, %f, %f), bodyOutVelocities[%i] = (%f, %f, %f, %f), scale = %f\n",
				globalThreadIdx + bodyOffset, vel0.x, vel0.y, vel0.z, vel0.w,
				globalThreadIdx + bodyOffset + totalBodiesIncKinematics, vel1.x, vel1.y, vel1.z, vel1.w, scale);*/

			scale = 1.f / scale;

			const PxU32 outputBody = deltaVOffset + globalThreadIdx + bodyOffset;
			bodyVelocities[outputBody] = vel0*scale;
			bodyVelocities[outputBody + totalBodiesIncKinematics] = vel1*scale;
		}
	}
}

