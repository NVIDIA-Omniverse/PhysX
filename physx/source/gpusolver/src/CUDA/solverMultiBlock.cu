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
#include "solverBlock.cuh"
#include "PxgSolverKernelIndices.h"
#include "PxgDynamicsConfiguration.h"
#include "PxgIntrinsics.h"
#include "stdio.h"
#include "assert.h"
#include "reduction.cuh"
#include "solver.cuh"
#include "PxgArticulationCoreDesc.h"

using namespace physx;

extern "C" __host__ void initSolverKernels6() {}

PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 ComputeBodyBatchStartIndex(const PxU32 index)
{
	//return 2*(index & (~31)) + (index&31);
	return index;
}

extern "C" __global__ void ZeroBodies(const PxgSolverCoreDesc* constraintPrepDesc, const PxgSolverSharedDesc<IterativeSolveData>* sharedDesc)
{
	__shared__ float4* bodyVelocities;
	__shared__ float4* motionVelocities;
	__shared__ uint totalNumBodies;
	__shared__ uint totalNumBodiesConstraints;
	__shared__ uint offset;

	if(threadIdx.x == 0)
	{
		bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;
		motionVelocities = constraintPrepDesc->motionVelocityArray;
		totalNumBodies = constraintPrepDesc->numSolverBodies*2;
		totalNumBodiesConstraints = (constraintPrepDesc->numBatches + constraintPrepDesc->numArticBatches) * 32 * 2 *2;
		offset = constraintPrepDesc->accumulatedBodyDeltaVOffset;

	}

	const uint blockStride = (blockDim.x * gridDim.x);

	//This identifies which warp a specific thread is in
	const uint threadIndex = (threadIdx.x + blockIdx.x * blockDim.x);

	__syncthreads();

	const float4 zero = make_float4(0.f);
	//(1) Set all motion velocities to zero.
	//(2) Set the delta velocities in the accumulation offset to zero
	for(uint a = threadIndex; a < totalNumBodies; a+=blockStride)
	{
		motionVelocities[a]					= zero;
		bodyVelocities[offset + a]			= zero;
	}
	
	//(2) Set all velocities to zero. Strictly, we only need to set the first instance of each body
	// in the solver to 0 but, for now, we'll just initialize them all to zero...
	for(uint a = threadIndex; a < totalNumBodiesConstraints; a+= blockStride)
	{
		bodyVelocities[a]					= zero;
	}
}

__device__ __inline__ float4 loadFloat4(const float4* PX_RESTRICT address)
{
	/*float4 ret;

	asm("ld.global.cg.v4.f32 {%0, %1, %2, %3}, [%4];" : "=f"(ret.x), "=f"(ret.y), "=f"(ret.z), "=f"(ret.w) : "r"(address));
	return ret;*/
	return *address;
}

PX_FORCE_INLINE __device__ float4 cudeShuffle3(const PxU32 syncMask, bool condition, float4 reg0, float4 reg1, PxU32 shuffleMask)
{
	float4 ret0, ret1;

	ret0.x = __shfl_sync(syncMask, reg0.x, shuffleMask);
	ret0.y = __shfl_sync(syncMask, reg0.y, shuffleMask);
	ret0.z = __shfl_sync(syncMask, reg0.z, shuffleMask);

	ret1.x = __shfl_sync(syncMask, reg1.x, shuffleMask);
	ret1.y = __shfl_sync(syncMask, reg1.y, shuffleMask);
	ret1.z = __shfl_sync(syncMask, reg1.z, shuffleMask);

	return condition ? ret0 : ret1;
}

// Marking active slabs loosely following "solveBlockPartition"
static __device__ void markActiveSlab_rigidBodyPGS(
	const PxgSolverCoreDesc* PX_RESTRICT solverDesc, 
	const PxgSolverSharedDesc<IterativeSolveData>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 lastPartition)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];
	const PxU32 startPartitionIndex = island.mStartPartitionIndex;

	const PxU32 startIndex = island.mBatchStartIndex;
	const PxU32 endIndex = solverDesc->constraintsPerPartition[lastPartition + startPartitionIndex];

	const IterativeSolveData& iterativeData = sharedDesc->iterativeData;

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

			const PxU32 outputOffset = solverDesc->accumulatedBodyDeltaVOffset; //deltaVOffset
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

				const float4 linVel0 = Pxldcg(iterativeData.solverBodyVelPool[finalIdA]);
				const float4 angVel0 = Pxldcg(iterativeData.solverBodyVelPool[finalIdA + totalBodiesIncKinematics]);
				const float4 linVel1 = Pxldcg(iterativeData.solverBodyVelPool[finalIdB]);
				const float4 angVel1 = Pxldcg(iterativeData.solverBodyVelPool[finalIdB + totalBodiesIncKinematics]);

				PxVec3 lv0(linVel0.x, linVel0.y, linVel0.z);
				PxVec3 lv1(linVel1.x, linVel1.y, linVel1.z);
				PxVec3 av0(angVel0.x, angVel0.y, angVel0.z);
				PxVec3 av1(angVel1.x, angVel1.y, angVel1.z);

				// Check if the contact/normal constraint is active.
				isActiveSlab = checkActiveContactBlock(batch, lv0, av0, lv1, av1, threadIndexInWarp, iterativeData.blockContactHeaders,
													   iterativeData.blockContactPoints);
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

// Marking active slabs loosely following "artiSolveBlockPartition"
static __device__ void markActiveSlab_articulationPGS(
	const PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveData>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 lastPartition, const PxgArticulationCoreDesc* const PX_RESTRICT artiDesc)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];
	const PxU32 startPartitionIndex = island.mStartPartitionIndex;

	const PxU32 startIndex = island.mArtiBatchStartIndex;
	const PxU32 articulationBatchOffset = solverDesc->islandContextPool->mBatchCount;
	const PxU32 endIndex = solverDesc->artiConstraintsPerPartition[lastPartition + startPartitionIndex] + articulationBatchOffset;

	const IterativeSolveData& iterativeData = sharedDesc->iterativeData;

	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + threadIdx.y;
	const PxU32 threadIndexInWarp = threadIdx.x;
	const PxU32 batchIndex = startIndex + globalWarpIndex + articulationBatchOffset;

	const PxU32 bodyOffset = island.mBodyStartIndex;
	const PxU32 numDynamicBodies = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!

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

			Cm::UnAlignedSpatialVector vel0, vel1;

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
				const PxU32 readIndex = batchIndex * 128 + threadIndexInWarp;

				const PxU32 outputOffset = solverDesc->accumulatedBodyDeltaVOffset;
				const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;

				Cm::UnAlignedSpatialVector vel0, vel1;
				if (igNodeIndexA.isArticulation())
				{
					// For articulations, read velocities using readIndex as done in artiSolveBlockPartition.
					const float4 lin = Pxldcg(iterativeData.solverBodyVelPool[readIndex]);
					const float4 ang = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 32]);
					vel0 = Cm::UnAlignedSpatialVector(PxVec3(ang.x, ang.y, ang.z), PxVec3(lin.x, lin.y, lin.z));
				}
				else
				{
					// For rigid bodies, use the original rigid body velocity, not a slab velocity, in case velocities
					// at readIndex are not set.
					const PxU32 finalIdA = outputOffset + bodyIdA;
					const float4 lin = Pxldcg(iterativeData.solverBodyVelPool[finalIdA]);
					const float4 ang = Pxldcg(iterativeData.solverBodyVelPool[finalIdA + totalBodiesIncKinematics]);
					vel0 = Cm::UnAlignedSpatialVector(PxVec3(ang.x, ang.y, ang.z), PxVec3(lin.x, lin.y, lin.z));
				}

				if (igNodeIndexB.isArticulation())
				{
					// For articulations, read velocities using readIndex as done in artiSolveBlockPartition.
					const float4 lin = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 64]);
					const float4 ang = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 96]);
					vel1 = Cm::UnAlignedSpatialVector(PxVec3(ang.x, ang.y, ang.z), PxVec3(lin.x, lin.y, lin.z));
				}
				else
				{
					// For rigid bodies, use the original rigid body velocity, not a slab velocity, in case velocities
					// at readIndex are not set.
					const PxU32 finalIdB = outputOffset + bodyIdB;
					const float4 lin = Pxldcg(iterativeData.solverBodyVelPool[finalIdB]);
					const float4 ang = Pxldcg(iterativeData.solverBodyVelPool[finalIdB + totalBodiesIncKinematics]);
					vel1 = Cm::UnAlignedSpatialVector(PxVec3(ang.x, ang.y, ang.z), PxVec3(lin.x, lin.y, lin.z));
				}

				// Check if the contact/normal constraint is active.
				isActiveSlab = checkExtActiveContactBlock(batch, vel0, vel1, iterativeData.blockContactHeaders,
					iterativeData.blockContactPoints, iterativeData.artiResponse, threadIndexInWarp);
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

// Marking active rigid body slabs, loosely following "solveBlockPartition" and "artiSolveBlockPartition"
extern "C" __global__
__launch_bounds__(PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 8)
void markActiveSlabPGS(const PxgSolverCoreDesc * PX_RESTRICT solverDesc,
	const PxgSolverSharedDesc<IterativeSolveData>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 lastPartition, const PxgArticulationCoreDesc* const PX_RESTRICT artiDesc)
{
	if (blockIdx.y == 0)
	{
		markActiveSlab_rigidBodyPGS(solverDesc, sharedDesc, islandIndex, lastPartition);
	}
	else
	{
		markActiveSlab_articulationPGS(solverDesc, sharedDesc, islandIndex, lastPartition, artiDesc);
	}
}

extern "C" __global__
//__launch_bounds__(PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 16)
void solveBlockPartition(
	PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveData>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 partitionIndex, bool doFriction)
{	
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];

	const PxU32 startPartitionIndex = island.mStartPartitionIndex;

	PxU32 startIndex = partitionIndex == 0 ? island.mBatchStartIndex : solverDesc->constraintsPerPartition[partitionIndex + startPartitionIndex - 1];

	PxU32 endIndex = solverDesc->constraintsPerPartition[partitionIndex + startPartitionIndex];

	const uint warpSize = 32;

	//This identifies which warp a specific thread is in
	const uint warpIndex = (threadIdx.x + blockIdx.x * blockDim.x)/warpSize;

	//This identifies which thread within a warp a specific thread is
	const uint threadIndexInWarp = threadIdx.x&(warpSize-1);

	__shared__ IterativeSolveData iterativeData;

	PxU32 idx = threadIdx.x;

	//if(threadIdx.x < sizeof(iterativeData)/sizeof(float))
	while(idx < sizeof(iterativeData) / sizeof(float))
	{
		float* iterData = reinterpret_cast<float*>(&iterativeData);

		iterData[idx] = reinterpret_cast<const float*>(&sharedDesc->iterativeData)[idx];
		idx += warpSize;
	}

	__syncthreads();

	bool residualAccumulationEnabled = solverDesc->contactErrorAccumulator.mCounter >= 0;
	PxgErrorAccumulator error;

	//for(uint k = startIndex + warpIndex; k < endIndex; k+=blockStride)
	uint k = startIndex + warpIndex;
	if(k < endIndex)
	{
		assert(k < solverDesc->numBatches);

		const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[k];

		const PxU32 readIndex = k*128 + threadIndexInWarp;

		//Pull out shared memory into float4 format in registers to solve constraints
		if(threadIndexInWarp < batch.mDescStride)
		{
			//The linear/angular velocity pair for body 0 for threads 0 - 15, loaded by threads 0 - 31 in the format (b0.linVel, b0.angVel, b0.linVel, b0.angVel...)
			float4 linVel0 = Pxldcg(iterativeData.solverBodyVelPool[readIndex]);
			//The linear/angular velocity pair for body 0 for threads 16-31, loaded by threads 0 - 31  in the format (b0.linVel, b0.angVel, b0.linVel, b0.angVel...)
			float4 angVel0 = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 32]);
			//The linear/angular velocity pair for body 1 for threads 0 - 15, loaded by threads 0 - 31 in the format (b1.linVel, b1.angVel, b1.linVel, b1.angVel...)
			float4 linVel1 = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 64]);
			//The linear/angular velocity pair for body 1 for threads 16 - 31, loaded by threads 0 - 31 in the format (b1.linVel, b1.angVel, b1.linVel, b1.angVel...)
			float4 angVel1 = Pxldcg(iterativeData.solverBodyVelPool[readIndex + 96]);

			//printf("Rigid ReadIndex = %i, linVel0=(%f, %f, %f), angVel0=(%f, %f, %f)\n", readIndex, linVel0.x, linVel0.y, linVel0.z, angVel0.x, angVel0.y, angVel0.z);

			//printf("Rigid ReadIndex = %i, linVel0=(%f, %f, %f), angVel0=(%f, %f, %f)\n", readIndex+64, linVel1.x, linVel1.y, linVel1.z, angVel1.x, angVel1.y, angVel1.z);

			PxVec3 lv0(linVel0.x, linVel0.y, linVel0.z);
			PxVec3 lv1(linVel1.x, linVel1.y, linVel1.z);
			PxVec3 av0(angVel0.x, angVel0.y, angVel0.z);
			PxVec3 av1(angVel1.x, angVel1.y, angVel1.z);

			// Reference counts to be used for the current sub-timestep or iteration.
			PxReal curRef0 = 1.0f;
			PxReal curRef1 = 1.0f;

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

			if (batch.constraintType == PxgSolverConstraintDesc::eCONTACT)
				solveContactBlock(batch, lv0, av0, lv1, av1, doFriction, threadIndexInWarp, iterativeData.blockContactHeaders, iterativeData.blockFrictionHeaders,
					iterativeData.blockContactPoints, iterativeData.blockFrictions, residualAccumulationEnabled ? &error : NULL, 
					curRef0, curRef1);
			else
				solve1DBlock(batch, lv0, av0, lv1, av1, threadIndexInWarp, iterativeData.blockJointConstraintHeaders, iterativeData.blockJointConstraintRowsCon,
					iterativeData.blockJointConstraintRowsMod, solverDesc->contactErrorAccumulator.mCounter >= 0, curRef0, curRef1);


			const PxU32 remapA = batch.remappedBodyAIndex[threadIndexInWarp];
			const PxU32 remapB = batch.remappedBodyBIndex[threadIndexInWarp];
			
			const PxU32 indexA = ComputeBodyBatchStartIndex(remapA);
			const PxU32 indexB = ComputeBodyBatchStartIndex(remapB);

			Pxstcg(&iterativeData.solverBodyVelPool[indexA], make_float4(lv0.x, lv0.y, lv0.z, 0.f));
			Pxstcg(&iterativeData.solverBodyVelPool[indexA + 32], make_float4(av0.x, av0.y, av0.z, 0.f));
			Pxstcg(&iterativeData.solverBodyVelPool[indexB], make_float4(lv1.x, lv1.y, lv1.z, 0.f));
			Pxstcg(&iterativeData.solverBodyVelPool[indexB + 32], make_float4(av1.x, av1.y, av1.z, 0.f));
		}

		//KS - Even threads in a warp output the linear velocities, odd threads output the angular velocities.
		//We output for threadIndexInWarp/2 and 16 + threadIndexInWarp/2
		
#if 0
		const PxU32 firstWarpOutput = threadIndexInWarp/2;
		const PxU32 secondWarpOutput = threadIndexInWarp/2 + 16;

		float4 indexAOutput0 = cudeShuffle3(threadIndexInWarp & 1, angVel0, linVel0, firstWarpOutput);
		float4 indexBOutput0 = cudeShuffle3(threadIndexInWarp & 1, angVel1, linVel1, firstWarpOutput);

		float4 indexAOutput1 = cudeShuffle3(threadIndexInWarp & 1, angVel0, linVel0, secondWarpOutput);
		float4 indexBOutput1 = cudeShuffle3(threadIndexInWarp & 1, angVel1, linVel1, secondWarpOutput);

		if(firstWarpOutput < batch.mDescStride)
		{
			const PxU32 indexA = (2*batch.remappedBodyAIndex[firstWarpOutput] + (threadIdx.x&1));
			const PxU32 indexB = (2*batch.remappedBodyBIndex[firstWarpOutput] + (threadIdx.x&1));

			iterativeData.solverBodyVelPool[indexA] = indexAOutput0;
			iterativeData.solverBodyVelPool[indexB] = indexBOutput0;
		}

		if(secondWarpOutput < batch.mDescStride)
		{
			const PxU32 indexA = (2*batch.remappedBodyAIndex[secondWarpOutput] + (threadIdx.x&1));
			const PxU32 indexB = (2*batch.remappedBodyBIndex[secondWarpOutput] + (threadIdx.x&1));

			iterativeData.solverBodyVelPool[indexA] = indexAOutput1;
			iterativeData.solverBodyVelPool[indexB] = indexBOutput1;
		}
#else

#endif
	}

	if (residualAccumulationEnabled)
	{
		error.accumulateErrorGlobalFullWarp(solverDesc->contactErrorAccumulator, threadIndexInWarp);
	}
}

extern "C" __global__ void writebackBlocks(
	PxgSolverCoreDesc* constraintPrepDesc, 
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc, const PxU32 islandIndex)
{	
	PxgIslandContext& island = constraintPrepDesc->islandContextPool[islandIndex];

	const PxU32 startIndex = island.mBatchStartIndex;
	const PxU32 endIndex = island.mBatchCount + island.mArtiBatchCount + island.mStaticArtiBatchCount + 
		island.mSelfArtiBatchCount + island.mStaticRigidBatchCount + startIndex;

	const uint warpSize = 32;

	//const uint blockStride = (blockDim.x * gridDim.x)/warpSize;

	//This identifies which warp a specific thread is in
	const uint warpIndex = (threadIdx.x + blockIdx.x * blockDim.x)/warpSize;

	const uint warpIndexInBlock = threadIdx.x/warpSize;

	//This identifies which thread within a warp a specific thread is
	const uint threadIndexInWarp = threadIdx.x&(warpSize-1);

	PxgBlockConstraintBatch* batchHeaders = sharedDesc->iterativeData.blockConstraintBatch;

	PxgBlockSolverConstraint1DHeader* jointHeaders = sharedDesc->iterativeData.blockJointConstraintHeaders;
	PxgBlockSolverConstraint1DCon* jointRowsCon = sharedDesc->iterativeData.blockJointConstraintRowsCon;
	PxgBlockSolverConstraint1DMod* jointRowsMod = sharedDesc->iterativeData.blockJointConstraintRowsMod;

	PxgConstraintWriteback* constraintWriteBack = constraintPrepDesc->constraintWriteBack;

	PxgSolverBodyData* solverBodyDatas = constraintPrepDesc->solverBodyDataPool;

	PxgBlockSolverContactHeader* contactHeaders = sharedDesc->iterativeData.blockContactHeaders;
	PxgBlockSolverFrictionHeader* frictionHeaders = sharedDesc->iterativeData.blockFrictionHeaders;
	PxgBlockSolverContactPoint* contactPoints = sharedDesc->iterativeData.blockContactPoints;
	PxgBlockSolverContactFriction* frictions = sharedDesc->iterativeData.blockFrictions;

	PxgBlockFrictionPatch* baseFrictionPatches = sharedDesc->blockCurrentFrictionPatches;
	PxF32* baseWritebackForceBuffer = constraintPrepDesc->forceBuffer;

	PxgFrictionPatchGPU* frictionPatches = reinterpret_cast<PxgFrictionPatchGPU*>(constraintPrepDesc->frictionPatches);

	//__shared__  Dy::ThresholdStreamElement elems[PxgKernelBlockDim::WRITEBACK_BLOCKS];
	__shared__  PxU8 elemsMem[sizeof(Dy::ThresholdStreamElement)*PxgKernelBlockDim::WRITEBACK_BLOCKS];
	Dy::ThresholdStreamElement* elems = reinterpret_cast<Dy::ThresholdStreamElement*>(elemsMem);
	__shared__  PxI32 index[PxgKernelBlockDim::WRITEBACK_BLOCKS/warpSize];

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
				writeBackContactBlock(batch, threadIndexInWarp, solverBodyDatas, startAddress, &index[warpIndexInBlock], contactHeaders, frictionHeaders, contactPoints, frictions,
					baseWritebackForceBuffer, baseFrictionPatches[batch.mConstraintBatchIndex], frictionPatches);
			}
			else
			{
				//Do nothing (for now)
				writeBack1DBlock(batch, threadIndexInWarp, jointHeaders, jointRowsCon, jointRowsMod, constraintWriteBack);
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

extern "C" __global__ void concludeBlocks(
	PxgSolverCoreDesc* constraintPrepDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc, const PxU32 islandIndex)
{	
	PxgIslandContext& island = constraintPrepDesc->islandContextPool[islandIndex];

	const PxU32 startIndex = island.mBatchStartIndex;
	const PxU32 endIndex = island.mBatchCount + island.mArtiBatchCount + island.mStaticArtiBatchCount 
		+ island.mSelfArtiBatchCount + island.mStaticRigidBatchCount + startIndex;

	

	const uint warpSize = 32;

	//const uint blockStride = (blockDim.x * gridDim.x)/warpSize;

	//This identifies which warp a specific thread is in
	const uint warpIndex = (threadIdx.x + blockIdx.x * blockDim.x)/warpSize;

	//This identifies which thread within a warp a specific thread is
	const uint threadIndexInWarp = threadIdx.x&(warpSize-1);

	PxgBlockConstraintBatch* batchHeaders = sharedDesc->iterativeData.blockConstraintBatch;

	PxgBlockSolverConstraint1DHeader* jointHeaders = sharedDesc->iterativeData.blockJointConstraintHeaders;
	PxgBlockSolverConstraint1DMod* jointRowsMod = sharedDesc->iterativeData.blockJointConstraintRowsMod;

	PxgBlockSolverContactHeader* contactHeaders = sharedDesc->iterativeData.blockContactHeaders;
	PxgBlockSolverFrictionHeader* frictionHeaders = sharedDesc->iterativeData.blockFrictionHeaders;
	PxgBlockSolverContactPoint* contactPoints = sharedDesc->iterativeData.blockContactPoints;
	PxgBlockSolverContactFriction* frictions = sharedDesc->iterativeData.blockFrictions;

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
				concludeContactBlock(batch, threadIndexInWarp, contactHeaders, frictionHeaders, contactPoints, frictions);
			}
			else
			{
				conclude1DBlock(batch, threadIndexInWarp, jointHeaders, jointRowsMod);
			}
		}
	}
}

extern "C" __global__ void writeBackBodies(
	const PxgSolverCoreDesc* constraintPrepDesc,
	const PxgSolverSharedDesc<IterativeSolveData>* sharedDesc, const PxU32 islandIndex)
{	
	__shared__ float4* bodyVelocities;
	__shared__ float4* motionVelocities;
	__shared__ PxU32 bodyStartIndex;
	__shared__ PxU32 bodyEndIndex;
	__shared__ PxU32 totalBodyCount;
	__shared__ PxU32 outputOffsetIndex;

	if(threadIdx.x == 0)
	{
		PxgIslandContext& island = constraintPrepDesc->islandContextPool[islandIndex];
		bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;
		motionVelocities = constraintPrepDesc->motionVelocityArray;
		bodyStartIndex = island.mBodyStartIndex;
		totalBodyCount = constraintPrepDesc->numSolverBodies;
		bodyEndIndex = totalBodyCount;
		outputOffsetIndex = sharedDesc->deltaOutOffset;
	}
	__syncthreads();

	//const uint blockStride = (blockDim.x * gridDim.x);

	//This identifies which warp a specific thread is in
	const uint threadIndex = (threadIdx.x + blockIdx.x * blockDim.x);

	//for(uint a = threadIndex+bodyStartIndex; a < bodyEndIndex; a+=blockStride)
	uint a = threadIndex+bodyStartIndex;
	if(a < bodyEndIndex)
	{
		motionVelocities[a]						= bodyVelocities[a + outputOffsetIndex];		//Linear velocity
		motionVelocities[a+totalBodyCount]		= bodyVelocities[a + outputOffsetIndex + totalBodyCount];	//Angular velocity stored after all linear velocities
	}
}

extern "C" __global__ void computeAverageSolverBodyVelocity(
	const PxgSolverCoreDesc* const PX_RESTRICT solverDesc,
	const PxgSolverSharedDesc<IterativeSolveData>* PX_RESTRICT sharedDesc)
{
	//we need to fill in the writeIndex for the solver body data
	const PxgSolverReferences* const PX_RESTRICT solverReferences = solverDesc->solverBodyReferences;
	float4* bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;

	PxU32* encodedReferenceCount = sharedDesc->iterativeData.solverEncodedReferenceCount;
	const PxU32 totalBodyCount = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!
	const PxU32 bodyOffset = solverDesc->islandContextPool->mBodyStartIndex;
	const PxU32 numSlabs = solverDesc->numSlabs;
	const PxU32 numBatches = solverDesc->numBatches;
	const PxU32 numArticBatches = solverDesc->numArticBatches;
	const PxU32 deltaVOffset = solverDesc->accumulatedBodyDeltaVOffset;

	const PxU32 averageOutputOffsets = (numBatches + numArticBatches) * PXG_BATCH_SIZE * 2 * 2;

	const PxU32 numThreadsPerBody = PxMin(isPowerOfTwo(numSlabs) ? numSlabs : nextPowerOfTwo(numSlabs), 32u);
	const PxU32 numBodiesPerWarp = WARP_SIZE / numThreadsPerBody;

	const uint warpIndex = (blockIdx.x * blockDim.y + threadIdx.y);

	const uint warpBodyStartIndex = warpIndex * numBodiesPerWarp;

	const PxU32 threadIdInWorkUnit = threadIdx.x & (numThreadsPerBody - 1);
	const PxU32 subWarpIndex = threadIdx.x / numThreadsPerBody;

	const PxU32 bodyInWarp = threadIdx.x / numThreadsPerBody;

	const PxU32 maskHigh = ((1 << ((subWarpIndex + 1) * numThreadsPerBody)) - 1);
	const PxU32 maskLow = ((1 << ((subWarpIndex)*numThreadsPerBody)) - 1);
	const PxU32 mask = maskHigh - maskLow;

	const uint bodyId = warpBodyStartIndex + bodyInWarp;

	if (bodyId < totalBodyCount)
	{
		float4 linDelta = make_float4(0.f);
		float4 angDelta = make_float4(0.f);

		const PxU32 outputBody = deltaVOffset + bodyId + bodyOffset;

		//Store linear and angular velocity!!!
		const float4 lastLinearVel = bodyVelocities[outputBody];
		const float4 lastAngularVel = bodyVelocities[outputBody + totalBodyCount + bodyOffset];

		bool hasRef = false;
		//get out velocity for the correponding body in a slab and put on weight to calculate the final velocity for that body
		for (uint b = threadIdInWorkUnit; b < numSlabs; b += numThreadsPerBody)
		{
			const uint bodyIndex = bodyId * numSlabs + b;
			PxgSolverReferences reference = solverReferences[bodyIndex];

			PxU32 remappedBodyIndex = reference.mRemappedBodyIndex;

			if (remappedBodyIndex != 0xFFFFFFFF)
			{
				//Outputs are grouped into little clusters of 32...
				const uint velIndex = averageOutputOffsets + ComputeAverageBodyBatchStartIndex(bodyIndex);

				const float4 curLinearDeltaV = bodyVelocities[velIndex];
				const float4 curAngularDeltaV = bodyVelocities[velIndex + 32];

				float4 lDelta = (curLinearDeltaV - lastLinearVel);
				float4 aDelta = (curAngularDeltaV - lastAngularVel);

				{
					linDelta += lDelta;
					angDelta += aDelta;
				}
				hasRef = true;
			}
		}

		//Now do the reduction...
#pragma unroll
		for (PxU32 reductionRadius = numThreadsPerBody >> 1; reductionRadius > 0; reductionRadius >>= 1)
		{
			linDelta.x += __shfl_xor_sync(mask, linDelta.x, reductionRadius, numThreadsPerBody);
			linDelta.y += __shfl_xor_sync(mask, linDelta.y, reductionRadius, numThreadsPerBody);
			linDelta.z += __shfl_xor_sync(mask, linDelta.z, reductionRadius, numThreadsPerBody);

			angDelta.x += __shfl_xor_sync(mask, angDelta.x, reductionRadius, numThreadsPerBody);
			angDelta.y += __shfl_xor_sync(mask, angDelta.y, reductionRadius, numThreadsPerBody);
			angDelta.z += __shfl_xor_sync(mask, angDelta.z, reductionRadius, numThreadsPerBody);
		}

		hasRef = (__ballot_sync(mask, hasRef) & mask);


		if (hasRef)
		{
			if (threadIdInWorkUnit == 0)
			{
				// Counting the number of active slabs
				const PxU32 numDynamicBodies = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!
				const PxU32 numArticulations = solverDesc->islandContextPool->mArticulationCount;
				const PxU32 numTotalBodies = bodyOffset + numDynamicBodies + numArticulations;

				const PxU32 referenceCount =
				    countActiveSlabs(bodyOffset + bodyId, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount);
				const PxReal recipRefs = 1.f / static_cast<PxReal>(referenceCount);

				linDelta = linDelta * recipRefs;
				angDelta = angDelta * recipRefs;

				// Resetting rigid body reference count
				resetSlabCount(bodyOffset + bodyId, solverDesc->numSlabs, numTotalBodies, encodedReferenceCount);

				//Store linear and angular velocity!!!
				bodyVelocities[outputBody] = lastLinearVel + linDelta;
				bodyVelocities[outputBody + totalBodyCount + bodyOffset] = lastAngularVel + angDelta;
			}
		}
	}
}

extern "C" __global__ void propagateSolverBodyVelocity(
	const PxgSolverCoreDesc* solverDesc,
	const PxgSolverSharedDesc<IterativeSolveData>* sharedDesc)
{
	//we need to fill in the writeIndex for the solver body data
	PxgSolverReferences* solverReferences = solverDesc->solverBodyReferences;
	float4* bodyVelocities = sharedDesc->iterativeData.solverBodyVelPool;
	const PxU32 totalBodyCount = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!
	const PxU32 bodyOffset = solverDesc->islandContextPool->mBodyStartIndex; //nbBodies minus offset!
	const PxU32 numSlabs = solverDesc->numSlabs;
	const PxU32 deltaVOffset = solverDesc->accumulatedBodyDeltaVOffset;

	//This identifies which warp a specific thread is in
	const uint threadIndex = (threadIdx.x + blockIdx.x * blockDim.x);

	const PxU32 numThreadsPerBody = PxMin(isPowerOfTwo(numSlabs) ? numSlabs : nextPowerOfTwo(numSlabs), 32u);
	const PxU32 numBodiesPerWarp = WARP_SIZE / numThreadsPerBody;

	const PxU32 warpIndex = threadIndex / WARP_SIZE;

	const uint warpBodyStartIndex = warpIndex * numBodiesPerWarp;

	const PxU32 threadIdInWorkUnit = threadIdx.x&(numThreadsPerBody - 1);

	const PxU32 bodyInWarp = (threadIdx.x&31) / numThreadsPerBody;

	const uint bodyId = warpBodyStartIndex + bodyInWarp;

	if(bodyId < totalBodyCount)
	{
		const PxU32 outputBody = deltaVOffset + bodyId + bodyOffset;

		//Store linear and angular velocity!!!
		float4 linVel = bodyVelocities[outputBody];
		float4 angVel = bodyVelocities[outputBody + totalBodyCount + bodyOffset];
		linVel.w = 0.f;

		for(uint b=threadIdInWorkUnit; b<numSlabs; b += numThreadsPerBody)
		{
			const uint bodyIndex = (bodyId)*numSlabs + b;

			PxgSolverReferences reference = solverReferences[bodyIndex];
			PxU32 remappedBodyIndex = reference.mRemappedBodyIndex;
			if(remappedBodyIndex != 0xFFFFFFFF)
			{
				const uint velIndex = ComputeBodyBatchStartIndex(remappedBodyIndex);
				bodyVelocities[velIndex] = linVel;
				bodyVelocities[velIndex + 32] = angVel;
			}
		}
	}
}

extern "C" __global__ void dmaBackChangedElems(const PxgSolverCoreDesc* solverDesc, Dy::ThresholdStreamElement* hostChangedElems)
{
	Dy::ThresholdStreamElement* changeElems = solverDesc->forceChangeThresholdElements;
	PxU32 nbElemsChanges = solverDesc->nbForceChangeElements;

	PxU32 nbThreadsRequired = (sizeof(Dy::ThresholdStreamElement) * nbElemsChanges)/sizeof(PxU32);

	PxU32* src = reinterpret_cast<PxU32*>(changeElems);
	PxU32* dst = reinterpret_cast<PxU32*>(hostChangedElems);

	PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	for(PxU32 i = globalThreadIdx; i < nbThreadsRequired; i+= blockDim.x * gridDim.x)
	{
		dst[i] = src[i];
	}
}

extern "C" __global__ void dmaConstraintResidual(const PxgConstraintWriteback* writebacks, PxReal* residuals, PxU32 count)
{
	PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	if (globalThreadIdx < count)
	{
		residuals[globalThreadIdx] = writebacks[globalThreadIdx].angularImpulse_residual.w;
	}
}

extern "C" __global__
//__launch_bounds__(PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 16)
void solveStaticBlock(
	PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveData>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 nbStaticSlabs, const PxU32 maxStaticPartitions, bool doFriction)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];
	const IterativeSolveData& iterativeData = sharedDesc->iterativeData;

	float4* PX_RESTRICT bodyVelocities = iterativeData.solverBodyVelPool;
	float4* PX_RESTRICT bodyOutVelocities = iterativeData.tempStaticBodyOutputPool;
	const PxU32 numDynamicBodies = island.mBodyCount; //nbBodies minus offset!
	const PxU32 bodyOffset = island.mBodyStartIndex;
	const PxU32 deltaVOffset = solverDesc->accumulatedBodyDeltaVOffset;
	const PxU32 totalBodiesIncKinematics = numDynamicBodies + bodyOffset;

	const PxU32 numDynamicBodiesRounded = (island.mBodyCount + 31)&(~31); //Rounded to a multiple of 32

	const uint warpSize = 32;

	//This identifies which warp a specific thread is in
	const uint globalThreadIdx = (threadIdx.x + blockIdx.x * blockDim.x);

	//This identifies which thread within a warp a specific thread is
	const uint threadIndexInWarp = threadIdx.x&(warpSize - 1);

	const PxU32 nbDynamicBodiesToSolve = numDynamicBodiesRounded * nbStaticSlabs;

	const uint bodyIndex = globalThreadIdx % numDynamicBodiesRounded;
	const PxU32 slabIdx = (globalThreadIdx / numDynamicBodiesRounded);
	const PxU32 startIndex = slabIdx * maxStaticPartitions;

	bool residualAccumulationEnabled = solverDesc->contactErrorAccumulator.mCounter >= 0;
	PxgErrorAccumulator error;

	if (globalThreadIdx < nbDynamicBodiesToSolve)
	{
		if (bodyIndex < numDynamicBodies)
		{
			PxU32 contactCount = PxMin(maxStaticPartitions, PxMax(startIndex, solverDesc->mRigidStaticContactCounts[bodyIndex]) - startIndex);
			PxU32 jointCount = PxMin(maxStaticPartitions, PxMax(startIndex, solverDesc->mRigidStaticJointCounts[bodyIndex]) - startIndex);

			const PxU32 outputBody = slabIdx * totalBodiesIncKinematics * 2;

			//printf("BodyIndex = %i, globalThreadIdx = %i, nbDynamicBodiesToSolve = %i, nbStaticSlabs = %i, contactCount = %i, startIndex = %i, outputBody = %i\n", bodyIndex, globalThreadIdx,
			//	nbDynamicBodiesToSolve, nbStaticSlabs, contactCount, startIndex, outputBody);

			if (contactCount != 0 || jointCount != 0)
			{
				//We have some constraints to solve...

				const PxU32 startContactIndex = solverDesc->mRigidStaticContactStartIndices[bodyIndex] + startIndex;
				const PxU32 startJointIndex = solverDesc->mRigidStaticJointStartIndices[bodyIndex] + startIndex;

				assert(startContactIndex >= solverDesc->numBatches);

				const PxU32 inputBody = deltaVOffset + bodyIndex + bodyOffset;

				//Load in velocity data...
				float4 linVel = bodyVelocities[inputBody];
				float4 angVel = bodyVelocities[inputBody + totalBodiesIncKinematics];

				PxVec3 lv0(linVel.x, linVel.y, linVel.z);
				PxVec3 lv1(0.f);
				PxVec3 av0(angVel.x, angVel.y, angVel.z);
				PxVec3 av1(0.f);

				for (PxU32 i = 0; i < jointCount; ++i)
				{
					const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[startJointIndex + i];

					assert(batch.constraintType == PxgSolverConstraintDesc::eCONSTRAINT_1D);

					PxU32 idx = warpScanExclusive(batch.mask, threadIndexInWarp);

					// For interaction with static objects, mass-splitting is not used; thus, reference counts are 1.
					solve1DBlock(batch, lv0, av0, lv1, av1, idx, iterativeData.blockJointConstraintHeaders, iterativeData.blockJointConstraintRowsCon,
						iterativeData.blockJointConstraintRowsMod, solverDesc->contactErrorAccumulator.mCounter >= 0, 1.f, 1.f);
				}

				for (PxU32 i = 0; i < contactCount; ++i)
				{
					const PxgBlockConstraintBatch& batch = iterativeData.blockConstraintBatch[startContactIndex + i];

					assert(batch.constraintType == PxgSolverConstraintDesc::eCONTACT);

					PxU32 idx = warpScanExclusive(batch.mask, threadIndexInWarp);

					// For interaction with static objects, mass-splitting is not used; thus, reference counts are 1.
					solveContactBlock(batch, lv0, av0, lv1, av1, doFriction, idx, iterativeData.blockContactHeaders, iterativeData.blockFrictionHeaders,
						iterativeData.blockContactPoints, iterativeData.blockFrictions, residualAccumulationEnabled ? &error : NULL, 
						1.f, 1.f);
				}

				//if (globalThreadIdx == 33)
				////if(startContactIndex == (solverDesc->numBatches))
				////if(warpScanExclusive(iterativeData.blockConstraintBatch[startContactIndex].mask, threadIndexInWarp) == 0)
				//{
				//	printf("%i: NumContacts = %i, beforeVel = (%f, %f, %f), afterlinVel = (%f, %f, %f), lv1 (%f, %f, %f), av1(%f, %f, %f), startContactIndex = %i, numBatches = %i\n", 
				//		globalThreadIdx, contactCount, linVel.x, linVel.y, linVel.z, lv0.x, lv0.y, lv0.z, lv1.x, lv1.y, lv1.z, av1.x, av1.y, av1.z, startContactIndex, solverDesc->numBatches);
				//}

				linVel.x = lv0.x; linVel.y = lv0.y; linVel.z = lv0.z;
				angVel.x = av0.x; angVel.y = av0.y; angVel.z = av0.z;

				/*printf("%i: BodyOutVelocities[%i] = (%f, %f, %f, %f), bodyOutVelocities[%i] = (%f, %f, %f, %f)\n",
					globalThreadIdx, bodyIndex + outputBody, linVel.x, linVel.y, linVel.z, linVel.w,
					bodyIndex + outputBody + totalBodiesIncKinematics, angVel.x, angVel.y, angVel.z, angVel.w);*/

				bodyOutVelocities[bodyIndex + outputBody] = linVel;
				bodyOutVelocities[bodyIndex + outputBody + totalBodiesIncKinematics] = angVel;

			}
		}
	}
	if (residualAccumulationEnabled)
	{
		error.accumulateErrorGlobalFullWarp(solverDesc->contactErrorAccumulator, threadIndexInWarp);
	}
}


extern "C" __global__
//__launch_bounds__(PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 16)
void propagateStaticSolverBodyVelocities(
	const PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveData>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 nbStaticSlabs, const PxU32 maxStaticPartitions)
{
	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];
	const IterativeSolveData& iterativeData = sharedDesc->iterativeData;

	float4* PX_RESTRICT bodyVelocities = iterativeData.solverBodyVelPool;
	float4* PX_RESTRICT bodyOutVelocities = iterativeData.tempStaticBodyOutputPool;
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

			//We have velocity changes we need to propagate!
			const PxU32 outputBody = deltaVOffset + globalThreadIdx + bodyOffset;

			PxReal scale = 0.f;
			float4 vel0 = make_float4(0.f);
			float4 vel1 = make_float4(0.f);
			for (PxU32 i = 0, index = globalThreadIdx; i < maxOutputs; i += maxStaticPartitions,
				index += totalBodiesIncKinematics * 2, scale += 1.0f)
			{
				//We have velocity changes we need to propagate!
				vel0 += bodyOutVelocities[index];
				vel1 += bodyOutVelocities[index + totalBodiesIncKinematics];
			}

			scale = 1.f / scale;

			/*printf("BodyOutVelocities[%i] = (%f, %f, %f, %f), bodyOutVelocities[%i] = (%f, %f, %f, %f), scale = %f\n",
				globalThreadIdx + bodyOffset, vel0.x, vel0.y, vel0.z, vel0.w,
				globalThreadIdx + bodyOffset + totalBodiesIncKinematics, vel1.x, vel1.y, vel1.z, vel1.w, scale);*/

			bodyVelocities[outputBody] = vel0* scale;
			bodyVelocities[outputBody + totalBodiesIncKinematics] = vel1* scale;
		}
	}
}

