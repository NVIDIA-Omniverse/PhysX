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

#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "cutil_math.h"
#include "PxgParticleSystemCoreKernelIndices.h"
#include "reduction.cuh"
#include "shuffle.cuh"
#include "PxgSolverCoreDesc.h"
#include "PxNodeIndex.h"
#include "assert.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgArticulationCoreDesc.h"

using namespace physx;

extern "C" __host__ void initSimulationControllerKernels2() {}

/*
 * This kernel takes a *sorted* list of PxNodeIndex (of size *numContacts), 
 * and a corresponding deltaV list.
 * 
 * deltaV is updated with the cumulativ delta values, such that for each last entry of a rigid body deltaV is the total sum 
 * for that rigid body - *however* only for the rigid body entries which are processed within one block.
 * blockRigidId, blockDeltaV are updated, such that entry represents the deltaV sum and rigid body ID, of the last
 * occuring rigid body in that corresponding block. This can then be used in the subsequent kernel to complete the sum 
 * for rigid bodies, which entries are overlapping a block.
 */
extern "C" __global__ void accumulateDeltaVRigidFirstLaunch(
	const PxU64*					sortedRigidIds,			//input
	const PxU32*					numContacts,			//input
	float4*							deltaV,					//input/output
	float4*							blockDeltaV,			//output
	PxU64*							blockRigidId			//output
)
{
	__shared__ PxU64 sRigidId[PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + 1];

	//numWarpsPerBlock can't be larger than 32
	const PxU32 numWarpsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA / WARP_SIZE;

	__shared__ float4 sLinWarpAccumulator[WARP_SIZE];
	__shared__ float4 sAngWarpAccumulator[WARP_SIZE];
	__shared__ PxU64  sWarpRigidId[WARP_SIZE];
	__shared__ float4 sLinBlockAccumulator;
	__shared__ float4 sAngBlockAccumulator;
	__shared__ PxU64 sBlockRigidId;

	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);
	const PxU32 warpIndex = threadIdx.x / WARP_SIZE;

	if (threadIdx.x < 4)
	{
		float* tLinBlockAccumulator = reinterpret_cast<float*>(&sLinBlockAccumulator.x);
		tLinBlockAccumulator[threadIdx.x] = 0.f;

		float* tAngBlockAccumulator = reinterpret_cast<float*>(&sAngBlockAccumulator.x);
		tAngBlockAccumulator[threadIdx.x] = 0.f;

		if (threadIdx.x == 0)
			sBlockRigidId = 0x8fffffffffffffff;
	}
	__syncthreads();

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = blockDim.x*(blockIdx.x*nbIterationsPerBlock + i) + threadIdx.x;

		PxU64 rigidId = 0x8fffffffffffffff;
		float4 linDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);
		float4 angDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);
		if (workIndex < tNumContacts)
		{
			rigidId = sortedRigidIds[workIndex];
			sRigidId[threadIdx.x] = rigidId;
			linDeltaV = deltaV[workIndex];
			angDeltaV = deltaV[workIndex + tNumContacts];
		}
		__syncthreads();

		for (PxU32 reductionRadius = 1; reductionRadius < WARP_SIZE; reductionRadius <<= 1)
		{
			const PxU32 lane = threadIndexInWarp - reductionRadius;

			float4 linVal = shuffle(FULL_MASK, linDeltaV, lane);
			float4 angVal = shuffle(FULL_MASK, angDeltaV, lane);

			//workIndex < tNumContacts guarantees that sRigidId[WARP_SIZE * warpIndex + lane]
			//always points to initialized memory, since lane is always smaller than threadIndexInWarp.
			if (threadIndexInWarp >= reductionRadius && workIndex < tNumContacts && 
				rigidId == sRigidId[WARP_SIZE * warpIndex + lane])
			{
				linDeltaV += linVal;
				angDeltaV += angVal;
			}
		}

		if (threadIndexInWarp == (WARP_SIZE - 1))
		{
			sLinWarpAccumulator[warpIndex] = linDeltaV;
			sAngWarpAccumulator[warpIndex] = angDeltaV;
			sWarpRigidId[warpIndex] = rigidId;
		}

		const float4 prevLinBlockAccumulator = sLinBlockAccumulator;
		const float4 prevAngBlockAccumulator = sAngBlockAccumulator;
		const PxU64 prevBlockRigidId = sBlockRigidId;

		//Don't allow write until we've finished all reading...
		__syncthreads();

		if (warpIndex == 0)
		{
			float4 linDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);
			float4 angDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);

			PxU64 warpRigidId = 0x8fffffffffffffff;
			if (threadIndexInWarp < numWarpsPerBlock)
			{
				linDeltaV = sLinWarpAccumulator[threadIndexInWarp];
				angDeltaV = sAngWarpAccumulator[threadIndexInWarp];
				warpRigidId = sWarpRigidId[threadIndexInWarp];
			}

			float4 tLinDeltaV = linDeltaV;
			float4 tAngDeltaV = angDeltaV;

			for (PxU32 reductionRadius = 1; reductionRadius < numWarpsPerBlock; reductionRadius <<= 1)
			{
				const PxU32 lane = threadIndexInWarp - reductionRadius;
				float4 linVal = shuffle(FULL_MASK, tLinDeltaV, lane);
				float4 angVal = shuffle(FULL_MASK, tAngDeltaV, lane);

				if (threadIndexInWarp >= reductionRadius && warpRigidId == sWarpRigidId[lane])
				{
					tLinDeltaV += linVal;
					tAngDeltaV += angVal;
				}
			}

			if (threadIndexInWarp == (numWarpsPerBlock - 1))
			{
				if (sBlockRigidId != warpRigidId)
				{
					//need to clear block accumulators in case previous iteration
					//stored other sBlockRigidId
					sLinBlockAccumulator = make_float4(0.f, 0.f, 0.f, 0.f);
					sAngBlockAccumulator = make_float4(0.f, 0.f, 0.f, 0.f);
				}
				sLinBlockAccumulator += tLinDeltaV;
				sAngBlockAccumulator += tAngDeltaV;
				sBlockRigidId = warpRigidId;
			}

			sLinWarpAccumulator[threadIndexInWarp] = tLinDeltaV;
			sAngWarpAccumulator[threadIndexInWarp] = tAngDeltaV;
		}
		__syncthreads();

		if (workIndex < tNumContacts)
		{
			float4 accumLin = make_float4(0.f, 0.f, 0.f, 0.f);
			float4 accumAng = make_float4(0.f, 0.f, 0.f, 0.f);

			//if rigidId and the previous element rigid Id is the same, we need to add the previous warp accumulate velocity to
			//the current rigid body
			if (warpIndex > 0 && rigidId == sWarpRigidId[warpIndex - 1])
			{
				accumLin = sLinWarpAccumulator[warpIndex - 1];
				accumAng = sAngWarpAccumulator[warpIndex - 1];
			}

			if (i != 0 && rigidId == prevBlockRigidId)
			{
				accumLin += prevLinBlockAccumulator;
				accumAng += prevAngBlockAccumulator;
			}

			//Now output both offsets...
			deltaV[workIndex] = linDeltaV + accumLin;
			deltaV[workIndex + tNumContacts] = angDeltaV + accumAng;
		}
	}

	if (threadIdx.x == 0)
	{
		blockDeltaV[blockIdx.x] = sLinBlockAccumulator;
		blockDeltaV[blockIdx.x + PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA] = sAngBlockAccumulator;
		blockRigidId[blockIdx.x] = sBlockRigidId;
	}
}


//32 blocks. Each block compute the exclusive ransum for the blockOffset
extern "C" __global__ void accumulateDeltaVRigidSecondLaunch(
	const PxU64*								sortedRigidIds,			//input
	const PxU32*								numContacts,			//input
	const float4*								deltaV,					//input
	const float4*								blockDeltaV,			//input
	const PxU64*								blockRigidId,			//input
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>*	sharedDesc,
	const bool isTGS
)
{
	__shared__ float4 sBlockLinDeltaV[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ float4 sBlockAngDeltaV[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ PxU64 sBlockRigidId[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ PxU64 sRigidId[PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + 1];

	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	float4 linBlockDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);
	float4 angBlockDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);

	PxU64 tBlockRigidId = 0x8fffffffffffffff;
	if (threadIdx.x < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
	{
		linBlockDeltaV = blockDeltaV[threadIdx.x];
		angBlockDeltaV = blockDeltaV[threadIdx.x + PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
		tBlockRigidId = blockRigidId[threadIdx.x];

		sBlockLinDeltaV[threadIdx.x] = linBlockDeltaV;
		sBlockAngDeltaV[threadIdx.x] = angBlockDeltaV;
		sBlockRigidId[threadIdx.x] = tBlockRigidId;
	}

	__syncthreads(); //sBlockRigidId is written above and read below

	float4 tLinDeltaV = linBlockDeltaV;
	float4 tAngDeltaV = angBlockDeltaV;
	//add on block deltaV if blockRigid id match
	for (PxU32 reductionRadius = 1; reductionRadius < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA; reductionRadius <<= 1)
	{
		const PxU32 lane = threadIndexInWarp - reductionRadius;
		float4 linVal = shuffle(FULL_MASK, tLinDeltaV, lane);
		float4 angVal = shuffle(FULL_MASK, tAngDeltaV, lane);

		if (threadIndexInWarp >= reductionRadius && tBlockRigidId == sBlockRigidId[lane])
		{
			tLinDeltaV += linVal;
			tAngDeltaV += angVal;
		}
	}

	__syncthreads(); //sBlockRigidId is read above and written below

	if (threadIdx.x < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
	{
		sBlockLinDeltaV[threadIdx.x] = tLinDeltaV;
		sBlockAngDeltaV[threadIdx.x] = tAngDeltaV;
		sBlockRigidId[threadIdx.x] = blockRigidId[threadIdx.x];
	}

	__syncthreads();

	float4* solverBodyDeltaVel = sharedDesc->iterativeData.solverBodyVelPool + solverCoreDesc->accumulatedBodyDeltaVOffset;
	//float4* initialVel = solverCoreDesc->outSolverVelocity;
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;

	PxgArticulationBlockData* artiData = artiCoreDesc->mArticulationBlocks;
	PxgArticulationBlockLinkData* artiLinkData = artiCoreDesc->mArticulationLinkBlocks;

	const PxU32 maxLinks = artiCoreDesc->mMaxLinksPerArticulation;
	
	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		__syncthreads(); //sRigidId is read and written in the same loop - read and write must be separated by syncs

		const PxU32 workIndex = blockDim.x * (blockIdx.x * nbIterationsPerBlock + i) + threadIdx.x;

		PxU64 rigidId = 0x8fffffffffffffff;
		if (workIndex < tNumContacts)
		{
			rigidId = sortedRigidIds[workIndex];
			if (threadIdx.x > 0)
				sRigidId[threadIdx.x - 1] = rigidId;

			if (workIndex == tNumContacts - 1)
			{
				sRigidId[threadIdx.x] = 0x8fffffffffffffff;
			}
			else if (threadIdx.x == PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA - 1)
			{
				// first thread in block must load neighbor particle 
				sRigidId[threadIdx.x] = sortedRigidIds[workIndex + 1];
			}
		}

		__syncthreads();

		if (workIndex < tNumContacts)
		{
			float4 accumLin = make_float4(0.f, 0.f, 0.f, 0.f);
			float4 accumAng = make_float4(0.f, 0.f, 0.f, 0.f);

			if (rigidId != sRigidId[threadIdx.x])
			{
				float4 linVel = deltaV[workIndex];
				float4 angVel = deltaV[workIndex + tNumContacts];

				PxU64 preBlockRigidId = blockIdx.x > 0 ? sBlockRigidId[blockIdx.x - 1] : 0x8fffffffffffffff;

				if (rigidId == preBlockRigidId)
				{
					linVel += sBlockLinDeltaV[blockIdx.x - 1];
					angVel += sBlockAngDeltaV[blockIdx.x - 1];
				}

				//nodeIndex
				const PxNodeIndex nodeId = reinterpret_cast<PxNodeIndex&>(rigidId);

				PxU32 solverBodyIndex = 0;

				if (!nodeId.isStaticBody())
				{
					PxU32 nodeIndex = nodeId.index();
					solverBodyIndex = prePrepDesc->solverBodyIndices[nodeIndex];

					if (nodeId.isArticulation())
					{
						//solverBodyIndex is the globalThreadIndex for the active articulation in the block format
						const PxU32 blockIndex = solverBodyIndex / WARP_SIZE;

						PxgArticulationBlockData& articulation = artiData[blockIndex];
						PxgArticulationBlockLinkData* artiLinks = &artiLinkData[blockIndex * maxLinks];
						
						
						const PxU32 artiIndexInBlock = solverBodyIndex % WARP_SIZE;

						articulation.mStateDirty[artiIndexInBlock] = PxgArtiStateDirtyFlag::eHAS_IMPULSES;

						const PxU32 linkID = nodeId.articulationLinkId();

						const PxReal denom = PxMax(1.0f, linVel.w);
						PxReal ratio = 1.f / denom;
						linVel.w = 0.f;

						//for articulation, linVel and angVel accumulate impulse
						Cm::UnAlignedSpatialVector impulse;
						impulse.top = PxVec3(linVel.x, linVel.y, linVel.z );
						impulse.bottom = PxVec3(angVel.x, angVel.y, angVel.z);

						impulse.top *= ratio;
						impulse.bottom *= ratio;
						
						/*printf("blockIndex %i artiIndexInBlock %i linkID %i ratio %f impulse linear(%f, %f, %f) angular(%f, %f, %f)\n", blockIndex, artiIndexInBlock, linkID, ratio,
							impulse.top.x, impulse.top.y, impulse.top.z, impulse.bottom.x, impulse.bottom.y, impulse.bottom.z);*/
					
						storeSpatialVector(artiLinks[linkID].mScratchImpulse, -impulse, artiIndexInBlock);
					}
					else
					{
						float4 linearVelocity = solverBodyDeltaVel[solverBodyIndex];
						float4 angularVelocity = solverBodyDeltaVel[solverBodyIndex + numSolverBodies];

						const PxReal denom = PxMax(1.0f, linVel.w);
						PxReal ratio = 1.f / denom;
						linVel.w = 0.f;

						if (isTGS)
						{
							linearVelocity.x += linVel.x * ratio;
							linearVelocity.y += linVel.y * ratio;
							linearVelocity.z += linVel.z * ratio;
							linearVelocity.w += angVel.x * ratio;
							angularVelocity.x += angVel.y * ratio;
							angularVelocity.y += angVel.z * ratio;
							//The rest is the delta position buffer
						}
						else
						{
							/*assert(PxIsFinite(linVel.x)); assert(PxIsFinite(linVel.y)); assert(PxIsFinite(linVel.z));
							assert(PxIsFinite(angVel.x)); assert(PxIsFinite(angVel.y)); assert(PxIsFinite(angVel.z));
							assert(PxIsFinite(ratio));*/

							/*printf("Accum linVelDelta = (%f, %f, %f, %f), angVelDelta = (%f, %f, %f, %f), ratio = %f, denom = %f, globalRelax = %f\n",
								linVel.x, linVel.y, linVel.z, linVel.w, angVel.x, angVel.y, angVel.z, angVel.w, ratio, denom, globalRelaxationCoefficient);*/

							linearVelocity += linVel * ratio;
							angularVelocity += angVel * ratio;
						}

						solverBodyDeltaVel[solverBodyIndex] = linearVelocity;
						solverBodyDeltaVel[solverBodyIndex + numSolverBodies] = angularVelocity;

					}
					//printf("solverBodyIndex %i\n", solverBodyIndex);
					//printf("linearVelocity(%f, %f, %f, %f)\n", linearVelocity.x, linearVelocity.y, linearVelocity.z, linearVelocity.w);
					//printf("angularVelocity(%f, %f, %f, %f)\n", angularVelocity.x, angularVelocity.y, angularVelocity.z, angularVelocity.w);
				}
			}
		}
	}
}


//32 blocks. Each block compute the exclusive ransum for the blockOffset
extern "C" __global__ void clearDeltaVRigidSecondLaunchMulti(
	PxU64*							sortedRigidIds,			//input
	PxU32*							numContacts,			//input
	PxgPrePrepDesc*					prePrepDesc,
	PxgSolverCoreDesc*				solverCoreDesc,
	PxgArticulationCoreDesc*		artiCoreDesc,
	PxReal*							tempDenom
)
{
	__shared__ PxU64 sRigidId[PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + 1];

	const PxU32 tNumContacts = *numContacts;
	const PxU32 idx = threadIdx.x;

	const PxU32 totalBlockRequired = (tNumContacts + (PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;
	const PxU32 numIterationPerBlock = (totalBlockRequired + (PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;

	PxgArticulationBlockLinkData* artiLinkData = artiCoreDesc->mArticulationLinkBlocks;

	const PxU32 maxLinks = artiCoreDesc->mMaxLinksPerArticulation;

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU64 rigidId = 0x8fffffffffffffff;
		if (workIndex < tNumContacts)
		{
			rigidId = sortedRigidIds[workIndex];
			if (idx > 0)
				sRigidId[idx - 1] = rigidId;

			if (workIndex == tNumContacts - 1)
			{
				sRigidId[idx] = 0x8fffffffffffffff;
			}
			else if (threadIdx.x == PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA - 1)
			{
				// first thread in block must load neighbor particle 
				sRigidId[idx] = sortedRigidIds[workIndex + 1];
			}


		}

		__syncthreads();

		if (workIndex < tNumContacts)
		{
			if (rigidId != sRigidId[idx])
			{

				//nodeIndex
				const PxNodeIndex nodeId = reinterpret_cast<PxNodeIndex&>(rigidId);

				PxU32 solverBodyIndex = 0;

				if (!nodeId.isStaticBody())
				{
					PxU32 nodeIndex = nodeId.index();
					solverBodyIndex = prePrepDesc->solverBodyIndices[nodeIndex];

					if (nodeId.isArticulation())
					{
						//solverBodyIndex is the globalThreadIndex for the active articulation in the block format
						const PxU32 blockIndex = solverBodyIndex / WARP_SIZE;

						PxgArticulationBlockLinkData* artiLinks = &artiLinkData[blockIndex * maxLinks];

						const PxU32 artiIndexInBlock = solverBodyIndex % WARP_SIZE;

						const PxU32 linkID = nodeId.articulationLinkId();

						artiLinks[linkID].mDeltaScale[artiIndexInBlock] = 0.f;
					}
					else
					{
						tempDenom[solverBodyIndex] = 0.f;
					}
				}
			}
		}
	}
}



//32 blocks. Each block compute the exclusive ransum for the blockOffset
extern "C" __global__ void accumulateDeltaVRigidSecondLaunchMultiStage1(
	PxU64*							sortedRigidIds,			//input
	PxU32*							numContacts,			//input
	float4*							deltaV,					//input
	float4*							blockDeltaV,			//input
	PxU64*							blockRigidId,			//input
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>*	sharedDesc,
	PxReal*							tempDenom,
	const bool useLocalRelax,
	const float globalRelaxationCoefficient,
	bool isTGS
)
{
	__shared__ PxReal sBlockLinDeltaVW[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ PxU64 sBlockRigidId[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ PxU64 sRigidId[PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + 1];

	const PxU32 tNumContacts = *numContacts;
	const PxU32 idx = threadIdx.x;

	const PxU32 threadIndexInWarp = idx & (WARP_SIZE - 1);

	const PxU32 totalBlockRequired = (tNumContacts + (PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;

	float4 linBlockDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);
	float4 angBlockDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);

	PxU64 tBlockRigidId = 0x8fffffffffffffff;
	if (idx < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
	{
		linBlockDeltaV = blockDeltaV[idx];
		tBlockRigidId = blockRigidId[idx];

		sBlockLinDeltaVW[idx] = linBlockDeltaV.w;
		sBlockRigidId[idx] = tBlockRigidId;
	}

	__syncthreads(); //sBlockRigidId is written above and read below

	PxReal tLinDeltaVW = linBlockDeltaV.w;
	//add on block deltaV if blockRigid id match
	for (PxU32 reductionRadius = 1; reductionRadius < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA; reductionRadius <<= 1)
	{
		const PxU32 lane = threadIndexInWarp - reductionRadius;
		PxReal w = __shfl_sync(FULL_MASK, tLinDeltaVW, lane);

		if (threadIndexInWarp >= reductionRadius && tBlockRigidId == sBlockRigidId[lane])
		{
			tLinDeltaVW += w;
		}
	}

	__syncthreads(); //sBlockRigidId is read above and written below

	if (idx < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
	{
		sBlockLinDeltaVW[idx] = tLinDeltaVW;
		sBlockRigidId[idx] = blockRigidId[idx];
	}

	const PxU32 numIterationPerBlock = (totalBlockRequired + (PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;

	__syncthreads();

	PxgArticulationBlockLinkData* artiLinkData = artiCoreDesc->mArticulationLinkBlocks;

	const PxU32 maxLinks = artiCoreDesc->mMaxLinksPerArticulation;

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU64 rigidId = 0x8fffffffffffffff;
		if (workIndex < tNumContacts)
		{
			rigidId = sortedRigidIds[workIndex];
			if (idx > 0)
				sRigidId[idx - 1] = rigidId;

			if (workIndex == tNumContacts - 1)
			{
				sRigidId[idx] = 0x8fffffffffffffff;
			}
			else if (threadIdx.x == PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA - 1)
			{
				// first thread in block must load neighbor particle 
				sRigidId[idx] = sortedRigidIds[workIndex + 1];
			}


		}

		__syncthreads();

		if (workIndex < tNumContacts)
		{
			if (rigidId != sRigidId[idx])
			{
				PxReal linVelW = deltaV[workIndex].w;

				PxU64 preBlockRigidId = blockIdx.x > 0 ? sBlockRigidId[blockIdx.x - 1] : 0x8fffffffffffffff;

				if (rigidId == preBlockRigidId)
				{
					linVelW += sBlockLinDeltaVW[blockIdx.x - 1];
				}

				//nodeIndex
				const PxNodeIndex nodeId = reinterpret_cast<PxNodeIndex&>(rigidId);

				PxU32 solverBodyIndex = 0;

				if (!nodeId.isStaticBody())
				{
					PxU32 nodeIndex = nodeId.index();
					solverBodyIndex = prePrepDesc->solverBodyIndices[nodeIndex];

					PxReal denom = globalRelaxationCoefficient;

					if (useLocalRelax)
						denom = PxMax(denom, linVelW);

					if (nodeId.isArticulation())
					{
						//solverBodyIndex is the globalThreadIndex for the active articulation in the block format
						const PxU32 blockIndex = solverBodyIndex / WARP_SIZE;

						PxgArticulationBlockLinkData* artiLinks = &artiLinkData[blockIndex * maxLinks];

						const PxU32 artiIndexInBlock = solverBodyIndex % WARP_SIZE;

						const PxU32 linkID = nodeId.articulationLinkId();

						atomicAdd(&artiLinks[linkID].mDeltaScale[artiIndexInBlock], denom);
					}
					else
					{
						atomicAdd(&tempDenom[solverBodyIndex], denom);
						
					}
				}
			}
		}
	}
}

//32 blocks. Each block compute the exclusive ransum for the blockOffset
extern "C" __global__ void accumulateDeltaVRigidSecondLaunchMultiStage2(
	PxU64*							sortedRigidIds,			//input
	PxU32*							numContacts,			//input
	float4*							deltaV,					//input
	float4*							blockDeltaV,			//input
	PxU64*							blockRigidId,			//input
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>*	sharedDesc,
	PxReal*							tempDenom,
	const bool useLocalRelax,
	const float globalRelaxationCoefficient,
	bool isTGS
)
{
	__shared__ float4 sBlockLinDeltaV[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ float4 sBlockAngDeltaV[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ PxU64 sBlockRigidId[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ PxU64 sRigidId[PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + 1];

	const PxU32 tNumContacts = *numContacts;
	const PxU32 idx = threadIdx.x;

	const PxU32 threadIndexInWarp = idx & (WARP_SIZE - 1);

	const PxU32 totalBlockRequired = (tNumContacts + (PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;

	float4 linBlockDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);
	float4 angBlockDeltaV = make_float4(0.f, 0.f, 0.f, 0.f);

	PxU64 tBlockRigidId = 0x8fffffffffffffff;
	if (idx < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
	{
		linBlockDeltaV = blockDeltaV[idx];
		angBlockDeltaV = blockDeltaV[idx + PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
		tBlockRigidId = blockRigidId[idx];

		sBlockLinDeltaV[idx] = linBlockDeltaV;
		sBlockAngDeltaV[idx] = angBlockDeltaV;
		sBlockRigidId[idx] = tBlockRigidId;
	}

	__syncthreads(); //sBlockRigidId is written above and read below

	float4 tLinDeltaV = linBlockDeltaV;
	float4 tAngDeltaV = angBlockDeltaV;
	//add on block deltaV if blockRigid id match
	for (PxU32 reductionRadius = 1; reductionRadius < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA; reductionRadius <<= 1)
	{
		const PxU32 lane = threadIndexInWarp - reductionRadius;
		float4 linVal = shuffle(FULL_MASK, tLinDeltaV, lane);
		float4 angVal = shuffle(FULL_MASK, tAngDeltaV, lane);

		if (threadIndexInWarp >= reductionRadius && tBlockRigidId == sBlockRigidId[lane])
		{
			tLinDeltaV += linVal;
			tAngDeltaV += angVal;
		}
	}

	__syncthreads(); //sBlockRigidId is read above and written below

	if (idx < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
	{
		sBlockLinDeltaV[idx] = tLinDeltaV;
		sBlockAngDeltaV[idx] = tAngDeltaV;
		sBlockRigidId[idx] = blockRigidId[idx];
	}

	const PxU32 numIterationPerBlock = (totalBlockRequired + (PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;

	__syncthreads();

	float4* solverBodyDeltaVel = sharedDesc->iterativeData.solverBodyVelPool + solverCoreDesc->accumulatedBodyDeltaVOffset;
	//float4* initialVel = solverCoreDesc->outSolverVelocity;
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;

	PxgArticulationBlockData* artiData = artiCoreDesc->mArticulationBlocks;
	PxgArticulationBlockLinkData* artiLinkData = artiCoreDesc->mArticulationLinkBlocks;

	const PxU32 maxLinks = artiCoreDesc->mMaxLinksPerArticulation;

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU64 rigidId = 0x8fffffffffffffff;
		if (workIndex < tNumContacts)
		{
			rigidId = sortedRigidIds[workIndex];
			if (idx > 0)
				sRigidId[idx - 1] = rigidId;

			if (workIndex == tNumContacts - 1)
			{
				sRigidId[idx] = 0x8fffffffffffffff;
			}
			else if (threadIdx.x == PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA - 1)
			{
				// first thread in block must load neighbor particle 
				sRigidId[idx] = sortedRigidIds[workIndex + 1];
			}


		}

		__syncthreads();

		if (workIndex < tNumContacts)
		{
			float4 accumLin = make_float4(0.f, 0.f, 0.f, 0.f);
			float4 accumAng = make_float4(0.f, 0.f, 0.f, 0.f);

			if (rigidId != sRigidId[idx])
			{
				float4 linVel = deltaV[workIndex];
				float4 angVel = deltaV[workIndex + tNumContacts];

				PxU64 preBlockRigidId = blockIdx.x > 0 ? sBlockRigidId[blockIdx.x - 1] : 0x8fffffffffffffff;

				if (rigidId == preBlockRigidId)
				{
					linVel += sBlockLinDeltaV[blockIdx.x - 1];
					angVel += sBlockAngDeltaV[blockIdx.x - 1];
				}

				//nodeIndex
				const PxNodeIndex nodeId = reinterpret_cast<PxNodeIndex&>(rigidId);

				PxU32 solverBodyIndex = 0;

				if (!nodeId.isStaticBody())
				{
					PxU32 nodeIndex = nodeId.index();
					solverBodyIndex = prePrepDesc->solverBodyIndices[nodeIndex];

					if (nodeId.isArticulation())
					{
						//solverBodyIndex is the globalThreadIndex for the active articulation in the block format
						const PxU32 blockIndex = solverBodyIndex / WARP_SIZE;

						PxgArticulationBlockData& articulation = artiData[blockIndex];
						PxgArticulationBlockLinkData* artiLinks = &artiLinkData[blockIndex * maxLinks];


						const PxU32 artiIndexInBlock = solverBodyIndex % WARP_SIZE;

						articulation.mStateDirty[artiIndexInBlock] = PxgArtiStateDirtyFlag::eHAS_IMPULSES;

						const PxU32 linkID = nodeId.articulationLinkId();

						PxReal denom = artiLinks[linkID].mDeltaScale[artiIndexInBlock];
						PxReal ratio = 1.f / denom;

						//for articulation, linVel and angVel accumulate impulse
						Cm::UnAlignedSpatialVector impulse;
						impulse.top = PxVec3(linVel.x, linVel.y, linVel.z);
						impulse.bottom = PxVec3(angVel.x, angVel.y, angVel.z);

						impulse.top *= ratio;
						impulse.bottom *= ratio;

						/*printf("blockIndex %i artiIndexInBlock %i linkID %i ratio %f impulse linear(%f, %f, %f) angular(%f, %f, %f)\n", blockIndex, artiIndexInBlock, linkID, ratio,
							impulse.top.x, impulse.top.y, impulse.top.z, impulse.bottom.x, impulse.bottom.y, impulse.bottom.z);*/

						atomicAddSpatialVector(artiLinks[linkID].mScratchImpulse, -impulse, artiIndexInBlock);
					}
					else
					{
						PxReal denom = tempDenom[solverBodyIndex];// globalRelaxationCoefficient;

						PxReal ratio = 1.f / denom;

						if (isTGS)
						{
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex].x, linVel.x*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex].y, linVel.y*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex].z, linVel.z*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex].w, angVel.x*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex + numSolverBodies].x, angVel.y*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex + numSolverBodies].y, angVel.z*ratio);
							//The rest is the delta position buffer
						}
						else
						{

							atomicAdd(&solverBodyDeltaVel[solverBodyIndex].x, linVel.x*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex].y, linVel.y*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex].z, linVel.z*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex + numSolverBodies].x, angVel.x*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex + numSolverBodies].y, angVel.y*ratio);
							atomicAdd(&solverBodyDeltaVel[solverBodyIndex + numSolverBodies].z, angVel.z*ratio);

						}
					}
					//printf("solverBodyIndex %i\n", solverBodyIndex);
					//printf("linearVelocity(%f, %f, %f, %f)\n", linearVelocity.x, linearVelocity.y, linearVelocity.z, linearVelocity.w);
					//printf("angularVelocity(%f, %f, %f, %f)\n", angularVelocity.x, angularVelocity.y, angularVelocity.z, angularVelocity.w);
				}
			}
		}
	}
}

