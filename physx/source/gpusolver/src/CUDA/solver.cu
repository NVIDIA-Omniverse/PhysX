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

#include "PxgSolverBody.h"
#include "PxgSolverConstraintBlock1D.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgConstraint.h"
#include "PxgConstraintBlock.h"
#include "PxgIslandContext.h"
#include "PxgIntrinsics.h"
#include "PxgSolverCoreDesc.h"
#include "solver.cuh"
#include "solverBlock.cuh"
#include "PxgArticulation.h"
#include "assert.h"
#include "PxgArticulationCoreDesc.h"

using namespace physx;

extern "C" __host__ void initSolverKernels7() {}

extern "C" __global__
//__launch_bounds__(PxgKernelBlockDim::SOLVE_BLOCK_PARTITION, 8)
void artiSolveBlockPartition(PxgSolverCoreDesc* PX_RESTRICT solverDesc, const PxgSolverSharedDesc<IterativeSolveData>* PX_RESTRICT sharedDesc,
	const PxU32 islandIndex, const PxU32 partitionIndex, bool doFriction_, const PxgArticulationCoreDesc* const PX_RESTRICT artiDesc)
{
	const PxU32 warpIndex = threadIdx.y;
	PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;

	const PxgIslandContext& island = solverDesc->islandContextPool[islandIndex];

	//PxgBodySim* gBodySims = sharedDesc->bodySims;
	PxgArticulationBlockData* gArticulations = artiDesc->mArticulationBlocks;

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

	PxgErrorAccumulator error;
	const bool accumulateError = solverDesc->contactErrorAccumulator.mCounter >= 0;

	if (k < endIndex)
	{
		/*if(threadIndexInWarp == 0)
			printf("arti Contact Batch k = %i, endIndex = %i\n", k, endIndex);*/
		/*	if (threadIndexInWarp == 0)
		printf("============================================\n");*/

		const bool doFriction = doFriction_;

		const IterativeSolveData& msIterativeData = sharedDesc->iterativeData;

		const PxgBlockConstraintBatch& batch = msIterativeData.blockConstraintBatch[k];

		if(threadIndexInWarp >= batch.mDescStride)
			return;

		//printf("threadIndexInWarp %i descStride %i\n", threadIndexInWarp, batch.mDescStride);
		//printf("constraintBatchIndex %i startConstraintIndex %i", batch.mConstraintBatchIndex, batch.startConstraintIndex);
		const PxU32 nbArticulations = solverDesc->islandContextPool->mArticulationCount;

		const PxNodeIndex igNodeIndexA = batch.bodyANodeIndex[threadIndexInWarp];
		const PxNodeIndex igNodeIndexB = batch.bodyBNodeIndex[threadIndexInWarp];

		const PxU32 slabId = batch.slabId[threadIndexInWarp];

		const PxU32 nodeIndexA = igNodeIndexA.index();
		const PxU32 nodeIndexB = igNodeIndexB.index();

		const PxU32 solverBodyId0 = batch.bodyAIndex[threadIndexInWarp];
		const PxU32 solverBodyId1 = batch.bodyBIndex[threadIndexInWarp];

		const PxU32 readIndex = k * 128 + threadIndexInWarp;

		PxgArticulationBlockResponse* responses = sharedDesc->iterativeData.artiResponse;
		const PxU32 responseIndex = batch.mArticulationResponseIndex;

		//printf("responseIndex %i\n", responseIndex);

		//Cm::UnAlignedSpatialVector v0, v1;
		PxU32 linkIndexA = igNodeIndexA.articulationLinkId(); 
		PxU32 linkIndexB = igNodeIndexB.articulationLinkId();

		Cm::UnAlignedSpatialVector vel0, vel1;
		{
			float4 lin = Pxldcg(msIterativeData.solverBodyVelPool[readIndex]);
			float4 ang = Pxldcg(msIterativeData.solverBodyVelPool[readIndex + 32]);
			vel0 = Cm::UnAlignedSpatialVector(PxVec3(ang.x, ang.y, ang.z), PxVec3(lin.x, lin.y, lin.z));
		}

		{
			float4 lin = Pxldcg(msIterativeData.solverBodyVelPool[readIndex + 64]);
			float4 ang = Pxldcg(msIterativeData.solverBodyVelPool[readIndex + 96]);
			vel1 = Cm::UnAlignedSpatialVector(PxVec3(ang.x, ang.y, ang.z), PxVec3(lin.x, lin.y, lin.z));
		}

		Cm::UnAlignedSpatialVector impulse0 = Cm::UnAlignedSpatialVector::Zero();
		Cm::UnAlignedSpatialVector impulse1 = Cm::UnAlignedSpatialVector::Zero();

		PxReal curRef0 = 1.f;
		PxReal curRef1 = 1.f;

		const PxU32 bodyOffset = solverDesc->islandContextPool->mBodyStartIndex;
		const PxU32 numDynamicBodies = solverDesc->islandContextPool->mBodyCount; //nbBodies minus offset!

		const PxU32 numArticulations = solverDesc->islandContextPool->mArticulationCount;
		const PxU32 numSolverBodies = bodyOffset + numDynamicBodies + numArticulations;

		const PxU32* const PX_RESTRICT encodedReferenceCount = sharedDesc->iterativeData.solverEncodedReferenceCount;

		if(igNodeIndexA.isArticulation())
		{
			const PxU32 articulationBodyIdA = batch.remappedBodyAIndex[threadIndexInWarp];

			// Articulation IDs are at the back of rigid body IDs.
			const PxU32 globalBodyIdA = articulationBodyIdA + numDynamicBodies + bodyOffset;

			// Counting the number of active slabs
			curRef0 = static_cast<PxReal>(countActiveSlabs(globalBodyIdA, solverDesc->numSlabs, numSolverBodies, encodedReferenceCount));
		}
		else if(solverBodyId0 >= bodyOffset)
		{
			// Counting the number of active slabs
			curRef0 = static_cast<PxReal>(countActiveSlabs(solverBodyId0, solverDesc->numSlabs, numSolverBodies, encodedReferenceCount));
		}

		if(igNodeIndexB.isArticulation())
		{
			const PxU32 articulationBodyIdB = batch.remappedBodyBIndex[threadIndexInWarp];

			// Articulation IDs are at the back of rigid body IDs.
			const PxU32 globalBodyIdB = articulationBodyIdB + numDynamicBodies + bodyOffset;

			// Counting the number of active slabs
			curRef1 = static_cast<PxReal>(countActiveSlabs(globalBodyIdB, solverDesc->numSlabs, numSolverBodies, encodedReferenceCount));
		}
		else if(solverBodyId1 >= bodyOffset)
		{
			// Counting the number of active slabs
			curRef1 = static_cast<PxReal>(countActiveSlabs(solverBodyId1, solverDesc->numSlabs, numSolverBodies, encodedReferenceCount));
		}

		if(batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONTACT)
		{
			solveExtContactsBlock(batch, vel0, vel1, doFriction, msIterativeData.blockContactHeaders,
			                      msIterativeData.blockFrictionHeaders, msIterativeData.blockContactPoints,
			                      msIterativeData.blockFrictions, msIterativeData.artiResponse, impulse0, impulse1,
			                      threadIndexInWarp, accumulateError ? &error : NULL, curRef0, curRef1);
		}
		else
		{
			assert(batch.constraintType == PxgSolverConstraintDesc::eARTICULATION_CONSTRAINT_1D);
			solveExt1DBlock(batch, vel0, vel1, threadIndexInWarp, msIterativeData.blockJointConstraintHeaders,
			                msIterativeData.blockJointConstraintRowsCon, msIterativeData.blockJointConstraintRowsMod,
			                &responses[responseIndex], impulse0, impulse1,
			                solverDesc->contactErrorAccumulator.mCounter >= 0, curRef0, curRef1);
		}

		//Pull impulse from threads 6-12
		if (!igNodeIndexA.isArticulation())
		{
			const PxU32 outIndex = (batch.remappedBodyAIndex[threadIndexInWarp]);
			{
				msIterativeData.solverBodyVelPool[outIndex] = make_float4(vel0.bottom.x, vel0.bottom.y, vel0.bottom.z, 0.f);
				msIterativeData.solverBodyVelPool[outIndex+32] = make_float4(vel0.top.x, vel0.top.y, vel0.top.z, 0.f);
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

			/*printf("%i: Output ArticA index = %i, articId = %i (%f, %f, %f, %f, %f, %f)\n", threadIndexInWarp, index, solverBodyId0,
				impulse0.top.x, impulse0.top.y, impulse0.top.z, impulse0.bottom.x, impulse0.bottom.y, impulse0.bottom.z);*/

			//KS - TODO - let's see if we can skip *all* of this code below. It should be avoidable!
			isSlabDirty[solverBodyId0 + slabId*nbArticulations].x = linkIndexA; // this works because the articulations are enumerated first in solverBodyIds
			PxU32 articBlockId = solverBodyId0 / 32;
			gArticulations[articBlockId].mStateDirty[solverBodyId0&31] = PxgArtiStateDirtyFlag::eHAS_IMPULSES | PxgArtiStateDirtyFlag::eVEL_DIRTY;
		}

		if (!igNodeIndexB.isArticulation())
		{
			//const PxU32 indexB = (2 * batch.remappedBodyBIndex);
			const PxU32 outIndex = (batch.remappedBodyBIndex[threadIndexInWarp]);
			{
				msIterativeData.solverBodyVelPool[outIndex] = make_float4(vel1.bottom.x, vel1.bottom.y, vel1.bottom.z, 0.f);
				msIterativeData.solverBodyVelPool[outIndex + 32] = make_float4(vel1.top.x, vel1.top.y, vel1.top.z, 0.f);

				//printf("Output rigid B to %i\n", outIndex);
			}
		}
		else
		{
			//Write to GPU articulation!
			const PxU32 index = computeDeltaVIndex(nbArticulations, maxLinks, solverBodyId1, linkIndexB, slabId);
			//printf("B index %i\n", index);
			deferredZ[index] = impulse1;
			storeSpatialVector(deferredZ + index, impulse1);
			isSlabDirty[solverBodyId1 + slabId*nbArticulations].y = linkIndexB;
			PxU32 articBlockId = solverBodyId1 / 32;
			/*gArticulations[articBlockId].mJointDirty[solverBodyId1&31] = true;
			gArticulations[articBlockId].mHasInternalImpulses[solverBodyId1&31] = true;*/
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