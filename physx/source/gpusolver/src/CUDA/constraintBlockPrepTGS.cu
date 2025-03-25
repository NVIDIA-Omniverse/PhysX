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

#include "PxgSolverBody.h"
#include "PxgConstraint.h"
#include "PxgFrictionPatch.h"
#include "PxgConstraintPrep.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgSolverCoreDesc.h"
#include "DyConstraintPrep.h"
#include "contactConstraintBlockPrep.cuh"
#include "jointConstraintBlockPrepTGS.cuh"


using namespace physx;

extern "C" __host__ void initSolverKernels10() {}

extern "C" __global__ void jointConstraintBlockPrepareParallelLaunchTGS( PxgConstraintPrepareDesc* solverDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc)
{
	//threadCounts[threadIdx.x] = 0;

	//__syncthreads();

	//PxgBlockWorkUnit* workUnits = constraintPrepDesc->workUnit;

	const PxU32 warpSize = 32;

	const PxU32 blockStride = blockDim.x/warpSize;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = blockIdx.x * blockStride + threadIdx.x/warpSize;

	//This identifies which thread within a warp a specific thread is
	const PxU32 threadIndexInWarp = threadIdx.x&(warpSize-1);

	//total numbers of warps in all blocks
	//const PxU32 totalNumWarps = blockStride * gridDim.x;

	//PxF32* baseForceStream = constraintPrepDesc->forceBuffer;

	PxgSolverBodyData* solverBodyDatas = solverDesc->solverBodyDataPool;
	PxgSolverTxIData* solverTxIData = solverDesc->solverBodyTxIDataPool;

	PxgTGSBlockSolverConstraint1DHeader* jointConstraintHeaders = sharedDesc->iterativeData.blockJointConstraintHeaders;
	PxgTGSBlockSolverConstraint1DCon* jointConstraintRowsCon = sharedDesc->iterativeData.blockJointConstraintRowsCon;
	//PxgBlockSolverConstraint1DMod* jointConstraintRowsMod = sharedDesc->iterativeData.blockJointConstraintRowsMod;
	PxU32* batchIndices = solverDesc->jointConstraintBatchIndices;
	const PxU32 total1dConstraintBatches = solverDesc->num1dConstraintBatches + solverDesc->numStatic1dConstraintBatches;


	//for(PxU32 i=warpIndex; i< constraintPrepDesc->num1dConstraintBatches; i+=totalNumWarps)
	PxU32 i = warpIndex;
	if (i < total1dConstraintBatches)
	{

		const PxU32 batchIndex = batchIndices[i];
		PxgBlockConstraintBatch& batch = sharedDesc->iterativeData.blockConstraintBatch[batchIndex];
		const PxU32 bodyAIndex = batch.bodyAIndex[threadIndexInWarp];
		const PxU32 bodyBIndex = batch.bodyBIndex[threadIndexInWarp];
			
		const PxU32 descIndexBatch = batch.mConstraintBatchIndex;

		const PxU32 descStride = batch.mDescStride;

		//PxgSolverBodyPrepData bodyData0, bodyData1;

#if LOAD_BODY_DATA
		loadBodyData(solverBodyDatas, descStride, bodyAIndex, threadIndexInWarp, warpIndexInBlock, bodyData0.initialLinVelXYZ_invMassW, bodyData0.initialAngVelXYZ_penBiasClamp,
			bodyData0.sqrtInvInertia, bodyData0.body2World);
		loadBodyData(solverBodyDatas, descStride, bodyBIndex, threadIndexInWarp, warpIndexInBlock, bodyData1.initialLinVelXYZ_invMassW, bodyData1.initialAngVelXYZ_penBiasClamp,
			bodyData1.sqrtInvInertia, bodyData1.body2World);
#endif


		//mDescStride might less than 32, we need to guard against it
		if(threadIndexInWarp < descStride)
		{
				//desc.descIndex for joint in fact is the batch index
			PxgBlockConstraint1DData& constraintData = solverDesc->blockJointPrepPool[descIndexBatch];
			PxgBlockConstraint1DVelocities* rowVelocities = &solverDesc->blockJointPrepPool0[descIndexBatch * Dy::MAX_CONSTRAINT_ROWS];
			PxgBlockConstraint1DParameters* rowParameters = &solverDesc->blockJointPrepPool1[descIndexBatch * Dy::MAX_CONSTRAINT_ROWS];

			PxgSolverBodyData* bodyData0 = &solverBodyDatas[bodyAIndex];
			PxgSolverBodyData* bodyData1 = &solverBodyDatas[bodyBIndex];
			PxgSolverTxIData* txIData0 = &solverTxIData[bodyAIndex];
			PxgSolverTxIData* txIData1 = &solverTxIData[bodyBIndex];

			PxU32 uniqueIndex = solverDesc->constraintUniqueIndices[batch.mStartPartitionIndex + threadIndexInWarp];
				
			setupSolverConstraintBlockGPUTGS<PxgKernelBlockDim::CONSTRAINT_PREPARE_BLOCK_PARALLEL>(&constraintData, rowVelocities, rowParameters, bodyData0, bodyData1, txIData0, txIData1, 
				sharedDesc->stepDt, sharedDesc->stepInvDtF32, sharedDesc->dt, sharedDesc->invDtF32, sharedDesc->lengthScale, 
				solverDesc->biasCoefficient, batch, threadIndexInWarp,
					&jointConstraintHeaders[descIndexBatch], &jointConstraintRowsCon[batch.startConstraintIndex],
					solverDesc->solverConstantData[uniqueIndex]);
		}    
	}
}

extern "C" __global__ void contactConstraintBlockPrepareParallelLaunchTGS(
	PxgConstraintPrepareDesc* constraintPrepDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc)
{
	//threadCounts[threadIdx.x] = 0;

	//__syncthreads();

	PxgBlockWorkUnit* workUnits = constraintPrepDesc->blockWorkUnit;

	const PxU32 warpSize = 32;

	const PxU32 blockStride = blockDim.x/warpSize;

	//This identifies which warp a specific thread is in, we treat all warps in all blocks as a flatten warp array
	//and we are going to index the work based on that
	const PxU32 warpIndex = blockIdx.x * blockStride + threadIdx.x/warpSize;

	//This identifies which thread within a warp a specific thread is
	const PxU32 threadIndexInWarp = threadIdx.x&(warpSize-1);

	const PxU32 totalPreviousEdges = constraintPrepDesc->totalPreviousEdges;
	const PxU32 totalCurrentEdges = constraintPrepDesc->totalCurrentEdges;
	const PxU32 nbContactBatches = constraintPrepDesc->numContactBatches + constraintPrepDesc->numStaticContactBatches;
	

	__shared__ PxgSolverBodyData* solverBodyDatas;
	__shared__ PxgSolverTxIData* solverTxIDatas;

	__shared__ PxgTGSBlockSolverContactHeader* contactHeaders;
	__shared__ PxgTGSBlockSolverFrictionHeader* frictionHeaders;
	__shared__ PxgTGSBlockSolverContactPoint* contactPoints;
	__shared__ PxgTGSBlockSolverContactFriction* frictions;
	__shared__ PxU32* batchIndices;
	__shared__ PxgBlockFrictionIndex* frictionIndices;
	__shared__ PxgBlockFrictionIndex* prevFrictionIndices;
	__shared__ PxgBlockContactPoint* contactBase;
	__shared__ PxgBlockConstraintBatch* constraintBatch;
	__shared__ PxgBlockContactData* contactCurrentPrepPool;
	__shared__ PxgBlockFrictionPatch* prevFrictionPatches;
	__shared__ PxgBlockFrictionPatch* currFrictionPatches;
	__shared__ PxgBlockFrictionAnchorPatch* prevFrictionAnchors;
	__shared__ PxgBlockFrictionAnchorPatch* currFrictionAnchors;
	__shared__ PxAlignedTransform* bodyFrames;

	volatile __shared__ char sInertias[sizeof(PxMat33) * (PxgKernelBlockDim::CONSTRAINT_PREPARE_BLOCK_PARALLEL / warpSize) * warpSize];
	volatile PxMat33* inertias = reinterpret_cast<volatile PxMat33*>(sInertias);

	//volatile __shared__ PxMat33 inertias[PxgKernelBlockDim::CONSTRAINT_PREPARE_BLOCK_PARALLEL / 32][32];

	if(threadIdx.x == 0)
	{
		solverBodyDatas = constraintPrepDesc->solverBodyDataPool;
		solverTxIDatas = constraintPrepDesc->solverBodyTxIDataPool;

		contactHeaders = sharedDesc->iterativeData.blockContactHeaders;
		frictionHeaders = sharedDesc->iterativeData.blockFrictionHeaders;
		contactPoints = sharedDesc->iterativeData.blockContactPoints;
		frictions = sharedDesc->iterativeData.blockFrictions;
		batchIndices = constraintPrepDesc->contactConstraintBatchIndices;
		frictionIndices = constraintPrepDesc->blockCurrentFrictionIndices;
		prevFrictionIndices = constraintPrepDesc->blockPreviousFrictionIndices;

		contactBase = constraintPrepDesc->blockContactPoints;
		constraintBatch = sharedDesc->iterativeData.blockConstraintBatch;
		contactCurrentPrepPool = constraintPrepDesc->blockContactCurrentPrepPool;
		currFrictionPatches = sharedDesc->blockCurrentFrictionPatches;
		prevFrictionPatches = sharedDesc->blockPreviousFrictionPatches;
		prevFrictionAnchors = constraintPrepDesc->blockPreviousAnchorPatches;
		currFrictionAnchors = constraintPrepDesc->blockCurrentAnchorPatches;
		bodyFrames = constraintPrepDesc->body2WorldPool;
	}
	
	__syncthreads();

	PxU32 i = warpIndex;
	//unsigned mask_nbContactBatches = __ballot_sync(FULL_MASK, i < nbContactBatches);
	if(i < nbContactBatches)
	{
		/*if (threadIndexInWarp == 0)
			printf("Processing batch %i\n", i);*/
		const PxU32 batchIndex = batchIndices[i];
		PxgBlockConstraintBatch& batch = constraintBatch[batchIndex];
		const PxU32 bodyAIndex = batch.bodyAIndex[threadIndexInWarp];
		const PxU32 bodyBIndex = batch.bodyBIndex[threadIndexInWarp];

		/*if (threadIndexInWarp == 0)
			printf("Processing batchIndex %i\n", batchIndex);*/
			
		const PxU32 descIndexBatch = batch.mConstraintBatchIndex;

		const PxU32 descStride = batch.mDescStride;

		//PxgSolverBodyPrepData bodyData0, bodyData1;

#if LOAD_BODY_DATA
		loadBodyData(solverBodyDatas, descStride, bodyAIndex, threadIndexInWarp, warpIndexInBlock, bodyData0.initialLinVelXYZ_invMassW, bodyData0.initialAngVelXYZ_penBiasClamp,
			bodyData0.sqrtInvInertia, bodyData0.body2World);
		loadBodyData(solverBodyDatas, descStride, bodyBIndex, threadIndexInWarp, warpIndexInBlock, bodyData1.initialLinVelXYZ_invMassW, bodyData1.initialAngVelXYZ_penBiasClamp,
			bodyData1.sqrtInvInertia, bodyData1.body2World);
#endif

		//Read in 16 bytes at a time, we take 3 threads to read in a single inertia tensor, and we have some spare bandwidth. We can read
		//32 inertia tensors in 3 passes

		const PxU32 descStride2 = descStride*2;

		/*if(threadIndexInWarp == 0)
			printf("Loading first txIData\n");*/
		for (PxU32 i = 0; i < descStride2; i += 32)
		{
			PxU32 idx = i + threadIndexInWarp;
			PxU32 bodyToLoad = idx/2;

			PxU32 bodyIdx = __shfl_sync(FULL_MASK, bodyAIndex, bodyToLoad);

			if (idx < descStride2)
			{
				PxU32 offset = idx & 1;
				float4* val = reinterpret_cast<float4*>(&solverTxIDatas[bodyIdx].sqrtInvInertia.column0.y);
				//volatile float* sh = reinterpret_cast<volatile float*>(&inertias[threadIdx.x / 32][bodyToLoad]);
				const PxU32 ind = (threadIdx.x / warpSize) * warpSize + bodyToLoad;
				volatile float* sh = reinterpret_cast<volatile float*>(&inertias[ind]);

				float4 v = val[offset];

				float v0 = solverTxIDatas[bodyIdx].sqrtInvInertia.column0.x;

				sh[1 + offset * 4] = v.x;
				sh[2 + offset * 4] = v.y;
				sh[3 + offset * 4] = v.z;
				sh[4 + offset * 4] = v.w;

				if (offset == 0)
					sh[offset * 4] = v0;
			}
		}

		__syncwarp();

		PxMat33 invInertia0;
		const PxU32 index = (threadIdx.x / warpSize) * warpSize + threadIndexInWarp;
		if (threadIndexInWarp < descStride)
		{
			invInertia0.column0.x = inertias[index].column0.x;
			invInertia0.column0.y = inertias[index].column0.y;
			invInertia0.column0.z = inertias[index].column0.z;
			invInertia0.column1.x = inertias[index].column1.x;
			invInertia0.column1.y = inertias[index].column1.y;
			invInertia0.column1.z = inertias[index].column1.z;
			invInertia0.column2.x = inertias[index].column2.x;
			invInertia0.column2.y = inertias[index].column2.y;
			invInertia0.column2.z = inertias[index].column2.z;

			//printf("%i: (%f, %f, %f) (%f, %f, %f) (%f, %f, %f)\n", threadIdx.x, invInertia0.column0.x, invInertia0.column0.y, invInertia0.column0.z, invInertia0.column1.x, invInertia0.column1.y, invInertia0.column1.z, invInertia0.column2.x, invInertia0.column2.y, invInertia0.column2.z);
		}

		__syncwarp(); //Required (racecheck confirmed) because inertias (Ptr sh points to inertias) is written below and read above

		/*if (threadIndexInWarp == 0)
			printf("Loading second txIData\n");*/
		for (PxU32 i = 0; i < descStride2; i += 32)
		{
			PxU32 idx = i + threadIndexInWarp;
			PxU32 bodyToLoad = idx / 2;

			PxU32 bodyIdx = __shfl_sync(FULL_MASK, bodyBIndex, bodyToLoad);

			if (idx < descStride2)
			{
				PxU32 offset = idx & 1;
				float4* val = reinterpret_cast<float4*>(&solverTxIDatas[bodyIdx].sqrtInvInertia.column0.y);
				const PxU32 ind = (threadIdx.x / warpSize) * warpSize + bodyToLoad;
				volatile float* sh = reinterpret_cast<volatile float*>(&inertias[ind]);

				float4 v = val[offset];

				float v0 = solverTxIDatas[bodyIdx].sqrtInvInertia.column0.x;

				sh[1 + offset * 4] = v.x;
				sh[2 + offset * 4] = v.y;
				sh[3 + offset * 4] = v.z;
				sh[4 + offset * 4] = v.w;

				if (offset == 0)
					sh[offset * 4] = v0;
			}
		}

		__syncwarp();

		/*if (threadIndexInWarp == 0)
			printf("Loaded second txIData\n");*/

		PxMat33 invInertia1;

		if (threadIndexInWarp < descStride)
		{
			invInertia1.column0.x = inertias[index].column0.x;
			invInertia1.column0.y = inertias[index].column0.y;
			invInertia1.column0.z = inertias[index].column0.z;
			invInertia1.column1.x = inertias[index].column1.x;
			invInertia1.column1.y = inertias[index].column1.y;
			invInertia1.column1.z = inertias[index].column1.z;
			invInertia1.column2.x = inertias[index].column2.x;
			invInertia1.column2.y = inertias[index].column2.y;
			invInertia1.column2.z = inertias[index].column2.z;
		}

		//mDescStride might less than 32, we need to guard against it
		if(threadIndexInWarp < descStride)
		{
			//port contact code
			PxgBlockContactData& contactData = contactCurrentPrepPool[descIndexBatch];
			PxgBlockContactPoint* baseContact = contactBase + batch.blockContactIndex;
			PxgBlockFrictionPatch& frictionPatch = currFrictionPatches[descIndexBatch];
			PxgBlockFrictionAnchorPatch& fAnchor = currFrictionAnchors[descIndexBatch];

			//Fill in correlation information for next frame...

			PxgBlockWorkUnit& unit = workUnits[descIndexBatch];

			PxgBlockFrictionIndex index;
			index.createPatchIndex(descIndexBatch, threadIndexInWarp);

			//PxU32 frictionIndex = unit.mFrictionIndex[threadIndexInWarp];
			PxU32 edgeIndex = unit.mEdgeIndex[threadIndexInWarp];
			PxU32 frictionIndex = edgeIndex + totalCurrentEdges * unit.mPatchIndex[threadIndexInWarp];
			PxgBlockFrictionIndex* targetIndex = &frictionIndices[frictionIndex];
				
			*reinterpret_cast<uint2*>(targetIndex) = reinterpret_cast<uint2&>(index);

			//KS - todo - get some of this in shared memory/registers as quickly as possible...
			PxgSolverBodyData* bodyData0 = &solverBodyDatas[bodyAIndex];
			PxgSolverBodyData* bodyData1 = &solverBodyDatas[bodyBIndex];
			//PxgSolverTxIData* txIData0 = &solverTxIDatas[bodyAIndex];
			//PxgSolverTxIData* txIData1 = &solverTxIDatas[bodyBIndex];

			const PxAlignedTransform bodyFrame0 = bodyFrames[bodyAIndex];
			const PxAlignedTransform bodyFrame1 = bodyFrames[bodyBIndex];

			//KS - temporarily read the velocities the "slow" way so we can store the inertia-scaled velocities 
			//in velocities buffer for now. We can then switch over later when we create the new prep code for the 
			//TGS solver and leave the PGS solver as-is
#if 0
			const float4 linVel_invMass0 = velocities[bodyAIndex];
			const float4 angVelXYZ_penBiasClamp0 = velocities[bodyAIndex + totalBodies];

			const float4 linVel_invMass1 = velocities[bodyBIndex];
			const float4 angVelXYZ_penBiasClamp1 = velocities[bodyBIndex + totalBodies];
#else

			//We use these velocities because these are not multiplied by sqrtInertia. This is a bit slower to read but 
			//means we can treat kinematics and statics the same in the below code.
			const float4 linVel_invMass0 = bodyData0->initialLinVelXYZ_invMassW;
			const float4 angVelXYZ_penBiasClamp0 = bodyData0->initialAngVelXYZ_penBiasClamp;

			const float4 linVel_invMass1 = bodyData1->initialLinVelXYZ_invMassW;
			const float4 angVelXYZ_penBiasClamp1 = bodyData1->initialAngVelXYZ_penBiasClamp;
#endif

			const PxReal offsetSlop = PxMax(bodyData0->offsetSlop, bodyData1->offsetSlop);

			PxU32 offset = unit.mWriteback[threadIndexInWarp];
			const float2 torsionalData = unit.mTorsionalFrictionData[threadIndexInWarp];
			createFinalizeSolverContactsBlockGPUTGS(&contactData, baseContact, frictionPatch, prevFrictionPatches, fAnchor, prevFrictionAnchors, prevFrictionIndices, *bodyData0, *bodyData1,
				invInertia0, invInertia1, bodyFrame0, bodyFrame1, linVel_invMass0, angVelXYZ_penBiasClamp0, linVel_invMass1, angVelXYZ_penBiasClamp1,
				sharedDesc->stepInvDtF32, sharedDesc->stepDt, sharedDesc->dt, sharedDesc->invDtF32, constraintPrepDesc->bounceThresholdF32, constraintPrepDesc->frictionOffsetThreshold, constraintPrepDesc->correlationDistance,
				constraintPrepDesc->biasCoefficient, threadIndexInWarp, offset, &contactHeaders[descIndexBatch], &frictionHeaders[descIndexBatch], &contactPoints[batch.startConstraintIndex], 
				&frictions[batch.startFrictionIndex], totalPreviousEdges, edgeIndex, constraintPrepDesc->ccdMaxSeparation, offsetSlop,
				torsionalData);

			frictionPatch.patchIndex[threadIndexInWarp] = unit.mFrictionPatchIndex[threadIndexInWarp];

			PxgBlockFrictionPatch& fpatch = frictionPatch;
			if (fpatch.anchorCount[threadIndexInWarp] >= 1)
				fpatch.anchorPoints[0][threadIndexInWarp] = bodyFrame0.transform(fAnchor.body0Anchors[0][threadIndexInWarp]);
			if (fpatch.anchorCount[threadIndexInWarp] == 2)
				fpatch.anchorPoints[1][threadIndexInWarp] = bodyFrame0.transform(fAnchor.body0Anchors[1][threadIndexInWarp]);
		}
	}
}
