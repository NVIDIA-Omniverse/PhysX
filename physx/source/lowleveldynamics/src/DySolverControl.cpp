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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxPreprocessor.h"
#include "foundation/PxAtomic.h"
#include "DySolverBody.h"
#include "DyThresholdTable.h"
#include "DySolverControl.h"
#include "DyArticulationPImpl.h"
#include "DySolverContext.h"
#include "DyCpuGpuArticulation.h"

namespace physx
{
namespace Dy
{
void solve1DBlock					(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactBlock				(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExtContactBlock			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExt1DBlock				(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContact_BStaticBlock		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactPreBlock			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactPreBlock_Static	(DY_PGS_SOLVE_METHOD_PARAMS);
void solve1D4_Block					(DY_PGS_SOLVE_METHOD_PARAMS);

void solve1DConcludeBlock				(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactConcludeBlock			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExtContactConcludeBlock		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExt1DConcludeBlock			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContact_BStaticConcludeBlock	(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactPreBlock_Conclude		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactPreBlock_ConcludeStatic(DY_PGS_SOLVE_METHOD_PARAMS);
void solve1D4Block_Conclude				(DY_PGS_SOLVE_METHOD_PARAMS);

void solve1DBlockWriteBack				(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactBlockWriteBack			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExtContactBlockWriteBack		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExt1DBlockWriteBack			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContact_BStaticBlockWriteBack	(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactPreBlock_WriteBack		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactPreBlock_WriteBackStatic(DY_PGS_SOLVE_METHOD_PARAMS);
void solve1D4Block_WriteBack			(DY_PGS_SOLVE_METHOD_PARAMS);

// PT: not sure what happened, these ones were declared but not actually used
//void writeBack1DBlock				(DY_PGS_SOLVE_METHOD_PARAMS);
//void contactBlockWriteBack		(DY_PGS_SOLVE_METHOD_PARAMS);
//void extContactBlockWriteBack		(DY_PGS_SOLVE_METHOD_PARAMS);
//void ext1DBlockWriteBack			(DY_PGS_SOLVE_METHOD_PARAMS);
//void contactPreBlock_WriteBack	(DY_PGS_SOLVE_METHOD_PARAMS);
//void writeBack1D4Block			(DY_PGS_SOLVE_METHOD_PARAMS);

SolveBlockMethod gVTableSolveBlock[] PX_UNUSED_ATTRIBUTE = 
{
	0,
	solveContactBlock,				// DY_SC_TYPE_RB_CONTACT
	solve1DBlock,					// DY_SC_TYPE_RB_1D
	solveExtContactBlock,			// DY_SC_TYPE_EXT_CONTACT
	solveExt1DBlock,				// DY_SC_TYPE_EXT_1D
	solveContact_BStaticBlock,		// DY_SC_TYPE_STATIC_CONTACT
	solveContactPreBlock,			// DY_SC_TYPE_BLOCK_RB_CONTACT
	solveContactPreBlock_Static,	// DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT
	solve1D4_Block,					// DY_SC_TYPE_BLOCK_1D,
};

SolveWriteBackBlockMethod gVTableSolveWriteBackBlock[] PX_UNUSED_ATTRIBUTE = 
{
	0,
	solveContactBlockWriteBack,				// DY_SC_TYPE_RB_CONTACT
	solve1DBlockWriteBack,					// DY_SC_TYPE_RB_1D
	solveExtContactBlockWriteBack,			// DY_SC_TYPE_EXT_CONTACT
	solveExt1DBlockWriteBack,				// DY_SC_TYPE_EXT_1D
	solveContact_BStaticBlockWriteBack,		// DY_SC_TYPE_STATIC_CONTACT
	solveContactPreBlock_WriteBack,			// DY_SC_TYPE_BLOCK_RB_CONTACT
	solveContactPreBlock_WriteBackStatic,	// DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT
	solve1D4Block_WriteBack,				// DY_SC_TYPE_BLOCK_1D,
};

SolveBlockMethod gVTableSolveConcludeBlock[] PX_UNUSED_ATTRIBUTE = 
{
	0,
	solveContactConcludeBlock,				// DY_SC_TYPE_RB_CONTACT
	solve1DConcludeBlock,					// DY_SC_TYPE_RB_1D
	solveExtContactConcludeBlock,			// DY_SC_TYPE_EXT_CONTACT
	solveExt1DConcludeBlock,				// DY_SC_TYPE_EXT_1D
	solveContact_BStaticConcludeBlock,		// DY_SC_TYPE_STATIC_CONTACT
	solveContactPreBlock_Conclude,			// DY_SC_TYPE_BLOCK_RB_CONTACT
	solveContactPreBlock_ConcludeStatic,	// DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT
	solve1D4Block_Conclude,					// DY_SC_TYPE_BLOCK_1D,
};

struct SolverDt
{
	PxReal simDt;
	PxReal stepDt;
	PxReal invStepDt; 
};

constexpr bool tWriteBackInternalConstraints = true;
constexpr bool tResetPosError =  true;
constexpr bool tResetVelError = true;
constexpr bool tIsVelIter = true;

template<bool isVelocityIteration, bool writeBackInternalConstraints, bool resetPosIter, bool resetVelIter>
void solveArticulations
(ArticulationSolverDesc* articulationListStart, const PxU32 articulationListSize,
 const SolverDt& solverDt,
 const ArticulationConstraintProcessingConfigCPU& articulationConstraintProcessingConfig,
 const PxReal biasCoefficient,
 const bool residualReportingActive)
{
	for (PxU32 i = 0; i < articulationListSize; ++i)
	{
		if(resetPosIter)
		{
			articulationListStart[i].articulation->mInternalErrorAccumulatorPosIter.reset();
			articulationListStart[i].articulation->mContactErrorAccumulatorPosIter.reset();
		}
		if(resetVelIter)
		{
			articulationListStart[i].articulation->mInternalErrorAccumulatorVelIter.reset();
			articulationListStart[i].articulation->mContactErrorAccumulatorVelIter.reset();
		}
		articulationListStart[i].articulation->solveInternalConstraints(
			solverDt.simDt, solverDt.stepDt, solverDt.invStepDt, 
			isVelocityIteration, false, 
			articulationConstraintProcessingConfig,
			0.f, 
			biasCoefficient, 
			residualReportingActive);

		if(writeBackInternalConstraints)
		{
			articulationListStart[i].articulation->writebackInternalConstraints(false);
		}
	}
}

template<bool isVelocityIteration, bool writeBackInternalConstraints, bool resetPosError, bool resetVelError>
void processSolverIterationBlock
(const SolverDt& solverDt, const bool residualReportingActive,
//solve artics:
 const bool solveArticulationContactLast,
 ArticulationSolverDesc* articulationListStart, const PxU32 articulationListSize,
 const PxReal articulationBiasCoefficient,
 //solve rbodies:
 const PxSolverConstraintDesc* PX_RESTRICT constraintList, PxI32 batchCount, const PxI32 headerCount, 
 SolverContext& cache, BatchIterator& contactIterator,
 SolveBlockMethod solveTable[],
 PxI32 normalIter)
{
	if(solveArticulationContactLast)
	{
		const ArticulationConstraintProcessingConfigCPU firstPassArticulationConstraintProcessingConfig = ArticulationConstraintProcessingConfigCPU::getFirstPassConfig();
		const ArticulationConstraintProcessingConfigCPU secondPassArticulationConstraintProcessingConfig = ArticulationConstraintProcessingConfigCPU::getSecondPassConfig();

		solveArticulations<isVelocityIteration, false, resetPosError, resetVelError>(
				articulationListStart, articulationListSize, 
				solverDt,
				firstPassArticulationConstraintProcessingConfig,
				articulationBiasCoefficient, 
				residualReportingActive);

		SolveBlockParallel(constraintList, batchCount, normalIter * batchCount, headerCount, 
			cache, contactIterator, solveTable, normalIter);

		solveArticulations<isVelocityIteration, writeBackInternalConstraints, false, false>(
				articulationListStart, articulationListSize, 
				solverDt,
				secondPassArticulationConstraintProcessingConfig,
				articulationBiasCoefficient, 
				residualReportingActive);
	}
	else
	{
		const ArticulationConstraintProcessingConfigCPU singlePassArticulationConstraintProcessingConfig = ArticulationConstraintProcessingConfigCPU::getSinglePassConfig(solveArticulationContactLast);

		SolveBlockParallel(constraintList, batchCount, normalIter * batchCount, headerCount, 
			cache, contactIterator, solveTable, normalIter);

		solveArticulations<isVelocityIteration, writeBackInternalConstraints, resetPosError, resetVelError>(
				articulationListStart, articulationListSize, 
				solverDt,
				singlePassArticulationConstraintProcessingConfig,
				articulationBiasCoefficient, 
				residualReportingActive);
	}
}

void solveV_Blocks(SolverIslandParams& params, bool solveFrictionEveryIteration, bool solveArticulationContactLast)
{
	const bool residualReportingActive = params.errorAccumulator != NULL;

	const PxI32 TempThresholdStreamSize = 32;
	ThresholdStreamElement tempThresholdStream[TempThresholdStreamSize];

	SolverContext cache;
	cache.solverBodyArray			= params.bodyDataList;
	cache.mThresholdStream			= tempThresholdStream;
	cache.mThresholdStreamLength	= TempThresholdStreamSize;
	cache.mThresholdStreamIndex		= 0;
	cache.writeBackIteration		= false;
	cache.deltaV					= params.deltaV;

	const PxI32 batchCount = PxI32(params.numConstraintHeaders);

	const PxSolverBody* PX_RESTRICT bodyListStart = params.bodyListStart;
	const PxU32 bodyListSize = params.bodyListSize;

	Cm::SpatialVector* PX_RESTRICT motionVelocityArray = params.motionVelocityArray;

	const PxU32 velocityIterations = params.velocityIterations;
	const PxU32 positionIterations = params.positionIterations;

	const PxF32 articulationBiasCoefficient = computeArticulationBiasCoefficient<false>(positionIterations);

	const PxU32 numConstraintHeaders = params.numConstraintHeaders;
	const PxU32 articulationListSize = params.articulationListSize;

	ArticulationSolverDesc* PX_RESTRICT articulationListStart = params.articulationListStart;

	PX_ASSERT(positionIterations >= 1);

	if(numConstraintHeaders == 0)
	{
		solveNoContactsCase(bodyListSize, bodyListStart, motionVelocityArray,
							articulationListSize, articulationListStart, cache.deltaV,
							positionIterations, velocityIterations, params.dt, params.invDt, articulationBiasCoefficient, residualReportingActive, solveArticulationContactLast);
		return;
	}

	BatchIterator contactIterator(params.constraintBatchHeaders, params.numConstraintHeaders);

	const PxSolverConstraintDesc* PX_RESTRICT constraintList = params.constraintList;

	const SolverDt solverDt = {params.dt, params.dt, params.invDt};

	//0-(n-1) iterations
	PxI32 normalIter = 0;

	cache.isPositionIteration = true;
	cache.contactErrorAccumulator = residualReportingActive ? &params.errorAccumulator->mPositionIterationErrorAccumulator : NULL;
	for (PxU32 iteration = positionIterations; iteration > 0; iteration--)	//decreasing positive numbers == position iters
	{
		if (cache.contactErrorAccumulator)
			cache.contactErrorAccumulator->reset();

		cache.doFriction = solveFrictionEveryIteration ? true : iteration <= 3;

		processSolverIterationBlock<!tIsVelIter, !tWriteBackInternalConstraints, tResetPosError, !tResetVelError>
			(solverDt, residualReportingActive,
			 //solve artics:
			 solveArticulationContactLast,
			 articulationListStart, articulationListSize,
			 articulationBiasCoefficient,
			 //solve rbodies:
			 constraintList,  batchCount, batchCount, 
			 cache, contactIterator,
			 iteration == 1 ? gVTableSolveConcludeBlock : gVTableSolveBlock, normalIter);

		++normalIter;
	}

	saveMotionVelocities(bodyListSize, bodyListStart, motionVelocityArray);
	
	for (PxU32 i = 0; i < articulationListSize; i++)
		ArticulationPImpl::saveVelocity(articulationListStart[i].articulation, cache.deltaV);

	const PxI32 velItersMinOne = (PxI32(velocityIterations)) - 1;

	cache.isPositionIteration = false;
	cache.contactErrorAccumulator = residualReportingActive ? &params.errorAccumulator->mVelocityIterationErrorAccumulator : NULL;
	for(PxI32 iteration = 0; iteration < velItersMinOne; ++iteration)
	{
		if (cache.contactErrorAccumulator)
			cache.contactErrorAccumulator->reset();

		processSolverIterationBlock<tIsVelIter, !tWriteBackInternalConstraints, !tResetPosError, !tResetVelError>
			(solverDt, residualReportingActive,
			 //solve artics:
			 solveArticulationContactLast,
			 articulationListStart, articulationListSize,
			 articulationBiasCoefficient,
			 //solve rbodies:
			 constraintList,  batchCount, batchCount, 
			 cache, contactIterator,
			 gVTableSolveBlock, normalIter);

		++normalIter;
	}

	PxI32* outThresholdPairs = params.outThresholdPairs;
	ThresholdStreamElement* PX_RESTRICT thresholdStream = params.thresholdStream;
	PxU32 thresholdStreamLength = params.thresholdStreamLength;

	cache.writeBackIteration = true;	
	cache.mSharedThresholdStream = thresholdStream;
	cache.mSharedThresholdStreamLength = thresholdStreamLength;
	cache.mSharedOutThresholdPairs = outThresholdPairs;
	//PGS solver always runs at least one velocity iteration (otherwise writeback won't happen)
	{
		if (cache.contactErrorAccumulator)
			cache.contactErrorAccumulator->reset();

		processSolverIterationBlock<tIsVelIter, tWriteBackInternalConstraints, !tResetPosError, tResetVelError>
			(solverDt, residualReportingActive,
			 //solve artics:
			 solveArticulationContactLast,
			 articulationListStart, articulationListSize,
			 articulationBiasCoefficient,
			 //solve rbodies:
			 constraintList,  batchCount, batchCount, 
			 cache, contactIterator,
			 gVTableSolveWriteBackBlock, normalIter);

		++normalIter;
	}

	//Write back remaining threshold streams
	if(cache.mThresholdStreamIndex > 0)
	{
		//Write back to global buffer
		const PxI32 threshIndex = PxAtomicAdd(outThresholdPairs, PxI32(cache.mThresholdStreamIndex)) - PxI32(cache.mThresholdStreamIndex);
		for(PxU32 b = 0; b < cache.mThresholdStreamIndex; ++b)
		{
			thresholdStream[b + threshIndex] = cache.mThresholdStream[b];
		}
		cache.mThresholdStreamIndex = 0;
	}
}

static void solveVBlockParallelPartition
( const PxU32 headersInPartition,
 const PxI32 normalIteration, const PxI32 unrollCount, const  PxI32 batchCount, 
 const PxSolverConstraintDesc* PX_RESTRICT constraintList, 
 SolverContext& cache, BatchIterator& contactIter,
 SolveBlockMethod* solveTable,
 PxI32& maxNormalIndex, PxI32& index, PxI32& endIndexCount, PxI32& targetConstraintIndex,
 PxI32* constraintIndex, PxI32* constraintIndexCompleted)
{
	maxNormalIndex += headersInPartition;
				
	PxI32 nbSolved = 0;
	while(index < maxNormalIndex)
	{
		const PxI32 remainder = PxMin(maxNormalIndex - index, endIndexCount);
		SolveBlockParallel(constraintList, remainder, index, batchCount, cache, contactIter, solveTable, 
			normalIteration);
		index += remainder;
		endIndexCount -= remainder;
		nbSolved += remainder;
		if(endIndexCount == 0)
		{
			endIndexCount = unrollCount;
			index = PxAtomicAdd(constraintIndex, unrollCount) - unrollCount;
		}
	}
	if(nbSolved)
	{
		PxMemoryBarrier();
		PxAtomicAdd(constraintIndexCompleted, nbSolved);
	}
	targetConstraintIndex += headersInPartition; //Increment target constraint index by batch count
}

static void solveVBlockParallelPartitionsAndWaitOnCompletion
(const PxU32 nbPartitions, const PxU32* headersPerPartition,
 const PxI32 normalIteration, const PxI32 unrollCount, const  PxI32 batchCount, 
 const PxSolverConstraintDesc* PX_RESTRICT constraintList, 
 SolverContext& cache, BatchIterator& contactIter,
 SolveBlockMethod* solveTable,
 PxI32& maxNormalIndex, PxI32& index, PxI32& endIndexCount,  PxI32& targetConstraintIndex,
 PxI32* constraintIndex, PxI32* constraintIndexCompleted)
{
	for(PxU32 b = 0; b < nbPartitions; ++b)
	{
		solveVBlockParallelPartition(
				headersPerPartition[b],
				normalIteration, unrollCount, batchCount, 
				constraintList, 
				cache, contactIter,
				solveTable,
				maxNormalIndex, index, endIndexCount, targetConstraintIndex,
				constraintIndex, constraintIndexCompleted);
		WAIT_FOR_PROGRESS(constraintIndexCompleted, targetConstraintIndex); // wait for this rigid partition to be done
	}
}

template<bool writeBackInternalConstraints, bool isVelIter, bool resetPosError, bool resetVelError>
void solveInternalConstraintsAndWaitForCompletion
(const PxI32 articulationListSize, const PxI32 ArticCount,
 const ArticulationSolverDesc* PX_RESTRICT articulationListStart,
 const SolverDt& solverDt, 
 const ArticulationConstraintProcessingConfigCPU& articulationConstraintProcessingConfig, 
 const PxReal biasCoefficient, 
 const bool residualReportingActive, 
 PxI32& maxArticIndex, PxI32& targetArticIndex, PxI32& articSolveStart, PxI32& articIndexCounter, PxI32& articSolveEnd,
 PxI32* articIndexCompleted, PxI32* articIndex)
{
	maxArticIndex += articulationListSize;
	targetArticIndex += articulationListSize;

	//We are definitely executing PGS here.
	//elapsedTime is always 0.0f for PGS because we not 
	//advance time during pos iters.
	constexpr bool isTGS = false;
	constexpr PxReal elapsedTime = 0.0f;

	while (articSolveStart < maxArticIndex)
	{
		const PxI32 endIdx = PxMin(articSolveEnd, maxArticIndex);

		PxI32 nbSolved = 0;
		while (articSolveStart < endIdx)
		{
			if(resetPosError)
			{
				articulationListStart[articSolveStart - articIndexCounter].articulation->mInternalErrorAccumulatorPosIter.reset();
				articulationListStart[articSolveStart - articIndexCounter].articulation->mContactErrorAccumulatorPosIter.reset();
			}
			if(resetVelError)
			{
				articulationListStart[articSolveStart - articIndexCounter].articulation->mInternalErrorAccumulatorVelIter.reset();
				articulationListStart[articSolveStart - articIndexCounter].articulation->mContactErrorAccumulatorVelIter.reset();
			}
			articulationListStart[articSolveStart - articIndexCounter].articulation->solveInternalConstraints(
				solverDt.simDt, solverDt.stepDt, solverDt.invStepDt, 
				isVelIter, isTGS, 
				articulationConstraintProcessingConfig,
				elapsedTime,
				biasCoefficient, 
				residualReportingActive);
			if(writeBackInternalConstraints)
			{
				articulationListStart[articSolveStart - articIndexCounter].articulation->writebackInternalConstraints(false);
			}
			articSolveStart++;
			nbSolved++;
		}

		if (nbSolved)
		{
			PxMemoryBarrier();
			PxAtomicAdd(articIndexCompleted, nbSolved);
		}

		const PxI32 remaining = articSolveEnd - articSolveStart;

		if (remaining == 0)
		{
			articSolveStart = PxAtomicAdd(articIndex, ArticCount) - ArticCount;
			articSolveEnd = articSolveStart + ArticCount;
		}
	}

	articIndexCounter += articulationListSize;

	WAIT_FOR_PROGRESS(articIndexCompleted, targetArticIndex); 
}

void saveVelocitiesAndWaitForCompletion
(const PxI32 articulationListSize, const PxI32 saveUnrollCount, const PxI32 bodyListSize, 
 const PxSolverBody* PX_RESTRICT bodyListStart, const ArticulationSolverDesc* PX_RESTRICT articulationListStart, 
 Cm::SpatialVector* PX_RESTRICT motionVelocityArray,
 SolverContext& cache, 
 PxI32* bodyListIndex, PxI32* bodyListIndexCompleted)
{
	PxI32 endIndexCount2 = saveUnrollCount;
	PxI32 index2 = PxAtomicAdd(bodyListIndex, saveUnrollCount) - saveUnrollCount;
	{
		PxI32 nbConcluded = 0;
		while(index2 < articulationListSize)
		{
			const PxI32 remainder = PxMin(saveUnrollCount, (articulationListSize - index2));
			endIndexCount2 -= remainder;
			for(PxI32 b = 0; b < remainder; ++b, ++index2)
			{
				ArticulationPImpl::saveVelocity(articulationListStart[index2].articulation, cache.deltaV);
			}
			if(endIndexCount2 == 0)
			{
				index2 = PxAtomicAdd(bodyListIndex, saveUnrollCount) - saveUnrollCount;
				endIndexCount2 = saveUnrollCount;
			}
			nbConcluded += remainder;
		}

		index2 -= articulationListSize;

		//save velocity
		
		while(index2 < bodyListSize)
		{
			const PxI32 remainder = PxMin(endIndexCount2, (bodyListSize - index2));
			endIndexCount2 -= remainder;

			saveMotionVelocities(remainder, bodyListStart + index2, motionVelocityArray + index2);
			index2 += remainder;

			nbConcluded += remainder;
			
			//Branch not required because this is the last time we use this atomic variable
			//if(index2 < articulationListSizePlusbodyListSize)
			{
				index2 = PxAtomicAdd(bodyListIndex, saveUnrollCount) - saveUnrollCount - articulationListSize;
				endIndexCount2 = saveUnrollCount;
			}
		}

		if(nbConcluded)
		{
			PxMemoryBarrier();
			PxAtomicAdd(bodyListIndexCompleted, nbConcluded);
		}
	}

	WAIT_FOR_PROGRESS(bodyListIndexCompleted, (bodyListSize + articulationListSize)); // wait for all velocity saves to be done
}

template<bool writeBackInternalConstraints, bool isVelIter>
void processSolverIterationParallel
(//solve artics:
 const bool solveArticulationContactLast, 
 const PxI32 articulationListSize, const PxI32 articCount,
 const ArticulationSolverDesc* PX_RESTRICT articulationListStart,
 const SolverDt& solverDt, 
 const PxReal articulationBiasCoefficient, 
 const bool residualReportingActive, 
 PxI32& maxArticIndex, PxI32& targetArticIndex, PxI32& articSolveStart, PxI32& articIndexCounter, PxI32& articSolveEnd,
 PxI32* articIndexCompleted, PxI32* articIndex,
 //solve rbodies:
 const PxU32 nbPartitions, const PxU32* headersPerPartition,
 const PxI32 normalIteration, const PxI32 unrollCount, const  PxI32 batchCount, 
 const PxSolverConstraintDesc* PX_RESTRICT constraintList, 
 SolverContext& cache, BatchIterator& contactIter,
 SolveBlockMethod* solveTable,
 PxI32& maxNormalIndex, PxI32& index, PxI32& endIndexCount,  PxI32& targetConstraintIndex,
 PxI32* constraintIndex, PxI32* constraintIndexCompleted)
{
	//We report the pos(vel) error that accumulates across the last pos(vel) iter.
	//Ideally, we'd only reset the pos(vel) error at the beginning of the very last 
	//pos(vel) iter but we don't know if we are on the last pos(vel) iter here.
	//Instead, we just reset the pos(vel) iter at the start of each pos(vel) iteration.
	//We do, however, know if we are performing a pos or vel iter.
	//If we are doing pos iters then reset the pos error.  
	//Likewise, if we are doing vel iters then reset the vel error.
	constexpr bool resetPosError = !isVelIter;
	constexpr bool resetVelError = isVelIter;

	if(solveArticulationContactLast)
	{
		const ArticulationConstraintProcessingConfigCPU firstPassArticulationConstraintProcessingConfig = ArticulationConstraintProcessingConfigCPU::getFirstPassConfig();
		const ArticulationConstraintProcessingConfigCPU secondPassArticulationConstraintProcessingConfig = ArticulationConstraintProcessingConfigCPU::getSecondPassConfig();

		//Solve articulation internal constraints 1st pass and wait on completion.
		solveInternalConstraintsAndWaitForCompletion<false, isVelIter, resetPosError, resetVelError>(	 
			articulationListSize, articCount,
			articulationListStart,
			solverDt,
			firstPassArticulationConstraintProcessingConfig,
			articulationBiasCoefficient, 
			residualReportingActive, 
			maxArticIndex, targetArticIndex, articSolveStart, articIndexCounter, articSolveEnd, 
			articIndexCompleted, articIndex);

		//Solve each partition and wait for the last partition to complete.
		solveVBlockParallelPartitionsAndWaitOnCompletion(
			nbPartitions, headersPerPartition,
			normalIteration, unrollCount, batchCount, 
			constraintList, 
			cache, contactIter,
			solveTable,
			maxNormalIndex, index, endIndexCount, targetConstraintIndex,
			constraintIndex, constraintIndexCompleted);


		//Solve articulation internal constraints 2nd pass and wait on completion.
		solveInternalConstraintsAndWaitForCompletion<writeBackInternalConstraints, isVelIter, false, false>(	 
			articulationListSize, articCount,
			articulationListStart,
			solverDt,
			secondPassArticulationConstraintProcessingConfig,
			articulationBiasCoefficient, 
			residualReportingActive, 
			maxArticIndex, targetArticIndex, articSolveStart, articIndexCounter, articSolveEnd, 
			articIndexCompleted, articIndex);
	}
	else
	{
		const ArticulationConstraintProcessingConfigCPU singlePassArticulationConstraintProcessingConfig = ArticulationConstraintProcessingConfigCPU::getSinglePassConfig(solveArticulationContactLast);

		//Solve each partition and wait for the last partiton to complete.
		solveVBlockParallelPartitionsAndWaitOnCompletion(
			nbPartitions, headersPerPartition,
			normalIteration, unrollCount, batchCount, 
			constraintList, 
			cache, contactIter,
			solveTable,
			maxNormalIndex, index, endIndexCount, targetConstraintIndex,
			constraintIndex, constraintIndexCompleted);

		//Solve articulation internal constraints single pass and wait on completion.
		solveInternalConstraintsAndWaitForCompletion<writeBackInternalConstraints, isVelIter, resetPosError, resetVelError>(	 
			articulationListSize, articCount,
			articulationListStart,
			solverDt,
			singlePassArticulationConstraintProcessingConfig,
			articulationBiasCoefficient, 
			residualReportingActive, 
			maxArticIndex, targetArticIndex, articSolveStart, articIndexCounter, articSolveEnd, 
			articIndexCompleted, articIndex);
	}
}

void solveVParallelAndWriteBack(SolverIslandParams& params, Cm::SpatialVectorF* deltaV, Dy::ErrorAccumulatorEx* errorAccumulator,
	bool solveFrictionEveryIteration, bool solveArticulationContactLast)
{
#if PX_PROFILE_SOLVE_STALLS
	PxU64 startTime = readTimer();

	PxU64 stallCount = 0;
#endif
	const bool residualReportingActive = errorAccumulator != NULL;

	SolverContext cache;
	cache.solverBodyArray = params.bodyDataList;
	const PxU32 batchSize = params.batchSize;

	const PxI32 unrollCount = PxI32(batchSize);
	const PxI32 articCount = 2;
	const PxI32 saveUnrollCount = 32;

	const PxI32 tempThresholdStreamSize = 32;
	ThresholdStreamElement tempThresholdStream[tempThresholdStreamSize];

	const PxI32 bodyListSize = PxI32(params.bodyListSize);
	const PxI32 articulationListSize = PxI32(params.articulationListSize);

	const PxI32 batchCount = PxI32(params.numConstraintHeaders);
	cache.mThresholdStream = tempThresholdStream;
	cache.mThresholdStreamLength = tempThresholdStreamSize;
	cache.mThresholdStreamIndex = 0;
	cache.writeBackIteration = false;
	cache.deltaV = deltaV;

	const PxI32 positionIterations = PxI32(params.positionIterations);

	const PxF32 articulationBiasCoefficient = computeArticulationBiasCoefficient<false>(positionIterations);

	PxI32* constraintIndex = &params.constraintIndex; // counter for distributing constraints to tasks, incremented before they're solved
	PxI32* constraintIndexCompleted = &params.constraintIndexCompleted; // counter for completed constraints, incremented after they're solved

	PxI32* articIndex = &params.articSolveIndex;
	PxI32* articIndexCompleted = &params.articSolveIndexCompleted;

	const PxSolverConstraintDesc* PX_RESTRICT constraintList = params.constraintList;

	const ArticulationSolverDesc* PX_RESTRICT articulationListStart = params.articulationListStart;

	const PxU32 nbPartitions = params.nbPartitions;	

	const PxU32* headersPerPartition = params.headersPerPartition;

	PX_ASSERT(positionIterations >= 1);

	PxI32 endIndexCount = unrollCount;
	PxI32 index = PxAtomicAdd(constraintIndex, unrollCount) - unrollCount;

	PxI32 articSolveStart = 0;
	PxI32 articSolveEnd = 0;
	PxI32 maxArticIndex = 0;
	PxI32 articIndexCounter = 0;
	
	BatchIterator contactIter(params.constraintBatchHeaders, params.numConstraintHeaders);

	PxI32 maxNormalIndex = 0;
	PxI32 normalIteration = 0;
	PxU32 a = 0;
	PxI32 targetConstraintIndex = 0;
	PxI32 targetArticIndex = 0;

	const SolverDt solverDt = {params.dt, params.dt, params.invDt};	

	//Run all position iterations with:
	//gVTableSolveConcludeBlock on the last position iteration
	//gVTableSolveBlock on all prior iterations.
	cache.contactErrorAccumulator = residualReportingActive ? &errorAccumulator->mPositionIterationErrorAccumulator : NULL;
	cache.isPositionIteration = true;
	for(PxU32 i = 0; i < 2; ++i)
	{
		SolveBlockMethod* solveTable = i == 0 ? gVTableSolveBlock : gVTableSolveConcludeBlock;
		for(; a < positionIterations - 1 + i; ++a)
		{
			if (i == 0 && cache.contactErrorAccumulator)
				cache.contactErrorAccumulator->reset();

			cache.doFriction = solveFrictionEveryIteration ? true : (positionIterations - a) <= 3;

			processSolverIterationParallel<!tWriteBackInternalConstraints, !tIsVelIter>
				(//solve artics:
				 solveArticulationContactLast, 
				 articulationListSize, articCount,
				 articulationListStart,
				 solverDt,
				 articulationBiasCoefficient, 
				 residualReportingActive, 
				 maxArticIndex, targetArticIndex, articSolveStart, articIndexCounter, articSolveEnd, 
				 articIndexCompleted, articIndex,
				 //solve rbodies:
				 nbPartitions, headersPerPartition,
				 normalIteration, unrollCount, batchCount, 
				 constraintList, 
				 cache, contactIter,
				 solveTable,
				 maxNormalIndex, index, endIndexCount, targetConstraintIndex,
				 constraintIndex, constraintIndexCompleted);

			++normalIteration;
		}
	}

	//Save articulation velocities.
	saveVelocitiesAndWaitForCompletion
		(articulationListSize, saveUnrollCount, bodyListSize, params.bodyListStart,
		 articulationListStart, params.motionVelocityArray,
		 cache, 
		 &params.bodyListIndex, &params.bodyListIndexCompleted);

	
	cache.contactErrorAccumulator = residualReportingActive ? &errorAccumulator->mVelocityIterationErrorAccumulator : NULL;
	cache.isPositionIteration = false;

	//Perform (nbVelIters -1) velocity iterations.
	//We'll run a final velocity iteration later to make sure that we always run at least 1.
	a = 1;
	for(; a < params.velocityIterations; ++a)
	{
		if (residualReportingActive)
			cache.contactErrorAccumulator->reset();

		processSolverIterationParallel<!tWriteBackInternalConstraints, tIsVelIter>
			   (//solve artics:
				solveArticulationContactLast, 
				articulationListSize, articCount,
				articulationListStart,
				solverDt,
				articulationBiasCoefficient, 
				residualReportingActive, 
				maxArticIndex, targetArticIndex, articSolveStart, articIndexCounter, articSolveEnd, 
				articIndexCompleted, articIndex,
				//solve rbodies:
				nbPartitions, headersPerPartition,
				normalIteration, unrollCount, batchCount, 
				constraintList, 
				cache, contactIter,
				gVTableSolveBlock,
				maxNormalIndex, index, endIndexCount, targetConstraintIndex,
				constraintIndex, constraintIndexCompleted);

		++normalIteration;
	}

	ThresholdStreamElement* PX_RESTRICT thresholdStream = params.thresholdStream;
	PxU32 thresholdStreamLength = params.thresholdStreamLength;
	PxI32* outThresholdPairs = params.outThresholdPairs;

	cache.mSharedOutThresholdPairs = outThresholdPairs;
	cache.mSharedThresholdStream = thresholdStream;
	cache.mSharedThresholdStreamLength = thresholdStreamLength;

	//Perform a single velocity iteration to make sure that we always run at least 1.
	//Last iteration - do writeback as well!
	cache.writeBackIteration = true;
	{		
		if (residualReportingActive)
			cache.contactErrorAccumulator->reset();

		processSolverIterationParallel<tWriteBackInternalConstraints, tIsVelIter>
			   (//solve artics:
				solveArticulationContactLast, 
				articulationListSize, articCount,
				articulationListStart,
				solverDt,
				articulationBiasCoefficient, 
				residualReportingActive, 
				maxArticIndex, targetArticIndex, articSolveStart, articIndexCounter, articSolveEnd, 
				articIndexCompleted, articIndex,
				//solve rbodies:
				nbPartitions, headersPerPartition,
				normalIteration, unrollCount, batchCount, 
				constraintList, 
				cache, contactIter,
				gVTableSolveWriteBackBlock,
				maxNormalIndex, index, endIndexCount, targetConstraintIndex,
				constraintIndex, constraintIndexCompleted);

		// At this point we've awaited the completion all rigid partitions and all articulations
		// No more syncing on the outside of this function is required.

		if(cache.mThresholdStreamIndex > 0)
		{
			//Write back to global buffer
			PxI32 threshIndex = PxAtomicAdd(outThresholdPairs, PxI32(cache.mThresholdStreamIndex)) - PxI32(cache.mThresholdStreamIndex);
			for(PxU32 b = 0; b < cache.mThresholdStreamIndex; ++b)
			{
				thresholdStream[b + threshIndex] = cache.mThresholdStream[b];
			}
			cache.mThresholdStreamIndex = 0;
		}

		++normalIteration;
	}

#if PX_PROFILE_SOLVE_STALLS
	PxU64 endTime = readTimer();
	PxReal totalTime = (PxReal)(endTime - startTime);
	PxReal stallTime = (PxReal)stallCount;
	PxReal stallRatio = stallTime/totalTime;
	if(0)//stallRatio > 0.2f)
	{
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency( &frequency );
		printf("Warning -- percentage time stalled = %f; stalled for %f seconds; total Time took %f seconds\n", 
			stallRatio * 100.f, stallTime/(PxReal)frequency.QuadPart, totalTime/(PxReal)frequency.QuadPart);
	}
#endif
}

} //namespace Dy
} //namespace physx

