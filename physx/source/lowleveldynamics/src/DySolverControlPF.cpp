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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxPreprocessor.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxIntrinsics.h"
#include "DySolverBody.h"
#include "DySolverConstraint1D.h"
#include "DySolverContact.h"
#include "DyThresholdTable.h"
#include "DySolverControl.h"
#include "DyArticulationPImpl.h"
#include "foundation/PxThread.h"
#include "DySolverConstraintDesc.h"
#include "DySolverContext.h"
#include "DySolverControlPF.h"
#include "DyArticulationCpuGpu.h"

namespace physx
{
namespace Dy
{
void solve1DBlock		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExt1DBlock	(DY_PGS_SOLVE_METHOD_PARAMS);
void solve1D4_Block		(DY_PGS_SOLVE_METHOD_PARAMS);

void solve1DConcludeBlock		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExt1DConcludeBlock	(DY_PGS_SOLVE_METHOD_PARAMS);
void solve1D4Block_Conclude		(DY_PGS_SOLVE_METHOD_PARAMS);

void solve1DBlockWriteBack		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExt1DBlockWriteBack	(DY_PGS_SOLVE_METHOD_PARAMS);
void solve1D4Block_WriteBack	(DY_PGS_SOLVE_METHOD_PARAMS);

void writeBack1DBlock		(DY_PGS_SOLVE_METHOD_PARAMS);
void ext1DBlockWriteBack	(DY_PGS_SOLVE_METHOD_PARAMS);
void writeBack1D4Block		(DY_PGS_SOLVE_METHOD_PARAMS);

void solveFrictionBlock					(DY_PGS_SOLVE_METHOD_PARAMS);
void solveFriction_BStaticBlock			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExtFrictionBlock				(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactCoulombBlock			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExtContactCoulombBlock		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactCoulomb_BStaticBlock	(DY_PGS_SOLVE_METHOD_PARAMS);

void solveContactCoulombConcludeBlock			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExtContactCoulombConcludeBlock		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactCoulomb_BStaticConcludeBlock	(DY_PGS_SOLVE_METHOD_PARAMS);

void solveContactCoulombBlockWriteBack			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExtContactCoulombBlockWriteBack		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactCoulomb_BStaticBlockWriteBack	(DY_PGS_SOLVE_METHOD_PARAMS);
void solveFrictionBlockWriteBack				(DY_PGS_SOLVE_METHOD_PARAMS);
void solveFriction_BStaticBlockWriteBack		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveExtFrictionBlockWriteBack				(DY_PGS_SOLVE_METHOD_PARAMS);

//Pre-block 1d/2d friction stuff...

void solveContactCoulombPreBlock				(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactCoulombPreBlock_Static			(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactCoulombPreBlock_Conclude		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactCoulombPreBlock_ConcludeStatic	(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactCoulombPreBlock_WriteBack		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveContactCoulombPreBlock_WriteBackStatic(DY_PGS_SOLVE_METHOD_PARAMS);
void solveFrictionCoulombPreBlock				(DY_PGS_SOLVE_METHOD_PARAMS);

void solveFrictionCoulombPreBlock_Static		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveFrictionCoulombPreBlock_Conclude		(DY_PGS_SOLVE_METHOD_PARAMS);
void solveFrictionCoulombPreBlock_ConcludeStatic(DY_PGS_SOLVE_METHOD_PARAMS);

void solveFrictionCoulombPreBlock_WriteBack		(DY_PGS_SOLVE_METHOD_PARAMS);

void solveFrictionCoulombPreBlock_WriteBackStatic(DY_PGS_SOLVE_METHOD_PARAMS);

static SolveBlockMethod gVTableSolveBlockCoulomb[] PX_UNUSED_ATTRIBUTE = 
{
	0,
	solveContactCoulombBlock,				// DY_SC_TYPE_RB_CONTACT
	solve1DBlock,							// DY_SC_TYPE_RB_1D
	solveExtContactCoulombBlock,			// DY_SC_TYPE_EXT_CONTACT
	solveExt1DBlock,						// DY_SC_TYPE_EXT_1D
	solveContactCoulomb_BStaticBlock,		// DY_SC_TYPE_STATIC_CONTACT
	solveContactCoulombBlock,				// DY_SC_TYPE_NOFRICTION_RB_CONTACT
	solveContactCoulombPreBlock,			// DY_SC_TYPE_BLOCK_RB_CONTACT
	solveContactCoulombPreBlock_Static,		// DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT
	solve1D4_Block,							// DY_SC_TYPE_BLOCK_1D,
	solveFrictionBlock,						// DY_SC_TYPE_FRICTION
	solveFriction_BStaticBlock,				// DY_SC_TYPE_STATIC_FRICTION
	solveExtFrictionBlock,					// DY_SC_TYPE_EXT_FRICTION
	solveFrictionCoulombPreBlock,			// DY_SC_TYPE_BLOCK_FRICTION					
	solveFrictionCoulombPreBlock_Static		// DY_SC_TYPE_BLOCK_STATIC_FRICTION
};

static SolveWriteBackBlockMethod gVTableSolveWriteBackBlockCoulomb[] PX_UNUSED_ATTRIBUTE = 
{
	0,
	solveContactCoulombBlockWriteBack,				// DY_SC_TYPE_RB_CONTACT
	solve1DBlockWriteBack,							// DY_SC_TYPE_RB_1D
	solveExtContactCoulombBlockWriteBack,			// DY_SC_TYPE_EXT_CONTACT
	solveExt1DBlockWriteBack,						// DY_SC_TYPE_EXT_1D
	solveContactCoulomb_BStaticBlockWriteBack,		// DY_SC_TYPE_STATIC_CONTACT
	solveContactCoulombBlockWriteBack,				// DY_SC_TYPE_NOFRICTION_RB_CONTACT
	solveContactCoulombPreBlock_WriteBack,			// DY_SC_TYPE_BLOCK_RB_CONTACT
	solveContactCoulombPreBlock_WriteBackStatic,	// DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT
	solve1D4Block_WriteBack,						// DY_SC_TYPE_BLOCK_1D,
	solveFrictionBlockWriteBack,					// DY_SC_TYPE_FRICTION
	solveFriction_BStaticBlockWriteBack,			// DY_SC_TYPE_STATIC_FRICTION
	solveExtFrictionBlockWriteBack,					// DY_SC_TYPE_EXT_FRICTION
	solveFrictionCoulombPreBlock_WriteBack,			// DY_SC_TYPE_BLOCK_FRICTION
	solveFrictionCoulombPreBlock_WriteBackStatic	// DY_SC_TYPE_BLOCK_STATIC_FRICTION
};

static SolveBlockMethod gVTableSolveConcludeBlockCoulomb[] PX_UNUSED_ATTRIBUTE = 
{
	0,
	solveContactCoulombConcludeBlock,				// DY_SC_TYPE_RB_CONTACT
	solve1DConcludeBlock,							// DY_SC_TYPE_RB_1D
	solveExtContactCoulombConcludeBlock,			// DY_SC_TYPE_EXT_CONTACT
	solveExt1DConcludeBlock,						// DY_SC_TYPE_EXT_1D
	solveContactCoulomb_BStaticConcludeBlock,		// DY_SC_TYPE_STATIC_CONTACT
	solveContactCoulombConcludeBlock,				// DY_SC_TYPE_NOFRICTION_RB_CONTACT
	solveContactCoulombPreBlock_Conclude,			// DY_SC_TYPE_BLOCK_RB_CONTACT
	solveContactCoulombPreBlock_ConcludeStatic,		// DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT
	solve1D4Block_Conclude,							// DY_SC_TYPE_BLOCK_1D,
	solveFrictionBlock,								// DY_SC_TYPE_FRICTION
	solveFriction_BStaticBlock,						// DY_SC_TYPE_STATIC_FRICTION
	solveExtFrictionBlock,							// DY_SC_TYPE_EXT_FRICTION
	solveFrictionCoulombPreBlock_Conclude,			// DY_SC_TYPE_BLOCK_FRICTION
	solveFrictionCoulombPreBlock_ConcludeStatic		// DY_SC_TYPE_BLOCK_STATIC_FRICTION
};

// PT: code shared with patch friction solver. Ideally should move to a shared DySolverCore.cpp file.
void solveNoContactsCase(	PxU32 bodyListSize, PxSolverBody* PX_RESTRICT bodyListStart, Cm::SpatialVector* PX_RESTRICT motionVelocityArray,
							PxU32 articulationListSize, ArticulationSolverDesc* PX_RESTRICT articulationListStart, Cm::SpatialVectorF* PX_RESTRICT Z, Cm::SpatialVectorF* PX_RESTRICT deltaV,
							PxU32 positionIterations, PxU32 velocityIterations, PxF32 dt, PxF32 invDt);

void saveMotionVelocities(PxU32 nbBodies, PxSolverBody* PX_RESTRICT solverBodies, Cm::SpatialVector* PX_RESTRICT motionVelocityArray);

void SolverCoreGeneralPF::solveV_Blocks(SolverIslandParams& params) const
{
	const PxF32 biasCoefficient = DY_ARTICULATION_PGS_BIAS_COEFFICIENT;
	const bool isTGS = false;

	const PxI32 TempThresholdStreamSize = 32;
	ThresholdStreamElement tempThresholdStream[TempThresholdStreamSize];

	SolverContext cache;
	cache.solverBodyArray			= params.bodyDataList;
	cache.mThresholdStream			= tempThresholdStream;
	cache.mThresholdStreamLength	= TempThresholdStreamSize;
	cache.mThresholdStreamIndex		= 0;
	cache.writeBackIteration		= false;
	cache.deltaV					= params.deltaV;
	cache.Z							= params.Z;

	const PxI32 batchCount = PxI32(params.numConstraintHeaders);

	PxSolverBody* PX_RESTRICT bodyListStart = params.bodyListStart;
	const PxU32 bodyListSize = params.bodyListSize;

	Cm::SpatialVector* PX_RESTRICT motionVelocityArray = params.motionVelocityArray;

	const PxU32 velocityIterations = params.velocityIterations;
	const PxU32 positionIterations = params.positionIterations;

	const PxU32 numConstraintHeaders = params.numConstraintHeaders;
	const PxU32 articulationListSize = params.articulationListSize;

	ArticulationSolverDesc* PX_RESTRICT articulationListStart = params.articulationListStart;

	PX_ASSERT(velocityIterations >= 1);
	PX_ASSERT(positionIterations >= 1);

	if(numConstraintHeaders == 0)
	{
		solveNoContactsCase(bodyListSize, bodyListStart, motionVelocityArray,
							articulationListSize, articulationListStart, cache.Z, cache.deltaV,
							positionIterations, velocityIterations, params.dt, params.invDt);
		return;
	}

	BatchIterator contactIterator(params.constraintBatchHeaders, params.numConstraintHeaders);
	BatchIterator frictionIterator(params.frictionConstraintBatches, params.numFrictionConstraintHeaders);

	const PxI32 frictionBatchCount = PxI32(params.numFrictionConstraintHeaders);

	PxSolverConstraintDesc* PX_RESTRICT constraintList = params.constraintList;

	PxSolverConstraintDesc* PX_RESTRICT frictionConstraintList = params.frictionConstraintList;

	//0-(n-1) iterations
	PxI32 normalIter = 0;
	PxI32 frictionIter = 0;
	for (PxU32 iteration = positionIterations; iteration > 0; iteration--)	//decreasing positive numbers == position iters
	{
		SolveBlockParallel(constraintList, batchCount, normalIter * batchCount, batchCount, 
			cache, contactIterator, iteration == 1 ? gVTableSolveConcludeBlockCoulomb : gVTableSolveBlockCoulomb, normalIter);

		for (PxU32 i = 0; i < articulationListSize; ++i)
			articulationListStart[i].articulation->solveInternalConstraints(params.dt, params.invDt, cache.Z, cache.deltaV, false, isTGS, 0.f, biasCoefficient);

		++normalIter;
	}

	if(frictionBatchCount>0)
	{
		const PxU32 numIterations = positionIterations * 2;
		for (PxU32 iteration = numIterations; iteration > 0; iteration--)	//decreasing positive numbers == position iters
		{
			SolveBlockParallel(frictionConstraintList, frictionBatchCount, frictionIter * frictionBatchCount, frictionBatchCount, 
				cache, frictionIterator, iteration == 1 ? gVTableSolveConcludeBlockCoulomb : gVTableSolveBlockCoulomb, frictionIter);
			++frictionIter;
		}
	}

	saveMotionVelocities(bodyListSize, bodyListStart, motionVelocityArray);
	
	for (PxU32 i = 0; i < articulationListSize; i++)
		ArticulationPImpl::saveVelocity(articulationListStart[i].articulation, cache.deltaV);

	const PxU32 velItersMinOne = velocityIterations - 1;

	PxU32 iteration = 0;

	for(; iteration < velItersMinOne; ++iteration)
	{	
		SolveBlockParallel(constraintList, batchCount, normalIter * batchCount, batchCount, 
			cache, contactIterator, gVTableSolveBlockCoulomb, normalIter);

		for (PxU32 i = 0; i < articulationListSize; ++i)
			articulationListStart[i].articulation->solveInternalConstraints(params.dt, params.invDt, cache.Z, cache.deltaV, true, isTGS, 0.f, biasCoefficient);
		++normalIter;

		if(frictionBatchCount > 0)
		{
			SolveBlockParallel(frictionConstraintList, frictionBatchCount, frictionIter * frictionBatchCount, frictionBatchCount, 
				cache, frictionIterator, gVTableSolveBlockCoulomb, frictionIter);
			++frictionIter;
		}
	}

	PxI32* outThresholdPairs = params.outThresholdPairs;
	ThresholdStreamElement* PX_RESTRICT thresholdStream = params.thresholdStream;
	PxU32 thresholdStreamLength = params.thresholdStreamLength;

	cache.writeBackIteration = true;

	cache.mSharedOutThresholdPairs = outThresholdPairs;
	cache.mSharedThresholdStreamLength = thresholdStreamLength;
	cache.mSharedThresholdStream = thresholdStream;

	//PGS always runs one velocity iteration
	{
		SolveBlockParallel(constraintList, batchCount, normalIter * batchCount, batchCount, 
			cache, contactIterator, gVTableSolveWriteBackBlockCoulomb, normalIter);
		++normalIter;

		for (PxU32 i = 0; i < articulationListSize; ++i)
		{
			articulationListStart[i].articulation->solveInternalConstraints(params.dt, params.invDt, cache.Z, cache.deltaV, true, isTGS, 0.f, biasCoefficient);
			articulationListStart[i].articulation->writebackInternalConstraints(false);
		}

		if(frictionBatchCount > 0)
		{
			SolveBlockParallel(frictionConstraintList, frictionBatchCount, frictionIter * frictionBatchCount, frictionBatchCount, 
				cache, frictionIterator, gVTableSolveWriteBackBlockCoulomb, frictionIter);
				++frictionIter;
		}
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

void SolverCoreGeneralPF::solveVParallelAndWriteBack(SolverIslandParams& params,
	Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV) const
{
	const PxF32 biasCoefficient = DY_ARTICULATION_PGS_BIAS_COEFFICIENT;
	const bool isTGS = false;

	SolverContext cache;
	cache.solverBodyArray = params.bodyDataList;

	const PxI32 UnrollCount = PxI32(params.batchSize);
	const PxI32 SaveUnrollCount = 64;
	const PxI32 ArticCount = 2;

	const PxI32 TempThresholdStreamSize = 32;
	ThresholdStreamElement tempThresholdStream[TempThresholdStreamSize];

	const PxI32 batchCount = PxI32(params.numConstraintHeaders);
	const PxI32 frictionBatchCount = PxI32(params.numFrictionConstraintHeaders);//frictionConstraintBatches.size();
	cache.mThresholdStream = tempThresholdStream;
	cache.mThresholdStreamLength = TempThresholdStreamSize;
	cache.mThresholdStreamIndex = 0;
	cache.Z = Z;
	cache.deltaV = deltaV;

	const PxReal dt = params.dt;
	const PxReal invDt = params.invDt;

	ArticulationSolverDesc* PX_RESTRICT articulationListStart = params.articulationListStart;

	const PxI32 positionIterations = PxI32(params.positionIterations);
	const PxU32 velocityIterations = params.velocityIterations;

	const PxI32 bodyListSize = PxI32(params.bodyListSize);
	const PxI32 articulationListSize = PxI32(params.articulationListSize);

	PX_ASSERT(velocityIterations >= 1);
	PX_ASSERT(positionIterations >= 1);

	PxI32* constraintIndex = &params.constraintIndex;
	PxI32* constraintIndexCompleted = &params.constraintIndexCompleted;
	PxI32* frictionConstraintIndex = &params.frictionConstraintIndex;

	PxI32 endIndexCount = UnrollCount;
	PxI32 index = PxAtomicAdd(constraintIndex, UnrollCount) - UnrollCount;
	PxI32 frictionIndex = PxAtomicAdd(frictionConstraintIndex, UnrollCount) - UnrollCount;
	
	BatchIterator contactIter(params.constraintBatchHeaders, params.numConstraintHeaders);
	BatchIterator frictionIter(params.frictionConstraintBatches, params.numFrictionConstraintHeaders);

	PxU32* headersPerPartition = params.headersPerPartition;
	PxU32 nbPartitions = params.nbPartitions;

	PxU32* frictionHeadersPerPartition = params.frictionHeadersPerPartition;
	PxU32 nbFrictionPartitions = params.nbFrictionPartitions;

	PxSolverConstraintDesc* PX_RESTRICT constraintList = params.constraintList;
	PxSolverConstraintDesc* PX_RESTRICT frictionConstraintList = params.frictionConstraintList;

	PxI32 maxNormalIndex = 0;
	PxI32 maxProgress = 0;
	PxI32 frictionEndIndexCount = UnrollCount;
	PxI32 maxFrictionIndex = 0;

	PxI32 articSolveStart = 0;
	PxI32 articSolveEnd = 0;
	PxI32 maxArticIndex = 0;
	PxI32 articIndexCounter = 0;
	PxI32 targetArticIndex = 0;

	PxI32* articIndex = &params.articSolveIndex;
	PxI32* articIndexCompleted = &params.articSolveIndexCompleted;

	PxI32 normalIteration = 0;
	PxI32 frictionIteration = 0;
	PxU32 a = 0;
	for(PxU32 i = 0; i < 2; ++i)
	{
		SolveBlockMethod* solveTable = i == 0 ? gVTableSolveBlockCoulomb : gVTableSolveConcludeBlockCoulomb;
		for(; a < positionIterations - 1 + i; ++a)
		{
			WAIT_FOR_PROGRESS(articIndexCompleted, targetArticIndex);

			for(PxU32 b = 0; b < nbPartitions; ++b)
			{
				WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);
				maxNormalIndex += headersPerPartition[b];
				maxProgress += headersPerPartition[b];
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
						endIndexCount = UnrollCount;
						index = PxAtomicAdd(constraintIndex, UnrollCount) - UnrollCount;
					}
				}
				if(nbSolved)
				{
					PxMemoryBarrier();
					PxAtomicAdd(constraintIndexCompleted, nbSolved);
				}
			}

			WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);

			maxArticIndex += articulationListSize;
			targetArticIndex += articulationListSize;

			while (articSolveStart < maxArticIndex)
			{
				const PxI32 endIdx = PxMin(articSolveEnd, maxArticIndex);

				PxI32 nbSolved = 0;
				while (articSolveStart < endIdx)
				{
					articulationListStart[articSolveStart - articIndexCounter].articulation->solveInternalConstraints(dt, invDt, cache.Z, cache.deltaV, false, isTGS, 0.f, biasCoefficient);
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

			++normalIteration;
		}
	}

	WAIT_FOR_PROGRESS(articIndexCompleted, targetArticIndex);

	for(PxU32 i = 0; i < 2; ++i)
	{
		SolveBlockMethod* solveTable = i == 0 ? gVTableSolveBlockCoulomb : gVTableSolveConcludeBlockCoulomb;
		const PxI32 numIterations = positionIterations *2;
		for(; a <  numIterations - 1 + i; ++a)
		{
			for(PxU32 b = 0; b < nbFrictionPartitions; ++b)
			{
				WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);
				maxProgress += frictionHeadersPerPartition[b];
				maxFrictionIndex += frictionHeadersPerPartition[b];
				PxI32 nbSolved = 0;
				while(frictionIndex < maxFrictionIndex)
				{
					const PxI32 remainder = PxMin(maxFrictionIndex - frictionIndex, frictionEndIndexCount);
					SolveBlockParallel(frictionConstraintList, remainder, frictionIndex, frictionBatchCount, cache, frictionIter, 
						solveTable, frictionIteration);
					frictionIndex += remainder;
					frictionEndIndexCount -= remainder;
					nbSolved += remainder;
					if(frictionEndIndexCount == 0)
					{
						frictionEndIndexCount = UnrollCount;
						frictionIndex  = PxAtomicAdd(frictionConstraintIndex, UnrollCount) - UnrollCount;
					}
				}
				if(nbSolved)
				{
					PxMemoryBarrier();
					PxAtomicAdd(constraintIndexCompleted, nbSolved);
				}
			}
			++frictionIteration;
		}
	}

	WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);

	PxI32* bodyListIndex = &params.bodyListIndex;
	PxI32* bodyListIndexCompleted = &params.bodyListIndexCompleted;

	PxSolverBody* PX_RESTRICT bodyListStart = params.bodyListStart;

	Cm::SpatialVector* PX_RESTRICT motionVelocityArray = params.motionVelocityArray;

	PxI32 endIndexCount2 = SaveUnrollCount;
	PxI32 index2 = PxAtomicAdd(bodyListIndex, SaveUnrollCount) - SaveUnrollCount;
	{
		PxI32 nbConcluded = 0;
		while(index2 < articulationListSize)
		{
			const PxI32 remainder = PxMin(SaveUnrollCount, (articulationListSize - index2));
			endIndexCount2 -= remainder;
			for(PxI32 b = 0; b < remainder; ++b, ++index2)
			{
				ArticulationPImpl::saveVelocity(articulationListStart[index2].articulation, cache.deltaV);
			}
			nbConcluded += remainder;
			if(endIndexCount2 == 0)
			{
				index2 = PxAtomicAdd(bodyListIndex, SaveUnrollCount) - SaveUnrollCount;
				endIndexCount2 = SaveUnrollCount;
			}
		}

		index2 -= articulationListSize;

		//save velocity
		
		while(index2 < bodyListSize)
		{
			const PxI32 remainder = PxMin(endIndexCount2, (bodyListSize - index2));
			endIndexCount2 -= remainder;
			for(PxI32 b = 0; b < remainder; ++b, ++index2)
			{
				PxPrefetchLine(&bodyListStart[index2 + 8]);
				PxPrefetchLine(&motionVelocityArray[index2 + 8]);
				PxSolverBody& body = bodyListStart[index2];
				Cm::SpatialVector& motionVel = motionVelocityArray[index2];
				motionVel.linear = body.linearVelocity;
				motionVel.angular = body.angularState;
				PX_ASSERT(motionVel.linear.isFinite());
				PX_ASSERT(motionVel.angular.isFinite());
			}

			nbConcluded += remainder;
			
			//Branch not required because this is the last time we use this atomic variable
			//if(index2 < articulationListSizePlusbodyListSize)
			{
				index2 = PxAtomicAdd(bodyListIndex, SaveUnrollCount) - SaveUnrollCount - articulationListSize;
				endIndexCount2 = SaveUnrollCount;
			}
		}

		if(nbConcluded)
		{
			PxMemoryBarrier();
			PxAtomicAdd(bodyListIndexCompleted, nbConcluded);
		}
	}

	WAIT_FOR_PROGRESS(bodyListIndexCompleted, (bodyListSize + articulationListSize));

	a = 0;
	for(; a < velocityIterations-1; ++a)
	{
		WAIT_FOR_PROGRESS(articIndexCompleted, targetArticIndex);
		for(PxU32 b = 0; b < nbPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);
			maxNormalIndex += headersPerPartition[b];
			maxProgress += headersPerPartition[b];
			
			PxI32 nbSolved = 0;
			while(index < maxNormalIndex)
			{
				const PxI32 remainder = PxMin(maxNormalIndex - index, endIndexCount);
				SolveBlockParallel(constraintList, remainder, index, batchCount, cache, contactIter, gVTableSolveBlockCoulomb, normalIteration);
				index += remainder;
				endIndexCount -= remainder;
				nbSolved += remainder;
				if(endIndexCount == 0)
				{
					endIndexCount = UnrollCount;
					index = PxAtomicAdd(constraintIndex, UnrollCount) - UnrollCount;
				}
			}
			if(nbSolved)
			{
				PxMemoryBarrier();
				PxAtomicAdd(constraintIndexCompleted, nbSolved);
			}
		}
		++normalIteration;

		for(PxU32 b = 0; b < nbFrictionPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);
			maxFrictionIndex += frictionHeadersPerPartition[b];
			maxProgress += frictionHeadersPerPartition[b];

			PxI32 nbSolved = 0;
			while(frictionIndex < maxFrictionIndex)
			{
				const PxI32 remainder = PxMin(maxFrictionIndex - frictionIndex, frictionEndIndexCount);
				SolveBlockParallel(frictionConstraintList, remainder, frictionIndex, frictionBatchCount, cache, frictionIter, gVTableSolveBlockCoulomb, 
					frictionIteration);

				frictionIndex += remainder;
				frictionEndIndexCount -= remainder;
				nbSolved += remainder;
				if(frictionEndIndexCount == 0)
				{
					frictionEndIndexCount = UnrollCount;
					frictionIndex  = PxAtomicAdd(frictionConstraintIndex, UnrollCount) - UnrollCount;
				}
			}
			if(nbSolved)
			{
				PxMemoryBarrier();
				PxAtomicAdd(constraintIndexCompleted, nbSolved);
			}
		}
		++frictionIteration;

		WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);

		maxArticIndex += articulationListSize;
		targetArticIndex += articulationListSize;

		while (articSolveStart < maxArticIndex)
		{
			const PxI32 endIdx = PxMin(articSolveEnd, maxArticIndex);

			PxI32 nbSolved = 0;
			while (articSolveStart < endIdx)
			{
				articulationListStart[articSolveStart - articIndexCounter].articulation->solveInternalConstraints(dt, invDt, cache.Z, cache.deltaV, true, isTGS, 0.f, biasCoefficient);
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
	}

	ThresholdStreamElement* PX_RESTRICT thresholdStream = params.thresholdStream;
	const PxU32 thresholdStreamLength = params.thresholdStreamLength;
	PxI32* outThresholdPairs = params.outThresholdPairs;

	cache.mSharedThresholdStream = thresholdStream;
	cache.mSharedOutThresholdPairs = outThresholdPairs;
	cache.mSharedThresholdStreamLength = thresholdStreamLength;

	// last velocity + write-back iteration
	{
		WAIT_FOR_PROGRESS(articIndexCompleted, targetArticIndex);

		for(PxU32 b = 0; b < nbPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);
			maxNormalIndex += headersPerPartition[b];
			maxProgress += headersPerPartition[b];
			
			PxI32 nbSolved = 0;
			while(index < maxNormalIndex)
			{
				const PxI32 remainder = PxMin(maxNormalIndex - index, endIndexCount);

				SolveBlockParallel(constraintList, remainder, index, batchCount,
					cache, contactIter, gVTableSolveWriteBackBlockCoulomb, normalIteration);

				index += remainder;
				endIndexCount -= remainder;
				nbSolved += remainder;
				if(endIndexCount == 0)
				{
					endIndexCount = UnrollCount;
					index = PxAtomicAdd(constraintIndex, UnrollCount) - UnrollCount;
				}
			}
			if(nbSolved)
			{
				PxMemoryBarrier();
				PxAtomicAdd(constraintIndexCompleted, nbSolved);
			}
		}

		++normalIteration;

		cache.mSharedOutThresholdPairs = outThresholdPairs;
		cache.mSharedThresholdStream = thresholdStream;
		cache.mSharedThresholdStreamLength = thresholdStreamLength;

		for(PxU32 b = 0; b < nbFrictionPartitions; ++b)
		{
			WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);
			maxFrictionIndex += frictionHeadersPerPartition[b];
			maxProgress += frictionHeadersPerPartition[b];

			PxI32 nbSolved = 0;
			while(frictionIndex < maxFrictionIndex)
			{
				const PxI32 remainder = PxMin(maxFrictionIndex - frictionIndex, frictionEndIndexCount);

				SolveBlockParallel(frictionConstraintList, remainder, frictionIndex, frictionBatchCount, cache, frictionIter, 
					gVTableSolveWriteBackBlockCoulomb, frictionIteration);

				frictionIndex += remainder;
				frictionEndIndexCount -= remainder;
				nbSolved += remainder;
				if(frictionEndIndexCount == 0)
				{
					frictionEndIndexCount = UnrollCount;
					frictionIndex  = PxAtomicAdd(frictionConstraintIndex, UnrollCount) - UnrollCount;
				}
			}
			if(nbSolved)
			{
				PxMemoryBarrier();
				PxAtomicAdd(constraintIndexCompleted, nbSolved);
			}
		}
		++frictionIteration;

		{
			WAIT_FOR_PROGRESS(constraintIndexCompleted, maxProgress);
			maxArticIndex += articulationListSize;
			targetArticIndex += articulationListSize;

			while (articSolveStart < maxArticIndex)
			{
				const PxI32 endIdx = PxMin(articSolveEnd, maxArticIndex);

				PxI32 nbSolved = 0;
				while (articSolveStart < endIdx)
				{
					articulationListStart[articSolveStart - articIndexCounter].articulation->solveInternalConstraints(dt, invDt, cache.Z, cache.deltaV, false, isTGS, 0.f, biasCoefficient);
					articulationListStart[articSolveStart - articIndexCounter].articulation->writebackInternalConstraints(false);
					articSolveStart++;
					nbSolved++;
				}

				if (nbSolved)
				{
					PxMemoryBarrier();
					PxAtomicAdd(articIndexCompleted, nbSolved);
				}

				PxI32 remaining = articSolveEnd - articSolveStart;

				if (remaining == 0)
				{
					articSolveStart = PxAtomicAdd(articIndex, ArticCount) - ArticCount;
					articSolveEnd = articSolveStart + ArticCount;
				}
			}

			articIndexCounter += articulationListSize; // not strictly necessary but better safe than sorry
			WAIT_FOR_PROGRESS(articIndexCompleted, targetArticIndex);
		}

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

	}
}

}

}


//#endif
