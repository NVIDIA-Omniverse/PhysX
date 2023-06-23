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

#if PX_SUPPORT_GPU_PHYSX

#include "NpParticleSystem.h"

#include "foundation/PxAllocator.h"
#include "foundation/PxArray.h"
#include "foundation/PxMath.h"
#include "foundation/PxMemory.h"
#include "foundation/PxSort.h"

#include "common/PxPhysXCommonConfig.h"

#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "PxGridParticleSystem.h"
#endif
#include "PxRigidActor.h"

#include "PxsSimulationController.h"

#include "NpArticulationLink.h"
#include "NpRigidDynamic.h"
#include "NpScene.h"
#include "NpCheck.h"

#include "ScBodyCore.h"
#include "ScParticleSystemCore.h"
#include "ScParticleSystemSim.h"
#include "ScParticleSystemShapeCore.h"

#include "DyParticleSystemCore.h"

#include "CmVisualization.h"

#define PARTICLE_MAX_NUM_PARTITIONS_TEMP	32
#define PARTICLE_MAX_NUM_PARTITIONS_FINAL   8

using namespace physx;

namespace physx
{
#if PX_ENABLE_DEBUG_VISUALIZATION
	static void visualizeParticleSystem(PxRenderOutput& out, NpScene& npScene, const Sc::ParticleSystemCore& core)
	{
		if (!(core.getActorFlags() & PxActorFlag::eVISUALIZATION))
			return;

		const Sc::Scene& scScene = npScene.getScScene();

		const bool visualizeAABBs = scScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_AABBS) != 0.0f;
		if (visualizeAABBs)
		{
			out << PxU32(PxDebugColor::eARGB_YELLOW) << PxMat44(PxIdentity);
			Cm::renderOutputDebugBox(out, core.getSim()->getBounds());
		}
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

	////////////////////////////////////////////////////////////////////////////////////////

	PxPartitionedParticleCloth::PxPartitionedParticleCloth() 
	{ 
		PxMemZero(this, sizeof(*this)); 
	}

	PxPartitionedParticleCloth::~PxPartitionedParticleCloth()
	{
		if (mCudaManager)
		{
			PxScopedCudaLock lock(*mCudaManager);
			PxCudaContext* context = mCudaManager->getCudaContext();

			if (context)
			{
				context->memFreeHost(accumulatedSpringsPerPartitions);
				context->memFreeHost(accumulatedCopiesPerParticles);
				context->memFreeHost(remapOutput);
				context->memFreeHost(orderedSprings);
				context->memFreeHost(sortedClothStartIndices);
				context->memFreeHost(cloths);
			}
		}
	}

	void PxPartitionedParticleCloth::allocateBuffers(PxU32 nbParticles, PxCudaContextManager* cudaManager)
	{
		mCudaManager = cudaManager;

		PxScopedCudaLock lock(*mCudaManager);
		PxCudaContext* context = mCudaManager->getCudaContext();

		const unsigned int CU_MEMHOSTALLOC_DEVICEMAP = 0x02;
		const unsigned int CU_MEMHOSTALLOC_PORTABLE = 0x01;

		PxCUresult result = context->memHostAlloc(reinterpret_cast<void**>(&accumulatedSpringsPerPartitions), size_t(sizeof(PxU32) * PARTICLE_MAX_NUM_PARTITIONS_FINAL), CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE); // TODO AD: WTF where does 32 come from?
		result = context->memHostAlloc(reinterpret_cast<void**>(&accumulatedCopiesPerParticles), size_t(sizeof(PxU32) * nbParticles), CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
		result = context->memHostAlloc(reinterpret_cast<void**>(&orderedSprings), size_t(sizeof(PxParticleSpring) * nbSprings), CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
		result = context->memHostAlloc(reinterpret_cast<void**>(&remapOutput), size_t(sizeof(PxU32) * nbSprings * 2), CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
		result = context->memHostAlloc(reinterpret_cast<void**>(&sortedClothStartIndices), size_t(sizeof(PxU32) * nbCloths), CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
		result = context->memHostAlloc(reinterpret_cast<void**>(&cloths), size_t(sizeof(PxParticleCloth) * nbCloths), CU_MEMHOSTALLOC_DEVICEMAP | CU_MEMHOSTALLOC_PORTABLE);
	}

	////////////////////////////////////////////////////////////////////////////////////////
	
	PxU32 NpParticleClothPreProcessor::computeSpringPartition(const PxParticleSpring& spring, const PxU32 partitionStartIndex, PxU32* partitionProgresses)
	{
		PxU32 partitionA = partitionProgresses[spring.ind0];
		PxU32 partitionB = partitionProgresses[spring.ind1];

		const PxU32 combinedMask = (~partitionA & ~partitionB);
		PxU32 availablePartition = combinedMask == 0 ? PARTICLE_MAX_NUM_PARTITIONS_TEMP : PxLowestSetBit(combinedMask);

		if (availablePartition == PARTICLE_MAX_NUM_PARTITIONS_TEMP)
		{
			return 0xFFFFFFFF;
		}

		const PxU32 partitionBit = (1u << availablePartition);
		partitionA |= partitionBit;
		partitionB |= partitionBit;

		availablePartition += partitionStartIndex;

		partitionProgresses[spring.ind0] = partitionA;
		partitionProgresses[spring.ind1] = partitionB;

		return availablePartition;

	}

	void NpParticleClothPreProcessor::writeSprings(const PxParticleSpring* springs, PxU32* partitionProgresses, PxU32* tempSprings,
		PxU32* orderedSprings, PxU32* accumulatedSpringsPerPartition)
	{
		//initialize the partition progress counter to be zero
		PxMemZero(partitionProgresses, sizeof(PxU32) * mNumParticles);

		PxU32 numUnpartitionedSprings = 0;

		// Goes through all the springs and assigns them to a partition. This code is exactly the same as in classifySprings
		// except that we now know the start indices of all the partitions so we can write THEIR INDEX into the ordered spring
		// index list.
		// AD: All of this relies on the fact that we partition exactly the same way twice. Remember that when changing anything
		// here.
		for (PxU32 i = 0; i < mNumSprings; ++i)
		{
			const PxParticleSpring& spring = springs[i];

			const PxU32 availablePartition = computeSpringPartition(spring, 0, partitionProgresses);

			if (availablePartition == 0xFFFFFFFF)
			{
				tempSprings[numUnpartitionedSprings++] = i;
				continue;
			}

			//output springs
			orderedSprings[accumulatedSpringsPerPartition[availablePartition]++] = i;
		}

		PxU32 partitionStartIndex = 0;

		// handle the overflow of springs we couldn't partition above.
		while (numUnpartitionedSprings > 0)
		{
			//initialize the partition progress counter to be zero
			PxMemZero(partitionProgresses, sizeof(PxU32) * mNumParticles);

			partitionStartIndex += PARTICLE_MAX_NUM_PARTITIONS_TEMP;

			PxU32 newNumUnpartitionedSprings = 0;

			for (PxU32 i = 0; i < numUnpartitionedSprings; ++i)
			{
				const PxU32 springInd = tempSprings[i];
				const PxParticleSpring& spring = springs[springInd];

				const PxU32 availablePartition = computeSpringPartition(spring, partitionStartIndex, partitionProgresses);

				if (availablePartition == 0xFFFFFFFF)
				{
					tempSprings[newNumUnpartitionedSprings++] = springInd;
					continue;
				}

				//output springs
				orderedSprings[accumulatedSpringsPerPartition[availablePartition]++] = springInd;
			}

			numUnpartitionedSprings = newNumUnpartitionedSprings;
		}

		// at this point all of the springs are partitioned and in the ordered list.
	}

	void NpParticleClothPreProcessor::classifySprings(const PxParticleSpring* springs, PxU32* partitionProgresses, PxU32* tempSprings, physx::PxArray<PxU32>& tempSpringsPerPartition)
	{
		//initialize the partition progress counter to be zero
		PxMemZero(partitionProgresses, sizeof(PxU32) * mNumParticles);

		PxU32 numUnpartitionedSprings = 0;

		// Goes through all the springs and tries to partition, but will max out at 32 partitions (because we only have 32 bits)
		for (PxU32 i = 0; i < mNumSprings; ++i)
		{
			const PxParticleSpring& spring = springs[i];

			// will return the first partition where it's possible to place this spring.
			const PxU32 availablePartition = computeSpringPartition(spring, 0, partitionProgresses);

			if (availablePartition == 0xFFFFFFFF)
			{
				// we couldn't find a partition, so we add the index to this list for later.
				tempSprings[numUnpartitionedSprings++] = i;
				continue;
			}

			// tracks how many springs we have in each partition.
			tempSpringsPerPartition[availablePartition]++;
		}

		PxU32 partitionStartIndex = 0;

		// handle the overflow of the springs we couldn't partition above
		// we work in batches of 32 bits.
		while (numUnpartitionedSprings > 0)
		{
			//initialize the partition progress counter to be zero
			PxMemZero(partitionProgresses, sizeof(PxU32) * mNumParticles);

			partitionStartIndex += PARTICLE_MAX_NUM_PARTITIONS_TEMP;

			//Keep partitioning the un-partitioned constraints and blat the whole thing to 0!
			tempSpringsPerPartition.resize(PARTICLE_MAX_NUM_PARTITIONS_TEMP + tempSpringsPerPartition.size());
			PxMemZero(tempSpringsPerPartition.begin() + partitionStartIndex, sizeof(PxU32) * PARTICLE_MAX_NUM_PARTITIONS_TEMP);

			PxU32 newNumUnpartitionedSprings = 0;

			for (PxU32 i = 0; i < numUnpartitionedSprings; ++i)
			{
				const PxU32 springInd = tempSprings[i];

				const PxParticleSpring& spring = springs[springInd];

				const PxU32 availablePartition = computeSpringPartition(spring, partitionStartIndex, partitionProgresses);

				if (availablePartition == 0xFFFFFFFF)
				{
					tempSprings[newNumUnpartitionedSprings++] = springInd;
					continue;
				}

				tempSpringsPerPartition[availablePartition]++;
			}

			numUnpartitionedSprings = newNumUnpartitionedSprings;
		}

		// after all of this we have the number of springs per partition in tempSpringsPerPartition. we don't really know what spring will 
		// go where yet, that will follow later.
	}

	PxU32* NpParticleClothPreProcessor::partitions(const PxParticleSpring* springs, PxU32* orderedSpringIndices)
	{
		//each particle has a partition progress counter
		PxU32* tempPartitionProgresses = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * mNumParticles, "tempPartitionProgresses"));

		//this stores the spring index for the unpartitioned springs 
		PxU32* tempSprings = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * mNumSprings, "tempSprings"));

		PxArray<PxU32> tempSpringsPerPartition;
		tempSpringsPerPartition.reserve(PARTICLE_MAX_NUM_PARTITIONS_TEMP);
		tempSpringsPerPartition.forceSize_Unsafe(PARTICLE_MAX_NUM_PARTITIONS_TEMP);

		PxMemZero(tempSpringsPerPartition.begin(), sizeof(PxU32) * PARTICLE_MAX_NUM_PARTITIONS_TEMP);

		classifySprings(springs, tempPartitionProgresses, tempSprings, tempSpringsPerPartition);

		//compute number of partitions
		PxU32 maxPartitions = 0;
		for (PxU32 a = 0; a < tempSpringsPerPartition.size(); ++a, maxPartitions++)
		{
			if (tempSpringsPerPartition[a] == 0)
				break;
		}

		PxU32* tempAccumulatedSpringsPerPartition = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * maxPartitions, "mAccumulatedSpringsPerPartition"));
		mNbPartitions = maxPartitions; // save the current number of partitions

		//compute run sum
		PxU32 accumulation = 0;
		for (PxU32 a = 0; a < maxPartitions; ++a)
		{
			PxU32 count = tempSpringsPerPartition[a];
			tempAccumulatedSpringsPerPartition[a] = accumulation;
			accumulation += count;
		}
		PX_ASSERT(accumulation == mNumSprings);

		// this will assign the springs to partitions
		writeSprings(springs, tempPartitionProgresses, tempSprings,
			orderedSpringIndices, tempAccumulatedSpringsPerPartition);

#if 0 && PX_CHECKED
		//validate spring partitions
		for (PxU32 i = 0; i < mNumSprings; ++i)
		{
			PxU32 springInd = orderedSprings[i];
			for (PxU32 j = i + 1; j < mNumSprings; ++j)
			{
				PxU32 otherSpringInd = orderedSprings[j];
				PX_ASSERT(springInd != otherSpringInd);
			}
		}

		PxArray<bool> mFound(mNumParticles);
		PxU32 startIndex = 0;
		for (PxU32 i = 0; i < maxPartition; ++i)
		{
			PxU32 endIndex = tempAccumulatedSpringsPerPartition[i];
			PxMemZero(mFound.begin(), sizeof(bool) * mNumParticles);

			for (PxU32 j = startIndex; j < endIndex; ++j)
			{
				PxU32 tetrahedronIdx = orderedSprings[j];
				const PxParticleSpring& spring = springs[tetrahedronIdx];

				PX_ASSERT(!mFound[spring.ind0]);
				PX_ASSERT(!mFound[spring.ind1]);

				mFound[spring.ind0] = true;
				mFound[spring.ind1] = true;
			}

			startIndex = endIndex;
		}
#endif

		PX_FREE(tempPartitionProgresses);
		PX_FREE(tempSprings);
		return tempAccumulatedSpringsPerPartition;
	}

	PxU32 NpParticleClothPreProcessor::combinePartitions(const PxParticleSpring* springs, const PxU32* orderedSpringIndices, const PxU32* accumulatedSpringsPerPartition,
		PxU32* accumulatedSpringsPerCombinedPartition, PxParticleSpring* orderedSprings, PxU32* accumulatedCopiesPerParticles, PxU32* remapOutput)
	{
		// reduces the number of partitions from mNbPartitions to PARTICLE_MAX_NUM_PARTITIONS_FINAL

		const PxU32 nbPartitions = mNbPartitions;
		mNbPartitions = PARTICLE_MAX_NUM_PARTITIONS_FINAL;

		PxMemZero(accumulatedSpringsPerCombinedPartition, sizeof(PxU32) * PARTICLE_MAX_NUM_PARTITIONS_FINAL);

		// ceil(nbPartitions/maxPartitions) -basically the number of "repetitions" before combining. Example, MAX_FINAL is 8, we have 20 total, so this is 3.
		const PxU32 maxAccumulatedCP = (nbPartitions + PARTICLE_MAX_NUM_PARTITIONS_FINAL - 1) / PARTICLE_MAX_NUM_PARTITIONS_FINAL;

		// enough space for all partitions.
		const PxU32 partitionArraySize = maxAccumulatedCP * PARTICLE_MAX_NUM_PARTITIONS_FINAL;

		// for each particle, have a table of all partitions.
		const PxU32 nbPartitionTables = partitionArraySize * mNumParticles;

		// per-particle, stores whether particle is part of partition
		PxU32* tempPartitionTablePerVert = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * nbPartitionTables, "tempPartitionTablePerVert"));
		// per-particle, stores remapping ?????
		PxU32* tempRemapTablePerVert = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * nbPartitionTables, "tempRemapTablePerVert"));

		// per-particle, stores the number of copies for each particle.
		PxU32* tempNumCopiesEachVerts = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * mNumParticles, "tempNumCopiesEachVerts"));
		PxMemZero(tempNumCopiesEachVerts, sizeof(PxU32) * mNumParticles);

		//initialize partitionTablePerVert
		for (PxU32 i = 0; i < nbPartitionTables; ++i)
		{
			tempPartitionTablePerVert[i] = 0xffffffff;
			tempRemapTablePerVert[i] = 0xffffffff;
		}

		// combine partitions:
		// let PARTICLE_MAX_NUM_PARTITIONS_FINAL be 8 and the current number of partitions be 20
		// this will merge partition 0, 8, 16 into the first partition,
		// then put 1, 9, 17 into the second partition, etc.

		// we move all the springs of partition x*PARTICLE_MAX_NUM_PARTITION_FINAL to the end of partition x,
		// using the count variable.

		// output of this stage
		// orderedSprings - has springs per partition
		// accumulatedSpringsPerCombinedPartition - has number of springs of each partition
		// tempPartitionTablePerVert - has spring index of spring that connects to this particle in this partition.

		mMaxSpringsPerPartition = 0;
		PxU32 count = 0;
		for (PxU32 i = 0; i < PARTICLE_MAX_NUM_PARTITIONS_FINAL; ++i)
		{
			PxU32 totalSpringsInPartition = 0;
			for (PxU32 j = 0; j < maxAccumulatedCP; ++j)
			{
				PxU32 partitionId = i + PARTICLE_MAX_NUM_PARTITIONS_FINAL * j;
				if (partitionId < nbPartitions)
				{
					const PxU32 startInd = partitionId == 0 ? 0 : accumulatedSpringsPerPartition[partitionId - 1];
					const PxU32 endInd = accumulatedSpringsPerPartition[partitionId];
					const PxU32 index = i * maxAccumulatedCP + j;

					for (PxU32 k = startInd; k < endInd; ++k)
					{
						const PxU32 springInd = orderedSpringIndices[k];

						const PxParticleSpring& spring = springs[springInd];

						orderedSprings[count] = spring;

						PX_ASSERT(spring.ind0 != spring.ind1);
						tempPartitionTablePerVert[spring.ind0 * partitionArraySize + index] = count;
						tempPartitionTablePerVert[spring.ind1 * partitionArraySize + index] = count + mNumSprings;

						count++;
					}

					totalSpringsInPartition += (endInd - startInd);
				}
			}

			accumulatedSpringsPerCombinedPartition[i] = count;
			mMaxSpringsPerPartition = PxMax(mMaxSpringsPerPartition, totalSpringsInPartition);
		}

		PX_ASSERT(count == mNumSprings);

		PxMemZero(tempNumCopiesEachVerts, sizeof(PxU32) * mNumParticles);
		bool* tempHasOccupied = reinterpret_cast<bool*>(PX_ALLOC(sizeof(bool) * partitionArraySize, "tempOrderedSprings"));

		// compute num of copies and remap index
		//
		// remap table - builds a chain of indices of the same particle across partitions
		// if particle x is at index y in partition 1, we build a table such that particle x in partition 2 can look up the index in partition 1
		// This basically maintains the gauss-seidel part of the solver, where each partition works on the results of the another partition.
		// This remap table is build across combined partitions. So there will never be a remap into the same partition.
		// 
		// numCopies is then the number of final copies, meaning all copies of each particle that don't have a remap into one of the following 
		// partitions.
		//
		for (PxU32 i = 0; i < mNumParticles; ++i)
		{
			// for each particle, use this list to track which partition is occupied.
			PxMemZero(tempHasOccupied, sizeof(bool) * partitionArraySize);

			// partition table has size numPartitions for each particle, tells you the spring index for this partition
			const PxU32* partitionTable = &tempPartitionTablePerVert[i * partitionArraySize];
			// remapTable is still empty (0xFFFFFFFF)
			PxU32* remapTable = &tempRemapTablePerVert[i * partitionArraySize];

			// for all of the final partitions.
			for (PxU32 j = 0; j < PARTICLE_MAX_NUM_PARTITIONS_FINAL; ++j)
			{
				// start index of this combined partition in the initial partition array
				const PxU32 startInd = j * maxAccumulatedCP;
				// start index if the next combined partition in the initial partition array
				PxU32 nextStartInd = (j + 1) * maxAccumulatedCP;

				// for our 8/20 example, that would be startInd 0, nextStartInd 3 because every
				// final partition will be combined from 3 partitions.

				// go through the indices of this combined partition (0-2)
				for (PxU32 k = 0; k < maxAccumulatedCP; ++k)
				{
					const PxU32 index = startInd + k;
					if (partitionTable[index] != 0xffffffff)
					{
						// there is a spring in this partition connected to this particle

						bool found = false;

						// look at the next partition, potentially also further ahead to figure out if there is any other partition having this particle index.
						for (PxU32 h = nextStartInd; h < partitionArraySize; ++h)
						{
							// check if any of the partitions in this combined partition is occupied.
							const PxU32 remapInd = partitionTable[h];
							if (remapInd != 0xffffffff && !tempHasOccupied[h])
							{
								// if it is, and none of the other partitions in the partition before already remapped to that one
								remapTable[index] = remapInd;	// maps from partition i to one of the next ones.
								found = true;
								tempHasOccupied[h] = true;		// mark as occupied
								nextStartInd++;					// look one more (initial!) partition ahead for next remap.
								break;
							}
						}

						if (!found)
						{
							tempNumCopiesEachVerts[i]++; // if not found, add one more copy as there won't be any follow-up partition taking this position as an input.
						}
					}
				}
			}
		}


		const PxU32 totalNumVerts = mNumSprings * 2;

		// compute a runSum for the number of copies for each particle
		PxU32 totalCopies = 0;
		for (PxU32 i = 0; i < mNumParticles; ++i)
		{
			totalCopies += tempNumCopiesEachVerts[i];
			accumulatedCopiesPerParticles[i] = totalCopies;
		}

		const PxU32 remapOutputSize = totalNumVerts + totalCopies;

		// fill the output of the remap
		//
		// for all particle copies that are at the end of a remap chain, calculate the remap
		// into the final accumulation buffer.
		// 
		// the final accumulation buffer will have numCopies entries for each particle.
		//
		for (PxU32 i = 0; i < mNumParticles; ++i)
		{
			const PxU32 index = i * partitionArraySize;
			const PxU32* partitionTable = &tempPartitionTablePerVert[index];
			PxU32* remapTable = &tempRemapTablePerVert[index];

			PxU32 accumulatedCount = 0;
			for (PxU32 j = 0; j < partitionArraySize; ++j)
			{
				const PxU32 vertInd = partitionTable[j];
				if (vertInd != 0xffffffff)
				{
					PxU32 remapInd = remapTable[j];

					//this remap is in the accumulation buffer
					if (remapInd == 0xffffffff)
					{
						const PxU32 start = i == 0 ? 0 : accumulatedCopiesPerParticles[i - 1];
						remapInd = totalNumVerts + start + accumulatedCount;
						accumulatedCount++;
					}
					PX_ASSERT(remapInd < remapOutputSize);
					remapOutput[vertInd] = remapInd;
				}
			}

		}

		PX_FREE(tempHasOccupied);
		PX_FREE(tempPartitionTablePerVert);
		PX_FREE(tempRemapTablePerVert);
		PX_FREE(tempNumCopiesEachVerts);

		return remapOutputSize;
	}

	void NpParticleClothPreProcessor::partitionSprings(const PxParticleClothDesc& clothDesc, PxPartitionedParticleCloth& output)
	{
		mNumSprings = clothDesc.nbSprings;
		mNumParticles = clothDesc.nbParticles;

		// prepare the output
		output.nbSprings = clothDesc.nbSprings;
		output.nbCloths = clothDesc.nbCloths;
		output.allocateBuffers(mNumParticles, mCudaContextManager);

		// will create a temp partitioning with too many partitions
		PxU32* orderedSpringIndices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * mNumSprings, "orderedSpringIndices"));
		PxU32* accumulatedSpringsPerPartitionTemp = partitions(clothDesc.springs, orderedSpringIndices);

		// combine these partitions to a max of PARTICLE_MAX_NUM_PARTITIONS_FINAL
		// build remap chains and accumulation buffer.
		output.remapOutputSize = combinePartitions(clothDesc.springs, orderedSpringIndices, accumulatedSpringsPerPartitionTemp, output.accumulatedSpringsPerPartitions,
			output.orderedSprings, output.accumulatedCopiesPerParticles, output.remapOutput);
	
		// get the max number of partitions for each cloth
		// AD Todo: figure out why blendScale is computed like this.
		PxParticleCloth* cloths = clothDesc.cloths;
		for (PxU32 i = 0; i < clothDesc.nbCloths; ++i)
		{
			PxU32 maxPartitions = 0;
			
			for (PxU32 p = cloths[i].startVertexIndex, endIndex = cloths[i].startVertexIndex + cloths[i].numVertices;
				p < endIndex; p++)
			{
				PxU32 copyStart = p == 0 ? 0 : output.accumulatedCopiesPerParticles[p - 1];
				PxU32 copyEnd = output.accumulatedCopiesPerParticles[p];

				maxPartitions = PxMax(maxPartitions, copyEnd - copyStart);
			}

			cloths[i].clothBlendScale = 1.f / (maxPartitions + 1);
		}

		// sort the cloths in this clothDesc according to their startVertexIndex into the particle list.
		PxSort(cloths, clothDesc.nbCloths);

		// reorder such that things still match after the sorting.
		for (PxU32 i = 0; i < clothDesc.nbCloths; ++i)
		{
			output.sortedClothStartIndices[i] = cloths[i].startVertexIndex;
			output.cloths[i] = cloths[i];
		}

		output.nbPartitions = mNbPartitions;
		output.maxSpringsPerPartition = mMaxSpringsPerPartition;

		PX_FREE(accumulatedSpringsPerPartitionTemp);
		PX_FREE(orderedSpringIndices);
	}

	void NpParticleClothPreProcessor::release()
	{
		PX_DELETE_THIS;
	}

	///////////////////////////////////////////////////////////////////////////////////////

	NpPBDParticleSystem::NpPBDParticleSystem(PxU32 maxNeighborhood, PxCudaContextManager& cudaContextManager) :
		NpParticleSystem<PxPBDParticleSystem>(cudaContextManager, PxConcreteType::ePBD_PARTICLESYSTEM, NpType::ePBD_PARTICLESYSTEM, PxActorType::ePBD_PARTICLESYSTEM)
	{
		//PX_ASSERT(mCudaContextManager);
		setSolverType(PxParticleSolverType::ePBD);
		mCore.getShapeCore().initializeLLCoreData(maxNeighborhood);
		enableCCD(false);
	}
	

	PxU32 NpPBDParticleSystem::createPhase(PxParticleMaterial* material, const PxParticlePhaseFlags flags)
	{
		if (material->getConcreteType() == PxConcreteType::ePBD_MATERIAL)
		{
			Sc::ParticleSystemShapeCore& shapeCore = mCore.getShapeCore();
			Dy::ParticleSystemCore& core = shapeCore.getLLCore();

			PxU16 materialHandle = static_cast<NpPBDMaterial*>(material)->mMaterial.mMaterialIndex;
		
			const PxU32 groupID = mNextPhaseGroupID++;

			core.mPhaseGroupToMaterialHandle.pushBack(materialHandle);

			if (mCore.getSim())
				mCore.getSim()->getLowLevelParticleSystem()->mFlag |= Dy::ParticleSystemFlag::eUPDATE_PHASE;

			return (groupID & PxParticlePhaseFlag::eParticlePhaseGroupMask)
				| (PxU32(flags) & PxParticlePhaseFlag::eParticlePhaseFlagsMask);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxPBDParticleSystem:createPhase(): the provided material is not supported by this type of particle system.");
			return 0;
		}
	}

	void NpPBDParticleSystem::release()
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		//	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, PxArticulationBase::userData);

		if (npScene)
		{
			npScene->scRemoveParticleSystem(*this);
			npScene->removeFromParticleSystemList(*this);
		}

		PX_ASSERT(!isAPIWriteForbidden());
		NpDestroyParticleSystem(this);
	}

	void NpPBDParticleSystem::addParticleBuffer(PxParticleBuffer* clothBuffer)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpPBDParticleSystem::addParticleBuffer: this function cannot be called when the particle system is not inserted into the scene!");
			
		mCore.getShapeCore().addParticleBuffer(clothBuffer);
	}

	void NpPBDParticleSystem::removeParticleBuffer(PxParticleBuffer* clothBuffer)
	{
		mCore.getShapeCore().removeParticleBuffer(clothBuffer);
	}

#if PX_ENABLE_DEBUG_VISUALIZATION
	void NpPBDParticleSystem::visualize(PxRenderOutput& out, NpScene& npScene)	const
	{
		visualizeParticleSystem(out, npScene, mCore);
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

	static void internalAddRigidAttachment(PxRigidActor* actor, Sc::ParticleSystemCore& psCore)
	{
		Sc::BodyCore* core = getBodyCore(actor);

		psCore.addRigidAttachment(core);
	}

	static void internalRemoveRigidAttachment(PxRigidActor* actor, Sc::ParticleSystemCore& psCore)
	{
		Sc::BodyCore* core = getBodyCore(actor);
		psCore.removeRigidAttachment(core);
	}

	void NpPBDParticleSystem::addRigidAttachment(PxRigidActor* actor)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpPBDParticleSystem::addRigidAttachment: particleSystem must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpPBDParticleSystem::addRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpPBDParticleSystem::addRigidAttachment: Illegal to call while simulation is running.");

		internalAddRigidAttachment(actor, mCore);
	}

	void NpPBDParticleSystem::removeRigidAttachment(PxRigidActor* actor)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpPBDParticleSystem::removeRigidAttachment: particleSystem must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpPBDParticleSystem::removeRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpPBDParticleSystem::removeRigidAttachment: Illegal to call while simulation is running.");

		internalRemoveRigidAttachment(actor, mCore);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	NpFLIPParticleSystem::NpFLIPParticleSystem(PxCudaContextManager& cudaContextManager) :
		NpParticleSystem<PxFLIPParticleSystem>(cudaContextManager, PxConcreteType::eFLIP_PARTICLESYSTEM, NpType::eFLIP_PARTICLESYSTEM, PxActorType::eFLIP_PARTICLESYSTEM)
	{
		//PX_ASSERT(mCudaContextManager);
		setSolverType(PxParticleSolverType::eFLIP);
		mCore.getShapeCore().initializeLLCoreData(0);
		enableCCD(false);
	}

	PxU32 NpFLIPParticleSystem::createPhase(PxParticleMaterial* material, const PxParticlePhaseFlags flags)
	{
		if (material->getConcreteType() == PxConcreteType::eFLIP_MATERIAL)
		{
			Sc::ParticleSystemShapeCore& shapeCore = mCore.getShapeCore();
			Dy::ParticleSystemCore& core = shapeCore.getLLCore();

			PxU16 materialHandle = static_cast<NpFLIPMaterial*>(material)->mMaterial.mMaterialIndex;

			const PxU32 groupID = mNextPhaseGroupID++;

			core.mPhaseGroupToMaterialHandle.pushBack(materialHandle);

			if (mCore.getSim())
				mCore.getSim()->getLowLevelParticleSystem()->mFlag |= Dy::ParticleSystemFlag::eUPDATE_PHASE;

			return (groupID & PxParticlePhaseFlag::eParticlePhaseGroupMask)
				| (PxU32(flags) & PxParticlePhaseFlag::eParticlePhaseFlagsMask);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxFLIPParticleSystem:createPhase(): the provided material is not supported by this type of particle system.");
			return 0;
		}
	}

	void NpFLIPParticleSystem::getSparseGridCoord(PxI32& x, PxI32& y, PxI32& z, PxU32 id)
	{
		const PxI32 iid = static_cast<PxI32>(id);
		x = iid % MAX_SPARSEGRID_DIM + MIN_SPARSEGRID_ID;
		y = (iid / MAX_SPARSEGRID_DIM) % MAX_SPARSEGRID_DIM + MIN_SPARSEGRID_ID;
		z = iid / MAX_SPARSEGRID_DIM / MAX_SPARSEGRID_DIM + MIN_SPARSEGRID_ID;
	}

	void NpFLIPParticleSystem::release()
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		//	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, PxArticulationBase::userData);

		if (npScene)
		{
			npScene->scRemoveParticleSystem(*this);
			npScene->removeFromParticleSystemList(*this);
		}

		PX_ASSERT(!isAPIWriteForbidden());
		NpDestroyParticleSystem(this);
	}

	void* NpFLIPParticleSystem::getSparseGridDataPointer(PxSparseGridDataFlag::Enum flags)
	{
		NpScene* scene = getNpScene();
		if (!scene)
			return NULL;

		if ((flags & (PxSparseGridDataFlag::eSUBGRID_MASK | PxSparseGridDataFlag::eSUBGRID_ID
			| PxSparseGridDataFlag::eGRIDCELL_SOLID_GRADIENT_AND_SDF | PxSparseGridDataFlag::eGRIDCELL_SOLID_VELOCITY
			| PxSparseGridDataFlag::eGRIDCELL_FLUID_SDF | PxSparseGridDataFlag::eGRIDCELL_FLUID_VELOCITY
			)) == 0)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxParticleSystem::getSparseGridDataPointer, specified data is not available.");
			return NULL;
		}

		NP_READ_CHECK(scene);

		return scene->getSimulationController()->getSparseGridDataPointer(*getCore().getSim()->getLowLevelParticleSystem(), flags, getCore().getSolverType());
	}

#if PX_ENABLE_DEBUG_VISUALIZATION
	void NpFLIPParticleSystem::visualize(PxRenderOutput& out, NpScene& npScene)	const
	{
		visualizeParticleSystem(out, npScene, mCore);
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

	void NpFLIPParticleSystem::addParticleBuffer(PxParticleBuffer* particleBuffer)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpFLIPParticleSystem::addParticleBuffer: this function cannot be called when the particle system is not inserted into the scene!");

		PxType type = particleBuffer->getConcreteType();
		if (type == PxConcreteType::ePARTICLE_BUFFER || type == PxConcreteType::ePARTICLE_DIFFUSE_BUFFER)
		{
			mCore.getShapeCore().addParticleBuffer(particleBuffer);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "NpFLIPParticleSystem:addParticleBuffer(): the provided buffer type is not supported by this type of particle system.");
		}
	}

	void NpFLIPParticleSystem::removeParticleBuffer(PxParticleBuffer* particleBuffer)
	{
		PxType type = particleBuffer->getConcreteType();
		if (type == PxConcreteType::ePARTICLE_BUFFER || type == PxConcreteType::ePARTICLE_DIFFUSE_BUFFER)
		{
			mCore.getShapeCore().removeParticleBuffer(particleBuffer);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "NpFLIPParticleSystem:addParticleBuffer(): the provided buffer type is not supported by this type of particle system.");
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	NpMPMParticleSystem::NpMPMParticleSystem(PxCudaContextManager& cudaContextManager) :
		NpParticleSystem<PxMPMParticleSystem>(cudaContextManager, PxConcreteType::eMPM_PARTICLESYSTEM, NpType::eMPM_PARTICLESYSTEM, PxActorType::eMPM_PARTICLESYSTEM)
	{
		//PX_ASSERT(mCudaContextManager);
		setSolverType(PxParticleSolverType::eMPM);
		mCore.getShapeCore().initializeLLCoreData(0);
		enableCCD(false);
	}

	PxU32 NpMPMParticleSystem::createPhase(PxParticleMaterial* material, const PxParticlePhaseFlags flags)
	{
		if (material->getConcreteType() == PxConcreteType::eMPM_MATERIAL)
		{
			Sc::ParticleSystemShapeCore& shapeCore = mCore.getShapeCore();
			Dy::ParticleSystemCore& core = shapeCore.getLLCore();

			PxU16 materialHandle = static_cast<NpMPMMaterial*>(material)->mMaterial.mMaterialIndex;

			const PxU32 groupID = mNextPhaseGroupID++;

			core.mPhaseGroupToMaterialHandle.pushBack(materialHandle);

			if (mCore.getSim())
				mCore.getSim()->getLowLevelParticleSystem()->mFlag |= Dy::ParticleSystemFlag::eUPDATE_PHASE;

			return (groupID & PxParticlePhaseFlag::eParticlePhaseGroupMask)
				| (PxU32(flags) & PxParticlePhaseFlag::eParticlePhaseFlagsMask);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxMPMParticleSystem:createPhase(): the provided material is not supported by this type of particle system.");
			return 0;
		}
	}

	void NpMPMParticleSystem::getSparseGridCoord(PxI32& x, PxI32& y, PxI32& z, PxU32 id)
	{
		const PxI32 iid = static_cast<PxI32>(id);
		x = iid % MAX_SPARSEGRID_DIM + MIN_SPARSEGRID_ID;
		y = (iid / MAX_SPARSEGRID_DIM) % MAX_SPARSEGRID_DIM + MIN_SPARSEGRID_ID;
		z = iid / MAX_SPARSEGRID_DIM / MAX_SPARSEGRID_DIM + MIN_SPARSEGRID_ID;
	}

	void NpMPMParticleSystem::release()
	{
		NpScene* npScene = getNpScene();
		NP_WRITE_CHECK(npScene);

		//	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, PxArticulationBase::userData);

		if (npScene)
		{
			npScene->scRemoveParticleSystem(*this);
			npScene->removeFromParticleSystemList(*this);
		}

		PX_ASSERT(!isAPIWriteForbidden());
		NpDestroyParticleSystem(this);
	}

	void* NpMPMParticleSystem::getSparseGridDataPointer(PxSparseGridDataFlag::Enum flags)
	{
		if ((flags & (PxSparseGridDataFlag::eSUBGRID_MASK | PxSparseGridDataFlag::eSUBGRID_ID
			| PxSparseGridDataFlag::eGRIDCELL_SOLID_GRADIENT_AND_SDF | PxSparseGridDataFlag::eGRIDCELL_SOLID_VELOCITY
			| PxSparseGridDataFlag::eGRIDCELL_FLUID_SDF | PxSparseGridDataFlag::eGRIDCELL_FLUID_VELOCITY
			)) == 0)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxParticleSystem::getSparseGridDataPointer, specified data is not available.");
			return NULL;
		}

		NP_READ_CHECK(getNpScene());

		NpScene* scene = getNpScene();
		return scene->getSimulationController()->getSparseGridDataPointer(*getCore().getSim()->getLowLevelParticleSystem(), flags, getCore().getSolverType());
	}

	void* NpMPMParticleSystem::getMPMDataPointer(PxMPMParticleDataFlag::Enum flags)
	{
		NpScene* scene = getNpScene();

		if (!scene)
			return NULL;

		if ((flags & (PxMPMParticleDataFlag::eMPM_AFFINE_C1 | PxMPMParticleDataFlag::eMPM_AFFINE_C2 | PxMPMParticleDataFlag::eMPM_AFFINE_C3
			| PxMPMParticleDataFlag::eMPM_DEFORMATION_GRADIENT_F1 | PxMPMParticleDataFlag::eMPM_DEFORMATION_GRADIENT_F2 | PxMPMParticleDataFlag::eMPM_DEFORMATION_GRADIENT_F3)) == 0)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxMPMParticleSystem::copyData, specified data is not available.");
			return NULL;
		}

		NP_READ_CHECK(scene);
		
		return scene->getSimulationController()->getMPMDataPointer(*getCore().getSim()->getLowLevelParticleSystem(), flags);
	}

#if PX_ENABLE_DEBUG_VISUALIZATION
	void NpMPMParticleSystem::visualize(PxRenderOutput& out, NpScene& npScene)	const
	{
		visualizeParticleSystem(out, npScene, mCore);
	}
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

	void NpMPMParticleSystem::addParticleBuffer(PxParticleBuffer* particleBuffer)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpMPMParticleSystem::addParticleBuffer: this function cannot be called when the particle system is not inserted into the scene!");

		if (particleBuffer->getConcreteType() == PxConcreteType::ePARTICLE_BUFFER)
		{
			mCore.getShapeCore().addParticleBuffer(particleBuffer);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "NpMPMParticleSystem:addParticleBuffer(): the provided buffer type is not supported by this type of particle system.");
		}
	}

	void NpMPMParticleSystem::removeParticleBuffer(PxParticleBuffer* particleBuffer)
	{
		if (particleBuffer->getConcreteType() == PxConcreteType::ePARTICLE_BUFFER)
		{
			mCore.getShapeCore().removeParticleBuffer(particleBuffer);
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "NpMPMParticleSystem:addParticleBuffer(): the provided buffer type is not supported by this type of particle system.");
		}
	}

	void NpMPMParticleSystem::addRigidAttachment(PxRigidActor* actor)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpMPMParticleSystem::addRigidAttachment: particleSystem must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpMPMParticleSystem::addRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpMPMParticleSystem::addRigidAttachment: Illegal to call while simulation is running.");

		internalAddRigidAttachment(actor, mCore);
	}

	void NpMPMParticleSystem::removeRigidAttachment(PxRigidActor* actor)
	{
		NP_WRITE_CHECK(getNpScene());
		PX_CHECK_AND_RETURN(getNpScene() != NULL, "NpMPMParticleSystem::removeRigidAttachment: particleSystem must be inserted into the scene.");
		PX_CHECK_AND_RETURN((actor == NULL || actor->getScene() != NULL), "NpMPMParticleSystem::removeRigidAttachment: actor must be inserted into the scene.");

		PX_CHECK_SCENE_API_WRITE_FORBIDDEN(getNpScene(), "NpMPMParticleSystem::removeRigidAttachment: Illegal to call while simulation is running.");

		internalRemoveRigidAttachment(actor, mCore);
	}
#endif

}

physx::PxParticleClothPreProcessor* PxCreateParticleClothPreProcessor(physx::PxCudaContextManager* cudaContextManager)
{
	physx::PxParticleClothPreProcessor* processor = PX_NEW(physx::NpParticleClothPreProcessor)(cudaContextManager);
	return processor;
}


#endif //PX_SUPPORT_GPU_PHYSX
