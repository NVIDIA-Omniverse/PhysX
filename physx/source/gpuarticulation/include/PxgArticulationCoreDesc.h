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

#ifndef PXG_ARTICULATION_CORE_DESC_H
#define PXG_ARTICULATION_CORE_DESC_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "PxgArticulation.h"

namespace physx
{
	struct PxgBodySim;
	struct PxgSolverBodySleepData;

	class PxGpuTendonAttachmentData;
	class PxGpuTendonJointCoefficientData;

	namespace IG
	{
		class NodeIndex;
	};

	namespace Cm
	{
		struct UnAlignedSpatialVector;
	}

	struct PxgArticulationCoreDesc
	{
		PxgBodySim*						mBodySimBufferDeviceData;
		PxgSolverBodySleepData*			articulationSleepData; //sleep data for the articulation
		PxNodeIndex*					islandNodeIndices;
		PxU32*							solverBodyIndices;

		PxgArticulation*				articulations;
		PxU32							articulationOffset;//offset to the islandNodeIndices
		PxU32							nbArticulations;
		PxReal							dt;
		PxVec3							gravity;
		PxReal							invLengthScale;
		bool							isExternalForcesEveryTgsIterationEnabled;

		Cm::UnAlignedSpatialVector*		impulses;
		PxU32							nbSlabs;
		PxU32							nbPartitions;
		uint2*							slabHasChanges; // one uint2 per articulation per slab. Stores up to two link indices to communicate which links holds an impulse from the solver.
		uint4*							slabDirtyMasks;  // one uint4 [linkIndexA, writeIndexA, linkIndexB, writeIndexB] per articulation per partition per slab, index = articulationId + slab * nbArticulations + partitionId * nbArticulations * nbSlabs. Used for internal velocity propagation (artiPropagateVelocityInternal).

		//The dirty paths we need to process for each partition. This defines
		//how we propagate impulses in the articulations. There is one of these
		//per partition, per articulation
		PxgArticulationBitFieldStackData*				mPathToRootsPerPartition;
		//This defines which link is holding the current accumulated impulse. There is
		//one entry per articulation per partition
		PxU32*											mImpulseHoldingLink;
		PxReal*											mPartitionAverageScale; // One float per partition per articulation. Stores 1/numSlabsInThisPartitionInvolvingThisArticulation, see artiComputeDependencies

		PxgArticulationBlockData*						mArticulationBlocks;
		PxgArticulationBlockLinkData*					mArticulationLinkBlocks;
		PxgArticulationTraversalStackData*				mArticulationTraversalStackBlocks;
		PxgArticulationBitFieldStackData*				mTempPathToRootBitFieldBlocks;
		PxgArticulationBitFieldStackData*				mTempSharedBitFieldBlocks;
		PxgArticulationBitFieldStackData*				mTempRootBitFieldBlocks;

		//A quick reminder of the indexing of mPathToRootBitFieldBlocks. 
		//A bitfield is just a PxU64. It can describe the path to root for a single link in an articulation with 64 links.
		//We support more than 64 links so we need a bitfield array to describe the path to root for a single link.
		//For each link we need bitfield[maxWordCount] with maxWordCount = (maxLinks+63)/64 to describe the path to the root.
		//For each articulation we need bitField[maxLinks*maxWordCount] to describe the path to root for all links.
		//The array of bitfields for an articulation with globalWarpIndex will begin at mPathToRootBitFieldBlocks[globalWarpIndex*(maxLinks*maxWordCount))
		PxgArticulationBitFieldData*					mPathToRootBitFieldBlocks;

		PxgArticulationBlockDofData*					mArticulationDofBlocks;

		PxgArticulationBlockSpatialTendonData*			mArticulationSpatialTendonBlocks;
		PxgArticulationInternalTendonConstraintData*	mArticulationSpatialTendonConstraintBlocks;
		PxgArticulationBlockAttachmentData*				mArticulationAttachmentBlocks;

		PxgArticulationBlockFixedTendonData*			mArticulationFixedTendonBlocks;
		PxgArticulationInternalTendonConstraintData*	mArticulationFixedTendonConstraintBlocks;
		PxgArticulationBlockTendonJointData*			mArticulationTendonJointBlocks;

		PxgArticulationBlockMimicJointData*				mArticulationMimicJointBlocks;

		PxU32							mMaxLinksPerArticulation;
		PxU32							mMaxDofsPerArticulation;
		PxU32							mMaxMimicJointsPerArticulation;
		PxU32							mMaxSpatialTendonsPerArticulation;
		PxU32							mMaxAttachmentPerArticulation;

		PxU32							mMaxFixedTendonsPerArticulation;
		PxU32							mMaxTendonJointPerArticulation;

		PxU32*							mTempContactUniqueIndicesBlock;
		PxU32*							mTempConstraintUniqueIndicesBlock;
		PxU32*							mTempContactHeaderBlock;
		PxU32*							mTempConstraintHeaderBlock;

		PxU32*							mTempSelfContactUniqueIndicesBlock;
		PxU32*							mTempSelfConstraintUniqueIndicesBlock;
		PxU32*							mTempSelfContactHeaderBlock;
		PxU32*							mTempSelfConstraintHeaderBlock;

		Dy::ErrorAccumulator			mContactErrorAccumulator;
	};

	struct PxgArticulationOutputDesc
	{
	public:
		//see PxgArticulationLinkJointRootStateData
		PxU8*								linkAndJointAndRootStateData;
		//PxReal*								jointPosition_Vel_Accel;
		PxgSolverBodySleepData*				sleepData;
		Dy::ErrorAccumulator*				errorAccumulator; //Per articulation, collects internal residuals (no contacts or external PxJoints connected to the articulation)
		Dy::ErrorAccumulator*				contactResidualAccumulator; //Only one value accumulating contact residuals over all articulations
	};
}

#endif
