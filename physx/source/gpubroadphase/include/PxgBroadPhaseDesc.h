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

#ifndef PXG_BROADPHASE_DESC_H
#define PXG_BROADPHASE_DESC_H

#include "foundation/PxSimpleTypes.h"

// PT: the GPU AABB manager apparently DMAs the updated handles' *bitmap* to the GPU directly, bypassing the
// BP API. This creates coupling between the GPU BP and the GPU AABB manager, i.e. the GPU BP cannot be used
// outside of the SDK as a standalone BP. The code controlled by this define tries to undo that and support
// a "regular" update handles array (not a bitmap), like the CPU implementations do.
#define SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU

namespace physx
{
	class PxgIntegerAABB;
	class PxgSapBox1D;
	struct PxgBroadPhasePair;
	struct PxgHandleRegion;
	struct PxgIntegerRegion;
	class PxBounds3;

	namespace Bp
	{
		struct VolumeData;
	}

	typedef PxU64	regionOverlapType;

	struct PxgBroadPhaseDesc
	{
		PxU32*					updateData_createdHandles;				// PT: copy of updateData buffer in device memory
		PxU32					numCreatedHandles;

		PxU32*					updateData_removedHandles;				// PT: copy of updateData buffer in device memory
		PxU32					numRemovedHandles;

#ifdef SUPPORT_UPDATE_HANDLES_ARRAY_FOR_GPU
		PxU32*					updateData_updatedHandles;				// PT: copy of updateData buffer in device memory
		PxU32					numUpdatedHandles;
#endif

		PxU32*					aabbMngr_changedHandleMap;				// PT: data coming from the AABB manager, creating all the coupling problems
		PxU32					aabbMngr_changedHandleBitMapWordCounts;	// PT: data coming from the AABB manager, creating all the coupling problems
		PxU32*					aabbMngr_addedHandleMap;				// PT: data coming from the AABB manager, creating all the coupling problems
		PxU32*					aabbMngr_removedHandleMap;				// PT: data coming from the AABB manager, creating all the coupling problems
		PxU32*					aabbMngr_aggregatedBoundHandles;		// PT: data coming from the AABB manager, creating all the coupling problems

		PxBounds3*				updateData_fpBounds;					// PT: copy of updateData buffer in device memory
		PxReal*					updateData_contactDistances;			// PT: copy of updateData buffer in device memory
		PxgIntegerAABB*			newIntegerBounds;						// PT: computed by translateAABBsLaunch kernel.
		PxU32*					updateData_groups;						// PT: copy of updateData buffer in device memory TODO: type could be better
		PxU32*					updateData_envIDs;						// PT: copy of updateData buffer in device memory

		PxgIntegerAABB*			oldIntegerBounds;
		PxgSapBox1D*			boxSapBox1D[3];
		PxgSapBox1D*			boxNewSapBox1D[3];
	
		PxU32*					boxProjections[3];
		PxU32*					boxHandles[2][3];
		PxU32					numPreviousHandles;
		PxU32					sharedFoundPairIndex;
		PxU32					sharedLostPairIndex;

		PxU32					sharedFoundAggPairIndex;
		PxU32					sharedLostAggPairIndex;
		
		PxgBroadPhasePair*		foundPairReport;			//device memory for GPU foundReport (include actors and aggregates)
		PxgBroadPhasePair*		lostPairReport;				//device memory for GPU lostReport (include actors and aggregates)

		PxgBroadPhasePair*		foundAggPairReport;			//device memory for GPU aggregate foundReport
		PxgBroadPhasePair*		lostAggPairReport;			//device memory for GPU aggregate lostReport

		PxgBroadPhasePair*		foundActorPairReport;		//device memory for GPU actor foundReport
		PxgBroadPhasePair*		lostActorPairReport;		//device memory for GPU actor lostReport

		PxgBroadPhasePair*		foundPairReportMap;			//mapped address in the GPU for the cpu foundReport for actor pairs (not include aggregate);
		PxgBroadPhasePair*		lostPairReportMap;			//mapped address in the GPU for the cpu lostReport  for actor pairs(not include aggregate);

		Bp::VolumeData*			aabbMngr_volumeData;		// PT: data coming from the AABB manager, creating all the coupling problems

		PxU32*					activeRegionsHistogram;		//! Histogram used for active handles, 64 regions and each regions has nbProjections(multiply of 4) elements
		PxU32*					startRegionsHistogram;		//! Histogram for all start handles
		PxU32*					orderedActiveRegionHandles;	//! An ordered list of active handles
		PxU32*					orderedStartRegionHandles;	//! An ordered list of start handles 

		regionOverlapType*		blockOverlapChecksRegion;
		regionOverlapType*		overlapChecksRegion;		//this variable is to store the exclusive scan add overlap checks for each objects
		regionOverlapType		overlapChecksTotalRegion;	//this variable is to store the total number of overlap checks for all objects

		PxgHandleRegion*		overlapChecksHandleRegiones;

		PxgIntegerRegion*		regionRange;

		PxU32*					startRegionAccum;
		PxU32*					blockStartRegionAccum;
		PxU32					startRegionAccumTotal;

		PxU32*					regionAccum;
		PxU32*					blockRegionAccum;
		PxU32					regionAccumTotal;

		PxU32*					endPtHistogram[2][3];
		PxU32*					blockEndPtHistogram[2][3];
		PxU32*					endPointHandles[2][3];

		PxU32*					totalEndPtHistogram[3];
		PxU32*					blockTotalEndPtHistogram[3];

		PxU32*					startPtHistogram[2][3];
		PxU32*					blockStartPtHistogram[2][3];
		PxU32*					startPointHandles[2][3];
	
		PxU32*					boxProjectionRanks[3];

		PxU32*					incrementalComparisons[3];
		PxU32*					incrementalBlockComparisons[3];

		PxU32*					aggReportBlock[2];
		PxU32*					actorReportBlock[2];

		PxU32					totalIncrementalComparisons[3];

		PxU32					max_found_lost_pairs;
		PxU32					max_found_lost_agg_pairs;

		bool 					found_lost_pairs_overflow_flags;
	};

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 createHandle(const PxU32 handle, const bool isStart, const bool isNew)
	{
		return (handle << 3) | (PxU32)isStart | (PxU32)isNew << 1;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 markDeleted(const PxU32 handle)
	{
		return handle | (1 << 2);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 getHandle(const PxU32 sortedHandle)
	{
		return sortedHandle >> 3;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool isStartProjection(const PxU32 sortedHandle)
	{
		return !!(sortedHandle & 1);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool isNewProjection(const PxU32 sortedHandle)
	{
		return !!((sortedHandle >> 1) & 1);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool isDeletedProjection(const PxU32 sortedHandle)
	{
		return !!((sortedHandle) & (1 << 2));
	}

}

#endif
