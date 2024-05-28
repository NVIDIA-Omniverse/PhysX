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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "BpAABBManagerBase.h"
#include "BpBroadPhase.h"

using namespace physx;
using namespace Bp;

AABBManagerBase::AABBManagerBase(	BroadPhase& bp, BoundsArray& boundsArray, PxFloatArrayPinned& contactDistance,
									PxU32 maxNbAggregates, PxU32 maxNbShapes, PxVirtualAllocator& allocator, PxU64 contextID,
									PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode) :
	mAddedHandleMap			(allocator),
	mRemovedHandleMap		(allocator),
	mChangedHandleMap		(allocator),
	mGroups					(allocator),
	mContactDistance		(contactDistance),
	mVolumeData				(allocator),
	mFilters				(kineKineFilteringMode == PxPairFilteringMode::eKILL, staticKineFilteringMode == PxPairFilteringMode::eKILL),
	mAddedHandles			(allocator),
	mUpdatedHandles			(allocator),
	mRemovedHandles			(allocator),
	mBroadPhase				(bp),
	mBoundsArray			(boundsArray),
	mUsedSize				(0),
	mNbAggregates			(0),
#if PX_ENABLE_SIM_STATS
	mGpuDynamicsLostFoundPairsStats(0),
	mGpuDynamicsTotalAggregatePairsStats(0),
	mGpuDynamicsLostFoundAggregatePairsStats(0),
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
#ifdef BP_USE_AGGREGATE_GROUP_TAIL
	mAggregateGroupTide		(PxU32(Bp::FilterGroup::eAGGREGATE_BASE)),
#endif
	mContextID				(contextID),
	mOriginShifted			(false)
{
	PX_UNUSED(maxNbAggregates);	// PT: TODO: use it or remove it
	reserveShapeSpace(PxMax(maxNbShapes, 1u));

	//	mCreatedOverlaps.reserve(16000);
}

void AABBManagerBase::reserveShapeSpace(PxU32 nbTotalBounds)
{
	nbTotalBounds = PxNextPowerOfTwo(nbTotalBounds);
	mGroups.resize(nbTotalBounds, Bp::FilterGroup::eINVALID);
	mVolumeData.resize(nbTotalBounds);					//KS - must be initialized so that userData is NULL for SQ-only shapes
	mContactDistance.resizeUninitialized(nbTotalBounds);
	mAddedHandleMap.resize(nbTotalBounds);
	mRemovedHandleMap.resize(nbTotalBounds);
}

void AABBManagerBase::reserveSpaceForBounds(BoundsIndex index)
{
	if ((index + 1) >= mVolumeData.size())
		reserveShapeSpace(index + 1);

	resetEntry(index); //KS - make sure this entry is flagged as invalid
}

void AABBManagerBase::freeBuffers()
{
	// PT: TODO: investigate if we need more stuff here
	mBroadPhase.freeBuffers();
}

void AABBManagerBase::shiftOrigin(const PxVec3& shift)
{
	mBroadPhase.shiftOrigin(shift, mBoundsArray.begin(), mContactDistance.begin());
	mOriginShifted = true;
}

