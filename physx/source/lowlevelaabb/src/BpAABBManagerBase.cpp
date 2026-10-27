// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BpAABBManagerBase.h"
#include "BpBroadPhase.h"

using namespace physx;
using namespace Bp;
using namespace Cm;

AABBManagerBase::AABBManagerBase(	BroadPhase& bp, BoundsArray& boundsArray, PinnableArray<PxReal>& contactDistance,
									PxU32 maxNbAggregates, PxU32 maxNbShapes, VirtualAllocatorCallback& allocator, PxU64 contextID,
									PxPairFilteringMode::Enum kineKineFilteringMode, PxPairFilteringMode::Enum staticKineFilteringMode) :
	mAddedHandleMap			(allocator),
	mRemovedHandleMap		(allocator),
	mChangedHandleMap		(allocator),
	mGroups					(allocator),
	mEnvIDs					(allocator),
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
#if BP_USE_AGGREGATE_GROUP_TAIL
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

