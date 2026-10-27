// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXV_SIM_STATS_H
#define PXV_SIM_STATS_H

#include "foundation/PxAssert.h"
#include "foundation/PxMemory.h"
#include "foundation/PxSimpleTypes.h"
#include "geometry/PxGeometry.h"

namespace physx
{
/*!
Description: contains statistics for the simulation.
*/
struct PxvSimStats
{
	PxvSimStats() { clearAll(); }
	void clearAll() { PxMemZero(this, sizeof(PxvSimStats)); }		// set counters to zero

	PX_FORCE_INLINE void incCCDPairs(PxGeometryType::Enum g0, PxGeometryType::Enum g1)
	{
		PX_ASSERT(g0 <= g1);  // That's how they should be sorted
		mNbCCDPairs[g0][g1]++;
	}

	PX_FORCE_INLINE void decCCDPairs(PxGeometryType::Enum g0, PxGeometryType::Enum g1)
	{
		PX_ASSERT(g0 <= g1);  // That's how they should be sorted
		PX_ASSERT(mNbCCDPairs[g0][g1]);
		mNbCCDPairs[g0][g1]--;
	}

	PX_FORCE_INLINE void incModifiedContactPairs(PxGeometryType::Enum g0, PxGeometryType::Enum g1)
	{
		PX_ASSERT(g0 <= g1);  // That's how they should be sorted
		mNbModifiedContactPairs[g0][g1]++;
	}

	PX_FORCE_INLINE void decModifiedContactPairs(PxGeometryType::Enum g0, PxGeometryType::Enum g1)
	{
		PX_ASSERT(g0 <= g1);  // That's how they should be sorted
		PX_ASSERT(mNbModifiedContactPairs[g0][g1]);
		mNbModifiedContactPairs[g0][g1]--;
	}

	// PT: those guys are now persistent and shouldn't be cleared each frame
	PxU32	mNbDiscreteContactPairs	[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];
	PxU32	mNbCCDPairs				[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];

	PxU32	mNbModifiedContactPairs	[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];

	PxU32	mNbDiscreteContactPairsTotal;		// PT: sum of mNbDiscreteContactPairs, i.e. number of pairs reaching narrow phase
	PxU32	mNbDiscreteContactPairsWithCacheHits;
	PxU32	mNbDiscreteContactPairsWithContacts;
	PxU32	mNbActiveConstraints;
	PxU32	mNbActiveDynamicBodies;
	PxU32	mNbActiveKinematicBodies;

	PxU32	mNbAxisSolverConstraints;
	PxU32	mTotalCompressedContactSize;
	PxU32	mTotalConstraintSize;
	PxU32	mPeakConstraintBlockAllocations;

	PxU32	mNbNewPairs;
	PxU32	mNbLostPairs;

	PxU32	mNbNewTouches;
	PxU32	mNbLostTouches;

	PxU32	mNbPartitions;

	PxU64 	mGpuDynamicsTempBufferCapacity;
	PxU32	mGpuDynamicsRigidContactCount;
	PxU32	mGpuDynamicsRigidPatchCount;
	PxU32	mGpuDynamicsFoundLostPairs;
	PxU32	mGpuDynamicsFoundLostAggregatePairs;
	PxU32	mGpuDynamicsTotalAggregatePairs;
	PxU32	mGpuDynamicsDeformableSurfaceContacts;
	PxU32	mGpuDynamicsDeformableVolumeContacts;
	PxU32	mGpuDynamicsParticleContacts; // not implemented
	PxU32	mGpuDynamicsCollisionStackSize;
};

}

#endif
