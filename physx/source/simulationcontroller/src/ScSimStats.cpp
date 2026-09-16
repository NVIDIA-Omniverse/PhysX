// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxMemory.h"
#include "ScSimStats.h"
#include "PxvSimStats.h"

using namespace physx;

Sc::SimStats::SimStats()
{
	numBroadPhaseAdds = numBroadPhaseRemoves = 0;

	gpuMemSizeParticles = 0;
	gpuMemSizeDeformableSurfaces = 0;
	gpuMemSizeDeformableVolumes = 0;

	clear();
}

void Sc::SimStats::clear()
{
#if PX_ENABLE_SIM_STATS
	PxMemZero(const_cast<void*>(reinterpret_cast<volatile void*>(&numTriggerPairs)), sizeof(TriggerPairCounts));
	numBroadPhaseAddsPending = numBroadPhaseRemovesPending = 0;
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
}

void Sc::SimStats::simStart()
{
#if PX_ENABLE_SIM_STATS
	// pending broadphase adds/removes are now the current ones
	numBroadPhaseAdds = numBroadPhaseAddsPending;
	numBroadPhaseRemoves = numBroadPhaseRemovesPending;
	clear();
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
}

void Sc::SimStats::readOut(PxSimulationStatistics& s, const PxvSimStats& simStats) const
{
#if PX_ENABLE_SIM_STATS
	s = PxSimulationStatistics();  // clear stats

	for(PxU32 i=0; i < PxGeometryType::eCONVEXMESH+1; i++)
	{
		for(PxU32 j=0; j < PxGeometryType::eGEOMETRY_COUNT; j++)
		{
			s.nbTriggerPairs[i][j] += PxU32(numTriggerPairs[i][j]);
			if (i != j)
				s.nbTriggerPairs[j][i] += PxU32(numTriggerPairs[i][j]);
		}
	}

	s.nbBroadPhaseAdds = numBroadPhaseAdds;
	s.nbBroadPhaseRemoves = numBroadPhaseRemoves;

	for(PxU32 i=0; i < PxGeometryType::eGEOMETRY_COUNT; i++)
	{
		s.nbDiscreteContactPairs[i][i] = simStats.mNbDiscreteContactPairs[i][i];
		s.nbModifiedContactPairs[i][i] = simStats.mNbModifiedContactPairs[i][i];
		s.nbCCDPairs[i][i] = simStats.mNbCCDPairs[i][i];

		for(PxU32 j=i+1; j < PxGeometryType::eGEOMETRY_COUNT; j++)
		{
			PxU32 c = simStats.mNbDiscreteContactPairs[i][j];
			s.nbDiscreteContactPairs[i][j] = c;
			s.nbDiscreteContactPairs[j][i] = c;

			c = simStats.mNbModifiedContactPairs[i][j];
			s.nbModifiedContactPairs[i][j] = c;
			s.nbModifiedContactPairs[j][i] = c;

			c = simStats.mNbCCDPairs[i][j];
			s.nbCCDPairs[i][j] = c;
			s.nbCCDPairs[j][i] = c;
		}
#if PX_DEBUG
		for(PxU32 j=0; j < i; j++)
		{
			// PxvSimStats should only use one half of the matrix
			PX_ASSERT(simStats.mNbDiscreteContactPairs[i][j] == 0);
			PX_ASSERT(simStats.mNbModifiedContactPairs[i][j] == 0);
			PX_ASSERT(simStats.mNbCCDPairs[i][j] == 0);
		}
#endif
	}

	s.nbDiscreteContactPairsTotal = simStats.mNbDiscreteContactPairsTotal;
	s.nbDiscreteContactPairsWithCacheHits = simStats.mNbDiscreteContactPairsWithCacheHits;
	s.nbDiscreteContactPairsWithContacts = simStats.mNbDiscreteContactPairsWithContacts;
	s.nbActiveConstraints = simStats.mNbActiveConstraints;
	s.nbActiveDynamicBodies = simStats.mNbActiveDynamicBodies;
	s.nbActiveKinematicBodies = simStats.mNbActiveKinematicBodies;

	s.nbAxisSolverConstraints = simStats.mNbAxisSolverConstraints;

	s.peakConstraintMemory = simStats.mPeakConstraintBlockAllocations * 16 * 1024;
	s.compressedContactSize = simStats.mTotalCompressedContactSize;
	s.requiredContactConstraintMemory = simStats.mTotalConstraintSize;
	s.nbNewPairs = simStats.mNbNewPairs;
	s.nbLostPairs = simStats.mNbLostPairs;
	s.nbNewTouches = simStats.mNbNewTouches;
	s.nbLostTouches = simStats.mNbLostTouches;
	s.nbPartitions = simStats.mNbPartitions;

	s.gpuDynamicsMemoryConfigStatistics.tempBufferCapacity = simStats.mGpuDynamicsTempBufferCapacity;
	s.gpuDynamicsMemoryConfigStatistics.rigidContactCount = simStats.mGpuDynamicsRigidContactCount;
	s.gpuDynamicsMemoryConfigStatistics.rigidPatchCount = simStats.mGpuDynamicsRigidPatchCount;
	s.gpuDynamicsMemoryConfigStatistics.foundLostPairs = simStats.mGpuDynamicsFoundLostPairs;
	s.gpuDynamicsMemoryConfigStatistics.foundLostAggregatePairs = simStats.mGpuDynamicsFoundLostAggregatePairs;
	s.gpuDynamicsMemoryConfigStatistics.totalAggregatePairs = simStats.mGpuDynamicsTotalAggregatePairs;
	s.gpuDynamicsMemoryConfigStatistics.deformableSurfaceContacts = simStats.mGpuDynamicsDeformableSurfaceContacts;
	s.gpuDynamicsMemoryConfigStatistics.deformableVolumeContacts = simStats.mGpuDynamicsDeformableVolumeContacts;
	s.gpuDynamicsMemoryConfigStatistics.softbodyContacts = simStats.mGpuDynamicsDeformableVolumeContacts; //deprecated
	s.gpuDynamicsMemoryConfigStatistics.particleContacts = simStats.mGpuDynamicsParticleContacts;
	s.gpuDynamicsMemoryConfigStatistics.collisionStackSize = simStats.mGpuDynamicsCollisionStackSize;

#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
	PX_UNUSED(s);
	PX_UNUSED(simStats);
#endif
}
