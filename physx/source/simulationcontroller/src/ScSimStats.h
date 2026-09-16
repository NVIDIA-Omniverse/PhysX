// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_SIM_STATS_H
#define SC_SIM_STATS_H

#include "geometry/PxGeometry.h"
#include "PxSimulationStatistics.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{

struct PxvSimStats;

namespace Sc
{

	/*
	Description: contains statistics for the scene.
	*/
	class SimStats : public PxUserAllocated
	{
	public:
		SimStats();

		void clear();		//set counters to zero
		void simStart();
		void readOut(PxSimulationStatistics& dest, const PxvSimStats& simStats) const;

		PX_INLINE void incBroadphaseAdds()
		{
			numBroadPhaseAddsPending++;
		}

		PX_INLINE void incBroadphaseRemoves()
		{
			numBroadPhaseRemovesPending++;
		}

	private:
		// Broadphase adds/removes for the current simulation step
		PxU32 numBroadPhaseAdds;
		PxU32 numBroadPhaseRemoves;

		// Broadphase adds/removes for the next simulation step
		PxU32 numBroadPhaseAddsPending;
		PxU32 numBroadPhaseRemovesPending;

	public:
		typedef PxI32 TriggerPairCountsNonVolatile[PxGeometryType::eCONVEXMESH+1][PxGeometryType::eGEOMETRY_COUNT];
		typedef volatile TriggerPairCountsNonVolatile TriggerPairCounts;
		TriggerPairCounts numTriggerPairs;

		PxU64 gpuMemSizeParticles;
		PxU64 gpuMemSizeDeformableSurfaces;
		PxU64 gpuMemSizeDeformableVolumes;
	};

} // namespace Sc

}

#endif
