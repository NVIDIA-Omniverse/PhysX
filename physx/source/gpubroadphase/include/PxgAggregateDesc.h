// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_AGGREGATE_DESC_H
#define PXG_AGGREGATE_DESC_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
	struct PxgBroadPhasePair;
	struct PxgAggregate;
	struct PxgAggregatePair;


	struct PxgFreeBufferList
	{
		PxU32 numFreeIndices;
		PxU32 maxIndex;
	};

	struct PxgAggregateDesc
	{
		public:
			PxgBroadPhasePair*	foundPairReport;
			PxgBroadPhasePair*	lostPairReport;

			PxgBroadPhasePair*	foundPairReportMap;
			PxgBroadPhasePair*	lostPairReportMap;
			PxU32 				sharedFoundPairIndex;
			PxU32 				sharedLostPairIndex;

			PxgAggregate*		aggregates;
			PxU32				numAgregates;

			PxgFreeBufferList*	freeBufferList;
			PxU32*				freeIndices;
			PxU32*				removeBitmap;
			PxU32*				removeHistogram;
			PxU32				nbRemoved;

			PxgAggregatePair*	aggPairs;
			PxU32*				aggPairCount;

			PxU32				max_found_lost_pairs;
			PxU32				max_agg_pairs;

			PxU32				aggPairOverflowCount;
			PxU32				foundCandidatePairOverflowCount;

			bool				found_pairs_overflow_flags;
			bool				lost_pairs_overflow_flags;
			bool				agg_pairs_overflow_flags;
			
	};
}
#endif
