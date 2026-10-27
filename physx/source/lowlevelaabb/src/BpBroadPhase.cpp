// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "BpBroadPhase.h"
#include "BpBroadPhaseSap.h"
#include "BpBroadPhaseMBP.h"
#include "BpBroadPhaseABP.h"

using namespace physx;
using namespace Bp;

BroadPhase* BroadPhase::create(
	const PxBroadPhaseType::Enum bpType,
	const PxU32 maxNbRegions,
	const PxU32 maxNbBroadPhaseOverlaps,
	const PxU32 maxNbStaticShapes,
	const PxU32 maxNbDynamicShapes,
	PxU64 contextID)
{
	if(bpType==PxBroadPhaseType::eABP)
		return PX_NEW(BroadPhaseABP)(maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID, false);
	else if(bpType==PxBroadPhaseType::ePABP)
		return PX_NEW(BroadPhaseABP)(maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID, true);
	else if(bpType==PxBroadPhaseType::eMBP)
		return PX_NEW(BroadPhaseMBP)(maxNbRegions, maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID);
	else if(bpType==PxBroadPhaseType::eSAP)
		return PX_NEW(BroadPhaseSap)(maxNbBroadPhaseOverlaps, maxNbStaticShapes, maxNbDynamicShapes, contextID);
	else
	{
		PX_ASSERT(0);
		return NULL;
	}
}
