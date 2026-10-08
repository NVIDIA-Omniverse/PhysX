// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_BROADPHASE_PAIR_REPORT_H
#define PXG_BROADPHASE_PAIR_REPORT_H

#include "PxgBroadPhaseCommonDefines.h"
#include "foundation/PxMath.h"


namespace physx
{

/*
\brief Structure used to report created and deleted broadphase pairs
\note The indices mVolA and mVolB correspond to the bounds indices 
BroadPhaseUpdateData::mCreated used by BroadPhase::update
\see BroadPhase::getCreatedPairs, BroadPhase::getDeletedPairs
*/
struct PxgBroadPhasePair
{
	PX_CUDA_CALLABLE PxgBroadPhasePair(PxU32 volA, PxU32 volB)
	{
		mVolA=PxMin(volA,volB);
		mVolB=PxMax(volA,volB);
	}

	PX_CUDA_CALLABLE PxgBroadPhasePair()
		: mVolA(PXG_INVALID_BP_HANDLE),
		  mVolB(PXG_INVALID_BP_HANDLE)
	{
	}

	PxU32			mVolA;		// NB: mVolA < mVolB
	PxU32			mVolB;
};

}

#endif