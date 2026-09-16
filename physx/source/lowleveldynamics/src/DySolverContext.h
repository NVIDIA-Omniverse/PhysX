// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SOLVER_CONTEXT_H
#define DY_SOLVER_CONTEXT_H

namespace physx
{
	struct PxSolverBodyData;

namespace Dy
{
	struct ThresholdStreamElement;
	
struct SolverContext
{
	bool					doFriction;
	bool					writeBackIteration;

	// for threshold stream output
	ThresholdStreamElement*	mThresholdStream;
	PxU32					mThresholdStreamIndex;
	PxU32					mThresholdStreamLength;
	PxSolverBodyData*		solverBodyArray;

	ThresholdStreamElement*	mSharedThresholdStream;
	PxU32					mSharedThresholdStreamLength;
	PxI32*					mSharedOutThresholdPairs;
	Cm::SpatialVectorF*		deltaV; // used temporarily in PxcFsFlushVelocities
};

}

}

#endif
