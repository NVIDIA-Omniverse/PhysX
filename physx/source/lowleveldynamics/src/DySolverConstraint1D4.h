// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SOLVER_CONSTRAINT_1D4_H
#define DY_SOLVER_CONSTRAINT_1D4_H

#include "foundation/PxVec3.h"
#include "PxPhysXConfig.h"
#include "DySolverConstraint1D.h"

namespace physx
{

namespace Dy
{

struct SolverConstraint1DHeader4
{
	PxU8	type;			// enum SolverConstraintType - must be first byte
	PxU8	pad0[3];	
	//These counts are the max of the 4 sets of data.
	//When certain pairs have fewer constraints than others, they are padded with 0s so that no work is performed but 
	//calculations are still shared (afterall, they're computationally free because we're doing 4 things at a time in SIMD)
	PxU32	count;
	PxU8	count0, count1, count2, count3;
	PxU8	break0, break1, break2, break3;

	aos::Vec4V	linBreakImpulse;
	aos::Vec4V	angBreakImpulse;
	aos::Vec4V	invMass0D0;
	aos::Vec4V	invMass1D1;
	aos::Vec4V	angD0;
	aos::Vec4V	angD1;

	aos::Vec4V	body0WorkOffsetX;
	aos::Vec4V	body0WorkOffsetY;
	aos::Vec4V	body0WorkOffsetZ;
};

struct SolverConstraint1DBase4 
{
public:
	aos::Vec4V		lin0X;
	aos::Vec4V		lin0Y;
	aos::Vec4V		lin0Z;
	aos::Vec4V		ang0X;
	aos::Vec4V		ang0Y;
	aos::Vec4V		ang0Z;
	aos::Vec4V		ang0WritebackX;
	aos::Vec4V		ang0WritebackY;
	aos::Vec4V		ang0WritebackZ;
	aos::Vec4V		constant;
	aos::Vec4V		unbiasedConstant;
	aos::Vec4V		velMultiplier;
	aos::Vec4V		impulseMultiplier;
	aos::Vec4V		minImpulse;
	aos::Vec4V		maxImpulse;
	aos::Vec4V		appliedForce;
	PxU32		flags[4];
};

PX_COMPILE_TIME_ASSERT(sizeof(SolverConstraint1DBase4) == 272);

struct SolverConstraint1DDynamic4 : public SolverConstraint1DBase4
{
	aos::Vec4V		lin1X;
	aos::Vec4V		lin1Y;
	aos::Vec4V		lin1Z;
	aos::Vec4V		ang1X;
	aos::Vec4V		ang1Y;
	aos::Vec4V		ang1Z;
};
PX_COMPILE_TIME_ASSERT(sizeof(SolverConstraint1DDynamic4) == 368);

}

}

#endif
