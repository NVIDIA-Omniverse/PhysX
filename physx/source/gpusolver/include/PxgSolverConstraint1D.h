// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SOLVER_CONSTRAINT_1D_H
#define PXG_SOLVER_CONSTRAINT_1D_H

#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "CmSpatialVector.h"
#include <vector_types.h>

namespace physx
{

struct PxgSolverConstraint1DHeader
{
	float4		body0WorldOffset_linBreakImpulse;
	//Strict ordering required - invInertiaScale0->invMassScale0->invInertiaScale1->invMassScale1. Do not change!
	PxReal		invInertiaScale0;
	PxReal		invMassScale0;
	PxReal		invInertiaScale1;
	PxReal		invMassScale1;
	

	PxU32		rowCounts;								// numbers of rows each 1D constraints 
	PxU32		breakable;											// indicate whether the constraint are breakable or not 
	PxReal		angBreakImpulse;
	PxU32		writeBackOffset;
};

PX_COMPILE_TIME_ASSERT(sizeof(PxgSolverConstraint1DHeader) == 48);

PX_ALIGN_PREFIX(16)
struct PxgSolverConstraint1DCon
{
	PxVec3 ang0;							//12	12
	PxVec3 lin0;							//12	24
	PxVec3 ang1;							//12	36
	PxVec3 lin1;							//12	48
	Cm::UnAlignedSpatialVector deltaVA;		//24	72
	Cm::UnAlignedSpatialVector deltaVB;		//24	96
	PxReal minImpulse;						//4		100
	PxReal maxImpulse;						//4		104
	PxReal velMultiplier;					//4		108
	PxReal impulseMultiplier;				//4		112

} PX_ALIGN_SUFFIX(16);

PX_COMPILE_TIME_ASSERT(sizeof(PxgSolverConstraint1DCon) == 112);

struct PxgSolverConstraint1DMod
{
	PxVec3	ang0Writeback;					//!< unscaled angular velocity projection (body 0)
	PxReal	constant;						//!< constant	 
	PxReal	unbiasedConstant;				//!< unbiased constant
	PxReal	appliedForce;					//!< applied force to correct velocity+bias
	PxU32	flags;							
};



struct PxgTGSSolverConstraint1DHeader
{
	PxU16		rowCounts;								// numbers of rows each 1D constraints 
	PxU16		breakable;								// indicate whether the constraint are breakable or not 
	PxReal		linBreakImpulse;
	PxReal		angBreakImpulse;
	PxU32		writeBackOffset;

	PxVec4		raWorld;
	PxVec4		rbWorld;

	//Strict ordering required - invInertiaScale0->invMassScale0->invInertiaScale1->invMassScale1. Do not change!
	PxReal		invInertiaScale0;
	PxReal		invMassScale0;
	PxReal		invInertiaScale1;
	PxReal		invMassScale1;
	

	//There is no orthogonalization with articulation constraints, so we do not need to
	//add anything to reflect that in this code!
};

PX_ALIGN_PREFIX(16)
struct PxgTGSSolverConstraint1DCon
{
	PxVec3 ang0;							//12	12
	PxVec3 lin0;							//12	24
	PxVec3 ang1;							//12	36
	PxVec3 lin1;							//12	48
	Cm::UnAlignedSpatialVector deltaVA;		//24	72
	Cm::UnAlignedSpatialVector deltaVB;		//24	96

	PxReal minImpulse;						//4		100
	PxReal maxImpulse;						//4		104
	PxReal velMultiplier;					//4		108
	PxReal impulseMultiplier;				//4		112

	PxReal error;							//4		116
	PxReal velTarget;						//4		120
	PxReal recipResponse;					//4		124
	PxReal angularErrorScale;				//4		128

} PX_ALIGN_SUFFIX(16);

PX_COMPILE_TIME_ASSERT(sizeof(PxgTGSSolverConstraint1DCon) == 128);

struct PxgTGSSolverConstraint1DMod
{
	PxReal	appliedForce;					//!< applied force to correct velocity+bias
	PxReal	maxBias;
	PxReal	biasScale;						
	PxU32	flags;
};

struct PxgJointParams
{
	PxgSolverConstraint1DHeader* jointHeader;
	PxgSolverConstraint1DCon* jointCon;
	PxgSolverConstraint1DMod* jointMod;
};

struct PxgTGSJointParams
{
	PxgTGSSolverConstraint1DHeader* jointHeader;
	PxgTGSSolverConstraint1DCon* jointCon;
	PxgTGSSolverConstraint1DMod* jointMod;
};


}

#endif