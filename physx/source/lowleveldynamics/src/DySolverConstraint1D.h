// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SOLVER_CONSTRAINT_1D_H
#define DY_SOLVER_CONSTRAINT_1D_H

#include "foundation/PxVec3.h"
#include "PxPhysXConfig.h"
#include "DySolverConstraintTypes.h"
#include "DySolverBody.h"
#include "PxConstraintDesc.h"
#include "DySolverConstraintDesc.h"
#include "DyCpuGpu1dConstraint.h"

namespace physx
{

namespace Dy
{

// dsequeira: we should probably fork these structures for constraints and extended constraints,
// since there's a few things that are used for one but not the other

struct SolverConstraint1DHeader
{
	PxU8	type;			// enum SolverConstraintType - must be first byte
	PxU8	count;			// count of following 1D constraints
	PxU8	dominance;
	PxU8	breakable;		// indicate whether this constraint is breakable or not						

	PxReal	linBreakImpulse;
	PxReal	angBreakImpulse;
	PxReal	invMass0D0;
	PxVec3	body0WorldOffset;
	PxReal	invMass1D1;
	PxReal	linearInvMassScale0;		// only used by articulations
	PxReal	angularInvMassScale0;		// only used by articulations
	PxReal	linearInvMassScale1;		// only used by articulations
	PxReal	angularInvMassScale1;		// only used by articulations
};

PX_COMPILE_TIME_ASSERT(sizeof(SolverConstraint1DHeader) == 48);

PX_ALIGN_PREFIX(16)
struct SolverConstraint1D 
{
public:
	PxVec3		lin0;					//!< linear velocity projection (body 0)	
	PxReal		constant;				//!< constraint constant term

	PxVec3		lin1;					//!< linear velocity projection (body 1)
	PxReal		unbiasedConstant;		//!< constraint constant term without bias

	PxVec3		ang0;					//!< angular velocity projection (body 0)
	PxReal		velMultiplier;			//!< constraint velocity multiplier

	PxVec3		ang1;					//!< angular velocity projection (body 1)
	PxReal		impulseMultiplier;		//!< constraint impulse multiplier

	PxVec3		ang0Writeback;			//!< unscaled angular velocity projection (body 0)
	PxU32		pad;

	PxReal		minImpulse;				//!< Lower bound on impulse magnitude	 
	PxReal		maxImpulse;				//!< Upper bound on impulse magnitude
	PxReal		appliedForce;			//!< applied force to correct velocity+bias
	PxU32		flags;

	void setSolverConstants(const Constraint1dSolverConstantsPGS& solverConstants)
	{
		constant = solverConstants.constant;
		unbiasedConstant = solverConstants.unbiasedConstant;
		velMultiplier = solverConstants.velMultiplier;
		impulseMultiplier = solverConstants.impulseMultiplier;
	}
} PX_ALIGN_SUFFIX(16); 	

PX_COMPILE_TIME_ASSERT(sizeof(SolverConstraint1D) == 96);


struct SolverConstraint1DExt : public SolverConstraint1D
{
public:
	Cm::SpatialVector deltaVA_;
	Cm::SpatialVector deltaVB_;
};

PX_COMPILE_TIME_ASSERT(sizeof(SolverConstraint1DExt) == 160);


PX_FORCE_INLINE void init(SolverConstraint1DHeader& h, 
						  PxU8 count, 
						  bool isExtended,
						  const PxConstraintInvMassScale& ims)
{
	h.type			= PxU8(isExtended ? DY_SC_TYPE_EXT_1D : DY_SC_TYPE_RB_1D);
	h.count			= count;
	h.dominance		= 0;
	h.linearInvMassScale0	= ims.linear0;
	h.angularInvMassScale0	= ims.angular0;
	h.linearInvMassScale1	= -ims.linear1;
	h.angularInvMassScale1	= -ims.angular1;
}

PX_FORCE_INLINE void init(SolverConstraint1D& c,
						  const PxVec3& _linear0, const PxVec3& _linear1, 
						  const PxVec3& _angular0, const PxVec3& _angular1,
						  PxReal _minImpulse, PxReal _maxImpulse)
{
	PX_ASSERT(_linear0.isFinite());
	PX_ASSERT(_linear1.isFinite());
	c.lin0					= _linear0;
	c.lin1					= _linear1;
	c.ang0					= _angular0;
	c.ang1					= _angular1;
	c.minImpulse			= _minImpulse;
	c.maxImpulse			= _maxImpulse;
	c.flags					= 0;
	c.appliedForce			= 0;
}

PX_FORCE_INLINE bool needsNormalVel(const Px1DConstraint &c)
{
	return c.flags & Px1DConstraintFlag::eRESTITUTION
		|| (c.flags & Px1DConstraintFlag::eSPRING && c.flags & Px1DConstraintFlag::eACCELERATION_SPRING);
}

}
}

#endif
