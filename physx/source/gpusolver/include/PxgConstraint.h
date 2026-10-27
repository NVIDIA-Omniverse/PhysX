// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_CONSTRAINT_H
#define PXG_CONSTRAINT_H

#include "PxPhysXConfig.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "vector_types.h"

namespace physx
{

struct PxgSolverBodyData;
struct PxConstraintInvMassScale;

struct PxgSolverContactHeader
{
	float4	invMass0_1_angDom0_1;
	float4	normal_staticFriction;

	PxU32	flags;
	PxU32	numNormalConstr;
	PxU32	forceWritebackOffset;
	PxReal	accumNormalForce;
}; 

PX_COMPILE_TIME_ASSERT(sizeof(PxgSolverContactHeader) == 48);

/**
\brief A single articulation contact point for the solver.
*/
struct PxgSolverContactPointExt
{
	PxVec3	angDeltaVA;						//12	12
	PxVec3  linDeltaVA;						//12	24
	PxVec3	angDeltaVB;						//12	36
	PxVec3	linDeltaVB;						//12	48
	PxVec3	raXn;							//12	60
	PxVec3	rbXn;							//12	72
	PxReal	velMultiplier;					//4		76
	PxReal	maxImpulse;						//4		80
	PxReal	biasedErr;						//4		84
	PxReal	unbiasedErr;					//4		88
	PxReal	appliedForce;					//4		92
	PxU32	padding;						//4		96
}; 


PX_COMPILE_TIME_ASSERT(sizeof(PxgSolverContactPointExt) == 96);

struct PxgSolverFrictionHeader
{
	float4	frictionNormals[2];		
	PxU32	numFrictionConstr;			
	PxReal	dynamicFriction;			
	PxU32	broken;					
};

/**
\brief A single articulation friction constraint for the solver.
*/
#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
struct PX_ALIGN_PREFIX(16) PxgSolverContactFrictionExt
{
	PxVec3	angDeltaVA;						//12	12
	PxVec3  linDeltaVA;						//12	24
	PxVec3	angDeltaVB;						//12	36
	PxVec3	linDeltaVB;						//12	48
	PxVec3	raXn;							//12	60
	PxVec3	rbXn;							//12	72
	PxReal	velMultiplier;					//4		76
	PxReal	targetVel;						//4		80
	PxReal	bias;							//4		84
	PxReal	appliedForce;					//4		88
	PxU32	padding[2];						//8		96

} PX_ALIGN_SUFFIX(16); 
#if PX_VC
#pragma warning(pop)
#endif

struct PxgContactParams
{
	PxgSolverContactHeader* contactHeader;
	PxgSolverFrictionHeader* frictionHeader;
	PxgSolverContactPointExt* solverContacts;
	PxgSolverContactFrictionExt* solverFrictions;
};

PX_COMPILE_TIME_ASSERT(sizeof(PxgSolverContactFrictionExt) == 96);


struct PxgTGSSolverContactHeader
{
	float4	dom0_1_angDom0_1;			//16
	float4	normal_maxPenBias;			//32
	
	PxReal	staticFriction;
	PxReal	dynamicFriction;
	PxReal	minNormalForce;
	PxU32	flags;						//48

	PxU16	numNormalConstr;
	PxU16	numFrictionConstr;
	PxU32	forceWritebackOffset;
	PxU32	broken;						
	PxU32	pad;						//64

};

PX_COMPILE_TIME_ASSERT(sizeof(PxgTGSSolverContactHeader) == 64);

struct PxgTGSSolverContactPointExt
{
	//Grouped together in contiguous memory so we can load all 48 bytes in a single instruction
	PxVec3	angDeltaVA;					//12	12
	PxVec3  linDeltaVA;					//12	24
	PxVec3	angDeltaVB;					//12	36
	PxVec3	linDeltaVB;					//12	48
	
	//Grouped so we can load 24 bytes in single instruction
	PxVec3	raXn;						//12	60
	PxVec3	rbXn;						//12	72

	//All the loose items - loaded incoherently
	PxReal	separation;					//4		76
	PxReal	velMultiplier;				//4		80
	PxReal	targetVelocity;				//4		84
	PxReal	biasCoefficient;			//4		88
	PxReal	maxImpulse;					//4		92
	PxReal	appliedForce;				//4		96
};

struct PxgTGSSolverFrictionExt
{
	//Grouped together in contiguous memory so we can load all 48 bytes in a single instruction
	PxVec3	angDeltaVA;					//12	12
	PxVec3  linDeltaVA;					//12	24
	PxVec3	angDeltaVB;					//12	36
	PxVec3	linDeltaVB;					//12	48

	//Grouped so we can load 24 bytes in single instruction
	PxVec3	raXn;						//12	60
	PxVec3	rbXn;						//12	72

	//Loose items - loaded incoherently
	PxVec3	normal;						//12	84
	PxReal	error;						//4		88
	PxReal	targetVel;					//4		92
	PxReal	velMultiplier;				//4		96

	PxReal	biasScale;					//4		100
	PxReal	frictionScale;				//4		104
	PxReal	appliedForce;				//4		108
	PxU32	pad;						//4		112
};

struct PxgTGSContactParams
{
	PxgTGSSolverContactHeader* contactHeader;
	PxgTGSSolverContactPointExt* solverContacts;
	PxgTGSSolverFrictionExt* solverFrictions;
};


}

#endif

