// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_ARTICULATION_LINK_H
#define PXG_ARTICULATION_LINK_H

#include "foundation/PxSimpleTypes.h"
#include "PxgSolverBody.h"
#include "DyFeatherstoneArticulationUtils.h"


namespace physx
{
	typedef PxU64 ArticulationBitField;

	struct PxgArticulationLinkData
	{
	public:
		Cm::UnAlignedSpatialVector		IsW[3];//stI is the transpose of Is
		Cm::UnAlignedSpatialVector		IsInvDW[3];
		//PxVec3							r;  //vector from parent com to child com
		//PxVec3							rw;	//world space r
		PxReal							qstZIc[3];//jointForce - stZIc
		PxReal							invStIs[3][3];
	};

	struct PxgArticulationLink 
	{
#if !PX_CUDA_COMPILER
		PX_ALIGN(16, PxVec3			initialAngVel);					//	12	12 initial ang vel
		PxReal						penBiasClamp;					//	4	16 the penetration bias clamp
		PxVec3						initialLinVel;					//	12	28 initial lin vel
		PxReal						invMass;						//	4	32 inverse mass
		
#else
		float4						initialAngVelXYZ_penBiasClamp;
		float4						initialLinVelXYZ_invMassW;
#endif

		PxReal						maxAngularVelocitySq;			//	4	36
		PxReal						maxLinearVelocitySq;			//	4	40
		PxReal						linearDamping;					//	4	44
		PxReal						angularDamping;					//	4	48

		PxU32						pathToRootOffset;				//	4	52
		PxU32						childrenOffset;					//	4	56
		PxU16						numPathToRoot;					//	2	58
		PxU16						numChildren;					//	2	60
		PxReal						offsetSlop;						//	4	64

		ArticulationBitField		pathToRoot;						//	8	72 path to root, including link and root
		PxReal						cfmScale;						//	4	76
		bool						disableGravity;					//	1	77
		bool						retainsAccelerations;			//	1	78
		bool						padding[2];						//	1	80
	};


	struct PxgArticulationLinkSleepData
	{
#if !PX_CUDA_COMPILER
		PX_ALIGN(16, PxVec3			sleepLinVelAcc);					//12	12
		PxReal						padding0;							//4		16
		PX_ALIGN(16, PxVec3			sleepAngVelAcc);					//12	28
		PxReal						padding1;							//4		32
#else
		float4						sleepLinVelAccXYZ;
		float4						sleepAngVelAccXYZ;
#endif
	};

	struct PxgArticulationLinkProp
	{
#if !PX_CUDA_COMPILER
		PX_ALIGN(16, PxVec3			invInertia);						//12	12
		PxReal						invMass;							//4		16
#else
		float4						invInertiaXYZ_invMass;
#endif
	};

}

#endif