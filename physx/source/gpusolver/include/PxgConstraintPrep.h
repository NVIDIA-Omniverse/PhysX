// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_CONSTRAINT_PREP_H
#define PXG_CONSTRAINT_PREP_H

#include "PxConstraintDesc.h"
#include "PxgSolverConstraintDesc.h"
#include "PxgD6JointData.h"
#include "AlignedTransform.h"

#include "PxcNpWorkUnit.h"

namespace physx
{
	typedef PxcNpWorkUnitFlag	PxgNpWorkUnitFlag;

	struct PxgSolverBody;
	struct PxgSolverBodyData;
	struct PxgSolverConstraintDesc;

	//This structure can update everyframe
	struct PxgConstraintPrePrep
	{
	public:
		PxNodeIndex mNodeIndexA;	//8		8
		PxNodeIndex mNodeIndexB;	//16	8
		PxU32 mFlags;				//20	4
		float mLinBreakForce;		//24	4
		float mAngBreakForce;		//28	4
	};

	struct PxgConstraintData
	{
		PxgConstraintInvMassScale mInvMassScale;	//16
		float4 mRaWorld_linBreakForceW;				//16
		float4 mRbWorld_angBreakForceW;				//16
		uint4 mNumRows_Flags_StartIndex;			//16
	};

	struct PxgBlockConstraint1DData
	{
		PX_ALIGN(256, PxgConstraintInvMassScale mInvMassScale[32]);		//512			512
		PX_ALIGN(256, float4 mRAWorld_linBreakForce[32]);				//1024			512
		PX_ALIGN(256, float4 mRBWorld_AngBreakForce[32]);				//1152			128
		PX_ALIGN(128, PxU32 mNumRows[32]);								//1284			128
		PX_ALIGN(128, PxU32 mFlags[32]);								//1412			128
	};

	struct PxgConstraint1DData
	{
		PxgConstraintInvMassScale	mInvMassScale;						//16	16						
		float4						mBody0WorldOffset_linBreakForce;	//16	32					
		float						mAngBreakForce;						//4		36				
		PxU32						mNumRows;							//4		40					
		PxU32						mFlags;								//4		44
		PxU32						mPadding;							//4		48
	};

	struct /*__device_builtin__*/ __builtin_align__(16) PxgMaterialContactData
	{
		PxReal restDistance;				//4		4
		PxReal staticFriction;				//8		4
		PxReal dynamicFriction;				//12	4
		PxU8 mNumContacts;					//13	1
		PxU8 mSolverFlags;					//14	1
		PxU8 prevFrictionPatchCount;		//15	1
		PxU8 pad;							//16	1
	};

	struct PxgBlockContactData
	{
		PX_ALIGN(128,	PxgConstraintInvMassScale	mInvMassScale[32]);				//512	512		
		PX_ALIGN(128,	float4						normal_restitutionW[32]);		//1024	512
		PX_ALIGN(128,	PxgMaterialContactData		contactData[32]);				//1536	512
		PX_ALIGN(128,	PxReal						damping[32]);					//1664	128
	};

	//This is for articulation contact
	struct PxgContactData
	{
		PxgConstraintInvMassScale mInvMassScale;
		float4 normal_restitutionW;
		PxgMaterialContactData contactData;
	};

	struct PxgBlockContactPoint
	{
		PX_ALIGN(256,	float4	point_separationW[32]);
		PX_ALIGN(256,	float4	targetVel_maxImpulseW[32]);
	};

	struct PxgContactPoint
	{
		float4	point_separationW;
		float4	targetVel_maxImpulseW;
	};
}

#endif