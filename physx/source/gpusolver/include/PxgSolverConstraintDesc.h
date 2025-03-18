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

#ifndef PXG_SOLVER_CONSTRAINT_DESC_H
#define PXG_SOLVER_CONSTRAINT_DESC_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxPreprocessor.h"
#include <vector_types.h>
#include "PxNodeIndex.h"


namespace physx
{
struct PxgSolverBody;

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
struct PxgBlockConstraint1DVelocities
{
	PX_ALIGN(256, float4	linear0XYZ_geometricErrorW[32]);			//!< linear component of velocity jacobian in world space, geometric error of the constraint along this axi
	PX_ALIGN(256, float4	angular0XYZ_velocityTargetW[32]);			//!< angular component of velocity jacobian in world space, velocity target for the constraint along this axis

	PX_ALIGN(256, float4	linear1XYZ_minImpulseW[32]);				//!< linear component of velocity jacobian in world space, minimum impulse the solver may apply to enforce this constraint
	PX_ALIGN(256, float4	angular1XYZ_maxImpulseW[32]);				//!< angular component of velocity jacobian in world space, maximum impulse the solver may apply to enforce this constraint
};

struct PxgBlockConstraint1DParameters
{
	
	union
	{
		struct SpringModifiers
		{
			PX_ALIGN(128, PxReal	stiffness[32]);				//!< spring parameter, for spring constraints
			PX_ALIGN(128, PxReal	damping[32]);				//!< damping parameter, for spring constraints
		} spring;
		struct RestitutionModifiers
		{
			PX_ALIGN(128, PxReal	restitution[32]);			//!< restitution parameter for determining additional "bounce"
			PX_ALIGN(128, PxReal	velocityThreshold[32]);		//!< minimum impact velocity for bounce
		} bounce;
	} mods;

	PX_ALIGN(128, PxU32		flags[32]);					//!< a set of Px1DConstraintFlags
	PX_ALIGN(128, PxU32		solveHint[32]);				//!< constraint optimization hint, should be an element of PxConstraintSolveHint
};


struct PxgConstraint1DVelocities
{
	float4	linear0XYZ_geometricErrorW;			//!< linear component of velocity jacobian in world space, geometric error of the constraint along this axi
	float4	angular0XYZ_velocityTargetW;			//!< angular component of velocity jacobian in world space, velocity target for the constraint along this axis

	float4	linear1XYZ_minImpulseW;				//!< linear component of velocity jacobian in world space, minimum impulse the solver may apply to enforce this constraint
	float4	angular1XYZ_maxImpulseW;				//!< angular component of velocity jacobian in world space, maximum impulse the solver may apply to enforce this constraint
};

struct PxgConstraint1DParameters
{

	union
	{
		struct SpringModifiers
		{
			PxReal	stiffness;				//!< spring parameter, for spring constraints
			PxReal	damping;				//!< damping parameter, for spring constraints
		} spring;
		struct RestitutionModifiers
		{
			PxReal	restitution;			//!< restitution parameter for determining additional "bounce"
			PxReal	velocityThreshold;		//!< minimum impact velocity for bounce
		} bounce;
	} mods;

	PxU16		flags;					//!< a set of Px1DConstraintFlags
	PxU16		solveHint;				//!< constraint optimization hint, should be an element of PxConstraintSolveHint

	PxU32		pad;
};

struct PxgSolverConstraintDesc
{
	enum PxgConstraintType
	{
		eCONSTRAINT_1D,
		eCONTACT,
		eARTICULATION_CONSTRAINT_1D,
		eARTICULATION_CONTACT,
	};												
	PxU8*					constraint;				//8
	PxU32					bodyAIndex;
	PxU32					bodyBIndex;
	PxU16					constraintType;
	PxU16					patchIndex;
	//PxU16					pad;
};

#if !PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(sizeof(PxgSolverConstraintDesc) == 16);
#endif

PX_ALIGN_PREFIX(16)
struct PxgConstraintInvMassScale
{

#if !PX_CUDA_COMPILER
	PxReal linear0;		//!< multiplier for inverse mass of body0
	PxReal angular0;	//!< multiplier for inverse MoI of body0
	PxReal linear1;		//!< multiplier for inverse mass of body1
	PxReal angular1;	//!< multiplier for inverse MoI of body1
#else
	float4 lin0X_ang0Y_lin1Z_ang1W;
#endif
}PX_ALIGN_SUFFIX(16);


struct PxgBlockContactData;
struct PxgBlockContactPoint;

namespace Sc
{
	class ShapeInteraction;
}

struct PxgConstraintBatchHeader
{
	PxU16										mDescStride;				//number between 1 to 32
	PxU16										constraintType;				//constraint type (joint or contact)
	PxU32										mConstraintBatchIndex;		//constraint batch index (the index for the specific joint/contact batch)
	PxU32										mStartPartitionIndex;		//start partition index (the start index for the set of partition edges representing this batch)
	PxU32										mask;						//Only used by the articulation internal constraint solver
};


struct PxgBlockConstraintBatch
{
	PxU16										mDescStride; //number between 1 to 32
	PxU16										constraintType;
	PxU32										blockContactIndex;
	PxU32										mConstraintBatchIndex;
	PxU32										startConstraintIndex;
	PxU32										startFrictionIndex;  
	PxU32										mStartPartitionIndex;
	PxU32										mArticulationResponseIndex; //Only required for articulation constraints!
	PxU32										mask;

	PX_ALIGN(128, PxNodeIndex bodyANodeIndex[32]);
	PX_ALIGN(128, PxNodeIndex bodyBNodeIndex[32]);

	PX_ALIGN(128, PxU32	bodyAIndex[32]);
	PX_ALIGN(128, PxU32	bodyBIndex[32]);

	PX_ALIGN(128, PxU32	remappedBodyAIndex[32]);   
	PX_ALIGN(128, PxU32	remappedBodyBIndex[32]);

	PX_ALIGN(128, PxU32 slabId[32]);
	
	PX_ALIGN(128, Sc::ShapeInteraction* shapeInteraction[32]); // used for force-threshold reporting
};

struct PxgBlockWorkUnit
{
	PX_ALIGN(128, PxU32 mWriteback[32]);
	
	PX_ALIGN(128, float	mRestDistance[32]);

	PX_ALIGN(128,	PxU32	mEdgeIndex[32]);
	PX_ALIGN(128,	PxU32	mFlags[32]);
	PX_ALIGN(128,	PxU32	mPatchIndex[32]);
	PX_ALIGN(128,	PxU32	mFrictionPatchIndex[32]);
	PX_ALIGN(128,	float2	mTorsionalFrictionData[32]);
};

#if PX_VC
#pragma warning(pop)
#endif

//This used in contact preprep(constraintContactBlockPrePrepLaunch) and joint prep code(setupSolverConstraintBlockGPU) in GPU 
struct PxgSolverConstraintManagerConstants
{
	PxU32 mEdgeIndex;
	PxU32 mConstraintWriteBackIndex;
};


}

#endif

