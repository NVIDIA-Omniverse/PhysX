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

#ifndef PXG_SOLVER_CONSTRAINT_BLOCK_1D_H
#define PXG_SOLVER_CONSTRAINT_BLOCK_1D_H

#include "foundation/PxVec3.h"
#include "PxConstraintDesc.h"
#include "DySolverConstraintTypes.h"
#include "PxgSolverConstraintDesc.h"
#include "vector_types.h"
#include "vector_functions.h"


namespace physx
{

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
struct PxgBlockSolverConstraint1DHeader
{
	PX_ALIGN(128, PxU16		rowCounts[32]);								// numbers of rows each 1D constraints 
	PX_ALIGN(128, PxU16		breakable[32]);											// indicate whether the constraint are breakable or not 
	PX_ALIGN(128, PxReal	angBreakImpulse[32]);
	PX_ALIGN(128, float4	body0WorldOffset_linBreakImpulse[32]);
	PX_ALIGN(128, PxReal	invMass0D0[32]);
	PX_ALIGN(128, PxReal	invMass1D1[32]);
	PX_ALIGN(128, PxReal	invInertiaScale0[32]);
	PX_ALIGN(128, PxReal	invInertiaScale1[32]);
	PX_ALIGN(128, PxU32		writeBackOffset[32]);

	PX_ALIGN(128, PxReal	cfm[32]);
};

  

struct PxgBlockSolverConstraint1DCon 
{
public:

	// To use different mass for mass-splitting every sub-timestep (or iteration),
	// unitResponse, recipResponse, velMultiplier, etc. are computed every sub-timestep (or iteration).
	// To compute them at every sub-timestep (or iteration), resp0, resp1, and other relevant data are stored additionally.
	PX_ALIGN(256, float4	lin0XYZ_minImpulse[32]);			//512	512		//!< linear velocity projection (body 0) and min impulse term	
	PX_ALIGN(256, float4	lin1XYZ_maxImpulse[32]);			//1024	512		//!< linear velocity projection (body 1) and max impulse term
	PX_ALIGN(256, float4	ang0XYZ_resp0[32]);					//1536	512		//!< angular velocity projection (body 0) and resp0
	PX_ALIGN(256, float4	ang1XYZ_resp1[32]);					//2048	512		//!< angular velocity projection (body 1) and resp1
	PX_ALIGN(256, PxReal	initJointSpeed[32]);				
} ;

struct PxgBlockSolverConstraint1DMod 
{
public:
	PX_ALIGN(128, PxVec3	ang0Writeback[32]);					//!< unscaled angular velocity projection (body 0)
	PX_ALIGN(128, PxReal	appliedForce[32]);					//!< applied force to correct velocity+bias
	PX_ALIGN(128, PxU32		flags[32]);							
	PX_ALIGN(128, PxReal	residual[32]);

	// coeff0, coeff1: coefficients used to compute constant, unbiasedConstant, velMultiplier, and impulseMultiplier.
	// See also "queryReduced1dConstraintSolverConstantsPGS" 
	PX_ALIGN(128, PxReal	coeff0[32]);						
	PX_ALIGN(128, PxReal	coeff1[32]);						
} ;

#if PX_VC
#pragma warning(pop)
#endif


#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
struct PxgTGSBlockSolverConstraint1DHeader
{
	PX_ALIGN(128, uchar4	rowCounts_breakable_orthoAxisCount[32]);
	PX_ALIGN(128, float4	rAWorld_invMass0D0[32]);
	PX_ALIGN(128, float4	rBWorld_invMass1D1[32]);
	PX_ALIGN(128, PxReal	invInertiaScale0[32]);
	PX_ALIGN(128, PxReal	invInertiaScale1[32]);
	PX_ALIGN(128, PxU32		writeBackOffset[32]);

	//Orthogonalization data
	PX_ALIGN(128, float4	angOrthoAxis0_recipResponseW[3][32]);
	PX_ALIGN(128, float4	angOrthoAxis1_ErrorW[3][32]);

	PX_ALIGN(128, PxReal	linBreakImpulse[32]);
	PX_ALIGN(128, PxReal	angBreakImpulse[32]);

	PX_ALIGN(128, PxReal	cfm[32]);
};



struct PxgTGSBlockSolverConstraint1DCon
{
public:
	// For rigid body joints, coef0, coef1, coef2, and coef3 store initBias, biasScale, velMultiplier, and velTarget,
	// respectively.
	 
	// For articulation, the coefficients used in "compute1dConstraintSolverConstantsTGS" and
	// "queryReduced1dConstraintSolverConstantsTGS" are stored in the last (w) component. 

	PX_ALIGN(128, float4	lin0XYZ_initBiasOrCoeff0[32]);				//512	512		//!< linear velocity projection (body 0) and an additional coef	
	PX_ALIGN(128, float4	lin1XYZ_biasScaleOrCoeff1[32]);				//1024	512		//!< linear velocity projection (body 1) and an additional coef
	PX_ALIGN(128, float4	ang0XYZ_velMultiplierOrCoeff2[32]);			//1536	512		//!< angular velocity projection (body 0) and an additional coef
	PX_ALIGN(128, float4	ang1XYZ_velTargetOrCoeff3[32]);				//2048	512		//!< angular velocity projection (body 1) and an additional coef

	// resp0, resp1, and other relevant data are stored additionally.
	PX_ALIGN(128, PxReal	resp0[32]);
	PX_ALIGN(128, PxReal	resp1[32]);
	PX_ALIGN(128, PxReal	geometricError[32]);

	PX_ALIGN(128, PxReal	minImpulse[32]);
	PX_ALIGN(128, PxReal	maxImpulse[32]);
	PX_ALIGN(128, PxReal	maxBias[32]);
	PX_ALIGN(128, PxReal	angularErrorScale[32]);
	PX_ALIGN(128, PxU32		flags[32]);
	PX_ALIGN(128, PxReal	appliedForce[32]);
	PX_ALIGN(128, PxReal	residual[32]);
};


#if PX_VC
#pragma warning(pop)
#endif


PX_CUDA_CALLABLE PX_FORCE_INLINE void init(PxgBlockSolverConstraint1DCon& ccon, PxgBlockSolverConstraint1DMod& cmod,
						  const PxVec3& _linear0, const PxVec3& _linear1, 
						  const PxVec3& _angular0, const PxVec3& _angular1,
						  const PxReal _minImpulse, const PxReal _maxImpulse, 
						  const PxU32 index)
{
	PX_ASSERT(_linear0.isFinite());
	PX_ASSERT(_linear1.isFinite());

	ccon.lin0XYZ_minImpulse[index] = make_float4(_linear0.x, _linear0.y, _linear0.z, _minImpulse);
	ccon.lin1XYZ_maxImpulse[index] = make_float4(_linear1.x, _linear1.y, _linear1.z, _maxImpulse);
	ccon.ang0XYZ_resp0[index] = make_float4(_angular0.x, _angular0.y, _angular0.z, 0.f);
	ccon.ang1XYZ_resp1[index] = make_float4(_angular1.x, _angular1.y, _angular1.z, 0.f);
	ccon.initJointSpeed[index] = 0.f;

	cmod.coeff0[index] = 0.f;
	cmod.coeff1[index] = 0.f;

	cmod.flags[index]					= 0;
	cmod.appliedForce[index]			= 0.f;
	cmod.residual[index]				= 0.f;
}

struct PxgJointBlockParams
{
	PxgBlockSolverConstraint1DHeader* jointHeader;
	PxgBlockSolverConstraint1DCon* jointCon;
	PxgBlockSolverConstraint1DMod* jointMod;
	PxReal dt;
	PxReal invDt;
};

struct PxgTGSJointBlockParams
{
	PxgTGSBlockSolverConstraint1DHeader* jointHeader;
	PxgTGSBlockSolverConstraint1DCon* jointCon;
	PxReal dt;
	PxReal totalDt;
	PxReal invDt;
	PxReal invTotalDt;
	PxReal lengthScale;
	PxReal biasCoefficient;
};

}

#endif
