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

#ifndef PXG_CONSTRAINT_BLOCK_H
#define PXG_CONSTRAINT_BLOCK_H

#include "PxvConfig.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "PxgSolverBody.h"

namespace physx
{

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4324)
#endif

struct PxgBlockSolverContactHeader	
{
	PX_ALIGN(128, float4	invMass0_1_angDom0_1[32]);		//512		512
	PX_ALIGN(128, float4	normal_staticFriction[32]);		//1024		512
	PX_ALIGN(128, PxReal	accumNormalForce[32]);
	//Only used by articulation constraints. Forces the minimum normal force for friction.
	//Without this, articulations can drift due to no normal force when multi-link systems contact with surfaces.
	PX_ALIGN(128, PxReal	minNormalForce[32]);

	PX_ALIGN(128, PxU32		flags[32]);						//1152		128
	PX_ALIGN(128, PxU32		numNormalConstr[32]);			//1280		128
	PX_ALIGN(128, PxU32		forceWritebackOffset[32]);		//1408		128

	// To use different mass for mass-splitting every sub-timestep (or iteration),
	// recipResponse, velMultipler, biasCoefficient, etc. are computed every sub-timestep (or iteration).
	// To compute them every sub-timestep (or iteration), restitution and cfm are additionally stored.
	// This does not change the previous impulse formulation, but a different mass is used due to mass-splitting.
	PX_ALIGN(128, PxReal	restitution[32]);
	PX_ALIGN(128, PxReal	cfm[32]);

};

struct PxgBlockSolverFrictionHeader
{
	PX_ALIGN(128, float4	frictionNormals[2][32]);		//1024		1024
	PX_ALIGN(128, PxU32		numFrictionConstr[32]);			//1152		128
	PX_ALIGN(128, PxReal	dynamicFriction[32]);			//1280		128
	PX_ALIGN(128, PxU32		broken[32]);					//1408		128
};

//PX_COMPILE_TIME_ASSERT(sizeof(PxgBlockSolverContactHeader) == 1280);

/**
\brief A single rigid body contact point for the solver.
*/
struct PxgBlockSolverContactPoint
{
	// To use different mass for mass-splitting every sub-timestep (or iteration),
	// unitResponse, recipResponse, velMultiplier, etc. are computed every sub-timestep (or iteration).
	// To compute them at every sub-timestep (or iteration), resp0, resp1, and other relevant data are stored additionally.
	// This does not change the previous impulse formulation, but a different mass is used due to mass-splitting.

	PX_ALIGN(128, float4	raXn_targetVelocity[32]); 
	PX_ALIGN(128, float4	rbXn_maxImpulse[32]);
	PX_ALIGN(128, PxReal	appliedForce[32]);

	PX_ALIGN(128, PxReal	resp0[32]);
	PX_ALIGN(128, PxReal	resp1[32]);

	// Two coefficients used in "queryReducedCompliantContactCoefficients" and "computeCompliantContactCoefficients"
	PX_ALIGN(128, PxReal	coeff0[32]); 
	PX_ALIGN(128, PxReal	coeff1[32]);
};

struct PxgArticulationBlockResponse
{
	PX_ALIGN(128, float	deltaRALin_x[32]);
	PX_ALIGN(128, float	deltaRALin_y[32]);
	PX_ALIGN(128, float	deltaRALin_z[32]);
	PX_ALIGN(128, float	deltaRAAng_x[32]);
	PX_ALIGN(128, float	deltaRAAng_y[32]);
	PX_ALIGN(128, float	deltaRAAng_z[32]);
	PX_ALIGN(128, float	deltaRBLin_x[32]);
	PX_ALIGN(128, float	deltaRBLin_y[32]);
	PX_ALIGN(128, float	deltaRBLin_z[32]);
	PX_ALIGN(128, float	deltaRBAng_x[32]);
	PX_ALIGN(128, float	deltaRBAng_y[32]);
	PX_ALIGN(128, float	deltaRBAng_z[32]);
};

/**
\brief A single friction constraint for the solver.
*/
struct PxgBlockSolverContactFriction
{
	// To use different mass for mass-splitting every sub-timestep (or iteration),
	// unitResponse, recipResponse, velMultiplier, etc. are computed every sub-timestep (or iteration).
	// To compute them at every sub-timestep (or iteration), resp0, resp1, and other relevant data are stored additionally.
	// This does not change the previous impulse formulation, but a different mass is used due to mass-splitting.

	PX_ALIGN(128, float4	raXn_bias[32]);
	PX_ALIGN(128, float4	rbXn_targetVelW[32]);
	PX_ALIGN(128, PxReal	appliedForce[32]);
	PX_ALIGN(128, PxReal	resp0[32]);
	PX_ALIGN(128, PxReal	resp1[32]);
};


struct PxgTGSBlockSolverContactHeader
{
	PX_ALIGN(128, float4	invMass0_1_angDom0_1[32]);		//512		512
	PX_ALIGN(128, float4	normal_staticFriction[32]);		//1024		512

	//Only used by articulation constraints. Forces the minimum normal force for friction.
	//Without this, articulations can drift due to no normal force when multi-link systems contact with surfaces.
	PX_ALIGN(128, PxReal	minNormalForce[32]);

	PX_ALIGN(128, PxF32		maxPenBias[32]);				//1152		128
	PX_ALIGN(128, PxU32		flags[32]);						//1408		128
	PX_ALIGN(128, PxU32		numNormalConstr[32]);			//1536		128
	PX_ALIGN(128, PxU32		forceWritebackOffset[32]);		//1664		128

	// To use different mass for mass-splitting every sub-timestep (or iteration),
	// recipResponse, velMultipler, biasCoefficient, etc. are computed every sub-timestep (or iteration).
	// To compute them every sub-timestep (or iteration), restitution, cfm, and p8 are additionally stored.
	// This does not change the previous impulse formulation, but a different mass is used due to mass-splitting.

	PX_ALIGN(128, PxReal	restitutionXdt[32]);
	PX_ALIGN(128, PxReal	cfm[32]);						
	PX_ALIGN(128, PxReal	p8[32]);						
};

struct PxgTGSBlockSolverFrictionHeader
{
	PX_ALIGN(128, float4	frictionNormals[2][32]);		//1024		1024
	PX_ALIGN(128, PxU32		numFrictionConstr[32]);			//1152		128
	PX_ALIGN(128, PxReal	dynamicFriction[32]);			//1280		128
	PX_ALIGN(128, PxU32		broken[32]);					//1408		128
	PX_ALIGN(128, PxReal	biasCoefficient[32]);
	PX_ALIGN(128, PxReal	torsionalFrictionScale[32]);
};

//PX_COMPILE_TIME_ASSERT(sizeof(PxgBlockSolverContactHeader) == 1280);

/**
\brief A single rigid body contact point for the solver.
*/
struct PxgTGSBlockSolverContactPoint
{
	// To use different mass for mass-splitting every sub-timestep (or iteration),
	// unitResponse, recipResponse, velMultiplier, etc. are computed every sub-timestep (or iteration).
	// To compute them at every sub-timestep (or iteration), resp0, resp1, and other relevant data are stored additionally.
	// This does not change the previous impulse formulation, but a different mass is used due to mass-splitting.

	PX_ALIGN(128, float4	raXn_extraCoeff[32]); // For contact constraints, extraCoeff is the compliant contact coefficient "a"
												  // used in "computeCompliantContactCoefficientsTGS".

	PX_ALIGN(128, float4	rbXn_targetVelW[32]);
	PX_ALIGN(128, PxReal	separation[32]);
	PX_ALIGN(128, PxReal	maxImpulse[32]);
	PX_ALIGN(128, PxReal	appliedForce[32]);
	PX_ALIGN(128, PxReal	biasCoefficient[32]);

	PX_ALIGN(128, PxReal	resp0[32]);
	PX_ALIGN(128, PxReal	resp1[32]);
};

/**
\brief A single friction constraint for the solver.
*/
struct PxgTGSBlockSolverContactFriction
{
	// To use different mass for mass-splitting every sub-timestep (or iteration),
	// unitResponse, recipResponse, velMultiplier, etc. are computed every sub-timestep (or iteration).
	// To compute them every sub-timestep (or iteration), resp0 and resp1 are stored separately.
	// This does not change the previous impulse formulation, but a different mass is used due to mass-splitting.

	PX_ALIGN(128, float4	raXn_error[32]); 
	PX_ALIGN(128, float4	rbXn_targetVelW[32]);
	PX_ALIGN(128, PxReal	appliedForce[32]);

	PX_ALIGN(128, PxReal	resp0[32]);
	PX_ALIGN(128, PxReal	resp1[32]);
};

struct PxgContactBlockParams
{
	PxgBlockSolverContactHeader*	blockContactHeader;
	PxgBlockSolverFrictionHeader*	blockFrictionHeader;
	PxgBlockSolverContactPoint*		blockContactPoints;
	PxgBlockSolverContactFriction*	blockFrictions;
};

struct PxgTGSContactBlockParams
{
	PxgTGSBlockSolverContactHeader*		blockContactHeader;
	PxgTGSBlockSolverFrictionHeader*	blockFrictionHeader;
	PxgTGSBlockSolverContactPoint*		blockContactPoints;
	PxgTGSBlockSolverContactFriction*	blockFrictions;
};



#if PX_VC
#pragma warning(pop)
#endif

}



#endif

