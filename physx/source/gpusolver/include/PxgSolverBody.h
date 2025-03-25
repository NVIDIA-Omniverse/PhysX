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

#ifndef PXG_SOLVER_BODY_H
#define PXG_SOLVER_BODY_H

#include "PxvConfig.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxTransform.h"
#if !PX_CUDA_COMPILER
#include <vector_types.h>
#endif
#include "AlignedMat33.h"
#include "AlignedTransform.h"
#include "PxNodeIndex.h"
#include "PxSpatialMatrix.h"

namespace physx
{

class PxsRigidBody;
struct PxgSolverBody;
class PxgArticulation;

struct PxgSolverTxIData
{
	PxTransform			deltaBody2World;	// 64 body delta transform
	PxMat33				sqrtInvInertia;		// 36 inverse inertia in world space
};

struct PxgSolverBodyPrepData
{
#if !PX_CUDA_COMPILER
	PX_ALIGN(16, PxVec3			initialAngVel);					//	12 initial ang vel
	PxReal						penBiasClamp;					//	16 the penetration bias clamp
	PxVec3						initialLinVel;					//	28 initial lin vel
	PxReal						invMass;						//	32 inverse mass
#else
	float4						initialAngVelXYZ_penBiasClamp;
	float4						initialLinVelXYZ_invMassW;
#endif	

	PxAlignedTransform			body2World;

#if !PX_CUDA_COMPILER
	PX_FORCE_INLINE PxReal projectVelocity(const PxVec3& lin, const PxVec3& ang)	const
	{
		return initialLinVel.dot(lin) + initialAngVel.dot(ang);
	}
#else
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal projectVelocity(const PxVec3& lin, const PxVec3& ang)	const
	{
		//return initialLinVel.dot(lin) + initialAngVel.dot(ang);
		PxVec3 initialLinVel(initialLinVelXYZ_invMassW.x, initialLinVelXYZ_invMassW.y, initialLinVelXYZ_invMassW.z);
		PxVec3 initialAngVel(initialAngVelXYZ_penBiasClamp.x, initialAngVelXYZ_penBiasClamp.y, initialAngVelXYZ_penBiasClamp.z);
		return initialLinVel.dot(lin) + initialAngVel.dot(ang);
	}
#endif
};

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4324)
#endif

struct PxgSolverBodyData : public PxgSolverBodyPrepData
{
	PxNodeIndex		islandNodeIndex;		// 40
	PxReal			reportThreshold;		// 44 contact force threshold	
	PxReal			maxImpulse;				// 48
	PxU32			flags;					// 52 hasSpeculativeCCD etc.
	PxReal			offsetSlop;
};
#if PX_VC
#pragma warning(pop)
#endif

PX_COMPILE_TIME_ASSERT((sizeof(PxgSolverBodyData)& 0xf) == 0);

class PxgSolverExtBody
{
public:
	union
	{
		const PxgArticulation* articulation;
		const PxgSolverBodyData* body;
	};

	//if linkIndex is 0xffff, the solver body is rigid body, otherwise, it is articulation
	PxU16 linkIndex;
	PxU16 isKinematic;
	PxU32 bodyIndex;
	PxU32 islandNodeIndex;

};

struct PxgSolverExtBody2
{
	PxSpatialMatrix mSpatialResponse;		//144
	Cm::UnAlignedSpatialVector velocity;	//168
	PxTransform body2World;					//196
	PxReal penBiasClamp;					//200
	PxReal maxImpulse;						//204
	PxU16 linkIndex;						//206
	PxU16 isKinematic;						//208
	PxU32 bodyIndex;						//212
	PxNodeIndex islandNodeIndex;			//216
	PxReal cfm;								//220
	PxReal offsetSlop;						//224
};

//we need to DMA back the sleep data to CPU. PxgBodySim has the same information. However, PxgBodySim is too
//big to dma back.
struct PxgSolverBodySleepData
{
	PxReal						wakeCounter;
	PxU32						internalFlags;
};

#if PX_VC
#pragma warning(push)
#pragma warning (disable : 4201)
#endif
struct PxgSolverBody
{
#if !PX_CUDA_COMPILER
	PX_ALIGN(16, PxVec3	linearVelocity);	// post-solver linear velocity in world space
	PxU32 pad;
	PxVec3				angularVelocity;	// post-solver angular velocity in world space	
	PxU32 pad2;	
#else
	float4				linearVelocity;
	float4				angularVelocity;
#endif
};
#if PX_VC
#pragma warning(pop)
#endif

PX_COMPILE_TIME_ASSERT(sizeof(PxgSolverBody) == 32);


#if PX_VC
#pragma warning(push)
#pragma warning (disable : 4201)
#endif
struct PxgTGSSolverBody
{
#if !PX_CUDA_COMPILER
	PX_ALIGN(16, PxVec3		linearVelocity);				// 12 post-solver linear velocity in world space
	PxVec3					angularVelocity;				// 24 post-solver angular velocity in world space	
	PxVec3					linearDelta;					// 36 linear delta motion in world space
	PxVec3					angularDelta;					// 48 angular delta motion in world space
#else
	float4					linearVelocityXYZ_angX;
	float4					angularVelocityYZ_linDeltaXY;
	float4					linDeltaZ_angDeltaXYZ;
#endif
};
#if PX_VC
#pragma warning(pop)
#endif

PX_COMPILE_TIME_ASSERT(sizeof(PxgSolverBody) == 32);


struct PxgSolverReferences
{
	PxU32 mRemappedBodyIndex;
};

}

#endif

