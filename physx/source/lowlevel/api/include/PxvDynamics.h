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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXV_DYNAMICS_H
#define PXV_DYNAMICS_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxIntrinsics.h"
#include "PxRigidDynamic.h"

namespace physx
{

/*!
\file
Dynamics interface.
*/

struct PxsRigidCore
{
	PxsRigidCore() : mFlags(0), solverIterationCounts(0)	{}
	PxsRigidCore(const PxEMPTY) : mFlags(PxEmpty)			{}

	PX_ALIGN_PREFIX(16)
	PxTransform			body2World PX_ALIGN_SUFFIX(16);
	PxRigidBodyFlags	mFlags;					// API body flags
	PxU16				solverIterationCounts;	// vel iters are in low word and pos iters in high word.

	PX_FORCE_INLINE	PxU32 isKinematic()			const	{ return mFlags & PxRigidBodyFlag::eKINEMATIC;				}
	PX_FORCE_INLINE PxU32 hasCCD()				const	{ return mFlags & PxRigidBodyFlag::eENABLE_CCD;				}
	PX_FORCE_INLINE	PxU32 hasCCDFriction()		const	{ return mFlags & PxRigidBodyFlag::eENABLE_CCD_FRICTION;	}
	PX_FORCE_INLINE	PxU32 hasIdtBody2Actor()	const	{ return mFlags & PxRigidBodyFlag::eRESERVED;				}
};
PX_COMPILE_TIME_ASSERT(sizeof(PxsRigidCore) == 32);

#define PXV_CONTACT_REPORT_DISABLED	PX_MAX_F32

struct PxsBodyCore : public PxsRigidCore
{
	PxsBodyCore() : PxsRigidCore() { fixedBaseLink = PxU8(0); }
	PxsBodyCore(const PxEMPTY) : PxsRigidCore(PxEmpty)		{}

	PX_FORCE_INLINE	const PxTransform& getBody2Actor()	const	{ return body2Actor;	}
	PX_FORCE_INLINE	void setBody2Actor(const PxTransform& t)
	{
		if(t.p.isZero() && t.q.isIdentity())
			mFlags.raise(PxRigidBodyFlag::eRESERVED);
		else
			mFlags.clear(PxRigidBodyFlag::eRESERVED);

		body2Actor = t;
	}
	protected:
	PxTransform				body2Actor;
	public:
	PxReal					ccdAdvanceCoefficient;	//64

	PxVec3					linearVelocity;
	PxReal					maxPenBias;

	PxVec3					angularVelocity;
	PxReal					contactReportThreshold;	//96
    
	PxReal					maxAngularVelocitySq;
	PxReal					maxLinearVelocitySq;
	PxReal					linearDamping;
	PxReal					angularDamping;			//112

	PxVec3					inverseInertia;
	PxReal					inverseMass;			//128
	
	PxReal					maxContactImpulse;			
	PxReal					sleepThreshold;
	union
	{
		PxReal				freezeThreshold;
		PxReal				cfmScale;
	};
		
	PxReal					wakeCounter;			//144 this is authoritative wakeCounter

	PxReal					solverWakeCounter;		//this is calculated by the solver when it performs sleepCheck. It is committed to wakeCounter in ScAfterIntegrationTask if the body is still awake.
	PxU32					numCountedInteractions;
	PxReal					offsetSlop;				//Slop value used to snap contact line of action back in-line with the COM
	PxU8					isFastMoving;			//This could be a single bit but it's a u8 at the moment for simplicity's sake
	PxU8					disableGravity;			//This could be a single bit but it's a u8 at the moment for simplicity's sake
	PxRigidDynamicLockFlags	lockFlags;				//This is u8. 
	PxU8					fixedBaseLink;			//160 This indicates whether the articulation link has PxArticulationFlag::eFIX_BASE. All fits into 16 byte alignment
	
	// PT: moved from Sc::BodyCore ctor - we don't want to duplicate all this in immediate mode
	PX_FORCE_INLINE	void	init(	const PxTransform& bodyPose,
									const PxVec3& inverseInertia_, PxReal inverseMass_,
									PxReal wakeCounter_, PxReal scaleSpeed,
									PxReal linearDamping_, PxReal angularDamping_,
									PxReal maxLinearVelocitySq_, PxReal maxAngularVelocitySq_,
									PxActorType::Enum type)
	{
		PX_ASSERT(bodyPose.p.isFinite());
		PX_ASSERT(bodyPose.q.isFinite());

		// PT: TODO: unify naming convention

		// From PxsRigidCore
		body2World				= bodyPose;
		mFlags					= PxRigidBodyFlags();
		solverIterationCounts	= (1 << 8) | 4;

		setBody2Actor(PxTransform(PxIdentity));

		ccdAdvanceCoefficient	= 0.15f;
		linearVelocity			= PxVec3(0.0f);
		maxPenBias				= -1e32f;//-PX_MAX_F32;
		angularVelocity			= PxVec3(0.0f);
		contactReportThreshold	= PXV_CONTACT_REPORT_DISABLED;
		maxAngularVelocitySq	= maxAngularVelocitySq_;
		maxLinearVelocitySq		= maxLinearVelocitySq_;
		linearDamping			= linearDamping_;
		angularDamping			= angularDamping_;
		inverseInertia			= inverseInertia_;
		inverseMass				= inverseMass_;
		maxContactImpulse		= 1e32f;// PX_MAX_F32;
		sleepThreshold			= 5e-5f * scaleSpeed * scaleSpeed;
		if(type == PxActorType::eARTICULATION_LINK)
			cfmScale			= 0.025f;
		else
			freezeThreshold		= 2.5e-5f * scaleSpeed * scaleSpeed;
		wakeCounter				= wakeCounter_;
		offsetSlop				= 0.f;
		// PT: this one is not initialized?
		//solverWakeCounter
		// PT: these are initialized in BodySim ctor
		//numCountedInteractions;
		//numBodyInteractions;
		isFastMoving			= false;
		disableGravity			= false;
		lockFlags				= PxRigidDynamicLockFlags(0);
	}
};

PX_COMPILE_TIME_ASSERT(sizeof(PxsBodyCore) == 160);

}

#endif
