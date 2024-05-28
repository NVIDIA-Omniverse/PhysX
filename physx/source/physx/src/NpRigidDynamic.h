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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef NP_RIGID_DYNAMIC_H
#define NP_RIGID_DYNAMIC_H

#include "common/PxMetaData.h"
#include "PxRigidDynamic.h"
#include "NpRigidBodyTemplate.h"

namespace physx
{
typedef NpRigidBodyTemplate<PxRigidDynamic> NpRigidDynamicT;

class NpRigidDynamic : public NpRigidDynamicT
{
public:
// PX_SERIALIZATION
									NpRigidDynamic(PxBaseFlags baseFlags) : NpRigidDynamicT(baseFlags) {}

				void				preExportDataReset();
	virtual		void				requiresObjects(PxProcessPxBaseCallback& c);

	static		NpRigidDynamic*		createObject(PxU8*& address, PxDeserializationContext& context);
	static		void				getBinaryMetaData(PxOutputStream& stream);	
//~PX_SERIALIZATION
									NpRigidDynamic(const PxTransform& bodyPose);
	virtual							~NpRigidDynamic();

	// PxBase
	virtual		void				release()	PX_OVERRIDE PX_FINAL;
	//~PxBase

	// PxActor
	virtual		PxActorType::Enum	getType() const PX_OVERRIDE PX_FINAL	{ return PxActorType::eRIGID_DYNAMIC; }
	//~PxActor

	// PxRigidActor
	PX_FORCE_INLINE	PxTransform		getGlobalPoseFast() const
	{
		const Sc::BodyCore& body = getCore();
		// PT:: tag: scalar transform*transform
		return body.getBody2World() * body.getBody2Actor().getInverse();
	}

	virtual		PxTransform			getGlobalPose() const	PX_OVERRIDE PX_FINAL
	{
		NP_READ_CHECK(getNpScene());
		PX_CHECK_SCENE_API_READ_FORBIDDEN_EXCEPT_COLLIDE_AND_RETURN_VAL(getNpScene(), "PxRigidDynamic::getGlobalPose() not allowed while simulation is running (except during PxScene::collide()).", PxTransform(PxIdentity));
		return getGlobalPoseFast();
	}

	virtual		void 				setGlobalPose(const PxTransform& pose, bool autowake)	PX_OVERRIDE PX_FINAL;
	//~PxRigidActor

	// PxRigidBody
	// Center of mass pose
	virtual		void				setCMassLocalPose(const PxTransform&)	PX_OVERRIDE PX_FINAL;

	// Acceleration
	virtual		PxVec3				getLinearAcceleration()		const PX_OVERRIDE PX_FINAL;
	virtual		PxVec3				getAngularAcceleration()	const PX_OVERRIDE PX_FINAL;

	// Force/Torque modifiers
	virtual		void				addForce(const PxVec3&, PxForceMode::Enum mode, bool autowake)	PX_OVERRIDE PX_FINAL;
	virtual		void				addTorque(const PxVec3&, PxForceMode::Enum mode, bool autowake)	PX_OVERRIDE PX_FINAL;
	virtual		void				clearForce(PxForceMode::Enum mode)	PX_OVERRIDE PX_FINAL;
	virtual		void				clearTorque(PxForceMode::Enum mode)	PX_OVERRIDE PX_FINAL;
	virtual		void				setForceAndTorque(const PxVec3& force, const PxVec3& torque, PxForceMode::Enum mode = PxForceMode::eFORCE)	PX_OVERRIDE PX_FINAL;
	//~PxRigidBody

	// PxRigidDynamic
	virtual		void				setKinematicTarget(const PxTransform& destination)	PX_OVERRIDE PX_FINAL;
	virtual		bool				getKinematicTarget(PxTransform& target)	const	PX_OVERRIDE PX_FINAL;
	// Sleeping
	virtual		bool				isSleeping() const	PX_OVERRIDE PX_FINAL;
	virtual		void				setSleepThreshold(PxReal threshold)	PX_OVERRIDE PX_FINAL;
	virtual		PxReal				getSleepThreshold() const	PX_OVERRIDE PX_FINAL;
	virtual		void				setStabilizationThreshold(PxReal threshold)	PX_OVERRIDE PX_FINAL;
	virtual		PxReal				getStabilizationThreshold() const	PX_OVERRIDE PX_FINAL;
	virtual		void				setWakeCounter(PxReal wakeCounterValue)	PX_OVERRIDE PX_FINAL;
	virtual		PxReal				getWakeCounter() const	PX_OVERRIDE PX_FINAL;
	virtual		void				wakeUp()	PX_OVERRIDE PX_FINAL;
	virtual		void				putToSleep()	PX_OVERRIDE PX_FINAL;
	// Lock flags
	virtual		PxRigidDynamicLockFlags getRigidDynamicLockFlags() const	PX_OVERRIDE PX_FINAL;
	virtual		void				setRigidDynamicLockFlags(PxRigidDynamicLockFlags flags)	PX_OVERRIDE PX_FINAL;
	virtual		void				setRigidDynamicLockFlag(PxRigidDynamicLockFlag::Enum flag, bool value)	PX_OVERRIDE PX_FINAL;
	// Velocity
	virtual		void				setLinearVelocity(const PxVec3&, bool autowake)	PX_OVERRIDE PX_FINAL;
	virtual		void				setAngularVelocity(const PxVec3&, bool autowake)	PX_OVERRIDE PX_FINAL;
	virtual		void				setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters)	PX_OVERRIDE PX_FINAL;
	virtual		void				getSolverIterationCounts(PxU32 & positionIters, PxU32 & velocityIters) const	PX_OVERRIDE PX_FINAL;
	virtual		PxReal				getContactReportThreshold() const	PX_OVERRIDE PX_FINAL;
	virtual		void				setContactReportThreshold(PxReal threshold)	PX_OVERRIDE PX_FINAL;
	virtual		PxRigidDynamicGPUIndex getGPUIndex() const PX_OVERRIDE PX_FINAL;
	//~PxRigidDynamic

	// NpRigidActorTemplate
	virtual		void				switchToNoSim()	PX_OVERRIDE PX_FINAL;
	virtual		void				switchFromNoSim()	PX_OVERRIDE PX_FINAL;
	//~NpRigidActorTemplate

	PX_FORCE_INLINE void			wakeUpInternal();
					void			wakeUpInternalNoKinematicTest(bool forceWakeUp, bool autowake);

	static PX_FORCE_INLINE size_t	getCoreOffset()				{ return PX_OFFSET_OF_RT(NpRigidDynamic, mCore);			}
	static PX_FORCE_INLINE size_t	getNpShapeManagerOffset()	{ return PX_OFFSET_OF_RT(NpRigidDynamic, mShapeManager);	}

#if PX_CHECKED
	PX_FORCE_INLINE	bool			checkConstraintValidity() const	{ return true;	}
#endif

private:
	PX_FORCE_INLINE	void			setKinematicTargetInternal(const PxTransform& destination);

#if PX_ENABLE_DEBUG_VISUALIZATION
public:
				void				visualize(PxRenderOutput& out, NpScene& scene, float scale)	const;
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif
};

PX_FORCE_INLINE void NpRigidDynamic::wakeUpInternal()
{
	PX_ASSERT(getNpScene());

	const Sc::BodyCore& body = getCore();

	const PxRigidBodyFlags currentFlags = body.getFlags();

	if (!(currentFlags & PxRigidBodyFlag::eKINEMATIC))  // kinematics are only awake when a target is set, else they are asleep
		wakeUpInternalNoKinematicTest(false, true);
}


}

#endif
