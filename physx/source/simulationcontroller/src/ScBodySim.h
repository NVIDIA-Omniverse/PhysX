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

#ifndef SC_BODYSIM_H
#define SC_BODYSIM_H

#include "foundation/PxUtilities.h"
#include "foundation/PxIntrinsics.h"
#include "ScRigidSim.h"
#include "PxvDynamics.h"
#include "ScBodyCore.h"
#include "ScSimStateData.h"
#include "PxRigidDynamic.h"
#include "PxsRigidBody.h"

namespace physx
{
namespace Bp
{
	class BoundsArray;
}
	struct PxsExternalAccelerationProvider;
	class PxsTransformCache;
namespace Sc
{
	class Scene;
	class ArticulationSim;

#if PX_VC 
    #pragma warning(push)   
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	class BodySim : public RigidSim
	{
	public:
												BodySim(Scene&, BodyCore&, bool);
		virtual									~BodySim();

						void					switchToKinematic();
						void					switchToDynamic();

		PX_FORCE_INLINE const SimStateData*		getSimStateData(bool isKinematic)	const	{ return (mSimStateData && (checkSimStateKinematicStatus(isKinematic)) ? mSimStateData : NULL); }
		PX_FORCE_INLINE SimStateData*			getSimStateData(bool isKinematic)			{ return (mSimStateData && (checkSimStateKinematicStatus(isKinematic)) ? mSimStateData : NULL); }
		PX_FORCE_INLINE SimStateData*			getSimStateData_Unchecked()			const	{ return mSimStateData; }
		PX_FORCE_INLINE	bool					checkSimStateKinematicStatus(bool isKinematic) const
												{
													PX_ASSERT(mSimStateData);
													return mSimStateData->isKine() == isKinematic;
												}

						void					setKinematicTarget(const PxTransform& p);

						void					addSpatialAcceleration(const PxVec3* linAcc, const PxVec3* angAcc);
						void					setSpatialAcceleration(const PxVec3* linAcc, const PxVec3* angAcc);
						void					clearSpatialAcceleration(bool force, bool torque);
						void					addSpatialVelocity(const PxVec3* linVelDelta, const PxVec3* angVelDelta);
						void					clearSpatialVelocity(bool force, bool torque);

						void					updateCached(PxBitMapPinned* shapeChangedMap);
						void					updateCached(PxsTransformCache& transformCache, Bp::BoundsArray& boundsArray);
						void					updateContactDistance(PxReal* contactDistance, PxReal dt, const Bp::BoundsArray& boundsArray);

		// hooks for actions in body core when it's attached to a sim object. Generally
		// we get called after the attribute changed.
			
		virtual			void					postActorFlagChange(PxU32 oldFlags, PxU32 newFlags)	PX_OVERRIDE;
						void					postBody2WorldChange();
						void					postSetWakeCounter(PxReal t, bool forceWakeUp);
						void					postPosePreviewChange(PxU32 posePreviewFlag);  // called when PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW changes

		PX_FORCE_INLINE const PxTransform&		getBody2World()		const	{ return getBodyCore().getCore().body2World;		}
		PX_FORCE_INLINE const PxTransform&		getBody2Actor()		const	{ return getBodyCore().getCore().getBody2Actor();	}
		PX_FORCE_INLINE const PxsRigidBody&		getLowLevelBody()	const	{ return mLLBody;									}
		PX_FORCE_INLINE	PxsRigidBody&			getLowLevelBody()			{ return mLLBody;									}

						void					setActive(bool active, bool asPartOfCreation=false);
						void					wakeUp();  // note: for user API call purposes only, i.e., use from BodyCore. For simulation internal purposes there is internalWakeUp().
						void					putToSleep();

						void					disableCompound();

		static			PxU32					getRigidBodyOffset()		{ return  PxU32(PX_OFFSET_OF_RT(BodySim, mLLBody));}
	
						void					activate();
						void					deactivate();

		// Kinematics
		PX_FORCE_INLINE bool					isKinematic()								const	{ return getBodyCore().getFlags() & PxRigidBodyFlag::eKINEMATIC;	}
		PX_FORCE_INLINE bool					isArticulationLink()						const	{ return getActorType() == PxActorType::eARTICULATION_LINK;			}
		PX_FORCE_INLINE bool					hasForcedKinematicNotif()					const
												{
													return getBodyCore().getFlags() & (PxRigidBodyFlag::eFORCE_KINE_KINE_NOTIFICATIONS|PxRigidBodyFlag::eFORCE_STATIC_KINE_NOTIFICATIONS);
												}
						void					calculateKinematicVelocity(PxReal oneOverDt);
						void					updateKinematicPose();
						bool					deactivateKinematic();
		// Sleeping
		virtual			void					internalWakeUp(PxReal wakeCounterValue)	PX_OVERRIDE;		// PT: TODO: does it need to be virtual?
						void					internalWakeUpArticulationLink(PxReal wakeCounterValue);	// called by ArticulationSim to wake up this link

						PxReal					updateWakeCounter(PxReal dt, PxReal energyThreshold, const Cm::SpatialVector& motionVelocity);

						void					notifyReadyForSleeping();			// inform the sleep island generation system that the body is ready for sleeping
						void					notifyNotReadyForSleeping();		// inform the sleep island generation system that the body is not ready for sleeping
		PX_FORCE_INLINE bool					checkSleepReadinessBesidesWakeCounter();  // for API triggered changes to test sleep readiness

		// PT: TODO: this is only used for the rigid bodies' sleep check, the implementations in derived classes look useless
		virtual			void					registerCountedInteraction()		PX_OVERRIDE	{ mLLBody.getCore().numCountedInteractions++; PX_ASSERT(mLLBody.getCore().numCountedInteractions);	}
		virtual			void					unregisterCountedInteraction()		PX_OVERRIDE	{ PX_ASSERT(mLLBody.getCore().numCountedInteractions); mLLBody.getCore().numCountedInteractions--;	}
		// PT: TODO: this is only used for the rigid bodies' sleep check called from the articulation sim code
		virtual			PxU32					getNumCountedInteractions()	const	PX_OVERRIDE	{ return mLLBody.getCore().numCountedInteractions;													}

		PX_FORCE_INLINE PxIntBool				isFrozen()					const	{ return PxIntBool(mLLBody.mInternalFlags & PxsRigidBody::eFROZEN);									}

		// External velocity changes - returns true if any forces were applied to this body
						bool					updateForces(PxReal dt, PxsRigidBody** updatedBodySims, PxU32* updatedBodyNodeIndices, PxU32& index, Cm::SpatialVector* acceleration, 
													PxsExternalAccelerationProvider* externalAccelerations = NULL, PxU32 maxNumExternalAccelerations = 0);

		PX_FORCE_INLINE bool					readVelocityModFlag(VelocityModFlags f) { return (mVelModState & f) != 0; }

		// Miscellaneous
		PX_FORCE_INLINE	bool					notInScene()									const	{ return mActiveListIndex == SC_NOT_IN_SCENE_INDEX; }
		PX_FORCE_INLINE	PxU32					getNbShapes()									const 	{ return mShapes.getCount(); }
		PX_FORCE_INLINE PxU32					getFlagsFast()									const	{ return getBodyCore().getFlags();					}
		PX_FORCE_INLINE	BodyCore&				getBodyCore()									const	{ return static_cast<BodyCore&>(getRigidCore());	}

		PX_FORCE_INLINE	ArticulationSim*		getArticulation()								const	{ return mArticulation; }
						void 					setArticulation(ArticulationSim* a, PxReal wakeCounter, bool asleep, PxU32 bodyIndex);

		PX_FORCE_INLINE void					onConstraintAttach()									{ raiseInternalFlag(BF_HAS_CONSTRAINTS); registerCountedInteraction(); }
						void					onConstraintDetach();

		PX_FORCE_INLINE	void					onOriginShift(const PxVec3& shift, const bool isKinematic)						
												{ 
													PX_ASSERT(!mSimStateData || checkSimStateKinematicStatus(isKinematic));
													mLLBody.mLastTransform.p -= shift; 
													if (mSimStateData && isKinematic && mSimStateData->getKinematicData()->targetValid)
														mSimStateData->getKinematicData()->targetPose.p -= shift;
												}

		PX_FORCE_INLINE bool					usingSqKinematicTarget()						const	
												{
													const PxU32 ktFlags(PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES | PxRigidBodyFlag::eKINEMATIC);
													return (getFlagsFast()&ktFlags) == ktFlags;
												}

						void					createSqBounds();
						void					destroySqBounds();
						void					freezeTransforms(PxBitMapPinned* shapeChangedMap);

						void					addToSpeculativeCCDMap();
						void					removeFromSpeculativeCCDMap();
	private:
		// Base body
						PxsRigidBody			mLLBody;

		// External velocity changes
		// VelocityMod data allocated on the fly when the user applies velocity changes
		// which need to be accumulated.
		// VelMod dirty flags stored in BodySim so we can save ourselves the expense of looking at 
		// the separate velmod data if no forces have been set.
						//PxU16					mInternalFlags;
						SimStateData*			mSimStateData;
						PxU8					mVelModState;

		// Articulation
						ArticulationSim*		mArticulation;				// NULL if not in an articulation

		// Joints & joint groups

						bool					setupSimStateData(bool isKinematic);
						void					tearDownSimStateData(bool isKinematic);

						void					raiseVelocityModFlagAndNotify(VelocityModFlags flag);
		PX_FORCE_INLINE	void					notifyDirtySpatialAcceleration()	{ raiseVelocityModFlagAndNotify(VMF_ACC_DIRTY);	}
		PX_FORCE_INLINE	void					notifyDirtySpatialVelocity()		{ raiseVelocityModFlagAndNotify(VMF_VEL_DIRTY);	}

		PX_FORCE_INLINE void					initKinematicStateBase(BodyCore&, bool asPartOfCreation);

						void					notifyWakeUp();					// inform the sleep island generation system that the object got woken up
						void					notifyPutToSleep();				// inform the sleep island generation system that the object was put to sleep
						void					internalWakeUpBase(PxReal wakeCounterValue);

		PX_FORCE_INLINE void					raiseVelocityModFlag(VelocityModFlags f)	{ mVelModState |= f;	}
		PX_FORCE_INLINE void					clearVelocityModFlag(VelocityModFlags f)	{ mVelModState &= ~f;	}
		PX_FORCE_INLINE void					setForcesToDefaults(bool enableGravity);
	};

#if PX_VC 
     #pragma warning(pop) 
#endif

} // namespace Sc

PX_FORCE_INLINE void Sc::BodySim::setForcesToDefaults(bool enableGravity)
{
	if (!(mLLBody.mCore->mFlags & PxRigidBodyFlag::eRETAIN_ACCELERATIONS))
	{
		SimStateData* simStateData = getSimStateData(false);
		if(simStateData) 
		{
			VelocityMod* velmod = simStateData->getVelocityModData();
			velmod->clear();
		}

		if (enableGravity)
			mVelModState = VMF_GRAVITY_DIRTY;	// We want to keep the gravity flag to make sure the acceleration gets changed to gravity-only
												// in the next step (unless the application adds new forces of course)
		else
			mVelModState = 0;
	}
	else
	{
		SimStateData* simStateData = getSimStateData(false);
		if (simStateData)
		{
			VelocityMod* velmod = simStateData->getVelocityModData();
			velmod->clearPerStep();
		}

		mVelModState &= (~(VMF_VEL_DIRTY));
	}
}

PX_FORCE_INLINE bool Sc::BodySim::checkSleepReadinessBesidesWakeCounter()
{
	const BodyCore& bodyCore = getBodyCore();
	const SimStateData* simStateData = getSimStateData(false);
	const VelocityMod* velmod = simStateData ? simStateData->getVelocityModData() : NULL;

	bool readyForSleep = bodyCore.getLinearVelocity().isZero() && bodyCore.getAngularVelocity().isZero();
	if (readVelocityModFlag(VMF_ACC_DIRTY))
	{
		readyForSleep = readyForSleep && (!velmod || velmod->getLinearVelModPerSec().isZero());
		readyForSleep = readyForSleep && (!velmod || velmod->getAngularVelModPerSec().isZero());
	}
	if (readVelocityModFlag(VMF_VEL_DIRTY))
	{
		readyForSleep = readyForSleep && (!velmod || velmod->getLinearVelModPerStep().isZero());
		readyForSleep = readyForSleep && (!velmod || velmod->getAngularVelModPerStep().isZero());
	}

	return readyForSleep;
}


}

#endif
