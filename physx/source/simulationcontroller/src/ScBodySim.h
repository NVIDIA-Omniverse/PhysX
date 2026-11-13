// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_BODYSIM_H
#define SC_BODYSIM_H

#include "foundation/PxUtilities.h"
#include "foundation/PxIntrinsics.h"
#include "CmPinnableArray.h"
#include "CmPinnableBitMap.h"
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
	struct UpdateCachedParams;

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

						void					updateCached(const UpdateCachedParams& params, Cm::PinnableBitMap* shapeChangedMap, bool fromTask, bool useAtomics);
						void					updateCached_NotThreadSafe(const UpdateCachedParams& params, Cm::PinnableBitMap* shapeChangedMap, bool fromTask, bool useAtomics);
						void					updateCached_ThreadSafe(const UpdateCachedParams& params);
						bool					updateCachedIfChanged(const UpdateCachedParams& params, Cm::PinnableBitMap* shapeChangedMap);	// see ShapeSimBase::updateCachedIfChanged()

		// Software prefetch for the deactivation loops in Scene. Each function is issued fewer bodies ahead than the
		// previous one, in this order, since it dereferences what the previous one fetched. Loops that don't read the
		// interactions call prefetchCoreAndShapeArray() instead of prefetchCoreAndArrays().
		PX_FORCE_INLINE	void					prefetchSelf()					const;	// this object
		PX_FORCE_INLINE	void					prefetchCoreAndArrays()			const;	// body core, interaction and shape arrays
		PX_FORCE_INLINE	void					prefetchCoreAndShapeArray()		const;	// body core and shape array
		PX_FORCE_INLINE	void					prefetchInteractions()			const;
		PX_FORCE_INLINE	void					prefetchShapes()				const;
						void					prefetchShapesForUpdateCached(const UpdateCachedParams& params, bool bounds)	const;	// what updateCached() reads and writes per shape, the bounds entry optionally
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
						bool					checkSleepReadinessBesidesWakeCounter();  // for API triggered changes to test sleep readiness

		// PT: TODO: this is only used for the rigid bodies' sleep check, the implementations in derived classes look useless
		virtual			void					registerCountedInteraction()		PX_OVERRIDE	{ mLLBody.getCore().numCountedInteractions++; PX_ASSERT(mLLBody.getCore().numCountedInteractions);	}
		virtual			void					unregisterCountedInteraction()		PX_OVERRIDE	{ PX_ASSERT(mLLBody.getCore().numCountedInteractions); mLLBody.getCore().numCountedInteractions--;	}
		// PT: TODO: this is only used for the rigid bodies' sleep check called from the articulation sim code
		virtual			PxU32					getNumCountedInteractions()	const	PX_OVERRIDE	{ return mLLBody.getCore().numCountedInteractions;													}

		PX_FORCE_INLINE PxIntBool				isFrozen()					const	{ return PxIntBool(mLLBody.mInternalFlags & PxsRigidBody::eFROZEN);									}

		// External velocity changes - returns true if any forces were applied to this body
						bool					updateForces(PxReal dt, PxsRigidBody** updatedBodySims, PxU32* updatedBodyNodeIndices, PxU32& index, Cm::SpatialVector* acceleration, 
													PxsExternalAccelerationProvider* externalAccelerations = NULL, PxU32 maxNumExternalAccelerations = 0);

		// PT: TODO: PxIntBool
		PX_FORCE_INLINE bool					readVelocityModFlag(VelocityModFlags f) { return (mLLBody.mInternalFlags & f) != 0; }

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

		// Deactivation split for the parallel sleep path in ScSleep.cpp. deactivateThreadSafe() does the part that writes
		// only to this body: it takes the body out of the active list index-wise, returning the index it had, and zeroes
		// velocities and forces. It returns the work that touches shared scene data as DeactivationWork bits, which the
		// caller passes to finishDeactivation() from a single thread. Together the two do what setActive(false) does.
		enum DeactivationWork
		{
			eDEACTIVATE_INTERACTIONS	= 1 << 0,	// constraint, trigger or articulation joint interactions to deactivate
			eSLEEP_NOTIFICATION			= 1 << 1,	// the scene reports sleep events and this body needs one
			ePOSE_PREVIEW				= 1 << 2,	// the body is in the pose preview list
			eSQ_BOUNDS					= 1 << 3,	// at least one shape has scene query bounds to destroy
			eSPECULATIVE_CCD			= 1 << 4,	// the body is in the speculative CCD bitmap
			eCOMPOUND					= 1 << 5	// the body is in the active compound body list
		};
						PxU32					deactivateThreadSafe(PxU32& activeListIndex);
						void					finishDeactivation(PxU32 work);

						void					freezeTransforms(const UpdateCachedParams& params, Cm::PinnableBitMap* shapeChangedMap);
						void					freezeTransforms(PxsTransformCache& transformCache);

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

		PX_FORCE_INLINE void					raiseVelocityModFlag(VelocityModFlags f)	{ mLLBody.mInternalFlags |= f;	}
		PX_FORCE_INLINE void					clearVelocityModFlag(VelocityModFlags f)	{ mLLBody.mInternalFlags &= ~f;	}
		PX_FORCE_INLINE void					setForcesToDefaults(bool enableGravity);
	};

#if PX_VC 
     #pragma warning(pop) 
#endif

} // namespace Sc

PX_FORCE_INLINE void Sc::BodySim::prefetchSelf() const
{
	PxPrefetchLine(&mShapes);
	PxPrefetchLine(&mActiveListIndex);
	PxPrefetchLine(&mLLBody.mInternalFlags);
	PxPrefetchLine(&mArticulation);
}

PX_FORCE_INLINE void Sc::BodySim::prefetchCoreAndShapeArray() const
{
	const PxsBodyCore& core = *mLLBody.mCore;
	PxPrefetchLine(&core.body2World);
	PxPrefetchLine(&core.linearVelocity);
	PxPrefetchLine(&core.mFlags);

	// A single shape is stored in this object.
	if(getNbElements() > 1)
		PxPrefetchLine(getElements());
}

PX_FORCE_INLINE void Sc::BodySim::prefetchCoreAndArrays() const
{
	prefetchCoreAndShapeArray();

	// The interaction array starts in this object and moves out for good once it outgrows INLINE_INTERACTION_CAPACITY,
	// so test where it is rather than its size.
	const PxU32 nbInteractions = getActorInteractionCount();
	Interaction*const* interactions = getActorInteractions();
	if(nbInteractions && interactions != mInlineInteractionMem)
	{
		PxPrefetchLine(interactions);
		if(nbInteractions > 8)
			PxPrefetchLine(interactions, 64);
	}
}

PX_FORCE_INLINE void Sc::BodySim::prefetchInteractions() const
{
	const PxU32 nbInteractions = getActorInteractionCount();
	Interaction*const* interactions = getActorInteractions();
	for(PxU32 i=0; i<nbInteractions; i++)
		PxPrefetchLine(interactions[i]);
}

PX_FORCE_INLINE void Sc::BodySim::prefetchShapes() const
{
	const PxU32 nbShapes = getNbElements();
	ElementSim*const* shapes = getElements();
	for(PxU32 i=0; i<nbShapes; i++)
		PxPrefetchLine(shapes[i]);
}

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
			raiseVelocityModFlag(VMF_GRAVITY_DIRTY);	// We want to keep the gravity flag to make sure the acceleration gets changed to gravity-only
														// in the next step (unless the application adds new forces of course)
		else
			clearVelocityModFlag(VMF_GRAVITY_DIRTY);

		clearVelocityModFlag(VMF_ACC_DIRTY);
		clearVelocityModFlag(VMF_VEL_DIRTY);
	}
	else
	{
		SimStateData* simStateData = getSimStateData(false);
		if (simStateData)
		{
			VelocityMod* velmod = simStateData->getVelocityModData();
			velmod->clearPerStep();
		}

		clearVelocityModFlag(VMF_VEL_DIRTY);
	}
}



}

#endif
