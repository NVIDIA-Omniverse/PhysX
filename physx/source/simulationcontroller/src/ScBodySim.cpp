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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "ScBodySim.h"
#include "ScShapeSim.h"
#include "ScScene.h"
#include "ScArticulationSim.h"
#include "PxsContext.h"
#include "PxsSimpleIslandManager.h"
#include "PxsSimulationController.h"

using namespace physx;
using namespace physx::Dy;
using namespace Sc;

#define PX_FREEZE_INTERVAL 1.5f
#define PX_FREE_EXIT_THRESHOLD 4.f
#define PX_FREEZE_TOLERANCE 0.25f

#define PX_SLEEP_DAMPING	0.5f
#define PX_FREEZE_SCALE		0.9f

BodySim::BodySim(Scene& scene, BodyCore& core, bool compound) :
	RigidSim				(scene, core),
	mLLBody					(&core.getCore(), PX_FREEZE_INTERVAL),
	mSimStateData			(NULL),
	mVelModState			(VMF_GRAVITY_DIRTY),
	mArticulation			(NULL),
	mConstraintGroup		(NULL)
{
	core.getCore().numCountedInteractions = 0;
	core.getCore().disableGravity = core.getActorFlags() & PxActorFlag::eDISABLE_GRAVITY;
	if(core.getFlags() & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
		mLLBody.mInternalFlags |= PxsRigidBody::eSPECULATIVE_CCD;

	if (core.getFlags() & PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES)
		mLLBody.mInternalFlags |= PxsRigidBody::eENABLE_GYROSCROPIC;

	if (core.getFlags() & PxRigidBodyFlag::eRETAIN_ACCELERATIONS)
		mLLBody.mInternalFlags |= PxsRigidBody::eRETAIN_ACCELERATION;

	// PT: don't read the core ptr we just wrote, use input param
	// PT: at time of writing we get a big L2 here because even though bodycore has been prefetched, the wake counter is 160 bytes away
	const bool isAwake =	(core.getWakeCounter() > 0) || 
							(!core.getLinearVelocity().isZero()) ||
							(!core.getAngularVelocity().isZero());

	const bool isKine = isKinematic();

	IG::SimpleIslandManager* simpleIslandManager = scene.getSimpleIslandManager();
	if (!isArticulationLink())
	{
		mNodeIndex = simpleIslandManager->addRigidBody(&mLLBody, isKine, isAwake);
	}
	else
	{
		if(mArticulation)
		{
			PxU32 linkIndex = mArticulation->findBodyIndex(*this);
			PxNodeIndex index = mArticulation->getIslandNodeIndex();
			mNodeIndex.setIndices(index.index(), linkIndex);
		}
	}

	PX_ASSERT(mActiveListIndex == SC_NOT_IN_SCENE_INDEX);

	// A.B. need to set the compound rigid flag early enough, so that we add the rigid into 
	// active list and do not create the shape bounds
	if(compound)
		raiseInternalFlag(BF_IS_COMPOUND_RIGID);

	setActive(isAwake, ActorSim::AS_PART_OF_CREATION);

	if (isAwake)
	{
		scene.addToActiveList(*this);
		PX_ASSERT(isActive());
	}
	else
	{
		mActiveListIndex = SC_NOT_IN_ACTIVE_LIST_INDEX;
		mActiveCompoundListIndex = SC_NOT_IN_ACTIVE_LIST_INDEX;
		PX_ASSERT(!isActive());

		simpleIslandManager->deactivateNode(mNodeIndex);
	}

	if (isKine)
	{
		initKinematicStateBase(core, true);
		setupSimStateData(scene.getSimStateDataPool(), true);
		notifyPutToSleep();  // sleep state of kinematics is fully controlled by the simulation controller not the island manager
	}
}

BodySim::~BodySim()
{
	Scene& scene = getScene();
	const bool active = isActive();

	tearDownSimStateData(scene.getSimStateDataPool(), isKinematic() ? true : false);
	PX_ASSERT(!mSimStateData);

	PX_ASSERT(!readInternalFlag(BF_ON_DEATHROW)); // Before 3.0 it could happen that destroy could get called twice. Assert to make sure this is fixed.
	raiseInternalFlag(BF_ON_DEATHROW);

	scene.removeBody(*this);
	PX_ASSERT(!getConstraintGroup());  // Removing from scene should erase constraint group node if it existed


	//Articulations are represented by a single node, so they must only be removed by the articulation and not the links!
	if(mArticulation == NULL && mNodeIndex.articulationLinkId() == 0) //If it wasn't an articulation link, then we can remove it
		scene.getSimpleIslandManager()->removeNode(mNodeIndex);

	PX_ASSERT(mActiveListIndex != SC_NOT_IN_SCENE_INDEX);

	if (active)
		scene.removeFromActiveList(*this);

	mActiveListIndex = SC_NOT_IN_SCENE_INDEX;
	mActiveCompoundListIndex = SC_NOT_IN_SCENE_INDEX;

	mCore.setSim(NULL);
}

void BodySim::updateCached(PxBitMapPinned* shapeChangedMap)
{
	if(!(mLLBody.mInternalFlags & PxsRigidBody::eFROZEN))
	{
		PxU32 nbElems = getNbElements();
		ElementSim** elems = getElements();
		while (nbElems--)
		{
			ShapeSim* current = static_cast<ShapeSim*>(*elems++);
			current->updateCached(0, shapeChangedMap);
		}
	}
}

void BodySim::updateCached(PxsTransformCache& transformCache, Bp::BoundsArray& boundsArray)
{
	PX_ASSERT(!(mLLBody.mInternalFlags & PxsRigidBody::eFROZEN));	// PT: should not be called otherwise

	PxU32 nbElems = getNbElements();
	ElementSim** elems = getElements();
	while (nbElems--)
	{
		ShapeSim* current = static_cast<ShapeSim*>(*elems++);
		current->updateCached(transformCache, boundsArray);
	}
}

//--------------------------------------------------------------
//
// BodyCore interface implementation
//
//--------------------------------------------------------------

bool BodySim::setupSimStateData(PxPool<SimStateData>* simStateDataPool, const bool isKinematic)
{
	SimStateData* data = mSimStateData;
	if (!data)
	{
		data = simStateDataPool->construct();
		if (!data)
			return false;
	}

	if (isKinematic)
	{
		PX_ASSERT(!mSimStateData || !mSimStateData->isKine());

		PX_PLACEMENT_NEW(data, SimStateData(SimStateData::eKine));
		Kinematic* kine = data->getKinematicData();
		kine->targetValid = 0;
		simStateBackupAndClearBodyProperties(data, getBodyCore().getCore());
	}
	else
	{
		PX_ASSERT(!mSimStateData || !mSimStateData->isVelMod());

		PX_PLACEMENT_NEW(data, SimStateData(SimStateData::eVelMod));
		VelocityMod* velmod = data->getVelocityModData();
		velmod->clear();
	}
	mSimStateData = data;
	return true;
}

void BodySim::tearDownSimStateData(PxPool<SimStateData>* simStateDataPool, const bool isKinematic)
{
	PX_ASSERT(!mSimStateData || mSimStateData->isKine() == isKinematic);

	if (mSimStateData)
	{
		if (isKinematic)
			simStateRestoreBodyProperties(mSimStateData, getBodyCore().getCore());

		simStateDataPool->destroy(mSimStateData);
		mSimStateData = NULL;
	}
}

void BodySim::switchToKinematic(PxPool<SimStateData>* simStateDataPool)
{
	setupSimStateData(simStateDataPool, true);
	postSwitchToKinematic();
	getScene().setDynamicsDirty();
}

void BodySim::switchToDynamic(PxPool<SimStateData>* simStateDataPool)
{
	tearDownSimStateData(simStateDataPool, true);
	postSwitchToDynamic();
	getScene().setDynamicsDirty();
}

void BodySim::setKinematicTarget(const PxTransform& p)
{
	PX_ASSERT(getSimStateData(true));
	PX_ASSERT(getSimStateData(true)->isKine());
	simStateSetKinematicTarget(getSimStateData_Unchecked(), p);
	PX_ASSERT(getSimStateData(true)->getKinematicData()->targetValid);

	raiseInternalFlag(BF_KINEMATIC_MOVED);	// Important to set this here already because trigger interactions need to have this information when being activated.
	clearInternalFlag(BF_KINEMATIC_SURFACE_VELOCITY);
}

void BodySim::addSpatialAcceleration(PxPool<SimStateData>* simStateDataPool, const PxVec3* linAcc, const PxVec3* angAcc)
{
	notifyAddSpatialAcceleration();

	if (!mSimStateData || !mSimStateData->isVelMod())
		setupSimStateData(simStateDataPool, false);

	VelocityMod* velmod = mSimStateData->getVelocityModData();
	if (linAcc) velmod->accumulateLinearVelModPerSec(*linAcc);
	if (angAcc) velmod->accumulateAngularVelModPerSec(*angAcc);
}

void BodySim::setSpatialAcceleration(PxPool<SimStateData>* simStateDataPool, const PxVec3* linAcc, const PxVec3* angAcc)
{
	notifyAddSpatialAcceleration();

	if (!mSimStateData || !mSimStateData->isVelMod())
		setupSimStateData(simStateDataPool, false);

	VelocityMod* velmod = mSimStateData->getVelocityModData();
	if (linAcc) velmod->setLinearVelModPerSec(*linAcc);
	if (angAcc) velmod->setAngularVelModPerSec(*angAcc);
}

void BodySim::clearSpatialAcceleration(bool force, bool torque)
{
	PX_ASSERT(force || torque);

	notifyClearSpatialAcceleration();

	if (mSimStateData)
	{
		PX_ASSERT(mSimStateData->isVelMod());
		VelocityMod* velmod = mSimStateData->getVelocityModData();
		if (force)
			velmod->clearLinearVelModPerSec();
		if (torque)
			velmod->clearAngularVelModPerSec();
	}
}

void BodySim::addSpatialVelocity(PxPool<SimStateData>* simStateDataPool, const PxVec3* linVelDelta, const PxVec3* angVelDelta)
{
	notifyAddSpatialVelocity();

	if (!mSimStateData || !mSimStateData->isVelMod())
		setupSimStateData(simStateDataPool, false);

	VelocityMod* velmod = mSimStateData->getVelocityModData();
	if (linVelDelta)
		velmod->accumulateLinearVelModPerStep(*linVelDelta);
	if (angVelDelta)
		velmod->accumulateAngularVelModPerStep(*angVelDelta);
}

void BodySim::clearSpatialVelocity(bool force, bool torque)
{
	PX_ASSERT(force || torque);

	notifyClearSpatialVelocity();

	if (mSimStateData)
	{
		PX_ASSERT(mSimStateData->isVelMod());
		VelocityMod* velmod = mSimStateData->getVelocityModData();
		if (force)
			velmod->clearLinearVelModPerStep();
		if (torque)
			velmod->clearAngularVelModPerStep();
	}
}

void BodySim::raiseVelocityModFlagAndNotify(VelocityModFlags flag)
{
	//The dirty flag is stored separately in the BodySim so that we query the dirty flag before going to 
	//the expense of querying the simStateData for the velmod values.
	raiseVelocityModFlag(flag);

	if (!isArticulationLink())
		getScene().getVelocityModifyMap().growAndSet(getNodeIndex().index());
	else
		getScene().addDirtyArticulationSim(getArticulation());
}

void BodySim::postActorFlagChange(PxU32 oldFlags, PxU32 newFlags)
{
	// PT: don't convert to bool if not needed
	const PxU32 wasWeightless = oldFlags & PxActorFlag::eDISABLE_GRAVITY;
	const PxU32 isWeightless = newFlags & PxActorFlag::eDISABLE_GRAVITY;

	if (isWeightless != wasWeightless)
	{
		if (mVelModState == 0)
			raiseVelocityModFlag(VMF_GRAVITY_DIRTY);

		getBodyCore().getCore().disableGravity = isWeightless!=0;
	}
}

void BodySim::postBody2WorldChange()
{
	mLLBody.saveLastCCDTransform();
	notifyShapesOfTransformChange();
}

void BodySim::postSetWakeCounter(PxReal t, bool forceWakeUp)
{
	if ((t > 0.0f) || forceWakeUp)
		notifyNotReadyForSleeping();
	else
	{
		const bool readyForSleep = checkSleepReadinessBesidesWakeCounter();
		if (readyForSleep)
			notifyReadyForSleeping();
	}
}

static void updateBPGroup(ActorSim* sim)
{
	PxU32 nbElems = sim->getNbElements();
	ElementSim** elems = sim->getElements();
	while (nbElems--)
	{
		ShapeSim* current = static_cast<ShapeSim*>(*elems++);
		current->updateBPGroup();
	}
}

void BodySim::postSwitchToKinematic()
{
	initKinematicStateBase(getBodyCore(), false);

	// - interactions need to get refiltered to make sure that kinematic-kinematic and kinematic-static pairs get suppressed
	// - unlike postSwitchToDynamic(), constraint interactions are not marked dirty here because a transition to kinematic will put the object asleep which in turn 
	//   triggers onDeactivate_() on the constraint pairs that are active. If such pairs get deactivated, they will get removed from the list of active breakable
	//   constraints automatically.
	setActorsInteractionsDirty(InteractionDirtyFlag::eBODY_KINEMATIC, NULL, InteractionFlag::eFILTERABLE);

	getScene().getSimpleIslandManager()->setKinematic(mNodeIndex);

	updateBPGroup(this);
}

void BodySim::postSwitchToDynamic()
{
	mScene.getSimpleIslandManager()->setDynamic(mNodeIndex);

	setForcesToDefaults(true);

	if(getConstraintGroup())
		getConstraintGroup()->markForProjectionTreeRebuild(mScene.getProjectionManager());

	// - interactions need to get refiltered to make sure that former kinematic-kinematic and kinematic-static pairs get enabled
	// - switching from kinematic to dynamic does not change the sleep state of the body. The constraint interactions are marked dirty
	//   to check later whether they need to be activated plus potentially tracked for constraint break testing. This special treatment
	//   is necessary because constraints between two kinematic bodies are considered inactive, no matter whether one of the kinematics
	//   is active (has a target) or not.
	setActorsInteractionsDirty(InteractionDirtyFlag::eBODY_KINEMATIC, NULL, InteractionFlag::eFILTERABLE | InteractionFlag::eCONSTRAINT);

	clearInternalFlag(BF_KINEMATIC_MOVE_FLAGS);

	if(isActive())
		mScene.swapInActiveBodyList(*this);

	//
	updateBPGroup(this);
}

void BodySim::postPosePreviewChange(const PxU32 posePreviewFlag)
{
	if (isActive())
	{
		if (posePreviewFlag & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)
			getScene().addToPosePreviewList(*this);
		else
			getScene().removeFromPosePreviewList(*this);
	}
	else
		PX_ASSERT(!getScene().isInPosePreviewList(*this));
}

//--------------------------------------------------------------
//
// Sleeping
//
//--------------------------------------------------------------

void BodySim::activate()
{
	// Activate body
	{
		PX_ASSERT((!isKinematic()) || notInScene() || readInternalFlag(InternalFlags(BF_KINEMATIC_MOVED | BF_KINEMATIC_SURFACE_VELOCITY)));	// kinematics should only get activated when a target is set.
																								// exception: object gets newly added, then the state change will happen later
		if(!isArticulationLink())
		{
			mLLBody.mInternalFlags &= (~PxsRigidBody::eFROZEN);
			// Put in list of activated bodies. The list gets cleared at the end of a sim step after the sleep callbacks have been fired.
			getScene().onBodyWakeUp(this);
		}

		BodyCore& core = getBodyCore();
		if(core.getFlags() & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)
		{
			PX_ASSERT(!getScene().isInPosePreviewList(*this));
			getScene().addToPosePreviewList(*this);
		}
		createSqBounds();
	}

	activateInteractions(*this);

	//set speculative CCD bit map if speculative CCD flag is on
	{
		BodyCore& core = getBodyCore();
		if (core.getFlags() & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
		{
			if (isArticulationLink())
			{
				if (getNodeIndex().isValid())
					getScene().setSpeculativeCCDArticulationLink(getNodeIndex().index());
			}
			else
				getScene().setSpeculativeCCDRigidBody(getNodeIndex().index());
		}
	}
}

void BodySim::deactivate()
{
	deactivateInteractions(*this);

	// Deactivate body
	{
		PX_ASSERT((!isKinematic()) || notInScene() || !readInternalFlag(BF_KINEMATIC_MOVED));	// kinematics should only get deactivated when no target is set.
																								// exception: object gets newly added, then the state change will happen later
		BodyCore& core = getBodyCore();
		if(!readInternalFlag(BF_ON_DEATHROW))
		{
			// Set velocity to 0.
			// Note: this is also fine if the method gets called because the user puts something to sleep (this behavior is documented in the API)
			PX_ASSERT(core.getWakeCounter() == 0.0f);
			const PxVec3 zero(0.0f);
			core.setLinearVelocityInternal(zero);
			core.setAngularVelocityInternal(zero);
	
			setForcesToDefaults(!core.getCore().disableGravity);
		}

		if(!isArticulationLink())  // Articulations have their own sleep logic.
			getScene().onBodySleep(this);

		if(core.getFlags() & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)
		{
			PX_ASSERT(getScene().isInPosePreviewList(*this));
			getScene().removeFromPosePreviewList(*this);
		}
		destroySqBounds();
	}

	// reset speculative CCD bit map if speculative CCD flag is on
	{
		BodyCore& core = getBodyCore();
		if (core.getFlags() & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
		{
			if (isArticulationLink())
			{
				if (getNodeIndex().isValid())
					getScene().resetSpeculativeCCDArticulationLink(getNodeIndex().index());
			}
			else
				getScene().resetSpeculativeCCDRigidBody(getNodeIndex().index());
		}
	}
}


void BodySim::wakeUp()
{
	setActive(true);
	notifyWakeUp();
}

void BodySim::putToSleep()
{
	PX_ASSERT(getBodyCore().getWakeCounter() == 0.0f);
	PX_ASSERT(getBodyCore().getLinearVelocity().isZero());
	PX_ASSERT(getBodyCore().getAngularVelocity().isZero());

	notifyClearSpatialAcceleration();
	notifyClearSpatialVelocity();
	simStateClearVelMod(getSimStateData_Unchecked());

	setActive(false);
	notifyPutToSleep();
}

void BodySim::internalWakeUp(PxReal wakeCounterValue)
{
	if(mArticulation)
		mArticulation->internalWakeUp(wakeCounterValue);
	else
		internalWakeUpBase(wakeCounterValue);
}

void BodySim::internalWakeUpArticulationLink(PxReal wakeCounterValue)
{
	PX_ASSERT(mArticulation);
	internalWakeUpBase(wakeCounterValue);
}

void BodySim::internalWakeUpBase(PxReal wakeCounterValue)	//this one can only increase the wake counter, not decrease it, so it can't be used to put things to sleep!
{
	if ((!isKinematic()) && (getBodyCore().getWakeCounter() < wakeCounterValue))
	{
		PX_ASSERT(wakeCounterValue > 0.0f);
		getBodyCore().setWakeCounterFromSim(wakeCounterValue);

		//we need to update the gpu body sim because we reset the wake counter for the body core
		mScene.updateBodySim(*this);
		setActive(true);
		notifyWakeUp();

		if(0)	// PT: commented-out for PX-2197
			mLLBody.mInternalFlags &= (~PxsRigidBody::eFROZEN);
	}
}

void BodySim::notifyReadyForSleeping()
{
	if(mArticulation == NULL)
		getScene().getSimpleIslandManager()->deactivateNode(mNodeIndex);
}

void BodySim::notifyNotReadyForSleeping()
{
	getScene().getSimpleIslandManager()->activateNode(mNodeIndex);
}

void BodySim::notifyWakeUp()
{
	getScene().getSimpleIslandManager()->activateNode(mNodeIndex);
}

void BodySim::notifyPutToSleep()
{
	getScene().getSimpleIslandManager()->putNodeToSleep(mNodeIndex);
}

void BodySim::resetSleepFilter()
{
	mLLBody.sleepAngVelAcc = PxVec3(0.0f);
	mLLBody.sleepLinVelAcc = PxVec3(0.0f);
}

//This function will be called by CPU sleepCheck code
PxReal BodySim::updateWakeCounter(PxReal dt, PxReal energyThreshold, const Cm::SpatialVector& motionVelocity)
{
	// update the body's sleep state and 
	BodyCore& core = getBodyCore();

	const PxReal wakeCounterResetTime = ScInternalWakeCounterResetValue;

	PxReal wc = core.getWakeCounter();
	
	{
		PxVec3 bcSleepLinVelAcc = mLLBody.sleepLinVelAcc;
		PxVec3 bcSleepAngVelAcc = mLLBody.sleepAngVelAcc;

		if(wc < wakeCounterResetTime * 0.5f || wc < dt)
		{
			const PxTransform& body2World = getBody2World();

			// calculate normalized energy: kinetic energy divided by mass
			const PxVec3 t = core.getInverseInertia();
			const PxVec3 inertia(t.x > 0.0f ? 1.0f/t.x : 1.0f, t.y > 0.0f ? 1.0f/t.y : 1.0f, t.z > 0.0f ? 1.0f/t.z : 1.0f);

			PxVec3 sleepLinVelAcc =motionVelocity.linear;
			PxVec3 sleepAngVelAcc = body2World.q.rotateInv(motionVelocity.angular);

			bcSleepLinVelAcc += sleepLinVelAcc;
			bcSleepAngVelAcc += sleepAngVelAcc;

			PxReal invMass = core.getInverseMass();
			if(invMass == 0.0f)
				invMass = 1.0f;

			const PxReal angular = bcSleepAngVelAcc.multiply(bcSleepAngVelAcc).dot(inertia) * invMass;
			const PxReal linear = bcSleepLinVelAcc.magnitudeSquared();
			PxReal normalizedEnergy = 0.5f * (angular + linear);

			// scale threshold by cluster factor (more contacts => higher sleep threshold)
			const PxReal clusterFactor = PxReal(1 + getNumCountedInteractions());
			const PxReal threshold = clusterFactor*energyThreshold;
		
			if (normalizedEnergy >= threshold)
			{
				PX_ASSERT(isActive());
				resetSleepFilter();
				const float factor = threshold == 0.0f ? 2.0f : PxMin(normalizedEnergy/threshold, 2.0f);
				PxReal oldWc = wc;
				wc = factor * 0.5f * wakeCounterResetTime + dt * (clusterFactor - 1.0f);
				core.setWakeCounterFromSim(wc);
				if (oldWc == 0.0f)  // for the case where a sleeping body got activated by the system (not the user) AND got processed by the solver as well
					notifyNotReadyForSleeping();
				
				return wc;
			}
		}

		mLLBody.sleepLinVelAcc = bcSleepLinVelAcc;
		mLLBody.sleepAngVelAcc = bcSleepAngVelAcc;
	}

	wc = PxMax(wc-dt, 0.0f);
	core.setWakeCounterFromSim(wc);
	return wc;
}

//--------------------------------------------------------------
//
// Kinematics
//
//--------------------------------------------------------------

PX_FORCE_INLINE void BodySim::initKinematicStateBase(BodyCore&, bool asPartOfCreation)
{
	PX_ASSERT(!readInternalFlag(BF_KINEMATIC_MOVED));

	if (!asPartOfCreation && isActive())
		getScene().swapInActiveBodyList(*this);

	//mLLBody.setAccelerationV(Cm::SpatialVector::zero());

	// Need to be before setting setRigidBodyFlag::KINEMATIC

	if (getConstraintGroup())
		getConstraintGroup()->markForProjectionTreeRebuild(getScene().getProjectionManager());
}

void BodySim::calculateKinematicVelocity(PxReal oneOverDt)
{
	PX_ASSERT(isKinematic());
	
	/*------------------------------------------------\
	| kinematic bodies are moved directly by the user and are not influenced by external forces
	| we simply determine the distance moved since the last simulation frame and 
	| assign the appropriate delta to the velocity. This vel will be used to shove dynamic
	| objects in the solver.
	| We have to do this like so in a delayed way, because when the user sets the target pos the dt is not
	| yet known.
	\------------------------------------------------*/
	PX_ASSERT(isActive());

	BodyCore& core = getBodyCore();

	if (readInternalFlag(BF_KINEMATIC_MOVED))
	{
		clearInternalFlag(InternalFlags(BF_KINEMATIC_SETTLING | BF_KINEMATIC_SETTLING_2));
		const SimStateData* kData = getSimStateData(true);
		PX_ASSERT(kData);
		PX_ASSERT(kData->isKine());
		PX_ASSERT(kData->getKinematicData()->targetValid);
		PxVec3 linVelLL, angVelLL;
		const PxTransform targetPose = kData->getKinematicData()->targetPose;
		const PxTransform& currBody2World = getBody2World();

		//the kinematic target pose is now the target of the body (CoM) and not the actor.

		PxVec3 deltaPos = targetPose.p;
		deltaPos -= currBody2World.p;
		linVelLL = deltaPos * oneOverDt;

		PxQuat q = targetPose.q * currBody2World.q.getConjugate();

		if (q.w < 0)	//shortest angle.
			q = -q;

		PxReal angle;
 		PxVec3 axis;
		q.toRadiansAndUnitAxis(angle, axis);
		angVelLL = axis * angle * oneOverDt;

		core.getCore().linearVelocity = linVelLL;
		core.getCore().angularVelocity = angVelLL;

		// Moving a kinematic should trigger a wakeUp call on a higher level.
		PX_ASSERT(core.getWakeCounter()>0);
		PX_ASSERT(isActive());
		
	}
	else if (!readInternalFlag(BF_KINEMATIC_SURFACE_VELOCITY))
	{
		core.setLinearVelocity(PxVec3(0));
		core.setAngularVelocity(PxVec3(0));
	}
}

void BodySim::updateKinematicPose()
{
	/*------------------------------------------------\
	| kinematic bodies are moved directly by the user and are not influenced by external forces
	| we simply determine the distance moved since the last simulation frame and 
	| assign the appropriate delta to the velocity. This vel will be used to shove dynamic
	| objects in the solver.
	| We have to do this like so in a delayed way, because when the user sets the target pos the dt is not
	| yet known.
	\------------------------------------------------*/

	PX_ASSERT(isKinematic());
	PX_ASSERT(isActive());

	if (readInternalFlag(BF_KINEMATIC_MOVED))
	{
		clearInternalFlag(InternalFlags(BF_KINEMATIC_SETTLING | BF_KINEMATIC_SETTLING_2));
		const SimStateData* kData = getSimStateData(true);
		PX_ASSERT(kData);
		PX_ASSERT(kData->isKine());
		PX_ASSERT(kData->getKinematicData()->targetValid);
		
		const PxTransform targetPose = kData->getKinematicData()->targetPose;
		getBodyCore().getCore().body2World = targetPose;
	}
}

bool BodySim::deactivateKinematic()
{
	BodyCore& core = getBodyCore();
	if(readInternalFlag(BF_KINEMATIC_SETTLING_2))
	{
		clearInternalFlag(BF_KINEMATIC_SETTLING_2);
		core.setWakeCounterFromSim(0);	// For sleeping objects the wake counter must be 0. This needs to hold for kinematics too.
		notifyReadyForSleeping();
		notifyPutToSleep();
		setActive(false);
		return true;
	}
	else if (readInternalFlag(BF_KINEMATIC_SETTLING))
	{
		clearInternalFlag(BF_KINEMATIC_SETTLING);
		raiseInternalFlag(BF_KINEMATIC_SETTLING_2);
	}
	else if (!readInternalFlag(BF_KINEMATIC_SURFACE_VELOCITY))
	{
		clearInternalFlag(BF_KINEMATIC_MOVED);
		raiseInternalFlag(BF_KINEMATIC_SETTLING);
	}
	return false;
}

//--------------------------------------------------------------
//
// Miscellaneous
//
//--------------------------------------------------------------

void BodySim::updateForces(PxReal dt, PxsRigidBody** updatedBodySims, PxU32* updatedBodyNodeIndices, PxU32& index, Cm::SpatialVector* acceleration)
{
	PxVec3 linVelDt(0.0f), angVelDt(0.0f);

	const bool accDirty = readVelocityModFlag(VMF_ACC_DIRTY);
	const bool velDirty = readVelocityModFlag(VMF_VEL_DIRTY);

	SimStateData* simStateData = NULL;

	//if we change the logic like this, which means we don't need to have two seperate variables in the pxgbodysim to represent linAcc and angAcc. However, this
	//means angAcc will be always 0
	if( (accDirty || velDirty) &&  ((simStateData = getSimStateData(false)) != NULL) )
	{
		VelocityMod* velmod = simStateData->getVelocityModData();

		//we don't have support for articulation yet
		if (updatedBodySims)
		{
			updatedBodySims[index] = &getLowLevelBody();
			updatedBodyNodeIndices[index++] = getNodeIndex().index();
		}

		if(velDirty)
		{
			linVelDt = velmod->getLinearVelModPerStep();
			angVelDt = velmod->getAngularVelModPerStep();
		}
		
		if (accDirty)
		{
			linVelDt += velmod->getLinearVelModPerSec()*dt;
			angVelDt += velmod->getAngularVelModPerSec()*dt;
		}	

		if (acceleration)
		{
			const PxReal invDt = 1.f / dt;
			acceleration->linear = linVelDt * invDt;
			acceleration->angular = angVelDt * invDt;
		}
		else
		{
			getBodyCore().updateVelocities(linVelDt, angVelDt);
		}
	}

	setForcesToDefaults(readVelocityModFlag(VMF_ACC_DIRTY));
}

void BodySim::onConstraintDetach()
{
	PX_ASSERT(readInternalFlag(BF_HAS_CONSTRAINTS));

	PxU32 size = getActorInteractionCount();
	Interaction** interactions = getActorInteractions();
	unregisterCountedInteraction();

	while(size--)
	{
		const Interaction* interaction = *interactions++;
		if(interaction->getType() == InteractionType::eCONSTRAINTSHADER)
			return;
	}

	clearInternalFlag(BF_HAS_CONSTRAINTS);  // There are no other constraint interactions left
}

void BodySim::setArticulation(ArticulationSim* a, PxReal wakeCounter, bool asleep, PxU32 bodyIndex)
{
	mArticulation = a; 
	if(a)
	{
		PxNodeIndex index = mArticulation->getIslandNodeIndex();
		mNodeIndex.setIndices(index.index(), bodyIndex);
		getBodyCore().setWakeCounterFromSim(wakeCounter);

		if (getFlagsFast() & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
			getScene().setSpeculativeCCDArticulationLink(mNodeIndex.index());

		//Articulations defer registering their shapes with the nphaseContext until the IG node index is known.
		{
			ElementSim** current = getElements();
			PxU32 nbElements = getNbElements();
			while (nbElements--)
			{
				ShapeSim* sim = static_cast<ShapeSim*>(*current++);
				getScene().getLowLevelContext()->getNphaseImplementationContext()->registerShape(mNodeIndex, sim->getCore().getCore(), sim->getElementID(), sim->getActor().getPxActor());
			}
		}

		//Force node index into LL shapes
		setBodyNodeIndex(mNodeIndex);

		if (a->getCore().getArticulationFlags() & PxArticulationFlag::eDISABLE_SELF_COLLISION)
		{
			//We need to reset the group IDs for all shapes in this body...
			ElementSim** current = getElements();
			PxU32 nbElements = getNbElements();

			Bp::AABBManagerBase* aabbMgr = getScene().getAABBManager();
			
			Bp::FilterGroup::Enum rootGroup = Bp::getFilterGroup(false, a->getRootActorIndex(), false);

			while (nbElements--)
			{
				ShapeSim* sim = static_cast<ShapeSim*>(*current++);
				aabbMgr->setBPGroup(sim->getElementID(), rootGroup);
			}
		}

		if (!asleep)
		{
			setActive(true);
			notifyWakeUp();
		}
		else
		{
			notifyReadyForSleeping();
			notifyPutToSleep();
			setActive(false);
		}
	}
	else
	{
		//Setting a 1 in the articulation ID to avoid returning the node Index to the node index
		//manager
		mNodeIndex.setIndices(PX_INVALID_NODE, 1);
	}
}

void BodySim::createSqBounds()
{
	if(!isActive() || usingSqKinematicTarget() || readInternalFlag(BF_IS_COMPOUND_RIGID))
		return;

	PX_ASSERT(!isFrozen());
	
	PxU32 nbElems = getNbElements();
	ElementSim** elems = getElements();
	while (nbElems--)
	{
		ShapeSim* current = static_cast<ShapeSim*>(*elems++);
		current->createSqBounds();
	}
}

void BodySim::destroySqBounds()
{
	PxU32 nbElems = getNbElements();
	ElementSim** elems = getElements();
	while (nbElems--)
	{
		ShapeSim* current = static_cast<ShapeSim*>(*elems++);
		current->destroySqBounds();
	}
}

void BodySim::freezeTransforms(PxBitMapPinned* shapeChangedMap)
{
	PxU32 nbElems = getNbElements();
	ElementSim** elems = getElements();
	while (nbElems--)
	{
		ShapeSim* sim = static_cast<ShapeSim*>(*elems++);
		sim->updateCached(PxsTransformFlag::eFROZEN, shapeChangedMap);
		sim->destroySqBounds();
	}
}

void BodySim::disableCompound()
{
	if(isActive())
		getScene().removeFromActiveCompoundBodyList(*this);
	clearInternalFlag(BF_IS_COMPOUND_RIGID);
}

