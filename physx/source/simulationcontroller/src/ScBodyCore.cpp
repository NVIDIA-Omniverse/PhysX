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

#include "ScBodyCore.h"
#include "ScBodySim.h"
#include "ScPhysics.h"
#include "ScScene.h"
#include "PxsSimulationController.h"
#include "ScArticulationSim.h"

using namespace physx;

static void updateBodySim(Sc::BodyCore& bodyCore)
{
	Sc::BodySim* bodySim = bodyCore.getSim();
	if(bodySim)
		bodySim->getScene().updateBodySim(*bodySim);
}

Sc::BodyCore::BodyCore(PxActorType::Enum type, const PxTransform& bodyPose) : RigidCore(type)
{
	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();

	const bool isDynamic = type == PxActorType::eRIGID_DYNAMIC;
	const float linearDamping = isDynamic ? 0.0f : 0.05f;
	const float maxLinearVelocitySq = isDynamic ? 1e32f /*PX_MAX_F32*/ : 100.f * 100.f * scale.length * scale.length;
	const float maxAngularVelocitySq = isDynamic ? 100.0f * 100.0f : 50.0f * 50.0f;

	mCore.init(bodyPose, PxVec3(1.0f), 1.0f, Sc::Physics::sWakeCounterOnCreation, scale.speed, linearDamping, 0.05f, maxLinearVelocitySq, maxAngularVelocitySq,
		type);
}

Sc::BodyCore::~BodyCore()
{
	PX_ASSERT(getSim() == 0);
}

Sc::BodySim* Sc::BodyCore::getSim() const
{
	return static_cast<BodySim*>(Sc::ActorCore::getSim());
}

void Sc::BodyCore::restoreDynamicData()
{
	BodySim* sim = getSim();
	PX_ASSERT(sim);
	const SimStateData* simStateData = sim->getSimStateData(true);
	PX_ASSERT(simStateData);
	PxsBodyCore& core = getCore();
	simStateRestoreBodyProperties(simStateData, core);
}

//--------------------------------------------------------------
//
// BodyCore interface implementation
//
//--------------------------------------------------------------

void Sc::BodyCore::setBody2World(const PxTransform& p)
{
	mCore.body2World = p;
	PX_ASSERT(p.p.isFinite());
	PX_ASSERT(p.q.isFinite());

	BodySim* sim = getSim();
	if(sim)
	{
		sim->postBody2WorldChange();
		sim->getScene().updateBodySim(*sim);
	}
}

void Sc::BodyCore::setCMassLocalPose(const PxTransform& newBody2Actor)
{
	const PxTransform oldActor2World = mCore.body2World * mCore.getBody2Actor().getInverse();
	const PxTransform newBody2World = oldActor2World * newBody2Actor;

	PX_ASSERT(newBody2World.p.isFinite());
	PX_ASSERT(newBody2World.q.isFinite());
	mCore.body2World = newBody2World;

	setBody2Actor(newBody2Actor);
}

void Sc::BodyCore::setLinearVelocity(const PxVec3& v, bool skipBodySimUpdate)
{
	mCore.linearVelocity = v;

	PX_ASSERT(!skipBodySimUpdate || (getFlags() & PxRigidBodyFlag::eKINEMATIC));

	if(!skipBodySimUpdate)
		updateBodySim(*this);
}

void Sc::BodyCore::setAngularVelocity(const PxVec3& v, bool skipBodySimUpdate)
{
	mCore.angularVelocity = v;

	PX_ASSERT(!skipBodySimUpdate || (getFlags() & PxRigidBodyFlag::eKINEMATIC));

	if(!skipBodySimUpdate)
		updateBodySim(*this);
}

void Sc::BodyCore::setCfmScale(PxReal cfmScale)
{
	mCore.cfmScale = cfmScale;

	updateBodySim(*this);
}

void Sc::BodyCore::setBody2Actor(const PxTransform& p)
{
	PX_ASSERT(p.p.isFinite());
	PX_ASSERT(p.q.isFinite());

	mCore.setBody2Actor(p);

	updateBodySim(*this);
}

void Sc::BodyCore::addSpatialAcceleration(const PxVec3* linAcc, const PxVec3* angAcc)
{
	BodySim* sim = getSim();
	PX_ASSERT(sim);
	sim->addSpatialAcceleration(linAcc, angAcc);
}

void Sc::BodyCore::setSpatialAcceleration(const PxVec3* linAcc, const PxVec3* angAcc)
{
	BodySim* sim = getSim();
	PX_ASSERT(sim);
	sim->setSpatialAcceleration(linAcc, angAcc);
}

void Sc::BodyCore::clearSpatialAcceleration(bool force, bool torque)
{
	PX_ASSERT(force || torque);
	BodySim* sim = getSim();
	PX_ASSERT(sim);
	sim->clearSpatialAcceleration(force, torque);
}

void Sc::BodyCore::addSpatialVelocity(const PxVec3* linVelDelta, const PxVec3* angVelDelta)
{
	BodySim* sim = getSim();
	PX_ASSERT(sim);
	sim->addSpatialVelocity(linVelDelta, angVelDelta);
}

void Sc::BodyCore::clearSpatialVelocity(bool force, bool torque)
{
	PX_ASSERT(force || torque);
	BodySim* sim = getSim();
	PX_ASSERT(sim);
	sim->clearSpatialVelocity(force, torque);
}

PxReal Sc::BodyCore::getInverseMass() const
{
	BodySim* sim = getSim();
	if(!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		return mCore.inverseMass;
	}
	else
	{
		const SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		return simStateData->getKinematicData()->backupInvMass;
	}
}

void Sc::BodyCore::setInverseMass(PxReal m)
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		mCore.inverseMass = m;
		updateBodySim(*this);
	}
	else
	{
		SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		simStateData->getKinematicData()->backupInvMass = m;
	}
}

const PxVec3& Sc::BodyCore::getInverseInertia() const
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		return mCore.inverseInertia;
	}
	else
	{
		const SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		return (simStateData->getKinematicData()->backupInverseInertia);
	}
}

void Sc::BodyCore::setInverseInertia(const PxVec3& i)
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		mCore.inverseInertia = i;
		updateBodySim(*this);
	}
	else
	{
		SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		simStateData->getKinematicData()->backupInverseInertia = i;
	}
}

PxReal Sc::BodyCore::getLinearDamping() const
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		return mCore.linearDamping;
	}
	else
	{
		const SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		return (simStateData->getKinematicData()->backupLinearDamping);
	}
}

void Sc::BodyCore::setLinearDamping(PxReal d)
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		mCore.linearDamping = d;
		updateBodySim(*this);
	}
	else
	{
		SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		simStateData->getKinematicData()->backupLinearDamping = d;
	}
}

PxReal Sc::BodyCore::getAngularDamping() const
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		return mCore.angularDamping;
	}
	else
	{
		const SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		return (simStateData->getKinematicData()->backupAngularDamping);
	}
}

void Sc::BodyCore::setAngularDamping(PxReal v)
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		mCore.angularDamping = v;
		updateBodySim(*this);
	}
	else
	{
		SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		simStateData->getKinematicData()->backupAngularDamping = v;
	}
}

PxReal Sc::BodyCore::getMaxAngVelSq() const
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		return mCore.maxAngularVelocitySq;
	}
	else
	{
		const SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		return (simStateData->getKinematicData()->backupMaxAngVelSq);
	}
}

void Sc::BodyCore::setMaxAngVelSq(PxReal v)
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		mCore.maxAngularVelocitySq = v;
		updateBodySim(*this);
	}
	else
	{
		SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		simStateData->getKinematicData()->backupMaxAngVelSq = v;
	}
}

PxReal Sc::BodyCore::getMaxLinVelSq() const
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		return mCore.maxLinearVelocitySq;
	}
	else
	{
		const SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		return (simStateData->getKinematicData()->backupMaxLinVelSq);
	}
}

void Sc::BodyCore::setMaxLinVelSq(PxReal v)
{
	BodySim* sim = getSim();
	if (!sim || (!(getFlags() & PxRigidBodyFlag::eKINEMATIC)))
	{
		mCore.maxLinearVelocitySq = v;
		updateBodySim(*this);
	}
	else
	{
		SimStateData* simStateData = sim->getSimStateData(true);
		PX_ASSERT(simStateData);
		PX_ASSERT(simStateData->getKinematicData());
		simStateData->getKinematicData()->backupMaxLinVelSq = v;
	}
}

void Sc::BodyCore::setFlags(PxRigidBodyFlags f)
{
	const PxRigidBodyFlags old = mCore.mFlags;
	if(f != old)
	{
		const PxU32 wasKinematic = old & PxRigidBodyFlag::eKINEMATIC;
		const PxU32 isKinematic = f & PxRigidBodyFlag::eKINEMATIC;
		const bool switchToKinematic = ((!wasKinematic) && isKinematic);
		const bool switchToDynamic = (wasKinematic && (!isKinematic));

		mCore.mFlags = f;
		BodySim* sim = getSim();
		if (sim)
		{
			const PxU32 posePreviewFlag = f & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW;
			if(PxU32(old & PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW) != posePreviewFlag)
				sim->postPosePreviewChange(posePreviewFlag);

			// for those who might wonder about the complexity here:
			// our current behavior is that you are not allowed to set a kinematic target unless the object is in a scene.
			// Thus, the kinematic data should only be created/destroyed when we know for sure that we are in a scene.

			if(switchToKinematic)
				sim->switchToKinematic();
			else if(switchToDynamic)
				sim->switchToDynamic();

			const PxU32 wasSpeculativeCCD = old & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD;
			const PxU32 isSpeculativeCCD = f & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD;

			if(wasSpeculativeCCD ^ isSpeculativeCCD)
			{
				if(wasSpeculativeCCD)
				{
					sim->removeFromSpeculativeCCDMap();

					sim->getLowLevelBody().mInternalFlags &= (~PxsRigidBody::eSPECULATIVE_CCD);
				}
				else
				{
					//Kinematic body switch puts the body to sleep, so we do not mark the speculative CCD bitmap for this actor to true in this case.
					if(!switchToKinematic)
						sim->addToSpeculativeCCDMap();

					sim->getLowLevelBody().mInternalFlags |= (PxsRigidBody::eSPECULATIVE_CCD);
				}
			}

			const PxU32 wasIntegrateGyroscopic = old & PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES;
			const PxU32 isIntegrateGyroscopic = f & PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES;
			if (wasIntegrateGyroscopic ^ isIntegrateGyroscopic)
			{
				if(wasIntegrateGyroscopic)
					sim->getLowLevelBody().mInternalFlags &= (PxsRigidBody::eENABLE_GYROSCOPIC);
				else
					sim->getLowLevelBody().mInternalFlags |= (PxsRigidBody::eENABLE_GYROSCOPIC);				
			}

			const PxU32 wasRetainAccel = old & PxRigidBodyFlag::eRETAIN_ACCELERATIONS;
			const PxU32 isRetainAccel = f & PxRigidBodyFlag::eRETAIN_ACCELERATIONS;

			if (wasRetainAccel ^ isRetainAccel)
			{
				if (wasRetainAccel)
					sim->getLowLevelBody().mInternalFlags &= (PxsRigidBody::eRETAIN_ACCELERATION);
				else
					sim->getLowLevelBody().mInternalFlags |= (PxsRigidBody::eRETAIN_ACCELERATION);
			}

			//Force flag change through...
			sim->getScene().updateBodySim(*sim);
			
		}

		if(switchToKinematic)
			putToSleep();

		if(sim)
		{
			const PxRigidBodyFlags ktFlags(PxRigidBodyFlag::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES | PxRigidBodyFlag::eKINEMATIC);
			const bool hadKt = (old & ktFlags) == ktFlags;
			const bool hasKt = (f & ktFlags) == ktFlags;
			if(hasKt && !hadKt)
				sim->destroySqBounds();
			else if(hadKt && !hasKt)
				sim->createSqBounds();
		}
	}
}

void Sc::BodyCore::setMaxContactImpulse(PxReal m)	
{ 
	mCore.maxContactImpulse = m; 
	updateBodySim(*this);
}

void Sc::BodyCore::setOffsetSlop(PxReal slop)
{
	mCore.offsetSlop = slop;
	updateBodySim(*this);
}

PxNodeIndex Sc::BodyCore::getInternalIslandNodeIndex() const
{
	BodySim* sim = getSim();
	return sim ? sim->getNodeIndex() : PxNodeIndex(PX_INVALID_NODE);
}

void Sc::BodyCore::setWakeCounter(PxReal wakeCounter, bool forceWakeUp)
{
	mCore.wakeCounter = wakeCounter;
	BodySim* sim = getSim();
	if(sim)
	{
		//wake counter change, we need to trigger dma pxgbodysim data again
		sim->getScene().updateBodySim(*sim);
		if ((wakeCounter > 0.0f) || forceWakeUp)
			sim->wakeUp();
		sim->postSetWakeCounter(wakeCounter, forceWakeUp);
	}
}

void Sc::BodyCore::setSleepThreshold(PxReal t)
{
	mCore.sleepThreshold = t;
	updateBodySim(*this);
}

void Sc::BodyCore::setFreezeThreshold(PxReal t)
{
	mCore.freezeThreshold = t;
	updateBodySim(*this);
}

bool Sc::BodyCore::isSleeping() const
{
	BodySim* sim = getSim();
	return sim ? !sim->isActive() : true;
}

void Sc::BodyCore::putToSleep()
{
	mCore.linearVelocity = PxVec3(0.0f);
	mCore.angularVelocity = PxVec3(0.0f);

	// important to clear all values before setting the wake counter because the values decide
	// whether an object is ready to go to sleep or not.
	setWakeCounter(0.0f);

	BodySim* sim = getSim();
	if(sim)
		sim->putToSleep();
}

void Sc::BodyCore::onOriginShift(const PxVec3& shift)
{
	mCore.body2World.p -= shift;

	BodySim* b = getSim();
	if(b)
		b->onOriginShift(shift, getFlags() & PxRigidBodyFlag::eKINEMATIC);  // BodySim might not exist if actor has simulation disabled (PxActorFlag::eDISABLE_SIMULATION)
}

// PT: TODO: why do we test againt NULL everywhere but not in 'isFrozen' ?
PxIntBool Sc::BodyCore::isFrozen() const
{
	return getSim()->isFrozen();
}

void Sc::BodyCore::setSolverIterationCounts(PxU16 c)	
{ 
	mCore.solverIterationCounts = c;	
	Sc::BodySim* sim = getSim();
	if (sim)
	{
		sim->getLowLevelBody().solverIterationCounts = c;
		sim->getScene().setDynamicsDirty();
	}
}

///////////////////////////////////////////////////////////////////////////////

bool Sc::BodyCore::getKinematicTarget(PxTransform& p) const
{
	PX_ASSERT(mCore.mFlags & PxRigidBodyFlag::eKINEMATIC);
	const BodySim* sim = getSim();
	return (sim && simStateGetKinematicTarget(sim->getSimStateData_Unchecked(), p));
}

bool Sc::BodyCore::getHasValidKinematicTarget() const
{
	//The use pattern for this is that we should only look for kinematic data if we know it is kinematic.
	//We might look for velmod data even if it is kinematic.
	BodySim* sim = getSim();
	return (sim && simStateGetHasValidKinematicTarget(sim->getSimStateData_Unchecked()));
}

void Sc::BodyCore::setKinematicTarget(const PxTransform& p, PxReal wakeCounter)
{
	PX_ASSERT(mCore.mFlags & PxRigidBodyFlag::eKINEMATIC);
	Sc::BodySim* sim = getSim();
	PX_ASSERT(sim);
	sim->setKinematicTarget(p);
	wakeUp(wakeCounter);
}

void Sc::BodyCore::invalidateKinematicTarget()
{ 
	Sc::BodySim* sim = getSim();
	PX_ASSERT(sim);
	simStateInvalidateKinematicTarget(sim->getSimStateData_Unchecked());
}

void Sc::BodyCore::setFixedBaseLink(bool value)
{
	BodySim* sim = getSim();

	if(sim)
		sim->getLowLevelBody().mCore->fixedBaseLink = PxU8(value);
}

void Sc::BodyCore::onRemoveKinematicFromScene()
{
	PX_ASSERT(mCore.mFlags & PxRigidBodyFlag::eKINEMATIC);
	PX_ASSERT(getSim() && getSim()->checkSimStateKinematicStatus(true));

	// make sure that a kinematic which is not part of a scene is in the expected state
	mCore.wakeCounter = 0.0f;
	mCore.linearVelocity = PxVec3(0.0f);
	mCore.angularVelocity = PxVec3(0.0f);
}

///////////////////////////////////////////////////////////////////////////////
