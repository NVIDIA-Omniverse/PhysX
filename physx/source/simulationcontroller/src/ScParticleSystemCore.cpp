// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScParticleSystemCore.h"

#include "ScPhysics.h"
#include "ScParticleSystemSim.h"
#include "DyParticleSystem.h"
#include "PxvGlobals.h"
#include "PxPhysXGpu.h"

using namespace physx;

Sc::ParticleSystemCore::ParticleSystemCore(PxActorType::Enum actorType) :
	ActorCore(actorType, PxActorFlag::eVISUALIZATION, PX_DEFAULT_CLIENT, 0)
{
}

Sc::ParticleSystemCore::~ParticleSystemCore()
{
}

Sc::ParticleSystemSim* Sc::ParticleSystemCore::getSim()	const 
{ 
	return static_cast<ParticleSystemSim*>(ActorCore::getSim()); 
}

PxReal Sc::ParticleSystemCore::getSleepThreshold() const
{
	return mShapeCore.getLLCore().sleepThreshold;//mCore.sleepThreshold;
}

void Sc::ParticleSystemCore::setSleepThreshold(const PxReal v)
{
	mShapeCore.getLLCore().sleepThreshold = v;
}

PxReal Sc::ParticleSystemCore::getRestOffset() const
{
	return mShapeCore.getLLCore().restOffset;
}

void Sc::ParticleSystemCore::setRestOffset(const PxReal v)
{
	mShapeCore.getLLCore().restOffset = v;
}

PxReal Sc::ParticleSystemCore::getContactOffset() const
{
	return mShapeCore.getContactOffset();
}

void Sc::ParticleSystemCore::setContactOffset(const PxReal v)
{
	mShapeCore.setContactOffset(v);

	Sc::ParticleSystemSim* sim = getSim();
	if (sim)
	{
		sim->getScene().updateContactDistance(sim->getShapeSim().getElementID(), v);
	}
}

PxReal Sc::ParticleSystemCore::getParticleContactOffset() const
{
	return mShapeCore.getLLCore().particleContactOffset;
}

void Sc::ParticleSystemCore::setParticleContactOffset(const PxReal v)
{
	mShapeCore.getLLCore().particleContactOffset = v;
}

PxReal Sc::ParticleSystemCore::getSolidRestOffset() const
{
	return mShapeCore.getLLCore().solidRestOffset;
}

void Sc::ParticleSystemCore::setSolidRestOffset(const PxReal v)
{
	mShapeCore.getLLCore().solidRestOffset = v;
}

PxReal Sc::ParticleSystemCore::getFluidRestOffset() const
{
	return mShapeCore.getLLCore().fluidRestOffset;
}

void Sc::ParticleSystemCore::setFluidRestOffset(const PxReal v)
{
	mShapeCore.getLLCore().fluidRestOffset = v;
}

void Sc::ParticleSystemCore::setMaxDepenetrationVelocity(const PxReal v)
{
	mShapeCore.getLLCore().maxDepenetrationVelocity = v;
}

PxReal Sc::ParticleSystemCore::getMaxDepenetrationVelocity() const
{
	return mShapeCore.getLLCore().maxDepenetrationVelocity;
}

void Sc::ParticleSystemCore::setMaxVelocity(const PxReal v)
{
	mShapeCore.getLLCore().maxVelocity = v;
}

PxReal Sc::ParticleSystemCore::getMaxVelocity() const
{
	return mShapeCore.getLLCore().maxVelocity;
}

PxParticleSystemCallback* Sc::ParticleSystemCore::getParticleSystemCallback() const
{
	return mShapeCore.getLLCore().mCallback;
}

void Sc::ParticleSystemCore::setParticleSystemCallback(PxParticleSystemCallback* callback)
{
	mShapeCore.getLLCore().mCallback = callback;
}

PxReal Sc::ParticleSystemCore::getFluidBoundaryDensityScale() const
{
	return mShapeCore.getLLCore().fluidBoundaryDensityScale;
}

void Sc::ParticleSystemCore::setFluidBoundaryDensityScale(const PxReal v)
{
	mShapeCore.getLLCore().fluidBoundaryDensityScale = v;
}

PxU32 Sc::ParticleSystemCore::getGridSizeX() const
{
	return mShapeCore.getLLCore().gridSizeX;
}

void Sc::ParticleSystemCore::setGridSizeX(const PxU32 v)
{
	mShapeCore.getLLCore().gridSizeX = v;
}

PxU32 Sc::ParticleSystemCore::getGridSizeY() const
{
	return mShapeCore.getLLCore().gridSizeY;
}

void Sc::ParticleSystemCore::setGridSizeY(const PxU32 v)
{
	mShapeCore.getLLCore().gridSizeY = v;
}

PxU32 Sc::ParticleSystemCore::getGridSizeZ() const
{
	return mShapeCore.getLLCore().gridSizeZ;
}

void Sc::ParticleSystemCore::setGridSizeZ(const PxU32 v)
{
	mShapeCore.getLLCore().gridSizeZ = v;
}

void Sc::ParticleSystemCore::setSolverIterationCounts(PxU16 c)
{
	mShapeCore.getLLCore().solverIterationCounts = c;
	Sc::ParticleSystemSim* sim = getSim();
	if (sim)
		sim->getScene().setDynamicsDirty();
}


PxReal Sc::ParticleSystemCore::getWakeCounter() const
{
	return mShapeCore.getLLCore().wakeCounter;
}

void Sc::ParticleSystemCore::setWakeCounter(const PxReal v)
{
	mShapeCore.getLLCore().wakeCounter = v;
}

void Sc::ParticleSystemCore::setWakeCounterInternal(const PxReal v)
{
	mShapeCore.getLLCore().wakeCounter = v;
}

bool Sc::ParticleSystemCore::isSleeping() const
{
	Sc::ParticleSystemSim* sim = getSim();
	return sim ? sim->isSleeping() : (mShapeCore.getLLCore().wakeCounter == 0.0f);
}

void Sc::ParticleSystemCore::wakeUp(PxReal wakeCounter)
{
	mShapeCore.getLLCore().wakeCounter = wakeCounter;
}

void Sc::ParticleSystemCore::putToSleep()
{
	mShapeCore.getLLCore().wakeCounter = 0.0f;
}

PxActor* Sc::ParticleSystemCore::getPxActor() const
{
	return PxPointerOffset<PxActor*>(const_cast<ParticleSystemCore*>(this), gOffsetTable.scCore2PxActor[getActorCoreType()]);
}

void Sc::ParticleSystemCore::setFlags(PxParticleFlags flags)
{
	Sc::ParticleSystemSim* sim = getSim();
	PxParticleFlags oldFlags = mShapeCore.getLLCore().mFlags;
	mShapeCore.getLLCore().mFlags = flags;

	if (sim)
	{
		bool wasRigidCollisionDisabled = oldFlags & PxParticleFlag::eDISABLE_RIGID_COLLISION;
		bool isRigidCollisionDisabled = flags & PxParticleFlag::eDISABLE_RIGID_COLLISION;

		if (wasRigidCollisionDisabled ^ isRigidCollisionDisabled)
		{
			if (wasRigidCollisionDisabled)
				sim->createLowLevelVolume();
			else
				sim->destroyLowLevelVolume();
		}
	}
}

#endif //PX_SUPPORT_GPU_PHYSX
