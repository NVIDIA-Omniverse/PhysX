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

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScHairSystemCore.h"

#include "ScHairSystemSim.h"
#include "ScPhysics.h"

namespace physx
{
namespace Sc
{

HairSystemCore::HairSystemCore()
: ActorCore(PxActorType::eHAIRSYSTEM, PxActorFlag::eVISUALIZATION, PX_DEFAULT_CLIENT, 0)
{
}

HairSystemCore::~HairSystemCore() {}

void HairSystemCore::setMaterial(const PxU16 handle) { mShapeCore.getLLCore().setMaterial(handle); }

void HairSystemCore::clearMaterials() { mShapeCore.getLLCore().clearMaterials(); }

void HairSystemCore::setSleepThreshold(const PxReal v)
{
	mShapeCore.getLLCore().mSleepThreshold = v;
	mShapeCore.getLLCore().mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
}

void HairSystemCore::setSolverIterationCounts(const PxU16 c)
{
	mShapeCore.getLLCore().mSolverIterationCounts = c;
	mShapeCore.getLLCore().mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
}

void HairSystemCore::setWakeCounter(const PxReal v)
{
	mShapeCore.getLLCore().mWakeCounter = v;
	mShapeCore.getLLCore().mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;

	HairSystemSim* sim = getSim();
	if(sim)
	{
		sim->onSetWakeCounter();
	}
}

bool HairSystemCore::isSleeping() const
{
	HairSystemSim* sim = getSim();
	return sim ? sim->isSleeping() : (mShapeCore.getLLCore().mWakeCounter == 0.0f);
}

void HairSystemCore::wakeUp(PxReal wakeCounter)
{
	mShapeCore.getLLCore().mWakeCounter = wakeCounter;
	mShapeCore.getLLCore().mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
}

void HairSystemCore::putToSleep()
{
	mShapeCore.getLLCore().mWakeCounter = 0.0f;
	mShapeCore.getLLCore().mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
}

PxActor* HairSystemCore::getPxActor() const
{
	return PxPointerOffset<PxActor*>(const_cast<HairSystemCore*>(this), gOffsetTable.scCore2PxActor[getActorCoreType()]);
}

HairSystemSim* HairSystemCore::getSim() const { return static_cast<HairSystemSim*>(ActorCore::getSim()); }

PxReal HairSystemCore::getContactOffset() const { return mShapeCore.getContactOffset(); }

void HairSystemCore::setContactOffset(PxReal v)
{
	mShapeCore.setContactOffset(v);
	HairSystemSim* sim = getSim();
	if(sim)
	{
		sim->getScene().updateContactDistance(sim->getShapeSim().getElementID(), v);
	}
}

void HairSystemCore::addAttachment(const BodySim& bodySim)
{
	const HairSystemSim* sim = getSim();
	if(sim)
	{
		sim->getScene().addAttachment(bodySim, *sim);
	}
}

void HairSystemCore::removeAttachment(const BodySim& bodySim)
{
	const HairSystemSim* sim = getSim();
	if(sim)
	{
		sim->getScene().removeAttachment(bodySim, *sim);
	}
}

void HairSystemCore::addAttachment(const SoftBodySim& sbSim)
{
	const Sc::HairSystemSim* sim = getSim();
	if(sim)
	{
		sim->getScene().addAttachment(sbSim, *sim);
	}
}

void HairSystemCore::removeAttachment(const SoftBodySim& sbSim)
{
	const Sc::HairSystemSim* sim = getSim();
	if(sim)
	{
		sim->getScene().removeAttachment(sbSim, *sim);
	}
}

void Sc::HairSystemCore::setFlags(PxHairSystemFlags flags)
{
	mShapeCore.getLLCore().mParams.mFlags = flags;
	mShapeCore.getLLCore().mDirtyFlags |= Dy::HairSystemDirtyFlag::ePARAMETERS;
}

} // namespace Sc
} // namespace physx

#endif // PX_SUPPORT_GPU_PHYSX
