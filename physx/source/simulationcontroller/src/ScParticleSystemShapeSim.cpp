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

#include "ScParticleSystemShapeSim.h"
#include "ScNPhaseCore.h"
#include "ScParticleSystemSim.h"
#include "PxsContext.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::ParticleSystemShapeSim::ParticleSystemShapeSim(ParticleSystemSim& particleSim, const ParticleSystemShapeCore* core) :
	ShapeSimBase(particleSim, core)
{
	createLowLevelVolume();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::ParticleSystemShapeSim::~ParticleSystemShapeSim()
{
	if (isInBroadPhase())
		destroyLowLevelVolume();

	PX_ASSERT(!isInBroadPhase());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::ParticleSystemShapeSim::updateBounds()
{
	Scene& scene = getScene();

	PxBounds3 worldBounds = PxBounds3(PxVec3(0.f), PxVec3(0.f));
	const PxReal contactOffset = getBodySim().getCore().getContactOffset();
	worldBounds.fattenSafe(contactOffset); // fatten for fast moving colliders
	scene.getBoundsArray().setBounds(worldBounds, getElementID());
	scene.getAABBManager()->getChangedAABBMgActorHandleMap().growAndSet(getElementID());
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::ParticleSystemShapeSim::updateBoundsInAABBMgr()
{
	//we are updating the bound in GPU so we just need to set the actor handle in CPU to make sure
	//the GPU BP will process the particles

	if (!(static_cast<Sc::ParticleSystemSim&>(getActor()).getCore().getFlags() & PxParticleFlag::eDISABLE_RIGID_COLLISION))
	{
		Scene& scene = getScene();
		scene.getAABBManager()->getChangedAABBMgActorHandleMap().growAndSet(getElementID());
		scene.getAABBManager()->setGPUStateChanged();
	}
}

PxBounds3 Sc::ParticleSystemShapeSim::getBounds() const
{
	PxBounds3 bounds = getScene().getBoundsArray().getBounds(getElementID());
	return bounds;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Sc::ParticleSystemShapeSim::createLowLevelVolume()
{
	//PX_ASSERT(getWorldBounds().isFinite());

	const PxU32 index = getElementID();


	if (!(static_cast<Sc::ParticleSystemSim&>(getActor()).getCore().getFlags() & PxParticleFlag::eDISABLE_RIGID_COLLISION))
	{
		getScene().getBoundsArray().setBounds(PxBounds3(PxVec3(PX_MAX_BOUNDS_EXTENTS), PxVec3(-PX_MAX_BOUNDS_EXTENTS)), index);
		mInBroadPhase = true;
	}
	else
		getScene().getAABBManager()->reserveSpaceForBounds(index);

	const PxReal contactOffset = getContactOffset();
	addToAABBMgr(contactOffset, Bp::FilterType::PARTICLESYSTEM);

	getScene().updateContactDistance(index, contactOffset);

	PxsTransformCache& cache = getScene().getLowLevelContext()->getTransformCache();
	cache.initEntry(index);

	PxTransform idt(PxIdentity);

	cache.setTransformCache(idt, 0, index);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::ParticleSystemShapeSim::destroyLowLevelVolume()
{
	if (!isInBroadPhase())
		return;

	Sc::Scene& scene = getScene();
	PxsContactManagerOutputIterator outputs = scene.getLowLevelContext()->getNphaseImplementationContext()->getContactManagerOutputs();
	scene.getNPhaseCore()->onVolumeRemoved(this, 0, outputs);
	removeFromAABBMgr();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::ParticleSystemSim& Sc::ParticleSystemShapeSim::getBodySim()	const
{
	return static_cast<ParticleSystemSim&>(getActor());
}

#endif //PX_SUPPORT_GPU_PHYSX
