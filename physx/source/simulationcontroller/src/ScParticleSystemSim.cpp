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

#include "ScParticleSystemSim.h"
#include "ScParticleSystemCore.h"
#include "ScScene.h"

using namespace physx;
using namespace physx::Dy;


Sc::ParticleSystemSim::ParticleSystemSim(ParticleSystemCore& core, Scene& scene) :
	ActorSim(scene, core),
	mShapeSim(*this, &core.getShapeCore())
{
	
	mLLParticleSystem = scene.createLLParticleSystem(this);

	mNodeIndex = scene.getSimpleIslandManager()->addParticleSystem(mLLParticleSystem, false);

	scene.getSimpleIslandManager()->activateNode(mNodeIndex);


	//mCore.setSim(this);

	mLLParticleSystem->setElementId(mShapeSim.getElementID());

	PxParticleSystemGeometry geometry;
	geometry.mSolverType = PxParticleSolverType::ePBD;

	core.getShapeCore().setGeometry(geometry);
	
	PxsShapeCore* shapeCore = const_cast<PxsShapeCore*>(&core.getShapeCore().getCore());
	mLLParticleSystem->setShapeCore(shapeCore);
}

Sc::ParticleSystemSim::~ParticleSystemSim()
{
	if (!mLLParticleSystem)
		return;

	mScene.destroyLLParticleSystem(*mLLParticleSystem);

	mScene.getSimpleIslandManager()->removeNode(mNodeIndex);

	mCore.setSim(NULL);
}

void Sc::ParticleSystemSim::updateBounds()
{
	mShapeSim.updateBounds();
}

void Sc::ParticleSystemSim::updateBoundsInAABBMgr()
{
	mShapeSim.updateBoundsInAABBMgr();
}

PxBounds3 Sc::ParticleSystemSim::getBounds() const
{
	return mShapeSim.getBounds();
}

bool Sc::ParticleSystemSim::isSleeping() const
{
	return false;
}

void Sc::ParticleSystemSim::sleepCheck(PxReal dt)
{
	PX_UNUSED(dt);
}

/*void Sc::ParticleSystemSim::activate()
{
	activateInteractions(*this);
}

void Sc::ParticleSystemSim::deactivate()
{
	deactivateInteractions(*this);
}*/

#endif //PX_SUPPORT_GPU_PHYSX
