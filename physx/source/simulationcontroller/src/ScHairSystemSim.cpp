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


#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScHairSystemSim.h"
#include "ScHairSystemCore.h"
#include "ScHairSystemShapeCore.h"
#include "ScScene.h"
#include "PxsSimulationController.h"

using namespace physx;
using namespace physx::Dy;

Sc::HairSystemSim::HairSystemSim(HairSystemCore& core, Scene& scene) :
	ActorSim(scene, core),
	mShapeSim(*this, &core.getShapeCore())
{
	mLLHairSystem = scene.createLLHairSystem(this);

	mNodeIndex = scene.getSimpleIslandManager()->addHairSystem(mLLHairSystem, false);
	scene.getSimpleIslandManager()->activateNode(mNodeIndex);

	mLLHairSystem->setElementId(mShapeSim.getElementID());

	PxHairSystemGeometry geometry;
	core.getShapeCore().setGeometry(geometry);
	PxsShapeCore* shapeCore = const_cast<PxsShapeCore*>(&core.getShapeCore().getCore());
	mLLHairSystem->setShapeCore(shapeCore);
}

Sc::HairSystemSim::~HairSystemSim()
{
	if (!mLLHairSystem) {
		return;
	}

	mScene.destroyLLHairSystem(*mLLHairSystem);
	mScene.getSimpleIslandManager()->removeNode(mNodeIndex);
	mCore.setSim(NULL);
}

void Sc::HairSystemSim::updateBounds()
{
	mShapeSim.updateBounds();
}

void Sc::HairSystemSim::updateBoundsInAABBMgr()
{
	mShapeSim.updateBoundsInAABBMgr();
}

PxBounds3 Sc::HairSystemSim::getBounds() const
{
	return mShapeSim.getBounds();
}


bool Sc::HairSystemSim::isSleeping() const
{
	IG::IslandSim& sim = mScene.getSimpleIslandManager()->getAccurateIslandSim();
	return sim.getActiveNodeIndex(mNodeIndex) == PX_INVALID_NODE;
}

void Sc::HairSystemSim::onSetWakeCounter()
{
	getScene().getSimulationController()->setHairSystemWakeCounter(mLLHairSystem);
	if (mLLHairSystem->getCore().mWakeCounter > 0.0f) {
		getScene().getSimpleIslandManager()->activateNode(mNodeIndex);
	}
	else
	{
		getScene().getSimpleIslandManager()->deactivateNode(mNodeIndex);
	}
}

/*void Sc::HairSystemSim::activate()
{
	activateInteractions(*this);
}

void Sc::HairSystemSim::deactivate()
{
	deactivateInteractions(*this);
}*/

#endif //PX_SUPPORT_GPU_PHYSX
