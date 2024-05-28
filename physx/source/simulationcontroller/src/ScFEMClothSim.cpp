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

#include "ScFEMClothSim.h"
#include "ScFEMClothCore.h"
#include "ScScene.h"

#include "ScInteraction.h"  // to be deleted
#include "PxsSimulationController.h"

using namespace physx;
using namespace physx::Dy;


Sc::FEMClothSim::FEMClothSim(FEMClothCore& core, Scene& scene) :
	ActorSim(scene, core),
	mShapeSim(*this)
{
	mLLFEMCloth = scene.createLLFEMCloth(this);

	mNodeIndex = scene.getSimpleIslandManager()->addFEMCloth(mLLFEMCloth, false);

	scene.getSimpleIslandManager()->activateNode(mNodeIndex);

	mLLFEMCloth->setElementId(mShapeSim.getElementID());
}

Sc::FEMClothSim::~FEMClothSim()
{
	if (!mLLFEMCloth)
		return;

	mScene.destroyLLFEMCloth(*mLLFEMCloth);

	mScene.getSimpleIslandManager()->removeNode(mNodeIndex);

	mCore.setSim(NULL);
}

void Sc::FEMClothSim::updateBounds()
{
	mShapeSim.updateBounds();
}

void Sc::FEMClothSim::updateBoundsInAABBMgr()
{
	mShapeSim.updateBoundsInAABBMgr();
}

PxBounds3 Sc::FEMClothSim::getBounds() const
{
	return mShapeSim.getBounds();
}

bool Sc::FEMClothSim::isSleeping() const
{
	IG::IslandSim& sim = mScene.getSimpleIslandManager()->getAccurateIslandSim();
	return sim.getActiveNodeIndex(mNodeIndex) == PX_INVALID_NODE;
}

void Sc::FEMClothSim::onSetWakeCounter()
{
	getScene().getSimulationController()->setClothWakeCounter(mLLFEMCloth);
	if (mLLFEMCloth->getCore().wakeCounter > 0.f)
		getScene().getSimpleIslandManager()->activateNode(mNodeIndex);
	else
		getScene().getSimpleIslandManager()->deactivateNode(mNodeIndex);
}

void Sc::FEMClothSim::attachShapeCore(ShapeCore* core)
{
	mShapeSim.attachShapeCore(core);

	PxsShapeCore* shapeCore = const_cast<PxsShapeCore*>(&core->getCore());
	mLLFEMCloth->setShapeCore(shapeCore);
}

#endif //PX_SUPPORT_GPU_PHYSX
