// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "ScDeformableSurfaceSim.h"

#include "geometry/PxTriangleMesh.h"

using namespace physx;
using namespace Dy;

Sc::DeformableSurfaceSim::DeformableSurfaceSim(DeformableSurfaceCore& core, Scene& scene) :
	GPUActorSim(scene, core, NULL)
{
	mLLDeformableSurface = scene.createLLDeformableSurface(this);

	mNodeIndex = scene.getSimpleIslandManager()->addNode(false, false, IG::Node::eDEFORMABLE_SURFACE_TYPE, mLLDeformableSurface);

	scene.getSimpleIslandManager()->activateNode(mNodeIndex);

	mLLDeformableSurface->setElementId(mShapeSim.getElementID());
}

Sc::DeformableSurfaceSim::~DeformableSurfaceSim()
{
	if (!mLLDeformableSurface)
		return;

	mScene.destroyLLDeformableSurface(*mLLDeformableSurface);

	mScene.getSimpleIslandManager()->removeNode(mNodeIndex);

	mCore.setSim(NULL);
}

bool Sc::DeformableSurfaceSim::isSleeping() const
{
	IG::IslandSim& sim = mScene.getSimpleIslandManager()->getAccurateIslandSim();
	return sim.getActiveNodeIndex(mNodeIndex) == PX_INVALID_NODE;
}

void Sc::DeformableSurfaceSim::onSetWakeCounter()
{
	mScene.getSimulationController()->setClothWakeCounter(mLLDeformableSurface);
	if (mLLDeformableSurface->getCore().wakeCounter > 0.f)
		mScene.getSimpleIslandManager()->activateNode(mNodeIndex);
	else
		mScene.getSimpleIslandManager()->deactivateNode(mNodeIndex);
}

void Sc::DeformableSurfaceSim::attachShapeCore(ShapeCore* core)
{
	mShapeSim.setCore(core);
	{
		PX_ASSERT(getWorldBounds().isFinite());

		const PxU32 index = mShapeSim.getElementID();

		mScene.getBoundsArray().setBounds(getWorldBounds(), index);

		addToAABBMgr(Bp::FilterType::DEFORMABLE_SURFACE);
	}

	mLLDeformableSurface->setShapeCore(core);
}

PxBounds3 Sc::DeformableSurfaceSim::getWorldBounds() const
{
	const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(mShapeSim.getCore().getGeometry());

	// PT: are you sure you want to go through the Px API here?
	return triGeom.triangleMesh->getLocalBounds();
}

#endif //PX_SUPPORT_GPU_PHYSX
