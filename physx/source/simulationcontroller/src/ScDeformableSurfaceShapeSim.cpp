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

#include "ScDeformableSurfaceShapeSim.h"

#include "ScNPhaseCore.h"
#include "ScScene.h"

#include "ScDeformableSurfaceSim.h"
#include "PxsContext.h"
#include "BpAABBManager.h"
#include "ScSqBoundsManager.h"
#include "geometry/PxTetrahedronMesh.h"
#include "geometry/PxTriangleMesh.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::DeformableSurfaceShapeSim::DeformableSurfaceShapeSim(DeformableSurfaceSim& deformableSurface) :
	ShapeSimBase(deformableSurface, NULL),
	initialTransform(PxVec3(0, 0, 0)),
	initialScale(1.0f)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::DeformableSurfaceShapeSim::~DeformableSurfaceShapeSim()
{
	if (isInBroadPhase())
		destroyLowLevelVolume();

	PX_ASSERT(!isInBroadPhase());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Sc::DeformableSurfaceShapeSim::attachShapeCore(const Sc::ShapeCore* shapeCore)
{
	setCore(shapeCore);
	createLowLevelVolume();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::DeformableSurfaceShapeSim::updateBounds()
{
	Scene& scene = getScene();

	PxBounds3 worldBounds = getWorldBounds();
	worldBounds.fattenSafe(getContactOffset()); // fatten for fast moving colliders
	scene.getBoundsArray().setBounds(worldBounds, getElementID());
	scene.getAABBManager()->getChangedAABBMgActorHandleMap().growAndSet(getElementID());
}

void Sc::DeformableSurfaceShapeSim::updateBoundsInAABBMgr()
{
	Scene& scene = getScene();
	scene.getAABBManager()->getChangedAABBMgActorHandleMap().growAndSet(getElementID());
	scene.getAABBManager()->setGPUStateChanged();
}

PxBounds3 Sc::DeformableSurfaceShapeSim::getBounds() const
{
	PxBounds3 bounds = getScene().getBoundsArray().getBounds(getElementID());
	return bounds;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::DeformableSurfaceShapeSim::createLowLevelVolume()
{
	PX_ASSERT(getWorldBounds().isFinite());

	const PxU32 index = getElementID();

	getScene().getBoundsArray().setBounds(getWorldBounds(), index);

	const PxReal contactOffset = getContactOffset();
	addToAABBMgr(contactOffset, Bp::FilterType::DEFORMABLE_SURFACE);

	getScene().updateContactDistance(index, contactOffset);

	PxsTransformCache& cache = getScene().getLowLevelContext()->getTransformCache();
	cache.initEntry(index);

	PxTransform idt(PxIdentity);

	cache.setTransformCache(idt, 0, index);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::DeformableSurfaceShapeSim::destroyLowLevelVolume()
{
	if (!isInBroadPhase())
		return;

	Sc::Scene& scene = getScene();
	PxsContactManagerOutputIterator outputs = scene.getLowLevelContext()->getNphaseImplementationContext()->getContactManagerOutputs();
	scene.getNPhaseCore()->onVolumeRemoved(this, 0, outputs);
	removeFromAABBMgr();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxBounds3 Sc::DeformableSurfaceShapeSim::getWorldBounds() const
{
	const PxTriangleMeshGeometry& triGeom = static_cast<const PxTriangleMeshGeometry&>(getCore().getGeometry());
	PxTriangleMesh* triMesh = triGeom.triangleMesh;

	// PT: are you sure you want to go through the Px API here?
	PxBounds3 bounds = triMesh->getLocalBounds();

	bounds.minimum *= initialScale;
	bounds.maximum *= initialScale;
	bounds = PxBounds3::transformFast(initialTransform, bounds);

	return bounds;
}

Sc::DeformableSurfaceSim& Sc::DeformableSurfaceShapeSim::getBodySim()	const
{
	return static_cast<DeformableSurfaceSim&>(getActor());
}

#endif //PX_SUPPORT_GPU_PHYSX
