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

#include "ScHairSystemShapeSim.h"
#include "ScNPhaseCore.h"
#include "ScHairSystemSim.h"
#include "PxsContext.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::HairSystemShapeSim::HairSystemShapeSim(HairSystemSim& hairSystemSim, const HairSystemShapeCore* core) :
	ShapeSimBase(hairSystemSim, core)
{
	createLowLevelVolume();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::HairSystemShapeSim::~HairSystemShapeSim()
{
	if (isInBroadPhase())
		destroyLowLevelVolume();

	PX_ASSERT(!isInBroadPhase());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::HairSystemShapeSim::getFilterInfo(PxFilterObjectAttributes& filterAttr, PxFilterData& filterData) const
{
	filterAttr = 0;
	setFilterObjectAttributeType(filterAttr, PxFilterObjectType::eHAIRSYSTEM);
	filterData = getBodySim().getCore().getShapeCore().getSimulationFilterData();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::HairSystemShapeSim::updateBounds()
{
	Scene& scene = getScene();

	PxBounds3 worldBounds = PxBounds3(PxVec3(0.f), PxVec3(0.f));
	const PxReal contactOffset = getBodySim().getCore().getContactOffset();
	worldBounds.fattenSafe(contactOffset); // fatten for fast moving colliders
	scene.getBoundsArray().setBounds(worldBounds, getElementID());
	scene.getAABBManager()->getChangedAABBMgActorHandleMap().growAndSet(getElementID());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::HairSystemShapeSim::updateBoundsInAABBMgr()
{
	//we are updating the bound in GPU so we just need to set the actor handle in CPU to make sure
	//the GPU BP will process the vertices
	Scene& scene = getScene();
	scene.getAABBManager()->getChangedAABBMgActorHandleMap().growAndSet(getElementID());
	scene.getAABBManager()->setGPUStateChanged();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxBounds3 Sc::HairSystemShapeSim::getBounds() const
{
	PxBounds3 bounds = getScene().getBoundsArray().getBounds(getElementID());
	return bounds;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::HairSystemShapeSim::createLowLevelVolume()
{
	// PX_ASSERT(getWorldBounds().isFinite());

	const PxU32 index = getElementID();

	getScene().getBoundsArray().setBounds(PxBounds3(PxVec3(0.f), PxVec3(0.f)), index);

	{
		const PxU32 group = Bp::FilterGroup::eDYNAMICS_BASE + getActor().getActorID();
		const PxU32 type = Bp::FilterType::HAIRSYSTEM;
		addToAABBMgr(getCore().getContactOffset(), Bp::FilterGroup::Enum((group << BP_FILTERING_TYPE_SHIFT_BIT) | type), Bp::ElementType::eSHAPE);
	}

	// PT: TODO: what's the difference between "getContactOffset()" and "getCore().getContactOffset()" above?
	getScene().updateContactDistance(index, getContactOffset());

	PxsTransformCache& cache = getScene().getLowLevelContext()->getTransformCache();
	cache.initEntry(index);

	PxTransform idt(PxIdentity);

	cache.setTransformCache(idt, 0, index);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sc::HairSystemShapeSim::destroyLowLevelVolume()
{
	if (!isInBroadPhase())
		return;

	Sc::Scene& scene = getScene();
	PxsContactManagerOutputIterator outputs = scene.getLowLevelContext()->getNphaseImplementationContext()->getContactManagerOutputs();
	scene.getNPhaseCore()->onVolumeRemoved(this, 0, outputs);
	removeFromAABBMgr();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sc::HairSystemSim& Sc::HairSystemShapeSim::getBodySim()	const
{
	return static_cast<HairSystemSim&>(getActor());
}

#endif // PX_SUPPORT_GPU_PHYSX
