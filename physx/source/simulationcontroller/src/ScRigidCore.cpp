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

#include "ScRigidCore.h"
#include "ScStaticCore.h"
#include "ScRigidSim.h"
#include "ScShapeSim.h"
#include "ScScene.h"

using namespace physx;
using namespace Sc;

static ShapeSim& getSimForShape(const ShapeCore& core, const ActorSim& actorSim)
{
	if(core.getExclusiveSim())
	{
		return *core.getExclusiveSim();
	}

	//Must be a shared shape.
	//Search backwards to emulate the behaviour of the previous linked list.
	PxU32 nbElems = actorSim.getNbElements();
	ElementSim*const* elems = actorSim.getElements() + (nbElems - 1);
	while (nbElems--)
	{
		ShapeSim* sim = static_cast<ShapeSim*>(*elems--);
		if (&sim->getCore() == &core)
			return *sim;
	}

	PX_ASSERT(0); // should never fail
	return *reinterpret_cast<ShapeSim*>(1);
}

RigidCore::RigidCore(const PxActorType::Enum type) : ActorCore(type, PxActorFlag::eVISUALIZATION, PX_DEFAULT_CLIENT, 0)
{
}

RigidCore::~RigidCore()
{
}

void RigidCore::addShapeToScene(ShapeCore& shapeCore)
{
	RigidSim* sim = getSim();
	PX_ASSERT(sim);
	if(!sim)
		return;
	sim->getScene().addShape_(*sim, shapeCore);
}

void RigidCore::removeShapeFromScene(ShapeCore& shapeCore, bool wakeOnLostTouch)
{
	RigidSim* sim = getSim();
	if(!sim)
		return;
	ShapeSim& s = getSimForShape(shapeCore, *sim);
	sim->getScene().removeShape_(s, wakeOnLostTouch);
}

void RigidCore::unregisterShapeFromNphase(Sc::ShapeCore& shapeCore)
{
	RigidSim* sim = getSim();
	if (!sim)
		return;
	ShapeSim& s = getSimForShape(shapeCore, *sim);
	s.getScene().unregisterShapeFromNphase(shapeCore, s.getElementID());
}

void RigidCore::registerShapeInNphase(Sc::ShapeCore& shapeCore)
{
	RigidSim* sim = getSim();
	if (!sim)
		return;
	ShapeSim& s = getSimForShape(shapeCore, *sim);
	s.getScene().registerShapeInNphase(this, shapeCore, s.getElementID());
}

void RigidCore::onShapeChange(ShapeCore& shape, ShapeChangeNotifyFlags notifyFlags)
{
	RigidSim* sim = getSim();
	if(!sim)
		return;
	ShapeSim& s = getSimForShape(shape, *sim);

	if(notifyFlags & ShapeChangeNotifyFlag::eGEOMETRY)
		s.onVolumeOrTransformChange();
	if(notifyFlags & ShapeChangeNotifyFlag::eMATERIAL)
		s.onMaterialChange();
	if(notifyFlags & ShapeChangeNotifyFlag::eRESET_FILTERING)
		s.onResetFiltering();
	if(notifyFlags & ShapeChangeNotifyFlag::eSHAPE2BODY)
		s.onVolumeOrTransformChange();
	if(notifyFlags & ShapeChangeNotifyFlag::eFILTERDATA)
		s.onFilterDataChange();
	if(notifyFlags & ShapeChangeNotifyFlag::eCONTACTOFFSET)
		s.onContactOffsetChange();
	if(notifyFlags & ShapeChangeNotifyFlag::eRESTOFFSET)
		s.onRestOffsetChange();
}

void RigidCore::onShapeFlagsChange(ShapeCore& shape, PxShapeFlags oldShapeFlags)
{
	// DS: We pass flags to avoid searching multiple times or exposing RigidSim outside SC.
	//If we start hitting this a lot we should do it
	// a different way, but shape modification after insertion is rare. 

	RigidSim* sim = getSim();
	if(!sim)
		return;
	ShapeSim& s = getSimForShape(shape, *sim);
	s.onFlagChange(oldShapeFlags);
}

RigidSim* RigidCore::getSim() const
{
	return static_cast<RigidSim*>(ActorCore::getSim());
}

PxU32 RigidCore::getRigidID() const
{
	return static_cast<RigidSim*>(ActorCore::getSim())->getActorID();
}

