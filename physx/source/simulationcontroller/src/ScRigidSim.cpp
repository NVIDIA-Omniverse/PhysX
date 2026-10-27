// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScScene.h"
#include "ScRigidSim.h"
#include "ScShapeSim.h"
#include "PxsSimulationController.h"

using namespace physx;
using namespace Sc;

/*
	PT:

	The BP group ID comes from a Cm::IDPool, and ActorSim is the only class releasing the ID.

	The rigid tracker ID comes from a Cm::IDPool internal to an ObjectIDTracker, and ActorSim
	is the only class using it.

	Thus we should:
	- promote the BP group ID stuff to a "tracker" object
	- use the BP group ID as a rigid ID
*/

RigidSim::RigidSim(Scene& scene, RigidCore& core) : ActorSim(scene, core)
{
}

RigidSim::~RigidSim()
{
}

void notifyActorInteractionsOfTransformChange(ActorSim& actor);
void RigidSim::notifyShapesOfTransformChange()
{
	PxU32 nbElems = getNbElements();
	ElementSim** elems = getElements();
	while (nbElems--)
	{
		ShapeSim* sim = static_cast<ShapeSim*>(*elems++);
		sim->markBoundsForUpdate();
	}

	notifyActorInteractionsOfTransformChange(*this);
}

