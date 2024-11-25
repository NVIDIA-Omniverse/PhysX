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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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

