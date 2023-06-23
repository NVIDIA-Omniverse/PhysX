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

#include "ScScene.h"
#include "ScConstraintSim.h"
#include "ScConstraintCore.h"
#include "ScConstraintInteraction.h"
#include "common/PxProfileZone.h"

using namespace physx;

// PT: the breakable constraints are added to / removed from mActiveBreakableConstraints:

void Sc::Scene::addActiveBreakableConstraint(Sc::ConstraintSim* c, Sc::ConstraintInteraction* ci)
{
	PX_ASSERT(ci && ci->readInteractionFlag(InteractionFlag::eIS_ACTIVE));
	PX_UNUSED(ci);
	PX_ASSERT(!mActiveBreakableConstraints.contains(c));
	PX_ASSERT(!c->isBroken());
	mActiveBreakableConstraints.insert(c);
	c->setFlag(ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED);
}

void Sc::Scene::removeActiveBreakableConstraint(Sc::ConstraintSim* c)
{
	const bool exists = mActiveBreakableConstraints.erase(c);
	PX_ASSERT(exists);
	PX_UNUSED(exists);
	c->clearFlag(ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED);
}

// PT: then at runtime we parse mActiveBreakableConstraints, check for max force exceeded,
// and add broken constraints to mBrokenConstraints:

void Sc::Scene::checkConstraintBreakage()
{
	PX_PROFILE_ZONE("Sim.checkConstraintBreakage", mContextId);

	PxU32 count = mActiveBreakableConstraints.size();
	if(!count)
		return;

	PxPinnedArray<Dy::ConstraintWriteback>& pool = mDynamicsContext->getConstraintWriteBackPool();

	ConstraintSim* const* constraints = mActiveBreakableConstraints.getEntries(); 
	while(count--)
	{
		ConstraintSim* sim = constraints[count];	// start from the back because broken constraints get removed from the list

		PX_ASSERT(sim->readFlag(ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED));

		const Dy::ConstraintWriteback& solverOutput = pool[sim->getLowLevelConstraint().index];
		if(solverOutput.broken)
		{
			sim->setFlag(ConstraintSim::eBROKEN);

			ConstraintCore& core = sim->getCore();
			if(mSimulationEventCallback)
			{
				PX_ASSERT(mBrokenConstraints.find(&core) == mBrokenConstraints.end());
				mBrokenConstraints.pushBack(&core);
			}

			core.breakApart();

			ConstraintInteraction* interaction = const_cast<ConstraintInteraction*>(sim->getInteraction());

			interaction->destroy();	// PT: this will call removeFromActiveBreakableList above

			// update related SIPs
			{
				ActorSim& a0 = interaction->getActorSim0();
				ActorSim& a1 = interaction->getActorSim1();
				ActorSim& actor = (a0.getActorInteractionCount() < a1.getActorInteractionCount()) ? a0 : a1;

				actor.setActorsInteractionsDirty(InteractionDirtyFlag::eFILTER_STATE, NULL, InteractionFlag::eRB_ELEMENT);
				// because broken constraints can re-enable contact response between the two bodies
			}

			PX_ASSERT(!sim->readFlag(ConstraintSim::eCHECK_MAX_FORCE_EXCEEDED));
		}
	}
}

// PT: finally mBrokenConstraints is parsed and callbacks issued:

void Sc::Scene::fireBrokenConstraintCallbacks()
{
	if(!mSimulationEventCallback)
		return;

	const PxU32 count = mBrokenConstraints.size();
	for(PxU32 i=0;i<count;i++)
	{
		Sc::ConstraintCore* c = mBrokenConstraints[i];
		PX_ASSERT(c->getSim());

		PxU32 typeID = 0xffffffff;
		void* externalRef = c->getPxConnector()->getExternalReference(typeID);
		PX_CHECK_MSG(typeID != 0xffffffff, "onConstraintBreak: Invalid constraint type ID.");

		PxConstraintInfo constraintInfo(c->getPxConstraint(), externalRef, typeID);
		mSimulationEventCallback->onConstraintBreak(&constraintInfo, 1);
	}
}
