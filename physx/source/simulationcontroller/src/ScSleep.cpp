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
#include "ScArticulationSim.h"
#include "ScBodySim.h"
#include "common/PxProfileZone.h"

#include "ScActorSim.h"
#include "ScArticulationSim.h"

#if PX_SUPPORT_GPU_PHYSX
	#include "ScSoftBodySim.h"
	#include "ScFEMClothSim.h"
	#include "ScParticleSystemSim.h"
	#include "ScHairSystemSim.h"
#endif

using namespace physx;
using namespace physx::Cm;
using namespace physx::Dy;
using namespace Sc;

// PT: "setActive()" moved from ActorSim to BodySim because GPU classes silently re-implement this in a very different way (see below),
// i.e. it defeats the purpose of the virtual activate()/deactivate() functions.
void Sc::BodySim::setActive(bool active, bool asPartOfCreation)
{
	PX_ASSERT(!active || isDynamicRigid());  // Currently there should be no need to activate an actor that does not take part in island generation

	if(asPartOfCreation || isActive() != active)
	{
		PX_ASSERT(!asPartOfCreation || (getActorInteractionCount() == 0)); // On creation or destruction there should be no interactions

		if(active)
		{
			if(!asPartOfCreation)
				getScene().addToActiveList(*this);	// Inactive => Active

			activate();

			PX_ASSERT(asPartOfCreation || isActive());
		}
		else
		{
			if(!asPartOfCreation)
				getScene().removeFromActiveList(*this);	// Active => Inactive

			deactivate();

			PX_ASSERT(asPartOfCreation || (!isActive()));
		}
	}
}

void Sc::ArticulationSim::setActive(bool b, bool asPartOfCreation)
{
	const PxReal wakeCounter = mCore.getWakeCounter();
	const PxU32 nbBodies = mBodies.size();
	for(PxU32 i=0;i<nbBodies;i++)
	{
		if(i+1 < nbBodies)
		{
			PxPrefetchLine(mBodies[i+1],0);
			PxPrefetchLine(mBodies[i+1],128);
		}
		//KS - force in the wake counter from the articulation to its links. This is required because
		//GPU articulation simulation does not DMA back wake counters for each link - it just brings back a global wake counter
		mBodies[i]->getBodyCore().setWakeCounterFromSim(wakeCounter);
		mBodies[i]->setActive(b, asPartOfCreation);
	}
}

// PT: moving all the sleeping-related implementations to the same file clearly exposes the inconsistencies between them
#if PX_SUPPORT_GPU_PHYSX
void Sc::ParticleSystemSim::setActive(bool /*active*/, bool /*asPartOfCreation*/)
{
}

void Sc::FEMClothSim::activate()
{
	mScene.getSimulationController()->activateCloth(mLLFEMCloth);

	activateInteractions(*this);
}

void Sc::FEMClothSim::deactivate()
{
	mScene.getSimulationController()->deactivateCloth(mLLFEMCloth);

	deactivateInteractions(*this);
}

void Sc::FEMClothSim::setActive(bool active, bool /*asPartOfCreation*/)
{
	if(active)
		activate();
	else
		deactivate();
}

void Sc::SoftBodySim::setActive(bool active, bool /*asPartOfCreation*/)
{
	if(active)
		getScene().getSimulationController()->activateSoftbody(mLLSoftBody);
	else
		getScene().getSimulationController()->deactivateSoftbody(mLLSoftBody);
}

void Sc::HairSystemSim::setActive(bool active, bool /*asPartOfCreation*/)
{
	if(active)
		getScene().getSimulationController()->activateHairSystem(mLLHairSystem);
	else
		getScene().getSimulationController()->deactivateHairSystem(mLLHairSystem);
}
#endif

namespace
{
struct GetRigidSim		{ static PX_FORCE_INLINE BodySim* getSim(const IG::Node& node)			{ return reinterpret_cast<BodySim*>(reinterpret_cast<PxU8*>(node.mRigidBody) - BodySim::getRigidBodyOffset());	}	};
struct GetArticSim		{ static PX_FORCE_INLINE ArticulationSim* getSim(const IG::Node& node)	{ return reinterpret_cast<ArticulationSim*>(node.mLLArticulation->getUserData());								}	};
#if PX_SUPPORT_GPU_PHYSX
struct GetSoftBodySim	{ static PX_FORCE_INLINE SoftBodySim* getSim(const IG::Node& node)		{ return node.mLLSoftBody->getSoftBodySim();																	}	};
struct GetFEMClothSim	{ static PX_FORCE_INLINE FEMClothSim* getSim(const IG::Node& node)		{ return node.mLLFEMCloth->getFEMClothSim();																	}	};
struct GetHairSystemSim	{ static PX_FORCE_INLINE HairSystemSim* getSim(const IG::Node& node)	{ return node.mLLHairSystem->getHairSystemSim();																}	};
#endif
}

template<class SimT, class SimAccessT, const bool active>
static void setActive(PxU32& nbModified, const IG::IslandSim& islandSim, IG::Node::NodeType type)
{
	PxU32 nbToProcess = active ? islandSim.getNbNodesToActivate(type) : islandSim.getNbNodesToDeactivate(type);
	const PxNodeIndex* indices = active ? islandSim.getNodesToActivate(type) : islandSim.getNodesToDeactivate(type);

	while(nbToProcess--)
	{
		const IG::Node& node = islandSim.getNode(*indices++);
		PX_ASSERT(node.mType == type);
		if(node.isActive()==active)
		{
			SimT* sim = SimAccessT::getSim(node);
			if(sim)
			{
				sim->setActive(active);
				nbModified++;
			}
		}
	}
}

#ifdef BATCHED
namespace
{
struct SetActiveRigidSim
{
	template<const bool active>
	static void setActive(Scene& /*scene*/, PxU32 nbObjects, BodySim** objects)
	{
		if(1)
		{
			while(nbObjects--)
			{
				(*objects)->setActive(active);
				objects++;
			}
		}
		else
		{
			if(active)
			{
//				scene.addToActiveList(*this);
//				activate();
//				PX_ASSERT(isActive());
			}
			else
			{
//				scene.removeFromActiveList(*this);
//				deactivate();
//				PX_ASSERT(!isActive());
			}
		}
	}
};
}

template<class SimT, class SimAccessT, class SetActiveBatchedT, const bool active>
static void setActiveBatched(Scene& scene, PxU32& nbModified, const IG::IslandSim& islandSim, IG::Node::NodeType type)
{
	PxU32 nbToProcess = active ? islandSim.getNbNodesToActivate(type) : islandSim.getNbNodesToDeactivate(type);
	const PxNodeIndex* indices = active ? islandSim.getNodesToActivate(type) : islandSim.getNodesToDeactivate(type);

	PX_ALLOCA(batch, SimT*, nbToProcess);

	PxU32 nb = 0;
	while(nbToProcess--)
	{
		const IG::Node& node = islandSim.getNode(*indices++);
		PX_ASSERT(node.mType == type);
		if(node.isActive()==active)
		{
			SimT* sim = SimAccessT::getSim(node);
			if(sim && sim->isActive()!=active)
				batch.mPointer[nb++] = sim;
		}
	}

	SetActiveBatchedT::setActive<active>(scene, nb, batch.mPointer);

	nbModified = nb;
}

/*
Batched version would be just:
a) addToActiveList(batched objects)
b) activate(batched objects)

void Sc::ActorSim::setActive(bool active)
{
	PX_ASSERT(!active || isDynamicRigid());  // Currently there should be no need to activate an actor that does not take part in island generation

	if(isActive() != active)
	{
		if(active)
		{
			// Inactive => Active
			getScene().addToActiveList(*this);

			activate();

			PX_ASSERT(isActive());
		}
		else
		{
			// Active => Inactive
			getScene().removeFromActiveList(*this);

			deactivate();

			PX_ASSERT(!isActive());
		}
	}
}
*/
#endif

void Sc::Scene::putObjectsToSleep()
{
	PX_PROFILE_ZONE("Sc::Scene::putObjectsToSleep", mContextId);

	//Set to sleep all bodies that were in awake islands that have just been put to sleep.

	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	PxU32 nbBodiesDeactivated = 0;
	//setActiveBatched<BodySim, GetRigidSim, SetActiveRigidSim, false>(*this, nbBodiesDeactivated, islandSim, IG::Node::eRIGID_BODY_TYPE);
	setActive<BodySim, GetRigidSim, false>(nbBodiesDeactivated, islandSim, IG::Node::eRIGID_BODY_TYPE);
	setActive<ArticulationSim, GetArticSim, false>(nbBodiesDeactivated, islandSim, IG::Node::eARTICULATION_TYPE);

#if PX_SUPPORT_GPU_PHYSX
	setActive<SoftBodySim, GetSoftBodySim, false>(nbBodiesDeactivated, islandSim, IG::Node::eSOFTBODY_TYPE);
	setActive<FEMClothSim, GetFEMClothSim, false>(nbBodiesDeactivated, islandSim, IG::Node::eFEMCLOTH_TYPE);
	setActive<HairSystemSim, GetHairSystemSim, false>(nbBodiesDeactivated, islandSim, IG::Node::eHAIRSYSTEM_TYPE);
#endif

	if(nbBodiesDeactivated)
		mDynamicsContext->setStateDirty(true);
}

void Sc::Scene::wakeObjectsUp()
{
	PX_PROFILE_ZONE("Sc::Scene::wakeObjectsUp", mContextId);

	//Wake up all bodies that were in sleeping islands that have just been hit by a moving object.

	const IG::IslandSim& islandSim = mSimpleIslandManager->getAccurateIslandSim();

	PxU32 nbBodiesWoken = 0;
	setActive<BodySim, GetRigidSim, true>(nbBodiesWoken, islandSim, IG::Node::eRIGID_BODY_TYPE);
	setActive<ArticulationSim, GetArticSim, true>(nbBodiesWoken, islandSim, IG::Node::eARTICULATION_TYPE);

#if PX_SUPPORT_GPU_PHYSX
	setActive<SoftBodySim, GetSoftBodySim, true>(nbBodiesWoken, islandSim, IG::Node::eSOFTBODY_TYPE);
	setActive<FEMClothSim, GetFEMClothSim, true>(nbBodiesWoken, islandSim, IG::Node::eFEMCLOTH_TYPE);
	setActive<HairSystemSim, GetHairSystemSim, true>(nbBodiesWoken, islandSim, IG::Node::eHAIRSYSTEM_TYPE);
#endif

	if(nbBodiesWoken)
		mDynamicsContext->setStateDirty(true);
}

