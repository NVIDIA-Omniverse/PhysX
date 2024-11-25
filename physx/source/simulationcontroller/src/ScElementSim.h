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

#ifndef SC_ELEMENT_SIM_H
#define SC_ELEMENT_SIM_H

#include "PxFiltering.h"
#include "PxvConfig.h"
#include "ScActorSim.h"
#include "ScInteraction.h"
#include "BpAABBManager.h"
#include "ScObjectIDTracker.h"
#include "ScScene.h"

namespace physx
{
namespace Sc
{
	class ElementSimInteraction;

	// A ElementSim is a part of a ActorSim. It contributes to the activation framework by adding its interactions to the actor.
	class ElementSim
	{
		PX_NOCOPY(ElementSim)

	public:
		class ElementInteractionIterator
		{
			public:
				PX_FORCE_INLINE			ElementInteractionIterator(const ElementSim& e, PxU32 nbInteractions, Interaction** interactions) :
					mInteractions(interactions), mInteractionsLast(interactions + nbInteractions), mElement(&e) {}
				ElementSimInteraction*	getNext();

			private:
				Interaction**					mInteractions;
				Interaction**					mInteractionsLast;
				const ElementSim*				mElement;
		};

		class ElementInteractionReverseIterator
		{
			public:
				PX_FORCE_INLINE			ElementInteractionReverseIterator(const ElementSim& e, PxU32 nbInteractions, Interaction** interactions) :
					mInteractions(interactions), mInteractionsLast(interactions + nbInteractions), mElement(&e) {}
				ElementSimInteraction*	getNext();

			private:
				Interaction**					mInteractions;
				Interaction**					mInteractionsLast;
				const ElementSim*				mElement;
		};

												ElementSim(ActorSim& actor);
		protected:
												~ElementSim();
		public:

		// Get an iterator to the interactions connected to the element
		// PT: this may seem strange at first glance since the "element interactions" appear to use the "actor interactions". The thing that makes this work is hidden
		// inside the iterator implementation: it does parse all the actor interactions indeed, but filters out the ones that do not contain "this", i.e. the desired element.
		// So this is inefficient (parsing potentially many more interactions than needed, imagine in a large compound) but it works, and the iterator has a point - it isn't
		// just the same as parsing the actor's array.
		PX_FORCE_INLINE	ElementInteractionIterator			getElemInteractions()			const	{ return ElementInteractionIterator(*this, mActor.getActorInteractionCount(), mActor.getActorInteractions());			}
		PX_FORCE_INLINE	ElementInteractionReverseIterator	getElemInteractionsReverse()	const	{ return ElementInteractionReverseIterator(*this, mActor.getActorInteractionCount(), mActor.getActorInteractions());	}

		PX_FORCE_INLINE	ActorSim&				getActor()					const	{ return mActor; }

		PX_FORCE_INLINE	Scene&					getScene()					const	{ return mActor.getScene();	}

		PX_FORCE_INLINE PxU32					getElementID()				const	{ return mElementID;	}
		PX_FORCE_INLINE bool					isInBroadPhase()			const	{ return mInBroadPhase;	}

						void					addToAABBMgr(PxReal contactDistance, Bp::FilterGroup::Enum group, Bp::ElementType::Enum type);
		PX_FORCE_INLINE	void					addToAABBMgr(PxReal contactOffset, Bp::FilterType::Enum type)
												{
													const PxU32 group = Bp::FilterGroup::eDYNAMICS_BASE + mActor.getActorID();
													addToAABBMgr(contactOffset, Bp::FilterGroup::Enum((group << BP_FILTERING_TYPE_SHIFT_BIT) | type), Bp::ElementType::eSHAPE);
												}

						bool					removeFromAABBMgr();

		PX_FORCE_INLINE	void					initID()
												{
													Scene& scene = getScene();
													mElementID = scene.getElementIDPool().createID();
													scene.getBoundsArray().initEntry(mElementID);
												}

		PX_FORCE_INLINE	void					releaseID()
												{
													getScene().getElementIDPool().releaseID(mElementID);
												}
	protected:
						ActorSim&				mActor;

						PxU32					mElementID : 31;	// PT: ID provided by Sc::Scene::mElementIDPool
						PxU32					mInBroadPhase : 1;
	public:
						PxU32					mShapeArrayIndex;
	};

	PX_FORCE_INLINE void setFilterObjectAttributeType(PxFilterObjectAttributes& attr, PxFilterObjectType::Enum type)
	{
		PX_ASSERT((attr & (PxFilterObjectType::eMAX_TYPE_COUNT-1)) == 0);
		attr |= type;
	}
} // namespace Sc
}

#endif
