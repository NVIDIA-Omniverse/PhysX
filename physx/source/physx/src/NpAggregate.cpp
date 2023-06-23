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

#include "NpAggregate.h"
#include "PxActor.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpActor.h"
#include "GuBVH.h"
#include "CmUtils.h"
#include "NpArticulationReducedCoordinate.h"

using namespace physx;
using namespace Gu;

namespace
{
#if PX_SUPPORT_PVD
	PX_FORCE_INLINE void PvdAttachActorToAggregate(NpAggregate* pAggregate, NpActor* pscActor)
	{
		NpScene* npScene = pAggregate->getNpScene();
		if(npScene/* && scScene->getScenePvdClient().isInstanceValid(pAggregate)*/)
			npScene->getScenePvdClientInternal().attachAggregateActor( pAggregate, pscActor );
	}

	PX_FORCE_INLINE void PvdDetachActorFromAggregate(NpAggregate* pAggregate, NpActor* pscActor)
	{
		NpScene* npScene = pAggregate->getNpScene();
		if(npScene/*&& scScene->getScenePvdClient().isInstanceValid(pAggregate)*/)
			npScene->getScenePvdClientInternal().detachAggregateActor( pAggregate, pscActor );
	}

	PX_FORCE_INLINE void PvdUpdateProperties(NpAggregate* pAggregate)
	{
		NpScene* npScene = pAggregate->getNpScene();
		if(npScene /*&& scScene->getScenePvdClient().isInstanceValid(pAggregate)*/)
			npScene->getScenePvdClientInternal().updatePvdProperties( pAggregate );
	}
#else
	#define PvdAttachActorToAggregate(aggregate, scActor)	{}
	#define PvdDetachActorFromAggregate(aggregate, scActor)	{}
	#define PvdUpdateProperties(aggregate)					{}
#endif
}

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

PX_FORCE_INLINE void setAggregate(NpAggregate* aggregate, PxActor& actor)
{
	NpActor& np = NpActor::getFromPxActor(actor);
	np.setAggregate(aggregate, actor);
}

///////////////////////////////////////////////////////////////////////////////

NpAggregate::NpAggregate(PxU32 maxActors, PxU32 maxShapes, PxAggregateFilterHint filterHint) :
	PxAggregate		(PxConcreteType::eAGGREGATE, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE),
	NpBase			(NpType::eAGGREGATE),
	mAggregateID	(PX_INVALID_U32),
	mMaxNbActors	(maxActors),
	mMaxNbShapes	(maxShapes),
	mFilterHint		(filterHint),
	mNbActors		(0),
	mNbShapes		(0)
{
	mActors = PX_ALLOCATE(PxActor*, maxActors, "PxActor*");
}

NpAggregate::~NpAggregate()
{
	NpFactory::getInstance().onAggregateRelease(this);
	if(getBaseFlags()&PxBaseFlag::eOWNS_MEMORY)
		PX_FREE(mActors);
}

void NpAggregate::scAddActor(NpActor& actor)
{
	PX_ASSERT(!isAPIWriteForbidden());

	actor.getActorCore().setAggregateID(mAggregateID);
	PvdAttachActorToAggregate( this, &actor );
	PvdUpdateProperties( this );
}

void NpAggregate::scRemoveActor(NpActor& actor, bool reinsert)
{
	PX_ASSERT(!isAPIWriteForbidden());

	Sc::ActorCore& ac = actor.getActorCore();
	ac.setAggregateID(PX_INVALID_U32);
		
	if(getNpScene() && reinsert)
		ac.reinsertShapes();

	//Update pvd status
	PvdDetachActorFromAggregate( this, &actor );
	PvdUpdateProperties( this );
}

void NpAggregate::removeAndReinsert(PxActor& actor, bool reinsert)
{
	NpActor& np = NpActor::getFromPxActor(actor);

	np.setAggregate(NULL, actor);
	
	scRemoveActor(np, reinsert);
}

void NpAggregate::release()
{
	NpScene* s = getNpScene();
	NP_WRITE_CHECK(s);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(s, "PxAggregate::release() not allowed while simulation is running. Call will be ignored.")

	PX_SIMD_GUARD;

	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, NULL);

	// "An aggregate should be empty when it gets released. If it isn't, the behavior should be: remove the actors from
	// the aggregate, then remove the aggregate from the scene (if any) then delete it. I guess that implies the actors
	// get individually reinserted into the broad phase if the aggregate is in a scene."
	for(PxU32 i=0;i<mNbActors;i++)
	{
		if (mActors[i]->getType() == PxActorType::eARTICULATION_LINK)
		{
			NpArticulationLink* link = static_cast<NpArticulationLink*>(mActors[i]);
			NpArticulationReducedCoordinate& articulation = static_cast<NpArticulationReducedCoordinate&>(link->getRoot());
			articulation.setAggregate(NULL);
		}

		removeAndReinsert(*mActors[i], true);
	}

	if(s)
	{
		s->scRemoveAggregate(*this);
		s->removeFromAggregateList(*this);
	}

	NpDestroyAggregate(this);
}

void NpAggregate::addActorInternal(PxActor& actor, NpScene& s, const PxBVH* bvh)
{
	if (actor.getType() != PxActorType::eARTICULATION_LINK)
	{
		NpActor& np = NpActor::getFromPxActor(actor);

		scAddActor(np);

		s.addActorInternal(actor, bvh);
	}
	else if (!actor.getScene())  // This check makes sure that a link of an articulation gets only added once.
	{
		NpArticulationLink& al = static_cast<NpArticulationLink&>(actor);
		PxArticulationReducedCoordinate& npArt = al.getRoot();
		for(PxU32 i=0; i < npArt.getNbLinks(); i++)
		{
			PxArticulationLink* link;
			npArt.getLinks(&link, 1, i);
			scAddActor(*static_cast<NpArticulationLink*>(link));
		}

		s.addArticulationInternal(npArt);
	}
}

void NpAggregate::addToScene(NpScene& scene)
{
	const PxU32 nb = mNbActors;

	for(PxU32 i=0;i<nb;i++)
	{
		PX_ASSERT(mActors[i]);
		PxActor& actor = *mActors[i];

		//A.B. check if a bvh was connected to that actor, we will use it for the insert and remove it
		NpActor& npActor = NpActor::getFromPxActor(actor);
		BVH* bvh = NULL;			
		if(npActor.getConnectors<BVH>(NpConnectorType::eBvh, &bvh, 1))
			npActor.removeConnector(actor, NpConnectorType::eBvh, bvh, "PxBVH connector could not have been removed!");				

		addActorInternal(actor, scene, bvh);

		// if a bvh was used dec ref count, we increased the ref count when adding the actor connection
		if(bvh)
			bvh->decRefCount();
	}
}

bool NpAggregate::addActor(PxActor& actor, const PxBVH* bvh)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(npScene, "PxAggregate::addActor() not allowed while simulation is running. Call will be ignored.", false);

	PX_SIMD_GUARD;

	if(mNbActors==mMaxNbActors)
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add actor to aggregate, max number of actors reached");

	PxRigidActor* rigidActor = actor.is<PxRigidActor>();

	PxU32 numShapes = 0;
	if(rigidActor)
	{
		numShapes = rigidActor->getNbShapes();
		if ((mNbShapes + numShapes) > mMaxNbShapes)
			return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add actor to aggregate, max number of shapes reached");
	}

	if(actor.getAggregate())
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add actor to aggregate, actor already belongs to an aggregate");

	if(actor.getScene())
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add actor to aggregate, actor already belongs to a scene");

	const PxType ctype = actor.getConcreteType();
	if(ctype == PxConcreteType::eARTICULATION_LINK)
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add articulation link to aggregate, only whole articulations can be added");

	if(PxGetAggregateType(mFilterHint)==PxAggregateType::eSTATIC && ctype != PxConcreteType::eRIGID_STATIC)
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add non-static actor to static aggregate");

	if(PxGetAggregateType(mFilterHint)==PxAggregateType::eKINEMATIC)
	{	
		bool isKine = false;
		if(ctype == PxConcreteType::eRIGID_DYNAMIC)
		{
			PxRigidDynamic& dyna = static_cast<PxRigidDynamic&>(actor);
			isKine = dyna.getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC);
		}
		if(!isKine)
			return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add non-kinematic actor to kinematic aggregate");
	}

	setAggregate(this, actor);

	mActors[mNbActors++] = &actor;

	mNbShapes += numShapes;

	OMNI_PVD_ADD(PxAggregate, actors, static_cast<PxAggregate&>(*this), actor);

	// PT: when an object is added to a aggregate at runtime, i.e. when the aggregate has already been added to the scene,
	// we need to immediately add the newcomer to the scene as well.
	if(npScene)
	{
		addActorInternal(actor, *npScene, bvh);
	}
	else
	{
		// A.B. if BVH is provided we need to keep it stored till the aggregate is inserted into a scene
		if(bvh)
		{
			PxBVH* bvhMutable = const_cast<PxBVH*>(bvh);
			static_cast<BVH*>(bvhMutable)->incRefCount();
			NpActor::getFromPxActor(actor).addConnector(NpConnectorType::eBvh, bvhMutable, "PxBVH already added to the PxActor!");
		}
	}
	return true;
}

bool NpAggregate::removeActorAndReinsert(PxActor& actor, bool reinsert)
{
	for(PxU32 i=0;i<mNbActors;i++)
	{
		if(mActors[i]==&actor)
		{
			PxRigidActor* rigidActor = actor.is<PxRigidActor>();
			if(rigidActor)
				mNbShapes -= rigidActor->getNbShapes();
			
			mActors[i] = mActors[--mNbActors];
			removeAndReinsert(actor, reinsert);

			return true;
		}
	}
	return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't remove actor, actor doesn't belong to aggregate");
}

bool NpAggregate::removeActor(PxActor& actor)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(npScene, "PxAggregate::removeActor() not allowed while simulation is running. Call will be ignored.", false);

	PX_SIMD_GUARD;

	if(actor.getType() == PxActorType::eARTICULATION_LINK)
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't remove articulation link, only whole articulations can be removed");

	// A.B. remove the BVH reference if there is and the aggregate was not added to a scene
	if(!npScene)
	{
		NpActor& np = NpActor::getFromPxActor(actor);
		BVH* bvh = NULL;
		if(np.getConnectors<BVH>(NpConnectorType::eBvh, &bvh, 1))
		{
			np.removeConnector(actor, NpConnectorType::eBvh, bvh, "PxBVH connector could not have been removed!");
			bvh->decRefCount();
		}
	}

	OMNI_PVD_REMOVE(PxAggregate, actors, static_cast<PxAggregate&>(*this), actor);

	// PT: there are really 2 cases here:
	// a) the user just wants to remove the actor from the aggregate, but the actor is still alive so if the aggregate has been added to a scene,
	//    we must reinsert the removed actor to that same scene
	// b) this is called by the framework when releasing an actor, in which case we don't want to reinsert it anywhere.
	//
	// We assume that when called by the user, we always want to reinsert. The framework however will call the internal function
	// without reinsertion.
	return removeActorAndReinsert(actor, true);
}

bool NpAggregate::addArticulation(PxArticulationReducedCoordinate& art)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(npScene, "PxAggregate::addArticulation() not allowed while simulation is running. Call will be ignored.", false);

	PX_SIMD_GUARD;

	if((mNbActors+art.getNbLinks()) > mMaxNbActors)
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add articulation links, max number of actors reached");

	const PxU32 numShapes = art.getNbShapes();
	if((mNbShapes + numShapes) > mMaxNbShapes)
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add articulation, max number of shapes reached");

	if(art.getAggregate())
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add articulation to aggregate, articulation already belongs to an aggregate");

	if(art.getScene())
		return outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't add articulation to aggregate, articulation already belongs to a scene");

	NpArticulationReducedCoordinate* impl = static_cast<NpArticulationReducedCoordinate*>(&art);
	impl->setAggregate(this);
	NpArticulationLink* const* links = impl->getLinks();

	for(PxU32 i=0; i < impl->getNbLinks(); i++)
	{
		NpArticulationLink& l = *links[i];

		setAggregate(this, l);

		mActors[mNbActors++] = &l;

		scAddActor(l);
	}

	mNbShapes += numShapes;

	// PT: when an object is added to a aggregate at runtime, i.e. when the aggregate has already been added to the scene,
	// we need to immediately add the newcomer to the scene as well.
	if(npScene)
		npScene->addArticulationInternal(art);

	return true;
}

bool NpAggregate::removeArticulationAndReinsert(PxArticulationReducedCoordinate& art, bool reinsert)
{
	bool found = false;
	PxU32 idx = 0;
	while(idx < mNbActors)
	{
		if ((mActors[idx]->getType() == PxActorType::eARTICULATION_LINK) && (&static_cast<NpArticulationLink*>(mActors[idx])->getRoot() == &art))
		{
			PxActor* a = mActors[idx];
			mNbShapes -= static_cast<NpArticulationLink*>(mActors[idx])->getNbShapes();
			mActors[idx] = mActors[--mNbActors];
			removeAndReinsert(*a, reinsert);
			found = true;
		}
		else
			idx++;
	}

	static_cast<NpArticulationReducedCoordinate&>(art).setAggregate(NULL);

	if(!found)
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxAggregate: can't remove articulation, articulation doesn't belong to aggregate");
	return found;
}

bool NpAggregate::removeArticulation(PxArticulationReducedCoordinate& art)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_AND_RETURN_VAL(npScene, "PxAggregate::removeArticulation() not allowed while simulation is running. Call will be ignored.", false);

	PX_SIMD_GUARD;

	// see comments in removeActor()
	return removeArticulationAndReinsert(art, true);
}

PxU32 NpAggregate::getNbActors() const
{
	NP_READ_CHECK(getNpScene());
	return mNbActors;
}

PxU32 NpAggregate::getMaxNbActors() const
{
	NP_READ_CHECK(getNpScene());
	return mMaxNbActors;
}

PxU32 NpAggregate::getMaxNbShapes() const
{
	NP_READ_CHECK(getNpScene());
	return mMaxNbShapes;
}

PxU32 NpAggregate::getActors(PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getNpScene());
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mActors, getCurrentSizeFast());
}

PxScene* NpAggregate::getScene()
{
	return getNpScene();
}

bool NpAggregate::getSelfCollision() const
{
	NP_READ_CHECK(getNpScene());
	return getSelfCollideFast();
}

// PX_SERIALIZATION

void NpAggregate::preExportDataReset()
{
	mAggregateID = PX_INVALID_U32;
}

void NpAggregate::exportExtraData(PxSerializationContext& stream)
{
	if(mActors)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mActors, mNbActors * sizeof(PxActor*));
	}
}

void NpAggregate::importExtraData(PxDeserializationContext& context)
{
	if(mActors)
		mActors = context.readExtraData<PxActor*, PX_SERIAL_ALIGN>(mNbActors);
}

void NpAggregate::resolveReferences(PxDeserializationContext& context)
{
	// Resolve actor pointers if needed
	for(PxU32 i=0; i < mNbActors; i++)
	{
		context.translatePxBase(mActors[i]);
		{
			//update aggregate if mActors is in external reference
			NpActor& np = NpActor::getFromPxActor(*mActors[i]);
			if(np.getAggregate() == NULL)
			{
				np.setAggregate(this, *mActors[i]);
			}
			if(mActors[i]->getType() == PxActorType::eARTICULATION_LINK)
			{
				PxArticulationReducedCoordinate& articulation = static_cast<NpArticulationLink*>(mActors[i])->getRoot();
				if(!articulation.getAggregate())
					static_cast<NpArticulationReducedCoordinate&>(articulation).setAggregate(this);
			}
		}
	}	
}

NpAggregate* NpAggregate::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpAggregate* obj = PX_PLACEMENT_NEW(address, NpAggregate(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(NpAggregate);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void NpAggregate::requiresObjects(PxProcessPxBaseCallback& c)
{
	for(PxU32 i=0; i < mNbActors; i++)
	{
		PxArticulationLink* link = mActors[i]->is<PxArticulationLink>();
		if(link)
			c.process(link->getArticulation());
		else
			c.process(*mActors[i]);
	}
}
// ~PX_SERIALIZATION

void NpAggregate::incShapeCount()
{
	if(mNbShapes == mMaxNbShapes)
		outputError<PxErrorCode::eDEBUG_WARNING>(__LINE__, "PxRigidActor::attachShape: Actor is part of an aggregate and max number of shapes reached!");

	mNbShapes++;
}

void NpAggregate::decShapeCount()
{
	PX_ASSERT(mNbShapes > 0);
	mNbShapes--;
}
