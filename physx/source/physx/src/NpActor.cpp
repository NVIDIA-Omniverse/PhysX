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

#include "NpActor.h"
#include "PxRigidActor.h"
#include "NpConstraint.h"
#include "NpFactory.h"
#include "NpShape.h"
#include "NpPhysics.h"
#include "NpAggregate.h"
#include "NpScene.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationLink.h"
#include "CmTransformUtils.h"

#if PX_SUPPORT_GPU_PHYSX
#include "NpSoftBody.h"
#include "NpParticleSystem.h"

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "NpHairSystem.h"
#include "NpFEMCloth.h"
#endif
#endif

using namespace physx;

///////////////////////////////////////////////////////////////////////////////

const Sc::BodyCore* physx::getBodyCore(const PxRigidActor* actor)
{
	const Sc::BodyCore* core = NULL;
	if(actor)
	{
		const PxType type = actor->getConcreteType();
		if(type == PxConcreteType::eRIGID_DYNAMIC)
		{
			const NpRigidDynamic* dyn = static_cast<const NpRigidDynamic*>(actor);
			core = &dyn->getCore();
		}
		else if(type == PxConcreteType::eARTICULATION_LINK)
		{
			const NpArticulationLink* link = static_cast<const NpArticulationLink*>(actor);
			core = &link->getCore();
		}
	}
	return core;
}

///////////////////////////////////////////////////////////////////////////////

NpActor::NpActor(NpType::Enum type) :
	NpBase			(type),
	mName			(NULL),
	mConnectorArray	(NULL)
{
}

typedef PxHashMap<NpActor*, NpConnectorArray*> ConnectorMap;
struct NpActorUserData
{
	PxU32			referenceCount;
	ConnectorMap*	tmpOriginalConnectors;
};

void NpActor::exportExtraData(PxSerializationContext& stream)
{
	const PxCollection& collection = stream.getCollection();
	if(mConnectorArray)
	{		
		PxU32 connectorSize = mConnectorArray->size();	
		PxU32 missedCount = 0;
		for(PxU32 i = 0; i < connectorSize; ++i)
		{
			NpConnector& c = (*mConnectorArray)[i];
			PxBase* object = c.mObject;
			if(!collection.contains(*object))
			{
				++missedCount;
			}
		}
		
		NpConnectorArray* exportConnectorArray = mConnectorArray;

		if(missedCount > 0)
		{
			exportConnectorArray = NpFactory::getInstance().acquireConnectorArray();
			if(missedCount < connectorSize)
			{
				exportConnectorArray->reserve(connectorSize - missedCount);	
				for(PxU32 i = 0; i < connectorSize; ++i)
				{
					NpConnector& c = (*mConnectorArray)[i];
					PxBase* object = c.mObject;
					if(collection.contains(*object))
					{
						exportConnectorArray->pushBack(c);
					}
				}
			}			
		}

		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(exportConnectorArray, sizeof(NpConnectorArray));
		Cm::exportInlineArray(*exportConnectorArray, stream);

		if(missedCount > 0)
			 NpFactory::getInstance().releaseConnectorArray(exportConnectorArray);
	}
	stream.writeName(mName);
}

void NpActor::importExtraData(PxDeserializationContext& context)
{
	if(mConnectorArray)
	{
		mConnectorArray = context.readExtraData<NpConnectorArray, PX_SERIAL_ALIGN>();
		PX_PLACEMENT_NEW(mConnectorArray, NpConnectorArray(PxEmpty));

		if(mConnectorArray->size() == 0)
			mConnectorArray = NULL;
		else
			Cm::importInlineArray(*mConnectorArray, context);
	}
	context.readName(mName);
}

void NpActor::resolveReferences(PxDeserializationContext& context)
{
	// Resolve connector pointers if needed
	if(mConnectorArray)
	{
		const PxU32 nbConnectors = mConnectorArray->size();
		for(PxU32 i=0; i<nbConnectors; i++)
		{
			NpConnector& c = (*mConnectorArray)[i];
			context.translatePxBase(c.mObject);
		}
	}	
}

///////////////////////////////////////////////////////////////////////////////

void NpActor::removeConstraints(PxRigidActor& owner)
{
	if(mConnectorArray)
	{
		PxU32 nbConnectors = mConnectorArray->size();
		PxU32 currentIndex = 0;
		while(nbConnectors--)
		{
			NpConnector& connector = (*mConnectorArray)[currentIndex];
			if (connector.mType == NpConnectorType::eConstraint)
			{
				NpConstraint* c = static_cast<NpConstraint*>(connector.mObject);
				c->actorDeleted(&owner);

				NpScene* s = c->getNpScene();
				if(s)
					s->removeFromConstraintList(*c);

				removeConnector(owner, currentIndex);
			}
			else
				currentIndex++;
		}
	}
}

void NpActor::removeFromAggregate(PxActor& owner)
{
	if(mConnectorArray)  // Need to test again because the code above might purge the connector array if no element remains
	{
		PX_ASSERT(mConnectorArray->size() == 1);  // At this point only the aggregate should remain
		PX_ASSERT((*mConnectorArray)[0].mType == NpConnectorType::eAggregate);

		NpAggregate* a = static_cast<NpAggregate*>((*mConnectorArray)[0].mObject);
		bool status = a->removeActorAndReinsert(owner, false);
		PX_ASSERT(status);
		PX_UNUSED(status);
		PX_ASSERT(!mConnectorArray);  // Remove should happen in aggregate code
	}

	PX_ASSERT(!mConnectorArray);  // All the connector objects should have been removed at this point
}

///////////////////////////////////////////////////////////////////////////////

PxU32 NpActor::findConnector(NpConnectorType::Enum type, PxBase* object) const
{
	if(!mConnectorArray)
		return 0xffffffff;

	for(PxU32 i=0; i < mConnectorArray->size(); i++)
	{
		NpConnector& c = (*mConnectorArray)[i];
		if (c.mType == type && c.mObject == object)
			return i;
	}

	return 0xffffffff;
}

void NpActor::addConnector(NpConnectorType::Enum type, PxBase* object, const char* errMsg)
{
	if(!mConnectorArray)
		mConnectorArray = NpFactory::getInstance().acquireConnectorArray();

	PX_CHECK_MSG(findConnector(type, object) == 0xffffffff, errMsg);
	PX_UNUSED(errMsg);
		
	if(mConnectorArray->isInUserMemory() && mConnectorArray->size() == mConnectorArray->capacity())
	{		
		NpConnectorArray* newConnectorArray = NpFactory::getInstance().acquireConnectorArray();		
		newConnectorArray->assign(mConnectorArray->begin(), mConnectorArray->end());
		mConnectorArray->~NpConnectorArray();
		mConnectorArray = newConnectorArray;
	}

	NpConnector c(type, object);
	mConnectorArray->pushBack(c);
}

void NpActor::removeConnector(PxActor& /*owner*/, PxU32 index)
{
	PX_ASSERT(mConnectorArray);
	PX_ASSERT(index < mConnectorArray->size());
	
	mConnectorArray->replaceWithLast(index);

	if(mConnectorArray->size() == 0)
	{
		if(!mConnectorArray->isInUserMemory())
			NpFactory::getInstance().releaseConnectorArray(mConnectorArray);
		else
			mConnectorArray->~NpConnectorArray();
		mConnectorArray = NULL;
	}
}

void NpActor::removeConnector(PxActor& owner, NpConnectorType::Enum type, PxBase* object, const char* errorMsg)
{
	PX_CHECK_MSG(mConnectorArray, errorMsg);
	PX_UNUSED(errorMsg);

	if(mConnectorArray)
	{
		PxU32 index = findConnector(type, object);

		PX_CHECK_MSG(index != 0xffffffff, errorMsg);

		removeConnector(owner, index);
	}
}

PxU32 NpActor::getNbConnectors(NpConnectorType::Enum type) const
{
	PxU32 nbConnectors = 0;

	if(mConnectorArray)
	{
		for(PxU32 i=0; i < mConnectorArray->size(); i++)
		{
			if ((*mConnectorArray)[i].mType == type)
				nbConnectors++;
		}
	}

	return nbConnectors;
}

///////////////////////////////////////////////////////////////////////////////

NpAggregate* NpActor::getNpAggregate(PxU32& index) const
{
	PX_ASSERT(getNbConnectors(NpConnectorType::eAggregate) <= 1);

	if(mConnectorArray)
	{
		// PT: TODO: sort by type to optimize this...
		for(PxU32 i=0; i < mConnectorArray->size(); i++)
		{
			NpConnector& c = (*mConnectorArray)[i];
			if (c.mType == NpConnectorType::eAggregate)
			{
				index = i;
				return static_cast<NpAggregate*>(c.mObject);
			}
		}
	}

	return NULL;
}

void NpActor::setAggregate(NpAggregate* np, PxActor& owner)
{
	PxU32 index = 0xffffffff;
	NpAggregate* a = getNpAggregate(index);

	if (!a)
	{
		PX_ASSERT(np);
		addConnector(NpConnectorType::eAggregate, np, "NpActor::setAggregate() failed");
	}
	else
	{
		PX_ASSERT(mConnectorArray);
		PX_ASSERT(index != 0xffffffff);
		if (!np)
			removeConnector(owner, index);
		else
			(*mConnectorArray)[index].mObject = np;
	}
}

PxAggregate* NpActor::getAggregate()	const
{
	PxU32 index = 0xffffffff;
	NpAggregate* a = getNpAggregate(index);
	return static_cast<PxAggregate*>(a);
}

///////////////////////////////////////////////////////////////////////////////

void NpActor::removeConstraintsFromScene()
{
	NpConnectorIterator iter = getConnectorIterator(NpConnectorType::eConstraint);
	while (PxBase* ser = iter.getNext())
	{
		NpConstraint* c = static_cast<NpConstraint*>(ser);
			
		NpScene* s = c->getNpScene();
		if(s)
			s->removeFromConstraintList(*c);
	}
}

void NpActor::addConstraintsToSceneInternal()
{
	if(!mConnectorArray)
		return;

	NpConnectorIterator iter = getConnectorIterator(NpConnectorType::eConstraint);
	while (PxBase* ser = iter.getNext())
	{
		NpConstraint* c = static_cast<NpConstraint*>(ser);
		PX_ASSERT(c->getNpScene() == NULL);

		c->markDirty();	// PT: "temp" fix for crash when removing/re-adding jointed actor from/to a scene

		NpScene* s = c->getSceneFromActors();
		if(s)
			s->addToConstraintList(*c);
	}
}

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE const NpShapeManager* getShapeManager(const PxRigidActor& actor)
{
	// DS: if the performance here becomes an issue we can use the same kind of offset hack as below

	const PxType actorType = actor.getConcreteType();

	if (actorType == PxConcreteType::eRIGID_DYNAMIC)
		return &static_cast<const NpRigidDynamic&>(actor).getShapeManager();
	else if(actorType == PxConcreteType::eRIGID_STATIC)
		return &static_cast<const NpRigidStatic&>(actor).getShapeManager();
	else if (actorType == PxConcreteType::eARTICULATION_LINK)
		return &static_cast<const NpArticulationLink&>(actor).getShapeManager();
	else
	{
		PX_ASSERT(0);
		return NULL;
	}
}

NpShapeManager* NpActor::getShapeManager_(PxRigidActor& actor)				{ return const_cast<NpShapeManager*>(getShapeManager(actor));	}
const NpShapeManager* NpActor::getShapeManager_(const PxRigidActor& actor)	{ return getShapeManager(actor);								}

void NpActor::onActorRelease(PxActor* actor)
{
	NpFactory::getInstance().onActorRelease(actor);
}

void	NpActor::scSetDominanceGroup(PxDominanceGroup v)
		{
			PX_ASSERT(!isAPIWriteForbidden());
			getActorCore().setDominanceGroup(v);
			UPDATE_PVD_PROPERTY
			OMNI_PVD_SET(PxActor, dominance, *getPxActor(), v)
		}

void	NpActor::scSetOwnerClient(PxClientID inId)
		{
			//This call is only valid if we aren't in a scene.
			PX_ASSERT(!isAPIWriteForbidden());
			getActorCore().setOwnerClient(inId);
			UPDATE_PVD_PROPERTY
			OMNI_PVD_SET(PxActor, ownerClient, *getPxActor(), inId)
		}

const PxActor* NpActor::getPxActor() const
		{
			const PxActorType::Enum type = getActorCore().getActorCoreType();
			if(type == PxActorType::eRIGID_DYNAMIC)
				return static_cast<const NpRigidDynamic*>(this);
			else if(type == PxActorType::eRIGID_STATIC)
				return static_cast<const NpRigidStatic*>(this);
			else if(type == PxActorType::eARTICULATION_LINK)
				return static_cast<const NpArticulationLink*>(this);
			PX_ASSERT(0);
			return NULL;
		}

namespace
{
	template <typename N> NpActor* pxToNpActor(PxActor *p) 
	{  
		return static_cast<NpActor*>(static_cast<N*>(p));
	}
}

NpActor::Offsets::Offsets()
{
	for(PxU32 i=0;i<PxConcreteType::ePHYSX_CORE_COUNT;i++)
		pxActorToNpActor[i] = 0;
	size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
	PxActor* n = reinterpret_cast<PxActor*>(addr);
	pxActorToNpActor[PxConcreteType::eRIGID_STATIC]			= size_t(pxToNpActor<NpRigidStatic>(n)) - addr;
	pxActorToNpActor[PxConcreteType::eRIGID_DYNAMIC]		= size_t(pxToNpActor<NpRigidDynamic>(n)) - addr;
	pxActorToNpActor[PxConcreteType::eARTICULATION_LINK]	= size_t(pxToNpActor<NpArticulationLink>(n)) - addr;

#if PX_SUPPORT_GPU_PHYSX
	pxActorToNpActor[PxConcreteType::eSOFT_BODY]							= size_t(pxToNpActor<NpSoftBody>(n)) - addr;
	pxActorToNpActor[PxConcreteType::ePBD_PARTICLESYSTEM]					= size_t(pxToNpActor<NpPBDParticleSystem>(n)) - addr;
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	pxActorToNpActor[PxConcreteType::eFLIP_PARTICLESYSTEM]					= size_t(pxToNpActor<NpFLIPParticleSystem>(n)) - addr;
	pxActorToNpActor[PxConcreteType::eMPM_PARTICLESYSTEM]					= size_t(pxToNpActor<NpMPMParticleSystem>(n)) - addr;
	pxActorToNpActor[PxConcreteType::eFEM_CLOTH]							= size_t(pxToNpActor<NpFEMCloth>(n)) - addr;
	pxActorToNpActor[PxConcreteType::eHAIR_SYSTEM]							= size_t(pxToNpActor<NpHairSystem>(n)) - addr;
#endif
#endif
}

const NpActor::Offsets NpActor::sOffsets;

NpActor::NpOffsets::NpOffsets()
{
	// PT: .....
	{
		size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
		NpRigidStatic* n = reinterpret_cast<NpRigidStatic*>(addr);
		const size_t npOffset		= size_t(static_cast<NpActor*>(n)) - addr;
		const size_t staticOffset	= NpRigidStatic::getCoreOffset() - npOffset;
		npToSc[NpType::eRIGID_STATIC] = staticOffset;
	}
	{
		size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
		NpRigidDynamic* n = reinterpret_cast<NpRigidDynamic*>(addr);
		const size_t npOffset	= size_t(static_cast<NpActor*>(n)) - addr;
		const size_t bodyOffset	= NpRigidDynamic::getCoreOffset() - npOffset;
		npToSc[NpType::eBODY] = bodyOffset;
	}
	{
		size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
		NpArticulationLink* n = reinterpret_cast<NpArticulationLink*>(addr);
		const size_t npOffset	= size_t(static_cast<NpActor*>(n)) - addr;
		const size_t bodyOffset	= NpArticulationLink::getCoreOffset() - npOffset;
		npToSc[NpType::eBODY_FROM_ARTICULATION_LINK] = bodyOffset;
	}
#if PX_SUPPORT_GPU_PHYSX
	{
		size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
		NpSoftBody* n = reinterpret_cast<NpSoftBody*>(addr);
		const size_t npOffset = size_t(static_cast<NpActor*>(n)) - addr;
		const size_t bodyOffset = NpSoftBody::getCoreOffset() - npOffset;
		npToSc[NpType::eSOFTBODY] = bodyOffset;
	}
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	{
		size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
		NpFEMCloth* n = reinterpret_cast<NpFEMCloth*>(addr);
		const size_t npOffset = size_t(static_cast<NpActor*>(n)) - addr;
		const size_t bodyOffset = NpFEMCloth::getCoreOffset() - npOffset;
		npToSc[NpType::eFEMCLOTH] = bodyOffset;
	}
#endif
	{
		size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
		NpPBDParticleSystem* n = reinterpret_cast<NpPBDParticleSystem*>(addr);
		const size_t npOffset = size_t(static_cast<NpActor*>(n)) - addr;
		const size_t bodyOffset = NpPBDParticleSystem::getCoreOffset() - npOffset;
		npToSc[NpType::ePBD_PARTICLESYSTEM] = bodyOffset;
	}
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	{
		size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
		NpFLIPParticleSystem* n = reinterpret_cast<NpFLIPParticleSystem*>(addr);
		const size_t npOffset = size_t(static_cast<NpActor*>(n)) - addr;
		const size_t bodyOffset = NpFLIPParticleSystem::getCoreOffset() - npOffset;
		npToSc[NpType::eFLIP_PARTICLESYSTEM] = bodyOffset;
	}

	{
		size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
		NpMPMParticleSystem* n = reinterpret_cast<NpMPMParticleSystem*>(addr);
		const size_t npOffset = size_t(static_cast<NpActor*>(n)) - addr;
		const size_t bodyOffset = NpMPMParticleSystem::getCoreOffset() - npOffset;
		npToSc[NpType::eMPM_PARTICLESYSTEM] = bodyOffset;
	}

	{
		size_t addr = 0x100;	// casting the null ptr takes a special-case code path, which we don't want
		NpHairSystem* n = reinterpret_cast<NpHairSystem*>(addr);
		const size_t npOffset = size_t(static_cast<NpActor*>(n)) - addr;
		const size_t bodyOffset = NpHairSystem::getCoreOffset() - npOffset;
		npToSc[NpType::eHAIRSYSTEM] = bodyOffset;
	}
#endif
#endif
}

const NpActor::NpOffsets NpActor::sNpOffsets;


