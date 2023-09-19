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

#ifndef NP_ACTOR_TEMPLATE_H
#define NP_ACTOR_TEMPLATE_H

#include "NpCheck.h"
#include "NpActor.h"
#include "NpScene.h"

#include "omnipvd/NpOmniPvdSetData.h"

namespace physx
{

// PT: only API (virtual) functions should be implemented here. Other shared non-virtual functions should go to NpActor.

/**
This is an API class. API classes run in a different thread than the simulation.
For the sake of simplicity they have their own methods, and they do not call simulation
methods directly. To set simulation state, they also have their own custom set
methods in the implementation classes.

Changing the data layout of this class breaks the binary serialization format.
See comments for PX_BINARY_SERIAL_VERSION.
*/
template<class APIClass>
class NpActorTemplate : public APIClass, public NpActor
{
	PX_NOCOPY(NpActorTemplate)
public:
// PX_SERIALIZATION
									NpActorTemplate(PxBaseFlags baseFlags) : APIClass(baseFlags), NpActor(PxEmpty) {}

	virtual	void					exportExtraData(PxSerializationContext& context) { NpActor::exportExtraData(context); }
	virtual	void					importExtraData(PxDeserializationContext& context) { NpActor::importExtraData(context); }
	virtual void					resolveReferences(PxDeserializationContext& context) { NpActor::resolveReferences(context); }
//~PX_SERIALIZATION

									NpActorTemplate(PxType concreteType, PxBaseFlags baseFlags, NpType::Enum type);
	virtual							~NpActorTemplate();

	// The rule is: If an API method is used somewhere in here, it has to be redeclared, else GCC whines

	// PxActor
	virtual		void				release()	= 0;
	virtual		PxActorType::Enum	getType()	const = 0;
	virtual		PxScene*			getScene()	const	PX_OVERRIDE;
	virtual		void				setName(const char*)	PX_OVERRIDE;
	virtual		const char*			getName()	const	PX_OVERRIDE;
	virtual		PxBounds3			getWorldBounds(float inflation=1.01f)	const = 0;
	virtual		void				setActorFlag(PxActorFlag::Enum flag, bool value)	PX_OVERRIDE;
	virtual		void				setActorFlags(PxActorFlags inFlags)	PX_OVERRIDE;
	virtual		PxActorFlags		getActorFlags()	const	PX_OVERRIDE;
	virtual		void				setDominanceGroup(PxDominanceGroup dominanceGroup)	PX_OVERRIDE;
	virtual		PxDominanceGroup	getDominanceGroup()	const	PX_OVERRIDE;
	virtual		void				setOwnerClient( PxClientID inClient )	PX_OVERRIDE;
	virtual		PxClientID			getOwnerClient()	const	PX_OVERRIDE;
	virtual		PxAggregate*		getAggregate()	const	PX_OVERRIDE { return NpActor::getAggregate();	}
	//~PxActor

protected:
	PX_FORCE_INLINE void			setActorFlagInternal(PxActorFlag::Enum flag, bool value);
	PX_FORCE_INLINE void			setActorFlagsInternal(PxActorFlags inFlags);
};

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
NpActorTemplate<APIClass>::NpActorTemplate(PxType concreteType, PxBaseFlags baseFlags, NpType::Enum type) :
	APIClass(concreteType, baseFlags),
	NpActor	(type)
{
	PX_ASSERT(!APIClass::userData);
}

template<class APIClass>
NpActorTemplate<APIClass>::~NpActorTemplate()
{
	NpActor::onActorRelease(this);
}

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
PxScene* NpActorTemplate<APIClass>::getScene() const
{
	return getNpScene();
}

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
void NpActorTemplate<APIClass>::setName(const char* debugName)
{
	NP_WRITE_CHECK(getNpScene());

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN_EXCEPT_SPLIT_SIM(getNpScene(), "PxActor::setName() not allowed while simulation is running. Call will be ignored.")

	mName = debugName;

#if PX_SUPPORT_OMNI_PVD
	PxActor & a = *this;
	streamActorName(a, mName);
#endif


#if PX_SUPPORT_PVD
	NpScene* npScene = getNpScene();
	//Name changing is not bufferred
	if(npScene)
		npScene->getScenePvdClientInternal().updatePvdProperties(static_cast<NpActor*>(this));	
#endif
}

template<class APIClass>
const char* NpActorTemplate<APIClass>::getName() const
{
	NP_READ_CHECK(getNpScene());
	return mName;
}

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
void NpActorTemplate<APIClass>::setDominanceGroup(PxDominanceGroup dominanceGroup)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxActor::setDominanceGroup() not allowed while simulation is running. Call will be ignored.")

	NpActor::scSetDominanceGroup(dominanceGroup);
}

template<class APIClass>
PxDominanceGroup NpActorTemplate<APIClass>::getDominanceGroup() const
{
	NP_READ_CHECK(getNpScene());
	return NpActor::getDominanceGroup();
}

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
void NpActorTemplate<APIClass>::setOwnerClient( PxClientID inId )
{
	if ( getNpScene() != NULL )
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, 
				"Attempt to set the client id when an actor is already in a scene.");
	}
	else
		NpActor::scSetOwnerClient( inId );
}

template<class APIClass>
PxClientID NpActorTemplate<APIClass>::getOwnerClient() const
{
	return NpActor::getOwnerClient();
}

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
PX_FORCE_INLINE void NpActorTemplate<APIClass>::setActorFlagInternal(PxActorFlag::Enum flag, bool value)
{
	NpActor& a = *this;
	if (value)
		a.scSetActorFlags( a.getActorFlags() | flag );
	else
		a.scSetActorFlags( a.getActorFlags() & (~PxActorFlags(flag)) );
}

template<class APIClass>
PX_FORCE_INLINE void NpActorTemplate<APIClass>::setActorFlagsInternal(PxActorFlags inFlags)
{
	NpActor::scSetActorFlags(inFlags);
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxActor, flags, static_cast<PxActor&>(*this), inFlags)
}

template<class APIClass>
void NpActorTemplate<APIClass>::setActorFlag(PxActorFlag::Enum flag, bool value)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxActor::setActorFlag() not allowed while simulation is running. Call will be ignored.")

	setActorFlagInternal(flag, value);
}

template<class APIClass>
void NpActorTemplate<APIClass>::setActorFlags(PxActorFlags inFlags)
{
	NpScene* npScene = getNpScene();
	NP_WRITE_CHECK(npScene);

	PX_CHECK_SCENE_API_WRITE_FORBIDDEN(npScene, "PxActor::setActorFlags() not allowed while simulation is running. Call will be ignored.")
	
	setActorFlagsInternal(inFlags);
}

template<class APIClass>
PxActorFlags NpActorTemplate<APIClass>::getActorFlags() const
{
	NP_READ_CHECK(getNpScene());
	return NpActor::getActorFlags();
}

///////////////////////////////////////////////////////////////////////////////

}

#endif
