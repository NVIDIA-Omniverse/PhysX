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

#ifndef NP_ACTOR_H
#define NP_ACTOR_H

#include "NpConnector.h"
#include "NpBase.h"

namespace physx
{
	class NpShapeManager;
	class NpAggregate;
	class NpScene;
	class NpShape;

	const Sc::BodyCore* getBodyCore(const PxRigidActor* actor);
	PX_FORCE_INLINE Sc::BodyCore* getBodyCore(PxRigidActor* actor)
	{
		const Sc::BodyCore* core = getBodyCore(static_cast<const PxRigidActor*>(actor));
		return const_cast<Sc::BodyCore*>(core);
	}

class NpActor : public NpBase
{
public:
// PX_SERIALIZATION
											NpActor(const PxEMPTY) : NpBase(PxEmpty)	{}				
					void					exportExtraData(PxSerializationContext& stream);	
					void					importExtraData(PxDeserializationContext& context);
					void					resolveReferences(PxDeserializationContext& context);
	static			void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
											NpActor(NpType::Enum type);

					void					removeConstraints(PxRigidActor& owner);
					void					removeFromAggregate(PxActor& owner);

					NpAggregate*			getNpAggregate(PxU32& index)	const;
					void					setAggregate(NpAggregate* np, PxActor& owner);
					PxAggregate*			getAggregate()					const;
					void					scSetDominanceGroup(PxDominanceGroup v);
					void					scSetOwnerClient(PxClientID inId);
					void					removeConstraintsFromScene();
	PX_FORCE_INLINE void					addConstraintsToScene()		// inline the fast path for addActors()
											{
												if(mConnectorArray)
													addConstraintsToSceneInternal();
											}

					PxU32					findConnector(NpConnectorType::Enum type, PxBase* object)	const;
					void					addConnector(NpConnectorType::Enum type, PxBase* object, const char* errMsg);
					void					removeConnector(PxActor& owner, NpConnectorType::Enum type, PxBase* object, const char* errorMsg);
					PxU32					getNbConnectors(NpConnectorType::Enum type)	const;

	static			NpShapeManager*			getShapeManager_(PxRigidActor& actor);			// bit misplaced here, but we don't want a separate subclass just for this
	static			const NpShapeManager*	getShapeManager_(const PxRigidActor& actor);	// bit misplaced here, but we don't want a separate subclass just for this

	static			NpActor&				getFromPxActor(PxActor& actor)			{ 	return *PxPointerOffset<NpActor*>(&actor, ptrdiff_t(sOffsets.pxActorToNpActor[actor.getConcreteType()])); }
	static			const NpActor&			getFromPxActor(const PxActor& actor)	{	return *PxPointerOffset<const NpActor*>(&actor, ptrdiff_t(sOffsets.pxActorToNpActor[actor.getConcreteType()])); }
				
					const PxActor*			getPxActor() const;

	static			NpScene*				getNpSceneFromActor(const PxActor& actor)
											{
												return getFromPxActor(actor).getNpScene();
											}

	PX_FORCE_INLINE	NpConnectorIterator		getConnectorIterator(NpConnectorType::Enum type)
											{
												if (mConnectorArray)
													return NpConnectorIterator(&mConnectorArray->front(), mConnectorArray->size(), type);
												else
													return NpConnectorIterator(NULL, 0, type);
											}

	static			void					onActorRelease(PxActor* actor);

	template<typename T>	PxU32			getConnectors(NpConnectorType::Enum type, T** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const
											{
												PxU32 nbConnectors = 0;
												if(mConnectorArray)
												{
													for(PxU32 i=0; i<mConnectorArray->size(); i++)
													{
														NpConnector& c = (*mConnectorArray)[i];
														if(c.mType == type && nbConnectors < bufferSize && i>=startIndex)
															userBuffer[nbConnectors++] = static_cast<T*>(c.mObject);
													}
												}
												return nbConnectors;
											}

	PX_INLINE		PxActorFlags			getActorFlags()		const		{ return getActorCore().getActorFlags();		}
	PX_INLINE		PxDominanceGroup		getDominanceGroup()	const		{ return getActorCore().getDominanceGroup();	}
	PX_INLINE		PxClientID				getOwnerClient()	const		{ return getActorCore().getOwnerClient();		}

	PX_INLINE		void					scSetActorFlags(PxActorFlags v)
											{
												PX_ASSERT(!isAPIWriteForbidden());

												// PT: TODO: move this check out of here, they should be done in Np!
#if PX_CHECKED
												const PxActorFlags aFlags = getActorFlags();
												const NpType::Enum npType = getNpType();
												if((!aFlags.isSet(PxActorFlag::eDISABLE_SIMULATION)) && v.isSet(PxActorFlag::eDISABLE_SIMULATION) &&
													(npType != NpType::eBODY) && (npType != NpType::eRIGID_STATIC))
												{
													PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, 
														"PxActor::setActorFlag: PxActorFlag::eDISABLE_SIMULATION is only supported by PxRigidDynamic and PxRigidStatic objects.");
												}
#endif
												getActorCore().setActorFlags(v);
												UPDATE_PVD_PROPERTY
											}


	PX_FORCE_INLINE const Sc::ActorCore&	getActorCore() const 
											{
												return *reinterpret_cast<const Sc::ActorCore*>(size_t(this) + sNpOffsets.npToSc[getNpType()]);
											}
	PX_FORCE_INLINE	Sc::ActorCore&			getActorCore()
											{
												return *reinterpret_cast<Sc::ActorCore*>(size_t(this) + sNpOffsets.npToSc[getNpType()]);
											}

	PX_INLINE const Sc::RigidCore&			getScRigidCore()	const
											{
												return static_cast<const Sc::RigidCore&>(getActorCore());
											}
	PX_INLINE Sc::RigidCore&				getScRigidCore()
											{
												return static_cast<Sc::RigidCore&>(getActorCore());
											}

	PX_FORCE_INLINE	void					scSwitchToNoSim()
											{
												NpScene* scene = getNpScene();

												if(scene && (!scene->isAPIWriteForbidden()))
													scene->scSwitchRigidToNoSim(*this);
											}

	PX_FORCE_INLINE void					scSwitchFromNoSim()
											{
												NpScene* scene = getNpScene();

												if(scene && (!scene->isAPIWriteForbidden()))
													scene->scSwitchRigidFromNoSim(*this);
											}
protected:
											~NpActor()	{}
					const char*				mName;
				// Lazy-create array for connector objects like constraints, observers, ... 
				// Most actors have no such objects, so we bias this class accordingly:
					NpConnectorArray*		mConnectorArray;
private:
					void					addConstraintsToSceneInternal();
					void					removeConnector(PxActor& owner, PxU32 index);
	struct Offsets
	{
		size_t pxActorToNpActor[PxConcreteType::ePHYSX_CORE_COUNT];
		Offsets();
	};
public:
	static const Offsets sOffsets;

	struct NpOffsets
	{
		size_t npToSc[NpType::eTYPE_COUNT];
		NpOffsets();
	};
	static const NpOffsets					sNpOffsets;
};

}

#endif
