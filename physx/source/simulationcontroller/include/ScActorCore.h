// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_ACTOR_CORE_H
#define SC_ACTOR_CORE_H

#include "foundation/PxBitAndData.h"
#include "PxActor.h"

#define SC_FILTERING_ID_SHIFT_BIT	24
#define SC_FILTERING_ID_MAX			(1<<SC_FILTERING_ID_SHIFT_BIT)
#define SC_FILTERING_ID_MASK		0x00ffffff

namespace physx
{
namespace Sc
{
	class ActorSim;

	class ActorCore
	{
	public:
// PX_SERIALIZATION
											ActorCore(const PxEMPTY) :	mSim(NULL), mActorFlags(PxEmpty)
											{
											}
//~PX_SERIALIZATION
											ActorCore(PxActorType::Enum actorType, PxU8 actorFlags, PxClientID owner, PxDominanceGroup dominanceGroup);
											~ActorCore();

		PX_FORCE_INLINE	ActorSim*			getSim()						const	{ return mSim;							}
		PX_FORCE_INLINE	void				setSim(ActorSim* sim)
											{
												PX_ASSERT((sim==NULL) ^ (mSim==NULL));
												mSim = sim;
											}

		PX_FORCE_INLINE	PxActorFlags		getActorFlags()					const	{ return mActorFlags;					}
						void				setActorFlags(PxActorFlags af);

		PX_FORCE_INLINE	PxDominanceGroup	getDominanceGroup()				const
											{
												return PxDominanceGroup(mDominanceGroup);	
											}
						void				setDominanceGroup(PxDominanceGroup g);

		PX_FORCE_INLINE	void				setOwnerClient(PxClientID inId)
											{
												const PxU32 id = mPackedIDs & SC_FILTERING_ID_MASK;
												mPackedIDs = (PxU32(inId)<<SC_FILTERING_ID_SHIFT_BIT) | id;
											}
		PX_FORCE_INLINE	PxClientID			getOwnerClient()				const
											{
												return mPackedIDs>>SC_FILTERING_ID_SHIFT_BIT;
											}

		PX_FORCE_INLINE	PxActorType::Enum	getActorCoreType()				const 	{ return PxActorType::Enum(mActorType);	}

						void				reinsertShapes();

						void				setAggregateID(PxU32 id);
		PX_FORCE_INLINE	PxU8				hasAggregateID()				const	{ return mDominanceGroup.isBitSet();	}
		PX_FORCE_INLINE	PxU32				getAggregateID()				const
											{
												if(!hasAggregateID())
													return PX_INVALID_U32;

												return mPackedIDs & SC_FILTERING_ID_MASK;
											}

						void				setEnvID(PxU32 id);
		PX_FORCE_INLINE	PxU32				getEnvID()				const
											{
												if(hasAggregateID())
													return PX_INVALID_U32;

												const PxU32 id = mPackedIDs & SC_FILTERING_ID_MASK;
												return id == SC_FILTERING_ID_MASK ? PX_INVALID_U32 : id;
											}
	private:
						ActorSim*			mSim;
						PxU32				mPackedIDs;			// PxClientID (8bit) | aggregate / env ID (24bit)
		// PT: TODO: the remaining members could be packed into just a 16bit mask
						PxActorFlags		mActorFlags;		// PxActor's flags (PxU8) => only 4 bits used
						PxU8				mActorType;			// Actor type (8 bits, but 3 would be enough)
						PxBitAndByte		mDominanceGroup;	// Aggregate bit | dominance group (7 bits, but 5 would be enough because "must be < 32")

		PX_FORCE_INLINE	void				setID(PxU32 id)
											{
												const PxU32 ownerClient = mPackedIDs & (~SC_FILTERING_ID_MASK);
												mPackedIDs = (id & SC_FILTERING_ID_MASK) | ownerClient;
											}

		PX_FORCE_INLINE	void				resetID()
											{
												mPackedIDs |= SC_FILTERING_ID_MASK;
											}
	};

#if PX_P64_FAMILY
	PX_COMPILE_TIME_ASSERT(sizeof(Sc::ActorCore)==16);
#else
	PX_COMPILE_TIME_ASSERT(sizeof(Sc::ActorCore)==12);
#endif

} // namespace Sc

}

//////////////////////////////////////////////////////////////////////////

#endif
