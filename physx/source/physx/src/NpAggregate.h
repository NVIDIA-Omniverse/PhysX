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

#ifndef NP_AGGREGATE_H
#define NP_AGGREGATE_H

#include "PxAggregate.h"
#include "NpBase.h"

namespace physx
{
class NpScene;

class NpAggregate : public PxAggregate, public NpBase
{
public:
// PX_SERIALIZATION
												NpAggregate(PxBaseFlags baseFlags) : PxAggregate(baseFlags), NpBase(PxEmpty) {}
						void					preExportDataReset();
	    virtual	        void	     			exportExtraData(PxSerializationContext& context);
						void					importExtraData(PxDeserializationContext& context);
						void					resolveReferences(PxDeserializationContext& context);
	    virtual	        void					requiresObjects(PxProcessPxBaseCallback& c);
		static			NpAggregate*			createObject(PxU8*& address, PxDeserializationContext& context);
		static			void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
												NpAggregate(PxU32 maxActors, PxU32 maxShapes, PxAggregateFilterHint filterHint);
		virtual									~NpAggregate();

		// PxAggregate
		virtual			void					release()	PX_OVERRIDE;
		virtual			bool					addActor(PxActor&, const PxBVH*)	PX_OVERRIDE;
		virtual			bool					removeActor(PxActor&)	PX_OVERRIDE;
		virtual			bool					addArticulation(PxArticulationReducedCoordinate&)	PX_OVERRIDE;
		virtual			bool					removeArticulation(PxArticulationReducedCoordinate&)	PX_OVERRIDE;
		virtual			PxU32					getNbActors() const	PX_OVERRIDE;
		virtual			PxU32					getMaxNbActors() const	PX_OVERRIDE;
		virtual			PxU32					getMaxNbShapes() const	PX_OVERRIDE;
		virtual			PxU32					getActors(PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const	PX_OVERRIDE;
		virtual			PxScene*				getScene()	PX_OVERRIDE;
		virtual			bool					getSelfCollision()	const	PX_OVERRIDE;
		//~PxAggregate

		PX_FORCE_INLINE	PxU32					getMaxNbShapesFast()	const	{ return mMaxNbShapes;	}
		PX_FORCE_INLINE	PxU32					getCurrentSizeFast()	const	{ return mNbActors;		}
		PX_FORCE_INLINE	PxActor*				getActorFast(PxU32 i)	const	{ return mActors[i];	}
		PX_FORCE_INLINE PxU32					getAggregateID()		const	{ return mAggregateID;	}
		PX_FORCE_INLINE void					setAggregateID(PxU32 cid)		{ mAggregateID = cid;	}

		PX_FORCE_INLINE	bool					getSelfCollideFast()	const	{ return PxGetAggregateSelfCollisionBit(mFilterHint)!=0;	}
		PX_FORCE_INLINE	PxAggregateFilterHint	getFilterHint()			const	{ return mFilterHint;	}

						void					scRemoveActor(NpActor& actor, bool reinsert);
						bool					removeActorAndReinsert(PxActor& actor, bool reinsert);
						bool					removeArticulationAndReinsert(PxArticulationReducedCoordinate& art, bool reinsert);
						void					addToScene(NpScene& scene);

						void					incShapeCount();
						void					decShapeCount();
private:
						PxU32					mAggregateID;
						PxU32					mMaxNbActors;
						PxU32					mMaxNbShapes;
						PxAggregateFilterHint	mFilterHint;
						PxU32					mNbActors;
						PxU32					mNbShapes;
						PxActor**				mActors;

						void					scAddActor(NpActor&);
						void					removeAndReinsert(PxActor& actor, bool reinsert);
						void					addActorInternal(PxActor& actor, NpScene& s, const PxBVH* bvh);
};

}

#endif
