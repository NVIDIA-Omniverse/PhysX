// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_AGGREGATE_H
#define NP_AGGREGATE_H

#include "PxAggregate.h"
#include "NpBase.h"
#include "BpVolumeData.h"

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
//~PX_SERIALIZATION
												NpAggregate(PxU32 maxActors, PxU32 maxShapes, PxAggregateFilterHint filterHint);
		virtual									~NpAggregate();

		// PxAggregate
		virtual			void					release()	PX_OVERRIDE PX_FINAL;
		virtual			bool					addActor(PxActor&, const PxBVH*)	PX_OVERRIDE PX_FINAL;
		virtual			bool					removeActor(PxActor&)	PX_OVERRIDE PX_FINAL;
		virtual			bool					addArticulation(PxArticulationReducedCoordinate&)	PX_OVERRIDE PX_FINAL;
		virtual			bool					removeArticulation(PxArticulationReducedCoordinate&)	PX_OVERRIDE PX_FINAL;
		virtual			PxU32					getNbActors() const	PX_OVERRIDE PX_FINAL;
		virtual			PxU32					getMaxNbActors() const	PX_OVERRIDE PX_FINAL;
		virtual			PxU32					getMaxNbShapes() const	PX_OVERRIDE PX_FINAL;
		virtual			PxU32					getActors(PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const	PX_OVERRIDE PX_FINAL;
		virtual			PxScene*				getScene()	PX_OVERRIDE PX_FINAL;
		virtual			bool					getSelfCollision()	const	PX_OVERRIDE PX_FINAL;
		virtual			bool					setEnvironmentID(PxU32 envID)	PX_OVERRIDE PX_FINAL;
		virtual			PxU32					getEnvironmentID()		const	PX_OVERRIDE PX_FINAL;
		//~PxAggregate

		PX_FORCE_INLINE	PxU32					getMaxNbShapesFast()	const	{ return mMaxNbShapes;	}
		PX_FORCE_INLINE	PxU32					getCurrentSizeFast()	const	{ return mNbActors;		}
		PX_FORCE_INLINE	PxActor*				getActorFast(PxU32 i)	const	{ return mActors[i];	}
		PX_FORCE_INLINE Bp::AggregateHandle		getAggregateHandle()	const	{ return mAggregateHandle;	}
		PX_FORCE_INLINE void					setAggregateHandle(Bp::AggregateHandle h)	{ mAggregateHandle = h;		}

		PX_FORCE_INLINE	bool					getSelfCollideFast()	const	{ return PxGetAggregateSelfCollisionBit(mFilterHint)!=0;	}
		PX_FORCE_INLINE	PxAggregateFilterHint	getFilterHint()			const	{ return mFilterHint;	}
		PX_FORCE_INLINE	PxU32					getEnvID()				const	{ return mEnvID;		}

						void					scRemoveActor(NpActor& actor, bool reinsert);
						bool					removeActorAndReinsert(PxActor& actor, bool reinsert);
						bool					removeArticulationAndReinsert(PxArticulationReducedCoordinate& art, bool reinsert);
						void					addToScene(NpScene& scene);

						void					incShapeCount();
						void					decShapeCount();
private:
						Bp::AggregateHandle		mAggregateHandle;
						PxU32					mMaxNbActors;
						PxU32					mMaxNbShapes;
						PxAggregateFilterHint	mFilterHint;
						PxU32					mEnvID;
						PxU32					mNbActors;
						PxU32					mNbShapes;
						PxActor**				mActors;

						void					scAddActor(NpActor&);
						void					removeAndReinsert(PxActor& actor, bool reinsert);
						void					addActorInternal(PxActor& actor, NpScene& s, const PxBVH* bvh);
};

}

#endif
