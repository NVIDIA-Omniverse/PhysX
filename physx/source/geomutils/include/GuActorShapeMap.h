// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_ACTOR_SHAPE_MAP_H
#define GU_ACTOR_SHAPE_MAP_H

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxHashMap.h"

namespace physx
{
namespace Gu
{
	typedef PxU64	ActorShapeData;

	#define	PX_INVALID_INDEX	0xffffffff

	class ActorShapeMap
	{
		public:
		PX_PHYSX_COMMON_API					ActorShapeMap();
		PX_PHYSX_COMMON_API					~ActorShapeMap();

		PX_PHYSX_COMMON_API	bool			add(PxU32 actorIndex, const void* actor, const void* shape, ActorShapeData actorShapeData);
		PX_PHYSX_COMMON_API	bool			remove(PxU32 actorIndex, const void* actor, const void* shape, ActorShapeData* removed);
		PX_PHYSX_COMMON_API	ActorShapeData	find(PxU32 actorIndex, const void* actor, const void* shape)	const;

			struct ActorShape
			{
				PX_FORCE_INLINE	ActorShape()																	{}
				PX_FORCE_INLINE	ActorShape(const void* actor, const void* shape) : mActor(actor), mShape(shape)	{}

				const void*	mActor;
				const void*	mShape;

				PX_FORCE_INLINE bool operator==(const ActorShape& p) const
				{
					return mActor == p.mActor && mShape == p.mShape;
				}
			};
		private:
			PxHashMap<ActorShape, ActorShapeData>	mDatabase;

			struct Cache
			{
//				const void*		mActor;
				const void*		mShape;
				ActorShapeData	mData;
			};
			PxU32				mCacheSize;
			Cache*				mCache;

			void			resizeCache(PxU32 index);
	};
}
}

#endif
