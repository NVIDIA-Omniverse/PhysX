// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_MAVERICK_NODE_H
#define GU_MAVERICK_NODE_H

#include "foundation/PxBounds3.h"
#include "foundation/PxTransform.h"
#include "common/PxPhysXCommonConfig.h"
#include "GuPrunerPayload.h"
#include "GuPrunerTypedef.h"

#define FREE_PRUNER_SIZE	16

#ifdef FREE_PRUNER_SIZE

namespace physx
{
namespace Gu
{
	class MaverickNode
	{
		public:
										MaverickNode() : mNbFree(0)	{}
										~MaverickNode()				{}

		PX_FORCE_INLINE	void			release()								{ mNbFree = 0;		}
		PX_FORCE_INLINE	const PxU32*	getPrimitives(const PxU32*)		const	{ return mIndices;	}
		PX_FORCE_INLINE	PxU32			getPrimitiveIndex()				const	{ return 0;			}
		PX_FORCE_INLINE	PxU32			getNbPrimitives()				const	{ return mNbFree;	}

						bool			addObject(const PrunerPayload& object, PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform, PxU32 timeStamp);

						bool			updateObject(const PrunerPayload& object, const PxBounds3& worldAABB, const PxTransform& transform);
						bool			updateObject(PrunerHandle handle, const PxBounds3& worldAABB, const PxTransform& transform);

						bool			removeObject(const PrunerPayload& object, PxU32& timeStamp);
						bool			removeObject(PrunerHandle handle, PxU32& timeStamp);

						PxU32			removeMarkedObjects(PxU32 timeStamp);
						void			shiftOrigin(const PxVec3& shift);

						void			remove(PxU32 index);

						PxU32			mNbFree;							// Current number of objects in the "free array" (mFreeObjects/mFreeBounds)
						PrunerPayload	mFreeObjects[FREE_PRUNER_SIZE];		// mNbFree objects are stored here
						PrunerHandle	mFreeHandles[FREE_PRUNER_SIZE];		// mNbFree handles are stored here
						PxBounds3		mFreeBounds[FREE_PRUNER_SIZE];		// mNbFree object bounds are stored here
						PxTransform		mFreeTransforms[FREE_PRUNER_SIZE];	// mNbFree transforms are stored here
						PxU32			mFreeStamps[FREE_PRUNER_SIZE];
		static			const PxU32		mIndices[FREE_PRUNER_SIZE];
	};
}
}

#endif
#endif
