// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_BROADPHASE_H
#define SC_BROADPHASE_H

#include "PxPhysXConfig.h"
#include "foundation/PxArray.h"

// PT: this class captures parts of the Sc::Scene that deals with broadphase matters.

namespace physx
{
	class PxBroadPhaseCallback;

namespace Bp
{
	class AABBManagerBase;
}

namespace Sc
{
	class ObjectIDTracker;

	class BroadphaseManager
	{
		public:
													BroadphaseManager();
													~BroadphaseManager();

			PX_FORCE_INLINE	void					setBroadPhaseCallback(PxBroadPhaseCallback* callback)	{ mBroadPhaseCallback = callback;	}
			PX_FORCE_INLINE	PxBroadPhaseCallback*	getBroadPhaseCallback()	const							{ return mBroadPhaseCallback;		}

							void					prepareOutOfBoundsCallbacks(Bp::AABBManagerBase* aabbManager);
							bool					fireOutOfBoundsCallbacks(Bp::AABBManagerBase* aabbManager, const ObjectIDTracker& tracker, PxU64 contextID);

							void					flush(Bp::AABBManagerBase* aabbManager);

							PxBroadPhaseCallback*	mBroadPhaseCallback;
							PxArray<PxU32>			mOutOfBoundsIDs;
	};

}
}

#endif
