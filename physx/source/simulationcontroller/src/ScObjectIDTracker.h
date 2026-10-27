// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_OBJECT_ID_TRACKER_H
#define SC_OBJECT_ID_TRACKER_H

#include "CmIDPool.h"
#include "foundation/PxBitMap.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
namespace Sc
{
	// PT: TODO: this has no direct dependency on "Sc". It should really be a "Cm" class.
	class ObjectIDTracker : public PxUserAllocated
	{
		PX_NOCOPY(ObjectIDTracker)
	public:
										ObjectIDTracker() : mPendingReleasedIDs("objectIDTrackerIDs") {}

		PX_INLINE		PxU32			createID()						{ return mIDPool.getNewID();				}

		PX_INLINE		void			releaseID(PxU32 id)				
										{
											markIDAsDeleted(id); 
											mPendingReleasedIDs.pushBack(id);	
										}

		PX_INLINE		PxIntBool		isDeletedID(PxU32 id)	const	{ return mDeletedIDsMap.boundedTest(id);	}
		PX_FORCE_INLINE PxU32			getDeletedIDCount()		const	{ return mPendingReleasedIDs.size();		}
		PX_INLINE		void			clearDeletedIDMap()				{ mDeletedIDsMap.clear();					}

		PX_INLINE		void			resizeDeletedIDMap(PxU32 id, PxU32 numIds)	
										{ 
											mDeletedIDsMap.resize(id); 
											mPendingReleasedIDs.reserve(numIds);
										}

		PX_INLINE		void			processPendingReleases()
										{
											for(PxU32 i=0; i < mPendingReleasedIDs.size(); i++)
												 mIDPool.freeID(mPendingReleasedIDs[i]);

											mPendingReleasedIDs.clear();
										}

		PX_INLINE		void			reset()
										{
											processPendingReleases();
											mPendingReleasedIDs.reset();

											// Don't free stuff in IDPool, we still need the list of free IDs
											// And it does not seem worth freeing the memory of the bitmap
										}

		PX_INLINE		PxU32			getMaxID()	const
										{
											return mIDPool.getMaxID();
										}
	private:
		PX_INLINE		void			markIDAsDeleted(PxU32 id)
										{
											PX_ASSERT(!isDeletedID(id));
											mDeletedIDsMap.growAndSet(id);
										}

						Cm::IDPool		mIDPool;
						PxBitMap		mDeletedIDsMap;
						PxArray<PxU32>	mPendingReleasedIDs;  // Buffer for released IDs to make sure newly created objects do not re-use these IDs immediately
	};
}
}

#endif
