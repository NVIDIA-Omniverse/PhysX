// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_SQ_BOUNDS_MANAGER_H
#define SC_SQ_BOUNDS_MANAGER_H

#include "foundation/PxUserAllocated.h"
#include "foundation/PxBitMap.h"
#include "foundation/PxArray.h"

#include "ScSqBoundsSync.h"

namespace physx
{
	class PxBounds3;

namespace Sc
{
	struct SqBoundsSync;
	struct SqRefFinder;
	class ShapeSimBase;

	class SqBoundsManager0 : public PxUserAllocated
	{
								PX_NOCOPY(SqBoundsManager0)
	public:
								SqBoundsManager0();

		void					addSyncShape(ShapeSimBase& shape);
		void					removeSyncShape(ShapeSimBase& shape);
		void					syncBounds(SqBoundsSync& sync, SqRefFinder& finder, const PxBounds3* bounds, const PxTransform32* transforms, PxU64 contextID, const PxBitMap& ignoredIndices);

	private:

		PxArray<ShapeSimBase*>	mShapes;		// 
		PxArray<ScPrunerHandle>	mRefs;			// SQ pruner references
		PxArray<PxU32>			mBoundsIndices;	// indices into the Sc bounds array
		PxArray<ShapeSimBase*>	mRefless;		// shapesims without references
	};

	class SqBoundsManagerEx : public PxUserAllocated
	{
								PX_NOCOPY(SqBoundsManagerEx)
	public:
								SqBoundsManagerEx();
								~SqBoundsManagerEx();

		void					addSyncShape(ShapeSimBase& shape);
		void					removeSyncShape(ShapeSimBase& shape);
		void					syncBounds(SqBoundsSync& sync, SqRefFinder& finder, const PxBounds3* bounds, const PxTransform32* transforms, PxU64 contextID, const PxBitMap& ignoredIndices);

	private:

		PxArray<ShapeSimBase*>	mWaitingRoom;

		// PT: one of the many solutions discussed in the confluence page "The new SQ system"
		// Just to get something working. This will most likely need revisiting later.

		struct PrunerSyncData : public PxUserAllocated
		{
			PxArray<ShapeSimBase*>	mShapes;		// 
			// PT: layout dictated by the SqPruner API here. We could consider merging these two arrays.
			PxArray<ScPrunerHandle>	mRefs;			// SQ pruner references
			PxArray<PxU32>			mBoundsIndices;	// indices into the Sc bounds array
		};

		PrunerSyncData**		mPrunerSyncData;
		PxU32					mPrunerSyncDataSize;

		void					resize(PxU32 index);
	};

	//class SqBoundsManager : public SqBoundsManager0
	class SqBoundsManager : public SqBoundsManagerEx
	{
		public:
	};
}
}

#endif
