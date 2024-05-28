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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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

		// PT: one of the many solutions discussed in https://confluence.nvidia.com/display/~pterdiman/The+new+SQ+system
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
