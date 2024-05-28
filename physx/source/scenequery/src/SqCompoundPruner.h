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

#ifndef SQ_COMPOUND_PRUNER_H
#define SQ_COMPOUND_PRUNER_H

#include "SqCompoundPruningPool.h"
#include "GuSqInternal.h"
#include "GuPrunerMergeData.h"
#include "GuIncrementalAABBTree.h"
#include "GuPruningPool.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"

namespace physx
{
namespace Sq
{
	///////////////////////////////////////////////////////////////////////////////////////////////

	typedef PxHashMap<PrunerCompoundId, Gu::PoolIndex>	ActorIdPoolIndexMap;
	typedef PxArray<PrunerCompoundId>					PoolIndexActorIdMap;

	///////////////////////////////////////////////////////////////////////////////////////////////

	class BVHCompoundPruner : public CompoundPruner
	{
		public:
												BVHCompoundPruner(PxU64 contextID);
		virtual									~BVHCompoundPruner();

					void						release();

		// BasePruner
												DECLARE_BASE_PRUNER_API
		//~BasePruner

		// CompoundPruner
		// compound level
		virtual		bool						addCompound(Gu::PrunerHandle* results, const Gu::BVH& bvh, PrunerCompoundId compoundId, const PxTransform& transform, bool isDynamic, const Gu::PrunerPayload* data, const PxTransform* transforms);
		virtual		bool						removeCompound(PrunerCompoundId compoundId, Gu::PrunerPayloadRemovalCallback* removalCallback);
		virtual		bool						updateCompound(PrunerCompoundId compoundId, const PxTransform& transform);
		// object level
		virtual		void						updateObjectAfterManualBoundsUpdates(PrunerCompoundId compoundId, const Gu::PrunerHandle handle);
		virtual		void						removeObject(PrunerCompoundId compoundId, const Gu::PrunerHandle handle, Gu::PrunerPayloadRemovalCallback* removalCallback);
		virtual		bool						addObject(PrunerCompoundId compoundId, Gu::PrunerHandle& result, const PxBounds3& bounds, const Gu::PrunerPayload userData, const PxTransform& transform);
		//queries
		virtual		bool						raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const;
		virtual		bool						overlap(const Gu::ShapeData& queryVolume, CompoundPrunerOverlapCallback&, PxCompoundPrunerQueryFlags flags) const;
		virtual		bool						sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const;
		virtual		const Gu::PrunerPayload&	getPayloadData(Gu::PrunerHandle handle, PrunerCompoundId compoundId, Gu::PrunerPayloadData* data) const;
		virtual		void						preallocate(PxU32 nbEntries);
		virtual		bool						setTransform(Gu::PrunerHandle handle, PrunerCompoundId compoundId, const PxTransform& transform);
		virtual		const PxTransform&			getTransform(PrunerCompoundId compoundId)	const;
		virtual		void						visualizeEx(PxRenderOutput& out, PxU32 color, bool drawStatic, bool drawDynamic)	const;
		// ~CompoundPruner

		private:
					void						updateMapping(const Gu::PoolIndex poolIndex, Gu::IncrementalAABBTreeNode* node);
					void						updateMainTreeNode(Gu::PoolIndex index);

					void						test();

					Gu::IncrementalAABBTree		mMainTree;
					UpdateMap					mMainTreeUpdateMap;
		
					CompoundTreePool			mCompoundTreePool;
					ActorIdPoolIndexMap			mActorPoolMap;
					PoolIndexActorIdMap			mPoolActorMap;
					Gu::NodeList				mChangedLeaves;
		mutable		bool						mDrawStatic;
		mutable		bool						mDrawDynamic;
	};
}
}

#endif
