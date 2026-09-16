// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
		virtual		bool						addCompound(Gu::PrunerHandle* results, const Gu::BVH& bvh, PrunerCompoundId compoundId, const PxTransform& transform, bool isDynamic, const Gu::PrunerPayload* data, const PxTransform* transforms) PX_OVERRIDE;
		virtual		bool						removeCompound(PrunerCompoundId compoundId, Gu::PrunerPayloadRemovalCallback* removalCallback) PX_OVERRIDE;
		virtual		bool						updateCompound(PrunerCompoundId compoundId, const PxTransform& transform) PX_OVERRIDE;
		// object level
		virtual		void						updateObjectAfterManualBoundsUpdates(PrunerCompoundId compoundId, const Gu::PrunerHandle handle) PX_OVERRIDE;
		virtual		void						removeObject(PrunerCompoundId compoundId, const Gu::PrunerHandle handle, Gu::PrunerPayloadRemovalCallback* removalCallback) PX_OVERRIDE;
		virtual		bool						addObject(PrunerCompoundId compoundId, Gu::PrunerHandle& result, const PxBounds3& bounds, const Gu::PrunerPayload userData, const PxTransform& transform) PX_OVERRIDE;
		//queries
		virtual		bool						raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const PX_OVERRIDE;
		virtual		bool						overlap(const Gu::ShapeData& queryVolume, CompoundPrunerOverlapCallback&, PxCompoundPrunerQueryFlags flags) const PX_OVERRIDE;
		virtual		bool						sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, CompoundPrunerRaycastCallback&, PxCompoundPrunerQueryFlags flags) const PX_OVERRIDE;
		virtual		const Gu::PrunerPayload&	getPayloadData(Gu::PrunerHandle handle, PrunerCompoundId compoundId, Gu::PrunerPayloadData* data) const PX_OVERRIDE;
		virtual		void						preallocate(PxU32 nbEntries) PX_OVERRIDE;
		virtual		bool						setTransform(Gu::PrunerHandle handle, PrunerCompoundId compoundId, const PxTransform& transform) PX_OVERRIDE;
		virtual		const PxTransform&			getTransform(PrunerCompoundId compoundId)	const PX_OVERRIDE;
		virtual		void						visualizeEx(PxRenderOutput& out, PxU32 color, bool drawStatic, bool drawDynamic)	const PX_OVERRIDE;
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
