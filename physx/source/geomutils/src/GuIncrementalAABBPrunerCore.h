// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INCREMENTAL_AABB_PRUNER_CORE_H
#define GU_INCREMENTAL_AABB_PRUNER_CORE_H

#include "GuPruner.h"
#include "GuIncrementalAABBTree.h"
#include "GuPruningPool.h"
#include "GuAABBTreeUpdateMap.h"
#include "foundation/PxHashMap.h"

namespace physx
{
	class PxRenderOutput;

namespace Gu
{
	typedef PxHashMap<PoolIndex, IncrementalAABBTreeNode*>	IncrementalPrunerMap;

	struct CoreTree
	{
		PX_FORCE_INLINE			CoreTree() : timeStamp(0), tree(NULL)	{}

		PxU32					timeStamp;
		IncrementalAABBTree*	tree;
		IncrementalPrunerMap	mapping;
	};

	class IncrementalAABBPrunerCore : public PxUserAllocated
	{
	public:
											IncrementalAABBPrunerCore(const PruningPool* pool);
											~IncrementalAABBPrunerCore();

						void				release();

						bool				addObject(const PoolIndex poolIndex, PxU32 timeStamp);
						bool				removeObject(const PoolIndex poolIndex, const PoolIndex poolRelocatedLastIndex, PxU32& timeStamp);

		// if we swap object from bucket pruner index with an index in the regular AABB pruner
						void				swapIndex(const PoolIndex poolIndex, const PoolIndex poolRelocatedLastIndex);

						bool				updateObject(const PoolIndex poolIndex);

						PxU32				removeMarkedObjects(PxU32 timeStamp);

						bool				raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback&) const;
						bool				overlap(const ShapeData& queryVolume, PrunerOverlapCallback&) const;
						bool				sweep(const ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerRaycastCallback&) const;
						void				getGlobalBounds(PxBounds3&)	const;

						void				shiftOrigin(const PxVec3& shift);

						void				visualize(PxRenderOutput& out, PxU32 color) const;

		PX_FORCE_INLINE void				timeStampChange()
											{
												// swap current and last tree
												mLastTree = (mLastTree + 1) % 2;
												mCurrentTree = (mCurrentTree + 1) % 2;
											}

						void				build() {}

		PX_FORCE_INLINE	PxU32				getNbObjects()	const { return mAABBTree[0].mapping.size() + mAABBTree[1].mapping.size(); }

	private:
						void				updateMapping(IncrementalPrunerMap& mapping, const PoolIndex poolIndex, IncrementalAABBTreeNode* node);
						void				test(bool hierarchyCheck = true);			

	private:
		static			const PxU32			NUM_TREES = 2;

						PxU32				mCurrentTree;
						PxU32				mLastTree;
						CoreTree			mAABBTree[NUM_TREES];
						const PruningPool*	mPool;					// Pruning pool from AABB pruner
						NodeList			mChangedLeaves;
	};

}}

#endif
