// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_INCREMENTAL_AABB_PRUNER_H
#define GU_INCREMENTAL_AABB_PRUNER_H

#include "common/PxPhysXCommonConfig.h"
#include "GuPruner.h"
#include "GuPruningPool.h"
#include "GuIncrementalAABBTree.h"
#include "GuSqInternal.h"

namespace physx
{
	class PxRenderOutput;

namespace Gu
{
	class IncrementalAABBPruner : public Pruner
	{
		public:
		PX_PHYSX_COMMON_API							IncrementalAABBPruner(PxU32 sceneLimit, PxU64 contextID);
		virtual										~IncrementalAABBPruner();

		// BasePruner
													DECLARE_BASE_PRUNER_API
		//~BasePruner

		// Pruner
													DECLARE_PRUNER_API_COMMON
		//~Pruner
		
		// direct access for test code
		PX_FORCE_INLINE	const IncrementalAABBTree*	getAABBTree()		const		{ return mAABBTree;	}
				
		private:
						void						release();
						void						fullRebuildAABBTree();
						void						test();
						void						updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node);

						IncrementalAABBTree*		mAABBTree; 						

						PruningPool					mPool; // Pool of AABBs

				PxArray<IncrementalAABBTreeNode*>	mMapping;

						PxU64						mContextID;
						NodeList					mChangedLeaves;
	};

}
}

#endif
