// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_PRUNER_MERGE_DATA_H
#define GU_PRUNER_MERGE_DATA_H

#include "foundation/PxSimpleTypes.h"
namespace physx
{
	namespace Gu
	{
		struct BVHNode;

		// PT: TODO: refactor with BVHCoreData ?
		struct AABBPrunerMergeData
		{
			AABBPrunerMergeData()
			{
				// PT: it's important to NOT initialize anything by default (for binary serialization)
			}

			PxU32		mNbNodes;			// Nb nodes in AABB tree
			BVHNode*	mAABBTreeNodes;		// AABB tree runtime nodes
			PxU32		mNbObjects;			// Nb objects in AABB tree
			PxU32*		mAABBTreeIndices;	// AABB tree indices

			void init(PxU32 nbNodes=0, BVHNode* nodes=NULL, PxU32 nbObjects=0, PxU32* indices=NULL)
			{
				mNbNodes			= nbNodes;
				mAABBTreeNodes		= nodes;
				mNbObjects			= nbObjects;
				mAABBTreeIndices	= indices;
			}
		};
	}
}

#endif
