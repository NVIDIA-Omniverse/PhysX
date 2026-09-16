// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_PRUNER_TYPEDEF_H
#define GU_PRUNER_TYPEDEF_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
	namespace Gu
	{
		typedef PxU32 PrunerHandle;
		static const PrunerHandle INVALID_PRUNERHANDLE = 0xffffffff;

		typedef PxU32 PoolIndex;
		static const PxU32 INVALID_POOL_ID = 0xffffffff;

		typedef PxU32 TreeNodeIndex;
		static const PxU32 INVALID_NODE_ID = 0xffffffff;

		enum CompanionPrunerType
		{
			COMPANION_PRUNER_NONE,
			COMPANION_PRUNER_BUCKET,
			COMPANION_PRUNER_INCREMENTAL,
			COMPANION_PRUNER_AABB_TREE
		};

		enum BVHBuildStrategy
		{
			BVH_SPLATTER_POINTS,
			BVH_SPLATTER_POINTS_SPLIT_GEOM_CENTER,
			BVH_SAH
		};
	}
}

#endif
