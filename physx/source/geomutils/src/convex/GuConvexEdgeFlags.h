// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CONVEX_EDGE_FLAGS_H
#define GU_CONVEX_EDGE_FLAGS_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
namespace Gu
{
	enum ExtraTrigDataFlag
	{
		ETD_SILHOUETTE_EDGE_01 = (1 << 0),	//First edge is a silhouette edge
		ETD_SILHOUETTE_EDGE_12 = (1 << 1),	//Second edge is a silhouette edge
		ETD_SILHOUETTE_EDGE_20 = (1 << 2),	//Third edge is a silhouette edge
		ETD_CONVEX_EDGE_01	= (1<<3),	// PT: important value, don't change
		ETD_CONVEX_EDGE_12	= (1<<4),	// PT: important value, don't change
		ETD_CONVEX_EDGE_20	= (1<<5),	// PT: important value, don't change

		ETD_CONVEX_EDGE_ALL	= ETD_CONVEX_EDGE_01|ETD_CONVEX_EDGE_12|ETD_CONVEX_EDGE_20
	};

	// PT: helper function to make sure we use the proper default flags everywhere
	PX_FORCE_INLINE PxU8 getConvexEdgeFlags(const PxU8* extraTrigData, PxU32 triangleIndex)
	{
		return extraTrigData ? extraTrigData[triangleIndex] : PxU8(ETD_CONVEX_EDGE_ALL);
	}

	PX_FORCE_INLINE void flipConvexEdgeFlags(PxU8& extraData)
	{
		// PT: this is a fix for PX-2327. When we flip the winding we also need to flip the precomputed edge flags.
		// 01 => 02
		// 12 => 21
		// 20 => 10

		const PxU8 convex01 = extraData & Gu::ETD_CONVEX_EDGE_01;
		const PxU8 convex12 = extraData & Gu::ETD_CONVEX_EDGE_12;
		const PxU8 convex20 = extraData & Gu::ETD_CONVEX_EDGE_20;
		const PxU8 silhouette01 = extraData & Gu::ETD_SILHOUETTE_EDGE_01;
		const PxU8 silhouette12 = extraData & Gu::ETD_SILHOUETTE_EDGE_12;
		const PxU8 silhouette20 = extraData & Gu::ETD_SILHOUETTE_EDGE_20;
		extraData = convex12|silhouette12;
		if(convex01)
			extraData |= Gu::ETD_CONVEX_EDGE_20;
		if(convex20)
			extraData |= Gu::ETD_CONVEX_EDGE_01;
		if(silhouette01)
			extraData |= Gu::ETD_SILHOUETTE_EDGE_20;
		if(silhouette20)
			extraData |= Gu::ETD_SILHOUETTE_EDGE_01;
	}
}
}

#endif
