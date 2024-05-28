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
