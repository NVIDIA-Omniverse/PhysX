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
