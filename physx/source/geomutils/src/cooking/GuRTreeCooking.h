// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_COOKING_RTREE_H
#define GU_COOKING_RTREE_H

#include "cooking/PxCooking.h"

#include "foundation/PxArray.h"
#include "GuMeshData.h"
#include "GuRTree.h"

namespace physx
{
	struct RTreeCooker
	{
		struct RemapCallback // a callback to convert indices from triangle to LeafTriangles or other uses
		{
            virtual ~RemapCallback() {}
			virtual void remap(PxU32* rtreePtr, PxU32 start, PxU32 leafCount) = 0;
		};

		// triangles will be remapped so that newIndex = resultPermute[oldIndex]
		static void buildFromTriangles(
			Gu::RTree& resultTree, const PxVec3* verts, PxU32 numVerts, const PxU16* tris16, const PxU32* tris32, PxU32 numTris,
			PxArray<PxU32>& resultPermute, RemapCallback* rc, PxReal sizePerfTradeOff01, PxMeshCookingHint::Enum hint);
	};
}

#endif
