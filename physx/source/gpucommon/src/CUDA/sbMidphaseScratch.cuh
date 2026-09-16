// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef __CU_SB_MIDPHASESCRATCH_CUH__
#define __CU_SB_MIDPHASESCRATCH_CUH__

#include "vector_types.h"

namespace physx
{
namespace Gu
{
	struct BV32DataDepthInfo;
	struct BV32DataPacked;
}

struct sbMidphaseScratch
{
	const float4 * PX_RESTRICT tetmeshVerts;
	const uint4 * PX_RESTRICT tetmeshTetIndices;
	const PxU8* PX_RESTRICT tetmeshSurfaceHint;

	const Gu::BV32DataDepthInfo* PX_RESTRICT bv32DepthInfo;
	const PxU32* PX_RESTRICT bv32RemapPackedNodeIndex;
	//bv32 tree
	Gu::BV32DataPacked* bv32PackedNodes;

	//stack for traversal
	int sBv32Nodes[192]; //6 depth of the bv32 tree
};
PX_COMPILE_TIME_ASSERT(sizeof(sbMidphaseScratch) <= WARP_SIZE * 7 * sizeof(PxU32));

} // namespace physx

#endif
