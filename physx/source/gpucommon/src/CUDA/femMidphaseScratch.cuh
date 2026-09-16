// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef __CU_FEM_CLOTH_MIDPHASESCRATCH_CUH__
#define __CU_FEM_CLOTH_MIDPHASESCRATCH_CUH__

#include "vector_types.h"

#define	FEM_MIDPHASE_SCRATCH_SIZE 224 // 192 (WARP SIZE * 6) < 198 (sizeof(femMidphaseScratch)/sizeof(unsigned int)) < 224 (WARP SIZE * 7)

namespace physx
{
	namespace Gu
	{
		struct BV32DataDepthInfo;
		struct BV32DataPacked;
	};
}

struct femMidphaseScratch
{
	const float4* PX_RESTRICT meshVerts;       // either tetrahedron mesh or triangle mesh
	const uint4* PX_RESTRICT meshVertsIndices; // either tetrahedron mesh or triangle mesh

	// const physx::Gu::BV32DataDepthInfo* PX_RESTRICT bv32DepthInfo;
	// const unsigned int* PX_RESTRICT bv32RemapPackedNodeIndex;
	// bv32 tree
	const physx::Gu::BV32DataPacked* bv32PackedNodes;

	// stack for traversal
	int sBv32Nodes[192]; // 6 depth of the bv32 tree
};
PX_COMPILE_TIME_ASSERT(sizeof(femMidphaseScratch) <= WARP_SIZE * 7 * sizeof(unsigned int));

class femClothRefitMidphaseScratch : public femMidphaseScratch
{
public:
	const physx::Gu::BV32DataDepthInfo* PX_RESTRICT bv32DepthInfo;
	const unsigned int* PX_RESTRICT bv32RemapPackedNodeIndex;
	
};
PX_COMPILE_TIME_ASSERT(sizeof(femClothRefitMidphaseScratch) <= WARP_SIZE * 7 * sizeof(unsigned int));

#endif
