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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.


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
