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

#ifndef __TRIANGLE_MESH_CUH__
#define __TRIANGLE_MESH_CUH__

#include "GuBV32.h"
#include "PxgConvexConvexShape.h"
#include "foundation/PxVec3.h"

namespace physx
{

struct PxgTriangleMesh
{
	PxU32 numVerts;
	PxU32 numTris;
	PxU32 numBv32TreeNodes;
	PxU32 nbPackedNodes;

	const Gu::BV32DataPacked* bv32PackedNodes;

	const float4* trimeshVerts;
	const uint4* indices;

	const PxU32* PX_RESTRICT trimeshFaceRemap;

	uint4 sdfDims;
	PxVec3 meshLower;
	PxReal spacing;
	PxU32 subgridSize;
	PxReal subgridsMinSdfValue;
	PxReal subgridsMaxSdfValue;
	PxU32 pad;

	//const PxReal* sdf;
	CUtexObject mTexObject;
	CUtexObject mTexObjectSparse;

	const PxU32* subgridStarts;
};

static PX_FORCE_INLINE __device__ uint4 readTriangleMesh(const PxU8* PX_RESTRICT & trimeshGeomPtr,
	const Gu::BV32DataPacked* PX_RESTRICT & bv32PackedNodes, const float4* PX_RESTRICT & trimeshVerts)
{
	const uint4 trimesh_nbVerts_nbTris_nbAdjVertsTotal = *reinterpret_cast<const uint4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(uint4);

	bv32PackedNodes = reinterpret_cast<const Gu::BV32DataPacked*>(trimeshGeomPtr);

	trimeshGeomPtr += sizeof(const Gu::BV32DataPacked)* trimesh_nbVerts_nbTris_nbAdjVertsTotal.w;

	trimeshVerts = reinterpret_cast<const float4 *>(trimeshGeomPtr);

	return trimesh_nbVerts_nbTris_nbAdjVertsTotal;
}

static PX_FORCE_INLINE __device__ uint4 readTriangleMesh(const PxU8 * PX_RESTRICT & trimeshGeomPtr,
	const Gu::BV32DataPacked* PX_RESTRICT & bv32PackedNodes, const float4* PX_RESTRICT & trimeshVerts, const uint4* PX_RESTRICT & trimeshTriIndices)
{
	uint4 result = readTriangleMesh(trimeshGeomPtr, bv32PackedNodes, trimeshVerts);

	trimeshGeomPtr += sizeof(float4) * result.x;
	trimeshTriIndices = reinterpret_cast<const uint4 *>(trimeshGeomPtr);

	return result;
}

static PX_FORCE_INLINE __device__ uint4 readTriangleMesh(const PxU8 * PX_RESTRICT & trimeshGeomPtr, const float4* PX_RESTRICT & trimeshVerts, const uint4* PX_RESTRICT & trimeshTriIndices,
	const uint4* PX_RESTRICT & trimeshTriAdjacencies)
{
	const Gu::BV32DataPacked* bv32PackedNodes;

	uint4 result = readTriangleMesh(trimeshGeomPtr, bv32PackedNodes, trimeshVerts, trimeshTriIndices);

	trimeshGeomPtr += sizeof(uint4)* result.y;

	trimeshTriAdjacencies = reinterpret_cast<const uint4 *>(trimeshGeomPtr);

	return result;
}

static PX_FORCE_INLINE __device__ uint4 readTriangleMesh(const PxU8 * PX_RESTRICT & trimeshGeomPtr, const float4* PX_RESTRICT & trimeshVerts, const uint4* PX_RESTRICT & trimeshTriIndices,
	const uint4* PX_RESTRICT & trimeshTriAdjacencies, const PxU32* PX_RESTRICT & trimeshFaceRemap)
{
	uint4 result = readTriangleMesh(trimeshGeomPtr, trimeshVerts, trimeshTriIndices, trimeshTriAdjacencies);

	trimeshGeomPtr += sizeof(uint4)* result.y;

	trimeshFaceRemap = reinterpret_cast<const PxU32 *>(trimeshGeomPtr);

	return result;
}

static __device__ void readTriangleMesh(const PxgShape& trimeshShape, PxgTriangleMesh& triangleMesh)
{
	const PxU8 * trimeshGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);

	const uint4 nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes = *reinterpret_cast<const uint4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(uint4);

	triangleMesh.numVerts = nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.x;
	triangleMesh.numTris = nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.y;
	triangleMesh.numBv32TreeNodes = nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.z;
	triangleMesh.nbPackedNodes = nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.w;

	triangleMesh.bv32PackedNodes = reinterpret_cast<const Gu::BV32DataPacked*>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(const Gu::BV32DataPacked)* nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.w;

	triangleMesh.trimeshVerts = reinterpret_cast<const float4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(float4) * nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.x;

	triangleMesh.indices = reinterpret_cast<const uint4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(uint4) * nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.y;

	//const uint4* PX_RESTRICT trimeshTriAdjacencies = reinterpret_cast<const uint4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(uint4)* nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.y;

	triangleMesh.trimeshFaceRemap = reinterpret_cast<const PxU32 *>(trimeshGeomPtr);
	//trimeshGeomPtr += ((nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.y + 3)& (~3)) * sizeof(PxU32);
	trimeshGeomPtr += nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.y * sizeof(PxU32);

	//triMesh->mAccumulatedTrianglesRef
	trimeshGeomPtr += (nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.x + 1) * sizeof(PxU32);

	//triMesh->mTrianglesReferences
	trimeshGeomPtr += (nbVerts_nbTris_nbAdjVertsTotal_nbBv32TreeNodes.y * 3) * sizeof(PxU32);

	trimeshGeomPtr = (PxU8*)(((size_t)trimeshGeomPtr + 15) & ~15);


	uint4 sdfDims = *reinterpret_cast<const uint4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(uint4);

	PxU32 numSdfs = sdfDims.x*sdfDims.y*sdfDims.z;

	triangleMesh.sdfDims = sdfDims;

	if (numSdfs)
	{
		float4 meshLower_spacing = *reinterpret_cast<const float4 *>(trimeshGeomPtr);
		trimeshGeomPtr += sizeof(float4);
		uint4 sizeInfo = *reinterpret_cast<const uint4 *>(trimeshGeomPtr);
		trimeshGeomPtr += sizeof(uint4);

		triangleMesh.meshLower = PxVec3(meshLower_spacing.x, meshLower_spacing.y, meshLower_spacing.z);
		triangleMesh.spacing = meshLower_spacing.w;

		/*PxU32 subgridSize;
		PxU32 unused0;
		PxU32 unused1;
		PxU32 numCoarseGridElements;*/
		triangleMesh.subgridSize = sizeInfo.x;
		triangleMesh.subgridsMinSdfValue = reinterpret_cast<PxReal&>(sizeInfo.y);
		triangleMesh.subgridsMaxSdfValue = reinterpret_cast<PxReal&>(sizeInfo.z);
		//triangleMesh.numCoarseGridElements = sizeInfo.w;

		//triangleMesh.sdf = reinterpret_cast<const PxReal*>(trimeshGeomPtr);
		triangleMesh.mTexObject = *reinterpret_cast<const CUtexObject*>(trimeshGeomPtr);
		trimeshGeomPtr += sizeof(const CUtexObject);
		triangleMesh.mTexObjectSparse = *reinterpret_cast<const CUtexObject*>(trimeshGeomPtr);
		trimeshGeomPtr += sizeof(const CUtexObject);

		triangleMesh.subgridStarts = reinterpret_cast<const PxU32 *>(trimeshGeomPtr);

		trimeshGeomPtr += sizeof(PxU32) * sizeInfo.w;
	}
	else
		triangleMesh.mTexObject = 0;
}

}
#endif
