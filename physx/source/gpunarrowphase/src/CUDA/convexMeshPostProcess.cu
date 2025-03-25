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

#include "cuda.h"
#include "convexNpCommon.h"
#include "triangle.cuh"
#include "dataReadWriteHelper.cuh"
#include "RadixSort.cuh"
#include "reduction.cuh"
#include "geometry/PxGeometry.h"
#include "heightfieldUtil.cuh"
#include "convexTriangle.cuh"
#include "assert.h"
#include "stdio.h"
#include "PxgNpKernelIndices.h"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels8() {}

////Based on edge edge checks
//__device__ void convexTrimeshPostProcess(
//	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,						// per CM
//	ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr,				// per cvx-tri
//	ConvexTriEdgeBuffer** PX_RESTRICT cvxTriEdgeBufferPtr,			// per cvx-tri
//	PxU32** PX_RESTRICT cvxTriSecondPassPairsPtr,					// per cvx-tri
//	const uint4* PX_RESTRICT pairsGPU,
//	PxU32* PX_RESTRICT nbSecondPassPairs,
//	const PxU32 globalWarpIndex)
//{
//	PxU32* cvxTriSecondPassPairs = *cvxTriSecondPassPairsPtr;
//	ConvexTriNormalAndIndex* cvxTriNI = *cvxTriNIPtr;
//
//	const PxU32 pairIdx = cvxTriSecondPassPairs[globalWarpIndex];
//
//	uint4 curPair = pairsGPU[pairIdx];
//
//	PxU32 cmIdx = curPair.x;
//	PxU32 testOffset = curPair.z;
//
//	ConvexMeshPair& pair = cvxTrimeshPair[cmIdx];
//	const PxU32 nbPairsPerCM = pair.count;
//	const PxU32 pairStartIndex = pair.startIndex;
//	PxU32 convexTriPairOffset = pairStartIndex + testOffset;
//
//	ConvexTriNormalAndIndex& cvxni = cvxTriNI[convexTriPairOffset];
//	//get the number of contacts
//	const PxU32 nbContacts = cvxni.index >> 28;
//
//	if (nbContacts == 0)
//		return;
//
//	ConvexTriEdgeBuffer* cvxTriEdgeBuffer = *cvxTriEdgeBufferPtr;
//
//	ConvexTriEdgeBuffer&  cvxTriEdge = cvxTriEdgeBuffer[convexTriPairOffset];
//
//	//load the verts into shared memory
//	__shared__ int sVertsIds[2][3];
//
//	if (threadIdx.x < 3)
//	{
//		sVertsIds[threadIdx.y][threadIdx.x] = cvxTriEdge.triVertexIds[threadIdx.x];
//	}
//
//	__threadfence_block();
//
//	bool foundTriangle = false;
//	//each triangle has 3 edges, there are 9 combinations. Each thread work with one edge vs another edge
//	for (PxU32 i = 0; i < nbPairsPerCM * 9; i += WARP_SIZE)
//	{
//		foundTriangle = false;
//		const PxU32 index = i + threadIdx.x;
//
//		if (index < nbPairsPerCM * 9)
//		{
//
//			const PxU32 offset = index / 9;
//
//			//same triangle, we don't need to compare the same triangle
//			if (offset == testOffset)
//				continue;
//
//			const PxU32 vStartIndex0 = index % 3;
//			const PxU32 vEndIndex0 = (vStartIndex0 + 1) % 3;
//
//			const PxU32 vertId00 = sVertsIds[threadIdx.y][vStartIndex0];
//			const PxU32 vertId01 = sVertsIds[threadIdx.y][vEndIndex0];
//
//			const PxU32 vOffset = index / 3;
//
//			const PxU32 vStartIndex1 = (vEndIndex0 + vOffset) % 3;
//			const PxU32 vEndIndex1 = (vStartIndex1 + 1) % 3;
//
//			const PxU32 neighbourOffset = pairStartIndex + offset;
//
//			ConvexTriNormalAndIndex& nCvxni = cvxTriNI[neighbourOffset];
//			//get the number of contacts
//			const PxU32 nbGeneratedContacts = nCvxni.index >> 28;
//
//			if (nbGeneratedContacts > 0)
//			{
//				const PxU32 vertId10 = cvxTriEdgeBuffer[neighbourOffset].triVertexIds[vStartIndex1];
//				const PxU32 vertId11 = cvxTriEdgeBuffer[neighbourOffset].triVertexIds[vEndIndex1];
//
//				if ((vertId00 == vertId10 || vertId00 == vertId11) &&
//					(vertId01 == vertId10 || vertId01 == vertId11))
//				{
//					foundTriangle = true;
//				}
//			}
//		}
//
//		if (__any(foundTriangle))
//		{
//			break;
//		}
//	}
//
//	if (__any(foundTriangle))
//	{
//		//we don't need to generate contacts for this pair. However, we have already generated contacts in
//		//the convexTriangleContactGen kernel. Therefore, we just need to clear the number of contacts in
//		//the corresponding ConvexTriNormalAndIndex
//		cvxni.index = 0 << 28;
//	}
//
//}

__device__ bool findTriangle(const PxU32* PX_RESTRICT orderedTriangleIndices, const PxU32 targetIndexWithMask, PxU32 count)
{
	PxU32 targetIndex = targetIndexWithMask | 0x80000000;

	PxU32 index = binarySearch(orderedTriangleIndices, count, targetIndex);

	return index < count && orderedTriangleIndices[index] == targetIndex;
}

__device__ bool findFaceContactTriangle(const PxU32* PX_RESTRICT orderedTriangleIndices, const PxU32 targetIndexWithMask, PxU32 count)
{
	PxU32 targetIndex = targetIndexWithMask | TriangleSearchFlags::eFACE_CONTACT;

	PxU32 index = binarySearch(orderedTriangleIndices, count, targetIndex);

	return index < count && orderedTriangleIndices[index] == targetIndex;
}

__device__ bool findEdgeContactTriangle(const PxU32* PX_RESTRICT orderedTriangleIndices, const PxU32 targetIndexWithMask, PxU32 count)
{
	PxU32 targetIndex = targetIndexWithMask | TriangleSearchFlags::eEDGE_CONTACT;

	PxU32 index = binarySearch(orderedTriangleIndices, count, targetIndex);

	return index < count && orderedTriangleIndices[index] == targetIndex;
}

//Based on neighbour face checks
__device__ void convexTrimeshPostProcessCore(
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,							// per CM
	ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr,					// per cvx-tri
	ConvexTriIntermediateData** PX_RESTRICT cvxTriIntermDataPtr,		// per cvx-tri
	const PxU32** PX_RESTRICT orderedCvxTriIntermDataPtr,	// per cvx-tri
	PxU32** PX_RESTRICT cvxTriSecondPassPairsPtr,						// per cvx-tri
	PxgShape* PX_RESTRICT gpuShapes,
	const uint4* PX_RESTRICT pairsGPU,
	PxU32* PX_RESTRICT nbSecondPassPairs,
	const PxU32 globalThreadIndex)
{
	
	PxU32* cvxTriSecondPassPairs = *cvxTriSecondPassPairsPtr;
	ConvexTriNormalAndIndex* cvxTriNI = *cvxTriNIPtr;

	const PxU32 pairIdxPlusMask = cvxTriSecondPassPairs[globalThreadIndex];
	const PxU32 pairIdx = pairIdxPlusMask & 0x1fffffff;

	uint4 curPair = pairsGPU[pairIdx];

	PxU32 cmIdx = curPair.x;
	PxU32 triangleIdx = curPair.y;
	PxU32 testOffset = curPair.z;
	PxU32 shapeID = curPair.w;

	ConvexMeshPair& pair = cvxTrimeshPair[cmIdx];
	const PxU32 nbPairsPerCM = pair.count;
	assert(nbPairsPerCM != 0);

	const PxU32 pairStartIndex = pair.startIndex;
	const PxU32 orderedPairStartIndex = pair.roundedStartIndex;
	PxU32 convexTriPairOffset = pairStartIndex + testOffset;

	ConvexTriNormalAndIndex& cvxni = cvxTriNI[convexTriPairOffset];
	//get the number of contacts
	const PxU32 nbContacts = ConvexTriNormalAndIndex::getNbContacts(cvxni.index);

	if (nbContacts == 0)
		return;

	PxgShape& trimeshShape = gpuShapes[shapeID];	

	assert(trimeshShape.type == PxGeometryType::eTRIANGLEMESH);

	const PxU32* orderedTriIndices = (*orderedCvxTriIntermDataPtr) + orderedPairStartIndex;


	const PxU8 * trimeshGeomPtr = reinterpret_cast<const PxU8 *>(trimeshShape.hullOrMeshPtr);

	const uint4 trimesh_nbVerts_nbTris_nbAdjVertsTotal = *reinterpret_cast<const uint4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(uint4);

	trimeshGeomPtr += sizeof(const Gu::BV32DataPacked)* trimesh_nbVerts_nbTris_nbAdjVertsTotal.w;

	const PxU32 numVerts = trimesh_nbVerts_nbTris_nbAdjVertsTotal.x;
	const PxU32 numTris = trimesh_nbVerts_nbTris_nbAdjVertsTotal.y;

	//const float4 * trimeshVerts = reinterpret_cast<const float4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(float4)* numVerts;
	const uint4 * trimeshTriIndices = reinterpret_cast<const uint4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(uint4)* numTris;

	const uint4* PX_RESTRICT trimeshTriAdjacencies = reinterpret_cast<const uint4 *>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(uint4)* numTris;

	trimeshGeomPtr += sizeof(PxU32) * numTris; //mGRB_faceRemap

	const PxU32* accumulatedTriangleRefs = reinterpret_cast<const PxU32*>(trimeshGeomPtr);
	trimeshGeomPtr += sizeof(PxU32) * (numVerts + 1);

	const PxU32* triangleReferences = reinterpret_cast<const PxU32*>(trimeshGeomPtr);


	//ConvexTriIntermediateData* cvxTriIntermData = *cvxTriIntermDataPtr;

	//ConvexTriIntermediateData&  cvxTriInterm = cvxTriIntermData[convexTriPairOffset];

	uint4 adjTriangles = trimeshTriAdjacencies[triangleIdx];
	uint4 indices = trimeshTriIndices[triangleIdx];

	const bool flipNormal = trimeshShape.scale.hasNegativeDeterminant();
	if(flipNormal)
	{
		PxSwap(adjTriangles.x, adjTriangles.z);
		PxSwap(indices.y, indices.z);
	}

	PxU32 mask = pairIdxPlusMask & PxU32(ConvexTriIntermediateData::eMASK);

	bool needsProcessing = false;

	PxU32 triangleIndex = 0xFFFFFFFF;
	PxU32 vertexIdx = 0xFFFFFFFF;
	uint2 adjTriangles2;

	if (mask == PxU32(ConvexTriIntermediateData::eV0))
	{
		vertexIdx = indices.x;
		adjTriangles2 = make_uint2(adjTriangles.x, adjTriangles.z);
//		printf("eV0\n");
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eV1))
	{
		vertexIdx = indices.y;
		adjTriangles2 = make_uint2(adjTriangles.x, adjTriangles.y);
//		printf("eV1\n");
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eV2))
	{
		vertexIdx = indices.z;
		adjTriangles2 = make_uint2(adjTriangles.y, adjTriangles.z);
//		printf("eV2\n");
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eE01))
	{
		triangleIndex = adjTriangles.x;
//		printf("eE01\n");
		if(triangleIndex == 0xFFFFFFFF)
			needsProcessing = true;
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eE12))
	{
		triangleIndex = adjTriangles.y;
//		printf("eE12\n");
		if(triangleIndex == 0xFFFFFFFF)
			needsProcessing = true;
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eE02))
	{
		triangleIndex = adjTriangles.z;
//		printf("eE02\n");
		if(triangleIndex == 0xFFFFFFFF)
			needsProcessing = true;
	}

	//printf("triangleIdx: %d; mask: %x; triangleIndex: %d; vertexIdx: %d\n", triangleIdx, mask, triangleIndex, vertexIdx);

	if (triangleIndex != 0xFFFFFFFF)
	{
//		printf("CHECKPOINT 0\n");
		needsProcessing = isBoundaryEdge(triangleIndex) || !findFaceContactTriangle(orderedTriIndices, triangleIndex, nbPairsPerCM);
		//if (!needsProcessing)
		//	printf("triangleIdx: %d; triangleIndex: %d\n", triangleIdx, triangleIndex);
	}
	else if (vertexIdx != 0xFFFFFFFF)
	{
//		printf("CHECKPOINT 1\n");
		//First, let's see if either of the edges sharing this vertex are active. If they are, we keep the contacts, otherwise
		//we look for a shared triangle...

		needsProcessing = true;

		if(!(isBoundaryEdge(adjTriangles2.x) && isBoundaryEdge(adjTriangles2.y)))
		{
			PxU32 startIdx = accumulatedTriangleRefs[vertexIdx];
			PxU32 endIdx = accumulatedTriangleRefs[vertexIdx + 1];

			for (PxU32 idx = startIdx; idx < endIdx; ++idx)
			{
				PxU32 triIdx = triangleReferences[idx];

				if (findFaceContactTriangle(orderedTriIndices, triIdx, nbPairsPerCM) ||
					findEdgeContactTriangle(orderedTriIndices, triIdx, nbPairsPerCM))
				{
					//printf("triangleIdx: %d; vertexIdx: %d\n", triangleIdx, vertexIdx);
					needsProcessing = false;
					break;
				}
			}
		}
	}

//	printf("triangleIndex %d\n", triangleIndex);
//	printf("vertexIdx %d\n", vertexIdx);
//	printf("needsProcessing %d\n", needsProcessing);

	if(!needsProcessing)
	{
		//we don't need to generate contacts for this pair. However, we have already generated contacts in
		//the convexTriangleContactGen kernel. Therefore, we just need to clear the number of contacts in
		//the corresponding ConvexTriNormalAndIndex
		cvxni.index = cvxni.index & 0x0ffffff;
	}
}

extern "C" __global__
//__launch_bounds__(NP_TRIMESH_WARPS_PER_BLOCK * WARP_SIZE, 32 / NP_TRIMESH_WARPS_PER_BLOCK)
void convexTrimeshPostProcess(
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,						// per CM
	ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr,				// per cvx-tri
	ConvexTriIntermediateData** PX_RESTRICT cvxTriIntermPtr,		// per cvx-tri
	const PxU32** PX_RESTRICT orderedCvxTriIntermPtr,		// per cvx-tri
	PxU32** PX_RESTRICT cvxTriSecondPassPairsPtr,					// per cvx-tri
	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT nbSecondPassPairs,
	PxgShape* PX_RESTRICT shapes)
{
	const PxU32 nbTpProcess = *nbSecondPassPairs;

	__shared__ uint4* sPairsGPU;

	if (threadIdx.x == 0 && threadIdx.y == 0)
	{
		sPairsGPU = reinterpret_cast<uint4 *>(stackPtr);
	}
	__syncthreads();



	for (PxU32 globalThreadIdx = blockIdx.x * blockDim.x + threadIdx.x; globalThreadIdx < nbTpProcess; globalThreadIdx += gridDim.x * blockDim.x)
	{
		convexTrimeshPostProcessCore(
			cvxTrimeshPair,
			cvxTriNIPtr,
			cvxTriIntermPtr,
			orderedCvxTriIntermPtr,
			cvxTriSecondPassPairsPtr,
			shapes,
			sPairsGPU,
			nbSecondPassPairs,
			globalThreadIdx);
	}

}



//Based on neighbour face checks
__device__ void convexHeightfieldPostProcessCore(
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,							// per CM
	ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr,					// per cvx-tri
	ConvexTriIntermediateData** PX_RESTRICT cvxTriIntermDataPtr,		// per cvx-tri
	const PxU32** PX_RESTRICT orderedCvxTriIntermDataPtr,	// per cvx-tri
	PxU32** PX_RESTRICT cvxTriSecondPassPairsPtr,						// per cvx-tri
	PxgShape* PX_RESTRICT gpuShapes,
	const uint4* PX_RESTRICT pairsGPU,
	PxU32* PX_RESTRICT nbSecondPassPairs,
	const PxU32 globalThreadIndex)
{
	PxU32* cvxTriSecondPassPairs = *cvxTriSecondPassPairsPtr;
	ConvexTriNormalAndIndex* cvxTriNI = *cvxTriNIPtr;

	const PxU32 pairIdxPlusMask = cvxTriSecondPassPairs[globalThreadIndex];
	const PxU32 pairIdx = pairIdxPlusMask & 0x1fffffff;

	uint4 curPair = pairsGPU[pairIdx];

	PxU32 cmIdx = curPair.x;
	PxU32 triangleIdx = curPair.y;
	PxU32 testOffset = curPair.z;
	PxU32 shapeID = curPair.w;

	ConvexMeshPair& pair = cvxTrimeshPair[cmIdx];
	const PxU32 nbPairsPerCM = pair.count;
	const PxU32 pairStartIndex = pair.startIndex;
	const PxU32 orderedPairStartIndex = pair.roundedStartIndex;
	PxU32 convexTriPairOffset = pairStartIndex + testOffset;

	ConvexTriNormalAndIndex& cvxni = cvxTriNI[convexTriPairOffset];
	//get the number of contacts
	const PxU32 nbContacts = ConvexTriNormalAndIndex::getNbContacts(cvxni.index);

	if (nbContacts == 0)
		return;

	PxgShape& heightFieldShape = gpuShapes[shapeID];

	PxVec3 scale = heightFieldShape.scale.scale;

	const PxReal rowScale = scale.x;
	const PxReal columnScale = scale.z;
	PxVec3 handedness(1.0f);	// Vector to invert normal coordinates according to the heightfield scales
	bool wrongHanded = false;
	if (columnScale < 0.f)
	{
		wrongHanded = !wrongHanded;
		handedness.z = -1.0f;
	}
	if (rowScale < 0.f)
	{
		wrongHanded = !wrongHanded;
		handedness.x = -1.0f;
	}


	const PxU32* orderedTriIndices = (*orderedCvxTriIntermDataPtr) + orderedPairStartIndex;


	PxU32* heightfieldData = reinterpret_cast<PxU32*>(heightFieldShape.hullOrMeshPtr);
	const PxU32 nbRows = heightfieldData[0];
	const PxU32 nbCols = heightfieldData[1];
	PxHeightFieldSample* samples = reinterpret_cast<PxHeightFieldSample*>(&heightfieldData[2]);


	//ConvexTriIntermediateData* cvxTriIntermData = *cvxTriIntermDataPtr;

	//ConvexTriIntermediateData&  cvxTriInterm = cvxTriIntermData[convexTriPairOffset];

	PxU32 tVertexIndices[3];
	PxU32 adjacencyIndices[3];
	getTriangleVertexIndices(triangleIdx, tVertexIndices[0], tVertexIndices[1 + wrongHanded], tVertexIndices[2 - wrongHanded], nbCols, samples);

	getTriangleAdjacencyIndices(triangleIdx, adjacencyIndices[wrongHanded ? 2 : 0], adjacencyIndices[1], adjacencyIndices[wrongHanded ? 0 : 2], nbRows, nbCols, samples);


	bool needsProcessing = false;

	PxU32 triangleIndex = 0xFFFFFFFF;
	PxU32 vertexIdx = 0xFFFFFFFF;

	PxU32 mask = pairIdxPlusMask & PxU32(ConvexTriIntermediateData::eMASK);

	if (mask == PxU32(ConvexTriIntermediateData::eV0))
	{
		vertexIdx = tVertexIndices[0];
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eV1))
	{
		vertexIdx = tVertexIndices[1];
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eV2))
	{
		vertexIdx = tVertexIndices[2];
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eE01))
	{
		triangleIndex = PxU32(adjacencyIndices[0]);
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eE12))
	{
		triangleIndex = adjacencyIndices[1];
	}
	else if (mask == PxU32(ConvexTriIntermediateData::eE02))
	{
		triangleIndex = adjacencyIndices[2];
	}

	if (triangleIndex != 0xFFFFFFFF)
	{
		
		needsProcessing = isActiveEdge(triangleIndex) && !findTriangle(orderedTriIndices, triangleIndex, nbPairsPerCM);
	}
	else if (vertexIdx != 0xFFFFFFFF)
	{
		PxU32 cellIdx = triangleIdx / 2;

		PxU32 column = cellIdx % nbCols;
		PxU32 row = cellIdx / nbCols;
		PxU32 startRow = PxMax(row, 1u) - 1;
		PxU32 endRow = PxMin(row, nbRows - 2) + 1;
		PxU32 startCol = PxMax(column, 1u) - 1;
		PxU32 endCol = PxMin(column, nbCols - 2) + 1;

		needsProcessing = true;

		for (PxU32 r = startRow; r <= endRow && needsProcessing; r++)
		{
			for (PxU32 c = startCol; c <= endCol; ++c)
			{
				PxU32 cellIdx = r * nbCols + c;

				PxU32 triIdx = cellIdx * 2;
				getTriangleVertexIndices(triIdx, tVertexIndices[0], tVertexIndices[1 + wrongHanded], tVertexIndices[2 - wrongHanded], nbCols, samples);

				if (tVertexIndices[0] == vertexIdx || tVertexIndices[1] == vertexIdx || tVertexIndices[2] == vertexIdx)
				{
					if (findTriangle(orderedTriIndices, triIdx, nbPairsPerCM))
					{
						needsProcessing = false;
						break;
					}
				}

				triIdx = triIdx + 1;
				getTriangleVertexIndices(triIdx, tVertexIndices[0], tVertexIndices[1 + wrongHanded], tVertexIndices[2 - wrongHanded], nbCols, samples);

				if (tVertexIndices[0] == vertexIdx || tVertexIndices[1] == vertexIdx || tVertexIndices[2] == vertexIdx)
				{
					if (findTriangle(orderedTriIndices, triIdx, nbPairsPerCM))
					{
						needsProcessing = false;
						break;
					}
				}

			}
		}

	}

	if (!needsProcessing)
	{
		//we don't need to generate contacts for this pair. However, we have already generated contacts in
		//the convexTriangleContactGen kernel. Therefore, we just need to clear the number of contacts in
		//the corresponding ConvexTriNormalAndIndex
		cvxni.index = cvxni.index & 0x0ffffff;
	}

}


extern "C" __global__
//__launch_bounds__(NP_TRIMESH_WARPS_PER_BLOCK * WARP_SIZE, 32 / NP_TRIMESH_WARPS_PER_BLOCK)
void convexHeightfieldPostProcess(
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,						// per CM
	ConvexTriNormalAndIndex** PX_RESTRICT cvxTriNIPtr,				// per cvx-tri
	ConvexTriIntermediateData** PX_RESTRICT cvxTriIntermPtr,		// per cvx-tri
	const PxU32** PX_RESTRICT orderedCvxTriIntermPtr,		// per cvx-tri
	PxU32** PX_RESTRICT cvxTriSecondPassPairsPtr,					// per cvx-tri
	PxU8* PX_RESTRICT stackPtr,
	PxU32* PX_RESTRICT nbSecondPassPairs,
	PxgShape* PX_RESTRICT shapes)
{
	__shared__ uint4* sPairsGPU;

	if (threadIdx.x == 0 && threadIdx.y == 0)
	{
		sPairsGPU = reinterpret_cast<uint4 *>(stackPtr);
	}
	__syncthreads();

	const PxU32 nbTpProcess = *nbSecondPassPairs;

	for (PxU32 globalThreadIdx = blockIdx.x * blockDim.x + threadIdx.x; globalThreadIdx < nbTpProcess; globalThreadIdx += gridDim.x * blockDim.x)
	{
		convexHeightfieldPostProcessCore(
			cvxTrimeshPair,
			cvxTriNIPtr,
			cvxTriIntermPtr,
			orderedCvxTriIntermPtr,
			cvxTriSecondPassPairsPtr,
			shapes,
			sPairsGPU,
			nbSecondPassPairs,
			globalThreadIdx);
	}

}

extern "C" __global__
//__launch_bounds__(NP_TRIMESH_WARPS_PER_BLOCK * WARP_SIZE, 32 / NP_TRIMESH_WARPS_PER_BLOCK)
void sortTriangleIndices(
	ConvexMeshPair* PX_RESTRICT cvxTrimeshPair,	// per CM
	PxU32** PX_RESTRICT orderedCvxTriIntermPtr,	// per cvx-tri
	const PxU32 nbPairs
	)
{
	const PxU32 globalWarpIndex = threadIdx.y + blockIdx.x * blockDim.y;

	if (globalWarpIndex >= nbPairs)
		return;

	PxU32 nbTriangles = cvxTrimeshPair[globalWarpIndex].count;
	PxU32 startIndex = cvxTrimeshPair[globalWarpIndex].roundedStartIndex;

	//Terminate if we had 0 triangles or we cached them from last frame
	if (nbTriangles == 0 || nbTriangles == 0xFFFFFFFF)
		return;

	assert((startIndex & 3) == 0);

	const PxU32 nbUint4s = (nbTriangles + 3) / 4;

	const PxU32 nbWarps = NP_TRIMESH_WARPS_PER_BLOCK;

	radixSortSingleWarpKeysOnly<nbWarps>(reinterpret_cast<uint4*>(*orderedCvxTriIntermPtr + startIndex),
		nbTriangles, nbUint4s, reinterpret_cast<uint4*>(*orderedCvxTriIntermPtr + startIndex) + nbUint4s, 8);
}