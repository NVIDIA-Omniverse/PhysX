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

#include "PxgSoftBody.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include "GuTetrahedronMesh.h"
#include "cutil_math.h"
//KS - currently need to include this as we are "borrowing" some of the block math types. Need to move them
//to a common header
#include "PxgArticulation.h"
#include "PxsDeformableVolumeMaterialCore.h"

using namespace physx;

PxU32 PxgSoftBodyUtil::computeTetMeshByteSize(const Gu::BVTetrahedronMesh* tetMesh)
{
	const PxU32 meshDataSize =
		sizeof(uint4);									// (nbVerts, numTets, maxDepth, nbBv32TreeNodes)
		//+ sizeof(PxU8) * numTets						// meshTetrahedronSurfaceHint	

	//ML: don't know whether we need to have local bound
	Gu::BV32Tree* bv32Tree = tetMesh->mGRB_BV32Tree;
	const PxU32 bv32Size = bv32Tree->mNbPackedNodes * sizeof(Gu::BV32DataPacked)
		+ bv32Tree->mMaxTreeDepth * sizeof(Gu::BV32DataDepthInfo)
		+ bv32Tree->mNbPackedNodes * sizeof(PxU32);

	return meshDataSize + bv32Size;
}

static void copyTetraRestPoses(PxU16* destOrderedMaterialIndices, PxU16* destMaterialIndices, PxgMat33Block* blockRestPoses, PxMat33* tetRestPoses, PxU32* indices, PxU16* materialIndices,
	const PxU32 numTetsGM, const PxU32 numTetsPerElement, const PxU16* materialHandles)
{
	const PxU32 numElements = numTetsGM / numTetsPerElement;
	
	for (PxU32 i = 0; i < numElements; i += 32)
	{
		const PxU32 offCount = PxMin(numElements -i, 32u);

		for (PxU32 elem = 0; elem < numTetsPerElement; elem++)
		{				
			for (PxU32 off = 0; off < offCount; off++)
			{
				const PxU32 index = indices[i + off] + elem;

				const PxU32 writeIndex = (i + off + elem * numElements);

				PxgMat33Block& block = blockRestPoses[writeIndex / 32];
				PxU32 writeOffset = writeIndex & 31;

				PxMat33& mat = tetRestPoses[index];
				//PxMat33& mat = tetRestPoses[i + off];
				block.mCol0[writeOffset].x = mat.column0.x;
				block.mCol0[writeOffset].y = mat.column0.y;
				block.mCol0[writeOffset].z = mat.column0.z;
				block.mCol0[writeOffset].w = mat.column1.x;
				block.mCol1[writeOffset].x = mat.column1.y;
				block.mCol1[writeOffset].y = mat.column1.z;
				block.mCol1[writeOffset].z = mat.column2.x;
				block.mCol1[writeOffset].w = mat.column2.y;
				block.mCol2[writeOffset] = mat.column2.z;
					
				PxU16 localIndex = 0;
				if (materialIndices)
					localIndex = materialIndices[index];

				destOrderedMaterialIndices[writeIndex] = materialHandles[localIndex];
			}
		}
	}

	if (materialIndices)
	{
		for (PxU32 i = 0; i < numTetsGM; ++i)
		{
			const PxU16 localIndex = materialIndices[i];
			destMaterialIndices[i] = materialHandles[localIndex];
		}
	}
	else
	{
		for (PxU32 i = 0; i < numTetsGM; ++i)
		{
			destMaterialIndices[i] = materialHandles[0];
		}
	}
}

/*static void copyTetRemaps(uint4* dstRemaps, PxU32* srcRemaps, PxU32 numTets)
{
	for (PxU32 i = 0; i < numTets; ++i)
	{
		dstRemaps[i].x = srcRemaps[i];
		dstRemaps[i].y = srcRemaps[i+numTets];
		dstRemaps[i].z = srcRemaps[i+2*numTets];
		dstRemaps[i].w = srcRemaps[i+3*numTets];
	}
}*/

PxU32 PxgSoftBodyUtil::loadOutTetMesh(void* mem, const Gu::BVTetrahedronMesh* tetMesh)
{
	const PxU32 numTets = tetMesh->getNbTetrahedronsFast();
	const PxU32 numVerts = tetMesh->getNbVerticesFast();
	//const PxU32 numSurfaceTriangles = tetMesh->getNbTrianglesFast();

	Gu::BV32Tree* bv32Tree = tetMesh->mGRB_BV32Tree;

	PxU8* m = (PxU8*)mem;
	*((uint4*)m) = make_uint4(numVerts, numTets, bv32Tree->mMaxTreeDepth, bv32Tree->mNbPackedNodes);
	m += sizeof(uint4);

	//Midphase
	PxMemCopy(m, bv32Tree->mPackedNodes, sizeof(Gu::BV32DataPacked) * bv32Tree->mNbPackedNodes);
	m += sizeof(Gu::BV32DataPacked) * bv32Tree->mNbPackedNodes;

	PX_ASSERT(bv32Tree->mNbPackedNodes > 0);

	PxMemCopy(m, bv32Tree->mTreeDepthInfo, sizeof(Gu::BV32DataDepthInfo) * bv32Tree->mMaxTreeDepth);
	m += sizeof(Gu::BV32DataDepthInfo) * bv32Tree->mMaxTreeDepth;

	PxMemCopy(m, bv32Tree->mRemapPackedNodeIndexWithDepth, sizeof(PxU32) * bv32Tree->mNbPackedNodes);
	m += sizeof(PxU32) * bv32Tree->mNbPackedNodes;

	/*PxMemCopy(m, tetMesh->mGRB_tetraSurfaceHint, sizeof(PxU8) * numTets);
	m += numTets * sizeof(PxU8);
*/
	//GPU to CPU remap table
	//PxMemCopy(m, tetMesh->mGRB_faceRemap, numTets * sizeof(PxU32));

#if 0
	PxArray<PxBounds3> bounds(bv32Tree->mNbPackedNodes);
	const PxReal eps = 1e-6f;
	const PxReal contactOffset = 0.02f;
	const PxReal epsilon = 0.000199999995f;//we inflated the extents with epsilon in the cooking

	for (PxU32 i = bv32Tree->mMaxTreeDepth; i > 0; i--)
	{
		const PxU32 iOffset = bv32Tree->mTreeDepthInfo[i - 1].offset;
		const PxU32 iCount = bv32Tree->mTreeDepthInfo[i - 1].count;
		PxU32* iRempapNodeIndex = &bv32Tree->mRemapPackedNodeIndexWithDepth[iOffset];

		for (PxU32 j = 0; j < iCount; ++j)
		{
			const PxU32 nodeIndex = iRempapNodeIndex[j];
			Gu::BV32DataPacked& currentNode = bv32Tree->mPackedNodes[nodeIndex];
			PX_ASSERT(currentNode.mDepth == i - 1);

			PxVec3 min(PX_MAX_F32);
			PxVec3 max(-PX_MAX_F32);

			for (PxU32 k = 0; k < currentNode.mNbNodes; ++k)
			{
				if (currentNode.isLeaf(k))
				{
					PxU32 numPrimitives = currentNode.getNbReferencedPrimitives(k);
					PxU32 startIndex = currentNode.getPrimitiveStartIndex(k);

					PX_ASSERT(numPrimitives <= 32);

					PxVec3 curMin(PX_MAX_F32);
					PxVec3 curMax(-PX_MAX_F32);
					for (PxU32 l = 0; l < numPrimitives; ++l)
					{
						const PxU32 index = l + startIndex;

						uint4 tetIndex = grbTetInd[index];
						const PxVec3 worldV0 = verts[tetIndex.x];
						const PxVec3 worldV1 = verts[tetIndex.y];
						const PxVec3 worldV2 = verts[tetIndex.z];
						const PxVec3 worldV3 = verts[tetIndex.w];

						PxReal tMinX0 = PxMin(worldV0.x, worldV1.x);
						PxReal tMinY0 = PxMin(worldV0.y, worldV1.y);
						PxReal tMinZ0 = PxMin(worldV0.z, worldV1.z);

						PxReal tMinX1 = PxMin(worldV2.x, worldV3.x);
						PxReal tMinY1 = PxMin(worldV2.y, worldV3.y);
						PxReal tMinZ1 = PxMin(worldV2.z, worldV3.z);

						tMinX1 = PxMin(tMinX0, tMinX1);
						tMinY1 = PxMin(tMinY0, tMinY1);
						tMinZ1 = PxMin(tMinZ0, tMinZ1);

						curMin.x = PxMin(tMinX1, curMin.x);
						curMin.y = PxMin(tMinY1, curMin.y);
						curMin.z = PxMin(tMinZ1, curMin.z);

						//compute max
						tMinX0 = PxMax(worldV0.x, worldV1.x);
						tMinY0 = PxMax(worldV0.y, worldV1.y);
						tMinZ0 = PxMax(worldV0.z, worldV1.z);

						tMinX1 = PxMax(worldV2.x, worldV3.x);
						tMinY1 = PxMax(worldV2.y, worldV3.y);
						tMinZ1 = PxMax(worldV2.z, worldV3.z);

						tMinX1 = PxMax(tMinX0, tMinX1);
						tMinY1 = PxMax(tMinY0, tMinY1);
						tMinZ1 = PxMax(tMinZ0, tMinZ1);

						curMax.x = PxMax(tMinX1, curMax.x);
						curMax.y = PxMax(tMinY1, curMax.y);
						curMax.z = PxMax(tMinZ1, curMax.z);
					}

					min.x = PxMin(min.x, curMin.x);
					min.y = PxMin(min.y, curMin.y);
					min.z = PxMin(min.z, curMin.z);

					max.x = PxMax(max.x, curMax.x);
					max.y = PxMax(max.y, curMax.y);
					max.z = PxMax(max.z, curMax.z);

					const PxVec4 tempMin = currentNode.mMin[k];
					const PxVec4 tempMax = currentNode.mMax[k];

					PxVec3 rMin(tempMin.x, tempMin.y, tempMin.z);
					PxVec3 rMax(tempMax.x, tempMax.y, tempMax.z);

					const PxVec3 difMin = curMin - rMin;
					const PxVec3 difMax = rMax - curMax;

					PX_UNUSED(difMin);
					PX_UNUSED(difMax);

					PX_ASSERT(PxAbs(difMin.x - epsilon) < eps && PxAbs(difMin.y - epsilon) < eps && PxAbs(difMin.z - epsilon) < eps);
					PX_ASSERT(PxAbs(difMax.x - epsilon) < eps && PxAbs(difMax.y - epsilon) < eps && PxAbs(difMax.z - epsilon) < eps);
				}
				else
				{
					const PxU32 childOffset = currentNode.getChildOffset(k);

					min.x = PxMin(bounds[childOffset].minimum.x, min.x);
					min.y = PxMin(bounds[childOffset].minimum.y, min.y);
					min.z = PxMin(bounds[childOffset].minimum.z, min.z);

					max.x = PxMax(bounds[childOffset].maximum.x, max.x);
					max.y = PxMax(bounds[childOffset].maximum.y, max.y);
					max.z = PxMax(bounds[childOffset].maximum.z, max.z);
				}
			}

			bounds[nodeIndex].minimum = min;
			bounds[nodeIndex].maximum = max;
		}
	}

	bounds[0].minimum.x -= contactOffset;
	bounds[0].minimum.y -= contactOffset;
	bounds[0].minimum.z -= contactOffset;

	bounds[0].maximum.x += contactOffset;
	bounds[0].maximum.y += contactOffset;
	bounds[0].maximum.z += contactOffset;
#endif

	return bv32Tree->mNbPackedNodes;
}

void PxgSoftBodyUtil::initialTetData(PxgSoftBody& softbody, const Gu::BVTetrahedronMesh* colTetMesh, 
	const Gu::TetrahedronMesh* simTetMesh, const Gu::DeformableVolumeAuxData* softBodyAuxData, const PxU16* materialsHandles,
	PxsHeapMemoryAllocator* alloc)
{	
	const PxU32 numTets = colTetMesh->getNbTetrahedronsFast();
	uint4* tetIndices = softbody.mTetIndices;

	const PxU32 numTetsGM = simTetMesh->getNbTetrahedronsFast();
	uint4* tetIndicesGM = softbody.mSimTetIndices;

	PxMat33* tetRestPoses = softbody.mTetraRestPoses;

	const PxU32 numTetsPerElement = softBodyAuxData->mNumTetsPerElement;
	const PxU32 numElements = numTetsGM / numTetsPerElement;
	const PxU32 numVertsPerElement = numTetsPerElement == 1 ? 4 : 8;

	//copy tetrahedron indices
	if (colTetMesh->has16BitIndices())
	{
		const PxU16* tetInds = reinterpret_cast<PxU16*>(colTetMesh->mGRB_tetraIndices);
		for (PxU32 i = 0; i < numTets; ++i)
		{
			tetIndices[i].x = tetInds[4 * i + 0];
			tetIndices[i].y = tetInds[4 * i + 1];
			tetIndices[i].z = tetInds[4 * i + 2];
			tetIndices[i].w = tetInds[4 * i + 3];
		}
	}
	else
	{
		const PxU32* tetInds = reinterpret_cast<PxU32*>(colTetMesh->mGRB_tetraIndices);
		for (PxU32 i = 0; i < numTets; ++i)
		{
			tetIndices[i].x = tetInds[4 * i + 0];
			tetIndices[i].y = tetInds[4 * i + 1];
			tetIndices[i].z = tetInds[4 * i + 2];
			tetIndices[i].w = tetInds[4 * i + 3];
		}
	}
	for (PxU32 i = 0; i < numTets; ++i)
	{
		tetRestPoses[i] = softBodyAuxData->mTetraRestPoses[i];
	}

	if (simTetMesh->has16BitIndices())
	{
		const PxU16* tetIndsGM = reinterpret_cast<const PxU16*>(simTetMesh->getTetrahedrons());
		for (PxU32 i = 0; i < numTetsGM; ++i)
		{
			tetIndicesGM[i].x = tetIndsGM[4 * i + 0];
			tetIndicesGM[i].y = tetIndsGM[4 * i + 1];
			tetIndicesGM[i].z = tetIndsGM[4 * i + 2];
			tetIndicesGM[i].w = tetIndsGM[4 * i + 3];
		}
	}
	else
	{		
		const PxU32* tetIndsGM = reinterpret_cast<const PxU32*>(simTetMesh->getTetrahedrons());
		for (PxU32 i = 0; i < numTetsGM; ++i)
		{
			tetIndicesGM[i].x = tetIndsGM[4 * i + 0];
			tetIndicesGM[i].y = tetIndsGM[4 * i + 1];
			tetIndicesGM[i].z = tetIndsGM[4 * i + 2];
			tetIndicesGM[i].w = tetIndsGM[4 * i + 3];
		}
	}

	const PxU32 numVerts = colTetMesh->getNbVerticesFast();

	PxMemCopy(softbody.mTetMeshSurfaceHint, colTetMesh->mGRB_tetraSurfaceHint, sizeof(PxU8) * numTets);
		
	PxMemCopy(softbody.mTetIndicesRemapTable, colTetMesh->mGRB_faceRemap, sizeof(PxU32) * numTets);
	
	const PxU32 numVertsGM = simTetMesh->getNbVerticesFast();

	//PxMemCopy(softbody.mMaterialIndices, simTetMesh->mMaterialIndices, sizeof(PxU16) * numTetsGM);

	copyTetraRestPoses(softbody.mOrderedMaterialIndices, softbody.mMaterialIndices, softbody.mSimTetraRestPoses, softBodyAuxData->mGridModelTetraRestPoses, softBodyAuxData->mGridModelOrderedTetrahedrons, simTetMesh->mMaterialIndices,
		numTetsGM, softBodyAuxData->mNumTetsPerElement, materialsHandles);
	PxMemCopy(softbody.mSimOrderedTetrahedrons, softBodyAuxData->mGridModelOrderedTetrahedrons, sizeof(PxU32) * numElements);

	if (softBodyAuxData->mGMRemapOutputSize) // tet mesh only (or old hex mesh)
	{
		PxMemCopy(softbody.mSimRemapOutputCP, softBodyAuxData->mGMRemapOutputCP, sizeof(PxU32) * numElements * numVertsPerElement);
		PxMemCopy(softbody.mSimAccumulatedCopiesCP, softBodyAuxData->mGMAccumulatedCopiesCP, sizeof(PxU32) * numVertsGM);
	}

	PxMemCopy(softbody.mSimAccumulatedPartitionsCP, softBodyAuxData->mGMAccumulatedPartitionsCP, sizeof(PxU32) * softBodyAuxData->getNbGMPartitionFast());
	PxMemCopy(softbody.mSimPullIndices, softBodyAuxData->mGMPullIndices, sizeof(PxU32) * numElements * numVertsPerElement);
	PxMemCopy(softbody.mVertsBarycentricInGridModel, softBodyAuxData->mVertsBarycentricInGridModel, sizeof(float4) * numVerts);
	PxMemCopy(softbody.mVertsRemapInGridModel, softBodyAuxData->mVertsRemapInGridModel, sizeof(PxU32) * numVerts);
	PxMemCopy(softbody.mTetsRemapColToSim, softBodyAuxData->mTetsRemapColToSim, sizeof(PxU32) * softBodyAuxData->getNbTetRemapSizeFast());
	PxMemCopy(softbody.mTetsAccumulatedRemapColToSim, softBodyAuxData->mTetsAccumulatedRemapColToSim, sizeof(PxU32) * numTets);
		
	PxMemCopy(softbody.mSurfaceVertsHint, softBodyAuxData->mCollisionSurfaceVertsHint, sizeof(PxU8) * numVerts);
		
	PxMemCopy(softbody.mSurfaceVertToTetRemap, softBodyAuxData->mCollisionSurfaceVertToTetRemap, sizeof(PxU32) * numVerts);

	softbody.mNumTetsPerElement = softBodyAuxData->mNumTetsPerElement;
	softbody.mJacobiScale = 1.f;
	softbody.mNumJacobiVertices = 0;
	const PxU32 PULL_IND_MASK = 0x7fffffff;

	// 1. for the extra Jacobi partition of a hex mesh, store the vertices in the partition to avoid running over
	// entire softbody vertices.
	// 2. check average or max number of adjacent voxels per vertex. This will be used to scale delta x in
	// Jacobi-style update while preserving momentum.
	if(softBodyAuxData->mNumTetsPerElement > 1 && softBodyAuxData->getNbGMPartitionFast() > SB_PARTITION_LIMIT)
	{
		const PxU32 startInd = softBodyAuxData->mGMAccumulatedPartitionsCP[SB_PARTITION_LIMIT - 1];
		const PxU32 endInd = softBodyAuxData->mGMAccumulatedPartitionsCP[SB_PARTITION_LIMIT];

		const PxU32 numVoxelVertices = (endInd - startInd) * 8; // 8 vertices per element
		PX_ASSERT(endInd > startInd);
		
		PxHashMap<PxU32, PxU32> numAdjVoxels; // <vertex id, # adjacent voxels>
		const uint4* pullIndices = softbody.mSimPullIndices;

		PxArray<PxU32> jacobiVertIndices;
		jacobiVertIndices.reserve(numVoxelVertices);

		PxU32 localIndexCount = 0;
		PxU32 maxCount = 1;
		for (PxU32 i = startInd; i < endInd; ++i)
		{
			uint4 pullInd[2];
			pullInd[0] = pullIndices[i];
			pullInd[1] = pullIndices[i + numElements];
			PxU32* pullIndPtr = &pullInd[0].x;
			pullInd[0].x &= PULL_IND_MASK;

			for (PxU32 j = 0; j < 8; ++j)
			{
				if (numAdjVoxels.insert(pullIndPtr[j], 1))
				{
					++localIndexCount;
					jacobiVertIndices.pushBack(pullIndPtr[j]);
				}
				else
				{
					PxU32 count = ++numAdjVoxels[pullIndPtr[j]];
					maxCount = PxMax(maxCount, count);
				}
			}
		}
		PX_ASSERT(localIndexCount == numAdjVoxels.size());

		// Jacobi scale can be defined using
		// 1. # average adjacency
		// 2. # max adjacency
		// 3. a magic number (e.g., 0.5)

		// using average adjacency
		//softbody.mJacobiScale = PxReal(localIndexCount) / PxReal(numVoxelVertices);

		// using max adjacency
		softbody.mJacobiScale = PxMax(1.f / PxReal(maxCount), 0.2f); // lower limit of 0.2 in case there are too
		                                                                // many duplicated voxels.

		softbody.mNumJacobiVertices = localIndexCount;
		softbody.mSimJacobiVertIndices = reinterpret_cast<PxU32*>(
		    alloc->allocate(sizeof(PxU32) * localIndexCount, PxsHeapStats::eSIMULATION, PX_FL));
		PxMemCopy(softbody.mSimJacobiVertIndices, jacobiVertIndices.begin(), sizeof(PxU32) * localIndexCount);

		PX_ASSERT(numVoxelVertices >= localIndexCount);
	}

#if 0 // check GS partition validataion

	const PxVec4T<PxU32>* pullIndices = reinterpret_cast<const PxVec4T<PxU32>*>(softBodyAuxData->mGMPullIndices);

	if(softBodyAuxData->mNumTetsPerElement > 1)
	{
		PX_ASSERT(softBodyAuxData->getNbGMPartitionFast() <= (PxU32)(SB_PARTITION_LIMIT + 1));
		PxHashSet<PxU32> voxelIndices; // checking if every voxel is used for simulation, and each voxel is only
		                                // used once.
		voxelIndices.reserve(numElements);

		for(PxU32 j = 0; j < SB_PARTITION_LIMIT; ++j) // GS partition
		{
			const PxU32 startInd = j == 0 ? 0 : softBodyAuxData->mGMAccumulatedPartitionsCP[j - 1];
			const PxU32 endInd = softBodyAuxData->mGMAccumulatedPartitionsCP[j];
			PX_ASSERT(endInd >= startInd);

			PxHashSet<PxU32> voxelVertIndices; // non-overlapping voxel vertices
			voxelVertIndices.reserve(numVertsPerElement * (endInd - startInd));

			for(PxU32 elementId = startInd; elementId < endInd; ++elementId)
			{
				// for GS partitions, make sure no two vertices are in the same partition.
				PxVec4T<PxU32> pullInd[2];
				pullInd[0] = pullIndices[elementId];
				pullInd[1] = pullIndices[elementId + numElements];

				PxU32* pullIndPtr = &pullInd[0].x;
				pullInd[0].x &= PULL_IND_MASK;

				for(PxU32 localVertIndex = 0; localVertIndex < 8; ++localVertIndex)
				{
					PX_ASSERT(pullIndPtr[localVertIndex] < numVerts);
					if(!voxelVertIndices.insert(pullIndPtr[localVertIndex]))
					{
						printf("Overlapping vertices found in the same partition\n");
						PX_ASSERT(false);
					}
				}

				// make sure each voxel is used only once.
				if(!voxelIndices.insert(elementId))
				{
					printf("Same voxel is used multiple times\n");
					PX_ASSERT(false);
				}
			}

			// for GS partitions, make sure every vertex in a partition is used only once.
			if(voxelVertIndices.size() != (endInd - startInd) * numVertsPerElement)
			{
				printf("Overlapping vertices found in the same partition\n");
				PX_ASSERT(false);
			}
		}

		if(softBodyAuxData->mGMNbPartitions > SB_PARTITION_LIMIT) // jacobi partition
		{
			const PxU32 startInd = softBodyAuxData->mGMAccumulatedPartitionsCP[SB_PARTITION_LIMIT - 1];
			const PxU32 endInd = softBodyAuxData->mGMAccumulatedPartitionsCP[SB_PARTITION_LIMIT];
			PX_ASSERT(endInd >= startInd);

			PxHashSet<PxU32> voxelVertIndices; // non-overlapping voxel vertices
			voxelVertIndices.reserve(numVertsPerElement * (endInd - startInd));

			for(PxU32 elementId = startInd; elementId < endInd; ++elementId)
			{
				// make sure each voxel is used only once.
				if(!voxelIndices.insert(elementId))
				{
					printf("Same voxel is used multiple times\n");
					PX_ASSERT(false);
				}
			}
		}

		// make sure each voxel is used only once.
		if(voxelIndices.size() != numElements)
		{
			printf("Same voxel is used multiple times\n");
			PX_ASSERT(false);
		}
	}
#endif
}

void PxgSoftBodyUtil::computeBasisMatrix(PxMat33* restPoses, const Gu::DeformableVolumeMesh* tetMesh)
{
	const PxVec3* positions = tetMesh->getCollisionMeshFast()->getVerticesFast();
	const PxU32 numTets = tetMesh->getCollisionMeshFast()->getNbTetrahedronsFast();
	//copy tetrahedron indices
	if (tetMesh->getCollisionMeshFast()->has16BitIndices())
	{
		const PxU16* tetInds = reinterpret_cast<PxU16*>(tetMesh->getCollisionMeshFast()->mGRB_tetraIndices);
		for (PxU32 i = 0; i < numTets; ++i)
		{
			PxVec3 v0 = positions[tetInds[4 * i + 0]];
			PxVec3 v1 = positions[tetInds[4 * i + 1]];
			PxVec3 v2 = positions[tetInds[4 * i + 2]];
			PxVec3 v3 = positions[tetInds[4 * i + 3]];

			v1 -= v0;
			v2 -= v0;
			v3 -= v0;

			PxMat33 D = PxMat33(v1, v2, v3);
			restPoses[i] = D.getInverse();
		}
	}
	else
	{
		const PxU32* tetInds = reinterpret_cast<PxU32*>(tetMesh->getCollisionMeshFast()->mGRB_tetraIndices);
		for (PxU32 i = 0; i < numTets; ++i)
		{
			PxVec3 v0 = positions[tetInds[4 * i + 0]];
			PxVec3 v1 = positions[tetInds[4 * i + 1]];
			PxVec3 v2 = positions[tetInds[4 * i + 2]];
			PxVec3 v3 = positions[tetInds[4 * i + 3]];

			v1 -= v0;
			v2 -= v0;
			v3 -= v0;

			PxMat33 D = PxMat33(v1, v2, v3);
			restPoses[i] = D.getInverse();
		}
	}
}

PxU32 PxgSoftBody::dataIndexFromFlagDEPRECATED(PxSoftBodyGpuDataFlag::Enum flag)
{
	switch (flag)
	{
	case PxSoftBodyGpuDataFlag::eTET_INDICES:
		return PX_OFFSET_OF_RT(PxgSoftBody, mTetIndices) / sizeof(CUdeviceptr);

	case PxSoftBodyGpuDataFlag::eTET_REST_POSES:
		return PX_OFFSET_OF_RT(PxgSoftBody, mTetraRestPoses) / sizeof(CUdeviceptr);

	case PxSoftBodyGpuDataFlag::eTET_ROTATIONS:
		return PX_OFFSET_OF_RT(PxgSoftBody, mTetraRotations) / sizeof(CUdeviceptr);

	case PxSoftBodyGpuDataFlag::eTET_POSITION_INV_MASS:
		return PX_OFFSET_OF_RT(PxgSoftBody, mPosition_InvMass) / sizeof(CUdeviceptr);

	case PxSoftBodyGpuDataFlag::eSIM_TET_INDICES:
		return PX_OFFSET_OF_RT(PxgSoftBody, mSimTetIndices) / sizeof(CUdeviceptr);

	case PxSoftBodyGpuDataFlag::eSIM_TET_ROTATIONS:
		return PX_OFFSET_OF_RT(PxgSoftBody, mSimTetraRotations) / sizeof(CUdeviceptr);

	case PxSoftBodyGpuDataFlag::eSIM_VELOCITY_INV_MASS:
		return PX_OFFSET_OF_RT(PxgSoftBody, mSimVelocity_InvMass) / sizeof(CUdeviceptr);

	case PxSoftBodyGpuDataFlag::eSIM_POSITION_INV_MASS:
		return PX_OFFSET_OF_RT(PxgSoftBody, mSimPosition_InvMass) / sizeof(CUdeviceptr);
	}
	PX_ASSERT(false);
	return 0;
}

void PxgSoftBody::deallocate(PxsHeapMemoryAllocator* allocator)
{
	allocator->deallocate(mTetMeshData);
	allocator->deallocate(mTetMeshSurfaceHint);
	allocator->deallocate(mTetIndices);
	allocator->deallocate(mTetIndicesRemapTable);
	allocator->deallocate(mTetraRestPoses);
	allocator->deallocate(mSimTetIndices);
	allocator->deallocate(mSimTetraRestPoses);
	allocator->deallocate(mSimOrderedTetrahedrons);
	allocator->deallocate(mVertsBarycentricInGridModel);
	allocator->deallocate(mVertsRemapInGridModel);
	allocator->deallocate(mTetsRemapColToSim);
	allocator->deallocate(mTetsAccumulatedRemapColToSim);
	allocator->deallocate(mSurfaceVertsHint);
	allocator->deallocate(mSurfaceVertToTetRemap);
	allocator->deallocate(mSimAccumulatedPartitionsCP);
	allocator->deallocate(mSimPullIndices);
	allocator->deallocate(mOrderedMaterialIndices);
	allocator->deallocate(mMaterialIndices);

	if (mNumTetsPerElement == 1) // used for tet mesh only
	{
		allocator->deallocate(mSimRemapOutputCP);
		allocator->deallocate(mSimAccumulatedCopiesCP);
	}

	if (mNumJacobiVertices) // when Jacobi vertices are used, deallocate mSimJacobiVertIndices.
	{
		allocator->deallocate(mSimJacobiVertIndices);
	}
}
