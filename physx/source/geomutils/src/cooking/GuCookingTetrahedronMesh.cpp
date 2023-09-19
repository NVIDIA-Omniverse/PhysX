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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved. 

#define USE_GJK_VIRTUAL

#include "GuCookingTetrahedronMesh.h"
#include "GuTetrahedron.h"
#include "GuInternal.h"
#include "foundation/PxHashMap.h"
#include "GuCookingTriangleMesh.h"
#include "GuBV4Build.h"
#include "GuBV32Build.h"
#include "GuDistancePointTetrahedron.h"
#ifdef USE_GJK_VIRTUAL
	#include "GuGJKTest.h"
#else
	#include "GuGJKUtil.h"
	#include "GuGJK.h"
#endif
#include "GuVecTetrahedron.h"
#include "GuGJKType.h"
#include "GuCooking.h"
#include "GuBounds.h"
#include "CmSerialize.h"
#include "foundation/PxFPU.h"
#include "common/PxInsertionCallback.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

using namespace physx;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*TetrahedronMeshBuilder::TetrahedronMeshBuilder(const PxCookingParams& params) : mParams(params)
{

}*/

void TetrahedronMeshBuilder::recordTetrahedronIndices(const TetrahedronMeshData& collisionMesh, SoftBodyCollisionData& collisionData, bool buildGPUData)
{
	if (buildGPUData)
	{
		PX_ASSERT(!(collisionMesh.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));
		PX_ASSERT(collisionData.mGRB_primIndices);

		//copy the BV4 tetrahedron indices to mGRB_primIndices
		PxMemCopy(collisionData.mGRB_primIndices, collisionMesh.mTetrahedrons, sizeof(IndTetrahedron32) * collisionMesh.mNbTetrahedrons);
	}
}

class SortedTriangleInds
{
public:

	SortedTriangleInds() {}

	SortedTriangleInds(const PxU32 ref0, const PxU32 ref1, const PxU32 ref2)
	{
		initialize(ref0, ref1, ref2);
	}

	SortedTriangleInds(const PxU16 ref0, const PxU16 ref1, const PxU16 ref2)
	{
		initialize(PxU32(ref0), PxU32(ref1), PxU32(ref2));
	}

	void initialize(const PxU32 ref0, const PxU32 ref1, const PxU32 ref2)
	{
		mOrigRef[0] = ref0;
		mOrigRef[1] = ref1;
		mOrigRef[2] = ref2;
		if (ref0 < ref1 && ref0 < ref2)
		{
			mRef0 = ref0;
			mRef1 = PxMin(ref1, ref2);
			mRef2 = PxMax(ref1, ref2);
		}
		else if (ref1 < ref2)
		{
			mRef0 = ref1;
			mRef1 = PxMin(ref0, ref2);
			mRef2 = PxMax(ref0, ref2);
		}
		else
		{
			mRef0 = ref2;
			mRef1 = PxMin(ref0, ref1);
			mRef2 = PxMax(ref0, ref1);
		}
	}

	bool operator == (const SortedTriangleInds& other) const
	{
		return other.mRef0 == mRef0 && other.mRef1 == mRef1 && other.mRef2 == mRef2;
	}

	static uint32_t hash(const SortedTriangleInds key)
	{
		uint64_t k0 = (key.mRef0 & 0xffff);
		uint64_t k1 = (key.mRef1 & 0xffff);
		uint64_t k2 = (key.mRef2 & 0xffff);

		uint64_t k = (k2 << 32) | (k1 << 16) | k0;
		k += ~(k << 32);
		k ^= (k >> 22);
		k += ~(k << 13);
		k ^= (k >> 8);
		k += (k << 3);
		k ^= (k >> 15);
		k += ~(k << 27);
		k ^= (k >> 31);
		return uint32_t(UINT32_MAX & k);
	}

	void setTetIndex(const PxU32 tetIndex)
	{
		mTetIndex = tetIndex;
	}

	PxU32 getTetIndex()
	{
		return mTetIndex;
	}

	PxU32 mOrigRef[3];

	PxU32 mRef0;
	PxU32 mRef1;
	PxU32 mRef2;

	PxU32 mTetIndex;
};

struct SortedTriangleIndsHash
{
	uint32_t operator()(const SortedTriangleInds& k) const
	{
		return SortedTriangleInds::hash(k);
	}
	bool equal(const SortedTriangleInds& k0, const SortedTriangleInds& k1) const
	{
		return k0 == k1;
	}
};

#if PX_CHECKED
bool checkInputFloats(PxU32 nb, const float* values, const char* file, PxU32 line, const char* errorMsg);
#endif

bool TetrahedronMeshBuilder::importMesh(const PxTetrahedronMeshDesc& collisionMeshDesc, const PxCookingParams& params, 
	TetrahedronMeshData& collisionMesh, SoftBodyCollisionData& collisionData, bool validateMesh)
{
	PX_UNUSED(validateMesh);
	//convert and clean the input mesh
	//this is where the mesh data gets copied from user mem to our mem

	PxVec3* verts = collisionMesh.allocateVertices(collisionMeshDesc.points.count);

	collisionMesh.allocateTetrahedrons(collisionMeshDesc.tetrahedrons.count, 1);
	if (params.buildGPUData)
		collisionData.allocateCollisionData(collisionMeshDesc.tetrahedrons.count);
		
	TetrahedronT<PxU32>* tets = reinterpret_cast<TetrahedronT<PxU32>*>(collisionMesh.mTetrahedrons);

	//copy, and compact to get rid of strides:
	immediateCooking::gatherStrided(collisionMeshDesc.points.data, verts, collisionMesh.mNbVertices, sizeof(PxVec3), collisionMeshDesc.points.stride);	
		
#if PX_CHECKED
	// PT: check all input vertices are valid
	if(!checkInputFloats(collisionMeshDesc.points.count*3, &verts->x, PX_FL, "input mesh contains corrupted vertex data"))
		return false;
#endif

	TetrahedronT<PxU32>* dest = tets;
	const TetrahedronT<PxU32>* pastLastDest = tets + collisionMesh.mNbTetrahedrons;
	const PxU8* source = reinterpret_cast<const PxU8*>(collisionMeshDesc.tetrahedrons.data);

	PX_ASSERT(source);

	//4 combos of 16 vs 32, feed in collisionMesh.mTetrahedrons
	if (collisionMeshDesc.flags & PxMeshFlag::e16_BIT_INDICES)
	{
		while (dest < pastLastDest)
		{
			const PxU16 *tet16 = reinterpret_cast<const PxU16*>(source);
			dest->v[0] = tet16[0];
			dest->v[1] = tet16[1];
			dest->v[2] = tet16[2];
			dest->v[3] = tet16[3];
			dest++;
			source += collisionMeshDesc.tetrahedrons.stride;
		}
	}
	else
	{
		while (dest < pastLastDest)
		{
			const PxU32 * tet32 = reinterpret_cast<const PxU32*>(source);
			dest->v[0] = tet32[0];
			dest->v[1] = tet32[1];
			dest->v[2] = tet32[2];
			dest->v[3] = tet32[3];
			dest++;
			source += collisionMeshDesc.tetrahedrons.stride;
		}
	}

	//copy the material index list if any:
	if (collisionMeshDesc.materialIndices.data)
	{
		PxFEMMaterialTableIndex* materials = collisionMesh.allocateMaterials();
		immediateCooking::gatherStrided(collisionMeshDesc.materialIndices.data, materials, collisionMesh.mNbTetrahedrons, sizeof(PxMaterialTableIndex), collisionMeshDesc.materialIndices.stride);

		// Check material indices
		for (PxU32 i = 0; i < collisionMesh.mNbTetrahedrons; i++)	PX_ASSERT(materials[i] != 0xffff);
	}

	// we need to fill the remap table if no cleaning was done
	if (params.suppressTriangleMeshRemapTable == false)
	{
		PX_ASSERT(collisionData.mFaceRemap == NULL);
		collisionData.mFaceRemap = PX_ALLOCATE(PxU32, collisionMesh.mNbTetrahedrons, "mFaceRemap");
		for (PxU32 i = 0; i < collisionMesh.mNbTetrahedrons; i++)
			collisionData.mFaceRemap[i] = i;
	}

	return true;
}

bool TetrahedronMeshBuilder::createGRBMidPhaseAndData(const PxU32 originalTetrahedronCount, TetrahedronMeshData& collisionMesh, SoftBodyCollisionData& collisionData, const PxCookingParams& params)
{
	PX_UNUSED(originalTetrahedronCount);
	if (params.buildGPUData)
	{
		PX_ASSERT(!(collisionMesh.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));

		BV32Tree* bv32Tree = PX_NEW(BV32Tree);
		collisionData.mGRB_BV32Tree = bv32Tree;

		if(!BV32TetrahedronMeshBuilder::createMidPhaseStructure(params, collisionMesh, *bv32Tree, collisionData))
			return false;

		//create surface triangles, one tetrahedrons has 4 triangles
		PxHashMap<SortedTriangleInds, PxU32, SortedTriangleIndsHash> triIndsMap;

		//for trigs index stride conversion and eventual reordering is also needed, I don't think flexicopy can do that for us.

		IndTetrahedron32* dest = reinterpret_cast<IndTetrahedron32*>(collisionData.mGRB_primIndices);
			
		for(PxU32 i = 0; i < collisionMesh.mNbTetrahedrons; ++i)
		{
			IndTetrahedron32& tetInd = dest[i];

			SortedTriangleInds t0(tetInd.mRef[0], tetInd.mRef[1], tetInd.mRef[2]);
			t0.setTetIndex(i);
			triIndsMap[t0] += 1;

			SortedTriangleInds t1(tetInd.mRef[1], tetInd.mRef[3], tetInd.mRef[2]);
			t1.setTetIndex(i);
			triIndsMap[t1] += 1;

			SortedTriangleInds t2(tetInd.mRef[0], tetInd.mRef[3], tetInd.mRef[1]);
			t2.setTetIndex(i);
			triIndsMap[t2] += 1;

			SortedTriangleInds t3(tetInd.mRef[0], tetInd.mRef[2], tetInd.mRef[3]);
			t3.setTetIndex(i);
			triIndsMap[t3] += 1;
		}

		PxMemZero(collisionData.mGRB_tetraSurfaceHint, collisionMesh.mNbTetrahedrons * sizeof(PxU8));

		PxU8* tetHint = reinterpret_cast<PxU8*>(collisionData.mGRB_tetraSurfaceHint);

		PxU32 triCount = 0;
		//compute the surface triangles for the tetrahedron mesh
		for (PxHashMap<SortedTriangleInds, PxU32, SortedTriangleIndsHash>::Iterator iter = triIndsMap.getIterator(); !iter.done(); ++iter)
		{
			SortedTriangleInds key = iter->first;

			// only output faces that are referenced by one tet (open faces)
			if (iter->second == 1)
			{
				PxU8 triHint = 0;
				IndTetrahedron32& localTetra = dest[key.mTetIndex];
				for (PxU32 i = 0; i < 3; ++i)
				{
					if (key.mOrigRef[i] == localTetra.mRef[0])
						triHint |= 1;
					else if (key.mOrigRef[i] == localTetra.mRef[1])
						triHint |= (1 << 1);
					else if (key.mOrigRef[i] == localTetra.mRef[2])
						triHint |= (1 << 2);
					else if (key.mOrigRef[i] == localTetra.mRef[3])
						triHint |= (1 << 3);
				}

				//if this tetrahedron isn't surface tetrahedron, hint will be zero
				//otherwise, the first 4 bits will indicate the indice of the
				//surface triangle

				PxU32 mask = 0;
				if (triHint == 7) //0111
				{
					mask = 1 << 0;
				}
				else if (triHint == 11)//1011
				{
					mask = 1 << 1;
				}
				else if (triHint == 13)//1101
				{
					mask = 1 << 2;
				}
				else //1110
				{
					mask = 1 << 3;
				}

				tetHint[key.mTetIndex] |= mask;

				triCount++;
			}
		}
			
#if BV32_VALIDATE
		IndTetrahedron32* grbTriIndices = reinterpret_cast<IndTetrahedron32*>(collisionData.mGRB_primIndices);
		IndTetrahedron32* cpuTriIndices = reinterpret_cast<IndTetrahedron32*>(collisionMesh.mTetrahedrons);
		//map CPU remap triangle index to GPU remap triangle index
		for (PxU32 i = 0; i < nbTetrahedrons; ++i)
		{
			PX_ASSERT(grbTriIndices[i].mRef[0] == cpuTriIndices[collisionData.mGRB_faceRemap[i]].mRef[0]);
			PX_ASSERT(grbTriIndices[i].mRef[1] == cpuTriIndices[collisionData.mGRB_faceRemap[i]].mRef[1]);
			PX_ASSERT(grbTriIndices[i].mRef[2] == cpuTriIndices[collisionData.mGRB_faceRemap[i]].mRef[2]);
			PX_ASSERT(grbTriIndices[i].mRef[3] == cpuTriIndices[collisionData.mGRB_faceRemap[i]].mRef[3]);
		}
#endif
	}
	return true;
}

void computeRestPoseAndPointMass(TetrahedronT<PxU32>* tetIndices, const PxU32 nbTets, 
	const PxVec3* verts, PxReal* invMasses, PxMat33* restPoses)
{
	for (PxU32 i = 0; i < nbTets; ++i)
	{
		TetrahedronT<PxU32>& tetInd = tetIndices[i];
		PxMat33 Q, QInv;
		const PxReal volume = computeTetrahedronVolume(verts[tetInd.v[0]], verts[tetInd.v[1]], verts[tetInd.v[2]], verts[tetInd.v[3]], Q);
		if (volume <= 1.e-9f)
		{
			//Neo-hookean model can deal with bad tets, so not issueing this error anymore
			//PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "computeRestPoseAndPointMass(): tretrahedron is degenerate or inverted");
			if (volume == 0)
				QInv = PxMat33(PxZero);
			else
				QInv = Q.getInverse();
		}
		else
			QInv = Q.getInverse();

		// add volume fraction to particles
		if (invMasses != NULL) 
		{
			invMasses[tetInd.v[0]] += volume * 0.25f;
			invMasses[tetInd.v[1]] += volume * 0.25f;
			invMasses[tetInd.v[2]] += volume * 0.25f;
			invMasses[tetInd.v[3]] += volume * 0.25f;
		}

		restPoses[i] = QInv;
	}
}

#define MAX_NUM_PARTITIONS 32

PxU32 computeTetrahedronPartition(const TetrahedronT<PxU32>* tets, const PxU32 partitionStartIndex, PxU32* partitionProgresses,
	const PxU32 numTetsPerElement)
{
	PxU32 combinedMask = 0xFFFFFFFF;

	for (PxU32 i = 0; i < numTetsPerElement; ++i)
	{
		PxU32 partitionA = partitionProgresses[tets[i].v[0]];
		PxU32 partitionB = partitionProgresses[tets[i].v[1]];
		PxU32 partitionC = partitionProgresses[tets[i].v[2]];
		PxU32 partitionD = partitionProgresses[tets[i].v[3]];

		combinedMask &= (~partitionA & ~partitionB & ~partitionC & ~partitionD);
	}
	PxU32 availablePartition = combinedMask == 0 ? MAX_NUM_PARTITIONS : PxLowestSetBit(combinedMask);

	if (availablePartition == MAX_NUM_PARTITIONS)
		return 0xFFFFFFFF;

	const PxU32 partitionBit = (1u << availablePartition);

	for (PxU32 i = 0; i < numTetsPerElement; ++i)
	{
		PxU32 partitionA = partitionProgresses[tets[i].v[0]];
		PxU32 partitionB = partitionProgresses[tets[i].v[1]];
		PxU32 partitionC = partitionProgresses[tets[i].v[2]];
		PxU32 partitionD = partitionProgresses[tets[i].v[3]];

		partitionA |= partitionBit;
		partitionB |= partitionBit;
		partitionC |= partitionBit;
		partitionD |= partitionBit;

		partitionProgresses[tets[i].v[0]] = partitionA;
		partitionProgresses[tets[i].v[1]] = partitionB;
		partitionProgresses[tets[i].v[2]] = partitionC;
		partitionProgresses[tets[i].v[3]] = partitionD;
	}

	availablePartition += partitionStartIndex;

	return availablePartition;
}

void classifyTetrahedrons(const TetrahedronT<PxU32>* tets, const PxU32 numTets, const PxU32 numVerts, const PxU32 numTetsPerElement,
	PxU32* partitionProgresses, PxU32* tempTetrahedrons, PxArray<PxU32>& tetrahedronsPerPartition)
{
	//initialize the partition progress counter to be zero
	PxMemZero(partitionProgresses, sizeof(PxU32) * numVerts);

	PxU32 numUnpartitionedTetrahedrons = 0;
	//compute partitions for each tetrahedron in the grid model

	for (PxU32 i = 0; i < numTets; i += numTetsPerElement)
	{
		const TetrahedronT<PxU32>* tet = &tets[i];

		const PxU32 availablePartition = computeTetrahedronPartition(tet, 0, partitionProgresses, numTetsPerElement);

		if (availablePartition == 0xFFFFFFFF)
		{
			tempTetrahedrons[numUnpartitionedTetrahedrons++] = i;
			continue;
		}

		tetrahedronsPerPartition[availablePartition]++;
	}

	PxU32 partitionStartIndex = 0;

	while (numUnpartitionedTetrahedrons > 0)
	{
		//initialize the partition progress counter to be zero
		PxMemZero(partitionProgresses, sizeof(PxU32) * numVerts);

		partitionStartIndex += MAX_NUM_PARTITIONS;
		//Keep partitioning the un-partitioned constraints and blat the whole thing to 0!
		tetrahedronsPerPartition.resize(MAX_NUM_PARTITIONS + tetrahedronsPerPartition.size());
		PxMemZero(tetrahedronsPerPartition.begin() + partitionStartIndex, sizeof(PxU32) * MAX_NUM_PARTITIONS);

		PxU32 newNumUnpartitionedConstraints = 0;

		for (PxU32 i = 0; i < numUnpartitionedTetrahedrons; ++i)
		{
			const PxU32 tetInd = tempTetrahedrons[i];

			const TetrahedronT<PxU32>* tet = &tets[tetInd];

			const PxU32 availablePartition = computeTetrahedronPartition(tet, partitionStartIndex, partitionProgresses, numTetsPerElement);
				
			if (availablePartition == 0xFFFFFFFF)
			{
				tempTetrahedrons[newNumUnpartitionedConstraints++] = tetInd;
				continue;
			}

			tetrahedronsPerPartition[availablePartition]++;
		}

		numUnpartitionedTetrahedrons = newNumUnpartitionedConstraints;
	}
}

void writeTetrahedrons(const TetrahedronT<PxU32>* tets, const PxU32 numTets, const PxU32 numVerts, const PxU32 numTetsPerElement,
	PxU32* partitionProgresses, PxU32* tempTetrahedrons, PxU32* orderedTetrahedrons, 
	PxU32* accumulatedTetrahedronPerPartition)
{
	//initialize the partition progress counter to be zero
	PxMemZero(partitionProgresses, sizeof(PxU32) * numVerts);

	PxU32 numUnpartitionedTetrahedrons = 0;

	for (PxU32 i = 0; i < numTets; i += numTetsPerElement)
	{
		const TetrahedronT<PxU32>* tet = &tets[i];

		const PxU32 availablePartition = computeTetrahedronPartition(tet, 0, partitionProgresses, numTetsPerElement);

		if (availablePartition == 0xFFFFFFFF)
		{
			tempTetrahedrons[numUnpartitionedTetrahedrons++] = i;
			continue;
		}

		//output tetrahedron
		orderedTetrahedrons[accumulatedTetrahedronPerPartition[availablePartition]++] = i;
	}

	PxU32 partitionStartIndex = 0;

	while (numUnpartitionedTetrahedrons > 0)
	{
		//initialize the partition progress counter to be zero
		PxMemZero(partitionProgresses, sizeof(PxU32) * numVerts);

		partitionStartIndex += MAX_NUM_PARTITIONS;

		PxU32 newNumUnpartitionedConstraints = 0;

		for (PxU32 i = 0; i < numUnpartitionedTetrahedrons; ++i)
		{
			const PxU32 tetInd = tempTetrahedrons[i];
			const TetrahedronT<PxU32>* tet = &tets[tetInd];

			const PxU32 availablePartition = computeTetrahedronPartition(tet, partitionStartIndex, partitionProgresses, numTetsPerElement);

			if (availablePartition == 0xFFFFFFFF)
			{
				tempTetrahedrons[newNumUnpartitionedConstraints++] = tetInd;
				continue;
			}

			//output tetrahedrons
			orderedTetrahedrons[accumulatedTetrahedronPerPartition[availablePartition]++] = tetInd;
		}

		numUnpartitionedTetrahedrons = newNumUnpartitionedConstraints;
	}
}

PxU32* computeGridModelTetrahedronPartitions(const TetrahedronMeshData& simulationMesh, SoftBodySimulationData& simulationData)
{
	const PxU32 numTets = simulationMesh.mNbTetrahedrons;
	const PxU32 numVerts = simulationMesh.mNbVertices;

	//each grid model verts has a partition progress counter
	PxU32* partitionProgresses = PX_ALLOCATE(PxU32, numVerts, "partitionProgress");

	//this store the tetrahedron index for the unpartitioned tetrahedrons 
	PxU32* tempTetrahedrons = PX_ALLOCATE(PxU32, numTets, "tempTetrahedrons");

	PxArray<PxU32> tetrahedronsPerPartition;
	tetrahedronsPerPartition.reserve(MAX_NUM_PARTITIONS);
	tetrahedronsPerPartition.forceSize_Unsafe(MAX_NUM_PARTITIONS);
		
	PxMemZero(tetrahedronsPerPartition.begin(), sizeof(PxU32) * MAX_NUM_PARTITIONS);

	const TetrahedronT<PxU32>* tetGM = reinterpret_cast<TetrahedronT<PxU32>*>(simulationMesh.mTetrahedrons);
		
	classifyTetrahedrons(tetGM, numTets, numVerts, simulationData.mNumTetsPerElement, partitionProgresses,
		tempTetrahedrons, tetrahedronsPerPartition);

	//compute number of partitions
	PxU32 maxPartition = 0;
	for (PxU32 a = 0; a < tetrahedronsPerPartition.size(); ++a, maxPartition++)
	{
		if (tetrahedronsPerPartition[a] == 0)
			break;
	}
	
	PxU32* accumulatedTetrahedronPerPartition = PX_ALLOCATE(PxU32, maxPartition, "accumulatedTetrahedronPerPartition");

	//compute run sum
	PxU32 accumulation = 0;
	for (PxU32 a = 0; a < maxPartition; ++a)
	{
		PxU32 count = tetrahedronsPerPartition[a];
		accumulatedTetrahedronPerPartition[a] = accumulation;
		accumulation += count;
	}

	PX_ASSERT(accumulation*simulationData.mNumTetsPerElement == numTets);

	simulationData.mGridModelOrderedTetrahedrons = PX_ALLOCATE(PxU32, numTets, "mGridModelPartitionTetrahedrons");
	simulationData.mGridModelNbPartitions = maxPartition;

	PxU32* orderedTetrahedrons = simulationData.mGridModelOrderedTetrahedrons;

	writeTetrahedrons(tetGM, numTets, numVerts, simulationData.mNumTetsPerElement, partitionProgresses, tempTetrahedrons,
		orderedTetrahedrons, accumulatedTetrahedronPerPartition);
		
	PX_FREE(partitionProgresses);
	PX_FREE(tempTetrahedrons);

	return accumulatedTetrahedronPerPartition;
}

bool findSlot(const TetrahedronT<PxU32>* tetraIndices, bool* occupied, const PxU32 tetrahedronIdx,
	const PxU32 offset, const PxU32 sVertInd, const PxU32 workIndex)
{
	const TetrahedronT<PxU32>& tetraInd = tetraIndices[tetrahedronIdx];

	for (PxU32 i = 0; i < 4; ++i)
	{
		const PxU32 dVertInd = i * offset + workIndex;
		if (sVertInd == tetraInd.v[i] && (!occupied[dVertInd]))
		{
			occupied[dVertInd] = true;
			return true;
		}
	}

	return false;
}

//output to remapOutput
bool findSlot(const TetrahedronT<PxU32>* tetraIndices, bool* occupied, const PxU32 tetrahedronIdx,
	const PxU32 offset, const PxU32 sVertInd, const PxU32 sVertIndOffset, PxU32* remapOutput,
	PxU32* accumulatedWriteBackIndex, const PxU32 workIndex)
{
	const TetrahedronT<PxU32>& tetraInd = tetraIndices[tetrahedronIdx];

	for (PxU32 i = 0; i < 4; ++i)
	{
		const PxU32 dVertIndOffset = i * offset + workIndex;
		if (sVertInd == tetraInd.v[i] && (!occupied[dVertIndOffset]))
		{
			remapOutput[sVertIndOffset] = dVertIndOffset;
			accumulatedWriteBackIndex[dVertIndOffset] = sVertIndOffset;
			occupied[dVertIndOffset] = true;
			return true;
		}
	}

	return false;
}

void computeNumberOfCopiesPerVerts(const PxU32 maximumPartitions, PxU32* combineAccumulatedTetraPerPartitions,
	const TetrahedronT<PxU32>* tetraIndices, const PxU32* orderedTetrahedrons, const PxU32 offset, bool* occupied, PxU32* numCopiesEachVerts)
{
	//compute numCopiesEachVerts
	PxU32 startId = 0;
	for (PxU32 i = 0; i < maximumPartitions; ++i)
	{
		PxU32 endId = combineAccumulatedTetraPerPartitions[i];

		for (PxU32 j = startId; j < endId; ++j)
		{
			const PxU32 tetrahedronInd = orderedTetrahedrons[j];

			const TetrahedronT<PxU32>& tetraInd = tetraIndices[tetrahedronInd];

			for (PxU32 b = 0; b < 4; ++b)
			{
				const PxU32 vertInd = tetraInd.v[b];
					
				bool found = false;
				for (PxU32 k = i + 1; k < maximumPartitions; ++k)
				{
					const PxU32 tStartId = combineAccumulatedTetraPerPartitions[k - 1];
					const PxU32 tEndId = combineAccumulatedTetraPerPartitions[k];

					bool foundSlotInThisPartition = false;
					for (PxU32 a = tStartId; a < tEndId; ++a)
					{
						const PxU32 otherTetrahedronInd = orderedTetrahedrons[a];
						if (findSlot(tetraIndices, occupied, otherTetrahedronInd, offset, vertInd, a))
						{
							foundSlotInThisPartition = true;
							break;
						}
					}

					if (foundSlotInThisPartition)
					{
						found = true;
						break;
					}
				}

				if (!found)
				{
					numCopiesEachVerts[vertInd]++;
				}

			}

		}

		startId = endId;
	}
}

//compute remapOutput
void computeRemapOutputForVertsAndAccumulatedBuffer(const PxU32 maximumPartitions, PxU32* combineAccumulatedTetraPerPartitions,
	const TetrahedronT<PxU32>* tetraIndices, const PxU32* orderedTetrahedrons, const PxU32 offset, bool* occupied, PxU32* tempNumCopiesEachVerts, const PxU32* accumulatedCopies,
	const PxU32 numVerts,  PxU32* remapOutput,
	PxU32* accumulatedWriteBackIndex, const PxU32 totalNumCopies)
{
	PxMemZero(tempNumCopiesEachVerts, sizeof(PxU32) *  numVerts);

	const PxU32 totalNumVerts = offset * 4;

	PxMemZero(occupied, sizeof(bool) * totalNumVerts);

	//initialize accumulatedWriteBackIndex to itself
	for (PxU32 i = 0; i < totalNumVerts; ++i)
	{
		accumulatedWriteBackIndex[i] = i;
	}
		
	//compute remap output
	PxU32 startId = 0;
	for (PxU32 i = 0; i < maximumPartitions; ++i)
	{
		const PxU32 endId = combineAccumulatedTetraPerPartitions[i];

		for (PxU32 j = startId; j < endId; ++j)
		{
			const PxU32 tetrahedronsIdx = orderedTetrahedrons[j];
			const TetrahedronT<PxU32>& tetraInd = tetraIndices[tetrahedronsIdx];

			for (PxU32 b = 0; b < 4; ++b)
			{
				const PxU32 vertInd = tetraInd.v[b];
				const PxU32 vertOffset = j + offset * b;
				bool found = false;
				for (PxU32 k = i + 1; k < maximumPartitions; ++k)
				{
					const PxU32 tStartId = combineAccumulatedTetraPerPartitions[k-1];
					const PxU32 tEndId = combineAccumulatedTetraPerPartitions[k];

					bool foundSlotInThisPartition = false;
					for (PxU32 a = tStartId; a < tEndId; ++a)
					{
						const PxU32 otherTetrahedronInd = orderedTetrahedrons[a];
						if (findSlot(tetraIndices, occupied, otherTetrahedronInd, offset, vertInd,
							vertOffset, remapOutput, accumulatedWriteBackIndex, a))
						{
							foundSlotInThisPartition = true;
							break;
						}
					}

					if (foundSlotInThisPartition)
					{
						found = true;
						break;
					}
				}

				if (!found)
				{
					const PxU32 abVertStartInd = vertInd == 0 ? 0 : accumulatedCopies[vertInd - 1];
					const PxU32 index = totalNumVerts + abVertStartInd + tempNumCopiesEachVerts[vertInd];
						
					//remapOutput for the current vert index
					remapOutput[vertOffset] = index;
					//const PxU32 writebackIndex = abVertStartInd + tempNumCopiesEachVerts[vertInd];
					//accumulatedWriteBackIndex[writebackIndex] = vertOffset;

					remapOutput[index] = vertOffset;

					tempNumCopiesEachVerts[vertInd]++;
				}
			}
		}

		startId = endId;
	}

	//PxU32* writeBackBuffer = &accumulatedWriteBackIndex[totalNumVerts];
	PxU32* accumulatedBufferRemap = &remapOutput[totalNumVerts];
	for (PxU32 i = 0; i < totalNumCopies; ++i)
	{
		PxU32 originalIndex = accumulatedBufferRemap[i];
		PxU32 wbIndex0, wbIndex1;
		do
		{
			wbIndex0 = originalIndex;
			wbIndex1 = accumulatedWriteBackIndex[wbIndex0];
			originalIndex = wbIndex1;

		} while (wbIndex0 != wbIndex1);

		accumulatedBufferRemap[i] = wbIndex1;
	}
}

//void combineGridModelPartitions(const TetrahedronMeshData& simulationMesh, SoftBodySimulationData& simulationData, PxU32** accumulatedTetrahedronPerPartitions)
//{
//	const PxU32 numTets = simulationMesh.mNbTetrahedrons;
//	const PxU32 numVerts = simulationMesh.mNbVertices;

//	const PxU32 nbPartitions = simulationData.mGridModelNbPartitions;

//	PxU32* accumulatedTetrahedronPerPartition = *accumulatedTetrahedronPerPartitions;

//	
//	const PxU32 maximumPartitions = 8;
//	PxU32* combineAccumulatedTetraPerPartitions = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * maximumPartitions, "combineAccumulatedTetraPerPartitions"));
//	simulationData.mGMAccumulatedPartitionsCP = combineAccumulatedTetraPerPartitions;
//	
//	PxMemZero(combineAccumulatedTetraPerPartitions, sizeof(PxU32) * maximumPartitions);

//	const PxU32 maxAccumulatedPartitionsPerPartitions = (nbPartitions + maximumPartitions - 1) / maximumPartitions;

//	PxU32* orderedTetrahedrons = simulationData.mGridModelOrderedTetrahedrons;

//	PxU32* tempOrderedTetrahedrons = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * numTets, "tempOrderedTetrahedrons"));

//	const TetrahedronT<PxU32>* tetrahedrons = reinterpret_cast<TetrahedronT<PxU32>*>( simulationMesh.mTetrahedrons);
//	
//	const PxU32 maxAccumulatedCP = (nbPartitions + maximumPartitions - 1) / maximumPartitions;

//	const PxU32 partitionArraySize = maxAccumulatedCP * maximumPartitions;
//	const PxU32 nbPartitionTables = partitionArraySize * numVerts;

//	PxU32* tempPartitionTablePerVert = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * nbPartitionTables, "tempPartitionTablePerVert"));
//	PxU32* tempRemapTablePerVert = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * nbPartitionTables, "tempRemapTablePerVert"));

//	PxU32* pullIndices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * numTets*4, "tempRemapTablePerVert"));
//	PxU32* lastRef = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)* maxAccumulatedCP*numVerts, "refCounts"));

//	PxU32* accumulatedCopiesEachVerts = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * numVerts, "accumulatedCopiesEachVerts"));
//	simulationData.mGMAccumulatedCopiesCP = accumulatedCopiesEachVerts;

//	PxU32* tempNumCopiesEachVerts = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * numVerts, "numCopiesEachVerts"));
//	PxMemZero(tempNumCopiesEachVerts, sizeof(PxU32) * numVerts);

//	PxMemSet(pullIndices, 0xffffffff, sizeof(PxU32)*numTets*4);
//	PxMemSet(lastRef, 0xffffffff, sizeof(PxU32)*maxAccumulatedCP*numVerts);

//	//initialize partitionTablePerVert
//	for (PxU32 i = 0; i < nbPartitionTables; ++i)
//	{
//		tempPartitionTablePerVert[i] = 0xffffffff;
//		tempRemapTablePerVert[i] = 0xffffffff;
//		
//	}

//	PxU32 maxTetPerPartitions = 0;
//	PxU32 count = 0;

//	const PxU32 totalNumVerts = numTets * 4;

//	PxU32 totalCopies = numVerts * maxAccumulatedCP;
//	simulationData.mGridModelNbPartitions = maximumPartitions;
//	simulationData.mGMRemapOutputSize = totalNumVerts + totalCopies;

//	////allocate enough memory for the verts and the accumulation buffer
//	//PxVec4* orderedVertsInMassCP = reinterpret_cast<PxVec4*>(PX_ALLOC(sizeof(PxVec4) * totalNumVerts, "mGMOrderedVertInvMassCP"));
//	//data.mGMOrderedVertInvMassCP = orderedVertsInMassCP;

//	//compute remap table
//	PxU32* remapOutput = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * simulationData.mGMRemapOutputSize, "remapOutput"));
//	simulationData.mGMRemapOutputCP = remapOutput;


//	for (PxU32 i = 0; i < maximumPartitions; ++i)
//	{
//		PxU32 totalTets = 0;
//		for (PxU32 j = 0; j < maxAccumulatedPartitionsPerPartitions; ++j)
//		{
//			PxU32 partitionId = i + maximumPartitions * j;
//			if (partitionId < nbPartitions)
//			{
//				const PxU32 startInd = partitionId == 0 ? 0 : accumulatedTetrahedronPerPartition[partitionId - 1];
//				const PxU32 endInd = accumulatedTetrahedronPerPartition[partitionId];

//				for (PxU32 k = startInd; k < endInd; ++k)
//				{
//					const PxU32 tetraInd = orderedTetrahedrons[k];
//					tempOrderedTetrahedrons[count] = tetraInd;
//					//tempCombinedTetraIndices[count] = tetGM[tetraInd];
//					//tempTetRestPose[count] = tetRestPose[tetraInd];

//					PxU32 index = i * maxAccumulatedCP + j;

//					TetrahedronT<PxU32> tet = tetrahedrons[tetraInd];
//					tempPartitionTablePerVert[tet.v[0] * partitionArraySize + index] = count;
//					tempPartitionTablePerVert[tet.v[1] * partitionArraySize + index] = count + numTets;
//					tempPartitionTablePerVert[tet.v[2] * partitionArraySize + index] = count + numTets * 2;
//					tempPartitionTablePerVert[tet.v[3] * partitionArraySize + index] = count + numTets * 3;

//					if (lastRef[tet.v[0] * maxAccumulatedCP + j] == 0xffffffff)
//					{
//						pullIndices[4 * count] = tet.v[0];
//						tempNumCopiesEachVerts[tet.v[0]]++;
//					}
//					else
//					{
//						remapOutput[lastRef[tet.v[0] * maxAccumulatedCP + j]] = count;
//					}
//					lastRef[tet.v[0] * maxAccumulatedCP + j] = 4*count;

//					if (lastRef[tet.v[1] * maxAccumulatedCP + j] == 0xffffffff)
//					{
//						pullIndices[4 * count + 1] = tet.v[1];
//						tempNumCopiesEachVerts[tet.v[1]]++;
//					}
//					else
//					{
//						remapOutput[lastRef[tet.v[1] * maxAccumulatedCP + j]] = count + numTets;
//					}
//					lastRef[tet.v[1] * maxAccumulatedCP + j] = 4*count + 1;

//					if (lastRef[tet.v[2] * maxAccumulatedCP + j] == 0xffffffff)
//					{
//						pullIndices[4 * count + 2] = tet.v[2];
//						tempNumCopiesEachVerts[tet.v[2]]++;
//						
//					}
//					else
//					{
//						remapOutput[lastRef[tet.v[2] * maxAccumulatedCP + j]] = count + 2*numTets;
//					}
//					lastRef[tet.v[2] * maxAccumulatedCP + j] = 4*count+2;

//					if (lastRef[tet.v[3] * maxAccumulatedCP + j] == 0xffffffff)
//					{
//						pullIndices[4 * count + 3] = tet.v[3];
//						tempNumCopiesEachVerts[tet.v[3]]++;
//					}
//					else
//					{
//						remapOutput[lastRef[tet.v[3] * maxAccumulatedCP + j]] = count + 3*numTets;
//					}
//					lastRef[tet.v[3] * maxAccumulatedCP + j] = 4*count+3;

//					count++;
//				}

//				totalTets += (endInd - startInd);
//			}
//		}

//		combineAccumulatedTetraPerPartitions[i] = count;
//		maxTetPerPartitions = PxMax(maxTetPerPartitions, totalTets);
//	}

//	//Last bit - output accumulation buffer...

//	
//	PxU32 outIndex = 0;
//	simulationData.mGridModelMaxTetsPerPartitions = maxTetPerPartitions;

//	//If this commented out, we don't use combined partition anymore
//	PxMemCopy(orderedTetrahedrons, tempOrderedTetrahedrons, sizeof(PxU32) * numTets);


//	/*bool* tempOccupied = reinterpret_cast <bool*>( PX_ALLOC(sizeof(bool) * totalNumVerts, "tempOccupied"));
//	PxMemZero(tempOccupied, sizeof(bool) * totalNumVerts);*/

//	
//	//data.mGridModelNbPartitions = maximumPartitions;
//	//data.mGMRemapOutputSize = totalNumVerts + totalCopies;
//	simulationData.mGridModelNbPartitions = maximumPartitions;
//	simulationData.mGMRemapOutputSize = totalNumVerts + totalCopies;

//	//data.mGMOrderedVertInvMassCP = orderedVertsInMassCP;
//	//mGMOrderedVertInvMassCP = orderedVertsInMassCP;

//	//Last bit - output accumulation buffer...

//	outIndex = 0;
//	for (PxU32 i = 0; i < numVerts; ++i)
//	{

//		for (PxU32 j = 0; j < maxAccumulatedCP; ++j)
//		{
//			if (lastRef[i * maxAccumulatedCP + j] != 0xffffffff)
//			{
//				remapOutput[lastRef[i * maxAccumulatedCP + j]] = totalNumVerts + outIndex++;
//			}
//		}
//		accumulatedCopiesEachVerts[i] = outIndex;
//	}


//	PX_ASSERT(count == numTets);

//	simulationData.mGridModelMaxTetsPerPartitions = maxTetPerPartitions;

//	simulationData.mGMPullIndices = pullIndices;

//	//If this commented out, we don't use combined partition anymore
//	PxMemCopy(orderedTetrahedrons, tempOrderedTetrahedrons, sizeof(PxU32) * numTets);

//	PX_FREE(tempNumCopiesEachVerts);
//	PX_FREE(tempOrderedTetrahedrons);
//	PX_FREE(tempPartitionTablePerVert);
//	PX_FREE(tempRemapTablePerVert);
//
//	PX_FREE(lastRef);

//}

PxU32 setBit(PxU32 value, PxU32 bitLocation, bool bitState)
{
	if (bitState)
		return value | (1 << bitLocation);
	else
		return value & (~(1 << bitLocation));
}

void combineGridModelPartitions(const TetrahedronMeshData& simulationMesh, SoftBodySimulationData& simulationData, PxU32** accumulatedTetrahedronPerPartitions)
{
const PxU32 numTets = simulationMesh.mNbTetrahedrons;
const PxU32 numVerts = simulationMesh.mNbVertices;

const PxU32 nbPartitions = simulationData.mGridModelNbPartitions;

PxU32* accumulatedTetrahedronPerPartition = *accumulatedTetrahedronPerPartitions;

const PxU32 maximumPartitions = 8;
PxU32* combineAccumulatedTetraPerPartitions = PX_ALLOCATE(PxU32, maximumPartitions, "combineAccumulatedTetraPerPartitions");
simulationData.mGMAccumulatedPartitionsCP = combineAccumulatedTetraPerPartitions;

PxMemZero(combineAccumulatedTetraPerPartitions, sizeof(PxU32) * maximumPartitions);

const PxU32 maxAccumulatedPartitionsPerPartitions = (nbPartitions + maximumPartitions - 1) / maximumPartitions;

PxU32* orderedTetrahedrons = simulationData.mGridModelOrderedTetrahedrons;

PxU32* tempOrderedTetrahedrons = PX_ALLOCATE(PxU32, numTets, "tempOrderedTetrahedrons");

const TetrahedronT<PxU32>* tetrahedrons = reinterpret_cast<TetrahedronT<PxU32>*>(simulationMesh.mTetrahedrons);

const PxU32 maxAccumulatedCP = (nbPartitions + maximumPartitions - 1) / maximumPartitions;

const PxU32 partitionArraySize = maxAccumulatedCP * maximumPartitions;
const PxU32 nbPartitionTables = partitionArraySize * numVerts;

PxU32* tempPartitionTablePerVert = PX_ALLOCATE(PxU32, nbPartitionTables, "tempPartitionTablePerVert");
PxU32* tempRemapTablePerVert = PX_ALLOCATE(PxU32, nbPartitionTables, "tempRemapTablePerVert");

PxU32* pullIndices = PX_ALLOCATE(PxU32, (numTets * 4), "tempRemapTablePerVert");
PxU32* lastRef = PX_ALLOCATE(PxU32, (maxAccumulatedCP*numVerts), "refCounts");

PxU32* accumulatedCopiesEachVerts = PX_ALLOCATE(PxU32, numVerts, "accumulatedCopiesEachVerts");
simulationData.mGMAccumulatedCopiesCP = accumulatedCopiesEachVerts;

PxU32* tempNumCopiesEachVerts = PX_ALLOCATE(PxU32, numVerts, "numCopiesEachVerts");
PxMemZero(tempNumCopiesEachVerts, sizeof(PxU32) * numVerts);

PxMemSet(pullIndices, 0xffffffff, sizeof(PxU32)*numTets * 4);
PxMemSet(lastRef, 0xffffffff, sizeof(PxU32)*maxAccumulatedCP*numVerts);

//initialize partitionTablePerVert
for (PxU32 i = 0; i < nbPartitionTables; ++i)
{
	tempPartitionTablePerVert[i] = 0xffffffff;
	tempRemapTablePerVert[i] = 0xffffffff;

}

PxU32 maxTetPerPartitions = 0;
PxU32 count = 0;

const PxU32 totalNumVerts = numTets * 4;

PxU32 totalCopies = numVerts * maxAccumulatedCP;
simulationData.mGridModelNbPartitions = maximumPartitions;
simulationData.mGMRemapOutputSize = totalNumVerts + totalCopies;

////allocate enough memory for the verts and the accumulation buffer
//PxVec4* orderedVertsInMassCP = reinterpret_cast<PxVec4*>(PX_ALLOC(sizeof(PxVec4) * totalNumVerts, "mGMOrderedVertInvMassCP"));
//data.mGMOrderedVertInvMassCP = orderedVertsInMassCP;

//compute remap table
PxU32* remapOutput = PX_ALLOCATE(PxU32, simulationData.mGMRemapOutputSize, "remapOutput");
simulationData.mGMRemapOutputCP = remapOutput;

for (PxU32 i = 0; i < maximumPartitions; ++i)
{
	PxU32 totalTets = 0;
	for (PxU32 j = 0; j < maxAccumulatedPartitionsPerPartitions; ++j)
	{
		PxU32 partitionId = i + maximumPartitions * j;
		if (partitionId < nbPartitions)
		{
			const PxU32 startInd = partitionId == 0 ? 0 : accumulatedTetrahedronPerPartition[partitionId - 1];
			const PxU32 endInd = accumulatedTetrahedronPerPartition[partitionId];

			for (PxU32 k = startInd; k < endInd; ++k)
			{
				const PxU32 tetraInd = orderedTetrahedrons[k];
				tempOrderedTetrahedrons[count] = tetraInd;
				//tempCombinedTetraIndices[count] = tetGM[tetraInd];
				//tempTetRestPose[count] = tetRestPose[tetraInd];

				PxU32 index = i * maxAccumulatedCP + j;

				TetrahedronT<PxU32> tet = tetrahedrons[tetraInd];
				tempPartitionTablePerVert[tet.v[0] * partitionArraySize + index] = count;
				tempPartitionTablePerVert[tet.v[1] * partitionArraySize + index] = count + numTets;
				tempPartitionTablePerVert[tet.v[2] * partitionArraySize + index] = count + numTets * 2;
				tempPartitionTablePerVert[tet.v[3] * partitionArraySize + index] = count + numTets * 3;

				if (lastRef[tet.v[0] * maxAccumulatedCP + j] == 0xffffffff)
				{
					pullIndices[4 * count] = tet.v[0];
					tempNumCopiesEachVerts[tet.v[0]]++;
				}
				else
				{
					remapOutput[lastRef[tet.v[0] * maxAccumulatedCP + j]] = count;
				}
				lastRef[tet.v[0] * maxAccumulatedCP + j] = 4 * count;

				if (lastRef[tet.v[1] * maxAccumulatedCP + j] == 0xffffffff)
				{
					pullIndices[4 * count + 1] = tet.v[1];
					tempNumCopiesEachVerts[tet.v[1]]++;
				}
				else
				{
					remapOutput[lastRef[tet.v[1] * maxAccumulatedCP + j]] = count + numTets;
				}
				lastRef[tet.v[1] * maxAccumulatedCP + j] = 4 * count + 1;

				if (lastRef[tet.v[2] * maxAccumulatedCP + j] == 0xffffffff)
				{
					pullIndices[4 * count + 2] = tet.v[2];
					tempNumCopiesEachVerts[tet.v[2]]++;
				}
				else
				{
					remapOutput[lastRef[tet.v[2] * maxAccumulatedCP + j]] = count + 2 * numTets;
				}
				lastRef[tet.v[2] * maxAccumulatedCP + j] = 4 * count + 2;

				if (lastRef[tet.v[3] * maxAccumulatedCP + j] == 0xffffffff)
				{
					pullIndices[4 * count + 3] = tet.v[3];
					tempNumCopiesEachVerts[tet.v[3]]++;
				}
				else
				{
					remapOutput[lastRef[tet.v[3] * maxAccumulatedCP + j]] = count + 3 * numTets;
				}
				lastRef[tet.v[3] * maxAccumulatedCP + j] = 4 * count + 3;

				count++;
			}

			totalTets += (endInd - startInd);
		}
	}

	combineAccumulatedTetraPerPartitions[i] = count;
	maxTetPerPartitions = PxMax(maxTetPerPartitions, totalTets);
}

//Last bit - output accumulation buffer...

PxU32 outIndex = 0;
for (PxU32 i = 0; i < numVerts; ++i)
{

	for (PxU32 j = 0; j < maxAccumulatedCP; ++j)
	{
		if (lastRef[i * maxAccumulatedCP + j] != 0xffffffff)
		{
			remapOutput[lastRef[i * maxAccumulatedCP + j]] = totalNumVerts + outIndex++;
		}
	}
	accumulatedCopiesEachVerts[i] = outIndex;
}

PX_ASSERT(count == numTets);

simulationData.mGridModelMaxTetsPerPartitions = maxTetPerPartitions;

simulationData.mGMPullIndices = pullIndices;

//If this commented out, we don't use combined partition anymore
PxMemCopy(orderedTetrahedrons, tempOrderedTetrahedrons, sizeof(PxU32) * numTets);

PX_FREE(tempNumCopiesEachVerts);
PX_FREE(tempOrderedTetrahedrons);
PX_FREE(tempPartitionTablePerVert);
PX_FREE(tempRemapTablePerVert);
PX_FREE(lastRef);
}

const PxI32 tetIndicesFromVoxels[8] = { 0, 1, 3, 14, 6, 11, 2, 18 };
//const PxI32 tets6PerVoxel[24] = { 0,1,6,2,  0,1,4,6,  1,4,6,5,  1,2,3,6,  1,3,7,6,  1,5,6,7 };



const PxI32 tetIndicesFromVoxelsA[8] = { 0, 5, 16, 2, 12, 3, 1, 9 };
const PxI32 tetIndicesFromVoxelsB[8] = { 5, 0, 3, 16, 2, 12, 9, 1 };

//const PxU32 tets5PerVoxel[] = {
//			 0, 6, 3, 5, 0, 1, 5, 3, 6, 7, 3, 5, 4, 5, 6, 0, 2, 3, 0, 6,
//			 1, 7, 4, 2, 1, 0, 2, 4, 7, 6, 4, 2, 5, 4, 1, 7, 3, 2, 7, 1 };
//           0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19

void combineGridModelPartitionsHexMesh(const TetrahedronMeshData& simulationMesh, SoftBodySimulationData& simulationData, 
	PxU32** accumulatedTetrahedronPerPartitions, PxU32 numTetsPerElement)
{
//const PxU32 numTets = simulationMesh.mNbTetrahedrons;
const PxU32 numElements = simulationMesh.mNbTetrahedrons/simulationData.mNumTetsPerElement;
const PxU32 numVerts = simulationMesh.mNbVertices;

const PxU32 NumVertsPerElement = 8;

const PxU32 nbPartitions = simulationData.mGridModelNbPartitions;

PxU32* accumulatedTetrahedronPerPartition = *accumulatedTetrahedronPerPartitions;

const PxU32 maximumPartitions = 8;
PxU32* combineAccumulatedTetraPerPartitions = PX_ALLOCATE(PxU32, maximumPartitions, "combineAccumulatedTetraPerPartitions");
simulationData.mGMAccumulatedPartitionsCP = combineAccumulatedTetraPerPartitions;

PxMemZero(combineAccumulatedTetraPerPartitions, sizeof(PxU32) * maximumPartitions);

const PxU32 maxAccumulatedPartitionsPerPartitions = (nbPartitions + maximumPartitions - 1) / maximumPartitions;

PxU32* orderedTetrahedrons = simulationData.mGridModelOrderedTetrahedrons;

PxU32* tempOrderedTetrahedrons = PX_ALLOCATE(PxU32, numElements, "tempOrderedTetrahedrons");

const TetrahedronT<PxU32>* tetrahedrons = reinterpret_cast<TetrahedronT<PxU32>*>(simulationMesh.mTetrahedrons);

const PxU32 maxAccumulatedCP = (nbPartitions + maximumPartitions - 1) / maximumPartitions;

const PxU32 partitionArraySize = maxAccumulatedCP * maximumPartitions;
const PxU32 nbPartitionTables = partitionArraySize * numVerts;

PxU32* tempPartitionTablePerVert = PX_ALLOCATE(PxU32, nbPartitionTables, "tempPartitionTablePerVert");
PxU32* tempRemapTablePerVert = PX_ALLOCATE(PxU32, nbPartitionTables, "tempRemapTablePerVert");

PxU32* pullIndices = PX_ALLOCATE(PxU32, (numElements * NumVertsPerElement), "tempRemapTablePerVert");
PxU32* lastRef = PX_ALLOCATE(PxU32, (maxAccumulatedCP*numVerts), "refCounts");

PxU32* accumulatedCopiesEachVerts = PX_ALLOCATE(PxU32, numVerts, "accumulatedCopiesEachVerts");
simulationData.mGMAccumulatedCopiesCP = accumulatedCopiesEachVerts;

PxU32* tempNumCopiesEachVerts = PX_ALLOCATE(PxU32, numVerts, "numCopiesEachVerts");
PxMemZero(tempNumCopiesEachVerts, sizeof(PxU32) * numVerts);

PxMemSet(pullIndices, 0xffffffff, sizeof(PxU32)*numElements * NumVertsPerElement);
PxMemSet(lastRef, 0xffffffff, sizeof(PxU32)*maxAccumulatedCP*numVerts);

//initialize partitionTablePerVert
for (PxU32 i = 0; i < nbPartitionTables; ++i)
{
	tempPartitionTablePerVert[i] = 0xffffffff;
	tempRemapTablePerVert[i] = 0xffffffff;

}

PxU32 maxTetPerPartitions = 0;
PxU32 count = 0;

const PxU32 totalNumVerts = numElements* NumVertsPerElement;

PxU32 totalCopies = numVerts * maxAccumulatedCP;
simulationData.mGridModelNbPartitions = maximumPartitions;
simulationData.mGMRemapOutputSize = totalNumVerts + totalCopies;

////allocate enough memory for the verts and the accumulation buffer
//PxVec4* orderedVertsInMassCP = reinterpret_cast<PxVec4*>(PX_ALLOC(sizeof(PxVec4) * totalNumVerts, "mGMOrderedVertInvMassCP"));
//data.mGMOrderedVertInvMassCP = orderedVertsInMassCP;

//compute remap table
PxU32* remapOutput = PX_ALLOCATE(PxU32, simulationData.mGMRemapOutputSize, "remapOutput");
simulationData.mGMRemapOutputCP = remapOutput;

for (PxU32 i = 0; i < maximumPartitions; ++i)
{
	PxU32 totalTets = 0;
	for (PxU32 j = 0; j < maxAccumulatedPartitionsPerPartitions; ++j)
	{
		PxU32 partitionId = i + maximumPartitions * j;
		if (partitionId < nbPartitions)
		{
			const PxU32 startInd = partitionId == 0 ? 0 : accumulatedTetrahedronPerPartition[partitionId - 1];
			const PxU32 endInd = accumulatedTetrahedronPerPartition[partitionId];

			for (PxU32 k = startInd; k < endInd; ++k)
			{
				const PxU32 tetraInd = orderedTetrahedrons[k];
				tempOrderedTetrahedrons[count] = tetraInd;

				PxU32 index = i * maxAccumulatedCP + j;

				const PxI32* map = NULL;
				const PxU32* tetInds = reinterpret_cast<const PxU32*>(&tetrahedrons[tetraInd]);

				//If 5 tets are used per voxel, some voxels have a flipped tetrahedron configuration
				//Tetmaker uses the following table to generate 5 tets per hex. The first row is the standard configuration, the second row the flipped config.
				//To distinguish the two, a pattern must be found that is only present in one of the two configurations
				//While 5 tets get created, this leads to 20 indices. The flipped configuration references the same vertex at indices[0] and indices[19] while 
				//the default config references different tets at indices[0] and indices[19]. This means that this comparsion can reliably detect flipped configurations.
				//const PxU32 tets5PerVoxel[] = {
				//			 0, 6, 3, 5, 0, 1, 5, 3, 6, 7, 3, 5, 4, 5, 6, 0, 2, 3, 0, 6,
				//			 1, 7, 4, 2, 1, 0, 2, 4, 7, 6, 4, 2, 5, 4, 1, 7, 3, 2, 7, 1
				bool flipped = tetInds[0] == tetInds[19];
				if (numTetsPerElement == 6)
				{
					map = tetIndicesFromVoxels;
				}
				else
				{
					if (!flipped)					
						map = tetIndicesFromVoxelsA;					
					else					
						map = tetIndicesFromVoxelsB;					
				}

				for (PxU32 v = 0; v < 4; ++v)
				{
					PxU32 vertInd = tetInds[map[v]];
					tempPartitionTablePerVert[vertInd * partitionArraySize + index] = count + numElements * v;

					if (lastRef[vertInd * maxAccumulatedCP + j] == 0xffffffff)
					{
						pullIndices[4 * count + v] = vertInd;
						tempNumCopiesEachVerts[vertInd]++;
					}
					else
					{
						remapOutput[lastRef[vertInd * maxAccumulatedCP + j]] = count + v * numElements;
					}
					lastRef[vertInd * maxAccumulatedCP + j] = 4 * count + v;
				}

				for (PxU32 v = 0; v < 4; ++v)
				{
					//vertex index
					PxU32 vertInd = tetInds[map[v+4]];
					//Where the vertex data will be written to/read from
					tempPartitionTablePerVert[vertInd * partitionArraySize + index] = count + numElements * (v+4);

					if (lastRef[vertInd * maxAccumulatedCP + j] == 0xffffffff)
					{
						pullIndices[4 * (count + numElements) + v] = vertInd;
						tempNumCopiesEachVerts[vertInd]++;
					}
					else
					{
						remapOutput[lastRef[vertInd * maxAccumulatedCP + j]] = count + (v+4) * numElements;
					}
					lastRef[vertInd * maxAccumulatedCP + j] = 4 * (numElements + count) + v;
				}

				if (numTetsPerElement == 5)
				{
					PxU32 ind = pullIndices[4 * count];
					ind = setBit(ind, 31, flipped);
					pullIndices[4 * count /*+ v*/] = ind;
				}

				count++;
			}

			totalTets += (endInd - startInd);
		}
	}

	combineAccumulatedTetraPerPartitions[i] = count;
	maxTetPerPartitions = PxMax(maxTetPerPartitions, totalTets);
}

//Last bit - output accumulation buffer...

PxU32 outIndex = 0;
for (PxU32 i = 0; i < numVerts; ++i)
{

	for (PxU32 j = 0; j < maxAccumulatedCP; ++j)
	{
		if (lastRef[i * maxAccumulatedCP + j] != 0xffffffff)
		{
			remapOutput[lastRef[i * maxAccumulatedCP + j]] = totalNumVerts + outIndex++;
		}
	}
	accumulatedCopiesEachVerts[i] = outIndex;
}


PX_ASSERT(count == numElements);

simulationData.mGridModelMaxTetsPerPartitions = maxTetPerPartitions;

simulationData.mGMPullIndices = pullIndices;

//If this commented out, we don't use combined partition anymore
PxMemCopy(orderedTetrahedrons, tempOrderedTetrahedrons, sizeof(PxU32) * numElements);

PX_FREE(tempNumCopiesEachVerts);
PX_FREE(tempOrderedTetrahedrons);
PX_FREE(tempPartitionTablePerVert);
PX_FREE(tempRemapTablePerVert);
PX_FREE(lastRef);

}

struct DistanceCheck
{
	//input
	PxVec3*	mVerts;
	IndTetrahedron32* mTetrahedron32;
	PxVec3 mOriginalVert;

	//output
	PxU32 mTetInd;
	PxReal mDistanceSq;
	PxVec3 mClosestPoint;
		
	//these data are for validation only
	PxU32 mNbPrimsPerLeaf;
	PxU32 mNbPrims;
};	

static bool gDistanceNodeCheckCallback(const AABBTreeNode* current, void* userData)
{
	DistanceCheck* Data = reinterpret_cast<DistanceCheck*>(userData);
	const PxVec3& p = Data->mOriginalVert;

	const AABBTreeNode* posNode = current->getPos();
	const AABBTreeNode* negNode = current->getNeg();

	PxReal distanceSqP = PX_MAX_F32;
	if (posNode)
	{
		const PxBounds3& posAABB = posNode->getAABB();
		const PxVec3 posClosest = posAABB.minimum.maximum(p.minimum(posAABB.maximum));
		distanceSqP = (posClosest - p).magnitudeSquared();
	}

	PxReal distanceSqN = PX_MAX_F32;

	if (negNode)
	{
		const PxBounds3& negAABB = negNode->getAABB();
		const PxVec3 negClosest = negAABB.minimum.maximum(p.minimum(negAABB.maximum));
		distanceSqN = (negClosest - p).magnitudeSquared();
	}

	return distanceSqP <= distanceSqN ? true : false;
}

static bool gDistanceCheckCallback(const AABBTreeNode* current, PxU32 /*depth*/, void* userData)
{
	DistanceCheck* Data = reinterpret_cast<DistanceCheck*>(userData);
	const PxVec3& p = Data->mOriginalVert;

	if (current->isLeaf())
	{
		const PxU32 n = current->getNbPrimitives();
		PX_ASSERT(n <= Data->mNbPrimsPerLeaf);
			
		PxU32* Prims = const_cast<PxU32*>(current->getPrimitives());
		PX_UNUSED(Prims);

		const PxVec3* verts = Data->mVerts;

		for (PxU32 i = 0; i < n; i++)
		{
			PX_ASSERT(Prims[i] < Data->mNbPrims);
			const PxU32 tetId = Prims[i];
			const IndTetrahedron32& tetrahedron = Data->mTetrahedron32[tetId];
			PX_UNUSED(tetrahedron);
			const PxVec3 a = verts[tetrahedron.mRef[0]];
			const PxVec3 b = verts[tetrahedron.mRef[1]];
			const PxVec3 c = verts[tetrahedron.mRef[2]];
			const PxVec3 d = verts[tetrahedron.mRef[3]];
			//compute distance between the vert and the tetrahedron
			const PxVec4 result = PointOutsideOfPlane4(p, a, b, c, d);

			if (result.x >= 0.f && result.y >= 0.f && result.z >= 0.f && result.w >= 0.f)
			{
				//point is inside the tetrahedron
				Data->mClosestPoint = closestPtPointTetrahedron(p, a, b, c, d);
				Data->mDistanceSq = 0.f;
				Data->mTetInd = tetId;			
			}
			else
			{
				//point is outside the tetrahedron
				const PxVec3 closestP = closestPtPointTetrahedron(p, a, b, c, d, result);
				const PxReal distanceSq = (closestP - p).magnitudeSquared();
				if (distanceSq < Data->mDistanceSq)
				{
					Data->mClosestPoint = closestP;
					Data->mDistanceSq = distanceSq;
					Data->mTetInd = tetId;
				}
			}				
		}
	}
	else
	{
		//compute distance
	
		const PxBounds3& aabb = current->getAABB();

		const PxVec3& min = aabb.minimum;
		const PxVec3& max = aabb.maximum;

		const PxVec3 closest = min.maximum(p.minimum(max));
		PxReal distanceSq = (closest-p).magnitudeSquared();

		if (distanceSq > Data->mDistanceSq)
			return false;
	}
	return true;
}

struct OverlapCheck
{
	//input
	IndTetrahedron32	mColTetrahedron32;
	PxVec3*				mColMeshVerts;
	PxBounds3			mColTetBound;
	PxVec3*				mSimMeshVerts;
	IndTetrahedron32*	mSimMeshTetra;
	
	//output
	PxArray<PxU32> mSimTetraIndices;

	//these data are for validation only
	PxU32 mNbPrimsPerLeaf;
	PxU32 mNbPrims;
};

static bool gOverlapCallback(const AABBTreeNode* current, PxU32 /*depth*/, void* userData)
{
	OverlapCheck* Data = reinterpret_cast<OverlapCheck*>(userData);
	const PxBounds3& bound = Data->mColTetBound;

	if (current->isLeaf())
	{
		const PxU32 n = current->getNbPrimitives();
		PX_ASSERT(n <= Data->mNbPrimsPerLeaf);

		PxU32* Prims = const_cast<PxU32*>(current->getPrimitives());
		PX_UNUSED(Prims);

		const IndTetrahedron32& colTetInd = Data->mColTetrahedron32;

		const PxVec3 a0 = Data->mColMeshVerts[colTetInd.mRef[0]];
		const PxVec3 a1 = Data->mColMeshVerts[colTetInd.mRef[1]];
		const PxVec3 a2 = Data->mColMeshVerts[colTetInd.mRef[2]];
		const PxVec3 a3 = Data->mColMeshVerts[colTetInd.mRef[3]];

		const PxVec3 center0 = (a0 + a1 + a2 + a3) * 0.25f;

		TetrahedronV tetV(aos::V3LoadU(a0), aos::V3LoadU(a1), aos::V3LoadU(a2),
			aos::V3LoadU(a3));
		const LocalConvex<TetrahedronV> convexA(tetV);

		aos::FloatV contactDist = aos::FLoad(1e-4f);

		const PxVec3* verts = Data->mSimMeshVerts;

		for (PxU32 i = 0; i < n; i++)
		{
			PX_ASSERT(Prims[i] < Data->mNbPrims);
			const PxU32 tetId = Prims[i];
			const IndTetrahedron32& tetrahedron = Data->mSimMeshTetra[tetId];
				
			const PxVec3 b0 = verts[tetrahedron.mRef[0]];
			const PxVec3 b1 = verts[tetrahedron.mRef[1]];
			const PxVec3 b2 = verts[tetrahedron.mRef[2]];
			const PxVec3 b3 = verts[tetrahedron.mRef[3]];
			const PxVec3 center1 = (b0 + b1 + b2 + b3) * 0.25f;

			const PxVec3 dir = center1 - center0;

			TetrahedronV tetV2(aos::V3LoadU(b0), aos::V3LoadU(b1), aos::V3LoadU(b2),
				aos::V3LoadU(b3));
			tetV2.setMinMargin(aos::FEps());
			const LocalConvex<TetrahedronV> convexB(tetV2);
				
			GjkOutput output;
				
#ifdef USE_GJK_VIRTUAL
			GjkStatus status = testGjk(convexA, convexB, aos::V3LoadU(dir), contactDist, output.closestA,
				output.closestB, output.normal, output.penDep);
#else
			GjkStatus status = gjk(convexA, convexB, aos::V3LoadU(dir), contactDist, output.closestA,
				output.closestB, output.normal, output.penDep);
#endif
			if (status == GjkStatus::GJK_CLOSE || status == GjkStatus::GJK_CONTACT)
			{
				Data->mSimTetraIndices.pushBack(tetId);
			}
		}
	}
	else
	{
		const PxBounds3& aabb = current->getAABB();
		return bound.intersects(aabb);
	}
	return true;
}

void TetrahedronMeshBuilder::createCollisionModelMapping(const TetrahedronMeshData& collisionMesh, const SoftBodyCollisionData& collisionData, CollisionMeshMappingData& mappingData)
{
	const PxU32 nbVerts = collisionMesh.mNbVertices;

	mappingData.mCollisionAccumulatedTetrahedronsRef = PX_ALLOCATE(PxU32, nbVerts, "tetCounts");
		
	PxU32* tempCounts = PX_ALLOCATE(PxU32, nbVerts, "tempCounts");

	PxU32* tetCounts = mappingData.mCollisionAccumulatedTetrahedronsRef;

	PxMemZero(tetCounts, sizeof(PxU32) * nbVerts);
	PxMemZero(tempCounts, sizeof(PxU32) * nbVerts);
		
	const PxU32 nbTetrahedrons = collisionMesh.mNbTetrahedrons;
		
	IndTetrahedron32* tetra = reinterpret_cast<IndTetrahedron32*>(collisionData.mGRB_primIndices);

	for (PxU32 i = 0; i < nbTetrahedrons; i++)
	{
		IndTetrahedron32& tet = tetra[i];

		tetCounts[tet.mRef[0]]++;
		tetCounts[tet.mRef[1]]++;
		tetCounts[tet.mRef[2]]++;
		tetCounts[tet.mRef[3]]++;
	}

	//compute runsum
	PxU32 totalReference = 0;
	for (PxU32 i = 0; i < nbVerts; ++i)
	{
		PxU32 originalReference = tetCounts[i];
		tetCounts[i] = totalReference;
		totalReference += originalReference;
	}

	mappingData.mCollisionTetrahedronsReferences = PX_ALLOCATE(PxU32, totalReference, "mGMMappedTetrahedrons");
	mappingData.mCollisionNbTetrahedronsReferences = totalReference;

	PxU32* tetrahedronRefs = mappingData.mCollisionTetrahedronsReferences;

	for (PxU32 i = 0; i < nbTetrahedrons; i++)
	{
		IndTetrahedron32& tet = tetra[i];

		const PxU32 ind0 = tet.mRef[0];
		const PxU32 ind1 = tet.mRef[1];
		const PxU32 ind2 = tet.mRef[2];
		const PxU32 ind3 = tet.mRef[3];

		tetrahedronRefs[tetCounts[ind0] + tempCounts[ind0]] = i;
		tempCounts[ind0]++;

		tetrahedronRefs[tetCounts[ind1] + tempCounts[ind1]] = i;
		tempCounts[ind1]++;

		tetrahedronRefs[tetCounts[ind2] + tempCounts[ind2]] = i;
		tempCounts[ind2]++;

		tetrahedronRefs[tetCounts[ind3] + tempCounts[ind3]] = i;
		tempCounts[ind3]++;
	}

	PxVec3* verts = collisionMesh.mVertices;

	PxU8* tetHint = reinterpret_cast<PxU8*>(collisionData.mGRB_tetraSurfaceHint);

	IndTetrahedron32* surfaceTets = PX_ALLOCATE(IndTetrahedron32, nbTetrahedrons, "surfaceTets");

	PxU8* surfaceVertsHint = PX_ALLOCATE(PxU8, nbVerts, "surfaceVertsHint");
	PxU32* surfaceVertToTetRemap = PX_ALLOCATE(PxU32, nbVerts, "surfaceVertToTetRemap");
	PxMemSet(surfaceVertsHint, 0, nbVerts);

	PxU32 nbSurfaceTets = 0;

	for (PxU32 i = 0; i < nbTetrahedrons; i++)
	{
		IndTetrahedron32& originalTet = tetra[i];

		PxU8 hint = tetHint[i]; 
		//This is a surface triangle
		if (hint != 0)
		{
			IndTetrahedron32& tet = surfaceTets[nbSurfaceTets];
			tet.mRef[0] = originalTet.mRef[0];
			tet.mRef[1] = originalTet.mRef[1];
			tet.mRef[2] = originalTet.mRef[2];
			tet.mRef[3] = originalTet.mRef[3];

			if (hint & 1) //0111
			{
				if (surfaceVertsHint[originalTet.mRef[0]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[0]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[0]] = i;
				}

				if (surfaceVertsHint[originalTet.mRef[1]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[1]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[1]] = i;
				}

				if (surfaceVertsHint[originalTet.mRef[2]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[2]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[2]] = i;
				}
			}

			if (hint & 2)//1011
			{
				if (surfaceVertsHint[originalTet.mRef[0]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[0]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[0]] = i;
				}

				if (surfaceVertsHint[originalTet.mRef[1]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[1]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[1]] = i;
				}

				if (surfaceVertsHint[originalTet.mRef[3]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[3]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[3]] = i;
				}
			}

			if (hint & 4) //1101
			{
				if (surfaceVertsHint[originalTet.mRef[0]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[0]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[0]] = i;
				}

				if (surfaceVertsHint[originalTet.mRef[2]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[2]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[2]] = i;
				}

				if (surfaceVertsHint[originalTet.mRef[3]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[3]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[3]] = i;
				}
			}

			if (hint & 8)//1110
			{			
				if (surfaceVertsHint[originalTet.mRef[1]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[1]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[1]] = i;
				}

				if (surfaceVertsHint[originalTet.mRef[2]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[2]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[2]] = i;
				}

				if (surfaceVertsHint[originalTet.mRef[3]] == 0)
				{
					surfaceVertsHint[originalTet.mRef[3]] = 1;
					surfaceVertToTetRemap[originalTet.mRef[3]] = i;
				}
			}
				
			nbSurfaceTets++;
		}
	}

	PxU32 numSurfaceVerts = 0;
	for (PxU32 i = 0; i < nbVerts; ++i)
	{
		PxU32 hint = surfaceVertsHint[i];
		if (hint)
			numSurfaceVerts++;
	}

	mappingData.mCollisionSurfaceVertsHint = PX_ALLOCATE(PxU8, nbVerts, "mCollisionSurfaceVertsHint");
	mappingData.mCollisionSurfaceVertToTetRemap = PX_ALLOCATE(PxU32, nbVerts, "mCollisionSurfaceVertToTetRemap");

	PxMemCopy(mappingData.mCollisionSurfaceVertsHint, surfaceVertsHint, sizeof(PxU8)*nbVerts);
	PxMemCopy(mappingData.mCollisionSurfaceVertToTetRemap, surfaceVertToTetRemap, sizeof(PxU32)*nbVerts);

	//Build the tree based on surface tetra
	TetrahedronSourceMesh meshInterface;
	//	const PxReal gBoxEpsilon = 0.1f;
	meshInterface.initRemap();
	meshInterface.setNbVertices(collisionMesh.mNbVertices);
	meshInterface.setNbTetrahedrons(nbSurfaceTets);

	meshInterface.setPointers(surfaceTets, NULL, verts);

	const PxU32 nbPrimsPerLeaf = 4;
	BV4_AABBTree aabbTree;

	if (!aabbTree.buildFromMesh(meshInterface, nbPrimsPerLeaf))
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "BV4_AABBTree tree failed to build.");
		return;
	}

	PX_FREE(tempCounts);
	PX_FREE(surfaceTets);
	PX_FREE(surfaceVertsHint);
	PX_FREE(surfaceVertToTetRemap);
}

/*//Keep for debugging & verification
void writeTets(const char* path, const PxVec3* tetPoints, PxU32 numPoints, const IndTetrahedron32* tets, PxU32 numTets)
{
	FILE *fp;

	fp = fopen(path, "w+");
	fprintf(fp, "# Tetrahedral mesh generated using\n\n");


	fprintf(fp, "# %d vertices\n", numPoints);
	for (PxU32 i = 0; i < numPoints; ++i)
	{
		fprintf(fp, "v %f %f %f\n", PxF64(tetPoints[i].x), PxF64(tetPoints[i].y), PxF64(tetPoints[i].z));
	}

	fprintf(fp, "\n");
	fprintf(fp, "# %d tetrahedra\n", numTets);
	for (PxU32 i = 0; i < numTets; ++i)
	{
		fprintf(fp, "t %d %d %d %d\n", tets[i].mRef[0], tets[i].mRef[1], tets[i].mRef[2], tets[i].mRef[3]);
	}

	fclose(fp);
}*/

void TetrahedronMeshBuilder::computeModelsMapping(TetrahedronMeshData& simulationMesh,
	const TetrahedronMeshData& collisionMesh, const SoftBodyCollisionData& collisionData, 
	CollisionMeshMappingData& mappingData, bool buildGPUData, const PxBoundedData* vertexToTet)
{
	createCollisionModelMapping(collisionMesh, collisionData, mappingData);

	if (buildGPUData)
	{
		const PxU32 gridModelNbVerts = simulationMesh.mNbVertices;
		PxVec3* gridModelVertices = PX_ALLOCATE(PxVec3, gridModelNbVerts, "gridModelVertices");

		PxVec3* gridModelVerticesInvMass = simulationMesh.mVertices;

		for (PxU32 i = 0; i < gridModelNbVerts; ++i)
		{
			gridModelVertices[i] = gridModelVerticesInvMass[i];
		}

		PX_ASSERT(!(collisionMesh.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));
	
		TetrahedronSourceMesh meshInterface;
		//	const PxReal gBoxEpsilon = 0.1f;
		meshInterface.initRemap();
		meshInterface.setNbVertices(simulationMesh.mNbVertices);
		meshInterface.setNbTetrahedrons(simulationMesh.mNbTetrahedrons);

		IndTetrahedron32* tetrahedron32 = reinterpret_cast<IndTetrahedron32*>(simulationMesh.mTetrahedrons);
		meshInterface.setPointers(tetrahedron32, NULL, gridModelVertices);

		//writeTets("C:\\tmp\\grid.tet", gridModelVertices, simulationMesh.mNbVertices, tetrahedron32, simulationMesh.mNbTetrahedrons);
		//writeTets("C:\\tmp\\col.tet", mVertices, mNbVertices, reinterpret_cast<IndTetrahedron32*>(mTetrahedrons), mNbTetrahedrons);

		const PxU32 nbPrimsPerLeaf = 2;
		BV4_AABBTree aabbTree;
		
		if (!aabbTree.buildFromMesh(meshInterface, nbPrimsPerLeaf))
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "BV32 tree failed to build.");
			return;
		}

		const PxU32 nbTetModelVerts = collisionMesh.mNbVertices;
			
		mappingData.mVertsBarycentricInGridModel = reinterpret_cast<PxReal*>(PX_ALLOC(nbTetModelVerts * sizeof(PxReal) * 4, "mVertsInfoMapOriginalGridModel"));
		mappingData.mVertsRemapInGridModel = reinterpret_cast<PxU32*>(PX_ALLOC(nbTetModelVerts * sizeof(PxU32), "mVertsRemapInGridModel"));

		PxReal* vertsBarycentricInGridModel = mappingData.mVertsBarycentricInGridModel;
		PxU32* vertsRemapInGridModel = mappingData.mVertsRemapInGridModel;

		if (vertexToTet && vertexToTet->count == nbTetModelVerts)
		{
			for (PxU32 i = 0; i < nbTetModelVerts; ++i)
			{
				vertsRemapInGridModel[i] = vertexToTet->at<PxU32>(i);

				const PxVec3& p = collisionMesh.mVertices[i];

				IndTetrahedron32& tetra = tetrahedron32[vertsRemapInGridModel[i]];
				const PxVec3& a = gridModelVertices[tetra.mRef[0]];
				const PxVec3& b = gridModelVertices[tetra.mRef[1]];
				const PxVec3& c = gridModelVertices[tetra.mRef[2]];
				const PxVec3& d = gridModelVertices[tetra.mRef[3]];

				PxVec4 bary;
				computeBarycentric(a, b, c, d, p, bary);

#if PX_DEBUG
				const PxReal eps = 1e-4f;
				PX_ASSERT((bary.x >= -eps && bary.x <= 1.f + eps) && (bary.y >= -eps && bary.y <= 1.f + eps) &&
					(bary.z >= -eps && bary.z <= 1.f + eps) && (bary.w >= -eps && bary.w <= 1.f + eps));

				PX_ASSERT(vertexToTet->at<PxI32>(i) >= 0);
#endif

				const PxU32 index = i * 4;
				vertsBarycentricInGridModel[index] = bary.x;
				vertsBarycentricInGridModel[index + 1] = bary.y;
				vertsBarycentricInGridModel[index + 2] = bary.z;
				vertsBarycentricInGridModel[index + 3] = bary.w;
			}
		}
		else
		{
			for (PxU32 i = 0; i < nbTetModelVerts; ++i)
			{
				DistanceCheck result;
				result.mVerts = gridModelVertices;
				result.mTetrahedron32 = tetrahedron32;

				result.mOriginalVert = collisionMesh.mVertices[i];

				result.mDistanceSq = PX_MAX_F32;
				result.mNbPrimsPerLeaf = 2;
				result.mNbPrims = simulationMesh.mNbTetrahedrons;

				aabbTree.walkDistance(gDistanceCheckCallback, gDistanceNodeCheckCallback, &result);

				IndTetrahedron32& tetra = tetrahedron32[result.mTetInd];

				const PxVec3& a = gridModelVertices[tetra.mRef[0]];
				const PxVec3& b = gridModelVertices[tetra.mRef[1]];
				const PxVec3& c = gridModelVertices[tetra.mRef[2]];
				const PxVec3& d = gridModelVertices[tetra.mRef[3]];

				PxVec4 bary;
				computeBarycentric(a, b, c, d, result.mOriginalVert, bary);

#if PX_DEBUG
				const PxReal eps = 1e-4f;
				PX_ASSERT((bary.x >= -eps && bary.x <= 1.f + eps) && (bary.y >= -eps && bary.y <= 1.f + eps) &&
					(bary.z >= -eps && bary.z <= 1.f + eps) && (bary.w >= -eps && bary.w <= 1.f + eps));
#endif

				const PxU32 index = i * 4;
				vertsBarycentricInGridModel[index] = bary.x;
				vertsBarycentricInGridModel[index + 1] = bary.y;
				vertsBarycentricInGridModel[index + 2] = bary.z;
				vertsBarycentricInGridModel[index + 3] = bary.w;

				vertsRemapInGridModel[i] = result.mTetInd;
			}
		}

		PxU16* colMaterials = collisionMesh.mMaterialIndices;
		PxU16* simMaterials = NULL;
		const PxU32 nbSimMeshTetra = simulationMesh.mNbTetrahedrons;

		if (colMaterials)
		{
			simMaterials = simulationMesh.allocateMaterials();
			for (PxU32 i = 0; i < nbSimMeshTetra; ++i)
			{
				simMaterials[i] = 0xffff;
			}
		}

		const PxU32 nbColMeshTetra = collisionMesh.mNbTetrahedrons;
		PxArray<PxU32> tetIndiceRunSum;
		tetIndiceRunSum.reserve(nbColMeshTetra * 4);
			
		mappingData.mTetsAccumulatedRemapColToSim = reinterpret_cast<PxU32*>( PX_ALLOC(sizeof(PxU32) * nbColMeshTetra, "mTetsAccumulatedRemapColToSim"));
			
		PxU32* runSum = mappingData.mTetsAccumulatedRemapColToSim;

		PxU32 offset = 0;
			
		IndTetrahedron32* colTetra = reinterpret_cast<IndTetrahedron32*>(collisionData.mGRB_primIndices);

		OverlapCheck result;
		result.mSimTetraIndices.reserve(100);

		//IndTetrahedron32* simTetra = reinterpret_cast<IndTetrahedron32*>(simulationMesh.mTetrahedrons);

		for (PxU32 i = 0; i < nbColMeshTetra; ++i)
		{
			IndTetrahedron32& tetInd = colTetra[i];
			const PxVec3 a = collisionMesh.mVertices[tetInd.mRef[0]];
			const PxVec3 b = collisionMesh.mVertices[tetInd.mRef[1]];
			const PxVec3 c = collisionMesh.mVertices[tetInd.mRef[2]];
			const PxVec3 d = collisionMesh.mVertices[tetInd.mRef[3]];

			const PxVec3 max = a.maximum(b.maximum(c.maximum(d)));
			const PxVec3 min = a.minimum(b.minimum(c.minimum(d)));
			PxBounds3 bound(min, max);

			result.mSimTetraIndices.forceSize_Unsafe(0);

			result.mColMeshVerts = collisionMesh.mVertices;
			result.mColTetBound = bound;
			result.mColTetrahedron32 = tetInd;

			result.mSimMeshTetra = tetrahedron32;
			result.mSimMeshVerts = gridModelVertices;

			result.mNbPrimsPerLeaf = 2;
			result.mNbPrims = simulationMesh.mNbTetrahedrons;

			aabbTree.walk(gOverlapCallback, &result);

			const PxU32 size = result.mSimTetraIndices.size();

			PX_ASSERT(size > 0);

			for (PxU32 j = 0; j < size; ++j)
			{
				const PxU32 simTetraInd = result.mSimTetraIndices[j];
				if (simMaterials && simMaterials[simTetraInd] == 0xffff)
					simMaterials[simTetraInd] = colMaterials[i];
				tetIndiceRunSum.pushBack(simTetraInd);
			}

			offset += size;
			runSum[i] = offset;
		}

		if (simMaterials)
		{
			//loop through all the simMaterials to make sure material indices has valid material index. If not,
			//we will use the first material index for the tet materials
			for (PxU32 i = 0; i < nbSimMeshTetra; ++i)
			{
				if (simMaterials[i] == 0xffff)
					simMaterials[i] = 0;
			}
		}

		mappingData.mTetsRemapSize = tetIndiceRunSum.size();
		mappingData.mTetsRemapColToSim = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32) * mappingData.mTetsRemapSize, "mTetsRemapInSimModel"));
		PxMemCopy(mappingData.mTetsRemapColToSim, tetIndiceRunSum.begin(), sizeof(PxU32) * mappingData.mTetsRemapSize);
			
#if PX_DEBUG
		for (PxU32 i = 0; i < tetIndiceRunSum.size(); ++i)
		{
			PX_ASSERT(tetIndiceRunSum[i] < 0xFFFFFFFF);
		}
		for (PxU32 i = 1; i < collisionMesh.mNbTetrahedrons; ++i)
		{
			PX_ASSERT(runSum[i - 1] < runSum[i]);
		}
#endif

		PX_FREE(gridModelVertices);			
	}
}

PX_FORCE_INLINE PxF32 tetVolume(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d)
{
	return (-1.0f / 6.0f) * (a - d).dot((b - d).cross(c - d));
}

template<class T>
static void writeToSimTetraIndice(const T* tetIndices, const PxVec3* verts, TetrahedronT<PxU32>* dest)
{
	const PxVec3 a = verts[tetIndices[0]];
	const PxVec3 b = verts[tetIndices[1]];
	const PxVec3 c = verts[tetIndices[2]];
	const PxVec3 d = verts[tetIndices[3]];
	if (tetVolume(a, b, c, d) < 0.f)
	{
		dest->v[0] = tetIndices[1];
		dest->v[1] = tetIndices[0];
		dest->v[2] = tetIndices[2];
		dest->v[3] = tetIndices[3];
	}
	else
	{
		dest->v[0] = tetIndices[0];
		dest->v[1] = tetIndices[1];
		dest->v[2] = tetIndices[2];
		dest->v[3] = tetIndices[3];
	}
}

void TetrahedronMeshBuilder::computeTetData(const PxTetrahedronMeshDesc& desc, TetrahedronMeshData& mesh)
{
	const PxU32 tetMeshNbPoints = desc.points.count;
	const PxU32 tetMeshNbTets = desc.tetrahedrons.count;
	mesh.mNbVertices = tetMeshNbPoints;
	mesh.mVertices = PX_ALLOCATE(PxVec3, tetMeshNbPoints, "mVertices");
	mesh.mNbTetrahedrons = tetMeshNbTets;
	mesh.mTetrahedrons = PX_ALLOC(tetMeshNbTets * sizeof(TetrahedronT<PxU32>), "mTetrahedrons");
		
	mesh.mFlags = desc.flags; //TODO: flags are not of same type...

	computeLocalBoundsAndGeomEpsilon(mesh.mVertices, tetMeshNbPoints, mesh.mAABB, mesh.mGeomEpsilon);
}

bool transferMass(PxI32 a, PxI32 b, PxArray<PxReal>& newMasses, const PxReal* mass, PxReal maxRatio, PxReal smoothingSpeed)
{
	const PxReal mA = mass[a];
	const PxReal mB = mass[b];
	const PxReal ratio = PxMax(mA, mB) / PxMin(mA, mB);
	if (ratio > maxRatio)
	{
		const PxReal delta = smoothingSpeed * PxMin(mA, mB);
		if (mA > mB)
		{
			newMasses[a] -= delta;
			newMasses[b] += delta;
		}
		else
		{
			newMasses[a] += delta;
			newMasses[b] -= delta;
		}
		return true;
	}
	return false;
}

void smoothMassRatiosWhilePreservingTotalMass( PxReal* massPerNode, PxU32 numNodes, const PxU32* tets, PxI32 numTets, PxReal maxRatio /*= 2.0f*/, PxReal smoothingSpeed = 0.25f)
{
	if (maxRatio == FLT_MAX)
		return;

	PxArray<PxReal> newMasses;
	newMasses.resize(numNodes);
	for (PxU32 i = 0; i < numNodes; ++i)
		newMasses[i] = massPerNode[i];

	const PxU32 tetEdges[6][2] = { {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3} };

	PxU32 l = 4 * numTets;

	PxU32 counter = 0;
	bool success = true;
	while (success)
	{
		++counter;
		success = false;

		for (PxU32 i = 0; i < l; i += 4)
		{
			for (PxU32 j = 0; j < 6; ++j)
				success = success || transferMass(tets[i + tetEdges[j][0]], tets[i + tetEdges[j][1]], newMasses, massPerNode, maxRatio, smoothingSpeed);
		}

		for (PxU32 i = 0; i < numNodes; ++i)
			massPerNode[i] = newMasses[i];

		if (counter > 100000)
			break;
	}
	//printf("%i", counter);
}

void TetrahedronMeshBuilder::computeSimData(const PxTetrahedronMeshDesc& desc, TetrahedronMeshData& simulationMesh, SoftBodySimulationData& simulationData, const PxCookingParams& params)
{
	const PxU32 simTetMeshNbPoints = desc.points.count;
	const PxU32 simTetMeshNbTets = desc.tetrahedrons.count;
	simulationMesh.mNbVertices = simTetMeshNbPoints;
	simulationMesh.mVertices = reinterpret_cast<PxVec3*>(PX_ALLOC(simTetMeshNbPoints * sizeof(PxVec3), "mGridModelVertices"));
	simulationData.mGridModelInvMass = reinterpret_cast<PxReal*>(PX_ALLOC(simTetMeshNbPoints * sizeof(PxReal), "mGridModelInvMass"));
	simulationMesh.mNbTetrahedrons = simTetMeshNbTets;
	simulationMesh.mTetrahedrons = PX_ALLOC(simTetMeshNbTets * sizeof(TetrahedronT<PxU32>), "mGridModelTetrahedrons");
	simulationData.mNumTetsPerElement = desc.tetsPerElement;

	immediateCooking::gatherStrided(desc.points.data, simulationMesh.mVertices, simTetMeshNbPoints, sizeof(PxVec3), desc.points.stride);

	TetrahedronT<PxU32>* gridModelTetrahedrons = reinterpret_cast<TetrahedronT<PxU32>*>(simulationMesh.mTetrahedrons);

	for (PxU32 i = 0; i < simTetMeshNbPoints; ++i)
		simulationData.mGridModelInvMass[i] =  0;

	TetrahedronT<PxU32>* dest = gridModelTetrahedrons;
	const TetrahedronT<PxU32>* pastLastDest = gridModelTetrahedrons + simTetMeshNbTets;

	const PxU8* source = reinterpret_cast<const PxU8*>(desc.tetrahedrons.data);
	if (desc.flags & PxMeshFlag::e16_BIT_INDICES)
	{
		while (dest < pastLastDest)
		{
			const PxU16* tet16 = reinterpret_cast<const PxU16*>(source);
			writeToSimTetraIndice<PxU16>(tet16, simulationMesh.mVertices, dest);
			dest++;
			source += desc.tetrahedrons.stride;
		}
	}
	else
	{
		while (dest < pastLastDest)
		{
			const PxU32* tet32 = reinterpret_cast<const PxU32*>(source);
			writeToSimTetraIndice<PxU32>(tet32, simulationMesh.mVertices, dest);
			dest++;
			source += desc.tetrahedrons.stride;
		}
	}

	simulationData.mGridModelTetraRestPoses = PX_ALLOCATE(PxMat33, desc.tetrahedrons.count, "mGridModelTetraRestPoses");

	computeRestPoseAndPointMass(gridModelTetrahedrons, simulationMesh.mNbTetrahedrons,
		simulationMesh.mVertices, simulationData.mGridModelInvMass, simulationData.mGridModelTetraRestPoses);
				
	PxU32* accumulatedTetrahedronPerPartition = computeGridModelTetrahedronPartitions(simulationMesh, simulationData);
	if (simulationData.mNumTetsPerElement == 1)
		combineGridModelPartitions(simulationMesh, simulationData, &accumulatedTetrahedronPerPartition);
	else
		combineGridModelPartitionsHexMesh(simulationMesh, simulationData, &accumulatedTetrahedronPerPartition, simulationData.mNumTetsPerElement);

	smoothMassRatiosWhilePreservingTotalMass(simulationData.mGridModelInvMass, simulationMesh.mNbVertices, reinterpret_cast<PxU32*>(gridModelTetrahedrons), simulationMesh.mNbTetrahedrons, params.maxWeightRatioInTet);

#if PX_DEBUG
	PxReal max = 0;
	PxReal min = FLT_MAX;
	for (PxU32 i = 0; i < simulationMesh.mNbVertices; ++i)
	{
		PxReal w = simulationData.mGridModelInvMass[i];
		max = PxMax(w, max);
		min = PxMin(w, min);
	}
	PxReal ratio = max / min;
	PX_UNUSED(ratio);
#endif

	for (PxU32 i = 0; i < simulationMesh.mNbVertices; ++i)
	{
		simulationData.mGridModelInvMass[i] = 1.0f / simulationData.mGridModelInvMass[i];
	}

	PX_FREE(accumulatedTetrahedronPerPartition);


	//const PxU32 gridModelNbVerts = simulationMesh.mNbVertices;
	//PxVec3* gridModelVertices = reinterpret_cast<PxVec3*>(PX_ALLOC(gridModelNbVerts * sizeof(PxVec3), "gridModelVertices"));

	//PxVec4* gridModelVerticesInvMass = simulationMesh.mVerticesInvMass;

	//for (PxU32 i = 0; i < gridModelNbVerts; ++i)
	//{
	//	gridModelVertices[i] = gridModelVerticesInvMass[i].getXYZ();
	//}

	//writeTets("C:\\tmp\\grid.tet", gridModelVertices, simulationMesh.mNbVertices, reinterpret_cast<IndTetrahedron32*>(simulationMesh.mTetrahedrons), simulationMesh.mNbTetrahedrons);
	//writeTets("C:\\tmp\\col.tet", mVertices, mNbVertices, reinterpret_cast<IndTetrahedron32*>(mTetrahedrons), mNbTetrahedrons);

	//PX_FREE(gridModelVertices);
}

bool TetrahedronMeshBuilder::computeCollisionData(const PxTetrahedronMeshDesc& collisionMeshDesc, TetrahedronMeshData& collisionMesh, SoftBodyCollisionData& collisionData,
	const PxCookingParams&	params, bool validateMesh)
{
	const PxU32 originalTetrahedronCount = collisionMeshDesc.tetrahedrons.count;
	// Create a local copy that we can modify
	PxTetrahedronMeshDesc desc = collisionMeshDesc;

	// Save simple params
	{
		// Handle implicit topology
		PxU32* topology = NULL;
		if (!desc.tetrahedrons.data)
		{
			// We'll create 32-bit indices
			desc.flags &= ~PxMeshFlag::e16_BIT_INDICES;
			desc.tetrahedrons.stride = sizeof(PxU32) * 4;

			{
				// Non-indexed mesh => create implicit topology
				desc.tetrahedrons.count = desc.points.count / 4;
				// Create default implicit topology
				topology = PX_ALLOCATE(PxU32, desc.points.count, "topology");
				for (PxU32 i = 0; i < desc.points.count; i++)
					topology[i] = i;
				desc.tetrahedrons.data = topology;
			}
		}
		// Continue as usual using our new descriptor

		// Convert and clean the input mesh
		if (!importMesh(collisionMeshDesc, params, collisionMesh, collisionData, validateMesh))
		{
			PX_FREE(topology);
			return false;
		}

		// Cleanup if needed
		PX_FREE(topology);
	}

	//copy the original tetrahedron indices to grb tetrahedron indices if buildGRBData is true
		
	if(!createMidPhaseStructure(collisionMesh, collisionData, params))
		return false;

	recordTetrahedronIndices(collisionMesh, collisionData, params.buildGPUData);

	// Compute local bounds
	computeLocalBoundsAndGeomEpsilon(collisionMesh.mVertices, collisionMesh.mNbVertices, collisionMesh.mAABB, collisionMesh.mGeomEpsilon);

	if(!createGRBMidPhaseAndData(originalTetrahedronCount, collisionMesh, collisionData, params))
		return false;
	
	// Use collisionData.mGRB_primIndices rather than collisionMesh.mTetrahedrons: we want rest poses for the topology-remapped mesh, which is the actual one in simulation.
	computeRestPoseAndPointMass(reinterpret_cast<TetrahedronT<PxU32>*>(collisionData.mGRB_primIndices), collisionMesh.mNbTetrahedrons, collisionMesh.mVertices, NULL, collisionData.mTetraRestPoses);

	return true;
}

bool TetrahedronMeshBuilder::loadFromDesc(const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
	PxSoftBodySimulationDataDesc softbodyDataDesc, TetrahedronMeshData& simulationMesh, SoftBodySimulationData& simulationData, 
	TetrahedronMeshData& collisionMesh, SoftBodyCollisionData& collisionData, CollisionMeshMappingData& mappingData, const PxCookingParams&	params, bool validateMesh)
{		
	if (!simulationMeshDesc.isValid() || !collisionMeshDesc.isValid() || !softbodyDataDesc.isValid())
		return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "TetrahedronMesh::loadFromDesc: desc.isValid() failed!");

	// verify the mesh params
	if (!params.midphaseDesc.isValid())
		return PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "TetrahedronMesh::loadFromDesc: mParams.midphaseDesc.isValid() failed!");

	if (!computeCollisionData(collisionMeshDesc, collisionMesh, collisionData, params, validateMesh))
		return false;
	computeSimData(simulationMeshDesc, simulationMesh, simulationData, params);
	computeModelsMapping(simulationMesh, collisionMesh, collisionData, mappingData, params.buildGPUData, &softbodyDataDesc.vertexToTet);

#if PX_DEBUG
	for (PxU32 i = 0; i < collisionMesh.mNbVertices; ++i) {
		PX_ASSERT(mappingData.mVertsRemapInGridModel[i] < simulationMesh.mNbTetrahedrons);
	}
#endif

	return true;
}

static void writeIndice(const PxU32 serialFlags, const PxU32* indices, const PxU32 nbIndices,
	const bool platformMismatch, PxOutputStream& stream)
{
	//write out tetrahedron indices
	if (serialFlags & IMSF_8BIT_INDICES)
	{
		for (PxU32 i = 0; i < nbIndices; i++)
		{
			PxI8 data = PxI8(indices[i]);
			stream.write(&data, sizeof(PxU8));
		}
	}
	else if (serialFlags & IMSF_16BIT_INDICES)
	{

		for (PxU32 i = 0; i < nbIndices; i++)
			writeWord(PxTo16(indices[i]), platformMismatch, stream);
	}
	else
	{
		writeIntBuffer(indices, nbIndices, platformMismatch, stream);
	}
}

bool TetrahedronMeshBuilder::saveTetrahedronMeshData(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params, const TetrahedronMeshData& mesh)
{
	// Export header
	if (!writeHeader('T', 'E', 'M', 'E', PX_TET_MESH_VERSION, platformMismatch, stream))
		return false;

	// Export serialization flags
	PxU32 serialFlags = 0;
		
	// Compute serialization flags for indices
	PxU32 maxIndex = 0;
	const TetrahedronT<PxU32>* tets = reinterpret_cast<const TetrahedronT<PxU32>*>(mesh.mTetrahedrons);

	for (PxU32 i = 0; i < mesh.mNbTetrahedrons; i++)
	{
		if (tets[i].v[0] > maxIndex)	maxIndex = tets[i].v[0];
		if (tets[i].v[1] > maxIndex)	maxIndex = tets[i].v[1];
		if (tets[i].v[2] > maxIndex)	maxIndex = tets[i].v[2];
		if (tets[i].v[3] > maxIndex)	maxIndex = tets[i].v[3];
	}

	bool force32 = (params.meshPreprocessParams & PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES);
	if (maxIndex <= 0xFFFF && !force32)
		serialFlags |= (maxIndex <= 0xFF ? IMSF_8BIT_INDICES : IMSF_16BIT_INDICES);
	writeDword(serialFlags, platformMismatch, stream);

	// Export mesh
	writeDword(mesh.mNbVertices, platformMismatch, stream);
	//writeDword(collisionData.mNbSurfaceTriangles, platformMismatch, stream);
	writeDword(mesh.mNbTetrahedrons, platformMismatch, stream);

	writeFloatBuffer(&mesh.mVertices->x, mesh.mNbVertices * 3, platformMismatch, stream);

	const PxU32 nbTetIndices = mesh.mNbTetrahedrons * 4;
	//write out tetrahedron indices
	writeIndice(serialFlags, tets->v, nbTetIndices, platformMismatch, stream);

	// Export local bounds
	writeFloat(mesh.mGeomEpsilon, platformMismatch, stream);

	writeFloat(mesh.mAABB.minimum.x, platformMismatch, stream);
	writeFloat(mesh.mAABB.minimum.y, platformMismatch, stream);
	writeFloat(mesh.mAABB.minimum.z, platformMismatch, stream);
	writeFloat(mesh.mAABB.maximum.x, platformMismatch, stream);
	writeFloat(mesh.mAABB.maximum.y, platformMismatch, stream);
	writeFloat(mesh.mAABB.maximum.z, platformMismatch, stream);

	return true;
}
	   
bool TetrahedronMeshBuilder::saveSoftBodyMeshData(PxOutputStream& stream, bool platformMismatch, const PxCookingParams& params, 
	const TetrahedronMeshData& simulationMesh, const SoftBodySimulationData& simulationData, const TetrahedronMeshData& collisionMesh, 
	const SoftBodyCollisionData& collisionData, const CollisionMeshMappingData& mappingData)
{
	// Export header
	if (!writeHeader('S', 'O', 'M', 'E', PX_SOFTBODY_MESH_VERSION, platformMismatch, stream))
		return false;

	// Export serialization flags
	PxU32 serialFlags = 0;
	if (collisionMesh.mMaterialIndices)		serialFlags |= IMSF_MATERIALS;
	if (collisionData.mFaceRemap)			serialFlags |= IMSF_FACE_REMAP;
	//if (mTetraSurfaceHint) serialFlags |= IMSF_ADJACENCIES; // using IMSF_ADJACENCIES to represent surfaceHint for tetrahedron mesh
	//if (mAdjacencies)		serialFlags |= IMSF_ADJACENCIES;
	if (params.buildGPUData)	serialFlags |= IMSF_GRB_DATA;
	// Compute serialization flags for indices
	PxU32 maxIndex = 0;
	const TetrahedronT<PxU32>* tets = reinterpret_cast<const TetrahedronT<PxU32>*>(collisionMesh.mTetrahedrons);

	for (PxU32 i = 0; i < collisionMesh.mNbTetrahedrons; i++)
	{
		if (tets[i].v[0] > maxIndex)	maxIndex = tets[i].v[0];
		if (tets[i].v[1] > maxIndex)	maxIndex = tets[i].v[1];
		if (tets[i].v[2] > maxIndex)	maxIndex = tets[i].v[2];
		if (tets[i].v[3] > maxIndex)	maxIndex = tets[i].v[3];
	}

	const TetrahedronT<PxU32>* gridModelTets = reinterpret_cast<const TetrahedronT<PxU32>*>(simulationMesh.mTetrahedrons);

	for (PxU32 i = 0; i < simulationMesh.mNbTetrahedrons; i++)
	{
		if (gridModelTets[i].v[0] > maxIndex)	maxIndex = gridModelTets[i].v[0];
		if (gridModelTets[i].v[1] > maxIndex)	maxIndex = gridModelTets[i].v[1];
		if (gridModelTets[i].v[2] > maxIndex)	maxIndex = gridModelTets[i].v[2];
		if (gridModelTets[i].v[3] > maxIndex)	maxIndex = gridModelTets[i].v[3];
	}

	bool force32 = (params.meshPreprocessParams & PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES);
	if (maxIndex <= 0xFFFF && !force32)
		serialFlags |= (maxIndex <= 0xFF ? IMSF_8BIT_INDICES : IMSF_16BIT_INDICES);
	writeDword(serialFlags, platformMismatch, stream);

	// Export mesh
	writeDword(collisionMesh.mNbVertices, platformMismatch, stream);
	//writeDword(collisionData.mNbSurfaceTriangles, platformMismatch, stream);
	writeDword(collisionMesh.mNbTetrahedrons, platformMismatch, stream);

	writeFloatBuffer(&collisionMesh.mVertices->x, collisionMesh.mNbVertices * 3, platformMismatch, stream);
		
	const PxU32 nbTetIndices = collisionMesh.mNbTetrahedrons * 4;
	//write out tetrahedron indices
	writeIndice(serialFlags, tets->v, nbTetIndices, platformMismatch, stream);
		
	//const PxU32 nbSurfaceTriangleIndices = collisionData.mNbSurfaceTriangles * 3;
	//const IndexedTriangle32* surfaceTriangles = reinterpret_cast<const IndexedTriangle32*>(collisionData.mSurfaceTriangles);
	//write out surface triangle indices
	//writeIndice(serialFlags, surfaceTriangles->v, nbSurfaceTriangleIndices, platformMismatch, stream);
		
	if (collisionMesh.mMaterialIndices)
		writeWordBuffer(collisionMesh.mMaterialIndices, collisionMesh.mNbTetrahedrons, platformMismatch, stream);

	if (collisionData.mFaceRemap)
	{
		PxU32 maxId = computeMaxIndex(collisionData.mFaceRemap, collisionMesh.mNbTetrahedrons);
		writeDword(maxId, platformMismatch, stream);
		storeIndices(maxId, collisionMesh.mNbTetrahedrons, collisionData.mFaceRemap, stream, platformMismatch);
		//		writeIntBuffer(mMeshData.mFaceRemap, mMeshData.mNbTriangles, platformMismatch, stream);
	}

/*	if (mAdjacencies)
		writeIntBuffer(mAdjacencies, mNbTetrahedrons * 4, platformMismatch, stream);*/

	// Export midphase structure
	saveMidPhaseStructure(stream, platformMismatch, collisionData);

	// Export local bounds
	writeFloat(collisionMesh.mGeomEpsilon, platformMismatch, stream);

	writeFloat(collisionMesh.mAABB.minimum.x, platformMismatch, stream);
	writeFloat(collisionMesh.mAABB.minimum.y, platformMismatch, stream);
	writeFloat(collisionMesh.mAABB.minimum.z, platformMismatch, stream);
	writeFloat(collisionMesh.mAABB.maximum.x, platformMismatch, stream);
	writeFloat(collisionMesh.mAABB.maximum.y, platformMismatch, stream);
	writeFloat(collisionMesh.mAABB.maximum.z, platformMismatch, stream);

	// GRB write -----------------------------------------------------------------
	if (params.buildGPUData)
	{
		const PxU32* tetIndices = reinterpret_cast<PxU32*>(collisionData.mGRB_primIndices);
		writeIndice(serialFlags, tetIndices, nbTetIndices, platformMismatch, stream);

		//writeIntBuffer(reinterpret_cast<PxU32*>(mMeshData.mGRB_triIndices), , mMeshData.mNbTriangles*3, platformMismatch, stream);

		//writeIntBuffer(reinterpret_cast<PxU32 *>(mGRB_surfaceTriIndices), mNbTriangles*3, platformMismatch, stream);
		stream.write(collisionData.mGRB_tetraSurfaceHint, collisionMesh.mNbTetrahedrons * sizeof(PxU8));

		//writeIntBuffer(reinterpret_cast<PxU32 *>(mGRB_primAdjacencies), mNbTetrahedrons * 4, platformMismatch, stream);
		writeIntBuffer(collisionData.mGRB_faceRemap, collisionMesh.mNbTetrahedrons, platformMismatch, stream);
		writeIntBuffer(collisionData.mGRB_faceRemapInverse, collisionMesh.mNbTetrahedrons, platformMismatch, stream);

		stream.write(collisionData.mTetraRestPoses, collisionMesh.mNbTetrahedrons * sizeof(PxMat33));

		//Export GPU midphase structure
		BV32TriangleMeshBuilder::saveMidPhaseStructure(collisionData.mGRB_BV32Tree, stream, platformMismatch);

		writeDword(simulationMesh.mNbTetrahedrons, platformMismatch, stream);
		writeDword(simulationMesh.mNbVertices, platformMismatch, stream);
		writeDword(simulationData.mGridModelNbPartitions, platformMismatch, stream);
		writeDword(simulationData.mGridModelMaxTetsPerPartitions, platformMismatch, stream);
		writeDword(simulationData.mGMRemapOutputSize, platformMismatch, stream);
		writeDword(simulationData.mNumTetsPerElement, platformMismatch, stream);
		writeDword(mappingData.mCollisionNbTetrahedronsReferences, platformMismatch, stream);
		writeDword(mappingData.mTetsRemapSize, platformMismatch, stream);
			
		const PxU32 nbGridModeIndices = 4 * simulationMesh.mNbTetrahedrons;
		const PxU32* gridModelTetIndices = reinterpret_cast<PxU32*>(simulationMesh.mTetrahedrons);
		writeIndice(serialFlags, gridModelTetIndices, nbGridModeIndices, platformMismatch, stream);

		const PxU32 numVertsPerElement = (simulationData.mNumTetsPerElement == 5 || simulationData.mNumTetsPerElement == 6) ? 8 : 4;
		const PxU32 numElements = simulationMesh.mNbTetrahedrons / simulationData.mNumTetsPerElement;

		writeFloatBuffer(&simulationMesh.mVertices->x, simulationMesh.mNbVertices * 3, platformMismatch, stream);
			
		if (simulationMesh.mMaterialIndices)
			writeWordBuffer(simulationMesh.mMaterialIndices, simulationMesh.mNbTetrahedrons, platformMismatch, stream);

			
		writeFloatBuffer(simulationData.mGridModelInvMass, simulationMesh.mNbVertices * 1, platformMismatch, stream);

		stream.write(simulationData.mGridModelTetraRestPoses, simulationMesh.mNbTetrahedrons * sizeof(PxMat33));

		stream.write(simulationData.mGridModelOrderedTetrahedrons, numElements * sizeof(PxU32));
			
		stream.write(simulationData.mGMRemapOutputCP, numElements * numVertsPerElement * sizeof(PxU32));

		stream.write(simulationData.mGMAccumulatedPartitionsCP, simulationData.mGridModelNbPartitions * sizeof(PxU32));

		stream.write(simulationData.mGMAccumulatedCopiesCP, simulationMesh.mNbVertices * sizeof(PxU32));

		stream.write(mappingData.mCollisionAccumulatedTetrahedronsRef, collisionMesh.mNbVertices * sizeof(PxU32));

		stream.write(mappingData.mCollisionTetrahedronsReferences, mappingData.mCollisionNbTetrahedronsReferences * sizeof(PxU32));

		stream.write(mappingData.mCollisionSurfaceVertsHint, collisionMesh.mNbVertices * sizeof(PxU8));

		stream.write(mappingData.mCollisionSurfaceVertToTetRemap, collisionMesh.mNbVertices * sizeof(PxU32));

		stream.write(simulationData.mGMPullIndices, numElements * numVertsPerElement *sizeof(PxU32));

		writeFloatBuffer(mappingData.mVertsBarycentricInGridModel, collisionMesh.mNbVertices * 4, platformMismatch, stream);

		writeIntBuffer(mappingData.mVertsRemapInGridModel, collisionMesh.mNbVertices, platformMismatch, stream);		
			
		writeIntBuffer(mappingData.mTetsRemapColToSim, mappingData.mTetsRemapSize, platformMismatch, stream);

		writeIntBuffer(mappingData.mTetsAccumulatedRemapColToSim, collisionMesh.mNbTetrahedrons, platformMismatch, stream);
	}

	// End of GRB write ----------------------------------------------------------

	return true;
}

bool TetrahedronMeshBuilder::createMidPhaseStructure(TetrahedronMeshData& collisionMesh, SoftBodyCollisionData& collisionData, const PxCookingParams& params)
{
	const PxReal gBoxEpsilon = 2e-4f;

	TetrahedronSourceMesh& meshInterface = collisionData.mMeshInterface;
	//	const PxReal gBoxEpsilon = 0.1f;
	meshInterface.initRemap();
	meshInterface.setNbVertices(collisionMesh.mNbVertices);
	meshInterface.setNbTetrahedrons(collisionMesh.mNbTetrahedrons);

	IndTetrahedron32* tetrahedrons32 = NULL;
	IndTetrahedron16* tetrahedrons16 = NULL;
	if (collisionMesh.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES)
		tetrahedrons16 = reinterpret_cast<IndTetrahedron16*>(collisionMesh.mTetrahedrons);
	else
		tetrahedrons32 = reinterpret_cast<IndTetrahedron32*>(collisionMesh.mTetrahedrons);

	collisionData.mMeshInterface.setPointers(tetrahedrons32, tetrahedrons16, collisionMesh.mVertices);

	const PxU32 nbTetsPerLeaf = 15;

	if (!BuildBV4Ex(collisionData.mBV4Tree, meshInterface, gBoxEpsilon, nbTetsPerLeaf, false))
		return PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "BV4 tree failed to build.");
			
	const PxU32* order = meshInterface.getRemap();
	if (!params.suppressTriangleMeshRemapTable || params.buildGPUData)
	{
		PxU32* newMap = PX_ALLOCATE(PxU32, collisionMesh.mNbTetrahedrons, "mFaceRemap");
		for (PxU32 i = 0; i < collisionMesh.mNbTetrahedrons; i++)
			newMap[i] = collisionData.mFaceRemap ? collisionData.mFaceRemap[order[i]] : order[i];
		PX_FREE(collisionData.mFaceRemap);
		collisionData.mFaceRemap = newMap;
	}

	meshInterface.releaseRemap();
	return true;
}

void TetrahedronMeshBuilder::saveMidPhaseStructure(PxOutputStream& stream, bool mismatch, const SoftBodyCollisionData& collisionData)
{
	// PT: in version 1 we defined "mismatch" as:
	// const bool mismatch = (littleEndian() == 1);
	// i.e. the data was *always* saved to file in big-endian format no matter what.
	// In version>1 we now do the same as for other structures in the SDK: the data is
	// exported either as little or big-endian depending on the passed parameter.
	const PxU32 bv4StructureVersion = 3;

	writeChunk('B', 'V', '4', ' ', stream);
	writeDword(bv4StructureVersion, mismatch, stream);

	writeFloat(collisionData.mBV4Tree.mLocalBounds.mCenter.x, mismatch, stream);
	writeFloat(collisionData.mBV4Tree.mLocalBounds.mCenter.y, mismatch, stream);
	writeFloat(collisionData.mBV4Tree.mLocalBounds.mCenter.z, mismatch, stream);
	writeFloat(collisionData.mBV4Tree.mLocalBounds.mExtentsMagnitude, mismatch, stream);
					
	writeDword(collisionData.mBV4Tree.mInitData, mismatch, stream);
					
	writeFloat(collisionData.mBV4Tree.mCenterOrMinCoeff.x, mismatch, stream);
	writeFloat(collisionData.mBV4Tree.mCenterOrMinCoeff.y, mismatch, stream);
	writeFloat(collisionData.mBV4Tree.mCenterOrMinCoeff.z, mismatch, stream);
	writeFloat(collisionData.mBV4Tree.mExtentsOrMaxCoeff.x, mismatch, stream);
	writeFloat(collisionData.mBV4Tree.mExtentsOrMaxCoeff.y, mismatch, stream);
	writeFloat(collisionData.mBV4Tree.mExtentsOrMaxCoeff.z, mismatch, stream);

	// PT: version 3
	writeDword(PxU32(collisionData.mBV4Tree.mQuantized), mismatch, stream);

	writeDword(collisionData.mBV4Tree.mNbNodes, mismatch, stream);

#ifdef GU_BV4_USE_SLABS
	// PT: we use BVDataPacked to get the size computation right, but we're dealing with BVDataSwizzled here!
	const PxU32 NodeSize = collisionData.mBV4Tree.mQuantized ? sizeof(BVDataPackedQ) : sizeof(BVDataPackedNQ);
	stream.write(collisionData.mBV4Tree.mNodes, NodeSize*collisionData.mBV4Tree.mNbNodes);
	PX_ASSERT(!mismatch);
#else
	#error	Not implemented
#endif
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool BV32TetrahedronMeshBuilder::createMidPhaseStructure(const PxCookingParams& params, TetrahedronMeshData& collisionMesh, BV32Tree& bv32Tree, SoftBodyCollisionData& collisionData)
{
	PX_UNUSED(params);
	PX_UNUSED(collisionMesh);
	PX_UNUSED(bv32Tree);
	const PxReal gBoxEpsilon = 2e-4f;

	TetrahedronSourceMesh meshInterface;
	//	const PxReal gBoxEpsilon = 0.1f;
	meshInterface.initRemap();
	meshInterface.setNbVertices(collisionMesh.mNbVertices);
	meshInterface.setNbTetrahedrons(collisionMesh.mNbTetrahedrons);

	//meshInterface.setNbVertices(meshData.mNbVertices);
	//meshInterface.setNbTriangles(meshData.mNbTriangles);

	PX_ASSERT(!(collisionMesh.mFlags & PxTriangleMeshFlag::e16_BIT_INDICES));

	IndTetrahedron32* tetrahedron32 = reinterpret_cast<IndTetrahedron32*>(collisionData.mGRB_primIndices);

	meshInterface.setPointers(tetrahedron32, NULL, collisionMesh.mVertices);

	PxU32 nbTetrahedronPerLeaf = 32;

	if (!BuildBV32Ex(bv32Tree, meshInterface, gBoxEpsilon, nbTetrahedronPerLeaf))
		return PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "BV32 tree failed to build.");

	const PxU32* order = meshInterface.getRemap();

	if (collisionMesh.mMaterialIndices)
	{
		PxFEMMaterialTableIndex* newMat = PX_ALLOCATE(PxFEMMaterialTableIndex, collisionMesh.mNbTetrahedrons, "mMaterialIndices");
		for (PxU32 i = 0; i < collisionMesh.mNbTetrahedrons; i++)
			newMat[i] = collisionMesh.mMaterialIndices[order[i]];
		PX_FREE(collisionMesh.mMaterialIndices);
		collisionMesh.mMaterialIndices = newMat;
	}

	//suppressTriangleMeshRemapTable can use for tetrahedron mesh remap table
	if (!params.suppressTriangleMeshRemapTable || params.buildGPUData)
	{
		PxU32* newMap = PX_ALLOCATE(PxU32, collisionMesh.mNbTetrahedrons, "mGRB_faceRemap");
		for (PxU32 i = 0; i<collisionMesh.mNbTetrahedrons; i++)
			newMap[i] = collisionData.mGRB_faceRemap ? collisionData.mGRB_faceRemap[order[i]] : order[i];
		PX_FREE(collisionData.mGRB_faceRemap);
		collisionData.mGRB_faceRemap = newMap;
		
		PxU32* newMapInverse = PX_ALLOCATE(PxU32, collisionMesh.mNbTetrahedrons, "mGRB_faceRemapInverse");
		for (PxU32 i = 0; i < collisionMesh.mNbTetrahedrons; ++i)
			newMapInverse[collisionData.mGRB_faceRemap[i]] = i;
		PX_FREE(collisionData.mGRB_faceRemapInverse);
		collisionData.mGRB_faceRemapInverse = newMapInverse;
	}

	meshInterface.releaseRemap();
	return true;
}

void BV32TetrahedronMeshBuilder::saveMidPhaseStructure(BV32Tree* bv32Tree, PxOutputStream& stream, bool mismatch)
{
	// PT: in version 1 we defined "mismatch" as:
	// const bool mismatch = (littleEndian() == 1);
	// i.e. the data was *always* saved to file in big-endian format no matter what.
	// In version>1 we now do the same as for other structures in the SDK: the data is
	// exported either as little or big-endian depending on the passed parameter.
	const PxU32 bv32StructureVersion = 2;

	writeChunk('B', 'V', '3', '2', stream);
	writeDword(bv32StructureVersion, mismatch, stream);

	writeFloat(bv32Tree->mLocalBounds.mCenter.x, mismatch, stream);
	writeFloat(bv32Tree->mLocalBounds.mCenter.y, mismatch, stream);
	writeFloat(bv32Tree->mLocalBounds.mCenter.z, mismatch, stream);
	writeFloat(bv32Tree->mLocalBounds.mExtentsMagnitude, mismatch, stream);

	writeDword(bv32Tree->mInitData, mismatch, stream);

	writeDword(bv32Tree->mNbPackedNodes, mismatch, stream);

	PX_ASSERT(bv32Tree->mNbPackedNodes > 0);
	for (PxU32 i = 0; i < bv32Tree->mNbPackedNodes; ++i)
	{
		BV32DataPacked& node = bv32Tree->mPackedNodes[i];

		const PxU32 nbElements = node.mNbNodes * 4;
		writeDword(node.mNbNodes, mismatch, stream);
		writeDword(node.mDepth, mismatch, stream);
		WriteDwordBuffer(node.mData, node.mNbNodes, mismatch, stream);
		writeFloatBuffer(&node.mMin[0].x, nbElements, mismatch, stream);
		writeFloatBuffer(&node.mMax[0].x, nbElements, mismatch, stream);	
	}
}

///////////////////////////////////////////////////////////////////////////////

bool immediateCooking::cookTetrahedronMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& meshDesc, PxOutputStream& stream)
{
	TetrahedronMeshData data;
	TetrahedronMeshBuilder::computeTetData(meshDesc, data);
	TetrahedronMeshBuilder::saveTetrahedronMeshData(stream, platformMismatch(), params, data);
	return true;
}

PxTetrahedronMesh* immediateCooking::createTetrahedronMesh(const PxCookingParams& /*params*/, const PxTetrahedronMeshDesc& meshDesc, PxInsertionCallback& insertionCallback)
{
	PX_FPU_GUARD;

	TetrahedronMeshData tetData;
	TetrahedronMeshBuilder::computeTetData(meshDesc, tetData);

	PxConcreteType::Enum type = PxConcreteType::eTETRAHEDRON_MESH; 
	PxTetrahedronMesh* tetMesh = static_cast<PxTetrahedronMesh*>(insertionCallback.buildObjectFromData(type, &tetData));

	return tetMesh;
}

bool immediateCooking::cookSoftBodyMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
											const PxSoftBodySimulationDataDesc& softbodyDataDesc, PxOutputStream& stream)
{
	PX_FPU_GUARD;

	TetrahedronMeshData simulationMesh;
	SoftBodySimulationData simulationData;
	TetrahedronMeshData collisionMesh;
	SoftBodyCollisionData collisionData;
	CollisionMeshMappingData mappingData;
	SoftBodyMeshData data(simulationMesh, simulationData, collisionMesh, collisionData, mappingData);
	if(!TetrahedronMeshBuilder::loadFromDesc(simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, data.mSimulationMesh, data.mSimulationData, data.mCollisionMesh, data.mCollisionData, data.mMappingData, params, false))
		return false;

	TetrahedronMeshBuilder::saveSoftBodyMeshData(stream, platformMismatch(), params, data.mSimulationMesh, data.mSimulationData, data.mCollisionMesh, data.mCollisionData, data.mMappingData);
	return true;
}

PxSoftBodyMesh* immediateCooking::createSoftBodyMesh(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc, const PxTetrahedronMeshDesc& collisionMeshDesc,
																const PxSoftBodySimulationDataDesc& softbodyDataDesc, PxInsertionCallback& insertionCallback)
{
	PX_UNUSED(simulationMeshDesc);
	PX_UNUSED(collisionMeshDesc);
	PX_UNUSED(softbodyDataDesc);
	PX_UNUSED(insertionCallback);

	// cooking code does lots of float bitwise reinterpretation that generates exceptions
	PX_FPU_GUARD;

	TetrahedronMeshData simulationMesh;
	SoftBodySimulationData simulationData;
	TetrahedronMeshData collisionMesh;
	SoftBodyCollisionData collisionData;
	CollisionMeshMappingData mappingData;
	SoftBodyMeshData data(simulationMesh, simulationData, collisionMesh, collisionData, mappingData);
	if(!TetrahedronMeshBuilder::loadFromDesc(simulationMeshDesc, collisionMeshDesc, softbodyDataDesc, data.mSimulationMesh, data.mSimulationData, data.mCollisionMesh, data.mCollisionData, data.mMappingData, params, false))
		return NULL;

	PxConcreteType::Enum type = PxConcreteType::eSOFTBODY_MESH; 
	PxSoftBodyMesh* tetMesh = static_cast<PxSoftBodyMesh*>(insertionCallback.buildObjectFromData(type, &data));
	
	/*SoftbodySimulationTetrahedronMesh simulationMesh(data.simulationMesh, data.simulationData);
	SoftbodyCollisionTetrahedronMesh collisionMesh(data.collisionMesh, data.collisionData);
	SoftbodyShapeMapping embedding(data.mappingData);

	SoftBodyMesh* tetMesh = NULL;
	PX_NEW_SERIALIZED(tetMesh, SoftBodyMesh)(simulationMesh, collisionMesh, embedding);*/

	return tetMesh;
}

PxCollisionMeshMappingData* immediateCooking::computeModelsMapping(const PxCookingParams& params, PxTetrahedronMeshData& simulationMesh, const PxTetrahedronMeshData& collisionMesh, 
																				const PxSoftBodyCollisionData& collisionData, const PxBoundedData* vertexToTet)
{
	CollisionMeshMappingData* mappingData = PX_NEW(CollisionMeshMappingData);
	TetrahedronMeshBuilder::computeModelsMapping(*static_cast<TetrahedronMeshData*>(&simulationMesh),
		*static_cast<const TetrahedronMeshData*>(&collisionMesh), *static_cast<const SoftBodyCollisionData*>(&collisionData), *mappingData, params.buildGPUData, vertexToTet);
	return mappingData;
}
	
PxCollisionTetrahedronMeshData* immediateCooking::computeCollisionData(const PxCookingParams& params, const PxTetrahedronMeshDesc& collisionMeshDesc)
{
	PX_UNUSED(collisionMeshDesc);

	TetrahedronMeshData* mesh = PX_NEW(TetrahedronMeshData);
	SoftBodyCollisionData* collisionData = PX_NEW(SoftBodyCollisionData);

	if(!TetrahedronMeshBuilder::computeCollisionData(collisionMeshDesc, *mesh, *collisionData, params, false)) {
		PX_FREE(mesh);
		PX_FREE(collisionData);
		return NULL;
	}
	CollisionTetrahedronMeshData* data = PX_NEW(CollisionTetrahedronMeshData);
	data->mMesh = mesh;
	data->mCollisionData = collisionData;
	return data;
}

PxSimulationTetrahedronMeshData* immediateCooking::computeSimulationData(const PxCookingParams& params, const PxTetrahedronMeshDesc& simulationMeshDesc)
{
	TetrahedronMeshData* mesh = PX_NEW(TetrahedronMeshData);
	SoftBodySimulationData* simulationData = PX_NEW(SoftBodySimulationData);
	//KS - This really needs the collision mesh as well. 
	TetrahedronMeshBuilder::computeSimData(simulationMeshDesc, *mesh, *simulationData, params);
	SimulationTetrahedronMeshData* data = PX_NEW(SimulationTetrahedronMeshData);
	data->mMesh = mesh;
	data->mSimulationData = simulationData;
	return data;
}

PxSoftBodyMesh*	immediateCooking::assembleSoftBodyMesh(PxTetrahedronMeshData& simulationMesh, PxSoftBodySimulationData& simulationData, PxTetrahedronMeshData& collisionMesh,
																	PxSoftBodyCollisionData& collisionData, PxCollisionMeshMappingData& mappingData, PxInsertionCallback& insertionCallback)
{
	SoftBodyMeshData data(static_cast<TetrahedronMeshData&>(simulationMesh),
		static_cast<SoftBodySimulationData&>(simulationData),
		static_cast<TetrahedronMeshData&>(collisionMesh),
		static_cast<SoftBodyCollisionData&>(collisionData),
		static_cast<CollisionMeshMappingData&>(mappingData));

	PxConcreteType::Enum type = PxConcreteType::eSOFTBODY_MESH;
	PxSoftBodyMesh* tetMesh = static_cast<PxSoftBodyMesh*>(insertionCallback.buildObjectFromData(type, &data));

	return tetMesh;
}
	
PxSoftBodyMesh*	immediateCooking::assembleSoftBodyMesh_Sim(PxSimulationTetrahedronMeshData& simulationMesh, PxCollisionTetrahedronMeshData& collisionMesh, 
															PxCollisionMeshMappingData& mappingData, PxInsertionCallback& insertionCallback)
{
	return assembleSoftBodyMesh(*simulationMesh.getMesh(), *simulationMesh.getData(), *collisionMesh.getMesh(), *collisionMesh.getData(), mappingData, insertionCallback);
}
