// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgFEMCloth.h"
#include "GuTriangleMesh.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "PxsHeapStats.h"
#include "CmVirtualAllocatorCallback.h"
#include "foundation/PxVec2.h"
#include "foundation/PxSort.h"
#include "cutil_math.h"

namespace physx
{

/*******************************************************************************
 *
 *
 * Forward Declarations
 *
 *
 ******************************************************************************/

// init
static void initialMaterialData(const PxU16* materialIndices, const PxU16* materialHandles, const PxsDeformableSurfaceMaterialData* materials,
								const PxU32* gpuToCpuFaceRemap, const PxU32 triangleIndex, PxU32 vi0, PxU32 vi1, PxU32 vi2,
								const PxU32 nbMaterials, PxU16* destMaterialIndices, float* destDynamicFrictions);

// query data for FEM models
void queryRestConfigurationPerTriangle(float4& triangleRestPosInv, PxU32 vi0, PxU32 vi1, PxU32 vi2, const PxVec3* const positions);

// others
void queryVertexIndicesAndTriangleIndexPairs(PxArray<uint4>& vertexIndicesAndTriangleIndexPair, PxU32 vi0, PxU32 vi1, PxU32 vi2, PxU32 ti);

void encodeEdgeOwnership(PxU8* edgeOwnershipNibbles, PxU32 triIndex, bool ownsEdge0, bool ownsEdge1, bool ownsEdge2);
void computeEdgeOwnership(PxArray<bool>& triangleUsed, PxArray<PxU8>& edgeOwnershipNibbles, const uint4* triangleVertexIndices,
						  PxU32 nbVertices, PxU32 nbTriangles, const PxArray<PxU32>& innerTriangles,
						  const PxArray<PxU32>& borderTriangles, PxU32 maxEdgesPerVertex);

/*******************************************************************************
 *
 *
 * Init functions
 *
 *
 ******************************************************************************/

PxU32 PxgFEMClothUtil::computeTriangleMeshByteSize(const Gu::TriangleMesh* triangleMesh)
{
	const PxU32 meshDataSize = sizeof(uint4); // (nbVerts, nbTriangles, maxDepth, nbBv32TreeNodes)

	// ML: don't know whether we need to have local bound
	Gu::BV32Tree* bv32Tree = triangleMesh->mGRB_BV32Tree;
	const PxU32 bv32Size = bv32Tree->mNbPackedNodes * sizeof(Gu::BV32DataPacked) + bv32Tree->mMaxTreeDepth * sizeof(Gu::BV32DataDepthInfo) +
						   bv32Tree->mNbPackedNodes * sizeof(PxU32);

	return meshDataSize + bv32Size;
}

PxU32 PxgFEMClothUtil::loadOutTriangleMesh(void* mem, const Gu::TriangleMesh* triangleMesh)
{
	const PxU32 nbTriangles = triangleMesh->getNbTrianglesFast();
	const PxU32 numVerts = triangleMesh->getNbVerticesFast();

	Gu::BV32Tree* bv32Tree = triangleMesh->mGRB_BV32Tree;

	PxU8* m = (PxU8*)mem;
	*((uint4*)m) = make_uint4(numVerts, nbTriangles, bv32Tree->mMaxTreeDepth, bv32Tree->mNbPackedNodes);
	m += sizeof(uint4);

	// Midphase
	PxMemCopy(m, bv32Tree->mPackedNodes, sizeof(Gu::BV32DataPacked) * bv32Tree->mNbPackedNodes);
	m += sizeof(Gu::BV32DataPacked) * bv32Tree->mNbPackedNodes;

	PX_ASSERT(bv32Tree->mNbPackedNodes > 0);

	PxMemCopy(m, bv32Tree->mTreeDepthInfo, sizeof(Gu::BV32DataDepthInfo) * bv32Tree->mMaxTreeDepth);
	m += sizeof(Gu::BV32DataDepthInfo) * bv32Tree->mMaxTreeDepth;

	PxMemCopy(m, bv32Tree->mRemapPackedNodeIndexWithDepth, sizeof(PxU32) * bv32Tree->mNbPackedNodes);
	m += sizeof(PxU32) * bv32Tree->mNbPackedNodes;

	return bv32Tree->mNbPackedNodes;
}

// Derives the mesh topology a cloth needs, the bending triangle pairs and the edge ownership deciding
// which triangle tests each shared edge. It reads the mesh and nothing else, so it can run before a
// cloth exists on the GPU.
static void computeTriangleTopology(const Gu::TriangleMesh& triangleMesh, PxArray<PxU8>& edgeOwnershipNibbles,
									PxArray<uint2>& trianglePairTriangleIndices, PxArray<uint4>& trianglePairVertexIndices)
{
	const PxU32 nbTriangles = triangleMesh.getNbTrianglesFast();
	const PxU32 nbVertices = triangleMesh.getNbVerticesFast();

	// Temporary triangle vertex indices
	PxArray<uint4> triangleVertexIndices(nbTriangles);

	if(triangleMesh.has16BitIndices())
	{
		const PxU16* const triangleInds = reinterpret_cast<PxU16*>(triangleMesh.mGRB_triIndices);
		for(PxU32 i = 0; i < nbTriangles; ++i)
			triangleVertexIndices[i] = make_uint4(triangleInds[3 * i + 0], triangleInds[3 * i + 1], triangleInds[3 * i + 2], 0);
	}
	else
	{
		const PxU32* const triangleInds = reinterpret_cast<PxU32*>(triangleMesh.mGRB_triIndices);
		for(PxU32 i = 0; i < nbTriangles; ++i)
			triangleVertexIndices[i] = make_uint4(triangleInds[3 * i + 0], triangleInds[3 * i + 1], triangleInds[3 * i + 2], 0);
	}

	PxArray<uint4> vertexIndicesAndTriangleIndexPair;
	const PxU32 nbConservativeTrianglePairs = 3 * nbTriangles;
	vertexIndicesAndTriangleIndexPair.reserve(nbConservativeTrianglePairs);
	vertexIndicesAndTriangleIndexPair.forceSize_Unsafe(nbConservativeTrianglePairs);
	trianglePairTriangleIndices.reserve(nbConservativeTrianglePairs);
	trianglePairVertexIndices.reserve(nbConservativeTrianglePairs);

	for(PxU32 i = 0; i < nbTriangles; ++i)
	{
		queryVertexIndicesAndTriangleIndexPairs(vertexIndicesAndTriangleIndexPair, triangleVertexIndices[i].x,
												triangleVertexIndices[i].y, triangleVertexIndices[i].z, i);
	}

	// "unordered" triangle-pairs
	struct IndexPredicate
	{
		IndexPredicate(PxU32 nbVertices) : mNbVertices(nbVertices) {}
		bool operator()(const uint4& left, const uint4& right) const
		{
			const PxU32 leftId = left.x + mNbVertices * left.y;
			const PxU32 rightId = right.x + mNbVertices * right.y;
			return leftId > rightId;
		}
		PxU32 mNbVertices;
	};
	IndexPredicate indexPredicate(nbVertices);
	PxSort(vertexIndicesAndTriangleIndexPair.begin(), vertexIndicesAndTriangleIndexPair.size(), indexPredicate);

	PxU32 numBendPairs = 0;
	bool isNewEdge = false;

	PxArray<PxU32> edgeVertices;
	edgeVertices.reserve(6 * nbTriangles);

	PxArray<PxU32> borderTriangles;
	borderTriangles.reserve(nbTriangles);

	PxArray<PxU32> innerTriangles;
	innerTriangles.reserve(nbTriangles);

	PxArray<PxU32> numEdgesPerVertex(nbVertices, 0);

	PxU32 borderEdgeCount = 0;
	for(PxU32 e = 1; e < vertexIndicesAndTriangleIndexPair.size(); ++e)
	{
		const bool isLastIter = e == vertexIndicesAndTriangleIndexPair.size() - 1;
		const PxU32 prevE = e - 1;

		if(vertexIndicesAndTriangleIndexPair[prevE].x == vertexIndicesAndTriangleIndexPair[e].x &&
		   vertexIndicesAndTriangleIndexPair[prevE].y == vertexIndicesAndTriangleIndexPair[e].y)
		{
			isNewEdge = false;
			++numBendPairs;
		}
		else
		{
			isNewEdge = true;
		}

		// Handling non-manifold cases
		if(numBendPairs > 0 && (isNewEdge || isLastIter))
		{
			PxU32 firstIndex = (isLastIter && !isNewEdge) ? e - numBendPairs : e - numBendPairs - 1;
			PxU32 lastIndex = (isLastIter && !isNewEdge) ? e : e - 1;

			const PxU32 vi2 = vertexIndicesAndTriangleIndexPair[lastIndex].x;
			const PxU32 vi3 = vertexIndicesAndTriangleIndexPair[lastIndex].y;
			const PxU32 lastT = vertexIndicesAndTriangleIndexPair[lastIndex].w;

			++numEdgesPerVertex[vi2];
			++numEdgesPerVertex[vi3];

			edgeVertices.pushBack(vi2);
			edgeVertices.pushBack(vi3);
			innerTriangles.pushBack(lastT);

			for(PxU32 i = firstIndex; i < lastIndex; ++i)
			{
				for(PxU32 j = i + 1; j <= lastIndex; ++j)
				{
					const PxU32 vi0 = vertexIndicesAndTriangleIndexPair[i].z;
					const PxU32 vi1 = vertexIndicesAndTriangleIndexPair[j].z;

					const PxU32 ti0 = vertexIndicesAndTriangleIndexPair[i].w;
					const PxU32 ti1 = vertexIndicesAndTriangleIndexPair[j].w;

					// This case should not happen: bending within the same triangle. However, in case triangle duplicates are not removed,
					// extra branching is used.
					if (vi0 == vi1)
						continue;

					trianglePairVertexIndices.pushBack(uint4{ vi0, vi1, vi2, vi3 });
					trianglePairTriangleIndices.pushBack(uint2{ ti0, ti1 });
				}

				innerTriangles.pushBack(vertexIndicesAndTriangleIndexPair[i].w);
			}

			numBendPairs = 0;

			if(isLastIter && isNewEdge) // Boundary edge
			{
				const PxU32 tempVi2 = vertexIndicesAndTriangleIndexPair[e].x;
				const PxU32 tempVi3 = vertexIndicesAndTriangleIndexPair[e].y;
				const PxU32 tempOwningTriangle = vertexIndicesAndTriangleIndexPair[e].w;

				++numEdgesPerVertex[tempVi2];
				++numEdgesPerVertex[tempVi3];
				++borderEdgeCount;
				borderTriangles.pushBack(tempOwningTriangle);
			}
		}
		else if (isNewEdge)
		{
			// An edge can belong to multiple triangles, potentially resulting in the same edge being visited multiple times during triangle iteration.
			// By assigning each edge to one triangle, we ensure that each edge is processed only once during iteration.
			// Here, mark the ownership of outer (border) edges within the triangles.

			for(PxU32 de = 0; de < (isLastIter ? 2u : 1u); ++de)
			{
				const PxU32 newE = prevE + de;
				const PxU32 vi2 = vertexIndicesAndTriangleIndexPair[newE].x;
				const PxU32 vi3 = vertexIndicesAndTriangleIndexPair[newE].y;
				const PxU32 owningTriangle = vertexIndicesAndTriangleIndexPair[newE].w;

				++numEdgesPerVertex[vi2];
				++numEdgesPerVertex[vi3];
				++borderEdgeCount;
				borderTriangles.pushBack(owningTriangle);
			}

			numBendPairs = 0;
		}
	}

	PxU32 maxEdgesPerVertex = 0;
	for (PxU32 v = 0; v < nbVertices; ++v)
	{
		maxEdgesPerVertex = PxMax(numEdgesPerVertex[v], maxEdgesPerVertex);
	}

	// Compute the edge ownership per triangle such that the number of triangles having any edges assigned is minimized.
	PxArray<bool> triangleUsed;
	computeEdgeOwnership(triangleUsed, edgeOwnershipNibbles, triangleVertexIndices.begin(), nbVertices, nbTriangles, innerTriangles, borderTriangles,
						 maxEdgesPerVertex);
}

PxgDeformableSurfaceTopology::PxgDeformableSurfaceTopology(const Gu::TriangleMesh& triangleMesh)
{
	computeTriangleTopology(triangleMesh, mEdgeOwnershipNibbles, mTrianglePairTriangleIndices, mTrianglePairVertexIndices);
}

void PxgDeformableSurfaceTopology::releaseUploadData()
{
	mTrianglePairTriangleIndices.reset();
	mTrianglePairVertexIndices.reset();
}

PxU32 PxgFEMClothUtil::initialTriangleData(PxgFEMCloth& femCloth, const Gu::TriangleMesh* triangleMesh,
										   const PxgDeformableSurfaceTopology& topology,
										   const PxU16* materialHandles, PxsDeformableSurfaceMaterialData* materials,
										   const PxU32 nbMaterials, Cm::VirtualAllocatorCallback& hostAlloc)
{
	const PxU32 nbTriangles = triangleMesh->getNbTrianglesFast();
	const PxU32 nbVertices = triangleMesh->getNbVerticesFast();

	const PxU16* materialIndices = triangleMesh->getMaterials();
	PxU16* destMaterialIndices = femCloth.mMaterialIndices;
	float* destDynamicFrictions = femCloth.mDynamicFrictions;

	const PxU32* vertAccumulatedTriangleRefs = triangleMesh->getAccumulatedTriangleRef();
	const PxU32 nbTriangleReferences = triangleMesh->getNbTriangleReferences();

	PxMemZero(destDynamicFrictions, sizeof(float) * nbVertices);

	uint4* triangleVertexIndices_ownedEdges = femCloth.mTriangleVertexIndices_ownedEdges;

	const PxU32* gpuToCpuFaceRemap = triangleMesh->mGRB_faceRemap;
	if(triangleMesh->has16BitIndices())
	{
		const PxU16* const triangleInds = reinterpret_cast<PxU16*>(triangleMesh->mGRB_triIndices);

		for(PxU32 i = 0; i < nbTriangles; ++i)
		{
			const PxU16 vInd0 = triangleInds[3 * i + 0];
			const PxU16 vInd1 = triangleInds[3 * i + 1];
			const PxU16 vInd2 = triangleInds[3 * i + 2];

			triangleVertexIndices_ownedEdges[i] = make_uint4(vInd0, vInd1, vInd2, topology.getEdgeOwnershipBits(i));

			initialMaterialData(materialIndices, materialHandles, materials, gpuToCpuFaceRemap, i, vInd0, vInd1, vInd2, nbMaterials,
								destMaterialIndices, destDynamicFrictions);
		}
	}
	else
	{
		const PxU32* const triangleInds = reinterpret_cast<PxU32*>(triangleMesh->mGRB_triIndices);

		for(PxU32 i = 0; i < nbTriangles; ++i)
		{
			const PxU32 vInd0 = triangleInds[3 * i + 0];
			const PxU32 vInd1 = triangleInds[3 * i + 1];
			const PxU32 vInd2 = triangleInds[3 * i + 2];

			triangleVertexIndices_ownedEdges[i] = make_uint4(vInd0, vInd1, vInd2, topology.getEdgeOwnershipBits(i));

			initialMaterialData(materialIndices, materialHandles, materials, gpuToCpuFaceRemap, i, vInd0, vInd1, vInd2, nbMaterials,
								destMaterialIndices, destDynamicFrictions);
		}
	}

	for(PxU32 i = 0; i < nbVertices; ++i)
	{
		PxU32 nbRefs = 0;
		if((i + 1) < nbVertices)
			nbRefs = vertAccumulatedTriangleRefs[i + 1] - vertAccumulatedTriangleRefs[i];
		else
			nbRefs = nbTriangleReferences - vertAccumulatedTriangleRefs[i];

		destDynamicFrictions[i] /= nbRefs;
	}

	// Create compacted list of triangles that at least own one edge.
	PxArray<PxU32> trianglesWithOwnedEdges;
	trianglesWithOwnedEdges.reserve(nbTriangles);

	for(PxU32 t = 0; t < nbTriangles; ++t)
	{
		if(topology.getEdgeOwnershipBits(t))
			trianglesWithOwnedEdges.pushBack(t);
	}

	const PxU32 elementIndex = topology.getNbTrianglePairs();

	femCloth.mNbVerts = nbVertices;
	femCloth.mNbTriangles = nbTriangles;
	femCloth.mNbTrianglePairs = elementIndex;

	femCloth.mNbTrianglesWithOwnedEdges = trianglesWithOwnedEdges.size();

	femCloth.mTrianglesWithOwnedEdges = reinterpret_cast<PxU32*>(
		hostAlloc.allocate(sizeof(PxU32) * trianglesWithOwnedEdges.size(), PxsHeapStats::eSIMULATION_FEMCLOTH, PX_FL));

	PxMemCopy(femCloth.mTrianglesWithOwnedEdges, &trianglesWithOwnedEdges[0], sizeof(PxU32) * trianglesWithOwnedEdges.size());

	return elementIndex;
}

void PxgFEMClothUtil::categorizeClothConstraints(PxArray<PxU32>& sharedTrianglePairs, PxArray<PxU32>& nonSharedTriangles,
												 PxArray<PxU32>& nonSharedTrianglePairs, PxgFEMCloth& femCloth,
												 const PxArray<uint2>& trianglePairTriangleIndices)
{
	const PxU32 numTriangles = femCloth.mNbTriangles;
	const PxU32 numTrianglePairs = trianglePairTriangleIndices.size();

	PxArray<bool> triangleVisited(numTriangles, false);

	// Shared structure to use both triangle constraints and bending constraints
	sharedTrianglePairs.reserve(numTrianglePairs);

	// Non-shared structure only for in-plane triangle constraints and bending constraints
	nonSharedTriangles.reserve(numTriangles);

	// Non-shared structure only for bending constraints
	nonSharedTrianglePairs.reserve(numTrianglePairs);

	for(PxU32 i = 0; i < numTrianglePairs; ++i)
	{
		const PxU32 t0 = trianglePairTriangleIndices[i].x;
		const PxU32 t1 = trianglePairTriangleIndices[i].y;

		if(!triangleVisited[t0] && !triangleVisited[t1])
		{
			triangleVisited[t0] = true;
			triangleVisited[t1] = true;

			sharedTrianglePairs.pushBack(i);
		}
		else
		{
			nonSharedTrianglePairs.pushBack(i);
		}
	}

	for(PxU32 i = 0; i < numTriangles; ++i)
	{
		if(!triangleVisited[i])
		{
			nonSharedTriangles.pushBack(i);
		}
	}

	sharedTrianglePairs.shrink();
	nonSharedTrianglePairs.shrink();
	nonSharedTriangles.shrink();
}

static void initialMaterialData(const PxU16* materialIndices, const PxU16* materialHandles, const PxsDeformableSurfaceMaterialData* materials,
								const PxU32* gpuToCpuFaceRemap, const PxU32 triangleIndex, PxU32 vi0, PxU32 vi1, PxU32 vi2,
								const PxU32 nbMaterials, PxU16* destMaterialIndices, float* destDynamicFrictions)
{
	PX_UNUSED(nbMaterials);
	if(materialIndices)
	{
		const PxU16 localMaterialIndex = materialIndices[gpuToCpuFaceRemap[triangleIndex]];
		const PxU16 globalMaterialIndex = materialHandles[localMaterialIndex];
		PX_ASSERT(localMaterialIndex < nbMaterials);
		destMaterialIndices[triangleIndex] = globalMaterialIndex;
		const float dynamicFriction = materials[globalMaterialIndex].dynamicFriction;
		destDynamicFrictions[vi0] += dynamicFriction;
		destDynamicFrictions[vi1] += dynamicFriction;
		destDynamicFrictions[vi2] += dynamicFriction;
	}
	else
	{
		const PxU16 globalMaterialIndex = materialHandles[0];
		const float dynamicFriction = materials[globalMaterialIndex].dynamicFriction;

		destMaterialIndices[triangleIndex] = globalMaterialIndex;
		destDynamicFrictions[vi0] += dynamicFriction;
		destDynamicFrictions[vi1] += dynamicFriction;
		destDynamicFrictions[vi2] += dynamicFriction;
	}
}


/*******************************************************************************
 *
 *
 * For per-triangle energies
 *
 *
 ******************************************************************************/

void PxgFEMClothUtil::computeNonSharedTriangleConfiguration(PxgFEMCloth& femCloth, const PxArray<PxU32>& orderedNonSharedTriangles,
															const PxArray<PxU32>& activeTriangleIndices,
															const Gu::TriangleMesh* const triangleMesh)
{
	const PxVec3* positions = triangleMesh->getVerticesFast();
	float4* orderedTriangleRestPoseInv = femCloth.mOrderedNonSharedTriangleRestPoseInv;

	for (PxU32 it = 0; it < activeTriangleIndices.size(); ++it)
	{
		// Ordered indices
		const PxU32 index = activeTriangleIndices[orderedNonSharedTriangles[it]];
		PX_ASSERT(index < femCloth.mNbTriangles);

		const uint4& triangleVertexIndex = femCloth.mTriangleVertexIndices_ownedEdges[index];
		const PxU32 vi0 = triangleVertexIndex.x;
		const PxU32 vi1 = triangleVertexIndex.y;
		const PxU32 vi2 = triangleVertexIndex.z;

		femCloth.mOrderedNonSharedTriangleVertexIndices_triIndex[it] = triangleVertexIndex;
		femCloth.mOrderedNonSharedTriangleVertexIndices_triIndex[it].w = index;
		queryRestConfigurationPerTriangle(orderedTriangleRestPoseInv[it], vi0, vi1, vi2, positions);
	}
}

// Neo-hookean, St VK, Corot, etc
void queryRestConfigurationPerTriangle(float4& orderedTriangleRestPoseInv, PxU32 vi0, PxU32 vi1, PxU32 vi2, const PxVec3* const positions)
{
	const PxVec3& x0 = positions[vi0];
	const PxVec3& x1 = positions[vi1];
	const PxVec3& x2 = positions[vi2];

	const PxVec3 x01 = x1 - x0;
	const PxVec3 x02 = x2 - x0;

	const PxVec3 axis0 = x01.getNormalized();
	const PxVec3 n = x01.cross(x02);
	const PxVec3 axis1 = n.cross(axis0).getNormalized();

	PxVec2 u01(axis0.dot(x01), axis1.dot(x01));
	PxVec2 u02(axis0.dot(x02), axis1.dot(x02));

	const PxReal det = u01[0] * u02[1] - u02[0] * u01[1];
	PX_ASSERT(det != 0.0f);

	const PxReal detUInv = (det == 0.0f) ? 1.0f : 1.0f / det;
	orderedTriangleRestPoseInv.x = u02[1] * detUInv;
	orderedTriangleRestPoseInv.y = -u01[1] * detUInv;
	orderedTriangleRestPoseInv.z = -u02[0] * detUInv;
	orderedTriangleRestPoseInv.w = u01[0] * detUInv;
}

/*******************************************************************************
 *
 *
 * For per-triangle-pair energies
 *
 *
 ******************************************************************************/

float PxgFEMClothUtil::updateFlexuralStiffnessPerTrianglePair(float t0Area, float t1Area, float hingeLength, float thickness,
															  float inputStiffness)
{
#if 0 // Original formulation to compute bending stiffness using Young's modulus, Poisson's ratio, and thickness
	const float y = (material0.youngs + material1.youngs) * 0.5f;
	const float t = (material0.thickness + material1.thickness) * 0.5f;
	const float p = (material0.poissons + material1.poissons) * 0.5f;

	const float dualLength = 2.0f * (t0Area + t1Area) / (3.0f * hingeLength);
	return hingeLength / dualLength * y * t * t * t / (6.0f * (1.0f - p * p));
#endif

	// Use user-input bending stiffness
	const float dualLength = 2.0f * (t0Area + t1Area) / (3.0f * hingeLength);

	return (inputStiffness * thickness * thickness * thickness) * (hingeLength / dualLength);
}

bool PxgFEMClothUtil::updateRestConfiguration(float4* orderedRestAngleAndStiffness_damping, uint4* orderedTrianglePairVertexIndices,
											  PxgFEMCloth& femCloth, PxU32 it, PxU32 index, PxArray<uint2>& trianglePairTriangleIndices,
											  const PxArray<uint4>& trianglePairVertexIndices,
											  const PxsDeformableSurfaceMaterialData* materials, const PxVec3* positions,
											  bool zeroRestBendingAngle, float4* orderedRestEdge0_edge1,
											  float4* orderedRestEdgeLength_material0_material1)
{
	const uint vi0 = trianglePairVertexIndices[index].x;
	const uint vi1 = trianglePairVertexIndices[index].y;
	const uint vi2 = trianglePairVertexIndices[index].z;
	const uint vi3 = trianglePairVertexIndices[index].w;

	orderedTrianglePairVertexIndices[it] = uint4{ vi0, vi1, vi2, vi3 };
	const uint ti0 = trianglePairTriangleIndices[index].x;
	const uint ti1 = trianglePairTriangleIndices[index].y;

	const PxU32 globalMaterialIndex0 = femCloth.mMaterialIndices[ti0];
	const PxU32 globalMaterialIndex1 = femCloth.mMaterialIndices[ti1];

	const PxsDeformableSurfaceMaterialData& material0 = materials[globalMaterialIndex0];
	const PxsDeformableSurfaceMaterialData& material1 = materials[globalMaterialIndex1];

	const PxVec3& x0 = positions[vi0];
	const PxVec3& x1 = positions[vi1];
	const PxVec3& x2 = positions[vi2];
	const PxVec3& x3 = positions[vi3];

	const PxVec3 x02 = x2 - x0;
	const PxVec3 x03 = x3 - x0;
	const PxVec3 x13 = x3 - x1;
	const PxVec3 x12 = x2 - x1;
	const PxVec3 x23 = x3 - x2;
	const float x23Norm = x23.magnitude();

	// The shared axis (x23) that belongs to both triangles (triangle0, triangle1).
	const PxVec3 sharedAxis = x23 / x23Norm;

	// Set up bending configuration
	float restBendingAngle = 0.f;
	bool hasActiveBending = false;

	if (!zeroRestBendingAngle)
	{
		const PxVec3 n0 = x02.cross(x03).getNormalized();
		const PxVec3 n1 = x13.cross(x12).getNormalized();

		const float cosAngle = n0.dot(n1);
		const float sinAngle = n0.cross(n1).dot(sharedAxis);
		restBendingAngle = (float)atan2((double)sinAngle, (double)cosAngle);
	}

	// User-input stiffness not considering the geometry of the shell (e.g., thickness, area, etc.)
	const float inputBendingStiffness = 0.5f * (material0.bendingStiffness + material1.bendingStiffness);
	const float inputBendingDamping = 0.5f * (material0.bendingDamping + material1.bendingDamping);
	const float thickness = 0.5f * (material0.thickness + material1.thickness);

	orderedRestAngleAndStiffness_damping[it].x = restBendingAngle;
	orderedRestAngleAndStiffness_damping[it].z = inputBendingDamping;
	if (inputBendingStiffness > 0.f)
	{
		const float edgeLen02 = x02.magnitude();
		const float edgeLen03 = x03.magnitude();
		const float edgeLen12 = x12.magnitude();
		const float edgeLen13 = x13.magnitude();

		const float s0 = 0.5f * (x23Norm + edgeLen02 + edgeLen03);
		const float area0 = sqrtf(s0 * (s0 - x23Norm) * (s0 - edgeLen02) * (s0 - edgeLen03));

		const float s1 = 0.5f * (x23Norm + edgeLen12 + edgeLen13);
		const float area1 = sqrtf(s1 * (s1 - x23Norm) * (s1 - edgeLen12) * (s1 - edgeLen13));

		orderedRestAngleAndStiffness_damping[it].y =
			1.f / updateFlexuralStiffnessPerTrianglePair(area0, area1, x23Norm, thickness, inputBendingStiffness);
		hasActiveBending = true;
	}
	else
	{
		orderedRestAngleAndStiffness_damping[it].y = -1.f;
	}

	// Set up in-plane configuration
	if (orderedRestEdge0_edge1 && orderedRestEdgeLength_material0_material1)
	{
		// The other axis (x20) that spans the plane of triangle0, along with the shared axis.
		const PxVec3 tn0 = sharedAxis.cross(-x02);
		const PxVec3 axis0 = tn0.cross(sharedAxis).getNormalized();

		// The other axis (x21) that spans the plane of triangle1, along with the shared axis.
		const PxVec3 tn1 = sharedAxis.cross(-x12);
		const PxVec3 axis1 = tn1.cross(sharedAxis).getNormalized();

		// Adding edge vectors in the rest configuration (x20, x21)
		orderedRestEdge0_edge1[it] = make_float4(sharedAxis.dot(-x02), axis0.dot(-x02), sharedAxis.dot(-x12), axis1.dot(-x12));
		orderedRestEdgeLength_material0_material1[it] = make_float4(x23Norm, static_cast<PxReal>(globalMaterialIndex0), static_cast<PxReal>(globalMaterialIndex1), 0.0f);
	}

	return hasActiveBending;
}

void PxgFEMClothUtil::computeTrianglePairConfiguration(
	PxgFEMCloth& femCloth, PxArray<uint2>& trianglePairTriangleIndices, const PxArray<uint4>& trianglePairVertexIndices,
	const PxArray<PxU32>& orderedTrianglePairs, const PxArray<PxU32>& activeTrianglePairIndices, const Gu::TriangleMesh* const triangleMesh,
	const PxsDeformableSurfaceMaterialData* materials, bool zeroRestBendingAngle, bool isSharedPartition)
{
	const PxVec3* positions = triangleMesh->getVerticesFast();
	float4* orderedRestAngleAndStiffness_damping;
	uint4* orderedTrianglePairVertexIndices;

	float4* orderedSharedRestEdge0_edge1 = NULL;
	float4* orderedSharedRestEdgeLength_material0_material1 = NULL;
	bool hasActiveBending = false;

	if (isSharedPartition)
	{
		orderedRestAngleAndStiffness_damping = femCloth.mOrderedSharedRestBendingAngle_flexuralStiffness_damping;
		orderedTrianglePairVertexIndices = femCloth.mOrderedSharedTrianglePairVertexIndices;
		orderedSharedRestEdge0_edge1 = femCloth.mOrderedSharedRestEdge0_edge1;
		orderedSharedRestEdgeLength_material0_material1 = femCloth.mOrderedSharedRestEdgeLength_material0_material1;
	}
	else
	{
		orderedRestAngleAndStiffness_damping = femCloth.mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping;
		orderedTrianglePairVertexIndices = femCloth.mOrderedNonSharedTrianglePairVertexIndices;
	}

	for(PxU32 it = 0; it < activeTrianglePairIndices.size(); ++it)
	{
		// Ordered indices
		PxU32 index = activeTrianglePairIndices[orderedTrianglePairs[it]];

		PX_ASSERT(index < femCloth.mNbTrianglePairs);

		hasActiveBending |=
			updateRestConfiguration(orderedRestAngleAndStiffness_damping, orderedTrianglePairVertexIndices, femCloth, it, index,
									trianglePairTriangleIndices, trianglePairVertexIndices, materials, positions, zeroRestBendingAngle,
									orderedSharedRestEdge0_edge1, orderedSharedRestEdgeLength_material0_material1);
	}

	if (!isSharedPartition)
	{
		femCloth.mNonSharedTriPair_hasActiveBending = hasActiveBending;
	}
}

/*******************************************************************************
 *
 *
 * Others
 *
 *
 ******************************************************************************/

void queryVertexIndicesAndTriangleIndexPairs(PxArray<uint4>& vertexIndicesAndTriangleIndexPair, PxU32 vi0, PxU32 vi1, PxU32 vi2, PxU32 ti)
{
	// edge0
	if(vi0 < vi1)
	{
		vertexIndicesAndTriangleIndexPair[3 * ti] = uint4{ vi0, vi1, vi2, ti };
	}
	else
	{
		vertexIndicesAndTriangleIndexPair[3 * ti] = uint4{ vi1, vi0, vi2, ti };
	}
	// edge1
	if(vi1 < vi2)
	{
		vertexIndicesAndTriangleIndexPair[3 * ti + 1] = uint4{ vi1, vi2, vi0, ti };
	}
	else
	{
		vertexIndicesAndTriangleIndexPair[3 * ti + 1] = uint4{ vi2, vi1, vi0, ti };
	}
	// edge2
	if(vi2 < vi0)
	{
		vertexIndicesAndTriangleIndexPair[3 * ti + 2] = uint4{ vi2, vi0, vi1, ti };
	}
	else
	{
		vertexIndicesAndTriangleIndexPair[3 * ti + 2] = uint4{ vi0, vi2, vi1, ti };
	}
}

void computeEdgeOwnership(PxArray<bool>& triangleUsed, PxArray<PxU8>& edgeOwnershipNibbles, const uint4* triangleVertexIndices,
						  PxU32 nbVertices, PxU32 nbTriangles, const PxArray<PxU32>& innerTriangles,
						  const PxArray<PxU32>& borderTriangles, PxU32 maxEdgesPerVertex)
{
	triangleUsed.clear();
	triangleUsed.resize(nbTriangles, false);

	// Two triangles to a byte, three bits to a nibble. Zeroed because a triangle owning no edge is never encoded.
	edgeOwnershipNibbles.clear();
	edgeOwnershipNibbles.resize((nbTriangles + 1) / 2, 0);

	PxArray<PxArray<PxU32> > edgeList(nbVertices);
	PxArray<PxU32> edgeCount(nbVertices, 0);

	for(PxU32 i = 0; i < nbVertices; ++i)
		edgeList[i].reserve(maxEdgesPerVertex);

	// Process border triangles first
	for(PxU32 i = 0; i < borderTriangles.size(); ++i)
	{
		const PxU32 triIdx = borderTriangles[i];
		if(triangleUsed[triIdx])
			continue;

		const uint4& tri = triangleVertexIndices[triIdx];
		bool edge0 = true, edge1 = true, edge2 = true;

		// Check edge0
		PxU32 a = tri.x < tri.y ? tri.x : tri.y;
		PxU32 b = tri.x < tri.y ? tri.y : tri.x;
		for(PxU32 j = 0; j < edgeCount[a]; ++j)
		{
			if(edgeList[a][j] == b)
			{
				edge0 = false;
				break;
			}
		}

		// Check edge1
		a = tri.y < tri.z ? tri.y : tri.z;
		b = tri.y < tri.z ? tri.z : tri.y;
		for(PxU32 j = 0; j < edgeCount[a]; ++j)
		{
			if(edgeList[a][j] == b)
			{
				edge1 = false;
				break;
			}
		}

		// Check edge2
		a = tri.z < tri.x ? tri.z : tri.x;
		b = tri.z < tri.x ? tri.x : tri.z;
		for(PxU32 j = 0; j < edgeCount[a]; ++j)
		{
			if(edgeList[a][j] == b)
			{
				edge2 = false;
				break;
			}
		}

		if(edge0 || edge1 || edge2)
		{
			triangleUsed[triIdx] = true;

			if(edge0)
			{
				PxU32 vMin = tri.x < tri.y ? tri.x : tri.y;
				PxU32 vMax = tri.x < tri.y ? tri.y : tri.x;
				edgeList[vMin].pushBack(vMax);
				++edgeCount[vMin];
			}

			if(edge1)
			{
				PxU32 vMin = tri.y < tri.z ? tri.y : tri.z;
				PxU32 vMax = tri.y < tri.z ? tri.z : tri.y;
				edgeList[vMin].pushBack(vMax);
				++edgeCount[vMin];
			}

			if(edge2)
			{
				PxU32 vMin = tri.z < tri.x ? tri.z : tri.x;
				PxU32 vMax = tri.z < tri.x ? tri.x : tri.z;
				edgeList[vMin].pushBack(vMax);
				++edgeCount[vMin];
			}

			encodeEdgeOwnership(edgeOwnershipNibbles.begin(), triIdx, edge0, edge1, edge2);
		}
	}

	for(PxU32 gainLevel = 3; gainLevel >= 1; --gainLevel)
	{
		for(PxU32 i = 0; i < innerTriangles.size(); ++i)
		{
			const PxU32 triIdx = innerTriangles[i];
			if(triangleUsed[triIdx])
				continue;

			const uint4& tri = triangleVertexIndices[triIdx];
			bool edge0 = true, edge1 = true, edge2 = true;
			PxU32 gain = 0;

			// Check edge0
			PxU32 a = tri.x < tri.y ? tri.x : tri.y;
			PxU32 b = tri.x < tri.y ? tri.y : tri.x;
			for(PxU32 j = 0; j < edgeCount[a]; ++j)
				if(edgeList[a][j] == b)
					edge0 = false;
			if(edge0)
				++gain;

			// Check edge1
			a = tri.y < tri.z ? tri.y : tri.z;
			b = tri.y < tri.z ? tri.z : tri.y;
			for(PxU32 j = 0; j < edgeCount[a]; ++j)
				if(edgeList[a][j] == b)
					edge1 = false;
			if(edge1)
				++gain;

			// Check edge2
			a = tri.z < tri.x ? tri.z : tri.x;
			b = tri.z < tri.x ? tri.x : tri.z;
			for(PxU32 j = 0; j < edgeCount[a]; ++j)
				if(edgeList[a][j] == b)
					edge2 = false;
			if(edge2)
				++gain;

			if(gain >= gainLevel)
			{
				triangleUsed[triIdx] = true;

				if(edge0)
				{
					PxU32 vMin = tri.x < tri.y ? tri.x : tri.y;
					PxU32 vMax = tri.x < tri.y ? tri.y : tri.x;
					edgeList[vMin].pushBack(vMax);
					++edgeCount[vMin];
				}

				if(edge1)
				{
					PxU32 vMin = tri.y < tri.z ? tri.y : tri.z;
					PxU32 vMax = tri.y < tri.z ? tri.z : tri.y;
					edgeList[vMin].pushBack(vMax);
					++edgeCount[vMin];
				}

				if(edge2)
				{
					PxU32 vMin = tri.z < tri.x ? tri.z : tri.x;
					PxU32 vMax = tri.z < tri.x ? tri.x : tri.z;
					edgeList[vMin].pushBack(vMax);
					++edgeCount[vMin];
				}

				encodeEdgeOwnership(edgeOwnershipNibbles.begin(), triIdx, edge0, edge1, edge2);
			}
		}
	}
}

// Store the three edge ownership bits for a given triangle index to the one nibble per triangle format
void encodeEdgeOwnership(PxU8* edgeOwnershipNibbles, PxU32 triIndex, bool ownsEdge0, bool ownsEdge1, bool ownsEdge2)
{
	const PxU32 bits = (ownsEdge0 ? 1u : 0u) | (ownsEdge1 ? 2u : 0u) | (ownsEdge2 ? 4u : 0u);
	const PxU32 shift = 4 * (triIndex & 1);

	PxU8& packed = edgeOwnershipNibbles[triIndex >> 1];
	packed = PxU8((packed & ~(0xFu << shift)) | (bits << shift));
}

void PxgFEMCloth::deallocate(Cm::VirtualAllocatorCallback& hostAlloc)
{
	if (mNbNonSharedTriangles)
	{
		hostAlloc.deallocate(mOrderedNonSharedTriangleVertexIndices_triIndex);
		hostAlloc.deallocate(mOrderedNonSharedTriangleRestPoseInv);
		hostAlloc.deallocate(mNonSharedTriAccumulatedPartitionsCP);
	}

	if (mNbSharedTrianglePairs)
	{
		hostAlloc.deallocate(mOrderedSharedTrianglePairVertexIndices);
		hostAlloc.deallocate(mOrderedSharedRestBendingAngle_flexuralStiffness_damping);
		hostAlloc.deallocate(mOrderedSharedRestEdge0_edge1);
		hostAlloc.deallocate(mOrderedSharedRestEdgeLength_material0_material1);

		hostAlloc.deallocate(mSharedTriPairRemapOutputCP);
		hostAlloc.deallocate(mSharedTriPairAccumulatedPartitionsCP);
		hostAlloc.deallocate(mSharedTriPairAccumulatedCopiesCP);
	}

	if (mNbNonSharedTrianglePairs)
	{
		hostAlloc.deallocate(mOrderedNonSharedTrianglePairVertexIndices);
		hostAlloc.deallocate(mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping);

		hostAlloc.deallocate(mNonSharedTriPairRemapOutputCP);
		hostAlloc.deallocate(mNonSharedTriPairAccumulatedPartitionsCP);
		hostAlloc.deallocate(mNonSharedTriPairAccumulatedCopiesCP);
	}

	hostAlloc.deallocate(mTriMeshData);
	hostAlloc.deallocate(mTrianglesWithOwnedEdges);
	hostAlloc.deallocate(mTriangleVertexIndices_ownedEdges);

	hostAlloc.deallocate(mMaterialIndices);
	hostAlloc.deallocate(mDynamicFrictions);
}

} // namespace physx
