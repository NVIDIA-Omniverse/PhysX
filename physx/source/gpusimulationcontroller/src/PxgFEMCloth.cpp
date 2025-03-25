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

#include "PxgFEMCloth.h"
#include "GuTriangleMesh.h"
#include "GuInternal.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "cutil_math.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include <stdio.h>

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

PxU32 PxgFEMClothUtil::initialTriangleData(PxgFEMCloth& femCloth, PxArray<uint2>& trianglePairTriangleIndices,
										   PxArray<uint4>& trianglePairVertexIndices, const Gu::TriangleMesh* triangleMesh,
										   const PxU16* materialHandles, PxsDeformableSurfaceMaterialData* materials, const PxU32 nbMaterials)
{
	PX_UNUSED(nbMaterials);
	const PxU32 nbTriangles = triangleMesh->getNbTrianglesFast();
	const PxU32 nbVertices = triangleMesh->getNbVerticesFast();

	const PxU16* materialIndices = triangleMesh->getMaterials();
	PxU16* destMaterialIndices = femCloth.mMaterialIndices;
	float* destDynamicFrictions = femCloth.mDynamicFrictions;

	const PxU32* vertAccumulatedTriangleRefs = triangleMesh->getAccumulatedTriangleRef();
	const PxU32 nbTriangleReferences = triangleMesh->getNbTriangleReferences();

	PxMemZero(destDynamicFrictions, sizeof(float) * nbVertices);

	uint4* triangleIndices = femCloth.mTriangleVertexIndices;

	// copy triangle indices and query triangle pairs
	PxArray<uint4> vertexIndicesAndTriangleIndexPair;
	const PxU32 nbConservativeTrianglePairs = 3 * nbTriangles;
	vertexIndicesAndTriangleIndexPair.reserve(nbConservativeTrianglePairs);
	vertexIndicesAndTriangleIndexPair.forceSize_Unsafe(nbConservativeTrianglePairs);
	trianglePairTriangleIndices.reserve(nbConservativeTrianglePairs);
	trianglePairVertexIndices.reserve(nbConservativeTrianglePairs);

	const PxU32* gpuToCpuFaceRemap = triangleMesh->mGRB_faceRemap;

	if(triangleMesh->has16BitIndices())
	{
		const PxU16* const triangleInds = reinterpret_cast<PxU16*>(triangleMesh->mGRB_triIndices);

		for(PxU32 i = 0; i < nbTriangles; ++i)
		{
			const PxU16 vInd0 = triangleInds[3 * i + 0];
			const PxU16 vInd1 = triangleInds[3 * i + 1];
			const PxU16 vInd2 = triangleInds[3 * i + 2];

			triangleIndices[i].x = vInd0;
			triangleIndices[i].y = vInd1;
			triangleIndices[i].z = vInd2;

			initialMaterialData(materialIndices, materialHandles, materials, gpuToCpuFaceRemap, i, vInd0, vInd1, vInd2, nbMaterials,
								destMaterialIndices, destDynamicFrictions);
			queryVertexIndicesAndTriangleIndexPairs(vertexIndicesAndTriangleIndexPair, triangleIndices[i].x, triangleIndices[i].y,
													triangleIndices[i].z, i);
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

			triangleIndices[i].x = vInd0;
			triangleIndices[i].y = vInd1;
			triangleIndices[i].z = vInd2;

			initialMaterialData(materialIndices, materialHandles, materials, gpuToCpuFaceRemap, i, vInd0, vInd1, vInd2,
			                    nbMaterials, destMaterialIndices, destDynamicFrictions);
			queryVertexIndicesAndTriangleIndexPairs(vertexIndicesAndTriangleIndexPair, triangleIndices[i].x,
			                                        triangleIndices[i].y, triangleIndices[i].z, i);
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

	PxU32 elementIndex = 0;
	PxU32 numEdgePairs = 0;
	bool queryEdgePairs = false;

	PxArray<PxPair<PxU32, PxU32>> length_elementId;
	length_elementId.reserve(nbConservativeTrianglePairs);

	for(PxU32 e = 1; e < vertexIndicesAndTriangleIndexPair.size(); ++e)
	{
		PxU32 prevE = e - 1;

		if(vertexIndicesAndTriangleIndexPair[prevE].x == vertexIndicesAndTriangleIndexPair[e].x &&
		   vertexIndicesAndTriangleIndexPair[prevE].y == vertexIndicesAndTriangleIndexPair[e].y)
		{
			queryEdgePairs = false;
			++numEdgePairs;
		}
		else
		{
			if(numEdgePairs)
			{
				queryEdgePairs = true;
			}
			else
			{
				queryEdgePairs = false;
			}
		}

		// handling non-manifold cases
		if(queryEdgePairs || (e == vertexIndicesAndTriangleIndexPair.size() - 1 && numEdgePairs > 0))
		{
			PxU32 firstIndex = queryEdgePairs ? e - numEdgePairs - 1 : e - numEdgePairs;
			PxU32 lastIndex = queryEdgePairs ? e - 1 : e;
			for(PxU32 i = firstIndex; i < lastIndex; ++i)
			{
				for(PxU32 j = i + 1; j <= lastIndex; ++j)
				{
					const uint vi0 = vertexIndicesAndTriangleIndexPair[i].z;
					const uint vi1 = vertexIndicesAndTriangleIndexPair[j].z;
					const uint vi2 = vertexIndicesAndTriangleIndexPair[j].x;
					const uint vi3 = vertexIndicesAndTriangleIndexPair[j].y;

					uint ti0 = vertexIndicesAndTriangleIndexPair[i].w;
					uint ti1 = vertexIndicesAndTriangleIndexPair[j].w;

					const float sharedEdgeLength = (triangleMesh->getVerticesFast()[vi2] - triangleMesh->getVerticesFast()[vi3]).magnitudeSquared();
					const PxU32 lengthPredicate = *reinterpret_cast<const PxU32*>(&sharedEdgeLength);

					// This case should not happen: bending within the same triangle. However, in case triangle duplicates are not removed,
					// extra branching is used.
					if (vi0 == vi1)
						continue;

					trianglePairVertexIndices.pushBack(uint4{ vi0, vi1, vi2, vi3 });
					trianglePairTriangleIndices.pushBack(uint2{ ti0, ti1});
					length_elementId.pushBack(PxPair<PxU32, PxU32>(lengthPredicate, elementIndex));
					++elementIndex;
				}
			}

			queryEdgePairs = false;
			numEdgePairs = 0;
		}
	}

	femCloth.mNbVerts = nbVertices;
	femCloth.mNbTriangles = nbTriangles;
	femCloth.mNbTrianglePairs = elementIndex;

#if 0 // Sort triangle-pair again based-on shared edge length.
	PxSort(length_elementId.begin(), length_elementId.size());

	PxArray<uint4> tempVertexIndices(trianglePairVertexIndices);
	PxArray<uint2> tempTriangleIndices(trianglePairTriangleIndices);

	// Saving them in descending order.
	for (PxU32 i = 0; i < length_elementId.size(); ++i)
	{
		const PxU32 elementId = length_elementId[length_elementId.size() - 1 - i].second;
		trianglePairVertexIndices[i] = tempVertexIndices[elementId];
		trianglePairTriangleIndices[i] = tempTriangleIndices[elementId];
	}
#else
	PX_UNUSED(length_elementId);
#endif

	return elementIndex;
}

void PxgFEMClothUtil::categorizeClothConstraints(PxArray<PxU32>& sharedTrianglePairs, PxArray<PxU32>& nonSharedTriangles,
												 PxArray<PxU32>& nonSharedTrianglePairs, PxgFEMCloth& femCloth,
												 const PxArray<uint2>& trianglePairTriangleIndices,
												 const PxArray<uint4>& trianglePairVertexIndices)
{
	PX_UNUSED(trianglePairVertexIndices);
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

	//printf("DEBUG PRINTING nonSharedTriangles: %i / %i \n", nonSharedTriangles.size(), numTriangles);
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

		const uint4& triangleVertexIndex = femCloth.mTriangleVertexIndices[index];
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

void PxgFEMCloth::deallocate(PxsHeapMemoryAllocator *allocator)
{
	if (mNbNonSharedTriangles)
	{
		allocator->deallocate(mOrderedNonSharedTriangleVertexIndices_triIndex);
		allocator->deallocate(mOrderedNonSharedTriangleRestPoseInv);
		allocator->deallocate(mNonSharedTriAccumulatedPartitionsCP);
	}

	if (mNbSharedTrianglePairs)
	{
		allocator->deallocate(mOrderedSharedTrianglePairVertexIndices);
		allocator->deallocate(mOrderedSharedRestBendingAngle_flexuralStiffness_damping);
		allocator->deallocate(mOrderedSharedRestEdge0_edge1);
		allocator->deallocate(mOrderedSharedRestEdgeLength_material0_material1);

		allocator->deallocate(mSharedTriPairRemapOutputCP);
		allocator->deallocate(mSharedTriPairAccumulatedPartitionsCP);
		allocator->deallocate(mSharedTriPairAccumulatedCopiesCP);
	}

	if (mNbNonSharedTrianglePairs)
	{
		allocator->deallocate(mOrderedNonSharedTrianglePairVertexIndices);
		allocator->deallocate(mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping);

		allocator->deallocate(mNonSharedTriPairRemapOutputCP);
		allocator->deallocate(mNonSharedTriPairAccumulatedPartitionsCP);
		allocator->deallocate(mNonSharedTriPairAccumulatedCopiesCP);
	}

	allocator->deallocate(mPrevPosition_InvMass);
	allocator->deallocate(mTriMeshData);
	allocator->deallocate(mTriangleVertexIndices);

	allocator->deallocate(mMaterialIndices);
	allocator->deallocate(mDynamicFrictions);
}

} // namespace physx
