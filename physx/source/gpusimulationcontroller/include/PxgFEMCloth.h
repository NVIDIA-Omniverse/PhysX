// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_FEMCLOTH_H
#define PXG_FEMCLOTH_H

#include "foundation/PxArray.h"
#include "foundation/PxBounds3.h"
#include <vector_types.h>

#include "foundation/PxUserAllocated.h"
#include "PxgSimulationCoreDesc.h" // for PxgDeformablePairInfo

namespace physx
{
namespace Gu
{
class TriangleMesh;
}
namespace Cm
{
class VirtualAllocatorCallback;
}

struct PxsDeformableSurfaceMaterialData;

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4324) // padding was added at the end of a structure because of a __declspec(align) value.
#endif

// The mesh topology of one deformable surface, derived when it joins a scene. Everything but the edge
// ownership is consumed by the upload and freed by releaseUploadData.
class PxgDeformableSurfaceTopology : public PxUserAllocated
{
  public:
	PxgDeformableSurfaceTopology(const Gu::TriangleMesh& triangleMesh);

	// Frees what the upload consumed, keeping the edge ownership.
	void releaseUploadData();

	// Whether the triangle is the one that tests this edge, which is what ownsEdge reads on the device.
	PX_FORCE_INLINE bool ownsEdge(PxU32 triangleIndex, PxU32 localEdge) const
	{
		return (getEdgeOwnershipBits(triangleIndex) & (1u << localEdge)) != 0;
	}

	// Unpacks the three edge ownership bits for a given triangle.
	PX_FORCE_INLINE PxU32 getEdgeOwnershipBits(PxU32 triangleIndex) const
	{
		return (mEdgeOwnershipNibbles[triangleIndex >> 1] >> (4 * (triangleIndex & 1))) & 7;
	}

	PX_FORCE_INLINE PxArray<uint2>& getTrianglePairTriangleIndices() { return mTrianglePairTriangleIndices; }
	PX_FORCE_INLINE PxArray<uint4>& getTrianglePairVertexIndices() { return mTrianglePairVertexIndices; }
	PX_FORCE_INLINE PxU32 getNbTrianglePairs() const
	{
		PX_ASSERT(mTrianglePairTriangleIndices.size() == mTrianglePairVertexIndices.size());
		return mTrianglePairTriangleIndices.size();
	}

  private:
	PxArray<uint2> mTrianglePairTriangleIndices;
	PxArray<uint4> mTrianglePairVertexIndices;

	// The edge ownership alone, one nibble per triangle, which is all the element filters need and all
	// that outlives the upload.
	PxArray<PxU8> mEdgeOwnershipNibbles;
};

struct PxgFEMClothData
{
  public:
	PxU32 mMaxNbNonSharedTrisPerPartition;

	PxU32 mSharedTriPairRemapOutputSize;
	PxU32 mNonSharedTriPairRemapOutputSize;

	PxU32 mMaxNbSharedTriPairsPerPartition;
	PxU32 mMaxNbNonSharedTriPairsPerPartition;

	PxU32 mNbPackedNodes;
};

PX_ALIGN_PREFIX(16)
class PxgFEMCloth
{
  public:

	// to deallocate the host mirror. Make sure you pass in the right allocator!
	void deallocate(Cm::VirtualAllocatorCallback& hostAlloc);

	void* mTriMeshData;

	float4* mVelocity_InvMass;
	float4* mPosition_InvMass;
	float4* mRestPosition;

	float4* mPrevPositionInContactOffset; // After contact pairs are updated, cloth vertices have moved by mPosition_InvMass -
										  // mPrevPositionInContactOffset.
	float4* mPrevPositionInRestOffset; // After previous "step()", cloth vertices moved from mPosition_InvMass to mPrevPositionInRestOffset.

	float* mDynamicFrictions; // dynamic friction per vertex
	PxU16* mMaterialIndices;

	PxU32* mTrianglesWithOwnedEdges;

	// The w component holds the three edge owning bits. See ownsEdge in deformableCollision.cuh.
	uint4* mTriangleVertexIndices_ownedEdges;
	uint4* mOrderedNonSharedTriangleVertexIndices_triIndex;
	float2* mOrderedSharedTriangleLambdas; // Two lambdas per triangle: ARAP, area
	float2* mOrderedNonSharedTriangleLambdas; // Two lambdas per triangle: ARAP, area
	float4* mOrderedNonSharedTriangleRestPoseInv; // Four components of restPoseInv: m00, m10, m01, m11

	// Shared triangle pair: A pair of triangles where both in-plane and bending constraints are applied together.
	// Non-shared triangle pair: A pair of triangles where only bending constraints are applied.

	uint4* mOrderedSharedTrianglePairVertexIndices;
	uint4* mOrderedNonSharedTrianglePairVertexIndices;

	float4* mOrderedSharedRestBendingAngle_flexuralStiffness_damping;
	float4* mOrderedNonSharedRestBendingAngle_flexuralStiffness_damping;

	float* mSharedBendingLambdas;
	float* mNonSharedBendingLambdas;

	bool mNonSharedTriPair_hasActiveBending;

	// To solve in-plane energies for shared triangle pairs, the rest pose of the two triangles is also stored.
	// Since the two triangles share an edge, only one additional edge needs to be stored for each triangle, which are edge0 and edge1.
	// By choosing the shared edge to be in the direction of (1, 0) in 2D, only the magnitude of the shared edge needs to be stored.
	// Consequently:
	// Rest edges for triangle 0: (RestEdgeLength, 0), (Edge0.x, Edge0.y)
	// Rest edges for triangle 1: (RestEdgeLength, 0), (Edge1.x, Edge1.y)
	// This approach saves significant memory compared to storing restPoseInv matrices for each triangle.

	float4* mOrderedSharedRestEdge0_edge1;
	float4* mOrderedSharedRestEdgeLength_material0_material1;

	float4* mPosition_InvMassCP; 
	
	PxU32* mNonSharedTriAccumulatedPartitionsCP;

	PxU32* mSharedTriPairRemapOutputCP;
	PxU32* mNonSharedTriPairRemapOutputCP;
	
	PxU32* mSharedTriPairAccumulatedCopiesCP;
	PxU32* mNonSharedTriPairAccumulatedCopiesCP;

	PxU32* mSharedTriPairAccumulatedPartitionsCP;
	PxU32* mNonSharedTriPairAccumulatedPartitionsCP;

	// Per-iteration solve scratch, the cloth analog of PxgSoftBody::mSimDelta
	// despite the name. The solves scatter their deltas into .xyz and the
	// count+prep kernels put the mass-splitting refCount into .w. Every iteration
	// cloth_applyExternalDeltasLaunch consumes and zeroes it, updating
	// mVelocity_InvMass always and mPosition_InvMass plus mAccumulatedDeltaPos in
	// position iterations only.
	float4* mDeltaPos;

	// Accumulated position delta since step start (integration plus constraint
	// corrections). Advances in position iterations only and stays frozen through
	// velocity iterations. The contact solves read it for computing the separation.
	float4* mAccumulatedDeltaPos;
	float4* mAccumulatedDeltaVel; // Used for damping

	PxBounds3* mPackedNodeBounds;

	// Element filter pairs of this surface, one sorted array per pair type. The type order of each
	// pair matches the type order expected by the find helpers in deformableElementFilter.cuh. The
	// pointers are grouped ahead of the counts to keep the struct free of interior padding.
	PxgDeformablePairInfo* mRigidSurfaceFilterPairs;
	PxgDeformablePairInfo* mSurfaceSurfaceFilterPairs;
	PxU32 mNumRigidSurfaceFilterPairs;
	PxU32 mNumSurfaceSurfaceFilterPairs;

	PxReal mLinearDamping;
	PxReal mMaxLinearVelocity;
	PxReal mPenBiasClamp;

	PxReal mSettlingThreshold;
	PxReal mSleepThreshold;
	PxReal mSettlingDamping;
	PxReal mSelfCollisionFilterDistance;

	PxU32 mNbVerts;
	PxU32 mNbTriangles;
	PxU32 mNbNonSharedTriangles;
	PxU32 mNbTrianglesWithOwnedEdges;

	PxU32 mNbTrianglePairs;
	PxU32 mNbSharedTrianglePairs;
	PxU32 mNbNonSharedTrianglePairs;

	PxU32 mNbNonSharedTriPartitions;
	PxU32 mNbSharedTriPairPartitions;
	PxU32 mNbNonSharedTriPairPartitions;

	// For partitions that contain only a small number of elements, run them in a single kernel call instead of launching multiple kernels
	// one by one.
	// clusterId stores the first partition that has fewer elements than PxgFEMClothKernelBlockDim::CLOTH_SOLVESHELL.

	PxU32 mNonSharedTriClusterId;
	PxU32 mSharedTriPairClusterId;
	PxU32 mNonSharedTriPairClusterId;

	PxU32 mElementIndex;
	PxU32 mGpuRemapIndex;
	PxU8 mActorFlags;
	PxU8 mBodyFlags;
	PxU16 mSurfaceFlags;

	PxU32 mIsActive;
	PxReal mRestOffset;
	PxReal mOriginalContactOffset;

	PxU32 mNbCollisionPairUpdatesPerTimestep;
	PxU32 mNbCollisionSubsteps;

} PX_ALIGN_SUFFIX(16);

#if PX_VC
#pragma warning(pop)
#endif

class PxgFEMClothUtil
{
  public:
	static PxU32 computeTriangleMeshByteSize(const Gu::TriangleMesh* triangleMesh);
	static PxU32 loadOutTriangleMesh(void* mem, const Gu::TriangleMesh* triangleMesh);

	static PxU32 initialTriangleData(PxgFEMCloth& femCloth, const Gu::TriangleMesh* triangleMesh,
									 const PxgDeformableSurfaceTopology& topology,
									 const PxU16* materialHandles, PxsDeformableSurfaceMaterialData* materials, const PxU32 nbMaterials,
									 Cm::VirtualAllocatorCallback& hostAlloc);

	static void categorizeClothConstraints(PxArray<PxU32>& sharedTrianglePairs, PxArray<PxU32>& nonSharedTriangles,
										   PxArray<PxU32>& nonSharedTrianglePairs, PxgFEMCloth& femCloth,
										   const PxArray<uint2>& trianglePairTriangleIndices);

	static void computeNonSharedTriangleConfiguration(PxgFEMCloth& femCloth, const PxArray<PxU32>& orderedNonSharedTriangles,
													  const PxArray<PxU32>& activeTriangleIndices,
													  const Gu::TriangleMesh* const triangleMesh);

	static float updateFlexuralStiffnessPerTrianglePair(float t0Area, float t1Area, float hingeLength, float thickness, float inputStiffness);

	static bool updateRestConfiguration(float4* orderedRestAngleAndStiffness_damping, uint4* orderedTrianglePairVertexIndices,
										PxgFEMCloth& femCloth, PxU32 it, PxU32 index, PxArray<uint2>& trianglePairTriangleIndices,
										const PxArray<uint4>& trianglePairVertexIndices, const PxsDeformableSurfaceMaterialData* materials,
										const PxVec3* positions, bool zeroRestBendingAngle, float4* orderedRestEdge0_edge1 = NULL,
										float4* orderedRestEdgeLength_material0_material1 = NULL);

	static void computeTrianglePairConfiguration(PxgFEMCloth& femCloth, PxArray<uint2>& trianglePairTriangleIndices,
												 const PxArray<uint4>& trianglePairVertexIndices, const PxArray<PxU32>& orderedTrianglePairs,
												 const PxArray<PxU32>& activeTrianglePairIndices, const Gu::TriangleMesh* const triangleMesh,
												 const PxsDeformableSurfaceMaterialData* materials, bool zeroRestBendingAngle,
												 bool isSharedPartition);
};

} // namespace physx

#endif