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

#ifndef PXG_FEMCLOTH_H
#define PXG_FEMCLOTH_H

#include "PxDeformableSurface.h"
#include "PxgCudaBuffer.h"
#include "PxsHeapMemoryAllocator.h"
#include "cutil_math.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxVec2.h"
#include "foundation/PxVec4.h"

namespace physx
{
namespace Gu
{
class TriangleMesh;
};

struct PxgFemContactInfo;
struct PxgFemRigidConstraintBlock;
struct PxsDeformableSurfaceMaterialData;

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4324) // padding was added at the end of a structure because of a __declspec(align) value.
#endif

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
	void deallocate(PxsHeapMemoryAllocator* allocator);


	void* mTriMeshData;

	/*******************************************************************************
	 *
	 *
	 * For each vertex
	 *
	 *
	 ******************************************************************************/

	float4* mVelocity_InvMass;
	float4* mPosition_InvMass;
	float4* mRestPosition;
	float4* mPrevPosition_InvMass;
	float* mDynamicFrictions; // dynamic friction per vertex
	PxU16* mMaterialIndices;

	/*******************************************************************************
	 *
	 *
	 * For each triangle
	 *
	 *
	 ******************************************************************************/

	uint4* mTriangleVertexIndices;
	uint4* mOrderedNonSharedTriangleVertexIndices_triIndex;
	float2* mOrderedSharedTriangleLambdas; // Two lambdas per triangle: ARAP, area
	float2* mOrderedNonSharedTriangleLambdas; // Two lambdas per triangle: ARAP, area
	float4* mOrderedNonSharedTriangleRestPoseInv; // Four components of restPoseInv: m00, m10, m01, m11

	/*******************************************************************************
	 *
	 *
	 * For each triangle-pair (2 adjacent triangles - 4 vertices)
	 *
	 *
	 ******************************************************************************/

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

	/*******************************************************************************
	 *
	 *
	 * Map between simulation vertices and mesh vertices
	 *
	 *
	 ******************************************************************************/

	float4* mPosition_InvMassCP; 
	
	PxU32* mNonSharedTriAccumulatedPartitionsCP;

	PxU32* mSharedTriPairRemapOutputCP;
	PxU32* mNonSharedTriPairRemapOutputCP;
	
	PxU32* mSharedTriPairAccumulatedCopiesCP;
	PxU32* mNonSharedTriPairAccumulatedCopiesCP;

	PxU32* mSharedTriPairAccumulatedPartitionsCP;
	PxU32* mNonSharedTriPairAccumulatedPartitionsCP;

	/*******************************************************************************
	 *
	 *
	 * Attachment / Contact
	 *
	 *
	 ******************************************************************************/

	float4* mDeltaPos; // Initialize to zero and zero every time in the apply delta kernel
	float4* mAccumulatedDeltaPos;
	float4* mAccumulatedDeltaVel; // Used for damping

	// After contact pairs are updated, cloth vertices have moved by mAccumulatedDeltaPos - mPrevAccumulatedDeltaPos.
	float4* mPrevAccumulatedDeltaPos; // Used for TGS cloth-cloth collision.

	PxBounds3* mPackedNodeBounds;

	// For cloth-rigid contact preparation.
	PxgFemRigidConstraintBlock* mRigidConstraints; // ((numVerts + 31) / 32) * maxNumContactPerVertex *
												   // sizeof(PxgFemRigidConstraintBlock)

	/*******************************************************************************
	 *
	 *
	 * Others
	 *
	 *
	 ******************************************************************************/

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
	PxReal mRestDistance;
	PxReal mOriginalContactOffset;

	PxU32 mNbCollisionPairUpdatesPerTimestep;
	PxU32 mNbCollisionSubsteps;

} PX_ALIGN_SUFFIX(16);

#if PX_VC
#pragma warning(pop)
#endif

class PxgFEMClothBuffer : public PxUserAllocated
{
  public:
	PxgFEMClothBuffer(PxgHeapMemoryAllocatorManager* heapMemoryManager);

	PxgCudaBuffer triangleMeshData;

	/*******************************************************************************
	 *
	 *
	 * For each vertex
	 *
	 *
	 ******************************************************************************/

	PxgTypedCudaBuffer<float4> prevPosition_InvMass;

	PxgTypedCudaBuffer<float4> deltaPos;
	PxgTypedCudaBuffer<float4> accumulatedDeltaPos;
	PxgTypedCudaBuffer<float4> accumulatedDeltaVel; // Used for damping
	PxgTypedCudaBuffer<float4> prevAccumulatedDeltaPos; // Used for TGS cloth-cloth collision

	PxgTypedCudaBuffer<PxU16> materialIndices;
	PxgTypedCudaBuffer<float> dynamicfrictions;

	/*******************************************************************************
	 *
	 *
	 * For each triangle
	 *
	 *
	 ******************************************************************************/

	PxgTypedCudaBuffer<uint4> triangleVertexIndices;
	PxgTypedCudaBuffer<uint4> orderedNonSharedTriangleVertexIndices_triIndex;

	PxgTypedCudaBuffer<float2> orderedSharedTriangleLambdas;
	PxgTypedCudaBuffer<float2> orderedNonSharedTriangleLambdas;
	
	PxgTypedCudaBuffer<float4> orderedNonSharedTriangleRestPoseInv;

	/*******************************************************************************
	 *
	 *
	 * For each triangle-pair (2 adjacent triangles - 4 vertices)
	 *
	 *
	 ******************************************************************************/

	PxgTypedCudaBuffer<uint4> orderedSharedTrianglePairVertexIndices;
	PxgTypedCudaBuffer<uint4> orderedNonSharedTrianglePairVertexIndices;
	
	PxgTypedCudaBuffer<float4> orderedSharedRestBendingAngle_flexuralStiffness_damping;
	PxgTypedCudaBuffer<float4> orderedNonSharedRestBendingAngle_flexuralStiffness_damping;

	PxgTypedCudaBuffer<float4> orderedSharedRestEdge0_edge1;
	PxgTypedCudaBuffer<float4> orderedSharedRestEdgeLength_material0_material1;

	PxgTypedCudaBuffer<float> sharedBendingLambdas;
	PxgTypedCudaBuffer<float> nonSharedBendingLambdas;
	
	/*******************************************************************************
	 *
	 *
	 * Map between simulation vertices and mesh vertices
	 *
	 *
	 ******************************************************************************/

	PxgTypedCudaBuffer<float4> position_InvMassCP;

	PxgTypedCudaBuffer<PxU32> nonSharedTriAccumulatedPartitionsCP;

	PxgTypedCudaBuffer<PxU32> sharedTriPairRemapOutputCP;
	PxgTypedCudaBuffer<PxU32> nonSharedTriPairRemapOutputCP;

	PxgTypedCudaBuffer<PxU32> sharedTriPairAccumulatedCopiesCP;
	PxgTypedCudaBuffer<PxU32> nonSharedTriPairAccumulatedCopiesCP;

	PxgTypedCudaBuffer<PxU32> sharedTriPairAccumulatedPartitionsCP;
	PxgTypedCudaBuffer<PxU32> nonSharedTriPairAccumulatedPartitionsCP;

	/*******************************************************************************
	 *
	 *
	 * Attachment / Contact
	 *
	 *
	 ******************************************************************************/

	PxgTypedCudaBuffer<PxBounds3> packedNodeBounds; // for refit

	PxgTypedCudaBuffer<PxU32> numPenetratedTets;
};

class PxgFEMClothUtil
{
  public:
	static PxU32 computeTriangleMeshByteSize(const Gu::TriangleMesh* triangleMesh);
	static PxU32 loadOutTriangleMesh(void* mem, const Gu::TriangleMesh* triangleMesh);
	static PxU32 initialTriangleData(PxgFEMCloth& femCloth, PxArray<uint2>& trianglePairTriangleIndices,
									 PxArray<uint4>& trianglePairVertexIndices, const Gu::TriangleMesh* triangleMesh,
									 const PxU16* materialHandles, PxsDeformableSurfaceMaterialData* materials, const PxU32 nbMaterials);
	static void categorizeClothConstraints(PxArray<PxU32>& sharedTrianglePairs, PxArray<PxU32>& nonSharedTriangles,
										   PxArray<PxU32>& nonSharedTrianglePairs, PxgFEMCloth& femCloth,
										   const PxArray<uint2>& trianglePairTriangleIndices,
										   const PxArray<uint4>& trianglePairVertexIndices);

	/*******************************************************************************
	 *
	 *
	 * For per-triangle energies
	 *
	 *
	 ******************************************************************************/

	static void computeNonSharedTriangleConfiguration(PxgFEMCloth& femCloth, const PxArray<PxU32>& orderedNonSharedTriangles,
													  const PxArray<PxU32>& activeTriangleIndices,
													  const Gu::TriangleMesh* const triangleMesh);

	/*******************************************************************************
	 *
	 *
	 * For per-triangle-pair energies
	 *
	 *
	 ******************************************************************************/

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