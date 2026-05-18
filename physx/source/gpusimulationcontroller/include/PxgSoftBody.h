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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved. 

#ifndef PXG_SOFTBODY_H
#define PXG_SOFTBODY_H

#include "foundation/PxMat33.h"
#include "foundation/PxBounds3.h"
#include "PxSoftBodyFlag.h" // deprecated
#include <vector_types.h>

#define MAX_SELF_COLLISION_CONTACTS		1000
#define NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA	2

#define SB_PARTITION_LIMIT 8 // max # partitions allowed. This value SHOULD NOT change. See also GuCookingTetrahedronMesh.cpp.

namespace physx
{
	namespace Gu
	{
		class DeformableVolumeMesh;
		class BVTetrahedronMesh;
		class TetrahedronMesh;
		class DeformableVolumeAuxData;
	}

	namespace Cm
	{
		class VirtualAllocatorCallback;
	}

	struct PxgMat33Block;
	struct PxgSpatialVectorBlock;
	class PxgNonRigidFilterPair;
	struct PxgAllocatorDesc;

#if PX_VC 
#pragma warning(push)   
#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	struct PxgSoftBodyData
	{
	public:
		
		PxU32			mRemapOutputSizeGM;
		PxU32			mMaxTetsPerPartitionsGM;
		PxU32			mTetsRemapSize; //size of tetrahedrons remap from collision mesh to simulation mesh

		PxU32			mNbPackedNodes;
	};

	PX_ALIGN_PREFIX(16)
	class PxgSoftBody
	{
	public:

		PX_DEPRECATED static PxU32 dataIndexFromFlagDEPRECATED(PxSoftBodyGpuDataFlag::Enum flag);

		// AD: We only use this for the host mirror as the GPU-side are PxgCudaBuffers and we only assign the pointers. 
		// Make sure to pass the right allocator in here!
		void deallocate(Cm::VirtualAllocatorCallback& hostAlloc); 

		void*					mTetMeshData;
		PxU8*					mTetMeshSurfaceHint;
		uint4*					mTetIndices;
		PxU32*					mTetIndicesRemapTable; //remap the tet indices to the source indices
		PxMat33*				mTetraStresses;
		PxMat33*				mTetraRestPoses;
		PxReal*					mTetraStressCoefficient;
		float4*					mTetraRotations; //sizeof mNumTets
		float4*					mPosition_InvMass;
		float4*					mRestPosition; //rest position of the verts - 4th component unused.

		uint4*					mSimTetIndices;
		float4*					mSimVelocity_InvMass;
		float4*					mSimPosition_InvMass;

		const float4*			mSimKinematicTarget;
		bool*					mVertsAreDeformed;
		bool*					mVertsCanNotDeform;
		float4*					mSimDeltaPos; //sizeof mNumVertsGM, store the deltaPos change for the grid model verts
		PxgMat33Block*			mSimTetraRestPoses;
		PxgMat33Block*			mOrigQinv;
		
		PxU32*					mSimOrderedTetrahedrons;
		PxU32*					mSimJacobiVertIndices;

		float4*					mSimTetraRotations; //sizeof mNumTetsGM
	
		float4*					mSimVelocity_invMassCP;//sizeof mRemapOutputSizeGM
		float4*					mSimPosition_InvMassCP; //sizeof mRemapOutputSizeGM
		uint4*					mSimRemapOutputCP; //sizeof numElements * numVertsPerElement
		PxU32*					mSimAccumulatedCopiesCP; //sizeof mNumVertsGM
		PxU32*					mSimAccumulatedPartitionsCP; //sizeof mMaxPartitionsGM

		float4*					mVertsBarycentricInGridModel;	//the barycentric of verts(from collision mesh) in simulation mesh
		PxU32*					mVertsRemapInGridModel;// the verts in collision mesh map to the tetrahedrons in simulation mesh 
		
		PxU32*					mTetsRemapColToSim; //tetrahedrons remap from collision mesh to simulation mesh
		PxU32*					mTetsAccumulatedRemapColToSim; // runsum of mTetsRemapColToSim sizeof mNumTets

		PxU8*					mSurfaceVertsHint;
		PxU32*					mSurfaceVertToTetRemap;

		PxgSpatialVectorBlock*	mSimTetraMultipliers;

		float4*					mSimDelta; //initialize to zero and zero every time in the apply delta kernel

		PxBounds3*				mPackedNodeBounds;

		PxU16*					mOrderedMaterialIndices; //indices to global material array ordered by partition, size of mNumVertsGM(simulation mesh)
		PxU16*					mMaterialIndices; //indices to global material array, size of mNumVertsGM(simulation mesh)

		//filter pair for myself and other soft body. Index0 is myself and vertId. Index1 is other soft body and tetId
		PxgNonRigidFilterPair*	mFilteringPairs; 
		PxU32					mNumFilterPairs;

		PxReal					mLinearDamping;
		PxReal					mMaxLinearVelocity;
		PxReal					mPenBiasClamp;

		PxReal					mSettlingThreshold;
		PxReal					mSleepThreshold;
		PxReal					mSettlingDamping;
		PxReal					mSelfCollisionFilterDistance;
		PxReal					mSelfCollisionStressTolerance;

		PxQuat					mInitialRotation;
		PxU32					mNumVerts;
		PxU32					mNumTets;

		PxU32					mNumVertsGM;
		PxU32					mNumTetsGM;
		PxU32					mNumPartitionsGM;
		
		PxU32					mElementIndex;
		PxU32					mGpuRemapIndex;
		PxU8					mActorFlags;
		PxU8					mBodyFlags;
		PxU16					mVolumeFlags;

		uint4*					mSimPullIndices;

		PxU32					mAwake;
		PxU32					mNumTetsPerElement;
		PxU32					mNumJacobiVertices;

		PxReal					mRestDistance;
		PxReal					mOriginalContactOffset;
		PxReal					mJacobiScale;
		
	}PX_ALIGN_SUFFIX(16);

#if PX_VC 
#pragma warning(pop)   
#endif

	class PxgSoftBodyBuffer;

	class PxgSoftBodyUtil
	{
	public:
		static PxU32 computeTetMeshByteSize(const Gu::BVTetrahedronMesh* tetMesh);
		static PxU32 loadOutTetMesh(void* mem, const Gu::BVTetrahedronMesh* tetMesh);
		static void initialTetData(PxgSoftBody& softbody, const Gu::BVTetrahedronMesh* colTetMesh, const Gu::TetrahedronMesh* simTetMesh, 
			const Gu::DeformableVolumeAuxData* softBodyAuxData, const PxU16* materialsHandles, Cm::VirtualAllocatorCallback& hostAlloc);
		static void computeBasisMatrix(PxMat33* restPoses, const Gu::DeformableVolumeMesh* tetMesh);

	};
}

#endif