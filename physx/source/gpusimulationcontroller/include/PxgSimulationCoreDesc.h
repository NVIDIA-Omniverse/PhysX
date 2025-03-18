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

#ifndef PXG_SIMULATION_CORE_DESC_H
#define	PXG_SIMULATION_CORE_DESC_H

#include "foundation/PxSimpleTypes.h"
#include "DyVArticulation.h"
#include "PxDeformableSurface.h"
#include "PxDeformableVolume.h"

#include <vector_types.h>

#define PXG_CHECK_BITSHIFT_32(lowmask, highmask, bitshift) \
	((1 << bitshift) - 1 == lowmask) && (0xffffffff >> bitshift == highmask)

#define PXG_BITSHIFT_TET_ID 20
#define PXG_BITSHIFT_ELEMENT_ID 20

PX_COMPILE_TIME_ASSERT(PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
PX_COMPILE_TIME_ASSERT(PXG_CHECK_BITSHIFT_32(PX_MAX_NB_DEFORMABLE_SURFACE_TRI, PX_MAX_NB_DEFORMABLE_SURFACE, PXG_BITSHIFT_ELEMENT_ID));
PX_COMPILE_TIME_ASSERT(PXG_CHECK_BITSHIFT_32(PX_MAX_NB_DEFORMABLE_VOLUME_TET, PX_MAX_NB_DEFORMABLE_VOLUME, PXG_BITSHIFT_TET_ID));

namespace physx
{
	struct PxgSolverBodySleepData;
	struct PxgShape;
	struct PxgBodySim;
	struct PxgShapeSim;
	struct PxgArticulationLink;
	struct PxgArticulationLinkProp;
	struct PxsCachedTransform;
	struct PxgBodySimVelocityUpdate;
	struct PxgD6JointData;
	struct PxgConstraintPrePrep;
	class PxgArticulation;
	class PxNodeIndex;
	class PxgArticulationTendon;
	class PxGpuSpatialTendonData;
	class PxGpuFixedTendonData;
	class PxGpuTendonAttachmentData;
	class PxgArticulationTendonElementFixedData;
	class PxGpuTendonJointCoefficientData;

	namespace Dy
	{
		struct ArticulationJointCore;
		class ArticulationJointCoreData;
	}
	
	struct PxgNewBodiesDesc
	{
		const PxgBodySim*	mNewBodySim;
		PxgBodySim*			mBodySimBufferDeviceData;
		PxU32				mNbNewBodies;	//number of newly added bodies
	};

	struct PX_ALIGN_PREFIX(16) PxgArticulationSimUpdate
	{
		PxU32 dirtyFlags;							//What data is updated (ArticulationDirtyFlag)
		PxU32 articulationIndex;					//which articulation on the GPU			
		PxU32 linkStartIndex;						//Where the link/joint data starts in memory
		PxU32 dofDataStartIndex;					//Where the dof data starts in memory, e.g. jointVelocity, jointPosition, jointForce etc.

		PxU32 spatialTendonStartIndex;				//Where the spatial tendon starts in memory
		PxU32 spatialTendonAttachmentStartIndex;	//Where the spatial tendon attachment starts in memory
		PxU32 fixedTendonStartIndex;				//Where the fixed tendon starts in memory
		PxU32 fixedTendonJointStartIndex;			//Where the fixed tendon joint starts in memory

		PxU32 mimicJointStartIndex;
		PxU32 pathToRootIndex;						//Where the pathToRoot array starts in memory

		PxU8 userFlags;								//(PxArticulationFlag)
		PxU8 pad[7];
		
	}PX_ALIGN_SUFFIX(16);

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxEncodeClothIndex(const PxU32 clothId, const PxU32 elementId)
	{
		// elementId can be PX_MAX_NB_DEFORMABLE_SURFACE_TRI/PX_MAX_NB_DEFORMABLE_SURFACE_VTX to get at mask for "any tri or vtx"
		PX_ASSERT(clothId < PX_MAX_NB_DEFORMABLE_SURFACE);
		PX_ASSERT(elementId <= PX_MAX_NB_DEFORMABLE_SURFACE_TRI); 
		PX_ASSERT(elementId <= PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
		return (clothId << PXG_BITSHIFT_ELEMENT_ID) | elementId;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxEncodeSoftBodyIndex(const PxU32 softBodyId, const PxU32 tetId)
	{
		// tetId can be PX_MAX_NB_DEFORMABLE_VOLUME_TET to get at mask for "any tet"
		PX_ASSERT(softBodyId < PX_MAX_NB_DEFORMABLE_VOLUME);
		PX_ASSERT(tetId <= PX_MAX_NB_DEFORMABLE_VOLUME_TET); 
		return (softBodyId << PXG_BITSHIFT_TET_ID) | tetId;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxEncodeParticleIndex(const PxU32 particleSystemId, const PxU32 particleId)
	{
		PX_ASSERT(particleSystemId < 0xffffffff);
		PX_ASSERT(particleId < 0xffffffff);
		return (PxU64(particleSystemId) << 32ull) | particleId; 
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxGetSoftBodyElementIndex(const PxU32 compressedId)
	{
		return PX_MAX_NB_DEFORMABLE_VOLUME_TET & compressedId;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxGetSoftBodyId(const PxU32 compressedId)
	{
		return compressedId >> PXG_BITSHIFT_TET_ID;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxGetClothElementIndex(const PxU32 compressedId)
	{
		return PX_MAX_NB_DEFORMABLE_SURFACE_TRI & compressedId;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxGetClothId(const PxU32 compressedId)
	{
		return compressedId >> PXG_BITSHIFT_ELEMENT_ID;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxGetIsVertexType(const float4& baryOrType)
	{
		return (baryOrType.x == 0.0f && baryOrType.y == 0.0f && baryOrType.z == 0.0f && baryOrType.w == 0.0f);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxGetParticleIndex(const PxU64 compressedId)
	{
		return (0xffffffff) & compressedId;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxGetParticleSystemId(const PxU64 compressedId)
	{
		return compressedId >> 32;
	}

	PX_ALIGN_PREFIX(16)
	class PxgRigidFilterPair
	{
	public:
		PX_CUDA_CALLABLE PxgRigidFilterPair() : index2(0)
		{}

		PxU64 index0; //rigid body index will always in index0
		PxU64 index1;		
		PxU32 index2;
		PxU32 pading[3];

		PX_CUDA_CALLABLE PxI32 compare(const PxgRigidFilterPair& other) const
		{
			if (index0 < other.index0)
				return -1;
			if (index0 > other.index0)
				return 1;
			if (index1 < other.index1)
				return -1;
			if (index1 > other.index1)
				return 1;
			if (index2 < other.index2)
				return -1;
			if (index2 > other.index2)
				return 1;
			return 0;
		}
	}PX_ALIGN_SUFFIX(16);


	struct PxgConeLimitParams
	{
		union
		{
			float4 low_high_limits;
			float4 low_high_angle;
		};

		union
		{
			float4 axis_angle;
			float4 barycentric;
		};
	};

	//KS - consider splitting into 2x structures - 16 bytes and 8 bytes!
	//soft body/cloth with rigid
	class PxgFEMRigidAttachment : public PxgRigidFilterPair
	{
	public:
		union 
		{
			float4 localPose0;
			float4 baryOrType0;
		};

		PxgConeLimitParams coneLimitParams;
		float4 baryOrType1;
	};
	
	PX_ALIGN_PREFIX(16)
	struct PxgParticleRigidConstraint
	{
		float4	raXn0_biasW[32];
		float4	raXn1_biasW[32];
		float4	raXn2_biasW[32];
		float4	velMultiplierXYZ_invMassW[32];
		float4	low_high_limits[32];
		float4	axis_angle[32];
		PxU64	particleId[32];
		PxU64	rigidId[32];
	}PX_ALIGN_SUFFIX(16);

	PX_ALIGN_PREFIX(16)
	class PxgNonRigidFilterPair
	{
	public:
		PX_CUDA_CALLABLE PxgNonRigidFilterPair() : index2(0) // : referenceCount(0)
		{}

		PxU64 index0;
		PxU32 index1;
		PxU32 index2;
		

		PX_CUDA_CALLABLE PxI32 compare(const PxgNonRigidFilterPair& other) const
		{
			if (index0 < other.index0)
				return -1;
			if (index0 > other.index0)
				return 1;
			if (index1 < other.index1)
				return -1;
			if (index1 > other.index1)
				return 1;
			if (index2 < other.index2)
				return -1;
			if (index2 > other.index2)
				return 1;
			return 0;
		}
	}PX_ALIGN_SUFFIX(16);

	PX_ALIGN_PREFIX(16)
	class PxgFEMFEMAttachment
	{
	public:
		PxgConeLimitParams coneLimitParams;
		float4 barycentricCoordinates0;
		float4 barycentricCoordinates1;
		PxU64  index0;
		PxU32  index1;
		PxReal constraintOffset;

	}PX_ALIGN_SUFFIX(16);

	PX_COMPILE_TIME_ASSERT(sizeof(PxgFEMFEMAttachment) % 16 == 0);

	struct PxgUpdateArticulationDesc
	{
		PxgArticulation*						mNewArticulations;
		PxReal*									mNewLinkWakeCounters;
		PxgArticulationLink*					mNewLinks;
		PxgArticulationLinkProp*				mNewLinkProps;
		
		PxU32*									mNewLinkParents;
		Dy::ArticulationBitField*				mNewLinkChildren;
		PxTransform*							mNewLinkBody2Worlds;
		PxTransform*							mNewLinkBody2Actors;
		Cm::UnAlignedSpatialVector*				mNewLinkExtAccels;
		Dy::ArticulationJointCore*				mNewJointCores;
		Dy::ArticulationJointCoreData*			mNewJointData;
		PxgArticulationSimUpdate*				mIndicesOffset;

		PxgArticulation*						mArticulationPool;
		PxgSolverBodySleepData*					mArticulationSleepDataPool;

		PxGpuSpatialTendonData*					mNewSpatialTendonParamsPool;
		PxgArticulationTendon*					mNewSpatialTendonPool;
		PxgArticulationTendonElementFixedData*	mNewAttachmentFixedPool;
		PxGpuTendonAttachmentData*				mNewAttachmentModPool;
		PxU32*									mNewTendonAttachmentRemapPool;

		PxGpuFixedTendonData*					mNewFixedTendonParamsPool;
		PxgArticulationTendon*					mNewFixedTendonPool;
		PxgArticulationTendonElementFixedData*	mNewTendonJointFixedPool;
		PxGpuTendonJointCoefficientData*		mNewTendonJointCoefficientPool;
		PxU32*									mNewTendonTendonJointRemapPool;
		Dy::ArticulationMimicJointCore*			mNewArticulationMimicJointPool;	

		PxU32*									mNewPathToRootPool;
		
		PxU32									mNbNewArticulations;	//number of newly added aritculations	
	};

	struct PxgUpdatedBodiesDesc
	{
		PxgBodySim*					mBodySimBufferDeviceData;
		PxgBodySimVelocityUpdate*	mUpdatedBodySim;
		PxU32						mNbUpdatedBodies;
	};

	struct PxgUpdatedJointsDesc
	{
		const PxgD6JointData*		mD6RigidJointCPUPool;			//map memory
		PxgD6JointData*				mD6RigidJointGPUPool;			//device memory

		const PxgConstraintPrePrep*	mD6RigidJointPrePrepCPUPool;	//map memory
		PxgConstraintPrePrep*		mD6RigidJointPrePrepGPUPool;	//device memory

		const PxgD6JointData*		mD6ArtiJointCPUPool;			//map memory
		PxgD6JointData*				mD6ArtiJointGPUPool;			//device memory

		const PxgConstraintPrePrep*	mD6ArtiJointPrePrepCPUPool;		//map memory
		PxgConstraintPrePrep*		mD6ArtiJointPrePrepGPUPool;		//device memory

		const PxU32*				mUpdatedRigidJointIndices;		//map memory
		PxU32						mNbUpdatedRigidJoints;

		const PxU32*				mUpdatedArtiJointIndices;		//map memory
		PxU32						mNbUpdatedArtiJoints;
	};

	struct PxgSimulationCoreDesc
	{
		PxU32*					mChangedAABBMgrHandles;
		PxU32*					mFrozen;
		PxU32*					mUnfrozen;
		PxU32*					mFrozenBlockAndRes;//this array is used to store frozen block value for the scan and also store the final frozen shape index
		PxU32*					mUnfrozenBlockAndRes;//this array is used to store unfrozen block value for the scane and also store the final unfrozen shape index
		PxU32*					mUpdated;
		PxU32*					mActivate;
		PxU32*					mDeactivate;

		PxgBodySim*				mBodySimBufferDeviceData;
		const PxgShapeSim*		mShapeSimsBufferDeviceData;
		PxgArticulation*		mArticulationPool;
		PxgSolverBodySleepData*	mArticulationSleepDataPool;

		PxsCachedTransform*		mTransformCache;
		PxBounds3*				mBounds;

		PxU32*					mBodyDataIndices;

		PxgSolverBodySleepData*	mSleepData;
		PxgShape*				mShapes;
		
		PxU32					mNbTotalShapes;	//number of total shapes;
		PxU32					mBitMapWordCounts; //number of words in gChangedAABBMgrHandles, this will include shapes and aggregates

		PxU32					mTotalFrozenShapes;   // AD: these two members are the only reason we copy the whole descriptor back to cpu.
		PxU32					mTotalUnfrozenShapes;
	};

	struct PxgUpdateActorDataDesc
	{
		PxgBodySim*			mBodySimBufferDeviceData;

		PxgShape*			mShapes;

		PxsCachedTransform*	mTransformCache;
		PxBounds3*			mBounds;

		PxNodeIndex*		mRigidNodeIndices;
		PxU32*				mShapeIndices;

		PxU32*				mUpdated;					//Direct API changed handle
		PxU32*				mChangedAABBMgrHandles;		//CPU API changed handle

		PxU32				mBitMapWordCounts;

		const PxgShapeSim*	mShapeSimsBufferDeviceData;
	};
}
#endif
