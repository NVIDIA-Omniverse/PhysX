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

#ifndef PXG_PARTICLE_SYSTEM_H
#define PXG_PARTICLE_SYSTEM_H

#include "foundation/PxPinnedArray.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxUserAllocated.h"

#include "PxParticleBuffer.h"
#include "PxParticleGpu.h"

#include "PxgCudaBuffer.h"

#include "DyParticleSystemCore.h"
#include "PxsParticleBuffer.h"

#include <vector_types.h>
#include "PxNodeIndex.h"
#include "PxgSimulationCoreDesc.h"

namespace physx
{
#define	EMPTY_CELL	0xffffffff
#define NUM_SPRING_PER_BLOCK			4

	struct PxsParticleMaterialData;
	struct PxParticleRigidFilterPair;
	struct PxParticleRigidAttachment;
	class PxCudaContextManager;
	class PxsHeapMemoryAllocatorManager;
	class PxNodeIndex;
	class PxgPBDMaterialDerived;

	template<typename MaterialType>
	PX_CUDA_CALLABLE PX_FORCE_INLINE const MaterialType& getParticleMaterial(const PxsParticleMaterialData* PX_RESTRICT materials,
		const PxU32 id, const PxU32 stride)
	{
		const PxU8* PX_RESTRICT data = reinterpret_cast<const PxU8*>(materials) + id * stride;
		return *reinterpret_cast<const MaterialType*>(data);
	}

	struct PX_ALIGN_PREFIX(16) PxgPBDParticleMaterialDerived
	{
		PxReal	surfaceTensionDerived;
		PxReal	cohesion;
		PxReal	cohesion1;
		PxReal	cohesion2;
		

		__device__ PxgPBDParticleMaterialDerived() {}

		__device__ PxgPBDParticleMaterialDerived(const PxReal surfaceTension_, 
			const PxReal cohesion_, const PxReal cohesion1_, const PxReal cohesion2_) 
			: surfaceTensionDerived(surfaceTension_), cohesion(cohesion_), cohesion1(cohesion1_), cohesion2(cohesion2_)
		{
		}
	};

	
	struct PxgParticleSystemData
	{
	public:
		PxVec3		mBoundCenter;

		PxU32		mElementIndex;
		PxU32		mRemapIndex;

		PxReal		mRestOffset;
		PxReal		mSolidRestOffset;

		PxReal		mSpiky1;
		PxReal		mSpiky2;
		PxReal		mRestDensity;
		PxReal		mRestDensityBoundary;
		PxReal		mFluidSurfaceConstraintScale;
		PxReal		mLambdaScale;

		PxReal		mRelaxationFactor;
		PxReal		mFluidRestOffset;
		PxReal		mInvRestDensity;

		PxU32		mFlags;

		PxReal		mMaxVelocity;
	
		PxU16		mLockFlags;
		PxU32		mNumPhaseToMaterials;

		PxVec3		mWind;
	};

	
	//each particle has one collision header
	struct PX_ALIGN_PREFIX(8) PxgParticleCollisionHeader
	{
	public:
		PxU32 mPrimitiveCollisionStartIndex;
		PxU32 mPrimitiveCounts;					//how many primitives are colliding with this particle
	} PX_ALIGN_SUFFIX(8);

	PX_ALIGN_PREFIX(16)
	struct PxgParticleContactInfo
	{
		static const PxU32 MaxStaticContactsPerParticle = 12;
		static const PxU32 MaxStaticContactsPerMesh = 6;
		float4 mNormal_PenW;
	}PX_ALIGN_SUFFIX(16);

	struct PxgParticleSimBuffer
	{	
		float4* 					mPositionInvMasses;
		float4* 					mVelocities;
		float4* 					mRestPositions;
		PxU32*						mPhases;
		PxParticleVolume* 			mVolumes;
		PxParticleRigidFilterPair* 	mFilterPairs;
		PxParticleRigidAttachment* 	mRigidAttachments;

		PxU32						mNumActiveParticles;
		PxU32						mNumVolumes;
		PxU32						mNumFilterPairs;
		PxU32						mNumRigidAttachments;

		PxU32						mFlags;
		PxU32						mDiffuseParticleBufferIndex;
		PxU32						mUniqueId; //Remains unchanged over the whole lifetime of a buffer
	};

	struct PxgParticleClothSimBuffer
	{	
		//Cloth
		PxU32* mAccumulatedSpringsPerPartitions; //numPartitions;
		PxU32* mAccumulatedCopiesPerParticles;	//numSprings
		PxU32* mRemapOutput;					//numSprings * 2
		PxParticleSpring* mOrderedSprings;		//numSprings
		PxU32* mTriangles;						//numTriangles  * 3

		PxU32* mSortedClothStartIndices;		//numCloths

		PxParticleCloth* mCloths;				//numCloths

		float4* mRemapPositions;
		float4* mRemapVelocities;
		PxReal* mSpringLambda;
		PxReal* mInflatableLambda;

		PxU32 mParticleBufferIndex;				//which particle buffer this cloth buffer associated with
		PxU32 mNumSprings;
		PxU32 mNumPartitions;
		PxU32 mNumCloths;
		PxU32 mNumTriangles;
	};

	struct PxgParticleRigidSimBuffer
	{
		PxReal* mRigidCoefficients;		//mNumRigids;
		float4* mRigidTranslations;		//mNumRigids
		float4* mRigidRotations;		//mNumRigids
		PxU32*	mRigidOffsets;			//mNumRigids + 1
		float4* mRigidLocalPositions;	//mNumActiveParticles
		float4* mRigidLocalNormals;		//mNumActiveParticles

		PxU32   mNumRigids;
		PxU32   mParticleBufferIndex;
	};

	struct PxgParticleDiffuseSimBuffer
	{
		PxDiffuseParticleParams mParams;

		float4* mDiffusePositions_LifeTime;
		float4* mDiffuseVelocities;

		PxU32	mMaxNumParticles;
		int*	mNumDiffuseParticles;		//device memory
		int*	mNumActiveDiffuseParticles; //pinned memory

		PxU32	mStartIndex;

		PxU32   mFlags;
	};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	template<class BufferClass>
	class PxgParticleBufferBase : public BufferClass, public PxUserAllocated
	{
	public:
		PxgParticleBufferBase(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxCudaContextManager& contextManager);
		virtual ~PxgParticleBufferBase();

		//PxsParticleBuffer
		virtual PxVec4* getPositionInvMassesD() const PX_OVERRIDE PX_FINAL { return mPositionInvMassesD; }
		virtual PxVec4* getVelocitiesD() const PX_OVERRIDE PX_FINAL { return mVelocitiesD; }
		virtual PxU32* getPhasesD() const PX_OVERRIDE PX_FINAL { return mPhasesD; }
		virtual PxParticleVolume* getParticleVolumesD() const PX_OVERRIDE PX_FINAL { return mVolumesD; }
		virtual PxVec4* getPositionInvMassesH() const PX_OVERRIDE PX_FINAL { return mPositionInvMassesH; }
		virtual PxVec4* getVelocitiesH() const PX_OVERRIDE PX_FINAL { return mVelocitiesH; }
		virtual PxU32* getPhasesH() const PX_OVERRIDE PX_FINAL { return mPhasesH; }
		virtual PxParticleVolume* getParticleVolumesH() const PX_OVERRIDE PX_FINAL { return mVolumesH; }
		virtual void setNbActiveParticles(PxU32 nbActiveParticles) PX_OVERRIDE;
		virtual PxU32 getNbActiveParticles() const PX_OVERRIDE PX_FINAL { return mNumActiveParticles; }
		virtual PxU32 getMaxParticles() const PX_OVERRIDE PX_FINAL { return mMaxNumParticles; }
		virtual PxU32 getNbParticleVolumes() const PX_OVERRIDE PX_FINAL { return mNumParticleVolumes; }
		virtual void setNbParticleVolumes(PxU32 nbParticleVolumes) PX_OVERRIDE PX_FINAL { mNumParticleVolumes = nbParticleVolumes; }
		virtual PxU32 getMaxParticleVolumes() const PX_OVERRIDE PX_FINAL { return mMaxNumVolumes; }
		virtual void setRigidFilters(PxParticleRigidFilterPair* filters, PxU32 nbFilters) PX_OVERRIDE PX_FINAL;
		virtual void setRigidAttachments(PxParticleRigidAttachment* attachments, PxU32 nbAttachments) PX_OVERRIDE PX_FINAL;
		virtual PxU32 getFlatListStartIndex() const PX_OVERRIDE PX_FINAL { return mFlatListStartIndex; }
		virtual void raiseFlags(PxParticleBufferFlag::Enum flags) PX_OVERRIDE PX_FINAL { mBufferFlags |= flags; }
		virtual PxU32 getUniqueId() const PX_OVERRIDE PX_FINAL { return mUniqueId; }
		virtual void allocHostBuffers() PX_OVERRIDE PX_FINAL;
		//~PxsParticleBuffer

		void setFlatListStartIndex(PxU32 flatListStartIndex) { mFlatListStartIndex = flatListStartIndex; }
		PxParticleRigidAttachment* getRigidAttachments() const { return mRigidAttachments; }
		PxU32 getNbRigidAttachments() const { return mNumRigidAttachments; }
		PxParticleRigidFilterPair* getRigidFilters() const { return mFilterPairs; }
		PxU32 getNbRigidFilters() const { return mNumFilterPairs; }
		void copyToHost(CUstream stream);

		PxU32 mBufferFlags;

	public:
		PxCudaContextManager& mContextManager;

		PxVec4* mPositionInvMassesD;
		PxVec4* mVelocitiesD;
		PxU32* mPhasesD;
		PxParticleVolume* mVolumesD;
		PxVec4* mPositionInvMassesH;
		PxVec4* mVelocitiesH;
		PxU32* mPhasesH;
		PxParticleVolume* mVolumesH;
		PxParticleRigidFilterPair* mFilterPairs;
		PxParticleRigidAttachment* mRigidAttachments;
		PxU32 mNumActiveParticles;
		PxU32 mMaxNumParticles;
		PxU32 mNumParticleVolumes;
		PxU32 mMaxNumVolumes;
		PxU32 mNumFilterPairs;
		PxU32 mNumRigidAttachments;
		PxU32 mFlatListStartIndex;
		PxU32 mUniqueId;
	};

	class PxgParticleBuffer : public PxgParticleBufferBase<PxsParticleBuffer>
	{
	public:
		PxgParticleBuffer(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxCudaContextManager& contextManager);
		virtual ~PxgParticleBuffer() {}

		//PxsParticleBuffer
		virtual void release() PX_OVERRIDE PX_FINAL { PX_DELETE_THIS; }
		//~PxsParticleBuffer

		void copyToHost(CUstream stream);
	};

	class PxgParticleAndDiffuseBuffer : public PxgParticleBufferBase<PxsParticleAndDiffuseBuffer>
	{
	public:
		PxgParticleAndDiffuseBuffer(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxU32 maxNumDiffuseParticles, PxCudaContextManager& contextManager);
		virtual ~PxgParticleAndDiffuseBuffer();
		
		//PxsParticleAndDiffuseBuffer
		virtual void release() PX_OVERRIDE PX_FINAL { PX_DELETE_THIS; }
		virtual PxVec4* getDiffusePositionLifeTimeD() const PX_OVERRIDE PX_FINAL { return mDiffusePositionsLifeTimeD; }
		virtual PxVec4* getDiffuseVelocitiesD() const PX_OVERRIDE PX_FINAL { return mDiffuseVelocitiesD; }
		virtual PxU32 getNbActiveDiffuseParticles() const PX_OVERRIDE PX_FINAL { return mNumActiveDiffuseParticlesH[0]; }
		virtual void setMaxActiveDiffuseParticles(PxU32 maxActiveDiffuseParticles) PX_OVERRIDE PX_FINAL;
		virtual PxU32 getMaxDiffuseParticles() const PX_OVERRIDE PX_FINAL { return mMaxNumDiffuseParticles; }
		virtual void setDiffuseParticleParams(const PxDiffuseParticleParams& params) PX_OVERRIDE PX_FINAL;
		virtual const PxDiffuseParticleParams& getDiffuseParticleParams() const PX_OVERRIDE PX_FINAL { return mParams; }
		//~PxsParticleAndDiffuseBuffer

		void copyToHost(CUstream stream);

	public:
		PxDiffuseParticleParams	mParams;
		PxVec4* mDiffusePositionsLifeTimeD;
		PxVec4* mDiffuseVelocitiesD;
		PxI32* mNumDiffuseParticlesD;
		PxU32 mMaxNumDiffuseParticles;
		PxU32 mMaxActiveDiffuseParticles;
		PxI32* mNumActiveDiffuseParticlesH; //pinned memory
	};

	class PxgParticleClothBuffer : public PxgParticleBufferBase<PxsParticleClothBuffer>
	{
	public:
		PxgParticleClothBuffer(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxU32 maxNumCloths, 
			PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager& contextManager);
		virtual ~PxgParticleClothBuffer();

		//PxsParticleClothBuffer
		virtual void release() PX_OVERRIDE PX_FINAL { PX_DELETE_THIS; }
		virtual PxVec4* getRestPositionsD() PX_OVERRIDE PX_FINAL { return mRestPositionsD; }
		virtual PxU32* getTrianglesD() const PX_OVERRIDE PX_FINAL { return mTriangleIndicesD; }
		virtual void setNbTriangles(PxU32 nbTriangles) PX_OVERRIDE PX_FINAL { mNumTriangles = nbTriangles; }
		virtual PxU32 getNbTriangles() const PX_OVERRIDE PX_FINAL { return mNumTriangles; }
		virtual PxU32 getNbSprings() const PX_OVERRIDE PX_FINAL { return mNumSprings; }
		virtual PxParticleSpring* getSpringsD() PX_OVERRIDE PX_FINAL { return mOrderedSpringsD; }
		virtual void setCloths(PxPartitionedParticleCloth& cloths) PX_OVERRIDE PX_FINAL;
		virtual void setNbActiveParticles(PxU32 nbActiveParticles) PX_OVERRIDE PX_FINAL;
		//~PxsParticleClothBuffer

		void copyToHost(CUstream stream);

	public:
		PxVec4* mRestPositionsD;
		PxU32* mTriangleIndicesD;

		PxU32* mAccumulatedSpringsPerPartitionsD;	//numPartitions;
		PxU32* mAccumulatedCopiesPerParticlesD;		//numSprings
		PxU32* mRemapOutputD;						//numSprings * 2
		PxParticleSpring* mOrderedSpringsD;			//numSprings

		PxU32* mSortedClothStartIndicesD;			//numCloths

		PxParticleCloth* mClothsD;					//numClothes

		PxVec4* mRemapPositionsD;
		PxVec4* mRemapVelocitiesD;
		PxReal* mSpringLambdaD;
		PxReal* mInflatableLambdaD;

		PxU32 mMaxNumCloths;
		PxU32 mMaxNumTriangles;
		PxU32 mMaxNumSprings;
		PxU32 mNumPartitions;
		PxU32 mMaxSpringsPerPartition;

		PxU32 mNumSprings;
		PxU32 mNumCloths;
		PxU32 mNumTriangles;
		PxU32 mRemapOutputSize;
	};

	class PxgParticleRigidBuffer : public PxgParticleBufferBase<PxsParticleRigidBuffer>
	{
	public:
		PxgParticleRigidBuffer(PxU32 maxNumParticles, PxU32 maxNumVolumes, PxU32 maxNumRigids, PxCudaContextManager& contextManager);
		virtual ~PxgParticleRigidBuffer();

		//PxsParticleRigidBuffer
		virtual void release() PX_OVERRIDE PX_FINAL { PX_DELETE_THIS; }
		PxU32* getRigidOffsetsD() const PX_OVERRIDE PX_FINAL { return mRigidOffsetsD; }
		PxReal* getRigidCoefficientsD() const PX_OVERRIDE PX_FINAL { return mRigidCoefficientsD; }
		PxVec4* getRigidLocalPositionsD() const PX_OVERRIDE PX_FINAL { return mRigidLocalPositionsD; }
		PxVec4* getRigidLocalNormalsD() const PX_OVERRIDE PX_FINAL { return mRigidLocalNormalsD; }
		PxVec4* getRigidTranslationsD() const PX_OVERRIDE PX_FINAL { return mRigidTranslationsD; }
		PxVec4* getRigidRotationsD() const PX_OVERRIDE PX_FINAL { return mRigidRotationsD; }
		void setNbRigids(const PxU32 nbRigids) PX_OVERRIDE PX_FINAL { mNumActiveRigids = nbRigids; }
		PxU32 getNbRigids() const PX_OVERRIDE PX_FINAL { return mNumActiveRigids; }
		//~PxsParticleRigidBuffer

		void copyToHost(CUstream stream);

	public:
		PxU32* mRigidOffsetsD;
		PxReal* mRigidCoefficientsD;
		PxVec4* mRigidLocalPositionsD;
		PxVec4* mRigidLocalNormalsD;
		PxVec4* mRigidTranslationsD;
		PxVec4* mRigidRotationsD;
		PxU32	mNumActiveRigids;
		PxU32	mMaxNumRigids;
	};

#if PX_VC 
#pragma warning(push)   
#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	PX_ALIGN_PREFIX(16)
	class PxgParticleSystem : public PxGpuParticleSystem
	{
	public:
		

		float4*						mOriginPos_InvMass;			//8		16
		PxU32*						mGridParticleHash;			//16	32
		
		
		float4*						mAccumDeltaP;				//32	80
		//float4*					mDeltaP;					//32	80
		float4*						mSortedDeltaP;				//32	80
		PxU32*						mCellStart;					//36	88
		PxU32*						mCellEnd;					//40	96
		PxgParticleCollisionHeader*	mCollisionHeaders;			//44	104
		
		float2*						mDensityCollisionImpulses;
		PxgParticleContactInfo*		mOneWayContactInfos;		//56	128
		float2*						mOneWayForces;				//60
		PxNodeIndex*				mOneWayNodeIndex;			//64	136
		float4*						mOneWaySurfaceVelocity;		//68	144
		PxU32*						mOneWayContactCount;		//72	152
		
		float4*						mRestArray;					//84

		PxgParticleSystemData		mData;

		PxReal*						mDensity;
		PxReal*						mStaticDensity;
		float4*						mSurfaceNormal;
		float4*						mDelta;
		float4*						mCurl;

		// Normals
		float4*						mNormalArray;

		float4*						mSortedOriginPos_InvMass;

		PxgParticleSimBuffer*		mParticleSimBuffers;
		PxU32*						mParticleBufferRunsum;
		PxU32*						mParticleBufferSortedUniqueIds;
		PxU32*						mParticleBufferSortedUniqueIdsOriginalIndex;

		PxgParticleClothSimBuffer*		mClothSimBuffers;
		PxgParticleRigidSimBuffer*		mRigidSimBuffers;
		PxgParticleDiffuseSimBuffer*	mDiffuseSimBuffers;

		PxU32*						mAttachmentRunsum;
		
		PxU32						mNumClothBuffers;
		PxU32						mNumRigidBuffers;
		PxU32						mNumDiffuseBuffers;
		PxU32						mNumRigidAttachments;
		PxU32						mRigidAttachmentOffset;
				
		// Diffuse particles
		int*						mNumDiffuseParticles;		
		float4*						mDiffusePosition_LifeTime;
		float4*						mDiffuseVelocity;
		float2*						mDiffusePotentials;
		PxU32*						mDiffuseCellStart;
		PxU32*						mDiffuseCellEnd;
		PxU32*						mDiffuseGridParticleHash;
		float4*						mDiffuseOriginPos_LifeTime;
		float4*						mDiffuseSortedPos_LifeTime;
		float4*						mDiffuseSortedOriginPos_LifeTime;
		float4*						mDiffuseSortedVel;
		//GPU pointer to the mapping from sorted particle ID to unsorted particle ID
		PxU32*						mDiffuseSortedToUnsortedMapping;
		//GPU pointer to the mapping from unsortedParticle ID to sorted particle ID
		PxU32*						mDiffuseUnsortedToSortedMapping;
		PxgParticleContactInfo*		mDiffuseOneWayContactInfos;
		float*						mDiffuseOneWayForces;
		PxNodeIndex*				mDiffuseOneWayNodeIndex;
		PxU32*						mDiffuseOneWayContactCount;

		
		PxgPBDParticleMaterialDerived*	mDerivedPBDMaterialData;
		PxU32						mParticleMaterialStride;

		PxU16*						mPhaseGroupToMaterialHandle;

	}PX_ALIGN_SUFFIX(16);

#if PX_VC 
#pragma warning(pop)   
#endif	

	class PxgParticleSystemBuffer
	{
	public:

		PxgParticleSystemBuffer(PxgHeapMemoryAllocatorManager* heapMemoryManager);

		PxgTypedCudaBuffer<PxVec4>			originalPosition_mass;			//we should be able to get rid of this buffer
		PxgTypedCudaBuffer<PxU32>			grid_particle_hash;
		PxgTypedCudaBuffer<PxU32>			grid_particle_index;
		PxgTypedCudaBuffer<PxVec4>			sorted_position_mass;
		PxgTypedCudaBuffer<PxVec4>			sorted_velocity;
		PxgTypedCudaBuffer<PxVec4>			accumDeltaV;
		PxgTypedCudaBuffer<PxVec4>			sortedDeltaP;
		PxgTypedCudaBuffer<PxU32>			cell_start;
		PxgTypedCudaBuffer<PxU32>			cell_end;
		PxgTypedCudaBuffer<PxgParticleCollisionHeader> collision_headers;
		PxgTypedCudaBuffer<PxU32>			collision_counts;
		PxgTypedCudaBuffer<PxU32>			collision_index;
		PxgTypedCudaBuffer<float2>			collision_impulses;

		PxgTypedCudaBuffer<PxU32>			phases;
		PxgTypedCudaBuffer<float4>			unsortedpositions;
		PxgTypedCudaBuffer<float4>			unsortedvelocities;
		PxgTypedCudaBuffer<float4>			restArray;
		PxgTypedCudaBuffer<float4>			normal;

		PxgTypedCudaBuffer<PxReal>			density;
		PxgTypedCudaBuffer<PxReal>			staticDensity;
		PxgTypedCudaBuffer<PxVec4>			surfaceNormal;
		
		PxgTypedCudaBuffer<PxVec4>			delta;
		PxgTypedCudaBuffer<PxVec4>			curl;

		PxgTypedCudaBuffer<float4>			sorted_originalPosition_mass;
		PxgTypedCudaBuffer<PxU32>			sortedPhaseArray;

		PxgTypedCudaBuffer<PxgParticleContactInfo> particleOneWayContacts;
		PxgTypedCudaBuffer<float2>			particleOneWayForces;
		PxgTypedCudaBuffer<PxNodeIndex>		particleOneWayContactsNodeIndices;
		PxgTypedCudaBuffer<float4>			particleOneWayContactsSurfaceVelocities;
		PxgTypedCudaBuffer<PxU32>			particleOneWayContactCount;

		PxgTypedCudaBuffer<PxU32>			reverseLookup;

		PxgTypedCudaBuffer<PxU16>			phase_group_to_material_handle;
		PxgTypedCudaBuffer<PxgPBDParticleMaterialDerived> derivedPBDMaterialProperties;

		PxgTypedCudaBuffer<PxgParticleSimBuffer> user_particle_buffer; //PxgPBDParticleBuffer
		PxgTypedCudaBuffer<PxU32>			user_particle_buffer_runsum; //PxU32*
		PxgTypedCudaBuffer<PxU32>			user_particle_buffer_sorted_unique_ids; //PxU32*
		PxgTypedCudaBuffer<PxU32>			user_particle_buffer_runsum_sorted_unique_ids_original_index; //PxU32*
		PxgTypedCudaBuffer<PxgParticleClothSimBuffer> user_cloth_buffer;
		PxgTypedCudaBuffer<PxgParticleRigidSimBuffer> user_rigid_buffer;
		PxgTypedCudaBuffer<PxgParticleDiffuseSimBuffer> user_diffuse_buffer;

		PxgTypedCudaBuffer<PxU32>			attachmentRunSum;
		PxgTypedCudaBuffer<PxU32>			referencedRigidsRunsum;

		PxPinnedArray<PxgParticleSimBuffer> mHostParticleBuffers;
		PxPinnedArray<PxgParticleClothSimBuffer> mHostClothBuffers;
		PxPinnedArray<PxgParticleRigidSimBuffer> mHostRigidBuffers;
		PxPinnedArray<PxgParticleDiffuseSimBuffer> mHostDiffuseBuffers;

		PxInt32ArrayPinned					mAttachmentRunSum;
		PxInt32ArrayPinned					mParticleBufferRunSum;
		PxInt32ArrayPinned					mReferencedRigidsRunsum;

		PxInt32ArrayPinned					mParticleBufferSortedUniqueIds;
		PxInt32ArrayPinned					mParticleBufferSortedUniqueIdsOriginalIndex;
		PxFloatArrayPinned					mRandomTable;
		PxInt16ArrayPinned					mHostPhaseGroupToMaterialHandle;
	};

	class PxgParticleSystemDiffuseBuffer
	{
	public:
		PxgParticleSystemDiffuseBuffer(PxgHeapMemoryAllocatorManager* heapMemoryManager);

		// Diffuse particle data
		PxgTypedCudaBuffer<PxVec4>                      diffuse_positions;
		PxgTypedCudaBuffer<PxVec4>                      diffuse_velocities;
		PxgTypedCudaBuffer<PxVec4>                      diffuse_potentials;
		PxgTypedCudaBuffer<PxU32>                       diffuse_cell_start;
		PxgTypedCudaBuffer<PxU32>                       diffuse_cell_end;
		PxgTypedCudaBuffer<PxU32>                       diffuse_grid_particle_hash;
		PxgTypedCudaBuffer<PxU32>                       diffuse_sorted_to_unsorted_mapping;
		PxgTypedCudaBuffer<PxU32>                       diffuse_unsorted_to_sorted_mapping;
		PxgTypedCudaBuffer<PxVec4>                      diffuse_origin_pos_life_time;
		PxgTypedCudaBuffer<PxVec4>                      diffuse_sorted_pos_life_time;
		PxgTypedCudaBuffer<PxVec4>                      diffuse_sorted_origin_pos_life_time;
		PxgTypedCudaBuffer<PxVec4>                      diffuse_sorted_vel;
		PxgTypedCudaBuffer<PxgParticleContactInfo>      diffuse_one_way_contacts;
		PxgTypedCudaBuffer<PxReal>                      diffuse_one_way_forces;
		PxgTypedCudaBuffer<PxNodeIndex>                 diffuse_one_way_contacts_node_indices;
		PxgTypedCudaBuffer<PxU32>                       diffuse_one_way_contact_count;
		PxgTypedCudaBuffer<PxU32>                       diffuse_particle_count;
	};


	PX_ALIGN_PREFIX(16)
		struct PxgParticlePrimitiveContact
	{
	public:
		PxVec4		normal_pen;				//normal pen

		PxU64		rigidId;				//the corresponding rigid body node index
		PxU64		particleId;				//particle index
	}PX_ALIGN_SUFFIX(16);


	struct PxgParticleContactWriter
	{
		PxgParticlePrimitiveContact* PX_RESTRICT	particleContacts;
		PxU32*										numTotalContacts;
		PxU64*										contactSortedByParticle;
		PxU32*										tempContactByParticle;
		PxU32*										contactIndexSortedByParticle;
		PxNodeIndex*								contactByRigid;
		PxU32*										tempContactByRigid;
		PxU32*										contactIndexSortedByRigid;
		PxU32									maxContacts;

		PX_FORCE_INLINE PX_CUDA_CALLABLE void writeContact(PxU32 index, const PxVec4& normalPen,
			PxU64 compressedParticleIndex, PxNodeIndex rigidId)
		{
			if (index < maxContacts)
			{
				contactByRigid[index] = rigidId;
			    tempContactByRigid[index] = PxU32(rigidId.getInd() & 0xffffffff);
				contactIndexSortedByRigid[index] = index;

				contactSortedByParticle[index] = compressedParticleIndex;
			    tempContactByParticle[index] = PxGetParticleIndex(compressedParticleIndex);
				contactIndexSortedByParticle[index] = index;

				PxgParticlePrimitiveContact& contact = particleContacts[index];
				contact.normal_pen = normalPen;
				contact.rigidId = rigidId.getInd();
				contact.particleId = compressedParticleIndex;
			}
		}
	};
}

#endif
