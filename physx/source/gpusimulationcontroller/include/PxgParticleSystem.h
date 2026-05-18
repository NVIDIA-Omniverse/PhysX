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

#ifndef PXG_PARTICLE_SYSTEM_H
#define PXG_PARTICLE_SYSTEM_H

#include "foundation/PxVec4.h"
#include "PxParticleGpu.h"
#include "PxParticleBuffer.h"
#include "PxNodeIndex.h"

#include "PxgSimulationCoreDesc.h"
#include <vector_types.h>

namespace physx
{
#define	EMPTY_CELL	0xffffffff
#define NUM_SPRING_PER_BLOCK			4

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

	struct PxgParticlePrimitiveConstraintBlock
	{
		float4 raXn_velMultiplierW[32];
		float4 normal_errorW[32];
		//Friction tangent + invMass of the rigid body (avoids needing to read the mass)
		//Second tangent can be found by cross producting normal with fricTan0
		float4 fricTan0_invMass0[32];
		float4 raXnF0_velMultiplierW[32];
		float4 raXnF1_velMultiplierW[32];
		PxU64  rigidId[32];
		PxU64  particleId[32];
	};

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
