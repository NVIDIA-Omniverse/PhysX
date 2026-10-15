// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

		PxReal		mPenBiasClamp; // = -maxDepenetrationVelocity

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

		// Penetration relative to the pair rest distance, which contact generation incorporates. The other
		// contact types report a raw distance and apply the rest distance in the contact prep.
		float4 mNormal_PenW;
	}PX_ALIGN_SUFFIX(16);

	// Contact block for particle-rigid pairs. Field names match
	// PxgRigidDbContactBlock so PxgRigidPart::readContactPrep templates
	// over both. Particle side is a single vertex, filled inline by the solver.
	struct PxgRigidParticleContactBlock
	{
		float4 raXn_resp[32];
		float4 raXnF0_resp[32];
		float4 raXnF1_resp[32];
		float4 normal_errorW[32];
		// Second tangent is normal x fricTan0.
		float4 fricTan0_invMass0[32];
		PxReal penBiasClampRigid[32]; // Not supported for static triangle meshes / height fields
	};

	struct PxgParticleSimBuffer
	{
		float4* 					mPositionInvMasses;
		float4* 					mVelocities;
		float4* 					mRestPositions;
		PxU32*						mPhases;
		PxU32						mNumActiveParticles;

		PxU32						mFlags;
		PxU32						mDiffuseParticleBufferIndex;
		PxU32						mUniqueId; //Remains unchanged over the whole lifetime of a buffer
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
		
		
		// Per-iteration solve scratch despite the name, the particle analog of
		// PxgSoftBody::mSimDelta in sorted particle order. Every iteration
		// ps_updateParticleLaunch consumes and zeroes it, updating mSortedVelocities
		// always and mSortedPositions_InvMass plus mSortedDeltaP in position
		// iterations only.
		float4*						mAccumDeltaP;				//32	80
		// Accumulated position delta since step start, in sorted particle order.
		// Advances in position iterations only and stays frozen through velocity
		// iterations. The contact solves read it for computing the separation.
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

		PxgParticleDiffuseSimBuffer*	mDiffuseSimBuffers;

		PxU32						mNumDiffuseBuffers;
				
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

	struct PxgRigidParticleContactWriter
	{
		PxgDeformablePairInfo* PX_RESTRICT			particleContactInfos;
		// Penetration is relative to the pair rest distance, which contact generation incorporates. The
		// other contact types report a raw distance and apply the rest distance in the contact prep.
		float4* PX_RESTRICT							particleContactNormalPens;
		PxU32*										numTotalContacts;
		PxU64*										contactSortedByParticle;
		PxU32*										tempContactByParticle;
		PxU32*										contactIndexSortedByParticle;
		PxNodeIndex*								contactByRigid;
		PxU32*										tempContactByRigid;
		PxU32*										contactIndexSortedByRigid;
		PxU32										maxContacts;

		PX_FORCE_INLINE PX_CUDA_CALLABLE void writeContact(PxU32 index, const PxVec4& normalPen,
			PxNodeIndex rigidId, PxU64 particleRef)
		{
			if (index < maxContacts)
			{
				contactByRigid[index] = rigidId;
			    tempContactByRigid[index] = PxgGetContactSortKey(PxgEncodeRigidNode(rigidId));
				contactIndexSortedByRigid[index] = index;

				contactSortedByParticle[index] = particleRef;
			    tempContactByParticle[index] = PxgGetContactSortKey(particleRef);
				contactIndexSortedByParticle[index] = index;

				particleContactInfos[index].ref0 = PxgEncodeRigidNode(rigidId);
				particleContactInfos[index].ref1 = particleRef;
				// Field by field, since make_float4 is device only and this writer also compiles on the host.
				float4& outNormalPen = particleContactNormalPens[index];
				outNormalPen.x = normalPen.x;
				outNormalPen.y = normalPen.y;
				outNormalPen.z = normalPen.z;
				outNormalPen.w = normalPen.w;
			}
		}
	};
}

#endif
