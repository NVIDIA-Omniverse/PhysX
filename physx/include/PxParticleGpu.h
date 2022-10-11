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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_GPU_PARTICLE_SYSTEM_H
#define PX_GPU_PARTICLE_SYSTEM_H
/** \addtogroup physics
@{ */

#include "PxParticleSystem.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
	/**
		Common material properties for particles.
		Accessed by either integration or particle-rigid collisions
	*/
	struct PxsParticleMaterialData
	{
		PxReal			friction;			 //4
		PxReal			damping;			 //8	
		PxReal			adhesion;			 //12
		PxReal			gravityScale;		 //16
		PxReal			adhesionRadiusScale; //20
	};
#if !PX_DOXYGEN
} // namespace physx
#endif

#if PX_SUPPORT_GPU_PHYSX

struct float4;

PX_CUDA_CALLABLE inline physx::PxU32 PxGetGroup(physx::PxU32 phase) { return phase & physx::PxParticlePhaseFlag::eParticlePhaseGroupMask; }
PX_CUDA_CALLABLE inline bool PxGetFluid(physx::PxU32 phase) { return (phase & physx::PxParticlePhaseFlag::eParticlePhaseFluid) != 0; }
PX_CUDA_CALLABLE inline bool PxGetSelfCollide(physx::PxU32 phase) { return (phase & physx::PxParticlePhaseFlag::eParticlePhaseSelfCollide) != 0; }
PX_CUDA_CALLABLE inline bool PxGetSelfCollideFilter(physx::PxU32 phase) { return (phase & physx::PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter) != 0; }

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxNeighborhoodIterator
	{
		const PxU32* PX_RESTRICT mCollisionIndex;
		PxU32 mMaxParticles;
	public:
		PX_CUDA_CALLABLE PxNeighborhoodIterator(const PxU32* PX_RESTRICT collisionIndex, const PxU32 maxParticles) :
			mCollisionIndex(collisionIndex), mMaxParticles(maxParticles)
		{
		}

		PX_CUDA_CALLABLE PxU32 getNextIndex()
		{
			PxU32 result = *mCollisionIndex;
			mCollisionIndex += mMaxParticles;
			return result;
		}

		PX_INLINE PxNeighborhoodIterator(const PxNeighborhoodIterator& params)
		{
			mCollisionIndex = params.mCollisionIndex;
			mMaxParticles = params.mMaxParticles;
		}

		PX_INLINE void operator = (const PxNeighborhoodIterator& params)
		{
			mCollisionIndex = params.mCollisionIndex;
			mMaxParticles = params.mMaxParticles;
		}
	};

	struct PxGpuParticleData
	{
		PxVec3	mPeriod;

		PxU32	mGridSizeX;
		PxU32	mGridSizeY;
		PxU32	mGridSizeZ;

		PxReal	mParticleContactDistance;
		PxReal	mParticleContactDistanceInv;
		PxReal	mParticleContactDistanceSq;

		PxU32	mNumParticles;
		PxU32	mMaxParticles;
		PxU32	mMaxNeighborhood;
		PxU32	mMaxDiffuseParticles;
		PxU32	mNumParticleBuffers;
	};

	class PxGpuParticleSystem
	{
	public:
		PX_FORCE_INLINE PxU32 getNumCells() { return mCommonData.mGridSizeX * mCommonData.mGridSizeY * mCommonData.mGridSizeZ; }
		/*
			** Unsorted particle state buffers **
		*/
		//GPU pointer to unsorted particle positions
		float4* mUnsortedPositions_InvMass;
		//GPU pointer to unsorted particle velocities
		float4*	mUnsortedVelocities;
		//GPU pointer to unsorted particle phase array
		PxU32*	mUnsortedPhaseArray;


		/*
			** Sorted particle state buffers. Sorted by hash table **
		*/
		//GPU pointer to sorted particle positions
		float4* mSortedPositions_InvMass;
		//GPU pointer to sorted particle velocities
		float4* mSortedVelocities;
		//GPU pointer to sorted particle phase array
		PxU32*	mSortedPhaseArray;


		/*
			** Mappings to/from sorted particle states**
		*/

		//GPU pointer to the mapping from unsortedParticle ID to sorted particle ID
		PxU32* mUnsortedToSortedMapping;
		//GPU pointer to the mapping from sorted particle ID to unsorted particle ID
		PxU32* mSortedToUnsortedMapping;

		/*
			** Neighborhood information**
		*/

		//Per-particle neighborhood count
		PxU32*	mParticleSelfCollisionCount;
		//Set of sorted particle indices per neighbor
		PxU32*	mCollisionIndex;

		PxsParticleMaterialData*		mParticleMaterials;

		PxGpuParticleData mCommonData;


		PX_CUDA_CALLABLE PxNeighborhoodIterator getIterator(const PxU32 particleId) const
		{
			return PxNeighborhoodIterator(mCollisionIndex + particleId, mCommonData.mMaxParticles);
		}

	};

	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif

/** @} */
#endif

