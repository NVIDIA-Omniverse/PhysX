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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

#ifndef DY_PARTICLESYSTEM_CORE_H
#define DY_PARTICLESYSTEM_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxArray.h"
#include "foundation/PxMemory.h"
#include "PxParticleSystem.h"
#include "PxParticleBuffer.h"
#include "CmIDPool.h"
#include "PxParticleSolverType.h"
#include "PxSparseGridParams.h"

namespace physx
{
	class PxsParticleBuffer;

namespace Dy
{	

class ParticleSystemCore
{
public:
	
	PxReal					sleepThreshold;
	PxReal					freezeThreshold;
	PxReal					wakeCounter;

	PxU32					gridSizeX;
	PxU32					gridSizeY;
	PxU32					gridSizeZ;

	PxU16					solverIterationCounts;

	PxSparseGridParams		sparseGridParams;

	PxReal					restOffset;
	PxReal					particleContactOffset;
	PxReal					particleContactOffset_prev;
	PxReal					solidRestOffset;
	PxReal					fluidRestOffset;
	PxReal					fluidRestOffset_prev;

	PxReal					fluidBoundaryDensityScale;

	PxReal					maxDepenetrationVelocity;
	PxReal					maxVelocity;

	PxParticleFlags			mFlags;
	PxParticleLockFlags		mLockFlags;
			
	PxVec3					mWind;

	PxU32					mMaxNeighborhood;
	PxReal					mNeighborhoodScale;
		
	PxArray<PxU16>			mPhaseGroupToMaterialHandle;
	PxArray<PxU16>			mUniqueMaterialHandles; //just for reporting

	PxU32 getNumUserBuffers() const
	{ 
		return mParticleBuffers.size() + 
			   mParticleDiffuseBuffers.size() +
			   mParticleClothBuffers.size() + 
			   mParticleRigidBuffers.size();
	}

	//device
	PxArray<PxsParticleBuffer*>		mParticleBuffers;
	PxArray<PxsParticleBuffer*>		mParticleDiffuseBuffers;
	PxArray<PxsParticleBuffer*>		mParticleClothBuffers;
	PxArray<PxsParticleBuffer*>		mParticleRigidBuffers;

	bool							mParticleBufferUpdate;
	bool							mParticleDiffuseBufferUpdate;
	bool							mParticleClothBufferUpdate;
	bool							mParticleRigidBufferUpdate;

	PxParticleSystemCallback* mCallback;

	ParticleSystemCore()
	{
		PxMemSet(this, 0, sizeof(*this));
		mParticleBufferUpdate = false;
		mParticleDiffuseBufferUpdate = false;
		mParticleClothBufferUpdate = false;
		mParticleRigidBufferUpdate = false;
	}

};

} // namespace Dy
} // namespace physx

#endif

