// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_PARTICLESYSTEM_CORE_H
#define DY_PARTICLESYSTEM_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxArray.h"
#include "foundation/PxMemory.h"
#include "PxParticleSystem.h"
#include "PxParticleBuffer.h"
#include "CmIDPool.h"
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
		return mParticleBuffers.size() + mParticleDiffuseBuffers.size();
	}

	//device
	PxArray<PxsParticleBuffer*>		mParticleBuffers;
	PxArray<PxsParticleBuffer*>		mParticleDiffuseBuffers;

	bool							mParticleBufferUpdate;
	bool							mParticleDiffuseBufferUpdate;

	PxParticleSystemCallback* mCallback;

	ParticleSystemCore()
	{
		PxMemSet(this, 0, sizeof(*this));
		mParticleBufferUpdate = false;
		mParticleDiffuseBufferUpdate = false;
	}

};

} // namespace Dy
} // namespace physx

#endif

