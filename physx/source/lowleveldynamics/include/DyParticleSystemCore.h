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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.

#ifndef DY_PARTICLESYSTEM_CORE_H
#define DY_PARTICLESYSTEM_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxArray.h"
#include "foundation/PxMemory.h"
#include "PxParticleSystem.h"
#include "PxParticleBuffer.h"
#include "CmIDPool.h"
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "PxFLIPParticleSystem.h"
#include "PxMPMParticleSystem.h"
#endif
#include "PxParticleSolverType.h"
#include "PxSparseGridParams.h"

namespace physx
{
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

	PxParticleSolverType::Enum	solverType;

	bool					enableCCD;

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
			
	PxVec3					mWind;

	PxU32					mMaxNeighborhood; 
		
	PxArray<PxU16>			mPhaseGroupToMaterialHandle;

	void addParticleBuffer(PxParticleBuffer* particleBuffer)
	{
		if (particleBuffer->bufferIndex == 0xffffffff)
		{
			switch (particleBuffer->getConcreteType())
			{
				case (PxConcreteType::ePARTICLE_BUFFER):
				{
					particleBuffer->bufferIndex = mParticleBuffers.size();
					mParticleBuffers.pushBack(particleBuffer);
					mParticleBufferUpdate = true;
					particleBuffer->setInternalData(this);
					return;
				}

				case (PxConcreteType::ePARTICLE_DIFFUSE_BUFFER):
				{
					particleBuffer->bufferIndex = mParticleAndDiffuseBuffers.size();
					mParticleAndDiffuseBuffers.pushBack(reinterpret_cast<PxParticleAndDiffuseBuffer*>(particleBuffer));
					mParticleAndDiffuseBufferUpdate = true;
					particleBuffer->setInternalData(this);
					return;
				}

				case (PxConcreteType::ePARTICLE_CLOTH_BUFFER):
				{
					particleBuffer->bufferIndex = mClothBuffers.size();
					mClothBuffers.pushBack(reinterpret_cast<PxParticleClothBuffer*>(particleBuffer)); 
					mClothBufferUpdate = true;
					particleBuffer->setInternalData(this);
					return;
				}

				case (PxConcreteType::ePARTICLE_RIGID_BUFFER):
				{
					particleBuffer->bufferIndex = mRigidBuffers.size();
					mRigidBuffers.pushBack(reinterpret_cast<PxParticleRigidBuffer*>(particleBuffer));
					mRigidBufferUpdate = true;
					particleBuffer->setInternalData(this);
					return;
				}

				default:
				{
					PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "addParticleBuffer : Error, this buffer does not have a valid type!");
					return;
				}
					
			}
		}
		else
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "addParticleBuffer : Error, this buffer cannot be added to multiple particle systems!");
		}
	}

	void removeParticleBuffer(PxParticleBuffer* particleBuffer)
	{
		const PxU32 index = particleBuffer->bufferIndex;

		switch (particleBuffer->getConcreteType())
		{
			case (PxConcreteType::ePARTICLE_BUFFER):
			{
				if (index < mParticleBuffers.size())
				{ 
					mParticleBuffers.replaceWithLast(particleBuffer->bufferIndex);
					if (mParticleBuffers.size() > index)
						mParticleBuffers[index]->bufferIndex = index;
					mParticleBufferUpdate = true;
					particleBuffer->bufferIndex = 0xffffffff;
					particleBuffer->onParticleSystemDestroy();
				}
				return;
			}

			case (PxConcreteType::ePARTICLE_DIFFUSE_BUFFER):
			{
				if (index < mParticleAndDiffuseBuffers.size())
				{ 
					mParticleAndDiffuseBuffers.replaceWithLast(particleBuffer->bufferIndex);
					if (mParticleAndDiffuseBuffers.size() > index)
						mParticleAndDiffuseBuffers[index]->bufferIndex = index;

					mParticleAndDiffuseBufferUpdate = true;
					particleBuffer->bufferIndex = 0xffffffff;
					particleBuffer->onParticleSystemDestroy();
				}
				return;
			}

			case (PxConcreteType::ePARTICLE_CLOTH_BUFFER):
			{
				if (index < mClothBuffers.size())
				{
					mClothBuffers.replaceWithLast(particleBuffer->bufferIndex);
					if (mClothBuffers.size() > index)
						mClothBuffers[index]->bufferIndex = index;
					mClothBufferUpdate = true;
					particleBuffer->bufferIndex = 0xffffffff;
					particleBuffer->onParticleSystemDestroy();
				}
				return;
			}

			case (PxConcreteType::ePARTICLE_RIGID_BUFFER):
			{
				if (index < mParticleBuffers.size())
				{
					mRigidBuffers.replaceWithLast(particleBuffer->bufferIndex);
					if (mRigidBuffers.size() > index)
						mRigidBuffers[index]->bufferIndex = index;

					mRigidBufferUpdate = true;
					particleBuffer->bufferIndex = 0xffffffff;
					particleBuffer->onParticleSystemDestroy();
				}
				return;
			}

			default:
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "removeParticleBuffer : Error, this buffer does not have a valid type!");
				return;
			}

		}
	}

	PxU32 getNumUserBuffers() const { return mParticleBuffers.size() + mClothBuffers.size() + mRigidBuffers.size() + mParticleAndDiffuseBuffers.size(); }
	//device
	PxArray<PxParticleBuffer*>				mParticleBuffers;
	PxArray<PxParticleClothBuffer*>			mClothBuffers;
	PxArray<PxParticleRigidBuffer*>			mRigidBuffers;
	PxArray<PxParticleAndDiffuseBuffer*>	mParticleAndDiffuseBuffers;

	bool							mParticleBufferUpdate;
	bool							mClothBufferUpdate;
	bool							mRigidBufferUpdate;
	bool							mParticleAndDiffuseBufferUpdate;

	PxParticleSystemCallback* mCallback;

	ParticleSystemCore()
	{
		PxMemSet(this, 0, sizeof(*this));
		mParticleBufferUpdate = false;
		mClothBufferUpdate = false;
		mRigidBufferUpdate = false;
		mParticleAndDiffuseBufferUpdate = false;
	}

	~ParticleSystemCore()
	{
		for(PxU32 i = 0; i < mParticleBuffers.size(); ++i)
		{ 
			mParticleBuffers[i]->onParticleSystemDestroy();
		}

		for (PxU32 i = 0; i < mClothBuffers.size(); ++i)
		{
			mClothBuffers[i]->onParticleSystemDestroy();
		}

		for (PxU32 i = 0; i < mRigidBuffers.size(); ++i)
		{
			mRigidBuffers[i]->onParticleSystemDestroy();
		}

		for (PxU32 i = 0; i < mParticleAndDiffuseBuffers.size(); ++i)
		{
			mParticleAndDiffuseBuffers[i]->onParticleSystemDestroy();
		}
	}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	//Leave these members at the end to remain binary compatible with public builds
	PxFLIPParams			flipParams;
	PxMPMParams				mpmParams;
#endif

};

} // namespace Dy
} // namespace physx

#endif

