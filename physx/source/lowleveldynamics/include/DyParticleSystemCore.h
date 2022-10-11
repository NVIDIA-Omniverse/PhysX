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
#include "PxCustomParticleSystemSolverCallback.h"
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
			
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
			PxFLIPParams			flipParams;
			PxMPMParams				mpmParams;
#endif

			PxReal					restOffset;
			PxReal					particleContactOffset;
			PxReal					particleContactOffset_prev;
			PxReal					solidRestOffset;
			PxReal					fluidRestOffset;
			PxReal					fluidRestOffset_prev;

			PxReal					fluidBoundaryDensityScale;

			PxReal					maxDepenetrationVelocity;
			PxReal					maxVelocity;

			PxVec3					periodicBoundary;

			PxParticleFlags			mFlags;
			
			PxVec3					mWind;

			PxU32					mMaxParticles;
		
			PxU32					mMaxNeighborhood; 
		
			PxArray<PxU16>			mPhaseGroupToMaterialHandle;

			PX_FORCE_INLINE void addParticleBuffer(PxUserParticleBuffer* particleBuffer)
			{
				if (particleBuffer->bufferIndex == 0xffffffff)
				{
					particleBuffer->bufferIndex = mParticleBuffers.size();
					mParticleBuffers.pushBack(particleBuffer);
					mParticleBufferUpdate = true;
					particleBuffer->setInternalData(this);
				}
				else
				{
					PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "addParticleBuffer : Error, this buffer cannot be added to multiple particle systems!");
				}
			}

			PX_FORCE_INLINE void removeParticleBuffer(PxUserParticleBuffer* particleBuffer)
			{
				const PxU32 index = particleBuffer->bufferIndex;

				if (index < mParticleBuffers.size())
				{ 
					mParticleBuffers.replaceWithLast(particleBuffer->bufferIndex);
					if (mParticleBuffers.size() > index)
						mParticleBuffers[index]->bufferIndex = index;
					mParticleBufferUpdate = true;
					particleBuffer->bufferIndex = 0xffffffff;
					particleBuffer->onParticleSystemDestroy();
				}
			}

			PX_FORCE_INLINE void addParticleClothBuffer(PxUserParticleClothBuffer* clothBuffer) 
			{ 
				if (clothBuffer->bufferIndex == 0xffffffff)
				{
					clothBuffer->bufferIndex = mClothBuffers.size();
					mClothBuffers.pushBack(clothBuffer); 
					mClothBufferUpdate = true;
					clothBuffer->setInternalData(this);
				}
				else
				{
					PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "addClothBuffer : Error, this buffer cannot be added to multiple particle systems!");
				}
			}

			PX_FORCE_INLINE void removeParticleClothBuffer(PxUserParticleClothBuffer* clothBuffer)
			{
				const PxU32 index = clothBuffer->bufferIndex;

				if (index < mClothBuffers.size())
				{
					mClothBuffers.replaceWithLast(index);
					if (mClothBuffers.size() > index)
						mClothBuffers[index]->bufferIndex = index;
					mClothBufferUpdate = true;
					clothBuffer->bufferIndex = 0xffffffff;
					clothBuffer->onParticleSystemDestroy();
				}
			}

			PX_FORCE_INLINE void addParticleRigidBuffer(PxUserParticleRigidBuffer* rigidBuffer)
			{
				if (rigidBuffer->bufferIndex == 0xffffffff)
				{
					rigidBuffer->bufferIndex = mRigidBuffers.size();
					mRigidBuffers.pushBack(rigidBuffer);
					mRigidBufferUpdate = true;
					rigidBuffer->setInternalData(this);
				}
				else
				{
					PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "addRigidBuffer : Error, this buffer cannot be added to multiple particle systems!");
				}
			}

			PX_FORCE_INLINE void removeParticleRigidBuffer(PxUserParticleRigidBuffer* rigidBuffer)
			{
				const PxU32 index = rigidBuffer->bufferIndex;

				if (index < mParticleBuffers.size())
				{
					mRigidBuffers.replaceWithLast(rigidBuffer->bufferIndex);
					if (mRigidBuffers.size() > index)
						mRigidBuffers[index]->bufferIndex = index;

					mRigidBufferUpdate = true;
					rigidBuffer->bufferIndex = 0xffffffff;
					rigidBuffer->onParticleSystemDestroy();
				}
			}

			PX_FORCE_INLINE void addParticleAndDiffuseBuffer(PxUserParticleAndDiffuseBuffer* diffuseParticleBuffer)
			{
				if (diffuseParticleBuffer->bufferIndex == 0xffffffff)
				{
					diffuseParticleBuffer->bufferIndex = mParticleAndDiffuseBuffers.size();
					mParticleAndDiffuseBuffers.pushBack(diffuseParticleBuffer);
					mParticleAndDiffuseBufferUpdate = true;
					diffuseParticleBuffer->setInternalData(this);
				}
				else
				{
					PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "addParticleAndDiffuseBuffer : Error, this buffer cannot be added to multiple particle systems!");
				}
			}

			PX_FORCE_INLINE void removeParticleAndDiffuseBuffer(PxUserParticleAndDiffuseBuffer* diffuseParticleBuffer)
			{
				const PxU32 index = diffuseParticleBuffer->bufferIndex;

				if (index < mParticleAndDiffuseBuffers.size())
				{ 
					mParticleAndDiffuseBuffers.replaceWithLast(diffuseParticleBuffer->bufferIndex);
					if (mParticleAndDiffuseBuffers.size() > index)
						mParticleAndDiffuseBuffers[index]->bufferIndex = index;

					mParticleAndDiffuseBufferUpdate = true;
					diffuseParticleBuffer->bufferIndex = 0xffffffff;
					diffuseParticleBuffer->onParticleSystemDestroy();
				}
			}

			PxU32 getNumUserBuffers() const { return mParticleBuffers.size() + mClothBuffers.size() + mRigidBuffers.size() + mParticleAndDiffuseBuffers.size(); }
			//device
			PxArray<PxUserParticleBuffer*>				mParticleBuffers;
			PxArray<PxUserParticleClothBuffer*>			mClothBuffers;
			PxArray<PxUserParticleRigidBuffer*>			mRigidBuffers;
			PxArray<PxUserParticleAndDiffuseBuffer*>	mParticleAndDiffuseBuffers;

			bool							mParticleBufferUpdate;
			bool							mClothBufferUpdate;
			bool							mRigidBufferUpdate;
			bool							mParticleAndDiffuseBufferUpdate;

			PxParticleSystemCallback* mCallback;
			PxCustomParticleSystemSolverCallback*	mSolverCallback;

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

		};
	}
}

#endif

