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


#ifndef PXD_PARTICLESYSTEM_H
#define PXD_PARTICLESYSTEM_H

#include "foundation/PxSimpleTypes.h"
#include "DyParticleSystemCore.h"
#include "PxvGeometry.h"

#define MAX_SPARSEGRID_DIM 1024
#define MIN_SPARSEGRID_ID -512
#define MAX_SPARSEGRID_ID 511

namespace physx
{
	namespace Sc
	{
		class ParticleSystemSim;
	}


	namespace Dy
	{
		typedef size_t ParticleSystemHandle;

		class ParticleSystemCore;

		struct ParticleSystemFlag
		{
			enum Enum
			{
				eUPDATE_PARAMS						= 1 << 1,
				eUPDATE_MATERIAL					= 1 << 2,
				eUPDATE_PHASE						= 1 << 3,
				eUPDATE_ACTIVE_PARTICLECOUNT		= 1 << 4,
				eENABLE_GPU_DATA_SYNC				= 1 << 5,
			};
		};


		class ParticleSystem
		{
			PX_NOCOPY(ParticleSystem)
		public:

			ParticleSystem(Dy::ParticleSystemCore& core) : mCore(core), mShapeCore(NULL),
				mElementId(0xffffffff), mGpuRemapId(0xffffffff)
			{
				mFlag = 0;
			}

			~ParticleSystem() {}

			PX_FORCE_INLINE void setShapeCore(PxsShapeCore* shapeCore)
			{
				mShapeCore = shapeCore;
			}

			PX_FORCE_INLINE void setGpuRemapId(const PxU32 remapId)
			{
				mGpuRemapId = remapId;
				PxParticleSystemGeometryLL& geom = mShapeCore->mGeometry.get<PxParticleSystemGeometryLL>();
				geom.materialsLL.gpuRemapId = remapId;
			}

			PX_FORCE_INLINE PxU32				getGpuRemapId()				const	{ return mGpuRemapId; }

			PX_FORCE_INLINE void				setElementId(const PxU32 elementId)	{ mElementId = elementId; }
			PX_FORCE_INLINE PxU32				getElementId()						{ return mElementId; }

			PX_FORCE_INLINE ParticleSystemCore&	getCore()					const	{ return mCore; }
			PX_FORCE_INLINE PxsShapeCore&		getShapeCore()						{ return *mShapeCore; }

			PX_FORCE_INLINE PxU16				getIterationCounts()				{ return mCore.solverIterationCounts; }

			PxU32			mFlag;

		private:
			ParticleSystemCore&			mCore;
			PxsShapeCore*				mShapeCore;
			PxU32						mElementId; //this is used for the bound array
			PxU32						mGpuRemapId;
			
		};

		struct ParticleSystemSolverDesc
		{
			ParticleSystem*		particleSystem;
		};

		PX_FORCE_INLINE ParticleSystem* getParticleSystem(ParticleSystemHandle handle)
		{
			return reinterpret_cast<ParticleSystem*>(handle);
		}

	}
}

#endif
