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

#ifndef PXG_SMOOTHING_H
#define PXG_SMOOTHING_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"

#include "foundation/PxArray.h"

#include "PxSmoothing.h"
#include "PxgKernelWrangler.h"
#include "PxgAnisotropyData.h"
#include "PxgKernelLauncher.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX

	class PxgSmoothedPositionGenerator : public PxSmoothedPositionGenerator, public PxUserAllocated
	{
	private:
		PxgKernelLauncher mKernelLauncher;
		PxSmoothedPositionData mPositionSmoothingDataHost;
		PxSmoothedPositionData* mPositionSmoothingDataPerParticleSystemDevice;
		PxU32 mNumParticles;
		bool mDirty;
		bool mOwnsSmoothedPositionGPUBuffers;
		PxVec4* mSmoothedPositions;
		bool mEnabled;

		void releaseGPUSmoothedPositionBuffers();

		void allocateGPUSmoothedPositionBuffers();

	public:
		PxgSmoothedPositionGenerator(PxgKernelLauncher& cudaContextManager, PxU32 maxNumParticles, PxReal smoothingStrenght);

		virtual ~PxgSmoothedPositionGenerator() { }

		virtual void setSmoothing(float smoothingStrenght)
		{
			mPositionSmoothingDataHost.mSmoothing = smoothingStrenght;
			mDirty = true;
		}

		virtual void release();

		//Replaces the former readData method
		virtual void setResultBufferHost(PxVec4* smoothedPositions)
		{
			mSmoothedPositions = smoothedPositions;
			allocateGPUSmoothedPositionBuffers();
			mDirty = true;
		}

		virtual void setResultBufferDevice(PxVec4* smoothedPositions)
		{
			if (mOwnsSmoothedPositionGPUBuffers)
				releaseGPUSmoothedPositionBuffers();
			mPositionSmoothingDataHost.mPositions = smoothedPositions;
			mDirty = true;
			mSmoothedPositions = NULL;
		}

		virtual void generateSmoothedPositions(PxGpuParticleSystem* gpuParticleSystem, PxU32 numParticles, CUstream stream);

		virtual void generateSmoothedPositions(PxVec4* particlePositionsGpu, PxParticleNeighborhoodProvider& neighborhoodProvider, PxU32 numParticles, PxReal particleContactOffset, CUstream stream);

		virtual PxU32 getMaxParticles() const
		{
			return mNumParticles;
		}

		virtual void setMaxParticles(PxU32 maxParticles);

		virtual PxVec4* getSmoothedPositionsDevicePointer() const
		{
			return mPositionSmoothingDataHost.mPositions;
		}

		virtual void setEnabled(bool enabled)
		{
			mEnabled = enabled;
		}

		virtual bool isEnabled() const
		{
			return mEnabled;
		}
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
