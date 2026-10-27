// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SMOOTHING_H
#define PXG_SMOOTHING_H

#include "foundation/PxUserAllocated.h"
#include "PxSmoothing.h"
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

		virtual void setSmoothing(float smoothingStrenght) PX_OVERRIDE
		{
			mPositionSmoothingDataHost.mSmoothing = smoothingStrenght;
			mDirty = true;
		}

		virtual void release() PX_OVERRIDE;

		//Replaces the former readData method
		virtual void setResultBufferHost(PxVec4* smoothedPositions) PX_OVERRIDE
		{
			mSmoothedPositions = smoothedPositions;
			allocateGPUSmoothedPositionBuffers();
			mDirty = true;
		}

		virtual void setResultBufferDevice(PxVec4* smoothedPositions) PX_OVERRIDE
		{
			if (mOwnsSmoothedPositionGPUBuffers)
				releaseGPUSmoothedPositionBuffers();
			mPositionSmoothingDataHost.mPositions = smoothedPositions;
			mDirty = true;
			mSmoothedPositions = NULL;
		}

		virtual void generateSmoothedPositions(PxGpuParticleSystem* gpuParticleSystem, PxU32 numParticles, CUstream stream) PX_OVERRIDE;

		virtual void generateSmoothedPositions(PxVec4* particlePositionsGpu, PxParticleNeighborhoodProvider& neighborhoodProvider, PxU32 numParticles, PxReal particleContactOffset, CUstream stream) PX_OVERRIDE;

		virtual PxU32 getMaxParticles() const PX_OVERRIDE
		{
			return mNumParticles;
		}

		virtual void setMaxParticles(PxU32 maxParticles) PX_OVERRIDE;

		virtual PxVec4* getSmoothedPositionsDevicePointer() const PX_OVERRIDE
		{
			return mPositionSmoothingDataHost.mPositions;
		}

		virtual void setEnabled(bool enabled) PX_OVERRIDE
		{
			mEnabled = enabled;
		}

		virtual bool isEnabled() const PX_OVERRIDE
		{
			return mEnabled;
		}
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
