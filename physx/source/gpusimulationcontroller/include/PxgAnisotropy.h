// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_ANISOTROPY_H
#define PXG_ANISOTROPY_H


#include "PxAnisotropy.h"
#include "PxgAnisotropyData.h"
#include "foundation/PxUserAllocated.h"
#include "PxgKernelLauncher.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX

	class PxgAnisotropyGenerator : public PxAnisotropyGenerator, public PxUserAllocated
	{
		PxgKernelLauncher mKernelLauncher;
		PxAnisotropyData mAnisotropyDataHost;
		PxAnisotropyData* mAnisotropyDataPerParticleSystemDevice;
		PxU32 mNumParticles;
		bool mDirty;
		bool mOwnsAnisotropyGPUBuffers;
		PxVec4* mAnisotropy1;
		PxVec4* mAnisotropy2;
		PxVec4* mAnisotropy3;
		bool mEnabled;

		void releaseGPUAnisotropyBuffers();

		void allocateGPUAnisotropyBuffers();

	public:

		PxgAnisotropyGenerator(PxgKernelLauncher& cudaContextManager, PxU32 maxNumParticles, PxReal anisotropyScale, PxReal minAnisotropy, PxReal maxAnisotropy);

		virtual ~PxgAnisotropyGenerator() { }

		virtual void setAnisotropyMax(float maxAnisotropy) PX_OVERRIDE
		{
			mAnisotropyDataHost.mAnisotropyMax = maxAnisotropy;
			mDirty = true;
		}

		virtual void setAnisotropyMin(float minAnisotropy) PX_OVERRIDE
		{
			mAnisotropyDataHost.mAnisotropyMin = minAnisotropy;
			mDirty = true;
		}

		virtual void setAnisotropyScale(float anisotropyScale) PX_OVERRIDE
		{
			mAnisotropyDataHost.mAnisotropy = anisotropyScale;
			mDirty = true;
		}

		virtual void release() PX_OVERRIDE;

		virtual void setResultBufferHost(PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3) PX_OVERRIDE;

		virtual void setResultBufferDevice(PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3) PX_OVERRIDE;

		virtual void generateAnisotropy(PxGpuParticleSystem* gpuParticleSystem, PxU32 numParticles, CUstream stream) PX_OVERRIDE;

		virtual void generateAnisotropy(PxVec4* particlePositionsGpu, PxParticleNeighborhoodProvider& neighborhoodProvider, PxU32 numParticles, PxReal particleContactOffset, CUstream stream) PX_OVERRIDE;

		virtual PxU32 getMaxParticles() const PX_OVERRIDE
		{
			return mNumParticles;
		}

		virtual void setMaxParticles(PxU32 maxParticles) PX_OVERRIDE;

		virtual PxVec4* getAnisotropy1DevicePointer() const PX_OVERRIDE
		{
			return mAnisotropyDataHost.mAnisotropy_q1;
		}

		virtual PxVec4* getAnisotropy2DevicePointer() const PX_OVERRIDE
		{
			return mAnisotropyDataHost.mAnisotropy_q2;
		}

		virtual PxVec4* getAnisotropy3DevicePointer() const PX_OVERRIDE
		{
			return mAnisotropyDataHost.mAnisotropy_q3;
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
