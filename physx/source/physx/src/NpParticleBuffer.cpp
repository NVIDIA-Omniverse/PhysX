// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "NpParticleBuffer.h"
#include "NpPBDParticleSystem.h"
#include "NpFactory.h"
#include "PxvGlobals.h"
#include "PxPhysXGpu.h"

using namespace physx;

namespace physx
{
	///////////////////////////////////////////////////////////////////////////////////////

	NpParticleBuffer::NpParticleBuffer(PxU32 maxNumParticles, PxCudaContextManager& cudaContextManager)
		: NpParticleBufferBase<PxParticleBuffer>(PxConcreteType::ePARTICLE_BUFFER)
	{
		PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);
		PX_ASSERT(physxGpu);
		mGpuBuffer = physxGpu->createParticleBuffer(maxNumParticles, cudaContextManager);
	}

	void NpParticleBuffer::release()
	{
		if (mParticleSystem)
		{
			mParticleSystem->removeParticleBuffer(this);
		}
		PX_RELEASE(mGpuBuffer);

		PX_ASSERT(!isAPIWriteForbidden());
		NpDestroyParticleBuffer(this);
	}

	///////////////////////////////////////////////////////////////////////////////////////

	NpParticleAndDiffuseBuffer::NpParticleAndDiffuseBuffer(PxU32 maxNumParticles,
		PxU32 maxNumDiffuseParticles, PxCudaContextManager& cudaContextManager)
		: NpParticleBufferBase<PxParticleAndDiffuseBuffer>(PxConcreteType::ePARTICLE_DIFFUSE_BUFFER)
	{
		PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);
		PX_ASSERT(physxGpu);
		mGpuBuffer = physxGpu->createParticleAndDiffuseBuffer(maxNumParticles, maxNumDiffuseParticles, cudaContextManager);
	}

	void NpParticleAndDiffuseBuffer::release()
	{
		if (mParticleSystem)
		{
			mParticleSystem->removeParticleBuffer(this);
		}

		//need to destroy PxDiffuseParticleParams ovd representation before releasing ll object.
		OMNI_PVD_DESTROY(OMNI_PVD_CONTEXT_HANDLE, PxDiffuseParticleParams, getDiffuseParticleParamsRef());

		PX_RELEASE(mGpuBuffer);

		PX_ASSERT(!isAPIWriteForbidden());
		NpDestroyParticleBuffer(this);
	}

	///////////////////////////////////////////////////////////////////////////////////////

} // physx

#endif //PX_SUPPORT_GPU_PHYSX
