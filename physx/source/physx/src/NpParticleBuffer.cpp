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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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
