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

#include "PxParticleBuffer.h"
#include "extensions/PxCudaHelpersExt.h"
#include "extensions/PxParticleExt.h"

#include "foundation/PxUserAllocated.h"
#include "PxPhysics.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"

namespace physx
{
namespace ExtGpu
{

void PxDmaDataToDevice(PxCudaContextManager* cudaContextManager, PxParticleBuffer* particleBuffer, const PxParticleBufferDesc& desc)
{
#if PX_SUPPORT_GPU_PHYSX
	cudaContextManager->acquireContext();

	PxVec4* posInvMass = particleBuffer->getPositionInvMasses();
	PxVec4* velocities = particleBuffer->getVelocities();
	PxU32* phases = particleBuffer->getPhases();

	PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

	cudaContext->memcpyHtoDAsync(CUdeviceptr(posInvMass), desc.positions, desc.numActiveParticles * sizeof(PxVec4), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(velocities), desc.velocities, desc.numActiveParticles * sizeof(PxVec4), 0);
	cudaContext->memcpyHtoDAsync(CUdeviceptr(phases), desc.phases, desc.numActiveParticles * sizeof(PxU32), 0);

	particleBuffer->setNbActiveParticles(desc.numActiveParticles);

	cudaContext->streamSynchronize(0);

	cudaContextManager->releaseContext();
#else
	PX_UNUSED(cudaContextManager);
	PX_UNUSED(particleBuffer);
	PX_UNUSED(desc);
#endif
}

PxParticleBuffer* PxCreateAndPopulateParticleBuffer(const PxParticleBufferDesc& desc, PxCudaContextManager* cudaContextManager)
{
	PxParticleBuffer* particleBuffer = PxGetPhysics().createParticleBuffer(desc.maxParticles, cudaContextManager);
	PxDmaDataToDevice(cudaContextManager, particleBuffer, desc);
	return particleBuffer;
}

PxParticleAndDiffuseBuffer* PxCreateAndPopulateParticleAndDiffuseBuffer(const PxParticleAndDiffuseBufferDesc& desc, PxCudaContextManager* cudaContextManager)
{
	PxParticleAndDiffuseBuffer* particleBuffer = PxGetPhysics().createParticleAndDiffuseBuffer(desc.maxParticles, desc.maxDiffuseParticles, cudaContextManager);
	PxDmaDataToDevice(cudaContextManager, particleBuffer, desc);
	particleBuffer->setMaxActiveDiffuseParticles(desc.maxActiveDiffuseParticles);
	return particleBuffer;
}


} //namespace ExtGpu
} //namespace physx
