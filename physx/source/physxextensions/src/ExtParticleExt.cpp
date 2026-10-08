// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
