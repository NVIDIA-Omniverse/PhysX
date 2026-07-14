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

#ifndef PX_PARTICLE_EXT_H
#define PX_PARTICLE_EXT_H

#include "PxParticleBuffer.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

namespace ExtGpu
{

/**
\brief Structure to define user-defined particle state when constructing a new particle system.
*/
struct PxParticleBufferDesc
{
	PxVec4* positions;
	PxVec4* velocities;
	PxU32*  phases;
	PxU32	numActiveParticles;
	PxU32	maxParticles;

	PxParticleBufferDesc() : positions(NULL), velocities(NULL), phases(NULL), numActiveParticles(0), maxParticles(0) { }
};

/**
\brief Structure to define user-defined particle state when constructing a new particle system that includes diffuse particles.
*/
struct PxParticleAndDiffuseBufferDesc : public PxParticleBufferDesc
{
	PxDiffuseParticleParams diffuseParams;
	PxU32 maxDiffuseParticles;
	PxU32 maxActiveDiffuseParticles;

	PxParticleAndDiffuseBufferDesc() : PxParticleBufferDesc() { }
};

/**
\brief Creates and populates a particle buffer

\param[in] desc The particle buffer descriptor
\param[in] cudaContextManager A cuda context manager
\return A fully populated particle buffer ready to use
*/
PxParticleBuffer*						PxCreateAndPopulateParticleBuffer(const ExtGpu::PxParticleBufferDesc& desc, PxCudaContextManager* cudaContextManager);
		
/**
\brief Creates and populates a particle buffer that includes support for diffuse particles

\param[in] desc The particle buffer descriptor
\param[in] cudaContextManager A cuda context manager
\return A fully populated particle buffer ready to use
*/
PxParticleAndDiffuseBuffer*				PxCreateAndPopulateParticleAndDiffuseBuffer(const ExtGpu::PxParticleAndDiffuseBufferDesc& desc, PxCudaContextManager* cudaContextManager);

} // namespace ExtGpu

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif

