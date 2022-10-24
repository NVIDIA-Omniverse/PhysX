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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_EXT_MD_H
#define PX_EXT_MD_H

#include "foundation/PxSimpleTypes.h"
#include "PxParticleGpu.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxsCustomMaterialData;

namespace ExtGpu
{
/**
\brief Defines interaction properties for Lennard Jones molecular dynamics simulations
*/
struct PxLennardJonesMaterialData
{
	PxReal cutDistance;
	PxReal LennardJonesEpsilon;
	PxReal LennardJonesSigma;
};

#if PX_SUPPORT_GPU_PHYSX

/**
\brief Contains the simulation state for a Lennard Jones molecular dynamics simulation
*/
struct PxLennardJonesSystemData
{
	float4* sortedForces;

	PxReal* sortedMeasurements;
	float4* unsortedMeasurements;

	PxU32	materialID;
};

/**
\brief Computes the forces (using the Lennard Jones model) that act between molecules

\param[in] particleSystem Pointer to the general purpose particle system
\param[in] sysData The Lennard Jones simulation state
\param[in] numParticles The number of particles
\param[in] stream The stream on which the work is scheduled to execute
*/
void PxComputeLennardJonedForces(
	PxGpuParticleSystem* particleSystem,
	ExtGpu::PxLennardJonesSystemData* sysData,
	const PxU32 numParticles,
	CUstream stream);

/**
\brief Applies explicit integration on a point cloud given the forces that act on the particles

\param[in] particleSystem Pointer to the general purpose particle system
\param[in] sysData The Lennard Jones simulation state
\param[in] dt The timestep
\param[in] numParticles The number of particles
\param[in] stream The stream on which the work is scheduled to execute
*/
void PxExplicitIntegration(
	PxGpuParticleSystem* particleSystem,
	ExtGpu::PxLennardJonesSystemData* sysData,
	const PxReal dt,
	const PxU32 numParticles,
	CUstream stream);

/**
\brief Transforms particle data from sorted to unsorted order

\param[in] particleSystems Pointer to the general purpose particle system
\param[in] sysData The Lennard Jones simulation state
\param[in] numParticles The number of particles
\param[in] stream The stream on which the work is scheduled to execute
*/
void PxCopySortedToUnsortedBuffer(
	PxGpuParticleSystem* particleSystems,
	ExtGpu::PxLennardJonesSystemData* sysData,
	const PxU32 numParticles,
	CUstream stream
);
#endif

}
#if !PX_DOXYGEN
} // namespace physx
#endif

#endif //PX_EXT_MD_H
