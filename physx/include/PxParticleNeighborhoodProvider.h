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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_PARTICLE_NEIGHBORHOOD_PROVIDER_H
#define PX_PARTICLE_NEIGHBORHOOD_PROVIDER_H
/** \addtogroup extensions
  @{
*/


#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "PxParticleSystem.h"

#include "foundation/PxArray.h"
#include "PxParticleGpu.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_SUPPORT_GPU_PHYSX

	/**
	\brief Computes neighborhood information for a point cloud
	*/
	class PxParticleNeighborhoodProvider
	{
	public:
		/**
		\brief Schedules the compuation of neighborhood information on the specified cuda stream

		\param[in] deviceParticlePos A gpu pointer containing the particle positions
		\param[in] numParticles The number of particles
		\param[in] stream The stream on which the cuda call gets scheduled
		\param[in] devicePhases An optional gpu pointer with particle phases
		\param[in] validPhaseMask An optional phase mask to define which particles should be included into the neighborhood computation
		\param[in] deviceActiveIndices An optional device pointer containing all indices of particles that are currently active
		*/
		virtual void buildNeighborhood(PxVec4* deviceParticlePos, const PxU32 numParticles, CUstream stream, PxU32* devicePhases = NULL,
			PxU32 validPhaseMask = PxParticlePhaseFlag::eParticlePhaseFluid, const PxU32* deviceActiveIndices = NULL) = 0;

		/**
		\brief Gets the maximal number of particles

		\return The maximal number of particles
		*/
		virtual PxU32 getMaxParticles() const = 0;

		/**
		\brief Sets the maximal number of particles

		\param[in] maxParticles The maximal number of particles
		*/
		virtual void setMaxParticles(PxU32 maxParticles) = 0;

		/**
		\brief Gets the maximal number of grid cells

		\return The maximal number of grid cells
		*/
		virtual PxU32 getMaxGridCells() const = 0;

		/**
		\brief Gets the cell size

		\return The cell size
		*/
		virtual PxReal getCellSize() const = 0;

		/**
		\brief Gets the number of grid cells in use

		\return The number of grid cells in use
		*/
		virtual PxU32 getNumGridCellsInUse() const = 0;

		/**
		\brief Sets the maximal number of particles

		\param[in] maxGridCells The maximal number of grid cells
		\param[in] cellSize The cell size. Should be equal to 2*contactOffset for PBD particle systems.
		*/
		virtual void setCellProperties(PxU32 maxGridCells, PxReal cellSize) = 0;

		/**
		\brief Releases the instance and its data
		*/
		virtual void release() = 0;

		/**
		\brief Destructor
		*/
		virtual ~PxParticleNeighborhoodProvider() {}
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
