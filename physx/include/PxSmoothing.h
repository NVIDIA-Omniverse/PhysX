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

#ifndef PX_SMOOTHING_H
#define PX_SMOOTHING_H
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

	class PxgKernelLauncher;
	class PxParticleNeighborhoodProvider;

	/**
	\brief Ccomputes smoothed positions for a particle system to improve rendering quality
	*/
	class PxSmoothedPositionGenerator
	{
	public:
		/**
		\brief Schedules the compuation of smoothed positions on the specified cuda stream

		\param[in] gpuParticleSystem A gpu pointer to access particle system data
		\param[in] numParticles The number of particles
		\param[in] stream The stream on which the cuda call gets scheduled
		*/
		virtual void generateSmoothedPositions(PxGpuParticleSystem* gpuParticleSystem, PxU32 numParticles, CUstream stream) = 0;

		/**
		\brief Schedules the compuation of smoothed positions on the specified cuda stream

		\param[in] particlePositionsGpu A gpu pointer containing the particle positions
		\param[in] neighborhoodProvider A neighborhood provider object that supports fast neighborhood queries
		\param[in] numParticles The number of particles
		\param[in] particleContactOffset The particle contact offset
		\param[in] stream The stream on which the cuda call gets scheduled
		*/
		virtual void generateSmoothedPositions(PxVec4* particlePositionsGpu, PxParticleNeighborhoodProvider& neighborhoodProvider, PxU32 numParticles, PxReal particleContactOffset, CUstream stream) = 0;

		/**
		\brief Set a host buffer that holds the smoothed position data after the timestep completed

		\param[in] smoothedPositions A host buffer with memory for all particles already allocated
		*/
		virtual void setResultBufferHost(PxVec4* smoothedPositions) = 0;

		/**
		\brief Set a device buffer that holds the smoothed position data after the timestep completed

		\param[in] smoothedPositions A device buffer with memory for all particles already allocated
		*/
		virtual void setResultBufferDevice(PxVec4* smoothedPositions) = 0;

		/**
		\brief Sets the intensity of the position smoothing effect

		\param[in] smoothingStrenght The strength of the smoothing effect
		*/
		virtual void setSmoothing(float smoothingStrenght) = 0;

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
		\brief Gets the device pointer for the smoothed positions. Only available after calling setResultBufferHost or setResultBufferDevice

		\return The device pointer for the smoothed positions
		*/
		virtual PxVec4* getSmoothedPositionsDevicePointer() const = 0;

		/**
		\brief Enables or disables the smoothed position generator

		\param[in] enabled The boolean to set the generator to enabled or disabled
		*/
		virtual void setEnabled(bool enabled) = 0;

		/**
		\brief Allows to query if the smoothed position generator is enabled

		\return True if enabled, false otherwise
		*/
		virtual bool isEnabled() const = 0;

		/**
		\brief Releases the instance and its data
		*/
		virtual void release() = 0;

		/**
		\brief Destructor
		*/
		virtual ~PxSmoothedPositionGenerator() {}
	};

	/**
	\brief Default implementation of a particle system callback to trigger smoothed position calculations. A call to fetchResultsParticleSystem() on the
	PxScene will synchronize the work such that the caller knows that the post solve task completed.
	*/
	class PxSmoothedPositionCallback : public PxParticleSystemCallback
	{
	public:
		/**
		\brief Initializes the smoothing callback

		\param[in] smoothedPositionGenerator The smoothed position generator
		*/
		void initialize(PxSmoothedPositionGenerator* smoothedPositionGenerator)
		{
			mSmoothedPositionGenerator = smoothedPositionGenerator;
		}

		virtual void onPostSolve(const PxGpuMirroredPointer<PxGpuParticleSystem>& gpuParticleSystem, CUstream stream)
		{
			if (mSmoothedPositionGenerator)
			{
				mSmoothedPositionGenerator->generateSmoothedPositions(gpuParticleSystem.mDevicePtr, gpuParticleSystem.mHostPtr->mCommonData.mMaxParticles, stream);
			}
		}

		virtual void onBegin(const PxGpuMirroredPointer<PxGpuParticleSystem>& /*gpuParticleSystem*/, CUstream /*stream*/) { }

		virtual void onAdvance(const PxGpuMirroredPointer<PxGpuParticleSystem>& /*gpuParticleSystem*/, CUstream /*stream*/) { }

	private:
		PxSmoothedPositionGenerator* mSmoothedPositionGenerator;
	};
	
#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
