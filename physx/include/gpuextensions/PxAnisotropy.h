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

#ifndef PX_ANISOTROPY_H
#define PX_ANISOTROPY_H
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
#include "PxParticleNeighborhoodProvider.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

	namespace ExtGpu
	{

#if PX_SUPPORT_GPU_PHYSX		

		/**
		\brief Computes anisotropy information for a particle system to improve rendering quality
		*/
		class PxAnisotropyGenerator
		{
		public:
			/**
			\brief Schedules the compuation of anisotropy information on the specified cuda stream

			\param[in] gpuParticleSystem A gpu pointer to access particle system data
			\param[in] numParticles The number of particles
			\param[in] stream The stream on which the cuda call gets scheduled
			*/
			virtual void generateAnisotropy(PxGpuParticleSystem* gpuParticleSystem, PxU32 numParticles, CUstream stream) = 0;
			
			/**
			\brief Schedules the compuation of anisotropy information on the specified cuda stream

			\param[in] particlePositionsGpu A gpu pointer containing the particle positions
			\param[in] neighborhoodProvider A neighborhood provider object that supports fast neighborhood queries
			\param[in] numParticles The number of particles
			\param[in] particleContactOffset The particle contact offset
			\param[in] stream The stream on which the cuda call gets scheduled
			*/
			virtual void generateAnisotropy(PxVec4* particlePositionsGpu, PxParticleNeighborhoodProvider& neighborhoodProvider, PxU32 numParticles, PxReal particleContactOffset, CUstream stream) = 0;

			/**
			\brief Set a host buffer that holds the anisotropy data after the timestep completed

			\param[in] anisotropy1 A host buffer holding the first row of the anisotropy matrix with memory for all particles already allocated
			\param[in] anisotropy2 A host buffer holding the second row of the anisotropy matrix with memory for all particles already allocated
			\param[in] anisotropy3 A host buffer holding the third row of the anisotropy matrix with memory for all particles already allocated
			*/
			virtual void setResultBufferHost(PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3) = 0;
			
			/**
			\brief Set a device buffer that holds the anisotrpy data after the timestep completed

			\param[in] anisotropy1 A device buffer holding the first row of the anisotropy matrix with memory for all particles already allocated
			\param[in] anisotropy2 A device buffer holding the second row of the anisotropy matrix with memory for all particles already allocated
			\param[in] anisotropy3 A device buffer holding the third row of the anisotropy matrix with memory for all particles already allocated
			*/
			virtual void setResultBufferDevice(PxVec4* anisotropy1, PxVec4* anisotropy2, PxVec4* anisotropy3) = 0;

			/**
			\brief Sets the maximum value anisotropy can reach in any direction

			\param[in] maxAnisotropy The maximum anisotropy value
			*/
			virtual void setAnisotropyMax(float maxAnisotropy) = 0;

			/**
			\brief Sets the minimum value anisotropy can reach in any direction

			\param[in] minAnisotropy The minimum anisotropy value
			*/
			virtual void setAnisotropyMin(float minAnisotropy) = 0;

			/**
			\brief Sets the anisotropy scale

			\param[in] anisotropyScale The anisotropy scale
			*/
			virtual void setAnisotropyScale(float anisotropyScale) = 0;

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
			\brief Gets the device pointer for the anisotropy in x direction. Only available after calling setResultBufferHost or setResultBufferDevice

			\return The device pointer for the anisotropy x direction and scale (w component contains the scale)
			*/
			virtual PxVec4* getAnisotropy1DevicePointer() const = 0;

			/**
			\brief Gets the device pointer for the anisotropy in y direction. Only available after calling setResultBufferHost or setResultBufferDevice

			\return The device pointer for the anisotropy y direction and scale (w component contains the scale)
			*/
			virtual PxVec4* getAnisotropy2DevicePointer() const = 0;

			/**
			\brief Gets the device pointer for the anisotropy in z direction. Only available after calling setResultBufferHost or setResultBufferDevice

			\return The device pointer for the anisotropy z direction and scale (w component contains the scale)
			*/
			virtual PxVec4* getAnisotropy3DevicePointer() const = 0;

			/**
			\brief Enables or disables the anisotropy generator

			\param[in] enabled The boolean to set the generator to enabled or disabled
			*/
			virtual void setEnabled(bool enabled) = 0;

			/**
			\brief Allows to query if the anisotropy generator is enabled

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
			virtual ~PxAnisotropyGenerator() {}
		};

		/**
		\brief Default implementation of a particle system callback to trigger anisotropy calculations. A call to fetchResultsParticleSystem() on the 
		PxScene will synchronize the work such that the caller knows that the post solve task completed.
		*/
		class PxAnisotropyCallback : public PxParticleSystemCallback
		{
		public:
			/**
			\brief Initializes the anisotropy callback

			\param[in] anistropyGenerator The anisotropy generator
			*/
			void initialize(PxAnisotropyGenerator* anistropyGenerator)
			{
				mAnistropyGenerator = anistropyGenerator;
			}

			virtual void onPostSolve(const PxGpuMirroredPointer<PxGpuParticleSystem>& gpuParticleSystem, CUstream stream)
			{
				if (mAnistropyGenerator) 
				{
					mAnistropyGenerator->generateAnisotropy(gpuParticleSystem.mDevicePtr, gpuParticleSystem.mHostPtr->mCommonData.mMaxParticles, stream);
				}
			}

			virtual void onBegin(const PxGpuMirroredPointer<PxGpuParticleSystem>& /*gpuParticleSystem*/, CUstream /*stream*/) { }

			virtual void onAdvance(const PxGpuMirroredPointer<PxGpuParticleSystem>& /*gpuParticleSystem*/, CUstream /*stream*/) { }

		private:
			PxAnisotropyGenerator* mAnistropyGenerator;
		};

		/**
		\brief Creates an anisotropy generator

		\param[in] cudaContextManager A cuda context manager
		\param[in] maxNumParticles The number of particles
		\param[in] anisotropyScale A uniform scaling factor to increase or decrease anisotropy
		\param[in] minAnisotropy The minimum scaling factor in any dimension that anisotropy can have
		\param[in] maxAnisotropy The maximum scaling factor in any dimension that anisotropy can have
		*/
		PxAnisotropyGenerator* PxCreateAnisotropyGenerator(PxCudaContextManager* cudaContextManager, PxU32 maxNumParticles, 
			PxReal anisotropyScale = 1.0f, PxReal minAnisotropy = 0.1f, PxReal maxAnisotropy = 2.0f);

#endif
	}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
