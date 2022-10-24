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

#ifndef PX_MULTI_CALLBACK_H
#define PX_MULTI_CALLBACK_H
/** \addtogroup extensions
  @{
*/


#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "PxParticleSystem.h"

#include "PxGpuMemory.h"
#include "foundation/PxArray.h"
#include "PxParticleGpu.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

	namespace ExtGpu
	{

#if PX_SUPPORT_GPU_PHYSX

		/**
		\brief Special callback that forwards calls to arbitrary many sub-callbacks
		*/
		class PxMultiCallback : public PxParticleSystemCallback
		{
		private:
			PxArray<PxParticleSystemCallback*> mCallbacks;

		public:
			PxMultiCallback() : mCallbacks(0) {}

			virtual void onPostSolve(const PxGpuMirroredPointer<PxGpuParticleSystem>& gpuParticleSystem, CUstream stream)
			{
				for (PxU32 i = 0; i < mCallbacks.size(); ++i)
					mCallbacks[i]->onPostSolve(gpuParticleSystem, stream);
			}

			virtual void onBegin(const PxGpuMirroredPointer<PxGpuParticleSystem>& gpuParticleSystem, CUstream stream)
			{
				for (PxU32 i = 0; i < mCallbacks.size(); ++i)
					mCallbacks[i]->onBegin(gpuParticleSystem, stream);
			}

			virtual void onAdvance(const PxGpuMirroredPointer<PxGpuParticleSystem>& gpuParticleSystem, CUstream stream)
			{
				for (PxU32 i = 0; i < mCallbacks.size(); ++i)
					mCallbacks[i]->onAdvance(gpuParticleSystem, stream);
			}

			/**
			\brief Adds a callback

			\param[in] callback The callback to add
			\return True if the callback was added
			*/
			bool addCallback(PxParticleSystemCallback* callback)
			{
				if (mCallbacks.find(callback) != mCallbacks.end())
					return false;
				mCallbacks.pushBack(callback);
				return true;
			}

			/**
			\brief Removes a callback

			\param[in] callback The callback to remove
			\return True if the callback was removed
			*/
			bool removeCallback(PxParticleSystemCallback* callback)
			{
				for (PxU32 i = 0; i < mCallbacks.size(); ++i)
				{
					if (mCallbacks[i] == callback)
					{
						mCallbacks.remove(i);
						return true;
					}
				}
				return false;
			}
		};

#endif
	}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
