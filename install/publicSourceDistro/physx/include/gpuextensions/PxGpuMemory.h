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

#ifndef PX_GPU_MEMORY_H
#define PX_GPU_MEMORY_H
/** \addtogroup extensions
  @{
*/


#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"

#include "PxParticleGpu.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

	namespace ExtGpu
	{

#if PX_SUPPORT_GPU_PHYSX

		/**
		\brief Helper class to allocate/free device memory or pinned host memory
		*/
		class PxGpuMemory
		{
		private:
			static const PxU32 cudaSuccess = 0;

			static void allocDeviceBufferInternal(PxCudaContextManager* cudaContextManager, void*& deviceBuffer, PxU32 numBytes, const char* filename = NULL, PxI32 line = -1);
			static void allocPinnedHostBufferInternal(PxCudaContextManager* cudaContextManager, void*& pinnedHostBuffer, PxU32 numBytes, const char* filename = NULL, PxI32 line = -1);

			static void freeDeviceBufferInternal(PxCudaContextManager* cudaContextManager, void* deviceBuffer);
			static void freePinnedHostBufferInternal(PxCudaContextManager* cudaContextManager, void* pinnedHostBuffer);

			static void clearDeviceBufferInternal(PxCudaContextManager* cudaContextManager, void* deviceBuffer, PxU32 numBytes, CUstream stream, PxI32 value);

			static void copyDToHAsyncInternal(PxCudaContextManager* cudaContextManager, void* deviceBuffer, void* hostBuffer, PxI32 numBytes, CUstream stream);
			static void copyHToDAsyncInternal(PxCudaContextManager* cudaContextManager, void* hostBuffer, void* deviceBuffer, PxI32 numBytes, CUstream stream);
			static void copyDToDAsyncInternal(PxCudaContextManager* cudaContextManager, void* srcDeviceBuffer, void* dstDeviceBuffer, PxI32 numBytes, CUstream stream);

			static void copyDToHInternal(PxCudaContextManager* cudaContextManager, void* deviceBuffer, void* hostBuffer, PxI32 numBytes);
			static void copyHToDInternal(PxCudaContextManager* cudaContextManager, const void* hostBuffer, void* deviceBuffer, PxI32 numBytes);

		public:
			template<typename T>
			static void clearDeviceBuffer(PxCudaContextManager* cudaContextManager, T* deviceBuffer, PxU32 numElements, CUstream stream, PxI32 value = 0)
			{
				clearDeviceBufferInternal(cudaContextManager, deviceBuffer, numElements * sizeof(T), stream, value);
			}

			template<typename T>
			static void copyDToH(PxCudaContextManager* cudaContextManager, T* deviceBuffer, T* hostBuffer, PxU32 numElements)
			{
				copyDToHInternal(cudaContextManager, deviceBuffer, hostBuffer, numElements * sizeof(T));
			}

			template<typename T>
			static void copyHToD(PxCudaContextManager* cudaContextManager, const T* hostBuffer, T* deviceBuffer, PxU32 numElements)
			{
				copyHToDInternal(cudaContextManager, hostBuffer, deviceBuffer, numElements * sizeof(T));
			}

			template<typename T>
			static void copyDToHAsync(PxCudaContextManager* cudaContextManager, T* deviceBuffer, T* hostBuffer, PxU32 numElements, CUstream stream)
			{
				copyDToHAsyncInternal(cudaContextManager, deviceBuffer, hostBuffer, numElements * sizeof(T), stream);
			}

			template<typename T>
			static void copyHToDAsync(PxCudaContextManager* cudaContextManager, T* hostBuffer, T* deviceBuffer, PxU32 numElements, CUstream stream)
			{
				copyHToDAsyncInternal(cudaContextManager, hostBuffer, deviceBuffer, numElements * sizeof(T), stream);
			}

			template<typename T>
			static void copyDToDAsync(PxCudaContextManager* cudaContextManager, T* srcDeviceBuffer, T* dstDeviceBuffer, PxU32 numElements, CUstream stream)
			{
				copyDToDAsyncInternal(cudaContextManager, srcDeviceBuffer, dstDeviceBuffer, numElements * sizeof(T), stream);
			}

			template<typename T>
			static void allocDeviceBuffer(PxCudaContextManager* cudaContextManager, T*& deviceBuffer, PxU32 numElements, const char* filename = __FILE__, PxI32 line = __LINE__)
			{
				void* ptr;
				allocDeviceBufferInternal(cudaContextManager, ptr, numElements * sizeof(T), filename, line);
				deviceBuffer = reinterpret_cast<T*>(ptr);
			}

			template<typename T>
			static T* allocDeviceBuffer(PxCudaContextManager* cudaContextManager, PxU32 numElements, const char* filename = __FILE__, PxI32 line = __LINE__)
			{
				void* ptr;
				allocDeviceBufferInternal(cudaContextManager, ptr, numElements * sizeof(T), filename, line);
				return reinterpret_cast<T*>(ptr);
			}

			template<typename T>
			static void freeDeviceBuffer(PxCudaContextManager* cudaContextManager, T*& deviceBuffer)
			{
				freeDeviceBufferInternal(cudaContextManager, deviceBuffer);
				deviceBuffer = NULL;
			}

			template<typename T>
			static void allocPinnedHostBuffer(PxCudaContextManager* cudaContextManager, T*& pinnedHostBuffer, PxU32 numElements, const char* filename = __FILE__, PxI32 line = __LINE__)
			{
				void* ptr;
				allocPinnedHostBufferInternal(cudaContextManager, ptr, numElements * sizeof(T), filename, line);
				pinnedHostBuffer = reinterpret_cast<T*>(ptr);
			}

			template<typename T>
			static T* allocPinnedHostBuffer(PxCudaContextManager* cudaContextManager, PxU32 numElements, const char* filename = __FILE__, PxI32 line = __LINE__)
			{
				void* ptr;
				allocPinnedHostBufferInternal(cudaContextManager, ptr, numElements * sizeof(T), filename, line);
				return reinterpret_cast<T*>(ptr);
			}

			template<typename T>
			static void freePinnedHostBuffer(PxCudaContextManager* cudaContextManager, T*& pinnedHostBuffer)
			{
				freePinnedHostBufferInternal(cudaContextManager, pinnedHostBuffer);
				pinnedHostBuffer = NULL;
			}


			static CUdeviceptr getMappedDevicePtr(PxCudaContextManager* cudaContextManager, void* pinnedHostBuffer);
		};


		#define PX_DEVICE_ALLOC(cudaContextManager, deviceBuffer, numElements) PxGpuMemory::allocDeviceBuffer(cudaContextManager, deviceBuffer, numElements, __FILE__, __LINE__)
		#define PX_DEVICE_ALLOC_T(T, cudaContextManager, numElements) PxGpuMemory::allocDeviceBuffer<T>(cudaContextManager, numElements, __FILE__, __LINE__)
		#define PX_DEVICE_FREE(cudaContextManager, deviceBuffer) PxGpuMemory::freeDeviceBuffer(cudaContextManager, deviceBuffer);

		#define PX_PINNED_HOST_ALLOC(cudaContextManager, pinnedHostBuffer, numElements) PxGpuMemory::allocPinnedHostBuffer(cudaContextManager, pinnedHostBuffer, numElements, __FILE__, __LINE__)
		#define PX_PINNED_HOST_ALLOC_T(T, cudaContextManager, numElements) PxGpuMemory::allocPinnedHostBuffer<T>(cudaContextManager, numElements, __FILE__, __LINE__)
		#define PX_PINNED_HOST_FREE(cudaContextManager, pinnedHostBuffer) PxGpuMemory::freePinnedHostBuffer(cudaContextManager, pinnedHostBuffer);

		template<typename T>
		struct PxDeviceMemoryPointer
		{
		private:
			PxCudaContextManager* mCudaContextManager;
			T* mPointer;
			PxI32 mNumElements;

		public:
			PxDeviceMemoryPointer(PxCudaContextManager* cudaContextManager)
				: mCudaContextManager(cudaContextManager), mPointer(NULL)
			{ }

			PxDeviceMemoryPointer(PxCudaContextManager* cudaContextManager, PxI32 numElements) 
				: mCudaContextManager(cudaContextManager), mPointer(NULL)
			{
				allocate(numElements);
			}

			void initialize(PxCudaContextManager* cudaContextManager)
			{
				mCudaContextManager = cudaContextManager;
			}

			//If false is returned, then the memory must be freed first
			bool allocate(PxI32 numElements, bool autoReleaseExistingBuffer = true)
			{
				if (mPointer) 
				{
					if (!autoReleaseExistingBuffer)
						return false;
					release();
				}

				PxGpuMemory::allocDeviceBuffer(mCudaContextManager, mPointer, numElements);
				mNumElements = numElements;

				return true;
			}

			bool release()
			{
				if (!mPointer)
					return false;

				PxGpuMemory::freeDeviceBuffer(mCudaContextManager, mPointer);
				mPointer = NULL;
				mNumElements = 0;
				return true;
			}

			void writeData(T* dataSourceHost, PxI32 numElements, CUstream stream)
			{
				PX_ASSERT(numElements <= mNumElements);
				PxGpuMemory::copyHToDAsync(mCudaContextManager, dataSourceHost, mPointer, numElements, stream);
			}

			void readData(T* dataDestinationHost, PxI32 numElements, CUstream stream)
			{
				PX_ASSERT(numElements <= mNumElements);
				PxGpuMemory::copyDToHAsync(mCudaContextManager, mPointer, dataDestinationHost, numElements, stream);
			}

			PxU32 getNumElements() const
			{
				return mNumElements;
			}

			T* getDevicePointer()
			{
				return mPointer;
			}

			~PxDeviceMemoryPointer()
			{
				release();
			}
		};

		template<typename T>
		struct PxPinnedHostMemoryPointer
		{
		private:
			PxCudaContextManager* mCudaContextManager;
			T* mPointer;
			PxI32 mNumElements;

		public:
			PxPinnedHostMemoryPointer(PxCudaContextManager* cudaContextManager)
				: mCudaContextManager(cudaContextManager), mPointer(NULL)
			{ }

			PxPinnedHostMemoryPointer(PxCudaContextManager* cudaContextManager, PxI32 numElements)
				: mCudaContextManager(cudaContextManager), mPointer(NULL)
			{
				allocate(numElements);
			}

			void initialize(PxCudaContextManager* cudaContextManager)
			{
				mCudaContextManager = cudaContextManager;
			}

			//If false is returned, then the memory must be freed first
			bool allocate(PxI32 numElements, bool autoReleaseExistingBuffer = true)
			{
				if (mPointer)
				{
					if (!autoReleaseExistingBuffer)
						return false;
					release();
				}

				PxGpuMemory::allocPinnedHostBuffer(mCudaContextManager, mPointer, numElements);
				mNumElements = numElements;
				return true;
			}

			bool release()
			{
				if (!mPointer)
					return false;

				PxGpuMemory::freePinnedHostBuffer(mCudaContextManager, mPointer);
				mPointer = NULL;
				mNumElements = 0;
				return true;
			}

			void writeData(T* dataSourceHost, PxI32 numElements)
			{
				PX_ASSERT(numElements <= mNumElements);
				memcpy(mPointer, dataSourceHost, numElements * sizeof(T));
			}

			void readData(T* dataDestinationHost, PxI32 numElements)
			{
				PX_ASSERT(numElements <= mNumElements);
				memcpy(dataDestinationHost, mPointer, numElements * sizeof(T));
			}

			PxU32 getNumElements() const
			{
				return mNumElements;
			}

			T* getHostPointer()
			{
				return mPointer;
			}

			CUdeviceptr getMappedDevicePointer()
			{
				return PxGpuMemory::getMappedDevicePtr(mCudaContextManager, mPointer);
			}

			~PxPinnedHostMemoryPointer()
			{
				release();
			}
		};

#endif
	}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
