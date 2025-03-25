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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXG_CUDA_BUFFER_H
#define PXG_CUDA_BUFFER_H

#include "foundation/PxPreprocessor.h"
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#pragma clang diagnostic ignored "-Wdisabled-macro-expansion"
#endif
#include "cuda.h"
#if PX_LINUX && PX_CLANG
#pragma clang diagnostic pop
#endif

#include "PxgHeapMemAllocator.h"
#include "PxgDevicePointer.h"

namespace physx
{
	class PxCudaContext;
	
	class PxgCudaBuffer
	{
		PX_NOCOPY(PxgCudaBuffer)
	public:
		PxgCudaBuffer(PxgHeapMemoryAllocatorManager* heapMemoryManager, PxsHeapStats::Enum statGroup)
			: mPtr(0)
			, mHeapMemoryAllocator(heapMemoryManager->mDeviceMemoryAllocators)
			, mSize(0)
			, mStatGroup(statGroup)
		{
		}

		~PxgCudaBuffer();

		void allocate(const PxU64 size, const char* filename, PxI32 line);
		void allocateCopyOldDataAsync(const PxU64 size, PxCudaContext* cudaContext, CUstream stream, const char* filename, PxI32 line);

		void deallocate();

		/* defer deallocation until the beginning of the next simulation step */
		void deallocateDeferred();

		PX_FORCE_INLINE	CUdeviceptr getDevicePtr()				const	{ return (mPtr + 127) & (~127);	}
		PX_FORCE_INLINE	PxU64		getSize()					const	{ return mSize;					}
		PX_FORCE_INLINE	void		set(CUdeviceptr ptr, PxU64 size)	{ mPtr = ptr;	mSize = size;	}

		static void swapBuffer(PxgCudaBuffer& buf0, PxgCudaBuffer& buf1)
		{
			const CUdeviceptr tempPtr = buf0.getDevicePtr();
			const PxU64 tempSize = buf0.getSize();

			buf0.set(buf1.getDevicePtr(), buf1.getSize());
			buf1.set(tempPtr, tempSize);
		}

		void assign(PxgCudaBuffer& b1)
		{
			PX_ASSERT(mHeapMemoryAllocator == b1.mHeapMemoryAllocator);
			PX_ASSERT(mStatGroup == b1.mStatGroup);

			deallocate();

			mPtr = b1.mPtr;
			mSize = b1.mSize;

			b1.mPtr = 0;
			b1.mSize = 0;
		}

	protected:
		CUdeviceptr					mPtr;
		PxgHeapMemoryAllocator*		mHeapMemoryAllocator;
		PxU64						mSize;
		const PxsHeapStats::Enum	mStatGroup;
	};

	template <typename T>
	class PxgTypedCudaBuffer : public PxgCudaBuffer
	{
	public:
		PxgTypedCudaBuffer(PxgHeapMemoryAllocatorManager* heapMemoryManager, PxsHeapStats::Enum statGroup)
			: PxgCudaBuffer(heapMemoryManager, statGroup) 
		{ }

		PX_FORCE_INLINE	void allocateElements(const PxU64 nbElements, const char* filename, PxI32 line) { allocate(nbElements * sizeof(T), filename, line); }

		PX_FORCE_INLINE	PxU64 getNbElements() const { return mSize / sizeof(T); }

		PX_FORCE_INLINE	PxgDevicePointer<T> getTypedDevicePtr() const { return PxgDevicePointer<T>(getDevicePtr()); }

		PX_FORCE_INLINE	T* getTypedPtr() const { return reinterpret_cast<T*>(getDevicePtr()); }
	};

	template <unsigned int NbBuffers>
	class PxgCudaBufferN
	{
		PxU8 mCudaArrays[sizeof(PxgCudaBuffer)*NbBuffers];
	public:
		PxgCudaBufferN(PxgHeapMemoryAllocatorManager* heapMemoryManager, PxsHeapStats::Enum statGroup)
		{
			PxgCudaBuffer* buffers = reinterpret_cast<PxgCudaBuffer*>(mCudaArrays);
			for (PxU32 i = 0; i < NbBuffers; ++i)
			{
				PX_PLACEMENT_NEW(&buffers[i], PxgCudaBuffer)(heapMemoryManager, statGroup);
			}
		}

		~PxgCudaBufferN()
		{
			PxgCudaBuffer* buffers = reinterpret_cast<PxgCudaBuffer*>(mCudaArrays);
			for (PxU32 i = 0; i < NbBuffers; ++i)
			{
				buffers[i].~PxgCudaBuffer();
			}
		}

		PxgCudaBuffer& operator [](PxU32 index) { PX_ASSERT(index < NbBuffers); return reinterpret_cast<PxgCudaBuffer*>(mCudaArrays)[index]; }

		const PxgCudaBuffer& operator [](PxU32 index) const { PX_ASSERT(index < NbBuffers); return reinterpret_cast<const PxgCudaBuffer*>(mCudaArrays)[index]; }

		PxgCudaBuffer* begin(){ return reinterpret_cast<PxgCudaBuffer*>(mCudaArrays); }

		PxU32 size() { return NbBuffers; }
	};

}

#endif
