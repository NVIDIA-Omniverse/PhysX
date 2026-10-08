// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

#include "foundation/PxAssert.h"
#include "foundation/PxAllocator.h"
#include "PxgDevicePointer.h"

namespace physx
{
	class PxCudaContext;
	class PxgHeapMemoryAllocator;
	
	class PxgCudaBuffer
	{
		PX_NOCOPY(PxgCudaBuffer)
	public:
		PxgCudaBuffer(PxgHeapMemoryAllocator& deviceAlloc, PxI32 statGroup);
		~PxgCudaBuffer();

		void allocate(const PxU64 size, const char* filename, PxI32 line);
		void allocateCopyOldDataAsync(const PxU64 size, PxCudaContext* cudaContext, CUstream stream, const char* filename, PxI32 line);

		void deallocate();

		/* defer deallocation until the beginning of the next simulation step */
		void deallocateDeferred();

		PX_FORCE_INLINE	CUdeviceptr getDevicePtr()				const	{ return (mPtr + 127) & (~127);	}
		PX_FORCE_INLINE	PxU64		getSize()					const	{ return mSize;					}

		static void swapBuffer(PxgCudaBuffer& buf0, PxgCudaBuffer& buf1)
		{
			//Swap the raw allocation pointers, not getDevicePtr(): the heap allocator keys its lookup table on the
			//address it handed out, while getDevicePtr() returns the 128-byte aligned version of it. Going through
			//getDevicePtr() here would silently replace mPtr with an address the allocator does not know about as
			//soon as a block is less than 128-byte aligned, and the buffer could then not be released anymore.
			const CUdeviceptr tempPtr = buf0.mPtr;
			const PxU64 tempSize = buf0.mSize;

			buf0.mPtr = buf1.mPtr;
			buf0.mSize = buf1.mSize;

			buf1.mPtr = tempPtr;
			buf1.mSize = tempSize;
		}

		void assign(PxgCudaBuffer& b1)
		{
			PX_ASSERT(&mDeviceAlloc == &b1.mDeviceAlloc);
			PX_ASSERT(mStatGroup == b1.mStatGroup);

			deallocate();

			mPtr = b1.mPtr;
			mSize = b1.mSize;

			b1.mPtr = 0;
			b1.mSize = 0;
		}

	protected:
		CUdeviceptr					mPtr;
		PxgHeapMemoryAllocator&		mDeviceAlloc;
		PxU64						mSize;
		const PxU32					mStatGroup;
	};

	template <typename T>
	class PxgTypedCudaBuffer : public PxgCudaBuffer
	{
	public:
		PxgTypedCudaBuffer(PxgHeapMemoryAllocator& deviceAlloc, PxI32 statGroup)
			: PxgCudaBuffer(deviceAlloc, statGroup) 
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
		PxgCudaBufferN(PxgHeapMemoryAllocator& deviceAlloc, PxI32 statGroup)
		{
			PxgCudaBuffer* buffers = reinterpret_cast<PxgCudaBuffer*>(mCudaArrays);
			for (PxU32 i = 0; i < NbBuffers; ++i)
			{
				PX_PLACEMENT_NEW(&buffers[i], PxgCudaBuffer)(deviceAlloc, statGroup);
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
