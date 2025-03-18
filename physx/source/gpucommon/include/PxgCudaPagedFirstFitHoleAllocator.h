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

#ifndef PXG_PAGED_FIRST_FIT_HOLE_ALLOCATOR_H
#define PXG_PAGED_FIRST_FIT_HOLE_ALLOCATOR_H

#include "foundation/PxArray.h"

namespace physx
{
	namespace cudaMappedMemAllocatorInternal
	{
		template<typename PointedToT>
		struct Pointer
		{
			PX_FORCE_INLINE Pointer() {}
			PX_FORCE_INLINE Pointer(const Pointer& ref) : hostPtr(ref.hostPtr), devPtr(ref.devPtr) {}
			PX_FORCE_INLINE explicit Pointer(const PxZERO&) : hostPtr(NULL), devPtr(0) {}

			PX_FORCE_INLINE Pointer& operator=(const Pointer& ref)
			{
				if (&ref != this)
				{
					hostPtr = ref.hostPtr;
					devPtr = ref.devPtr;
				}

				return *this;
			}

			PX_FORCE_INLINE Pointer& operator+=(const ptrdiff_t& ref)
			{
				hostPtr += ref;
				devPtr += ref * sizeof(PointedToT);

				return *this;
			}

			PX_FORCE_INLINE Pointer& operator-=(const ptrdiff_t& ref)
			{
				hostPtr -= ref;
				devPtr -= ref * sizeof(PointedToT);

				return *this;
			}

			PX_FORCE_INLINE Pointer operator+(const ptrdiff_t& ref) const
			{
				Pointer ret;
				ret.hostPtr = hostPtr + ref;
				ret.devPtr = devPtr + ref * sizeof(PointedToT);

				return ret;
			}

			PX_FORCE_INLINE Pointer operator-(const ptrdiff_t& ref) const
			{
				Pointer ret;
				ret.hostPtr = hostPtr - ref;
				ret.devPtr = devPtr - ref * sizeof(PointedToT);

				return ret;
			}

			PX_FORCE_INLINE ptrdiff_t operator-(const Pointer& ref) const
			{
				ptrdiff_t ret;
				ret = hostPtr - ref.hostPtr;
				PX_ASSERT(static_cast<ptrdiff_t>(ret * sizeof(PointedToT)) ==
					(static_cast<ptrdiff_t>(devPtr)-static_cast<ptrdiff_t>(ref.devPtr)));

				return ret;
			}

			PX_FORCE_INLINE bool operator<(const Pointer& ref) const
			{
				PX_ASSERT(hostPtr < ref.hostPtr == devPtr < ref.devPtr);

				return devPtr < ref.devPtr;
			}

			PX_FORCE_INLINE bool operator>(const Pointer& ref) const
			{
				return ref.operator<(*this);
			}

			PX_FORCE_INLINE bool operator<=(const Pointer& ref) const
			{
				return !(operator>(ref));
			}

			PX_FORCE_INLINE bool operator>=(const Pointer& ref) const
			{
				return !(operator<(ref));
			}

			PX_FORCE_INLINE bool operator==(const Pointer& ref) const
			{
				PX_ASSERT((hostPtr == ref.hostPtr) == (devPtr == ref.devPtr));

				return devPtr == ref.devPtr;
			}

			PX_FORCE_INLINE bool operator!=(const Pointer& ref) const
			{
				return !(operator==(ref));
			}

			//this is to allow NULL ptr comparison
			PX_FORCE_INLINE bool operator==(int i) const
			{
				PX_ASSERT((hostPtr == NULL) == (devPtr == 0));

				return i == 0 && devPtr == 0;
			}

			PX_FORCE_INLINE bool operator!=(int i) const
			{
				return !(operator==(i));
			}

			PointedToT*					hostPtr;
			CUdeviceptr					devPtr;
		};

		template<>
		struct Pointer < void >
		{
			PX_FORCE_INLINE Pointer() {}
			PX_FORCE_INLINE Pointer(const Pointer& ref) : hostPtr(ref.hostPtr), devPtr(ref.devPtr) {}
			PX_FORCE_INLINE explicit Pointer(const PxZERO&) : hostPtr(NULL), devPtr(0) {}
			PX_FORCE_INLINE Pointer& operator=(const Pointer& ref)
			{
				if (&ref != this)
				{
					hostPtr = ref.hostPtr;
					devPtr = ref.devPtr;
				}

				return *this;
			}

			PX_FORCE_INLINE bool operator<(const Pointer& ref) const
			{
				PX_ASSERT((hostPtr < ref.hostPtr) == (devPtr < ref.devPtr));

				return devPtr < ref.devPtr;
			}

			PX_FORCE_INLINE bool operator>(const Pointer& ref) const
			{
				return ref.operator<(*this);
			}

			PX_FORCE_INLINE bool operator<=(const Pointer& ref) const
			{
				return !(operator>(ref));
			}

			PX_FORCE_INLINE bool operator>=(const Pointer& ref) const
			{
				return !(operator<(ref));
			}

			PX_FORCE_INLINE bool operator==(const Pointer& ref) const
			{
				PX_ASSERT((hostPtr == ref.hostPtr) == (devPtr == ref.devPtr));

				return devPtr == ref.devPtr;
			}

			PX_FORCE_INLINE bool operator!=(const Pointer& ref) const
			{
				return !(operator==(ref));
			}

			//this is to allow NULL ptr comparison
			PX_FORCE_INLINE bool operator==(int i) const
			{
				PX_ASSERT((hostPtr == NULL) == (devPtr == 0));

				return i == 0 && devPtr == 0;
			}

			PX_FORCE_INLINE bool operator!=(int i) const
			{
				return !(operator==(i));
			}

			void*						hostPtr;
			CUdeviceptr					devPtr;
		};


		template<typename PointedToT>
		Pointer<PointedToT> operator+(const ptrdiff_t& argL,
			const Pointer<PointedToT>& argR)
		{
			Pointer<PointedToT> ret;
			ret.hostPtr = argR.hostPtr + argL;
			ret.devPtr = argR.devPtr + argL * sizeof(PointedToT);

			return ret;
		}
	}

template<typename PointedToT>
static PX_FORCE_INLINE cudaMappedMemAllocatorInternal::Pointer<void> castTo(const cudaMappedMemAllocatorInternal::Pointer<PointedToT>& toPtr)
{
	cudaMappedMemAllocatorInternal::Pointer<void> ret;
	ret.hostPtr = static_cast<void *>(toPtr.hostPtr);
	ret.devPtr = toPtr.devPtr;

	return ret;
}

template<typename PointedToT>
static PX_FORCE_INLINE cudaMappedMemAllocatorInternal::Pointer<PointedToT> castTo(const cudaMappedMemAllocatorInternal::Pointer<void>& ref)
{
	cudaMappedMemAllocatorInternal::Pointer<PointedToT> ret;
	ret.hostPtr = static_cast<PointedToT *>(ref.hostPtr);
	ret.devPtr = ref.devPtr;

	return ret;
}

}

namespace physx
{
	namespace cudaPagedFirstFitHoleAllocatorInternal
	{
		template<typename AllocT, typename PointedToT>
		struct Pointer
		{
			PX_FORCE_INLINE Pointer() {}
			PX_FORCE_INLINE Pointer(const Pointer& ref) : ptr(ref.hostPtr), sz(ref.sz) {}
			PX_FORCE_INLINE explicit Pointer(const PxZERO&) : ptr(PxZERO()), sz(0) {}

			PX_FORCE_INLINE Pointer& operator=(const Pointer& ref)
			{
				if (&ref != this)
				{
					ptr = ref.hostPtr;
					sz = ref.sz;
				}

				return this;
			}

			PX_FORCE_INLINE Pointer& operator+=(const ptrdiff_t& ref)
			{
				ptr += ref;
				sz = (size_t)-1;

				return this;
			}

			PX_FORCE_INLINE Pointer& operator-=(const ptrdiff_t& ref)
			{
				ptr -= ref;
				sz = (size_t)-1;

				return this;
			}

			PX_FORCE_INLINE Pointer operator+(const ptrdiff_t& ref) const
			{
				Pointer ret;
				ret.ptr = ptr + ref;
				ret.sz = (size_t)-1;

				return ret;
			}

			PX_FORCE_INLINE Pointer operator-(const ptrdiff_t& ref) const
			{
				Pointer ret;
				ret.ptr = ptr - ref;
				ret.sz = (size_t)-1;

				return ret;
			}

			PX_FORCE_INLINE ptrdiff_t operator-(const Pointer& ref) const
			{
				ptrdiff_t ret;
				ret = ptr - ref.ptr;

				return ret;
			}

			PX_FORCE_INLINE bool operator<(const Pointer& ref) const
			{
				return ptr < ref.ptr;
			}

			PX_FORCE_INLINE bool operator>(const Pointer& ref) const
			{
				return ref.operator<(*this);
			}

			PX_FORCE_INLINE bool operator<=(const Pointer& ref) const
			{
				return !(operator>(ref));
			}

			PX_FORCE_INLINE bool operator>=(const Pointer& ref) const
			{
				return !(operator<(ref));
			}

			PX_FORCE_INLINE bool operator==(const Pointer& ref) const
			{
				return ptr == ref.ptr;
			}

			PX_FORCE_INLINE bool operator!=(const Pointer& ref) const
			{
				return !(operator==(ref));
			}

			//this is to allow NULL ptr comparison
			PX_FORCE_INLINE bool operator==(int i) const
			{
				return i == 0 && ptr == NULL;
			}

			PX_FORCE_INLINE bool operator!=(int i) const
			{
				return !(operator==(i));
			}

			typename AllocT::template PointerType<PointedToT>::type			ptr;
			size_t													sz;
		};

		template<typename AllocT>
		struct Pointer <AllocT, void >
		{
			PX_FORCE_INLINE Pointer() {}
			PX_FORCE_INLINE Pointer(const Pointer& ref) : ptr(ref.ptr), sz(ref.sz) {}
			PX_FORCE_INLINE explicit Pointer(const PxZERO&) : ptr(PxZERO()), sz(0) {}
			PX_FORCE_INLINE Pointer& operator=(const Pointer& ref)
			{
				if (&ref != this)
				{
					ptr = ref.ptr;
					sz = ref.sz;
				}

				return *this;
			}

			PX_FORCE_INLINE bool operator<(const Pointer& ref) const
			{
				return ptr < ref.ptr;
			}

			PX_FORCE_INLINE bool operator>(const Pointer& ref) const
			{
				return ref.operator<(*this);
			}

			PX_FORCE_INLINE bool operator<=(const Pointer& ref) const
			{
				return !(operator>(ref));
			}

			PX_FORCE_INLINE bool operator>=(const Pointer& ref) const
			{
				return !(operator<(ref));
			}

			PX_FORCE_INLINE bool operator==(const Pointer& ref) const
			{
				return ptr == ref.ptr;
			}

			PX_FORCE_INLINE bool operator!=(const Pointer& ref) const
			{
				return !(operator==(ref));
			}

			//this is to allow NULL ptr comparison
			PX_FORCE_INLINE bool operator==(int i) const
			{
				return i == 0 && ptr == NULL;
			}

			PX_FORCE_INLINE bool operator!=(int i) const
			{
				return !(operator==(i));
			}

			typename AllocT::template PointerType<void>::type		ptr;
			size_t											sz;
		};
	}

template<typename AllocT, size_t defaultPageBytesize = 1024, size_t holeAlignment = 256>
class PxgCudaPagedFirstFitHoleAllocator
{
	PX_NOCOPY(PxgCudaPagedFirstFitHoleAllocator)
public:
	//this is a workaround to enable allocators with typedefed pointer types
	template<typename PointedToT>
	struct PointerType
	{
		typedef cudaPagedFirstFitHoleAllocatorInternal::Pointer<AllocT ,PointedToT> type;
	};
	
	PxgCudaPagedFirstFitHoleAllocator(AllocT&	alloc): mAlloc(alloc),
														mFirstHoleIdx(-1),
														mLastHoleIdx(-1),
														mFirstDeinitializedHoleIdx(-1)
	{}
	
	virtual ~PxgCudaPagedFirstFitHoleAllocator()
	{
		resetAndRelease();
	}

	void resetAndRelease()
	{
		for (typename PxArray<typename AllocT::template PointerType<PxU8>::type >::Iterator it = mPages.begin(), end = mPages.end(); it != end; ++it)
		{
			mAlloc.deallocate(castTo<PxU8>(*it));
		}

		mPages.resize(0);
		mHoles.resize(0);
		mFirstHoleIdx = -1;
		mLastHoleIdx = -1;
		mFirstDeinitializedHoleIdx = -1;
	}

	typename PointerType<void>::type allocate(size_t byteSize)
	{
		byteSize = (byteSize + holeAlignment - 1) & ~(holeAlignment - 1);

		PxI32 idx = mFirstHoleIdx;

		while (idx != -1)
		{
			Hole& hole = mHoles[(PxU32)idx];

			if (hole.byteSize >= byteSize)
				break;

			idx = hole.nextIndex;
		}

		if (idx == -1)
		{
			idx = addNewPage(byteSize);

			if (idx == -1)
			{
				return typename PointerType<void>::type(PxZERO());
			}
		}

		Hole& hole = mHoles[(PxU32)idx];
		typename PointerType<void>::type ret;
		ret.ptr = castTo<PxU8>(hole.ptr);
		ret.sz = byteSize;
		hole.ptr += (ptrdiff_t)byteSize;
		PX_ASSERT(hole.byteSize >= byteSize);
		hole.byteSize -= byteSize;

		if (hole.byteSize == 0)
		{
			PxI32& prevIdx = hole.prevIndex != -1 ? mHoles[(PxU32)hole.prevIndex].nextIndex : mFirstHoleIdx;
			PxI32& nextIdx = hole.nextIndex != -1 ? mHoles[(PxU32)hole.nextIndex].prevIndex : mLastHoleIdx;

			prevIdx = hole.nextIndex;
			nextIdx = hole.prevIndex;

			deallocateHole(idx);
		}

		return ret;
	}

	void deallocate(const typename PointerType<void>::type& ptr)
	{
		//don't try to deallocate pointers that were calculated, only those obtained from allocate() 
		PX_ASSERT(ptr.sz != (size_t) -1);

		//we don't want to have zero-sized holes from deleting zero-sized allocations (which are valid)
		if (ptr.sz == 0 || ptr.sz == (size_t) -1)
			return;

		deallocateInternal(castTo<PxU8>(ptr.ptr), ptr.sz);
	}

#if PX_CHECKED	
	bool consistencyCheck()
	{
		for (PxU32 i = 0; i < mHoles.size(); ++i)
		{
			if (mHoles[i].ptr != NULL && mHoles[i].byteSize == 0)
			{
				PX_ASSERT(false);
				
				return false;
			}
		}

		PxI32 prevI = -1;
		PxU32 holeCtr = 0;

		for (PxI32 i = mFirstHoleIdx; i != -1; prevI = i, i = mHoles[(PxU32)i].nextIndex)
		{
			++holeCtr;
		}

		if (prevI != mLastHoleIdx)
		{
			PX_ASSERT(false);
				
			return false;
		}

		prevI = -1;
		PxU32 holeCtr2 = 0;

		for (PxI32 i = mLastHoleIdx; i != -1; prevI = i, i = mHoles[(PxU32)i].prevIndex)
		{
			++holeCtr2;
		}

		if (prevI != mFirstHoleIdx)
		{
			PX_ASSERT(false);
				
			return false;
		}

		if (holeCtr2 != holeCtr)
		{
			PX_ASSERT(false);
				
			return false;
		}

		PxU32 holeCtr3 = 0;
		for (PxI32 i = mFirstDeinitializedHoleIdx; i != -1; i = mHoles[(PxU32)i].nextIndex)
		{
			++holeCtr3;
		}

		if (holeCtr2 + holeCtr3 != mHoles.size())
		{
			PX_ASSERT(false);
				
			return false;
		}

		prevI = mFirstHoleIdx;

		if (prevI != -1)
		{
			for (PxI32 i = mHoles[(PxU32) prevI].nextIndex; i != -1; prevI = i, i = mHoles[(PxU32) i].nextIndex)
			{
				if (mHoles[(PxU32) prevI].ptr + (ptrdiff_t) mHoles[(PxU32) prevI].byteSize >= mHoles[(PxU32) i].ptr)
				{
					PX_ASSERT(false);
					return false;
				}
			}
		}

		return true;
	}
#endif
	
protected:
	
	struct Hole
	{
		Hole(): ptr(PxZERO()), byteSize(0), prevIndex(-1), nextIndex(-1) {}
		Hole(PxI32 prev, PxI32 next):
									ptr(PxZERO()),
									byteSize(0),
									prevIndex(prev),
									nextIndex(next) 
		{
		}

		void initForUse(const typename AllocT::template PointerType<PxU8>::type& p, size_t sz, PxI32 prev, PxI32 next)
		{
			ptr = p;
			byteSize = sz;
			prevIndex = prev;
			nextIndex = next;
		}

		void initForPool(PxI32 next)
		{
			ptr = typename AllocT::template PointerType<PxU8>::type(PxZERO());
			byteSize = 0;
			prevIndex = -1;
			nextIndex = next;
		}

		typename AllocT::template PointerType<PxU8>::type	ptr;
		size_t									byteSize;
		PxI32									prevIndex;
		PxI32									nextIndex;
	};

	PxI32 allocateHole()
	{
		PxI32 retIdx;

		if (mFirstDeinitializedHoleIdx != -1)
		{
			retIdx = mFirstDeinitializedHoleIdx;
			mFirstDeinitializedHoleIdx = mHoles[(PxU32) retIdx].nextIndex;
		}
		else
		{
			mHoles.pushBack(Hole(-1, -1));
			retIdx = (PxI32) mHoles.size() - 1;
		}

		return retIdx;
	}

	void deallocateHole(PxI32 idx)
	{
		PX_ASSERT(idx < (PxI32) mHoles.size());
		
		mHoles[(PxU32) idx].initForPool(mFirstDeinitializedHoleIdx);
		mFirstDeinitializedHoleIdx = idx;
	}

	PxPair<PxI32, PxI32> findPrevAndNextHoles(const typename AllocT::template PointerType<PxU8>::type& ptr)
	{
		PxI32 i = mFirstHoleIdx;

		while (i != -1 && ptr > mHoles[(PxU32) i].ptr)
			i = mHoles[(PxU32) i].nextIndex;

		if (i == -1)
		{
			return PxPair<PxI32, PxI32>(mLastHoleIdx, -1);
		}
		else
		{
			return PxPair<PxI32, PxI32>(mHoles[(PxU32) i].prevIndex, i);
		}
	}

	PxI32 deallocateInternal(const typename AllocT::template PointerType<PxU8>::type& ptr, size_t sz) 
	{
		PxPair<PxI32, PxI32> prevAndNext = findPrevAndNextHoles(ptr);

		PxI32 newHole = -1;

		if (prevAndNext.first != -1)
		{
			Hole& prevHole = mHoles[(PxU32) prevAndNext.first];

			PX_ASSERT(prevHole.ptr + (ptrdiff_t) prevHole.byteSize <= ptr);

			if (prevHole.ptr + (ptrdiff_t) prevHole.byteSize == ptr)
			{
				prevHole.byteSize += sz;
				newHole = prevAndNext.first;
			}
		}

		if (newHole == -1)
		{
			newHole = allocateHole();
			Hole& hole = mHoles[(PxU32) newHole];
			hole.initForUse(ptr, sz, prevAndNext.first, prevAndNext.second);
			PxI32& prevIdx = prevAndNext.first != -1 ? mHoles[(PxU32) prevAndNext.first].nextIndex : mFirstHoleIdx;
			prevIdx = newHole;
			PxI32& nextIdx = prevAndNext.second != -1 ? mHoles[(PxU32) prevAndNext.second].prevIndex : mLastHoleIdx;
			nextIdx = newHole;
		}

		Hole& hole = mHoles[(PxU32) newHole];

		if (prevAndNext.second != -1)
		{
			Hole& nextHole = mHoles[(PxU32) prevAndNext.second];

			PX_ASSERT(ptr + (ptrdiff_t) sz <= nextHole.ptr);

			if (ptr + (ptrdiff_t) sz == nextHole.ptr)
			{
				hole.byteSize += nextHole.byteSize;
				hole.nextIndex = nextHole.nextIndex;
				PxI32& nextIdx = hole.nextIndex != -1 ? mHoles[(PxU32) hole.nextIndex].prevIndex : mLastHoleIdx;
				nextIdx = newHole;
				deallocateHole(prevAndNext.second);
			}
		}

		return newHole;
	}

	PxI32 addNewPage(size_t requestedAllocByteSize)
	{
		size_t sz = PxMax(requestedAllocByteSize, defaultPageBytesize);
		mPages.pushBack(castTo<PxU8>(mAlloc.allocate(sz)));
		PX_ASSERT(mPages.back() != 0);

		if (mPages.back() == 0)
		{
			mPages.popBack();

			return -1;
		}

		//by "deallocating" the newly allocated mem we put into the sorted free mem list - holes management inside
		PxI32 newHole = deallocateInternal(mPages.back(), sz); 

		return newHole;
	}

	AllocT&												mAlloc;
	PxArray<typename AllocT::template PointerType<PxU8>::type >			mPages;
	PxArray<Hole>										mHoles;
	PxI32												mFirstHoleIdx;
	PxI32												mLastHoleIdx;
	PxI32												mFirstDeinitializedHoleIdx; //pool of holes
};

template<typename AllocT, size_t defaultPageBytesize, size_t holeAlignment, typename PointedToT>
typename PxgCudaPagedFirstFitHoleAllocator<AllocT, defaultPageBytesize, holeAlignment>::template PointerType<PointedToT>::type operator+(const ptrdiff_t& argL,
							const typename PxgCudaPagedFirstFitHoleAllocator<AllocT, defaultPageBytesize, holeAlignment>::template PointerType<PointedToT>::type& argR)
{
	typename PxgCudaPagedFirstFitHoleAllocator<AllocT, defaultPageBytesize, holeAlignment>::template PointerType<PointedToT>::type ret;
	ret.ptr = argR.ptr + argL;
	ret.sz = (size_t) -1;
	
	return ret;
}

}

#endif
