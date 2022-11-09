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
// Copyright (c) 2014-2022 NVIDIA Corporation. All rights reserved.

#pragma once

#define NV_FLOW_ARRAY_CACHE_ENABLED 1

#include <new>
#include <utility>

template<class T, NvFlowUint64 staticCapacity = 0u, void(prerelease)(void* data, NvFlowUint64 size) = nullptr>
struct NvFlowArray
{
	#if NV_FLOW_ARRAY_CACHE_ENABLED
	static const NvFlowUint64 s_staticCapacity = staticCapacity;
	#else
	static const NvFlowUint64 s_staticCapacity = 0u;
	#endif

	T* data = nullptr;
	NvFlowUint64 capacity = 0u;
	NvFlowUint64 size = 0u;	
	unsigned char cache[s_staticCapacity * sizeof(T) + 8u];

	void release()
	{
		for (NvFlowUint64 i = 0; i < capacity; i++)
		{
			data[i].~T();
		}
		if (data != nullptr && (T*)cache != data)
		{
			operator delete[](data);
		}
		data = nullptr;
		capacity = 0u;
		size = 0u;
	}

	void move(NvFlowArray& rhs)
	{
		data = rhs.data;
		capacity = rhs.capacity;
		size = rhs.size;
		if (rhs.data == (T*)rhs.cache)
		{
			data = (T*)cache;
			for (NvFlowUint64 idx = 0u; idx < capacity; idx++)
			{
				new(data + idx) T(std::move(rhs.data[idx]));
			}
		}
		// to match destructed state
		rhs.data = nullptr;
		rhs.capacity = 0u;
		rhs.size = 0u;
	}

	void reserve(NvFlowUint64 requestedCapacity)
	{
		if (requestedCapacity <= capacity) 
		{
			return;
		}

		NvFlowUint64 newSize = size;
		NvFlowUint64 newCapacity = capacity;
		if (newCapacity < s_staticCapacity)
		{
			newCapacity = s_staticCapacity;
		}
		if (newCapacity == 0u)
		{
			newCapacity = 1u;
		}
		while (newCapacity < requestedCapacity)
		{
			newCapacity *= 2u;
		}

		T* newData = (T*)(newCapacity <= s_staticCapacity ? (void*)cache : operator new[](newCapacity * sizeof(T)));
		// copy to new
		for (NvFlowUint64 i = 0; i < newSize; i++)
		{
			new(newData + i) T(std::move(data[i]));
		}
		for (NvFlowUint64 i = newSize; i < newCapacity; i++)
		{
			new(newData + i) T();
		}
		if (prerelease)
		{
			prerelease(data + size, capacity - size);
		}
		// cleanup old
		release();
		// commit new
		data = newData;
		capacity = newCapacity;
		size = newSize;
	}

	NvFlowArray()
	{
		reserve(s_staticCapacity);
	}

	NvFlowArray(NvFlowArray&& rhs)
	{
		move(rhs);
	}

	~NvFlowArray()
	{
		if (prerelease)
		{
			prerelease(data, capacity);
		}
		release();
	}

	T& operator[](NvFlowUint64 idx)
	{
		return data[idx];
	}

	const T& operator[](NvFlowUint64 idx) const
	{
		return data[idx];
	}

	NvFlowUint64 allocateBack()
	{
		reserve(size + 1);
		size++;
		return size - 1;
	}
	
	void pushBack(const T& v)
	{
		operator[](allocateBack()) = v;
	}

	T& back()
	{
		return operator[](size - 1);
	}

	void popBack()
	{
		size--;
	}
};

/// Copy utility
template <class T, NvFlowUint64 staticCapacity = 0u, void(prerelease)(void* data, NvFlowUint64 size) = nullptr>
NV_FLOW_INLINE void NvFlowArray_copy(NvFlowArray<T, staticCapacity, prerelease>& dst, const NvFlowArray<T, staticCapacity, prerelease>& src)
{
	dst.size = 0u;
	dst.reserve(src.size);
	dst.size = src.size;
	for (NvFlowUint64 idx = 0u; idx < dst.size; idx++)
	{
		dst[idx] = src[idx];
	}
}

template<class T>
NV_FLOW_INLINE void NvFlowArrayPointer_prerelease(void* dataIn, NvFlowUint64 size)
{
	T* data = (T*)dataIn;
	for (NvFlowUint64 idx = 0u; idx < size; idx++)
	{
		if (data[idx])
		{
			delete data[idx];
			data[idx] = nullptr;
		}
	}
}

template<class T>
NV_FLOW_INLINE void NvFlowArrayPointer_allocate(T*& ptr)
{
	ptr = new T();
}

template<class T, NvFlowUint64 staticCapacity = 0u>
struct NvFlowArrayPointer : public NvFlowArray<T, staticCapacity, NvFlowArrayPointer_prerelease<T>>
{
	NvFlowArrayPointer() : NvFlowArray<T, staticCapacity, NvFlowArrayPointer_prerelease<T>>()
	{
	}
	NvFlowArrayPointer(NvFlowArrayPointer&& rhs) : NvFlowArray<T, staticCapacity, NvFlowArrayPointer_prerelease<T>>(std::move(rhs))
	{
	}
	~NvFlowArrayPointer()
	{
	}
	T allocateBackPointer()
	{
		NvFlowUint64 allocIdx = this->allocateBack();
		if (!(*this)[allocIdx])
		{
			NvFlowArrayPointer_allocate((*this)[allocIdx]);
		}
		return (*this)[allocIdx];
	}
	void pushBackPointer(const T& v)
	{
		NvFlowUint64 allocIdx = this->allocateBack();
		deletePointerAtIndex(allocIdx);
		(*this)[allocIdx] = v;
	}
	void swapPointers(NvFlowUint64 idxA, NvFlowUint64 idxB)
	{
		T temp = (*this)[idxA];
		(*this)[idxA] = (*this)[idxB];
		(*this)[idxB] = temp;
	}
	void removeSwapPointerAtIndex(NvFlowUint64 idx)
	{
		swapPointers(idx, this->size - 1u);
		this->size--;
	}
	void removeSwapPointer(T ptr)
	{
		for (NvFlowUint64 idx = 0u; idx < this->size; idx++)
		{
			if ((*this)[idx] == ptr)
			{
				removeSwapPointerAtIndex(idx);
				break;
			}
		}
	}
	void deletePointerAtIndex(NvFlowUint64 idx)
	{
		if ((*this)[idx])
		{
			delete (*this)[idx];
			(*this)[idx] = nullptr;
		}
	}
	void deletePointers()
	{
		this->size = this->capacity;
		for (NvFlowUint64 idx = 0u; idx < this->size; idx++)
		{
			deletePointerAtIndex(idx);
		}
		this->size = 0u;
	}
};

template<class T, NvFlowUint64 staticCapacity = 0u>
struct NvFlowRingBufferPointer
{
	NvFlowArrayPointer<T, staticCapacity> arr;
	NvFlowUint64 freeIdx = 0u;
	NvFlowUint64 frontIdx = 0u;
	NvFlowUint64 backIdx = 0u;

	NvFlowRingBufferPointer() : arr()
	{
	}
	NvFlowRingBufferPointer(NvFlowRingBufferPointer&& rhs) : 
		arr(std::move(rhs.arr)),
		freeIdx(rhs.freeIdx),
		frontIdx(rhs.frontIdx), 
		backIdx(rhs.backIdx)
	{
	}
	~NvFlowRingBufferPointer()
	{
	}

	T& front()
	{
		return arr[frontIdx];
	}

	T& back()
	{
		return arr[(backIdx - 1u) & (arr.size - 1)];
	}

	NvFlowUint64 activeCount()
	{
		return (backIdx - frontIdx) & (arr.size - 1);
	}

	NvFlowUint64 freeCount()
	{
		return (frontIdx - freeIdx) & (arr.size - 1);
	}

	void popFront()
	{
		frontIdx = (frontIdx + 1u) & (arr.size - 1);
	}

	void popFree()
	{
		freeIdx = (freeIdx + 1u) & (arr.size - 1);
	}

	T& operator[](NvFlowUint64 idx)
	{
		return arr[(frontIdx + idx) & (arr.size - 1)];
	}

	const T& operator[](NvFlowUint64 idx) const
	{
		return arr[(frontIdx + idx) & (arr.size - 1)];
	}

	NvFlowUint64 allocateBack()
	{
		if (arr.size == 0u)
		{
			arr.allocateBack();
		}
		if (freeCount() > 0u)
		{
			auto tmp = arr[freeIdx];
			arr[freeIdx] = arr[backIdx];
			arr[backIdx] = tmp;
			popFree();
		}
		else if ((activeCount() + 1u) > (arr.size - 1))
		{
			NvFlowUint64 oldSize = arr.size;
			arr.reserve(2u * oldSize);
			arr.size = 2u * oldSize;
			if (backIdx < frontIdx)
			{
				for (NvFlowUint64 idx = 0u; idx < backIdx; idx++)
				{
					auto tmp = arr[idx + oldSize];
					arr[idx + oldSize] = arr[idx];
					arr[idx] = tmp;
				}
				backIdx += oldSize;
			}
		}
		NvFlowUint64 allocIdx = backIdx;
		backIdx = (backIdx + 1u) & (arr.size - 1);
		return allocIdx;
	}

	void pushBack(const T& v)
	{
		NvFlowUint64 allocIdx = allocateBack();
		arr.deletePointerAtIndex(allocIdx);
		arr[allocIdx] = v;
	}

	T allocateBackPointer()
	{
		NvFlowUint64 allocIdx = allocateBack();
		if (!arr[allocIdx])
		{
			NvFlowArrayPointer_allocate(arr[allocIdx]);
		}
		return arr[allocIdx];
	}

	void deletePointers()
	{
		arr.deletePointers();
	}
};