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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#ifndef NVBLASTFIXEDARRAY_H
#define NVBLASTFIXEDARRAY_H

#include "NvBlastAssert.h"
#include "NvBlastMemory.h"

namespace Nv
{
namespace Blast
{

/*!
FixedArray is a sequential container which is intended to be used with placement new on chunk of memory.
It'll use following memory for data layout. As follows:

// some memory
char ​*buf = new char[64 *​ 1024];

// placement new on this memory
FixedArray<SomeClass>* arr = new (buf) FixedArray<SomeClass>();

// you can get max requiredMemorySize by an array of 'capacity' elements count to use memory left
buf = buf + FixedArray<SomeClass>::requiredMemorySize(capacity);

buf:

+------------------------------------------------------------+
|  uint32_t  |  T[0]  |  T[1]  |  T[2]  |         ...        |
+------------------------------------------------------------+


!!!TODO:
- check ctor/dtor of elements calls
*/
template <class T>
class FixedArray
{
public:
    explicit FixedArray() : m_size(0)
    {
    }

    static size_t requiredMemorySize(uint32_t capacity)
    {
        return align16(sizeof(FixedArray<T>)) + align16(capacity * sizeof(T));
    }

    NV_FORCE_INLINE T& pushBack(T& t)
    {
        new (data() + m_size) T(t);
        return data()[m_size++];
    }

    T popBack()
    {
        NVBLAST_ASSERT(m_size);
        T t = data()[m_size - 1];
        data()[--m_size].~T();
        return t;
    }

    void clear()
    {
        for(T* first = data(); first < data() + m_size; ++first)
            first->~T();
        m_size = 0;
    }

    NV_FORCE_INLINE void forceSize_Unsafe(uint32_t s)
    {
        m_size = s;
    }

    NV_FORCE_INLINE T& operator[](uint32_t idx)
    {
        NVBLAST_ASSERT(idx < m_size);
        return data()[idx];
    }

    NV_FORCE_INLINE const T& operator[](uint32_t idx) const
    {
        NVBLAST_ASSERT(idx < m_size);
        return data()[idx];
    }

    NV_FORCE_INLINE T& at(uint32_t idx)
    {
        NVBLAST_ASSERT(idx < m_size);
        return data()[idx];
    }

    NV_FORCE_INLINE const T& at(uint32_t idx) const
    {
        NVBLAST_ASSERT(idx < m_size);
        return data()[idx];
    }

    NV_FORCE_INLINE uint32_t size() const
    {
        return m_size;
    }

private:
    uint32_t m_size;

    NV_FORCE_INLINE T* data()
    {
        return (T*)((char*)this + sizeof(FixedArray<T>));
    }

private:
    FixedArray(const FixedArray& that);
};

} // namespace Blast
} // namespace Nv

#endif // ifndef NVBLASTFIXEDARRAY_H
