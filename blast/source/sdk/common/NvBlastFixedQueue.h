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


#ifndef NVBLASTFIXEDQUEUE_H
#define NVBLASTFIXEDQUEUE_H

#include "NvBlastAssert.h"
#include "NvBlastMemory.h"

namespace Nv
{
namespace Blast
{

/*!
FixedQueue is a queue container which is intended to be used with placement new on chunk of memory.
It'll use following memory for data layout. As follows:

// some memory
char ​*buf = new char[64 *​ 1024];

// placement new on this memory
FixedQueue<SomeClass>* arr = new (buf) FixedQueue<SomeClass>();

// you can get max requiredMemorySize by an array of 'capacity' elements count to use memory left
buf = buf + FixedQueue<SomeClass>::requiredMemorySize(capacity);

*/
template <class T>
class FixedQueue
{
public:
    explicit FixedQueue(uint32_t maxEntries) : m_num(0), m_head(0), m_tail(0), m_maxEntries(maxEntries)
    {
    }

    static size_t requiredMemorySize(uint32_t capacity)
    {
        return align16(sizeof(FixedQueue<T>)) + align16(capacity * sizeof(T));
    }

    T popFront()
    {
        NVBLAST_ASSERT(m_num>0);

        m_num--;
        T& element = data()[m_tail];
        m_tail = (m_tail+1) % (m_maxEntries);
        return element;
    }

    T front()
    {
        NVBLAST_ASSERT(m_num>0);

        return data()[m_tail];
    }

    T popBack()
    {
        NVBLAST_ASSERT(m_num>0);

        m_num--;
        m_head = (m_head-1) % (m_maxEntries);
        return data()[m_head];
    }

    T back()
    {
        NVBLAST_ASSERT(m_num>0);

        uint32_t headAccess = (m_head-1) % (m_maxEntries);
        return data()[headAccess];
    }

    bool pushBack(const T& element)
    {
        if (m_num == m_maxEntries) return false;
        data()[m_head] = element;

        m_num++;
        m_head = (m_head+1) % (m_maxEntries);

        return true;
    }

    bool empty() const
    {
        return m_num == 0;
    }

    uint32_t size() const
    {
        return m_num;
    }   


private:
    uint32_t        m_num;
    uint32_t        m_head;
    uint32_t        m_tail;
    uint32_t        m_maxEntries;

    T* data()
    {
        return (T*)((char*)this + sizeof(FixedQueue<T>));
    }

private:
    FixedQueue(const FixedQueue& that);
};

} // namespace Blast
} // namespace Nv

#endif // ifndef NVBLASTFIXEDQUEUE_H
