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


#ifndef NVBLASTFIXEDBOOLARRAY_H
#define NVBLASTFIXEDBOOLARRAY_H

#include "NvBlastAssert.h"
#include "NvBlastMemory.h"
#include <cstring>

namespace Nv
{
namespace Blast
{

/*!
FixedBoolArray is an array of bools of fixed size, it's intended to be used with placement new on chunk of memory.
It'll use following memory for data layout. As follows:

// some memory
char ​*buf = new char[64 *​ 1024];

const uint32_t size = 100;

// placement new on this memory
FixedBoolArray* arr = new (buf) FixedBoolArray(size);

// you can get max requiredMemorySize by an bitMap to use memory left
buf = buf + FixedBoolArray<SomeClass>::requiredMemorySize(size);

buf:

+------------------------------------------------------------+
|  uint32_t  |  bool0  |  bool1  |  bool2  |       ...       |
+------------------------------------------------------------+

*/
class FixedBoolArray
{
public:
    explicit FixedBoolArray(uint32_t size)
    {
        m_size = size;
    }

    static size_t requiredMemorySize(uint32_t size)
    {
        return align16(sizeof(FixedBoolArray)) + align16(size);
    }

    void clear()
    {
        memset(data(), 0, m_size);
    }

    void fill()
    {
        memset(data(), 1, m_size);
    }

    int test(uint32_t index) const
    {
        NVBLAST_ASSERT(index < m_size);
        return data()[index];
    }

    void set(uint32_t index)
    {
        NVBLAST_ASSERT(index < m_size);
        data()[index] = 1;
    }

    void setData(const char* newData, uint32_t newSize)
    {
        m_size = newSize;
        memcpy(data(), newData, m_size);
    }

    const char* getData() const
    {
        return data();
    }

    uint32_t getSize() const
    {
        return m_size;
    }

    void reset(uint32_t index)
    {
        NVBLAST_ASSERT(index < m_size);
        data()[index] = 0;
    }

private:
    uint32_t m_size;

    NV_FORCE_INLINE char* data()
    {
        return ((char*)this + sizeof(FixedBoolArray));
    }

    NV_FORCE_INLINE const char* data() const
    {
        return ((char*)this + sizeof(FixedBoolArray));
    }

private:
    FixedBoolArray(const FixedBoolArray& that);
};

} // namespace Blast
} // namespace Nv

#endif // ifndef NVBLASTFIXEDBOOLARRAY_H
