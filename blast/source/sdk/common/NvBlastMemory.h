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


#ifndef NVBLASTMEMORY_H
#define NVBLASTMEMORY_H

#include <math.h>

namespace Nv
{
namespace Blast
{


/**
Utility function to align the given value to the next 16-byte boundary.

Returns the aligned value.
*/
template<typename T>
NV_INLINE T align16(T value)
{
    return (value + 0xF)&~(T)0xF;
}


/** Offset void* pointer by 'offset' bytes helper-functions */

template <typename T>
NV_INLINE T pointerOffset(void* p, ptrdiff_t offset)
{
    return reinterpret_cast<T>(reinterpret_cast<char*>(p)+offset);
}

template <typename T>
NV_INLINE T pointerOffset(const void* p, ptrdiff_t offset)
{
    return reinterpret_cast<T>(reinterpret_cast<const char*>(p)+offset);
}

NV_INLINE const void* pointerOffset(const void* p, ptrdiff_t offset)
{
    return pointerOffset<const void*>(p, offset);
}

NV_INLINE void* pointerOffset(void* p, ptrdiff_t offset)
{
    return pointerOffset<void*>(p, offset);
}

} // namespace Blast
} // namespace Nv


/** Block data offset and accessor macro. */
#define NvBlastBlockData(_dataType, _name, _accessor) \
_dataType* _accessor() const \
{ \
    return (_dataType*)((uintptr_t)this + _name); \
} \
uint32_t _name


/** Block data offset and accessor macro for an array (includes an _accessor##ArraySize() function which returns the last expression). */
#define NvBlastBlockArrayData(_dataType, _name, _accessor, _sizeExpr) \
_dataType* _accessor() const \
{ \
    return (_dataType*)((uintptr_t)this + _name); \
} \
uint32_t _accessor##ArraySize() const \
{ \
    return _sizeExpr; \
} \
uint32_t _name


/** Block data offset generation macros. */

/** Start offset generation with this. */
#define NvBlastCreateOffsetStart(_baseOffset) \
size_t _lastOffset = _baseOffset; \
size_t _lastSize = 0

/** Create the next offset generation with this.  The value will be aligned to a 16-byte boundary. */
#define NvBlastCreateOffsetAlign16(_name, _size) \
_name = align16(_lastOffset + _lastSize); \
_lastOffset = _name; \
_lastSize = _size

/** End offset generation with this.  It evaluates to the (16-byte aligned) total size of the data block. */
#define NvBlastCreateOffsetEndAlign16() \
align16(_lastOffset + _lastSize)


/** Stack allocation */
#if NV_WINDOWS_FAMILY
#include <malloc.h>
#define NvBlastAlloca(x) _alloca(x)
#elif NV_LINUX || NV_ANDROID
#include <alloca.h>
#define NvBlastAlloca(x) alloca(x)
#elif NV_APPLE_FAMILY
#include <alloca.h>
#define NvBlastAlloca(x) alloca(x)
#endif

#endif // #ifndef NVBLASTMEMORY_H
