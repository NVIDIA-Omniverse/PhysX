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

#ifndef PX_PINNED_BITMAP_H
#define PX_PINNED_BITMAP_H

#include "foundation/PxBitMap.h"
#include "foundation/PxPinnedArray.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/*!
Overloaded allocation helper for pinned memory support
*/
PX_INLINE void* PxBitMapAlloc(PxPinnedAllocator<PxU32>& a, PxU32 bytes, const char* file, int line, uint32_t* cookie)
{
	return a.allocate(bytes, file, line, cookie);
}

/*!
Overloaded de-allocation helper for pinned memory support
*/
PX_INLINE void PxBitMapDealloc(PxPinnedAllocator<PxU32>& a, void* ptr, uint32_t* cookie)
{
	a.deallocate(ptr, cookie);
}

/*!
As opposed to PxPinnedArray, PxBitMapPinned has non-pinned allocation fallback, 
same as PxPinnedArraySafe.
*/
typedef PxBitMapBase< PxPinnedAllocator<PxU32> > PxBitMapPinned;


#if !PX_DOXYGEN
} // namespace physx
#endif

#endif // PX_PINNED_BITMAP_H
