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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "foundation/PxMath.h"
#include "foundation/PxIntrinsics.h"
#include "foundation/PxBitUtils.h"
#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxTempAllocator.h"
#include "FdFoundation.h"

#if PX_VC
#pragma warning(disable : 4706) // assignment within conditional expression
#endif

namespace physx
{
namespace
{
typedef PxTempAllocatorChunk Chunk;
typedef PxArray<Chunk*, PxAllocator> AllocFreeTable;

PX_INLINE Foundation::AllocFreeTable& getFreeTable()
{
	return getFoundation().getTempAllocFreeTable();
}
PX_INLINE Foundation::Mutex& getMutex()
{
	return getFoundation().getTempAllocMutex();
}

const PxU32 sMinIndex = 8;  // 256B min
const PxU32 sMaxIndex = 17; // 128kB max
}

void* PxTempAllocator::allocate(size_t size, const char* filename, PxI32 line)
{
	if(!size)
		return 0;

	PxU32 index = PxMax(PxHighestSetBit(PxU32(size) + sizeof(Chunk) - 1), sMinIndex);

	Chunk* chunk = 0;
	if(index < sMaxIndex)
	{
		Foundation::Mutex::ScopedLock lock(getMutex());

		// find chunk up to 16x bigger than necessary
		Chunk** it = getFreeTable().begin() + index - sMinIndex;
		Chunk** end = PxMin(it + 3, getFreeTable().end());
		while(it < end && !(*it))
			++it;

		if(it < end)
		{
			// pop top off freelist
			chunk = *it;
			*it = chunk->mNext;
			index = PxU32(it - getFreeTable().begin() + sMinIndex);
		}
		else
			// create new chunk
			chunk = reinterpret_cast<Chunk*>(PxAllocator().allocate(size_t(2 << index), filename, line));
	}
	else
	{
		// too big for temp allocation, forward to base allocator
		chunk = reinterpret_cast<Chunk*>(PxAllocator().allocate(size + sizeof(Chunk), filename, line));
	}

	chunk->mIndex = index;
	void* ret = chunk + 1;
	PX_ASSERT((size_t(ret) & 0xf) == 0); // SDK types require at minimum 16 byte alignment.
	return ret;
}

void PxTempAllocator::deallocate(void* ptr)
{
	if(!ptr)
		return;

	Chunk* chunk = reinterpret_cast<Chunk*>(ptr) - 1;
	PxU32 index = chunk->mIndex;

	if(index >= sMaxIndex)
		return PxAllocator().deallocate(chunk);

	Foundation::Mutex::ScopedLock lock(getMutex());

	index -= sMinIndex;
	if(getFreeTable().size() <= index)
		getFreeTable().resize(index + 1);

	chunk->mNext = getFreeTable()[index];
	getFreeTable()[index] = chunk;
}

} // namespace physx
