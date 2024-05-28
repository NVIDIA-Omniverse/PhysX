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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PXC_NP_CACHE_H
#define PXC_NP_CACHE_H

#include "foundation/PxMemory.h"
#include "foundation/PxIntrinsics.h"
#include "foundation/PxPool.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxUtilities.h"

#include "PxcNpCacheStreamPair.h"

#include "GuContactMethodImpl.h"

namespace physx
{

PX_FORCE_INLINE void PxcNpCacheReserve(PxcNpCacheStreamPair& streams, Gu::Cache& cache, PxU32 bytes)
{
	bool sizeTooLarge;
	PxU8* ls = streams.reserve(bytes, sizeTooLarge);
	cache.mCachedData = ls;
	
	if(sizeTooLarge)
	{
		// PT: TODO: consider changing the error message, it will silently become obsolete if we change the value of PxcNpMemBlock::SIZE.
		// On the other hand the PxSceneDesc::maxNbContactDataBlocks also hardcodes "16K data blocks" so this isn't urgent.
		PX_WARN_ONCE(
			"Attempting to allocate more than 16K of contact data for a single contact pair in narrowphase. "
			"Either accept dropped contacts or simplify collision geometry.");
	}
	else if(ls==NULL)
	{
		PX_WARN_ONCE(
			"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for narrow phase. "
			"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.");
	}
}

template <typename T>
void PxcNpCacheWrite(PxcNpCacheStreamPair& streams,
					 Gu::Cache& cache,
					 const T& payload,
					 PxU32 bytes, 
					 const PxU8* data)
{
	PxU8* ls = PxcNpCacheWriteInitiate(streams, cache, payload, bytes);

	if (ls == NULL)
		return;

	PxcNpCacheWriteFinalize(ls, payload, bytes, data);
}


template <typename T>
PxU8* PxcNpCacheWriteInitiate(PxcNpCacheStreamPair& streams, Gu::Cache& cache, const T& payload, PxU32 bytes)
{
	const PxU32 payloadSize = (sizeof(payload)+3)&~3;
	cache.mCachedSize = PxTo16((payloadSize + 4 + bytes + 0xF)&~0xF);

	PxcNpCacheReserve(streams, cache, cache.mCachedSize);

	return cache.mCachedData;
}

template <typename T>
PX_FORCE_INLINE void PxcNpCacheWriteFinalize(PxU8* ls, const T& payload, PxU32 bytes, const PxU8* data)
{
	const PxU32 payloadSize = (sizeof(payload)+3)&~3;
	*reinterpret_cast<T*>(ls) = payload;
	*reinterpret_cast<PxU32*>(ls+payloadSize) = bytes;
	if(data)
		PxMemCopy(ls+payloadSize+sizeof(PxU32), data, bytes);
}

template <typename T>
PX_FORCE_INLINE PxU8* PxcNpCacheRead(Gu::Cache& cache, T*& payload)
{
	PxU8* ls = cache.mCachedData;
	payload = reinterpret_cast<T*>(ls);
	const PxU32 payloadSize = (sizeof(T)+3)&~3;
	return reinterpret_cast<PxU8*>(ls+payloadSize+sizeof(PxU32));
}

template <typename T>
const PxU8* PxcNpCacheRead2(Gu::Cache& cache, T& payload, PxU32& bytes)
{
	const PxU8* ls = cache.mCachedData;
	if(ls==NULL)
	{
		bytes = 0;
		return NULL;
	}

	const PxU32 payloadSize = (sizeof(payload)+3)&~3;
	payload = *reinterpret_cast<const T*>(ls);
	bytes = *reinterpret_cast<const PxU32*>(ls+payloadSize);
	PX_ASSERT(cache.mCachedSize == ((payloadSize + 4 + bytes+0xF)&~0xF));
	return reinterpret_cast<const PxU8*>(ls+payloadSize+sizeof(PxU32));
}

}

#endif
