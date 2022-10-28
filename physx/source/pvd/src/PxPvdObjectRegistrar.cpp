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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "PxPvdObjectRegistrar.h"

namespace physx
{
namespace pvdsdk
{

bool ObjectRegistrar::addItem(const void* inItem)
{
	physx::PxMutex::ScopedLock lock(mRefCountMapLock);

	if(mRefCountMap.find(inItem))
	{
		uint32_t& counter = mRefCountMap[inItem];
		counter++;
		return false;
	}
	else
	{
		mRefCountMap.insert(inItem, 1);
		return true;
	}
}

bool ObjectRegistrar::decItem(const void* inItem)
{
	physx::PxMutex::ScopedLock lock(mRefCountMapLock);
	const physx::PxHashMap<const void*, uint32_t>::Entry* entry = mRefCountMap.find(inItem);
	if(entry)
	{
		uint32_t& retval(const_cast<uint32_t&>(entry->second));
		if(retval)
			--retval;
		uint32_t theValue = retval;
		if(theValue == 0)
		{
			mRefCountMap.erase(inItem);
			return true;
		}
	}
	return false;
}

void ObjectRegistrar::clear()
{
	physx::PxMutex::ScopedLock lock(mRefCountMapLock);
	mRefCountMap.clear();
}

} // pvdsdk
} // physx
