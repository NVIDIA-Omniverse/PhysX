// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
