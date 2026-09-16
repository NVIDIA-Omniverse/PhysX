// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/windows/PxWindowsInclude.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxSList.h"

using namespace physx;

template <typename T>
static PX_FORCE_INLINE SLIST_HEADER* getDetail(T* impl)
{
	return reinterpret_cast<SLIST_HEADER*>(impl);
}

PxSListImpl::PxSListImpl()
{
	InitializeSListHead(getDetail(this));
}

PxSListImpl::~PxSListImpl()
{
}

void PxSListImpl::push(PxSListEntry* entry)
{
	InterlockedPushEntrySList(getDetail(this), reinterpret_cast<SLIST_ENTRY*>(entry));
}

PxSListEntry* PxSListImpl::pop()
{
	return reinterpret_cast<PxSListEntry*>(InterlockedPopEntrySList(getDetail(this)));
}

PxSListEntry* PxSListImpl::flush()
{
	return reinterpret_cast<PxSListEntry*>(InterlockedFlushSList(getDetail(this)));
}

uint32_t PxSListImpl::getSize()
{
	return sizeof(SLIST_HEADER);
}
