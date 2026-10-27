// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/windows/PxWindowsInclude.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxSync.h"

using namespace physx;

static PX_FORCE_INLINE HANDLE& getSync(PxSyncImpl* impl)
{
	return *reinterpret_cast<HANDLE*>(impl);
}

uint32_t PxSyncImpl::getSize()
{
	return sizeof(HANDLE);
}

PxSyncImpl::PxSyncImpl()
{
	getSync(this) = CreateEvent(0, true, false, 0);
}

PxSyncImpl::~PxSyncImpl()
{
	CloseHandle(getSync(this));
}

void PxSyncImpl::reset()
{
	ResetEvent(getSync(this));
}

void PxSyncImpl::set()
{
	SetEvent(getSync(this));
}

bool PxSyncImpl::wait(uint32_t milliseconds)
{
	if(milliseconds == -1)
		milliseconds = INFINITE;

	return WaitForSingleObject(getSync(this), milliseconds) == WAIT_OBJECT_0;
}

