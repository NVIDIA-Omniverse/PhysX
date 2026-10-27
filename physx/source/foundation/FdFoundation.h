// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_FOUNDATION_PSFOUNDATION_H
#define PX_FOUNDATION_PSFOUNDATION_H

#include "foundation/PxAllocator.h"
#include "foundation/PxArray.h"
#include "foundation/PxMutex.h"

namespace physx
{
	union PxTempAllocatorChunk;

	typedef PxMutexT<PxAllocator> Mutex;
	typedef PxArray<PxTempAllocatorChunk*, PxAllocator> AllocFreeTable;

} // namespace physx


#endif
