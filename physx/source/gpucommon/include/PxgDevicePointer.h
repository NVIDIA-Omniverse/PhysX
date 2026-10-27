// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_DEVICE_POINTER_H
#define PXG_DEVICE_POINTER_H

#include "foundation/PxPreprocessor.h"

namespace physx
{
    
	//This should be a basic pointer wrapper that has the same memory footprint as a raw pointer. Please don't add additional members to the struct.
	template <typename T>
	struct PxgDevicePointer
	{
		CUdeviceptr mPtr;

		PxgDevicePointer(CUdeviceptr ptr) : mPtr(ptr) {}

		operator CUdeviceptr& () { return mPtr; }
		operator CUdeviceptr() const { return mPtr; }

		T* getPointer() const { return reinterpret_cast<T*>(mPtr); }
	};
    
	PX_COMPILE_TIME_ASSERT(sizeof(PxgDevicePointer<PxU32>) == sizeof(CUdeviceptr));
}

#endif