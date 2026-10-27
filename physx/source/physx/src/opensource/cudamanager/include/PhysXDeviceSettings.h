// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PHYSX_DEVICE_SETTINGS_H
#define PHYSX_DEVICE_SETTINGS_H

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

namespace physx
{
	class PxErrorCallback;

	/**
	Helper functions to expose control panel functionality 
	*/
	class PhysXDeviceSettings
	{
	private:
		PhysXDeviceSettings() {}

	public:
		static int getSuggestedCudaDeviceOrdinal(PxErrorCallback& errc)
		{
			int deviceOrdinal;
			const char* deviceOrdinalString = ::getenv("PHYSX_GPU_DEVICE");
			if (!deviceOrdinalString)
				deviceOrdinal = 0;	// Set the default to the first CUDA capable device
			else
				deviceOrdinal = atoi(deviceOrdinalString);

			if (deviceOrdinal < 0)
				errc.reportError(PxErrorCode::eDEBUG_WARNING, "Invalid PhysX CUDA device ordinal\n", PX_FL);

			return deviceOrdinal;
		}
	};
}

#endif

#endif

