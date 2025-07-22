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

