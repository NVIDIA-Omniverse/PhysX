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

#include "extensions/PxSerialization.h"
#include "foundation/PxPhysicsVersion.h"

#include "SnSerialUtils.h"
#include "foundation/PxString.h"
#include "foundation/PxBasicTemplates.h"

using namespace physx;

namespace
{

#define SN_NUM_BINARY_PLATFORMS 9
const PxU32 sBinaryPlatformTags[SN_NUM_BINARY_PLATFORMS] =
{
	PX_MAKE_FOURCC('W','_','3','2'),
	PX_MAKE_FOURCC('W','_','6','4'),
	PX_MAKE_FOURCC('L','_','3','2'),
	PX_MAKE_FOURCC('L','_','6','4'),
	PX_MAKE_FOURCC('M','_','3','2'),
	PX_MAKE_FOURCC('M','_','6','4'),
	PX_MAKE_FOURCC('N','X','3','2'),
	PX_MAKE_FOURCC('N','X','6','4'),
	PX_MAKE_FOURCC('L','A','6','4')
};

const char* sBinaryPlatformNames[SN_NUM_BINARY_PLATFORMS] =
{
	"win32",
	"win64",
	"linux32",
	"linux64",
	"mac32",
	"mac64",
	"switch32",
	"switch64",
	"linuxaarch64"
};

}

namespace physx { namespace Sn {

PxU32 getBinaryPlatformTag()
{
#if PX_WINDOWS && PX_X86
	return sBinaryPlatformTags[0];
#elif PX_WINDOWS && PX_X64
	return sBinaryPlatformTags[1];
#elif PX_LINUX && PX_X86
	return sBinaryPlatformTags[2];
#elif PX_LINUX && PX_X64
	return sBinaryPlatformTags[3];
#elif PX_OSX && PX_X86
	return sBinaryPlatformTags[4];
#elif PX_OSX && PX_X64
	return sBinaryPlatformTags[5];
#elif PX_SWITCH && !PX_A64
	return sBinaryPlatformTags[6];
#elif PX_SWITCH && PX_A64
	return sBinaryPlatformTags[7];
#elif PX_LINUX && PX_A64
	return sBinaryPlatformTags[8];
#else
	#error Unknown binary platform
#endif
}

bool isBinaryPlatformTagValid(physx::PxU32 platformTag)
{
	PxU32 platformIndex = 0;
	while (platformIndex < SN_NUM_BINARY_PLATFORMS && platformTag != sBinaryPlatformTags[platformIndex]) platformIndex++;
	return platformIndex < SN_NUM_BINARY_PLATFORMS;
}

const char* getBinaryPlatformName(physx::PxU32 platformTag)
{
	PxU32 platformIndex = 0;
	while (platformIndex < SN_NUM_BINARY_PLATFORMS && platformTag != sBinaryPlatformTags[platformIndex]) platformIndex++;
	return (platformIndex == SN_NUM_BINARY_PLATFORMS) ? "unknown" : sBinaryPlatformNames[platformIndex];
}

const char* getBinaryVersionGuid()
{
	PX_COMPILE_TIME_ASSERT(sizeof(PX_BINARY_SERIAL_VERSION) == SN_BINARY_VERSION_GUID_NUM_CHARS + 1);
	return PX_BINARY_SERIAL_VERSION;
}

bool checkCompatibility(const char* binaryVersionGuidCandidate)
{
	for(PxU32 i=0; i<SN_BINARY_VERSION_GUID_NUM_CHARS; i++)
	{
		if (binaryVersionGuidCandidate[i] != PX_BINARY_SERIAL_VERSION[i])
		{
			return false;
		}
	}
	return true;
}

} // Sn
} // physx

