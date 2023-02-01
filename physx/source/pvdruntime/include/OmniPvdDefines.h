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

#ifndef OMNI_PVD_DEFINES_H
#define OMNI_PVD_DEFINES_H

#define OMNI_PVD_VERSION_MAJOR 0
#define OMNI_PVD_VERSION_MINOR 3
#define OMNI_PVD_VERSION_PATCH 0

////////////////////////////////////////////////////////////////////////////////
// Versions so far : (major, minor, patch), top one is newest
//
// [0, 3,  0]
//   writes/read out the base class handle in the class registration call
//   backwards compatible with [0, 2, 0] and [0, 1, 42]
// [0, 2,  0]
//   intermediate version was never official, no real change, but there are many files with this version
// [0, 1, 42]
//   no proper base class written/read in the class registration call
//
////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#define OMNI_PVD_WIN
#define OMNI_PVD_CALL __cdecl
#define OMNI_PVD_EXPORT extern "C" __declspec(dllexport)
#else
#ifdef __cdecl__
	#define OMNI_PVD_CALL __attribute__((__cdecl__))
#else
	#define OMNI_PVD_CALL
#endif
#if __GNUC__ >= 4
	#define OMNI_PVD_EXPORT extern "C" __attribute__((visibility("default")))
	#else
	#define OMNI_PVD_EXPORT extern "C"
#endif
#endif

typedef uint64_t OmniPvdObjectHandle;
typedef uint64_t OmniPvdContextHandle;
typedef uint32_t OmniPvdClassHandle;
typedef uint32_t OmniPvdAttributeHandle;
typedef uint16_t OmniPvdAttributeDataType;
typedef uint32_t OmniPvdVersionType;

typedef void (OMNI_PVD_CALL *OmniPvdLogFunction)(char *logLine);

struct OmniPvdDataTypeEnum
{
	enum Enum
	{
		eINT8,
		eINT16,
		eINT32,
		eINT64,
		eUINT8,
		eUINT16,
		eUINT32,
		eUINT64,
		eFLOAT32,
		eFLOAT64,
		eSTRING,
		eOBJECT_HANDLE,
		eENUM_VALUE,
		eFLAGS_WORD
	};
};

#endif
