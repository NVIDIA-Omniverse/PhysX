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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef OMNI_PVD_DEFINES_H
#define OMNI_PVD_DEFINES_H

#define OMNI_PVD_INVALID_HANDLE 0

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
typedef uint32_t OmniPvdVersionType;
typedef uint32_t OmniPvdEnumValueType;

typedef void (OMNI_PVD_CALL *OmniPvdLogFunction)(char *logLine);

struct OmniPvdDataType
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

template<uint32_t tType>
inline uint32_t getOmniPvdDataTypeSize() { return 0; }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eINT8>() { return sizeof(int8_t); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eINT16>() { return sizeof(int16_t); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eINT32>() { return sizeof(int32_t); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eINT64>() { return sizeof(int64_t); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eUINT8>() { return sizeof(uint8_t); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eUINT16>() { return sizeof(uint16_t); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eUINT32>() { return sizeof(uint32_t); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eUINT64>() { return sizeof(uint64_t); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eFLOAT32>() { return sizeof(float); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eFLOAT64>() { return sizeof(double); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eSTRING>() { return 0; }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eOBJECT_HANDLE>() { return sizeof(OmniPvdObjectHandle); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eENUM_VALUE>() { return sizeof(OmniPvdEnumValueType); }

template<>
inline uint32_t getOmniPvdDataTypeSize<OmniPvdDataType::eFLAGS_WORD>() { return sizeof(OmniPvdClassHandle); }

inline uint32_t getOmniPvdDataTypeSizeFromEnum(OmniPvdDataType::Enum dataType)
{
	switch (dataType)
	{
	case OmniPvdDataType::eINT8:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eINT8>();
		break;
	case OmniPvdDataType::eINT16:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eINT16>();
		break;
	case OmniPvdDataType::eINT32:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eINT32>();
		break;
	case OmniPvdDataType::eINT64:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eINT64>();
		break;
	case OmniPvdDataType::eUINT8:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eUINT8>();
		break;
	case OmniPvdDataType::eUINT16:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eUINT16>();
		break;
	case OmniPvdDataType::eUINT32:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eUINT32>();
		break;
	case OmniPvdDataType::eUINT64:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eUINT64>();
		break;
	case OmniPvdDataType::eFLOAT32:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eFLOAT32>();
		break;
	case OmniPvdDataType::eFLOAT64:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eFLOAT64>();
		break;
	case OmniPvdDataType::eSTRING:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eSTRING>();
		break;
	case OmniPvdDataType::eOBJECT_HANDLE:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eOBJECT_HANDLE>();
		break;
	case OmniPvdDataType::eENUM_VALUE:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eENUM_VALUE>();
		break;
	case OmniPvdDataType::eFLAGS_WORD:
		return getOmniPvdDataTypeSize<OmniPvdDataType::eFLAGS_WORD>();
		break;
	default:
		return 0;
		break;
	}
}

#endif
