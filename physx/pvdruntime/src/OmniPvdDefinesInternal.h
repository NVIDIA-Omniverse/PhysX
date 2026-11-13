// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_DEFINES_INTERNAL_H
#define OMNI_PVD_DEFINES_INTERNAL_H

#include "OmniPvdDefines.h"
#include <limits.h>


// Types used to store fields in the PVD data stream.
typedef uint8_t OmniPvdCommandStorageType;
typedef uint16_t OmniPvdDataTypeStorageType;
typedef uint16_t OmniPvdStringLengthStorageType;
typedef uint8_t OmniPvdAttributeHandleDepthStorageType;
typedef uint32_t OmniPvdAttributeDataLengthStorageType;
typedef uint32_t OmniPvdAttributeElementCountStorageType;
typedef uint64_t OmniPvdFrameTimestampStorageType;
typedef uint32_t OmniPvdMessageLineStorageType;
typedef uint32_t OmniPvdMessageTypeStorageType;

static_assert(CHAR_BIT == 8, "OmniPVD requires 8-bit bytes.");
static_assert(sizeof(OmniPvdCommandStorageType) == 1, "OmniPVD commands must be 1 byte.");
static_assert(sizeof(OmniPvdDataTypeStorageType) == 2, "OmniPVD data types must be 2 bytes.");
static_assert(sizeof(OmniPvdStringLengthStorageType) == 2, "OmniPVD string lengths must be 2 bytes.");
static_assert(sizeof(OmniPvdAttributeHandleDepthStorageType) == 1, "OmniPVD path depths must be 1 byte.");
static_assert(sizeof(OmniPvdAttributeDataLengthStorageType) == 4, "OmniPVD payload lengths must be 4 bytes.");
static_assert(sizeof(OmniPvdAttributeElementCountStorageType) == 4, "OmniPVD element counts must be 4 bytes.");
static_assert(sizeof(OmniPvdFrameTimestampStorageType) == 8, "OmniPVD frame timestamps must be 8 bytes.");
static_assert(sizeof(OmniPvdMessageLineStorageType) == 4, "OmniPVD message lines must be 4 bytes.");
static_assert(sizeof(OmniPvdMessageTypeStorageType) == 4, "OmniPVD message types must be 4 bytes.");
static_assert(sizeof(OmniPvdClassHandle) == 4, "OmniPVD class handles must be 4 bytes.");
static_assert(sizeof(OmniPvdAttributeHandle) == 4, "OmniPVD attribute handles must be 4 bytes.");
static_assert(sizeof(OmniPvdContextHandle) == 8, "OmniPVD context handles must be 8 bytes.");
static_assert(sizeof(OmniPvdObjectHandle) == 8, "OmniPVD object handles must be 8 bytes.");
static_assert(sizeof(OmniPvdVersionType) == 4, "OmniPVD version fields must be 4 bytes.");
static_assert(sizeof(OmniPvdEnumValueType) == 4, "OmniPVD enum values must be 4 bytes.");


#endif
