// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_SERIAL_UTILS_H
#define SN_SERIAL_UTILS_H

#define SN_BINARY_VERSION_GUID_NUM_CHARS 32

namespace physx
{

namespace Sn
{
	PxU32 getBinaryPlatformTag();
	bool isBinaryPlatformTagValid(PxU32 platformTag);
	const char* getBinaryPlatformName(PxU32 platformTag);
	const char* getBinaryVersionGuid();
	bool checkCompatibility(const char* binaryVersionGuidCandidate);

	PX_INLINE PxU32 getPadding(size_t value, PxU32 alignment)
	{
		const PxU32 mask = alignment - 1;
		const PxU32 overhead = PxU32(value) & mask;
		return (alignment - overhead) & mask;
	}

}

}

#endif
