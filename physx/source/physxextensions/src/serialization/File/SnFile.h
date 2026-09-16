// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef SN_FILE_H
#define SN_FILE_H

// fopen_s - returns 0 on success, non-zero on failure

#if PX_WINDOWS_FAMILY

#include <stdio.h>

namespace physx
{
namespace sn
{
PX_INLINE PxI32 fopen_s(FILE** file, const char* name, const char* mode)
{
	static const PxU32 MAX_LEN = 300; 
	char buf[MAX_LEN+1];

	PxU32 i;
	for(i = 0; i<MAX_LEN && name[i]; i++)
		buf[i] = name[i] == '/' ? '\\' : name[i];
	buf[i] = 0;

	return i == MAX_LEN ? -1 : ::fopen_s(file, buf, mode);
};

} // namespace sn
} // namespace physx

#elif PX_UNIX_FAMILY || PX_SWITCH

#include <stdio.h>

namespace physx
{
namespace sn
{
PX_INLINE PxI32 fopen_s(FILE** file, const char* name, const char* mode)
{
	FILE* fp = ::fopen(name, mode);
	if(fp)
	{
		*file = fp;
		return PxI32(0);
	}
	return -1;
}
} // namespace sn
} // namespace physx
#else
#error "Platform not supported!"
#endif

#endif //SN_FILE_H

