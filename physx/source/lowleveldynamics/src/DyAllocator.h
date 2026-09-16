// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

   
#ifndef DY_ALLOCATOR_H
#define DY_ALLOCATOR_H

#include "foundation/PxFoundation.h"

namespace physx
{
namespace Dy
{
	extern const char* gConstraintDataErrorMsg_Null_Joints;
	extern const char* gConstraintDataErrorMsg_TooLarge_Joints;
	extern const char* gConstraintDataErrorMsg_Null_Contacts;
	extern const char* gConstraintDataErrorMsg_TooLarge_Contacts;
	extern const char* gFrictionDataErrorMsg_TooLarge;

	template<const bool jointsErrorMsg>
	PX_FORCE_INLINE static bool checkConstraintDataPtr(void* ptr)
	{
		if(NULL==ptr)
		{
			PX_WARN_ONCE(jointsErrorMsg ? gConstraintDataErrorMsg_Null_Joints : gConstraintDataErrorMsg_Null_Contacts);
			return false;
		}
		// PT: I leave this codepath here for now but I think this is not needed anymore.
		// The reserveConstraintData() calls do not return -1 anymore.
		else if(ptr==reinterpret_cast<void*>(-1))
		{
			PX_WARN_ONCE(jointsErrorMsg ? gConstraintDataErrorMsg_TooLarge_Joints : gConstraintDataErrorMsg_TooLarge_Contacts);
			return false;
		}
		return true;
	}

	PX_FORCE_INLINE static bool checkFrictionDataPtr(void* ptr)
	{
		if(NULL==ptr)
		{
			// PT: for friction the error msg here is similar to gConstraintDataErrorMsg_Null_Contacts
			PX_WARN_ONCE(gConstraintDataErrorMsg_Null_Contacts);
			return false;
		}
		// PT: for friction this is still needed....
		else if(ptr==reinterpret_cast<void*>(-1))
		{
			PX_WARN_ONCE(gFrictionDataErrorMsg_TooLarge);
			return false;
		}
		return true;
	}
}
}

#endif

