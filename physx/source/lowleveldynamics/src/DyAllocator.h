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

