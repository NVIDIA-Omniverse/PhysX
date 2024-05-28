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

#ifndef PX_ASSERT_H
#define PX_ASSERT_H

#include "foundation/PxFoundationConfig.h"
#include "foundation/Px.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

/**
 * \brief  Built-in assert function
 */
PX_FOUNDATION_API void PxAssert(const char* exp, const char* file, int line, bool& ignore);

#if !PX_ENABLE_ASSERTS
	#define PX_ASSERT(exp) ((void)0)
	#define PX_ALWAYS_ASSERT_MESSAGE(exp) ((void)0)
	#define PX_ASSERT_WITH_MESSAGE(condition, message) ((void)0)
#else
#if PX_VC
	#define PX_CODE_ANALYSIS_ASSUME(exp)	\
		__analysis_assume(!!(exp)) // This macro will be used to get rid of analysis warning messages if a PX_ASSERT is used
	// to "guard" illegal mem access, for example.
#else
	#define PX_CODE_ANALYSIS_ASSUME(exp)
#endif
	#define PX_ASSERT(exp)																			\
		{																							\
			static bool _ignore = false;															\
			((void)((!!(exp)) || (!_ignore && (physx::PxAssert(#exp, PX_FL, _ignore), false))));	\
			PX_CODE_ANALYSIS_ASSUME(exp);															\
		}
	#define PX_ALWAYS_ASSERT_MESSAGE(exp)															\
		{																							\
			static bool _ignore = false;															\
			if(!_ignore)																			\
				physx::PxAssert(exp, PX_FL, _ignore);												\
		}
	#define PX_ASSERT_WITH_MESSAGE(exp, message)													\
		{																							\
			static bool _ignore = false;															\
			((void)((!!(exp)) || (!_ignore && (physx::PxAssert(message, PX_FL, _ignore), false))));	\
			PX_CODE_ANALYSIS_ASSUME(exp);															\
		}
#endif // !PX_ENABLE_ASSERTS

#define PX_ALWAYS_ASSERT() PX_ASSERT(0)

#if !PX_DOXYGEN
} // namespace physx
#endif


#endif

