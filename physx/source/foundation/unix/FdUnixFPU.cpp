// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxFPU.h"

#if !defined(__CYGWIN__)
#include <fenv.h>
PX_COMPILE_TIME_ASSERT(8 * sizeof(uint32_t) >= sizeof(fenv_t));
#endif

#if PX_OSX
// osx defines SIMD as standard for floating point operations.
#include <xmmintrin.h>
#endif

physx::PxFPUGuard::PxFPUGuard()
{
#if defined(__CYGWIN__)
#pragma message "FPUGuard::FPUGuard() is not implemented"
#elif PX_OSX
	mControlWords[0] = _mm_getcsr();
	// set default (disable exceptions: _MM_MASK_MASK) and FTZ (_MM_FLUSH_ZERO_ON), DAZ (_MM_DENORMALS_ZERO_ON: (1<<6))
	_mm_setcsr(_MM_MASK_MASK | _MM_FLUSH_ZERO_ON | (1 << 6));
#elif defined(__EMSCRIPTEN__)
// not supported
#else
	PX_COMPILE_TIME_ASSERT(sizeof(fenv_t) <= sizeof(mControlWords));

	fegetenv(reinterpret_cast<fenv_t*>(mControlWords));
	fesetenv(FE_DFL_ENV);

#if PX_LINUX
	// need to explicitly disable exceptions because fesetenv does not modify
	// the sse control word on 32bit linux (64bit is fine, but do it here just be sure)
	fedisableexcept(FE_ALL_EXCEPT);
#endif

#endif
}

physx::PxFPUGuard::~PxFPUGuard()
{
#if defined(__CYGWIN__)
#pragma message "PxFPUGuard::~PxFPUGuard() is not implemented"
#elif PX_OSX
	// restore control word and clear exception flags
	// (setting exception state flags cause exceptions on the first following fp operation)
	_mm_setcsr(mControlWords[0] & ~_MM_EXCEPT_MASK);
#elif defined(__EMSCRIPTEN__)
// not supported
#else
	fesetenv(reinterpret_cast<fenv_t*>(mControlWords));
#endif
}

PX_FOUNDATION_API void physx::PxEnableFPExceptions()
{
#if PX_LINUX && !defined(__EMSCRIPTEN__)
	feclearexcept(FE_ALL_EXCEPT);
	feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);
#elif PX_OSX
	// clear any pending exceptions
	// (setting exception state flags cause exceptions on the first following fp operation)
	uint32_t control = _mm_getcsr() & ~_MM_EXCEPT_MASK;

	// enable all fp exceptions except inexact and underflow (common, benign)
	// note: denorm has to be disabled as well because underflow can create denorms
	_mm_setcsr((control & ~_MM_MASK_MASK) | _MM_MASK_INEXACT | _MM_MASK_UNDERFLOW | _MM_MASK_DENORM);

#endif
}

PX_FOUNDATION_API void physx::PxDisableFPExceptions()
{
#if PX_LINUX && !defined(__EMSCRIPTEN__)
	fedisableexcept(FE_ALL_EXCEPT);
#elif PX_OSX
	// clear any pending exceptions
	// (setting exception state flags cause exceptions on the first following fp operation)
	uint32_t control = _mm_getcsr() & ~_MM_EXCEPT_MASK;
	_mm_setcsr(control | _MM_MASK_MASK);
#endif
}
