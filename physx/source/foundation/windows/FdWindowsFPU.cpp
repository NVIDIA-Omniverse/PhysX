// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxFPU.h"
#include "float.h"
#include "foundation/PxIntrinsics.h"

#if PX_X64 || PX_ARM || PX_A64
#define _MCW_ALL _MCW_DN | _MCW_EM | _MCW_RC
#else
#define _MCW_ALL _MCW_DN | _MCW_EM | _MCW_IC | _MCW_RC | _MCW_PC
#endif

physx::PxFPUGuard::PxFPUGuard()
{
// default plus FTZ and DAZ
#if PX_X64 || PX_ARM || PX_A64
	// query current control word state
	_controlfp_s(mControlWords, 0, 0);

	// set both x87 and sse units to default + DAZ
	unsigned int cw;
	_controlfp_s(&cw, _CW_DEFAULT | _DN_FLUSH, _MCW_ALL);
#else
	// query current control word state
	__control87_2(0, 0, mControlWords, mControlWords + 1);

	// set both x87 and sse units to default + DAZ
	unsigned int x87, sse;
	__control87_2(_CW_DEFAULT | _DN_FLUSH, _MCW_ALL, &x87, &sse);
#endif
}

physx::PxFPUGuard::~PxFPUGuard()
{
	_clearfp();

#if PX_X64 || PX_ARM || PX_A64
	// reset FP state
	unsigned int cw;
	_controlfp_s(&cw, *mControlWords, _MCW_ALL);
#else

	// reset FP state
	unsigned int x87, sse;
	__control87_2(mControlWords[0], _MCW_ALL, &x87, 0);
	__control87_2(mControlWords[1], _MCW_ALL, 0, &sse);
#endif
}

void physx::PxEnableFPExceptions()
{
	// clear any pending exceptions
	_clearfp();

	// enable all fp exceptions except inexact and underflow (common, benign)
	_controlfp_s(NULL, uint32_t(~_MCW_EM) | _EM_INEXACT | _EM_UNDERFLOW, _MCW_EM);
}

void physx::PxDisableFPExceptions()
{
	_controlfp_s(NULL, _MCW_EM, _MCW_EM);
}
