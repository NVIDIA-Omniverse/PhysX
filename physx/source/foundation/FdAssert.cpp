// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxAssert.h"
#include "foundation/PxString.h"
#include <stdio.h>
#include <stdlib.h>

#if PX_WINDOWS_FAMILY
#include <crtdbg.h>
#elif PX_SWITCH
#include "foundation/switch/PxSwitchAbort.h"
#endif

void physx::PxAssert(const char* expr, const char* file, int line, bool& ignore)
{
	PX_UNUSED(ignore); // is used only in debug windows config
	char buffer[1024];
	Pxsnprintf(buffer, sizeof(buffer), "%s(%d) : Assertion failed: %s\n", file, line, expr);
	physx::PxPrintString(buffer);
#if PX_WINDOWS_FAMILY&& PX_DEBUG && PX_DEBUG_CRT
	// _CrtDbgReport returns -1 on error, 1 on 'retry', 0 otherwise including 'ignore'.
	// Hitting 'abort' will terminate the process immediately.
	int result = _CrtDbgReport(_CRT_ASSERT, file, line, NULL, "%s", buffer);
	int mode = _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_REPORT_MODE);
	ignore = _CRTDBG_MODE_WNDW == mode && result == 0;
	if(ignore)
		return;
	__debugbreak();
#elif PX_WINDOWS_FAMILY&& PX_CHECKED
	__debugbreak();
#elif PX_SWITCH
	abort(buffer);
#else
	abort();
#endif
}
