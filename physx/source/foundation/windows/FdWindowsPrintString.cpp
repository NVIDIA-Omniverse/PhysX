// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxString.h"
#include <stdio.h>
#include "foundation/windows/PxWindowsInclude.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

void physx::PxPrintString(const char* str)
{
	puts(str); // do not use printf here, since str can contain multiple % signs that will not be printed
	OutputDebugStringA(str);
	OutputDebugStringA("\n");
}
