// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "PvdDomLog.h"
#include <cstdarg>
#include <cstdio>

static PvdDomLogFunction gLogFunction = nullptr;

void pvdDomSetLogFunction(PvdDomLogFunction fn) { gLogFunction = fn; }

void pvdDomLog(PvdDomLogLevel level, const char* fmt, ...)
{
    if (!gLogFunction) return;
    char buf[2048];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    gLogFunction(level, buf);
}
