// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OmniPvdLog.h"

#include <stdarg.h>
#include <stdio.h>

OmniPvdLog::OmniPvdLog()
{
	mLogFunction = 0;
}

OmniPvdLog::~OmniPvdLog()
{
}

void OmniPvdLog::setLogFunction(OmniPvdLogFunction logFunction)
{
	mLogFunction = logFunction;
}

void OmniPvdLog::outputLine(const char* fmt, ...)
{
	if (!mLogFunction) return;
	char logLineBuff[2048];
	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(logLineBuff, sizeof(logLineBuff), fmt, args);
	va_end(args);
	if (n < 0)
		logLineBuff[0] = '\0';
	mLogFunction(logLineBuff);
}
