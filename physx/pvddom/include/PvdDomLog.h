// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef PVDDOM_LOG_H
#define PVDDOM_LOG_H

enum PvdDomLogLevel
{
    ePvdDomLogInfo,
    ePvdDomLogWarn,
    ePvdDomLogError
};

typedef void (*PvdDomLogFunction)(PvdDomLogLevel level, const char* message);

void pvdDomSetLogFunction(PvdDomLogFunction fn);
void pvdDomLog(PvdDomLogLevel level, const char* fmt, ...);

#define PVDDOM_LOG_INFO(...)  pvdDomLog(ePvdDomLogInfo, __VA_ARGS__)
#define PVDDOM_LOG_WARN(...)  pvdDomLog(ePvdDomLogWarn, __VA_ARGS__)
#define PVDDOM_LOG_ERROR(...) pvdDomLog(ePvdDomLogError, __VA_ARGS__)

#endif
