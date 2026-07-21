// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef BENCHMARK_UTILS_H
#define BENCHMARK_UTILS_H

#include "carb/Defines.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if CARB_PLATFORM_WINDOWS
#    pragma warning(push)
#    pragma warning(disable : 4996) // unsafe string functions
#    pragma warning(disable : 4668) //'symbol' is not defined as a preprocessor macro, replacing with '0' for
                                    //'directives'
#    include <windows.h>
#endif

static const size_t MAX_PRINTFORMATTED_LENGTH = 1024;

#if CARB_PLATFORM_WINDOWS
inline void printString(const char* str)
{
    puts(str); // do not use printf here, since str can contain multiple % signs that will not be printed
    OutputDebugStringA(str);
    OutputDebugStringA("\n");
}
#else
inline void printString(const char* str)
{
    puts(str);
}
#endif

inline void printFormatted(const char* format, ...)
{
    char buf[MAX_PRINTFORMATTED_LENGTH];

    va_list arg;
    va_start(arg, format);
    vsnprintf(buf, MAX_PRINTFORMATTED_LENGTH, format, arg);
    va_end(arg);

    printString(buf);
}

inline int32_t BmStricmp(const char* str, const char* str1)
{
#if CARB_PLATFORM_WINDOWS
    return (::_stricmp(str, str1));
#else
    return (::strcasecmp(str, str1));
#endif
}


template <class T>
inline T BmMax(T a, T b)
{
    return a < b ? b : a;
}

/**
\brief The return value is the lesser of the two specified values.
*/
template <class T>
inline T BmMin(T a, T b)
{
    return a < b ? a : b;
}

#if CARB_PLATFORM_WINDOWS
#    pragma warning(pop)
#endif


#endif
