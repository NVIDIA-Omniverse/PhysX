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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.


#include "NvBlastAssert.h"

#include <stdio.h>
#include <stdlib.h>

#if NV_WINDOWS_FAMILY
#include <crtdbg.h>
#endif

extern "C"
{

void NvBlastAssertHandler(const char* expr, const char* file, int line, bool& ignore)
{
    NV_UNUSED(ignore); // is used only in debug windows config
    char buffer[1024];
#if NV_WINDOWS_FAMILY
    sprintf_s(buffer, 1024, "%s(%d) : Assertion failed: %s\n", file, line, expr);
#else
    sprintf(buffer, "%s(%d) : Assertion failed: %s\n", file, line, expr);
#endif
    puts(buffer);
#if NV_WINDOWS_FAMILY && NV_DEBUG
    // _CrtDbgReport returns -1 on error, 1 on 'retry', 0 otherwise including 'ignore'.
    // Hitting 'abort' will terminate the process immediately.
    int result = _CrtDbgReport(_CRT_ASSERT, file, line, NULL, "%s", buffer);
    int mode = _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_REPORT_MODE);
    ignore = _CRTDBG_MODE_WNDW == mode && result == 0;
    if (ignore)
        return;
    __debugbreak();
#elif (NV_WINDOWS_FAMILY && NV_CHECKED)
    __debugbreak();
#else
    abort();
#endif
}

} // extern "C"
