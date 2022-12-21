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
// Copyright (c) 2016-2022 NVIDIA Corporation. All rights reserved.


#include "stdint.h"
#include "NvBlastProfilerInternal.h"

#if NV_PROFILE || NV_CHECKED || NV_DEBUG

namespace Nv
{
namespace Blast
{

class EmptyProfilerCallback : public ProfilerCallback
{
    void zoneStart(const char*) {}
    void zoneEnd() {}
};
EmptyProfilerCallback g_EmptyCallback;

ProfilerCallback* g_ProfilerCallback = &g_EmptyCallback;
ProfilerDetail::Level g_ProfilerDetail = ProfilerDetail::LOW;

} // namespace Blast
} // namespace Nv 


void NvBlastProfilerSetCallback(Nv::Blast::ProfilerCallback* pcb)
{
    Nv::Blast::g_ProfilerCallback = pcb != nullptr ? pcb : &Nv::Blast::g_EmptyCallback;
}

Nv::Blast::ProfilerCallback* NvBlastProfilerGetCallback()
{
    return Nv::Blast::g_ProfilerCallback;
}


void NvBlastProfilerSetDetail(Nv::Blast::ProfilerDetail::Level level)
{
    Nv::Blast::g_ProfilerDetail = level;
}

Nv::Blast::ProfilerDetail::Level NvBlastProfilerGetDetail()
{
    return Nv::Blast::g_ProfilerDetail;
}


void NvBlastProfilerBegin(const char* name, Nv::Blast::ProfilerDetail::Level level)
{
    if (level <= NvBlastProfilerGetDetail())
    {
        NvBlastProfilerGetCallback()->zoneStart(name);
    }
}

void NvBlastProfilerEnd(const void* /*name*/, Nv::Blast::ProfilerDetail::Level level)
{
    if (level <= NvBlastProfilerGetDetail())
    {
        NvBlastProfilerGetCallback()->zoneEnd();
    }
}

#else

void NvBlastProfilerSetCallback(Nv::Blast::ProfilerCallback*) {}
void NvBlastProfilerSetDetail(Nv::Blast::ProfilerDetail::Level) {}

#endif // NV_PROFILE
