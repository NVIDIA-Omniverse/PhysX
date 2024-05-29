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


#include "stdint.h"
#include "NvProfiler.h"
#include "NvBlastGlobals.h"
#include "NvBlastInternalProfiler.h"
#include "NvBlastAssert.h"

namespace Nv
{
namespace Blast
{

#define SUPPORTS_THREAD_LOCAL (!NV_VC || NV_VC > 12)

struct InternalProfilerData
{
    const char* name;
    void* data;
};

#if SUPPORTS_THREAD_LOCAL
static const int32_t PROFILER_MAX_NESTED_DEPTH = 64;
static thread_local InternalProfilerData th_ProfileData[PROFILER_MAX_NESTED_DEPTH];
static thread_local int32_t th_depth = 0;
#endif

extern nvidia::NvProfilerCallback *g_profilerCallback;

/**
Wraps the nvidia::NvProfilerCallback set in NvBlastGlobalSetProfilerCallback.
*/
class InternalProfiler
{
public:
    /**
    Construct a InternalProfiler with platform specific profiler signals disabled.
    */
    InternalProfiler() : m_platformEnabled(false) {}

    void zoneStart(const char* name)
    {

#if SUPPORTS_THREAD_LOCAL
        if (g_profilerCallback)
        {
            void* data = g_profilerCallback->zoneStart(name, false, 0xb1a57);

            if (th_depth < PROFILER_MAX_NESTED_DEPTH && th_depth >= 0)
            {
                th_ProfileData[th_depth].name = name;
                th_ProfileData[th_depth].data = data;
                th_depth++;
            }
            else
            {
                NVBLAST_ASSERT(th_depth < PROFILER_MAX_NESTED_DEPTH && th_depth >= 0);
            }
        }
#endif

        if (m_platformEnabled)
        {
            platformZoneStart(name);
        }
    }

    void zoneEnd()
    {

#if SUPPORTS_THREAD_LOCAL
        if (g_profilerCallback)
        {
            th_depth--;

            if (th_depth >= 0)
            {
                InternalProfilerData& pd = th_ProfileData[th_depth];
                g_profilerCallback->zoneEnd(pd.data, pd.name, false, 0xb1a57);
            }
            else
            {
                NVBLAST_ASSERT(th_depth >= 0);
            }
        }
#endif

        if (m_platformEnabled)
        {
            platformZoneEnd();
        }
    }


    ////// local interface //////

    /**
    Enable or disable platform specific profiler signals. Disabled by default.

    \param[in]  enabled     true enables, false disables platform profiler calls.
    */
    void setPlatformEnabled(bool enabled)
    {
        m_platformEnabled = enabled;
    }

private:
    bool m_platformEnabled;
};

static InternalProfiler g_InternalProfiler;
static InternalProfilerDetail::Level g_ProfilerDetail = InternalProfilerDetail::LOW;

void NvBlastInternalProfilerSetPlatformEnabled(bool platformEnabled)
{
    return g_InternalProfiler.setPlatformEnabled(platformEnabled);
}

void NvBlastInternalProfilerSetDetail(InternalProfilerDetail::Level level)
{
    g_ProfilerDetail = level;
}

InternalProfilerDetail::Level NvBlastProfilerGetDetail()
{
    return g_ProfilerDetail;
}

void NvBlastProfilerBegin(const char* name, InternalProfilerDetail::Level level)
{
    if (level <= NvBlastProfilerGetDetail())
    {
        g_InternalProfiler.zoneStart(name);
    }
}

void NvBlastProfilerEnd(const void* /*name*/, InternalProfilerDetail::Level level)
{
    if (level <= NvBlastProfilerGetDetail())
    {
        g_InternalProfiler.zoneEnd();
    }
}

} // namespace Blast
} // namespace Nv 
