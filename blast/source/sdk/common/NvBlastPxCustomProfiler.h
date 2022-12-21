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


#ifndef NVBLASTDEFAULTPROFILER_H
#define NVBLASTDEFAULTPROFILER_H

#include "NvBlastProfiler.h"
#include "PxFoundation.h"
#include "foundation/PxProfiler.h"

#if NV_NVTX  
#include "nvToolsExt.h"
NV_INLINE void platformZoneStart(const char* name) { nvtxRangePushA(name); }
NV_INLINE void platformZoneEnd() { nvtxRangePop(); }

#else
NV_INLINE void platformZoneStart(const char*) { }
NV_INLINE void platformZoneEnd() { }

#endif

#define SUPPORTS_THREAD_LOCAL (!NV_VC || NV_VC > 12)

namespace Nv
{
namespace Blast
{

struct ExtProfileData
{
    const char* name;
    void* data;
};

#if SUPPORTS_THREAD_LOCAL
static const int32_t PROFILER_MAX_NESTED_DEPTH = 64;
static thread_local ExtProfileData th_ProfileData[PROFILER_MAX_NESTED_DEPTH];
static thread_local int32_t th_depth = 0;
#endif


/**
Implements Nv::Blast::ProfilerCallback to serve the physx::PxProfilerCallback set in PxFoundation
for PhysX Visual Debugger support and platform specific profilers like NVIDIA(R) NSight(TM).
*/
class ExtCustomProfiler : public ProfilerCallback
{
public:
    /**
    Construct an ExtCustomProfiler with platform specific profiler signals disabled.
    */
    ExtCustomProfiler() : m_platformEnabled(false) {}


    ////// ProfilerCallback interface //////

    virtual void zoneStart(const char* name) override
    {

#if SUPPORTS_THREAD_LOCAL
        if (PxGetProfilerCallback())
        {
            void* data = PxGetProfilerCallback()->zoneStart(name, false, 0xb1a57);

            if (th_depth < PROFILER_MAX_NESTED_DEPTH && th_depth >= 0)
            {
                th_ProfileData[th_depth].name = name;
                th_ProfileData[th_depth].data = data;
                th_depth++;
            }
            else
            {
                assert(th_depth < PROFILER_MAX_NESTED_DEPTH && th_depth >= 0);
            }
        }
#endif

        if (m_platformEnabled)
        {
            platformZoneStart(name);
        }
    }

    virtual void zoneEnd() override
    {

#if SUPPORTS_THREAD_LOCAL
        if (PxGetProfilerCallback())
        {
            th_depth--;

            if (th_depth >= 0)
            {
                ExtProfileData& pd = th_ProfileData[th_depth];
                PxGetProfilerCallback()->zoneEnd(pd.data, pd.name, false, 0xb1a57);
            }
            else
            {
                assert(th_depth >= 0);
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

} // namespace Blast
} // namespace Nv


#endif // NVBLASTDEFAULTPROFILER_H
