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

//! @file
//!
//! @brief Profiler utility API in the NvBlastGlobals library

#ifndef NVBLASTPROFILER_H
#define NVBLASTPROFILER_H

#include "NvBlastPreprocessor.h"


namespace Nv
{
namespace Blast
{


/**
Custom Blast profiler interface.
*/
class ProfilerCallback
{
protected:
    virtual ~ProfilerCallback() {}

public:
    /**
    Called when a nested profile zone starts.
    */
    virtual void zoneStart(const char* name) = 0;

    /**
    Called when the current profile zone ends.
    */
    virtual void zoneEnd() = 0;
};


/**
Profiler detail to be reported. The higher setting is used, the more details are reported.
*/
struct ProfilerDetail
{
    enum Level
    {
        LOW,
        MEDIUM,
        HIGH
    };
};


} // namespace Blast
} // namespace Nv


/**
Profiler features are only active in checked, debug and profile builds.
*/

/**
Set a custom profiler callback. May be nullptr (the default).
*/
NVBLAST_API void NvBlastProfilerSetCallback(Nv::Blast::ProfilerCallback* pcb);


/**
Sets the depth of reported profile zones.
Higher levels (more nesting) of instrumentation can have a significant impact.
Defaults to Nv::Blast::ProfilerDetail::Level::LOW.
*/
NVBLAST_API void NvBlastProfilerSetDetail(Nv::Blast::ProfilerDetail::Level);


#endif
