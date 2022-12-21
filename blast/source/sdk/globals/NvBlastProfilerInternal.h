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


#ifndef NVBLASTPROFILERINTERNAL_H
#define NVBLASTPROFILERINTERNAL_H

#include "NvBlastPreprocessor.h"
#include "NvBlastProfiler.h"

#if NV_PROFILE || NV_CHECKED || NV_DEBUG

NVBLAST_API void NvBlastProfilerBegin(const char* name, Nv::Blast::ProfilerDetail::Level);
NVBLAST_API void NvBlastProfilerEnd(const void* name, Nv::Blast::ProfilerDetail::Level);

Nv::Blast::ProfilerCallback* NvBlastProfilerGetCallback();
Nv::Blast::ProfilerDetail::Level NvBlastProfilerGetDetail();


namespace Nv
{
namespace Blast
{

    
class ProfileScope
{
public:
    ProfileScope(const char* name, ProfilerDetail::Level level) :m_name(name), m_level(level)
    {
        NvBlastProfilerBegin(m_name, m_level);
    }

    ~ProfileScope()
    {
        NvBlastProfilerEnd(m_name, m_level);
    }

private:
    const char* m_name;
    ProfilerDetail::Level m_level;
};


} // namespace Blast
} // namespace Nv


#define BLAST_PROFILE_PREFIX                "Blast: "
#define BLAST_PROFILE_ZONE_BEGIN(name)      NvBlastProfilerBegin(BLAST_PROFILE_PREFIX name, Nv::Blast::ProfilerDetail::HIGH)
#define BLAST_PROFILE_ZONE_END(name)        NvBlastProfilerEnd(BLAST_PROFILE_PREFIX name, Nv::Blast::ProfilerDetail::HIGH)
#define BLAST_PROFILE_SCOPE(name, detail)   Nv::Blast::ProfileScope NV_CONCAT(_scope,__LINE__) (BLAST_PROFILE_PREFIX name, detail)
#define BLAST_PROFILE_SCOPE_L(name)         BLAST_PROFILE_SCOPE(name, Nv::Blast::ProfilerDetail::LOW)
#define BLAST_PROFILE_SCOPE_M(name)         BLAST_PROFILE_SCOPE(name, Nv::Blast::ProfilerDetail::MEDIUM)
#define BLAST_PROFILE_SCOPE_H(name)         BLAST_PROFILE_SCOPE(name, Nv::Blast::ProfilerDetail::HIGH)

#else

#define BLAST_PROFILE_ZONE_BEGIN(name)  
#define BLAST_PROFILE_ZONE_END(name)
#define BLAST_PROFILE_SCOPE_L(name)
#define BLAST_PROFILE_SCOPE_M(name)
#define BLAST_PROFILE_SCOPE_H(name)

#endif

#endif
