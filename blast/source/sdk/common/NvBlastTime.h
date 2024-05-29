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


#ifndef NVBLASTTIME_H
#define NVBLASTTIME_H

#include "NvBlastTypes.h"


namespace Nv
{
namespace Blast
{

class Time
{
public:
    Time() : m_lastTickCount(getTimeTicks()) {}

    int64_t         getElapsedTicks()
    {
        const int64_t lastTickCount = m_lastTickCount;
        m_lastTickCount = getTimeTicks();
        return m_lastTickCount - lastTickCount;
    }

    int64_t         peekElapsedTicks() const
    {
        return getTimeTicks() - m_lastTickCount;
    }

    int64_t         getLastTickCount() const
    {
        return m_lastTickCount;
    }

    static double   seconds(int64_t ticks)
    {
        return s_secondsPerTick * ticks;
    }

private:
    int64_t         getTimeTicks() const;
    static double   getTickDuration();

    int64_t             m_lastTickCount;
    static const double s_secondsPerTick;
};

} // namespace Blast
} // namespace Nv


//////// Time inline functions for various platforms ////////

#if NV_MICROSOFT_FAMILY

#include "NvBlastIncludeWindows.h"

NV_INLINE int64_t Nv::Blast::Time::getTimeTicks() const
{
    LARGE_INTEGER a;
    QueryPerformanceCounter(&a);
    return a.QuadPart;
}

NV_INLINE double Nv::Blast::Time::getTickDuration()
{
    LARGE_INTEGER a;
    QueryPerformanceFrequency(&a);
    return 1.0 / (double)a.QuadPart;
}

#elif NV_UNIX_FAMILY

#include <time.h>

NV_INLINE int64_t Nv::Blast::Time::getTimeTicks() const
{
    struct timespec mCurrTimeInt;
    clock_gettime(CLOCK_REALTIME, &mCurrTimeInt);
    return (static_cast<int64_t>(mCurrTimeInt.tv_sec) * 1000000000) + (static_cast<int64_t>(mCurrTimeInt.tv_nsec));
}

NV_INLINE double Nv::Blast::Time::getTickDuration()
{
    return 1.e-9;
}

#endif

#endif // #ifndef NVBLASTTIME_H
