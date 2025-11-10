// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdTimer.h"
#include <stdlib.h>

OmniPvdTimer::OmniPvdTimer(
)
{
#ifdef OMNI_PVD_WIN
    QueryPerformanceFrequency(&frequency);
    startCount.QuadPart = 0;
#else
    startCount.tv_sec = startCount.tv_usec = 0;
#endif    
}

OmniPvdTimer::~OmniPvdTimer()
{
}

void OmniPvdTimer::start()
{
#ifdef OMNI_PVD_WIN
    QueryPerformanceCounter(&startCount);
#else
    gettimeofday(&startCount, NULL);
#endif
}

double OmniPvdTimer::getElapsedTimeInMicroSec()
{
    double startTimeInMicroSec;
    double local_endTimeInMicroSec;
#ifdef OMNI_PVD_WIN
    LARGE_INTEGER local_endCount;
    QueryPerformanceCounter(&local_endCount);
    startTimeInMicroSec = ((double)startCount.QuadPart) * ((double)1000000.0 / ((double)frequency.QuadPart));
    local_endTimeInMicroSec = ((double)local_endCount.QuadPart) * ((double)1000000.0 / ((double)frequency.QuadPart));
#else
    timeval local_endCount;
    gettimeofday(&local_endCount, NULL);
    startTimeInMicroSec = (startCount.tv_sec * 1000000.0) + startCount.tv_usec;
    local_endTimeInMicroSec = (local_endCount.tv_sec * 1000000.0) + local_endCount.tv_usec;
#endif
    return local_endTimeInMicroSec - startTimeInMicroSec;
}

double OmniPvdTimer::getElapsedTime()
{
    return this->getElapsedTimeInMicroSec() * 0.000001;
}
