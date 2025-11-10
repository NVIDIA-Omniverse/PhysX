// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdDefines.h"

class OmniPvdTimer
{
public:
    OmniPvdTimer();
    ~OmniPvdTimer();
    void start();
    double getElapsedTime();
    double getElapsedTimeInMicroSec();

private:
#ifdef OMNI_PVD_WIN
    LARGE_INTEGER frequency;
    LARGE_INTEGER startCount;
#else
    timeval startCount;
#endif
};
