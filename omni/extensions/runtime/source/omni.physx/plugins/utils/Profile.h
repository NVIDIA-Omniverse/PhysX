// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <private/omni/physx/IPhysxTests.h>

#include "Time.h"
#include <OmniPhysX.h>

#include <vector>

// Scoped profiling, for internal OmniPhysX profiling
class ScopedProfile
{
public:
    ScopedProfile(const char* profileZone) : mProfileZone(profileZone)
    {
        if (omni::physx::OmniPhysX::getInstance().isOmniPhysXProfilingEnabled())
        {
            mStartTime = Time::getCurrentCounterValue();
        }
    }

    ~ScopedProfile()
    {

        if (omni::physx::OmniPhysX::getInstance().isOmniPhysXProfilingEnabled())
        {
            const uint64_t endTime = Time::getCurrentCounterValue();
            const float timeInMs = Time::getCounterFrequency().toTensOfNanos(endTime - mStartTime) / (100.0f * 1000.0f);
            omni::physx::OmniPhysX::getInstance().getOmniPhysXProfileStats().push_back({ mProfileZone, timeInMs });
        }
    }

    uint64_t mStartTime;
    const char* mProfileZone;
};

#define PHYSICS_PROFILE(zoneName) ScopedProfile scopedProfileZone(zoneName)

#define PHYSICS_CROSS_THREAD_PROFILE_START(zoneName)                                                                   \
    if (omni::physx::OmniPhysX::getInstance().isOmniPhysXProfilingEnabled())                                           \
    {                                                                                                                  \
        omni::physx::OmniPhysX::getInstance().getOmniPhysXCrossThreadProfileMap()[zoneName] =                          \
            Time::getCurrentCounterValue();                                                                            \
    }

#define PHYSICS_CROSS_THREAD_PROFILE_END(zoneName)                                                                     \
    if (omni::physx::OmniPhysX::getInstance().isOmniPhysXProfilingEnabled())                                           \
    {                                                                                                                  \
        CrossThreadProfileMap::const_iterator fit =                                                                    \
            omni::physx::OmniPhysX::getInstance().getOmniPhysXCrossThreadProfileMap().find(zoneName);                  \
        if (fit != omni::physx::OmniPhysX::getInstance().getOmniPhysXCrossThreadProfileMap().end())                    \
        {                                                                                                              \
            const uint64_t endTime = Time::getCurrentCounterValue();                                                   \
            const float timeInMs =                                                                                     \
                Time::getCounterFrequency().toTensOfNanos(endTime - fit->second) / (100.0f * 1000.0f);                 \
            omni::physx::OmniPhysX::getInstance().getOmniPhysXProfileStats().push_back({ zoneName, timeInMs });        \
        }                                                                                                              \
    }

class ScopedPrintfProfile
{
public:
    ScopedPrintfProfile(const char* profileZone) : mProfileZone(profileZone)
    {
        if (omni::physx::OmniPhysX::getInstance().isOmniPhysXProfilingEnabled())
        {
            mStartTime = Time::getCurrentCounterValue();
        }
    }

    ~ScopedPrintfProfile()
    {

        if (omni::physx::OmniPhysX::getInstance().isOmniPhysXProfilingEnabled())
        {
            const uint64_t endTime = Time::getCurrentCounterValue();
            const float timeInMs = Time::getCounterFrequency().toTensOfNanos(endTime - mStartTime) / (100.0f * 1000.0f);
            printf("Profile zone %s: %f\n", mProfileZone, timeInMs);
        }
    }

    uint64_t mStartTime;
    const char* mProfileZone;
};

#define PHYSICS_PRINTF_PROFILE(zoneName) ScopedPrintfProfile scopedPrintfProfileZone(zoneName)
