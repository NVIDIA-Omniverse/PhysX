// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#include "internal/Profiling.h"

#include <carb/Framework.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <omni/physx/IPhysxSettings.h>

#include <cctype>
#include <cstdlib>

namespace ovphysx {
namespace profiling {
namespace {

const char* const kEnvBackend = "OVPHYSX_PROFILER";
const char* const kEnvOutputPath = "OVPHYSX_PROFILER_OUTPUT";

bool equalsIgnoreCase(const char* lhs, const char* rhs)
{
    if (!lhs || !rhs)
    {
        return false;
    }
    while (*lhs && *rhs)
    {
        const unsigned char a = static_cast<unsigned char>(*lhs++);
        const unsigned char b = static_cast<unsigned char>(*rhs++);
        if (a != b && std::tolower(a) != std::tolower(b))
        {
            return false;
        }
    }
    return *lhs == '\0' && *rhs == '\0';
}

bool isKnownBackend(int32_t value)
{
    return value == OVPHYSX_PROFILER_BACKEND_NONE || value == OVPHYSX_PROFILER_BACKEND_NVTX ||
           value == OVPHYSX_PROFILER_BACKEND_TRACE;
}

bool parseBackendName(const char* name, ovphysx_profiler_backend_t* out_backend)
{
    if (!name || !out_backend)
    {
        return false;
    }
    if (equalsIgnoreCase(name, "none") || equalsIgnoreCase(name, "off") || equalsIgnoreCase(name, "0"))
    {
        *out_backend = OVPHYSX_PROFILER_BACKEND_NONE;
        return true;
    }
    if (equalsIgnoreCase(name, "nvtx"))
    {
        *out_backend = OVPHYSX_PROFILER_BACKEND_NVTX;
        return true;
    }
    if (equalsIgnoreCase(name, "trace"))
    {
        *out_backend = OVPHYSX_PROFILER_BACKEND_TRACE;
        return true;
    }
    return false;
}

} // namespace

const char* backendName(ovphysx_profiler_backend_t value)
{
    switch (value)
    {
    case OVPHYSX_PROFILER_BACKEND_NVTX:
        return "nvtx";
    case OVPHYSX_PROFILER_BACKEND_TRACE:
        return "trace";
    case OVPHYSX_PROFILER_BACKEND_NONE:
    default:
        return "none";
    }
}

ovphysx_profiler_backend_t resolveBackend()
{
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;

    // The environment variable is the profiling entry point that needs no code change in
    // the host application, so it outranks the config entry -- which has already been
    // applied to the setting by the time this runs -- and is written through over it.
    // That ordering is the point: an application that hard-codes a backend can still be
    // profiled another way. Written through even for "none", so that OVPHYSX_PROFILER=none is
    // authoritative: the setting is process-global and sticky, and writing only the
    // enabling cases would make it impossible to turn profiling back off for a later
    // instance in a process where an earlier one switched it on.
    ovphysx_profiler_backend_t selected = OVPHYSX_PROFILER_BACKEND_NONE;
    const char* envBackend = std::getenv(kEnvBackend);
    if (envBackend && envBackend[0] != '\0')
    {
        ovphysx_profiler_backend_t parsed = OVPHYSX_PROFILER_BACKEND_NONE;
        if (parseBackendName(envBackend, &parsed))
        {
            if (settings)
            {
                settings->setInt(kSettingProfilerBackend, static_cast<int32_t>(parsed));
            }
            selected = parsed;
        }
        else
        {
            // Deliberately contributes nothing rather than forcing NONE: a typo should
            // not silently switch off profiling a config entry asked for.
            CARB_LOG_WARN("[ovphysx] %s=\"%s\" is not a known profiler backend; expected none, nvtx or "
                          "trace. Ignoring it; any existing selection stands.",
                          kEnvBackend, envBackend);
        }
    }

    const char* envOutputPath = std::getenv(kEnvOutputPath);
    if (envOutputPath && envOutputPath[0] != '\0' && settings)
    {
        settings->setString(kSettingProfilerOutputPath, envOutputPath);
    }

    if (settings)
    {
        const int32_t fromSettings = settings->getAsInt(kSettingProfilerBackend);
        if (isKnownBackend(fromSettings))
        {
            selected = static_cast<ovphysx_profiler_backend_t>(fromSettings);
        }
        else
        {
            CARB_LOG_WARN("[ovphysx] %s = %d is not a known profiler backend; profiling stays off",
                          kSettingProfilerBackend, fromSettings);
            selected = OVPHYSX_PROFILER_BACKEND_NONE;
            settings->setInt(kSettingProfilerBackend, static_cast<int32_t>(selected));
        }
    }

    if (selected != OVPHYSX_PROFILER_BACKEND_NONE)
    {
        CARB_LOG_INFO("[ovphysx] Profiler backend: %s", backendName(selected));
    }
    return selected;
}

} // namespace profiling
} // namespace ovphysx
