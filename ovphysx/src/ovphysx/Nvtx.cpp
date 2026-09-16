// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-NVTX-001
 * @covers AC-1 AC-2 AC-3
 */

#include "internal/Nvtx.h"

#if OVPHYSX_NVTX_ENABLED

#include <carb/Framework.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>

#include <omni/physx/IPhysxSettings.h>

#include <atomic>
#include <cstdlib>
#include <cstring>

namespace ovphysx {
namespace nvtx {
namespace {

const char* const kEnvVar = "OVPHYSX_NVTX";

// Atomic because resolveEnabled() runs on every ovphysx_create_instance() while
// other threads may be inside instrumented entry points of an existing instance:
// instances coexist, so the write is not confined to process startup. Relaxed is
// sufficient: the value guards nothing but itself, and a zone that observes a
// stale value simply is or is not recorded.
std::atomic<bool> g_enabled{ false };

bool isTruthy(const char* value)
{
    if (!value || value[0] == '\0')
    {
        return false;
    }
    return strcmp(value, "0") != 0 && strcmp(value, "false") != 0 && strcmp(value, "False") != 0 &&
           strcmp(value, "FALSE") != 0;
}

// Created on first use rather than at load time, so a run without profiling makes
// no NVTX call at all. Zone registration can happen on any thread, hence the
// function-local static for the one-time initialization.
nvtxDomainHandle_t domain()
{
    static nvtxDomainHandle_t handle = nvtxDomainCreateA("ovphysx");
    return handle;
}

} // namespace

bool resolveEnabled()
{
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;

    // The environment variable is the profiling entry point that needs no code
    // change in the host application, so it wins over an absent config entry and
    // is written through to the setting: the omni.physx runtime reads the setting
    // to decide whether to emit the PhysX SDK zones.
    // Written through in both directions when the variable is present, so that
    // OVPHYSX_NVTX=0 is authoritative. The setting is process-global and sticky:
    // writing only the true case would make it impossible to turn profiling back
    // off for a later instance in a process where an earlier one enabled it.
    const char* envValue = std::getenv(kEnvVar);
    if (envValue && envValue[0] != '\0' && settings)
    {
        settings->setBool(omni::physx::kSettingNvtxEnabled, isTruthy(envValue));
    }

    bool enabled = false;
    if (settings)
    {
        enabled = settings->getAsBool(omni::physx::kSettingNvtxEnabled);
    }
    else
    {
        enabled = envValue && isTruthy(envValue);
    }
    g_enabled.store(enabled, std::memory_order_relaxed);

    if (enabled)
    {
        CARB_LOG_INFO("[ovphysx] NVTX profiling enabled (domains: ovphysx, PhysX)");
    }
    return enabled;
}

bool isEnabled()
{
    return g_enabled.load(std::memory_order_relaxed);
}

nvtxStringHandle_t registerZone(const char* name)
{
    return nvtxDomainRegisterStringA(domain(), name);
}

void pushZone(nvtxStringHandle_t zone)
{
    nvtxEventAttributes_t attributes;
    memset(&attributes, 0, sizeof(attributes));
    attributes.version = NVTX_VERSION;
    attributes.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    attributes.messageType = NVTX_MESSAGE_TYPE_REGISTERED;
    attributes.message.registered = zone;
    nvtxDomainRangePushEx(domain(), &attributes);
}

void popZone()
{
    nvtxDomainRangePop(domain());
}

} // namespace nvtx
} // namespace ovphysx

#endif
