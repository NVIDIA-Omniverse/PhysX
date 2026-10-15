// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-1 AC-2 AC-3 AC-5
 */

#pragma once

// Which profiler backend the process asked for.
//
// The public C API zones use OVPHYSX_API_PROFILE_ZONE at the entry points and publish
// through carb::profiler so they reach whichever backend the process bound -- NVTX
// for an Nsight Systems capture, or the chrome-trace writer. The omni.physx runtime's
// own zones and the PhysX SDK zones reach the same backend by the same route, so one
// selection covers all three.
//
// Selecting and binding a backend is CarboniteLoader's job. This header only resolves
// which one was asked for.
//
// Emission is gated by carb::profiler itself, which drops every zone while no profiler
// is bound. One consequence worth knowing is that a host process which has already bound
// its own profiler collects these zones even with OVPHYSX_PROFILER_BACKEND_NONE
// selected, the same way the runtime's CARB_PROFILE_* zones behave.

#include "ovphysx/ovphysx_types.h"

#include <carb/profiler/Profile.h>

namespace ovphysx {
namespace profiling {

/// Scoped public-API zone with an explicit close for lifecycle calls that release
/// the profiler backend before returning. All other entry points rely on the
/// destructor, matching CARB_PROFILE_ZONE semantics.
class ApiProfileZone final
{
public:
    ApiProfileZone(uint64_t mask, carb::profiler::ZoneId zoneId) noexcept : m_mask(mask), m_zoneId(zoneId)
    {
    }

    ~ApiProfileZone()
    {
        end();
    }

    ApiProfileZone(const ApiProfileZone&) = delete;
    ApiProfileZone& operator=(const ApiProfileZone&) = delete;

    void end() noexcept
    {
        if (m_zoneId != carb::profiler::kNoZoneId)
        {
            CARB_PROFILE_END(m_mask, m_zoneId);
            m_zoneId = carb::profiler::kNoZoneId;
        }
    }

private:
    uint64_t m_mask;
    carb::profiler::ZoneId m_zoneId;
};

/// Carbonite settings carrying the selection. The typed config keys
/// OVPHYSX_CONFIG_PROFILER_BACKEND and OVPHYSX_CONFIG_PROFILER_OUTPUT_PATH write these,
/// as do the OVPHYSX_PROFILER and OVPHYSX_PROFILER_OUTPUT environment variables.
constexpr const char* kSettingProfilerBackend = "/physics/profilerBackend";
constexpr const char* kSettingProfilerOutputPath = "/physics/profilerOutputPath";

/// Reads the environment variables, writes them through to the settings above, and
/// returns the resolved backend.
///
/// Called once per instance creation, before the PhysX plugins load: the profiler is
/// bound during engine startup, and a selection made after that point would arrive too
/// late for the zones it was meant to record.
ovphysx_profiler_backend_t resolveBackend();

/// The name of a backend value, for logs and diagnostics. Returns "none" for a value
/// outside the enum.
const char* backendName(ovphysx_profiler_backend_t value);

} // namespace profiling
} // namespace ovphysx

// Mark the complete runtime of a public ovphysx C API boundary. Keep these
// zones distinct from nested implementation work so captures can separate
// client-facing API cost from the operations that implement it. The fixed local
// name allows the final destroy entry point to close its zone before releasing
// the profiler backend; every other entry point closes it by RAII.
#define OVPHYSX_API_PROFILE_ZONE(entryPoint)                                                       \
    ::ovphysx::profiling::ApiProfileZone ovphysxApiProfileZone(                                    \
        ::carb::profiler::kCaptureMaskDefault,                                                      \
        CARB_PROFILE_BEGIN(::carb::profiler::kCaptureMaskDefault, "ovphysx_" #entryPoint))

#define OVPHYSX_API_PROFILE_ZONE_END() ovphysxApiProfileZone.end()
