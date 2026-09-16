// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-NVTX-001
 * @covers AC-1 AC-5
 */

#pragma once

// NVTX ranges for the public C API entry points, visible in Nsight Systems under
// the "ovphysx" domain. The PhysX SDK's own zones are emitted separately by the
// omni.physx runtime under the "PhysX" domain.
//
// OVPHYSX_NVTX_ENABLED is defined by the build when the NVTX 3 headers are
// available. Instrumentation is compiled in for release builds and shipped
// wheels. Whether ranges are emitted is a runtime decision, see resolveEnabled().

#if OVPHYSX_NVTX_ENABLED
// Two things the NVTX headers need from their includer, so that this header stays
// safe to include first in a translation unit:
//   - <Windows.h> configured the way PlatformIncludes.hpp does it. The NVTX
//     loader includes Windows.h itself, which drags in the min/max macros and
//     breaks any std::numeric_limits<T>::max() further down the file.
//   - <stdlib.h>. The Windows loader calls _wgetenv (nvtxDetail/nvtxInit.h) but
//     only includes stdlib.h on the non-Windows paths.
#include "internal/sdk/PlatformIncludes.hpp"
#include <stdlib.h>
#include <nvtx3/nvToolsExt.h>
#include <atomic>
#endif

namespace ovphysx {
namespace nvtx {

#if OVPHYSX_NVTX_ENABLED

/// Resolves the process-wide gate from the OVPHYSX_NVTX environment variable and
/// the /physics/nvtxEnabled setting, and returns the resolved value. Called once
/// during instance creation, before any instrumented entry point can run.
bool resolveEnabled();

/// True when NVTX ranges should be emitted. resolveEnabled() runs on every
/// instance creation and instances coexist, so this can be written while other
/// threads are inside instrumented entry points. The flag is atomic for that
/// reason. Ordering is relaxed: it guards nothing but itself.
bool isEnabled();

/// Registers a zone name with the ovphysx domain. Only call while isEnabled().
nvtxStringHandle_t registerZone(const char* name);

void pushZone(nvtxStringHandle_t zone);
void popZone();

/// A zone name registered with the domain on first use while profiling is on.
/// Declared static at the call site, so the registration cost is paid once per
/// site. The handle is atomic because a site can be reached from any thread.
///
/// Registration is deliberately NOT done in the constructor: a process may create
/// an instance with profiling off and a later one with it on, and a handle decided
/// at the site's first execution would leave that site dead for the whole process.
class RegisteredZone
{
public:
    explicit RegisteredZone(const char* name) : mName(name), mHandle(nullptr)
    {
    }

    /// Only call while isEnabled(): registering needs the domain.
    nvtxStringHandle_t handle()
    {
        nvtxStringHandle_t cached = mHandle.load(std::memory_order_acquire);
        if (!cached)
        {
            cached = registerZone(mName);
            mHandle.store(cached, std::memory_order_release);
        }
        return cached;
    }

private:
    const char* mName;
    std::atomic<nvtxStringHandle_t> mHandle;
};

class ScopedZone
{
public:
    explicit ScopedZone(RegisteredZone& zone) : mPushed(false)
    {
        // The gate is read per call rather than captured once per site, so that
        // enabling profiling for a later instance in the process takes effect.
        // Read once, though: the outcome is latched in mPushed and the pop follows
        // it, because resolveEnabled() may run on another thread while this scope
        // is open and a pop without its push would close the enclosing range.
        if (!isEnabled())
        {
            return;
        }
        const nvtxStringHandle_t handle = zone.handle();
        if (handle)
        {
            pushZone(handle);
            mPushed = true;
        }
    }

    ~ScopedZone()
    {
        if (mPushed)
        {
            popZone();
        }
    }

    ScopedZone(const ScopedZone&) = delete;
    ScopedZone& operator=(const ScopedZone&) = delete;

private:
    bool mPushed;
};

#define OVPHYSX_NVTX_CONCAT_INNER(a, b) a##b
#define OVPHYSX_NVTX_CONCAT(a, b) OVPHYSX_NVTX_CONCAT_INNER(a, b)

/// Opens an NVTX range named `zoneName` for the remainder of the enclosing scope.
#define OVPHYSX_NVTX_ZONE(zoneName)                                                                                    \
    static ::ovphysx::nvtx::RegisteredZone OVPHYSX_NVTX_CONCAT(_ovphysxNvtxName, __LINE__)(zoneName);                   \
    const ::ovphysx::nvtx::ScopedZone OVPHYSX_NVTX_CONCAT(_ovphysxNvtxZone, __LINE__)(                                  \
        OVPHYSX_NVTX_CONCAT(_ovphysxNvtxName, __LINE__))

#else

inline bool resolveEnabled()
{
    return false;
}

inline bool isEnabled()
{
    return false;
}

#define OVPHYSX_NVTX_ZONE(zoneName)

#endif

} // namespace nvtx
} // namespace ovphysx
