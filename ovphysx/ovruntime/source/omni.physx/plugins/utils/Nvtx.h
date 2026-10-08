// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SIM-NVTX-001
 * @covers AC-2 AC-4
 */

#pragma once

// NVTX ranges for the PhysX SDK profile zones, visible in Nsight Systems under the
// "PhysX" domain. The zones themselves are emitted by the SDK's PX_PROFILE_ZONE
// macros and reach us through PxProfilerCallback, so this only translates them.
//
// The ovphysx C API instruments its own entry points separately, under the
// "ovphysx" domain.
//
// OMNI_PHYSX_NVTX_ENABLED is defined by the build when the NVTX 3 headers are
// available. Emission is a runtime decision driven by /physics/nvtxEnabled.

#include <cstddef>
#include <cstdint>

#if OMNI_PHYSX_NVTX_ENABLED
#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#endif

#if OMNI_PHYSX_NVTX_ENABLED
// The NVTX implementation headers pull in <Windows.h> for their loader, which
// brings the min/max macros along unless NOMINMAX is already set. The ovruntime
// targets set it project-wide; assert that rather than silently depending on it,
// since a leak shows up as errors in unrelated headers further down.
#if defined(_WIN32) && !defined(NOMINMAX)
#error "utils/Nvtx.h requires NOMINMAX: NVTX includes <Windows.h> and the min/max macros break std::numeric_limits"
#endif
// The NVTX Windows loader calls _wgetenv (nvtxDetail/nvtxInit.h) but only includes
// stdlib.h on its non-Windows paths.
#include <stdlib.h>
#include <nvtx3/nvToolsExt.h>
#include <cstring>
#endif

namespace omni
{
namespace physx
{
namespace nvtx
{

#if OMNI_PHYSX_NVTX_ENABLED

/// Process-wide gate, written whenever the PhysX SDK is created and read from
/// every thread that runs PhysX work. The SDK can be created while an existing
/// scene is being stepped, so the write is not confined to a quiescent moment and
/// the flag is atomic. Ordering is relaxed: it guards nothing but itself, and a
/// zone that observes a stale value simply is or is not recorded.
inline std::atomic<bool>& enabledFlag()
{
    static std::atomic<bool> enabled{ false };
    return enabled;
}

inline void setEnabled(bool enabled)
{
    enabledFlag().store(enabled, std::memory_order_relaxed);
}

inline bool isEnabled()
{
    return enabledFlag().load(std::memory_order_relaxed);
}

/// Created on first use, so a run without profiling makes no NVTX call at all.
inline nvtxDomainHandle_t domain()
{
    static nvtxDomainHandle_t handle = nvtxDomainCreateA("PhysX");
    return handle;
}

inline void fillAttributes(nvtxEventAttributes_t& attributes, const char* eventName)
{
    memset(&attributes, 0, sizeof(attributes));
    attributes.version = NVTX_VERSION;
    attributes.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    attributes.messageType = NVTX_MESSAGE_TYPE_ASCII;
    attributes.message.ascii = eventName;
}

/// Pushes a range on the calling thread's stack. PhysX hands us zone names as
/// plain strings, so they are passed through as ASCII messages rather than
/// pre-registered handles.
///
/// Deliberately ungated: the push and its pop have to agree, and isEnabled() may
/// change between them. The caller decides once when the zone opens and records
/// that for the close (utils/ProfileZoneBalance.h), so consulting the gate again
/// here could pop a range that was never pushed.
inline void pushZone(const char* eventName)
{
    nvtxEventAttributes_t attributes;
    fillAttributes(attributes, eventName);
    nvtxDomainRangePushEx(domain(), &attributes);
}

/// Ungated for the same reason as pushZone: call this only for a zone whose push
/// actually happened.
inline void popZone()
{
    nvtxDomainRangePop(domain());
}

// Cross-thread ("detached") zones cannot use the per-thread push/pop stack above:
// they may close on a different thread than they opened on, and sibling phases do
// not close in LIFO order. They need explicit start/end range ids instead.
//
// PhysX does not carry that id for us. PX_PROFILE_STOP_CROSSTHREAD passes NULL as
// the callback's profilerData (physx/include/common/PxProfileZone.h), discarding
// whatever zoneStart() returned, so the pairing has to be reconstructed here. Both
// calls do receive the same zone name and context id -- "Basic.simulate" with the
// scene's context id, started in NpScene.cpp and stopped in NpSceneFetchResults.cpp
// -- and that pair is the key used below.
//
// Starts are kept as a stack per key so a repeated start before its stop closes in
// reverse order rather than leaking the earlier id.

using DetachedKey = std::pair<std::string, uint64_t>;

inline std::mutex& detachedMutex()
{
    static std::mutex mutex;
    return mutex;
}

inline std::map<DetachedKey, std::vector<nvtxRangeId_t>>& detachedRanges()
{
    static std::map<DetachedKey, std::vector<nvtxRangeId_t>> ranges;
    return ranges;
}

inline void startDetachedZone(const char* eventName, uint64_t contextId)
{
    if (!isEnabled())
    {
        return;
    }
    nvtxEventAttributes_t attributes;
    fillAttributes(attributes, eventName);
    const nvtxRangeId_t rangeId = nvtxDomainRangeStartEx(domain(), &attributes);

    std::lock_guard<std::mutex> lock(detachedMutex());
    detachedRanges()[DetachedKey(eventName, contextId)].push_back(rangeId);
}

inline void endDetachedZone(const char* eventName, uint64_t contextId)
{
    if (!isEnabled())
    {
        return;
    }

    nvtxRangeId_t rangeId = 0;
    {
        std::lock_guard<std::mutex> lock(detachedMutex());
        auto it = detachedRanges().find(DetachedKey(eventName, contextId));
        // A stop with no matching start has nothing to close. That happens for the
        // first stop of a zone whose start ran before profiling was switched on.
        if (it == detachedRanges().end() || it->second.empty())
        {
            return;
        }
        rangeId = it->second.back();
        it->second.pop_back();
        if (it->second.empty())
        {
            detachedRanges().erase(it);
        }
    }
    nvtxDomainRangeEnd(domain(), rangeId);
}

/// Number of detached ranges started but not yet ended. Exposed for tests: an
/// unmatched start leaves an NVTX range open forever, which shows up in a capture
/// as a timeline that nests without ever closing.
inline size_t pendingDetachedCount()
{
    std::lock_guard<std::mutex> lock(detachedMutex());
    size_t count = 0;
    for (const std::pair<const DetachedKey, std::vector<nvtxRangeId_t>>& entry : detachedRanges())
    {
        count += entry.second.size();
    }
    return count;
}

#else

inline void setEnabled(bool)
{
}

inline bool isEnabled()
{
    return false;
}

inline void pushZone(const char*)
{
}

inline void popZone()
{
}

inline void startDetachedZone(const char*, uint64_t)
{
}

inline void endDetachedZone(const char*, uint64_t)
{
}

inline size_t pendingDetachedCount()
{
    return 0;
}

#endif

} // namespace nvtx
} // namespace physx
} // namespace omni
