// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-SIM-NVTX-001
 * @covers AC-7 AC-8
 */

#pragma once

// Pairing record for the non-detached PhysX profile zones.
//
// Those zones are published to two per-thread stacks: the Carbonite profiler's
// push/pop stack, and NVTX's per-thread range stack. Both require that a zone
// closes on the thread that opened it. The PhysX SDK upholds this by flagging
// genuinely cross-thread zones as detached (PX_PROFILE_START_CROSSTHREAD), which
// the callback routes to explicit start/end ranges instead.
//
// A zone that crossed threads *without* that flag would silently corrupt both
// stacks, and nothing in the callback signature reveals it. Rather than assume the
// SDK keeps its side of the bargain across version bumps, the pairing is checked
// here: an unmatched close is dropped and reported instead of being published.
//
// The same record also carries the NVTX decision for each open zone. That gate is
// mutable -- creating a second PhysX SDK writes it while another thread may have
// zones open -- so re-reading it at close time could pop a range that was never
// pushed, which closes the enclosing range instead and shifts every later nesting
// on that thread. Deciding once when the zone opens removes the window; the close
// follows the recorded bit rather than the current gate.
//
// Both are per-thread, so neither needs synchronization. Cost is a thread-local
// increment and one bit per zone, only on the path where a profiler sink is
// already installed.

#include <carb/logging/Log.h>

#include <atomic>
#include <cstdint>

namespace omni
{
namespace physx
{
namespace profilebalance
{

inline int32_t& threadZoneDepth()
{
    static thread_local int32_t depth = 0;
    return depth;
}

/// One bit per open zone on the calling thread, indexed by the zone's depth:
/// set when that zone pushed an NVTX range. PhysX nests its zones a handful of
/// levels deep, so a single word covers real nesting with room to spare.
inline uint64_t& threadNvtxMask()
{
    static thread_local uint64_t mask = 0;
    return mask;
}

/// Depths beyond this are still counted for the balance check but carry no NVTX
/// bit, so such a zone is neither pushed nor popped. Losing detail that deep is
/// preferable to an unbalanced pair.
constexpr int32_t kMaxTrackedDepth = 64;

inline std::atomic<uint64_t>& unmatchedZoneEnds()
{
    static std::atomic<uint64_t> count{ 0 };
    return count;
}

/// Records that a non-detached zone opened on the calling thread, together with
/// the NVTX decision for it. Returns whether the NVTX range should actually be
/// pushed, which is `wantNvtx` except past the tracking depth.
inline bool onZoneStart(bool wantNvtx)
{
    const int32_t depth = threadZoneDepth()++;
    if (depth >= kMaxTrackedDepth)
    {
        return false;
    }

    const uint64_t bit = uint64_t(1) << depth;
    if (wantNvtx)
    {
        threadNvtxMask() |= bit;
    }
    else
    {
        threadNvtxMask() &= ~bit;
    }
    return wantNvtx;
}

/// Records a zone that publishes to the Carbonite sink only. For tests.
inline void onZoneStart()
{
    onZoneStart(false);
}

/// True when the calling thread has a matching open zone, so the close may be
/// published to the profiler sinks. False for an unmatched close, which the caller
/// must drop: publishing it would close a zone this thread does not own. Only the
/// first occurrence is logged, since a broken zone pair tends to repeat per step.
///
/// `nvtxWasPushed` reports the decision recorded when this zone opened, and is the
/// only thing the caller may use to decide the pop -- see the note at the top of
/// this file on why the live gate must not be consulted again here.
inline bool onZoneEnd(bool& nvtxWasPushed)
{
    nvtxWasPushed = false;

    int32_t& depth = threadZoneDepth();
    if (depth <= 0)
    {
        if (unmatchedZoneEnds().fetch_add(1, std::memory_order_relaxed) == 0)
        {
            CARB_LOG_ERROR(
                "A PhysX profile zone closed on a thread that did not open it. Non-detached zones "
                "must not cross threads: doing so corrupts the Carbonite and NVTX per-thread zone "
                "stacks, so this close is being dropped. If this appears after a PhysX SDK update, "
                "a zone that crosses threads is missing the detached flag (REQ-SIM-NVTX-001 AC-7).");
        }
        return false;
    }

    --depth;
    if (depth < kMaxTrackedDepth)
    {
        nvtxWasPushed = ((threadNvtxMask() >> depth) & uint64_t(1)) != 0;
    }
    return true;
}

/// Balance check only, discarding the NVTX decision. For tests.
inline bool onZoneEnd()
{
    bool nvtxWasPushed = false;
    return onZoneEnd(nvtxWasPushed);
}

/// Open non-detached zones on the calling thread. For tests.
inline int32_t openZoneDepth()
{
    return threadZoneDepth();
}

/// Unmatched closes observed process-wide. For tests.
inline uint64_t unmatchedZoneEndCount()
{
    return unmatchedZoneEnds().load(std::memory_order_relaxed);
}

} // namespace profilebalance
} // namespace physx
} // namespace omni
