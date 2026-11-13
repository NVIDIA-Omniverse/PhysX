// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-INPUT-DEVICE-001
// @covers AC-12
//
// @implements REQ-READ-DEVICE-001
// @covers AC-11 AC-13

#pragma once

#include "tensors/gpu/CudaCommon.h"

#include <cstddef>
#include <cstdint>

namespace omni
{
namespace physx
{
namespace tensors
{

// How one stream-ordered ovstage operation runs on the device, shared by a write commit (ADR-0052) and a read
// (ADR-0055): its stream, the events PhysX joins it through, and device scratch the operation's session or query
// owns.
struct OvStageDeviceStreamArgs
{
    // Where the operation's kernels run. PhysX joins it through `fork` and the stream rejoins PhysX through
    // `join`, so everything queued on it afterwards runs after the operation.
    CUstream stream = nullptr;
    CUevent fork = nullptr;
    CUevent join = nullptr;
    // Device memory the session or query owns, laid out by the view method the operation goes through; its leading
    // list, if it has one (a root write's rows, say), is uploaded before the operation runs.
    void* scratch = nullptr;
    size_t scratchBytes = 0;
    // True while `stream` is being captured: the operation may not block, allocate or upload.
    bool capturing = false;
    // Set when `stream` could not be made to wait for a PhysX call: nothing queued on it afterwards proves that
    // call finished.
    bool* unjoined = nullptr;

    // Whether the operation's arrays, the last ending at `end`, lie inside the scratch that was sized for it. Logs
    // when they do not -- the layout outgrew the size its view reported -- and the operation must do nothing.
    bool scratchHolds(const void* end, const char* attribName) const
    {
        if (!scratch || static_cast<const char*>(end) <= static_cast<const char*>(scratch) + scratchBytes)
            return true;
        CARB_LOG_ERROR("%s: the operation's device arrays outgrow the %zu bytes of scratch sized for it; nothing "
                       "was done.",
                       attribName, scratchBytes);
        return false;
    }
};

// How one ovstage write runs on the device (ADR-0052).
//
// A session commit passes the caller's stream, an optional row mask and device buffers the session owns:
// its work is then ordered on that stream alone, touches no buffer another operation can resize or
// overwrite besides the views' tables, which the host rewrites only after it (GpuSimulationData::rewriteTable),
// and can be captured into a CUDA graph and replayed. The ovstage drain passes no scratch, which keeps the
// view's own buffers and apply events; it runs on the stream ovstage's device read hands back
// (drainDeviceWriteArgs(applyEvent, stream)), the legacy default stream only when that stream is null.
struct OvStageDeviceWriteArgs : OvStageDeviceStreamArgs
{
    // One byte per group row on the column's device, nonzero selecting the row. Null selects every row.
    const uint8_t* mask = nullptr;
    // Link force and wrench writes: the articulations prepareLinkWrenchOvStage resolved at open.
    uint32_t applyCount = 0;
    bool applyWholeView = false;

    bool sessionOwned() const
    {
        return scratch != nullptr;
    }
};

// How one ovstage read runs on the device (ADR-0055): on the read's stream, in scratch its query holds, so a held
// query's reads reuse one device layout and a capture records reads that replay against it.
using OvStageDeviceReadArgs = OvStageDeviceStreamArgs;

// Carves consecutive device arrays out of a session's scratch, each aligned for any element type.
class OvStageScratch
{
public:
    explicit OvStageScratch(void* base) : mBase(static_cast<char*>(base))
    {
    }

    template <typename T>
    T* take(size_t count)
    {
        T* const at = reinterpret_cast<T*>(mBase ? mBase + mUsed : nullptr);
        mUsed += alignUp(count * sizeof(T));
        return at;
    }

    size_t used() const
    {
        return mUsed;
    }

    static size_t alignUp(size_t bytes)
    {
        return (bytes + 255u) & ~size_t(255u);
    }

private:
    char* mBase;
    size_t mUsed = 0;
};

// A PhysX direct-GPU call ordered on `deviceArgs.stream`: `fork` is recorded there and handed to PhysX as its
// start event, PhysX records `join` when done, and the stream waits for it. The stream waits even when PhysX
// refuses the call, since a refused call may still have queued work.
template <typename Call>
bool physxOnStream(const OvStageDeviceStreamArgs& deviceArgs, Call&& call)
{
    int status = 0;
    if (!getCudaShim()->eventRecord(
            reinterpret_cast<uintptr_t>(deviceArgs.fork), reinterpret_cast<uintptr_t>(deviceArgs.stream), &status))
    {
        CARB_LOG_ERROR("Ordering a PhysX direct-GPU call after stream 0x%llx failed (CUresult %d).",
                       static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(deviceArgs.stream)), status);
        return false;
    }
    const bool issued = call(deviceArgs.fork, deviceArgs.join);
    if (!getCudaShim()->streamWaitEvent(reinterpret_cast<uintptr_t>(deviceArgs.stream),
                                        reinterpret_cast<uintptr_t>(deviceArgs.join), 0, &status))
    {
        CARB_LOG_ERROR("Ordering stream 0x%llx after a PhysX direct-GPU call failed (CUresult %d).",
                       static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(deviceArgs.stream)), status);
        if (deviceArgs.unjoined)
            *deviceArgs.unjoined = true;
        return false;
    }
    return issued;
}

} // namespace tensors
} // namespace physx
} // namespace omni
