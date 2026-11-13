// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-INPUT-DEVICE-001
// @covers AC-3 AC-12
//
// @implements REQ-READ-DEVICE-001
// @covers AC-13
//
// @implements REQ-TENSOR-FK-001
// @covers AC-6

#pragma once

#include <common/utilities/CudaShimWrappers.h>

#include <cuda.h>

#include <cstdint>

namespace omni
{
namespace physx
{

// A caller's CUDA stream as a stream-ordered session operation uses it (ADR-0052).
//
// The operation's work runs in the scene's CUDA context, and CUDA queues work on a stream only from that stream's own
// context. A stream of the scene's context carries the work itself, which is what lets a capture record it. A stream
// of another context cannot: the operation waits for it on the host instead, as for a group applied there, and runs
// on the scene's own stream; on a capturing stream it is refused. Warp and PyTorch use the device's primary context,
// which the scene shares only when it was created on an explicit CUDA device.
struct CallerStream
{
    uintptr_t stream = 0;
    uintptr_t waitEvent = 0;
    // The context `stream` belongs to; 0 for a zero stream, which names the scene's own default stream.
    uintptr_t context = 0;
    bool capturing = false;

    bool sharesContext(uintptr_t sceneContext) const
    {
        return stream == 0 || context == sceneContext;
    }
};

// Whether the legacy default stream of the current context is held by a capture: a blocking stream of the context is
// being captured. The legacy stream synchronizes with every blocking stream, so CUDA refuses work on it until that
// capture ends, and the attempt invalidates the capture; a query of it reports CUDA_ERROR_STREAM_CAPTURE_IMPLICIT and
// leaves the capture intact. A zero caller stream names that stream in the scene's context: ask with the scene's
// context current.
inline bool currentLegacyStreamHeldByCapture()
{
    IOptionalCuda* const cuda = cudaShimWrappers::getCudaShim();
    bool capturing = false;
    int status = 0;
    return cuda && !cuda->streamIsCapturing(0, &capturing, &status) && status == CUDA_ERROR_STREAM_CAPTURE_IMPLICIT;
}

// The oldest driver CUDA graph capture works with: CUDA 12.5. An older driver refuses cuStreamGetCtx for a stream being
// captured (CUDA_ERROR_STREAM_CAPTURE_UNSUPPORTED) and invalidates the capture, so a capture is refused before its
// stream is resolved.
constexpr int kCaptureMinimumDriverVersion = 12050;

// The CUDA version the driver supports (cuDriverGetVersion), or 0 when it cannot be asked.
inline int cudaDriverVersion()
{
    IOptionalCuda* const cuda = cudaShimWrappers::getCudaShim();
    int version = 0;
    if (!cuda || !cuda->driverGetVersion(&version, nullptr))
        return 0;
    return version;
}

// Whether a stream being captured can be resolved on a driver reporting `driverVersion`.
inline bool driverSupportsCapture(int driverVersion)
{
    return driverVersion >= kCaptureMinimumDriverVersion;
}

// Resolves the caller's stream. Call before the scene's context is pushed: a context-relative handle (the legacy
// or per-thread default stream) names the default stream of the caller's current context, as for a host wait.
// Fails with CUDA_ERROR_STREAM_CAPTURE_IMPLICIT in `status` for the legacy default stream while a capture holds it, and
// with CUDA_ERROR_STREAM_CAPTURE_UNSUPPORTED for a stream being captured on a driver older than CUDA 12.5, leaving the
// capture valid.
inline bool resolveCallerStream(uintptr_t stream, uintptr_t waitEvent, CallerStream& out, int& status)
{
    out = CallerStream{};
    out.stream = stream;
    out.waitEvent = waitEvent;
    status = 0;
    if (stream == 0)
        return true;
    // The capture status first: while a capture holds the legacy default stream, asking for that stream's context
    // invalidates the capture, and asking whether it is capturing does not.
    IOptionalCuda* const cuda = cudaShimWrappers::getCudaShim();
    if (!cuda || !cuda->streamIsCapturing(stream, &out.capturing, &status))
        return false;
    if (out.capturing && !driverSupportsCapture(cudaDriverVersion()))
    {
        status = CUDA_ERROR_STREAM_CAPTURE_UNSUPPORTED;
        return false;
    }
    return cuda->streamGetCtx(stream, &out.context, &status);
}

} // namespace physx
} // namespace omni
