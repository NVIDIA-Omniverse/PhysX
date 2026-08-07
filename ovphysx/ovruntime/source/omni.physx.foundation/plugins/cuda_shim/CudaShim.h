// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Internal CUDA driver shim -- runtime-loaded, no link to libcuda.
// Exposes functions that match the IOptionalCuda function-table signatures.
// Used by PhysXFoundation.cpp to populate the static IOptionalCuda function table.

#pragma once

#include <omni/physx/IOptionalCuda.h>
#include <cuda.h>

namespace omni
{
namespace physx
{
namespace cudaShim
{

// Populate all function pointers in the IOptionalCuda interface.
// Called while building the foundation module's static IOptionalCuda table.
void fill(IOptionalCuda& iface);

// Returns true if the CUDA driver library loaded, cuInit succeeded, and at least
// one device is present. Safe on CPU-only machines -- uses dlopen/LoadLibrary
// internally, never calls cuInit via the static stub link.
bool isCudaAvailable();

// Returns the ordinal of the first available CUDA device, or -1 if none.
// Routes all CUDA calls through the shim's dlsym-resolved function pointers.
int selectBestPhysicsDevice();

// Activate the primary CUDA context for device ordinal and make it current on
// this thread (Driver API equivalent of cudaSetDevice).  Retains once per ordinal
// per process; releases only if cuCtxSetCurrent fails on the first call for that
// ordinal.  Safe to call on CPU-only machines -- ensureInit() returns false and
// the call is a no-op.
bool setDevice(int ordinal);

// Wrappers for the specific driver-API calls PhysXFoundation.cpp needs.
// These forward to the shim's function-pointer table so that PhysXFoundation.cpp
// has no undefined CUDA symbols on Linux without libcuda.so.1.
CUresult cuCtxGetCurrent_(CUcontext* ctx);
CUresult cuCtxSetCurrent_(CUcontext ctx);
CUresult cuDeviceGetCount_(int* count);

} // namespace cudaShim
} // namespace physx
} // namespace omni
