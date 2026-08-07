// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "internal/sidecar/ovphysxInternalExport.h"

#include <cstdint>

namespace omni
{
namespace physx
{
struct IPhysx;
struct IPhysxVisualization;
} // namespace physx
} // namespace omni

// Sidecar lifecycle entry points (libovphysx_internal).
// Currently just the version probe used by the SDK loader as a sanity-check
// handshake after dlopen/LoadLibrary. Future load/init/shutdown hooks live
// here too.

// This is not a compatibility promise. ovphysx and its sidecar may break ABI
// while the SDK is pre-1.0; the check only rejects stale, mismatched binaries
// before either library calls a sidecar function with the wrong signature.
// Bump when the ABI of any sidecar export changes.
// 1 -> 2: added parent_positions_xyz / num_parent_positions parameters.
// 2 -> 3: replaced with parent_transforms (7 floats: pos+quat), dropped count param.
// 3 -> 4: the SDK loader now seeds the sidecar's framework and OmniCore built-ins once at load via
//         ovphysx_internal_set_framework() / ovphysx_internal_set_omni_builtins().
// 4 -> 5: hardware restrictions (cpu_only, active_cuda_gpus) moved to create_args.
// 5 -> 6: injected owner-side PhysX runtime accessors; sidecar no longer
//         acquires IPhysx / IPhysxReplicator from Carbonite.
// 6 -> 7: dropped the stale framework pointer from the kinematic-update sidecar export.
// 7 -> 8: injected the owner-side IPhysxVisualization accessor; sidecar no longer
//         acquires that interface from Carbonite.
// 8 -> 9: dropped the unused IPhysxReplicator accessor from the PhysX runtime-accessor setter.
// 9 -> 10: added the checked tokenized visualization-scope sidecar export.
#define OVPHYSX_INTERNAL_INTERFACE_VERSION 10

// Filename of the sidecar shared object (no path). The SDK loader passes this
// to dlopen/LoadLibrary; secondary call sites that dlsym into the already-loaded
// module via RTLD_NOLOAD / GetModuleHandle use the same name to find it.
#ifdef _WIN32
#define OVPHYSX_INTERNAL_LIB_NAME "ovphysx_internal.dll"
#else
#define OVPHYSX_INTERNAL_LIB_NAME "libovphysx_internal.so"
#endif

using OvphysxInternalGetPhysxInterfaceFn = omni::physx::IPhysx* (*)();
using OvphysxInternalGetPhysxVisualizationInterfaceFn = omni::physx::IPhysxVisualization* (*)();

extern "C" {

// Query the sidecar interface version. The SDK loader compares against
// OVPHYSX_INTERNAL_INTERFACE_VERSION after loading the sidecar and refuses
// to use it on mismatch.
OVPHYSX_INTERNAL_API uint32_t ovphysx_plugin_version();

// libovphysx owns the single static PhysX runtime. The sidecar is a separate
// DLL/SO, so it receives callbacks to that owner runtime instead of acquiring
// these interfaces from Carbonite or linking another OvruntimePhysX copy.
OVPHYSX_INTERNAL_API void ovphysx_internal_set_physx_runtime_accessors(
    OvphysxInternalGetPhysxInterfaceFn getPhysxInterface,
    OvphysxInternalGetPhysxVisualizationInterfaceFn getPhysxVisualizationInterface);

// Inject the ovstage-native output-read entry points (omni::physx::ovx*). They live
// in the owner library's statically-linked OvruntimePhysX; the sidecar has no copy
// of its own and the owner's symbols are not exported, so the owner passes their
// addresses here at load time (as void* to keep this header free of the ovx/ovstage
// read types). Mirrors ovphysx_internal_set_physx_runtime_accessors().
OVPHYSX_INTERNAL_API void ovphysx_internal_set_ovx_read_accessors(
    void* query, void* fetchQueryResult, void* dict, void* read,
    void* fetch, void* releaseGroup, void* releaseRead, void* releaseQuery);

} // extern "C"
