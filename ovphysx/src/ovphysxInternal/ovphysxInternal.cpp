// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Sidecar lifecycle entry points: the version probe the SDK loader uses as a
// handshake after dlopen/LoadLibrary, and the injected PhysX runtime accessors.

#include "internal/sidecar/ovphysxInternal.h"
#include "ovphysxInternalPhysXAccess.hpp"

namespace
{

OvphysxInternalGetPhysxInterfaceFn g_getPhysxInterface = nullptr;
OvphysxInternalGetPhysxVisualizationInterfaceFn g_getPhysxVisualizationInterface = nullptr;

} // namespace

extern "C" OVPHYSX_INTERNAL_API uint32_t ovphysx_plugin_version() {
    return OVPHYSX_INTERNAL_INTERFACE_VERSION;
}

extern "C" OVPHYSX_INTERNAL_API void ovphysx_internal_set_physx_runtime_accessors(
    OvphysxInternalGetPhysxInterfaceFn getPhysxInterface,
    OvphysxInternalGetPhysxVisualizationInterfaceFn getPhysxVisualizationInterface)
{
    // Written once by loadInternalSidecar() before any sidecar entry point is
    // called. The sidecar does not mutate these accessors after startup.
    g_getPhysxInterface = getPhysxInterface;
    g_getPhysxVisualizationInterface = getPhysxVisualizationInterface;
}

namespace ovphysx
{
namespace internal
{
namespace sidecar
{

omni::physx::IPhysx* tryGetInjectedPhysxInterface()
{
    return g_getPhysxInterface ? g_getPhysxInterface() : nullptr;
}

omni::physx::IPhysxVisualization* tryGetInjectedPhysxVisualizationInterface()
{
    return g_getPhysxVisualizationInterface ? g_getPhysxVisualizationInterface() : nullptr;
}

} // namespace sidecar
} // namespace internal
} // namespace ovphysx
