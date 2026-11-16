// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "internal/sdk/ovphysxSDKSidecarLoader.hpp"
#include "internal/sidecar/ovphysxInternal.h"
#include "internal/sidecar/ovphysxInternalInterop.h"
#include "internal/sidecar/ovphysxInternalObjectChange.h"
#include <omni/physx/PhysXRuntime.h>
#include <omni/physx/IOvxPhysicsRead.h>
#include <omni/physx/IOvxPhysicsWrite.h>
#include <carb/Framework.h>
#include <atomic>
#include <cstring>
#include <mutex>

// The sidecar header mirrors OvxCommitFailure so the C layer can read the out-param without
// depending on ovruntime. This is the one translation unit that sees both, so it is where the copy
// is held to the original.
static_assert(static_cast<uint32_t>(omni::physx::kOvxReadOmissionUnsupportedAttribute) ==
                  OVPHYSX_READ_OMISSION_UNSUPPORTED_ATTRIBUTE,
              "Read omission attribute values must agree across the sidecar boundary");
static_assert(static_cast<uint32_t>(omni::physx::kOvxReadOmissionUnsupportedConfiguration) ==
                  OVPHYSX_READ_OMISSION_UNSUPPORTED_CONFIGURATION,
              "Read omission configuration values must agree across the sidecar boundary");
static_assert(static_cast<uint32_t>(omni::physx::kOvxReadOmissionNotReady) == OVPHYSX_READ_OMISSION_NOT_READY,
              "Read omission readiness values must agree across the sidecar boundary");
static_assert(static_cast<int32_t>(omni::physx::kOvxCommitFailureNotLive) == kOvphysxCommitFailureNotLive,
              "OvxCommitFailure::NotLive and its sidecar mirror disagree");
static_assert(static_cast<int32_t>(omni::physx::kOvxCommitFailurePublish) == kOvphysxCommitFailurePublish,
              "OvxCommitFailure::Publish and its sidecar mirror disagree");
static_assert(static_cast<int32_t>(omni::physx::kOvxCommitFailureBusy) == kOvphysxCommitFailureBusy,
              "OvxCommitFailure::Busy and its sidecar mirror disagree");
// Check the numeric value of each known mirrored member against its runtime counterpart; a new
// OvxCommitFailure value still needs its mirror and consumer mapping added here.
static_assert(static_cast<int32_t>(omni::physx::kOvxCommitFailureNone) == kOvphysxCommitFailureNone,
              "OvxCommitFailure::None and its sidecar mirror disagree");
static_assert(static_cast<int32_t>(omni::physx::kOvxCommitFailureInvalidArgument) ==
                  kOvphysxCommitFailureInvalidArgument,
              "OvxCommitFailure::InvalidArgument and its sidecar mirror disagree");


namespace
{
std::atomic<bool> loaded{false};
std::mutex loadMutex;

// The original sidecar implementation units are linked into this image. There is
// one framework and one set of Omni globals, so no peer-DSO globals are injected.
void injectRuntimeAccessors()
{
    ovphysx_internal_set_physx_runtime_accessors(
        &omni::physx::runtime::tryGetPhysxInterface,
        &omni::physx::runtime::tryGetPhysxVisualizationInterface);
    ovphysx_internal_set_ovx_read_accessors(
        reinterpret_cast<void*>(&omni::physx::ovxQuery),
        reinterpret_cast<void*>(&omni::physx::ovxQuerySelected),
        reinterpret_cast<void*>(&omni::physx::ovxReadAttributes),
        reinterpret_cast<void*>(&omni::physx::ovxFetchReadNext),
        reinterpret_cast<void*>(&omni::physx::ovxReleaseGroup),
        reinterpret_cast<void*>(&omni::physx::ovxReleaseRead),
        reinterpret_cast<void*>(&omni::physx::ovxReleaseQuery));
    ovphysx_internal_set_ovx_contact_accessors(reinterpret_cast<void*>(&omni::physx::ovxQueryContacts));
    ovphysx_internal_set_ovx_write_accessors(
        reinterpret_cast<void*>(&omni::physx::ovxWriteAttributes),
        reinterpret_cast<void*>(&omni::physx::ovxFetchWriteNext),
        reinterpret_cast<void*>(&omni::physx::ovxCommitGroup),
        reinterpret_cast<void*>(&omni::physx::ovxReleaseWrite));
    ovphysx_internal_set_ovx_masked_commit_accessor(
        reinterpret_cast<void*>(&omni::physx::ovxCommitGroupMasked),
        reinterpret_cast<void*>(&omni::physx::ovxValidateCommit));
}
}

bool refreshInternalSidecarRuntime()
{
    // All operations share the owner's globals and consult its runtime accessors.
    return carb::getFramework() != nullptr;
}

bool loadInternalSidecar()
{
    std::lock_guard<std::mutex> guard(loadMutex);
    if (!carb::getFramework() || !omni::physx::runtime::tryGetPhysxInterface() ||
        ovphysx_plugin_version() != OVPHYSX_INTERNAL_INTERFACE_VERSION)
        return false;
    if (loaded.load(std::memory_order_acquire))
        return true;
    injectRuntimeAccessors();

    g_sidecarOutputQuery.store(&ovphysx_internal_output_query, std::memory_order_release);
    g_sidecarOutputQuerySelected.store(&ovphysx_internal_output_query_selected, std::memory_order_release);
    g_sidecarOutputQueryContacts.store(&ovphysx_internal_output_query_contacts, std::memory_order_release);
    g_sidecarReadOutputs.store(&ovphysx_internal_read_outputs, std::memory_order_release);
    g_sidecarFetchReadNext.store(&ovphysx_internal_fetch_read_next, std::memory_order_release);
    g_sidecarReleaseGroup.store(&ovphysx_internal_release_group, std::memory_order_release);
    g_sidecarReleaseRead.store(&ovphysx_internal_release_read, std::memory_order_release);
    g_sidecarReleaseQuery.store(&ovphysx_internal_release_query, std::memory_order_release);
    g_sidecarWriteAttribute.store(&ovphysx_internal_write_attribute, std::memory_order_release);
    g_sidecarFetchWriteNext.store(&ovphysx_internal_fetch_write_next, std::memory_order_release);
    g_sidecarCommitGroup.store(&ovphysx_internal_commit_group, std::memory_order_release);
    g_sidecarCommitGroupMasked.store(&ovphysx_internal_commit_group_masked, std::memory_order_release);
    g_sidecarValidateCommit.store(&ovphysx_internal_validate_commit, std::memory_order_release);
    g_sidecarReleaseWrite.store(&ovphysx_internal_release_write, std::memory_order_release);
    g_sidecarSubscribeObjectChanges.store(&ovphysx_internal_subscribe_object_changes, std::memory_order_release);
    g_sidecarUnsubscribeObjectChanges.store(&ovphysx_internal_unsubscribe_object_changes, std::memory_order_release);
    g_sidecarEnableVisualization.store(&ovphysx_internal_enable_visualization, std::memory_order_release);
    g_sidecarSetVizParameter.store(&ovphysx_internal_set_visualization_parameter, std::memory_order_release);
    g_sidecarSetVizParameterValue.store(&ovphysx_internal_set_visualization_parameter_value, std::memory_order_release);
    g_sidecarSetVizScopeTokens.store(&ovphysx_internal_set_visualization_scope_tokens, std::memory_order_release);
    g_sidecarSetVizScale.store(&ovphysx_internal_set_visualization_scale, std::memory_order_release);
    g_sidecarSetVizCullingBox.store(&ovphysx_internal_set_visualization_culling_box, std::memory_order_release);
    g_sidecarGetDebugPoints.store(&ovphysx_internal_get_debug_points, std::memory_order_release);
    g_sidecarGetDebugLines.store(&ovphysx_internal_get_debug_lines, std::memory_order_release);
    g_sidecarGetDebugTriangles.store(&ovphysx_internal_get_debug_triangles, std::memory_order_release);
    g_sidecarDebugRenderApply.store(&ovphysx_internal_debug_render_apply, std::memory_order_release);
    g_sidecarDebugRenderCopy.store(&ovphysx_internal_debug_render_copy, std::memory_order_release);
    loaded.store(true, std::memory_order_release);
    return true;
}

void* resolveSidecarSymbol(const char* name)
{
    if (!name || !loaded.load(std::memory_order_acquire))
        return nullptr;
#define STATIC_SIDECAR_SYMBOL(symbol) \
    if (std::strcmp(name, #symbol) == 0) return reinterpret_cast<void*>(&symbol)
    STATIC_SIDECAR_SYMBOL(ovphysx_plugin_version);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_get_physx_ptr);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_physx_runtime_accessors);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_ovx_read_accessors);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_ovx_write_accessors);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_ovx_contact_accessors);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_ovx_masked_commit_accessor);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_output_query);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_output_query_selected);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_output_query_contacts);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_read_outputs);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_fetch_read_next);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_release_group);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_release_read);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_release_query);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_write_attribute);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_fetch_write_next);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_commit_group);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_commit_group_masked);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_validate_commit);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_release_write);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_subscribe_object_changes);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_unsubscribe_object_changes);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_enable_visualization);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_visualization_parameter);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_visualization_parameter_value);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_visualization_scope_tokens);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_visualization_scale);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_set_visualization_culling_box);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_get_debug_points);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_get_debug_lines);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_get_debug_triangles);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_debug_render_apply);
    STATIC_SIDECAR_SYMBOL(ovphysx_internal_debug_render_copy);
#undef STATIC_SIDECAR_SYMBOL
    return nullptr;
}
