// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-PYTHON-READ-001
// @covers AC-4

// @implements REQ-CAPI-SESSION-001
// @covers AC-1 AC-5 AC-8

// @implements REQ-CAPI-DETACH-001
// @covers AC-6

/**
 * @implements REQ-CAPI-VEHICLE-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-SLEEP-001
 * @covers AC-1
 *
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8
 *
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-5
 */

/**
 * @implements REQ-CAPI-PHYSXPTR-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

/**
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-1 AC-1a AC-2 AC-3 AC-4 AC-5 AC-5a AC-6 AC-7 AC-8 AC-10 AC-13 AC-15
 *
 * @implements REQ-CAPI-INSTANCE-SCOPE-001
 * @covers AC-2 AC-5
 *
 * The public C write surface: ovphysx_write / ovphysx_fetch_write_next / ovphysx_commit_group /
 * ovphysx_release_write. Validation is C-first (AC-8), so every argument, handle and lifecycle
 * check lives here rather than in a frontend. The Python bindings are a thin mirror and inherit
 * these guarantees instead of restating them.
 */

/**
 * @implements REQ-CAPI-DEBUG-RENDER-001
 * @covers AC-8
 *
 * A debug-render query is not a sidecar query. Its shared dictionary is the
 * attached stage's path dictionary, or null when no stage is attached.
 *
 * @implements REQ-CAPI-READ-CONTACT-001
 * @covers AC-1 AC-3 AC-4 AC-5 AC-6
 *
 * @implements REQ-CAPI-READ-CONTACT-002
 * @covers AC-1 AC-2 AC-4 AC-5
 *
 * The contact sensor's public surface: query, selected-query and writability calls admit the type through
 * their OVPHYSX_QUERY_TARGET_COUNT-bounded gates, and kWritabilityTable classifies every contact column READ_ONLY.
 *
 * @implements REQ-CAPI-SELECT-001
 * @covers AC-1 AC-2 AC-3
 *
 * `ovphysx_query_selection`: the selected-query entry point, its prim_paths-wins precedence, and the
 * selection validation it returns on the receipt.
 *
 * @implements REQ-CAPI-SDF-001
 * @covers AC-1 AC-2
 *
 * `ovphysx_evaluate_sdf_shapes`: the SDF view's successor on the session query, with its tensor checks
 * returned on the receipt.
 */

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_attributes.h"
#include "ovphysx/ovphysx_geometry_query.h"
#include "LogManager.hpp"
#include "internal/Profiling.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include <omni/physx/IOvxPhysicsRead.h>
#include "internal/sdk/TaskContext.hpp"
#include "internal/sdk/DLPackConvert.h"
#include "internal/sidecar/ovphysxInternalInterop.h"

#include <carb/Framework.h>
#include <ovstage/ovstage_api/ovstage_api.h>
#include <carb/logging/Log.h>
#include <omni/physx/IOvxPhysicsRead.h>
#include <omni/physx/PhysXRuntime.h>
#include <omni/physx/IOptionalCuda.h>
#include <omni/physx/RuntimeError.h>

#include <mutex>
#include <string>
#include <unordered_set>

// Physics output read (ADR-0007) sidecar pointers, owned here next to their
// public-API consumers below. The loader writes them during loadInternalSidecar().
std::atomic<OvphysxSidecarOutputQueryFn>      g_sidecarOutputQuery{nullptr};
std::atomic<OvphysxSidecarOutputQuerySelectedFn> g_sidecarOutputQuerySelected{nullptr};
std::atomic<OvphysxSidecarOutputQueryContactsFn> g_sidecarOutputQueryContacts{nullptr};
std::atomic<OvphysxSidecarReadOutputsFn>      g_sidecarReadOutputs{nullptr};
std::atomic<OvphysxSidecarFetchReadNextFn>    g_sidecarFetchReadNext{nullptr};
std::atomic<OvphysxSidecarReleaseGroupFn>     g_sidecarReleaseGroup{nullptr};
std::atomic<OvphysxSidecarReleaseReadFn>      g_sidecarReleaseRead{nullptr};
std::atomic<OvphysxSidecarReleaseQueryFn>     g_sidecarReleaseQuery{nullptr};
std::atomic<OvphysxSidecarWriteAttributeFn>   g_sidecarWriteAttribute{nullptr};
std::atomic<OvphysxSidecarFetchWriteNextFn>   g_sidecarFetchWriteNext{nullptr};
std::atomic<OvphysxSidecarCommitGroupFn>      g_sidecarCommitGroup{nullptr};
std::atomic<OvphysxSidecarReleaseWriteFn>     g_sidecarReleaseWrite{nullptr};
// Debug-visualization sidecar fn-ptrs (loader resolves them; see ovphysxSidecarLoader.cpp).
std::atomic<OvphysxSidecarEnableVisualizationFn> g_sidecarEnableVisualization{nullptr};
std::atomic<OvphysxSidecarSetVizParameterFn>     g_sidecarSetVizParameter{nullptr};
std::atomic<OvphysxSidecarSetVizParameterValueFn> g_sidecarSetVizParameterValue{nullptr};
std::atomic<OvphysxSidecarSetVizScopeTokensFn> g_sidecarSetVizScopeTokens{nullptr};
std::atomic<OvphysxSidecarSetVizScaleFn>         g_sidecarSetVizScale{nullptr};
std::atomic<OvphysxSidecarSetVizCullingBoxFn>    g_sidecarSetVizCullingBox{nullptr};
std::atomic<OvphysxSidecarGetDebugBufferFn>      g_sidecarGetDebugPoints{nullptr};
std::atomic<OvphysxSidecarGetDebugBufferFn>      g_sidecarGetDebugLines{nullptr};
std::atomic<OvphysxSidecarGetDebugBufferFn>      g_sidecarGetDebugTriangles{nullptr};
std::atomic<OvphysxSidecarDebugRenderApplyFn>   g_sidecarDebugRenderApply{nullptr};
std::atomic<OvphysxSidecarDebugRenderCopyFn>    g_sidecarDebugRenderCopy{nullptr};

namespace {

using OutputHandles = std::unordered_map<uint64_t, bool>;
using OutputHandleMember = OutputHandles InstanceData::*;

// Handle ownership survives detach and does not depend on the active stage.
ovphysx_api_status_t outputOwner(const InstanceData& instance, uint64_t resource,
                                OutputHandleMember member, bool allowMissing = false)
{
    std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
    const OutputHandles& handles = instance.*member;
    const auto found = handles.find(resource);
    if (found != handles.end())
        return !found->second || allowMissing ? OVPHYSX_API_SUCCESS : OVPHYSX_API_NOT_FOUND;
    for (const std::pair<const ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>& entry : g_instances)
        if (entry.second.get() != &instance && (entry.second.get()->*member).count(resource))
            return OVPHYSX_API_NOT_FOUND;
    return allowMissing ? OVPHYSX_API_SUCCESS : OVPHYSX_API_NOT_FOUND;
}

// Reserve the ownership node before opening a backend resource. Reinsert the
// returned handle into the same set without allocating after resource creation.
OutputHandles::node_type reserveOutputHandle(InstanceData& instance, OutputHandleMember member)
{
    std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
    OutputHandles& handles = instance.*member;
    handles.emplace(0, false);
    return handles.extract(0);
}

void publishOutputHandle(InstanceData& instance, OutputHandleMember member,
                         OutputHandles::node_type node, uint64_t handle)
{
    node.key() = handle;
    std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
    (instance.*member).insert(std::move(node));
}

template <typename Work>
ovphysx_enqueue_result_t outputTask(ovphysx_instance_handle_t handle, Work&& work)
{
    ovphysx::TaskContext admission;
    try
    {
        const std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid instance handle");
        // Pending work is drained by the operation on this instance. Peer
        // execution must not reject output; callback and same-owner reentry do.
        // Application log callbacks have separate thread-local tracking and may
        // run while process-shared backend mutexes are held.
        if (ovphysx::isInLogCallback() || omni::physx::runtime::isRuntimeBusy(instance->attachHandle, true))
            return set_enqueue_error(OVPHYSX_API_INVALID_STATE, "Output operation cannot reenter runtime execution");
        return work(*instance, admission);
    }
    catch (const std::exception& error)
    {
        return admission.fail(error.what());
    }
    catch (...)
    {
        return admission.fail("Output operation failed");
    }
}

ovphysx_enqueue_result_t outputFailure(ovphysx::TaskContext& admission,
                                      const omni::physx::RuntimeErrorScope& error,
                                      const char* fallback)
{
    return admission.fail(error.message()[0] ? error.message() : fallback);
}

ovphysx_enqueue_result_t releaseOutput(ovphysx_instance_handle_t handle, uint64_t resource,
                                      OutputHandleMember member, int (*release)(uint64_t))
{
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        const ovphysx_api_status_t owner = outputOwner(instance, resource, member, true);
        if (owner != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(owner, "Resource belongs to another instance");
        bool live = false;
        {
            std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
            const OutputHandles& handles = instance.*member;
            const auto found = handles.find(resource);
            live = found != handles.end() && !found->second;
        }
        if (!live)
        {
            admission.create_task(handle);
            return admission.complete();
        }
        if (!release)
            return set_enqueue_error(OVPHYSX_API_ERROR, "Output release API is unavailable");
        admission.create_task(handle);
        {
            std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
            (instance.*member).at(resource) = true;
        }
        omni::physx::RuntimeErrorScope error;
        const int released = release(resource);
        if (released >= 0)
        {
            std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
            (instance.*member).erase(resource);
        }
        if (released <= 0 || error.message()[0])
            return outputFailure(admission, error, "Output release failed; cleanup remains owned by the SDK");
        return admission.complete();
    });
}

} // anonymous namespace

namespace {
void cleanupOutputHandles(OutputHandles& handles, int (*release)(uint64_t)) noexcept
{
    OutputHandles releasedHandles;
    {
        // Peer instances inspect ownership under this lock. Retire ownership
        // before cleanup, then leave CUDA waits outside the global lock.
        std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
        releasedHandles.swap(handles);
    }
    for (const std::pair<const uint64_t, bool>& resource : releasedHandles)
    {
        try
        {
            omni::physx::RuntimeErrorScope error;
            if (!release || release(resource.first) <= 0 || error.message()[0])
                CARB_LOG_ERROR("Output session cleanup failed: %s",
                               error.message()[0] ? error.message() : "backend release unavailable or incomplete");
        }
        catch (...)
        {
            CARB_LOG_ERROR("Output session cleanup failed with an exception");
        }
    }
}
} // namespace

int releaseDebugOrQuery(uint64_t handle)
{
    const OvphysxSidecarReleaseQueryFn release = g_sidecarReleaseQuery.load(std::memory_order_acquire);
    return release ? release(handle) : -1;
}

int releaseDebugOrRead(uint64_t handle)
{
    const OvphysxSidecarReleaseReadFn release = g_sidecarReleaseRead.load(std::memory_order_acquire);
    return release ? release(handle) : -1;
}

int releaseDebugOrWrite(uint64_t handle)
{
    const OvphysxSidecarReleaseWriteFn release = g_sidecarReleaseWrite.load(std::memory_order_acquire);
    return release ? release(handle) : -1;
}

void cleanup_output_writes(InstanceData& instance) noexcept
{
    // Write sessions borrow scene pointers; retire them before the scene detaches.
    cleanupOutputHandles(instance.output_writes, releaseDebugOrWrite);
}

void cleanup_output_sessions(ovphysx_instance_handle_t handle, InstanceData& instance) noexcept
{
    // Drop visualization on this instance's scenes before its sessions go away.
    omni::physx::ovxDebugRenderForget(handle);
    // Release before unloading the sidecar. Read buffers retain their CUDA
    // contexts across stage detach.
    cleanup_output_writes(instance);
    cleanupOutputHandles(instance.output_reads, releaseDebugOrRead);
    cleanupOutputHandles(instance.output_queries, releaseDebugOrQuery);
}


// ---- Physics output read (ADR-0007) ----
//
// The public surface validates the handle. Simulated-type queries require an
// attached ovstage Stage and then forward to the sidecar. OVPHYSX_QUERY_TARGET_DEBUG_RENDER
// is an instance target and does not require a stage.
// See docs/ovstage_integration.md for the ordinal-coupling principle.

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_query(ovphysx_instance_handle_t handle,
    ovphysx_query_target_t object_type, ovphysx_object_scope_t scope, ovphysx_query_handle_t* out_query,
    ovstage_query_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(query);
    if (out_query)
        *out_query = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        if (!out_query)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query: out_query is NULL");
        if (scope != OVPHYSX_SCOPE_ALL && scope != OVPHYSX_SCOPE_ACTIVE)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query: invalid object type or scope");
        if (object_type == OVPHYSX_QUERY_TARGET_DEBUG_RENDER)
        {
            const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
            if (drain != OVPHYSX_API_SUCCESS)
                return set_enqueue_error(drain, "ovphysx_query: failed to finish pending simulation");
            admission.create_task(handle);
            OutputHandles::node_type ownership = reserveOutputHandle(instance, &InstanceData::output_queries);
            const uint64_t resource = omni::physx::ovxOpenDebugRenderQuery(handle, static_cast<uint32_t>(scope));
            if (!resource)
                return admission.fail("ovphysx_query: failed to open a debug-render query");
            publishOutputHandle(instance, &InstanceData::output_queries, std::move(ownership), resource);
            *out_query = resource;
            return admission.complete();
        }
        // Bounded by the sentinel, not the last type's name, so a newly added type is accepted
        // without editing this line. Debug render returns above and never reaches the sidecar.
        if (static_cast<uint32_t>(object_type) >= static_cast<uint32_t>(OVPHYSX_QUERY_TARGET_COUNT))
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query: invalid object type or scope");
        if (!instance.ovstage_attached)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_query: attach an ovstage Stage first");
        const OvphysxSidecarOutputQueryFn query = g_sidecarOutputQuery.load(std::memory_order_acquire);
        if (!query)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_query: read API is unavailable");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(drain, "ovphysx_query: failed to finish pending simulation");
        admission.create_task(handle);
        OutputHandles::node_type ownership = reserveOutputHandle(instance, &InstanceData::output_queries);
        omni::physx::RuntimeErrorScope error;
        // The initial scene parse inserts parsed articulations into their scenes. Without it, a read or
        // write opened over this query before the first step finds none of them.
        if (ovphysx_ensure_physics_attached(handle) != OVPHYSX_API_SUCCESS)
            return outputFailure(admission, error, "ovphysx_query: failed to attach physics stage");
        ovstage_query_result_t result{};
        const uint64_t resource = query(instance.attachHandle, static_cast<uint32_t>(object_type),
                                        static_cast<uint32_t>(scope), out_summary ? &result : nullptr);
        if (!resource)
            return outputFailure(admission, error, "ovphysx_query: failed to open an output query");
        publishOutputHandle(instance, &InstanceData::output_queries, std::move(ownership), resource);
        *out_query = resource;
        if (out_summary)
            *out_summary = result;
        return admission.complete();
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_query_selection(ovphysx_instance_handle_t handle,
    ovphysx_query_target_t object_type, const ovphysx_query_selection_t* selection,
    ovphysx_query_handle_t* out_query, ovstage_query_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(query_selection);
    if (out_query)
        *out_query = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        if (!out_query)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query_selection: out_query is NULL");
        if (!selection)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_selection: selection is NULL (the whole set is ovphysx_query)");
        // Bounded by the sentinel, not the last type's name, like ovphysx_query's gate.
        // Debug render is not a prim selection; ovphysx_query is its entry point.
        if (static_cast<uint32_t>(object_type) >= static_cast<uint32_t>(OVPHYSX_QUERY_TARGET_COUNT) ||
            object_type == OVPHYSX_QUERY_TARGET_DEBUG_RENDER)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query_selection: invalid object type");

        // prim_paths wins over pattern; the two are never combined (REQ-CAPI-SELECT-001 AC-2). prim_paths is
        // a list of native ovx_primpath_t handles the caller already resolved: validated like
        // ovphysx_debug_render_set_scope_tokens (no NULL array with a non-zero count, no OVX_INVALID_PRIMPATH
        // entry), NOT copied and NOT string-checked -- the caller owns path resolution. pattern is still a
        // glob string, validated for the embedded-NUL / oversized-component (REQ-CAPI-PATTERN-001) and
        // relative-path cases; the runtime resolves it at query-open and retains nothing.
        //
        // A NULL array with a non-zero count is a caller bug, rejected before precedence is decided:
        // otherwise it would read as "no explicit list" and quietly fall back to the pattern.
        if (!selection->prim_paths && selection->prim_paths_count > 0)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_selection: prim_paths is NULL with prim_paths_count > 0");
        const bool usingExplicitPaths = (selection->prim_paths != nullptr && selection->prim_paths_count > 0);
        if (usingExplicitPaths)
        {
            for (uint32_t i = 0; i < selection->prim_paths_count; i++)
                if (selection->prim_paths[i] == OVX_INVALID_PRIMPATH)
                    return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                        "ovphysx_query_selection: prim_paths must not contain OVX_INVALID_PRIMPATH");
        }
        else if (selection->pattern.ptr && selection->pattern.length > 0)
        {
            if (hasEmbeddedNul(selection->pattern))
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                    "ovphysx_query_selection: pattern contains an embedded NUL byte");
            if (hasOversizedPathComponent(selection->pattern))
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, oversizedPathComponentMessage("pattern"));
            // The runtime matcher would resolve a relative pattern from the root, but every other
            // pattern-taking entry point requires an absolute one; reject it so one pattern means the same
            // thing everywhere.
            if (selection->pattern.ptr[0] != '/')
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                    "ovphysx_query_selection: pattern must be an absolute path starting with '/'");
        }
        else
        {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_selection: selection must provide a pattern or prim_paths");
        }

        if (!instance.ovstage_attached)
            return set_enqueue_error(OVPHYSX_API_ERROR,
                "ovphysx_query_selection: attach an ovstage Stage first");
        const OvphysxSidecarOutputQuerySelectedFn querySelected =
            g_sidecarOutputQuerySelected.load(std::memory_order_acquire);
        if (!querySelected)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_query_selection: read API is unavailable");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(drain, "ovphysx_query_selection: failed to finish pending simulation");

        admission.create_task(handle);
        OutputHandles::node_type ownership = reserveOutputHandle(instance, &InstanceData::output_queries);
        omni::physx::RuntimeErrorScope error;
        // As in ovphysx_query: insert parsed articulations before a read or write opens over this query.
        if (ovphysx_ensure_physics_attached(handle) != OVPHYSX_API_SUCCESS)
            return outputFailure(admission, error, "ovphysx_query_selection: failed to attach physics stage");
        // Hand the runtime whichever selector is in play; the two are mutually exclusive above, so the other
        // is empty/NULL. The pattern buffer is the caller's and outlives the call. A zero resource means
        // FAILURE only (ovstage is attached); an empty match is still a valid, nonzero query -- discover it
        // in the optional creation summary (total_prim_count == 0).
        const ovx_primpath_t* primPaths = usingExplicitPaths ? selection->prim_paths : nullptr;
        const size_t primPathCount = usingExplicitPaths ? selection->prim_paths_count : 0u;
        const ovx_string_t glob{ selection->pattern.ptr, selection->pattern.length };
        const ovx_string_t* pattern = usingExplicitPaths ? nullptr : &glob;
        ovstage_query_result_t result{};
        const uint64_t resource = querySelected(instance.attachHandle, static_cast<uint32_t>(object_type),
                                                primPaths, primPathCount, pattern, out_summary ? &result : nullptr);
        if (!resource)
            return outputFailure(admission, error, "ovphysx_query_selection: failed to open an output query");
        publishOutputHandle(instance, &InstanceData::output_queries, std::move(ownership), resource);
        *out_query = resource;
        if (out_summary)
            *out_summary = result;
        return admission.complete();
    });
}

namespace
{
// A contact query's selections, validated like ovphysx_query_selection's one descriptor: the error text for
// the first invalid one, or empty when all are valid. `what` names the selection in the message.
std::string contactSelectionError(const ovphysx_query_selection_t& selection, const std::string& what)
{
    if (selection.prim_paths_count > 0 && !selection.prim_paths)
        return "ovphysx_query_contacts: " + what + " has prim_paths_count > 0 but prim_paths is NULL";
    if (selection.prim_paths && selection.prim_paths_count > 0)
    {
        for (uint32_t i = 0; i < selection.prim_paths_count; i++)
            if (selection.prim_paths[i] == OVX_INVALID_PRIMPATH)
                return "ovphysx_query_contacts: " + what + " prim_paths must not contain OVX_INVALID_PRIMPATH";
        return {};
    }
    if (selection.pattern.ptr && selection.pattern.length > 0)
    {
        if (hasEmbeddedNul(selection.pattern))
            return "ovphysx_query_contacts: " + what + " pattern contains an embedded NUL byte";
        if (hasOversizedPathComponent(selection.pattern))
            return "ovphysx_query_contacts: " + what + " " + oversizedPathComponentMessage("pattern");
        // As ovphysx_query_selection: the runtime matcher would resolve a relative pattern from the root, but
        // every pattern-taking entry point requires an absolute one.
        if (selection.pattern.ptr[0] != '/')
            return "ovphysx_query_contacts: " + what + " pattern must be an absolute path starting with '/'";
        return {};
    }
    return "ovphysx_query_contacts: " + what + " must provide a pattern or prim_paths";
}

// The runtime's view of one validated selection: the explicit handles, or else the glob.
omni::physx::OvxPrimSelection runtimeSelection(const ovphysx_query_selection_t& selection, ovx_string_t& glob)
{
    if (selection.prim_paths && selection.prim_paths_count > 0)
        return { selection.prim_paths, selection.prim_paths_count, nullptr };
    glob = ovx_string_t{ selection.pattern.ptr, selection.pattern.length };
    return { nullptr, 0u, &glob };
}
} // namespace

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_query_contacts(ovphysx_instance_handle_t handle,
    const ovphysx_contact_sensor_selection_t* selections, uint32_t selection_count, uint32_t filters_per_sensor,
    ovphysx_query_handle_t* out_query, ovstage_query_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(query_contacts);
    if (out_query)
        *out_query = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        if (!out_query)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query_contacts: out_query is NULL");
        if (!selections || selection_count == 0)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_contacts: selections must name at least one sensor selection");
        if (filters_per_sensor > omni::physx::kOvxMaxContactFilters)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_query_contacts: filters_per_sensor exceeds 21845");
        for (uint32_t s = 0; s < selection_count; s++)
        {
            const std::string where = "selection " + std::to_string(s);
            std::string error = contactSelectionError(selections[s].sensors, where + " sensors");
            if (error.empty() && filters_per_sensor > 0 && !selections[s].filters)
                error = "ovphysx_query_contacts: " + where + " has no filters while filters_per_sensor is " +
                        std::to_string(filters_per_sensor);
            for (uint32_t f = 0; error.empty() && f < filters_per_sensor; f++)
                error = contactSelectionError(selections[s].filters[f], where + " filter " + std::to_string(f));
            if (!error.empty())
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, error);
        }

        if (!instance.ovstage_attached)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_query_contacts: attach an ovstage Stage first");
        const OvphysxSidecarOutputQueryContactsFn queryContacts =
            g_sidecarOutputQueryContacts.load(std::memory_order_acquire);
        if (!queryContacts)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_query_contacts: read API is unavailable");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(drain, "ovphysx_query_contacts: failed to finish pending simulation");

        admission.create_task(handle);
        OutputHandles::node_type ownership = reserveOutputHandle(instance, &InstanceData::output_queries);
        omni::physx::RuntimeErrorScope error;
        // The runtime's descriptors point into the caller's buffers, which outlive the call; the globs and
        // filter arrays are held here for its duration.
        std::vector<ovx_string_t> globs(size_t(selection_count) * (1u + filters_per_sensor));
        std::vector<omni::physx::OvxPrimSelection> filters(size_t(selection_count) * filters_per_sensor);
        std::vector<omni::physx::OvxContactSensorSelection> runtime(selection_count);
        for (uint32_t s = 0; s < selection_count; s++)
        {
            ovx_string_t* const slot = &globs[size_t(s) * (1u + filters_per_sensor)];
            runtime[s].sensors = runtimeSelection(selections[s].sensors, slot[0]);
            for (uint32_t f = 0; f < filters_per_sensor; f++)
                filters[size_t(s) * filters_per_sensor + f] = runtimeSelection(selections[s].filters[f], slot[1 + f]);
            runtime[s].filters = filters_per_sensor ? &filters[size_t(s) * filters_per_sensor] : nullptr;
        }
        // A zero resource means FAILURE -- an unpairable filter or a repeated one, named by the runtime's
        // message -- since ovstage is attached; an empty match is still a valid, nonzero query.
        ovstage_query_result_t result{};
        const uint64_t resource = queryContacts(instance.attachHandle, runtime.data(), runtime.size(),
                                                 filters_per_sensor, out_summary ? &result : nullptr);
        if (!resource)
            return outputFailure(admission, error, "ovphysx_query_contacts: failed to open a contact query");
        publishOutputHandle(instance, &InstanceData::output_queries, std::move(ownership), resource);
        *out_query = resource;
        if (out_summary)
            *out_summary = result;
        return admission.complete();
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_update_articulations_kinematic(
    ovphysx_instance_handle_t handle, ovphysx_query_handle_t query, uint32_t flags)
{
    OVPHYSX_API_PROFILE_ZONE(update_articulations_kinematic);
    constexpr uint32_t kSupportedFlags = OVPHYSX_ARTICULATION_KINEMATIC_POSITION |
                                         OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY;
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        if (flags & ~kSupportedFlags)
            return set_enqueue_error(
                OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_update_articulations_kinematic: unsupported flags");
        const ovphysx_api_status_t owner = outputOwner(instance, query, &InstanceData::output_queries);
        if (owner != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(
                owner, "ovphysx_update_articulations_kinematic: query is not owned by this instance");
        if (!omni::physx::ovxUpdateArticulationsKinematic(query, 0))
            return set_enqueue_error(
                OVPHYSX_API_INVALID_ARGUMENT,
                "ovphysx_update_articulations_kinematic: query must select live whole articulations");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(
                drain, "ovphysx_update_articulations_kinematic: failed to finish pending simulation");

        admission.create_task(handle);
        omni::physx::RuntimeErrorScope error;
        if (!flags)
            return admission.complete();
        if (ovphysx_ensure_physics_attached(handle) != OVPHYSX_API_SUCCESS)
            return outputFailure(
                admission, error, "ovphysx_update_articulations_kinematic: failed to attach physics stage");
        const ovphysx_result_t warmup = ovphysx_warmup_if_needed(handle, false);
        if (warmup.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t cause = ovphysx_get_last_error();
            return admission.fail(cause.ptr ? cause.ptr : "Articulation update warmup failed");
        }
        if (!omni::physx::ovxUpdateArticulationsKinematic(query, flags) || error.message()[0])
            return outputFailure(admission, error, "Failed to update articulation kinematics");
        return admission.complete();
    });
}

// The form ovphysx_evaluate_sdf_shapes needs of its tensors, checked before a task exists: float32 [N, Q, 3]
// points and [N, Q, 4] out, C-contiguous on one CUDA device. Empty when they have it, else the reason. Whether N
// is the query's row count is the task's check, since only the task knows the rows.
static std::string sdfTensorsError(const DLTensor* points, const DLTensor* out)
{
    const DLTensor* const tensors[2] = { points, out };
    const char* const names[2] = { "points", "out" };
    const int64_t lanes[2] = { 3, 4 };
    omni::physics::tensors::TensorDesc descs[2];
    for (int i = 0; i < 2; ++i)
    {
        const ovphysx::internal::DLConvertError converted = ovphysx::internal::dlToTensorDesc(tensors[i], descs[i]);
        if (converted != ovphysx::internal::DLConvertError::Success)
            return std::string(names[i]) + ": " + ovphysx::internal::dlConvertErrorMessage(converted);
        const omni::physics::tensors::TensorDesc& d = descs[i];
        if (d.numDims != 3 || d.dims[2] != lanes[i])
            return std::string(names[i]) + " must have shape [N, Q, " + std::to_string(lanes[i]) + "]";
        if (d.dtype != omni::physics::tensors::TensorDataType::eFloat32)
            return std::string(names[i]) + " must be float32";
        if (d.device < 0)
            return std::string(names[i]) + " must be on a CUDA device";
    }
    if (descs[0].dims[0] != descs[1].dims[0] || descs[0].dims[1] != descs[1].dims[1])
        return "points [N, Q, 3] and out [N, Q, 4] must agree on N and Q";
    if (descs[0].dims[1] < 1)
        return "Q must be at least 1";
    if (descs[0].dims[0] > static_cast<int>(omni::physx::kOvxMaxSdfShapesPerEvaluation))
        return "N exceeds 65535: one evaluation addresses at most 65535 SDF shapes (a PhysX launch limit)";
    if (static_cast<int64_t>(descs[0].dims[0]) * descs[0].dims[1] >
        static_cast<int64_t>(omni::physx::kOvxMaxSdfPointsPerEvaluation))
        return "N x Q exceeds 2147483647: one evaluation addresses at most 2^31 - 1 points (a PhysX limit)";
    if (descs[0].device != descs[1].device)
        return "points and out must be on the same CUDA device";
    return std::string();
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_evaluate_sdf_shapes(ovphysx_instance_handle_t handle,
                                                                 ovphysx_query_handle_t query,
                                                                 const DLTensor* points,
                                                                 DLTensor* out,
                                                                 ovstage_cuda_sync_t points_ready)
{
    OVPHYSX_API_PROFILE_ZONE(evaluate_sdf_shapes);
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        const std::string tensors = sdfTensorsError(points, out);
        if (!tensors.empty())
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                     ("ovphysx_evaluate_sdf_shapes: " + tensors).c_str());
        const ovphysx_api_status_t owner = outputOwner(instance, query, &InstanceData::output_queries);
        if (owner != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(owner, "ovphysx_evaluate_sdf_shapes: query is not owned by this instance");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(drain, "ovphysx_evaluate_sdf_shapes: failed to finish pending simulation");

        admission.create_task(handle);
        omni::physx::RuntimeErrorScope error;
        if (ovphysx_ensure_physics_attached(handle) != OVPHYSX_API_SUCCESS)
            return outputFailure(admission, error, "ovphysx_evaluate_sdf_shapes: failed to attach physics stage");
        // A call that will be refused -- a stale, wrong-type or empty query, mismatched rows, a CPU scene -- is refused
        // before the warmup, which steps the simulation, as the SDF view refused a stale view before its own.
        if (!omni::physx::ovxCheckSdfShapes(query, points, out) || error.message()[0])
            return outputFailure(admission, error, "SDF shape evaluation failed");
        const ovphysx_result_t warmup = ovphysx_warmup_if_needed(handle, false);
        if (warmup.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t cause = ovphysx_get_last_error();
            return admission.fail(cause.ptr ? cause.ptr : "SDF evaluation warmup failed");
        }
        if (!omni::physx::ovxEvaluateSdfShapes(query, points, out, points_ready) || error.message()[0])
            return outputFailure(admission, error, "SDF shape evaluation failed");
        return admission.complete();
    });
}

// Private Python binding ABI: cache checks finish pending execution without collecting task outcomes.
// Ordinary queries retain their original attachment; debug-render queries follow the current attachment.
extern "C" OVPHYSX_API ovphysx_result_t ovphysx_internal_is_query_valid(ovphysx_instance_handle_t handle,
                                                                    ovphysx_query_handle_t query,
                                                                    bool* out_valid)
{
    if (!out_valid)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "internal_is_query_valid: out_valid is NULL");
    *out_valid = false;

    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "internal_is_query_valid: invalid handle");
    if (outputOwner(*instance, query, &InstanceData::output_queries) != OVPHYSX_API_SUCCESS)
        return set_error(OVPHYSX_API_NOT_FOUND, "internal_is_query_valid: query is not live on this instance");

    if (ovphysx::isInLogCallback() || omni::physx::runtime::isRuntimeBusy(instance->attachHandle, true))
        return set_error(OVPHYSX_API_INVALID_STATE, "Output metadata cannot reenter runtime execution");
    const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
    if (drain != OVPHYSX_API_SUCCESS)
        return set_error(drain, "internal_is_query_valid: failed to finish pending simulation");

    *out_valid = omni::physx::ovxQueryDictionary(query) != nullptr;
    return success();
}

// Discovery is the optional summary returned by ovphysx_query. Simulated-type
// queries require an attached stage. A debug-render query can exist with no stage.
OVPHYSX_API ovphysx_enqueue_result_t ovphysx_read(ovphysx_instance_handle_t handle,
    ovphysx_query_handle_t query, const ovx_string_or_token_t* attributes,
    size_t attribute_count, ovphysx_read_handle_t* out_read, ovphysx_read_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(read);
    if (out_read)
        *out_read = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        if (!out_read || (attribute_count && !attributes))
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_read: invalid output or attribute array");
        for (size_t i = 0; i < attribute_count; ++i)
            if (attributes[i].token == OVX_INVALID_TOKEN && !attributes[i].string.ptr && attributes[i].string.length)
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_read: NULL attribute string has nonzero length");
        const ovphysx_api_status_t owner = outputOwner(instance, query, &InstanceData::output_queries);
        if (owner != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(owner, "ovphysx_read: query is not owned by this instance");
        const OvphysxSidecarReadOutputsFn read = g_sidecarReadOutputs.load(std::memory_order_acquire);
        if (!read)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_read: read API is unavailable");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(drain, "ovphysx_read: failed to finish pending simulation");
        admission.create_task(handle);
        OutputHandles::node_type ownership = reserveOutputHandle(instance, &InstanceData::output_reads);
        omni::physx::RuntimeErrorScope error;
        ovphysx_read_result_t result{};
        const uint64_t resource = read(query, attributes, attribute_count, out_summary ? &result : nullptr);
        if (!resource)
            return outputFailure(admission, error, "ovphysx_read: failed to open a read session");
        publishOutputHandle(instance, &InstanceData::output_reads, std::move(ownership), resource);
        *out_read = resource;
        if (out_summary)
            *out_summary = result;
        return admission.complete();
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_fetch_read_next(ovphysx_instance_handle_t handle,
    ovphysx_read_handle_t read, const ovstage_read_group_t** out_group)
{
    OVPHYSX_API_PROFILE_ZONE(fetch_read_next);
    if (out_group)
        *out_group = nullptr;
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        if (!out_group)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_read_next: output is NULL");
        const ovphysx_api_status_t owner = outputOwner(instance, read, &InstanceData::output_reads);
        if (owner != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(owner, "ovphysx_fetch_read_next: read is not owned by this instance");
        const OvphysxSidecarFetchReadNextFn fetch = g_sidecarFetchReadNext.load(std::memory_order_acquire);
        if (!fetch)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_fetch_read_next: read API is unavailable");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(drain, "ovphysx_fetch_read_next: failed to finish pending simulation");
        admission.create_task(handle);
        omni::physx::RuntimeErrorScope error;
        if (fetch(read, out_group) < 0)
            return outputFailure(admission, error, "ovphysx_fetch_read_next: read failed");
        return admission.complete();
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_group(ovphysx_instance_handle_t handle,
    ovphysx_read_handle_t read, ovstage_read_group_id_t group_id)
{
    OVPHYSX_API_PROFILE_ZONE(release_group);
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        const ovphysx_api_status_t owner = outputOwner(instance, read, &InstanceData::output_reads, true);
        if (owner != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(owner, "ovphysx_release_group: read belongs to another instance");
        bool live = false;
        {
            std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
            const auto found = instance.output_reads.find(read);
            live = found != instance.output_reads.end() && !found->second;
        }
        if (!live)
        {
            admission.create_task(handle);
            return admission.complete();
        }
        const OvphysxSidecarReleaseGroupFn release = g_sidecarReleaseGroup.load(std::memory_order_acquire);
        if (!release)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_release_group: release API is unavailable");
        admission.create_task(handle);
        omni::physx::RuntimeErrorScope error;
        if (release(read, group_id) <= 0 || error.message()[0])
            return outputFailure(admission, error, "ovphysx_release_group: release failed");
        return admission.complete();
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_read(ovphysx_instance_handle_t handle, ovphysx_read_handle_t read)
{
    OVPHYSX_API_PROFILE_ZONE(release_read);
    return releaseOutput(handle, read, &InstanceData::output_reads, releaseDebugOrRead);
}

// --- App -> physics write (ADR-0012) ----------------------------------------
// The return direction of ovphysx_read above, and routed the same way: through the internal
// sidecar to the runtime's ovx* write entry points. Argument and handle validation lives HERE
// rather than in a frontend because REQ-CAPI-WRITE-001 AC-8 makes it C-first. Every frontend
// inherits these checks, and no frontend may add one the C API does not make.

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_write(ovphysx_instance_handle_t handle,
    ovphysx_query_handle_t query, const ovx_string_or_token_t* attributes, size_t attribute_count,
    ovphysx_write_handle_t* out_write, ovphysx_write_result_t* out_summary)
{
    OVPHYSX_API_PROFILE_ZONE(write);
    if (out_write)
        *out_write = 0;
    if (out_summary)
        *out_summary = {};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        if (!out_write || !attributes || !attribute_count)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_write: output and nonempty attributes are required");
        for (size_t i = 0; i < attribute_count; ++i)
            if (attributes[i].token == OVX_INVALID_TOKEN && !attributes[i].string.ptr && attributes[i].string.length)
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_write: NULL attribute string has nonzero length");
        const ovphysx_api_status_t owner = outputOwner(instance, query, &InstanceData::output_queries);
        if (owner != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(owner, "ovphysx_write: query is not owned by this instance");
        const OvphysxSidecarWriteAttributeFn write = g_sidecarWriteAttribute.load(std::memory_order_acquire);
        if (!write)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_write: write API is unavailable");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(drain, "ovphysx_write: failed to finish pending simulation");
        admission.create_task(handle);
        OutputHandles::node_type ownership = reserveOutputHandle(instance, &InstanceData::output_writes);
        omni::physx::RuntimeErrorScope error;
        ovphysx_write_result_t result{};
        const uint64_t resource = write(query, attributes, attribute_count, out_summary ? &result : nullptr);
        if (!resource)
            return outputFailure(admission, error, "ovphysx_write: failed to open a write session");
        publishOutputHandle(instance, &InstanceData::output_writes, std::move(ownership), resource);
        *out_write = resource;
        if (out_summary)
            *out_summary = result;
        return admission.complete();
    });
}

/**
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-2
 *
 * The (object type, attribute) writability classification. `ovphysx_writability` and its backing
 * table `kWritabilityTable` are a C-layer SNAPSHOT of the runtime write/read tables, kept here so
 * the query is bare-process (no instance, scene or step).
 */
namespace
{
// A self-contained (object type, attribute) -> writability snapshot of the runtime write and read
// attribute tables (OvxPhysicsWrite.cpp / OvxPhysicsRead.cpp): writable and readable is WRITABLE, a
// control input with no read-back is WRITE_ONLY, readable but not writable is READ_ONLY. It is kept
// in the C layer rather than derived from the runtime so the query needs no instance, scene or
// step, a deliberate second source of truth.
//
// This copy is hand-maintained. test_writability_parity.cpp compares its answers with actual
// read/write session admission over an empty Stage for every declared attribute and object type.
// Conditional scene/state requirements have separate live-object coverage. An attribute absent
// for its object type is UNCLASSIFIED, which ovphysx_write rejects.
struct WritabilityRow
{
    ovphysx_query_target_t object_type;
    const char* attribute;
    ovphysx_writability_t writability;
};

const WritabilityRow kWritabilityTable[] = {
    // Rigid body (standalone + point-instancer instances)
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_ORIENTATION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_ANGULAR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_ACCELERATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_ANGULAR_ACCELERATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_MASS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_INVERSE_MASS, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_INERTIA, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_INVERSE_INERTIA, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_CENTER_OF_MASS_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_CENTER_OF_MASS_ORIENTATION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, OVPHYSX_WRITABILITY_WRITABLE },
    // Conditional: a kinematic target drives the body on a CPU scene, but is refused on a DirectGPU
    // scene -- PxRigidDynamic::setKinematicTarget is a CPU-only integration with no DirectGPU write
    // counterpart, so under eENABLE_DIRECT_GPU_API the CPU-computed pose never reaches the device and
    // the body would not move (ovxWriteAttribute refuses it there, OvxPhysicsWrite planRigidGroup).
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_KINEMATIC_TARGET_POSITION, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_KINEMATIC_TARGET_ORIENTATION, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_STATIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_RESTITUTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_CONTACT_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_REST_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_SHAPE_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_IS_SLEEPING, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_FORCE, OVPHYSX_WRITABILITY_WRITE_ONLY },
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WRENCH, OVPHYSX_WRITABILITY_WRITE_ONLY },
    // wake is a command; isSleeping separately reports current state. The command is CONDITIONAL:
    // CPU scenes and GPU scenes with CPU readback support it; DirectGPU scenes refuse it.
    { OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WAKE, OVPHYSX_WRITABILITY_CONDITIONAL },

    // Articulation link (a rigid body PLUS the inbound-joint force; pose/velocity are DERIVED)
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_POSITION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_ORIENTATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_LINEAR_VELOCITY, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_ANGULAR_VELOCITY, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_LINEAR_ACCELERATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_ANGULAR_ACCELERATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_MASS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_INVERSE_MASS, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_INERTIA, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_INVERSE_INERTIA, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_CENTER_OF_MASS_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_CENTER_OF_MASS_ORIENTATION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_SIMULATION, OVPHYSX_WRITABILITY_READ_ONLY },
    // A link shares the rigid read table, so it READS the kinematic target via the undriven fallback
    // (its pose -- last-synced, so stale under DirectGPU suppressed readback), but a link has no
    // kinematic target to drive, so it is refused for write.
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_KINEMATIC_TARGET_POSITION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_KINEMATIC_TARGET_ORIENTATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_STATIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_RESTITUTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_CONTACT_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_REST_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_SHAPE_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_IS_SLEEPING, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_LINK_INCOMING_JOINT_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_FORCE, OVPHYSX_WRITABILITY_WRITE_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_WRENCH, OVPHYSX_WRITABILITY_WRITE_ONLY },
    // No wake row for a link, deliberately: wake is write-only (no read column) AND refused by name on a
    // link (a link cannot wake independently -- the whole articulation does), so it is usable in neither
    // direction. Every explicit class would overclaim one -- READ_ONLY a read-back it lacks, WRITE_ONLY a
    // write a link refuses -- so it stays UNCLASSIFIED, the table's value for a real name a type does not
    // serve (as jointPosition is on RIGID_BODY). Pinned by test_writability's ArticulationLink case.

    // Articulation joint (per-axis DOF state + drive properties)
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION_TARGET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY_TARGET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_ACTUATION_FORCE, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },
    // Conditional: an axis only HAS a limit interval when its motion is eLIMITED. PhysX refuses
    // setMotion() on an in-scene articulation, so a finite limit aimed at a free axis cannot land at
    // all. The write fails naming the condition (REQ-INPUT-COVERAGE-001 AC-10). Writing the read's
    // +/-FLT_MAX unlimited sentinel back to a free axis stays a legal no-op.
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_LIMIT, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_MAX_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_MAX_FORCE, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_ARMATURE, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_STATIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VISCOUS_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_SPEED_EFFORT_GRADIENT, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_MAX_ACTUATOR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY_DEPENDENT_RESISTANCE, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DRIVE_TYPE, OVPHYSX_WRITABILITY_WRITABLE },

    // Whole-vehicle controls are conditional on drive type or chassis eligibility.
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_ACCELERATOR, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_BRAKE0, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_BRAKE1, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_STEER, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_REQUESTED_GEAR, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_THRUST0, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_THRUST1, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_RESET_TO_REST_STATE, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_CLUTCH, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_CURRENT_GEAR, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_TARGET_GEAR, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_ENGINE_ROTATION_SPEED, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_AUTOMATIC_TRANSMISSION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_GEAR_SWITCH_TIME, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE, OVPHYSX_ATTR_AUTOBOX_TIME_SINCE_LAST_SHIFT, OVPHYSX_WRITABILITY_READ_ONLY },

    // Vehicle wheel (pose is derived from chassis + suspension + steer; only controls are writable)
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_POSITION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_ORIENTATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_DRIVE_TORQUE, OVPHYSX_WRITABILITY_WRITE_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_BRAKE_TORQUE, OVPHYSX_WRITABILITY_WRITE_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_STEER_ANGLE, OVPHYSX_WRITABILITY_WRITE_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_ROTATION_SPEED, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_ROTATION_ANGLE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_WHEEL_STEER_ANGLE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_SUSPENSION_JOUNCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_SUSPENSION_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_FRICTION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_LONGITUDINAL_DIRECTION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_LATERAL_DIRECTION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_LONGITUDINAL_SLIP, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_LATERAL_SLIP, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_TIRE_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_IS_ON_GROUND, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_PLANE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_HIT_POSITION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_ACTOR_PATH, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_SHAPE_PATH, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, OVPHYSX_ATTR_GROUND_MATERIAL_PATH, OVPHYSX_WRITABILITY_READ_ONLY },

    // Deformable volume (sim-mesh points / velocities). kinematicTarget is CONDITIONAL. Unlike the rigid
    // one it carries no DEVICE-mode restriction -- it drives through setKinematicTargetBufferD, a device
    // buffer that survives the DirectGPU path, and the read hands back the same targets. But it has
    // OBJECT-STATE preconditions the write reports as unmet: it is refused on a volume already kinematic
    // through another path (PxDeformableBodyFlag::eKINEMATIC, i.e. kinematicBody=true, or
    // PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC raised by a tensor-binding view), and a write that would
    // move an already-driven node back to free is refused (driving is monotonic -- no un-pinning).
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_POINTS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_VELOCITIES, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_REST_POINTS, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_SIM_ELEMENT_INDICES, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_COLLISION_ELEMENT_INDICES, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_KINEMATIC_TARGET, OVPHYSX_WRITABILITY_CONDITIONAL },

    // Deformable surface
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE, OVPHYSX_ATTR_POINTS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE, OVPHYSX_ATTR_VELOCITIES, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE, OVPHYSX_ATTR_REST_POINTS, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE, OVPHYSX_ATTR_SIM_ELEMENT_INDICES, OVPHYSX_WRITABILITY_READ_ONLY },

    // Particle set
    { OVPHYSX_QUERY_TARGET_PARTICLE_SET, OVPHYSX_ATTR_POINTS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_PARTICLE_SET, OVPHYSX_ATTR_VELOCITIES, OVPHYSX_WRITABILITY_WRITABLE },

    // Fixed tendon
    { OVPHYSX_QUERY_TARGET_FIXED_TENDON, OVPHYSX_ATTR_TENDON_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_FIXED_TENDON, OVPHYSX_ATTR_TENDON_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_FIXED_TENDON, OVPHYSX_ATTR_TENDON_LIMIT_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_FIXED_TENDON, OVPHYSX_ATTR_TENDON_LIMIT, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_FIXED_TENDON, OVPHYSX_ATTR_TENDON_REST_LENGTH, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_FIXED_TENDON, OVPHYSX_ATTR_TENDON_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },

    // Spatial tendon (limit / rest length live on the leaf attachment, so they are not writable here)
    { OVPHYSX_QUERY_TARGET_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_LIMIT_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },

    // Whole articulation (root-link state + inverse dynamics queries)
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_ORIENTATION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_ANGULAR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_CENTER_OF_MASS_WORLD, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_CENTER_OF_MASS_LOCAL, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_JACOBIAN, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_JACOBIAN_SHAPE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_MASS_MATRIX, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_CORIOLIS_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_GRAVITY_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_CENTROIDAL_MOMENTUM, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_STATIC_FRICTION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_RESTITUTION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_CONTACT_OFFSET, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_REST_OFFSET, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_SHAPE_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    // Whole-articulation topology metadata (read-only), replacing ovphysx_articulation_metadata_t.
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_DOF_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_LINK_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_JOINT_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_FIXED_TENDON_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_SPATIAL_TENDON_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_IS_FIXED_BASE, OVPHYSX_WRITABILITY_READ_ONLY },
    // DOF-column descriptor (read-only): maps each inverse-dynamics matrix column to (joint prim, axis).
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_DOF_JOINT_PATH, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_DOF_AXIS_INDEX, OVPHYSX_WRITABILITY_READ_ONLY },

    // Deformable material
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_YOUNGS_MODULUS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_POISSONS_RATIO, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_ELASTICITY_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_BENDING_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_THICKNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_BENDING_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },

    // Contact sensor. Every contact column -- the net forces, raw contacts and events and, on a contact query,
    // the force matrices and the filtered detail -- is a per-step simulation output, so writing one is an error
    // naming the type rather than a silent no-op.
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_NET_NORMAL_CONTACT_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_NET_FRICTION_CONTACT_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_NORMAL_CONTACT_FORCE_MATRIX, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FRICTION_CONTACT_FORCE_MATRIX, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_POINT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_NORMAL, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_SEPARATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_CONTACT_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_FRICTION_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_FRICTION_POINT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_FILTERED_FRICTION_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_POINT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_NORMAL, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_SEPARATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_RAW_CONTACT_OTHER_BODY, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_TYPE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_SENSOR_SIDE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_COLLIDER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_OTHER_BODY, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_OTHER_COLLIDER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_PROTO_INDEX, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_OTHER_PROTO_INDEX, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_ANCHOR_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_POSITION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_NORMAL, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_IMPULSE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_SEPARATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_FACE_INDEX, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_OTHER_FACE_INDEX, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_MATERIAL, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_POINT_OTHER_MATERIAL, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_ANCHOR_POSITION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_ATTR_CONTACT_EVENT_ANCHOR_IMPULSE, OVPHYSX_WRITABILITY_READ_ONLY },

    // Character controller. move is CONDITIONAL: refused on a DirectGPU scene, where the CPU
    // kinematic target PxController::move sets is not uploaded. Pose is the rigid-body read.
    { OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER, OVPHYSX_ATTR_CCT_MOVE, OVPHYSX_WRITABILITY_CONDITIONAL },

    // Borrowed native addresses. Readable only; the column does not accept a write.
    { OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_PARTICLE_SET, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_SCENE, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_PHYSX_MATERIAL, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_SHAPE, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_COMPOUND_SHAPE, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_RIGID_ACTOR, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_JOINT, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_CUSTOM_JOINT, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_PARTICLE_SYSTEM, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_QUERY_TARGET_PHYSICS, OVPHYSX_ATTR_PHYSX_POINTER, OVPHYSX_WRITABILITY_READ_ONLY },
};

// True iff the NUL-terminated table literal equals name[0..len). Walking the literal and stopping
// at its own NUL never reads past a literal shorter than len, which std::memcmp(row.attribute,
// name, len) would. The trailing check pins its length to len.
bool attributeNameEquals(const char* attribute, const char* name, size_t len)
{
    for (size_t i = 0; i < len; ++i)
        if (attribute[i] == '\0' || attribute[i] != name[i])
            return false;
    return attribute[len] == '\0';
}

ovphysx_writability_t classifyWritability(ovphysx_query_target_t type, const char* name, size_t len)
{
    if (type == OVPHYSX_QUERY_TARGET_DEBUG_RENDER)
        return static_cast<ovphysx_writability_t>(omni::physx::ovxDebugRenderWritability(name, len));
    for (const WritabilityRow& row : kWritabilityTable)
        if (row.object_type == type && attributeNameEquals(row.attribute, name, len))
            return row.writability;
    return OVPHYSX_WRITABILITY_UNCLASSIFIED;
}
} // namespace

OVPHYSX_API ovphysx_result_t ovphysx_writability(ovphysx_query_target_t object_type,
                                                 const ovx_string_or_token_t* attribute,
                                                 ovphysx_writability_t* out_writability)
{
    OVPHYSX_API_PROFILE_ZONE(writability);
    if (!out_writability)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_writability: out_writability is NULL");
    *out_writability = OVPHYSX_WRITABILITY_UNCLASSIFIED;
    if (!attribute)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_writability: attribute is NULL");

    // String-only: writability classifies an attribute by NAME (OVPHYSX_ATTR_*). Resolving an
    // interned token would need a path dictionary, which this scene-free query does not have, so a
    // token-only key is rejected outright rather than silently misclassified as UNCLASSIFIED.
    if (!attribute->string.ptr || attribute->string.length == 0)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "ovphysx_writability: attribute must be a string name (OVPHYSX_ATTR_*); "
                         "interned tokens are not supported");

    // Reject an object_type outside ovphysx_query_target_t. The enum
    // is contiguous [RIGID_BODY, last), and the uint32_t cast folds negatives into the high range, so
    // this one bound against the OVPHYSX_QUERY_TARGET_COUNT sentinel covers them and tracks the enum
    // automatically -- a new type extends the valid range without editing this line. Without it an
    // unknown type falls through classifyWritability and returns SUCCESS/UNCLASSIFIED, indistinguishable
    // to a caller from a valid type that simply does not accept the name.
    if (static_cast<uint32_t>(object_type) >= OVPHYSX_QUERY_TARGET_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_writability: unknown object type");

    *out_writability = classifyWritability(object_type, attribute->string.ptr, attribute->string.length);
    return success();
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_fetch_write_next(ovphysx_instance_handle_t handle,
    ovphysx_write_handle_t write, ovphysx_fetched_write_group_t* out_group)
{
    OVPHYSX_API_PROFILE_ZONE(fetch_write_next);
    if (out_group)
        *out_group = {nullptr, SIZE_MAX};
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        if (!out_group)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_write_next: output is NULL");
        const ovphysx_api_status_t owner = outputOwner(instance, write, &InstanceData::output_writes);
        if (owner != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(owner, "ovphysx_fetch_write_next: write is not owned by this instance");
        const OvphysxSidecarFetchWriteNextFn fetch = g_sidecarFetchWriteNext.load(std::memory_order_acquire);
        if (!fetch)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_fetch_write_next: write API is unavailable");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(drain, "ovphysx_fetch_write_next: failed to finish pending simulation");
        admission.create_task(handle);
        omni::physx::RuntimeErrorScope error;
        if (fetch(write, out_group) < 0)
            return outputFailure(admission, error, "ovphysx_fetch_write_next: fetch failed");
        return admission.complete();
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_commit_group(ovphysx_instance_handle_t handle,
    ovphysx_write_handle_t write, const ovstage_map_group_t* group, ovstage_cuda_sync_t write_done_sync)
{
    OVPHYSX_API_PROFILE_ZONE(commit_group);
    return outputTask(handle, [&](InstanceData& instance, ovphysx::TaskContext& admission)
    {
        if (!group)
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_commit_group: group is NULL");
        const ovphysx_api_status_t owner = outputOwner(instance, write, &InstanceData::output_writes);
        if (owner != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(owner, "ovphysx_commit_group: write is not owned by this instance");
        const OvphysxSidecarCommitGroupFn commit = g_sidecarCommitGroup.load(std::memory_order_acquire);
        if (!commit)
            return set_enqueue_error(OVPHYSX_API_ERROR, "ovphysx_commit_group: write API is unavailable");
        const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(handle);
        if (drain != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(drain, "ovphysx_commit_group: failed to finish pending simulation");
        admission.create_task(handle);
        int32_t why = kOvphysxCommitFailureNone;
        omni::physx::RuntimeErrorScope error;
        if (!commit(write, group, write_done_sync, &why))
        {
            const char* context = "ovphysx_commit_group: outcome is unknown; do not retry the commit";
            if (why == kOvphysxCommitFailureNotLive)
                context = "ovphysx_commit_group: group is not live; nothing was published";
            else if (why == kOvphysxCommitFailurePublish)
                context = "ovphysx_commit_group: group is spent; commit is not retryable";
            if (error.message()[0])
                return admission.fail((std::string(context) + ": " + error.message()).c_str());
            return admission.fail(context);
        }
        if (error.message()[0])
            return admission.fail(error.message());
        return admission.complete();
    });
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_write(ovphysx_instance_handle_t handle, ovphysx_write_handle_t write)
{
    OVPHYSX_API_PROFILE_ZONE(release_write);
    return releaseOutput(handle, write, &InstanceData::output_writes, releaseDebugOrWrite);
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_release_query(ovphysx_instance_handle_t handle, ovphysx_query_handle_t query)
{
    OVPHYSX_API_PROFILE_ZONE(release_query);
    return releaseOutput(handle, query, &InstanceData::output_queries, releaseDebugOrQuery);
}

// Private Python binding ABI: check CUDA wait failures before exposing borrowed arrays.
extern "C" OVPHYSX_API ovphysx_result_t ovphysx_internal_cuda_stream_wait_event(uintptr_t stream, uintptr_t event)
{
    // event == 0 means "nothing to wait for". It is answered before the shim is touched so a
    // CPU-only process never reaches CUDA. Stream 0 is not a sentinel, it is the valid NULL stream.
    if (event == 0)
        return success();

    omni::physx::IOptionalCuda* cuda = omni::physx::runtime::tryGetOptionalCudaInterface();
    if (!cuda || !cuda->streamWaitEvent)
        return set_error(OVPHYSX_API_ERROR,
                         "ovphysx_internal_cuda_stream_wait_event: CUDA is not available in this process");

    // No context push: the driver call must run in the CALLER's current context. Stream sentinels 1
    // and 2 are context-relative (the legacy / per-thread default stream of whatever context is
    // current), so making the ovphysx context current would order a stream the caller never used,
    // yet still succeed.

    // Enqueue-only: the driver records the dependency on `stream` and returns, with no host sync.
    int status = 0;
    if (!cuda->streamWaitEvent(stream, event, /*flags=*/0u, &status))
        return set_error(OVPHYSX_API_ERROR,
                         "ovphysx_internal_cuda_stream_wait_event: cuStreamWaitEvent failed with CUDA status " +
                             std::to_string(status));
    return success();
}
