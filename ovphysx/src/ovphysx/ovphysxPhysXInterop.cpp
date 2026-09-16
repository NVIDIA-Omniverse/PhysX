// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-PHYSXPTR-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

/**
 * @implements REQ-CAPI-CUDA-002
 * @covers AC-1 AC-2 AC-3
 *
 * `ovphysx_cuda_stream_wait_event` is the C bridge the requirement is built on: AC-1 enqueues the
 * wait on the caller's stream, AC-2 short-circuits `event == 0` before CUDA is touched, and AC-3
 * reports a CUDA-less process through `ovphysx_get_last_error`. The Python output-read contract
 * layered on this call is REQ-PYTHON-READ-001 AC-4, annotated in `ovphysx/python/ovphysx/api.py`.
 */

/**
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-1 AC-1a AC-2 AC-3 AC-4 AC-5 AC-5a AC-6 AC-7 AC-8 AC-10
 *
 * The public C write surface: ovphysx_write / ovphysx_fetch_write_next / ovphysx_commit_group /
 * ovphysx_release_write. Validation is C-first (AC-8), so every argument, handle and lifecycle
 * check lives here rather than in a frontend. The Python bindings are a thin mirror and inherit
 * these guarantees instead of restating them.
 */

// PhysX object interop: exposes raw PhysX pointers by selector + type enum.
//
// The selector-to-ObjectKey/PhysX lookup runs in the internal sidecar (which includes the PhysX SDK headers).
// The sidecar loader resolves the function pointer at instance creation and
// publishes it via g_sidecarGetPhysXPtr, which this file reads.

#include "ovphysx/ovphysx.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sidecar/ovphysxInternalInterop.h"  // g_sidecarGetPhysXPtr

#include <carb/Framework.h>
#include <omni/physx/IOptionalCuda.h>
#include <omni/physx/PhysXRuntime.h>

#include <cmath>   // std::isfinite for debug-render arg validation
#include <string>

// Sidecar get-physx-ptr atomic, owned here next to its consumer. The loader writes
// it during loadInternalSidecar() via the extern in ovphysxInternalInterop.h.
std::atomic<OvphysxSidecarGetPhysXPtrFn> g_sidecarGetPhysXPtr{nullptr};
std::atomic<OvphysxSidecarUpdateKinematicFn> g_sidecarUpdateKinematic{nullptr};

// Physics output read (ADR-0007) sidecar pointers, owned here next to their
// public-API consumers below. The loader writes them during loadInternalSidecar().
std::atomic<OvphysxSidecarOutputQueryFn>      g_sidecarOutputQuery{nullptr};
std::atomic<OvphysxSidecarFetchQueryResultFn> g_sidecarFetchQueryResult{nullptr};
std::atomic<OvphysxSidecarQueryDictionaryFn>  g_sidecarQueryDictionary{nullptr};
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

namespace {

enum class LookupStatus { kOk, kPluginUnavailable, kNotFound };

struct LookupResult
{
    void* ptr;
    LookupStatus status;
};

LookupResult lookupPhysXPtr(const char* prim_path, int type)
{
    OvphysxSidecarGetPhysXPtrFn fn = g_sidecarGetPhysXPtr.load(std::memory_order_acquire);
    if (!fn)
        return {nullptr, LookupStatus::kPluginUnavailable};
    void* ptr = fn(prim_path, type);
    return {ptr, ptr ? LookupStatus::kOk : LookupStatus::kNotFound};
}

static ovphysx_result_t validateInteropArgs(const ovphysx_string_t& prim_path,
                                           ovphysx_physx_type_t physx_type,
                                           void** out_ptr)
{
    if (!out_ptr)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_ptr is NULL");
    *out_ptr = nullptr;

    if (!prim_path.ptr && prim_path.length > 0)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "prim_path is NULL with non-zero length");
    if (hasEmbeddedNul(prim_path))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "prim_path contains an embedded NUL byte");

    if (physx_type == OVPHYSX_PHYSX_TYPE_PHYSICS)
    {
        if (prim_path.length != 0)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "prim_path must be empty for OVPHYSX_PHYSX_TYPE_PHYSICS");
        return success();
    }

    if (!isValid(prim_path))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "prim_path is NULL or empty");

    return success();
}

} // anonymous namespace


// ---- Unified API ----

OVPHYSX_API ovphysx_result_t ovphysx_get_physx_ptr(
    ovphysx_handle_t handle,
    ovphysx_string_t prim_path,
    ovphysx_physx_type_t physx_type,
    void** out_ptr)
{
    ovphysx_result_t check = validateInteropArgs(prim_path, physx_type, out_ptr);
    if (check.status != OVPHYSX_API_SUCCESS)
        return check;

    const std::string prim_path_str = toStdString(prim_path);

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || !instance->ovstage_attached)
        return set_error(OVPHYSX_API_ERROR, "no physics stage attached");

    LookupResult lookup = lookupPhysXPtr(prim_path_str.c_str(), static_cast<int>(physx_type));
    switch (lookup.status)
    {
    case LookupStatus::kOk:
        *out_ptr = lookup.ptr;
        return success();

    case LookupStatus::kPluginUnavailable:
        return set_error(OVPHYSX_API_ERROR,
            "internal sidecar not loaded -- cannot resolve PhysX pointers");

    case LookupStatus::kNotFound:
        return set_error(OVPHYSX_API_NOT_FOUND,
            "no PhysX object of type " + std::to_string(static_cast<int>(physx_type))
            + " at path '" + prim_path_str + "'");
    }

    return set_error(OVPHYSX_API_ERROR,
        "unexpected lookup status: " + std::to_string(static_cast<int>(lookup.status)));
}

// ---- Physics output read (ADR-0007) ----
//
// The public surface validates the handle + that an ovstage Stage is attached,
// then forwards to the sidecar (which talks to the runtime's ovstage-native read
// symbols). The read is ovstage-only; under any other attach the query yields 0
// objects. See docs/ovstage_integration.md for the ordinal-coupling principle.

OVPHYSX_API ovphysx_result_t ovphysx_query(ovphysx_handle_t handle,
                                                  ovphysx_sim_object_type_t object_type,
                                                  ovphysx_object_scope_t scope,
                                                  ovphysx_query_handle_t* out_query)
{
    if (!out_query)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query: out_query is NULL");
    *out_query = 0;

    // The read observes the latest step's sealed output, so pending sim work
    // has to complete before querying.
    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query: invalid handle");
    if (!instance->ovstage_attached)
        return set_error(OVPHYSX_API_ERROR,
                         "ovphysx_query: the output read is ovstage-only; attach an ovstage Stage first");

    auto fn = g_sidecarOutputQuery.load(std::memory_order_acquire);
    if (!fn)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_query: internal sidecar read API not loaded");

    // An EMPTY match is still a valid, nonzero query. Discover it via
    // ovphysx_fetch_query_result (total_prim_count == 0) or an immediate
    // end-of-iteration from ovphysx_fetch_read_next. A zero handle therefore means
    // FAILURE only: an ovstage Stage is already confirmed attached above, so a
    // zero handle from the runtime indicates an internal failure to open the
    // query, not "nothing matched".
    *out_query = fn(static_cast<uint32_t>(object_type), static_cast<uint32_t>(scope));
    if (*out_query == 0)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_query: failed to open an output query");
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_fetch_query_result(ovphysx_handle_t handle,
                                                        ovphysx_query_handle_t query,
                                                        ovstage_query_result_t* out_result)
{
    if (!out_result)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_query_result: out_result is NULL");
    *out_result = ovstage_query_result_t{};

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_query_result: invalid handle");

    auto fn = g_sidecarFetchQueryResult.load(std::memory_order_acquire);
    if (!fn)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_fetch_query_result: internal sidecar read API not loaded");

    const int rc = fn(query, out_result);
    if (rc < 0)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_fetch_query_result: bad query handle or internal error");
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_query_shared_dictionary(ovphysx_handle_t handle,
                                                             ovphysx_query_handle_t query,
                                                             void** out_dictionary)
{
    if (!out_dictionary)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query_shared_dictionary: out_dictionary is NULL");
    *out_dictionary = nullptr;

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_query_shared_dictionary: invalid handle");

    auto fn = g_sidecarQueryDictionary.load(std::memory_order_acquire);
    if (!fn)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_query_shared_dictionary: internal sidecar read API not loaded");

    *out_dictionary = fn(query);
    return success();
}

// ovstage attachment is validated once, at ovphysx_query, the entry point of
// the query -> (fetch_query_result | read) -> fetch_read_next -> release sequence.
// The downstream calls operate on the handle that query produced and do not
// re-check attachment: a live handle can only exist for an attached Stage, and
// they fail safely (the runtime returns 0 / end-of-iteration / error) if the
// session was released or the Stage detached underneath them.
OVPHYSX_API ovphysx_result_t ovphysx_read(ovphysx_handle_t handle,
                                                  ovphysx_query_handle_t query,
                                                  const ovx_string_or_token_t* attributes,
                                                  size_t attribute_count,
                                                  ovphysx_read_handle_t* out_read)
{
    if (!out_read)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_read: out_read is NULL");
    *out_read = 0;
    if (attribute_count && !attributes)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "ovphysx_read: attributes is NULL with attribute_count > 0");

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_read: invalid handle");

    auto fn = g_sidecarReadOutputs.load(std::memory_order_acquire);
    if (!fn)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_read: internal sidecar read API not loaded");

    *out_read = fn(query, attributes, attribute_count);
    if (*out_read == 0)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_read: failed to open a read session");
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_fetch_read_next(ovphysx_handle_t handle,
                                                     ovphysx_read_handle_t read,
                                                     const ovstage_read_group_t** out_group)
{
    if (!out_group)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_read_next: out_group must be non-NULL");
    *out_group = nullptr;

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_read_next: invalid handle");

    auto fn = g_sidecarFetchReadNext.load(std::memory_order_acquire);
    if (!fn)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_fetch_read_next: internal sidecar read API not loaded");

    // The group is producer-owned: a borrowed const ovstage_read_group_t* valid
    // until ovphysx_release_group / ovphysx_release_read. ovstage owns the struct's
    // layout/versioning; ovphysx adds no mirror.
    const int rc = fn(read, out_group);
    if (rc < 0)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_fetch_read_next: bad read handle or internal error");
    if (rc == 0)
        return { OVPHYSX_API_END_OF_ITERATION }; // not an error: iteration exhausted
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_release_group(ovphysx_handle_t handle,
                                                   ovphysx_read_handle_t read,
                                                   ovstage_read_group_id_t group_id)
{
    (void)handle;
    auto fn = g_sidecarReleaseGroup.load(std::memory_order_acquire);
    if (fn)
        fn(read, group_id);
    return success(); // idempotent
}

OVPHYSX_API ovphysx_result_t ovphysx_release_read(ovphysx_handle_t handle, ovphysx_read_handle_t read)
{
    (void)handle;
    auto fn = g_sidecarReleaseRead.load(std::memory_order_acquire);
    if (fn)
        fn(read);
    return success(); // idempotent
}

// --- App -> physics write (ADR-0012) ----------------------------------------
// The return direction of ovphysx_read above, and routed the same way: through the internal
// sidecar to the runtime's ovx* write entry points. Argument and handle validation lives HERE
// rather than in a frontend because REQ-CAPI-WRITE-001 AC-8 makes it C-first. Every frontend
// inherits these checks, and no frontend may add one the C API does not make.

OVPHYSX_API ovphysx_result_t ovphysx_write(ovphysx_handle_t handle,
                                           ovphysx_query_handle_t query,
                                           const ovx_string_or_token_t* attribute,
                                           ovphysx_write_handle_t* out_write)
{
    if (!out_write)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_write: out_write is NULL");
    *out_write = 0;
    // One attribute per session, so this is required rather than a count that may be 0:
    // ovstage_map_group_t carries no attribute field, and a session with nothing to name
    // could not label the groups it emits.
    if (!attribute)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_write: attribute is NULL");

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_write: invalid handle");

    auto fn = g_sidecarWriteAttribute.load(std::memory_order_acquire);
    if (!fn)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_write: internal sidecar write API not loaded");

    // The runtime resolves the attribute against the query's own selector and refuses an unknown
    // or non-writable name there, where the selector actually lives. A zero handle is therefore a
    // real rejection and must surface as one: an unproduced name on the READ emits no group and
    // that is correct, but the same silence on a write would mean the caller's data went nowhere
    // while the call reported success.
    *out_write = fn(query, attribute);
    if (*out_write == 0)
        return set_error(OVPHYSX_API_ERROR,
                         "ovphysx_write: failed to open a write session -- the attribute may not be "
                         "writable for this query's object type, or the query handle may be invalid");
    return success();
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
// IMPORTANT: this copy is hand-maintained and not checked against the runtime. test_writability.cpp
// only asserts that ovphysx_writability returns these rows. When the runtime write/read tables
// change, update this table and the expected values in test_writability.cpp. The rows were produced
// by dumping the runtime's answer for every (type, attribute) pair. An attribute absent for its
// object type is UNCLASSIFIED, which ovphysx_write rejects.
struct WritabilityRow
{
    ovphysx_sim_object_type_t object_type;
    const char* attribute;
    ovphysx_writability_t writability;
};

const WritabilityRow kWritabilityTable[] = {
    // Rigid body (standalone + point-instancer instances)
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_ORIENTATION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_ANGULAR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_LINEAR_ACCELERATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_ANGULAR_ACCELERATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_MASS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_INVERSE_MASS, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_INERTIA, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_INVERSE_INERTIA, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_CENTER_OF_MASS_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_CENTER_OF_MASS_ORIENTATION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_STATIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_RESTITUTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_CONTACT_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_REST_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_SHAPE_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_FORCE, OVPHYSX_WRITABILITY_WRITE_ONLY },
    { OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_ATTR_WRENCH, OVPHYSX_WRITABILITY_WRITE_ONLY },

    // Articulation link (a rigid body PLUS the inbound-joint force; pose/velocity are DERIVED)
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_POSITION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_ORIENTATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_LINEAR_VELOCITY, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_ANGULAR_VELOCITY, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_LINEAR_ACCELERATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_ANGULAR_ACCELERATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_MASS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_INVERSE_MASS, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_INERTIA, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_INVERSE_INERTIA, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_CENTER_OF_MASS_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_CENTER_OF_MASS_ORIENTATION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_SIMULATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_STATIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_RESTITUTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_CONTACT_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_REST_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_SHAPE_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_LINK_INCOMING_JOINT_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_FORCE, OVPHYSX_WRITABILITY_WRITE_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_ATTR_WRENCH, OVPHYSX_WRITABILITY_WRITE_ONLY },

    // Articulation joint (per-axis DOF state + drive properties)
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION_TARGET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY_TARGET, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_ACTUATION_FORCE, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },
    // Conditional: an axis only HAS a limit interval when its motion is eLIMITED. PhysX refuses
    // setMotion() on an in-scene articulation, so a finite limit aimed at a free axis cannot land at
    // all. The write fails naming the condition (REQ-INPUT-COVERAGE-001 AC-10). Writing the read's
    // +/-FLT_MAX unlimited sentinel back to a free axis stays a legal no-op.
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_LIMIT, OVPHYSX_WRITABILITY_CONDITIONAL },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_MAX_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_MAX_FORCE, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_ARMATURE, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_STATIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VISCOUS_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_SPEED_EFFORT_GRADIENT, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_MAX_ACTUATOR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY_DEPENDENT_RESISTANCE, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DRIVE_TYPE, OVPHYSX_WRITABILITY_WRITABLE },

    // Vehicle wheel (pose is derived from chassis + suspension + steer; only controls are writable)
    { OVPHYSX_OBJECT_VEHICLE_WHEEL, OVPHYSX_ATTR_POSITION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_VEHICLE_WHEEL, OVPHYSX_ATTR_ORIENTATION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_VEHICLE_WHEEL, OVPHYSX_ATTR_DRIVE_TORQUE, OVPHYSX_WRITABILITY_WRITE_ONLY },
    { OVPHYSX_OBJECT_VEHICLE_WHEEL, OVPHYSX_ATTR_BRAKE_TORQUE, OVPHYSX_WRITABILITY_WRITE_ONLY },
    { OVPHYSX_OBJECT_VEHICLE_WHEEL, OVPHYSX_ATTR_STEER_ANGLE, OVPHYSX_WRITABILITY_WRITE_ONLY },

    // Deformable volume (sim-mesh points / velocities)
    { OVPHYSX_OBJECT_DEFORMABLE_VOLUME, OVPHYSX_ATTR_POINTS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_VOLUME, OVPHYSX_ATTR_VELOCITIES, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_VOLUME, OVPHYSX_ATTR_REST_POINTS, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_DEFORMABLE_VOLUME, OVPHYSX_ATTR_SIM_ELEMENT_INDICES, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_DEFORMABLE_VOLUME, OVPHYSX_ATTR_COLLISION_ELEMENT_INDICES, OVPHYSX_WRITABILITY_READ_ONLY },

    // Deformable surface
    { OVPHYSX_OBJECT_DEFORMABLE_SURFACE, OVPHYSX_ATTR_POINTS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_SURFACE, OVPHYSX_ATTR_VELOCITIES, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_SURFACE, OVPHYSX_ATTR_REST_POINTS, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_DEFORMABLE_SURFACE, OVPHYSX_ATTR_SIM_ELEMENT_INDICES, OVPHYSX_WRITABILITY_READ_ONLY },

    // Particle set
    { OVPHYSX_OBJECT_PARTICLE_SET, OVPHYSX_ATTR_POINTS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_PARTICLE_SET, OVPHYSX_ATTR_VELOCITIES, OVPHYSX_WRITABILITY_WRITABLE },

    // Fixed tendon
    { OVPHYSX_OBJECT_FIXED_TENDON, OVPHYSX_ATTR_TENDON_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_FIXED_TENDON, OVPHYSX_ATTR_TENDON_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_FIXED_TENDON, OVPHYSX_ATTR_TENDON_LIMIT_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_FIXED_TENDON, OVPHYSX_ATTR_TENDON_LIMIT, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_FIXED_TENDON, OVPHYSX_ATTR_TENDON_REST_LENGTH, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_FIXED_TENDON, OVPHYSX_ATTR_TENDON_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },

    // Spatial tendon (limit / rest length live on the leaf attachment, so they are not writable here)
    { OVPHYSX_OBJECT_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_LIMIT_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_SPATIAL_TENDON, OVPHYSX_ATTR_TENDON_OFFSET, OVPHYSX_WRITABILITY_WRITABLE },

    // Whole articulation (root-link state + inverse dynamics queries)
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_ROOT_ORIENTATION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_ROOT_ANGULAR_VELOCITY, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_CENTER_OF_MASS_WORLD, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_CENTER_OF_MASS_LOCAL, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_JACOBIAN, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_JACOBIAN_SHAPE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_MASS_MATRIX, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_CORIOLIS_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_GRAVITY_FORCE, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_CENTROIDAL_MOMENTUM, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_STATIC_FRICTION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_RESTITUTION, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_CONTACT_OFFSET, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_REST_OFFSET, OVPHYSX_WRITABILITY_READ_ONLY },
    { OVPHYSX_OBJECT_ARTICULATION, OVPHYSX_ATTR_SHAPE_COUNT, OVPHYSX_WRITABILITY_READ_ONLY },

    // Deformable material
    { OVPHYSX_OBJECT_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_DYNAMIC_FRICTION, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_YOUNGS_MODULUS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_POISSONS_RATIO, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_ELASTICITY_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_BENDING_STIFFNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_THICKNESS, OVPHYSX_WRITABILITY_WRITABLE },
    { OVPHYSX_OBJECT_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_BENDING_DAMPING, OVPHYSX_WRITABILITY_WRITABLE },
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

ovphysx_writability_t classifyWritability(ovphysx_sim_object_type_t type, const char* name, size_t len)
{
    for (const WritabilityRow& row : kWritabilityTable)
        if (row.object_type == type && attributeNameEquals(row.attribute, name, len))
            return row.writability;
    return OVPHYSX_WRITABILITY_UNCLASSIFIED;
}
} // namespace

OVPHYSX_API ovphysx_result_t ovphysx_writability(ovphysx_sim_object_type_t object_type,
                                                 const ovx_string_or_token_t* attribute,
                                                 ovphysx_writability_t* out_writability)
{
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

    // Reject an object_type outside ovphysx_sim_object_type_t (e.g. a value from the separate
    // OVPHYSX_OBJECT_TYPE_* tensor-binding domain ovphysx_types.h warns against confusing). The enum
    // is contiguous [RIGID_BODY, DEFORMABLE_MATERIAL], and the uint32_t cast folds negatives into the
    // high range, so this one upper bound covers them. Without it an unknown type falls through
    // classifyWritability and returns SUCCESS/UNCLASSIFIED, indistinguishable to a caller from a
    // valid type that simply does not accept the name.
    if (static_cast<uint32_t>(object_type) > OVPHYSX_OBJECT_DEFORMABLE_MATERIAL)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_writability: unknown object type");

    *out_writability = classifyWritability(object_type, attribute->string.ptr, attribute->string.length);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_fetch_write_next(ovphysx_handle_t handle,
                                                      ovphysx_write_handle_t write,
                                                      const ovstage_map_group_t** out_group)
{
    if (!out_group)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_write_next: out_group must be non-NULL");
    *out_group = nullptr;

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_fetch_write_next: invalid handle");

    auto fn = g_sidecarFetchWriteNext.load(std::memory_order_acquire);
    if (!fn)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_fetch_write_next: internal sidecar write API not loaded");

    // The group is producer-owned and its ADDRESS is the commit identity, so it travels back
    // unchanged. ovphysx adds no mirror struct, and must not, because a copy would be a pointer
    // the runtime cannot recognise at commit.
    const int rc = fn(write, out_group);
    if (rc < 0)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_fetch_write_next: bad write handle or internal error");
    if (rc == 0)
        return { OVPHYSX_API_END_OF_ITERATION }; // not an error: iteration exhausted
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_commit_group(ovphysx_handle_t handle,
                                                  ovphysx_write_handle_t write,
                                                  const ovstage_map_group_t* group,
                                                  ovstage_cuda_sync_t write_done_sync)
{
    if (!group)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_commit_group: group is NULL");

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_commit_group: invalid handle");

    auto fn = g_sidecarCommitGroup.load(std::memory_order_acquire);
    if (!fn)
        return set_error(OVPHYSX_API_ERROR, "ovphysx_commit_group: internal sidecar write API not loaded");

    // Deliberately an error rather than a quiet success: commit IS the mutation, so a success
    // return would tell the caller state was published when it was not.
    //
    // The two failures are reported apart because they mean different things to a caller. A group
    // that was never live was rejected before anything ran. A publish that failed had a live group
    // and a scatter that started, and this layer cannot say how much of it landed. Claiming
    // "nothing was published" for it would be a guarantee the runtime does not make.
    int32_t why = kOvphysxCommitFailureNone;
    if (fn(write, group, write_done_sync, &why) == 0)
    {
        if (why == kOvphysxCommitFailurePublish)
            return set_error(OVPHYSX_API_ERROR,
                             "ovphysx_commit_group: the group was live but publishing it failed. The "
                             "group is spent -- commit is not retryable -- and how much of it reached "
                             "the solver is not reported here; the runtime log names the reason.");
        if (why == kOvphysxCommitFailureNotLive)
            return set_error(OVPHYSX_API_ERROR,
                             "ovphysx_commit_group: the group is not live -- unknown, from another "
                             "session, or already committed. Nothing was published.");
        return set_error(OVPHYSX_API_ERROR,
                         "ovphysx_commit_group: the commit failed and the runtime reported no reason, "
                         "so whether anything was published is unknown -- the write sidecar faulted or "
                         "is not installed.");
    }
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_release_write(ovphysx_handle_t handle, ovphysx_write_handle_t write)
{
    (void)handle;
    // Idempotent for an unknown handle, matching ovphysx_release_read: teardown is always safe.
    // Uncommitted groups are DISCARDED here, not published.
    auto fn = g_sidecarReleaseWrite.load(std::memory_order_acquire);
    if (fn)
        fn(write);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_release_query(ovphysx_handle_t handle, ovphysx_query_handle_t query)
{
    (void)handle;
    auto fn = g_sidecarReleaseQuery.load(std::memory_order_acquire);
    if (fn)
        fn(query);
    return success(); // idempotent
}

OVPHYSX_API ovphysx_result_t ovphysx_cuda_stream_wait_event(uintptr_t stream, uintptr_t event)
{
    // event == 0 means "nothing to wait for". It is answered before the shim is touched so a
    // CPU-only process never reaches CUDA. Stream 0 is not a sentinel, it is the valid NULL stream.
    if (event == 0)
        return success();

    omni::physx::IOptionalCuda* cuda = omni::physx::runtime::tryGetOptionalCudaInterface();
    if (!cuda || !cuda->streamWaitEvent)
        return set_error(OVPHYSX_API_ERROR,
                         "ovphysx_cuda_stream_wait_event: CUDA is not available in this process");

    // No context push: the driver call must run in the CALLER's current context. Stream sentinels 1
    // and 2 are context-relative (the legacy / per-thread default stream of whatever context is
    // current), so making the ovphysx context current would order a stream the caller never used,
    // yet still succeed.

    // Enqueue-only: the driver records the dependency on `stream` and returns, with no host sync.
    int status = 0;
    if (!cuda->streamWaitEvent(stream, event, /*flags=*/0u, &status))
        return set_error(OVPHYSX_API_ERROR,
                         "ovphysx_cuda_stream_wait_event: cuStreamWaitEvent failed with CUDA status " +
                             std::to_string(status));
    return success();
}

// ---- PhysX debug visualization (forwards to the sidecar's IPhysxVisualization) ----
// Pattern mirrors ovphysx_get_physx_ptr: validate the handle/stage under the shared
// instances lock, then call the resolved sidecar fn-ptr. The sidecar (and OmniPhysX)
// own all PhysX access. A missing fn-ptr is a clean no-op SUCCESS.

namespace {
ovphysx_result_t vizValidate(ovphysx_handle_t handle)
{
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || !instance->ovstage_attached)
        return set_error(OVPHYSX_API_ERROR, "no physics stage attached");
    return success();
}

ovphysx_result_t vizOvstageValidate(ovphysx_handle_t handle)
{
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid OVPhysX handle");
    if (!instance->ovstage_attached)
        return set_error(OVPHYSX_API_ERROR,
                         "debug visualization scope requires an attached OVStage");
    return success();
}

// Cached debug-render state. omni::physx::IPhysxVisualization is process-global and
// exposes no getters, so OvPhysX remembers what it last set and the _get_* accessors
// return that. g_debugRenderParamValues[i] holds parameter i's value (0 = off, the
// default). The scale defaults to 1.0 (omni.physx default).
std::atomic<float> g_debugRenderParamValues[OVPHYSX_DEBUG_RENDER_PARAM_COUNT] = {};
std::atomic<float> g_debugRenderScale{1.0f};

bool vizParamInRange(uint32_t param)
{
    return param > static_cast<uint32_t>(OVPHYSX_DEBUG_RENDER_PARAM_NONE) &&
           param < static_cast<uint32_t>(OVPHYSX_DEBUG_RENDER_PARAM_COUNT);
}
} // anonymous namespace

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_enable(ovphysx_handle_t handle, bool enable)
{
    omni_sdk_physx_wait_all_pending_internal(handle);
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    auto v = vizValidate(handle);
    if (v.status != OVPHYSX_API_SUCCESS)
        return v;
    if (auto fn = g_sidecarEnableVisualization.load(std::memory_order_acquire))
        fn(enable);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_parameter(
    ovphysx_handle_t handle, uint32_t param, float value)
{
    // Reject NONE (0) and out-of-range BEFORE forwarding: omni::physx does
    // visMask |= (1ull << param) without a bound check (param >= 64 is UB), and
    // param 0 collides with the eSCALE slot in the enable loop.
    if (!vizParamInRange(param))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "param is NONE or out of range (expected 1 .. OVPHYSX_DEBUG_RENDER_PARAM_COUNT-1)");
    if (!std::isfinite(value) || value < 0.0f)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "value must be finite and >= 0");
    // Record the request for ovphysx_debug_render_get_parameter (last value requested
    // through ovphysx). Done before the stage/forward so the getter round-trips.
    g_debugRenderParamValues[param].store(value, std::memory_order_relaxed);
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    auto v = vizValidate(handle);
    if (v.status != OVPHYSX_API_SUCCESS)
        return v;
    // Two interface members drive one setting: forward the value only when drawing
    // (value > 0, so 0 does not overwrite the remembered value), then the on/off state.
    if (value > 0.0f)
    {
        if (OvphysxSidecarSetVizParameterValueFn fnv =
                g_sidecarSetVizParameterValue.load(std::memory_order_acquire))
            fnv(param, value);
    }
    if (auto fn = g_sidecarSetVizParameter.load(std::memory_order_acquire))
        fn(param, value > 0.0f);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_scope_tokens(
    ovphysx_handle_t handle, const ovx_primpath_t* tokens, uint32_t count)
{
    // Exact interned-path membership. No prefix expansion
    // here; the caller expands any hierarchy into its object set and creates
    // handles through the Stage's OVX dictionary. count 0 restores every object.
    if (count > 0 && !tokens)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "tokens is NULL with count > 0");
    for (uint32_t i = 0u; i < count; ++i)
    {
        if (tokens[i] == OVX_INVALID_PRIMPATH)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "tokens must not contain OVX_INVALID_PRIMPATH");
    }
    {
        std::shared_lock<std::shared_mutex> preflight_lock(g_instances_mutex);
        if (!get_instance_ptr(handle))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "invalid OVPhysX handle");
    }
    const ovphysx_api_status_t wait_status =
        omni_sdk_physx_wait_all_pending_internal(handle);
    if (wait_status != OVPHYSX_API_SUCCESS)
        return set_error(wait_status, "failed waiting for pending OVPhysX work");
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    auto v = vizOvstageValidate(handle);
    if (v.status != OVPHYSX_API_SUCCESS)
        return v;
    OvphysxSidecarSetVizScopeTokensFn fn =
        g_sidecarSetVizScopeTokens.load(std::memory_order_acquire);
    if (!fn || !fn(count != 0 ? tokens : nullptr, count))
        return set_error(OVPHYSX_API_ERROR,
                         "attached OVStage token visualization scope could not be applied");
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_parameter(
    ovphysx_handle_t handle, uint32_t param, float* out_value)
{
    (void)handle;
    if (!out_value)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_value is NULL");
    *out_value = 0.0f;
    if (!vizParamInRange(param))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "param is NONE or out of range (expected 1 .. OVPHYSX_DEBUG_RENDER_PARAM_COUNT-1)");
    *out_value = g_debugRenderParamValues[param].load(std::memory_order_relaxed);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_scale(ovphysx_handle_t handle, float scale)
{
    if (!std::isfinite(scale) || scale < 0.0f)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "scale must be finite and >= 0");
    // Record the request for ovphysx_debug_render_get_scale (see set_parameter).
    g_debugRenderScale.store(scale, std::memory_order_relaxed);
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    auto v = vizValidate(handle);
    if (v.status != OVPHYSX_API_SUCCESS)
        return v;
    if (auto fn = g_sidecarSetVizScale.load(std::memory_order_acquire))
        fn(scale);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_scale(ovphysx_handle_t handle, float* out_scale)
{
    (void)handle;
    if (!out_scale)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_scale is NULL");
    *out_scale = g_debugRenderScale.load(std::memory_order_relaxed);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_set_culling_box(
    ovphysx_handle_t handle, const float min3[3], const float max3[3])
{
    if (!min3 || !max3)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "min3/max3 is NULL");
    for (int i = 0; i < 3; ++i)
    {
        if (!std::isfinite(min3[i]) || !std::isfinite(max3[i]))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "min3/max3 must be finite");
        if (min3[i] > max3[i])
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "min3 must be <= max3 on every axis");
    }
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    auto v = vizValidate(handle);
    if (v.status != OVPHYSX_API_SUCCESS)
        return v;
    if (auto fn = g_sidecarSetVizCullingBox.load(std::memory_order_acquire))
        fn(min3, max3);
    return success();
}

// Getters: the returned pointer aliases the OvPhysX debug buffer (valid until the
// next step OR any stage/scene change). wait_all_pending ensures the buffer reflects
// the completed step. The CALLER's typed out-pointer is validated by each wrapper
// below (this internal local is always non-null).
static ovphysx_result_t vizGetBuffer(
    ovphysx_handle_t handle, std::atomic<OvphysxSidecarGetDebugBufferFn>& slot,
    const void** out_ptr, uint32_t* out_count)
{
    if (!out_count)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_count is NULL");
    *out_ptr = nullptr;
    *out_count = 0;
    omni_sdk_physx_wait_all_pending_internal(handle);
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    auto v = vizValidate(handle);
    if (v.status != OVPHYSX_API_SUCCESS)
        return v;
    if (auto fn = slot.load(std::memory_order_acquire))
        fn(out_ptr, out_count);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_points(
    ovphysx_handle_t handle, const ovphysx_debug_point_t** out_points, uint32_t* out_count)
{
    if (!out_points)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_points is NULL");
    const void* p = nullptr;
    auto r = vizGetBuffer(handle, g_sidecarGetDebugPoints, &p, out_count);
    *out_points = static_cast<const ovphysx_debug_point_t*>(p);
    return r;
}

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_lines(
    ovphysx_handle_t handle, const ovphysx_debug_line_t** out_lines, uint32_t* out_count)
{
    if (!out_lines)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_lines is NULL");
    const void* p = nullptr;
    auto r = vizGetBuffer(handle, g_sidecarGetDebugLines, &p, out_count);
    *out_lines = static_cast<const ovphysx_debug_line_t*>(p);
    return r;
}

OVPHYSX_API ovphysx_result_t ovphysx_debug_render_get_triangles(
    ovphysx_handle_t handle, const ovphysx_debug_triangle_t** out_triangles, uint32_t* out_count)
{
    if (!out_triangles)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_triangles is NULL");
    const void* p = nullptr;
    auto r = vizGetBuffer(handle, g_sidecarGetDebugTriangles, &p, out_count);
    *out_triangles = static_cast<const ovphysx_debug_triangle_t*>(p);
    return r;
}
