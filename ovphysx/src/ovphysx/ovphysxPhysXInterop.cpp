// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// PhysX object interop: exposes raw PhysX pointers by USD prim path + type enum.
//
// The actual SdfPath->PhysX lookup runs in the internal sidecar (which links USD).
// The sidecar loader resolves the function pointer at instance creation and
// publishes it via g_sidecarGetPhysXPtr; this file just reads that.

#include "ovphysx/ovphysx.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sidecar/ovphysxInternalInterop.h"  // g_sidecarGetPhysXPtr

#include <carb/Framework.h>

#include <cmath>   // std::isfinite for debug-render arg validation
#include <string>

// Sidecar get-physx-ptr atomic owned here next to its consumer; loader writes
// it during loadInternalSidecar() via the extern in ovphysxInternalInterop.h.
// (g_sidecarEncodeSdfPath lives in ovphysxSceneQuery.cpp.)
std::atomic<OvphysxSidecarGetPhysXPtrFn> g_sidecarGetPhysXPtr{nullptr};
std::atomic<OvphysxSidecarUpdateKinematicFn> g_sidecarUpdateKinematic{nullptr};

// Physics output read (ADR-0007) sidecar pointers, owned here next to their
// public-API consumers below; the loader writes them during loadInternalSidecar().
std::atomic<OvphysxSidecarOutputQueryFn>      g_sidecarOutputQuery{nullptr};
std::atomic<OvphysxSidecarFetchQueryResultFn> g_sidecarFetchQueryResult{nullptr};
std::atomic<OvphysxSidecarQueryDictionaryFn>  g_sidecarQueryDictionary{nullptr};
std::atomic<OvphysxSidecarReadOutputsFn>      g_sidecarReadOutputs{nullptr};
std::atomic<OvphysxSidecarFetchReadNextFn>    g_sidecarFetchReadNext{nullptr};
std::atomic<OvphysxSidecarReleaseGroupFn>     g_sidecarReleaseGroup{nullptr};
std::atomic<OvphysxSidecarReleaseReadFn>      g_sidecarReleaseRead{nullptr};
std::atomic<OvphysxSidecarReleaseQueryFn>     g_sidecarReleaseQuery{nullptr};
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
    auto fn = g_sidecarGetPhysXPtr.load(std::memory_order_acquire);
    if (!fn)
        return {nullptr, LookupStatus::kPluginUnavailable};
    void* ptr = fn(prim_path, type);
    return {ptr, ptr ? LookupStatus::kOk : LookupStatus::kNotFound};
}

static ovphysx_result_t validateInteropArgs(
    const ovphysx_string_t& prim_path, void** out_ptr)
{
    if (!out_ptr)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "out_ptr is NULL");
    *out_ptr = nullptr;

    if (!isValid(prim_path))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "prim_path is NULL or empty");
    if (hasEmbeddedNul(prim_path))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "prim_path contains an embedded NUL byte");

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
    auto check = validateInteropArgs(prim_path, out_ptr);
    if (check.status != OVPHYSX_API_SUCCESS)
        return check;

    const std::string prim_path_str = toStdString(prim_path);

    omni_sdk_physx_wait_all_pending_internal(handle);

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || instance->attachedStageId == 0)
        return set_error(OVPHYSX_API_ERROR, "no USD stage loaded");

    auto lookup = lookupPhysXPtr(prim_path_str.c_str(), static_cast<int>(physx_type));
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

    // The read observes the latest step's sealed output; make sure pending sim
    // work has completed before querying.
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

    // #6: an EMPTY match is still a valid, nonzero query — discover it via
    // ovphysx_fetch_query_result (total_prim_count == 0) or an immediate
    // end-of-iteration from ovphysx_fetch_read_next. A zero handle therefore means
    // FAILURE only. We have already confirmed an ovstage Stage is attached above,
    // so a zero handle from the runtime indicates an internal failure to open the
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

// #12: ovstage-attachment is validated once, at ovphysx_query — the entry point of
// the query → (fetch_query_result | read) → fetch_read_next → release sequence.
// The downstream calls here operate on the handle that query produced; they do not
// re-check attachment because a live handle can only exist for an attached Stage,
// and they fail safely (the runtime returns 0 / end-of-iteration / error) if the
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

OVPHYSX_API ovphysx_result_t ovphysx_release_query(ovphysx_handle_t handle, ovphysx_query_handle_t query)
{
    (void)handle;
    auto fn = g_sidecarReleaseQuery.load(std::memory_order_acquire);
    if (fn)
        fn(query);
    return success(); // idempotent
}

// ---- PhysX debug visualization (forwards to the sidecar's IPhysxVisualization) ----
// Pattern mirrors ovphysx_get_physx_ptr: validate the handle/stage under the shared
// instances lock, then call the resolved sidecar fn-ptr. The sidecar (and OmniPhysX)
// own all PhysX access; missing fn-ptr -> a clean no-op SUCCESS.

namespace {
ovphysx_result_t vizValidate(ovphysx_handle_t handle)
{
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance || (instance->attachedStageId == 0 && !instance->ovstage_attached))
        return set_error(OVPHYSX_API_ERROR, "no USD stage loaded");
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
// default); scale defaults to 1.0 (omni.physx default).
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
