// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-SESSION-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-3 AC-5 AC-7
 */

#pragma once

#include "internal/sidecar/ovphysxInternalExport.h"
#include "ovphysx/ovphysx_types.h" // ovstage_read_group_t / ovstage_query_result_t / ovx_string_or_token_t

#include <atomic>
#include <cstddef>
#include <cstdint>

// Sidecar interop exports that bridge omni::physx types into C-ABI values the
// main library can use directly.

extern "C" {

// Look up a raw PhysX object pointer by selector and PhysX type. An empty path
// selects the process-global PxPhysics and is valid only with ePTPhysics.
// `physx_type` maps to omni::physx::PhysXType enum values.
// Returns the void* pointer or nullptr if not found.
OVPHYSX_INTERNAL_API void* ovphysx_internal_get_physx_ptr(uint64_t attach, const char* prim_path, int physx_type);

// PhysX debug-visualization bridge, forwarding to omni::physx::IPhysxVisualization.
// enable authors the eVISUALIZATION flags, eSCALE and the param bitmask.
// set_parameter toggles one PhysXVisualizationParameter index. get_* hand back the
// OvPhysX debug buffer base pointer and element count, valid until the next step,
// which rewrites and may reallocate it. All are no-ops when the interface is
// unavailable.
OVPHYSX_INTERNAL_API void ovphysx_internal_enable_visualization(uint64_t attach, bool enable);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter(uint64_t attach, uint32_t param, bool on);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter_value(uint64_t attach, uint32_t param, float value);
// Restricts debug viz to the interned prim paths by exact membership
// (count 0 = all objects). Needs the ovstage token dictionary.
OVPHYSX_INTERNAL_API bool ovphysx_internal_set_visualization_scope_tokens(uint64_t attach, const ovx_primpath_t* tokens, uint32_t count);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_scale(uint64_t attach, float scale);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_culling_box(uint64_t attach, const float* min3, const float* max3);
OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_points(uint64_t attach, const void** out, uint32_t* count);
OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_lines(uint64_t attach, const void** out, uint32_t* count);
OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_triangles(uint64_t attach, const void** out, uint32_t* count);

// Instance-filtered debug render. apply writes settings onto scenes whose
// attach handle matches. copy reads those scenes' primitives into caller memory.
// A null buffer queries the count. *count is capacity on entry when the buffer
// is non-null, and the full count on return.
typedef struct ovphysx_internal_debug_render_state_t
{
    uint64_t attach_handle;
    int32_t enabled;
    float scale;
    float params[OVPHYSX_DEBUG_RENDER_PARAM_COUNT];
    int32_t has_culling_box;
    float cull_min[3];
    float cull_max[3];
    const uint64_t* scope_tokens;
    uint32_t scope_count;
} ovphysx_internal_debug_render_state_t;

OVPHYSX_INTERNAL_API void ovphysx_internal_debug_render_apply(
    const ovphysx_internal_debug_render_state_t* state);
OVPHYSX_INTERNAL_API void ovphysx_internal_debug_render_copy(
    uint64_t attach_handle,
    void* points, uint32_t* point_count,
    void* lines, uint32_t* line_count,
    void* triangles, uint32_t* triangle_count);

// Physics output read (ADR-0007). These bridge the ovstage-native read symbols
// exported by the runtime (ovxQuery, ovxReadAttributes,
// ovxFetchReadNext, ovxReleaseGroup, ovxReleaseRead, ovxReleaseQuery,
// with optional creation summaries). The public ovphysx surface uses ovstage's own types
// directly (ovstage_read_group_t / ovstage_query_result_t / ovx_string_or_token_t),
// so the sidecar passes them straight through. It retains each live group by
// read_group_id so the group struct and prim list stay valid until
// ovphysx_internal_release_group. Numeric storage remains owned by the read
// session until release_read.
//
// `object_type` / `scope` are ovphysx_query_target_t / ovphysx_object_scope_t.
// Handles are the runtime's ovx query/read handles passed through opaquely.
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_output_query(uint64_t attach, uint32_t object_type, uint32_t scope,
                                                            ovstage_query_result_t* out_summary);
// Prim-set selected query (ADR-0025): `primPaths` is an explicit list of native ovx_primpath_t handles the
// caller resolved, `pattern` a single glob resolved through the runtime matcher; the two are mutually
// exclusive (handles win, pattern is NULL then). There is no scope: a selected query covers its whole prim
// set. Declared here so the definition gets C linkage and the loader can dlsym it by plain name.
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_output_query_selected(uint64_t attach, uint32_t object_type,
                                                                     const ovx_primpath_t* primPaths,
                                                                     size_t primPathCount,
                                                                     const ovx_string_t* pattern,
                                                                     ovstage_query_result_t* out_summary);
// Contact query (ADR-0044): `selections` is a `const omni::physx::OvxContactSensorSelection*` of
// `selectionCount` entries, passed as void* to keep this header free of the runtime read types. Returns 0
// on failure, with the runtime error recorded.
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_output_query_contacts(uint64_t attach,
                                                                     const void* selections,
                                                                     size_t selectionCount,
                                                                     uint32_t filterCount,
                                                                     ovstage_query_result_t* out_summary);
// Attributes are ovstage's ovx_string_or_token_t (string name or interned token per
// entry), forwarded straight to the runtime read with no translation.
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_read_outputs(uint64_t query,
                                                            const ovx_string_or_token_t* attributes,
                                                            size_t attribute_count,
                                                            ovphysx_read_result_t* out_summary);
// Returns 1 and points *out_group at a producer-owned ovstage_read_group_t (valid
// until ovphysx_internal_release_group / _release_read), 0 at end of iteration, -1 on
// error. The sidecar retains the ovstage group and hands back its address. The
// runtime owns the underlying storage.
OVPHYSX_INTERNAL_API int  ovphysx_internal_fetch_read_next(uint64_t read, const ovstage_read_group_t** out_group);
// Release status (including release_write below): 1 confirms terminal success,
// 0 confirms terminal failure, and -1 means cleanup completion is unconfirmed
// (unavailable backend or caught exception). The SDK retains cleanup responsibility;
// a failed accepted public release does not permit the caller to retry it.
OVPHYSX_INTERNAL_API int ovphysx_internal_release_group(uint64_t read, ovstage_read_group_id_t group_id);
OVPHYSX_INTERNAL_API int ovphysx_internal_release_read(uint64_t read);
OVPHYSX_INTERNAL_API int ovphysx_internal_release_query(uint64_t query);

// App -> physics write (ADR-0012). The return direction of the read above, bridged the same
// way: the public surface uses ovstage's own ovstage_map_group_t, so these pass it straight
// through with no ovphysx mirror. A map group carries no id, so a committed group is identified
// by the address the runtime handed out. The group pointer therefore travels back unchanged
// rather than being copied into sidecar-side storage as the read's groups are.
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_write_attribute(uint64_t query,
                                                               const ovx_string_or_token_t* attributes, size_t attribute_count,
                                                               ovphysx_write_result_t* out_summary);
// Returns 1 and points *out_group at a producer-owned ovstage_map_group_t (valid until
// ovphysx_internal_release_write), 0 at end of iteration, -1 on error.
OVPHYSX_INTERNAL_API int  ovphysx_internal_fetch_write_next(uint64_t write, ovphysx_fetched_write_group_t* out_group);
// Returns 1 on success, 0 when the group is not live (unknown, foreign, or already committed).
OVPHYSX_INTERNAL_API int  ovphysx_internal_commit_group(uint64_t write,
                                                        const ovstage_map_group_t* group,
                                                        ovstage_cuda_sync_t write_done_sync,
                                                        int32_t* out_failure);
OVPHYSX_INTERNAL_API int ovphysx_internal_release_write(uint64_t write);

// SDK-side function-pointer typedefs (mirror the above exports) for dlsym use.
typedef void     (*OvphysxSidecarEnableVisualizationFn)(uint64_t, bool);
typedef void     (*OvphysxSidecarSetVizParameterFn)(uint64_t, uint32_t, bool);
typedef void     (*OvphysxSidecarSetVizParameterValueFn)(uint64_t, uint32_t, float);
typedef bool     (*OvphysxSidecarSetVizScopeTokensFn)(uint64_t, const ovx_primpath_t*, uint32_t);
typedef void     (*OvphysxSidecarSetVizScaleFn)(uint64_t, float);
typedef void     (*OvphysxSidecarSetVizCullingBoxFn)(uint64_t, const float*, const float*);
typedef void     (*OvphysxSidecarGetDebugBufferFn)(uint64_t, const void**, uint32_t*);
typedef void     (*OvphysxSidecarDebugRenderApplyFn)(const ovphysx_internal_debug_render_state_t*);
typedef void     (*OvphysxSidecarDebugRenderCopyFn)(uint64_t, void*, uint32_t*, void*, uint32_t*, void*, uint32_t*);
typedef uint64_t (*OvphysxSidecarOutputQueryFn)(uint64_t, uint32_t, uint32_t, ovstage_query_result_t*);
typedef uint64_t (*OvphysxSidecarOutputQuerySelectedFn)(uint64_t, uint32_t, const ovx_primpath_t*, size_t, const ovx_string_t*, ovstage_query_result_t*);
typedef uint64_t (*OvphysxSidecarReadOutputsFn)(uint64_t, const ovx_string_or_token_t*, size_t, ovphysx_read_result_t*);
typedef uint64_t (*OvphysxSidecarOutputQueryContactsFn)(uint64_t, const void*, size_t, uint32_t, ovstage_query_result_t*);
typedef int      (*OvphysxSidecarFetchReadNextFn)(uint64_t, const ovstage_read_group_t**);
typedef int      (*OvphysxSidecarReleaseGroupFn)(uint64_t, ovstage_read_group_id_t);
typedef int      (*OvphysxSidecarReleaseReadFn)(uint64_t);
typedef int      (*OvphysxSidecarReleaseQueryFn)(uint64_t);
typedef uint64_t (*OvphysxSidecarWriteAttributeFn)(uint64_t, const ovx_string_or_token_t*, size_t, ovphysx_write_result_t*);
typedef int      (*OvphysxSidecarFetchWriteNextFn)(uint64_t, ovphysx_fetched_write_group_t*);
// Mirrors omni::physx::OvxCommitFailure. Duplicated rather than included because this side of the
// sidecar boundary does not depend on the ovruntime headers. ovphysxSidecarLoader.cpp sees both
// and static_asserts that the two agree, so the copy cannot drift silently.
enum OvphysxSidecarCommitFailure : int32_t
{
    kOvphysxCommitFailureNone    = 0,
    kOvphysxCommitFailureNotLive = 1, //!< Rejected before any publish. Nothing was written.
    kOvphysxCommitFailurePublish = 2, //!< The group was live and the publish failed, possibly partway.
};

// The trailing int32_t* is ovxCommitGroup's optional OvxCommitFailure out-param: it separates a
// group that was never live from one whose publish failed, which the bool cannot.
typedef int      (*OvphysxSidecarCommitGroupFn)(uint64_t, const ovstage_map_group_t*, ovstage_cuda_sync_t, int32_t*);
typedef int      (*OvphysxSidecarReleaseWriteFn)(uint64_t);

} // extern "C"

// Resolved sidecar function pointers (populated by loadInternalSidecar()).
extern std::atomic<OvphysxSidecarEnableVisualizationFn> g_sidecarEnableVisualization;
extern std::atomic<OvphysxSidecarSetVizParameterFn>  g_sidecarSetVizParameter;
extern std::atomic<OvphysxSidecarSetVizParameterValueFn> g_sidecarSetVizParameterValue;
extern std::atomic<OvphysxSidecarSetVizScopeTokensFn> g_sidecarSetVizScopeTokens;
extern std::atomic<OvphysxSidecarSetVizScaleFn>      g_sidecarSetVizScale;
extern std::atomic<OvphysxSidecarSetVizCullingBoxFn> g_sidecarSetVizCullingBox;
extern std::atomic<OvphysxSidecarGetDebugBufferFn>   g_sidecarGetDebugPoints;
extern std::atomic<OvphysxSidecarGetDebugBufferFn>   g_sidecarGetDebugLines;
extern std::atomic<OvphysxSidecarGetDebugBufferFn>   g_sidecarGetDebugTriangles;
extern std::atomic<OvphysxSidecarDebugRenderApplyFn> g_sidecarDebugRenderApply;
extern std::atomic<OvphysxSidecarDebugRenderCopyFn>  g_sidecarDebugRenderCopy;
extern std::atomic<OvphysxSidecarOutputQueryFn>      g_sidecarOutputQuery;
extern std::atomic<OvphysxSidecarOutputQuerySelectedFn> g_sidecarOutputQuerySelected;
extern std::atomic<OvphysxSidecarOutputQueryContactsFn> g_sidecarOutputQueryContacts;
extern std::atomic<OvphysxSidecarReadOutputsFn>      g_sidecarReadOutputs;
extern std::atomic<OvphysxSidecarFetchReadNextFn>    g_sidecarFetchReadNext;
extern std::atomic<OvphysxSidecarReleaseGroupFn>     g_sidecarReleaseGroup;
extern std::atomic<OvphysxSidecarReleaseReadFn>      g_sidecarReleaseRead;
extern std::atomic<OvphysxSidecarReleaseQueryFn>     g_sidecarReleaseQuery;
extern std::atomic<OvphysxSidecarWriteAttributeFn>   g_sidecarWriteAttribute;
extern std::atomic<OvphysxSidecarFetchWriteNextFn>   g_sidecarFetchWriteNext;
extern std::atomic<OvphysxSidecarCommitGroupFn>      g_sidecarCommitGroup;
extern std::atomic<OvphysxSidecarReleaseWriteFn>     g_sidecarReleaseWrite;
