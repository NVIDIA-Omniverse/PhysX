// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
OVPHYSX_INTERNAL_API void* ovphysx_internal_get_physx_ptr(
    const char* prim_path, int physx_type);

// Calls PxArticulationReducedCoordinate::updateKinematic on the articulation
// at `prim_path`. `flags` is a bitmask matching ovphysx_articulation_kinematic_flag_t
// (POSITION=1, VELOCITY=2). Returns false when the articulation is not found.
// Used by ovphysx_articulation_update_kinematic to propagate root and DOF state
// into the link buffer without simulating.
OVPHYSX_INTERNAL_API bool ovphysx_internal_update_kinematic(
    const char* prim_path, uint32_t flags);

// PhysX debug-visualization bridge, forwarding to omni::physx::IPhysxVisualization.
// enable authors the eVISUALIZATION flags, eSCALE and the param bitmask.
// set_parameter toggles one PhysXVisualizationParameter index. get_* hand back the
// OvPhysX debug buffer base pointer and element count, valid until the next step,
// which rewrites and may reallocate it. All are no-ops when the interface is
// unavailable.
OVPHYSX_INTERNAL_API void ovphysx_internal_enable_visualization(bool enable);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter(uint32_t param, bool on);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter_value(uint32_t param, float value);
// Restricts debug viz to the interned prim paths by exact membership
// (count 0 = all objects). Needs the ovstage token dictionary.
OVPHYSX_INTERNAL_API bool ovphysx_internal_set_visualization_scope_tokens(
    const ovx_primpath_t* tokens, uint32_t count);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_scale(float scale);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_culling_box(
    const float* min3, const float* max3);
OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_points(const void** out, uint32_t* count);
OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_lines(const void** out, uint32_t* count);
OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_triangles(const void** out, uint32_t* count);

// Physics output read (ADR-0007). These bridge the ovstage-native read symbols
// exported by the runtime (ovxQuery, ovxFetchQueryResult, ovxReadAttributes,
// ovxFetchReadNext, ovxReleaseGroup, ovxReleaseRead, ovxReleaseQuery,
// ovxQueryDictionary). The public ovphysx surface uses ovstage's own types
// directly (ovstage_read_group_t / ovstage_query_result_t / ovx_string_or_token_t),
// so the sidecar passes them straight through. It retains each live group by
// read_group_id so the group struct and prim list stay valid until
// ovphysx_internal_release_group. Numeric storage remains owned by the read
// session until release_read.
//
// `object_type` / `scope` are ovphysx_sim_object_type_t / ovphysx_object_scope_t.
// Handles are the runtime's ovx query/read handles passed through opaquely.
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_output_query(uint32_t object_type, uint32_t scope);
// Returns 1 and fills *out_result (ovstage's own type), 0 if not ready / no result, -1 on error.
OVPHYSX_INTERNAL_API int  ovphysx_internal_fetch_query_result(uint64_t query, ovstage_query_result_t* out_result);
// Opaque path_dictionary_instance_t* that interns the query's prim lists / tokens (NULL if none).
OVPHYSX_INTERNAL_API void* ovphysx_internal_query_dictionary(uint64_t query);
// Attributes are ovstage's ovx_string_or_token_t (string name or interned token per
// entry), forwarded straight to the runtime read with no translation.
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_read_outputs(uint64_t query,
                                                            const ovx_string_or_token_t* attributes,
                                                            size_t attribute_count);
// Returns 1 and points *out_group at a producer-owned ovstage_read_group_t (valid
// until ovphysx_internal_release_group / _release_read), 0 at end of iteration, -1 on
// error. The sidecar retains the ovstage group and hands back its address. The
// runtime owns the underlying storage.
OVPHYSX_INTERNAL_API int  ovphysx_internal_fetch_read_next(uint64_t read, const ovstage_read_group_t** out_group);
OVPHYSX_INTERNAL_API void ovphysx_internal_release_group(uint64_t read, ovstage_read_group_id_t group_id);
OVPHYSX_INTERNAL_API void ovphysx_internal_release_read(uint64_t read);
OVPHYSX_INTERNAL_API void ovphysx_internal_release_query(uint64_t query);

// App -> physics write (ADR-0012). The return direction of the read above, bridged the same
// way: the public surface uses ovstage's own ovstage_map_group_t, so these pass it straight
// through with no ovphysx mirror. A map group carries no id, so a committed group is identified
// by the address the runtime handed out. The group pointer therefore travels back unchanged
// rather than being copied into sidecar-side storage as the read's groups are.
OVPHYSX_INTERNAL_API uint64_t ovphysx_internal_write_attribute(uint64_t query,
                                                               const ovx_string_or_token_t* attribute);
// Returns 1 and points *out_group at a producer-owned ovstage_map_group_t (valid until
// ovphysx_internal_release_write), 0 at end of iteration, -1 on error.
OVPHYSX_INTERNAL_API int  ovphysx_internal_fetch_write_next(uint64_t write, const ovstage_map_group_t** out_group);
// Returns 1 on success, 0 when the group is not live (unknown, foreign, or already committed).
OVPHYSX_INTERNAL_API int  ovphysx_internal_commit_group(uint64_t write,
                                                        const ovstage_map_group_t* group,
                                                        ovstage_cuda_sync_t write_done_sync,
                                                        int32_t* out_failure);
OVPHYSX_INTERNAL_API void ovphysx_internal_release_write(uint64_t write);

// SDK-side function-pointer typedefs (mirror the above exports) for dlsym use.
typedef void*    (*OvphysxSidecarGetPhysXPtrFn)(const char*, int);
typedef bool     (*OvphysxSidecarUpdateKinematicFn)(const char*, uint32_t);
typedef void     (*OvphysxSidecarEnableVisualizationFn)(bool);
typedef void     (*OvphysxSidecarSetVizParameterFn)(uint32_t, bool);
typedef void     (*OvphysxSidecarSetVizParameterValueFn)(uint32_t, float);
typedef bool     (*OvphysxSidecarSetVizScopeTokensFn)(const ovx_primpath_t*, uint32_t);
typedef void     (*OvphysxSidecarSetVizScaleFn)(float);
typedef void     (*OvphysxSidecarSetVizCullingBoxFn)(const float*, const float*);
typedef void     (*OvphysxSidecarGetDebugBufferFn)(const void**, uint32_t*);
typedef uint64_t (*OvphysxSidecarOutputQueryFn)(uint32_t, uint32_t);
typedef int      (*OvphysxSidecarFetchQueryResultFn)(uint64_t, ovstage_query_result_t*);
typedef void*    (*OvphysxSidecarQueryDictionaryFn)(uint64_t);
typedef uint64_t (*OvphysxSidecarReadOutputsFn)(uint64_t, const ovx_string_or_token_t*, size_t);
typedef int      (*OvphysxSidecarFetchReadNextFn)(uint64_t, const ovstage_read_group_t**);
typedef void     (*OvphysxSidecarReleaseGroupFn)(uint64_t, ovstage_read_group_id_t);
typedef void     (*OvphysxSidecarReleaseReadFn)(uint64_t);
typedef void     (*OvphysxSidecarReleaseQueryFn)(uint64_t);
typedef uint64_t (*OvphysxSidecarWriteAttributeFn)(uint64_t, const ovx_string_or_token_t*);
typedef int      (*OvphysxSidecarFetchWriteNextFn)(uint64_t, const ovstage_map_group_t**);
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
typedef void     (*OvphysxSidecarReleaseWriteFn)(uint64_t);

} // extern "C"

// Resolved sidecar function pointers (populated by loadInternalSidecar()).
extern std::atomic<OvphysxSidecarGetPhysXPtrFn>      g_sidecarGetPhysXPtr;
extern std::atomic<OvphysxSidecarUpdateKinematicFn>  g_sidecarUpdateKinematic;
extern std::atomic<OvphysxSidecarEnableVisualizationFn> g_sidecarEnableVisualization;
extern std::atomic<OvphysxSidecarSetVizParameterFn>  g_sidecarSetVizParameter;
extern std::atomic<OvphysxSidecarSetVizParameterValueFn> g_sidecarSetVizParameterValue;
extern std::atomic<OvphysxSidecarSetVizScopeTokensFn> g_sidecarSetVizScopeTokens;
extern std::atomic<OvphysxSidecarSetVizScaleFn>      g_sidecarSetVizScale;
extern std::atomic<OvphysxSidecarSetVizCullingBoxFn> g_sidecarSetVizCullingBox;
extern std::atomic<OvphysxSidecarGetDebugBufferFn>   g_sidecarGetDebugPoints;
extern std::atomic<OvphysxSidecarGetDebugBufferFn>   g_sidecarGetDebugLines;
extern std::atomic<OvphysxSidecarGetDebugBufferFn>   g_sidecarGetDebugTriangles;
extern std::atomic<OvphysxSidecarOutputQueryFn>      g_sidecarOutputQuery;
extern std::atomic<OvphysxSidecarFetchQueryResultFn> g_sidecarFetchQueryResult;
extern std::atomic<OvphysxSidecarQueryDictionaryFn>  g_sidecarQueryDictionary;
extern std::atomic<OvphysxSidecarReadOutputsFn>      g_sidecarReadOutputs;
extern std::atomic<OvphysxSidecarFetchReadNextFn>    g_sidecarFetchReadNext;
extern std::atomic<OvphysxSidecarReleaseGroupFn>     g_sidecarReleaseGroup;
extern std::atomic<OvphysxSidecarReleaseReadFn>      g_sidecarReleaseRead;
extern std::atomic<OvphysxSidecarReleaseQueryFn>     g_sidecarReleaseQuery;
extern std::atomic<OvphysxSidecarWriteAttributeFn>   g_sidecarWriteAttribute;
extern std::atomic<OvphysxSidecarFetchWriteNextFn>   g_sidecarFetchWriteNext;
extern std::atomic<OvphysxSidecarCommitGroupFn>      g_sidecarCommitGroup;
extern std::atomic<OvphysxSidecarReleaseWriteFn>     g_sidecarReleaseWrite;
