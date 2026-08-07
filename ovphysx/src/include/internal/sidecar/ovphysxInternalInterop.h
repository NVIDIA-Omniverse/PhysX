// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "internal/sidecar/ovphysxInternalExport.h"
#include "ovphysx/ovphysx_types.h" // ovstage_read_group_t / ovstage_query_result_t / ovx_string_or_token_t

#include <atomic>
#include <cstddef>
#include <cstdint>

// Sidecar interop exports that bridge omni::physx / USD types into C-ABI
// values the main library can use directly: raw PhysX object pointer lookup
// by prim path, and USD SdfPath -> uint64_t encoding for IPhysxSceneQuery
// hit results.

extern "C" {

// Look up a raw PhysX object pointer by USD prim path and PhysX type.
// `physx_type` maps to omni::physx::PhysXType enum values.
// Returns the void* pointer or nullptr if not found.
OVPHYSX_INTERNAL_API void* ovphysx_internal_get_physx_ptr(
    const char* prim_path, int physx_type);

// Encode a USD prim path string to the uint64_t representation used by
// IPhysxSceneQuery and hit result structs. Returns 0 on failure.
OVPHYSX_INTERNAL_API uint64_t ovphysx_encode_sdf_path(const char* prim_path);

// Call PxArticulationReducedCoordinate::updateKinematic on the articulation
// at `prim_path`. `flags` is a bitmask matching ovphysx_articulation_kinematic_flag_t
// (POSITION=1, VELOCITY=2). Returns true if the articulation was found and
// updateKinematic was called, false otherwise. Used by ovphysx's
// ovphysx_articulation_update_kinematic to propagate root + DOF state into the
// link buffer without simulating. The sidecar resolves IPhysx through the
// load-time runtime accessor injection from ovphysx_internal_set_physx_runtime_accessors().
OVPHYSX_INTERNAL_API bool ovphysx_internal_update_kinematic(
    const char* prim_path, uint32_t flags);

// PhysX debug-visualization bridge (forwards to omni::physx::IPhysxVisualization,
// which links here in the sidecar). enable authors the eVISUALIZATION flags +
// eSCALE + the param bitmask; set_parameter toggles one PhysXVisualizationParameter
// index; get_* hand back the OvPhysX debug buffer base pointer + element count
// (valid until the next step, which rewrites and may reallocate it). All are
// no-ops when the interface is unavailable.
OVPHYSX_INTERNAL_API void ovphysx_internal_enable_visualization(bool enable);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter(uint32_t param, bool on);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_parameter_value(uint32_t param, float value);
// Scope: restrict debug viz to interned prim paths, exact
// membership (count 0 = all objects). Needs the ovstage token dictionary.
OVPHYSX_INTERNAL_API bool ovphysx_internal_set_visualization_scope_tokens(
    const ovx_primpath_t* tokens, uint32_t count);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_scale(float scale);
OVPHYSX_INTERNAL_API void ovphysx_internal_set_visualization_culling_box(
    const float* min3, const float* max3);
OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_points(const void** out, uint32_t* count);
OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_lines(const void** out, uint32_t* count);
OVPHYSX_INTERNAL_API void ovphysx_internal_get_debug_triangles(const void** out, uint32_t* count);

// Physics output read (ADR-0007). These bridge the ovstage-native read symbols
// exported by the runtime plugin (ovxQuery / ovxFetchQueryResult / ovxReadAttributes
// / ovxFetchReadNext / ovxReleaseGroup / ovxReleaseRead / ovxReleaseQuery /
// ovxQueryDictionary). The public ovphysx surface now speaks ovstage's own types
// directly (ovstage_read_group_t / ovstage_query_result_t / ovx_string_or_token_t),
// so the sidecar passes them straight through — no ovphysx mirror, no translation —
// and retains each live group by its read_group_id so its borrowed storage stays
// valid until ovphysx_internal_release_group.
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
// error. The sidecar retains the ovstage group and hands back its address — the
// runtime owns the underlying storage.
OVPHYSX_INTERNAL_API int  ovphysx_internal_fetch_read_next(uint64_t read, const ovstage_read_group_t** out_group);
OVPHYSX_INTERNAL_API void ovphysx_internal_release_group(uint64_t read, ovstage_read_group_id_t group_id);
OVPHYSX_INTERNAL_API void ovphysx_internal_release_read(uint64_t read);
OVPHYSX_INTERNAL_API void ovphysx_internal_release_query(uint64_t query);

// SDK-side function-pointer typedefs (mirror the above exports) for dlsym use.
typedef void*    (*OvphysxSidecarGetPhysXPtrFn)(const char*, int);
typedef uint64_t (*OvphysxSidecarEncodeSdfPathFn)(const char*);
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

} // extern "C"

// Resolved sidecar function pointers (populated by loadInternalSidecar()).
extern std::atomic<OvphysxSidecarGetPhysXPtrFn>      g_sidecarGetPhysXPtr;
extern std::atomic<OvphysxSidecarEncodeSdfPathFn>    g_sidecarEncodeSdfPath;
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
