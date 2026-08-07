// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <ovstage/ovx_path_dictionary.h>           // ovx_primpath_list_t / ovx_path_dictionary_t
#include <ovstage/ovstage_api/ovstage_api_types.h> // ovstage_read_group_t / ovstage_query_result_t
#include <ovx/string_types.h>                       // ovx_string_or_token_t

#include <cstddef>
#include <cstdint>

// These are exported plugin symbols (the plugin is built with hidden default
// visibility), called by consumers that link the plugin .so directly — they are
// NOT routed through a carb interface.
#if defined(_MSC_VER)
#    if defined(OMNI_PHYSX_OVX_STATIC)
#        define OMNI_OVX_API
#    elif defined(OMNI_PHYSX_OVX_EXPORTS)
#        define OMNI_OVX_API __declspec(dllexport)
#    else
#        define OMNI_OVX_API __declspec(dllimport)
#    endif
#else
#    define OMNI_OVX_API __attribute__((visibility("default")))
#endif

namespace omni::physx
{

// Simulated object type a query selects (engine-internal type, not a USD
// predicate — ADR-0007 §3.3, the one deviation from a plain ovstage filter query).
enum OvxObjectType : uint32_t
{
    kOvxRigidBody         = 0, //!< dynamic rigid bodies (standalone + point-instancer instances)
    kOvxArticulationLink  = 1, //!< articulation link body transforms
    kOvxArticulationJoint = 2, //!< articulation joint state (per-axis; array group per joint)
    kOvxVehicleWheel      = 3, //!< vehicle wheel transforms
    kOvxDeformableVolume  = 4, //!< volume deformable meshes (points / velocities)
    kOvxDeformableSurface = 5, //!< surface deformable meshes
    kOvxParticleSet       = 6, //!< particle sets
};

// Selection scope.
enum OvxObjectScope : uint32_t
{
    kOvxAll = 0, //!< every object of the type. Stable across steps, but valid only until a
                 //!< STRUCTURAL change (object add/remove, instancer instance-count change),
                 //!< which requires an explicit re-query; current output metadata does not
                 //!< signal this change.
    kOvxActive = 1, //!< rigid/link/vehicle: getActiveActors(); deformable: awake; particle: dirty flags.
                    //!< SINGLE-FRAME: the active set is recomputed every step, so a kOvxActive
                    //!< handle (and groups/lists from it) is valid only for the step it was opened
                    //!< against — re-query each frame; do not cache across steps.
};

// Canonical physics-output attribute names (semantic, NOT USD attr names — e.g.
// "position", not "xformOp:translate"). Pass any of these — or your own — as an
// ovx_string_or_token_t to ovxReadAttributes. Which names a type produces is
// documented per type in ADR-0007 / PROPOSAL-ovx-generic-read-api.md.
namespace OvxAttr
{
inline constexpr const char* kPosition         = "position";         //!< vec3 f32 (world)
inline constexpr const char* kOrientation      = "orientation";      //!< quat f32 xyzw (world)
inline constexpr const char* kLinearVelocity   = "linearVelocity";   //!< vec3 f32
inline constexpr const char* kAngularVelocity  = "angularVelocity";  //!< vec3 f32
inline constexpr const char* kPoints           = "points";           //!< vec3 f32 [array]
inline constexpr const char* kVelocities       = "velocities";       //!< vec3 f32 [array]
inline constexpr const char* kPositions        = "positions";        //!< vec3 f32 [array] (instancer-local)
inline constexpr const char* kOrientations     = "orientations";     //!< quat f32 xyzw [array] (instancer-local)
inline constexpr const char* kAngularVelocities = "angularVelocities"; //!< vec3 f32 [array]
inline constexpr const char* kJointPosition    = "jointPosition";    //!< f32 [array, per-axis]
inline constexpr const char* kJointVelocity    = "jointVelocity";    //!< f32 [array, per-axis]
} // namespace OvxAttr

// Opaque handles. 0 is the invalid sentinel.
typedef uint64_t OvxOutputQueryHandle;
typedef uint64_t OvxReadHandle;

// A query is a reusable SELECTOR (type + scope), NOT a captured membership
// snapshot: matched prims and column values are evaluated LAZILY — the prim count
// at ovxFetchQueryResult time, the columns at ovxReadAttributes time. Both observe
// the engine state as of the most recently completed step (callers must have
// drained pending sim before reading). A step that lands between ovxQuery and
// ovxReadAttributes does not make the query stale; the read simply reflects the
// newer step. For kOvxActive this means "the active set of whatever step had last
// completed when ovxReadAttributes ran" — re-read before stepping again to capture
// a specific step's active set. The current output producer returns
// layout_generation == 0; rebuild after known structural changes rather than
// treating it as an invalidation signal.

// Three-valued result of ovxFetchReadNext, so a distinct end-of-iteration and a
// real error can each be forwarded upward (a bool collapses them). On any non-Ok
// status `*outGroup` is zero-initialized (no borrowed storage to release).
enum OvxReadStatus : int32_t
{
    kOvxReadStatusOk             = 0, //!< `*outGroup` filled with the next group.
    kOvxReadStatusEndOfIteration = 1, //!< all groups consumed (NOT an error); `*outGroup` zeroed.
    kOvxReadStatusError          = 2, //!< bad handle / internal failure; `*outGroup` zeroed.
};

// C linkage: pure C-ABI entry points (only ovstage C types + DLTensor), exported
// under stable, unmangled names. A consumer that does not link the plugin at load
// time resolves them by name with dlsym. Their C++ qualified name stays
// `omni::physx::ovx*`.
extern "C"
{

// QUERY (ovstage `query` analog): reserve a handle for the objects of `type`
// (OvxObjectType) in `scope` (OvxObjectScope). Synchronous — the handle is valid
// on return (like ovstage `query_from_path_list`). The result is the handle, NOT
// a path list: matched prims come back per group as ovstage_read_group_t::prims.list.
// An EMPTY match is still a valid, nonzero handle: ovxFetchQueryResult then reports
// total_prim_count == 0 and ovxReadAttributes yields a read that reaches
// end-of-iteration immediately. 0 is returned ONLY on failure — no active
// ovstage-backed simulation (the read is ovstage-only).
OMNI_OVX_API OvxOutputQueryHandle ovxQuery(uint32_t type, uint32_t scope);

// The shared source path dictionary backing every prim list this query's reads
// produce (the analog of ovstage `ovstage_get_path_dictionary(instance)`). A
// by-type consumer does not know the matched prims a priori, so it uses this to
// resolve a group's `prims.list` back to paths/tokens. Returns null when there is
// no active ovstage-backed simulation. Valid for the life of the simulation.
OMNI_OVX_API ovx_path_dictionary_t* ovxQueryDictionary(OvxOutputQueryHandle query);

// Optional discovery (ovstage `fetch_query_result` analog): fills `outResult` with
// the matched prim count (`total_prim_count`) and the output attributes available
// for this query's type (`attributes` / `attribute_count`, interned tokens valid
// until ovxReleaseQuery). `all_handle` / `query_result_id` are 0 (no ovstage query
// handle is involved). Returns false on a bad handle / null arg.
OMNI_OVX_API bool ovxFetchQueryResult(OvxOutputQueryHandle query, ovstage_query_result_t* outResult);

// READ (ovstage `read_attributes` analog): request named output attributes for a
// query. Names unknown for the queried type are skipped (no group emitted for
// them). Returns 0 on a bad query handle / null args.
OMNI_OVX_API OvxReadHandle ovxReadAttributes(OvxOutputQueryHandle query,
                                             const ovx_string_or_token_t* attrs,
                                             size_t attrCount);

// ITERATE (ovstage `fetch_read_next` analog): fill the next typed column group.
// Each group is homogeneous — fixed-size (one tensor over its prims) or array
// (one ragged tensor per prim). The group's tensors / prims.list / index_map are
// borrowed and stay valid until ovxReleaseRead (see "Group lifetime" above; an
// intervening step does not invalidate them). Returns kOvxReadStatusOk with
// `*outGroup` filled, kOvxReadStatusEndOfIteration when exhausted, or
// kOvxReadStatusError on a bad handle / internal failure; `*outGroup` is zeroed on
// any non-Ok status.
OMNI_OVX_API OvxReadStatus ovxFetchReadNext(OvxReadHandle read, ovstage_read_group_t* outGroup);

// Release a fetched group's borrowed storage. The group's prims.list is part of
// the read session and is released by ovxReleaseRead (not here).
OMNI_OVX_API void ovxReleaseGroup(OvxReadHandle read, const ovstage_read_group_t* group);

// Release the read session (and every prims.list / scratch column it owns).
OMNI_OVX_API void ovxReleaseRead(OvxReadHandle read);

// Release the query.
OMNI_OVX_API void ovxReleaseQuery(OvxOutputQueryHandle query);

} // extern "C"

} // namespace omni::physx
