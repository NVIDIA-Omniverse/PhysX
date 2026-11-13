// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-READ-RESULT-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6

/**
 * @implements REQ-READ-VEHICLE-001
 * @covers AC-8 AC-9
 *
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-20 AC-21
 *
 * @implements REQ-READ-DEVICE-001
 * @covers AC-10
 *
 * @implements REQ-TENSOR-FK-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-READ-CCT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-9
 *
 * @implements REQ-READ-SDF-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 *
 * @implements REQ-READ-SLEEP-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-READ-POINTER-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-ATTACH-IO-001
 * @covers AC-2
 *
 * @implements REQ-READ-CONTACT-002
 * @covers AC-1 AC-2
 *
 * @implements REQ-READ-CONTACT-003
 * @covers AC-1
 */

#pragma once

#include <ovstage/ovx_path_dictionary.h>           // ovx_primpath_list_t / ovx_path_dictionary_t
#include <ovstage/ovstage_api/ovstage_api_types.h> // ovstage_read_group_t / ovstage_query_result_t
#include <ovx/string_types.h>                       // ovx_token_or_string_t

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

// Query target a read selects (engine-internal type, not a USD
// predicate — ADR-0007 §3.3, the one deviation from a plain ovstage filter query).
//
// This enum belongs to the generic output-read API. Values match
// ovphysx_query_target_t, including its append-only numbering. In particular,
// kOvxArticulation is not the TensorBindings ObjectType::ARTICULATION selector
// (numeric value 2): the two APIs have separate enum domains and their values
// must not be mixed.
enum OvxObjectType : uint32_t
{
    kOvxRigidBody         = 0, //!< dynamic rigid bodies (standalone + point-instancer instances)
    kOvxArticulationLink  = 1, //!< articulation link body transforms
    kOvxArticulationJoint = 2, //!< articulation joint DOFs (one row per unlocked axis)
    kOvxVehicleWheel      = 3, //!< vehicle wheel transforms
    kOvxDeformableVolume  = 4, //!< volume deformable meshes (points / velocities)
    kOvxDeformableSurface = 5, //!< surface deformable meshes
    kOvxParticleSet       = 6, //!< particle sets
    kOvxFixedTendon       = 7, //!< articulation fixed tendons (prim: the joint carrying the root axis)
    kOvxSpatialTendon     = 8, //!< articulation spatial tendons (prim: the link carrying the root attachment)
    // Overlaps kOvxArticulationLink -- a root prim is also a link -- which is fine, because these
    // are queries and not a partition: the two types reach the same prim through different state. A
    // link reaches its own body properties; this reaches the articulation's ROOT pose and velocity,
    // which PhysX exposes per articulation and not per link.
    kOvxArticulation      = 9, //!< whole articulations (prim: the articulation-root API prim)
    kOvxDeformableMaterial = 10, //!< deformable materials (prim: the Material prim a deformable binds)
    // Contact-reporting sensors: every body whose contacts PhysX reports -- a body carrying
    // PhysxContactReportAPI, or a link of an articulation carrying it -- and, on a host scene, every
    // collider carrying the schema. A body row aggregates all of its shapes' contacts; a collider row reads
    // only its own shape's, so the two overlap -- do not sum net forces or count contacts across a body and
    // its own colliders, or they double-count. Collider rows are host-only: a DirectGPU scene does not
    // enumerate them. Discovered as an engine-internal set like every other type, not a USD predicate; there
    // is no active notion here, so kOvxActive behaves as kOvxAll. A sensor with no contact this step is still
    // a row, valued zero. 11 matches OVPHYSX_QUERY_TARGET_CONTACT_SENSOR.
    kOvxContactSensor      = 11, //!< contact-reporting bodies and (host scene) colliders (their contact forces, contacts and events)
    kOvxCharacterController = 12, //!< character controllers (prim: the capsule carrying PhysxCharacterControllerAPI)
    kOvxVehicle            = 13, //!< whole-vehicle controls and drive state
    kOvxScene = 14,               //!< PxScene (prim: the physics scene)
    kOvxPhysxMaterial = 15,       //!< PxMaterial (not a deformable material)
    kOvxShape = 16,               //!< PxShape
    kOvxCompoundShape = 17,       //!< compound-shape wrapper
    kOvxRigidActor = 18,          //!< PxRigidDynamic and PxRigidStatic (not the dynamics-only rigid-body query)
    kOvxJoint = 19,               //!< standalone PxJoint
    kOvxCustomJoint = 20,         //!< plugin custom joint
    kOvxParticleSystem = 21,      //!< PxPBDParticleSystem
    kOvxPhysics = 22,             //!< process-global PxPhysics; one row, path "/"
    // Instance target, not a simulated prim. Same numbering as the entries above:
    // this value stays put, and a later target appends after it.
    // A query does not require an attachment; settings are stored for the calling
    // instance and survive detach.
    kOvxDebugRender        = 23,
    // SDF collision shapes (ADR-0048): the shapes the SDF view accepts -- a collider carrying
    // PhysxSDFMeshCollisionAPI whose PhysX shape is a triangle mesh with an SDF, runtime clones included.
    // Query-only: no read or write attribute; ovxEvaluateSdfShapes consumes the query. No active notion, so
    // kOvxActive behaves as kOvxAll. 24 matches OVPHYSX_QUERY_TARGET_SDF_SHAPE.
    kOvxSdfShape           = 24, //!< SDF collision shapes (prim: the collider carrying PhysxSDFMeshCollisionAPI)
    // Skin meshes of volume and surface deformables, one row per skin mesh prim whatever the owner's kind.
    // Read-only because skinning rewrites the positions every step. To move a skin mesh, write its owner's
    // `points`.
    // 25 matches OVPHYSX_QUERY_TARGET_DEFORMABLE_SKIN_MESH.
    kOvxDeformableSkinMesh = 25, //!< deformable skin meshes (prim: the skinned mesh prim; `points` only, skin-mesh-local)
};

// Selection scope.
enum OvxObjectScope : uint32_t
{
    kOvxAll = 0, //!< every object of the type. Stable across steps, but valid only until a
                 //!< STRUCTURAL change (object add/remove, instancer instance-count change),
                 //!< which requires an explicit re-query; current output metadata does not
                 //!< signal this change.
    kOvxActive = 1, //!< rigid/link/vehicle: active-actor reports where available. Without reporting,
                    //!< rigid bodies use awake non-kinematic state, links use articulation awake state,
                    //!< and wheels use chassis awake state. Whole articulations use any active link,
                    //!< falling back to awake state without reporting. DirectGPU scenes disable sleeping,
                    //!< so whole-articulation ACTIVE membership there is equivalent to ALL.
                    //!< character controller: the controller actor's active-actor membership when its
                    //!< scene reports one, otherwise that actor's awake state.
                    //!< deformable: awake. particle/joint/tendon/contact sensor/SDF shape, and the
                    //!< pointer-only types (scene, PhysX material, shape, compound shape,
                    //!< rigid actor, standalone joint, custom joint, particle system,
                    //!< PxPhysics): no active notion -- behaves as kOvxAll.
                    //!< kOvxPhysics is one row and ignores prim-set selection.
                    //!< A whole-type query is reusable across steps. Each new session captures
                    //!< membership using these type-specific rules; existing session snapshots
                    //!< retain their original membership until released or invalidated.
};

// Canonical physics-output attribute names (semantic, NOT USD attr names — e.g.
// "position", not "xformOp:translate"). Pass any of these — or your own — as an
// ovx_token_or_string_t to ovxReadAttributes. Which names a type produces is
// documented per type in ADR-0007 / PROPOSAL-ovx-generic-read-api.md.
namespace OvxAttr
{
// Angular joint conversion is per quantity: coordinates/rates use degrees, gains and
// per-rate coefficients use inverse degrees, and efforts, inertia and enums take no fold.
// Signed quantities also carry the joint body-order sign. Applying one fold to a whole
// block causes 57.3x / 3283x errors; REQ-READ-ATTRS-001 AC-15 lists the per-attribute rules.
//
// Prefixes distinguish quantities and layouts: joint friction is per DOF, rigid friction
// is per shape, and tendon stiffness/damping are separate from joint drive gains.
// Deformable material properties are one scalar per material prim, unlike per-shape
// rigid dynamicFriction. The deformable prefix also leaves material* names available
// for a rigid-material type.
//
// dofAxisIndex counts only unlocked axes in PhysX TWIST/SWING1/SWING2 order. A locked
// TWIST compacts SWING1/SWING2 to 0/1; the result is not a canonical axis ID.
#include <omni/physx/generated/AttributeNames.inc>
} // namespace OvxAttr

// Opaque handles. 0 is the invalid sentinel.
typedef uint64_t OvxOutputQueryHandle;
typedef uint64_t OvxReadHandle;

// Reasons a requested column was omitted. Bits aggregate across applicable
// selected partitions; they do not identify or count the affected objects.
enum OvxReadOmissionReason : uint32_t
{
    kOvxReadOmissionUnsupportedAttribute = 1,
    kOvxReadOmissionUnsupportedConfiguration = 2,
    kOvxReadOmissionNotReady = 4,
};

struct OvxReadAttributeResult
{
    ovx_token_t attribute; //!< Canonical token; invalid for an unsupported name.
    size_t groupCount; //!< Emitted groups (batches), not objects or tensor rows.
    uint32_t omissionReasons; //!< Bitwise OR of omission reasons; may coexist with emitted groups.
};

// Optional read-open summary. Counts describe this open, not future reads or
// completion of device work. The caller owns this struct; attributes borrows
// attributeCount read-owned rows until ovxReleaseRead releases the session.
// Copying this struct copies the attributes pointer, not its rows. Copy the rows before
// release to retain them longer; their tokens still require the owner's dictionary.
struct OvxReadResult
{
    size_t totalPrimCount; //!< Matched prim identities, not emitted rows or ready GPU objects.
    size_t attributeCount; //!< Original request length, including unsupported attributes.
    const OvxReadAttributeResult* attributes;
};

// A whole-type query is a reusable selector: read membership is evaluated when
// ovxReadAttributes runs. Optional creation metadata is only a snapshot of the
// current membership. A selected query instead captures its prim selection once.
// Reads observe the most recently completed step and require idle runtime admission.
// The current producer returns layout_generation == 0; rebuild after known
// structural changes rather than treating it as an invalidation signal.
//
// Readiness is per selected scene partition. DirectGPU rigid bodies, instancers,
// articulation links/roots/joints and tendons may require a completed first step.
// Their applicable attributes report kOvxReadOmissionNotReady until then. Point-set
// producers can expose authored initial values when their buffers are available.
// A known backend error fails read creation; expected unavailability is metadata.

// One prim-set selection: EITHER explicit prim handles the caller already holds OR a glob `pattern`,
// the pair ovxQuerySelected takes as separate arguments. The public C boundary resolves prim_paths-wins
// precedence, so at most one is set.
struct OvxPrimSelection
{
    const ovx_primpath_t* primPaths; //!< explicit handles (null when the pattern is used)
    size_t primPathCount;            //!< number of handles (0 when the pattern is used)
    const ovx_string_t* pattern;     //!< glob (null when handles are used)
};

// One contact-sensor selection with its filters (ADR-0044): `filters` points at the query's filterCount
// filter selections, and is null when filterCount is 0.
struct OvxContactSensorSelection
{
    OvxPrimSelection sensors;
    const OvxPrimSelection* filters;
};

// The most filters a contact query can pair with each sensor: a force-matrix row is filterCount x 3
// floats folded into one group's 16-bit lanes (ADR-0026 lanes-fold).
inline constexpr uint32_t kOvxMaxContactFilters = 65535u / 3u;

// The most SDF shapes one ovxEvaluateSdfShapes call evaluates: PhysX launches its SDF evaluation with one CUDA grid row
// per shape, and a grid has at most 65535 rows.
inline constexpr uint32_t kOvxMaxSdfShapesPerEvaluation = 65535u;

// The most points, N x Q, one ovxEvaluateSdfShapes call evaluates: PhysX addresses the concatenated points in 32-bit
// arithmetic, row offsets and launch rounding included, and this bound keeps all of it in range.
inline constexpr uint32_t kOvxMaxSdfPointsPerEvaluation = 0x7FFFFFFFu;

// Result of ovxFetchReadNext, so a distinct end-of-iteration and a
// real error can each be forwarded upward (a bool collapses them). On end-of-iteration/error
// status `*outGroup` is zero-initialized (no borrowed storage to release).
enum OvxReadStatus : int32_t
{
    kOvxReadStatusOk             = 0, //!< `*outGroup` filled with the next group.
    kOvxReadStatusEndOfIteration = 1, //!< all groups consumed (NOT an error); `*outGroup` zeroed.
    kOvxReadStatusBusy           = 3, //!< runtime busy; output and cursor are unchanged.
    kOvxReadStatusError          = 2, //!< bad handle / internal failure; `*outGroup` zeroed.
};

// C linkage: pure C-ABI entry points (only ovstage C types + DLTensor), exported
// under stable, unmangled names. A consumer that does not link the plugin at load
// time resolves them by name with dlsym. Their C++ qualified name stays
// `omni::physx::ovx*`.
// Operations require idle runtime admission. Release returns false on busy
// without dropping the caller's ownership; retry after pending work completes.
extern "C"
{

// QUERY (ovstage `query` analog): reserve a handle for the objects of `type`
// (OvxObjectType) in `scope` (OvxObjectScope). Synchronous — the handle is valid
// on return (like ovstage `query_from_path_list`). The result is the handle, NOT
// a path list: matched prims come back per group as ovstage_read_group_t::prims.list.
// An EMPTY match is still a valid, nonzero handle: optional metadata reports
// total_prim_count == 0 and ovxReadAttributes yields a read that reaches
// end-of-iteration immediately. 0 is returned ONLY on failure -- no live matching
// ovstage-backed attachment (the read is ovstage-only). If outSummary is non-null,
// its immutable borrowed attribute array belongs to the query until release, and
// the prim count and attributes describe creation time. Later discovery does not
// change them. query_result_id and all_handle are reserved zero.
// A null outSummary skips discovery enumeration and token interning.
OMNI_OVX_API OvxOutputQueryHandle ovxQuery(uint64_t attachHandle, uint32_t type, uint32_t scope,
                                         ovstage_query_result_t* outSummary = nullptr);

// Debug-render query. Does not require an attachment. `instanceId` owns the settings,
// which survive detach. `scope` is stored and ignored; geometry filtering is the
// debug-render scope setting. 0 is failure.
OMNI_OVX_API OvxOutputQueryHandle ovxOpenDebugRenderQuery(uint64_t instanceId, uint32_t scope);

// Record the instance's current attach. `clearScope` drops dictionary-bound scope tokens.
// Pass attach 0 when nothing is attached.
OMNI_OVX_API void ovxDebugRenderNoteAttach(uint64_t instanceId, uint64_t attach, int clearScope);

// Drop the instance's settings. A live attach is pushed a disabled visualization first.
OMNI_OVX_API void ovxDebugRenderForget(uint64_t instanceId);

// The explicit attachment has the same idle-owner admission rules as ovxQuery.
// Like ovxQuery, but the objects of `type` are narrowed to a prim-set selection instead of a scope, for
// BOTH the read and a write opened over the query (ADR-0025). A selected query has no scope: it always
// covers its whole prim set (kOvxAll), sleeping members included, so a write never skips a selected prim
// for not having moved. The selection is EITHER an explicit list of native `ovx_primpath_t` prim handles
// the caller already holds (dictionary-interned, e.g. from a read group's prim list) OR a glob `pattern`;
// the public C boundary resolves prim_paths-wins precedence, so at most one arrives non-null. Explicit
// handles map straight to keys through the source (no string round trip -- the caller did the
// resolution) and need an ovstage source: on any other, the call fails and returns 0. A glob resolves
// ONCE here through the same matcher the tensor-binding views use, so a pattern query matches a legacy
// binding. Either way, a kOvxArticulation query also selects the articulation of every selected link or
// joint, as the articulation view does; other types keep only their own selected objects. An empty
// resolved set is a valid empty query, exactly like an empty ovxQuery match.
OMNI_OVX_API OvxOutputQueryHandle ovxQuerySelected(uint64_t attachHandle, uint32_t type,
                                                   const ovx_primpath_t* primPaths,
                                                   size_t primPathCount,
                                                   const ovx_string_t* pattern,
                                                   ovstage_query_result_t* outSummary = nullptr);

// The contact query (ADR-0044): a kOvxContactSensor query of the attachment `attachHandle` whose rows are the bodies
// `selections` select, in selection order then match order, each paired with `filterCount` filter objects. Resolved
// ONCE at query-open with the binding's rules: a body matched by an earlier selection is not repeated, and each filter
// selection must match one object (shared by every sensor of its selection) or exactly one per sensor (paired by
// position). Pairing by position follows that de-duplication, as in the binding: a filter of a selection that
// re-matches earlier sensors and happens to match as many objects as its remaining sensors pairs with the wrong ones
// without an error, so sensor selections should not overlap. The row rule of kOvxContactSensor still applies at each
// read, so a selected body that does not report contacts is not a row. Like a selected query it has no scope, and
// explicit handles need an ovstage source: on any other the call fails and returns 0. Returns 0 and records a runtime
// error naming the selection when a filter selection cannot be paired, when one sensor would name the same filter
// object twice, or when filterCount exceeds kOvxMaxContactFilters; 0 also means `attachHandle` names no ovstage-backed
// simulation. A selection matching nothing adds no rows: an empty contact query is valid. Optional
// outSummary follows ovxQuery: creation-time row count and the query's supported attribute tokens, including
// per-filter attributes when filterCount is nonzero; null skips the discovery work.
OMNI_OVX_API OvxOutputQueryHandle ovxQueryContacts(uint64_t attachHandle,
                                                   const OvxContactSensorSelection* selections,
                                                   size_t selectionCount,
                                                   uint32_t filterCount,
                                                   ovstage_query_result_t* outSummary = nullptr);

// The shared source path dictionary backing every prim list this query's reads
// produce (the analog of ovstage `ovstage_get_path_dictionary(instance)`). A
// by-type consumer does not know the matched prims a priori, so it uses this to
// resolve a group's `prims.list` back to paths/tokens. Returns null when there is
// no active ovstage-backed simulation. Valid for the life of the simulation.
OMNI_OVX_API ovx_path_dictionary_t* ovxQueryDictionary(OvxOutputQueryHandle query);

// Optional discovery (ovstage `fetch_query_result` analog): fills `outResult` with
// the matched prim count (`total_prim_count`) and the output attributes available
// for this query's current membership (`attributes` / `attribute_count`). The
// borrowed array remains valid until the next discovery call or query release;
// it is separate from immutable query-creation metadata. Discovery counts every
// matching prim. A DirectGPU
// rigid-body group may contain fewer rows because disabled rigid dynamics have no
// device state and are omitted; CPU groups remain inclusive. `all_handle` /
// `query_result_id` are 0 (no ovstage query handle is involved). Returns false on
// a bad handle / null arg.
OMNI_OVX_API bool ovxFetchQueryResult(OvxOutputQueryHandle query, ovstage_query_result_t* outResult);

// Apply PxArticulationReducedCoordinate::updateKinematic to the whole-articulation
// members selected by `query`. The query must have type kOvxArticulation. Membership
// is evaluated when this call runs. `flags` uses bit 0 for position and bit 1 for
// velocity, matching PxArticulationKinematicFlag. Zero flags are a successful no-op
// after query validation. Returns false for an invalid/stale/wrong-type query or if
// any selected articulation cannot be resolved. Membership and native objects belong
// to the query's exact attachment. Returns false while that owner awaits fetch or
// when called from a runtime callback, without updating any articulation.
//
// On a DirectGPU scene the refresh is ordered on the device after `sync` -- everything queued
// on `sync.stream` and `sync.wait_event` -- and everything queued on `sync.stream` afterwards
// runs after it (ADR-0052). Nothing blocks the host, so the call can be captured into a CUDA
// graph after the write it follows; a selected query must have refreshed once outside the
// capture, which uploads its articulation list. A DirectGPU scene that has not stepped is
// refused: nothing warms it up. A CPU scene refreshes on the host at once and refuses a
// capturing stream. The legacy default stream -- the legacy sentinel, or a null stream on a
// DirectGPU scene -- is refused while a blocking stream of its context is being captured. A
// `sync.stream` of another CUDA context than the scene's is drained on the host instead, and
// the refresh runs on the scene's own stream.
OMNI_OVX_API bool ovxUpdateArticulationsKinematic(OvxOutputQueryHandle query,
                                                  uint32_t flags,
                                                  ovstage_cuda_sync_t sync = ovstage_cuda_sync_t{ 0, 0 });

// Evaluate the signed distance fields of the SDF shapes `query` selects (ADR-0048), the successor of the SDF view. The
// query must have type kOvxSdfShape; its rows are its selection in order -- explicit handles as given, a pattern in the
// binding matcher's order -- or, unselected, every SDF shape in database order, each prim that is not an SDF shape
// skipped. `points` is [N, Q, 3] float32 and `out` [N, Q, 4] float32 (gradient x, y, z, then signed distance), both
// C-contiguous on the scene's CUDA device; N is the row count and Q, the points per shape, at least 1, read from the
// tensors on every call, with N at most kOvxMaxSdfShapesPerEvaluation and N x Q at most kOvxMaxSdfPointsPerEvaluation.
// Points and gradients are in each shape's mesh-local space (the unscaled frame of the mesh's vertices); no transform
// is applied. The rows must all be in one DirectGPU scene that has simulated since the attachment last created or
// retired a record: PhysX uploads shape data to the GPU only when it simulates, so step once after such a change.
// `pointsReady` is ovstage's producer handoff for `points`, as ovxCommitGroup takes it: a non-zero stream is drained
// and a non-zero event waited on, both before the points are read and with the output-cache lock not held; {0, 0}
// asserts nothing is outstanding. `out` is complete when the call returns. The query keeps its rows and device buffers
// between calls, resolving the rows again once the attachment creates or retires a record; releasing the query frees
// them. Returns false, with a runtime error naming the cause, for an invalid/stale/ wrong-type query, a query with no
// SDF shape or more than kOvxMaxSdfShapesPerEvaluation, mismatched tensors, rows outside one DirectGPU scene, a scene
// not simulated since the last record change, or a failed handoff.
OMNI_OVX_API bool ovxEvaluateSdfShapes(OvxOutputQueryHandle query,
                                       const DLTensor* points,
                                       DLTensor* out,
                                       ovstage_cuda_sync_t pointsReady = {});

// The refusals of ovxEvaluateSdfShapes that need no stepped scene and no device, run without evaluating: an invalid,
// stale or wrong-type query, malformed or mismatched tensors, and rows that are empty, over the limit, not the
// tensors' N, or not all in one DirectGPU scene. Reads neither tensor's data and waits for no producer, so a caller can
// refuse such a call before it warms the scene up. True when the evaluation could go ahead once the scene has stepped.
OMNI_OVX_API bool ovxCheckSdfShapes(OvxOutputQueryHandle query, const DLTensor* points, const DLTensor* out);

// READ (ovstage `read_attributes` analog): request named output attributes for a
// query. A non-NULL string takes precedence over the token. An unresolvable token or a duplicate
// canonical name fails the request, with its request index in the runtime error.
// The particle alias positions canonicalizes to points. Unsupported names produce
// no groups; optional metadata keeps their original slot with an invalid token
// and kOvxReadOmissionUnsupportedAttribute. Unknown strings are not interned.
// Zero attributes is a valid empty read. A known backend failure returns zero and
// publishes no groups. A successful read is independent of the query's lifetime.
// Optional metadata has one row per request in request order; groupCount counts
// groups, and omissionReasons may be nonzero alongside available groups. A known
// name with no applicable data has zero groups and no omission reason. The rows
// remain valid until read release. A null outSummary skips optional summary work.
// `sync` is the caller's producer handoff (ADR-0055): a DirectGPU device group is gathered on `sync.stream` after
// everything queued on it and after `sync.wait_event`, with no host wait; a stream of another CUDA context, and every
// other group, are waited for on the host. A read on a stream being captured records its gathers into the graph. It
// needs the plan a released read over the query with the same attributes and rows left held, and is refused before
// anything is queued without one, or when it holds a host-resident or contact group.
OMNI_OVX_API OvxReadHandle ovxReadAttributes(OvxOutputQueryHandle query,
                                             const ovx_token_or_string_t* attrs,
                                             size_t attrCount,
                                             OvxReadResult* outSummary = nullptr,
                                             ovstage_cuda_sync_t sync = ovstage_cuda_sync_t{ 0, 0 });

// ITERATE (ovstage `fetch_read_next` analog): fill the next typed column group.
// Each group carries one attribute column, columns[0] (column_count == 1), and is
// homogeneous — fixed-size (one tensor over its prims) or array (one ragged tensor
// per prim). The column, its numeric tensors and the prim and data index maps are
// borrowed and stay valid until ovxReleaseRead; the stage-derived prims.list stays valid until
// ovxReleaseGroup (see "Group lifetime" above; an intervening step invalidates
// neither lifetime). Returns kOvxReadStatusOk with
// `*outGroup` filled, kOvxReadStatusEndOfIteration when exhausted, or
// kOvxReadStatusError on a bad handle / internal failure; `*outGroup` is zeroed on
// error/end-of-iteration; Busy leaves it untouched.
//
// Backend failures already known during gathering reject read creation. Fetch
// can still report an error for a bad handle or an attachment that was detached.
// Expected omissions recorded at open do not turn end-of-iteration into failure.
//
// How many tensors a column carries depends on is_array, and reading only tensors[0] is
// wrong for half of them. A FIXED group (is_array == false: rigid-body / link pose and
// velocity, articulation joint DOFs) stacks every row into ONE tensor, so tensor_count == 1
// and row i belongs to prims.list entry i; a joint group has one row per DOF, so a joint's
// path repeats once per axis (ADR-0059). An ARRAY group (is_array == true: point-instancer
// arrays, deformable points) carries ONE TENSOR PER PRIM, so tensor_count == prims.count and
// tensors[i] is prim i's own variable-length array, and taking tensors[0] yields the first
// prim's values with no error to say the rest were dropped.
OMNI_OVX_API OvxReadStatus ovxFetchReadNext(OvxReadHandle read, ovstage_read_group_t* outGroup);

// Release a fetched group's stage-derived prims.list. Numeric tensor/index-map
// storage remains owned by the read session until ovxReleaseRead.
//
// RELEASE EACH GROUP WHILE THE STAGE IS STILL ATTACHED. A prims.list belongs to the source's path
// dictionary, whose lifetime the stage owner controls; the device columns belong to the read session
// and may be borrowed past it (see "Group lifetime"). ovxReleaseRead does destroy any list still
// outstanding, so this is not the only place they are freed -- it is the only place they are freed
// SAFELY. A borrowed column defers ovxReleaseRead, possibly past detach, and destroying a list
// through a dictionary whose Stage is gone is a use-after-free. Idempotent, and safe to call for a
// group already released.
OMNI_OVX_API bool ovxReleaseGroup(OvxReadHandle read, const ovstage_read_group_t* group);

// Release the read session, every scratch / device column it owns, and any prims.list still
// outstanding. Device columns held for the query's next read go back to it without a host wait, after the read's
// last use -- its last replay's gathers, for a captured read -- which the next read waits for on the device. When the
// query already keeps columns of that scene and kind, the read's own are freed instead, after a host wait for that
// use. Call it outside any CUDA graph capture.
// False reports rejected admission (the session remains live) or a terminal release failure
// (unsafe device buffers are quarantined).
OMNI_OVX_API bool ovxReleaseRead(OvxReadHandle read);

// Release the query. It frees the device columns the query keeps after a host wait for their last use, so call it
// outside any CUDA graph capture.
OMNI_OVX_API bool ovxReleaseQuery(OvxOutputQueryHandle query);

} // extern "C"

} // namespace omni::physx
