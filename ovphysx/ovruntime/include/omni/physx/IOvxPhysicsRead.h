// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
//
// This enum belongs to the generic output-read API. In particular, kOvxArticulation
// is not the TensorBindings ObjectType::ARTICULATION selector (numeric value 2):
// the two APIs have separate enum domains and their values must not be mixed.
enum OvxObjectType : uint32_t
{
    kOvxRigidBody         = 0, //!< dynamic rigid bodies (standalone + point-instancer instances)
    kOvxArticulationLink  = 1, //!< articulation link body transforms
    kOvxArticulationJoint = 2, //!< articulation joint state (per-axis; array group per joint)
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
};

// Selection scope.
enum OvxObjectScope : uint32_t
{
    kOvxAll = 0, //!< every object of the type. Stable across steps, but valid only until a
                 //!< STRUCTURAL change (object add/remove, instancer instance-count change),
                 //!< which requires an explicit re-query; current output metadata does not
                 //!< signal this change.
    kOvxActive = 1, //!< rigid/link/vehicle: getActiveActors(); articulation: any link reported active
                    //!< by its owning scene, falling back to articulation awake state only when that
                    //!< scene cannot report active actors. DirectGPU scenes currently disable sleeping,
                    //!< so whole-articulation ACTIVE membership there is equivalent to ALL.
                    //!< deformable: awake. particle/joint/tendon: no active notion -- behaves as kOvxAll.
                    //!< SINGLE-FRAME: the active set is recomputed every step, so a kOvxActive
                    //!< handle (and groups/lists from it) is valid only for the step it was opened
                    //!< against -- re-query each frame; do not cache across steps.
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
inline constexpr const char* kLinearAcceleration  = "linearAcceleration";  //!< vec3 f32
inline constexpr const char* kAngularAcceleration = "angularAcceleration"; //!< vec3 f32
// Frames on a deformable, where the three below are not all in the same one: `points` and
// `restPoints` are sim-mesh-local, `velocities` is world (matching the tensor binding's
// `getSimulationNodalVelocities`). So differencing `points` against `restPoints` is meaningful,
// while differencing `points` against a time-integrated `velocities` is not.
//
// Scoped to deformables: particle sets and the point-instancer array form read `points` /
// `velocities` too, and there the pair shares one frame.
inline constexpr const char* kPoints           = "points";           //!< vec3 f32 [array]
inline constexpr const char* kVelocities       = "velocities";       //!< vec3 f32 [array]
inline constexpr const char* kRestPoints       = "restPoints";       //!< vec3 f32 [array] (sim-mesh-local)
inline constexpr const char* kSimElementIndices = "simElementIndices"; //!< int32 [array] (4 per tet / 3 per tri)
inline constexpr const char* kCollisionElementIndices = "collisionElementIndices"; //!< int32 [array] (4 per tet)
inline constexpr const char* kPositions        = "positions";        //!< vec3 f32 [array] (instancer-local)
inline constexpr const char* kOrientations     = "orientations";     //!< quat f32 xyzw [array] (instancer-local)
inline constexpr const char* kAngularVelocities = "angularVelocities"; //!< vec3 f32 [array]
inline constexpr const char* kAccelerations     = "accelerations";     //!< vec3 f32 [array]
inline constexpr const char* kAngularAccelerations = "angularAccelerations"; //!< vec3 f32 [array]
// Body properties. Simulation inputs rather than outputs, so these are host-resident columns even on
// a DirectGPU scene, and they have no point-instancer array form (an instancer authors no per-instance
// mass). centerOfMass* is in the body's local frame, unlike position/orientation which are world.
inline constexpr const char* kMass             = "mass";             //!< f32
inline constexpr const char* kInverseMass      = "inverseMass";      //!< f32
inline constexpr const char* kInertia          = "inertia";          //!< mat3 f32 (9, COM frame)
inline constexpr const char* kInverseInertia   = "inverseInertia";   //!< mat3 f32 (9, COM frame)
inline constexpr const char* kCenterOfMassPosition    = "centerOfMassPosition";    //!< vec3 f32 (local)
inline constexpr const char* kCenterOfMassOrientation = "centerOfMassOrientation"; //!< quat f32 xyzw (local)
// Whole-articulation root-link state. Qualified instead of reusing the bare position / orientation /
// linearVelocity / angularVelocity above, because on this type the row's prim is whichever prim
// carries PhysicsArticulationRootAPI -- possibly an ancestor Xform of the root link. The bare names
// mean "this row's own prim's pose" on every other type (REQ-READ-ATTRS-001 AC-11 corollary).
inline constexpr const char* kRootPosition        = "rootPosition";        //!< vec3 f32 (scene world)
inline constexpr const char* kRootOrientation     = "rootOrientation";     //!< quat f32 xyzw (scene world)
inline constexpr const char* kRootLinearVelocity  = "rootLinearVelocity";  //!< vec3 f32
inline constexpr const char* kRootAngularVelocity = "rootAngularVelocity"; //!< vec3 f32
// Whole-articulation centres of mass. "world" is the same scene-world frame as that articulation's
// `rootPosition` and is independent of TensorBindings subspace roots. "local" is the root link's
// centre-of-mass (mass) frame, not its actor/prim frame.
inline constexpr const char* kCenterOfMassWorld = "centerOfMassWorld"; //!< vec3 f32 (scene world)
inline constexpr const char* kCenterOfMassLocal = "centerOfMassLocal"; //!< vec3 f32 (root-link mass frame)
// Whole-articulation inverse dynamics. Row-major flattened into dtype.lanes, whose width depends on the
// articulation's own topology (M = numDofs + (fixedBase ? 0 : 6)) rather than on the attribute --
// so a read emits one group per cohort of structurally identical articulations (ADR-0026), and
// lanes is read per group, never assumed. Four of the five recover their shape from lanes alone:
// massMatrix is square, the two force columns are [lanes], centroidalMomentum has six rows.
// jacobian is the exception -- both its dimensions vary -- so it carries kJacobianShape alongside.
// Generalized joint coordinates follow the authored USD body relationship: +1 when body0 is the
// articulation parent and -1 when body1 is the parent. With S_dof containing those signs and
// T=S_dof for a fixed base or T=diag(I6,S_dof) for a floating base, the values satisfy J=J_physx*T,
// M=T*M_physx*T, c=T*c_physx, g=T*g_physx, and [A|b]=[A_physx*T|b_physx]. The six floating-root
// coordinates and bias are unchanged. Angular generalized-coordinate dimensions use radians and receive
// no degree conversion; linear dimensions retain their units.
inline constexpr const char* kJacobian           = "jacobian";           //!< f32 [rows*cols]
inline constexpr const char* kJacobianShape      = "jacobianShape";      //!< int32 (2: rows, cols)
inline constexpr const char* kMassMatrix         = "massMatrix";         //!< f32 [M*M]
inline constexpr const char* kCoriolisForce      = "coriolisForce";      //!< f32 [M]
inline constexpr const char* kGravityForce       = "gravityForce";       //!< f32 [M]
inline constexpr const char* kCentroidalMomentum = "centroidalMomentum"; //!< f32 [6*(numDofs+7)], floating base only
inline constexpr const char* kDisableGravity    = "disableGravity";    //!< uint8 (nonzero = disabled)
inline constexpr const char* kDisableSimulation = "disableSimulation"; //!< uint8 (nonzero = disabled)
// Per-shape properties. One value per shape of the selected object, padded to the widest object in
// the read, so a row is `shapeCount`-of-`lanes` real values followed by zeros. Read `shapeCount` to
// find the end; the padding is zero-filled but 0.0 is a legal offset, so it is not a terminator.
inline constexpr const char* kStaticFriction  = "staticFriction";  //!< f32 [per shape]
inline constexpr const char* kDynamicFriction = "dynamicFriction"; //!< f32 [per shape]
inline constexpr const char* kRestitution     = "restitution";     //!< f32 [per shape]
inline constexpr const char* kContactOffset   = "contactOffset";   //!< f32 [per shape]
inline constexpr const char* kRestOffset      = "restOffset";      //!< f32 [per shape]
inline constexpr const char* kShapeCount = "shapeCount"; //!< int32 (one per body or whole articulation)
// Articulation DOF attributes. One value per enabled axis of the joint prim, so the emitted tensor
// is one array per joint (`dtype.lanes` carries the per-axis width where an attribute has one).
// On an angular axis the unit fold is per quantity, not per column: a coordinate or a rate is in
// degrees as USD authored it, a gain or a per-rate coefficient is per degree (so jointStiffness,
// jointDamping, jointViscousFriction and jointVelocityDependentResistance divide where a position
// multiplies), and an effort, inertia or enum takes no fold. One fold applied to the whole block
// below is the 57.3x-and-3283x class of bug; REQ-READ-ATTRS-001 AC-15 tabulates it per attribute.
// A signed quantity also carries the joint's body-order sign. The `joint` prefix is load-bearing:
// staticFriction/dynamicFriction also exist as per-shape rigid attributes, and tendonStiffness /
// tendonDamping below are a different quantity from jointStiffness / jointDamping above.
inline constexpr const char* kJointPosition    = "jointPosition";    //!< f32 [array, per-axis]
inline constexpr const char* kJointVelocity    = "jointVelocity";    //!< f32 [array, per-axis]
inline constexpr const char* kJointPositionTarget = "jointPositionTarget"; //!< f32 [array, per-axis]
inline constexpr const char* kJointVelocityTarget = "jointVelocityTarget"; //!< f32 [array, per-axis]
inline constexpr const char* kJointActuationForce = "jointActuationForce"; //!< f32 [array, per-axis]
//!< The link's incoming joint force resolved onto this DOF's own axis -- "projected" is onto the
//!< axis. Derived per read rather than read from a buffer.
inline constexpr const char* kJointProjectedForce = "jointProjectedForce"; //!< f32 [array, per-axis] (read-only)
// DOF properties. Simulation inputs rather than outputs, so these columns are host-resident (kDLCPU)
// even on a DirectGPU scene: there is no device copy to hand back. One read can therefore mix a
// kDLCUDA jointPosition with a kDLCPU jointStiffness; branch on `device_type`.
inline constexpr const char* kJointStiffness   = "jointStiffness";   //!< f32: drive effort per unit position error
inline constexpr const char* kJointDamping     = "jointDamping";     //!< f32: drive effort per unit rate error
inline constexpr const char* kJointLimit       = "jointLimit";       //!< f32 x2 (low, high), FLT_MAX when free
inline constexpr const char* kJointMaxVelocity = "jointMaxVelocity"; //!< f32: the joint's rate bound
inline constexpr const char* kJointMaxForce    = "jointMaxForce";    //!< f32: the drive's effort bound
inline constexpr const char* kJointArmature    = "jointArmature";    //!< f32: added rotor inertia on the axis
inline constexpr const char* kJointStaticFriction  = "jointStaticFriction";  //!< f32: break-away effort
inline constexpr const char* kJointDynamicFriction = "jointDynamicFriction"; //!< f32: sliding effort
//!< A coefficient, where the two friction columns above are efforts -- which is why this one carries
//!< an angular fold on a rotational axis and they do not.
inline constexpr const char* kJointViscousFriction = "jointViscousFriction"; //!< f32: effort per unit rate
inline constexpr const char* kJointSpeedEffortGradient = "jointSpeedEffortGradient"; //!< f32: envelope rate per effort
//!< The actuator's rate bound, from the drive envelope. Not kJointMaxVelocity above, which bounds
//!< the joint itself.
inline constexpr const char* kJointMaxActuatorVelocity = "jointMaxActuatorVelocity"; //!< f32
inline constexpr const char* kJointVelocityDependentResistance =
    "jointVelocityDependentResistance"; //!< f32: drive envelope, resisting effort per unit rate
inline constexpr const char* kJointDriveType   = "jointDriveType";   //!< uint8 [array] (0 none, 1 force, 2 accel)
// On kOvxArticulationLink: the spatial force the inbound joint exerts on the link. The leading
// `link` is the type qualifier every other qualified name here leads with, and matches PhysX's own
// spelling (PxArticulationCache::linkIncomingJointForce) -- it is a link column, not a DOF one.
inline constexpr const char* kLinkIncomingJointForce =
    "linkIncomingJointForce"; //!< f32 x6: force xyz then torque xyz, in the joint frame USD authored
// Tendon properties. Authoring-time values, so they change only when the tendon does. Named
// tendon* rather than bare stiffness/damping/offset because those words also describe joint drive
// and DOF properties, which this API will grow its own attributes for.
inline constexpr const char* kTendonStiffness      = "tendonStiffness";      //!< f32
inline constexpr const char* kTendonDamping        = "tendonDamping";        //!< f32
inline constexpr const char* kTendonLimitStiffness = "tendonLimitStiffness"; //!< f32
inline constexpr const char* kTendonLimit          = "tendonLimit";          //!< f32 x2 (low, high), fixed tendons only
inline constexpr const char* kTendonRestLength     = "tendonRestLength";     //!< f32 — fixed tendons only
inline constexpr const char* kTendonOffset         = "tendonOffset";         //!< f32

// Deformable material properties. One row per material prim, host-resident: these are authored
// simulation inputs that PhysX never writes back (REQ-TENSOR-CPU-ONLY-001).
//
// The `deformable` prefix is load-bearing, the same way `joint` is above: unqualified
// `dynamicFriction` already names a per-shape attribute served by the rigid and whole-articulation
// tables, so one f32 per prim under that token would make one name mean two shapes. `deformable`
// rather than `material` leaves the `material*` names free for a rigid-material type.
inline constexpr const char* kDeformableDynamicFriction   = "deformableDynamicFriction";   //!< f32
inline constexpr const char* kDeformableYoungsModulus     = "deformableYoungsModulus";     //!< f32
inline constexpr const char* kDeformablePoissonsRatio     = "deformablePoissonsRatio";     //!< f32
inline constexpr const char* kDeformableElasticityDamping = "deformableElasticityDamping"; //!< f32
inline constexpr const char* kDeformableBendingStiffness  = "deformableBendingStiffness";  //!< f32 — surface only
inline constexpr const char* kDeformableThickness         = "deformableThickness";         //!< f32 — surface only
inline constexpr const char* kDeformableBendingDamping    = "deformableBendingDamping";    //!< f32 — surface only

// WRITE-ONLY control inputs (ADR-0012). Named here beside the readable attributes because
// the vocabulary is one flat list, but the READ never emits them: a force is consumed by the solver
// and cleared each step, so there is no stored value to report. Asking for either through
// ovxReadAttributes yields no column.
inline constexpr const char* kForce  = "force";  //!< vec3 f32, at the centre of mass [write-only]
inline constexpr const char* kWrench = "wrench"; //!< 9 x f32: force(3), torque(3), application point(3),
                                                 //!< the point in WORLD space [write-only]

// Vehicle wheel CONTROLS (ADR-0012). Inputs the solver consumes, unlike the wheel
// transform, which it produces -- so these are writable on kOvxVehicleWheel where `position` and
// `orientation` are not.
//
// They serve vehicles WITHOUT a drive, whose control surface is per wheel. A vehicle WITH a drive is
// accelerated and steered as a whole and its commands live on the vehicle prim, which needs a
// per-vehicle selector this enum does not have.
//
// Named here rather than in the write's own header because the vocabulary is one flat list, and
// because these PERSIST -- they are current control state until overwritten, not consumed and
// cleared like force -- so a read of them is meaningful and may well want the same names.
inline constexpr const char* kDriveTorque = "driveTorque"; //!< f32, per wheel
inline constexpr const char* kBrakeTorque = "brakeTorque"; //!< f32, per wheel
inline constexpr const char* kSteerAngle  = "steerAngle";  //!< f32 radians, per wheel
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
//
// STEP-FIRST PRECONDITION (DirectGPU scenes). PhysX rejects direct-GPU reads until the
// first step has run, so a read issued before it emits NO groups for the types sourced
// from it. That is every covered type, not a closed list: rigid bodies, articulation links,
// whole articulations and articulation joint DOFs, and equally point instancers, vehicle
// wheels, deformables and particle sets -- even though
// ovxFetchQueryResult still reports the matching prims. Not an error: step once, then
// read. Readiness is evaluated per scene: in a multi-scene query, ready scene partitions may emit
// groups while an unready DirectGPU scene's partition is omitted. CPU scenes do report authored
// initial state pre-step (ADR-0008 Decision 10).

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
// until ovxReleaseQuery). Discovery counts every matching prim. A DirectGPU
// rigid-body group may contain fewer rows because disabled rigid dynamics have no
// device state and are omitted; CPU groups remain inclusive. `all_handle` /
// `query_result_id` are 0 (no ovstage query handle is involved). Returns false on
// a bad handle / null arg.
OMNI_OVX_API bool ovxFetchQueryResult(OvxOutputQueryHandle query, ovstage_query_result_t* outResult);

// READ (ovstage `read_attributes` analog): request named output attributes for a
// query. Names unknown for the queried type are skipped (no group emitted for
// them). Returns 0 on a bad query handle / null args.
OMNI_OVX_API OvxReadHandle ovxReadAttributes(OvxOutputQueryHandle query,
                                             const ovx_string_or_token_t* attrs,
                                             size_t attrCount);

// ITERATE (ovstage `fetch_read_next` analog): fill the next typed column group.
// Each group is homogeneous — fixed-size (one tensor over its prims) or array
// (one ragged tensor per prim). The numeric tensors / prim and data index maps are borrowed and
// stay valid until ovxReleaseRead; the stage-derived prims.list stays valid until
// ovxReleaseGroup (see "Group lifetime" above; an intervening step invalidates
// neither lifetime). Returns kOvxReadStatusOk with
// `*outGroup` filled, kOvxReadStatusEndOfIteration when exhausted, or
// kOvxReadStatusError on a bad handle / internal failure; `*outGroup` is zeroed on
// any non-Ok status.
//
// The two terminal statuses differ in whether the set was COMPLETE. A backend build
// or gather that fails omits its columns and ends the drain with kOvxReadStatusError
// instead of kOvxReadStatusEndOfIteration, so a short set cannot be mistaken for the
// whole one. Groups already fetched remain valid and must still be released; the
// error reports that more were expected, it does not retract what was delivered.
//
// Three things are NOT failures, and all still end the drain with
// kOvxReadStatusEndOfIteration: an attribute this object type does not produce, the
// step-first or pending-scene-insertion unavailable state, and a device-only type
// (deformables, particles) on a sim with no CUDA context. The last is structural rather
// than transient -- that type cannot produce a column on that sim at all -- so it is the
// same answer as an inapplicable attribute, not something a caller could retry.
//
// How many tensors a group carries depends on is_array, and reading only tensors[0] is
// wrong for half of them. A FIXED group (is_array == false: rigid-body / link pose and
// velocity) stacks every prim into ONE tensor, so tensor_count == 1 and row i belongs to
// prim i. An ARRAY group (is_array == true: joint state, point-instancer arrays,
// deformable points) carries ONE TENSOR PER PRIM, so tensor_count == prims.count and
// tensors[i] is prim i's own variable-length array -- a joint read over 16k joints is ONE
// group with 16k tensors, and taking tensors[0] yields the first joint's values with no
// error to say the rest were dropped.
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
OMNI_OVX_API void ovxReleaseGroup(OvxReadHandle read, const ovstage_read_group_t* group);

// Release the read session, every scratch / device column it owns, and any prims.list still
// outstanding.
OMNI_OVX_API void ovxReleaseRead(OvxReadHandle read);

// Release the query.
OMNI_OVX_API void ovxReleaseQuery(OvxOutputQueryHandle query);

} // extern "C"

} // namespace omni::physx
