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
//!< Volume-deformable sim-node kinematic target: vec4 f32 [array], one PxVec4 per sim-mesh node
//!< (xyz target + w active-flag; w==0 drives the node, w!=0 leaves it free). Volume only.
inline constexpr const char* kKinematicTarget  = "kinematicTarget";
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
// Host-resident sleep state for standalone rigid dynamics and articulation links. A link reports
// its whole articulation's state. Scenes with sleeping disabled report 0; ACTIVE remains a selection
// scope, not a sleep-state query. This has no point-instancer array spelling.
inline constexpr const char* kIsSleeping = "isSleeping"; //!< uint8: 1 = sleeping, 0 = awake [read-only]
//!< Borrowed PhysX SDK address. Host uint64. The column snapshots the address; it does not own
//!< the object. See ADR-0044.
inline constexpr const char* kPhysXPointer = "physxPointer"; //!< uint64: borrowed address [read-only]
// Kinematic target pose, split into a POINT + QUATERNION pair like the live pose. Only a kinematic
// PxRigidDynamic is driven; the read falls back to the current pose for everything else.
inline constexpr const char* kKinematicTargetPosition    = "kinematicTargetPosition";    //!< vec3 f32 (world)
inline constexpr const char* kKinematicTargetOrientation = "kinematicTargetOrientation"; //!< quat f32 xyzw (world)
// Per-shape properties. One value per shape of the selected object, padded to the widest object in
// the read, so a row is `shapeCount`-of-`lanes` real values followed by zeros. Read `shapeCount` to
// find the end; the padding is zero-filled but 0.0 is a legal offset, so it is not a terminator.
inline constexpr const char* kStaticFriction  = "staticFriction";  //!< f32 [per shape]
inline constexpr const char* kDynamicFriction = "dynamicFriction"; //!< f32 [per shape]
inline constexpr const char* kRestitution     = "restitution";     //!< f32 [per shape]
inline constexpr const char* kContactOffset   = "contactOffset";   //!< f32 [per shape]
inline constexpr const char* kRestOffset      = "restOffset";      //!< f32 [per shape]
inline constexpr const char* kShapeCount = "shapeCount"; //!< int32 (one per body or whole articulation)
// Whole-articulation topology metadata: one int32 per articulation. dofCount, linkCount and the two
// tendon counts come from the per-articulation entry; jointCount comes from the interned metatype.
// isFixedBase is read LIVE from the SDK articulation flag, not the metatype, so a between-step
// setArticulationFlag(eFIX_BASE, ...) toggle is reflected rather than reading a value captured at view
// build. Host-resident on every device like shapeCount. These replace the deprecated
// ovphysx_articulation_metadata_t field-for-field.
// `linkCount` is the count of articulation LINKS (the old struct's mislabelled `body_count`) -- not
// standalone rigid bodies. Served as one scene-wide group even when base type varies per row, unlike the
// legacy homogeneous-view getter.
inline constexpr const char* kDofCount           = "dofCount";           //!< int32: DOFs
inline constexpr const char* kLinkCount          = "linkCount";          //!< int32: articulation links
inline constexpr const char* kJointCount         = "jointCount";         //!< int32: joints
inline constexpr const char* kFixedTendonCount   = "fixedTendonCount";   //!< int32: fixed tendons
inline constexpr const char* kSpatialTendonCount = "spatialTendonCount"; //!< int32: spatial tendons
inline constexpr const char* kIsFixedBase        = "isFixedBase";        //!< int32: 1 = fixed base, 0 = floating
// Whole-articulation DOF-column descriptor: which joint prim and which of its axes each inverse-dynamics
// matrix column belongs to. Emitted like the topology metadata above -- one scene-wide group, one row per
// articulation, host-resident -- but the row is a per-DOF array padded to the widest selected
// articulation's `dofCount`, which is the real length (the pad is trailing, like shapeCount terminates a
// per-shape row). The array runs in PhysX flattened cache-DOF order, i.e. jacobian / massMatrix column
// `(isFixedBase ? 0 : 6) + k` maps to DOF `k` here -- so a consumer holding a cohort's `jacobian` recovers
// `column -> (joint prim, axis)` with no side channel. `dofJointPath` is the joint's interned prim-path
// handle, resolved through `ovxQueryDictionary` exactly as `prims.list` is (0 / OVX_INVALID_PRIMPATH pads);
// prim-path identity replaces the deprecated tensor-binding DOF/body/joint name strings. `dofAxisIndex` is
// the DOF's ordinal among that joint's UNLOCKED axes (in PhysX TWIST/SWING1/SWING2 order): 0 for a 1-DOF
// joint, 0/1/2 for a fully free spherical joint -- a locked leading axis compacts it (locked TWIST gives
// SWING1=0, SWING2=1, not 1/2), not a canonical axis id.
inline constexpr const char* kDofJointPath = "dofJointPath"; //!< uint64 [array, per DOF]: owning joint prim handle
inline constexpr const char* kDofAxisIndex = "dofAxisIndex"; //!< int32 [array, per DOF]: axis within that joint
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

// Contact-sensor aggregates. One row per contact-reporting body, one component each: the net NORMAL
// force (the sensor's contact-point impulses summed) and the net FRICTION force (its friction-anchor
// impulses summed) PhysX reports on the sensor this step, each divided by the step dt and matching the
// tensor binding's getNetNormalContactForces / getNetFrictionContactForces. Their sum is the total contact
// force. Friction anchor forces that oppose each other cancel in the net vector, so a friction torque
// does not show in it. Residency follows the scene's execution, like every read column: host (kDLCPU) on
// a non-DirectGPU scene, device (kDLCUDA, reduced on-device) on a DirectGPU scene -- branch on the
// column's device.device_type before touching the data.
inline constexpr const char* kNetNormalContactForce   = "netNormalContactForce";   //!< vec3 f32 (per contact sensor)
inline constexpr const char* kNetFrictionContactForce = "netFrictionContactForce"; //!< vec3 f32 (per contact sensor)
// Contact-sensor force matrices, served on a contact query with filters (ovxQueryContacts, ADR-0044): one
// row per sensor, its F filters' forces as F x 3 floats folded into the lanes (ADR-0026 lanes-fold), filter f
// at lanes 3f..3f+2. Each cell is the force the sensor receives from that filter's body -- normal from the
// pair's contact points, friction from its anchors -- matching getNormalContactForceMatrix /
// getFrictionContactForceMatrix. Not served on a query without filters. A filter that is a collider rather
// than a body is matched through its collider id on a host scene but reads zero on a DirectGPU scene (the
// device reducer resolves body filters only), as in the binding; likewise for the filtered detail below.
inline constexpr const char* kNormalContactForceMatrix   = "normalContactForceMatrix";   //!< F x vec3 f32 (per sensor)
inline constexpr const char* kFrictionContactForceMatrix = "frictionContactForceMatrix"; //!< F x vec3 f32 (per sensor)
// Contact-sensor filtered detail, served on a contact query with filters: the contact points (normal
// family) and friction anchors (friction family) of each pair between a sensor and one of its filters'
// bodies, matching getNormalContactData / getFrictionContactData. Each element attribute is an ARRAY group,
// one tensor per sensor holding its elements ordered by filter column (host scenes: report order within a
// column; DirectGPU scenes: unspecified order within a column), sized to this step's elements -- no capacity,
// no truncation. A sensor with no element this step is absent from these groups. The count companions are
// fixed groups over every sensor row, F int32 lanes: row r's lane f is the number of its elements from
// filter f, so its tensor splits into per-filter runs. Forces are the element impulse divided by the step
// dt, negated where the sensor is the pair's second body; normals are as reported (not flipped).
inline constexpr const char* kFilteredContactForce      = "filteredContactForce";      //!< f32 (array per sensor)
inline constexpr const char* kFilteredContactPoint      = "filteredContactPoint";      //!< vec3 f32 (array per sensor)
inline constexpr const char* kFilteredContactNormal     = "filteredContactNormal";     //!< vec3 f32 (array per sensor)
inline constexpr const char* kFilteredContactSeparation = "filteredContactSeparation"; //!< f32 (array per sensor)
inline constexpr const char* kFilteredContactCount      = "filteredContactCount";      //!< F x i32 (per sensor)
inline constexpr const char* kFilteredFrictionForce     = "filteredFrictionForce";     //!< vec3 f32 (array per sensor)
inline constexpr const char* kFilteredFrictionPoint     = "filteredFrictionPoint";     //!< vec3 f32 (array per sensor)
inline constexpr const char* kFilteredFrictionCount     = "filteredFrictionCount";     //!< F x i32 (per sensor)
// Contact-sensor raw contacts, served on every contact-sensor query form: every contact point of every pair a
// sensor is in, whatever the other body, matching getRawContactData. Each is an ARRAY group, one tensor per
// sensor holding its points (host scenes: report order; DirectGPU scenes: unspecified order), sized to this
// step's points; a sensor with no point this step is absent. The force is the point's impulse divided by the
// step dt, negated where the sensor is the pair's second body; the normal is as reported (not flipped).
// rawContactOtherBody is the other body's prim, as the path dictionary's interned prim-path handle
// (ovx_primpath_t; 0 when the body has none).
inline constexpr const char* kRawContactForce      = "rawContactForce";      //!< f32 (array per sensor)
inline constexpr const char* kRawContactPoint      = "rawContactPoint";      //!< vec3 f32 (array per sensor)
inline constexpr const char* kRawContactNormal     = "rawContactNormal";     //!< vec3 f32 (array per sensor)
inline constexpr const char* kRawContactSeparation = "rawContactSeparation"; //!< f32 (array per sensor)
inline constexpr const char* kRawContactOtherBody  = "rawContactOtherBody";  //!< u64 prim handle (array per sensor)
// Contact-sensor events, served on every contact-sensor query form over a non-DirectGPU scene (PhysX's GPU
// contact pairs carry no event type, so a DirectGPU scene returns no event group): each contact-report header
// a sensor is a side of -- found, persisting or lost -- as seen from that sensor, in report order. Every name
// is an ARRAY group at one of three levels: one element per event (the contactEvent* names without
// Point/Anchor, and the two counts contactEventPointCount and contactEventAnchorCount), one per contact point
// (the other contactEventPoint* names) or one per friction anchor (the other contactEventAnchor* names), points
// and anchors concatenated in event order and split by those two counts. A sensor has a tensor when it has at
// least one element at the name's level this step, so a sensor whose only event is a lost pair, which carries
// no point, has event tensors but no point or anchor tensor. A pair between two sensors appears under each.
// Identities (colliders, bodies, materials) are interned prim-path handles (0 when none), ordered own then
// other; values are as reported -- impulses, not forces, and never negated. The type is ContactEventType's
// value (0 found, 1 lost, 2 persist). The ProtoIndex names hold the collider's instance index in its point
// instancer -- its slot in protoIndices, not the prototype that slot selects -- and 0xFFFFFFFF when the
// collider is not an instance.
inline constexpr const char* kContactEventType                = "contactEventType";                //!< i32
inline constexpr const char* kContactEventSensorSide          = "contactEventSensorSide";          //!< 0 or 1 (i32)
inline constexpr const char* kContactEventCollider            = "contactEventCollider";            //!< u64 prim handle
inline constexpr const char* kContactEventOtherBody           = "contactEventOtherBody";           //!< u64 prim handle
inline constexpr const char* kContactEventOtherCollider       = "contactEventOtherCollider";       //!< u64 prim handle
inline constexpr const char* kContactEventProtoIndex          = "contactEventProtoIndex";          //!< u32
inline constexpr const char* kContactEventOtherProtoIndex     = "contactEventOtherProtoIndex";     //!< u32
inline constexpr const char* kContactEventPointCount          = "contactEventPointCount";          //!< i32
inline constexpr const char* kContactEventAnchorCount         = "contactEventAnchorCount";         //!< i32
inline constexpr const char* kContactEventPointPosition       = "contactEventPointPosition";       //!< vec3 f32
inline constexpr const char* kContactEventPointNormal         = "contactEventPointNormal";         //!< vec3 f32
inline constexpr const char* kContactEventPointImpulse        = "contactEventPointImpulse";        //!< vec3 f32
inline constexpr const char* kContactEventPointSeparation     = "contactEventPointSeparation";     //!< f32
inline constexpr const char* kContactEventPointFaceIndex      = "contactEventPointFaceIndex";      //!< u32
inline constexpr const char* kContactEventPointOtherFaceIndex = "contactEventPointOtherFaceIndex"; //!< u32
inline constexpr const char* kContactEventPointMaterial       = "contactEventPointMaterial";       //!< u64 prim handle
inline constexpr const char* kContactEventPointOtherMaterial  = "contactEventPointOtherMaterial";  //!< u64 prim handle
inline constexpr const char* kContactEventAnchorPosition      = "contactEventAnchorPosition";      //!< vec3 f32
inline constexpr const char* kContactEventAnchorImpulse       = "contactEventAnchorImpulse";       //!< vec3 f32

// WRITE-ONLY control inputs (ADR-0012). Named here beside the readable attributes because
// the vocabulary is one flat list, but the READ never emits them: a force is consumed by the solver
// and cleared each step, so there is no stored value to report. Asking for either through
// ovxReadAttributes yields no column.
inline constexpr const char* kForce  = "force";  //!< vec3 f32, at the centre of mass [write-only]
inline constexpr const char* kWrench = "wrench"; //!< 9 x f32: force(3), torque(3), application point(3),
                                                 //!< the point in WORLD space [write-only]
// wake is a one-time command: nonzero wakes the addressed rigid dynamics, zero sleeps them. Later
// simulation or mutations can change that state. The command remains write-only; read isSleeping
// for the current state rather than the last command.
inline constexpr const char* kWake = "wake"; //!< uint8, nonzero = wake / zero = sleep [write-only]

// Persistent raw-wheel controls, writable only on vehicles without a drive.
inline constexpr const char* kDriveTorque = "driveTorque"; //!< f32, per wheel
inline constexpr const char* kBrakeTorque = "brakeTorque"; //!< f32, per wheel
inline constexpr const char* kSteerAngle = "steerAngle"; //!< commanded radians, per wheel

// Whole-vehicle controls and drive state. All columns are CPU-resident.
inline constexpr const char* kAccelerator = "accelerator";
inline constexpr const char* kBrake0 = "brake0";
inline constexpr const char* kBrake1 = "brake1";
inline constexpr const char* kSteer = "steer";
inline constexpr const char* kRequestedGear = "requestedGear";
inline constexpr const char* kThrust0 = "thrust0";
inline constexpr const char* kThrust1 = "thrust1";
inline constexpr const char* kResetToRestState = "resetToRestState";
inline constexpr const char* kClutch = "clutch";
inline constexpr const char* kCurrentGear = "currentGear";
inline constexpr const char* kTargetGear = "targetGear";
inline constexpr const char* kEngineRotationSpeed = "engineRotationSpeed";
inline constexpr const char* kAutomaticTransmission = "automaticTransmission";
inline constexpr const char* kGearSwitchTime = "gearSwitchTime";
inline constexpr const char* kAutoboxTimeSinceLastShift = "autoboxTimeSinceLastShift";

// Wheel telemetry. wheelSteerAngle is measured; steerAngle above is an input.
inline constexpr const char* kRotationSpeed = "rotationSpeed";
inline constexpr const char* kRotationAngle = "rotationAngle";
inline constexpr const char* kWheelSteerAngle = "wheelSteerAngle";
inline constexpr const char* kSuspensionJounce = "suspensionJounce";
inline constexpr const char* kSuspensionForce = "suspensionForce";
inline constexpr const char* kTireFriction = "tireFriction";
inline constexpr const char* kTireLongitudinalDirection = "tireLongitudinalDirection";
inline constexpr const char* kTireLateralDirection = "tireLateralDirection";
inline constexpr const char* kTireLongitudinalSlip = "tireLongitudinalSlip";
inline constexpr const char* kTireLateralSlip = "tireLateralSlip";
inline constexpr const char* kTireForce = "tireForce";
inline constexpr const char* kIsOnGround = "isOnGround";
inline constexpr const char* kGroundPlane = "groundPlane";
inline constexpr const char* kGroundHitPosition = "groundHitPosition";
inline constexpr const char* kGroundActorPath = "groundActorPath";
inline constexpr const char* kGroundShapePath = "groundShapePath";
inline constexpr const char* kGroundMaterialPath = "groundMaterialPath";

// Character-controller movement. A commit calls PxController::move once with this world-space
// displacement and the owning scene's timestep. It is not replayed on later steps. The controller
// prim's pose is the rigid-body position and orientation; this type has no readable columns.
// Collision flags from move() are not published.
inline constexpr const char* kMove = "move"; //!< vec3 f32, world-space displacement [write]
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

// 0 unclassified, 1 writable, 4 read-only. Matches the public writability enum.
OMNI_OVX_API int ovxDebugRenderWritability(const char* name, size_t length);

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
OMNI_OVX_API bool ovxUpdateArticulationsKinematic(OvxOutputQueryHandle query, uint32_t flags);

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
// query. Token takes precedence over string. An unresolvable token or a duplicate
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
OMNI_OVX_API OvxReadHandle ovxReadAttributes(OvxOutputQueryHandle query,
                                             const ovx_string_or_token_t* attrs,
                                             size_t attrCount,
                                             OvxReadResult* outSummary = nullptr);

// ITERATE (ovstage `fetch_read_next` analog): fill the next typed column group.
// Each group is homogeneous — fixed-size (one tensor over its prims) or array
// (one ragged tensor per prim). The numeric tensors / prim and data index maps are borrowed and
// stay valid until ovxReleaseRead; the stage-derived prims.list stays valid until
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
OMNI_OVX_API bool ovxReleaseGroup(OvxReadHandle read, const ovstage_read_group_t* group);

// Release the read session, every scratch / device column it owns, and any prims.list still
// outstanding.
// False reports rejected admission (the session remains live) or a terminal release failure
// (unsafe device buffers are quarantined).
OMNI_OVX_API bool ovxReleaseRead(OvxReadHandle read);

// Release the query.
OMNI_OVX_API bool ovxReleaseQuery(OvxOutputQueryHandle query);

} // extern "C"

} // namespace omni::physx
